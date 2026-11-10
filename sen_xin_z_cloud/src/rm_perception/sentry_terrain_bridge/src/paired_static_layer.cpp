// Mapping-only paired raw-map / memory-mask handoff. StaticLayer owns all
// occupancy interpretation, resizing and master-grid writes.
#include "static_correction_contract.hpp"
#include <chrono>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <nav2_costmap_2d/static_layer.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace sentry_terrain_bridge {
class PairedStaticLayer : public nav2_costmap_2d::StaticLayer {
  using Ptr = Grid::ConstSharedPtr;
  using Steady = std::chrono::steady_clock;
  struct Sample { Ptr grid; Steady::time_point received; };
public:
  void onInitialize() override {
    global_frame_ = layered_costmap_->getGlobalFrameID();
    getParameters();  // Reuse upstream parameters, cost conversion and lifecycle.
    declareParameter("allow_verified_evidence", rclcpp::ParameterValue(false));
    declareParameter("evidence_timeout", rclcpp::ParameterValue(1.0));
    auto node = node_.lock();
    if (!node) { throw std::runtime_error("PairedStaticLayer node expired"); }
    // Humble StaticLayer has no footprint_clearing_enabled_ member.
    // Read the same named parameter on both old and new Nav2 releases.
    if (!node->has_parameter(name_+".footprint_clearing_enabled")) {
      declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(false));
    }
    bool footprint_clearing_enabled = false;
    node->get_parameter(name_+".footprint_clearing_enabled", footprint_clearing_enabled);
    bool allowed = false;
    node->get_parameter(name_+".allow_verified_evidence", allowed);
    node->get_parameter(name_+".evidence_timeout", timeout_);
    if (!allowed || map_topic_!="/map" || subscribe_to_updates_ ||
        footprint_clearing_enabled || use_maximum_ ||
        layered_costmap_->isRolling() || global_frame_!="map" ||
        !std::isfinite(timeout_) || timeout_<=0 || timeout_>5) {
      throw std::invalid_argument("PairedStaticLayer requires mapping opt-in and unchanged map semantics");
    }
    rclcpp::SubscriptionOptions options; options.callback_group=callback_group_;
    map_sub_=node->create_subscription<Grid>("/map",rclcpp::QoS(1).reliable().transient_local(),
      [this](Ptr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!validGrid(*msg,false)) { revoke(); return; }
        // Do not project an old mask onto a new canvas. Keep the previous
        // complete snapshot only within its original proof lifetime while the
        // next complete pair is pending, including non-integral SLAM origins.
        // Explicit map-load epoch changes revoke immediately.
        if (raw_ && raw_->info.map_load_time!=msg->info.map_load_time) { revoke(); }
        raw_=msg;
      },options);
    source_sub_=node->create_subscription<Grid>("/passage/static_correction_source",10,
      [this](Ptr msg) { cache(sources_,msg,false); },options);
    mask_sub_=node->create_subscription<Grid>("/passage/verified_static_clearing",10,
      [this](Ptr msg) { cache(masks_,msg,true); },options);
  }

  void updateBounds(double x,double y,double yaw,double *minx,double *miny,
                    double *maxx,double *maxy) override {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto now=clock_->now();
      if (last_now_ && now.nanoseconds()<last_now_) { revoke(); high_water_=0; }
      last_now_=now.nanoseconds();
      if (!validOrder()) { revoke(); }
      Ptr paired_source, paired_mask;
      Steady::time_point received;
      if (raw_ && validOrder()) {
        // Never let a late pair roll the canvas or occupancy back to an older
        // /map. Both contents and association metadata must match latest raw.
        for (const auto &s:sources_) {
          const int64_t stamp=rclcpp::Time(s.grid->header.stamp).nanoseconds();
          if (stamp<=high_water_ || !fresh(s,now) ||
              !sameMap(*s.grid,*raw_)) { continue; }
          for (const auto &m:masks_) {
            if (m.grid->header.stamp==s.grid->header.stamp && fresh(m,now) &&
                sameGeometry(*s.grid,*m.grid) &&
                s.grid->info.map_load_time==m.grid->info.map_load_time &&
                (!paired_source || stamp>rclcpp::Time(paired_source->header.stamp).nanoseconds())) {
              paired_source=s.grid; paired_mask=m.grid;
              received=std::min(s.received,m.received);
            }
          }
        }
      }
      if (paired_source) {
        auto complete=std::make_shared<Grid>(*paired_source);
        for (size_t i=0;i<complete->data.size();++i) {
          if (clearsStatic(complete->data[i],paired_mask->data[i])) { complete->data[i]=0; }
        }
        high_water_=rclcpp::Time(paired_source->header.stamp).nanoseconds();
        active_stamp_=high_water_; active_received_=received; proof_active_=true;
        apply(complete);
        sources_.clear(); masks_.clear();
      }
      const double age=(now-rclcpp::Time(active_stamp_,now.get_clock_type())).seconds();
      if (proof_active_ && (age<-.1 || age>timeout_ ||
          std::chrono::duration<double>(Steady::now()-active_received_).count()>timeout_)) {
        proof_active_=false;
      }
      if (!proof_active_ && raw_ && applied_.get()!=raw_.get()) { apply(raw_); }
    }
    nav2_costmap_2d::StaticLayer::updateBounds(x,y,yaw,minx,miny,maxx,maxy);
  }

  void reset() override {
    { std::lock_guard<std::mutex> lock(state_mutex_); revoke(); }
    nav2_costmap_2d::StaticLayer::reset();
  }
  void deactivate() override {
    { std::lock_guard<std::mutex> lock(state_mutex_); revoke(); }
    nav2_costmap_2d::StaticLayer::deactivate();
  }
private:
  void apply(Ptr grid) {
    // Called exclusively from costmap update, never from a DDS callback.
    // Resizing and filling the corrected snapshot happen in one update phase.
    processMap(*grid); map_received_=true; applied_=grid;
  }
  void revoke() {
    proof_active_=false; sources_.clear(); masks_.clear();
    // Keep high_water: replaying a previously consumed pair cannot renew proof.
  }
  void cache(std::deque<Sample> &queue,Ptr msg,bool mask) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!validGrid(*msg,mask)) { revoke(); return; }
    queue.push_back({msg,Steady::now()});
    while (queue.size()>8) { queue.pop_front(); }
  }
  bool fresh(const Sample &sample,const rclcpp::Time &now) const {
    const double age=(now-rclcpp::Time(sample.grid->header.stamp)).seconds();
    return age>=-.1 && age<=timeout_ &&
      std::chrono::duration<double>(Steady::now()-sample.received).count()<=timeout_;
  }
  static bool sameMap(const Grid &a,const Grid &b) {
    return sameGeometry(a,b) && a.info.map_load_time==b.info.map_load_time && a.data==b.data;
  }
  bool validOrder() const {
    const auto &plugins=*layered_costmap_->getPlugins();
    auto node=node_.lock(); std::string type;
    if (node) { node->get_parameter("terrain_layer.plugin",type); }
    return layered_costmap_->getFilters()->empty() && plugins.size()==3 &&
      plugins[0].get()==this && plugins[1]->getName()=="terrain_layer" && plugins[1]->isEnabled() &&
      type=="sentry_terrain_bridge::TerrainLayer" &&
      dynamic_cast<nav2_costmap_2d::InflationLayer *>(plugins[2].get()) && plugins[2]->isEnabled();
  }
  std::mutex state_mutex_;
  Ptr raw_,applied_;
  std::deque<Sample> sources_,masks_;
  rclcpp::Subscription<Grid>::SharedPtr source_sub_,mask_sub_;
  double timeout_{1.0};
  int64_t high_water_{0},active_stamp_{0},last_now_{0};
  Steady::time_point active_received_{};
  bool proof_active_{false};
};
}
PLUGINLIB_EXPORT_CLASS(sentry_terrain_bridge::PairedStaticLayer,nav2_costmap_2d::Layer)
