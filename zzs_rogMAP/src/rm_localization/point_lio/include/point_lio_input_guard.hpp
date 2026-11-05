#pragma once
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>

namespace point_lio {
// Only continuous, finite prediction can recover inside the existing odom frame.
class InputGuard {
 public:
  double max_imu_gap = .10, max_lidar_gap = .50, max_scan_duration = .25;
  double input_timeout = .30, max_clock_jump = .10, startup_timeout = 30.;
  double max_unconstrained_time = .30;
  void validate() const {
    for (double v : {max_imu_gap,max_lidar_gap,max_scan_duration,input_timeout,
                     max_clock_jump,startup_timeout,max_unconstrained_time})
      if (!std::isfinite(v) || v <= 0.) throw std::invalid_argument("Invalid Point-LIO timing limit");
  }
  bool fail(const std::string &why) {
    if (!failed()) reason_ = why;
    if (established_) {if (!failed_ && !degraded_) ++generation_;failed_ = true;}
    else restart_ = true;
    return false;
  }
  bool reject(const std::string &why) {
    if (!failed()) {if (established_ && !degraded_) ++generation_;reason_ = why; degraded_ = established_;}
    return false;
  }
  bool failed() const { return failed_; }
  bool healthy() const { return established_ && !failed_ && !degraded_; }
  const std::string &reason() const { return reason_; }
  const char *state() const {
    return failed_ ? "NEEDS_RESET" : !established_ ? "INITIALIZING" :
           degraded_ ? "DEGRADED" : "TRACKING";
  }
  struct TimestampGap {
    std::string sensor;
    double previous = 0., current = 0., limit = 0.;
  };
  const TimestampGap &timestamp_gap() const {return timestamp_gap_;}
  uint64_t generation() const {return generation_;}
  double last_valid_stamp() const {return last_good_stamp_;}
  bool take_startup_restart() {
    if (!restart_) return false;
    restart_=false; degraded_=false;timestamp_gap_={};
    last_imu_=last_lidar_=imu_arrival_=lidar_arrival_=-1.;
    clock_steady_=clock_ros_=start_=-1.;
    return true;
  }
  bool imu(double stamp,double arrival) {
    return observe(stamp,arrival,last_imu_,imu_arrival_,max_imu_gap,"IMU");
  }
  bool lidar(double stamp,double arrival) {
    return observe(stamp,arrival,last_lidar_,lidar_arrival_,max_lidar_gap,"LiDAR");
  }
  bool scan(double duration) {
    if (failed() || restart_) return false;
    return (std::isfinite(duration) && duration>=0. && duration<=max_scan_duration) ||
           reject("discarded invalid LiDAR frame time span");
  }
  void prediction(double dt) {
    if (failed() || !std::isfinite(dt) || dt<-.001 || dt>max_imu_gap) {
      fail("invalid prediction dt="+std::to_string(dt));
      throw std::runtime_error(reason_);
    }
  }
  bool frame(bool accepted,double stamp,double steady) {
    if (failed() || restart_) return false;
    if (established_ && stamp-last_good_stamp_>max_unconstrained_time+1e-6)
      return fail("LiDAR constraint age exceeded recovery window");
    if (!accepted) return reject("no accepted LiDAR update; map/output withheld");
    established_=true;degraded_=false;reason_="valid LiDAR update";
    last_good_stamp_=stamp;last_good_arrival_=steady;
    return true;
  }
  bool poll(double steady,double ros,bool imu_enabled,bool wall) {
    if (failed() || restart_) return false;
    if (start_<0.) start_=steady;
    if (wall && clock_steady_>=0. &&
        std::abs((ros-clock_ros_)-(steady-clock_steady_))>max_clock_jump)
      return fail("system/ROS clock discontinuity");
    clock_steady_=steady;clock_ros_=ros;
    // Callback processing time does not establish a sensor timestamp gap.
    // Keep motion blocked until a fresh constraint; observe() still latches real gaps.
    if (wall && imu_enabled && imu_arrival_>=0. && steady-imu_arrival_>input_timeout) {
      if (!established_) return fail("startup input interrupted; recollecting");
      reject("IMU arrival timeout");
    }
    if (wall && lidar_arrival_>=0. && steady-lidar_arrival_>input_timeout) {
      if (!established_) return fail("startup input interrupted; recollecting");
      reject("LiDAR arrival timeout");
    }
    // Wall-time processing backlog is not a missing sensor-time interval.
    // Consumers also check last_valid_stamp; do not resume motion on old poses.
    if (wall && established_ && steady-last_good_arrival_>max_unconstrained_time)
      reject("waiting for a fresh processed LiDAR constraint");
    if (!established_ && wall && steady-start_>startup_timeout)
      reason_="waiting for synchronized startup input";
    return !failed() && !restart_;
  }
 private:
  bool observe(double stamp,double arrival,double &last,double &seen,double gap,const char *name) {
    if (failed() || restart_) return false;
    if (!std::isfinite(stamp) || stamp<=0.) return fail(std::string(name)+" invalid timestamp");
    if (last>=0.) {
      if (stamp<last) return fail(std::string(name)+" timestamp went backwards");
      if (stamp==last) return false;
      if (stamp-last>gap) {
        timestamp_gap_={name,last,stamp,gap};
        return fail(std::string(name)+" timestamp gap");
      }
    }
    last=stamp;seen=arrival;return true;
  }
  TimestampGap timestamp_gap_;
  uint64_t generation_=0;
  bool established_=false,failed_=false,degraded_=false,restart_=false;
  double last_imu_=-1.,last_lidar_=-1.,imu_arrival_=-1.,lidar_arrival_=-1.;
  double start_=-1.,clock_steady_=-1.,clock_ros_=-1.;
  double last_good_stamp_=0.,last_good_arrival_=-1.;
  std::string reason_="waiting for initialization";
};
}  // namespace point_lio
