#include <gtest/gtest.h>
#include "common_lib.h"
using Filter=esekfom::esekf<state_input,24,input_ikfom>;
static Filter *tested=nullptr;
static void brokenModel(state_input &x,Eigen::Matrix3d,Eigen::Matrix3d,
                        esekfom::dyn_share_modified<double> &obs) {
  // Inject partial changes before rejecting the observation. Both must roll back.
  x.pos+=Eigen::Vector3d(2.,3.,4.);tested->P_*=.2;obs.valid=false;
}
TEST(PointLIOFilterTransaction, RejectedObservationRestoresActualStateAndCovariance) {
  Filter f;tested=&f;f.init_dyn_share_modified_2h(nullptr,nullptr,brokenModel);
  f.x_.pos<<1.,2.,3.;f.P_*=2.;
  const auto before=f.x_;const auto covariance=f.P_;
  EXPECT_FALSE(f.update_iterated_dyn_share_modified());
  EXPECT_EQ((f.x_.pos-before.pos).norm(),0.);
  EXPECT_EQ((f.P_-covariance).norm(),0.);
}
