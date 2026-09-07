#include <gtest/gtest.h>
#include "belugaslam_core/fastslam_oc_grid_core.hpp"
namespace {
Sophus::SE2d Pose(double x=0,double y=0,double a=0) {return {Sophus::SO2d{a},Eigen::Vector2d{x,y}};}
std::unique_ptr<BelugaSLAM> Slam(FastSLAMParams params={}) {
  params.enable_loop_closure=false;params.enable_pgo=false;
  return std::make_unique<BelugaSLAM>(BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{0.1,0.05,0.1,0.05}},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100,2,0.5,0.5,0.2,true},GridTypeOC{}},params);
}
}
TEST(QualityIntegration, StartsWithRequestedPopulation) {
  FastSLAMParams p;p.min_particles=5;p.max_particles=30;
  auto slam=Slam(p);EXPECT_EQ(slam->particles().size(),30U);
  double sum=0;for(const auto& particle:slam->particles()) sum+=static_cast<double>(std::get<1>(particle));
  EXPECT_NEAR(sum,1,1e-12);
}
TEST(QualityIntegration, FieldInvalidatesOnGridWriteAndDetachesAcrossClones) {
  Submap a(0,Pose(),80,80,0.1);
  a.mutable_grid().at(40,40)=5;
  const auto original=a.tracking_field();
  auto b=a.clone();b->mutable_grid().at(50,40)=5;
  const auto changed=b->tracking_field();
  EXPECT_EQ(a.tracking_field(),original);EXPECT_NE(changed,original);
  EXPECT_NEAR(changed->sample(1.05,0.05).distance,0,1e-6);
  EXPECT_GT(original->sample(1.05,0.05).distance,0.5);
}
TEST(QualityIntegration, RejectedScanDoesNotWriteGridOrGraph) {
  auto slam=Slam();auto h=std::get<2>(*slam->particles().begin());
  BelugaSLAM::measurement_type wall;for(int i=0;i<80;++i) wall.emplace_back(2.05,-1.05+i*.025);
  slam->measurement_model_map(wall);slam->update_occupancy_grid(wall,0);
  const auto cells=h->submaps.active_submaps.front()->grid().data();
  const auto nodes=h->submaps.trajectory_nodes.size();
  BelugaSLAM::measurement_type invalid;for(int i=0;i<80;++i) invalid.emplace_back(100+i,100);
  slam->sample_motion_model({Pose(.01),Pose()});slam->measurement_model_map(invalid);
  EXPECT_FALSE(h->tracking_usable);
  slam->update_occupancy_grid(invalid,10);
  EXPECT_EQ(h->submaps.trajectory_nodes.size(),nodes);
  EXPECT_EQ(h->submaps.active_submaps.front()->grid().data(),cells);
}
TEST(QualityIntegration, ZeroNoiseAndRepeatedSeedsAreIndependent) {
  beluga::DifferentialDriveModelParam p{0,0,0,0};BelugaSLAM::MotionModel exact(p);
  std::mt19937 first(42),second(42);
  auto draw=exact(std::make_tuple(Pose(.05),Pose()));
  for(int i=0;i<3;++i) EXPECT_NEAR(draw(Pose(),first).translation().x(),.05,1e-12);
  BelugaSLAM::MotionModel noisy(beluga::DifferentialDriveModelParam{.1,.1,.1,.1});
  auto sample=noisy(std::make_tuple(Pose(.1,0,.03),Pose()));
  first.seed(42);
  for(int i=0;i<7;++i) {
    const auto a=sample(Pose(),first),b=sample(Pose(),second);
    EXPECT_NEAR((a.inverse()*b).translation().norm(),0,1e-12);
    EXPECT_NEAR((a.inverse()*b).so2().log(),0,1e-12);
  }
}
TEST(QualityIntegration, MatchingDoesNotGreedilyMoveStationaryParticles) {
  auto slam=Slam();auto h=std::get<2>(*slam->particles().begin());
  auto map=std::make_shared<Submap>(0,Pose(),80,80,0.1);
  for(int y=10;y<70;++y) map->mutable_grid().at(60,y)=5;
  h->submaps.active_submaps.push_back(map);h->has_local_pose=true;
  std::vector<state_type> before;
  int i=0;for(auto&& particle:slam->particles()) {std::get<0>(particle)=Pose(.002*i++);before.push_back(std::get<0>(particle));}
  slam->sample_motion_model({Pose(),Pose()});
  BelugaSLAM::measurement_type scan;for(int y=10;y<70;++y) scan.emplace_back(2.05,-4+(y+.5)*.1);
  slam->measurement_model_map(scan);
  for(std::size_t j=0;j<before.size();++j) EXPECT_NEAR((before[j].inverse()*std::get<0>(*(slam->particles().begin()+j))).translation().norm(),0,1e-12);
}
TEST(QualityIntegration, RuntimeResolutionIsUsedBySubmapsAndPublishedMap) {
  FastSLAMParams p;p.map_resolution=.05;auto slam=Slam(p);
  slam->measurement_model_map({{1.05,.05}});const auto events=slam->update_occupancy_grid({{1.05,.05}},0);
  slam->post_update({},events);
  const auto h=std::get<2>(*slam->particles().begin());
  EXPECT_DOUBLE_EQ(h->submaps.active_submaps.front()->grid().resolution(),.05);
  EXPECT_DOUBLE_EQ(slam->best_occupancy_grid().resolution(),.05);
}
