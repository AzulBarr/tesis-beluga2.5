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

TEST(QualityIntegration, SpatialSplitRefreshesOutputEvenWithoutEssResampling) {
  FastSLAMParams p;p.min_particles=30;p.max_particles=30;p.split_persistence=1;
  auto slam=Slam(p);
  const auto parent=std::get<2>(*slam->particles().begin());
  auto other=std::make_shared<Hypothesis>(*parent);other->id=100;
  parent->has_local_pose=true;parent->local_pose=Pose();
  other->has_local_pose=true;other->local_pose=Pose(20);
  slam->install_population({{parent,parent,.55,Pose(),true},{other,parent,.45,Pose(),true}},30);
  std::size_t count=0;
  for(const auto& particle:slam->particles())if(std::get<2>(particle)==parent)++count;
  ASSERT_GE(count,4U);
  std::size_t index=0;
  for(auto&& particle:slam->particles()) {
    if(std::get<2>(particle)==parent)std::get<0>(particle)=Pose(index++<count/2?0:5);
    else std::get<0>(particle)=Pose(20);
  }
  slam->post_update({},{});
  ASSERT_EQ(slam->best_hypothesis_id(),parent->id);
  const auto before_count=slam->particles().size();
  slam->resample();
  EXPECT_EQ(slam->get_active_hypotheses_count(),3U);
  EXPECT_EQ(slam->particles().size(),before_count);
  EXPECT_EQ(slam->best_hypothesis_id(),other->id);
  EXPECT_NEAR((other->local_pose.inverse()*slam->best_pose()).translation().norm(),0,1e-12);
  EXPECT_NEAR(slam->hypothesis_masses().at(other->id),.45,1e-12);
}

TEST(QualityIntegration, PruningRefreshesOutputAfterPopulationInstallation) {
  FastSLAMParams p;p.min_particles=30;p.max_particles=30;
  auto slam=Slam(p);const auto parent=std::get<2>(*slam->particles().begin());
  auto other=std::make_shared<Hypothesis>(*parent);other->id=100;
  parent->has_local_pose=true;parent->local_pose=Pose();
  other->has_local_pose=true;other->local_pose=Pose(20);
  slam->install_population({{parent,parent,.6,Pose(),true},{other,parent,.4,Pose(),true}},30);
  slam->post_update({},{});ASSERT_EQ(slam->best_hypothesis_id(),parent->id);
  for(auto&& particle:slam->particles()) {
    if(std::get<2>(particle)==parent)std::get<1>(particle)=beluga::Weight(0);
    else std::get<0>(particle)=Pose(20);
  }
  slam->resample();
  EXPECT_EQ(slam->get_active_hypotheses_count(),1U);
  EXPECT_EQ(slam->best_hypothesis_id(),other->id);
  EXPECT_NEAR((other->local_pose.inverse()*slam->best_pose()).translation().norm(),0,1e-12);
  EXPECT_NEAR(slam->hypothesis_masses().at(other->id),1,1e-12);
}

TEST(QualityIntegration, OutputRiskUsesCombinedPositionMassWithoutChangingInference) {
  for (const std::string mode : {"map", "pose_risk"}) {
    FastSLAMParams p;p.min_particles=30;p.max_particles=30;p.output_selection_mode=mode;
    auto slam=Slam(p);const auto parent=std::get<2>(*slam->particles().begin());
    auto a=std::make_shared<Hypothesis>(*parent);a->id=100;
    auto b=std::make_shared<Hypothesis>(*parent);b->id=101;
    parent->has_local_pose=a->has_local_pose=b->has_local_pose=true;
    parent->local_pose=Pose();a->local_pose=Pose(1,0,.2);b->local_pose=Pose(1,0,.3);
    slam->install_population({{parent,parent,.4,Pose(),true},{a,parent,.3,Pose(),true},
                             {b,parent,.3,Pose(),true}},30);
    const auto masses=slam->hypothesis_masses();
    std::vector<state_type> poses;
    std::vector<double> weights;
    std::vector<std::shared_ptr<Hypothesis>> owners;
    for(const auto& particle:slam->particles()) {
      poses.push_back(std::get<0>(particle));
      weights.push_back(static_cast<double>(std::get<1>(particle)));
      owners.push_back(std::get<2>(particle));
    }
    slam->refresh_output_selection();
    EXPECT_EQ(slam->map_hypothesis_id(),parent->id);
    EXPECT_EQ(slam->best_hypothesis_id(),mode=="map"?parent->id:a->id);
    EXPECT_NEAR(slam->best_pose().translation().x(),mode=="map"?0:1,1e-12);
    EXPECT_NEAR(slam->best_pose().so2().log(),mode=="map"?0:.2,1e-12);
    EXPECT_NEAR(slam->map_position_risk_m2(),.6,1e-12);
    EXPECT_NEAR(slam->selected_position_risk_m2(),mode=="map"?.6:.4,1e-12);
    EXPECT_EQ(slam->hypothesis_masses(),masses);
    std::size_t i=0;
    for(const auto& particle:slam->particles()) {
      EXPECT_NEAR((poses[i].inverse()*std::get<0>(particle)).translation().norm(),0,1e-12);
      EXPECT_NEAR((poses[i].inverse()*std::get<0>(particle)).so2().log(),0,1e-12);
      EXPECT_DOUBLE_EQ(static_cast<double>(std::get<1>(particle)),weights[i]);
      EXPECT_EQ(std::get<2>(particle),owners[i]);++i;
    }
  }
}

TEST(QualityIntegration, InvalidOutputModeIsRejected) {
  FastSLAMParams p;p.output_selection_mode="unknown";
  EXPECT_THROW(Slam(p),std::invalid_argument);
}
