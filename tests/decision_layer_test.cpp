// Copyright 2026. All Rights Reserved.
//
// Description: GTest based verification for the Decision Layer Pipeline

#include <algorithm>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/core/context/DecisionContext.h"
#include "1q/airborne_radar/decision/classifier/TargetClassifier.h"
#include "1q/airborne_radar/decision/eccm/EccmController.h"
#include "1q/airborne_radar/decision/lpi/LpiController.h"
#include "1q/airborne_radar/environment/database/FeatureRepository.h"

using namespace airborne_radar;

namespace {

common::DecisionTrackSnapshotList BuildSingleTrackSnapshot(
    float speed, float rcs, bool jamming) {
  return common::DecisionTrackSnapshotList{
      common::DecisionTrackSnapshot(speed, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f,
                                    jamming)};
}

std::string PrimaryCategory(
    const core::context::DecisionContext &context) {
  if (context.target_classification_result.empty()) {
    return "UNKNOWN";
  }
  return context.target_classification_result.front().target_type;
}

bool ContainsCommandType(const std::vector<common::RadarCommand> &commands,
                         common::RadarCommandType type) {
  return std::find_if(commands.begin(), commands.end(),
                      [type](const common::RadarCommand &cmd) {
                        return cmd.type == type;
                      }) != commands.end();
}

} // namespace

class DecisionPipelineTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Assemble the Pipeline (Chain of Responsibility)
    auto classifier =
        std::unique_ptr<decision::classifier::TargetClassifier>(new decision::classifier::TargetClassifier());
    auto lpi = std::unique_ptr<decision::lpi::LpiController>(new decision::lpi::LpiController());
    auto eccm = std::unique_ptr<decision::eccm::EccmController>(new decision::eccm::EccmController());

    // Connect the chain: Classifier -> LPI -> ECCM
    classifier->SetNext(std::move(lpi))->SetNext(std::move(eccm));

    pipeline_head_ = std::move(classifier);
  }

  std::unique_ptr<decision::pipeline::ITacticalProcessor> pipeline_head_;
};

TEST_F(DecisionPipelineTest, HighThreatAndJamming) {
  // Speed 800, RCS 2.5, Jamming TRUE
  core::context::DecisionContext ctx(
      BuildSingleTrackSnapshot(800.0f, 2.5f, true));
  ctx.eccm_source_info.has_jamming_signal = true;

  pipeline_head_->ProcessTactics(ctx);

  EXPECT_EQ(PrimaryCategory(ctx), "HIGH_THREAT_TARGET");

  const auto &cmds = ctx.decision_commands;
  EXPECT_EQ(cmds.size(), 6u);
  EXPECT_TRUE(ContainsCommandType(cmds, common::RadarCommandType::SET_LPI_POWER));
  EXPECT_TRUE(
      ContainsCommandType(cmds, common::RadarCommandType::ENABLE_SIDELOBE_CANCELLER));
  EXPECT_TRUE(ContainsCommandType(cmds, common::RadarCommandType::SET_AGILITY_FREQ));
}

TEST_F(DecisionPipelineTest, LowThreatClearEnvironment) {
  // Speed 150, RCS 1.0, Jamming FALSE
  core::context::DecisionContext ctx(
      BuildSingleTrackSnapshot(150.0f, 1.0f, false));
  ctx.eccm_source_info.has_jamming_signal = false;

  pipeline_head_->ProcessTactics(ctx);

  EXPECT_EQ(PrimaryCategory(ctx), "LOW_THREAT_TARGET");
  EXPECT_TRUE(ctx.decision_commands.empty());
}

TEST_F(DecisionPipelineTest, HighThreatClearEnvironment) {
  // Speed 900, RCS 5.0, Jamming FALSE
  core::context::DecisionContext ctx(
      BuildSingleTrackSnapshot(900.0f, 5.0f, false));
  ctx.eccm_source_info.has_jamming_signal = false;

  pipeline_head_->ProcessTactics(ctx);

  EXPECT_EQ(PrimaryCategory(ctx), "HIGH_THREAT_TARGET");

  const auto &cmds = ctx.decision_commands;
  EXPECT_EQ(cmds.size(), 1u);
  EXPECT_EQ(cmds[0].type, common::RadarCommandType::SET_LPI_POWER);
}

TEST_F(DecisionPipelineTest, HighRcsCanPromoteThreatWithModerateSpeed) {
  // Speed 180, RCS 4.2, Jamming FALSE
  core::context::DecisionContext ctx(
      BuildSingleTrackSnapshot(180.0f, 4.2f, false));
  ctx.eccm_source_info.has_jamming_signal = false;

  pipeline_head_->ProcessTactics(ctx);

  EXPECT_EQ(PrimaryCategory(ctx), "HIGH_THREAT_TARGET");
}

TEST_F(DecisionPipelineTest, RuleBasedClassificationProducesCategory) {
  core::context::DecisionContext ctx(
      BuildSingleTrackSnapshot(150.0f, 1.0f, false));
  ctx.eccm_source_info.has_jamming_signal = false;

  pipeline_head_->ProcessTactics(ctx);

  EXPECT_FALSE(ctx.target_classification_result.empty());
  EXPECT_NE(PrimaryCategory(ctx), "UNKNOWN");
}

TEST_F(DecisionPipelineTest, ClassifiesAllTargetsInView) {
  common::DecisionTrackSnapshotList tracks;
  tracks.push_back(common::DecisionTrackSnapshot(900.0f, 0.0f, 0.0f, 4.5f));
  tracks.push_back(common::DecisionTrackSnapshot(160.0f, 0.0f, 0.0f, 0.8f));

  core::context::DecisionContext ctx(tracks);
  ctx.eccm_source_info.has_jamming_signal = false;
  pipeline_head_->ProcessTactics(ctx);

  ASSERT_EQ(ctx.target_classification_result.size(), tracks.size());
  EXPECT_EQ(ctx.target_classification_result[0].target_type,
            "HIGH_THREAT_TARGET");
  EXPECT_EQ(ctx.target_classification_result[1].target_type,
            "LOW_THREAT_TARGET");
}

TEST(DecisionClassifierRepositoryTest, RepositoryMatchProvidesProbability) {
  environment::database::FeatureRepository repository;
  decision::classifier::TargetClassifier classifier(&repository);

  core::context::DecisionContext ctx(
      BuildSingleTrackSnapshot(780.0f, 2.8f, true));
  ctx.eccm_source_info.has_jamming_signal = true;

  classifier.ProcessTactics(ctx);

  EXPECT_EQ(PrimaryCategory(ctx), "HIGH_THREAT_FIGHTER");
}
