/**
 * @file ar_eccm_evaluator_test.cpp
 * @brief 验证 EccmEvaluator::Evaluate 的全部分支（基础重载此前 0% 覆盖）。
 */

#include <gtest/gtest.h>

#include <vector>

#include "1q/airborne_radar/config/JammingSemantics.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/DecisionSourceInfo.h"
#include "airborne_radar/decision/EccmEvaluator.h"

namespace airborne_radar {
namespace decision {
namespace {

using session::EccmSourceInfo;
using session::EccmJammerSourceInfo;
using session::TacticalProposal;

EccmJammerSourceInfo MakeCredibleSource() {
  EccmJammerSourceInfo source;
  source.technique = session::JammingTechnique::kDeception;
  source.jammer_power_db = 10.0f;
  source.jammer_to_signal_db = 8.0f;
  source.frequency_overlap_ratio = 0.9f;
  source.prf_lock_risk = 0.9f;
  source.jammer_in_sidelobe = true;
  source.confidence = 1.0f;
  return source;
}

// =============================================================================
// 基础 Evaluate 重载（此前 0% 覆盖，26 个未覆盖分支）
// =============================================================================

TEST(EccmEvaluatorTest, NullProposalsReturnsResultWithoutCrash) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, nullptr);
  EXPECT_FALSE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, HoldOnlyProducesCautiousFallback) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, true, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, EmptyJammerSourcesWithNoHoldProducesFallback) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  info.jammer_sources.clear();
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, LowConfidenceSourceProducesFallback) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  EccmJammerSourceInfo source = MakeCredibleSource();
  source.confidence = 0.1f;  // 低于可信阈值
  info.jammer_sources.push_back(source);
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, CredibleDeceptionSourceActivatesEccm) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  info.jammer_sources.push_back(MakeCredibleSource());
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
  EXPECT_FALSE(proposals.empty());
}

TEST(EccmEvaluatorTest, NoiseSuppressionTechniqueActivatesEccm) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  EccmJammerSourceInfo source = MakeCredibleSource();
  source.technique = session::JammingTechnique::kNoiseSuppression;
  info.jammer_sources.push_back(source);
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, RepeaterTechniqueActivatesEccm) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  EccmJammerSourceInfo source = MakeCredibleSource();
  source.technique = session::JammingTechnique::kRepeater;
  info.jammer_sources.push_back(source);
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, UnknownTechniqueActivatesEccm) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  EccmJammerSourceInfo source = MakeCredibleSource();
  source.technique = session::JammingTechnique::kUnknown;
  info.jammer_sources.push_back(source);
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

// =============================================================================
// 关联质量压力 Evaluate 重载（AccumulateAssociationPressureFacts 分支）
// =============================================================================

TEST(EccmEvaluatorTest, AssociationPressureNoiseSuppressionActivatesEccm) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  session::AssociationQualityInfo assoc;
  assoc.dominant_jamming_semantic = config::JammingSemantic::kNoiseSuppression;
  assoc.jamming_severity = 0.6f;
  assoc.association_stress = 0.3f;
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, assoc, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, AssociationPressureDeceptionActivatesEccm) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  session::AssociationQualityInfo assoc;
  assoc.dominant_jamming_semantic = config::JammingSemantic::kDeception;
  assoc.jamming_severity = 0.7f;
  assoc.association_stress = 0.25f;
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, assoc, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, AssociationPressureRepeaterActivatesEccm) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  session::AssociationQualityInfo assoc;
  assoc.dominant_jamming_semantic = config::JammingSemantic::kRepeater;
  assoc.jamming_severity = 0.6f;
  assoc.association_stress = 0.2f;
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, assoc, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, AssociationPressureMixedActivatesEccm) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  session::AssociationQualityInfo assoc;
  assoc.dominant_jamming_semantic = config::JammingSemantic::kMixed;
  assoc.jamming_severity = 0.55f;
  assoc.association_stress = 0.2f;
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, assoc, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

TEST(EccmEvaluatorTest, LowSeverityAssociationPressureProducesFallback) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  session::AssociationQualityInfo assoc;
  assoc.dominant_jamming_semantic = config::JammingSemantic::kNone;
  assoc.jamming_severity = 0.1f;
  assoc.association_stress = 0.05f;
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, assoc, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
}

}  // namespace
}  // namespace decision
}  // namespace airborne_radar
