/**
 * @file ar_eccm_evaluator_test.cpp
 * @brief 验证 EccmEvaluator::Evaluate 的全部分支（基础重载此前 0% 覆盖）。
 *
 * @note 基础 Evaluate 重载已标记为 `[[deprecated]]`（生产路径已迁移到带
 *       AssociationQualityInfo 的关联重载，见 EccmEvaluator.h）。本文件作为历史
 *       API 的回归覆盖仍需调用基础重载，故对整个文件抑制 deprecated 警告；
 *       关联重载的测试不受影响（其调用本身不触发该警告）。
 */

// 基础重载已 [[deprecated]]，但本文件作为历史回归覆盖仍需调用，整文件抑制。
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif

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
using ActivationSource = EccmEvaluator::ActivationSource;

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

float FindBurnthroughGain(const std::vector<TacticalProposal>& proposals) {
  for (std::size_t i = 0; i < proposals.size(); ++i) {
    if (proposals[i].directive.type ==
        session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN) {
      return proposals[i].directive.requested_value;
    }
  }
  return 1.0f;
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
  EXPECT_EQ(result.activation_source, ActivationSource::kNone);
}

TEST(EccmEvaluatorTest, HoldOnlyProducesCautiousFallback) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, true, &proposals);
  EXPECT_TRUE(result.eccm_activated);
  EXPECT_EQ(result.activation_source, ActivationSource::kCautiousFallback);
}

TEST(EccmEvaluatorTest, EmptyJammerSourcesWithNoHoldProducesFallback) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  info.jammer_sources.clear();
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
  EXPECT_EQ(result.activation_source, ActivationSource::kCautiousFallback);
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
  EXPECT_EQ(result.activation_source, ActivationSource::kCautiousFallback);
}

TEST(EccmEvaluatorTest, CredibleDeceptionSourceActivatesEccm) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  info.jammer_sources.push_back(MakeCredibleSource());
  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
  EXPECT_EQ(result.activation_source, ActivationSource::kEvidenceBased);
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
  EXPECT_EQ(result.activation_source, ActivationSource::kEvidenceBased);
}

TEST(EccmEvaluatorTest, BurnthroughGainIsMonotonicAndClamped) {
  EccmEvaluator evaluator;
  EccmSourceInfo moderate_info;
  moderate_info.has_jamming_signal = true;
  EccmJammerSourceInfo moderate = MakeCredibleSource();
  moderate.technique = session::JammingTechnique::kNoiseSuppression;
  moderate.jammer_power_db = 8.0f;
  moderate.jammer_to_signal_db = 6.0f;
  moderate_info.jammer_sources.push_back(moderate);
  std::vector<TacticalProposal> moderate_proposals;
  evaluator.Evaluate(moderate_info, false, &moderate_proposals);

  EccmSourceInfo strong_info = moderate_info;
  strong_info.jammer_sources[0].jammer_power_db = 32.0f;
  strong_info.jammer_sources[0].jammer_to_signal_db = 24.0f;
  std::vector<TacticalProposal> strong_proposals;
  evaluator.Evaluate(strong_info, false, &strong_proposals);

  const float moderate_gain = FindBurnthroughGain(moderate_proposals);
  const float strong_gain = FindBurnthroughGain(strong_proposals);
  EXPECT_GT(moderate_gain, 1.0f);
  EXPECT_GT(strong_gain, moderate_gain);
  EXPECT_LE(strong_gain, 2.0f);
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
  EXPECT_EQ(result.activation_source, ActivationSource::kEvidenceBased);
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
  EXPECT_EQ(result.activation_source, ActivationSource::kEvidenceBased);
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
  EXPECT_EQ(result.activation_source, ActivationSource::kAssociationPressure);
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
  EXPECT_EQ(result.activation_source, ActivationSource::kAssociationPressure);
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
  EXPECT_EQ(result.activation_source, ActivationSource::kAssociationPressure);
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
  EXPECT_EQ(result.activation_source, ActivationSource::kAssociationPressure);
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
  EXPECT_EQ(result.activation_source, ActivationSource::kCautiousFallback);
}

// =============================================================================
// credible 证据但评分全部不达标 → 走最低保底（activation_source 应为 kCautiousFallback）
//
// 覆盖一条此前无测试的混合路径：has_credible_multisource_evidence=true（confidence
// 刚过 kMinimumCredibleConfidence），但五项评分累加后全部低于各自阈值，控制流进入
// 末尾「最低保底」分支强制激活自适应波束形成。此时实际达标的 proposal 来自保底路径，
// 故 activation_source 标记为 kCautiousFallback 而非 kEvidenceBased。
// 见 EccmEvaluator.h 的 ActivationSource 枚举注释。
// =============================================================================

namespace {

/// 构造一个 credible（confidence 刚过阈值）但各项评分都被压低的干扰源：
/// - confidence = 0.35f（== kMinimumCredibleConfidence）→ has_credible=true，但
///   confidence_weight 小，累加后 adaptive_beamforming_score 仅 0.8*0.35=0.28 < 0.8 阈值；
/// - frequency_overlap_ratio / prf_lock_risk / jammer_power / jammer_to_signal 均低于
///   high 阈值，且 jammer_in_sidelobe=false，其余四项评分分支不触发；
/// - technique=kUnknown 仅 +0.4*0.35=0.14，不足以翻盘。
EccmJammerSourceInfo MakeCredibleButLowScoreSource() {
  EccmJammerSourceInfo source;
  source.technique = session::JammingTechnique::kUnknown;
  source.jammer_power_db = 1.0f;       // < kHighJammerPowerDb(8.0)
  source.jammer_to_signal_db = 1.0f;   // < kHighJammerToSignalDb(6.0)
  source.frequency_overlap_ratio = 0.1f;  // < kHighFrequencyOverlapRatio(0.5)
  source.prf_lock_risk = 0.1f;            // < kHighPrfLockRisk(0.5)
  source.jammer_in_sidelobe = false;
  source.confidence = 0.35f;  // == kMinimumCredibleConfidence
  return source;
}

}  // namespace

TEST(EccmEvaluatorTest, CredibleEvidenceButScoresBelowThresholdProducesFallback) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  info.jammer_sources.push_back(MakeCredibleButLowScoreSource());

  std::vector<TacticalProposal> proposals;
  // 基础重载：五项评分全不达标 → 末尾保底分支强制激活自适应波束形成。
  const EccmEvaluator::Result result = evaluator.Evaluate(info, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
  EXPECT_EQ(result.activation_source, ActivationSource::kCautiousFallback);
}

TEST(EccmEvaluatorTest, CredibleEvidenceButScoresBelowThresholdProducesFallbackAssoc) {
  EccmEvaluator evaluator;
  EccmSourceInfo info;
  info.has_jamming_signal = true;
  info.jammer_sources.push_back(MakeCredibleButLowScoreSource());
  // 关联重载：jammer_sources 非空走多源证据路径，逻辑与基础重载一致；
  // 这里关联压力为低（不触发 association 分支），验证同一保底语义。
  session::AssociationQualityInfo assoc;
  assoc.dominant_jamming_semantic = config::JammingSemantic::kNone;
  assoc.jamming_severity = 0.1f;
  assoc.association_stress = 0.05f;

  std::vector<TacticalProposal> proposals;
  const EccmEvaluator::Result result = evaluator.Evaluate(info, assoc, false, &proposals);
  EXPECT_TRUE(result.eccm_activated);
  EXPECT_EQ(result.activation_source, ActivationSource::kCautiousFallback);
}

}  // namespace
}  // namespace decision
}  // namespace airborne_radar

#ifdef __clang__
#pragma clang diagnostic pop
#endif
