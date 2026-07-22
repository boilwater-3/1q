#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "airborne_radar/decision/EccmEvaluator.h"

namespace airborne_radar {
namespace {

session::ArInterferenceObservation BuildObservation(
    oneq::electromagnetics::RfSceneWaveformKind waveform_kind,
    double off_boresight_deg, double jammer_to_noise_db) {
  session::ArInterferenceObservation observation;
  observation.observation_id = 1U;
  observation.estimated_bearing_azimuth_deg = 10.0;
  observation.estimated_bearing_elevation_deg = 2.0;
  observation.estimated_off_boresight_deg = off_boresight_deg;
  observation.estimated_center_frequency_hz = 3.0e9;
  observation.estimated_bandwidth_hz = 2.0e6;
  observation.estimated_waveform_kind = waveform_kind;
  observation.jammer_to_noise_db = jammer_to_noise_db;
  observation.bearing_standard_deviation_deg = 1.0;
  observation.frequency_standard_deviation_hz = 1000.0;
  observation.bandwidth_standard_deviation_hz = 2000.0;
  return observation;
}

bool ContainsDirective(const std::vector<session::TacticalProposal>& proposals,
                       session::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const session::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

const session::TacticalProposal* FindDirective(
    const std::vector<session::TacticalProposal>& proposals,
    session::ControlDirectiveType type) {
  const auto it = std::find_if(proposals.begin(), proposals.end(),
                               [type](const session::TacticalProposal& proposal) {
                                 return proposal.directive.type == type;
                               });
  return it == proposals.end() ? nullptr : &*it;
}

TEST(EccmEvaluatorTest, EmptyObservationSetDoesNotActivateOrAppend) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;

  const decision::EccmEvaluator::Result result = evaluator.Evaluate({}, &proposals);

  EXPECT_FALSE(result.eccm_activated);
  EXPECT_EQ(result.activation_source, decision::EccmEvaluator::ActivationSource::kNone);
  EXPECT_TRUE(proposals.empty());
  EXPECT_FALSE(evaluator.Evaluate({}, nullptr).eccm_activated);
}

TEST(EccmEvaluatorTest, ContinuousOnAxisObservationSelectsBeamAndFrequencyOnly) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kContinuous, 0.5, 3.0);

  const decision::EccmEvaluator::Result result = evaluator.Evaluate({observation}, &proposals);

  EXPECT_TRUE(result.eccm_activated);
  EXPECT_EQ(result.activation_source, decision::EccmEvaluator::ActivationSource::kReceiverRf);
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_FALSE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
}

TEST(EccmEvaluatorTest, StrongOffAxisPulseObservationSelectsAllPhysicalMeasures) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> proposals;
  const session::ArInterferenceObservation observation = BuildObservation(
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain, 8.0, 12.0);

  const decision::EccmEvaluator::Result result = evaluator.Evaluate({observation}, &proposals);

  EXPECT_TRUE(result.eccm_activated);
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_TRUE(ContainsDirective(
      proposals, session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
}

TEST(EccmEvaluatorTest, BurnthroughRequestIsMonotonicAndHardwareBounded) {
  decision::EccmEvaluator evaluator;
  std::vector<session::TacticalProposal> moderate_proposals;
  std::vector<session::TacticalProposal> strong_proposals;
  evaluator.Evaluate({BuildObservation(
                         oneq::electromagnetics::RfSceneWaveformKind::kBandLimitedNoise,
                         0.0, 9.0)},
                     &moderate_proposals);
  evaluator.Evaluate({BuildObservation(
                         oneq::electromagnetics::RfSceneWaveformKind::kBandLimitedNoise,
                         0.0, 60.0)},
                     &strong_proposals);

  const session::TacticalProposal* moderate = FindDirective(
      moderate_proposals, session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN);
  const session::TacticalProposal* strong = FindDirective(
      strong_proposals, session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN);
  ASSERT_NE(moderate, nullptr);
  ASSERT_NE(strong, nullptr);
  EXPECT_TRUE(moderate->directive.has_requested_value);
  EXPECT_GE(strong->directive.requested_value, moderate->directive.requested_value);
  EXPECT_GE(moderate->directive.requested_value, 1.0f);
  EXPECT_LE(strong->directive.requested_value, 2.0f);
}

}  // namespace
}  // namespace airborne_radar
