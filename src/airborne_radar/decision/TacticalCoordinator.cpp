// Copyright 2026. All Rights Reserved.
//
// Description: TacticalCoordinator 的默认实现。

#include "1q/airborne_radar/decision/TacticalCoordinator.h"

#include <algorithm>
#include <cmath>

#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {

namespace {

const float kMinRepositoryMatchProbability = 0.55f;
const float kMaxRepositoryMatchDistance = 1.80f;
const float kHighThreatConfidenceThreshold = 0.55f;
const std::uint32_t kLpiHoldCycles = 2;
const std::uint32_t kEccmHoldCycles = 2;

bool IsFinite(float value) {
  return std::isfinite(value);
}

bool ShouldAcceptRepositoryMatch(
    const environment::database::MatchResult& match_result) {
  if (match_result.target_type == "UNKNOWN") {
    return false;
  }

  if (!IsFinite(match_result.probability) || !IsFinite(match_result.distance)) {
    return false;
  }

  return match_result.probability >= kMinRepositoryMatchProbability &&
         match_result.distance <= kMaxRepositoryMatchDistance;
}

float ComputeThreatScore(const common::DecisionTrackSnapshot& track_snapshot) {
  float threat_score = 0.0f;
  const float track_speed = track_snapshot.state.speed;
  const float track_rcs = track_snapshot.state.rcs;
  const bool jamming_detected = track_snapshot.state.jamming_detected;

  if (track_speed > 300.0f) {
    threat_score += 2.0f;
  } else if (track_speed > 120.0f) {
    threat_score += 1.0f;
  }

  if (track_rcs > 3.0f) {
    threat_score += 1.0f;
  } else if (track_rcs > 1.2f) {
    threat_score += 0.5f;
  }

  if (jamming_detected) {
    threat_score += 1.0f;
  }

  if (track_snapshot.state.status == common::DecisionTrackStatus::kConfirmed) {
    threat_score += 0.25f;
  }

  return threat_score;
}

std::string ClassifyTrack(
    const common::DecisionTrackSnapshot& track_snapshot,
    const environment::database::IFeatureRepository* feature_repository) {
  if (feature_repository != nullptr) {
    environment::database::FeatureVector input;
    input.Set("speed", track_snapshot.state.speed);
    input.Set("rcs", track_snapshot.state.rcs);
    input.Set("jamming", track_snapshot.state.jamming_detected ? 1.0f : 0.0f);

    environment::database::MatchResult match_result;
    if (feature_repository->QueryBestMatch(input, match_result) &&
        ShouldAcceptRepositoryMatch(match_result)) {
      return match_result.target_type;
    }
  }

  const float threat_score = ComputeThreatScore(track_snapshot);
  if (threat_score >= 2.0f) {
    return "HIGH_THREAT_TARGET";
  }
  if (threat_score >= 0.8f) {
    return "LOW_THREAT_TARGET";
  }
  return "UNKNOWN";
}

bool IsHighThreatCategory(const std::string& category) {
  return category == "HIGH_THREAT_TARGET" ||
         category == "HIGH_THREAT_FIGHTER";
}

float UpdateConfidence(const common::DecisionTrackSnapshot& track_snapshot,
                       float previous_confidence) {
  float confidence = previous_confidence;
  if (track_snapshot.evidence.has_measurement_evidence) {
    confidence = std::min(1.0f, confidence + 0.35f);
  } else {
    confidence *= 0.60f;
  }

  if (track_snapshot.state.status == common::DecisionTrackStatus::kConfirmed) {
    confidence = std::max(confidence, 0.70f);
  }

  if (track_snapshot.state.status == common::DecisionTrackStatus::kTentative &&
      !track_snapshot.evidence.has_measurement_evidence) {
    confidence = std::min(confidence, 0.30f);
  }

  if (track_snapshot.state.status == common::DecisionTrackStatus::kLost) {
    confidence *= 0.50f;
  }

  return confidence;
}

} // namespace

TacticalCoordinator::TacticalCoordinator(
    const environment::database::IFeatureRepository* feature_repository)
    : feature_repository_(feature_repository) {}

TacticalDecisionResult TacticalCoordinator::Evaluate(
    const common::DecisionInputFrame& input_frame,
    TacticalStateStore& state_store) {
  TacticalDecisionResult result;
  result.target_classification_result.reserve(input_frame.tracks.size());

  bool should_reduce_power = false;
  bool should_enable_eccm = input_frame.environment_jamming_detected;

  if (!should_reduce_power && state_store.lpi_hold_cycles_remaining > 0U) {
    should_reduce_power = true;
    --state_store.lpi_hold_cycles_remaining;
  }
  if (!should_enable_eccm && state_store.eccm_hold_cycles_remaining > 0U) {
    should_enable_eccm = true;
    --state_store.eccm_hold_cycles_remaining;
  }

  for (std::size_t i = 0; i < input_frame.tracks.size(); ++i) {
    const common::DecisionTrackSnapshot& track_snapshot =
        input_frame.tracks[i];
    const std::string classification =
        ClassifyTrack(track_snapshot, feature_repository_);
    result.target_classification_result.emplace_back(classification);

    const std::uint64_t track_key = track_snapshot.state.association_key;
    const float previous_confidence =
        state_store.confidence_memory.count(track_key) != 0U
            ? state_store.confidence_memory[track_key]
            : 0.0f;
    const float confidence =
        UpdateConfidence(track_snapshot, previous_confidence);
    state_store.confidence_memory[track_key] = confidence;
    state_store.threat_memory[track_key] = ComputeThreatScore(track_snapshot);

    const bool can_trigger_aggressive_controls =
        track_snapshot.state.status != common::DecisionTrackStatus::kLost &&
        confidence >= kHighThreatConfidenceThreshold &&
        (track_snapshot.evidence.has_measurement_evidence ||
         track_snapshot.state.status == common::DecisionTrackStatus::kConfirmed);

    if (IsHighThreatCategory(classification) && can_trigger_aggressive_controls) {
      should_reduce_power = true;
    }

    if (track_snapshot.state.jamming_detected && can_trigger_aggressive_controls) {
      should_enable_eccm = true;
    }
  }

  if (should_enable_eccm) {
    result.selected_mode = TacticalMode::kProtectedEmission;
    state_store.eccm_hold_cycles_remaining = kEccmHoldCycles;
  } else if (should_reduce_power) {
    result.selected_mode = TacticalMode::kThreatResponse;
    state_store.lpi_hold_cycles_remaining = kLpiHoldCycles;
  } else {
    result.selected_mode = TacticalMode::kBaseline;
  }

  if (should_reduce_power) {
    result.proposals.push_back(TacticalProposal{
        common::ControlDirective(
            common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
            common::ControlDirectiveSource::EMISSION_CONTROL),
        60,
        "high-confidence threat requires reduced emission"});
  }

  if (should_enable_eccm) {
    result.proposals.push_back(TacticalProposal{
        common::ControlDirective(
            common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
            common::ControlDirectiveSource::SURVIVABILITY),
        90,
        "jamming environment requires sidelobe cancellation"});
    result.proposals.push_back(TacticalProposal{
        common::ControlDirective(
            common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
            common::ControlDirectiveSource::SURVIVABILITY),
        85,
        "jamming environment requires adaptive beamforming"});
    result.proposals.push_back(TacticalProposal{
        common::ControlDirective(
            common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
            common::ControlDirectiveSource::SURVIVABILITY),
        84,
        "jamming environment requires agility frequency"});
    result.proposals.push_back(TacticalProposal{
        common::ControlDirective(
            common::ControlDirectiveType::REQUEST_ECCM_REJITTER,
            common::ControlDirectiveSource::SURVIVABILITY),
        83,
        "jamming environment requires rejitter"});
    result.proposals.push_back(TacticalProposal{
        common::ControlDirective(
            common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
            common::ControlDirectiveSource::SURVIVABILITY),
        82,
        "jamming environment requires burnthrough gain"});
  }

  state_store.current_mode = result.selected_mode;
  state_store.last_classification_labels.clear();
  state_store.last_classification_labels.reserve(
      result.target_classification_result.size());
  for (std::size_t i = 0; i < result.target_classification_result.size(); ++i) {
    state_store.last_classification_labels.push_back(
        result.target_classification_result[i].target_type);
  }
  state_store.last_decision_summary = should_enable_eccm
                                          ? "protected-emission"
                                          : (should_reduce_power
                                                 ? "threat-response"
                                                 : "baseline");

  spdlog::debug(
      "[TacticalCoordinator] cycle_index={} tracks={} mode={} proposals={}",
      input_frame.cycle_index, input_frame.tracks.size(),
      static_cast<int>(result.selected_mode), result.proposals.size());
  return result;
}

} // namespace decision
} // namespace airborne_radar
