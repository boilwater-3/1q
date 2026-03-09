// Copyright 2026. All Rights Reserved.
//
// Description: SignalPipeline 的实现。

#include "airborne_radar/signal/pipeline/SignalPipeline.h"

#include <algorithm>

#include "1q/airborne_radar/environment/IEnvironmentService.h"

namespace airborne_radar::signal::pipeline {

SignalPipeline::SignalPipeline(SignalPipelineConfig config) : config_(config) {}

common::TargetFeatureList SignalPipeline::RunCycle(
    const common::TargetFeatureList &input_state,
    const environment::IEnvironmentService &environment) {
  const auto snapshot = environment.SampleEnvironment();

  common::TargetFeatureList output_state = input_state;

  for (std::size_t i = 0; i < output_state.size(); ++i) {
    const common::TargetFeature &input_feature = input_state[i];
    common::TargetFeature &output_feature = output_state[i];

    output_feature.check_jamming_detected = snapshot.jamming_detected;

    const float detection_margin_db =
        EstimateDetectionMarginDb(input_feature, snapshot);

    if (detection_margin_db < config_.min_detection_margin_db) {
      output_feature.current_track_speed =
          std::max(0.0f, input_feature.current_track_speed *
                             config_.speed_decay_ratio_on_loss);
      output_feature.current_track_rcs =
          std::max(0.05f,
                   input_feature.current_track_rcs * config_.rcs_decay_ratio_on_loss);
    }

    if (snapshot.jamming_detected) {
      output_feature.current_track_acceleration =
          input_feature.current_track_acceleration -
          config_.jamming_acceleration_penalty;
    } else {
      output_feature.current_track_acceleration =
          input_feature.current_track_acceleration +
          config_.stable_acceleration_gain * detection_margin_db;
    }
  }

  return output_state;
}

void SignalPipeline::UpdateConfig(SignalPipelineConfig config) {
  config_ = config;
}

float SignalPipeline::EstimateDetectionMarginDb(
    const common::TargetFeature &state,
    const environment::EnvironmentSnapshot &snapshot) const {
  const float target_signal_term_db = state.current_track_rcs * 6.0f;
  const float speed_penalty_db = state.current_track_speed * 0.002f;
  const float environment_penalty_db =
      snapshot.propagation_loss_db * 0.2f + snapshot.clutter_power_db * 0.3f +
      (snapshot.jamming_detected ? 5.0f : 0.0f);

  return target_signal_term_db - speed_penalty_db - environment_penalty_db;
}

} // namespace airborne_radar::signal::pipeline
