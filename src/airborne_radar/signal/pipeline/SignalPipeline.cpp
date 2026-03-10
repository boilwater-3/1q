// Copyright 2026. All Rights Reserved.
//
// Description: SignalPipeline 的实现。

#include "airborne_radar/signal/pipeline/SignalPipeline.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/core/pipeline/IChainProcessor.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

tracking::TrackFilterConfig ToTrackFilterConfig(
    const SignalPipelineConfig &config) {
  tracking::TrackFilterConfig filter_config;
  filter_config.speed_decay_ratio_on_loss = config.speed_decay_ratio_on_loss;
  filter_config.rcs_decay_ratio_on_loss = config.rcs_decay_ratio_on_loss;
  filter_config.jamming_acceleration_penalty =
      config.jamming_acceleration_penalty;
  filter_config.stable_acceleration_gain = config.stable_acceleration_gain;
  return filter_config;
}

const association::AssociationMatch *FindAssociationMatch(
    const association::AssociationResult &result,
    std::size_t target_index) {
  for (const association::AssociationMatch &match : result.matches) {
    if (match.target_index == target_index) {
      return &match;
    }
  }
  return nullptr;
}

struct SignalCycleContext {
  const common::TargetFeatureList *input_state{nullptr};
  const environment::IEnvironmentService *environment{nullptr};
  common::TargetFeatureList output_state;
  std::vector<tracking::TrackMeasurement> track_measurements;

  environment::EnvironmentSnapshot environment_snapshot{};

  // Reused, cache-friendly SoA buffers to avoid per-stage temporary allocations.
  std::vector<float> signal_term_db;
  std::vector<float> speed_penalty_db;
  std::vector<float> detection_margin_db;
  std::vector<std::uint8_t> detection_succeeded;
  association::AssociationResult association_result;
  std::vector<std::uint64_t> association_keys;
};

class EnvironmentSamplingStage
    : public core::pipeline::IChainProcessor<SignalCycleContext> {
protected:
  void ProcessNode(SignalCycleContext &context) override {
    context.environment_snapshot = context.environment->SampleEnvironment();

    const std::size_t target_count = context.input_state->size();
    context.output_state = *context.input_state;

    context.signal_term_db.resize(target_count);
    context.speed_penalty_db.resize(target_count);
    context.detection_margin_db.resize(target_count);
    context.detection_succeeded.resize(target_count);
    context.association_keys.resize(target_count);
    context.track_measurements.clear();
  }
};

class EchoEstimationStage
    : public core::pipeline::IChainProcessor<SignalCycleContext> {
protected:
  void ProcessNode(SignalCycleContext &context) override {
    const common::TargetFeatureList &input = *context.input_state;
    const std::size_t count = input.size();

    for (std::size_t i = 0; i < count; ++i) {
      context.signal_term_db[i] = input[i].current_track_rcs * 6.0f;
      context.speed_penalty_db[i] = input[i].current_track_speed * 0.002f;
    }
  }
};

class DetectionStage : public core::pipeline::IChainProcessor<SignalCycleContext> {
public:
  explicit DetectionStage(const SignalPipelineConfig *config) : config_(config) {}

protected:
  void ProcessNode(SignalCycleContext &context) override {
    const float environment_penalty_db =
        context.environment_snapshot.propagation_loss_db * 0.2f +
        context.environment_snapshot.clutter_power_db * 0.3f +
        (context.environment_snapshot.jamming_detected ? 5.0f : 0.0f);

    const std::size_t count = context.signal_term_db.size();
    Eigen::Map<const Eigen::ArrayXf> signal_term(context.signal_term_db.data(),
                                                 static_cast<Eigen::Index>(count));
    Eigen::Map<const Eigen::ArrayXf> speed_penalty(context.speed_penalty_db.data(),
                                                   static_cast<Eigen::Index>(count));
    Eigen::Map<Eigen::ArrayXf> detection_margin(context.detection_margin_db.data(),
                                                static_cast<Eigen::Index>(count));
    detection_margin =
        signal_term - speed_penalty - Eigen::ArrayXf::Constant(detection_margin.size(),
                                                                environment_penalty_db);

    for (std::size_t i = 0; i < count; ++i) {
      const float margin = context.detection_margin_db[i];
      context.detection_succeeded[i] =
          static_cast<std::uint8_t>(margin >= config_->min_detection_margin_db);
    }
  }

private:
  const SignalPipelineConfig *config_{nullptr};
};

class AssociationStage
    : public core::pipeline::IChainProcessor<SignalCycleContext> {
public:
  explicit AssociationStage(association::DataAssociationEngine *engine)
      : engine_(engine) {}

protected:
  void ProcessNode(SignalCycleContext &context) override {
    context.association_result =
        engine_->AssociateDetections(*context.input_state,
                                     context.detection_succeeded);
    context.association_keys = context.association_result.target_keys;
  }

private:
  association::DataAssociationEngine *engine_{nullptr};
};

class TrackingFilterStage
    : public core::pipeline::IChainProcessor<SignalCycleContext> {
public:
  explicit TrackingFilterStage(tracking::TrackFilter *filter) : filter_(filter) {}

protected:
  void ProcessNode(SignalCycleContext &context) override {
    const common::TargetFeatureList &input = *context.input_state;
    common::TargetFeatureList &output = context.output_state;

    const std::size_t count = output.size();
    context.track_measurements.clear();
    for (std::size_t i = 0; i < count; ++i) {
      tracking::TrackFilterContext filter_context;
      filter_context.detection_succeeded = context.detection_succeeded[i] != 0U;
      filter_context.jamming_detected =
          context.environment_snapshot.jamming_detected;
      filter_context.detection_margin_db = context.detection_margin_db[i];
      output[i] = filter_->Filter(input[i], filter_context);

      if (!filter_context.detection_succeeded) {
        continue;
      }

        const association::AssociationMatch *match =
          FindAssociationMatch(context.association_result, i);
      tracking::TrackMeasurement measurement;
        measurement.source_index = i;
      measurement.association_key = context.association_keys[i];
        measurement.matched_existing_track = match != nullptr;
        measurement.association_cost = match != nullptr ? match->cost : 0.0f;
        measurement.detection_margin_db = context.detection_margin_db[i];
        measurement.has_cartesian_position = false;
      measurement.position = Eigen::Vector3f::Zero();
        measurement.observed_speed = output[i].current_track_speed;
      measurement.velocity =
          Eigen::Vector3f(output[i].current_track_speed, 0.0f, 0.0f);
        measurement.observed_acceleration = output[i].current_track_acceleration;
      measurement.acceleration =
          Eigen::Vector3f(output[i].current_track_acceleration, 0.0f, 0.0f);
      measurement.rcs = output[i].current_track_rcs;
      measurement.jamming_detected = output[i].check_jamming_detected;
      context.track_measurements.push_back(measurement);
    }
  }

private:
  tracking::TrackFilter *filter_{nullptr};
};

} // namespace

struct SignalPipeline::Impl {
  explicit Impl(SignalPipelineConfig initial_config)
  : config(initial_config),
    track_filter(ToTrackFilterConfig(initial_config)) {
    // 按配置条件创建 Kalman 组件
    if (config.enable_kalman_filter) {
      tracking::KalmanPredictorConfig pred_cfg;
      pred_cfg.noise_diff_coeff = config.kalman_noise_diff_coeff;
      kalman_predictor = std::unique_ptr<tracking::KalmanPredictor>(new tracking::KalmanPredictor(pred_cfg));

      tracking::KalmanUpdaterConfig upd_cfg;
      upd_cfg.measurement_noise_std = config.kalman_measurement_noise_std;
      kalman_updater = std::unique_ptr<tracking::KalmanUpdater>(new tracking::KalmanUpdater(upd_cfg));
    }

    pipeline_head = std::unique_ptr<EnvironmentSamplingStage>(new EnvironmentSamplingStage());
    pipeline_head->SetNext(std::unique_ptr<EchoEstimationStage>(new EchoEstimationStage()))
      ->SetNext(std::unique_ptr<DetectionStage>(new DetectionStage(&config)))
        ->SetNext(std::unique_ptr<AssociationStage>(new AssociationStage(&association_engine)))
      ->SetNext(std::unique_ptr<TrackingFilterStage>(new TrackingFilterStage(&track_filter)));
  }

  common::TargetFeatureList RunCycle(
      const common::TargetFeatureList &input_state,
      const environment::IEnvironmentService &environment) {
    cached_context.input_state = &input_state;
    cached_context.environment = &environment;

    pipeline_head->Process(cached_context);
    return cached_context.output_state;
  }

  std::vector<tracking::TrackMeasurement> GetLastTrackMeasurements() const {
    return cached_context.track_measurements;
  }

  /// @brief 获取 Kalman 预测器指针（可为 nullptr）。
  const tracking::IKalmanPredictor *GetKalmanPredictor() const {
    return kalman_predictor.get();
  }

  /// @brief 获取 Kalman 更新器指针（可为 nullptr）。
  const tracking::IKalmanUpdater *GetKalmanUpdater() const {
    return kalman_updater.get();
  }

  void UpdateConfig(SignalPipelineConfig new_config) {
    config = new_config;
    track_filter.UpdateConfig(ToTrackFilterConfig(new_config));
  }

  SignalPipelineConfig config{};
  association::DataAssociationEngine association_engine{};
  tracking::TrackFilter track_filter{};
  std::unique_ptr<tracking::KalmanPredictor> kalman_predictor;
  std::unique_ptr<tracking::KalmanUpdater> kalman_updater;
  SignalCycleContext cached_context{};
  std::unique_ptr<core::pipeline::IChainProcessor<SignalCycleContext> > pipeline_head;
};

SignalPipeline::SignalPipeline(SignalPipelineConfig config)
    : impl_(std::unique_ptr<Impl>(new Impl(config))) {}

SignalPipeline::~SignalPipeline() = default;

common::TargetFeatureList SignalPipeline::RunCycle(
    const common::TargetFeatureList &input_state,
    const environment::IEnvironmentService &environment) {
  return impl_->RunCycle(input_state, environment);
}

std::vector<tracking::TrackMeasurement>
SignalPipeline::GetLastTrackMeasurements() const {
  return impl_->GetLastTrackMeasurements();
}

void SignalPipeline::UpdateConfig(SignalPipelineConfig config) {
  impl_->UpdateConfig(config);
}

} // namespace pipeline
} // namespace signal
} // namespace airborne_radar
