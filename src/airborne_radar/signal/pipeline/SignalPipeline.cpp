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
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/core/pipeline/IChainProcessor.h"

namespace airborne_radar::signal::pipeline {

namespace {

struct SignalCycleContext {
  const common::TargetFeatureList *input_state{nullptr};
  const environment::IEnvironmentService *environment{nullptr};
  common::TargetFeatureList output_state;

  environment::EnvironmentSnapshot environment_snapshot{};

  // Reused, cache-friendly SoA buffers to avoid per-stage temporary allocations.
  std::vector<float> signal_term_db;
  std::vector<float> speed_penalty_db;
  std::vector<float> detection_margin_db;
  std::vector<std::uint8_t> detection_succeeded;
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
    context.association_keys =
        engine_->Associate(*context.input_state, context.detection_succeeded);
  }

private:
  association::DataAssociationEngine *engine_{nullptr};
};

class TrackingFilterStage
    : public core::pipeline::IChainProcessor<SignalCycleContext> {
public:
  explicit TrackingFilterStage(const SignalPipelineConfig *config) : config_(config) {}

protected:
  void ProcessNode(SignalCycleContext &context) override {
    const common::TargetFeatureList &input = *context.input_state;
    common::TargetFeatureList &output = context.output_state;

    const bool jamming = context.environment_snapshot.jamming_detected;
    const float jamming_penalty = config_->jamming_acceleration_penalty;
    const float stable_gain = config_->stable_acceleration_gain;

    const std::size_t count = output.size();
    for (std::size_t i = 0; i < count; ++i) {
      output[i].check_jamming_detected = jamming;

      if (context.detection_succeeded[i] == 0U) {
        output[i].current_track_speed =
          std::max(0.0f, input[i].current_track_speed * config_->speed_decay_ratio_on_loss);
        output[i].current_track_rcs =
          std::max(0.05f, input[i].current_track_rcs * config_->rcs_decay_ratio_on_loss);
      }

      if (jamming) {
        output[i].current_track_acceleration =
            input[i].current_track_acceleration - jamming_penalty;
      } else {
        output[i].current_track_acceleration =
            input[i].current_track_acceleration +
            stable_gain * context.detection_margin_db[i];
      }
    }
  }

private:
  const SignalPipelineConfig *config_{nullptr};
};

} // namespace

struct SignalPipeline::Impl {
  explicit Impl(SignalPipelineConfig initial_config)
  : config(initial_config) {
    pipeline_head = std::make_unique<EnvironmentSamplingStage>();
    pipeline_head->SetNext(std::make_unique<EchoEstimationStage>())
      ->SetNext(std::make_unique<DetectionStage>(&config))
        ->SetNext(std::make_unique<AssociationStage>(&association_engine))
      ->SetNext(std::make_unique<TrackingFilterStage>(&config));
  }

  common::TargetFeatureList RunCycle(
      const common::TargetFeatureList &input_state,
      const environment::IEnvironmentService &environment) {
    cached_context.input_state = &input_state;
    cached_context.environment = &environment;

    pipeline_head->Process(cached_context);
    return cached_context.output_state;
  }

  void UpdateConfig(SignalPipelineConfig new_config) {
    config = new_config;
  }

  SignalPipelineConfig config{};
  association::DataAssociationEngine association_engine{};
  SignalCycleContext cached_context{};
  std::unique_ptr<core::pipeline::IChainProcessor<SignalCycleContext> > pipeline_head;
};

SignalPipeline::SignalPipeline(SignalPipelineConfig config)
    : impl_(std::make_unique<Impl>(config)) {}

SignalPipeline::~SignalPipeline() = default;

common::TargetFeatureList SignalPipeline::RunCycle(
    const common::TargetFeatureList &input_state,
    const environment::IEnvironmentService &environment) {
  return impl_->RunCycle(input_state, environment);
}

void SignalPipeline::UpdateConfig(SignalPipelineConfig config) {
  impl_->UpdateConfig(config);
}

} // namespace airborne_radar::signal::pipeline
