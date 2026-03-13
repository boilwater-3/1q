// Copyright 2026. All Rights Reserved.
//
// Description: SignalPipeline 的实现。

#include "airborne_radar/signal/pipeline/SignalPipeline.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <numeric>
#include <vector>

#include <Eigen/Core>
#include <spdlog/spdlog.h>

#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/core/pipeline/IChainProcessor.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

AssociationQualityMetrics ToPipelineAssociationQualityMetrics(
    const association::AssociationQualityMetrics &source) {
  AssociationQualityMetrics metrics;
  metrics.prior_track_count = source.prior_track_count;
  metrics.detection_count = source.detection_count;
  metrics.matched_count = source.matched_count;
  metrics.new_track_count = source.new_track_count;
  metrics.missed_track_count = source.missed_track_count;
  metrics.match_rate = source.match_rate;
  metrics.new_track_rate = source.new_track_rate;
  metrics.missed_track_rate = source.missed_track_rate;
  metrics.mean_match_cost = source.mean_match_cost;
  metrics.p95_match_cost = source.p95_match_cost;
  return metrics;
}

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

association::DataAssociationConfig ToAssociationConfig(
    const SignalPipelineConfig &config) {
  association::DataAssociationConfig association_config;
  association_config.kalman_noise_diff_coeff = config.kalman_noise_diff_coeff;
  association_config.kalman_measurement_noise_std =
      config.kalman_measurement_noise_std;
  return association_config;
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

tracking::MeasurementCovariance BuildMeasurementCovariance(
    const common::TargetFeature &target,
    float range_error_std,
    float angle_error_std,
    float default_measurement_noise_std) {
  if (range_error_std <= 0.0f || angle_error_std <= 0.0f) {
    return tracking::MeasurementCovariance::Identity() *
           default_measurement_noise_std * default_measurement_noise_std;
  }

  const float range_m = target.range_m > 0.1f
                            ? target.range_m
                            : std::max(
                                  Eigen::Vector3f(target.position_x,
                                                  target.position_y,
                                                  target.position_z)
                                      .norm(),
                                  0.1f);
  const float var_r = range_error_std * range_error_std;
  const float var_theta = angle_error_std * angle_error_std;
  Eigen::Vector3f pos = (target.position_x != 0.0f || target.position_y != 0.0f ||
                         target.position_z != 0.0f)
                            ? Eigen::Vector3f(target.position_x, target.position_y,
                                              target.position_z)
                            : Eigen::Vector3f(range_m, 0.0f, 0.0f);
  const float pos_norm = pos.norm();
  if (pos_norm > 0.1f) {
    const Eigen::Vector3f u = pos / pos_norm;
    const Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f uu_t = u * u.transpose();
    return var_r * uu_t + (range_m * range_m * var_theta) * (identity - uu_t);
  }

  return tracking::MeasurementCovariance::Identity() * var_r;
}

float ResolveSpeedMagnitude(const common::TargetFeature &target) {
  const Eigen::Vector3f velocity(target.current_track_velocity_x,
                                 target.current_track_velocity_y,
                                 target.current_track_velocity_z);
  if (velocity.squaredNorm() > 0.0f) {
    return velocity.norm();
  }
  return target.current_track_speed;
}

Eigen::Vector3f ResolveVelocityVector(const common::TargetFeature &target) {
  const Eigen::Vector3f velocity(target.current_track_velocity_x,
                                 target.current_track_velocity_y,
                                 target.current_track_velocity_z);
  if (velocity.squaredNorm() > 0.0f) {
    return velocity;
  }
  return Eigen::Vector3f(target.current_track_speed, 0.0f, 0.0f);
}

float ResolveAccelerationMagnitude(const common::TargetFeature &target) {
  const Eigen::Vector3f acceleration(target.current_track_acceleration_x,
                                     target.current_track_acceleration_y,
                                     target.current_track_acceleration_z);
  if (acceleration.squaredNorm() > 0.0f) {
    return acceleration.norm();
  }
  return target.current_track_acceleration;
}

Eigen::Vector3f ResolveAccelerationVector(const common::TargetFeature &target) {
  const Eigen::Vector3f acceleration(target.current_track_acceleration_x,
                                     target.current_track_acceleration_y,
                                     target.current_track_acceleration_z);
  if (acceleration.squaredNorm() > 0.0f) {
    return acceleration;
  }
  return Eigen::Vector3f(target.current_track_acceleration, 0.0f, 0.0f);
}

[[noreturn]] void AbortLifecycleAssemblyConfigViolation(
    const char *message,
    float value) {
  spdlog::critical(
      "[SignalPipeline] lifecycle auto-assembly config violation: {} (value={})",
      message,
      value);
  std::abort();
}

[[noreturn]] void AbortLifecycleAssemblyConfigViolation(
    const char *message,
    std::size_t value) {
  spdlog::critical(
      "[SignalPipeline] lifecycle auto-assembly config violation: {} (value={})",
      message,
      value);
  std::abort();
}

class AutoConfiguredLifecycleManager final
    : public tracking::ITrackLifecycleManager {
 public:
  explicit AutoConfiguredLifecycleManager(const SignalPipelineConfig &config)
      : pool_(config.lifecycle_track_pool_initial_chunk,
              config.lifecycle_track_pool_max_chunks) {
    const float measurement_noise_std =
        std::max(config.kalman_measurement_noise_std, 0.001f);

    if (config.enable_imm_lifecycle) {
      const std::size_t model_count = config.imm_model_noise_diff_coeffs.size();
      if (model_count == 0U) {
        AbortLifecycleAssemblyConfigViolation(
            "IMM enabled but imm_model_noise_diff_coeffs is empty", model_count);
      }

      imm_predictors_owned_.reserve(model_count);
      imm_updaters_owned_.reserve(model_count);
      imm_predictors_.reserve(model_count);
      imm_updaters_.reserve(model_count);

      for (std::size_t i = 0; i < model_count; ++i) {
        const float noise_diff_coeff = config.imm_model_noise_diff_coeffs[i];
        if (noise_diff_coeff <= 0.0f) {
          AbortLifecycleAssemblyConfigViolation(
              "IMM model noise_diff_coeff must be > 0", noise_diff_coeff);
        }

        tracking::KalmanPredictorConfig predictor_config;
        predictor_config.noise_diff_coeff = noise_diff_coeff;
        std::unique_ptr<tracking::KalmanPredictor> predictor(
            new tracking::KalmanPredictor(predictor_config));

        tracking::KalmanUpdaterConfig updater_config;
        updater_config.measurement_noise_std = measurement_noise_std;
        std::unique_ptr<tracking::KalmanUpdater> updater(
            new tracking::KalmanUpdater(updater_config));

        imm_predictors_.push_back(predictor.get());
        imm_updaters_.push_back(updater.get());
        imm_predictors_owned_.push_back(std::move(predictor));
        imm_updaters_owned_.push_back(std::move(updater));
      }

      Eigen::MatrixXf transition_probability = BuildTransitionProbability(config, model_count);
      Eigen::VectorXf initial_weights = BuildInitialWeights(config, model_count);

      lifecycle_manager_.reset(new tracking::TrackLifecycleManager(
          pool_, config.lifecycle_config, imm_predictors_, imm_updaters_,
          transition_probability, initial_weights));
      return;
    }

    if (config.enable_kalman_filter) {
      tracking::KalmanPredictorConfig predictor_config;
      predictor_config.noise_diff_coeff = std::max(config.kalman_noise_diff_coeff, 0.001f);
      kalman_predictor_.reset(new tracking::KalmanPredictor(predictor_config));

      tracking::KalmanUpdaterConfig updater_config;
      updater_config.measurement_noise_std = measurement_noise_std;
      kalman_updater_.reset(new tracking::KalmanUpdater(updater_config));

      lifecycle_manager_.reset(new tracking::TrackLifecycleManager(
          pool_, config.lifecycle_config, kalman_predictor_.get(),
          kalman_updater_.get()));
      return;
    }

    lifecycle_manager_.reset(
        new tracking::TrackLifecycleManager(pool_, config.lifecycle_config));
  }

  void Update(const tracking::CycleContext &cycle,
              const std::vector<tracking::TrackMeasurement> &measurements) override {
    lifecycle_manager_->Update(cycle, measurements);
  }

  common::TargetFeatureList BuildFeatureSnapshot() const override {
    return lifecycle_manager_->BuildFeatureSnapshot();
  }

  std::vector<tracking::AssociationTrackSeed> BuildAssociationSeeds() const override {
    return lifecycle_manager_->BuildAssociationSeeds();
  }

 private:
  static Eigen::MatrixXf BuildTransitionProbability(
      const SignalPipelineConfig &config,
      std::size_t model_count) {
    if (config.imm_transition_probability.empty()) {
      Eigen::MatrixXf matrix = Eigen::MatrixXf::Constant(
          static_cast<Eigen::Index>(model_count),
          static_cast<Eigen::Index>(model_count),
          model_count > 1U ? 0.05f / static_cast<float>(model_count - 1U) : 1.0f);
      matrix.diagonal().setConstant(model_count > 1U ? 0.95f : 1.0f);
      return matrix;
    }

    if (config.imm_transition_probability.size() != model_count * model_count) {
      AbortLifecycleAssemblyConfigViolation(
          "imm_transition_probability size must equal model_count*model_count",
          config.imm_transition_probability.size());
    }

    Eigen::MatrixXf matrix(static_cast<Eigen::Index>(model_count),
                           static_cast<Eigen::Index>(model_count));
    for (std::size_t r = 0; r < model_count; ++r) {
      float row_sum = 0.0f;
      for (std::size_t c = 0; c < model_count; ++c) {
        const float value =
            config.imm_transition_probability[r * model_count + c];
        if (value < 0.0f || value > 1.0f) {
          AbortLifecycleAssemblyConfigViolation(
              "IMM transition probability must be in [0,1]", value);
        }
        matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = value;
        row_sum += value;
      }
      if (std::fabs(row_sum - 1.0f) > 1e-3f) {
        AbortLifecycleAssemblyConfigViolation(
            "each IMM transition matrix row must sum to 1", row_sum);
      }
    }
    return matrix;
  }

  static Eigen::VectorXf BuildInitialWeights(
      const SignalPipelineConfig &config,
      std::size_t model_count) {
    if (config.imm_initial_weights.empty()) {
      return Eigen::VectorXf::Constant(
          static_cast<Eigen::Index>(model_count),
          1.0f / static_cast<float>(model_count));
    }

    if (config.imm_initial_weights.size() != model_count) {
      AbortLifecycleAssemblyConfigViolation(
          "imm_initial_weights size must equal model_count",
          config.imm_initial_weights.size());
    }

    Eigen::VectorXf weights(static_cast<Eigen::Index>(model_count));
    float sum = 0.0f;
    for (std::size_t i = 0; i < model_count; ++i) {
      const float value = config.imm_initial_weights[i];
      if (value < 0.0f || value > 1.0f) {
        AbortLifecycleAssemblyConfigViolation(
            "IMM initial weight must be in [0,1]", value);
      }
      weights(static_cast<Eigen::Index>(i)) = value;
      sum += value;
    }

    if (std::fabs(sum - 1.0f) > 1e-3f) {
      AbortLifecycleAssemblyConfigViolation(
          "IMM initial weights must sum to 1", sum);
    }
    return weights;
  }

 private:
  tracking::BoostTrackPool pool_;
  std::unique_ptr<tracking::KalmanPredictor> kalman_predictor_;
  std::unique_ptr<tracking::KalmanUpdater> kalman_updater_;
  std::vector<std::unique_ptr<tracking::KalmanPredictor> > imm_predictors_owned_;
  std::vector<std::unique_ptr<tracking::KalmanUpdater> > imm_updaters_owned_;
  std::vector<const tracking::IKalmanPredictor *> imm_predictors_;
  std::vector<const tracking::IKalmanUpdater *> imm_updaters_;
  std::unique_ptr<tracking::TrackLifecycleManager> lifecycle_manager_;
};

struct SignalCycleContext {
  const common::TargetFeatureList *input_state{nullptr};
  const environment::IEnvironmentService *environment{nullptr};
  common::TargetFeatureList output_state;
  std::vector<tracking::TrackMeasurement> track_measurements;

  environment::EnvironmentSnapshot environment_snapshot{};

  std::vector<float> signal_term_db;
  std::vector<float> speed_penalty_db;
  std::vector<float> detection_margin_db;
  std::vector<std::uint8_t> detection_succeeded;
  association::AssociationResult association_result;
  std::vector<std::uint64_t> association_keys;
  std::vector<tracking::MeasurementCovariance> measurement_covariances;
};

class EnvironmentSamplingStage
    : public core::pipeline::IChainProcessor<SignalCycleContext> {
public:
  explicit EnvironmentSamplingStage(const SignalPipelineConfig *config)
      : config_(config) {}

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
    context.measurement_covariances.assign(
        target_count,
        tracking::MeasurementCovariance::Identity() *
            config_->kalman_measurement_noise_std *
            config_->kalman_measurement_noise_std);
    context.track_measurements.clear();
  }

private:
  const SignalPipelineConfig *config_{nullptr};
};

class EchoEstimationStage
    : public core::pipeline::IChainProcessor<SignalCycleContext> {
protected:
  void ProcessNode(SignalCycleContext &context) override {
    const common::TargetFeatureList &input = *context.input_state;
    const std::size_t count = input.size();

    for (std::size_t i = 0; i < count; ++i) {
      context.signal_term_db[i] = input[i].current_track_rcs * 6.0f;
      context.speed_penalty_db[i] = ResolveSpeedMagnitude(input[i]) * 0.002f;
    }
  }
};

/// @brief 物理化回波估计阶段，使用 SignalDetector 执行雷达方程。
class PhysicsEchoEstimationStage
    : public core::pipeline::IChainProcessor<SignalCycleContext> {
public:
  explicit PhysicsEchoEstimationStage(
      detection::SignalDetector *detector,
      const SignalPipelineConfig *config,
      int pulse_count,
      bool coherent_integration)
      : detector_(detector),
        config_(config),
        pulse_count_(pulse_count),
        coherent_integration_(coherent_integration) {}

protected:
  void ProcessNode(SignalCycleContext &context) override {
    const common::TargetFeatureList &input = *context.input_state;
    const std::size_t count = input.size();

    const float clutter_w =
        std::pow(10.0f,
                 context.environment_snapshot.clutter_power_db / 10.0f);
    const float jam_w =
        context.environment_snapshot.jamming_detected ? 1e-12f : 0.0f;

    detection::EnvironmentState env;
    env.propagation_loss_db = context.environment_snapshot.propagation_loss_db;
    env.clutter_noise_w = clutter_w;
    env.jam_noise_w = jam_w;

    for (std::size_t i = 0; i < count; ++i) {
      const float range =
          input[i].range_m > 0.0f ? input[i].range_m : 50000.0f;
          
      detection::TargetReturn target;
      target.rcs_m2 = input[i].current_track_rcs;
      target.range_m = range;
      target.swerling_type = static_cast<detection::SwerlingModel>(
          input[i].target_swerling_type);

      detection::DetectionResult det = detector_->Detect(
          target,
          env,
          pulse_count_,
          coherent_integration_);

      context.signal_term_db[i] = det.snr_db;
      context.speed_penalty_db[i] = 0.0f;
      context.detection_margin_db[i] = det.snr_db;
      context.detection_succeeded[i] =
          static_cast<std::uint8_t>(det.detected ? 1U : 0U);
      context.measurement_covariances[i] = BuildMeasurementCovariance(
          input[i], det.range_error_std_m, det.angle_error_std_rad,
          config_->kalman_measurement_noise_std);
    }
  }

private:
  detection::SignalDetector *detector_{nullptr};
  const SignalPipelineConfig *config_{nullptr};
  int pulse_count_{1};
  bool coherent_integration_{false};
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
                                     context.detection_succeeded,
                                     context.measurement_covariances);
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
      measurement.external_target_id = input[i].external_target_id;
      measurement.association_key = context.association_keys[i];
      measurement.matched_existing_track = match != nullptr;
      measurement.association_cost = match != nullptr ? match->cost : 0.0f;
      measurement.used_position_association =
          context.association_result.used_position_association;
      measurement.used_external_association_seeds =
          context.association_result.used_external_association_seeds;
      measurement.detection_margin_db = context.detection_margin_db[i];
      measurement.has_cartesian_position =
          input[i].position_x != 0.0f || input[i].position_y != 0.0f ||
          input[i].position_z != 0.0f;
      measurement.position = measurement.has_cartesian_position
                                 ? Eigen::Vector3f(input[i].position_x,
                                                   input[i].position_y,
                                                   input[i].position_z)
                                 : Eigen::Vector3f::Zero();
        measurement.observed_speed = ResolveSpeedMagnitude(output[i]);
        measurement.velocity = ResolveVelocityVector(output[i]);
        measurement.observed_acceleration = ResolveAccelerationMagnitude(output[i]);
        measurement.acceleration = ResolveAccelerationVector(output[i]);
      measurement.rcs = output[i].current_track_rcs;
        measurement.jamming_detected = context.environment_snapshot.jamming_detected;
      measurement.measurement_covariance = context.measurement_covariances[i];

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
    association_engine(ToAssociationConfig(initial_config)),
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

    if (config.enable_physics_detection) {
      signal_detector = std::unique_ptr<detection::SignalDetector>(
          new detection::SignalDetector(config.radar_system));

      pipeline_head = std::unique_ptr<EnvironmentSamplingStage>(
          new EnvironmentSamplingStage(&config));
      pipeline_head
          ->SetNext(std::unique_ptr<PhysicsEchoEstimationStage>(
              new PhysicsEchoEstimationStage(
                  signal_detector.get(),
              &config,
                  config.pulse_count,
                  config.coherent_integration)))
          ->SetNext(std::unique_ptr<AssociationStage>(
              new AssociationStage(&association_engine)))
          ->SetNext(std::unique_ptr<TrackingFilterStage>(
              new TrackingFilterStage(&track_filter)));
    } else {
      pipeline_head = std::unique_ptr<EnvironmentSamplingStage>(
        new EnvironmentSamplingStage(&config));
      pipeline_head
          ->SetNext(std::unique_ptr<EchoEstimationStage>(
              new EchoEstimationStage()))
          ->SetNext(std::unique_ptr<DetectionStage>(
              new DetectionStage(&config)))
          ->SetNext(std::unique_ptr<AssociationStage>(
              new AssociationStage(&association_engine)))
          ->SetNext(std::unique_ptr<TrackingFilterStage>(
              new TrackingFilterStage(&track_filter)));
    }
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

  AssociationQualityMetrics GetLastAssociationQualityMetrics() const {
    return ToPipelineAssociationQualityMetrics(
        cached_context.association_result.quality_metrics);
  }

  void SetAssociationSeeds(
      const std::vector<tracking::AssociationTrackSeed> &seeds) {
    association_engine.SetAssociationSeeds(seeds);
  }

  void ResetAssociationSeedModeToStateless() {
    association_engine.ResetAssociationSeedModeToStateless();
  }

  std::unique_ptr<tracking::ITrackLifecycleManager>
  CreateAutoLifecycleManager() const {
    if (!config.enable_auto_lifecycle_manager) {
      return std::unique_ptr<tracking::ITrackLifecycleManager>();
    }
    return std::unique_ptr<tracking::ITrackLifecycleManager>(
        new AutoConfiguredLifecycleManager(config));
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
    association_engine.UpdateConfig(ToAssociationConfig(new_config));
    track_filter.UpdateConfig(ToTrackFilterConfig(new_config));
  }

  SignalPipelineConfig config{};
  association::DataAssociationEngine association_engine{};
  tracking::TrackFilter track_filter{};
  std::unique_ptr<tracking::KalmanPredictor> kalman_predictor;
  std::unique_ptr<tracking::KalmanUpdater> kalman_updater;
  std::unique_ptr<detection::SignalDetector> signal_detector;
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

AssociationQualityMetrics
SignalPipeline::GetLastAssociationQualityMetrics() const {
  return impl_->GetLastAssociationQualityMetrics();
}

void SignalPipeline::SetAssociationSeeds(
    const std::vector<tracking::AssociationTrackSeed> &seeds) {
  impl_->SetAssociationSeeds(seeds);
}

void SignalPipeline::ResetAssociationSeedModeToStateless() {
  impl_->ResetAssociationSeedModeToStateless();
}

std::unique_ptr<tracking::ITrackLifecycleManager>
SignalPipeline::CreateAutoLifecycleManager() const {
  return impl_->CreateAutoLifecycleManager();
}

void SignalPipeline::UpdateConfig(SignalPipelineConfig config) {
  impl_->UpdateConfig(config);
}

} // namespace pipeline
} // namespace signal
} // namespace airborne_radar
