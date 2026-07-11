#include "sbirs_sensor/pipeline/SbirsTrackingCoordinator.h"

#include <limits>

#include <Eigen/Cholesky>

#include "sbirs_sensor/foundation/SbirsGeometry.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

constexpr float kSbirsNisChiSquare2Dof95 = 5.99f;

float ComputeNormalizedInnovationSquared(const tracking::SbirsKalmanUpdateResult& update_result) {
  const Eigen::LLT<tracking::SbirsMeasurementCovariance> llt(
      update_result.innovation_covariance);
  if (llt.info() != Eigen::Success) {
    return std::numeric_limits<float>::infinity();
  }
  const tracking::SbirsMeasurementVector solved = llt.solve(update_result.innovation);
  return update_result.innovation.dot(solved);
}

}  // namespace

void SbirsTrackingCoordinator::InitializeTarget(
    std::uint64_t target_id, const session::SbirsSceneTarget& target,
    const config::SbirsTrackingConfig& tracking) {
  tracking::SbirsGaussianState initial_state;
  initial_state.mean(0) = static_cast<float>(target.position_ecef_m.x);
  initial_state.mean(2) = static_cast<float>(target.position_ecef_m.y);
  initial_state.mean(4) = static_cast<float>(target.position_ecef_m.z);
  if (target.has_velocity_ecef_m_per_s) {
    initial_state.mean(1) = static_cast<float>(target.velocity_ecef_m_per_s.x);
    initial_state.mean(3) = static_cast<float>(target.velocity_ecef_m_per_s.y);
    initial_state.mean(5) = static_cast<float>(target.velocity_ecef_m_per_s.z);
  }
  const float pos_var = tracking.initial_position_std_m * tracking.initial_position_std_m;
  const float vel_var = tracking.initial_velocity_std_m_per_s * tracking.initial_velocity_std_m_per_s;
  initial_state.covariance = tracking::SbirsStateCovariance::Zero();
  initial_state.covariance(0, 0) = pos_var;
  initial_state.covariance(1, 1) = vel_var;
  initial_state.covariance(2, 2) = pos_var;
  initial_state.covariance(3, 3) = vel_var;
  initial_state.covariance(4, 4) = pos_var;
  initial_state.covariance(5, 5) = vel_var;
  filter_states_[target_id] = initial_state;
  nis_gate_exceeded_counts_[target_id] = 0U;
  if (!tracking.enable_imm_tracking) {
    return;
  }
  if (!imm_initialized_) {
    InitializeImmComponents(tracking);
  }
  const int num_models = static_cast<int>(imm_filter_->GetModelStates().size());
  const float init_weight = 1.0f / static_cast<float>(num_models);
  std::vector<tracking::SbirsImmModelState> init_states;
  for (int i = 0; i < num_models; ++i) {
    init_states.push_back({initial_state, init_weight});
  }
  imm_filter_->SetModelStates(init_states);
}

SbirsTrackingUpdateResult SbirsTrackingCoordinator::Update(
    std::uint64_t target_id, const config::SbirsPolicyConfig& policy,
    foundation::SbirsRandomSource* random_source, float azimuth_deg, float elevation_deg,
    double range_m, float angular_rate_deg_per_sec, float dt_sec,
    const session::SbirsVector3M& satellite_position_ecef_m) {
  SbirsTrackingUpdateResult result;
  const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
      policy.error_model, random_source, azimuth_deg, elevation_deg, range_m, angular_rate_deg_per_sec);
  tracking::SbirsMeasurementVector measurement_rad;
  const float deg2rad = 0.0174532925f;
  measurement_rad << bearing.azimuth_deg * deg2rad, bearing.elevation_deg * deg2rad;
  const tracking::SbirsMeasurementCovariance measurement_covariance =
      tracking::BuildMeasurementCovariance(policy.error_model, range_m, elevation_deg,
                                           angular_rate_deg_per_sec);

  tracking::SbirsGaussianState combined;
  if (policy.tracking.enable_imm_tracking) {
    if (!imm_initialized_) {
      InitializeImmComponents(policy.tracking);
    }
    auto imm_it = imm_snapshots_.find(target_id);
    if (imm_it != imm_snapshots_.end() && !imm_it->second.model_states.empty()) {
      imm_filter_->SetModelStates(imm_it->second.model_states);
      imm_snapshots_.erase(imm_it);
    }
    for (auto& measurement_model : imm_measurement_models_) {
      measurement_model->SetSatellitePosition(satellite_position_ecef_m);
    }
    imm_filter_->Process(measurement_rad, dt_sec, measurement_covariance);
    combined = imm_filter_->GetCombinedState();
    filter_states_[target_id] = combined;

    tracking::SbirsImmSnapshot imm_snapshot;
    imm_snapshot.model_states = imm_filter_->GetModelStates();
    imm_snapshot.model_weights = imm_filter_->GetModelWeights();
    imm_snapshots_[target_id] = imm_snapshot;

    result.has_estimation_nis = true;
    const auto& imm_results = imm_filter_->GetModelUpdateResults();
    for (std::size_t index = 0U; index < imm_results.size(); ++index) {
      const float model_nis = ComputeNormalizedInnovationSquared(imm_results[index]);
      if (model_nis > result.estimation_nis) result.estimation_nis = model_nis;
      if (model_nis > kSbirsNisChiSquare2Dof95) result.estimation_nis_gate_exceeded = true;
    }
  } else {
    angle_measurement_model_.SetSatellitePosition(satellite_position_ecef_m);
    const tracking::SbirsEkfPredictor predictor(
        &cv_transition_model_, tracking::SbirsEkfPredictorConfig{policy.tracking.process_noise_diff_coeff});
    const tracking::SbirsGaussianState predicted = predictor.Predict(filter_states_[target_id], dt_sec);
    const tracking::SbirsEkfUpdater updater(&angle_measurement_model_);
    const tracking::SbirsKalmanUpdateResult update_result =
        updater.Update(predicted, measurement_rad, measurement_covariance);
    combined = update_result.posterior;
    filter_states_[target_id] = combined;
    result.has_estimation_nis = true;
    result.estimation_nis = ComputeNormalizedInnovationSquared(update_result);
    result.estimation_nis_gate_exceeded = result.estimation_nis > kSbirsNisChiSquare2Dof95;
  }

  if (policy.tracking.nis_gate_loss_cycles > 0U && result.estimation_nis_gate_exceeded) {
    const unsigned int exceeded_count = ++nis_gate_exceeded_counts_[target_id];
    result.lost_due_to_estimation_nis = exceeded_count >= policy.tracking.nis_gate_loss_cycles;
  } else {
    nis_gate_exceeded_counts_[target_id] = 0U;
  }
  const session::SbirsVector3M estimated_position{combined.mean(0), combined.mean(2),
                                                   combined.mean(4)};
  const session::SbirsVector3M estimated_los =
      foundation::Subtract(estimated_position, satellite_position_ecef_m);
  result.output_azimuth_deg = foundation::ComputeAzimuthDeg(estimated_los);
  result.output_elevation_deg = foundation::ComputeElevationDeg(estimated_los);
  return result;
}

void SbirsTrackingCoordinator::ReleaseTarget(std::uint64_t target_id) {
  filter_states_.erase(target_id);
  nis_gate_exceeded_counts_.erase(target_id);
  imm_snapshots_.erase(target_id);
}

void SbirsTrackingCoordinator::ClearForStandby() {
  nis_gate_exceeded_counts_.clear();
  imm_snapshots_.clear();
  imm_initialized_ = false;
}

SbirsTrackingRuntimeState SbirsTrackingCoordinator::CaptureRuntimeState() const {
  SbirsTrackingRuntimeState state;
  state.filter_states = filter_states_;
  state.nis_gate_exceeded_counts = nis_gate_exceeded_counts_;
  state.imm_active = imm_initialized_;
  state.imm_snapshots = imm_snapshots_;
  return state;
}

void SbirsTrackingCoordinator::RestoreRuntimeState(const SbirsTrackingRuntimeState& state) {
  filter_states_ = state.filter_states;
  nis_gate_exceeded_counts_ = state.nis_gate_exceeded_counts;
  imm_snapshots_ = state.imm_snapshots;
  imm_initialized_ = false;
}

void SbirsTrackingCoordinator::InitializeImmComponents(const config::SbirsTrackingConfig& tracking) {
  imm_predictors_owned_.clear();
  imm_predictors_.clear();
  imm_measurement_models_.clear();
  imm_updaters_owned_.clear();
  imm_updaters_.clear();
  imm_filter_.reset();

  const std::vector<float> q_values = tracking.imm_model_noise_diff_coeffs.empty()
                                          ? std::vector<float>{1.0f, 100.0f}
                                          : tracking.imm_model_noise_diff_coeffs;
  const int num_models = static_cast<int>(q_values.size());
  for (int i = 0; i < num_models; ++i) {
    auto measurement_model = std::make_unique<tracking::SbirsAngleMeasurementModel>();
    imm_measurement_models_.push_back(std::move(measurement_model));
    tracking::SbirsEkfPredictorConfig predictor_config;
    predictor_config.noise_diff_coeff = q_values[static_cast<std::size_t>(i)];
    auto predictor = std::make_unique<tracking::SbirsEkfPredictor>(&cv_transition_model_, predictor_config);
    imm_predictors_.push_back(predictor.get());
    imm_predictors_owned_.push_back(std::move(predictor));
    auto updater = std::make_unique<tracking::SbirsEkfUpdater>(imm_measurement_models_.back().get());
    imm_updaters_.push_back(updater.get());
    imm_updaters_owned_.push_back(std::move(updater));
  }

  tracking::SbirsImmConfig imm_config;
  imm_config.transition_probability.resize(num_models, num_models);
  for (int row = 0; row < num_models; ++row) {
    for (int column = 0; column < num_models; ++column) {
      imm_config.transition_probability(row, column) =
          (row == column) ? 0.95f : 0.05f / static_cast<float>(num_models - 1);
    }
  }
  imm_config.initial_weights.setConstant(num_models, 1.0f / static_cast<float>(num_models));
  imm_filter_ = std::make_unique<tracking::SbirsImmFilter>(imm_config, imm_predictors_, imm_updaters_);
  imm_initialized_ = true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
