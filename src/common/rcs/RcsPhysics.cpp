#include "common/rcs/RcsPhysics.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"
#include "common/numerics/Constants.h"

namespace oneq {
namespace internal {
namespace rcs {

namespace {

}  // namespace

float rcs_f419_xmm4r4(float radius_m, float wavenumber_k0) {
  const float safe_radius_m = std::max(radius_m, 0.0f);
  const float safe_k0 = std::max(wavenumber_k0, 0.0f);
  const float k0a = safe_k0 * safe_radius_m;
  const float area_m2 = static_cast<float>(oneq::internal::numerics::kPi) * safe_radius_m * safe_radius_m;
  if (k0a <= 0.0f || area_m2 <= 0.0f) {
    return 0.0f;
  }
  const float shaping = (k0a * k0a) / (1.0f + k0a * k0a);
  return area_m2 * shaping;
}

float rcs_f4322_xmm4r4(float wavenumber_k0, float radius_m, float psi_i_deg, float psi_s_deg,
                       float phi_deg) {
  const float base_rcs = rcs_f419_xmm4r4(radius_m, wavenumber_k0);
  if (base_rcs <= 0.0f) {
    return 0.0f;
  }
  const float psi_i = std::fabs(oneq::internal::numerics::DegToRad(psi_i_deg));
  const float psi_s = std::fabs(oneq::internal::numerics::DegToRad(psi_s_deg));
  const float phi = oneq::internal::numerics::DegToRad(phi_deg);
  const float angle_gain = std::max(0.0f, std::cos(psi_i)) * std::max(0.0f, std::cos(psi_s));
  const float phase_gain = 0.5f * (1.0f + std::cos(phi));
  return base_rcs * angle_gain * std::max(0.0f, phase_gain);
}

float RCS_f743_v128b_ps(float wavenumber_k0, float radius_m, float theta_deg) {
  const float safe_radius_m = std::max(radius_m, 0.0f);
  const float safe_k0 = std::max(wavenumber_k0, 0.0f);
  if (safe_radius_m <= 0.0f || safe_k0 <= 0.0f) {
    return 0.0f;
  }
  const float theta_rad = oneq::internal::numerics::DegToRad(theta_deg);
  const float lambda_m = 2.0f * static_cast<float>(oneq::internal::numerics::kPi) / safe_k0;
  const float area_m2 = static_cast<float>(oneq::internal::numerics::kPi) * safe_radius_m * safe_radius_m;
  const float cos_theta = std::max(0.0f, std::cos(theta_rad));
  const float plate_rcs = (4.0f * static_cast<float>(oneq::internal::numerics::kPi) * area_m2 * area_m2) / std::max(lambda_m * lambda_m, 1.0e-9f);
  return oneq::internal::numerics::ClampNonNegative(plate_rcs * cos_theta * cos_theta);
}

LeafPhaseMatrices compute_leaf_phase_matrices(float leaf_size_m, float dielectric_constant_real,
                                              float incidence_deg, float scatter_deg) {
  const float safe_leaf_size_m = std::max(leaf_size_m, 0.0f);
  const float safe_eps = std::max(dielectric_constant_real, 1.0f);
  const float incidence_rad = oneq::internal::numerics::DegToRad(incidence_deg);
  const float scatter_rad = oneq::internal::numerics::DegToRad(scatter_deg);
  const float anisotropy = std::max(0.0f, safe_eps - 1.0f);
  const float geometry_gain = std::max(0.0f, std::cos(incidence_rad) * std::cos(scatter_rad));
  const float depolarization = std::max(0.0f, std::sin(std::fabs(incidence_rad - scatter_rad)));

  LeafPhaseMatrices matrices;
  matrices.m11 = safe_leaf_size_m * geometry_gain * (1.0f + 0.5f * anisotropy);
  matrices.m22 = safe_leaf_size_m * geometry_gain / (1.0f + 0.5f * anisotropy);
  matrices.m12 = safe_leaf_size_m * depolarization * 0.25f;
  matrices.m21 = matrices.m12;
  return matrices;
}

TreeScattererState InitTreeScatterer_AVX(const TreeScattererConfig& config) {
  TreeScattererState state;
  if (config.leaf_count == 0U) {
    return state;
  }

  state.leaf_azimuth_deg.resize(config.leaf_count, 0.0f);
  state.leaf_elevation_deg.resize(config.leaf_count, 0.0f);
  const float safe_canopy_radius_m = std::max(config.canopy_radius_m, 0.1f);
  const float safe_canopy_height_m = std::max(config.canopy_height_m, 0.1f);
  const float elevation_span_deg = std::min(85.0f, 35.0f + 10.0f * safe_canopy_height_m / safe_canopy_radius_m);
  for (std::size_t i = 0; i < config.leaf_count; ++i) {
    const float phase = static_cast<float>(i) / static_cast<float>(config.leaf_count);
    state.leaf_azimuth_deg[i] = phase * 360.0f;
    state.leaf_elevation_deg[i] = (phase - 0.5f) * elevation_span_deg;
  }
  return state;
}

void ComputeLeavesParamEq_ymm8r4(const TreeScattererState& state, float va, float vb,
                                 std::vector<float>* out_x_param,
                                 std::vector<float>* out_y_param) {
  if (out_x_param == nullptr || out_y_param == nullptr) {
    return;
  }
  out_x_param->assign(state.leaf_azimuth_deg.size(), 0.0f);
  out_y_param->assign(state.leaf_azimuth_deg.size(), 0.0f);
  const float safe_va = std::max(va, 0.0f);
  const float safe_vb = std::max(vb, 0.0f);
  for (std::size_t i = 0; i < state.leaf_azimuth_deg.size(); ++i) {
    const float az_rad = oneq::internal::numerics::DegToRad(state.leaf_azimuth_deg[i]);
    (*out_x_param)[i] = safe_va * std::cos(az_rad);
    (*out_y_param)[i] = safe_vb * std::sin(az_rad);
  }
}

}  // namespace rcs
}  // namespace internal
}  // namespace oneq
