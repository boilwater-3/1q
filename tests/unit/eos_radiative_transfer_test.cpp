/**
 * @file eos_radiative_transfer_unit_test.cpp
 * @brief 验证 EOS 辐射传输模型的可替换行为。
 */

#include <gtest/gtest.h>

#include "electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace foundation {
namespace radiative_transfer {
namespace {

TEST(EosRadiativeTransferTest, TransmittanceDecreasesWithPathLength) {
  RadiativeTransferInputs short_path_inputs;
  short_path_inputs.model = config::RadiativeTransferModel::kDerivedBeerLambert;
  short_path_inputs.base_transmittance = 0.82f;
  short_path_inputs.cloud_coverage_ratio = 0.2f;
  short_path_inputs.path_length_m = 1200.0f;

  RadiativeTransferInputs long_path_inputs = short_path_inputs;
  long_path_inputs.path_length_m = 5200.0f;

  const RadiativeTransferResult short_path_result =
      EvaluateRadiativeTransfer(short_path_inputs);
  const RadiativeTransferResult long_path_result =
      EvaluateRadiativeTransfer(long_path_inputs);

  EXPECT_GT(short_path_result.transmittance, long_path_result.transmittance);
}

TEST(EosRadiativeTransferTest, AdaptiveModelHasHigherExtinctionThanBaseline) {
  RadiativeTransferInputs baseline_inputs;
  baseline_inputs.model = config::RadiativeTransferModel::kDerivedBeerLambert;
  baseline_inputs.base_transmittance = 0.80f;
  baseline_inputs.cloud_coverage_ratio = 0.45f;
  baseline_inputs.path_length_m = 3000.0f;
  baseline_inputs.aerosol_density_factor = 1.5f;
  baseline_inputs.turbulence_factor = 1.4f;

  RadiativeTransferInputs adaptive_inputs = baseline_inputs;
  adaptive_inputs.model = config::RadiativeTransferModel::kAdaptivePathRadiance;

  const RadiativeTransferResult baseline_result =
      EvaluateRadiativeTransfer(baseline_inputs);
  const RadiativeTransferResult adaptive_result =
      EvaluateRadiativeTransfer(adaptive_inputs);

  EXPECT_GT(adaptive_result.total_extinction_coeff_per_m,
            baseline_result.total_extinction_coeff_per_m);
  EXPECT_GT(adaptive_result.path_radiance_penalty_scale, 1.0f);
}

}  // namespace
}  // namespace radiative_transfer
}  // namespace foundation
}  // namespace electro_optical_sensor
