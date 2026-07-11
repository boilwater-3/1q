#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "electro_optical_sensor/foundation/EosNoiseModel.h"
#include "electro_optical_sensor/foundation/EosPropagation.h"
#include "electro_optical_sensor/foundation/EosRadiometry.h"
#include "sbirs_sensor/foundation/SbirsNoiseModel.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"

namespace {

TEST(EosSbirsPhysicalCharacterizationTest, PlanckValuesAgreeOnlyInSharedValidDomain) {
  const float eos_value =
      electro_optical_sensor::foundation::radiometry::ComputePlanckRadiance(4.0f, 320.0f);
  const double sbirs_value = sbirs_sensor::foundation::ComputePlanckRadiance(4.0, 320.0);

  EXPECT_NEAR(static_cast<double>(eos_value), sbirs_value,
              std::max(1.0e-6, std::fabs(sbirs_value) * 5.0e-5));
  EXPECT_GT(electro_optical_sensor::foundation::radiometry::ComputePlanckRadiance(-1.0f, 320.0f),
            0.0f);
  EXPECT_DOUBLE_EQ(sbirs_sensor::foundation::ComputePlanckRadiance(-1.0, 320.0), 0.0);
}

TEST(EosSbirsPhysicalCharacterizationTest, ReceivedPowerUsesDifferentGeometryContracts) {
  constexpr double kApertureDiameterM = 0.5;
  constexpr double kApertureAreaM2 =
      3.14159265358979323846 * kApertureDiameterM * kApertureDiameterM * 0.25;
  const float eos_power = electro_optical_sensor::foundation::propagation::ComputeReceivedPowerW(
      100.0f, 2.0f, 1000.0f, static_cast<float>(kApertureAreaM2), 0.8f, 0.7f);
  const double sbirs_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      100.0, 2.0, 1000.0, kApertureDiameterM, 0.7, 0.8, 1.0);

  EXPECT_NEAR(sbirs_power / static_cast<double>(eos_power), 4.0 * 3.14159265358979323846, 1.0e-5);
}

TEST(EosSbirsPhysicalCharacterizationTest, NoiseFallbacksHaveDifferentContracts) {
  electro_optical_sensor::foundation::noise::BackgroundNoiseModelInputs eos_inputs;
  const electro_optical_sensor::foundation::noise::BackgroundNoiseStatistics eos_noise =
      electro_optical_sensor::foundation::noise::ComputeBackgroundNoiseStatistics(eos_inputs);
  EXPECT_FLOAT_EQ(eos_noise.equivalent_noise_power_w, 0.0f);
  EXPECT_FLOAT_EQ(electro_optical_sensor::foundation::noise::ComputeEffectiveSignalPowerW(
                      1.0f, 0.0f, eos_noise),
                  1.0f);

  sbirs_sensor::config::SbirsHardwareConfig sbirs_hardware;
  sbirs_hardware.noise_equivalent_power_w = 2.0e-12f;
  sbirs_hardware.background_radiance_w_sr_m2 = 0.0f;
  sbirs_hardware.optical_aperture_m = 0.0f;
  sbirs_hardware.detector_temperature_k = 0.0f;
  sbirs_hardware.integration_time_sec = 0.0f;
  sbirs_hardware.readout_noise_rms_w = 0.0f;
  const sbirs_sensor::foundation::SbirsNoiseStatistics sbirs_noise =
      sbirs_sensor::foundation::ComputeBackgroundNoiseStatistics(sbirs_hardware);
  EXPECT_DOUBLE_EQ(sbirs_noise.total_noise_w, 0.0);
  EXPECT_NEAR(sbirs_sensor::foundation::ResolveEffectiveNoiseW(sbirs_hardware, sbirs_noise),
              2.0e-12, 1.0e-18);
}

}  // namespace
