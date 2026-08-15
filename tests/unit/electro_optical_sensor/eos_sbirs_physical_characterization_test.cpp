#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "electro_optical_sensor/foundation/EosNoiseModel.h"
#include "electro_optical_sensor/foundation/EosPropagation.h"
#include "electro_optical_sensor/foundation/EosRadiometry.h"
#include "sbirs_sensor/foundation/SbirsNoiseModel.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"

namespace {

TEST(EosSbirsPhysicalCharacterizationTest, PlanckStaysEosOnlyAfterSbirsSignatureChange) {
  // 冻结结论（contract.md 跨模块物理基元复用表）：SBIRS 目标签名改为调用方提供
  // 辐射强度（W/sr）后不再实现 Planck 换算，本测试只保留 EOS 侧 Planck 特征。
  const float cool = electro_optical_sensor::foundation::radiometry::ComputePlanckRadiance(
      4.0f, 280.0f);
  const float warm = electro_optical_sensor::foundation::radiometry::ComputePlanckRadiance(
      4.0f, 340.0f);
  EXPECT_GT(warm, cool);
  EXPECT_GT(electro_optical_sensor::foundation::radiometry::ComputePlanckRadiance(-1.0f, 320.0f),
            0.0f);
}

TEST(EosSbirsPhysicalCharacterizationTest, ReceivedPowerUsesDifferentGeometryContracts) {
  constexpr double kApertureDiameterM = 0.5;
  constexpr double kApertureAreaM2 =
      3.14159265358979323846 * kApertureDiameterM * kApertureDiameterM * 0.25;
  const float eos_power = electro_optical_sensor::foundation::propagation::ComputeReceivedPowerW(
      100.0f, 2.0f, 1000.0f, static_cast<float>(kApertureAreaM2), 0.8f, 0.7f);
  // SBIRS 辐射强度口径：I_t = 200 W/sr（等价旧 radiance·area = 100·2）。
  const double sbirs_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      200.0, 1000.0, kApertureDiameterM, 0.7, 0.8, 1.0);

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
