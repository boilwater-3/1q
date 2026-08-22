// 验收旁路 MTI/MTD 核：零多普勒、PRF/2、8 路守恒、非法输入。

#include "common/radar/MtiMtdAcceptanceBank.h"

#include <array>
#include <cmath>

#include "gtest/gtest.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

constexpr double kPrfHz = 400.0;
constexpr double kCarrierHz = 3.0e9;

TEST(CommonMtiMtdAcceptanceBankTest, ZeroDopplerMtiNearFloor) {
  MtiMtdAcceptanceInput input;
  input.echo_power_w = 1.0e-12;
  input.thermal_noise_power_w = 1.0e-15;
  input.clutter_power_w = 2.0e-15;
  input.two_way_doppler_shift_hz = 0.0;
  input.prf_hz = kPrfHz;
  input.center_frequency_hz = kCarrierHz;
  MtiMtdAcceptanceResult result;
  ASSERT_TRUE(TryResolveMtiMtdAcceptanceBank(input, &result));
  EXPECT_LT(result.mti_gain_db, -100.0);
  double target_sum = 0.0;
  for (double value : result.target_w) {
    target_sum += value;
    EXPECT_NEAR(value, 0.0, 1.0e-18);
  }
  EXPECT_NEAR(target_sum, 0.0, 1.0e-18);
}

TEST(CommonMtiMtdAcceptanceBankTest, HalfPrfMtiNearSixDb) {
  MtiMtdAcceptanceInput input;
  input.echo_power_w = 4.0e-12;
  input.thermal_noise_power_w = 1.0e-15;
  input.clutter_power_w = 0.0;
  input.two_way_doppler_shift_hz = 0.5 * kPrfHz;
  input.prf_hz = kPrfHz;
  input.center_frequency_hz = kCarrierHz;
  MtiMtdAcceptanceResult result;
  ASSERT_TRUE(TryResolveMtiMtdAcceptanceBank(input, &result));
  EXPECT_NEAR(result.mti_gain_db, 10.0 * std::log10(4.0), 1.0e-9);
  EXPECT_EQ(result.selected_channel, 4U);
  EXPECT_NEAR(result.mtd_gain_db, 0.0, 1.0e-9);
  double target_sum = 0.0;
  for (double value : result.target_w) {
    target_sum += value;
  }
  EXPECT_NEAR(target_sum, input.echo_power_w * 4.0, 1.0e-18);
}

TEST(CommonMtiMtdAcceptanceBankTest, EightChannelPowerConservation) {
  MtiMtdAcceptanceInput input;
  input.echo_power_w = 1.0e-12;
  input.thermal_noise_power_w = 8.0e-15;
  input.clutter_power_w = 3.0e-14;
  input.two_way_doppler_shift_hz = 50.0;
  input.prf_hz = kPrfHz;
  input.center_frequency_hz = kCarrierHz;
  MtiMtdAcceptanceResult result;
  ASSERT_TRUE(TryResolveMtiMtdAcceptanceBank(input, &result));
  double noise_sum = 0.0;
  double clutter_sum = 0.0;
  for (std::size_t k = 0U; k < kMtiMtdChannelCount; ++k) {
    noise_sum += result.noise_w[k];
    clutter_sum += result.clutter_w[k];
    EXPECT_NEAR(result.noise_w[k], input.thermal_noise_power_w * kMtiNoiseFactor / 8.0, 1.0e-24);
  }
  EXPECT_NEAR(noise_sum, input.thermal_noise_power_w * kMtiNoiseFactor, 1.0e-24);
  EXPECT_NEAR(clutter_sum, result.mti_residual_clutter_w, 1.0e-18);
  EXPECT_LT(result.mti_residual_clutter_w, input.clutter_power_w);
  EXPECT_FALSE(result.has_jam_channels);
}

TEST(CommonMtiMtdAcceptanceBankTest, InvalidInputLeavesOutputUntouched) {
  MtiMtdAcceptanceResult result;
  result.mti_gain_db = 123.0;
  result.selected_channel = 7U;
  MtiMtdAcceptanceInput input;
  input.echo_power_w = 1.0;
  input.thermal_noise_power_w = 1.0;
  input.clutter_power_w = 1.0;
  input.prf_hz = 0.0;
  input.center_frequency_hz = kCarrierHz;
  EXPECT_FALSE(TryResolveMtiMtdAcceptanceBank(input, &result));
  EXPECT_DOUBLE_EQ(result.mti_gain_db, 123.0);
  EXPECT_EQ(result.selected_channel, 7U);
  EXPECT_FALSE(TryResolveMtiMtdAcceptanceBank(input, nullptr));
}

TEST(CommonMtiMtdAcceptanceBankTest, JamTonesSplitWithoutEqualShare) {
  MtiMtdInterferenceTone tone;
  tone.doppler_hz = 0.5 * kPrfHz;
  tone.power_w = 1.0e-13;
  MtiMtdAcceptanceInput input;
  input.echo_power_w = 0.0;
  input.thermal_noise_power_w = 1.0e-15;
  input.clutter_power_w = 0.0;
  input.two_way_doppler_shift_hz = 0.0;
  input.prf_hz = kPrfHz;
  input.center_frequency_hz = kCarrierHz;
  input.tones = &tone;
  input.tone_count = 1U;
  MtiMtdAcceptanceResult result;
  ASSERT_TRUE(TryResolveMtiMtdAcceptanceBank(input, &result));
  ASSERT_TRUE(result.has_jam_channels);
  EXPECT_NEAR(result.mti_residual_jam_w, tone.power_w * 4.0, 1.0e-20);
  EXPECT_NEAR(result.jam_w[4], tone.power_w * 4.0, 1.0e-20);
  for (std::size_t k = 0U; k < kMtiMtdChannelCount; ++k) {
    if (k != 4U) {
      EXPECT_NEAR(result.jam_w[k], 0.0, 1.0e-20);
    }
  }
}

}  // namespace
}  // namespace radar
}  // namespace common
}  // namespace oneq
