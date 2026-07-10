#include <gtest/gtest.h>

#include <cmath>

#include "common/atmosphere/StandardAtmosphere.h"

namespace oneq {
namespace common {
namespace atmosphere {
namespace {

// ISA 层边界定义在位势高度，API 接受几何高度。
// 低海拔时两者差异 < 0.2%，高海拔时差异增大（71km 处约 800m）。
// 使用相对宽松的容差反映几何→位势转换的固有偏差。
constexpr double kREarth = 6356766.0;

double GeopotentialOfGeometric(double h_m) { return kREarth * h_m / (kREarth + h_m); }

TEST(StandardAtmosphereTest, SeaLevelValues) {
  StandardAtmosphere isa;
  const auto state = isa.GetSeaLevelState();

  EXPECT_NEAR(state.temperature_k, 288.15f, 0.01f);
  EXPECT_NEAR(state.pressure_pa, 101325.0f, 1.0f);
  EXPECT_NEAR(state.pressure_hpa, 1013.25f, 0.01f);
  EXPECT_NEAR(state.density_kg_m3, 1.225f, 0.01f);
  EXPECT_NEAR(state.speed_of_sound_mps, 340.29f, 0.1f);
  EXPECT_FLOAT_EQ(state.altitude_m, 0.0f);
}

TEST(StandardAtmosphereTest, TroposphereLapseRate) {
  StandardAtmosphere isa;

  const auto state_5500 = isa.GetState(5500.0f);
  // H = 6356766*5500/(6356766+5500) = 5495.2, T = 288.15 - 0.0065*5495.2 = 252.43 K
  EXPECT_NEAR(state_5500.temperature_k, 252.43f, 0.1f);
  EXPECT_GT(state_5500.pressure_pa, 0.0f);
  EXPECT_GT(state_5500.density_kg_m3, 0.0f);

  const auto sl = isa.GetSeaLevelState();
  EXPECT_LT(state_5500.temperature_k, sl.temperature_k);
  EXPECT_LT(state_5500.pressure_pa, sl.pressure_pa);
  EXPECT_LT(state_5500.density_kg_m3, sl.density_kg_m3);
}

TEST(StandardAtmosphereTest, TropopauseIsConstant) {
  StandardAtmosphere isa;

  // 11km 几何 → H ≈ 10982m，仍在对流层梯度层内
  const auto state_11 = isa.GetState(11000.0f);
  const double h_11 = GeopotentialOfGeometric(11000.0);
  const double expected_t_11 = 288.15 - 0.0065 * h_11;
  EXPECT_NEAR(state_11.temperature_k, expected_t_11, 0.1f);

  // 15km 几何 → H ≈ 14970m，在等温层内
  const auto state_15 = isa.GetState(15000.0f);
  EXPECT_NEAR(state_15.temperature_k, 216.65f, 0.01f);

  // 20km 几何 → H ≈ 19937m，仍在等温层内
  const auto state_20 = isa.GetState(20000.0f);
  EXPECT_NEAR(state_20.temperature_k, 216.65f, 0.01f);

  // 气压随高度单调递减
  EXPECT_GT(state_11.pressure_pa, state_15.pressure_pa);
  EXPECT_GT(state_15.pressure_pa, state_20.pressure_pa);
}

TEST(StandardAtmosphereTest, StratosphereWarming) {
  StandardAtmosphere isa;

  // 32km 几何 → H ≈ 31837m，在平流层下层（lapse +0.001）
  const auto state_32 = isa.GetState(32000.0f);
  const double h_32 = GeopotentialOfGeometric(32000.0);
  const double expected_t_32 = 216.65 + 0.001 * (h_32 - 20000.0);
  EXPECT_NEAR(state_32.temperature_k, expected_t_32, 0.1f);

  // 47km 几何 → H ≈ 46641m，在平流层上层（lapse +0.0028）
  const auto state_47 = isa.GetState(47000.0f);
  const double h_47 = GeopotentialOfGeometric(47000.0);
  const double expected_t_47 = 228.65 + 0.0028 * (h_47 - 32000.0);
  EXPECT_NEAR(state_47.temperature_k, expected_t_47, 0.1f);
}

TEST(StandardAtmosphereTest, MesosphereCooling) {
  StandardAtmosphere isa;

  // 71km 几何 → H ≈ 70200m，在中间层下层（lapse -0.0028）
  const auto state_71 = isa.GetState(71000.0f);
  const double h_71 = GeopotentialOfGeometric(71000.0);
  const double expected_t_71 = 270.65 - 0.0028 * (h_71 - 51000.0);
  EXPECT_NEAR(state_71.temperature_k, expected_t_71, 0.2f);

  // 86km 几何 → H ≈ 84852m ≈ ISA 上限
  const auto state_86 = isa.GetState(86000.0f);
  const double h_86 = GeopotentialOfGeometric(86000.0);
  const double expected_t_86 = 214.65 - 0.002 * (h_86 - 71000.0);
  EXPECT_NEAR(state_86.temperature_k, expected_t_86, 0.2f);
}

TEST(StandardAtmosphereTest, DensityConsistency) {
  StandardAtmosphere isa;
  const auto state = isa.GetState(5000.0f);

  constexpr double kRSpecific = 287.05287;
  const double expected_density =
      static_cast<double>(state.pressure_pa) / (kRSpecific * static_cast<double>(state.temperature_k));
  EXPECT_NEAR(static_cast<double>(state.density_kg_m3), expected_density, 0.01);
}

TEST(StandardAtmosphereTest, SpeedOfSoundConsistency) {
  StandardAtmosphere isa;
  const auto state = isa.GetState(10000.0f);

  constexpr double kGamma = 1.4;
  constexpr double kRSpecific = 287.05287;
  const double expected_a =
      std::sqrt(kGamma * kRSpecific * static_cast<double>(state.temperature_k));
  EXPECT_NEAR(static_cast<double>(state.speed_of_sound_mps), expected_a, 0.1);
}

TEST(StandardAtmosphereTest, MonotonicPressureDecay) {
  StandardAtmosphere isa;
  float prev_pressure = 1.0e10f;

  const float altitudes[] = {0.0f, 1000.0f, 5000.0f, 11000.0f, 20000.0f,
                             32000.0f, 47000.0f, 51000.0f, 71000.0f, 86000.0f};

  for (const float alt : altitudes) {
    const auto state = isa.GetState(alt);
    EXPECT_LT(state.pressure_pa, prev_pressure)
        << "Pressure not monotonically decreasing at " << alt << " m";
    EXPECT_GT(state.pressure_pa, 0.0f) << "Pressure negative at " << alt << " m";
    prev_pressure = state.pressure_pa;
  }
}

TEST(StandardAtmosphereTest, AltitudeEcho) {
  StandardAtmosphere isa;
  const float test_alt = 12345.0f;
  const auto state = isa.GetState(test_alt);
  EXPECT_FLOAT_EQ(state.altitude_m, test_alt);
}

TEST(StandardAtmosphereTest, PressureHpaConsistency) {
  StandardAtmosphere isa;
  const auto state = isa.GetState(8000.0f);
  EXPECT_NEAR(state.pressure_hpa, state.pressure_pa / 100.0f, 0.01f);
}

TEST(StandardAtmosphereTest, NegativeAltitudeClamped) {
  StandardAtmosphere isa;
  const auto state = isa.GetState(-100.0f);
  const auto sl = isa.GetSeaLevelState();

  EXPECT_NEAR(state.temperature_k, sl.temperature_k, 0.01f);
  EXPECT_NEAR(state.pressure_pa, sl.pressure_pa, 0.01f);
}

TEST(StandardAtmosphereTest, AboveIsaCeilingClamped) {
  StandardAtmosphere isa;

  // 87200m 几何 → H ≈ 86027m ≈ ISA 上限 (86000m 位势)
  // 这个高度恰好在位势高度 86000m 附近
  const auto state_at_ceiling = isa.GetState(87200.0f);

  // 200000m 几何 → H ≈ 193844m，远超 ISA 上限，应钳位
  const auto state_above = isa.GetState(200000.0f);

  // 两个结果的温度应非常接近（都在顶层/钳位到顶层）
  EXPECT_NEAR(state_above.temperature_k, state_at_ceiling.temperature_k, 0.5f);
  EXPECT_NEAR(state_above.pressure_pa, state_at_ceiling.pressure_pa, 0.1f);
  EXPECT_FLOAT_EQ(state_above.altitude_m, 200000.0f);
}

TEST(StandardAtmosphereTest, PressureHpaAtSeaLevel) {
  StandardAtmosphere isa;
  const auto state = isa.GetSeaLevelState();
  EXPECT_NEAR(state.pressure_hpa, 1013.25f, 0.01f);
}

TEST(StandardAtmosphereTest, ISAReferenceValues) {
  StandardAtmosphere isa;

  // 使用较宽容差：ISA 断点定义在位势高度，API 接受几何高度
  const struct {
    float geometric_alt_m;
    float tolerance;
  } checkpoints[] = {
      {0.0f, 0.01f},     // 海平面：几何=位势
      {11000.0f, 0.15f}, // 对流层顶：位势 10982m
      {20000.0f, 0.01f}, // 等温层：位势 19937m，T ≈ 216.65K
      {32000.0f, 0.2f},  // 平流层下层：位势 31837m
      {47000.0f, 1.0f},  // 平流层上层：位势 46641m
      {51000.0f, 0.5f},  // 等温层：位势 50594m
      {71000.0f, 2.5f},  // 中间层：位势 70200m
  };

  for (const auto& cp : checkpoints) {
    const auto state = isa.GetState(cp.geometric_alt_m);
    const double H = GeopotentialOfGeometric(cp.geometric_alt_m);

    // 手算期望温度（根据位势高度落入的层）
    double expected_t = 0.0;
    if (H < 11000.0) {
      expected_t = 288.15 - 0.0065 * H;
    } else if (H < 20000.0) {
      expected_t = 216.65;
    } else if (H < 32000.0) {
      expected_t = 216.65 + 0.001 * (H - 20000.0);
    } else if (H < 47000.0) {
      expected_t = 228.65 + 0.0028 * (H - 32000.0);
    } else if (H < 51000.0) {
      expected_t = 270.65;
    } else if (H < 71000.0) {
      expected_t = 270.65 - 0.0028 * (H - 51000.0);
    } else {
      expected_t = 214.65 - 0.002 * (H - 71000.0);
    }

    EXPECT_NEAR(state.temperature_k, expected_t, cp.tolerance)
        << "Temperature mismatch at " << cp.geometric_alt_m << " m geometric ("
        << H << " m geopotential)";
  }
}

}  // namespace
}  // namespace atmosphere
}  // namespace common
}  // namespace oneq
