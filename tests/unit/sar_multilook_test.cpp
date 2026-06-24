#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "sar/imaging/SarMultilook.h"
#include "sar/signal/SarFft.h"

// SAR 多视降斑后处理测试(契约 multilook_processing.md §4-6)。
// 聚焦后图像域非相干多视:分块 + 幅度/功率平均。与聚焦算法解耦(消费任意 ComplexMatrix)。
namespace sar {
namespace imaging {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

// 构造全零复矩阵(rows × cols)。
signal::ComplexMatrix MakeZeroImage(std::size_t rows, std::size_t cols) {
  signal::ComplexMatrix image;
  image.rows = rows;
  image.cols = cols;
  image.values.assign(rows * cols, signal::ComplexSample(0.0, 0.0));
  return image;
}

// 确定性伪随机复数生成(模拟相干斑:随机相位 + 单位幅度)。
// 用于降斑量化测试——多视应使随机相位的斑噪声方差下降。
signal::ComplexSample NextSpeckleSample(std::uint64_t* state) {
  *state += UINT64_C(0x9e3779b97f4a7c15);
  std::uint64_t value = *state;
  value = (value ^ (value >> 30U)) * UINT64_C(0xbf58476d1ce4e5b9);
  value = (value ^ (value >> 27U)) * UINT64_C(0x94d049bb133111eb);
  value ^= value >> 31U;
  // 均匀分布 [0,1)。
  const double u = (static_cast<double>(value >> 11U) + 0.5) / 9007199254740992.0;
  // 相干斑:随机相位 + 单位幅度(完全发展斑噪声的标准模型)。
  const double phase = 2.0 * kPi * u;
  return signal::ComplexSample(std::cos(phase), std::sin(phase));
}

// 构造含相干斑的合成图像(均匀区域,随机相位 + 单位幅度)。
signal::ComplexMatrix MakeSpeckleImage(std::size_t rows, std::size_t cols, std::uint64_t seed) {
  signal::ComplexMatrix image = MakeZeroImage(rows, cols);
  std::uint64_t state = seed;
  for (std::size_t i = 0U; i < image.values.size(); ++i) {
    image.values[i] = NextSpeckleSample(&state);
  }
  return image;
}

// 计算实数幅度图的均值与标准差。
struct AmplitudeStats {
  double mean{0.0};
  double stddev{0.0};
};

AmplitudeStats ComputeAmplitudeStats(const RealMatrix& image) {
  AmplitudeStats stats;
  if (image.values.empty()) {
    return stats;
  }
  double sum = 0.0;
  for (double v : image.values) {
    sum += v;
  }
  stats.mean = sum / static_cast<double>(image.values.size());
  double sum_sq = 0.0;
  for (double v : image.values) {
    sum_sq += (v - stats.mean) * (v - stats.mean);
  }
  stats.stddev = std::sqrt(sum_sq / static_cast<double>(image.values.size()));
  return stats;
}

// ── 不变量 1:单视退化 — looks=(1,1) 输出 = 输入逐像素幅度图 ──
TEST(SarMultilookTest, SingleLookDegeneratesToAmplitudeImage) {
  signal::ComplexMatrix image = MakeZeroImage(4U, 4U);
  image(0, 0) = signal::ComplexSample(3.0, 4.0);   // |z| = 5
  image(1, 2) = signal::ComplexSample(0.0, -2.0);  // |z| = 2
  image(3, 1) = signal::ComplexSample(1.0, 1.0);   // |z| = √2

  MultilookConfig config;
  config.azimuth_looks = 1U;
  config.range_looks = 1U;
  config.average_type = MultilookAverageType::kAmplitude;

  RealMatrix output;
  ASSERT_TRUE(ApplyMultilook(config, image, &output));
  ASSERT_EQ(output.rows, 4U);
  ASSERT_EQ(output.cols, 4U);

  // 逐像素幅度图(尺寸不变)。
  EXPECT_NEAR(output(0, 0), 5.0, 1e-12);
  EXPECT_NEAR(output(1, 2), 2.0, 1e-12);
  EXPECT_NEAR(output(3, 1), std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(output(0, 1), 0.0, 1e-12);  // 零像素幅度 0
}

// ── 不变量 2:尺寸收缩 — looks=(N,M) 输出 = (⌊rows/N⌋ × ⌊cols/M⌋)──
// looks 是降采样步长(每视像素数):每 N×M 块 → 1 输出像素。
TEST(SarMultilookTest, OutputSizeShrinksByLookCount) {
  signal::ComplexMatrix image = MakeSpeckleImage(10U, 8U, 42U);

  // looks=(2,2):⌊10/2⌋×⌊8/2⌋ = 5×4。
  MultilookConfig config22;
  config22.azimuth_looks = 2U;
  config22.range_looks = 2U;
  RealMatrix out22;
  ASSERT_TRUE(ApplyMultilook(config22, image, &out22));
  EXPECT_EQ(out22.rows, 5U);
  EXPECT_EQ(out22.cols, 4U);

  // looks=(5,4):⌊10/5⌋×⌊8/4⌋ = 2×2。
  MultilookConfig config54;
  config54.azimuth_looks = 5U;
  config54.range_looks = 4U;
  RealMatrix out54;
  ASSERT_TRUE(ApplyMultilook(config54, image, &out54));
  EXPECT_EQ(out54.rows, 2U);
  EXPECT_EQ(out54.cols, 2U);

  // 非整除(10/3=3 余 1):输出 3×2,丢弃余数。
  MultilookConfig config33;
  config33.azimuth_looks = 3U;
  config33.range_looks = 3U;
  RealMatrix out33;
  ASSERT_TRUE(ApplyMultilook(config33, image, &out33));
  EXPECT_EQ(out33.rows, 3U);  // ⌊10/3⌋=3
  EXPECT_EQ(out33.cols, 2U);  // ⌊8/3⌋=2
}

// ── 不变量 3:降斑量化 — looks 增大,幅度标准差按 1/√ENL 下降 ──
// 完全发展斑噪声:单视幅度标准差/均值 ≈ 0.523(瑞利分布)。
// 多视 N 视后,标准差/均值 ≈ 0.523/√N(等效视数 ENL=N)。
TEST(SarMultilookTest, SpeckleStddevDecreasesBySqrtENL) {
  // 大尺寸均匀斑噪声图像,保证统计显著性。
  signal::ComplexMatrix image = MakeSpeckleImage(80U, 80U, 2026U);

  // 单视(looks=1,1):全图幅度统计。
  MultilookConfig single;
  single.azimuth_looks = 1U;
  single.range_looks = 1U;
  single.average_type = MultilookAverageType::kAmplitude;
  RealMatrix single_out;
  ASSERT_TRUE(ApplyMultilook(single, image, &single_out));
  const AmplitudeStats single_stats = ComputeAmplitudeStats(single_out);
  ASSERT_GT(single_stats.stddev, 0.0);

  // 4 视(looks=4,4):16 倍等效视数,标准差应降至 ~1/4。
  MultilookConfig multi4;
  multi4.azimuth_looks = 4U;
  multi4.range_looks = 4U;
  multi4.average_type = MultilookAverageType::kAmplitude;
  RealMatrix multi4_out;
  ASSERT_TRUE(ApplyMultilook(multi4, image, &multi4_out));
  const AmplitudeStats multi4_stats = ComputeAmplitudeStats(multi4_out);

  // 单视变异系数 CV = stddev/mean。
  const double single_cv = single_stats.stddev / single_stats.mean;
  const double multi4_cv = multi4_stats.stddev / multi4_stats.mean;

  // 4×4=16 视,CV 应降至单视的 ~1/4(1/√16=0.25)。允许统计涨落,验证显著下降。
  EXPECT_LT(multi4_cv, single_cv * 0.5) << "multi4_cv=" << multi4_cv
                                         << " single_cv=" << single_cv;
}

// ── 平均类型:幅度 vs 功率平均均收敛 ──
TEST(SarMultilookTest, PowerAverageProducesFiniteOutput) {
  signal::ComplexMatrix image = MakeSpeckleImage(8U, 8U, 100U);

  MultilookConfig amp_config;
  amp_config.azimuth_looks = 2U;
  amp_config.range_looks = 2U;
  amp_config.average_type = MultilookAverageType::kAmplitude;
  RealMatrix amp_out;
  ASSERT_TRUE(ApplyMultilook(amp_config, image, &amp_out));

  MultilookConfig pow_config;
  pow_config.azimuth_looks = 2U;
  pow_config.range_looks = 2U;
  pow_config.average_type = MultilookAverageType::kPower;
  RealMatrix pow_out;
  ASSERT_TRUE(ApplyMultilook(pow_config, image, &pow_out));

  // 两者尺寸一致。
  ASSERT_EQ(amp_out.rows, pow_out.rows);
  ASSERT_EQ(amp_out.cols, pow_out.cols);

  // 功率平均通常 ≥ 幅度平均(功率域对高幅度像素权重更大)。
  // 主要验证:两者都有限、非负。
  for (std::size_t i = 0U; i < amp_out.values.size(); ++i) {
    EXPECT_TRUE(std::isfinite(amp_out.values[i]));
    EXPECT_TRUE(std::isfinite(pow_out.values[i]));
    EXPECT_GE(amp_out.values[i], 0.0);
    EXPECT_GE(pow_out.values[i], 0.0);
  }
}

// ── 不变量 4:峰值保持 — 点目标多视后峰值像素可定位 ──
TEST(SarMultilookTest, PointTargetPeakPreserved) {
  // 4×4 图像,中心 (1,1) 块有强点目标。
  signal::ComplexMatrix image = MakeZeroImage(4U, 4U);
  image(1, 1) = signal::ComplexSample(100.0, 0.0);  // 强点目标,其余为 0。

  // looks=(2,2):每视 2×2 块。点目标 (1,1) 落在视 (0,0) 块 [0,2)×[0,2)。
  MultilookConfig config;
  config.azimuth_looks = 2U;
  config.range_looks = 2U;
  RealMatrix output;
  ASSERT_TRUE(ApplyMultilook(config, image, &output));
  ASSERT_EQ(output.rows, 2U);
  ASSERT_EQ(output.cols, 2U);

  // 峰值应在视 (0,0)(含点目标的块),幅度 = 100/4 = 25(块内平均)。
  double peak = 0.0;
  std::size_t peak_az = 0U;
  std::size_t peak_rg = 0U;
  for (std::size_t r = 0U; r < output.rows; ++r) {
    for (std::size_t c = 0U; c < output.cols; ++c) {
      if (output(r, c) > peak) {
        peak = output(r, c);
        peak_az = r;
        peak_rg = c;
      }
    }
  }
  EXPECT_EQ(peak_az, 0U);
  EXPECT_EQ(peak_rg, 0U);
  EXPECT_NEAR(peak, 25.0, 1e-9);  // 100/(2×2 块)
}

// ── 确定性 — 相同输入相同输出 ──
TEST(SarMultilookTest, Deterministic) {
  signal::ComplexMatrix image = MakeSpeckleImage(6U, 6U, 7U);
  MultilookConfig config;
  config.azimuth_looks = 3U;
  config.range_looks = 2U;

  RealMatrix first;
  RealMatrix second;
  ASSERT_TRUE(ApplyMultilook(config, image, &first));
  ASSERT_TRUE(ApplyMultilook(config, image, &second));
  EXPECT_EQ(first.values, second.values);
}

// ── 拒绝路径 ──
TEST(SarMultilookTest, RejectsNullOutput) {
  signal::ComplexMatrix image = MakeZeroImage(4U, 4U);
  MultilookConfig config;
  EXPECT_FALSE(ApplyMultilook(config, image, nullptr));
}

TEST(SarMultilookTest, RejectsEmptyImage) {
  signal::ComplexMatrix empty;
  MultilookConfig config;
  RealMatrix output;
  EXPECT_FALSE(ApplyMultilook(config, empty, &output));
}

TEST(SarMultilookTest, RejectsZeroLooks) {
  signal::ComplexMatrix image = MakeZeroImage(4U, 4U);
  MultilookConfig config;
  config.azimuth_looks = 0U;
  RealMatrix output;
  EXPECT_FALSE(ApplyMultilook(config, image, &output));

  config.azimuth_looks = 1U;
  config.range_looks = 0U;
  EXPECT_FALSE(ApplyMultilook(config, image, &output));
}

TEST(SarMultilookTest, RejectsLooksExceedingImageSize) {
  signal::ComplexMatrix image = MakeZeroImage(4U, 4U);
  MultilookConfig config;
  config.azimuth_looks = 5U;  // 超过 rows=4
  config.range_looks = 1U;
  RealMatrix output;
  EXPECT_FALSE(ApplyMultilook(config, image, &output));

  config.azimuth_looks = 1U;
  config.range_looks = 5U;  // 超过 cols=4
  EXPECT_FALSE(ApplyMultilook(config, image, &output));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
