#include "sar/imaging/SarMultilook.h"

#include <cmath>

namespace sar {
namespace imaging {

double& RealMatrix::operator()(std::size_t row, std::size_t col) {
  return values[row * cols + col];
}

const double& RealMatrix::operator()(std::size_t row, std::size_t col) const {
  return values[row * cols + col];
}

bool ApplyMultilook(const MultilookConfig& config, const signal::ComplexMatrix& focused_image,
                    RealMatrix* output) {
  if (output == nullptr) {
    return false;
  }
  *output = RealMatrix{};

  // 拒绝路径(契约 §4.3):空图像。
  if (focused_image.rows == 0U || focused_image.cols == 0U ||
      focused_image.values.size() != focused_image.rows * focused_image.cols) {
    return false;
  }
  // 拒绝路径:零视数。
  if (config.azimuth_looks == 0U || config.range_looks == 0U) {
    return false;
  }
  // 拒绝路径:视数超过图像尺寸(无法分块,每视至少 1 像素)。
  if (config.azimuth_looks > focused_image.rows || config.range_looks > focused_image.cols) {
    return false;
  }

  // looks = 每视包含的像素数(降采样步长)。每 azimuth_looks × range_looks 像素块 → 1 输出像素。
  // 单视退化(looks=1,1):每像素自成一视 → 输出 = 输入逐像素幅度图(尺寸不变,契约 §4.2 不变量 1)。
  // 多视(looks=N,M):N×M 像素块平均成 1 像素 → 输出尺寸 = (⌊rows/N⌋, ⌊cols/M⌋),降采样降斑。
  const std::size_t out_rows = focused_image.rows / config.azimuth_looks;
  const std::size_t out_cols = focused_image.cols / config.range_looks;
  if (out_rows == 0U || out_cols == 0U) {
    return false;
  }

  // 输出尺寸 = 降采样后尺寸(每视块产生 1 像素)。
  output->rows = out_rows;
  output->cols = out_cols;
  output->values.assign(out_rows * out_cols, 0.0);

  const std::size_t pixels_per_block = config.azimuth_looks * config.range_looks;
  const double inv_count = 1.0 / static_cast<double>(pixels_per_block);

  for (std::size_t az = 0U; az < out_rows; ++az) {
    for (std::size_t rg = 0U; rg < out_cols; ++rg) {
      // 该输出像素覆盖的输入块 [az*looks_az, az*looks_az+looks_az) ×
      //                         [rg*looks_rg, rg*looks_rg+looks_rg)。
      double accumulator = 0.0;
      for (std::size_t r = 0U; r < config.azimuth_looks; ++r) {
        for (std::size_t c = 0U; c < config.range_looks; ++c) {
          const signal::ComplexSample sample =
              focused_image(az * config.azimuth_looks + r, rg * config.range_looks + c);
          const double magnitude = std::abs(sample);
          if (config.average_type == MultilookAverageType::kPower) {
            accumulator += magnitude * magnitude;  // 功率 |z|²
          } else {
            accumulator += magnitude;  // 幅度 |z|
          }
        }
      }

      double pixel = accumulator * inv_count;  // 均值
      if (config.average_type == MultilookAverageType::kPower) {
        pixel = std::sqrt(pixel);  // 功率平均后开方回幅度域
      }
      if (!std::isfinite(pixel)) {
        return false;  // 无 NaN/Inf 保证
      }
      (*output)(az, rg) = pixel;
    }
  }
  return true;
}

}  // namespace imaging
}  // namespace sar
