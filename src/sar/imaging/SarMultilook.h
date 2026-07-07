/**
 * @file SarMultilook.h
 * @brief SAR 聚焦后图像域非相干多视降斑后处理。
 *
 * 对聚焦复图像按 azimuth_looks × range_looks 非重叠分块,每视取幅度/功率平均,输出实数幅度图。
 * 以 N 倍分辨率为代价换取相干斑标准差降至 1/√N。与聚焦算法解耦(消费任意 ComplexMatrix)。
 *
 * 契约:multilook_processing.md。阶段 A 评估:聚焦后图像域多视是收益比最优路径
 * (multilook_value_assessment.md §4),RD 域/raw 域多视冻结。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_MULTILOOK_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_MULTILOOK_H_

#include <cstddef>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 非相干平均类型。
 */
enum class MultilookAverageType {
  kAmplitude = 0,  /**< 幅度平均:mean(|z|)。对点目标峰值保持较好(默认)。 */
  kPower = 1,      /**< 功率平均:sqrt(mean(|z|²))。功率域方差更小,降斑略优。 */
};

/**
 * @brief 多视配置。
 */
struct MultilookConfig {
  std::size_t azimuth_looks{1U};  /**< 方位视数(行向分块数),≥1 */
  std::size_t range_looks{1U};    /**< 距离视数(列向分块数),≥1 */
  MultilookAverageType average_type{MultilookAverageType::kAmplitude};
};

/**
 * @brief 实数幅度图(行主序)。多视产物是非相干叠加后的幅度图,无相位。
 *
 * 与 ComplexMatrix 区分:多视产物语义上是检测后的幅度/强度图,非复图像。
 */
struct RealMatrix {
  std::size_t rows{0U};
  std::size_t cols{0U};
  std::vector<double> values{};

  double& operator()(std::size_t row, std::size_t col);
  const double& operator()(std::size_t row, std::size_t col) const;
};

/**
 * @brief 聚焦后图像域非相干多视降斑。
 *
 * 对聚焦复图像按 azimuth_looks × range_looks 非重叠分块(向下取整丢弃余数),
 * 每视(块)取幅度/功率平均,输出实数幅度图。算法复杂度 O(rows×cols),无 FFT。
 *
 * 不变量(契约 §4.2):
 * 1. 单视退化:looks=(1,1) → 输出 = 输入逐像素幅度图(尺寸不变)。
 * 2. 尺寸收缩:looks=(N,M) → 输出尺寸 = (⌊rows/N⌋, ⌊cols/M⌋)。
 * 3. 降斑单调性:looks 越大,均匀区域幅度标准差按 1/√ENL 下降。
 * 4. 峰值保持:点目标多视后峰值像素可定位。
 * 5. 确定性 + 无 NaN/Inf。
 *
 * @param config 视数 + 平均类型。
 * @param focused_image 聚焦复图像(任意聚焦器产出)。
 * @param output 实数幅度图。
 * @return 成功则 true;空指针/空图像/零视数/视数超尺寸则 false(契约 §4.3)。
 */
bool ApplyMultilook(const MultilookConfig& config, const signal::ComplexMatrix& focused_image,
                    RealMatrix* output);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_MULTILOOK_H_
