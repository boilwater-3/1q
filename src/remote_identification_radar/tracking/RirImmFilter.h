/**
 * @file RirImmFilter.h
 * @brief RIR 交互多模型（IMM）滤波器包装（阶段 2-T N4）。
 *
 * 数值核心为 common/estimation/ImmFilter<6,3>（AR 侧同名头仅为向后兼容外观，
 * RIR 不引 AR 头）；包装层负责：按模型数生成默认 CV 模型集（对数等距
 * 过程噪声差异系数，口径同 AR BuildDefaultImmNoiseDiffCoeffs）、默认
 * 转移概率矩阵（对角 0.95）与均匀初始权重，并持有模型预测/更新器实例。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_IMM_FILTER_H_
#define REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_IMM_FILTER_H_

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <vector>

#include "common/estimation/ImmFilter.h"
#include "common/estimation/KalmanPredictor.h"
#include "common/estimation/KalmanUpdater.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tracking {

/**
 * @brief RirImmFilter 双/多模型 CV IMM 滤波器（每航迹一份，按关联键持有）。
 */
class RirImmFilter {
 public:
  using ImmFilterT = ::oneq::common::estimation::ImmFilter<6, 3>;

  /**
   * @brief RirImmFilterConfig IMM 包装配置。
   * @note 嵌套类成员不使用类内初始化（与缺省实参 `= {}` 的完整类上下文限制冲突）；
   *       缺省语义由构造函数承载。
   */
  struct Config {
    /**
     * @brief 各模型 CV 过程噪声差异系数 q（m/s²），按噪声由低到高排列。
     * @note 空向量（缺省）时使用双模型缺省 {1.0, 10.0}
     *       （10^(i/(N-1)) 对数等距首两点）。
     */
    std::vector<float> model_noise_diff_coeffs;
    /** @brief 转移概率矩阵对角元（自保持概率）；非对角元均分余量；缺省 0.95。 */
    float transition_diagonal_probability;
  };

  /** @brief 构造 IMM；模型数 < 2 时进入惰性状态（IsValid 为 false）。 */
  explicit RirImmFilter(const Config& config = {});

  /** @return 注入校验是否通过。 */
  bool IsValid() const { return filter_ != nullptr && filter_->IsValid(); }

  /** @return 模型数；惰性状态为 0。 */
  std::size_t ModelCount() const { return filter_ != nullptr ? model_count_ : 0U; }

  /**
   * @brief 以初始高斯状态播种各模型分支（权重保持当前值）。
   * @param initial_state 初始状态（新航迹/滤波重置时刻的 CV 初始化）。
   */
  void Initialize(const RirGaussianState& initial_state);

  /**
   * @brief 完整 IMM 循环（混合 → 预测 → 更新 → 组合），动态 R 口径。
   * @param position 位置量测（m）。
   * @param dt_sec 时间步长（s），须为正。
   * @param dynamic_R 量测噪声协方差（m²）。
   */
  void Process(const Eigen::Vector3f& position, float dt_sec,
               const RirMeasurementCovariance& dynamic_R);

  /**
   * @brief 仅预测（失配周期）：混合 → 预测 → 组合。
   * @param dt_sec 时间步长（s），须为正。
   */
  void Predict(float dt_sec);

  /** @return 组合高斯状态。 */
  RirGaussianState GetCombinedState() const;

  /** @return 各模型当前权重。
   *  @note 权重仅内部消费（组合态之外不输出）——2026-08-20 验收输出统计裁定
   *        不新增输出字段（docs/review/acceptance_output_inventory_2026-08-20.md §4.5/§6）。 */
  Eigen::VectorXf GetModelWeights() const;

 private:
  std::size_t model_count_{0U};
  std::vector<::oneq::common::estimation::KalmanPredictor<6, 3>> predictors_{};
  std::vector<::oneq::common::estimation::KalmanUpdater<6, 3>> updaters_{};
  std::unique_ptr<ImmFilterT> filter_{};
};

}  // namespace tracking
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_IMM_FILTER_H_
