/**
 * @file SbirsErrorModel.h
 * @brief SBIRS-inspired WFOV 带误差位置模型（design 2.10）。
 *
 * 实现 design.md 2.10 的 5 类物理误差：
 *   1. 卫星轨道误差（orbit_sigma_deg）
 *   2. 卫星姿态误差（attitude_sigma_deg）
 *   3. 探测器视场误差（fov_sigma_deg）
 *   4. 大气折射误差（deterministic，随俯仰角与距离）
 *   5. 动态滞后误差（deterministic，随目标角速度与探测器带宽）
 * 角度类误差用确定性、可注入种子的 Box-Muller 高斯采样源；折射与滞后为确定性公式。
 * 随机源状态可 Capture/Restore，保证同一 trace 回放产生相同带误差位置与捕获结果。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_ERROR_MODEL_H_
#define ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_ERROR_MODEL_H_

#include <cstdint>

#include "1q/sbirs_sensor/config/SbirsPolicyConfig.h"

namespace sbirs_sensor {
namespace foundation {

/**
 * @brief 可注入、可快照的确定性伪随机源（xorshift32），保证 replay 可复现。
 * @note 随机源状态可 Capture/Restore，使同一 trace 回放产生相同带误差位置与捕获结果。
 */
class SbirsRandomSource {
 public:
  /**
   * @brief 构造随机源；种子为 0 时内部归一为 1 以避免退化。
   * @param[in] seed 随机种子
   */
  explicit SbirsRandomSource(unsigned int seed) : state_(seed == 0U ? 1U : seed) {}

  /** @return 当前内部状态值，用于快照。 */
  unsigned int Capture() const { return state_; }
  /** @brief 恢复内部状态；state 为 0 时归一为 1。 */
  void Restore(unsigned int state) { state_ = (state == 0U) ? 1U : state; }

  /** @brief 生成 [0,1) 均匀分布样本。 */
  double NextUniform();

  /** @brief 生成 Box-Muller 标准正态分布样本（均值 0，标准差 1）。 */
  double NextStandardNormal();

 private:
  unsigned int state_;
};

/**
 * @brief WFOV 带误差位置（design 2.10）。
 * @note 角度单位 deg，距离为乘法比例误差后的值。
 */
struct SbirsErrorBearing {
  float azimuth_deg{0.0f};  /**< 带误差方位角，单位 deg */
  float elevation_deg{0.0f}; /**< 带误差俯仰角，单位 deg */
  double range_m{0.0};      /**< 带误差距离，单位 m */
};

/**
 * @brief 对真值方位/俯仰/距离施加 5 类误差，返回带误差 cue 位置。
 * @param[in] model 误差模型配置
 * @param[in,out] random 随机源，采样后状态前推（不可为 nullptr）
 * @param[in] true_azimuth_deg 真值方位角，单位 deg
 * @param[in] true_elevation_deg 真值俯仰角，单位 deg
 * @param[in] true_range_m 真值距离，单位 m
 * @param[in] target_angular_rate_deg_per_sec 目标角速度（动态滞后用），单位 deg/s
 * @return 带误差 cue 位置
 * @note 角度类高斯误差采用可注入种子的 Box-Muller 采样；折射与滞后为确定性公式。
 */
SbirsErrorBearing ApplyAngularErrorModel(
    const config::SbirsErrorModelConfig& model, SbirsRandomSource* random, float true_azimuth_deg,
    float true_elevation_deg, double true_range_m, float target_angular_rate_deg_per_sec);

/**
 * @brief 2.10 大气折射误差（红外波段近似）：Δθ_refr = 1.5e-6 / (d·cosβ)，单位 deg。
 * @param[in] range_m 距离，单位 m
 * @param[in] elevation_deg 目标俯仰角 β，单位 deg
 * @return 折射角误差，单位 deg
 */
double RefractionErrorDeg(double range_m, float elevation_deg);

/**
 * @brief 2.10 动态滞后误差：Δθ_lag = ω_tar / (2π·f_det)，单位 deg。
 * @param[in] target_angular_rate_deg_per_sec 目标角速度 ω_tar，单位 deg/s
 * @param[in] detector_bandwidth_hz 探测器带宽 f_det，单位 Hz
 * @return 动态滞后角误差，单位 deg
 */
double DynamicLagErrorDeg(float target_angular_rate_deg_per_sec, float detector_bandwidth_hz);

}  // namespace foundation
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_ERROR_MODEL_H_
