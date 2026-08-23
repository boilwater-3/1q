/**
 * @file FusionAcceptanceRecords.h
 * @brief 融合层验收行拼装。
 */

#ifndef ONEQ_SRC_FUSION_FUSION_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_FUSION_FUSION_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <vector>

#include "1q/fusion/FusedTarget.h"
#include "1q/fusion/FusionConfig.h"

namespace fusion {

/** @brief 接力视线角采样（方位为原始缠绕值，解缠在估计函数内完成）。 */
struct RelayAngularSample {
  double time_sec{0.0}; /**< 采样时刻（s）。 */
  double az_deg{0.0};   /**< 方位角（deg，可跨 ±180 缠绕）。 */
  double el_deg{0.0};   /**< 俯仰角（deg，不缠绕）。 */
};

/**
 * @brief 方位差分归一到 (-180, 180]（deg）：跨 ±180 缠绕不产生 360° 假差。
 * @note 不受验收日志宏门控，恒编译可测。
 */
double WrappedAzimuthDeltaDeg(double az_from_deg, double az_to_deg);

/**
 * @brief 滑窗最小二乘视线角速率（单位：deg/s）。
 * @param[in] samples 时间升序采样窗（方位在内部逐差分解缠后做方位/俯仰对时间的
 *                    最小二乘斜率，速率 = √(斜率_az² + 斜率_el²)）。
 * @return 可用（≥2 个不同时刻采样）时返回角速率（恒非负）；否则返回 -1.0。
 * @note 逐拍差分对单拍抖动敏感（批注口径验收行曾因此跳变）；最小二乘斜率把
 *       噪声均摊到窗内，2026-08-22 甲方批注「加个简易算法预测」的稳健化。
 */
double LeastSquaresAngularSpeedDegPerSec(const std::vector<RelayAngularSample>& samples);

/**
 * @brief 接力剩余覆盖时长（单位：s）：(视场角宽度 − 已扫过角) ÷ 角速率。
 * @param[in] fov_width_deg   视场角宽度（deg，FusionConfig::relay_fov_width_deg）。
 * @param[in] swept_deg       自首见累计已扫过角（deg，解缠）。
 * @param[in] omega_deg_per_s 角速率（deg/s，滑窗最小二乘估计）。
 * @return 视场/速率有效时返回剩余时长（扫满则 0，夹取不下穿 0）；否则 -1.0
 *         （不可用，调用方写无）。
 * @note 简易外推（2026-08-22 甲方批注「加个简易算法预测」口径，库内无接力协议）：
 *       分子用实测已扫过角（时间锚定倒计时会在速率抖动时跳变甚至变大），按目标
 *       自首见起匀速扫掠剩余角量估计；首见不在视场边缘时为上界近似。
 */
double RelayRemainingCoverageSec(double fov_width_deg, double swept_deg,
                                 double omega_deg_per_s);

void WriteFusionAcceptance(std::uint32_t cycle, const std::vector<FusedTarget>& tracks,
                           bool filtering_enabled, const FusionConfig& config);

}  // namespace fusion

#endif
