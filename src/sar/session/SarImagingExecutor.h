/**
 * @file SarImagingExecutor.h
 * @brief SAR L1 RDA 与 L3 BP 聚焦成像执行入口。
 */

#ifndef ONEQ_SRC_SAR_SESSION_SAR_IMAGING_EXECUTOR_H_
#define ONEQ_SRC_SAR_SESSION_SAR_IMAGING_EXECUTOR_H_

#include <deque>

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleResult.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace session {

/**
 * @brief 执行 L1 RDA 聚焦成像。
 *
 * 当 policy.enable_l2_motion_compensation 为真时，先在 raw history 上执行一阶运动
 * 补偿（基于理想/实际轨迹缓冲），再进入 RDA。成功时写入：
 *   - result.focused_image（按 retain_focused_image 决定全矩阵或占位元数据）
 *   - sar.motion_compensation / sar.rda_peak 诊断
 *   - output_frame 的 L1 阶段标记与图像质量字段
 * 失败时向 result 写入结构化错误诊断（abort_reason / sar.<tag>）并返回 false；调用方应据此中止本周期。
 * @note 行为与诊断字符串与历史实现逐字符保持一致，以维持 replay 严格相等。
 * @param[in] config 会话配置。
 * @param[in] raw_history 原始相位历史矩阵。
 * @param[in] matched_filter 匹配滤波器系数。
 * @param[in] ideal_trajectory_buffer 理想轨迹脉冲缓冲（运动补偿时使用）。
 * @param[in] actual_trajectory_buffer 实际轨迹脉冲缓冲。
 * @param[out] result 单周期结果，成功时写入聚焦图像与诊断。
 * @return 成功返回 true；失败返回 false 并已向 result 写入错误诊断。
 */
bool ExecuteL1RdaImaging(const config::SarSessionConfig& config,
                         const signal::ComplexMatrix& raw_history,
                         const signal::ComplexVector& matched_filter,
                         const std::deque<geometry::PlatformPulseState>& ideal_trajectory_buffer,
                         const std::deque<geometry::PlatformPulseState>& actual_trajectory_buffer,
                         SarCycleResult* result);

/**
 * @brief 执行 L3 BP 聚焦成像。
 *
 * 逐脉冲实际轨迹来自 actual_trajectory_buffer。成功时写入：
 *   - result.focused_image（按 retain_focused_image 决定全矩阵或占位元数据）
 *   - sar.bp_peak / sar.bp_traversal 诊断
 *   - output_frame 的 L3 阶段标记与图像质量字段
 * 失败时写入结构化错误诊断并返回 false。
 * @note 行为与诊断字符串与历史实现逐字符保持一致，以维持 replay 严格相等。
 * @param[in] config 会话配置。
 * @param[in] raw_history 原始相位历史矩阵。
 * @param[in] matched_filter 匹配滤波器系数。
 * @param[in] actual_trajectory_buffer 实际轨迹脉冲缓冲。
 * @param[out] result 单周期结果，成功时写入聚焦图像与诊断。
 * @return 成功返回 true；失败返回 false 并已向 result 写入错误诊断。
 */
bool ExecuteL3BpImaging(const config::SarSessionConfig& config,
                        const signal::ComplexMatrix& raw_history,
                        const signal::ComplexVector& matched_filter,
                        const std::deque<geometry::PlatformPulseState>& actual_trajectory_buffer,
                        SarCycleResult* result);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_IMAGING_EXECUTOR_H_
