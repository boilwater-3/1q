/**
 * @file SarRawHistoryBuilder.h
 * @brief SAR raw history 构造工具：外部 IQ 接入、脉冲生成与 SNR 估算。
 */

#ifndef ONEQ_SRC_SAR_SESSION_SAR_RAW_HISTORY_BUILDER_H_
#define ONEQ_SRC_SAR_SESSION_SAR_RAW_HISTORY_BUILDER_H_

#include <cstdint>
#include <deque>

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/runtime/PulseRingBuffer.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace session {

/**
 * @brief 判定输入是否携带外部完整孔径 raw IQ 数据。
 * @param[in] input 单周期输入载荷。
 * @return 携带外部完整孔径 raw IQ 返回 true，否则返回 false。
 */
bool HasExternalRawIq(const SarCycleInput& input);

/**
 * @brief 根据 hardware 配置生成 LFM 发射波形与其匹配滤波器。
 * @param[in] config 会话配置。
 * @param[out] waveform 生成的 LFM 发射波形。
 * @param[out] matched_filter 生成的匹配滤波器系数。
 * @return 成功返回 true，失败返回 false。
 */
bool BuildWaveformAndFilter(const config::SarSessionConfig& config, signal::LfmWaveform* waveform,
                            signal::ComplexVector* matched_filter);

/**
 * @brief 由外部完整孔径 raw IQ 直接构造 raw history，同时回填理想/实际脉冲轨迹缓冲。
 * @param[in] config 会话配置。
 * @param[in] input 单周期输入载荷（含外部 IQ）。
 * @param[out] history 构造出的原始相位历史矩阵。
 * @param[out] ideal_trajectory_buffer 回填的理想脉冲轨迹缓冲。
 * @param[out] actual_trajectory_buffer 回填的实际脉冲轨迹缓冲。
 * @param[out] result 单周期结果，失败时写入结构化错误诊断。
 * @return 成功返回 true；失败返回 false 并已向 result 写入错误诊断。
 */
bool BuildExternalRawIqHistory(const config::SarSessionConfig& config, const SarCycleInput& input,
                               signal::ComplexMatrix* history,
                               std::deque<geometry::PlatformPulseState>* ideal_trajectory_buffer,
                               std::deque<geometry::PlatformPulseState>* actual_trajectory_buffer,
                               SarCycleResult* result);

/**
 * @brief 基于脉冲 ring buffer 生成新脉冲、追加重放孔径并构造 raw history。
 *
 * 消费并维护跨周期状态（pulse id、分数余量、轨迹缓冲）。
 * @param[in] config 会话配置。
 * @param[in] input 单周期输入载荷。
 * @param[in] transmit_waveform 发射波形。
 * @param[in,out] pulse_buffer 脉冲 ring buffer，生成新脉冲并追加孔径。
 * @param[in,out] next_pulse_id 下一个待分配脉冲 ID，调用后更新。
 * @param[in,out] pulse_fraction_carry PRF 重采样分数余量，调用后更新。
 * @param[out] history 构造出的原始相位历史矩阵。
 * @param[out] ideal_trajectory_buffer 回填的理想脉冲轨迹缓冲。
 * @param[out] actual_trajectory_buffer 回填的实际脉冲轨迹缓冲。
 * @param[out] result 单周期结果，失败时写入错误诊断。
 * @return 成功返回 true；失败返回 false 并已向 result 写入错误诊断。
 * @note 会修改跨周期状态，调用方须保证单线程或外部串行化。
 */
bool BuildRawPulseHistory(const config::SarSessionConfig& config, const SarCycleInput& input,
                          const signal::ComplexVector& transmit_waveform,
                          runtime::PulseRingBuffer* pulse_buffer, std::uint64_t* next_pulse_id,
                          double* pulse_fraction_carry, signal::ComplexMatrix* history,
                          std::deque<geometry::PlatformPulseState>* ideal_trajectory_buffer,
                          std::deque<geometry::PlatformPulseState>* actual_trajectory_buffer,
                          SarCycleResult* result);

/**
 * @brief 基于峰均功率比估算 raw history 的启发式 SNR（dB）。
 *
 * 空矩阵、全零孔径或非有限值返回 -inf。调用方应将 -inf 视为“无可估计信号”，而非低 SNR 失败。
 * @param[in] history 原始相位历史矩阵。
 * @return 估算 SNR（dB）；不可估计时返回 -inf。
 */
double EstimateRawHistorySnrDb(const signal::ComplexMatrix& history);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_RAW_HISTORY_BUILDER_H_
