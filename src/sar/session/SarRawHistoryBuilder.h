/**
 * @file SarRawHistoryBuilder.h
 * @brief SAR raw history 构造工具：外部 IQ 接入、脉冲生成与 SNR 估算。
 */

#ifndef ONEQ_SRC_SAR_SESSION_SAR_RAW_HISTORY_BUILDER_H_
#define ONEQ_SRC_SAR_SESSION_SAR_RAW_HISTORY_BUILDER_H_

#include <cstdint>
#include <deque>
#include <vector>

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
 * @brief 计算本周期脉冲数并生成理想/实际脉冲轨迹（不写入缓冲、不更新 pulse id）。
 *
 * 供成像 squint 门控前置使用：门控需在 raw echo 生成前获得"积累窗孔径轨迹"，
 * 而轨迹生成本身廉价（µs 级），echo 生成才是周期耗时主体。本函数同时按原
 * 逻辑更新 PRF 分数余量；生成的脉冲可经 BuildRawPulseHistory 的 prebuilt
 * 参数复用，避免二次生成（也避免 kL3Trajectory 诊断重复写入）。
 * @param[in] config 会话配置。
 * @param[in] input 单周期输入载荷。
 * @param[in] next_pulse_id 下一个待分配脉冲 ID（轨迹时间锚定，不改动）。
 * @param[in,out] pulse_fraction_carry PRF 重采样分数余量，调用后更新。
 * @param[in] pulse_buffer_size 脉冲 ring buffer 当前大小（冷启动补齐依据）。
 * @param[in] previous_actual 上一周期末实际脉冲（时间衔接），无则传 nullptr。
 * @param[out] ideal_pulses 生成的理想脉冲轨迹。
 * @param[out] actual_pulses 生成的实际脉冲轨迹。
 * @param[out] result 单周期结果，失败时写入错误诊断。
 * @return 成功返回 true；失败返回 false 并已向 result 写入错误诊断。
 */
bool PrepareCycleTrajectory(const config::SarSessionConfig& config, const SarCycleInput& input,
                            std::uint64_t next_pulse_id, double* pulse_fraction_carry,
                            std::size_t pulse_buffer_size,
                            const geometry::PlatformPulseState* previous_actual,
                            std::vector<geometry::PlatformPulseState>* ideal_pulses,
                            std::vector<geometry::PlatformPulseState>* actual_pulses,
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
 * @param[in] prebuilt_ideal 预生成理想轨迹（PrepareCycleTrajectory 产物）；
 *            非空时跳过脉冲数计算/轨迹生成，直接使用（须与 prebuilt_actual 成对）。
 * @param[in] prebuilt_actual 预生成实际轨迹；非空时跳过脉冲数计算/轨迹生成。
 * @return 成功返回 true；失败返回 false 并已向 result 写入错误诊断。
 * @note 会修改跨周期状态，调用方须保证单线程或外部串行化。
 */
bool BuildRawPulseHistory(const config::SarSessionConfig& config, const SarCycleInput& input,
                          const signal::ComplexVector& transmit_waveform,
                          runtime::PulseRingBuffer* pulse_buffer, std::uint64_t* next_pulse_id,
                          double* pulse_fraction_carry, signal::ComplexMatrix* history,
                          std::deque<geometry::PlatformPulseState>* ideal_trajectory_buffer,
                          std::deque<geometry::PlatformPulseState>* actual_trajectory_buffer,
                          double* estimated_snr_db, SarCycleResult* result,
                          const std::vector<geometry::PlatformPulseState>* prebuilt_ideal = nullptr,
                          const std::vector<geometry::PlatformPulseState>* prebuilt_actual =
                              nullptr);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_RAW_HISTORY_BUILDER_H_
