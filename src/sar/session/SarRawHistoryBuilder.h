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

// 判定输入是否携带外部完整孔径 raw IQ 数据。
bool HasExternalRawIq(const SarCycleInput& input);

// 根据 hardware 配置生成 LFM 发射波形与其匹配滤波器。
bool BuildWaveformAndFilter(const config::SarSessionConfig& config, signal::LfmWaveform* waveform,
                            signal::ComplexVector* matched_filter);

// 由外部完整孔径 raw IQ 直接构造 raw history，同时回填理想/实际脉冲轨迹缓冲。
// 返回 false 时已向 result 写入结构化错误诊断。
bool BuildExternalRawIqHistory(const config::SarSessionConfig& config, const SarCycleInput& input,
                               signal::ComplexMatrix* history,
                               std::deque<geometry::PlatformPulseState>* ideal_trajectory_buffer,
                               std::deque<geometry::PlatformPulseState>* actual_trajectory_buffer,
                               SarCycleResult* result);

// 基于脉冲 ring buffer 生成新脉冲、追加重放孔径并构造 raw history。
// 消费并维护跨周期状态（pulse id、分数余量、轨迹缓冲）；返回 false 时已写入错误诊断。
bool BuildRawPulseHistory(const config::SarSessionConfig& config, const SarCycleInput& input,
                          const signal::ComplexVector& transmit_waveform,
                          runtime::PulseRingBuffer* pulse_buffer, std::uint64_t* next_pulse_id,
                          double* pulse_fraction_carry, signal::ComplexMatrix* history,
                          std::deque<geometry::PlatformPulseState>* ideal_trajectory_buffer,
                          std::deque<geometry::PlatformPulseState>* actual_trajectory_buffer,
                          SarCycleResult* result);

// 基于峰均功率比估算 raw history 的启发式 SNR（dB）；空矩阵、全零孔径或
// 非有限值返回 -inf。调用方应将 -inf 视为“无可估计信号”，而非低 SNR 失败。
double EstimateRawHistorySnrDb(const signal::ComplexMatrix& history);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_RAW_HISTORY_BUILDER_H_
