/**
 * @file EcmInternalTypes.h
 * @brief ECM 模块内部共享类型：调度威胁表示、种子派生和域标签常量。
 *
 * 此头文件仅被 src/electronic_countermeasure/ 下的内部实现文件包含，不暴露到公共 API。
 */

#ifndef ELECTRONIC_COUNTERMEASURE_ECM_INTERNAL_TYPES_H_
#define ELECTRONIC_COUNTERMEASURE_ECM_INTERNAL_TYPES_H_

#include <cstdint>

namespace electronic_countermeasure {
namespace session {

/// @brief 调度 RNG 域标签（"SCHD"）：专责 sweep 方向采样。
constexpr std::uint32_t kSchedulingDomain = UINT32_C(0x53434844);

/// @brief 平局裁决 RNG 域标签（"TIEB"）：专责等分威胁排序。
constexpr std::uint32_t kTieBreakDomain = UINT32_C(0x54494542);

/// @brief 欺骗 RNG 域标签（"DEPT"）：专责欺骗发射 timing seed 生成。
constexpr std::uint32_t kDeceptionDomain = UINT32_C(0x44455054);

/// @brief 运行时状态快照 schema 版本号。
constexpr std::uint32_t kRuntimeStateSchemaVersion = 2U;

/// @brief 传感器观测最大滑行成功周期数。
constexpr std::uint32_t kMaximumGlideSuccessfulCycles = 2U;

/**
 * @brief 从单一配置种子按 splitmix32 终结器派生独立的 RNG 子流。
 *
 * 两个相同基础种子但不同 domain tag 的流互不相关。此约定与 SBIRS
 * DeriveMeasurementSeed 一致（参见 src/sbirs_sensor/pipeline/SbirsPipeline.cpp）。
 *
 * @param base_seed 配置中的 random_seed。
 * @param domain_tag 消费者域标签（如 kSchedulingDomain）。
 * @return 派生出的 32 位种子（保证非零）。
 */
inline std::uint32_t DeriveStreamSeed(std::uint32_t base_seed, std::uint32_t domain_tag) {
  std::uint32_t value = base_seed ^ domain_tag;
  value ^= value >> 16U;
  value *= 0x7feb352dU;
  value ^= value >> 15U;
  value *= 0x846ca68bU;
  value ^= value >> 16U;
  return value == 0U ? 1U : value;
}

/**
 * @brief 调度阶段的统一威胁表示。
 *
 * 传感器驱动的威胁携带 observation_id，真值辅助的威胁携带 truth_entity_id；
 * 每条威胁仅有一个非零标识。
 */
struct SchedulingThreat {
  std::uint64_t observation_id{0U};
  std::uint64_t truth_entity_id{0U};
  double center_frequency_hz{0.0};
  double bandwidth_hz{0.0};
  float score{0.0f};
  /// @brief 在排序前派生的确定性平局裁决键；与输入顺序无关。
  std::uint32_t tie_break_key{0U};
  /// @brief 欺骗专用：来自威胁源的估计 PRI（秒）。
  double estimated_pri_s{1.0e-3};
  /// @brief 欺骗专用：来自威胁源的估计脉宽（秒）。
  double estimated_pulse_width_s{1.0e-6};
};

/**
 * @brief 返回威胁的稳定标识符。
 *
 * 传感器驱动的威胁返回 observation_id，真值辅助的威胁返回 truth_entity_id。
 */
inline std::uint64_t ThreatStableId(const SchedulingThreat& threat) {
  return threat.observation_id != 0U ? threat.observation_id : threat.truth_entity_id;
}

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ELECTRONIC_COUNTERMEASURE_ECM_INTERNAL_TYPES_H_
