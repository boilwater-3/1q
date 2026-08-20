/**
 * @file EcmResourceLedger.h
 * @brief ECM 资源账本——拥有发射 ID、热预算和调度/平局裁决 RNG 流。
 *
 * 提供显式的 reserve/commit 语义：调用方复制本对象为候选，在候选上操作，
 * 所有波形构造成功后通过赋值提交候选到真实账本。失败时丢弃候选即可。
 */

#ifndef ELECTRONIC_COUNTERMEASURE_ECM_RESOURCE_LEDGER_H_
#define ELECTRONIC_COUNTERMEASURE_ECM_RESOURCE_LEDGER_H_

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "1q/electronic_countermeasure/EcmTypes.h"
#include "electronic_countermeasure/EcmInternalTypes.h"

namespace electronic_countermeasure {
namespace session {

/**
 * @brief 拥有所有资源分配状态：发射 ID、热能和两条 RNG 流。
 *
 * 候选模式：StepWithResult 在顶部复制此对象 (`auto candidate = ledger_`)，
 * 对候选进行操作（reserve ID、累加热量、消费 RNG），所有波形成功后
 * `ledger_ = candidate` 提交。如果任一波形失败，候选出作用域即丢弃。
 */
class EcmResourceLedger {
 public:
  /// @brief 从基础种子派生 scheduling 和 tie-break 两条 RNG 子流。
  explicit EcmResourceLedger(std::uint32_t random_seed);

  // --- Emission ID ---

  /// @brief 查看下一个将分配的发射 ID（不推进计数器）。
  std::uint64_t next_emission_id() const;

  /// @brief 预留并返回下一个发射 ID，推进内部计数器。
  std::uint64_t ReserveEmissionId();

  /// @brief 直接设置发射 ID（快照恢复用）。
  void SetNextEmissionId(std::uint64_t id);

  // --- Thermal ---

  /// @brief 查看当前累计热能（焦耳）。
  double thermal_energy_j() const;

  /// @brief 应用冷却：减去 cooling_power_w * dt_sec，下限 0。
  void ApplyCooling(double cooling_power_w, double dt_sec);

  /// @brief 累加热能：增加 energy_j 焦耳。
  void AddThermalEnergy(double energy_j);

  /// @brief 直接设置热能（快照恢复用）。
  void SetThermalEnergy(double energy_j);

  /// @brief 计算热容量施加的功率上限（纯计算，不修改状态）。
  double ComputeThermalPowerLimit(double thermal_capacity_j, double dt_sec) const;

  // --- RNG 访问 ---

  /// @brief 调度 RNG（sweep 方向采样）。波形工厂通过候选副本消费。
  std::mt19937& scheduling_rng();
  const std::mt19937& scheduling_rng() const;

  /// @brief 平局裁决 RNG（等分威胁排序）。AssignTieBreakKeys 消费。
  std::mt19937& tie_break_rng();
  const std::mt19937& tie_break_rng() const;

  // --- 威胁可行性 ---

  /**
   * @brief 判定威胁是否在硬件频率范围内可行。
   *
   * 欺骗模式下额外检查 Doppler 偏移后的占用带宽是否越界。
   */
  static bool IsFeasibleThreat(const SchedulingThreat& threat,
                               const config::EcmSessionConfig& config);

  // --- 平局裁决 ---

  /**
   * @brief 为威胁列表派发确定性的、与输入顺序无关的平局裁决键。
   *
   * 按威胁稳定 ID 的规范序（而非输入序）逐 ID 从 tie_break_rng 抽取一个
   * 32-bit 字，将 {id, draw} 组合为最终键回填到各威胁。相同威胁集合在任意
   * 输入顺序下产生相同的 {ID → 键} 映射。
   */
  void AssignTieBreakKeys(std::vector<SchedulingThreat>* threats);

  // --- 序列化支持（快照用）---

  /// @brief 将 scheduling RNG 序列化为字符串。
  std::string SerializeSchedulingRng() const;
  /// @brief 将 tie-break RNG 序列化为字符串。
  std::string SerializeTieBreakRng() const;

  /// @brief 从字符串反序列化 scheduling RNG。
  bool DeserializeSchedulingRng(const std::string& state);
  /// @brief 从字符串反序列化 tie-break RNG。
  bool DeserializeTieBreakRng(const std::string& state);

 private:
  std::uint64_t next_emission_id_{1U};
  double thermal_energy_j_{0.0};
  std::mt19937 scheduling_rng_;
  std::mt19937 tie_break_rng_;
};

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ELECTRONIC_COUNTERMEASURE_ECM_RESOURCE_LEDGER_H_
