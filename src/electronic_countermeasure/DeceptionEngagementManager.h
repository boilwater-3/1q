/**
 * @file DeceptionEngagementManager.h
 * @brief 欺骗交战状态管理器——唯一拥有交战状态、相位转换和模式迁移。
 *
 * 不涉及波形构造、功率分配或发射 ID。仅管理欺骗状态机和该域的 RNG。
 */

#ifndef ELECTRONIC_COUNTERMEASURE_DECEPTION_ENGAGEMENT_MANAGER_H_
#define ELECTRONIC_COUNTERMEASURE_DECEPTION_ENGAGEMENT_MANAGER_H_

#include <cstdint>
#include <random>
#include <vector>

#include "1q/electronic_countermeasure/EcmTypes.h"

namespace electronic_countermeasure {
namespace session {

/**
 * @brief 欺骗交战状态的唯一拥有者。
 *
 * 管理 EcmDeceptionState 向量的增删、相位推进和模式切换时的清空。
 * 候选模式：StepWithResult 在顶部复制此对象，操作候选，成功后提交。
 */
class DeceptionEngagementManager {
 public:
  /// @brief 使用派生种子初始化 deception RNG。
  explicit DeceptionEngagementManager(std::uint32_t random_seed);

  /**
   * @brief 推进所有活跃交战的相位状态机。
   *
   * 在每个 ECM 周期开始时调用。处理 kTowing → kHolding → kStopped → kIdle
   * 转换，并在转为 kIdle 后清理非活跃交战。
   */
  void AdvanceStates(const config::EcmSessionConfig& config, double dt_sec);

  /**
   * @brief 查找或创建指定威胁的欺骗交战。
   *
   * 如果已有该威胁的活跃交战，返回其指针。否则创建新模式为 kTowing 的新交战，
   * 前提是当前活跃交战数未达到 max_active 上限。
   *
   * @return 交战状态指针（仅在下次 mutation 前有效），上限已达时返回 nullptr。
   */
  EcmDeceptionState* FindOrCreate(std::uint64_t threat_id, EcmDeceptionMode mode,
                                  std::uint32_t max_active);

  /// @brief 清空所有欺骗交战状态。
  void Clear();

  /**
   * @brief 判断配置变更是否需要清空交战状态。
   *
   * 以下情况需要清空：关闭电源、技术不再是 kDeception、欺骗模式变更。
   */
  static bool ModeChangeInvalidates(const config::EcmSessionConfig& old_config,
                                    const config::EcmSessionConfig& new_config);

  // --- 快照支持 ---

  /// @brief 捕获当前所有欺骗交战状态。
  std::vector<EcmDeceptionState> CaptureStates() const;

  /// @brief 恢复欺骗交战状态（快照恢复用）。
  void RestoreStates(const std::vector<EcmDeceptionState>& states);

  // --- RNG 访问 ---

  /// @brief 欺骗 RNG（波形工厂通过候选副本消费脉冲 timing 随机性）。
  std::mt19937& deception_rng();
  const std::mt19937& deception_rng() const;

  /// @brief 直接设置欺骗 RNG（快照恢复用）。
  void SetDeceptionRng(std::mt19937 rng);

  /// @brief 将欺骗 RNG 序列化为字符串（快照捕获用）。
  std::string SerializeDeceptionRng() const;

  /// @brief 从字符串反序列化欺骗 RNG（快照恢复用）。
  bool DeserializeDeceptionRng(const std::string& state);

 private:
  std::vector<EcmDeceptionState> states_;
  std::mt19937 deception_rng_;
};

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ELECTRONIC_COUNTERMEASURE_DECEPTION_ENGAGEMENT_MANAGER_H_
