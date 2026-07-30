/**
 * @file EcmConfigValidator.h
 * @brief ECM 纯配置与输入校验器——无状态，所有方法均为静态纯函数。
 *
 * 职责：配置有效性、传感器观测有效性、周期输入合同、运行时补丁合并校验和
 * 快照内部一致性验证的单一归属地。不拥有可变状态，不产生副作用。
 */

#ifndef ELECTRONIC_COUNTERMEASURE_ECM_CONFIG_VALIDATOR_H_
#define ELECTRONIC_COUNTERMEASURE_ECM_CONFIG_VALIDATOR_H_

#include "1q/electronic_countermeasure/EcmTypes.h"

namespace electronic_countermeasure {
namespace session {

/**
 * @brief 纯校验函数集合，无成员状态。
 *
 * 所有方法均为静态；调用方负责将校验结果用于决策（接受/拒绝/提交）。
 */
class EcmConfigValidator {
 public:
  /// @brief 判定 technique 是否为已知枚举值。
  static bool IsKnownTechnique(EcmTechnique technique);

  /// @brief 判定 mode 是否为已知欺骗模式枚举值。
  static bool IsKnownDeceptionMode(EcmDeceptionMode mode);

  /// @brief 完整验证 EcmSessionConfig 的所有字段边界和组合约束。
  static bool IsValidConfig(const config::EcmSessionConfig& config);

  /// @brief 验证单个传感器观测的标量限值和有限性。
  static bool IsValidSensorObservation(const EcmSensorObservation& observation);

  /// @brief 验证 EcmCycleInput 的互斥模式、provenance 和嵌套观测字段。
  static bool IsValidInput(const EcmCycleInput& input);

  /**
   * @brief 将运行时配置补丁合并到候选配置并验证。
   *
   * 如果补丁未请求任何更新，返回 false（candidate 不变）。
   * 如果合并后的配置无效，返回 false（candidate 为部分合并结果，不得使用）。
   *
   * @param current 当前活跃配置。
   * @param patch 运行时补丁。
   * @param[out] candidate 合并后的候选配置（仅在返回 true 时有效）。
   * @return 合并成功且候选配置有效时返回 true。
   */
  static bool TryMergePatch(const config::EcmSessionConfig& current,
                            const config::EcmRuntimeConfigPatch& patch,
                            config::EcmSessionConfig* candidate);

  /**
   * @brief 运行时状态快照的内部一致性完整校验。
   *
   * 验证嵌套 observation 有效性、重复 ID、provenance、模式组合、
   * 欺骗状态 phase/mode 合法性、并发上限和威胁 ID 唯一性。
   * 对标 design.md §3 要求。
   *
   * @return 仅当快照可由合法会话产生时返回 true。
   */
  static bool IsSnapshotInternallyConsistent(const EcmRuntimeState& state);
};

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ELECTRONIC_COUNTERMEASURE_ECM_CONFIG_VALIDATOR_H_
