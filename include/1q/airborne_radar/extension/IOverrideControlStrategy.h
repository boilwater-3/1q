/**
 * @file IOverrideControlStrategy.h
 * @brief 定义外部策略覆盖接口，允许用户直接接管 LPI/ECCM 决策。
 *
 * 当实现该接口并注入到 TacticalCoordinator 后，内部评估器（LpiEvaluator、
 * EccmEvaluator）将不再自主运行，改为由外部策略直接生成控制提案。
 * 适用于用户需要在外部脚本或规则引擎中定义自定义策略的场景。
 */

#ifndef AIRBORNE_RADAR_EXTENSION_I_OVERRIDE_CONTROL_STRATEGY_H_
#define AIRBORNE_RADAR_EXTENSION_I_OVERRIDE_CONTROL_STRATEGY_H_

#include <vector>

#include "1q/api.hpp"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/model/DecisionSourceInfo.h"

namespace airborne_radar {
namespace model {

/** @brief 供 LPI 模块消费的威胁来源信息（内部定义，此处前向声明）。 */
struct LpiSourceInfo;

}  // namespace model
namespace extension {

/**
 * @brief IOverrideControlStrategy 提供外部策略覆盖能力。
 *
 * 实现此接口并注入 TacticalCoordinator 后，OverrideLpi 和 OverrideEccm
 * 方法将分别在 LPI 和 ECCM 评估阶段被调用。返回 true 表示本次决策已由
 * 外部策略处理，内部评估器不再参与。
 */
class ONEQ_API IOverrideControlStrategy {
 public:
  virtual ~IOverrideControlStrategy() = default;

  /**
   * @brief 覆盖 LPI 决策。
   *
   * @param lpi_source_info  ThreatAssessment 提供的 LPI 来源信息。
   * @param proposals        [out] 追加外部策略决定使用的控制提案。
   * @return true 表示外部策略已接管 LPI 决策，内部 LpiEvaluator 将跳过；
   *         false 表示仍使用内部评估。
   */
  virtual bool OverrideLpi(
      const model::LpiSourceInfo& lpi_source_info,
      std::vector<TacticalProposal>* proposals) = 0;

  /**
   * @brief 覆盖 ECCM 决策。
   *
   * @param eccm_source_info 环境层提供的干扰来源摘要（含安全网回填）。
   * @param proposals        [out] 追加外部策略决定使用的控制提案。
   * @return true 表示外部策略已接管 ECCM 决策，内部 EccmEvaluator 将跳过；
   *         false 表示仍使用内部评估。
   */
  virtual bool OverrideEccm(
      const model::EccmSourceInfo& eccm_source_info,
      std::vector<TacticalProposal>* proposals) = 0;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_EXTENSION_I_OVERRIDE_CONTROL_STRATEGY_H_
