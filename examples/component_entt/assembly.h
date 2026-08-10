/**
 * @file assembly.h
 * @brief 行为层装配：registry 接线各 session 输出/输入，固化周期系统调用序。
 *
 * 装配层职责（冻结契约 §5）：
 *   - 创建平台实体（长机 + 僚机 + 传感器实体）并挂载数据组件；
 *   - 会话与引擎（ArSession / EsrSession / EosSession / FusionEngine /
 *     AreaCoveragePlanner）放入 registry ctx 的 BehaviorContext，
 *     生命周期由装配层管理；
 *   - 暴露周期推进入口 StepBehaviorLayer 与融合态势观察者工厂。
 */

#ifndef EXAMPLES_BEHAVIOR_LAYER_ASSEMBLY_H_
#define EXAMPLES_BEHAVIOR_LAYER_ASSEMBLY_H_

#include <cstdint>
#include <memory>
#include <vector>

#include <entt/entt.hpp>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/navigation/AreaCoveragePlanner.h"

namespace component_entt {

/**
 * @brief 行为层装配配置：三传感器会话配置聚合。
 */
struct BehaviorLayerConfig {
  airborne_radar::config::ArSessionConfig ar{};                 /**< AR 会话配置 */
  electronic_surveillance_radar::config::EsrSessionConfig esr{}; /**< ESR 会话配置 */
  electro_optical_sensor::config::EosSessionConfig eos{};       /**< EOS 会话配置 */
};

/**
 * @brief 行为层运行时上下文：会话/引擎/世界事实（registry ctx 持有）。
 * @note 会话与引擎生命周期由装配层管理；三套世界真值（AR 目标/ESR 辐射源/
 *       EOS 光学目标）由消费方脚本每周期注入（去真值化纪律下目标事实属
 *       消费方场景编排）。
 */
struct BehaviorContext {
  std::unique_ptr<airborne_radar::session::ArSession> ar_session{}; /**< AR 会话 */
  std::unique_ptr<electronic_surveillance_radar::session::EsrSession> esr_session{}; /**< ESR 会话 */
  std::unique_ptr<electro_optical_sensor::session::EosSession> eos_session{}; /**< EOS 会话 */
  std::unique_ptr<fusion::FusionEngine> fusion_engine{};          /**< 融合引擎 */
  navigation::AreaCoveragePlanner planner{};                      /**< 区域覆盖规划器 */
  std::uint64_t cycle{0U}; /**< 已推进周期数（StepBehaviorLayer 每次调用递增） */
  std::vector<airborne_radar::session::ArTargetInput> world_targets{}; /**< AR 世界目标事实（消费方注入） */
  std::vector<oneq::electromagnetics::RfSceneEmission> emitter_truths{}; /**< ESR 辐射源真值（消费方注入） */
  std::vector<electro_optical_sensor::session::EosExternalTargetInput> optical_targets{}; /**< EOS 光学目标真值（消费方注入） */
  airborne_radar::session::ArCycleResult last_ar_result{};         /**< 最近一次 AR 周期结果（供消费方读取） */
  electronic_surveillance_radar::session::EsrCycleResult esr_last_result{}; /**< 最近一次 ESR 周期结果 */
  electro_optical_sensor::session::EosCycleResult eos_last_result{}; /**< 最近一次 EOS 周期结果 */
};

/**
 * @brief 装配行为层：创建长机 + 僚机 + 三传感器实体、注入上下文、挂载组件。
 * @param[in] registry 目标 registry。
 * @param[in] config 三传感器会话配置（复用 examples/configs 同源 JSON）。
 * @return 长机实体（消费方以此读取融合态势/命令帧/航路）。
 */
entt::entity AssembleBehaviorLayer(entt::registry& registry, const BehaviorLayerConfig& config);

/**
 * @brief 推进一个仿真周期：按 recon → maneuver → jam → decision 顺序执行系统。
 * @param[in] registry 已装配的 registry。
 */
void StepBehaviorLayer(entt::registry& registry);

/**
 * @brief 创建融合态势观察者（监听 FusedSituationComponent 变化事件）。
 * @param[in] registry 已装配的 registry。
 * @return 已接线的观察者；EnTT 3.14 的 observer 不可移动，以 unique_ptr 返回。
 *         消费方每周期迭代后调用 clear()（报告节奏属业务层）。
 */
std::unique_ptr<entt::observer> MakeSituationObserver(entt::registry& registry);

}  // namespace component_entt

#endif  // EXAMPLES_BEHAVIOR_LAYER_ASSEMBLY_H_
