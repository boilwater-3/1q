/**
 * @file EsrSession.h
 * @brief 定义面向外部调用方的电子侦察会话门面。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/foundation/SensorContract.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief ESR 运行期补丁应用状态码。
 */
enum class EsrRuntimeConfigApplyStatus {
  kApplied = 0,                         /**< 补丁已成功应用 */
  kNoRequestedUpdate,                   /**< 补丁未请求任何更新 */
  kRejectedInvalidScanRate,             /**< 因扫描数据率非法被拒绝 */
  kRejectedInvalidScanCenterAz,         /**< 因扫描中心方位角非法被拒绝 */
  kRejectedInvalidScanCenterEl,         /**< 因扫描中心俯仰角非法被拒绝 */
  kRejectedInvalidExplicitScanBounds,   /**< 因显式扫描边界非法被拒绝 */
};

/**
 * @brief ESR 运行期补丁应用结果。
 */
struct ONEQ_API EsrRuntimeConfigApplyResult {
  EsrRuntimeConfigApplyStatus status{EsrRuntimeConfigApplyStatus::kNoRequestedUpdate};
  bool has_requested_update{false};
  bool applied{false};
};

/**
 * @brief EsrSession 提供单周期外部接入门面。
 * @note 线程模型：会话对象持有可变状态，默认非线程安全；并发访问需外部同步。
 */
class ONEQ_API EsrSession {
 public:
  EsrSession();
  ~EsrSession();

  EsrSession(const EsrSession&) = delete;
  EsrSession& operator=(const EsrSession&) = delete;
  EsrSession(EsrSession&&) noexcept;
  EsrSession& operator=(EsrSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   */
  EsrOutputFrame Step(const EsrCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult StepWithResult(const EsrCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁。
   * @param[in] patch 运行期补丁。
   */
  void ApplyRuntimeConfig(const config::EsrRuntimeConfigPatch& patch);

  /**
   * @brief 尝试应用运行期可变配置补丁并返回是否生效。
   * @param[in] patch 运行期补丁。
   * @return true 表示补丁被接受并已应用；false 表示未请求更新或补丁被拒绝。
   */
  bool TryApplyRuntimeConfig(const config::EsrRuntimeConfigPatch& patch);

  /**
   * @brief 应用运行期补丁并返回结构化结果。
   * @param[in] patch 运行期补丁。
   * @return 结构化应用结果（含状态码）。
   */
  EsrRuntimeConfigApplyResult ApplyRuntimeConfigWithResult(const config::EsrRuntimeConfigPatch& patch);

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static EsrSession Create(const config::EsrSessionConfig& config = {});

  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   *
   * 与 `Create()` 唯一区别：构造前调用 `config::ValidateEsrSessionConfig`
   * 校验配置合法性，将发现的问题写入 @p issues。无论 @p issues 是否为空，
   * 都会构造并返回会话（不阻断），调用方据 `issues->empty()` 决定后续。
   *
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note `ValidateEsrSessionConfig` 由此路径被实调用，构成真实契约。
   */
  static EsrSession CreateWithValidation(const config::EsrSessionConfig& config,
                                         config::ValidationIssueList* issues);

 private:
  struct Impl;
  explicit EsrSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

// 跨域传感器会话形状契约：锚定 Step/StepWithResult 签名，防止伪对称漂移。
ONEQ_SENSOR_SESSION_CONTRACT(session::EsrSession, session::EsrCycleInput,
                             session::EsrOutputFrame, session::EsrCycleResult);

}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_H_
