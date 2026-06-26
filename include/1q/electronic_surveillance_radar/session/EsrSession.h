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
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/foundation/SensorContract.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief ESR 运行期补丁应用状态码。
 */
enum class EsrRuntimeConfigApplyStatus {
  kApplied = 0,
  kNoRequestedUpdate,
  kRejectedInvalidScanRate,
  kRejectedInvalidScanCenterAz,
  kRejectedInvalidScanCenterEl,
  kRejectedInvalidExplicitScanBounds,
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

 private:
  friend class EsrSessionFactory;
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
