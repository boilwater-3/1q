/**
 * @file EsrSessionFactory.h
 * @brief 定义 EsrSession 的公共创建入口。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_FACTORY_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_FACTORY_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrSessionFactory 负责电子侦察会话装配与创建。
 */
class ONEQ_API EsrSessionFactory {
 public:
  /**
   * @brief 使用默认配置创建会话。
   * @param config 会话配置。
   * @return 新创建的 EsrSession。
   */
  static EsrSession Create(const config::EsrSessionConfig& config = {});
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_FACTORY_H_
