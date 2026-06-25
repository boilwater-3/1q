/**
 * @file EosSessionFactory.h
 * @brief 定义 EosSession 的公共创建入口。
 * @note 管线与环境服务已完全内部化，不再支持外部注入。
 *       使用 Create() 自动装配默认管线与环境模型。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_FACTORY_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_FACTORY_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {

class ONEQ_API EosSessionFactory {
 public:
  static EosSession Create(const config::EosSessionConfig& config = {});
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_FACTORY_H_
