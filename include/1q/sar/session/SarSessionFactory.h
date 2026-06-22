/**
 * @file SarSessionFactory.h
 * @brief 定义 SarSession 的公共创建入口。
 */

#ifndef ONEQ_SAR_SESSION_SAR_SESSION_FACTORY_H_
#define ONEQ_SAR_SESSION_SAR_SESSION_FACTORY_H_

#include "1q/api.hpp"
#include "1q/sar/session/SarSession.h"

namespace sar {
namespace session {

/**
 * @brief SAR 会话公共工厂。
 */
class ONEQ_API SarSessionFactory {
 public:
  static SarSession Create(const config::SarSessionConfig& config = {});
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_SESSION_FACTORY_H_
