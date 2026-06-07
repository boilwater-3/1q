/**
 * @file StandardAtmosphere.h
 * @brief 提供 ISA 1976 标准大气模型工厂。
 */

#ifndef ONEQ_ENVIRONMENT_STANDARD_ATMOSPHERE_H_
#define ONEQ_ENVIRONMENT_STANDARD_ATMOSPHERE_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/environment/IAtmosphereProvider.h"

namespace oneq {
namespace environment {

/**
 * @brief 创建 ISA 1976 标准大气（0–86 km）provider 实例。
 * @return 拥有所有权的 IAtmosphereProvider 实例。
 */
ONEQ_API std::unique_ptr<IAtmosphereProvider> CreateStandardAtmosphere();

}  // namespace environment
}  // namespace oneq

#endif  // ONEQ_ENVIRONMENT_STANDARD_ATMOSPHERE_H_
