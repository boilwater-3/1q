/**
 * @file StandardAtmosphere.h
 * @brief ISA 1976 标准大气模型实现。
 */

#ifndef COMMON_ATMOSPHERE_STANDARD_ATMOSPHERE_H_
#define COMMON_ATMOSPHERE_STANDARD_ATMOSPHERE_H_

#include "1q/environment/IAtmosphereProvider.h"

namespace oneq {
namespace common {
namespace atmosphere {

/**
 * @brief ISA 1976 标准大气模型（0–86 km）。
 */
class StandardAtmosphere : public environment::IAtmosphereProvider {
 public:
  environment::AtmosphericState GetState(float altitude_m) const override;
  environment::AtmosphericState GetSeaLevelState() const override;
};

}  // namespace atmosphere
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ATMOSPHERE_STANDARD_ATMOSPHERE_H_
