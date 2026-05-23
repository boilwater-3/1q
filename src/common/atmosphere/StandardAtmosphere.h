/**
 * @file StandardAtmosphere.h
 * @brief ISA 1976 标准大气模型实现。
 */

#ifndef COMMON_ATMOSPHERE_STANDARD_ATMOSPHERE_H_
#define COMMON_ATMOSPHERE_STANDARD_ATMOSPHERE_H_

#include "1q/foundation/atmosphere_provider.h"

namespace oneq {
namespace internal {
namespace atmosphere {

/**
 * @brief ISA 1976 标准大气模型（0–86 km）。
 */
class StandardAtmosphere : public foundation::IAtmosphereProvider {
 public:
  foundation::AtmosphericState GetState(float altitude_m) const override;
  foundation::AtmosphericState GetSeaLevelState() const override;
};

}  // namespace atmosphere
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_ATMOSPHERE_STANDARD_ATMOSPHERE_H_
