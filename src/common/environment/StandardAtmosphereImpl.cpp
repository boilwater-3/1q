/**
 * @file StandardAtmosphereImpl.cpp
 * @brief 实现 ISA 1976 标准大气工厂函数。
 */

#include "1q/environment/StandardAtmosphere.h"

#include "common/atmosphere/StandardAtmosphere.h"

namespace oneq {
namespace environment {

namespace {

/**
 * @brief 将 internal StandardAtmosphere 适配为公开 IAtmosphereProvider。
 */
class StandardAtmosphereAdapter : public IAtmosphereProvider {
 public:
  AtmosphericState GetState(float altitude_m) const override {
    const auto internal_state = impl_.GetState(altitude_m);
    AtmosphericState state;
    state.altitude_m = internal_state.altitude_m;
    state.temperature_k = internal_state.temperature_k;
    state.pressure_pa = internal_state.pressure_pa;
    state.density_kg_m3 = internal_state.density_kg_m3;
    state.speed_of_sound_mps = internal_state.speed_of_sound_mps;
    state.pressure_hpa = internal_state.pressure_hpa;
    return state;
  }

  AtmosphericState GetSeaLevelState() const override {
    const auto internal_state = impl_.GetSeaLevelState();
    AtmosphericState state;
    state.altitude_m = internal_state.altitude_m;
    state.temperature_k = internal_state.temperature_k;
    state.pressure_pa = internal_state.pressure_pa;
    state.density_kg_m3 = internal_state.density_kg_m3;
    state.speed_of_sound_mps = internal_state.speed_of_sound_mps;
    state.pressure_hpa = internal_state.pressure_hpa;
    return state;
  }

 private:
  oneq::internal::atmosphere::StandardAtmosphere impl_;
};

}  // namespace

std::unique_ptr<IAtmosphereProvider> CreateStandardAtmosphere() {
  return std::unique_ptr<IAtmosphereProvider>(new StandardAtmosphereAdapter());
}

}  // namespace environment
}  // namespace oneq
