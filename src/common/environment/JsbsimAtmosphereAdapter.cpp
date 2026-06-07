/**
 * @file JsbsimAtmosphereAdapter.cpp
 * @brief 实现 JSBSim → IAtmosphereProvider 桥接。
 */

#include "1q/environment/JsbsimAtmosphereAdapter.h"

#include "FGFDMExec.h"
#include "models/FGAtmosphere.h"

namespace oneq {
namespace environment {

namespace {

// ── imperial → SI 转换常数 ──
constexpr double kFtToM = 0.3048;
constexpr double kSlugToKg = 14.593903;
constexpr double kRankineToKelvin = 5.0 / 9.0;
constexpr double kPsfToPa = 47.880258;

}  // namespace

JsbsimAtmosphereAdapter::JsbsimAtmosphereAdapter(const JSBSim::FGFDMExec& fdm_exec)
    : fdm_exec_(fdm_exec) {}

AtmosphericState JsbsimAtmosphereAdapter::GetState(float altitude_m) const {
  const double altitude_ft = static_cast<double>(altitude_m) / kFtToM;

  const JSBSim::FGAtmosphere& atm = *fdm_exec_.GetAtmosphere();
  const double temperature_rankine = atm.GetTemperature(altitude_ft);
  const double density_slugs_ft3 = atm.GetDensity(altitude_ft);
  const double pressure_psf = atm.GetPressure(altitude_ft);
  const double speed_fps = atm.GetSoundSpeed(altitude_ft);

  AtmosphericState state;
  state.altitude_m = altitude_m;
  state.temperature_k = static_cast<float>(temperature_rankine * kRankineToKelvin);
  state.pressure_pa = static_cast<float>(pressure_psf * kPsfToPa);
  state.density_kg_m3 = static_cast<float>(
      density_slugs_ft3 * kSlugToKg / (kFtToM * kFtToM * kFtToM));
  state.speed_of_sound_mps = static_cast<float>(speed_fps * kFtToM);
  state.pressure_hpa = state.pressure_pa / 100.0f;
  return state;
}

AtmosphericState JsbsimAtmosphereAdapter::GetSeaLevelState() const {
  return GetState(0.0f);
}

}  // namespace environment
}  // namespace oneq
