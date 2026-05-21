#include "common/atmosphere/AtmospherePhysics.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"
#include "common/numerics/Constants.h"

namespace oneq {
namespace internal {
namespace atmosphere {

namespace {

constexpr double kMinKelvin = 150.0;
constexpr double kSeaLevelPressureHpa = 1013.25;
constexpr double kSeaLevelDensity = 1.225;
constexpr double kPressureScaleHeightM = 8434.5;
constexpr double kRefractivityScaleHeightM = 7350.0;

double EstimatePressureFromAltitudeHpa(double altitude_m) {
  const double safe_altitude_m = std::max(0.0, altitude_m);
  return kSeaLevelPressureHpa * std::exp(-safe_altitude_m / kPressureScaleHeightM);
}

}  // namespace

float blake_atmos_loss_r4_1(float h_a_m, float f_hz, float theta_deg, float r_m, float k) {
  return static_cast<float>(blake_atmos_loss_r8_1(
      static_cast<double>(h_a_m), static_cast<double>(f_hz), static_cast<double>(theta_deg),
      static_cast<double>(r_m), static_cast<double>(k)));
}

double blake_atmos_loss_r8_1(double h_a_m, double f_hz, double theta_deg, double r_m, double k) {
  const double safe_freq_hz = std::max(f_hz, 1.0e6);
  const double safe_range_m = std::max(r_m, 0.0);
  const double safe_altitude_m = std::max(h_a_m, 0.0);
  const double safe_k = std::max(k, 0.2);
  (void)theta_deg;

  const double pressure_ratio =
      EstimatePressureFromAltitudeHpa(safe_altitude_m) / kSeaLevelPressureHpa;
  const double f_ghz = safe_freq_hz / 1.0e9;
  const double oxygen_specific_db_per_km = 0.005 * std::pow(f_ghz, 1.25) * pressure_ratio;
  const double vapor_specific_db_per_km =
      0.008 * std::pow(f_ghz, 1.1) * std::exp(-safe_altitude_m / 2000.0);
  const double specific_attenuation_db_per_km =
      oxygen_specific_db_per_km + vapor_specific_db_per_km;
  const double path_km = safe_range_m * 1.0e-3;
  return std::max(0.0, specific_attenuation_db_per_km * path_km / safe_k);
}

float refractivity_index_n_r4(float tc_celsius, float tk_kelvin, float pd_hpa, float p_hpa,
                              float h_rel, int water_or_ice) {
  return static_cast<float>(refractivity_index_n_r8(
      static_cast<double>(tc_celsius), static_cast<double>(tk_kelvin), static_cast<double>(pd_hpa),
      static_cast<double>(p_hpa), static_cast<double>(h_rel), water_or_ice));
}

double refractivity_index_n_r8(double tc_celsius, double tk_kelvin, double pd_hpa, double p_hpa,
                               double h_rel, int water_or_ice) {
  const double safe_tk = std::max(tk_kelvin, kMinKelvin);
  const double safe_rh = oneq::internal::numerics::Clamp(h_rel, 0.0, 1.0);
  const double safe_pressure_hpa = std::max(p_hpa, 0.0);
  const double saturation_vapor_hpa =
      6.112 * std::exp((17.67 * tc_celsius) / std::max(tc_celsius + 243.5, 1.0));
  const double vapor_pressure_hpa = oneq::internal::numerics::Clamp(safe_rh * saturation_vapor_hpa, 0.0, safe_pressure_hpa);
  const double safe_dry_pressure_hpa = std::max(pd_hpa, safe_pressure_hpa - vapor_pressure_hpa);

  const double dry_refractivity = 77.6 * safe_dry_pressure_hpa / safe_tk;
  const double wet_coeff = water_or_ice == 0 ? 3.73e5 : 3.46e5;
  const double wet_refractivity = wet_coeff * vapor_pressure_hpa / (safe_tk * safe_tk);
  const double total_refractivity = std::max(0.0, dry_refractivity + wet_refractivity);
  return 1.0 + total_refractivity * 1.0e-6;
}

float refractivity_index_nh_r4(float n0_index, float h_m, float h0_m) {
  const double safe_h = std::max(static_cast<double>(h_m), 0.0);
  const double safe_h0 = std::max(static_cast<double>(h0_m), 1.0);
  const double n0 = std::max(static_cast<double>(n0_index), 1.0);
  const double n0_refractivity = (n0 - 1.0) * 1.0e6;
  const double nh_refractivity = n0_refractivity * std::exp(-safe_h / safe_h0);
  return static_cast<float>(1.0 + nh_refractivity * 1.0e-6);
}

Gtd7Profile GTD7(int day_of_year, double sec, double alt_m, double glat_deg, double glong_deg,
                 double stl, double f107a, double f107, double ap, int mass) {
  (void)sec;
  (void)glat_deg;
  (void)glong_deg;
  (void)stl;
  (void)mass;

  const double safe_alt_m = std::max(0.0, alt_m);
  const int wrapped_day = std::max(1, std::min(366, day_of_year));
  const double seasonal_phase = (2.0 * oneq::internal::numerics::constants::kPi * static_cast<double>(wrapped_day)) / 365.0;
  const double seasonal_factor = 1.0 + 0.03 * std::cos(seasonal_phase);
  const double solar_factor = oneq::internal::numerics::Clamp((f107a + f107) / 300.0, 0.6, 1.6);
  const double geomagnetic_factor = 1.0 + oneq::internal::numerics::Clamp(ap, 0.0, 400.0) * 1.0e-3;

  double base_temperature_k = 216.65;
  if (safe_alt_m < 11000.0) {
    base_temperature_k = 288.15 - 0.0065 * safe_alt_m;
  } else if (safe_alt_m < 25000.0) {
    base_temperature_k = 216.65;
  } else {
    base_temperature_k = 216.65 + 0.002 * (safe_alt_m - 25000.0);
  }
  const double temperature_k =
      std::max(kMinKelvin, base_temperature_k * solar_factor * seasonal_factor);

  const double scale_height_m = 7500.0 * solar_factor * geomagnetic_factor;
  const double density_kg_m3 =
      kSeaLevelDensity * std::exp(-safe_alt_m / std::max(scale_height_m, 1000.0)) * seasonal_factor;

  Gtd7Profile profile;
  profile.temperature_k = temperature_k;
  profile.density_kg_m3 = std::max(1.0e-8, density_kg_m3);
  return profile;
}

AtmosphericPropagationResult EvaluateAtmosphericPropagation(
    const AtmosphericPropagationInputs& inputs) {
  AtmosphericPropagationResult result;
  if (!inputs.enable_physics) {
    return result;
  }

  const double mid_altitude_m = 0.5 * (static_cast<double>(inputs.radar_altitude_m) +
                                       static_cast<double>(inputs.target_altitude_m));
  const Gtd7Profile profile =
      GTD7(inputs.day_of_year, 0.0, mid_altitude_m, 0.0, 0.0, 0.0, inputs.solar_flux_f107a,
           inputs.solar_flux_f107, inputs.geomagnetic_ap, 48);
  const double pressure_hpa = inputs.pressure_hpa > 0.0f
                                  ? static_cast<double>(inputs.pressure_hpa)
                                  : EstimatePressureFromAltitudeHpa(mid_altitude_m);
  const double dry_pressure_hpa =
      pressure_hpa * (1.0 - 0.15 * oneq::internal::numerics::Clamp(inputs.relative_humidity, 0.0f, 1.0f));
  const double temperature_k = inputs.temperature_k > 0.0f
                                   ? static_cast<double>(inputs.temperature_k)
                                   : profile.temperature_k;
  const double temperature_c = temperature_k - 273.15;

  const double blake_loss_db =
      blake_atmos_loss_r8_1(mid_altitude_m, inputs.frequency_hz, inputs.elevation_deg,
                            inputs.path_length_m, inputs.k_factor);
  const double n_index = refractivity_index_n_r8(temperature_c, temperature_k, dry_pressure_hpa,
                                                 pressure_hpa, inputs.relative_humidity, 0);
  const double altitude_span_m = std::fabs(static_cast<double>(inputs.target_altitude_m) -
                                           static_cast<double>(inputs.radar_altitude_m));
  const double n_index_h =
      refractivity_index_nh_r4(static_cast<float>(n_index), static_cast<float>(altitude_span_m),
                               static_cast<float>(kRefractivityScaleHeightM));
  const double refractivity_gradient_db =
      std::max(0.0, (n_index - static_cast<double>(n_index_h)) * 1.0e6 * 1.0e-3);
  const double density_factor = oneq::internal::numerics::Clamp(profile.density_kg_m3 / kSeaLevelDensity, 0.05, 4.0);
  const double total_loss_db =
      std::max(0.0, blake_loss_db * (0.85 + 0.15 * density_factor) + refractivity_gradient_db);

  result.blake_loss_db = static_cast<float>(blake_loss_db);
  result.refractivity_index = static_cast<float>(n_index);
  result.refractivity_index_h = static_cast<float>(n_index_h);
  result.neutral_density_kg_m3 = static_cast<float>(profile.density_kg_m3);
  result.total_physics_loss_db = static_cast<float>(total_loss_db);
  return result;
}

}  // namespace atmosphere
}  // namespace internal
}  // namespace oneq
