#include <cmath>
#include <iostream>
#include <string>

#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "config_loader.h"

namespace eos_cfg = electro_optical_sensor::config;

namespace {

eos_cfg::EosSessionConfig MakeFusedSearchConfig() {
  eos_cfg::EosSessionConfig config;
  config.hardware.wavelength_lower_um = 3.0f;
  config.hardware.wavelength_upper_um = 5.0f;
  config.hardware.optical_aperture_m = 0.25f;
  config.mission.work_mode = eos_cfg::EosWorkMode::kFused;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  config.mission.frame_rate_hz = 30.0f;
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.horizontal_fov_deg = 20.0f;
  config.mission.vertical_fov_deg = 8.0f;
  config.mission.scan_center_el_deg = -48.0f;
  config.mission.boresight_depression_deg = 48.0f;
  config.policy.detection.minimum_snr_db = 4.0f;
  config.policy.detection.detection_sensitivity_w = 1.0e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 800.0f;
  config.policy.stray_light.enable_straylight_filter = false;
  config.policy.stray_light.hood_inner_half_angle_deg = 12.0f;
  config.policy.stray_light.hood_outer_half_angle_deg = 75.0f;
  config.policy.stray_light.hood_min_suppression_ratio = 0.2f;
  config.policy.stray_light.hood_max_suppression_ratio = 0.85f;
  config.environment.scenario_config.preset = eos_cfg::EosEnvironmentPreset::kStandard;
  return config;
}

int failed = 0;

void ReportF(const char* name, float a, float b) {
  if (std::abs(a - b) > 0.0001f) {
    std::cerr << "  FAIL " << name << ": " << a << " != " << b << "\n";
    ++failed;
  }
}

void ReportI(const char* name, int a, int b) {
  if (a != b) {
    std::cerr << "  FAIL " << name << ": " << a << " != " << b << "\n";
    ++failed;
  }
}

void ReportB(const char* name, bool a, bool b) {
  if (a != b) {
    std::cerr << "  FAIL " << name << ": " << a << " != " << b << "\n";
    ++failed;
  }
}

}  // namespace

int main() {
  const eos_cfg::EosSessionConfig builder_cfg = MakeFusedSearchConfig();

  eos_cfg::EosSessionConfig file_cfg;
  {
    std::string error;
    if (!examples::LoadEosSessionConfigFromFile("configs/electro_optical.json", &file_cfg,
                                                &error)) {
      std::cerr << "FAIL: " << error << "\n";
      return 1;
    }
  }

  std::cout << "=== Comparing EOS SessionConfig ===\n";

  // hardware
  const auto& bh = builder_cfg.hardware;
  const auto& fh = file_cfg.hardware;
  ReportF("hardware.wavelength_lower_um", bh.wavelength_lower_um, fh.wavelength_lower_um);
  ReportF("hardware.wavelength_upper_um", bh.wavelength_upper_um, fh.wavelength_upper_um);
  ReportF("hardware.optical_aperture_m", bh.optical_aperture_m, fh.optical_aperture_m);

  // mission
  const auto& bm = builder_cfg.mission;
  const auto& fm = file_cfg.mission;
  ReportI("mission.work_mode", static_cast<int>(bm.work_mode), static_cast<int>(fm.work_mode));
  ReportF("mission.horizontal_fov_deg", bm.horizontal_fov_deg, fm.horizontal_fov_deg);
  ReportF("mission.vertical_fov_deg", bm.vertical_fov_deg, fm.vertical_fov_deg);
  ReportF("mission.scan_rate_deg_per_sec", bm.scan_rate_deg_per_sec, fm.scan_rate_deg_per_sec);
  ReportF("mission.frame_rate_hz", bm.frame_rate_hz, fm.frame_rate_hz);
  ReportF("mission.scan_start_az_deg", bm.scan_start_az_deg, fm.scan_start_az_deg);
  ReportF("mission.scan_end_az_deg", bm.scan_end_az_deg, fm.scan_end_az_deg);
  ReportF("mission.scan_center_el_deg", bm.scan_center_el_deg, fm.scan_center_el_deg);
  ReportF("mission.boresight_depression_deg", bm.boresight_depression_deg,
          fm.boresight_depression_deg);

  // policy
  const auto& bp = builder_cfg.policy;
  const auto& fp = file_cfg.policy;
  ReportF("policy.detection.minimum_snr_db", bp.detection.minimum_snr_db,
          fp.detection.minimum_snr_db);
  ReportF("policy.detection.detection_sensitivity_w", bp.detection.detection_sensitivity_w,
          fp.detection.detection_sensitivity_w);
  ReportF("policy.detection.visible_reference_irradiance_w_m2",
          bp.detection.visible_reference_irradiance_w_m2,
          fp.detection.visible_reference_irradiance_w_m2);

  ReportB("policy.stray_light.enable_straylight_filter", bp.stray_light.enable_straylight_filter,
          fp.stray_light.enable_straylight_filter);
  ReportF("policy.stray_light.hood_inner_half_angle_deg", bp.stray_light.hood_inner_half_angle_deg,
          fp.stray_light.hood_inner_half_angle_deg);
  ReportF("policy.stray_light.hood_outer_half_angle_deg", bp.stray_light.hood_outer_half_angle_deg,
          fp.stray_light.hood_outer_half_angle_deg);
  ReportF("policy.stray_light.hood_min_suppression_ratio",
          bp.stray_light.hood_min_suppression_ratio, fp.stray_light.hood_min_suppression_ratio);
  ReportF("policy.stray_light.hood_max_suppression_ratio",
          bp.stray_light.hood_max_suppression_ratio, fp.stray_light.hood_max_suppression_ratio);

  // environment
  const auto& be = builder_cfg.environment;
  const auto& fe = file_cfg.environment;
  ReportI("environment.scenario_config.preset", static_cast<int>(be.scenario_config.preset),
          static_cast<int>(fe.scenario_config.preset));
  ReportB("environment.scenario_config.atmospheric_physics.enable_physical_model",
          be.scenario_config.atmospheric_physics.enable_physical_model,
          fe.scenario_config.atmospheric_physics.enable_physical_model);
  ReportF("environment.scenario_config.atmospheric_physics.relative_humidity",
          be.scenario_config.atmospheric_physics.relative_humidity,
          fe.scenario_config.atmospheric_physics.relative_humidity);
  if (failed == 0) {
    std::cout << "PASS: all fields match\n";
    return 0;
  }
  std::cerr << "FAILED: " << failed << " field(s) differ\n";
  return 1;
}
