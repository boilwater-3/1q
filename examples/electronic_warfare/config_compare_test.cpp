#include <cmath>
#include <iostream>
#include <string>

#include "1q/coordinate/types.h"
#include "1q/electronic_surveillance_radar/config/EsrProfileConstants.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "config_loader.h"

namespace esr_cfg = electronic_surveillance_radar::config;
namespace esr_env = electronic_surveillance_radar::environment;
namespace esr_session = electronic_surveillance_radar::session;

namespace {

esr_cfg::EsrSessionConfig MakeEmitterSearchConfig() {
  esr_cfg::EsrSessionConfig config;
  config.mission = esr_cfg::profiles::kElectronicOrderOfBattleMission;
  // kStandard 灵敏度档位为 no-op（struct 默认即该档位），不再显式赋值。
  config.environment.scenario_config.preset = esr_cfg::EsrEnvironmentPreset::kStandard;
  config.mission.power_on = true;
  config.mission.work_mode = esr_cfg::EsrWorkMode::kEsm;
  config.mission.scan.scan_rate_hz = 1.0f;
  config.policy.detection.minimum_snr_db = 6.0f;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;
  config.hardware.antenna_peak_gain_dbi = 20.0f;
  return config;
}

int failed = 0;

void ReportF(const char* name, double a, double b) {
  if (std::abs(a - b) > 0.0001) {
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
  const esr_cfg::EsrSessionConfig builder_cfg = MakeEmitterSearchConfig();

  esr_cfg::EsrSessionConfig file_cfg;
  {
    std::string error;
    if (!examples::LoadEsrSessionConfigFromFile("configs/electronic_warfare.json", &file_cfg,
                                                &error)) {
      std::cerr << "FAIL: " << error << "\n";
      return 1;
    }
  }

  std::cout << "=== Comparing ESR SessionConfig ===\n";

  // hardware
  const auto& bh = builder_cfg.hardware;
  const auto& fh = file_cfg.hardware;
  ReportF("hardware.receiver_band_lower_hz", bh.receiver_band_lower_hz, fh.receiver_band_lower_hz);
  ReportF("hardware.receiver_band_upper_hz", bh.receiver_band_upper_hz, fh.receiver_band_upper_hz);
  ReportF("hardware.receiver_sensitivity_w", bh.receiver_sensitivity_w, fh.receiver_sensitivity_w);
  ReportF("hardware.integrated_receive_loss_db", bh.integrated_receive_loss_db,
          fh.integrated_receive_loss_db);
  ReportF("hardware.beam_az_width_deg", bh.beam_az_width_deg, fh.beam_az_width_deg);
  ReportF("hardware.beam_el_width_deg", bh.beam_el_width_deg, fh.beam_el_width_deg);
  ReportF("hardware.az_scan_range_deg", bh.az_scan_range_deg, fh.az_scan_range_deg);
  ReportF("hardware.el_scan_range_deg", bh.el_scan_range_deg, fh.el_scan_range_deg);
  ReportF("hardware.antenna_mount_az_deg", bh.antenna_mount_az_deg, fh.antenna_mount_az_deg);
  ReportF("hardware.antenna_mount_el_deg", bh.antenna_mount_el_deg, fh.antenna_mount_el_deg);
  ReportF("hardware.antenna_peak_gain_dbi", bh.antenna_peak_gain_dbi,
          fh.antenna_peak_gain_dbi);
  ReportF("hardware.antenna_sidelobe_level_db", bh.antenna_sidelobe_level_db,
          fh.antenna_sidelobe_level_db);
  ReportF("hardware.antenna_backlobe_level_db", bh.antenna_backlobe_level_db,
          fh.antenna_backlobe_level_db);
  ReportF("hardware.cross_polarization_isolation_db", bh.cross_polarization_isolation_db,
          fh.cross_polarization_isolation_db);
  ReportF("hardware.minimum_far_field_range_m", bh.minimum_far_field_range_m,
          fh.minimum_far_field_range_m);
  ReportF("hardware.maximum_linear_input_power_w", bh.maximum_linear_input_power_w,
          fh.maximum_linear_input_power_w);

  // mission
  const auto& bm = builder_cfg.mission;
  const auto& fm = file_cfg.mission;
  ReportB("mission.power_on", bm.power_on, fm.power_on);
  ReportI("mission.work_mode", static_cast<int>(bm.work_mode), static_cast<int>(fm.work_mode));

  const auto& bs = bm.scan;
  const auto& fs = fm.scan;
  ReportF("mission.scan.scan_center_az_deg", bs.scan_center_az_deg, fs.scan_center_az_deg);
  ReportF("mission.scan.scan_center_el_deg", bs.scan_center_el_deg, fs.scan_center_el_deg);
  ReportF("mission.scan.scan_rate_hz", bs.scan_rate_hz, fs.scan_rate_hz);
  ReportB("mission.scan.use_explicit_scan_bounds", bs.use_explicit_scan_bounds,
          fs.use_explicit_scan_bounds);
  ReportF("mission.scan.scan_start_az_deg", bs.scan_start_az_deg, fs.scan_start_az_deg);
  ReportF("mission.scan.scan_end_az_deg", bs.scan_end_az_deg, fs.scan_end_az_deg);
  ReportF("mission.scan.scan_start_el_deg", bs.scan_start_el_deg, fs.scan_start_el_deg);
  ReportF("mission.scan.scan_end_el_deg", bs.scan_end_el_deg, fs.scan_end_el_deg);

  // policy
  const auto& bp = builder_cfg.policy;
  const auto& fp = file_cfg.policy;
  ReportF("policy.detection.minimum_snr_db", bp.detection.minimum_snr_db,
          fp.detection.minimum_snr_db);
  ReportF("policy.detection.pfa", bp.detection.pfa, fp.detection.pfa);
  ReportI("policy.detection.pulse_count", static_cast<int>(bp.detection.pulse_count),
          static_cast<int>(fp.detection.pulse_count));
  ReportF("policy.detection.threshold_scale", bp.detection.threshold_scale,
          fp.detection.threshold_scale);
  ReportB("policy.detection.enable_statistical_detection",
          bp.detection.enable_statistical_detection, fp.detection.enable_statistical_detection);

  // environment
  const auto& be = builder_cfg.environment;
  const auto& fe = file_cfg.environment;
  ReportI("environment.scenario_config.preset", static_cast<int>(be.scenario_config.preset),
          static_cast<int>(fe.scenario_config.preset));
  ReportB("environment.scenario_config.atmospheric_physics.enable_physical_model",
          be.scenario_config.atmospheric_physics.enable_physical_model,
          fe.scenario_config.atmospheric_physics.enable_physical_model);
  ReportF("environment.scenario_config.atmospheric_physics.pressure_hpa",
          be.scenario_config.atmospheric_physics.pressure_hpa,
          fe.scenario_config.atmospheric_physics.pressure_hpa);
  ReportF("environment.scenario_config.atmospheric_physics.temperature_k",
          be.scenario_config.atmospheric_physics.temperature_k,
          fe.scenario_config.atmospheric_physics.temperature_k);
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
