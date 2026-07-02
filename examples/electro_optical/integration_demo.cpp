/**
 * @file integration_demo.cpp
 * @brief Electro-optical sensor integration demo — showing EosModule usage
 *        inside an external simulation engine.
 *
 * @par Scenario
 * Simulates the integration pattern of an external simulation engine,
 * demonstrating the complete lifecycle of EosModule. The engine injects
 * runtime configuration changes through a subscriber-pattern callback
 * mechanism.
 *
 * @par Key Concepts
 * - Three-phase lifecycle: initialize -> preStart -> stepImp
 * - Config flattening: hierarchical EosSessionConfig expanded into flat
 *   private members
 * - Subscriber pattern: registerConfigPatchCallback to register callbacks,
 *   automatically collected each cycle in stepImp
 * - File config loading: preStart loads from JSON via config_loader.h
 *
 * @par Run
 *   cd examples/
 *   ../build/llvm-ninja-release-local/bin/eos_integration_demo
 *   or with config path:
 *   ./eos_integration_demo ../configs/electro_optical.json
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "EosModule.h"

// =============================================================================
// Part 1: Simulated external engine runtime config manager
// =============================================================================
//
// This manager injects runtime configuration changes through the EosModule
// subscriber pattern. In a real integration scenario, such a manager might
// listen to DDS topics, UI control panels, or higher-level command systems.
// =============================================================================

/// Configuration change phase enum.
enum class ConfigPhase {
  kNormalFused,    ///< Normal fused search (default)
  kInfraredOnly,   ///< Switch to IR-only (cycle 20 trigger)
  kInfraredActive, ///< Infrared-only mode in progress
  kRestoreFused,   ///< Restore fused mode (cycle 35 trigger)
  kFusedBack,      ///< Back to normal fused tracking
  kSensorOff,      ///< Turn off sensor (cycle 45 trigger)
  kDone            ///< Done
};

/**
 * @brief Simulated runtime configuration manager for an external engine.
 *
 * Injects runtime configuration patches through the callback chain in
 * stepImp. Uses a simple state machine to toggle work modes at specific
 * cycle numbers.
 */
class SimulatedConfigManager {
 public:
  /**
   * @brief Generates a runtime configuration patch.
   *
   * Registered as EosModule::ConfigPatchCallback, called automatically each
   * cycle by stepImp. Injects config changes at these trigger points:
   *   - Cycle 20: switch to InfraredOnly mode
   *   - Cycle 35: restore Fused mode
   *   - Cycle 45: turn off sensor (simulated emergency)
   *
   * @param[in]  cycle  Current cycle number (1-based)
   * @param[out] patch  Config patch to fill
   */
  void evaluatePatch(std::uint32_t cycle,
                     eos_config::EosRuntimeConfigPatch& patch) {
    switch (phase_) {
      case ConfigPhase::kNormalFused:
        if (cycle >= 20) {
          phase_ = ConfigPhase::kInfraredOnly;
          fillInfraredOnlyPatch(patch);
        }
        break;

      case ConfigPhase::kInfraredOnly:
        phase_ = ConfigPhase::kInfraredActive;
        break;

      case ConfigPhase::kInfraredActive:
        if (cycle >= 35) {
          phase_ = ConfigPhase::kRestoreFused;
          fillFusedPatch(patch);
        }
        break;

      case ConfigPhase::kRestoreFused:
        phase_ = ConfigPhase::kFusedBack;
        break;

      case ConfigPhase::kFusedBack:
        if (cycle == 45) {
          phase_ = ConfigPhase::kSensorOff;
          fillSensorOffPatch(patch);
        }
        break;

      case ConfigPhase::kSensorOff:
        phase_ = ConfigPhase::kDone;
        break;

      case ConfigPhase::kDone:
        break;
    }
  }

  /** @brief Whether a patch was actually applied in the last call. */
  bool lastPatchApplied() const { return last_patch_applied_; }

  /** @brief Clear the last-patch-applied flag. */
  void clearLastPatchFlag() { last_patch_applied_ = false; }

 private:
  void fillInfraredOnlyPatch(eos_config::EosRuntimeConfigPatch& patch) {
    patch.has_work_mode = true;
    patch.work_mode = eos_config::EosWorkMode::kInfraredOnly;
    last_patch_applied_ = true;
  }

  void fillFusedPatch(eos_config::EosRuntimeConfigPatch& patch) {
    patch.has_work_mode = true;
    patch.work_mode = eos_config::EosWorkMode::kFused;
    last_patch_applied_ = true;
  }

  void fillSensorOffPatch(eos_config::EosRuntimeConfigPatch& patch) {
    patch.has_sensor_enabled = true;
    patch.sensor_enabled = false;
    last_patch_applied_ = true;
  }

  ConfigPhase phase_{ConfigPhase::kNormalFused};
  bool last_patch_applied_{false};
};

// =============================================================================
// Part 2: Helper functions — scenario construction
// =============================================================================

namespace {

/// Target appearance description for EOS scenario.
struct EosTargetDesc {
  std::uint64_t id;
  double lat_deg;
  double lon_deg;
  double alt_m;
  double vel_east_mps;
  double vel_north_mps;
  double vel_up_mps;
  float temperature_k;
  float area_m2;
};

EosTargetDesc MakeTarget(std::uint64_t id, double lat_deg, double lon_deg,
                         double alt_m, double vel_east_mps,
                         double vel_north_mps, double vel_up_mps,
                         float temperature_k, float area_m2) {
  return {id,    lat_deg,   lon_deg,        alt_m,
          vel_east_mps, vel_north_mps, vel_up_mps, temperature_k, area_m2};
}

eos_session::EosExternalTargetInput ToExternalInput(const EosTargetDesc& td,
                                                     std::uint32_t cycle) {
  eos_session::EosExternalTargetInput t;
  t.target_id = td.id;

  // Compute moving-target position at current cycle
  constexpr double kDt = 1.0;
  double lat = td.lat_deg;
  double lon = td.lon_deg;

  if (td.vel_north_mps != 0.0 || td.vel_east_mps != 0.0) {
    // Moving target: propagate position
    const double kDegPerMeterLat = 1.0 / 111111.0;
    const double lat_rad = td.lat_deg * 3.141592653589793 / 180.0;
    const double kDegPerMeterLon =
        1.0 / (111111.0 * std::cos(lat_rad));

    lat = td.lat_deg +
          td.vel_north_mps * kDt * static_cast<double>(cycle) * kDegPerMeterLat;
    lon = td.lon_deg +
          td.vel_east_mps * kDt * static_cast<double>(cycle) * kDegPerMeterLon;
  }

  t.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  t.kinematics.position_lla_deg_m.latitude_deg = lat;
  t.kinematics.position_lla_deg_m.longitude_deg = lon;
  t.kinematics.position_lla_deg_m.altitude_m = td.alt_m;

  // Convert ENU velocity to ECEF velocity for the kinematics
  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = lat;
  target_lla.longitude_deg = lon;
  target_lla.altitude_m = td.alt_m;

  oneq::coordinate::EnuVelocityMps vel_enu;
  vel_enu.east_mps = td.vel_east_mps;
  vel_enu.north_mps = td.vel_north_mps;
  vel_enu.up_mps = td.vel_up_mps;

  oneq::coordinate::TryEnuToEcefVelocity(vel_enu, target_lla,
                                          &t.kinematics.velocity_mps);

  t.appearance.apparent_temperature_k = td.temperature_k;
  t.appearance.emissivity = 0.92f;
  t.appearance.reflectance = 0.35f;
  t.appearance.projected_area_m2 = td.area_m2;
  return t;
}

}  // namespace

// =============================================================================
// Part 3: Main function
// =============================================================================

/**
 * @brief Entry point: run the EOS integration demo.
 *
 * @param[in] argc  Argument count
 * @param[in] argv  argv[1] optional JSON config file path
 * @return 0 on success, 1 on failure
 */
int main(int argc, char** argv) {
  const std::string config_path =
      (argc > 1) ? argv[1] : "configs/electro_optical.json";
  constexpr std::uint32_t kNumCycles = 50;

  std::cout << "=== Electro-Optical Sensor Integration Module Demo ===\n"
            << "  Config file: " << config_path << "\n"
            << "  Total cycles: " << kNumCycles << "\n\n";

  // ===============================================
  // Step 1: Create EosModule
  // ===============================================
  // Constructor only initialises defaults; no heavyweight work.
  std::cout << "[1/6] Constructing EosModule...\n";
  EosModule eos;

  // ===============================================
  // Step 2: initialize — initialise internal state
  // ===============================================
  // Create an EosSession instance (using default empty config).
  std::cout << "[2/6] initialize()...\n";
  if (!eos.initialize()) {
    std::cerr << "  ERROR: initialize() failed\n";
    return 1;
  }
  std::cout << "  EosSession created\n";

  // ===============================================
  // Step 3: preStart — load config from file and flatten
  // ===============================================
  // Read four-domain config from JSON (hardware/mission/policy/environment),
  // flatten all leaf-node parameters into private members of EosModule.
  // Also rebuilds EosSession with the full config.
  std::cout << "[3/6] preStart(\"" << config_path << "\")...\n";
  if (!eos.preStart(config_path)) {
    std::cerr << "  ERROR: preStart() failed\n";
    return 1;
  }
  std::cout << "  Config loaded and flattened\n"
            << "  Use printConfigSummary() to inspect\n";

  // ===============================================
  // Step 4: Register runtime config callbacks (subscriber pattern)
  // ===============================================
  // The engine registers callbacks via registerConfigPatchCallback.
  // These callbacks are automatically collected at the start of each
  // stepImp and synthesised into a runtime config patch.
  //
  // [Core pattern] The engine does not call ApplyRuntimeConfig manually;
  // it only registers callbacks, and stepImp handles collection -> application.
  std::cout << "[4/6] Registering runtime config callbacks "
               "(subscriber pattern)...\n";

  SimulatedConfigManager config_mgr;

  // Primary callback: captures cycle count via closure.
  eos.registerConfigPatchCallback(
      [&eos, &config_mgr](eos_config::EosRuntimeConfigPatch& patch) {
        config_mgr.evaluatePatch(eos.cycleCount() + 1, patch);
      });

  std::cout << "  Callback registered (SimulatedConfigManager ready)\n";

  // ===============================================
  // Step 5: Main simulation loop
  // ===============================================
  // Each cycle calls stepImp(input) to drive the EOS simulation.
  // stepImp internally:
  //   1. Collects all registered callbacks -> synthesises
  //      EosRuntimeConfigPatch
  //   2. If a valid patch exists, calls TryApplyRuntimeConfig
  //   3. Executes StepWithResult
  //   4. Caches result in lastResult()
  std::cout << "[5/6] Main simulation loop (" << kNumCycles << " cycles)...\n\n";

  // Platform initial position (ECEF): corresponds to ~35.0N, 114.5E, 7000m
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 35.0;
  platform_lla.longitude_deg = 114.5;
  platform_lla.altitude_m = 7000.0;

  oneq::coordinate::EcefPositionM platform_pos;
  if (!oneq::coordinate::TryLlaToEcef(platform_lla, &platform_pos)) {
    std::cerr << "  ERROR: failed to convert platform LLA to ECEF\n";
    return 1;
  }

  // Platform moves East at 150 m/s
  oneq::coordinate::EnuVelocityMps platform_vel_enu;
  platform_vel_enu.east_mps = 150.0;
  platform_vel_enu.north_mps = 0.0;
  platform_vel_enu.up_mps = 0.0;

  oneq::coordinate::EcefVelocityMps platform_vel;
  if (!oneq::coordinate::TryEnuToEcefVelocity(platform_vel_enu, platform_lla,
                                               &platform_vel)) {
    std::cerr << "  ERROR: failed to convert platform velocity to ECEF\n";
    return 1;
  }

  // Build initial environment
  eos_session::EosEnvironmentInput env_input;
  env_input.solar_altitude_deg = 42.0f;
  env_input.solar_azimuth_deg = 165.0f;
  env_input.solar_irradiance_w_m2 = 850.0f;
  env_input.cloud_coverage_ratio = 0.15f;
  env_input.background_temperature_k = 288.0f;
  env_input.day_night_type = eos_session::DayNightType::kDay;

  // 4 targets: 2 stationary, 2 moving
  // Platform boresight depression ~48 deg at 7000m intersects ground at
  // ~6303m ground distance. One degree longitude at 35N is ~91024m.
  // Base longitude of sensor footprint centre at start is 114.5 + 0.06925.
  constexpr double kBaseLon = 114.5 + 0.06925;

  std::vector<EosTargetDesc> targets;
  targets.push_back(
      MakeTarget(101, 35.0, kBaseLon + 0.013, 0.0, 0.0, 0.0, 0.0, 480.0f, 15.0f));
  targets.push_back(
      MakeTarget(102, 35.002, kBaseLon + 0.050, 0.0, 0.0, 0.0, 0.0, 490.0f, 12.0f));
  targets.push_back(
      MakeTarget(201, 34.998, kBaseLon + 0.025, 0.0, 0.0, 15.0, 0.0, 500.0f, 14.0f));
  targets.push_back(
      MakeTarget(202, 35.001, kBaseLon + 0.060, 0.0, -25.0, 0.0, 0.0, 520.0f, 18.0f));

  // Statistics
  std::size_t max_detected = 0;
  std::size_t min_detected = 100;
  std::uint32_t patch_applied_count = 0;

  for (std::uint32_t i = 0; i < kNumCycles; ++i) {
    const double dt = 1.0;

    // ---- Construct platform pose ----
    eos_session::EosExternalPoseInput platform;
    platform.platform_position_ecef_m.x_m =
        platform_pos.x_m + platform_vel.x_mps * dt * static_cast<double>(i);
    platform.platform_position_ecef_m.y_m =
        platform_pos.y_m + platform_vel.y_mps * dt * static_cast<double>(i);
    platform.platform_position_ecef_m.z_m =
        platform_pos.z_m + platform_vel.z_mps * dt * static_cast<double>(i);
    platform.platform_velocity_mps = platform_vel;
    platform.platform_attitude_deg.yaw_deg = 0.0;
    platform.platform_attitude_deg.pitch_deg = 0.0;
    platform.platform_attitude_deg.roll_deg = 0.0;

    // ---- Construct target inputs ----
    std::vector<eos_session::EosExternalTargetInput> target_inputs;
    target_inputs.reserve(targets.size());
    for (const auto& tgt : targets) {
      target_inputs.push_back(ToExternalInput(tgt, i));
    }

    // ---- Build EosCycleInput via adapter ----
    eos_session::EosCycleInput input;
    eos_session::EosCoordinateStatus status;
    if (!eos_session::EosCycleInputAdapter::Build(
            platform, target_inputs, static_cast<float>(dt), env_input,
            &input, &status)) {
      std::cerr << "  Cycle " << (i + 1)
                << ": EosCycleInputAdapter::Build failed (status="
                << static_cast<int>(status) << ")\n";
      return 1;
    }
    input.cycle_index = i + 1;

    // === [Core]: Drive the cycle through stepImp ===
    config_mgr.clearLastPatchFlag();
    eos.stepImp(input);

    // Check if this cycle triggered a config change
    if (config_mgr.lastPatchApplied()) {
      ++patch_applied_count;
      std::cout << "  Cycle " << (i + 1) << ": "
                << "-> config change injected via callback into stepImp\n";
    }

    // ---- Statistics ----
    const auto& result = eos.lastResult();
    std::size_t ndetected = 0;
    for (const auto& det : result.output_frame.detections) {
      if (det.detected) ++ndetected;
    }
    if (ndetected > max_detected) max_detected = ndetected;
    if (ndetected < min_detected) min_detected = ndetected;

    // Print every 5 cycles
    if ((i + 1) % 5 == 0) {
      std::string detected_ids;
      for (const auto& det : result.output_frame.detections) {
        if (det.detected) {
          detected_ids += std::to_string(det.detection_id) + " ";
        }
      }
      std::cout << "  [Cycle " << (i + 1) << "/" << kNumCycles << "]"
                << " detections=" << result.output_frame.detections.size()
                << " detected=" << ndetected
                << " (IDs: " << (detected_ids.empty() ? "None" : detected_ids)
                << ")"
                << (result.executed_this_cycle ? "" : " [not executed]")
                << "\n";
    }
  }

  // ===============================================
  // Step 6: Results summary and output
  // ===============================================
  std::cout << "\n[6/6] Simulation results summary\n"
            << "  Total cycles: " << kNumCycles << "\n"
            << "  Config change injections: " << patch_applied_count << "\n"
            << "  Min detected: " << min_detected << "\n"
            << "  Max detected: " << max_detected << "\n"
            << "  Current cycle count: " << eos.cycleCount() << "\n\n";

  // Print full config summary (via EosModule's internal method)
  eos.printConfigSummary();

  // ===============================================
  // Three-view output
  // ===============================================
  std::cout << "\n====== Three-View Output (Last Cycle) ======\n";

  // --- View 1: EosOutputFrame (system output) ---
  const auto& final_result = eos.lastResult();
  std::cout << "[View 1] EosOutputFrame — system-side detection output\n"
            << "  Input cycle: " << final_result.input_cycle_index << "\n"
            << "  Detections: " << final_result.output_frame.detections.size()
            << "\n";

  for (std::size_t k = 0; k < final_result.output_frame.detections.size();
       ++k) {
    const auto& det = final_result.output_frame.detections[k];
    std::cout << "    [" << k << "] id=" << det.detection_id
              << " range=" << det.range_m << "m"
              << " az=" << det.azimuth_deg << "deg"
              << " el=" << det.elevation_deg << "deg"
              << " IR_SNR_lin=" << det.infrared_snr_linear
              << " Vis_SNR_lin=" << det.visible_snr_linear
              << " Fused_SNR_db=" << det.fused_snr_db
              << " detected=" << (det.detected ? "yes" : "no") << "\n";
  }

  std::cout << "  Validation errors: "
            << (final_result.has_validation_error ? "yes" : "no") << "\n"
            << "  Executed: "
            << (final_result.executed_this_cycle ? "yes" : "no") << "\n\n";

  // --- View 2: EosOutputDebugView (human-readable debug view) ---
  eos_session::EosOutputDebugView debug_view = eos.buildLastDebugView();
  std::cout << "[View 2] EosOutputDebugView — human-readable debug view\n"
            << "  Input cycle: " << debug_view.input_cycle_index << "\n"
            << "  Output cycle: " << debug_view.output_cycle_index << "\n"
            << "  Executed: " << (debug_view.executed_this_cycle ? "yes" : "no")
            << "  Reused: "
            << (debug_view.reused_previous_output ? "yes" : "no") << "\n"
            << "  Target states:\n";

  for (std::size_t k = 0; k < debug_view.targets.size(); ++k) {
    const auto& t = debug_view.targets[k];
    const char* status_str = "";
    switch (t.status) {
      case eos_session::EosDebugTargetStatus::kDetected:
        status_str = "Detected";
        break;
      case eos_session::EosDebugTargetStatus::kObservedBelowThreshold:
        status_str = "BelowThreshold";
        break;
      case eos_session::EosDebugTargetStatus::kNotInOutput:
        status_str = "NotInOutput";
        break;
      case eos_session::EosDebugTargetStatus::kCycleNotExecuted:
        status_str = "CycleNotExecuted";
        break;
    }
    std::cout << "    [" << k << "] target_id=" << t.target_id
              << " status=" << status_str
              << " present_in_input=" << (t.present_in_input ? "yes" : "no")
              << " detected=" << (t.detected ? "yes" : "no")
              << " range=" << t.range_m
              << " az=" << t.azimuth_deg << "deg"
              << " el=" << t.elevation_deg << "deg"
              << " SNR_db=" << t.fused_snr_db << "\n";
  }
  std::cout << "\n";

  // --- View 3: EosDetectionLifecycleRecorder (lifecycle events) ---
  const auto& events = eos.lifecycleEvents();
  std::cout << "[View 3] EosDetectionLifecycleRecorder — detection "
               "lifecycle events\n"
            << "  Events this cycle: " << events.size() << "\n";

  for (std::size_t k = 0; k < events.size(); ++k) {
    const auto& e = events[k];
    const char* kind_str = "";
    switch (e.kind) {
      case eos_session::EosDetectionLifecycleEventKind::kFirstDetected:
        kind_str = "FirstDetected";
        break;
      case eos_session::EosDetectionLifecycleEventKind::kUpdated:
        kind_str = "Updated";
        break;
      case eos_session::EosDetectionLifecycleEventKind::kLost:
        kind_str = "Lost";
        break;
      case eos_session::EosDetectionLifecycleEventKind::kNotDetected:
        kind_str = "NotDetected";
        break;
    }
    std::cout << "    [" << k << "] cycle=" << e.cycle_index
              << " target_id=" << e.target_id << " kind=" << kind_str
              << " SNR_db=" << e.fused_snr_db << " range=" << e.range_m
              << "\n";
  }
  std::cout << "\n";

  // --- Auxiliary view: EosCycleOutputAdapter (ECEF external coordinates) ---
  eos_session::EosExternalPoseInput last_platform;
  last_platform.platform_position_ecef_m = platform_pos;
  last_platform.platform_velocity_mps = platform_vel;
  last_platform.platform_attitude_deg.yaw_deg = 0.0;
  last_platform.platform_attitude_deg.pitch_deg = 0.0;
  last_platform.platform_attitude_deg.roll_deg = 0.0;

  eos_session::EosExternalOutputFrame ext_output;
  if (eos.buildExternalOutput(last_platform, &ext_output)) {
    std::cout << "[Auxiliary] EosCycleOutputAdapter — ECEF external "
                 "coordinate conversion\n"
              << "  External detections: " << ext_output.detections.size()
              << "\n";
    for (std::size_t k = 0; k < ext_output.detections.size(); ++k) {
      const auto& det = ext_output.detections[k];
      std::cout << "    [" << k << "] id=" << det.detection_id
                << " ecef=(" << det.target_position_ecef_m.x_m << ","
                << det.target_position_ecef_m.y_m << ","
                << det.target_position_ecef_m.z_m << ")"
                << " range=" << det.range_m
                << " az=" << det.azimuth_deg << "deg"
                << " el=" << det.elevation_deg << "deg"
                << " SNR_db=" << det.fused_snr_db
                << " detected=" << (det.detected ? "yes" : "no") << "\n";
    }
  }
  std::cout << "\n";

  // ===============================================
  // Replay API demonstration
  // ===============================================
  std::cout << "====== Replay API ======\n"
            << "  enableTrace() — call before preStart to enable trace recording:\n"
            << "    eos.enableTrace(\"/tmp/eos_trace\");\n"
            << "  replayTrace() — post-hoc replay of a recorded trace:\n"
            << "    auto replay_result = EosModule::replayTrace(\"/tmp/eos_trace\");\n"
            << "    replay_result.ok = ...\n"
            << "    replay_result.report.total_events = ...\n\n";

  std::cout << "=== Demo Complete ===\n";
  return 0;
}
