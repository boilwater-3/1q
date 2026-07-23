/**
 * @file integration_demo.cpp
 * @brief 机载雷达集成示例 — 展示 RadarModule 在外部引擎中的使用方式。
 *
 * @par 场景描述
 * 模拟外部仿真引擎的集成模式，演示 RadarModule 的完整生命周期。
 * 引擎通过订阅者模式的回调机制在运行期注入配置变更，
 * 验证平铺配置 getter 对初始配置的扁平访问。
 *
 * @par 关键概念
 * - 三阶段生命周期：initialize → preStart → stepImp
 * - 配置平铺 (Config Flattening)：层次化 ArSessionConfig 展开为私有扁平成员
 * - 订阅者模式：registerConfigPatchCallback 注册回调，stepImp 每周期自动收集
 * - 文件配置保留：preStart 使用现有 config_loader.h 从 JSON 加载
 *
 * @par 运行方式
 *   cd examples/
 *   ../build/llvm-ninja-release-local/bin/radar_integration_demo
 *   或指定配置路径：
 *   ./radar_integration_demo ../configs/airborne_radar.json
 */

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "RadarModule.h"

// =============================================================================
// 第一部分：模拟外部引擎运行期配置管理器
// =============================================================================
//
// 该管理器通过 RadarModule 的订阅者模式注入运行期配置变更。
// 在真实集成中，此类可能监听 DDS 主题、UI 控制面板或上级指挥系统的指令。
// =============================================================================

/// 配置变更阶段枚举。
enum class ConfigPhase {
  kNormalTws,   ///< 正常 TWS 搜索跟踪（默认）
  kSttEngage,   ///< 单目标跟踪介入（第 20 周期触发）
  kSttActive,   ///< STT 模式持续中
  kRestoreTws,  ///< 恢复 TWS 模式（第 35 周期触发）
  kTwsBack,     ///< 恢复正常跟踪
  kSensorOff,   ///< 关闭传感器（第 45 周期触发）
  kDone         ///< 结束
};

/**
 * @brief 模拟外部引擎的运行时配置管理器。
 *
 * 在 stepImp 的回调链中注入运行期配置补丁。
 * 使用简单状态机按周期号切换工作模式。
 */
class SimulatedConfigManager {
 public:
  /** @brief 生成运行期配置补丁。
   *
   *  该函数作为 RadarModule::ConfigPatchCallback 注册，
   *  每周期被 stepImp 自动调用。它在以下时机注入配置变更：
   *    - 第 20 周期：切换到 STT 单目标跟踪模式
   *    - 第 35 周期：恢复 TWS 边扫边跟踪模式
   *    - 第 45 周期：关闭传感器（模拟应急操作）
   *
   *  @param[in]  cycle  当前周期号（从 1 开始）
   *  @param[out] patch  待填充的配置补丁
   */
  void evaluatePatch(std::uint32_t cycle,
                     ar_config::ArRuntimeConfigPatch& patch) {
    switch (phase_) {
      case ConfigPhase::kNormalTws:
        if (cycle >= 20) {
          phase_ = ConfigPhase::kSttEngage;
          fillSttPatch(patch);
        }
        break;

      case ConfigPhase::kSttEngage:
        phase_ = ConfigPhase::kSttActive;
        break;

      case ConfigPhase::kSttActive:
        if (cycle >= 35) {
          phase_ = ConfigPhase::kRestoreTws;
          fillTwsPatch(patch);
        }
        break;

      case ConfigPhase::kRestoreTws:
        phase_ = ConfigPhase::kTwsBack;
        break;

      case ConfigPhase::kTwsBack:
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

  /** @brief 最近一次是否实际注入了补丁。 */
  bool lastPatchApplied() const { return last_patch_applied_; }

  /** @brief 重置最近一次补丁标记。 */
  void clearLastPatchFlag() { last_patch_applied_ = false; }

 private:
  void fillSttPatch(ar_config::ArRuntimeConfigPatch& patch) {
    patch.has_work_mode = true;
    patch.work_mode = ar_config::ArWorkMode::kStt;
    patch.has_scan_center_deg = true;
    patch.scan_center_deg.az_deg = 5.0f;
    patch.scan_center_deg.el_deg = 2.0f;
    last_patch_applied_ = true;
  }

  void fillTwsPatch(ar_config::ArRuntimeConfigPatch& patch) {
    patch.has_work_mode = true;
    patch.work_mode = ar_config::ArWorkMode::kTws;
    patch.has_scan_center_deg = true;
    patch.scan_center_deg.az_deg = 0.0f;
    patch.scan_center_deg.el_deg = 0.0f;
    last_patch_applied_ = true;
  }

  void fillSensorOffPatch(ar_config::ArRuntimeConfigPatch& patch) {
    patch.has_sensor_enabled = true;
    patch.sensor_enabled = false;
    last_patch_applied_ = true;
  }

  ConfigPhase phase_{ConfigPhase::kNormalTws};
  bool last_patch_applied_{false};
};

// =============================================================================
// 第二部分：辅助函数 — 仿真场景构造
// =============================================================================

namespace {

/// 目标运动数据。
struct MovingTarget {
  std::uint64_t id;
  double x_m, y_m, z_m;
  double vx_mps, vy_mps, vz_mps;
  float rcs;
};

MovingTarget MakeTarget(std::uint64_t id, double x, double y, double z,
                        double vx, double vy, double vz, float rcs) {
  return {id, x, y, z, vx, vy, vz, rcs};
}

ar_session::ArExternalTargetInput ToExternalInput(const MovingTarget& mt) {
  ar_session::ArExternalTargetInput t;
  t.target_id = mt.id;
  t.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  t.kinematics.position_ecef_m.x_m = mt.x_m;
  t.kinematics.position_ecef_m.y_m = mt.y_m;
  t.kinematics.position_ecef_m.z_m = mt.z_m;
  t.kinematics.velocity_mps.x_mps = mt.vx_mps;
  t.kinematics.velocity_mps.y_mps = mt.vy_mps;
  t.kinematics.velocity_mps.z_mps = mt.vz_mps;
  t.rcs = mt.rcs;
  t.swerling_type = 0;
  return t;
}

}  // namespace

// =============================================================================
// 第三部分：主函数
// =============================================================================

/**
 * @brief 入口：运行机载雷达集成示例。
 *
 * @param[in] argc  参数个数
 * @param[in] argv  argv[1] 可选，JSON 配置文件路径
 * @return 0  成功，1 失败
 */
int main(int argc, char** argv) {
  const std::string config_path =
      (argc > 1) ? argv[1] : "configs/airborne_radar.json";
  constexpr std::uint32_t kNumCycles = 50;

  std::cout << "=== 机载雷达集成模块 Demo ===\n"
            << "  配置文件: " << config_path << "\n"
            << "  总周期数: " << kNumCycles << "\n\n";

  // ===============================================
  // 步骤 1：创建 RadarModule
  // ===============================================
  // 构造函数仅初始化默认值，不进行重量级操作。
  std::cout << "[1/6] 构造 RadarModule...\n";
  RadarModule radar;

  // ===============================================
  // 步骤 2：initialize — 初始化内部状态
  // ===============================================
  // 创建 ArSession 实例（使用默认空配置）。
  std::cout << "[2/6] initialize()...\n";
  if (!radar.initialize()) {
    std::cerr << "  ERROR: initialize() 失败\n";
    return 1;
  }
  std::cout << "  ArSession 已创建\n";

  // ===============================================
  // 步骤 3：preStart — 从文件加载配置并平铺
  // ===============================================
  // 从 JSON 读取四域配置（hardware/mission/policy/environment），
  // 将所有叶节点参数平铺至 RadarModule 的私有成员。
  // 此步骤还会用完整配置重建 ArSession。
  std::cout << "[3/6] preStart(\"" << config_path << "\")...\n";
  if (!radar.preStart(config_path)) {
    std::cerr << "  ERROR: preStart() 失败\n";
    return 1;
  }
  std::cout << "  配置加载并平铺完成\n"
            << "  配置已通过 printConfigSummary() 可查\n";

  // ===============================================
  // 步骤 4：注册运行期配置回调（订阅者模式）
  // ===============================================
  // 引擎通过 registerConfigPatchCallback 注册回调函数。
  // 这些回调会在每次 stepImp 开始时自动收集，合成为运行时配置补丁。
  //
  // 【核心模式】引擎无需在循环中手动调用 ApplyRuntimeConfig，
  // 只需注册回调，stepImp 会自动处理收集→应用流程。
  std::cout << "[4/6] 注册运行期配置回调（订阅者模式）...\n";

  SimulatedConfigManager config_mgr;

  radar.registerConfigPatchCallback(
      [&config_mgr](ar_config::ArRuntimeConfigPatch& patch) {
        // 注意：此回调不直接知道周期号，但 SimulatedConfigManager
        // 内部会跟踪已被调用的阶段。实际调用时由 stepImp 传入。
        // 此处使用静态周期计数器的替代方案：config_mgr 的内部状态机
        // 在 evaluatePatch 中被驱动，而我们通过雷达对象的 cycleCount
        // 来获取当前周期号。由于回调签名限制，这里需要一个闭包技巧。
        // 我们用另一种方式实现 —— 见下方第二个回调。
        (void)patch;
        (void)config_mgr;
      });

  // 主回调：通过闭包捕获当前周期号。
  // 在真实集成中，引擎通常知道当前仿真时间或周期号。
  radar.registerConfigPatchCallback(
      [&radar, &config_mgr](ar_config::ArRuntimeConfigPatch& patch) {
        // 使用 radar.cycleCount() + 1 获取即将执行的周期号
        // （stepImp 在收集回调时尚未递增 cycle_index_）
        config_mgr.evaluatePatch(radar.cycleCount() + 1, patch);
      });

  std::cout << "  已注册回调链（SimulatedConfigManager 就绪）\n";

  // ===============================================
  // 步骤 5：主仿真循环
  // ===============================================
  // 每周期调用 stepImp(input) 驱动雷达仿真。
  // stepImp 内部自动执行：
  //   1. 收集所有注册回调 → 合成 ArRuntimeConfigPatch
  //   2. 如有有效补丁，调用 TryApplyRuntimeConfig
  //   3. 执行 StepWithResult
  //   4. 缓存结果至 lastResult()
  std::cout << "[5/6] 主仿真循环（" << kNumCycles << " 周期）...\n\n";

  // 平台初始位置（ECEF 坐标）
  oneq::coordinate::EcefPositionM platform_pos;
  platform_pos.x_m = -2289512.0;
  platform_pos.y_m = 4909946.0;
  platform_pos.z_m = 3640982.0;

  oneq::coordinate::EcefVelocityMps platform_vel;
  platform_vel.x_mps = 120.0;
  platform_vel.y_mps = -80.0;
  platform_vel.z_mps = 30.0;

  // 3 个空中运动目标
  std::vector<MovingTarget> targets;
  targets.push_back(
      MakeTarget(1001, -2289512.0 + 18000.0, 4909946.0 + 2500.0,
                 3640982.0 + 1200.0, -120.0, 8.0, 0.0, 2.2f));
  targets.push_back(
      MakeTarget(1002, -2289512.0 + 24000.0, 4909946.0 - 4000.0,
                 3640982.0 + 2000.0, -90.0, -12.0, 0.0, 1.4f));
  targets.push_back(
      MakeTarget(1003, -2289512.0 + 30000.0, 4909946.0 + 1000.0,
                 3640982.0 + 1500.0, -150.0, 0.0, -5.0, 3.0f));

  // 统计量
  std::uint32_t min_tracks = 100;
  std::uint32_t max_tracks = 0;
  std::uint32_t patch_applied_count = 0;

  for (std::uint32_t i = 0; i < kNumCycles; ++i) {
    // ---- 构造平台位姿 ----
    ar_session::ArExternalPoseInput platform;
    platform.platform_position_ecef_m = platform_pos;
    platform.platform_velocity_mps = platform_vel;
    platform.platform_attitude_deg.yaw_deg = 0.0;
    platform.platform_attitude_deg.pitch_deg = 0.0;
    platform.platform_attitude_deg.roll_deg = 0.0;
    platform.radar_mount_angles_deg.yaw_deg = 0.0;
    platform.radar_mount_angles_deg.pitch_deg = 0.0;
    platform.radar_mount_angles_deg.roll_deg = 0.0;

    // ---- 构造目标输入 ----
    std::vector<ar_session::ArExternalTargetInput> target_inputs;
    target_inputs.reserve(targets.size());
    for (const auto& mt : targets) {
      target_inputs.push_back(ToExternalInput(mt));
    }

    // ---- 构造 ArCycleInput ----
    ar_session::ArCycleInput input;
    ar_session::ArEnvironmentInput env;
    env.atmospheric_observation.enable_physical_model = false;
    env.atmospheric_context.has_simulation_unix_seconds = true;
    env.atmospheric_context.simulation_unix_seconds = 1770000000 + i;

    input.cycle_index = i + 1;
    input.cycle_start_time_s = static_cast<double>(i);
    input.dt_sec = 1.0;
    platform.platform_entity_id = 1U;
    input.platform = platform;
    input.targets = target_inputs;
    input.environment = env;

    // ---- 推进目标位置（简单欧拉积分） ----
    const double dt = input.dt_sec;
    for (auto& mt : targets) {
      mt.x_m += mt.vx_mps * dt;
      mt.y_m += mt.vy_mps * dt;
      mt.z_m += mt.vz_mps * dt;
    }

    // === 【核心】：通过 stepImp 驱动周期 ===
    // 使用 RadarModule::stepImp 而非直接 session().StepWithResult，
    // 确保回调收集 → 补丁应用 → 执行 → 结果缓存的完整流程。
    // stepImp 在此处自动调用注册的 SimulatedConfigManager 回调。
    config_mgr.clearLastPatchFlag();
    radar.stepImp(input);

    // 检查是否本次触发了配置变更
    if (config_mgr.lastPatchApplied()) {
      ++patch_applied_count;
      std::cout << "  第 " << (i + 1) << " 周期："
                << "→ 配置变更已通过回调注入 stepImp\n";
    }

    // ---- 统计 ----
    const auto& result = radar.lastResult();
    std::size_t ntracks = result.track_output_frame.tracks.size();
    if (ntracks > max_tracks) max_tracks = static_cast<std::uint32_t>(ntracks);
    if (ntracks < min_tracks) min_tracks = static_cast<std::uint32_t>(ntracks);

    // 每 5 周期输出一次
    if ((i + 1) % 5 == 0) {
      std::cout << "  【周期 " << (i + 1) << "/" << kNumCycles << "】"
                << " 航迹=" << ntracks
                << " 确认="
                << ar_session::CountTracksByStatus(
                       result.track_output_frame,
                       ar_session::TrackStatus::kConfirmed)
                << " 暂定="
                << ar_session::CountTracksByStatus(
                       result.track_output_frame,
                       ar_session::TrackStatus::kTentative)
                << (result.status == ar_session::ArCycleStatus::kCompleted
                        ? ""
                        : " [未完成]")
                << "\n";
    }
  }

  // ===============================================
  // 步骤 6：结果输出与验证
  // ===============================================
  std::cout << "\n[6/6] 仿真结果汇总\n"
            << "  总周期数: " << kNumCycles << "\n"
            << "  配置变更注入次数: " << patch_applied_count << "\n"
            << "  最小航迹数: " << min_tracks << "\n"
            << "  最大航迹数: " << max_tracks << "\n"
            << "  当前周期号: " << radar.cycleCount() << "\n\n";

  // 打印完整配置摘要（通过 RadarModule 内部方法）
  radar.printConfigSummary();

  // ===============================================
  // 输出三视图展示
  // ===============================================
  std::cout << "\n====== 输出三视图 (Last Cycle) ======\n";

  // --- 视图一：TrackOutputFrame（系统输出）---
  const auto& final_result = radar.lastResult();
  std::cout << "[视图一] TrackOutputFrame — 系统侧航迹输出\n"
            << "  执行周期: " << final_result.input_cycle_index << "\n"
            << "  航迹数: " << final_result.track_output_frame.tracks.size() << "\n";

  // 打印每个航迹的详细信息
  for (std::size_t k = 0; k < final_result.track_output_frame.tracks.size(); ++k) {
    const auto& t = final_result.track_output_frame.tracks[k];
    std::cout << "    [" << k << "] key=" << t.association_key
              << " status=" << static_cast<int>(t.status)
              << " pos=(" << t.position_x << "," << t.position_y
              << "," << t.position_z << ")"
              << " speed=" << t.speed << " rcs=" << t.rcs
              << " hits=" << t.hit_count << "\n";
  }
  std::cout << "  指令数: " << final_result.submitted_commands.size() << "\n"
            << "  校验错误: " << (final_result.has_validation_error ? "是" : "否") << "\n"
            << "  执行成功: "
            << (final_result.status == ar_session::ArCycleStatus::kCompleted
                    ? "是"
                    : "否")
            << "\n\n";

  // --- 视图二：ArTrackOutputDebugView（调试视图）---
  ar_session::ArTrackOutputDebugView debug_view = radar.buildLastDebugView();
  std::cout << "[视图二] ArTrackOutputDebugView — 人读排查视图\n"
            << "  世界周期: " << debug_view.world_cycle_index << "\n"
            << "  输出周期: " << debug_view.output_cycle_index << "\n"
            << "  完成: " << (debug_view.completed_this_cycle ? "是" : "否")
            << "\n"
            << "  轨迹状态:\n";
  for (std::size_t k = 0; k < debug_view.tracks.size(); ++k) {
    const auto& d = debug_view.tracks[k];
    std::cout << "    [" << k << "] ext_id=" << d.external_target_id
              << " status=" << static_cast<int>(d.status)
              << " has_track=" << (d.has_track ? "是" : "否")
              << " present_in_input=" << (d.present_in_input ? "是" : "否")
              << " speed=" << d.speed << " rcs=" << d.rcs
              << " hits=" << d.hit_count << " miss=" << d.miss_count << "\n";
  }
  std::cout << "\n";

  // --- 视图三：ArTrackLifecycleRecorder（生命周期事件）---
  const auto& events = radar.lifecycleEvents();
  std::cout << "[视图三] ArTrackLifecycleRecorder — 航迹生命周期事件\n"
            << "  本周期事件数: " << events.size() << "\n";
  for (std::size_t k = 0; k < events.size(); ++k) {
    const auto& e = events[k];
    const char* kind_str = "";
    switch (e.kind) {
      case ar_session::ArTrackLifecycleEventKind::kFirstConfirmed:
        kind_str = "首次确认"; break;
      case ar_session::ArTrackLifecycleEventKind::kUpdated:
        kind_str = "持续更新"; break;
      case ar_session::ArTrackLifecycleEventKind::kLost:
        kind_str = "丢失"; break;
      case ar_session::ArTrackLifecycleEventKind::kNotTracked:
        kind_str = "未跟踪"; break;
    }
    std::cout << "    [" << k << "] cycle=" << e.world_cycle_index
              << " target_id=" << e.external_target_id
              << " kind=" << kind_str
              << " key=" << e.association_key
              << " status=" << static_cast<int>(e.track_status)
              << "\n";
  }
  std::cout << "\n";

  // --- 辅助视图：ArCycleOutputAdapter（ECEF 外部坐标转换）---
  // 使用最后一个周期的平台位姿做 ECEF 转换演示
  ar_session::ArExternalPoseInput last_platform;
  last_platform.platform_position_ecef_m.x_m = -2289512.0;
  last_platform.platform_position_ecef_m.y_m = 4909946.0;
  last_platform.platform_position_ecef_m.z_m = 3640982.0;
  last_platform.platform_velocity_mps.x_mps = 120.0;
  last_platform.platform_velocity_mps.y_mps = -80.0;
  last_platform.platform_velocity_mps.z_mps = 30.0;

  ar_session::ArExternalTrackOutputFrame ext_output;
  if (radar.buildExternalOutput(last_platform, &ext_output)) {
    std::cout << "[辅助] ArCycleOutputAdapter — ECEF 外部坐标转换\n"
              << "  外部航迹数: " << ext_output.tracks.size() << "\n";
    for (std::size_t k = 0; k < ext_output.tracks.size(); ++k) {
      const auto& et = ext_output.tracks[k];
      std::cout << "    [" << k << "] ext_id=" << et.external_target_id
                << " status=" << static_cast<int>(et.status)
                << " ecef=(" << et.target_position_ecef_m.x_m << ","
                << et.target_position_ecef_m.y_m << ","
                << et.target_position_ecef_m.z_m << ")"
                << " speed=" << et.speed << " rcs=" << et.rcs << "\n";
    }
  }
  std::cout << "\n";

  // ===============================================
  // Replay API 演示
  // ===============================================
  std::cout << "====== Replay API ======\n"
            << "  enableTrace() 演示 — 在 preStart 前调用以开启 trace 录制：\n"
            << "    雷达.enableTrace(\"/tmp/ar_trace\");\n"
            << "  replayTrace() 演示 — 事后回放已录制的 trace：\n"
            << "    auto replay_result = RadarModule::replayTrace(\"/tmp/ar_trace\");\n"
            << "    replay_result.ok = ...\n"
            << "    replay_result.report.total_events = ...\n\n";

  std::cout << "=== Demo 完成 ===\n";
  return 0;
}
