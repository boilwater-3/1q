/**
 * @file integration_demo.cpp
 * @brief 电子侦察集成示例 — 展示 EsrModule 在外部引擎中的使用方式。
 *
 * @par 场景描述
 * 模拟外部仿真引擎的集成模式，演示 EsrModule 的完整生命周期。
 * 引擎通过订阅者模式的回调机制在运行期注入配置变更，
 * 验证平铺配置 getter 对初始配置的扁平访问。
 *
 * @par 关键概念
 * - 三阶段生命周期：initialize → preStart → stepImp
 * - 配置平铺 (Config Flattening)：层次化 EsrSessionConfig 展开为私有扁平成员
 * - 订阅者模式：registerConfigPatchCallback 注册回调，stepImp 每周期自动收集
 * - 文件配置保留：preStart 使用现有 config_loader.h 从 JSON 加载
 * - 三通道输出：观测通道、侦察假设通道、真值评估通道
 *
 * @par 运行方式
 *   cd examples/
 *   ../build/llvm-ninja-release-local/bin/esr_integration_demo
 *   或指定配置路径：
 *   ./esr_integration_demo ../configs/electronic_warfare.json
 */

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "EsrModule.h"

// =============================================================================
// 第一部分：模拟外部引擎运行期配置管理器
// =============================================================================
//
// 该管理器通过 EsrModule 的订阅者模式注入运行期配置变更。
// 在真实集成中，此类可能监听 DDS 主题、UI 控制面板或上级指挥系统的指令。
// =============================================================================

/// 配置变更阶段枚举。
enum class ConfigPhase {
  kNormalEsm,     ///< 常规 ESM 侦察（默认）
  kHgesmEngage,   ///< 切换高增益 ESM（第 20 周期触发）
  kHgesmActive,   ///< 高增益 ESM 模式持续中
  kRestoreEsm,    ///< 恢复常规 ESM（第 35 周期触发）
  kEsmBack,       ///< 恢复正常侦察
  kSensorOff,     ///< 关闭传感器（第 45 周期触发）
  kDone           ///< 结束
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
   *  该函数作为 EsrModule::ConfigPatchCallback 注册，
   *  每周期被 stepImp 自动调用。它在以下时机注入配置变更：
   *    - 第 20 周期：切换到 HGESM 高增益侦察模式
   *    - 第 35 周期：恢复常规 ESM 侦察模式
   *    - 第 45 周期：关闭传感器（模拟应急操作）
   *
   *  @param[in]  cycle  当前周期号（从 1 开始）
   *  @param[out] patch  待填充的配置补丁
   */
  void evaluatePatch(std::uint32_t cycle,
                     esr_config::EsrRuntimeConfigPatch& patch) {
    switch (phase_) {
      case ConfigPhase::kNormalEsm:
        if (cycle >= 20) {
          phase_ = ConfigPhase::kHgesmEngage;
          fillHgesmPatch(patch);
        }
        break;

      case ConfigPhase::kHgesmEngage:
        phase_ = ConfigPhase::kHgesmActive;
        break;

      case ConfigPhase::kHgesmActive:
        if (cycle >= 35) {
          phase_ = ConfigPhase::kRestoreEsm;
          fillEsmPatch(patch);
        }
        break;

      case ConfigPhase::kRestoreEsm:
        phase_ = ConfigPhase::kEsmBack;
        break;

      case ConfigPhase::kEsmBack:
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
  void fillHgesmPatch(esr_config::EsrRuntimeConfigPatch& patch) {
    patch.has_work_mode = true;
    patch.work_mode = esr_config::EsrWorkMode::kHgesm;
    patch.has_scan_rate_hz = true;
    patch.scan_rate_hz = 2.0f;
    last_patch_applied_ = true;
  }

  void fillEsmPatch(esr_config::EsrRuntimeConfigPatch& patch) {
    patch.has_work_mode = true;
    patch.work_mode = esr_config::EsrWorkMode::kEsm;
    patch.has_scan_rate_hz = true;
    patch.scan_rate_hz = 1.0f;
    last_patch_applied_ = true;
  }

  void fillSensorOffPatch(esr_config::EsrRuntimeConfigPatch& patch) {
    patch.has_sensor_enabled = true;
    patch.sensor_enabled = false;
    last_patch_applied_ = true;
  }

  ConfigPhase phase_{ConfigPhase::kNormalEsm};
  bool last_patch_applied_{false};
};

// =============================================================================
// 第二部分：辅助函数 — 仿真场景构造
// =============================================================================

namespace {

/// 辐射源运动数据。
struct MovingEmitter {
  std::uint64_t id;
  double x_m, y_m, z_m;
  double vx_mps, vy_mps, vz_mps;
  double carrier_hz;
  double bandwidth_hz;
  double tx_power_w;
};

MovingEmitter MakeEmitter(std::uint64_t id, double x, double y, double z,
                          double vx, double vy, double vz,
                          double carrier_hz, double bandwidth_hz,
                          double tx_power_w) {
  return {id, x, y, z, vx, vy, vz, carrier_hz, bandwidth_hz, tx_power_w};
}

esr_session::EsrExternalEmitterInput ToExternalInput(const MovingEmitter& me) {
  esr_session::EsrExternalEmitterInput e;
  e.emitter_id = me.id;
  e.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  e.kinematics.position_ecef_m.x_m = me.x_m;
  e.kinematics.position_ecef_m.y_m = me.y_m;
  e.kinematics.position_ecef_m.z_m = me.z_m;
  e.kinematics.velocity_mps.x_mps = me.vx_mps;
  e.kinematics.velocity_mps.y_mps = me.vy_mps;
  e.kinematics.velocity_mps.z_mps = me.vz_mps;
  e.carrier_hz = me.carrier_hz;
  e.bandwidth_hz = me.bandwidth_hz;
  e.tx_power_w = me.tx_power_w;
  e.pulse_width_s = 1.0e-6;
  e.pri_s = 1.0e-4;
  e.is_emitting = true;
  return e;
}

}  // namespace

// =============================================================================
// 第三部分：主函数
// =============================================================================

/**
 * @brief 入口：运行电子侦察集成示例。
 *
 * @param[in] argc  参数个数
 * @param[in] argv  argv[1] 可选，JSON 配置文件路径
 * @return 0  成功，1 失败
 */
int main(int argc, char** argv) {
  const std::string config_path =
      (argc > 1) ? argv[1] : "configs/electronic_warfare.json";
  constexpr std::uint32_t kNumCycles = 50;

  std::cout << "=== 电子侦察集成模块 Demo ===\n"
            << "  配置文件: " << config_path << "\n"
            << "  总周期数: " << kNumCycles << "\n\n";

  // ===============================================
  // 步骤 1：创建 EsrModule
  // ===============================================
  // 构造函数仅初始化默认值，不进行重量级操作。
  std::cout << "[1/6] 构造 EsrModule...\n";
  EsrModule esr;

  // ===============================================
  // 步骤 2：initialize — 初始化内部状态
  // ===============================================
  // 创建 EsrSession 实例（使用默认空配置）。
  std::cout << "[2/6] initialize()...\n";
  if (!esr.initialize()) {
    std::cerr << "  ERROR: initialize() 失败\n";
    return 1;
  }
  std::cout << "  EsrSession 已创建\n";

  // ===============================================
  // 步骤 3：preStart — 从文件加载配置并平铺
  // ===============================================
  // 从 JSON 读取四域配置（hardware/mission/policy/environment），
  // 将所有叶节点参数平铺至 EsrModule 的私有成员。
  // 此步骤还会用完整配置重建 EsrSession。
  std::cout << "[3/6] preStart(\"" << config_path << "\")...\n";
  if (!esr.preStart(config_path)) {
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

  // 主回调：通过闭包捕获当前周期号。
  // 在真实集成中，引擎通常知道当前仿真时间或周期号。
  esr.registerConfigPatchCallback(
      [&esr, &config_mgr](esr_config::EsrRuntimeConfigPatch& patch) {
        // 使用 esr.cycleCount() + 1 获取即将执行的周期号
        // （stepImp 在收集回调时尚未递增 cycle_index_）
        config_mgr.evaluatePatch(esr.cycleCount() + 1, patch);
      });

  std::cout << "  已注册回调（SimulatedConfigManager 就绪）\n";

  // ===============================================
  // 步骤 5：主仿真循环
  // ===============================================
  // 每周期调用 stepImp(input) 驱动电子侦察仿真。
  // stepImp 内部自动执行：
  //   1. 收集所有注册回调 → 合成 EsrRuntimeConfigPatch
  //   2. 如有有效补丁，调用 TryApplyRuntimeConfig
  //   3. 执行 StepWithResult
  //   4. 缓存结果至 lastResult()
  std::cout << "[5/6] 主仿真循环（" << kNumCycles << " 周期）...\n\n";

  // 平台初始位置（ECEF 坐标）
  oneq::coordinate::EcefPositionM platform_pos;
  platform_pos.x_m = -2289512.0;
  platform_pos.y_m = 4909946.0;
  platform_pos.z_m = 3640982.0 + 9000.0;

  oneq::coordinate::EcefVelocityMps platform_vel;
  platform_vel.x_mps = 120.0;
  platform_vel.y_mps = -80.0;
  platform_vel.z_mps = 30.0;

  // 3 个空中运动辐射源
  std::vector<MovingEmitter> emitters;
  emitters.push_back(
      MakeEmitter(1001,
                  -2289512.0, 4909946.0 + 12000.0, 3640982.0 + 5000.0,
                  -20.0, 0.0, 0.0,
                  10.0e9, 2.0e6, 5.0e7));
  emitters.push_back(
      MakeEmitter(1002,
                  -2289512.0, 4909946.0 + 18000.0, 3640982.0 + 5000.0,
                  -15.0, 0.0, 0.0,
                  9.4e9, 2.0e6, 5.0e7));
  emitters.push_back(
      MakeEmitter(1003,
                  -2289512.0, 4909946.0 + 25000.0, 3640982.0 + 5000.0,
                  -10.0, 0.0, 0.0,
                  9.8e9, 2.0e6, 5.0e7));

  // 统计量
  std::size_t max_observations = 0;
  std::size_t min_observations = 100;
  std::size_t max_hypotheses = 0;
  std::uint32_t patch_applied_count = 0;

  // 构造固定环境输入（传播环境描述）
  esr_session::EsrEnvironmentInput env_input;
  env_input.propagation_profile = esr_session::EsrPropagationEnvironmentProfile::kOpen;
  env_input.clutter_density = esr_session::EsrClutterDensityLevel::kMedium;
  env_input.spectrum_occupancy_ratio = 0.25f;

  for (std::uint32_t i = 0; i < kNumCycles; ++i) {
    // ---- 构造平台位姿 ----
    esr_session::EsrExternalPoseInput platform;
    platform.platform_position_ecef_m = platform_pos;
    platform.platform_velocity_mps = platform_vel;
    platform.platform_attitude_deg.yaw_deg = 0.0;
    platform.platform_attitude_deg.pitch_deg = 0.0;
    platform.platform_attitude_deg.roll_deg = 0.0;

    // ---- 构造辐射源输入 ----
    std::vector<esr_session::EsrExternalEmitterInput> emitter_inputs;
    emitter_inputs.reserve(emitters.size());
    for (const auto& me : emitters) {
      emitter_inputs.push_back(ToExternalInput(me));
    }

    // ---- 构造 EsrCycleInput ----
    esr_session::EsrCycleInput input;
    esr_session::EsrCoordinateStatus status;
    if (!esr_session::EsrCycleInputAdapter::Build(
            platform, emitter_inputs, 1.0f, env_input, &input, &status)) {
      std::cerr << "  周期 " << (i + 1)
                << "：EsrCycleInputAdapter::Build 失败（status="
                << static_cast<int>(status) << "）\n";
      return 1;
    }
    input.cycle_index = i + 1;

    // ---- 推进辐射源位置（简单欧拉积分） ----
    const float dt = input.dt_sec;
    for (auto& me : emitters) {
      me.x_m += me.vx_mps * dt;
      me.y_m += me.vy_mps * dt;
      me.z_m += me.vz_mps * dt;
    }

    // === 【核心】：通过 stepImp 驱动周期 ===
    // 使用 EsrModule::stepImp 而非直接 session().StepWithResult，
    // 确保回调收集 → 补丁应用 → 执行 → 结果缓存的完整流程。
    // stepImp 在此处自动调用注册的 SimulatedConfigManager 回调。
    config_mgr.clearLastPatchFlag();
    esr.stepImp(input);

    // 检查是否本次触发了配置变更
    if (config_mgr.lastPatchApplied()) {
      ++patch_applied_count;
      std::cout << "  第 " << (i + 1) << " 周期："
                << "→ 配置变更已通过回调注入 stepImp\n";
    }

    // ---- 统计 ----
    const auto& result = esr.lastResult();
    std::size_t nobs = result.output_frame.observation_output.observations.size();
    std::size_t nhyp = result.output_frame.emitter_output.hypotheses.size();
    if (nobs > max_observations) max_observations = nobs;
    if (nobs < min_observations) min_observations = nobs;
    if (nhyp > max_hypotheses) max_hypotheses = nhyp;

    // 每 5 周期输出一次
    if ((i + 1) % 5 == 0) {
      std::size_t nassoc = result.output_frame.truth_evaluation_output.associations.size();
      std::cout << "  【周期 " << (i + 1) << "/" << kNumCycles << "】"
                << " 观测=" << nobs
                << " 假设=" << nhyp
                << " 关联=" << nassoc
                << (result.executed_this_cycle ? "" : " [未执行]")
                << "\n";
    }
  }

  // ===============================================
  // 步骤 6：结果输出与验证
  // ===============================================
  std::cout << "\n[6/6] 仿真结果汇总\n"
            << "  总周期数: " << kNumCycles << "\n"
            << "  配置变更注入次数: " << patch_applied_count << "\n"
            << "  最小观测数: " << min_observations << "\n"
            << "  最大观测数: " << max_observations << "\n"
            << "  最大假设数: " << max_hypotheses << "\n"
            << "  当前周期号: " << esr.cycleCount() << "\n\n";

  // 打印完整配置摘要（通过 EsrModule 内部方法）
  esr.printConfigSummary();

  // ===============================================
  // 输出三视图展示
  // ===============================================
  std::cout << "\n====== 输出三视图 (Last Cycle) ======\n";

  // --- 视图一：EsrOutputFrame（系统输出）---
  const auto& final_result = esr.lastResult();
  std::cout << "[视图一] EsrOutputFrame — 系统侧三通道输出\n"
            << "  执行周期: " << final_result.input_cycle_index << "\n"
            << "  观测通道: "
            << final_result.output_frame.observation_output.observations.size()
            << "\n"
            << "  假设通道: "
            << final_result.output_frame.emitter_output.hypotheses.size()
            << "\n"
            << "  真值评估通道: "
            << final_result.output_frame.truth_evaluation_output.associations.size()
            << "\n";

  // 打印每个观测的详细信息
  std::cout << "  --- 观测记录 ---\n";
  for (std::size_t k = 0; k < final_result.output_frame.observation_output.observations.size(); ++k) {
    const auto& obs = final_result.output_frame.observation_output.observations[k];
    std::cout << "    [" << k << "] id=" << obs.observation_id
              << " az=" << obs.aoa_az_deg << "deg"
              << " el=" << obs.aoa_el_deg << "deg"
              << " rf_hz=" << obs.rf_hz
              << " snr_db=" << obs.snr_db
              << " pw_us=" << (obs.pulse_width_s * 1.0e6)
              << "\n";
  }

  // 打印每个假设的详细信息
  std::cout << "  --- 侦察假设 ---\n";
  for (std::size_t k = 0; k < final_result.output_frame.emitter_output.hypotheses.size(); ++k) {
    const auto& hyp = final_result.output_frame.emitter_output.hypotheses[k];
    const char* mode_str = "";
    switch (hyp.mode) {
      case esr_session::EsrEmitterMode::kUnknown:  mode_str = "未知"; break;
      case esr_session::EsrEmitterMode::kSearch:   mode_str = "搜索"; break;
      case esr_session::EsrEmitterMode::kTracking: mode_str = "跟踪"; break;
      case esr_session::EsrEmitterMode::kGuidance: mode_str = "制导"; break;
    }
    const char* threat_str = "";
    switch (hyp.threat_level) {
      case esr_session::EsrThreatLevel::kLow:   threat_str = "低"; break;
      case esr_session::EsrThreatLevel::kMedium: threat_str = "中"; break;
      case esr_session::EsrThreatLevel::kHigh:   threat_str = "高"; break;
    }
    std::cout << "    [" << k << "] id=" << hyp.hypothesis_id
              << " mode=" << mode_str
              << " threat=" << threat_str
              << " confidence=" << hyp.confidence
              << " az=" << hyp.bearing_az_deg << "deg"
              << " el=" << hyp.bearing_el_deg << "deg"
              << " last_seen=" << hyp.last_seen_cycle
              << "\n";
  }

  std::cout << "  校验错误: " << (final_result.has_validation_error ? "是" : "否") << "\n"
            << "  执行成功: " << (final_result.executed_this_cycle ? "是" : "否") << "\n\n";

  // --- 视图二：EsrOutputDebugView（调试视图）---
  esr_session::EsrOutputDebugView debug_view = esr.buildLastDebugView();
  std::cout << "[视图二] EsrOutputDebugView — 人读排查视图\n"
            << "  输入周期: " << debug_view.input_cycle_index << "\n"
            << "  输出周期: " << debug_view.output_cycle_index << "\n"
            << "  执行: " << (debug_view.executed_this_cycle ? "是" : "否")
            << " 复用: " << (debug_view.reused_previous_output ? "是" : "否") << "\n"
            << "  辐射源状态:\n";
  for (std::size_t k = 0; k < debug_view.emitters.size(); ++k) {
    const auto& e = debug_view.emitters[k];
    const char* status_str = "";
    switch (e.status) {
      case esr_session::EsrDebugEmitterStatus::kObserved:
        status_str = "已观测"; break;
      case esr_session::EsrDebugEmitterStatus::kNotObserved:
        status_str = "未观测"; break;
      case esr_session::EsrDebugEmitterStatus::kNotEmitting:
        status_str = "未发射"; break;
      case esr_session::EsrDebugEmitterStatus::kCycleNotExecuted:
        status_str = "周期未执行"; break;
    }
    std::cout << "    [" << k << "] id=" << e.emitter_id
              << " status=" << status_str
              << " present_in_input=" << (e.present_in_input ? "是" : "否")
              << " matched_obs=" << (e.matched_observation ? "是" : "否")
              << " obs_id=" << e.observation_id
              << " confidence=" << e.confidence
              << "\n";
  }
  std::cout << "\n";

  // --- 视图三：EsrEmitterLifecycleRecorder（生命周期事件）---
  const auto& events = esr.lifecycleEvents();
  std::cout << "[视图三] EsrEmitterLifecycleRecorder — 辐射源生命周期事件\n"
            << "  本周期事件数: " << events.size() << "\n";
  for (std::size_t k = 0; k < events.size(); ++k) {
    const auto& ev = events[k];
    const char* kind_str = "";
    switch (ev.kind) {
      case esr_session::EsrEmitterLifecycleEventKind::kFirstObserved:
        kind_str = "首次观测"; break;
      case esr_session::EsrEmitterLifecycleEventKind::kUpdated:
        kind_str = "持续更新"; break;
      case esr_session::EsrEmitterLifecycleEventKind::kLost:
        kind_str = "丢失"; break;
      case esr_session::EsrEmitterLifecycleEventKind::kNotObserved:
        kind_str = "未观测"; break;
    }
    const char* reason_str = "";
    switch (ev.reason) {
      case esr_session::EsrEmitterLifecycleReason::kNone:
        reason_str = ""; break;
      case esr_session::EsrEmitterLifecycleReason::kNotEmitting:
        reason_str = "未发射"; break;
      case esr_session::EsrEmitterLifecycleReason::kNoMatchedObservation:
        reason_str = "无匹配观测"; break;
      case esr_session::EsrEmitterLifecycleReason::kUnknown:
        reason_str = "未知"; break;
    }
    std::cout << "    [" << k << "] cycle=" << ev.cycle_index
              << " emitter_id=" << ev.emitter_id
              << " kind=" << kind_str
              << " reason=" << reason_str
              << " obs_id=" << ev.observation_id
              << " confidence=" << ev.confidence
              << "\n";
  }
  std::cout << "\n";

  // --- 辅助视图：EsrCycleOutputAdapter（ECEF 外部坐标转换）---
  // ESR 输出没有距离量，外部输出为 ECEF 单位方位线
  esr_session::EsrExternalPoseInput last_platform;
  last_platform.platform_position_ecef_m.x_m = -2289512.0;
  last_platform.platform_position_ecef_m.y_m = 4909946.0;
  last_platform.platform_position_ecef_m.z_m = 3640982.0 + 9000.0;
  last_platform.platform_velocity_mps.x_mps = 120.0;
  last_platform.platform_velocity_mps.y_mps = -80.0;
  last_platform.platform_velocity_mps.z_mps = 30.0;

  esr_session::EsrExternalOutputFrame ext_output;
  if (esr.buildExternalOutput(last_platform, &ext_output)) {
    std::cout << "[辅助] EsrCycleOutputAdapter — ECEF 外部坐标转换\n"
              << "  外部观测数: " << ext_output.observations.size() << "\n"
              << "  外部假设数: " << ext_output.hypotheses.size() << "\n";
    for (std::size_t k = 0; k < ext_output.observations.size(); ++k) {
      const auto& eo = ext_output.observations[k];
      std::cout << "    [观测" << k << "] id=" << eo.observation_id
                << " bearing=(" << eo.bearing_unit_ecef.x << ","
                << eo.bearing_unit_ecef.y << ","
                << eo.bearing_unit_ecef.z << ")"
                << " rf_hz=" << eo.rf_hz
                << " snr_db=" << eo.snr_db << "\n";
    }
    for (std::size_t k = 0; k < ext_output.hypotheses.size(); ++k) {
      const auto& eh = ext_output.hypotheses[k];
      std::cout << "    [假设" << k << "] id=" << eh.hypothesis_id
                << " bearing_az=" << eh.bearing_az_deg << "deg"
                << " bearing_el=" << eh.bearing_el_deg << "deg"
                << " confidence=" << eh.confidence
                << " std=" << eh.bearing_std_deg << "deg\n";
    }
  }
  std::cout << "\n";

  // ===============================================
  // Replay API 演示
  // ===============================================
  std::cout << "====== Replay API ======\n"
            << "  enableTrace() 演示 — 在 preStart 前调用以开启 trace 录制：\n"
            << "    esr.enableTrace(\"/tmp/esr_trace\");\n"
            << "  replayTrace() 演示 — 事后回放已录制的 trace：\n"
            << "    auto replay_result = EsrModule::replayTrace(\"/tmp/esr_trace\");\n"
            << "    replay_result.ok = ...\n"
            << "    replay_result.report.total_events = ...\n\n";

  std::cout << "=== Demo 完成 ===\n";
  return 0;
}
