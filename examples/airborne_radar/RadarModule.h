/**
 * @file RadarModule.h
 * @brief 机载雷达集成模块 — 适配外部仿真引擎的标准生命周期。
 *
 * @par 设计目标
 * 提供一种"复制即可用"的集成模式，使外部引擎（仿真框架、数字孪生平台等）
 * 能通过统一生命周期驱动机载雷达模块：
 *   - initialize() → 初始化内部状态
 *   - preStart()  → 从文件加载四域配置并平铺至私有成员
 *   - stepImp()   → 每周期执行雷达仿真
 *   - 订阅者模式    → 引擎可在运行期回调注入配置变更
 *
 * @par 典型使用流程
 * @code
 *   RadarModule radar;
 *   radar.initialize();
 *   radar.preStart("configs/airborne_radar.json");
 *
 *   // 注册运行期回调（可选）
 *   radar.registerConfigPatchCallback([](ArRuntimeConfigPatch& patch) {
 *     patch.has_work_mode = true;
 *     patch.work_mode = ArWorkMode::kStt;
 *   });
 *
 *   while (running) {
 *     radar.stepImp(dt);
 *     const auto& result = radar.lastResult();
 *     // ... 使用结果
 *   }
 * @endcode
 *
 * @par 配置平铺（Config Flattening）
 * preStart() 将 JSON 中的四域层次配置（hardware/mission/policy/environment）
 * 全部展开为类的扁平私有成员变量（hw_* / mission_* / policy_* / env_*），
 * 便于引擎参数管理系统直接按名访问每个叶节点参数。
 */

#ifndef EXAMPLES_RADAR_MODULE_H_
#define EXAMPLES_RADAR_MODULE_H_

#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/airborne_radar/session/ArExternalOutputAdapter.h"
#include "1q/airborne_radar/session/ArReplaySession.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArTraceSession.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "ArDebugViewToJson.h"
#include "config_loader.h"

namespace ar = airborne_radar;
namespace ar_config = airborne_radar::config;
namespace ar_session = airborne_radar::session;

/**
 * @brief 机载雷达集成模块。
 *
 * 封装 ArSession 并提供引擎友好的集成接口：
 *   - initialize / preStart / stepImp 三阶段生命周期
 *   - 四域配置平铺至扁平私有成员
 *   - 基于回调的订阅者模式支持运行期配置修改
 */
class RadarModule {
 public:
  // ==================== 构造 / 析构 ====================

  RadarModule() = default;
  ~RadarModule() = default;

  // 不可拷贝
  RadarModule(const RadarModule&) = delete;
  RadarModule& operator=(const RadarModule&) = delete;

  // 可移动
  RadarModule(RadarModule&&) = default;
  RadarModule& operator=(RadarModule&&) = default;

  // ==================== 生命周期接口 ====================

  /**
   * @brief 初始化内部状态。
   *
   * 创建 ArSession 实例并准备环境状态管理器。
   * 可在构造后立即调用；默认使用 ArSessionConfig{} 创建空配置 Session。
   *
   * @param[in] config  初始会话配置（可选，默认空配置）
   * @return true  初始化成功
   * @return false 初始化失败（不应发生，留作扩展）
   */
  bool initialize(const ar_config::ArSessionConfig& config = {}) {
    session_ = ar_session::ArSession::Create(config);
    initialized_ = true;
    return true;
  }

  /**
   * @brief 预启动：从 JSON 配置文件加载四域参数，平铺至私有成员。
   *
   * 调用 initialize() 之后、首次 stepImp() 之前调用。
   * 执行步骤：
   *   1. 解析 JSON 文件为 ArSessionConfig
   *   2. 将 ArSessionConfig 中所有叶节点参数平铺至本类私有成员
   *   3. 使用加载的配置重新创建 ArSession
   *
   * @param[in] config_path  JSON 配置文件路径
   * @return true  加载并创建成功
   * @return false 文件读取或解析失败
   */
  bool preStart(const std::string& config_path) {
    if (!initialized_) {
      std::cerr << "[RadarModule] ERROR: must call initialize() before preStart()\n";
      return false;
    }

    // 1. 从 JSON 文件加载配置
    ar_config::ArSessionConfig config;
    std::string error;
    if (!examples::LoadArSessionConfigFromFile(config_path.c_str(), &config, &error)) {
      std::cerr << "[RadarModule] ERROR: failed to load config from " << config_path << ": "
                << error << "\n";
      return false;
    }

    // 2. 平铺至私有成员
    flattenConfig(config);

    // 3. 用完整配置重建 Session
    session_ = ar_session::ArSession::Create(config);
    session_.AttachTrackLifecycleRecorder(&lifecycle_recorder_);
    started_ = true;
    return true;
  }

  /**
   * @brief 主步进函数：执行一个雷达仿真周期。
   *
   * 每个周期执行：
   *   1. 收集所有注册回调的运行期配置补丁
   *   2. 应用补丁到 Session（如有）
   *   3. 调用 Session::StepWithResult
   *   4. 缓存结果
   *
   * 外部引擎直接填充 ArCycleInput 的周期、平台、目标、自然环境与干扰字段。
   *
   * @param[in] input  本周期完整输入
   */
  void stepImp(const ar_session::ArCycleInput& input) {
    // 0. 检查是否已启动
    if (!started_) {
      std::cerr << "[RadarModule] WARN: stepImp called before preStart(), using default config\n";
      if (!initialized_) initialize();
      ar_config::ArSessionConfig default_config;
      flattenConfig(default_config);
      session_ = ar_session::ArSession::Create(default_config);
      session_.AttachTrackLifecycleRecorder(&lifecycle_recorder_);
      started_ = true;
    }

    // 1. 收集运行期配置补丁
    ar_config::ArRuntimeConfigPatch patch = collectConfigPatches();

    // 2. 应用补丁（如有）
    if (hasAnyPatch(patch)) {
      if (!session_.TryApplyRuntimeConfig(patch)) {
        std::cerr << "[RadarModule] WARN: runtime config patch rejected at cycle "
                  << last_result_.input_cycle_index + 1 << "\n";
      }
    }

    // 3. 执行一个周期
    ar_session::ArCycleInput mutable_input = input;
    if (mutable_input.cycle_index == 0) {
      mutable_input.cycle_index = cycle_index_;
    }
    last_input_ = mutable_input;
    last_result_ = session_.StepWithResult(mutable_input);
    ++cycle_index_;

    // 4. 读取生命周期事件（三视图之一；Session 已通过 Attach 自动驱动记录）
    lifecycle_events_ = lifecycle_recorder_.GetLastEvents();
  }

  // ==================== 订阅者模式 ====================

  /** @brief 运行期配置补丁回调类型。引擎通过回调修改补丁各字段以动态调整运行参数。 */
  using ConfigPatchCallback = std::function<void(ar_config::ArRuntimeConfigPatch& patch)>;

  /**
   * @brief 注册运行期配置变更回调。
   *
   * 注册的回调将在每次 stepImp 开始时调用。
   * 回调接收 ArRuntimeConfigPatch 引用，引擎可设置其 has_* 标志和对应字段值。
   *
   * @param[in] callback  回调函数对象
   */
  void registerConfigPatchCallback(ConfigPatchCallback callback) {
    config_patch_callbacks_.push_back(std::move(callback));
  }

  /** @brief 清除所有已注册的运行期回调。 */
  void clearConfigPatchCallbacks() { config_patch_callbacks_.clear(); }

  // ==================== 运行期查询 ====================

  /** @brief 返回最近一次 stepImp 的执行结果。 */
  const ar_session::ArCycleResult& lastResult() const { return last_result_; }

  /** @brief 返回内部 ArSession 引用（高级用途）。 */
  ar_session::ArSession& session() { return session_; }

  /** @brief 返回内部 ArSession 常量引用。 */
  const ar_session::ArSession& session() const { return session_; }

  /** @brief 当前已执行的周期数。 */
  std::uint32_t cycleCount() const { return cycle_index_; }

  /** @brief 模块是否已通过 preStart 启动。 */
  bool isStarted() const { return started_; }

  // ==================== 输出三视图 ====================
  //
  // 根据 docs/airborne_radar/data-flow.md（输出、调试与归属边界），AR 输出分为三个层次：
  //
  //   1. TrackOutputFrame（系统输出）      — 航迹输出帧，通过 lastResult() 获取
  //   2. ArTrackOutputDebugView（调试视图） — 人读排查视图，通过 buildLastDebugView() 获取
  //   3. ArTrackLifecycleRecorder（生命周期） — 确认/丢失/回收事件，通过 lifecycleEvents() 获取
  //
  // 此外将通过 buildExternalOutput() 提供 ECEF 外部坐标转换。
  // （跨模块差异：AR/EOS/ESR 均提供 buildExternalOutput()；SAR 因产品为聚焦图像、
  // 无需 ECEF 坐标转换，不暴露此能力，见 examples/sar/SarModule.h。）

  /** @brief 返回最近一次 stepImp 输入的缓存（供三视图构建使用）。 */
  const ar_session::ArCycleInput& lastInput() const { return last_input_; }

  /**
   * @brief 构建调试视图（三视图之一）。
   *
   * 将最后周期的输入与输出合成为人读可排查的轨迹状态视图。
   * 显示每个输入目标是否有对应 track、track status、位置、速度、RCS 等。
   */
  ar_session::ArTrackOutputDebugView buildLastDebugView() const {
    return ar_session::ArTrackOutputDebugViewBuilder::Build(
        last_input_.targets, last_result_);
  }

  /**
   * @brief 把最近一次调试视图序列化为 JSON 字符串（session_contract.md 规则 12 参考实现）。
   *
   * 集成方把返回的字符串写入自己的日志/事件系统即可；跨周期累积由调用方日志承担。
   * 序列化函数见 examples/airborne_radar/ArDebugViewToJson.h，可独立 copy。
   */
  std::string buildLastDebugViewJson() const {
    return ArDebugViewToJson(buildLastDebugView());
  }

  /** @brief 返回最近一次的生命周期事件列表（三视图之一）。 */
  const std::vector<ar_session::ArTrackLifecycleEvent>& lifecycleEvents() const {
    return lifecycle_events_;
  }

  /**
   * @brief 构建外部 ECEF 坐标输出（辅助视图）。
   *
   * 将雷达局部坐标系的 TrackOutputFrame 转换为外部引擎可用的 ECEF 世界坐标。
   *
   * @param[in]  platform  平台位姿（ECEF 位置/速度/姿态）
   * @param[out] output    外部 ECEF 轨迹输出
   * @return true  转换成功
   */
  bool buildExternalOutput(const ar_session::ArExternalPoseInput& platform,
                           ar_session::ArExternalTrackOutputFrame* output) const {
    return ar_session::ArCycleOutputAdapter::Build(platform, last_result_.track_output_frame,
                                                   output);
  }

  // ==================== 回放 (Replay) ====================
  //
  // AR 支持记录运行期 trace 并在事后回放，用于调试和回归验证。
  // RadarModule 提供 enableTrace() 开启录制，replayTrace() 供事后回放。

  /**
   * @brief 启用运行期 trace 录制（API 占位）。
   *
   * 实际工程中应在构造/initialize 时传入 trace 选项，
   * 使用 ArTraceSession 代替 ArSession 完成录制。
   * 回放通过静态方法 replayTrace() 完成。
   *
   * @param[in] trace_dir   trace 输出目录名称（仅作标识）
   * @return true  （占位返回值）
   */
  bool enableTrace(const std::string& trace_dir) {
    std::cerr << "[RadarModule] enableTrace(\"" << trace_dir << "\") — "
              << "实际工程中应使用 ArTraceSession 代替 ArSession；"
              << "此处仅做 API 演示。\n";
    trace_enabled_ = true;
    return true;
  }

  /**
   * @brief 回放已录制的 trace（静态方法）。
   *
   * @param[in] trace_dir  之前 enableTrace() 的输出目录
   * @return ArReplaySessionResult  回放结果详情
   */
  static ar_session::ArReplaySessionResult replayTrace(const std::string& trace_dir) {
    return ar_session::ReplayArTrace(trace_dir);
  }

  // ==================== 调试 / 诊断 ====================

  /** @brief 打印当前平铺配置摘要（用于验证 flatten 结果）。 */
  void printConfigSummary(std::ostream& os = std::cout) const {
    os << "=== RadarModule Config Summary ===\n"
       << "[Hardware]\n"
       << "  peak_power_w=" << hw_peak_power_w_ << " freq_hz=" << hw_frequency_hz_
       << " prf_hz=" << hw_prf_hz_ << " pulse_count=" << policy_pulse_count_
       << "\n  gain_db=" << hw_main_beam_gain_db_ << " az_bw=" << hw_nominal_az_beamwidth_deg_
       << " el_bw=" << hw_nominal_el_beamwidth_deg_ << " pfa=" << policy_pfa_
       << " minimum_snr_db=" << policy_minimum_snr_db_ << "\n[Mission]\n"
       << "  sensor_enabled=" << sensor_enabled_
       << " work_mode=" << static_cast<int>(mission_work_mode_) << " scan_center=["
       << mission_scan_center_az_deg_ << "," << mission_scan_center_el_deg_ << "]"
       << "\n[Policy]\n"
       << "  kalman_filter=" << policy_enable_kalman_filter_
       << " noise_std=" << policy_kalman_measurement_noise_std_
       << " confirm_hits=" << policy_confirm_hits_ << " max_miss=" << policy_max_miss_before_lost_
       << "\n[Environment]\n"
       << "  atmos_model=" << env_enable_atmospheric_model_ << " temp_k=" << env_temperature_k_
       << " vegetation=" << static_cast<int>(env_vegetation_cover_profile_) << "\n";
  }

 private:
  // ==================== 内部方法 ====================

  /**
   * @brief 将 ArSessionConfig 所有叶节点参数平铺至私有成员。
   *
   * 这是 preStart() 的核心步骤，将层次化配置结构展开为类的扁平成员变量，
   * 使外部引擎能通过 getter 直接访问每个配置项，无需理解库内部结构层次。
   */
  void flattenConfig(const ar_config::ArSessionConfig& config) {
    // ---- 硬件域 (Hardware / Detection) ----
    const auto& det = config.hardware;

    // Transmitter
    hw_peak_power_w_ = static_cast<double>(det.transmitter.peak_power_w);
    hw_frequency_hz_ = static_cast<double>(det.transmitter.frequency_hz);
    hw_bandwidth_hz_ = static_cast<double>(det.transmitter.bandwidth_hz);
    hw_pulse_width_s_ = static_cast<double>(det.transmitter.pulse_width_s);
    hw_prf_hz_ = static_cast<double>(det.transmitter.prf_hz);
    hw_transmit_loss_db_ = static_cast<double>(det.transmitter.transmit_loss_db);

    // Antenna
    hw_main_beam_gain_db_ = static_cast<double>(det.antenna.main_beam_gain_db);
    hw_nominal_az_beamwidth_deg_ = static_cast<double>(det.antenna.nominal_az_beamwidth_deg);
    hw_nominal_el_beamwidth_deg_ = static_cast<double>(det.antenna.nominal_el_beamwidth_deg);
    hw_antenna_length_m_ = static_cast<double>(det.antenna.antenna_length_m);
    hw_antenna_width_m_ = static_cast<double>(det.antenna.antenna_width_m);
    hw_enable_directional_pattern_ = det.antenna.enable_directional_pattern;

    // Antenna pattern
    hw_antenna_pattern_model_ = det.antenna.pattern.model_type;
    hw_max_sidelobe_level_db_ = static_cast<double>(det.antenna.pattern.max_sidelobe_level_db);
    hw_backlobe_level_db_ = static_cast<double>(det.antenna.pattern.backlobe_level_db);
    hw_scan_loss_coeff_db_per_deg2_ =
        static_cast<double>(det.antenna.pattern.scan_loss_coeff_db_per_deg2);
    hw_max_scan_loss_db_ = static_cast<double>(det.antenna.pattern.max_scan_loss_db);
    hw_boresight_offset_az_deg_ =
        static_cast<double>(det.antenna.pattern.boresight_offset_deg.az_deg);
    hw_boresight_offset_el_deg_ =
        static_cast<double>(det.antenna.pattern.boresight_offset_deg.el_deg);

    // Receiver
    hw_noise_figure_db_ = static_cast<double>(det.receiver.noise_figure_db);
    hw_receive_loss_db_ = static_cast<double>(det.receiver.receive_loss_db);

    // RCS physics
    hw_enable_physical_rcs_ = det.rcs_physics.enable_physical_rcs;
    hw_rcs_physics_mix_ratio_ = static_cast<double>(det.rcs_physics.physics_mix_ratio);
    hw_cylinder_weight_ = static_cast<double>(det.rcs_physics.cylinder_weight);
    hw_min_equivalent_radius_m_ = static_cast<double>(det.rcs_physics.min_equivalent_radius_m);
    hw_max_equivalent_radius_m_ = static_cast<double>(det.rcs_physics.max_equivalent_radius_m);
    hw_min_rcs_m2_ = static_cast<double>(det.rcs_physics.min_rcs_m2);
    hw_max_rcs_m2_ = static_cast<double>(det.rcs_physics.max_rcs_m2);
    hw_bistatic_psi_offset_deg_ = static_cast<double>(det.rcs_physics.bistatic_psi_offset_deg);

    // ---- 任务域 (Mission) ----
    const auto& mission = config.mission;
    // 电源状态提升到 SessionConfig 顶层（COMMON-OQ-4 收敛）
    sensor_enabled_ = config.sensor_enabled;

    const auto& orient = mission.orientation;
    mission_mount_yaw_deg_ = static_cast<double>(orient.mount_angles_deg.yaw_deg);
    mission_mount_pitch_deg_ = static_cast<double>(orient.mount_angles_deg.pitch_deg);
    mission_mount_roll_deg_ = static_cast<double>(orient.mount_angles_deg.roll_deg);

    mission_scan_center_az_deg_ = static_cast<double>(orient.scan_center_deg.az_deg);
    mission_scan_center_el_deg_ = static_cast<double>(orient.scan_center_deg.el_deg);

    mission_mech_scan_az_min_deg_ =
        static_cast<double>(orient.mechanical_scan_limits_deg.az_min_deg);
    mission_mech_scan_az_max_deg_ =
        static_cast<double>(orient.mechanical_scan_limits_deg.az_max_deg);
    mission_mech_scan_el_min_deg_ =
        static_cast<double>(orient.mechanical_scan_limits_deg.el_min_deg);
    mission_mech_scan_el_max_deg_ =
        static_cast<double>(orient.mechanical_scan_limits_deg.el_max_deg);

    mission_electronic_scan_az_min_deg_ =
        static_cast<double>(orient.electronic_scan_limits_deg.az_min_deg);
    mission_electronic_scan_az_max_deg_ =
        static_cast<double>(orient.electronic_scan_limits_deg.az_max_deg);
    mission_electronic_scan_el_min_deg_ =
        static_cast<double>(orient.electronic_scan_limits_deg.el_min_deg);
    mission_electronic_scan_el_max_deg_ =
        static_cast<double>(orient.electronic_scan_limits_deg.el_max_deg);

    mission_scan_start_position_ = orient.scan_start_position;
    mission_scan_sequence_ = orient.scan_sequence;
    mission_work_mode_ = orient.work_mode;

    mission_commanded_beamwidth_enabled_ = orient.commanded_beamwidth_enabled;
    mission_commanded_az_beamwidth_deg_ =
        static_cast<double>(orient.commanded_beamwidth_deg.commanded_az_beamwidth_deg);
    mission_commanded_el_beamwidth_deg_ =
        static_cast<double>(orient.commanded_beamwidth_deg.commanded_el_beamwidth_deg);

    mission_stabilization_mode_ = orient.stabilization_mode;

    // ---- 策略域 (Policy) ----
    const auto& policy = config.policy;

    policy_pfa_ = static_cast<double>(policy.detection.pfa);
    policy_minimum_snr_db_ = static_cast<double>(policy.detection.minimum_snr_db);
    policy_minimum_detection_margin_db_ =
        static_cast<double>(policy.detection.minimum_detection_margin_db);
    policy_pulse_count_ = policy.detection.pulse_count;

    policy_nominal_beamwidth_az_deg_ = static_cast<double>(
        policy.beam_control.pointing.nominal_beamwidth_deg.commanded_az_beamwidth_deg);
    policy_nominal_beamwidth_el_deg_ = static_cast<double>(
        policy.beam_control.pointing.nominal_beamwidth_deg.commanded_el_beamwidth_deg);

    policy_azimuth_step_count_hint_ = policy.beam_control.scheduler.azimuth_step_count_hint;
    policy_elevation_step_count_hint_ = policy.beam_control.scheduler.elevation_step_count_hint;
    policy_prefer_dense_tas_sampling_ = policy.beam_control.scheduler.prefer_dense_tas_sampling;

    policy_distance_gate_sigma_ = static_cast<double>(policy.association.distance_gate_sigma);

    policy_enable_kalman_filter_ = policy.tracking.enable_kalman_filter;
    policy_kalman_measurement_noise_std_ =
        static_cast<double>(policy.tracking.kalman_measurement_noise_std);
    policy_speed_decay_ratio_on_loss_ =
        static_cast<double>(policy.tracking.speed_decay_ratio_on_loss);
    policy_rcs_decay_ratio_on_loss_ = static_cast<double>(policy.tracking.rcs_decay_ratio_on_loss);

    policy_confirm_hits_ = policy.lifecycle.confirm_hits;
    policy_max_miss_before_lost_ = policy.lifecycle.max_miss_before_lost;
    policy_max_lost_cycles_ = policy.lifecycle.max_lost_cycles;
    policy_enable_imm_lifecycle_ = policy.lifecycle.enable_imm_lifecycle;
    policy_model_count_hint_ = policy.lifecycle.model_count_hint;

    // ---- 环境域 (Environment) ----
    const auto& env = config.environment;
    const auto& scenario = env.scenario_config;

    env_enable_atmospheric_model_ = scenario.atmospheric_physics.enable_physical_model;
    env_pressure_hpa_ = static_cast<double>(scenario.atmospheric_physics.pressure_hpa);
    env_temperature_k_ = static_cast<double>(scenario.atmospheric_physics.temperature_k);
    env_relative_humidity_ = static_cast<double>(scenario.atmospheric_physics.relative_humidity);

    env_vegetation_cover_profile_ = scenario.vegetation_scatter_physics.cover_profile;
    env_enable_vegetation_scatter_ = scenario.vegetation_scatter_physics.enable_physical_model;

  }

  /**
   * @brief 收集所有注册回调，合成为 ArRuntimeConfigPatch。
   *
   * 按注册顺序依次调用所有回调。每个回调独立修改补丁，
   * 后调用的回调可覆盖先前回调已设置的字段。
   */
  ar_config::ArRuntimeConfigPatch collectConfigPatches() {
    ar_config::ArRuntimeConfigPatch patch;
    for (const auto& cb : config_patch_callbacks_) {
      if (cb) cb(patch);
    }
    return patch;
  }

  /// 判断补丁中是否至少有一个字段被设置。
  static bool hasAnyPatch(const ar_config::ArRuntimeConfigPatch& patch) {
    return patch.has_mission || patch.has_policy || patch.has_environment || patch.has_work_mode ||
           patch.has_scan_center_deg || patch.has_dwell_center_deg ||
           patch.has_commanded_beamwidth_deg || patch.has_commanded_beamwidth_enabled ||
           patch.has_sensor_enabled;
  }

  // ==================== 内部状态 ====================

  ar_session::ArSession session_{};
  ar_session::ArCycleInput last_input_{};
  ar_session::ArCycleResult last_result_{};

  /// 生命周期记录器（三视图之一）
  ar_session::ArTrackLifecycleRecorder lifecycle_recorder_{};
  std::vector<ar_session::ArTrackLifecycleEvent> lifecycle_events_{};

  bool initialized_{false};
  bool started_{false};
  bool trace_enabled_{false};
  std::uint32_t cycle_index_{0};

  // 运行期回调列表
  std::vector<ConfigPatchCallback> config_patch_callbacks_;

  // ==================== 平铺的四域参数 ====================

  // -- 硬件域 (Hardware / Detection) --
  // Transmitter
  double hw_peak_power_w_{0.0};
  double hw_frequency_hz_{0.0};
  double hw_bandwidth_hz_{0.0};
  double hw_pulse_width_s_{0.0};
  double hw_prf_hz_{0.0};
  double hw_transmit_loss_db_{0.0};
  // Antenna
  double hw_main_beam_gain_db_{0.0};
  double hw_nominal_az_beamwidth_deg_{0.0};
  double hw_nominal_el_beamwidth_deg_{0.0};
  double hw_antenna_length_m_{0.0};
  double hw_antenna_width_m_{0.0};
  bool hw_enable_directional_pattern_{false};
  // Antenna pattern
  ar_config::AntennaPatternModelType hw_antenna_pattern_model_{
      ar_config::AntennaPatternModelType::kGaussianMainLobe};
  double hw_max_sidelobe_level_db_{0.0};
  double hw_backlobe_level_db_{0.0};
  double hw_scan_loss_coeff_db_per_deg2_{0.0};
  double hw_max_scan_loss_db_{0.0};
  double hw_boresight_offset_az_deg_{0.0};
  double hw_boresight_offset_el_deg_{0.0};
  // Receiver
  double hw_noise_figure_db_{0.0};
  double hw_receive_loss_db_{0.0};
  // RCS physics
  bool hw_enable_physical_rcs_{false};
  double hw_rcs_physics_mix_ratio_{0.0};
  double hw_cylinder_weight_{0.0};
  double hw_min_equivalent_radius_m_{0.0};
  double hw_max_equivalent_radius_m_{0.0};
  double hw_min_rcs_m2_{0.0};
  double hw_max_rcs_m2_{0.0};
  double hw_bistatic_psi_offset_deg_{0.0};
  // -- 任务域 (Mission) --
  bool sensor_enabled_{true};
  // Mount angles
  double mission_mount_yaw_deg_{0.0};
  double mission_mount_pitch_deg_{0.0};
  double mission_mount_roll_deg_{0.0};
  // Scan center
  double mission_scan_center_az_deg_{0.0};
  double mission_scan_center_el_deg_{0.0};
  // Mechanical scan limits
  double mission_mech_scan_az_min_deg_{0.0};
  double mission_mech_scan_az_max_deg_{0.0};
  double mission_mech_scan_el_min_deg_{0.0};
  double mission_mech_scan_el_max_deg_{0.0};
  // Electronic scan limits
  double mission_electronic_scan_az_min_deg_{0.0};
  double mission_electronic_scan_az_max_deg_{0.0};
  double mission_electronic_scan_el_min_deg_{0.0};
  double mission_electronic_scan_el_max_deg_{0.0};
  // Scan strategy
  oneq::foundation::ScanStartPosition mission_scan_start_position_{
      oneq::foundation::ScanStartPosition::kLeftTop};
  oneq::foundation::ScanSequence mission_scan_sequence_{
      oneq::foundation::ScanSequence::kAzimuthFirst};
  ar_config::ArWorkMode mission_work_mode_{ar_config::ArWorkMode::kTws};
  // Commanded beamwidth
  bool mission_commanded_beamwidth_enabled_{false};
  double mission_commanded_az_beamwidth_deg_{0.0};
  double mission_commanded_el_beamwidth_deg_{0.0};
  // Stabilization
  ar_config::StabilizationMode mission_stabilization_mode_{
      ar_config::StabilizationMode::kBodyStabilized};

  // -- 策略域 (Policy) --
  // Detection
  double policy_pfa_{0.0};
  double policy_minimum_snr_db_{0.0};
  double policy_minimum_detection_margin_db_{0.0};
  int policy_pulse_count_{0};
  // Beam control — pointing
  double policy_nominal_beamwidth_az_deg_{0.0};
  double policy_nominal_beamwidth_el_deg_{0.0};
  // Beam control — scheduler
  std::uint32_t policy_azimuth_step_count_hint_{0U};
  std::uint32_t policy_elevation_step_count_hint_{0U};
  bool policy_prefer_dense_tas_sampling_{false};
  // Association
  double policy_distance_gate_sigma_{0.0};
  // Tracking
  bool policy_enable_kalman_filter_{true};
  double policy_kalman_measurement_noise_std_{0.0};
  double policy_speed_decay_ratio_on_loss_{0.0};
  double policy_rcs_decay_ratio_on_loss_{0.0};
  // Lifecycle
  std::uint32_t policy_confirm_hits_{0U};
  std::uint32_t policy_max_miss_before_lost_{0U};
  std::uint32_t policy_max_lost_cycles_{0U};
  bool policy_enable_imm_lifecycle_{false};
  std::uint32_t policy_model_count_hint_{0U};

  // -- 环境域 (Environment) --
  // Atmospheric physics
  bool env_enable_atmospheric_model_{false};
  double env_pressure_hpa_{0.0};
  double env_temperature_k_{0.0};
  double env_relative_humidity_{0.0};
  // Vegetation scatter
  ar_config::VegetationCoverProfile env_vegetation_cover_profile_{
      ar_config::VegetationCoverProfile::kDisabled};
  bool env_enable_vegetation_scatter_{false};
};

#endif  // EXAMPLES_RADAR_MODULE_H_
