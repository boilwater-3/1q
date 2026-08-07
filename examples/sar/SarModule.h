/**
 * @file SarModule.h
 * @brief SAR 集成模块 — 适配外部仿真引擎的标准生命周期。
 *
 * @par 设计目标
 * 提供一种"复制即可用"的集成模式，使外部引擎（仿真框架、数字孪生平台等）
 * 能通过统一生命周期驱动 SAR 模块：
 *   - initialize() → 初始化内部状态
 *   - preStart()  → 从文件加载四域配置并平铺至私有成员
 *   - stepImp()   → 每周期执行 SAR 仿真与成像
 *   - 订阅者模式    → 引擎可在运行期回调注入配置变更
 *
 * @par 典型使用流程
 * @code
 *   SarModule sar;
 *   sar.initialize();
 *   sar.preStart("configs/sar.json");
 *
 *   // 注册运行期回调（可选）
 *   sar.registerConfigPatchCallback([](SarRuntimeConfigPatch& patch) {
 *     patch.has_enable_l1_rda_imaging = true;
 *     patch.enable_l1_rda_imaging = true;
 *   });
 *
 *   while (running) {
 *     sar.stepImp(input);
 *     const auto& result = sar.lastResult();
 *     if (result.status != sar::session::SarCycleStatus::kCompleted) { ... }
 *     if (result.focused_image.row_count > 0) { ... }
 *   }
 * @endcode
 *
 * @par 与 RadarModule/EosModule/EsrModule 的差异
 * - SAR 不需要环境输入（SarCycleInput 无 environment 字段），因此无 environment_state_
 * - SAR 产品是图像而非轨迹/航迹/辐射源，因此不暴露外部坐标输出适配
 * - SarProductLifecycleRecorder 通过 Session::AttachProductLifecycleRecorder 自动驱动，不接收输入
 *
 * @par 配置平铺（Config Flattening）
 * preStart() 将 JSON 中的四域层次配置（hardware/mission/processing/environment）
 * 全部展开为类的扁平私有成员变量（hw_* / mission_* / policy_* / env_*），
 * 便于引擎参数管理系统直接按名访问每个叶节点参数。
 */

#ifndef EXAMPLES_SAR_MODULE_H_
#define EXAMPLES_SAR_MODULE_H_

#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "1q/sar/sar.hpp"
#include "1q/sar/session/SarProductDebugView.h"
#include "1q/sar/session/SarProductLifecycleRecorder.h"
#include "1q/sar/session/SarReplaySession.h"
#include "1q/sar/session/SarTraceSession.h"
#include "config_loader.h"

namespace sar_config = sar::config;
namespace sar_session = sar::session;

/**
 * @brief SAR 集成模块。
 *
 * 封装 SarSession 并提供引擎友好的集成接口：
 *   - initialize / preStart / stepImp 三阶段生命周期
 *   - 四域配置平铺至扁平私有成员
 *   - 基于回调的订阅者模式支持运行期配置修改
 *
 * @note SAR 无需环境输入（environment_state_），输出面为系统结果、调试视图与产品生命周期。
 */
class SarModule {
 public:
  // ==================== 构造 / 析构 ====================

  SarModule() = default;
  ~SarModule() = default;

  // 不可拷贝
  SarModule(const SarModule&) = delete;
  SarModule& operator=(const SarModule&) = delete;

  // 可移动
  SarModule(SarModule&&) = default;
  SarModule& operator=(SarModule&&) = default;

  // ==================== 生命周期接口 ====================

  /**
   * @brief 初始化内部状态。
   *
   * 创建 SarSession 实例。
   * 可在构造后立即调用；默认使用 SarSessionConfig{} 创建空配置 Session。
   *
   * @param[in] config  初始会话配置（可选，默认空配置）
   * @return true  初始化成功
   * @return false 初始化失败（不应发生，留作扩展）
   */
  bool initialize(const sar_config::SarSessionConfig& config = {}) {
    session_ = sar_session::SarSession::Create(config);
    session_.AttachProductLifecycleRecorder(&lifecycle_recorder_);
    initialized_ = true;
    return true;
  }

  /**
   * @brief 预启动：从 JSON 配置文件加载四域参数，平铺至私有成员。
   *
   * 调用 initialize() 之后、首次 stepImp() 之前调用。
   * 执行步骤：
   *   1. 解析 JSON 文件为 SarSessionConfig
   *   2. 将 SarSessionConfig 中所有叶节点参数平铺至本类私有成员
   *   3. 使用加载的配置重新创建 SarSession
   *
   * @param[in] config_path  JSON 配置文件路径
   * @return true  加载并创建成功
   * @return false 文件读取或解析失败
   */
  bool preStart(const std::string& config_path) {
    if (!initialized_) {
      std::cerr << "[SarModule] ERROR: must call initialize() before preStart()\n";
      return false;
    }

    // 1. 从 JSON 文件加载配置
    sar_config::SarSessionConfig config;
    std::string error;
    if (!examples::LoadSarSessionConfigFromFile(config_path.c_str(), &config, &error)) {
      std::cerr << "[SarModule] ERROR: failed to load config from " << config_path
                << ": " << error << "\n";
      return false;
    }

    // 2. 平铺至私有成员
    flattenConfig(config);

    // 3. 用完整配置重建 Session
    session_ = sar_session::SarSession::Create(config);
    session_.AttachProductLifecycleRecorder(&lifecycle_recorder_);
    started_ = true;
    return true;
  }

  /**
   * @brief 主步进函数：执行一个 SAR 仿真/成像周期。
   *
   * 每个周期执行：
   *   1. 收集所有注册回调的运行期配置补丁
   *   2. 应用补丁到 Session（如有）
   *   3. 调用 Session::StepWithResult
   *   4. 缓存结果并记录产品生命周期事件
   *
   * 外部引擎应使用 SarCycleInputAdapter::Build 构造 SarCycleInput，
   * 或直接填充各字段后传入。
   *
   * @param[in] input  本周期完整输入
   */
  void stepImp(const sar_session::SarCycleInput& input) {
    // 0. 检查是否已启动
    if (!started_) {
      std::cerr << "[SarModule] WARN: stepImp called before preStart(), using default config\n";
      if (!initialized_) initialize();
      sar_config::SarSessionConfig default_config;
      flattenConfig(default_config);
      session_ = sar_session::SarSession::Create(default_config);
      session_.AttachProductLifecycleRecorder(&lifecycle_recorder_);
      started_ = true;
    }

    // 1. 收集运行期配置补丁
    sar_config::SarRuntimeConfigPatch patch = collectConfigPatches();

    // 2. 应用补丁（如有）
    if (hasAnyPatch(patch)) {
      if (!session_.TryApplyRuntimeConfig(patch)) {
        std::cerr << "[SarModule] WARN: runtime config patch rejected at cycle "
                  << last_result_.input_cycle_index + 1 << "\n";
      }
    }

    // 3. 执行一个周期
    sar_session::SarCycleInput mutable_input = input;
    if (mutable_input.cycle_index == 0) {
      mutable_input.cycle_index = cycle_index_;
    }
    last_input_ = mutable_input;
    last_result_ = session_.StepWithResult(mutable_input);
    ++cycle_index_;

    // 4. 读取产品生命周期事件（三视图之一，由 Session 自动驱动记录器）
    lifecycle_events_ = lifecycle_recorder_.GetLastEvents();
  }

  // ==================== 订阅者模式 ====================

  /** @brief 运行期配置补丁回调类型。引擎通过回调修改补丁各字段以动态调整运行参数。 */
  using ConfigPatchCallback =
      std::function<void(sar_config::SarRuntimeConfigPatch& patch)>;

  /**
   * @brief 注册运行期配置变更回调。
   *
   * 注册的回调将在每次 stepImp 开始时调用。
   * 回调接收 SarRuntimeConfigPatch 引用，引擎可设置其 has_* 标志和对应字段值。
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
  const sar_session::SarCycleResult& lastResult() const { return last_result_; }

  /** @brief 返回内部 SarSession 引用（高级用途）。 */
  sar_session::SarSession& session() { return session_; }

  /** @brief 返回内部 SarSession 常量引用。 */
  const sar_session::SarSession& session() const { return session_; }

  /** @brief 当前已执行的周期数。 */
  std::uint32_t cycleCount() const { return cycle_index_; }

  /** @brief 模块是否已通过 preStart 启动。 */
  bool isStarted() const { return started_; }

  // ==================== 输出三视图 ====================
  //
  // 根据 SAR 模块设计约定，SAR 输出分为三个层次：
  //
  //   1. SarCycleResult（系统输出）    — 完整结果帧，通过 lastResult() 获取
  //   2. SarProductDebugView（调试视图）— 人读排查视图，通过 buildLastDebugView() 获取
  //   3. SarProductLifecycleRecorder（生命周期）— 图像生成/更新/丢失/失败事件，
  //      通过 lifecycleEvents() 获取
  //
  /** @brief 返回最近一次 stepImp 输入的缓存（供三视图构建使用）。 */
  const sar_session::SarCycleInput& lastInput() const { return last_input_; }

  /**
   * @brief 构建 SAR 产品调试视图（三视图之一）。
   *
   * 将最后周期的输入与输出合成为人读可排查的产品状态视图。
   * 显示成像阶段、诊断信息、点目标列表等。
   */
  sar_session::SarProductDebugView buildLastDebugView() const {
    return sar_session::SarProductDebugViewBuilder::Build(last_input_, last_result_);
  }

  /** @brief 返回最近一次的产品生命周期事件列表（三视图之一）。 */
  const std::vector<sar_session::SarProductLifecycleEvent>& lifecycleEvents() const {
    return lifecycle_events_;
  }

  // ==================== 回放 (Replay) ====================
  //
  // SAR 支持记录运行期 trace 并在事后回放，用于调试和回归验证。
  // SarModule 提供 enableTrace() 开启录制，replayTrace() 供事后回放。

  /**
   * @brief 启用运行期 trace 录制（API 占位）。
   *
   * 实际工程中应在构造/initialize 时传入 trace 选项，
   * 使用 SarTraceSession 代替 SarSession 完成录制。
   * 回放通过静态方法 replayTrace() 完成。
   *
   * @param[in] trace_dir   trace 输出目录名称（仅作标识）
   * @return true  （占位返回值）
   */
  bool enableTrace(const std::string& trace_dir) {
    std::cerr << "[SarModule] enableTrace(\"" << trace_dir << "\") — "
              << "实际工程中应使用 SarTraceSession 代替 SarSession；"
              << "此处仅做 API 演示。\n";
    trace_enabled_ = true;
    return true;
  }

  /**
   * @brief 回放已录制的 trace（静态方法）。
   *
   * @param[in] trace_dir  之前 enableTrace() 的输出目录
   * @return SarReplaySessionResult  回放结果详情
   */
  static sar_session::SarReplaySessionResult replayTrace(const std::string& trace_dir) {
    return sar_session::ReplaySarTrace(trace_dir);
  }

  // ==================== 调试 / 诊断 ====================

  /** @brief 打印当前平铺配置摘要（用于验证 flatten 结果）。 */
  void printConfigSummary(std::ostream& os = std::cout) const {
    os << "=== SarModule Config Summary ===\n"
       << "[Hardware]\n"
       << "  carrier_frequency_hz=" << hw_carrier_frequency_hz_
       << " bandwidth_hz=" << hw_bandwidth_hz_
       << " pulse_width_s=" << hw_pulse_width_s_
       << " prf_hz=" << hw_prf_hz_
       << "\n  sample_rate_hz=" << hw_sample_rate_hz_
       << " peak_power_w=" << hw_peak_power_w_
       << " antenna_gain_db=" << hw_antenna_gain_db_
       << " noise_figure_db=" << hw_receiver_noise_figure_db_
       << "\n  antenna_len_m=" << hw_antenna_length_m_
       << " antenna_wid_m=" << hw_antenna_width_m_
       << " system_loss_db=" << hw_system_loss_db_
       << "\n[Mission]\n"
       << "  scene_center=[" << mission_scene_center_latitude_deg_
       << "," << mission_scene_center_longitude_deg_
       << "] alt=" << mission_scene_center_altitude_m_
       << "\n  slant_range_m=" << mission_nominal_slant_range_m_
       << " speed_mps=" << mission_platform_speed_mps_
       << "\n  range_samples=" << mission_range_sample_count_
       << " azimuth_pulses=" << mission_azimuth_pulse_count_
       << " resolution=[" << mission_desired_ground_range_resolution_m_
       << "," << mission_desired_azimuth_resolution_m_ << "]"
       << "\n[Policy]\n"
       << "  raw_echo=" << policy_enable_raw_echo_generation_
       << " l1_rda=" << policy_enable_l1_rda_imaging_
       << " l2_moco=" << policy_enable_l2_motion_compensation_
       << " l3_bp=" << policy_enable_l3_bp_imaging_
       << "\n  diagnostics=" << policy_enable_diagnostics_
       << " retain_phase=" << policy_retain_raw_phase_history_
       << " retain_image=" << policy_retain_focused_image_
       << " max_squint=" << policy_max_allowed_squint_angle_deg_
       << " min_snr_db=" << policy_minimum_snr_db_
       << "\n[Environment]\n"
       << "  terrain_ref_alt_m=" << env_terrain_reference_altitude_m_
       << " atmos_loss_db_per_km=" << env_atmospheric_loss_db_per_km_
       << " sigma0_db=" << env_surface_backscatter_sigma0_db_
       << "\n  flat_earth=" << env_use_flat_earth_geometry_
       << " atmos_atten=" << env_enable_atmospheric_attenuation_
       << "\n";
  }

 private:
  // ==================== 内部方法 ====================

  /**
   * @brief 将 SarSessionConfig 所有叶节点参数平铺至私有成员。
   *
   * 这是 preStart() 的核心步骤，将层次化配置结构展开为类的扁平成员变量，
   * 使外部引擎能通过 getter 直接访问每个配置项，无需理解库内部结构层次。
   */
  void flattenConfig(const sar_config::SarSessionConfig& config) {
    // ---- 硬件域 (Hardware) ----
    const auto& hw = config.hardware;
    hw_carrier_frequency_hz_ = static_cast<double>(hw.carrier_frequency_hz);
    hw_bandwidth_hz_ = static_cast<double>(hw.bandwidth_hz);
    hw_pulse_width_s_ = static_cast<double>(hw.pulse_width_s);
    hw_prf_hz_ = static_cast<double>(hw.pulse_repetition_frequency_hz);
    hw_sample_rate_hz_ = static_cast<double>(hw.sample_rate_hz);
    hw_peak_power_w_ = static_cast<double>(hw.peak_power_w);
    hw_antenna_length_m_ = static_cast<double>(hw.antenna_length_m);
    hw_antenna_width_m_ = static_cast<double>(hw.antenna_width_m);
    hw_antenna_gain_db_ = static_cast<double>(hw.antenna_gain_db);
    hw_receiver_noise_figure_db_ = static_cast<double>(hw.receiver_noise_figure_db);
    hw_system_loss_db_ = static_cast<double>(hw.system_loss_db);

    // ---- 任务域 (Mission) ----
    const auto& mission = config.mission;
    mission_scene_center_latitude_deg_ = static_cast<double>(mission.scene_center_latitude_deg);
    mission_scene_center_longitude_deg_ = static_cast<double>(mission.scene_center_longitude_deg);
    mission_scene_center_altitude_m_ = static_cast<double>(mission.scene_center_altitude_m);
    mission_nominal_slant_range_m_ = static_cast<double>(mission.nominal_slant_range_m);
    mission_platform_speed_mps_ = static_cast<double>(mission.platform_speed_mps);
    mission_range_sample_count_ = static_cast<std::uint32_t>(mission.range_sample_count);
    mission_azimuth_pulse_count_ = static_cast<std::uint32_t>(mission.azimuth_pulse_count);
    mission_desired_ground_range_resolution_m_ =
        static_cast<double>(mission.desired_ground_range_resolution_m);
    mission_desired_azimuth_resolution_m_ =
        static_cast<double>(mission.desired_azimuth_resolution_m);
    mission_l2_velocity_error_stddev_x_mps_ =
        static_cast<double>(mission.l2_velocity_error_stddev_x_mps);
    mission_l2_velocity_error_stddev_y_mps_ =
        static_cast<double>(mission.l2_velocity_error_stddev_y_mps);
    mission_l2_velocity_error_stddev_z_mps_ =
        static_cast<double>(mission.l2_velocity_error_stddev_z_mps);
    mission_l2_random_seed_ = static_cast<std::uint32_t>(mission.l2_random_seed);

    // ---- 策略域 (Policy) ----
    const auto& policy = config.policy;
    policy_enable_raw_echo_generation_ = policy.enable_raw_echo_generation;
    policy_enable_l1_rda_imaging_ = policy.enable_l1_rda_imaging;
    policy_enable_l2_motion_compensation_ = policy.enable_l2_motion_compensation;
    policy_enable_l3_bp_imaging_ = policy.enable_l3_bp_imaging;
    policy_enable_diagnostics_ = policy.enable_diagnostics;
    policy_retain_raw_phase_history_ = policy.retain_raw_phase_history;
    policy_retain_focused_image_ = policy.retain_focused_image;
    policy_max_allowed_squint_angle_deg_ =
        static_cast<double>(policy.max_allowed_squint_angle_deg);
    policy_minimum_snr_db_ = static_cast<double>(policy.minimum_snr_db);

    // ---- 环境域 (Environment) ----
    const auto& env = config.environment;
    env_terrain_reference_altitude_m_ = static_cast<double>(env.terrain_reference_altitude_m);
    env_atmospheric_loss_db_per_km_ = static_cast<double>(env.atmospheric_loss_db_per_km);
    env_surface_backscatter_sigma0_db_ = static_cast<double>(env.surface_backscatter_sigma0_db);
    env_use_flat_earth_geometry_ = env.use_flat_earth_geometry;
    env_enable_atmospheric_attenuation_ = env.enable_atmospheric_attenuation;
  }

  /**
   * @brief 收集所有注册回调，合成为 SarRuntimeConfigPatch。
   *
   * 按注册顺序依次调用所有回调。每个回调独立修改补丁，
   * 后调用的回调可覆盖先前回调已设置的字段。
   */
  sar_config::SarRuntimeConfigPatch collectConfigPatches() {
    sar_config::SarRuntimeConfigPatch patch;
    for (const auto& cb : config_patch_callbacks_) {
      if (cb) cb(patch);
    }
    return patch;
  }

  /// 判断补丁中是否至少有一个字段被设置。
  static bool hasAnyPatch(const sar_config::SarRuntimeConfigPatch& patch) {
    return patch.has_enable_raw_echo_generation ||
           patch.has_enable_l1_rda_imaging ||
           patch.has_retain_raw_phase_history ||
           patch.has_retain_focused_image ||
           patch.has_minimum_snr_db;
  }

  // ==================== 内部状态 ====================

  sar_session::SarSession session_{};
  sar_session::SarCycleInput last_input_{};
  sar_session::SarCycleResult last_result_{};

  /// 产品生命周期记录器（三视图之一）
  sar_session::SarProductLifecycleRecorder lifecycle_recorder_{};
  std::vector<sar_session::SarProductLifecycleEvent> lifecycle_events_{};

  bool initialized_{false};
  bool started_{false};
  bool trace_enabled_{false};
  std::uint32_t cycle_index_{0};

  // 运行期回调列表
  std::vector<ConfigPatchCallback> config_patch_callbacks_;

  // ==================== 平铺的四域参数 ====================

  // -- 硬件域 (Hardware) --
  double hw_carrier_frequency_hz_{0.0};
  double hw_bandwidth_hz_{0.0};
  double hw_pulse_width_s_{0.0};
  double hw_prf_hz_{0.0};
  double hw_sample_rate_hz_{0.0};
  double hw_peak_power_w_{0.0};
  double hw_antenna_length_m_{0.0};
  double hw_antenna_width_m_{0.0};
  double hw_antenna_gain_db_{0.0};
  double hw_receiver_noise_figure_db_{0.0};
  double hw_system_loss_db_{0.0};

  // -- 任务域 (Mission) --
  double mission_scene_center_latitude_deg_{0.0};
  double mission_scene_center_longitude_deg_{0.0};
  double mission_scene_center_altitude_m_{0.0};
  double mission_nominal_slant_range_m_{0.0};
  double mission_platform_speed_mps_{0.0};
  std::uint32_t mission_range_sample_count_{0U};
  std::uint32_t mission_azimuth_pulse_count_{0U};
  double mission_desired_ground_range_resolution_m_{0.0};
  double mission_desired_azimuth_resolution_m_{0.0};
  double mission_l2_velocity_error_stddev_x_mps_{0.0};
  double mission_l2_velocity_error_stddev_y_mps_{0.0};
  double mission_l2_velocity_error_stddev_z_mps_{0.0};
  std::uint32_t mission_l2_random_seed_{0U};

  // -- 策略域 (Policy) --
  bool policy_enable_raw_echo_generation_{true};
  bool policy_enable_l1_rda_imaging_{false};
  bool policy_enable_l2_motion_compensation_{false};
  bool policy_enable_l3_bp_imaging_{false};
  bool policy_enable_diagnostics_{true};
  bool policy_retain_raw_phase_history_{false};
  bool policy_retain_focused_image_{true};
  double policy_max_allowed_squint_angle_deg_{0.0};
  double policy_minimum_snr_db_{0.0};

  // -- 环境域 (Environment) --
  double env_terrain_reference_altitude_m_{0.0};
  double env_atmospheric_loss_db_per_km_{0.0};
  double env_surface_backscatter_sigma0_db_{0.0};
  bool env_use_flat_earth_geometry_{true};
  bool env_enable_atmospheric_attenuation_{false};
};

#endif  // EXAMPLES_SAR_MODULE_H_
