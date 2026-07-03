/**
 * @file EsrModule.h
 * @brief 电子侦察集成模块 — 适配外部仿真引擎的标准生命周期。
 *
 * @par 设计目标
 * 提供一种"复制即可用"的集成模式，使外部引擎（仿真框架、数字孪生平台等）
 * 能通过统一生命周期驱动电子侦察模块：
 *   - initialize() → 初始化内部状态
 *   - preStart()  → 从文件加载四域配置并平铺至私有成员
 *   - stepImp()   → 每周期执行电子侦察仿真
 *   - 订阅者模式    → 引擎可在运行期回调注入配置变更
 *
 * @par 典型使用流程
 * @code
 *   EsrModule esr;
 *   esr.initialize();
 *   esr.preStart("configs/electronic_warfare.json");
 *
 *   // 注册运行期回调（可选）
 *   esr.registerConfigPatchCallback([](EsrRuntimeConfigPatch& patch) {
 *     patch.has_work_mode = true;
 *     patch.work_mode = EsrWorkMode::kEsm;
 *   });
 *
 *   while (running) {
 *     esr.stepImp(input);
 *     const auto& result = esr.lastResult();
 *     // ... 使用结果
 *   }
 * @endcode
 *
 * @par 配置平铺（Config Flattening）
 * preStart() 将 JSON 中的四域层次配置（hardware/mission/policy/environment）
 * 全部展开为类的扁平私有成员变量（hw_* / mission_* / policy_* / env_*），
 * 便于引擎参数管理系统直接按名访问每个叶节点参数。
 */

#ifndef EXAMPLES_ESR_MODULE_H_
#define EXAMPLES_ESR_MODULE_H_

#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleOutputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrEmitterLifecycleRecorder.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputDebugView.h"
#include "1q/electronic_surveillance_radar/session/EsrReplaySession.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "config_loader.h"

namespace esr_config = electronic_surveillance_radar::config;
namespace esr_session = electronic_surveillance_radar::session;

/**
 * @brief 电子侦察集成模块。
 *
 * 封装 EsrSession 并提供引擎友好的集成接口：
 *   - initialize / preStart / stepImp 三阶段生命周期
 *   - 四域配置平铺至扁平私有成员
 *   - 基于回调的订阅者模式支持运行期配置修改
 */
class EsrModule {
 public:
  // ==================== 构造 / 析构 ====================

  EsrModule() = default;
  ~EsrModule() = default;

  // 不可拷贝
  EsrModule(const EsrModule&) = delete;
  EsrModule& operator=(const EsrModule&) = delete;

  // 可移动
  EsrModule(EsrModule&&) = default;
  EsrModule& operator=(EsrModule&&) = default;

  // ==================== 生命周期接口 ====================

  /**
   * @brief 初始化内部状态。
   *
   * 创建 EsrSession 实例并准备环境状态管理器。
   * 可在构造后立即调用；默认使用 EsrSessionConfig{} 创建空配置 Session。
   *
   * @param[in] config  初始会话配置（可选，默认空配置）
   * @return true  初始化成功
   * @return false 初始化失败（不应发生，留作扩展）
   */
  bool initialize(const esr_config::EsrSessionConfig& config = {}) {
    session_ = esr_session::EsrSession::Create(config);
    environment_state_ = esr_session::EsrEnvironmentInputState(esr_session::EsrEnvironmentInput{});
    initialized_ = true;
    return true;
  }

  /**
   * @brief 预启动：从 JSON 配置文件加载四域参数，平铺至私有成员。
   *
   * 调用 initialize() 之后、首次 stepImp() 之前调用。
   * 执行步骤：
   *   1. 解析 JSON 文件为 EsrSessionConfig
   *   2. 将 EsrSessionConfig 中所有叶节点参数平铺至本类私有成员
   *   3. 使用加载的配置重新创建 EsrSession
   *
   * @param[in] config_path  JSON 配置文件路径
   * @return true  加载并创建成功
   * @return false 文件读取或解析失败
   */
  bool preStart(const std::string& config_path) {
    if (!initialized_) {
      std::cerr << "[EsrModule] ERROR: must call initialize() before preStart()\n";
      return false;
    }

    // 1. 从 JSON 文件加载配置
    esr_config::EsrSessionConfig config;
    std::string error;
    if (!examples::LoadEsrSessionConfigFromFile(config_path.c_str(), &config, &error)) {
      std::cerr << "[EsrModule] ERROR: failed to load config from " << config_path
                << ": " << error << "\n";
      return false;
    }

    // 2. 平铺至私有成员
    flattenConfig(config);

    // 3. 用完整配置重建 Session
    session_ = esr_session::EsrSession::Create(config);
    environment_state_ = esr_session::EsrEnvironmentInputState(MakeDefaultEnvironmentInput());
    started_ = true;
    return true;
  }

  /**
   * @brief 主步进函数：执行一个电子侦察仿真周期。
   *
   * 每个周期执行：
   *   1. 收集所有注册回调的运行期配置补丁
   *   2. 应用补丁到 Session（如有）
   *   3. 调用 Session::StepWithResult
   *   4. 缓存结果并记录生命周期事件
   *
   * 外部引擎应使用 EsrCycleInputAdapter::Build 构造 EsrCycleInput，
   * 或直接填充各字段后传入。
   *
   * @param[in] input  本周期完整输入
   */
  void stepImp(const esr_session::EsrCycleInput& input) {
    // 0. 检查是否已启动
    if (!started_) {
      std::cerr << "[EsrModule] WARN: stepImp called before preStart(), using default config\n";
      if (!initialized_) initialize();
      esr_config::EsrSessionConfig default_config;
      flattenConfig(default_config);
      session_ = esr_session::EsrSession::Create(default_config);
      environment_state_ = esr_session::EsrEnvironmentInputState(MakeDefaultEnvironmentInput());
      started_ = true;
    }

    // 1. 收集运行期配置补丁
    esr_config::EsrRuntimeConfigPatch patch = collectConfigPatches();

    // 2. 应用补丁（如有）
    if (hasAnyPatch(patch)) {
      if (!session_.TryApplyRuntimeConfig(patch)) {
        std::cerr << "[EsrModule] WARN: runtime config patch rejected at cycle "
                  << last_result_.input_cycle_index + 1 << "\n";
      }
    }

    // 3. 执行一个周期
    esr_session::EsrCycleInput mutable_input = input;
    if (mutable_input.cycle_index == 0) {
      mutable_input.cycle_index = cycle_index_;
    }
    last_input_ = mutable_input;
    last_result_ = session_.StepWithResult(mutable_input);
    ++cycle_index_;

    // 4. 记录生命周期事件（三视图之一）
    lifecycle_events_ = lifecycle_recorder_.Update(last_input_, last_result_);
  }

  // ==================== 订阅者模式 ====================

  /** @brief 运行期配置补丁回调类型。引擎通过回调修改补丁各字段以动态调整运行参数。 */
  using ConfigPatchCallback =
      std::function<void(esr_config::EsrRuntimeConfigPatch& patch)>;

  /**
   * @brief 注册运行期配置变更回调。
   *
   * 注册的回调将在每次 stepImp 开始时调用。
   * 回调接收 EsrRuntimeConfigPatch 引用，引擎可设置其 has_* 标志和对应字段值。
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
  const esr_session::EsrCycleResult& lastResult() const { return last_result_; }

  /** @brief 返回内部 EsrSession 引用（高级用途）。 */
  esr_session::EsrSession& session() { return session_; }

  /** @brief 返回内部 EsrSession 常量引用。 */
  const esr_session::EsrSession& session() const { return session_; }

  /** @brief 当前已执行的周期数。 */
  std::uint32_t cycleCount() const { return cycle_index_; }

  /** @brief 模块是否已通过 preStart 启动。 */
  bool isStarted() const { return started_; }

  // ==================== 平台 / 场景输入设置 ====================

  /**
   * @brief 设置当前环境输入（环境状态管理器维护跨周期的环境状态）。
   *
   * 引擎可在每周期前调用此方法更新环境事实（传播环境、干扰源等），
   * 这些更新将在下次 stepImp 构造的 EsrCycleInput 中反映。
   */
  void updateEnvironment(const esr_session::EsrEnvironmentInputPatch& patch) {
    environment_state_.Update(patch);
  }

  /** @brief 返回当前环境快照。 */
  esr_session::EsrEnvironmentInput currentEnvironment() const {
    return environment_state_.Snapshot();
  }

  // ==================== 输出三视图 ====================
  //
  // 根据电子侦察模块设计约定，ESR 输出分为三个层次：
  //
  //   1. EsrOutputFrame（系统输出）      — 侦察输出帧，通过 lastResult() 获取
  //   2. EsrOutputDebugView（调试视图）   — 人读排查视图，通过 buildLastDebugView() 获取
  //   3. EsrEmitterLifecycleRecorder（生命周期） — 首次观测/更新/丢失事件，通过 lifecycleEvents() 获取
  //
  // 此外将通过 buildExternalOutput() 提供 ECEF 外部坐标转换。
  // （跨模块差异：AR/EOS/ESR 均提供 buildExternalOutput()；SAR 因产品为聚焦图像、
  // 无需 ECEF 坐标转换，不暴露此能力，见 examples/sar/SarModule.h。）

  /** @brief 返回最近一次 stepImp 输入的缓存（供三视图构建使用）。 */
  const esr_session::EsrCycleInput& lastInput() const { return last_input_; }

  /**
   * @brief 构建调试视图（三视图之一）。
   *
   * 将最后周期的输入与输出合成为人读可排查的辐射源状态视图。
   * 显示每个输入辐射源是否有对应观测、置信度、状态等。
   */
  esr_session::EsrOutputDebugView buildLastDebugView() const {
    return esr_session::EsrOutputDebugViewBuilder::Build(last_input_, last_result_);
  }

  /** @brief 返回最近一次的生命周期事件列表（三视图之一）。 */
  const std::vector<esr_session::EsrEmitterLifecycleEvent>& lifecycleEvents() const {
    return lifecycle_events_;
  }

  /**
   * @brief 构建外部 ECEF 坐标输出（辅助视图）。
   *
   * 将侦察局部坐标系的 EsrOutputFrame 转换为外部引擎可用的 ECEF 世界坐标输出。
   *
   * @param[in]  platform  外部平台位姿输入（ECEF 位置/速度/姿态）
   * @param[out] output    外部 ECEF 侦察输出帧
   * @return true  转换成功
   */
  bool buildExternalOutput(
      const esr_session::EsrExternalPoseInput& platform,
      esr_session::EsrExternalOutputFrame* output) const {
    return esr_session::EsrCycleOutputAdapter::Build(
        platform, last_result_.output_frame, output);
  }

  /**
   * @brief 构建外部 ECEF 坐标输出（辅助视图，显式参考系版本）。
   *
   * 将侦察局部坐标系的 EsrOutputFrame 转换为外部引擎可用的 ECEF 世界坐标输出。
   * 本重载允许调用方分离指定局部参考系与平台位姿。
   *
   * @param[in]  reference       局部坐标参考系（决定 ECEF/ENU 到 ESR 局部坐标的转换基准）
   * @param[in]  platform_pose   平台局部位姿状态
   * @param[out] output          外部 ECEF 侦察输出帧
   * @return true  转换成功
   */
  bool buildExternalOutput(
      const oneq::coordinate::LocalFrameReference& reference,
      const oneq::foundation::PoseState& platform_pose,
      esr_session::EsrExternalOutputFrame* output) const {
    return esr_session::EsrCycleOutputAdapter::Build(
        reference, platform_pose, last_result_.output_frame, output);
  }

  // ==================== 回放 (Replay) ====================
  //
  // ESR 支持记录运行期 trace 并在事后回放，用于调试和回归验证。
  // EsrModule 提供 enableTrace() 开启录制，replayTrace() 供事后回放。

  /**
   * @brief 启用运行期 trace 录制（API 占位）。
   *
   * 实际工程中应在构造/initialize 时传入 trace 选项，
   * 使用 EsrTraceSession 代替 EsrSession 完成录制。
   * 回放通过静态方法 replayTrace() 完成。
   *
   * @param[in] trace_dir   trace 输出目录名称（仅作标识）
   * @return true  （占位返回值）
   */
  bool enableTrace(const std::string& trace_dir) {
    std::cerr << "[EsrModule] enableTrace(\"" << trace_dir << "\") — "
              << "实际工程中应使用 EsrTraceSession 代替 EsrSession；"
              << "此处仅做 API 演示。\n";
    trace_enabled_ = true;
    return true;
  }

  /**
   * @brief 回放已录制的 trace（静态方法）。
   *
   * @param[in] trace_dir  之前 enableTrace() 的输出目录
   * @return EsrReplaySessionResult  回放结果详情
   */
  static esr_session::EsrReplaySessionResult replayTrace(const std::string& trace_dir) {
    return esr_session::ReplayEsrTrace(trace_dir);
  }

  // ==================== 调试 / 诊断 ====================

  /** @brief 打印当前平铺配置摘要（用于验证 flatten 结果）。 */
  void printConfigSummary(std::ostream& os = std::cout) const {
    os << "=== EsrModule Config Summary ===\n"
       << "[Hardware]\n"
       << "  receiver_band_hz=[" << hw_receiver_band_lower_hz_
       << "," << hw_receiver_band_upper_hz_ << "]"
       << " sensitivity_w=" << hw_receiver_sensitivity_w_
       << " loss_db=" << hw_integrated_receive_loss_db_
       << "\n  beam_az=" << hw_beam_az_width_deg_
       << " beam_el=" << hw_beam_el_width_deg_
       << " scan_az=" << hw_az_scan_range_deg_
       << " scan_el=" << hw_el_scan_range_deg_
       << " mount=[" << hw_antenna_mount_az_deg_
       << "," << hw_antenna_mount_el_deg_ << "]"
       << "\n[Mission]\n"
       << "  power_on=" << mission_power_on_
       << " work_mode=" << static_cast<int>(mission_work_mode_)
       << "\n  scan_center=[" << mission_scan_center_az_deg_
       << "," << mission_scan_center_el_deg_ << "]"
       << " scan_rate=" << mission_scan_rate_hz_
       << " start_pos=" << static_cast<int>(mission_scan_start_position_)
       << " sequence=" << static_cast<int>(mission_scan_sequence_)
       << "\n  explicit_bounds=" << mission_use_explicit_scan_bounds_
       << " scan_az=[" << mission_scan_start_az_deg_
       << "," << mission_scan_end_az_deg_ << "]"
       << " scan_el=[" << mission_scan_start_el_deg_
       << "," << mission_scan_end_el_deg_ << "]"
       << "\n[Policy]\n"
       << "  min_snr_db=" << policy_minimum_snr_db_
       << " pfa=" << policy_pfa_
       << " pulse_count=" << policy_pulse_count_
       << " threshold_scale=" << policy_threshold_scale_
       << " stat_detection=" << policy_enable_statistical_detection_
       << "\n[Environment]\n"
       << "  preset=" << static_cast<int>(env_preset_)
       << " atmos_model=" << env_enable_physical_model_
       << " pressure_hpa=" << env_pressure_hpa_
       << " temp_k=" << env_temperature_k_
       << " humidity=" << env_relative_humidity_
       << "\n";
  }

 private:
  // ==================== 内部方法 ====================

  /**
   * @brief 将 EsrSessionConfig 所有叶节点参数平铺至私有成员。
   *
   * 这是 preStart() 的核心步骤，将层次化配置结构展开为类的扁平成员变量，
   * 使外部引擎能通过 getter 直接访问每个配置项，无需理解库内部结构层次。
   */
  void flattenConfig(const esr_config::EsrSessionConfig& config) {
    // ---- 硬件域 (Hardware) ----
    const auto& hw = config.hardware;
    hw_receiver_band_lower_hz_ = static_cast<double>(hw.receiver_band_lower_hz);
    hw_receiver_band_upper_hz_ = static_cast<double>(hw.receiver_band_upper_hz);
    hw_receiver_sensitivity_w_ = static_cast<double>(hw.receiver_sensitivity_w);
    hw_integrated_receive_loss_db_ = static_cast<double>(hw.integrated_receive_loss_db);
    hw_beam_az_width_deg_ = static_cast<double>(hw.beam_az_width_deg);
    hw_beam_el_width_deg_ = static_cast<double>(hw.beam_el_width_deg);
    hw_az_scan_range_deg_ = static_cast<double>(hw.az_scan_range_deg);
    hw_el_scan_range_deg_ = static_cast<double>(hw.el_scan_range_deg);
    hw_antenna_mount_az_deg_ = static_cast<double>(hw.antenna_mount_az_deg);
    hw_antenna_mount_el_deg_ = static_cast<double>(hw.antenna_mount_el_deg);

    // ---- 任务域 (Mission) ----
    const auto& mission = config.mission;
    mission_power_on_ = mission.power_on;
    mission_work_mode_ = mission.work_mode;

    const auto& scan = mission.scan;
    mission_scan_center_az_deg_ = static_cast<double>(scan.scan_center_az_deg);
    mission_scan_center_el_deg_ = static_cast<double>(scan.scan_center_el_deg);
    mission_scan_rate_hz_ = static_cast<double>(scan.scan_rate_hz);
    mission_scan_start_position_ = scan.scan_start_position;
    mission_scan_sequence_ = scan.scan_sequence;
    mission_use_explicit_scan_bounds_ = scan.use_explicit_scan_bounds;
    mission_scan_start_az_deg_ = static_cast<double>(scan.scan_start_az_deg);
    mission_scan_end_az_deg_ = static_cast<double>(scan.scan_end_az_deg);
    mission_scan_start_el_deg_ = static_cast<double>(scan.scan_start_el_deg);
    mission_scan_end_el_deg_ = static_cast<double>(scan.scan_end_el_deg);

    // ---- 策略域 (Policy) ----
    const auto& det = config.policy.detection;
    policy_minimum_snr_db_ = static_cast<double>(det.minimum_snr_db);
    policy_pfa_ = static_cast<double>(det.pfa);
    policy_pulse_count_ = det.pulse_count;
    policy_threshold_scale_ = static_cast<double>(det.threshold_scale);
    policy_enable_statistical_detection_ = det.enable_statistical_detection;

    // ---- 环境域 (Environment) ----
    const auto& scenario = config.environment.scenario_config;
    env_preset_ = scenario.preset;
    env_enable_physical_model_ = scenario.atmospheric_physics.enable_physical_model;
    env_pressure_hpa_ = static_cast<double>(scenario.atmospheric_physics.pressure_hpa);
    env_temperature_k_ = static_cast<double>(scenario.atmospheric_physics.temperature_k);
    env_relative_humidity_ = static_cast<double>(scenario.atmospheric_physics.relative_humidity);
    env_has_simulation_unix_seconds_ = scenario.atmospheric_context.has_simulation_unix_seconds;
    env_simulation_unix_seconds_ = scenario.atmospheric_context.simulation_unix_seconds;
    env_solar_flux_f107a_ = static_cast<double>(scenario.atmospheric_context.solar_flux_f107a);
    env_solar_flux_f107_ = static_cast<double>(scenario.atmospheric_context.solar_flux_f107);
    env_geomagnetic_ap_ = static_cast<double>(scenario.atmospheric_context.geomagnetic_ap);
  }

  /**
   * @brief 收集所有注册回调，合成为 EsrRuntimeConfigPatch。
   *
   * 按注册顺序依次调用所有回调。每个回调独立修改补丁，
   * 后调用的回调可覆盖先前回调已设置的字段。
   */
  esr_config::EsrRuntimeConfigPatch collectConfigPatches() {
    esr_config::EsrRuntimeConfigPatch patch;
    for (const auto& cb : config_patch_callbacks_) {
      if (cb) cb(patch);
    }
    return patch;
  }

  /// 判断补丁中是否至少有一个字段被设置。
  static bool hasAnyPatch(const esr_config::EsrRuntimeConfigPatch& patch) {
    return patch.has_mission || patch.has_policy || patch.has_environment ||
           patch.has_sensor_enabled || patch.has_work_mode ||
           patch.has_scan_rate_hz || patch.has_scan_start_position ||
           patch.has_scan_sequence || patch.has_scan_center_az_deg ||
           patch.has_scan_center_el_deg || patch.has_explicit_scan_bounds;
  }

  /// 构造默认环境输入（用于 stepImp 的默认值）。
  static esr_session::EsrEnvironmentInput MakeDefaultEnvironmentInput() {
    esr_session::EsrEnvironmentInput env;
    env.propagation_profile = esr_session::EsrPropagationEnvironmentProfile::kTypical;
    env.clutter_density = esr_session::EsrClutterDensityLevel::kMedium;
    env.spectrum_occupancy_ratio = 0.0f;
    env.atmospheric_observation.relative_humidity_ratio = 0.5f;
    env.atmospheric_observation.precipitation_rate_mmph = 0.0f;
    env.atmospheric_observation.visibility_km = 20.0f;
    return env;
  }

  // ==================== 内部状态 ====================

  esr_session::EsrSession session_{};
  esr_session::EsrCycleInput last_input_{};
  esr_session::EsrCycleResult last_result_{};
  esr_session::EsrEnvironmentInputState environment_state_{MakeDefaultEnvironmentInput()};

  /// 生命周期记录器（三视图之一）
  esr_session::EsrEmitterLifecycleRecorder lifecycle_recorder_{};
  std::vector<esr_session::EsrEmitterLifecycleEvent> lifecycle_events_{};

  bool initialized_{false};
  bool started_{false};
  bool trace_enabled_{false};
  std::uint32_t cycle_index_{0};

  // 运行期回调列表
  std::vector<ConfigPatchCallback> config_patch_callbacks_;

  // ==================== 平铺的四域参数 ====================

  // -- 硬件域 (Hardware) --
  double hw_receiver_band_lower_hz_{0.0};
  double hw_receiver_band_upper_hz_{0.0};
  double hw_receiver_sensitivity_w_{0.0};
  double hw_integrated_receive_loss_db_{0.0};
  double hw_beam_az_width_deg_{0.0};
  double hw_beam_el_width_deg_{0.0};
  double hw_az_scan_range_deg_{0.0};
  double hw_el_scan_range_deg_{0.0};
  double hw_antenna_mount_az_deg_{0.0};
  double hw_antenna_mount_el_deg_{0.0};

  // -- 任务域 (Mission) --
  bool mission_power_on_{true};
  esr_config::EsrWorkMode mission_work_mode_{esr_config::EsrWorkMode::kEsm};
  double mission_scan_center_az_deg_{0.0};
  double mission_scan_center_el_deg_{0.0};
  double mission_scan_rate_hz_{0.0};
  esr_config::EsrScanStartPosition mission_scan_start_position_{
      esr_config::EsrScanStartPosition::kLeftTop};
  esr_config::EsrScanSequence mission_scan_sequence_{
      esr_config::EsrScanSequence::kAzimuthFirst};
  bool mission_use_explicit_scan_bounds_{false};
  double mission_scan_start_az_deg_{0.0};
  double mission_scan_end_az_deg_{0.0};
  double mission_scan_start_el_deg_{0.0};
  double mission_scan_end_el_deg_{0.0};

  // -- 策略域 (Policy) --
  double policy_minimum_snr_db_{0.0};
  double policy_pfa_{0.0};
  std::uint32_t policy_pulse_count_{0U};
  double policy_threshold_scale_{0.0};
  bool policy_enable_statistical_detection_{true};

  // -- 环境域 (Environment) --
  esr_config::EsrEnvironmentPreset env_preset_{
      esr_config::EsrEnvironmentPreset::kStandard};
  bool env_enable_physical_model_{false};
  double env_pressure_hpa_{0.0};
  double env_temperature_k_{0.0};
  double env_relative_humidity_{0.0};
  bool env_has_simulation_unix_seconds_{false};
  std::int64_t env_simulation_unix_seconds_{0};
  double env_solar_flux_f107a_{0.0};
  double env_solar_flux_f107_{0.0};
  double env_geomagnetic_ap_{0.0};
};

#endif  // EXAMPLES_ESR_MODULE_H_
