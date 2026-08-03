/**
 * @file EosModule.h
 * @brief 光电传感器集成模块 — 适配外部仿真引擎的标准生命周期。
 *
 * @par 设计目标
 * 提供一种"复制即可用"的集成模式，使外部引擎（仿真框架、数字孪生平台等）
 * 能通过统一生命周期驱动光电传感器模块：
 *   - initialize() → 初始化内部状态
 *   - preStart()  → 从文件加载四域配置并平铺至私有成员
 *   - stepImp()   → 每周期执行光学传感器仿真
 *   - 订阅者模式    → 引擎可在运行期回调注入配置变更
 *
 * @par 典型使用流程
 * @code
 *   EosModule eos;
 *   eos.initialize();
 *   eos.preStart("configs/electro_optical.json");
 *
 *   // 注册运行期回调（可选）
 *   eos.registerConfigPatchCallback([](EosRuntimeConfigPatch& patch) {
 *     patch.has_work_mode = true;
 *     patch.work_mode = EosWorkMode::kInfraredOnly;
 *   });
 *
 *   while (running) {
 *     eos.stepImp(input);
 *     const auto& result = eos.lastResult();
 *     // ... 使用结果
 *   }
 * @endcode
 *
 * @par 配置平铺（Config Flattening）
 * preStart() 将 JSON 中的四域层次配置（hardware/mission/policy/environment）
 * 全部展开为类的扁平私有成员变量（hw_* / mission_* / policy_* / env_*），
 * 便于引擎参数管理系统直接按名访问每个叶节点参数。
 */

#ifndef EXAMPLES_EOS_MODULE_H_
#define EXAMPLES_EOS_MODULE_H_

#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/electro_optical_sensor/session/EosCycleOutputAdapter.h"
#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "EosDebugViewToJson.h"
#include "1q/electro_optical_sensor/session/EosReplaySession.h"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "config_loader.h"

namespace eos_config = electro_optical_sensor::config;
namespace eos_session = electro_optical_sensor::session;

/**
 * @brief 光电传感器集成模块。
 *
 * 封装 EosSession 并提供引擎友好的集成接口：
 *   - initialize / preStart / stepImp 三阶段生命周期
 *   - 四域配置平铺至扁平私有成员
 *   - 基于回调的订阅者模式支持运行期配置修改
 */
class EosModule {
 public:
  // ==================== 构造 / 析构 ====================

  EosModule() = default;
  ~EosModule() = default;

  // 不可拷贝
  EosModule(const EosModule&) = delete;
  EosModule& operator=(const EosModule&) = delete;

  // 可移动
  EosModule(EosModule&&) = default;
  EosModule& operator=(EosModule&&) = default;

  // ==================== 生命周期接口 ====================

  /**
   * @brief 初始化内部状态。
   *
   * 创建 EosSession 实例并准备环境状态管理器。
   * 可在构造后立即调用；默认使用 EosSessionConfig{} 创建空配置 Session。
   *
   * @param[in] config  初始会话配置（可选，默认空配置）
   * @return true  初始化成功
   * @return false 初始化失败（不应发生，留作扩展）
   */
  bool initialize(const eos_config::EosSessionConfig& config = {}) {
    session_ = eos_session::EosSession::Create(config);
    initialized_ = true;
    return true;
  }

  /**
   * @brief 预启动：从 JSON 配置文件加载四域参数，平铺至私有成员。
   *
   * 调用 initialize() 之后、首次 stepImp() 之前调用。
   * 执行步骤：
   *   1. 解析 JSON 文件为 EosSessionConfig
   *   2. 将 EosSessionConfig 中所有叶节点参数平铺至本类私有成员
   *   3. 使用加载的配置重新创建 EosSession
   *
   * @param[in] config_path  JSON 配置文件路径
   * @return true  加载并创建成功
   * @return false 文件读取或解析失败
   */
  bool preStart(const std::string& config_path) {
    if (!initialized_) {
      std::cerr << "[EosModule] ERROR: must call initialize() before preStart()\n";
      return false;
    }

    // 1. 从 JSON 文件加载配置
    eos_config::EosSessionConfig config;
    std::string error;
    if (!examples::LoadEosSessionConfigFromFile(config_path.c_str(), &config, &error)) {
      std::cerr << "[EosModule] ERROR: failed to load config from " << config_path
                << ": " << error << "\n";
      return false;
    }

    // 2. 平铺至私有成员
    flattenConfig(config);

    // 3. 用完整配置重建 Session，并挂载生命周期记录器
    session_ = eos_session::EosSession::Create(config);
    session_.AttachDetectionLifecycleRecorder(&lifecycle_recorder_);
    started_ = true;
    return true;
  }

  /**
   * @brief 主步进函数：执行一个光电传感器仿真周期。
   *
   * 每个周期执行：
   *   1. 收集所有注册回调的运行期配置补丁
   *   2. 应用补丁到 Session（如有）
   *   3. 调用 Session::StepWithResult
   *   4. 缓存结果并记录生命周期事件
   *
   * 外部引擎应使用 EosCycleInputAdapter::Build 构造 EosCycleInput，
   * 或直接填充各字段后传入。
   *
   * @param[in] input  本周期完整输入
   */
  void stepImp(const eos_session::EosCycleInput& input) {
    // 0. 检查是否已启动
    if (!started_) {
      std::cerr << "[EosModule] WARN: stepImp called before preStart(), using default config\n";
      if (!initialized_) initialize();
      eos_config::EosSessionConfig default_config;
      flattenConfig(default_config);
      session_ = eos_session::EosSession::Create(default_config);
      session_.AttachDetectionLifecycleRecorder(&lifecycle_recorder_);
      started_ = true;
    }

    // 1. 收集运行期配置补丁
    eos_config::EosRuntimeConfigPatch patch = collectConfigPatches();

    // 2. 应用补丁（如有）
    if (hasAnyPatch(patch)) {
      if (!session_.TryApplyRuntimeConfig(patch)) {
        std::cerr << "[EosModule] WARN: runtime config patch rejected at cycle "
                  << last_result_.input_cycle_index + 1 << "\n";
      }
    }

    // 3. 执行一个周期
    eos_session::EosCycleInput mutable_input = input;
    if (mutable_input.cycle_index == 0) {
      mutable_input.cycle_index = cycle_index_;
    }
    last_input_ = mutable_input;
    last_result_ = session_.StepWithResult(mutable_input);
    ++cycle_index_;

    // 4. 读取生命周期事件（Session 自动驱动 recorder）
    lifecycle_events_ = lifecycle_recorder_.GetLastEvents();
  }

  // ==================== 订阅者模式 ====================

  /** @brief 运行期配置补丁回调类型。引擎通过回调修改补丁各字段以动态调整运行参数。 */
  using ConfigPatchCallback =
      std::function<void(eos_config::EosRuntimeConfigPatch& patch)>;

  /**
   * @brief 注册运行期配置变更回调。
   *
   * 注册的回调将在每次 stepImp 开始时调用。
   * 回调接收 EosRuntimeConfigPatch 引用，引擎可设置其 has_* 标志和对应字段值。
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
  const eos_session::EosCycleResult& lastResult() const { return last_result_; }

  /** @brief 返回内部 EosSession 引用（高级用途）。 */
  eos_session::EosSession& session() { return session_; }

  /** @brief 返回内部 EosSession 常量引用。 */
  const eos_session::EosSession& session() const { return session_; }

  /** @brief 当前已执行的周期数。 */
  std::uint32_t cycleCount() const { return cycle_index_; }

  /** @brief 模块是否已通过 preStart 启动。 */
  bool isStarted() const { return started_; }

  // ==================== 输出三视图 ====================
  //
  // 根据光电传感器模块设计约定，EOS 输出分为三个层次：
  //
  //   1. EosOutputFrame（系统输出）      — 探测输出帧，通过 lastResult() 获取
  //   2. EosOutputDebugView（调试视图）   — 人读排查视图，通过 buildLastDebugView() 获取
  //   3. EosDetectionLifecycleRecorder（生命周期） — 首次发现/更新/丢失事件，通过 lifecycleEvents() 获取
  //
  // 此外将通过 buildExternalOutput() 提供 ECEF 外部坐标转换。
  // （跨模块差异：AR/EOS/ESR 均提供 buildExternalOutput()；SAR 因产品为聚焦图像、
  // 无需 ECEF 坐标转换，不暴露此能力，见 examples/sar/SarModule.h。）

  /** @brief 返回最近一次 stepImp 输入的缓存（供三视图构建使用）。 */
  const eos_session::EosCycleInput& lastInput() const { return last_input_; }

  /**
   * @brief 构建调试视图（三视图之一）。
   *
   * 将最后周期的输入与输出合成为人读可排查的目标探测状态视图。
   * 显示每个输入目标是否有对应探测、SNR、位置等。
   */
  eos_session::EosOutputDebugView buildLastDebugView() const {
    return eos_session::EosOutputDebugViewBuilder::Build(last_input_, last_result_);
  }

  /**
   * @brief 把最近一次调试视图序列化为 JSON 字符串（session_contract.md 规则 12 参考实现）。
   *
   * 集成方把返回的字符串写入自己的日志/事件系统即可；跨周期累积由调用方日志承担。
   * 序列化函数见 examples/electro_optical/EosDebugViewToJson.h，可独立 copy。
   */
  std::string buildLastDebugViewJson() const {
    return EosDebugViewToJson(buildLastDebugView());
  }

  /** @brief 返回最近一次的生命周期事件列表（三视图之一）。 */
  const std::vector<eos_session::EosDetectionLifecycleEvent>& lifecycleEvents() const {
    return lifecycle_events_;
  }

  /**
   * @brief 构建外部 ECEF 坐标输出（辅助视图）。
   *
   * 将传感器局部坐标系的 EosOutputFrame 转换为外部引擎可用的 ECEF 世界坐标。
   * 本重载使用 EosExternalPoseInput 一次传递平台位姿与参考系。
   *
   * @param[in]  platform  外部平台位姿输入（ECEF 位置/速度/姿态）
   * @param[out] output    外部 ECEF 探测输出帧
   * @return true  转换成功
   */
  bool buildExternalOutput(
      const eos_session::EosExternalPoseInput& platform,
      eos_session::EosExternalOutputFrame* output) const {
    return eos_session::EosCycleOutputAdapter::Build(
        platform, last_result_.output_frame, output);
  }

  /**
   * @brief 构建外部 ECEF 坐标输出（辅助视图，显式参考系版本）。
   *
   * 将传感器局部坐标系的 EosOutputFrame 转换为外部引擎可用的 ECEF 世界坐标。
   * 本重载允许调用方分离指定局部参考系与平台位姿。
   *
   * @param[in]  reference       局部坐标参考系（决定 ECEF/ENU 到 EOS 局部坐标的转换基准）
   * @param[in]  platform_pose   平台局部位姿状态
   * @param[out] output          外部 ECEF 探测输出帧
   * @return true  转换成功
   */
  bool buildExternalOutput(
      const oneq::coordinate::LocalFrameReference& reference,
      const oneq::foundation::PoseState& platform_pose,
      eos_session::EosExternalOutputFrame* output) const {
    return eos_session::EosCycleOutputAdapter::Build(
        reference, platform_pose, last_result_.output_frame, output);
  }

  // ==================== 回放 (Replay) ====================
  //
  // EOS 支持记录运行期 trace 并在事后回放，用于调试和回归验证。
  // EosModule 提供 enableTrace() 开启录制，replayTrace() 供事后回放。

  /**
   * @brief 启用运行期 trace 录制（API 占位）。
   *
   * 实际工程中应在构造/initialize 时传入 trace 选项，
   * 使用 EosTraceSession 代替 EosSession 完成录制。
   * 回放通过静态方法 replayTrace() 完成。
   *
   * @param[in] trace_dir   trace 输出目录名称（仅作标识）
   * @return true  （占位返回值）
   */
  bool enableTrace(const std::string& trace_dir) {
    std::cerr << "[EosModule] enableTrace(\"" << trace_dir << "\") — "
              << "实际工程中应使用 EosTraceSession 代替 EosSession；"
              << "此处仅做 API 演示。\n";
    trace_enabled_ = true;
    return true;
  }

  /**
   * @brief 回放已录制的 trace（静态方法）。
   *
   * @param[in] trace_dir  之前 enableTrace() 的输出目录
   * @return EosReplaySessionResult  回放结果详情
   */
  static eos_session::EosReplaySessionResult replayTrace(const std::string& trace_dir) {
    return eos_session::ReplayEosTrace(trace_dir);
  }

  // ==================== 调试 / 诊断 ====================

  /** @brief 打印当前平铺配置摘要（用于验证 flatten 结果）。 */
  void printConfigSummary(std::ostream& os = std::cout) const {
    os << "=== EosModule Config Summary ===\n"
       << "[Hardware]\n"
       << "  wavelength_lower_um=" << hw_wavelength_lower_um_
       << " wavelength_upper_um=" << hw_wavelength_upper_um_
       << " aperture_m=" << hw_optical_aperture_m_
       << "\n  detectivity=" << hw_detector_detectivity_cm_sqrt_hz_per_w_
       << " area_cm2=" << hw_detector_area_cm2_
       << " depression=[" << hw_min_detection_depression_deg_
       << "," << hw_max_detection_depression_deg_ << "]"
       << "\n[Mission]\n"
       << "  work_mode=" << static_cast<int>(mission_work_mode_)
       << " hfov=" << mission_horizontal_fov_deg_
       << " vfov=" << mission_vertical_fov_deg_
       << " scan_rate=" << mission_scan_rate_deg_per_sec_
       << " frame_rate=" << mission_frame_rate_hz_
       << "\n  scan_az=[" << mission_scan_start_az_deg_
       << "," << mission_scan_end_az_deg_ << "]"
       << " scan_el=" << mission_scan_center_el_deg_
       << " depression=" << mission_boresight_depression_deg_
       << " sensor_enabled=" << sensor_enabled_
       << "\n[Policy]\n"
       << "  min_snr_db=" << policy_minimum_snr_db_
       << " sensitivity_w=" << policy_detection_sensitivity_w_
       << " ref_irradiance_w_m2=" << policy_visible_reference_irradiance_w_m2_
       << "\n  stray_light_filter=" << policy_enable_straylight_filter_
       << " hood_inner=" << policy_hood_inner_half_angle_deg_
       << " hood_outer=" << policy_hood_outer_half_angle_deg_
       << " hood_suppress=[" << policy_hood_min_suppression_ratio_
       << "," << policy_hood_max_suppression_ratio_ << "]"
       << "\n[Environment]\n"
       << "  preset=" << static_cast<int>(env_preset_)
       << "\n  physical_model=" << env_enable_physical_model_
       << " pressure_hpa=" << env_pressure_hpa_
       << " temp_k=" << env_temperature_k_
       << " humidity=" << env_relative_humidity_
       << "\n";
  }

 private:
  // ==================== 内部方法 ====================

  /**
   * @brief 将 EosSessionConfig 所有叶节点参数平铺至私有成员。
   *
   * 这是 preStart() 的核心步骤，将层次化配置结构展开为类的扁平成员变量，
   * 使外部引擎能通过 getter 直接访问每个配置项，无需理解库内部结构层次。
   */
  void flattenConfig(const eos_config::EosSessionConfig& config) {
    // ---- 硬件域 (Hardware) ----
    const auto& hw = config.hardware;
    hw_wavelength_lower_um_ = static_cast<double>(hw.wavelength_lower_um);
    hw_wavelength_upper_um_ = static_cast<double>(hw.wavelength_upper_um);
    hw_optical_aperture_m_ = static_cast<double>(hw.optical_aperture_m);
    hw_detector_detectivity_cm_sqrt_hz_per_w_ =
        static_cast<double>(hw.detector_detectivity_cm_sqrt_hz_per_w);
    hw_detector_area_cm2_ = static_cast<double>(hw.detector_area_cm2);
    hw_min_detection_depression_deg_ = static_cast<double>(hw.min_detection_depression_deg);
    hw_max_detection_depression_deg_ = static_cast<double>(hw.max_detection_depression_deg);

    // ---- 任务域 (Mission) ----
    const auto& mission = config.mission;
    mission_work_mode_ = mission.work_mode;
    mission_horizontal_fov_deg_ = static_cast<double>(mission.horizontal_fov_deg);
    mission_vertical_fov_deg_ = static_cast<double>(mission.vertical_fov_deg);
    mission_scan_rate_deg_per_sec_ = static_cast<double>(mission.scan_rate_deg_per_sec);
    mission_frame_rate_hz_ = static_cast<double>(mission.frame_rate_hz);
    mission_scan_start_az_deg_ = static_cast<double>(mission.scan_start_az_deg);
    mission_scan_end_az_deg_ = static_cast<double>(mission.scan_end_az_deg);
    mission_scan_center_el_deg_ = static_cast<double>(mission.scan_center_el_deg);
    mission_boresight_depression_deg_ = static_cast<double>(mission.boresight_depression_deg);
    // 电源状态提升到 SessionConfig 顶层（COMMON-OQ-4 收敛）
    sensor_enabled_ = config.sensor_enabled;

    // ---- 策略域 (Policy) ----
    const auto& policy = config.policy;
    const auto& det = policy.detection;
    policy_minimum_snr_db_ = static_cast<double>(det.minimum_snr_db);
    policy_detection_sensitivity_w_ = static_cast<double>(det.detection_sensitivity_w);
    policy_visible_reference_irradiance_w_m2_ =
        static_cast<double>(det.visible_reference_irradiance_w_m2);

    const auto& stray = policy.stray_light;
    policy_enable_straylight_filter_ = stray.enable_straylight_filter;
    policy_hood_inner_half_angle_deg_ = static_cast<double>(stray.hood_inner_half_angle_deg);
    policy_hood_outer_half_angle_deg_ = static_cast<double>(stray.hood_outer_half_angle_deg);
    policy_hood_min_suppression_ratio_ = static_cast<double>(stray.hood_min_suppression_ratio);
    policy_hood_max_suppression_ratio_ = static_cast<double>(stray.hood_max_suppression_ratio);

    // ---- 环境域 (Environment) ----
    const auto& env = config.environment;
    const auto& scenario = env.scenario_config;
    env_preset_ = scenario.preset;
    env_enable_physical_model_ = scenario.atmospheric_physics.enable_physical_model;
    env_pressure_hpa_ = static_cast<double>(scenario.atmospheric_physics.pressure_hpa);
    env_temperature_k_ = static_cast<double>(scenario.atmospheric_physics.temperature_k);
    env_relative_humidity_ = static_cast<double>(scenario.atmospheric_physics.relative_humidity);
  }

  /**
   * @brief 收集所有注册回调，合成为 EosRuntimeConfigPatch。
   *
   * 按注册顺序依次调用所有回调。每个回调独立修改补丁，
   * 后调用的回调可覆盖先前回调已设置的字段。
   */
  eos_config::EosRuntimeConfigPatch collectConfigPatches() {
    eos_config::EosRuntimeConfigPatch patch;
    for (const auto& cb : config_patch_callbacks_) {
      if (cb) cb(patch);
    }
    return patch;
  }

  /// 判断补丁中是否至少有一个字段被设置。
  static bool hasAnyPatch(const eos_config::EosRuntimeConfigPatch& patch) {
    return patch.has_mission || patch.has_policy || patch.has_environment ||
           patch.has_work_mode || patch.has_scan_rate_deg_per_sec ||
           patch.has_frame_rate_hz || patch.has_sensor_enabled;
  }

  // ==================== 内部状态 ====================

  eos_session::EosSession session_{};
  eos_session::EosCycleInput last_input_{};
  eos_session::EosCycleResult last_result_{};

  /// 生命周期记录器（三视图之一）
  eos_session::EosDetectionLifecycleRecorder lifecycle_recorder_{};
  std::vector<eos_session::EosDetectionLifecycleEvent> lifecycle_events_{};

  bool initialized_{false};
  bool started_{false};
  bool trace_enabled_{false};
  std::uint32_t cycle_index_{0};

  // 运行期回调列表
  std::vector<ConfigPatchCallback> config_patch_callbacks_;

  // ==================== 平铺的四域参数 ====================

  // -- 硬件域 (Hardware) --
  double hw_wavelength_lower_um_{0.0};
  double hw_wavelength_upper_um_{0.0};
  double hw_optical_aperture_m_{0.0};
  double hw_detector_detectivity_cm_sqrt_hz_per_w_{0.0};
  double hw_detector_area_cm2_{0.0};
  double hw_min_detection_depression_deg_{0.0};
  double hw_max_detection_depression_deg_{0.0};

  // -- 任务域 (Mission) --
  eos_config::EosWorkMode mission_work_mode_{eos_config::EosWorkMode::kFused};
  double mission_horizontal_fov_deg_{0.0};
  double mission_vertical_fov_deg_{0.0};
  double mission_scan_rate_deg_per_sec_{0.0};
  double mission_frame_rate_hz_{0.0};
  double mission_scan_start_az_deg_{0.0};
  double mission_scan_end_az_deg_{0.0};
  double mission_scan_center_el_deg_{0.0};
  double mission_boresight_depression_deg_{0.0};
  bool sensor_enabled_{true};

  // -- 策略域 (Policy) --
  // Detection
  double policy_minimum_snr_db_{0.0};
  double policy_detection_sensitivity_w_{0.0};
  double policy_visible_reference_irradiance_w_m2_{0.0};
  // Stray light
  bool policy_enable_straylight_filter_{false};
  double policy_hood_inner_half_angle_deg_{0.0};
  double policy_hood_outer_half_angle_deg_{0.0};
  double policy_hood_min_suppression_ratio_{0.0};
  double policy_hood_max_suppression_ratio_{0.0};

  // -- 环境域 (Environment) --
  eos_config::EosEnvironmentPreset env_preset_{
      eos_config::EosEnvironmentPreset::kStandard};
  bool env_enable_physical_model_{false};
  double env_pressure_hpa_{0.0};
  double env_temperature_k_{0.0};
  double env_relative_humidity_{0.0};
};

#endif  // EXAMPLES_EOS_MODULE_H_
