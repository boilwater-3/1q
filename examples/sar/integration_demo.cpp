/**
 * @file integration_demo.cpp
 * @brief SAR 集成示例 — 展示 SarModule 在外部引擎中的使用方式。
 *
 * @par 场景描述
 * 模拟外部仿真引擎的集成模式，演示 SarModule 的完整生命周期。
 * 平台沿经线匀速平飞，场景中心有一个静止点目标。
 *
 * @par 关键概念
 * - 三阶段生命周期：initialize -> preStart -> stepImp
 * - 配置平铺 (Config Flattening)：层次化 SarSessionConfig 展开为私有扁平成员
 * - 订阅者模式：registerConfigPatchCallback 注册回调，stepImp 每周期自动收集
 * - 平台使用 LLA+NED 大地坐标（SarPlatformState），不同于 AR 的 ECEF 位姿
 * - 输入直接构造 SarCycleInput（填 platform/point_targets，raw_iq 留空），无需 Adapter。
 *   原因：SAR 内部 SarPlatformState/SarPointTarget 本就存 LLA+NED，调用方填什么格式、
 *   内部就用什么格式，平台/目标无需任何坐标转换。这跟 AR/EOS/ESR 不同——那三个模块
 *   内部存局部直角坐标（雷达为原点的 x/y/z），外部却给 ECEF，所以它们的 Adapter 是
 *   "ECEF→局部直角"的必经转换。SAR 的 Adapter 唯一的活儿是转换【脉冲】坐标
 *   （外部 ECEF/LLA → scene-center ENU），且只在提供外部脉冲（SarExternalPulseInput）
 *   时才做；它从不转换平台/目标。本 demo 不提供外部脉冲（raw_iq 留空），系统据此判定
 *   "无外部 IQ"，走内部 raw echo 生成路径（根据 platform+target 合成回波），所以根本
 *   用不到 Adapter。详见 @par 为何 SAR 不需要 Adapter。
 * - SAR 输出为聚焦图像而非轨迹/检测，无外部 ECEF 坐标转换适配器
 *
 * @par 为何 SAR 不需要 Adapter
 * 对比可厘清"何时需要 Adapter"：
 *   AR/EOS/ESR：外部 ECEF 位姿 → Adapter 转成内部局部直角 → Adapter 是刚需
 *   SAR：外部 LLA → 内部也存 LLA（格式一致）→ Adapter 对平台/目标无事可做
 * SAR 的 Adapter 仅在外部脉冲输入时把脉冲坐标转到 scene-center ENU 局部直角；
 * 不提供外部脉冲时（本 demo），Adapter::Build 与直接构造结果完全一致，故直接构造。
 *
 * @par 运行方式
 *   cd build/llvm-ninja-release-local/
 *   ./bin/sar_integration_demo
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "SarModule.h"

// =============================================================================
// 第一部分：辅助函数 — SAR 场景与配置构造
// =============================================================================

namespace {

/// 地球参数与场景常量。
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kPlatformAltitudeM = 10000.0;
constexpr double kTargetAltitudeM = 0.0;
constexpr double kNominalSlantRangeM = 100000.0;  // 100 km（匹配原始 session_usage 验证参数）
constexpr double kPlatformSpeedMps = 180.0;
constexpr double kSceneCenterLatDeg = 40.1;   ///< 场景中心纬度（与平台 ~11km 地面偏移，使斜距≈15km）
constexpr double kSceneCenterLonDeg = -105.0;
constexpr double kTargetRcsDbsm = 80.0;     ///< 强点目标标定源，确保通过真实链路预算 SNR 门限
constexpr std::uint32_t kPulseCount = 33U;    ///< 孔径脉冲数（匹配原始 session_usage 验证参数）
constexpr std::uint32_t kRangeSamples = 1024U; ///< 距离向采样点数
constexpr double kPulseRepetitionFrequencyHz = 100.0;  ///< PRF（与下方 JSON hardware 配置一致）
constexpr double kSampleRateHz = 1.0e6;        ///< 采样率（与下方 JSON hardware 配置一致）
constexpr double kDefaultDtSec = 0.1;          ///< 每周期步长（适配 100 Hz PRF）
const char* kTempConfigPath = "/tmp/1q_sar_integration_demo_config.json";

/**
 * @brief 将演示配置写入临时 JSON 文件，供 SarModule::preStart 加载。
 *
 * JSON 字段名与 config_loader_detail.h / config_loader_common.h 一致。
 * 之所以写文件而非代码直传，是因为 SarModule::preStart 是集成模式的标准
 * 入口（负责文件加载 + flatten），与 AR/EOS/ESR 三模块保持对称。
 */
void WriteTempConfig() {
  std::ofstream ofs(kTempConfigPath);
  if (!ofs) {
    std::cerr << "  FATAL: 无法写入临时配置 " << kTempConfigPath << "\n";
    std::exit(1);
  }

  ofs << "{\n"
      << "  \"hardware\": {\n"
      << "    \"carrier_frequency_hz\": 9.6e9,\n"
      << "    \"bandwidth_hz\": 0.5e6,\n"
      << "    \"pulse_width_s\": 20.0e-6,\n"
      << "    \"pulse_repetition_frequency_hz\": 100.0,\n"
      << "    \"sample_rate_hz\": 1.0e6,\n"
      << "    \"peak_power_w\": 10000.0,\n"
      << "    \"antenna_length_m\": 1.2,\n"
      << "    \"antenna_width_m\": 0.3,\n"
      << "    \"antenna_gain_db\": 30.0,\n"
      << "    \"receiver_noise_figure_db\": 4.0,\n"
      << "    \"system_loss_db\": 3.0\n"
      << "  },\n"
      << "  \"mission\": {\n"
      << "    \"scene_center_latitude_deg\": " << kSceneCenterLatDeg << ",\n"
      << "    \"scene_center_longitude_deg\": " << kSceneCenterLonDeg << ",\n"
      << "    \"scene_center_altitude_m\": " << kTargetAltitudeM << ",\n"
      << "    \"nominal_slant_range_m\": " << kNominalSlantRangeM << ",\n"
      << "    \"platform_speed_mps\": " << kPlatformSpeedMps << ",\n"
      << "    \"range_sample_count\": " << kRangeSamples << ",\n"
      << "    \"azimuth_pulse_count\": " << kPulseCount << ",\n"
      << "    \"desired_ground_range_resolution_m\": 1.5,\n"
      << "    \"desired_azimuth_resolution_m\": 1.5,\n"
      << "    \"l2_velocity_error_stddev_x_mps\": 0.0,\n"
      << "    \"l2_velocity_error_stddev_y_mps\": 0.0,\n"
      << "    \"l2_velocity_error_stddev_z_mps\": 0.0,\n"
      << "    \"l2_random_seed\": 0\n"
      << "  },\n"
      << "  \"processing\": {\n"
      << "    \"enable_raw_echo_generation\": true,\n"
      << "    \"enable_l1_rda_imaging\": true,\n"
      << "    \"enable_l2_motion_compensation\": false,\n"
      << "    \"enable_l3_bp_imaging\": false,\n"
      << "    \"enable_diagnostics\": true,\n"
      << "    \"retain_raw_phase_history\": false,\n"
      << "    \"retain_focused_image\": true,\n"
      << "    \"max_allowed_squint_angle_deg\": 5.0,\n"
      << "    \"minimum_snr_db\": -10.0\n"
      << "  },\n"
      << "  \"environment\": {\n"
      << "    \"terrain_reference_altitude_m\": 0.0,\n"
      << "    \"atmospheric_loss_db_per_km\": 0.0,\n"
      << "    \"surface_backscatter_sigma0_db\": -12.0,\n"
      << "    \"use_flat_earth_geometry\": true,\n"
      << "    \"enable_atmospheric_attenuation\": false\n"
      << "  }\n"
      << "}\n";

  if (!ofs) {
    std::cerr << "  FATAL: 写入临时配置失败 " << kTempConfigPath << "\n";
    std::exit(1);
  }
}

/**
 * @brief 构建单周期 SAR 输入（平台 + 点目标）。
 *
 * 平台以恒定速度沿经度方向匀速运动；点目标静止位于场景中心。
 * 直接填充 SarCycleInput（platform/point_targets），不提供外部脉冲，
 * 走内部原始回波生成路径。SarPlatformState/SarPointTarget 本就存 LLA+NED，
 * 无需 SarCycleInputAdapter 做坐标转换（该 Adapter 仅在外部脉冲输入时转换脉冲坐标）。
 */
sar::session::SarCycleInput MakeCycleInput(
    std::uint32_t cycle_index,
    const sar::config::SarMissionConfig& mission) {

  // 平台沿东向匀速运动
  const double elapsed_s = static_cast<double>(cycle_index - 1) * kDefaultDtSec;
  const double east_displacement_m = kPlatformSpeedMps * elapsed_s;
  const double delta_lon_deg = east_displacement_m /
      (kEarthRadiusM * std::cos(kSceneCenterLatDeg * kPi / 180.0)) *
      (180.0 / kPi);

  sar::session::SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = static_cast<float>(kDefaultDtSec);

  // 平台状态（直接填充，避免 Adapter 坐标系转换）
  // 平台起始于场景中心以北 ~100km（0.9°），使斜距≈100km
  constexpr double kPlatformLatOffset = 0.9;  // ~100km 地面偏移
  input.platform.time_s = elapsed_s;
  input.platform.latitude_deg = kSceneCenterLatDeg - kPlatformLatOffset;
  input.platform.longitude_deg = kSceneCenterLonDeg + delta_lon_deg;
  input.platform.altitude_m = kPlatformAltitudeM;
  input.platform.velocity_north_mps = 0.0;
  input.platform.velocity_east_mps = kPlatformSpeedMps;
  input.platform.velocity_down_mps = 0.0;
  input.platform.roll_deg = 0.0;
  input.platform.pitch_deg = 0.0;
  input.platform.yaw_deg = 90.0;

  // 点目标（静止位于场景中心，与平台有地面偏移使斜距≈标称值）
  sar::session::SarPointTarget target;
  target.target_id = 101;
  target.target_name = "scene_center_target";
  target.latitude_deg = mission.scene_center_latitude_deg;
  target.longitude_deg = mission.scene_center_longitude_deg;
  target.altitude_m = mission.scene_center_altitude_m;
  target.radar_cross_section_dbsm = kTargetRcsDbsm;

  input.point_targets = {target};

  // 直接构造 SarCycleInput：input 仅负责提供平台状态和点目标列表；
  // range_sample_count / azimuth_pulse_count 等处理参数由 session 从 config 读取，
  // 不在 input 中重复传递。
  return input;
}

/// 人读的处理阶段名称。
const char* StageName(sar::session::SarProcessingStage stage) {
  switch (stage) {
    case sar::session::SarProcessingStage::kNone:
      return "None";
    case sar::session::SarProcessingStage::kRawEcho:
      return "RawEcho";
    case sar::session::SarProcessingStage::kL1RdaImage:
      return "L1RdaImage";
    case sar::session::SarProcessingStage::kL3BpImage:
      return "L3BpImage";
  }
  return "Unknown";
}

/// 人读的产品生命周期事件类型名称。
const char* LifecycleEventKindName(
    sar::session::SarProductLifecycleEventKind kind) {
  switch (kind) {
    case sar::session::SarProductLifecycleEventKind::kImageProduced:
      return "ImageProduced";
    case sar::session::SarProductLifecycleEventKind::kProductUpdated:
      return "ProductUpdated";
    case sar::session::SarProductLifecycleEventKind::kProductLost:
      return "ProductLost";
    case sar::session::SarProductLifecycleEventKind::kProcessingFailed:
      return "ProcessingFailed";
    case sar::session::SarProductLifecycleEventKind::kNoProduct:
      return "NoProduct";
  }
  return "Unknown";
}

}  // namespace

// =============================================================================
// 第二部分：运行期配置模拟管理器
// =============================================================================

/// 配置变更阶段枚举。
enum class ConfigPhase {
  kRdaOn,       ///< L1 RDA 成像开启（默认）
  kToggleL1,    ///< 第 3 周期关闭 RDA
  kRdaOff,      ///< RDA 已关闭
  kDone         ///< 结束
};

/**
 * @brief 模拟外部引擎的运行期配置管理器。
 *
 * 通过 SarModule 的订阅者模式注入运行期配置变更。
 * 在真实集成中，此类可能监听 DDS 主题、UI 控制面板或上级指挥系统的指令。
 */
class SimulatedConfigManager {
 public:
  /** @brief 生成运行期配置补丁。 */
  void evaluatePatch(std::uint32_t cycle,
                     sar::config::SarRuntimeConfigPatch& patch) {
    switch (phase_) {
      case ConfigPhase::kRdaOn:
        if (cycle >= 3) {
          phase_ = ConfigPhase::kToggleL1;
          fillDisableL1Patch(patch);
        }
        break;
      case ConfigPhase::kToggleL1:
        phase_ = ConfigPhase::kRdaOff;
        break;
      case ConfigPhase::kRdaOff:
        if (cycle >= 5) {
          phase_ = ConfigPhase::kDone;
          fillEnableL1Patch(patch);
        }
        break;
      case ConfigPhase::kDone:
        break;
    }
  }

  bool lastPatchApplied() const { return last_patch_applied_; }
  void clearLastPatchFlag() { last_patch_applied_ = false; }

 private:
  void fillDisableL1Patch(sar::config::SarRuntimeConfigPatch& patch) {
    patch.has_enable_l1_rda_imaging = true;
    patch.enable_l1_rda_imaging = false;
    last_patch_applied_ = true;
  }

  void fillEnableL1Patch(sar::config::SarRuntimeConfigPatch& patch) {
    patch.has_enable_l1_rda_imaging = true;
    patch.enable_l1_rda_imaging = true;
    last_patch_applied_ = true;
  }

  ConfigPhase phase_{ConfigPhase::kRdaOn};
  bool last_patch_applied_{false};
};

// =============================================================================
// 第三部分：主函数
// =============================================================================

/**
 * @brief 入口：运行 SAR 集成示例。
 *
 * 遵循六步生命周期流程（与 AR integration_demo 一致）：
 *   1. 构造 SarModule
 *   2. initialize() — 创建初始 SarSession
 *   3. preStart(config_path) — 加载 JSON、平铺配置、重建会话
 *   4. 注册运行期配置回调（订阅者模式）
 *   5. 主仿真循环（stepImp 驱动）
 *   6. 结果输出与三视图展示 + Replay API 演示
 *
 * @return 0 成功，非零失败
 */
int main() {
  constexpr std::uint32_t kNumCycles = 6;

  std::cout << "=== SAR 集成模块 Demo ===\n"
            << "  场景: 单点目标 @ 场景中心\n"
            << "  总周期数: " << kNumCycles << "\n\n";

  // ===============================================
  // 步骤 1：创建 SarModule
  // ===============================================
  // 构造函数仅初始化默认值，不进行重量级操作。
  std::cout << "[1/6] 构造 SarModule...\n";
  SarModule sar;

  // ===============================================
  // 步骤 2：initialize — 初始化内部状态
  // ===============================================
  // 创建 SarSession 实例（使用默认空配置）。
  // 实际参数将在 preStart 阶段通过 JSON 文件加载后重建。
  std::cout << "[2/6] initialize()...\n";
  if (!sar.initialize()) {
    std::cerr << "  ERROR: initialize() 失败\n";
    return 1;
  }
  std::cout << "  SarSession 已创建\n";

  // ===============================================
  // 步骤 3：preStart — 加载配置文件、平铺、初始化
  // ===============================================
  // 将演示配置写入临时 JSON 文件，供 preStart() 加载。
  // 此举保持与 AR/EOS/ESR 三模块集成模式完全对称。
  //
  // 从 JSON 读取四域配置（hardware/mission/processing/environment），
  // 将所有叶节点参数平铺至 SarModule 的私有扁平成员。
  // 此步骤还会用完整配置重建 SarSession。
  //
  // 与 AR 的差异：SAR 的 policy 域在 JSON 中对应 "processing" 节，
  // 以区分 AR 的 "policy" 命名。详见 config_loader_detail.h 的
  // LoadSarProcessing 函数。
  //
  // 保留 mission 副本供循环中构造 SarCycleInput 使用（场景中心坐标、目标位置等）。
  // 直接构造路径无需 Adapter，但 mission 元数据用于确定目标在场景中的放置。
  std::cout << "[3/6] 写入临时配置文件并调用 preStart...\n";
  WriteTempConfig();
  std::cout << "   配置文件: " << kTempConfigPath << "\n";

  if (!sar.preStart(kTempConfigPath)) {
    std::cerr << "  ERROR: preStart() 失败\n";
    return 1;
  }
  std::cout << "  配置加载并平铺完成\n";

  // 从代码构造一份 mission 副本（供循环中 adapter 使用）。
  // 内容与临时 JSON 文件中的 mission 域完全一致。
  sar::config::SarMissionConfig mission_config;
  mission_config.scene_center_latitude_deg = kSceneCenterLatDeg;
  mission_config.scene_center_longitude_deg = kSceneCenterLonDeg;
  mission_config.scene_center_altitude_m = kTargetAltitudeM;
  mission_config.nominal_slant_range_m = kNominalSlantRangeM;
  mission_config.platform_speed_mps = kPlatformSpeedMps;
  mission_config.range_sample_count = kRangeSamples;
  mission_config.azimuth_pulse_count = kPulseCount;
  mission_config.desired_ground_range_resolution_m = 1.5;
  mission_config.desired_azimuth_resolution_m = 1.5;

  // 打印平铺配置摘要
  sar.printConfigSummary();

  // ===============================================
  // 步骤 4：注册运行期配置回调（订阅者模式）
  // ===============================================
  // 引擎通过 registerConfigPatchCallback 注册回调函数。
  // 这些回调会在每次 stepImp 开始时自动收集，合成为运行时配置补丁。
  //
  // 【核心模式】引擎无需在循环中手动调用 ApplyRuntimeConfig，
  // 只需注册回调，stepImp 会自动处理收集→应用流程。
  std::cout << "\n[4/6] 注册运行期配置回调（订阅者模式）...\n";

  SimulatedConfigManager config_mgr;

  // 主回调：通过闭包捕获当前周期号，驱动模拟配置管理器
  sar.registerConfigPatchCallback(
      [&sar, &config_mgr](sar::config::SarRuntimeConfigPatch& patch) {
        // 使用 sar.cycleCount() + 1 获取即将执行的周期号
        // （stepImp 在收集回调时尚未递增 cycle_index_）
        config_mgr.evaluatePatch(sar.cycleCount() + 1, patch);
      });

  std::cout << "  已注册回调链（SimulatedConfigManager 就绪）\n";

  // ===============================================
  // 步骤 5：主仿真循环
  // ===============================================
  // 每周期调用 stepImp(input) 驱动 SAR 仿真与成像。
  // stepImp 内部自动执行：
  //   1. 收集所有注册回调 → 合成 SarRuntimeConfigPatch
  //   2. 如有有效补丁，调用 TryApplyRuntimeConfig
  //   3. 执行 StepWithResult
  //   4. 缓存结果至 lastResult()
  //   5. 记录产品生命周期事件
  std::cout << "\n[5/6] 主仿真循环（" << kNumCycles << " 周期）...\n\n";

  // 统计量
  std::uint32_t patch_applied_count = 0;
  std::uint32_t peak_cycle = 0;
  double peak_magnitude = 0.0;

  for (std::uint32_t i = 0; i < kNumCycles; ++i) {
    // ---- 构造 SarCycleInput ----
    sar::session::SarCycleInput input = MakeCycleInput(i + 1, mission_config);

    // ---- 步进 ----
    config_mgr.clearLastPatchFlag();
    sar.stepImp(input);

    // 检查是否本次触发了配置变更
    if (config_mgr.lastPatchApplied()) {
      ++patch_applied_count;
      std::cout << "  第 " << (i + 1) << " 周期："
                << "-> 配置变更已通过回调注入 stepImp\n";
    }

    // ---- 打印周期摘要 ----
    const auto& result = sar.lastResult();
    std::cout << "  【周期 " << (i + 1) << "/" << kNumCycles << "】"
              << " stage=" << StageName(result.output_frame.completed_stage)
              << " exec=" << (result.executed_this_cycle ? "Y" : "N")
              << " err=" << (result.has_error ? "Y" : "N");

    if (result.output_frame.estimated_snr_db > -1e6) {
      std::cout << " snr_db=" << result.output_frame.estimated_snr_db;
    }

    if (result.has_error) {
      std::cout << " abort=" << result.abort_reason;
    }
    std::cout << "\n";

    // 记录聚焦图像峰值
    if (result.focused_image.source !=
        sar::session::SarFocusedImageSource::kNone) {
      double cycle_peak = 0.0;
      for (std::size_t s = 0;
           s < result.focused_image.real_values.size(); ++s) {
        const double power =
            result.focused_image.real_values[s] *
                result.focused_image.real_values[s] +
            result.focused_image.imaginary_values[s] *
                result.focused_image.imaginary_values[s];
        if (power > cycle_peak) {
          cycle_peak = power;
        }
      }
      if (cycle_peak > peak_magnitude) {
        peak_magnitude = cycle_peak;
        peak_cycle = i + 1;
      }
      std::cout << "    focused_image["
                << result.focused_image.row_count << "x"
                << result.focused_image.column_count
                << "] peak_power=" << cycle_peak << "\n";
    }

    // 打印诊断信息
    for (const auto& issue : result.diagnostics) {
      std::cout << "    [diag] " << issue.code << ": " << issue.message << "\n";
    }
  }

  // ===============================================
  // 步骤 6：仿真结果汇总与三视图展示
  // ===============================================
  std::cout << "\n[6/6] 仿真结果汇总\n"
            << "  总周期数: " << kNumCycles << "\n"
            << "  配置变更注入次数: " << patch_applied_count << "\n"
            << "  当前周期号: " << sar.cycleCount() << "\n"
            << "  成像峰值（最大功率）: " << peak_magnitude
            << " @ 周期 " << peak_cycle << "\n\n";

  // ===============================================
  // 输出三视图展示
  // ===============================================

  // --- 视图一：SarCycleResult（系统输出）---
  const auto& final_result = sar.lastResult();
  std::cout << "====== SAR 输出三视图 ======\n\n"
            << "[视图一] SarCycleResult — 系统侧成像输出\n"
            << "  输入周期: " << final_result.input_cycle_index << "\n"
            << "  完成阶段: "
            << StageName(final_result.output_frame.completed_stage) << "\n"
            << "  原始回波: "
            << (final_result.output_frame.has_raw_echo ? "Y" : "N") << "\n"
            << "  距离压缩: "
            << (final_result.output_frame.has_range_compressed_echo ? "Y" : "N")
            << "\n"
            << "  L1 RDA 图像: "
            << (final_result.output_frame.has_l1_image ? "Y" : "N") << "\n"
            << "  L3 BP 图像: "
            << (final_result.output_frame.has_l3_bp_image ? "Y" : "N") << "\n"
            << "  聚焦图像: "
            << final_result.focused_image.row_count << "x"
            << final_result.focused_image.column_count
            << " source=";

  switch (final_result.focused_image.source) {
    case sar::session::SarFocusedImageSource::kL1Rda:
      std::cout << "L1Rda"; break;
    case sar::session::SarFocusedImageSource::kL3Bp:
      std::cout << "L3Bp"; break;
    default:
      std::cout << "None"; break;
  }
  std::cout << "\n  估计 SNR: "
            << final_result.output_frame.estimated_snr_db << " dB\n"
            << "  诊断数: " << final_result.diagnostics.size() << "\n"
            << "  执行: " << (final_result.executed_this_cycle ? "Y" : "N")
            << " 错误: " << (final_result.has_error ? "Y" : "N") << "\n";
  if (final_result.has_error) {
    std::cout << "  中止原因: " << final_result.abort_reason << "\n";
  }

  // 若有聚焦图像，打印峰值信息
  if (final_result.focused_image.source !=
      sar::session::SarFocusedImageSource::kNone) {
    std::size_t peak_idx = 0;
    double pk_power = -1.0;
    for (std::size_t s = 0;
         s < final_result.focused_image.real_values.size(); ++s) {
      const double p = final_result.focused_image.real_values[s] *
                           final_result.focused_image.real_values[s] +
                       final_result.focused_image.imaginary_values[s] *
                           final_result.focused_image.imaginary_values[s];
      if (p > pk_power) {
        pk_power = p;
        peak_idx = s;
      }
    }
    const std::size_t peak_row =
        peak_idx / final_result.focused_image.column_count;
    const std::size_t peak_col =
        peak_idx % final_result.focused_image.column_count;
    const double range_bin_spacing_m =
        kSpeedOfLightMps / (2.0 * kSampleRateHz);
    const double observed_slant_range_m =
        static_cast<double>(peak_col) * range_bin_spacing_m;
    std::cout << "  峰值像素: row=" << peak_row << " col=" << peak_col
              << " power=" << pk_power
              << " slant_range=" << observed_slant_range_m << " m\n";
  }
  std::cout << "\n";

  // --- 视图二：SarProductDebugView（调试视图）---
  sar::session::SarProductDebugView debug_view = sar.buildLastDebugView();
  std::cout << "[视图二] SarProductDebugView — 人读排查视图\n"
            << "  输入周期: " << debug_view.input_cycle_index << "\n"
            << "  输出周期: " << debug_view.output_cycle_index << "\n"
            << "  执行: " << (debug_view.executed_this_cycle ? "Y" : "N")
            << " 错误: " << (debug_view.has_error ? "Y" : "N") << "\n"
            << "  完成阶段: " << StageName(debug_view.completed_stage) << "\n"
            << "  原始回波: " << (debug_view.has_raw_echo ? "Y" : "N") << "\n"
            << "  距离压缩: " << (debug_view.has_range_compressed_echo ? "Y" : "N")
            << "\n"
            << "  L1 RDA: " << (debug_view.has_l1_image ? "Y" : "N") << "\n"
            << "  L3 BP: " << (debug_view.has_l3_bp_image ? "Y" : "N") << "\n"
            << "  聚焦像素: " << (debug_view.has_focused_pixels ? "Y" : "N")
            << "\n"
            << "  估计 SNR: " << debug_view.estimated_snr_db << " dB\n"
            << "  距离样本: " << debug_view.range_sample_count << "\n"
            << "  方位脉冲: " << debug_view.azimuth_pulse_count << "\n"
            << "  点目标数: " << debug_view.point_targets.size() << "\n";
  for (std::size_t k = 0; k < debug_view.point_targets.size(); ++k) {
    const auto& pt = debug_view.point_targets[k];
    std::cout << "    [" << k << "] id=" << pt.target_id
              << " name=" << pt.target_name
              << " rcs_dbsm=" << pt.radar_cross_section_dbsm << "\n";
  }
  for (const auto& issue : debug_view.diagnostics) {
    std::cout << "    [diag] " << issue.code << ": " << issue.message << "\n";
  }
  std::cout << "\n";

  // --- 视图三：SarProductLifecycleRecorder（生命周期事件）---
  const auto& events = sar.lifecycleEvents();
  std::cout << "[视图三] SarProductLifecycleRecorder — 产品生命周期事件\n"
            << "  总事件数: " << events.size() << "\n";
  for (std::size_t k = 0; k < events.size(); ++k) {
    const auto& e = events[k];
    const char* reason_str = "";
    switch (e.reason) {
      case sar::session::SarProductLifecycleReason::kNone:
        reason_str = "None";
        break;
      case sar::session::SarProductLifecycleReason::kNoImageProduct:
        reason_str = "NoImageProduct";
        break;
      case sar::session::SarProductLifecycleReason::kAbortReason:
        reason_str = "AbortReason";
        break;
      case sar::session::SarProductLifecycleReason::kError:
        reason_str = "Error";
        break;
    }
    std::cout << "    [" << k << "] cycle=" << e.cycle_index
              << " kind=" << LifecycleEventKindName(e.kind)
              << " reason=" << reason_str
              << " stage=" << StageName(e.completed_stage)
              << " snr_db=" << e.estimated_snr_db;
    if (!e.abort_reason.empty()) {
      std::cout << " abort=" << e.abort_reason;
    }
    std::cout << "\n";
  }
  std::cout << "\n";

  // ===============================================
  // Replay API 演示
  // ===============================================
  std::cout << "====== Replay API ======\n"
            << "  enableTrace() — 在 initialize 前调用以开启 trace 录制：\n"
            << "    sar.enableTrace(\"/tmp/sar_trace\");\n"
            << "  replayTrace() — 事后回放已录制的 trace：\n"
            << "    auto replay_result = SarModule::replayTrace(\"/tmp/sar_trace\");\n"
            << "    replay_result.ok = ...\n"
            << "    replay_result.report.total_events = ...\n\n";

  // ===============================================
  // 清理临时文件
  // ===============================================
  std::remove(kTempConfigPath);

  std::cout << "=== Demo 完成 ===\n";
  return 0;
}
