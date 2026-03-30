/**
 * @file RadarOrientationConfig.h
 * @brief 定义机载雷达方向、扫描窗口与波束指向配置。
 * @note “可外部调整”定义：调用方可在不重建 `RadarSession` 的前提下，通过公开 API 直接提交修改。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_ORIENTATION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_ORIENTATION_CONFIG_H_

#include "1q/airborne_radar/config/RadarWorkMode.h"
#include "1q/common/scan_schedule_types.h"

namespace airborne_radar {
namespace common {
namespace config {

/**
 * @brief EulerAnglesDeg 表示欧拉角定义（单位：度）。
 * 约定:
 * - yaw   表示偏航 / 方位角
 * - pitch 表示俯仰角
 * - roll  表示滚转角
 * @note 该结构为通用角度载体，本身不绑定“可变/不可变”语义。
 */
struct EulerAnglesDeg {
  float yaw_deg{0.0f};   /**< [通用载体] 偏航 / 方位角（单位：度） */
  float pitch_deg{0.0f}; /**< [通用载体] 俯仰角（单位：度） */
  float roll_deg{0.0f};  /**< [通用载体] 滚转角（单位：度） */
};

/** @brief PlatformAttitudeDeg 表示搭载平台姿态角（单位：度） */
using PlatformAttitudeDeg = EulerAnglesDeg;

/**
 * @brief AzimuthElevationDeg 表示方位-俯仰二维角度（单位：度）。
 * @note 该结构为通用角度载体，本身不绑定“可变/不可变”语义。
 */
struct AzimuthElevationDeg {
  float az_deg{0.0f}; /**< [通用载体] 方位角（单位：度） */
  float el_deg{0.0f}; /**< [通用载体] 俯仰角（单位：度） */
};

/**
 * @brief AzimuthElevationLimitsDeg 表示方位-俯仰扫描限位（单位：度）。
 * @note 该结构通常用于初始化固定边界；运行期若确需修改，建议整体更新 `SignalPipelineConfig`。
 */
struct AzimuthElevationLimitsDeg {
  float az_min_deg{-60.0f}; /**< [初始化固定] 方位最小扫描角（单位：度） */
  float az_max_deg{60.0f};  /**< [初始化固定] 方位最大扫描角（单位：度） */
  float el_min_deg{-30.0f}; /**< [初始化固定] 俯仰最小扫描角（单位：度） */
  float el_max_deg{30.0f};  /**< [初始化固定] 俯仰最大扫描角（单位：度） */
};

/**
 * @brief CommandedBeamwidthDeg 表示指令态波束宽度配置（单位：度）。
 * @note 该配置表示战术控制/ECCM/LPI 等逻辑下发的瞬时波束宽度，
 *       不等同于雷达体制的名义波束宽度。
 * @note 两个成员均为“可外部调整”的运行期控制量。
 */
struct CommandedBeamwidthDeg {
  float commanded_az_beamwidth_deg{4.0f}; /**< [可外部调整] 指令态方位波束宽度（单位：度） */
  float commanded_el_beamwidth_deg{4.0f}; /**< [可外部调整] 指令态俯仰波束宽度（单位：度） */
};

/**
 * @brief StabilizationMode 表示雷达波束稳定方式。
 * @note 该枚举作为 `RadarOrientationConfig::stabilization_mode` 的取值域，
 *       默认属于初始化固定配置。
 */
enum class StabilizationMode {
  kBodyStabilized = 0,     /**< 随机体稳定，波束方向随平台姿态变化 */
  kInertialStabilized = 1, /**< 对惯性空间稳定，尽量保持相对惯性坐标系方向不变 */
  kGroundStabilized = 2    /**< 对地稳定，适用于对地搜索或地形跟随场景 */
};

/**
 * @brief RadarOrientationConfig 表示机载雷达方向与扫描相关配置。
 * 建议组合关系如下:
 * actual_beam_pointing =
 *     platform_attitude + mount_angles_deg + scan_center_deg + dwell_center_deg
 * @note 运行期可外部调整成员：`scan_center_deg`、`work_sub_mode`、`dwell_center_deg`、
 * `commanded_beamwidth_deg`、`commanded_beamwidth_enabled`。
 * 其余成员默认为初始化固定基线。
 */
struct RadarOrientationConfig {
  EulerAnglesDeg mount_angles_deg; /**< [初始化固定] 雷达相对机体坐标系的安装偏置角 */
  AzimuthElevationDeg scan_center_deg; /**< [可外部调整] 搜索窗口中心方向 */
  AzimuthElevationLimitsDeg mechanical_scan_limits_deg; /**< [初始化固定] 机械扫描限位 */
  AzimuthElevationLimitsDeg electronic_scan_limits_deg; /**< [初始化固定] 电子扫描限位 */
  oneq::common::ScanStartPosition scan_start_position{
      oneq::common::ScanStartPosition::kLeftTop}; /**< [初始化固定] 扫描起始象限 */
  oneq::common::ScanSequence scan_sequence{
      oneq::common::ScanSequence::kAzimuthFirst}; /**< [初始化固定] 二维扫描推进顺序 */
  RadarWorkSubMode work_sub_mode{RadarWorkSubMode::kTws}; /**< [可外部调整] 当前工作子模式 */
  AzimuthElevationDeg dwell_center_deg; /**< [可外部调整] 当前波束驻留中心 */
  CommandedBeamwidthDeg commanded_beamwidth_deg; /**< [可外部调整] 当前指令态瞬时波束宽度 */

  /**
   * @brief 是否启用指令态波束宽度覆盖。
   * @note false 时，探测/测量等链路应回退到雷达名义波束宽度。
   */
  bool commanded_beamwidth_enabled{false}; /**< [可外部调整] 指令态波束宽度覆盖使能 */

  StabilizationMode stabilization_mode{StabilizationMode::kBodyStabilized}; /**< [初始化固定] 波束稳定方式 */
};

}  // namespace config
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_ORIENTATION_CONFIG_H_
