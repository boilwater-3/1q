/**
 * @file ArOrientationConfig.h
 * @brief 机载雷达方向与扫描相关主配置类型。
 *
 * 姿态、工作模式与扫描调度的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_ORIENTATION_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_ORIENTATION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/foundation/pose_types.h"
#include "1q/foundation/scan_schedule_types.h"

namespace airborne_radar {
namespace config {

/** @brief EulerAnglesDeg 兼容别名：欧拉角姿态（单位：deg）。 */
using EulerAnglesDeg = oneq::foundation::EulerAnglesDeg;

/** @brief PlatformAttitudeDeg 表示搭载平台姿态角（单位：度） */
using PlatformAttitudeDeg = EulerAnglesDeg;

/**
 * @brief AzimuthElevationDeg 表示方位-俯仰二维角度（单位：度）。
 */
struct ONEQ_API AzimuthElevationDeg {
  float az_deg{0.0f}; /**< 方位角（单位：度） */
  float el_deg{0.0f}; /**< 俯仰角（单位：度） */
};

/**
 * @brief AzimuthElevationLimitsDeg 表示方位-俯仰扫描限位（单位：度）。
 * @note 该结构通常用于初始化固定边界；
 */
struct ONEQ_API AzimuthElevationLimitsDeg {
  float az_min_deg{-60.0f}; /**< 方位最小扫描角（单位：度） */
  float az_max_deg{60.0f};  /**< 方位最大扫描角（单位：度） */
  float el_min_deg{-30.0f}; /**< 俯仰最小扫描角（单位：度） */
  float el_max_deg{30.0f};  /**< 俯仰最大扫描角（单位：度） */
};

/**
 * @brief CommandedBeamwidthDeg 表示指令态波束宽度配置（单位：度）。
 * @note 该配置表示战术控制/ECCM/LPI 等逻辑下发的瞬时波束宽度，
 *       不等同于雷达体制的名义波束宽度。
 * @note CommandedBeamwidthDeg 不是静态初始化配置，为运行期可改的控制量；
 *       但它只有在 commanded_beamwidth_enabled == true 时才会真正生效。
 *       该默认值设置意在"指令态波束宽度"和"名义波束宽度"保持同一量级，避免一开局就出现很窄或很宽的异常控制值。
 */
struct ONEQ_API CommandedBeamwidthDeg {
  float commanded_az_beamwidth_deg{
      4.0f}; /**< [可外部调整，但建议在初始化配置后不修改] 指令态方位波束宽度（单位：度） */
  float commanded_el_beamwidth_deg{
      4.0f}; /**< [可外部调整，但建议在初始化配置后不修改] 指令态俯仰波束宽度（单位：度） */
};

/**
 * @brief ArWorkMode 表示机载雷达工作模式。
 * @note 该枚举对应运行期可外部调整控制量。
 * @note "可外部调整"定义：调用方可在不重建 `ArSession` 的前提下，通过公开 API 直接提交修改。
 * @note 命名对齐 EOS/ESR 的 work_mode / EosWorkMode / EsrWorkMode（原名带 Sub 后缀，
 *       已去掉以消除跨域"同概念异名"；由 check_cross_domain_naming.cmake 守护不回归）。
 */
enum class ONEQ_API ArWorkMode {
  kStby = 0, /**< 待机：波束停泊并停止扫描 */
  kTas = 1,  /**< 目标捕获扫描：更高空间采样密度的二维扫描（步进更小） */
  kTws = 2,  /**< 边扫边跟踪：常规二维扫描 */
  kStt = 3   /**< 单目标跟踪：固定于 scan_center 的驻留指向 */
};

/**
 * @brief StabilizationMode 表示雷达波束稳定方式。
 * @note 该枚举作为 `ArOrientationConfig::stabilization_mode` 的取值域，
 *       默认属于初始化固定配置。
 */
enum class ONEQ_API StabilizationMode {
  kBodyStabilized = 0,     /**< 随机体稳定，波束方向随平台姿态变化 */
  kInertialStabilized = 1, /**< 对惯性空间稳定，尽量保持相对惯性坐标系方向不变 */
  kGroundStabilized = 2    /**< 对地稳定；当前无地理参考输入时，代码上等效于惯性稳定 */
};

/**
 * @brief ArOrientationConfig 表示机载雷达方向与扫描相关配置。
 * @note 本结构描述初始化/静态配置项，不包含运行期驻留偏移。
 * @note 静态基准组合关系:
 *       actual_beam_pointing_base = platform_attitude + mount_angles_deg + scan_center_deg
 * @note 运行期若存在驻留偏移（dwell center），由运行态控制链路在上述基准上追加叠加。
 */
struct ONEQ_API ArOrientationConfig {
  /**
   * @brief 雷达安装偏置角（单位：deg，参考系：Body -> Radar）。
   * @note 该角度定义在机体坐标系下，描述雷达坐标系相对机体系的固定安装偏置；
   *       与运行期平台姿态（Body -> ENU）通过旋转矩阵复合后，
   *       可得到"ENU -> Radar"的等效姿态。
   * @note 当前为静态配置。若未来需要支持可动天线座（运行期安装角变化），
   *       可通过 ArRuntimeConfigPatch 扩展此字段的运行期更新能力。
   */
  EulerAnglesDeg mount_angles_deg;
  AzimuthElevationDeg scan_center_deg; /**< [可外部调整] 基准指向方向；不是扫描体积中心 */
  AzimuthElevationLimitsDeg mechanical_scan_limits_deg; /**< 机械扫描限位 */
  AzimuthElevationLimitsDeg electronic_scan_limits_deg; /**< 电子扫描限位 */
  oneq::foundation::ScanStartPosition scan_start_position{
      oneq::foundation::ScanStartPosition::kLeftTop}; /**< 扫描起始象限 */
  oneq::foundation::ScanSequence scan_sequence{
      oneq::foundation::ScanSequence::kAzimuthFirst};         /**< 二维扫描推进顺序 */
  ArWorkMode work_mode{ArWorkMode::kTws}; /**< [可外部调整] 当前工作模式 */
  bool commanded_beamwidth_enabled{false};       /**< 指令态波束宽度覆盖使能 */
  CommandedBeamwidthDeg commanded_beamwidth_deg; /**< 当前指令态瞬时波束宽度 */
  StabilizationMode stabilization_mode{StabilizationMode::kBodyStabilized}; /**< 波束稳定方式 */
};


}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_ORIENTATION_CONFIG_H_
