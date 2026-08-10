/**
 * @file scene_data.h
 * @brief 自定义实体-组件示例：场景描述（scenes 目录的 JSON 场景文件 → SceneData）。
 *
 * 场景描述文件是"消费方世界模型 + 业务调参"的数据化载体：目标脚本、
 * 平台飞行脚本（原点/航向/巡航/航点或区域巡逻 coverage 块）、ESR 辐射源
 * 波形、天基平台位置、EOS 扫描与 SAR 任务几何/链路覆写、融合配置与冒烟
 * 断言下限。场景文件加载（LoadSceneData）遵循 examples/common/config_loaders
 * 惯例：缺省字段静默默认（成员初始化值 = 默认值，与历史代码覆写值一致）、
 * JSON 语法错误与缺必需块/几何字段经 error 字符串报出。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_SCENE_DATA_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_SCENE_DATA_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/fusion/FusionConfig.h"
#include "1q/navigation/CoverageArea.h"
#include "1q/navigation/CoveragePlanConfig.h"
#include "1q/navigation/RoutePoint.h"

namespace component_attachment {
namespace demo {

/// 目标变速机动（场景文件 targets[].maneuvers[] 条目）：分段匀速语义——
/// 目标在 start_cycle 及之后使用新速度（绝对速度，非增量），直到下一条机动
/// 生效。供"目标大机动/逃逸"类场景验证 AR 失跟/重捕、SAR 几何破坏等。
struct TargetManeuver {
  std::uint32_t start_cycle{0U}; /**< 生效周期（含；相对周期 1 起） */
  double v_east_mps{0.0};        /**< 该周期起的新东向速度（m/s） */
  double v_north_mps{0.0};       /**< 该周期起的新北向速度（m/s） */
};

/// 区域巡逻任务（场景文件可选 coverage 块）：覆盖区域 + 规划参数。
/// 加载时经 navigation::AreaCoveragePlanner 生成巡逻航路（填入 waypoints），
/// 平台按航路循环巡逻（见 FlightComponent::loop_route）。与显式
/// platform.waypoints 互斥（同时出现报错，避免航路来源歧义）。
struct CoverageTask {
  navigation::CoverageArea area{};                 /**< 覆盖区域（多边形 / 圆形） */
  navigation::CoveragePlanConfig config{};         /**< 覆盖规划参数 */
  bool planned{false};                             /**< 已由规划器生成航路（= 巡逻场景） */
};

/// 目标脚本（场景文件 targets[] 条目）：四通道共享同一物理目标
/// （方位/距离/高度 → ECEF 位置；东/北速度 → ECEF 速度；外观/RCS/辐射源
/// 中心频率随真值流转到各通道转换函数，转换函数不再按数组下标回查脚本）。
struct ScriptedTarget {
  std::uint32_t id{0U};                     /**< 外部目标标识（AR/ESR/EOS/SBIRS/SAR 共用） */
  double azimuth_deg{0.0};                  /**< 真方位（北偏东，deg） */
  double range_m{0.0};                      /**< 斜距（m） */
  double altitude_m{0.0};                   /**< 目标高度（m；与平台巡航高度解耦） */
  double v_east_mps{0.0};                   /**< 局部东向速度（m/s） */
  double v_north_mps{0.0};                  /**< 局部北向速度（m/s） */
  double temperature_k{0.0};                /**< 等效温度（EOS/SBIRS 外观） */
  double rcs{0.0};                          /**< 雷达截面积（m²；SAR dBsm 换算源） */
  double projected_area_m2{0.0};            /**< 等效投影面积（EOS/SBIRS 外观，m²） */
  double emitter_center_frequency_hz{0.0};  /**< ESR 辐射源中心频率（Hz；≤0 = 该目标不配辐射源） */
  std::vector<TargetManeuver> maneuvers{};  /**< 变速机动表（可选；start_cycle 严格递增） */
};

/// ESR 辐射源共享波形参数（场景文件 esr 块；中心频率在目标条目内）。
/// 默认值 = 历史基线：200 脉冲 @ 10 GHz 级，单周期积分脉冲数足够，
/// 统计检测概率趋近 1（pfa=1e-6 下多脉冲积分过检）。
struct EsrEmitterParams {
  double peak_gain_dbi{30.0};        /**< 天线峰值增益（dBi） */
  double bandwidth_hz{2.0e6};        /**< 带宽（Hz） */
  double peak_power_w{5.0e7};        /**< 峰值功率（W） */
  double pulse_width_s{1.0e-6};      /**< 脉宽（s） */
  double pri_s{1.0e-3};              /**< 脉冲重复间隔（s） */
  std::uint32_t pulse_count{200U};   /**< 单周期积分脉冲数 */
  std::uint32_t timing_seed{42U};    /**< 波形时序种子（确定性复现） */
};

/// 场景描述：scenes/*.json 的全部业务数据。缺省字段 = 成员初始化值
/// （数值与 demo_config 历史常量一致：kNumCycles/kDtSec 移入本结构后，
/// 周期数/步长按场景文件控制）；平台与目标几何字段为必填（loader 校验）。
struct SceneData {
  std::string name{"unnamed_scene"};        /**< 场景名（摘要打印） */
  std::uint32_t cycles{400U};               /**< 周期数（缺省 = demo_config kNumCycles） */
  double dt_sec{1.0};                       /**< 步长（s，缺省 = demo_config kDtSec） */

  oneq::coordinate::LlaPositionDegM platform_origin{}; /**< 机场位置（度制 LLA；必填） */
  double initial_heading_deg{90.0};         /**< 起飞航向（deg，北偏东） */
  double cruise_altitude_m{400.0};          /**< 巡航高度（m；c172x 低空巡航量级，原 kCruiseAltitudeM） */
  double cruise_speed_mps{50.0};            /**< 巡航速度参考（m/s，原 kCruiseSpeedMps） */
  std::vector<navigation::RoutePoint> waypoints{}; /**< 巡航/巡逻航路（缺省空 = 直飞；
                                              coverage 块存在时为规划器输出的巡逻航路） */
  CoverageTask coverage{};                  /**< 区域巡逻任务（可选；planned=true 时
                                              waypoints 为规划结果，平台循环巡逻） */

  std::vector<ScriptedTarget> targets{};    /**< 目标脚本（四通道共享；几何字段必填） */
  EsrEmitterParams esr{};                   /**< ESR 辐射源波形参数 */

  double sbirs_satellite_altitude_m{500000.0}; /**< 天基平台高度（m；凝视目标群质心正上方） */

  // EOS 业务覆写（原 demo_config 内硬编码，迁入场景数据）：跨会话时间对齐
  // 与视场适配——周期校验要求 dt ≤ 10/frame_rate_hz（10 Hz 对应 1 s 步长
  // 上限）；原 JSON 为下视地面监视（视轴下俯 45°），与空中目标场景不匹配
  // → 覆写为水平扫描（默认扇区 50°~130°，覆盖平台正北目标）。
  float eos_frame_rate_hz{10.0f};           /**< EOS 帧率（Hz） */
  float eos_scan_rate_deg_per_sec{20.0f};   /**< EOS 扫描速率（deg/s） */
  float eos_scan_start_az_deg{50.0f};       /**< EOS 扫描扇区起点（deg，平台局部系 az 0 = 东） */
  float eos_scan_end_az_deg{130.0f};        /**< EOS 扫描扇区终点（deg） */
  float eos_scan_center_el_deg{0.0f};       /**< EOS 扫描中心俯仰（deg，0 = 水平） */
  float eos_boresight_depression_deg{0.0f}; /**< EOS 视轴下俯（deg，0 = 不俯视） */

  // SAR 任务几何/链路覆写（原 demo_config 内硬编码，迁入场景数据）：
  // sar.json 为 100 km 斜距 / 180 m/s 的远程监视档，演示场景需覆写为
  // 低空巡航几何；目标 RCS 仅 2.2/1.4 m²，10 kW 峰值功率下链路 SNR ≈ −29 dB
  // → 功率提升至 1 MW、天线增益 40 dBi（SAR 常用量级），SNR ≈ +10 dB 过门限。
  double sar_peak_power_w{1.0e6};           /**< SAR 峰值功率（W） */
  double sar_antenna_gain_db{40.0};         /**< SAR 天线增益（dBi） */
  double sar_max_squint_angle_deg{10.0};    /**< SAR squint 门限（deg，覆写自 sar.json 的 5°） */
  double sar_scene_center_latitude_deg{30.0 + 13.0e3 / 111.0e3}; /**< SAR 场景中心纬度（deg） */
  double sar_scene_center_longitude_deg{120.06}; /**< SAR 场景中心经度（deg） */
  double sar_scene_center_altitude_m{400.0}; /**< SAR 场景中心高度（m） */
  double sar_nominal_slant_range_m{13000.0}; /**< SAR 标称斜距（m） */
  double sar_platform_speed_mps{50.0};      /**< SAR 平台速度（m/s） */

  fusion::FusionConfig fusion{};            /**< 融合配置（缺省 = FusionConfig 默认值） */
  double high_threat_confidence{3.0};       /**< 决策门限：融合置信度达到该值视为高置信威胁
                                                 （示例业务策略，原 demo_config kHighThreatConfidence） */

  /// 冒烟断言下限（场景文件 smoke 块；"无目标"等零产出场景显式置 0）。
  struct SmokeExpectations {
    std::uint32_t min_key_events{1U};       /**< 关键事件数下限 */
    std::uint32_t min_sbirs_events{1U};     /**< SBIRS 关键探测事件数下限 */
    std::uint32_t min_sar_products{1U};     /**< SAR 关键产品事件数下限 */
    std::uint32_t min_fused_targets{1U};    /**< 融合目标数（峰值）下限 */
  };
  SmokeExpectations smoke{};
};

/// 场景文件 → SceneData。返回 false 并置 error（JSON 语法错误/根节点非对象/
/// 缺必需块（platform/targets）/几何字段缺失）；可缺省字段按成员默认值填充。
bool LoadSceneData(const char* path, SceneData* scene, std::string* error);

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_SCENE_DATA_H_
