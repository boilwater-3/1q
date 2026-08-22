/**
 * @file RirSceneTypes.h
 * @brief 远程识别雷达场景实体输入类型集合。
 *
 * 场景目标输入的主头文件。识别真值字段（aspect/polarization/scatterer）随
 * 解耦从 AR `ArSceneTypes.h`（审计基线 96de367c）迁入本模块；阶段 2-S 增加
 * 速度、目标名称与 Swerling 起伏模型；`rcs`（m²）为探测链标量。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SCENE_TYPES_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SCENE_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirSwerlingType 场景目标 Swerling 起伏模型（阶段 2-S D-A7）。
 * @note 取值 0-4 与内部 `RirSwerlingModel` 对齐；公共枚举加性扩展，
 *       不得重排既有值（replay/字节兼容规则）。
 */
enum class ONEQ_API RirSwerlingType : std::uint8_t {
  kSwerling0 = 0, /**< 非起伏（Marcum 平稳目标）。 */
  kSwerling1 = 1, /**< 慢起伏（扫描间 Rayleigh，Chi-2）。 */
  kSwerling2 = 2, /**< 快起伏（脉冲间 Rayleigh，Chi-2）。 */
  kSwerling3 = 3, /**< 慢起伏（一个主加 Rayleigh，Chi-4）。 */
  kSwerling4 = 4  /**< 快起伏（脉冲间 Chi-4）。 */
};

/**
 * @brief RirAspectRcsSample 按入射方位/俯仰角离散的 RCS 真值样本。
 * @note 识别专用特征输入；RCS 以 dBsm 表达，区别于 `RirSceneTarget::rcs`（m²）。
 */
struct ONEQ_API RirAspectRcsSample {
  float aspect_az_deg{0.0f}; /**< 入射方位角（deg），任意有限值。 */
  float aspect_el_deg{0.0f}; /**< 入射俯仰角（deg），任意有限值。 */
  float rcs_dbsm{0.0f};      /**< 该视角下 RCS（dBsm），有限值。 */
};

/**
 * @brief RirPolarizationRcsSample 同一观测几何下极化 RCS 样本。
 * @note channel_1 = HH，channel_2 = VV。交叉极化与 HH–VV 相位仅验收旁路消费；
 *       0 dBsm 合法，必须用 has_* 声明已填，不得把 0 当缺省。
 */
struct ONEQ_API RirPolarizationRcsSample {
  float aspect_az_deg{0.0f};       /**< 入射方位角（deg），任意有限值。 */
  float aspect_el_deg{0.0f};       /**< 入射俯仰角（deg），任意有限值。 */
  float channel_1_rcs_dbsm{0.0f};  /**< HH 通道 RCS（dBsm）。 */
  float channel_2_rcs_dbsm{0.0f};  /**< VV 通道 RCS（dBsm）。 */
  float cross_rcs_dbsm{0.0f};      /**< 交叉极化 HV=VH RCS（dBsm）；has_cross_pol 为假时忽略。 */
  float phase_vv_rel_hh_deg{0.0f}; /**< VV 相对 HH 相位（deg）；has_phase_vv 为假时忽略。 */
  bool has_cross_pol{false};       /**< 已显式提供交叉极化 RCS。 */
  bool has_phase_vv{false};        /**< 已显式提供 HH–VV 相位。 */
};

/**
 * @brief RirRangeRcsScatterer 距离向散射中心样本。
 * @note 一维距离像只使用 `RirSceneTarget::range_rcs_scatterers`，
 *       不允许以单个总体 RCS 代替；`phase_deg == 0` 表示非相干叠加。
 */
struct ONEQ_API RirRangeRcsScatterer {
  float range_offset_m{0.0f};     /**< 相对目标参考点的距离向位置（m）。 */
  float rcs_dbsm{0.0f};           /**< 该散射中心 RCS（dBsm）。 */
  float channel_1_rcs_dbsm{0.0f}; /**< 可选：第一极化通道散射 RCS（dBsm）。 */
  float channel_2_rcs_dbsm{0.0f}; /**< 可选：第二极化通道散射 RCS（dBsm）。 */
  float phase_deg{0.0f};          /**< 可选：相位（deg），0 表示非相干叠加。 */
  float fluctuation_std_db{0.0f}; /**< 可选：周期起伏标准差（dB），0 表示无起伏。 */
};

/**
 * @brief RirSceneTarget 描述识别雷达单周期场景目标输入。
 * @note 阶段 2-S 起场景目标补齐自持检测与滤波所需事实：速度（量测种子与
 *       多普勒/径向速度）、`target_name`（人读与识别真值准确率统计）与
 *       `target_swerling_type`（CFAR 起伏模型）。
 * @note `position_x/y/z` 为雷达局部 ENU 右手坐标系（x=东、y=北、z=天）；
 *       该坐标系也是波束指向角 `config::RirAzimuthElevationDeg` 与目标视线角
 *       的参考系。集成层用户侧以 ECEF 描述目标位置/速度，适配层在调用
 *       `StepWithResult` 前完成 ECEF→ENU 转换后再填入本结构体。
 */
struct ONEQ_API RirSceneTarget {
  std::uint64_t external_target_id{0}; /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  std::string target_name{};           /**< 可选目标名称，仅用于人读与真值准确率统计。 */
  float position_x{0.0f};              /**< 雷达局部 ENU 坐标 x（东向，单位：m） */
  float position_y{0.0f};              /**< 雷达局部 ENU 坐标 y（北向，单位：m） */
  float position_z{0.0f};              /**< 雷达局部 ENU 坐标 z（天向，单位：m） */
  float velocity_x{0.0f};              /**< 目标速度向量 x 分量（单位：m/s）。 */
  float velocity_y{0.0f};              /**< 目标速度向量 y 分量（单位：m/s）。 */
  float velocity_z{0.0f};              /**< 目标速度向量 z 分量（单位：m/s）。 */
  float rcs{0.0f};                     /**< 目标雷达散射截面积（单位：m²，SNR 门控用） */
  float range_m{0.0f};                 /**< 目标到雷达的斜距（单位：m） */
  RirSwerlingType target_swerling_type{
      RirSwerlingType::kSwerling0}; /**< 目标起伏模型（CFAR Pd 输入）。 */

  /** 识别专用特征真值输入（默认空；空向量表示该维度不可用） */
  std::vector<RirAspectRcsSample> aspect_rcs_samples{};
  std::vector<RirPolarizationRcsSample> polarization_rcs_samples{};
  std::vector<RirRangeRcsScatterer> range_rcs_scatterers{};
};

/** @brief RirSceneTargetList 表示识别雷达场景目标输入列表。 */
using RirSceneTargetList = std::vector<RirSceneTarget>;

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SCENE_TYPES_H_
