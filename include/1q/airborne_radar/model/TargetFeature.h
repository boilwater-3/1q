/**
 * @file TargetFeature.h
 * @brief 定义表示当前雷达处理周期内目标的核心状态数据结构。
 */

#ifndef AIRBORNE_RADAR_COMMON_TARGET_FEATURE_H_
#define AIRBORNE_RADAR_COMMON_TARGET_FEATURE_H_

#include <cmath>
#include <cstdint>
#include <vector>

namespace airborne_radar {
namespace model {

/**
 * @brief TargetFeature 封装了单个处理周期的目标输入状态。
 * 它作为只读的有效载荷在行为决策层管线中传递。
 *
 * @note
 * 输入必要性说明：
 * - <b>position_x/y/z</b>：必须提供。雷达局部笛卡尔坐标系下的目标位置（单位：m），
 *   用于 Kalman 滤波更新和轨迹初始化。若未提供，轨迹关联和滤波将无法正常工作。
 * - <b>current_track_velocity_x/y/z</b>：强烈建议提供。目标速度向量（单位：m/s），
 *   坐标系与 `position_x/y/z` 完全一致，均为雷达局部笛卡尔坐标分量。
 *   初始化 Kalman 状态时直接填入状态向量的速度分量。若不提供（默认为零），
 *   轨迹将退化为纯位置跟踪，第一个预测周期内位置不会外推，
 *   且多维运动的速度估算精度会严重下降。
 *   若外部系统提供的是 ECEF 速度，请先转换到雷达局部坐标后再写入该字段。
 * - <b>current_track_rcs</b>：可选。雷达散射截面积（单位：m^2），默认 0。
 *
 * @note
 * 各字段默认值：位置为 0（会被 `NormalizeTargetGeometry` 覆盖），速度为 0 向量，
 * RCS 为 0，Swerling 起伏模型为 0（无起伏）。
 */
struct TargetFeature {
  std::uint64_t external_target_id{0}; /**< 外部输入原始目标标识符（0 表示未知/未提供） */

  float current_track_velocity_x{0.0f}; /**< 雷达局部坐标系速度 x 分量（单位：m/s） */
  float current_track_velocity_y{0.0f}; /**< 雷达局部坐标系速度 y 分量（单位：m/s） */
  float current_track_velocity_z{0.0f}; /**< 雷达局部坐标系速度 z 分量（单位：m/s） */

  /**
   * @brief 当前被跟踪主要目标速度模长（单位：m/s）。
   * @note 由构造函数根据速度向量自动计算赋值，不作为输入参数。
   */
  float current_track_speed{0.0f};

  float current_track_rcs{0.0f}; /**< 目标的估计雷达散射截面积（RCS）（单位：平方米） */

  float range_m{0.0f}; /**< 目标到雷达的斜距（单位：m） */

  /**
   * @brief 标记该目标是否携带有效的笛卡尔位置量测。
   * @note 必须显式置 true 才表明 position_x/y/z 中携带真实坐标。
   *       仅当此标志为 true 时，position_x/y/z 才参与关联及滤波。
   */
  bool has_cartesian_position{false};

  /**
   * @brief 雷达局部笛卡尔坐标位置（单位：m），[x, y, z]。
   * @note 该坐标系以当前雷达为原点，仅接受雷达局部坐标输入；
   *       速度字段 `current_track_velocity_*` 与本坐标轴定义一致。
   *       注：不使用 Eigen 以保持 POD 布局，Pipeline 内部按需转换。
   */
  float position_x{0.0f};
  float position_y{0.0f};
  float position_z{0.0f};

  /**
   * @brief 目标的 Swerling 起伏模型，默认 0 (kSwerling0，无起伏)
   *        保持整数类型以避免在公开头文件中引入底层检测枚举依赖
   */
  int target_swerling_type{0};

  TargetFeature() = default; /**< 默认构造函数 */

  /**
   * @brief 带参数的构造函数，为了方便初始化。
   * @note 速度模长由 [vx,vy,vz] 自动计算，调用方只输入向量。
   */
  TargetFeature(float velocity_x, float velocity_y, float velocity_z, float rcs,
                float range = 0.0f, int swerling_type = 0,
                std::uint64_t ext_target_id = 0)
      : external_target_id(ext_target_id),
        current_track_velocity_x(velocity_x),
        current_track_velocity_y(velocity_y),
        current_track_velocity_z(velocity_z),
        current_track_speed(
            std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y + velocity_z * velocity_z)),
        current_track_rcs(rcs),
        range_m(range),
        target_swerling_type(swerling_type) {}
};

/** @brief TargetFeatureList 是 TargetFeature 的列表，表示当前处理周期内所有相关目标的特征集合 */
using TargetFeatureList = std::vector<TargetFeature>;

}  // namespace model
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_TARGET_FEATURE_H_
