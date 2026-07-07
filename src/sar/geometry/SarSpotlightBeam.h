/**
 * @file SarSpotlightBeam.h
 * @brief 聚束(Spotlight)模式时变波束指向序列生成。
 *
 * 聚束 = 平台直线匀速 + 天线反向跟踪场景中心。波束指向与平台轨迹正交分离
 * (契约 spotlight_mode.md §3.1 方案 2),在回波生成处汇合。
 */

#ifndef ONEQ_SRC_SAR_GEOMETRY_SAR_SPOTLIGHT_BEAM_H_
#define ONEQ_SRC_SAR_GEOMETRY_SAR_SPOTLIGHT_BEAM_H_

#include <cstdint>
#include <vector>

#include "sar/geometry/SarGeometry.h"

namespace sar {
namespace geometry {

/**
 * @brief 单脉冲的聚束波束指向状态(与 PlatformPulseState 时间对齐)。
 */
struct SpotlightBeamState {
  double time_s{0.0};
  double boresight_azimuth_rad{0.0};  /**< 该脉冲天线方位指向(rad) */
};

/**
 * @brief 聚束波束指向序列生成配置。
 */
struct SpotlightBeamTrackConfig {
  LocalPoint scene_center_m{};             /**< 聚束场景中心(波束反向跟踪目标) */
  std::vector<double> pulse_times_s{};     /**< 与平台脉冲序列对齐的慢时间 */
};

/**
 * @brief 逐脉冲计算波束指向:boresight_azimuth(t) = atan2(cx − px, cy − py)。
 *
 * 波束始终指向场景中心,实现聚束的持续照射。退化解:scene_center 在平台正侧方
 * 且平台直线飞行时,所有 boresight 近似相同(等价固定 broadside)。
 *
 * @param platform_pulses 平台脉冲序列(提供位置)。
 * @param beam_states 输出波束指向序列(与 platform_pulses 等长,时间对齐)。
 * @return 成功则 true;长度不匹配或指针空则 false。
 */
bool GenerateSpotlightBeamTrack(const SpotlightBeamTrackConfig& config,
                                const std::vector<PlatformPulseState>& platform_pulses,
                                std::vector<SpotlightBeamState>* beam_states);

/**
 * @brief 聚束合成孔径时间(由波束跟踪角决定,非条带公式)。
 *
 * 聚束合成孔径由天线转过的角度 θ_synth 决定:T_synth = θ_synth · R_center / v。
 * θ_synth 由波束指向序列的首尾指向角差给出。
 *
 * @param beam_states 波束指向序列。
 * @param slant_range_m 场景中心斜距。
 * @param platform_velocity_mps 平台速度。
 * @return 合成孔径时间(s);序列不足两点或速度非正则返回 0。
 */
double SpotlightSyntheticApertureTime(const std::vector<SpotlightBeamState>& beam_states,
                                      double slant_range_m, double platform_velocity_mps);

/**
 * @brief 聚束轨迹组合配置(平台直线匀速 + 天线反向跟踪场景中心)。
 */
struct SpotlightTrackConfig {
  StraightStripmapTrackConfig platform_track{};  /**< 平台直线匀速轨迹 */
  LocalPoint scene_center_m{};                   /**< 聚束场景中心 */
};

/**
 * @brief 生成聚束场景的平台轨迹 + 波束指向序列。
 *
 * 内部:GenerateStraightStripmapTrack 产平台脉冲 → GenerateSpotlightBeamTrack 产波束序列。
 * 两个序列时间对齐、等长。
 *
 * @return 成功则 true;平台轨迹或波束生成失败则 false。
 */
bool GenerateSpotlightTrack(const SpotlightTrackConfig& config,
                            std::vector<PlatformPulseState>* platform_pulses,
                            std::vector<SpotlightBeamState>* beam_states);

}  // namespace geometry
}  // namespace sar

#endif  // ONEQ_SRC_SAR_GEOMETRY_SAR_SPOTLIGHT_BEAM_H_
