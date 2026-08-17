/**
 * @file inference_component.cpp
 * @brief 目标推演组件实现（融合运动学估计 → 轨迹/发射点/类型推演）。
 *
 * 1. 读同实体 FusionComponent 本周期融合态势，筛选携带运动学估计的目标；
 * 2. LLA 估计 → ECEF 状态帧（协方差透传），一次 TargetInferenceEngine::Infer；
 * 3. 每目标视图摘要行直写集成端日志（发射点/落点/σ/类型概率——误差预算
 *    是产品语义，视图行必须携带，不打印裸点估计）。
 */

#include "inference_component.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "core/world.h"
#include "fusion_component.h"
#include "logger/logger.h"

namespace component_attachment {

InferenceComponent::InferenceComponent(const target_inference::TargetInferenceConfig& config)
    : engine_(config) {}

void InferenceComponent::Step(World& world, double dt_sec) {
  (void)dt_sec;
  if (host_ == nullptr) {
    return;  // 未挂载：无推演
  }
  const auto* fusion = host_->Find<FusionComponent>();
  if (fusion == nullptr) {
    return;  // 无融合组件：无推演输入
  }

  std::vector<target_inference::InferenceTrackState> tracks;
  for (const auto& target : fusion->targets()) {
    if (!target.has_kinematic_estimate) {
      continue;  // 无运动学估计（滤波未启用/未起始/仅方位无原点）：非推演输入
    }
    target_inference::InferenceTrackState track;
    track.key = target.key;
    if (!oneq::coordinate::TryLlaToEcef(target.kinematic_estimate.position, &track.position)) {
      continue;  // 坐标回写异常：跳过该目标（视图计数呈现）
    }
    track.velocity_ecef_m_per_s = target.kinematic_estimate.velocity_ecef_m_per_s;
    track.covariance_ecef = target.kinematic_estimate.covariance_ecef;
    track.has_covariance = true;
    tracks.push_back(track);
  }

  results_ = engine_.Infer(tracks);

  for (const auto& result : results_) {
    const auto& trajectory = result.trajectory;
    const std::string launch = trajectory.has_launch
                                   ? spdlog::fmt_lib::format(
                                         "发射=({:.3f},{:.3f}) t={:.0f}s σ={:.0f}m",
                                         trajectory.launch_point.latitude_deg,
                                         trajectory.launch_point.longitude_deg,
                                         trajectory.launch_time_offset_sec,
                                         trajectory.launch_position_sigma_m)
                                   : std::string("发射=未解算");
    const std::string impact = trajectory.has_impact
                                   ? spdlog::fmt_lib::format(
                                         "落点=({:.3f},{:.3f}) t=+{:.0f}s σ={:.0f}m",
                                         trajectory.impact_point.latitude_deg,
                                         trajectory.impact_point.longitude_deg,
                                         trajectory.impact_time_offset_sec,
                                         trajectory.impact_position_sigma_m)
                                   : std::string("落点=时域外");
    CA_LOG_VIEW("inference", "键={} 类型={} p={:.2f} {} {}",
                static_cast<unsigned long long>(result.key),
                static_cast<int>(result.type.category), result.type.probability,
                launch.c_str(), impact.c_str());
  }
  if (tracks.empty()) {
    CA_LOG_VIEW("inference", "本周期无运动学估计目标（跳过推演）");
  }
}

}  // namespace component_attachment
