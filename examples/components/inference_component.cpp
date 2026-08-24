/**
 * @file inference_component.cpp
 * @brief 目标推演组件实现（融合运动学估计 → 轨迹/发射点/落点/类型推演）。
 *
 * 1. 订阅融合态势事件（on_fusion_updated），逐目标缓存运动学估计展平字段
 *    （融合组件挂载序在前，同周期事件先于本组件 Step 同步到达）；
 * 2. Step 用本周期新鲜缓存重建库输入（LLA 估计 → ECEF 状态帧，协方差
 *    透传），一次 TargetInferenceEngine::Infer——不调用融合组件方法；
 * 3. 每目标视图摘要行直写集成端日志（发射点/落点/σ/类型概率——误差预算
 *    是产品语义，视图行必须携带，不打印裸点估计）。
 */

#include "inference_component.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "core/world.h"
#include "logger/logger.h"

namespace component_attachment {

InferenceComponent::InferenceComponent(const target_inference::TargetInferenceConfig& config)
    : engine_(config) {}

void InferenceComponent::OnFusionUpdated(const FusionUpdatedEvent& event) {
  // 逐事件缓存（事件为集成契约展平字段；周期号供 Step 侧新鲜度过滤——
  // 融合目标消失后不再发事件，陈旧条目按周期过滤即等效剔除）。
  KinematicSnapshot& snapshot = kinematics_by_key_[event.key];
  snapshot.cycle = event.cycle;
  snapshot.has_kinematic_estimate = event.has_kinematic_estimate;
  snapshot.latitude_deg = event.latitude_deg;
  snapshot.longitude_deg = event.longitude_deg;
  snapshot.altitude_m = event.altitude_m;
  snapshot.velocity_ecef_m_per_s = event.velocity_ecef_m_per_s;
  snapshot.covariance_ecef = event.covariance_ecef;
}

void InferenceComponent::Step(World& world, double dt_sec) {
  (void)dt_sec;
  // 事件接线（首次 Step 惰性连接；scoped_connection 随组件析构自动断开）。
  if (!fusion_connection_.connected()) {
    fusion_connection_ = world.signals().on_fusion_updated.connect(
        [this](const FusionUpdatedEvent& event) { OnFusionUpdated(event); });
  }

  // key 升序遍历（与融合态势输出序一致，结果序确定）；仅用本周期新鲜条目。
  std::vector<std::uint64_t> keys;
  keys.reserve(kinematics_by_key_.size());
  for (const auto& entry : kinematics_by_key_) {
    keys.push_back(entry.first);
  }
  std::sort(keys.begin(), keys.end());

  const std::uint64_t cycle = world.scene_state().cycle;
  std::vector<target_inference::InferenceTrackState> tracks;
  for (const std::uint64_t key : keys) {
    const KinematicSnapshot& snapshot = kinematics_by_key_.at(key);
    if (!snapshot.has_kinematic_estimate || snapshot.cycle != cycle) {
      continue;  // 无运动学估计（滤波未启用/未起始/仅方位无原点）或本周期未更新：非推演输入
    }
    target_inference::InferenceTrackState track;
    track.key = key;
    // 采样仿真时间/周期号：验收行（关机点等时间戳量）需要真实值；world 共享
    // 场景状态与各组件事件同源。
    track.sim_time_sec = world.scene_state().t_sec;
    track.input_cycle_index = static_cast<std::uint32_t>(cycle);
    oneq::coordinate::LlaPositionDegM lla;
    lla.latitude_deg = snapshot.latitude_deg;
    lla.longitude_deg = snapshot.longitude_deg;
    lla.altitude_m = snapshot.altitude_m;
    if (!oneq::coordinate::TryLlaToEcef(lla, &track.position)) {
      continue;  // 坐标回写异常：跳过该目标（视图计数呈现）
    }
    track.velocity_ecef_m_per_s = snapshot.velocity_ecef_m_per_s;
    track.covariance_ecef = snapshot.covariance_ecef;
    track.has_covariance = true;
    tracks.push_back(track);
  }

  results_ = engine_.Infer(tracks);

  for (const auto& result : results_) {
    const auto& trajectory = result.trajectory;
    const std::string launch =
        trajectory.has_launch
            ? std::string("发射=(") + std::to_string(trajectory.launch_point.latitude_deg) +
                  "," + std::to_string(trajectory.launch_point.longitude_deg) + "," +
                  std::to_string(trajectory.launch_point.altitude_m) + ") t=" +
                  std::to_string(trajectory.launch_time_offset_sec) + "s σ=" +
                  std::to_string(trajectory.launch_position_sigma_m) + "m"
            : std::string("发射=未解算");
    const std::string impact =
        trajectory.has_impact
            ? std::string("落点=(") + std::to_string(trajectory.impact_point.latitude_deg) +
                  "," + std::to_string(trajectory.impact_point.longitude_deg) + "," +
                  std::to_string(trajectory.impact_point.altitude_m) + ") t=+" +
                  std::to_string(trajectory.impact_time_offset_sec) + "s σ=" +
                  std::to_string(trajectory.impact_position_sigma_m) + "m"
            : std::string("落点=时域外");
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string inference_view_log =
        std::string("键=") +
        std::to_string(static_cast<unsigned long long>(result.key)) +
        " 类型=" +
        std::to_string(static_cast<int>(result.type.category)) +
        " p=" +
        std::to_string(result.type.probability) +
        " " +
        (launch.c_str()) +
        " " +
        (impact.c_str());
    CA_LOG_VIEW("inference", "键={} 类型={} p={:.2f} {} {}",
                static_cast<unsigned long long>(result.key),
                static_cast<int>(result.type.category), result.type.probability,
                launch.c_str(), impact.c_str());
  }
  if (tracks.empty()) {
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string inference_view_log_2 =
        std::string("本周期无运动学估计目标（跳过推演）");
    CA_LOG_VIEW("inference", "本周期无运动学估计目标（跳过推演）");
  }
}

}  // namespace component_attachment
