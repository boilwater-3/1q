#include "airborne_radar/signal/tracking/TrackFilter.h"

#include <algorithm>
#include <cmath>

namespace airborne_radar {
namespace signal {
namespace tracking {

namespace {
/**
 * @brief 计算三维向量欧氏范数。
 * @param x X 分量。
 * @param y Y 分量。
 * @param z Z 分量。
 * @return 三维向量模长。
 */
float VectorNorm3(float x, float y, float z) { return std::sqrt(x * x + y * y + z * z); }
/**
 * @brief 判断三维向量是否包含任一非零分量。
 * @param x X 分量。
 * @param y Y 分量。
 * @param z Z 分量。
 * @return 至少存在一个非零分量时返回 true。
 */
bool HasNonZero3(float x, float y, float z) { return x != 0.0f || y != 0.0f || z != 0.0f; }
/**
 * @brief 将输入向量归一化；若范数过小则使用兜底方向。
 * @param x 输入 X 分量。
 * @param y 输入 Y 分量。
 * @param z 输入 Z 分量。
 * @param nx [out] 归一化后的 X 分量。
 * @param ny [out] 归一化后的 Y 分量。
 * @param nz [out] 归一化后的 Z 分量。
 * @param fallback_x 兜底 X 分量。
 * @param fallback_y 兜底 Y 分量。
 * @param fallback_z 兜底 Z 分量。
 */
void NormalizeOrFallback(float x, float y, float z, float& nx, float& ny, float& nz,
                         float fallback_x, float fallback_y, float fallback_z) {
  const float norm = VectorNorm3(x, y, z);
  if (norm > 1e-6f) {
    nx = x / norm;
    ny = y / norm;
    nz = z / norm;
    return;
  }

  nx = fallback_x;
  ny = fallback_y;
  nz = fallback_z;
}
/**
 * @brief 判断是否属于易导致关联脆弱的干扰语义。
 * @param semantic 当前主导干扰语义。
 * @return 属于 deception/repeater/mixed 时返回 true。
 */
bool IsAssociationFragileJamming(config::JammingSemantic semantic) {
  return semantic == config::JammingSemantic::kDeception ||
         semantic == config::JammingSemantic::kRepeater ||
         semantic == config::JammingSemantic::kMixed;
}

}  // namespace

PredictedTrackState IdentityTrackPredictor::Predict(
    const session::RadarSceneTarget& input) const {
  const bool has_velocity_axis =
      HasNonZero3(input.velocity_x, input.velocity_y, input.velocity_z);
  const float speed =
      has_velocity_axis ? VectorNorm3(input.velocity_x, input.velocity_y, input.velocity_z) : 0.0f;

  const float vx = has_velocity_axis ? input.velocity_x : speed;
  const float vy = has_velocity_axis ? input.velocity_y : 0.0f;
  const float vz = has_velocity_axis ? input.velocity_z : 0.0f;

  return PredictedTrackState{speed, input.rcs, vx, vy, vz};
}

SimpleTrackUpdater::SimpleTrackUpdater(TrackFilterConfig config) : config_(config) {}

session::RadarSceneTarget SimpleTrackUpdater::Update(const PredictedTrackState& predicted,
                                                        const TrackFilterContext& context) const {
  session::RadarSceneTarget output(predicted.velocity_x, predicted.velocity_y,
                                      predicted.velocity_z, predicted.rcs);
  output.velocity_x = predicted.velocity_x;
  output.velocity_y = predicted.velocity_y;
  output.velocity_z = predicted.velocity_z;

  if (!context.detection_succeeded) {
    float speed_decay_ratio = config_.speed_decay_ratio_on_loss;
    float rcs_decay_ratio = config_.rcs_decay_ratio_on_loss;
    if (context.jamming_detected &&
        IsAssociationFragileJamming(context.dominant_jamming_semantic)) {
      const float relief_scale = 0.10f * std::max(0.0f, context.jamming_severity);
      speed_decay_ratio = std::min(0.995f, speed_decay_ratio + relief_scale);
      rcs_decay_ratio = std::min(0.999f, rcs_decay_ratio + 1.2f * relief_scale);
    }
    const float decayed_speed = std::max(0.0f, predicted.speed * speed_decay_ratio);
    output.rcs = std::max(0.05f, predicted.rcs * rcs_decay_ratio);

    float dir_vx = 1.0f;
    float dir_vy = 0.0f;
    float dir_vz = 0.0f;
    NormalizeOrFallback(predicted.velocity_x, predicted.velocity_y, predicted.velocity_z, dir_vx,
                        dir_vy, dir_vz, 1.0f, 0.0f, 0.0f);
    output.velocity_x = dir_vx * decayed_speed;
    output.velocity_y = dir_vy * decayed_speed;
    output.velocity_z = dir_vz * decayed_speed;
  }

  return output;
}

void SimpleTrackUpdater::UpdateConfig(TrackFilterConfig config) { config_ = config; }

TrackFilter::TrackFilter(TrackFilterConfig config) : updater_(config) {}

session::RadarSceneTarget TrackFilter::Filter(const session::RadarSceneTarget& input,
                                                 const TrackFilterContext& context) const {
  const PredictedTrackState predicted = predictor_.Predict(input);
  return updater_.Update(predicted, context);
}

void TrackFilter::UpdateConfig(TrackFilterConfig config) { updater_.UpdateConfig(config); }

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
