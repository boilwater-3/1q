/**
 * @file TrackFilter.h
 * @brief 简单的基于预测/更新的轨迹滤波器抽象。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_FILTER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_FILTER_H_

#include "1q/airborne_radar/common/JammingSemantics.h"
#include "1q/airborne_radar/common/TargetFeature.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/// @brief 轨迹滤波器配置参数。
struct TrackFilterConfig {
  /// @brief 失配时的速度衰减系数。
  float speed_decay_ratio_on_loss{0.90f};
  /// @brief 失配时的 RCS 衰减系数。
  float rcs_decay_ratio_on_loss{0.85f};
  /// @brief 干扰时的加速度惩罚系数。
  float jamming_acceleration_penalty{0.5f};
  /// @brief 稳定状态下的加速度增益。
  float stable_acceleration_gain{0.05f};
};

/// @brief 轨迹滤波器处理上下文。
struct TrackFilterContext {
  /// @brief 本周期是否检测成功。
  bool detection_succeeded{false};
  /// @brief 是否探测到干扰。
  bool jamming_detected{false};
  /// @brief 主要干扰语义类型。
  common::JammingSemantic dominant_jamming_semantic{
      common::JammingSemantic::kNone};
  /// @brief 干扰严重程度。
  float jamming_severity{0.0f};
  /// @brief 检测余量（dB）。
  float detection_margin_db{0.0f};
};

/// @brief 预测后的轨迹状态。
struct PredictedTrackState {
  PredictedTrackState() = default;
  /// @brief 构造函数。
  PredictedTrackState(float s, float r, float a,
                      float vx, float vy, float vz,
                      float ax, float ay, float az)
      : speed(s), rcs(r), acceleration(a),
        velocity_x(vx), velocity_y(vy), velocity_z(vz),
        acceleration_x(ax), acceleration_y(ay), acceleration_z(az) {}

  float speed{0.0f};           ///< 速度大小
  float rcs{0.0f};             ///< 雷达散射截面积
  float acceleration{0.0f};    ///< 加速度大小
  float velocity_x{0.0f};      ///< X轴速度
  float velocity_y{0.0f};      ///< Y轴速度
  float velocity_z{0.0f};      ///< Z轴速度
  float acceleration_x{0.0f};  ///< X轴加速度
  float acceleration_y{0.0f};  ///< Y轴加速度
  float acceleration_z{0.0f};  ///< Z轴加速度
};

/// @brief 轨迹预测器抽象接口。
class ITrackPredictor {
 public:
  virtual ~ITrackPredictor() = default;

  /// @brief 执行轨迹预测。
  /// @param input 输入目标特征。
  /// @return 预测后的轨迹状态。
  virtual PredictedTrackState Predict(
      const common::TargetFeature &input) const = 0;
};

/// @brief 轻量轨迹预测器。
/// @details 默认保持输入语义；当轴向速度/加速度分量缺失时，
///          会使用标量速度/加速度回填 X 轴并将其余轴置零。
class IdentityTrackPredictor final : public ITrackPredictor {
 public:
  /// @brief 执行恒等预测。
  /// @param input 输入目标特征。
  /// @return 预测后的轨迹状态。
  PredictedTrackState Predict(const common::TargetFeature &input) const override;
};

/// @brief 轨迹更新器抽象接口。
class ITrackUpdater {
 public:
  virtual ~ITrackUpdater() = default;

  /// @brief 执行轨迹更新。
  /// @param predicted 预测后的状态。
  /// @param context 处理上下文。
  /// @return 更新后的目标特征。
  virtual common::TargetFeature Update(
      const PredictedTrackState &predicted,
      const TrackFilterContext &context) const = 0;
};

/// @brief 简单轨迹更新器实现。
class SimpleTrackUpdater final : public ITrackUpdater {
 public:
  /// @brief 构造函数。
  /// @param config 滤波器配置。
  explicit SimpleTrackUpdater(TrackFilterConfig config = {});

  /// @brief 执行简单更新逻辑。
  /// @param predicted 预测后的状态。
  /// @param context 处理上下文。
  /// @return 更新后的目标特征。
  common::TargetFeature Update(const PredictedTrackState &predicted,
                               const TrackFilterContext &context) const override;

  /// @brief 更新配置参数。
  /// @param config 新配置。
  void UpdateConfig(TrackFilterConfig config);

 private:
  TrackFilterConfig config_{};
};

/// @brief 综合轨迹滤波器。
class TrackFilter final {
 public:
  /// @brief 构造函数。
  /// @param config 滤波器配置。
  explicit TrackFilter(TrackFilterConfig config = {});

  /// @brief 执行滤波处理。
  /// @param input 输入目标特征（来自检测或关联）。
  /// @param context 当前周期的传感器处理上下文。
  /// @return 滤波后的平滑目标特征。
  common::TargetFeature Filter(const common::TargetFeature &input,
                               const TrackFilterContext &context) const;

  /// @brief 更新配置参数。
  /// @param config 全新的配置结构。
  void UpdateConfig(TrackFilterConfig config);

private:
	IdentityTrackPredictor predictor_{};
	SimpleTrackUpdater updater_{};
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_FILTER_H_
