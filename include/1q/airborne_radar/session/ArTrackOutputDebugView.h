/**
 * @file ArTrackOutputDebugView.h
 * @brief AR module primary track output debug view types.
 *
 * Primary header for track debug views.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_OUTPUT_DEBUG_VIEW_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_OUTPUT_DEBUG_VIEW_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/airborne_radar/session/ArCycleResult.h"

namespace airborne_radar {
namespace session {

// 前向声明：Build 参数为 const 引用，header 无需完整类型，避免拉入 ArCycleInput 重依赖。
struct ArCycleInput;

enum class ONEQ_API ArDebugTrackStatus {
  kConfirmed = 0,
  kTentative = 1,
  kLost = 2,
  kNotInOutput = 3,
  kCycleNotExecuted = 4
};

struct ONEQ_API ArDebugTrackState {
  std::uint64_t external_target_id{0U};
  std::string target_name{};
  ArDebugTrackStatus status{ArDebugTrackStatus::kNotInOutput};
  bool present_in_input{false};
  bool has_track{false};
  std::uint64_t association_key{0U};
  float position_x{0.0f};
  float position_y{0.0f};
  float position_z{0.0f};
  float speed{0.0f};
  float rcs{0.0f};
  std::uint32_t hit_count{0U};
  std::uint32_t miss_count{0U};
  std::string target_type{};
};

struct ONEQ_API ArTrackOutputDebugView {
  std::uint32_t input_cycle_index{0U};
  std::uint32_t output_cycle_index{0U};
  bool executed_this_cycle{false};
  bool reused_previous_output{false};
  bool has_validation_error{false};
  session::SignalCycleAbortReason abort_reason{session::SignalCycleAbortReason::kNone};
  std::vector<ArDebugTrackState> tracks{};
};

/**
 * @brief 把轨迹输出帧、执行结果与输入目标表合成为开发可读轨迹状态。
 *
 * 该构建器只读组合，不反向影响 signal/decision pipeline。实现见 .cpp。
 * 与 EOS 不同，AR 的 track 是系统估计（非传感器原始输出），target_name 作为
 * 人读标签直接附在 track 上，无需独立 attribution 层。
 */
class ONEQ_API ArTrackOutputDebugViewBuilder {
 public:
  static ArTrackOutputDebugView Build(const ArCycleInput& input, const ArCycleResult& result);
};


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_OUTPUT_DEBUG_VIEW_H_
