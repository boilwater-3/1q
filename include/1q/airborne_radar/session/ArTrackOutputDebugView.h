/**
 * @file ArTrackOutputDebugView.h
 * @brief 机载雷达轨迹输出调试视图类型集合。
 *
 * 轨迹输出调试视图（开发可读轨迹状态合成）的主头文件。
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

/**
 * @brief 调试视图下的轨迹状态分类（含周期未执行/输出中缺失等诊断态）。
 */
enum class ONEQ_API ArDebugTrackStatus {
  kConfirmed = 0,       /**< 轨迹处于已确认状态。 */
  kTentative = 1,       /**< 轨迹处于候选状态。 */
  kLost = 2,            /**< 轨迹处于丢失状态。 */
  kNotInOutput = 3,     /**< 输入目标在本周期输出中无对应 track。 */
  kCycleNotExecuted = 4 /**< 本周期主链路未真正执行，无法判定状态。 */
};

/**
 * @brief 单个输入目标在调试视图下的合成状态。
 */
struct ONEQ_API ArDebugTrackState {
  std::uint64_t external_target_id{0U};                              /**< 输入目标外部标识符 */
  std::string target_name{};                                         /**< 目标名称（人读标签） */
  ArDebugTrackStatus status{ArDebugTrackStatus::kNotInOutput};       /**< 调试状态分类 */
  bool present_in_input{false};                                      /**< 是否出现在本周期输入目标表 */
  bool has_track{false};                                             /**< 本周期输出中是否存在对应 track */
  std::uint64_t association_key{0U};                                 /**< 关联轨迹的关联键 */
  float position_x{0.0f};                                            /**< 轨迹位置 x（雷达局部，m） */
  float position_y{0.0f};                                            /**< 轨迹位置 y（雷达局部，m） */
  float position_z{0.0f};                                            /**< 轨迹位置 z（雷达局部，m） */
  float speed{0.0f};                                                 /**< 轨迹速度模长（m/s） */
  float rcs{0.0f};                                                   /**< 目标 RCS（m^2） */
  std::uint32_t hit_count{0U};                                       /**< 命中累计计数 */
  std::uint32_t miss_count{0U};                                      /**< 连续失配计数 */
  std::string target_type{};                                         /**< 决策层填充的目标类型标签 */
};

/**
 * @brief 单周期的轨迹输出调试视图聚合。
 */
struct ONEQ_API ArTrackOutputDebugView {
  std::uint32_t input_cycle_index{0U};                               /**< 本次调用输入周期号 */
  std::uint32_t output_cycle_index{0U};                              /**< 输出帧周期号 */
  bool executed_this_cycle{false};                                   /**< 本周期主链路是否真正执行 */
  bool reused_previous_output{false};                                /**< 输出是否复用了上一有效周期 */
  bool has_validation_error{false};                                  /**< 是否存在 error 级输入问题 */
  session::SignalCycleAbortReason abort_reason{session::SignalCycleAbortReason::kNone}; /**< abort 原因 */
  std::vector<ArDebugTrackState> tracks{};                           /**< 逐输入目标的调试轨迹状态 */
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
  /**
   * @brief 把轨迹输出帧、执行结果与输入目标表合成为开发可读轨迹状态。
   *
   * 只读组合，不反向影响 signal/decision pipeline；按输入目标逐项合成调试状态。
   *
   * @param[in] input 当前周期输入（用于遍历输入目标表）。
   * @param[in] result 当前周期执行结果（用于查询轨迹状态与执行标志）。
   * @return 合成的单周期轨迹输出调试视图。
   */
  static ArTrackOutputDebugView Build(const ArCycleInput& input, const ArCycleResult& result);
};


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_OUTPUT_DEBUG_VIEW_H_
