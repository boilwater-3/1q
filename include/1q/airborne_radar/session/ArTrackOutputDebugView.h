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

#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief 调试视图下的轨迹状态分类（含周期未执行/输出中缺失等诊断态）。
 */
enum class ONEQ_API ArDebugTrackStatus {
  kConfirmed = 0,        /**< 轨迹处于已确认状态。 */
  kTentative = 1,        /**< 轨迹处于候选状态。 */
  kLost = 2,             /**< 轨迹处于丢失状态。 */
  kNotInOutput = 3,      /**< 输入目标在本周期输出中无对应 track。 */
  kCycleNotCompleted = 4 /**< 本周期未成功完成，无法判定状态。 */
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
  std::uint64_t world_cycle_index{0U};  /**< 单周期结果所属世界周期号 */
  std::uint32_t output_cycle_index{0U}; /**< 输出帧内部周期号 */
  bool completed_this_cycle{false};     /**< 单周期执行是否成功 */
  ArReceiverImpairment receiver_impairment{ArReceiverImpairment::kNone}; /**< 接收机损伤 */
  std::vector<ArDebugTrackState> tracks{}; /**< 逐输入目标的调试轨迹状态 */
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
   * @brief 把单周期结果与目标事实合成为开发可读轨迹状态。
   *
   * 只读组合，不反向影响 signal/decision pipeline；按输入目标逐项合成调试状态。
   *
   * @param[in] targets 当前周期目标事实（用于遍历输入目标表）。
   * @param[in] result 当前周期结果（用于查询轨迹状态与接收机损伤）。
   * @return 合成的单周期轨迹输出调试视图。
   */
  static ArTrackOutputDebugView Build(const ArTargetInputList& targets,
                                      const ArCycleResult& result);
};


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_OUTPUT_DEBUG_VIEW_H_
