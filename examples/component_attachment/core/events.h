/**
 * @file events.h
 * @brief 自定义实体-组件示例：组件间通信的事件类型集合。
 *
 * 事件为纯数据结构（无逻辑、无虚函数），作为组件间通信的协议载荷：
 * 组件经 World 的信号（Boost.Signals2，见 signals.h）发布，消费方订阅。
 * 周期内同步数据聚合（如融合读传感器探测）用同实体组件的类型化访问，
 * 跨周期通知/记录走事件——两种通信形态在本示例中同时演示。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_CORE_EVENTS_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_CORE_EVENTS_H_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"

namespace component_attachment {

/** @brief 平台状态事件：FlightComponent 每周期推进后发布。 */
struct PlatformStateEvent {
  std::uint64_t cycle{0U};                     /**< 世界周期号 */
  double t_sec{0.0};                           /**< 周期绝对时间（s） */
  oneq::coordinate::EcefPositionM position_ecef_m{}; /**< 平台位置（ECEF，m） */
  double altitude_m{0.0};                      /**< 海拔（m） */
  double heading_deg{0.0};                     /**< 航向（deg，北偏东） */
  double speed_mps{0.0};                       /**< 速度（m/s） */
  std::size_t waypoint_index{0U};              /**< 下一航点索引 */
  std::size_t waypoint_count{0U};              /**< 航点总数 */
};

/** @brief 航点到达事件：FlightComponent 在航点完成判定时发布。 */
struct WaypointReachedEvent {
  double t_sec{0.0};              /**< 到达时刻（s） */
  std::size_t waypoint_index{0U}; /**< 完成航点索引 */
  double distance_m{0.0};         /**< 到达时刻距离（m） */
};

/** @brief AR 目标首次确认事件（kConfirmed 首次出现）。 */
struct TargetConfirmedEvent {
  std::uint64_t cycle{0U};                   /**< 世界周期号 */
  std::uint64_t target_id{0U};               /**< 外部目标 ID（关联键） */
  oneq::coordinate::LlaPositionDegM position{}; /**< 目标位置（度制 LLA） */
};

/** @brief AR 目标失跟事件（kLost 且此前已确认）。 */
struct TargetLostEvent {
  std::uint64_t cycle{0U};     /**< 世界周期号 */
  std::uint64_t target_id{0U}; /**< 外部目标 ID（关联键） */
  std::string reason{};        /**< 失跟原因（可读文本） */
};

/** @brief ESR 辐射源假设事件：每条新假设发布一次。 */
struct EmitterHypothesisEvent {
  std::uint64_t cycle{0U};                                           /**< 世界周期号 */
  std::uint64_t hypothesis_id{0U};                                   /**< 假设标识 */
  double bearing_az_deg{0.0};                                        /**< 方位线方位角（deg） */
  double confidence{0.0};                                            /**< 假设置信度 [0,1] */
  electronic_surveillance_radar::session::EsrEmitterMode mode{       /**< 工作模式假设 */
      electronic_surveillance_radar::session::EsrEmitterMode::kUnknown};
  electronic_surveillance_radar::session::EsrThreatLevel threat_level{ /**< 威胁等级 */
      electronic_surveillance_radar::session::EsrThreatLevel::kLow};
};

/** @brief EOS 探测事件：通过探测门限的记录各发布一次（target_id 经归属映射）。 */
struct EosDetectionEvent {
  std::uint64_t cycle{0U};       /**< 世界周期号 */
  std::uint64_t detection_id{0U}; /**< 本输出帧内探测记录标识 */
  std::uint64_t target_id{0U};   /**< 归属目标 ID（无归属时为 0） */
  double snr_db{0.0};            /**< 融合 SNR（dB） */
  double az_deg{0.0};            /**< 探测方位（deg，平台局部系） */
};

/** @brief 融合态势更新事件：FusionComponent 每周期更新后发布。 */
struct FusionUpdatedEvent {
  std::uint64_t cycle{0U};                  /**< 世界周期号 */
  std::uint64_t key{0U};                    /**< 融合目标键 */
  double confidence{0.0};                   /**< 融合置信度 */
  std::vector<std::pair<std::uint32_t, std::size_t>> channels{}; /**< (源通道, 样本数) 列表 */
  std::size_t new_targets{0U};              /**< 本周期新目标数 */
  std::size_t lost_targets{0U};             /**< 本周期消失目标数 */
};

/** @brief 决策指令事件：高置信威胁判定后由决策侧（订阅者）发布。 */
struct CommandIssuedEvent {
  std::uint64_t cycle{0U}; /**< 世界周期号 */
  std::string command{};   /**< 指令描述（可读文本） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_CORE_EVENTS_H_
