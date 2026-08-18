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
#include "1q/sar/session/SarCycleResult.h"
#include "1q/threat_assessment/ThreatResult.h"

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

/** @brief AR 目标首确认事件（源：库内 ArTrackLifecycleRecorder 的 kFirstConfirmed）。 */
struct TargetConfirmedEvent {
  std::uint64_t cycle{0U};                   /**< 世界周期号 */
  std::uint64_t target_id{0U};               /**< 外部目标 ID（关联键） */
  oneq::coordinate::LlaPositionDegM position{}; /**< 目标位置（度制 LLA） */
};

/** @brief AR 目标失跟事件（源：库内 ArTrackLifecycleRecorder 的 kLost）。 */
struct TargetLostEvent {
  std::uint64_t cycle{0U};     /**< 世界周期号 */
  std::uint64_t target_id{0U}; /**< 外部目标 ID（关联键） */
  std::string reason{};        /**< 失跟原因（可读文本） */
};

/** @brief ESR 辐射源假设事件：每条新假设发布一次。 */
struct EmitterHypothesisEvent {
  std::uint64_t cycle{0U};                                     /**< 世界周期号 */
  std::uint64_t hypothesis_id{0U};                             /**< 假设标识 */
  double bearing_az_deg{0.0};                                  /**< 方位线方位角（deg） */
  double confidence{0.0};                                      /**< 假设置信度 [0,1] */
  electronic_surveillance_radar::session::EsrEmitterMode mode{ /**< 工作模式假设 */
      electronic_surveillance_radar::session::EsrEmitterMode::kUnknown};
};

/** @brief EOS 探测事件类型（生命周期语义，源为库内 EosDetectionLifecycleRecorder）。 */
enum class EosDetectionEventKind {
  kFirstDetected = 0, /**< 首次被发现 */
  kUpdated = 1,       /**< 持续探测并刷新 */
  kLost = 2,          /**< 此前已发现，本周期丢失 */
};

/** @brief EOS 探测事件：生命周期事件各发布一次（target_id 经归属映射）。 */
struct EosDetectionEvent {
  std::uint64_t cycle{0U};                /**< 世界周期号 */
  EosDetectionEventKind kind{EosDetectionEventKind::kUpdated}; /**< 事件类型 */
  std::uint64_t detection_id{0U};         /**< 本输出帧内探测记录标识（kLost 时为 0） */
  std::uint64_t target_id{0U};            /**< 归属目标 ID（无归属时为 0） */
  double snr_db{0.0};                     /**< 融合 SNR（dB；kLost 携带最后一次下检值） */
  double az_deg{0.0};                     /**< 探测方位（deg，平台局部系；kLost 时为 0） */
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

/** @brief 威胁评估更新事件：ThreatComponent 每周期评估后逐目标发布。 */
struct ThreatUpdatedEvent {
  std::uint64_t cycle{0U};  /**< 世界周期号 */
  threat_assessment::ThreatResult result{}; /**< 评估结果（键/威胁分/等级/贡献分解，组合而非复制） */
};

/** @brief SBIRS 探测事件类型（生命周期语义，源为库内 SbirsDetectionLifecycleRecorder）。 */
enum class SbirsDetectionEventKind {
  kFirstDetected = 0, /**< 首次被发现 */
  kUpdated = 1,       /**< 持续探测并刷新 */
  kCoasting = 2,      /**< 暂无有效 NFOV 量测但仍保持锁定 */
  kLost = 3,          /**< 此前已发现，本周期丢失 */
};

/**
 * @brief SBIRS 丢失/未检测原因（示例层枚举，源为库内
 * SbirsDetectionLifecycleReason，组件映射）。kLost 事件携带细分原因，
 * 区分"目标真消失"与"扫描间隙/调度跳过/门失败"——避免消费方把扫描间隙
 * 误读为目标丢失。
 */
enum class SbirsDetectionLossReason {
  kNone = 0,               /**< 无具体原因（非丢失事件） */
  kOutOfFieldOfView,       /**< 目标在视场（FOV）外（扫描相位未覆盖） */
  kBelowSnrThreshold,      /**< IR SNR 低于门限 */
  kTargetMissingFromInput, /**< 目标从输入场景消失 */
  kAcquisitionFailed,      /**< 进入 NFOV 待捕获但捕获失败 */
  kSchedulerSkipped,       /**< WFOV 候选未被调度器选中（通道被占） */
  kNisGateLost,            /**< EKF NIS 连续超限导致释放 NFOV 锁定 */
  kPointingTimeout,        /**< ATP 光轴未在派生等待上限内稳定 */
  kTrackingGateLost,       /**< NFOV 跟踪几何/SNR 门连续失败 */
  kUnknown                 /**< 未知原因 */
};

/** @brief SBIRS 探测事件：生命周期事件各发布一次（target_id 经归属映射）。 */
struct SbirsDetectionEvent {
  std::uint64_t cycle{0U};                     /**< 世界周期号 */
  SbirsDetectionEventKind kind{SbirsDetectionEventKind::kUpdated}; /**< 事件类型 */
  SbirsDetectionLossReason reason{SbirsDetectionLossReason::kNone}; /**< 丢失/未检测细分原因（非丢失事件为 kNone） */
  std::uint64_t detection_id{0U};              /**< 本输出帧内检测记录标识（kLost 时为 0） */
  std::uint64_t target_id{0U};                 /**< 归属目标 ID（无归属时为 0） */
  float infrared_snr_linear{0.0f};             /**< 红外通道线性 IR SNR（kLost 携带最后一次下检值） */
  double az_deg{0.0};                          /**< 探测方位（deg，卫星局部系；kLost 时为 0） */
};

/** @brief SAR 产品事件类型（生命周期语义，源为库内 SarProductLifecycleRecorder）。 */
enum class SarProductEventKind {
  kImageProduced = 0,    /**< 首次产出图像产品 */
  kProductSustained = 1, /**< 产品持续存在 */
  kProductLost = 2,      /**< 已有产品本周期丢失 */
  kProcessingFailed = 3, /**< 处理失败 */
};

/** @brief SAR 产品事件：产品生命周期事件各发布一次（kNoProduct 不转发）。 */
struct SarProductEvent {
  std::uint64_t cycle{0U};                              /**< 世界周期号 */
  SarProductEventKind kind{SarProductEventKind::kProductSustained}; /**< 事件类型 */
  sar::session::SarProcessingStage stage{               /**< 本周期完成处理阶段 */
      sar::session::SarProcessingStage::kNone};
  double estimated_snr_db{0.0};                         /**< 估计 SNR（dB） */
  std::string abort_reason{};                           /**< 中止原因（kProcessingFailed 时非空） */
};

/** @brief 决策指令事件：高置信威胁判定后由决策侧（订阅者）发布。 */
struct CommandIssuedEvent {
  std::uint64_t cycle{0U}; /**< 世界周期号 */
  std::string command{};   /**< 指令描述（可读文本） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_CORE_EVENTS_H_
