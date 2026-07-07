/**
 * @file EosDetectionLifecycleRecorder.h
 * @brief 定义 EOS 目标探测生命周期记录器。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DETECTION_LIFECYCLE_RECORDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DETECTION_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"

namespace electro_optical_sensor {
namespace session {

// 前向声明：Update 参数为 const 引用，header 无需完整类型，避免拉入 EosCycleInput 重依赖。
struct EosCycleInput;

/**
 * @brief EosDetectionLifecycleEventKind 表示单目标探测生命周期事件类型。
 */
enum class EosDetectionLifecycleEventKind {
  kFirstDetected = 0,  /**< 首次被发现 */
  kUpdated = 1,        /**< 持续跟踪并刷新 */
  kLost = 2,           /**< 此前已发现，本周期丢失 */
  kNotDetected = 3     /**< 本周期未发现（需显式开启） */
};

/**
 * @brief EosDetectionLifecycleReason 表示丢失/未发现事件的推断原因。
 */
enum class EosDetectionLifecycleReason {
  kNone = 0,                 /**< 无原因（事件为发现/更新类） */
  kOutOfFov = 1,             /**< 目标处于视场 (FOV) 之外 */
  kBelowSnrThreshold = 2,    /**< 融合信噪比 (SNR) 低于门限 */
  kValidationRejected = 3,   /**< 输入校验失败导致周期未执行 */
  kCycleNotExecuted = 4,     /**< 本周期核心 pipeline 未实际执行 */
  kUnknown = 5               /**< 原因不明 */
};

/**
 * @brief EosDetectionLifecycleEvent 描述单个目标在本周期的生命周期事件。
 */
struct ONEQ_API EosDetectionLifecycleEvent {
  std::uint32_t cycle_index{0U};                                       /**< 触发事件的周期号 */
  std::uint64_t target_id{0U};                                         /**< 目标标识 */
  std::string target_name{};                                           /**< 目标名称，仅用于人读 */
  EosDetectionLifecycleEventKind kind{EosDetectionLifecycleEventKind::kUpdated}; /**< 事件类型 */
  EosDetectionLifecycleReason reason{EosDetectionLifecycleReason::kNone};         /**< 丢失/未发现原因 */
  float fused_snr_db{0.0f};                                            /**< 融合信噪比（单位：dB） */
  float range_m{0.0f};                                                 /**< 目标斜距（单位：m） */
};

/**
 * @brief EosDetectionLifecycleRecorderConfig 描述生命周期记录器配置。
 */
struct ONEQ_API EosDetectionLifecycleRecorderConfig {
  bool emit_not_detected_events{false}; /**< 是否对「未发现」状态也输出事件，默认关闭 */
};

/**
 * @brief 记录目标首次发现/更新/丢失/未发现事件；未发现原因需显式开启。
 *
 * 私有状态(含 unordered_map)与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 * @note 仅可移动、不可拷贝；内部状态按 target_id 跨周期持续跟踪，非线程安全。
 */
class ONEQ_API EosDetectionLifecycleRecorder {
 public:
  /**
   * @brief 构造生命周期记录器。
   * @param[in] config 记录器配置，控制是否输出未发现事件。
   */
  explicit EosDetectionLifecycleRecorder(
      EosDetectionLifecycleRecorderConfig config = EosDetectionLifecycleRecorderConfig{});
  ~EosDetectionLifecycleRecorder();

  EosDetectionLifecycleRecorder(const EosDetectionLifecycleRecorder&) = delete;
  EosDetectionLifecycleRecorder& operator=(const EosDetectionLifecycleRecorder&) = delete;
  // 移动操作声明在 header、定义在 .cpp：unique_ptr<Impl> 析构需要完整类型，
  // 不能内联定义否则破坏 PImpl 不透明性。
  EosDetectionLifecycleRecorder(EosDetectionLifecycleRecorder&&) noexcept;
  EosDetectionLifecycleRecorder& operator=(EosDetectionLifecycleRecorder&&) noexcept;

  /**
   * @brief 根据本周期输入与结果生成生命周期事件。
   *
   * 遍历输入场景中的每个目标，结合归属映射与输出帧判定其相对上一周期的状态
   * （首次发现/更新/丢失/未发现），并返回本周期产生的事件列表。
   *
   * @param[in] input 当前周期输入，提供场景目标列表。
   * @param[in] result 当前周期聚合结果，提供输出帧与归属映射。
   * @return 本周期产生的生命周期事件列表，按场景目标顺序排列。
   */
  std::vector<EosDetectionLifecycleEvent> Update(const EosCycleInput& input, const EosCycleResult& result);

  /**
   * @brief 清空所有目标的跨周期跟踪状态。
   */
  void Reset();

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DETECTION_LIFECYCLE_RECORDER_H_
