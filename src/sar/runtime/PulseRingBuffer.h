/**
 * @file PulseRingBuffer.h
 * @brief SAR 内部 pulse_id 连续性环形缓冲区。
 */

#ifndef ONEQ_SRC_SAR_RUNTIME_PULSE_RING_BUFFER_H_
#define ONEQ_SRC_SAR_RUNTIME_PULSE_RING_BUFFER_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace runtime {

/**
 * @brief 单脉冲记录（pulse_id + 复采样）。
 */
struct PulseRecord {
  std::uint64_t pulse_id{0U};        /**< 脉冲 ID */
  signal::ComplexVector samples{};   /**< 该脉冲的复采样 */
  double signal_power_w{0.0};        /**< 加噪前的平均接收信号功率（W） */
  double noise_power_w{0.0};         /**< 接收机复采样平均噪声功率（W） */
};

/**
 * @brief 脉冲环形缓冲区运行期可序列化状态快照。
 */
struct PulseRingBufferRuntimeState {
  const void* owner_identity{nullptr};   /**< 用于恢复时校验所有者身份的 opaque 指针 */
  std::uint32_t schema_version{0U};      /**< 状态结构 schema 版本 */
  std::size_t capacity{0U};              /**< 缓冲区容量 */
  std::deque<PulseRecord> records{};     /**< 当前持有的脉冲记录 */
  bool overflow_sticky{false};           /**< 粘滞溢出标志（曾发生溢出则置真） */
};

/**
 * @brief 按 pulse_id 维护连续性的脉冲环形缓冲区。
 *
 * 容量固定；超出容量时丢弃最旧脉冲并置位粘滞溢出标志。支持按 ID 区间或最新 N 条读取。
 * @note 非线程安全；调用方需保证对同一实例的访问串行化。
 */
class PulseRingBuffer {
 public:
  /**
   * @brief 构造指定容量的环形缓冲区。
   * @param[in] capacity 最大保留脉冲数。
   */
  explicit PulseRingBuffer(std::size_t capacity);

  /**
   * @brief 追加一条脉冲记录。
   * @param[in] pulse 待追加的脉冲记录。
   * @return 追加成功返回 true；pulse_id 非连续（倒退或跳跃过大）返回 false。
   */
  bool Push(const PulseRecord& pulse);
  /**
   * @brief 读取从指定 pulse_id 起连续 count 条脉冲。
   * @param[in] first_pulse_id 起始脉冲 ID。
   * @param[in] count 读取条数。
   * @param[out] output 输出脉冲记录列表。
   * @return 全部命中且连续返回 true；否则返回 false。
   */
  bool ReadRange(std::uint64_t first_pulse_id, std::size_t count,
                 std::vector<PulseRecord>* output) const;
  /**
   * @brief 读取最新 count 条脉冲。
   * @param[in] count 读取条数。
   * @param[out] output 输出脉冲记录列表。
   * @return 命中足够条数返回 true；否则返回 false。
   */
  bool ReadLatest(std::size_t count, std::vector<PulseRecord>* output) const;

  /**
   * @brief 返回当前持有脉冲数。
   * @return 当前条数。
   */
  std::size_t size() const;
  /**
   * @brief 返回缓冲区容量。
   * @return 容量。
   */
  std::size_t capacity() const;
  /**
   * @brief 返回粘滞溢出标志。
   * @return 曾发生溢出返回 true。
   */
  bool overflow_sticky() const;
  /**
   * @brief 捕获当前运行期状态快照。
   * @return 可用于 RestoreRuntimeState 的状态快照。
   */
  PulseRingBufferRuntimeState CaptureRuntimeState() const;
  /**
   * @brief 从快照恢复运行期状态。
   * @param[in] state 之前捕获的状态快照。
   * @return schema_version 等校验通过返回 true，否则返回 false。
   */
  bool RestoreRuntimeState(const PulseRingBufferRuntimeState& state);

 private:
  bool IsContiguous(std::size_t first_index, std::size_t count) const;

  std::size_t capacity_{0U};
  std::deque<PulseRecord> records_{};
  bool overflow_sticky_{false};
};

}  // namespace runtime
}  // namespace sar

#endif  // ONEQ_SRC_SAR_RUNTIME_PULSE_RING_BUFFER_H_
