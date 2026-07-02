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

struct PulseRecord {
  std::uint64_t pulse_id{0U};
  signal::ComplexVector samples{};
};

struct PulseRingBufferRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  std::size_t capacity{0U};
  std::deque<PulseRecord> records{};
  bool overflow_sticky{false};
};

class PulseRingBuffer {
 public:
  explicit PulseRingBuffer(std::size_t capacity);

  bool Push(const PulseRecord& pulse);
  bool ReadRange(std::uint64_t first_pulse_id, std::size_t count,
                 std::vector<PulseRecord>* output) const;
  bool ReadLatest(std::size_t count, std::vector<PulseRecord>* output) const;

  std::size_t size() const;
  std::size_t capacity() const;
  bool overflow_sticky() const;
  PulseRingBufferRuntimeState CaptureRuntimeState() const;
  bool RestoreRuntimeState(const PulseRingBufferRuntimeState& state);

 private:
  bool Contains(std::uint64_t pulse_id) const;
  bool IsContiguous(std::size_t first_index, std::size_t count) const;

  std::size_t capacity_{0U};
  std::deque<PulseRecord> records_{};
  bool overflow_sticky_{false};
};

}  // namespace runtime
}  // namespace sar

#endif  // ONEQ_SRC_SAR_RUNTIME_PULSE_RING_BUFFER_H_
