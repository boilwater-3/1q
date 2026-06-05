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
