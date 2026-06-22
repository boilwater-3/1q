#include "sar/runtime/PulseRingBuffer.h"

namespace sar {
namespace runtime {

PulseRingBuffer::PulseRingBuffer(std::size_t capacity) : capacity_(capacity) {}

bool PulseRingBuffer::Push(const PulseRecord& pulse) {
  if (capacity_ == 0U || Contains(pulse.pulse_id)) {
    return false;
  }

  if (!records_.empty() && pulse.pulse_id <= records_.back().pulse_id) {
    return false;
  }

  if (records_.size() == capacity_) {
    records_.pop_front();
    overflow_sticky_ = true;
  }
  records_.push_back(pulse);
  return true;
}

bool PulseRingBuffer::ReadRange(std::uint64_t first_pulse_id, std::size_t count,
                                std::vector<PulseRecord>* output) const {
  if (output == nullptr || count == 0U || count > records_.size()) {
    return false;
  }

  for (std::size_t i = 0U; i < records_.size(); ++i) {
    if (records_[i].pulse_id == first_pulse_id) {
      if (!IsContiguous(i, count)) {
        return false;
      }
      output->assign(records_.begin() + static_cast<std::ptrdiff_t>(i),
                     records_.begin() + static_cast<std::ptrdiff_t>(i + count));
      return true;
    }
  }
  return false;
}

bool PulseRingBuffer::ReadLatest(std::size_t count, std::vector<PulseRecord>* output) const {
  if (output == nullptr || count == 0U || count > records_.size()) {
    return false;
  }
  const std::size_t first_index = records_.size() - count;
  if (!IsContiguous(first_index, count)) {
    return false;
  }
  output->assign(records_.begin() + static_cast<std::ptrdiff_t>(first_index), records_.end());
  return true;
}

std::size_t PulseRingBuffer::size() const { return records_.size(); }

std::size_t PulseRingBuffer::capacity() const { return capacity_; }

bool PulseRingBuffer::overflow_sticky() const { return overflow_sticky_; }

bool PulseRingBuffer::Contains(std::uint64_t pulse_id) const {
  for (const PulseRecord& record : records_) {
    if (record.pulse_id == pulse_id) {
      return true;
    }
  }
  return false;
}

bool PulseRingBuffer::IsContiguous(std::size_t first_index, std::size_t count) const {
  if (first_index + count > records_.size()) {
    return false;
  }
  for (std::size_t i = 1U; i < count; ++i) {
    if (records_[first_index + i].pulse_id != records_[first_index].pulse_id + i) {
      return false;
    }
  }
  return true;
}

}  // namespace runtime
}  // namespace sar
