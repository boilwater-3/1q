#include "sar/runtime/PulseRingBuffer.h"

#include <cmath>

namespace sar {
namespace runtime {

namespace {

constexpr std::uint32_t kPulseRingBufferRuntimeStateSchemaVersion = 2U;

bool IsStrictlyIncreasing(const std::deque<PulseRecord>& records) {
  for (std::size_t i = 0U; i < records.size(); ++i) {
    if (!std::isfinite(records[i].signal_power_w) || records[i].signal_power_w < 0.0 ||
        !std::isfinite(records[i].noise_power_w) || records[i].noise_power_w < 0.0 ||
        (i > 0U && records[i].pulse_id <= records[i - 1U].pulse_id)) {
      return false;
    }
  }
  return true;
}

}  // namespace

PulseRingBuffer::PulseRingBuffer(std::size_t capacity) : capacity_(capacity) {}

bool PulseRingBuffer::Push(const PulseRecord& pulse) {
  if (capacity_ == 0U) {
    return false;
  }
  // 单调性守卫同时承担重复拒绝：缓冲维护 pulse_id 严格递增不变量（Push 拒绝
  // <= back，RestoreRuntimeState 校验 IsStrictlyIncreasing），重复或乱序 id 必然
  // <= back，无需 O(n) 全量查重（原 Contains 线性扫描已移除）。
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

PulseRingBufferRuntimeState PulseRingBuffer::CaptureRuntimeState() const {
  PulseRingBufferRuntimeState state;
  state.owner_identity = this;
  state.schema_version = kPulseRingBufferRuntimeStateSchemaVersion;
  state.capacity = capacity_;
  state.records = records_;
  state.overflow_sticky = overflow_sticky_;
  return state;
}

bool PulseRingBuffer::RestoreRuntimeState(const PulseRingBufferRuntimeState& state) {
  if (state.owner_identity != this ||
      state.schema_version != kPulseRingBufferRuntimeStateSchemaVersion ||
      state.records.size() > state.capacity || !IsStrictlyIncreasing(state.records)) {
    return false;
  }
  capacity_ = state.capacity;
  records_ = state.records;
  overflow_sticky_ = state.overflow_sticky;
  return true;
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
