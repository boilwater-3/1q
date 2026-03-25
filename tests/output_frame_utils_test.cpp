/**
 * @file output_frame_utils_test.cpp
 * @brief 验证输出帧通用工具的计数与帧头填充行为。
 */

#include <gtest/gtest.h>

#include <vector>

#include "common/output/OutputFrameUtils.h"

namespace oneq {
namespace internal {
namespace output {
namespace {

struct DummyFrame {
  std::uint32_t cycle_index{0U};
  std::uint64_t batch_id{0U};
};

TEST(OutputFrameUtilsTest, CountMatchingReturnsZeroForEmptyContainer) {
  const std::vector<int> values;
  const std::size_t count = CountMatching(values, [](int value) { return value > 0; });
  EXPECT_EQ(count, 0U);
}

TEST(OutputFrameUtilsTest, CountMatchingCountsAllMatchedElements) {
  const std::vector<int> values(4U, 7);
  const std::size_t count = CountMatching(values, [](int value) { return value == 7; });
  EXPECT_EQ(count, 4U);
}

TEST(OutputFrameUtilsTest, CountMatchingCountsPartiallyMatchedElements) {
  const std::vector<int> values = {1, 2, 3, 4, 5, 6};
  const std::size_t count = CountMatching(values, [](int value) { return (value % 2) == 0; });
  EXPECT_EQ(count, 3U);
}

TEST(OutputFrameUtilsTest, SetCycleAndBatchWritesFrameHeaderFields) {
  DummyFrame frame;
  SetCycleAndBatch(frame, 9U, 42U);
  EXPECT_EQ(frame.cycle_index, 9U);
  EXPECT_EQ(frame.batch_id, 42U);
}

}  // namespace
}  // namespace output
}  // namespace internal
}  // namespace oneq
