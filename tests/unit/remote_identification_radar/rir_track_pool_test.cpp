// Copyright 2026. All Rights Reserved.
//
// @file rir_track_pool_test.cpp
// @brief 验证 RIR 航迹对象池申请/归还/双重释放拒绝语义（阶段 2-T N3）。

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "remote_identification_radar/tracking/RirTrackPool.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using tracking::RirTrackPool;

/// @brief 预充对象可被申请；计数与容量随申请/归还正确变化。
TEST(RirTrackPoolTest, AcquireFromPrewarmedTracksCounts) {
  RirTrackPool pool(4U, 8U);
  EXPECT_EQ(pool.FreeCount(), 4U);
  EXPECT_EQ(pool.InUseCount(), 0U);
  EXPECT_EQ(pool.Capacity(), 4U);

  tracking::RirTrackState* first = pool.Acquire();
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(pool.InUseCount(), 1U);
  EXPECT_EQ(pool.FreeCount(), 3U);

  pool.Release(first);
  EXPECT_EQ(pool.InUseCount(), 0U);
  EXPECT_EQ(pool.FreeCount(), 4U);
}

/// @brief 归还后的槽位被下次申请复用（LIFO 自由表），申请超预充数可实时构造。
TEST(RirTrackPoolTest, ReleaseThenAcquireReusesSlotAndGrowsOnDemand) {
  RirTrackPool pool(1U, 8U);
  tracking::RirTrackState* first = pool.Acquire();
  ASSERT_NE(first, nullptr);
  tracking::RirTrackState* second = pool.Acquire();  // 超预充：实时构造
  ASSERT_NE(second, nullptr);
  EXPECT_NE(first, second);

  pool.Release(first);
  tracking::RirTrackState* reused = pool.Acquire();  // LIFO 复用刚归还的槽位
  EXPECT_EQ(reused, first);
}

/// @brief 双重释放被拒绝：对象不重入自由表，计数不重复扣减。
TEST(RirTrackPoolTest, DoubleReleaseIsRejected) {
  RirTrackPool pool(2U, 8U);
  tracking::RirTrackState* track = pool.Acquire();
  ASSERT_NE(track, nullptr);

  pool.Release(track);
  const std::size_t free_after_release = pool.FreeCount();
  const std::size_t in_use_after_release = pool.InUseCount();

  pool.Release(track);  // 双重释放：被拒绝
  EXPECT_EQ(pool.FreeCount(), free_after_release);
  EXPECT_EQ(pool.InUseCount(), in_use_after_release);

  // 防护语义是不重复入表（防后续重复发放）：对象经首次合法释放仍可复用，
  // 连续两次申请必须得到两个不同槽位。
  tracking::RirTrackState* first_take = pool.Acquire();
  tracking::RirTrackState* second_take = pool.Acquire();
  ASSERT_NE(first_take, nullptr);
  ASSERT_NE(second_take, nullptr);
  EXPECT_NE(first_take, second_take);
  EXPECT_EQ(pool.InUseCount(), in_use_after_release + 2U);
}

/// @brief 空指针归还为无害空操作。
TEST(RirTrackPoolTest, NullReleaseIsNoOp) {
  RirTrackPool pool(2U, 8U);
  pool.Release(nullptr);
  EXPECT_EQ(pool.InUseCount(), 0U);
  EXPECT_EQ(pool.FreeCount(), 2U);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
