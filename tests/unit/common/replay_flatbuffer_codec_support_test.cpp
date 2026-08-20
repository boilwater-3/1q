// 验证 replay codec 内部 helper 只复制完成后的 FlatBuffer 字节，不改变其大小或内容。

#include <gtest/gtest.h>

#include <string>

#include "common/replay/ReplayFlatbufferCodecSupport.h"

namespace oneq {
namespace common {
namespace replay {
namespace tests {

TEST(ReplayFlatbufferCodecSupportTest, CopiesFinishedBufferWithoutChangingBytes) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::uoffset_t start = builder.StartTable();
  // vtable 偏移从 4 开始（偏移 0 是 vtable 大小保留区）；字段偏移 0 会在
  // EndTable 覆写 buffer 开头（debug 断言、release 静默损坏）。
  builder.AddElement<std::uint32_t>(4, 42U, 0U);
  const flatbuffers::Offset<void> root = builder.EndTable(start);
  builder.Finish(root);

  const std::string bytes = CopyFinishedFlatbuffer(builder);
  ASSERT_EQ(bytes.size(), builder.GetSize());
  EXPECT_EQ(bytes, std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                               builder.GetSize()));
}

}  // namespace tests
}  // namespace replay
}  // namespace common
}  // namespace oneq
