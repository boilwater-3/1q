/**
 * @file replay_trace_compression_test.cpp
 * @brief 验证 ReplayTraceWriter 的 compress_closed_chunks 功能：
 *        chunk 关闭后生成 .gz、Reader 透明解压、事件内容一致、
 *        大体积事件流体积显著缩减。
 */

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "1q/replay/ReplayTrace.h"

#if ONEQ_HAVE_ZLIB
#include <zlib.h>
#endif

namespace oneq {
namespace replay {
namespace tests {

namespace {

// 创建带指定选项的临时 trace 目录（用 mkdtemp 风格的唯一路径）。
std::string MakeTempTraceDir(const std::string& suffix) {
  const std::string base =
      std::string(std::getenv("TMPDIR") != nullptr ? std::getenv("TMPDIR") : "/tmp") + "/oneq_zlib_test_" + suffix;
  return base;
}

ReplayTraceManifest MakeTestManifest(bool compress, std::uint32_t chunk_size = 3U) {
  ReplayTraceManifest m;
  m.trace_id = "test-zlib-" + std::string(compress ? "on" : "off");
  m.module = "test";
  m.event_chunk_size = chunk_size;
  m.failure_window_event_count = 8U;
  m.compress_closed_chunks = compress;
  return m;
}

ReplayTraceEvent MakeEvent(const std::string& payload) {
  ReplayTraceEvent ev;
  ev.module = "test";
  ev.event_type = "cycle_input";
  ev.payload_type = "TestPayload";
  ev.payload_encoding = "flatbuffers";
  ev.payload_bytes = payload;
  ev.has_cycle_index = true;
  ev.cycle_index = 0U;
  return ev;
}

bool FileExists(const std::string& path) {
  std::ifstream f(path.c_str());
  return f.good();
}

std::uint64_t FileSize(const std::string& path) {
  std::ifstream f(path.c_str(), std::ios::ate | std::ios::binary);
  if (!f.is_open()) {
    return 0U;
  }
  return static_cast<std::uint64_t>(f.tellg());
}

}  // namespace

// ---------------------------------------------------------------------------
// 压缩关闭时：sealed chunk 必须存在 .gz，不存在原始 .jsonl
// ---------------------------------------------------------------------------

#if ONEQ_HAVE_ZLIB
TEST(ReplayTraceCompressionTest, SealedChunkIsCompressedAndOriginalRemoved) {
  const std::string trace_dir = MakeTempTraceDir("sealed");
  {
    ReplayTraceWriter writer(trace_dir, MakeTestManifest(true, /*chunk_size=*/2U), true);
    // 写 3 个事件：第 3 个触发 chunk 0→1 轮转并压缩 chunk 0
    writer.WriteEvent(MakeEvent("payload-a"));
    writer.WriteEvent(MakeEvent("payload-b"));
    writer.WriteEvent(MakeEvent("payload-c"));
  }

  const std::string chunk0_plain = trace_dir + "/events/000000.events.jsonl";
  const std::string chunk0_gz    = trace_dir + "/events/000000.events.jsonl.gz";
  const std::string chunk1_plain = trace_dir + "/events/000001.events.jsonl";

  // chunk 0 已被压缩 → 原文件不存在，.gz 文件存在
  EXPECT_FALSE(FileExists(chunk0_plain)) << "原始 chunk0 应已被删除";
  EXPECT_TRUE(FileExists(chunk0_gz))    << ".gz chunk0 应存在";
  // chunk 1 是当前写入中，仍为普通文件
  EXPECT_TRUE(FileExists(chunk1_plain)) << "当前 chunk1 应为普通文件";
}

// ---------------------------------------------------------------------------
// Reader 透明解压：读出的事件与写入完全一致
// ---------------------------------------------------------------------------

TEST(ReplayTraceCompressionTest, ReaderTransparentlyDecompressesGzChunks) {
  const std::string trace_dir = MakeTempTraceDir("reader");
  const std::vector<std::string> payloads = {"aaa", "bbb", "ccc", "ddd", "eee"};

  {
    // chunk_size=2 → 事件 0-1 在 chunk0，事件 2-3 在 chunk1，事件 4 在 chunk2
    ReplayTraceWriter writer(trace_dir, MakeTestManifest(true, /*chunk_size=*/2U), true);
    for (const std::string& p : payloads) {
      writer.WriteEvent(MakeEvent(p));
    }
  }

  ReplayTraceReader reader(trace_dir);
  std::vector<std::string> read_payloads;
  ReplayTraceReadEvent ev;
  while (reader.ReadNextEvent(&ev)) {
    read_payloads.push_back(ev.payload_bytes);
  }

  ASSERT_EQ(read_payloads.size(), payloads.size());
  for (std::size_t i = 0; i < payloads.size(); ++i) {
    EXPECT_EQ(read_payloads[i], payloads[i]) << "事件 " << i << " payload 不匹配";
  }
}

// ---------------------------------------------------------------------------
// 压缩体积：实际 .gz 比原 .jsonl 小（用重复数据验证）
// ---------------------------------------------------------------------------

TEST(ReplayTraceCompressionTest, CompressedChunkIsSmallerThanPlain) {
  // 写两份 trace：一份压缩，一份不压缩，使用高重复性 payload
  const std::string trace_compressed = MakeTempTraceDir("compress_size");
  const std::string trace_plain      = MakeTempTraceDir("plain_size");

  // 构造 1 KB 高重复 payload
  const std::string repeated_payload(1024U, 'X');

  auto write_trace = [&](const std::string& dir, bool compress) {
    // chunk_size=3: 写 4 个事件 → chunk0 (3 events) 被封存并压缩，chunk1 (1 event) 为当前
    ReplayTraceWriter writer(dir, MakeTestManifest(compress, /*chunk_size=*/3U), true);
    for (int i = 0; i < 4; ++i) {
      writer.WriteEvent(MakeEvent(repeated_payload));
    }
  };

  write_trace(trace_compressed, true);
  write_trace(trace_plain, false);

  const std::string gz_chunk   = trace_compressed + "/events/000000.events.jsonl.gz";
  const std::string plain_chunk = trace_plain      + "/events/000000.events.jsonl";

  // 只要 .gz 存在就认为压缩成功（体积缩减是概率性的，但对重复数据几乎必然）
  ASSERT_TRUE(FileExists(gz_chunk))    << ".gz chunk 应存在";
  ASSERT_TRUE(FileExists(plain_chunk)) << "plain chunk 应存在";

  const std::uint64_t gz_size    = FileSize(gz_chunk);
  const std::uint64_t plain_size = FileSize(plain_chunk);
  EXPECT_GT(plain_size, gz_size)
      << "压缩后体积应小于原始（plain=" << plain_size << " gz=" << gz_size << ")";
}

// ---------------------------------------------------------------------------
// 不压缩时：行为与旧版一致，无 .gz 文件
// ---------------------------------------------------------------------------

TEST(ReplayTraceCompressionTest, NoCompressionLeavesPlainFiles) {
  const std::string trace_dir = MakeTempTraceDir("no_compress");
  {
    ReplayTraceWriter writer(trace_dir, MakeTestManifest(false, /*chunk_size=*/2U), true);
    for (int i = 0; i < 3; ++i) {
      writer.WriteEvent(MakeEvent("plain-payload"));
    }
  }

  const std::string chunk0_gz    = trace_dir + "/events/000000.events.jsonl.gz";
  const std::string chunk0_plain = trace_dir + "/events/000000.events.jsonl";

  EXPECT_FALSE(FileExists(chunk0_gz))   << "未启用压缩时不应有 .gz 文件";
  EXPECT_TRUE(FileExists(chunk0_plain)) << "未启用压缩时原始文件应存在";
}

// ---------------------------------------------------------------------------
// hash chain 完整性：压缩 trace 的 ScanReplayTrace 应 ok
// ---------------------------------------------------------------------------

TEST(ReplayTraceCompressionTest, CompressedTraceScanIsClean) {
  const std::string trace_dir = MakeTempTraceDir("scan");
  {
    ReplayTraceWriter writer(trace_dir, MakeTestManifest(true, /*chunk_size=*/3U), true);
    for (int i = 0; i < 7; ++i) {
      ReplayTraceEvent ev = MakeEvent("scan-payload");
      ev.has_cycle_index = true;
      ev.cycle_index = static_cast<std::uint32_t>(i);
      writer.WriteEvent(ev);
    }
  }

  const ReplayTraceScanResult scan = ScanReplayTrace(trace_dir);
  EXPECT_TRUE(scan.ok)                    << scan.first_error;
  EXPECT_TRUE(scan.payload_hashes_ok)     << "payload hashes 应全部匹配";
  EXPECT_TRUE(scan.event_chain_ok)        << "事件链应完整";
  EXPECT_TRUE(scan.sequences_contiguous)  << "序号应连续";
  EXPECT_EQ(scan.event_count, 7U);
}
#endif  // ONEQ_HAVE_ZLIB

}  // namespace tests
}  // namespace replay
}  // namespace oneq
