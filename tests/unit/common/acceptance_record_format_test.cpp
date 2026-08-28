// 验收日志一行四段格式与文件 sink。

#include "common/logging/AcceptanceFileLog.h"
#include "common/logging/AcceptanceRecordFormat.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

#include "gtest/gtest.h"

namespace {

const char* kTestPath = "oneq_acceptance_format_test.log";

class AcceptanceRecordFormatTest : public ::testing::Test {
 protected:
  void TearDown() override {
    oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kSbirs);
    std::remove(kTestPath);
  }
};

TEST_F(AcceptanceRecordFormatTest, FourFieldLineShape) {
  const std::string line =
      oneq::logging::FormatAcceptanceLine(1.0f, 1U, "最大探测距离", "目标ID=1001 相对卫星最大探测距离=4500000.0m");
  EXPECT_EQ(line,
            "仿真时间=1.000s 仿真周期=1 [最大探测距离] 验收内容：目标ID=1001 "
            "相对卫星最大探测距离=4500000.0m");
}

TEST_F(AcceptanceRecordFormatTest, WriteAndReadFile) {
  oneq::logging::OpenAcceptanceLog(oneq::logging::AcceptanceChannel::kSbirs, kTestPath);
  ASSERT_TRUE(oneq::logging::IsAcceptanceLogOpen(oneq::logging::AcceptanceChannel::kSbirs));
  oneq::logging::WriteAcceptanceLog(
      oneq::logging::AcceptanceChannel::kSbirs,
      oneq::logging::FormatAcceptanceLine(0.0f, 0U, "初始化时间", "见integration_events.log"));
  oneq::logging::FlushAcceptanceLog(oneq::logging::AcceptanceChannel::kSbirs);
  oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kSbirs);

  std::ifstream in(kTestPath, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
#if defined(_MSC_VER)
  EXPECT_NE(content.find(u8"[初始化时间]"), std::string::npos);
  EXPECT_NE(content.find(u8"integration_events"), std::string::npos);
#else
  EXPECT_NE(content.find("[初始化时间]"), std::string::npos);
  EXPECT_NE(content.find("integration_events"), std::string::npos);
#endif
}

// 覆写语义：进程内首开截断旧文件（重跑进程不向归档证据追加重复行），
// 进程内重开追加（会话重启不丢已写行）。用独立通道 kFusion 保证本用例
// 拥有该 sink 的进程内首开。
TEST_F(AcceptanceRecordFormatTest, FirstOpenTruncatesThenReopenAppends) {
  const char* rerun_path = "oneq_acceptance_rerun_test.log";
  {
    std::ofstream stale(rerun_path, std::ios::binary);
    stale << "STALE_ROW_FROM_PREVIOUS_PROCESS\n";
  }
  const oneq::logging::AcceptanceChannel ch = oneq::logging::AcceptanceChannel::kFusion;
  oneq::logging::OpenAcceptanceLog(ch, rerun_path);
  oneq::logging::WriteAcceptanceLog(
      ch, oneq::logging::FormatAcceptanceLine(1.0f, 1U, "首开行", "A"));
  oneq::logging::CloseAcceptanceLog(ch);

  std::ifstream first(rerun_path, std::ios::binary);
  std::string after_first((std::istreambuf_iterator<char>(first)),
                          std::istreambuf_iterator<char>());
  EXPECT_EQ(after_first.find("STALE_ROW_FROM_PREVIOUS_PROCESS"), std::string::npos);
#if defined(_MSC_VER)
  EXPECT_NE(after_first.find(u8"[首开行]"), std::string::npos);
#else
  EXPECT_NE(after_first.find("[首开行]"), std::string::npos);
#endif

  oneq::logging::OpenAcceptanceLog(ch, rerun_path);
  oneq::logging::WriteAcceptanceLog(
      ch, oneq::logging::FormatAcceptanceLine(2.0f, 2U, "重开行", "B"));
  oneq::logging::CloseAcceptanceLog(ch);

  std::ifstream second(rerun_path, std::ios::binary);
  std::string after_reopen((std::istreambuf_iterator<char>(second)),
                           std::istreambuf_iterator<char>());
#if defined(_MSC_VER)
  EXPECT_NE(after_reopen.find(u8"[首开行]"), std::string::npos);
  EXPECT_NE(after_reopen.find(u8"[重开行]"), std::string::npos);
#else
  EXPECT_NE(after_reopen.find("[首开行]"), std::string::npos);
  EXPECT_NE(after_reopen.find("[重开行]"), std::string::npos);
#endif
  std::remove(rerun_path);
}

}  // namespace
