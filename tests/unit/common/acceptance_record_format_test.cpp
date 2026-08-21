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
            "仿真时间=1.000s 仿真周期=1 [验收项：最大探测距离] 验收内容：目标ID=1001 "
            "相对卫星最大探测距离=4500000.0m");
}

TEST_F(AcceptanceRecordFormatTest, WriteAndReadFile) {
  oneq::logging::OpenAcceptanceLog(oneq::logging::AcceptanceChannel::kSbirs, kTestPath);
  ASSERT_TRUE(oneq::logging::IsAcceptanceLogOpen(oneq::logging::AcceptanceChannel::kSbirs));
  oneq::logging::WriteAcceptanceLog(
      oneq::logging::AcceptanceChannel::kSbirs,
      oneq::logging::FormatAcceptanceLine(0.0f, 0U, "初始化时间", "暂无（未做Session建链计时；门限≤100ms）"));
  oneq::logging::FlushAcceptanceLog(oneq::logging::AcceptanceChannel::kSbirs);
  oneq::logging::CloseAcceptanceLog(oneq::logging::AcceptanceChannel::kSbirs);

  std::ifstream in(kTestPath, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("[验收项：初始化时间]"), std::string::npos);
  EXPECT_NE(content.find("暂无"), std::string::npos);
}

}  // namespace
