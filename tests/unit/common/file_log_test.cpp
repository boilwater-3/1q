// 文件日志后端单测：oneq::logging 迷你 sink + 格式化引擎 + PROJECT_LOG_* 宏路由。
//
// 仅当 PROJECT_LOG_BACKEND_FILE=1（ONEQ_ENABLE_FILE_LOG=ON，Windows 默认）时
// 本文件编译为真实测试；否则为空 TU（守卫与库编译在 Unit.cmake 中同步）。
//
// 测试使用 CWD 下的固定文件名，串行 ctest 无冲突；SetUp/TearDown 清理。

#if defined(PROJECT_LOG_BACKEND_FILE) && PROJECT_LOG_BACKEND_FILE

#include "common/logging/ProjectLog.h"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

namespace {

const char* kTestLogPath = "oneq_file_log_test.log";

// 读取日志文件全部内容（二进制，逐字节）。
std::string ReadAll() {
  std::ifstream in(kTestLogPath, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// 统计行数（按 '\n' 计数）。
std::size_t CountLines(const std::string& content) {
  std::size_t count = 0;
  for (std::size_t i = 0; i < content.size(); ++i) {
    if (content[i] == '\n') {
      ++count;
    }
  }
  return count;
}

class FileLogTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::remove(kTestLogPath);
    oneq::logging::SetFileLogLevel(oneq::logging::Level::kInfo);
    oneq::logging::CloseFileLog();
  }

  void TearDown() override {
    oneq::logging::CloseFileLog();
    std::remove(kTestLogPath);
  }
};

// 打开 → 写入 → 显式 flush → 关闭，文件内容含时间戳前缀、级别标签与消息。
TEST_F(FileLogTest, OpenWriteFlushClose) {
  ASSERT_FALSE(oneq::logging::IsFileLogOpen());
  oneq::logging::OpenFileLog(kTestLogPath);
  ASSERT_TRUE(oneq::logging::IsFileLogOpen());

  oneq::logging::LogInfo("hello world {}", 42);
  oneq::logging::FlushFileLog();
  oneq::logging::CloseFileLog();
  ASSERT_FALSE(oneq::logging::IsFileLogOpen());

  const std::string content = ReadAll();
  EXPECT_NE(content.find("hello world 42"), std::string::npos);
  EXPECT_NE(content.find("[info]"), std::string::npos);
  // 时间戳前缀形态 "[YYYY-MM-DD HH:MM:SS.mmm] ["
  EXPECT_NE(content.find("] ["), std::string::npos);
}

// 默认形态 {} 覆盖常用类型。
TEST_F(FileLogTest, FormatBareTypes) {
  oneq::logging::OpenFileLog(kTestLogPath);
  oneq::logging::LogWarn("i={} d={} s={} b={} c={}",
                         -7, 3.5, std::string("txt"), true, 'x');
  oneq::logging::LogInfo("u={} e={}", static_cast<std::uint64_t>(1234567890123ULL), 'z');
  oneq::logging::CloseFileLog();

  const std::string content = ReadAll();
  EXPECT_NE(content.find("i=-7 d=3.5 s=txt b=true c=x"), std::string::npos);
  EXPECT_NE(content.find("u=1234567890123"), std::string::npos);
}

// 精度形态 {:.Nf} 的定点输出。
TEST_F(FileLogTest, FormatFixedPrecision) {
  oneq::logging::OpenFileLog(kTestLogPath);
  oneq::logging::LogInfo("v={:.0f} w={:.1f} t={:.3f}", 3.7, 2.5, 1.23456);
  oneq::logging::CloseFileLog();

  const std::string content = ReadAll();
  EXPECT_NE(content.find("v=4 w=2.5 t=1.235"), std::string::npos);
}

// 默认最低级别 kInfo：debug 不落盘；调级别后落盘。
TEST_F(FileLogTest, LevelFilter) {
  oneq::logging::OpenFileLog(kTestLogPath);
  oneq::logging::LogDebug("secret-detail");
  oneq::logging::FlushFileLog();
  EXPECT_TRUE(ReadAll().empty());

  oneq::logging::SetFileLogLevel(oneq::logging::Level::kDebug);
  EXPECT_TRUE(PROJECT_LOG_SHOULD_LOG_DEBUG());
  oneq::logging::LogDebug("visible-detail");
  oneq::logging::CloseFileLog();

  EXPECT_NE(ReadAll().find("visible-detail"), std::string::npos);
}

// 多次写入追加为多行。
TEST_F(FileLogTest, AppendMultipleLines) {
  oneq::logging::OpenFileLog(kTestLogPath);
  oneq::logging::LogInfo("line one");
  oneq::logging::LogInfo("line two");
  oneq::logging::LogInfo("line three");
  oneq::logging::CloseFileLog();

  const std::string content = ReadAll();
  EXPECT_EQ(CountLines(content), 3u);
  EXPECT_NE(content.find("line one"), std::string::npos);
  EXPECT_NE(content.find("line two"), std::string::npos);
  EXPECT_NE(content.find("line three"), std::string::npos);
}

// 多线程写入冒烟：4 线程 × 50 条，全部落盘。
TEST_F(FileLogTest, ThreadedWrite) {
  oneq::logging::OpenFileLog(kTestLogPath);
  const int kThreads = 4;
  const int kPerThread = 50;
  std::vector<std::thread> workers;
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([t, kPerThread]() {
      for (int i = 0; i < kPerThread; ++i) {
        oneq::logging::LogInfo("thread {} msg {}", t, i);
      }
    });
  }
  for (auto& worker : workers) {
    worker.join();
  }
  oneq::logging::CloseFileLog();

  const std::string content = ReadAll();
  EXPECT_EQ(CountLines(content), static_cast<std::size_t>(kThreads * kPerThread));
}

// fmt 风格花括号转义：{{ -> {，}} -> }，与 spdlog/fmt 一致。
TEST_F(FileLogTest, BraceEscapes) {
  oneq::logging::OpenFileLog(kTestLogPath);
  oneq::logging::LogInfo("esc {{x}} {} }}", 7);
  oneq::logging::CloseFileLog();

  const std::string content = ReadAll();
  EXPECT_NE(content.find("esc {x} 7 }"), std::string::npos);
}

// 未知占位符形态按字面输出且不崩溃；参数仍与后续占位符对齐。
TEST_F(FileLogTest, UnknownSpecifierLiteral) {
  oneq::logging::OpenFileLog(kTestLogPath);
  oneq::logging::LogInfo("x={:x} y={}", 1, 2);
  oneq::logging::CloseFileLog();

  const std::string content = ReadAll();
  EXPECT_NE(content.find("{:x}"), std::string::npos);
  EXPECT_NE(content.find("y=2"), std::string::npos);
}

// 缺参时剩余占位符按字面输出，不崩溃。
TEST_F(FileLogTest, MissingArgumentLiteral) {
  oneq::logging::OpenFileLog(kTestLogPath);
  oneq::logging::LogInfo("a={} b={}", 1);
  oneq::logging::CloseFileLog();

  const std::string content = ReadAll();
  EXPECT_NE(content.find("a=1"), std::string::npos);
  EXPECT_NE(content.find("b={}"), std::string::npos);
}

// PROJECT_LOG_* 宏在 FILE 分支下的路由：宏调用落盘同一文件。
TEST_F(FileLogTest, MacroRouting) {
  oneq::logging::OpenFileLog(kTestLogPath);
  PROJECT_LOG_INFO("macro routed {}", 5);
  PROJECT_LOG_WARN("macro warn {}", 6.25);
  PROJECT_LOG_FLUSH_DEFAULT();
  oneq::logging::CloseFileLog();

  const std::string content = ReadAll();
  EXPECT_NE(content.find("macro routed 5"), std::string::npos);
  EXPECT_NE(content.find("macro warn 6.25"), std::string::npos);
  EXPECT_NE(content.find("[warn]"), std::string::npos);
}

// 未打开时写入触发懒打开（默认路径）；此处以显式打开路径验证懒打开不误开默认文件。
TEST_F(FileLogTest, LazyOpenUsesExplicitPath) {
  oneq::logging::LogInfo("lazy first write");
  ASSERT_TRUE(oneq::logging::IsFileLogOpen());
  oneq::logging::CloseFileLog();
  // 懒打开走默认路径（CWD/1q_library.log，ONEQ_FILE_LOG_PATH），非测试路径。
  EXPECT_TRUE(ReadAll().empty());
  std::remove("1q_library.log");
}

}  // namespace

#endif  // PROJECT_LOG_BACKEND_FILE
