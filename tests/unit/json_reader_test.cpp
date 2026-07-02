#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include "1q/foundation/json_reader.h"

namespace oneq {
namespace {

std::string MakeTempJsonPath(const char* prefix) {
  const char* temp_dir = std::getenv("TMPDIR");
  if (temp_dir == nullptr || temp_dir[0] == '\0') {
    temp_dir = "/tmp";
  }

  std::ostringstream stream;
  stream << temp_dir;
  const std::string dir = stream.str();
  if (!dir.empty() && dir[dir.size() - 1] != '/') {
    stream << "/";
  }
  stream << prefix << "-" << std::time(nullptr) << "-"
         << std::chrono::high_resolution_clock::now().time_since_epoch().count() << ".json";
  return stream.str();
}

bool WriteJsonFile(const std::string& path, const std::string& content) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
  output << content;
  return output.good();
}

bool ParseContent(const std::string& content, JsonValue* root, std::string* error) {
  const std::string path = MakeTempJsonPath("oneq-json-reader");
  if (!WriteJsonFile(path, content)) {
    return false;
  }
  const bool parsed = JsonReader::ParseFile(path.c_str(), root, error);
  std::remove(path.c_str());
  return parsed;
}

std::string BuildNestedArrays(int depth) {
  std::string json;
  for (int i = 0; i < depth; ++i) {
    json += '[';
  }
  json += '0';
  for (int i = 0; i < depth; ++i) {
    json += ']';
  }
  return json;
}

TEST(JsonReaderTest, ParsesValidObjectAndRequiresEof) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("{\"answer\":42,\"name\":\"radar\"}", &root, &error)) << error;

  EXPECT_TRUE(root.Has("answer"));
  EXPECT_EQ(root["answer"].AsInt(), 42);
  EXPECT_EQ(root["name"].AsString(), "radar");
}

TEST(JsonReaderTest, RejectsTrailingContent) {
  JsonValue root;
  std::string error;

  EXPECT_FALSE(ParseContent("{\"answer\":42} garbage", &root, &error));
  EXPECT_NE(error.find("trailing"), std::string::npos);
}

TEST(JsonReaderTest, RejectsExcessiveNestingDepth) {
  JsonValue root;
  std::string error;

  EXPECT_FALSE(ParseContent(BuildNestedArrays(160), &root, &error));
  EXPECT_NE(error.find("depth"), std::string::npos);
}

TEST(JsonReaderTest, RejectsTruncatedUnicodeEscape) {
  JsonValue root;
  std::string error;

  EXPECT_FALSE(ParseContent("{\"bad\":\"\\u12\"}", &root, &error));
  EXPECT_NE(error.find("\\uXXXX"), std::string::npos);
}

TEST(JsonReaderTest, RejectsUtf16SurrogateEscape) {
  JsonValue root;
  std::string error;

  EXPECT_FALSE(ParseContent("{\"bad\":\"\\uD800\"}", &root, &error));
  EXPECT_NE(error.find("surrogate"), std::string::npos);
}

}  // namespace
}  // namespace oneq
