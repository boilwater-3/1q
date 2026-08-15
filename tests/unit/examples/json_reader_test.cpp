#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include "json_reader.h"
#include "support/oneq_test_temp_dir.h"

namespace examples {
namespace {

std::string MakeTempJsonPath(const char* prefix) {
  std::ostringstream stream;
  stream << oneq_test::TempDir() << prefix << "-" << std::time(nullptr) << "-"
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

// =============================================================================
// 值类型覆盖：array / number / string escapes / bool / null
// =============================================================================

TEST(JsonReaderTest, ParsesArrayOfMixedTypes) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("[1, 2.5, \"three\", true, null]", &root, &error)) << error;

  EXPECT_EQ(root.Size(), 5u);
  EXPECT_EQ(root[static_cast<std::size_t>(0)].AsInt(), 1);
  EXPECT_NEAR(root[static_cast<std::size_t>(1)].AsDouble(), 2.5, 1.0e-9);
  EXPECT_EQ(root[static_cast<std::size_t>(2)].AsString(), "three");
  EXPECT_TRUE(root[static_cast<std::size_t>(3)].AsBool());
  EXPECT_TRUE(root[static_cast<std::size_t>(4)].IsNull());
}

TEST(JsonReaderTest, ParsesEmptyArray) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("[]", &root, &error)) << error;
  EXPECT_EQ(root.Size(), 0u);
}

TEST(JsonReaderTest, ParsesNestedArrays) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("[[1,2],[3,4]]", &root, &error)) << error;
  ASSERT_EQ(root.Size(), 2u);
  EXPECT_EQ(root[static_cast<std::size_t>(0)][static_cast<std::size_t>(1)].AsInt(), 2);
  EXPECT_EQ(root[static_cast<std::size_t>(1)][static_cast<std::size_t>(0)].AsInt(), 3);
}

TEST(JsonReaderTest, RejectsMalformedArrayMissingComma) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("[1 2]", &root, &error));
  EXPECT_NE(error.find("array"), std::string::npos);
}

TEST(JsonReaderTest, ParsesNegativeInteger) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("-42", &root, &error)) << error;
  EXPECT_EQ(root.AsInt(), -42);
}

TEST(JsonReaderTest, ParsesDoubleWithExponent) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("1.5e3", &root, &error)) << error;
  EXPECT_NEAR(root.AsDouble(), 1500.0, 1.0e-6);
}

TEST(JsonReaderTest, ParsesNegativeExponent) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("1e-2", &root, &error)) << error;
  EXPECT_NEAR(root.AsDouble(), 0.01, 1.0e-9);
}

TEST(JsonReaderTest, ParsesIntegerAsDoubleWhenAccessed) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("42", &root, &error)) << error;
  // AsDouble() on kInt should still return a usable double
  EXPECT_NEAR(root.AsDouble(), 42.0, 1.0e-9);
}

TEST(JsonReaderTest, RejectsInvalidNumberLoneMinus) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("-", &root, &error));
  EXPECT_NE(error.find("number"), std::string::npos);
}

TEST(JsonReaderTest, RejectsInvalidNumberTrailingDecimalPoint) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("1.", &root, &error));
  EXPECT_NE(error.find("number"), std::string::npos);
}

TEST(JsonReaderTest, RejectsInvalidNumberMissingExponentDigits) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("1e+", &root, &error));
  EXPECT_NE(error.find("number"), std::string::npos);
}

TEST(JsonReaderTest, RejectsInvalidNumberWithLeadingZero) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("01", &root, &error));
  EXPECT_NE(error.find("number"), std::string::npos);
}

TEST(JsonReaderTest, ParsesStringWithBasicEscapes) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("\"tab\\tend\\nline\"", &root, &error)) << error;
  EXPECT_EQ(root.AsString(), "tab\tend\nline");
}

TEST(JsonReaderTest, ParsesStringWithSolidusAndBackslashEscapes) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("\"path\\\\to\\/file\"", &root, &error)) << error;
  EXPECT_EQ(root.AsString(), "path\\to/file");
}

TEST(JsonReaderTest, ParsesStringWithUnicodeEscape) {
  JsonValue root;
  std::string error;
  // \u00E9 = é (U+00E9), two-byte UTF-8: 0xC3 0xA9
  ASSERT_TRUE(ParseContent("\"caf\\u00E9\"", &root, &error)) << error;
  EXPECT_EQ(root.AsString(), "caf\xc3\xa9");
}

TEST(JsonReaderTest, ParsesStringWithThreeByteUnicodeEscape) {
  JsonValue root;
  std::string error;
  // \u4E2D = 中 (U+4E2D), three-byte UTF-8: 0xE4 0xB8 0xAD
  ASSERT_TRUE(ParseContent("\"\\u4E2D\"", &root, &error)) << error;
  EXPECT_EQ(root.AsString(), "\xe4\xb8\xad");
}

TEST(JsonReaderTest, RejectsInvalidEscapeCharacter) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("\"bad\\xescape\"", &root, &error));
  EXPECT_NE(error.find("escape"), std::string::npos);
}

TEST(JsonReaderTest, RejectsUnterminatedString) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("\"unterminated", &root, &error));
  EXPECT_NE(error.find("string"), std::string::npos);
}

TEST(JsonReaderTest, ParsesTrueAndFalse) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("true", &root, &error)) << error;
  EXPECT_TRUE(root.AsBool());

  JsonValue root2;
  ASSERT_TRUE(ParseContent("false", &root2, &error)) << error;
  EXPECT_FALSE(root2.AsBool());
}

TEST(JsonReaderTest, RejectsMisspelledTrue) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("tru", &root, &error));
  EXPECT_NE(error.find("true"), std::string::npos);
}

TEST(JsonReaderTest, RejectsMisspelledFalse) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("fals", &root, &error));
  EXPECT_NE(error.find("false"), std::string::npos);
}

TEST(JsonReaderTest, ParsesNull) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("null", &root, &error)) << error;
  EXPECT_TRUE(root.IsNull());
}

TEST(JsonReaderTest, RejectsMisspelledNull) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("nul", &root, &error));
  EXPECT_NE(error.find("null"), std::string::npos);
}

TEST(JsonReaderTest, RejectsUnexpectedCharacter) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(ParseContent("@", &root, &error));
  EXPECT_NE(error.find("unexpected"), std::string::npos);
}

// =============================================================================
// JsonValue 访问器：AsDouble / operator[](index) / Has 边界
// =============================================================================

TEST(JsonReaderTest, ArrayIndexOutOfRangeReturnsNull) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("[1, 2, 3]", &root, &error));
  EXPECT_EQ(root[static_cast<std::size_t>(0)].AsInt(), 1);
  // 越界访问返回 kNull
  EXPECT_TRUE(root[static_cast<std::size_t>(5)].IsNull());
}

TEST(JsonReaderTest, IndexOnNonArrayReturnsNull) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("42", &root, &error));
  // 对非数组做 index 返回 kNull
  EXPECT_TRUE(root[static_cast<std::size_t>(0)].IsNull());
}

TEST(JsonReaderTest, KeyLookupOnNonObjectReturnsNull) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("42", &root, &error));
  EXPECT_FALSE(root.Has("key"));
  EXPECT_TRUE(root["key"].IsNull());
}

TEST(JsonReaderTest, HasReturnsFalseForMissingKey) {
  JsonValue root;
  std::string error;
  ASSERT_TRUE(ParseContent("{\"a\":1}", &root, &error));
  EXPECT_TRUE(root.Has("a"));
  EXPECT_FALSE(root.Has("missing"));
}

// =============================================================================
// ParseFile 文件级路径：BOM / 打开失败
// =============================================================================

TEST(JsonReaderTest, ParsesFileWithUtf8Bom) {
  const std::string path = MakeTempJsonPath("oneq-json-bom");
  // 写入 UTF-8 BOM + JSON
  std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
  out << static_cast<char>(0xEF) << static_cast<char>(0xBB) << static_cast<char>(0xBF);
  out << "{\"bom\":true}";
  out.close();

  JsonValue root;
  std::string error;
  ASSERT_TRUE(JsonReader::ParseFile(path.c_str(), &root, &error)) << error;
  EXPECT_TRUE(root.Has("bom"));
  std::remove(path.c_str());
}

TEST(JsonReaderTest, ReturnsErrorForMissingFile) {
  JsonValue root;
  std::string error;
  EXPECT_FALSE(JsonReader::ParseFile("/nonexistent/path/oneq-test-12345.json", &root, &error));
  EXPECT_NE(error.find("cannot open"), std::string::npos);
}

}  // namespace
}  // namespace oneq
