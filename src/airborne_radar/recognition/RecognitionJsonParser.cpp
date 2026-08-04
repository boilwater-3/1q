/**
 * @file RecognitionJsonParser.cpp
 * @brief 识别特征数据库 JSON 解析器实现。
 */

#include "airborne_radar/recognition/RecognitionJsonParser.h"

#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace airborne_radar {
namespace recognition {

namespace {

/** @brief 最大嵌套深度（contract.md 自研解析器约束）。 */
constexpr std::size_t kMaxNestingDepth = 64U;

struct ParserState {
  const std::string& text;
  std::size_t index{0U};
  std::string error{};
};

bool SkipWhitespace(ParserState* state) {
  while (state->index < state->text.size()) {
    const char c = state->text[state->index];
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
      ++state->index;
    } else {
      break;
    }
  }
  return state->index < state->text.size();
}

bool ParseValue(ParserState* state, std::size_t depth, RecognitionJsonValue* out);

bool ParseString(ParserState* state, std::string* out) {
  if (state->index >= state->text.size() || state->text[state->index] != '"') {
    state->error = "expected string at offset " + std::to_string(state->index);
    return false;
  }
  ++state->index;
  std::string result;
  while (state->index < state->text.size()) {
    const char c = state->text[state->index];
    if (c == '"') {
      ++state->index;
      *out = result;
      return true;
    }
    if (c == '\\') {
      ++state->index;
      if (state->index >= state->text.size()) {
        state->error = "unterminated escape at offset " + std::to_string(state->index);
        return false;
      }
      const char escaped = state->text[state->index];
      switch (escaped) {
        case '"':
          result.push_back('"');
          break;
        case '\\':
          result.push_back('\\');
          break;
        case '/':
          result.push_back('/');
          break;
        case 'b':
          result.push_back('\b');
          break;
        case 'f':
          result.push_back('\f');
          break;
        case 'n':
          result.push_back('\n');
          break;
        case 'r':
          result.push_back('\r');
          break;
        case 't':
          result.push_back('\t');
          break;
        case 'u': {
          // 接受 \uXXXX 四字符转义：原样保留（UTF-8 输入下代理对原样透传）。
          if (state->index + 4 >= state->text.size()) {
            state->error = "truncated \\u escape at offset " + std::to_string(state->index);
            return false;
          }
          result.append(state->text, state->index - 1, 6);
          state->index += 4;  // 跳过 4 个十六进制位；循环末尾 ++index 跳过 'u' 后的位置
          break;
        }
        default:
          state->error = "invalid escape \\" + std::string(1, escaped) + " at offset " +
                         std::to_string(state->index);
          return false;
      }
      ++state->index;
      continue;
    }
    if (static_cast<unsigned char>(c) < 0x20U) {
      state->error = "unescaped control character in string at offset " +
                     std::to_string(state->index);
      return false;
    }
    result.push_back(c);
    ++state->index;
  }
  state->error = "unterminated string at offset " + std::to_string(state->index);
  return false;
}

bool ParseNumber(ParserState* state, double* out) {
  const std::size_t start = state->index;
  if (state->index < state->text.size() && state->text[state->index] == '-') {
    ++state->index;
  }
  bool has_digits = false;
  while (state->index < state->text.size() &&
         std::isdigit(static_cast<unsigned char>(state->text[state->index]))) {
    ++state->index;
    has_digits = true;
  }
  if (!has_digits) {
    state->error = "invalid number at offset " + std::to_string(start);
    return false;
  }
  if (state->index < state->text.size() && state->text[state->index] == '.') {
    ++state->index;
    bool has_fraction = false;
    while (state->index < state->text.size() &&
           std::isdigit(static_cast<unsigned char>(state->text[state->index]))) {
      ++state->index;
      has_fraction = true;
    }
    if (!has_fraction) {
      state->error = "invalid fraction at offset " + std::to_string(start);
      return false;
    }
  }
  if (state->index < state->text.size() &&
      (state->text[state->index] == 'e' || state->text[state->index] == 'E')) {
    ++state->index;
    if (state->index < state->text.size() &&
        (state->text[state->index] == '+' || state->text[state->index] == '-')) {
      ++state->index;
    }
    bool has_exponent_digits = false;
    while (state->index < state->text.size() &&
           std::isdigit(static_cast<unsigned char>(state->text[state->index]))) {
      ++state->index;
      has_exponent_digits = true;
    }
    if (!has_exponent_digits) {
      state->error = "invalid exponent at offset " + std::to_string(start);
      return false;
    }
  }
  const std::string token = state->text.substr(start, state->index - start);
  char* end = nullptr;
  *out = std::strtod(token.c_str(), &end);
  if (end == nullptr || *end != '\0') {
    state->error = "invalid number token at offset " + std::to_string(start);
    return false;
  }
  return true;
}

bool ParseArray(ParserState* state, std::size_t depth, RecognitionJsonValue* out) {
  if (state->index >= state->text.size() || state->text[state->index] != '[') {
    state->error = "expected '[' at offset " + std::to_string(state->index);
    return false;
  }
  ++state->index;
  out->type_ = RecognitionJsonValue::kArray;
  if (SkipWhitespace(state) && state->text[state->index] == ']') {
    ++state->index;
    return true;
  }
  while (true) {
    RecognitionJsonValue element;
    if (!ParseValue(state, depth + 1U, &element)) {
      return false;
    }
    out->children_.push_back(std::move(element));
    if (!SkipWhitespace(state)) {
      state->error = "unterminated array at offset " + std::to_string(state->index);
      return false;
    }
    const char c = state->text[state->index];
    if (c == ',') {
      ++state->index;
      continue;
    }
    if (c == ']') {
      ++state->index;
      return true;
    }
    state->error = "expected ',' or ']' at offset " + std::to_string(state->index);
    return false;
  }
}

bool ParseObject(ParserState* state, std::size_t depth, RecognitionJsonValue* out) {
  if (state->index >= state->text.size() || state->text[state->index] != '{') {
    state->error = "expected '{' at offset " + std::to_string(state->index);
    return false;
  }
  ++state->index;
  out->type_ = RecognitionJsonValue::kObject;
  if (SkipWhitespace(state) && state->text[state->index] == '}') {
    ++state->index;
    return true;
  }
  while (true) {
    if (!SkipWhitespace(state)) {
      state->error = "unterminated object at offset " + std::to_string(state->index);
      return false;
    }
    std::string key;
    if (!ParseString(state, &key)) {
      state->error = "expected object key: " + state->error;
      return false;
    }
    if (!SkipWhitespace(state) || state->text[state->index] != ':') {
      state->error = "expected ':' after key at offset " + std::to_string(state->index);
      return false;
    }
    ++state->index;
    RecognitionJsonValue element;
    if (!ParseValue(state, depth + 1U, &element)) {
      return false;
    }
    out->keys_.push_back(std::move(key));
    out->children_.push_back(std::move(element));
    if (!SkipWhitespace(state)) {
      state->error = "unterminated object at offset " + std::to_string(state->index);
      return false;
    }
    const char c = state->text[state->index];
    if (c == ',') {
      ++state->index;
      continue;
    }
    if (c == '}') {
      ++state->index;
      return true;
    }
    state->error = "expected ',' or '}' at offset " + std::to_string(state->index);
    return false;
  }
}

bool ParseValue(ParserState* state, std::size_t depth, RecognitionJsonValue* out) {
  if (depth > kMaxNestingDepth) {
    state->error = "nesting depth exceeds " + std::to_string(kMaxNestingDepth);
    return false;
  }
  if (!SkipWhitespace(state)) {
    state->error = "unexpected end of input at offset " + std::to_string(state->index);
    return false;
  }
  const char c = state->text[state->index];
  switch (c) {
    case '{':
      return ParseObject(state, depth, out);
    case '[':
      return ParseArray(state, depth, out);
    case '"': {
      out->type_ = RecognitionJsonValue::kString;
      return ParseString(state, &out->string_);
    }
    case 't':
      if (state->text.compare(state->index, 4, "true") == 0) {
        state->index += 4;
        out->type_ = RecognitionJsonValue::kBool;
        out->bool_ = true;
        return true;
      }
      state->error = "invalid token at offset " + std::to_string(state->index);
      return false;
    case 'f':
      if (state->text.compare(state->index, 5, "false") == 0) {
        state->index += 5;
        out->type_ = RecognitionJsonValue::kBool;
        out->bool_ = false;
        return true;
      }
      state->error = "invalid token at offset " + std::to_string(state->index);
      return false;
    case 'n':
      if (state->text.compare(state->index, 4, "null") == 0) {
        state->index += 4;
        out->type_ = RecognitionJsonValue::kNull;
        return true;
      }
      state->error = "invalid token at offset " + std::to_string(state->index);
      return false;
    default:
      if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
        out->type_ = RecognitionJsonValue::kDouble;
        return ParseNumber(state, &out->double_);
      }
      state->error = "unexpected character at offset " + std::to_string(state->index);
      return false;
  }
}

}  // namespace

const RecognitionJsonValue& RecognitionJsonValue::operator[](const char* key) const {
  static const RecognitionJsonValue kNullValue;
  if (type_ != kObject) {
    return kNullValue;
  }
  for (std::size_t i = 0U; i < keys_.size(); ++i) {
    if (keys_[i] == key) {
      return children_[i];
    }
  }
  return kNullValue;
}

const RecognitionJsonValue& RecognitionJsonValue::operator[](std::size_t index) const {
  static const RecognitionJsonValue kNullValue;
  if (type_ != kArray || index >= children_.size()) {
    return kNullValue;
  }
  return children_[index];
}

bool RecognitionJsonValue::Has(const char* key) const {
  if (type_ != kObject) {
    return false;
  }
  for (std::size_t i = 0U; i < keys_.size(); ++i) {
    if (keys_[i] == key) {
      return true;
    }
  }
  return false;
}

bool RecognitionJsonReader::Parse(const std::string& text, RecognitionJsonValue* root,
                                  std::string* error) {
  if (root == nullptr) {
    if (error != nullptr) {
      *error = "null root output";
    }
    return false;
  }
  ParserState state{text, 0U, {}};
  RecognitionJsonValue parsed;
  if (!ParseValue(&state, 0U, &parsed)) {
    if (error != nullptr) {
      *error = state.error;
    }
    return false;
  }
  // 顶层 value 后的 EOF 校验：不允许尾随内容。
  if (SkipWhitespace(&state)) {
    if (error != nullptr) {
      *error = "trailing content after top-level value at offset " +
               std::to_string(state.index);
    }
    return false;
  }
  *root = std::move(parsed);
  return true;
}

bool RecognitionJsonReader::ParseFile(const std::string& path, RecognitionJsonValue* root,
                                      std::string* error) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    if (error != nullptr) {
      *error = "cannot open JSON file: " + path;
    }
    return false;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  if (!stream.good() && !stream.eof()) {
    if (error != nullptr) {
      *error = "failed reading JSON file: " + path;
    }
    return false;
  }
  const std::string text = buffer.str();
  if (text.empty()) {
    if (error != nullptr) {
      *error = "empty JSON file: " + path;
    }
    return false;
  }
  return Parse(text, root, error);
}

}  // namespace recognition
}  // namespace airborne_radar
