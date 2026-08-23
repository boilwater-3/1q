#ifndef EXAMPLES_COMMON_JSON_READER_H_
#define EXAMPLES_COMMON_JSON_READER_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace examples {

/// Lightweight JSON value tree. Supports the subset of JSON needed for config
/// loading: objects, arrays, strings, numbers (int/double), booleans, null.
///
/// 本工具属于 examples 层，不是 oneq 库的 public surface。库内部不消费 JSON；
/// 配置文件解析是 example 便利层的能力。详见 docs/common/open_questions.md
/// 的 OQ-7 收口记录。
class JsonValue {
 public:
  enum Type { kNull, kBool, kInt, kDouble, kString, kArray, kObject };

  JsonValue() : type_(kNull), bool_(false), int_(0), double_(0.0) {}

  Type type() const { return type_; }
  bool IsNull() const { return type_ == kNull; }
  bool IsBool() const { return type_ == kBool; }
  bool IsInt() const { return type_ == kInt; }
  bool IsDouble() const { return type_ == kDouble; }
  bool IsString() const { return type_ == kString; }

  bool AsBool() const { return bool_; }
  std::int64_t AsInt() const { return int_; }
  double AsDouble() const;
  const std::string& AsString() const { return string_; }

  /// Look up a key in an object. Returns null value if not found.
  const JsonValue& operator[](const char* key) const;
  /// Index into an array. Returns null value if out of range.
  const JsonValue& operator[](std::size_t index) const;

  std::size_t Size() const { return children_.size(); }
  bool Has(const char* key) const;
  /// Object member names in insertion order (empty for non-objects)——schema
  /// 校验（未知键拒绝）遍历用。
  const std::vector<std::string>& Keys() const { return keys_; }

 private:
  friend class JsonReader;
  friend struct JsonInternalParser;

  Type type_;
  bool bool_;
  std::int64_t int_;
  double double_;
  std::string string_;
  // For objects: keys_[i] = member name, children_[i] = member value.
  // For arrays:  children_[i] = element value, keys_ unused.
  std::vector<std::string> keys_;
  std::vector<JsonValue> children_;
};

/// Minimal recursive-descent JSON parser. No exceptions; errors via status string.
class JsonReader {
 public:
  /// Parse a UTF-8 JSON file. Returns false and sets @p error on failure.
  static bool ParseFile(const char* path, JsonValue* root, std::string* error);
};

}  // namespace examples

#endif  // EXAMPLES_COMMON_JSON_READER_H_
