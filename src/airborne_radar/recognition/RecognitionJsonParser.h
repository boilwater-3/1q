/**
 * @file RecognitionJsonParser.h
 * @brief 识别特征数据库专用的轻量 JSON 解析器（库内部）。
 *
 * 仅支持数据库所需子集：对象、数组、字符串、数字、布尔、null。
 * 遵循 contract.md「实现安全与失败语义」#4：最大嵌套深度限制、
 * 顶层 value 后的 EOF 校验、转义完整性校验；解析失败不抛异常，
 * 以状态字符串报告（含出错位置）。
 */

#ifndef AIRBORNE_RADAR_RECOGNITION_RECOGNITION_JSON_PARSER_H_
#define AIRBORNE_RADAR_RECOGNITION_RECOGNITION_JSON_PARSER_H_

#include <cstddef>
#include <string>
#include <vector>

namespace airborne_radar {
namespace recognition {

/**
 * @brief RecognitionJsonValue 轻量 JSON 值树。
 */
class RecognitionJsonValue {
 public:
  enum Type { kNull, kBool, kDouble, kString, kArray, kObject };

  RecognitionJsonValue() = default;

  Type type() const { return type_; }
  bool IsNull() const { return type_ == kNull; }
  bool IsBool() const { return type_ == kBool; }
  bool IsDouble() const { return type_ == kDouble; }
  bool IsString() const { return type_ == kString; }
  bool IsArray() const { return type_ == kArray; }
  bool IsObject() const { return type_ == kObject; }

  bool AsBool() const { return bool_; }
  double AsDouble() const { return double_; }
  const std::string& AsString() const { return string_; }

  /** @brief 对象按键查值；不存在时返回 null 值。 */
  const RecognitionJsonValue& operator[](const char* key) const;
  /** @brief 数组按下标取值；越界时返回 null 值。 */
  const RecognitionJsonValue& operator[](std::size_t index) const;

  std::size_t Size() const { return children_.size(); }
  bool Has(const char* key) const;

  // 内部类型：解析器直接写成员（库内部使用，不构成 public surface）。
  Type type_{kNull};
  bool bool_{false};
  double double_{0.0};
  std::string string_{};
  // 对象：keys_[i] 为成员名，children_[i] 为成员值；数组：仅 children_ 有效。
  std::vector<std::string> keys_{};
  std::vector<RecognitionJsonValue> children_{};
};

/**
 * @brief RecognitionJsonReader 最小递归下降 JSON 解析器（无异常）。
 */
class RecognitionJsonReader {
 public:
  /**
   * @brief 解析 UTF-8 JSON 文本。
   * @param[in] text 输入文本。
   * @param[out] root 解析结果；失败时不保证内容。
   * @param[out] error 失败原因（含字节偏移与说明）。
   * @return 解析成功返回 true。
   */
  static bool Parse(const std::string& text, RecognitionJsonValue* root, std::string* error);

  /**
   * @brief 解析 JSON 文件（先读取，再 Parse）。
   * @param[in] path 文件路径。
   * @param[out] root 解析结果。
   * @param[out] error 失败原因（含路径）。
   * @return 解析成功返回 true。
   */
  static bool ParseFile(const std::string& path, RecognitionJsonValue* root, std::string* error);
};

}  // namespace recognition
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RECOGNITION_RECOGNITION_JSON_PARSER_H_
