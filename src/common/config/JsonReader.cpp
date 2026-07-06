#include "1q/foundation/json_reader.h"

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace oneq {

namespace {

const JsonValue kNullValue;
constexpr int kMaxJsonDepth = 128;

int HexValue(int c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  return -1;
}

bool IsDigit(int c) {
  return c >= '0' && c <= '9';
}

}  // anonymous namespace

// -- JsonInternalParser (friend of JsonValue, so can access private members) --

struct JsonInternalParser {
  static bool SkipWhitespace(std::istream& in) {
    while (in) {
      const int c = in.peek();
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        in.get();
      } else {
        break;
      }
    }
    return in.good();
  }

  static bool ExpectChar(std::istream& in, char expected, std::string* error) {
    if (!SkipWhitespace(in)) { *error = "unexpected end of input"; return false; }
    const int c = in.get();
    if (c != expected) {
      std::ostringstream oss;
      oss << "expected '" << expected << "' but got '" << static_cast<char>(c) << "'";
      *error = oss.str();
      return false;
    }
    return true;
  }

  static bool ParseString(std::istream& in, std::string* out, std::string* error) {
    if (!ExpectChar(in, '"', error)) return false;
    out->clear();
    while (in) {
      const int c = in.get();
      if (c == '"') return true;
      if (c == '\\') {
        const int esc = in.get();
        switch (esc) {
          case '"':  *out += '"'; break;
          case '\\': *out += '\\'; break;
          case '/':  *out += '/'; break;
          case 'b':  *out += '\b'; break;
          case 'f':  *out += '\f'; break;
          case 'n':  *out += '\n'; break;
          case 'r':  *out += '\r'; break;
          case 't':  *out += '\t'; break;
          case 'u': {
            unsigned long cp = 0;
            for (int i = 0; i < 4; ++i) {
              const int hex_char = in.get();
              const int hex_value = HexValue(hex_char);
              if (hex_value < 0) {
                *error = "invalid \\uXXXX escape";
                return false;
              }
              cp = (cp << 4U) | static_cast<unsigned long>(hex_value);
            }
            if (cp >= 0xD800UL && cp <= 0xDFFFUL) {
              *error = "unsupported UTF-16 surrogate escape";
              return false;
            }
            if (cp <= 0x7F) {
              *out += static_cast<char>(cp);
            } else if (cp <= 0x7FF) {
              *out += static_cast<char>(0xC0 | (cp >> 6));
              *out += static_cast<char>(0x80 | (cp & 0x3F));
            } else {
              *out += static_cast<char>(0xE0 | (cp >> 12));
              *out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
              *out += static_cast<char>(0x80 | (cp & 0x3F));
            }
            break;
          }
          default: *error = "invalid escape character"; return false;
        }
      } else if (c >= 0) {
        *out += static_cast<char>(c);
      } else {
        *error = "unterminated string";
        return false;
      }
    }
    *error = "unterminated string";
    return false;
  }

  static bool ParseNumber(std::istream& in, JsonValue* out, std::string* error) {
    std::string buf;
    if (in.peek() == '-') { buf += static_cast<char>(in.get()); }
    const int first = in.peek();
    if (first == '0') {
      buf += static_cast<char>(in.get());
      if (IsDigit(in.peek())) {
        *error = "invalid number";
        return false;
      }
    } else if (first >= '1' && first <= '9') {
      buf += static_cast<char>(in.get());
      int c;
      while (IsDigit(c = in.peek())) { buf += static_cast<char>(in.get()); }
    } else {
      *error = "invalid number";
      return false;
    }
    bool is_double = false;
    if (in.peek() == '.') {
      is_double = true;
      buf += static_cast<char>(in.get());
      if (!IsDigit(in.peek())) {
        *error = "invalid number";
        return false;
      }
      int c;
      while (IsDigit(c = in.peek())) { buf += static_cast<char>(in.get()); }
    }
    if (in.peek() == 'e' || in.peek() == 'E') {
      is_double = true;
      buf += static_cast<char>(in.get());
      if (in.peek() == '+' || in.peek() == '-') buf += static_cast<char>(in.get());
      if (!IsDigit(in.peek())) {
        *error = "invalid number";
        return false;
      }
      int c;
      while (IsDigit(c = in.peek())) { buf += static_cast<char>(in.get()); }
    }
    if (buf.empty() || buf == "-") { *error = "invalid number"; return false; }
    if (is_double) {
      char* end = nullptr;
      out->double_ = std::strtod(buf.c_str(), &end);
      out->type_ = JsonValue::kDouble;
    } else {
      char* end = nullptr;
      out->int_ = static_cast<std::int64_t>(std::strtoll(buf.c_str(), &end, 10));
      out->type_ = JsonValue::kInt;
    }
    return true;
  }

  static bool ParseArray(std::istream& in, JsonValue* out, std::string* error, int depth) {
    out->type_ = JsonValue::kArray;
    if (!SkipWhitespace(in)) { *error = "unexpected end in array"; return false; }
    if (in.peek() == ']') { in.get(); return true; }
    for (;;) {
      out->children_.push_back(JsonValue());
      if (!Value(in, &out->children_.back(), error, depth + 1)) return false;
      out->keys_.push_back(std::string());
      if (!SkipWhitespace(in)) { *error = "unexpected end in array"; return false; }
      const int c = in.peek();
      if (c == ']') { in.get(); return true; }
      if (c == ',') { in.get(); continue; }
      *error = "expected ',' or ']' in array";
      return false;
    }
  }

  static bool ParseObject(std::istream& in, JsonValue* out, std::string* error, int depth) {
    out->type_ = JsonValue::kObject;
    if (!SkipWhitespace(in)) { *error = "unexpected end in object"; return false; }
    if (in.peek() == '}') { in.get(); return true; }
    for (;;) {
      std::string key;
      if (!ParseString(in, &key, error)) return false;
      if (!ExpectChar(in, ':', error)) return false;
      out->keys_.push_back(key);
      out->children_.push_back(JsonValue());
      if (!Value(in, &out->children_.back(), error, depth + 1)) return false;
      if (!SkipWhitespace(in)) { *error = "unexpected end in object"; return false; }
      const int c = in.peek();
      if (c == '}') { in.get(); return true; }
      if (c == ',') { in.get(); continue; }
      *error = "expected ',' or '}' in object";
      return false;
    }
  }

  static bool Value(std::istream& in, JsonValue* out, std::string* error, int depth) {
    if (depth > kMaxJsonDepth) {
      *error = "maximum JSON nesting depth exceeded";
      return false;
    }
    if (!SkipWhitespace(in)) { *error = "unexpected end of input"; return false; }
    const int c = in.peek();
    if (c == '"') {
      out->type_ = JsonValue::kString;
      return ParseString(in, &out->string_, error);
    }
    if (c == '{') { in.get(); return ParseObject(in, out, error, depth); }
    if (c == '[') { in.get(); return ParseArray(in, out, error, depth); }
    if (c == '-' || (c >= '0' && c <= '9')) return ParseNumber(in, out, error);
    if (c == 't') {
      char buf[5] = {};
      for (int i = 0; i < 4 && in; ++i) buf[i] = static_cast<char>(in.get());
      if (std::string(buf, 4) == "true") { out->type_ = JsonValue::kBool; out->bool_ = true; return true; }
      *error = "invalid token (expected true)";
      return false;
    }
    if (c == 'f') {
      char buf[6] = {};
      for (int i = 0; i < 5 && in; ++i) buf[i] = static_cast<char>(in.get());
      if (std::string(buf, 5) == "false") { out->type_ = JsonValue::kBool; out->bool_ = false; return true; }
      *error = "invalid token (expected false)";
      return false;
    }
    if (c == 'n') {
      if (in.get() != 'n' || in.get() != 'u' || in.get() != 'l' || in.get() != 'l') {
        *error = "invalid token (expected null)";
        return false;
      }
      return true;
    }
    *error = "unexpected character";
    return false;
  }

  static bool Value(std::istream& in, JsonValue* out, std::string* error) {
    return Value(in, out, error, 0);
  }

  static bool ExpectEndOfFile(std::istream& in, std::string* error) {
    for (;;) {
      const int c = in.peek();
      if (c == std::char_traits<char>::eof()) {
        return true;
      }
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        in.get();
        continue;
      }
      *error = "unexpected trailing content after JSON value";
      return false;
    }
  }
};

// -- JsonValue --------------------------------------------------------------

double JsonValue::AsDouble() const {
  return type_ == kDouble ? double_ : static_cast<double>(int_);
}

const JsonValue& JsonValue::operator[](const char* key) const {
  if (type_ != kObject) return kNullValue;
  for (std::size_t i = 0; i < keys_.size(); ++i) {
    if (keys_[i] == key) return children_[i];
  }
  return kNullValue;
}

const JsonValue& JsonValue::operator[](std::size_t index) const {
  if (type_ != kArray || index >= children_.size()) return kNullValue;
  return children_[index];
}

bool JsonValue::Has(const char* key) const {
  if (type_ != kObject) return false;
  for (std::size_t i = 0; i < keys_.size(); ++i) {
    if (keys_[i] == key) return true;
  }
  return false;
}

// -- JsonReader -------------------------------------------------------------

bool JsonReader::ParseFile(const char* path, JsonValue* root, std::string* error) {
  std::ifstream in(path, std::ios::in | std::ios::binary);
  if (!in) {
    *error = std::string("cannot open file: ") + path;
    return false;
  }
  // skip UTF-8 BOM if present
  char bom[3] = {};
  in.read(bom, 3);
  if (static_cast<unsigned char>(bom[0]) != 0xEFU ||
      static_cast<unsigned char>(bom[1]) != 0xBBU ||
      static_cast<unsigned char>(bom[2]) != 0xBFU) {
    in.clear();
    in.seekg(0);
  }
  JsonValue result;
  if (!JsonInternalParser::Value(in, &result, error)) return false;
  if (!JsonInternalParser::ExpectEndOfFile(in, error)) return false;
  *root = result;
  return true;
}

}  // namespace oneq
