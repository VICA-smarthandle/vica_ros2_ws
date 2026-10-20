#include "vica_user_guidance_cpp/pyjson.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace vica_guidance
{
namespace
{

void append_utf8(std::string & out, uint32_t cp)
{
  if (cp < 0x80) {
    out.push_back(static_cast<char>(cp));
  } else if (cp < 0x800) {
    out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else if (cp < 0x10000) {
    out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  }
}

class Parser
{
public:
  explicit Parser(const std::string & s) : s_(s) {}

  bool parse_document(JsonValue & out)
  {
    if (s_.size() >= 3 && static_cast<unsigned char>(s_[0]) == 0xEF &&
      static_cast<unsigned char>(s_[1]) == 0xBB && static_cast<unsigned char>(s_[2]) == 0xBF)
    {
      return false;
    }
    skip_ws();
    if (!parse_value(out, 0)) {return false;}
    skip_ws();
    return pos_ == s_.size();
  }

private:
  const std::string & s_;
  std::size_t pos_ = 0;

  bool at_end() const {return pos_ >= s_.size();}
  char peek() const {return s_[pos_];}

  void skip_ws()
  {
    while (!at_end() && (peek() == ' ' || peek() == '\t' || peek() == '\n' || peek() == '\r')) {
      ++pos_;
    }
  }

  bool starts_with(const char * lit) const
  {
    std::size_t k = 0;
    for (; lit[k]; ++k) {
      if (pos_ + k >= s_.size() || s_[pos_ + k] != lit[k]) {return false;}
    }
    return true;
  }

  bool parse_value(JsonValue & out, int depth)
  {
    if (at_end()) {return false;}
    const char c = peek();
    if (c == '"') {out.kind = JsonValue::Kind::String; return parse_string(out.s);}
    if (c == '{') {return parse_object(out, depth + 1);}
    if (c == '[') {return parse_array(out, depth + 1);}
    if (c == 'n' && starts_with("null")) {pos_ += 4; out.kind = JsonValue::Kind::Null; return true;}
    if (c == 't' && starts_with("true")) {
      pos_ += 4; out.kind = JsonValue::Kind::Bool; out.b = true; return true;
    }
    if (c == 'f' && starts_with("false")) {
      pos_ += 5; out.kind = JsonValue::Kind::Bool; out.b = false; return true;
    }
    if (parse_number(out)) {return true;}
    if (c == 'N' && starts_with("NaN")) {
      pos_ += 3; out.kind = JsonValue::Kind::Float; out.d = std::nan(""); return true;
    }
    if (c == 'I' && starts_with("Infinity")) {
      pos_ += 8; out.kind = JsonValue::Kind::Float;
      out.d = std::numeric_limits<double>::infinity(); return true;
    }
    if (c == '-' && starts_with("-Infinity")) {
      pos_ += 9; out.kind = JsonValue::Kind::Float;
      out.d = -std::numeric_limits<double>::infinity(); return true;
    }
    return false;
  }

  static bool is_digit(char c) {return c >= '0' && c <= '9';}

  bool parse_number(JsonValue & out)
  {
    std::size_t p = pos_;
    if (p < s_.size() && s_[p] == '-') {++p;}
    if (p >= s_.size() || !is_digit(s_[p])) {return false;}
    if (s_[p] == '0') {
      ++p;
    } else {
      while (p < s_.size() && is_digit(s_[p])) {++p;}
    }
    bool is_float = false;
    if (p + 1 < s_.size() && s_[p] == '.' && is_digit(s_[p + 1])) {
      p += 2;
      while (p < s_.size() && is_digit(s_[p])) {++p;}
      is_float = true;
    }
    if (p < s_.size() && (s_[p] == 'e' || s_[p] == 'E')) {
      std::size_t q = p + 1;
      if (q < s_.size() && (s_[q] == '+' || s_[q] == '-')) {++q;}
      if (q < s_.size() && is_digit(s_[q])) {
        while (q < s_.size() && is_digit(s_[q])) {++q;}
        p = q;
        is_float = true;
      }
    }
    const std::string text = s_.substr(pos_, p - pos_);
    pos_ = p;
    if (is_float) {
      out.kind = JsonValue::Kind::Float;
      out.d = std::strtod(text.c_str(), nullptr);
      return true;
    }
    out.kind = JsonValue::Kind::Int;
    errno = 0;
    const long long iv = std::strtoll(text.c_str(), nullptr, 10);
    if (errno == ERANGE) {
      out.big_int = true;
      out.d = std::strtod(text.c_str(), nullptr);
    } else {
      out.i = iv;
    }
    return true;
  }

  static int hexval(char c)
  {
    if (c >= '0' && c <= '9') {return c - '0';}
    if (c >= 'a' && c <= 'f') {return c - 'a' + 10;}
    if (c >= 'A' && c <= 'F') {return c - 'A' + 10;}
    return -1;
  }

  bool read_hex4(std::size_t at, uint32_t & v) const
  {
    if (at + 4 > s_.size()) {return false;}
    v = 0;
    for (std::size_t k = 0; k < 4; ++k) {
      const int h = hexval(s_[at + k]);
      if (h < 0) {return false;}
      v = (v << 4) | static_cast<uint32_t>(h);
    }
    return true;
  }

  bool parse_string(std::string & out)
  {
    ++pos_;
    out.clear();
    while (true) {
      if (at_end()) {return false;}
      const auto c = static_cast<unsigned char>(peek());
      if (c == '"') {++pos_; return true;}
      if (c < 0x20) {return false;}
      if (c != '\\') {out.push_back(static_cast<char>(c)); ++pos_; continue;}
      ++pos_;
      if (at_end()) {return false;}
      const char e = peek();
      ++pos_;
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
            uint32_t u = 0;
            if (!read_hex4(pos_, u)) {return false;}
            pos_ += 4;
            if (u >= 0xD800 && u <= 0xDBFF && pos_ + 1 < s_.size() && s_[pos_] == '\\' &&
              s_[pos_ + 1] == 'u')
            {
              uint32_t lo = 0;
              if (read_hex4(pos_ + 2, lo) && lo >= 0xDC00 && lo <= 0xDFFF) {
                u = 0x10000 + (((u - 0xD800) << 10) | (lo - 0xDC00));
                pos_ += 6;
              }
            }
            append_utf8(out, u);
            break;
          }
        default:
          return false;
      }
    }
  }

  bool parse_object(JsonValue & out, int depth)
  {
    if (depth > JSON_MAX_DEPTH) {return false;}
    out.kind = JsonValue::Kind::Object;
    ++pos_;
    skip_ws();
    if (!at_end() && peek() == '}') {++pos_; return true;}
    while (true) {
      if (at_end() || peek() != '"') {return false;}
      std::string key;
      if (!parse_string(key)) {return false;}
      skip_ws();
      if (at_end() || peek() != ':') {return false;}
      ++pos_;
      skip_ws();
      JsonValue v;
      if (!parse_value(v, depth)) {return false;}
      out.o.emplace_back(std::move(key), std::move(v));
      skip_ws();
      if (at_end()) {return false;}
      if (peek() == '}') {++pos_; return true;}
      if (peek() != ',') {return false;}
      ++pos_;
      skip_ws();
    }
  }

  bool parse_array(JsonValue & out, int depth)
  {
    if (depth > JSON_MAX_DEPTH) {return false;}
    out.kind = JsonValue::Kind::Array;
    ++pos_;
    skip_ws();
    if (!at_end() && peek() == ']') {++pos_; return true;}
    while (true) {
      JsonValue v;
      if (!parse_value(v, depth)) {return false;}
      out.a.push_back(std::move(v));
      skip_ws();
      if (at_end()) {return false;}
      if (peek() == ']') {++pos_; return true;}
      if (peek() != ',') {return false;}
      ++pos_;
      skip_ws();
    }
  }
};

}

const JsonValue * JsonValue::get(const std::string & key) const
{
  const JsonValue * found = nullptr;
  for (const auto & kv : o) {
    if (kv.first == key) {found = &kv.second;}
  }
  return found;
}

std::optional<JsonValue> py_json_loads(const std::string & text)
{
  JsonValue v;
  Parser p(text);
  if (!p.parse_document(v)) {return std::nullopt;}
  return v;
}

}
