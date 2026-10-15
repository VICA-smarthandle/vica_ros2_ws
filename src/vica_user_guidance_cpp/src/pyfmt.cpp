#include "vica_user_guidance_cpp/pyfmt.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace vica_guidance
{
namespace
{

// UTF-8 → 코드 포인트. 깨진 바이트는 하나씩 U+FFFD 로 바꾼다.
std::u32string decode_utf8(const std::string & s)
{
  std::u32string out;
  std::size_t i = 0;
  const std::size_t n = s.size();
  while (i < n) {
    const auto c = static_cast<unsigned char>(s[i]);
    char32_t cp = 0;
    std::size_t len = 0;
    if (c < 0x80) {
      cp = c; len = 1;
    } else if (c >= 0xC2 && c <= 0xDF) {
      cp = c & 0x1F; len = 2;
    } else if (c >= 0xE0 && c <= 0xEF) {
      cp = c & 0x0F; len = 3;
    } else if (c >= 0xF0 && c <= 0xF4) {
      cp = c & 0x07; len = 4;
    } else {
      out.push_back(0xFFFD); ++i; continue;
    }
    if (i + len > n) {out.push_back(0xFFFD); ++i; continue;}
    bool ok = true;
    for (std::size_t k = 1; k < len; ++k) {
      const auto cc = static_cast<unsigned char>(s[i + k]);
      if ((cc & 0xC0) != 0x80) {ok = false; break;}
      cp = (cp << 6) | (cc & 0x3F);
    }
    // 겹쳐 쓴 인코딩·서로게이트·범위 밖은 깨진 것으로 본다.
    if (ok && ((len == 3 && cp < 0x800) || (len == 4 && (cp < 0x10000 || cp > 0x10FFFF)) ||
      (cp >= 0xD800 && cp <= 0xDFFF)))
    {
      ok = false;
    }
    if (!ok) {out.push_back(0xFFFD); ++i; continue;}
    out.push_back(cp);
    i += len;
  }
  return out;
}

void append_utf8(std::string & out, char32_t cp)
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

std::string encode_utf8(const std::u32string & s)
{
  std::string out;
  for (char32_t cp : s) {append_utf8(out, cp);}
  return out;
}

// str.isprintable 의 ASCII 밖 부분. 유니코드 범주 Cc·Cf·Cs·Co·Zl·Zp·Zs(공백 제외)와
// 비문자는 출력 불가다. 아직 배정되지 않은 코드 포인트(Cn)는 표가 없어 출력 가능으로 본다.
bool is_printable_non_ascii(char32_t c)
{
  if (c <= 0xA0) {return false;}              // 0x80~0x9F 제어, 0xA0 줄 안 바꿈 공백
  if (c == 0xAD) {return false;}
  if (c >= 0x600 && c <= 0x605) {return false;}
  if (c == 0x61C || c == 0x6DD || c == 0x70F || c == 0x890 || c == 0x891 || c == 0x8E2) {
    return false;
  }
  if (c == 0x1680 || c == 0x180E) {return false;}
  if (c >= 0x2000 && c <= 0x200F) {return false;}
  if (c >= 0x2028 && c <= 0x202F) {return false;}
  if (c >= 0x205F && c <= 0x2064) {return false;}
  if (c >= 0x2066 && c <= 0x206F) {return false;}
  if (c == 0x3000) {return false;}
  if (c >= 0xD800 && c <= 0xF8FF) {return false;}   // 서로게이트 + 사적 사용
  if (c >= 0xFDD0 && c <= 0xFDEF) {return false;}
  if (c == 0xFEFF) {return false;}
  if (c >= 0xFFF9 && c <= 0xFFFB) {return false;}
  if ((c & 0xFFFE) == 0xFFFE) {return false;}       // 각 평면 끝 두 비문자
  if (c == 0x110BD || c == 0x110CD) {return false;}
  if (c >= 0x13430 && c <= 0x1343F) {return false;}
  if (c >= 0x1BCA0 && c <= 0x1BCA3) {return false;}
  if (c >= 0x1D173 && c <= 0x1D17A) {return false;}
  if (c == 0xE0001 || (c >= 0xE0020 && c <= 0xE007F)) {return false;}
  if (c >= 0xF0000) {return false;}                 // 사적 사용 평면 15·16
  return true;
}

// str.isspace — 양방향 범주 WS·B·S 이거나 일반 범주 Zs.
bool is_py_space(char32_t c)
{
  return (c >= 0x09 && c <= 0x0D) || (c >= 0x1C && c <= 0x20) || c == 0x85 || c == 0xA0 ||
         c == 0x1680 || (c >= 0x2000 && c <= 0x200A) || c == 0x2028 || c == 0x2029 ||
         c == 0x202F || c == 0x205F || c == 0x3000;
}

// 정수값 double 의 정확한 십진 글자(크기 제한 없음).
std::string exact_integer_decimal(double r)
{
  if (std::fabs(r) < 9.0e18) {
    return std::to_string(static_cast<int64_t>(r));
  }
  int exp = 0;
  const double m = std::frexp(std::fabs(r), &exp);       // |r| = m × 2^exp, 0.5 <= m < 1
  auto mant = static_cast<uint64_t>(std::ldexp(m, 53));   // 53 비트 정수
  int shift = exp - 53;                                   // |r| = mant × 2^shift, shift > 0
  std::vector<uint32_t> limbs;                            // 1e9 진법, 낮은 자리부터
  while (mant) {limbs.push_back(static_cast<uint32_t>(mant % 1000000000ULL)); mant /= 1000000000ULL;}
  for (; shift > 0; --shift) {
    uint64_t carry = 0;
    for (auto & limb : limbs) {
      const uint64_t v = static_cast<uint64_t>(limb) * 2 + carry;
      limb = static_cast<uint32_t>(v % 1000000000ULL);
      carry = v / 1000000000ULL;
    }
    if (carry) {limbs.push_back(static_cast<uint32_t>(carry));}
  }
  std::string out = r < 0 ? "-" : "";
  out += std::to_string(limbs.back());
  char buf[16];
  for (std::size_t k = limbs.size() - 1; k-- > 0; ) {
    std::snprintf(buf, sizeof(buf), "%09u", limbs[k]);
    out += buf;
  }
  return out;
}

}  // namespace

std::string py_float(double x)
{
  if (std::isnan(x)) {return "nan";}
  if (std::isinf(x)) {return x > 0 ? "inf" : "-inf";}
  if (x == 0.0) {return std::signbit(x) ? "-0.0" : "0.0";}
  char buf[64];
  const auto res = std::to_chars(buf, buf + sizeof(buf), std::fabs(x), std::chars_format::scientific);
  const std::string s(buf, res.ptr);                  // "d[.ddd]e±XX" — 가장 짧은 자릿수
  const auto epos = s.find('e');
  std::string digits;
  for (std::size_t k = 0; k < epos; ++k) {
    if (s[k] != '.') {digits.push_back(s[k]);}
  }
  const int e10 = std::stoi(s.substr(epos + 1));
  const int decpt = e10 + 1;                           // 첫 자리 앞 소수점 위치
  std::string out;
  if (decpt > -4 && decpt <= 16) {
    if (decpt <= 0) {
      out = "0." + std::string(static_cast<std::size_t>(-decpt), '0') + digits;
    } else if (static_cast<std::size_t>(decpt) >= digits.size()) {
      out = digits + std::string(static_cast<std::size_t>(decpt) - digits.size(), '0') + ".0";
    } else {
      out = digits.substr(0, static_cast<std::size_t>(decpt)) + "." +
        digits.substr(static_cast<std::size_t>(decpt));
    }
  } else {
    out = digits.substr(0, 1);
    if (digits.size() > 1) {out += "." + digits.substr(1);}
    char eb[16];
    std::snprintf(eb, sizeof(eb), "e%c%02d", e10 < 0 ? '-' : '+', std::abs(e10));
    out += eb;
  }
  return x < 0 ? "-" + out : out;
}

std::string py_str_repr(const std::string & utf8)
{
  const std::u32string s = decode_utf8(utf8);
  bool squote = false, dquote = false;
  for (char32_t c : s) {
    if (c == U'\'') {squote = true;} else if (c == U'"') {dquote = true;}
  }
  const char32_t quote = (squote && !dquote) ? U'"' : U'\'';
  std::string out;
  append_utf8(out, quote);
  char buf[16];
  for (char32_t c : s) {
    if (c == quote || c == U'\\') {
      out.push_back('\\'); append_utf8(out, c);
    } else if (c == U'\t') {
      out += "\\t";
    } else if (c == U'\n') {
      out += "\\n";
    } else if (c == U'\r') {
      out += "\\r";
    } else if (c < 0x20 || c == 0x7F) {
      std::snprintf(buf, sizeof(buf), "\\x%02x", static_cast<unsigned>(c)); out += buf;
    } else if (c < 0x7F) {
      out.push_back(static_cast<char>(c));
    } else if (is_printable_non_ascii(c)) {
      append_utf8(out, c);
    } else if (c <= 0xFF) {
      std::snprintf(buf, sizeof(buf), "\\x%02x", static_cast<unsigned>(c)); out += buf;
    } else if (c <= 0xFFFF) {
      std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c)); out += buf;
    } else {
      std::snprintf(buf, sizeof(buf), "\\U%08x", static_cast<unsigned>(c)); out += buf;
    }
  }
  append_utf8(out, quote);
  return out;
}

std::string py_list_repr(const std::vector<std::string> & items)
{
  std::string out = "[";
  for (std::size_t k = 0; k < items.size(); ++k) {
    if (k) {out += ", ";}
    out += py_str_repr(items[k]);
  }
  return out + "]";
}

std::string py_list_repr(const std::vector<double> & items)
{
  std::string out = "[";
  for (std::size_t k = 0; k < items.size(); ++k) {
    if (k) {out += ", ";}
    out += py_float(items[k]);
  }
  return out + "]";
}

std::string py_int_list_repr(const std::vector<std::string> & decimal_items)
{
  std::string out = "[";
  for (std::size_t k = 0; k < decimal_items.size(); ++k) {
    if (k) {out += ", ";}
    out += decimal_items[k];
  }
  return out + "]";
}

std::string py_slice(const std::string & utf8, std::size_t n)
{
  std::u32string s = decode_utf8(utf8);
  if (s.size() > n) {s.resize(n);}
  return encode_utf8(s);
}

std::string py_strip(const std::string & utf8)
{
  const std::u32string s = decode_utf8(utf8);
  std::size_t b = 0, e = s.size();
  while (b < e && is_py_space(s[b])) {++b;}
  while (e > b && is_py_space(s[e - 1])) {--e;}
  return encode_utf8(s.substr(b, e - b));
}

std::string py_lower_for_lookup(const std::string & utf8)
{
  std::u32string s = decode_utf8(utf8);
  for (auto & c : s) {
    if (c >= U'A' && c <= U'Z') {c = c - U'A' + U'a';} else if (c == 0x212A) {c = U'k';}
  }
  return encode_utf8(s);
}

int64_t py_int_saturating(double x)
{
  if (std::isnan(x)) {throw PyError("ValueError", "cannot convert float NaN to integer");}
  if (std::isinf(x)) {throw PyError("OverflowError", "cannot convert float infinity to integer");}
  const double t = std::trunc(x);
  if (t >= 9223372036854775807.0) {return std::numeric_limits<int64_t>::max();}
  if (t < -9223372036854775807.0) {return std::numeric_limits<int64_t>::min();}
  return static_cast<int64_t>(t);
}

std::string py_round_decimal(double x)
{
  if (std::isnan(x)) {throw PyError("ValueError", "cannot convert float NaN to integer");}
  if (std::isinf(x)) {throw PyError("OverflowError", "cannot convert float infinity to integer");}
  // CPython float.__round__(None): round() 뒤 정확히 반이면 짝수 쪽으로.
  double r = std::round(x);
  if (std::fabs(x - r) == 0.5) {r = 2.0 * std::round(x / 2.0);}
  if (r == 0.0) {return "0";}
  return exact_integer_decimal(r);
}

int64_t sec_to_ns(double seconds)
{
  return py_int_saturating(seconds * 1000000000.0);
}

}  // namespace vica_guidance
