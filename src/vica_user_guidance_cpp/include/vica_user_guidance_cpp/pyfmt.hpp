// 파이썬 판 로그와 글자까지 같게 찍기 위한 도우미 — repr(float)·repr(str)·int(float)·round().
//
// 파이썬 판의 로그·예외 문구에는 f-string 이 만든 글자가 그대로 들어간다(예: "[0.4, 1.5]",
// "payload='…'"). 같은 글자를 내야 bag 의 /rosout 분석·대조 시험이 두 판에서 똑같이 돈다.
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace vica_guidance
{

// 파이썬에서라면 프로세스를 멈췄을 예외. type 은 "ValueError" 같은 파이썬 예외 이름이다.
// 노드는 이것을 잡아 "<type>: <msg>" 를 stderr 에 찍고 종료 코드 1 로 끝낸다(파이썬 트레이스백의
// 마지막 줄과 같은 글자).
class PyError : public std::runtime_error
{
public:
  PyError(std::string type, const std::string & message)
  : std::runtime_error(message), type_(std::move(type)) {}
  const std::string & type() const {return type_;}

private:
  std::string type_;
};

// repr(x) / str(x) / f"{x}" — 가장 짧게 되돌아오는 자릿수, 1e16 이상·1e-4 미만은 지수 표기.
std::string py_float(double x);

// repr(s) — 따옴표 고르기, \\ \n \r \t \xNN \uNNNN \UNNNNNNNN.
// 출력 가능 판정(str.isprintable)은 흔한 비출력 범위(제어·서식·공백·사적 사용·비문자)만 맞춘다.
std::string py_str_repr(const std::string & utf8);

// repr(list[str]) / repr(list[float]) / repr(list[int]).
std::string py_list_repr(const std::vector<std::string> & items);
std::string py_list_repr(const std::vector<double> & items);
std::string py_int_list_repr(const std::vector<std::string> & decimal_items);

// s[:n] — 바이트가 아니라 코드 포인트로 자른다.
std::string py_slice(const std::string & utf8, std::size_t n);

// s.strip() — str.isspace 인 문자(유니코드 공백 포함)를 양끝에서 걷어낸다.
std::string py_strip(const std::string & utf8);

// s.lower() 를 햅틱 패턴 이름 비교용으로만: ASCII 와, 소문자가 ASCII 가 되는 유일한 문자
// U+212A(켈빈 기호 → 'k')만 바꾼다. 다른 글자는 어떻게 바뀌어도 패턴 이름과 같아질 수 없다.
std::string py_lower_for_lookup(const std::string & utf8);

// int(x) — 0 쪽으로 자른다. NaN 은 ValueError, ±inf 는 OverflowError(파이썬과 같은 문구).
// 파이썬 int 는 크기 제한이 없지만 여기 쓰이는 비교(0 <= age <= timeout)에서는 int64 로
// 포화시켜도 결과가 같다.
int64_t py_int_saturating(double x);

// round(x) 의 십진 글자(짝수 쪽 반올림, 크기 제한 없음). NaN·inf 는 int() 와 같은 예외.
std::string py_round_decimal(double x);

// timebase.sec_to_ns(seconds) = int(seconds * 1_000_000_000)
int64_t sec_to_ns(double seconds);

}  // namespace vica_guidance
