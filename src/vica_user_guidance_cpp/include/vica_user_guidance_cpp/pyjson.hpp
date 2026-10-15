// 파이썬 json.loads 와 같은 문법의 JSON 읽기 — /vica_goal_event 판정과 대조 시험 파라미터용.
//
// 일반 C++ JSON 라이브러리를 쓰지 않는 이유: 미션은 json.dumps 로 보내고, 그 기본값은 NaN·
// Infinity 를 그대로 쓴다(allow_nan=True). 엄격한 파서는 그런 도착 이벤트를 "파싱 실패"로 버려
// 손잡이 도착 표시가 조용히 사라진다. 그래서 파이썬 json 모듈의 문법을 그대로 따른다:
//   공백 " \t\n\r", NaN·Infinity·-Infinity, 같은 키는 뒤의 값, 끝의 남는 글자·앞의 BOM·
//   문자열 안 제어 문자(< 0x20)·뒤따르는 쉼표는 오류.
// 다른 점 하나: 파이썬은 약 1000 겹 넘게 중첩되면 RecursionError 로 노드가 죽는다. 여기서는
// MAX_DEPTH 를 넘으면 파싱 실패(경고 한 줄)로 본다.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace vica_guidance
{

struct JsonValue
{
  enum class Kind {Null, Bool, Int, Float, String, Array, Object};
  Kind kind = Kind::Null;
  bool b = false;
  int64_t i = 0;          // Int 이고 int64 에 들어갈 때
  bool big_int = false;   // Int 인데 int64 를 넘음(파이썬은 그대로 정수)
  double d = 0.0;         // Float
  std::string s;          // String(UTF-8)
  std::vector<JsonValue> a;
  std::vector<std::pair<std::string, JsonValue>> o;   // 들어온 순서 그대로(같은 키 여러 번 가능)

  // 파이썬 dict.get(key) — 같은 키가 여러 번이면 마지막 값.
  const JsonValue * get(const std::string & key) const;
};

constexpr int JSON_MAX_DEPTH = 512;

// json.loads(text). 실패(JSONDecodeError)면 std::nullopt.
std::optional<JsonValue> py_json_loads(const std::string & text);

}  // namespace vica_guidance
