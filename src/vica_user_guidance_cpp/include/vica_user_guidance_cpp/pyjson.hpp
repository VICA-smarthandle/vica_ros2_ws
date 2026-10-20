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
  int64_t i = 0;
  bool big_int = false;
  double d = 0.0;
  std::string s;
  std::vector<JsonValue> a;
  std::vector<std::pair<std::string, JsonValue>> o;

  const JsonValue * get(const std::string & key) const;
};

constexpr int JSON_MAX_DEPTH = 512;

std::optional<JsonValue> py_json_loads(const std::string & text);

}
