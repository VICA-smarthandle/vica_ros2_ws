#include "vica_user_guidance_cpp/guidance.hpp"

#include "vica_user_guidance_cpp/pyjson.hpp"

namespace vica_guidance
{

std::optional<std::string> parse_goal_event(const std::string & payload)
{
  const auto doc = py_json_loads(payload);
  if (!doc || doc->kind != JsonValue::Kind::Object) {return std::nullopt;}
  const JsonValue * event = doc->get("event");
  if (!event || event->kind != JsonValue::Kind::String) {return std::nullopt;}
  return event->s;
}

}  // namespace vica_guidance
