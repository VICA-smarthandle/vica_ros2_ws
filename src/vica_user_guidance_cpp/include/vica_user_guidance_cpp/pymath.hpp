// 파이썬 math 모듈과 끝자리까지 같은 값 — 방향지시 판정(turn_detector·rail_turn_forecast)이 쓰는 것만.
//
// sin·cos·atan2·sqrt 는 파이썬도 같은 glibc libm 을 부른다(특수값 처리만 CPython 이 앞에서 한다).
// hypot·dist 는 다르다: CPython 3.8 부터 libm hypot 이 아니라 자기 vector_norm(3.10 은 보정 단계까지)으로
// 계산해 마지막 비트가 libm 과 다를 수 있다. 코너 신원(0.5 m 반경)·레일 위 판정(0.8 m) 같은 문턱 비교가
// 경계에서 갈리지 않게 CPython 3.10 의 계산을 그대로 옮겼다(test_turn_core 가 파이썬과 비트 비교).
#pragma once

namespace vica_guidance
{

constexpr double PY_PI = 3.14159265358979323846;

// math.atan2 — CPython m_atan2(NaN·inf·0 처리) 뒤 libm atan2.
double py_atan2(double y, double x);
// math.sin / math.cos / math.sqrt — 유한 입력이 NaN 이 되면 ValueError("math domain error").
double py_sin(double x);
double py_cos(double x);
double py_sqrt(double x);
// math.hypot(a, b)
double py_hypot(double a, double b);
// math.dist((px, py), (qx, qy))
double py_dist(double px, double py, double qx, double qy);
// math.degrees / math.radians
inline double py_degrees(double x) {return x * (180.0 / PY_PI);}
inline double py_radians(double x) {return x * (PY_PI / 180.0);}

}  // namespace vica_guidance
