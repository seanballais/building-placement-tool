#include <corex/core/math_functions.hpp>
#include <corex/core/ds/Vec2.hpp>

namespace corex::core
{
  Vec2::Vec2()
    : x()
    , y() {}

  Vec2::Vec2(float x, float y)
    : x(x)
    , y(y) {}

  Vec2::Vec2(const Vec2& rhs)
    : x(rhs.x)
    , y(rhs.y) {}

  Vec2 operator+(const Vec2& p, const Vec2& q)
  {
    return Vec2{ setDecPlaces(p.x + q.x, 6), setDecPlaces(p.y + q.y, 6) };
  }

  Vec2 operator-(const Vec2& p, const Vec2& q)
  {
    return Vec2{ setDecPlaces(p.x - q.x, 6), setDecPlaces(p.y - q.y, 6) };
  }

  Vec2 operator*(const Vec2& p, const int32_t& a)
  {
    return Vec2{ setDecPlaces(p.x * a, 6), setDecPlaces(p.y * a, 6) };
  }

  Vec2 operator*(const Vec2& p, const float& a)
  {
    return Vec2{ setDecPlaces(p.x * a, 6), setDecPlaces(p.y * a, 6) };
  }

  Vec2 operator*(const int32_t& a, const Vec2& p)
  {
    return Vec2{ setDecPlaces(a * p.x, 6), setDecPlaces(a * p.y, 6) };
  }

  Vec2 operator*(const float& a, const Vec2& p)
  {
    return Vec2{ setDecPlaces(a * p.x, 6), setDecPlaces(a * p.y, 6) };
  }

  Vec2 operator/(const Vec2& p, const int32_t& a)
  {
    return Vec2{ setDecPlaces(p.x / a, 6), setDecPlaces(p.y / a, 6) };
  }

  Vec2 operator/(const Vec2& p, const float& a)
  {
    return Vec2{ setDecPlaces(p.x / a, 6), setDecPlaces(p.y / a, 6) };
  }

  bool operator==(const Vec2& p, const Vec2& q)
  {
    return floatEquals(p.x, q.x) && floatEquals(p.y, q.x);
  }

  bool operator!=(const Vec2& p, const Vec2& q)
  {
    return !(p == q);
  }
}