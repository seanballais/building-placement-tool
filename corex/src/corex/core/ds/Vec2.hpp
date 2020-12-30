#ifndef COREX_CORE_DS_VEC2_HPP
#define COREX_CORE_DS_VEC2_HPP

#include <cstdlib>

namespace corex::core
{
  struct Vec2
  {
    float x;
    float y;

    Vec2();
    Vec2(float x, float y);
    Vec2(const Vec2& rhs);
  };

  Vec2 operator+(const Vec2& p, const Vec2& q);
  Vec2 operator-(const Vec2& p, const Vec2& q);
  Vec2 operator*(const Vec2& p, const int32_t& a);
  Vec2 operator*(const Vec2& p, const float& a);
  Vec2 operator*(const int32_t& a, const Vec2& p);
  Vec2 operator*(const float& a, const Vec2& p);
  Vec2 operator/(const Vec2& p, const int32_t& a);
  Vec2 operator/(const Vec2& p, const float& a);
  bool operator==(const Vec2& p, const Vec2& q);
  bool operator!=(const Vec2& p, const Vec2& q);
}

#endif