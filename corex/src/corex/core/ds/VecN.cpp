#include <cassert>
#include <cstdlib>
#include <iostream>

#include <EASTL/vector.h>

#include <corex/core/math_functions.hpp>
#include <corex/core/ds/VecN.hpp>

namespace corex::core
{
  VecN::VecN(int32_t size, float initialValue)
    : elements(size, initialValue)
    , maxNumElements(size) {}

  VecN::VecN(eastl::vector<float> elements)
    : elements(elements)
    , maxNumElements(elements.size()) {}

  size_t VecN::size() const
  {
    return this->maxNumElements;
  }

  float& VecN::operator[](int32_t i)
  {
    assert(i < this->maxNumElements);
    return this->elements[i];
  }

  VecN VecN::operator-()
  {
    VecN p = *this;
    for (int32_t i = 0; i < p.size(); i++) {
      p[i] *= -1;
    }

    return p;
  }

  float VecN::operator[](int32_t i) const
  {
    assert(i < this->maxNumElements);
    return this->elements[i];
  }

  const VecN VecN::operator-() const
  {
    VecN p = *this;
    for (int32_t i = 0; i < p.size(); i++) {
      p[i] *= -1;
    }

    return p;
  }

  eastl::vector<float>::iterator VecN::begin()
  {
    return this->elements.begin();
  }

  eastl::vector<float>::iterator VecN::end()
  {
    return this->elements.end();
  }

  eastl::vector<float>::const_iterator VecN::begin() const
  {
    return this->elements.begin();
  }

  eastl::vector<float>::const_iterator VecN::end() const
  {
    return this->elements.end();
  }

  VecN operator+(const VecN& p, const VecN& q)
  {
    assert(p.size() == q.size());

    VecN r = p;
    for (int32_t i = 0; i < r.size(); i++) {
      const float qVal = q[i];
      r[i] += qVal;
    }

    return r;
  }

  VecN operator-(const VecN& p, const VecN& q)
  {
    assert(p.size() == q.size());

    return p + (-q);
  }

  VecN operator*(const VecN& p, const float& a)
  {
    VecN q = p;
    for (int32_t i = 0; i < q.size(); i++) {
      q[i] *= a;
    }

    return q;
  }

  VecN operator*(const float& a, const VecN& p)
  {
    return p * a;
  }

  VecN operator*(const VecN& p, const double& a)
  {
    VecN q = p;
    for (int32_t i = 0; i < q.size(); i++) {
      q[i] = static_cast<float>(static_cast<double>(q[i]) * a);
    }

    return q;
  }

  VecN operator*(const double& a, const VecN& p)
  {
    return p * a;
  }

  VecN operator/(const VecN& p, const float& a)
  {
    return p * (1 / a);
  }

  bool operator==(const VecN& p, const VecN& q)
  {
    if (p.size() != q.size()) {
      return false;
    }

    for (int32_t i = 0; i < p.size(); i++) {
      if (!floatEquals(p[i], q[i])) {
        return false;
      }
    }

    return true;
  }

  bool operator!=(const VecN& p, const VecN& q)
  {
    return !(p == q);
  }

  std::ostream& operator<<(std::ostream& os, const VecN& vec)
  {
    for (const float& f : vec) {
      os << f << " ";
    }

    return os;
  }
}
