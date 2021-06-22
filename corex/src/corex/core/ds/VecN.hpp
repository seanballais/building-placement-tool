#ifndef COREX_CORE_DS_VECN_HPP
#define COREX_CORE_DS_VECN_HPP

#include <cstdlib>
#include <iostream>

#include <EASTL/vector.h>

namespace corex::core
{
  class VecN
  {
  public:
    explicit VecN(int32_t size, float initialValue = 0.f);
    explicit VecN(eastl::vector<float> elements);

    size_t size() const;

    float& operator[](int32_t i);
    VecN operator-();
    float operator[](int32_t i) const;
    const VecN operator-() const;

    eastl::vector<float>::iterator begin();
    eastl::vector<float>::iterator end();
    eastl::vector<float>::const_iterator begin() const;
    eastl::vector<float>::const_iterator end() const;
  private:
    eastl::vector<float> elements;
    int32_t maxNumElements;
  };

  VecN operator+(const VecN& p, const VecN& q);
  VecN operator-(const VecN& p, const VecN& q);
  VecN operator*(const VecN& p, const float& a);
  VecN operator*(const float& a, const VecN& p);
  VecN operator*(const double& a, const VecN& p);
  VecN operator*(const VecN& p, const double& a);
  VecN operator/(const VecN& p, const float& a);

  bool operator==(const VecN& p, const VecN& q);
  bool operator!=(const VecN& p, const VecN& q);

  std::ostream& operator<<(std::ostream& os, const VecN& vec);
}

namespace cx
{
  using namespace corex::core;
}

#endif
