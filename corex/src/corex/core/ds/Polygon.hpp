#ifndef COREX_CORE_DS_POLYGON_HPP
#define COREX_CORE_DS_POLYGON_HPP

#include <cstdlib>

#include <EASTL/array.h>

#include <corex/core/ds/Point.hpp>

namespace corex::core
{
  template <uint32_t numVertices>
  struct Polygon
  {
    eastl::array<Point, numVertices> vertices;
  };
}

#endif
