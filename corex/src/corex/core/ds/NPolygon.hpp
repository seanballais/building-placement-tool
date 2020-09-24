#ifndef COREX_CORE_DS_NPOLYGON_HPP
#define COREX_CORE_DS_NPOLYGON_HPP

#include <EASTL/vector.h>

#include <corex/core/ds/Point.hpp>

namespace corex::core
{
  struct NPolygon
  {
    // Sometimes, we don't know how many vertices a polygon have during
    // compile-time. So, we're going to use a polygon whose number of vertices
    // is only known during runtime.
    eastl::vector<Point> vertices;
  };
}

#endif
