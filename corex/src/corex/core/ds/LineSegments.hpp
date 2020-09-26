#ifndef COREX_CORE_DS_LINE_SEGMENTS_HPP
#define COREX_CORE_DS_LINE_SEGMENTS_HPP

#include <EASTL/vector.h>

#include <corex/core/ds/Point.hpp>

namespace corex::core
{
  struct LineSegments
  {
    eastl::vector<Point> vertices;
  };
}

#endif
