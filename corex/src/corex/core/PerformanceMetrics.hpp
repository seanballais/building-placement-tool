#ifndef COREX_CORE_PERFORMANCE_METRICS_HPP
#define COREX_CORE_PERFORMANCE_METRICS_HPP

#include <cstdlib>

namespace corex::core
{
  struct PerformanceMetrics
  {
    int32_t fps = 0;
    int32_t numFrames = 0;
    float timeDelta = 0.0f;
    float appTimeDelta = 0.0f;
  };
}

#endif
