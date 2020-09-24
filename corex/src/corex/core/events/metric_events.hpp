#ifndef COREX_CORE_EVENTS_METRIC_EVENTS_HPP
#define COREX_CORE_EVENTS_METRIC_EVENTS_HPP

namespace corex::core
{
  struct FrameDataEvent
  {
    int fps;
    int numFrames;
    float timeDelta;
    float appTimeDelta;
  };
}

#endif
