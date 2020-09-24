#ifndef COREX_CORE_DEBUG_SETTINGS_HPP
#define COREX_CORE_DEBUG_SETTINGS_HPP

#include <cstdlib>

namespace corex::core
{
  struct DebugSettings
  {
    bool isFreeFlyCameraEnabled;
    float freeFlyCameraSpeed;
    float cameraX;
    float cameraY;
    float cameraZ;
    float cameraZNear;
    float cameraZFar;
    float cameraZoomX;
    float cameraZoomY;
    float cameraZoomXDelta;
    float cameraZoomYDelta;
    float cameraAngle;
    bool isDebugUIDisplayed;
    bool isCameraControlsDisplayed;
    bool isPerformanceMetricsDisplayed;
  };
}

#endif
