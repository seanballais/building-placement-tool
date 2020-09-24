#ifndef COREX_CORE_CAMERA_HPP
#define COREX_CORE_CAMERA_HPP

#include <cstdlib>

#include <EASTL/unique_ptr.h>
#include <SDL_gpu.h>

#include <corex/core/CameraZoomState.hpp>

namespace corex::core
{
  class Camera
  {
  public:
    Camera(float x = 0.0f, float y = 0.0f, float z = 0.0f,
           float zNear = -100.0f, float zFar = 100.0f,
           float zoomX = 1.0f, float zoomY = 1.0f,
           float zoomXDelta = 0.15f, float zoomYDelta = 0.15f,
           float angle = 0.0f);

    GPU_Camera* getGPUCamera();
    float getX();
    float getY();
    float getZ();
    float getZNear();
    float getZFar();
    float getZoomX();
    float getZoomY();
    float getZoomXDelta();
    float getZoomYDelta();
    float getAngle();

    void setX(float x);
    void setY(float y);
    void setZ(float z);
    void setZNear(float near);
    void setZFar(float far);
    void setZoomX(float x);
    void setZoomY(float y);
    void setZoomXDelta(float delta);
    void setZoomYDelta(float delta);
    void setAngle(float angle);

    void moveX(float delta);
    void moveY(float delta);
    void zoom(CameraZoomState state);

  private:
    float zoomXDelta;
    float zoomYDelta;
    eastl::unique_ptr<GPU_Camera> gpuCamera;
  };
}

#endif
