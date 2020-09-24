#include <assert.h>
#include <cstdlib>

#include <EASTL/unique_ptr.h>
#include <SDL_gpu.h>

#include <corex/core/Camera.hpp>
#include <corex/core/CameraZoomState.hpp>

namespace corex::core
{
  Camera::Camera(float x, float y, float z,
                 float zNear, float zFar,
                 float zoomX, float zoomY,
                 float zoomXDelta, float zoomYDelta,
                 float angle)
    : zoomXDelta(zoomXDelta)
    , zoomYDelta(zoomYDelta)
    // No choice. Gotta use a raw new because GPU_Camera is a struct. Still
    // waiting for C++ 20 to mature so that aggregates can be initialized from
    // a parenthesized list of values.
    //   Relevant Link: http://www.open-std.org/jtc1/sc22/wg21
    //                        /docs/papers/2019/p0960r3.html
    , gpuCamera(new GPU_Camera {
        x, y, z, angle, zoomX, zoomY, zNear, zFar, true
      }) {}

  GPU_Camera* Camera::getGPUCamera()
  {
    return this->gpuCamera.get();
  }

  float Camera::getX()
  {
    return this->gpuCamera->x;
  }
  
  float Camera::getY()
  {
    return this->gpuCamera->y;
  }
  
  float Camera::getZ()
  {
    return this->gpuCamera->z;
  }
  
  float Camera::getZNear()
  {
    return this->gpuCamera->z_near;
  }
  
  float Camera::getZFar()
  {
    return this->gpuCamera->z_far;
  }
  
  float Camera::getZoomX()
  {
    return this->gpuCamera->zoom_x;
  }
  
  float Camera::getZoomY()
  {
    return this->gpuCamera->zoom_y;
  }
  
  float Camera::getZoomXDelta()
  {
    return this->zoomXDelta;
  }
  
  float Camera::getZoomYDelta()
  {
    return this->zoomYDelta;
  }
  
  float Camera::getAngle()
  {
    return this->gpuCamera->angle;
  }

  void Camera::setX(float x)
  {
    this->gpuCamera->x = x;
  }
    
  void Camera::setY(float y)
  {
    this->gpuCamera->y = y;
  }
  
  void Camera::setZ(float z)
  {
    this->gpuCamera->z = z;
  }
  
  void Camera::setZNear(float near)
  {
    this->gpuCamera->z_near = near;
  }
  
  void Camera::setZFar(float far)
  {
    this->gpuCamera->z_far = far;
  }
  
  void Camera::setZoomX(float x)
  {
    this->gpuCamera->zoom_x = x;
  }
  
  void Camera::setZoomY(float y)
  {
    this->gpuCamera->zoom_y = y;
  }
  
  void Camera::setZoomXDelta(float delta)
  {
    this->zoomXDelta = delta;
  }
  
  void Camera::setZoomYDelta(float delta)
  {
    this->zoomYDelta = delta;
  }
  
  void Camera::setAngle(float angle)
  {
    // The rotation is clockwise. Negating the angle will rotate the camera
    // in the way we typically expect -- counterclockwise.
    this->gpuCamera->angle = -angle;
  }

  void Camera::moveX(float delta)
  {
    this->gpuCamera->x += delta;
  }

  void Camera::moveY(float delta)
  {
    this->gpuCamera->y += delta;
  }

  void Camera::zoom(CameraZoomState state)
  {
    switch (state) {
      case CameraZoomState::IN:
        this->gpuCamera->zoom_x += this->zoomXDelta;
        this->gpuCamera->zoom_y += this->zoomYDelta;
        break;
      case CameraZoomState::OUT:
        this->gpuCamera->zoom_x -= this->zoomXDelta;
        this->gpuCamera->zoom_y -= this->zoomYDelta;
        break;
    }
  }
}
