#ifndef COREX_CORE_SDL_DELETERS_HPP
#define COREX_CORE_SDL_DELETERS_HPP

#include <SDL2/SDL.h>
#include <SDL_gpu.h>

namespace corex::core
{
  struct SDLRendererDeleter
  {
  public:
    void operator()(SDL_Renderer* renderer);
  };

  struct SDLTextureDeleter
  {
  public:
    void operator()(SDL_Texture* texture);
  };

  struct SDLWindowDeleter
  {
  public:
    void operator()(SDL_Window* window);
  };

  struct SDLGPUTargetDeleter
  {
  public:
    void operator()(GPU_Target* target);
  };

  struct SDLGPUImageDeleter
  {
  public:
    void operator()(GPU_Image* image);
  };
}

#endif
