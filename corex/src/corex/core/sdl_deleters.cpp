#include <SDL2/SDL.h>
#include <SDL_gpu.h>

#include <corex/core/sdl_deleters.hpp>

namespace corex::core
{
  void SDLRendererDeleter::operator()(SDL_Renderer* renderer)
  {
    SDL_DestroyRenderer(renderer);
  }

  void SDLTextureDeleter::operator()(SDL_Texture* texture)
  {
    SDL_DestroyTexture(texture);
  }

  void SDLWindowDeleter::operator()(SDL_Window* window)
  {
    SDL_DestroyWindow(window);
  }

  void SDLGPUTargetDeleter::operator()(GPU_Target* target)
  {
    GPU_FreeTarget(target);
  }

  void SDLGPUImageDeleter::operator()(GPU_Image* image)
  {
    GPU_FreeImage(image);
  }
}
