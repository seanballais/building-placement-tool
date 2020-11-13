#include <EASTL/string.h>
#include <EASTL/unique_ptr.h>
#include <SDL2/SDL.h>
#include <SDL_gpu.h>

#include <corex/core/asset_types/Font.hpp>
#include <corex/core/components/Text.hpp>
#include <corex/core/sdl_deleters.hpp>

namespace corex::core
{
  Text::Text(const eastl::string&& text, Font font, SDL_Color colour)
    : renderableText(nullptr)
  {
    SDL_Surface* textSurface = TTF_RenderText_Solid(font.get(),
                                                    text.c_str(),
                                                    colour);
    this->renderableText = eastl::unique_ptr<GPU_Image, SDLGPUImageDeleter>(
      GPU_CopyImageFromSurface(textSurface));

    SDL_FreeSurface(textSurface);
  }

  GPU_Image* Text::getRenderableText() const
  {
    return this->renderableText.get();
  }
}
