#ifndef COREX_CORE_COMPONENTS_TEXT_HPP
#define COREX_CORE_COMPONENTS_TEXT_HPP

#include <EASTL/string.h>
#include <EASTL/unique_ptr.h>
#include <SDL2/SDL.h>
#include <SDL_gpu.h>

#include <corex/core/asset_types/Font.hpp>
#include <corex/core/sdl_deleters.hpp>

namespace corex::core
{
  class Text
  {
  public:
    Text(const eastl::string&& text, Font font, SDL_Color colour);
    GPU_Image* getRenderableText() const;
    void setText(const eastl::string&& text);
    void setFont(const Font font);
    void setColour(const SDL_Color colour);

  private:
    eastl::unique_ptr<GPU_Image, SDLGPUImageDeleter> renderableText;
    eastl::string text;
    Font font;
    SDL_Color colour;

    void generateTextTexture();
  };
}

#endif
