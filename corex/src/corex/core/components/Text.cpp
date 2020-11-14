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
    , text(text)
    , font(font)
    , colour(colour)
  {
    this->generateTextTexture();
  }

  GPU_Image* Text::getRenderableText() const
  {
    return this->renderableText.get();
  }

  void Text::setText(const eastl::string&& text)
  {
    this->text = text;
    this->generateTextTexture();
  }

  void Text::setFont(const Font font)
  {
    this->font = font;
    this->generateTextTexture();
  }

  void Text::setColour(const SDL_Color colour)
  {
    this->colour = colour;
    this->generateTextTexture();
  }

  void Text::generateTextTexture()
  {
    SDL_Surface* textSurface = TTF_RenderText_Blended(this->font.get(),
                                                      this->text.c_str(),
                                                      this->colour);
    this->renderableText = eastl::unique_ptr<GPU_Image, SDLGPUImageDeleter>(
      GPU_CopyImageFromSurface(textSurface));

    SDL_FreeSurface(textSurface);
  }
}
