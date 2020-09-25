#ifndef COREX_EVENTS_SYS_EVENTS_HPP
#define COREX_EVENTS_SYS_EVENTS_HPP

#include <SDL2/SDL.h>

namespace corex::core
{
  struct WindowEvent { SDL_Event event; };
  struct KeyDownEvent { SDL_Event event; };
  struct KeyUpEvent { SDL_Event event; };
  struct MouseButtonDownEvent { SDL_Event event; };
  struct MouseButtonUpEvent { SDL_Event event; };
  struct MouseMotionEvent { SDL_Event event; };
  struct MouseWheelEvent { SDL_Event event; };

  // TODO: Add other SDL events.
}

#endif
