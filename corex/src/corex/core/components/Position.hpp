#ifndef COREX_CORE_COMPONENTS_POSITION_HPP
#define COREX_CORE_COMPONENTS_POSITION_HPP

#include <cstdint>

namespace corex::core
{
  struct Position
  {
    // x, y, and z correspond to the center coordinate of the entity.
    float x;
    float y;
    float z;
    int8_t sortingLayerID;
  };
}

#endif