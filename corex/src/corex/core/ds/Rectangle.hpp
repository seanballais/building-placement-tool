#ifndef COREX_CORE_DS_RECTANGLE_HPP
#define COREX_CORE_DS_RECTANGLE_HPP

namespace corex::core
{
  struct Rectangle
  {
    // x and y will refer to the center of the rectangle.
    float x;
    float y;
    float width;
    float height;
    float angle;
  };
}

#endif
