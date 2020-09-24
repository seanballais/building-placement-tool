#ifndef COREX_CORE_CUSTOM_DEFINITIONS_HPP
#define COREX_CORE_CUSTOM_DEFINITIONS_HPP

#include <cstdlib>

namespace corex::core
{
  struct CoreXNull
  {
  public:
    CoreXNull();

    // Allows type conversion from int32_t to CoreXNull.
    CoreXNull(const int32_t& x);
  }; // A way for to have our own null data type.

  extern CoreXNull coreXNull;
}

#endif
