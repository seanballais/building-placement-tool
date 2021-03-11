#ifndef COREX_CORE_CVAR_HPP
#define COREX_CORE_CVAR_HPP

#include <cstdlib>

#include <EASTL/string.h>
#include <EASTL/variant.h>

#include <corex/core/CoreXNull.hpp>

namespace corex::core
{
  // It's typically called a CVAR, but, eh, let's use SVar for consistency.
  using SVar = eastl::variant<
    eastl::monostate,
    int32_t,
    uint32_t,
    float,
    double,
    bool,
    CoreXNull,
    eastl::string
  >;
}

#endif
