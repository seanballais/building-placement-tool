#ifndef COREX_CORE_RETURN_VALUE_HPP
#define COREX_CORE_RETURN_VALUE_HPP

#include <corex/core/ReturnState.hpp>

namespace corex::core
{
  template <class T>
  struct ReturnValue
  {
    T value;
    ReturnState status;
  };
}

#endif
