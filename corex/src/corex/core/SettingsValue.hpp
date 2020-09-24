#ifndef COREX_CORE_SETTINGS_VALUE_HPP
#define COREX_CORE_SETTINGS_VALUE_HPP

#include <corex/core/ReturnState.hpp>

namespace corex::core
{
  template <class T>
  struct SettingsValue
  {
    T value;
    ReturnState returnState;
  };
}

#endif
