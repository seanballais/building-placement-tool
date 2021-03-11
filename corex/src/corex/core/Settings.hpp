#ifndef COREX_CORE_SETTINGS_HPP
#define COREX_CORE_SETTINGS_HPP

#include <cstdlib>
#include <iostream>

#include <EASTL/string.h>
#include <EASTL/unordered_map.h>

#include <corex/core/CoreXNull.hpp>
#include <corex/core/ReturnState.hpp>
#include <corex/core/SettingsValue.hpp>
#include <corex/core/SVar.hpp>
#include <corex/core/utils.hpp>

namespace corex::core
{
  class Settings
  {
  public:
    Settings();
    explicit Settings(const eastl::string& filename);

    void setVariable(const eastl::string& name, CoreXNull);
    void setVariable(const eastl::string& name, bool value);
    void setVariable(const eastl::string& name, eastl::string value);
    void setVariable(const eastl::string& name, int32_t value);
    void setVariable(const eastl::string& name, uint32_t value);
    void setVariable(const eastl::string& name, float value);
    void setVariable(const eastl::string& name, double value);

    SettingsValue<CoreXNull> getNullVariable(const eastl::string& name);
    SettingsValue<bool> getBooleanVariable(const eastl::string& name);
    SettingsValue<eastl::string> getStringVariable(const eastl::string& name);
    SettingsValue<int32_t> getIntegerVariable(const eastl::string& name);
    SettingsValue<uint32_t> getUnsignedIntVariable(const eastl::string& name);
    SettingsValue<float> getFloatVariable(const eastl::string& name);
    SettingsValue<double> getDoubleVariable(const eastl::string& name);

    void save();
    void reset();

  private:
    template <class T>
    SettingsValue<T> getVariable(const eastl::string& name)
    {
      auto iter = this->settings.find(name);
      if (iter == this->settings.end()) {
        std::cout << "Settings variable '" << eaStrToStdStr(name)
                  << "' does not exist." << std::endl;
        STUBBED("The assert above should be an error log message.");
      } else {
        if (T* value = eastl::get_if<T>(&(iter->second))) {
          return SettingsValue<T>{ *value, ReturnState::RETURN_OK };
        }
      }

      return SettingsValue<T>{
        static_cast<T>(0),
        ReturnState::RETURN_FAIL
      };
    }

    void loadSavedSettings();

    eastl::unordered_map<eastl::string, SVar> settings;
    eastl::string settingsFilePath;
  };
}

namespace cx
{
  using namespace corex::core;
}

#endif
