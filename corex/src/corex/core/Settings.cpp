#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <EASTL/string.h>
#include <EASTL/unordered_map.h>
#include <EASTL/variant.h>

#include <corex/core/CoreXNull.hpp>
#include <corex/core/Settings.hpp>
#include <corex/core/SettingsValue.hpp>
#include <corex/core/SVar.hpp>
#include <corex/core/utils.hpp>

namespace corex::core
{
  Settings::Settings()
    : settings()
    , settingsFilePath(
        stdStrToEAStr((getSettingsFolder() / "settings.cxstg").string())
      )
  {
    this->loadSettingsFile();
  }

  void Settings::setVariable(const eastl::string& name, CoreXNull)
  {
    this->settings[name] = coreXNull;
  }

  void Settings::setVariable(const eastl::string& name, bool value)
  {
    this->settings[name] = value;
  }

  void Settings::setVariable(const eastl::string& name, eastl::string value)
  {
    this->settings[name] = value;
  }

  void Settings::setVariable(const eastl::string& name, int32_t value)
  {
    this->settings[name] = value;
  }

  void Settings::setVariable(const eastl::string& name, uint32_t value)
  {
    this->settings[name] = value;
  }

  void Settings::setVariable(const eastl::string& name, float value)
  {
    this->settings[name] = value;
  }

  SettingsValue<CoreXNull> Settings::getNullVariable(const eastl::string& name)
  {
    return this->getVariable<CoreXNull>(name);
  }

  SettingsValue<bool> Settings::getBooleanVariable(const eastl::string& name)
  {
    return this->getVariable<bool>(name);
  }

  SettingsValue<eastl::string>
  Settings::getStringVariable(const eastl::string& name)
  {
    return this->getVariable<eastl::string>(name);
  }

  SettingsValue<int32_t> Settings::getIntegerVariable(const eastl::string& name)
  {
    return this->getVariable<int32_t>(name);
  }

  SettingsValue<uint32_t>
  Settings::getUnsignedIntVariable(const eastl::string& name)
  {
    return this->getVariable<uint32_t>(name);
  }

  SettingsValue<float> Settings::getFloatVariable(const eastl::string& name)
  {
    return this->getVariable<float>(name);
  }

  void Settings::save()
  {
    nlohmann::json settingsJSON;
    for (auto& pair : this->settings) {
      std::string name = eaStrToStdStr(pair.first);
      SVar& value = pair.second;
      eastl::visit([&settingsJSON, &name](SVar&& value) {
        if (eastl::holds_alternative<CoreXNull>(value)) {
          settingsJSON[name] = nullptr;
        } else if (eastl::holds_alternative<bool>(value)) {
          settingsJSON[name] = eastl::get<bool>(value);
        } else if (eastl::holds_alternative<eastl::string>(value)) {
          settingsJSON[name] = eaStrToStdStr(eastl::get<eastl::string>(value));
        } else if (eastl::holds_alternative<int32_t>(value)) {
          settingsJSON[name] = eastl::get<int32_t>(value);
        } else if (eastl::holds_alternative<uint32_t>(value)) {
          settingsJSON[name] = eastl::get<uint32_t>(value);
        } else if (eastl::holds_alternative<float>(value)) {
          settingsJSON[name] = eastl::get<float>(value);
        }
      }, value);
    }

    std::ofstream jsonFile(eaStrToStdStr(this->settingsFilePath),
                           std::ofstream::trunc);
    jsonFile << settingsJSON;
  }

  void Settings::reset()
  {
    this->settings.clear();
    this->loadSettingsFile();
  }

  void Settings::loadSettingsFile()
  {
    std::ifstream settingsFile(eaStrToStdStr(this->settingsFilePath),
                               std::ifstream::in);
    if (settingsFile.peek() == std::ifstream::traits_type::eof()) {
      // File is empty. We shouldn't' bother parsing through the data, just like
      // when we find out that her feelings for you are empty.
      return;
    }

    nlohmann::json settingsJSON;
    settingsFile >> settingsJSON;

    for (auto& [key, value] : settingsJSON.items()) {
      STUBBED("Handle fail states for settings loading.");
      // We ditched using an implementation of from_json(...) for eastl::string
      // here, since it causes a compile error where there is ambiguity of end()
      // in nlohmann/json.hpp (at least in Clang 10). So, we do the next best
      // thing -- converting the result of .get<std::string>() to an
      // eastl::string through a utility function.
      eastl::string varName = stdStrToEAStr(key);

      SVar settingSVar;
      if (value.type() == nlohmann::json::value_t::null) {
        settingSVar = coreXNull;
      } else if (value.type() == nlohmann::json::value_t::boolean) {
        settingSVar = value.get<bool>();
      } else if (value.type() == nlohmann::json::value_t::string) {
        settingSVar = stdStrToEAStr(value.get<std::string>());
      } else if (value.type() == nlohmann::json::value_t::number_integer) {
        settingSVar = value.get<int32_t>();
      } else if (value.type() == nlohmann::json::value_t::number_unsigned) {
        settingSVar = value.get<uint32_t>();
      } else if (value.type() == nlohmann::json::value_t::number_float) {
        settingSVar = value.get<float>();
      } else {
        STUBBED("Handle fail states for loading an unsupported data type.");
      }

      this->settings[varName] = settingSVar;
    }
  }
}
