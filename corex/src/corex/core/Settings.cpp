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
    this->loadSavedSettings();
  }

  Settings::Settings(const eastl::string& filename)
    : settings()
    , settingsFilePath(
        stdStrToEAStr((getSettingsFolder() / eaStrToStdStr(filename)).string())
      )
  {
    this->loadSavedSettings();
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

  void Settings::setVariable(const eastl::string& name, double value)
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

  SettingsValue<double> Settings::getDoubleVariable(const eastl::string& name)
  {
    return this->getVariable<double>(name);
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
          settingsJSON[name] = {
            { "type", "int" },
            { "value", eastl::get<int32_t>(value) }
          };
        } else if (eastl::holds_alternative<uint32_t>(value)) {
          settingsJSON[name] = {
            { "type", "uint" },
            { "value", eastl::get<uint32_t>(value) }
          };
        } else if (eastl::holds_alternative<float>(value)) {
          settingsJSON[name] = {
            { "type", "float" },
            { "value", eastl::get<float>(value) }
          };
        } else if (eastl::holds_alternative<double>(value)) {
          settingsJSON[name] = {
            { "type", "double" },
            { "value", eastl::get<double>(value) }
          };
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
    this->loadSavedSettings();
  }

  void Settings::loadSavedSettings()
  {
    std::ifstream settingsFile(eaStrToStdStr(this->settingsFilePath),
                               std::ifstream::in);
    if (settingsFile.peek() == std::ifstream::traits_type::eof()) {
      // File is empty. We shouldn't' bother parsing through the data, just like
      // when we find out that her feelings for you are empty.
      return;
    }

    // TODO: Add `type` attribute for integer values since JSON does not have
    //       signed or unsigned data types. It only has a number. The `type`
    //       attribute will let us easily know which keys are signed or
    //       unsigned.
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
      if (value.is_object()
          && value.find("type") != value.end()
          && value.find("value") != value.end()
          && (value["type"].get<std::string>() == "int"
              || value["type"].get<std::string>() == "uint"
              || value["type"].get<std::string>() == "float"
              || value["type"].get<std::string>() == "double")) {
        // JSON does not have a concept of signed and unsigned integers. To have
        // that, we'll save an integer object instead which includes a type
        // attribute that determines whether the integer is signed or not.
        if (value["type"].get<std::string>() == "int") {
          settingSVar = value["value"].get<int32_t>();
        } else if (value["type"].get<std::string>() == "uint") {
          settingSVar = value["value"].get<uint32_t>();
        } else if (value["type"].get<std::string>() == "float") {
          settingSVar = value["value"].get<float>();
        } else if (value["type"].get<std::string>() == "double") {
          settingSVar = value["value"].get<double>();
        } else {
          STUBBED("Handle fail states for specifying an unsupported data type "
                  "in an integer object.");
        }
      } else if (value.is_null()) {
        settingSVar = coreXNull;
      } else if (value.is_boolean()) {
        settingSVar = value.get<bool>();
      } else if (value.is_string()) {
        settingSVar = stdStrToEAStr(value.get<std::string>());
      } else if (value.is_number_float()) {
        settingSVar = value.get<float>();
      } else {
        STUBBED("Handle fail states for loading an unsupported data type.");
      }

      this->settings[varName] = settingSVar;
    }
  }
}
