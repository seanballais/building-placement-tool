#ifndef COREX_CORE_UTILS_HPP
#define COREX_CORE_UTILS_HPP

#include <filesystem>
#include <iostream>
#include <string>
#include <random>

#include <EASTL/string.h>
#include <nlohmann/json.hpp>

#include <corex/core/Camera.hpp>
#include <corex/core/ds/Point.hpp>

// Stubbed function based on Ryan Gordon's implementation here:
//   http://icculus.org/SteamDevDays/SteamDevDays2014-LinuxPorting.pdf
#define STUBBED(msg)                                          \
do {                                                          \
  static bool seenThis = false;                               \
  if (!seenThis) {                                            \
    seenThis = true;                                          \
    std::cerr << "STUBBED: " << msg << " at " << __FUNCTION__ \
              << " (" << __FILE__ << ":" << __LINE__ << ")"   \
              << std::endl;                                   \
  }                                                           \
} while (0)

namespace corex::core
{
  std::filesystem::path getBinFolder();
  std::filesystem::path getSettingsFolder();

  std::string eaStrToStdStr(const eastl::string& str);
  eastl::string stdStrToEAStr(const std::string& str);

  template <class T>
  T generateRandomInt(std::uniform_int_distribution<T> distribution)
  {
    std::random_device randDevice;
    std::mt19937 mt(randDevice());

    return distribution(mt);
  }

  template <class T>
  T generateRandomReal(std::uniform_real_distribution<T> distribution)
  {
    std::random_device randDevice;
    std::mt19937 mt(randDevice());

    return distribution(mt);
  }

  float metersToPixels(float meters, float ppmRatio);
  float pixelsToMeters(float pixels, float ppmRatio);
  Point screenToWorldCoordinates(const Point&& point, Camera& camera);
}

#endif
