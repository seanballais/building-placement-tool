#ifndef COREX_CORE_UTILS_HPP
#define COREX_CORE_UTILS_HPP

#include <filesystem>
#include <iostream>
#include <string>
#include <random>

#include <EASTL/set.h>
#include <EASTL/string.h>
#include <nlohmann/json.hpp>
#include <pcg_random.hpp>

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

  template <
    template<class, class, class> class S,
    class K,
    class Compare,
    class Allocator
  > bool isKeyInSet(S<K, Compare, Allocator> s, K key)
  {
    auto search = s.find(key);
    return search != s.end();
  }

  template <class T>
  T generateRandomInt(std::uniform_int_distribution<T> distribution)
  {
    pcg_extras::seed_seq_from<std::random_device> seedSource;
    pcg32 rng(seedSource);

    return distribution(rng);
  }

  template <class T>
  T generateRandomReal(std::uniform_real_distribution<T> distribution)
  {
    pcg_extras::seed_seq_from<std::random_device> seedSource;
    pcg32 rng(seedSource);

    return distribution(rng);
  }

  float getRandomRealUniformly(float a, float b);
  double getRandomRealUniformly(double a, double b);
  int32_t getRandomIntUniformly(int32_t a, int32_t b);

  float metersToPixels(float meters, float ppmRatio);
  float pixelsToMeters(float pixels, float ppmRatio);
  Point screenToWorldCoordinates(const Point&& point, Camera& camera);
}

namespace cx
{
  using namespace corex::core;
}

#endif
