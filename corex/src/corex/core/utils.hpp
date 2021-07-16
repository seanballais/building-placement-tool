#ifndef COREX_CORE_UTILS_HPP
#define COREX_CORE_UTILS_HPP

#include <cassert>
#include <filesystem>
#include <iostream>
#include <string>
#include <random>

#include <EASTL/algorithm.h>
#include <EASTL/functional.h>
#include <EASTL/set.h>
#include <EASTL/string.h>
#include <EASTL/vector.h>
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

  template <class T>
  T generateRandomReal(std::normal_distribution<T> distribution)
  {
    pcg_extras::seed_seq_from<std::random_device> seedSource;
    pcg32 rng(seedSource);

    return distribution(rng);
  }

  float getRandomRealNormDistrib(float mean, float stddev);

  float getRandomRealUniformly(float a, float b);
  double getRandomRealUniformly(double a, double b);
  int32_t getRandomIntUniformly(int32_t a, int32_t b);

  float metersToPixels(float meters, float ppmRatio);
  float pixelsToMeters(float pixels, float ppmRatio);
  Point screenToWorldCoordinates(const Point&& point, Camera& camera);

  template <class T, class Allocator>
  void reorderVector(eastl::vector<T, Allocator>& targetVec,
                     eastl::vector<int32_t> indices)
  {
    assert(targetVec.size() == indices.size());

    for (int32_t i = 0; i < targetVec.size(); i++) {
      while (indices[i] != i) {
        eastl::swap(targetVec[indices[i]], targetVec[i]);
        eastl::swap(indices[indices[i]], indices[i]);
      }
    }
  }

  template <class T, class Allocator, class Comparator>
  eastl::vector<int32_t> rankVectors(eastl::vector<T, Allocator>& vec,
                                     Comparator cmp)
  {
    // Based on:
    //   https://stackoverflow.com/a/35595758/1116098
    eastl::vector<int32_t> ranking(vec.size(), 0);
    for (int32_t i = 0; i < vec.size(); i++) {
      int32_t currRank = 0;

      for (int32_t j = 0; j < i; j++) {
        if (!cmp(vec[i], vec[j])) {
          // We negate the comparator function because we want it to show the
          // expected order of the elements.
          currRank++;
        } else {
          ranking[j]++;
        }
      }

      ranking[i] = currRank;
    }

    return ranking;
  }
}

namespace cx
{
  using namespace corex::core;
}

#endif
