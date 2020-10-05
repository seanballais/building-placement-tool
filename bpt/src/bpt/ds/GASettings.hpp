#ifndef BPT_DS_GA_SETTINGS_HPP
#define BPT_DS_GA_SETTINGS_HPP

#include <cstdlib>

namespace bpt
{
  struct GASettings
  {
    float mutationRate;
    int32_t populationSize;
    int32_t numGenerations;
  };
}

#endif