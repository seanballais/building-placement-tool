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
    int32_t tournamentSize;
    int32_t numPrevGenOffsprings;
    float floodProneAreaPenalty;
    float landslideProneAreaPenalty;
    float buildingDistanceWeight;
    int32_t maxNHSize;
    bool isLocalSearchEnabled;
    bool isVNSEnabled;
  };
}

#endif
