#ifndef BPT_DS_GA_SETTINGS_HPP
#define BPT_DS_GA_SETTINGS_HPP

#include <cstdlib>

#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/SelectionType.hpp>

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
    bool isLocalSearchEnabled;
    CrossoverType crossoverType;
    SelectionType selectionType;
    bool keepInfeasibleSolutions;
  };
}

#endif
