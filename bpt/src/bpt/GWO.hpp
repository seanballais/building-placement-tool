#ifndef BPT_GWO_HPP
#define BPT_GWO_HPP

#include <cstdlib>

#include <EASTL/vector.h>

#include <corex/core/Timer.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/VecN.hpp>

#include <bpt/HC.hpp>
#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/GWOResult.hpp>
#include <bpt/ds/Wolf.hpp>

namespace bpt
{
  class GWO
  {
  public:
    GWO();
    GWOResult generateSolutions(
      const eastl::vector<InputBuilding> &a,
      const corex::core::NPolygon &b,
      const eastl::vector<eastl::vector<float>> &flowRates,
      const eastl::vector<corex::core::NPolygon> &floodProneAreas,
      const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
      const int32_t numWolves,
      const int32_t numIterations,
      const float alphaDecayRate,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const float buildingDistanceWeight,
      const bool isLocalSearchEnabled,
      const double timeLimit,
      const bool &keepInfeasibleSolutions);
    int32_t getCurrentRunIterationNumber();
  private:
    void computeWolfValues(
      eastl::vector<Wolf>& wolves,
      const eastl::vector<InputBuilding>& inputBuildings,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<eastl::vector<float>>& flowRates,
      const eastl::vector<corex::core::NPolygon>& floodProneAreas,
      const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const float buildingDistanceWeight);
    void updateWolves(
      eastl::vector<Wolf>& wolves,
      const float& alpha,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);
    Wolf huntPrey(
      const int32_t& wolfIdx,
      const eastl::vector<Wolf>& wolves,
      const float& alpha,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);
    cx::VecN createRandomVector(const int32_t vectorSize,
                                float min = 0.f, float max = 1.f);
    cx::VecN createACoefficientVector(const int32_t vectorSize,
                                      const cx::VecN randomVector,
                                      const float alpha);
    cx::VecN createCCoefficientVector(const int32_t vectorSize,
                                      const cx::VecN randomVector);

    cx::Timer runTimer;
    int32_t currRunIterationNumber;
    HC hillClimbing;
  };
}

#endif
