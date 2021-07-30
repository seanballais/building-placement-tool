#ifndef BPT_PSO_HPP
#define BPT_PSO_HPP

#include <cstdlib>

#include <EASTL/vector.h>

#include <corex/core/Timer.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/VecN.hpp>

#include <bpt/HC.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/PSOResult.hpp>
#include <bpt/ds/Particle.hpp>

namespace bpt
{
  class PSO
  {
  public:
    PSO();
    PSOResult generateSolutions(
      const eastl::vector<InputBuilding> &inputBuildings,
      const corex::core::NPolygon &boundingArea,
      const eastl::vector<eastl::vector<float>> &flowRates,
      const float buildingDistanceWeight,
      const int32_t numIterations,
      const int32_t numParticles,
      const float w,
      const float c1,
      const float c2);

    int32_t getCurrentRunIterationNumber();
  private:
    void computeWolfValues(
      eastl::vector<Particle>& particles,
      const eastl::vector<InputBuilding>& inputBuildings,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<eastl::vector<float>>& flowRates,
      const float buildingDistanceWeight);

    cx::Timer runTimer;
    int32_t currRunIterationNumber;
  };
}

#endif
