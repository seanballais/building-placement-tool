#ifndef BPT_GENERATOR_HPP
#define BPT_GENERATOR_HPP

#include <EASTL/vector.h>

#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Polygon.hpp>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution generateRandomSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea);
}

#endif
