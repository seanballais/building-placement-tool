#ifndef BPT_GENERATOR_HPP
#define BPT_GENERATOR_HPP

#include <EASTL/vector.h>

#include <corex/core/ds/TreeNode.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Polygon.hpp>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution generateRandomSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea);

  cx::TreeNode<cx::Polygon<4>, 4>*
  findUsableRegion(cx::TreeNode<cx::Polygon<4>, 4>* node,
                   float minWidth,
                   float minHeight);
}

#endif
