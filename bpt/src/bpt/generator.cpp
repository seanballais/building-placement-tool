#include <random>

#include <EASTL/iterator.h>
#include <EASTL/list.h>
#include <EASTL/unordered_set.h>
#include <EASTL/vector.h>
#include <pcg_random.hpp>

#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Polygon.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/ds/Tree.hpp>
#include <corex/core/math_functions.hpp>

#include <bpt/evaluator.hpp>
#include <bpt/generator.hpp>
#include <bpt/operators.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution generateRandomSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea)
  {
    Solution solution{ static_cast<int32_t>(inputBuildings.size()) };
    
    // Put in the buildings!
    const cx::Point& boundingMinPt = boundingArea.vertices[0];
    const cx::Point& boundingMaxPt = boundingArea.vertices[2];

    // Shuffle order of building placement.
    int32_t numBuildings = inputBuildings.size();
    eastl::vector<int32_t> shuffledBuildingIndexes(numBuildings);
    std::iota(shuffledBuildingIndexes.begin(),
              shuffledBuildingIndexes.end(),
              0);

    pcg_extras::seed_seq_from<std::random_device> seedSource;
    pcg32 rng(seedSource);
    std::shuffle(shuffledBuildingIndexes.begin(),
                 shuffledBuildingIndexes.end(),
                 rng);

    for (const int32_t& i : shuffledBuildingIndexes) {
      corex::core::Point buildingPos { 0.f, 0.f };
      float buildingAngle = 0.f;
      corex::core::Rectangle buildingRect {
        buildingPos.x,
        buildingPos.y,
        inputBuildings[i].width,
        inputBuildings[i].length,
        buildingAngle
      };

      // We need to get the angle first so that we can correctly get the width
      // and height of the buildingRect.
      buildingAngle = cx::selectItemRandomly(
        eastl::vector<float>{ 0.f, 90.f });
      buildingRect.angle = buildingAngle;

      cx::Polygon<4> poly = cx::rotateRectangle(buildingRect);

      float polyWidth = poly.vertices[1].x - poly.vertices[0].x;
      float polyHeight = poly.vertices[3].y - poly.vertices[0].y;

      int32_t minBuildingXVal = boundingMinPt.x + (polyWidth / 2);
      int32_t maxBuildingXVal = boundingMaxPt.x - (polyWidth / 2);
      int32_t minBuildingYVal = boundingMinPt.y + (polyHeight / 2);
      int32_t maxBuildingYVal = boundingMaxPt.y - (polyHeight / 2);

      if (minBuildingXVal > maxBuildingXVal) {
        eastl::swap(minBuildingXVal, maxBuildingXVal);
      }

      if (minBuildingYVal > maxBuildingYVal) {
        eastl::swap(minBuildingYVal, maxBuildingYVal);
      }

      buildingPos.x = cx::getRandomIntUniformly(minBuildingXVal,
                                                maxBuildingXVal);
      buildingPos.y = cx::getRandomIntUniformly(minBuildingYVal,
                                                maxBuildingYVal);

      buildingRect.x = buildingPos.x;
      buildingRect.y = buildingPos.y;

      solution.setBuildingXPos(i, buildingPos.x);
      solution.setBuildingYPos(i, buildingPos.y);
      solution.setBuildingAngle(i, buildingAngle);
    }

    return solution;
  }

  cx::TreeNode<cx::Polygon<4>, 4>*
  findUsableRegion(cx::TreeNode<cx::Polygon<4>, 4>* node,
                   float minWidth,
                   float minHeight)
  {
    if (node->isLeaf()) {
      cx::Polygon<4> regionPoly = node->data;
      float regionPolyWidth = regionPoly.vertices[1].x
                              - regionPoly.vertices[0].x;
      float regionPolyHeight = regionPoly.vertices[3].y
                               - regionPoly.vertices[0].y;

      // We need a buffer since we do not buildings to be intersecting by
      // exactly 1 pixel.
      if (cx::floatLessEqual(regionPolyWidth, minWidth)
          || cx::floatLessEqual(regionPolyHeight, minHeight)) {
        // Our polygon is not gonna fit here. Skip.
        return nullptr;
      } else {
        return node;
      }
    }

    eastl::vector<int32_t> childIndexes{ 0, 1, 2, 3 };
    eastl::vector<float> regionArea;
    for (int32_t i : eastl::vector{ 0, 1, 2, 3 }) {
      cx::Polygon<4>& poly = node->getChild(i)->data;

      const float polyWidth = poly.vertices[1].x - poly.vertices[0].x;
      const float polyHeight = poly.vertices[3].y - poly.vertices[0].y;

      regionArea.push_back(polyWidth * polyHeight);
    }

    cx::TreeNode<cx::Polygon<4>, 4>* suitableRegionPtr = nullptr;
    do {
      int32_t randIdx = cx::selectRandomWeightedIndex(regionArea);

      auto childIter = childIndexes.begin();
      auto areaIter = regionArea.begin();
      eastl::advance(childIter, randIdx);
      eastl::advance(areaIter, randIdx);

      int32_t childIdx = *childIter;
      childIndexes.erase(childIter);
      regionArea.erase(areaIter);

      suitableRegionPtr = findUsableRegion(node->getChild(childIdx),
                                           minWidth,
                                           minHeight);
    } while (suitableRegionPtr == nullptr && !childIndexes.empty());

    return suitableRegionPtr;
  }
}
