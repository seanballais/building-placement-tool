#include <random>

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
#include <bpt/operators.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution generateRandomSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea)
  {
    constexpr int32_t numTreeNodeChildren = 4;
    cx::Tree<cx::Polygon<4>, numTreeNodeChildren> boundingRegions;
    auto* currRegionNode = boundingRegions.getRoot();
    currRegionNode->data = cx::createRectangle(boundingArea);

    Solution solution{ static_cast<int32_t>(inputBuildings.size()) };

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

    std::cout << "Shuffled Building Indexes\n";
    for (const int32_t& i : shuffledBuildingIndexes) {
      std::cout << i << " ";
    }
    std::cout << "\n";

    // Put in the buildings!
    for (const int32_t& i : shuffledBuildingIndexes) {
      currRegionNode = boundingRegions.getRoot();

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

      // Determine which bounding region to put the building in.
      eastl::unordered_set<int32_t> accessedChildren;
      while (!currRegionNode->isLeaf()) {
        int32_t childIdx = cx::getRandomIntUniformly(0,
                                                     numTreeNodeChildren - 1);
        accessedChildren.insert(childIdx);
        currRegionNode = currRegionNode->getChild(childIdx);

        std::cout << "Selected child " << childIdx << "\n";

        cx::Polygon<4> regionPoly = currRegionNode->data;
        float regionPolyWidth = regionPoly.vertices[1].x
                                - regionPoly.vertices[0].x;
        float regionPolyHeight = regionPoly.vertices[3].y
                                 - regionPoly.vertices[0].y;

        std::cout << "==========================\n";
        std::cout << "Region W: " << regionPolyWidth << " "
                  << "Region H: " << regionPolyHeight << "\n";
        std::cout << "Poly W: " << polyWidth << " "
                  << "Poly H: " << polyHeight << "\n";
        std::cout << "(" << regionPoly.vertices[0].x << ", "
                  << regionPoly.vertices[0].y << ")\n";
        std::cout << "(" << regionPoly.vertices[1].x << ", "
                  << regionPoly.vertices[1].y << ")\n";
        std::cout << "(" << regionPoly.vertices[2].x << ", "
                  << regionPoly.vertices[2].y << ")\n";
        std::cout << "(" << regionPoly.vertices[3].x << ", "
                  << regionPoly.vertices[3].y << ")\n";
        std::cout << "==========================\n";

        // We need a buffer since we do not buildings to be intersecting by
        // exactly 1 pixel.
        if (cx::floatLessEqual(regionPolyWidth, polyWidth)
            || cx::floatLessEqual(regionPolyHeight, polyHeight)) {
          // Our polygon is not gonna fit here. Skip.
          // TODO: Go back to the upper level and find a new region if the
          //       regions can't fit the building.
          currRegionNode = currRegionNode->parentPtr;

          if (accessedChildren.size() == numTreeNodeChildren) {
            // We exhausted the children already. Move back up.
            currRegionNode = currRegionNode->parentPtr;
            accessedChildren.clear();

            std::cout << "Went back to parent's parent.\n";
          } else {
            std::cout << "Went back to parent.\n";
          }

          continue;
        }
      }

      // Find possible location for the building in the region.
      const cx::Point& regionMinPt = currRegionNode->data.vertices[0];
      const cx::Point& regionMaxPt = currRegionNode->data.vertices[2];

      // We need a buffer since we do not buildings to be intersecting by
      // exactly 1 pixel.
      int32_t minBuildingXVal = (regionMinPt.x + (polyWidth / 2)) + 1;
      int32_t maxBuildingXVal = (regionMaxPt.x - (polyWidth / 2)) - 1;
      int32_t minBuildingYVal = (regionMinPt.y + (polyHeight / 2)) + 1;
      int32_t maxBuildingYVal = (regionMaxPt.y - (polyHeight / 2)) - 1;

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

      cx::Point buildingDelta{ polyWidth / 2, polyHeight / 2 };
      cx::Point buildingMinPt = buildingPos - buildingDelta;
      cx::Point buildingMaxPt = buildingPos + buildingDelta;

      // Subdivide the current region.
      int32_t sliceTechniqueID = cx::getRandomIntUniformly(0, 1);
      cx::Polygon<4> leftSubRegion;
      cx::Polygon<4> topSubRegion;
      cx::Polygon<4> bottomSubRegion;
      cx::Polygon<4> rightSubRegion;
      if (sliceTechniqueID == 0) {
        // Slice vertically.
        // Create the left subregion.
        leftSubRegion = {
          regionMinPt,
          cx::Point{ buildingMinPt.x, regionMinPt.y },
          cx::Point{ buildingMinPt.x, regionMaxPt.y },
          cx::Point{ regionMinPt.x, regionMaxPt.y }
        };

        // Create the top subregion.
        topSubRegion = {
          cx::Point{ buildingMinPt.x, regionMinPt.y },
          cx::Point{ buildingMaxPt.x, regionMinPt.y },
          cx::Point{ buildingMaxPt.x, buildingMinPt.y },
          buildingMinPt
        };

        // Create the bottom subregion.
        bottomSubRegion = {
          cx::Point{ buildingMinPt.x, buildingMaxPt.y },
          buildingMaxPt,
          cx::Point{ buildingMaxPt.x, regionMaxPt.y },
          cx::Point{ buildingMinPt.x, regionMaxPt.y }
        };

        // Create the right subregion.
        rightSubRegion = {
          cx::Point{ buildingMaxPt.x, regionMinPt.y },
          cx::Point{ regionMaxPt.x, regionMinPt.y },
          regionMaxPt,
          cx::Point{ buildingMaxPt.x, regionMaxPt.y }
        };
      } else {
        // Slice horizontally.

        // Create the top subregion.
        topSubRegion = {
          regionMinPt,
          cx::Point{ regionMaxPt.x, regionMinPt.y },
          cx::Point{ regionMaxPt.x, buildingMinPt.y },
          cx::Point{ regionMinPt.x, buildingMinPt.y }
        };

        // Create the left subregion.
        leftSubRegion = {
          cx::Point{ regionMinPt.x, buildingMinPt.y },
          buildingMinPt,
          cx::Point{ buildingMinPt.x, buildingMaxPt.y },
          cx::Point{ regionMinPt.x, buildingMaxPt.y }
        };

        // Create the right subregion.
        rightSubRegion = {
          cx::Point{ buildingMaxPt.x, buildingMinPt.y },
          cx::Point{ regionMaxPt.x, buildingMinPt.y },
          cx::Point{ regionMaxPt.x, buildingMaxPt.y },
          buildingMaxPt
        };

        // Create the bottom subregion.
        bottomSubRegion = {
          cx::Point{ regionMinPt.x, buildingMaxPt.y },
          cx::Point{ regionMaxPt.x, buildingMaxPt.y },
          regionMaxPt,
          cx::Point{ regionMinPt.x, regionMaxPt.y }
        };
      }

      currRegionNode->setChildValue(0, topSubRegion);
      currRegionNode->setChildValue(1, leftSubRegion);
      currRegionNode->setChildValue(2, rightSubRegion);
      currRegionNode->setChildValue(3, bottomSubRegion);
    }

    return solution;
  }
}
