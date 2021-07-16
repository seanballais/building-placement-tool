#ifndef COREX_CORE_MATH_FUNCTIONS_HPP
#define COREX_CORE_MATH_FUNCTIONS_HPP

#include <cassert>
#include <cstdlib>
#include <limits>
#include <type_traits>

#include <EASTL/algorithm.h>
#include <EASTL/array.h>
#include <EASTL/vector.h>
#include <EASTL/utility.h>
#include <SDL_gpu.h>

#include <corex/core/ReturnValue.hpp>
#include <corex/core/utils.hpp>
#include <corex/core/ds/Line.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Polygon.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/ds/Vec2.hpp>
#include <corex/core/ds/VecN.hpp>

namespace corex::core
{
  constexpr double pi = 3.14159265358979323846;

  bool floatEquals(float x, float y,
                   float tolerance = std::numeric_limits<float>::epsilon());
  bool floatAbsEquals(float x, float y,
                      float tolerance = std::numeric_limits<float>::epsilon());
  bool floatRelEquals(float x, float y,
                      float tolerance = std::numeric_limits<float>::epsilon());
  bool floatGreEqual(float x, float y,
                     float tolerance = std::numeric_limits<float>::epsilon());
  bool floatAbsGreEqual(float x, float y,
                        float tolerance = std::numeric_limits<float>
                                             ::epsilon());
  bool floatRelGreEqual(float x, float y,
                        float tolerance = std::numeric_limits<float>
                                             ::epsilon());
  bool floatLessEqual(float x, float y,
                      float tolerance = std::numeric_limits<float>::epsilon());
  bool floatAbsLessEqual(float x, float y,
                         float tolerance = std::numeric_limits<float>
                                              ::epsilon());
  bool floatRelLessEqual(float x, float y,
                         float tolerance = std::numeric_limits<float>
                                              ::epsilon());
  bool floatGreater(float x, float y,
                    float tolerance = std::numeric_limits<float>::epsilon());
  bool floatAbsGreater(float x, float y,
                       float tolerance = std::numeric_limits<float>::epsilon());
  bool floatRelGreater(float x, float y,
                       float tolerance = std::numeric_limits<float>::epsilon());
  bool floatLessThan(float x, float y,
                    float tolerance = std::numeric_limits<float>::epsilon());
  bool floatAbsLessThan(float x, float y,
                        float tolerance = std::numeric_limits<float>
                                             ::epsilon());
  bool floatRelLessThan(float x, float y,
                        float tolerance = std::numeric_limits<float>
                                             ::epsilon());
  float setDecPlaces(float n, int32_t numDecPlaces);
  double setDecPlaces(double n, int32_t numDecPlaces);
  bool isFloatInclusiveBetween(float a, float val, float b);
  bool isFloatIncExcBetween(float a, float val, float b);

  template <class T>
  T clamp(T val, T min, T max)
  {
    if (val < min) {
      return min;
    } else if (val > max) {
      return max;
    }

    return val;
  }

  int32_t abs(int32_t n);
  float abs(float n);
  int32_t factorial(int32_t n);
  int32_t mod(int32_t x, int32_t divisor);
  float mod(float x, float divisor);
  int32_t pow(int32_t base, int32_t exponent);
  float pow(float base, int32_t exponent);
  double pow(double base, int32_t exponent);

  float degreesToRadians(float degrees);
  float radiansToDegrees(float radians);
  float distance2D(const Point& start, const Point& end);
  float lineLength(const Line& line);
  float lineSlope(const Line& line);
  Line longestLine(const eastl::vector<Line*> lines);

  float det3x3(const Vec2& v0, const Vec2& v1, const Vec2& v2);
  float vec2Magnitude(const Vec2& p);
  float vec2Angle(const Vec2& p);
  float dotProduct(const Vec2& p, const Vec2& q);
  float crossProduct(const Vec2& p, const Vec2& q);
  float angleBetweenTwoVectors(const Vec2& p, const Vec2& q);
  Vec2 rotateVec2(const Vec2& p, float angle);
  Vec2 projectVec2(const Vec2& p, const Vec2& q);
  Vec2 vec2Perp(const Vec2& p);
  Vec2 translateVec2(const Vec2& vec, float deltaX, float deltaY);
  Vec2 minVec2Magnitude(const eastl::vector<Vec2*> vectors);
  Vec2 maxVec2Magnitude(const eastl::vector<Vec2*> vectors);
  Vec2 unitVector(const Vec2& vec);
  Vec2 lineToVec(const Line& line);
  Vec2 lineDirectionVector(const Line& line);
  Vec2 lineNormalVector(const Line& line);
  VecN pairwiseMult(const VecN& p, const VecN& q);
  VecN pairwiseSubt(const VecN& p, const float& a);
  VecN pairwiseSubt(const float& a, const VecN& p);
  VecN vecNAbs(const VecN& vec);
  VecN translateVecN(const VecN& vec, const VecN& deltaVec);

  template <typename... Args>
  inline constexpr auto rotatePoint(Args&&... args)
  -> decltype(rotateVec2(eastl::forward<Args>(args)...))
  {
    return rotateVec2(eastl::forward<Args>(args)...);
  }

  template <typename... Args>
  inline constexpr auto projectPoint(Args&&... args)
  -> decltype(projectVec2(eastl::forward<Args>(args)...))
  {
    return projectVec2(eastl::forward<Args>(args)...);
  }

  template <typename... Args>
  inline constexpr auto translatePoint(Args&&... args)
  -> decltype(translateVec2(eastl::forward<Args>(args)...))
  {
    return translateVec2(eastl::forward<Args>(args)...);
  }

  float signedDistPointToInfLine(const Point& point, const Line& line);

  Polygon<4> createRectangle(const Point& startingPoint,
                             float width,
                             float height);
  Polygon<4> createRectangle(const NPolygon& nPolygon);
  Polygon<4> rotateRectangle(float centerX, float centerY, float width,
                             float height, float angle);
  Polygon<4> rotateRectangle(const Rectangle& rect);
  Polygon<4> convertRectangleToPolygon(const Rectangle& rect);
  bool areTwoRectsIntersecting(const Rectangle& rect0, const Rectangle& rect1);
  bool areTwoRectsAABBIntersecting(const Rectangle& rect0,
                                   const Rectangle& rect1);
  ReturnValue<Point> intersectionOfTwoInfLines(const Line& line0,
                                               const Line& line1);
  ReturnValue<Point> intersectionOfLineandInfLine(const Line& line,
                                                  const Line& infLine);
  ReturnValue<Point> intersectionOfLineAndLine(const Line& line0,
                                               const Line& line1);
  bool areTwoLinesIntersecting(const Line& line0, const Line& line1);
  NPolygon clippedPolygonFromTwoRects(const Rectangle& targetRect,
                                      const Rectangle& clippingRect);
  Point getRandomPointInTriangle(const Polygon<3>& triangle);
  Point getPolygonCentroid(const NPolygon& polygon);
  int32_t getNumNPolygonSides(const NPolygon& polygon);
  bool isPointWithinNPolygon(const Point& point, const NPolygon& polygon);
  bool isRectWithinNPolygon(const Rectangle& rect, const NPolygon& polygon);
  bool isRectWithinNPolygonAABB(const Rectangle& rect, const NPolygon& polygon);
  bool isRectWithinRectAABB(const Rectangle& insideRect,
                            const Rectangle& outsideRect);
  bool isRectIntersectingNPolygon(const Rectangle& rect,
                                  const NPolygon& polygon);
  Polygon<4> getIntersectingRectAABB(const Rectangle& rect0,
                                     const Rectangle& rect1);
  eastl::vector<Polygon<3>> earClipTriangulate(NPolygon polygon);
  eastl::vector<int32_t> findEarVertexIndexes(const NPolygon& polygon);
  eastl::vector<int32_t> findConvexVertexIndexes(const NPolygon& polygon);
  eastl::vector<int32_t> findReflexVertexIndexes(const NPolygon& polygon);
  eastl::vector<float> computePolygonInteriorAngles(const NPolygon& polygon);
  bool isVertexAnEarInPolygon(const int32_t & vertexIndex,
                              const NPolygon& polygon);

  template <class P>
  eastl::array<Point, 3> findConvexHull(const P& polygon)
  {
    // Find three vertices that is representative of the polygon's convex hull.
    // Refer to: https://en.wikipedia.org/wiki/
    //                   Curve_orientation#Practical_considerations
    auto& vertices = polygon.vertices;
    auto chVertexIter = eastl::min_element(
      vertices.begin(),
      vertices.end(),
      [](const Point& a, const Point& b) -> bool {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
      }
    );
    int32_t chVertexIndex = chVertexIter - vertices.begin();
    int32_t prevCHVertexIndex = mod(chVertexIndex - 1, vertices.size());
    int32_t nextCHVertexIndex = mod(chVertexIndex + 1, vertices.size());

    return {
      vertices[prevCHVertexIndex],
      vertices[chVertexIndex],
      vertices[nextCHVertexIndex]
    };
  }

  template <class P>
  double getPolygonArea(const P& polygon)
  {
    // Let's use the Shoelace algorithm.
    double area = 0.f;
    auto& vertices = polygon.vertices;
    auto convexHull = findConvexHull(polygon);
    float determinant = det3x3(convexHull[0], convexHull[1], convexHull[2]);
    for (int32_t i = 0; i < polygon.vertices.size(); i++) {
      int32_t currVertIndex = 0;
      int32_t nextVertIndex = 0;
      if (floatGreEqual(determinant, 0.f)) {
        // Polygon is oriented counterclockwise when the origin is in the
        // center.
        currVertIndex = i;
        nextVertIndex = mod(currVertIndex + 1, vertices.size());
      } else {
        // Polygon is oriented clockwise when the origin is in the
        // center.
        currVertIndex = vertices.size() - (i + 1);
        nextVertIndex = mod(currVertIndex - 1, vertices.size());
      }
      area +=
        (static_cast<double>(vertices[currVertIndex].x)
         * static_cast<double>(vertices[nextVertIndex].y))
        - (static_cast<double>(vertices[nextVertIndex].x)
           * static_cast<double>(vertices[currVertIndex].y));
    }

    return fabs(area) / 2.0;
  }

  template <uint32_t numVertices>
  eastl::array<Line, numVertices>
  convertPolygonToLines(const Polygon<numVertices>& polygon)
  {
    // The polygon passed is assumed to contain the points in a consecutive
    // manner.
    eastl::array<Line, numVertices> lines;
    for (int32_t i = 0; i < numVertices; i++) {
      lines[i] = Line{polygon.vertices[i],
                      polygon.vertices[(i + 1) % numVertices]};
    }

    return lines;
  }

  template <uint32_t numVertices>
  NPolygon convertPolygonToNPolygon(const Polygon<numVertices>& polygon)
  {
    NPolygon nPolygon;
    for (auto& vertex : polygon.vertices) {
      nPolygon.vertices.push_back(vertex);
    }

    return nPolygon;
  }

  // Disallow weights that are non-float.
  template <
    template<class, class> class V, class T, class VAllocator,
    template<class, class> class W, class U, class WAllocator
  > const T& selectRandomItemWithWeights(
      const V<T, VAllocator>& items,
      const W<U, WAllocator>& weights) = delete;

  // We should not allow rvalues for the items parameter, because we're
  // returning a reference to an item there.
  template <
    template<class, class> class V, class T, class VAllocator,
    template<class, class> class W, class U, class WAllocator
  > const T& selectRandomItemWithWeights(
      const V<T, VAllocator>&& items,
      const W<U, WAllocator>& weights) = delete;

  template <
    template<class, class> class V, class T, class VAllocator,
    template<class, class> class W, class WAllocator
  > const T& selectRandomItemWithWeights(const V<T, VAllocator>& items,
                                         const W<float, WAllocator>& weights)
  {
    // Algorithm based on:
    //   https://softwareengineering.stackexchange.com/a/150618/208923
    float weightSum = std::accumulate(weights.begin(), weights.end(), 0);
    std::uniform_real_distribution nDistrib{ 0.f, weightSum };
    float n = generateRandomReal(nDistrib);
    int32_t currIntervalVal = 0;
    int32_t selectedItemIndex = 0;
    for (int32_t i = 0; i < weights.size(); i++) {
      currIntervalVal += weights[i];
      if (floatGreEqual(currIntervalVal, n)) {
        selectedItemIndex = i;
        break;
      }
    }

    return items[selectedItemIndex];
  }

  template <
    template<class, class> class V, class T, class VAllocator
  > const T selectItemRandomly(const V<T, VAllocator>& items)
  {
    eastl::vector<float> weights(items.size(), 1.f);
    return selectRandomItemWithWeights(items, weights);
  }

  template <template<class, class> class W, class WAllocator>
  int32_t selectRandomWeightedIndex(const W<float, WAllocator>& weights)
  {
    // Algorithm based on:
    //   https://softwareengineering.stackexchange.com/a/150618/208923
    float weightSum = std::accumulate(weights.begin(), weights.end(), 0);
    std::uniform_real_distribution nDistrib{ 0.f, weightSum };
    float n = generateRandomReal(nDistrib);
    int32_t currIntervalVal = 0;
    int32_t selectedItemIndex = 0;
    for (int32_t i = 0; i < weights.size(); i++) {
      currIntervalVal += weights[i];
      if (floatGreEqual(currIntervalVal, n)) {
        selectedItemIndex = i;
        break;
      }
    }

    return selectedItemIndex;
  }

  template <class RealType>
  std::normal_distribution<RealType> multiplyDistributions(
      const std::normal_distribution<RealType>& distribA,
      const std::normal_distribution<RealType>& distribB) {
    double newMean = (
      ((distribA.mean() * pow(distribB.stddev(), 2))
      + ((distribB.mean() * pow(distribA.stddev(), 2))))
      / (pow(distribB.stddev(), 2) + pow(distribA.stddev(), 2))
    );
    double newStdDev = sqrt(
      (pow(distribA.stddev(), 2) * pow(distribB.stddev(), 2))
      / (pow(distribA.stddev(), 2) + pow(distribB.stddev(), 2))
    );
    return std::normal_distribution<RealType>{ newMean, newStdDev };
  }
}

namespace cx
{
  using namespace corex::core;
}

#endif
