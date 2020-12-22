#ifndef COREX_CORE_MATH_FUNCTIONS_HPP
#define COREX_CORE_MATH_FUNCTIONS_HPP

#include <cstdlib>
#include <limits>

#include <EASTL/array.h>
#include <EASTL/vector.h>
#include <EASTL/utility.h>
#include <SDL_gpu.h>

#include <corex/core/ReturnValue.hpp>
#include <corex/core/ds/Line.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Polygon.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/ds/Vec2.hpp>

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

  int32_t factorial(int32_t n);
  int32_t pyModInt32(int32_t x, int32_t divisor);
  int32_t pow(int32_t base, int32_t exponent);
  float pow(float base, int32_t exponent);

  float degreesToRadians(float degrees);
  float radiansToDegrees(float radians);
  float distance2D(const Point& start, const Point& end);
  float lineLength(const Line& line);
  Line longestLine(const eastl::vector<Line*> lines);

  float det3x3(const Vec2& v0, const Vec2& v1, const Vec2& v2);
  float vec2Magnitude(const Vec2& p);
  float vec2Angle(const Vec2& p);
  float dotProduct(const Vec2& p, const Vec2& q);
  float crossProduct(const Vec2& p, const Vec2& q);
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

  Polygon<4> rotateRectangle(float centerX, float centerY, float width,
                             float height, float angle);
  Polygon<4> rotateRectangle(const Rectangle& rect);
  Polygon<4> convertRectangleToPolygon(const Rectangle& rect);
  bool areTwoRectsIntersecting(const Rectangle& rect0, const Rectangle& rect1);
  ReturnValue<Point> intersectionOfTwoInfLines(const Line& line0,
                                               const Line& line1);
  ReturnValue<Point> intersectionOfLineandInfLine(const Line& line,
                                                  const Line& infLine);
  ReturnValue<Point> intersectionOfLineAndLine(const Line& line0,
                                               const Line& line1);
  bool areTwoLinesIntersecting(const Line& line0, const Line& line1);
  NPolygon clippedPolygonFromTwoRects(const Rectangle& targetRect,
                                      const Rectangle& clippingRect);
  Point getPolygonCentroid(const NPolygon& polygon);
  double getPolygonArea(const NPolygon& polygon);
  bool isPointWithinNPolygon(const Point& point, const NPolygon& polygon);
  bool isRectWithinNPolygon(const Rectangle& rect, const NPolygon& polygon);
  bool isRectIntersectingNPolygon(const Rectangle& rect,
                                  const NPolygon& polygon);

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
}

namespace cx
{
  using namespace corex::core;
}

#endif
