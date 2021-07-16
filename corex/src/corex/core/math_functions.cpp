#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>

#include <EASTL/set.h>
#include <EASTL/vector.h>

#include <corex/core/math_functions.hpp>
#include <corex/core/ReturnState.hpp>
#include <corex/core/ReturnValue.hpp>
#include <corex/core/utils.hpp>
#include <corex/core/ds/Line.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/ds/Vec2.hpp>
#include <corex/core/ds/VecN.hpp>

namespace corex::core {
  // Functions and that should only be accessible here.
  bool _areTwoRectsCollidingInAnAxis(const Rectangle &rect0,
                                     const Rectangle &rect1,
                                     const Vec2 &axis);

  Line _projectRectToAnAxis(const Rectangle &rect, const Vec2 &axis);

  bool _areTwoRectsCollidingInAnAxis(const Rectangle &rect0,
                                     const Rectangle &rect1,
                                     const Vec2 &axis) {
    Line projLine0 = _projectRectToAnAxis(rect0, axis);
    Line projLine1 = _projectRectToAnAxis(rect1, axis);

    // We need to find the line that merges both segments together (whether
    // or not they are intersecting). We could have used a loop here for
    // generalization, but we don't need to do so for now. So, no loops.
    Line possibleRectLine0 = Line{projLine0.start, projLine0.end};
    Line possibleRectLine1 = Line{projLine0.start, projLine1.start};
    Line possibleRectLine2 = Line{projLine0.start, projLine1.end};
    Line possibleRectLine3 = Line{projLine0.end, projLine1.start};
    Line possibleRectLine4 = Line{projLine0.end, projLine1.end};
    Line possibleRectLine5 = Line{projLine1.start, projLine1.end};

    Line projRectLine = longestLine({
                                      &possibleRectLine0,
                                      &possibleRectLine1,
                                      &possibleRectLine2,
                                      &possibleRectLine3,
                                      &possibleRectLine4,
                                      &possibleRectLine5
                                    });

    return floatLessEqual(lineLength(projRectLine),
                          lineLength(projLine0) + lineLength(projLine1));
  }

  Line _projectRectToAnAxis(const Rectangle &rect, const Vec2 &axis) {
    auto rotatedRect = rotateRectangle(rect);
    Point topLeftPt = rotatedRect.vertices[0];
    Point topRightPt = rotatedRect.vertices[1];
    Point bottomLeftPt = rotatedRect.vertices[3];
    Point bottomRightPt = rotatedRect.vertices[2];
    Point projectedTopLeftPt = projectVec2(topLeftPt, axis);
    Point projectedTopRightPt = projectVec2(topRightPt, axis);
    Point projectedBottomLeftPt = projectVec2(bottomLeftPt, axis);
    Point projectedBottomRightPt = projectVec2(bottomRightPt, axis);

    Line possibleRectLine0{projectedTopLeftPt, projectedTopRightPt};
    Line possibleRectLine1{projectedTopLeftPt, projectedBottomLeftPt};
    Line possibleRectLine2{projectedTopLeftPt, projectedBottomRightPt};
    Line possibleRectLine3{projectedTopRightPt, projectedBottomLeftPt};
    Line possibleRectLine4{projectedTopRightPt, projectedBottomRightPt};
    Line possibleRectLine5{projectedBottomLeftPt, projectedBottomRightPt};

    return longestLine({
                         &possibleRectLine0,
                         &possibleRectLine1,
                         &possibleRectLine2,
                         &possibleRectLine3,
                         &possibleRectLine4,
                         &possibleRectLine5
                       });
  }
  /////////////////////////////////////////////////

  bool floatEquals(float x, float y, float tolerance) {
    return fabs(x - y)
           <= (tolerance * std::max(std::max(fabs(x), fabs(y)), 1.0f));
  }

  bool floatAbsEquals(float x, float y, float tolerance) {
    return fabs(x - y) <= tolerance;
  }

  bool floatRelEquals(float x, float y, float tolerance) {
    return fabs(x - y) <= (tolerance * std::max(fabs(x), fabs(y)));
  }

  bool floatGreEqual(float x, float y, float tolerance) {
    return (x > y) || floatEquals(x, y, tolerance);
  }

  bool floatAbsGreEqual(float x, float y, float tolerance) {
    return (x > y) || floatAbsEquals(x, y, tolerance);
  }

  bool floatRelGreEqual(float x, float y, float tolerance) {
    return (x > y) || floatRelEquals(x, y, tolerance);
  }

  bool floatLessEqual(float x, float y, float tolerance) {
    return (x < y) || floatEquals(x, y, tolerance);
  }

  bool floatAbsLessEqual(float x, float y, float tolerance) {
    return (x < y) || floatAbsEquals(x, y, tolerance);
  }

  bool floatRelLessEqual(float x, float y, float tolerance) {
    return (x < y) || floatRelEquals(x, y, tolerance);
  }

  bool floatGreater(float x, float y, float tolerance) {
    return !floatEquals(x, y, tolerance) ? x > y : false;
  }

  bool floatAbsGreater(float x, float y, float tolerance) {
    return !floatAbsEquals(x, y, tolerance) ? x > y : false;
  }

  bool floatRelGreater(float x, float y, float tolerance) {
    return !floatRelEquals(x, y, tolerance) ? x > y : false;
  }

  bool floatLessThan(float x, float y, float tolerance) {
    return !floatEquals(x, y, tolerance) ? x < y : false;
  }

  bool floatAbsLessThan(float x, float y, float tolerance) {
    return !floatAbsEquals(x, y, tolerance) ? x < y : false;
  }

  bool floatRelLessThan(float x, float y, float tolerance) {
    return !floatRelEquals(x, y, tolerance) ? x < y : false;
  }

  float setDecPlaces(float n, int32_t numDecPlaces) {
    // We're using a custom cx::pow() because std::pow() is slow.
    int32_t multiplier = cx::pow(10, numDecPlaces);
    return roundf(n * multiplier) / multiplier;
  }

  double setDecPlaces(double n, int32_t numDecPlaces) {
    int32_t multiplier = cx::pow(10, numDecPlaces);
    return roundf(n * multiplier) / multiplier;
  }

  bool isFloatInclusiveBetween(float a, float val, float b) {
    if (floatGreater(a, b)) {
      std::swap(a, b);
    }

    return floatLessEqual(a, val) && floatLessEqual(val, b);
  }

  bool isFloatIncExcBetween(float a, float val, float b)
  {
    if (floatGreater(a, b)) {
      std::swap(a, b);
    }

    return floatLessEqual(a, val) && floatLessThan(val, b);
  }

  int32_t abs(int32_t n) {
    if (n < 0) {
      return n * -1;
    } else {
      return n;
    }
  }

  float abs(float n)
  {
    return std::fabs(n);
  }

  int32_t factorial(int32_t n) {
    int32_t total = 1;
    for (; n > 1; n--) {
      total *= n;
    }

    return total;
  }

  int32_t mod(int32_t x, int32_t divisor) {
    // From: https://stackoverflow.com/a/44197900/1116098
    return (divisor + (x % divisor)) % divisor;
  }

  float mod(float x, float divisor)
  {
    return std::fmod(divisor + std::fmod(x, divisor), divisor);
  }

  int32_t pow(int32_t base, int32_t exponent)
  {
    assert(exponent >= 0);

    // Note that std::pow() is slow. So, we're using a custom implementation
    // of a basic pow() for integers.
    if (exponent == 0) {
      return 1;
    } else if (base == 0) {
      return 0;
    } else {
      int result = 1;
      for (int i = 0; i < exponent; i++) {
        result *= base;
      }

      return result;
    }
  }

  float pow(float base, int32_t exponent)
  {
    assert(exponent >= 0);

    // Note that std::pow() is slow. So, we're using a custom implementation
    // of a basic pow() for integers.
    if (exponent == 0) {
      return 1.f;
    } else if (floatEquals(base, 0.f)) {
      return 0.f;
    } else {
      double result = 1; // Let's not lose precision during computation.
      for (int i = 0; i < exponent; i++) {
        result *= static_cast<double>(base);
      }

      return static_cast<float>(result);
    }
  }

  double pow(double base, int32_t exponent)
  {
    assert(exponent >= 0);

    // Note that std::pow() is slow. So, we're using a custom implementation
    // of a basic pow() for integers.
    if (exponent == 0) {
      return 1.f;
    } else if (floatEquals(base, 0.f)) {
      return 0.f;
    } else {
      double result = 1; // Let's not lose precision during computation.
      for (int i = 0; i < exponent; i++) {
        result *= static_cast<double>(base);
      }

      return static_cast<float>(result);
    }
  }

  float degreesToRadians(float degrees)
  {
    return setDecPlaces(degrees * (pi / 180.0), 6);
  }

  float radiansToDegrees(float radians)
  {
    return radians * (180.0 / pi);
  }

  float distance2D(const Point& start, const Point& end)
  {
    return sqrt(cx::pow(end.x - start.x, 2) + cx::pow(end.y - start.y, 2));
  }

  float lineLength(const Line& line)
  {
    return distance2D(line.start, line.end);
  }

  float lineSlope(const Line& line)
  {
    return (line.end.y - line.start.y) / (line.end.x - line.start.x);
  }

  Line longestLine(const eastl::vector<Line*> lines)
  {
    // We don't want copies and references can't hold null objects.
    Line* longestLine = nullptr;
    bool isFirstElement = true;
    for (Line* line : lines) {
      if (isFirstElement
          || floatGreater(lineLength(*line), lineLength(*longestLine))) {
        // Let's utilize boolean short-circuit.
        isFirstElement = false;
        longestLine = line;
      }
    }

    return *longestLine;
  }

  float det3x3(const Vec2& v0, const Vec2& v1, const Vec2& v2)
  {
    return ((v1.x * v2.y) + (v0.x * v1.y) + (v0.y * v2.x))
           - ((v0.y * v1.x) + (v1.y * v2.x) + (v0.x * v2.y));
  }

  float vec2Magnitude(const Vec2& p)
  {
    return distance2D(Vec2{0.f, 0.f}, p);
  }

  float vec2Angle(const Vec2& p)
  {
    // Based on: https://stackoverflow.com/a/48227232/1116098
    // Returns the angle in degrees. Note also that we are using the
    // standard Cartesian coordinate plane, not the window space Cartesian
    // coordinate plane where the origin is on the top-left.
    // Let's tackle the special cases first.
    if (floatEquals(p.x, 0.f)) {
      return floatGreEqual(p.y, 0.f) ? 90.f
           : floatEquals(p.y, 0.f) ? 0.f
           : 270.f;
    } else if (floatEquals(p.y, 0.f)) {
      return floatGreEqual(p.x, 0.f) ? 0
           : 180.f;
    }

    float rotDelta = radiansToDegrees(
      std::atanf(static_cast<float>(p.y / p.x)));
    if (floatLessThan(p.x, 0.f) && floatGreater(p.y, 0.f)) {
      // Quadrant II
      return 90 + rotDelta;
    } else if (floatLessThan(p.x, 0.f) && floatLessThan(p.y, 0.f)) {
      // Quadrant III
      return 180 + rotDelta;
    } else if (floatGreater(p.x, 0.f) && floatLessThan(p.y, 0.f)) {
      return 270 + rotDelta;
    } else {
      // Quadrant I
      return rotDelta;
    }
  }

  float dotProduct(const Vec2& p, const Vec2& q)
  {
    return (p.x * q.x) + (p.y * q.y);
  }

  float crossProduct(const Vec2& p, const Vec2& q)
  {
    return (p.x * q.y) - (p.y * q.x);
  }

  float angleBetweenTwoVectors(const Vec2& p, const Vec2& q)
  {
    return radiansToDegrees(
      acos(dotProduct(p, q) / (vec2Magnitude(p) * vec2Magnitude(q))));
  }

  Vec2 rotateVec2(const Vec2& p, float angle)
  {
    // The angle parameter is expected to be in degrees. And we subtract the
    // angle by 360 so that we can rotate the point counterclockwise, which is
    // the rotation direction we usually expect,
    float angleRadians = degreesToRadians(angle);
    return Vec2{
      setDecPlaces(
        (p.x * std::cos(angleRadians)) - (p.y * std::sin(angleRadians)),
        6),
      setDecPlaces(
        (p.x * std::sin(angleRadians)) + (p.y * std::cos(angleRadians)),
        6)
    };
  }

  Vec2 projectVec2(const Vec2& p, const Vec2& q)
  {
    return static_cast<float>(dotProduct(p, q) / cx::pow(vec2Magnitude(q), 2)) * q;
  }

  Vec2 vec2Perp(const Vec2& p)
  {
    // Vectors are arranged in a clockwise direction. This means that their
    // perpendiculars must be rotated counterclockwise. However, since the
    // origin is at the top left corner, rather than the bottom left, positive
    // angles will be rotated clockwise. As such, we have to use negative angles
    // to rotate the vectors counterclockwise to get their correct perpendicular
    // vectors.
    return rotateVec2(p, -90.f);
  }

  Vec2 translateVec2(const Vec2& vec, float deltaX, float deltaY)
  {
    return Vec2{vec.x + deltaX, vec.y + deltaY};
  }

  Vec2 minVec2Magnitude(const eastl::vector<Vec2*> vectors)
  {
    // We don't want copies and references can't hold null objects.
    Vec2* minVec = nullptr;
    bool isFirstElement = true;
    for (Vec2* vec : vectors) {
      if (isFirstElement
          || floatLessThan(vec2Magnitude(*vec), vec2Magnitude(*minVec))) {
        // Let's utilize boolean short-circuit.
        isFirstElement = false;
        minVec = vec;
      }
    }

    return *minVec;
  }

  Vec2 maxVec2Magnitude(const eastl::vector<Vec2*> vectors)
  {
    // We don't want copies and references can't hold null objects.
    Vec2* maxVec = nullptr;
    bool isFirstElement = true;
    for (Vec2* vec : vectors) {
      if (isFirstElement
          || floatGreater(vec2Magnitude(*vec), vec2Magnitude(*maxVec))) {
        // Let's utilize boolean short-circuit.
        isFirstElement = false;
        maxVec = vec;
      }
    }

    return *maxVec;
  }

  Vec2 unitVector(const Vec2& vec)
  {
    return vec / vec2Magnitude(vec);
  }

  Vec2 lineToVec(const Line& line)
  {
    return Vec2{ line.end.x - line.start.x, line.end.y - line.start.y };
  }

  Vec2 lineDirectionVector(const Line& line)
  {
    return unitVector(lineToVec(line));
  }

  Vec2 lineNormalVector(const Line& line)
  {
    // The vertices of lines and polygons are arranged in a clockwise direction.
    // This means that the unit vectors representing the direction of lines must
    // be rotated counterclockwise. However, since the origin is at the top left
    // corner, rather than the bottom left, positive angles will be rotated
    // clockwise. As such, we have to use negative angles to rotate the unit
    // vectors counterclockwise to get the proper normal vectors of lines.
    return rotateVec2(lineDirectionVector(line), -90.f);
  }

  VecN pairwiseMult(const VecN& p, const VecN& q)
  {
    // This is unlike dot product, which produces a scalar value.
    VecN r = p;
    for (int32_t i = 0; i < r.size(); i++) {
      r[i] *= q[i];
    }

    return r;
  }

  VecN pairwiseSubt(const VecN& p, const float& a)
  {
    VecN vec = p;
    for (float& f : vec) {
      f -= a;
    }

    return vec;
  }

  VecN pairwiseSubt(const float& a, const VecN& p)
  {
    VecN vec = p;
    for (float& f : vec) {
      f = a - f;
    }

    return vec;
  }

  VecN vecNAbs(const VecN& vec)
  {
    VecN w{static_cast<int32_t>(vec.size())};
    for (int32_t i = 0; i < w.size(); i++) {
      w[i] = std::fabs(vec[i]);
    }

    return w;
  }

  VecN translateVecN(const VecN& vec, const VecN& deltaVec)
  {
    assert(vec.size() == deltaVec.size());

    cx::VecN newVec(vec.size());
    for (int32_t i = 0; i < vec.size(); i++) {
      newVec[i] = vec[i] + deltaVec[i];
    }

    return newVec;
  }

  float signedDistPointToInfLine(const Point& point, const Line& line)
  {
    return setDecPlaces(dotProduct(lineNormalVector(line), (point - line.end)),
                        4);
  }

  Polygon<4> createRectangle(const Point& startingPoint,
                             float width,
                             float height)
  {
    return Polygon<4> {
      {
        startingPoint,
        startingPoint + cx::Point{ width, 0.f },
        startingPoint + cx::Point{ width, height },
        startingPoint + cx::Point{ 0.f, height },
      }
    };
  }

  Polygon<4> createRectangle(const NPolygon& nPolygon)
  {
    assert(nPolygon.vertices.size() == 4);

    const Point& minPt = nPolygon.vertices[0];
    const Point& maxPt = nPolygon.vertices[2];

    float width = maxPt.x - minPt.y;
    float height = maxPt.y - minPt.y;

    return Polygon<4>{
      minPt,
      Point{ maxPt.x, minPt.y },
      maxPt,
      Point{ minPt.x, maxPt.y }
    };
  }

  Polygon<4> rotateRectangle(float centerX, float centerY, float width,
                             float height, float angle)
  {
    // NOTE: Angle is expected to be in degrees.
    Point topLeftPt = Point{centerX - (width / 2.f), centerY - (height / 2.f)};
    Point topRightPt = Point{topLeftPt.x + width, topLeftPt.y};
    Point bottomLeftPt = Point{topLeftPt.x, topLeftPt.y + height};
    Point bottomRightPt = Point{topLeftPt.x + width, topLeftPt.y + height};

    // Center X and Y are effectively the distance of the rectangle from
    // its center to the origin
    Point transTopLeftPt = translatePoint(topLeftPt, -centerX, -centerY);
    Point transTopRightPt = translatePoint(topRightPt, -centerX, -centerY);
    Point transBottomLeftPt = translatePoint(bottomLeftPt, -centerX, -centerY);
    Point transBottomRightPt = translatePoint(bottomRightPt,
                                              -centerX,
                                              -centerY);

    Point rotatedTopLeftPt = rotatePoint(transTopLeftPt, angle);
    Point rotatedTopRightPt = rotatePoint(transTopRightPt, angle);
    Point rotatedBottomLeftPt = rotatePoint(transBottomLeftPt, angle);
    Point rotatedBottomRightPt = rotatePoint(transBottomRightPt, angle);

    // Set the vertex with the smallest x position *and then* smallest y
    // position as the top-left most vertex.
    eastl::array<Point, 4> tempPolygon{
      translatePoint(rotatedTopLeftPt, centerX, centerY),
      translatePoint(rotatedTopRightPt, centerX, centerY),
      translatePoint(rotatedBottomRightPt, centerX, centerY),
      translatePoint(rotatedBottomLeftPt, centerX, centerY)
    };

    // Make sure that the first element in the vertices list is the top-left
    // corner, and the rest are arranged in a clockwise manner.
    int32_t topLeftIdx = eastl::distance(
      tempPolygon.begin(),
      eastl::min_element(
        tempPolygon.begin(),
        tempPolygon.end(),
        [](const Point& a, Point& b) {
          return (a.x < b.x) || (floatEquals(a.x, b.x) && a.y < b.y);
        }));
    eastl::array<Point, 4> polygon = tempPolygon;
    for (int32_t i = 0, j = topLeftIdx;
         i < polygon.size();
         i++, j = (j + 1) % 4) {
      polygon[i] = tempPolygon[j];
    }

    return Polygon<4>{ polygon };
  }

  Polygon<4> rotateRectangle(const Rectangle& rect)
  {
    return rotateRectangle(rect.x, rect.y, rect.width, rect.height, rect.angle);
  }

  Polygon<4> convertRectangleToPolygon(const Rectangle& rect)
  {
    // Should we just forward this instead of performing a function call?
    return rotateRectangle(rect.x, rect.y, rect.width, rect.height, rect.angle);
  }

  bool areTwoRectsIntersecting(const Rectangle& rect0, const Rectangle& rect1)
  {
    // Oh, boy. Let's do some SAT (Separating Axis Theorem)!
    Vec2 axisX0 = rotateVec2(Vec2{1.f, 0.f}, rect0.angle);
    Vec2 axisY0 = rotateVec2(Vec2{0.f, 1.f}, rect0.angle);
    Vec2 axisX1 = rotateVec2(Vec2{1.f, 0.f}, rect1.angle);
    Vec2 axisY1 = rotateVec2(Vec2{0.f, 1.f}, rect1.angle);

    return _areTwoRectsCollidingInAnAxis(rect0, rect1, axisX0)
           && _areTwoRectsCollidingInAnAxis(rect0, rect1, axisY0)
           && _areTwoRectsCollidingInAnAxis(rect0, rect1, axisX1)
           && _areTwoRectsCollidingInAnAxis(rect0, rect1, axisY1);
  }

  bool areTwoRectsAABBIntersecting(const Rectangle& rect0,
                                   const Rectangle& rect1)
  {
    Polygon<4> rectangle0 = rotateRectangle(rect0);
    Polygon<4> rectangle1 = rotateRectangle(rect1);

    float rect0TopLeftX = rectangle0.vertices[0].x;
    float rect0TopLeftY = rectangle0.vertices[0].y;
    float rect1TopLeftX = rectangle1.vertices[0].x;
    float rect1TopLeftY = rectangle1.vertices[0].y;
    float rect0Width = rectangle0.vertices[1].x - rect0TopLeftX;
    float rect0Height = rectangle0.vertices[3].y - rect0TopLeftY;
    float rect1Width = rectangle1.vertices[1].x - rect1TopLeftX;
    float rect1Height = rectangle1.vertices[3].y - rect1TopLeftY;

    return (rect0TopLeftX < rect1TopLeftX + rect1Width
            && rect0TopLeftX + rect0Width >= rect1TopLeftX
            && rect0TopLeftY < rect1TopLeftY + rect1Height
            && rect0TopLeftY + rect0Height >= rect1TopLeftY)
           || isRectWithinRectAABB(rect0, rect1)
           || isRectWithinRectAABB(rect1, rect0);
  }

  ReturnValue<Point> intersectionOfTwoInfLines(const Line& line0,
                                               const Line& line1)
  {
    // We're using the vector form of a line equation since it's easier to
    // determine whether two lines are intersecting or not with such a form.
    Vec2 line0DirVec = lineDirectionVector(line0);
    Vec2 line1DirVec = lineDirectionVector(line1);

    if (floatEquals(dotProduct(line0DirVec, vec2Perp(line1DirVec)), 0.f)) {
      // The two lines are parallel, so no point of intersection.
      return ReturnValue<Point>{ Point{}, ReturnState::RETURN_FAIL };
    }

    // Note that the vector form equation of a line is defined as:
    //                   r(λ) = p + λd
    // where:
    //   p - position vector
    //   λ - a scalar value determining the point in a line
    //   d - direction vector
    Vec2 line0PosVec = line0.start; // A Point object is an alias for a Vec2.
    Vec2 line1PosVec = line1.start;

    float lambda = (
      dotProduct(line1PosVec - line0PosVec, vec2Perp(line1DirVec))
      / dotProduct(line0DirVec, vec2Perp(line1DirVec))
    );
    return ReturnValue<Point>{ line0PosVec + (lambda * line0DirVec),
                               ReturnState::RETURN_OK };
  }

  ReturnValue<Point> intersectionOfLineandInfLine(const Line& line,
                                                  const Line& infLine)
  {
    auto intersectionPt = intersectionOfTwoInfLines(line, infLine);
    if (intersectionPt.status == ReturnState::RETURN_FAIL) {
      return intersectionPt;
    }

    Point intersectPt = intersectionPt.value;

    // Previously, we used to check if the intersection point is on the line
    // segment. However, we are sure that the intersection point will lay on the
    // line segment if the segment is a line, because the intersection is where
    // two lines sharer the same point. This renders the check unnecessary. We
    // just need to make sure that the the point will fall on the line segment.
    // So, we removed the check.
    if (isFloatInclusiveBetween(line.start.x, intersectPt.x, line.end.x)
        && isFloatInclusiveBetween(line.start.y, intersectPt.y, line.end.y)) {
      return ReturnValue<Point>{ intersectPt, ReturnState::RETURN_OK };
    }

    return ReturnValue<Point>{ Point{}, ReturnState::RETURN_FAIL };
  }

  ReturnValue<Point> intersectionOfLineAndLine(const Line& line0,
                                               const Line& line1)
  {
    auto intersectionPt = intersectionOfTwoInfLines(line0, line1);
    if (intersectionPt.status == ReturnState::RETURN_FAIL) {
      return intersectionPt;
    }

    Point intersectPt = intersectionPt.value;

    // For an intersection point to exist between two line segments, the point
    // must within the two line segments.
    if (isFloatInclusiveBetween(line0.start.x, intersectPt.x, line0.end.x)
        && isFloatInclusiveBetween(line0.start.y, intersectPt.y, line0.end.y)
        && isFloatInclusiveBetween(line1.start.x, intersectPt.x, line1.end.x)
        && isFloatInclusiveBetween(line1.start.y, intersectPt.y, line1.end.y)) {
      return ReturnValue<Point>{ intersectPt, ReturnState::RETURN_OK };
    }

    return ReturnValue<Point>{ Point{}, ReturnState::RETURN_FAIL };
  }

  bool areTwoLinesIntersecting(const Line& line0, const Line& line1)
  {
    auto intersectionPt = intersectionOfLineAndLine(line0, line1);
    return intersectionPt.status == ReturnState::RETURN_OK;
  }

  NPolygon clippedPolygonFromTwoRects(const Rectangle& targetRect,
                                      const Rectangle& clippingRect)
  {
    // Heck, yeah! Let's do some Sutherland-Hodgman.
    auto targetRectPoly = convertRectangleToPolygon(targetRect);
    auto clippingEdges = convertPolygonToLines(
      convertRectangleToPolygon(clippingRect));

    eastl::vector<Point> clippedPolyVerts;
    for (Point& point : targetRectPoly.vertices) {
      clippedPolyVerts.push_back(point);
    }

    eastl::vector<Point> currTargetPoly;
    for (Line& clipEdge : clippingEdges) {
      currTargetPoly = clippedPolyVerts;
      clippedPolyVerts.clear();

      for (int32_t i = 0; i < currTargetPoly.size(); i++) {
        Line targetLine = Line{
          currTargetPoly[i],
          currTargetPoly[(i + 1) % currTargetPoly.size()]
        };
        auto intersectionPt = intersectionOfLineandInfLine(targetLine,
                                                           clipEdge);
        if (floatLessEqual(signedDistPointToInfLine(targetLine.start, clipEdge),
                           0.f)) {
          // The start of the target line is inside the clip edge.
          clippedPolyVerts.push_back(targetLine.start);

          if (floatGreEqual(signedDistPointToInfLine(targetLine.end, clipEdge),
                            0.f)) {
            // There should be an intersection point here since the start
            // and end points of the target line are in opposite sides of the
            // clipping edge.
            if (intersectionPt.status == ReturnState::RETURN_OK) {
              clippedPolyVerts.push_back(intersectionPt.value);
            }
          }
        } else if (floatLessEqual(
            signedDistPointToInfLine(targetLine.end, clipEdge), 0.f)) {
          // The end of the target line is inside the clip edge.
          // There should be an intersection point here since the start
          // and end points of the target line are in opposite sides of the
          // clipping edge.
          if (intersectionPt.status == ReturnState::RETURN_OK) {
            clippedPolyVerts.push_back(intersectionPt.value);
          }
        }
      }
    }

    return NPolygon{ clippedPolyVerts };
  }

  Point getRandomPointInTriangle(const Polygon<3>& triangle)
  {
    // Algorithm based on the algorithm in:
    //   https://blogs.sas.com
    //          /content/iml/2020/10/19/random-points-in-triangle.html
    const Point p0 = triangle.vertices[0];
    const Point p1 = triangle.vertices[1];
    const Point p2 = triangle.vertices[2];

    Vec2 a = p1 - p0;
    Vec2 b = p2 - p0;

    std::uniform_real_distribution<float> normalizedDistrib{ 0.f, 1.f };
    float u0 = generateRandomReal(normalizedDistrib);
    float u1 = generateRandomReal(normalizedDistrib);
    if (floatGreater(u0 + u1, 1.f)) {
      u0 = 1 - u0;
      u1 = 1 - u1;
    }

    return ((u0 * a) + (u1 * b)) + p0;
  }

  Point getPolygonCentroid(const NPolygon& polygon)
  {
    // From "Calculating the area and centroid of a polygon" by Paul Bourke.
    // URL: http://paulbourke.net/geometry/polygonmesh/
    double polygonArea = getPolygonArea(polygon);
    auto& vertices = polygon.vertices;
    double centroidX = 0.0;
    double centroidY = 0.0;

    float determinant = corex::core::det3x3(vertices[0],
                                            vertices[1],
                                            vertices[2]);

    for (int32_t i = 0; i < vertices.size(); i++) {
      // If the polygon is oriented counterclockwise when the origin is in the
      // center, then it is oriented clockwise when the origin is in the
      // top-left. We must iterate through the vertices from left to right.
      //
      // If the polygon is oriented clockwise when the origin is in the center,
      // then it is oriented counterclockwise when the origin is in the
      // top-left. We must iterate through the vertices from right to left.
      int32_t currVertIndex = 0;
      int32_t nextVertIndex = 0;
      if (corex::core::floatGreEqual(determinant, 0.f)) {
        // Polygon is oriented counterclockwise when the origin is in the
        // center.
        currVertIndex = i;
        nextVertIndex = corex::core::mod(currVertIndex + 1,
                                         vertices.size());
      } else {
        // Polygon is oriented clockwise when the origin is in the
        // center.
        currVertIndex = vertices.size() - (i + 1);
        nextVertIndex = corex::core::mod(currVertIndex - 1,
                                         vertices.size());
      }

      centroidX += (vertices[currVertIndex].x + vertices[nextVertIndex].x)
                   * ((vertices[currVertIndex].x * vertices[nextVertIndex].y)
                     - (vertices[nextVertIndex].x * vertices[currVertIndex].y));
      centroidY += (vertices[currVertIndex].y + vertices[nextVertIndex].y)
                   * ((vertices[currVertIndex].x * vertices[nextVertIndex].y)
                     - (vertices[nextVertIndex].x * vertices[currVertIndex].y));
    }

    double areaConstant = 1.0 / (6.0 * polygonArea);
    centroidX *= areaConstant;
    centroidY *= areaConstant;

    return Point{
      static_cast<float>(centroidX),
      static_cast<float>(centroidY)
    };
  }

  int32_t getNumNPolygonSides(const NPolygon& polygon)
  {
    return polygon.vertices.size();
  }

  bool isPointWithinNPolygon(const Point& point, const NPolygon& polygon)
  {
    // Code based from:
    //     https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
    // NOTE: I think the algorithm expects that the origin is situated in the
    //       bottom-left corner of the space. Since our vertices are stored with
    //       the assumption that the origin is situated in the top-left corner.
    //       from the perspective of the algorithm, the vertices are flipped
    //       vertically.
    bool isPointInside = false;
    for (int32_t i = 0, j = polygon.vertices.size() - 1;
         i < polygon.vertices.size();
         j = i++) {
      Line polyLine = Line{ polygon.vertices[i], polygon.vertices[j] };
      if (((polyLine.start.y > point.y) != (polyLine.end.y > point.y))
          && (point.x < ((polyLine.end.x - polyLine.start.x)
                          * (point.y - polyLine.start.y)
                          / (polyLine.end.y - polyLine.start.y)
                          + polyLine.start.x))) {
        isPointInside = !isPointInside;
      }
    }

    return isPointInside;
  }

  bool isRectWithinNPolygon(const Rectangle& rect, const NPolygon& polygon)
  {
    auto rectPoly = convertRectangleToPolygon(rect);

    for (int32_t i = 0; i < polygon.vertices.size(); i++) {
      Line boundaryLine = Line{
        polygon.vertices[i],
        polygon.vertices[(i + 1) % polygon.vertices.size()]
      };

      for (int32_t j = 0; j < rectPoly.vertices.size(); j++) {
        Line rectLine = Line{
          rectPoly.vertices[j],
          rectPoly.vertices[(j + 1) % rectPoly.vertices.size()]
        };

        if (areTwoLinesIntersecting(boundaryLine, rectLine)
            && (floatGreater(signedDistPointToInfLine(rectLine.start,
                                                      boundaryLine),
                             0.f)
                || floatGreater(signedDistPointToInfLine(rectLine.end,
                                                         boundaryLine),
                                0.f))) {
          return false;
        }
      }
    }

    return isPointWithinNPolygon(rectPoly.vertices[0], polygon);
  }

  bool isRectWithinNPolygonAABB(const Rectangle& rect, const NPolygon& polygon)
  {
    // We are assuming the NPolygon is a quadrilateral.
    assert(polygon.vertices.size() == 4);

    float minX = polygon.vertices[0].x;
    float minY = polygon.vertices[0].y;
    float maxX = polygon.vertices[2].x;
    float maxY = polygon.vertices[2].y;
    float width = maxX - minX;
    float height = maxY - minY;

    // Rectangle have their xy coordinates in the center, so we need to move
    // minX and minY to the center, because they are originally referring to the
    // top-left corner.
    Rectangle poly{
      minX + (width / 2),
      minY + (height / 2),
      width,
      height,
      0.f
    };

    return isRectWithinRectAABB(rect, poly);
  }

  bool isRectWithinRectAABB(const Rectangle& insideRect,
                            const Rectangle& outsideRect)
  {
    Polygon<4> inside = rotateRectangle(insideRect);
    Polygon<4> outside = rotateRectangle(outsideRect);

    float insideTopLeftX = inside.vertices[0].x;
    float insideTopLeftY = inside.vertices[0].y;
    float outsideTopLeftX = outside.vertices[0].x;
    float outsideTopLeftY = outside.vertices[0].y;
    float insideWidth = inside.vertices[1].x - insideTopLeftX;
    float insideHeight = inside.vertices[3].y - insideTopLeftY;
    float outsideWidth = outside.vertices[1].x - outsideTopLeftX;
    float outsideHeight = outside.vertices[3].y - outsideTopLeftY;

    return outsideTopLeftX <= insideTopLeftX
           && outsideTopLeftX + outsideWidth >= insideTopLeftX + insideWidth
           && outsideTopLeftY <= insideTopLeftY
           && outsideTopLeftY + outsideHeight >= insideTopLeftY + insideHeight;
  }

  bool isRectIntersectingNPolygon(const Rectangle& rect,
                                  const NPolygon& polygon)
  {
    // NOTE: "A subset of a set is equal to its intersections."
    // Source: https://www.quora.com
    //                /Set-Theory-Is-a-subset-a-type-of-intersection
    //                /answer/Vinay-Madhusudanan
    auto rectPoly = convertRectangleToPolygon(rect);

    for (int32_t i = 0; i < polygon.vertices.size(); i++) {
      Line boundaryLine = Line{
        polygon.vertices[i],
        polygon.vertices[(i + 1) % polygon.vertices.size()]
      };

      for (int32_t j = 0; j < rectPoly.vertices.size(); j++) {
        Line rectLine = Line{
          rectPoly.vertices[j],
          rectPoly.vertices[(j + 1) % rectPoly.vertices.size()]
        };

        if (areTwoLinesIntersecting(boundaryLine, rectLine)) {
          return true;
        }
      }
    }

    return isPointWithinNPolygon(rectPoly.vertices[0], polygon);
  }

  Polygon<4> getIntersectingRectAABB(const Rectangle& rect0,
                                     const Rectangle& rect1)
  {
    if (areTwoRectsAABBIntersecting(rect0, rect1)) {
      cx::Polygon<4> poly0 = convertRectangleToPolygon(rect0);
      cx::Polygon<4> poly1 = convertRectangleToPolygon(rect1);

      if (isRectWithinRectAABB(rect0, rect1)) {
        return poly0;
      } else if (isRectWithinRectAABB(rect1, rect0)) {
        return poly1;
      }

      const auto& topLeftPt0 = poly0.vertices[0];
      const auto& bottomRightPt0 = poly0.vertices[2];
      const auto& topLeftPt1 = poly1.vertices[0];
      const auto& bottomRightPt1 = poly1.vertices[2];

      // Get the intersected rectangle top-left and bottom-right coordinates.
      const auto interTopLeftPt0 = cx::Point{
        std::max(topLeftPt0.x, topLeftPt1.x),
        std::max(topLeftPt0.y, topLeftPt1.y)
      };
      const auto interBottomRightPt1 = cx::Point{
        std::min(bottomRightPt0.x, bottomRightPt1.x),
        std::min(bottomRightPt0.y, bottomRightPt1.y)
      };
      const float interWidth = interBottomRightPt1.x - interTopLeftPt0.x;

      return Polygon<4>{
        {
          interTopLeftPt0,
          interTopLeftPt0 + cx::Vec2{ interWidth, 0.f },
          interBottomRightPt1,
          interBottomRightPt1 - cx::Vec2{ interWidth, 0.f }
        }
      };
    }

    return Polygon<4>{};
  }

  eastl::vector<Polygon<3>> earClipTriangulate(NPolygon polygon)
  {
    // TODO: Tweak this triangulation method.
    // We are iterating through the vertices in a counterclockwise manner when
    // the origin is on the bottom right.
    eastl::vector<Polygon<3>> triangles;
    auto& vertices = polygon.vertices;
    int32_t numTriangles = vertices.size() - 2;

    // It's triangulation time!
    auto vertexAngles = computePolygonInteriorAngles(polygon);
    auto earVertexIndexes = findEarVertexIndexes(polygon);
    const auto convexVertexIndexes = findConvexVertexIndexes(polygon);
    const auto reflexVertexIndexes = findReflexVertexIndexes(polygon);
    eastl::set<int32_t> earVertexIndexesSet(earVertexIndexes.begin(),
                                            earVertexIndexes.end());
    eastl::set<int32_t> convexVertexIndexesSet(convexVertexIndexes.begin(),
                                               convexVertexIndexes.end());
    eastl::set<int32_t> reflexVertexIndexesSet(reflexVertexIndexes.begin(),
                                               reflexVertexIndexes.end());

    while (true) {
      // Using ears from the back removes the need for us to the update the
      // vertex indexes in the ear vertex indexes vector when deleting ears.
      int32_t targetEarIndex = earVertexIndexes.back();

      // Add triangle.
      int32_t prevVertexIndex = mod(targetEarIndex - 1, vertices.size());
      int32_t nextVertexIndex = mod(targetEarIndex + 1, vertices.size());

      triangles.push_back({
        {
          vertices[targetEarIndex],
          vertices[nextVertexIndex],
          vertices[prevVertexIndex]
        }
      });

      // Remove ear.
      earVertexIndexes.pop_back();
      earVertexIndexesSet.erase(targetEarIndex);
      convexVertexIndexesSet.erase(targetEarIndex);
      vertices.erase(vertices.begin() + targetEarIndex);

      if (triangles.size() == numTriangles) {
        break;
      }

      // Update the indexes of the vertices succeeding the newly-removed ear, if
      // the ear is not the last vertex of the polygon, because the vertex
      // indexes will remain the same post-removal if it is so.
      if (nextVertexIndex != 0) {
        for (int32_t i = nextVertexIndex; i < vertices.size(); i++) {
          if (isKeyInSet(reflexVertexIndexesSet, i)) {
            reflexVertexIndexesSet.erase(i);
            reflexVertexIndexesSet.insert(i - 1);
          } else if (isKeyInSet(convexVertexIndexesSet, i)) {
            convexVertexIndexesSet.erase(i);
            convexVertexIndexesSet.insert(i - 1);

            // NOTE: An ear is always a convex, but a convex vertex is not always
            //       an ear.
            if (isKeyInSet(earVertexIndexesSet, i)) {
              earVertexIndexesSet.erase(i);
              earVertexIndexesSet.insert(i - 1);

              auto iter = eastl::find(earVertexIndexes.begin(),
                                      earVertexIndexes.end(),
                                      i);
              if (iter != earVertexIndexes.end()) {
                (*iter)--;
              }
            }
          }
        }

        nextVertexIndex--;
      }

      // Update vertex angles.
      // TODO: Optimize this so that only the angles of the vertices adjacent
      //       to the newly-removed ear vertex are recomputed.
      vertexAngles = computePolygonInteriorAngles(polygon);

      // Update status of adjacent vertices of newly-removed ear.
      eastl::array<int32_t, 2> adjacentIndexes = {
        prevVertexIndex,
        nextVertexIndex
      };
      for (int32_t idx : adjacentIndexes) {
        if (isKeyInSet(earVertexIndexesSet, idx)) {
          // Vertex is an ear. It might still be an ear or not. Check to
          // confirm.
          if (!isVertexAnEarInPolygon(idx, polygon)) {
            // Oh, no. It's no longer an ear. No need to remove the vertex from
            // the set of convex vertices though. Even though it is no longer an
            // ear, a convex vertex will remain a convex.
            // Yep, it's that loyal (unlike your ex).

            earVertexIndexesSet.erase(idx);
            auto iter = eastl::find(earVertexIndexes.begin(),
                                    earVertexIndexes.end(),
                                    idx);
            if (iter != earVertexIndexes.end()) {
              earVertexIndexes.erase(iter);
            }
          }
        } else if (isKeyInSet(convexVertexIndexesSet, idx)) {
          // Vertex is a convex but not an ear. It will still be convex, but
          // let's check if it has turned into an ear.
          if (isVertexAnEarInPolygon(idx, polygon)) {
            // Let's make sure that we do not add duplicates to the data
            // structures holding information about ear vertices.
            if (!isKeyInSet(earVertexIndexesSet, idx)) {
              earVertexIndexes.push_back(idx);
              earVertexIndexesSet.insert(idx);
            }
          }
        } else if (isKeyInSet(reflexVertexIndexesSet, idx)) {
          // Vertex is a reflex. It might have turned into an ear or a convex.
          // Check to confirm. Check first if the vertex has turned into a
          // convex vertex.
          if (floatLessThan(vertexAngles[idx], 180)) {
            // The vertex has become a convex vertex.
            reflexVertexIndexesSet.erase(idx);
            convexVertexIndexesSet.insert(idx);

            // Let's check if it's an ear now.
            if (isVertexAnEarInPolygon(idx, polygon)) {
              // Let's make sure that we do not add duplicates to the data
              // structures holding information about ear vertices.
              if (!isKeyInSet(earVertexIndexesSet, idx)) {
                earVertexIndexes.push_back(idx);
                earVertexIndexesSet.insert(idx);
              }
            }
          }
        }
      }
    }

    return triangles;
  }

  eastl::vector<int32_t> findEarVertexIndexes(const NPolygon& polygon)
  {
    eastl::vector<int32_t> earVertexIndexes;
    auto& vertices = polygon.vertices;
    auto convexVertexIndexes = findConvexVertexIndexes(polygon);
    for (int32_t& currVertIndex : convexVertexIndexes) {
      if (isVertexAnEarInPolygon(currVertIndex, polygon)) {
        earVertexIndexes.push_back(currVertIndex);
      }
    }

    return earVertexIndexes;
  }

  eastl::vector<int32_t> findConvexVertexIndexes(const NPolygon& polygon)
  {
    eastl::vector<int32_t> convexVertexIndexes;
    auto vertexInteriorAngles = computePolygonInteriorAngles(polygon);
    for (int32_t i = 0; i < vertexInteriorAngles.size(); i++) {
      if (floatLessThan(vertexInteriorAngles[i], 180)) {
        convexVertexIndexes.push_back(i);
      }
    }

    return convexVertexIndexes;
  }

  eastl::vector<int32_t> findReflexVertexIndexes(const NPolygon& polygon)
  {
    eastl::vector<int32_t> reflexVertexIndexes;
    auto vertexInteriorAngles = computePolygonInteriorAngles(polygon);
    for (int32_t i = 0; i < vertexInteriorAngles.size(); i++) {
      if (floatGreEqual(vertexInteriorAngles[i], 180)) {
        reflexVertexIndexes.push_back(i);
      }
    }

    return reflexVertexIndexes;
  }

  eastl::vector<float> computePolygonInteriorAngles(const NPolygon& polygon)
  {
    // We are iterating through the vertices in a counterclockwise manner when
    // the origin is on the bottom right.
    auto& vertices = polygon.vertices;
    eastl::vector<float> angles(vertices.size());

    auto convexHull = findConvexHull(polygon);
    float determinant = det3x3(convexHull[0], convexHull[1], convexHull[2]);
    for (int32_t i = 0; i < vertices.size(); i++) {
      // Ensure that we are iterating through the polygon counterclockwise.
      int32_t currVertIndex = 0;
      int32_t prevVertIndex = 0;
      int32_t nextVertIndex = 0;
      if (floatGreEqual(determinant, 0.f)) {
        // Polygon is oriented counterclockwise when the origin is in the
        // center.
        currVertIndex = i;
        prevVertIndex = mod(currVertIndex - 1, vertices.size());
        nextVertIndex = mod(currVertIndex + 1, vertices.size());
      } else {
        // Polygon is oriented clockwise when the origin is in the
        // center.
        currVertIndex = vertices.size() - (i + 1);
        prevVertIndex = mod(currVertIndex + 1, vertices.size());
        nextVertIndex = mod(currVertIndex - 1, vertices.size());
      }

      Vec2 p;
      Vec2 q;
      p = lineToVec(Line{ vertices[currVertIndex], vertices[prevVertIndex] });
      q = lineToVec(Line{ vertices[currVertIndex], vertices[nextVertIndex] });
      float angle = angleBetweenTwoVectors(p, q);

      if (floatGreater(crossProduct(p, q), 0)) {
        // The angle we calculated is actually the exterior angle. Adjust angle
        // to get the interior angle.
        angle = 360.f - angle;
      }

      angles[currVertIndex] = angle;
    }

    return angles;
  }

  bool isVertexAnEarInPolygon(const int32_t & vertexIndex,
                              const NPolygon& polygon)
  {
    auto& vertices = polygon.vertices;
    int32_t currVertIndex = vertexIndex;
    int32_t prevVertIndex = mod(currVertIndex - 1, vertices.size());
    int32_t nextVertIndex = mod(currVertIndex + 1, vertices.size());

    auto triangle = NPolygon{
      {
        vertices[prevVertIndex],
        vertices[currVertIndex],
        vertices[nextVertIndex]
      }
    };

    // Check if a vertex from the polygon is in the triangle.
    for (int32_t j = mod(nextVertIndex + 1, vertices.size());
         j != prevVertIndex;
         j = mod(j + 1, vertices.size())) {
      if (isPointWithinNPolygon(vertices[j], triangle)) {
        return false;
      }
    }

    return true;
  }
}
