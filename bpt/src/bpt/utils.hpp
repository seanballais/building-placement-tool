#ifndef BPT_UTILS_HPP
#define BPT_UTILS_HPP

#include <corex/core/ds/VecN.hpp>

#include <bpt/ds/AlgorithmType.hpp>
#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/SelectionType.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  const char* castToCString(AlgorithmType type);
  const char* castToCString(CrossoverType type);
  const char* castToCString(SelectionType type);

  CrossoverType cStringToCrossoverType(const char* str);

  cx::VecN convertSolutionToVecN(const Solution& solution);
  Solution convertVecNToSolution(const cx::VecN& vecN);
  void printVecN(const cx::VecN& vecN);

  Solution translateSolutionOrigin(const Solution& solution,
                                   float deltaX,
                                   float deltaY);
  cx::VecN clampSolutionVecN(const cx::VecN& vec,
                             float minX,
                             float minY,
                             float maxX,
                             float maxY);
  cx::VecN wrapAroundSolutionVecN(const cx::VecN& vec,
                                  float minX,
                                  float minY,
                                  float maxX,
                                  float maxY);
  cx::VecN createRandomVector(const int32_t vectorSize,
                              float min = 0.f, float max = 1.f);
}

#endif
