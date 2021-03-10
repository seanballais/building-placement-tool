#ifndef BPT_UTILS_HPP
#define BPT_UTILS_HPP

#include <bpt/ds/AlgorithmType.hpp>
#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/SelectionType.hpp>

namespace bpt
{
  const char* castToCString(AlgorithmType type);
  const char* castToCString(CrossoverType type);
  const char* castToCString(SelectionType type);

  CrossoverType cStringToCrossoverType(const char* str);
}

#endif
