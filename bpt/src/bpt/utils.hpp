#ifndef BPT_UTILS_HPP
#define BPT_UTILS_HPP

#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/SelectionType.hpp>

namespace bpt
{
  const char* castToCString(SelectionType type);
  const char* castToCString(CrossoverType type);
}

#endif
