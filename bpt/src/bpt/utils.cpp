#include <cassert>
#include <cstring>

#include <bpt/ds/AlgorithmType.hpp>
#include <bpt/ds/SelectionType.hpp>
#include <bpt/utils.hpp>

namespace bpt
{
  const char* castToCString(AlgorithmType type)
  {
    switch (type) {
      case AlgorithmType::GA:
        return "Genetic Algorithm";
      case AlgorithmType::GWO:
        return "Grey Wolf Optimization";
      case AlgorithmType::HC:
        return "Naive Hill Climbing";
    }
  }

  const char* castToCString(CrossoverType type)
  {
    switch (type) {
      case CrossoverType::NONE:
        return "None";
      case CrossoverType::UNIFORM:
        return "Uniform";
      case CrossoverType::BOX:
        return "Box";
    }
  }

  const char* castToCString(SelectionType type)
  {
    switch (type) {
      case SelectionType::NONE:
        return "None";
      case SelectionType::RS:
        return "Ranked Selection";
      case SelectionType::RWS:
        return "Roulette Wheel";
      case SelectionType::TS:
        return "Tournament";
    }
  }

  CrossoverType cStringToCrossoverType(const char* str)
  {
    if (strcmp(str, "None") == 0) {
      return CrossoverType::NONE;
    } else if (strcmp(str, "Uniform") == 0) {
      return CrossoverType::UNIFORM;
    } else if (strcmp(str, "Box") == 0) {
      return CrossoverType::BOX;
    } else {
      assert(false);
    }
  }
}
