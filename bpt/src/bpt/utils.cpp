#include <bpt/ds/SelectionType.hpp>
#include <bpt/utils.hpp>

namespace bpt
{
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
}
