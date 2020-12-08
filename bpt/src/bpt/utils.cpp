#include <bpt/SelectionType.hpp>
#include <bpt/utils.hpp>

namespace bpt
{
  const char* castToCString(SelectionType type)
  {
    switch (type) {
      case SelectionType::NONE:
        return "None";
      case SelectionType::RWS:
        return "Roulette Wheel";
      case SelectionType::TS:
        return "Tournament";
    }
  }
}
