#include <nlohmann/json.hpp>

#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/SelectionType.hpp>

// Set up nlohmann/json specializations.
// Note that this is a macro provided by nlohmann/json.
namespace bpt
{
  NLOHMANN_JSON_SERIALIZE_ENUM(bpt::SelectionType, {
    { bpt::SelectionType::NONE, nullptr },
    { bpt::SelectionType::RS, "rs" },
    { bpt::SelectionType::RWS, "rws" },
    { bpt::SelectionType::TS, "ts" },
  });

  NLOHMANN_JSON_SERIALIZE_ENUM(bpt::CrossoverType, {
    { bpt::CrossoverType::NONE, nullptr },
    { bpt::CrossoverType::UNIFORM, "uniform" },
    { bpt::CrossoverType::BOX, "box" }
  });
}
