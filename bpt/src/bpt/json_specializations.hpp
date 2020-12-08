#include <nlohmann/json.hpp>

#include <bpt/SelectionType.hpp>

// Set up nlohmann/json specializations.
// Note that this is a macro provided by nlohmann/json.
namespace bpt
{
  NLOHMANN_JSON_SERIALIZE_ENUM(bpt::SelectionType, {
    { bpt::SelectionType::NONE, nullptr },
    { bpt::SelectionType::RWS, "rws" },
    { bpt::SelectionType::TS, "ts" },
  });
}
