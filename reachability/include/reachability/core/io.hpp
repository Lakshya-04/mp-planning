#pragma once

#include <string>

#include "reachability/core/map4d.hpp"

namespace reachability
{

/// Versioned binary map file: magic, format version, MapParams, a fingerprint of the robot it was
/// built for, then the cell array. Throws std::runtime_error on any problem.
void saveMap(const std::string & path, const Map4D & map, const std::string & robot_fingerprint = "");

/// Loads a map. If `expected_fingerprint` is non-empty it must equal the stored one, otherwise
/// the map was built for a different robot and loading fails.
Map4D loadMap(const std::string & path, const std::string & expected_fingerprint = "");

}  // namespace reachability
