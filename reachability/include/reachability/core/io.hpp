#pragma once

#include <cstdint>
#include <string>

#include "reachability/core/map4d.hpp"

namespace reachability
{

/// What the file header says about the arm, besides the robot fingerprint (format version 2).
/// A version 1 file loads with arm_dof 0 and an empty redundancy text ("not recorded").
struct MapInfo
{
  uint32_t arm_dof{0};         ///< Joints of the arm the map was built for.
  std::string redundancy;      ///< How a redundant arm's null space was handled (see redundancy.hpp).
};

/// Versioned binary map file: magic, format version, MapParams, a fingerprint of the robot it was
/// built for, the MapInfo, then the cell array. Throws std::runtime_error on any problem.
void saveMap(const std::string & path, const Map4D & map, const std::string & robot_fingerprint = "",
  const MapInfo & info = {});

/// Loads a map (format version 1 or 2). If `expected_fingerprint` is non-empty it must equal the
/// stored one, otherwise the map was built for a different robot and loading fails. `info`, when
/// given, receives the header's MapInfo.
Map4D loadMap(const std::string & path, const std::string & expected_fingerprint = "",
  MapInfo * info = nullptr);

}  // namespace reachability
