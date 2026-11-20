#include "reachability/core/io.hpp"

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace reachability
{
namespace
{
constexpr char kMagic[8] = {'R', 'M', '4', 'D', 'M', 'A', 'P', '\0'};
constexpr uint32_t kVersion = 2;   // 1: no MapInfo (still readable)

template<typename T>
void put(std::ofstream & o, const T & v) {o.write(reinterpret_cast<const char *>(&v), sizeof(T));}
template<typename T>
bool get(std::ifstream & i, T & v) {return static_cast<bool>(i.read(reinterpret_cast<char *>(&v), sizeof(T)));}
}  // namespace

void saveMap(const std::string & path, const Map4D & map, const std::string & fp,
  const MapInfo & info)
{
  std::ofstream o(path, std::ios::binary);
  if (!o) {
    throw std::runtime_error(path + ": cannot open for writing");
  }
  o.write(kMagic, sizeof(kMagic));
  put(o, kVersion);
  const MapParams & p = map.params();
  put(o, p.xy_extent); put(o, p.z_min); put(o, p.z_max); put(o, p.resolution); put(o, p.theta_bin);
  const uint32_t fp_len = static_cast<uint32_t>(fp.size());
  put(o, fp_len);
  o.write(fp.data(), static_cast<std::streamsize>(fp.size()));
  put(o, info.arm_dof);
  const uint32_t red_len = static_cast<uint32_t>(info.redundancy.size());
  put(o, red_len);
  o.write(info.redundancy.data(), static_cast<std::streamsize>(info.redundancy.size()));
  const uint64_t n = map.raw().size();
  put(o, n);
  o.write(reinterpret_cast<const char *>(map.raw().data()), static_cast<std::streamsize>(n));
  if (!o) {
    throw std::runtime_error(path + ": write failed");
  }
}

Map4D loadMap(const std::string & path, const std::string & expected, MapInfo * info)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error(path + ": cannot open");
  }
  char magic[8];
  uint32_t version = 0;
  if (!in.read(magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    throw std::runtime_error(path + ": not a reachability map");
  }
  if (!get(in, version) || (version != 1 && version != kVersion)) {
    throw std::runtime_error(path + ": unsupported map format version " + std::to_string(version));
  }
  MapParams p;
  if (!get(in, p.xy_extent) || !get(in, p.z_min) || !get(in, p.z_max) || !get(in, p.resolution) ||
    !get(in, p.theta_bin))
  {
    throw std::runtime_error(path + ": truncated header");
  }
  uint32_t fp_len = 0;
  if (!get(in, fp_len) || fp_len > (1U << 20)) {
    throw std::runtime_error(path + ": bad fingerprint length");
  }
  std::string fp(fp_len, '\0');
  if (!in.read(fp.data(), fp_len)) {
    throw std::runtime_error(path + ": truncated fingerprint");
  }
  if (!expected.empty() && fp != expected) {
    throw std::runtime_error(path + ": the map was built for a different robot (fingerprint mismatch)");
  }
  MapInfo header;
  if (version >= 2) {
    uint32_t red_len = 0;
    if (!get(in, header.arm_dof) || !get(in, red_len) || red_len > (1U << 20)) {
      throw std::runtime_error(path + ": bad map info");
    }
    header.redundancy.assign(red_len, '\0');
    if (!in.read(header.redundancy.data(), red_len)) {
      throw std::runtime_error(path + ": truncated map info");
    }
  }
  if (info) {
    *info = header;
  }
  Map4D map(p);
  uint64_t n = 0;
  if (!get(in, n) || n != map.raw().size()) {
    throw std::runtime_error(path + ": cell count does not match the map parameters");
  }
  if (!in.read(reinterpret_cast<char *>(map.raw().data()), static_cast<std::streamsize>(n))) {
    throw std::runtime_error(path + ": truncated cell data");
  }
  return map;
}

}  // namespace reachability
