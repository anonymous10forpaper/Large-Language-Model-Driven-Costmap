#pragma once

#include <cmath>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace my_costmap_layers
{

struct Zone
{
  double x_min;
  double y_min;
  double x_max;
  double y_max;
};

inline std::unordered_map<std::string, Zone> defaultZoneDatabase()
{
  return {
    {"A", {8.01, -1.55, 23.96, 1.90}},
    {"B", {8.01, -6.05, 24.01, -2.60}},
    {"C", {8.01, -10.70, 24.06, -7.20}},
    {"D", {8.01, -15.20, 24.01, -11.70}},
    {"E", {8.01, -19.70, 24.01, -16.20}},
  };
}

inline bool parseZoneGeometry(
  const std::string & payload,
  std::unordered_map<std::string, Zone> & zones,
  std::string & error)
{
  try {
    const auto document = nlohmann::json::parse(payload);
    if (!document.is_object() || !document.contains("zones") ||
      !document.at("zones").is_object())
    {
      error = "payload must contain a zones object";
      return false;
    }
    if (document.contains("frame_id") && document.at("frame_id") != "map") {
      error = "zone geometry frame_id must be map";
      return false;
    }

    std::unordered_map<std::string, Zone> parsed;
    for (const auto & entry : document.at("zones").items()) {
      const std::string & name = entry.key();
      const auto & bounds = entry.value();
      if (name.size() != 1 || name[0] < 'A' || name[0] > 'Z' ||
        !bounds.is_array() || bounds.size() != 4)
      {
        error = "each zone must use an A-Z label and four bounds";
        return false;
      }
      const double x_min = bounds.at(0).get<double>();
      const double y_min = bounds.at(1).get<double>();
      const double x_max = bounds.at(2).get<double>();
      const double y_max = bounds.at(3).get<double>();
      if (!std::isfinite(x_min) || !std::isfinite(y_min) ||
        !std::isfinite(x_max) || !std::isfinite(y_max) ||
        x_min >= x_max || y_min >= y_max)
      {
        error = "zone bounds must be finite and ordered";
        return false;
      }
      parsed.emplace(name, Zone{x_min, y_min, x_max, y_max});
    }
    zones.swap(parsed);
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

}
