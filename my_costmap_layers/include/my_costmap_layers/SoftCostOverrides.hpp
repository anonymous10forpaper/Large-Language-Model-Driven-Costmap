#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nlohmann/json.hpp>

namespace my_costmap_layers
{

inline bool parseSoftCostOverrides(
  const std::string & payload,
  std::unordered_map<std::string, int> & costs,
  std::string & error)
{
  try {
    // Reject duplicate JSON keys rather than silently keeping the last value.
    std::vector<std::unordered_set<std::string>> object_keys;
    bool duplicate_key = false;
    const auto document = nlohmann::json::parse(
      payload, [&](int, nlohmann::json::parse_event_t event, nlohmann::json & value) {
        if (event == nlohmann::json::parse_event_t::object_start) {
          object_keys.emplace_back();
        } else if (event == nlohmann::json::parse_event_t::key) {
          if (!object_keys.back().insert(value.get<std::string>()).second) {
            duplicate_key = true;
          }
        } else if (event == nlohmann::json::parse_event_t::object_end) {
          object_keys.pop_back();
        }
        return true;
      });
    if (duplicate_key) {
      error = "duplicate JSON keys are not allowed";
      return false;
    }
    if (!document.is_object() ||
      (!document.contains("zones") && !document.contains("weights")))
    {
      error = "payload must contain a zones object or weights array";
      return false;
    }

    std::unordered_map<std::string, int> parsed;
    const auto add_cost = [&](const std::string & zone, const nlohmann::json & value) {
        if (zone.size() != 1 || zone[0] < 'A' || zone[0] > 'Z') {
          error = "each zone must use an A-Z label";
          return false;
        }
        if (!value.is_number_integer() || value < 0 ||
          value > nav2_costmap_2d::MAX_NON_OBSTACLE)
        {
          error = "soft cost for zone " + zone + " must be an integer in 0..252";
          return false;
        }
        const int cost = value.get<int>();
        const auto inserted = parsed.emplace(zone, cost);
        if (!inserted.second && inserted.first->second != cost) {
          error = "conflicting costs for zone " + zone;
          return false;
        }
        return true;
      };

    if (document.contains("zones")) {
      const auto & zones = document.at("zones");
      if (!zones.is_object()) {
        error = "zones must be an object";
        return false;
      }
      for (const auto & entry : zones.items()) {
        if (!add_cost(entry.key(), entry.value())) {
          return false;
        }
      }
    }
    if (document.contains("weights")) {
      const auto & weights = document.at("weights");
      if (!weights.is_array()) {
        error = "weights must be an array";
        return false;
      }
      for (const auto & item : weights) {
        if (!item.is_object() || !item.contains("zone") || !item.at("zone").is_string() ||
          !item.contains("cost"))
        {
          error = "each weight must contain a zone string and an integer cost";
          return false;
        }
        if (!add_cost(item.at("zone").get<std::string>(), item.at("cost"))) {
          return false;
        }
      }
    }

    // Commit only a complete valid update; explicit empty collections clear costs.
    costs.swap(parsed);
    error.clear();
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace my_costmap_layers

