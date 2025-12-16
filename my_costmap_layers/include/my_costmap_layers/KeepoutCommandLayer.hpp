#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>
#include <vector>
#include <string>
#include <limits>

namespace my_costmap_layers {

struct Zone { double x_min, y_min, x_max, y_max; };

class KeepoutCommandLayer : public nav2_costmap_2d::CostmapLayer {
public:
  KeepoutCommandLayer();
  void onInitialize() override;
  void updateBounds(double, double, double, double*, double*, double*, double*) override;
  void updateCosts(nav2_costmap_2d::Costmap2D&, int, int, int, int) override;
  void reset() override;
  bool isClearable() override { return true; }

private:
  void loadZoneDatabase();
  void forbiddenZonesCallback(const std_msgs::msg::String::SharedPtr);

  void computeZonesUnion(const std::vector<std::string>& zones,
                         double& min_x, double& min_y, double& max_x, double& max_y,
                         bool& valid);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr forbidden_zones_sub_;
  std::vector<std::string> forbidden_zones_;
  std::unordered_map<std::string, Zone> zone_database_;
 
  std::string forbidden_topic_name_;

  bool   updated_{false};
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  bool   prev_valid_{false};
  double prev_min_x_, prev_min_y_, prev_max_x_, prev_max_y_;
};

}

