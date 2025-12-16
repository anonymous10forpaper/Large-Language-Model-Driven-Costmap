#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <limits>
#include <regex>

namespace my_costmap_layers {

struct Zone { double x_min, y_min, x_max, y_max; };

class ZoneSoftCostLayer : public nav2_costmap_2d::CostmapLayer {
public:
  ZoneSoftCostLayer();

  void onInitialize() override;
  void updateBounds(double, double, double, double*, double*, double*, double*) override;
  void updateCosts(nav2_costmap_2d::Costmap2D&, int, int, int, int) override;
  void reset() override;
  bool isClearable() override { return true; }

private:
  void costOverridesCallback(const std_msgs::msg::String::SharedPtr msg);
  void loadZoneDatabase();
  static int clampCost(int v);

  void computeZonesUnion(const std::vector<std::string>& zones,
                         double& min_x, double& min_y, double& max_x, double& max_y,
                         bool& valid);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::unordered_map<std::string, Zone> zone_db_;
  std::unordered_map<std::string, int>  zone_costs_;

  std::string topic_name_;
  bool updated_{false};
  double last_min_x_{0.0}, last_min_y_{0.0}, last_max_x_{0.0}, last_max_y_{0.0};

  bool   prev_valid_{false};
  double prev_min_x_{0.0}, prev_min_y_{0.0}, prev_max_x_{0.0}, prev_max_y_{0.0};
};

}
