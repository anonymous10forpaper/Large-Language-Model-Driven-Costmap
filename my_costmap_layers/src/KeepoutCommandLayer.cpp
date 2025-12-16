#include "my_costmap_layers/KeepoutCommandLayer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <algorithm>
#include <limits>
#include <mutex>
#include <sstream>
#include "nav2_util/node_utils.hpp"

using nav2_costmap_2d::Costmap2D;

namespace my_costmap_layers
{

KeepoutCommandLayer::KeepoutCommandLayer()
: updated_(false),
  last_min_x_(0.0), last_min_y_(0.0), last_max_x_(0.0), last_max_y_(0.0),
  prev_valid_(false),
  prev_min_x_(0.0), prev_min_y_(0.0), prev_max_x_(0.0), prev_max_y_(0.0)
{}

void KeepoutCommandLayer::loadZoneDatabase()
{
  zone_database_["A"] = {8.01, -1.55, 23.96, 1.90};
  zone_database_["B"] = {8.01, -6.05, 24.01, -2.60};
  zone_database_["C"] = {8.01, -10.70, 24.06, -7.20};
  zone_database_["D"] = {8.01, -15.2, 24.01, -11.70};
  zone_database_["E"] = {8.01, -19.7, 24.01, -16.20};
  zone_database_["F"] = {24.00, -6.70, 28.00, -2.6};

  RCLCPP_INFO(rclcpp::get_logger("KeepoutCommandLayer"),
              "Zone database loaded with %zu zones.", zone_database_.size());
}

void KeepoutCommandLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in KeepoutCommandLayer::onInitialize");
  }

  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".forbidden_zones_topic", rclcpp::ParameterValue("/forbidden_zones_update"));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".enabled", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".forbidden_zones_topic", forbidden_topic_name_);
  node->get_parameter(name_ + ".enabled", enabled_);

  loadZoneDatabase();
  matchSize();

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  forbidden_zones_sub_ = node->create_subscription<std_msgs::msg::String>(
    forbidden_topic_name_, qos,
    std::bind(&KeepoutCommandLayer::forbiddenZonesCallback, this, std::placeholders::_1));

  current_ = true;
  updated_ = false;
}

void KeepoutCommandLayer::computeZonesUnion(const std::vector<std::string>& zones,
                                            double& min_x, double& min_y,
                                            double& max_x, double& max_y,
                                            bool& valid)
{
  valid = false;
  min_x =  std::numeric_limits<double>::infinity();
  min_y =  std::numeric_limits<double>::infinity();
  max_x = -std::numeric_limits<double>::infinity();
  max_y = -std::numeric_limits<double>::infinity();

  for (const auto& name : zones) {
    auto it = zone_database_.find(name);
    if (it == zone_database_.end()) continue;
    const auto& z = it->second;
    min_x = std::min(min_x, z.x_min);
    min_y = std::min(min_y, z.y_min);
    max_x = std::max(max_x, z.x_max);
    max_y = std::max(max_y, z.y_max);
    valid = true;
  }

  if (!valid) {
    min_x = min_y = max_x = max_y = 0.0;
  }
}

void KeepoutCommandLayer::forbiddenZonesCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());

  std::vector<std::string> parsed;
  {
    std::stringstream ss(msg->data);
    std::string token;
    while (std::getline(ss, token, ',')) {
      auto l = token.find_first_not_of(" \t");
      auto r = token.find_last_not_of(" \t");
      if (l == std::string::npos) continue;
      std::string t = token.substr(l, r - l + 1);
      if (!t.empty()) parsed.push_back(t);
    }
  }

  double new_min_x, new_min_y, new_max_x, new_max_y; bool new_valid=false;
  computeZonesUnion(parsed, new_min_x, new_min_y, new_max_x, new_max_y, new_valid);

  if (prev_valid_ || new_valid) {
    last_min_x_ = prev_valid_ ? std::min(prev_min_x_, new_min_x) : new_min_x;
    last_min_y_ = prev_valid_ ? std::min(prev_min_y_, new_min_y) : new_min_y;
    last_max_x_ = prev_valid_ ? std::max(prev_max_x_, new_max_x) : new_max_x;
    last_max_y_ = prev_valid_ ? std::max(prev_max_y_, new_max_y) : new_max_y;
  } else {
    last_min_x_ = last_min_y_ = last_max_x_ = last_max_y_ = 0.0;
  }

  forbidden_zones_.swap(parsed);

  prev_valid_ = new_valid;
  prev_min_x_ = new_min_x; prev_min_y_ = new_min_y;
  prev_max_x_ = new_max_x; prev_max_y_ = new_max_y;

  updated_ = true;
  current_ = false;
}

void KeepoutCommandLayer::updateBounds(
  double, double, double, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_ || !updated_) return;
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());

  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
}

void KeepoutCommandLayer::updateCosts(
  nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());

  const int grid_w = static_cast<int>(getSizeInCellsX());
  const int grid_h = static_cast<int>(getSizeInCellsY());
  const int sx = 0, sy = 0;
  const int ex = grid_w - 1;
  const int ey = grid_h - 1;

  const int roi_x0 = std::max(sx, std::max(0, min_i));
  const int roi_y0 = std::max(sy, std::max(0, min_j));
  const int roi_x1 = std::min(ex, max_i - 1);
  const int roi_y1 = std::min(ey, max_j - 1);
  if (roi_x0 > roi_x1 || roi_y0 > roi_y1) {
    updated_ = false;
    current_ = true;
    return;
  }

  for (int j = roi_y0; j <= roi_y1; ++j) {
    for (int i = roi_x0; i <= roi_x1; ++i) {
      setCost(static_cast<unsigned int>(i),
              static_cast<unsigned int>(j),
              nav2_costmap_2d::NO_INFORMATION);
    }
  }

  for (const auto& zone_name : forbidden_zones_) {
    auto it = zone_database_.find(zone_name);
    if (it == zone_database_.end()) continue;
    const auto& z = it->second;

    unsigned int umx0, umy0, umx1, umy1;
    if (!master_grid.worldToMap(z.x_min, z.y_min, umx0, umy0)) continue;
    if (!master_grid.worldToMap(z.x_max, z.y_max, umx1, umy1)) continue;

    const int mx0 = static_cast<int>(umx0);
    const int my0 = static_cast<int>(umy0);
    const int mx1 = static_cast<int>(umx1);
    const int my1 = static_cast<int>(umy1);

    const int zx0 = std::min(mx0, mx1);
    const int zy0 = std::min(my0, my1);
    const int zx1 = std::max(mx0, mx1);
    const int zy1 = std::max(my0, my1);

    const int ix_min = std::max(roi_x0, zx0);
    const int iy_min = std::max(roi_y0, zy0);
    const int ix_max = std::min(roi_x1, zx1);
    const int iy_max = std::min(roi_y1, zy1);
    if (ix_min > ix_max || iy_min > iy_max) continue;

    for (int j = iy_min; j <= iy_max; ++j) {
      for (int i = ix_min; i <= ix_max; ++i) {
        setCost(static_cast<unsigned int>(i),
                static_cast<unsigned int>(j),
                nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  updateWithMax(master_grid, roi_x0, roi_y0, roi_x1 + 1, roi_y1 + 1);

  updated_ = false;
  current_ = true;
}


void KeepoutCommandLayer::reset()
{
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());

  for (unsigned int j = 0; j < getSizeInCellsY(); ++j) {
    for (unsigned int i = 0; i < getSizeInCellsX(); ++i) {
      setCost(i, j, nav2_costmap_2d::NO_INFORMATION);
    }
  }

  forbidden_zones_.clear();
  prev_valid_ = false;

  const double origin_x = getOriginX();
  const double origin_y = getOriginY();
  const double wx_max = origin_x + getSizeInCellsX() * getResolution();
  const double wy_max = origin_y + getSizeInCellsY() * getResolution();

  last_min_x_ = origin_x; last_min_y_ = origin_y;
  last_max_x_ = wx_max;   last_max_y_ = wy_max;

  updated_ = true;
  current_ = false;
}

}

PLUGINLIB_EXPORT_CLASS(my_costmap_layers::KeepoutCommandLayer, nav2_costmap_2d::Layer)

