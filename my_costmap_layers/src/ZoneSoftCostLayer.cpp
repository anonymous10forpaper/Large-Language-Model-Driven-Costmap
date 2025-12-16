#include "my_costmap_layers/ZoneSoftCostLayer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <sstream>

using nav2_costmap_2d::Costmap2D;

namespace my_costmap_layers
{

ZoneSoftCostLayer::ZoneSoftCostLayer()
: updated_(false)
{}

void ZoneSoftCostLayer::loadZoneDatabase()
{
  zone_db_["A"] = {8.01, -1.55, 23.96,  1.90};
  zone_db_["B"] = {8.01, -6.05, 24.01, -2.60};
  zone_db_["C"] = {8.01,-10.70, 24.06, -7.20};
  zone_db_["D"] = {8.01, -15.2, 24.01, -11.70};
  zone_db_["E"] = {8.01, -19.7, 24.01, -16.20};
}

int ZoneSoftCostLayer::clampCost(int v)
{
  if (v < 0) return 0;
  if (v > 255) v = 255;
  if (v == 255) return 253;
  return v;
}

void ZoneSoftCostLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in ZoneSoftCostLayer::onInitialize");
  }

  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".zone_cost_overrides_topic", rclcpp::ParameterValue("/zone_cost_overrides"));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".enabled", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".zone_cost_overrides_topic", topic_name_);
  node->get_parameter(name_ + ".enabled", enabled_);

  loadZoneDatabase();
  matchSize();

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.reliable();
  sub_ = node->create_subscription<std_msgs::msg::String>(
    topic_name_, qos,
    std::bind(&ZoneSoftCostLayer::costOverridesCallback, this, std::placeholders::_1));

  current_ = true;
  updated_ = false;

  RCLCPP_INFO(rclcpp::get_logger("ZoneSoftCostLayer"),
              "ZoneSoftCostLayer listening on %s", topic_name_.c_str());
}

void ZoneSoftCostLayer::computeZonesUnion(const std::vector<std::string>& zones,
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
    auto it = zone_db_.find(name);
    if (it == zone_db_.end()) continue;
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

void ZoneSoftCostLayer::costOverridesCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());

  RCLCPP_INFO(rclcpp::get_logger("ZoneSoftCostLayer"),
              "[RX] len=%zu payload=%.60s...", msg->data.size(), msg->data.c_str());

  std::vector<std::string> prev_keys;
  prev_keys.reserve(zone_costs_.size());
  for (auto& kv : zone_costs_) prev_keys.push_back(kv.first);

  std::unordered_map<std::string,int> new_costs;

  const std::string& s = msg->data;
  std::regex zkv_re(R"REGEX("([A-Z])"\s*:\s*([0-9]{1,3}))REGEX");
  std::smatch m;
  std::string zones_block;
  {
    std::regex zones_re(R"REGEX("zones"\s*:\s*\{([^}]*)\})REGEX");
    if (std::regex_search(s, m, zones_re)) {
      zones_block = m[1].str();
      auto begin = std::sregex_iterator(zones_block.begin(), zones_block.end(), zkv_re);
      auto end   = std::sregex_iterator();
      for (auto it = begin; it != end; ++it) {
        std::string lab = (*it)[1].str();
        int v = clampCost(std::stoi((*it)[2].str()));
        new_costs[lab] = v;
      }
    }
  }
  {
    std::regex witem_re(R"REGEX(\{\s*"zone"\s*:\s*"([A-Z])"[^}]*?"cost"\s*:\s*([0-9]{1,3})[^}]*\})REGEX");
    auto begin = std::sregex_iterator(s.begin(), s.end(), witem_re);
    auto end   = std::sregex_iterator();
    for (auto it = begin; it != end; ++it) {
      std::string lab = (*it)[1].str();
      int v = clampCost(std::stoi((*it)[2].str()));
      new_costs[lab] = v;
    }
  }

  zone_costs_.swap(new_costs);

  std::vector<std::string> keys_now;
  keys_now.reserve(zone_costs_.size());
  for (auto& kv : zone_costs_) keys_now.push_back(kv.first);

  double prev_min_x, prev_min_y, prev_max_x, prev_max_y; bool prev_valid=false;
  computeZonesUnion(prev_keys, prev_min_x, prev_min_y, prev_max_x, prev_max_y, prev_valid);

  double new_min_x, new_min_y, new_max_x, new_max_y; bool new_valid=false;
  computeZonesUnion(keys_now, new_min_x, new_min_y, new_max_x, new_max_y, new_valid);

  if (prev_valid || new_valid) {
    last_min_x_ = prev_valid ? std::min(prev_min_x, new_min_x) : new_min_x;
    last_min_y_ = prev_valid ? std::min(prev_min_y, new_min_y) : new_min_y;
    last_max_x_ = prev_valid ? std::max(prev_max_x, new_max_x) : new_max_x;
    last_max_y_ = prev_valid ? std::max(prev_max_y, new_max_y) : new_max_y;
  } else {
    last_min_x_ = last_min_y_ = last_max_x_ = last_max_y_ = 0.0;
  }

  prev_valid_ = new_valid;
  prev_min_x_ = new_min_x; prev_min_y_ = new_min_y;
  prev_max_x_ = new_max_x; prev_max_y_ = new_max_y;

  updated_ = (prev_valid || new_valid);
  current_ = false;

  RCLCPP_INFO(rclcpp::get_logger("ZoneSoftCostLayer"),
              "[PARSED] zones=%zu keys=%s",
              zone_costs_.size(),
              [&](){ std::string s;
                    for (auto& kv: zone_costs_) { if(!s.empty()) s+=','; s+=kv.first+":"+std::to_string(kv.second); }
                    return s; }().c_str());

  RCLCPP_INFO(rclcpp::get_logger("ZoneSoftCostLayer"),
              "[ROI-WORLD] last: [%.2f,%.2f] ~ [%.2f,%.2f] updated_=%d",
              last_min_x_, last_min_y_, last_max_x_, last_max_y_, (int)updated_);
}

void ZoneSoftCostLayer::updateBounds(double, double, double,
                                     double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_ || !updated_) return;
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());
  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
}

void ZoneSoftCostLayer::updateCosts(nav2_costmap_2d::Costmap2D& master,
                                    int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());

  const int grid_w = static_cast<int>(getSizeInCellsX());
  const int grid_h = static_cast<int>(getSizeInCellsY());

  const int roi_x0 = std::max(0, min_i);
  const int roi_y0 = std::max(0, min_j);
  const int roi_x1 = std::min(grid_w, max_i);
  const int roi_y1 = std::min(grid_h, max_j);

  RCLCPP_INFO(rclcpp::get_logger("ZoneSoftCostLayer"),
              "[ROI-CELLS] in=(%d,%d)~(%d,%d) grid=(%d,%d)",
              min_i, min_j, max_i, max_j, grid_w, grid_h);

  if (roi_x0 >= roi_x1 || roi_y0 >= roi_y1) {
    RCLCPP_WARN(rclcpp::get_logger("ZoneSoftCostLayer"), "ROI empty → skip");
    updated_ = false; current_ = true; return;
  }

  for (int y = roi_y0; y < roi_y1; ++y)
    for (int x = roi_x0; x < roi_x1; ++x)
      setCost(x, y, nav2_costmap_2d::NO_INFORMATION);

  size_t painted = 0;
  for (const auto& kv : zone_costs_) {
    const std::string& name = kv.first;
    int val = clampCost(kv.second);

    auto it = zone_db_.find(name);
    if (it == zone_db_.end()) continue;
    const auto& z = it->second;

    unsigned int umx0, umy0, umx1, umy1;
    if (!master.worldToMap(z.x_min, z.y_min, umx0, umy0)) {
      RCLCPP_WARN(rclcpp::get_logger("ZoneSoftCostLayer"), "worldToMap(min) fail %s", name.c_str());
      continue;
    }
    if (!master.worldToMap(z.x_max, z.y_max, umx1, umy1)) {
      RCLCPP_WARN(rclcpp::get_logger("ZoneSoftCostLayer"), "worldToMap(max) fail %s", name.c_str());
      continue;
    }

    int zx0 = std::min((int)umx0, (int)umx1);
    int zy0 = std::min((int)umy0, (int)umy1);
    int zx1 = std::max((int)umx0, (int)umx1) + 1;
    int zy1 = std::max((int)umy0, (int)umy1) + 1;

    int ix0 = std::max(roi_x0, zx0);
    int iy0 = std::max(roi_y0, zy0);
    int ix1 = std::min(roi_x1, zx1);
    int iy1 = std::min(roi_y1, zy1);
    if (ix0 >= ix1 || iy0 >= iy1) continue;

    for (int y = iy0; y < iy1; ++y) {
      for (int x = ix0; x < ix1; ++x) {
        const unsigned char prev = getCost(x, y);
        const unsigned char cval = static_cast<unsigned char>(val);

        const unsigned char nv = (prev == nav2_costmap_2d::NO_INFORMATION) ? cval
                                                                       : std::max(prev, cval);
        setCost(x, y, nv);
        if (nv != prev) ++painted;
      }
    }

  }

  updateWithMax(master, roi_x0, roi_y0, roi_x1, roi_y1);

  RCLCPP_INFO(rclcpp::get_logger("ZoneSoftCostLayer"),
              "[PAINT] cells=%zu (after merge)", painted);

  updated_ = false;
  current_ = true;
}

void ZoneSoftCostLayer::reset()
{
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());

  for (unsigned int j = 0; j < getSizeInCellsY(); ++j)
    for (unsigned int i = 0; i < getSizeInCellsX(); ++i)
      setCost(i, j, nav2_costmap_2d::NO_INFORMATION);

  zone_costs_.clear();
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

PLUGINLIB_EXPORT_CLASS(my_costmap_layers::ZoneSoftCostLayer, nav2_costmap_2d::Layer)

