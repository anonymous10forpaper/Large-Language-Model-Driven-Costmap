#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include "my_costmap_layers/SoftCostOverrides.hpp"
#include "my_costmap_layers/ZoneSoftCostLayer.hpp"

using my_costmap_layers::parseSoftCostOverrides;
using Costs = std::unordered_map<std::string, int>;

TEST(SoftCostOverrides, PreservesEveryAllowedIntegerInBothFormats)
{
  Costs costs;
  std::string error;
  for (int value = 0; value <= 252; ++value) {
    for (const auto & payload : {
        nlohmann::json{{"zones", {{"A", value}}}},
        nlohmann::json{{"weights", {{{"cost", value}, {"zone", "B"}}}}}})
    {
      ASSERT_TRUE(parseSoftCostOverrides(payload.dump(), costs, error)) << error;
      ASSERT_EQ(costs.size(), 1u);
      EXPECT_EQ(costs.begin()->second, value);
    }
  }
}

TEST(SoftCostOverrides, RejectsInvalidCostsWithoutChangingPreviousState)
{
  const std::vector<nlohmann::json> invalid = {
    -1, 253, 254, 255, 256, 999, 1000,
    std::numeric_limits<uint64_t>::max(), 252.0, 252.9, -0.1,
    true, false, nullptr, "252", "253", nlohmann::json::array(),
    nlohmann::json::object()};
  const Costs previous{{"E", 200}};
  for (const auto & value : invalid) {
    for (const auto & payload : {
        nlohmann::json{{"zones", {{"A", 5}, {"B", value}}}},
        nlohmann::json{{"weights", {
            {{"zone", "A"}, {"cost", 5}}, {{"zone", "B"}, {"cost", value}}}}}})
    {
      SCOPED_TRACE(payload.dump());
      auto costs = previous;
      std::string error;
      EXPECT_FALSE(parseSoftCostOverrides(payload.dump(), costs, error));
      EXPECT_EQ(costs, previous);
      EXPECT_FALSE(error.empty());
    }
  }
}

TEST(SoftCostOverrides, RejectsMalformedOrConflictingUpdatesAtomically)
{
  const std::vector<std::string> invalid = {
    "", "{", "[]", "null", "{}", R"({"zone": "A", "cost": 200})",
    R"({"zones": null})", R"({"zones": []})", R"({"weights": {}})",
    R"({"zones": {"A": 5}, "weights": null})",
    R"({"weights": [{"zone": "A"}]})", R"({"weights": [5]})",
    R"({"weights": [{"zone": 1, "cost": 200}]})",
    R"({"zones": {"a": 5}})", R"({"zones": {"AA": 5}})",
    R"({"zones": {"A": 253, "A": 5}})",
    R"({"zones": {"A": 253}, "zones": {"B": 5}})",
    R"({"zones": {"A": 5}, "weights": [{"zone": "A", "cost": 200}]})",
    R"({"weights": [{"zone": "A", "cost": 5}, {"zone": "A", "cost": 200}]})",
    R"({"zones": {"A": 5}} trailing)"
  };
  const Costs previous{{"E", 200}};
  for (const auto & payload : invalid) {
    SCOPED_TRACE(payload);
    auto costs = previous;
    std::string error;
    EXPECT_FALSE(parseSoftCostOverrides(payload, costs, error));
    EXPECT_EQ(costs, previous);
  }
}

TEST(SoftCostOverrides, ValidUpdatesReplaceRatherThanAccumulate)
{
  Costs costs{{"E", 200}};
  std::string error;
  ASSERT_TRUE(parseSoftCostOverrides(
      R"({"zones": {"A": 252}, "weights": [{"zone": "A", "cost": 252}, {"zone": "B", "cost": 0}]})",
      costs, error)) << error;
  EXPECT_EQ(costs, (Costs{{"A", 252}, {"B", 0}}));
  for (const auto & payload : {R"({"zones": {}})", R"({"weights": []})"}) {
    costs = {{"E", 200}};
    ASSERT_TRUE(parseSoftCostOverrides(payload, costs, error)) << error;
    EXPECT_TRUE(costs.empty());
  }
}

class SoftCostLayerTopicTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<nav2_util::LifecycleNode>("soft_cost_layer_test");
    publisher_node_ = std::make_shared<rclcpp::Node>("soft_cost_test_sender");
    node_->declare_parameter("soft.zone_cost_overrides_topic", "/soft_cost_contract/costs");
    node_->declare_parameter("soft.zone_geometry_topic", "/soft_cost_contract/geometry");
    map_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    map_->resizeMap(60, 60, 1.0, -25.0, -25.0);
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer_ = std::make_unique<my_costmap_layers::ZoneSoftCostLayer>();
    layer_->initialize(map_.get(), "soft", nullptr, node_, group_);
    publisher_ = publisher_node_->create_publisher<std_msgs::msg::String>(
      "/soft_cost_contract/costs", rclcpp::QoS(10).reliable().transient_local());
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(publisher_node_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (publisher_->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(publisher_->get_subscription_count(), 0u);
  }

  void TearDown() override
  {
    executor_.remove_node(node_->get_node_base_interface());
    executor_.remove_node(publisher_node_);
    publisher_.reset();
    layer_.reset();
    map_.reset();
    group_.reset();
    node_.reset();
    publisher_node_.reset();
  }

  void send(const std::string & payload)
  {
    std_msgs::msg::String message;
    message.data = payload;
    publisher_->publish(message);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    do {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    auto * master = map_->getCostmap();
    master->resetMap(0, 0, master->getSizeInCellsX(), master->getSizeInCellsY());
    layer_->updateCosts(*master, 0, 0, master->getSizeInCellsX(), master->getSizeInCellsY());
  }

  unsigned char cost(double x, double y)
  {
    auto * master = map_->getCostmap();
    unsigned int mx, my;
    if (!master->worldToMap(x, y, mx, my)) {
      ADD_FAILURE() << "Test point outside costmap";
      return nav2_costmap_2d::NO_INFORMATION;
    }
    return master->getCost(mx, my);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  nav2_util::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map_;
  std::unique_ptr<my_costmap_layers::ZoneSoftCostLayer> layer_;
};

TEST_F(SoftCostLayerTopicTest, Applies252AndRetainsCostsAfterRejectedMessages)
{
  send(R"({"zones": {"A": 252}})");
  ASSERT_EQ(cost(10.0, 0.0), 252);
  for (const auto & payload : {
      R"({"zones": {"A": 5, "B": 253}})",
      R"({"zones": {"A": 5, "B": 254}})",
      R"({"zones": {"A": 5, "B": 255}})",
      R"({"weights": [{"zone": "A", "cost": 5}, {"zone": "B", "cost": 252.9}]})",
      R"({"zones": {"A": 5, "B": -1}})",
      R"({"zones": {"A": 5)"
    })
  {
    send(payload);
    EXPECT_EQ(cost(10.0, 0.0), 252) << payload;
    EXPECT_EQ(cost(10.0, -4.0), nav2_costmap_2d::FREE_SPACE) << payload;
  }
  send(R"({"weights": [{"zone": "B", "cost": 200}]})");
  EXPECT_EQ(cost(10.0, 0.0), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(10.0, -4.0), 200);
  send(R"({"zones": {}})");
  EXPECT_EQ(cost(10.0, -4.0), nav2_costmap_2d::FREE_SPACE);
}
