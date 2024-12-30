#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <cmath>
#include <memory>
#include <utility>

int main(int argc, char ** argv)
{
  // Initialize node and publisher
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("static_grid_map_demo");
  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());

  // Create grid map
  grid_map::GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(1.2, 2.0), 0.03);  // 1.2m x 2.0m with 0.03m resolution

  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Fill the map with a consistent pattern - a hill in the center
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map.getPosition(*it, position);
    
    // Calculate distance from center
    double center_x = map.getLength().x() / 2.0;
    double center_y = map.getLength().y() / 2.0;
    double dx = position.x() - center_x;
    double dy = position.y() - center_y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // Create a gaussian hill
    double sigma = 0.3;  // Controls the width of the hill
    double height = 0.2; // Maximum height of the hill
    map.at("elevation", *it) = height * std::exp(-(distance * distance) / (2 * sigma * sigma));
  }

  // Publish loop
  rclcpp::Rate rate(1.0);  // Reduced rate since map doesn't change
  rclcpp::Clock clock;
  while (rclcpp::ok()) {
    // Publish grid map
    map.setTimestamp(node->now().nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map);
    publisher->publish(std::move(message));
    
    
    rate.sleep();
  }

  return 0;
}