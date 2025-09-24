#include "rclcpp/rclcpp.hpp"

class DriveArbitratorNode : public rclcpp::Node
{
public:
  DriveArbitratorNode()
  : Node("drive_arbitrator_node")
  {
    RCLCPP_INFO(this->get_logger(), "Drive Arbitrator Node started.");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveArbitratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
