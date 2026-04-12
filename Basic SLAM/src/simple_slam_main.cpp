#include "rclcpp/rclcpp.hpp"
#include "simple_slam_node.hpp"

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<SimpleSlamSpace::SimpleSlamNode>());
   rclcpp::shutdown();

   return EXIT_SUCCESS;
}
