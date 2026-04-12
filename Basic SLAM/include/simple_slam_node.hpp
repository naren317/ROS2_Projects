#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

struct RobotData
{
   double xCoord;
   double yCoord;
   double yaw;
};

namespace SimpleSlamSpace {

class SimpleSlamNode : public rclcpp::Node
{
public:
   explicit SimpleSlamNode();

private:
   void OnScannedDataReceived(sensor_msgs::msg::LaserScan::SharedPtr scanData);
   void InitializeMapData();
   void UpdateMap(double x, double y);
   void OnOdometricDataReceived(nav_msgs::msg::Odometry::SharedPtr odomData);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_subscription;
  nav_msgs::msg::OccupancyGrid::SharedPtr m_gridOccupancyMap;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odomSubscriber;
  RobotData m_robotData;
};

}

