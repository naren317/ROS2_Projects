#include "simple_slam_node.hpp"

static constexpr double ONE = 1.0;
static constexpr double TWO = 2.0;

namespace SimpleSlamSpace {

SimpleSlamNode::SimpleSlamNode()
   : rclcpp::Node("SimpleSlamNode")
   , m_robotData({0.0, 0.0, 0.0})
   {
      m_gridOccupancyMap = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      m_subscription = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 20
      , std::bind(&SimpleSlamNode::OnScannedDataReceived, this, std::placeholders::_1));

      m_odomSubscriber = create_subscription<nav_msgs::msg::Odometry>("/odom", 20,
      std::bind(&SimpleSlamNode::OnOdometricDataReceived, this, std::placeholders::_1));
 
      InitializeMapData();

      m_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 20);
}

void SimpleSlamNode::InitializeMapData()
{
   uint32_t width                             = 500;
   m_gridOccupancyMap->info.resolution        = 0.05;
   m_gridOccupancyMap->info.width             = width;
   m_gridOccupancyMap->info.height            = width;
   m_gridOccupancyMap->info.origin.position.x = -20;
   m_gridOccupancyMap->info.origin.position.y = -20;

   m_gridOccupancyMap->data.resize(width * width, -1);
}

void SimpleSlamNode::OnScannedDataReceived(sensor_msgs::msg::LaserScan::SharedPtr scanData)
{
   double angle = scanData->angle_min;

   for (size_t i = 0; i < scanData->ranges.size(); i++) {
            double range = scanData->ranges[i];

            if (std::isinf(range) || std::isnan(range)) {
                angle += scanData->angle_increment;
                continue;
            }

            double localX = range * cos(angle);
            double localY = range * sin(angle);

            double globalX = m_robotData.xCoord + (localX * std::cos(m_robotData.yaw) - localY * std::sin(m_robotData.yaw));
            double globalY = m_robotData.yCoord + (localX * std::sin(m_robotData.yaw) + localY * std::cos(m_robotData.yaw));

            UpdateMap(globalX, globalY);

            angle += scanData->angle_increment;
        }

        m_gridOccupancyMap->header.stamp = now();
        m_gridOccupancyMap->header.frame_id = "map";

        m_publisher->publish(*m_gridOccupancyMap); 
}

void SimpleSlamNode::UpdateMap(double x, double y) {

        int mx = (x - m_gridOccupancyMap->info.origin.position.x) / m_gridOccupancyMap->info.resolution;
        int my = (y - m_gridOccupancyMap->info.origin.position.y) / m_gridOccupancyMap->info.resolution;

        if (!std::isnormal(mx) || !std::isnormal(my))
        {
           return;
        }

        int index = my * m_gridOccupancyMap->info.width + mx;

        if (index < 0 || index >= static_cast <int>(m_gridOccupancyMap->data.size()))
        {
          return;
        }

        m_gridOccupancyMap->data[index] = 100;
}


void SimpleSlamNode::OnOdometricDataReceived(nav_msgs::msg::Odometry::SharedPtr odomData)
{
  m_robotData.xCoord = odomData->pose.pose.position.x;
  m_robotData.yCoord = odomData->pose.pose.position.y;

  double sinYcosP = TWO * (odomData->pose.pose.orientation.w * odomData->pose.pose.orientation.z +
                          odomData->pose.pose.orientation.x * odomData->pose.pose.orientation.y);
  double cosYcosP = ONE - TWO * (std::pow(odomData->pose.pose.orientation.y, TWO) +
                                std::pow(odomData->pose.pose.orientation.z, TWO));

  m_robotData.yaw = std::atan2(sinYcosP, cosYcosP);
}

}

