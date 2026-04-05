#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class SimpleSlamNode : public rclcpp::Node
{
public:
   SimpleSlamNode() : Node("SimpleSlamNode")
   {
      m_gridOccupancyMap = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      m_subscription = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 20
      , std::bind(&SimpleSlamNode::OnScannedDataReceived, this, std::placeholders::_1));
 
      InitializeMapData();

      m_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 20);
   }

private:
   void OnScannedDataReceived(sensor_msgs::msg::LaserScan::SharedPtr scanData);
   void InitializeMapData();
   void update_map(double x, double y);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_subscription;
  nav_msgs::msg::OccupancyGrid::SharedPtr m_gridOccupancyMap;
};

void SimpleSlamNode::InitializeMapData()
{
   uint32_t width                             = 500;
   m_gridOccupancyMap->info.resolution        = 0.05;
   m_gridOccupancyMap->info.width             = width;
   m_gridOccupancyMap->info.height            = width;
   m_gridOccupancyMap->info.origin.position.x = -10;
   m_gridOccupancyMap->info.origin.position.y = -10;

   m_gridOccupancyMap->data.resize(width * width, -1);
}

void SimpleSlamNode::OnScannedDataReceived(sensor_msgs::msg::LaserScan::SharedPtr scanData)
{
   double angle = scanData->angle_min;

   for (size_t i = 0; i < scanData->ranges.size(); i++) {
            double r = scanData->ranges[i];

            if (std::isinf(r) || std::isnan(r)) {
                angle += scanData->angle_increment;
                continue;
            }

            double x = r * cos(angle);
            double y = r * sin(angle);

            update_map(x, y);

            angle += scanData->angle_increment;
        }

        m_gridOccupancyMap->header.stamp = now();
        m_gridOccupancyMap->header.frame_id = "map";

        m_publisher->publish(*m_gridOccupancyMap); 
}

void SimpleSlamNode::update_map(double x, double y) {

        int mx = (x - m_gridOccupancyMap->info.origin.position.x) / m_gridOccupancyMap->info.resolution;
        int my = (y - m_gridOccupancyMap->info.origin.position.y) / m_gridOccupancyMap->info.resolution;

        if (!std::isnormal(mx) || !std::isnormal(my))
        {
           return;
        }

        int index = my * m_gridOccupancyMap->info.width + mx;

        if (index < 0 || index >= m_gridOccupancyMap->data.size())
        {
          return;
        }

        m_gridOccupancyMap->data[index] = 100;  // occupied
}

int main(int argc, char* argv[])
{

   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<SimpleSlamNode>());
   rclcpp::shutdown();

   return EXIT_SUCCESS;
}
