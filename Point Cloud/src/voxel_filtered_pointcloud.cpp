#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

static constexpr size_t QOS_QUEUE = 10;
class CloudFilter : public rclcpp::Node
{
public:
  CloudFilter() : Node("cloud_filter")
  {
    m_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/points", QOS_QUEUE,
      std::bind(&CloudFilter::callback, this, std::placeholders::_1));

    m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_points", QOS_QUEUE);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_in);

    // Downsampling the points.
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_in);
    
    // Set the voxel size.
    voxel.setLeafSize(0.1f, 0.1f, 0.1f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*cloud_filtered);

    // Convert back to a ROS pointcloud.
    sensor_msgs::msg::PointCloud2 output;
    
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = msg->header;

    m_publisher->publish(output);

    RCLCPP_INFO(this->get_logger(),
                "Input: %lu points | Output: %lu points",
                cloud_in->size(), cloud_filtered->size());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudFilter>());
  rclcpp::shutdown();
  return 0;
}
