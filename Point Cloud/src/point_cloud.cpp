#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Filters
#include <pcl/filters/voxel_grid.h>

// Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// Clustering
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

// TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class PointCloud : public rclcpp::Node
{
public:
    PointCloud()
    : Node("PointCloud")
    , m_cloud (new pcl::PointCloud <pcl::PointXYZ>)
    , m_inliers (new pcl::PointIndices)
    , m_coefficients (new pcl::ModelCoefficients)
    , m_kdTree (new pcl::search::KdTree <pcl::PointXYZ>)
    {
        m_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_image/points",
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloud::onPointCloudReceived, this, std::placeholders::_1)
        );


        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer, this, false);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscription;

    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::PointIndices::Ptr m_inliers;
    pcl::ModelCoefficients::Ptr m_coefficients;
    pcl::search::KdTree <pcl::PointXYZ>::Ptr m_kdTree;

private:
    void onPointCloudReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        geometry_msgs::msg::PointStamped pointStamped, pt_world;
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> euClustExtract;

        // Convert ROS message to point cloud.
        pcl::fromROSMsg(*msg, *m_cloud);

        if (m_cloud->empty())
        {
            return;
        }

        // Downsample, voxel filtering.
        vg.setInputCloud(m_cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);

        m_cloud->clear();
        m_inliers->indices.clear();
        m_coefficients->values.clear();

        vg.filter(*m_cloud);

        // RANSAC and Remove ground
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(m_cloud);
        seg.segment(*m_inliers, *m_coefficients);

        if (m_inliers->indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No plane found");
            return;
        }

        // Extract objects (remove plane)
        extract.setInputCloud(m_cloud);
        extract.setIndices(m_inliers);
        extract.setNegative(true);

        m_cloud->clear();

        extract.filter(*m_cloud);

        // Clustering
        m_kdTree->setInputCloud(m_cloud);

        euClustExtract.setClusterTolerance(0.1);
        euClustExtract.setMinClusterSize(50);
        euClustExtract.setMaxClusterSize(25000);
        euClustExtract.setSearchMethod(m_kdTree);
        euClustExtract.setInputCloud(m_cloud);
        euClustExtract.extract(cluster_indices);

        for (auto& indices : cluster_indices)
        {
            float cx = 0, cy = 0, cz = 0;

            for (int idx : indices.indices)
            {
                cx += m_cloud->points[idx].x;
                cy += m_cloud->points[idx].y;
                cz += m_cloud->points[idx].z;
            }

            int size = indices.indices.size();
            cx /= size;
            cy /= size;
            cz /= size;

            // Transforming to world frame

            pointStamped.header = msg->header;
            pointStamped.point.x = cx;
            pointStamped.point.y = cy;
            pointStamped.point.z = cz;

            try
            {
                pt_world = m_tfBuffer->transform(pointStamped, "world");

            }

            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
            }

            catch (const std::exception& ex)
            {
                RCLCPP_WARN(get_logger(), "Unknow exception occurred: %s", ex.what());
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloud>());
    rclcpp::shutdown();
    return 0;
}
