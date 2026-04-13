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
    PointCloud()) : Node("PointCloudProcessing")
    {
        // QoS for sensor data (IMPORTANT in Jazzy)
        m_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_image/points",
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloud::onPointCloudReceived, this, std::placeholders::_1)
        );

        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscription;

    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    void onPointCloudReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS message to point cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
            return;

        // Downsample, voxel filtering.
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*cloud_filtered);

        // RANSAC and Remove ground
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.02);
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No plane found");
            return;
        }

        // Extract objects (remove plane)
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_objects);

        // Clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_objects);

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.1);
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_objects);
        ec.extract(cluster_indices);

        int cluster_id = 0;

        for (const auto& indices : cluster_indices)
        {
            float cx = 0, cy = 0, cz = 0;

            for (int idx : indices.indices)
            {
                cx += cloud_objects->points[idx].x;
                cy += cloud_objects->points[idx].y;
                cz += cloud_objects->points[idx].z;
            }

            int size = indices.indices.size();
            cx /= size;
            cy /= size;
            cz /= size;

            // Step 4: Transform to world frame
            geometry_msgs::msg::PointStamped pt, pt_world;

            pt.header = msg->header;
            pt.point.x = cx;
            pt.point.y = cy;
            pt.point.z = cz;

            try {
                pt_world = m_tfBuffer->transform(pt, "world");

                RCLCPP_INFO(this->get_logger(),
                    "Cluster %d WORLD (%.2f, %.2f, %.2f)",
                    cluster_id++,
                    pt_world.point.x,
                    pt_world.point.y,
                    pt_world.point.z);

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
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
