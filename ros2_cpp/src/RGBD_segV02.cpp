// ROS2 related libraries
#include <rclcpp/rclcpp.hpp>
// For synchronization of messages, we need these libraries
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL required libraries
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
// For segmentation part
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
// For Clustering part
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
// For Rviz markers
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
// For convertion of camera_optical frame to world frame
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// For Eigen related manipulation
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

// For server definations
#include <ros2_msgs/srv/get_centroid.hpp>

#include <cmath>
#include <memory>

typedef pcl::PointXYZRGB PointT;

class FemtoSubscriber : public rclcpp::Node
{
public:
    FemtoSubscriber() : Node("femto_subscriber"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        point_cloud_.subscribe(this, "/femto/points", rmw_qos_profile_sensor_data);
        rgb_sub_.subscribe(this, "/femto/image_raw", rmw_qos_profile_sensor_data);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), point_cloud_, rgb_sub_
        );
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.033));

        sync_->registerCallback(
            std::bind(&FemtoSubscriber::syncCallback, this,
                       std::placeholders::_1, std::placeholders::_2)
        );
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filter_cloud", 10);
        cluster_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud", 10);
        centroid_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_centroids", 10);

        centroid_service_ = create_service<ros2_msgs::srv::GetCentroid>("get_centroid", std::bind(&FemtoSubscriber::handleCentroidRequest, this, std::placeholders::_1, std::placeholders::_2));


    }
    inline void visualize(const pcl::PointCloud<PointT>::Ptr& cloud,
                          const std_msgs::msg::Header& header,
                          const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub)
    {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header = header;
        pub->publish(msg);
    }
    static std::tuple<uint8_t, uint8_t, uint8_t> clusterColour(std::size_t idx)
    {
        // 10 visually distinct colours
        static const uint8_t palette[][3] = {
            {255, 50, 50}, {50, 255, 50}, {50, 50, 255},
            {255, 255, 0}, {0, 255, 255}, {255, 0, 255},
            {255, 165, 0}, {128, 0, 128}, {0, 128, 128},
            {128, 128, 0}
        };
        constexpr std::size_t N = 10;
        const auto& c = palette[idx % N];
        return {c[0], c[1], c[2]};
    }
    std::optional<geometry_msgs::msg::Pose>
    toWorldFrame(
        const Eigen::Vector4f& centroid_sensor,
        const Eigen::Matrix3f& eigen_vectors,
        const std_msgs::msg::Header& cloud_header
    )
    {
        try
        {
            auto tf = tf_buffer_.lookupTransform(
                "world",
                cloud_header.frame_id,
                rclcpp::Time(cloud_header.stamp),
                rclcpp::Duration::from_seconds(0.05)
            );

            // TF as Eigen transform
            Eigen::Affine3d tf_eigen =
                tf2::transformToEigen(tf);

            // -----------------------------
            // POSITION TRANSFORM
            // -----------------------------
            Eigen::Vector3d centroid_world =
                tf_eigen *
                Eigen::Vector3d(
                    centroid_sensor[0],
                    centroid_sensor[1],
                    centroid_sensor[2]
                );

            // -----------------------------
            // ORIENTATION TRANSFORM
            // -----------------------------

            // Rotation from camera -> world
            Eigen::Matrix3d R_world_camera =
                tf_eigen.rotation();

            // PCA rotation in camera frame
            Eigen::Matrix3d R_camera_object =
                eigen_vectors.cast<double>();

            // Object rotation in world frame
            Eigen::Matrix3d R_world_object =
                R_world_camera * R_camera_object;

            // Convert to quaternion
            Eigen::Quaterniond quat_world(R_world_object);

            // -----------------------------
            // BUILD POSE
            // -----------------------------
            geometry_msgs::msg::Pose pose;

            pose.position.x = centroid_world.x();
            pose.position.y = centroid_world.y();
            pose.position.z = centroid_world.z();

            pose.orientation.x = quat_world.x();
            pose.orientation.y = quat_world.y();
            pose.orientation.z = quat_world.z();
            pose.orientation.w = quat_world.w();

            return pose;
        }
        catch(const tf2::TransformException& ex)
        {
            RCLCPP_WARN(
                get_logger(),
                "TF lookup failed: %s",
                ex.what()
            );

            return std::nullopt;
        }
    }
    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud,
                      const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg)
    {
        (void)rgb_msg;
        pcl::fromROSMsg(*point_cloud, *cloud);

        // --- Voxel Filtering ---
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.005, 0.005, 0.005);
        voxel_filter.filter(*cloud);

        // --- PassThrough Filter along z axis ---
        pcl::PassThrough<PointT> passing_z;
        passing_z.setInputCloud(cloud);
        passing_z.setFilterFieldName("z");
        passing_z.setFilterLimits(0.0, 0.4);
        passing_z.filter(*cloud);

        // --- Passthrough Filter along y axis ---
        pcl::PassThrough<PointT> passing_y;
        passing_y.setInputCloud(cloud);
        passing_y.setFilterFieldName("y");
        passing_y.setFilterLimits(-0.1, 0.2);
        passing_y.filter(*cloud);

        // --- Passthrough Filter along x axis ---
        pcl::PassThrough<PointT> passing_x;
        passing_x.setInputCloud(cloud);
        passing_x.setFilterFieldName("x");
        passing_x.setFilterLimits(-0.35, 0.1);
        passing_x.filter(*cloud);

        visualize(cloud, point_cloud->header, pub_);

        // --- Clustering components into individual components ---
        if (cloud->empty()) return;

        pcl::search::KdTree<PointT>::Ptr tree = std::make_shared<pcl::search::KdTree<PointT>>();
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.02);
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        auto merged = std::make_shared<pcl::PointCloud<PointT>>();

        visualization_msgs::msg::MarkerArray marker_array;
        {
            visualization_msgs::msg::Marker del;

            del.header.frame_id = point_cloud->header.frame_id;
            del.header.stamp = point_cloud->header.stamp;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(del);
        }
        RCLCPP_INFO(get_logger(), "--- Cluster found: %zu ---", cluster_indices.size());
        for(std::size_t ci=0; ci<cluster_indices.size(); ++ci)
        {
            const auto& indices = cluster_indices[ci];
            auto [r, g, b] = clusterColour(ci);

            auto cluster_cloud = std::make_shared<pcl::PointCloud<PointT>>();
            cluster_cloud->reserve(indices.indices.size());
            for(int idx : indices.indices)
            {
                PointT pt = (*cloud)[idx];
                pt.r = r;
                pt.g = g;
                pt.b = b;

                cluster_cloud->push_back(pt);
            }

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster_cloud, centroid);
            // Let's compute the orientation
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*cluster_cloud, centroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            Eigen::Matrix3f rotation = solver.eigenvectors();
            Eigen::Quaternionf quat(rotation);

            auto world = toWorldFrame(centroid, rotation, point_cloud->header);

            if(!world)
                continue;

            latest_pose_ = *world;
            centroid_available_ = true;

            RCLCPP_INFO(
                get_logger(),
                "Cluster #%zu | pts=%zu | "
                "Position: (x=%.4f y=%.4f z=%.4f) m | "
                "Orientation: (qx=%.4f qy=%.4f qz=%.4f qw=%.4f)",
                ci,
                indices.indices.size(),

                world->position.x,
                world->position.y,
                world->position.z,

                world->orientation.x,
                world->orientation.y,
                world->orientation.z,
                world->orientation.w
            );

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = point_cloud->header.frame_id;
            marker.header.stamp    = point_cloud->header.stamp;
            marker.ns = "centroids";
            marker.id = static_cast<int>(ci);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.03;
            marker.scale.y = 0.03;
            marker.scale.z = 0.03;
            marker.color.r = r / 255.0f;
            marker.color.g = g / 255.0f;
            marker.color.b = b / 255.0f;
            marker.color.a = 0.9f;
            marker.lifetime =
                rclcpp::Duration::from_seconds(0.1);
            marker_array.markers.push_back(marker);

            *merged += *cluster_cloud;
        }
        visualize(merged, point_cloud->header, cluster_pub_);
        centroid_pub_->publish(marker_array);
    }
    void handleCentroidRequest(
        const std::shared_ptr<ros2_msgs::srv::GetCentroid::Request> request,
        const std::shared_ptr<ros2_msgs::srv::GetCentroid::Response> response)
    {
        (void)request;

        if(!centroid_available_)
        {
            response->x = 0.0;
            response->y = 0.0;
            response->z = 0.0;

            response->qx = 0.0;
            response->qy = 0.0;
            response->qz = 0.0;
            response->qw = 1.0;

            response->success = false;

            return;
        }

        // Position
        response->x = latest_pose_.position.x;
        response->y = latest_pose_.position.y;
        response->z = latest_pose_.position.z;

        // Orientation
        response->qx = latest_pose_.orientation.x;
        response->qy = latest_pose_.orientation.y;
        response->qz = latest_pose_.orientation.z;
        response->qw = latest_pose_.orientation.w;

        response->success = true;
    }
private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::Pose latest_pose_;
    bool centroid_available_ = false;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centroid_pub_;
    rclcpp::Service<ros2_msgs::srv::GetCentroid>::SharedPtr centroid_service_;

    pcl::PointCloud<PointT>::Ptr cloud = std::make_shared<pcl::PointCloud<PointT>>();

    pcl::VoxelGrid<PointT> voxel_filter;
};

int main(int argc,char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FemtoSubscriber>());
    rclcpp::shutdown();

    return 0;
}