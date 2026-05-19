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

#include <cmath>
#include <memory>

typedef pcl::PointXYZRGB PointT;

class FemtoSubscriber : public rclcpp::Node
{
public:
    FemtoSubscriber() : Node("femto_subscriber"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        declare_parameter<bool>("debug", false);
        get_parameter("debug", debug_);
        declare_parameter<bool>("voxel", false);
        get_parameter("voxel", voxel_);
        declare_parameter<bool>("passz", false);
        get_parameter("passz", passz_);
        declare_parameter<bool>("passy", false);
        get_parameter("passy", passy_);
        declare_parameter<bool>("passx", false);
        get_parameter("passx", passx_);

        declare_parameter<bool>("cluster", false);
        get_parameter("cluster", cluster_);
        declare_parameter<bool>("print_centroids", false);
        get_parameter("print_centroids", print_centroids_);
        declare_parameter<double>("cluster_tolerance", 0.02); //meters
        get_parameter("cluster_tolerance", cluster_tolerance_);
        declare_parameter<int>("cluster_min_size", 50);
        get_parameter("cluster_min_size", cluster_min_size_);
        declare_parameter<int>("cluster_max_size", 25000);
        get_parameter("cluster_max_size", cluster_max_size_);

        // declare_parameter<bool>("passz_seg", false);
        // get_parameter("passz_seg", passz_seg_);

        point_cloud_.subscribe(this, "/femto/points", rmw_qos_profile_sensor_data);
        rgb_sub_.subscribe(this, "/femto/image_raw", rmw_qos_profile_sensor_data);

        // Buffers up to 10 messages per topic, matches within 33ms
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), point_cloud_, rgb_sub_
        );
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.033));

        sync_->registerCallback(
            std::bind(&FemtoSubscriber::syncCallback, this,
                       std::placeholders::_1, std::placeholders::_2)
        );

        if(voxel_)
            voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_cloud", 10);
        if(passz_)
            passz_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/passz_cloud", 10);
        if(passy_)
            passy_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/passy_cloud", 10);
        if(passx_)
            passx_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/passx_cloud", 10);
        if(cluster_)
        {
            // Individual coloured cluster clouds (one topic, all clusters merged with color)
            cluster_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud", 10);
            // Sphere markers at each centroid
            centroid_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_centroids", 10);
        }
    }

    static inline void visualize(const pcl::PointCloud<PointT>::Ptr& cloud,
                              const std_msgs::msg::Header& header,
                              const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub)
    {
        if (!pub) return; // false then publisher never created, lets skip this
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header = header;
        pub->publish(msg);
    }

    // Distinct RGB colours for up to N clusters (cycles if more)
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

    // Returns world-frame centroid, or std::nullopt if TF lookup fails
    std::optional<Eigen::Vector3d> toWorldFrame(const Eigen::Vector4f& centroid_sensor,
                                                const std_msgs::msg::Header& cloud_header)
    {
        try
        {
            auto tf = tf_buffer_.lookupTransform(
                "world",
                cloud_header.frame_id,
                rclcpp::Time(cloud_header.stamp),
                rclcpp::Duration::from_seconds(0.05)
            );
            Eigen::Vector3d c_world = 
                tf2::transformToEigen(tf) *
                Eigen::Vector3d(centroid_sensor[0],
                                centroid_sensor[1],
                                centroid_sensor[2]);
            return c_world;
        }
        catch(const tf2::TransformException& ex)
        {
            RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
            return std::nullopt;
        }
    }

    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud,
                      const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg)
    {
        if(debug_){
            // Both messages are guaranteed to be a matched pair here
            double stamp_delta_ms = std::abs((rclcpp::Time(point_cloud->header.stamp) - rclcpp::Time(rgb_msg->header.stamp)).seconds() * 1000.0);

            // Should work properly because we are using use_sim_time="true"
            rclcpp::Time now = this->get_clock()->now();
            double point_latency = (now - rclcpp::Time(point_cloud->header.stamp)).seconds() * 1000.0;
            double rgb_latency   = (now - rclcpp::Time(rgb_msg->header.stamp)).seconds()   * 1000.0;

            RCLCPP_INFO(get_logger(),
                "Synced pair | stamp deltaT=%.2f ms | point latency=%.2f ms | rgb latency=%.2f ms",
                stamp_delta_ms, point_latency, rgb_latency);
        }

        (void)rgb_msg;

        pcl::fromROSMsg(*point_cloud, *cloud);

        // --- Voxel Filtering ---
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.005, 0.005, 0.005);
        voxel_filter.filter(*voxel_cloud);

        visualize(voxel_cloud, point_cloud->header, voxel_pub_);

        // --- Passthrough Filter along z axis ---
        pcl::PassThrough<PointT> passing_z;
        passing_z.setInputCloud(voxel_cloud);
        passing_z.setFilterFieldName("z");
        passing_z.setFilterLimits(0.0, 0.4);
        passing_z.filter(*pass_filter_z);

        visualize(pass_filter_z, point_cloud->header, passz_pub_);

        // --- Passthrough Filter along y axis ---
        pcl::PassThrough<PointT> passing_y;
        passing_y.setInputCloud(pass_filter_z);
        passing_y.setFilterFieldName("y");
        passing_y.setFilterLimits(-0.1, 0.2); // -1.5, 1.3
        passing_y.filter(*pass_filter_y);

        visualize(pass_filter_y, point_cloud->header, passy_pub_);

        // --- Passthrough Filter along x axis ---
        pcl::PassThrough<PointT> passing_x;
        passing_x.setInputCloud(pass_filter_y);
        passing_x.setFilterFieldName("x");
        passing_x.setFilterLimits(-0.35, 0.1);
        passing_x.filter(*pass_filter_x);

        visualize(pass_filter_x, point_cloud->header, passx_pub_);

        // --- Clustering components into individual component ---
        if (!cluster_ || pass_filter_x->empty()) return;

        // KD-Tree on the filtered cloud
        pcl::search::KdTree<PointT>::Ptr tree =
            std::make_shared<pcl::search::KdTree<PointT>>();

        tree->setInputCloud(pass_filter_x);

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<PointT> ec;

        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(cluster_min_size_);
        ec.setMaxClusterSize(cluster_max_size_);

        ec.setSearchMethod(tree);
        ec.setInputCloud(pass_filter_x);

        ec.extract(cluster_indices);

        // Merged coloured cloud
        auto merged = std::make_shared<pcl::PointCloud<PointT>>();

        // Marker array for centroids
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete stale markers
        {
            visualization_msgs::msg::Marker del;

            del.header.frame_id = point_cloud->header.frame_id;
            del.header.stamp    = point_cloud->header.stamp;

            del.action = visualization_msgs::msg::Marker::DELETEALL;

            marker_array.markers.push_back(del);
        }

        if (print_centroids_) {
            RCLCPP_INFO(
                get_logger(),
                "--- Clusters found: %zu ---",
                cluster_indices.size()
            );
        }

        for (std::size_t ci = 0; ci < cluster_indices.size(); ++ci)
        {
            const auto& indices = cluster_indices[ci];

            auto [r, g, b] = clusterColour(ci);

            // Extract cluster points
            auto cluster_cloud =
                std::make_shared<pcl::PointCloud<PointT>>();

            cluster_cloud->reserve(indices.indices.size());

            for (int idx : indices.indices)
            {
                PointT pt = (*pass_filter_x)[idx];

                pt.r = r;
                pt.g = g;
                pt.b = b;

                cluster_cloud->push_back(pt);
            }

            // Compute centroid
            Eigen::Vector4f centroid;

            pcl::compute3DCentroid(*cluster_cloud, centroid);

            auto world = toWorldFrame(centroid, point_cloud->header);

            // Print centroid
            if (world && print_centroids_) {
                RCLCPP_INFO(
                    get_logger(),
                    "  Cluster #%zu | pts=%zu | centroid (x=%.4f  y=%.4f  z=%.4f) m",
                    ci,
                    indices.indices.size(),
                    world->x(),
                    world->y(),
                    world->z()
                );
            }

            // Build centroid marker
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = point_cloud->header.frame_id;
            marker.header.stamp    = point_cloud->header.stamp;

            marker.ns = "centroids";
            marker.id = static_cast<int>(ci);

            marker.type   = visualization_msgs::msg::Marker::SPHERE;
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

            // Accumulate merged cloud
            *merged += *cluster_cloud;
        }

        // Publish merged coloured clusters
        visualize(
            merged,
            point_cloud->header,
            cluster_pub_
        );

        // Publish centroid markers
        if (centroid_pub_) {
            centroid_pub_->publish(marker_array);
        }

    }

private:

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;

    tf2_ros::Buffer           tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passz_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passy_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passx_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centroid_pub_;

    bool   debug_            = false;
    bool   voxel_            = false;
    bool   passz_            = false;
    bool   passy_            = false;
    bool   passx_            = false;
    bool   cluster_          = false;
    bool   print_centroids_  = false;
    double cluster_tolerance_= 0.02;
    int    cluster_min_size_ = 50;
    int    cluster_max_size_ = 25000;


    // Okay for single threaded, but later when we do multithreaded, then this becomes an issue
    // Better do it like this: Inside callback:
    pcl::PointCloud<PointT>::Ptr cloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::PointCloud<PointT>::Ptr voxel_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::PointCloud<PointT>::Ptr pass_filter_z = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::PointCloud<PointT>::Ptr pass_filter_y = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::PointCloud<PointT>::Ptr pass_filter_x = std::make_shared<pcl::PointCloud<PointT>>();
    // pcl::PointCloud<PointT>::Ptr plane_segmented_cloud = std::make_shared<pcl::PointCloud<PointT>>();

    pcl::VoxelGrid<PointT> voxel_filter;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FemtoSubscriber>());
    rclcpp::shutdown();

    return 0;
}