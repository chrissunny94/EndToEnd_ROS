#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <adaptive_clustering/msg/cluster_array.hpp>

class AdaptiveClusteringNode : public rclcpp::Node {
public:
    AdaptiveClusteringNode() : Node("adaptive_clustering") {
        // Parameters
        this->declare_parameter("sensor_model", "VLP-16");
        this->declare_parameter("print_fps", false);
        this->declare_parameter("leaf", 1);
        this->declare_parameter("z_axis_min", -0.8);
        this->declare_parameter("z_axis_max", 2.0);
        this->declare_parameter("cluster_size_min", 3);
        this->declare_parameter("cluster_size_max", 2200000);

        this->get_parameter("sensor_model", sensor_model_);
        this->get_parameter("print_fps", print_fps_);
        this->get_parameter("leaf", leaf_);
        this->get_parameter("z_axis_min", z_axis_min_);
        this->get_parameter("z_axis_max", z_axis_max_);
        this->get_parameter("cluster_size_min", cluster_size_min_);
        this->get_parameter("cluster_size_max", cluster_size_max_);

        // Subscribers
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "velodyne_points", 1, std::bind(&AdaptiveClusteringNode::pointCloudCallback, this, std::placeholders::_1));

        // Publishers
        cluster_array_pub_ = this->create_publisher<adaptive_clustering::msg::ClusterArray>("clusters", 10);
        cloud_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", 10);
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("poses", 10);
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

        // Initialize region sizes based on sensor model
        if (sensor_model_ == "VLP-16") {
            regions_ = {2, 3, 3, 3, 3, 3, 3, 2, 3, 3, 3, 3, 3, 3};
        } else if (sensor_model_ == "HDL-32E") {
            regions_ = {4, 5, 4, 5, 4, 5, 5, 4, 5, 4, 5, 5, 4, 5};
        } else if (sensor_model_ == "HDL-64E") {
            regions_ = {14, 14, 14, 15, 14};
        } else {
            RCLCPP_FATAL(this->get_logger(), "Unknown sensor model!");
        }
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ros_pc2_in) {
        if (print_fps_ && reset_) {
            frames_ = 0;
            start_time_ = this->now();
            reset_ = false;
        }

        // Convert ROS message to PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);

        // Downsampling + ground & ceiling removal
        pcl::IndicesPtr pc_indices(new std::vector<int>);
        for (int i = 0; i < pcl_pc_in->size(); ++i) {
            if (i % leaf_ == 0) {
                if (pcl_pc_in->points[i].z >= z_axis_min_ && pcl_pc_in->points[i].z <= z_axis_max_) {
                    pc_indices->push_back(i);
                }
            }
        }

        // Divide the point cloud into nested circular regions
        boost::array<std::vector<int>, region_max_> indices_array;
        for (int i = 0; i < pc_indices->size(); ++i) {
            float range = 0.0;
            for (int j = 0; j < region_max_; j++) {
                float d2 = pcl_pc_in->points[(*pc_indices)[i]].x * pcl_pc_in->points[(*pc_indices)[i]].x +
                            pcl_pc_in->points[(*pc_indices)[i]].y * pcl_pc_in->points[(*pc_indices)[i]].y +
                            pcl_pc_in->points[(*pc_indices)[i]].z * pcl_pc_in->points[(*pc_indices)[i]].z;
                if (d2 > range * range && d2 <= (range + regions_[j]) * (range + regions_[j])) {
                    indices_array[j].push_back((*pc_indices)[i]);
                    break;
                }
                range += regions_[j];
            }
        }

        // Euclidean clustering
        float tolerance = 0.0;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;

        for (int i = 0; i < region_max_; i++) {
            tolerance += 0.1;
            if (indices_array[i].size() > cluster_size_min_) {
                auto indices_array_ptr = std::make_shared<std::vector<int>>(indices_array[i]);
                pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
                tree->setInputCloud(pcl_pc_in, indices_array_ptr);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
                ec.setClusterTolerance(tolerance);
                ec.setMinClusterSize(cluster_size_min_);
                ec.setMaxClusterSize(cluster_size_max_);
                ec.setSearchMethod(tree);
                ec.setInputCloud(pcl_pc_in);
                ec.setIndices(indices_array_ptr);
                ec.extract(cluster_indices);

                for (const auto& cluster_idx : cluster_indices) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
                    for (const auto& idx : cluster_idx.indices) {
                        cluster->points.push_back(pcl_pc_in->points[idx]);
                    }
                    cluster->width = cluster->size();
                    cluster->height = 1;
                    cluster->is_dense = true;
                    clusters.push_back(cluster);
                }
            }
        }

        // Output
        if (cloud_filtered_pub_->get_subscription_count() > 0) {
            auto pcl_pc_out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            sensor_msgs::msg::PointCloud2 ros_pc2_out;
            pcl::copyPointCloud(*pcl_pc_in, *pc_indices, *pcl_pc_out);
            pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
            cloud_filtered_pub_->publish(ros_pc2_out);
        }

        auto cluster_array = adaptive_clustering::msg::ClusterArray();
        auto pose_array = geometry_msgs::msg::PoseArray();
        auto marker_array = visualization_msgs::msg::MarkerArray();

        for (int i = 0; i < clusters.size(); ++i) {
            if (cluster_array_pub_->get_subscription_count() > 0) {
                sensor_msgs::msg::PointCloud2 ros_pc2_out;
                pcl::toROSMsg(*clusters[i], ros_pc2_out);
                cluster_array.clusters.push_back(ros_pc2_out);
            }

            if (pose_array_pub_->get_subscription_count() > 0) {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*clusters[i], centroid);

                geometry_msgs::msg::Pose pose;
                pose.position.x = centroid[0];
                pose.position.y = centroid[1];
                pose.position.z = centroid[2];
                pose.orientation.w = 1;
                pose_array.poses.push_back(pose);
            }

            if (marker_array_pub_->get_subscription_count() > 0) {
                Eigen::Vector4f min, max;
                pcl::getMinMax3D(*clusters[i], min, max);

                visualization_msgs::msg::Marker marker;
                marker.header = ros_pc2_in->header;
                marker.ns = "adaptive_clustering";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::LINE_LIST;

                geometry_msgs::msg::Point p[24];
                p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
                p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
                p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
                p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
                p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
                p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
                p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
                p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
                p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
                p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
                p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
                p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
                p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
                p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
                p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
                p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
                p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
                p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
                p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
                p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
                p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
                p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
                p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
                p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
                for (int j = 0; j < 24; ++j) {
                    marker.points.push_back(p[j]);
                }

                marker.scale.x = 0.02;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.5;
                marker.lifetime = rclcpp::Duration(0.1).to_ros_duration();
                marker_array.markers.push_back(marker);
            }
        }

        if (cluster_array.clusters.size()) {
            cluster_array.header = ros_pc2_in->header;
            cluster_array_pub_->publish(cluster_array);
        }

        if (pose_array.poses.size()) {
            pose_array.header = ros_pc2_in->header;
            pose_array_pub_->publish(pose_array);
        }

        if (marker_array.markers.size()) {
            marker_array_pub_->publish(marker_array);
        }

        if (print_fps_ && ++frames_ > 10) {
            auto elapsed_time = (this->now() - start_time_).seconds();
            RCLCPP_INFO(this->get_logger(), "[adaptive_clustering] fps = %.2f, timestamp = %.2f", float(frames_) / elapsed_time, this->now().seconds());
            reset_ = true;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<adaptive_clustering::msg::ClusterArray>::SharedPtr cluster_array_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_filtered_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

    bool print_fps_;
    int leaf_;
    float z_axis_min_;
    float z_axis_max_;
    int cluster_size_min_;
    int cluster_size_max_;

    const int region_max_ = 10; // Adjust as needed
    std::vector<int> regions_;
    int frames_;
    rclcpp::Time start_time_;
    bool reset_ = true;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveClusteringNode>());
    rclcpp::shutdown();
    return 0;
}
