#include <cmath>
#include <cstdint>
#include <oculus_interfaces/msg/ping.hpp>
#include <oculus_ros2/sonar_viewer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <vector>

class SonarPointCloudNode : public rclcpp::Node {
public:
    SonarPointCloudNode()
        : Node("sonar_pointcloud"),
          sonar_viewer_(this),
          frame_id_(this->declare_parameter<std::string>("frame_id", "auv4/sonar")),
          use_gain_compensation_(this->declare_parameter<bool>("use_gain_compensation", false)) {
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("oculus/pointcloud2", 10);
        ping_sub_       = this->create_subscription<oculus_interfaces::msg::Ping>(
            "oculus/ping", 10, std::bind(&SonarPointCloudNode::pingCallback, this, std::placeholders::_1));
    }

private:
    void pingCallback(const oculus_interfaces::msg::Ping::SharedPtr ros_ping_msg) {
        const int num_ranges   = ros_ping_msg->n_ranges;
        const int num_bearings = ros_ping_msg->n_beams;
        const double range_res = ros_ping_msg->range_resolution;

        // Reuse existing pipeline for image conversion and intensity extraction
        int bearings = 0, ranges = 0, cols = 0, rows = 0;
        cv::Mat map_bb_x, map_bb_y, map_img_x, map_img_y;
        sonar_viewer_.pingToImageConversion(*ros_ping_msg, bearings, ranges, map_bb_x, map_bb_y, cols, rows, map_img_x,
                                            map_img_y);

        cv::Mat intensity;
        sonar_viewer_.pingToIntensity(*ros_ping_msg, intensity, use_gain_compensation_);

        // Pre-compute bearing angles in radians
        std::vector<double> bearings_rad(num_bearings);
        for (int b = 0; b < num_bearings; b++) {
            bearings_rad[b] = ros_ping_msg->bearings[b] * 0.01 * M_PI / 180.0;
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp    = ros_ping_msg->header.stamp;
        cloud_msg.header.frame_id = frame_id_;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(
            4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
            sensor_msgs::msg::PointField::FLOAT32, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(static_cast<size_t>(num_ranges) * num_bearings);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");

        for (int r = 0; r < num_ranges; r++) {
            const double range = (r + 1) * range_res;
            for (int b = 0; b < num_bearings; b++) {
                const double bearing = bearings_rad[b];

                *iter_x = static_cast<float>(range * std::cos(bearing));
                *iter_y = static_cast<float>(-range * std::sin(bearing));
                *iter_z = 0.0f;
                *iter_i = static_cast<float>(intensity.at<uchar>(r, b)) / 255.0f;

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_i;
            }
        }

        pointcloud_pub_->publish(cloud_msg);
    }

    SonarViewer sonar_viewer_;
    const std::string frame_id_;
    const bool use_gain_compensation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr ping_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SonarPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
