#include <cv_bridge/cv_bridge.h>

#include <oculus_interfaces/msg/ping.hpp>
#include <oculus_ros2/sonar_viewer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class SonarRawImageNode : public rclcpp::Node {
public:
    SonarRawImageNode()
        : Node("sonar_raw_image"),
          sonar_viewer_(this),
          frame_id_(this->declare_parameter<std::string>("frame_id", "sonar")),
          use_gain_compensation_(this->declare_parameter<bool>("use_gain_compensation", false)) {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("oculus/not_polar", 10);
        ping_sub_  = this->create_subscription<oculus_interfaces::msg::Ping>(
            "oculus/ping", 10, std::bind(&SonarRawImageNode::pingCallback, this, std::placeholders::_1));
    }

private:
    void pingCallback(const oculus_interfaces::msg::Ping::SharedPtr ros_ping_msg) {
        // Compute polar-to-cartesian remap tables
        sonar_viewer_.pingToImageConversion(*ros_ping_msg, num_bearings_, num_ranges_, map_bb_x_, map_bb_y_, img_cols_,
                                            img_rows_, map_img_x_, map_img_y_);

        // Extract intensity from ping data
        cv::Mat intensity;
        sonar_viewer_.pingToIntensity(*ros_ping_msg, intensity, use_gain_compensation_);

        // Remap bin-beam intensity to polar fan image
        cv::Mat img_raw(cv::Size(img_cols_, img_rows_), intensity.type());
        cv::remap(intensity, img_raw, map_img_x_, map_img_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                  cv::Scalar(0, 0, 0));

        // Publish not polar sonar image
        std_msgs::msg::Header header;
        header.frame_id = frame_id_;
        header.stamp    = ros_ping_msg->header.stamp;

        sensor_msgs::msg::Image img_msg;
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_raw).toImageMsg(img_msg);
        image_pub_->publish(img_msg);
    }

    SonarViewer sonar_viewer_;
    const std::string frame_id_;
    const bool use_gain_compensation_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr ping_sub_;

    // Cached remap tables
    cv::Mat map_bb_x_, map_bb_y_, map_img_x_, map_img_y_;
    int num_bearings_ = 0, num_ranges_ = 0, img_cols_ = 0, img_rows_ = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SonarRawImageNode>());
    rclcpp::shutdown();
    return 0;
}
