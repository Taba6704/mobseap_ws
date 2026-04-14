#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <cv_bridge/cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>
#include <limits>
#include <cmath>
#include <string>

class TargetRangeNode : public rclcpp::Node
{
public:
    TargetRangeNode() : Node("target_range_node")
    {
        center_region_scale_ = this->declare_parameter<double>("center_region_scale", 0.3);
        min_valid_depth_m_ = this->declare_parameter<double>("min_valid_depth_m", 0.2);
        max_valid_depth_m_ = this->declare_parameter<double>("max_valid_depth_m", 20.0);
        publish_last_valid_when_lost_ = this->declare_parameter<bool>("publish_last_valid_when_lost", false);

        visible_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/target_visible",
            10,
            std::bind(&TargetRangeNode::visibleCallback, this, std::placeholders::_1));

        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/target_detection",
            10,
            std::bind(&TargetRangeNode::detectionCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/vision/depth_image",
            10,
            std::bind(&TargetRangeNode::depthCallback, this, std::placeholders::_1));

        range_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/target_range_m", 10);

        RCLCPP_INFO(this->get_logger(), "target_range_node started");
    }

private:
    void visibleCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        target_visible_ = msg->data;
    }

    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        latest_detection_ = *msg;
        have_detection_ = !msg->detections.empty();
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if(!target_visible_) {
            if(!publish_last_valid_when_lost_) {
                return;
            }
        }

        if(!have_detection_ || latest_detection_.detections.empty()) {
            return;
        }

        const auto& det = latest_detection_.detections.front();

        const double bbox_cx = det.bbox.center.position.x;
        const double bbox_cy = det.bbox.center.position.y;
        const double bbox_w = det.bbox.size_x;
        const double bbox_h = det.bbox.size_y;

        cv::Mat depth;
        bool is_16uc1_mm = false;
        bool is_32fc1_m = false;

        try {
            if(msg->encoding == "16UC1") {
                depth = cv_bridge::toCvCopy(msg, "16UC1")->image;
                is_16uc1_mm = true;
            } else if(msg->encoding == "32FC1") {
                depth = cv_bridge::toCvCopy(msg, "32FC1")->image;
                is_32fc1_m = true;
            } else {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Unsupported depth encoding: %s",
                    msg->encoding.c_str());
                return;
            }
        } catch(const std::exception& e) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "cv_bridge conversion failed: %s",
                e.what());
            return;
        }

        if(depth.empty()) {
            return;
        }

        const int img_w = depth.cols;
        const int img_h = depth.rows;

        const double roi_scale = std::max(0.05, std::min(center_region_scale_, 1.0));
        const double roi_w = bbox_w * roi_scale;
        const double roi_h = bbox_h * roi_scale;

        int x1 = static_cast<int>(std::round(bbox_cx - roi_w / 2.0));
        int y1 = static_cast<int>(std::round(bbox_cy - roi_h / 2.0));
        int x2 = static_cast<int>(std::round(bbox_cx + roi_w / 2.0));
        int y2 = static_cast<int>(std::round(bbox_cy + roi_h / 2.0));

        x1 = std::max(0, std::min(x1, img_w - 1));
        y1 = std::max(0, std::min(y1, img_h - 1));
        x2 = std::max(0, std::min(x2, img_w - 1));
        y2 = std::max(0, std::min(y2, img_h - 1));

        if(x2 <= x1 || y2 <= y1) {
            return;
        }

        std::vector<double> valid_depths_m;
        valid_depths_m.reserve(static_cast<std::size_t>((x2 - x1 + 1) * (y2 - y1 + 1)));

        for(int y = y1; y <= y2; ++y) {
            for(int x = x1; x <= x2; ++x) {
                double z_m = std::numeric_limits<double>::quiet_NaN();

                if(is_16uc1_mm) {
                    const uint16_t z_mm = depth.at<uint16_t>(y, x);
                    if(z_mm > 0) {
                        z_m = static_cast<double>(z_mm) / 1000.0;
                    }
                } else if(is_32fc1_m) {
                    const float z = depth.at<float>(y, x);
                    if(std::isfinite(z) && z > 0.0f) {
                        z_m = static_cast<double>(z);
                    }
                }

                if(std::isfinite(z_m) &&
                   z_m >= min_valid_depth_m_ &&
                   z_m <= max_valid_depth_m_) {
                    valid_depths_m.push_back(z_m);
                }
            }
        }

        if(valid_depths_m.empty()) {
            return;
        }

        std::sort(valid_depths_m.begin(), valid_depths_m.end());
        const std::size_t mid = valid_depths_m.size() / 2;
        double median_range_m = 0.0;

        if(valid_depths_m.size() % 2 == 0) {
            median_range_m = 0.5 * (valid_depths_m[mid - 1] + valid_depths_m[mid]);
        } else {
            median_range_m = valid_depths_m[mid];
        }

        last_valid_range_m_ = median_range_m;
        have_last_valid_range_ = true;

        std_msgs::msg::Float64 range_msg;
        range_msg.data = median_range_m;
        range_pub_->publish(range_msg);
    }

    double center_region_scale_;
    double min_valid_depth_m_;
    double max_valid_depth_m_;
    bool publish_last_valid_when_lost_;

    bool target_visible_{false};
    bool have_detection_{false};
    bool have_last_valid_range_{false};
    double last_valid_range_m_{0.0};

    vision_msgs::msg::Detection2DArray latest_detection_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visible_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr range_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetRangeNode>());
    rclcpp::shutdown();
    return 0;
}