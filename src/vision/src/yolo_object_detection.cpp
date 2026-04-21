#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <cv_bridge/cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

#include <thread>
#include <atomic>
#include <chrono>
#include <optional>
#include <memory>
#include <string>
#include <vector>
#include <limits>

static cv::Rect frameNorm(
    const cv::Mat& frame,
    float xmin,
    float ymin,
    float xmax,
    float ymax)
{
    int x1 = static_cast<int>(xmin * frame.cols);
    int y1 = static_cast<int>(ymin * frame.rows);
    int x2 = static_cast<int>(xmax * frame.cols);
    int y2 = static_cast<int>(ymax * frame.rows);

    x1 = std::max(0, std::min(x1, frame.cols - 1));
    y1 = std::max(0, std::min(y1, frame.rows - 1));
    x2 = std::max(0, std::min(x2, frame.cols - 1));
    y2 = std::max(0, std::min(y2, frame.rows - 1));

    return cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
}

class OakYoloNode : public rclcpp::Node
{
public:
    OakYoloNode() : Node("yolo_object_detection")
    {
        model_path_ = this->declare_parameter<std::string>(
            "model_path",
            "/home/mobseap/models/YOLOv6_Nano-R2_COCO_512x288.rvc4.tar.xz");

        sports_ball_conf_min_ = this->declare_parameter<double>(
            "sports_ball_conf_min", 0.30);

        person_conf_min_ = this->declare_parameter<double>(
            "person_conf_min", 0.20);

        prefer_sports_ball_ = this->declare_parameter<bool>(
            "prefer_sports_ball", true);

        show_only_selected_target_ = this->declare_parameter<bool>(
            "show_only_selected_target", true);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/image_annotated", 10);

        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/depth_image", 10);
        
        target_visible_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/target_visible", 10);

        target_pixel_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/target_pixel_error", 10);

        target_center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/target_bbox_center", 10);

        target_detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "/target_detection", 10);

        setup_pipeline();

        running_.store(true);
        worker_ = std::thread(&OakYoloNode::loop, this);

        RCLCPP_INFO(
            this->get_logger(),
            "yolo_object_detection started. model_path=%s",
            model_path_.c_str());
    }

    ~OakYoloNode() override
    {
        running_.store(false);
        if(worker_.joinable()) {
            worker_.join();
        }
    }

private:
    struct CandidateDetection
    {
        bool valid{false};
        dai::ImgDetection det{};
        std::string label;
        float confidence{0.0f};
    };

    void setup_pipeline()
    {
        pipeline_ = std::make_unique<dai::Pipeline>();

        // RGB camera for detection
        auto camera = pipeline_->create<dai::node::Camera>();
        camera->build();

        // Detection network
        auto det = pipeline_->create<dai::node::DetectionNetwork>();
        dai::NNArchive archive(model_path_);
        det->build(camera, archive);

        labels_ = det->getClasses();

        // Stereo depth node
        auto stereo = pipeline_->create<dai::node::StereoDepth>();
        stereo->build(
            true,  // auto-create stereo cameras
            dai::node::StereoDepth::PresetMode::DEFAULT
        );

        // Create output queues
        q_rgb_ = det->passthrough.createOutputQueue();
        q_det_ = det->out.createOutputQueue();
        q_depth_ = stereo->depth.createOutputQueue();

        pipeline_->start();
    }

    std::string getLabelString(uint32_t label_id) const
    {
        if(labels_.has_value() && static_cast<std::size_t>(label_id) < labels_->size()) {
            return labels_.value()[label_id];
        }
        return std::to_string(label_id);
    }

    bool isTargetClass(const std::string& label) const
    {
        return (label == "sports ball" || label == "person");
    }

    bool passesConfidenceThreshold(const std::string& label, float confidence) const
    {
        if(label == "sports ball") {
            return confidence >= static_cast<float>(sports_ball_conf_min_);
        }
        if(label == "person") {
            return confidence >= static_cast<float>(person_conf_min_);
        }
        return false;
    }

    void publishNoTarget()
    {
        std_msgs::msg::Bool visible_msg;
        visible_msg.data = false;
        target_visible_pub_->publish(visible_msg);

        std_msgs::msg::Float64 pixel_error_msg;
        pixel_error_msg.data = 0.0;
        target_pixel_error_pub_->publish(pixel_error_msg);

        vision_msgs::msg::Detection2DArray empty_detection_msg;
        empty_detection_msg.header.stamp = this->now();
        empty_detection_msg.header.frame_id = "oak_rgb";
        target_detection_pub_->publish(empty_detection_msg);
    }

    vision_msgs::msg::Detection2D buildDetectionMsg(
        const cv::Rect& box,
        const std::string& label,
        float confidence) const
    {
        vision_msgs::msg::Detection2D det_out;
        det_out.bbox.center.position.x = box.x + box.width / 2.0;
        det_out.bbox.center.position.y = box.y + box.height / 2.0;
        det_out.bbox.center.theta = 0.0;
        det_out.bbox.size_x = static_cast<double>(box.width);
        det_out.bbox.size_y = static_cast<double>(box.height);

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = label;
        hyp.hypothesis.score = confidence;
        det_out.results.push_back(hyp);

        return det_out;
    }

    void loop()
    {
        auto last_fps_time = std::chrono::steady_clock::now();
        int frame_count = 0;

        while(rclcpp::ok() && running_.load())
        {
            auto rgb_msg = q_rgb_->get();
            auto det_msg_any = q_det_->get();
            auto depth_msg_any = q_depth_->tryGet();

            auto in_rgb = std::dynamic_pointer_cast<dai::ImgFrame>(rgb_msg);
            auto in_det = std::dynamic_pointer_cast<dai::ImgDetections>(det_msg_any);
            auto in_depth = std::dynamic_pointer_cast<dai::ImgFrame>(depth_msg_any);

            if(!in_rgb) {
                continue;
            }

            cv::Mat frame = in_rgb->getCvFrame();

            CandidateDetection best_sports_ball;
            CandidateDetection best_person;

            if(in_det) {
                for(const auto& d : in_det->detections) {
                    const std::string label = getLabelString(d.label);
                    const float confidence = d.confidence;

                    if(!isTargetClass(label)) {
                        continue;
                    }

                    if(!passesConfidenceThreshold(label, confidence)) {
                        continue;
                    }

                    if(label == "sports ball") {
                        if(!best_sports_ball.valid || confidence > best_sports_ball.confidence) {
                            best_sports_ball.valid = true;
                            best_sports_ball.det = d;
                            best_sports_ball.label = label;
                            best_sports_ball.confidence = confidence;
                        }
                    } else if(label == "person") {
                        if(!best_person.valid || confidence > best_person.confidence) {
                            best_person.valid = true;
                            best_person.det = d;
                            best_person.label = label;
                            best_person.confidence = confidence;
                        }
                    }
                }
            }

            CandidateDetection selected;
            if(prefer_sports_ball_) {
                if(best_sports_ball.valid) {
                    selected = best_sports_ball;
                } else if(best_person.valid) {
                    selected = best_person;
                }
            } else {
                if(best_person.valid && best_sports_ball.valid) {
                    selected = (best_person.confidence >= best_sports_ball.confidence)
                        ? best_person
                        : best_sports_ball;
                } else if(best_person.valid) {
                    selected = best_person;
                } else if(best_sports_ball.valid) {
                    selected = best_sports_ball;
                }
            }

            vision_msgs::msg::Detection2DArray target_detection_array;
            target_detection_array.header.stamp = this->now();
            target_detection_array.header.frame_id = "oak_rgb";

            if(selected.valid) {
                const cv::Rect box = frameNorm(
                    frame,
                    selected.det.xmin,
                    selected.det.ymin,
                    selected.det.xmax,
                    selected.det.ymax);

                const vision_msgs::msg::Detection2D det_out =
                    buildDetectionMsg(box, selected.label, selected.confidence);

                target_detection_array.detections.push_back(det_out);

                const double image_center_x = static_cast<double>(frame.cols) / 2.0;
                const double bbox_center_x = det_out.bbox.center.position.x;
                const double pixel_error =
                    (bbox_center_x - image_center_x) / (static_cast<double>(frame.cols) / 2.0);

                std_msgs::msg::Bool visible_msg;
                visible_msg.data = true;
                target_visible_pub_->publish(visible_msg);

                std_msgs::msg::Float64 pixel_error_msg;
                pixel_error_msg.data = pixel_error;
                target_pixel_error_pub_->publish(pixel_error_msg);

                geometry_msgs::msg::PointStamped center_msg;
                center_msg.header.stamp = this->now();
                center_msg.header.frame_id = "oak_rgb";
                center_msg.point.x = det_out.bbox.center.position.x;
                center_msg.point.y = det_out.bbox.center.position.y;
                center_msg.point.z = 0.0;
                target_center_pub_->publish(center_msg);

                target_detection_pub_->publish(target_detection_array);

                if(show_only_selected_target_) {
                    cv::rectangle(frame, box, cv::Scalar(255, 0, 0), 2);

                    cv::putText(
                        frame,
                        selected.label,
                        cv::Point(box.x + 5, box.y + 20),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(255, 255, 255),
                        2);

                    cv::putText(
                        frame,
                        std::to_string(static_cast<int>(selected.confidence * 100.0f)) + "%",
                        cv::Point(box.x + 5, box.y + 40),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(255, 255, 255),
                        2);

                    const std::string err_text = "err_x: " + std::to_string(pixel_error);
                    cv::putText(
                        frame,
                        err_text,
                        cv::Point(10, frame.rows - 20),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(0, 255, 0),
                        2);
                }
            } else {
                publishNoTarget();

                cv::putText(
                    frame,
                    "No valid target",
                    cv::Point(10, frame.rows - 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(0, 0, 255),
                    2);
            }

            frame_count++;
            auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - last_fps_time).count();
            if(dt > 1.0) {
                const double fps = static_cast<double>(frame_count) / dt;
                frame_count = 0;
                last_fps_time = now;

                cv::putText(
                    frame,
                    "FPS: " + std::to_string(fps),
                    cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(255, 255, 255),
                    2);
            }
            
            if(in_depth) {
                cv::Mat depth_frame = in_depth->getFrame();

                auto depth_img_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "16UC1",
                depth_frame).toImageMsg();

                depth_img_msg->header.stamp = this->now();
                depth_img_msg->header.frame_id = "oak_depth";
                depth_pub_->publish(*depth_img_msg);
            }

            auto img_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "bgr8",
                frame).toImageMsg();

            img_msg->header.stamp = this->now();
            img_msg->header.frame_id = "oak_rgb";
            image_pub_->publish(*img_msg);
        }
    }

    std::string model_path_;
    double sports_ball_conf_min_;
    double person_conf_min_;
    bool prefer_sports_ball_;
    bool show_only_selected_target_;

    std::optional<std::vector<std::string>> labels_;

    std::unique_ptr<dai::Pipeline> pipeline_;
    std::shared_ptr<dai::MessageQueue> q_rgb_;
    std::shared_ptr<dai::MessageQueue> q_det_;
    std::shared_ptr<dai::MessageQueue> q_depth_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_visible_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_pixel_error_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_center_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr target_detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

    std::atomic<bool> running_{false};
    std::thread worker_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OakYoloNode>());
    rclcpp::shutdown();
    return 0;
}