#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <cv_bridge/cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

#include <thread>
#include <atomic>
#include <chrono>
#include <optional>
#include <memory>

static cv::Rect frameNorm(const cv::Mat& frame,
                          float xmin, float ymin,
                          float xmax, float ymax)
{
    int x1 = static_cast<int>(xmin * frame.cols);
    int y1 = static_cast<int>(ymin * frame.rows);
    int x2 = static_cast<int>(xmax * frame.cols);
    int y2 = static_cast<int>(ymax * frame.rows);
    return cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
}

class OakYoloNode : public rclcpp::Node
{
public:
    OakYoloNode() : Node("yolo_object_detection")
    {
        model_name_ = this->declare_parameter<std::string>("model_name", "yolov6-nano");

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/image_annotated", 10);

        det_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "/vision/detections", 10);

        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/vision/target_center_px", 10);

        setup_pipeline();

        running_.store(true);
        worker_ = std::thread(&OakYoloNode::loop, this);

        RCLCPP_INFO(this->get_logger(), "OAK YOLO node started with model: %s",
                    model_name_.c_str());
    }

    ~OakYoloNode() override
    {
        running_.store(false);
        if(worker_.joinable())
            worker_.join();
    }

private:
    void setup_pipeline()
    {
        pipeline_ = std::make_unique<dai::Pipeline>();

        auto camera = pipeline_->create<dai::node::Camera>();
        camera->build();

        auto det = pipeline_->create<dai::node::DetectionNetwork>();

        model_path_ = this->declare_parameter<std::string>(
            "model_path",
            "/home/mobseap/models/YOLOv6_Nano-R2_COCO_512x288.rvc4.tar.xz"
        );

        dai::NNArchive archive(model_path_);
        det->build(camera, archive);

        labels_ = det->getClasses();

        q_rgb_ = det->passthrough.createOutputQueue();
        q_det_ = det->out.createOutputQueue();

        pipeline_->start();
    }

    void loop()
    {
        auto last = std::chrono::steady_clock::now();
        int frames = 0;

        while(rclcpp::ok() && running_.load())
        {
            // MessageQueue API (robust across DepthAI versions)
            auto rgb_msg = q_rgb_->get();
            auto det_msg_any = q_det_->get();

            auto in_rgb = std::dynamic_pointer_cast<dai::ImgFrame>(rgb_msg);
            auto in_det = std::dynamic_pointer_cast<dai::ImgDetections>(det_msg_any);

            if(!in_rgb) continue;

            cv::Mat frame = in_rgb->getCvFrame();

            vision_msgs::msg::Detection2DArray det_array;
            det_array.header.stamp = this->now();
            det_array.header.frame_id = "oak_rgb";

            bool found_person = false;
	    dai::ImgDetection best_det{};
            std::string best_label;
            float best_conf = -1.0f;

            if(in_det) {
              for(const auto& d : in_det->detections) {

                // Determine label string
                std::string label = std::to_string(d.label);
                if(labels_.has_value() && static_cast<std::size_t>(d.label) < labels_->size()) {
                  label = labels_.value()[d.label];
                }

                // Keep only "person"
                if(label != "person") continue;

                float conf = d.confidence;
                if(conf > best_conf) {
                  best_conf = conf;
                  best_det = d;
                  best_label = label;
                  found_person = true;
                }
              }
            }

            if(found_person) {
              auto box = frameNorm(frame, best_det.xmin, best_det.ymin, best_det.xmax, best_det.ymax);

              // Draw only best person
              cv::rectangle(frame, box, cv::Scalar(255,0,0), 2);
              cv::putText(frame, best_label,
                          cv::Point(box.x + 5, box.y + 20),
                          cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cv::Scalar(255,255,255), 2);
              cv::putText(frame, std::to_string(int(best_conf * 100)) + "%",
                          cv::Point(box.x + 5, box.y + 40),
                          cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cv::Scalar(255,255,255), 2);

              // Publish detections array containing ONLY this detection (optional)
              det_array.detections.clear();
              vision_msgs::msg::Detection2D det_out;
              det_out.bbox.center.position.x = box.x + box.width / 2.0;
              det_out.bbox.center.position.y = box.y + box.height / 2.0;
              det_out.bbox.center.theta = 0.0;
              det_out.bbox.size_x = static_cast<double>(box.width);
              det_out.bbox.size_y = static_cast<double>(box.height);

              vision_msgs::msg::ObjectHypothesisWithPose hyp;
              hyp.hypothesis.class_id = best_label;
              hyp.hypothesis.score = best_conf;
              det_out.results.push_back(hyp);
              det_array.detections.push_back(det_out);

              // Publish center pixel coordinates
              geometry_msgs::msg::PointStamped p;
              p.header.stamp = this->now();
              p.header.frame_id = "oak_rgb";
              p.point.x = det_out.bbox.center.position.x;
              p.point.y = det_out.bbox.center.position.y;
              p.point.z = 0.0;
              target_pub_->publish(p);
            } else {
              // No person found: you can choose to publish nothing or publish a sentinel
              det_array.detections.clear();
            }

            // FPS overlay
            frames++;
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last).count();
            if(dt > 1.0)
            {
                double fps = frames / dt;
                frames = 0;
                last = now;

                cv::putText(frame, "FPS: " + std::to_string(fps),
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                            0.8, cv::Scalar(255, 255, 255), 2);
            }

            auto img_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            img_msg->header.stamp = this->now();
            img_msg->header.frame_id = "oak_rgb";

            image_pub_->publish(*img_msg);
            det_pub_->publish(det_array);
        }
    }

    std::string model_name_;
    std::string model_path_;
    std::optional<std::vector<std::string>> labels_;

    std::unique_ptr<dai::Pipeline> pipeline_;
    std::shared_ptr<dai::MessageQueue> q_rgb_;
    std::shared_ptr<dai::MessageQueue> q_det_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;

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
