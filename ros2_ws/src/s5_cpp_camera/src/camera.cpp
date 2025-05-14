#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

void find_devices(){
    for (int i = 0; i < 5; ++i){ // Check up to 5 devices
        cv::VideoCapture cap_test(i);
        if (cap_test.isOpened()) {
            std::cout << "Device " << i << " is available." << std::endl;
        }
        // Release the VideoCapture object
        cap_test.release();
    }
}

class CameraNode : public rclcpp::Node {
    public:
        CameraNode(): 
            Node("cpp_camera_node"), frame_count_(0), fps_(0.0) {
                RCLCPP_INFO(this->get_logger(), "C++ Camera node has been started");

                // Check available camera IDs
                // find_devices();

                // Declare parameters with default values
                this->declare_parameter<int>("camera.deviceID", 0);
                this->declare_parameter<int>("camera.width", 1280);
                this->declare_parameter<int>("camera.height", 720);

                // Get parameters
                this->get_parameter("camera.deviceID", deviceID_);
                this->get_parameter("camera.width", width_);
                this->get_parameter("camera.height", height_);

                // Open the camera
                cap_.open(deviceID_);
                if (!cap_.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
                    rclcpp::shutdown();
                }

                // Set camera properties
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
                std::cout << "Set Frame Width: " << width_ << std::endl;
                std::cout << "Set Frame Height: " << height_ << std::endl;

                cv::Mat frame;
                cap_ >> frame;

                // Current cam roperties
                std::cout << "Current camera properties:" << std::endl;
                std::cout << "Device ID: " << deviceID_ << std::endl;
                std::cout << "Frame Width: " << frame.cols << std::endl;
                std::cout << "Frame Height: " << frame.rows << std::endl;

                // Initialize start time
                start_time_ = std::chrono::steady_clock::now();
            }
            ~CameraNode() {
                if (cap_.isOpened()) {
                    cap_.release();
                    RCLCPP_INFO(this->get_logger(), "Camera resource released.");
                }
            }

        void init() {
            // Initialize ImageTransport and create a publisher
            image_transport::ImageTransport it(shared_from_this());
            pub_ = it.advertise("camera/image_raw", 1);

            // Start the timer for capturing and publishing images
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(16),
                std::bind(&CameraNode::CaptureAndPublish, this));
        }

    private:
        void CaptureAndPublish() {
            cv::Mat frame;
            cap_ >> frame;
            if (!frame.empty()) {
                // Increment frame count
                frame_count_++;

                // Calculate FPS every second
                std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed = now - start_time_;
                if (elapsed.count() >= 1.0) {
                    fps_ = frame_count_ / elapsed.count();
                    frame_count_ = 0;
                    start_time_ = now;
                }

                // Convert FPS to string
                std::ostringstream fps_text;
                fps_text << "FPS: " << std::fixed << std::setprecision(2) << fps_;

                // Put FPS text on the frame
                cv::putText(frame, fps_text.str(), cv::Point(10, 30),cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

                // Convert OpenCV image to ROS message
                std_msgs::msg::Header header;
                header.stamp = this->now();
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

                // Publish the image
                pub_.publish(msg);
            }
        }

    int deviceID_;
    int width_;
    int height_;
    cv::VideoCapture cap_;
    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // FPS calculation variables
    std::chrono::steady_clock::time_point start_time_;
    int frame_count_;
    double fps_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<CameraNode> node = std::make_shared<CameraNode>();
    node->init();  // Call the initialization function after construction
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
