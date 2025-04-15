#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber()
        : Node("cpp_camera_user_node") {
        RCLCPP_INFO(this->get_logger(), "C++ Camera User node has been started");
    }
    ~ImageSubscriber() {
        cv::destroyAllWindows();
        RCLCPP_INFO(this->get_logger(), "All OpenCV windows destroyed.");
    }

    void init() {
        // Initialize ImageTransport and create a subscriber using image_transport
        image_transport::ImageTransport it(shared_from_this());
        subscription_ = it.subscribe("camera/image_raw", 1, std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        try {
            // Convert ROS Image message to OpenCV image
            cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            // Display the image
            cv::imshow("C++ Camera User", cv_image);
            cv::waitKey(1);
            // ImageSubscriber::main_process(cv_image);
        }
        catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void main_process(const cv::Mat image) {
        // Resize the image to half its original size
        cv::Mat resized_image;
        cv::resize(image, resized_image, cv::Size(), 0.5, 0.5);

        // Convert the resized image to grayscale
        cv::Mat gray_image;
        cv::cvtColor(resized_image, gray_image, cv::COLOR_BGR2GRAY);

        // Apply standard histogram equalization
        cv::Mat equalized_image;
        cv::equalizeHist(gray_image, equalized_image);

        // Apply CLAHE
        cv::Mat clahe_image;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(0.5, cv::Size(8, 8));
        clahe->apply(gray_image, clahe_image);

        // Concatenate images horizontally
        cv::Mat concatenated_image;
        std::vector<cv::Mat> images = {gray_image, equalized_image, clahe_image};
        cv::hconcat(images, concatenated_image);

        // Display the concatenated images
        cv::imshow("Grayscale - HE - CLAHE (CPP)", concatenated_image);
        cv::waitKey(1);
    }

    image_transport::Subscriber subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<ImageSubscriber> node = std::make_shared<ImageSubscriber>();
    node->init();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
