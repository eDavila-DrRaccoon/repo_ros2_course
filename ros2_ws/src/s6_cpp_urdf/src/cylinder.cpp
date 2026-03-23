#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // tf2_geometry_msgs.h for Foxy, tf2_geometry_msgs.hpp for Humble
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono;

class StatePublisher : public rclcpp::Node{
    public:
        StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
            Node("cpp_state_publisher_node",options){
                RCLCPP_INFO(this->get_logger(),"C++ state publisher node has been started");
                
                // Create a publisher to tell robot_state_publisher the JointState information.
                // robot_state_publisher will deal with this transformation
                joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);

                // Create a broadcaster to publish the transform between coordinate frames that
                // will determine the position of coordinate system 'base_footprint' in coordinate system 'odom'
                broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

                // Timer for publishing at ~30Hz
                timer_=this->create_wall_timer(33ms,std::bind(&StatePublisher::publish,this));
            }

        void publish();

    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
        rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    // degree means one degree
    const double degree = M_PI/180.0;
    double spin_angle = 0.0;
    double step = degree;
    double angle = 0.0;
};

void StatePublisher::publish(){
    // Create JointState message
    sensor_msgs::msg::JointState joint_state;
    // Add time stamp
    joint_state.header.stamp=this->get_clock()->now();
    // Specify joints' name which are defined in the URDF file and their content
    joint_state.name={"base_joint"};
    joint_state.position={spin_angle};

    // Create TransformStamped message
    geometry_msgs::msg::TransformStamped t;
    // Add time stamp
    t.header.stamp=this->get_clock()->now();
    // Specify the father and child frames
    // odom is the base coordinate system of tf2
    t.header.frame_id="odom";
    // base_footprint is defined in URDF file and it is the base coordinate of model
    t.child_frame_id="cylinder_base_footprint";

    // Add translation change
    t.transform.translation.x = cos(angle) * 1.0;
    t.transform.translation.y = sin(angle) * 1.0;
    t.transform.translation.z = 0.0;

    // Euler angle into Quaternion and add rotation change
    tf2::Quaternion q;
    q.setRPY(0,0,angle + M_PI / 2);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Update state for next cycle
    spin_angle -= step;
    if (spin_angle<-M_PI || spin_angle>M_PI){
        spin_angle *= -1;
    }

    angle += degree; // Change the angle at a slow pace

    // Publish messages
    joint_pub_->publish(joint_state);
    broadcaster->sendTransform(t);

    RCLCPP_INFO(this->get_logger(),"Publishing joint state and transform");
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::shared_ptr<StatePublisher> node = std::make_shared<StatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}