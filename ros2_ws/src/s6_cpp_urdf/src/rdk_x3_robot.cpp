#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono;

class StatePublisher : public rclcpp::Node{
    public:

    StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
        Node("state_publisher",options){
            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
            // create a publisher to tell robot_state_publisher the JointState information.
            // robot_state_publisher will deal with this transformation
            broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            // create a broadcaster to tell the tf2 state information
            // this broadcaster will determine the position of coordinate system 'asix' in coordinate system 'odom'
            RCLCPP_INFO(this->get_logger(),"Starting state publisher");

            loop_rate_=std::make_shared<rclcpp::Rate>(33ms);

            timer_=this->create_wall_timer(33.333ms,std::bind(&StatePublisher::publish,this));
        }

        void publish();
    private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::Rate::SharedPtr loop_rate_;
    rclcpp::TimerBase::SharedPtr timer_;

    //Robot state variables
    // degree means one degree
    const double degree=M_PI/180.0;
    double spin_angle = 0.0;
    double step = degree;
    double angle = 0.0;
    double wheel_angle = 0.0;
};

void StatePublisher::publish(){
    // Create the necessary messages
    geometry_msgs::msg::TransformStamped t;
    sensor_msgs::msg::JointState joint_state;

    // Add time stamp
    joint_state.header.stamp=this->get_clock()->now();
    // Specify joints' name which are defined in the URDF file and their content
    joint_state.name={"base_joint","left_front_joint","left_back_joint","right_front_joint","right_back_joint"};
    joint_state.position={spin_angle,wheel_angle,wheel_angle,-wheel_angle,-wheel_angle};

    // Add time stamp
    t.header.stamp=this->get_clock()->now();
    // specify the father and child frame

    // odom is the base coordinate system of tf2
    t.header.frame_id="odom";
    // base_footprint is defined in urdf file and it is the base coordinate of model
    t.child_frame_id="base_footprint";

    // Add translation change
    t.transform.translation.x=cos(angle)*1;
    t.transform.translation.y=sin(angle)*1;
    t.transform.translation.z=0.0;
    tf2::Quaternion q;
    // Euler angle into Quanternion and add rotation change
    q.setRPY(0,0,angle+M_PI/2);
    t.transform.rotation.x=q.x();
    t.transform.rotation.y=q.y();
    t.transform.rotation.z=q.z();
    t.transform.rotation.w=q.w();

    // Update state for next time
    spin_angle-=step;
    if (spin_angle<-M_PI || spin_angle>M_PI){
        spin_angle*=-1;
    }

    wheel_angle+=step;
    if (wheel_angle<-M_PI || wheel_angle>M_PI){
        wheel_angle*=-1;
    }

    angle+=degree;    // Change angle at a slower pace

    // send message
    broadcaster->sendTransform(t);
    joint_pub_->publish(joint_state);

    RCLCPP_INFO(this->get_logger(),"Publishing joint state");
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}