#include "rclcpp/rclcpp.hpp"
#include "optitrack_multiplexer_ros2_msgs/msg/rigid_body_stamped.hpp"
#include "optitrack_multiplexer_ros2/memory.h"  

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <thread>

#include <iostream>

class RigidBodyStampedListener : public rclcpp::Node
{
public:
    RigidBodyStampedListener()
        : Node("rigid_body_listener")
    {
        subscription_ = this->create_subscription<optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>(
            "/optitrack_multiplexer_node/rigid_body/drone", 10, std::bind(&RigidBodyStampedListener::topic_callback, this, std::placeholders::_1));

        this->robot_interface_.Initialize(false);

        std::cout << "Shared memory initialized!" << std::endl;
    }

private:
    rclcpp::Subscription<optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>::SharedPtr subscription_;


    void topic_callback(const optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped::SharedPtr msg)
    {

        Eigen::Vector<double, 7> mocap_measurement;
        mocap_measurement(0) = msg->rigid_body.pose.position.x;
        mocap_measurement(1) = msg->rigid_body.pose.position.y;
        mocap_measurement(2) = msg->rigid_body.pose.position.z;
        mocap_measurement(3) = msg->rigid_body.pose.orientation.q_x;
        mocap_measurement(4) = msg->rigid_body.pose.orientation.q_y;
        mocap_measurement(5) = msg->rigid_body.pose.orientation.q_z;
        mocap_measurement(6) = msg->rigid_body.pose.orientation.q_w;

        this->robot_interface_.SetMocapMeasurement(mocap_measurement);

        std::cout << "x: " << msg->rigid_body.pose.position.x << ", y: " << msg->rigid_body.pose.position.y << ", z: " << msg->rigid_body.pose.position.z << std::endl;
        std::cout << "qw: " << msg->rigid_body.pose.orientation.q_w << ", qx: " << msg->rigid_body.pose.orientation.q_x << ", qy: " << msg->rigid_body.pose.orientation.q_y << ", qz: " << msg->rigid_body.pose.orientation.q_z << std::endl;

    }


    SharedMemoryInterface robot_interface_;

};

int main(int argc, char **argv)
{

    // std::cout << std::fixed;
    // std::cout << std::setprecision(5);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RigidBodyStampedListener>());
    rclcpp::shutdown();
    return 0;
}
