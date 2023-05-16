#include "optitrack_wrapper_ros2_msgs/msg/analog_channel_data.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/device_data.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/force_plate_data.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/marker.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/marker_set_data.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/rigid_body_data.hpp"
#include "optitrack_wrapper_ros2_msgs/msg/skeleton_data.hpp"
#include "rclcpp/rclcpp.hpp"

#include <NatNetCAPI.h>
#include <NatNetTypes.h>

namespace optitrack_wrapper {

// get marker set data message
::optitrack_wrapper_ros2_msgs::msg::MarkerSetData
GetMarkerSetDataMessage(sMarkerSetData *p_ms,
                        ::rclcpp::Node::SharedPtr ros_node, bool verbose);

// get rigid body data message
::optitrack_wrapper_ros2_msgs::msg::RigidBodyData
GetRigidBodyDataMessage(sRigidBodyData *p_rb,
                        ::rclcpp::Node::SharedPtr ros_node, bool verbose);

// get skeleton data message
::optitrack_wrapper_ros2_msgs::msg::SkeletonData
GetSkeletonDataMessage(sSkeletonData *p_sk, ::rclcpp::Node::SharedPtr ros_node,
                       bool verbose);

// get labeled marker message
::optitrack_wrapper_ros2_msgs::msg::Marker
GetLabeledMarkerMessage(sMarker *p_marker, ::rclcpp::Node::SharedPtr ros_node,
                        bool verbose);

// get force plate data message
::optitrack_wrapper_ros2_msgs::msg::ForcePlateData
GetForcePlateDataMessage(sForcePlateData *p_fp,
                         int analog_samples_per_mocap_frame,
                         ::rclcpp::Node::SharedPtr ros_node, bool verbose);

// get device data message
::optitrack_wrapper_ros2_msgs::msg::DeviceData
GetDeviceDataMessage(sDeviceData *p_device, int analog_samples_per_mocap_frame,
                     ::rclcpp::Node::SharedPtr ros_node, bool verbose);

// get analog channel data message
::optitrack_wrapper_ros2_msgs::msg::AnalogChannelData
GetAnalogChannelDataMessage(sAnalogChannelData *p_acd,
                            int analog_samples_per_mocap_frame,
                            ::rclcpp::Node::SharedPtr ros_node, bool verbose);

} // namespace optitrack_wrapper
