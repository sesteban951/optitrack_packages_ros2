// #include "src/definitions_memory.h"

#include "optitrack_multiplexer_ros2/definitions_memory.h"  // Use package-style include

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <thread>

#include <iostream>

//using std::placeholders::_1;

using namespace boost::interprocess;

class SharedMemoryInterface
{
    public: SharedMemoryInterface();
    public: ~SharedMemoryInterface();

    public: void Initialize(bool create_memory);

    private: managed_shared_memory* segment_joint_commands_;
    private: managed_shared_memory* segment_g1_measurements_;
    private: managed_shared_memory* segment_mocap_;
    private: managed_shared_memory* segment_camera_;
    private: managed_shared_memory* segment_lower_robot_;
    private: managed_shared_memory* segment_release_robot_;

    // Named mutexes
    private: named_mutex* mutex_joint_commands_;
    private: named_mutex* mutex_g1_measurements_;
    private: named_mutex* mutex_mocap_;
    private: named_mutex* mutex_camera_;
    private: named_mutex* mutex_lower_robot_;
    private: named_mutex* mutex_release_robot_;

    // Allocated objects in shared memory
    private: G1Commands* joint_commands_;
    private: G1Measurements* g1_measurements_;
    private: MocapMeasurement* mocap_ptr_;
    private: CameraMeasurement* camera_ptr_;
    private: LowerRobotInstructions* lower_robot_ptr_;
    private: bool* release_robot_ptr_;

    // Get shared memory data
    public: void GetG1Commands(G1JointVec &joint_pos_cmd,
                               G1JointVec &joint_vel_cmd,
                               G1JointVec &joint_tor_cmd,
                               G1JointVec &kp_gains,
                               G1JointVec &kd_gains);

    public: void GetG1Measurements(G1JointVec &joint_pos,
                                    G1JointVec &joint_vel,
                                   G1JointVec &joint_acc,
                                   G1JointVec &joint_tor,
                                   G1IMUVec &imu);

    public: void GetMocapMeasurement(MocapMeasurement &mocap_measurement);

    public: void GetCameraMeasurement(CameraMeasurement &camera_measurement);

    public: void GetLowerRobotInstructions(LowerRobotInstructions &lower_robot_instructions);

    public: void GetReleaseRobotFlag(bool &release_robot);

    // Set shared memory data
    public: void SetG1Commands(G1JointVec &joint_pos_cmd,
                               G1JointVec &joint_vel_cmd,
                               G1JointVec &joint_tor_cmd,
                               G1JointVec &kp_gains,
                               G1JointVec &kd_gains);

    public: void SetG1Measurements(G1JointVec &joint_pos,
                                   G1JointVec &joint_vel,
                                   G1JointVec &joint_acc,
                                   G1JointVec &joint_tor,
                                   G1IMUVec &imu);

    public: void SetMocapMeasurement(MocapMeasurement &mocap_measurement);

    public: void SetCameraMeasurement(CameraMeasurement &camera_measurement);

    public: void SetLowerRobotInstructions(LowerRobotInstructions &lower_robot_instructions);

    public: void SetReleaseRobotFlag(bool &release_robot);
};