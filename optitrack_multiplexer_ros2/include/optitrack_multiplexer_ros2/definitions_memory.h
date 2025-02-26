#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <Eigen/Dense>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

#define INFO_IMU 0        // Set 1 to info IMU states
#define INFO_MOTOR 0      // Set 1 to info motor states
#define HIGH_FREQ 1 // Set 1 to subscribe to low states with high frequencies (500Hz)

enum PRorAB { PR = 0, AB = 1 };

using std::placeholders::_1;

const int G1_NUM_MOTOR = 29;

namespace bip = boost::interprocess;

// Define Eigen Vectors with Fixed Length
using G1JointVec = Eigen::Vector<float, G1_NUM_MOTOR>;
using G1IMUVec  = Eigen::Vector<float, 13>;
using MocapMeasurement = Eigen::Vector<double, 7>;
using CameraMeasurement = Eigen::Vector<double, 13>;

/// @brief Number of bytes for measurements
/// @details (4 * 29 + 13) * 4 = 516 bytes
const int N_BYTES_MEASUREMENTS = 1000;

/// @brief Number of bytes for commands
/// @details (5 * 29) * 4 = 580 bytes
const int N_BYTES_COMMANDS = 1000;

/// @brief Number of bytes for mocap measurements
const int N_BYTES_MOCAP = 400;

/// @brief Number of bytes for camera measurements
const int N_BYTES_CAMERA = 500;

/// @brief Number of bytes for boolean
const int N_BYTES_BOOL = 400;

struct G1Measurements
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure Eigen memory alignment

    G1JointVec joint_pos;
    G1JointVec joint_vel;
    G1JointVec joint_acc;
    G1JointVec joint_tor;
    G1IMUVec imu;

    G1Measurements()
    {
        joint_pos.setZero();
        joint_vel.setZero();
        joint_acc.setZero();
        joint_tor.setZero();
        imu.setZero();
    }
};

struct G1Commands
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure Eigen memory alignment

    G1JointVec joint_pos_cmd;
    G1JointVec joint_vel_cmd;
    G1JointVec joint_tor_cmd;
    G1JointVec joint_kp_cmd;
    G1JointVec joint_kd_cmd;

    G1Commands()
    {
        joint_pos_cmd.setZero();
        joint_vel_cmd.setZero();
        joint_tor_cmd.setZero();
        joint_kp_cmd.setZero();
        joint_kd_cmd.setZero();
    }
};

struct LowerRobotInstructions
{
    bool lowering_active;
    double lowering_period;
    double final_height;

    LowerRobotInstructions()
    {
        lowering_active = false;
        lowering_period = 0.0;
        final_height = 0.0;
    }
};

// IMU indices
namespace ImuIDX
{
    enum eImuIDX
    {
        Q_X, Q_Y, Q_Z, Q_W,
        ACC_X, ACC_Y, ACC_Z, 
        GYRO_X, GYRO_Y, GYRO_Z,
        ROLL, PITCH, YAW,
        
    };
}

// Mocap indices
namespace MocapIDX
{
    enum eMocapIDX
    {
        POS_X = 0, POS_Y, POS_Z, 
        Q_X, Q_Y, Q_Z, Q_W
    };
}

// Camera indices
namespace CameraIDX
{
    enum eCameraIDX
    {
        POS_X = 0, POS_Y, POS_Z, 
        Q_X, Q_Y, Q_Z, Q_W,
        VEL_X, VEL_Y, VEL_Z,
        W_X, W_Y, W_Z
    };
}

enum G1JointIndex 
{
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
  LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,  // NOTE INVALID for g1 23dof
  RightWristYaw = 28     // NOTE INVALID for g1 23dof
};

#endif  // DEFINITIONS_HPP