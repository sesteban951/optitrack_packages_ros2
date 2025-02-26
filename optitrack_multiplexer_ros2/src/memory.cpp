#include "memory.h"

SharedMemoryInterface::SharedMemoryInterface(){}

SharedMemoryInterface::~SharedMemoryInterface(){}

void SharedMemoryInterface::Initialize(bool create_memory)
{
    if(create_memory == true)
    {
        // Remove existing shared memory
        std::cout << "Removing existing shared memory..." << std::endl;
        shared_memory_object::remove("SharedMemoryCommands");
        shared_memory_object::remove("SharedMemoryMeasurements");
        shared_memory_object::remove("SharedMemoryMocap");
        shared_memory_object::remove("SharedMemoryCamera");
        shared_memory_object::remove("SharedMemoryLowerRobot");
        shared_memory_object::remove("SharedMemoryReleaseRobot");

        // Remove existing named mutex
        std::cout << "Removing existing named mutex..." << std::endl;
        named_mutex::remove("SharedMutexCommands");
        named_mutex::remove("SharedMutexMeasurements");
        named_mutex::remove("SharedMutexMocap");
        named_mutex::remove("SharedMutexCamera");
        named_mutex::remove("SharedMutexLowerRobot");
        named_mutex::remove("SharedMutexReleaseRobot");

        // Create shared memory
        std::cout << "Creating shared memory..." << std::endl;
        this->segment_joint_commands_ = new managed_shared_memory(create_only, "SharedMemoryCommands", N_BYTES_COMMANDS);
        this->segment_g1_measurements_ = new managed_shared_memory(create_only, "SharedMemoryMeasurements", N_BYTES_MEASUREMENTS);
        this->segment_mocap_ = new managed_shared_memory(create_only, "SharedMemoryMocap", N_BYTES_MOCAP);
        this->segment_camera_ = new managed_shared_memory(create_only, "SharedMemoryCamera", N_BYTES_CAMERA);
        this->segment_lower_robot_ = new managed_shared_memory(create_only, "SharedMemoryLowerRobot", N_BYTES_BOOL);
        this->segment_release_robot_ = new managed_shared_memory(create_only, "SharedMemoryReleaseRobot", N_BYTES_BOOL);

        // Create named mutex
        std::cout << "Creating named mutex..." << std::endl;
        this->mutex_joint_commands_ = new named_mutex(create_only, "SharedMutexCommands");
        this->mutex_g1_measurements_ = new named_mutex(create_only, "SharedMutexMeasurements");
        this->mutex_mocap_ = new named_mutex(create_only, "SharedMutexMocap");
        this->mutex_camera_ = new named_mutex(create_only, "SharedMutexCamera");
        this->mutex_lower_robot_ = new named_mutex(create_only, "SharedMutexLowerRobot");
        this->mutex_release_robot_ = new named_mutex(create_only, "SharedMutexReleaseRobot");

        // Access the allocated objects in shared memory
        std::cout << "Accessing allocated objects in shared memory..." << std::endl;
        this->joint_commands_ = segment_joint_commands_->construct<G1Commands>("G1Commands")();
        this->g1_measurements_ = segment_g1_measurements_->construct<G1Measurements>("G1Measurements")();
        this->mocap_ptr_ = segment_mocap_->construct<MocapMeasurement>("MocapMeasurement")();
        this->camera_ptr_ = segment_camera_->construct<CameraMeasurement>("CameraMeasurement")();
        this->lower_robot_ptr_ = segment_lower_robot_->construct<LowerRobotInstructions>("LowerRobot")();
        this->release_robot_ptr_ = segment_release_robot_->construct<bool>("ReleaseRobot")();

    }
    else
    {
        // Open existing shared memory
        std::cout << "Opening existing shared memory..." << std::endl;
        this->segment_joint_commands_ = new managed_shared_memory(open_only, "SharedMemoryCommands");
        this->segment_g1_measurements_ = new managed_shared_memory(open_only, "SharedMemoryMeasurements");
        this->segment_mocap_ = new managed_shared_memory(open_only, "SharedMemoryMocap");
        this->segment_camera_ = new managed_shared_memory(open_only, "SharedMemoryCamera");
        this->segment_lower_robot_ = new managed_shared_memory(open_only, "SharedMemoryLowerRobot");
        this->segment_release_robot_ = new managed_shared_memory(open_only, "SharedMemoryReleaseRobot");

        // Open existing named mutex
        std::cout << "Opening existing named mutex..." << std::endl;
        this->mutex_joint_commands_ = new named_mutex(open_only, "SharedMutexCommands");
        this->mutex_g1_measurements_ = new named_mutex(open_only, "SharedMutexMeasurements");
        this->mutex_mocap_ = new named_mutex(open_only, "SharedMutexMocap");
        this->mutex_camera_ = new named_mutex(open_only, "SharedMutexCamera");
        this->mutex_lower_robot_ = new named_mutex(open_only, "SharedMutexLowerRobot");
        this->mutex_release_robot_ = new named_mutex(open_only, "SharedMutexReleaseRobot");

        // Access the allocated objects in shared memory
        std::cout << "Accessing allocated objects in shared memory..." << std::endl;
        this->joint_commands_ = segment_joint_commands_->find<G1Commands>("G1Commands").first;
        this->g1_measurements_ = segment_g1_measurements_->find<G1Measurements>("G1Measurements").first;
        this->mocap_ptr_ = segment_mocap_->find<MocapMeasurement>("MocapMeasurement").first;
        this->camera_ptr_ = segment_camera_->find<CameraMeasurement>("CameraMeasurement").first;
        this->lower_robot_ptr_ = segment_lower_robot_->find<LowerRobotInstructions>("LowerRobot").first;
        this->release_robot_ptr_ = segment_release_robot_->find<bool>("ReleaseRobot").first;
    }



    if(create_memory == true)
    {
        // Initialize the memory
        std::cout << "Initializing shared memory..." << std::endl;

        // Commands
        G1JointVec joint_pos_cmd = G1JointVec::Zero();
        G1JointVec joint_vel_cmd = G1JointVec::Zero();
        G1JointVec joint_tor_cmd = G1JointVec::Zero();
        G1JointVec kp_gains = G1JointVec::Zero();
        G1JointVec kd_gains = G1JointVec::Zero();

        // Measurements
        G1JointVec joint_pos = G1JointVec::Zero();
        G1JointVec joint_vel = G1JointVec::Zero();
        G1JointVec joint_acc = G1JointVec::Zero();
        G1JointVec joint_tor = G1JointVec::Zero();
        G1IMUVec imu = G1IMUVec::Zero();
        imu(ImuIDX::Q_W) = 1.0;

        // Mocap
        MocapMeasurement mocap_measurement = MocapMeasurement::Zero();
        mocap_measurement(MocapIDX::Q_W) = 1.0;

        // Camera
        CameraMeasurement camera_measurement = CameraMeasurement::Zero();
        camera_measurement(CameraIDX::Q_W) = 1.0;

        // Lower robot
        LowerRobotInstructions lower_robot_instructions;

        // Release robot
        bool release_robot = false;
        std::cerr << "Release robot: " << std::endl;
        // Update the shared memory

        // Commands
        this->mutex_joint_commands_->lock();
        std::cerr << "Mutex locked: " << std::endl;
        this->joint_commands_->joint_pos_cmd = joint_pos_cmd;
        std::cerr << "Joint pos cmd: " << std::endl;
        this->joint_commands_->joint_vel_cmd = joint_vel_cmd;
        this->joint_commands_->joint_tor_cmd = joint_tor_cmd;
        this->joint_commands_->joint_kp_cmd = kp_gains;
        this->joint_commands_->joint_kd_cmd = kd_gains;
        this->mutex_joint_commands_->unlock();
        std::cerr << "Commands: " << std::endl;
        // Measurements
        this->mutex_g1_measurements_->lock();
        this->g1_measurements_->joint_pos = joint_pos;
        this->g1_measurements_->joint_vel = joint_vel;
        this->g1_measurements_->joint_acc = joint_acc;
        this->g1_measurements_->joint_tor = joint_tor;
        this->g1_measurements_->imu = imu;
        this->mutex_g1_measurements_->unlock();
        std::cerr << "Measurements: " << std::endl;
        // Mocap
        this->mutex_mocap_->lock();
        *this->mocap_ptr_ = mocap_measurement;
        this->mutex_mocap_->unlock();

        // Camera
        this->mutex_camera_->lock();
        *this->camera_ptr_ = camera_measurement;
        this->mutex_camera_->unlock();
        std::cerr << "Camera measurement: " << std::endl;
        // Lower robot
        this->mutex_lower_robot_->lock();
        *this->lower_robot_ptr_ = lower_robot_instructions;
        this->mutex_lower_robot_->unlock();

        // Release robot
        this->mutex_release_robot_->lock();
        *this->release_robot_ptr_ = release_robot;
        this->mutex_release_robot_->unlock();

        std::cout << "Shared memory initialized!" << std::endl;       
    }
}

void SharedMemoryInterface::GetG1Commands(G1JointVec &joint_pos_cmd,
                                          G1JointVec &joint_vel_cmd,
                                          G1JointVec &joint_tor_cmd,
                                          G1JointVec &kp_gains,
                                          G1JointVec &kd_gains)
{
    // Lock the mutex
    this->mutex_joint_commands_->lock();

    // Get the commands
    joint_pos_cmd = this->joint_commands_->joint_pos_cmd;
    joint_vel_cmd = this->joint_commands_->joint_vel_cmd;
    joint_tor_cmd = this->joint_commands_->joint_tor_cmd;
    kp_gains = this->joint_commands_->joint_kp_cmd;
    kd_gains = this->joint_commands_->joint_kd_cmd;

    // Unlock the mutex
    this->mutex_joint_commands_->unlock();
}

void SharedMemoryInterface::GetG1Measurements(G1JointVec &joint_pos,
                                              G1JointVec &joint_vel,
                                              G1JointVec &joint_acc,
                                              G1JointVec &joint_tor,
                                              G1IMUVec &imu)
{
    // Lock the mutex
    this->mutex_g1_measurements_->lock();

    // Get the measurements
    joint_pos = this->g1_measurements_->joint_pos;
    joint_vel = this->g1_measurements_->joint_vel;
    joint_acc = this->g1_measurements_->joint_acc;
    joint_tor = this->g1_measurements_->joint_tor;
    imu = this->g1_measurements_->imu;

    // Unlock the mutex
    this->mutex_g1_measurements_->unlock();
}

void SharedMemoryInterface::GetMocapMeasurement(MocapMeasurement &mocap_measurement)
{
    // Lock the mutex
    this->mutex_mocap_->lock();

    // Get the mocap measurement
    mocap_measurement = *this->mocap_ptr_;

    // Unlock the mutex
    this->mutex_mocap_->unlock();
}

void SharedMemoryInterface::GetCameraMeasurement(CameraMeasurement &camera_measurement)
{
    // Lock the mutex
    this->mutex_camera_->lock();

    // Get the camera measurement
    camera_measurement = *this->camera_ptr_;

    // Unlock the mutex
    this->mutex_camera_->unlock();
}

void SharedMemoryInterface::GetLowerRobotInstructions(LowerRobotInstructions &lower_robot_instructions)
{
    // Lock the mutex
    this->mutex_lower_robot_->lock();

    // Get the lower robot flag
    lower_robot_instructions.lowering_active = this->lower_robot_ptr_->lowering_active;
    lower_robot_instructions.lowering_period = this->lower_robot_ptr_->lowering_period;
    lower_robot_instructions.final_height = this->lower_robot_ptr_->final_height;

    // Unlock the mutex
    this->mutex_lower_robot_->unlock();
}

void SharedMemoryInterface::GetReleaseRobotFlag(bool &release_robot)
{
    // Lock the mutex
    this->mutex_release_robot_->lock();

    // Get the release robot flag
    release_robot = *this->release_robot_ptr_;

    // Unlock the mutex
    this->mutex_release_robot_->unlock();
}

void SharedMemoryInterface::SetG1Commands(G1JointVec &joint_pos_cmd,
                                          G1JointVec &joint_vel_cmd,
                                          G1JointVec &joint_tor_cmd,
                                          G1JointVec &kp_gains,
                                          G1JointVec &kd_gains)
{
    // Lock the mutex
    this->mutex_joint_commands_->lock();

    // Set the commands
    this->joint_commands_->joint_pos_cmd = joint_pos_cmd;
    this->joint_commands_->joint_vel_cmd = joint_vel_cmd;
    this->joint_commands_->joint_tor_cmd = joint_tor_cmd;
    this->joint_commands_->joint_kp_cmd = kp_gains;
    this->joint_commands_->joint_kd_cmd = kd_gains;

    // Unlock the mutex
    this->mutex_joint_commands_->unlock();
}

void SharedMemoryInterface::SetG1Measurements(G1JointVec &joint_pos,
                                              G1JointVec &joint_vel,
                                              G1JointVec &joint_acc,
                                              G1JointVec &joint_tor,
                                              G1IMUVec &imu)
{
    // Lock the mutex
    this->mutex_g1_measurements_->lock();

    // Set the measurements
    this->g1_measurements_->joint_pos = joint_pos;
    this->g1_measurements_->joint_vel = joint_vel;
    this->g1_measurements_->joint_acc = joint_acc;
    this->g1_measurements_->joint_tor = joint_tor;
    this->g1_measurements_->imu = imu;

    // Unlock the mutex
    this->mutex_g1_measurements_->unlock();
}

void SharedMemoryInterface::SetMocapMeasurement(MocapMeasurement &mocap_measurement)
{
    // Lock the mutex
    this->mutex_mocap_->lock();

    // Set the mocap measurement
    *this->mocap_ptr_ = mocap_measurement;

    // Unlock the mutex
    this->mutex_mocap_->unlock();
}

void SharedMemoryInterface::SetCameraMeasurement(CameraMeasurement &camera_measurement)
{
    // Lock the mutex
    this->mutex_camera_->lock();

    // Set the camera measurement
    *this->camera_ptr_ = camera_measurement;

    // Unlock the mutex
    this->mutex_camera_->unlock();
}

void SharedMemoryInterface::SetLowerRobotInstructions(LowerRobotInstructions &lower_robot_instructions)
{
    // Lock the mutex
    this->mutex_lower_robot_->lock();

    // Set the lower robot flag
    this->lower_robot_ptr_->lowering_active = lower_robot_instructions.lowering_active;
    this->lower_robot_ptr_->lowering_period = lower_robot_instructions.lowering_period;
    this->lower_robot_ptr_->final_height = lower_robot_instructions.final_height;

    // Unlock the mutex
    this->mutex_lower_robot_->unlock();
}

void SharedMemoryInterface::SetReleaseRobotFlag(bool &release_robot)
{
    // Lock the mutex
    this->mutex_release_robot_->lock();

    // Set the release robot flag
    *this->release_robot_ptr_ = release_robot;

    // Unlock the mutex
    this->mutex_release_robot_->unlock();
}