
#include<GazeboXBotPlugin/GazeboXBotImu.h>


gazebo::GazeboXBotImu::GazeboXBotImu()
{
}

gazebo::GazeboXBotImu::~GazeboXBotImu()
{
}

void gazebo::GazeboXBotImu::setRobot(
    XBot::RobotInterface::Ptr robot,
    std::map<int, gazebo::sensors::ImuSensorPtr> imu_gazebo_map)
{
    _robot = robot;
    _imu_gazebo_map = imu_gazebo_map;
}

bool gazebo::GazeboXBotImu::get_imu(int imu_id,
                                       std::vector< double >& lin_acc,
                                       std::vector< double >& ang_vel,
                                       std::vector< double >& quaternion)
{

    auto it =_imu_gazebo_map.find(imu_id);
    // if imu is not found
    if( it == _imu_gazebo_map.end() ) {
        std::cout << "WARNING: IMU with id : " << imu_id << " not found in the GAZEBO model you loaded: check you put the right urdf_path, srdf_path and joint_config_map in your YAML config file." << std::endl;
        return false;
    }

    // imu found
    auto imu_gazebo = it->second;
    
    lin_acc.assign(3, 0.0);
    ang_vel.assign(3, 0.0);
    quaternion.assign(4, 0.0);
    quaternion[3] = 1.0;

    if(!imu_gazebo){
        lin_acc.assign(3, 0.0);
        ang_vel.assign(3, 0.0);
        quaternion.assign(4, 0.0);
        quaternion[3] = 1.0;

        return false;
    }

    lin_acc[0] = imu_gazebo->LinearAcceleration().X();
    lin_acc[1] = imu_gazebo->LinearAcceleration().Y();
    lin_acc[2] = imu_gazebo->LinearAcceleration().Z();

    ang_vel[0] = imu_gazebo->AngularVelocity().X();
    ang_vel[1] = imu_gazebo->AngularVelocity().Y();
    ang_vel[2] = imu_gazebo->AngularVelocity().Z();

    quaternion[0] = imu_gazebo->Orientation().X();
    quaternion[1] = imu_gazebo->Orientation().Y();
    quaternion[2] = imu_gazebo->Orientation().Z();
    quaternion[3] = imu_gazebo->Orientation().W();
    
    return true;


}

bool gazebo::GazeboXBotImu::get_imu_fault(int imu_id, double& fault)
{
    auto imu_ptr = _robot->getImu(imu_id);

    if(!imu_ptr){
        fault = 0;
        return false;
    }

    fault = 0;
    return true;
}

bool gazebo::GazeboXBotImu::get_imu_rtt(int imu_id, double& rtt)
{
    auto imu_ptr = _robot->getImu(imu_id);

    if(!imu_ptr){
        rtt = 0;
        return false;
    }

    rtt = 0;
    return true;
}


