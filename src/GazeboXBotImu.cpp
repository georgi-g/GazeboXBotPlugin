
#include<GazeboXBotPlugin/GazeboXBotImu.h>


gazebo::GazeboXBotImu::GazeboXBotImu()
{
}

gazebo::GazeboXBotImu::~GazeboXBotImu()
{
}

bool gazebo::GazeboXBotImu::loadImuSensors(XBot::RobotInterface::Ptr robot, gazebo::sensors::Sensor_V& _sensors_attached_to_robot)
{
    _robot = robot;
    
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    bool ret = true;
    std::cout << "Loading IMU sensors ... " << std::endl;
    
    for( const auto& imu_pair : _robot->getImu() ) {

        // check XBot FT ptr
        if(!imu_pair.second){
            std::cout << "ERROR! imu NULLPTR!!!" << std::endl;
            continue;
        }

        // get id and name of the FT sensor
        const XBot::ImuSensor& imu = *imu_pair.second;
        int imu_id = imu.getSensorId();
        std::string imu_name = imu.getSensorName();

        for(unsigned int i = 0; i < _sensors_attached_to_robot.size(); ++i) {
            #if GAZEBO_MAJOR_VERSION <= 6
            // if the sensor is a IMU and has the ft_name
            if( ( _sensors_attached_to_robot[i]->GetType().compare("imu") == 0 ) &&
                ( _sensors_attached_to_robot[i]->GetName() == imu_name ) )
            {
                _imu_gazebo_map[imu_id] = boost::static_pointer_cast<gazebo::sensors::ImuSensor>(_sensors_attached_to_robot[i]); 
                std::cout << "IMU found: " << _imu_gazebo_map.at(imu_id)->GetName() << std::endl;
            }
            #else 
            if( ( _sensors_attached_to_robot[i]->Type().compare("imu") == 0 ) &&
                ( _sensors_attached_to_robot[i]->Name() == imu_name ) )
            {
                _imu_gazebo_map[imu_id] = std::static_pointer_cast<gazebo::sensors::ImuSensor>(_sensors_attached_to_robot[i]); 
                std::cout << "IMU found: " << _imu_gazebo_map.at(imu_id)->Name() << std::endl;
            }
            #endif

        }
        
    }
    
    return ret;
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


