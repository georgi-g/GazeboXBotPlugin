
#include<GazeboXBotPlugin/GazeboXBotFt.h>


gazebo::GazeboXBotFt::GazeboXBotFt()
{
}

gazebo::GazeboXBotFt::~GazeboXBotFt()
{
}

bool gazebo::GazeboXBotFt::loadFTSensors(XBot::RobotInterface::Ptr robot, gazebo::sensors::Sensor_V& _sensors_attached_to_robot)
{
    _robot = robot;
    
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    bool ret = true;
    std::cout << "Loading F-T sensors ... " << std::endl;
    
    for( const auto& FT_pair : _robot->getForceTorque() ) {

        // check XBot FT ptr
        if(!FT_pair.second){
            std::cout << "ERROR! FT NULLPTR!!!" << std::endl;
            continue;
        }

        // get id and name of the FT sensor
        const XBot::ForceTorqueSensor& ft = *FT_pair.second;
        int ft_id = ft.getSensorId();
        std::string ft_name = ft.getSensorName();

        for(unsigned int i = 0; i < _sensors_attached_to_robot.size(); ++i) {
            #if GAZEBO_MAJOR_VERSION <= 6
            // if the sensor is a FT and has the ft_name
            if( ( _sensors_attached_to_robot[i]->GetType().compare("force_torque") == 0 ) &&
                ( _sensors_attached_to_robot[i]->GetName() == ft_name ) )
            {
                _ft_gazebo_map[ft_id] = boost::static_pointer_cast<gazebo::sensors::ForceTorqueSensor>(_sensors_attached_to_robot[i]); 
                std::cout << "F-T found: " << _ft_gazebo_map.at(ft_id)->GetName() << std::endl;
            }
            #else 
            if( ( _sensors_attached_to_robot[i]->Type().compare("force_torque") == 0 ) &&
                ( _sensors_attached_to_robot[i]->Name() == ft_name ) )
            {
                _ft_gazebo_map[ft_id] = std::static_pointer_cast<gazebo::sensors::ForceTorqueSensor>(_sensors_attached_to_robot[i]); 
                std::cout << "F-T found: " << _ft_gazebo_map.at(ft_id)->Name() << std::endl;
            }
            #endif

        }
        
    }

    return ret;
}

bool gazebo::GazeboXBotFt::get_ft(int ft_id, std::vector< double >& ft, int channels)
{
    auto it =_ft_gazebo_map.find(ft_id);
    // if ft is not found
    if( it == _ft_gazebo_map.end() ) {
        std::cout << "WARNING: FT with id : " << ft_id << " not found in the GAZEBO model you loaded: check you put the right urdf_path, srdf_path and joint_config_map in your YAML config file." << std::endl;
        return false;
    }

    // imu found
    auto ft_gazebo = it->second;

    ft.assign(channels, 0.0);

    if( !ft_gazebo ) {
        ft.assign(channels, 0.0);
        return false;
    }

    ft[0] = ft_gazebo->Force().X();
    ft[1] = ft_gazebo->Force().Y();
    ft[2] = ft_gazebo->Force().Z();
    ft[3] = ft_gazebo->Torque().X();
    ft[4] = ft_gazebo->Torque().Y();
    ft[5] = ft_gazebo->Torque().Z();

    return true;
}

bool gazebo::GazeboXBotFt::get_ft_fault(int ft_id, double& fault)
{
    auto ft_ptr = _robot->getForceTorque(ft_id);

    if(!ft_ptr){
        fault = 0;
        return false;
    }

    fault = 0;
    return true;
}

bool gazebo::GazeboXBotFt::get_ft_rtt(int ft_id, double& rtt)
{
    auto ft_ptr = _robot->getForceTorque(ft_id);

    if(!ft_ptr){
        rtt = 0;
        return false;
    }

    rtt = 0;
    return true;
}


