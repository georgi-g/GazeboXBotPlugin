
#include<GazeboXBotPlugin/GazeboXBotFt.h>


gazebo::GazeboXBotFt::GazeboXBotFt()
{
}

gazebo::GazeboXBotFt::~GazeboXBotFt()
{
}

void gazebo::GazeboXBotFt::setRobot(
    XBot::RobotInterface::Ptr robot,
    std::map<int, gazebo::sensors::ForceTorqueSensorPtr> ft_gazebo_map)
{
    _robot = robot;
    _ft_gazebo_map = ft_gazebo_map;
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


