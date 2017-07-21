#ifndef __GAZEBO_XBOT_FT_H__
#define __GAZEBO_XBOT_FT_H__

#include <gazebo/sensors/ForceTorqueSensor.hh>

#include <XBotCore-interfaces/All.h>

#include <XBotInterface/XBotInterface.h>

namespace gazebo
{
class GazeboXBotFt :
    public XBot::IXBotFT

{

public :
    /**
     * @brief constructor
     *
     */
    GazeboXBotFt();

    /**
     * @brief destructor
     *
     */
    virtual ~GazeboXBotFt();

    void setRobot(
        XBot::RobotInterface::Ptr robot,
        std::map<int, gazebo::sensors::ForceTorqueSensorPtr> ft_gazebo_map);
    
protected:

private:

    // xbot robot
    XBot::RobotInterface::Ptr _robot;

    // ft callback helpers
    std::map<int, gazebo::sensors::ForceTorqueSensorPtr> _ft_gazebo_map;

    // NOTE IXBotFT
    
    virtual bool get_ft(int ft_id, std::vector< double >& ft, int channels = 6) final;

    virtual bool get_ft_fault(int ft_id, double& fault) final;

    virtual bool get_ft_rtt(int ft_id, double& rtt) final;
    
};

}

#endif
