#ifndef __GAZEBO_XBOT_IMU_H__
#define __GAZEBO_XBOT_IMU_H__

#include <gazebo/sensors/ImuSensor.hh>

#include <XBotCore-interfaces/All.h>

#include <XBotInterface/XBotInterface.h>

namespace gazebo
{
class GazeboXBotImu :
    public XBot::IXBotIMU
{

public :
    /**
     * @brief constructor
     *
     */
    GazeboXBotImu();

    /**
     * @brief destructor
     *
     */
    virtual ~GazeboXBotImu();

    bool loadImuSensors(
        XBot::RobotInterface::Ptr robot,
        gazebo::sensors::Sensor_V& _sensors_attached_to_robot);

protected:

private:
    // xbot robot
    XBot::RobotInterface::Ptr _robot;
    
    // imu callback helpers
    std::map<int, gazebo::sensors::ImuSensorPtr> _imu_gazebo_map;

    // NOTE IXBotIMU 
    virtual bool get_imu(int imu_id, std::vector< double >& lin_acc,
                         std::vector< double >& ang_vel,
                         std::vector< double >& quaternion) final;

    virtual bool get_imu_fault(int imu_id, double& fault) final;

    virtual bool get_imu_rtt(int imu_id, double& rtt) final;


};

}

#endif
