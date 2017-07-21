
#ifndef __GAZEBO_XBOT_JOINT_H__
#define __GAZEBO_XBOT_JOINT_H__

#include <XBotCore-interfaces/All.h>

#include <XBotInterface/XBotInterface.h>

#include <GazeboXBotPlugin/JointController.h>

namespace gazebo
{
class GazeboXBotJoint :
    public XBot::IXBotJoint
{

public :
    /**
     * @brief constructor
     *
     */
    GazeboXBotJoint();

    /**
     * @brief destructor
     *
     */
    virtual ~GazeboXBotJoint();

    void setRobot(
        XBot::RobotInterface::Ptr robot,
        std::map<std::string, gazebo::physics::JointPtr> jointMap,
        std::map<std::string, XBot::JointController::Ptr> joint_controller_map);
    
    void XBotUpdate();
    
protected:

private:
    // xbot robot
    XBot::RobotInterface::Ptr _robot;
    
    // Gazebo joint map
    std::map<std::string, gazebo::physics::JointPtr> _jointMap;
    std::map<std::string, XBot::JointController::Ptr> _joint_controller_map;

    gazebo::physics::JointPtr getJoint(int joint_id);
    XBot::JointController::Ptr getJointController(int joint_id);
  

    // NOTE IXBotJoint getters
    virtual bool get_link_pos(int joint_id, double& link_pos) final;

    virtual bool get_motor_pos(int joint_id, double& motor_pos) final;

    virtual bool get_link_vel(int joint_id, double& link_vel) final;

    virtual bool get_motor_vel(int joint_id, double& motor_vel) final;

    virtual bool get_torque(int joint_id, double& torque) final;

    virtual bool get_temperature(int joint_id, double& temperature) final;

    virtual bool get_gains(int joint_id, std::vector<double>& gain_vector) final;

    virtual bool get_fault(int joint_id, double& fault) final;

    virtual bool get_rtt(int joint_id, double& rtt) final;

    virtual bool get_op_idx_ack(int joint_id, double& op_idx_ack) final;

    virtual bool get_aux(int joint_id, double& aux) final;

    virtual bool get_pos_ref(int joint_id, double& pos_ref) final;

    virtual bool get_vel_ref(int joint_id, double& vel_ref) final;

    virtual bool get_tor_ref(int joint_id, double& tor_ref) final;

    // NOTE IXBotJoint setters
    virtual bool set_pos_ref(int joint_id, const double& pos_ref) final;

    virtual bool set_vel_ref(int joint_id, const double& vel_ref) final;

    virtual bool set_tor_ref(int joint_id, const double& tor_ref) final;

    virtual bool set_gains(int joint_id, const std::vector<double>& gains) final;

    virtual bool set_fault_ack(int joint_id, const double& fault_ack) final;

    virtual bool set_ts(int joint_id, const double& ts) final;

    virtual bool set_op_idx_aux(int joint_id, const double& op_idx_aux) final;

    virtual bool set_aux(int joint_id, const double& aux) final;

};

}

#endif
