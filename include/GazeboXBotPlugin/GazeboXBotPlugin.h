/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef __GAZEBO_XBOT_PLUGIN_H__
#define __GAZEBO_XBOT_PLUGIN_H__

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/ImuSensor.hh>

#include <XBotCore-interfaces/All.h>

#include <XBotInterface/XBotInterface.h>

#include <XCM/XBotPluginHandler.h>

#include <GazeboXBotPlugin/JointController.h>
#include <GazeboXBotPlugin/CallbackHelper.h>
#include <GazeboXBotPlugin/GazeboXBotHand.h>


namespace gazebo
{
class GazeboXBotPlugin :
    public ModelPlugin,
    public XBot::IXBotJoint, 
    public XBot::IXBotIMU,
    public XBot::IXBotFT

{

public :
    /**
     * @brief constructor
     *
     */
    GazeboXBotPlugin();

    /**
     * @brief destructor
     *
     */
    virtual ~GazeboXBotPlugin();

    /**
     * @brief Called after the plugin has been constructed.
     *
     * @param _argc Number of command line arguments.
     * @param _argv Array of command line arguments.
     * @return void
     */
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /**
     * @brief Called once after Load
     *
     * @return void
     */
    virtual void Init();

    /**
     * @brief Custom plugin reset behavior.
     *
     * @return void
     */
    virtual void Reset();

protected:

private:



    void XBotUpdate(const common::UpdateInfo& _info);

    static bool computeAbsolutePath(const std::string& input_path,
                                    const std::string& midlle_path,
                                    std::string& absolute_path);  // TBD do it with UTILS

    bool loadPlugins();
    
    bool loadFTSensors();

    bool loadImuSensors();

    bool initPlugins();

    void close_all();

    double get_time();


    // xbot robot
    XBot::RobotInterface::Ptr _robot;

    // xbot plugin handler
    XBot::PluginHandler::Ptr _pluginHandler;
    
    // xbot shared memory for RT plugin
    XBot::SharedMemory::Ptr _shared_memory;

    // control rate
    double _control_rate;

    // previous iteration time (for keeping 1kHz control rate)
    double _previous_time;

    // robot map
    std::map<std::string, std::vector<int>> _XBotRobot;

    // path to config file
    std::string _path_to_config;

    // Gazebo joint names vector
    std::vector<std::string> _jointNames;

    std::shared_ptr<GazeboXBotHand> _xbot_hand;
    
    // Gazebo joint map
    std::map<std::string, gazebo::physics::JointPtr> _jointMap;
    std::map<std::string, XBot::JointController::Ptr> _joint_controller_map;

    // model
    physics::ModelPtr _model;

    // world
    gazebo::physics::WorldPtr _world;

    // Pointer to the update event connection
    event::ConnectionPtr _updateConnection;

    // pointer to sdf
    sdf::ElementPtr _sdf;

    // gazebo transport node
    gazebo::transport::NodePtr _node;
    
    // gazebo sensors
    gazebo::sensors::Sensor_V _sensors;
    
    // gazebo sensors attached to the current robot
    gazebo::sensors::Sensor_V _sensors_attached_to_robot;
    
    // imu callback helpers
    std::map<int, gazebo::sensors::ImuSensorPtr> _imu_gazebo_map;
    // ft callback helpers
    std::map<int, gazebo::sensors::ForceTorqueSensorPtr> _ft_gazebo_map;

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

    // NOTE IXBotFT
    
    virtual bool get_ft(int ft_id, std::vector< double >& ft, int channels = 6) final;

    virtual bool get_ft_fault(int ft_id, double& fault) final;

    virtual bool get_ft_rtt(int ft_id, double& rtt) final;
    
    
    // NOTE IXBotIMU 
    virtual bool get_imu(int imu_id, std::vector< double >& lin_acc,
                         std::vector< double >& ang_vel,
                         std::vector< double >& quaternion) final;

    virtual bool get_imu_fault(int imu_id, double& fault) final;

    virtual bool get_imu_rtt(int imu_id, double& rtt) final;


};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboXBotPlugin)

}

#endif
