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

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <XBotCore-interfaces/All.h>
#include <GazeboXBotPlugin/JointController.h>
#include <XBotCoreModel.h>

#include <SharedLibraryClassFactory.h>
#include <SharedLibraryClass.h>




namespace gazebo {
class GazeboXBotPlugin : public ModelPlugin,
                         public XBot::IXBotJoint,
                         public XBot::IXBotChain,
                         public XBot::IXBotRobot

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

    // NOTE IXBotRobot getters
    virtual bool get_robot_link_pos(std::map<std::string, float>& link_pos) final;
    virtual bool get_robot_link_pos(std::map<int, float>& link_pos) final;

    virtual bool get_robot_motor_pos(std::map<std::string, float>& motor_pos) final;
    virtual bool get_robot_motor_pos(std::map<int, float>& motor_pos) final;

    virtual bool get_robot_link_vel(std::map<std::string, int16_t>& link_vel) final;
    virtual bool get_robot_link_vel(std::map<int, int16_t>& link_vel) final;

    virtual bool get_robot_motor_vel(std::map<std::string, int16_t>& motor_vel) final;
    virtual bool get_robot_motor_vel(std::map<int, int16_t>& motor_vel) final;

    virtual bool get_robot_torque(std::map<std::string, float>& torque) final;
    virtual bool get_robot_torque(std::map<int, float>& torque) final;

    virtual bool get_robot_temperature(std::map<int, uint16_t>& temperature) final;
    virtual bool get_robot_temperature(std::map<std::string, uint16_t>& temperature) final;

    virtual bool get_robot_fault(std::map<int, uint16_t>& fault) final;
    virtual bool get_robot_fault(std::map<std::string, uint16_t>& fault) final;

    virtual bool get_robot_rtt(std::map<int, uint16_t>& rtt) final;
    virtual bool get_robot_rtt(std::map<std::string, uint16_t>& rtt) final;

    virtual bool get_robot_op_idx_ack(std::map<int, uint16_t>& op_idx_ack) final;
    virtual bool get_robot_op_idx_ack(std::map<std::string, uint16_t>& op_idx_ack) final;

    virtual bool get_robot_aux(std::map<std::string, float>& aux) final;
    virtual bool get_robot_aux(std::map<int, float>& aux) final;

    // NOTE IXBotRobot setters
    virtual bool set_robot_pos_ref(const std::map<std::string, float>& pos_ref) final;
    virtual bool set_robot_pos_ref(const std::map<int, float>& pos_ref) final;

    virtual bool set_robot_vel_ref(const std::map<std::string, int16_t>& vel_ref) final;
    virtual bool set_robot_vel_ref(const std::map<int, int16_t>& vel_ref) final;

    virtual bool set_robot_tor_ref(const std::map<std::string, int16_t>& tor_ref) final;
    virtual bool set_robot_tor_ref(const std::map<int, int16_t>& tor_ref) final;

    virtual bool set_robot_gains(const std::map<std::string, std::vector<uint16_t> >& gains) final;
    virtual bool set_robot_gains(const std::map<int, std::vector<uint16_t> >& gains) final;

    virtual bool set_robot_fault_ack(const std::map<std::string, int16_t>& fault_ack) final;
    virtual bool set_robot_fault_ack(const std::map<int, int16_t>& fault_ack) final;

    virtual bool set_robot_ts(const std::map<int, uint16_t>& ts) final;
    virtual bool set_robot_ts(const std::map<std::string, uint16_t>& ts) final;

    virtual bool set_robot_op_idx_aux(const std::map<int, uint16_t>& op_idx_aux) final;
    virtual bool set_robot_op_idx_aux(const std::map<std::string, uint16_t>& op_idx_aux) final;

    virtual bool set_robot_aux(const std::map<std::string, float>& aux) final;
    virtual bool set_robot_aux(const std::map<int, float>& aux) final;

    // NOTE IXBotChain getters
    virtual bool get_chain_link_pos(std::string chain_name, std::map<std::string, float>& link_pos) final;
    virtual bool get_chain_link_pos(std::string chain_name, std::map<int, float>& link_pos) final;

    virtual bool get_chain_motor_pos(std::string chain_name, std::map<std::string, float>& motor_pos) final;
    virtual bool get_chain_motor_pos(std::string chain_name, std::map<int, float>& motor_pos) final;

    virtual bool get_chain_link_vel(std::string chain_name, std::map<std::string, int16_t>& link_vel) final;
    virtual bool get_chain_link_vel(std::string chain_name, std::map<int, int16_t>& link_vel) final;

    virtual bool get_chain_motor_vel(std::string chain_name, std::map<std::string, int16_t>& motor_vel) final;
    virtual bool get_chain_motor_vel(std::string chain_name, std::map<int, int16_t>& motor_vel) final;

    virtual bool get_chain_torque(std::string chain_name, std::map<std::string, float>& torque) final;
    virtual bool get_chain_torque(std::string chain_name, std::map<int, float>& torque) final;

    virtual bool get_chain_temperature(std::string chain_name, std::map<int, uint16_t>& temperature) final;
    virtual bool get_chain_temperature(std::string chain_name, std::map<std::string, uint16_t>& temperature) final;

    virtual bool get_chain_fault(std::string chain_name, std::map<int, uint16_t>& fault) final;
    virtual bool get_chain_fault(std::string chain_name, std::map<std::string, uint16_t>& fault) final;

    virtual bool get_chain_rtt(std::string chain_name, std::map<int, uint16_t>& rtt) final;
    virtual bool get_chain_rtt(std::string chain_name, std::map<std::string, uint16_t>& rtt) final;

    virtual bool get_chain_op_idx_ack(std::string chain_name, std::map<int, uint16_t>& op_idx_ack) final;
    virtual bool get_chain_op_idx_ack(std::string chain_name, std::map<std::string, uint16_t>& op_idx_ack) final;

    virtual bool get_chain_aux(std::string chain_name, std::map<std::string, float>& aux) final;
    virtual bool get_chain_aux(std::string chain_name, std::map<int, float>& aux) final;

    // NOTE IXBotChain setters
    virtual bool set_chain_pos_ref(std::string chain_name, const std::map<std::string, float>& pos_ref) final;
    virtual bool set_chain_pos_ref(std::string chain_name, const std::map<int, float>& pos_ref) final;

    virtual bool set_chain_vel_ref(std::string chain_name, const std::map<std::string, int16_t>& vel_ref) final;
    virtual bool set_chain_vel_ref(std::string chain_name, const std::map<int, int16_t>& vel_ref) final;

    virtual bool set_chain_tor_ref(std::string chain_name, const std::map<std::string, int16_t>& tor_ref) final;
    virtual bool set_chain_tor_ref(std::string chain_name, const std::map<int, int16_t>& tor_ref) final;

    virtual bool set_chain_gains(std::string chain_name, const std::map<std::string, std::vector<uint16_t> >& gains) final;
    virtual bool set_chain_gains(std::string chain_name, const std::map<int, std::vector<uint16_t> >& gains) final;

    virtual bool set_chain_fault_ack(std::string chain_name, const std::map<std::string, int16_t>& fault_ack) final;
    virtual bool set_chain_fault_ack(std::string chain_name, const std::map<int, int16_t>& fault_ack) final;

    virtual bool set_chain_ts(std::string chain_name, const std::map<int, uint16_t>& ts) final;
    virtual bool set_chain_ts(std::string chain_name, const std::map<std::string, uint16_t>& ts) final;

    virtual bool set_chain_op_idx_aux(std::string chain_name, const std::map<int, uint16_t>& op_idx_aux) final;
    virtual bool set_chain_op_idx_aux(std::string chain_name, const std::map<std::string, uint16_t>& op_idx_aux) final;

    virtual bool set_chain_aux(std::string chain_name, const std::map<std::string, float>& aux) final;
    virtual bool set_chain_aux(std::string chain_name, const std::map<int, float>& aux) final;

private:

     void close_all();

     void XBotUpdate(const common::UpdateInfo & _info);

     bool parseYAML ( const std::string &path_to_cfg ); // TBD do it with UTILS

     static bool computeAbsolutePath ( const std::string& input_path,
                                       const std::string& midlle_path,
                                       std::string& absolute_path ); // TBD do it with UTILS

     bool loadPlugins();

     bool initPlugins();

     // internal XBotCoreModel object: it does the trick using URDF, SRDF and joint map configuration
     XBot::XBotCoreModel _XBotModel;

     std::map<std::string, std::vector<int>> _XBotRobot;

     // URDF SRDF and Joint Map Configuration
     std::string _urdf_path;
     std::string _srdf_path;
     std::string _joint_map_config;
     std::string _path_to_config;

     YAML::Node _root_cfg;

     // Dynamic loading related variables
     std::vector<std::shared_ptr<shlibpp::SharedLibraryClassFactory<XBot::XBotPlugin>>> _rtplugin_factory;
     std::vector<std::string> _rtplugin_names;
     std::vector<std::shared_ptr<shlibpp::SharedLibraryClass<XBot::XBotPlugin>>> _rtplugin_vector;

     std::vector<double> _last_time, _time, _period, _elapsed_time;
     bool _first_loop;

     // Gazebo joint names vector
     std::vector<std::string> _jointNames;

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

     // NOTE IXBotJoint getters
    virtual bool get_link_pos(int joint_id, float& link_pos) final;

    virtual bool get_motor_pos(int joint_id, float& motor_pos) final;

    virtual bool get_link_vel(int joint_id, int16_t& link_vel) final;

    virtual bool get_motor_vel(int joint_id, int16_t& motor_vel) final;

    virtual bool get_torque(int joint_id, float& torque) final;

    virtual bool get_temperature(int joint_id, uint16_t& temperature) final;

    virtual bool get_gains(int joint_id, std::vector<uint16_t>& gain_vector) final;

    virtual bool get_fault(int joint_id, uint16_t& fault) final;

    virtual bool get_rtt(int joint_id, uint16_t& rtt) final;

    virtual bool get_op_idx_ack(int joint_id, uint16_t& op_idx_ack) final;

    virtual bool get_aux(int joint_id, float& aux) final;

    // NOTE IXBotJoint setters
    virtual bool set_pos_ref(int joint_id, const float& pos_ref) final;

    virtual bool set_vel_ref(int joint_id, const int16_t& vel_ref) final;

    virtual bool set_tor_ref(int joint_id, const int16_t& tor_ref) final;

    virtual bool set_gains(int joint_id, const std::vector<uint16_t>& gains) final;

    virtual bool set_fault_ack(int joint_id, const int16_t& fault_ack) final;

    virtual bool set_ts(int joint_id, const uint16_t& ts) final;

    virtual bool set_op_idx_aux(int joint_id, const uint16_t& op_idx_aux) final;

    virtual bool set_aux(int joint_id, const float& aux) final;


};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN ( GazeboXBotPlugin )
}

#endif
