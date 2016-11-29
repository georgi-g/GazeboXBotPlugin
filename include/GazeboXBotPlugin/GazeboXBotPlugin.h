/*
 * Copyright (C) 2016 Walkman
 * Author: Luca Muratore
 * email:  luca.muratore@iit.it
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

#include <XBotCore/XBotPlugin.h>
#include <XBotCore/IXBotJoint.h>
#include <XBotCoreModel.h>

#include <XBotInterface/RobotInterface.h>
#include <SharedLibraryClassFactory.h>




namespace gazebo {
class GazeboXBotPlugin : public ModelPlugin,
                         public XBot::IXBotJoint

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

private:

     void XBotUpdate(const common::UpdateInfo & _info);
     
     bool parseYAML ( const std::string &path_to_cfg ); // TBD do it with UTILS
     
     static bool computeAbsolutePath ( const std::string& input_path,
                                       const std::string& midlle_path,
                                       std::string& absolute_path ); // TBD do it with UTILS
     
     bool loadPlugins();
     
     bool initPlugins();
     
     // internal XBotCoreModel object: it does the trick using URDF, SRDF and joint map configuration
     XBot::XBotCoreModel _XBotModel;
     
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
     
     // RobotInterface instance
     XBot::RobotInterface::Ptr _robot;
     
     // Gazebo joint names vector
     std::vector<std::string> _jointNames;
     
     // Gazebo joint map
     std::map<std::string, gazebo::physics::JointPtr> _jointMap;

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
    
    virtual bool get_link_vel(int joint_id, float& link_vel) final;
    
    virtual bool get_motor_vel(int joint_id, int16_t& motor_vel) final;
    
    virtual bool get_torque(int joint_id, int16_t& torque) final;
    
    virtual bool get_max_temperature(int joint_id, uint16_t& max_temperature) final;
    
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
