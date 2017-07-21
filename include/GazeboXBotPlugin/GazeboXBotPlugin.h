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
#include <GazeboXBotPlugin/GazeboXBotJoint.h>
#include <GazeboXBotPlugin/GazeboXBotImu.h>
#include <GazeboXBotPlugin/GazeboXBotFt.h>


namespace gazebo
{
class GazeboXBotPlugin :
    public ModelPlugin

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

    std::shared_ptr<GazeboXBotJoint> _xbot_joint;
    std::shared_ptr<GazeboXBotImu> _xbot_imu;
    std::shared_ptr<GazeboXBotFt> _xbot_ft;
    
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
    
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboXBotPlugin)

}

#endif
