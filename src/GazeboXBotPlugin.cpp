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


#include <iostream>
#include <csignal>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>

#include <XCM/TimeProvider.h>

#include<GazeboXBotPlugin/GazeboXBotPlugin.h>

#include <GazeboXBotPlugin/DefaultGazeboPID.h>

#include <gazebo/sensors/sensors.hh>



sig_atomic_t g_loop_ok = 1;

void sigint_handler(int s){
    g_loop_ok = 0;
}


gazebo::GazeboXBotPlugin::GazeboXBotPlugin()
{
    _shared_memory = std::make_shared<XBot::SharedMemory>();
    signal(SIGINT, sigint_handler);

}


gazebo::GazeboXBotPlugin::~GazeboXBotPlugin()
{
    close_all();
}


void gazebo::GazeboXBotPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    _model = _parent;

    // Get a handle to the world
    _world = _model->GetWorld();

    // Get node
    _node.reset(new transport::Node());
    _node->Init();

    // Listen to the update event. This event is broadcast every simulation iteration
    _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboXBotPlugin::XBotUpdate, this, _1));

    // Save the SDF handle
    this->_sdf = _sdf;

    // Get the path to config file from SDF
    if( !_sdf->HasElement("path_to_config_file") ){
        std::cerr << "ERROR in " << __func__ << "! Missing element path_to_config_file!" << std::endl;
        return;
    }
    _path_to_config = _sdf->GetElement("path_to_config_file")->Get<std::string>();

    // compute path
    computeAbsolutePath(_path_to_config, "/", _path_to_config);

    // TBD check if the load is successfull
    YAML::Node root = YAML::LoadFile(_path_to_config);

    // create robot from config file and any map
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    _xbot_joint = std::make_shared<GazeboXBotJoint>();
    _xbot_imu = std::make_shared<GazeboXBotImu>();
    _xbot_ft = std::make_shared<GazeboXBotFt>();
    _xbot_hand = std::make_shared<GazeboXBotHand>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint = _xbot_joint;
    std::shared_ptr<XBot::IXBotFT> xbot_ft = _xbot_ft;
    std::shared_ptr<XBot::IXBotIMU> xbot_imu = _xbot_imu;
    std::shared_ptr<XBot::IXBotHand> xbot_hand = _xbot_hand;
    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);
    (*anymap)["XBotIMU"] = boost::any(xbot_imu);
    (*anymap)["XBotHand"] = boost::any(xbot_hand);

    // initialize the robot
    _robot = XBot::RobotInterface::getRobot(_path_to_config, anymap, "XBotRT");

    // create time provider function
    boost::function<double()> time_func = boost::bind(&gazebo::GazeboXBotPlugin::get_time, this);
    // create time provider
    auto time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);

    // create plugin handler
    _pluginHandler = std::make_shared<XBot::PluginHandler>(_robot, time_provider, "XBotRTPlugins");

    _xbot_joint->loadJoints(_robot, _model, root);

    // set the control rate
    if(root["GazeboXBotPlugin"]["control_rate"]){
        _control_rate = root["GazeboXBotPlugin"]["control_rate"].as<double>();
    }
    else{
        // defaulte control rate at 1ms
        _control_rate = 0.001;
    }
    
    // gazebo sensors
    // get the list of sensors
    gazebo::sensors::Sensor_V _sensors = gazebo::sensors::SensorManager::Instance()->GetSensors();
    
    // gazebo sensors attached to the current robot
    gazebo::sensors::Sensor_V _sensors_attached_to_robot;
    
    
    // if multiple robot are simulated we need to get only the sensors attached to our robot
    for(unsigned int i = 0; i < _sensors.size(); ++i) {
        #if GAZEBO_MAJOR_VERSION <= 6
        if(_sensors[i]->GetScopedName().find("::"+_model->GetName()+"::") != std::string::npos) { 
            _sensors_attached_to_robot.push_back(_sensors[i]);
            std::cout << _sensors_attached_to_robot[i]->GetScopedName() << std::endl;
        }
        #else 
        if(_sensors[i]->ScopedName().find("::"+_model->GetName()+"::") != std::string::npos) { 
            _sensors_attached_to_robot.push_back(_sensors[i]);
            std::cout << _sensors_attached_to_robot[i]->ScopedName() << std::endl;
        }
        #endif
    }

    // load FT sensors
    _xbot_ft->loadFTSensors(_robot, _sensors_attached_to_robot);

    // load IMU sensors
    _xbot_imu->loadImuSensors(_robot, _sensors_attached_to_robot);


}

void gazebo::GazeboXBotPlugin::Init()
{
    gazebo::ModelPlugin::Init();
    std::cout << "GazeboXBotPlugin Init() started" << std::endl;

    // Load plugins
    loadPlugins();

    // Init plugins
    initPlugins();

    // Set _previous_time
    _previous_time = get_time();

    // init grasping ROS node
    _xbot_hand->initGrasping(_robot);
}

bool gazebo::GazeboXBotPlugin::loadPlugins()
{
    return _pluginHandler->load_plugins();
}

bool gazebo::GazeboXBotPlugin::initPlugins()
{
    return _pluginHandler->init_plugins(_shared_memory, _xbot_joint);
}

void gazebo::GazeboXBotPlugin::close_all()
{
    _pluginHandler->close();

}

double gazebo::GazeboXBotPlugin::get_time()
{
    return _world->GetSimTime().Double();
}



void gazebo::GazeboXBotPlugin::XBotUpdate(const common::UpdateInfo & _info)
{
//     std::cout << __PRETTY_FUNCTION__ << std::endl;

    if( g_loop_ok == 0 ){
        std::cout << "CTRL+C detected..." << std::endl;
        // NOTE calling explicitly the destructor
        this->~GazeboXBotPlugin();
        exit(1);
    }

    if( (get_time() - _previous_time) >= (_control_rate * 0.999) ){

        _pluginHandler->run();
        _previous_time = get_time();

    }

    _xbot_joint->XBotUpdate();
}

void gazebo::GazeboXBotPlugin::Reset()
{
    gazebo::ModelPlugin::Reset();
}



bool gazebo::GazeboXBotPlugin::computeAbsolutePath (const std::string& input_path,
                                                    const std::string& middle_path,
                                                    std::string& absolute_path)
    {
    // if not an absolute path
    if(!(input_path.at(0) == '/')) {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
        // check the env, otherwise error
        if(env_p) {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += middle_path;
            current_path += input_path;
            absolute_path = current_path;
            return true;
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return false;
        }
    }
    // already an absolute path
    absolute_path = input_path;
    return true;
}



