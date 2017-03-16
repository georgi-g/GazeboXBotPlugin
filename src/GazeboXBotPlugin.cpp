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

#include <XCM/TimeProvider.h>

#include<GazeboXBotPlugin/GazeboXBotPlugin.h>

#include <GazeboXBotPlugin/DefaultGazeboPID.h>
#include <GazeboXBotPlugin/JointImpedanceController.h>


sig_atomic_t g_loop_ok = 1;

void sigint_handler(int s){
    g_loop_ok = 0;
}


gazebo::GazeboXBotPlugin::GazeboXBotPlugin()
{
    std::cout << "GazeboXBotPlugin()" << std::endl;
    signal(SIGINT, sigint_handler);

}


gazebo::GazeboXBotPlugin::~GazeboXBotPlugin()
{
    std::cout << "~GazeboXBotPlugin()" << std::endl;
    close_all();

}

void gazebo::GazeboXBotPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::cout << "GazeboXBotPlugin Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)" << std::endl;
//     int a;
//     std::cin >> a;

    // Store the pointer to the model
    _model = _parent;

    // Get a handle to the world
    _world = _model->GetWorld();

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
    
    // create robot from config file and any map
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint(this);
    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    
    _robot = XBot::RobotInterface::getRobot(_path_to_config, anymap, "XBotRT");
    
    // create time provider function
    boost::function<double()> time_func = boost::bind(&gazebo::GazeboXBotPlugin::get_time, this);
    // create time provider
    auto time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);
    
    // create plugin handler
    _pluginHandler = std::make_shared<XBot::PluginHandler>(_robot, time_provider);
    
    // iterate over Gazebo model Joint vector and store Joint pointers in a map
    const gazebo::physics::Joint_V & gazebo_models_joints = _model->GetJoints();
    for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size(); gazebo_joint++) {
        std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
        _jointNames.push_back(gazebo_joint_name);
        _jointMap[gazebo_joint_name] = _model->GetJoint(gazebo_joint_name);

//         _joint_controller_map[gazebo_joint_name] =
//             std::make_shared<XBot::DefaultGazeboPID>( _model->GetJoint(gazebo_joint_name),
//                                                       _model->GetJointController() );

        _joint_controller_map[gazebo_joint_name] =
            std::make_shared<XBot::JointImpedanceController>( _model->GetJoint(gazebo_joint_name) );

        _joint_controller_map.at(gazebo_joint_name)->setGains(1000, 0, 1);
        _joint_controller_map.at(gazebo_joint_name)->enableFeedforward();

        std::cout << "Joint # " << gazebo_joint << " - " << gazebo_joint_name << std::endl;

    }

}




void gazebo::GazeboXBotPlugin::Init()
{
    gazebo::ModelPlugin::Init();
    std::cout << "GazeboXBotPlugin Init() started" << std::endl;

    // Load plugins
    loadPlugins();

    // Init plugins
    initPlugins();

    std::cout << "GazeboXBotPlugin Init() completed" << std::endl;


}

bool gazebo::GazeboXBotPlugin::loadPlugins()
{
    return _pluginHandler->load_plugins();
}

bool gazebo::GazeboXBotPlugin::initPlugins()
{

    return _pluginHandler->init_plugins();
}

void gazebo::GazeboXBotPlugin::close_all()
{
    std::cout << "void gazebo::GazeboXBotPlugin::close_all()" << std::endl;
    
    _pluginHandler->close();

}

double gazebo::GazeboXBotPlugin::get_time()
{
    return _world->GetSimTime().Double();
}



void gazebo::GazeboXBotPlugin::XBotUpdate(const common::UpdateInfo & _info)
{
    if( g_loop_ok == 0 ){
        std::cout << "CTRL+C detected..." << std::endl;
        close_all();
        exit(1);
    }
    
    _pluginHandler->run();

    for( auto& pair : _joint_controller_map ){
        pair.second->sendControlInput();
    }
}

void gazebo::GazeboXBotPlugin::Reset()
{
    gazebo::ModelPlugin::Reset();
    std::cout << "Reset()" << std::endl;
}



bool gazebo::GazeboXBotPlugin::get_link_pos ( int joint_id, double& link_pos )
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _jointMap.find(current_joint_name);

    if(current_joint_name != "" && it != _jointMap.end()) {
        link_pos = (it->second)->GetAngle(0).Radian();
        return true;
    }
    else {
        link_pos = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::get_aux ( int joint_id, double& aux )
{
    aux = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_fault ( int joint_id, double& fault )
{
    fault = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_link_vel ( int joint_id, double& link_vel )
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _jointMap.find(current_joint_name);

    if(current_joint_name != "" && it != _jointMap.end()) {
        link_vel = (it->second)->GetVelocity(0)*1000.;
        return true;
    }
    else {
        link_vel = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::get_temperature ( int joint_id, double& temperature )
{
    temperature = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_gains(int joint_id, std::vector< double >& gain_vector)
{

    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _joint_controller_map.find(current_joint_name);


    if(current_joint_name != "" && it != _joint_controller_map.end()){
        if(gain_vector.size() < 2){
            gain_vector.assign(2, 0);
        }
        gain_vector[0] = it->second->getP();
        gain_vector[1] = it->second->getD();

        return true;
    }

    return false;
}


bool gazebo::GazeboXBotPlugin::get_motor_pos ( int joint_id, double& motor_pos )
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _jointMap.find(current_joint_name);

    if(current_joint_name != "" && it != _jointMap.end()) {
        motor_pos = (it->second)->GetAngle(0).Radian();
        // NOTE we return false because we are reading the link position form gazebo TBD a plugin should simulate this
        return false;
    }
    else {
        motor_pos = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::get_motor_vel ( int joint_id, double& motor_vel )
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _jointMap.find(current_joint_name);

    if(current_joint_name != "" && it != _jointMap.end()) {
        motor_vel = (it->second)->GetVelocity(0)*1000.;
        // NOTE we return false because we are reading the link position form gazebo TBD a plugin should simulate this
        return false;
    }
    else {
        motor_vel = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::get_op_idx_ack ( int joint_id, double& op_idx_ack )
{
    op_idx_ack = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_rtt ( int joint_id, double& rtt )
{
    // TBD is it possible to get this info??
    rtt = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_torque ( int joint_id, double& torque )
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _jointMap.find(current_joint_name);

    if(current_joint_name != "" && it != _jointMap.end()) {
        torque = (it->second)->GetForce(0);
        return true;
    }
    else {
        torque = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::set_aux ( int joint_id, const double& aux )
{
    return false;
}

bool gazebo::GazeboXBotPlugin::set_fault_ack ( int joint_id, const double& fault_ack )
{
    return false;
}

bool gazebo::GazeboXBotPlugin::set_gains ( int joint_id, const std::vector< double >& gains )
{
    if(gains.size() < 2){
        std::cerr << "ERROR in " << __func__ << "! Provided gains vector size is " << gains.size() << " less than 2!" << std::endl;
        return false;
    }

    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _joint_controller_map.find(current_joint_name);

    if(current_joint_name != "" && it != _joint_controller_map.end()) {
        it->second->setGains(gains[0], 0, gains[1]);
        return true;
    }

    return false;
}

bool gazebo::GazeboXBotPlugin::set_op_idx_aux ( int joint_id, const double& op_idx_aux )
{
    return false;
}

bool gazebo::GazeboXBotPlugin::set_pos_ref ( int joint_id, const double& pos_ref )
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _joint_controller_map.find(current_joint_name);

    if(current_joint_name != "" && it != _joint_controller_map.end()) {
        it->second->setPositionReference(pos_ref);
        return true;
    }

    return false;
}

bool gazebo::GazeboXBotPlugin::set_tor_ref ( int joint_id, const double& tor_ref )
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _joint_controller_map.find(current_joint_name);

    if(current_joint_name != "" && it != _joint_controller_map.end()) {
        it->second->setTorqueReference(double(tor_ref) / 100); // NOTE torque scaling random but suitable to avoid the int16t overflow
        return true;
    }

    return false;
}

bool gazebo::GazeboXBotPlugin::set_ts ( int joint_id, const double& ts )
{
    return false;
}

bool gazebo::GazeboXBotPlugin::set_vel_ref ( int joint_id, const double& vel_ref )
{
    // TBD support velocity reference

//     std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
//     auto it = _jointMap.find(current_joint_name);
//
//     if(current_joint_name != "" && it != _jointMap.end()) {
//         _model->GetJointController()->SetVelocityTarget((it->second)->GetScopedName(), vel_ref);
//         _model->GetJointController()->Update();
//         return true;
//     }

    return false;

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




/*
///////////////////////////////
///////////////////////////////
// ROBOT PROTECTED FUNCTIONS //
///////////////////////////////
///////////////////////////////

bool gazebo::GazeboXBotPlugin::get_robot_link_pos(std::map< std::string, float >& link_pos)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_link_pos(c.first, link_pos);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_link_pos(std::map< int, float >& link_pos)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_link_pos(c.first, link_pos);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_motor_pos(std::map< std::string, float >& motor_pos)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_motor_pos(c.first, motor_pos);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_motor_pos(std::map< int, float >& motor_pos)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_motor_pos(c.first, motor_pos);
    }
    return ret;
}


bool gazebo::GazeboXBotPlugin::get_robot_link_vel(std::map< std::string, int16_t >& link_vel)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_link_vel(c.first, link_vel);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_link_vel(std::map< int, int16_t >& link_vel)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_link_vel(c.first, link_vel);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_motor_vel(std::map< std::string, int16_t >& motor_vel)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_motor_vel(c.first, motor_vel);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_motor_vel(std::map< int, int16_t >& motor_vel)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_motor_vel(c.first, motor_vel);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_torque(std::map< std::string, float >& torque)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_torque(c.first, torque);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_torque(std::map< int, float >& torque)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_torque(c.first, torque);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_temperature(std::map< std::string, uint16_t >& temperature)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_temperature(c.first, temperature);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_temperature(std::map< int, uint16_t >& temperature)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_temperature(c.first, temperature);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_fault(std::map< std::string, uint16_t >& fault)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_fault(c.first, fault);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_fault(std::map< int, uint16_t >& fault)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_fault(c.first, fault);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_rtt(std::map< std::string, uint16_t >& rtt)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_rtt(c.first, rtt);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_rtt(std::map< int, uint16_t >& rtt)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_rtt(c.first, rtt);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_op_idx_ack(std::map< std::string, uint16_t >& op_idx_ack)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_op_idx_ack(c.first, op_idx_ack);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_op_idx_ack(std::map< int, uint16_t >& op_idx_ack)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_op_idx_ack(c.first, op_idx_ack);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_aux(std::map< std::string, float >& aux)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_aux(c.first, aux);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_aux(std::map< int, float >& aux)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_aux(c.first, aux);
    }
    return ret;
}




bool gazebo::GazeboXBotPlugin::set_robot_pos_ref(const std::map< std::string, float >& pos_ref)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_pos_ref(c.first, pos_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_pos_ref(const std::map< int, float >& pos_ref)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_pos_ref(c.first, pos_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_vel_ref(const std::map< std::string, int16_t >& vel_ref)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_vel_ref(c.first, vel_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_vel_ref(const std::map< int, int16_t >& vel_ref)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_vel_ref(c.first, vel_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_tor_ref(const std::map< std::string, int16_t >& tor_ref)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_tor_ref(c.first, tor_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_tor_ref(const std::map< int, int16_t >& tor_ref)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_tor_ref(c.first, tor_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_gains(const std::map< std::string, std::vector< uint16_t > >& gains)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_gains(c.first, gains);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_gains(const std::map< int, std::vector< uint16_t > >& gains)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_gains(c.first, gains);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_fault_ack(const std::map< std::string, int16_t >& fault_ack)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_fault_ack(c.first, fault_ack);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_fault_ack(const std::map< int, int16_t >& fault_ack)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_fault_ack(c.first, fault_ack);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_ts(const std::map< std::string, uint16_t >& ts)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_ts(c.first, ts);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_ts(const std::map< int, uint16_t >& ts)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_ts(c.first, ts);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_op_idx_aux(const std::map< std::string, uint16_t >& op_idx_aux)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_op_idx_aux(c.first, op_idx_aux);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_op_idx_aux(const std::map< int, uint16_t >& op_idx_aux)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_op_idx_aux(c.first, op_idx_aux);
    }
    return ret;
}


bool gazebo::GazeboXBotPlugin::set_robot_aux(const std::map< std::string, float >& aux)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_aux(c.first, aux);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_aux(const std::map< int, float >& aux)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= set_chain_aux(c.first, aux);
    }
    return ret;
}
//

















///////////////////////////////
///////////////////////////////
// CHAIN PROTECTED FUNCTIONS //
///////////////////////////////
///////////////////////////////

bool gazebo::GazeboXBotPlugin::get_chain_link_pos(std::string chain_name, std::map< std::string, float>& link_pos)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            link_pos[actual_joint_name] = 0;
            if( !get_link_pos(actual_chain_enabled_joints[i], link_pos.at(actual_joint_name)))  {
                printf("ERROR: get_chain_link_pos() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_link_pos() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_link_pos(std::string chain_name, std::map< int, float >& link_pos)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            link_pos[actual_chain_enabled_joints[i]] = 0;
            if( !get_link_pos(actual_chain_enabled_joints[i], link_pos.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_link_pos() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_link_pos() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_motor_pos(std::string chain_name, std::map< std::string, float >& motor_pos)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            motor_pos[actual_joint_name] = 0;
            if( !get_motor_pos(actual_chain_enabled_joints[i], motor_pos.at(actual_joint_name)))  {
                printf("ERROR: get_chain_motor_pos() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_motor_pos() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_motor_pos(std::string chain_name, std::map< int, float >& motor_pos)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            motor_pos[actual_chain_enabled_joints[i]] = 0;
            if( !get_motor_pos(actual_chain_enabled_joints[i], motor_pos.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_motor_pos() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_motor_pos() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_link_vel(std::string chain_name, std::map< std::string, int16_t >& link_vel)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            link_vel[actual_joint_name] = 0;
            if( !get_link_vel(actual_chain_enabled_joints[i], link_vel.at(actual_joint_name)))  {
                printf("ERROR: get_chain_link_vel() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_link_vel() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_link_vel(std::string chain_name, std::map< int, int16_t >& link_vel)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            link_vel[actual_chain_enabled_joints[i]] = 0;
            if( !get_link_vel(actual_chain_enabled_joints[i], link_vel.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_link_vel() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_link_vel() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_motor_vel(std::string chain_name, std::map< std::string, int16_t >& motor_vel)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            motor_vel[actual_joint_name] = 0;
            if( !get_motor_vel(actual_chain_enabled_joints[i], motor_vel.at(actual_joint_name)))  {
                printf("ERROR: get_chain_motor_vel() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_motor_vel() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_motor_vel(std::string chain_name, std::map< int, int16_t >& motor_vel)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            motor_vel[actual_chain_enabled_joints[i]] = 0;
            if( !get_motor_vel(actual_chain_enabled_joints[i], motor_vel.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_motor_vel() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_motor_vel() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_torque(std::string chain_name, std::map< std::string, float >& torque)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            torque[actual_joint_name] = 0;
            if( !get_torque(actual_chain_enabled_joints[i], torque.at(actual_joint_name)))  {
                printf("ERROR: get_chain_torque() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_torque() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_torque(std::string chain_name, std::map< int, float >& torque)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            torque[actual_chain_enabled_joints[i]] = 0;
            if( !get_torque(actual_chain_enabled_joints[i], torque.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_torque() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_torque() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_temperature(std::string chain_name, std::map< std::string, uint16_t >& temperature)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            temperature[actual_joint_name] = 0;
            if( !get_temperature(actual_chain_enabled_joints[i], temperature.at(actual_joint_name)))  {
                printf("ERROR: get_chain_temperature() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_temperature() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_temperature(std::string chain_name, std::map< int, uint16_t >& temperature)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            temperature[actual_chain_enabled_joints[i]] = 0;
            if( !get_temperature(actual_chain_enabled_joints[i], temperature.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_temperature() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_temperature() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_fault(std::string chain_name, std::map< std::string, uint16_t >& fault)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            fault[actual_joint_name] = 0;
            if( !get_fault(actual_chain_enabled_joints[i], fault.at(actual_joint_name)))  {
                printf("ERROR: get_chain_fault() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_fault() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_fault(std::string chain_name, std::map< int, uint16_t >& fault)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            fault[actual_chain_enabled_joints[i]] = 0;
            if( !get_fault(actual_chain_enabled_joints[i], fault.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_fault() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_fault() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_rtt(std::string chain_name, std::map< std::string, uint16_t >& rtt)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            rtt[actual_joint_name] = 0;
            if( !get_rtt(actual_chain_enabled_joints[i], rtt.at(actual_joint_name)))  {
                printf("ERROR: get_chain_rtt() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_rtt() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_rtt(std::string chain_name, std::map< int, uint16_t >& rtt)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            rtt[actual_chain_enabled_joints[i]] = 0;
            if( !get_rtt(actual_chain_enabled_joints[i], rtt.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_rtt() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_rtt() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_op_idx_ack(std::string chain_name, std::map< std::string, uint16_t >& op_idx_ack)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            op_idx_ack[actual_joint_name] = 0;
            if( !get_op_idx_ack(actual_chain_enabled_joints[i], op_idx_ack.at(actual_joint_name)))  {
                printf("ERROR: get_chain_op_idx_ack() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_op_idx_ack() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_op_idx_ack(std::string chain_name, std::map< int, uint16_t >& op_idx_ack)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            op_idx_ack[actual_chain_enabled_joints[i]] = 0;
            if( !get_op_idx_ack(actual_chain_enabled_joints[i], op_idx_ack.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_op_idx_ack() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_op_idx_ack() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_aux(std::string chain_name, std::map< std::string, float >& aux)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            aux[actual_joint_name] = 0;
            if( !get_aux(actual_chain_enabled_joints[i], aux.at(actual_joint_name)))  {
                printf("ERROR: get_chain_aux() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_aux() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_aux(std::string chain_name, std::map< int, float >& aux)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            aux[actual_chain_enabled_joints[i]] = 0;
            if( !get_aux(actual_chain_enabled_joints[i], aux.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_aux() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }

    printf("ERROR: get_chain_aux() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}




bool gazebo::GazeboXBotPlugin::set_chain_pos_ref(std::string chain_name, const std::map< std::string, float >& pos_ref)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            if(pos_ref.count(actual_joint_name)) {
                if( !set_pos_ref(actual_chain_enabled_joints[i], pos_ref.at(actual_joint_name)))  {
                    printf("ERROR: set_pos_ref() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_pos_ref() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_pos_ref(std::string chain_name, const std::map< int, float >& pos_ref)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(pos_ref.count(actual_chain_enabled_joints[i])) {
                if( !set_pos_ref(actual_chain_enabled_joints[i], pos_ref.at(actual_chain_enabled_joints[i])))  {
                    printf("ERROR: set_pos_ref() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_pos_ref() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_vel_ref(std::string chain_name, const std::map< std::string, int16_t >& vel_ref)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            if(vel_ref.count(actual_joint_name)) {
                if( !set_vel_ref(actual_chain_enabled_joints[i], vel_ref.at(actual_joint_name)))  {
                    printf("ERROR: set_vel_ref() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_vel_ref() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_vel_ref(std::string chain_name, const std::map< int, int16_t >& vel_ref)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(vel_ref.count(actual_chain_enabled_joints[i])) {
                if( !set_vel_ref(actual_chain_enabled_joints[i], vel_ref.at(actual_chain_enabled_joints[i])))  {
                    printf("ERROR: set_vel_ref() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_vel_ref() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_tor_ref(std::string chain_name, const std::map< std::string, int16_t >& tor_ref)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            if(tor_ref.count(actual_joint_name)) {
                if( !set_tor_ref(actual_chain_enabled_joints[i], tor_ref.at(actual_joint_name)))  {
                    printf("ERROR: set_tor_ref() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_tor_ref() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_tor_ref(std::string chain_name, const std::map< int, int16_t >& tor_ref)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(tor_ref.count(actual_chain_enabled_joints[i])) {
                if( !set_tor_ref(actual_chain_enabled_joints[i], tor_ref.at(actual_chain_enabled_joints[i])))  {
                    printf("ERROR: set_tor_ref() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_tor_ref() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_gains(std::string chain_name, const std::map< std::string, std::vector<uint16_t> >& gains)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            if(gains.count(actual_joint_name)) {
                if( !set_gains(actual_chain_enabled_joints[i], gains.at(actual_joint_name)))  {
                    printf("ERROR: set_chain_gains() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_pos_ref() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_gains(std::string chain_name, const std::map< int, std::vector<uint16_t> >& gains)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(gains.count(actual_chain_enabled_joints[i])) {
                if( !set_gains(actual_chain_enabled_joints[i], gains.at(actual_chain_enabled_joints[i])))  {
                    printf("ERROR: set_gains() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_gains() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_fault_ack(std::string chain_name, const std::map< std::string, int16_t >& fault_ack)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            if(fault_ack.count(actual_joint_name)) {
                if( !set_fault_ack(actual_chain_enabled_joints[i], fault_ack.at(actual_joint_name)))  {
                    printf("ERROR: set_fault_ack() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_fault_ack() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_fault_ack(std::string chain_name, const std::map< int, int16_t >& fault_ack)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(fault_ack.count(actual_chain_enabled_joints[i])) {
                if( !set_fault_ack(actual_chain_enabled_joints[i], fault_ack.at(actual_chain_enabled_joints[i])))  {
                    printf("ERROR: set_fault_ack() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_fault_ack() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_ts(std::string chain_name, const std::map< std::string, uint16_t >& ts)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            if(ts.count(actual_joint_name)) {
                if( !set_ts(actual_chain_enabled_joints[i], ts.at(actual_joint_name)))  {
                    printf("ERROR: set_ts() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_ts() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_ts(std::string chain_name, const std::map< int, uint16_t >& ts)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(ts.count(actual_chain_enabled_joints[i])) {
                if( !set_ts(actual_chain_enabled_joints[i], ts.at(actual_chain_enabled_joints[i])))  {
                    printf("ERROR: set_ts() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_ts() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_op_idx_aux(std::string chain_name, const std::map< std::string, uint16_t >& op_idx_aux)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            if(op_idx_aux.count(actual_joint_name)) {
                if( !set_op_idx_aux(actual_chain_enabled_joints[i], op_idx_aux.at(actual_joint_name)))  {
                    printf("ERROR: set_op_idx_aux() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_op_idx_aux() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_op_idx_aux(std::string chain_name, const std::map< int, uint16_t >& op_idx_aux)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(op_idx_aux.count(actual_chain_enabled_joints[i])) {
                if( !set_op_idx_aux(actual_chain_enabled_joints[i], op_idx_aux.at(actual_chain_enabled_joints[i])))  {
                    printf("ERROR: set_op_idx_aux() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_op_idx_aux() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_aux(std::string chain_name, const std::map< std::string, float >& aux)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _robot->getJointByID(actual_chain_enabled_joints[i])->getJointName();
            if(aux.count(actual_joint_name)) {
                if( !set_aux(actual_chain_enabled_joints[i], aux.at(actual_joint_name)))  {
                    printf("ERROR: set_aux() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_aux() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_aux(std::string chain_name, const std::map< int, float >& aux)
{
    if( _robot->hasChain(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(aux.count(actual_chain_enabled_joints[i])) {
                if( !set_aux(actual_chain_enabled_joints[i], aux.at(actual_chain_enabled_joints[i])))  {
                    printf("ERROR: set_aux() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }

    printf("ERROR: set_chain_aux() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}*/
