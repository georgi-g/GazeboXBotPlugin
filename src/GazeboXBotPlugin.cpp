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
#include <GazeboXBotPlugin/JointImpedanceController.h>

#include <gazebo/sensors/sensors.hh>



sig_atomic_t g_loop_ok = 1;

void sigint_handler(int s){
    g_loop_ok = 0;
}


gazebo::GazeboXBotPlugin::GazeboXBotPlugin()
{
    std::cout << "GazeboXBotPlugin()" << std::endl;
    _shared_memory = std::make_shared<XBot::SharedMemory>();
    signal(SIGINT, sigint_handler);

}


gazebo::GazeboXBotPlugin::~GazeboXBotPlugin()
{
    std::cerr << "~GazeboXBotPlugin()" << std::endl;
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

    YAML::Node root = YAML::LoadFile(_path_to_config);

    // create robot from config file and any map
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint(this, [](XBot::IXBotJoint* p){});
    std::shared_ptr<XBot::IXBotFT> xbot_ft(this, [](XBot::IXBotFT* p){});
    std::shared_ptr<XBot::IXBotIMU> xbot_imu(this, [](XBot::IXBotIMU* p){});
    std::shared_ptr<XBot::IXBotHand> xbot_hand(this, [](XBot::IXBotHand* p){});
    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);
    (*anymap)["XBotIMU"] = boost::any(xbot_imu);
    (*anymap)["XBotHand"] = boost::any(xbot_hand);
    
    std::cout<<"BEFORE"<<std::endl;

    _robot = XBot::RobotInterface::getRobot(_path_to_config, anymap, "XBotRT");
    
    std::cout<<"after"<<std::endl;

    // create time provider function
    boost::function<double()> time_func = boost::bind(&gazebo::GazeboXBotPlugin::get_time, this);
    // create time provider
    auto time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);

    // create plugin handler
    _pluginHandler = std::make_shared<XBot::PluginHandler>(_robot, time_provider, "XBotRTPlugins");

    // iterate over Gazebo model Joint vector and store Joint pointers in a map
    const gazebo::physics::Joint_V & gazebo_models_joints = _model->GetJoints();
    for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size(); gazebo_joint++) {
        std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();

        if(std::find(_robot->getEnabledJointNames().begin(),
                  _robot->getEnabledJointNames().end(), gazebo_joint_name) ==
                _robot->getEnabledJointNames().end())
        {
            std::cout << gazebo_joint_name<<" is not present in the list of enabled joints, ";
            std::cout << " therefore will not be controlled by XBot!" << std::endl;
            continue;
        }


        _jointNames.push_back(gazebo_joint_name);
        _jointMap[gazebo_joint_name] = _model->GetJoint(gazebo_joint_name);

        _joint_controller_map[gazebo_joint_name] =
            std::make_shared<XBot::JointImpedanceController>( _model->GetJoint(gazebo_joint_name) );

            double p_gain = 300;
            double d_gain = 1;

        if( root["GazeboXBotPlugin"]["gains"][gazebo_joint_name] ){
            p_gain = root["GazeboXBotPlugin"]["gains"][gazebo_joint_name]["p"].as<double>();
            d_gain = root["GazeboXBotPlugin"]["gains"][gazebo_joint_name]["d"].as<double>();
        }


        _joint_controller_map.at(gazebo_joint_name)->setGains(p_gain, 0, d_gain);

        _joint_controller_map.at(gazebo_joint_name)->enableFeedforward();

        std::cout << "Joint # " << gazebo_joint << " - " << gazebo_joint_name << std::endl;

    }

    // set the control rate
    if(root["GazeboXBotPlugin"]["control_rate"]){
        _control_rate = root["GazeboXBotPlugin"]["control_rate"].as<double>();
    }
    else{
        // defaulte control rate at 1ms
        _control_rate = 0.001;
    }
    
    // init and update sensors
//     gazebo::sensors::SensorManager::Instance()->Update(true);
    
    // get the list of sensors
    _sensors = gazebo::sensors::SensorManager::Instance()->GetSensors();
    
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
    loadFTSensors();

    // load IMU sensors
    loadImuSensors();

}

bool gazebo::GazeboXBotPlugin::loadFTSensors()
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    bool ret = true;
    std::cout << "Loading F-T sensors ... " << std::endl;
    
    for( const auto& FT_pair : _robot->getForceTorque() ) {

        // check XBot FT ptr
        if(!FT_pair.second){
            std::cout << "ERROR! FT NULLPTR!!!" << std::endl;
            continue;
        }

        // get id and name of the FT sensor
        const XBot::ForceTorqueSensor& ft = *FT_pair.second;
        int ft_id = ft.getSensorId();
        std::string ft_name = ft.getSensorName();

        for(unsigned int i = 0; i < _sensors_attached_to_robot.size(); ++i) {
            #if GAZEBO_MAJOR_VERSION <= 6
            // if the sensor is a FT and has the ft_name
            if( ( _sensors_attached_to_robot[i]->GetType().compare("force_torque") == 0 ) &&
                ( _sensors_attached_to_robot[i]->GetName() == ft_name ) )
            {
                _ft_gazebo_map[ft_id] = boost::static_pointer_cast<gazebo::sensors::ForceTorqueSensor>(_sensors_attached_to_robot[i]); 
                std::cout << "F-T found: " << _ft_gazebo_map.at(ft_id)->GetName() << std::endl;
            }
            #else 
            if( ( _sensors_attached_to_robot[i]->Type().compare("force_torque") == 0 ) &&
                ( _sensors_attached_to_robot[i]->Name() == ft_name ) )
            {
                _ft_gazebo_map[ft_id] = std::static_pointer_cast<gazebo::sensors::ForceTorqueSensor>(_sensors_attached_to_robot[i]); 
                std::cout << "F-T found: " << _ft_gazebo_map.at(ft_id)->Name() << std::endl;
            }
            #endif

        }
        
    }

    return ret;
}

bool gazebo::GazeboXBotPlugin::loadImuSensors()
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    bool ret = true;
    std::cout << "Loading IMU sensors ... " << std::endl;
    
    for( const auto& imu_pair : _robot->getImu() ) {

        // check XBot FT ptr
        if(!imu_pair.second){
            std::cout << "ERROR! imu NULLPTR!!!" << std::endl;
            continue;
        }

        // get id and name of the FT sensor
        const XBot::ImuSensor& imu = *imu_pair.second;
        int imu_id = imu.getSensorId();
        std::string imu_name = imu.getSensorName();

        for(unsigned int i = 0; i < _sensors_attached_to_robot.size(); ++i) {
            #if GAZEBO_MAJOR_VERSION <= 6
            // if the sensor is a IMU and has the ft_name
            if( ( _sensors_attached_to_robot[i]->GetType().compare("imu") == 0 ) &&
                ( _sensors_attached_to_robot[i]->GetName() == imu_name ) )
            {
                _imu_gazebo_map[imu_id] = boost::static_pointer_cast<gazebo::sensors::ImuSensor>(_sensors_attached_to_robot[i]); 
                std::cout << "IMU found: " << _imu_gazebo_map.at(imu_id)->GetName() << std::endl;
            }
            #else 
            if( ( _sensors_attached_to_robot[i]->Type().compare("imu") == 0 ) &&
                ( _sensors_attached_to_robot[i]->Name() == imu_name ) )
            {
                _imu_gazebo_map[imu_id] = std::static_pointer_cast<gazebo::sensors::ImuSensor>(_sensors_attached_to_robot[i]); 
                std::cout << "IMU found: " << _imu_gazebo_map.at(imu_id)->Name() << std::endl;
            }
            #endif

        }
        
    }
    
    return ret;
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
    
    initGrasping();

    std::cout << "GazeboXBotPlugin Init() completed" << std::endl;


}

bool gazebo::GazeboXBotPlugin::loadPlugins()
{
    return _pluginHandler->load_plugins();
}

bool gazebo::GazeboXBotPlugin::initPlugins()
{
    std::shared_ptr<XBot::IXBotJoint> xbot_joint(this);
    return _pluginHandler->init_plugins(_shared_memory, xbot_joint);
}

void gazebo::GazeboXBotPlugin::initGrasping()
{
    
    int argc = 1;
    const char *arg = "MagneticGrasping";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;

    if(!ros::isInitialized()){
        ros::init(argc, argv, "MagneticGrasping");
    }
    
    _nh = std::make_shared<ros::NodeHandle>();
    //_robot->get
    _grasp = _nh->advertise<std_msgs::Bool>("/grasp/RWrMot3/autoGrasp",1); //TBD take the general link
    
     std::cout << "INITGRASPING()" << std::endl;
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
        link_vel = (it->second)->GetVelocity(0);
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
        // NOTE we return true but we are reading the link position from gazebo TBD a plugin should simulate this
        return true;
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
        motor_vel = (it->second)->GetVelocity(0);
        // NOTE we return false but we are reading the link position drom gazebo TBD a plugin should simulate this
        return true;
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

bool gazebo::GazeboXBotPlugin::get_pos_ref(int joint_id, double& pos_ref)
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _joint_controller_map.find(current_joint_name);

    if(current_joint_name != "" && it != _joint_controller_map.end()) {
        pos_ref = it->second->getPositionReference();
        return true;
    }

    return false;
}


bool gazebo::GazeboXBotPlugin::get_vel_ref(int joint_id, double& vel_ref)
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _joint_controller_map.find(current_joint_name);

    if(current_joint_name != "" && it != _joint_controller_map.end()) {
        vel_ref = it->second->getVelocityReference();
        return true;
    }

    return false;
}


bool gazebo::GazeboXBotPlugin::get_tor_ref(int joint_id, double& tor_ref)
{
    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    auto it = _joint_controller_map.find(current_joint_name);

    if(current_joint_name != "" && it != _joint_controller_map.end()) {
        tor_ref = it->second->getTorqueReference();
        return true;
    }

    return false;
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
        it->second->setTorqueReference(double(tor_ref)); // NOTE torque scaling random but suitable to avoid the int16t overflow
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

bool gazebo::GazeboXBotPlugin::get_imu(int imu_id,
                                       std::vector< double >& lin_acc,
                                       std::vector< double >& ang_vel,
                                       std::vector< double >& quaternion)
{

    auto it =_imu_gazebo_map.find(imu_id);
    // if imu is not found
    if( it == _imu_gazebo_map.end() ) {
        std::cout << "WARNING: IMU with id : " << imu_id << " not found in the GAZEBO model you loaded: check you put the right urdf_path, srdf_path and joint_config_map in your YAML config file." << std::endl;
        return false;
    }

    // imu found
    auto imu_gazebo = it->second;
    
    lin_acc.assign(3, 0.0);
    ang_vel.assign(3, 0.0);
    quaternion.assign(4, 0.0);
    quaternion[3] = 1.0;

    if(!imu_gazebo){
        lin_acc.assign(3, 0.0);
        ang_vel.assign(3, 0.0);
        quaternion.assign(4, 0.0);
        quaternion[3] = 1.0;

        return false;
    }

    lin_acc[0] = imu_gazebo->LinearAcceleration().X();
    lin_acc[1] = imu_gazebo->LinearAcceleration().Y();
    lin_acc[2] = imu_gazebo->LinearAcceleration().Z();

    ang_vel[0] = imu_gazebo->AngularVelocity().X();
    ang_vel[1] = imu_gazebo->AngularVelocity().Y();
    ang_vel[2] = imu_gazebo->AngularVelocity().Z();

    quaternion[0] = imu_gazebo->Orientation().X();
    quaternion[1] = imu_gazebo->Orientation().Y();
    quaternion[2] = imu_gazebo->Orientation().Z();
    quaternion[3] = imu_gazebo->Orientation().W();
    
    return true;


}

bool gazebo::GazeboXBotPlugin::get_imu_fault(int imu_id, double& fault)
{
    auto imu_ptr = _robot->getImu(imu_id);

    if(!imu_ptr){
        fault = 0;
        return false;
    }

    fault = 0;
    return true;
}

bool gazebo::GazeboXBotPlugin::get_imu_rtt(int imu_id, double& rtt)
{
    auto imu_ptr = _robot->getImu(imu_id);

    if(!imu_ptr){
        rtt = 0;
        return false;
    }

    rtt = 0;
    return true;
}


bool gazebo::GazeboXBotPlugin::get_ft(int ft_id, std::vector< double >& ft, int channels)
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

bool gazebo::GazeboXBotPlugin::get_ft_fault(int ft_id, double& fault)
{
    auto ft_ptr = _robot->getForceTorque(ft_id);

    if(!ft_ptr){
        fault = 0;
        return false;
    }

    fault = 0;
    return true;
}

bool gazebo::GazeboXBotPlugin::get_ft_rtt(int ft_id, double& rtt)
{
    auto ft_ptr = _robot->getForceTorque(ft_id);

    if(!ft_ptr){
        rtt = 0;
        return false;
    }

    rtt = 0;
    return true;
}


bool gazebo::GazeboXBotPlugin::grasp(int hand_id, double grasp_percentage)
{
   std::cout << "GRASP" << std::endl;
   
  std_msgs::Bool message;
  if(grasp_percentage == 0)
    message.data = false;
  else
    message.data = true;
  std::cout<<"GRASP_COMMAND"<<message.data<<std::endl;
  _grasp.publish (message);
  
  return true;
}
    
double gazebo::GazeboXBotPlugin::get_grasp_state(int hand_id)
{
  //TBD get from outside (ask to implement as a service)
   //std::cout << "GRASPSTATE" << std::endl;
   
   return 0;
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



