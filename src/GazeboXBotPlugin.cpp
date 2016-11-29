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


#include<GazeboXBotPlugin/GazeboXBotPlugin.h>

#include<iostream>

gazebo::GazeboXBotPlugin::GazeboXBotPlugin()
{
    std::cout << "GazeboXBotPlugin()" << std::endl;
}


gazebo::GazeboXBotPlugin::~GazeboXBotPlugin()
{
    std::cout << "~GazeboXBotPlugin()" << std::endl;
    
    for( const auto& plugin : _rtplugin_vector ){
        (*plugin)->close();
    }
    
    
}

void gazebo::GazeboXBotPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::cout << "GazeboXBotPlugin Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)" << std::endl;

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
    
    computeAbsolutePath(_path_to_config, "/", _path_to_config);

}
    



void gazebo::GazeboXBotPlugin::Init()
{
    gazebo::ModelPlugin::Init();
    std::cout << "GazeboXBotPlugin Init()" << std::endl;     
        
    std::string path_to_cfg("/home/alaurenzi/Code/robotology-superbuild/configs/ADVR_shared/centauro/configs/config_centauro-rt_upperbody.yaml");
    
    path_to_cfg = _path_to_config;
    
    // init XBotCoreModel
    // parse the YAML file to initialize internal variables
    parseYAML(path_to_cfg); 
    
    // Load plugins
    loadPlugins();
    
    // Init plugins
    initPlugins();
    
    // initialize the model
    if (!_XBotModel.init(_urdf_path, _srdf_path, _joint_map_config)) {
        printf("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n"); 
        return;
    }
    // generate the robot
    _XBotModel.generate_robot();
    // get the map of joint
    _XBotRobot = _XBotModel.get_robot();
    
    // iterate over Gazebo model Joint vector and store Joint pointers in a map
    const gazebo::physics::Joint_V & gazebo_models_joints = _model->GetJoints();
    for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size(); gazebo_joint++) {
        std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
        _jointNames.push_back(gazebo_joint_name);
        _jointMap[gazebo_joint_name] = _model->GetJoint(gazebo_joint_name);
        gazebo::common::PID pid;
        pid.Init(600, 0, 3);
        
        _model->GetJointController()->SetPositionPID(_jointMap.at(gazebo_joint_name)->GetScopedName(), pid);
        
        std::cout << "Joint # " << gazebo_joint << " - " << gazebo_joint_name << std::endl;
        
    }

}

bool gazebo::GazeboXBotPlugin::loadPlugins()
{
    
    if(!_root_cfg["XBotRTPlugins"]){
        std::cerr << "ERROR in " << __func__ << "! Config file does NOT contain mandatory node XBotRTPlugins!" << std::endl;
        return false;
    }
    else{
        
        if(!_root_cfg["XBotRTPlugins"]["plugins"]){
            std::cerr << "ERROR in " << __func__ << "!XBotRTPlugins node does NOT contain mandatory node plugins!" << std::endl;
        return false;
        }
        else{
            
            for(const auto& plugin : _root_cfg["XBotRTPlugins"]["plugins"]){
                _rtplugin_names.push_back(plugin.as<std::string>());
            }
        }
        
    }
    

    
    bool success = true;
    
    for( const std::string& plugin_name : _rtplugin_names ){
        
        std::string path_to_so;
        computeAbsolutePath(plugin_name, "/build/install/lib/lib", path_to_so);
        path_to_so += std::string(".so");
        
        std::string factory_name = plugin_name + std::string("_factory");
        
        auto factory_ptr = std::make_shared<shlibpp::SharedLibraryClassFactory<XBot::XBotPlugin>>(path_to_so.c_str(), factory_name.c_str());
        
        if (!factory_ptr->isValid()) {
            // NOTE print to celebrate the wizard
            printf("error (%s) : %s\n", shlibpp::Vocab::decode(factory_ptr->getStatus()).c_str(),
                factory_ptr->getLastNativeError().c_str());
            std::cerr << "Unable to load plugin " << plugin_name << "!" << std::endl;
            success = false;
            continue;
        }
        else{
            std::cout << "Found plugin " << plugin_name << "!" << std::endl;
        }
        
        _rtplugin_factory.push_back(factory_ptr);
        
        auto plugin_ptr = std::make_shared<shlibpp::SharedLibraryClass<XBot::XBotPlugin>>(*factory_ptr);
        
        _rtplugin_vector.push_back(plugin_ptr);

    }

    return success;
    
    
    
}

bool gazebo::GazeboXBotPlugin::initPlugins()
{
    
    std::shared_ptr<XBot::IXBotModel> actual_model = std::make_shared<XBot::XBotCoreModel>(_XBotModel);
    std::shared_ptr<XBot::IXBotChain> actual_chain(this); // TBD ? [](XBot::IXBotChain* ptr){return;}
    std::shared_ptr<XBot::IXBotRobot> actual_robot(this);
    std::shared_ptr<XBot::IXBotFT> actual_ft;
    
    bool ret = true;
    for(int i = 0; i < _rtplugin_vector.size(); i++) {
        if(!(*_rtplugin_vector[i])->init( _path_to_config,
                                _rtplugin_names[i],
                                actual_model, 
                                actual_chain,
                                actual_robot,
                                actual_ft)) {
            printf("ERROR: plugin %s - init() failed\n", (*_rtplugin_vector[i])->name.c_str());
            ret = false;
        }
    }
    return ret;
}


void gazebo::GazeboXBotPlugin::XBotUpdate(const common::UpdateInfo & _info)
{
    for( const auto& plugin : _rtplugin_vector ){
        (*plugin)->run(0, -1); // TBD actual time
    }
}

void gazebo::GazeboXBotPlugin::Reset()
{
    gazebo::ModelPlugin::Reset();
    std::cout << "Reset()" << std::endl;     
}



bool gazebo::GazeboXBotPlugin::get_link_pos ( int joint_id, float& link_pos )
{
    std::string current_joint_name = _XBotModel.rid2Joint(joint_id);
    if(current_joint_name != "") {
        link_pos = _jointMap.at(current_joint_name)->GetAngle(0).Radian();
        return true;
    }
    else {
        link_pos = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::get_aux ( int joint_id, float& aux )
{
    aux = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_fault ( int joint_id, uint16_t& fault )
{
    fault = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_link_vel ( int joint_id, float& link_vel )
{
    std::string current_joint_name = _XBotModel.rid2Joint(joint_id);
    if(current_joint_name != "") {
        link_vel = _jointMap.at(current_joint_name)->GetVelocity(0);
        return true;
    }
    else {
        link_vel = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::get_max_temperature ( int joint_id, uint16_t& max_temperature )
{
    max_temperature = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_motor_pos ( int joint_id, float& motor_pos )
{
    std::string current_joint_name = _XBotModel.rid2Joint(joint_id);
    if(current_joint_name != "") {
        motor_pos = _jointMap.at(current_joint_name)->GetAngle(0).Radian();
        // NOTE we return false because we are reading the link position form gazebo TBD a plugin should simulate this
        return false;
    }
    else {
        motor_pos = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::get_motor_vel ( int joint_id, int16_t& motor_vel )
{
    std::string current_joint_name = _XBotModel.rid2Joint(joint_id);
    if(current_joint_name != "") {
        motor_vel = (int16_t) _jointMap.at(current_joint_name)->GetVelocity(0);
        // NOTE we return false because we are reading the link position form gazebo TBD a plugin should simulate this
        return false;
    }
    else {
        motor_vel = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::get_op_idx_ack ( int joint_id, uint16_t& op_idx_ack )
{
    op_idx_ack = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_rtt ( int joint_id, uint16_t& rtt )
{
    // TBD is it possible to get this info??
    rtt = 0;
    return false;
}

bool gazebo::GazeboXBotPlugin::get_torque ( int joint_id, int16_t& torque )
{
    std::string current_joint_name = _XBotModel.rid2Joint(joint_id);
    if(current_joint_name != "") {
        torque = (int16_t) _jointMap.at(current_joint_name)->GetForce(0);
        return true;
    }
    else {
        torque = 0;
        return false;
    }
}

bool gazebo::GazeboXBotPlugin::set_aux ( int joint_id, const float& aux )
{
    return false;
}

bool gazebo::GazeboXBotPlugin::set_fault_ack ( int joint_id, const int16_t& fault_ack )
{
    return false;
}

bool gazebo::GazeboXBotPlugin::set_gains ( int joint_id, const std::vector< uint16_t >& gains )
{
    return false;
}

bool gazebo::GazeboXBotPlugin::set_op_idx_aux ( int joint_id, const uint16_t& op_idx_aux )
{
    return false;
}

bool gazebo::GazeboXBotPlugin::set_pos_ref ( int joint_id, const float& pos_ref )
{
    std::string current_joint_name = _XBotModel.rid2Joint(joint_id);
    // TBD check PID
    
    
    _model->GetJointController()->SetPositionTarget(_jointMap.at(current_joint_name)->GetScopedName(), pos_ref);
    _model->GetJointController()->Update();
    

    return true;
}

bool gazebo::GazeboXBotPlugin::set_tor_ref ( int joint_id, const int16_t& tor_ref )
{
    std::string current_joint_name = _XBotModel.rid2Joint(joint_id);
    _jointMap.at(current_joint_name)->SetForce(0, tor_ref);
    return true;
}

bool gazebo::GazeboXBotPlugin::set_ts ( int joint_id, const uint16_t& ts )
{
    return false;
}

bool gazebo::GazeboXBotPlugin::set_vel_ref ( int joint_id, const int16_t& vel_ref )
{
    std::string current_joint_name = _XBotModel.rid2Joint(joint_id);
    _model->GetJointController()->SetVelocityTarget(_jointMap.at(current_joint_name)->GetScopedName(), vel_ref);
    _model->GetJointController()->Update();
    return true;
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

bool gazebo::GazeboXBotPlugin::parseYAML ( const std::string& path_to_cfg )
{
    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        std::cerr << "ERROR in " << __func__ << "! Can NOT open " << path_to_cfg << "!" << std::endl;
        return false;
    }
    

    _root_cfg = YAML::LoadFile(path_to_cfg);
    YAML::Node x_bot_interface;
    if(_root_cfg["XBotInterface"]) {
        x_bot_interface = _root_cfg["XBotInterface"]; 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << "  does not contain XBotInterface mandatory node!!" << std::endl;
        return false;
    }
   
    // check the urdf_filename
    if(x_bot_interface["urdf_path"]) {
        computeAbsolutePath(x_bot_interface["urdf_path"].as<std::string>(), 
                            "/",
                            _urdf_path); 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : XBotInterface node of  " << path_to_cfg << "  does not contain urdf_path mandatory node!!" << std::endl;
        return false;
    }
    
    // check the srdf_filename
    if(x_bot_interface["srdf_path"]) {
        computeAbsolutePath(x_bot_interface["srdf_path"].as<std::string>(),
                            "/",
                            _srdf_path); 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : XBotInterface node of  " << path_to_cfg << "  does not contain srdf_path mandatory node!!" << std::endl;
        return false;
    }
    
    // check joint_map_config
    if(x_bot_interface["joint_map_path"]) {
        computeAbsolutePath(x_bot_interface["joint_map_path"].as<std::string>(), 
                            "/",
                            _joint_map_config); 
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : XBotInterface node of  " << path_to_cfg << "  does not contain joint_map_path mandatory node!!" << std::endl;
        return false;
    }
    
    

}



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


bool gazebo::GazeboXBotPlugin::get_robot_link_vel(std::map< std::string, float >& link_vel)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_link_vel(c.first, link_vel);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_link_vel(std::map< int, float >& link_vel)
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

bool gazebo::GazeboXBotPlugin::get_robot_torque(std::map< std::string, int16_t >& torque)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_torque(c.first, torque);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_torque(std::map< int, int16_t >& torque)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_torque(c.first, torque);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_max_temperature(std::map< std::string, uint16_t >& max_temperature)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_max_temperature(c.first, max_temperature);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_max_temperature(std::map< int, uint16_t >& max_temperature)
{
    bool ret = true;
    for(auto& c : _XBotRobot) {
        ret &= get_chain_max_temperature(c.first, max_temperature);
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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

bool gazebo::GazeboXBotPlugin::get_chain_link_vel(std::string chain_name, std::map< std::string, float >& link_vel)
{
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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

bool gazebo::GazeboXBotPlugin::get_chain_link_vel(std::string chain_name, std::map< int, float >& link_vel)
{
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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

bool gazebo::GazeboXBotPlugin::get_chain_torque(std::string chain_name, std::map< std::string, int16_t >& torque)
{
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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

bool gazebo::GazeboXBotPlugin::get_chain_torque(std::string chain_name, std::map< int, int16_t >& torque)
{
    if( _XBotRobot.count(chain_name) ) {
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

bool gazebo::GazeboXBotPlugin::get_chain_max_temperature(std::string chain_name, std::map< std::string, uint16_t >& max_temperature)
{
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            max_temperature[actual_joint_name] = 0;
            if( !get_max_temperature(actual_chain_enabled_joints[i], max_temperature.at(actual_joint_name)))  {
                printf("ERROR: get_chain_max_temperature() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    printf("ERROR: get_chain_max_temperature() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_max_temperature(std::string chain_name, std::map< int, uint16_t >& max_temperature)
{
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            max_temperature[actual_chain_enabled_joints[i]] = 0;
            if( !get_max_temperature(actual_chain_enabled_joints[i], max_temperature.at(actual_chain_enabled_joints[i])))  {
                printf("ERROR: get_chain_max_temperature() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    printf("ERROR: get_chain_max_temperature() on chain %s, that does not exits in the _XBotRobot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_fault(std::string chain_name, std::map< std::string, uint16_t >& fault)
{
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
    if( _XBotRobot.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = _XBotRobot.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
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
    if( _XBotRobot.count(chain_name) ) {
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
}