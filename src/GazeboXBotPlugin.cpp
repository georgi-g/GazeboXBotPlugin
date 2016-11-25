#include<GazeboXBotPlugin/GazeboXBotPlugin.h>

#include<iostream>

// TBD do it dynamically
#include<XBotPlugin/RobotInterfaceXBotRT_test.h>

gazebo::GazeboXBotPlugin::GazeboXBotPlugin()
{
    std::cout << "GazeboXBotPlugin()" << std::endl;
}


gazebo::GazeboXBotPlugin::~GazeboXBotPlugin()
{
    std::cout << "~GazeboXBotPlugin()" << std::endl;
}

void gazebo::GazeboXBotPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    std::cout << "GazeboXBotPlugin Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)" << std::endl;

    // Store the pointer to the model
    _model = _parent;
    
    // get the world
    _world = _model->GetWorld();
      
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboXBotPlugin::XBotUpdate, this, _1));
    
   

}

void gazebo::GazeboXBotPlugin::Init()
{
    gazebo::ModelPlugin::Init();
    std::cout << "GazeboXBotPlugin Init()" << std::endl;     
        
    // init XBotCoreModel
    // parse the YAML file to initialize internal variables
    parseYAML("/home/lucamuratore/src/centauro-superbuild/configs/ADVR_shared/bigman/configs/config_walkman.yaml"); // TBD do it with plugin params
    
    // initialize the model
    if (!_XBotModel.init(_urdf_path, _srdf_path, _joint_map_config)) {
        printf("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n"); 
        return;
    }
    // generate the robot
    _XBotModel.generate_robot();
    // get the robot map
    robot_map = _XBotModel.get_robot();
    
    // iterate over model Joint vector and store Joint pointers in a map
    const gazebo::physics::Joint_V & gazebo_models_joints = _model->GetJoints();
    for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size(); gazebo_joint++) {
        std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
        _jointNames.push_back(gazebo_joint_name);
        _jointMap[gazebo_joint_name] = _model->GetJoint(gazebo_joint_name);
        std::cout << "Joint # " << gazebo_joint << " - " << gazebo_joint_name << std::endl;
    }
    
    // TBD load dynamically the plugins
    std::shared_ptr<XBot::IXBotModel> actual_model = std::make_shared<XBot::XBotCoreModel>(_XBotModel);
    std::shared_ptr<XBot::IXBotChain> actual_chain(this, [](XBot::IXBotChain* ptr){return;});
    std::shared_ptr<XBot::IXBotRobot> actual_robot(this, [](XBot::IXBotRobot* ptr){return;});
    std::shared_ptr<XBot::IXBotFT> actual_ft(this, [](XBot::IXBotFT* ptr){return;});
    std::shared_ptr<XBot::RobotInterfaceXBotRT_test> robot_interface_RT_plugin(new XBot::RobotInterfaceXBotRT_test( "robot_interface_RT_plugin",
                                                                                                                    actual_model, 
                                                                                                                    actual_chain,
                                                                                                                    actual_robot,
                                                                                                                    actual_ft));
    std::cout << "RobotInterfaceXBotRT_test constructed" << std::endl;    
    
    plugins.push_back(robot_interface_RT_plugin);
     
    // iterate over the plugins and call the init()
    bool ret = true;
    for(int i = 0; i < plugins.size(); i++) {
        if(!plugins[i]->init()) {
            printf("ERROR: plugin %s - init() failed\n", plugins[i]->name.c_str());
            ret = false;
        }
    }
    
    // NOTE no bool
    return;

}

void gazebo::GazeboXBotPlugin::XBotUpdate(const common::UpdateInfo & _info)
{
    // TBD run the RT plugins
    std::vector<float> plugin_execution_time(plugins.size()); // TBD circular array and write to file in the plugin_handler_close
    for(int i = 0; i < plugins.size(); i++) {
        float plugin_start_time = (get_time_ns() / 10e3); //microsec
        plugins[i]->run();
        plugin_execution_time[i] = (get_time_ns() / 10e3) - plugin_start_time; //microsec
//         DPRINTF("Plugin %d - %s : execution_time = %f microsec\n", i, plugins[i]->name.c_str(), plugin_execution_time[i]);
    }
    

    
//     set_pos_ref(11, std::cos(_world->GetSimTime().Double()));

}

void gazebo::GazeboXBotPlugin::Reset()
{
    gazebo::ModelPlugin::Reset();
    std::cout << "Reset()" << std::endl;     
    
    // TBD close plugins
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

///////////////////////////////
///////////////////////////////
// ROBOT PROTECTED FUNCTIONS //
///////////////////////////////
///////////////////////////////

bool gazebo::GazeboXBotPlugin::get_robot_link_pos(std::map< std::string, float >& link_pos)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_link_pos(c.first, link_pos);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_link_pos(std::map< int, float >& link_pos)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_link_pos(c.first, link_pos);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_motor_pos(std::map< std::string, float >& motor_pos)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_motor_pos(c.first, motor_pos);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_motor_pos(std::map< int, float >& motor_pos)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_motor_pos(c.first, motor_pos);
    }
    return ret;
}


bool gazebo::GazeboXBotPlugin::get_robot_link_vel(std::map< std::string, float >& link_vel)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_link_vel(c.first, link_vel);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_link_vel(std::map< int, float >& link_vel)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_link_vel(c.first, link_vel);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_motor_vel(std::map< std::string, int16_t >& motor_vel)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_motor_vel(c.first, motor_vel);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_motor_vel(std::map< int, int16_t >& motor_vel)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_motor_vel(c.first, motor_vel);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_torque(std::map< std::string, int16_t >& torque)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_torque(c.first, torque);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_torque(std::map< int, int16_t >& torque)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_torque(c.first, torque);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_max_temperature(std::map< std::string, uint16_t >& max_temperature)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_max_temperature(c.first, max_temperature);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_max_temperature(std::map< int, uint16_t >& max_temperature)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_max_temperature(c.first, max_temperature);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_fault(std::map< std::string, uint16_t >& fault)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_fault(c.first, fault);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_fault(std::map< int, uint16_t >& fault)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_fault(c.first, fault);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_rtt(std::map< std::string, uint16_t >& rtt)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_rtt(c.first, rtt);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_rtt(std::map< int, uint16_t >& rtt)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_rtt(c.first, rtt);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_op_idx_ack(std::map< std::string, uint16_t >& op_idx_ack)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_op_idx_ack(c.first, op_idx_ack);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_op_idx_ack(std::map< int, uint16_t >& op_idx_ack)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_op_idx_ack(c.first, op_idx_ack);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_aux(std::map< std::string, float >& aux)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_aux(c.first, aux);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::get_robot_aux(std::map< int, float >& aux)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= get_chain_aux(c.first, aux);
    }
    return ret;
}




bool gazebo::GazeboXBotPlugin::set_robot_pos_ref(const std::map< std::string, float >& pos_ref)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_pos_ref(c.first, pos_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_pos_ref(const std::map< int, float >& pos_ref)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_pos_ref(c.first, pos_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_vel_ref(const std::map< std::string, int16_t >& vel_ref)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_vel_ref(c.first, vel_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_vel_ref(const std::map< int, int16_t >& vel_ref)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_vel_ref(c.first, vel_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_tor_ref(const std::map< std::string, int16_t >& tor_ref)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_tor_ref(c.first, tor_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_tor_ref(const std::map< int, int16_t >& tor_ref)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_tor_ref(c.first, tor_ref);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_gains(const std::map< std::string, std::vector< uint16_t > >& gains)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_gains(c.first, gains);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_gains(const std::map< int, std::vector< uint16_t > >& gains)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_gains(c.first, gains);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_fault_ack(const std::map< std::string, int16_t >& fault_ack)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_fault_ack(c.first, fault_ack);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_fault_ack(const std::map< int, int16_t >& fault_ack)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_fault_ack(c.first, fault_ack);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_ts(const std::map< std::string, uint16_t >& ts)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_ts(c.first, ts);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_ts(const std::map< int, uint16_t >& ts)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_ts(c.first, ts);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_op_idx_aux(const std::map< std::string, uint16_t >& op_idx_aux)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_op_idx_aux(c.first, op_idx_aux);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_op_idx_aux(const std::map< int, uint16_t >& op_idx_aux)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_op_idx_aux(c.first, op_idx_aux);
    }
    return ret;
}


bool gazebo::GazeboXBotPlugin::set_robot_aux(const std::map< std::string, float >& aux)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_aux(c.first, aux);
    }
    return ret;
}

bool gazebo::GazeboXBotPlugin::set_robot_aux(const std::map< int, float >& aux)
{
    bool ret = true;
    for(auto& c : robot_map) {
        ret &= set_chain_aux(c.first, aux);
    }
    return ret;
}









///////////////////////////////
///////////////////////////////
// CHAIN PROTECTED FUNCTIONS //
///////////////////////////////
///////////////////////////////

bool gazebo::GazeboXBotPlugin::get_chain_link_pos(std::string chain_name, std::map< std::string, float>& link_pos)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            link_pos[actual_joint_name] = 0;
            if( !get_link_pos(actual_chain_enabled_joints[i], link_pos.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_link_pos() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_link_pos() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_link_pos(std::string chain_name, std::map< int, float >& link_pos)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            link_pos[actual_chain_enabled_joints[i]] = 0;
            if( !get_link_pos(actual_chain_enabled_joints[i], link_pos.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_link_pos() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_link_pos() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_motor_pos(std::string chain_name, std::map< std::string, float >& motor_pos)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            motor_pos[actual_joint_name] = 0;
            if( !get_motor_pos(actual_chain_enabled_joints[i], motor_pos.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_motor_pos() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_motor_pos() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_motor_pos(std::string chain_name, std::map< int, float >& motor_pos)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            motor_pos[actual_chain_enabled_joints[i]] = 0;
            if( !get_motor_pos(actual_chain_enabled_joints[i], motor_pos.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_motor_pos() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_motor_pos() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_link_vel(std::string chain_name, std::map< std::string, float >& link_vel)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            link_vel[actual_joint_name] = 0;
            if( !get_link_vel(actual_chain_enabled_joints[i], link_vel.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_link_vel() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_link_vel() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_link_vel(std::string chain_name, std::map< int, float >& link_vel)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            link_vel[actual_chain_enabled_joints[i]] = 0;
            if( !get_link_vel(actual_chain_enabled_joints[i], link_vel.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_link_vel() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_link_vel() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_motor_vel(std::string chain_name, std::map< std::string, int16_t >& motor_vel)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            motor_vel[actual_joint_name] = 0;
            if( !get_motor_vel(actual_chain_enabled_joints[i], motor_vel.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_motor_vel() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_motor_vel() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_motor_vel(std::string chain_name, std::map< int, int16_t >& motor_vel)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            motor_vel[actual_chain_enabled_joints[i]] = 0;
            if( !get_motor_vel(actual_chain_enabled_joints[i], motor_vel.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_motor_vel() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_motor_vel() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_torque(std::string chain_name, std::map< std::string, int16_t >& torque)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            torque[actual_joint_name] = 0;
            if( !get_torque(actual_chain_enabled_joints[i], torque.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_torque() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_torque() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_torque(std::string chain_name, std::map< int, int16_t >& torque)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            torque[actual_chain_enabled_joints[i]] = 0;
            if( !get_torque(actual_chain_enabled_joints[i], torque.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_torque() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_torque() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_max_temperature(std::string chain_name, std::map< std::string, uint16_t >& max_temperature)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            max_temperature[actual_joint_name] = 0;
            if( !get_max_temperature(actual_chain_enabled_joints[i], max_temperature.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_max_temperature() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_max_temperature() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_max_temperature(std::string chain_name, std::map< int, uint16_t >& max_temperature)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            max_temperature[actual_chain_enabled_joints[i]] = 0;
            if( !get_max_temperature(actual_chain_enabled_joints[i], max_temperature.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_max_temperature() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_max_temperature() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_fault(std::string chain_name, std::map< std::string, uint16_t >& fault)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            fault[actual_joint_name] = 0;
            if( !get_fault(actual_chain_enabled_joints[i], fault.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_fault() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_fault() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_fault(std::string chain_name, std::map< int, uint16_t >& fault)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            fault[actual_chain_enabled_joints[i]] = 0;
            if( !get_fault(actual_chain_enabled_joints[i], fault.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_fault() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_fault() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_rtt(std::string chain_name, std::map< std::string, uint16_t >& rtt)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            rtt[actual_joint_name] = 0;
            if( !get_rtt(actual_chain_enabled_joints[i], rtt.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_rtt() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_rtt() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_rtt(std::string chain_name, std::map< int, uint16_t >& rtt)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            rtt[actual_chain_enabled_joints[i]] = 0;
            if( !get_rtt(actual_chain_enabled_joints[i], rtt.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_rtt() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_rtt() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_op_idx_ack(std::string chain_name, std::map< std::string, uint16_t >& op_idx_ack)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            op_idx_ack[actual_joint_name] = 0;
            if( !get_op_idx_ack(actual_chain_enabled_joints[i], op_idx_ack.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_op_idx_ack() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_op_idx_ack() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_op_idx_ack(std::string chain_name, std::map< int, uint16_t >& op_idx_ack)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            op_idx_ack[actual_chain_enabled_joints[i]] = 0;
            if( !get_op_idx_ack(actual_chain_enabled_joints[i], op_idx_ack.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_op_idx_ack() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_op_idx_ack() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_aux(std::string chain_name, std::map< std::string, float >& aux)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            aux[actual_joint_name] = 0;
            if( !get_aux(actual_chain_enabled_joints[i], aux.at(actual_joint_name)))  {
                DPRINTF("ERROR: get_chain_aux() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_aux() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::get_chain_aux(std::string chain_name, std::map< int, float >& aux)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            aux[actual_chain_enabled_joints[i]] = 0;
            if( !get_aux(actual_chain_enabled_joints[i], aux.at(actual_chain_enabled_joints[i])))  {
                DPRINTF("ERROR: get_chain_aux() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                return false;
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: get_chain_aux() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}




bool gazebo::GazeboXBotPlugin::set_chain_pos_ref(std::string chain_name, const std::map< std::string, float >& pos_ref)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            if(pos_ref.count(actual_joint_name)) {
                if( !set_pos_ref(actual_chain_enabled_joints[i], pos_ref.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_pos_ref() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_pos_ref() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_pos_ref(std::string chain_name, const std::map< int, float >& pos_ref)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(pos_ref.count(actual_chain_enabled_joints[i])) {
                if( !set_pos_ref(actual_chain_enabled_joints[i], pos_ref.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_pos_ref() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_pos_ref() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_vel_ref(std::string chain_name, const std::map< std::string, int16_t >& vel_ref)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            if(vel_ref.count(actual_joint_name)) {
                if( !set_vel_ref(actual_chain_enabled_joints[i], vel_ref.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_vel_ref() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_vel_ref() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_vel_ref(std::string chain_name, const std::map< int, int16_t >& vel_ref)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(vel_ref.count(actual_chain_enabled_joints[i])) {
                if( !set_vel_ref(actual_chain_enabled_joints[i], vel_ref.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_vel_ref() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_vel_ref() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_tor_ref(std::string chain_name, const std::map< std::string, int16_t >& tor_ref)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            if(tor_ref.count(actual_joint_name)) {
                if( !set_tor_ref(actual_chain_enabled_joints[i], tor_ref.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_tor_ref() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_tor_ref() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_tor_ref(std::string chain_name, const std::map< int, int16_t >& tor_ref)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(tor_ref.count(actual_chain_enabled_joints[i])) {
                if( !set_tor_ref(actual_chain_enabled_joints[i], tor_ref.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_tor_ref() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_tor_ref() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_gains(std::string chain_name, const std::map< std::string, std::vector<uint16_t> >& gains)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            if(gains.count(actual_joint_name)) {
                if( !set_gains(actual_chain_enabled_joints[i], gains.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_chain_gains() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_pos_ref() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_gains(std::string chain_name, const std::map< int, std::vector<uint16_t> >& gains)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(gains.count(actual_chain_enabled_joints[i])) {
                if( !set_gains(actual_chain_enabled_joints[i], gains.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_gains() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_gains() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_fault_ack(std::string chain_name, const std::map< std::string, int16_t >& fault_ack)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            if(fault_ack.count(actual_joint_name)) {
                if( !set_fault_ack(actual_chain_enabled_joints[i], fault_ack.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_fault_ack() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_fault_ack() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_fault_ack(std::string chain_name, const std::map< int, int16_t >& fault_ack)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(fault_ack.count(actual_chain_enabled_joints[i])) {
                if( !set_fault_ack(actual_chain_enabled_joints[i], fault_ack.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_fault_ack() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_fault_ack() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_ts(std::string chain_name, const std::map< std::string, uint16_t >& ts)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            if(ts.count(actual_joint_name)) {
                if( !set_ts(actual_chain_enabled_joints[i], ts.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_ts() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_ts() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_ts(std::string chain_name, const std::map< int, uint16_t >& ts)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(ts.count(actual_chain_enabled_joints[i])) {
                if( !set_ts(actual_chain_enabled_joints[i], ts.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_ts() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_ts() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_op_idx_aux(std::string chain_name, const std::map< std::string, uint16_t >& op_idx_aux)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            if(op_idx_aux.count(actual_joint_name)) {
                if( !set_op_idx_aux(actual_chain_enabled_joints[i], op_idx_aux.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_op_idx_aux() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_op_idx_aux() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_op_idx_aux(std::string chain_name, const std::map< int, uint16_t >& op_idx_aux)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(op_idx_aux.count(actual_chain_enabled_joints[i])) {
                if( !set_op_idx_aux(actual_chain_enabled_joints[i], op_idx_aux.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_op_idx_aux() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_op_idx_aux() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_aux(std::string chain_name, const std::map< std::string, float >& aux)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        std::string actual_joint_name;
        for( int i = 0; i < enabled_joints_num; i++) {
            actual_joint_name = _XBotModel.rid2Joint(actual_chain_enabled_joints[i]);
            if(aux.count(actual_joint_name)) {
                if( !set_aux(actual_chain_enabled_joints[i], aux.at(actual_joint_name)))  {
                    DPRINTF("ERROR: set_aux() on joint %s, that does not exits in the chain %s\n", actual_joint_name.c_str(), chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_aux() on chain %s, that does not exits in the robot\n", chain_name.c_str());
    return false;
}

bool gazebo::GazeboXBotPlugin::set_chain_aux(std::string chain_name, const std::map< int, float >& aux)
{
    if( robot_map.count(chain_name) ) {
        std::vector<int> actual_chain_enabled_joints = robot_map.at(chain_name);
        int enabled_joints_num = actual_chain_enabled_joints.size();
        for( int i = 0; i < enabled_joints_num; i++) {
            if(aux.count(actual_chain_enabled_joints[i])) {
                if( !set_aux(actual_chain_enabled_joints[i], aux.at(actual_chain_enabled_joints[i])))  {
                    DPRINTF("ERROR: set_aux() on joint %d, that does not exits in the chain %s\n", actual_chain_enabled_joints[i], chain_name.c_str());
                    return false;
                }
            }
        }
        return true;
    }
    
    DPRINTF("ERROR: set_chain_aux() on chain %s, that does not exits in the robot\n", chain_name.c_str());
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

bool gazebo::GazeboXBotPlugin::parseYAML ( const std::string& path_to_cfg )
{
    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        std::cerr << "ERROR in " << __func__ << "! Can NOT open " << path_to_cfg << "!" << std::endl;
        return false;
    }
    

    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);
    YAML::Node x_bot_interface;
    if(root_cfg["XBotInterface"]) {
        x_bot_interface = root_cfg["XBotInterface"]; 
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

// TBD implement it
bool gazebo::GazeboXBotPlugin::get_ft(int ft_id, std::vector< float >& ft, int channels)
{
//     // check if the joint requested exists
//     if( fts.count(rid2Pos(ft_id)) ) {
//         // get the data, resize the ft vector and copy only Fx,Fy,Fz and Tx,Ty,Tz
//         iit::ecat::advr::Ft6EscPdoTypes::pdo_rx actual_pdo_rx_ft = fts[rid2Pos(ft_id)]->getRxPDO();
//         ft.resize(channels);
//         std::memcpy(ft.data(), &(actual_pdo_rx_ft.force_X), channels*sizeof(float));
//         
//         return true;
//     }
//     
//     // we don't touch the value that you passed
//     DPRINTF("Trying to get_ft() on ft with ft_id : %d that does not exists\n", ft_id);
//     return false;   
     return false; 
}

// TBD implement it
bool gazebo::GazeboXBotPlugin::get_ft_fault(int ft_id, uint16_t& fault)
{
//     // check if the joint requested exists
//     if( fts.count(rid2Pos(ft_id)) ) {
//         // get the data
//         fault = fts[rid2Pos(ft_id)]->getRxPDO().fault;
//         return true;
// }
//     
//     // we don't touch the value that you passed
//     DPRINTF("Trying to get_ft_fault() on ft with ft_id : %d that does not exists\n", ft_id);
//     return false;  
     return false; 
}

// TBD implement it
bool gazebo::GazeboXBotPlugin::get_ft_rtt(int ft_id, uint16_t& rtt)
{
//     // check if the joint requested exists
//     if( fts.count(rid2Pos(ft_id)) ) {
//         // get the data
//         rtt = fts[rid2Pos(ft_id)]->getRxPDO().rtt;
//         return true;
//     }
//     
//     // we don't touch the value that you passed
//     DPRINTF("Trying to get_ft_rtt() on ft with ft_id : %d that does not exists\n", ft_id);
//     return false;   
     return false; 
}


// NOTE get system time: should we use simulation?
uint64_t gazebo::GazeboXBotPlugin::get_time_ns(clockid_t clock_id)
{
    uint64_t time_ns;
    struct timespec ts;
    clock_gettime(clock_id, &ts);
    time_ns = ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
    return time_ns;
}

