#include<GazeboXBotPlugin/GazeboXBotPlugin.h>

#include<iostream>

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
    // simulation iteration
    _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboXBotPlugin::XBotUpdate, this, _1));
    
    // save the sdf handle
    this->_sdf = _sdf;
    
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
    parseYAML(path_to_cfg); // TBD do it with plugin params
    
    // initialize the model
    if (!_XBotModel.init(_urdf_path, _srdf_path, _joint_map_config)) {
        printf("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n"); 
        return;
    }
    // generate the robot
    _XBotModel.generate_robot();
    
    // iterate over model Joint vector and store Joint pointers in a map
    const gazebo::physics::Joint_V & gazebo_models_joints = _model->GetJoints();
    for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size(); gazebo_joint++) {
        std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
        _jointNames.push_back(gazebo_joint_name);
        _jointMap[gazebo_joint_name] = _model->GetJoint(gazebo_joint_name);
        gazebo::common::PID pid;
        pid.Init(600, 0, 3, 0, 0, 10000, -10000);
        
        _model->GetJointController()->SetPositionPID(_jointMap.at(gazebo_joint_name)->GetScopedName(), pid);
        
        std::cout << "Joint # " << gazebo_joint << " - " << gazebo_joint_name << std::endl;
        
    }
    
    XBot::AnyMapPtr any_map = std::make_shared<XBot::AnyMap>();
    (*any_map)["XBotJoint"] = std::shared_ptr<XBot::IXBotJoint>(this, [](XBot::IXBotJoint* ptr){} );
    
    _robot = XBot::RobotInterface::getRobot(path_to_cfg, any_map);
    _robot->getRobotState("home", _q_home);
    _robot->sense();
    _robot->getJointPosition(_q0);
    
    _previous_time = _world->GetSimTime().Double();
    
    
    

}

void gazebo::GazeboXBotPlugin::XBotUpdate(const common::UpdateInfo & _info)
{
//     // TBD run the RT plugin
//     float link_pos = -1;
// //     get_link_pos( 11, link_pos);
//     std::cout << "Joint 11 - link_pos : " << link_pos << std::endl;
//     
//     float motor_pos = -1;
//     get_motor_pos( 11, motor_pos);
//     std::cout << "Joint 11 - motor_pos : " << motor_pos << std::endl;
//     
//     float link_vel = -1;
//     get_link_vel( 11, link_vel);
//     std::cout << "Joint 11 - link_vel : " << link_vel << std::endl;
//     
//     int16_t motor_vel = -1;
//     get_motor_vel( 11, motor_vel);
//     std::cout << "Joint 11 - motor_vel : " << motor_vel << std::endl;
//     
//     int16_t torque = -1;
//     get_torque( 11, torque);
//     std::cout << "Joint 11 - torque : " << torque << std::endl;
//     
//     set_pos_ref(2, std::cos(_world->GetSimTime().Double()));
    double time = _world->GetSimTime().Double();
    std::cout << "DT = " << time - _previous_time << std::endl;
    _previous_time = time;
    
    _robot->sense();
    _robot->setPositionReference(_q0 + 0.5*(1-std::cos(_world->GetSimTime().Double()))*(_q_home-_q0));
    _robot->printTracking();
    _robot->move();
    _model->GetJointController()->Update();

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


