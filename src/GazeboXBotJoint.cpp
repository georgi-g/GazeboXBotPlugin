
#include<GazeboXBotPlugin/GazeboXBotJoint.h>

#include <GazeboXBotPlugin/JointImpedanceController.h>


gazebo::GazeboXBotJoint::GazeboXBotJoint()
{
}

gazebo::GazeboXBotJoint::~GazeboXBotJoint()
{
}

bool gazebo::GazeboXBotJoint::loadJoints(XBot::RobotInterface::Ptr robot, physics::ModelPtr model, YAML::Node& root)
{
    _robot = robot;
    
    // iterate over Gazebo model Joint vector and store Joint pointers in a map
    const gazebo::physics::Joint_V & gazebo_models_joints = model->GetJoints();
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
        _jointMap[gazebo_joint_name] = model->GetJoint(gazebo_joint_name);

        _joint_controller_map[gazebo_joint_name] =
            std::make_shared<XBot::JointImpedanceController>( model->GetJoint(gazebo_joint_name) );

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
    
    return true;
}

void gazebo::GazeboXBotJoint::XBotUpdate()
{
    for( auto& pair : _joint_controller_map ){
        pair.second->sendControlInput();
    }
}

gazebo::physics::JointPtr gazebo::GazeboXBotJoint::getJoint(int joint_id)
{
    if (!_robot)
        return 0;

    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    if (current_joint_name == "")
        return 0;
        
    auto it = _jointMap.find(current_joint_name);
    if (it == _jointMap.end())
        return 0;
    
    return it->second;
}

XBot::JointController::Ptr gazebo::GazeboXBotJoint::getJointController(int joint_id)
{

    if (!_robot)
        return 0;

    std::string current_joint_name = _robot->getJointByID(joint_id)->getJointName();
    if (current_joint_name == "")
        return 0;
        
    auto it = _joint_controller_map.find(current_joint_name);
    if (it == _joint_controller_map.end())
        return 0;
    
    return it->second;
}



bool gazebo::GazeboXBotJoint::get_link_pos ( int joint_id, double& link_pos )
{
    auto j(getJoint(joint_id));

    if(j) {
        link_pos = j->GetAngle(0).Radian();
        return true;
    }
    else {
        link_pos = 0;
        return false;
    }
}

bool gazebo::GazeboXBotJoint::get_aux ( int joint_id, double& aux )
{
    aux = 0;
    return false;
}

bool gazebo::GazeboXBotJoint::get_fault ( int joint_id, double& fault )
{
    fault = 0;
    return false;
}

bool gazebo::GazeboXBotJoint::get_link_vel ( int joint_id, double& link_vel )
{
    auto j(getJoint(joint_id));

    if(j) {
        link_vel = j->GetVelocity(0);
        return true;
    }
    else {
        link_vel = 0;
        return false;
    }
}

bool gazebo::GazeboXBotJoint::get_temperature ( int joint_id, double& temperature )
{
    temperature = 0;
    return false;
}

bool gazebo::GazeboXBotJoint::get_gains(int joint_id, std::vector< double >& gain_vector)
{

    auto j(getJointController(joint_id));

    if(j) {
        if(gain_vector.size() < 2){
            gain_vector.assign(2, 0);
        }
        gain_vector[0] = j->getP();
        gain_vector[1] = j->getD();

        return true;
    }

    return false;
}


bool gazebo::GazeboXBotJoint::get_motor_pos ( int joint_id, double& motor_pos )
{
    auto j(getJoint(joint_id));

    if(j) {
        motor_pos = j->GetAngle(0).Radian();
        // NOTE we return true but we are reading the link position from gazebo TBD a plugin should simulate this
        return true;
    }
    else {
        motor_pos = 0;
        return false;
    }
}

bool gazebo::GazeboXBotJoint::get_motor_vel ( int joint_id, double& motor_vel )
{
    auto j(getJoint(joint_id));

    if(j) {
        motor_vel = j->GetVelocity(0);
        // NOTE we return false but we are reading the link position drom gazebo TBD a plugin should simulate this
        return true;
    }
    else {
        motor_vel = 0;
        return false;
    }
}

bool gazebo::GazeboXBotJoint::get_op_idx_ack ( int joint_id, double& op_idx_ack )
{
    op_idx_ack = 0;
    return false;
}

bool gazebo::GazeboXBotJoint::get_rtt ( int joint_id, double& rtt )
{
    // TBD is it possible to get this info??
    rtt = 0;
    return false;
}

bool gazebo::GazeboXBotJoint::get_torque ( int joint_id, double& torque )
{
    auto j(getJoint(joint_id));

    if(j) {
        torque = j->GetForce(0);
        return true;
    }
    else {
        torque = 0;
        return false;
    }
}

bool gazebo::GazeboXBotJoint::get_pos_ref(int joint_id, double& pos_ref)
{
    auto j(getJointController(joint_id));

    if(j) {
        pos_ref = j->getPositionReference();
        return true;
    }

    return false;
}


bool gazebo::GazeboXBotJoint::get_vel_ref(int joint_id, double& vel_ref)
{
    auto j(getJointController(joint_id));

    if(j) {
        vel_ref = j->getVelocityReference();
        return true;
    }

    return false;
}


bool gazebo::GazeboXBotJoint::get_tor_ref(int joint_id, double& tor_ref)
{
    auto j(getJointController(joint_id));

    if(j) {
        tor_ref = j->getTorqueReference();
        return true;
    }

    return false;
}


bool gazebo::GazeboXBotJoint::set_aux ( int joint_id, const double& aux )
{
    return false;
}

bool gazebo::GazeboXBotJoint::set_fault_ack ( int joint_id, const double& fault_ack )
{
    return false;
}

bool gazebo::GazeboXBotJoint::set_gains ( int joint_id, const std::vector< double >& gains )
{
    if(gains.size() < 2){
        std::cerr << "ERROR in " << __func__ << "! Provided gains vector size is " << gains.size() << " less than 2!" << std::endl;
        return false;
    }

    auto j(getJointController(joint_id));

    if(j) {
        j->setGains(gains[0], 0, gains[1]);
        return true;
    }

    return false;
}

bool gazebo::GazeboXBotJoint::set_op_idx_aux ( int joint_id, const double& op_idx_aux )
{
    return false;
}

bool gazebo::GazeboXBotJoint::set_pos_ref ( int joint_id, const double& pos_ref )
{
    auto j(getJointController(joint_id));

    if(j) {
        j->setPositionReference(pos_ref);
        return true;
    }

    return false;
}

bool gazebo::GazeboXBotJoint::set_tor_ref ( int joint_id, const double& tor_ref )
{
    auto j(getJointController(joint_id));

    if(j) {
        j->setTorqueReference(double(tor_ref)); // NOTE torque scaling random but suitable to avoid the int16t overflow
        return true;
    }

    return false;
}

bool gazebo::GazeboXBotJoint::set_ts ( int joint_id, const double& ts )
{
    return false;
}

bool gazebo::GazeboXBotJoint::set_vel_ref ( int joint_id, const double& vel_ref )
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




