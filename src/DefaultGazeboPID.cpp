/*
 * Copyright (C) 2017 IIT-ADVR
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

#include <GazeboXBotPlugin/DefaultGazeboPID.h>

namespace XBot {
    
bool DefaultGazeboPID::disableFeedforward()
{
    return XBot::JointController::disableFeedforward();
}

bool DefaultGazeboPID::enableFeedforward()
{
    return false;
}

DefaultGazeboPID::DefaultGazeboPID(gazebo::physics::JointPtr joint, 
                                           gazebo::physics::JointControllerPtr joint_ctrl): 
    JointController(joint),
    _joint_ctrl(joint_ctrl)
{

}

double DefaultGazeboPID::sendControlInput(double pos_ref, 
                                                  double vel_ref, 
                                                  double tau_ref)
{
    _joint_ctrl->SetPositionTarget(_joint->GetScopedName(), pos_ref);
    return _joint_ctrl->GetForces().at(_joint->GetScopedName());
}
 
bool DefaultGazeboPID::set_gains_internal(double p, double i, double d)
{
    gazebo::common::PID pid;
    pid.Init(p, i, d);
    _joint_ctrl->SetPositionPID(_joint->GetScopedName(), pid);
    
    return true;
}


 
}