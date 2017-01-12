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

#include <GazeboXBotPlugin/JointImpedanceController.h>

namespace XBot {
    
JointImpedanceController::JointImpedanceController(gazebo::physics::JointPtr joint): JointController(joint)
{

}


double JointImpedanceController::sendControlInput(double pos_ref, double vel_ref, double tau_ref)
{
    double torque = 0;
    
    torque = getP()*( pos_ref - getJointPosition() ) + getD()*( -getJointVelocity() );
    
    if( isFeedforwardEnabled() ){
        torque += tau_ref;
    }
    
    _joint->SetForce(0, torque);
    
    return torque;
}

    
    
}