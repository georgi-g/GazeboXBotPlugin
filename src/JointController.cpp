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

#include <GazeboXBotPlugin/JointController.h>

namespace XBot {
    
JointController::JointController(gazebo::physics::JointPtr joint):
    _joint(joint),
    _p(0),
    _i(0),
    _d(0),
    _feedforward_enabled(false)
{

}

bool JointController::setGains(double p, double i, double d)
{
    _p = p;
    _i = i;
    _d = d;
    
    return set_gains_internal();
}

bool JointController::enableFeedforward()
{
    _feedforward_enabled = true;
    return true;
}

bool JointController::disableFeedforward()
{
    _feedforward_enabled = false;
    return true;
}

bool JointController::isFeedforwardEnabled() const
{
    return _feedforward_enabled;
}

double JointController::getP() const
{
    return _p;
}

double JointController::getI() const
{
    return _i;
}

double JointController::getD() const
{
    return _d;
}

double JointController::getJointPosition() const
{
    return _joint->GetAngle(0).Radian();
}

double JointController::getJointVelocity() const
{
    return _joint->GetVelocity(0);
}

bool JointController::set_gains_internal(double p, double i, double d)
{
    return true;
}




















    
    
}