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

#ifndef __GAZEBO_XBOT_PLUGIN_JOINT_CONTROLLER_H__
#define __GAZEBO_XBOT_PLUGIN_JOINT_CONTROLLER_H__

#include <gazebo/physics/physics.hh>
#include <memory>

namespace XBot {
    
    class JointController {
        
    public:
        
        typedef std::shared_ptr<JointController> Ptr;
        
        JointController(gazebo::physics::JointPtr joint);

        bool setGains(double p, double i, double d);
        
        void setReference(double pos_ref, double vel_ref, double tau_ref);
        
        virtual void sendControlInput();
        
        double getP() const;
        double getI() const;
        double getD() const;
        
        double getPositionReference() const;
        double getVelocityReference() const;
        double getTorqueReference() const;
        
        virtual bool enableFeedforward();
        virtual bool disableFeedforward();
        bool isFeedforwardEnabled() const;
        

    protected:
        
        double getJointPosition() const;
        double getJointVelocity() const;
        
        virtual bool set_gains_internal(double p, double i, double d);
        virtual double compute_control_input() = 0;
        
        gazebo::physics::JointPtr _joint;
        
    private:
        
        double _p, _i, _d;
        double _pos_ref, _vel_ref, _tau_ref;
        bool _feedforward_enabled;
        

    };
}
#endif