
#ifndef __GAZEBO_XBOT_HAND_H__
#define __GAZEBO_XBOT_HAND_H__

#include <XBotCore-interfaces/All.h>

#include <XBotInterface/XBotInterface.h>

#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace gazebo
{
class GazeboXBotHand :
    public XBot::IXBotHand
{

public :
    /**
     * @brief constructor
     *
     */
    GazeboXBotHand();

    /**
     * @brief destructor
     *
     */
    virtual ~GazeboXBotHand();

    void initGrasping(
        XBot::RobotInterface::Ptr robot);

    void grasp_status_Callback(const std_msgs::Bool::ConstPtr& msg, int hand_id);
    
protected:

private:
    // xbot robot
    XBot::RobotInterface::Ptr _robot;
    
    //grasping
    std::map<int, ros::Publisher> _grasp;
    std::map<int, ros::Subscriber> _state_grasp;
    std::map<int, bool> _status_grasp;
        
    std::shared_ptr<ros::NodeHandle> _nh;

    virtual bool grasp(int hand_id, double grasp_percentage) final;
    
    virtual double get_grasp_state(int hand_id) final;

};

}

#endif
