
#include<GazeboXBotPlugin/GazeboXBotHand.h>


gazebo::GazeboXBotHand::GazeboXBotHand()
{
}

gazebo::GazeboXBotHand::~GazeboXBotHand()
{
}


void gazebo::GazeboXBotHand::grasp_status_Callback(const std_msgs::Bool::ConstPtr& msg, int hand_id)
{
  
  _status_grasp[hand_id] = msg.get()->data;
  return;
}


void gazebo::GazeboXBotHand::initGrasping(
        XBot::RobotInterface::Ptr robot)
{
    _robot = robot;
    
    int argc = 1;
    const char *arg = "MagneticGrasping";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;

    if(!ros::isInitialized()){
        ros::init(argc, argv, "MagneticGrasping");
    }
      
    _nh = std::make_shared<ros::NodeHandle>();
   
    for( auto& pair : _robot->getHand()){      
      std::string hand_name= pair.second->getHandName();
      int hand_id = pair.second->getHandId();
      std::string hand_link =_robot->getUrdf().getJoint(hand_name)->parent_link_name;
      _grasp[hand_id] = _nh->advertise<std_msgs::Bool>("/grasp/"+hand_link+"/autoGrasp",1);
      _state_grasp[hand_id] = _nh->subscribe<std_msgs::Bool>("/grasp/"+hand_link+"/state", 1, boost::bind(&gazebo::GazeboXBotHand::grasp_status_Callback,this,_1,hand_id));
      _status_grasp[hand_id] = false;
      
    }
    
}

bool gazebo::GazeboXBotHand::grasp(int hand_id, double grasp_percentage)
{
    std_msgs::Bool message;    
    
    if(grasp_percentage == 0)
      message.data = false;
    else
      message.data = true;
    
    _grasp[hand_id].publish (message);
  
  return true;
}
    
double gazebo::GazeboXBotHand::get_grasp_state(int hand_id)
{
  
   double grasp_state = 0;
   grasp_state = _status_grasp[hand_id];
       
   return grasp_state;
}

