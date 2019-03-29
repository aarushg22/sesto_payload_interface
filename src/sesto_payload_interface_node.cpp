#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <payload_msgs/PayloadActionAction.h>
#include <atm_msgs/AgvCommand.h>

class AgvPauseAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<payload_msgs::PayloadActionAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  payload_msgs::PayloadActionFeedback feedback_;
  payload_msgs::PayloadActionResult result_;
  ros::Publisher pub_paused_state;
  ros::Subscriber sub_unpause;

public:

  //@TODO = create  the contructor with a param, agv_id so different nodes can be spawned for different agv's
  AgvPauseAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    as_.registerGoalCallback(boost::bind(&AgvPauseAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&AgvPauseAction::preemptCB, this));

    pub_paused_state = nh_.advertise<atm_msgs::AgvCommand>("agv_paused_state",10,true);
    sub_unpause = nh_.subscribe("agv_unpaused_state",10,&AgvPauseAction::agv_unpaused_state_callback, this);
    as_.start();
  }

  ~AgvPauseAction(void)
  {
  }

  void agv_unpaused_state_callback(const atm_msgs::AgvCommandConstPtr &msg) {

    //@TODO = Also check the agv_id, when it's initialized in the constructor
    if(msg->pause==0){
      result_.status = 0;
      result_.message = "Unpause AGV";
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else if(msg->pause==1){
      ROS_INFO("%s: Pause state recieved in the agv unpause command, ignored", action_name_.c_str());
    }
    else {
      ROS_INFO("%s: Unknown state recieved in the agv unpause command, ignored", action_name_.c_str());
    }
  }

  void goalCB() {

    ROS_INFO("%s: new goal received for", action_name_.c_str());
    atm_msgs::AgvCommand paused_trigger;
    paused_trigger.pause=1;
    payload_msgs::PayloadActionGoalConstPtr new_goal_msg;
    new_goal_msg = as_.acceptNewGoal();
    if(new_goal_msg->mode==new_goal_msg->LOAD){
      paused_trigger.current_station=new_goal_msg->payloads.payload_list[0].pickup_alias;
    }
    if(new_goal_msg->mode==new_goal_msg->UNLOAD){
      paused_trigger.current_station=new_goal_msg->payloads.payload_list[0].delivery_alias;
    }
    pub_paused_state.publish(paused_trigger);

  }

  void preemptCB(){
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
  }

};


int main(int argc, char** argv)
{

  //@TODO = Get the node name and a agv_id in a parameter, so can initialize different nodes for idfferent agv's with same code
  ros::init(argc, argv, "AgvPauseAction");

  AgvPauseAction dummy("sesto_payload_interface_action");
  ros::spin();

  return 0;
}
