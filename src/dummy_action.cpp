#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <payload_msgs/PayloadActionAction.h>

class DummyAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<payload_msgs::PayloadActionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  payload_msgs::PayloadActionFeedback feedback_;
  payload_msgs::PayloadActionResult result_;

public:

  DummyAction(std::string name) :
    as_(nh_, name, boost::bind(&DummyAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~DummyAction(void)
  {
  }

  void executeCB(const payload_msgs::PayloadActionGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;


    // publish info to the console for the user
    ROS_INFO("%s: Executing, dummy action", action_name_.c_str());

    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
    }

    if(success)
    {
      result_.status = 0;
      result_.message = "SUCCESS";
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_action");

  DummyAction fibonacci("dummy_action");
  ros::spin();

  return 0;
}
