#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

class TestHandAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  control_msgs::FollowJointTrajectoryActionFeedback feedback_;
  control_msgs::FollowJointTrajectoryActionResult result_;

public:

  TestHandAction(std::string name) :
    as_(nh_, name, boost::bind(&TestHandAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~TestHandAction(void)
  {
  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(10);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.feedback.desired = goal->trajectory.points[0];

    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<5; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.feedback.actual = feedback_.feedback.desired;
      // publish the feedback
      as_.publishFeedback(feedback_.feedback);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      //result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_.result);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestHandAction");

  TestHandAction test_hand_action_right("/rh_trajectory_controller/follow_joint_trajectory");
  TestHandAction test_hand_action_left("/lh_trajectory_controller/follow_joint_trajectory");
  ros::spin();

  return 0;
}

