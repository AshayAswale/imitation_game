#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <tough_common/robot_state.h>
#include <razer_hydra/HydraRaw.h>
#include <time.h>
#include <tough_common/robot_state.h>
#include <time.h>
#include <tough_footstep/robot_walker.h>

bool hydra_register_trig_front_ = false;
bool hydra_register_trig_back_ = false;

void hydraTriggerCB(const razer_hydra::HydraRaw &msg)
{
  hydra_register_trig_front_ = (msg.buttons.at(0) == 2 && msg.buttons.at(1) == 16) ? true : false;
  hydra_register_trig_back_ = (msg.buttons.at(0) == 4 && msg.buttons.at(1)  == 8)? true : false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imitation_walking_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  ros::Rate rate(1);
  spinner.start();

  int i = 0;
  ros::Subscriber sub = nh.subscribe("/hydra_raw", 1, hydraTriggerCB);
  RobotStateInformer *state_informer = RobotStateInformer::getRobotStateInformer(nh);
  RobotDescription *rd = RobotDescription::getRobotDescription(nh);
  RobotWalker walker(nh);

  ros::Duration(1).sleep();
  geometry_msgs::PoseStamped pose_object;

  while (ros::ok())
  {
    if (hydra_register_trig_front_)
    {
      ROS_WARN("Walking Forward");
      walker.walkNSteps(1, 0.2, 00, RobotSide::LEFT, true);
    }
    else if (hydra_register_trig_back_)
    {
      ROS_WARN("Walking Back");
      walker.walkNSteps(1, -0.2, 00, RobotSide::RIGHT, true);
    }
    rate.sleep();
  }

  return 0;
}
