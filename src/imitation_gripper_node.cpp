#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <tough_common/robot_state.h>
#include <razer_hydra/HydraRaw.h>
#include <time.h>
#include <tough_common/robot_state.h>
#include <time.h>
#include <tough_footstep/robot_walker.h>
#include <tough_controller_interface/gripper_control_interface.h>

bool left_gripper_close = false;
bool left_gripper_open = false;
bool right_gripper_open = false;
bool right_gripper_close = false;

void hydraTriggerCB(const razer_hydra::HydraRaw &msg)
{
  left_gripper_open = (msg.buttons.at(1) == 2) ? true : false;
  left_gripper_close = (msg.buttons.at(1) == 4) ? true : false;
  right_gripper_open = (msg.buttons.at(0) == 16) ? true : false;
  right_gripper_close = (msg.buttons.at(0) == 8) ? true : false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imitation_gripper_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  ros::Rate rate(1);
  spinner.start();

  int i = 0;
  ros::Subscriber sub = nh.subscribe("/hydra_raw", 1, hydraTriggerCB);
  GripperControlInterface gripper(nh);

  ros::Duration(1).sleep();

  while (ros::ok())
  {
    bool hydra_register_trig_ = left_gripper_open || left_gripper_close || right_gripper_open || right_gripper_close;
    if (hydra_register_trig_)
    {
      if(left_gripper_open)
      {
        ROS_INFO("left_gripper_open");
        gripper.openGripper(RobotSide::LEFT);
      }
      else if(left_gripper_close)
      {
        ROS_INFO("left_gripper_close");
        gripper.closeGripper(RobotSide::LEFT);
      }
      else if(right_gripper_open)
      {
        ROS_INFO("right_gripper_open");
        gripper.openGripper(RobotSide::RIGHT);
      }
      else if(right_gripper_close)
      {
        ROS_INFO("right_gripper_close");
        gripper.closeGripper(RobotSide::RIGHT);
      }
      else
        ROS_INFO("ABNORMAL!!");
    }
    rate.sleep();
  }

  return 0;
}
