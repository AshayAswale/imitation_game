#include "ros/ros.h"
#include <imitation_game/shadow_upper_body.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_upper_body_shadow");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  ShadowUpperBody shadow(nh);
  shadow.startShadowMotion();

  while (ros::ok())
  {
    ros::Duration(5).sleep();
  }

  return 0;
}
