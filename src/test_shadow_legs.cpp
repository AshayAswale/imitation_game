#include <imitation_game/shadow_legs.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_shadow_legs");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ShadowLegs shadow_legs(nh);
  

  shadow_legs.startLegsShadowMotion();

  ros::Duration(40.0f).sleep();

  shadow_legs.stopLegsShadowMotion();

  spinner.stop();
  return 0;
}
