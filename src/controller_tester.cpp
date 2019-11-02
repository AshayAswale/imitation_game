#include <imitation_game/joint_angles_controller.h>
#include <tough_common/robot_state.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_tester");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(10);
  spinner.start();

  RobotStateInformer* state_informer = RobotStateInformer::getRobotStateInformer(nh);
  RobotDescription* rd = RobotDescription::getRobotDescription(nh);
  ros::Duration(0.5).sleep();
  JointAnglesController joint_controller(nh);
  ArmControlInterface arm_controller(nh);

  std::vector<std::string> left_arm_names;
  std::vector<double> joint_positions;
  joint_positions.resize(17);
  for (auto& i : joint_positions)
    i = 0;

  std::vector<double> joint_accelerations;
  std::vector<double> arm_accelerations;
  int left_arm_position = 3;
  int right_arm_position = 10;
  int arm_position;
  RobotSide side = RobotSide::LEFT;

  while (ros::ok)
  {
    arm_accelerations.resize(0);
    side = side == RobotSide::LEFT ? RobotSide::RIGHT : RobotSide::LEFT;
    arm_position = side == RobotSide::LEFT ? left_arm_position : right_arm_position;
    joint_accelerations = joint_controller.getControlledJointAngles(joint_positions);
    arm_accelerations.insert(arm_accelerations.end(), joint_accelerations.begin() + arm_position,
                             joint_accelerations.begin() + arm_position + 7);
    arm_controller.moveArmJointsAcceleration(side, arm_accelerations);
  }

  spinner.start();
  return 0;
}
