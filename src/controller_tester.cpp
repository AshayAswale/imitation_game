#include <imitation_game/joint_angles_controller.h>
#include <tough_common/robot_state.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_controller_interface/wholebody_control_interface.h>
#include <tough_kinematics/tough_kinematics.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_tester");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(10);
  spinner.start();

  RobotDescription* rd = RobotDescription::getRobotDescription(nh);
  RobotStateInformer* state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  ros::Duration(1).sleep();
  WholebodyControlInterface wb_controller(nh);
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  pose.header.frame_id = rd->getPelvisFrame();

  if (argc != 4)
  {
    pose.pose.position.x = 0.4;
    pose.pose.position.y = 0.8;
    pose.pose.position.z = 0.2;
  }
  else
  {
    pose.pose.position.x = std::atof(argv[1]);
    pose.pose.position.y = std::atof(argv[2]);
    pose.pose.position.z = std::atof(argv[3]);
  }
  std::vector<double> joint_angles;
  trajectory_msgs::JointTrajectory result_joint_angles;

  ToughKinematics tough_kinematics(nh);
  JointAnglesController joint_controller(nh);
  if (tough_kinematics.solveIK(TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP, pose, result_joint_angles))
  {
    while(ros::ok())
    {
      joint_controller.updateJointAccelerations(result_joint_angles);
      wb_controller.executeAccnTrajectory(result_joint_angles);
      // ros::Duration(0.02).sleep();
    }
    // wb_controller.executeTrajectory(result_joint_angles);
    // ros::Duration(1).sleep();
  }
  else
    ROS_ERROR("COULD NOT PLAN FOR THE TRAJECTORY");

  spinner.stop();
  return 0;
}
