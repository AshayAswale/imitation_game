#include <tough_controller_interface/arm_control_interface.h>
/**
 * @brief Small code written to tune gains for the arm joints. This 
 * tunes the gains for one joint a time. 
 * 
 * PROCEDURE:
 *        For comparision, the opposite arm is also moved to the same set value. 
 * The time for the movement is given as 1 second. The tuned values should also 
 * produce the graph of joint angle movements same as the ones moved by the IHMC
 * controllers. 
 * 
 * TRICKS:
 *        For making the graph similar to IHMC controlled values, the slope of 
 * at the start of the movement can be changed by the max/min acceleration. The
 * length of the straight line in the graph can be manipulated with the Kp. The
 * bend and the curve at the damping is manipulated by the Kd.
 * 
 * CAUTION:
 *        This does not take care of the joint limits. This does not limit any
 * other kind of mis-values. 
 * 
 * ##### BE EXTREMELY CAUTIOUS IF AT ALL THIS IS BEING TESTED ON THE REAL ROBOT #####
 * 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_gain_tuner");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (argc!=3)
  {
    ROS_INFO("Please give Kp and Kd as arguments");
    return -1;
  }

  /* Arm best values: 
      Kp = 110
      Kd = 30
  */
  double Kp = std::atof(argv[1]);
  double Kd = std::atof(argv[2]);

  ArmControlInterface arm_controller(nh);
  RobotStateInformer* robot_state = RobotStateInformer::getRobotStateInformer(nh);
  RobotDescription* rd = RobotDescription::getRobotDescription(nh);

  int current_joint_number = 0;         // Current joint being tuned.
  double set_joint_position = 0.5;      // Desired value to be reached by the joint.
  std::vector<std::string> left_arm_joint_names;
  rd->getLeftArmJointNames(left_arm_joint_names);

  RobotSide side = RobotSide::LEFT;
  arm_controller.moveToZeroPose(side, 1.0);
  ros::Duration(0.1).sleep();

  RobotSide oppositeSide = side == RobotSide::LEFT ? RobotSide::RIGHT : RobotSide::LEFT;
  arm_controller.moveToZeroPose(oppositeSide, 1.0);
  ros::Duration(2.0).sleep();
  arm_controller.moveArmJoint(RobotSide::RIGHT, current_joint_number, set_joint_position, 1.0);
  ROS_WARN("Starting to Control joint %s to %d", left_arm_joint_names.at(current_joint_number).c_str(), set_joint_position);
  
  double current_position, error, previous_error = 0, previous_position = 0;
  double derivative;
  double max_acceleration = 5, min_accleration = -5;
  double Pout, Dout, output;
  double dt = 0.02;

  std::vector<double> arm_accelerations;
  arm_accelerations.resize(7);
  for (int i = 0; i < 7; i++)
    arm_accelerations.at(i) = 0;

  while (ros::ok())
  {
    current_position = robot_state->getJointPosition(left_arm_joint_names.at(current_joint_number));
    error = set_joint_position - current_position;
    Pout = Kp * error;

    derivative = (error - previous_error) / dt;
    Dout = Kd * derivative;
    previous_error = error;

    output = Pout + Dout;
    
    if(output>max_acceleration)
    {
      output = max_acceleration;
      ROS_INFO(".");
    }
    else if(output<min_accleration)
      output = min_accleration;

    arm_accelerations.at(current_joint_number) = output;
    arm_controller.moveArmJointsAcceleration(side, arm_accelerations);
  }

  return 0;
}
