#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <imitation_game/Joints.h>
#include <tough_common/robot_state.h>
#include <imitation_game/Vector.h>

imitation_game::Eulers getTransformation(const std::string &track_frame, const std::string &ref_frame, const std::string &name, const bool elb_comp = false)
{
  imitation_game::Eulers euler;
  static tf::TransformListener listener;
  static tf::StampedTransform transform;
  try
  {
    listener.waitForTransform(track_frame, ref_frame, ros::Time(0.0), ros::Duration(0.25));
    listener.lookupTransform(track_frame, ref_frame, ros::Time(0.0), transform);
    tf::Matrix3x3 mat(transform.getRotation());
    mat.getRPY(euler.roll, euler.pitch, euler.yaw);

    if (elb_comp)
    { 
      euler.yaw = atan(-transform.getOrigin().getX() / transform.getOrigin().getY());
      euler.roll = atan(transform.getOrigin().getZ() / transform.getOrigin().getY());
      euler.roll = std::max(std::abs(euler.yaw), std::abs(euler.roll));
      euler.pitch = atan(transform.getOrigin().getX() / transform.getOrigin().getZ());
      euler.pitch = (euler.pitch < 0) ? M_PI + euler.pitch : euler.pitch;
    }

    euler.name = name;
  }
  catch (const std::exception &e)
  {
    static int fail = 0;
    std::cerr << e.what() << '\n';
    ros::Duration(0.01).sleep();
    if (fail < 5)
    {
      fail++;
      getTransformation(track_frame, ref_frame, name);
    }
    fail = 0;
    euler.name = "FAILED!!!";
  }
  return euler;
}

void normalizeEuler(imitation_game::Joints &joints)
{
  joints.joints.at(0).yaw *= -1;
  joints.joints.at(0).roll *= -1;
  // joints.joints.at(1).roll *= -1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle nh;
  ros::Rate rate(1000);
  ros::AsyncSpinner spin(1);
  spin.start();

  ros::Publisher publisher = nh.advertise<imitation_game::Joints>("elbow_pose", 1000, true);
  ros::Publisher publisher2 = nh.advertise<imitation_game::Vector>("something_", 1000, true);
  RobotStateInformer *state = RobotStateInformer::getRobotStateInformer(nh);
  
  imitation_game::Joints all_joints;
  all_joints.joints.resize(4);
  // sensor_msgs::JointState jt_st;
  std::vector<double> jt_pos;
  while (ros::ok())
  {
    all_joints.joints.at(0) = getTransformation("/openni/left_elbow", "/openni/left_shoulder", "left_shoulder");
    all_joints.joints.at(1) = getTransformation("/openni/left_hand", "/openni/left_elbow", "left_elbow", true);
    all_joints.joints.at(2) = getTransformation("/openni/right_elbow", "/openni/right_shoulder", "right_shoulder");
    all_joints.joints.at(3) = getTransformation("/openni/right_hand", "/openni/right_elbow", "right_elbow", true);
    normalizeEuler(all_joints);
    state->getJointPositions(jt_pos);

    publisher.publish(all_joints);
    publisher2.publish(jt_pos);
    rate.sleep();
  }
  spin.stop();
  return 0;
}
