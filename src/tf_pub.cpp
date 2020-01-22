#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <imitation_game/Joints.h>

imitation_game::Eulers getTransformation(const std::string& track_frame, const std::string& ref_frame, const std::string& name)
{
  imitation_game::Eulers euler;
  static tf::TransformListener listener;
  static tf::StampedTransform transform;
  try
  {
    listener.waitForTransform(track_frame, ref_frame, ros::Time(0.0), ros::Duration(0.25));
    listener.lookupTransform(track_frame, ref_frame, ros::Time(0.0), transform);
    euler.roll = atan(transform.getOrigin().getZ() / transform.getOrigin().getY());
    euler.pitch = atan(transform.getOrigin().getX() / transform.getOrigin().getZ());
    euler.yaw = atan(-transform.getOrigin().getX() / transform.getOrigin().getY());
    euler.name = name;
  }
  catch (const std::exception &e)
  {
    static int fail = 0;
    std::cerr << e.what() << '\n';
    ros::Duration(0.01).sleep();
    if (fail<5)
    {
      fail++;
      getTransformation(track_frame, ref_frame, name);
    }
    fail = 0;
    euler.name = "FAILED!!!";
  }
  return euler;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle nh;
  ros::Rate rate(500);
  ros::AsyncSpinner spin(1);
  spin.start();

  ros::Publisher publisher = nh.advertise<imitation_game::Joints>("elbow_pose", 1000, true);
  imitation_game::Joints all_joints;
  all_joints.joints.resize(4);
  while (ros::ok())
  {
    all_joints.joints.at(0) = getTransformation("/openni/left_elbow", "/openni/left_shoulder", "left_shoulder");
    all_joints.joints.at(1) = getTransformation("/openni/left_hand", "/openni/left_elbow", "left_elbow");
    all_joints.joints.at(2) = getTransformation("/openni/right_elbow", "/openni/right_shoulder", "right_shoulder");
    all_joints.joints.at(3) = getTransformation("/openni/right_hand", "/openni/right_elbow", "right_elbow");

    publisher.publish(all_joints);
    rate.sleep();
  }
  spin.stop();
  return 0;
}
