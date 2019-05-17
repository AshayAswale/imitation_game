#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>

tf::StampedTransform hydra_transform_left, hydra_transform_right;
tf::Quaternion left_quat_calib, right_quat_calib;

void joy_state(sensor_msgs::Joy joy_msg)
{
  // ROS_INFO_STREAM("In callback, calib 1 status:, " << joy_msg.buttons.at(0)
  //                                                  << " calib 2 status: " << joy_msg.buttons.at(3));
  static int counter = 0;
  if(counter == 700) {
  if (joy_msg.buttons[0] & joy_msg.buttons[3])
  {
    ROS_WARN("CALIBRATING!!!");
    left_quat_calib = hydra_transform_left.getRotation();
    right_quat_calib = hydra_transform_right.getRotation();
  }
  counter = 0;
  }
  ++counter;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "frame_publisher");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf::TransformListener hydra_listener_left, hydra_listener_right, skeleton_listener;
  tf::TransformBroadcaster kinect_hydra_link_left, kinect_hydra_link_right;
  tf::StampedTransform hydra_transform_left_temp, hydra_transform_right_temp;
  left_quat_calib.setW(1);
  right_quat_calib.setW(1);

  hydra_listener_left.waitForTransform("/hydra_left_pivot", "/hydra_base",
                            ros::Time::now(), ros::Duration(3.0));

  skeleton_listener.waitForTransform("/torso", "/head",ros::Time::now(), ros::Duration(3.0));

  std::string user_no = argc == 1 ? "1" : argv[1];
  std::string left_hand = "left_hand", right_hand = "right_hand";
  std::string left_palm = "left_palm", right_palm = "right_palm";

  ros::Subscriber subscribe_hydra = nh.subscribe("/hydra_joy", 0.01, joy_state);

  while (ros::ok())
  {
    hydra_listener_left.lookupTransform("/hydra_base", "/hydra_left_pivot", ros::Time(0.0), hydra_transform_right);
    hydra_listener_right.lookupTransform("/hydra_base", "/hydra_right_pivot", ros::Time(0.0), hydra_transform_left);

    hydra_transform_left_temp = hydra_transform_left;
    hydra_transform_right_temp = hydra_transform_right;

    hydra_transform_left_temp.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    hydra_transform_right_temp.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

    hydra_transform_left_temp.setRotation(hydra_transform_left.getRotation()*left_quat_calib.inverse());
    hydra_transform_right_temp.setRotation(hydra_transform_right.getRotation()*right_quat_calib.inverse());

    kinect_hydra_link_left.sendTransform(
        tf::StampedTransform(hydra_transform_left_temp, ros::Time::now(), left_hand, left_palm));
    kinect_hydra_link_right.sendTransform(
        tf::StampedTransform(hydra_transform_right_temp, ros::Time::now(), right_hand, right_palm));
  }
  return 0;
}
