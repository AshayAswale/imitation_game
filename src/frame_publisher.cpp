#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>

// bool pause = true;

void joy_state(sensor_msgs::Joy joy_msg)
{
  // ROS_INFO_STREAM("In callback, calib 1 status:, " << joy_msg.buttons.at(0)
  //                                                  << " calib 2 status: " << joy_msg.buttons.at(3));
  if (joy_msg.buttons[0] & joy_msg.buttons[3])
  {
    ROS_WARN("CALIBRATING!!!");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "frame_publisher");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf::TransformListener hydra_listener_left, hydra_listener_right;
  tf::TransformBroadcaster kinect_hydra_link_left, kinect_hydra_link_right;
  tf::StampedTransform hydra_transform_left, hydra_transform_right;
  tf::StampedTransform hydra_transform_left_temp, hydra_transform_right_temp;

  hydra_listener_left.waitForTransform("/hydra_left_pivot", "/hydra_base",
                            ros::Time::now(), ros::Duration(3.0));

  std::string user_no = argc == 1 ? "1" : argv[1];
  std::string left_hand = "left_hand_", right_hand = "right_hand_";
  std::string left_palm = "left_palm_", right_palm = "right_palm_";

  ros::Subscriber subscribe = nh.subscribe("/hydra_joy", 0.01, joy_state);
  // ros::Subscriber subscribe = nh.subscribe("szdf", )

  while (ros::ok())
  {
    hydra_listener_left.lookupTransform("/hydra_base", "/hydra_left_pivot", ros::Time(0.0), hydra_transform_right);
    hydra_listener_right.lookupTransform("/hydra_base", "/hydra_right_pivot", ros::Time(0.0), hydra_transform_left);

    hydra_transform_left.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    hydra_transform_right.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

    kinect_hydra_link_left.sendTransform(
        tf::StampedTransform(hydra_transform_left, ros::Time::now(), left_hand + user_no, left_palm + user_no));
    kinect_hydra_link_right.sendTransform(
        tf::StampedTransform(hydra_transform_right, ros::Time::now(), right_hand + user_no, right_palm + user_no));
  }
  return 0;
}
