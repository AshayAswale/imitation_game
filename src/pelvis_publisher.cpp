#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "frame_publisher");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

tf::StampedTransform hip_transform_left, hip_transform_right;
tf::StampedTransform pelvis_transform;
tf::TransformListener hip_listener_left, hip_listener_right;
tf::TransformBroadcaster pelvis_broadcaster;
tf::Vector3 pelvis_vector;
tf::TransformListener skeleton_listener;
skeleton_listener.waitForTransform("/torso", "/head", ros::Time::now(), ros::Duration(1.0));

while (ros::ok())
{
  ROS_INFO_STREAM("publishing the frame");
  hip_listener_left.lookupTransform("/openni_depth_frame", "/left_hip", ros::Time(0.0), hip_transform_left);
  hip_listener_right.lookupTransform("/openni_depth_frame", "/right_hip", ros::Time(0.0), hip_transform_right);
  pelvis_vector = hip_transform_left.getOrigin() / 2 + hip_transform_right.getOrigin() / 2;
  pelvis_transform.setOrigin(pelvis_vector);
  // ROS_INFO_STREAM(pelvis_vector);
  pelvis_broadcaster.sendTransform(
      tf::StampedTransform(pelvis_transform, ros::Time::now(), "openni_depth_frame", "pelvis"));
  }
  return 0;
}
