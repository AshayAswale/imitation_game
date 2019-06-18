#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Scalar.h>

int vector_length = 200;

tf::Quaternion filterQuaternion(const std::vector<std::vector<float>>& quater_vec)
{
  tf::Quaternion final_quat;
  static float x_avg, y_avg, z_avg, w_avg;
  x_avg = 0, y_avg = 0, z_avg = 0, w_avg = 0;
  for (int i = 0; i < quater_vec.at(0).size(); i++)
  {
    x_avg += quater_vec.at(0).at(i) / quater_vec.at(0).size();
    y_avg += quater_vec.at(1).at(i) / quater_vec.at(1).size();
    z_avg += quater_vec.at(2).at(i) / quater_vec.at(2).size();
    w_avg += quater_vec.at(3).at(i) / quater_vec.at(3).size();
  }

  final_quat.setX(x_avg);
  final_quat.setY(y_avg);
  final_quat.setZ(z_avg);
  final_quat.setW(w_avg);

  final_quat.normalize();
  // ROS_INFO_STREAM("normalized: X:" << final_quat.getX() << "  Y:" << final_quat.getY() << "  Z:" << final_quat.getZ()
  //                                  << "  W:" << final_quat.getW());
  return final_quat;
}

void copyValues(const std::vector<std::vector<float>>& quater_vec, std::vector<std::vector<float>>& quater_vec_copy)
{
  for (int i = 0; i < quater_vec.size(); i++)
    quater_vec_copy.at(i) = quater_vec.at(i);
}

void copyValues(std::vector<std::vector<float>>& quater_vec, tf::StampedTransform& hip_transform, const int i)
{
  quater_vec.at(0).at(i) = hip_transform.getRotation().getX();
  quater_vec.at(1).at(i) = hip_transform.getRotation().getY();
  quater_vec.at(2).at(i) = hip_transform.getRotation().getZ();
  quater_vec.at(3).at(i) = hip_transform.getRotation().getW();
}

void getValuesIntoVec(std::vector<std::vector<float>>& quater_vec, tf::StampedTransform& hip_transform, const int i)
{
  if (i < vector_length - 1)
  {
    copyValues(quater_vec, hip_transform, i);
    return;
  }
  else
  {
    for (int j = 0; j < quater_vec.size(); j++)
    {
      std::rotate(quater_vec.at(j).begin(), quater_vec.at(j).begin() + 1, quater_vec.at(j).end());
    }
    copyValues(quater_vec, hip_transform, i);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "frame_publisher");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf::StampedTransform hip_transform_left, hip_transform_right;
  tf::StampedTransform filtered_hip_transform;
  tf::TransformListener hip_listener_left, hip_listener_right;
  tf::TransformBroadcaster filtered_hip_broadcaster;
  
  tf::Vector3 pelvis_vector;
  tf::TransformListener skeleton_listener;
  skeleton_listener.waitForTransform("/torso", "/head", ros::Time::now(), ros::Duration(1.0));
  bool init_ten_values = true;

  ros::Rate rate(500.0);

  ros::Publisher filtered_hip_publisher = nh.advertise<tf2_msgs::TFMessage>("/tf_filtered", 500);

  // std::vector<tf::Quaternion> quater_left_vec(vector_length), quater_right_vec(vector_length);
  std::vector<float> x(vector_length), y(vector_length), z(vector_length), w(vector_length);
  std::vector<std::vector<float>> quater_left_vec = { x, y, z, w };
  std::vector<std::vector<float>> quater_copy_vec(quater_left_vec);
  static int i = 0;
  tf::Quaternion final_quat;
  tf2_msgs::TFMessage tf_message;
  
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = "openni_depth_frame";
  transform_stamped.child_frame_id = "filtered_frame";

  tf_message.transforms.push_back(transform_stamped);
  while (ros::ok())
  {
    hip_listener_left.lookupTransform("/openni_depth_frame", "/left_hip", ros::Time(0.0), hip_transform_left);
    hip_listener_right.lookupTransform("/openni_depth_frame", "/right_hip", ros::Time(0.0), hip_transform_right);

    getValuesIntoVec(quater_left_vec, hip_transform_left, i);
    i = (i < vector_length - 1) ? i + 1 : vector_length - 1;
    copyValues(quater_left_vec, quater_copy_vec);
    final_quat = filterQuaternion(quater_copy_vec);

    filtered_hip_transform.setOrigin(hip_transform_left.getOrigin());
    filtered_hip_transform.setRotation(final_quat);

    tf_message.transforms.at(0).transform.rotation.w = final_quat.getW();
    tf_message.transforms.at(0).transform.rotation.x = final_quat.getX();
    tf_message.transforms.at(0).transform.rotation.y = final_quat.getY();
    tf_message.transforms.at(0).transform.rotation.z = final_quat.getZ();

    tf_message.transforms.at(0).transform.translation.x = hip_transform_left.getOrigin().getX();
    tf_message.transforms.at(0).transform.translation.y = hip_transform_left.getOrigin().getY();
    tf_message.transforms.at(0).transform.translation.z = hip_transform_left.getOrigin().getX();

    filtered_hip_broadcaster.sendTransform(
        tf::StampedTransform(filtered_hip_transform, ros::Time::now(), "openni_depth_frame",
        "filtered_left_hip"));
    tf_message.transforms.at(0).header.stamp.sec = ros::Time::now().sec;
    tf_message.transforms.at(0).header.stamp.nsec = ros::Time::now().nsec;
    filtered_hip_publisher.publish(tf_message);

    ROS_INFO_STREAM("X diff: " << filtered_hip_transform.getRotation().getX() -
                                      hip_transform_left.getRotation().getX());
    rate.sleep();
  }
  return 0;
}
