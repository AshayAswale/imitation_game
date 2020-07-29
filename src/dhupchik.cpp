#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <tough_common/robot_state.h>
#include <razer_hydra/HydraRaw.h>
#include <time.h>
#include <tough_common/robot_state.h>
#include <time.h>

bool hydra_register_trig_ = false;

void hydraTriggerCB(const razer_hydra::HydraRaw &msg)
{
  static bool change_of_trigger = true;
  if (msg.buttons.at(1) == 2 && msg.buttons.at(0) == 16)
  {
    if (change_of_trigger)
    {
      hydra_register_trig_ = true;
      change_of_trigger = false;
    }
  }
  else
    change_of_trigger = true;
}

double getRandom()
{
  return ((double)rand() / (RAND_MAX));
}

void populatePose(geometry_msgs::Pose &pose)
{
  pose.position.x = (8 + (getRandom() * 4)) / 100;
  pose.position.y = (-1 + (getRandom() * 2)) / 100;
  pose.position.z = (-1 + (getRandom() * 2)) / 100;
  pose.orientation.w = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_register");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::srand(time(NULL));

  std::string file_name = "experiment_data.csv";
  std::ofstream myfile(file_name);
  int i = 0;
  ros::Subscriber sub = nh.subscribe("/hydra_raw", 1, hydraTriggerCB);
  RobotStateInformer *state_informer = RobotStateInformer::getRobotStateInformer(nh);
  RobotDescription *rd = RobotDescription::getRobotDescription(nh);
  ros::Duration(1).sleep();
  geometry_msgs::PoseStamped pose_object;
  pose_object.header.frame_id = rd->getRightEEFrame();
  double r;
  myfile << "All readings with respect to the pelvis frame\n";
  myfile << "sr no, obj x, obj y, obj z, palm x, palm y, palm z\n";
  while (ros::ok())
  {
    if (hydra_register_trig_)
    {
      myfile << i++ << ",";
      geometry_msgs::Pose pose_palm;
      geometry_msgs::Pose pose_obj_palm, pose_obj_pelvis;
      populatePose(pose_obj_palm);
      state_informer->transformPose(pose_obj_palm, pose_obj_pelvis, rd->getRightEEFrame(), rd->getPelvisFrame());
      state_informer->getCurrentPose(rd->getRightEEFrame(), pose_palm);

      // Object position wrt pelvis
      myfile << pose_obj_pelvis.position.x << "," << pose_obj_pelvis.position.y << "," << pose_obj_pelvis.position.z << ",";

      // end effector wrt pelvis for object
      myfile << pose_palm.position.x << "," << pose_palm.position.y << "," << pose_palm.position.z << "\n";

      hydra_register_trig_ = false;
      ros::Duration(0.1).sleep();
      ROS_INFO("Entering data");
    }
  }
  ROS_INFO("YEAAAAA");
  myfile.close();

  return 0;
}
