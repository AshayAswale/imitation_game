#include <tough_controller_interface/leg_control_interface.h>
#include <tough_controller_interface/pelvis_control_interface.h>
#include <tough_common/robot_state.h>
#include <tf/transform_listener.h>

bool start_code = false;
bool calib_height = true;
float factor = 1.0f;

void calib_height_CB(std_msgs::Bool calib_message)
{
  calib_height = true;
}

void start_code_CB(std_msgs::Bool start_message)
{
  start_code = start_message.data;
}

float get_height_factor(tf::StampedTransform leg_transform_pelvis)
{
  ROS_WARN("Setting Factor!");
  float z_height = leg_transform_pelvis.getOrigin().getZ();
  float factor = 0.79 / z_height;
  calib_height = false;
  return factor;
}

geometry_msgs::Pose getPoseFromVector3(const tf::Vector3& vector, float height_factor)
{
  geometry_msgs::Pose pose;
  pose.position.x = -vector.getX();
  pose.position.y = -vector.getY();
  pose.position.z = vector.getZ();
  pose.orientation.w = 1.0;
  return pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leg_trial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber start_code_sub = nh.subscribe("/start_code", 0.01, start_code_CB);
  ros::Subscriber calib_height_sub = nh.subscribe("/calib_height", 0.01, calib_height_CB);

  tf::TransformListener left_leg_listener_pelvis;
  tf::StampedTransform left_leg_transform_pelvis;

  LegControlInterface leg_controller(nh);
  PelvisControlInterface pelvis_controller(nh);
  RobotStateInformer* robot_state = RobotStateInformer::getRobotStateInformer(nh);
  RobotDescription* rd = RobotDescription::getRobotDescription(nh);

  float height_factor = 1.0;
  tf::Vector3 leg_position;
  geometry_msgs::PoseStamped leg_pose_pelvis;
  geometry_msgs::PoseStamped leg_pose_world;
  geometry_msgs::Pose ground_pose;
  ground_pose.position.x = 0.0;
  ground_pose.position.y = 0.16;
  ground_pose.position.z = 0.1;

  leg_pose_pelvis.header.frame_id = rd->getPelvisFrame();

  left_leg_listener_pelvis.waitForTransform("/openni/pelvis", "/openni/right_foot", ros::Time::now(),
                                            ros::Duration(1.0));

  pelvis_controller.controlPelvisHeight(0.79f, 2.0);
  ros::Duration(0.5f).sleep();

  // while(ros::ok && !start_code)
  //   ros::Duration(0.5f).sleep();

  left_leg_listener_pelvis.lookupTransform("/openni/pelvis", "/openni/right_foot", ros::Time(0.0),
                                           left_leg_transform_pelvis);
  if (calib_height)
    height_factor = get_height_factor(left_leg_transform_pelvis);

  while (ros::ok)
  {
    left_leg_listener_pelvis.lookupTransform("/openni/pelvis", "/openni/right_foot", ros::Time(0.0),
                                             left_leg_transform_pelvis);
                                             
    leg_pose_pelvis.pose = getPoseFromVector3(left_leg_transform_pelvis.getOrigin(), height_factor);
    robot_state->transformPose(leg_pose_pelvis, leg_pose_world, rd->getWorldFrame());

    if (leg_pose_world.pose.position.z > 0.12)
    {
      if(left_leg_transform_pelvis.getOrigin().length()<0.8)
      {
        leg_controller.moveFoot(RobotSide::LEFT, leg_pose_world.pose, 1.0f);
        ROS_INFO_STREAM("Sending Command:");
        // ROS_INFO_STREAM("Sending Command: \n X: " << leg_pose_world.pose.position.x
        //                                           << "\n Y: " << leg_pose_world.pose.position.y
        //                                           << "\n Z: " << leg_pose_world.pose.position.z);
        ros::Duration(1.0f).sleep();
      }
      else
      {
        continue;
      }
    }
    else if (robot_state->isRobotInDoubleSupport())
    {
      // ROS_INFO("Skipping.");
      // ros::Duration(1.0f).sleep();
      continue;
    }
    else
    {
      ROS_INFO("Placing leg");
      leg_controller.moveFoot(RobotSide::LEFT, ground_pose, 1.0);
      ros::Duration(1.0f).sleep();
      leg_controller.placeLeg(RobotSide::LEFT, 0.1, 0.5);
      ros::Duration(0.2f).sleep();
    }
  }
  spinner.stop();
  return 0;
}
