#include "ros/ros.h"
#include <tough_stability/com.h>
#include <tough_common/robot_state.h>
#include <geometry_msgs/PolygonStamped.h>
// #include 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing_COM");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  RobotStateInformer* robot_state = RobotStateInformer::getRobotStateInformer(nh);
  RobotDescription* rd = RobotDescription::getRobotDescription(nh);
  sensor_msgs::JointState joint_state;
  ros::Publisher com_publisher = nh.advertise<geometry_msgs::PointStamped>("/center_of_mass", 1);
  ros::Publisher polygon_publisher = nh.advertise<geometry_msgs::PolygonStamped>("/support_polygon", 1);

  std::map<std::string, double> joint_positions;
  tf::Point com_wrt_root;
  double mass_robot = 181.82;

  ros::Duration(1.0).sleep();

  std::vector<std::string> left_arm_names, chest_names;
  rd->getLeftArmJointNames(left_arm_names);
  rd->getChestJointNames(chest_names);

  tf::Transform tf_right_foot, tf_left_foot;
  com COM(nh, rd->getPelvisFrame(), rd->getRightFootFrameName(), rd->getLeftFootFrameName(), true);

  geometry_msgs::Polygon polygon_points;
  geometry_msgs::PolygonStamped polygon_points_stamp;
  geometry_msgs::Point point_geom;
  geometry_msgs::PointStamped point_stamp;
  point_stamp.header.frame_id = rd->getPelvisFrame();
  polygon_points_stamp.header.frame_id = rd->getWorldFrame();

  while (ros::ok())
  {
    joint_positions.clear();
    ros::Duration(0.2).sleep();
    robot_state->getJointStateMessage(joint_state);
    for (std::size_t i = 0; i < joint_state.name.size(); i++)
    {
      // if(std::find(chest_names.begin(), chest_names.end(), joint_state.name[i]) != chest_names.end())
      //   joint_positions.insert({ joint_state.name[i], 0.34 });
      // else
      joint_positions.insert({ joint_state.name[i], joint_state.position[i] });
    }

    COM.computeCOM(joint_positions, com_wrt_root, mass_robot, tf_right_foot, tf_left_foot);
    tf::pointTFToMsg(com_wrt_root, point_geom);
    point_stamp.point = point_geom;
    point_stamp.point.z = -0.78;
    com_publisher.publish(point_stamp);

    robot_state->getSupportPolygon(polygon_points);
    polygon_points_stamp.polygon = polygon_points;
    polygon_publisher.publish(polygon_points_stamp);
    bool point_in = robot_state->isPointInSupportPolygon(point_geom);
    ROS_INFO_STREAM("COM IN STATUS  "<<point_in);
  }
  return 0;
}