#include <imitation_game/shadow_legs.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_shadow_legs");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RobotStateInformer *robot_state_ = RobotStateInformer::getRobotStateInformer(nh);
  ros::Duration(0.5).sleep();

  ShadowLegs shadow_legs(nh);
  visualization_msgs::MarkerArray markerArray;
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

  shadow_legs.startLegsShadowMotion();

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/openni/left_foot";
  marker.header.stamp = ros::Time();
  // marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05; // Shaft Diameter
  marker.scale.y = 0.1;  // Head Diameter
  marker.scale.z = 0.07; // Head Length
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.points.resize(2);
  marker.points.at(0).x = 0;
  marker.points.at(0).y = 0;
  marker.points.at(0).z = 0;

  markerArray.markers.resize(3);
  markerArray.markers.at(0) = marker;
  markerArray.markers.at(1) = marker;
  markerArray.markers.at(2) = marker;

  markerArray.markers.at(0).id = 1;
  markerArray.markers.at(1).id = 2;
  markerArray.markers.at(1).id = 3;

  markerArray.markers.at(1).color.b = 1.0;
  markerArray.markers.at(1).color.g = 0.0;

  markerArray.markers.at(2).color.b = 0.0;
  markerArray.markers.at(2).color.g = 0.0;
  markerArray.markers.at(2).color.r = 1.0;

  // markerArray.markers.at(0).points.resize(2);
  // markerArray.markers.at(0).points.at(0).x = 0;
  // markerArray.markers.at(0).points.at(0).y = 0;
  // markerArray.markers.at(0).points.at(0).z = 0;

  // markerArray.markers.at(0).points.at(1).x = 0.1;
  // markerArray.markers.at(0).points.at(1).y = 0.1;
  // markerArray.markers.at(0).points.at(1).z = 0.1;
  // markerArray.markers.at(1) = marker;

  while (ros::ok())
  {
    // shadow_legs.getGBR(markerArray);
    shadow_legs.getMarkerVectors(markerArray);
    vis_pub.publish(markerArray);
    ros::Duration(0.001).sleep();
  }

  shadow_legs.stopLegsShadowMotion();

  spinner.stop();
  return 0;
}
