#include <imitation_game/shadow_legs.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_shadow_legs");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ShadowLegs shadow_legs(nh);
  visualization_msgs::MarkerArray markerArray;
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

  shadow_legs.startLegsShadowMotion();

  visualization_msgs::Marker marker;
  markerArray.markers.resize(2);
  marker.header.frame_id = "/openni/pelvis";
  marker.header.stamp = ros::Time();
  // marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 0.5; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  markerArray.markers.at(0) = marker;

  marker.id = 1;
  marker.color.b = 1.0;
  marker.color.g = 0.0;

  markerArray.markers.at(1) = marker;

  while (ros::ok())
  {
    // shadow_legs.getGBR(markerArray);
    vis_pub.publish(markerArray);
    ros::Duration(0.001).sleep();
  }

  shadow_legs.stopLegsShadowMotion();

  spinner.stop();
  return 0;
}
