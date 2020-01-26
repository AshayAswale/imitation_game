#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <imitation_game/Joints.h>

geometry_msgs::Point getTransformation(const std::string &track_frame, const std::string &ref_frame, const std::string &name, const bool is_op = false)
{
  static geometry_msgs::Point point;
  static tf::TransformListener listener;
  static tf::StampedTransform transform;
  try
  {
    listener.waitForTransform(track_frame, ref_frame, ros::Time(0.0), ros::Duration(0.25));
    listener.lookupTransform(track_frame, ref_frame, ros::Time(0.0), transform);
    // tf::Matrix3x3 mat(transform.getRotation());
    // mat.getRPY(euler.roll, euler.pitch, euler.yaw);

    // if (elb_comp)
    //   euler.roll = std::max(std::abs(euler.yaw), std::abs(euler.roll));
    float scale = is_op ? -0.83 : 1;
    point.x = transform.getOrigin().getX() * scale;
    point.y = transform.getOrigin().getY() * scale;
    point.z = transform.getOrigin().getZ() * scale;
  }
  catch (const std::exception &e)
  {
    static int fail = 0;
    std::cerr << e.what() << '\n';
    ros::Duration(0.01).sleep();
    if (fail < 5)
    {
      fail++;
      getTransformation(track_frame, ref_frame, name);
    }
    fail = 0;
    // euler.name = "FAILED!!!";
  }
  return point;
}

void normalizeEuler(imitation_game::Joints &joints)
{
  joints.joints.at(0).yaw *= -1;
  joints.joints.at(0).roll *= -1;
  // joints.joints.at(1).roll *= -1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle nh;
  ros::Rate rate(500);
  ros::AsyncSpinner spin(1);
  spin.start();

  ros::Publisher op_publisher = nh.advertise<geometry_msgs::Point>("operator_leg_pose", 10, true);
  ros::Publisher rob_publisher = nh.advertise<geometry_msgs::Point>("robot_leg_pose", 10, true);
  geometry_msgs::Point operator_leg, robot_leg;

  while (ros::ok())
  {
    operator_leg = getTransformation("/openni/left_foot", "/openni/right_foot", "operator_left_leg", true);
    robot_leg = getTransformation("/l_foot", "/r_foot", "robot_left_leg");

    operator_leg.x *= -1;
    op_publisher.publish(operator_leg);
    rob_publisher.publish(robot_leg);
    rate.sleep();
  }
  spin.stop();
  return 0;
}
