#include <imitation_game/shadow_upper_body.h>

ShadowUpperBody::ShadowUpperBody(ros::NodeHandle nh) : nh_(nh)
{
  initialize();
}

ShadowUpperBody::~ShadowUpperBody()
{
}
  
void ShadowUpperBody::initialize()
{
  rd_ = RobotDescription::getRobotDescription(nh_);
  robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
  wholebodyController_ = new WholebodyControlInterface(nh_);
}

void ShadowUpperBody::update()
{
  clearJointTrajectory();
  updateTranforms();
  resizeJointTrajectory();
}

void ShadowUpperBody::clearJointTrajectory()
{
  joint_trajectory_.header.frame_id = rd_->getPelvisFrame();
  joint_trajectory_.header.seq = 1;
  joint_trajectory_.joint_names.clear();
  joint_trajectory_.points.resize(1);
  joint_trajectory_.points.front().accelerations.clear();
  joint_trajectory_.points.front().effort.clear();
  joint_trajectory_.points.front().positions.clear();
  joint_trajectory_.points.front().velocities.clear();
  joint_trajectory_.points.front().time_from_start = ros::Duration(time_execution);
}

void ShadowUpperBody::resizeJointTrajectory()
{
  size_t size = joint_trajectory_.points.front().positions.size();
  joint_trajectory_.points.front().accelerations.resize(size);
  joint_trajectory_.points.front().effort.resize(size);
  joint_trajectory_.points.front().velocities.resize(size);
}

void ShadowUpperBody::updateTranforms()
{
  double yaw, roll;
  // trajectory_msgs::JointTrajectory joint_trajectory;
  tf::StampedTransform transform;
  for (auto i : frames_vector_)
  {
    transform = getTransform(i.data(), child_parent_frames_[i.data()]);
    getRollYaw(transform, roll, yaw);
    updateJointTrajectoryMsg(i.data(), roll, yaw);    
  }
}

void ShadowUpperBody::getRollYaw(const tf::StampedTransform& transform, double& roll, double& yaw)
{
  tfScalar pitch;
  tf::Matrix3x3 mat(transform.getRotation());
  mat.getRPY(roll, pitch, yaw);
}

void ShadowUpperBody::updateJointTrajectoryMsg(const std::string& frame_name, double roll, double yaw)
{
  std::string yaw_frame = frame_name + yaw_;
  std::string roll_frame = frame_name + roll_;
  
  string_map_iterator_ = human_robot_joint_map_.find(yaw_frame);
  if(string_map_iterator_ != human_robot_joint_map_.end())
  {
    joint_trajectory_.joint_names.push_back(human_robot_joint_map_[yaw_frame]);
    joint_trajectory_.points.front().positions.push_back(yaw);
  }
  else
  {
    ROS_ERROR("%s frame not found in map human_robot_joint_map_", yaw_frame.c_str());
  }
  
  string_map_iterator_ = human_robot_joint_map_.find(roll_frame);
  if(string_map_iterator_ != human_robot_joint_map_.end())
  {
    joint_trajectory_.joint_names.push_back(human_robot_joint_map_[roll_frame]);
    joint_trajectory_.points.front().positions.push_back(roll);
  }
  else
  {
    ROS_ERROR("%s frame not found in map human_robot_joint_map_", roll_frame);
  }
}

tf::StampedTransform ShadowUpperBody::getTransform(const std::string& frame_name, const std::string& parent_frame)
{
  static tf::StampedTransform latest_transform;
  try
  {
    tf_listener_.waitForTransform(parent_frame, frame_name, ros::Time(0.0), ros::Duration(0.5));
    tf_listener_.lookupTransform(parent_frame, frame_name, ros::Time(0.0), latest_transform);
    return latest_transform;
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    ros::Duration(0.01).sleep();
    getTransform(frame_name, parent_frame);
  }
}

void ShadowUpperBody::control()
{
  // TODO
}

void ShadowUpperBody::execute()
{
  wholebodyController_->executeTrajectory(joint_trajectory_);
}

void ShadowUpperBody::startShadowMotion()
{
  update();
  execute();
}