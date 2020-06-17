#include <imitation_game/shadow_upper_body.h>

ShadowUpperBody::ShadowUpperBody(ros::NodeHandle nh) : nh_(nh)
{
  initialize();
}

ShadowUpperBody::~ShadowUpperBody()
{
  run_code = false;
  delete wholebodyController_;
  delete joint_controller_;
}

void ShadowUpperBody::initialize()
{
  rd_ = RobotDescription::getRobotDescription(nh_);
  RobotStateInformer *state_inf = RobotStateInformer::getRobotStateInformer(nh_);
  ros::Duration(0.01).sleep();
  wholebodyController_ = new WholebodyControlInterface(nh_);
  armController_ = new ArmControlInterface(nh_);
  joint_controller_ = new JointAnglesController(nh_);
  chest_controller_ = new ChestControlInterface(nh_);

  rd_->getChestJointLimits(chest_limits);
  updateJointLimits();
  // chest_frame_vector.push_back("chest_names")
}

void ShadowUpperBody::updateJointLimits()
{
  std::vector<std::pair<double, double>> joint_limits_temp;

  rd_->getLeftArmJointNames(left_arm_names_);
  rd_->getLeftArmJointLimits(joint_limits_temp);
  insertJointLImits(left_arm_names_, joint_limits_temp);
  joint_limits_temp.clear();

  rd_->getRightArmJointNames(right_arm_names_);
  rd_->getRightArmJointLimits(joint_limits_temp);
  insertJointLImits(right_arm_names_, joint_limits_temp);
  joint_limits_temp.clear();

  rd_->getChestJointNames(chest_names_);
  rd_->getChestJointLimits(joint_limits_temp);
  insertJointLImits(chest_names_, joint_limits_temp);
  joint_limits_temp.clear();
}

void ShadowUpperBody::insertJointLImits(const std::vector<std::string> &joint_names_vector, const std::vector<std::pair<double, double>> &joint_limits_temp)
{
  for (int i = 0; i < joint_names_vector.size(); i++)
    joint_limits_map.insert(std::pair<std::string, std::pair<double, double>>(joint_names_vector.at(i), joint_limits_temp.at(i)));
}

void ShadowUpperBody::update()
{
  clearJointTrajectory();
  updateTranforms();
  insertRestJoints();
  resizeJointTrajectory();
}

void ShadowUpperBody::clearJointTrajectory()
{
  joint_trajectory_.header = std_msgs::Header();
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
  double yaw, roll, pitch;
  // trajectory_msgs::JointTrajectory joint_trajectory;
  tf::StampedTransform transform;
  for (auto group : frames_vector_)
  {
    // #### HARDCODING ####
    std::string frame = group.at(0);
    transform = getTransform(frame.data(), child_parent_frames_[frame.data()]);
    getRollPitchYaw(transform, roll, pitch, yaw);
    updateJointTrajectoryMsg(child_parent_frames_[frame.data()], roll, yaw);

    frame = group.at(1);
    transform = getTransform(frame.data(), child_parent_frames_[frame.data()]);
    getRollPitchYaw(transform, roll, pitch, yaw);
    updateJointTrajectoryMsg(child_parent_frames_[frame.data()], roll, pitch, yaw);
  }
  transform = getTransform(neck_frame_, child_parent_frames_[neck_frame_]);
  getRollPitchYaw(transform, roll, pitch, yaw);
  roll = atan(transform.getOrigin().getY() / transform.getOrigin().getZ());
  // ROS_INFO("Roll - %f", roll);
  updateJointTrajectoryMsg(child_parent_frames_[neck_frame_], -roll, pitch);
  // {
  //   std::vector<std::string> link = group.at(0);
  //   transform = getTransform(link.data(), child_parent_frames_[link.data()]);
  //   getRollPitchYaw(transform, roll, pitch, yaw);
  //   updateJointTrajectoryMsg(child_parent_frames_[link.data()], roll, pitch, yaw);

  // std::string frame_name = i.data();
  // if (frame_name.compare(left_wrist_dummy) == 0)
  // {
  //   for (int index = 4; index < 7; index++)
  //   {
  //     joint_trajectory_.joint_names.push_back(left_arm_names_.at(index));
  //     joint_trajectory_.points.front().positions.push_back(0);
  //   }
  //   continue;
  // }
  // else if(frame_name.compare(right_wrist_dummy)==0)
  // {
  //   for (int index = 4; index < 7; index++)
  //   {
  //     joint_trajectory_.joint_names.push_back(right_arm_names_.at(index));
  //     joint_trajectory_.points.front().positions.push_back(0);
  //   }
  //   continue;
  // }
  // transform = getTransform(i.data(), child_parent_frames_[i.data()]);
  // getRollPitchYaw(transform, roll, pitch, yaw);
  // updateJointTrajectoryMsg(child_parent_frames_[i.data()], roll, pitch, yaw);
  // }
}

void ShadowUpperBody::getRollPitchYaw(const tf::StampedTransform &transform, double &roll, double &pitch, double &yaw)
{
  // tf::Matrix3x3 mat(transform.getRotation());
  // mat.getRPY(roll, pitch, yaw);
  yaw = atan(-transform.getOrigin().getX() / transform.getOrigin().getY());
  roll = atan(transform.getOrigin().getZ() / transform.getOrigin().getY());
  pitch = atan(transform.getOrigin().getX() / transform.getOrigin().getZ());
}

void ShadowUpperBody::updateJointTrajectoryMsg(const std::string &frame_name, double roll, double pitch, double yaw)
{
  int right_compensate = 1;
  std::string yaw_frame = frame_name + yaw_;
  std::string roll_frame = frame_name + roll_;
  std::string pitch_frame = frame_name + pitch_;
  // DO NOT CHANGE THE SEQUENCE

  // std::cout <<frame_name<< "  pitch: " << pitch << "   roll: " << std::max(std::abs(yaw), std::abs(roll)) << std::endl;
  if (frame_name.at(7) == 'r')
    right_compensate = -1;

  addToJointTrajectory(frame_name, pitch_frame, pitch);
  addToJointTrajectory(frame_name, roll_frame, right_compensate * std::max(std::abs(yaw), std::abs(roll)));
}

void ShadowUpperBody::updateJointTrajectoryMsg(const std::string &frame_name, double roll, double yaw)
{
  std::string yaw_frame = frame_name + yaw_;
  std::string roll_frame = frame_name + roll_;

  // DO NOT CHANGE THE SEQUENCE
  //   std::cout <<frame_name<< "  yaw: " << yaw << "   roll: " << std::max(std::abs(yaw), std::abs(roll)) << std::endl;

  addToJointTrajectory(frame_name, yaw_frame, yaw);
  addToJointTrajectory(frame_name, roll_frame, roll);
}

void ShadowUpperBody::addToJointTrajectory(const std::string &frame_name, const std::string &rotation_frame, double rotation)
{
  std::pair<double, double> joint_limit_temp;

  string_map_iterator_ = human_robot_joint_map_.find(rotation_frame);
  if (string_map_iterator_ != human_robot_joint_map_.end())
  {
    std::string robot_rot_frame = human_robot_joint_map_[rotation_frame];
    joint_limits_iterator_ = joint_limits_map.find(robot_rot_frame);
    if (joint_limits_iterator_ != joint_limits_map.end())
    {
      joint_limit_temp = joint_limits_map[robot_rot_frame];
      if (rotation < joint_limit_temp.first)
        rotation = joint_limit_temp.first;
      else if (rotation > joint_limit_temp.second)
        rotation = joint_limit_temp.second;
    }
    joint_trajectory_.joint_names.push_back(robot_rot_frame);
    if (robot_rot_frame == chest_names_.at(1))
    {
      chest_pitch_index_ = joint_trajectory_.joint_names.size() - 1;
    }
    else if (robot_rot_frame == chest_names_.at(2))
    {
      chest_roll_index_ = joint_trajectory_.joint_names.size() - 1;
    }

    joint_trajectory_.points.front().positions.push_back(rotation);
  }
  else
  {
    ROS_ERROR("%s frame not found in map human_robot_joint_map_", rotation_frame.c_str());
  }
}

tf::StampedTransform ShadowUpperBody::getTransform(const std::string &frame_name, const std::string &parent_frame)
{
  static tf::StampedTransform latest_transform;
  try
  {
    tf_listener_.waitForTransform(parent_frame, frame_name, ros::Time(0.0), ros::Duration(0.5));
    tf_listener_.lookupTransform(parent_frame, frame_name, ros::Time(0.0), latest_transform);
    return latest_transform;
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    ros::Duration(0.01).sleep();
    getTransform(frame_name, parent_frame);
  }
}

void ShadowUpperBody::insertRestJoints()
{
  joint_trajectory_.joint_names.insert(joint_trajectory_.joint_names.end(), rest_joints.begin(), rest_joints.end());
  for (auto &pts : joint_trajectory_.points)
    pts.positions.resize(joint_trajectory_.joint_names.size());
}

void ShadowUpperBody::control()
{
  joint_controller_->updateJointAccelerations(joint_trajectory_);
}

void ShadowUpperBody::execute()
{
  wholebodyController_->executeTrajectory(joint_trajectory_);
  // armController_->
  // ArmControlInterface double roll = 0.8 * (joint_trajectory_.points.front().positions.at(chest_roll_index_));
  // if (roll < chest_limits.at(2).first)
  // {
  //   roll = chest_limits.at(2).first;
  // }
  // else if (roll > chest_limits.at(2).second)
  // {
  //   roll = chest_limits.at(2).second;
  // }

  // double pitch = 0.8 * (joint_trajectory_.points.front().positions.at(chest_pitch_index_));
  // if (pitch < chest_limits.at(1).first)
  // {
  //   pitch = chest_limits.at(1).first;
  // }
  // else if (pitch > chest_limits.at(1).second)
  // {
  //   pitch = chest_limits.at(1).second;
  // }

  // double yaw = 0;
  // chest_controller_->controlChest(roll, pitch, yaw, 0.75);
  // ros::Duration(0.02).sleep();
}

void ShadowUpperBody::startShadowMotion()
{
  while (run_code && ros::ok())
  {
    update();
    control();
    execute();
  }
}