#include <imitation_game/shadow_legs.h>

ShadowLegs::ShadowLegs(ros::NodeHandle nh) : nh_(nh)
{
  leg_controller_ = new LegControlInterface(nh_);
  pelvis_controller_ = new PelvisControlInterface(nh);
  robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
  rd_ = RobotDescription::getRobotDescription(nh_);
  setGroundPose();
  leg_pose_pelvis_.header.frame_id = rd_->getPelvisFrame();
}

ShadowLegs::~ShadowLegs()
{
  delete leg_controller_;
  delete pelvis_controller_;
}

void ShadowLegs::setGroundPose()
{
  ground_pose_.position.x = 0.0;
  ground_pose_.position.y = 0.16;
  ground_pose_.position.z = 0.1;
}

void ShadowLegs::startMotionController()
{
  ros::Rate rate(100.0);
  while (control_motion_)
  {
    current_leg_ = LegUpSide::LEFT;  //(current_leg_ == LegUpSide::LEFT) ? LegUpSide::RIGHT : LegUpSide::LEFT;
    update();
    control();
    // execute();
    rate.sleep();
  }
}

void ShadowLegs::startLegsShadowMotion()
{
  setHeightFactor();
  // thread_for_move_group_init_ = std::thread(&TaskspacePlanner::initializeMoveGroupsForCartesianPath, this);
  thread_for_shadow_motion_ = std::thread(&ShadowLegs::startMotionController, this);
}

void ShadowLegs::stopLegsShadowMotion()
{
  control_motion_ = false;
}

void ShadowLegs::setMotionTime(float time)
{
  motion_time_ = time;
}

float ShadowLegs::getMotionTime()
{
  return motion_time_;
}

void ShadowLegs::update()
{
  updateHumanLegUpStatus();
  updateRobotLegUpStatus();
  updateHumanFootTransform();
}

void ShadowLegs::updateHumanLegUpStatus()
{
  if(std::abs(std::abs(leg_transform_pelvis_.getOrigin().getZ())>=(human_height_-0.05)))
  {
    ROS_INFO("Human Both Legs Down");
    human_leg_up_side_ = LegUpSide::NONE;
  }
  else
  {
    ROS_INFO("Human Leg Up");
    human_leg_up_side_ = current_leg_;
  }
}

void ShadowLegs::updateRobotLegUpStatus()
{
  is_robot_leg_up = !robot_state_->isRobotInDoubleSupport();
}

void ShadowLegs::updateHumanFootTransform()
{
  ROS_INFO("Finding the Transform.");
  try
  {
  leg_listener_pelvis_.waitForTransform(pelvis_frame_, left_foot_frame_, ros::Time(0), ros::Duration(1.0));
  leg_listener_pelvis_.lookupTransform(pelvis_frame_, right_foot_frame_, ros::Time(0.0), leg_transform_pelvis_);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  // ROS_INFO("Transform Found!");
}

void ShadowLegs::setRobotLegGoal()
{
  leg_pose_pelvis_.pose = getPoseFromVector3(leg_transform_pelvis_.getOrigin());
  robot_state_->transformPose(leg_pose_pelvis_, leg_pose_world_, rd_->getWorldFrame());
  execute_once_ = true;
}

geometry_msgs::Pose ShadowLegs::getPoseFromVector3(const tf::Vector3& vector)
{
  static geometry_msgs::Pose pose;
  pose.position.x = -vector.getX();
  pose.position.y = -vector.getY();
  pose.position.z = vector.getZ();
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  return pose;
}

void ShadowLegs::setHeightFactor()
{
  updateHumanFootTransform();
  human_leg_length_ = leg_transform_pelvis_.getOrigin().length();
  human_height_ = std::abs(leg_transform_pelvis_.getOrigin().getZ());
  scale_factor_ = robot_leg_height_ / (human_leg_length_ + 0.05);
  ROS_ERROR("Height Factor %f", scale_factor_);
}

void ShadowLegs::control()
{
  if(isMotionExecutable())
    setRobotLegGoal();
}

bool ShadowLegs::isMotionExecutable()
{
  if((leg_transform_pelvis_.getOrigin().length() * scale_factor_) >= robot_leg_height_ + 0.1)
  {
    ROS_WARN("Out of reach point!");
    return false;
  }
  else if((human_leg_up_side_ != robot_leg_up_side_)&&(human_leg_up_side_ != LegUpSide::NONE))
  {
    ROS_WARN("Leg Change Detected!");
    shift_robot_support_leg = true;
    return false;
  }
  else
    return true;
}

void ShadowLegs::execute()
{
  if(shift_robot_support_leg)
    executeLegShift();
  else if (human_leg_up_side_ == LegUpSide::NONE)
    executePlaceLeg();
  else
    executeLegMotion();
}

void ShadowLegs::executeLegMotion()
{
  if (execute_once_)
  {
    robot_side_ = (current_leg_ == LegUpSide::LEFT) ? RobotSide::LEFT : RobotSide::RIGHT;
    leg_controller_->moveFoot(robot_side_, leg_pose_world_.pose, 1.0f);
    ROS_INFO_STREAM("Sending Command:");
    ros::Duration(1.0f).sleep();
    execute_once_ = false;
  }
}

void ShadowLegs::executePlaceLeg()
{
  if(is_robot_leg_up)
  {
    ROS_INFO("Placing Leg");
    robot_side_ = (current_leg_ == LegUpSide::LEFT) ? RobotSide::LEFT : RobotSide::RIGHT;
    leg_controller_->moveFoot(robot_side_, ground_pose_, 1.0);
    ros::Duration(1.0f).sleep();
    leg_controller_->placeLeg(robot_side_, 0.1, 0.5);
    ros::Duration(0.2f).sleep();
  }
}

void ShadowLegs::executeLegShift()
{
  executePlaceLeg();
  executeLegMotion();
}