#include <imitation_game/shadow_legs.h>
#include <math.h>

ShadowLegs::ShadowLegs(ros::NodeHandle nh) : nh_(nh)
{
  leg_controller_ = new LegControlInterface(nh_);
  pelvis_controller_ = new PelvisControlInterface(nh);
  robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
  rd_ = RobotDescription::getRobotDescription(nh_);
  setPelvisInitialHeight();
}

ShadowLegs::~ShadowLegs()
{
  stopLegsShadowMotion();
  delete leg_controller_;
  delete pelvis_controller_;
}

void ShadowLegs::setPelvisInitialHeight()
{
  tf::StampedTransform robot_pelvis_transform;
  robot_pelvis_transform = getTransform(rd_->getPelvisFrame(), rd_->getWorldFrame());
  robot_pelvis_init_height_ = robot_pelvis_transform.getOrigin().getZ();  
}

/**
 * @brief Main Starting Function
 * 
 */
void ShadowLegs::startLegsShadowMotion()
{
  setCalibValues();
  thread_for_shadow_motion_ = std::thread(&ShadowLegs::startMotionController, this);
}

tf::StampedTransform ShadowLegs::getTransform(const std::string& foot_frame, const std::string& ref_frame)
{
  static tf::StampedTransform latest_transform;
  try
  {
  leg_listener_pelvis_.waitForTransform(ref_frame, foot_frame, ros::Time(0.0), ros::Duration(1.0));
  leg_listener_pelvis_.lookupTransform(ref_frame, foot_frame, ros::Time(0.0), latest_transform);
  return latest_transform;
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    ros::Duration(0.01).sleep();
    getTransform(foot_frame, ref_frame);
  }
}

void ShadowLegs::setCalibValues()
{
  updateLegsTransform();
  updateLegsTransform();
  left_leg_init_ = left_leg_transform_.getOrigin();
  right_leg_init_ = right_leg_transform_.getOrigin();
  pelvis_init_ = pelvis_transform_.getOrigin();
  
  setAlphaValue();
}

void ShadowLegs::startMotionController()
{
  ros::Rate rate(100.0);
  while (control_motion_)
  {
    updateLegsTransform();
    if(isOperatorInDoubleSupport())
    {
      if (isRobotInDoubleSupport())
      {
        setPelvisHeight();
      }
      else
      {
        ROS_INFO("Placing Leg Down.");
        placeLegDown();
      }
    }
    else
    {
      ROS_INFO("Operator in Swing Phase");
      if (isPoseReachable())
      {
        ROS_INFO("Executing Motion");
        moveLeg();
      }
      else
      {
        ROS_WARN("Out of Reach!");
      }
    }         
    // rate.sleep();
  }
}

bool ShadowLegs::isOperatorInDoubleSupport()
{
  static bool left_leg_in_gbr, right_leg_in_gbr;
  left_leg_in_gbr = isLegInGbr(left_foot_frame_);
  right_leg_in_gbr = isLegInGbr(right_foot_frame_);

  if(left_leg_in_gbr && right_leg_in_gbr)
  {
    swing_leg_side = LegUpSide::NONE;
    return true;
  }
  else if (!left_leg_in_gbr && !right_leg_in_gbr)
  {
    return false;
  }
  else
  {
    swing_leg_side = (!left_leg_in_gbr) ? LegUpSide::LEFT : LegUpSide::RIGHT;
    return false;
  }
}

bool ShadowLegs::isLegInGbr(std::string leg_frame)
{
  static tf::StampedTransform* current_leg_transform;
  static tf::Vector3* current_leg_init_vector;

  static float delta_x, delta_y, delta_z;
  
  
  current_leg_transform = (leg_frame.compare(left_foot_frame_) == 0) ? &left_leg_transform_ : &right_leg_transform_;
  current_leg_init_vector = (leg_frame.compare(left_foot_frame_) == 0) ? &left_leg_init_ : &right_leg_init_;

  delta_x = std::abs(current_leg_transform->getOrigin().getX() - current_leg_init_vector->getX());
  delta_y = std::abs(current_leg_transform->getOrigin().getY() - current_leg_init_vector->getY());
  delta_z = std::abs((current_leg_transform->getOrigin().getZ() + delta_P) - current_leg_init_vector->getZ());

  if (delta_x<psi_x_/2&&delta_z<psi_z_/2)
  {
    return true;
    // ROS_WARN("%s Inside ")
  }
  else
    return false;
}

void ShadowLegs::updateLegsTransform()
{
  left_leg_transform_ = getTransform(left_foot_frame_, pelvis_frame_);
  right_leg_transform_ = getTransform(right_foot_frame_, pelvis_frame_);
  pelvis_transform_ = getTransform(pelvis_frame_, openni_base_frame_);
  delta_P = pelvis_transform_.getOrigin().getZ() - pelvis_init_.getZ();
}

void ShadowLegs::setPelvisHeight()
{
  static tf::StampedTransform robot_pelvis_transform;
  static float current_delta_P, current_height;
  robot_pelvis_transform = getTransform(rd_->getPelvisFrame(), rd_->getWorldFrame());
  current_height = robot_pelvis_transform.getOrigin().getZ();
  current_delta_P = current_height - robot_pelvis_init_height_;
  if (std::abs(delta_P*alpha_ - current_delta_P) > pelvis_threshold_)
  {
    if (robot_pelvis_init_height_ + delta_P > pelvis_min_height )
    {
      ROS_INFO("delta_P=%f", delta_P);
      // {
      //   ROS_INFO("WTF! %f", robot_pelvis_init_height_);
      //   executePelvisHeight(robot_pelvis_init_height_);
      // }
      // else
      if (delta_P < 0)
        executePelvisHeight(robot_pelvis_init_height_ + delta_P);
    }
    // else if (current_height > pelvis_min_height)
    // {
    //   executePelvisHeight(pelvis_min_height);
    // }
  }
}

void ShadowLegs::executePelvisHeight(float height)
{
  ROS_INFO("Setting Pelvis Height to %f", height);
  pelvis_controller_->controlPelvisHeight(height, pelvis_motion_time_);
  ros::Duration(0.1).sleep();  
}

void ShadowLegs::placeLegDown()
{
  std::string foot_frame_name;
  RobotSide side;
  geometry_msgs::Pose foot_goal_pose;
  float offset = rd_->getFootFrameOffset() + 0.01;
  tf::StampedTransform robot_foot_transform = getTransform(rd_->getLeftFootFrameName(), rd_->getWorldFrame());
  if (robot_foot_transform.getOrigin().getZ() > (rd_->getFootFrameOffset() + 0.01))
  {
    side = RobotSide::LEFT;
    robot_foot_transform = getTransform(rd_->getRightFootFrameName(), rd_->getWorldFrame());
    foot_goal_pose.position.y = robot_foot_transform.getOrigin().getY() + 0.32;
  }
  else
  {
    side = RobotSide::RIGHT;
    foot_goal_pose.position.y = robot_foot_transform.getOrigin().getY() - 0.32;
  }
  foot_goal_pose.orientation.w = 1.0;
  foot_goal_pose.position.x = robot_foot_transform.getOrigin().getX();
  foot_goal_pose.position.z = offset;
  leg_controller_->moveFoot(side, foot_goal_pose, motion_time_);
  ros::Duration(motion_time_).sleep();

  leg_controller_->placeLeg(side, offset, 0.5);
  ros::Duration(0.5).sleep();
}

void ShadowLegs::getGBR(visualization_msgs::MarkerArray& markerArray)
{

  markerArray.markers.at(0).pose.position.x = left_leg_init_.getX();
  markerArray.markers.at(0).pose.position.y = left_leg_init_.getY();
  markerArray.markers.at(0).pose.position.z = left_leg_init_.getZ() - delta_P;

  markerArray.markers.at(1).pose.position.x = right_leg_init_.getX();
  markerArray.markers.at(1).pose.position.y = right_leg_init_.getY();
  markerArray.markers.at(1).pose.position.z = right_leg_init_.getZ() - delta_P;
  
  if(execute_once_)
  {
    markerArray.markers.at(0).scale.x = psi_x_;
    markerArray.markers.at(0).scale.y = psi_y_;
    markerArray.markers.at(0).scale.z = psi_z_;
    markerArray.markers.at(1).scale.x = psi_x_;
    markerArray.markers.at(1).scale.y = psi_y_;
    markerArray.markers.at(1).scale.z = psi_z_;
    execute_once_ = false;
  }
}

void ShadowLegs::stopLegsShadowMotion()
{
  control_motion_ = false;
}

bool ShadowLegs::isPoseReachable()
{
  tf::StampedTransform* current_leg_transform;
  current_leg_transform = (swing_leg_side == LegUpSide::LEFT) ? &left_leg_transform_ : &right_leg_transform_;
  if (current_leg_transform->getOrigin().length() * alpha_>l_r_)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void ShadowLegs::moveLeg()
{
  RobotSide side;
  geometry_msgs::PoseStamped foot_goal_pose, foot_goal_pose_world;
  foot_goal_pose.header.frame_id = rd_->getPelvisFrame();
  foot_goal_pose_world.header.frame_id = rd_->getWorldFrame();
  tf::StampedTransform * current_leg_transform;

  current_leg_transform = (swing_leg_side == LegUpSide::LEFT) ? &left_leg_transform_ : &right_leg_transform_;
  side = (swing_leg_side == LegUpSide::LEFT) ? RobotSide::LEFT : RobotSide::RIGHT;
  // foot_goal_pose.pose.orientation.w = 1.0;
  foot_goal_pose.pose.position.x = current_leg_transform->getOrigin().getX()*alpha_;
  foot_goal_pose.pose.position.y = current_leg_transform->getOrigin().getY()*alpha_;
  foot_goal_pose.pose.position.z = current_leg_transform->getOrigin().getZ()*alpha_;
  getFootOrientation(foot_goal_pose);
  robot_state_->transformPose(foot_goal_pose, foot_goal_pose_world, rd_->getWorldFrame());
  // ROS_INFO_STREAM(foot_goal_pose.pose << "\n" << foot_goal_pose_world.pose);
  leg_controller_->moveFoot(side, foot_goal_pose_world.pose, motion_time_);
  ros::Duration(0.2).sleep();
}

void ShadowLegs::getFootOrientation(geometry_msgs::PoseStamped& foot_goal_pose)
{
  float roll = 0.0, pitch = 0.0, yaw = 0.0;
  float y_offset = (swing_leg_side == LegUpSide::LEFT) ? -0.16 : 0.16;
  roll = atan2((foot_goal_pose.pose.position.y + y_offset), -foot_goal_pose.pose.position.z);
  // pitch = atan2(foot_goal_pose.pose.position.y, -foot_goal_pose.pose.position.z);
  ROS_INFO_STREAM("Roll--> " << roll << " Pitch--> " << pitch);
  ROS_INFO_STREAM("x -->" << foot_goal_pose.pose.position.x << " y--> " << foot_goal_pose.pose.position.y << " z--> "
                          << foot_goal_pose.pose.position.z);
  tf::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(quat, foot_goal_pose.pose.orientation);
}