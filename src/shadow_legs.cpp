#include <imitation_game/shadow_legs.h>

ShadowLegs::ShadowLegs(ros::NodeHandle nh) : nh_(nh)
{
  leg_controller_ = new LegControlInterface(nh_);
  pelvis_controller_ = new PelvisControlInterface(nh);
  robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
  rd_ = RobotDescription::getRobotDescription(nh_);
  leg_pose_pelvis_.header.frame_id = rd_->getPelvisFrame();
}

ShadowLegs::~ShadowLegs()
{
  delete leg_controller_;
  delete pelvis_controller_;
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
        // placeLegDown();
      }
    }
    else
    {
      // Identify Swing Leg
      ROS_INFO("Operator in Swing Phase");
      // if (isPoseReachable())
      // {
      //   ROS_INFO("Executing Motion");
      // }
      // else
      // {
      //   ROS_WARN("Out of Reach!");
      // }
    }         
    rate.sleep();
  }
}

bool ShadowLegs::isOperatorInDoubleSupport()
{
  static bool left_leg_in_gbr, right_leg_in_gbr;
  left_leg_in_gbr = isLegInGbr(left_foot_frame_);
  right_leg_in_gbr = isLegInGbr(right_foot_frame_);

  if(left_leg_in_gbr && right_leg_in_gbr)
  {
    human_leg_up_side_ = LegUpSide::NONE;
    return true;
  }
  else if (!left_leg_in_gbr && !right_leg_in_gbr)
  {
    return false;
  }
  else
  {
    human_leg_up_side_ = (left_leg_in_gbr) ? LegUpSide::LEFT : LegUpSide::RIGHT;
    return false;
  }
}

bool ShadowLegs::isLegInGbr(std::string leg_frame)
{
  static tf::StampedTransform* current_leg_transform;
  static tf::Vector3* current_leg_init_vector;

  static float delta_x, delta_y, delta_z;
  static float delta_P;
  delta_P = pelvis_transform_.getOrigin().getZ() - pelvis_init_.getZ();
  
  current_leg_transform = (leg_frame.compare(left_foot_frame_) == 0) ? &left_leg_transform_ : &right_leg_transform_;
  current_leg_init_vector = (leg_frame.compare(left_foot_frame_) == 0) ? &left_leg_init_ : &right_leg_init_;

  delta_x = std::abs(current_leg_transform->getOrigin().getX() - current_leg_init_vector->getX());
  delta_y = std::abs(current_leg_transform->getOrigin().getY() - current_leg_init_vector->getY());
  delta_z = std::abs((current_leg_transform->getOrigin().getZ() + delta_P) - current_leg_init_vector->getZ());

  if (delta_x<psi_x_/2&&delta_y<psi_y_/2&&delta_z<psi_z_/2)
  {
    return true;
    // ROS_WARN("%s Inside ")
  }
  else
    return false;
}

bool ShadowLegs::updateLegsTransform()
{
  left_leg_transform_ = getTransform(left_foot_frame_, pelvis_frame_);
  right_leg_transform_ = getTransform(right_foot_frame_, pelvis_frame_);
  pelvis_transform_ = getTransform(pelvis_frame_, openni_base_frame_);
}

void ShadowLegs::setPelvisHeight()
{
  static tf::StampedTransform robot_pelvis_transform;
  static float delta_P, current_delta_P, current_height;
  delta_P = pelvis_transform_.getOrigin().getZ() - pelvis_init_.getZ();
  robot_pelvis_transform = getTransform(robot_pelvis_frame_, robot_world_frame_);
  current_height = robot_pelvis_transform.getOrigin().getZ();
  current_delta_P = current_height - robot_pelvis_init_height_;
  if ((delta_P*alpha_ - current_delta_P) < pelvis_threshold_)
  {
    ROS_INFO("Setting Pelvis Height to %f", robot_pelvis_init_height_ + delta_P);
    pelvis_controller_->controlPelvisHeight(robot_pelvis_init_height_ + delta_P, motion_time_);
    ros::Duration(motion_time_).sleep();
  }
}

// void ShadowLegs::placeLegDown()
// {

// }

void ShadowLegs::getGBR(visualization_msgs::MarkerArray& markerArray)
{
  static float delta_P;
  delta_P = pelvis_transform_.getOrigin().getZ() - pelvis_init_.getZ();

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