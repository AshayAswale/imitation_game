#include <imitation_game/shadow_legs.h>
#include <math.h>

ShadowLegs::ShadowLegs(ros::NodeHandle nh) : nh_(nh)
{
  leg_controller_ = new LegControlInterface(nh_);
  pelvis_controller_ = new PelvisControlInterface(nh);
  robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
  ros::Duration(0.01).sleep();
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
  // robot_pelvis_transform = getTransform(rd_->getPelvisFrame());
  // robot_pelvis_init_height_ = 0.79;
  robot_state_->getTransform(rd_->getPelvisFrame(), robot_pelvis_transform);
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

tf::Transform ShadowLegs::getTransform(const std::string &track_frame)
{
  static tf::StampedTransform latest_transform;
  try
  {
    latest_transform.setIdentity();
    trans_listener_wrt_l_leg_.waitForTransform(op_left_foot_frame_, track_frame, ros::Time(0.0), ros::Duration(0.5));
    trans_listener_wrt_l_leg_.lookupTransform(op_left_foot_frame_, track_frame, ros::Time(0.0), latest_transform);
    return latest_transform;
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    ros::Duration(0.01).sleep();
    getTransform(track_frame);
  }
}

void ShadowLegs::setCalibValues()
{
  updateTransforms();

  setAlphaValue();
}

void ShadowLegs::startMotionController()
{
  ros::Rate rate(100.0);
  setLeftLegSwingLeg();
  while (control_motion_)
  {
    updateTransforms();
    if (isOperatorInDoubleSupport() && !isRobotInDoubleSupport())
    {
      // if (!isRobotInDoubleSupport() && isOperatorMoving())
      {
        ROS_INFO("Placing Leg Down.");
        placeLegDown();
      }
      // movePelvis();
    }
    else
    {
      // if (isRobotInDoubleSupport())
      // {
      //   ROS_INFO("Lifting Leg");
      //   liftLeg();
      // }
      ROS_INFO("Operator in Swing Phase");

      // if (isPoseReachable())
      // {
      ROS_INFO("Executing Motion");
      moveLeg();
      // movePelvis();
      // }
      // else
      // {
      //   ROS_WARN("Out of Reach!");
      // }
    }
    // rate.sleep();
  }
}

void ShadowLegs::setLeftLegSwingLeg()
{
  swing_leg_.side = RobotSide::LEFT;
  swing_leg_.leg_pose = pelv_wrt_l_leg_;
}

bool ShadowLegs::isOperatorInDoubleSupport()
{
  float ht_r_leg_wrt_l_leg = r_leg_wrt_l_leg_.getOrigin().getZ();
  if (std::abs(ht_r_leg_wrt_l_leg) < gbr_height_)
  {
    return true; // Operator in double support
  }

  swing_leg_.side = (ht_r_leg_wrt_l_leg > 0.0) ? RobotSide::RIGHT : RobotSide::LEFT;

  if (swing_leg_.side == RobotSide::LEFT)
    swing_leg_.leg_pose = r_leg_wrt_l_leg_.inverse();
  else
    swing_leg_.leg_pose = r_leg_wrt_l_leg_;

  return false; // Operator in swing phase
}

bool ShadowLegs::isOperatorMoving()
{
  static tf::Vector3 leg_transform;
  leg_transform = last_leg_moved_.leg_pose.getOrigin() - swing_leg_.leg_pose.getOrigin();

  if (leg_transform.length() > movement_threshold_)
    return true;
  else
    return false;
}

void ShadowLegs::updateTransforms()
{
  r_leg_wrt_l_leg_ = getTransform(op_right_foot_frame_);
  pelv_wrt_l_leg_ = getTransform(op_pelvis_frame_);
}

/**
   * @brief NO FUNCTION IN TOUGH TO HAVE PELVIS TRAJECTORY.
   * @todo implement when TOUGH facility introduced
   * 
   * 
   */
void ShadowLegs::movePelvis()
{
  static tf::Transform pelv_wrt_anchor_leg;
  pelv_wrt_l_leg_.setIdentity();
  if (swing_leg_.side == RobotSide::RIGHT)
    pelv_wrt_anchor_leg = pelv_wrt_l_leg_;
  else
    pelv_wrt_anchor_leg = r_leg_wrt_l_leg_.inverse() * pelv_wrt_l_leg_;

  // Should also have move pelvis along with the height.
  executePelvisHeight(pelv_wrt_anchor_leg.getOrigin().getZ() * alpha_);
}

void ShadowLegs::executePelvisHeight(float height)
{
  ROS_INFO("Setting Pelvis Height to %f", height);
  pelvis_controller_->controlPelvisHeight(height, pelvis_motion_time_);
  ros::Duration(0.1).sleep();
}

void ShadowLegs::placeLegDown()
{
  geometry_msgs::Pose leg_place = tfTransformToMsg(swing_leg_.leg_pose);
  leg_place.position.z += gbr_height_;
  leg_controller_->moveFoot(swing_leg_.side, leg_place, motion_time_);
  ros::Duration(motion_time_).sleep();

  leg_controller_->placeLeg(swing_leg_.side, gbr_height_ * 2, 0.5);
  ros::Duration(0.5).sleep();
}

// void ShadowLegs::getGBR(visualization_msgs::MarkerArray& markerArray)
// {
//   markerArray.markers.at(0).pose.position.x = left_leg_init_.getX();
//   markerArray.markers.at(0).pose.position.y = left_leg_init_.getY();
//   markerArray.markers.at(0).pose.position.z = left_leg_init_.getZ() - delta_P;

//   markerArray.markers.at(1).pose.position.x = right_leg_init_.getX();
//   markerArray.markers.at(1).pose.position.y = right_leg_init_.getY();
//   markerArray.markers.at(1).pose.position.z = right_leg_init_.getZ() - delta_P;

//   if (execute_once_)
//   {
//     markerArray.markers.at(0).scale.x = psi_x_;
//     markerArray.markers.at(0).scale.y = psi_y_;
//     markerArray.markers.at(0).scale.z = psi_z_;
//     markerArray.markers.at(1).scale.x = psi_x_;
//     markerArray.markers.at(1).scale.y = psi_y_;
//     markerArray.markers.at(1).scale.z = psi_z_;
//     execute_once_ = false;
//   }
// }

void ShadowLegs::getMarkerVectors(visualization_msgs::MarkerArray &markerArray)
{
  markerArray.markers.at(0).header.frame_id = "/openni/right_foot";
  markerArray.markers.at(0).points.at(1).x = r_leg_wrt_l_leg_.inverse().getOrigin().getX();
  markerArray.markers.at(0).points.at(1).y = r_leg_wrt_l_leg_.inverse().getOrigin().getY();
  markerArray.markers.at(0).points.at(1).z = r_leg_wrt_l_leg_.inverse().getOrigin().getZ();

  markerArray.markers.at(1).points.at(1).x = pelv_wrt_l_leg_.getOrigin().getX();
  markerArray.markers.at(1).points.at(1).y = pelv_wrt_l_leg_.getOrigin().getY();
  markerArray.markers.at(1).points.at(1).z = pelv_wrt_l_leg_.getOrigin().getZ();

  static tf::Transform temp_r_pel;
  temp_r_pel.setIdentity();
  temp_r_pel = r_leg_wrt_l_leg_.inverse() * pelv_wrt_l_leg_;

  markerArray.markers.at(2).header.frame_id = "/openni/right_foot";
  markerArray.markers.at(2).points.at(1).x = temp_r_pel.getOrigin().getX();
  markerArray.markers.at(2).points.at(1).y = temp_r_pel.getOrigin().getY();
  markerArray.markers.at(2).points.at(1).z = temp_r_pel.getOrigin().getZ();
}

void ShadowLegs::stopLegsShadowMotion()
{
  control_motion_ = false;
}

bool ShadowLegs::isPoseReachable()
{
  bool reachable = (swing_leg_.leg_pose.getOrigin().length() > 1.0) ? true : false;
  return reachable;
}

void ShadowLegs::moveLeg()
{
  last_leg_moved_ = swing_leg_;
  RobotSide side;
  geometry_msgs::PoseStamped foot_goal_pose, foot_goal_pose_world;
  foot_goal_pose.header.frame_id = (swing_leg_.side == RobotSide::RIGHT) ? rd_->getLeftFootFrameName() : rd_->getRightFootFrameName();
  foot_goal_pose_world.header.frame_id = rd_->getWorldFrame();

  foot_goal_pose.pose.position.x = swing_leg_.leg_pose.getOrigin().getX() * alpha_;
  foot_goal_pose.pose.position.y = swing_leg_.leg_pose.getOrigin().getY() * alpha_;
  foot_goal_pose.pose.position.z = swing_leg_.leg_pose.getOrigin().getZ() * alpha_;
  foot_goal_pose.pose.orientation.w = 1;
  // getFootOrientation(foot_goal_pose);
  robot_state_->transformPose(foot_goal_pose, foot_goal_pose_world, rd_->getWorldFrame());

  side = swing_leg_.side;
  ROS_INFO_STREAM("Leg Side: " << swing_leg_.side);
  ROS_INFO_STREAM(foot_goal_pose.pose << "\n"
                                      << foot_goal_pose_world.pose);
  leg_controller_->moveFoot(side, foot_goal_pose_world.pose, motion_time_);
  ros::Duration(0.2).sleep();
}

void ShadowLegs::getFootOrientation(geometry_msgs::PoseStamped &foot_goal_pose)
{
  float roll = 0.0, pitch = 0.0, yaw = 0.0;
  geometry_msgs::PoseStamped temp_foot_goal_pose;
  temp_foot_goal_pose.header.frame_id = rd_->getPelvisFrame();
  robot_state_->transformPose(foot_goal_pose, temp_foot_goal_pose, rd_->getPelvisFrame());
  float y_offset = (swing_leg_.side == RobotSide::LEFT) ? -0.16 : 0.16;
  roll = atan2((temp_foot_goal_pose.pose.position.y + y_offset), -temp_foot_goal_pose.pose.position.z);
  // pitch = atan2(temp_foot_goal_pose.pose.position.y, -temp_foot_goal_pose.pose.position.z);
  ROS_INFO_STREAM("Roll--> " << roll << " Pitch--> " << pitch);
  ROS_INFO_STREAM("x -->" << temp_foot_goal_pose.pose.position.x << " y--> " << temp_foot_goal_pose.pose.position.y << " z--> "
                          << temp_foot_goal_pose.pose.position.z);
  tf::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(quat, foot_goal_pose.pose.orientation);
}
