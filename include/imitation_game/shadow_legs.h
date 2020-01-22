#include <tough_common/robot_state.h>
#include <tough_common/robot_description.h>
#include <tough_controller_interface/pelvis_control_interface.h>
#include <tough_controller_interface/leg_control_interface.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

struct LegMovement
{
  RobotSide side;
  tf::Transform leg_pose;
};

class ShadowLegs
{
private:
  // enum LegUpSide
  // {
  //   LEFT = 0,
  //   RIGHT,
  //   NONE
  // };

  ros::NodeHandle nh_;
  tf::TransformListener trans_listener_wrt_l_leg_;
  tf::Transform r_leg_wrt_l_leg_, pelv_wrt_l_leg_;
  tf::Vector3 left_leg_init_, right_leg_init_, pelvis_init_;

  LegMovement last_leg_moved_, swing_leg_;

  float alpha_ = 1.0f;
  float l_r_ = 0.85f; // ATLAS
  // float l_r = 1.0; // Valkyrie
  float l_h_;
  float motion_time_ = 1.0f;
  float movement_threshold_ = 0.05;
  float robot_pelvis_init_height_;
  float pelvis_min_height = 0.7;
  float pelvis_motion_time_ = 0.3;
  float delta_P;
  float gbr_height_ = 0.05;

  LegControlInterface *leg_controller_;
  PelvisControlInterface *pelvis_controller_;
  RobotStateInformer *robot_state_;
  RobotDescription *rd_;

  // LegUpSide swing_leg_side, robot_leg_up_side_, current_leg_;
  RobotSide robot_side_;
  geometry_msgs::PoseStamped leg_pose_world_, leg_pose_pelvis_;
  geometry_msgs::Pose ground_pose_;

  bool is_robot_leg_up, is_human_leg_up;
  bool shift_robot_support_leg = false;
  bool control_motion_ = true;
  bool execute_once_ = true;
  bool op_in_double_support_;

  std::string PREFIX_OPENNI = "/openni/";
  std::string op_left_foot_frame_ = PREFIX_OPENNI + "left_foot", op_right_foot_frame_ = PREFIX_OPENNI + "right_foot",
              op_pelvis_frame_ = PREFIX_OPENNI + "pelvis", op_openni_base_frame_ = "openni_depth_frame";

  // std::string robot_pelvis_frame_ = "pelvis", robot_world_frame_ = "world";

  std::thread thread_for_shadow_motion_;

  tf::Transform getTransform(const std::string &track_frame);

  inline void setAlphaValue()
  {
    tf::Transform latest_transform = getTransform(op_pelvis_frame_);
    float sigma = 0.2;
    l_h_ = latest_transform.getOrigin().length();
    alpha_ = l_r_ / (l_h_ + sigma);
    //     alpha_ = 0.5;
    ROS_INFO("Alpha --> %f", alpha_);
  }

  inline bool isRobotInDoubleSupport()
  {
    return robot_state_->isRobotInDoubleSupport();
  }

  inline geometry_msgs::Pose tfTransformToMsg(const tf::Transform &trans)
  {
    geometry_msgs::Pose msg;
    tf::quaternionTFToMsg(trans.getRotation(), msg.orientation);
    msg.position.x = trans.getOrigin().getX();
    msg.position.y = trans.getOrigin().getY();
    msg.position.z = trans.getOrigin().getZ();
    return msg;
  }

  void updateTransforms();
  void setCalibValues();
  void startMotionController();
  void setLeftLegSwingLeg();
  bool isOperatorInDoubleSupport();
  bool isOperatorMoving();
  bool isPoseReachable();

  void placeLegDown();
  void setPelvisHeight();
  void moveLeg();
  void movePelvis();
  void getFootOrientation(geometry_msgs::PoseStamped &foot_goal_pose);
  void setPelvisInitialHeight();
  void executePelvisHeight(float height);
  void registerLegPlacementPose();

public:
  ShadowLegs(ros::NodeHandle nh);
  ~ShadowLegs();
  void startLegsShadowMotion();
  void stopLegsShadowMotion();
  void setMotionTime(float time);
  float getMotionTime();
  void getGBR(visualization_msgs::MarkerArray &markerArray);
  void getMarkerVectors(visualization_msgs::MarkerArray &markerArray);
};
