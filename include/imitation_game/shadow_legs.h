#include <tough_common/robot_state.h>
#include <tough_common/robot_description.h>
#include <tough_controller_interface/pelvis_control_interface.h>
#include <tough_controller_interface/leg_control_interface.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

struct LegPlacement
{
  RobotSide side;
  geometry_msgs::PoseStamped leg_pose;
};

class ShadowLegs
{
private:
  enum LegUpSide
  {
    LEFT = 0,
    RIGHT,
    NONE
  };

  ros::NodeHandle nh_;
  tf::TransformListener leg_listener_pelvis_;
  // tf::StampedTransform left_leg_transform_, right_leg_transform_, pelvis_transform_;
  tf::StampedTransform swing_leg_transform_, pelvis_transform_;
  tf::Vector3 swing_leg_init_, pelvis_init_;
  LegPlacement leg_place_pose_;

  float alpha_ = 1.0f;
  float l_r_ = 0.85f; // ATLAS
  // float l_r = 1.0; // Valkyrie
  float l_h_;
  float motion_time_ = 1.0f;
  float pelvis_threshold_ = 0.1;
  float robot_pelvis_init_height_;
  float pelvis_min_height = 0.7;
  float pelvis_motion_time_ = 0.3;
  float delta_P;
  bool leg_place_posn_register_flag = true;
  bool leg_in_motion = false;

  float psi_x_ = 0.4, psi_y_ = 0.5, psi_z_ = 0.2;

  LegControlInterface *leg_controller_;
  PelvisControlInterface *pelvis_controller_;
  RobotStateInformer *robot_state_;
  RobotDescription *rd_;

  RobotSide robot_side_, anchor_leg_, swing_leg_;
  geometry_msgs::PoseStamped leg_pose_world_, leg_pose_pelvis_;
  geometry_msgs::Pose ground_pose_;

  bool is_robot_leg_up, is_human_leg_up;
  bool shift_robot_support_leg = false;
  bool control_motion_ = true;
  bool execute_once_ = true;
  bool human_in_double_support_;

  std::string PREFIX_OPENNI = "/openni/";
  std::string left_foot_frame_ = PREFIX_OPENNI + "left_foot", right_foot_frame_ = PREFIX_OPENNI + "right_foot",
              pelvis_frame_ = PREFIX_OPENNI + "pelvis", openni_base_frame_ = "openni_depth_frame";

  std::string swing_leg_frame_, anchor_leg_frame_;

  // std::string robot_pelvis_frame_ = "pelvis", robot_world_frame_ = "world";

  std::thread thread_for_shadow_motion_;

  void setCalibValues();

  inline void setAnchorLeg(RobotSide robot_side)
  {
    anchor_leg_ = robot_side;
    RobotSide opposite_side = robot_side == RobotSide::LEFT ? RobotSide::RIGHT : RobotSide::LEFT;
    swing_leg_ = opposite_side;
    swing_leg_frame_ = opposite_side == RobotSide::LEFT ? left_foot_frame_ : right_foot_frame_;
    anchor_leg_frame_ = robot_side == RobotSide::LEFT ? left_foot_frame_ : right_foot_frame_;
  }

  inline bool isRobotInDoubleSupport()
  {
    return robot_state_->isRobotInDoubleSupport();
  }

  void startMotionController();
  bool isOperatorInDoubleSupport();
  bool isLegInGbr();
  void updateLegsTransform();

  bool isPoseReachable();

  tf::StampedTransform getTransform(const std::string &foot_frame, const std::string &ref_frame);

  inline void setAlphaValue()
  {
    tf::StampedTransform latest_transform = getTransform(right_foot_frame_, pelvis_frame_);
    float sigma = 0.2;
    l_h_ = latest_transform.getOrigin().length();
    // alpha_ = l_r_ / (l_h_ + sigma);
    //     alpha_ = 0.5;
    alpha_ = 1;
    ROS_INFO("Alpha --> %f", alpha_);
  }

  void placeLegDown();
  void setPelvisHeight();
  void moveLeg(const bool is_for_placing = false);
  void getFootOrientation(geometry_msgs::PoseStamped &foot_goal_pose);
  void setPelvisInitialHeight();
  void executePelvisHeight(float height);
  void registerLegPlacementPose();

  // void stopLegsShadowMotion();

  // /**
  //  * @brief Old Code
  //  *
  //  */
  // void setGroundPose();
  // void setHeightFactor();
  // void waitforTransform(float time = 2.0f);
  // geometry_msgs::Pose getPoseFromVector3(const tf::Vector3& vector);
  // void update();
  // void updateHumanFootTransform(std::string foot_frame, , std::string ref_frame = pelvis_frame_);
  // void updateHumanLegUpStatus();
  // void updateRobotLegUpStatus();
  // void update()

  // void control();
  // bool isMotionExecutable();
  // void setRobotLegGoal();

  // void execute();
  // void executeLegMotion();
  // void executeLegShift();
  // void executePlaceLeg();

  // void doControl();

public:
  ShadowLegs(ros::NodeHandle nh);
  ~ShadowLegs();
  void startLegsShadowMotion();
  void stopLegsShadowMotion();
  void setMotionTime(float time);
  float getMotionTime();
  void getGBR(visualization_msgs::MarkerArray &markerArray);
};
