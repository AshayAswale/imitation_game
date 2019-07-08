#include <tough_common/robot_state.h>
#include <tough_common/robot_description.h>
#include <tough_controller_interface/pelvis_control_interface.h>
#include <tough_controller_interface/leg_control_interface.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

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
  tf::StampedTransform left_leg_transform_, right_leg_transform_, pelvis_transform_;
  tf::Vector3 left_leg_init_, right_leg_init_, pelvis_init_;

  float alpha_ = 1.0f;
  float l_r_ = 0.85f; // ATLAS
  // float l_r = 1.0; // Valkyrie
  float l_h_;
  float motion_time_ = 1.0f;
  float pelvis_threshold_ = 0.1;
  float robot_pelvis_init_height_ = 0.85; //Atlas
  // float robot_pelvis_init_height_ = 1.0; //Valkyrie

  float psi_x_ = 0.3, psi_y_ = 0.5, psi_z_ = 0.2;

  LegControlInterface* leg_controller_;
  PelvisControlInterface* pelvis_controller_;
  RobotStateInformer* robot_state_;
  RobotDescription* rd_;

  LegUpSide swing_leg_side, robot_leg_up_side_, current_leg_;
  RobotSide robot_side_;
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

  // std::string robot_pelvis_frame_ = "pelvis", robot_world_frame_ = "world";

  std::thread thread_for_shadow_motion_;

  void setCalibValues();


  inline bool isRobotInDoubleSupport()
  {
    return robot_state_->isRobotInDoubleSupport();
  }

  void startMotionController();
  bool isOperatorInDoubleSupport();
  bool isLegInGbr(std::string leg_frame);
  void updateLegsTransform();

  bool isPoseReachable();

  tf::StampedTransform getTransform(const std::string& foot_frame, const std::string& ref_frame);

  inline void setAlphaValue()
  {
    tf::StampedTransform latest_transform = getTransform(right_foot_frame_, pelvis_frame_);
    float sigma = 0.2;
    l_h_ = latest_transform.getOrigin().length();
    alpha_ = l_r_ / (l_h_ + sigma);
//     alpha_ = 0.5;
    ROS_INFO("Alpha --> %f", alpha_);
  }

  void placeLegDown();
  void setPelvisHeight();
  void moveLeg();

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
  void getGBR(visualization_msgs::MarkerArray& markerArray);
};
