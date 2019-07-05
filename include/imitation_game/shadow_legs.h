#include <tough_common/robot_state.h>
#include <tough_common/robot_description.h>
#include <tough_controller_interface/pelvis_control_interface.h>
#include <tough_controller_interface/leg_control_interface.h>
#include <tf/transform_listener.h>
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
  tf::StampedTransform leg_transform_pelvis_;

  float scale_factor_ = 1.0f;
  float robot_leg_height_ = 0.90f;
  float human_height_, human_leg_length_;
  float motion_time_ = 1.0f;

  LegControlInterface* leg_controller_;
  PelvisControlInterface* pelvis_controller_;
  RobotStateInformer* robot_state_;
  RobotDescription* rd_;

  LegUpSide human_leg_up_side_, robot_leg_up_side_, current_leg_;
  RobotSide robot_side_;
  geometry_msgs::PoseStamped leg_pose_world_, leg_pose_pelvis_;
  geometry_msgs::Pose ground_pose_;
  
  bool is_robot_leg_up, is_human_leg_up;
  bool shift_robot_support_leg = false;
  bool control_motion_ = true;
  bool execute_once_ = false;

  std::string PREFIX_OPENNI = "/openni/";
  std::string left_foot_frame_ = PREFIX_OPENNI + "left_foot", right_foot_frame_ = PREFIX_OPENNI + "right_foot",
              pelvis_frame_ = PREFIX_OPENNI + "pelvis";

  std::thread thread_for_shadow_motion_;

  void setGroundPose();
  void setHeightFactor();
  void waitforTransform(float time = 2.0f);
  geometry_msgs::Pose getPoseFromVector3(const tf::Vector3& vector);
  void setPelvisHeight();

  void startMotionController();

  void update();
  void updateHumanFootTransform();
  void updateHumanLegUpStatus();
  void updateRobotLegUpStatus();

  void control();
  bool isMotionExecutable();
  void setRobotLegGoal();

  void execute();
  void executeLegMotion();
  void executeLegShift();
  void executePlaceLeg();

  void doControl();

public:
  ShadowLegs(ros::NodeHandle nh);
  ~ShadowLegs();
  void startLegsShadowMotion();
  void stopLegsShadowMotion();
  void setMotionTime(float time);
  float getMotionTime();
};
