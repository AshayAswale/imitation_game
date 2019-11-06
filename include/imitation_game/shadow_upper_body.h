#include <tough_common/robot_description.h>
#include <tough_controller_interface/wholebody_control_interface.h>

#include <imitation_game/joint_angles_controller.h>

class ShadowUpperBody
{
private:
  ros::NodeHandle nh_;
  RobotStateInformer* robot_state_;
  RobotDescription* rd_;
  WholebodyControlInterface* wholebodyController_;
  JointAnglesController* joint_controller_;

  std::map<std::string, int> jointIndexMap();
  trajectory_msgs::JointTrajectory joint_trajectory_;

  double time_execution = 0.02;
  bool run_code = true;

  std::string OPNNI_PREFIX_ = "openni/";
  std::string left_shoulder_frame_ = OPNNI_PREFIX_ + "left_shoulder",
              right_shoulder_frame_ = OPNNI_PREFIX_ + "right_shoulder", right_elbow_frame_ = OPNNI_PREFIX_ + "right_elbow",
              left_elbow_frame_ = OPNNI_PREFIX_ + "left_elbow", right_hand_frame_ = OPNNI_PREFIX_ + "right_hand",
              left_hand_frame_ = OPNNI_PREFIX_ + "left_hand", operator_pelvis_frame_ = OPNNI_PREFIX_ + "pelvis",
              neck_frame_ = OPNNI_PREFIX_ + "neck";

  std::string yaw_ = "_yaw", roll_ = "_roll", pitch_ = "_pitch";

  std::map<std::string, std::string> human_robot_joint_map_ = {
    { left_shoulder_frame_+ yaw_, "l_arm_shz" },    { left_shoulder_frame_+ roll_, "l_arm_shx" },
    { right_shoulder_frame_+ yaw_, "r_arm_shz" },   { right_shoulder_frame_+ roll_, "r_arm_shx" },
    { left_elbow_frame_+ pitch_, "l_arm_ely" },     { left_elbow_frame_+ roll_, "l_arm_elx" },
    { right_elbow_frame_+ pitch_, "r_arm_ely" },    { right_elbow_frame_+ roll_, "r_arm_elx" },
    { operator_pelvis_frame_ + roll_, "back_bkx"},             { operator_pelvis_frame_ + pitch_, "back_bkz"},
    { operator_pelvis_frame_ + yaw_, "back_bky"}
  };

  std::map<std::string, std::string>::iterator string_map_iterator_;
  std::map<std::string, std::string> child_parent_frames_ = { { right_elbow_frame_, right_shoulder_frame_ },
                                                              { left_elbow_frame_, left_shoulder_frame_ },
                                                              { right_hand_frame_, right_elbow_frame_ },
                                                              { left_hand_frame_, left_elbow_frame_ },
                                                              { neck_frame_, operator_pelvis_frame_} };

  std::vector<std::string> rest_joints = { "back_bkz",  "l_arm_wry", "l_arm_wrx", "l_arm_wry2",
                                           "r_arm_wry", "r_arm_wrx", "r_arm_wry2" };

  std::map<std::string, std::pair<double, double>> joint_limits_map;
  std::map<std::string, std::pair<double, double>>::iterator joint_limits_iterator_;

  // DO NOT CHANGE THE SEQUENCE
  std::vector<std::string> left_arm_names_, right_arm_names_, chest_names_;

  std::vector<std::string> chest_frame_vector = {neck_frame_};

  std::vector<std::string> left_frames_vector_ = {
    left_elbow_frame_,
    left_hand_frame_
  };

  std::vector<std::string> right_frames_vector_ = {
    right_elbow_frame_,
    right_hand_frame_
  };

  std::vector<std::vector<std::string>> frames_vector_ = {
    left_frames_vector_,
    right_frames_vector_};

  tf::TransformListener tf_listener_;
  tf::StampedTransform tf_transform_;

  std::vector<double> input_joint_angles_, control_joint_angles_;
  
  void initialize();
  // void initializeJointTrajectory();
  // void initializeJointIndexMap();

  void update();
  void updateTranforms();
  void updateJointTrajectoryMsg(const std::string& frame_name, double roll, double yaw);
  void updateJointTrajectoryMsg(const std::string& frame_name, double roll, double pitch, double yaw);
  void clearJointTrajectory();
  void getRollPitchYaw(const tf::StampedTransform& transform, double& roll, double& pitch, double& yaw);
  tf::StampedTransform getTransform(const std::string& foot_frame, const std::string& ref_frame);
  void resizeJointTrajectory();
  void updateJointLimits();
  void insertJointLImits(const std::vector<std::string>& joint_names_vector, const std::vector <std::pair<double, double>>& joint_limits_temp);
  void addToJointTrajectory(const std::string& frame_name, const std::string& rotation_frame, double rotation);
  void insertRestJoints();
  // bool updateJointAngle(std::string frame_name, );

  void control();

  void execute();

public:
  ShadowUpperBody(ros::NodeHandle nh);
  ~ShadowUpperBody();

  void startShadowMotion();
  void stopShadowMotion();
};
