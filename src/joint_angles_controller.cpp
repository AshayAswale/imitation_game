#include <imitation_game/joint_angles_controller.h>

JointAnglesController::JointAnglesController(ros::NodeHandle nh):nh_(nh)
{
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh_);
  rd_ = RobotDescription::getRobotDescription(nh_);
  initializeVariables();
}

JointAnglesController::~JointAnglesController()
{
}

void JointAnglesController::initializeVariables()
{
  std::vector<std::string> chest_joint_names, left_arm_joint_names, right_arm_joint_names;

  rd_->getChestJointNames(chest_joint_names);
  rd_->getLeftArmJointNames(left_arm_joint_names);
  rd_->getRightArmJointNames(right_arm_joint_names);

  chest_joint_number_ = state_informer_->getJointNumber(chest_joint_names.front());
  left_arm_joint_number_ = state_informer_->getJointNumber(left_arm_joint_names.front());
  right_arm_joint_number_ = state_informer_->getJointNumber(right_arm_joint_names.front());

  chest_size_ = chest_joint_names.size();
  left_arm_size_ = left_arm_joint_names.size();
  right_arm_size_ = right_arm_joint_names.size();

  chest_index_ = 0;
  left_arm_index_ = chest_index_ + chest_size_;
  right_arm_index_ = left_arm_index_ + left_arm_size_;

  total_joints_size_ =  chest_size_ + left_arm_size_ + right_arm_size_;
  initializeMatrices(total_joints_size_);

  setDefaultGains();

  d_t = 0.2;
  max_jt_accn = 5;
  min_jt_accn = -5;
}

void JointAnglesController::initializeMatrices(const size_t size)
{
  k_p_.resize(size);
  k_d_.resize(size);
  derivative_.resize(size);
  p_out_.resize(size);
  d_out_.resize(size);
  contr_output_.resize(size);
  curr_position_.resize(size);
  prev_position_.resize(size);
  desd_position_.resize(size);
  error_.resize(size);
  prev_error_.resize(size);

  k_p_.setZero();
  k_d_.setZero();
  derivative_.setZero();
  p_out_.setZero();
  d_out_.setZero();
  contr_output_.setZero();
  curr_position_.setZero();
  prev_position_.setZero();
  desd_position_.setZero();
  error_.setZero();
  prev_error_.setZero();
}

void JointAnglesController::setDefaultGains()
{
  for (size_t i = 0; i < total_joints_size_;i++)
  {
    k_p_.diagonal()(i) = 110;
    k_d_.diagonal()(i) = 30;
  }
}

std::vector<double> JointAnglesController::getControlledJointAngles(const std::vector<double>& joint_angles)
{
  vectorToDiagonalMatrix(joint_angles, desd_position_);
  static std::vector<double> curr_joint_angles_;
  state_informer_->getJointPositions(curr_joint_angles_);

  insertValuesInMatrix(joint_angles, chest_index_, chest_index_ + chest_size_, curr_position_);
  insertValuesInMatrix(joint_angles, left_arm_index_, left_arm_index_ + left_arm_size_, curr_position_);
  insertValuesInMatrix(joint_angles, right_arm_index_, right_arm_index_ + right_arm_size_, curr_position_);

  error_ = desd_position_ - curr_position_;
  p_out_ = k_p_ * error_;

  derivative_ = (error_ - prev_error_) / d_t;
  d_out_ = k_d_ * derivative_;
  prev_error_ = error_;

  contr_output_ = p_out_ + d_out_;

  limitAccelerations(contr_output_);
  
  return diagonalMatrixToVector(contr_output_);
}
