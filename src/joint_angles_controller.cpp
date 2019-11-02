#include <imitation_game/joint_angles_controller.h>

JointAnglesController::JointAnglesController(ros::NodeHandle nh) : nh_(nh)
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

  total_joints_size_ = chest_size_ + left_arm_size_ + right_arm_size_;
  initializeMatrices(total_joints_size_);

  setDefaultGains();

  d_t = 0.02;
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
  for (size_t i = 0; i < total_joints_size_; i++)
  {
    k_p_.diagonal()(i) = 110;
    k_d_.diagonal()(i) = 30;
  }
}

std::vector<double> JointAnglesController::getControlledJointAngles(const std::vector<double>& joint_angles)
{
  vectorToDiagonalMatrix(joint_angles, desd_position_);
  curr_joint_angles_.resize(0);
  while (curr_joint_angles_.size() == 0)
    state_informer_->getJointPositions(curr_joint_angles_);

  insertValuesInMatrix(curr_joint_angles_, chest_joint_number_, chest_joint_number_ + chest_size_, chest_index_, curr_position_);
  insertValuesInMatrix(curr_joint_angles_, left_arm_joint_number_, left_arm_joint_number_ + left_arm_size_, left_arm_index_,
                       curr_position_);
  insertValuesInMatrix(curr_joint_angles_, right_arm_joint_number_, right_arm_joint_number_ + right_arm_size_, right_arm_index_,
                       curr_position_);

  updateControlOutput();

  return diagonalMatrixToVector(contr_output_);
}

void JointAnglesController::updateControlOutput()
{
  error_ = desd_position_ - curr_position_;
  // printMatrix(desd_position_, "desd_position_");
  // printMatrix(curr_position_, "curr_position_");
  // printMatrix(error_, "error_");
  
  p_out_ = k_p_ * error_;
  // printMatrix(k_p_, "k_p_");
  // printMatrix(p_out_, "p_out_");

  derivative_ = (error_ - prev_error_) * (1/d_t);
  // printMatrix(prev_error_, "prev_error_");
  // printMatrix(derivative_, "derivative_");

  d_out_ = k_d_ * derivative_;
  // printMatrix(k_d_, "k_d_");
  // printMatrix(d_out_, "d_out_");
  prev_error_ = error_;

  contr_output_ = p_out_ + d_out_;
  // printMatrix(contr_output_, "contr_output_");

  limitAccelerations(contr_output_);
}

void JointAnglesController::limitAccelerations(Eigen::DiagonalMatrix<double, Eigen::Dynamic>& matrix)
{
  for (int i = 0; i < matrix.diagonal().size(); i++)
  {
    if (matrix.diagonal()(i) > max_jt_accn)
    {
      matrix.diagonal()(i) = max_jt_accn;
    }
    else if (matrix.diagonal()(i) < min_jt_accn)
      matrix.diagonal()(i) = min_jt_accn;
  }
}

std::vector<double>
JointAnglesController::diagonalMatrixToVector(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& matrix)
{
  std::vector<double> vector;
  int size = matrix.diagonal().size();
  vector.resize(size);

  for (int i = 0; i < size; i++)
  {
    vector.at(i) = matrix.diagonal()(i);
  }
  return vector;
}

void JointAnglesController::printMatrix(Eigen::DiagonalMatrix<double, Eigen::Dynamic>& matrix, const std::string& matrix_name)
{
  std::cout << "####  "<<matrix_name << "  ####  "<< std::endl;
  for (int j = 0; j < matrix.diagonal().size();j++)
  {
    for (int i = 0; i < matrix.diagonal().size();i++)
      std::cout << matrix.toDenseMatrix()(j,i)<<"  ";
    std::cout << "" << std::endl;
  }
}