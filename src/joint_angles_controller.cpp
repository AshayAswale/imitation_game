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

  joint_names_.resize(0);
  joint_names_.insert(joint_names_.begin() + chest_index_, chest_joint_names.begin(), chest_joint_names.end());
  joint_names_.insert(joint_names_.begin() + left_arm_index_, left_arm_joint_names.begin(), left_arm_joint_names.end());
  joint_names_.insert(joint_names_.begin() + right_arm_index_, right_arm_joint_names.begin(),
                      right_arm_joint_names.end());

  total_joints_size_ = chest_size_ + left_arm_size_ + right_arm_size_;
  initializeMatrices(total_joints_size_);

  setDefaultGains();

  std::vector<std::pair<double, double>> chest_joint_limits, left_arm_joint_limits, right_arm_joint_limits;
  rd_->getChestJointLimits(chest_joint_limits);
  rd_->getLeftArmJointLimits(left_arm_joint_limits);
  rd_->getRightArmJointLimits(right_arm_joint_limits);

  joint_limits_.resize(0);
  joint_limits_.insert(joint_limits_.begin() + chest_index_, chest_joint_limits.begin(), chest_joint_limits.end());
  joint_limits_.insert(joint_limits_.begin() + left_arm_index_, left_arm_joint_limits.begin(),
                       left_arm_joint_limits.end());
  joint_limits_.insert(joint_limits_.begin() + right_arm_index_, right_arm_joint_limits.begin(),
                       right_arm_joint_limits.end());

  d_t = 0.02;
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
  //@todo some better method to define kp kd tables
  /*
      Kp Kd sequence in terms of joint names:
          back_bkz
          back_bky
          back_bkx
          l_arm_shz
          l_arm_shx
          l_arm_ely
          l_arm_elx
          l_arm_wry
          l_arm_wrx
          l_arm_wry2
          r_arm_shz
          r_arm_shx
          r_arm_ely
          r_arm_elx
          r_arm_wry
          r_arm_wrx
          r_arm_wry2

  */
  k_p_.diagonal() << 3, 3, 3, 3, 3, 28, 2, 3, 3, 3, 3, 3, 28, 2, 3, 3, 3;
  k_d_.diagonal() << 3, 3, 3, 3.3, 11, 1.9, 4.5, 0.1, 0.1, 0.1, 3, 2.9, 11, 1.9, 0.1, 0.1, 0.1;

  max_jt_accn = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5 };
  min_jt_accn = { -5, -5, -5, -5, -5, -5, -5, -20, -20, -20, -5, -5, -5, -5, -20, -20, -20 };
}

std::vector<double> JointAnglesController::getControlledJointAngles(const std::vector<double>& joint_angles)
{
  stdToEigenVector(joint_angles, desd_position_);

  updateDesdPosForJointLimits();
  updateCurrJointAngles();
  updateControlOutput();

  return EigenToStdVector(contr_output_);
}

void JointAnglesController::updateCurrJointAngles()
{
  curr_joint_angles_.resize(0);
  state_informer_->getJointPositions(curr_joint_angles_);

  insertValuesInMatrix(curr_joint_angles_, chest_joint_number_, chest_joint_number_ + chest_size_, chest_index_,
                       curr_position_);
  insertValuesInMatrix(curr_joint_angles_, left_arm_joint_number_, left_arm_joint_number_ + left_arm_size_,
                       left_arm_index_, curr_position_);
  insertValuesInMatrix(curr_joint_angles_, right_arm_joint_number_, right_arm_joint_number_ + right_arm_size_,
                       right_arm_index_, curr_position_);
}

void JointAnglesController::updateControlOutput()
{
  error_ = desd_position_ - curr_position_;
  p_out_ = k_p_ * error_;

  derivative_ = (error_ - prev_error_) * (1 / d_t);
  d_out_ = k_d_ * derivative_;
  prev_error_ = error_;

  contr_output_ = p_out_ + d_out_;

  limitAccelerations(contr_output_);
}

void JointAnglesController::limitAccelerations(Eigen::VectorXd& acc_vec)
{
  for (int i = 0; i < acc_vec.size(); i++)
  {
    acc_vec(i) = acc_vec(i) > max_jt_accn.at(i) ? max_jt_accn.at(i) : acc_vec(i);
    acc_vec(i) = acc_vec(i) < min_jt_accn.at(i) ? min_jt_accn.at(i) : acc_vec(i);
  }
}

std::vector<double> JointAnglesController::EigenToStdVector(Eigen::VectorXd& eigen_vec)
{
  std::vector<double> vector;
  int size = eigen_vec.size();
  vector.resize(size);

  for (int i = 0; i < size; i++)
  {
    vector.at(i) = eigen_vec(i);
  }
  return vector;
}

void JointAnglesController::printMatrix(Eigen::DiagonalMatrix<double, Eigen::Dynamic>& matrix,
                                        const std::string& matrix_name)
{
  std::cout << "####  " << matrix_name << "  ####  " << std::endl;
  for (int j = 0; j < matrix.diagonal().size(); j++)
  {
    for (int i = 0; i < matrix.diagonal().size(); i++)
      std::cout << matrix.toDenseMatrix()(j, i) << "  ";
    std::cout << "" << std::endl;
  }
}

void JointAnglesController::updateJointAccelerations(trajectory_msgs::JointTrajectory& traj_msg)
{
  for (auto& traj_pt : traj_msg.points)
  {
    updateCurrJointAngles();
    desd_position_ = curr_position_;

    for (int i = 0; i < traj_msg.joint_names.size(); i++)
    {
      int jt_no = getJointNumber(traj_msg.joint_names.at(i));
      desd_position_(jt_no) = traj_pt.positions.at(i);
    }

    updateDesdPosForJointLimits();
    updateCurrJointAngles();
    updateControlOutput();

    for (int i = 0; i < traj_msg.joint_names.size(); i++)
    {
      int jt_no = getJointNumber(traj_msg.joint_names.at(i));
      traj_pt.accelerations.at(i) = contr_output_(jt_no);
    }
  }
}

int JointAnglesController::getJointNumber(const std::string& joint_name)
{
  for (int i = 0; i < joint_names_.size(); i++)
  {
    if (joint_names_.at(i) == joint_name)
      return i;
  }
  ROS_ERROR("Could not find joint name %s in controller joint_names\n", joint_name);
  return -1;
}

void JointAnglesController::updateDesdPosForJointLimits()
{
  for (int i = 0; i < desd_position_.size(); i++)
  {
    desd_position_(i) = desd_position_(i) < joint_limits_.at(i).first ? joint_limits_.at(i).first : desd_position_(i);

    desd_position_(i) = desd_position_(i) > joint_limits_.at(i).second ? joint_limits_.at(i).second : desd_position_(i);
  }
}