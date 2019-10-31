#include <tough_common/robot_state.h>
#include <eigen3/Eigen3.3.7/Core>

class JointAnglesController
{
private:
  int chest_index_, left_arm_index_, right_arm_index_;
  int chest_joint_number_, left_arm_joint_number_, right_arm_joint_number_;
  int chest_size_, left_arm_size_, right_arm_size_;

  Eigen::DiagonalMatrix<double, Eigen::Dynamic> k_p_, k_d_, derivative_;
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> p_out_, d_out_, contr_output_;
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> curr_position_, prev_position_, desd_position_;
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> error_, prev_error_;

  size_t total_joints_size_;
  double max_jt_accn, min_jt_accn;
  double d_t;

  ros::NodeHandle nh_;
  RobotStateInformer* state_informer_;
  RobotDescription* rd_;

  void initializeVariables();
  void initializeMatrices(const size_t size);
  

  inline void limitAccelerations(Eigen::DiagonalMatrix<double, Eigen::Dynamic>& matrix)
  {
    for(int i = 0; i<matrix.diagonal().size();i++)
    {
      if(matrix.diagonal()(i)>max_jt_accn)
      {
        matrix.diagonal()(i) = max_jt_accn;
      }
      else if(matrix.diagonal()(i)<min_jt_accn)
        matrix.diagonal()(i) = min_jt_accn;
    }
  }

  /**
   * @brief Inserts values in the matrix
   * 
   * @param vector            - Input vector
   * @param start_index       - Index of the first element which should go into matrix
   * @param end_index         - Index of the element next to the last element. (or start_index + size)
   * @param matrix            - [output]
   */
  inline void insertValuesInMatrix(const std::vector<double>& vector, const int start_index, const int end_index,
                                   Eigen::DiagonalMatrix<double, Eigen::Dynamic>& matrix)
  {
    for (int i = start_index; i < end_index; i++)
    {
      matrix.diagonal()(i) = vector.at(i);
    }
  }

  /*
      SHOULD BE IMPLEMENTED USING THE RETURN VALUE OPTIMISATION, NOT CALL BY REF.
      THIS IS ONLY TEMPORARY FIX  
  */
  inline void vectorToDiagonalMatrix(const std::vector<double>& vector, Eigen::DiagonalMatrix<double, Eigen::Dynamic>& matrix)
  {
    int size = vector.size();

    for (int i = 0; i < vector.size(); i++)
    {
      matrix.diagonal()(i) = vector.at(i);
    }
  }

  inline std::vector<double> diagonalMatrixToVector(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& matrix)
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

public:
  JointAnglesController(ros::NodeHandle nh);
  ~JointAnglesController();

  /**
   * @brief Get the Controlled Joint Angles object
   * 
   * @param joint_angles            - Vector of the desired positions for the chest, left arm and right arm.
   *                                  Indices for the start of the joints should be inquired through getters.
   * @return std::vector<double>    - Returns the vector of accelerations 
   */
  std::vector<double> getControlledJointAngles(const std::vector<double>& joint_angles);

  void getKp(const int joint_number) const;
  void getKd(const int joint_number) const;

  void setKp(const double Kp, const int joint_number);
  void setKd(const double Kd, const int joint_number);

  std::vector<double> getJointsKp() const;
  std::vector<double> getJointsKd() const;

  void setJointsKp(const std::vector<double>& joints_kp);
  void setJointsKd(const std::vector<double>& joints_kd);

  std::vector<double> getDefaultGains();
  void setDefaultGains();

  int getChestIndexAcceleration() const;
  int getLeftArmIndexAcceleration() const;
  int getRightArmIndexAcceleration() const;
};

Eigen::DiagonalMatrix<double, Eigen::Dynamic>
operator-(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& mat_1,
          const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& mat_2)
{
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> output_mat;
  output_mat.diagonal() = mat_1.diagonal() - mat_2.diagonal();
  return output_mat;
}

Eigen::DiagonalMatrix<double, Eigen::Dynamic>
operator+(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& mat_1,
          const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& mat_2)
{
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> output_mat;
  output_mat.diagonal() = mat_1.diagonal() + mat_2.diagonal();
  return output_mat;
}

Eigen::DiagonalMatrix<double, Eigen::Dynamic>
operator*(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& mat_1,
          const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& mat_2)
{
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> output_mat;
  output_mat.diagonal() = mat_1.toDenseMatrix() * mat_2.diagonal();
  return output_mat;
}

Eigen::DiagonalMatrix<double, Eigen::Dynamic>
operator/(const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& mat_1, const int scaler)
{
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> output_mat;
  output_mat.diagonal() = mat_1.diagonal() / scaler;
  return output_mat;
}