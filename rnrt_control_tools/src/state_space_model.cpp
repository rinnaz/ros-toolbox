#include "rnrt_control_tools/state_space_model.h"

StateSpaceModel::StateSpaceModel(const TransferFcn &tfcn)
    : m_matrix_size{(tfcn.getDenominator()).size() - 1},
      m_current_state(m_matrix_size),
      m_A_matrix(m_matrix_size, m_matrix_size),
      m_B_vector(m_matrix_size),
      m_C_vector(m_matrix_size)
{
}

StateSpaceModel::~StateSpaceModel() {}

Eigen::MatrixXd StateSpaceModel::setAMatrix(const std::vector<double> &a_vec)
{
    Eigen::MatrixXd result;
    return result;
}

Eigen::VectorXd StateSpaceModel::setBVector(const std::vector<double> &a_vec)
{
    Eigen::VectorXd result;
    return result;
}

Eigen::RowVectorXd StateSpaceModel::setCRowVector(const std::vector<double> &num)
{
    Eigen::RowVectorXd result;
    return result;
}