#include "rnrt_robot_controllers/gravity_compensation_controller.h"

GravityCompenationController::GravityCompenationController()
{
    m_gravity = composeGravityMsg(0.0, 0.0, -9.8);
    m_dynamic_solver = std::make_shared<dynamics_solver::DynamicsSolver>(m_kinematic_model,
                                                                         m_planning_group_name,
                                                                         m_gravity);
    for (auto i : m_joint_values_current)
    {
        m_wrenches.push_back(composeZeroWrenchMsg());
    }
}

rnrt_msgs::JointEffortFeedForward GravityCompenationController::composeEffortFFMsg()
{
    rnrt_msgs::JointEffortFeedForward ret;
    const std::vector<double> m_zeros(m_joint_values_current.size(), 0.0);
    std::vector<double> m_torques(m_joint_values_current.size(), 0.0);
    ret.header.stamp = ros::Time::now();
    ret.header.frame_id = m_base_frame_id;
    ret.name = m_joint_names;
    m_dynamic_solver->getTorques(m_joint_values_current,
                                 m_zeros,
                                 m_zeros,
                                 m_wrenches,
                                 m_torques);

    ret.effort_feed_forward = m_torques;
    return ret;
}

geometry_msgs::Vector3 GravityCompenationController::composeGravityMsg(const double &x,
                                                                       const double &y,
                                                                       const double &z)
{
    geometry_msgs::Vector3 ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    return ret;
}

geometry_msgs::Wrench GravityCompenationController::composeZeroWrenchMsg()
{
    geometry_msgs::Wrench ret;
    geometry_msgs::Vector3 temp_v3;
    temp_v3.x = 0.0;
    temp_v3.y = 0.0;
    temp_v3.z = 0.0;

    ret.force = temp_v3;
    ret.torque = temp_v3;
    return ret;
}
