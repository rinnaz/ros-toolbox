#include "rnrt_robot_controllers/gravity_compensation_controller.h"

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