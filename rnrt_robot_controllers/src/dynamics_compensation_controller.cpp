#include "rnrt_robot_controllers/dynamics_compensation_controller.h"

DynamicsCompenationController::DynamicsCompenationController()
{
}

rnrt_msgs::JointEffortFeedForward
DynamicsCompenationController::composeEffortFFMsg(const sensor_msgs::JointState &joint_state)
{
    const std::vector<double> zeros(m_joint_names.size(), 0.0);
    rnrt_msgs::JointEffortFeedForward ret;
    std::vector<double> torques(m_joint_names.size(), 0.0);
    ret.header.stamp = ros::Time::now();
    ret.header.frame_id = m_base_frame_id;
    ret.name = m_joint_names;
    m_dynamic_solver->getTorques(joint_state.position,
                                 joint_state.velocity,
                                 m_time_last.isZero() ? zeros : calcAcceleration(joint_state.velocity, joint_state.header.stamp),
                                 m_wrenches,
                                 torques);

    m_velocity_last = joint_state.velocity;
    m_time_last = joint_state.header.stamp;

    ret.effort_feed_forward = torques;
    return ret;
}

std::vector<double>
DynamicsCompenationController::calcAcceleration(const std::vector<double> &velocity,
                                                const ros::Time &time)
{
    std::vector<double> result;
    for (auto i{0}; i < m_joint_names.size(); i++)
    {
        result.push_back(0.95*(velocity[i] - m_velocity_last[i]) / (time - m_time_last).toSec());
    }
    return result;
}
