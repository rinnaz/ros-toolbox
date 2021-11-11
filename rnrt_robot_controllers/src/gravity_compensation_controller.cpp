#include "rnrt_robot_controllers/gravity_compensation_controller.h"

GravityCompenationController::GravityCompenationController()
{
    std::vector<double> gravity_vector;
    m_nh.getParam(m_node_name + "/gravity_vector", gravity_vector);

    try
    {
        m_gravity = composeGravityMsg(gravity_vector);
    }
    catch (const std::invalid_argument e)
    {
        ROS_ERROR("%s", e.what());
        m_gravity = composeGravityMsg({0.0, 0.0, -9.8});
    }

    m_dynamic_solver = std::make_shared<dynamics_solver::DynamicsSolver>(m_kinematic_model,
                                                                         m_planning_group_name,
                                                                         m_gravity);
    for (auto i : m_joint_names)
    {
        m_wrenches.push_back(composeZeroWrenchMsg());
    }
}

rnrt_msgs::JointEffortFeedForward
GravityCompenationController::composeEffortFFMsg(const sensor_msgs::JointState &joint_state)
{
    rnrt_msgs::JointEffortFeedForward ret;
    const std::vector<double> m_zeros(m_joint_names.size(), 0.0);
    std::vector<double> m_torques(m_joint_names.size(), 0.0);
    ret.header.stamp = ros::Time::now();
    ret.header.frame_id = m_base_frame_id;
    ret.name = m_joint_names;
    m_dynamic_solver->getTorques(joint_state.position,
                                 m_zeros,
                                 m_zeros,
                                 m_wrenches,
                                 m_torques);

    ret.effort_feed_forward = m_torques;
    return ret;
}

geometry_msgs::Vector3
GravityCompenationController::composeGravityMsg(const std::vector<double> &gravity_vector)
{
    if (gravity_vector.size() != 3)
    {
        throw std::invalid_argument("Invalid gravity vector size");
    }

    geometry_msgs::Vector3 ret;
    ret.x = gravity_vector.at(0);
    ret.y = gravity_vector.at(1);
    ret.z = gravity_vector.at(2);
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
