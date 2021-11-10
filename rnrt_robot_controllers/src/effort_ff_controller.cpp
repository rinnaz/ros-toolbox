#include "rnrt_robot_controllers/effort_ff_controller.h"

EffortFFController::EffortFFController()
    : m_node_name{ros::this_node::getName()}
{
    initRosParameters();
    initMoveitCore();

    ROS_INFO("Model frame: %s", m_kinematic_model->getModelFrame().c_str());

    m_sub_joint_states = m_nh.subscribe(m_joint_states_topic,
                                        10,
                                        &EffortFFController::callbackJointStates,
                                        this);

    m_pub_effort_ff = m_nh.advertise<rnrt_msgs::JointEffortFeedForward>(m_effort_ff_topic, 1);
}

void EffortFFController::initRosParameters()
{
    m_nh.getParam(m_node_name + "/robot_description", m_robot_description);
    m_nh.getParam(m_node_name + "/planning_group_name", m_planning_group_name);
    m_nh.getParam(m_node_name + "/joint_states_topic", m_joint_states_topic);
    m_nh.getParam(m_node_name + "/effort_ff_topic", m_effort_ff_topic);
    m_nh.getParam(m_node_name + "/base_frame_id", m_base_frame_id);
}

void EffortFFController::initMoveitCore()
{
    m_robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(m_robot_description);

    m_kinematic_model = m_robot_model_loader->getModel();
    m_kinematic_state = std::make_shared<robot_state::RobotState>(m_kinematic_model);

    m_joint_model_group = std::shared_ptr<robot_state::JointModelGroup>(m_kinematic_model->getJointModelGroup("iiwa_arm"));
    m_joint_names = m_joint_model_group->getJointModelNames();
    m_joint_values_current = {0., 0., 0., 0., 0., 0., 0.};
}

void EffortFFController::callbackJointStates(const sensor_msgs::JointState &pose)
{
    m_joint_values_current = pose.position;
    m_pub_effort_ff.publish(composeEffortFFMsg());
}

rnrt_msgs::JointEffortFeedForward EffortFFController::composeEffortFFMsg()
{
    rnrt_msgs::JointEffortFeedForward ret;
    ret.header.stamp = ros::Time::now();
    ret.header.frame_id = m_base_frame_id;
    ret.name = m_joint_names;
    for (auto i{0}; i < m_joint_names.size(); i++)
    {
        ret.effort_feed_forward.push_back(0.0);
    }
    return ret;
}
