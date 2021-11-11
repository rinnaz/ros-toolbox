#pragma once

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include "rnrt_msgs/JointEffortFeedForward.h"

#include <string>

class EffortFFController
{
public:
    EffortFFController();
    ~EffortFFController(){};

    void callbackJointStates(const sensor_msgs::JointState &joint_state);

protected:
    virtual rnrt_msgs::JointEffortFeedForward
    composeEffortFFMsg(const sensor_msgs::JointState &joint_state) = 0;

    void initRosParameters();
    void initMoveitCore();

    const std::string m_node_name;

    ros::NodeHandle m_nh;
    ros::Subscriber m_sub_joint_states;
    ros::Publisher m_pub_effort_ff;

    std::string m_robot_description;
    std::string m_planning_group_name;
    std::string m_joint_states_topic;
    std::string m_effort_ff_topic;
    std::string m_base_frame_id;

    std::shared_ptr<robot_model_loader::RobotModelLoader> m_robot_model_loader;

    robot_model::RobotModelPtr m_kinematic_model;
    robot_state::RobotStatePtr m_kinematic_state;
    std::shared_ptr<robot_state::JointModelGroup> m_joint_model_group;
    std::vector<std::string> m_joint_names;
};
