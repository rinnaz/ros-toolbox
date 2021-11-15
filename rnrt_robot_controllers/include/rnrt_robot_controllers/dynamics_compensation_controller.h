#pragma once

#include "rnrt_robot_controllers/gravity_compensation_controller.h"

class DynamicsCompenationController : public GravityCompenationController
{
public:
    DynamicsCompenationController();
    rnrt_msgs::JointEffortFeedForward
    composeEffortFFMsg(const sensor_msgs::JointState &joint_state) override;

protected:
    std::vector<double> calcAcceleration(const std::vector<double> &,
                                         const ros::Time &);

    std::vector<double> m_velocity_last;
    ros::Time m_time_last;
};
