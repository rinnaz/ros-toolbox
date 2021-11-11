#pragma once

#include "rnrt_robot_controllers/effort_ff_controller.h"
#include <geometry_msgs/Wrench.h>
#include <moveit/dynamics_solver/dynamics_solver.h>

class GravityCompenationController : public EffortFFController
{
public:
    GravityCompenationController();
    rnrt_msgs::JointEffortFeedForward
    composeEffortFFMsg(const sensor_msgs::JointState &joint_state) override;

protected:
    geometry_msgs::Wrench composeZeroWrenchMsg();
    geometry_msgs::Vector3 composeGravityMsg(const std::vector<double> &gravity_vector);
    geometry_msgs::Vector3 m_gravity;
    dynamics_solver::DynamicsSolverPtr m_dynamic_solver;
    std::vector<geometry_msgs::Wrench> m_wrenches;
};
