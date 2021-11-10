#pragma once

#include "rnrt_robot_controllers/effort_ff_controller.h"
#include <geometry_msgs/Wrench.h>
#include <moveit/dynamics_solver/dynamics_solver.h>

class GravityCompenationController : public EffortFFController
{
public:
    GravityCompenationController();
    rnrt_msgs::JointEffortFeedForward composeEffortFFMsg() override;

protected:
    geometry_msgs::Wrench composeZeroWrenchMsg();
    geometry_msgs::Vector3 composeGravityMsg(const double &x,
                                             const double &y,
                                             const double &z);
    geometry_msgs::Vector3 m_gravity;
    dynamics_solver::DynamicsSolverPtr m_dynamic_solver;
    std::vector<geometry_msgs::Wrench> m_wrenches;
};