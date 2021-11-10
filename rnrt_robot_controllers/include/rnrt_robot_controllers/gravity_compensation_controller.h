#pragma once

#include "rnrt_robot_controllers/effort_ff_controller.h"

class GravityCompenationController : public EffortFFController
{
public:
    rnrt_msgs::JointEffortFeedForward composeEffortFFMsg() override;
};