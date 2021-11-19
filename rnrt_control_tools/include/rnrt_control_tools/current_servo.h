#pragma once

#include <memory>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include "rnrt_control_tools/pm_motor.h"
#include <control_toolbox/pid.h>

class CurrentServo
{
public:
    CurrentServo();
    ~CurrentServo(){};

    void init(double &u_max,
              double &gear_ratio,
              const ros::NodeHandle &n,
              const double &efficiency = 1.0);
    void initPid(const ros::NodeHandle &n);
    void initMotor(const ros::NodeHandle &n);

    double getEffortResponse(const double &effort_command,
                             const double &velocity,
                             ros::Duration period);

protected:
    double m_u_max;
    double m_current_last;
    double m_gear_ratio;
    double m_efficiency;

    using MotorPtr = std::shared_ptr<PmMotor>;
    using PidPtr = std::shared_ptr<control_toolbox::Pid>;

    MotorPtr m_motor;
    PidPtr m_pid_current;
};
