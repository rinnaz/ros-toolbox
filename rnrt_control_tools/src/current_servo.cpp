#include "rnrt_control_tools/current_servo.h"

CurrentServo::CurrentServo()
    : m_current_last{0.0}
{
}

void CurrentServo::init(double &u_max,
                        double &gear_ratio,
                        const ros::NodeHandle &n,
                        const double &efficiency)
{
    // TODO: add range checks
    m_u_max = u_max;
    m_gear_ratio = gear_ratio;
    m_efficiency = efficiency;

    if (efficiency > 1.0 || efficiency <= 0.0)
    {
        ROS_WARN_STREAM("Efficiency is out of range, setting to 1.0");
        m_efficiency = 1.0;
    }

    initPid(n);
    initMotor(n);
}

void CurrentServo::initPid(const ros::NodeHandle &n)
{
    if (!m_pid_current->init(n))
    {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return;
    }
}

void CurrentServo::initMotor(const ros::NodeHandle &n)
{
    if (!m_motor->init(n))
    {
        ROS_WARN_STREAM("Failed to initialize motor from ROS parameter server.");
        return;
    }
}

double CurrentServo::getEffortResponse(const double &effort_command,
                                       const double &velocity,
                                       ros::Duration period)

{
    auto current_command = effort_command / m_motor->getKm() - m_current_last;

    auto voltage_command = m_pid_current->computeCommand(current_command, period);

    voltage_command = std::clamp(voltage_command, -m_u_max, m_u_max);

    m_current_last = m_motor->getCurrentResponse(voltage_command,
                                                 velocity * m_gear_ratio,
                                                 period.toNSec(),
                                                 SolverType::RUNGEKUTTA);

    return m_current_last * m_motor->getKm() * m_gear_ratio * m_efficiency;
}
