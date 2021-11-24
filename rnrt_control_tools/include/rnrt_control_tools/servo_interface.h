#pragma once

#include <memory>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

class ServoInterface
{
public:
    ServoInterface(){};
    ~ServoInterface(){};

    virtual bool init(const ros::NodeHandle &nh, std::string &joint_name) = 0;

    virtual double getEffortResponse(const double &command,
                             const double &position,
                             const double &velocity,
                             const double &effort,
                             ros::Duration period) = 0;
    
    virtual void reset() = 0;

protected:

};
