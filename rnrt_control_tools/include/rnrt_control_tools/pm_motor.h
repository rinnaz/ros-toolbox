#pragma once

#include <memory>
#include <vector>
#include "eigen3/Eigen/Core"

#include "rnrt_control_tools/state_space_model.h"

class PmMotor
{
public:
    PmMotor();
    PmMotor(const double &ind,
            const double &res,
            const double &km,
            const uint64_t &pole_pairs = 1);
    ~PmMotor(){};

    void setParameters(const double &ind,
                       const double &res,
                       const double &km,

                       const uint64_t &pole_pairs = 1);

    void setInductance(const double &ind);
    void setResistance(const double &res);
    void setKm(const double &km);
    void setTe(const double &te);
    void setPolePairs(const uint64_t &pole_pairs = 1);
    void initStateSpaceModel();

    double getCurrentResponse(const double &input_voltage,
                              const double &current_velocity,
                              const uint64_t &dt,
                              const SolverType = SolverType::EULER);

    double getTorqueResponse(const double &input_voltage,
                             const double &current_velocity,
                             const uint64_t &dt,
                             const SolverType = SolverType::EULER);

    double getKm() { return m_km; }
    double getKe() { return m_ke; }

protected:
    double m_l;  // inductance
    double m_r;  // resistance
    double m_te; // electrical time constant
    double m_km; // torque constant
    double m_ke; // electromechanical constant
    uint64_t m_pole_pairs;

    std::shared_ptr<StateSpaceModel> m_state_space_model_ptr;
};