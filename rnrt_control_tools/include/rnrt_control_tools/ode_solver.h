#ifndef ODE_SOLVER_H
#define ODE_SOLVER_H
#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include <numeric>

using namespace std;

ostream& operator<< (ostream& out, const vector<float>& right);
const vector<float> operator* (const vector<float>& left, const vector<float>& right);
const vector<float> operator* (const float& left, const vector<float>& right);
const vector<float> operator* (const vector<float>& left, const float& right);
const vector<float> operator/ (const vector<float>& right, const float& left);
const vector<float> operator+ (const vector<float>& left, const vector<float>& right);
const vector<float> operator- (const vector<float>& left, const vector<float>& right);
const vector<float> operator- (int, const vector<float>& right);


class OdeSolver
{
public:
//    data
    enum Solver {EULER, RUNGEKUTTA};

//    methods
    friend ostream& operator<< (ostream& out, const OdeSolver& a);

    OdeSolver();
    OdeSolver(const vector<float> list);
    void            initialize(const vector<float> inputEquation);
    void            solve(float computationTime, float stepValue, OdeSolver::Solver = EULER, ostream& out = cout);


private:
    vector<float>   equation;
    vector<float>   currentCondition;
    uint32_t        order;
    float           stepSize;

    vector<float>   normalize(const vector<float> inputEquation);
    vector<float>   computeDerivatives(const vector<float> condition);
    void            eulerCompute();
    void            rungekuttaCompute();
    void            (OdeSolver::*compute)();
};

#endif // ODE_SOLVER_H
