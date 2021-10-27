#ifndef ODE_SOLVER_H
#define ODE_SOLVER_H
#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include <numeric>

using namespace std;

ostream& operator<< (ostream& out, const vector<double>& right);
const vector<double> operator* (const vector<double>& left, const vector<double>& right);
const vector<double> operator* (const double& left, const vector<double>& right);
const vector<double> operator* (const vector<double>& left, const double& right);
const vector<double> operator/ (const vector<double>& right, const double& left);
const vector<double> operator+ (const vector<double>& left, const vector<double>& right);
const vector<double> operator- (const vector<double>& left, const vector<double>& right);
const vector<double> operator- (int, const vector<double>& right);


class OdeSolver
{
public:
//    data
    enum Solver {EULER, RUNGEKUTTA};

//    methods
    friend ostream& operator<< (ostream& out, const OdeSolver& a);

    OdeSolver();
    OdeSolver(const vector<double> list);
    void            initialize(const vector<double> inputEquation);
    void            solve(double computationTime, double stepValue, OdeSolver::Solver = EULER, ostream& out = cout);


private:
    vector<double>   equation;
    vector<double>   currentCondition;
    uint32_t        order;
    double           stepSize;

    vector<double>   normalize(const vector<double> inputEquation);
    vector<double>   computeDerivatives(const vector<double> condition);
    void            eulerCompute();
    void            rungekuttaCompute();
    void            (OdeSolver::*compute)();
};

#endif // ODE_SOLVER_H
