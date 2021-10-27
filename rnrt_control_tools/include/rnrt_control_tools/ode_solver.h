#ifndef ODE_SOLVER_H
#define ODE_SOLVER_H
#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include <numeric>

using namespace std;

ostream &operator<<(ostream &out, const vector<double> &right);
const vector<double> operator*(const vector<double> &left, const vector<double> &right);
const vector<double> operator*(const double &left, const vector<double> &right);
const vector<double> operator*(const vector<double> &left, const double &right);
const vector<double> operator/(const vector<double> &right, const double &left);
const vector<double> operator+(const vector<double> &left, const vector<double> &right);
const vector<double> operator-(const vector<double> &left, const vector<double> &right);
const vector<double> operator-(int, const vector<double> &right);

enum class SolverType
{
    EULER,
    RUNGEKUTTA
};

class TransferFcn
{
public:
    TransferFcn();
    TransferFcn(const vector<double>& num, const vector<double>& den)
        : m_numerator{num}, m_denominator{den}
    {};

    ~TransferFcn();
    void setNumerator(const vector<double>& num){m_numerator = num;};
    void setDenominator(const vector<double>& den){m_denominator = den;};
    vector<double> getNumerator(){return m_numerator;};
    vector<double> getDenominator(){return m_denominator;};

private:
    vector<double> m_numerator;
    vector<double> m_denominator;
};

class OdeSolver
{
public:
    //    methods
    friend ostream &operator<<(ostream &out, const OdeSolver &a);

    OdeSolver();
    OdeSolver(const vector<double> inputEquation);
    void initialize(const vector<double> inputEquation);
    void solve(double computationTime, double stepValue, SolverType = SolverType::EULER, ostream &out = cout);

private:
    vector<double> m_equation;
    vector<double> m_currentCondition;
    uint64_t m_order;
    double m_stepSize;
    vector<double> m_k1, m_k2, m_k3, m_k4;

    vector<double> normalize(const vector<double> inputEquation) const;
    vector<double> computeDerivatives(const vector<double> condition) const;
    void eulerCompute();
    void rungekuttaCompute();
    void (OdeSolver::*compute)();
};

#endif // ODE_SOLVER_H
