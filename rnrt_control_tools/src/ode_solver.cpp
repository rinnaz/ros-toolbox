#include "rnrt_control_tools/ode_solver.h"

ostream& operator<<(ostream &out, const OdeSolver& a){
    out << a.equation << endl << a.order;

    return out;
}

OdeSolver::OdeSolver(const vector<double> list)
{
    this->initialize(list);
}

void OdeSolver::initialize(const vector<double> inputEquation)
{
    this->equation.reserve(inputEquation.size());
    this->equation = this->normalize(inputEquation);

    this->order = inputEquation.size() - 2;
    this->currentCondition.reserve(this->order);
    for(uint32_t i = 0; i < this->order; i++){
        this->currentCondition.push_back(0.0);
    }

}

void OdeSolver::solve(double computationTime, double stepValue, OdeSolver::Solver solver, ostream& out)
{
    this->stepSize = stepValue;
    cout << std::fixed;

    switch (solver) {
    case Solver::EULER:
        compute = &OdeSolver::eulerCompute;
        break;

    case Solver::RUNGEKUTTA:
        compute = &OdeSolver::rungekuttaCompute;
        break;
    }

    for(double i = this->stepSize; i <= computationTime; i += this->stepSize){
        (this->*compute)();
        cout << this->currentCondition << i << endl;
    }

}

vector<double> OdeSolver::normalize(const vector<double> inputEquation)
{
    vector<double> result;

    for(auto i : inputEquation){
        result.push_back(i/inputEquation.at(0));
    }

    return result;
}

vector<double> OdeSolver::computeDerivatives(const vector<double> condition)
{
    vector<double> result;
    result.reserve(condition.size());

//    result = condition[1:]
    result = vector<double>(condition.begin() + 1, condition.end());

//    temp = eq[1:-1] * condition
    auto temp = vector<double>(this->equation.rbegin() + 1, this->equation.rend() - 1) * condition;

//    result.append(-eq[last] - sum(temp))
    result.push_back(-this->equation.back() - accumulate(temp.begin(), temp.end(), 0.0));

    return result;
}


void OdeSolver::eulerCompute()
{
    this->currentCondition = this->currentCondition +
            this->stepSize * this->computeDerivatives(this->currentCondition);
}

void OdeSolver::rungekuttaCompute()
{
    vector<double> k1, k2, k3, k4;
    k1.reserve(this->currentCondition.size());
    k2.reserve(this->currentCondition.size());
    k3.reserve(this->currentCondition.size());
    k4.reserve(this->currentCondition.size());

    k1 = this->stepSize * this->computeDerivatives(this->currentCondition);
    k2 = this->stepSize * this->computeDerivatives(this->currentCondition + k1/2.0);
    k3 = this->stepSize * this->computeDerivatives(this->currentCondition + k2/2.0);
    k4 = this->stepSize * this->computeDerivatives(this->currentCondition + k3);

    this->currentCondition = this->currentCondition +
            (k1 + 2.0*k2 + 2.0*k3 + k4)/6;
}

ostream& operator<<(ostream &out, const vector<double> &right)
{
    cout << std::setprecision(5);

    for(auto a : right){
        out << a << " ";
    }

    cout << std::setprecision(2);

    return out;
}

const vector<double> operator* (const vector<double>& left, const vector<double>& right){
    vector<double> result;

    for (uint32_t i = 0; i < left.size(); i++){
        result.push_back(left.at(i) * right.at(i));
    }

    return result;
}

const vector<double> operator*(const double &left, const vector<double> &right)
{
    vector<double> result;
    for (auto i : right){
        result.push_back(i * left);
    }

    return result;
}

const vector<double> operator+(const vector<double> &left, const vector<double> &right)
{
    vector<double> result;

    for (uint32_t i = 0; i < left.size(); i++){
        result.push_back(left.at(i) + right.at(i));
    }

    return result;
}

const vector<double> operator/(const vector<double> &left, const double &right)
{
    return (1/right) * left;
}

const vector<double> operator*(const vector<double> &left, const double &right)
{
    return right * left;
}

const vector<double> operator-(const vector<double> &left, const vector<double> &right)
{
    return left + (-1)*right;
}

const vector<double> operator-(int, const vector<double> &right)
{
    return (-1)*right;
}
