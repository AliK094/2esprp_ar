#ifndef LocalSearch_Deterministic_H
#define LocalSearch_Deterministic_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "LP_SE_Deterministic.h"
#include "R2EPRP.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

#include <thread>
#include <atomic>
#include <memory>

class LocalSearch_Deterministic
{
public:
    LocalSearch_Deterministic(const ParameterSetting &parameters, 
                              const SolutionFirstEchelon &sol_FE, 
                              const SolutionSecondEchelon_Deterministic &sol_SE,
                              const double objVal);

    void RVND();

    SolutionFirstEchelon getSolutionFE();
    SolutionSecondEchelon_Deterministic getSolutionSE();
    double getObjVal();

private:
    int scenario;
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE;
    SolutionSecondEchelon_Deterministic sol_SE_temp;
    double objVal;

    // SolutionSecondEchelon_Deterministic sol_SE_feasible;
    // double objVal_feasible;

    // SolutionFirstEchelon sol_FE_best_currStart;
    // SolutionSecondEchelon_Deterministic sol_SE_best_currStart;
    // double objVal_best_currStart;

    SolutionFirstEchelon sol_FE_best;
    SolutionSecondEchelon_Deterministic sol_SE_best;
    double objVal_best;

    vector<std::function<bool(SolutionSecondEchelon_Deterministic&)>> setOperators();

    bool OrOpt(int v, SolutionSecondEchelon_Deterministic &sol_SE);
    bool Shift(int v, SolutionSecondEchelon_Deterministic &sol_SE);
    bool Swap(int v1, int v2, SolutionSecondEchelon_Deterministic &sol_SE);
    bool Insert(SolutionSecondEchelon_Deterministic &sol_SE);
    bool Remove(SolutionSecondEchelon_Deterministic &sol_SE);
    bool Merge(SolutionSecondEchelon_Deterministic &sol_SE);
    bool Transfer(SolutionSecondEchelon_Deterministic &sol_SE);
    bool Remove_Insert(SolutionSecondEchelon_Deterministic &sol_SE);

    bool solveLP(SolutionFirstEchelon &sol_FE_best_currStart,
                SolutionSecondEchelon_Deterministic &sol_SE_best_currStart,
                double &objVal_best_currStart,
                SolutionSecondEchelon_Deterministic &sol_SE_feasible,
                double &objVal_feasible);

    // Helper Functions
    void printRoute(int warehouse, int period, int routeIndex, const vector<int> &route, const std::string &label = "") const;
    void printAllRoutes(SolutionSecondEchelon_Deterministic &sol_SE_feasible) const;

    // Random Number Generators
    std::mt19937 rng;
};

#endif // LocalSearch_Deterministic_H
