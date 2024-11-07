#ifndef LocalSearch_Deterministic_H
#define LocalSearch_Deterministic_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "deterministic/LP_SE_Deterministic.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class LocalSearch_Deterministic
{
public:
    LocalSearch_Deterministic(const ParameterSetting &parameters, 
                              const SolutionFirstEchelon &sol_FE, 
                              const SolutionSecondEchelon_Deterministic &sol_SE,
                              const double objVal,
                              const vector<vector<double>> &deterministicDemand,
						      bool shortageAllowed = true);

    void RVND();

    SolutionSecondEchelon_Deterministic getSolutionSE();
    double getObjVal();

private:
    int scenario;
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE_temp;
    SolutionSecondEchelon_Deterministic sol_SE_temp;
    double objVal;
    vector<vector<double>> demand;
    bool shortageAllowed;

    SolutionSecondEchelon_Deterministic sol_SE_feasible;
    double objVal_feasible;

    SolutionSecondEchelon_Deterministic sol_SE_best_currStart;
    double objVal_best_currStart;

    SolutionSecondEchelon_Deterministic sol_SE_best;
    double objVal_best;

    vector<std::function<bool()>> setOperators();

    bool OrOpt(int v);
    bool Shift(int v);
    bool Swap(int v1, int v2);
    bool Insert();
    bool Remove();
    bool Merge();
    bool Transfer();
    bool Remove_Insert();

    bool solveLP();

    // Helper Functions
    void printRoute(int warehouse, int period, int routeIndex, const vector<int> &route, const std::string &label = "") const;
    void printAllRoutes() const;

    // Random Number Generators
    std::mt19937 rng;
};

#endif // LocalSearch_Deterministic_H