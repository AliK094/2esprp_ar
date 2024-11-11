#ifndef PERTURBATION_Deterministic_H
#define PERTURBATION_Deterministic_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "deterministic/LP_SE_Deterministic.h"
#include "deterministic/LP_SE_EEV.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class Perturbation_Deterministic
{
public:
    Perturbation_Deterministic(const ParameterSetting &parameters, 
                 const SolutionFirstEchelon &sol_FE, 
                 const SolutionSecondEchelon_Deterministic &sol_SE,
                 const vector<vector<double>> &deterministicDemand,
                 bool shortageAllowed = true,
                 bool isEEV = false);

    bool run();

    SolutionFirstEchelon getSolutionFE();
    SolutionSecondEchelon_Deterministic getSolutionSE();
    double getObjVal();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE_temp;
    SolutionSecondEchelon_Deterministic sol_SE_temp;
    vector<vector<double>> demand;
    bool shortageAllowed;
    bool isEEV;

    SolutionFirstEchelon sol_FE_feasible;
    SolutionSecondEchelon_Deterministic sol_SE_feasible;
    double objVal_feasible;

    vector<std::function<bool()>> setPerturbOperators();
    bool randomShift(int v);
    bool randomSwap(int v1, int v2);
    bool randomInsertion();
    bool randomRemoval();

    // Helper Functions
    void printRoute(int warehouse, int period, int routeIndex, const vector<int> &route, const std::string &label = "") const;
    void printAllRoutes() const;

    // Random Number Generators
    std::mt19937 rng;
};

#endif // PERTURBATION_Deterministic_H