#ifndef PERTURBATION_H
#define PERTURBATION_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "LP_SE.h"
#include "LP_SE_Scenario.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class Perturbation
{
public:
    Perturbation(const int s, const ParameterSetting &parameters, 
                const SolutionFirstEchelon &sol_FE, 
                const ScenarioSolutionSecondEchelon &sol_SE_scenario);

    bool run();

    ScenarioSolutionSecondEchelon getSolutionSE_Scenario();
    double getObjVal_Scenario();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE_temp;
    ScenarioSolutionSecondEchelon sol_SE_temp_scenario;

    SolutionFirstEchelon sol_FE_feasible;
    ScenarioSolutionSecondEchelon sol_SE_scenario_feasible;
    double objVal_feasible_scenario;

    int scenario;
    // int warehouse;

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

#endif // PERTURBATION_H