#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

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

class LocalSearch
{
public:
    LocalSearch(const int s, const ParameterSetting &parameters, 
                const SolutionFirstEchelon &sol_FE, 
                const ScenarioSolutionSecondEchelon &sol_SE_scenario,
                const double objVal_Scenario);

    void RVND();

    ScenarioSolutionSecondEchelon getSolutionSE_Scenario();
    double getObjVal_Scenario();

private:
    int scenario;
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE_temp;
    ScenarioSolutionSecondEchelon sol_SE_temp_scenario;
    double objVal_Scenario;

    ScenarioSolutionSecondEchelon sol_SE_scenario_feasible;
    double objVal_feasible_scenario;

    ScenarioSolutionSecondEchelon sol_SE_best_scenario_currStart;
    double objVal_best_scenario_currStart;

    ScenarioSolutionSecondEchelon sol_SE_best_scenario;
    double objVal_best_scenario;

    vector<std::function<bool()>> setOperators();

    bool OrOpt(int v);
    bool Shift(int v);
    bool Swap(int v1, int v2);
    bool Insert();
    bool Remove();
    bool Merge();
    bool Transfer();
    bool Remove_Insert();

    bool solveLP_Scenario();

    // Helper Functions
    void printRoute(int warehouse, int period, int routeIndex, const vector<int> &route, const std::string &label = "") const;
    void printAllRoutes() const;

    // Random Number Generators
    std::mt19937 rng;
};

#endif // LOCALSEARCH_H