#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "LP_SE.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class LocalSearch
{
public:
    LocalSearch(int scenarioId, int warehouseId, const ParameterSetting &parameters, const Solution &initialSolution);

    void RVND();

    Solution getSolution();

private:
    int scenario;
    int warehouse;

    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol_temp;
    Solution sol_feasible;

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
    void printRoute(int scenario, int warehouse, int period, int routeIndex, const vector<int> &route, const std::string &label = "") const;
    void printAllRoutes() const;

    // Random Number Generators
    std::mt19937 rng;
};

#endif // LOCALSEARCH_H