#ifndef PERTURBATION_H
#define PERTURBATION_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "LP_SE.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class Perturbation
{
public:
    Perturbation(const int s, const ParameterSetting &parameters, const Solution &solution);

    bool run();

    Solution getSolution();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol_temp;
    Solution sol_feasible;

    int scenario;
    // int warehouse;

    vector<std::function<bool()>> setPerturbOperators();
    bool randomShift(int v);
    bool randomInsertion();
    bool randomRemoval();

    // Helper Functions
    void printRoute(int scenario, int warehouse, int period, int routeIndex, const vector<int> &route, const std::string &label = "") const;
    void printAllRoutes() const;

    // Random Number Generators
    std::mt19937 rng;
};

#endif // PERTURBATION_H