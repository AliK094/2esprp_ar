#ifndef ILS_SIRP_H
#define ILS_SIRP_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "ConsInitSol.h"
#include "LocalSearch.h"
#include "Perturbation.h"
#include "LP_SE.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class ILS_SIRP
{
public:
    ILS_SIRP(const ParameterSetting &parameters, const Solution &solution);

    bool run();

    Solution getSolution();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol;
    bool createInitSol;

    Solution sol_temp;
    vector<vector<vector<vector<int>>>> CATW;

    Solution sol_incumbent;


    double Tolerance;

    bool checkSolutionFeasiblity();
    void calculateObjFuncValue();
    vector<vector<vector<double>>> calcInvWarehouse();


    double objValue_FE;
    double objValue_SE;
};

#endif // ILS_SIRP_H