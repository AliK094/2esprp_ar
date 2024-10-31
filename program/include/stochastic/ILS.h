#ifndef ILS_SIRP_H
#define ILS_SIRP_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "stochastic/ConsInitSol.h"
#include "stochastic/LocalSearch.h"
#include "stochastic/Perturbation.h"
#include "stochastic/LP_SE.h"
#include "stochastic/LP_SE_Scenario.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

#include <thread>
#include <atomic>
#include <memory>

class ILS_SIRP
{
public:
    ILS_SIRP(const ParameterSetting &parameters,
             const SolutionFirstEchelon &sol_FE,
             const SolutionSecondEchelon &sol_SE = {});

    bool run();

    SolutionFirstEchelon getSolutionFE();
    SolutionSecondEchelon getSolutionSE();
    Result getResult();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE;
    SolutionSecondEchelon sol_SE;

    SolutionFirstEchelon sol_FE_incumbent;
    SolutionSecondEchelon sol_SE_incumbent;
    Result result_incumbent;

    bool createInitSol;

    vector<vector<vector<vector<int>>>> CATW;

    double Tolerance;

    bool solveLP(SolutionFirstEchelon &sol_FE_temp, SolutionSecondEchelon &sol_SE_temp, Result &result_temp);
    bool checkSolutionFeasiblity(SolutionFirstEchelon sol_FE_temp, SolutionSecondEchelon sol_SE_temp);
    void calculateObjFuncValue();
    vector<vector<vector<double>>> calcInvWarehouse();
};

#endif // ILS_SIRP_H