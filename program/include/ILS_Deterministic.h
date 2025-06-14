#ifndef ILS_SIRP_Deterministic_H
#define ILS_SIRP_Deterministic_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "ConsInitSol_Deterministic.h"
#include "LocalSearch_Deterministic.h"
#include "Perturbation_Deterministic.h"
#include "LP_SE_Deterministic.h"
#include "SolutionManager.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class ILS_SIRP_Deterministic
{
public:
    ILS_SIRP_Deterministic(const ParameterSetting &parameters,
                           const SolutionFirstEchelon &sol_FE,
                           const SolutionSecondEchelon_Deterministic &sol_SE = {},
                           int iter = -1,
                           std::chrono::high_resolution_clock::time_point startTime_HHA = std::chrono::high_resolution_clock::time_point{},
                           bool savePerIterSol = false);

    bool run();

    SolutionFirstEchelon getSolutionFE();
    SolutionSecondEchelon_Deterministic getSolutionSE();
    Result getResult();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE;
    SolutionSecondEchelon_Deterministic sol_SE;

    SolutionFirstEchelon sol_FE_incumbent;
    SolutionSecondEchelon_Deterministic sol_SE_incumbent;
    Result result_incumbent;

    bool createInitSol;

    vector<vector<vector<int>>> CATW;

    double Tolerance;

    // -----------------------------------------------------------------------------------------------
    int HHA_iter;
    std::chrono::high_resolution_clock::time_point startTime_HHA;
	bool savePerIterSol;
    // -----------------------------------------------------------------------------------------------

    bool solveLP(SolutionFirstEchelon &sol_FE_temp, SolutionSecondEchelon_Deterministic &sol_SE_temp, Result &result_temp);
    bool checkSolutionFeasiblity(SolutionFirstEchelon sol_FE_temp, SolutionSecondEchelon_Deterministic sol_SE_temp);
    void calculateObjFuncValue();
    vector<vector<vector<double>>> calcInvWarehouse();
};

#endif // ILS_SIRP_Deterministic_H