#ifndef ILS_EEV_H
#define ILS_EEV_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "deterministic/ConsInitSol_Deterministic.h"
#include "deterministic/LocalSearch_Deterministic.h"
#include "deterministic/Perturbation_Deterministic.h"
#include "deterministic/LP_SE_EEV.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class ILS_EEV
{
public:
    ILS_EEV(const ParameterSetting &parameters,
            const SolutionFirstEchelon &sol_FE,
            const SolutionSecondEchelon_Deterministic &sol_SE = {},
            const vector<vector<double>> &deterministicDemand = {});

    bool run();

    SolutionSecondEchelon_Deterministic getSolutionSE();
    Result getResult();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE_EV;
    SolutionSecondEchelon_Deterministic sol_SE;
    vector<vector<double>> demand;

    SolutionSecondEchelon_Deterministic sol_SE_incumbent;
    Result result_incumbent;

    bool createInitSol;

    vector<vector<vector<int>>> CATW;

    double Tolerance;

    bool solveLP(SolutionSecondEchelon_Deterministic &sol_SE_temp, Result &result_temp);
    bool checkSolutionFeasiblity(SolutionSecondEchelon_Deterministic sol_SE_temp);
    void calculateObjFuncValue();
    vector<vector<vector<double>>> calcInvWarehouse();
};

#endif // ILS_EEV_H