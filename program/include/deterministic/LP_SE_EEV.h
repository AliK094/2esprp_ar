#ifndef LP_SE_EEV_H
#define LP_SE_EEV_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class LP_SE_EEV
{
public:
    LP_SE_EEV(const ParameterSetting &parameters, 
             const SolutionFirstEchelon &solFE,
			 const SolutionSecondEchelon_Deterministic &solSE,
             const vector<vector<double>> &deterministicDemand);

    string solve();

    SolutionFirstEchelon getSolutionFE();
    SolutionSecondEchelon_Deterministic getSolutionSE();
    Result getResult();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE_EV;
    SolutionSecondEchelon_Deterministic sol_SE;
    vector<vector<double>> demand;

    SolutionFirstEchelon sol_FE_temp;
    SolutionSecondEchelon_Deterministic sol_SE_temp;
    Result result;

    double THRESHOLD;
    
    bool save_lpFile;
    bool save_mpsResultFile;

    // Decision Variables
    IloArray<IloNumVarArray> I_warehouse;
    IloArray<IloNumVarArray> I_customer;
    IloArray<IloNumVarArray> b_customer;
    IloArray<IloArray<IloNumVarArray>> w_customer;

    void DefineVariables(IloEnv &env, IloModel &model);
    void DefineObjectiveFunction(IloEnv &env, IloModel &model);
    void DefineConstraints(IloEnv &env, IloModel &model);
    void RetrieveSolutions(IloCplex &cplex);
    void CalculateCostsForEachPart();

    void DisplayCosts();
    void DisplayProductionSetupVars();
    void DisplayProductionQuantVars();
    void DisplayPlantInventoryVars();
    void DisplayWarehouseInventoryVars();
    void DisplayFirstEchelonRouteVars();
    void DisplayDeliveryQuantityToWarehousesVars();
    void DisplayCustomerInventoryVars();
    void DisplayCustomerUnmetDemandVars();
    void DisplayDeliveryQuantityToCustomersVars();
    void DisplayRoutesWarehouseToCustomersVars();
};

#endif // LP_SE_EEV_H