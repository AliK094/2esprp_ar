#ifndef LPSE_Deterministic_H
#define LPSE_Deterministic_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class LP_SE_Deterministic
{
public:
    LP_SE_Deterministic(const ParameterSetting &parameters, 
             const SolutionFirstEchelon &solFE,
			 const SolutionSecondEchelon_Deterministic &solSE
             const vector<vector<double>> &deterministicDemand,
			 bool shortageAllowed = true);

    string solve();

    SolutionFirstEchelon getSolutionFE();
    SolutionSecondEchelon_Deterministic getSolutionSE();
    Result getResult();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE;
    SolutionSecondEchelon_Deterministic sol_SE;
    bool shortageAllowed;

    SolutionFirstEchelon sol_FE_temp;
    SolutionSecondEchelon_Deterministic sol_SE_temp;
    Result result;

    double THRESHOLD;
    
    bool save_lpFile;
    bool save_mpsResultFile;

    // Decision Variables
    IloNumVarArray p;
    IloNumVarArray I_plant;
    IloArray<IloNumVarArray> I_warehouse;
    IloArray<IloNumVarArray> I_customer;
    IloArray<IloNumVarArray> b_customer;
    IloArray<IloArray<IloNumVarArray>> q;
    IloArray<IloArray<IloNumVarArray>> w_customer;

    void DefineVariables(IloEnv &env, IloModel &model);
    void DefineObjectiveFunction(IloEnv &env, IloModel &model);
    void DefineConstraints(IloEnv &env, IloModel &model);
    void RetrieveSolutions(IloCplex &cplex);
    void CalculateCostsForEachPart();

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

#endif // LPSE_Deterministic_H