#ifndef LPSE_SCEN_H
#define LPSE_SCEN_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class LP_SE_Scenario
{
public:
    LP_SE_Scenario(const ParameterSetting &parameters, const SolutionFirstEchelon &sol_FE, const ScenarioSolutionSecondEchelon &sol_SE_scenario, int s);

    string solve();

    ScenarioSolutionSecondEchelon getSolutionSE_Scenario();
    double getObjVal_Scenario();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE;
    ScenarioSolutionSecondEchelon sol_SE_scenario;

    SolutionFirstEchelon sol_FE_scenario_temp;
    ScenarioSolutionSecondEchelon sol_SE_scenario_temp;

    int scenario;

    double THRESHOLD;
    
    bool save_lpFile;
    bool save_mpsResultFile;

    double objVal_Scenario;

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
    void CalculateObjValue_Scenario();

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

#endif // LPSE_SCEN_H