#ifndef LPSE_H
#define LPSE_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class LP_SE
{
public:
    LP_SE(const ParameterSetting &parameters, const SolutionFirstEchelon &solFE, const SolutionSecondEchelon &solSE,
            bool saveModel = false, bool saveSolution = false);

    string solve();

    SolutionFirstEchelon getSolutionFE();
    SolutionSecondEchelon getSolutionSE();
    Result getResult();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    double THRESHOLD;
    bool saveLP;
    bool saveSol;

    SolutionFirstEchelon sol_FE;
    SolutionSecondEchelon sol_SE;

    SolutionFirstEchelon sol_FE_temp;
    SolutionSecondEchelon sol_SE_temp;
    Result result;

    // Decision Variables
    IloNumVarArray p;
    IloNumVarArray I_plant;
    IloArray<IloArray<IloNumVarArray>> I_warehouse;
    IloArray<IloArray<IloNumVarArray>> I_customer;
    IloArray<IloArray<IloNumVarArray>> b_customer;
    IloArray<IloArray<IloNumVarArray>> q;
    IloArray<IloArray<IloArray<IloNumVarArray>>> w_customer;

    void configureCplex(IloCplex &cplex, IloEnv &env);
    void handleCplexStatus(IloCplex &cplex, IloEnv &env, IloModel &model);
    string generateFileName(const string &baseDir, const string &extension);
    void refineConflict(IloCplex &cplex, IloEnv &env, IloModel &model);

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

#endif // LPSE_H