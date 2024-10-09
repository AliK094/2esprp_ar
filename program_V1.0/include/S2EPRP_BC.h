#ifndef S2EPRP_BC_H
#define S2EPRP_BC_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "BCCallbackManager.h"

class S2EPRP_BC
{
public:
    S2EPRP_BC(const ParameterSetting &parameters, const Solution &warmStart = {});

    bool Solve();

    Solution getSolution() const
    {
        return sol;
    }

private:
    double THRESHOLD;

    ParameterSetting params; // Member variable to hold the ParameterSetting object

    Solution sol;
    Solution warmStart;

    // Decision Variables
    IloNumVarArray y;
    IloNumVarArray p;
    IloNumVarArray I_plant;
    IloArray<IloArray<IloNumVarArray>> I_warehouse;
    IloArray<IloArray<IloNumVarArray>> I_customer;
    IloArray<IloArray<IloNumVarArray>> b_customer;
    IloArray<IloArray<IloNumVarArray>> q;
    IloArray<IloNumVarArray> o;
    IloArray<IloArray<IloArray<IloNumVarArray>>> w_customer;
    IloArray<IloArray<IloArray<IloNumVarArray>>> z;
    IloArray<IloArray<IloArray<IloNumVarArray>>> x;

    vector<vector<int>> selectedRoute;

    vector<vector<int>> routeMatrix_FirstEchelon;
    int numRoutes_FirstEchelon;
    vector<vector<int>> optimalRoutes_FirstEchelon;
    vector<double> routeCosts_FirstEchelon;

    bool save_lpFile;
    bool save_mpsResultFile;

    void DefineVariables(IloEnv &env, IloModel &model);
    void DefineObjectiveFunction(IloEnv &env, IloModel &model);
    void DefineConstraints(IloEnv &env, IloModel &model);
    void DefineValidInequalities(IloEnv &env, IloModel &model);
    void RetrieveSolutions(IloCplex &cplex);
    void DefineWarmStartSolution(IloEnv &env, IloCplex &cplex);
    void DisplayVariables();
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

#endif // S2EPRP_BC_H