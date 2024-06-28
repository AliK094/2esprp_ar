#ifndef MWPRP_FE_H
#define MWPRP_FE_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"

class MWPRP_FE
{
public:
    MWPRP_FE(const ParameterSetting &parameters,
        const vector<vector<vector<double>>>& warehouseInventory_Previous, 
        const vector<vector<vector<double>>>& dualValues_WarehouseInventoryLB);

    bool Solve();

    Solution getSolution() const
    {
        return sol;
    }

    Result getResult() const
    {
        return result;
    }

private:
    double THRESHOLD;

    ParameterSetting params; // Member variable to hold the ParameterSetting object

    Result result;
    Solution sol;

    // Decision Variables
    IloNumVarArray y;
    IloNumVarArray p;
    IloNumVarArray I_plant;
    IloArray<IloArray<IloNumVarArray>> I_warehouse;
    IloArray<IloArray<IloNumVarArray>> b_warehouse;
    IloArray<IloArray<IloNumVarArray>> q;
    IloArray<IloNumVarArray> o;
    vector<vector<int>> selectedRoute;

    vector<vector<int>> routeMatrix_FirstEchelon;
    int numRoutes_FirstEchelon;
    vector<vector<int>> optimalRoutes_FirstEchelon;
    vector<double> routeCosts_FirstEchelon;
    vector<vector<vector<double>>> demand_warehouse;

    vector<vector<vector<double>>> warehouseInventory_PreviousIter;
    vector<vector<vector<double>>> dualValues_WarehouseInventoryLB;

    vector<double> approximatedUnmetDemandPenalty_warehouse;

    bool save_lpFile;
    bool save_mpsResultFile;

    void DefineVariables(IloEnv &env, IloModel &model);
    void DefineObjectiveFunction(IloEnv &env, IloModel &model);
    void DefineConstraints(IloEnv &env, IloModel &model);
    void RetrieveSolutions(IloCplex &cplex);
    // void DefineWarmStartSolution(IloEnv &env, IloCplex &cplex, const Solution &warmStart);
    void DisplayVariables();
    void CalculateCostsForEachPart();

    void DisplayProductionSetupVars();
    void DisplayProductionQuantVars();
    void DisplayPlantInventoryVars();
    void DisplayWarehouseInventoryVars();
    void DisplayFirstEchelonRouteVars();
    void DisplayDeliveryQuantityToWarehousesVars();
};

#endif // MWPRP_FE_H