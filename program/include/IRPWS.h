#ifndef IRPWS_H
#define IRPWS_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"

class IRPWS
{
public:
    IRPWS(const ParameterSetting &parameters, const Solution &solution, int w, int s);

    bool Solve();

    vector<double> getDualValues_WarehouseInventoryLB();

    vector<vector<double>> getRetailerInventory_S();
    vector<vector<double>> getUnmetDemandQuantity_S();
    vector<vector<double>> getDeliveryQuantityToRetailers_W_S();


    // Solution getSolution() const
    // {
    //     return sol;
    // }

    // Result getResult() const
    // {
    //     return result;
    // }

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol_FE;

    int warehouse;
    int scenario;

    vector<double> unmetDemandPenalty;

    double THRESHOLD;

    // Result result;
    // Solution sol;

    // Decision Variables
    IloArray<IloNumVarArray> I;
    IloArray<IloNumVarArray> b;
    IloArray<IloNumVarArray> w;
    IloArray<IloNumVarArray> z;
    IloArray<IloArray<IloNumVarArray>> x;

    vector<IloRange> deliveryConstraints; // Vector to store constraints

    vector<vector<int>> RATW;
    vector<int> numRetailer_w;
    vector<vector<vector<double>>> warehouseInventory;
    int numRoutes_FirstEchelon;
    vector<vector<double>> DeliveredToWarehouse;
    vector<double> dualValues_WarehouseInventoryLB;

    vector<vector<double>> retailerInventory_S;
    vector<vector<double>> unmetDemandQuantity_S;
    vector<vector<double>> deliveryQuantityToRetailers_W_S;

    bool save_lpFile;
    bool save_mpsResultFile;

    void DefineVariables(IloEnv &env, IloModel &model);
    void DefineObjectiveFunction(IloEnv &env, IloModel &model);
    void DefineConstraints(IloEnv &env, IloModel &model);
    void RetrieveSolutions(IloCplex &cplex);
    
    // void RetrieveSolutions(IloCplex &cplex, int warehouse, int scenario);
    // void DefineWarmStartSolution(IloEnv &env, IloCplex &cplex, const Solution &warmStart);
    // void DisplayVariables();
    // void CalculateCostsForEachPart();

    // void DisplayProductionSetupVars();
    // void DisplayProductionQuantVars();
    // void DisplayPlantInventoryVars();
    // void DisplayWarehouseInventoryVars();
    // void DisplayFirstEchelonRouteVars();
    // void DisplayDeliveryQuantityToWarehousesVars();
};

#endif // IRPWS_H