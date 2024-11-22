#ifndef MWPRP_FE_H
#define MWPRP_FE_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"

class MWPRP_FE
{
public:
    MWPRP_FE(const ParameterSetting &parameters, bool saveModel = false, bool saveSolution = false);

    bool Solve();

    SolutionFirstEchelon getSolutionFE() const
    {
        return solFE;
    }

private:
    bool saveLP;
    bool saveSol;
    double THRESHOLD;

    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon solFE;

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
    vector<vector<vector<vector<int>>>> CATW;

    vector<vector<vector<double>>> approximatedUnmetDemandPenalty_warehouse;

    vector<vector<vector<double>>> warehouseInventory;

    void configureCplex(IloCplex &cplex, IloEnv &env);
    string handleCplexStatus(IloCplex &cplex, IloEnv &env, IloModel &model);
    string generateFileName(const string &baseDir, const string &extension);
    void refineConflict(IloCplex &cplex, IloEnv &env, IloModel &model);

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