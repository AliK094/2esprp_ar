#ifndef MWPRP_FE_Deterministic_H
#define MWPRP_FE_Deterministic_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"

class MWPRP_FE_Deterministic
{
public:
    MWPRP_FE_Deterministic(const ParameterSetting &parameters, bool saveModel = false, bool saveSolution = false);

    bool Solve();

    SolutionFirstEchelon getSolutionFE() const
    {
        return solFE;
    }

private:
    double THRESHOLD;

    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon solFE;

    bool saveLP;
    bool saveSol;
    bool shortageAllowed = false;

    // Decision Variables
    IloNumVarArray y;
    IloNumVarArray p;
    IloNumVarArray I_plant;
    IloArray<IloNumVarArray> I_warehouse;
    IloArray<IloNumVarArray> b_warehouse;
    IloArray<IloArray<IloNumVarArray>> q;
    IloArray<IloNumVarArray> o;
    vector<vector<int>> selectedRoute;

    vector<vector<int>> routeMatrix_FirstEchelon;
    int numRoutes_FirstEchelon;
    vector<vector<int>> optimalRoutes_FirstEchelon;
    vector<double> routeCosts_FirstEchelon;
    vector<vector<double>> demand_warehouse;
    vector<vector<vector<int>>> CATW;

    vector<vector<double>> approximatedUnmetDemandPenalty_warehouse;

    vector<vector<double>> warehouseInventory;

    void configureCplex(IloCplex &cplex, IloEnv &env);
    string handleCplexStatus(IloCplex &cplex, IloEnv &env, IloModel &model);
    string generateFileName(const string &baseDir, const string &extension);
    void refineConflict(IloCplex &cplex, IloEnv &env, IloModel &model);

    void DefineVariables(IloEnv &env, IloModel &model);
    void DefineObjectiveFunction(IloEnv &env, IloModel &model);
    void DefineConstraints(IloEnv &env, IloModel &model);
    void DefCons_ProductionCapacity(IloEnv &env, IloModel &model);
    void DefCons_PlantInventoryCapacity(IloEnv &env, IloModel &model);
    void DefCons_PlantInventoryBalance(IloEnv &env, IloModel &model);
    void DefCons_WarehouseCoverage_FirstEchelon(IloEnv &env, IloModel &model);
    void DefCons_WarehouseVisit_FirstEchelon(IloEnv &env, IloModel &model);
    void DefCons_VehicleCapacity_FirstEchelon(IloEnv &env, IloModel &model);
    void DefCons_FleetSize_FirstEchelon(IloEnv &env, IloModel &model);
    void DefCons_WarehouseInventoryCapacity(IloEnv &env, IloModel &model);
    void DefCons_WarehouseInventoryBalance(IloEnv &env, IloModel &model);
    void DefCons_SatelliteInventoryBalance(IloEnv &env, IloModel &model);

    void RetrieveSolutions(IloCplex &cplex);
    void DisplayVariables();
    void CalculateCostsForEachPart();
    void DisplayProductionSetupVars();
    void DisplayProductionQuantVars();
    void DisplayPlantInventoryVars();
    void DisplayWarehouseInventoryVars();
    void DisplayFirstEchelonRouteVars();
    void DisplayDeliveryQuantityToWarehousesVars();
};

#endif // MWPRP_FE_Deterministic_H