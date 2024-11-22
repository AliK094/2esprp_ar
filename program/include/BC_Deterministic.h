#ifndef BC_DETERMINISTIC_H
#define BC_DETERMINISTIC_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "DeterBCCallbackManager.h"

class BC_Deterministic
{
public:
    BC_Deterministic(const ParameterSetting &parameters, 
                    const SolutionWarmStart_Deterministic &warmStartSol = {}, 
                    const SolutionFirstEchelon &sol_FE = {},
                    bool saveModel = false,
                    bool saveSolution = false);

    bool Solve();

    SolutionFirstEchelon getSolutionFE() const
    {
        return solFE;
    }

    SolutionSecondEchelon_Deterministic getSolutionSE() const
    {
        return solSE;
    }

    Result getResult() const
    {
        return result;
    }

private:
    double THRESHOLD;

    ParameterSetting params; // Member variable to hold the ParameterSetting object
    bool shortageAllowed = false;
    bool saveLP;
    bool saveSol;

    vector<vector<double>> demand;
    SolutionWarmStart_Deterministic warmStart;

    SolutionFirstEchelon solFE;
    SolutionSecondEchelon_Deterministic solSE;
    Result result;

    // Decision Variables
    IloNumVarArray y;
    IloNumVarArray p;
    IloNumVarArray I_plant;
    IloArray<IloNumVarArray> I_warehouse;
    IloArray<IloNumVarArray> I_customer;
    IloArray<IloNumVarArray> b_customer;
    IloArray<IloArray<IloNumVarArray>> q;
    IloArray<IloNumVarArray> o;
    IloArray<IloArray<IloNumVarArray>> w_customer;
    IloArray<IloArray<IloNumVarArray>> z;
    IloArray<IloArray<IloNumVarArray>> x;

    vector<vector<int>> selectedRoute;

    vector<vector<int>> routeMatrix_FirstEchelon;
    int numRoutes_FirstEchelon;
    vector<vector<int>> optimalRoutes_FirstEchelon;
    vector<double> routeCosts_FirstEchelon;

    SolutionFirstEchelon sol_FE_EV;
    vector<vector<vector<double>>> warehouse_delivery;

    void configureCplex(IloCplex &cplex, IloEnv &env);
    void handleCplexStatus(IloCplex &cplex, IloEnv &env, IloModel &model);
    string generateFileName(const string &baseDir, const string &extension);
    void refineConflict(IloCplex &cplex, IloEnv &env, IloModel &model);

    void DefineVariables(IloEnv &env, IloModel &model);
    void DefineFirstStageVars(IloEnv &env, IloModel &model);
    void DefineSecondStageVars(IloEnv &env, IloModel &model);

    void DefineObjectiveFunction(IloEnv &env, IloModel &model);

    void DefineConstraints(IloEnv &env, IloModel &model);
    void DefCons_ProductionCapacity(IloEnv &env, IloModel &model);
    void DefCons_PlantInventoryCapacity(IloEnv &env, IloModel &model);
    void DefCons_PlantInventoryBalance(IloEnv &env, IloModel &model);
    void DefCons_WarehouseCoverage_FirstEchelon(IloEnv &env, IloModel &model);
    void DefCons_VehicleCapacity_FirstEchelon(IloEnv &env, IloModel &model);
    void DefCons_FleetSize_FirstEchelon(IloEnv &env, IloModel &model);
    void DefCons_WarehouseInventoryCapacity(IloEnv &env, IloModel &model);
    void DefCons_SatelliteInventoryBalance(IloEnv &env, IloModel &model);
    void DefCons_WarehouseInventoryBalance(IloEnv &env, IloModel &model);
    void DefCons_WarehouseInventoryBalance_EEV(IloEnv &env, IloModel &model);
    void DefCons_CustomerInventoryCapacity(IloEnv &env, IloModel &model);
    void DefCons_CustomerInventoryBalance(IloEnv &env, IloModel &model);
    void DefCons_CustomerVisit_SecondEchelon(IloEnv &env, IloModel &model);
    void DefCons_VehicleCapacity_SecondEchelon(IloEnv &env, IloModel &model);
    void DefCons_SplitDeliveries_SecondEchelon(IloEnv &env, IloModel &model);
    void DefCons_Degree_SecondEchelon(IloEnv &env, IloModel &model);

    void DefCons_WarehouseVisit_FirstEchelon(IloEnv &env, IloModel &model);

    void DefCons_ValidInequalities_LogicalOne(IloEnv &env, IloModel &model);
    void DefCons_ValidInequalities_LogicalTwo(IloEnv &env, IloModel &model);
    void DefCons_ValidInequalities_SymmetryBreakingOne(IloEnv &env, IloModel &model);
    void DefCons_ValidInequalities_Lexicographic(IloEnv &env, IloModel &model);

    void RetrieveSolutions(IloCplex &cplex);
    void DefineWarmStartSolution(IloEnv &env, IloCplex &cplex);
    void DisplayVariables();
    void CalculateCostsForEachPart();
    void checkSplitDeliveries(vector<vector<vector<int>>>& visitedNodes_SecondEchelon);
    void adjustRoutesSE(vector<vector<vector<int>>>& visitedNodes_SecondEchelon, vector<vector<vector<int>>>& visitedEdges_SecondEchelon);

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

#endif // BC_DETERMINISTIC_H