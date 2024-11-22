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
    LP_SE_Deterministic(const ParameterSetting &parameters, const SolutionFirstEchelon &solFE, const SolutionSecondEchelon_Deterministic &solSE,
                        bool saveModel = false, bool saveSolution = false);

    string solve();

    SolutionFirstEchelon getSolutionFE();
    SolutionSecondEchelon_Deterministic getSolutionSE();
    Result getResult();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon sol_FE;
    SolutionSecondEchelon_Deterministic sol_SE;
    bool shortageAllowed = false;
    double THRESHOLD;
    bool saveLP;
    bool saveSol;

    SolutionFirstEchelon sol_FE_temp;
    SolutionSecondEchelon_Deterministic sol_SE_temp;
    Result result;

    vector<vector<vector<double>>> warehouse_delivery;

    // Decision Variables
    IloNumVarArray p;
    IloNumVarArray I_plant;
    IloArray<IloNumVarArray> I_warehouse;
    IloArray<IloNumVarArray> I_customer;
    IloArray<IloNumVarArray> b_customer;
    IloArray<IloArray<IloNumVarArray>> q;
    IloArray<IloArray<IloNumVarArray>> w_customer;

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
    void DefCons_WarehouseVisit_FirstEchelon(IloEnv &env, IloModel &model);
    void DefCons_VehicleCapacity_FirstEchelon(IloEnv &env, IloModel &model);

    void DefCons_WarehouseInventoryCapacity(IloEnv &env, IloModel &model);
    void DefCons_WarehouseInventoryBalance(IloEnv &env, IloModel &model);
    void DefCons_SatelliteInventoryBalance(IloEnv &env, IloModel &model);
    void DefCons_WarehouseInventoryBalance_EEV(IloEnv &env, IloModel &model);
    
    void DefCons_CustomerInventoryCapacity(IloEnv &env, IloModel &model);
    void DefCons_CustomerInventoryBalance(IloEnv &env, IloModel &model);
    void DefCons_CustomerVisit_SecondEchelon(IloEnv &env, IloModel &model);
    void DefCons_VehicleCapacity_SecondEchelon(IloEnv &env, IloModel &model);

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

#endif // LPSE_Deterministic_H