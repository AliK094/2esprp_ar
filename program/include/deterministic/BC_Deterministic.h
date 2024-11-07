#ifndef BC_DETERMINISTIC_H
#define BC_DETERMINISTIC_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "deterministic/DeterBCCallbackManager.h"

class BC_Deterministic
{
public:
    BC_Deterministic(const ParameterSetting &parameters, 
                    const vector<vector<double>> &deterministicDemand, 
                    const SolutionWarmStart_Deterministic &warmStartSol = {}, 
                    bool shortageAllowed = true);

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

    vector<vector<double>> demand;
    SolutionWarmStart_Deterministic warmStart;
    bool shortageAllowed;

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

#endif // BC_DETERMINISTIC_H