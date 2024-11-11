#ifndef BC_EEV_H
#define BC_EEV_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "deterministic/DeterBCCallbackManager.h"

class BC_EEV
{
public:
    BC_EEV(const ParameterSetting &parameters,
            const SolutionFirstEchelon &solFE,
            const vector<vector<double>> &deterministicDemand, 
            const SolutionWarmStart_Deterministic &warmStartSol = {});

    bool Solve();

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

    SolutionFirstEchelon sol_FE_EV;
    SolutionSecondEchelon_Deterministic solSE;
    Result result;

    // Decision Variables
    IloArray<IloNumVarArray> I_warehouse;
    IloArray<IloNumVarArray> I_customer;
    IloArray<IloNumVarArray> b_customer;
    IloArray<IloArray<IloNumVarArray>> w_customer;
    IloArray<IloArray<IloNumVarArray>> z;
    IloArray<IloArray<IloNumVarArray>> x;

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

#endif // BC_EEV_H