#ifndef R2EPRP_H
#define R2EPRP_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "deterministic/DeterBCCallbackManager.h"

class R2EPRP
{
public:
    R2EPRP(const ParameterSetting &parameters, 
            const vector<vector<double>> &deterministicDemand,
            bool shortageAllowed = true,
            const SolutionWarmStart_Deterministic &warmStart = {});

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
    ParameterSetting params; // Member variable to hold the ParameterSetting object

    SolutionFirstEchelon solFE;
    SolutionSecondEchelon_Deterministic solSE;
    vector<vector<double>> demand;
    bool shortageAllowed;
    SolutionWarmStart_Deterministic warmStart;
    Result result;

    double THRESHOLD;
    
    bool save_lpFile;
    bool save_mpsResultFile;

    vector<vector<int>> routeMatrix_FirstEchelon;
    int numRoutes_FirstEchelon;
    vector<vector<int>> optimalRoutes_FirstEchelon;
    vector<double> routeCosts_FirstEchelon;


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

    vector<vector<int>> selectedRoute;

    void DefineVariables(IloEnv &env, IloModel &model);
    void DefineObjectiveFunction(IloEnv &env, IloModel &model);
    void DefineConstraints(IloEnv &env, IloModel &model);
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

#endif // R2EPRP_H