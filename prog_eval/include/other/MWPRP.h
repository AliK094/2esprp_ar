#ifndef MWPRP_H
#define MWPRP_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"

class MWPRP
{
public:
    MWPRP(const ParameterSetting &parameters);

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
    ParameterSetting params; // Member variable to hold the ParameterSetting object

    std::string solutionAlgorithm;
    std::string instance;
    int numWarehouses;
    int numRetailers;
    int numPeriods;
    int numVehiclesPlant;
    int numVehiclesWarehouse;
    int numScenarios;
    double unitProdCost;
    double setupCost;
    double prodCapacity;
    double vehicleCapacityPlant;
    double vehicleCapacityWarehouse;
    std::vector<double> probability;

    double storageCapacityPlant;
    std::vector<double> storageCapacityWarehouse;
    std::vector<double> storageCapacityRetailer;

    double unitHoldingCostPlant;
    std::vector<double> unitHoldingCostWarehouse;
    std::vector<double> unitHoldingCostRetailer;

    double initialInventoryPlant;
    std::vector<double> initialInventoryWarehouse;
    std::vector<double> initialInventoryRetailer;

    std::vector<std::vector<std::vector<double>>> demand;
    std::vector<double> unmetDemandPenalty;

    double THRESHOLD;

    Result result;
    Solution sol;

    // Decision Variables
    IloNumVarArray y;
    IloNumVarArray p;
    IloNumVarArray I_plant;
    IloArray<IloArray<IloNumVarArray>> I_warehouse;
    IloArray<IloArray<IloNumVarArray>> I_retailer;
    IloArray<IloArray<IloNumVarArray>> q;
    IloArray<IloArray<IloArray<IloNumVarArray>>> e;
    IloArray<IloNumVarArray> o;
    IloArray<IloArray<IloNumVarArray>> b_retailer;
    std::vector<std::vector<int>> selectedRoute;

    std::vector<std::vector<int>> routeMatrix_FirstEchelon;
    int numRoutes_FirstEchelon;
    std::vector<std::vector<int>> optimalRoutes_FirstEchelon;
    std::vector<double> routeCosts_FirstEchelon;

    std::vector<std::vector<std::vector<std::vector<int>>>> routeMatrix_SecondEchelon;

    bool save_lpFile;
    bool save_mpsResultFile;

    void DefineVariables(IloEnv &env, IloModel &model);
    void DefineObjectiveFunction(IloEnv &env, IloModel &model);
    void DefineConstraints(IloEnv &env, IloModel &model);
    void RetrieveSolutions(IloCplex &cplex);
    void DefineWarmStartSolution(IloEnv &env, IloCplex &cplex, const Solution &warmStart);
    void DisplayVariables();
    void CalculateCostsForEachPart();

    template <typename T>
    void PrintVariable(const std::string &varName, const T &data, double threshold = 1e-6, std::vector<int> indices = {}, int depth = 0);

    void DisplayProductionSetupVars();
    void DisplayProductionQuantVars();
    void DisplayPlantInventoryVars();
    void DisplayWarehouseInventoryVars();
    void DisplayRetailerInventoryVars();
    void DisplayRetailerUnmetDemandVars();
    void DisplayFirstEchelonRouteVars();
    void DisplayDeliveryQuantityToWarehousesVars();
    void DisplayDeliveryQuantityToRetailersVars();
};

#endif // MWPRP_H