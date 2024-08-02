#ifndef ILS_SIRP_H
#define ILS_SIRP_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class ILS_SIRP {
public:
    ILS_SIRP(const ParameterSetting &parameters, const Solution &solution);

    bool solve();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol_FE;

    vector<vector<int>> RATW;
    bool checkSolutionFeasiblity();
    double calculateObjFuncValue_ScenarioWarehouse(int s, int w, const vector<vector<double>> &Inv_Ret_Temp, const vector<vector<double>> &unmetDem_Ret_Temp, const vector<vector<vector<int>>> &route_WTR_Temp);
    double calculateObjFuncValue_SecondEchelon(const vector<vector<vector<double>>> &Inv_Retailers, const vector<vector<vector<double>>> &unmetDemand_Retailers, const vector<vector<vector<vector<vector<int>>>>> &routes_WarehouseToRetailers);

    vector<vector<vector<double>>> Inv_Retailers;
    vector<vector<vector<double>>> unmetDemand_Retailers;
    vector<vector<vector<double>>> deliveryQuantity_Retailers;
    vector<vector<vector<vector<vector<int>>>>> routes_WarehouseToRetailer;
    double objValue;
    
};

class ConstructHeuristic
{
public:
    ConstructHeuristic(const ParameterSetting &parameters, const Solution &solution);

    bool Construct_InitialSolution();

    vector<vector<vector<double>>> getInvRetailers();
    vector<vector<vector<double>>> getUnmetDemandRetailers();
    vector<vector<vector<double>>> getDeliveryQuantityRetailers();
    vector<vector<vector<vector<vector<int>>>>> getRoutesWarehouseToRetailer();
    double getBestObjValue();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol_FE;

    vector<vector<int>> RATW;
    vector<int> assignedWarehouseToRetailer;
    vector<vector<int>> sorted_Retailers_byPenaltyCostRatio;

    double bestObjValue;
    vector<vector<vector<double>>> Inv_Retailers_bestSolution;
    vector<vector<vector<double>>> unmetDemand_Retailers_bestSolution;
    vector<vector<vector<double>>> deliveryQuantity_Retailers_bestSolution;
    vector<vector<vector<vector<vector<int>>>>> routes_bestSolution;

    void orderRetailersByUnmetDemandToDeliveryRatio(vector<vector<int>> &sorted_retailer_costRatio);
    void calculateDecisionVariables(int s, int w, vector<vector<double>> &Inv_Retailers, vector<vector<double>> &unmetDemand_Retailers, vector<vector<double>> &deliveryQuantity_Retailers);
    void defineSetOne(int s, int w, int t, vector<int> &setOne, vector<vector<double>> &Inv_Retailers, const vector<vector<double>> &unmetDemand_Retailers, vector<double> &tempDeliveryQuantity);
    void defineSetTwo(int s, int w, int t, int look_ahead, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &Inv_Retailers, const vector<vector<double>> &unmetDemand_Retailers, vector<double> &tempDeliveryQuantity);
    void nearestNeighourInsertion(int s, int w, int t, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &deliveryQuantity_Retailers, vector<double> &tempDeliveryQuantity, vector<vector<int>> &routesPeriod);
    int findNextNodeToVisit(int current_node, vector<int> &setOne, vector<double> &tempDeliveryQuantity, double remainingVehicleCapacityWarehouse, double remainingAvailInventoryWarehouse);
    std::pair<int, double> minInsertionCost(const vector<int> &routesPeriod, int i);
    double calculateObjFuncValue(int s, int w, const vector<vector<double>> &Inv_Retailers, const vector<vector<double>> &unmetDemand_Retailers, const vector<vector<vector<int>>> &routesPeriod);
};

class LocalSearch
{
public:
    LocalSearch(const ParameterSetting &parameters, const Solution &solution);
    
    void RVND(int s, int w, 
		vector<vector<double>> &Inv_Ret_ScenWare, 
		vector<vector<double>> &unmetDemand_Ret_ScenWare,
		vector<vector<double>> &delQuant_Ret_ScenWare,
		vector<vector<vector<int>>> &routes_WareToRet_ScenWare,
        double &objValue_ScenarioWarehouse);

    double getBestObjValue();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol_FE;

    vector<vector<int>> RATW;

    vector<std::function<bool(int, vector<vector<vector<int>>> &)>> setOperators();
    bool OrOpt_One(int w, vector<vector<vector<int>>> &routes);
    bool OrOpt_Two(int w, vector<vector<vector<int>>> &routes);
    bool OrOpt_Three(int w, vector<vector<vector<int>>> &routes);
    bool Shift_One(int w, vector<vector<vector<int>>> &routes);
    bool Shift_Two(int w, vector<vector<vector<int>>> &routes);
    bool Shift_Three(int w, vector<vector<vector<int>>> &routes);
    bool Swap(int w, vector<vector<vector<int>>> &routes);
    bool Insert(int w, vector<vector<vector<int>>> &routes);
    bool Remove(int w, vector<vector<vector<int>>> &routes);
    bool Merge(int w, vector<vector<vector<int>>>& routes);
    bool Transfer(int w, vector<vector<vector<int>>>& routes);
    bool Remove_Insert(int w, vector<vector<vector<int>>>& routes);
    double calculateObjFuncValue_ScenarioWarehouse(int s, int w, const vector<vector<double>> &Inv_Ret_Temp, const vector<vector<double>> &unmetDem_Ret_Temp, const vector<vector<vector<int>>> &route_WTR_Temp);
};

class Perturbation
{
public:
    Perturbation(const ParameterSetting &parameters, const Solution &solution);
    
    bool run(int s, int w, 
		vector<vector<double>> &Inv_Ret_ScenWare, 
		vector<vector<double>> &unmetDemand_Ret_ScenWare,
		vector<vector<double>> &delQuant_Ret_ScenWare,
		vector<vector<vector<int>>> &routes_WareToRet_ScenWare);

    double getBestObjValue();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol_FE;

    vector<vector<int>> RATW;

    vector<vector<vector<double>>> Inv_Retailers_bestSolution;
    vector<vector<vector<double>>> unmetDemand_Retailers_bestSolution;
    vector<vector<vector<double>>> deliveryQuantity_Retailers_bestSolution;
    vector<vector<vector<vector<vector<int>>>>> routes_bestSolution;
    double bestObjValue;

    vector<std::function<bool(int, vector<vector<vector<int>>> &)>> setPerturbOperators();
    bool randomShift(int w, vector<vector<vector<int>>> &routes);
    bool randomInsertion(int w, vector<vector<vector<int>>> &routes);
    bool randomRemoval(int w, vector<vector<vector<int>>> &routes);
    double calculateObjFuncValue_ScenarioWarehouse(int s, int w, const vector<vector<double>> &Inv_Ret_Temp, const vector<vector<double>> &unmetDem_Ret_Temp, const vector<vector<vector<int>>> &route_WTR_Temp);
};

class LP_ScenarioWarehouse
{
public:
    LP_ScenarioWarehouse(const ParameterSetting &parameters, const Solution &solution,
                int scenario,
                int warehouse,
                vector<vector<vector<int>>> routes_WareToRet_ScenWare);
    string solve();
    
    vector<vector<double>> getInvRetailers();
    vector<vector<double>> getUnmetDemandRetailers();
    vector<vector<double>> getDeliveryQuantityRetailers();
private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol_FE;

    int scenario;
    int warehouse;
    int numRetailers_w;

    vector<vector<int>> RATW;

    vector<vector<double>> ret_Inv;
    vector<vector<double>> ret_unmDem;
    vector<vector<double>> ret_delQuant;
    vector<vector<vector<int>>> routes_WareToRet_ScenWare;
};

#endif // ILS_SIRP_H