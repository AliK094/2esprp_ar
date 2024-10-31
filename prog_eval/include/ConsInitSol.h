#ifndef CONSINITSOL_H
#define CONSINITSOL_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class ConstructHeuristic
{
public:
    ConstructHeuristic(const ParameterSetting &parameters, const SolutionFirstEchelon &sol_FE);

    bool Construct_InitialSolution();

    vector<vector<vector<double>>> getInvCustomers();
    vector<vector<vector<double>>> getUnmetDemandCustomers();
    vector<vector<vector<double>>> getDeliveryQuantityCustomers();
    vector<vector<vector<vector<vector<int>>>>> getRoutesWarehouseToCustomer();

    double getBestObjValue();


    // vector<vector<vector<vector<int>>>> getCustomerAssignmentToWarehouse();

    

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon solFE_init;

    vector<vector<int>> sortedWarehouseByDistance;
    vector<int> sorted_Customers_byPenaltyCostRatio;
    vector<vector<vector<vector<int>>>> CATW;

    double bestObjValue;
    vector<vector<vector<double>>> Inv_Customers_bestSolution;
    vector<vector<vector<double>>> unmetDemand_Customers_bestSolution;
    vector<vector<vector<double>>> deliveryQuantity_Customers_bestSolution;
    vector<vector<vector<vector<vector<int>>>>> routes_bestSolution;
    vector<vector<vector<vector<int>>>> customerAssignmentToWarehouse_bestSolution;
    
    void orderCustomersByUnmetDemandToDeliveryRatio(vector<int> &sorted_customer_costRatio);
    void calculateDecisionVariables(int s, int w, vector<vector<double>> &Inv_Customers, vector<vector<double>> &unmetDemand_Customers, vector<vector<double>> &deliveryQuantity_Customers);
    void defineSetOne(int s, int w, int t, vector<int> &setOne, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity);
    void defineSetTwo(int s, int w, int t, int look_ahead, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity);
    void nearestNeighourInsertion(int s, int w, int t, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &deliveryQuantity_Customers, vector<double> &tempDeliveryQuantity, vector<vector<int>> &routesPeriod);
    int findNextNodeToVisit(int current_node, vector<int> &setOne, vector<double> &tempDeliveryQuantity, double remainingVehicleCapacityWarehouse, double remainingAvailInventoryWarehouse);
    std::pair<int, double> minInsertionCost(const vector<int> &routes, int i);
    double calculateObjFuncValue(int s, int w, const vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, const vector<vector<vector<int>>> &routesPeriod);
    
    
    /*
    // vector<vector<vector<vector<int>>>> CATW;
    vector<vector<int>> sortedWarehouseByDistance;
    // vector<vector<vector<int>>> assignedWarehouseToCustomer;
    // vector<vector<vector<vector<int>>>> sorted_Customers_byPenaltyCostRatio;
    vector<int> sorted_Customers_byPenaltyCostRatio;

    double bestObjValue;
    vector<vector<vector<double>>> Inv_Customers_bestSolution;
    vector<vector<vector<double>>> unmetDemand_Customers_bestSolution;
    vector<vector<vector<double>>> deliveryQuantity_Customers_bestSolution;
    vector<vector<vector<vector<vector<int>>>>> routes_bestSolution;
    vector<vector<vector<vector<int>>>> customerAssignmentToWarehouse_bestSolution;

    void orderCustomersByUnmetDemandToDeliveryRatio(vector<int> &sorted_customer_costRatio);
    void calcDecVarValues_Scenario(int s, vector<vector<double>> &Inv_Customers, vector<vector<double>> &unmetDemand_Customers, vector<vector<double>> &deliveryQuantity_Customers);
    void defineSetOne(int s, int t, vector<int> &setOne, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity);
    void defineSetTwo(int s, int t, int look_ahead, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity);
    void nearestNeighourInsertion(int s, int t, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &deliveryQuantity_Customers, vector<double> &tempDeliveryQuantity, vector<vector<vector<vector<int>>>> &routes, vector<vector<vector<int>>> &tempCATW);
    int findNextNodeToVisit(int current_node, vector<int> &setOne, vector<double> &tempDeliveryQuantity, double remainingVehicleCapacityWarehouse, double remainingAvailInventoryWarehouse);
    std::pair<int, double> minInsertionCost(const vector<int> &routes, int i);
    double calculateObjFuncValue(int s, const vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, const vector<vector<vector<vector<int>>>> &routes);
    */
};

#endif // CONSINITSOL_H