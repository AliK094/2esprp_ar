#ifndef CONSINITSOL_Deterministic_H
#define CONSINITSOL_Deterministic_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include <functional>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <tuple>

class ConstructHeuristic_Deterministic
{
public:
    ConstructHeuristic_Deterministic(const ParameterSetting &parameters, 
                const SolutionFirstEchelon &sol_FE, 
                const vector<vector<double>> &deterministicDemand,
                vector<vector<vector<int>>> &CATW,
			    bool shortageAllowed = true);

    bool Construct_InitialSolution();

    vector<vector<double>> getInvCustomers();
    vector<vector<double>> getUnmetDemandCustomers();
    vector<vector<double>> getDeliveryQuantityCustomers();
    vector<vector<vector<vector<int>>>> getRoutesWarehouseToCustomer();

    double getBestObjValue();


    // vector<vector<vector<vector<int>>>> getCustomerAssignmentToWarehouse();

    

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    SolutionFirstEchelon solFE_init;
    bool shortageAllowed;
    vector<vector<double>> demand;

    vector<vector<int>> sortedWarehouseByDistance;
    vector<int> sorted_Customers_byPenaltyCostRatio;
    vector<vector<vector<int>>> CATW;

    double bestObjValue;
    vector<vector<double>> Inv_Customers_bestSolution;
    vector<vector<double>> unmetDemand_Customers_bestSolution;
    vector<vector<double>> deliveryQuantity_Customers_bestSolution;
    vector<vector<vector<vector<int>>>> routes_bestSolution;
    
    void orderCustomersByUnmetDemandToDeliveryRatio(vector<int> &sorted_customer_costRatio);
    void calculateDecisionVariables(int w, vector<vector<double>> &Inv_Customers, vector<vector<double>> &unmetDemand_Customers, vector<vector<double>> &deliveryQuantity_Customers);
    void defineSetOne(int w, int t, vector<int> &setOne, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity);
    void defineSetTwo(int w, int t, int look_ahead, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity);
    void nearestNeighourInsertion(int w, int t, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &deliveryQuantity_Customers, vector<double> &tempDeliveryQuantity, vector<vector<int>> &routesPeriod);
    int findNextNodeToVisit(int current_node, vector<int> &setOne, vector<double> &tempDeliveryQuantity, double remainingVehicleCapacityWarehouse, double remainingAvailInventoryWarehouse);
    std::pair<int, double> minInsertionCost(const vector<int> &routes, int i);
    double calculateObjFuncValue(int w, const vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, const vector<vector<vector<int>>> &routesPeriod);
};

#endif // CONSINITSOL_Deterministic_H