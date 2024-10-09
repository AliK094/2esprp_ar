#ifndef ILS_SIRP_H
#define ILS_SIRP_H

#include "ParameterSetting.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"

class ConstructHeuristic
{
public:
    ConstructHeuristic(const ParameterSetting &parameters, const Solution &solution);

    bool Construct_InitialSolution();






    bool Solve();

    // vector<double> getDualValues_WarehouseInventoryLB();

    // vector<vector<double>> getRetailerInventory_S();
    // vector<vector<double>> getUnmetDemandQuantity_S();
    // vector<vector<double>> getDeliveryQuantityToRetailers_W_S();


    // Solution getSolution() const
    // {
    //     return sol;
    // }

    // Result getResult() const
    // {
    //     return result;
    // }

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object
    Solution sol_FE;

    vector<vector<int>> RATW;
    vector<int> assignedWarehouseToRetailer;
    vector<vector<int>> sorted_customers_byPenaltyCostRatio;

    // vector<vector<vector<double>>> Inv_Customers;
    // vector<vector<vector<double>>> unmetDemand_Customers;
    // vector<vector<vector<double>>> deliveryQuantity_Customers;
    // vector<vector<vector<vector<vector<int>>>>> routes;

    void orderCustomersByUnmetDemandToDeliveryRatio(vector<vector<int>>& sorted_customer_costRatio);
    void calculateDecisionVariables(int s, int w, vector<vector<double>>& Inv_Customers, vector<vector<double>>& unmetDemand_Customers, vector<vector<double>>& deliveryQuantity_Customers);
    void defineSetOne(int s, int w, int t, vector<int>& setOne, const vector<vector<double>>& unmetDemand_Customers, vector<double>& tempDeliveryQuantity);
    void defineSetTwo(int s, int w, int t, int look_ahead, vector<int>& &setOne, vector<int>& setTwo, const vector<vector<double>>& unmetDemand_Customers, vector<double>& tempDeliveryQuantity);
    void nearestNeighourInsertion(int s, int w, int t, vector<int>& setOne, vector<int>& setTwo, vector<vector<double>>&  tempDeliveryQuantity, vector<vector<vector<int>>>& routesWarehousePeriod);
    int findNextNodeToVisit(int current_node, vector<int>& setOne, vector<double> &tempDeliveryQuantity)
    std::pair<int, double> minInsertionCost(const std::vector<int>& routesWarehousePeriod, int i)
{

};

#endif // ILS_SIRP_H