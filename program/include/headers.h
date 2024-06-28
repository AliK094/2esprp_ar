#ifndef HEADERS_H
#define HEADERS_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>
#include "ilcplex/ilocplex.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::cerr;

// Define a constant for unknown values
#define NONE -1

// Structure representing a cut set
struct CUTSET
{
    int dim; // Size of the cut
    int *S;  // Set of vertices in the cut

    CUTSET() : dim(0), S(nullptr) {} // Default constructor
    ~CUTSET() { delete[] S; }        // Destructor to free allocated memory
};

// Structure representing the result of an algorithm execution
struct Result
{
    bool success = true;        // Indicates if the algorithm succeeded
    string status;         // Status message
    double objValue = 0.0;      // Objective value
    double lowerBound = 0.0;    // Lower bound of the objective
    double optimalityGap = 0.0; // Optimality gap
    double CPUtime = 0.0;       // CPU time used
};

// Structure representing a solution
struct Solution
{
    vector<int> productionSetup;
    vector<double> productionQuantity;
    vector<double> plantInventory;
    vector<vector<vector<double>>> warehouseInventory;
    vector<vector<vector<double>>> retailerInventory;
    vector<vector<vector<double>>> retailerUnmetDemand;
    vector<vector<vector<double>>> deliveryQuantityToWarehouse;
    vector<vector<vector<vector<double>>>> deliveryQuantityToRetailer;
    vector<vector<vector<int>>> routePlantToWarehouse;
    vector<vector<vector<vector<vector<int>>>>> routeWarehouseToRetailer;

    double setupCost = 0.0;                                 // Total setup cost
    double productionCost = 0.0;                            // Total production cost
    double holdingCostPlant = 0.0;                          // Total holding cost at the plant
    double transportationCostPlantToWarehouse = 0.0;        // Total transportation cost from the plant to warehouses
    double holdingCostWarehouse_Avg = 0.0;                  // Average total holding cost at the warehouse
    double holdingCostRetailer_Avg = 0.0;                   // Average total holding cost at the retailer
    double transportationCostWarehouseToRetailer_Avg = 0.0; // Average total transportation cost from warehouses to retailers
    double costOfUnmetDemand_Avg = 0.0;                     // Average total cost of unmet demand

    // Check if the s        olution is empty
    bool empty() const
    {
        return productionSetup.empty() &&
               productionQuantity.empty() &&
               plantInventory.empty() &&
               warehouseInventory.empty() &&
               retailerInventory.empty() &&
               retailerUnmetDemand.empty() &&
               deliveryQuantityToWarehouse.empty() &&
               deliveryQuantityToRetailer.empty() &&
               routePlantToWarehouse.empty() &&
               routeWarehouseToRetailer.empty() &&
               (setupCost == 0.0) &&
               (productionCost == 0.0) &&
               (holdingCostPlant == 0.0) &&
               (transportationCostPlantToWarehouse == 0.0) &&
               (holdingCostWarehouse_Avg == 0.0) &&
               (holdingCostRetailer_Avg == 0.0) &&
               (transportationCostWarehouseToRetailer_Avg == 0.0) &&
               (costOfUnmetDemand_Avg == 0.0);
    }
};

// Structure representing saved cuts
struct SAVEDCUTS
{
    int veh_ind = 0;           // Vehicle index
    int per_ind = 0;           // Period index
    vector<int> edge_ind; // Edge indices in the cut
    int cut_rhs = 0;           // Right-hand side of the cut

    // Check if the saved cut is empty
    bool empty() const
    {
        return (veh_ind == 0) &&
               (per_ind == 0) &&
               edge_ind.empty() &&
               (cut_rhs == 0);
    }
};

#endif // HEADERS_H
