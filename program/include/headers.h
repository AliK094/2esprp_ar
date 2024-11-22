#ifndef HEADERS_H
#define HEADERS_H

#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>
#include "ilcplex/ilocplex.h"

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::tuple;

namespace fs = std::filesystem;


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

// Structure representing a solution
// Result
struct Result
{
    double objValue_firstEchelon = 0.0; // Total cost
    double objValue_secondEchelon = 0.0;
    double objValue_Total = 0.0;

    string status;
    double totalCPUTime = 0.0;
    double lowerBound = 0.0;
    double optimalityGap = 0.0;

    void clear()
    {
        // Reset the status string to an empty value
        status.clear();
    }
};

// First echelon
struct SolutionFirstEchelon
{
    vector<int> productionSetup;
    vector<double> productionQuantity;
    vector<double> plantInventory;
    vector<vector<double>> deliveryQuantityToWarehouse;
    vector<vector<vector<int>>> routesPlantToWarehouse;

    double setupCost = 0.0;                                 // Total setup cost
    double productionCost = 0.0;                            // Total production cost
    double holdingCostPlant = 0.0;                          // Total holding cost at the plant
    double transportationCostPlantToWarehouse = 0.0;        // Total transportation cost from the plant to warehouses

    // Check if the solution is empty
    bool empty() const
    {
        return productionSetup.empty() &&
               productionQuantity.empty() &&
               plantInventory.empty() &&
               deliveryQuantityToWarehouse.empty() &&
               routesPlantToWarehouse.empty();
    }

    void clear()
    {
        // Clear all vectors
        productionSetup.clear();
        productionQuantity.clear();
        plantInventory.clear();
        deliveryQuantityToWarehouse.clear();
        routesPlantToWarehouse.clear();
        setupCost = 0.0;
        productionCost = 0.0;
        holdingCostPlant = 0.0;
        transportationCostPlantToWarehouse = 0.0;
    }
};

// Structure representing a solution
struct SolutionSecondEchelon
{
    vector<vector<vector<double>>> warehouseInventory;
    vector<vector<vector<double>>> customerInventory;
    vector<vector<vector<double>>> customerUnmetDemand;
    vector<vector<vector<double>>> deliveryQuantityToCustomer;
    vector<vector<vector<vector<vector<int>>>>> routesWarehouseToCustomer;
    vector<vector<vector<vector<int>>>> customerAssignmentToWarehouse;

    double holdingCostWarehouse_Avg = 0.0;                  // Average total holding cost at the warehouse
    double holdingCostCustomer_Avg = 0.0;                   // Average total holding cost at the customer
    double transportationCostWarehouseToCustomer_Avg = 0.0; // Average total transportation cost from warehouses to customers
    double costOfUnmetDemand_Avg = 0.0;                     // Average total cost of unmet demand

    // Check if the solution is empty
    bool empty() const
    {
        return warehouseInventory.empty() &&
               customerInventory.empty() &&
               customerUnmetDemand.empty() &&
               deliveryQuantityToCustomer.empty() &&
               routesWarehouseToCustomer.empty() &&
               customerAssignmentToWarehouse.empty();
    }

    void clear()
    {
        // Clear all vectors
        warehouseInventory.clear();
        customerInventory.clear();
        customerUnmetDemand.clear();
        deliveryQuantityToCustomer.clear();
        routesWarehouseToCustomer.clear();
        customerAssignmentToWarehouse.clear();

        // Reset all cost and time values to 0
        holdingCostWarehouse_Avg = 0.0;
        holdingCostCustomer_Avg = 0.0;
        transportationCostWarehouseToCustomer_Avg = 0.0;
        costOfUnmetDemand_Avg = 0.0;
    }
};

// Structure representing a solution
struct ScenarioSolutionSecondEchelon
{
    int scenarioID;

    vector<vector<double>> warehouseInventory_Scenario;
    vector<vector<double>> customerInventory_Scenario;
    vector<vector<double>> customerUnmetDemand_Scenario;
    vector<vector<double>> deliveryQuantityToCustomer_Scenario;
    vector<vector<vector<vector<int>>>> routesWarehouseToCustomer_Scenario;
    vector<vector<vector<int>>> customerAssignmentToWarehouse_Scenario;

    double holdingCostWarehouse_Scenario = 0.0;                  // Average total holding cost at the warehouse
    double holdingCostCustomer_Scenario = 0.0;                   // Average total holding cost at the customer
    double transportationCostWarehouseToCustomer_Scenario = 0.0; // Average total transportation cost from warehouses to customers
    double costOfUnmetDemand_Scenario = 0.0;                     // Average total cost of unmet demand

    double ScenarioObjValue_SE = 0.0; // Total cost
    string status;
    double ScenarioCPUTime_SE = 0.0;

    // Check if the s        olution is empty
    bool empty() const
    {
        return warehouseInventory_Scenario.empty() &&
               customerInventory_Scenario.empty() &&
               customerUnmetDemand_Scenario.empty() &&
               deliveryQuantityToCustomer_Scenario.empty() &&
               routesWarehouseToCustomer_Scenario.empty() &&
               customerAssignmentToWarehouse_Scenario.empty() &&
               (holdingCostWarehouse_Scenario == 0.0) &&
               (holdingCostCustomer_Scenario == 0.0) &&
               (transportationCostWarehouseToCustomer_Scenario == 0.0) &&
               (costOfUnmetDemand_Scenario == 0.0);
    }

    void clear()
    {
        // Clear all vectors
        warehouseInventory_Scenario.clear();
        customerInventory_Scenario.clear();
        customerUnmetDemand_Scenario.clear();
        deliveryQuantityToCustomer_Scenario.clear();
        routesWarehouseToCustomer_Scenario.clear();
        customerAssignmentToWarehouse_Scenario.clear();

        // Reset all cost and time values to 0
        holdingCostWarehouse_Scenario = 0.0;
        holdingCostCustomer_Scenario = 0.0;
        transportationCostWarehouseToCustomer_Scenario = 0.0;
        costOfUnmetDemand_Scenario = 0.0;
        ScenarioObjValue_SE = 0.0;
        ScenarioCPUTime_SE = 0.0;

        // Reset the status string to an empty value
        status.clear();
    }  
};

struct SolutionWarmStart
{
    vector<int> productionSetup_WarmStart;
    vector<vector<vector<int>>> routesPlantToWarehouse_WarmStart;
    vector<vector<vector<vector<vector<int>>>>> routesWarehouseToCustomer_WarmStart;
    vector<vector<vector<vector<int>>>> customerAssignmentToWarehouse_WarmStart;

    // Check if the solution is empty
    bool empty() const
    {
        return productionSetup_WarmStart.empty() &&
               routesPlantToWarehouse_WarmStart.empty() &&
               routesWarehouseToCustomer_WarmStart.empty() &&
               customerAssignmentToWarehouse_WarmStart.empty();
    }

    void clear()
    {
        // Clear all vectors
        productionSetup_WarmStart.clear();
        routesPlantToWarehouse_WarmStart.clear();
        routesWarehouseToCustomer_WarmStart.clear();
        customerAssignmentToWarehouse_WarmStart.clear();
    }
};

// Structure representing a solution
struct SolutionSecondEchelon_Deterministic
{
    vector<vector<double>> warehouseInventory;
    vector<vector<double>> customerInventory;
    vector<vector<double>> customerUnmetDemand;
    vector<vector<double>> deliveryQuantityToCustomer;
    vector<vector<vector<vector<int>>>> routesWarehouseToCustomer;
    vector<vector<vector<int>>> customerAssignmentToWarehouse;

    double holdingCostWarehouse = 0.0;                  // Average total holding cost at the warehouse
    double holdingCostCustomer = 0.0;                   // Average total holding cost at the customer
    double transportationCostWarehouseToCustomer = 0.0; // Average total transportation cost from warehouses to customers
    double costOfUnmetDemand = 0.0;                     // Average total cost of unmet demand
    double handlingCostSatellite = 0.0;

    double ScenarioObjValue_SE = 0.0; // Total cost
    string status;
    double ScenarioCPUTime_SE = 0.0;

    // Check if the s        olution is empty
    bool empty() const
    {
        return warehouseInventory.empty() &&
               customerInventory.empty() &&
               customerUnmetDemand.empty() &&
               deliveryQuantityToCustomer.empty() &&
               routesWarehouseToCustomer.empty() &&
               customerAssignmentToWarehouse.empty() &&
               (holdingCostWarehouse == 0.0) &&
               (holdingCostCustomer == 0.0) &&
               (transportationCostWarehouseToCustomer == 0.0) &&
               (costOfUnmetDemand == 0.0) &&
               (handlingCostSatellite == 0.0);
    }

    void clear()
    {
        // Clear all vectors
        warehouseInventory.clear();
        customerInventory.clear();
        customerUnmetDemand.clear();
        deliveryQuantityToCustomer.clear();
        routesWarehouseToCustomer.clear();
        customerAssignmentToWarehouse.clear();

        // Reset all cost and time values to 0
        holdingCostWarehouse = 0.0;
        holdingCostCustomer = 0.0;
        transportationCostWarehouseToCustomer = 0.0;
        costOfUnmetDemand = 0.0;
        ScenarioObjValue_SE = 0.0;
        ScenarioCPUTime_SE = 0.0;
        handlingCostSatellite = 0.0;

        // Reset the status string to an empty value
        status.clear();
    }  
};

struct SolutionWarmStart_Deterministic
{
    vector<int> productionSetup_WarmStart;
    vector<vector<vector<int>>> routesPlantToWarehouse_WarmStart;
    vector<vector<vector<vector<int>>>> routesWarehouseToCustomer_WarmStart;
    vector<vector<vector<int>>> customerAssignmentToWarehouse_WarmStart;

    // Check if the solution is empty
    bool empty() const
    {
        return productionSetup_WarmStart.empty() &&
               routesPlantToWarehouse_WarmStart.empty() &&
               routesWarehouseToCustomer_WarmStart.empty() &&
               customerAssignmentToWarehouse_WarmStart.empty();
    }

    void clear()
    {
        // Clear all vectors
        productionSetup_WarmStart.clear();
        routesPlantToWarehouse_WarmStart.clear();
        routesWarehouseToCustomer_WarmStart.clear();
        customerAssignmentToWarehouse_WarmStart.clear();
    }
};

// Structure representing saved cuts
struct SAVEDCUTS
{
    int veh_ind = 0;      // Vehicle index
    int per_ind = 0;      // Period index
    vector<int> edge_ind; // Edge indices in the cut
    int cut_rhs = 0;      // Right-hand side of the cut

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
