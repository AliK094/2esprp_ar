#ifndef ParameterSetting_H
#define ParameterSetting_H

#include "headers.h"
#include "TSP.h"
#include <fstream>
#include <stdexcept>
#include <limits>
#include <cassert>
#include <numeric> 
#include <random>


class ParameterSetting
{
public:
    ParameterSetting(int argc, char *argv[]);

    bool setParameters();

    // Getters for private members
    vector<vector<int>> getSortedWarehousesByDistance() const;
    vector<vector<vector<vector<int>>>> getCustomersAssignedToWarehouse() const;
    vector<vector<vector<int>>> getWarehouseAssignedToCustomer() const;
    vector<vector<int>> getRouteMatrix() const;
    vector<vector<int>> getOptimalRoutes() const;
    vector<double> getRouteCosts() const;
    vector<double> getDualValues_WarehouseInventoryLB() const;
    SolutionWarmStart readSolutionWarmStart();

    vector<vector<vector<int>>> getCustomersAssignedToWarehouse_det() const;
    vector<vector<int>> getWarehouseAssignedToCustomer_det() const;

    // Public member variables
    string problemType = "";
    string solutionAlgorithm = "";
    string inputFile;
    int numWarehouses, numCustomers, numPeriods, numVehicles_Plant, numVehicles_Warehouse;
    string instance;

    // Stocastic Parameters
    int numScenarios;
    double unmetDemandPenaltyCoeff;
    double uncertaintyRange;
    string probabilityFunction;

    int scenarioIndex = -1;

    vector<double> unmetDemandPenalty;
    vector<vector<vector<double>>> demand;
    vector<double> probability;

    vector<vector<vector<double>>> DeliveryUB_perCustomer;
    vector<vector<double>> DeliveryUB;
    
    // Problem Parameters
    int numNodes_Total, numNodes_FirstEchelon, numNodes_SecondEchelon, numEdges_FirstEchelon, numEdges_SecondEchelon;
    int numVehicles_SecondEchelon;

    int originalDimension, originalHorizon, originalVehicleCapacity;

    double unitProdCost, setupCost;
    double prodCapacity;
    double vehicleCapacity_Plant, vehicleCapacity_Warehouse;

    double coordX_Plant, coordY_Plant;
    vector<double> coordX_Warehouse, coordY_Warehouse;
    vector<double> coordX_Customer, coordY_Customer;

    double unitHoldingCost_Plant;
    vector<double> unitHoldingCost_Warehouse, unitHoldingCost_Customer;

    double storageCapacity_Plant;
    vector<double> storageCapacity_Warehouse, storageCapacity_Customer;

    double initialInventory_Plant;
    vector<double> initialInventory_Warehouse, initialInventory_Customer;

    vector<double> consumeRate;

    vector<int> index_i_FirstEchelon, index_j_FirstEchelon;
    vector<vector<int>> index_e_FirstEchelon;
    vector<vector<double>> transportationCost_FirstEchelon;

    vector<int> index_i_SecondEchelon, index_j_SecondEchelon;
    vector<vector<int>> index_e_SecondEchelon;
    vector<vector<double>> transportationCost_SecondEchelon;

    vector<vector<int>> set_WarehouseVehicles;

    // for deterministic case
    vector<vector<double>> demand_Deterministic;
    vector<vector<double>> DelUB_perCus_Det;
    vector<double> DelUB_Det;
    vector<double> unitHandlingCost_Satellite;

    
private:
    void initializeIndices();
    bool generateData();
    bool readDataFromFile();
    void read2ESPRPlDatasetInfo(std::ifstream &file);

    bool monteCarloSimulation();
    void generateScenarioDemands(int scenario, int period);
    void checkDemandsDistribution() const;
    void calculateTransportationCost_FirstEchelon();
    void calculateTransportationCost_SecondEchelon();
    void calculateDeliveryUB();
    void calculateUnmetDemandPenalty();
    void setProbabilities();
    void printParameters() const;
    void saveInstance();
    
    void sort_warehouses_by_distance();
    void assign_customers_to_warehouse();
    void generateAllRoutes();
    void solveTSPForRoutes();

    void calculateDeterministicDemand();
    void calculateDeliveryUB_Deterministic();
    void setSatelliteUnitHandlingCost();

    // Distribution functions
    double uniformDistribution(double min, double max, unsigned long int seed = 0);
    double normalDistribution(double mean, double sd, unsigned int seed);
    double gammaDistribution(double mean, double sd, unsigned int seed);

    // Private member variables
    vector<vector<int>> sorted_warehouses_by_distance;
    vector<vector<vector<vector<int>>>> customers_assigned_to_warehouse;
    vector<vector<vector<int>>> warehouse_assigned_to_customer;
    vector<vector<int>> routeMatrix;
    vector<double> routeCosts;
    vector<vector<int>> optimalRoutes;

    vector<vector<vector<int>>> customers_assigned_to_warehouse_det;
    vector<vector<int>> warehouse_assigned_to_customer_det;
};

#endif // PROBLEMPARAMS_H