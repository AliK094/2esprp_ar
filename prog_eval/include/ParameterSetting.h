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

    // Public member variables
    string solutionAlgorithm;
    string inputFile;
    int numWarehouses;
    int numCustomers;
    int numPeriods;
    int numVehicles_Plant;
    int numVehicles_Warehouse;
    int numScenarios;
    double unmetDemandPenaltyCoeff;
    double uncertaintyRange;
    string probabilityFunction;
    string instance;
    int numNodes_Total;
    int numNodes_FirstEchelon;
    int numNodes_SecondEchelon;
    int numEdges_FirstEchelon;
    int numEdges_SecondEchelon;
    int numVehicles_SecondEchelon;

    int originalDimension;
    int originalHorizon;
    int originalVehicleCapacity;
    double unitProdCost;
    double setupCost;
    double prodCapacity;
    double vehicleCapacity_Plant;
    double vehicleCapacity_Warehouse;

    double coordX_Plant;
    double coordY_Plant;
    vector<double> coordX_Warehouse;
    vector<double> coordY_Warehouse;
    vector<double> coordX_Customer;
    vector<double> coordY_Customer;

    double unitHoldingCost_Plant;
    vector<double> unitHoldingCost_Warehouse;
    vector<double> unitHoldingCost_Customer;

    double storageCapacity_Plant;
    vector<double> storageCapacity_Warehouse;
    vector<double> storageCapacity_Customer;

    double initialInventory_Plant;
    vector<double> initialInventory_Warehouse;
    vector<double> initialInventory_Customer;

    vector<double> unmetDemandPenalty;
    vector<double> consumeRate;
    vector<vector<vector<double>>> demand;
    vector<double> probability;

    vector<int> index_i_FirstEchelon;
    vector<int> index_j_FirstEchelon;
    vector<vector<int>> index_e_FirstEchelon;

    vector<vector<double>> transportationCost_FirstEchelon;

    vector<int> index_i_SecondEchelon;
    vector<int> index_j_SecondEchelon;
    vector<vector<int>> index_e_SecondEchelon;

    vector<vector<int>> set_WarehouseVehicles;

    vector<vector<double>> transportationCost_SecondEchelon;

    vector<vector<vector<double>>> DeliveryUB_perCustomer;
    vector<vector<double>> DeliveryUB;
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
};

#endif // PROBLEMPARAMS_H