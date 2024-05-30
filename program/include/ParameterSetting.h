#ifndef PARAMETERSETTING_H
#define PARAMETERSETTING_H

#include "headers.h"
#include "TSP.h"
#include <fstream>
#include <stdexcept>
#include <limits>
#include <cassert>


class ParameterSetting
{
public:
    ParameterSetting(int argc, char *argv[]);

    bool setParameters();

    string solutionAlgorithm;
    string inp;
    int numWarehouses;
    int numRetailers;
    int numPeriods;
    int numVehiclesPlant;
    int numVehiclesWarehouse;
    int numScenarios;
    double unmetDemandPenaltyCoeff;
    double uncertaintyRange;
    string probabilityFunction;
    string instance;
    int numNodes;
    int numEdges;
    vector<double> coordX;
    vector<double> coordY;
    vector<double> unitHoldingCost;
    vector<double> storageCapacity;
    vector<double> initialInventory;
    vector<double> unmetDemandPenalty;
    vector<double> consumeRate;
    vector<vector<vector<double>>> demand;
    vector<int> index_i;
    vector<int> index_j;
    vector<vector<int>> index_e;
    vector<double> probability;
    vector<vector<double>> transportationCost;
    int originalDimension;
    int originalHorizon;
    int originalVehicleCapacity;
    double unitProdCost;
    double setupCost;
    double prodCapacity;
    double vehicleCapacityPlant;
    double vehicleCapacityWarehouse;

    vector<vector<int>> getSortedWarehousesByDistance() const
    {
        return sorted_warehouses_by_distance;
    }

    // Getter for retailers_assigned_to_warehouse
    vector<vector<int>> getRetailersAssignedToWarehouse() const
    {
        return retailers_assigned_to_warehouse;
    }

    vector<vector<int>> getRouteMatrix() const
    {
        return routeMatrix;
    }

    vector<vector<int>> getOptimalRoutes() const
    {
        return optimalRoutes;
    }

    vector<double> getRouteCosts() const
    {
        return routeCosts;
    }
private:
    void initializeIndices() bool readDataFromFile();
    void readOriginalDatasetInfo(ifstream &file) void readNodeData(ifstream &file) bool monteCarloSimulation();
    void checkDemandsDistribution();
    void adjustInitialInventory();
    void calculateProductionCapacity();
    void setWarehouseParameters();
    void calculateVehicleCapacities();
    void calculateTransportationCost();
    void calculateUnmetDemandPenalty();
    void setProbabilities();
    void printParameters();
    void saveInstance();
    
    vector<vector<int>> sorted_warehouses_by_distance;
    vector<vector<int>> retailers_assigned_to_warehouse;
    void sort_warehouses_by_distance();
    void assign_retailers_to_warehouse()

    double uniformDistribution(double min, double max, unsigned long int seed);
    double normalDistribution(double mean, double sd, unsigned int seed);
    double gammaDistribution(double mean, double sd, unsigned int seed);

    vector<vector<int>> routeMatrix;
    void generateAllRoutes();

    vector<double> routeCosts;
    vector<vector<int>> optimalRoutes;
    void solveTSPForRoutes();
};

#endif // PROBLEMPARAMS_H