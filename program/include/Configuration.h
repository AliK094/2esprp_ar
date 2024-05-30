// Configuration.h
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <vector>
#include <string>

struct Configuration {
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
    double unitProdCost;
    double setupCost;
    double vehicleCapacityPlant;
    double vehicleCapacityWarehouse;

    vector<int> index_i;
    vector<int> index_j;
    vector<vector<int>> index_e;

    vector<double> probability;
    vector<vector<double>> transportationCost;
};

#endif // CONFIGURATION_H
