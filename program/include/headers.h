#ifndef HEADERS_H
#define HEADERS_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

using namespace std;

// signed integers
#define NONE -1 // Unknown

struct CUTSET
{
    int dim; // size of cut
    int *S;  // set of vertices in cut
};

struct Result
{
    bool success = true;
    string status;
    double objValue;
    double lowerBound;
    double optimalityGap;
    double CPUtime;
};

struct Solution
{
    // Solution Values
    vector<double> productionSetup;
    vector<vector<double>> productionQuantity;
    vector<vector<vector<double>>> inventory;
    vector<vector<vector<double>>> backlog;
    vector<vector<vector<double>>> backorder;
    vector<vector<vector<vector<double>>>> delivery;
    vector<vector<vector<vector<double>>>> nodeVisit;
    vector<vector<vector<vector<double>>>> edgeVisit;
    vector<vector<double>> servicelevel;
    vector<vector<vector<double>>> alphaAux;
    vector<vector<vector<double>>> betaAux;

    double setupCost_Total;
    double productionCost_Total_Avg;
    double holdingCost_Total_Avg;
    double transpCost_Total_Avg;

    bool empty() const
    {
        return productionSetup.empty() &&
               productionQuantity.empty() &&
               alphaAux.empty() &&
               betaAux.empty() &&
               (setupCost_Total == 0.0) &&
               (productionCost_Total_Avg == 0.0) &&
               (holdingCost_Total_Avg == 0.0) &&
               (transpCost_Total_Avg == 0.0);
    }
};

struct SAVEDCUTS
{
    int veh_ind;
    int per_ind;
    vector<int> edge_ind;
    int cut_rhs;

    bool empty() const
    {
        return (veh_ind == 0) &&
               (per_ind == 0) &&
               edge_ind.empty() &&
               (cut_rhs == 0);
    }
};

#endif // HEADERS_H
