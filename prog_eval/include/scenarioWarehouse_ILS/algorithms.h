#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "ParameterSetting.h"
#include "MWPRP_FE.h"
#include "ILS.h"
#include "S2EPRP_BC.h"
#include "RS2EPRP.h"
#include "SolutionManager.h"


class Algorithms
{
public:
    Algorithms(const string solutionAlgorithm, const ParameterSetting &parameters);

    bool solve_S2EPRP_BC(Solution& solFinal, const Solution& warmStart = {});
    bool solve_S2EPRP_HILS(Solution& solFinal, const Solution& initSol = {});
private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object

    string solAlg;
   
   // Solves the first-echelon problem
    bool solveFirstEchelon(Solution &sol_Current);

    // Runs the ILS for the second-echelon problem
    bool runILSForSecondEchelon(Solution &sol_Current, Solution &solFinal);

    // Optimizes unmet demand and routes for all customers
    void optimizeUnmetDemandAndRoutes(Solution &sol);

    // Sorts customers in descending order of unmet demand
    void sortCustomersByUnmetDemand(Solution &sol, vector<tuple<int, double>> &unmetDemand_Descending, int t, int s);

    // Handles unmet demand for a specific customer
    void handleUnmetDemandForCustomer(Solution &sol, int s, int t, int customerIndex, double unmetDemand);

    // Finds the current warehouse assigned to a customer
    int findCurrentWarehouse(Solution &sol, int s, int t, int customerIndex);

    // Attempts to insert a customer into a different warehouse
    bool attemptToInsertCustomerIntoWarehouse(Solution &sol, int s, int t, int wareToInsert, int currentWarehouse, int customerIndex, double unmetDemand, double unmetDemCost);

    // Updates the remaining vehicle and warehouse capacities
    void updateRemainingCapacities(Solution &sol, int s, int t, int wareToInsert, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity);

    // Finds the best insertion position for a customer in a warehouse
    void findBestInsertionPosition(Solution &sol, int s, int t, int wareToInsert, int customerIndex, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity, int &routeToInsert, int &posToInsert, double &minCostToInsert);

    // Applies the insertion of a customer into a warehouse route
    void applyInsertion(Solution &sol, int s, int t, int wareToInsert, int currentWarehouse, int customerIndex, int routeToInsert, int posToInsert, double tempDeliveryQuantity);

    // Removes a customer from their current warehouse
    void removeCustomerFromCurrentRoute(Solution &sol, int s, int t, int currentWarehouse, int customerIndex);

    // Solves the restricted problem and finalizes the solution
    bool solveRestrictedProblemAndFinalize(Solution &sol);

    std::pair<int, double> minInsertionCost(const vector<int> &routesPeriod, int i);
};

#endif // ALGORITHMS_H