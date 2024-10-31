#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "ParameterSetting.h"
#include "MWPRP_FE.h"
#include "ILS.h"
#include "S2EPRP_BC.h"
#include "EV_BC.h"
#include "RS2EPRP.h"
#include "SolutionManager.h"


class Algorithms
{
public:
    Algorithms(const string solutionAlgorithm, const ParameterSetting &parameters);

    bool solve_S2EPRP_BC();
    bool solve_S2EPRP_HILS();
    bool solve_EV();
    bool solve_Deterministic_HILS();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object

    string solAlg;

    SolutionFirstEchelon sol_FE_incumbent;
    SolutionSecondEchelon sol_SE_incumbent;
    Result result_incumbent;
   
   // Solves the first-echelon problem
    bool solveFirstEchelon(SolutionFirstEchelon &solFE_current);

    // Runs the ILS for the second-echelon problem
    bool runILSForSecondEchelon(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon &solSE_current, Result &result_current);

    void update_incumbent(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon &solSE_current, Result &result_current);

    // Optimizes unmet demand and routes for all customers
    void optimizeUnmetDemandAndRoutes(SolutionSecondEchelon &sol_SE);

    // Sorts customers in descending order of unmet demand
    void sortCustomersByUnmetDemand(SolutionSecondEchelon &sol_SE, vector<tuple<int, double>> &unmetDemand_Descending, int t, int s);

    // Handles unmet demand for a specific customer
    void handleUnmetDemandForCustomer(SolutionSecondEchelon &sol_SE, int s, int t, int customerIndex, double unmetDemand);

    // Finds the current warehouse assigned to a customer
    int findCurrentWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int customerIndex);

    // Attempts to insert a customer into a different warehouse
    // bool attemptToInsertCustomerIntoWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, int currentWarehouse, int customerIndex, double unmetDemand, double unmetDemCost);
    void attemptToInsertCustomerIntoWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int currentWarehouse, int customerIndex, double unmetDemand, double unmetDemCost);

    // Updates the remaining vehicle and warehouse capacities
    void updateRemainingCapacities(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity);

    // Finds the best insertion position for a customer in a warehouse
    void findBestInsertionPosition(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, int customerIndex, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity, int &routeToInsert, int &posToInsert, double &minCostToInsert);

    // Applies the insertion of a customer into a warehouse route
    void applyInsertion(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, int currentWarehouse, int customerIndex, int routeToInsert, int posToInsert, double tempDeliveryQuantity);

    // Removes a customer from their current warehouse
    void removeCustomerFromCurrentRoute(SolutionSecondEchelon &sol_SE, int s, int t, int currentWarehouse, int customerIndex);

    // Solves the restricted problem and finalizes the solution
    bool solveRestrictedProblemAndFinalize(SolutionFirstEchelon &sol_FE, SolutionSecondEchelon &sol_SE);

    std::pair<int, double> minInsertionCost(const vector<int> &routesPeriod, int i);

    void printSolution();
    void organizeSolution();
};

#endif // ALGORITHMS_H