#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "ParameterSetting.h"
#include "SolutionManager.h"

#include "stochastic/MWPRP_FE.h"
#include "stochastic/ILS.h"
#include "stochastic/S2EPRP_BC.h"
#include "stochastic/RS2EPRP.h"

#include "deterministic/MWPRP_FE_Deterministic.h"
#include "deterministic/ILS_Deterministic.h"
#include "deterministic/R2EPRP.h"
#include "deterministic/BC_Deterministic.h"

class Algorithms
{
public:
    Algorithms(const string solutionAlgorithm, const ParameterSetting &parameters);

    bool solve_S2EPRP_BC();
    bool solve_S2EPRP_HILS();

    bool solve_EV();
    // bool solve_WS();
    bool solve_Deterministic_HILS(const vector<vector<double>> &deterministicDemand, bool shortageAllowed = true, int scenarioIndex = -1);

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object

    string solAlg;

    SolutionFirstEchelon sol_FE_incumbent;
    SolutionSecondEchelon sol_SE_incumbent;
    Result result_incumbent;

    SolutionFirstEchelon sol_FE_incumbent_Deterministic;
    SolutionSecondEchelon_Deterministic sol_SE_incumbent_Deterministic;
    Result result_incumbent_Deterministic;
   
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


    // Deterministic
    bool solveFirstEchelon_Deterministic(SolutionFirstEchelon &solFE_current, const vector<vector<double>> &deterministicDemand, bool shortageAllowed);
    bool runILSForSecondEchelon_Deterministic(SolutionFirstEchelon &solFE_current, 
                                            SolutionSecondEchelon_Deterministic &solSE_current, 
                                            Result &result_current, 
                                            const vector<vector<double>> &deterministicDemand, 
                                            bool shortageAllowed);
    void update_incumbent_Deterministic(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon_Deterministic &solSE_current, Result &result_current);
    void optimizeUnmetDemandAndRoutes_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, const vector<vector<double>> &deterministicDemand);
    bool solveRestrictedProblemAndFinalize_Deterministic(SolutionFirstEchelon &sol_FE, SolutionSecondEchelon_Deterministic &sol_SE, 
                                                         const vector<vector<double>> &deterministicDemand, bool shortageAllowed);

    void sortCustomersByUnmetDemand_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, vector<tuple<int, double>> &unmetDemand_Descending, int t);
    void handleUnmetDemandForCustomer_Deterministic(const vector<vector<double>> &deterministicDemand, SolutionSecondEchelon_Deterministic &sol_SE, 
                                                                                                        int t, int customerIndex, double unmetDemand);
    int findCurrentWarehouse_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int customerIndex);
    void attemptToInsertCustomerIntoWarehouse_Deterministic(const vector<vector<double>> &deterministicDemand, SolutionSecondEchelon_Deterministic &sol_SE, 
                                                            int t, int currentWarehouse, int customerIndex, double unmetDemand, double unmetDemCost);

    void updateRemainingCapacities_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, 
                                                 int wareToInsert, vector<double> &remainingVehicleCapacityWarehouse,double &remainingWarehouseCapacity);

    void findBestInsertionPosition_Deterministic(const vector<vector<double>> &deterministicDemand, SolutionSecondEchelon_Deterministic &sol_SE, 
                                                 int t, int wareToInsert, int customerIndex, vector<double> &remainingVehicleCapacityWarehouse, 
                                                 double &remainingWarehouseCapacity, int &routeToInsert, int &posToInsert, double &minCostToInsert);

    void applyInsertion_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int wareToInsert, int currentWarehouse, 
                                      int customerIndex, int routeToInsert, int posToInsert, double tempDeliveryQuantity);
    void removeCustomerFromCurrentRoute_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int currentWarehouse, int customerIndex);

    void printSolution_Deterministic();
    void organizeSolution_Deterministic();
};

#endif // ALGORITHMS_H