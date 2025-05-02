#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "ParameterSetting.h"
#include "SolutionManager.h"

#include "MWPRP_FE.h"
#include "ILS.h"
#include "S2EPRP_BC.h"
#include "RS2EPRP.h"

#include "MWPRP_FE_Deterministic.h"
#include "ILS_Deterministic.h"
#include "R2EPRP.h"
#include "BC_Deterministic.h"

#include <random>

class Algorithms
{
public:
    Algorithms(const ParameterSetting &parameters);

    bool solve_S2EPRP_BC();
    bool solve_S2EPRP_HILS();

    bool solve_Deterministic_HILS(const SolutionFirstEchelon &sol_FE_EV = {}, const SolutionSecondEchelon_Deterministic &sol_SE_EV = {});
    bool solve_2EPRP();

private:
    ParameterSetting params; // Member variable to hold the ParameterSetting object

    SolutionFirstEchelon sol_FE_incumbent;
    SolutionSecondEchelon sol_SE_incumbent;
    Result result_incumbent;

    SolutionFirstEchelon sol_FE_incumbent_Deterministic;
    SolutionSecondEchelon_Deterministic sol_SE_incumbent_Deterministic;
    Result result_incumbent_Deterministic;

    std::mt19937 rng;
    std::uniform_int_distribution<int> distribution;

    // Solves the first-echelon problem
    bool solveFirstEchelon(SolutionFirstEchelon &solFE_current);
    bool runILSForSecondEchelon(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon &solSE_current, Result &result_current,
                                int iter = -1,
                                std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::time_point{},
                                bool savePerIterSol = false);
    void update_incumbent(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon &solSE_current, Result &result_current);
    void rearrangeCustomerAssignments(SolutionSecondEchelon &sol_SE);
    bool Merge_Warehouses(SolutionSecondEchelon &sol_SE);
    void sortCustomersByUnmetDemand(SolutionSecondEchelon &sol_SE, vector<tuple<int, double>> &unmetDemand_Descending, int t, int s);
    void handleUnmetDemandForCustomer(SolutionSecondEchelon &sol_SE, int s, int t, int customerIndex, double unmetDemand);
    int findCurrentWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int customerIndex);
    void attemptToInsertCustomerIntoWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int currentWarehouse, int customerIndex, bool hasUnmetDemand);
    void updateRemainingCapacities(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity);

    void findBestInsertionPosition(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, int customerIndex,
                                   vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity,
                                   int &routeToInsert, int &posToInsert, double &minCostToInsert, double &deliveryQuantity);

    void applyInsertion(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, int currentWarehouse,
                        int customerIndex, int routeToInsert, int posToInsert, double tempDeliveryQuantity);

    void removeCustomerFromCurrentRoute(SolutionSecondEchelon &sol_SE, int s, int t, int currentWarehouse, int customerIndex);
    bool solveRestrictedProblemAndFinalize(SolutionFirstEchelon &sol_FE, SolutionSecondEchelon &sol_SE);
    std::pair<int, double> minInsertionCost(const vector<int> &routesPeriod, int i);

    std::tuple<int, int, int> visitedByWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int i);

    void printSolution();
    void organizeSolution();

    // -----------------------
    // Deterministic
    bool solveFirstEchelon_Deterministic(SolutionFirstEchelon &solFE_current);
    bool runILS_SE_Deterministic(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon_Deterministic &solSE_current, Result &result_current,
                                 int iter = -1,
                                 std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::time_point{},
                                 bool savePerIterSol = false);
    void update_incumbent_Deterministic(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon_Deterministic &solSE_current, Result &result_current);
    void rearrangeCustomerAssignments_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE);
    bool solveRestrictedProblemAndFinalize_Deterministic(SolutionFirstEchelon &sol_FE, SolutionSecondEchelon_Deterministic &sol_SE);

    bool Merge_Warehouses_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE);
    void sortCustomersByUnmetDemand_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, vector<tuple<int, double>> &unmetDemand_Descending, int t);
    void handleUnmetDemandForCustomer_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int customerIndex, double unmetDemand);
    int findCurrentWarehouse_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int customerIndex);
    void attemptToInsertCustomerIntoWarehouse_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int currentWarehouse, int customerIndex, bool hasUnmetDemand);

    void updateRemainingCapacities_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t,
                                                 int wareToInsert, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity);

    void findBestInsertionPosition_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int wareToInsert, int customerIndex,
                                                 vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity,
                                                 int &routeToInsert, int &posToInsert, double &minCostToInsert, double &deliveryQuantity);

    void applyInsertion_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int wareToInsert, int currentWarehouse,
                                      int customerIndex, int routeToInsert, int posToInsert, double tempDeliveryQuantity);
    void removeCustomerFromCurrentRoute_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int currentWarehouse, int customerIndex);

    std::tuple<int, int, int> visitedByWarehouse_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int i);

    void printSolution_Deterministic();
    void organizeSolution_Deterministic();

    bool read_SolutionEV(SolutionFirstEchelon &sol_FE_EV, SolutionSecondEchelon_Deterministic &sol_SE_EV);
    bool solve_EEV_HILS(const SolutionFirstEchelon &sol_FE_EV, const vector<vector<double>> &deterministicDemand);
    bool runILS_EEV(const SolutionFirstEchelon &sol_FE_EV,
                    SolutionSecondEchelon_Deterministic &solSE_current,
                    Result &result_current,
                    const vector<vector<double>> &deterministicDemand);
};

#endif // ALGORITHMS_H