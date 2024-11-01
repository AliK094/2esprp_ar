#include "algorithms.h"

Algorithms::Algorithms(const string solutionAlgorithm, const ParameterSetting &parameters)
	: solAlg(solutionAlgorithm),
	  params(parameters)
{
}

bool Algorithms::solve_S2EPRP_BC()
{
	cout << "Start Solving The Stochastic Two-Echelon Production Routing Problem With Adaptive Routing Using Branch-and-Cut Algorithm." << endl;
	cout << "-------------------------------------------------------------------" << endl;
	SolutionWarmStart warmStartSolution = params.readSolutionWarmStart();

	// Branch and Cut
	S2EPRP_BC s2eprp_bc(params, warmStartSolution);
	if (!s2eprp_bc.Solve())
	{
		return EXIT_FAILURE;
	}
	sol_FE_incumbent = s2eprp_bc.getSolutionFE();
	sol_SE_incumbent = s2eprp_bc.getSolutionSE();
	result_incumbent = s2eprp_bc.getResult();

	// Save the solution and check feasibility
	SolutionManager solMgr_ILS(params, solAlg);
	solMgr_ILS.saveSolution(sol_FE_incumbent, sol_SE_incumbent);
	solMgr_ILS.checkFeasibility();
	solMgr_ILS.saveResultSummary(sol_FE_incumbent, sol_SE_incumbent, result_incumbent);

	return true;
}

bool Algorithms::solve_S2EPRP_HILS()
{
	cout << "Start Solving The Stochastic Two-Echelon Production Routing Problem With Hybrid-ILS." << endl;
	cout << "-------------------------------------------------------------------" << endl;

	SolutionFirstEchelon sol_FE;
	SolutionSecondEchelon sol_SE;
	Result result_temp;

	result_incumbent.objValue_Total = std::numeric_limits<double>::max();
	result_incumbent.totalCPUTime = 0.0;

	auto elapsedTime = 0.0;
	auto currentTime = std::chrono::high_resolution_clock::now();
	auto startTime = std::chrono::high_resolution_clock::now();

	// -----------------------------------------------------------------------------------------------------------------

	// Solve the first-echelon problem
	if (!solveFirstEchelon(sol_FE))
	{
		return EXIT_FAILURE;
	}

	// Run ILS for the second-echelon problem
	if (!runILSForSecondEchelon(sol_FE, sol_SE, result_temp))
	{
		return EXIT_FAILURE;
	}

	update_incumbent(sol_FE, sol_SE, result_temp);
	cout << "Initial Phase is finished." << endl;
	printSolution();

	// Handle unmet demand and optimize routes
	double timeLimit = 7200.0;
	const int maxIteration = 20;

	int iter = 0;
	while (iter < maxIteration && elapsedTime < timeLimit)
	{
		cout << "-----------------------------------------------------------------" << endl;
		cout << "Solving The Problem With Hybrid-ILS. Iteration: " << iter + 1 << endl;
		cout << "-----------------------------------------------------------------" << endl;

		sol_FE = sol_FE_incumbent;
		sol_SE = sol_SE_incumbent;

		cout << "Rearrange customer-warehouse assignment..." << endl;
		optimizeUnmetDemandAndRoutes(sol_SE);

		// Solve the restricted problem and finalize the solution
		if (!solveRestrictedProblemAndFinalize(sol_FE, sol_SE))
		{
			return EXIT_FAILURE;
		}

		if (!runILSForSecondEchelon(sol_FE, sol_SE, result_temp))
		{
			return EXIT_FAILURE;
		}

		update_incumbent(sol_FE, sol_SE, result_temp);
		printSolution();

		currentTime = std::chrono::high_resolution_clock::now();
		elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Computation Time (Hybrid-ILS) after iteration: " << iter + 1 << " = " << elapsedTime << " seconds" << endl;
		iter++;
	}

	result_incumbent.totalCPUTime = elapsedTime;
	cout << "\nTotal Computation Time (Hybrid-ILS): " << result_incumbent.totalCPUTime << " seconds" << endl;

	cout << "Final Phase is finished." << endl;
	cout << "-------------------------------------------------------------------" << endl;
	organizeSolution();
	printSolution();

	// Save the solution and check feasibility
	SolutionManager solMgr_ILS(params, solAlg);
	solMgr_ILS.saveSolution(sol_FE_incumbent, sol_SE_incumbent);
	solMgr_ILS.checkFeasibility();
	solMgr_ILS.saveResultSummary(sol_FE_incumbent, sol_SE_incumbent, result_incumbent);

	return true;
}

bool Algorithms::solve_EV()
{
	cout << "Start Solving The Expected Value (EV) Problem For the S2EPRP." << endl;
	cout << "-------------------------------------------------------------------" << endl;
	vector<vector<double>> deterministicDemand(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			deterministicDemand[i][t] = params.consumeRate[i];
		}
	}

	solve_EV_HILS(deterministicDemand);

	// Solve the EV problem using the Branch-and-Cut
	EV_BC ev_bc(params);
	if (!ev_bc.Solve())
	{
		return EXIT_FAILURE;
	}
	sol_FE_incumbent_EV = ev_bc.getSolutionFE();
	sol_SE_incumbent_EV = ev_bc.getSolutionSE();
	result_incumbent_EV = ev_bc.getResult();

	// Save the solution and check feasibility
	SolutionManager solMgr_ILS(params, solAlg);
	solMgr_ILS.saveSolutionEV(sol_FE_incumbent, sol_SE_incumbent);
	solMgr_ILS.checkFeasibilityEV();

	return true;
}

bool Algorithms::solve_EV_HILS(const vector<vector<double>> &deterministicDemand)
{
	cout << "Start Solving The Deterministic Two-Echelon Production Routing Problem With Hybrid-ILS For the EV Problem." << endl;
	cout << "-------------------------------------------------------------------" << endl;

	SolutionFirstEchelon sol_FE;
	SolutionSecondEchelon_Deterministic sol_SE_EV;
	Result result_temp;

	result_incumbent_EV.objValue_Total = std::numeric_limits<double>::max();
	result_incumbent_EV.totalCPUTime = 0.0;

	auto elapsedTime = 0.0;
	auto currentTime = std::chrono::high_resolution_clock::now();
	auto startTime = std::chrono::high_resolution_clock::now();

	// -----------------------------------------------------------------------------------------------------------------

	// Solve the first-echelon problem
	if (!solveFirstEchelon_EV(sol_FE, deterministicDemand))
	{
		return EXIT_FAILURE;
	}

	// Run ILS for the second-echelon problem
	if (!runILSForSecondEchelon_EV(sol_FE, sol_SE_EV, result_temp, deterministicDemand))
	{
		return EXIT_FAILURE;
	}

	update_incumbent(sol_FE, sol_SE, result_temp);
	cout << "Initial Phase is finished." << endl;
	printSolution();

	// Handle unmet demand and optimize routes
	double timeLimit = 7200.0;
	const int maxIteration = 20;

	int iter = 0;
	while (iter < maxIteration && elapsedTime < timeLimit)
	{
		cout << "-----------------------------------------------------------------" << endl;
		cout << "Solving The Problem With Hybrid-ILS. Iteration: " << iter + 1 << endl;
		cout << "-----------------------------------------------------------------" << endl;

		sol_FE = sol_FE_incumbent;
		sol_SE = sol_SE_incumbent;

		cout << "Rearrange customer-warehouse assignment..." << endl;
		optimizeUnmetDemandAndRoutes(sol_SE);

		// Solve the restricted problem and finalize the solution
		if (!solveRestrictedProblemAndFinalize(sol_FE, sol_SE))
		{
			return EXIT_FAILURE;
		}

		if (!runILSForSecondEchelon(sol_FE, sol_SE, result_temp))
		{
			return EXIT_FAILURE;
		}

		update_incumbent(sol_FE, sol_SE, result_temp);
		printSolution();

		currentTime = std::chrono::high_resolution_clock::now();
		elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Computation Time (Hybrid-ILS) after iteration: " << iter + 1 << " = " << elapsedTime << " seconds" << endl;
		iter++;
	}

	result_incumbent.totalCPUTime = elapsedTime;
	cout << "\nTotal Computation Time (Hybrid-ILS): " << result_incumbent.totalCPUTime << " seconds" << endl;

	cout << "Final Phase is finished." << endl;
	cout << "-------------------------------------------------------------------" << endl;
	organizeSolution();
	printSolution();

	// Save the solution and check feasibility
	SolutionManager solMgr_ILS(params, solAlg);
	solMgr_ILS.saveSolution(sol_FE_incumbent, sol_SE_incumbent);
	solMgr_ILS.checkFeasibility();
	solMgr_ILS.saveResultSummary(sol_FE_incumbent, sol_SE_incumbent, result_incumbent);

	return true;
}

bool Algorithms::solveFirstEchelon(SolutionFirstEchelon &solFE)
{
	cout << "Solve The First-Echelon Problem" << endl;
	MWPRP_FE mwprp_fe(params);
	if (!mwprp_fe.Solve())
	{
		return false;
	}
	solFE = mwprp_fe.getSolutionFE();
	return true;
}

bool Algorithms::solveFirstEchelon_EV(SolutionFirstEchelon &solFE, const vector<vector<double>> &deterministicDemand)
{
	cout << "Solve The First-Echelon Problem For the EV Problem" << endl;

	MWPRP_FE_Deterministic mwprp_fe_det(params, deterministicDemand);
	if (!mwprp_fe_det.Solve())
	{
		return false;
	}
	solFE = mwprp_fe_det.getSolutionFE();
	return true;
}

bool Algorithms::runILSForSecondEchelon(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon &solSE_current, Result &result_current)
{
	ILS_SIRP ils_SIRP(params, solFE_current, solSE_current);
	if (!ils_SIRP.run())
	{
		return false;
	}

	solFE_current = ils_SIRP.getSolutionFE();
	solSE_current = ils_SIRP.getSolutionSE();
	result_current = ils_SIRP.getResult();

	return true;
}

bool Algorithms::runILSForSecondEchelon_EV(SolutionFirstEchelon &solFE_current, 
									SolutionSecondEchelon_Deterministic &solSE_current_EV, 
									Result &result_current
									const vector<vector<double>> &deterministicDemand)
{
	ILS_SIRP_Deterministic ils_SIRP_det(params, solFE_current, solSE_current_EV, deterministicDemand);
	if (!ils_SIRP_det.run())
	{
		return false;
	}

	solFE_current = ils_SIRP_det.getSolutionFE();
	solSE_current_EV = ils_SIRP_det.getSolutionSE();
	result_current = ils_SIRP_det.getResult();

	return true;
}

void Algorithms::update_incumbent(SolutionFirstEchelon &sol_FE, SolutionSecondEchelon &sol_SE, Result &result_temp)
{
	if (result_temp.objValue_Total < result_incumbent.objValue_Total)
	{
		sol_FE_incumbent = sol_FE;
		sol_SE_incumbent = sol_SE;
		result_incumbent = result_temp;

		cout << "New Incumbent: " << result_incumbent.objValue_Total << endl;
	}
}

void Algorithms::optimizeUnmetDemandAndRoutes(SolutionSecondEchelon &sol_SE)
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			// cout << "Optimizing Unmet Demand for s = " << s + 1 << ", t = " << t + 1 << endl;
			vector<tuple<int, double>> unmetDemand_Descending;
			sortCustomersByUnmetDemand(sol_SE, unmetDemand_Descending, t, s);

			for (const auto &entry : unmetDemand_Descending)
			{
				int customerIndex = std::get<0>(entry);
				double unmetDemand = std::get<1>(entry);

				if (unmetDemand > 1e-4)
				{
					handleUnmetDemandForCustomer(sol_SE, s, t, customerIndex, unmetDemand);
				}
			}
		}
	}
}

void Algorithms::sortCustomersByUnmetDemand(SolutionSecondEchelon &sol_SE, vector<tuple<int, double>> &unmetDemand_Descending, int t, int s)
{
	for (int i = 0; i < params.numCustomers; ++i)
	{
		unmetDemand_Descending.emplace_back(i, sol_SE.customerUnmetDemand[i][t][s]);
	}

	sort(unmetDemand_Descending.begin(), unmetDemand_Descending.end(),
		 [](const tuple<int, double> &a, const tuple<int, double> &b)
		 {
			 return std::get<1>(a) > std::get<1>(b);
		 });
}

void Algorithms::handleUnmetDemandForCustomer(SolutionSecondEchelon &sol_SE, int s, int t, int customerIndex, double unmetDemand)
{
	double unmetDemCost = params.unmetDemandPenalty[customerIndex] * unmetDemand;
	int currentWarehouse = findCurrentWarehouse(sol_SE, s, t, customerIndex);
	// cout << "currentWarehouse for customer " << customerIndex + params.numWarehouses << " = " << currentWarehouse + 1 << endl;

	int wareToInsert = -1;
	attemptToInsertCustomerIntoWarehouse(sol_SE, s, t, currentWarehouse, customerIndex, unmetDemand, unmetDemCost);

	// vector<vector<int>> WareToCustSortedByDistance = params.getSortedWarehousesByDistance();
	// for (int wareToInsert : WareToCustSortedByDistance[customerIndex])
	// {
	// 	if (wareToInsert != currentWarehouse)
	// 	{
	// 		// cout << "possible warehouse to insert customer " << customerIndex + params.numWarehouses << " into: " << wareToInsert + 1 << endl;
	// 		if (attemptToInsertCustomerIntoWarehouse(sol_SE, s, t, wareToInsert, currentWarehouse, customerIndex, unmetDemand, unmetDemCost))
	// 		{
	// 			break;
	// 		}
	// 	}
	// }
}

int Algorithms::findCurrentWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int customerIndex)
{
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		if (sol_SE.customerAssignmentToWarehouse[s][t][w][customerIndex] == 1)
		{
			return w;
		}
	}
	return -1;
}

void Algorithms::attemptToInsertCustomerIntoWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int currentWarehouse, int customerIndex, double unmetDemand, double unmetDemCost)
{
	// vector<vector<int>> WareToCustSortedByDistance = params.getSortedWarehousesByDistance();
	int wareToInsert = -1;
	int routeToInsert = -1;
	int posToInsert = -1;
	double tempDeliveryQuantity;
	double minCostToInsert = std::numeric_limits<double>::max();
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		if (w != currentWarehouse)
		{
			vector<double> remainingVehicleCapacityWarehouse(params.numVehicles_Warehouse, params.vehicleCapacity_Warehouse);
			double remainingWarehouseCapacity = params.storageCapacity_Warehouse[w];

			updateRemainingCapacities(sol_SE, s, t, w, remainingVehicleCapacityWarehouse, remainingWarehouseCapacity);

			auto maxIt = std::max_element(remainingVehicleCapacityWarehouse.begin(), remainingVehicleCapacityWarehouse.end());
			if (maxIt != remainingVehicleCapacityWarehouse.end() && *maxIt > 0.0 && remainingWarehouseCapacity > 0.0)
			{
				int routeToInsert_temp = -1;
				int posToInsert_temp = -1;
				double minCostToInsert_temp = std::numeric_limits<double>::max();

				findBestInsertionPosition(sol_SE, s, t, w, customerIndex, remainingVehicleCapacityWarehouse, remainingWarehouseCapacity, routeToInsert_temp, posToInsert_temp, minCostToInsert_temp);

				if ((minCostToInsert_temp < (minCostToInsert - 1e-4)) && routeToInsert != -1)
				{
					wareToInsert = w;
					routeToInsert = routeToInsert_temp;
					posToInsert = posToInsert_temp;
					minCostToInsert = minCostToInsert_temp;
					tempDeliveryQuantity = std::min({params.demand[customerIndex][t][s], remainingVehicleCapacityWarehouse[routeToInsert], remainingWarehouseCapacity});
				}
			}
		}
	}
	if (wareToInsert != -1)
	{
		applyInsertion(sol_SE, s, t, wareToInsert, currentWarehouse, customerIndex, routeToInsert, posToInsert, tempDeliveryQuantity);
	}
}

void Algorithms::updateRemainingCapacities(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity)
{
	for (int r = 0; r < params.numVehicles_Warehouse; ++r)
	{
		if (!sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][r].empty())
		{
			for (auto it = sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][r].begin() + 1; it != sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][r].end() - 1; ++it)
			{
				remainingVehicleCapacityWarehouse[r] -= sol_SE.deliveryQuantityToCustomer[*it - params.numWarehouses][t][s];
				remainingWarehouseCapacity -= sol_SE.deliveryQuantityToCustomer[*it - params.numWarehouses][t][s];
			}
		}
	}
}

void Algorithms::findBestInsertionPosition(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, int customerIndex, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity, int &routeToInsert, int &posToInsert, double &minCostToInsert)
{
	for (int r = 0; r < params.numVehicles_Warehouse; ++r)
	{
		if (remainingVehicleCapacityWarehouse[r] >= params.demand[customerIndex][t][s] && remainingWarehouseCapacity >= params.demand[customerIndex][t][s])
		{
			if (sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][r].empty())
			{
				double costToInsert = 2 * params.transportationCost_SecondEchelon[wareToInsert][customerIndex + params.numWarehouses];
				if (costToInsert < minCostToInsert)
				{
					posToInsert = 1;
					minCostToInsert = costToInsert;
					routeToInsert = r;
				}
			}
			else
			{
				std::pair<int, double> minInsertionCostResult = minInsertionCost(sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][r], customerIndex + params.numWarehouses);
				double costToInsert = minInsertionCostResult.second;

				if (costToInsert < minCostToInsert)
				{
					posToInsert = minInsertionCostResult.first;
					minCostToInsert = costToInsert;
					routeToInsert = r;
				}
			}
		}
	}
}

void Algorithms::applyInsertion(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, int currentWarehouse, int customerIndex, int routeToInsert, int posToInsert, double tempDeliveryQuantity)
{
	sol_SE.customerAssignmentToWarehouse[s][t][currentWarehouse][customerIndex] = 0;
	// cout << "Inserting customer " << customerIndex + params.numWarehouses << " from warehouse " << currentWarehouse << " into warehouse " << wareToInsert << " in Period " << t + 1 << " and Scenario " << s + 1 << endl;
	removeCustomerFromCurrentRoute(sol_SE, s, t, currentWarehouse, customerIndex);
	// cout << "CATW[" << s + 1 << "][" << t + 1 << "][" << currentWarehouse + 1 << "][" << customerIndex << "] = 0" << endl;

	if (sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].empty())
	{
		sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].push_back(wareToInsert);
		sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].push_back(customerIndex + params.numWarehouses);
		sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].push_back(wareToInsert);
	}
	else
	{
		sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].insert(sol_SE.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].begin() + posToInsert, customerIndex + params.numWarehouses);
	}
	sol_SE.deliveryQuantityToCustomer[customerIndex][t][s] = tempDeliveryQuantity;
	sol_SE.customerAssignmentToWarehouse[s][t][wareToInsert][customerIndex] = 1;
}

void Algorithms::removeCustomerFromCurrentRoute(SolutionSecondEchelon &sol_SE, int s, int t, int currentWarehouse, int customerIndex)
{
	for (auto &route : sol_SE.routesWarehouseToCustomer[s][currentWarehouse][t])
	{
		auto it = std::find(route.begin(), route.end(), customerIndex + params.numWarehouses);
		if (it != route.end())
		{
			route.erase(it);
			if (route.size() == 2)
			{
				route.clear();
			}

			break;
		}
	}
}

std::pair<int, double> Algorithms::minInsertionCost(const vector<int> &routesPeriod, int i)
{
	double minInsertionCost = std::numeric_limits<double>::max();
	int minInsertionPos = -1;

	for (int pos = 1; pos < routesPeriod.size(); ++pos)
	{
		double insertionCost = params.transportationCost_SecondEchelon[routesPeriod[pos - 1]][i] +
							   params.transportationCost_SecondEchelon[i][routesPeriod[pos]] -
							   params.transportationCost_SecondEchelon[routesPeriod[pos - 1]][routesPeriod[pos]];
		if (insertionCost < minInsertionCost)
		{
			minInsertionCost = insertionCost;
			minInsertionPos = pos;
		}
	}

	return std::make_pair(minInsertionPos, minInsertionCost);
}

bool Algorithms::solveRestrictedProblemAndFinalize(SolutionFirstEchelon &sol_FE, SolutionSecondEchelon &sol_SE)
{
	SolutionWarmStart sol_WarmStart;
	sol_WarmStart.customerAssignmentToWarehouse_WarmStart = sol_SE.customerAssignmentToWarehouse;
	sol_WarmStart.productionSetup_WarmStart = sol_FE.productionSetup;
	sol_WarmStart.routesPlantToWarehouse_WarmStart = sol_FE.routesPlantToWarehouse;
	sol_WarmStart.routesWarehouseToCustomer_WarmStart = sol_SE.routesWarehouseToCustomer;

	RS2EPRP rs2eprp(params, sol_WarmStart);
	if (!rs2eprp.Solve())
	{
		return false;
	}
	sol_FE = rs2eprp.getSolutionFE();
	sol_SE = rs2eprp.getSolutionSE();

	return true;
}

void Algorithms::printSolution()
{
	cout << "-------------------------------------------------------------------" << endl;
	cout << "Best Solution:" << endl;
	cout << "Setup Cost : " << sol_FE_incumbent.setupCost << endl;
	cout << "Production Cost : " << sol_FE_incumbent.productionCost << endl;
	cout << "Holding Cost Plant : " << sol_FE_incumbent.holdingCostPlant << endl;
	cout << "Transportation Cost Plant to Warehouse : " << sol_FE_incumbent.transportationCostPlantToWarehouse << endl;
	cout << "Holding Cost Warehouse : " << sol_SE_incumbent.holdingCostWarehouse_Avg << endl;
	cout << "Holding Cost Customer : " << sol_SE_incumbent.holdingCostCustomer_Avg << endl;
	cout << "Cost of Unmet Demand : " << sol_SE_incumbent.costOfUnmetDemand_Avg << endl;
	cout << "Transportation Cost Warehouse to Customer : " << sol_SE_incumbent.transportationCostWarehouseToCustomer_Avg << endl;

	cout << "\nObjective value (ILS) Total : " << result_incumbent.objValue_Total << endl;

	// cout << "\nRoutes (Warehouse To Customer):" << endl;
	// for (int s = 0; s < params.numScenarios; ++s)
	// {
	// 	for (int w = 0; w < params.numWarehouses; ++w)
	// 	{
	// 		for (int t = 0; t < params.numPeriods; ++t)
	// 		{
	// 			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
	// 			{
	// 				if (!sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][k].empty())
	// 				{
	// 					cout << "route[" << s + 1 << "][" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] : [";
	// 					for (auto it = sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][k].begin(); it != sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][k].end(); ++it)
	// 					{
	// 						if (it != sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][k].begin())
	// 						{
	// 							cout << " -> ";
	// 						}
	// 						cout << *it;
	// 					}
	// 					cout << "]" << endl;
	// 				}
	// 			}
	// 		}
	// 	}
	// }
}

void Algorithms::organizeSolution()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				// Step 1: Create a vector to store vehicles and their total delivery quantity
				std::vector<std::pair<int, double>> vehicleDeliveryQuantities;

				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					double totalDeliveryQuantity = 0.0;
					// Calculate total delivery quantity for this vehicle across all customers
					for (int i = 0; i < params.numCustomers; ++i)
					{
						int customerIndex = i + params.numWarehouses; // Adjust customer index
						std::vector<int> &route = sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][k];
						if (std::find(route.begin(), route.end(), customerIndex) != route.end())
						{
							totalDeliveryQuantity += sol_SE_incumbent.deliveryQuantityToCustomer[i][t][s];
						}
					}
					vehicleDeliveryQuantities.push_back({k, totalDeliveryQuantity}); // Store vehicle index and its total delivery
				}

				// Step 2: Sort vehicles by total delivery quantity (highest quantity first)
				std::sort(vehicleDeliveryQuantities.begin(), vehicleDeliveryQuantities.end(),
						  [](const std::pair<int, double> &a, const std::pair<int, double> &b)
						  {
							  return a.second > b.second; // Sort in descending order of delivery quantity
						  });

				// Step 3: Reorganize the solution based on the sorted vehicle indices
				std::vector<std::vector<int>> sortedRoutes;
				for (int sorted_k = 0; sorted_k < params.numVehicles_Warehouse; ++sorted_k)
				{
					int original_k = vehicleDeliveryQuantities[sorted_k].first;								 // Get the original vehicle index
					sortedRoutes.push_back(sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][original_k]); // Copy the sorted route
				}

				// Step 4: Replace the original routes with the sorted ones
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][k] = sortedRoutes[k];
				}
			}
		}
	}
}
