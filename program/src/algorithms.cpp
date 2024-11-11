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
	bool shortageAllowed = true;

	solve_Deterministic_HILS(deterministicDemand, shortageAllowed);
	
	SolutionWarmStart_Deterministic warmStart;

	warmStart.productionSetup_WarmStart = sol_FE_incumbent_Deterministic.productionSetup;
	warmStart.routesPlantToWarehouse_WarmStart = sol_FE_incumbent_Deterministic.routesPlantToWarehouse;
	warmStart.routesWarehouseToCustomer_WarmStart = sol_SE_incumbent_Deterministic.routesWarehouseToCustomer;
	warmStart.customerAssignmentToWarehouse_WarmStart = sol_SE_incumbent_Deterministic.customerAssignmentToWarehouse;

	// Solve the EV problem using the Branch-and-Cut
	BC_Deterministic ev_bc(params, deterministicDemand, warmStart, shortageAllowed);
	if (!ev_bc.Solve())
	{
		return EXIT_FAILURE;
	}
	sol_FE_incumbent_Deterministic = ev_bc.getSolutionFE();
	sol_SE_incumbent_Deterministic = ev_bc.getSolutionSE();
	result_incumbent_Deterministic = ev_bc.getResult();

	// Save the solution and check feasibility
	SolutionManager solMgr(params, solAlg);
	solMgr.saveSolution_Deterministic(sol_FE_incumbent_Deterministic, 
									  sol_SE_incumbent_Deterministic, 
									  deterministicDemand,
									  shortageAllowed);

	solMgr.checkFeasibility_Deterministic(shortageAllowed);

	solMgr.saveResultSummary_Deterministic(sol_FE_incumbent_Deterministic, 
										   sol_SE_incumbent_Deterministic, 
										   result_incumbent_Deterministic,
										   shortageAllowed);

	return true;
}

bool Algorithms::solve_WS(int scenarioIndex)
{
	cout << "Start Solving The Expected Value (WS) Problem For the S2EPRP." << endl;
	cout << "Scenario Index: " << scenarioIndex + 1 << endl;
	cout << "-------------------------------------------------------------------" << endl;
	vector<vector<double>> deterministicDemand(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			deterministicDemand[i][t] = params.demand[i][t][scenarioIndex];
		}
	}
	bool shortageAllowed = true;

	solve_Deterministic_HILS(deterministicDemand, shortageAllowed, scenarioIndex);
	
	SolutionWarmStart_Deterministic warmStart;

	warmStart.productionSetup_WarmStart = sol_FE_incumbent_Deterministic.productionSetup;
	warmStart.routesPlantToWarehouse_WarmStart = sol_FE_incumbent_Deterministic.routesPlantToWarehouse;
	warmStart.routesWarehouseToCustomer_WarmStart = sol_SE_incumbent_Deterministic.routesWarehouseToCustomer;
	warmStart.customerAssignmentToWarehouse_WarmStart = sol_SE_incumbent_Deterministic.customerAssignmentToWarehouse;

	// Solve the EV problem using the Branch-and-Cut
	BC_Deterministic ev_bc(params, deterministicDemand, warmStart, shortageAllowed);
	if (!ev_bc.Solve())
	{
		return EXIT_FAILURE;
	}
	sol_FE_incumbent_Deterministic = ev_bc.getSolutionFE();
	sol_SE_incumbent_Deterministic = ev_bc.getSolutionSE();
	result_incumbent_Deterministic = ev_bc.getResult();

	// Save the solution and check feasibility
	SolutionManager solMgr(params, solAlg);
	solMgr.saveSolution_Deterministic(sol_FE_incumbent_Deterministic, 
									  sol_SE_incumbent_Deterministic, 
									  deterministicDemand,
									  shortageAllowed,
									  scenarioIndex);

	solMgr.checkFeasibility_Deterministic(shortageAllowed, scenarioIndex);

	solMgr.saveResultSummary_Deterministic(sol_FE_incumbent_Deterministic, 
										   sol_SE_incumbent_Deterministic, 
										   result_incumbent_Deterministic,
										   shortageAllowed,
										   scenarioIndex);

	return true;
}

bool Algorithms::solve_Deterministic_HILS(const vector<vector<double>> &deterministicDemand, bool shortageAllowed, int scenarioIndex)
{
	cout << "Start Solving The Deterministic Two-Echelon Production Routing Problem With Hybrid-ILS." << endl;
	cout << "-------------------------------------------------------------------" << endl;

	SolutionFirstEchelon sol_FE;
	SolutionSecondEchelon_Deterministic sol_SE;
	Result result_temp;

	result_incumbent_Deterministic.objValue_Total = std::numeric_limits<double>::max();
	result_incumbent_Deterministic.totalCPUTime = 0.0;

	auto elapsedTime = 0.0;
	auto currentTime = std::chrono::high_resolution_clock::now();
	auto startTime = std::chrono::high_resolution_clock::now();

	// -----------------------------------------------------------------------------------------------------------------

	// Solve the first-echelon problem
	if (!solveFirstEchelon_Deterministic(sol_FE, deterministicDemand, shortageAllowed))
	{
		return EXIT_FAILURE;
	}

	// Run ILS for the second-echelon problem
	if (!runILSForSecondEchelon_Deterministic(sol_FE, sol_SE, result_temp, deterministicDemand, shortageAllowed))
	{
		return EXIT_FAILURE;
	}

	update_incumbent_Deterministic(sol_FE, sol_SE, result_temp);
	cout << "Initial Phase is finished." << endl;
	printSolution_Deterministic();

	// Handle unmet demand and optimize routes
	double timeLimit = 3600.0;
	const int maxIteration = 20;

	int iter = 0;
	while (iter < maxIteration && elapsedTime < timeLimit)
	{
		cout << "-----------------------------------------------------------------" << endl;
		cout << "Solving The Deterministic Problem With Hybrid-ILS. Iteration: " << iter + 1 << endl;
		cout << "-----------------------------------------------------------------" << endl;

		sol_FE = sol_FE_incumbent_Deterministic;
		sol_SE = sol_SE_incumbent_Deterministic;

		cout << "Rearrange customer-warehouse assignment..." << endl;
		if (shortageAllowed)
		{
			optimizeUnmetDemandAndRoutes_Deterministic(sol_SE, deterministicDemand);
		}

		// Solve the restricted problem and finalize the solution
		if (!solveRestrictedProblemAndFinalize_Deterministic(sol_FE, sol_SE, deterministicDemand, shortageAllowed))
		{
			return EXIT_FAILURE;
		}

		if (!runILSForSecondEchelon_Deterministic(sol_FE, sol_SE, result_temp, deterministicDemand, shortageAllowed))
		{
			return EXIT_FAILURE;
		}

		update_incumbent_Deterministic(sol_FE, sol_SE, result_temp);
		printSolution_Deterministic();

		currentTime = std::chrono::high_resolution_clock::now();
		elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Computation Time (Hybrid-ILS) after iteration: " << iter + 1 << " = " << elapsedTime << " seconds" << endl;
		iter++;
	}

	result_incumbent_Deterministic.totalCPUTime = elapsedTime;
	cout << "\nTotal Computation Time (Hybrid-ILS): " << result_incumbent_Deterministic.totalCPUTime << " seconds" << endl;

	cout << "Final Phase is finished." << endl;
	cout << "-------------------------------------------------------------------" << endl;
	organizeSolution_Deterministic();
	printSolution_Deterministic();

	return true;
}

bool Algorithms::solveFirstEchelon_Deterministic(SolutionFirstEchelon &solFE, 
												 const vector<vector<double>> &deterministicDemand, 
												 bool shortageAllowed)
{
	cout << "Solve The First-Echelon Problem For the EV Problem" << endl;

	MWPRP_FE_Deterministic mwprp_fe_det(params, deterministicDemand, shortageAllowed);
	if (!mwprp_fe_det.Solve())
	{
		return false;
	}
	solFE = mwprp_fe_det.getSolutionFE();
	return true;
}

bool Algorithms::runILSForSecondEchelon_Deterministic(SolutionFirstEchelon &solFE_current, 
										   SolutionSecondEchelon_Deterministic &solSE_current, 
										   Result &result_current,
										   const vector<vector<double>> &deterministicDemand,
										   bool shortageAllowed)
{
	ILS_SIRP_Deterministic ils_SIRP_det(params, 
									    solFE_current, 
										solSE_current, 
										deterministicDemand, 
										shortageAllowed);
	if (!ils_SIRP_det.run())
	{
		return false;
	}

	solFE_current = ils_SIRP_det.getSolutionFE();
	solSE_current = ils_SIRP_det.getSolutionSE();
	result_current = ils_SIRP_det.getResult();

	return true;
}

bool Algorithms::solveRestrictedProblemAndFinalize_Deterministic(SolutionFirstEchelon &sol_FE, 
																 SolutionSecondEchelon_Deterministic &sol_SE,
																 const vector<vector<double>> &deterministicDemand,
										   						 bool shortageAllowed)
{
	SolutionWarmStart_Deterministic sol_WarmStart;
	sol_WarmStart.customerAssignmentToWarehouse_WarmStart = sol_SE.customerAssignmentToWarehouse;
	sol_WarmStart.productionSetup_WarmStart = sol_FE.productionSetup;
	sol_WarmStart.routesPlantToWarehouse_WarmStart = sol_FE.routesPlantToWarehouse;
	sol_WarmStart.routesWarehouseToCustomer_WarmStart = sol_SE.routesWarehouseToCustomer;

	R2EPRP r2eprp(params, deterministicDemand, shortageAllowed, sol_WarmStart);
	if (!r2eprp.Solve())
	{
		return false;
	}
	sol_FE = r2eprp.getSolutionFE();
	sol_SE = r2eprp.getSolutionSE();

	return true;
}

void Algorithms::update_incumbent_Deterministic(SolutionFirstEchelon &sol_FE, SolutionSecondEchelon_Deterministic &sol_SE, Result &result_temp)
{
	if (result_temp.objValue_Total < result_incumbent_Deterministic.objValue_Total)
	{
		sol_FE_incumbent_Deterministic = sol_FE;
		sol_SE_incumbent_Deterministic = sol_SE;
		result_incumbent_Deterministic = result_temp;

		cout << "New Incumbent: " << result_incumbent_Deterministic.objValue_Total << endl;
	}
}

void Algorithms::printSolution_Deterministic()
{
	cout << "-------------------------------------------------------------------" << endl;
	cout << "Best Solution:" << endl;
	cout << "Setup Cost : " << sol_FE_incumbent_Deterministic.setupCost << endl;
	cout << "Production Cost : " << sol_FE_incumbent_Deterministic.productionCost << endl;
	cout << "Holding Cost Plant : " << sol_FE_incumbent_Deterministic.holdingCostPlant << endl;
	cout << "Transportation Cost Plant to Warehouse : " << sol_FE_incumbent_Deterministic.transportationCostPlantToWarehouse << endl;
	cout << "Holding Cost Warehouse : " << sol_SE_incumbent_Deterministic.holdingCostWarehouse << endl;
	cout << "Holding Cost Customer : " << sol_SE_incumbent_Deterministic.holdingCostCustomer << endl;
	cout << "Cost of Unmet Demand : " << sol_SE_incumbent_Deterministic.costOfUnmetDemand << endl;
	cout << "Transportation Cost Warehouse to Customer : " << sol_SE_incumbent_Deterministic.transportationCostWarehouseToCustomer << endl;

	cout << "\nObjective value (ILS) Total : " << result_incumbent_Deterministic.objValue_Total << endl;
}

void Algorithms::optimizeUnmetDemandAndRoutes_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, 
														   const vector<vector<double>> &demand)
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		vector<tuple<int, double>> unmetDemand_Descending;
		sortCustomersByUnmetDemand_Deterministic(sol_SE, unmetDemand_Descending, t);

		for (const auto &entry : unmetDemand_Descending)
		{
			int customerIndex = std::get<0>(entry);
			double unmetDemand = std::get<1>(entry);

			if (unmetDemand > 1e-4)
			{
				handleUnmetDemandForCustomer_Deterministic(demand, sol_SE, t, customerIndex, unmetDemand);
			}
		}
	}
}

void Algorithms::sortCustomersByUnmetDemand_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, vector<tuple<int, double>> &unmetDemand_Descending, int t)
{
	for (int i = 0; i < params.numCustomers; ++i)
	{
		unmetDemand_Descending.emplace_back(i, sol_SE.customerUnmetDemand[i][t]);
	}

	sort(unmetDemand_Descending.begin(), unmetDemand_Descending.end(),
		 [](const tuple<int, double> &a, const tuple<int, double> &b)
		 {
			 return std::get<1>(a) > std::get<1>(b);
		 });
}

void Algorithms::handleUnmetDemandForCustomer_Deterministic(const vector<vector<double>> &demand, SolutionSecondEchelon_Deterministic &sol_SE, 
																									int t, int customerIndex, double unmetDemand)
{
	double unmetDemCost = params.unmetDemandPenalty[customerIndex] * unmetDemand;
	int currentWarehouse = findCurrentWarehouse_Deterministic(sol_SE, t, customerIndex);

	int wareToInsert = -1;
	attemptToInsertCustomerIntoWarehouse_Deterministic(demand, sol_SE, t, currentWarehouse, customerIndex, unmetDemand, unmetDemCost);
}

int Algorithms::findCurrentWarehouse_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int customerIndex)
{
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		if (sol_SE.customerAssignmentToWarehouse[t][w][customerIndex] == 1)
		{
			return w;
		}
	}
	return -1;
}

void Algorithms::attemptToInsertCustomerIntoWarehouse_Deterministic(const vector<vector<double>> &demand, SolutionSecondEchelon_Deterministic &sol_SE, 
																	int t, int currentWarehouse, int customerIndex, 
																	double unmetDemand, double unmetDemCost)
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

			updateRemainingCapacities_Deterministic(sol_SE, t, w, remainingVehicleCapacityWarehouse, remainingWarehouseCapacity);

			auto maxIt = std::max_element(remainingVehicleCapacityWarehouse.begin(), remainingVehicleCapacityWarehouse.end());
			if (maxIt != remainingVehicleCapacityWarehouse.end() && *maxIt > 0.0 && remainingWarehouseCapacity > 0.0)
			{
				int routeToInsert_temp = -1;
				int posToInsert_temp = -1;
				double minCostToInsert_temp = std::numeric_limits<double>::max();

				findBestInsertionPosition_Deterministic(demand, sol_SE, t, w, customerIndex, 
									remainingVehicleCapacityWarehouse, remainingWarehouseCapacity, 
									routeToInsert_temp, posToInsert_temp, minCostToInsert_temp);

				if ((minCostToInsert_temp < (minCostToInsert - 1e-4)) && routeToInsert != -1)
				{
					wareToInsert = w;
					routeToInsert = routeToInsert_temp;
					posToInsert = posToInsert_temp;
					minCostToInsert = minCostToInsert_temp;
					tempDeliveryQuantity = std::min({demand[customerIndex][t], remainingVehicleCapacityWarehouse[routeToInsert], remainingWarehouseCapacity});
				}
			}
		}
	}
	if (wareToInsert != -1)
	{
		applyInsertion_Deterministic(sol_SE, t, wareToInsert, currentWarehouse, customerIndex, routeToInsert, posToInsert, tempDeliveryQuantity);
	}
}

void Algorithms::updateRemainingCapacities_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int wareToInsert, 
															vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity)
{
	for (int r = 0; r < params.numVehicles_Warehouse; ++r)
	{
		if (!sol_SE.routesWarehouseToCustomer[wareToInsert][t][r].empty())
		{
			for (auto it = sol_SE.routesWarehouseToCustomer[wareToInsert][t][r].begin() + 1; it != sol_SE.routesWarehouseToCustomer[wareToInsert][t][r].end() - 1; ++it)
			{
				remainingVehicleCapacityWarehouse[r] -= sol_SE.deliveryQuantityToCustomer[*it - params.numWarehouses][t];
				remainingWarehouseCapacity -= sol_SE.deliveryQuantityToCustomer[*it - params.numWarehouses][t];
			}
		}
	}
}

void Algorithms::findBestInsertionPosition_Deterministic(const vector<vector<double>> &demand, SolutionSecondEchelon_Deterministic &sol_SE, 
																int t, int wareToInsert, int customerIndex, 
																vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity, 
																int &routeToInsert, int &posToInsert, double &minCostToInsert)
{
	for (int r = 0; r < params.numVehicles_Warehouse; ++r)
	{
		if (remainingVehicleCapacityWarehouse[r] >= demand[customerIndex][t] && remainingWarehouseCapacity >= demand[customerIndex][t])
		{
			if (sol_SE.routesWarehouseToCustomer[wareToInsert][t][r].empty())
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
				std::pair<int, double> minInsertionCostResult = minInsertionCost(sol_SE.routesWarehouseToCustomer[wareToInsert][t][r], customerIndex + params.numWarehouses);
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

void Algorithms::applyInsertion_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int wareToInsert, int currentWarehouse, 
																int customerIndex, int routeToInsert, int posToInsert, double tempDeliveryQuantity)
{
	sol_SE.customerAssignmentToWarehouse[t][currentWarehouse][customerIndex] = 0;
	// cout << "Inserting customer " << customerIndex + params.numWarehouses << " from warehouse " << currentWarehouse << " into warehouse " << wareToInsert << " in Period " << t + 1 << " and Scenario " << s + 1 << endl;
	removeCustomerFromCurrentRoute_Deterministic(sol_SE, t, currentWarehouse, customerIndex);
	// cout << "CATW[" << s + 1 << "][" << t + 1 << "][" << currentWarehouse + 1 << "][" << customerIndex << "] = 0" << endl;

	if (sol_SE.routesWarehouseToCustomer[wareToInsert][t][routeToInsert].empty())
	{
		sol_SE.routesWarehouseToCustomer[wareToInsert][t][routeToInsert].push_back(wareToInsert);
		sol_SE.routesWarehouseToCustomer[wareToInsert][t][routeToInsert].push_back(customerIndex + params.numWarehouses);
		sol_SE.routesWarehouseToCustomer[wareToInsert][t][routeToInsert].push_back(wareToInsert);
	}
	else
	{
		sol_SE.routesWarehouseToCustomer[wareToInsert][t][routeToInsert].insert(sol_SE.routesWarehouseToCustomer[wareToInsert][t][routeToInsert].begin() + posToInsert, customerIndex + params.numWarehouses);
	}
	sol_SE.deliveryQuantityToCustomer[customerIndex][t] = tempDeliveryQuantity;
	sol_SE.customerAssignmentToWarehouse[t][wareToInsert][customerIndex] = 1;
}

void Algorithms::removeCustomerFromCurrentRoute_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int currentWarehouse, int customerIndex)
{
	for (auto &route : sol_SE.routesWarehouseToCustomer[currentWarehouse][t])
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

void Algorithms::organizeSolution_Deterministic()
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
					std::vector<int> &route = sol_SE_incumbent_Deterministic.routesWarehouseToCustomer[w][t][k];
					if (std::find(route.begin(), route.end(), customerIndex) != route.end())
					{
						totalDeliveryQuantity += sol_SE_incumbent_Deterministic.deliveryQuantityToCustomer[i][t];
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
				sortedRoutes.push_back(sol_SE_incumbent_Deterministic.routesWarehouseToCustomer[w][t][original_k]); // Copy the sorted route
			}

			// Step 4: Replace the original routes with the sorted ones
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				sol_SE_incumbent_Deterministic.routesWarehouseToCustomer[w][t][k] = sortedRoutes[k];
			}
		}
	}
}

bool Algorithms::solve_EEV(int scenarioIndex)
{
	cout << "Start Solving The Expected Value (EEV) Problem For the S2EPRP." << endl;
	cout << "Scenario Index: " << scenarioIndex + 1 << endl;
	cout << "-------------------------------------------------------------------" << endl;
	vector<vector<double>> deterministicDemand(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			deterministicDemand[i][t] = params.demand[i][t][scenarioIndex];
		}
	}
	bool shortageAllowed = true;

	// Read the First Echelon Solution From EV
	SolutionFirstEchelon sol_FE_EV;
	if(!read_SolutionFirstEchelon(sol_FE_EV))
		return false;

	// Solve the EV problem using the HILS
	solve_EEV_HILS(sol_FE_EV, deterministicDemand, scenarioIndex);
	
	SolutionWarmStart_Deterministic warmStart;

	warmStart.routesWarehouseToCustomer_WarmStart = sol_SE_incumbent_Deterministic.routesWarehouseToCustomer;
	warmStart.customerAssignmentToWarehouse_WarmStart = sol_SE_incumbent_Deterministic.customerAssignmentToWarehouse;

	// Solve the EV problem using the Branch-and-Cut
	BC_EEV ev_eev(params, sol_FE_EV, deterministicDemand, warmStart);
	if (!ev_eev.Solve())
	{
		return EXIT_FAILURE;
	}
	sol_SE_incumbent_Deterministic = ev_eev.getSolutionSE();
	result_incumbent_Deterministic = ev_eev.getResult();

	// Save the solution and check feasibility
	SolutionManager solMgr(params, solAlg);
	solMgr.saveSolution_Deterministic(sol_FE_EV, 
									  sol_SE_incumbent_Deterministic, 
									  deterministicDemand,
									  shortageAllowed,
									  scenarioIndex);

	solMgr.checkFeasibility_Deterministic(shortageAllowed, scenarioIndex);

	solMgr.saveResultSummary_Deterministic(sol_FE_EV, 
										   sol_SE_incumbent_Deterministic, 
										   result_incumbent_Deterministic,
										   shortageAllowed,
										   scenarioIndex);

	return true;
}

bool Algorithms::read_SolutionFirstEchelon(SolutionFirstEchelon &sol_FE_EV)
{
	cout << "Reading Solution For EV..." << endl;
	string directory;
    string filename;
	directory = "../Results/Solutions/SolEvaluation/EV/" 
								+ params.probabilityFunction
								+ "/S" + std::to_string(params.numScenarios);

	// Construct the filename
	filename = "Sol_S2EPRPAR_EV_" + params.probabilityFunction
						+ "_" + params.instance
						+ "_S" + std::to_string(params.numScenarios)
						+ "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%"
						+ "_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
	string solutionFileName = directory + "/" + filename;
 

    std::ifstream file(solutionFileName);
    if (file.is_open())
    {
        int intValue;
        double doubleValue;
        string strValue;

        // Read Parameters
        file >> intValue; // numNodes_Total;
        file >> intValue; // params.numWarehouses;
        file >> intValue; // numCustomers;
        file >> intValue; // numVehicles_Plant;
        file >> intValue; // numVehicles_Warehouse;
        file >> intValue; // numPeriods;
        file >> intValue; // numScenarios;

        file >> doubleValue; // uncertaintyRange;
        file >> strValue;    // ProbabilityFunction;
        file >> doubleValue; // unmetDemandPenaltyCoeff;

        file >> doubleValue; // unitProdCost;
        file >> doubleValue; // setupCost;
        file >> doubleValue; // prodCapacity;
        file >> doubleValue; // vehicleCapacity_Plant;
        file >> doubleValue; // vehicleCapacity_Warehouse;

        file >> doubleValue; // Plant X Coordinates
        file >> doubleValue; // Plant Y Coordinates

        for (int w = 0; w < params.numWarehouses; w++)
        {
            file >> doubleValue; // Warehouse X Coordinates
        }
        for (int w = 0; w < params.numWarehouses; w++)
        {
            file >> doubleValue; // Warehouse Y Coordinates
        }
        for (int i = 0; i < params.numCustomers; i++)
        {
            file >> doubleValue; // Customers X Coordinates
        }
        for (int i = 0; i < params.numCustomers; i++)
        {
            file >> doubleValue; // Customers Y Coordinates
        }

        file >> doubleValue; // unit holding cost plant
        for (int w = 0; w < params.numWarehouses; w++)
        {
            file >> doubleValue; // holding cost warehouse
        }
        for (int i = 0; i < params.numCustomers; i++)
        {
            file >> doubleValue; // holding cost customer
        }

        file >> doubleValue; // storage capacity plant
        for (int w = 0; w < params.numWarehouses; w++)
        {
            file >> doubleValue; // storage capacity warehouse
        }
        for (int i = 0; i < params.numCustomers; i++)
        {
            file >> doubleValue; // storage capacity customer
        }

        file >> doubleValue; // initial inventory plant
        for (int w = 0; w < params.numWarehouses; w++)
        {
            file >> doubleValue; // initial inventory warehouse
        }
        for (int i = 0; i < params.numCustomers; i++)
        {
            file >> doubleValue; // initial inventory customer
        }

        for (int i = 0; i < params.numCustomers; i++)
        {
            file >> doubleValue; // unmet demand penalty
        }

		for (int t = 0; t < params.numPeriods; t++)
		{
			for (int i = 0; i < params.numCustomers; i++)
			{
				file >> doubleValue; //Deterministic Demand
			}
		}

        for (int i = 0; i < params.numNodes_FirstEchelon; i++)
        {
            for (int j = 0; j < params.numNodes_FirstEchelon; j++)
            {
                file >> doubleValue; // Transportation Cost - First Echelon
            }
        }

        for (int i = 0; i < params.numNodes_SecondEchelon; i++)
        {
            for (int j = 0; j < params.numNodes_SecondEchelon; j++)
            {
                file >> doubleValue; // Transportation Cost - Second Echelon
            }
        }
 
        // Read Solutions
        sol_FE_EV.productionSetup.resize(params.numPeriods);
		sol_FE_EV.productionQuantity.resize(params.numPeriods);
		sol_FE_EV.plantInventory.resize(params.numPeriods);
        sol_FE_EV.routesPlantToWarehouse.resize(params.numPeriods, vector<vector<int>>(params.numVehicles_Plant, vector<int>()));
		sol_FE_EV.deliveryQuantityToWarehouse.resize(params.numWarehouses, vector<double>(params.numPeriods));

        // Read Solutions
		for (int t = 0; t < params.numPeriods; t++)
		{
			for (int w = 0; w < params.numWarehouses; w++)
			{
				for (int i = 0; i < params.numCustomers; i++)
				{
					file >> doubleValue; //Customer Assignment To Warehouse
				}
			}
		}

        for (int t = 0; t < params.numPeriods; t++)
        {
            file >> sol_FE_EV.productionSetup[t];
        }

        for (int t = 0; t < params.numPeriods; t++)
        {
            file >> sol_FE_EV.productionQuantity[t];
        }

        for (int t = 0; t < params.numPeriods; t++)
        {
            file >> sol_FE_EV.plantInventory[t];
        }

        for (int w = 0; w < params.numWarehouses; w++)
        {
            for (int t = 0; t < params.numPeriods; t++)
            {
				file >> doubleValue;
            }
        }

        for (int i = 0; i < params.numCustomers; i++)
        {
            for (int t = 0; t < params.numPeriods; t++)
            {
				file >> doubleValue;
            }
        }

        for (int i = 0; i < params.numCustomers; i++)
        {
            for (int t = 0; t < params.numPeriods; t++)
            {
				file >> doubleValue;
            }
        }

        string line_One;
        while (std::getline(file, line_One))
        {
            if (line_One == "endRoutesPlantToWarehouse")
                break;

            std::istringstream iss(line_One);
            int t_index, k_index;
            char colon;
            if (!(iss >> t_index >> k_index >> colon) || colon != ':')
            {
                cerr << "Error parsing line: " << line_One << std::endl;
                continue;
            }

            int node;
            vector<int> route;
            while (iss >> node)
            {
                route.push_back(node);
            }

            // Assign the route to the appropriate vehicle
            sol_FE_EV.routesPlantToWarehouse[t_index][k_index] = route;
        }

        for (int w = 0; w < params.numWarehouses; w++)
        {
            for (int t = 0; t < params.numPeriods; t++)
            {
                file >> sol_FE_EV.deliveryQuantityToWarehouse[w][t];
            }
        }

        string line_Two;
        while (std::getline(file, line_Two))
        {
            if (line_Two == "endRoutesWarehouseToCustomer")
                break;

            std::istringstream iss(line_Two);
            int w_index, t_index, k_index;
            char colon;
            if (!(iss >> w_index >> t_index >> k_index >> colon) || colon != ':')
            {
                cerr << "Error parsing line: " << line_Two << std::endl;
                continue;
            }

            int node;
            vector<int> route;
            while (iss >> node)
            {
                route.push_back(node);
            }
        }

        for (int i = 0; i < params.numCustomers; i++)
        {
            for (int t = 0; t < params.numPeriods; t++)
            {
				file >> doubleValue;
            }
        }

        file.close();

		for (int t = 0; t < params.numPeriods; ++t){
			sol_FE_EV.setupCost += params.setupCost * sol_FE_EV.productionSetup[t];
			sol_FE_EV.productionCost += params.unitProdCost * sol_FE_EV.productionQuantity[t];
			sol_FE_EV.holdingCostPlant += params.unitHoldingCost_Plant * sol_FE_EV.plantInventory[t];

			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Plant; ++k)
				{
					vector<int> &route = sol_FE_EV.routesPlantToWarehouse[t][k];
					int previousNode = 0;
					for (int j = 1; j < route.size(); ++j)
					{
						int currentNode = route[j];
						sol_FE_EV.transportationCostPlantToWarehouse += params.transportationCost_FirstEchelon[previousNode][currentNode];

						previousNode = currentNode;
					}
				}
			}
		}

        cout << "EV Solution loaded successfully." << endl;
        return true;
    }
    else
    {
        cerr << "EV solution was found!!" << endl;
        sol_FE_EV.clear();
        return false;
    }
}

bool Algorithms::solve_EEV_HILS(const SolutionFirstEchelon &sol_FE_EV, const vector<vector<double>> &deterministicDemand, int scenarioIndex)
{
	cout << "Start Solving The EEV for the Two-Echelon Production Routing Problem With Hybrid-ILS." << endl;
	cout << "Scenario Index: " << scenarioIndex + 1 << endl;
	cout << "-------------------------------------------------------------------" << endl;

	SolutionFirstEchelon sol_FE; 
	SolutionSecondEchelon_Deterministic sol_SE;
	Result result_temp;

	result_incumbent_Deterministic.objValue_Total = std::numeric_limits<double>::max();
	result_incumbent_Deterministic.totalCPUTime = 0.0;

	auto elapsedTime = 0.0;
	auto currentTime = std::chrono::high_resolution_clock::now();
	auto startTime = std::chrono::high_resolution_clock::now();

	// -----------------------------------------------------------------------------------------------------------------
	// Run ILS for the EEV
	if (!runILS_EEV(sol_FE_EV, sol_SE, result_temp, deterministicDemand))
	{
		return EXIT_FAILURE;
	}

	sol_FE = sol_FE_EV;
	update_incumbent_Deterministic(sol_FE, sol_SE, result_temp);
	cout << "Initial Phase is finished." << endl;
	printSolution_Deterministic();

	// Handle unmet demand and optimize routes
	double timeLimit = 3600.0;
	const int maxIteration = 20;

	int iter = 0;
	while (iter < maxIteration && elapsedTime < timeLimit)
	{
		cout << "-----------------------------------------------------------------" << endl;
		cout << "Solving The Deterministic Problem With Hybrid-ILS (EEV). Iteration: " << iter + 1 << endl;
		cout << "-----------------------------------------------------------------" << endl;

		sol_SE = sol_SE_incumbent_Deterministic;

		cout << "Rearrange customer-warehouse assignment..." << endl;
		optimizeUnmetDemandAndRoutes_Deterministic(sol_SE, deterministicDemand);

		// Solve the restricted problem and finalize the solution
		if (!runILS_EEV(sol_FE_EV, sol_SE, result_temp, deterministicDemand))
		{
			return EXIT_FAILURE;
		}

		update_incumbent_Deterministic(sol_FE, sol_SE, result_temp);
		printSolution_Deterministic();

		currentTime = std::chrono::high_resolution_clock::now();
		elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Computation Time (Hybrid-ILS) after iteration: " << iter + 1 << " = " << elapsedTime << " seconds" << endl;
		iter++;
	}

	result_incumbent_Deterministic.totalCPUTime = elapsedTime;
	cout << "\nTotal Computation Time (Hybrid-ILS): " << result_incumbent_Deterministic.totalCPUTime << " seconds" << endl;

	cout << "Final Phase is finished." << endl;
	cout << "-------------------------------------------------------------------" << endl;
	organizeSolution_Deterministic();
	printSolution_Deterministic();

	return true;
}

bool Algorithms::runILS_EEV(const SolutionFirstEchelon &sol_FE_EV, 
								SolutionSecondEchelon_Deterministic &solSE_current, 
								Result &result_current,
								const vector<vector<double>> &deterministicDemand)
{
	ILS_EEV ils_eev(params, 
					sol_FE_EV, 
					solSE_current, 
					deterministicDemand);
	if (!ils_eev.run())
	{
		return false;
	}

	solSE_current = ils_eev.getSolutionSE();
	result_current = ils_eev.getResult();

	return true;
}
