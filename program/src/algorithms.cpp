#include "algorithms.h"

Algorithms::Algorithms(const ParameterSetting &parameters)
	: params(parameters)
{
	std::random_device rd;
	rng = std::mt19937(rd());
}

bool Algorithms::solve_S2EPRP_BC()
{
	if (params.problemType != "S2EPRP-AR" || params.solutionAlgorithm != "BC")
	{
		cerr << "The solution algorithm is not Branch-and-Cut." << endl;
		return EXIT_FAILURE;
	}

	cout << "Start Solving The S2EPRP-AR Using Branch-and-Cut Algorithm." << endl;

	cout << "-------------------------------------------------------------------" << endl;
	SolutionWarmStart warmStartSolution = params.readSolutionWarmStart_Stoch();

	auto elapsedTime = 0.0;
	auto currentTime = std::chrono::high_resolution_clock::now();
	auto startTime = std::chrono::high_resolution_clock::now();

	// Branch and Cut
	S2EPRP_BC s2eprp_bc(params, warmStartSolution);
	if (!s2eprp_bc.Solve())
	{
		return EXIT_FAILURE;
	}
	sol_FE_incumbent = s2eprp_bc.getSolutionFE();
	sol_SE_incumbent = s2eprp_bc.getSolutionSE();
	result_incumbent = s2eprp_bc.getResult();

	currentTime = std::chrono::high_resolution_clock::now();
	elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
	cout << "Computation Time (BC) = " << elapsedTime << " seconds" << endl;

	// -----------------------------------------------------------------------------------------------------------------
	SolutionManager solMgr_OFIter_test(params);
	Result result_OFIter_test = result_incumbent;
	cout << "\nSave Sol for Hyper Parameter Tuning (BC): " << endl;
	result_OFIter_test.totalCPUTime = elapsedTime;
	solMgr_OFIter_test.saveOF_Iter_Deterministic(1, 0, result_OFIter_test, "BC");
	// -----------------------------------------------------------------------------------------------------------------

	// Save the solution and check feasibility
	SolutionManager solMgr_ILS(params);
	solMgr_ILS.saveSolution(sol_FE_incumbent, sol_SE_incumbent);
	solMgr_ILS.checkFeasibility();
	solMgr_ILS.saveResultSummary(sol_FE_incumbent, sol_SE_incumbent, result_incumbent);

	return true;
}

bool Algorithms::solve_S2EPRP_HILS()
{
	if (params.problemType != "S2EPRP-AR" || params.solutionAlgorithm != "Hybrid-ILS")
	{
		cerr << "The solution algorithm is not Hybrid-ILS." << endl;
		return EXIT_FAILURE;
	}

	cout << "Start Solving The S2EPRP-AR With Hybrid-ILS." << endl;
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
	// Construct the initial solution
	// Solve the first-echelon problem
	if (!solveFirstEchelon(sol_FE))
	{
		return EXIT_FAILURE;
	}

	// Run ILS for the second-echelon problem
	if (!runILSForSecondEchelon(sol_FE, sol_SE, result_temp, 0, startTime, true))
	{
		return EXIT_FAILURE;
	}

	update_incumbent(sol_FE, sol_SE, result_temp);
	cout << "Initial Phase is finished." << endl;
	printSolution();
	// Initial Solution is constructed.
	// -----------------------------------------------------------------------------------------------------------------
	double globalBestObjValue = result_incumbent.objValue_Total;
	double localBestObjValue = result_incumbent.objValue_Total;

	// we check whether the improvement is below 0.01% for more than maxNoImprovement iterations
	int num_NoImprovement_local = 0;
	int num_NoImprovement_global = 0;
	double minImprovementPercentage = 1e-2;
	double minImprovement = 1e-4;
	bool stop = false;

	int iter = 0;
	while (!stop)
	{
		cout << "-----------------------------------------------------------------" << endl;
		cout << "Solving The Problem With Hybrid-ILS. Iteration: " << iter + 1 << endl;
		cout << "-----------------------------------------------------------------" << endl;

		bool MergeWarehouses = false;

		if (num_NoImprovement_global % 4 == 0 && num_NoImprovement_global >= 12)
		{
			sol_FE = sol_FE_incumbent;
			sol_SE = sol_SE_incumbent;
			MergeWarehouses = true;
		}

		cout << "Rearrange customer-warehouse assignment..." << endl;
		// optimizeUnmetDemandAndRoutes(sol_SE);
		rearrangeCustomerAssignments(sol_SE);
		
		if (num_NoImprovement_local % 4 == 0 && num_NoImprovement_local != 0)
		{
			MergeWarehouses = true;
		}

		bool mergeSuccess = false;
		if (MergeWarehouses)
		{
			cout << "In each scenario, randomly remove a warehouse from a period..." << endl;
			mergeSuccess = Merge_Warehouses(sol_SE);
			if (!mergeSuccess)
			{
				cout << "Merge failed" << endl;
			}
		}

		// Solve the restricted problem and finalize the solution
		if (!solveRestrictedProblemAndFinalize(sol_FE, sol_SE))
		{
			return EXIT_FAILURE;
		}

		if (!runILSForSecondEchelon(sol_FE, sol_SE, result_temp, iter + 1, startTime, true))
		{
			return EXIT_FAILURE;
		}

		// -----------------------------------------------------------------------------------------------------------------
		cout << "----------------------------------------------------------------" << endl;
		double improvement_percentage_global = ((globalBestObjValue - result_temp.objValue_Total) / globalBestObjValue) * 100;
		cout << "OF Improvement Global: " << improvement_percentage_global << "%" << endl;
		if (improvement_percentage_global > minImprovementPercentage)
		{
			num_NoImprovement_global = 0;
		}
		else
		{
			num_NoImprovement_global++;
		}
		cout << "Number of No Improvement Global: " << num_NoImprovement_global << endl;

		double improvement_percentage_local = ((localBestObjValue - result_temp.objValue_Total) / localBestObjValue) * 100;
		cout << "OF Improvement Local: " << improvement_percentage_local << "%" << endl;
		if (improvement_percentage_local > minImprovementPercentage)
		{
			num_NoImprovement_local = 0;
		}
		else
		{
			num_NoImprovement_local++;
		}
		cout << "Number of No Improvement Local: " << num_NoImprovement_local << endl;
		// -----------------------------------------------------------------------------------------------------------------
		if (mergeSuccess)
		{
			localBestObjValue = result_temp.objValue_Total;
		}
		else if (result_temp.objValue_Total < (localBestObjValue - minImprovement))
		{
			localBestObjValue = result_temp.objValue_Total;
		}

		if (result_temp.objValue_Total < globalBestObjValue - minImprovement)
		{
			update_incumbent(sol_FE, sol_SE, result_temp);
			printSolution();

			globalBestObjValue = result_temp.objValue_Total;
		}
		// -----------------------------------------------------------------------------------------------------------------

		currentTime = std::chrono::high_resolution_clock::now();
		elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Computation Time (Hybrid-ILS) after iteration: " << iter + 1 << " = " << elapsedTime << " seconds" << endl;

		iter++;

		// check stopping criteria
		if (iter >= params.HILS_MaxIteration || elapsedTime >= params.HILS_TimeLimit || num_NoImprovement_global >= params.HILS_MaxNoImprovement)
		{
			stop = true;
		}
	}

	result_incumbent.totalCPUTime = elapsedTime;
	cout << "\nTotal Computation Time (Hybrid-ILS): " << result_incumbent.totalCPUTime << " seconds" << endl;

	cout << "Final Phase is finished." << endl;
	cout << "-------------------------------------------------------------------" << endl;
	organizeSolution();
	printSolution();

	// Save the solution and check feasibility
	SolutionManager solMgr_ILS(params);
	solMgr_ILS.saveSolution(sol_FE_incumbent, sol_SE_incumbent);
	solMgr_ILS.checkFeasibility();
	solMgr_ILS.saveResultSummary(sol_FE_incumbent, sol_SE_incumbent, result_incumbent);

	for (int t = 0; t < params.numPeriods; t++)
	{
		cout << "y[" << t + 1 << "] = " << sol_FE_incumbent.productionSetup[t] << endl;
	}

	for (int t = 0; t < params.numPeriods; t++)
	{
		cout << "y[" << t + 1 << "] = " << sol_FE_incumbent.productionQuantity[t] << endl;
	}

	return true;
}

bool Algorithms::Merge_Warehouses(SolutionSecondEchelon &sol_SE)
{
	int numMerged = 0;
	for (int s = 0; s < params.numScenarios; ++s)
	{
		// Shuffle periods for random selection
		vector<int> periods(params.numPeriods);
		for (int t = 0; t < params.numPeriods; ++t)
			periods[t] = t;
		std::shuffle(periods.begin(), periods.end(), rng);

		// Iterate through shuffled periods to find a valid one
		for (int t : periods)
		{
			vector<int> activeWarehouses;

			// Identify warehouses with active routes in period t
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					if (!sol_SE.routesWarehouseToCustomer[s][w][t][k].empty())
					{
						activeWarehouses.push_back(w);
						break;
					}
				}
			}

			// If there are at least two warehouses with routes, proceed with merging
			if (activeWarehouses.size() < 2)
				continue;

			// Randomly select a warehouse to omit
			std::shuffle(activeWarehouses.begin(), activeWarehouses.end(), rng);
			int warehouseToOmit = activeWarehouses.front();
			activeWarehouses.erase(activeWarehouses.begin());

			// Gather all customers from the omitted warehouse's routes
			std::set<int> customersToReassign;
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				vector<int> &route = sol_SE.routesWarehouseToCustomer[s][warehouseToOmit][t][k];

				if (route.empty())
					continue;

				for (size_t i = 1; i < route.size() - 1; ++i) // Ignore warehouse at start and end
				{
					customersToReassign.insert(route[i]);
				}
			}

			if (customersToReassign.empty())
				continue;

			// Reassign customers to other warehouses based on min insertion cost
			for (int customer : customersToReassign)
			{
				int bestVehicle = -1;
				int bestWarehouse = -1;
				int bestPosition = -1;
				double bestCost = std::numeric_limits<double>::max();

				for (int newWarehouse : activeWarehouses)
				{
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						vector<int> &route = sol_SE.routesWarehouseToCustomer[s][newWarehouse][t][k];
						if (route.empty())
							continue;
						
						std::pair<int, double> minInsertionCostResult = minInsertionCost(route, customer);
						int insertionPos = minInsertionCostResult.first;
						double insertionCost = minInsertionCostResult.second;

						if (insertionCost < bestCost)
						{
							bestCost = insertionCost;
							bestPosition = insertionPos;
							bestWarehouse = newWarehouse;
							bestVehicle = k;
						}
					}
				}

				// Update assignment if a valid position is found
				if (bestWarehouse != -1)
				{
					vector<int> &route = sol_SE.routesWarehouseToCustomer[s][bestWarehouse][t][bestVehicle];

					if (bestPosition != -1)
					{
						route.insert(route.begin() + bestPosition, customer);
						sol_SE.customerAssignmentToWarehouse[s][t][warehouseToOmit][customer - params.numWarehouses] = 0;
						sol_SE.customerAssignmentToWarehouse[s][t][bestWarehouse][customer - params.numWarehouses] = 1;
					}
				}
			}

			// Clear the omitted warehouse's routes
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				sol_SE.routesWarehouseToCustomer[s][warehouseToOmit][t][k].clear();

			numMerged++;
			break;
		}
	}

	cout << "Number of merged occurrences: " << numMerged << endl;
	if (numMerged == params.numScenarios)
	{
		return true;
	}
	return true;
}

bool Algorithms::solveFirstEchelon(SolutionFirstEchelon &solFE)
{
	cout << "Solve The First-Echelon Problem" << endl;
	MWPRP_FE mwprp_fe(params, false, false);
	if (!mwprp_fe.Solve())
	{
		return false;
	}
	solFE = mwprp_fe.getSolutionFE();
	return true;
}

bool Algorithms::runILSForSecondEchelon(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon &solSE_current, Result &result_current,
										int iter, std::chrono::high_resolution_clock::time_point startTime, bool savePerIterSol)
{
	ILS_SIRP ils_SIRP(params, solFE_current, solSE_current, iter, startTime, savePerIterSol);
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
	sol_FE_incumbent = sol_FE;
	sol_SE_incumbent = sol_SE;
	result_incumbent = result_temp;

	cout << "New Incumbent: " << result_incumbent.objValue_Total << endl;

	for (int t = 0; t < params.numPeriods; t++)
	{
		cout << "y[" << t + 1 << "] = " << sol_FE_incumbent.productionSetup[t] << endl;
	}

	for (int t = 0; t < params.numPeriods; t++)
	{
		cout << "y[" << t + 1 << "] = " << sol_FE_incumbent.productionQuantity[t] << endl;
	}
}

void Algorithms::rearrangeCustomerAssignments(SolutionSecondEchelon &sol_SE)
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
				bool hasUnmetDemand = false;

				int customerIndex = std::get<0>(entry);
				double unmetDemand = std::get<1>(entry);

				if (unmetDemand > 1e-4)
				{
					hasUnmetDemand = true;
					handleUnmetDemandForCustomer(sol_SE, s, t, customerIndex, unmetDemand);
				}
				else
				{
					if (std::get<0>(visitedByWarehouse(sol_SE, s, t, customerIndex)) != -1)
					{
						int currentWarehouse = std::get<0>(visitedByWarehouse(sol_SE, s, t, customerIndex));
						int currentVehicle = std::get<1>(visitedByWarehouse(sol_SE, s, t, customerIndex));
						int currentPosition = std::get<2>(visitedByWarehouse(sol_SE, s, t, customerIndex));

						attemptToInsertCustomerIntoWarehouse(sol_SE, s, t, currentWarehouse, customerIndex, hasUnmetDemand);
					}
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

std::tuple<int, int, int> Algorithms::visitedByWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int i)
{
	int customerIndex = i + params.numWarehouses;

	// Initialize result to (-1, -1, -1) indicating "not found"
	std::tuple<int, int, int> result = std::make_tuple(-1, -1, -1);

	// Loop through all warehouses and vehicles
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			// Get the route for warehouse `w`, period `t`, and vehicle `k`
			vector<int> &route = sol_SE.routesWarehouseToCustomer[s][w][t][k];

			// Check if the customer is in the route
			auto it = std::find(route.begin(), route.end(), customerIndex);
			if (it != route.end())
			{
				// Calculate position in the route
				int position = std::distance(route.begin(), it);

				// Return warehouse, vehicle, and position
				return std::make_tuple(w, k, position);
			}
		}
	}

	// Customer not found: return default (-1, -1, -1)
	return result;
}

void Algorithms::handleUnmetDemandForCustomer(SolutionSecondEchelon &sol_SE, int s, int t, int customerIndex, double unmetDemand)
{
	double unmetDemCost = params.unmetDemandPenalty[customerIndex] * unmetDemand;
	int currentWarehouse = findCurrentWarehouse(sol_SE, s, t, customerIndex);
	// cout << "currentWarehouse for customer " << customerIndex + params.numWarehouses << " = " << currentWarehouse + 1 << endl;

	int wareToInsert = -1;
	attemptToInsertCustomerIntoWarehouse(sol_SE, s, t, currentWarehouse, customerIndex, true);

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

void Algorithms::attemptToInsertCustomerIntoWarehouse(SolutionSecondEchelon &sol_SE, int s, int t, int currentWarehouse, int customerIndex, bool hasUnmetDemand)
{
	if (hasUnmetDemand)
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
					double deliveryQuantity_temp = std::min({params.demand[customerIndex][t][s], remainingVehicleCapacityWarehouse[routeToInsert], remainingWarehouseCapacity});

					if (remainingWarehouseCapacity >= deliveryQuantity_temp)
					{
						findBestInsertionPosition(sol_SE, s, t, w, customerIndex,
												  remainingVehicleCapacityWarehouse, remainingWarehouseCapacity,
												  routeToInsert_temp, posToInsert_temp, minCostToInsert_temp, deliveryQuantity_temp);

						if ((minCostToInsert_temp < (minCostToInsert - 1e-4)) && routeToInsert_temp != -1)
						{
							wareToInsert = w;
							routeToInsert = routeToInsert_temp;
							posToInsert = posToInsert_temp;
							minCostToInsert = minCostToInsert_temp;
							tempDeliveryQuantity = deliveryQuantity_temp;
						}
					}
				}
			}
		}
		if (wareToInsert != -1)
		{
			applyInsertion(sol_SE, s, t, wareToInsert, currentWarehouse, customerIndex, routeToInsert, posToInsert, tempDeliveryQuantity);
		}
	}
	else
	{
		int currentVehicle = std::get<1>(visitedByWarehouse(sol_SE, s, t, customerIndex));
		int currentPosition = std::get<2>(visitedByWarehouse(sol_SE, s, t, customerIndex));

		int previousNode = sol_SE.routesWarehouseToCustomer[s][currentWarehouse][t][currentVehicle][currentPosition - 1];
		int nextNode = sol_SE.routesWarehouseToCustomer[s][currentWarehouse][t][currentVehicle][currentPosition + 1];

		double minCostToInsert = params.transportationCost_SecondEchelon[previousNode][customerIndex + params.numWarehouses] + params.transportationCost_SecondEchelon[customerIndex + params.numWarehouses][nextNode] - params.transportationCost_SecondEchelon[previousNode][nextNode];

		int wareToInsert = -1;
		int routeToInsert = -1;
		int posToInsert = -1;
		double tempDeliveryQuantity;
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (w != currentWarehouse)
			{
				vector<double> remainingVehicleCapacityWarehouse(params.numVehicles_Warehouse, params.vehicleCapacity_Warehouse);
				double remainingWarehouseCapacity = params.storageCapacity_Warehouse[w];

				updateRemainingCapacities(sol_SE, s, t, w, remainingVehicleCapacityWarehouse, remainingWarehouseCapacity);

				auto maxIt = std::max_element(remainingVehicleCapacityWarehouse.begin(), remainingVehicleCapacityWarehouse.end());
				if (maxIt != remainingVehicleCapacityWarehouse.end() && *maxIt > 0.0 && remainingWarehouseCapacity > 1e-2)
				{
					int routeToInsert_temp = -1;
					int posToInsert_temp = -1;
					double minCostToInsert_temp = std::numeric_limits<double>::max();
					double deliveryQuantity_temp = sol_SE.deliveryQuantityToCustomer[customerIndex][t][s];

					if (remainingWarehouseCapacity >= deliveryQuantity_temp)
					{
						findBestInsertionPosition(sol_SE, s, t, w, customerIndex,
												  remainingVehicleCapacityWarehouse, remainingWarehouseCapacity,
												  routeToInsert_temp, posToInsert_temp, minCostToInsert_temp, deliveryQuantity_temp);

						if ((minCostToInsert_temp < (minCostToInsert - 1e-4)) && routeToInsert_temp != -1)
						{
							// cout << "HERE HERE" << endl;
							wareToInsert = w;
							routeToInsert = routeToInsert_temp;
							posToInsert = posToInsert_temp;
							minCostToInsert = minCostToInsert_temp;
							tempDeliveryQuantity = deliveryQuantity_temp;
						}
					}
				}
			}
		}

		if (wareToInsert != -1)
		{
			applyInsertion(sol_SE, s, t, wareToInsert, currentWarehouse, customerIndex, routeToInsert, posToInsert, tempDeliveryQuantity);
		}
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

void Algorithms::findBestInsertionPosition(SolutionSecondEchelon &sol_SE, int s, int t, int wareToInsert, int customerIndex,
										   vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity,
										   int &routeToInsert, int &posToInsert, double &minCostToInsert, double &deliveruQuantity)
{
	for (int r = 0; r < params.numVehicles_Warehouse; ++r)
	{
		if (remainingVehicleCapacityWarehouse[r] >= deliveruQuantity)
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
				// Step 1: Create a vector to store vehicles and their lowest node index
				vector<std::pair<int, int>> vehicleLowestNodeIndex; // {vehicleIndex, lowestNodeIndex}

				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					vector<int> &route = sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][k];
					if (!route.empty())
					{
						// Find the lowest node index in the route
						int lowestNodeIndex = *std::min_element(route.begin() + 1, route.end() - 1);
						vehicleLowestNodeIndex.push_back({k, lowestNodeIndex});
					}
					else
					{
						// If the route is empty, use a high value to push it to the end during sorting
						vehicleLowestNodeIndex.push_back({k, std::numeric_limits<int>::max()});
					}
				}

				// Step 2: Sort vehicles based on the lowest node index (ascending order)
				std::sort(vehicleLowestNodeIndex.begin(), vehicleLowestNodeIndex.end(),
						  [](const std::pair<int, int> &a, const std::pair<int, int> &b)
						  {
							  return a.second < b.second; // Sort in ascending order of lowest node index
						  });

				// Step 3: Reorganize the solution based on the sorted vehicle indices
				vector<vector<int>> sortedRoutes;
				for (const auto &vehicle : vehicleLowestNodeIndex)
				{
					int original_k = vehicle.first; // Get the original vehicle index
					sortedRoutes.push_back(sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][original_k]);
				}

				// Step 4: Replace the original routes with the sorted ones
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][k] = sortedRoutes[k];
				}
			}
		}
	}

	// Turn all customers assignment to warehouses if there is no delivery to the customer
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int i = 0; i < params.numCustomers; ++i)
				{
					if (sol_SE_incumbent.customerAssignmentToWarehouse[s][t][w][i] == 1)
					{
						bool customerVisited = false;
						for (int k = 0; k < params.numVehicles_Warehouse; ++k)
						{
							int customerIndex = i + params.numWarehouses; // Adjust customer index
							vector<int> &route = sol_SE_incumbent.routesWarehouseToCustomer[s][w][t][k];
							if (std::find(route.begin(), route.end(), customerIndex) != route.end())
							{
								customerVisited = true;
								break;
							}
						}

						if (customerVisited == false)
						{
							sol_SE_incumbent.customerAssignmentToWarehouse[s][t][w][i] = 0;
						}
					}
				}
			}
		}
	}
}

// --------------------------------------------------------------------------------------
bool Algorithms::solve_2EPRP()
{
	auto elapsedTime = 0.0;
	auto startTime = std::chrono::high_resolution_clock::now();
	auto currentTime = std::chrono::high_resolution_clock::now();

	cout << "Start Solving The " << params.problemType << endl;
	if (params.problemType == "WS" || params.problemType == "EEV")
	{
		cout << "Scenario Index: " << params.scenarioIndex + 1 << endl;
	}
	cout << "-------------------------------------------------------------------" << endl;

	if (params.problemType == "EEV")
	{
		// Read the First Echelon Solution From EV
		SolutionFirstEchelon sol_FE_EV;
		SolutionSecondEchelon_Deterministic sol_SE_EV;
		if (!read_SolutionEV(sol_FE_EV, sol_SE_EV))
			return false;

		if (params.solutionAlgorithm == "Hybrid-ILS")
		{
			if(!solve_Deterministic_HILS(sol_FE_EV, sol_SE_EV))
			{
				cout << "Error in Solving " << params.problemType << " With " << params.solutionAlgorithm << " Algorithm." << endl;
				return false;
			}
		}
		else if (params.solutionAlgorithm == "BC")
		{
			SolutionWarmStart_Deterministic warmStart;
			if (!params.readSolutionWarmStart_Deter(warmStart))
			{
				cout << "Error in Reading Warm Start Solution for " << params.problemType << endl;
				cout << "Solve the problem without a warm start." << endl;
			}

			// Solve the EEV problem using the Branch-and-Cut
			BC_Deterministic bc_eev(params, warmStart, sol_FE_EV, false, false);
			if(!bc_eev.Solve())
			{
				return EXIT_FAILURE;
			}
			sol_SE_incumbent_Deterministic = bc_eev.getSolutionSE();
			result_incumbent_Deterministic = bc_eev.getResult();

			// Save the solution and check feasibility
			SolutionManager solMgr(params);
			solMgr.saveSolution_Deterministic(sol_FE_EV, sol_SE_incumbent_Deterministic, "BC");
			solMgr.checkFeasibility_Deterministic("BC");
			solMgr.saveResultSummary_Deterministic(sol_FE_EV, sol_SE_incumbent_Deterministic, result_incumbent_Deterministic, "BC");
		}
	}
	else 
	{
		if (params.solutionAlgorithm == "Hybrid-ILS")
		{
			if(!solve_Deterministic_HILS())
			{
				cout << "Error in Solving " << params.problemType << " With " << params.solutionAlgorithm << " Algorithm." << endl;
				return false;
			}
		}
		else if (params.solutionAlgorithm == "BC")
		{
			SolutionWarmStart_Deterministic warmStart;
			if (!params.readSolutionWarmStart_Deter(warmStart))
			{
				cout << "Error in Reading Warm Start Solution for " << params.problemType << endl;
				cout << "Solve the problem without a warm start." << endl;
			}

			// Solve the 2EPRP problem using the Branch-and-Cut
			BC_Deterministic bc_det(params, warmStart, {}, false, false);
			if (!bc_det.Solve())
			{
				return EXIT_FAILURE;
			}
			sol_FE_incumbent_Deterministic = bc_det.getSolutionFE();
			sol_SE_incumbent_Deterministic = bc_det.getSolutionSE();
			result_incumbent_Deterministic = bc_det.getResult();

			// Save the solution and check feasibility
			SolutionManager solMgr(params);
			solMgr.saveSolution_Deterministic(sol_FE_incumbent_Deterministic, sol_SE_incumbent_Deterministic, "BC");
			solMgr.checkFeasibility_Deterministic("BC");
			solMgr.saveResultSummary_Deterministic(sol_FE_incumbent_Deterministic, sol_SE_incumbent_Deterministic, result_incumbent_Deterministic, "BC");
		}
	}

	currentTime = std::chrono::high_resolution_clock::now();
	elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
	cout << "Computation Time (" << params.solutionAlgorithm << ") for " << params.problemType << " = " << elapsedTime << " seconds" << endl;

	return true;
}

bool Algorithms::solve_Deterministic_HILS(const SolutionFirstEchelon &sol_FE_EV, const SolutionSecondEchelon_Deterministic &sol_SE_EV)
{
	cout << "Start Solving The Deterministic Two-Echelon Production Routing Problem With Hybrid-ILS For: " << params.problemType << "." << endl;
	if (params.problemType == "EEV" || params.problemType == "WS")
		cout << "Scenario Index: " << params.scenarioIndex + 1 << endl;
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

	if (params.problemType == "EEV")
	{
		sol_FE = sol_FE_EV;
		sol_SE = sol_SE_EV;
		sol_FE_incumbent_Deterministic = sol_FE_EV;
	} 
	else
	{
		// Solve the first-echelon problem
		if (!solveFirstEchelon_Deterministic(sol_FE))
		{
			return EXIT_FAILURE;
		}
	}

	// Run ILS for the second-echelon problem
	if (!runILS_SE_Deterministic(sol_FE, sol_SE, result_temp, 0, startTime, true))
	{
		return EXIT_FAILURE;
	}

	update_incumbent_Deterministic(sol_FE, sol_SE, result_temp);
	cout << "Initial Phase is finished." << endl;
	printSolution_Deterministic();

	double globalBestObjValue = result_incumbent_Deterministic.objValue_Total;
	double localBestObjValue = result_incumbent_Deterministic.objValue_Total;

	// we check whether the improvement is below 0.01% for more than maxNoImprovement iterations
	int num_NoImprovement_local = 0;
	int num_NoImprovement_global = 0;
	double minImprovementPercentage = 1e-2;
	double minImprovement = 1e-4;
	bool stop = false;

	int iter = 0;
	while (!stop)
	{
		cout << "-----------------------------------------------------------------" << endl;
		cout << "Solving The Deterministic Problem With Hybrid-ILS. Iteration: " << iter + 1 << endl;
		cout << "-----------------------------------------------------------------" << endl;

		bool MergeWarehouses = false;

		if (num_NoImprovement_global % 4 == 0 && num_NoImprovement_global >= 10)
		{
			if (params.problemType != "EEV")
			{
				sol_FE = sol_FE_incumbent_Deterministic;
			}
			sol_SE = sol_SE_incumbent_Deterministic;
			MergeWarehouses = true;
		}

		cout << "Rearrange customer-warehouse assignment..." << endl;
		rearrangeCustomerAssignments_Deterministic(sol_SE);

		
		if (num_NoImprovement_local % 4 == 0 && num_NoImprovement_local != 0)
		{
			MergeWarehouses = true;	
		}

		bool mergeSuccess = false;
		if (MergeWarehouses)
		{
			cout << "Randomly remove a warehouse from a period" << endl;
			mergeSuccess = Merge_Warehouses_Deterministic(sol_SE);
			if (!mergeSuccess)
			{
				cout << "Merge failed" << endl;
			}
		}

		if (params.problemType != "EEV")
		{
			// Solve the restricted problem and finalize the solution
			if (!solveRestrictedProblemAndFinalize_Deterministic(sol_FE, sol_SE))
			{
				return EXIT_FAILURE;
			}
		}

		if (!runILS_SE_Deterministic(sol_FE, sol_SE, result_temp, iter + 1, startTime, true))
		{
			cout << "Current Solution After ILS is Infeasible!!" << endl;
			// return EXIT_FAILURE;
		}

		// -----------------------------------------------------------------------------------------------------------------
		cout << "----------------------------------------------------------------" << endl;
		double improvement_percentage_global = ((globalBestObjValue - result_temp.objValue_Total) / globalBestObjValue) * 100;
		cout << "OF Improvement Global: " << improvement_percentage_global << "%" << endl;
		if (improvement_percentage_global > minImprovementPercentage)
		{
			num_NoImprovement_global = 0;
		}
		else
		{
			num_NoImprovement_global++;
		}
		cout << "Number of No Improvement Global: " << num_NoImprovement_global << endl;

		double improvement_percentage_local = ((localBestObjValue - result_temp.objValue_Total) / localBestObjValue) * 100;
		cout << "OF Improvement Local: " << improvement_percentage_local << "%" << endl;
		if (improvement_percentage_local > minImprovementPercentage)
		{
			num_NoImprovement_local = 0;
		}
		else
		{
			num_NoImprovement_local++;
		}
		cout << "Number of No Improvement Local: " << num_NoImprovement_local << endl;
		// -----------------------------------------------------------------------------------------------------------------
		if (mergeSuccess)
		{
			localBestObjValue = result_temp.objValue_Total;
		}
		else if (result_temp.objValue_Total < (localBestObjValue - minImprovement))
		{
			localBestObjValue = result_temp.objValue_Total;
		}

		if (result_temp.objValue_Total < globalBestObjValue - minImprovement)
		{
			update_incumbent_Deterministic(sol_FE, sol_SE, result_temp);
			printSolution_Deterministic();

			globalBestObjValue = result_temp.objValue_Total;
		}
		// -----------------------------------------------------------------------------------------------------------------

		currentTime = std::chrono::high_resolution_clock::now();
		elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Computation Time (Hybrid-ILS) after iteration: " << iter + 1 << " = " << elapsedTime << " seconds" << endl;

		iter++;

		// check stopping criteria
		if (iter >= params.HILS_MaxIteration || elapsedTime >= params.HILS_TimeLimit || num_NoImprovement_global >= params.HILS_MaxNoImprovement)
		{
			stop = true;
		}
	}

	result_incumbent_Deterministic.totalCPUTime = elapsedTime;
	cout << "\nTotal Computation Time (Hybrid-ILS): " << result_incumbent_Deterministic.totalCPUTime << " seconds" << endl;

	cout << "Final Phase is finished." << endl;
	cout << "-------------------------------------------------------------------" << endl;
	organizeSolution_Deterministic();
	printSolution_Deterministic();

	SolutionManager solMgr(params);
	solMgr.saveSolution_Deterministic(sol_FE_incumbent_Deterministic,
									  sol_SE_incumbent_Deterministic,
									  "Hybrid-ILS");
	solMgr.checkFeasibility_Deterministic("Hybrid-ILS");
	solMgr.saveResultSummary_Deterministic(sol_FE_incumbent_Deterministic,
										   sol_SE_incumbent_Deterministic,
										   result_incumbent_Deterministic,
										   "Hybrid-ILS");

	cout << "\n\n\n\n"
		 << endl;

	return true;
}

bool Algorithms::solveFirstEchelon_Deterministic(SolutionFirstEchelon &solFE)
{
	cout << "Solve The First-Echelon Problem For the Deterministic Problem" << endl;

	MWPRP_FE_Deterministic mwprp_fe_det(params, false, false);
	if (!mwprp_fe_det.Solve())
	{
		return false;
	}
	solFE = mwprp_fe_det.getSolutionFE();
	return true;
}

bool Algorithms::runILS_SE_Deterministic(SolutionFirstEchelon &solFE_current, SolutionSecondEchelon_Deterministic &solSE_current, Result &result_current,
										 int iter, std::chrono::high_resolution_clock::time_point startTime, bool savePerIterSol)
{
	cout << "HHA Iteration: " << iter + 1 << endl;
	ILS_SIRP_Deterministic ils_SIRP_det(params, solFE_current, solSE_current, iter, startTime, savePerIterSol);
	if (!ils_SIRP_det.run())
	{
		return false;
	}

	if (params.problemType != "EEV")
	{
		solFE_current = ils_SIRP_det.getSolutionFE();
	}
	solSE_current = ils_SIRP_det.getSolutionSE();
	result_current = ils_SIRP_det.getResult();

	return true;
}

bool Algorithms::solveRestrictedProblemAndFinalize_Deterministic(SolutionFirstEchelon &sol_FE,
																 SolutionSecondEchelon_Deterministic &sol_SE)
{
	SolutionWarmStart_Deterministic sol_WarmStart;
	sol_WarmStart.customerAssignmentToWarehouse_WarmStart = sol_SE.customerAssignmentToWarehouse;
	sol_WarmStart.productionSetup_WarmStart = sol_FE.productionSetup;
	sol_WarmStart.routesPlantToWarehouse_WarmStart = sol_FE.routesPlantToWarehouse;
	sol_WarmStart.routesWarehouseToCustomer_WarmStart = sol_SE.routesWarehouseToCustomer;

	R2EPRP r2eprp(params, sol_WarmStart);
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

	if (params.problemType != "EEV")
	{
		sol_FE_incumbent_Deterministic = sol_FE;
	}
	sol_SE_incumbent_Deterministic = sol_SE;
	result_incumbent_Deterministic = result_temp;

	cout << "\nNew Incumbent: " << result_incumbent_Deterministic.objValue_Total << endl;
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
	if (params.problemType == "2EPRPCS")
	{
		cout << "Handling Cost Satellite : " << sol_SE_incumbent_Deterministic.handlingCostSatellite << endl;
	}

	cout << "Holding Cost Customer : " << sol_SE_incumbent_Deterministic.holdingCostCustomer << endl;
	cout << "Cost of Unmet Demand : " << sol_SE_incumbent_Deterministic.costOfUnmetDemand << endl;

	cout << "Transportation Cost Warehouse to Customer : " << sol_SE_incumbent_Deterministic.transportationCostWarehouseToCustomer << endl;

	cout << "\nObjective value (ILS) Total : " << result_incumbent_Deterministic.objValue_Total << endl;

	// for (int w = 0; w < params.numWarehouses; ++w)
	// {
	// 	for (int t = 0; t < params.numPeriods; ++t)
	// 	{
	// 		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
	// 		{
	// 			vector<int> &route = sol_SE_incumbent_Deterministic.routesWarehouseToCustomer[w][t][k];
	// 			if (!route.empty())
	// 			{
	// 				cout << "route[" << w << "][" << t << "][" << k << "] : [";
	// 				int previousNode = w;
	// 				for (int j = 1; j < route.size(); ++j)
	// 				{
	// 					int currentNode = route[j];

	// 					// cout << previousNode << " -> " << currentNode << " : " << params.transportationCost_SecondEchelon[previousNode][currentNode] << ", ";
	// 					cout << previousNode << " -> ";

	// 					previousNode = currentNode;
	// 				}
	// 				cout << previousNode << "]" << endl;
	// 			}

	// 		}
	// 	}
	// }
}

void Algorithms::rearrangeCustomerAssignments_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE)
{
	// bool hasUnmetDemand_Overall = false;
	for (int t = 0; t < params.numPeriods; ++t)
	{
		bool hasUnmetDemand = false;

		// Check if unmet demand needs to be handled for specific problem types
		if (!sol_SE.customerUnmetDemand.empty())
		{
			vector<tuple<int, double>> sortedUnmetDemand;
			sortCustomersByUnmetDemand_Deterministic(sol_SE, sortedUnmetDemand, t);

			// Check if there is unmet demand above the threshold
			if (!sortedUnmetDemand.empty() && std::get<1>(sortedUnmetDemand[0]) > 1e-2)
			{
				hasUnmetDemand = true;
				// hasUnmetDemand_Overall = true;

				// Process unmet demand for each customer
				for (const auto &entry : sortedUnmetDemand)
				{
					int customerIndex = std::get<0>(entry);
					double unmetDemand = std::get<1>(entry);

					if (unmetDemand > 1e-4)
					{
						handleUnmetDemandForCustomer_Deterministic(sol_SE, t, customerIndex, unmetDemand);
					}
				}
			}
		}

		if (!hasUnmetDemand)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				// tuple : warehouse, vehicle, position
				if (std::get<0>(visitedByWarehouse_Deterministic(sol_SE, t, i)) != -1)
				{

					int currentWarehouse = std::get<0>(visitedByWarehouse_Deterministic(sol_SE, t, i));
					int currentVehicle = std::get<1>(visitedByWarehouse_Deterministic(sol_SE, t, i));
					int currentPosition = std::get<2>(visitedByWarehouse_Deterministic(sol_SE, t, i));

					attemptToInsertCustomerIntoWarehouse_Deterministic(sol_SE, t, currentWarehouse, i, hasUnmetDemand);
				}
			}
		}
	}

	// if (!hasUnmetDemand_Overall)
	// {
	// cout << "Merge warehouses" << endl;
	// bool mergeSuccess = Merge_Warehouses_Deterministic(sol_SE);
	// if (!mergeSuccess)
	// {
	// 	cout << "Merge failed" << endl;
	// }
	// }
}

bool Algorithms::Merge_Warehouses_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE)
{
	// Shuffle periods for random selection
	vector<int> periods(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
		periods[t] = t;
	std::shuffle(periods.begin(), periods.end(), rng);

	// Iterate through shuffled periods to find a valid one
	for (int t : periods)
	{
		vector<int> activeWarehouses;

		// Identify warehouses with active routes in period t
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				if (!sol_SE.routesWarehouseToCustomer[w][t][k].empty())
				{
					activeWarehouses.push_back(w);
					break;
				}
			}
		}

		// If there are at least two warehouses with routes, proceed with merging
		if (activeWarehouses.size() < 2)
			continue;

		// Randomly select a warehouse to omit
		std::shuffle(activeWarehouses.begin(), activeWarehouses.end(), rng);
		int warehouseToOmit = activeWarehouses.front();
		activeWarehouses.erase(activeWarehouses.begin());

		// Gather all customers from the omitted warehouse's routes
		std::set<int> customersToReassign;
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			vector<int> &route = sol_SE.routesWarehouseToCustomer[warehouseToOmit][t][k];
			if (route.empty())
				continue;

			for (size_t i = 1; i < route.size() - 1; ++i) // Ignore warehouse at start and end
			{
				customersToReassign.insert(route[i]);
			}
		}

		if (customersToReassign.empty())
			continue;

		// Reassign customers to other warehouses based on min insertion cost
		for (int customer : customersToReassign)
		{
			int bestVehicle = -1;
			int bestWarehouse = -1;
			int bestPosition = -1;
			double bestCost = std::numeric_limits<double>::max();

			for (int newWarehouse : activeWarehouses)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					vector<int> &route = sol_SE.routesWarehouseToCustomer[newWarehouse][t][k];
					if (route.empty())
						continue;

					std::pair<int, double> minInsertionCostResult = minInsertionCost(route, customer);
					int insertionPos = minInsertionCostResult.first;
					double insertionCost = minInsertionCostResult.second;

					if (insertionCost < bestCost)
					{
						bestCost = insertionCost;
						bestPosition = insertionPos;
						bestWarehouse = newWarehouse;
						bestVehicle = k;
					}
				}
			}

			// Update assignment if a valid position is found
			if (bestWarehouse != -1)
			{
				vector<int> &route = sol_SE.routesWarehouseToCustomer[bestWarehouse][t][bestVehicle];

				if (bestPosition != -1)
				{
					route.insert(route.begin() + bestPosition, customer);
					sol_SE.customerAssignmentToWarehouse[t][warehouseToOmit][customer - params.numWarehouses] = 0;
					sol_SE.customerAssignmentToWarehouse[t][bestWarehouse][customer - params.numWarehouses] = 1;
				}
			}
		}

		// Clear the omitted warehouse's routes
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			sol_SE.routesWarehouseToCustomer[warehouseToOmit][t][k].clear();

		return true; // Successfully merged a warehouse
	}
	return false; // No valid merge found
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

void Algorithms::handleUnmetDemandForCustomer_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int customerIndex, double unmetDemand)
{
	double unmetDemCost = params.unmetDemandPenalty[customerIndex] * unmetDemand;
	int currentWarehouse = findCurrentWarehouse_Deterministic(sol_SE, t, customerIndex);

	int wareToInsert = -1;
	attemptToInsertCustomerIntoWarehouse_Deterministic(sol_SE, t, currentWarehouse, customerIndex, true);
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

std::tuple<int, int, int> Algorithms::visitedByWarehouse_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int i)
{
	int customerIndex = i + params.numWarehouses;

	// Initialize result to (-1, -1, -1) indicating "not found"
	std::tuple<int, int, int> result = std::make_tuple(-1, -1, -1);

	// Loop through all warehouses and vehicles
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			// Get the route for warehouse `w`, period `t`, and vehicle `k`
			vector<int> &route = sol_SE.routesWarehouseToCustomer[w][t][k];

			// Check if the customer is in the route
			auto it = std::find(route.begin(), route.end(), customerIndex);
			if (it != route.end())
			{
				// Calculate position in the route
				int position = std::distance(route.begin(), it);

				// Return warehouse, vehicle, and position
				return std::make_tuple(w, k, position);
			}
		}
	}

	// Customer not found: return default (-1, -1, -1)
	return result;
}

void Algorithms::attemptToInsertCustomerIntoWarehouse_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int currentWarehouse, int customerIndex, bool hasUnmetDemand)
{

	if (hasUnmetDemand)
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
					double deliveryQuantity_temp = std::min({params.demand_Deterministic[customerIndex][t], remainingVehicleCapacityWarehouse[routeToInsert], remainingWarehouseCapacity});

					if (remainingWarehouseCapacity >= deliveryQuantity_temp)
					{

						findBestInsertionPosition_Deterministic(sol_SE, t, w, customerIndex,
																remainingVehicleCapacityWarehouse, remainingWarehouseCapacity,
																routeToInsert_temp, posToInsert_temp, minCostToInsert_temp, deliveryQuantity_temp);

						if ((minCostToInsert_temp < (minCostToInsert - 1e-4)) && routeToInsert != -1)
						{
							wareToInsert = w;
							routeToInsert = routeToInsert_temp;
							posToInsert = posToInsert_temp;
							minCostToInsert = minCostToInsert_temp;
							tempDeliveryQuantity = deliveryQuantity_temp;
						}
					}
				}
			}
		}

		if (wareToInsert != -1)
		{
			applyInsertion_Deterministic(sol_SE, t, wareToInsert, currentWarehouse, customerIndex, routeToInsert, posToInsert, tempDeliveryQuantity);
		}
	}
	else
	{
		int currentVehicle = std::get<1>(visitedByWarehouse_Deterministic(sol_SE, t, customerIndex));
		int currentPosition = std::get<2>(visitedByWarehouse_Deterministic(sol_SE, t, customerIndex));

		int previousNode = sol_SE.routesWarehouseToCustomer[currentWarehouse][t][currentVehicle][currentPosition - 1];
		int nextNode = sol_SE.routesWarehouseToCustomer[currentWarehouse][t][currentVehicle][currentPosition + 1];

		double minCostToInsert = params.transportationCost_SecondEchelon[previousNode][customerIndex + params.numWarehouses] + params.transportationCost_SecondEchelon[customerIndex + params.numWarehouses][nextNode] - params.transportationCost_SecondEchelon[previousNode][nextNode];
		// cout << "Period : " << t << endl;
		// cout << "Current Node: " << customerIndex + params.numWarehouses << " and Current Warehouse: " << currentWarehouse << endl;
		// cout << "Previous Node: " << previousNode << ", Next Node: " << nextNode << ", Min Cost: " << minCostToInsert << endl;

		// vector<int> &route = sol_SE.routesWarehouseToCustomer[currentWarehouse][t][currentVehicle];
		// if (!route.empty())
		// {
		// 	cout << "current route[" << currentWarehouse << "][" << t << "][" << currentVehicle << "] : [";
		// 	int previousNode = currentWarehouse;
		// 	for (int j = 1; j < route.size(); ++j)
		// 	{
		// 		int currentNode = route[j];

		// 		// cout << previousNode << " -> " << currentNode << " : " << params.transportationCost_SecondEchelon[previousNode][currentNode] << ", ";
		// 		cout << previousNode << " -> ";

		// 		previousNode = currentNode;
		// 	}
		// 	cout << previousNode << "]" << endl;
		// }

		int wareToInsert = -1;
		int routeToInsert = -1;
		int posToInsert = -1;
		double tempDeliveryQuantity;
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (w != currentWarehouse)
			{
				vector<double> remainingVehicleCapacityWarehouse(params.numVehicles_Warehouse, params.vehicleCapacity_Warehouse);
				double remainingWarehouseCapacity = params.storageCapacity_Warehouse[w];

				updateRemainingCapacities_Deterministic(sol_SE, t, w, remainingVehicleCapacityWarehouse, remainingWarehouseCapacity);

				auto maxIt = std::max_element(remainingVehicleCapacityWarehouse.begin(), remainingVehicleCapacityWarehouse.end());
				if (maxIt != remainingVehicleCapacityWarehouse.end() && *maxIt > 0.0 && remainingWarehouseCapacity > 1e-2)
				{
					int routeToInsert_temp = -1;
					int posToInsert_temp = -1;
					double minCostToInsert_temp = std::numeric_limits<double>::max();
					double deliveryQuantity_temp = sol_SE.deliveryQuantityToCustomer[customerIndex][t];

					if (remainingWarehouseCapacity >= deliveryQuantity_temp)
					{
						findBestInsertionPosition_Deterministic(sol_SE, t, w, customerIndex,
																remainingVehicleCapacityWarehouse, remainingWarehouseCapacity,
																routeToInsert_temp, posToInsert_temp, minCostToInsert_temp, deliveryQuantity_temp);

						// cout << "New Warehouse To Insert: " << w << ", New Route To Insert: " << routeToInsert_temp;
						// 	cout << ", New Position To Insert: " << posToInsert_temp << ", New Min Cost To Insert: " << minCostToInsert_temp << endl;

						// if (minCostToInsert_temp < (minCostToInsert - 1e-4))
						// {
						// 	cout << "previous min cost: " << minCostToInsert << endl;
						// 	cout << "new min cost: " << minCostToInsert_temp << endl;
						// }

						if ((minCostToInsert_temp < (minCostToInsert - 1e-4)) && routeToInsert_temp != -1)
						{
							// cout << "HERE HERE" << endl;
							wareToInsert = w;
							routeToInsert = routeToInsert_temp;
							posToInsert = posToInsert_temp;
							minCostToInsert = minCostToInsert_temp;
							tempDeliveryQuantity = deliveryQuantity_temp;
							// cout << "New Warehouse To Insert: " << wareToInsert << ", New Route To Insert: " << routeToInsert;
							// cout << ", New Position To Insert: " << posToInsert << ", New Min Cost To Insert: " << minCostToInsert << endl;
						}
					}
				}
			}
		}

		if (wareToInsert != -1)
		{
			// cout << "New minCostToInsert = " << minCostToInsert << endl;
			// cout << "Reallocation For With No Unmet Demand\n" << endl;
			applyInsertion_Deterministic(sol_SE, t, wareToInsert, currentWarehouse, customerIndex, routeToInsert, posToInsert, tempDeliveryQuantity);

			// vector<int> &route_current = sol_SE.routesWarehouseToCustomer[currentWarehouse][t][currentVehicle];
			// if (!route_current.empty())
			// {
			// 	cout << "current route[" << currentWarehouse << "][" << t << "][" << currentVehicle << "] : [";
			// 	int previousNode = currentWarehouse;
			// 	for (int j = 1; j < route_current.size(); ++j)
			// 	{
			// 		int currentNode = route_current[j];

			// 		// cout << previousNode << " -> " << currentNode << " : " << params.transportationCost_SecondEchelon[previousNode][currentNode] << ", ";
			// 		cout << previousNode << " -> ";

			// 		previousNode = currentNode;
			// 	}
			// 	cout << previousNode << "]" << endl;
			// }

			// vector<int> &route_toInsert = sol_SE.routesWarehouseToCustomer[wareToInsert][t][routeToInsert];
			// if (!route_toInsert.empty())
			// {
			// 	cout << "current route[" << wareToInsert << "][" << t << "][" << routeToInsert << "] : [";
			// 	int previousNode = wareToInsert;
			// 	for (int j = 1; j < route_toInsert.size(); ++j)
			// 	{
			// 		int currentNode = route_toInsert[j];

			// 		// cout << previousNode << " -> " << currentNode << " : " << params.transportationCost_SecondEchelon[previousNode][currentNode] << ", ";
			// 		cout << previousNode << " -> ";

			// 		previousNode = currentNode;
			// 	}
			// 	cout << previousNode << "]" << endl;
			// }
		}
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

void Algorithms::findBestInsertionPosition_Deterministic(SolutionSecondEchelon_Deterministic &sol_SE, int t, int wareToInsert, int customerIndex,
														 vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity,
														 int &routeToInsert, int &posToInsert, double &minCostToInsert, double &deliveryQuantity)
{
	for (int r = 0; r < params.numVehicles_Warehouse; ++r)
	{
		if (remainingVehicleCapacityWarehouse[r] >= deliveryQuantity)
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
			// Step 1: Create a vector to store vehicles and their lowest node index
			vector<std::pair<int, int>> vehicleLowestNodeIndex; // {vehicleIndex, lowestNodeIndex}

			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				vector<int> &route = sol_SE_incumbent_Deterministic.routesWarehouseToCustomer[w][t][k];
				if (!route.empty())
				{
					// Find the lowest node index in the route
					int lowestNodeIndex = *std::min_element(route.begin() + 1, route.end() - 1);
					vehicleLowestNodeIndex.push_back({k, lowestNodeIndex});
				}
				else
				{
					// If the route is empty, use a high value to push it to the end during sorting
					vehicleLowestNodeIndex.push_back({k, std::numeric_limits<int>::max()});
				}
			}

			// Step 2: Sort vehicles based on the lowest node index (ascending order)
			std::sort(vehicleLowestNodeIndex.begin(), vehicleLowestNodeIndex.end(),
					  [](const std::pair<int, int> &a, const std::pair<int, int> &b)
					  {
						  return a.second < b.second; // Sort in ascending order of lowest node index
					  });

			// Step 3: Reorganize the solution based on the sorted vehicle indices
			vector<vector<int>> sortedRoutes;
			for (const auto &vehicle : vehicleLowestNodeIndex)
			{
				int original_k = vehicle.first; // Get the original vehicle index
				sortedRoutes.push_back(sol_SE_incumbent_Deterministic.routesWarehouseToCustomer[w][t][original_k]);
			}

			// Step 4: Replace the original routes with the sorted ones
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				sol_SE_incumbent_Deterministic.routesWarehouseToCustomer[w][t][k] = sortedRoutes[k];
			}
		}
	}

	// Turn all customers assignment to warehouses if there is no delivery to the customer
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				if (sol_SE_incumbent_Deterministic.customerAssignmentToWarehouse[t][w][i] == 1)
				{
					bool customerVisited = false;
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						int customerIndex = i + params.numWarehouses; // Adjust customer index
						vector<int> &route = sol_SE_incumbent_Deterministic.routesWarehouseToCustomer[w][t][k];
						if (std::find(route.begin(), route.end(), customerIndex) != route.end())
						{
							customerVisited = true;
							break;
						}
					}

					if (!customerVisited)
					{
						sol_SE_incumbent_Deterministic.customerAssignmentToWarehouse[t][w][i] = 0;
					}
				}
			}
		}
	}
}

bool Algorithms::read_SolutionEV(SolutionFirstEchelon &sol_FE_EV, SolutionSecondEchelon_Deterministic &sol_SE_EV)
{
	cout << "Reading EV Solution..." << endl;
	string directory;
	string filename;
	directory = "../Results/Solutions/S2EPRP-AR/EV/BC/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios) +
				"/UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" +
				"/PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff));

	// Construct the filename
	filename = "Sol_S2EPRP-AR_EV_BC_" + params.probabilityFunction + "_" + params.instance + "_S" + std::to_string(params.numScenarios) +
			   "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" +
			   "_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
	string solutionFileName = directory + "/" + filename;

	if (!fs::exists(solutionFileName))
	{
		cout << "Solution file does not exist for BC, Checking for Hybrid-ILS..." << endl;
		directory = "../Results/Solutions/S2EPRP-AR/EV/Hybrid-ILS/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios) +
					"/UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" +
					"/PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff));

		filename = "Sol_S2EPRP-AR_EV_Hybrid-ILS_" + params.probabilityFunction + "_" + params.instance + "_S" + std::to_string(params.numScenarios) +
				   "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" +
				   "_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";

		solutionFileName = directory + "/" + filename;
		if (!fs::exists(solutionFileName))
		{
			cout << "File does not exist for Hybrid-ILS, Checking for ILS..." << endl;
			exit(1);
		}
	}

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
		file >> strValue;	 // ProbabilityFunction;
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
				file >> doubleValue; // Deterministic Demand
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

		sol_SE_EV.customerAssignmentToWarehouse.resize(params.numPeriods, vector<vector<int>>(params.numWarehouses, vector<int>(params.numCustomers)));
		sol_SE_EV.warehouseInventory.resize(params.numWarehouses, vector<double>(params.numPeriods));
		sol_SE_EV.customerInventory.resize(params.numCustomers, vector<double>(params.numPeriods));
		sol_SE_EV.customerUnmetDemand.resize(params.numCustomers, vector<double>(params.numPeriods));
		sol_SE_EV.routesWarehouseToCustomer.resize(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>())));
		sol_SE_EV.deliveryQuantityToCustomer.resize(params.numCustomers, vector<double>(params.numPeriods));
		
		// Read Solutions
		for (int t = 0; t < params.numPeriods; t++)
		{
			for (int w = 0; w < params.numWarehouses; w++)
			{
				for (int i = 0; i < params.numCustomers; i++)
				{
					file >> sol_SE_EV.customerAssignmentToWarehouse[t][w][i]; // Customer Assignment To Warehouse
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
				file >> sol_SE_EV.warehouseInventory[w][t];
			}
		}

		for (int i = 0; i < params.numCustomers; i++)
		{
			for (int t = 0; t < params.numPeriods; t++)
			{
				file >> sol_SE_EV.customerInventory[i][t];
			}
		}

		for (int i = 0; i < params.numCustomers; i++)
		{
			for (int t = 0; t < params.numPeriods; t++)
			{
				file >> sol_SE_EV.customerUnmetDemand[i][t];
			}
		}
		

		string line_One;
		while (std::getline(file, line_One))
		{
			if (line_One == "endRoutesPlantToWarehouse")
				break;

			if (line_One.find(':') == string::npos)
				continue;

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

			if (line_Two.find(':') == string::npos)
				continue;

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

			sol_SE_EV.routesWarehouseToCustomer[w_index][t_index][k_index] = route;
		}

		for (int i = 0; i < params.numCustomers; i++)
		{
			for (int t = 0; t < params.numPeriods; t++)
			{
				file >> sol_SE_EV.deliveryQuantityToCustomer[i][t];
			}
		}

		file.close();

		for (int t = 0; t < params.numPeriods; ++t)
		{
			sol_FE_EV.setupCost += params.setupCost * sol_FE_EV.productionSetup[t];
			sol_FE_EV.productionCost += params.unitProdCost * sol_FE_EV.productionQuantity[t];
			sol_FE_EV.holdingCostPlant += params.unitHoldingCost_Plant * sol_FE_EV.plantInventory[t];

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

		cout << "EV Solution loaded successfully." << endl;
		return true;
	}
	else
	{
		cerr << "EV solution not found!!" << endl;
		sol_FE_EV.clear();
		return false;
	}
}

