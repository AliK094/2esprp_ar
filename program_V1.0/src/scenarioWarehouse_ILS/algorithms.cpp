#include "algorithms.h"

Algorithms::Algorithms(const string solutionAlgorithm, const ParameterSetting &parameters)
	: solAlg(solutionAlgorithm),
	  params(parameters)
{
}

bool Algorithms::solve_S2EPRP_BC(Solution &solFinal, const Solution &warmStart)
{
	cout << "\n\n\nStart Solving The Problem With Branch-and-Cut." << endl;

	Solution warmStartSolution = params.readSolutionWarmStart();
	// Branch and Cut
	S2EPRP_BC s2eprp_bc(params, warmStartSolution);
	if (!s2eprp_bc.Solve())
	{
		return EXIT_FAILURE;
	}
	solFinal = s2eprp_bc.getSolution();

	return true;
}

bool Algorithms::solve_S2EPRP_HILS(Solution &solFinal, const Solution &initSol)
{
	cout << "\n-------------------------------------------------------------------" << endl;
	cout << "Start Solving The Problem With Hybrid-ILS." << endl;
	cout << "-------------------------------------------------------------------" << endl;

	auto elapsedTime_InitialSolution = 0.0;
	auto startTime_InitialSolution = std::chrono::high_resolution_clock::now();

	// -----------------------------------------------------------------------------------------------------------------
	Solution sol_Current;
	Solution solFinal_temp;

	// Solve the first-echelon problem
	if (!solveFirstEchelon(sol_Current))
	{
		return EXIT_FAILURE;
	}

	// Run ILS for the second-echelon problem
	if (!runILSForSecondEchelon(sol_Current, solFinal_temp))
	{
		return EXIT_FAILURE;
	}

	solFinal = solFinal_temp;

	cout << "Initial Phase is finished." << endl;
	cout << "-------------------------------------------------------------------" << endl;
	cout << "Best Solution:\n" << endl;
	cout << "Setup Cost : " << solFinal.setupCost << endl;
	cout << "Production Cost : " << solFinal.productionCost << endl;
	cout << "Holding Cost Plant : " << solFinal.holdingCostPlant << endl;
	cout << "Transportation Cost Plant to Warehouse : " << solFinal.transportationCostPlantToWarehouse << endl;
	cout << "Holding Cost Warehouse : " << solFinal.holdingCostWarehouse_Avg << endl;
	cout << "Holding Cost Customer : " << solFinal.holdingCostCustomer_Avg << endl;
	cout << "Cost of Unmet Demand : " << solFinal.costOfUnmetDemand_Avg << endl;
	cout << "Transportation Cost Warehouse to Customer : " << solFinal.transportationCostWarehouseToCustomer_Avg << endl;

	cout << "\nObjective value (ILS) Total : " << solFinal.totalObjValue << endl;

	cout << "\nRoutes (Warehouse To Customer):" << endl;
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					if (!solFinal.routesWarehouseToCustomer[s][w][t][k].empty()){
						cout << "route[" << s + 1 << "][" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] : [";
						for (auto it = solFinal.routesWarehouseToCustomer[s][w][t][k].begin(); it != solFinal.routesWarehouseToCustomer[s][w][t][k].end(); ++it)
						{
							if (it != solFinal.routesWarehouseToCustomer[s][w][t][k].begin())
							{
								cout << " -> ";
							}
							cout << *it;
						}
						cout << "]" << endl;
					}
				}
			}
		}
	}


	// Handle unmet demand and optimize routes
	const int maxIteration = 10;

	for (int iter = 1; iter <= maxIteration; ++iter)
	{
		cout << "\n-----------------------------------------------------------------" << endl;
		cout << "\nSolving The Problem With Hybrid-ILS. Iteration: " << iter << endl;

		sol_Current.clear();
		sol_Current = solFinal_temp;
		solFinal_temp.clear();

		optimizeUnmetDemandAndRoutes(sol_Current);

		// Solve the restricted problem and finalize the solution
		if (!solveRestrictedProblemAndFinalize(sol_Current))
		{
			return EXIT_FAILURE;
		}

		cout << "Routes:" << endl;
		for (int s = 0; s < params.numScenarios; ++s)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int t = 0; t < params.numPeriods; ++t)
				{
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						vector<int> route = sol_Current.routesWarehouseToCustomer[s][w][t][k];
						if (!route.empty())
						{
							cout << "s = " << s + 1 << ", w = " << w + 1 << ", t = " << t + 1 << ", k = " << k + 1 << ": ";
							for (size_t i = 0; i < route.size(); ++i)
							{
								cout << route[i] << " ";
							}
							cout << endl;
						}
					}
				}
			}
		}

		// Run ILS for the second-echelon problem
		if (!runILSForSecondEchelon(sol_Current, solFinal_temp))
		{
			return EXIT_FAILURE;
		}
	}

	solFinal = solFinal_temp;

	cout << "-------------------------------------------------------------------" << endl;
	cout << "Best Solution:\n" << endl;
	cout << "Setup Cost : " << solFinal.setupCost << endl;
	cout << "Production Cost : " << solFinal.productionCost << endl;
	cout << "Holding Cost Plant : " << solFinal.holdingCostPlant << endl;
	cout << "Transportation Cost Plant to Warehouse : " << solFinal.transportationCostPlantToWarehouse << endl;
	cout << "Holding Cost Warehouse : " << solFinal.holdingCostWarehouse_Avg << endl;
	cout << "Holding Cost Customer : " << solFinal.holdingCostCustomer_Avg << endl;
	cout << "Cost of Unmet Demand : " << solFinal.costOfUnmetDemand_Avg << endl;
	cout << "Transportation Cost Warehouse to Customer : " << solFinal.transportationCostWarehouseToCustomer_Avg << endl;

	cout << "\nObjective value (ILS) Total : " << solFinal.totalObjValue << endl;

	auto currentTime_InitialSolution = std::chrono::high_resolution_clock::now();
	elapsedTime_InitialSolution = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime_InitialSolution - startTime_InitialSolution).count();

	cout << "Total Computation Time (Hybrid-ILS): " << elapsedTime_InitialSolution << " seconds" << endl;

	cout << "\nRoutes (Warehouse To Customer):" << endl;
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					if (!solFinal.routesWarehouseToCustomer[s][w][t][k].empty()){
						cout << "route[" << s + 1 << "][" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] : [";
						for (auto it = solFinal.routesWarehouseToCustomer[s][w][t][k].begin(); it != solFinal.routesWarehouseToCustomer[s][w][t][k].end(); ++it)
						{
							if (it != solFinal.routesWarehouseToCustomer[s][w][t][k].begin())
							{
								cout << " -> ";
							}
							cout << *it;
						}
						cout << "]" << endl;
					}
				}
			}
		}
	}

	cout << "\nCustomer Unmet Demand:" << endl;
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (solFinal.customerUnmetDemand[i][t][s] > 1e-4)
				{
					cout << "b_customer[" << i + params.numWarehouses << "][" << t + 1 << "][" << s + 1 << "] = " << solFinal.customerUnmetDemand[i][t][s] << endl;
				}
			}
		}
	}

	cout << "\nDelivery Quantity To Customer:" << endl;
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (solFinal.deliveryQuantityToCustomer[i][t][s] > 1e-4)
				{
					cout << "w_customer[" << i + params.numWarehouses << "][" << t + 1 << "][" << s + 1 << "] = " << solFinal.deliveryQuantityToCustomer[i][t][s] << endl;
				}
			}
		}
	}

	cout << "Customers Assigned To Warehouse:" << endl;
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					if (solFinal.customerAssignmentToWarehouse[s][t][w][i] == 1)
					{
						cout << "customer " << i + params.numWarehouses << " is assigned to warehouse " << w + 1 << " in scenario " << s + 1 << " period " << t + 1 << endl;
					}
				}
			}
		}
	}


	return true;
}

bool Algorithms::solveFirstEchelon(Solution &sol_Current)
{
	cout << "Solve The First-Echelon Problem" << endl;
	MWPRP_FE mwprp_fe(params);
	if (!mwprp_fe.Solve())
	{
		return false;
	}
	sol_Current = mwprp_fe.getSolution();
	return true;
}

bool Algorithms::runILSForSecondEchelon(Solution &sol_Current, Solution &solFinal)
{

	ILS_SIRP ils_SIRP(params, sol_Current);
	if (!ils_SIRP.run())
	{
		return false;
	}

	solFinal = ils_SIRP.getSolution();
	return true;
}

void Algorithms::optimizeUnmetDemandAndRoutes(Solution &sol)
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			cout << "Optimizing Unmet Demand for s = " << s + 1 << ", t = " << t + 1 << endl;
			vector<tuple<int, double>> unmetDemand_Descending;
			sortCustomersByUnmetDemand(sol, unmetDemand_Descending, t, s);

			for (const auto &entry : unmetDemand_Descending)
			{
				int customerIndex = std::get<0>(entry);
				double unmetDemand = std::get<1>(entry);

				if (unmetDemand > 1e-4)
				{
					handleUnmetDemandForCustomer(sol, s, t, customerIndex, unmetDemand);
				}
			}
		}
	}
}

void Algorithms::sortCustomersByUnmetDemand(Solution &sol, vector<tuple<int, double>> &unmetDemand_Descending, int t, int s)
{
	for (int i = 0; i < params.numCustomers; ++i)
	{
		unmetDemand_Descending.emplace_back(i, sol.customerUnmetDemand[i][t][s]);
	}

	sort(unmetDemand_Descending.begin(), unmetDemand_Descending.end(),
		 [](const tuple<int, double> &a, const tuple<int, double> &b)
		 {
			 return std::get<1>(a) > std::get<1>(b);
		 });

	// print unmet demand for customers
	// for (const auto &entry : unmetDemand_Descending)
	// {
	// 	int customerIndex = std::get<0>(entry);
	// 	double unmetDemand = std::get<1>(entry);
	// 	if (unmetDemand > 0.0){
	// 		cout << "s = " << s + 1 << ", t = " << t + 1 << ", i = " << customerIndex + params.numWarehouses << ": " << unmetDemand << endl;
	// 	}
	// }
}

void Algorithms::handleUnmetDemandForCustomer(Solution &sol, int s, int t, int customerIndex, double unmetDemand)
{
	double unmetDemCost = params.unmetDemandPenalty[customerIndex] * unmetDemand;
	int currentWarehouse = findCurrentWarehouse(sol, s, t, customerIndex);
	// cout << "currentWarehouse for customer " << customerIndex + params.numWarehouses << " = " << currentWarehouse + 1 << endl;

	vector<vector<int>> WareToCustSortedByDistance = params.getSortedWarehousesByDistance();
	for (int wareToInsert : WareToCustSortedByDistance[customerIndex])
	{
		if (wareToInsert != currentWarehouse)
		{
			// cout << "possible warehouse to insert customer " << customerIndex + params.numWarehouses << " into: " << wareToInsert + 1 << endl;
			if (attemptToInsertCustomerIntoWarehouse(sol, s, t, wareToInsert, currentWarehouse, customerIndex, unmetDemand, unmetDemCost))
			{
				break;
			}
		}
	}
}

int Algorithms::findCurrentWarehouse(Solution &sol, int s, int t, int customerIndex)
{
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		if (sol.customerAssignmentToWarehouse[s][t][w][customerIndex] == 1)
		{
			return w;
		}
	}
	return -1;
}

bool Algorithms::attemptToInsertCustomerIntoWarehouse(Solution &sol, int s, int t, int wareToInsert, int currentWarehouse, int customerIndex, double unmetDemand, double unmetDemCost)
{
	vector<double> remainingVehicleCapacityWarehouse(params.numVehicles_Warehouse, params.vehicleCapacity_Warehouse);
	double remainingWarehouseCapacity = params.storageCapacity_Warehouse[wareToInsert];

	updateRemainingCapacities(sol, s, t, wareToInsert, remainingVehicleCapacityWarehouse, remainingWarehouseCapacity);

	auto maxIt = std::max_element(remainingVehicleCapacityWarehouse.begin(), remainingVehicleCapacityWarehouse.end());
	if (maxIt != remainingVehicleCapacityWarehouse.end() && *maxIt > 0.0 && remainingWarehouseCapacity > 0.0)
	{
		int routeToInsert = -1;
		int posToInsert = -1;
		double minCostToInsert = std::numeric_limits<double>::max();

		findBestInsertionPosition(sol, s, t, wareToInsert, customerIndex, remainingVehicleCapacityWarehouse, remainingWarehouseCapacity, routeToInsert, posToInsert, minCostToInsert);

		double tempDeliveryQuantity = std::min({params.demand[customerIndex][t][s], remainingVehicleCapacityWarehouse[routeToInsert], remainingWarehouseCapacity});
		if (minCostToInsert < unmetDemCost && routeToInsert != -1)
		{
			applyInsertion(sol, s, t, wareToInsert, currentWarehouse, customerIndex, routeToInsert, posToInsert, tempDeliveryQuantity);
			return true;
		}
	}
	return false;
}

void Algorithms::updateRemainingCapacities(Solution &sol, int s, int t, int wareToInsert, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity)
{
	for (int r = 0; r < params.numVehicles_Warehouse; ++r)
	{
		if (!sol.routesWarehouseToCustomer[s][wareToInsert][t][r].empty())
		{
			for (auto it = sol.routesWarehouseToCustomer[s][wareToInsert][t][r].begin() + 1; it != sol.routesWarehouseToCustomer[s][wareToInsert][t][r].end() - 1; ++it)
			{
				remainingVehicleCapacityWarehouse[r] -= sol.deliveryQuantityToCustomer[*it - params.numWarehouses][t][s];
				remainingWarehouseCapacity -= sol.deliveryQuantityToCustomer[*it - params.numWarehouses][t][s];
			}
		}
	}
}

void Algorithms::findBestInsertionPosition(Solution &sol, int s, int t, int wareToInsert, int customerIndex, vector<double> &remainingVehicleCapacityWarehouse, double &remainingWarehouseCapacity, int &routeToInsert, int &posToInsert, double &minCostToInsert)
{
	for (int r = 0; r < params.numVehicles_Warehouse; ++r)
	{
		if (remainingVehicleCapacityWarehouse[r] >= params.demand[customerIndex][t][s] && remainingWarehouseCapacity >= params.demand[customerIndex][t][s])
		{
			if (sol.routesWarehouseToCustomer[s][wareToInsert][t][r].empty())
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
				std::pair<int, double> minInsertionCostResult = minInsertionCost(sol.routesWarehouseToCustomer[s][wareToInsert][t][r], customerIndex + params.numWarehouses);
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

void Algorithms::applyInsertion(Solution &sol, int s, int t, int wareToInsert, int currentWarehouse, int customerIndex, int routeToInsert, int posToInsert, double tempDeliveryQuantity)
{
	sol.customerAssignmentToWarehouse[s][t][currentWarehouse][customerIndex] = 0;
	cout << "Inserting customer " << customerIndex + params.numWarehouses << " from warehouse " << currentWarehouse << " into warehouse " << wareToInsert << " in Period " << t + 1 << " and Scenario " << s + 1 << endl;
	removeCustomerFromCurrentRoute(sol, s, t, currentWarehouse, customerIndex);
	// cout << "CATW[" << s + 1 << "][" << t + 1 << "][" << currentWarehouse + 1 << "][" << customerIndex << "] = 0" << endl;

	if (sol.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].empty())
	{
		sol.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].push_back(wareToInsert);
		sol.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].push_back(customerIndex + params.numWarehouses);
		sol.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].push_back(wareToInsert);
	}
	else
	{
		sol.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].insert(sol.routesWarehouseToCustomer[s][wareToInsert][t][routeToInsert].begin() + posToInsert, customerIndex + params.numWarehouses);
	}
	sol.deliveryQuantityToCustomer[customerIndex][t][s] = tempDeliveryQuantity;
	sol.customerAssignmentToWarehouse[s][t][wareToInsert][customerIndex] = 1;


}

void Algorithms::removeCustomerFromCurrentRoute(Solution &sol, int s, int t, int currentWarehouse, int customerIndex)
{
	for (auto &route : sol.routesWarehouseToCustomer[s][currentWarehouse][t])
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

bool Algorithms::solveRestrictedProblemAndFinalize(Solution &sol)
{
	RS2EPRP rs2eprp(params, sol);
	if (!rs2eprp.Solve())
	{
		return false;
	}
	sol.clear();
	sol = rs2eprp.getSolution();

	return true;
}

