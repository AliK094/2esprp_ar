#include "stochastic/ILS.h"

ILS_SIRP::ILS_SIRP(const ParameterSetting &parameters, const SolutionFirstEchelon &sol_FE, const SolutionSecondEchelon &sol_SE)
	: params(parameters),
	  sol_FE(sol_FE),
	  sol_SE(sol_SE)
{
	// Initialize random seed
	srand(static_cast<unsigned int>(time(NULL)));
	// ----------------------------------------------------------------------------------------------------------
	Tolerance = 1e-6;
	cout << "\nILS..."
		 << endl;
}

bool ILS_SIRP::run()
{
	// Start the timer
	auto startTime_ILS = std::chrono::high_resolution_clock::now();
	auto elapsedTime_ILS = 0.0;

	SolutionFirstEchelon sol_FE_temp = sol_FE;
	SolutionSecondEchelon sol_SE_temp = sol_SE;
	Result result_temp;

	if (sol_SE_temp.routesWarehouseToCustomer.empty())
	{
		/*
			Construct Initial Solution: For Second Echelon
		*/
		sol_SE_temp.customerAssignmentToWarehouse = params.getCustomersAssignedToWarehouse();
		CATW = sol_SE_temp.customerAssignmentToWarehouse;

		cout << "Construct Initial Solution For Second Echelon" << endl;

		ConstructHeuristic consHeuristic(params, sol_FE_temp);
		bool status = consHeuristic.Construct_InitialSolution();
		if (!status)
		{
			cerr << "Failed to construct the initial solution for second echelon" << endl;
			return false;
		}

		sol_SE_temp.customerInventory = consHeuristic.getInvCustomers();
		sol_SE_temp.customerUnmetDemand = consHeuristic.getUnmetDemandCustomers();
		sol_SE_temp.deliveryQuantityToCustomer = consHeuristic.getDeliveryQuantityCustomers();
		sol_SE_temp.routesWarehouseToCustomer = consHeuristic.getRoutesWarehouseToCustomer();

		cout << "Improve Initial Solution For Second Echelon With Local Search" << endl;
	}

	// Initialize the LP solver with current parameters and feasible solution
	if (!solveLP(sol_FE_temp, sol_SE_temp, result_temp))
	{
		cerr << "Initial solution is not feasible" << endl;
		return false;
	}

	if (!checkSolutionFeasiblity(sol_FE_temp, sol_SE_temp))
	{
		cerr << "Initial solution is not feasible" << endl;
		return false;
	}

	sol_FE_incumbent = sol_FE_temp;
	sol_SE_incumbent = sol_SE_temp;
	result_incumbent = result_temp;

	// Define the number of threads you want to use
	int numThreads = std::thread::hardware_concurrency(); // Use the number of available CPU cores
	// cout << "\nAvailable CPU cores: " << numThreads << endl;
	vector<ScenarioSolutionSecondEchelon> sol_SE_incumbent_Scenarios(params.numScenarios);

	for (int s = 0; s < params.numScenarios; ++s)
	{
		sol_SE_incumbent_Scenarios[s].scenarioID = s;
		sol_SE_incumbent_Scenarios[s].routesWarehouseToCustomer_Scenario = sol_SE_incumbent.routesWarehouseToCustomer[s];
		sol_SE_incumbent_Scenarios[s].customerAssignmentToWarehouse_Scenario = sol_SE_incumbent.customerAssignmentToWarehouse[s];
	}

	cout << "\nRun ILS..." << endl;
	std::atomic<bool> stopProcessing(false); 
	vector<std::thread> threads;
	for (int s = 0; s < params.numScenarios; ++s) {
		threads.emplace_back([&, s]() {
			// Check if processing should continue
			if (!stopProcessing) 
			{
				LP_SE_Scenario lpse_scenario(params, sol_FE_incumbent, sol_SE_incumbent_Scenarios[s], s);		
				string status = lpse_scenario.solve();
				double best_objValue_Scenario = lpse_scenario.getObjVal_Scenario();

				// Initialize ILS for a specific Scenario
				const int maxIterILS = 20;
				int numIterILS = 0;
				bool stop = false;
				auto maxTime_ILS_Scenario = 120.0;

				// Start the timer
				auto startTime_ILS_Scenario = std::chrono::high_resolution_clock::now();
				auto elapsedTime_ILS_Scenario = 0.0;

				double objValue_Scenario = best_objValue_Scenario;
				while (!stop && numIterILS < maxIterILS && elapsedTime_ILS_Scenario < maxTime_ILS_Scenario)
				{
					ScenarioSolutionSecondEchelon sol_SE_temp_Scenario = sol_SE_incumbent_Scenarios[s];
					// cout << "\nIteration (ILS) for Scenario " << s + 1 << " Iteration : " << numIterILS + 1 << endl;
					if (numIterILS > 0)
					{
						/*
							Perturbation
						*/
						Perturbation perturb(s, params, sol_FE_incumbent, sol_SE_temp_Scenario);
						bool perturbSuccess = perturb.run();
						if (!perturbSuccess)
						{
							stop = true;
						}

						sol_SE_temp_Scenario = perturb.getSolutionSE_Scenario();
						objValue_Scenario = perturb.getObjVal_Scenario();
					}

					/*
						Local Search
					*/
					LocalSearch ls(s, params, sol_FE_incumbent, sol_SE_temp_Scenario, objValue_Scenario);
					ls.RVND();

					double objval_scenario_new = ls.getObjVal_Scenario();

					if (objval_scenario_new < best_objValue_Scenario - 1e-2)
					{
						best_objValue_Scenario = objval_scenario_new;
						sol_SE_incumbent_Scenarios[s] = ls.getSolutionSE_Scenario();

						// cout << "New incumbent solution found for Scenario " << s + 1 << " Iteration : " << numIterILS + 1 << endl;
					}

					numIterILS++;

					auto currentTime_ILS_Scenario = std::chrono::high_resolution_clock::now();
					elapsedTime_ILS_Scenario = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime_ILS_Scenario - startTime_ILS_Scenario).count();

					// cout << "Iteration : " << numIterILS << " for Scenario " << s + 1 << " is done" << " Time : " << elapsedTime_ILS_Scenario << endl;
				}

				// cout << "ILS for Scenario " << s + 1 << " is done" << endl;
			}
		});
	}
	
	// Join all the threads to wait for them to finish
	for (std::thread& t : threads) {
		if (t.joinable()) {
            t.join();
        }
	}
	// Join all the threads to wait for them to finish
	// -------------------------------------------
	cout << "ILS is done" << endl;
	sol_FE_temp = sol_FE_incumbent;

	for (int s = 0; s < params.numScenarios; ++s)
	{
		sol_SE_temp.routesWarehouseToCustomer[s] = sol_SE_incumbent_Scenarios[s].routesWarehouseToCustomer_Scenario;
		sol_SE_temp.customerAssignmentToWarehouse[s] = sol_SE_incumbent_Scenarios[s].customerAssignmentToWarehouse_Scenario;
	}

	if (!solveLP(sol_FE_temp, sol_SE_temp, result_temp))
	{
		cerr << "Failed to solve LP" << endl;
		return false;
	}

	if (!checkSolutionFeasiblity(sol_FE_temp, sol_SE_temp))
	{
		cerr << "Initial solution is not feasible" << endl;
		return false;
	}

	if (result_temp.objValue_Total < result_incumbent.objValue_Total - 1e-2)
	{
		sol_FE_incumbent = sol_FE_temp;
		sol_SE_incumbent = sol_SE_temp;
		result_incumbent = result_temp;
	}

	// Print the best objective function value
	cout << "Best Objective Function Value: " << std::setprecision(1) << std::fixed << result_incumbent.objValue_Total << endl;

	auto currentTime_ILS = std::chrono::high_resolution_clock::now();
	elapsedTime_ILS = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime_ILS - startTime_ILS).count();
	cout << "Total TIME ILS (Second Echelon): " << std::setprecision(3) << std::fixed << elapsedTime_ILS << endl;
	result_incumbent.totalCPUTime = elapsedTime_ILS;

	return true;
}

bool ILS_SIRP::checkSolutionFeasiblity(SolutionFirstEchelon sol_FE_temp, SolutionSecondEchelon sol_SE_temp)
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "ProductionCapacity(" + std::to_string(t + 1) + ")";

		if (sol_FE_temp.productionQuantity[t] > params.prodCapacity * sol_FE_temp.productionSetup[t])
		{
			cerr << "Constraint Violated: " << constraintName
				 << " | Production Quantity = " << sol_FE_temp.productionQuantity[t]
				 << " exceeds Production Capacity = " << params.prodCapacity << endl;
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "PlantInventoryCapacity(" + std::to_string(t + 1) + ")";

		// Assuming sol_FE.plantInventory[t] represents I_plant[t]
		if (sol_FE_temp.plantInventory[t] > params.storageCapacity_Plant)
		{
			cerr << "Constraint Violated: " << constraintName
				 << " | Plant Inventory = " << sol_FE_temp.plantInventory[t]
				 << " exceeds Storage Capacity = " << params.storageCapacity_Plant << endl;
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "PlantInventoryBalance(" + std::to_string(t + 1) + ")";

		double sumDeliveryToWarehouses = 0.0;
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			sumDeliveryToWarehouses += sol_FE_temp.deliveryQuantityToWarehouse[w][t];
		}

		// Calculate the expected plant inventory
		int expectedInventory;
		if (t == 0)
		{
			expectedInventory = params.initialInventory_Plant - sumDeliveryToWarehouses + sol_FE_temp.productionQuantity[t];
		}
		else
		{
			expectedInventory = sol_FE_temp.plantInventory[t - 1] - sumDeliveryToWarehouses + sol_FE_temp.productionQuantity[t];
		}

		// Check if the calculated inventory matches the expected inventory
		if (sol_FE_temp.plantInventory[t] != expectedInventory)
		{
			cerr << "Constraint Violated: " << constraintName
				 << " | Expected Plant Inventory = " << expectedInventory
				 << ", Actual Plant Inventory = " << sol_FE_temp.plantInventory[t] << endl;
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				string constraintName = "WarehouseInventoryCapacity(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				// Assuming sol_SE_temp.warehouseInventory[w][t][s] represents I_warehouse[w][t][s]
				if (sol_SE_temp.warehouseInventory[w][t][s] > params.storageCapacity_Warehouse[w])
				{
					cerr << "Constraint Violated: " << constraintName
						 << " | Warehouse Inventory = " << sol_SE_temp.warehouseInventory[w][t][s]
						 << " exceeds Storage Capacity = " << params.storageCapacity_Warehouse[w] << endl;
				}
			}
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				string constraintName = "WarehouseInventoryBalance(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				// calculate sum of delivery quantities to all customer
				double sumDeliveryQuantityToCustomers = 0.0;
				for (int i = 0; i < params.numCustomers; ++i)
				{
					if (sol_SE_temp.customerAssignmentToWarehouse[s][t][w][i] == 1)
					{
						sumDeliveryQuantityToCustomers += sol_SE_temp.deliveryQuantityToCustomer[i][t][s];
					}
				}

				// Determine expected inventory based on period
				double netInventoryChange;
				if (t == 0)
				{
					netInventoryChange = params.initialInventory_Warehouse[w] - sumDeliveryQuantityToCustomers + sol_FE_temp.deliveryQuantityToWarehouse[w][t];
				}
				else
				{
					netInventoryChange = sol_SE_temp.warehouseInventory[w][t - 1][s] - sumDeliveryQuantityToCustomers + sol_FE_temp.deliveryQuantityToWarehouse[w][t];
				}

				// Check if the actual inventory matches the expected net inventory change
				if (sol_SE_temp.warehouseInventory[w][t][s] != netInventoryChange)
				{
					cerr << "Constraint Violated: " << constraintName
						 << " | Expected Warehouse Inventory = " << netInventoryChange
						 << ", Actual Warehouse Inventory = " << sol_SE_temp.warehouseInventory[w][t][s] << endl;
				}
			}
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				string constraintName = "CustomerInventoryBalance(" + std::to_string(i + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				// Determine expected inventory based on period
				int expectedInventory;
				if (t == 0)
				{
					expectedInventory = params.initialInventory_Customer[i] + sol_SE_temp.deliveryQuantityToCustomer[i][t][s] - params.demand[i][t][s] + sol_SE_temp.customerUnmetDemand[i][t][s];
				}
				else
				{
					expectedInventory = sol_SE_temp.customerInventory[i][t - 1][s] + sol_SE_temp.deliveryQuantityToCustomer[i][t][s] - params.demand[i][t][s] + sol_SE_temp.customerUnmetDemand[i][t][s];
				}

				// Check if the actual inventory matches the expected inventory
				if (sol_SE_temp.customerInventory[i][t][s] != expectedInventory)
				{
					cerr << "Constraint Violated: " << constraintName
						 << " | Expected Customer Inventory = " << expectedInventory
						 << ", Actual Customer Inventory = " << sol_SE_temp.customerInventory[i][t][s] << endl;
				}
			}
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				string constraintName = "CustomerInventoryCapacity(" + std::to_string(i + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				// Calculate the maximum allowable inventory for the customer considering the demand
				int maxAllowableInventory = params.storageCapacity_Customer[i] - params.demand[i][t][s];

				// Check if the actual inventory exceeds this maximum allowable inventory
				if (sol_SE_temp.customerInventory[i][t][s] > maxAllowableInventory)
				{
					cerr << "Constraint Violated: " << constraintName
						 << " | Customer Inventory = " << sol_SE_temp.customerInventory[i][t][s]
						 << " exceeds Max Allowable Inventory = " << maxAllowableInventory << endl;
				}
			}
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					string constraintName = "VehicleCapacityWarehouse(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

					// Sum the deliveries assigned to vehicle k from warehouse w at time t in scenario s
					int totalDeliveriesByVehicle = 0;
					for (int i = 0; i < params.numCustomers; ++i)
					{
						auto it = std::find(sol_SE_temp.routesWarehouseToCustomer[s][w][t][k].begin(), sol_SE_temp.routesWarehouseToCustomer[s][w][t][k].end(), i + params.numWarehouses);
						if (it != sol_SE_temp.routesWarehouseToCustomer[s][w][t][k].end())
						{
							totalDeliveriesByVehicle += sol_SE_temp.deliveryQuantityToCustomer[i][t][s];
						}
					}

					// Check if the total deliveries exceed the vehicle capacity
					if (totalDeliveriesByVehicle > params.vehicleCapacity_Warehouse)
					{
						cerr << "Constraint Violated: " << constraintName
							 << " | Total Deliveries by Vehicle = " << totalDeliveriesByVehicle
							 << " exceeds Vehicle Capacity = " << params.vehicleCapacity_Warehouse << endl;
					}
				}
			}
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int i = 0; i < params.numCustomers; ++i)
				{
					bool found = false;
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						vector<int> &route = sol_SE_temp.routesWarehouseToCustomer[s][w][t][k];
						auto it = std::find(route.begin(), route.end(), i + params.numWarehouses);
						if (it != route.end())
						{
							found = true;
							break;
						}
					}

					if (!found && sol_SE_temp.deliveryQuantityToCustomer[i][t][s] > 0.0 && sol_SE_temp.customerAssignmentToWarehouse[s][t][w][i] == 1)
					{
						string constraintName = "CustomerVisit(" + std::to_string(i + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

						cerr << "Constraint Violated: " << constraintName
							 << " | Customer " << i + params.numWarehouses
							 << " is not visited in the route" << endl;

						cout << "Delivery Quantity: " << sol_SE_temp.deliveryQuantityToCustomer[i][t][s] << endl;
					}
				}
			}
		}
	}

	return true;
}

// Initialize the LP solver with current parameters and feasible solution
bool ILS_SIRP::solveLP(SolutionFirstEchelon &sol_FE_temp, SolutionSecondEchelon &sol_SE_temp, Result &result_temp)
{
	LP_SE lpse(params, sol_FE_temp, sol_SE_temp);
	string statusLP = lpse.solve();
	if (statusLP != "Optimal")
	{
		cerr << "LP solver failed with status: " << statusLP << endl;
		return false;
	}
	sol_FE_temp = lpse.getSolutionFE();
	sol_SE_temp = lpse.getSolutionSE();
	result_temp = lpse.getResult();

	return true;
}

SolutionFirstEchelon ILS_SIRP::getSolutionFE()
{
	return sol_FE_incumbent;
}

SolutionSecondEchelon ILS_SIRP::getSolutionSE()
{
	return sol_SE_incumbent;
}

Result ILS_SIRP::getResult()
{
	return result_incumbent;
}