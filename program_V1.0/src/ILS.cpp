#include "ILS.h"

ILS_SIRP::ILS_SIRP(const ParameterSetting &parameters, const Solution &solution)
	: params(parameters),
	  sol(solution)
{
	// Initialize random seed
	srand(static_cast<unsigned int>(time(NULL)));
	// ----------------------------------------------------------------------------------------------------------
	Tolerance = 1e-6;
	cout << "\nILS_SIRP...\n"
		 << endl;
}

bool ILS_SIRP::run()
{
	// Start the timer
	auto startTime_ILS = std::chrono::high_resolution_clock::now();
	auto elapsedTime_ILS = 0.0;

	if (sol.routesWarehouseToCustomer.empty())
	{
		/*
			Construct Initial Solution
		*/
		sol_temp.productionSetup = sol.productionSetup;
		sol_temp.productionQuantity = sol.productionQuantity;
		sol_temp.plantInventory = sol.plantInventory;
		sol_temp.deliveryQuantityToWarehouse = sol.deliveryQuantityToWarehouse;
		sol_temp.customerAssignmentToWarehouse = params.getCustomersAssignedToWarehouse();
		CATW = sol_temp.customerAssignmentToWarehouse;
		sol_temp.routesPlantToWarehouse = sol.routesPlantToWarehouse;

		sol_temp.setupCost = sol.setupCost;
		sol_temp.productionCost = sol.productionCost;
		sol_temp.holdingCostPlant = sol.holdingCostPlant;
		sol_temp.transportationCostPlantToWarehouse = sol.transportationCostPlantToWarehouse;

		cout << "Construct Initial Solution" << endl;

		ConstructHeuristic consHeuristic(params, sol_temp);
		bool status = consHeuristic.Construct_InitialSolution();
		if (!status)
		{
			cerr << "Failed to construct the initial solution" << endl;
			return false;
		}

		sol_temp.customerInventory = consHeuristic.getInvCustomers();
		sol_temp.customerUnmetDemand = consHeuristic.getUnmetDemandCustomers();
		sol_temp.deliveryQuantityToCustomer = consHeuristic.getDeliveryQuantityCustomers();
		sol_temp.routesWarehouseToCustomer = consHeuristic.getRoutesWarehouseToCustomer();
		// sol_temp.customerAssignmentToWarehouse = consHeuristic.getCustomerAssignmentToWarehouse();
		// CATW = sol_temp.customerAssignmentToWarehouse;
		sol_temp.warehouseInventory = calcInvWarehouse();

		// Initialize the LP solver with current parameters and feasible solution
		LP_SE lpse(params, sol_temp);
		string statusLP = lpse.solve();
		if (statusLP != "Optimal")
		{
			cerr << "LP solver failed with status: " << statusLP << endl;
			return false;
		}

		sol_temp = lpse.getSolution();

		for (int s = 0; s < params.numScenarios; ++s)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					for (int i = 0; i < params.numCustomers; ++i)
					{
						cout << sol_temp.customerAssignmentToWarehouse[s][t][w][i] << " ";
					}
					cout << endl;
				}
			}
		}

		// sol_incumbent = sol_temp;
		// return true;
	}
	else
	{
		cout << "We don't need to construct Initial Solution" << endl;
		/*
			Retrieve Solution from FE
		*/
		cout << "Retrieve Solution" << endl;
		sol_temp = sol;
	}

	// for (int s = 0; s < params.numScenarios; ++s)
	// {
	// 	for (int w = 0; w < params.numWarehouses; ++w)
	// 	{
	// 		for (int t = 0; t < params.numPeriods; ++t)
	// 		{
	// 			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
	// 			{
	// 				if (!sol_temp.routesWarehouseToCustomer[s][w][t][k].empty())
	// 				{
	// 					cout << "route[" << s + 1 << "][" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] : [";
	// 					for (auto it = sol_temp.routesWarehouseToCustomer[s][w][t][k].begin(); it != sol_temp.routesWarehouseToCustomer[s][w][t][k].end(); ++it)
	// 					{
	// 						if (it != sol_temp.routesWarehouseToCustomer[s][w][t][k].begin())
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

	if (!checkSolutionFeasiblity())
	{
		cerr << "Initial solution is not feasible" << endl;
		return false;
	}

	calculateObjFuncValue();
	cout << "Objective Function Value of First Echelon (Initial Solution): " << std::setprecision(1) << std::fixed << objValue_FE << endl;
	cout << "Objective Function Value of Second Echelon (Initial Solution): " << std::setprecision(1) << std::fixed << objValue_SE << endl;
	cout << "Objective Function Value of Total (Initial Solution): " << std::setprecision(1) << std::fixed << sol_temp.totalObjValue << endl;

	sol_incumbent = sol_temp;
	sol_temp.clear();

	for (int s = 0; s < params.numScenarios; ++s)
	{
		sol_temp = sol_incumbent;

		// Initialize ILS for a specific Scenario and Warehouse
		const int maxIterILS = 20;
		int numIterILS = 0;
		bool stop = false;
		auto maxTime_ILS_ScenarioWarehouse = 120.0;

		// Start the timer
		auto startTime_ILS_ScenarioWarehouse = std::chrono::high_resolution_clock::now();
		auto elapsedTime_ILS_ScenarioWarehouse = 0.0;

		while (!stop && numIterILS < maxIterILS && elapsedTime_ILS_ScenarioWarehouse < maxTime_ILS_ScenarioWarehouse)
		{
			cout << "\nIteration (ILS) for Scenario " << s  << " Iteration : " << numIterILS + 1 << endl;

			if (numIterILS > 0)
			{
				/*
					Perturbation
				*/
				Perturbation perturb(s, params, sol_temp);
				bool perturbSuccess = perturb.run();
				if (!perturbSuccess)
				{
					stop = true;
				}

				sol_temp = perturb.getSolution();
			}

			/*
				Local Search
			*/
			double objval = 0.0;
			LocalSearch ls(s, params, sol_temp);
			ls.RVND();

			sol_temp = ls.getSolution();

			if (sol_temp.totalObjValue < sol_incumbent.totalObjValue)
			{
				sol_incumbent = sol_temp;

				cout << "Best Objective Function Value (New): " << std::setprecision(1) << std::fixed << sol_incumbent.totalObjValue << endl;
			}

			// objValue_ILS += params.probability[s] * objVal;
			numIterILS++;

			auto currentTime_ILS_ScenarioWarehouse = std::chrono::high_resolution_clock::now();
			elapsedTime_ILS_ScenarioWarehouse = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime_ILS_ScenarioWarehouse - startTime_ILS_ScenarioWarehouse).count();
			cout << "TIME for Scenario: " << s << " : " << std::setprecision(3) << std::fixed << elapsedTime_ILS_ScenarioWarehouse << endl;
		}

		cout << "Best Objective Function Value After Scenario " << s << ": " << std::setprecision(1) << std::fixed << sol_incumbent.totalObjValue << endl;
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					if (!sol_temp.routesWarehouseToCustomer[s][w][t][k].empty())
					{
						cout << "route[" << s + 1 << "][" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] : [";
						for (auto it = sol_temp.routesWarehouseToCustomer[s][w][t][k].begin(); it != sol_temp.routesWarehouseToCustomer[s][w][t][k].end(); ++it)
						{
							if (it != sol_temp.routesWarehouseToCustomer[s][w][t][k].begin())
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

	// ----------------------------------------------------------------------------------------------------------
	if (!checkSolutionFeasiblity())
	{
		cerr << "Initial solution is not feasible" << endl;
		return false;
	}

	// for (int s = 0; s < params.numScenarios; ++s)
	// {
	// 	for (int t = 0; t < params.numPeriods; ++t)
	// 	{
	// 		for (int i = 0; i < params.numCustomers; ++i)
	// 		{
	// 			if (sol_temp.customerInventory[i][t][s] != 0)
	// 			{
	// 				cout << "Inv_Customers[" << i << "][" << t << "][" << s << "] = " << sol_temp.customerInventory[i][t][s] << endl;
	// 			}

	// 			if (sol_temp.customerUnmetDemand[i][t][s] != 0)
	// 			{
	// 				cout << "sol_temp.customerUnmetDemand[" << i << "][" << t << "][" << s << "] = " << sol_temp.customerUnmetDemand[i][t][s] << endl;
	// 			}
	// 		}
	// 	}
	// }
	// calculateObjFuncValue();
	// sol_incumbent.totalObjValue = objValue_Total;
	cout << "Best Objective Function Value: " << std::setprecision(1) << std::fixed << sol_incumbent.totalObjValue << endl;

	auto currentTime_ILS = std::chrono::high_resolution_clock::now();
	elapsedTime_ILS = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime_ILS - startTime_ILS).count();
	cout << "Total TIME ILS (Second Echelon): " << std::setprecision(3) << std::fixed << elapsedTime_ILS << endl;

	return true;
}

vector<vector<vector<double>>> ILS_SIRP::calcInvWarehouse()
{
	vector<vector<vector<double>>> Inv_Warehouses(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				double sumDeliveryFromWarehouse = 0.0;
				for (int i = 0; i < params.numCustomers; ++i)
				{
					if (CATW[s][t][w][i] == 1)
					{
						sumDeliveryFromWarehouse += sol_temp.deliveryQuantityToCustomer[i][t][s];
						// if (sol_temp.deliveryQuantityToCustomer[i][t][s] > 0.0)
						// 	cout << "sol_temp.deliveryQuantityToCustomer[" << i + params.numWarehouses << "][" << t << "][" << s << "] = " << sol_temp.deliveryQuantityToCustomer[i][t][s] << endl;
					}
				}

				// cout << "sumDeliveryFromWarehouse[" << w << "][" << t << "][" << s << "] = " << sumDeliveryFromWarehouse << endl;
				// cout << "deliveryQuantityToWarehouse[" << w << "][" << t << "] = " << sol_temp.deliveryQuantityToWarehouse[w][t] << endl;

				if (t == 0)
				{
					Inv_Warehouses[w][t][s] = params.initialInventory_Warehouse[w] + sol_temp.deliveryQuantityToWarehouse[w][t] - sumDeliveryFromWarehouse;
				}
				else
				{
					Inv_Warehouses[w][t][s] = Inv_Warehouses[w][t - 1][s] + sol_temp.deliveryQuantityToWarehouse[w][t] - sumDeliveryFromWarehouse;
				}

				// cout << "Inv_Warehouses_Scenario[" << w << "][" << t << "][" << s << "] = " << Inv_Warehouses[w][t][s] << endl;
			}
		}
	}

	return Inv_Warehouses;
}

bool ILS_SIRP::checkSolutionFeasiblity()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "ProductionCapacity(" + std::to_string(t + 1) + ")";

		if (sol_temp.productionQuantity[t] > params.prodCapacity * sol_temp.productionSetup[t])
		{
			cerr << "Constraint Violated: " << constraintName
				 << " | Production Quantity = " << sol_temp.productionQuantity[t]
				 << " exceeds Production Capacity = " << params.prodCapacity << endl;
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "PlantInventoryCapacity(" + std::to_string(t + 1) + ")";

		// Assuming sol_temp.plantInventory[t] represents I_plant[t]
		if (sol_temp.plantInventory[t] > params.storageCapacity_Plant)
		{
			cerr << "Constraint Violated: " << constraintName
				 << " | Plant Inventory = " << sol_temp.plantInventory[t]
				 << " exceeds Storage Capacity = " << params.storageCapacity_Plant << endl;
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "PlantInventoryBalance(" + std::to_string(t + 1) + ")";

		double sumDeliveryToWarehouses = 0.0;
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			sumDeliveryToWarehouses += sol_temp.deliveryQuantityToWarehouse[w][t];
		}

		// Calculate the expected plant inventory
		int expectedInventory;
		if (t == 0)
		{
			expectedInventory = params.initialInventory_Plant - sumDeliveryToWarehouses + sol_temp.productionQuantity[t];
		}
		else
		{
			expectedInventory = sol_temp.plantInventory[t - 1] - sumDeliveryToWarehouses + sol_temp.productionQuantity[t];
		}

		// Check if the calculated inventory matches the expected inventory
		if (sol_temp.plantInventory[t] != expectedInventory)
		{
			cerr << "Constraint Violated: " << constraintName
				 << " | Expected Plant Inventory = " << expectedInventory
				 << ", Actual Plant Inventory = " << sol_temp.plantInventory[t] << endl;
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				string constraintName = "WarehouseInventoryCapacity(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				// Assuming sol_temp.warehouseInventory[w][t][s] represents I_warehouse[w][t][s]
				if (sol_temp.warehouseInventory[w][t][s] > params.storageCapacity_Warehouse[w])
				{
					cerr << "Constraint Violated: " << constraintName
						 << " | Warehouse Inventory = " << sol_temp.warehouseInventory[w][t][s]
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
					if (sol_temp.customerAssignmentToWarehouse[s][t][w][i] == 1)
					{
						sumDeliveryQuantityToCustomers += sol_temp.deliveryQuantityToCustomer[i][t][s];
					}
				}

				// Determine expected inventory based on period
				double netInventoryChange;
				if (t == 0)
				{
					netInventoryChange = params.initialInventory_Warehouse[w] - sumDeliveryQuantityToCustomers + sol_temp.deliveryQuantityToWarehouse[w][t];
				}
				else
				{
					netInventoryChange = sol_temp.warehouseInventory[w][t - 1][s] - sumDeliveryQuantityToCustomers + sol_temp.deliveryQuantityToWarehouse[w][t];
				}

				// Check if the actual inventory matches the expected net inventory change
				if (sol_temp.warehouseInventory[w][t][s] != netInventoryChange)
				{
					cerr << "Constraint Violated: " << constraintName
						 << " | Expected Warehouse Inventory = " << netInventoryChange
						 << ", Actual Warehouse Inventory = " << sol_temp.warehouseInventory[w][t][s] << endl;
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
					expectedInventory = params.initialInventory_Customer[i] + sol_temp.deliveryQuantityToCustomer[i][t][s] - params.demand[i][t][s] + sol_temp.customerUnmetDemand[i][t][s];
				}
				else
				{
					expectedInventory = sol_temp.customerInventory[i][t - 1][s] + sol_temp.deliveryQuantityToCustomer[i][t][s] - params.demand[i][t][s] + sol_temp.customerUnmetDemand[i][t][s];
				}

				// Check if the actual inventory matches the expected inventory
				if (sol_temp.customerInventory[i][t][s] != expectedInventory)
				{
					cerr << "Constraint Violated: " << constraintName
						 << " | Expected Customer Inventory = " << expectedInventory
						 << ", Actual Customer Inventory = " << sol_temp.customerInventory[i][t][s] << endl;
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
				if (sol_temp.customerInventory[i][t][s] > maxAllowableInventory)
				{
					cerr << "Constraint Violated: " << constraintName
						 << " | Customer Inventory = " << sol_temp.customerInventory[i][t][s]
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
						auto it = std::find(sol_temp.routesWarehouseToCustomer[s][w][t][k].begin(), sol_temp.routesWarehouseToCustomer[s][w][t][k].end(), i + params.numWarehouses);
						if (it != sol_temp.routesWarehouseToCustomer[s][w][t][k].end())
						{
							totalDeliveriesByVehicle += sol_temp.deliveryQuantityToCustomer[i][t][s];
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
						vector<int> &route = sol_temp.routesWarehouseToCustomer[s][w][t][k];
						auto it = std::find(route.begin(), route.end(), i + params.numWarehouses);
						if (it != route.end())
						{
							found = true;
							break;
						}
					}

					if (!found && sol_temp.deliveryQuantityToCustomer[i][t][s] > 0.0 && sol_temp.customerAssignmentToWarehouse[s][t][w][i] == 1)
					{
						string constraintName = "CustomerVisit(" + std::to_string(i + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

						cerr << "Constraint Violated: " << constraintName
							 << " | Customer " << i + params.numWarehouses
							 << " is not visited in the route" << endl;

						cout << "Delivery Quantity: " << sol_temp.deliveryQuantityToCustomer[i][t][s] << endl;
					}
				}
			}
		}
	}

	return true;
}

void ILS_SIRP::calculateObjFuncValue()
{
	objValue_FE = sol_temp.setupCost + sol_temp.productionCost + sol_temp.holdingCostPlant + sol_temp.transportationCostPlantToWarehouse;
	cout << "setupCost = " << sol_temp.setupCost << endl;
	cout << "productionCost = " << sol_temp.productionCost << endl;
	cout << "holdingCostPlant = " << sol_temp.holdingCostPlant << endl;
	cout << "transportationCostPlantToWarehouse = " << sol_temp.transportationCostPlantToWarehouse << endl;
	cout << "objValue_FE = " << objValue_FE << endl;

	sol_temp.holdingCostWarehouse_Avg = 0.0;
	sol_temp.holdingCostCustomer_Avg = 0.0;
	sol_temp.costOfUnmetDemand_Avg = 0.0;
	sol_temp.transportationCostWarehouseToCustomer_Avg = 0.0;
	for (int s = 0; s < params.numScenarios; ++s)
	{
		// for (int t = 0; t < params.numPeriods; ++t)
		// {
		// 	for (int i = 0; i < params.numCustomers; ++i)
		// 	{
		// 		if (sol_temp.customerInventory[i][t][s] != 0)
		// 		{
		// 			cout << "Inv_Customers[" << i << "][" << t << "][" << s << "] = " << sol_temp.customerInventory[i][t][s] << endl;
		// 		}

		// 		if (sol_temp.customerUnmetDemand[i][t][s] != 0)
		// 		{
		// 			cout << "sol_temp.customerUnmetDemand[" << i << "][" << t << "][" << s << "] = " << sol_temp.customerUnmetDemand[i][t][s] << endl;
		// 		}
		// 	}
		// }

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				sol_temp.holdingCostWarehouse_Avg += params.probability[s] * params.unitHoldingCost_Warehouse[w] * sol_temp.warehouseInventory[w][t][s];
			}

			for (int i = 0; i < params.numCustomers; ++i)
			{
				sol_temp.holdingCostCustomer_Avg += params.probability[s] * params.unitHoldingCost_Customer[i] * sol_temp.customerInventory[i][t][s];
				sol_temp.costOfUnmetDemand_Avg += params.probability[s] * params.unmetDemandPenalty[i] * sol_temp.customerUnmetDemand[i][t][s];

				// cout << params.unitHoldingCost_Customer[i] << " * " << sol_temp.customerInventory[i][t][s] << " = " << params.unitHoldingCost_Customer[i] * sol_temp.customerInventory[i][t][s] << endl;

				// Inv_cost_ScenarioWarehouse += params.unitHoldingCost_Customer[i] * sol_temp.customerInventory[i][t][s];
				// unmetDemand_ScenarioWarehouse += params.unmetDemandPenalty[i] * sol_temp.customerUnmetDemand[i][t][s];

				// objValue_ScenarioWarehouse += params.unitHoldingCost_Customer[i] * sol_temp.customerInventory[i][t][s];
				// objValue_ScenarioWarehouse += params.unmetDemandPenalty[i] * sol_temp.customerUnmetDemand[i][t][s];
			}
		}

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					// double routeCost = 0.0;

					int previousNode = w;
					for (int j = 1; j < sol_temp.routesWarehouseToCustomer[s][w][t][k].size(); ++j)
					{
						int currentNode = sol_temp.routesWarehouseToCustomer[s][w][t][k][j];

						sol_temp.transportationCostWarehouseToCustomer_Avg += params.probability[s] * params.transportationCost_SecondEchelon[previousNode][currentNode];

						// routeCost += params.transportationCost_SecondEchelon[previousNode][currentNode];
						// objValue_ScenarioWarehouse += params.transportationCost_SecondEchelon[previousNode][currentNode];
						// routeCost_ScenarioWarehouse += params.transportationCost_SecondEchelon[previousNode][currentNode];

						previousNode = currentNode;
					}

					// cout << "Route cost (ILS) scenario " << s << " warehouse " << w << " period " << t << " vehicle " << k << " : " << routeCost << endl;
				}
			}
		}
		// cout << "Inventory cost (ILS) scenario " << s << " : " << Inv_cost_ScenarioWarehouse << endl;
		// cout << "Unmet demand cost (ILS) scenario " << s << " : " << unmetDemand_ScenarioWarehouse << endl;
		// cout << "Route cost (ILS) scenario " << s << " : " << routeCost_ScenarioWarehouse << endl;
		// cout << "Objective value (ILS) scenario " << s << " : " << objValue_ScenarioWarehouse << endl;
	}
	cout << "Avg holding cost warehouse (ILS) : " << sol_temp.holdingCostWarehouse_Avg << endl;
	cout << "Avg holding cost customer (ILS) : " << sol_temp.holdingCostCustomer_Avg << endl;
	cout << "Avg cost of unmet demand (ILS) : " << sol_temp.costOfUnmetDemand_Avg << endl;
	cout << "Avg transportation cost warehouse to customer (ILS) : " << sol_temp.transportationCostWarehouseToCustomer_Avg << endl;
	objValue_SE = sol_temp.holdingCostWarehouse_Avg + sol_temp.holdingCostCustomer_Avg + sol_temp.costOfUnmetDemand_Avg + sol_temp.transportationCostWarehouseToCustomer_Avg;

	sol_temp.totalObjValue = objValue_FE + objValue_SE;

	cout << "Objective value FE : " << objValue_FE << endl;
	cout << "Objective value (ILS) SE : " << objValue_SE << endl;
	cout << "Objective value (ILS) Total : " << sol_temp.totalObjValue << endl;
}

Solution ILS_SIRP::getSolution()
{
	return sol_incumbent;
}