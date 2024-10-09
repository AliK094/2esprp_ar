#include "ILS.h"

ILS_SIRP::ILS_SIRP(const ParameterSetting &parameters, const Solution &solution, bool initial)
	: params(parameters),
	  sol(solution),
	  createInitSol(initial)
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

	// Solution sol_temp;
	Solution sol_Final;	

	if(createInitSol)
	{
		/*
			Construct Initial Solution
		*/
		sol_temp.productionSetup = sol.productionSetup;
		sol_temp.productionQuantity = sol.productionQuantity;
		sol_temp.plantInventory = sol.plantInventory;
		sol_temp.deliveryQuantityToWarehouse = sol.deliveryQuantityToWarehouse;
		sol_temp.customerAssignmentToWarehouse = params.getCustomersAssignedToWarehouse();
		
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
		sol_temp.warehouseInventory = calcInvWarehouse();

		// Inv_Customers = consHeuristic.getInvCustomers();
		// sol_temp.customerUnmetDemand = consHeuristic.getUnmetDemandCustomers();
		// deliveryQuantity_Customers = consHeuristic.getDeliveryQuantityCustomers();
		// routes_WarehouseToCustomer = consHeuristic.getRoutesWarehouseToCustomer();

		// Inv_Warehouses = calcInvWarehouse();
	}
	else
	{
		/*
			Retrieve Solution from FE
		*/
		cout << "Retrieve Solution from FE" << endl;
		sol_temp = sol;
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					cout << "route[" << s << "][" << w << "][" << t << "][" << k << "] : [";
					for (auto it = sol_temp.routes_WarehouseToCustomer[s][w][t][k].begin(); it != sol_temp.routes_WarehouseToCustomer[s][w][t][k].end(); ++it)
					{
						if (it != sol_temp.routes_WarehouseToCustomer[s][w][t][k].begin())
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

	calculateObjFuncValue();
	cout << "Objective Function Value of First Echelon (Initial Solution): " << std::setprecision(1) << std::fixed << objValue_FE << endl;
	cout << "Objective Function Value of Second Echelon (Initial Solution): " << std::setprecision(1) << std::fixed << objValue_SE << endl;
	cout << "Objective Function Value of Total (Initial Solution): " << std::setprecision(1) << std::fixed << objValue_Total << endl;

	if (!checkSolutionFeasiblity())
	{
		cerr << "Initial solution is not feasible" << endl;
		return false;
	}

	double objValue_ILS = 0.0;
	for (int s = 0; s < params.numScenarios; ++s)
	{

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			// Initialize ILS for a specific Scenario and Warehouse
			const int maxIterILS = 20;
			int numIterILS = 0;
			bool stop = false;
			double best_ObjValue_ScenarioWarehouse = std::numeric_limits<double>::max();
			auto maxTime_ILS_ScenarioWarehouse = 120.0;

			// Start the timer
			auto startTime_ILS_ScenarioWarehouse = std::chrono::high_resolution_clock::now();
			auto elapsedTime_ILS_ScenarioWarehouse = 0.0;

			while (!stop && numIterILS < maxIterILS && elapsedTime_ILS_ScenarioWarehouse < maxTime_ILS_ScenarioWarehouse)
			{
				cout << "\nIteration (ILS) for Scenario " << s << ", Warehouse " << w << ": " << numIterILS + 1 << endl;
				vector<double> Inv_Warehouse_Scenario(params.numPeriods, 0.0);
				vector<vector<double>> Inv_Customers_Scenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
				vector<vector<double>> unmetDemand_Customers_Scenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
				vector<vector<double>> deliveryQuantity_Customers_Scenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));

				for (int t = 0; t < params.numPeriods; ++t){
					for (int i = 0; i < params.numCustomers; ++i) { 
						if (CATW[s][t][w][i] == 1) {
							Inv_Customers_Scenario[i][t] = Inv_Customers[i][t][s];
							unmetDemand_Customers_Scenario[i][t] = sol_temp.customerUnmetDemand[i][t][s];
							deliveryQuantity_Customers_Scenario[i][t] = sol_temp.deliveryQuantityToCustomer[i][t][s];

							// cout << "Inv_Cust_temp[" << i << "][" << t <<  "][" << s << "] = " << Inv_Cust_temp[i][t] << endl;
							// cout << "unmetDemand_Cust_temp[" << i << "][" << t <<  "][" << s << "] = " << unmetDemand_Cust_temp[i][t] << endl;
							// cout << "delQuant_Cust_temp[" << i + params.numWarehouses << "][" << t <<  "][" << s << "] = " << deliveryQuantity_Customers_Scenario[i][t] << endl;
						}
					}
				}
				Inv_Warehouse_Scenario = calcInvWarehouse_Scenario(s, w, deliveryQuantity_Customers_Scenario);
				vector<vector<vector<int>>> routes_wareToCust_Scenario = routes_WarehouseToCustomer[s][w];
				double objVal_ScenarioWarehouse_temp = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Warehouse_Scenario, Inv_Customers_Scenario, unmetDemand_Customers_Scenario, routes_wareToCust_Scenario);
				cout << "Best Objective Function Value (Scenario Warehouse): " << std::setprecision(1) << std::fixed << objVal_ScenarioWarehouse_temp << endl;

				if (numIterILS > 0)
				{
					/*
						Perturbation
					*/
					Perturbation perturb(s, w, params, sol_FE, CATW);
					bool perturbSuccess = perturb.run(Inv_Customers_Scenario,
													  unmetDemand_Customers_Scenario,
													  deliveryQuantity_Customers_Scenario,
													  routes_wareToCust_Scenario);
					if (!perturbSuccess)
					{
						stop = true;
					}
				}

				for (int t = 0; t < params.numPeriods; ++t)
				{
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						if (!routes_wareToCust_Scenario[t][k].empty())
						{
							cout << "route[" << t << "][" << k << "] : [";
							for (auto it = routes_wareToCust_Scenario[t][k].begin(); it != routes_wareToCust_Scenario[t][k].end(); ++it)
							{
								if (it != routes_wareToCust_Scenario[t][k].begin())
								{
									cout << " -> ";
								}
								cout << *it;
							}
							cout << "]" << endl;
						}
					}
				}

				/*
					Local Search
				*/
				double objval = 0.0;
				LocalSearch ls(s, w, params, sol_FE, CATW);
				ls.RVND(Inv_Customers_Scenario,
						unmetDemand_Customers_Scenario,
						deliveryQuantity_Customers_Scenario,
						routes_wareToCust_Scenario,
						objval);

				Inv_Warehouse_Scenario = calcInvWarehouse_Scenario(s, w, deliveryQuantity_Customers_Scenario);
				double objValue_ScenarioWarehouse = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Warehouse_Scenario, Inv_Customers_Scenario, unmetDemand_Customers_Scenario, routes_wareToCust_Scenario);
				// cout << "Best Objective Function Value (Scenario Warehouse): " << std::setprecision(1) << std::fixed << best_ObjValue_ScenarioWarehouse << endl;

				if (objValue_ScenarioWarehouse < best_ObjValue_ScenarioWarehouse)
				{
					best_ObjValue_ScenarioWarehouse = objValue_ScenarioWarehouse;
					for (int t = 0; t < params.numPeriods; ++t)
					{
						for (int i = 0; i < params.numCustomers; ++i) { 
							if (CATW[s][t][w][i] == 1) {
								Inv_Customers[i][t][s] = Inv_Customers_Scenario[i][t];
								sol_temp.customerUnmetDemand[i][t][s] = unmetDemand_Customers_Scenario[i][t];
								sol_temp.deliveryQuantityToCustomer[i][t][s] = deliveryQuantity_Customers_Scenario[i][t];
								// cout << "delQuant_Cust_temp[" << i + params.numWarehouses << "][" << t <<  "][" << s << "] = " << sol_temp.deliveryQuantityToCustomer[i][t][s] << endl;
							}
						}
					}
					routes_WarehouseToCustomer[s][w] = routes_wareToCust_Scenario;
					cout << "Best Objective Function Value (Scenario Warehouse): " << std::setprecision(1) << std::fixed << best_ObjValue_ScenarioWarehouse << endl;
				}

				// objValue_ILS += params.probability[s] * objVal;
				numIterILS++;

				auto currentTime_ILS_ScenarioWarehouse = std::chrono::high_resolution_clock::now();
				elapsedTime_ILS_ScenarioWarehouse = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime_ILS_ScenarioWarehouse - startTime_ILS_ScenarioWarehouse).count();
				cout << "TIME for Scenario: " << s << " Warehouse: " << w << " : " << std::setprecision(3) << std::fixed << elapsedTime_ILS_ScenarioWarehouse << endl;
			}
		}
	}
	Inv_Warehouses = calcInvWarehouse();

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					cout << "route[" << s << "][" << w << "][" << t << "][" << k << "] : [";
					for (auto it = routes_WarehouseToCustomer[s][w][t][k].begin(); it != routes_WarehouseToCustomer[s][w][t][k].end(); ++it)
					{
						if (it != routes_WarehouseToCustomer[s][w][t][k].begin())
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
	// 			if (Inv_Customers[i][t][s] != 0)
	// 			{
	// 				cout << "Inv_Customers[" << i << "][" << t << "][" << s << "] = " << Inv_Customers[i][t][s] << endl;
	// 			}

	// 			if (sol_temp.customerUnmetDemand[i][t][s] != 0)
	// 			{
	// 				cout << "sol_temp.customerUnmetDemand[" << i << "][" << t << "][" << s << "] = " << sol_temp.customerUnmetDemand[i][t][s] << endl;
	// 			}
	// 		}
	// 	}
	// }
	calculateObjFuncValue();
	cout << "\nObjValue_Total: " << objValue_Total << endl;

	retrieveSolutions();

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
				for (int i = 0; i < params.numCustomers; ++i) { 
					if (sol_temp.customerAssignmentToWarehouse[s][t][w][i] == 1) {
						sumDeliveryFromWarehouse += sol_temp.deliveryQuantityToCustomer[i][t][s];
						// if (sol_temp.deliveryQuantityToCustomer[i][t][s] > 0.0)
						// 	cout << "sol_temp.deliveryQuantityToCustomer[" << i + params.numWarehouses << "][" << t << "][" << s << "] = " << sol_temp.deliveryQuantityToCustomer[i][t][s] << endl;
					}
				}

				cout << "sumDeliveryFromWarehouse[" << w << "][" << t << "][" << s << "] = " << sumDeliveryFromWarehouse << endl;
				cout << "deliveryQuantityToWarehouse[" << w << "][" << t << "] = " << sol_temp.deliveryQuantityToWarehouse[w][t] << endl;

				

				if (t == 0)
				{
					Inv_Warehouses[w][t][s] = params.initialInventory_Warehouse[w] + sol_temp.deliveryQuantityToWarehouse[w][t] - sumDeliveryFromWarehouse;
				}
				else
				{
					Inv_Warehouses[w][t][s] = Inv_Warehouses[w][t - 1][s] + sol_temp.deliveryQuantityToWarehouse[w][t] - sumDeliveryFromWarehouse;
				}

				cout << "Inv_Warehouses_Scenario[" << w << "][" << t << "][" << s << "] = " << Inv_Warehouses[w][t][s] << endl;
			}
		}
	}

	return Inv_Warehouses;
}

vector<double> ILS_SIRP::calcInvWarehouse_Scenario(int s, int w, const vector<vector<double>> &deliveryQuantity_Customers_Scenario)
{
	vector<double> Inv_Warehouses_Scenario(params.numPeriods, 0.0);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		double sumDeliveryFromWarehouse_Scenario = 0.0;
		for (int i = 0; i < params.numCustomers; ++i) { 
			if (CATW[s][t][w][i] == 1) {
				// cout << "delQuant_Cust_temp[" << i << "][" << t <<  "][" << s << "] = " << deliveryQuantity_Customers_Scenario[i][t] << endl;
				sumDeliveryFromWarehouse_Scenario += deliveryQuantity_Customers_Scenario[i][t];
			}
		}

		if (t == 0)
		{
			Inv_Warehouses_Scenario[t] = params.initialInventory_Warehouse[w] + sol_FE.deliveryQuantityToWarehouse[w][t] - sumDeliveryFromWarehouse_Scenario;
		}
		else
		{
			Inv_Warehouses_Scenario[t] = Inv_Warehouses_Scenario[t - 1] + sol_FE.deliveryQuantityToWarehouse[w][t] - sumDeliveryFromWarehouse_Scenario;
		}

		// cout << "Inv_Warehouses_Scenario[" << w << "][" << t << "][" << s << "] = " << Inv_Warehouses_Scenario[t] << endl;
	}

	return Inv_Warehouses_Scenario;
}

double ILS_SIRP::calculateObjFuncValue_ScenarioWarehouse(int s, int w, const vector<double> &Inv_Ware_Temp, const vector<vector<double>> &Inv_Cust_Temp, const vector<vector<double>> &unmetDem_Cust_Temp, const vector<vector<vector<int>>> &route_WTC_Temp)
{
	cout << "\ncalculating objFuncValue_ScenarioWarehouse" << endl;
	// for (int t = 0; t < params.numPeriods; ++t)
	// {
	// 	for (int i = 0; i < CATW[w].size(); ++i)
	// 	{
	// 		if (Inv_Cust_Temp[i][t] > 0.0)
	// 		{
	// 			cout << "cust_Inv[" << i << "][" << t << "] = " << Inv_Cust_Temp[i][t] << endl;
	// 		}

	// 		if (unmetDem_Cust_Temp[i][t] > 0.0)
	// 		{
	// 			cout << "cust_unmDem[" << i << "][" << t << "] = " << unmetDem_Cust_Temp[i][t] << endl;
	// 		}
	// 	}
	// }

	double objFuncValue = 0.0;

	double inventoryCost_Warehouse = 0.0;
	double inventoryCost_Customer = 0.0;
	double unmetDemandCost = 0.0;
	double transportationCost = 0.0;

	// Inventory Holding Cost and Unmet Demand Cost
	for (int t = 0; t < params.numPeriods; ++t)
	{
		inventoryCost_Warehouse += params.unitHoldingCost_Warehouse[w] * Inv_Ware_Temp[t];
		for (int i = 0; i < params.numCustomers; ++i) { 
			if (CATW[s][t][w][i] == 1) {
				inventoryCost_Customer += params.unitHoldingCost_Customer[i] * Inv_Cust_Temp[i][t];
				unmetDemandCost += params.unmetDemandPenalty[i] * unmetDem_Cust_Temp[i][t];

				// objFuncValue += params.unitHoldingCost_Customer[CATW[s][t][w][i]] * Inv_Cust_Temp[i][t];
				// objFuncValue += params.unmetDemandPenalty[CATW[s][t][w][i]] * unmetDem_Cust_Temp[i][t];

				// cout << params.unitHoldingCost_Customer[CATW[w][i]] << " * " << Inv_Cust_Temp[i][t] << " = " << params.unitHoldingCost_Customer[CATW[w][i]] * Inv_Cust_Temp[i][t] << endl;
			}
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			double routeCost = 0.0;

			int previousNode = w;
			for (int j = 1; j < route_WTC_Temp[t][k].size(); ++j)
			{
				int currentNode = route_WTC_Temp[t][k][j];
				// objFuncValue += params.transportationCost_SecondEchelon[previousNode][currentNode];

				routeCost += params.transportationCost_SecondEchelon[previousNode][currentNode];
				previousNode = currentNode;
			}
			transportationCost += routeCost;

			// cout << "Route[" << w << "][" << t << "][" << k << "]: [";
			// for (int i = 0; i < route_WTC_Temp[t][k].size(); ++i)
			// {
			// 	if (i != route_WTC_Temp[t][k].size() - 1)
			// 	{
			// 		cout << route_WTC_Temp[t][k][i] << " -> ";
			// 	}
			// 	else
			// 	{
			// 		cout << route_WTC_Temp[t][k][i];
			// 	}
			// }
			// cout << "]" << endl;

			// cout << "Route Cost(RVND)[" << s << "][" << w << "][" << t << "][" << k << "]: " << routeCost << endl;
		}
	}

	// cout << "Inv Cost Warehouse: " << inventoryCost_Warehouse << endl;
	objFuncValue = inventoryCost_Warehouse + inventoryCost_Customer + unmetDemandCost + transportationCost;

	// cout << "Inventory Cost (RVND)[" << s << "][" << w << "]: " << inventoryCost << endl;
	// cout << "Unmet Demand Cost (RVND)[" << s << "][" << w << "]: " << unmetDemandCost << endl;
	// cout << "Route Cost (RVND)[" << s << "][" << w << "]: " << routeCost_Warehouse << endl;

	// cout << "objFuncValue:" << objFuncValue << endl;
	return objFuncValue;
}

bool ILS_SIRP::checkSolutionFeasiblity()
{
	// ----------------------------------------------------------------------------------------------------------
	// Customers Inventory Capacity of Customers
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				// if (Inv_Customers[i][t][s] > 0.0){
				// 	cout << "\nI[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << Inv_Customers[i][t][s] << endl;
				// }

				// if (sol_temp.customerUnmetDemand[i][t][s] > 0.0){
				// 	cout << "b[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << sol_temp.customerUnmetDemand[i][t][s] << endl;
				// }

				// if (sol_temp.deliveryQuantityToCustomer[i][t][s] > 0.0){
				// 	cout << "q[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << sol_temp.deliveryQuantityToCustomer[i][t][s] << endl;
				// }

				// cout << "demand[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << params.demand[i][t][s] << endl;
				// cout << "init_Inv[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << params.initialInventory_Customer[i] << endl;

				if (t == 0)
				{
					if (abs(Inv_Customers[i][t][s] - (params.initialInventory_Customer[i] + sol_temp.customerUnmetDemand[i][t][s] + sol_temp.deliveryQuantityToCustomer[i][t][s] - params.demand[i][t][s])) > Tolerance)
					{
						cout << "I[" << i << "][" << t << "][" << s << "] != ";
						cout << "init_Inv[" << i << "][" << t << "][" << s << "] + ";
						cout << "w[" << i << "][" << t << "][" << s << "] - ";
						cout << "demand[" << i << "][" << t << "][" << s << "]";
						cout << "b[" << i << "][" << t << "][" << s << "]" << endl;

						cout << Inv_Customers[i][t][s] << " != " << params.initialInventory_Customer[i] << " + " << sol_temp.deliveryQuantityToCustomer[i][t][s] << " - " << params.demand[i][t][s] << " + " << sol_temp.customerUnmetDemand[i][t][s] << endl;

						return false;
					}
				}
				else
				{
					if (std::abs(Inv_Customers[i][t][s] - (Inv_Customers[i][t - 1][s] + sol_temp.customerUnmetDemand[i][t][s] + sol_temp.deliveryQuantityToCustomer[i][t][s] - params.demand[i][t][s])) > Tolerance)
					{
						cout << "I[" << i << "][" << t << "][" << s << "] != ";
						cout << "I" << i << "][" << t - 1 << "][" << s << "] + ";
						cout << "w[" << i << "][" << t << "][" << s << "] - ";
						cout << "demand[" << i << "][" << t << "][" << s << "]";
						cout << "b[" << i << "][" << t << "][" << s << "]" << endl;

						cout << Inv_Customers[i][t][s] << " != " << Inv_Customers[i][t - 1][s] << " + " << sol_temp.deliveryQuantityToCustomer[i][t][s] << " - " << params.demand[i][t][s] << " + " << sol_temp.customerUnmetDemand[i][t][s] << endl;

						return false;
					}
				}

				if (Inv_Customers[i][t][s] + params.demand[i][t][s] > params.storageCapacity_Customer[i] + Tolerance)
				{
					cout << "Inv_Customers[i][t][s] + demand[i][t][s] > storageCapacity_Customer" << endl;
					return false;
				}
			}
		}

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				// for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				// {
				// 	cout << "route[" << s << "][" << w << "][" << t << "][" << k << "] : [";
				// 	for (auto it = routes_WarehouseToCustomer[s][w][t][k].begin(); it != routes_WarehouseToCustomer[s][w][t][k].end(); ++it)
				// 	{
				// 		if (it != routes_WarehouseToCustomer[s][w][t][k].begin())
				// 		{
				// 			cout << " -> ";
				// 		}
				// 		cout << *it;
				// 	}
				// 	cout << "]" << endl;
				// }

				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					if (!routes_WarehouseToCustomer[s][w][t][k].empty())
					{
						double sum = 0.0;
						for (auto it = routes_WarehouseToCustomer[s][w][t][k].begin() + 1; it != routes_WarehouseToCustomer[s][w][t][k].end() - 1; ++it)
						{
							sum += sol_temp.deliveryQuantityToCustomer[*it - params.numWarehouses][t][s];
						}

						if (sum > params.vehicleCapacity_Warehouse)
						{
							for (auto it = routes_WarehouseToCustomer[s][w][t][k].begin() + 1; it != routes_WarehouseToCustomer[s][w][t][k].end() - 1; ++it)
							{
								cout << "w[" << *it << "][" << t << "][" << s << "] + ";
							}
							cout << " > Q warehouse " << endl;
							cout << sum << " > " << params.vehicleCapacity_Warehouse << endl;

							return false;
						}
					}
				}
			}
		}
	}
	return true;
}

void ILS_SIRP::retrieveSolutions()
{
	// ----------------------------------------------------------------------------------------------------------
	/*
		Update Solution
	*/
	sol_Total.routesPlantToWarehouse = sol_FE.routesPlantToWarehouse;
	sol_Total.customerAssignmentToWarehouse = CATW;

	sol_Total.productionSetup = sol_FE.productionSetup;
	sol_Total.productionQuantity = sol_FE.productionQuantity;
	sol_Total.plantInventory = sol_FE.plantInventory;
	sol_Total.deliveryQuantityToWarehouse = sol_FE.deliveryQuantityToWarehouse;
	sol_Total.routesPlantToWarehouse = sol_FE.routesPlantToWarehouse;

	sol_Total.warehouseInventory = Inv_Warehouses;
	sol_Total.customerInventory = Inv_Customers;
	sol_Total.customerUnmetDemand = sol_temp.customerUnmetDemand;
	sol_Total.deliveryQuantityToCustomer = sol_temp.deliveryQuantityToCustomer;
	sol_Total.routesWarehouseToCustomer = routes_WarehouseToCustomer;
	cout << "Successfully updated solution" << endl;

	/*
		Update Costs
	*/
	sol_Total.setupCost = sol_FE.setupCost;
	sol_Total.productionCost = sol_FE.productionCost;
	sol_Total.holdingCostPlant = sol_FE.holdingCostPlant;
	sol_Total.transportationCostPlantToWarehouse = sol_FE.transportationCostPlantToWarehouse;
	sol_Total.holdingCostWarehouse_Avg = holdingCostWarehouse_Avg;
	sol_Total.holdingCostCustomer_Avg = holdingCostCustomer_Avg;
	sol_Total.costOfUnmetDemand_Avg = costOfUnmetDemand_Avg;
	sol_Total.transportationCostWarehouseToCustomer_Avg = transportationCostWarehouseToCustomer_Avg;
	cout << "Successfully updated costs" << endl;
	// ----------------------------------------------------------------------------------------------------------
}

void ILS_SIRP::calculateObjFuncValue()
{
	objValue_Total = 0.0;

	setupCost = sol_FE.setupCost;
	productionCost = sol_FE.productionCost;
	holdingCostPlant = sol_FE.holdingCostPlant;
	transportationCostPlantToWarehouse = sol_FE.transportationCostPlantToWarehouse;

	objValue_FE = setupCost + productionCost + holdingCostPlant + transportationCostPlantToWarehouse;
	cout << "setupCost = " << setupCost << endl;
	cout << "productionCost = " << productionCost << endl;
	cout << "holdingCostPlant = " << holdingCostPlant << endl;
	cout << "transportationCostPlantToWarehouse = " << transportationCostPlantToWarehouse << endl;
	cout << "objValue_FE = " << objValue_FE << endl;

	objValue_SE = 0.0;
	holdingCostWarehouse_Avg = 0.0;
	holdingCostCustomer_Avg = 0.0;
	costOfUnmetDemand_Avg = 0.0;
	transportationCostWarehouseToCustomer_Avg = 0.0;
	for (int s = 0; s < params.numScenarios; ++s)
	{	
		// for (int t = 0; t < params.numPeriods; ++t)
		// {
		// 	for (int i = 0; i < params.numCustomers; ++i)
		// 	{
		// 		if (Inv_Customers[i][t][s] != 0)
		// 		{
		// 			cout << "Inv_Customers[" << i << "][" << t << "][" << s << "] = " << Inv_Customers[i][t][s] << endl;
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
				holdingCostWarehouse_Avg += params.probability[s] * params.unitHoldingCost_Warehouse[w] * Inv_Warehouses[w][t][s];
			}

			for (int i = 0; i < params.numCustomers; ++i)
			{
				holdingCostCustomer_Avg += params.probability[s] * params.unitHoldingCost_Customer[i] * Inv_Customers[i][t][s];
				costOfUnmetDemand_Avg += params.probability[s] * params.unmetDemandPenalty[i] * sol_temp.customerUnmetDemand[i][t][s];

				// cout << params.unitHoldingCost_Customer[i] << " * " << Inv_Customers[i][t][s] << " = " << params.unitHoldingCost_Customer[i] * Inv_Customers[i][t][s] << endl;

				// Inv_cost_ScenarioWarehouse += params.unitHoldingCost_Customer[i] * Inv_Customers[i][t][s];
				// unmetDemand_ScenarioWarehouse += params.unmetDemandPenalty[i] * sol_temp.customerUnmetDemand[i][t][s];

				// objValue_ScenarioWarehouse += params.unitHoldingCost_Customer[i] * Inv_Customers[i][t][s];
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
					for (int j = 1; j < routes_WarehouseToCustomer[s][w][t][k].size(); ++j)
					{
						int currentNode = routes_WarehouseToCustomer[s][w][t][k][j];

						transportationCostWarehouseToCustomer_Avg += params.probability[s] * params.transportationCost_SecondEchelon[previousNode][currentNode];

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
	cout << "Avg holding cost warehouse (ILS) : " << holdingCostWarehouse_Avg << endl;
	cout << "Avg holding cost customer (ILS) : " << holdingCostCustomer_Avg << endl;
	cout << "Avg cost of unmet demand (ILS) : " << costOfUnmetDemand_Avg << endl;
	cout << "Avg transportation cost warehouse to customer (ILS) : " << transportationCostWarehouseToCustomer_Avg << endl;
	objValue_SE = holdingCostWarehouse_Avg + holdingCostCustomer_Avg + costOfUnmetDemand_Avg + transportationCostWarehouseToCustomer_Avg;

	objValue_Total = objValue_FE + objValue_SE;

	cout << "Objective value FE : " << objValue_FE << endl;
	cout << "Objective value (ILS) SE : " << objValue_SE << endl;
	cout << "Objective value (ILS) Total : " << objValue_Total << endl;
}

Solution ILS_SIRP::getSolution()
{
	return sol_Total;
}

ConstructHeuristic::ConstructHeuristic(const ParameterSetting &parameters, const Solution &solution)
	: params(parameters),
	  sol_init(solution),
	  CATW(sol_init.customerAssignmentToWarehouse)
{
	// ----------------------------------------------------------------------------------------------------------
	cout << "Construct Heuristic" << endl;

	// We calculate the Customers per unit unemt demand penalty cost to approximate per unit demand satisfaction cost ratio
	// We order Customers in descending order to have them from highest to lowest penalty cost ratio. (this shows the prioritun in demand satisfaction)
	sorted_Customers_byPenaltyCostRatio.resize(params.numScenarios, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numWarehouses, vector<int>())));
	orderCustomersByUnmetDemandToDeliveryRatio(sorted_Customers_byPenaltyCostRatio);

	bestObjValue = 0.0;
	Inv_Customers_bestSolution.resize(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	unmetDemand_Customers_bestSolution.resize(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	deliveryQuantity_Customers_bestSolution.resize(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	routes_bestSolution.resize(params.numScenarios, vector<vector<vector<vector<int>>>>(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>()))));
}

void ConstructHeuristic::orderCustomersByUnmetDemandToDeliveryRatio(vector<vector<vector<vector<int>>>> &sorted_customer_costRatio)
{
	// calculate the stockout to demand satisfaction cost ratio using: Ratio = penaltyCost[i] / F/C + 2c[0][w] + 2c[w][i] + look_ahead * h[i]
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				vector<std::pair<int, double>> customer_costRatio;
				for (int i = 0; i < params.numCustomers; ++i) { 
					if (CATW[s][t][w][i] == 1) {
						double costRatio = (params.unmetDemandPenalty[i]) /
										((params.setupCost / params.prodCapacity) +
											((2 * params.transportationCost_FirstEchelon[0][w + 1]) / params.vehicleCapacity_Plant) +
											((2 * params.transportationCost_SecondEchelon[w][i + params.numWarehouses]) / params.vehicleCapacity_Warehouse));

						customer_costRatio.emplace_back(i + params.numWarehouses, costRatio);
					}
				}

				std::sort(customer_costRatio.begin(), customer_costRatio.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b)
						  { return a.second > b.second; });

				for (const auto &pair : customer_costRatio)
				{
					sorted_customer_costRatio[s][t][w].push_back(pair.first);
				}
			}
		}
	}
}

void ConstructHeuristic::calculateDecisionVariables(int s, int w, vector<vector<double>> &Inv_Customers, vector<vector<double>> &unmetDemand_Customers, vector<vector<double>> &deliveryQuantity_Customers)
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i) { 
			if (CATW[s][t][w][i] == 1) {
				Inv_Customers[i][t] = 0.0;
				unmetDemand_Customers[i][t] = 0.0;
				if (t == 0)
				{
					Inv_Customers[i][t] = params.initialInventory_Customer[i];
				}
				else
				{
					Inv_Customers[i][t] = Inv_Customers[i][t - 1];
				}
				Inv_Customers[i][t] -= params.demand[i][t][s];
				Inv_Customers[i][t] += deliveryQuantity_Customers[i][t];

				if (Inv_Customers[i][t] < 0)
				{
					unmetDemand_Customers[i][t] = -Inv_Customers[i][t];
					Inv_Customers[i][t] = 0.0;
				}
			}
		}
	}
}

void ConstructHeuristic::defineSetOne(int s, int w, int t, vector<int> &setOne, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity)
{
	for (int i : sorted_Customers_byPenaltyCostRatio[s][t][w])
	{
		int custIndex = i - params.numWarehouses;

		if (unmetDemand_Customers[custIndex][t] > 0)
		{
			setOne.push_back(i);

			if (t == 0)
			{
				tempDeliveryQuantity[custIndex] = std::min({unmetDemand_Customers[custIndex][t], params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[custIndex] - params.initialInventory_Customer[custIndex])});
			}
			else if (t == params.numPeriods - 1)
			{
				tempDeliveryQuantity[custIndex] = std::min(params.vehicleCapacity_Warehouse, unmetDemand_Customers[custIndex][t]);
			}
			else
			{
				tempDeliveryQuantity[custIndex] = std::min({unmetDemand_Customers[custIndex][t], params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[custIndex] - Inv_Customers[custIndex][t - 1])});
			}
		}
	}
}

void ConstructHeuristic::defineSetTwo(int s, int w, int t, int look_ahead, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity)
{
	for (int i : sorted_Customers_byPenaltyCostRatio[s][t][w])
	{
		int custIndex = i - params.numWarehouses;

		double unetDemandCustomers_lookAhead = 0.0;
		for (int l = t + 1; l <= std::min(t + look_ahead, params.numPeriods - 1); ++l)
		{
			// cout << "look ahead demand [" << i << "] = " << unmetDemand_Customers[custIndex][l] << endl;
			unetDemandCustomers_lookAhead += unmetDemand_Customers[custIndex][l];
		}

		if (unetDemandCustomers_lookAhead > 0)
		{
			// check if customer i is in setOne
			auto it = std::find(setOne.begin(), setOne.end(), i);
			if (it != setOne.end())
			{
				if (t == 0)
				{
					tempDeliveryQuantity[custIndex] = std::min({unetDemandCustomers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[custIndex] - params.initialInventory_Customer[custIndex])});
				}
				else
				{
					tempDeliveryQuantity[custIndex] = std::min({unetDemandCustomers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[custIndex] - Inv_Customers[custIndex][t - 1])});
				}
			}
			else if (it == setOne.end())
			{
				setTwo.push_back(i);

				if (t == 0)
				{
					tempDeliveryQuantity[custIndex] = std::min({unetDemandCustomers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[custIndex] - params.initialInventory_Customer[custIndex])});
				}
				else
				{
					tempDeliveryQuantity[custIndex] = std::min({unetDemandCustomers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[custIndex] - Inv_Customers[custIndex][t - 1])});
				}
			}
		}
	}
}

void ConstructHeuristic::nearestNeighourInsertion(int s, int w, int t, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &deliveryQuantity_Customers, vector<double> &tempDeliveryQuantity, vector<vector<int>> &routesPeriod)
{
	int numRoutes = params.numVehicles_Warehouse;

	double remainingAvailInventoryWarehouse = params.initialInventory_Warehouse[w];
	for (int l = 0; l <= t; ++l)
	{
		remainingAvailInventoryWarehouse += sol_init.deliveryQuantityToWarehouse[w][l];
		for (int i = 0; i < params.numCustomers; ++i) { 
			if (CATW[s][t][w][i] == 1) {
				remainingAvailInventoryWarehouse -= deliveryQuantity_Customers[i][l];
			}
		}
	}
	if (remainingAvailInventoryWarehouse < 0.0)
	{
		remainingAvailInventoryWarehouse = 0.0;
	}

	vector<double> remainingVehicleCapacityWarehouse(numRoutes, params.vehicleCapacity_Warehouse);

	// We now have a partial route and we wanna add Customers (from setOne) to it
	bool AllNodesVisited = false;
	int r = 0;
	while (r < numRoutes && !AllNodesVisited)
	{
		// cout << "Vehicle " << r << endl;
		// Find closest customer to the current node
		int currentNode = w;
		if (remainingVehicleCapacityWarehouse[r] <= 0.0 || remainingAvailInventoryWarehouse <= 0.0)
		{
			break;
		}

		while (remainingVehicleCapacityWarehouse[r] > 0.0 && remainingAvailInventoryWarehouse > 0.0)
		{
			// cout << "Remaining Inventory: " << remainingAvailInventoryWarehouse << ", Remaining Vehicle Capacity: " << remainingVehicleCapacityWarehouse[r] << endl;
			int nextNodeToVisit = findNextNodeToVisit(currentNode, setOne, tempDeliveryQuantity, remainingVehicleCapacityWarehouse[r], remainingAvailInventoryWarehouse);
			// cout << "Next Node to Visit: " << nextNodeToVisit << endl;
			if (nextNodeToVisit != -1)
			{
				if (routesPeriod[r].empty())
				{
					routesPeriod[r].push_back(w);
				}

				currentNode = nextNodeToVisit;
				int custInd = currentNode - params.numWarehouses;

				routesPeriod[r].push_back(currentNode);
				deliveryQuantity_Customers[custInd][t] = tempDeliveryQuantity[custInd];
				remainingAvailInventoryWarehouse -= deliveryQuantity_Customers[custInd][t];
				remainingVehicleCapacityWarehouse[r] -= deliveryQuantity_Customers[custInd][t];

				auto it = std::find(setOne.begin(), setOne.end(), currentNode);
				if (it != setOne.end())
				{
					setOne.erase(it);
				}
			}
			else
			{
				// No more suitable nodes to visit in this route
				r++;
				if (setOne.empty())
				{
					AllNodesVisited = true;
					// cout << "No More Nodes in Set One to Visit " << endl;
				}
				break;
			}
		}
	}

	for (int r = 0; r < numRoutes; ++r)
	{
		if (!routesPeriod[r].empty())
		{
			routesPeriod[r].push_back(w);
		}
	}

	auto maxIt = std::max_element(remainingVehicleCapacityWarehouse.begin(), remainingVehicleCapacityWarehouse.end());
	if (maxIt != remainingVehicleCapacityWarehouse.end() && *maxIt > 0.0)
	{
		auto it = setTwo.begin();
		while (it != setTwo.end())
		{
			int i = *it;
			int custInd = i - params.numWarehouses;

			// cout << "\nInserting Node (check): " << i << endl;

			int routeToInsert = -1;
			int posToInsert = -1;
			double minCostToInsert = std::numeric_limits<double>::max();

			for (int r = 0; r < numRoutes; ++r)
			{
				// cout << "Route: " << r << endl;
				// cout << "Remaining Inventory (if visited): " << remainingAvailInventoryWarehouse - tempDeliveryQuantity[custInd] << ", Remaining Vehicle Capacity: " << remainingVehicleCapacityWarehouse[r] - tempDeliveryQuantity[custInd] << endl;
				if (remainingVehicleCapacityWarehouse[r] - tempDeliveryQuantity[custInd] >= 0.0 &&
					remainingAvailInventoryWarehouse - tempDeliveryQuantity[custInd] >= 0.0)
				{
					// cout << "size of routesPeriod[r]: " << routesPeriod[r].size() << endl;
					if (routesPeriod[r].empty())
					{
						double costToInsert = 2 * params.transportationCost_SecondEchelon[w][i];
						if (costToInsert < minCostToInsert)
						{
							posToInsert = 1;
							minCostToInsert = costToInsert;
							routeToInsert = r;
							// cout << "Possible Inserting in empty route " << routeToInsert << " at pos " << posToInsert << " with cost " << minCostToInsert << endl;
						}
					}
					else
					{
						std::pair<int, double> minInsertionCostResult = minInsertionCost(routesPeriod[r], i);

						double costToInsert = minInsertionCostResult.second;

						if (costToInsert < minCostToInsert)
						{
							posToInsert = minInsertionCostResult.first;
							minCostToInsert = costToInsert;
							routeToInsert = r;
							// cout << "Possible Inserting in route " << routeToInsert << " at pos " << posToInsert << " with cost " << minCostToInsert << endl;
						}
					}
				}
			}

			// cout << "Inserting " << i << " at pos " << posToInsert << " in route " << routeToInsert << endl;
			if (routeToInsert != -1)
			{
				if (routesPeriod[routeToInsert].empty())
				{
					routesPeriod[routeToInsert].push_back(w);
					routesPeriod[routeToInsert].push_back(i);
					routesPeriod[routeToInsert].push_back(w);
					deliveryQuantity_Customers[custInd][t] = tempDeliveryQuantity[custInd];
					remainingAvailInventoryWarehouse -= tempDeliveryQuantity[custInd];
					remainingVehicleCapacityWarehouse[routeToInsert] -= tempDeliveryQuantity[custInd];
					it = setTwo.erase(it);
				}
				else
				{
					routesPeriod[routeToInsert].insert(routesPeriod[routeToInsert].begin() + posToInsert, i);
					deliveryQuantity_Customers[custInd][t] = tempDeliveryQuantity[custInd];
					remainingAvailInventoryWarehouse -= tempDeliveryQuantity[custInd];
					remainingVehicleCapacityWarehouse[routeToInsert] -= tempDeliveryQuantity[custInd];
					it = setTwo.erase(it);
				}
			}
			else
			{
				++it;
			}
		}
	}

	for (int r = 0; r < numRoutes; ++r)
	{
		if (!routesPeriod[r].empty() && routesPeriod[r].back() != w)
		{
			routesPeriod[r].push_back(w);
		}
	}
}

int ConstructHeuristic::findNextNodeToVisit(int current_node, vector<int> &setOne, vector<double> &tempDeliveryQuantity, double remainingVehicleCapacityWarehouse, double remainingAvailInventoryWarehouse)
{
	double minVisitCost = std::numeric_limits<double>::max();
	int nearestNode = -1;

	// cout << "Current node: " << current_node << endl;
	// Loop through the first row to find the maximum value
	for (int i : setOne)
	{
		// cout << "next node (check): " << i << endl;
		int custIndex = i - params.numWarehouses;
		double visitCost = params.transportationCost_SecondEchelon[current_node][i];

		// cout << "visit cost: " << visitCost << endl;
		// cout << "Remaining Inventory (if visited): " << remainingAvailInventoryWarehouse - tempDeliveryQuantity[custIndex] << ", Remaining Vehicle Capacity: " << remainingVehicleCapacityWarehouse - tempDeliveryQuantity[custIndex] << endl;

		if (visitCost < minVisitCost && remainingVehicleCapacityWarehouse - tempDeliveryQuantity[custIndex] >= 0.0 && remainingAvailInventoryWarehouse - tempDeliveryQuantity[custIndex] >= 0.0)
		{
			minVisitCost = visitCost;
			nearestNode = i;
		}
	}

	return nearestNode;
}

std::pair<int, double> ConstructHeuristic::minInsertionCost(const vector<int> &routesPeriod, int i)
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

double ConstructHeuristic::calculateObjFuncValue(int s, int w, const vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, const vector<vector<vector<int>>> &routesPeriod)
{
	double objFuncValue = 0.0;

	// Inventory Holding Cost and Unmet Demand Cost
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i) { 
			if (CATW[s][t][w][i] == 1) {
				objFuncValue += params.unitHoldingCost_Customer[i] * Inv_Customers[i][t];
				objFuncValue += params.unmetDemandPenalty[i] * unmetDemand_Customers[i][t];
			}
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			int previousNode = w;
			for (int j = 1; j < routesPeriod[t][k].size(); ++j)
			{
				int currentNode = routesPeriod[t][k][j];
				objFuncValue += params.transportationCost_SecondEchelon[previousNode][currentNode];
				previousNode = currentNode;
			}
		}
	}

	return objFuncValue;
}

bool ConstructHeuristic::Construct_InitialSolution()
{
	auto elapsedTime_InitialSolution = 0.0;
	auto startTime_InitialSolution = std::chrono::high_resolution_clock::now();

	try
	{
		// Construct initial solution
		for (int s = 0; s < params.numScenarios; ++s)
		{
			vector<vector<double>> Inv_Customers_bestSolScenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
			vector<vector<double>> unmetDemand_Customers_bestSolScenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
			vector<vector<double>> deliveryQuantity_Customers_bestSolScenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
			vector<vector<vector<vector<int>>>> routes_bestSolScenario(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>())));
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				double bestObjValue_ScenarioWarehouse = std::numeric_limits<double>::max();

				int look_ahead = 1;
				while (look_ahead <= std::ceil(params.numPeriods / 2.0))
				{
					vector<vector<double>> Inv_Customers(params.numCustomers, vector<double>(params.numPeriods, 0.0));
					vector<vector<double>> unmetDemand_Customers(params.numCustomers, vector<double>(params.numPeriods, 0.0));
					vector<vector<double>> deliveryQuantity_Customers(params.numCustomers, vector<double>(params.numPeriods, 0.0));
					vector<vector<vector<int>>> routesPeriod(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>()));

					calculateDecisionVariables(s, w, Inv_Customers, unmetDemand_Customers, deliveryQuantity_Customers);

					for (int t = 0; t < params.numPeriods; ++t)
					{
						vector<double> tempDeliveryQuantity(params.numCustomers, 0.0);

						vector<int> setOne;
						defineSetOne(s, w, t, setOne, Inv_Customers, unmetDemand_Customers, tempDeliveryQuantity);

						vector<int> setTwo;
						if (t < params.numPeriods - 1)
						{
							defineSetTwo(s, w, t, look_ahead, setOne, setTwo, Inv_Customers, unmetDemand_Customers, tempDeliveryQuantity);
						}

						// cout << "setOne: [";
						// for (auto it = setOne.begin(); it != setOne.end(); ++it)
						// {
						// 	if (it != setOne.begin())
						// 	{
						// 		cout << ", ";
						// 	}
						// 	cout << *it << ": " << tempDeliveryQuantity[*it - params.numWarehouses];
						// }
						// cout << "]" << endl;

						// cout << "setTwo: [";
						// for (auto it = setTwo.begin(); it != setTwo.end(); ++it)
						// {
						// 	if (it != setTwo.begin())
						// 	{
						// 		cout << ", ";
						// 	}
						// 	cout << *it << ": " << tempDeliveryQuantity[*it - params.numWarehouses];
						// }
						// cout << "]" << endl;

						nearestNeighourInsertion(s, w, t, setOne, setTwo, deliveryQuantity_Customers, tempDeliveryQuantity, routesPeriod[t]);

						// for (int k = 0; k < params.numVehicles_Warehouse; ++k)
						// {
						// 	cout << "routesPeriod: [";
						// 	for (auto it = routesPeriod[t][k].begin(); it != routesPeriod[t][k].end(); ++it)
						// 	{
						// 		if (it != routesPeriod[t][k].begin())
						// 		{
						// 			cout << " -> ";
						// 		}
						// 		cout << *it;
						// 	}
						// 	cout << "]" << endl;
						// }

						calculateDecisionVariables(s, w, Inv_Customers, unmetDemand_Customers, deliveryQuantity_Customers);
					}

					// calculate objective cost of the second echelon for the current warehouse and scenario
					double objFuncValue_temp = calculateObjFuncValue(s, w, Inv_Customers, unmetDemand_Customers, routesPeriod);
					// cout << "objFuncValue_temp: " << objFuncValue_temp << endl;
					// update feasible solution
					if (objFuncValue_temp < bestObjValue_ScenarioWarehouse)
					{
						// cout << "New best feasible solution found = " << objFuncValue_temp << endl;

						bestObjValue_ScenarioWarehouse = objFuncValue_temp;
						for (int t = 0; t < params.numPeriods; ++t)
						{
							for (int i = 0; i < params.numCustomers; ++i) { 
								if (CATW[s][t][w][i] == 1) {
									Inv_Customers_bestSolScenario[i][t] = Inv_Customers[i][t];
									unmetDemand_Customers_bestSolScenario[i][t] = unmetDemand_Customers[i][t];
									deliveryQuantity_Customers_bestSolScenario[i][t] = deliveryQuantity_Customers[i][t];
								}
							}

							for (int k = 0; k < params.numVehicles_Warehouse; ++k)
							{
								routes_bestSolScenario[w][t][k] = routesPeriod[t][k];
							}
						}
					}
					++look_ahead;
				}

				bestObjValue += params.probability[s] * bestObjValue_ScenarioWarehouse;

				// cout << "\nbestObjValue: " << bestObjValue << endl;

				for (int i = 0; i < params.numCustomers; ++i)
				{
					for (int t = 0; t < params.numPeriods; ++t)
					{
						Inv_Customers_bestSolution[i][t][s] = Inv_Customers_bestSolScenario[i][t];
						unmetDemand_Customers_bestSolution[i][t][s] = unmetDemand_Customers_bestSolScenario[i][t];
						deliveryQuantity_Customers_bestSolution[i][t][s] = deliveryQuantity_Customers_bestSolScenario[i][t];
					}
				}

				for (int w = 0; w < params.numWarehouses; ++w)
				{
					for (int t = 0; t < params.numPeriods; ++t)
					{
						for (int k = 0; k < params.numVehicles_Warehouse; ++k)
						{
							routes_bestSolution[s][w][t][k] = routes_bestSolScenario[w][t][k];
						}
					}
				}
			}
		}
		
		auto currentTime_InitialSolution = std::chrono::high_resolution_clock::now();
		elapsedTime_InitialSolution = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime_InitialSolution - startTime_InitialSolution).count();
	}
	catch (std::exception &e)
	{
		return false;
	}
	cout << "Initial Solution Constructed Successfully" << endl;

	cout << "Time - Construct Initial Solution (seconds): " << std::setprecision(3) << std::fixed << elapsedTime_InitialSolution << endl;
	return true;
}

vector<vector<vector<double>>> ConstructHeuristic::getInvCustomers()
{
	return Inv_Customers_bestSolution;
}

vector<vector<vector<double>>> ConstructHeuristic::getUnmetDemandCustomers()
{
	return unmetDemand_Customers_bestSolution;
}

vector<vector<vector<double>>> ConstructHeuristic::getDeliveryQuantityCustomers()
{
	return deliveryQuantity_Customers_bestSolution;
}

vector<vector<vector<vector<vector<int>>>>> ConstructHeuristic::getRoutesWarehouseToCustomer()
{
	return routes_bestSolution;
}

double ConstructHeuristic::getBestObjValue()
{
	return bestObjValue;
}

LocalSearch::LocalSearch(const int s, const int w, const ParameterSetting &parameters, const Solution &solution, const vector<vector<vector<vector<int>>>> &custToWarehouse)
	: scenario(s), warehouse(w), params(parameters), sol_FE(solution), CATW(custToWarehouse)
{
	// ----------------------------------------------------------------------------------------------------------
	cout << "\nLocal Search for scenario " << scenario << " and warehouse " << warehouse << endl;
	// ----------------------------------------------------------------------------------------------------------
}

void LocalSearch::RVND(vector<vector<double>> &Inv_Cust_ScenWare,
					   vector<vector<double>> &unmetDemand_Cust_ScenWare,
					   vector<vector<double>> &delQuant_Cust_ScenWare,
					   vector<vector<vector<int>>> &routes_wareToCust_ScenWare,
					   double &objValue_ScenarioWarehouse)
{
	LP_ScenarioWarehouse LP(scenario, warehouse, CATW, params, sol_FE, routes_wareToCust_ScenWare);
	string status = LP.solve();
	if (status != "Optimal")
	{
		cerr << "Cannot solve LP for scenario " << scenario << " and warehouse " << warehouse << endl;
		exit(1);
	}

	vector<double> Inv_ware_ScenWare = LP.getInvWarehouse();
	Inv_Cust_ScenWare = LP.getInvCustomers();
	unmetDemand_Cust_ScenWare = LP.getUnmetDemandCustomers();
	delQuant_Cust_ScenWare = LP.getDeliveryQuantityCustomers();

	objValue_ScenarioWarehouse = calculateObjFuncValue_ScenarioWarehouse(Inv_ware_ScenWare,
																		 Inv_Cust_ScenWare,
																		 unmetDemand_Cust_ScenWare,
																		 routes_wareToCust_ScenWare);

	cout << "\nCurrent Objective Value for scenario " << scenario << " and warehouse " << warehouse << ": " << objValue_ScenarioWarehouse << endl;

	// Initialize the operators
	vector<std::function<bool(vector<vector<vector<int>>> &)>> operators = setOperators();
	int maxIterRVND = 25;

	while (!operators.empty() && maxIterRVND > 0)
	{
		vector<vector<vector<int>>> routes_wareToCust_ScenWare_temp = routes_wareToCust_ScenWare;

		vector<double> Inv_Ware_ScenWare_temp = Inv_ware_ScenWare;
		vector<vector<double>> Inv_Cust_ScenWare_temp = Inv_Cust_ScenWare;
		vector<vector<double>> unmetDemand_Cust_ScenWare_temp = unmetDemand_Cust_ScenWare;
		vector<vector<double>> delQuant_Cust_ScenWare_temp = delQuant_Cust_ScenWare;

		// Generate random number between 0 and operators.size()
		int index = rand() % operators.size();

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				if (!routes_wareToCust_ScenWare_temp[t][k].empty())
				{
					cout << "Route[" << scenario << "][" << warehouse << "][" << t << "][" << k << "]: [";
					for (int i = 0; i < routes_wareToCust_ScenWare_temp[t][k].size(); ++i)
					{
						if (i != routes_wareToCust_ScenWare_temp[t][k].size() - 1)
						{
							cout << routes_wareToCust_ScenWare_temp[t][k][i] << " -> ";
						}
						else
						{
							cout << routes_wareToCust_ScenWare_temp[t][k][i];
						}
					}
					cout << "]" << endl;
				}
			}
		}

		cout << "\n";
		// Execute the operator
		if (!operators[index](routes_wareToCust_ScenWare_temp))
		{
			// Remove the operator from the list
			operators.erase(operators.begin() + index);
			--maxIterRVND;
			continue;
		}

		LP_ScenarioWarehouse LP(scenario, warehouse, CATW, params, sol_FE, routes_wareToCust_ScenWare_temp);
		// solve LP to get the value of the coninuous variables
		string status = LP.solve();
		if (status != "Optimal")
		{
			operators.erase(operators.begin() + index);
			--maxIterRVND;
			continue;
		}
		Inv_Ware_ScenWare_temp = LP.getInvWarehouse();
		Inv_Cust_ScenWare_temp = LP.getInvCustomers();
		unmetDemand_Cust_ScenWare_temp = LP.getUnmetDemandCustomers();
		delQuant_Cust_ScenWare_temp = LP.getDeliveryQuantityCustomers();

		// for (int t = 0; t < params.numPeriods; ++t)
		// {
		// 	for (size_t i = 0; i < CATW[warehouse].size(); ++i)
		// 	{
		// 		cout << "Inv_Cust_temp[" << i << "][" << t <<  "][" << s << "] = " << Inv_Cust_ScenWare_temp[i][t] << endl;
		// 		cout << "unmetDemand_Cust_temp[" << i << "][" << t <<  "][" << s << "] = " << unmetDemand_Cust_ScenWare_temp[i][t] << endl;
		// 		cout << "delQuant_Cust_temp[" << i << "][" << t <<  "][" << s << "] = " << delQuant_Cust_ScenWare_temp[i][t] << endl;
		// 	}
		// }

		double objValue_ScenarioWarehouse_temp = calculateObjFuncValue_ScenarioWarehouse(Inv_Ware_ScenWare_temp,
																						 Inv_Cust_ScenWare_temp,
																						 unmetDemand_Cust_ScenWare_temp,
																						 routes_wareToCust_ScenWare_temp);

		cout << "\nBest objective value: " << objValue_ScenarioWarehouse << endl;
		cout << "Current objective value: " << objValue_ScenarioWarehouse_temp << endl;

		if (objValue_ScenarioWarehouse_temp < objValue_ScenarioWarehouse)
		{
			cout << "A Better solution found!\n"
				 << endl;
			// Update the best solution
			routes_wareToCust_ScenWare = routes_wareToCust_ScenWare_temp;

			Inv_ware_ScenWare = Inv_Ware_ScenWare_temp;
			Inv_Cust_ScenWare = Inv_Cust_ScenWare_temp;
			unmetDemand_Cust_ScenWare = unmetDemand_Cust_ScenWare_temp;
			delQuant_Cust_ScenWare = delQuant_Cust_ScenWare_temp;

			objValue_ScenarioWarehouse = objValue_ScenarioWarehouse_temp;

			operators = setOperators();
		}
		else
		{
			// Remove the operator from the list
			operators.erase(operators.begin() + index);
		}

		--maxIterRVND;
	}

	// for (int t = 0; t < params.numPeriods; ++t)
	// {
	// 	for (size_t i = 0; i < CATW[warehouse].size(); ++i)
	// 	{
	// 		if (Inv_Cust_ScenWare[i][t] != 0){
	// 			cout << "Inventory_Cust[" << i << "][" << t <<  "][" << s << "] = " << Inv_Cust_ScenWare[i][t] << endl;
	// 		}
	// 		if (unmetDemand_Cust_ScenWare[i][t] != 0){
	// 			cout << "unmetDemand_Cust[" << i << "][" << t <<  "][" << s << "] = " << unmetDemand_Cust_ScenWare[i][t] << endl;
	// 		}
	// 		if (delQuant_Cust_ScenWare[i][t] != 0){
	// 			cout << "delQuant_Cust[" << i << "][" << t <<  "][" << s << "] = " << delQuant_Cust_ScenWare[i][t] << endl;
	// 		}
	// 	}
	// }

	cout << "Best Objective Value for scenario " << scenario << " and warehouse " << warehouse << ": " << objValue_ScenarioWarehouse << endl;
}

vector<std::function<bool(vector<vector<vector<int>>> &)>> LocalSearch::setOperators()
{
	std::function<bool(vector<vector<vector<int>>> &)> OrOptOneFunc = std::bind(&LocalSearch::OrOpt_One, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> OrOptTwoFunc = std::bind(&LocalSearch::OrOpt_Two, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> OrOptThreeFunc = std::bind(&LocalSearch::OrOpt_Three, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> ShiftOneFunc = std::bind(&LocalSearch::Shift_One, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> ShiftTwoFunc = std::bind(&LocalSearch::Shift_Two, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> ShiftThreeFunc = std::bind(&LocalSearch::Shift_Three, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> SwapFunc = std::bind(&LocalSearch::Swap, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> InsertFunc = std::bind(&LocalSearch::Insert, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> RemoveFunc = std::bind(&LocalSearch::Remove, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> MergeFunc = std::bind(&LocalSearch::Merge, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> TransferFunc = std::bind(&LocalSearch::Transfer, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> RemInsFunc = std::bind(&LocalSearch::Remove_Insert, this, std::placeholders::_1);

	return {
		OrOptOneFunc,
		OrOptTwoFunc,
		OrOptThreeFunc,
		ShiftOneFunc,
		ShiftTwoFunc,
		ShiftThreeFunc,
		SwapFunc,
		InsertFunc,
		RemoveFunc,
		MergeFunc,
		TransferFunc,
		RemInsFunc};
}

bool LocalSearch::OrOpt_One(vector<vector<vector<int>>> &routes)
{
	int v = 1;
	cout << "Or-Opt(" << v << ")" << endl;

	// Choose random warehouse, period, and vehicle
	bool routeFound = false;
	const int maxAttempts = params.numPeriods * params.numVehicles_Warehouse;
	int attempts = 0;

	std::set<std::tuple<int, int>> usedCombinations;

	int t, k;
	vector<int> route;

	while (attempts < maxAttempts)
	{

		// Choose random w, t, k
		do
		{
			t = rand() % params.numPeriods;
			k = rand() % params.numVehicles_Warehouse;
		} while (usedCombinations.find(std::make_tuple(t, k)) != usedCombinations.end()); // Ensure the combination is not already used

		usedCombinations.insert(std::make_tuple(t, k)); // Mark this combination as used

		route = routes[t][k];

		if (route.size() >= v + 3)
		{
			cout << "Old route[" << warehouse << "][" << t << "][" << k << "]: [";
			for (int i = 0; i < route.size(); ++i)
			{
				if (i != route.size() - 1)
				{
					cout << route[i] << " -> ";
				}
				else
				{
					cout << route[i];
				}
			}
			cout << "]" << endl;
			cout << "route size = " << route.size() << endl;

			routeFound = true;
			break;
		}
		++attempts;
	}

	if (!routeFound)
	{
		cerr << "Could not find a route with " << v + 3 << " nodes" << endl;
		return false;
	}
	else
	{
		// Choose random starting position for the segment to move
		int startPos = 1 + rand() % (route.size() - v - 1); // Avoid first and last positions
		int newPos;

		// Ensure the new position is not the same as the current position
		do
		{
			newPos = 1 + rand() % (route.size() - v - 1);
		} while (newPos == startPos);

		// Move the segment of v customers to a new position
		vector<int> temp(route.begin() + startPos, route.begin() + startPos + v);
		route.erase(route.begin() + startPos, route.begin() + startPos + v);
		route.insert(route.begin() + newPos, temp.begin(), temp.end());

		cout << "Moved " << v << " customers from position " << startPos
			 << " to position " << newPos << " in route " << k << std::endl;

		routes[t][k] = route;
		cout << "Updated route[" << warehouse << "][" << t << "][" << k << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;
	}
	return true;
}

bool LocalSearch::OrOpt_Two(vector<vector<vector<int>>> &routes)
{
	int v = 2;
	cout << "Or-Opt(" << v << ")" << endl;

	// Choose random warehouse, period, and vehicle
	bool routeFound = false;
	const int maxAttempts = params.numPeriods * params.numVehicles_Warehouse;
	int attempts = 0;

	std::set<std::tuple<int, int>> usedCombinations;

	int t, k;
	vector<int> route;

	while (attempts < maxAttempts)
	{

		// Choose random w, t, k
		do
		{
			t = rand() % params.numPeriods;
			k = rand() % params.numVehicles_Warehouse;
		} while (usedCombinations.find(std::make_tuple(t, k)) != usedCombinations.end()); // Ensure the combination is not already used

		usedCombinations.insert(std::make_tuple(t, k)); // Mark this combination as used

		route = routes[t][k];

		if (route.size() >= v + 3)
		{
			cout << "Old route[" << warehouse << "][" << t << "][" << k << "]: [";
			for (int i = 0; i < route.size(); ++i)
			{
				if (i != route.size() - 1)
				{
					cout << route[i] << " -> ";
				}
				else
				{
					cout << route[i];
				}
			}
			cout << "]" << endl;
			cout << "route size = " << route.size() << endl;

			routeFound = true;
			break;
		}
		++attempts;
	}

	if (!routeFound)
	{
		cerr << "Could not find a route with " << v + 3 << " nodes" << endl;
		return false;
	}
	else
	{
		// Choose random starting position for the segment to move
		int startPos = 1 + rand() % (route.size() - v - 1); // Avoid first and last positions
		int newPos;

		// Ensure the new position is not the same as the current position
		do
		{
			newPos = 1 + rand() % (route.size() - v - 1);
		} while (newPos == startPos);

		// Move the segment of v customers to a new position
		vector<int> temp(route.begin() + startPos, route.begin() + startPos + v);
		route.erase(route.begin() + startPos, route.begin() + startPos + v);
		route.insert(route.begin() + newPos, temp.begin(), temp.end());

		cout << "Moved " << v << " customers from position " << startPos
			 << " to position " << newPos << " in route " << k << std::endl;

		routes[t][k] = route;
		cout << "Updated route[" << warehouse << "][" << t << "][" << k << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;
	}
	return true;
}

bool LocalSearch::OrOpt_Three(vector<vector<vector<int>>> &routes)
{
	int v = 3;
	cout << "Or-Opt(" << v << ")" << endl;

	// Choose random warehouse, period, and vehicle
	bool routeFound = false;
	const int maxAttempts = params.numPeriods * params.numVehicles_Warehouse;
	int attempts = 0;

	std::set<std::tuple<int, int>> usedCombinations;

	int t, k;
	vector<int> route;

	while (attempts < maxAttempts)
	{
		// Choose random w, t, k
		do
		{
			t = rand() % params.numPeriods;
			k = rand() % params.numVehicles_Warehouse;
		} while (usedCombinations.find(std::make_tuple(t, k)) != usedCombinations.end()); // Ensure the combination is not already used

		usedCombinations.insert(std::make_tuple(t, k)); // Mark this combination as used

		route = routes[t][k];

		if (route.size() >= v + 3)
		{
			cout << "Old route[" << warehouse << "][" << t << "][" << k << "]: [";
			for (int i = 0; i < route.size(); ++i)
			{
				if (i != route.size() - 1)
				{
					cout << route[i] << " -> ";
				}
				else
				{
					cout << route[i];
				}
			}
			cout << "]" << endl;
			cout << "route size = " << route.size() << endl;

			routeFound = true;
			break;
		}
		++attempts;
	}

	if (!routeFound)
	{
		cerr << "Could not find a route with " << v + 3 << " nodes" << endl;
		return false;
	}
	else
	{
		// Choose random starting position for the segment to move
		int startPos = 1 + rand() % (route.size() - v - 1); // Avoid first and last positions
		int newPos;

		// Ensure the new position is not the same as the current position
		do
		{
			newPos = 1 + rand() % (route.size() - v - 1);
		} while (newPos == startPos);

		// Move the segment of v customers to a new position
		vector<int> temp(route.begin() + startPos, route.begin() + startPos + v);
		route.erase(route.begin() + startPos, route.begin() + startPos + v);
		route.insert(route.begin() + newPos, temp.begin(), temp.end());

		routes[t][k] = route;

		cout << "Moved " << v << " customers from position " << startPos
			 << " to position " << newPos << " in route " << k << std::endl;

		cout << "Updated route[" << warehouse << "][" << t << "][" << k << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;
	}
	return true;
}

bool LocalSearch::Shift_One(vector<vector<vector<int>>> &routes)
{
	int v = 1;
	cout << "Shift(" << v << ")" << endl;

	const int maxAttempts = 5 * params.numPeriods * params.numVehicles_Warehouse;
	int attempts = 0;
	bool validShiftFound = false;

	while (attempts < maxAttempts && !validShiftFound)
	{
		int t = rand() % params.numPeriods;
		int sourceRouteIndex = rand() % params.numVehicles_Warehouse;
		int destRouteIndex = rand() % params.numVehicles_Warehouse;

		// Avoid shifting within the same route
		if (sourceRouteIndex == destRouteIndex)
		{
			++attempts;
			continue;
		}

		vector<int> &sourceRoute = routes[t][sourceRouteIndex];
		vector<int> &destRoute = routes[t][destRouteIndex];

		if (sourceRoute.size() == 0)
		{
			++attempts;
			continue;
		}

		// Check if source route has enough customers to shift
		if (sourceRoute.size() >= 2 + v)
		{
			cout << "Old Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
			for (int i = 0; i < sourceRoute.size(); ++i)
			{
				if (i != sourceRoute.size() - 1)
				{
					cout << sourceRoute[i] << " -> ";
				}
				else
				{
					cout << sourceRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Old Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
			for (int i = 0; i < destRoute.size(); ++i)
			{
				if (i != destRoute.size() - 1)
				{
					cout << destRoute[i] << " -> ";
				}
				else
				{
					cout << destRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Source route has enough customers to shift" << endl;
			int startPos = (rand() % (sourceRoute.size() - v - 1)) + 1;
			int endPos = startPos + v;

			vector<int> segment(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
			int insertPos = 0;
			if (destRoute.size() != 0)
			{
				insertPos = (rand() % (destRoute.size() - 1)) + 1;
			}

			// Execute the shift
			sourceRoute.erase(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
			destRoute.insert(destRoute.begin() + insertPos, segment.begin(), segment.end());
			if (sourceRoute.size() == 2)
			{
				sourceRoute.clear();
			}

			if (destRoute.size() == v)
			{
				destRoute.insert(destRoute.begin(), warehouse);
				destRoute.insert(destRoute.end(), warehouse);
			}

			cout << "Shifted " << v << " customers from route " << sourceRouteIndex
				 << " to route " << destRouteIndex << " in period " << t << endl;

			cout << "Updated Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
			for (int i = 0; i < sourceRoute.size(); ++i)
			{
				if (i != sourceRoute.size() - 1)
				{
					cout << sourceRoute[i] << " -> ";
				}
				else
				{
					cout << sourceRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Updated Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
			for (int i = 0; i < destRoute.size(); ++i)
			{
				if (i != destRoute.size() - 1)
				{
					cout << destRoute[i] << " -> ";
				}
				else
				{
					cout << destRoute[i];
				}
			}
			cout << "]" << endl;

			validShiftFound = true;
		}

		++attempts;
	}

	if (!validShiftFound)
	{
		cerr << "Failed to find a valid shift after " << maxAttempts << " attempts." << endl;
		return false;
	}

	return true;
}

bool LocalSearch::Shift_Two(vector<vector<vector<int>>> &routes)
{
	int v = 2;
	cout << "Shift(" << v << ")" << endl;

	const int maxAttempts = 5 * params.numPeriods * params.numVehicles_Warehouse;
	int attempts = 0;
	bool validShiftFound = false;

	while (attempts < maxAttempts && !validShiftFound)
	{
		int t = rand() % params.numPeriods;
		int sourceRouteIndex = rand() % params.numVehicles_Warehouse;
		int destRouteIndex = rand() % params.numVehicles_Warehouse;

		// Avoid shifting within the same route
		if (sourceRouteIndex == destRouteIndex)
		{
			++attempts;
			continue;
		}

		vector<int> &sourceRoute = routes[t][sourceRouteIndex];
		vector<int> &destRoute = routes[t][destRouteIndex];

		if (sourceRoute.size() == 0)
		{
			++attempts;
			continue;
		}

		// Check if source route has enough customers to shift
		if (sourceRoute.size() >= 2 + v)
		{
			cout << "Old Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
			for (int i = 0; i < sourceRoute.size(); ++i)
			{
				if (i != sourceRoute.size() - 1)
				{
					cout << sourceRoute[i] << " -> ";
				}
				else
				{
					cout << sourceRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Old Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
			for (int i = 0; i < destRoute.size(); ++i)
			{
				if (i != destRoute.size() - 1)
				{
					cout << destRoute[i] << " -> ";
				}
				else
				{
					cout << destRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Source route has enough customers to shift" << endl;
			int startPos = (rand() % (sourceRoute.size() - v - 1)) + 1;
			int endPos = startPos + v;

			vector<int> segment(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
			int insertPos = 0;
			if (destRoute.size() != 0)
			{
				insertPos = (rand() % (destRoute.size() - 1)) + 1;
			}

			// Execute the shift
			sourceRoute.erase(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
			destRoute.insert(destRoute.begin() + insertPos, segment.begin(), segment.end());
			if (sourceRoute.size() == 2)
			{
				sourceRoute.clear();
			}

			if (destRoute.size() == v)
			{
				destRoute.insert(destRoute.begin(), warehouse);
				destRoute.insert(destRoute.end(), warehouse);
			}

			cout << "Shifted " << v << " customers from route " << sourceRouteIndex
				 << " to route " << destRouteIndex << " in period " << t << endl;

			cout << "Updated Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
			for (int i = 0; i < sourceRoute.size(); ++i)
			{
				if (i != sourceRoute.size() - 1)
				{
					cout << sourceRoute[i] << " -> ";
				}
				else
				{
					cout << sourceRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Updated Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
			for (int i = 0; i < destRoute.size(); ++i)
			{
				if (i != destRoute.size() - 1)
				{
					cout << destRoute[i] << " -> ";
				}
				else
				{
					cout << destRoute[i];
				}
			}
			cout << "]" << endl;

			validShiftFound = true;
		}

		++attempts;
	}

	if (!validShiftFound)
	{
		cerr << "Failed to find a valid shift after " << maxAttempts << " attempts." << endl;
		return false;
	}

	return true;
}

bool LocalSearch::Shift_Three(vector<vector<vector<int>>> &routes)
{
	int v = 3;
	cout << "Shift(" << v << ")" << endl;

	const int maxAttempts = 5 * params.numPeriods * params.numVehicles_Warehouse;
	int attempts = 0;
	bool validShiftFound = false;

	while (attempts < maxAttempts && !validShiftFound)
	{
		int t = rand() % params.numPeriods;
		int sourceRouteIndex = rand() % params.numVehicles_Warehouse;
		int destRouteIndex = rand() % params.numVehicles_Warehouse;

		// Avoid shifting within the same route
		if (sourceRouteIndex == destRouteIndex)
		{
			++attempts;
			continue;
		}

		vector<int> &sourceRoute = routes[t][sourceRouteIndex];
		vector<int> &destRoute = routes[t][destRouteIndex];

		if (sourceRoute.size() == 0)
		{
			++attempts;
			continue;
		}

		// Check if source route has enough customers to shift
		if (sourceRoute.size() >= 2 + v)
		{
			cout << "Old Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
			for (int i = 0; i < sourceRoute.size(); ++i)
			{
				if (i != sourceRoute.size() - 1)
				{
					cout << sourceRoute[i] << " -> ";
				}
				else
				{
					cout << sourceRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Old Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
			for (int i = 0; i < destRoute.size(); ++i)
			{
				if (i != destRoute.size() - 1)
				{
					cout << destRoute[i] << " -> ";
				}
				else
				{
					cout << destRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Source route has enough customers to shift" << endl;
			int startPos = (rand() % (sourceRoute.size() - v - 1)) + 1;
			int endPos = startPos + v;

			vector<int> segment(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
			int insertPos = 0;
			if (destRoute.size() != 0)
			{
				insertPos = (rand() % (destRoute.size() - 1)) + 1;
			}

			// Execute the shift
			sourceRoute.erase(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
			destRoute.insert(destRoute.begin() + insertPos, segment.begin(), segment.end());
			if (sourceRoute.size() == 2)
			{
				sourceRoute.clear();
			}

			if (destRoute.size() == v)
			{
				destRoute.insert(destRoute.begin(), warehouse);
				destRoute.insert(destRoute.end(), warehouse);
			}

			cout << "Shifted " << v << " customers from route " << sourceRouteIndex
				 << " to route " << destRouteIndex << " in period " << t << endl;

			cout << "Updated Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
			for (int i = 0; i < sourceRoute.size(); ++i)
			{
				if (i != sourceRoute.size() - 1)
				{
					cout << sourceRoute[i] << " -> ";
				}
				else
				{
					cout << sourceRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Updated Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
			for (int i = 0; i < destRoute.size(); ++i)
			{
				if (i != destRoute.size() - 1)
				{
					cout << destRoute[i] << " -> ";
				}
				else
				{
					cout << destRoute[i];
				}
			}
			cout << "]" << endl;

			validShiftFound = true;
		}

		++attempts;
	}

	if (!validShiftFound)
	{
		cerr << "Failed to find a valid shift after " << maxAttempts << " attempts." << endl;
		return false;
	}

	return true;
}

bool LocalSearch::Swap(vector<vector<vector<int>>> &routes)
{
	cout << "Swap" << endl;

	const int maxAttempts = 100;
	int attempts = 0;
	bool validSwapFound = false;

	while (attempts < maxAttempts && !validSwapFound)
	{
		int t = rand() % params.numPeriods;
		int sourceRouteIndex = rand() % params.numVehicles_Warehouse;
		int destRouteIndex = rand() % params.numVehicles_Warehouse;

		// Avoid swapping within the same route
		if (sourceRouteIndex == destRouteIndex)
		{
			++attempts;
			continue;
		}

		vector<int> &sourceRoute = routes[t][sourceRouteIndex];
		vector<int> &destRoute = routes[t][destRouteIndex];

		// cout << "Old Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
		// for (int i = 0; i < sourceRoute.size(); ++i)
		// {
		// 	if (i != sourceRoute.size() - 1)
		// 	{
		// 		cout << sourceRoute[i] << " -> ";
		// 	}
		// 	else
		// 	{
		// 		cout << sourceRoute[i];
		// 	}
		// }
		// cout << "]" << endl;

		// cout << "Old Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
		// for (int i = 0; i < destRoute.size(); ++i)
		// {
		// 	if (i != destRoute.size() - 1)
		// 	{
		// 		cout << destRoute[i] << " -> ";
		// 	}
		// 	else
		// 	{
		// 		cout << destRoute[i];
		// 	}
		// }
		// cout << "]" << endl;

		if (sourceRoute.size() < 4 || destRoute.size() < 3)
		{
			++attempts;
			continue;
		}

		// Randomly select v1 and v2 where v1 >= v2
		int v1 = (rand() % 2) + 1;				   // v1 ∈ {1, 2}
		int v2 = (v1 == 2) ? (rand() % 2) + 1 : 1; // v2 ∈ {1, v1}

		if (sourceRoute.size() >= v1 + 2 && destRoute.size() >= v2 + 2)
		{
			int sourcePos = (rand() % (sourceRoute.size() - v1 - 1)) + 1;
			int destPos = (rand() % (destRoute.size() - v2 - 1)) + 1;

			// Extract segments
			vector<int> sourceSegment(sourceRoute.begin() + sourcePos, sourceRoute.begin() + sourcePos + v1);
			vector<int> destSegment(destRoute.begin() + destPos, destRoute.begin() + destPos + v2);

			sourceRoute.erase(sourceRoute.begin() + sourcePos, sourceRoute.begin() + sourcePos + v1);
			destRoute.erase(destRoute.begin() + destPos, destRoute.begin() + destPos + v2);

			sourceRoute.insert(sourceRoute.begin() + sourcePos, destSegment.begin(), destSegment.end());
			destRoute.insert(destRoute.begin() + destPos, sourceSegment.begin(), sourceSegment.end());

			cout << "Swapped " << v1 << " customers from route " << sourceRouteIndex
				 << " with " << v2 << " customers from route " << destRouteIndex
				 << " in period " << t << endl;

			cout << "Updated Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
			for (int i = 0; i < sourceRoute.size(); ++i)
			{
				if (i != sourceRoute.size() - 1)
				{
					cout << sourceRoute[i] << " -> ";
				}
				else
				{
					cout << sourceRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Updated Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
			for (int i = 0; i < destRoute.size(); ++i)
			{
				if (i != destRoute.size() - 1)
				{
					cout << destRoute[i] << " -> ";
				}
				else
				{
					cout << destRoute[i];
				}
			}
			cout << "]" << endl;

			validSwapFound = true;
		}

		++attempts;
	}

	if (!validSwapFound)
	{
		cerr << "Failed to find a valid swap after " << maxAttempts << " attempts." << endl;
		return false;
	}

	return true;
}

bool LocalSearch::Insert(vector<vector<vector<int>>> &routes)
{
	cout << "Insert" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period
	std::unordered_set<int> visitedNodes;
	vector<int> unvisitedNodes;

	// Gather all visited nodes in the selected period across all vehicles
	int rtIndex = 0;
	for (auto &route : routes[t])
	{
		if (route.size() > 2)
		{
			for (int nodeInd = 1; nodeInd < route.size() - 1; ++nodeInd)
			{
				visitedNodes.insert(route[nodeInd]);
			}
		}

		cout << "Old Route[" << warehouse << "][" << t << "][" << rtIndex << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;

		rtIndex++;
	}

	// Identify unvisited nodes
	for (int i = 0; i < params.numCustomers; ++i) { 
		if (CATW[scenario][t][warehouse][i] == 1) {
			if (visitedNodes.find(i + params.numWarehouses) == visitedNodes.end())
			{
				unvisitedNodes.push_back(i + params.numWarehouses);
			}
		
		}
	}

	// If no unvisited nodes are found, return false
	if (unvisitedNodes.empty())
	{
		cout << "No unvisited nodes available for insertion." << endl;
		return false;
	}

	// Randomly select an unvisited node from the list
	int unvisitedNode = unvisitedNodes[rand() % unvisitedNodes.size()];

	// Attempt to insert the unvisited node at the minimum cost position
	bool inserted = false;
	double minInsertionCost = std::numeric_limits<double>::max();
	int minInsertionPos = -1;
	int routeToIns = -1;
	int routeInd = 0;

	for (auto &route : routes[t])
	{
		if (route.empty())
		{
			// For an empty route, calculate the cost of inserting the node between two warehouse nodes
			double insertionCost = 2 * params.transportationCost_SecondEchelon[warehouse][unvisitedNode];
			if (insertionCost < minInsertionCost)
			{
				minInsertionCost = insertionCost;
				minInsertionPos = 0;
				routeToIns = routeInd;
			}
		}
		else
		{
			for (int pos = 1; pos < route.size(); ++pos)
			{
				double insertionCost = params.transportationCost_SecondEchelon[route[pos - 1]][unvisitedNode] +
									   params.transportationCost_SecondEchelon[unvisitedNode][route[pos]] -
									   params.transportationCost_SecondEchelon[route[pos - 1]][route[pos]];

				if (insertionCost < minInsertionCost)
				{
					minInsertionCost = insertionCost;
					minInsertionPos = pos;
					routeToIns = routeInd;
				}
			}
		}
		routeInd++;
	}

	// Insert the node if a position was found
	if (routeToIns != -1 && !routes[t][routeToIns].empty())
	{
		routes[t][routeToIns].insert(routes[t][routeToIns].begin() + minInsertionPos, unvisitedNode);
		inserted = true;
	}
	else if (routeToIns != -1 && routes[t][routeToIns].empty())
	{
		// If the route is empty, add warehouse -> node -> warehouse
		routes[t][routeToIns].push_back(warehouse);
		routes[t][routeToIns].push_back(unvisitedNode);
		routes[t][routeToIns].push_back(warehouse);
		inserted = true;
	}

	if (!inserted)
	{
		cout << "Failed to insert unvisited node." << endl;
		return false;
	}

	rtIndex = 0;
	for (auto &route : routes[t])
	{
		cout << "New Route[" << warehouse << "][" << t << "][" << rtIndex << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;

		rtIndex++;
	}

	cout << "Inserted unvisited node " << unvisitedNode << " at period " << t << " with minimum cost." << endl;
	return true;
}

bool LocalSearch::Remove(vector<vector<vector<int>>> &routes)
{
	cout << "Remove" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period
	vector<int> candidateRoutes;

	int rtIndex = 0;
	for (auto &route : routes[t])
	{
		cout << "Old Route[" << warehouse << "][" << t << "][" << rtIndex << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;

		rtIndex++;
	}

	// Gather routes that have more than just the warehouse start and end node
	for (int i = 0; i < routes[t].size(); ++i)
	{
		if (routes[t][i].size() > 2)
		{ // Ensure there's more than just the warehouse nodes
			candidateRoutes.push_back(i);
		}
	}

	// If no suitable routes are found, return false
	if (candidateRoutes.empty())
	{
		cout << "No suitable routes with removable nodes found." << endl;
		return false;
	}

	// Randomly select a route from candidate routes
	int routeIndex = candidateRoutes[rand() % candidateRoutes.size()];
	auto &selectedRoute = routes[t][routeIndex];

	// Randomly select a customer node to remove, avoiding the warehouse nodes at the start and end if present
	int nodeIndex = rand() % (selectedRoute.size() - 2) + 1; // Avoid first and last positions if they are warehouses
	selectedRoute.erase(selectedRoute.begin() + nodeIndex);
	if (selectedRoute.size() == 2)
	{
		// Clear the route since it only contains the warehouse start and end nodes
		selectedRoute.clear();
	}

	rtIndex = 0;
	for (auto &route : routes[t])
	{
		cout << "New Route[" << warehouse << "][" << t << "][" << rtIndex << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;

		rtIndex++;
	}

	// Additional checks or operations can be performed here if necessary
	cout << "Removed a visited node from period " << t << ", route " << routeIndex << endl;
	return true;
}

bool LocalSearch::Merge(vector<vector<vector<int>>> &routes)
{
	cout << "Merge" << endl;

	// Map to store customer appearances across periods
	std::unordered_map<int, vector<int>> customerPeriods;

	// Gather customers and the periods they appear in
	for (int period = 0; period < params.numPeriods; ++period)
	{
		std::unordered_set<int> periodCustomers;
		for (const auto &route : routes[period])
		{
			for (int customer : route)
			{
				if (customer != warehouse)
				{
					periodCustomers.insert(customer);
				}
			}
		}
		for (int customer : periodCustomers)
		{
			customerPeriods[customer].push_back(period);
		}
	}

	// Find customers that appear in more than one period
	vector<int> eligibleCustomers;
	for (const auto &entry : customerPeriods)
	{
		if (entry.second.size() > 1)
		{
			eligibleCustomers.push_back(entry.first);
		}
	}

	if (eligibleCustomers.empty())
	{
		cout << "No eligible customers found for merging." << endl;
		return false;
	}

	// Select a random customer to merge
	int selectedCustomer = eligibleCustomers[rand() % eligibleCustomers.size()];
	const auto &periods = customerPeriods[selectedCustomer];

	// Randomly select two different periods to merge customer presence
	int mergeFrom = periods[rand() % periods.size()];
	int mergeTo;
	do
	{
		mergeTo = periods[rand() % periods.size()];
	} while (mergeTo == mergeFrom);

	cout << "Merging customer " << selectedCustomer << " from period " << mergeFrom << " to period " << mergeTo << endl;

	int mergeFromRouteIndex = -1;
	int mergeToRouteIndex = -1;

	// Find the route index in mergeFrom period containing the selected customer
	for (int i = 0; i < routes[mergeFrom].size(); ++i)
	{
		auto it = std::find(routes[mergeFrom][i].begin(), routes[mergeFrom][i].end(), selectedCustomer);
		if (it != routes[mergeFrom][i].end())
		{
			mergeFromRouteIndex = i;
			break;
		}
	}

	if (mergeFromRouteIndex == -1)
	{
		cout << "Error: Customer not found in mergeFrom period." << endl;
		return false;
	}

	cout << "Old Route (Merge From)[" << warehouse << "][" << mergeFrom << "][" << mergeFromRouteIndex << "]: [";
	auto &fromRoute = routes[mergeFrom][mergeFromRouteIndex];
	for (int i = 0; i < fromRoute.size(); ++i)
	{
		if (i != fromRoute.size() - 1)
		{
			cout << fromRoute[i] << " -> ";
		}
		else
		{
			cout << fromRoute[i];
		}
	}
	cout << "]" << endl;

	// Find the route index in mergeTo period where the customer could be merged
	for (int i = 0; i < routes[mergeTo].size(); ++i)
	{
		auto it = std::find(routes[mergeTo][i].begin(), routes[mergeTo][i].end(), selectedCustomer);
		if (it != routes[mergeTo][i].end())
		{
			mergeToRouteIndex = i;
			break;
		}
	}

	// If the customer is not already in mergeTo, add them
	if (mergeToRouteIndex == -1)
	{
		mergeToRouteIndex = 0; // For simplicity, insert into the first route
		routes[mergeTo][mergeToRouteIndex].push_back(selectedCustomer);
	}

	cout << "Old Route (Merge To)[" << warehouse << "][" << mergeTo << "][" << mergeToRouteIndex << "]: [";
	auto &toRoute = routes[mergeTo][mergeToRouteIndex];
	for (int i = 0; i < toRoute.size(); ++i)
	{
		if (i != toRoute.size() - 1)
		{
			cout << toRoute[i] << " -> ";
		}
		else
		{
			cout << toRoute[i];
		}
	}
	cout << "]" << endl;

	// Remove customer from mergeFrom period
	fromRoute.erase(std::remove(fromRoute.begin(), fromRoute.end(), selectedCustomer), fromRoute.end());

	// If route is empty (only warehouse nodes), clear it
	if (fromRoute.size() == 2)
	{
		fromRoute.clear();
	}

	// Output new routes after the merge
	cout << "New Route (Merge From)[" << warehouse << "][" << mergeFrom << "][" << mergeFromRouteIndex << "]: [";
	for (int i = 0; i < fromRoute.size(); ++i)
	{
		if (i != fromRoute.size() - 1)
		{
			cout << fromRoute[i] << " -> ";
		}
		else
		{
			cout << fromRoute[i];
		}
	}
	cout << "]" << endl;

	cout << "New Route (Merge To)[" << warehouse << "][" << mergeTo << "][" << mergeToRouteIndex << "]: [";
	for (int i = 0; i < toRoute.size(); ++i)
	{
		if (i != toRoute.size() - 1)
		{
			cout << toRoute[i] << " -> ";
		}
		else
		{
			cout << toRoute[i];
		}
	}
	cout << "]" << endl;

	return true;
}

bool LocalSearch::Transfer(vector<vector<vector<int>>> &routes)
{
	cout << "Transfer" << endl;

	// Randomly select a customer who is assigned to the warehouse in any period for the given scenario
	vector<int> possibleCustomers;
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i) { 
			if (CATW[scenario][t][warehouse][i] == 1) {
				if (std::find(possibleCustomers.begin(), possibleCustomers.end(), i) == possibleCustomers.end())
				{
					possibleCustomers.push_back(i);
				}
			}
		}
	}

	if (possibleCustomers.empty())
	{
		cout << "No customers assigned to warehouse " << warehouse << " in any period for scenario " << scenario << endl;
		return false;
	}

	// Select a random customer from the list of possible customers
	int customer = possibleCustomers[rand() % possibleCustomers.size()] + params.numWarehouses;

	vector<int> visitedPeriods;
	vector<int> unvisitedPeriods;

	// Determine where the customer is visited and where they are not,
	// considering only periods where the customer is assigned to this warehouse
	for (int t = 0; t < params.numPeriods; ++t)
	{
		// Check if the customer is assigned to this warehouse in the current period
		if (CATW[scenario][t][warehouse][customer - params.numWarehouses] == 1) {
			bool found = false;
			for (auto &route : routes[t])
			{
				if (std::find(route.begin(), route.end(), customer) != route.end())
				{
					found = true;
					visitedPeriods.push_back(t);
					break;
				}
			}
			if (!found)
			{
				unvisitedPeriods.push_back(t);
			}
		}
	}

	// Remove the customer from all periods they are currently visited,
	// considering only periods where the customer is assigned to this warehouse
	for (int period : visitedPeriods)
	{
		for (auto &route : routes[period])
		{
			auto it = std::find(route.begin(), route.end(), customer);
			while (it != route.end())
			{ // Ensure all instances of the customer are removed
				route.erase(it);
				it = std::find(route.begin(), route.end(), customer);
			}

			if (route.size() == 2)
			{
				// Clear the route since it only contains the warehouse start and end nodes
				route.clear();
			}
		}
	}

	// Attempt to insert the customer into all periods they are not visited,
	// considering only periods where the customer is assigned to this warehouse
	bool inserted = false;
	for (int t : unvisitedPeriods)
	{
		double minInsertionCost = std::numeric_limits<double>::max();
		int minInsertionPos = -1;
		int routeToIns = -1;
		int routeInd = 0;

		for (auto &route : routes[t])
		{
			if (route.empty())
			{
				double insertionCost = 2 * params.transportationCost_SecondEchelon[warehouse][customer];
				if (insertionCost < minInsertionCost)
				{
					minInsertionCost = insertionCost;
					minInsertionPos = 0;
					routeToIns = routeInd;
				}
			}
			else
			{
				for (int pos = 1; pos < route.size(); ++pos)
				{
					double insertionCost = params.transportationCost_SecondEchelon[route[pos - 1]][customer] +
										   params.transportationCost_SecondEchelon[customer][route[pos]] -
										   params.transportationCost_SecondEchelon[route[pos - 1]][route[pos]];

					if (insertionCost < minInsertionCost)
					{
						minInsertionCost = insertionCost;
						minInsertionPos = pos;
						routeToIns = routeInd;
					}
				}
			}
			routeInd++;
		}

		// Insert the customer if a position was found
		if (routeToIns != -1 && !routes[t][routeToIns].empty())
		{
			routes[t][routeToIns].insert(routes[t][routeToIns].begin() + minInsertionPos, customer);
			inserted = true;
		}
		else if (routeToIns != -1 && routes[t][routeToIns].empty())
		{
			routes[t][routeToIns].push_back(warehouse);
			routes[t][routeToIns].push_back(customer);
			routes[t][routeToIns].push_back(warehouse);
			inserted = true;
		}
	}

	return inserted;
}

bool LocalSearch::Remove_Insert(vector<vector<vector<int>>> &routes)
{
	cout << "Remove/Insert" << endl;

	// Step 1: Find a period with at least one customer and randomly select one
	vector<int> visitedPeriods;
	for (int period = 0; period < params.numPeriods; ++period)
	{
		std::unordered_set<int> uniqueCustomers;
		for (const auto &route : routes[period])
		{
			for (int node : route)
			{
				if (node != warehouse)
				{ // Assuming 'w' is the warehouse node
					uniqueCustomers.insert(node);
				}
			}
		}
		if (!uniqueCustomers.empty())
		{
			visitedPeriods.push_back(period);
		}
	}

	if (visitedPeriods.empty())
	{
		cout << "No periods with customers found." << endl;
		return false;
	}

	int fromPeriod = visitedPeriods[rand() % visitedPeriods.size()];
	std::unordered_set<int> customersInFromPeriod;
	for (const auto &route : routes[fromPeriod])
	{
		for (int customer : route)
		{
			if (customer != warehouse)
			{
				customersInFromPeriod.insert(customer);
			}
		}
	}

	if (customersInFromPeriod.empty())
	{
		cout << "No customers in selected period to move." << endl;
		return false;
	}

	vector<int> customers(customersInFromPeriod.begin(), customersInFromPeriod.end());
	int selectedCustomer = customers[rand() % customers.size()];

	// Step 2: Identify a period where this customer is not visited
	vector<int> targetPeriods;
	for (int period = 0; period < params.numPeriods; ++period)
	{
		// Check if the customer is assigned to this warehouse in the current period
		if (period == fromPeriod || CATW[scenario][period][warehouse][selectedCustomer - params.numWarehouses] == 1)
			continue;
		bool found = false;
		for (const auto &route : routes[period])
		{
			if (std::find(route.begin(), route.end(), selectedCustomer) != route.end())
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			targetPeriods.push_back(period);
		}
	}

	if (targetPeriods.empty())
	{
		cout << "No eligible periods to move the customer to." << endl;
		return false;
	}

	int toPeriod = targetPeriods[rand() % targetPeriods.size()];

	// Step 3: Remove the customer from the original period
	for (auto &route : routes[fromPeriod])
	{
		auto it = std::find(route.begin(), route.end(), selectedCustomer);
		if (it != route.end())
		{
			route.erase(it);
			if (route.size() == 2)
			{
				// Clear the route since it only contains the warehouse start and end nodes
				route.clear();
			}
			break; // Assuming each customer only appears once per route
		}
	}

	// Step 4: Insert the customer into the new period using minimal insertion cost strategy
	double minInsertionCost = std::numeric_limits<double>::max();
	int minInsertionPos = -1;
	int routeToIns = -1;
	int routeInd = 0;

	for (auto &route : routes[toPeriod])
	{
		if (route.empty())
		{
			double insertionCost = 2 * params.transportationCost_SecondEchelon[warehouse][selectedCustomer];
			if (insertionCost < minInsertionCost)
			{
				minInsertionCost = insertionCost;
				minInsertionPos = 0;
				routeToIns = routeInd;
			}
		}
		else
		{
			for (int pos = 1; pos < route.size(); ++pos)
			{
				double insertionCost = params.transportationCost_SecondEchelon[route[pos - 1]][selectedCustomer] +
									   params.transportationCost_SecondEchelon[selectedCustomer][route[pos]] -
									   params.transportationCost_SecondEchelon[route[pos - 1]][route[pos]];

				if (insertionCost < minInsertionCost)
				{
					minInsertionCost = insertionCost;
					minInsertionPos = pos;
					routeToIns = routeInd;
				}
			}
		}
		routeInd++;
	}

	// Insert the customer if a position was found
	if (routeToIns != -1 && !routes[toPeriod][routeToIns].empty())
	{
		routes[toPeriod][routeToIns].insert(routes[toPeriod][routeToIns].begin() + minInsertionPos, selectedCustomer);
	}
	else if (routeToIns != -1 && routes[toPeriod][routeToIns].empty())
	{
		routes[toPeriod][routeToIns].push_back(warehouse);
		routes[toPeriod][routeToIns].push_back(selectedCustomer);
		routes[toPeriod][routeToIns].push_back(warehouse);
	}

	cout << "Moved customer " << selectedCustomer << " from period " << fromPeriod << " to period " << toPeriod << "." << endl;
	return true;
}

double LocalSearch::calculateObjFuncValue_ScenarioWarehouse(const vector<double> &Inv_W_Temp,
															const vector<vector<double>> &Inv_Cust_Temp,
															const vector<vector<double>> &unmetDem_Cust_Temp,
															const vector<vector<vector<int>>> &route_WTC_Temp)
{
	// cout << "\ncalculating objFuncValue_ScenarioWarehouse" << endl;
	// for (int t = 0; t < params.numPeriods; ++t)
	// {
	// 	for (int i = 0; i < CATW[warehouse].size(); ++i)
	// 	{
	// 		if (Inv_Cust_Temp[i][t] > 0.0)
	// 		{
	// 			cout << "cust_Inv[" << i << "][" << t << "] = " << Inv_Cust_Temp[i][t] << endl;
	// 		}

	// 		if (unmetDem_Cust_Temp[i][t] > 0.0)
	// 		{
	// 			cout << "cust_unmDem[" << i << "][" << t << "] = " << unmetDem_Cust_Temp[i][t] << endl;
	// 		}
	// 	}
	// }

	double objFuncValue = 0.0;

	// double inventoryCost = 0.0;
	// double unmetDemandCost = 0.0;
	// double routeCost_Warehouse = 0.0;

	// Inventory Holding Cost and Unmet Demand Cost
	for (int t = 0; t < params.numPeriods; ++t)
	{
		objFuncValue += params.unitHoldingCost_Warehouse[warehouse] * Inv_W_Temp[t];

		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[scenario][t][warehouse][i] == 1){
				objFuncValue += params.unitHoldingCost_Customer[i] * Inv_Cust_Temp[i][t];
				objFuncValue += params.unmetDemandPenalty[i] * unmetDem_Cust_Temp[i][t];

				// inventoryCost += params.unitHoldingCost_Customer[i] * Inv_Cust_Temp[i][t];
				// cout << params.unitHoldingCost_Customer[i] << " * " << Inv_Cust_Temp[i][t] << " = " << params.unitHoldingCost_Customer[i] * Inv_Cust_Temp[i][t] << endl;
				// unmetDemandCost += params.unmetDemandPenalty[i] * unmetDem_Cust_Temp[i][t];
			}
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			// double routeCost = 0.0;

			int previousNode = warehouse;
			for (int j = 1; j < route_WTC_Temp[t][k].size(); ++j)
			{
				int currentNode = route_WTC_Temp[t][k][j];
				objFuncValue += params.transportationCost_SecondEchelon[previousNode][currentNode];

				// routeCost += params.transportationCost_SecondEchelon[previousNode][currentNode];

				// routeCost_Warehouse += params.transportationCost_SecondEchelon[previousNode][currentNode];
				previousNode = currentNode;
			}

			// cout << "Route[" << warehouse << "][" << t << "][" << k << "]: [";
			// for (int i = 0; i < route_WTC_Temp[t][k].size(); ++i)
			// {
			// 	if (i != route_WTC_Temp[t][k].size() - 1)
			// 	{
			// 		cout << route_WTC_Temp[t][k][i] << " -> ";
			// 	}
			// 	else
			// 	{
			// 		cout << route_WTC_Temp[t][k][i];
			// 	}
			// }
			// cout << "]" << endl;

			// cout << "Route Cost(RVND)[" << s << "][" << warehouse << "][" << t << "][" << k << "]: " << routeCost << endl;
		}
	}

	// cout << "Inventory Cost (RVND)[" << s << "][" << warehouse << "]: " << inventoryCost << endl;
	// cout << "Unmet Demand Cost (RVND)[" << s << "][" << warehouse << "]: " << unmetDemandCost << endl;
	// cout << "Route Cost (RVND)[" << s << "][" << warehouse << "]: " << routeCost_Warehouse << endl;

	cout << "objFuncValue:" << objFuncValue << endl;
	return objFuncValue;
}

// --------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
Perturbation::Perturbation(const int s, const int w, const ParameterSetting &parameters, const Solution &solution, const vector<vector<vector<vector<int>>>> &custToWarehouse)
	: scenario(s), warehouse(warehouse), params(parameters), sol_FE(solution), CATW(custToWarehouse)
{
	// ----------------------------------------------------------------------------------------------------------
	cout << "\nInitializing Perturbation for scenario " << scenario << " and warehouse " << warehouse << endl;
	// ----------------------------------------------------------------------------------------------------------
}

bool Perturbation::run(vector<vector<double>> &Inv_Cust_ScenWare,
					   vector<vector<double>> &unmetDemand_Cust_ScenWare,
					   vector<vector<double>> &delQuant_Cust_ScenWare,
					   vector<vector<vector<int>>> &routes_wareToCust_ScenWare)
{
	// --------------------------------------------------------------------------------------------------------------------------
	// objValue_ScenarioWarehouse = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Cust_ScenWare, unmetDemand_Cust_ScenWare, routes_wareToCust_ScenWare);
	// cout << "\nCurrent Objective Value for scenario " << s << " and warehouse " << warehouse << ": " << objValue_ScenarioWarehouse << endl;

	// Initialize the operators
	vector<std::function<bool(vector<vector<vector<int>>> &)>> perturbOperators = setPerturbOperators();
	int max_perturb = 10;

	while (max_perturb > 0)
	{
		vector<vector<vector<int>>> routes_wareToCust_ScenWare_temp = routes_wareToCust_ScenWare;
		vector<vector<double>> Inv_Cust_ScenWare_temp = Inv_Cust_ScenWare;
		vector<vector<double>> unmetDemand_Cust_ScenWare_temp = unmetDemand_Cust_ScenWare;
		vector<vector<double>> delQuant_Cust_ScenWare_temp = delQuant_Cust_ScenWare;

		// double objValue_ScenarioWarehouse_temp = 0.0;
		// Generate random number between 0 and operators.size()
		int index = rand() % perturbOperators.size();

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				if (!routes_wareToCust_ScenWare_temp[t][k].empty())
				{
					cout << "Route[" << scenario << "][" << warehouse << "][" << t << "][" << k << "]: [";
					for (int i = 0; i < routes_wareToCust_ScenWare_temp[t][k].size(); ++i)
					{
						if (i != routes_wareToCust_ScenWare_temp[t][k].size() - 1)
						{
							cout << routes_wareToCust_ScenWare_temp[t][k][i] << " -> ";
						}
						else
						{
							cout << routes_wareToCust_ScenWare_temp[t][k][i];
						}
					}
					cout << "]" << endl;
				}
			}
		}
		cout << "\n";

		// Execute the operator
		if (!perturbOperators[index](routes_wareToCust_ScenWare_temp))
		{
			--max_perturb;
			continue;
		}

		LP_ScenarioWarehouse LP(scenario, warehouse, CATW, params, sol_FE, routes_wareToCust_ScenWare_temp);
		// solve LP to get the value of the coninuous variables
		string status = LP.solve();
		if (status != "Optimal")
		{
			--max_perturb;
			continue;
		}
		Inv_Cust_ScenWare = LP.getInvCustomers();
		unmetDemand_Cust_ScenWare = LP.getUnmetDemandCustomers();
		delQuant_Cust_ScenWare = LP.getDeliveryQuantityCustomers();
		routes_wareToCust_ScenWare = routes_wareToCust_ScenWare_temp;

		// for (int t = 0; t < params.numPeriods; ++t)
		// {
		// 	for (size_t i = 0; i < CATW[warehouse].size(); ++i)
		// 	{
		// 		cout << "Inv_Cust_temp[" << CATW[warehouse][i] << "][" << t <<  "][" << s << "] = " << Inv_Cust_ScenWare_temp[i][t] << endl;
		// 		cout << "unmetDemand_Cust_temp[" << CATW[warehouse][i] << "][" << t <<  "][" << s << "] = " << unmetDemand_Cust_ScenWare_temp[i][t] << endl;
		// 		cout << "delQuant_Cust_temp[" << CATW[warehouse][i] << "][" << t <<  "][" << s << "] = " << delQuant_Cust_ScenWare_temp[i][t] << endl;
		// 	}
		// }

		// double objValue_ScenarioWarehouse_temp = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Cust_ScenWare_temp, unmetDemand_Cust_ScenWare_temp, routes_wareToCust_ScenWare_temp);
		// cout << "Current objective value: " << objValue_ScenarioWarehouse_temp << "\n" << endl;

		return true;
	}
	return false;
}

vector<std::function<bool(vector<vector<vector<int>>> &)>> Perturbation::setPerturbOperators()
{
	std::function<bool(vector<vector<vector<int>>> &)> randomShiftFunc = std::bind(&Perturbation::randomShift, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> randomInsertionFunc = std::bind(&Perturbation::randomInsertion, this, std::placeholders::_1);
	std::function<bool(vector<vector<vector<int>>> &)> randomRemovalFunc = std::bind(&Perturbation::randomRemoval, this, std::placeholders::_1);

	return {
		randomShiftFunc,
		randomInsertionFunc,
		randomRemovalFunc};
}

bool Perturbation::randomShift(vector<vector<vector<int>>> &routes)
{
	int v = 1;
	cout << "Random Shift Perturbation(" << v << ")" << endl;

	const int maxAttempts = 10;
	int attempts = 0;
	bool validShiftFound = false;

	while (attempts < maxAttempts && !validShiftFound)
	{
		int t = rand() % params.numPeriods;
		int sourceRouteIndex = rand() % params.numVehicles_Warehouse;
		int destRouteIndex = rand() % params.numVehicles_Warehouse;

		// Avoid shifting within the same route
		if (sourceRouteIndex == destRouteIndex)
		{
			++attempts;
			continue;
		}

		vector<int> &sourceRoute = routes[t][sourceRouteIndex];
		vector<int> &destRoute = routes[t][destRouteIndex];

		if (sourceRoute.size() == 0)
		{
			++attempts;
			continue;
		}

		cout << "Old Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
		for (int i = 0; i < sourceRoute.size(); ++i)
		{
			if (i != sourceRoute.size() - 1)
			{
				cout << sourceRoute[i] << " -> ";
			}
			else
			{
				cout << sourceRoute[i];
			}
		}
		cout << "]" << endl;

		cout << "Old Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
		for (int i = 0; i < destRoute.size(); ++i)
		{
			if (i != destRoute.size() - 1)
			{
				cout << destRoute[i] << " -> ";
			}
			else
			{
				cout << destRoute[i];
			}
		}
		cout << "]" << endl;

		// Check if source route has enough customers to shift
		if (sourceRoute.size() >= 2 + v)
		{
			cout << "Source route has enough customers to shift" << endl;
			int startPos = (rand() % (sourceRoute.size() - v - 1)) + 1;
			int endPos = startPos + v;

			vector<int> segment(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
			int insertPos = 0;
			if (destRoute.size() != 0)
			{
				insertPos = (rand() % (destRoute.size() - 1)) + 1;
			}

			// Execute the shift
			sourceRoute.erase(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
			destRoute.insert(destRoute.begin() + insertPos, segment.begin(), segment.end());
			if (sourceRoute.size() == 2)
			{
				sourceRoute.clear();
			}

			if (destRoute.size() == v)
			{
				destRoute.insert(destRoute.begin(), warehouse);
				destRoute.insert(destRoute.end(), warehouse);
			}

			cout << "Shifted " << v << " customers from route " << sourceRouteIndex
				 << " to route " << destRouteIndex << " in period " << t << endl;

			cout << "Updated Source route[" << warehouse << "][" << t << "][" << sourceRouteIndex << "]: [";
			for (int i = 0; i < sourceRoute.size(); ++i)
			{
				if (i != sourceRoute.size() - 1)
				{
					cout << sourceRoute[i] << " -> ";
				}
				else
				{
					cout << sourceRoute[i];
				}
			}
			cout << "]" << endl;

			cout << "Updated Dest route[" << warehouse << "][" << t << "][" << destRouteIndex << "]: [";
			for (int i = 0; i < destRoute.size(); ++i)
			{
				if (i != destRoute.size() - 1)
				{
					cout << destRoute[i] << " -> ";
				}
				else
				{
					cout << destRoute[i];
				}
			}
			cout << "]" << endl;

			validShiftFound = true;
		}

		++attempts;
	}

	if (!validShiftFound)
	{
		cerr << "Failed to find a valid shift after " << maxAttempts << " attempts." << endl;
		return false;
	}

	return true;
}

bool Perturbation::randomInsertion(vector<vector<vector<int>>> &routes)
{
	cout << "Random Insertion Perturbation" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period
	std::unordered_set<int> visitedNodes;
	vector<int> unvisitedNodes;

	// Gather all visited nodes in the selected period across all vehicles
	int rtIndex = 0;
	for (auto &route : routes[t])
	{
		if (route.size() > 2)
		{
			for (int nodeInd = 1; nodeInd < route.size() - 1; ++nodeInd)
			{
				visitedNodes.insert(route[nodeInd]);
			}
		}

		cout << "Old Route[" << warehouse << "][" << t << "][" << rtIndex << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;

		rtIndex++;
	}

	// Identify unvisited nodes
	for (int i = 0; i < params.numCustomers; ++i) {
		if (CATW[scenario][t][warehouse][i] == 1) {
			if (visitedNodes.find(i + params.numWarehouses) == visitedNodes.end())
			{
				unvisitedNodes.push_back(i + params.numWarehouses);
			}
		}
	}

	// If no unvisited nodes are found, return false
	if (unvisitedNodes.empty())
	{
		cout << "No unvisited nodes available for insertion." << endl;
		return false;
	}

	// Randomly select an unvisited node from the list
	int unvisitedNode = unvisitedNodes[rand() % unvisitedNodes.size()];

	// Attempt to insert the unvisited node at the minimum cost position
	bool inserted = false;
	double minInsertionCost = std::numeric_limits<double>::max();
	int minInsertionPos = -1;
	int routeToIns = -1;
	int routeInd = 0;

	for (auto &route : routes[t])
	{
		if (route.empty())
		{
			// For an empty route, calculate the cost of inserting the node between two warehouse nodes
			double insertionCost = 2 * params.transportationCost_SecondEchelon[warehouse][unvisitedNode];
			if (insertionCost < minInsertionCost)
			{
				minInsertionCost = insertionCost;
				minInsertionPos = 0;
				routeToIns = routeInd;
			}
		}
		else
		{
			for (int pos = 1; pos < route.size(); ++pos)
			{
				double insertionCost = params.transportationCost_SecondEchelon[route[pos - 1]][unvisitedNode] +
									   params.transportationCost_SecondEchelon[unvisitedNode][route[pos]] -
									   params.transportationCost_SecondEchelon[route[pos - 1]][route[pos]];

				if (insertionCost < minInsertionCost)
				{
					minInsertionCost = insertionCost;
					minInsertionPos = pos;
					routeToIns = routeInd;
				}
			}
		}
		routeInd++;
	}

	// Insert the node if a position was found
	if (routeToIns != -1 && !routes[t][routeToIns].empty())
	{
		routes[t][routeToIns].insert(routes[t][routeToIns].begin() + minInsertionPos, unvisitedNode);
		inserted = true;
	}
	else if (routeToIns != -1 && routes[t][routeToIns].empty())
	{
		// If the route is empty, add warehouse -> node -> warehouse
		routes[t][routeToIns].push_back(warehouse);
		routes[t][routeToIns].push_back(unvisitedNode);
		routes[t][routeToIns].push_back(warehouse);
		inserted = true;
	}

	if (!inserted)
	{
		cout << "Failed to insert unvisited node." << endl;
		return false;
	}

	rtIndex = 0;
	for (auto &route : routes[t])
	{
		cout << "New Route[" << warehouse << "][" << t << "][" << rtIndex << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;

		rtIndex++;
	}

	cout << "Inserted unvisited node " << unvisitedNode << " at period " << t << " with minimum cost." << endl;
	return true;
}

bool Perturbation::randomRemoval(vector<vector<vector<int>>> &routes)
{
	cout << "Random Removal Perturbation" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period
	vector<int> candidateRoutes;

	int rtIndex = 0;
	for (auto &route : routes[t])
	{
		cout << "Old Route[" << warehouse << "][" << t << "][" << rtIndex << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;

		rtIndex++;
	}

	// Gather routes that have more than just the warehouse start and end node
	for (int i = 0; i < routes[t].size(); ++i)
	{
		if (routes[t][i].size() > 2)
		{ // Ensure there's more than just the warehouse nodes
			candidateRoutes.push_back(i);
		}
	}

	// If no suitable routes are found, return false
	if (candidateRoutes.empty())
	{
		cout << "No suitable routes with removable nodes found." << endl;
		return false;
	}

	// Randomly select a route from candidate routes
	int routeIndex = candidateRoutes[rand() % candidateRoutes.size()];
	auto &selectedRoute = routes[t][routeIndex];

	// Randomly select a customer node to remove, avoiding the warehouse nodes at the start and end if present
	int nodeIndex = rand() % (selectedRoute.size() - 2) + 1; // Avoid first and last positions if they are warehouses
	selectedRoute.erase(selectedRoute.begin() + nodeIndex);
	if (selectedRoute.size() == 2)
	{
		// Clear the route since it only contains the warehouse start and end nodes
		selectedRoute.clear();
	}

	rtIndex = 0;
	for (auto &route : routes[t])
	{
		cout << "New Route[" << warehouse << "][" << t << "][" << rtIndex << "]: [";
		for (int i = 0; i < route.size(); ++i)
		{
			if (i != route.size() - 1)
			{
				cout << route[i] << " -> ";
			}
			else
			{
				cout << route[i];
			}
		}
		cout << "]" << endl;

		rtIndex++;
	}

	// Additional checks or operations can be performed here if necessary
	cout << "Removed a visited node from period " << t << ", route " << routeIndex << endl;
	return true;
}

// ------------------------------------------------------------------------------------------------------------------------
LP_SE::LP_SE(const vector<vector<vector<vector<int>>>> &custToWarehouse,
			 const ParameterSetting &parameters,
			 const Solution &solution,
			 vector<vector<vector<int>>> routes_wareToCust_ScenWare)
	: params(parameters),
	  sol_FE(solution),
	  routes_WareToCust_ScenWare(routes_wareToCust_ScenWare),
	  THRESHOLD(1e-2),
	  save_lpFile(true),
	  save_mpsResultFile(true)
{
	// ----------------------------------------------------------------------------------------------------------
	cout << "Solving LP for scenario " << scenario << " and warehouse " << warehouse << endl;

	ware_Inv.resize(params.numPeriods, 0.0);
	cust_Inv.resize(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	cust_unmDem.resize(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	cust_delQuant.resize(params.numCustomers, vector<double>(params.numPeriods, 0.0));

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[scenario][t][warehouse][i] == 1)
				cout << i + params.numWarehouses << " ";
		}
		cout << endl;
	}
}

string LP_ScenarioWarehouse::solve()
{
	IloEnv env;
	IloModel model(env);
	IloCplex cplex(model);

	bool save_lpFile = false;
	bool save_mpsResultFile = false;

	// -------------------------------------------------------------------------------------------------------------------------------
	// Define variables
	// -------------------------------------------------------------------------------------------------------------------------------
	// Initialize Variable Manager
	VariableManager varManager(env);
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define I_warehouse[t] variables - inventory level at warehouse w, period t and scenario s)
	IloNumVarArray I_warehouse = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "I_warehouse[" + std::to_string(warehouse) + "][" + std::to_string(t) + "]";
		I_warehouse[t] = IloNumVar(env, 0.0, params.storageCapacity_Warehouse[warehouse], IloNumVar::Float, varName.c_str());
		model.add(I_warehouse[t]);
	}
	
	// Define I[i][t][s] variables - inventory level at customer i, period t and scenario s (customers are assigned to warehouses -> i in N_w))
	IloArray<IloNumVarArray> I_customer = varManager.create2D(params.numCustomers, params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			string varName = "I_customer[" + std::to_string(i) + "][" + std::to_string(t) + "]";
			I_customer[i][t] = IloNumVar(env, 0.0, params.storageCapacity_Customer[i], IloNumVar::Float, varName.c_str());
			model.add(I_customer[i][t]);
		}
	}

	// Define b[i][t][s] variables - unmet demand at customer i, period t and scenario s (customers are assigned to warehouses -> i in N_w)
	IloArray<IloNumVarArray> b_customer = varManager.create2D(params.numCustomers, params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			string varName = "b_customer[" + std::to_string(i) + "][" + std::to_string(t) + "]";
			b_customer[i][t] = IloNumVar(env, 0.0, params.demand[i][t][scenario], IloNumVar::Float, varName.c_str());
			model.add(b_customer[i][t]);
		}
	}

	// Define w[i][t][k][s] variables - delivery to customer i, period t with vehicle k and scenario s (customers are assigned to warehouses -> i in N_w)
	IloArray<IloArray<IloNumVarArray>> w_customer = varManager.create3D(params.numCustomers, params.numPeriods, params.numVehicles_Warehouse);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				string varName = "w_customer[" + std::to_string(i) + "][" + std::to_string(t) + "][" + std::to_string(k) + "]";
				w_customer[i][t][k] = IloNumVar(env, 0.0, params.DeliveryUB_perCustomer[i][t][scenario], IloNumVar::Float, varName.c_str());
				model.add(w_customer[i][t][k]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	/* Define objective function */
	// -------------------------------------------------------------------------------------------------------------------------------
	IloExpr obj(env);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[scenario][t][warehouse][i] == 1){
				obj += params.unitHoldingCost_Customer[i] * I_customer[i][t];
				obj += params.unmetDemandPenalty[i] * b_customer[i][t];
			}
		}
		obj += params.unitHoldingCost_Warehouse[warehouse] * I_warehouse[t];
	}
	model.add(IloMinimize(env, obj));
	// -------------------------------------------------------------------------------------------------------------------------------
	/* Define Constraints */
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Balance Constraints (Warehouse):
			I[warehouse][t]^([s]) = I[warehouse][t-1]^([s]) + q[warehouse][t] - sum(i in N_w) sum(k in K) w[i][t][k][s]	for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "WarehouseInventoryBalance(" + std::to_string(warehouse + 1) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

		IloExpr expr(env);
		if (t == 0)
		{
			expr += I_warehouse[t];
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				for (int i = 0; i < params.numCustomers; ++i)
				{
					if (CATW[scenario][t][warehouse][i] == 1){
						expr += w_customer[i][t][k];
					}
				}
			}
			IloConstraint WarehouseInventoryBalanceConstraint(expr == params.initialInventory_Warehouse[warehouse] + sol_FE.deliveryQuantityToWarehouse[warehouse][t]);
			expr.end();

			model.add(WarehouseInventoryBalanceConstraint).setName(constraintName.c_str());
		}
		else
		{
			expr += I_warehouse[t];
			expr += -I_warehouse[t - 1];
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				for (int i = 0; i < params.numCustomers; ++i)
				{
					if (CATW[scenario][t][warehouse][i] == 1){
						expr += w_customer[i][t][k];
					}
				}
			}
			IloConstraint WarehouseInventoryBalanceConstraint(expr == sol_FE.deliveryQuantityToWarehouse[warehouse][t]);
			expr.end();

			model.add(WarehouseInventoryBalanceConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Balance Constraints (Customers):
			I[i][t]^([s]) = I[i][t-1]^([s]) + sum(k in K) w[i][t][k]^([s]) - d[i][t]^([s]) + b[i][t]^([s]) 	for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[scenario][t][warehouse][i] == 1){
				cout << "Customer: " << i + params.numWarehouses << endl;
				
				string constraintName = "CustomerInventoryBalance(" + std::to_string(i + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

				IloExpr expr(env);

				if (t == 0)
				{
					expr += I_customer[i][t];
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						expr -= w_customer[i][t][k];
					}
					expr -= b_customer[i][t];
					IloConstraint CustomerInventoryBalanceConstraint(expr == params.initialInventory_Customer[i] - params.demand[i][t][scenario]);
					expr.end();

					model.add(CustomerInventoryBalanceConstraint).setName(constraintName.c_str());
				}
				else
				{
					expr += I_customer[i][t];
					expr += -I_customer[i][t - 1];
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						expr -= w_customer[i][t][k];
					}
					expr -= b_customer[i][t];
					IloConstraint CustomerInventoryBalanceConstraint(expr == -params.demand[i][t][scenario]);
					expr.end();

					model.add(CustomerInventoryBalanceConstraint).setName(constraintName.c_str());
				}
			}
		}
	}
	cout << "Done." << endl;
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Capacity Constraints (Customers):
			I[i][t]^([s]) + d[i][t]^([s]) <= params.storageCapacity_Customer[i] 		for all i in N_w, t in T
	// */
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[scenario][t][warehouse][i] == 1){
				string constraintName = "CustomerInventoryCapacity(" + std::to_string(i + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

				IloExpr expr(env);
				expr += I_customer[i][t];
				IloConstraint CustomerInventoryCapacityConstraint(expr <= params.storageCapacity_Customer[i] - params.demand[i][t][scenario]);
				expr.end();

				model.add(CustomerInventoryCapacityConstraint).setName(constraintName.c_str());
			}
		}
	}
	cout << "Done." << endl;
	// ---------------------------------------------------------------------------------------------------------------------------
	// /*
	// 	Define Inventory Capacity Constraints (Warehouses):
	// 		I_warehouse[w][t][s] + demand_warehouse[w][t][s] <= storageCapacity_warehouse 		for all w in W, t in T, s in S
	// */
	// for (int t = 0; t < params.numPeriods; ++t)
	// {
	// 	string constraintName = "WarehouseInventoryCapacity(" + std::to_string(warehouse + 1) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

	// 	IloExpr expr(env);
	// 	expr += I_warehouse[t];

	// 	// IloConstraint warehouseInventoryCapacityConstraint(expr <= params.storageCapacity_Warehouse[w] - demand_warehouse[w][t][s]);
	// 	IloConstraint warehouseInventoryCapacityConstraint(expr <= params.storageCapacity_Warehouse[w]);
	// 	expr.end();

	// 	model.add(warehouseInventoryCapacityConstraint).setName(constraintName.c_str());
	// }
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define delivery constraints to limit the deliveries to the available inventory of the warehouse:
			sum(i in N_w) sum(k in K) w[i][t][k]^([s]) <= warehouseInventory[warehouse][t-1]^([s]) + warehouseDelivery[warehouse][t]			for all t in T
	*/
	// for (int t = 0; t < params.numPeriods; ++t)
	// {
	// 	string constraintName = "WarehouseInventoryLimitConstraint(" + std::to_string(t + 1) + ")";

	// 	IloExpr expr(env);
	// 	for (int k = 0; k < params.numVehicles_Warehouse; ++k)
	// 	{
	// 		for (int i : CATW[scenario][t][warehouse])
	// 		{
	// 			expr += w_customer[i][t][k];
	// 		}
	// 	}

	// 	if (t == 0)
	// 	{
	// 		IloConstraint warehouseInvLimitConstraint(expr <= params.initialInventory_Warehouse[warehouse] + sol_FE.deliveryQuantityToWarehouse[warehouse][t]);
	// 		expr.end();

	// 		model.add(warehouseInvLimitConstraint).setName(constraintName.c_str());
	// 	}
	// 	else
	// 	{
	// 		IloConstraint warehouseInvLimitConstraint(expr <= sol_FE.warehouseInventory[warehouse][t - 1][scenario] + sol_FE.deliveryQuantityToWarehouse[warehouse][t]);

	// 		expr.end();

	// 		model.add(warehouseInvLimitConstraint).setName(constraintName.c_str());
	// 	}
	// }
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define delivery constraints to limit the deliveries to the available vehicle capacity of the warehouse:
			sum(i in N_w) w[i][t][k]^([s]) <= Q_warehouse[k]^([s])			for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			string constraintName = "vehicleCapacityConstraint(" + std::to_string(t + 1) + ")" + std::to_string(k + 1);

			IloExpr expr(env);

			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (CATW[scenario][t][warehouse][i] == 1){
					expr += w_customer[i][t][k];
				}
			}

			IloConstraint vehicleCapacityConstraint(expr <= params.vehicleCapacity_Warehouse);

			expr.end();

			model.add(vehicleCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Delivery limit constraints:
			w[i][t][k]^([s]) <= M[i][t]^([s]) * sum(j in N) a[s][w][t][i]			for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (CATW[scenario][t][warehouse][i] == 1){
					string constraintName = "deliveryLimitConstraint(" + std::to_string(i + params.numWarehouses) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

					auto it = std::find(routes_WareToCust_ScenWare[t][k].begin() + 1, routes_WareToCust_ScenWare[t][k].end() - 1, i + params.numWarehouses);
					if (it != routes_WareToCust_ScenWare[t][k].end() - 1)
					{
						IloExpr expr(env);
						expr += w_customer[i][t][k];
						expr -= params.DeliveryUB_perCustomer[i][t][scenario];
						IloConstraint deliveryLimitConstraint(expr <= 0);
						expr.end();

						model.add(deliveryLimitConstraint).setName(constraintName.c_str());
					}
					else
					{
						IloExpr expr(env);
						expr += w_customer[i][t][k];
						IloConstraint deliveryLimitConstraint(expr <= 0);
						expr.end();

						model.add(deliveryLimitConstraint).setName(constraintName.c_str());
					}
				}
			}
		}
	}
	cout << "Done." << endl;
	// ---------------------------------------------------------------------------------------------------------------------------
	/* Assure linear mappings between the presolved and original models */
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

	if (save_lpFile)
	{
		string directory = "../cplexFiles/lpModel/";
		string lpFileName = directory + "ILS_LP" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_scenario" + std::to_string(scenario) + "_warehouse" + std::to_string(warehouse) + "_Ins" + params.instance.c_str() + ".lp";

		// Export the model to an LP file
		cplex.exportModel(lpFileName.c_str());
	}

	// Extract model
	cplex.extract(model);

	// Solve the model
	cplex.solve();

	string status;
	double objValue = 0.0;

	if (cplex.getStatus() == IloAlgorithm::Optimal)
	{
		status = "Optimal";
		objValue = cplex.getObjValue();
		cout << "Optimal solution found with objective value: " << std::fixed << std::setprecision(1) << objValue << endl;

		if (save_mpsResultFile)
		{
			string directory = "../cplexFiles/solVal/";
			string solFileName = directory + "ILS_LP" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_scenario" + std::to_string(scenario) + "_warehouse" + std::to_string(warehouse) + "_Ins" + params.instance.c_str();

			// Export the model to an LP file
			cplex.writeSolution(solFileName.c_str());
		}

		for (int t = 0; t < params.numPeriods; ++t)
		{
			ware_Inv[t] = cplex.getValue(I_warehouse[t]);
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (CATW[scenario][t][warehouse][i] == 1){
					cust_Inv[i][t] = cplex.getValue(I_customer[i][t]);
					cust_unmDem[i][t] = cplex.getValue(b_customer[i][t]);

					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						cust_delQuant[i][t] += cplex.getValue(w_customer[i][t][k]);
					}
				}
			}
		}

		// Print the solution
		for (int t = 0; t < params.numPeriods; ++t)
		{
			cout << "deliveryQuantityToWarehouse[" << t << "] = " << sol_FE.deliveryQuantityToWarehouse[warehouse][t] << endl;
		}

		for (int t = 0; t < params.numPeriods; ++t)
		{
			cout << "ware_Inv[" << t << "] = " << ware_Inv[t] << endl;
		}

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (CATW[scenario][t][warehouse][i] == 1){
					if (cust_delQuant[i][t] > 0.0)
					{
						cout << "cust_delQuant[" << i + params.numWarehouses << "][" << t << "] = " << cust_delQuant[i][t] << endl;
					}
				}
			}
		}

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (CATW[scenario][t][warehouse][i] == 1){
					if (cust_unmDem[i][t] > 0.0)
					{
						cout << "cust_unmDem[" << i + params.numWarehouses << "][" << t << "] = " << cust_unmDem[i][t] << endl;
					}
				}
			}
		}
	}
	else if (cplex.getStatus() == IloAlgorithm::Infeasible)
	{
		status = "Infeasible";
		cout << "Problem is infeasible" << endl;

		string directory = "../cplexFiles/lpModel/";
		string lpFileName = directory + "ILS_LP" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_scenario" + std::to_string(scenario) + "_warehouse" + std::to_string(warehouse) + "_Ins" + params.instance.c_str() + ".lp";

		// Export the model to an LP file
		cplex.exportModel(lpFileName.c_str());
	}
	else
	{
		status = "Undefined";
		cout << "Solver terminated with status: " << status << endl;
	}

	env.end();

	return status;
}

vector<double> LP_ScenarioWarehouse::getInvWarehouse()
{
	return ware_Inv;
}

vector<vector<double>> LP_ScenarioWarehouse::getInvCustomers()
{
	return cust_Inv;
}

vector<vector<double>> LP_ScenarioWarehouse::getUnmetDemandCustomers()
{
	return cust_unmDem;
}

vector<vector<double>> LP_ScenarioWarehouse::getDeliveryQuantityCustomers()
{
	return cust_delQuant;
}
