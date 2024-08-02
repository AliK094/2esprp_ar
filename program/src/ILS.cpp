#include "ILS.h"

ILS_SIRP::ILS_SIRP(const ParameterSetting &parameters, const Solution &solution)
	: params(parameters), sol_FE(solution)
{
	// Initialize random seed
	srand(static_cast<unsigned int>(time(NULL)));
	// ----------------------------------------------------------------------------------------------------------
	cout << "\nILS_SIRP" << endl;
	RATW = params.getRetailersAssignedToWarehouse();
}

bool ILS_SIRP::solve()
{
	/*
		Construct Initial Solution
	*/
	cout << "Construct Initial Solution" << endl;
	ConstructHeuristic consHeuristic(params, sol_FE);
	bool status = consHeuristic.Construct_InitialSolution();
	if (!status)
	{
		cerr << "Failed to construct the initial solution" << endl;
		return false;
	}

	Inv_Retailers = consHeuristic.getInvRetailers();
	unmetDemand_Retailers = consHeuristic.getUnmetDemandRetailers();
	deliveryQuantity_Retailers = consHeuristic.getDeliveryQuantityRetailers();
	routes_WarehouseToRetailer = consHeuristic.getRoutesWarehouseToRetailer();
	objValue = consHeuristic.getBestObjValue();

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					cout << "route[" << s << "][" << w << "][" << t << "][" << k << "] : [";
					for (auto it = routes_WarehouseToRetailer[s][w][t][k].begin(); it != routes_WarehouseToRetailer[s][w][t][k].end(); ++it)
					{
						if (it != routes_WarehouseToRetailer[s][w][t][k].begin())
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

	double objValue_best = calculateObjFuncValue_SecondEchelon(Inv_Retailers, unmetDemand_Retailers, routes_WarehouseToRetailer);
	cout << "\nBest Objective Function Value (Second-Echelon): " << std::setprecision(1) << std::fixed << objValue_best << endl;

	if (!checkSolutionFeasiblity())
	{
		cerr << "Initial solution is not feasible" << endl;
		return false;
	}

	double objValue_ILS = 0.0;
	// vector<vector<double>> ObjValScenarioWarehouse(params.numScenarios, vector<double>(params.numWarehouses, 0.0));
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			// Local Search
			// Create scenario warehouse local solution
			// vector<vector<double>> Inv_Retailers_ScenarioWarehouse(RATW[w].size(), vector<double>(params.numPeriods, 0.0));
			// vector<vector<double>> unmetDemand_Retailers_ScenarioWarehouse(RATW[w].size(), vector<double>(params.numPeriods, 0.0));
			// vector<vector<double>> deliveryQuantity_Retailers_ScenarioWarehouse(RATW[w].size(), vector<double>(params.numPeriods, 0.0));

			// for (int t = 0; t < params.numPeriods; ++t)
			// {
			// 	for (size_t i = 0; i < RATW[w].size(); ++i)
			// 	{
			// 		Inv_Retailers_ScenarioWarehouse[i][t] = Inv_Retailers[RATW[w][i]][t][s];
			// 		unmetDemand_Retailers_ScenarioWarehouse[i][t] = unmetDemand_Retailers[RATW[w][i]][t][s];
			// 		deliveryQuantity_Retailers_ScenarioWarehouse[i][t] = deliveryQuantity_Retailers[RATW[w][i]][t][s];

			// 		// cout << "Inv_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << Inv_Ret_temp[i][t] << endl;
			// 		// cout << "unmetDemand_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << unmetDemand_Ret_temp[i][t] << endl;
			// 		// cout << "delQuant_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << delQuant_Ret_temp[i][t] << endl;
			// 	}
			// }
			// vector<vector<vector<int>>> routes_WareToRet_ScenarioWarehouse = routes_WarehouseToRetailer[s][w];

			// double objVal_ScenarioWarehouse_temp = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Retailers_ScenarioWarehouse, unmetDemand_Retailers_ScenarioWarehouse, routes_WareToRet_ScenarioWarehouse);
			// cout << "Best Objective Function Value (Scenario Warehouse): " << std::setprecision(1) << std::fixed << objVal_ScenarioWarehouse_temp << endl;

			// double objVal = 0.0;
			// LocalSearch ls(params, sol_FE);
			// ls.RVND(s, w,
			// 		Inv_Retailers_ScenarioWarehouse,
			// 		unmetDemand_Retailers_ScenarioWarehouse,
			// 		deliveryQuantity_Retailers_ScenarioWarehouse,
			// 		routes_WareToRet_ScenarioWarehouse,
			// 		objVal);

			// for (int t = 0; t < params.numPeriods; ++t)
			// {
			// 	for (size_t i = 0; i < RATW[w].size(); ++i)
			// 	{
			// 		Inv_Retailers[RATW[w][i]][t][s] = Inv_Retailers_ScenarioWarehouse[i][t];
			// 		unmetDemand_Retailers[RATW[w][i]][t][s] = unmetDemand_Retailers_ScenarioWarehouse[i][t];
			// 		deliveryQuantity_Retailers[RATW[w][i]][t][s] = deliveryQuantity_Retailers_ScenarioWarehouse[i][t];

			// 		// cout << "Inv_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << Inv_Retailers[RATW[w][i]][t][s] << endl;
			// 		// cout << "unmetDemand_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << unmetDemand_Retailers[RATW[w][i]][t][s] << endl;
			// 		// cout << "delQuant_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << deliveryQuantity_Retailers[RATW[w][i]][t][s] << endl;
			// 	}
			// }
			// routes_WarehouseToRetailer[s][w] = routes_WareToRet_ScenarioWarehouse;

			// ObjValScenarioWarehouse[s][w] = objVal;

			// ---------------------------------------------------------------------------------------------
			const int maxIterILS = 50;
			int numIterILS = 0;
			bool stop = false;
			double best_ObjValue_ScenarioWarehouse = std::numeric_limits<double>::max();
			while (!stop && numIterILS < maxIterILS)
			{
				cout << "\nIteration (ILS) for Scenario " << s << ", Warehouse " << w << ": " << numIterILS + 1 << endl;
				vector<vector<double>> Inv_Retailers_ScenarioWarehouse(RATW[w].size(), vector<double>(params.numPeriods, 0.0));
				vector<vector<double>> unmetDemand_Retailers_ScenarioWarehouse(RATW[w].size(), vector<double>(params.numPeriods, 0.0));
				vector<vector<double>> deliveryQuantity_Retailers_ScenarioWarehouse(RATW[w].size(), vector<double>(params.numPeriods, 0.0));

				for (int t = 0; t < params.numPeriods; ++t)
				{
					for (size_t i = 0; i < RATW[w].size(); ++i)
					{
						Inv_Retailers_ScenarioWarehouse[i][t] = Inv_Retailers[RATW[w][i]][t][s];
						unmetDemand_Retailers_ScenarioWarehouse[i][t] = unmetDemand_Retailers[RATW[w][i]][t][s];
						deliveryQuantity_Retailers_ScenarioWarehouse[i][t] = deliveryQuantity_Retailers[RATW[w][i]][t][s];

						// cout << "Inv_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << Inv_Ret_temp[i][t] << endl;
						// cout << "unmetDemand_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << unmetDemand_Ret_temp[i][t] << endl;
						// cout << "delQuant_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << delQuant_Ret_temp[i][t] << endl;
					}
				}
				vector<vector<vector<int>>> routes_WareToRet_ScenarioWarehouse = routes_WarehouseToRetailer[s][w];
				double objVal_ScenarioWarehouse_temp = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Retailers_ScenarioWarehouse, unmetDemand_Retailers_ScenarioWarehouse, routes_WareToRet_ScenarioWarehouse);
				cout << "Best Objective Function Value (Scenario Warehouse): " << std::setprecision(1) << std::fixed << objVal_ScenarioWarehouse_temp << endl;

				if (numIterILS > 0)
				{
					/*
						Perturbation
					*/
					Perturbation perturb(params, sol_FE);
					bool perturbSuccess = perturb.run(s, w,
													  Inv_Retailers_ScenarioWarehouse,
													  unmetDemand_Retailers_ScenarioWarehouse,
													  deliveryQuantity_Retailers_ScenarioWarehouse,
													  routes_WareToRet_ScenarioWarehouse);
					if (!perturbSuccess)
					{
						stop = true;
					}
				}

				for (int t = 0; t < params.numPeriods; ++t)
				{
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						cout << "route[" << t << "][" << k << "] : [";
						for (auto it = routes_WareToRet_ScenarioWarehouse[t][k].begin(); it != routes_WareToRet_ScenarioWarehouse[t][k].end(); ++it)
						{
							if (it != routes_WareToRet_ScenarioWarehouse[t][k].begin())
							{
								cout << " -> ";
							}
							cout << *it;
						}
						cout << "]" << endl;
					}
				}

				/*
					Local Search
				*/
				double objValue_ScenarioWarehouse = 0.0;
				double objval = 0.0;
				LocalSearch ls(params, sol_FE);
				ls.RVND(s, w,
						Inv_Retailers_ScenarioWarehouse,
						unmetDemand_Retailers_ScenarioWarehouse,
						deliveryQuantity_Retailers_ScenarioWarehouse,
						routes_WareToRet_ScenarioWarehouse,
						objval);

				objValue_ScenarioWarehouse = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Retailers_ScenarioWarehouse, unmetDemand_Retailers_ScenarioWarehouse, routes_WareToRet_ScenarioWarehouse);
				// cout << "Best Objective Function Value (Scenario Warehouse): " << std::setprecision(1) << std::fixed << best_ObjValue_ScenarioWarehouse << endl;

				if (objValue_ScenarioWarehouse < best_ObjValue_ScenarioWarehouse)
				{
					best_ObjValue_ScenarioWarehouse = objValue_ScenarioWarehouse;
					for (int t = 0; t < params.numPeriods; ++t)
					{
						for (size_t i = 0; i < RATW[w].size(); ++i)
						{
							Inv_Retailers[RATW[w][i]][t][s] = Inv_Retailers_ScenarioWarehouse[i][t];
							unmetDemand_Retailers[RATW[w][i]][t][s] = unmetDemand_Retailers_ScenarioWarehouse[i][t];
							deliveryQuantity_Retailers[RATW[w][i]][t][s] = deliveryQuantity_Retailers_ScenarioWarehouse[i][t];
						}
					}
					routes_WarehouseToRetailer[s][w] = routes_WareToRet_ScenarioWarehouse;
					cout << "Best Objective Function Value (Scenario Warehouse): " << std::setprecision(1) << std::fixed << best_ObjValue_ScenarioWarehouse << endl;
				}

				// objValue_ILS += params.probability[s] * objVal;
				numIterILS++;
			}
		}
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
					for (auto it = routes_WarehouseToRetailer[s][w][t][k].begin(); it != routes_WarehouseToRetailer[s][w][t][k].end(); ++it)
					{
						if (it != routes_WarehouseToRetailer[s][w][t][k].begin())
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
	// cout << "ObjValue_ILS: " << objValue_ILS << endl;

	// int maxIterILS = 50;
	// bool stop = false;

	// 	double objValue_new = 0.0;

	// 			if (objVal < ObjValScenarioWarehouse[s][w]){
	// 				ObjValScenarioWarehouse[s][w] = objVal;
	// 				for (int t = 0; t < params.numPeriods; ++t)
	// 				{
	// 					for (size_t i = 0; i < RATW[w].size(); ++i)
	// 					{
	// 						Inv_Retailers[RATW[w][i]][t][s] = Inv_Retailers_ScenarioWarehouse[i][t];
	// 						unmetDemand_Retailers[RATW[w][i]][t][s] = unmetDemand_Retailers_ScenarioWarehouse[i][t];
	// 						deliveryQuantity_Retailers[RATW[w][i]][t][s] = deliveryQuantity_Retailers_ScenarioWarehouse[i][t];
	// 					}
	// 				}
	// 				routes_WarehouseToRetailer[s][w] = routes_WareToRet_ScenarioWarehouse;
	// 			}

	// 			objValue_new += params.probability[s] * ObjValScenarioWarehouse[s][w];
	// 		}
	// 	}

	// 	cout << "objValue_new: " << objValue_new << endl;
	// 	cout << "objValue_ILS: " << objValue_ILS << endl;
	// 	if (objValue_new < objValue_ILS)
	// 	{
	// 		objValue_ILS = objValue_new;
	// 	}

	// }
	// objValue_ILS = objValue_new;
	// cout << "objValue_ILS: " << objValue_ILS << endl;

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
	// 		for (int i = 0; i < params.numRetailers; ++i)
	// 		{
	// 			if (Inv_Retailers[i][t][s] != 0)
	// 			{
	// 				cout << "Inv_Retailers[" << i << "][" << t << "][" << s << "] = " << Inv_Retailers[i][t][s] << endl;
	// 			}

	// 			if (unmetDemand_Retailers[i][t][s] != 0)
	// 			{
	// 				cout << "unmetDemand_Retailers[" << i << "][" << t << "][" << s << "] = " << unmetDemand_Retailers[i][t][s] << endl;
	// 			}
	// 		}
	// 	}
	// }

	double objValue_new = calculateObjFuncValue_SecondEchelon(Inv_Retailers, unmetDemand_Retailers, routes_WarehouseToRetailer);
	cout << "\nObjValue_new: " << objValue_new << endl;

	return true;
}

double ILS_SIRP::calculateObjFuncValue_ScenarioWarehouse(int s, int w, const vector<vector<double>> &Inv_Ret_Temp, const vector<vector<double>> &unmetDem_Ret_Temp, const vector<vector<vector<int>>> &route_WTR_Temp)
{
	// cout << "\ncalculating objFuncValue_ScenarioWarehouse" << endl;
	// for (int t = 0; t < params.numPeriods; ++t)
	// {
	// 	for (int i = 0; i < RATW[w].size(); ++i)
	// 	{
	// 		if (Inv_Ret_Temp[i][t] > 0.0)
	// 		{
	// 			cout << "ret_Inv[" << i << "][" << t << "] = " << Inv_Ret_Temp[i][t] << endl;
	// 		}

	// 		if (unmetDem_Ret_Temp[i][t] > 0.0)
	// 		{
	// 			cout << "ret_unmDem[" << i << "][" << t << "] = " << unmetDem_Ret_Temp[i][t] << endl;
	// 		}
	// 	}
	// }

	double objFuncValue = 0.0;

	double inventoryCost = 0.0;
	double unmetDemandCost = 0.0;
	double routeCost_Warehouse = 0.0;

	// Inventory Holding Cost and Unmet Demand Cost
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < RATW[w].size(); ++i)
		{
			objFuncValue += params.unitHoldingCost_Retailer[RATW[w][i]] * Inv_Ret_Temp[i][t];
			objFuncValue += params.unmetDemandPenalty[RATW[w][i]] * unmetDem_Ret_Temp[i][t];

			inventoryCost += params.unitHoldingCost_Retailer[RATW[w][i]] * Inv_Ret_Temp[i][t];
			// cout << params.unitHoldingCost_Retailer[RATW[w][i]] << " * " << Inv_Ret_Temp[i][t] << " = " << params.unitHoldingCost_Retailer[RATW[w][i]] * Inv_Ret_Temp[i][t] << endl;
			unmetDemandCost += params.unmetDemandPenalty[RATW[w][i]] * unmetDem_Ret_Temp[i][t];
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			double routeCost = 0.0;

			int previousNode = w;
			for (int j = 1; j < route_WTR_Temp[t][k].size(); ++j)
			{
				int currentNode = route_WTR_Temp[t][k][j];
				objFuncValue += params.transportationCost_SecondEchelon[previousNode][currentNode];

				routeCost += params.transportationCost_SecondEchelon[previousNode][currentNode];

				routeCost_Warehouse += params.transportationCost_SecondEchelon[previousNode][currentNode];
				previousNode = currentNode;
			}

			// cout << "Route[" << w << "][" << t << "][" << k << "]: [";
			// for (int i = 0; i < route_WTR_Temp[t][k].size(); ++i)
			// {
			// 	if (i != route_WTR_Temp[t][k].size() - 1)
			// 	{
			// 		cout << route_WTR_Temp[t][k][i] << " -> ";
			// 	}
			// 	else
			// 	{
			// 		cout << route_WTR_Temp[t][k][i];
			// 	}
			// }
			// cout << "]" << endl;

			// cout << "Route Cost(RVND)[" << s << "][" << w << "][" << t << "][" << k << "]: " << routeCost << endl;
		}
	}

	// cout << "Inventory Cost (RVND)[" << s << "][" << w << "]: " << inventoryCost << endl;
	// cout << "Unmet Demand Cost (RVND)[" << s << "][" << w << "]: " << unmetDemandCost << endl;
	// cout << "Route Cost (RVND)[" << s << "][" << w << "]: " << routeCost_Warehouse << endl;

	// cout << "objFuncValue:" << objFuncValue << endl;
	return objFuncValue;
}

bool ILS_SIRP::checkSolutionFeasiblity()
{
	// ----------------------------------------------------------------------------------------------------------
	// Retailers Inventory Capacity of Retailers
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numRetailers; ++i)
			{
				// if (Inv_Retailers[i][t][s] > 0.0){
				// 	cout << "\nI[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << Inv_Retailers[i][t][s] << endl;
				// }

				// if (unmetDemand_Retailers[i][t][s] > 0.0){
				// 	cout << "b[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << unmetDemand_Retailers[i][t][s] << endl;
				// }

				// if (deliveryQuantity_Retailers[i][t][s] > 0.0){
				// 	cout << "q[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << deliveryQuantity_Retailers[i][t][s] << endl;
				// }

				// cout << "demand[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << params.demand[i][t][s] << endl;
				// cout << "init_Inv[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << params.initialInventory_Retailer[i] << endl;

				if (t == 0)
				{
					if (Inv_Retailers[i][t][s] != params.initialInventory_Retailer[i] + unmetDemand_Retailers[i][t][s] + deliveryQuantity_Retailers[i][t][s] - params.demand[i][t][s])
					{
						cout << "I[" << i << "][" << t << "][" << s << "] != ";
						cout << "init_Inv[" << i << "][" << t << "][" << s << "] + ";
						cout << "w[" << i << "][" << t << "][" << s << "] - ";
						cout << "demand[" << i << "][" << t << "][" << s << "]";
						cout << "b[" << i << "][" << t << "][" << s << "]" << endl;

						cout << Inv_Retailers[i][t][s] << " != " << params.initialInventory_Retailer[i] << " + " << deliveryQuantity_Retailers[i][t][s] << " - " << params.demand[i][t][s] << " + " << unmetDemand_Retailers[i][t][s] << endl;

						return false;
					}
				}
				else
				{
					if (Inv_Retailers[i][t][s] != Inv_Retailers[i][t - 1][s] + unmetDemand_Retailers[i][t][s] + deliveryQuantity_Retailers[i][t][s] - params.demand[i][t][s])
					{
						cout << "I[" << i << "][" << t << "][" << s << "] != ";
						cout << "I" << i << "][" << t - 1 << "][" << s << "] + ";
						cout << "w[" << i << "][" << t << "][" << s << "] - ";
						cout << "demand[" << i << "][" << t << "][" << s << "]";
						cout << "b[" << i << "][" << t << "][" << s << "]" << endl;

						cout << Inv_Retailers[i][t][s] << " != " << Inv_Retailers[i][t - 1][s] << " + " << deliveryQuantity_Retailers[i][t][s] << " - " << params.demand[i][t][s] << " + " << unmetDemand_Retailers[i][t][s] << endl;

						return false;
					}
				}

				if (Inv_Retailers[i][t][s] + params.demand[i][t][s] > params.storageCapacity_Retailer[i])
				{
					cout << "Inv_Retailers[i][t][s] + demand[i][t][s] > storageCapacity_Retailer" << endl;
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
				// 	for (auto it = routes_WarehouseToRetailer[s][w][t][k].begin(); it != routes_WarehouseToRetailer[s][w][t][k].end(); ++it)
				// 	{
				// 		if (it != routes_WarehouseToRetailer[s][w][t][k].begin())
				// 		{
				// 			cout << " -> ";
				// 		}
				// 		cout << *it;
				// 	}
				// 	cout << "]" << endl;
				// }

				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					if (!routes_WarehouseToRetailer[s][w][t][k].empty())
					{
						double sum = 0.0;
						for (auto it = routes_WarehouseToRetailer[s][w][t][k].begin() + 1; it != routes_WarehouseToRetailer[s][w][t][k].end() - 1; ++it)
						{
							sum += deliveryQuantity_Retailers[*it - params.numWarehouses][t][s];
						}

						if (sum > params.vehicleCapacity_Warehouse)
						{
							for (auto it = routes_WarehouseToRetailer[s][w][t][k].begin() + 1; it != routes_WarehouseToRetailer[s][w][t][k].end() - 1; ++it)
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

double ILS_SIRP::calculateObjFuncValue_SecondEchelon(const vector<vector<vector<double>>> &Inv_Retailers, const vector<vector<vector<double>>> &unmetDemand_Retailers, const vector<vector<vector<vector<vector<int>>>>> &routes_WarehouseToRetailers)
{
	double objFuncValue = 0.0;
	for (int s = 0; s < params.numScenarios; ++s)
	{
		// for (int t = 0; t < params.numPeriods; ++t)
		// {
		// 	for (int i = 0; i < params.numRetailers; ++i)
		// 	{
		// 		if (Inv_Retailers[i][t][s] != 0)
		// 		{
		// 			cout << "Inv_Retailers[" << i << "][" << t << "][" << s << "] = " << Inv_Retailers[i][t][s] << endl;
		// 		}

		// 		if (unmetDemand_Retailers[i][t][s] != 0)
		// 		{
		// 			cout << "unmetDemand_Retailers[" << i << "][" << t << "][" << s << "] = " << unmetDemand_Retailers[i][t][s] << endl;
		// 		}
		// 	}
		// }

		double objValue_ScenarioWarehouse = 0.0;
		double Inv_cost_ScenarioWarehouse = 0.0;
		double unmetDemand_ScenarioWarehouse = 0.0;
		double routeCost_ScenarioWarehouse = 0.0;

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numRetailers; ++i)
			{
				objFuncValue += params.probability[s] * params.unitHoldingCost_Retailer[i] * Inv_Retailers[i][t][s];
				Inv_cost_ScenarioWarehouse += params.unitHoldingCost_Retailer[i] * Inv_Retailers[i][t][s];
				// cout << params.unitHoldingCost_Retailer[i] << " * " << Inv_Retailers[i][t][s] << " = " << params.unitHoldingCost_Retailer[i] * Inv_Retailers[i][t][s] << endl;

				objFuncValue += params.probability[s] * params.unmetDemandPenalty[i] * unmetDemand_Retailers[i][t][s];
				unmetDemand_ScenarioWarehouse += params.unmetDemandPenalty[i] * unmetDemand_Retailers[i][t][s];

				objValue_ScenarioWarehouse += params.unitHoldingCost_Retailer[i] * Inv_Retailers[i][t][s];
				objValue_ScenarioWarehouse += params.unmetDemandPenalty[i] * unmetDemand_Retailers[i][t][s];
			}
		}

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					double routeCost = 0.0;

					int previousNode = w;
					for (int j = 1; j < routes_WarehouseToRetailers[s][w][t][k].size(); ++j)
					{
						int currentNode = routes_WarehouseToRetailers[s][w][t][k][j];
						objFuncValue += params.probability[s] * params.transportationCost_SecondEchelon[previousNode][currentNode];

						routeCost += params.transportationCost_SecondEchelon[previousNode][currentNode];

						objValue_ScenarioWarehouse += params.transportationCost_SecondEchelon[previousNode][currentNode];
						routeCost_ScenarioWarehouse += params.transportationCost_SecondEchelon[previousNode][currentNode];
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
	return objFuncValue;
}

ConstructHeuristic::ConstructHeuristic(const ParameterSetting &parameters, const Solution &solution)
	: params(parameters),
	  sol_FE(solution)
{
	// ----------------------------------------------------------------------------------------------------------
	RATW = params.getRetailersAssignedToWarehouse();
	assignedWarehouseToRetailer = params.getWarehouseAssignedToRetailer();

	// we calculate the Retailers per unit unemt demand penalty cost to approximate per unit demand satisfaction cost ratio
	// We order Retailers in descending order to have them from highest to lowest penalty cost ratio. (this shows the prioritun in demand satisfaction)
	sorted_Retailers_byPenaltyCostRatio.resize(params.numWarehouses, vector<int>());
	orderRetailersByUnmetDemandToDeliveryRatio(sorted_Retailers_byPenaltyCostRatio);

	bestObjValue = 0.0;
	Inv_Retailers_bestSolution.resize(params.numRetailers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	unmetDemand_Retailers_bestSolution.resize(params.numRetailers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	deliveryQuantity_Retailers_bestSolution.resize(params.numRetailers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	routes_bestSolution.resize(params.numScenarios, vector<vector<vector<vector<int>>>>(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>()))));

	// for (int s = 0; s < params.numScenarios; ++s)
	// {
	// 	for (int t = 0; t < params.numPeriods; ++t)
	// 	{
	// 		for (int w = 0; w < params.numWarehouses; ++w)
	// 		{
	// 			cout << sol_FE.warehouseInventory[w][t][s] << " ";
	// 		}
	// 		cout << "| ";
	// 	}
	// 	cout << endl;
	// }
}

void ConstructHeuristic::orderRetailersByUnmetDemandToDeliveryRatio(vector<vector<int>> &sorted_retailer_costRatio)
{
	// calculate the stockout to demand satisfaction cost ratio using: Ratio = penaltyCost[i] / F/C + 2c[0][w] + 2c[w][i] + look_ahead * h[i]
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		vector<std::pair<int, double>> retailer_costRatio;
		for (int i : RATW[w])
		{
			double costRatio = (params.unmetDemandPenalty[i]) /
							   ((params.setupCost / params.prodCapacity) +
								((2 * params.transportationCost_FirstEchelon[0][w + 1]) / params.vehicleCapacity_Plant) +
								((2 * params.transportationCost_SecondEchelon[w][i + params.numWarehouses]) / params.vehicleCapacity_Warehouse));

			retailer_costRatio.emplace_back(i + params.numWarehouses, costRatio);
		}

		std::sort(retailer_costRatio.begin(), retailer_costRatio.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b)
				  { return a.second > b.second; });

		for (const auto &pair : retailer_costRatio)
		{
			sorted_retailer_costRatio[w].push_back(pair.first);
		}

		// for (const auto &i : sorted_retailer_costRatio[w])
		// {
		// 	cout << "Sorted retailer CostRatio[" << w << "] = " << i << endl;
		// }
	}
}

void ConstructHeuristic::calculateDecisionVariables(int s, int w, vector<vector<double>> &Inv_Retailers, vector<vector<double>> &unmetDemand_Retailers, vector<vector<double>> &deliveryQuantity_Retailers)
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i : RATW[w])
		{
			Inv_Retailers[i][t] = 0.0;
			unmetDemand_Retailers[i][t] = 0.0;
			if (t == 0)
			{
				Inv_Retailers[i][t] = params.initialInventory_Retailer[i];
			}
			else
			{
				Inv_Retailers[i][t] = Inv_Retailers[i][t - 1];
			}
			Inv_Retailers[i][t] -= params.demand[i][t][s];
			Inv_Retailers[i][t] += deliveryQuantity_Retailers[i][t];

			if (Inv_Retailers[i][t] < 0)
			{
				unmetDemand_Retailers[i][t] = -Inv_Retailers[i][t];
				Inv_Retailers[i][t] = 0.0;
			}

			// cout << "\nInv_Retailers_bestSolScenario[" << i + params.numWarehouses << "][" << t << "]: " << Inv_Retailers[i][t] << endl;
			// cout << "deliveryQuantity_Retailers_bestSolScenario[" << i + params.numWarehouses << "][" << t << "]: " << deliveryQuantity_Retailers[i][t] << endl;
			// cout << "unmetDemand_Retailers_bestSolScenario[" << i + params.numWarehouses << "][" << t << "]: " << unmetDemand_Retailers[i][t] << endl;
			// cout << "demand[" << i + params.numWarehouses << "][" << t << "][" << s << "]: " << params.demand[i][t][s] << endl;
		}
	}
}

void ConstructHeuristic::defineSetOne(int s, int w, int t, vector<int> &setOne, vector<vector<double>> &Inv_Retailers, const vector<vector<double>> &unmetDemand_Retailers, vector<double> &tempDeliveryQuantity)
{
	for (int i : sorted_Retailers_byPenaltyCostRatio[w])
	{
		int retIndex = i - params.numWarehouses;

		if (unmetDemand_Retailers[retIndex][t] > 0)
		{
			setOne.push_back(i);

			if (t == 0)
			{
				tempDeliveryQuantity[retIndex] = std::min({unmetDemand_Retailers[retIndex][t], params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailer[retIndex] - params.initialInventory_Retailer[retIndex])});
			}
			else if (t == params.numPeriods - 1)
			{
				tempDeliveryQuantity[retIndex] = std::min(params.vehicleCapacity_Warehouse, unmetDemand_Retailers[retIndex][t]);
			}
			else
			{
				tempDeliveryQuantity[retIndex] = std::min({unmetDemand_Retailers[retIndex][t], params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailer[retIndex] - Inv_Retailers[retIndex][t - 1])});
			}
		}
	}
}

void ConstructHeuristic::defineSetTwo(int s, int w, int t, int look_ahead, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &Inv_Retailers, const vector<vector<double>> &unmetDemand_Retailers, vector<double> &tempDeliveryQuantity)
{
	for (int i : sorted_Retailers_byPenaltyCostRatio[w])
	{
		int retIndex = i - params.numWarehouses;

		double unetDemandRetailers_lookAhead = 0.0;
		for (int l = t + 1; l <= std::min(t + look_ahead, params.numPeriods - 1); ++l)
		{
			// cout << "look ahead demand [" << i << "] = " << unmetDemand_Retailers[retIndex][l] << endl;
			unetDemandRetailers_lookAhead += unmetDemand_Retailers[retIndex][l];
		}

		if (unetDemandRetailers_lookAhead > 0)
		{
			// check if retailer i is in setOne
			auto it = std::find(setOne.begin(), setOne.end(), i);
			if (it != setOne.end())
			{
				if (t == 0)
				{
					tempDeliveryQuantity[retIndex] = std::min({unetDemandRetailers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailer[retIndex] - params.initialInventory_Retailer[retIndex])});
				}
				else
				{
					tempDeliveryQuantity[retIndex] = std::min({unetDemandRetailers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailer[retIndex] - Inv_Retailers[retIndex][t - 1])});
				}
			}
			else if (it == setOne.end())
			{
				setTwo.push_back(i);

				if (t == 0)
				{
					tempDeliveryQuantity[retIndex] = std::min({unetDemandRetailers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailer[retIndex] - params.initialInventory_Retailer[retIndex])});
				}
				else
				{
					tempDeliveryQuantity[retIndex] = std::min({unetDemandRetailers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailer[retIndex] - Inv_Retailers[retIndex][t - 1])});
				}
			}
		}
	}
}

void ConstructHeuristic::nearestNeighourInsertion(int s, int w, int t, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &deliveryQuantity_Retailers, vector<double> &tempDeliveryQuantity, vector<vector<int>> &routesPeriod)
{
	int numRoutes = params.numVehicles_Warehouse;

	double remainingAvailInventoryWarehouse;
	if (t == 0)
	{
		remainingAvailInventoryWarehouse = params.initialInventory_Warehouse[w] + sol_FE.deliveryQuantityToWarehouse[w][t];
	}
	else
	{
		remainingAvailInventoryWarehouse = sol_FE.warehouseInventory[w][t - 1][s] + sol_FE.deliveryQuantityToWarehouse[w][t];
	}

	vector<double> remainingVehicleCapacityWarehouse(numRoutes, params.vehicleCapacity_Warehouse);

	// We now have a partial route and we wanna add Retailers (from setOne) to it
	bool AllNodesVisited = false;
	int r = 0;
	while (r < numRoutes && !AllNodesVisited)
	{
		// cout << "Vehicle " << r << endl;
		// Find closest retailer to the current node
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
				int retInd = currentNode - params.numWarehouses;

				routesPeriod[r].push_back(currentNode);
				deliveryQuantity_Retailers[retInd][t] = tempDeliveryQuantity[retInd];
				remainingAvailInventoryWarehouse -= deliveryQuantity_Retailers[retInd][t];
				remainingVehicleCapacityWarehouse[r] -= deliveryQuantity_Retailers[retInd][t];

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
			int retInd = i - params.numWarehouses;

			// cout << "\nInserting Node (check): " << i << endl;

			int routeToInsert = -1;
			int posToInsert = -1;
			double minCostToInsert = std::numeric_limits<double>::max();

			for (int r = 0; r < numRoutes; ++r)
			{
				// cout << "Route: " << r << endl;
				// cout << "Remaining Inventory (if visited): " << remainingAvailInventoryWarehouse - tempDeliveryQuantity[retInd] << ", Remaining Vehicle Capacity: " << remainingVehicleCapacityWarehouse[r] - tempDeliveryQuantity[retInd] << endl;
				if (remainingVehicleCapacityWarehouse[r] - tempDeliveryQuantity[retInd] >= 0.0 &&
					remainingAvailInventoryWarehouse - tempDeliveryQuantity[retInd] >= 0.0)
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
					deliveryQuantity_Retailers[retInd][t] = tempDeliveryQuantity[retInd];
					remainingAvailInventoryWarehouse -= tempDeliveryQuantity[retInd];
					remainingVehicleCapacityWarehouse[routeToInsert] -= tempDeliveryQuantity[retInd];
					it = setTwo.erase(it);
				}
				else
				{
					routesPeriod[routeToInsert].insert(routesPeriod[routeToInsert].begin() + posToInsert, i);
					deliveryQuantity_Retailers[retInd][t] = tempDeliveryQuantity[retInd];
					remainingAvailInventoryWarehouse -= tempDeliveryQuantity[retInd];
					remainingVehicleCapacityWarehouse[routeToInsert] -= tempDeliveryQuantity[retInd];
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

	// 	for (int k = 0; k < params.numVehicles_Warehouse; ++k)
	// 	{
	// 		cout << "routesPeriod: [";
	// 		for (auto it = routesPeriod[k].begin(); it != routesPeriod[k].end(); ++it)
	// 		{
	// 			if (it != routesPeriod[k].begin())
	// 			{
	// 				cout << " -> ";
	// 			}
	// 			cout << *it;
	// 		}
	// 		cout << "]" << endl;
	// 	}
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
		int retIndex = i - params.numWarehouses;
		double visitCost = params.transportationCost_SecondEchelon[current_node][i];

		// cout << "visit cost: " << visitCost << endl;
		// cout << "Remaining Inventory (if visited): " << remainingAvailInventoryWarehouse - tempDeliveryQuantity[retIndex] << ", Remaining Vehicle Capacity: " << remainingVehicleCapacityWarehouse - tempDeliveryQuantity[retIndex] << endl;

		if (visitCost < minVisitCost && remainingVehicleCapacityWarehouse - tempDeliveryQuantity[retIndex] >= 0.0 && remainingAvailInventoryWarehouse - tempDeliveryQuantity[retIndex] >= 0.0)
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

double ConstructHeuristic::calculateObjFuncValue(int s, int w, const vector<vector<double>> &Inv_Retailers, const vector<vector<double>> &unmetDemand_Retailers, const vector<vector<vector<int>>> &routesPeriod)
{
	double objFuncValue = 0.0;

	// Inventory Holding Cost and Unmet Demand Cost
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i : RATW[w])
		{
			objFuncValue += params.unitHoldingCost_Retailer[i] * Inv_Retailers[i][t];
			objFuncValue += params.unmetDemandPenalty[i] * unmetDemand_Retailers[i][t];
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
	try
	{
		for (int s = 0; s < params.numScenarios; ++s)
		{
			vector<vector<double>> Inv_Retailers_bestSolScenario(params.numRetailers, vector<double>(params.numPeriods, 0.0));
			vector<vector<double>> unmetDemand_Retailers_bestSolScenario(params.numRetailers, vector<double>(params.numPeriods, 0.0));
			vector<vector<double>> deliveryQuantity_Retailers_bestSolScenario(params.numRetailers, vector<double>(params.numPeriods, 0.0));
			vector<vector<vector<vector<int>>>> routes_bestSolScenario(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>())));

			for (int w = 0; w < params.numWarehouses; ++w)
			{
				double bestObjValue_ScenarioWarehouse = std::numeric_limits<double>::max();

				int look_ahead = 1;
				while (look_ahead <= std::ceil(params.numPeriods / 2.0))
				{
					vector<vector<double>> Inv_Retailers(params.numRetailers, vector<double>(params.numPeriods, 0.0));
					vector<vector<double>> unmetDemand_Retailers(params.numRetailers, vector<double>(params.numPeriods, 0.0));
					vector<vector<double>> deliveryQuantity_Retailers(params.numRetailers, vector<double>(params.numPeriods, 0.0));

					vector<vector<vector<int>>> routesPeriod(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>()));

					calculateDecisionVariables(s, w, Inv_Retailers, unmetDemand_Retailers, deliveryQuantity_Retailers);

					for (int t = 0; t < params.numPeriods; ++t)
					{
						// cout << "scenario: " << s << ", warehouse: " << w << ", look_ahead: " << look_ahead << ", period: " << t << endl;

						vector<double> tempDeliveryQuantity(params.numRetailers, 0.0);

						vector<int> setOne;
						defineSetOne(s, w, t, setOne, Inv_Retailers, unmetDemand_Retailers, tempDeliveryQuantity);

						vector<int> setTwo;
						if (t < params.numPeriods - 1)
						{
							defineSetTwo(s, w, t, look_ahead, setOne, setTwo, Inv_Retailers, unmetDemand_Retailers, tempDeliveryQuantity);
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

						nearestNeighourInsertion(s, w, t, setOne, setTwo, deliveryQuantity_Retailers, tempDeliveryQuantity, routesPeriod[t]);

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

						calculateDecisionVariables(s, w, Inv_Retailers, unmetDemand_Retailers, deliveryQuantity_Retailers);
					}

					// calculate objective cost of the second echelon for the current warehouse and scenario
					double objFuncValue_temp = calculateObjFuncValue(s, w, Inv_Retailers, unmetDemand_Retailers, routesPeriod);
					// cout << "objFuncValue_temp: " << objFuncValue_temp << endl;
					// update feasible solution
					if (objFuncValue_temp < bestObjValue_ScenarioWarehouse)
					{
						// cout << "New best feasible solution found = " << objFuncValue_temp << endl;

						bestObjValue_ScenarioWarehouse = objFuncValue_temp;
						for (int t = 0; t < params.numPeriods; ++t)
						{
							for (int i : RATW[w])
							{
								Inv_Retailers_bestSolScenario[i][t] = Inv_Retailers[i][t];
								unmetDemand_Retailers_bestSolScenario[i][t] = unmetDemand_Retailers[i][t];
								deliveryQuantity_Retailers_bestSolScenario[i][t] = deliveryQuantity_Retailers[i][t];
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

				for (int i = 0; i < params.numRetailers; ++i)
				{
					for (int t = 0; t < params.numPeriods; ++t)
					{
						Inv_Retailers_bestSolution[i][t][s] = Inv_Retailers_bestSolScenario[i][t];
						unmetDemand_Retailers_bestSolution[i][t][s] = unmetDemand_Retailers_bestSolScenario[i][t];
						deliveryQuantity_Retailers_bestSolution[i][t][s] = deliveryQuantity_Retailers_bestSolScenario[i][t];

						// cout << "Inv_Retailers_bestSolution[" << i << "][" << t << "][" << s << "] = " << Inv_Retailers_bestSolution[i][t][s] << endl;
						// cout << "unmetDemand_Retailers_bestSolution[" << i << "][" << t << "][" << s << "] = " << unmetDemand_Retailers_bestSolution[i][t][s] << endl;
						// cout << "deliveryQuantity_Retailers_bestSolution[" << i << "][" << t << "][" << s << "] = " << deliveryQuantity_Retailers_bestSolution[i][t][s] << endl;
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

		// for (int s = 0; s < params.numScenarios; ++s)
		// {
		// 	cout << "Scenario: " << s << endl;
		// 	for (int t = 0; t < params.numPeriods; ++t)
		// 	{
		// 		for (int i = 0; i < params.numRetailers; ++i)
		// 		{
		// 			// cout << "Inv_Retailers_bestSolution[" << i << "][" << t << "][" << s << "] = " << Inv_Retailers_bestSolution[i][t][s] << endl;
		// 			// cout << "unmetDemand_Retailers_bestSolution[" << i << "][" << t << "][" << s << "] = " << unmetDemand_Retailers_bestSolution[i][t][s] << endl;
		// 			// cout << "deliveryQuantity_Retailers_bestSolution[" << i << "][" << t << "][" << s << "] = " << deliveryQuantity_Retailers_bestSolution[i][t][s] << endl;
		// 		}
		// 	}

		// 	// for (int w = 0; w < params.numWarehouses; ++w)
		// 	// {
		// 	// 	for (int t = 0; t < params.numPeriods; ++t)
		// 	// 	{
		// 	// 		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		// 	// 		{
		// 	// 			cout << "routesPeriod: [" << s << "][" << w << "][" << t << "][" << k << "] = [";
		// 	// 			for (auto it = routes_bestSolution[s][w][t][k].begin(); it != routes_bestSolution[s][w][t][k].end(); ++it)
		// 	// 			{
		// 	// 				if (it != routes_bestSolution[s][w][t][k].begin())
		// 	// 				{
		// 	// 					cout << " -> ";
		// 	// 				}
		// 	// 				cout << *it;
		// 	// 			}
		// 	// 			cout << "]" << endl;
		// 	// 		}
		// 	// 	}
		// 	// }
		// }
	}
	catch (std::exception &e)
	{
		return false;
	}

	return true;
}

vector<vector<vector<double>>> ConstructHeuristic::getInvRetailers()
{
	return Inv_Retailers_bestSolution;
}

vector<vector<vector<double>>> ConstructHeuristic::getUnmetDemandRetailers()
{
	return unmetDemand_Retailers_bestSolution;
}

vector<vector<vector<double>>> ConstructHeuristic::getDeliveryQuantityRetailers()
{
	return deliveryQuantity_Retailers_bestSolution;
}

vector<vector<vector<vector<vector<int>>>>> ConstructHeuristic::getRoutesWarehouseToRetailer()
{
	return routes_bestSolution;
}

double ConstructHeuristic::getBestObjValue()
{
	return bestObjValue;
}

LocalSearch::LocalSearch(const ParameterSetting &parameters, const Solution &solution)
	: params(parameters), sol_FE(solution)
{
	// ----------------------------------------------------------------------------------------------------------
	RATW = params.getRetailersAssignedToWarehouse();
	// ----------------------------------------------------------------------------------------------------------
}

void LocalSearch::RVND(int s, int w,
					   vector<vector<double>> &Inv_Ret_ScenWare,
					   vector<vector<double>> &unmetDemand_Ret_ScenWare,
					   vector<vector<double>> &delQuant_Ret_ScenWare,
					   vector<vector<vector<int>>> &routes_WareToRet_ScenWare,
					   double &objValue_ScenarioWarehouse)
{
	cout << "\nLocal Search for scenario " << s << " and warehouse " << w << endl;
	// --------------------------------------------------------------------------------------------------------------------------
	LP_ScenarioWarehouse LP(params, sol_FE, s, w, routes_WareToRet_ScenWare);
	string status = LP.solve();
	if (status != "Optimal")
	{
		cerr << "Cannot solve LP for scenario " << s << " and warehouse " << w << endl;
		exit(1);
	}
	Inv_Ret_ScenWare = LP.getInvRetailers();
	unmetDemand_Ret_ScenWare = LP.getUnmetDemandRetailers();
	delQuant_Ret_ScenWare = LP.getDeliveryQuantityRetailers();

	objValue_ScenarioWarehouse = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Ret_ScenWare, unmetDemand_Ret_ScenWare, routes_WareToRet_ScenWare);
	cout << "\nCurrent Objective Value for scenario " << s << " and warehouse " << w << ": " << objValue_ScenarioWarehouse << endl;

	// Initialize the operators
	vector<std::function<bool(int, vector<vector<vector<int>>> &)>> operators = setOperators();
	int maxIterRVND = 25;

	while (!operators.empty() && maxIterRVND > 0)
	{
		vector<vector<vector<int>>> routes_WareToRet_ScenWare_temp = routes_WareToRet_ScenWare;
		vector<vector<double>> Inv_Ret_ScenWare_temp = Inv_Ret_ScenWare;
		vector<vector<double>> unmetDemand_Ret_ScenWare_temp = unmetDemand_Ret_ScenWare;
		vector<vector<double>> delQuant_Ret_ScenWare_temp = delQuant_Ret_ScenWare;

		// double objValue_ScenarioWarehouse_temp = 0.0;
		// Generate random number between 0 and operators.size()
		int index = rand() % operators.size();

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				cout << "Route[" << s << "][" << w << "][" << t << "][" << k << "]: [";
				for (int i = 0; i < routes_WareToRet_ScenWare_temp[t][k].size(); ++i)
				{
					if (i != routes_WareToRet_ScenWare_temp[t][k].size() - 1)
					{
						cout << routes_WareToRet_ScenWare_temp[t][k][i] << " -> ";
					}
					else
					{
						cout << routes_WareToRet_ScenWare_temp[t][k][i];
					}
				}
				cout << "]" << endl;
			}
		}

		cout << "\n";
		// Execute the operator
		if (!operators[index](w, routes_WareToRet_ScenWare_temp))
		{
			// Remove the operator from the list
			operators.erase(operators.begin() + index);
			--maxIterRVND;
			continue;
		}

		LP_ScenarioWarehouse LP(params, sol_FE, s, w, routes_WareToRet_ScenWare_temp);
		// solve LP to get the value of the coninuous variables
		string status = LP.solve();
		if (status != "Optimal")
		{
			operators.erase(operators.begin() + index);
			--maxIterRVND;
			continue;
		}
		Inv_Ret_ScenWare_temp = LP.getInvRetailers();
		unmetDemand_Ret_ScenWare_temp = LP.getUnmetDemandRetailers();
		delQuant_Ret_ScenWare_temp = LP.getDeliveryQuantityRetailers();

		// for (int t = 0; t < params.numPeriods; ++t)
		// {
		// 	for (size_t i = 0; i < RATW[w].size(); ++i)
		// 	{
		// 		cout << "Inv_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << Inv_Ret_ScenWare_temp[i][t] << endl;
		// 		cout << "unmetDemand_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << unmetDemand_Ret_ScenWare_temp[i][t] << endl;
		// 		cout << "delQuant_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << delQuant_Ret_ScenWare_temp[i][t] << endl;
		// 	}
		// }

		double objValue_ScenarioWarehouse_temp = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Ret_ScenWare_temp, unmetDemand_Ret_ScenWare_temp, routes_WareToRet_ScenWare_temp);
		cout << "\nBest objective value: " << objValue_ScenarioWarehouse << endl;
		cout << "Current objective value: " << objValue_ScenarioWarehouse_temp << "\n"
			 << endl;

		if (objValue_ScenarioWarehouse_temp < objValue_ScenarioWarehouse)
		{
			cout << "A Better solution found!" << endl;
			// Update the best solution
			routes_WareToRet_ScenWare = routes_WareToRet_ScenWare_temp;
			Inv_Ret_ScenWare = Inv_Ret_ScenWare_temp;
			unmetDemand_Ret_ScenWare = unmetDemand_Ret_ScenWare_temp;
			delQuant_Ret_ScenWare = delQuant_Ret_ScenWare_temp;

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
	// 	for (size_t i = 0; i < RATW[w].size(); ++i)
	// 	{
	// 		if (Inv_Ret_ScenWare[i][t] != 0){
	// 			cout << "Inventory_Ret[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << Inv_Ret_ScenWare[i][t] << endl;
	// 		}
	// 		if (unmetDemand_Ret_ScenWare[i][t] != 0){
	// 			cout << "unmetDemand_Ret[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << unmetDemand_Ret_ScenWare[i][t] << endl;
	// 		}
	// 		if (delQuant_Ret_ScenWare[i][t] != 0){
	// 			cout << "delQuant_Ret[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << delQuant_Ret_ScenWare[i][t] << endl;
	// 		}
	// 	}
	// }

	cout << "Best Objective Value for scenario " << s << " and warehouse " << w << ": " << objValue_ScenarioWarehouse << endl;
}

vector<std::function<bool(int, vector<vector<vector<int>>> &)>> LocalSearch::setOperators()
{
	std::function<bool(int, vector<vector<vector<int>>> &)> OrOptOneFunc = std::bind(&LocalSearch::OrOpt_One, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> OrOptTwoFunc = std::bind(&LocalSearch::OrOpt_Two, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> OrOptThreeFunc = std::bind(&LocalSearch::OrOpt_Three, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> ShiftOneFunc = std::bind(&LocalSearch::Shift_One, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> ShiftTwoFunc = std::bind(&LocalSearch::Shift_Two, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> ShiftThreeFunc = std::bind(&LocalSearch::Shift_Three, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> SwapFunc = std::bind(&LocalSearch::Swap, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> InsertFunc = std::bind(&LocalSearch::Insert, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> RemoveFunc = std::bind(&LocalSearch::Remove, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> MergeFunc = std::bind(&LocalSearch::Merge, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> TransferFunc = std::bind(&LocalSearch::Transfer, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> RemInsFunc = std::bind(&LocalSearch::Remove_Insert, this, std::placeholders::_1, std::placeholders::_2);

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

bool LocalSearch::OrOpt_One(int w, vector<vector<vector<int>>> &routes)
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
		cout << "Old route[" << w << "][" << t << "][" << k << "]: [";
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

		if (route.size() >= v + 3)
		{
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

		// Move the segment of v retailers to a new position
		vector<int> temp(route.begin() + startPos, route.begin() + startPos + v);
		route.erase(route.begin() + startPos, route.begin() + startPos + v);
		route.insert(route.begin() + newPos, temp.begin(), temp.end());

		cout << "Moved " << v << " retailers from position " << startPos
			 << " to position " << newPos << " in route " << k << std::endl;

		routes[t][k] = route;
		cout << "Updated route[" << w << "][" << t << "][" << k << "]: [";
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

bool LocalSearch::OrOpt_Two(int w, vector<vector<vector<int>>> &routes)
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
		cout << "Old route[" << w << "][" << t << "][" << k << "]: [";
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

		if (route.size() >= v + 3)
		{
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

		// Move the segment of v retailers to a new position
		vector<int> temp(route.begin() + startPos, route.begin() + startPos + v);
		route.erase(route.begin() + startPos, route.begin() + startPos + v);
		route.insert(route.begin() + newPos, temp.begin(), temp.end());

		cout << "Moved " << v << " retailers from position " << startPos
			 << " to position " << newPos << " in route " << k << std::endl;

		routes[t][k] = route;
		cout << "Updated route[" << w << "][" << t << "][" << k << "]: [";
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

bool LocalSearch::OrOpt_Three(int w, vector<vector<vector<int>>> &routes)
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
		cout << "Old route[" << w << "][" << t << "][" << k << "]: [";
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

		if (route.size() >= v + 3)
		{
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

		// Move the segment of v retailers to a new position
		vector<int> temp(route.begin() + startPos, route.begin() + startPos + v);
		route.erase(route.begin() + startPos, route.begin() + startPos + v);
		route.insert(route.begin() + newPos, temp.begin(), temp.end());

		routes[t][k] = route;

		cout << "Moved " << v << " retailers from position " << startPos
			 << " to position " << newPos << " in route " << k << std::endl;

		cout << "Updated route[" << w << "][" << t << "][" << k << "]: [";
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

bool LocalSearch::Shift_One(int w, vector<vector<vector<int>>> &routes)
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

		cout << "Old Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

		cout << "Old Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

		// Check if source route has enough retailers to shift
		if (sourceRoute.size() >= 2 + v)
		{
			cout << "Source route has enough retailers to shift" << endl;
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
				destRoute.insert(destRoute.begin(), w);
				destRoute.insert(destRoute.end(), w);
			}

			cout << "Shifted " << v << " retailers from route " << sourceRouteIndex
				 << " to route " << destRouteIndex << " in period " << t << endl;

			cout << "Updated Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

			cout << "Updated Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

bool LocalSearch::Shift_Two(int w, vector<vector<vector<int>>> &routes)
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

		cout << "Old Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

		cout << "Old Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

		// Check if source route has enough retailers to shift
		if (sourceRoute.size() >= 2 + v)
		{
			cout << "Source route has enough retailers to shift" << endl;
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
				destRoute.insert(destRoute.begin(), w);
				destRoute.insert(destRoute.end(), w);
			}

			cout << "Shifted " << v << " retailers from route " << sourceRouteIndex
				 << " to route " << destRouteIndex << " in period " << t << endl;

			cout << "Updated Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

			cout << "Updated Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

bool LocalSearch::Shift_Three(int w, vector<vector<vector<int>>> &routes)
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

		cout << "Old Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

		cout << "Old Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

		// Check if source route has enough retailers to shift
		if (sourceRoute.size() >= 2 + v)
		{
			cout << "Source route has enough retailers to shift" << endl;
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
				destRoute.insert(destRoute.begin(), w);
				destRoute.insert(destRoute.end(), w);
			}

			cout << "Shifted " << v << " retailers from route " << sourceRouteIndex
				 << " to route " << destRouteIndex << " in period " << t << endl;

			cout << "Updated Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

			cout << "Updated Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

bool LocalSearch::Swap(int w, vector<vector<vector<int>>> &routes)
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

		cout << "Old Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

		cout << "Old Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

			cout << "Swapped " << v1 << " retailers from route " << sourceRouteIndex
				 << " with " << v2 << " retailers from route " << destRouteIndex
				 << " in period " << t << endl;

			cout << "Updated Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

			cout << "Updated Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

bool LocalSearch::Insert(int w, vector<vector<vector<int>>> &routes)
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

		cout << "Old Route[" << w << "][" << t << "][" << rtIndex << "]: [";
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
	for (int node : RATW[w])
	{
		if (visitedNodes.find(node + params.numWarehouses) == visitedNodes.end())
		{
			unvisitedNodes.push_back(node + params.numWarehouses);
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
			double insertionCost = 2 * params.transportationCost_SecondEchelon[w][unvisitedNode];
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
		routes[t][routeToIns].push_back(w);
		routes[t][routeToIns].push_back(unvisitedNode);
		routes[t][routeToIns].push_back(w);
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
		cout << "New Route[" << w << "][" << t << "][" << rtIndex << "]: [";
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

bool LocalSearch::Remove(int w, vector<vector<vector<int>>> &routes)
{
	cout << "Remove" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period
	vector<int> candidateRoutes;

	int rtIndex = 0;
	for (auto &route : routes[t])
	{
		cout << "Old Route[" << w << "][" << t << "][" << rtIndex << "]: [";
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

	// Randomly select a retailer node to remove, avoiding the warehouse nodes at the start and end if present
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
		cout << "New Route[" << w << "][" << t << "][" << rtIndex << "]: [";
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

bool LocalSearch::Merge(int w, vector<vector<vector<int>>> &routes)
{
	cout << "Merge" << endl;

	// Map to store retailer appearances across periods
	std::unordered_map<int, vector<int>> retailerPeriods;

	// Gather retailers and the periods they appear in
	for (int period = 0; period < params.numPeriods; ++period)
	{
		std::unordered_set<int> periodRetailers;
		for (const auto &route : routes[period])
		{
			for (int retailer : route)
			{
				if (retailer != w)
				{
					periodRetailers.insert(retailer);
				}
			}
		}
		for (int retailer : periodRetailers)
		{
			retailerPeriods[retailer].push_back(period);
		}
	}

	// Find retailers that appear in more than one period
	vector<int> eligibleRetailers;
	for (const auto &entry : retailerPeriods)
	{
		if (entry.second.size() > 1)
		{
			eligibleRetailers.push_back(entry.first);
		}
	}

	if (eligibleRetailers.empty())
	{
		cout << "No eligible retailers found for merging." << endl;
		return false;
	}

	// Select a random retailer to merge
	int selectedRetailer = eligibleRetailers[rand() % eligibleRetailers.size()];
	const auto &periods = retailerPeriods[selectedRetailer];

	// Randomly select two different periods to merge retailer presence
	int mergeFrom = periods[rand() % periods.size()];
	int mergeTo;
	do
	{
		mergeTo = periods[rand() % periods.size()];
	} while (mergeTo == mergeFrom);

	int rtIndex = 0;
	for (auto &route : routes[mergeFrom])
	{
		cout << "Old Route (Merge From)[" << w << "][" << mergeFrom << "][" << rtIndex << "]: [";
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

	rtIndex = 0;
	for (auto &route : routes[mergeTo])
	{
		cout << "Old Route (Merge To)[" << w << "][" << mergeTo << "][" << rtIndex << "]: [";
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

	// Remove retailer from mergeFrom period
	for (auto &route : routes[mergeFrom])
	{
		auto it = std::find(route.begin(), route.end(), selectedRetailer);
		if (it != route.end())
		{
			route.erase(it);
			if (route.size() == 2)
			{
				// Clear the route since it only contains the warehouse start and end nodes
				route.clear();
			}
			break; // Assume retailer appears only once per route
		}
	}

	// Ensure the retailer is in the mergeTo period, if not already
	bool found = false;
	for (const auto &route : routes[mergeTo])
	{
		if (std::find(route.begin(), route.end(), selectedRetailer) != route.end())
		{
			found = true;
			break;
		}
	}
	if (!found)
	{
		// Here you need logic to determine where to insert the retailer in mergeTo period
		// For simplicity, let's insert into the first route of the period
		routes[mergeTo][0].push_back(selectedRetailer); // Simplified, consider inserting at a better position
	}

	rtIndex = 0;
	for (auto &route : routes[mergeFrom])
	{
		cout << "New Route (Merge From)[" << w << "][" << mergeFrom << "][" << rtIndex << "]: [";
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

	rtIndex = 0;
	for (auto &route : routes[mergeTo])
	{
		cout << "New Route (Merge To)[" << w << "][" << mergeTo << "][" << rtIndex << "]: [";
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

	cout << "Merged retailer " << selectedRetailer << " from period " << mergeFrom << " to " << mergeTo << endl;
	return true;
}

bool LocalSearch::Transfer(int w, vector<vector<vector<int>>> &routes)
{
	cout << "Transfer" << endl;

	// Randomly select a customer from RATW[w]
	if (RATW[w].empty())
	{
		cout << "No customers assigned to warehouse " << w << endl;
		return false;
	}
	int customer = RATW[w][rand() % RATW[w].size()] + params.numWarehouses;

	vector<int> visitedPeriods;
	vector<int> unvisitedPeriods;

	// Determine where the customer is visited and where they are not
	for (int period = 0; period < params.numPeriods; ++period)
	{
		bool found = false;
		for (auto &route : routes[period])
		{
			if (std::find(route.begin(), route.end(), customer) != route.end())
			{
				found = true;
				visitedPeriods.push_back(period);
				break;
			}
		}
		if (!found)
		{
			unvisitedPeriods.push_back(period);
		}
	}

	// Remove the customer from all periods they are currently visited
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

	// Attempt to insert the customer into all periods they are not visited
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
				double insertionCost = 2 * params.transportationCost_SecondEchelon[w][customer];
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
			routes[t][routeToIns].push_back(w);
			routes[t][routeToIns].push_back(customer);
			routes[t][routeToIns].push_back(w);
			inserted = true;
		}
	}

	return inserted;
}

bool LocalSearch::Remove_Insert(int w, vector<vector<vector<int>>> &routes)
{
	cout << "Remove/Insert" << endl;

	// Step 1: Find a period with at least one customer and randomly select one
	vector<int> visitedPeriods;
	for (int period = 0; period < params.numPeriods; ++period)
	{
		std::unordered_set<int> uniqueCustomers;
		for (const auto &route : routes[period])
		{
			for (int customer : route)
			{
				if (customer != w)
				{ // Assuming 'w' is the warehouse node
					uniqueCustomers.insert(customer);
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
			if (customer != w)
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
		if (period == fromPeriod)
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
			double insertionCost = 2 * params.transportationCost_SecondEchelon[w][selectedCustomer];
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
		routes[toPeriod][routeToIns].push_back(w);
		routes[toPeriod][routeToIns].push_back(selectedCustomer);
		routes[toPeriod][routeToIns].push_back(w);
	}

	cout << "Moved customer " << selectedCustomer << " from period " << fromPeriod << " to period " << toPeriod << "." << endl;
	return true;
}

double LocalSearch::calculateObjFuncValue_ScenarioWarehouse(int s, int w, const vector<vector<double>> &Inv_Ret_Temp, const vector<vector<double>> &unmetDem_Ret_Temp, const vector<vector<vector<int>>> &route_WTR_Temp)
{
	// cout << "\ncalculating objFuncValue_ScenarioWarehouse" << endl;
	// for (int t = 0; t < params.numPeriods; ++t)
	// {
	// 	for (int i = 0; i < RATW[w].size(); ++i)
	// 	{
	// 		if (Inv_Ret_Temp[i][t] > 0.0)
	// 		{
	// 			cout << "ret_Inv[" << i << "][" << t << "] = " << Inv_Ret_Temp[i][t] << endl;
	// 		}

	// 		if (unmetDem_Ret_Temp[i][t] > 0.0)
	// 		{
	// 			cout << "ret_unmDem[" << i << "][" << t << "] = " << unmetDem_Ret_Temp[i][t] << endl;
	// 		}
	// 	}
	// }

	double objFuncValue = 0.0;

	double inventoryCost = 0.0;
	double unmetDemandCost = 0.0;
	double routeCost_Warehouse = 0.0;

	// Inventory Holding Cost and Unmet Demand Cost
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < RATW[w].size(); ++i)
		{
			objFuncValue += params.unitHoldingCost_Retailer[RATW[w][i]] * Inv_Ret_Temp[i][t];
			objFuncValue += params.unmetDemandPenalty[RATW[w][i]] * unmetDem_Ret_Temp[i][t];

			inventoryCost += params.unitHoldingCost_Retailer[RATW[w][i]] * Inv_Ret_Temp[i][t];
			// cout << params.unitHoldingCost_Retailer[RATW[w][i]] << " * " << Inv_Ret_Temp[i][t] << " = " << params.unitHoldingCost_Retailer[RATW[w][i]] * Inv_Ret_Temp[i][t] << endl;
			unmetDemandCost += params.unmetDemandPenalty[RATW[w][i]] * unmetDem_Ret_Temp[i][t];
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			double routeCost = 0.0;

			int previousNode = w;
			for (int j = 1; j < route_WTR_Temp[t][k].size(); ++j)
			{
				int currentNode = route_WTR_Temp[t][k][j];
				objFuncValue += params.transportationCost_SecondEchelon[previousNode][currentNode];

				routeCost += params.transportationCost_SecondEchelon[previousNode][currentNode];

				routeCost_Warehouse += params.transportationCost_SecondEchelon[previousNode][currentNode];
				previousNode = currentNode;
			}

			// cout << "Route[" << w << "][" << t << "][" << k << "]: [";
			// for (int i = 0; i < route_WTR_Temp[t][k].size(); ++i)
			// {
			// 	if (i != route_WTR_Temp[t][k].size() - 1)
			// 	{
			// 		cout << route_WTR_Temp[t][k][i] << " -> ";
			// 	}
			// 	else
			// 	{
			// 		cout << route_WTR_Temp[t][k][i];
			// 	}
			// }
			// cout << "]" << endl;

			// cout << "Route Cost(RVND)[" << s << "][" << w << "][" << t << "][" << k << "]: " << routeCost << endl;
		}
	}

	// cout << "Inventory Cost (RVND)[" << s << "][" << w << "]: " << inventoryCost << endl;
	// cout << "Unmet Demand Cost (RVND)[" << s << "][" << w << "]: " << unmetDemandCost << endl;
	// cout << "Route Cost (RVND)[" << s << "][" << w << "]: " << routeCost_Warehouse << endl;

	// cout << "objFuncValue:" << objFuncValue << endl;
	return objFuncValue;
}

// --------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
Perturbation::Perturbation(const ParameterSetting &parameters, const Solution &solution)
	: params(parameters), sol_FE(solution)
{
	// ----------------------------------------------------------------------------------------------------------
	RATW = params.getRetailersAssignedToWarehouse();
	// ----------------------------------------------------------------------------------------------------------
}

bool Perturbation::run(int s, int w,
					   vector<vector<double>> &Inv_Ret_ScenWare,
					   vector<vector<double>> &unmetDemand_Ret_ScenWare,
					   vector<vector<double>> &delQuant_Ret_ScenWare,
					   vector<vector<vector<int>>> &routes_WareToRet_ScenWare)
{
	cout << "\nPerturbation for scenario " << s << " and warehouse " << w << endl;
	// --------------------------------------------------------------------------------------------------------------------------
	// objValue_ScenarioWarehouse = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Ret_ScenWare, unmetDemand_Ret_ScenWare, routes_WareToRet_ScenWare);
	// cout << "\nCurrent Objective Value for scenario " << s << " and warehouse " << w << ": " << objValue_ScenarioWarehouse << endl;

	// Initialize the operators
	vector<std::function<bool(int, vector<vector<vector<int>>> &)>> perturbOperators = setPerturbOperators();
	int max_perturb = 10;

	while (max_perturb > 0)
	{
		vector<vector<vector<int>>> routes_WareToRet_ScenWare_temp = routes_WareToRet_ScenWare;
		vector<vector<double>> Inv_Ret_ScenWare_temp = Inv_Ret_ScenWare;
		vector<vector<double>> unmetDemand_Ret_ScenWare_temp = unmetDemand_Ret_ScenWare;
		vector<vector<double>> delQuant_Ret_ScenWare_temp = delQuant_Ret_ScenWare;

		// double objValue_ScenarioWarehouse_temp = 0.0;
		// Generate random number between 0 and operators.size()
		int index = rand() % perturbOperators.size();

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				cout << "Route[" << s << "][" << w << "][" << t << "][" << k << "]: [";
				for (int i = 0; i < routes_WareToRet_ScenWare_temp[t][k].size(); ++i)
				{
					if (i != routes_WareToRet_ScenWare_temp[t][k].size() - 1)
					{
						cout << routes_WareToRet_ScenWare_temp[t][k][i] << " -> ";
					}
					else
					{
						cout << routes_WareToRet_ScenWare_temp[t][k][i];
					}
				}
				cout << "]" << endl;
			}
		}
		cout << "\n";

		// Execute the operator
		if (!perturbOperators[index](w, routes_WareToRet_ScenWare_temp))
		{
			--max_perturb;
			continue;
		}

		LP_ScenarioWarehouse LP(params, sol_FE, s, w, routes_WareToRet_ScenWare_temp);
		// solve LP to get the value of the coninuous variables
		string status = LP.solve();
		if (status != "Optimal")
		{
			--max_perturb;
			continue;
		}
		Inv_Ret_ScenWare = LP.getInvRetailers();
		unmetDemand_Ret_ScenWare = LP.getUnmetDemandRetailers();
		delQuant_Ret_ScenWare = LP.getDeliveryQuantityRetailers();
		routes_WareToRet_ScenWare = routes_WareToRet_ScenWare_temp;

		// for (int t = 0; t < params.numPeriods; ++t)
		// {
		// 	for (size_t i = 0; i < RATW[w].size(); ++i)
		// 	{
		// 		cout << "Inv_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << Inv_Ret_ScenWare_temp[i][t] << endl;
		// 		cout << "unmetDemand_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << unmetDemand_Ret_ScenWare_temp[i][t] << endl;
		// 		cout << "delQuant_Ret_temp[" << RATW[w][i] << "][" << t <<  "][" << s << "] = " << delQuant_Ret_ScenWare_temp[i][t] << endl;
		// 	}
		// }

		// double objValue_ScenarioWarehouse_temp = calculateObjFuncValue_ScenarioWarehouse(s, w, Inv_Ret_ScenWare_temp, unmetDemand_Ret_ScenWare_temp, routes_WareToRet_ScenWare_temp);
		// cout << "Current objective value: " << objValue_ScenarioWarehouse_temp << "\n" << endl;

		return true;
	}
	return false;
}

vector<std::function<bool(int, vector<vector<vector<int>>> &)>> Perturbation::setPerturbOperators()
{
	std::function<bool(int, vector<vector<vector<int>>> &)> randomShiftFunc = std::bind(&Perturbation::randomShift, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> randomInsertionFunc = std::bind(&Perturbation::randomInsertion, this, std::placeholders::_1, std::placeholders::_2);
	std::function<bool(int, vector<vector<vector<int>>> &)> randomRemovalFunc = std::bind(&Perturbation::randomRemoval, this, std::placeholders::_1, std::placeholders::_2);

	return {
		randomShiftFunc,
		randomInsertionFunc,
		randomRemovalFunc};
}

bool Perturbation::randomShift(int w, vector<vector<vector<int>>> &routes)
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

		cout << "Old Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

		cout << "Old Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

		// Check if source route has enough retailers to shift
		if (sourceRoute.size() >= 2 + v)
		{
			cout << "Source route has enough retailers to shift" << endl;
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
				destRoute.insert(destRoute.begin(), w);
				destRoute.insert(destRoute.end(), w);
			}

			cout << "Shifted " << v << " retailers from route " << sourceRouteIndex
				 << " to route " << destRouteIndex << " in period " << t << endl;

			cout << "Updated Source route[" << w << "][" << t << "][" << sourceRouteIndex << "]: [";
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

			cout << "Updated Dest route[" << w << "][" << t << "][" << destRouteIndex << "]: [";
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

bool Perturbation::randomInsertion(int w, vector<vector<vector<int>>> &routes)
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

		cout << "Old Route[" << w << "][" << t << "][" << rtIndex << "]: [";
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
	for (int node : RATW[w])
	{
		if (visitedNodes.find(node + params.numWarehouses) == visitedNodes.end())
		{
			unvisitedNodes.push_back(node + params.numWarehouses);
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
			double insertionCost = 2 * params.transportationCost_SecondEchelon[w][unvisitedNode];
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
		routes[t][routeToIns].push_back(w);
		routes[t][routeToIns].push_back(unvisitedNode);
		routes[t][routeToIns].push_back(w);
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
		cout << "New Route[" << w << "][" << t << "][" << rtIndex << "]: [";
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

bool Perturbation::randomRemoval(int w, vector<vector<vector<int>>> &routes)
{
	cout << "Random Removal Perturbation" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period
	vector<int> candidateRoutes;

	int rtIndex = 0;
	for (auto &route : routes[t])
	{
		cout << "Old Route[" << w << "][" << t << "][" << rtIndex << "]: [";
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

	// Randomly select a retailer node to remove, avoiding the warehouse nodes at the start and end if present
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
		cout << "New Route[" << w << "][" << t << "][" << rtIndex << "]: [";
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
LP_ScenarioWarehouse::LP_ScenarioWarehouse(const ParameterSetting &parameters, const Solution &solution,
										   int scenario,
										   int warehouse,
										   vector<vector<vector<int>>> routes_WareToRet_ScenWare)
	: params(parameters),
	  sol_FE(solution),
	  scenario(scenario),
	  warehouse(warehouse),
	  routes_WareToRet_ScenWare(routes_WareToRet_ScenWare)
{
	// ----------------------------------------------------------------------------------------------------------
	cout << "Solving LP for scenario " << scenario << " and warehouse " << warehouse << endl;

	// Initialize random seed
	RATW = params.getRetailersAssignedToWarehouse();
	numRetailers_w = RATW[warehouse].size();
	// ----------------------------------------------------------------------------------------------------------
	ret_Inv.resize(numRetailers_w, vector<double>(params.numPeriods, 0.0));
	ret_unmDem.resize(numRetailers_w, vector<double>(params.numPeriods, 0.0));
	ret_delQuant.resize(numRetailers_w, vector<double>(params.numPeriods, 0.0));
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
	// Define I[i][t][s] variables - inventory level at retailer i, period t and scenario s (retailers are assigned to warehouses -> i in N_w))
	IloArray<IloNumVarArray> I_retailer = varManager.create2D(numRetailers_w, params.numPeriods);
	for (int i = 0; i < numRetailers_w; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "I_retailer[" + std::to_string(i) + "][" + std::to_string(t) + "]";
			I_retailer[i][t] = IloNumVar(env, 0.0, params.storageCapacity_Retailer[RATW[warehouse][i]], IloNumVar::Float, varName.c_str());
			model.add(I_retailer[i][t]);
		}
	}

	// Define b[i][t][s] variables - unmet demand at retailer i, period t and scenario s (retailers are assigned to warehouses -> i in N_w)
	IloArray<IloNumVarArray> b_retailer = varManager.create2D(numRetailers_w, params.numPeriods);
	for (int i = 0; i < numRetailers_w; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "b_retailer[" + std::to_string(i) + "][" + std::to_string(t) + "]";
			b_retailer[i][t] = IloNumVar(env, 0.0, params.demand[RATW[warehouse][i]][t][scenario], IloNumVar::Float, varName.c_str());
			model.add(b_retailer[i][t]);
		}
	}

	// Define w[i][t][k][s] variables - delivery to retailer i, period t with vehicle k and scenario s (retailers are assigned to warehouses -> i in N_w)
	IloArray<IloArray<IloNumVarArray>> w_retailer = varManager.create3D(numRetailers_w, params.numPeriods, params.numVehicles_Warehouse);
	for (int i = 0; i < numRetailers_w; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				string varName = "w_retailer[" + std::to_string(i) + "][" + std::to_string(t) + "][" + std::to_string(k) + "]";
				w_retailer[i][t][k] = IloNumVar(env, 0.0, params.DeliveryUB_perRetailer[RATW[warehouse][i]][t][scenario], IloNumVar::Float, varName.c_str());
				model.add(w_retailer[i][t][k]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	/* Define objective function */
	// -------------------------------------------------------------------------------------------------------------------------------
	IloExpr obj(env);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < numRetailers_w; ++i)
		{
			obj += params.unitHoldingCost_Retailer[RATW[warehouse][i]] * I_retailer[i][t];
			obj += params.unmetDemandPenalty[RATW[warehouse][i]] * b_retailer[i][t];
		}
	}
	model.add(IloMinimize(env, obj));
	// -------------------------------------------------------------------------------------------------------------------------------
	/* Define Constraints */
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Balance Constraints (Retailers):
			I[i][t]^([s]) = I[i][t-1]^([s]) + w[i][t]^([s]) - d[i][t]^([s]) + b[i][t]^([s]) 	for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < numRetailers_w; ++i)
		{
			string constraintName = "RetailerInventoryBalance(" + std::to_string(RATW[warehouse][i] + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			if (t == 0)
			{
				expr += I_retailer[i][t];
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					expr += -w_retailer[i][t][k];
				}
				expr += -b_retailer[i][t];
				IloConstraint RetailerInventoryBalanceConstraint(expr == params.initialInventory_Retailer[RATW[warehouse][i]] - params.demand[RATW[warehouse][i]][t][scenario]);
				expr.end();

				model.add(RetailerInventoryBalanceConstraint).setName(constraintName.c_str());
			}
			else
			{
				expr += I_retailer[i][t];
				expr += -I_retailer[i][t - 1];
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					expr += -w_retailer[i][t][k];
				}
				expr += -b_retailer[i][t];
				IloConstraint RetailerInventoryBalanceConstraint(expr == -params.demand[RATW[warehouse][i]][t][scenario]);
				expr.end();

				model.add(RetailerInventoryBalanceConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Capacity Constraints (Retailers):
			I[i][t]^([s]) + d[i][t]^([s]) <= params.storageCapacity_Retailer[i] 		for all i in N_w, t in T
	// */
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < numRetailers_w; ++i)
		{
			string constraintName = "RetailerInventoryCapacity(" + std::to_string(RATW[warehouse][i] + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			expr += I_retailer[i][t];
			IloConstraint RetailerInventoryCapacityConstraint(expr <= params.storageCapacity_Retailer[RATW[warehouse][i]] - params.demand[RATW[warehouse][i]][t][scenario]);
			expr.end();

			model.add(RetailerInventoryCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define delivery constraints to limit the deliveries to the available inventory of the warehouse:
			sum(i in N_w) sum(k in K) w[i][t][k]^([s]) <= warehouseInventory[warehouse][t-1]^([s]) + warehouseDelivery[warehouse][t]			for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "WarehouseInventoryLimitConstraint(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			for (int i = 0; i < numRetailers_w; ++i)
			{
				expr += w_retailer[i][t][k];
			}
		}

		if (t == 0)
		{
			IloConstraint warehouseInvLimitConstraint(expr <= params.initialInventory_Warehouse[warehouse] + sol_FE.deliveryQuantityToWarehouse[warehouse][t]);
			expr.end();

			model.add(warehouseInvLimitConstraint).setName(constraintName.c_str());
		}
		else
		{
			IloConstraint warehouseInvLimitConstraint(expr <= sol_FE.warehouseInventory[warehouse][t - 1][scenario] + sol_FE.deliveryQuantityToWarehouse[warehouse][t]);

			expr.end();

			model.add(warehouseInvLimitConstraint).setName(constraintName.c_str());
		}
	}
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

			for (int i = 0; i < numRetailers_w; ++i)
			{
				expr += w_retailer[i][t][k];
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
			for (int i = 0; i < numRetailers_w; ++i)
			{
				string constraintName = "deliveryLimitConstraint(" + std::to_string(RATW[warehouse][i] + params.numWarehouses) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

				auto it = std::find(routes_WareToRet_ScenWare[t][k].begin() + 1, routes_WareToRet_ScenWare[t][k].end() - 1, RATW[warehouse][i] + params.numWarehouses);
				if (it != routes_WareToRet_ScenWare[t][k].end() - 1)
				{
					IloExpr expr(env);
					expr += w_retailer[i][t][k];
					expr += -params.DeliveryUB_perRetailer[RATW[warehouse][i]][t][scenario];
					IloConstraint deliveryLimitConstraint(expr <= 0);
					expr.end();

					model.add(deliveryLimitConstraint).setName(constraintName.c_str());
				}
				else
				{
					IloExpr expr(env);
					expr += w_retailer[i][t][k];
					IloConstraint deliveryLimitConstraint(expr == 0);
					expr.end();

					model.add(deliveryLimitConstraint).setName(constraintName.c_str());
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/* Assure linear mappings between the presolved and original models */
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

	if (save_lpFile)
	{
		string directory = "../cplexFiles/lpModel/";
		string lpFileName = directory + "ILS_LP" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numRetailers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_scenario" + std::to_string(scenario) + "_warehouse" + std::to_string(warehouse) + "_Ins" + params.instance.c_str() + ".lp";

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
			string solFileName = directory + "ILS_LP" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numRetailers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_scenario" + std::to_string(scenario) + "_warehouse" + std::to_string(warehouse) + "_Ins" + params.instance.c_str();

			// Export the model to an LP file
			cplex.writeSolution(solFileName.c_str());
		}

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < numRetailers_w; ++i)
			{
				ret_Inv[i][t] = cplex.getValue(I_retailer[i][t]);
				ret_unmDem[i][t] = cplex.getValue(b_retailer[i][t]);

				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					ret_delQuant[i][t] += cplex.getValue(w_retailer[i][t][k]);
				}
			}
		}

		// for (int t = 0; t < params.numPeriods; ++t)
		// {
		// 	for (int i = 0; i < numRetailers_w; ++i)
		// 	{
		// 		if (ret_Inv[i][t] > 0.0){
		// 			cout << "ret_Inv[" << RATW[warehouse][i] + params.numWarehouses << "][" << t << "] = " << ret_Inv[i][t] << endl;
		// 		}

		// 		if (ret_unmDem[i][t] > 0.0){
		// 			cout << "ret_unmDem[" << RATW[warehouse][i] + params.numWarehouses << "][" << t << "] = " << ret_unmDem[i][t] << endl;
		// 		}

		// 		if (ret_delQuant[i][t] > 0.0){
		// 			cout << "ret_delQuant[" << RATW[warehouse][i] + params.numWarehouses << "][" << t << "] = " << ret_delQuant[i][t] << endl;
		// 		}
		// 	}
		// }
	}
	else if (cplex.getStatus() == IloAlgorithm::Infeasible)
	{
		status = "Infeasible";
		cout << "Problem is infeasible" << endl;
	}
	else
	{
		status = "Undefined";
		cout << "Solver terminated with status: " << status << endl;
	}

	env.end();

	return status;
}

vector<vector<double>> LP_ScenarioWarehouse::getInvRetailers()
{
	return ret_Inv;
}

vector<vector<double>> LP_ScenarioWarehouse::getUnmetDemandRetailers()
{
	return ret_unmDem;
}

vector<vector<double>> LP_ScenarioWarehouse::getDeliveryQuantityRetailers()
{
	return ret_delQuant;
}
