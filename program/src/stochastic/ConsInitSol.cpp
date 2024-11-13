#include "stochastic/ConsInitSol.h"

ConstructHeuristic::ConstructHeuristic(const ParameterSetting &parameters, const SolutionFirstEchelon &sol_FE)
	: params(parameters),
	  solFE_init(sol_FE)
{
	// ----------------------------------------------------------------------------------------------------------
	cout << "Construction Heuristic" << endl;

	// We calculate the Customers per unit unemt demand penalty cost to approximate per unit demand satisfaction cost ratio
	// We order Customers in descending order to have them from highest to lowest penalty cost ratio. (this shows the prioritun in demand satisfaction)
	sortedWarehouseByDistance = params.getSortedWarehousesByDistance();
	orderCustomersByUnmetDemandToDeliveryRatio(sorted_Customers_byPenaltyCostRatio);
	CATW = params.getCustomersAssignedToWarehouse();

	bestObjValue = 0.0;
	Inv_Customers_bestSolution.resize(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	unmetDemand_Customers_bestSolution.resize(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	deliveryQuantity_Customers_bestSolution.resize(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	routes_bestSolution.resize(params.numScenarios, vector<vector<vector<vector<int>>>>(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>()))));
}

void ConstructHeuristic::orderCustomersByUnmetDemandToDeliveryRatio(vector<int> &sorted_customer_costRatio)
{
	// calculate the stockout to demand satisfaction cost ratio using: Ratio = penaltyCost[i] / F/C + 2c[0][closest warehouse] + 2c[closest warehouse][i]
	vector<std::pair<int, double>> customer_costRatio;
	for (int i = 0; i < params.numCustomers; ++i)
	{
		int customerIndex = i + params.numWarehouses;
		double costRatio = (params.unmetDemandPenalty[i]) /
							( params.unitProdCost +
							(params.setupCost / params.prodCapacity) +
							((2 * params.transportationCost_FirstEchelon[0][sortedWarehouseByDistance[i][0] + 1]) / params.vehicleCapacity_Plant) +
							((2 * params.transportationCost_SecondEchelon[sortedWarehouseByDistance[i][0]][customerIndex]) / params.vehicleCapacity_Warehouse));

		customer_costRatio.emplace_back(customerIndex, costRatio);
	}

	std::sort(customer_costRatio.begin(), customer_costRatio.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b)
				{ return a.second > b.second; });

	for (const auto &pair : customer_costRatio)
	{
		sorted_customer_costRatio.push_back(pair.first);
	}
}

void ConstructHeuristic::calculateDecisionVariables(int s, int w, vector<vector<double>> &Inv_Customers, vector<vector<double>> &unmetDemand_Customers, vector<vector<double>> &deliveryQuantity_Customers)
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[s][t][w][i] == 1)
			{
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
	for (int i : sorted_Customers_byPenaltyCostRatio)
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
	for (int i : sorted_Customers_byPenaltyCostRatio)
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
					tempDeliveryQuantity[custIndex] = std::min({unetDemandCustomers_lookAhead + tempDeliveryQuantity[custIndex], params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[custIndex] - params.initialInventory_Customer[custIndex])});
				}
				else
				{
					tempDeliveryQuantity[custIndex] = std::min({unetDemandCustomers_lookAhead + tempDeliveryQuantity[custIndex], params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[custIndex] - Inv_Customers[custIndex][t - 1])});
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
		remainingAvailInventoryWarehouse += solFE_init.deliveryQuantityToWarehouse[w][l];
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[s][t][w][i] == 1)
			{
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
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[s][t][w][i] == 1)
			{
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
			vector<vector<double>> Inv_Customers_Scenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
			vector<vector<double>> unmetDemand_Customers_Scenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
			vector<vector<double>> deliveryQuantity_Customers_Scenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
			vector<vector<vector<vector<int>>>> routes_Scenario(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>())));
			
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

						// cout << "setOne Scenario " << s << " Warehouse " << w << " Period " << t << ": [";
						// for (auto it = setOne.begin(); it != setOne.end(); ++it)
						// {
						// 	if (it != setOne.begin())
						// 	{
						// 		cout << ", ";
						// 	}
						// 	cout << *it << ": " << tempDeliveryQuantity[*it - params.numWarehouses];
						// }
						// cout << "]" << endl;

						// cout << "setTwo Scenario " << s << " Warehouse " << w << " Period " << t << ": [";
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
							for (int i = 0; i < params.numCustomers; ++i)
							{
								if (CATW[s][t][w][i] == 1)
								{
									Inv_Customers_Scenario[i][t] = Inv_Customers[i][t];
									unmetDemand_Customers_Scenario[i][t] = unmetDemand_Customers[i][t];
									deliveryQuantity_Customers_Scenario[i][t] = deliveryQuantity_Customers[i][t];
								}
							}

							for (int k = 0; k < params.numVehicles_Warehouse; ++k)
							{
								routes_Scenario[w][t][k] = routesPeriod[t][k];
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
						Inv_Customers_bestSolution[i][t][s] = Inv_Customers_Scenario[i][t];
						unmetDemand_Customers_bestSolution[i][t][s] = unmetDemand_Customers_Scenario[i][t];
						deliveryQuantity_Customers_bestSolution[i][t][s] = deliveryQuantity_Customers_Scenario[i][t];
					}
				}

				for (int w = 0; w < params.numWarehouses; ++w)
				{
					for (int t = 0; t < params.numPeriods; ++t)
					{
						for (int k = 0; k < params.numVehicles_Warehouse; ++k)
						{
							routes_bestSolution[s][w][t][k] = routes_Scenario[w][t][k];
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


/*
void ConstructHeuristic::orderCustomersByUnmetDemandToDeliveryRatio(vector<int> &sorted_customer_costRatio)
{
	// calculate the stockout to demand satisfaction cost ratio using: Ratio = penaltyCost[i] / F/C + 2c[0][closest warehouse] + 2c[closest warehouse][i]

	vector<std::pair<int, double>> customer_costRatio;
	for (int i = 0; i < params.numCustomers; ++i)
	{
		int customerIndex = i + params.numWarehouses;
		double costRatio = (params.unmetDemandPenalty[i]) /
							((params.setupCost / params.prodCapacity) +
							((2 * params.transportationCost_FirstEchelon[0][sortedWarehouseByDistance[i][0] + 1]) / params.vehicleCapacity_Plant) +
							((2 * params.transportationCost_SecondEchelon[sortedWarehouseByDistance[i][0]][customerIndex]) / params.vehicleCapacity_Warehouse));

		customer_costRatio.emplace_back(i, costRatio);
	}

	std::sort(customer_costRatio.begin(), customer_costRatio.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b)
				{ return a.second > b.second; });

	for (const auto &pair : customer_costRatio)
	{
		sorted_customer_costRatio.push_back(pair.first);
	}
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
			double bestObjValue_ScenarioWarehouse = std::numeric_limits<double>::max();

			int look_ahead = 1;
			while (look_ahead <= std::ceil(params.numPeriods / 2.0))
			{
				vector<vector<double>> customerInv_Scenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
				vector<vector<double>> customerUnmetDemand_Scenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
				vector<vector<double>> customerDeliveryQuantity_Scenario(params.numCustomers, vector<double>(params.numPeriods, 0.0));
				vector<vector<vector<vector<int>>>> warehouseToCustomerRoutes_Scenario(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>())));
				vector<vector<vector<int>>> tempCATW(params.numPeriods, vector<vector<int>>(params.numWarehouses, vector<int>(params.numCustomers, 0)));

				calcDecVarValues_Scenario(s, customerInv_Scenario, customerUnmetDemand_Scenario, customerDeliveryQuantity_Scenario);

				for (int t = 0; t < params.numPeriods; ++t)
				{
					vector<double> tempDeliveryQuantity(params.numCustomers, 0.0);

					vector<int> setOne;
					defineSetOne(s, t, setOne, customerInv_Scenario, customerUnmetDemand_Scenario, tempDeliveryQuantity);

					vector<int> setTwo;
					if (t < params.numPeriods - 1)
					{
						defineSetTwo(s, t, look_ahead, setOne, setTwo, customerInv_Scenario, customerUnmetDemand_Scenario, tempDeliveryQuantity);
					}

					// cout << "setOne Scenario " << s << " Period " << t << ": [";
					// for (auto it = setOne.begin(); it != setOne.end(); ++it)
					// {
					// 	if (it != setOne.begin())
					// 	{
					// 		cout << ", ";
					// 	}
					// 	cout << *it << ": " << tempDeliveryQuantity[*it];
					// }
					// cout << "]" << endl;

					// cout << "setTwo Scenario " << s << " Period " << t << ": [";
					// for (auto it = setTwo.begin(); it != setTwo.end(); ++it)
					// {
					// 	if (it != setTwo.begin())
					// 	{
					// 		cout << ", ";
					// 	}
					// 	cout << *it << ": " << tempDeliveryQuantity[*it];
					// }
					// cout << "]" << endl;
					// cout << "\n\n" << endl;

					nearestNeighourInsertion(s, t, setOne, setTwo, customerDeliveryQuantity_Scenario, tempDeliveryQuantity, warehouseToCustomerRoutes_Scenario, tempCATW);
					calcDecVarValues_Scenario(s, customerInv_Scenario, customerUnmetDemand_Scenario, customerDeliveryQuantity_Scenario);
				}

				double objFuncValue_Scenario = calculateObjFuncValue(s, customerInv_Scenario, customerUnmetDemand_Scenario, warehouseToCustomerRoutes_Scenario);

				if (objFuncValue_Scenario < bestObjValue_ScenarioWarehouse)
				{
					// cout << "New best feasible solution found = " << objFuncValue_temp << endl;

					bestObjValue_ScenarioWarehouse = objFuncValue_Scenario;
					
					for (int t = 0; t < params.numPeriods; ++t)
					{
						for (int i = 0; i < params.numCustomers; ++i)
						{
							Inv_Customers_bestSolution[i][t][s] = customerInv_Scenario[i][t];
							unmetDemand_Customers_bestSolution[i][t][s] = customerUnmetDemand_Scenario[i][t];
							deliveryQuantity_Customers_bestSolution[i][t][s] = customerDeliveryQuantity_Scenario[i][t];
						}

						customerAssignmentToWarehouse_bestSolution[s] = tempCATW;
						routes_bestSolution[s] = warehouseToCustomerRoutes_Scenario;
					}
				}
				++look_ahead;
			}

			bestObjValue += params.probability[s] * bestObjValue_ScenarioWarehouse;
		}

		auto currentTime_InitialSolution = std::chrono::high_resolution_clock::now();
		elapsedTime_InitialSolution = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime_InitialSolution - startTime_InitialSolution).count();

		cout << "\nRoutes (Warehouse To Customer):" << endl;
		for (int s = 0; s < params.numScenarios; ++s)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int t = 0; t < params.numPeriods; ++t)
				{
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						if (!routes_bestSolution[s][w][t][k].empty()){
							cout << "route[" << s + 1 << "][" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] : [";
							for (auto it = routes_bestSolution[s][w][t][k].begin(); it != routes_bestSolution[s][w][t][k].end(); ++it)
							{
								if (it != routes_bestSolution[s][w][t][k].begin())
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
	}
	catch (std::exception &e)
	{
		return false;
	}
	cout << "Initial Solution Constructed Successfully" << endl;

	cout << "Time - Construct Initial Solution (seconds): " << std::setprecision(3) << std::fixed << elapsedTime_InitialSolution << endl;
	return true;
}

void ConstructHeuristic::calcDecVarValues_Scenario(int s, vector<vector<double>> &Inv_Customers, vector<vector<double>> &unmetDemand_Customers, vector<vector<double>> &deliveryQuantity_Customers)
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			Inv_Customers[i][t] = params.initialInventory_Customer[i];
			for (int l = 0; l <= t; ++l){
				Inv_Customers[i][t] += deliveryQuantity_Customers[i][t];
				Inv_Customers[i][t] -= params.demand[i][t][s];
			}
			
			unmetDemand_Customers[i][t] = 0.0;
			if (Inv_Customers[i][t] < 0.0)
			{
				unmetDemand_Customers[i][t] = std::abs(Inv_Customers[i][t]);
				Inv_Customers[i][t] = 0.0;
			}
		}
	}
}

double ConstructHeuristic::calculateObjFuncValue(int s, const vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, const vector<vector<vector<vector<int>>>> &routes)
{
	double objFuncValue = 0.0;

	// Inventory Holding Cost and Unmet Demand Cost
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			objFuncValue += params.unitHoldingCost_Customer[i] * Inv_Customers[i][t];
			objFuncValue += params.unmetDemandPenalty[i] * unmetDemand_Customers[i][t];
		}
	}

	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				int previousNode = w;
				for (int j = 1; j < routes[w][t][k].size(); ++j)
				{
					int currentNode = routes[w][t][k][j];
					objFuncValue += params.transportationCost_SecondEchelon[previousNode][currentNode];
					previousNode = currentNode;
				}
			}
		}
	}

	return objFuncValue;
}

void ConstructHeuristic::defineSetOne(int s, int t, vector<int> &setOne, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity)
{
	for (int i : sorted_Customers_byPenaltyCostRatio)
	{
		if (unmetDemand_Customers[i][t] > 0.0)
		{
			setOne.push_back(i);

			if (t == 0)
			{
				tempDeliveryQuantity[i] = std::min({unmetDemand_Customers[i][t], params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[i] - params.initialInventory_Customer[i])});
			}
			else if (t == params.numPeriods - 1)
			{
				tempDeliveryQuantity[i] = std::min(params.vehicleCapacity_Warehouse, unmetDemand_Customers[i][t]);
			}
			else
			{
				tempDeliveryQuantity[i] = std::min({unmetDemand_Customers[i][t], params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[i] - Inv_Customers[i][t - 1])});
			}
		}
	}
}

void ConstructHeuristic::defineSetTwo(int s, int t, int look_ahead, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity)
{
	for (int i : sorted_Customers_byPenaltyCostRatio)
	{
		double unmetDemandCustomers_lookAhead = 0.0;
		for (int l = t + 1; l <= std::min(t + look_ahead, params.numPeriods - 1); ++l)
		{
			// cout << "look ahead demand [" << i << "] = " << unmetDemand_Customers[custIndex][l] << endl;
			unmetDemandCustomers_lookAhead += unmetDemand_Customers[i][l];
		}

		if (unmetDemandCustomers_lookAhead > 0)
		{
			// check if customer i is in setOne
			auto it = std::find(setOne.begin(), setOne.end(), i);
			if (it != setOne.end())
			{
				if (t == 0)
				{
					tempDeliveryQuantity[i] = std::min({unmetDemandCustomers_lookAhead + tempDeliveryQuantity[i], params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[i] - params.initialInventory_Customer[i])});
				}
				else
				{
					tempDeliveryQuantity[i] = std::min({unmetDemandCustomers_lookAhead + tempDeliveryQuantity[i], params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[i] - Inv_Customers[i][t - 1])});
				}
			}
			else
			{
				setTwo.push_back(i);

				if (t == 0)
				{
					tempDeliveryQuantity[i] = std::min({unmetDemandCustomers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[i] - params.initialInventory_Customer[i])});
				}
				else
				{
					tempDeliveryQuantity[i] = std::min({unmetDemandCustomers_lookAhead, params.vehicleCapacity_Warehouse, (params.storageCapacity_Customer[i] - Inv_Customers[i][t - 1])});
				}
			}
		}
	}
}

void ConstructHeuristic::nearestNeighourInsertion(int s, int t, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &deliveryQuantity_Customers, vector<double> &tempDeliveryQuantity, vector<vector<vector<vector<int>>>> &routes, vector<vector<vector<int>>> &tempCATW)
{
	vector<double> remainingAvailInventoryWarehouse(params.numWarehouses);
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		remainingAvailInventoryWarehouse[w] = params.initialInventory_Warehouse[w];
		for (int l = 0; l <= t; ++l)
		{
			remainingAvailInventoryWarehouse[w] += solFE_init.deliveryQuantityToWarehouse[w][l];
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (tempCATW[l][w][i] == 1)
				{
					remainingAvailInventoryWarehouse[w] -= deliveryQuantity_Customers[i][l];
				}
			}
		}
		if (remainingAvailInventoryWarehouse[w] < 0.0)
		{
			remainingAvailInventoryWarehouse[w] = 0.0;
		}
	}

	vector<vector<double>> remainingVehicleCapacityWarehouse(params.numWarehouses, vector<double>(params.numVehicles_Warehouse, params.vehicleCapacity_Warehouse));
	
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		if (setOne.empty())
		{
			break;
		}
		// We now have a partial route and we wanna add Customers (from setOne) to it
		bool AllNodesVisited = false;
		int r = 0;
		while (r < params.numVehicles_Warehouse && !AllNodesVisited)
		{
			if (remainingAvailInventoryWarehouse[w] <= 0.0)
			{
				break;
			}
			if (remainingVehicleCapacityWarehouse[w][r] <= 0.0)
			{
				r++;
				continue;
			}

			// cout << "Vehicle " << r << endl;
			// Find closest customer to the current node
			int currentNode = w;
			while (remainingVehicleCapacityWarehouse[w][r] > 0.0 && remainingAvailInventoryWarehouse[w] > 0.0)
			{
				// cout << "Remaining Inventory: " << remainingAvailInventoryWarehouse << ", Remaining Vehicle Capacity: " << remainingVehicleCapacityWarehouse[r] << endl;
				int nextNodeToVisit = findNextNodeToVisit(currentNode, setOne, tempDeliveryQuantity, remainingVehicleCapacityWarehouse[w][r], remainingAvailInventoryWarehouse[w]);
				// cout << "Next Node to Visit: " << nextNodeToVisit << endl;
				if (nextNodeToVisit != -1)
				{
					if (routes[w][t][r].empty())
					{
						routes[w][t][r].push_back(w);
					}

					currentNode = nextNodeToVisit + params.numWarehouses;

					tempCATW[t][w][nextNodeToVisit] = 1;

					routes[w][t][r].push_back(currentNode);
					deliveryQuantity_Customers[nextNodeToVisit][t] = tempDeliveryQuantity[nextNodeToVisit];
					remainingAvailInventoryWarehouse[w] -= deliveryQuantity_Customers[nextNodeToVisit][t];
					remainingVehicleCapacityWarehouse[w][r] -= deliveryQuantity_Customers[nextNodeToVisit][t];

					auto it = std::find(setOne.begin(), setOne.end(), currentNode - params.numWarehouses);
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

	}

	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int r = 0; r < params.numVehicles_Warehouse; ++r)
		{
			if (!routes[w][t][r].empty() && routes[w][t][r].back() != w)
			{
				routes[w][t][r].push_back(w);
			}
		}
	}

	auto it = setTwo.begin();
	while (it != setTwo.end())
	{
		int i = *it;
		int custInd = i + params.numWarehouses;

		int warehouseToInsert = -1;
		int routeToInsert = -1;
		int posToInsert = -1;
		double minCostToInsert = std::numeric_limits<double>::max();

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			auto maxIt = std::max_element(remainingVehicleCapacityWarehouse[w].begin(), remainingVehicleCapacityWarehouse[w].end());
			if (*maxIt <= 0.0)
			{
				continue;
			}

			for (int r = 0; r < params.numVehicles_Warehouse; ++r)
			{
				// cout << "Route: " << r << endl;
				// cout << "Remaining Inventory (if visited): " << remainingAvailInventoryWarehouse - tempDeliveryQuantity[custInd] << ", Remaining Vehicle Capacity: " << remainingVehicleCapacityWarehouse[r] - tempDeliveryQuantity[custInd] << endl;
				if (remainingVehicleCapacityWarehouse[w][r] - tempDeliveryQuantity[i] >= 0.0 &&
					remainingAvailInventoryWarehouse[w] - tempDeliveryQuantity[i] >= 0.0)
				{
					// cout << "size of routesPeriod[r]: " << routesPeriod[r].size() << endl;
					if (routes[w][t][r].empty())
					{
						double costToInsert = 2 * params.transportationCost_SecondEchelon[w][custInd];
						if (costToInsert < minCostToInsert)
						{
							warehouseToInsert = w;
							posToInsert = 1;
							minCostToInsert = costToInsert;
							routeToInsert = r;
							// cout << "Possible Inserting in empty route " << routeToInsert << " at pos " << posToInsert << " with cost " << minCostToInsert << endl;
						}
					}
					else
					{
						std::pair<int, double> minInsertionCostResult = minInsertionCost(routes[w][t][r], custInd);

						double costToInsert = minInsertionCostResult.second;

						if (costToInsert < minCostToInsert)
						{
							warehouseToInsert = w;
							posToInsert = minInsertionCostResult.first;
							minCostToInsert = costToInsert;
							routeToInsert = r;
							// cout << "Possible Inserting in route " << routeToInsert << " at pos " << posToInsert << " with cost " << minCostToInsert << endl;
						}
					}
				}
			}
		}

		// cout << "Inserting " << i << " at pos " << posToInsert << " in route " << routeToInsert << endl;
		if (warehouseToInsert != -1 && routeToInsert != -1)
		{
			if (routes[warehouseToInsert][t][routeToInsert].empty())
			{
				tempCATW[t][warehouseToInsert][i] = 1;

				routes[warehouseToInsert][t][routeToInsert].push_back(warehouseToInsert);
				routes[warehouseToInsert][t][routeToInsert].push_back(custInd);
				routes[warehouseToInsert][t][routeToInsert].push_back(warehouseToInsert);
				deliveryQuantity_Customers[i][t] = tempDeliveryQuantity[i];
				remainingAvailInventoryWarehouse[warehouseToInsert] -= tempDeliveryQuantity[i];
				remainingVehicleCapacityWarehouse[warehouseToInsert][routeToInsert] -= tempDeliveryQuantity[i];
				it = setTwo.erase(it);

			}
			else
			{
				tempCATW[t][warehouseToInsert][i] = 1;

				routes[warehouseToInsert][t][routeToInsert].insert(routes[warehouseToInsert][t][routeToInsert].begin() + posToInsert, custInd);
				deliveryQuantity_Customers[i][t] = tempDeliveryQuantity[i];
				remainingAvailInventoryWarehouse[warehouseToInsert] -= tempDeliveryQuantity[i];
				remainingVehicleCapacityWarehouse[warehouseToInsert][routeToInsert] -= tempDeliveryQuantity[i];
				it = setTwo.erase(it);
			}
		}
		else
		{
			++it;
		}
	}

	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int r = 0; r < params.numVehicles_Warehouse; ++r)
		{
			if (!routes[w][t][r].empty() && routes[w][t][r].back() != w)
			{
				routes[w][t][r].push_back(w);
			}
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
		int custIndex = i + params.numWarehouses;
		double visitCost = params.transportationCost_SecondEchelon[current_node][custIndex];

		// cout << "visit cost: " << visitCost << endl;
		// cout << "Remaining Inventory (if visited): " << remainingAvailInventoryWarehouse - tempDeliveryQuantity[custIndex] << ", Remaining Vehicle Capacity: " << remainingVehicleCapacityWarehouse - tempDeliveryQuantity[custIndex] << endl;

		if (visitCost < minVisitCost && remainingVehicleCapacityWarehouse - tempDeliveryQuantity[i] >= 0.0 && remainingAvailInventoryWarehouse - tempDeliveryQuantity[i] >= 0.0)
		{
			minVisitCost = visitCost;
			nearestNode = i;
		}
	}

	return nearestNode;
}

std::pair<int, double> ConstructHeuristic::minInsertionCost(const vector<int> &route, int i)
{
	double minInsertionCost = std::numeric_limits<double>::max();
	int minInsertionPos = -1;

	for (int pos = 1; pos < route.size(); ++pos)
	{
		double insertionCost = params.transportationCost_SecondEchelon[route[pos - 1]][i] +
							   params.transportationCost_SecondEchelon[i][route[pos]] -
							   params.transportationCost_SecondEchelon[route[pos - 1]][route[pos]];
		if (insertionCost < minInsertionCost)
		{
			minInsertionCost = insertionCost;
			minInsertionPos = pos;
		}
	}

	return std::make_pair(minInsertionPos, minInsertionCost);
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

vector<vector<vector<vector<int>>>> ConstructHeuristic::getCustomerAssignmentToWarehouse()
{
	return customerAssignmentToWarehouse_bestSolution;
}

double ConstructHeuristic::getBestObjValue()
{
	return bestObjValue;
}

*/


