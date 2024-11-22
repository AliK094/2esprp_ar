#include "ConsInitSol_Deterministic.h"

ConstructHeuristic_Deterministic::ConstructHeuristic_Deterministic(const ParameterSetting &parameters, const SolutionFirstEchelon &sol_FE)
	: params(parameters),
	  solFE_init(sol_FE)
{
	// ----------------------------------------------------------------------------------------------------------
	cout << "Construct Heuristic" << endl;

	if (params.problemType == "EV" || params.problemType == "WS"  || params.problemType == "EEV")
		shortageAllowed = true;

	// We calculate the Customers per unit unemt demand penalty cost to approximate per unit demand satisfaction cost ratio
	// We order Customers in descending order to have them from highest to lowest penalty cost ratio. (this shows the prioritun in demand satisfaction)
	
	if (shortageAllowed)
	{
		sortedWarehouseByDistance = params.getSortedWarehousesByDistance();
		orderCustomersByUnmetDemandToDeliveryRatio(sorted_Customers_byPenaltyCostRatio);
	}
	else 
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			sorted_Customers_byPenaltyCostRatio.push_back(i + params.numWarehouses);
		}
	}

	CATW = params.getCustomersAssignedToWarehouse_det();

	bestObjValue = 0.0;
	Inv_Customers_bestSolution.resize(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	unmetDemand_Customers_bestSolution.resize(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	deliveryQuantity_Customers_bestSolution.resize(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	routes_bestSolution.resize(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>())));
}

void ConstructHeuristic_Deterministic::orderCustomersByUnmetDemandToDeliveryRatio(vector<int> &sorted_customer_costRatio)
{
	// calculate the stockout to demand satisfaction cost ratio using: Ratio = penaltyCost[i] / u + F/C + 2c[0][closest warehouse] + 2c[closest warehouse][i]
	vector<std::pair<int, double>> customer_costRatio;
	for (int i = 0; i < params.numCustomers; ++i)
	{
		int customerIndex = i + params.numWarehouses;
		double costRatio = (params.unmetDemandPenalty[i]) /
							(params.unitProdCost +
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

void ConstructHeuristic_Deterministic::calculateDecisionVariables(int w, vector<vector<double>> &Inv_Customers, vector<vector<double>> &unmetDemand_Customers, vector<vector<double>> &deliveryQuantity_Customers)
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[t][w][i] == 1)
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
				Inv_Customers[i][t] -= params.demand_Deterministic[i][t];
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

void ConstructHeuristic_Deterministic::defineSetOne(int w, int t, vector<int> &setOne, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity)
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

void ConstructHeuristic_Deterministic::defineSetTwo(int w, int t, int look_ahead, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, vector<double> &tempDeliveryQuantity)
{
	for (int i : sorted_Customers_byPenaltyCostRatio)
	{
		int custIndex = i - params.numWarehouses;

		double unetDemandCustomers_lookAhead = 0.0;
		for (int l = t + 1; l <= std::min(t + look_ahead, params.numPeriods - 1); ++l)
		{
			// cout << "look ahead params.demand_Deterministic [" << i << "] = " << unmetDemand_Customers[custIndex][l] << endl;
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

void ConstructHeuristic_Deterministic::nearestNeighourInsertion(int w, int t, vector<int> &setOne, vector<int> &setTwo, vector<vector<double>> &deliveryQuantity_Customers, vector<double> &tempDeliveryQuantity, vector<vector<int>> &routesPeriod)
{
	int numRoutes = params.numVehicles_Warehouse;

	double remainingAvailInventoryWarehouse = params.initialInventory_Warehouse[w];
	for (int l = 0; l <= t; ++l)
	{
		remainingAvailInventoryWarehouse += solFE_init.deliveryQuantityToWarehouse[w][l];
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[t][w][i] == 1)
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

int ConstructHeuristic_Deterministic::findNextNodeToVisit(int current_node, vector<int> &setOne, vector<double> &tempDeliveryQuantity, double remainingVehicleCapacityWarehouse, double remainingAvailInventoryWarehouse)
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

std::pair<int, double> ConstructHeuristic_Deterministic::minInsertionCost(const vector<int> &routesPeriod, int i)
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

double ConstructHeuristic_Deterministic::calculateObjFuncValue(int w, const vector<vector<double>> &Inv_Customers, const vector<vector<double>> &unmetDemand_Customers, const vector<vector<vector<int>>> &routesPeriod)
{
	double objFuncValue = 0.0;
	
	// Inventory Holding Cost and Unmet Demand Cost
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (CATW[t][w][i] == 1)
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

bool ConstructHeuristic_Deterministic::Construct_InitialSolution()
{
	auto elapsedTime_InitialSolution = 0.0;
	auto startTime_InitialSolution = std::chrono::high_resolution_clock::now();

	try
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
				
				calculateDecisionVariables(w, Inv_Customers, unmetDemand_Customers, deliveryQuantity_Customers);

				for (int t = 0; t < params.numPeriods; ++t)
				{
					vector<double> tempDeliveryQuantity(params.numCustomers, 0.0);
					
					vector<int> setOne;
					defineSetOne(w, t, setOne, Inv_Customers, unmetDemand_Customers, tempDeliveryQuantity);
					
					vector<int> setTwo;
					if (t < params.numPeriods - 1)
					{
						defineSetTwo(w, t, look_ahead, setOne, setTwo, Inv_Customers, unmetDemand_Customers, tempDeliveryQuantity);
					}
					

					// cout << "setOne Warehouse " << w << " Period " << t << ": [";
					// for (auto it = setOne.begin(); it != setOne.end(); ++it)
					// {
					// 	if (it != setOne.begin())
					// 	{
					// 		cout << ", ";
					// 	}
					// 	cout << *it << ": " << tempDeliveryQuantity[*it - params.numWarehouses];
					// }
					// cout << "]" << endl;

					// cout << "setTwo Warehouse " << w << " Period " << t << ": [";
					// for (auto it = setTwo.begin(); it != setTwo.end(); ++it)
					// {
					// 	if (it != setTwo.begin())
					// 	{
					// 		cout << ", ";
					// 	}
					// 	cout << *it << ": " << tempDeliveryQuantity[*it - params.numWarehouses];
					// }
					// cout << "]" << endl;

					nearestNeighourInsertion( w, t, setOne, setTwo, deliveryQuantity_Customers, tempDeliveryQuantity, routesPeriod[t]);

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

					calculateDecisionVariables(w, Inv_Customers, unmetDemand_Customers, deliveryQuantity_Customers);
				}

				// calculate objective cost of the second echelon for the current warehouse and scenario
				double objFuncValue_temp = calculateObjFuncValue(w, Inv_Customers, unmetDemand_Customers, routesPeriod);
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
							if (CATW[t][w][i] == 1)
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

			bestObjValue += bestObjValue_ScenarioWarehouse;

			// cout << "\nbestObjValue: " << bestObjValue << endl;

			for (int i = 0; i < params.numCustomers; ++i)
			{
				for (int t = 0; t < params.numPeriods; ++t)
				{
					Inv_Customers_bestSolution[i][t] = Inv_Customers_Scenario[i][t];
					unmetDemand_Customers_bestSolution[i][t] = unmetDemand_Customers_Scenario[i][t];
					deliveryQuantity_Customers_bestSolution[i][t] = deliveryQuantity_Customers_Scenario[i][t];
				}
			}

			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int t = 0; t < params.numPeriods; ++t)
				{
					for (int k = 0; k < params.numVehicles_Warehouse; ++k)
					{
						routes_bestSolution[w][t][k] = routes_Scenario[w][t][k];
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

vector<vector<double>> ConstructHeuristic_Deterministic::getInvCustomers()
{
	return Inv_Customers_bestSolution;
}

vector<vector<double>> ConstructHeuristic_Deterministic::getUnmetDemandCustomers()
{
	return unmetDemand_Customers_bestSolution;
}

vector<vector<double>> ConstructHeuristic_Deterministic::getDeliveryQuantityCustomers()
{
	return deliveryQuantity_Customers_bestSolution;
}

vector<vector<vector<vector<int>>>> ConstructHeuristic_Deterministic::getRoutesWarehouseToCustomer()
{
	return routes_bestSolution;
}

double ConstructHeuristic_Deterministic::getBestObjValue()
{
	return bestObjValue;
}
