#include "ILS_SIRP.h"

ConstructHeuristic::ConstructHeuristic(const ParameterSetting &parameters, const Solution &solution)
	: params(parameters),
	  sol_FE(solution),
{
	// ----------------------------------------------------------------------------------------------------------
	RATW = params.getRetailersAssignedToWarehouse();
	assignedWarehouseToRetailer = params.getWarehouseAssignedToRetailer();

	// we calculate the customers per unit unemt demand penalty cost to approximate per unit demand satisfaction cost ratio
	// We order customers in descending order to have them from highest to lowest penalty cost ratio. (this shows the prioritun in demand satisfaction)
	sorted_customers_byPenaltyCostRatio(params.numWarehouses, vector<int>());
	orderCustomersByUnmetDemandToDeliveryRatio(sorted_customers_byPenaltyCostRatio);

	// Inv_Customers(params.numRetailers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	// unmetDemand_Customers(params.numRetailers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	// deliveryQuantity_Customers(params.numRetailers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	// routes(params.numScenarios, vector<vector<vector<vector<int>>>>(params.numPeriods, vector<vector<vector<int>>>(params.numWarehouses, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>()))));
}

void ConstructHeuristic::orderCustomersByUnmetDemandToDeliveryRatio(vector<vector<int>>& sorted_customer_costRatio)
{
	// calculate the stockout to demand satisfaction cost ratio using: Ratio = penaltyCost[i] / F/C + 2c[0][w] + 2c[w][i] + look_ahead * h[i]
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		vector<std::pair<int, double>> customer_costRatio;
		for (int i : RATW[w])
		{
			double costRatio = (params.undemtDemandPenaltyCost[i]) /
									((params.setupCost / params.productionCapacity) +
									(2 * params.TransportationCost_FirstEchelon[0][w + 1]) +
									(2 * transportationCost_SecondEchelon[w][i + params.numWarehouses]) +
									look_ahead * params.unitHoldingCost[i]);

			customer_costRatio.emplace_back(i + params.numWarehouses, costRatio);
		}

		std::sort(customer_costRatio.begin(), customer_costRatio.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b)
			{ return a.second > b.second; });

		for (const auto &pair : customer_costRatio)
		{
			sorted_customer_costRatio[w].push_back(pair.first);
		}
	}
}

void ConstructHeuristic::calculateDecisionVariables(int s, int w, vector<vector<double>>& Inv_Customers, vector<vector<double>>& unmetDemand_Customers, vector<vector<double>>& deliveryQuantity_Customers)
{	
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i : RATW[w])
		{
			if (t == 0)
			{
				Inv_Customers[i][t] = params.initialInventory_Retailer[i];
				Inv_Customers[i][t] -= params.demand[i][t][s];
				Inv_Customers[i][t] += deliveryQuantity_Customers[i][l];

				if (Inv_Customers[i][t] < 0)
				{
					unmetDemand_Customers[i][t] = -Inv_Customers[i][t];
					Inv_Customers[i][t] = 0.0;
				}
			}
			else
			{
				Inv_Customers[i][t] = Inv_Customers[i][t - 1];
				Inv_Customers[i][t] -= params.demand[i][t][s];
				Inv_Customers[i][t] += deliveryQuantity_Customers[i][l];

				if (Inv_Customers[i][t] < 0)
				{
					unmetDemand_Customers[i][t] = -Inv_Customers[i][t];
					Inv_Customers[i][t] = 0.0;
				}
			}
		}
	}
}

void ConstructHeuristic::defineSetOne(int s, int w, int t, vector<int>& setOne, const vector<vector<double>>& unmetDemand_Customers, vector<double>& tempDeliveryQuantity)
{
	for (int i : sorted_customers_byPenaltyCostRatio_CurWarehouse[w])
	{
		retIndex = i - params.numWarehouses;
		if (unmetDemand_Customers[retIndex][t] > 0)
		{
			setOne[t].push_back(i);

			if (t == 0)
			{
				tempDeliveryQuantity[retIndex] = std::min(params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailers[retIndex] - param.initialInventory_Retailer[retIndex]));
			}
			else if (t == params.numPeriods - 1)
			{
				tempDeliveryQuantity[retIndex] = std::min(params.vehicleCapacity_Warehouse, unmetDemand_Customers[retIndex][t]);
			}
			else
			{
				tempDeliveryQuantity[retIndex] = std::min(params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailers[iretIndex] - Inv_Customers[retIndex][t - 1]));
			}
		}
	}
}

void ConstructHeuristic::defineSetTwo(int s, int w, int t, int look_ahead, vector<int>& setOne, vector<int>& setTwo, const vector<vector<double>>& unmetDemand_Customers, vector<double>& tempDeliveryQuantity)
{
	for (int i : sorted_customers_byPenaltyCostRatio_CurWarehouse)
	{
		retIndex = i - params.numWarehouses;
		// check if customer i is not in setOne
        auto it = std::find(setOne.begin(), setOne.end(), i);
        if (it == setOne.end()) 
		{
			double unetDemandCustomers_lookAhead = 0.0;
			for (int l = t + 1; l <= t + look_ahead; ++l)
			{
				unetDemandCustomers_lookAhead += unmetDemand_Customers[retIndex][l];
			}
			
			if (unetDemandCustomers_lookAhead > 0)
			{
				setTwo[t].push_back(i);
				
				if (t == 0)
				{
					tempDeliveryQuantity[retIndex] = std::min(params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailers[retIndex] - param.initialInventory_Retailer[retIndex]));
				}
				else if (t == params.numPeriods - 1)
				{
					tempDeliveryQuantity[retIndex] = std::min(params.vehicleCapacity_Warehouse, unmetDemand_Customers[retIndex][t]);
				}
				else
				{
					tempDeliveryQuantity[retIndex] = std::min(params.vehicleCapacity_Warehouse, (params.storageCapacity_Retailers[iretIndex] - Inv_Customers[retIndex][t - 1]));
				}
			}
		}
	}
}

void ConstructHeuristic::nearestNeighourInsertion(int s, int w, int t, vector<int>& setOne, vector<int>& setTwo, vector<vector<double>>&  tempDeliveryQuantity, vector<vector<vector<int>>>& routesWarehousePeriod)
{
	int numRoutes = params.numVehicles_Warehouse;
	remainingAvailInventoryWarehouse = sol_FE.warehouseInventory[w][t][s];
	vector<double> remainingVehicleCapacityWarehouse(numRoutes, params.vehicleCapacity_Warehouse);
    
	// We now have a partial route and we wanna add customers (from setOne) to it
	bool AllNodesVisited = false;
	int r = 0;
	while (r < numRoutes && !AllNodesVisited)
	{
		// Find closest customer to the current node
		int currentNode = w;

		while (remainingVehicleCapacityWarehouse[r] > 0.0 && remainingAvailInventoryWarehouse > 0.0)
		{
			int nextNodeToVisit = findNextNodeToVisit(currentNode, setOne);
			if (nextNodeToVisit != -1) 
			{
				currentNode = nextNodeToVisit;
				retInd = currentNode - params.numWarehouses;

				routesWarehousePeriod[r].push_back(currentNode);
				deliveryQuantity_Customers[retInd][t] += tempDeliveryQuantity[retInd];
				remainingAvailInventoryWarehouse -= deliveryQuantity_Customers[retInd][t];
				remainingVehicleCapacityWarehouse[r] -= deliveryQuantity_Customers[retInd][t];

				auto it = std::find(setOne.begin(), setOne.end(), currentNode);
				if (it != setOne.end()) {
					setOne.erase(it);
				}
			}
			else
			{
				// No more suitable nodes to visit in this route
				r++;
				if (setOne.empty()) {
					AllNodesVisited = true;
				}
				break;
			}

		}
	}


	// if (!AllNodesVisited)
	// {
	// 	auto it = setOne.begin();
	// 	while (it != setOne.end()) {
	// 		int i = *it;
	// 		auto maxIt = std::max_element(remainingVehicleCapacityWarehouse.begin(), remainingVehicleCapacityWarehouse.end());
	// 		if (maxIt != remainingVehicleCapacityWarehouse.end()) {
	// 			if (*maxIt == 0.0) {
	// 				break;
	// 			}

	// 			int index_maxAvailability = std::distance(remainingVehicleCapacityWarehouse.begin(), maxIt);
				
	// 			routesWarehousePeriod[index_maxAvailability].push_back(i);
	// 			double temp = std::min({tempDeliveryQuantity[i], remainingVehicleCapacityWarehouse[index_maxAvailability], remainingAvailInventoryWarehouse});
	// 			deliveryQuantity_Customers[i][t] += temp;
	// 			remainingAvailInventoryWarehouse -= temp;
	// 			remainingVehicleCapacityWarehouse[index_maxAvailability] -= temp;

	// 			it = setOne.erase(it);

	// 			if (setOne.empty()) {
	// 				AllNodesVisited = true;
	// 				break; // Exit the loop as there are no more nodes to process
	// 			}
	// 		} else {
	// 			++it;
	// 		}
	// 	}
	// }
	
	auto maxIt = std::max_element(remainingVehicleCapacityWarehouse.begin(), remainingVehicleCapacityWarehouse.end());
	if (maxIt != remainingVehicleCapacityWarehouse.end() && *maxIt > 0.0)
	{
		auto it = setTwo.begin();
		while (it != setTwo.end()) {
			int i = *it;
			int retInd = i - params.numWarehouses;

			int routeToInsert = -1;
			int posToInsert = -1;
			double minCostToInsert = std::numeric_limits<double>::max();

			for (int r = 0; r < numRoutes; ++r) {
				if (remainingVehicleCapacityWarehouse[r] - tempDeliveryQuantity[retInd] >= 0.0 && 
					remainingAvailInventoryWarehouse - tempDeliveryQuantity[retInd] >= 0.0) 
				{
					std::pair<int, double> minInsertionCost = minInsertionCost(routesWarehousePeriod[r], i);
					
					int posToInsert = minInsertionResult.first;
        			double costToInsert = ;
					
					if (costToInsert < minCostToInsert) {
						posToInsert = minInsertionResult.first;
						minCostToInsert = minInsertionResult.second;
						routeToInsert = r;
					}
				}
			}

			if (routeToInsert != -1) {
				routesWarehousePeriod[routeToInsert].insert(routesWarehousePeriod[routeToInsert].begin() + posToInsert, i);
				deliveryQuantity_Customers[retInd][t] += tempDeliveryQuantity[retInd];
				remainingAvailInventoryWarehouse -= tempDeliveryQuantity[retInd];
				remainingVehicleCapacityWarehouse[routeToInsert] -= tempDeliveryQuantity[retInd];
				it = setTwo.erase(it);
			} else {
				++it;
			}
		}

	}
}

int ConstructHeuristic::findNextNodeToVisit(int current_node, vector<int>& setOne, vector<double> &tempDeliveryQuantity)
{
	double minVisitCost = std::numeric_limits<double>::max();
	int nearestNode = -1;

    // Loop through the first row to find the maximum value
	for (int i : setOne)
	{
		int retIndex = i - params.numWarehouses;
		double visitCost = params.transportationCost_SecondEchelon[current_node][i];
		if (visitCost < minVisitCost && remainingVehicleCapacityWarehouse - tempDeliveryQuantity[retIndex] >= 0.0 && remainingAvailInventoryWarehouse - tempDeliveryQuantity[retIndex] >= 0.0)
		{
			minVisitCost = visitCost;
			nearestNode = i;
		}
	}

	return nearestNode;
}

std::pair<int, double> ConstructHeuristic::minInsertionCost(const std::vector<int>& routesWarehousePeriod, int i) 
{
    double minInsertionCost = std::numeric_limits<double>::max();
    int minInsertionPos = -1;

    for (int pos = 1; pos < routesWarehousePeriod.size(); ++pos) {
        double insertionCost = params.transportationCost_SecondEchelon[routesWarehousePeriod[pos - 1]][i] +
                               params.transportationCost_SecondEchelon[i][routesWarehousePeriod[pos]] -
                               params.transportationCost_SecondEchelon[routesWarehousePeriod[pos - 1]][routesWarehousePeriod[pos]];
        if (insertionCost < minInsertionCost) {
            minInsertionCost = insertionCost;
            minInsertionPos = pos;
        }
    }

    return std::make_pair(minInsertionPos, minInsertionCost);
}

bool ConstructHeuristic::Construct_InitialSolution()
{
	try
	{
		for (int s = 0; s < params.numScenarios; ++s)
		{	
			// fix this:
			vector<vector<double>> Inv_Customers(params.numRetailers, vector<double>(params.numPeriods, 0.0));
			vector<vector<double>> unmetDemand_Customers(params.numRetailers, vector<double>(params.numPeriods, 0.0));;
			vector<vector<double>> deliveryQuantity_Customers(params.numRetailers, vector<double>(params.numPeriods, 0.0));;
			// vector<vector<vector<int>>> routesWarehousePeriod(params.numRetailers, vector<vector<int>>(params.numVehicles_Warehouse, vector<int()));

			for (int w = 0; w < params.numWarehouses; ++w)
			{
				int look_ahead = 1;
				while (look_ahead < std::ceil(params.numPeriods / 2))
				{
					calculateDecisionVariables(s, w, Inv_Customers, unmetDemand_Customers, deliveryQuantity_Customers);

					for (int t = 0; t < params.numPeriods; ++t)
					{
						vector<double> tempDeliveryQuantity(RATW[w].size(), 0.0);

						vector<int> setOne;
						defineSetOne(s, w, t, setOne, unmetDemand_Customers, tempDeliveryQuantity);

						vector<int> setTwo;
						defineSetTwo(s, w, t, look_ahead, setOnd, setTwo, unmetDemand_Customers, tempDeliveryQuantity);
						
						nearestNeighourInsertion(s, w, t, setOne, setTwo, mappedNodes, transportationCost_SecondEchelon_Warehouse, Inv_Customers, unmetDemand_Customers, tempDeliveryQuantity, routesWarehousePeriod);

						calculateDecisionVariables(s, Inv_Customers, unmetDemand_Customers, deliveryQuantity_Customers);

					}
				}
				update feasible solution
				++look_ahead;
			}
		}
	}
	catch (std::exception &e)
	{
		return false;
	}

	return true;
}