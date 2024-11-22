#include "LocalSearch_Deterministic.h"

LocalSearch_Deterministic::LocalSearch_Deterministic(const ParameterSetting &parameters, 
													 const SolutionFirstEchelon &sol_FE, 
													 const SolutionSecondEchelon_Deterministic &sol_SE, 
													 const double objVal)
	: params(parameters), 
	  sol_FE_temp(sol_FE), 
	  sol_SE_temp(sol_SE), 
	  objVal(objVal), 
	  rng(std::random_device{}())
{
	// ----------------------------------------------------------------------------------------------------------
	// cout << "\nLocal Search for " << endl;
	// ----------------------------------------------------------------------------------------------------------
}

void LocalSearch_Deterministic::RVND()
{
	// cout << "\nCurrent Objective Value : " << objVal << endl;

	// Initialize the operators
	vector<std::function<bool()>> operators;

	int maxIterRVND = 10;
	int iter = 0;

	// Uniform distribution for operator selection
    std::uniform_int_distribution<int> operator_dist;

	sol_FE_best = sol_FE_temp;
	sol_SE_best = sol_SE_temp;
	objVal_best = objVal;

	// We can make it a multistart RVND by applying maxIterRVND times
	for (int i = 0; i < maxIterRVND; ++i)
	{
		// cout << "\nRVND Iteration : " << iter + 1 << endl;
		operators = setOperators();

		sol_FE_best_currStart = sol_FE_temp;
		sol_SE_best_currStart = sol_SE_temp;
		objVal_best_currStart = objVal;

		while (!operators.empty())
		{
			sol_SE_feasible = sol_SE_best_currStart;
			objVal_feasible = objVal_best_currStart;

			// printAllRoutes();

			// Generate random number between 0 and operators.size()
			operator_dist = std::uniform_int_distribution<int>(0, static_cast<int>(operators.size()) - 1);
			int operatorIndex = operator_dist(rng);
			auto currentOperator = operators[operatorIndex];

			// Execute the operator
			bool success = currentOperator();
			if (!success)
			{
				// Operator did not find an improvement; remove it from the list
				operators.erase(operators.begin() + operatorIndex);
				continue;
			}
			operators = setOperators();
		}

		if (objVal_best_currStart < objVal_best - 0.001)
		{
			sol_FE_best = sol_FE_best_currStart;
			sol_SE_best = sol_SE_best_currStart;
			objVal_best = objVal_best_currStart;
		}
		++iter;
	}

	// printAllRoutes();
	// cout << "\nBest Objective Value : " << objVal_best << endl;
}

vector<std::function<bool()>> LocalSearch_Deterministic::setOperators()
{
	return {
		[this]()
		{ return OrOpt(1); },
		[this]()
		{ return OrOpt(2); },
		[this]()
		{ return OrOpt(3); },
		[this]()
		{ return Shift(1); },
		[this]()
		{ return Shift(2); },
		[this]()
		{ return Shift(3); },
		[this]()
		{ return Swap(1, 1); },
		[this]()
		{ return Swap(1, 2); },
		[this]()
		{ return Swap(2, 2); },
		[this]()
		{ return Insert(); },
		[this]()
		{ return Remove(); },
		[this]()
		{ return Merge(); },
		[this]()
		{ return Transfer(); },
		[this]()
		{ return Remove_Insert(); }
	};
}

// Or-Opt Operator
bool LocalSearch_Deterministic::OrOpt(int v)
{
	// cout << "\nOr-Opt(" << v << ")" << endl;

	// Select a random warehouse
	std::uniform_int_distribution<int> warehouse_dist(0, static_cast<int>(params.numWarehouses) - 1);
	int warehouse = warehouse_dist(rng);

	// Generate all possible (t, k) combinations
	vector<std::pair<int, int>> allCombinations;
	allCombinations.reserve(params.numPeriods * params.numVehicles_Warehouse);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		{
			allCombinations.emplace_back(t, k);
		}
	}

	// Shuffle the combinations to ensure random selection
	std::shuffle(allCombinations.begin(), allCombinations.end(), rng);

	bool routeFound = false;
	int selectedPeriod = -1;
	int selectedVehicle = -1;
	vector<int> selectedRoute;

	// Iterate through shuffled combinations to find a suitable route
	for (const auto &[t, k] : allCombinations)
	{
		// Check if indices are within bounds
		if (warehouse >= sol_SE_feasible.routesWarehouseToCustomer.size() ||
			t >= sol_SE_feasible.routesWarehouseToCustomer[warehouse].size() ||
			k >= sol_SE_feasible.routesWarehouseToCustomer[warehouse][t].size())
		{
			continue; // Skip invalid combinations
		}

		const auto &route = sol_SE_feasible.routesWarehouseToCustomer[warehouse][t][k];

		if (route.size() >= static_cast<size_t>(v + 3))
		{
			// printRoute(warehouse, t, k, route, "Old Route");

			routeFound = true;
			selectedPeriod = t;
			selectedVehicle = k;
			selectedRoute = route;
			break;
		}
	}

	if (!routeFound)
	{
		// cerr << "Could not find a route with " << v + 3 << " nodes" << endl;
		return false;
	}

	// Define distributions based on the route size
	std::uniform_int_distribution<int> start_pos_dist(1, static_cast<int>(selectedRoute.size()) - v - 1);
	std::uniform_int_distribution<int> new_pos_dist(1, static_cast<int>(selectedRoute.size()) - v - 1);

	// Select a random starting position for the segment to move
	int startPos = start_pos_dist(rng);

	// Select a new position different from the starting position
	int newPos;
	do
	{
		newPos = new_pos_dist(rng);
	} while (newPos == startPos);

	// Move the segment of v customers from startPos to newPos
	vector<int> segment(selectedRoute.begin() + startPos, selectedRoute.begin() + startPos + v);
	selectedRoute.erase(selectedRoute.begin() + startPos, selectedRoute.begin() + startPos + v);
	selectedRoute.insert(selectedRoute.begin() + newPos, segment.begin(), segment.end());

	// cout << "Moved " << v << " customer(s) from position " << startPos
	// 	 << " to position " << newPos << " in route " << selectedVehicle + 1 << endl;

	// Update the route in the feasible solution
	sol_SE_feasible.routesWarehouseToCustomer[warehouse][selectedPeriod][selectedVehicle] = selectedRoute;

	// Print the updated route
	// printRoute(warehouse, selectedPeriod, selectedVehicle, selectedRoute, "Updated Route");

	// Attempt to solve the LP with the updated route
	if (!solveLP())
	{
		return false;
	}

	return true;
}

// Shift Operator
bool LocalSearch_Deterministic::Shift(int v)
{
	// cout << "\nShift(" << v << ")" << endl;

	// Collect all eligible periods
	vector<int> periods(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
		periods[t] = t;
	std::shuffle(periods.begin(), periods.end(), rng); // Shuffle to introduce randomness

	// Create random warehouse distributions for source and destination warehouses
	std::uniform_int_distribution<int> warehouse_dist(0, static_cast<int>(params.numWarehouses) - 1);

	for (const int t : periods)
	{
		// Select a random source warehouse
		int sourceWarehouse = warehouse_dist(rng);

		// Collect all eligible source routes for the current period in the selected source warehouse
		vector<int> sourceVehicles;
		for (int sourceVehicle = 0; sourceVehicle < params.numVehicles_Warehouse; ++sourceVehicle)
		{
			vector<int> &sourceRoute = sol_SE_feasible.routesWarehouseToCustomer[sourceWarehouse][t][sourceVehicle];

			// Check if the source route has enough customers to shift (assuming depots at both ends)
			if (sourceRoute.size() >= static_cast<size_t>(v + 2)) // 2 depots + v customers
			{
				sourceVehicles.push_back(sourceVehicle);
			}
		}

		if (sourceVehicles.empty())
			continue; // No eligible source routes in this period

		// Shuffle source vehicles to introduce randomness in selection
		std::shuffle(sourceVehicles.begin(), sourceVehicles.end(), rng);

		for (const int sourceVehicle : sourceVehicles)
		{
			vector<int> &sourceRoute = sol_SE_feasible.routesWarehouseToCustomer[sourceWarehouse][t][sourceVehicle];

			// Select a random destination warehouse (could be the same as the source warehouse)
			int destWarehouse = warehouse_dist(rng);

			// Collect all eligible destination routes (excluding the source route)
			vector<int> destVehicles;
			for (int destVehicle = 0; destVehicle < params.numVehicles_Warehouse; ++destVehicle)
			{
				if (destWarehouse == sourceWarehouse && destVehicle == sourceVehicle)
					continue; // Skip same route

				vector<int> &destRoute = sol_SE_feasible.routesWarehouseToCustomer[destWarehouse][t][destVehicle];

				// No specific size constraints for destination routes, but can add if needed
				destVehicles.push_back(destVehicle);
			}

			if (destVehicles.empty())
				continue; // No eligible destination routes

			// Shuffle destination vehicles to introduce randomness in selection
			std::shuffle(destVehicles.begin(), destVehicles.end(), rng);

			for (const int destVehicle : destVehicles)
			{
				vector<int> &destRoute = sol_SE_feasible.routesWarehouseToCustomer[destWarehouse][t][destVehicle];

				if (sourceRoute.size() == static_cast<size_t>(v + 2) && destRoute.size() == static_cast<size_t>(v + 2) 
					&& sourceWarehouse == destWarehouse)
					continue; // No shifting required

				// Collect all eligible start positions in the source route
				vector<int> eligibleStartPositions;
				for (int startPos = 1; startPos <= static_cast<int>(sourceRoute.size()) - v - 1; ++startPos)
				{
					eligibleStartPositions.push_back(startPos);
				}

				if (eligibleStartPositions.empty())
					continue; // No eligible start positions

				// Shuffle start positions to introduce randomness in selection
				std::shuffle(eligibleStartPositions.begin(), eligibleStartPositions.end(), rng);

				for (const int startPos : eligibleStartPositions)
				{
					int endPos = startPos + v;

					// Extract the segment to shift
					vector<int> segment(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);

					// Determine all possible insertion positions in the destination route
					vector<int> eligibleInsertPositions;
					for (int insertPos = 1; insertPos <= static_cast<int>(destRoute.size()) - 1; ++insertPos)
					{
						eligibleInsertPositions.push_back(insertPos);
					}

					if (eligibleInsertPositions.empty())
					{
						// If destination route has only depots, insert between them
						eligibleInsertPositions.push_back(0);
					}

					// Shuffle insertion positions to introduce randomness
					std::shuffle(eligibleInsertPositions.begin(), eligibleInsertPositions.end(), rng);

					for (const int insertPos : eligibleInsertPositions)
					{
						// Print old routes
						// printRoute(sourceWarehouse, t, sourceVehicle, sourceRoute, "Original Source Route");
						// printRoute(destWarehouse, t, destVehicle, destRoute, "Original Dest Route");

						// Perform the shift: remove from source and insert into destination
						sourceRoute.erase(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
						destRoute.insert(destRoute.begin() + insertPos, segment.begin(), segment.end());

						// Update the customer assignment matrix: change the warehouse assignment for each customer
						for (const int customer : segment)
						{
							// Set previous warehouse assignment to 0
							sol_SE_feasible.customerAssignmentToWarehouse[t][sourceWarehouse][customer - params.numWarehouses] = 0;

							// Set new warehouse assignment to 1
							sol_SE_feasible.customerAssignmentToWarehouse[t][destWarehouse][customer - params.numWarehouses] = 1;
						}

						// If sourceRoute has only depots left, clear it
						if (sourceRoute.size() == 2) // Assuming depots are at positions 0 and 1
						{
							sourceRoute.clear();
						}

						// If destRoute had only depots before and now has customers, ensure depots are correctly placed
						if (destRoute.size() < v + 2) // v customers + 2 depots
						{
							// Ensure depots are at the start and end
							if (destRoute.front() != destWarehouse)
							{
								destRoute.insert(destRoute.begin(), destWarehouse);
							}
							if (destRoute.back() != destWarehouse)
							{
								destRoute.push_back(destWarehouse);
							}
						}

						// Log the shift operation
						// cout << "Shifted " << v << " customer(s) from warehouse " << sourceWarehouse + 1 
						// 	 << " route " << sourceVehicle + 1 << " to warehouse " << destWarehouse + 1 
						// 	 << " route " << destVehicle + 1 << " in period " << t + 1 << endl;

						// Print updated routes
						// printRoute(sourceWarehouse, t, sourceVehicle, sourceRoute, "Updated Source Route");
						// printRoute(destWarehouse, t, destVehicle, destRoute, "Updated Dest Route");

						// Attempt to solve the LP with the updated routes
						if (solveLP())
						{
							return true;
						} 
						else
						{
							return false;
						}
					} // End of insertPos loop
				} // End of startPos loop
			} // End of destVehicle loop
		} // End of sourceVehicle loop
	} // End of period loop

	// If no valid shift found that improves the solution
	// cerr << "No valid shift found that improves the solution." << endl;
	return false;
}

// Swap Operator
bool LocalSearch_Deterministic::Swap(int v1, int v2)
{
	// cout << "\nSwap(" << v1 << ", " << v2 << ")" << endl;

	// Collect all eligible periods
	vector<int> periods(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
		periods[t] = t;
	std::shuffle(periods.begin(), periods.end(), rng); // Shuffle to introduce randomness

	// Create random warehouse distributions for source and destination warehouses
	std::uniform_int_distribution<int> warehouse_dist(0, static_cast<int>(params.numWarehouses) - 1);

	for (const int t : periods)
	{
		// Select a random source warehouse
		int sourceWarehouse = warehouse_dist(rng);

		// Collect all eligible source routes for the current period in the selected source warehouse
		vector<int> sourceVehicles;
		for (int sourceVehicle = 0; sourceVehicle < params.numVehicles_Warehouse; ++sourceVehicle)
		{
			vector<int> &sourceRoute = sol_SE_feasible.routesWarehouseToCustomer[sourceWarehouse][t][sourceVehicle];

			// Check if the source route has enough customers to perform a swap
			// Assuming depots are at positions 0 and sourceRoute.size() - 1
			if (sourceRoute.size() >= static_cast<size_t>(v1 + 2)) // At least v customer + 2 depots
			{
				sourceVehicles.push_back(sourceVehicle);
			}
		}

		if (sourceVehicles.empty())
			continue; // No eligible source routes in this period

		// Shuffle source vehicles to introduce randomness in selection
		std::shuffle(sourceVehicles.begin(), sourceVehicles.end(), rng);

		for (const int sourceVehicle : sourceVehicles)
		{
			vector<int> &sourceRoute = sol_SE_feasible.routesWarehouseToCustomer[sourceWarehouse][t][sourceVehicle];

			// Select a random destination warehouse (could be the same as the source warehouse)
			int destWarehouse = warehouse_dist(rng);

			// Collect all eligible destination routes in the destination warehouse
			vector<int> destVehicles;
			for (int destVehicle = 0; destVehicle < params.numVehicles_Warehouse; ++destVehicle)
			{
				// Ensure the destination route is not the same as the source route within the same warehouse
				if (sourceWarehouse == destWarehouse && sourceVehicle == destVehicle)
					continue; // Skip if source and destination routes are the same

				vector<int> &destRoute = sol_SE_feasible.routesWarehouseToCustomer[destWarehouse][t][destVehicle];

				// Check if the destination route has enough customers to perform a swap
				if (destRoute.size() >= static_cast<size_t>(v2 + 2)) // At least 1 customer + 2 depots
				{
					destVehicles.push_back(destVehicle);
				}
			}

			if (destVehicles.empty())
				continue; // No eligible destination routes

			// Shuffle destination vehicles to introduce randomness in selection
			std::shuffle(destVehicles.begin(), destVehicles.end(), rng);

			for (const int destVehicle : destVehicles)
			{
				vector<int> &destRoute = sol_SE_feasible.routesWarehouseToCustomer[destWarehouse][t][destVehicle];

				if (sourceRoute.size() == static_cast<size_t>(v1 + 2) && destRoute.size() == static_cast<size_t>(v2 + 2)
					&& sourceWarehouse == destWarehouse)
				{
					continue; // No swap required
				}

				// Print original routes
				// printRoute(sourceWarehouse, t, sourceVehicle, sourceRoute, "Original Source route");
				// printRoute(destWarehouse, t, destVehicle, destRoute, "Original Dest route");

				// Define distributions for selecting swap positions
				std::uniform_int_distribution<int> source_pos_dist(1, static_cast<int>(sourceRoute.size()) - v1 - 1);
				int sourcePos = source_pos_dist(rng);
				int sourceEndPos = sourcePos + v1;

				std::uniform_int_distribution<int> dest_pos_dist(1, static_cast<int>(destRoute.size()) - v2 - 1);
				int destPos = dest_pos_dist(rng);
				int destEndPos = destPos + v2;

				// Extract segments to swap
				vector<int> sourceSegment(sourceRoute.begin() + sourcePos, sourceRoute.begin() + sourceEndPos);
				vector<int> destSegment(destRoute.begin() + destPos, destRoute.begin() + destEndPos);

				// Perform the swap
				sourceRoute.erase(sourceRoute.begin() + sourcePos, sourceRoute.begin() + sourceEndPos);
				destRoute.erase(destRoute.begin() + destPos, destRoute.begin() + destEndPos);

				sourceRoute.insert(sourceRoute.begin() + sourcePos, destSegment.begin(), destSegment.end());
				destRoute.insert(destRoute.begin() + destPos, sourceSegment.begin(), sourceSegment.end());

				// Update the customer assignment matrix for swapped customers
				for (const int customer : sourceSegment)
				{
					// Set previous warehouse assignment to 0
					sol_SE_feasible.customerAssignmentToWarehouse[t][sourceWarehouse][customer - params.numWarehouses] = 0;

					// Set new warehouse assignment to 1
					sol_SE_feasible.customerAssignmentToWarehouse[t][destWarehouse][customer - params.numWarehouses] = 1;
				}

				for (const int customer : destSegment)
				{
					// Set previous warehouse assignment to 0
					sol_SE_feasible.customerAssignmentToWarehouse[t][destWarehouse][customer - params.numWarehouses] = 0;

					// Set new warehouse assignment to 1
					sol_SE_feasible.customerAssignmentToWarehouse[t][sourceWarehouse][customer - params.numWarehouses] = 1;
				}

				// Log the swap operation
				// cout << "Swapped " << v1 << " customer(s) from warehouse " << sourceWarehouse + 1
				// 		<< " route " << sourceVehicle + 1 << " with " << v2 << " customer(s) from warehouse " << destWarehouse + 1
				// 		<< " route " << destVehicle + 1 << " in period " << t + 1 << endl;

				// Print updated routes
				// printRoute(sourceWarehouse, t, sourceVehicle, sourceRoute, "Updated Source route");
				// printRoute(destWarehouse, t, destVehicle, destRoute, "Updated Dest route");

				// Attempt to solve the LP with the updated routes
				if (solveLP())
				{
					// Successful swap leading to improvement
					return true;
				}
				else
				{
					return false;
				}
			} // End of destVehicle loop
		} // End of sourceVehicle loop
	} // End of period loop

	// If no valid swap found that improves the solution
	// cerr << "Failed to find a valid swap that improves the solution." << endl;
	return false;
}

// Insert Operator
bool LocalSearch_Deterministic::Insert()
{
	// cout << "\nInsert" << endl;

	// Collect and shuffle periods
	vector<int> periods(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
		periods[t] = t;
	std::shuffle(periods.begin(), periods.end(), rng);

	// Randomly choose a warehouse
	std::uniform_int_distribution<int> warehouse_dist(0, static_cast<int>(params.numWarehouses) - 1);
	int selectedWarehouse = warehouse_dist(rng);

	vector<std::pair<int, int>> visitedNodesWithWarehouse;	 // Stores <node, warehouse>
	vector<std::pair<int, int>> unvisitedNodesWithWarehouse; // Stores <node, assigned warehouse> for unvisited nodes

	// Lambda function to check if a node is in visitedNodesWithWarehouse
	auto isNodeVisited = [&](int node)
	{
		return std::any_of(visitedNodesWithWarehouse.begin(), visitedNodesWithWarehouse.end(),
						   [&](const std::pair<int, int> &p)
						   { return p.first == node; });
	};

	// Iterate through shuffled periods
	for (const int t : periods)
	{
		visitedNodesWithWarehouse.clear(); // Clear visited nodes for each period

		// Loop through all warehouses to find visited nodes for the current period
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			int rtIndex = 0;
			for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[w][t])
			{
				if (route.size() > 2)
				{
					for (int nodeInd = 1; nodeInd < route.size() - 1; ++nodeInd)
					{
						int node = route[nodeInd];
						visitedNodesWithWarehouse.emplace_back(node, w); // Store the node with its warehouse
					}
				}

				// Output the route details
				// printRoute(w, t, rtIndex, route, "Old Route");
				rtIndex++;
			}
		}

		// Identify unvisited nodes for the selected period
		unvisitedNodesWithWarehouse.clear(); // Clear unvisited nodes for each period
		for (int i = 0; i < params.numCustomers; ++i)
		{
			int node = i + params.numWarehouses;
			if (!isNodeVisited(node))
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					if (sol_SE_feasible.customerAssignmentToWarehouse[t][w][i] == 1)
					{
						unvisitedNodesWithWarehouse.emplace_back(node, w); // Store the unvisited node with its warehouse
						break;
					}
				}
			}
		}

		// If no unvisited nodes are available in this period, continue to the next period
		if (unvisitedNodesWithWarehouse.empty())
		{
			continue;
		}

		// Randomly select a node from the unvisited nodes
		auto randomPair = unvisitedNodesWithWarehouse[rand() % unvisitedNodesWithWarehouse.size()];
		int unvisitedNode = randomPair.first;	   // The node part of the selected pair
		int assignedWarehouse = randomPair.second; // The warehouse part of the selected pair

		// Attempt to insert the unvisited node at the minimum cost position
		bool inserted = false;
		double minInsertionCost = std::numeric_limits<double>::max();
		int minInsertionPos = -1;
		int routeToIns = -1;
		int routeInd = 0;

		// Iterate over all routes in the selected warehouse to find the best insertion point
		for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t])
		{
			if (route.empty())
			{
				// For an empty route, calculate the cost of inserting the node between two warehouse nodes
				double insertionCost = 2 * params.transportationCost_SecondEchelon[selectedWarehouse][unvisitedNode];
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
		if (routeToIns != -1 && !sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t][routeToIns].empty())
		{
			// Insert node into the non-empty route
			sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t][routeToIns].insert(sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t][routeToIns].begin() + minInsertionPos, unvisitedNode);
			inserted = true;
		}
		else if (routeToIns != -1 && sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t][routeToIns].empty())
		{
			// If the route is empty, add warehouse -> node -> warehouse
			sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t][routeToIns] = {selectedWarehouse, unvisitedNode, selectedWarehouse};
			inserted = true;
		}

		if (!inserted)
		{
			// cout << "Failed to insert unvisited node." << endl;
			return false;
		}

		// Update the customerAssignmentToWarehouse matrix
		if (assignedWarehouse != selectedWarehouse)
		{
			sol_SE_feasible.customerAssignmentToWarehouse[t][assignedWarehouse][unvisitedNode - params.numWarehouses] = 0;
			sol_SE_feasible.customerAssignmentToWarehouse[t][selectedWarehouse][unvisitedNode - params.numWarehouses] = 1;
		}

		// Output updated routes for the selected warehouse and period
		int rtIndex = 0;
		for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t])
		{
			// printRoute(selectedWarehouse, t, rtIndex, route, "Updated Route");
			rtIndex++;
		}

		// cout << "Inserted unvisited node " << unvisitedNode << " at period " << t + 1 << " in warehouse " << selectedWarehouse + 1 << " with minimum cost." << endl;

		// Solve the linear program to check for improvement
		if (!solveLP())
		{
			// The LP did not find an improvement
			return false;
		}

		// Return true if an insertion was successfully made
		return true;
	}

	// If no insertion was made after checking all periods
	// cout << "No suitable insertion found in any period." << endl;
	return false;
}

// Remove Operator
bool LocalSearch_Deterministic::Remove()
{
	// cout << "\nRemove" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period
	
	// Randomly choose a warehouse
	std::uniform_int_distribution<int> warehouse_dist(0, static_cast<int>(params.numWarehouses) - 1);
	int selectedWarehouse = warehouse_dist(rng);

	vector<int> candidateRoutes;

	// int rtIndex = 0;
	// for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t])
	// {
	// 	printRoute(selectedWarehouse, t, rtIndex, route, "Old Route");

	// 	rtIndex++;
	// }

	// Gather routes that have more than just the warehouse start and end node
	for (int r = 0; r < sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t].size(); ++r)
	{
		if (sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t][r].size() > 2)
		{ // Ensure there's more than just the warehouse nodes
			candidateRoutes.push_back(r);
		}
	}

	// If no suitable routes are found, return false
	if (candidateRoutes.empty())
	{
		// cout << "No suitable routes with removable nodes found." << endl;
		return false;
	}

	// Randomly select a route from candidate routes
	int routeIndex = candidateRoutes[rand() % candidateRoutes.size()];
	auto &selectedRoute = sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t][routeIndex];

	// Randomly select a customer node to remove, avoiding the warehouse nodes at the start and end if present
	int nodeIndex = rand() % (selectedRoute.size() - 2) + 1; // Avoid first and last positions if they are warehouses
	selectedRoute.erase(selectedRoute.begin() + nodeIndex);
	int removedCustomer = selectedRoute[nodeIndex];
	if (selectedRoute.size() <= 2)
	{
		// Clear the route since it only contains the warehouse start and end nodes
		selectedRoute.clear();
	}

	// rtIndex = 0;
	// for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t])
	// {
	// 	printRoute(selectedWarehouse, t, rtIndex, route, "New Route");

	// 	rtIndex++;
	// }

	// Log the operation
	// cout << "Removed customer " << removedCustomer << " from period " << t + 1 
	// 	 << ", warehouse " << selectedWarehouse + 1 << ", route " << routeIndex + 1 << endl;

	if (!solveLP())
	{
		// Successful swap leading to improvement
		return false;
	}

	return true;
}

// Merge Operator
bool LocalSearch_Deterministic::Merge()
{
	// cout << "\nMerge" << endl;

	// Shuffle periods for random selection
	vector<int> periods(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
		periods[t] = t;
	std::shuffle(periods.begin(), periods.end(), rng);

	// Define customerPeriodsWarehouses to store a vector of (period, warehouse) pairs for each customer
	std::unordered_map<int, vector<std::pair<int, int>>> customerPeriodsWarehouses; // Map: customer -> [(period, warehouse)]

	// Gather customers and the periods/warehouses they appear in
	for (int t : periods)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			std::unordered_set<int> periodCustomers; // Temporarily store unique customers for this period and warehouse

			for (const auto &route : sol_SE_feasible.routesWarehouseToCustomer[w][t])
			{
				if (route.size() > 2)
				{
					for (int nodeInd = 1; nodeInd < route.size() - 1; ++nodeInd)
					{
						int customer = route[nodeInd];
						periodCustomers.insert(customer); // Insert unique customers in the period
					}
				}
			}

			// Add customers with their period and warehouse to the map
			for (int customer : periodCustomers)
			{
				customerPeriodsWarehouses[customer].emplace_back(t, w); // Store the period (t) and warehouse (w) for each customer
			}
		}
	}

	// Find customers that appear in more than one period
	vector<int> eligibleCustomers;
	for (const auto &entry : customerPeriodsWarehouses)
	{
		std::unordered_set<int> uniquePeriods;
		for (const auto &periodWarehouse : entry.second)
		{
			uniquePeriods.insert(periodWarehouse.first); // Collect unique periods for this customer
		}

		if (uniquePeriods.size() > 1)
		{
			eligibleCustomers.push_back(entry.first); // If a customer appears in more than one period, add to eligibleCustomers
		}
	}

	if (eligibleCustomers.empty())
	{
		// cout << "No eligible customers found for merging." << endl;
		return false;
	}

	// Select a random customer to merge
	int selectedCustomer = eligibleCustomers[rand() % eligibleCustomers.size()];
	const auto &periodsAndWarehouses = customerPeriodsWarehouses[selectedCustomer]; // Get vector of (period, warehouse)

	// Collect all unique periods the customer appears in, ignoring warehouses
	std::unordered_set<int> uniquePeriods;
	for (const auto &periodWarehouse : periodsAndWarehouses)
	{
		uniquePeriods.insert(periodWarehouse.first); // Only store the period, ignore the warehouse
	}

	// Convert the set of unique periods to a vector for easy access
	vector<int> uniquePeriodsVec(uniquePeriods.begin(), uniquePeriods.end());

	// Ensure the customer appears in at least two periods
	if (uniquePeriodsVec.size() < 2)
	{
		// cout << "Error: Customer does not appear in two different periods." << endl;
		return false;
	}

	// Randomly select two different periods from the unique periods
	int mergeFromPeriod = uniquePeriodsVec[rand() % uniquePeriodsVec.size()];
	int mergeToPeriod;
	do
	{
		mergeToPeriod = uniquePeriodsVec[rand() % uniquePeriodsVec.size()];
	} while (mergeToPeriod == mergeFromPeriod);

	int mergeFromRouteIndex = -1;
	int mergeToRouteIndex = -1;
	int mergeFromWarehouse = -1;
	int mergeToWarehouse = -1;

	// Find the route index and warehouse in mergeFrom period containing the selected customer
	for (const auto &periodWarehouse : periodsAndWarehouses)
	{
		if (periodWarehouse.first == mergeFromPeriod)
		{
			mergeFromWarehouse = periodWarehouse.second; // Get the warehouse for mergeFrom
			for (int r = 0; r < sol_SE_feasible.routesWarehouseToCustomer[mergeFromWarehouse][mergeFromPeriod].size(); ++r)
			{
				vector<int>& route = sol_SE_feasible.routesWarehouseToCustomer[mergeFromWarehouse][mergeFromPeriod][r];
				auto it = std::find(route.begin() + 1,
									route.end() - 1,
									selectedCustomer);
				if (it != route.end())
				{
					mergeFromRouteIndex = r;
					break;
				}
			}
			break; // Found the period, no need to search further
		}
	}

	if (mergeFromRouteIndex == -1 || mergeFromWarehouse == -1)
	{
		// cout << "Error: Could not find selected customer in mergeFrom period." << endl;
		return false;
	}

	// Find the route index and warehouse in mergeTo period containing the selected customer
	for (const auto &periodWarehouse : periodsAndWarehouses)
	{
		if (periodWarehouse.first == mergeToPeriod)
		{
			mergeToWarehouse = periodWarehouse.second; // Get the warehouse for mergeTo
			for (int r = 0; r < sol_SE_feasible.routesWarehouseToCustomer[mergeToWarehouse][mergeToPeriod].size(); ++r)
			{
				vector<int>& route = sol_SE_feasible.routesWarehouseToCustomer[mergeToWarehouse][mergeToPeriod][r];
				auto it = std::find(route.begin() + 1,
									route.end() - 1,
									selectedCustomer);
				if (it != route.end())
				{
					mergeToRouteIndex = r;
					break;
				}
			}
			break; // Found the period, no need to search further
		}
	}

	if (mergeToRouteIndex == -1 || mergeToWarehouse == -1)
	{
		// cout << "Error: Could not find selected customer in mergeTo period." << endl;
		return false;
	}

	// Print routes before merge
	// printRoute(mergeFromWarehouse, mergeFromPeriod, mergeFromRouteIndex, sol_SE_feasible.routesWarehouseToCustomer[mergeFromWarehouse][mergeFromPeriod][mergeFromRouteIndex], "Old Route (Merge From)");
	// printRoute(mergeToWarehouse, mergeToPeriod, mergeToRouteIndex, sol_SE_feasible.routesWarehouseToCustomer[mergeToWarehouse][mergeToPeriod][mergeToRouteIndex], "Old Route (Merge To)");

	// cout << "Merging customer " << selectedCustomer << " from period " << mergeFromPeriod + 1
	// 	 << " and warehouse " << mergeFromWarehouse + 1
	// 	 << " to period " << mergeToPeriod + 1
	// 	 << " and warehouse " << mergeToWarehouse + 1 << endl;

	// Remove customer from mergeFrom period
	auto &fromRoute = sol_SE_feasible.routesWarehouseToCustomer[mergeFromWarehouse][mergeFromPeriod][mergeFromRouteIndex];
	fromRoute.erase(std::remove(fromRoute.begin(), fromRoute.end(), selectedCustomer), fromRoute.end());

	// If route is empty (only warehouse nodes), clear it
	if (fromRoute.size() == 2)
	{
		fromRoute.clear();
	}

	// Update customer assignment matrix if warehouses are different
	if (mergeFromWarehouse != mergeToWarehouse)
	{
		sol_SE_feasible.customerAssignmentToWarehouse[mergeFromPeriod][mergeFromWarehouse][selectedCustomer - params.numWarehouses] = 0;
		sol_SE_feasible.customerAssignmentToWarehouse[mergeToPeriod][mergeToWarehouse][selectedCustomer - params.numWarehouses] = 1;
	}

	// Output new routes after the merge
	// printRoute(mergeFromWarehouse, mergeFromPeriod, mergeFromRouteIndex, fromRoute, "New Route (Merge From)");
	// printRoute(mergeToWarehouse, mergeToPeriod, mergeToRouteIndex, sol_SE_feasible.routesWarehouseToCustomer[mergeToWarehouse][mergeToPeriod][mergeToRouteIndex], "New Route (Merge To)");

	// Solve LP after merge to check for improvement
	if (!solveLP())
	{
		return false;
	}

	return true;
}

// Transfer Operator
bool LocalSearch_Deterministic::Transfer()
{
    // cout << "\nTransfer" << endl;

    // Step 1: Shuffle periods for random selection
    vector<int> periods(params.numPeriods);
    for (int t = 0; t < params.numPeriods; ++t)
        periods[t] = t;
    std::shuffle(periods.begin(), periods.end(), rng);

    // Randomly select a customer
    int randomCustomerIndex = rand() % params.numCustomers;
    int customer = randomCustomerIndex + params.numWarehouses;

    // cout << "Selected Customer: " << customer << endl;

    vector<std::pair<int, int>> visitedPeriodsWarehouses;
    vector<int> unvisitedPeriods;

    // Step 2: Iterate through shuffled periods and check visited/unvisited warehouses for the customer
    for (int t : periods)
    {
        bool visitedInPeriod = false;
        for (int w = 0; w < params.numWarehouses; ++w)
        {
            bool found = false;
            if (sol_SE_feasible.customerAssignmentToWarehouse[t][w][customer - params.numWarehouses] == 1)
            {
                for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[w][t])
                {
                    if (std::find(route.begin(), route.end(), customer) != route.end())
                    {
                        found = true;
                        visitedPeriodsWarehouses.emplace_back(t, w);
                        visitedInPeriod = true;
                        break;
                    }
                }
            }
        }
        if (!visitedInPeriod)
        {
            unvisitedPeriods.push_back(t);
        }
    }

    // Step 3: If the customer is not visited in any periods, skip transfer
    if (visitedPeriodsWarehouses.empty())
    {
        // cout << "Customer " << customer << " is not visited in any periods." << endl;
        return false;
    }

    // If customer is visited in all periods, skip transfer
    else if (unvisitedPeriods.empty())
    {
        // cout << "Customer " << customer << " is visited in all periods." << endl;
        return false;
    }

    // Step 4: Remove the customer from all periods they are currently visited
    for (auto &entry : visitedPeriodsWarehouses)
    {
        int periodToRemove = entry.first;
        int warehouseToRemove = entry.second;

        for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[warehouseToRemove][periodToRemove])
        {
            auto it = std::remove(route.begin(), route.end(), customer);
            if (it != route.end())
            {
                route.erase(it, route.end());
            }

            // Clear route if only start and end warehouse nodes remain
            if (route.size() == 2)
            {
                route.clear();
            }
        }
    }

    // cout << "Customer " << customer << " removed from periods: ";
    // for (auto &entry : visitedPeriodsWarehouses)
    // {
    //     cout << entry.first + 1 << " ";
    // }
    // cout << endl;

    // Step 5: Randomly select a warehouse for insertion and try to insert the customer into all unvisited periods
    std::uniform_int_distribution<int> warehouseDist(0, params.numWarehouses - 1);
    int warehouseToInsert = warehouseDist(rng);

    for (int periodToInsert : unvisitedPeriods)
    {
        double minInsertionCost = std::numeric_limits<double>::max();
        int minInsertionPos = -1;
        int routeToInsert = -1;
        int routeIndex = 0;

        // Iterate over all routes in the selected warehouse and period to find the best insertion position
        for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][periodToInsert])
        {
            if (route.empty())
            {
                // Insert into an empty route
                double insertionCost = 2 * params.transportationCost_SecondEchelon[warehouseToInsert][customer];
                if (insertionCost < minInsertionCost)
                {
                    minInsertionCost = insertionCost;
                    minInsertionPos = 0;
                    routeToInsert = routeIndex;
                }
            }
            else
            {
                // Calculate insertion cost for non-empty routes
                for (int pos = 1; pos < route.size(); ++pos)
                {
                    double insertionCost = params.transportationCost_SecondEchelon[route[pos - 1]][customer] +
                                           params.transportationCost_SecondEchelon[customer][route[pos]] -
                                           params.transportationCost_SecondEchelon[route[pos - 1]][route[pos]];

                    if (insertionCost < minInsertionCost)
                    {
                        minInsertionCost = insertionCost;
                        minInsertionPos = pos;
                        routeToInsert = routeIndex;
                    }
                }
            }
            routeIndex++;
        }

        // Perform the insertion if a valid position is found
        if (routeToInsert != -1)
        {
            // Ensure the customer is no longer assigned to any other warehouse for this period
            for (int w = 0; w < params.numWarehouses; ++w)
            {
                if (w != warehouseToInsert)
                {
                    sol_SE_feasible.customerAssignmentToWarehouse[periodToInsert][w][customer - params.numWarehouses] = 0;
                }
            }

            // Insert the customer into the selected position
            if (sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][periodToInsert][routeToInsert].empty())
            {
                // Insert into an empty route
                sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][periodToInsert][routeToInsert] = {warehouseToInsert, customer, warehouseToInsert};
            }
            else
            {
                // Insert into a non-empty route
                sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][periodToInsert][routeToInsert].insert(
                    sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][periodToInsert][routeToInsert].begin() + minInsertionPos,
                    customer);
            }

            // Update the customerAssignmentToWarehouse matrix for the new warehouse
            sol_SE_feasible.customerAssignmentToWarehouse[periodToInsert][warehouseToInsert][customer - params.numWarehouses] = 1;
        }
    }

    // cout << "Customer " << customer << " inserted into periods: ";
    // for (int periodToInsert : unvisitedPeriods)
    // {
    //     cout << periodToInsert + 1 << " ";
    // }
    // cout << endl;

    // Solve the linear program to check for improvement
    if (solveLP())
    {
        return true;
    }

    return false;
}

// Remove/Insert Operator
bool LocalSearch_Deterministic::Remove_Insert()
{
    // cout << "\nRemove/Insert" << endl;

    // Step 1: Shuffle periods to introduce randomness
    vector<int> periods(params.numPeriods);
    for (int t = 0; t < params.numPeriods; ++t)
        periods[t] = t;
    std::shuffle(periods.begin(), periods.end(), rng);

    // Step 2: Iterate through periods and find a period with at least one customer and their warehouse
    int fromPeriod = -1;
    std::pair<int, int> selectedCustomerWarehouse = {-1, -1}; // Pair to hold {customer, warehouse}

    for (const int t : periods)
    {
        vector<std::pair<int, int>> customerWarehousePairs; // Store {customer, warehouse}

        for (int w = 0; w < params.numWarehouses; ++w)
        {
            for (const auto &route : sol_SE_feasible.routesWarehouseToCustomer[w][t])
            {
                for (int node : route)
                {
                    if (node != w) // Avoid warehouse nodes
                    {
                        customerWarehousePairs.emplace_back(node, w); // Add customer and warehouse pair
                    }
                }
            }
        }

        if (!customerWarehousePairs.empty())
        {
            // Randomly select a customer and warehouse from the period
            selectedCustomerWarehouse = customerWarehousePairs[rand() % customerWarehousePairs.size()];
            fromPeriod = t;
            break;
        }
    }

    // If no valid period is found with customers, return false
    if (fromPeriod == -1 || selectedCustomerWarehouse.first == -1)
    {
        // cout << "No periods with customers found." << endl;
        return false;
    }

    int selectedCustomer = selectedCustomerWarehouse.first;
    int warehouseFrom = selectedCustomerWarehouse.second;

    // Step 3: Identify periods where the selected customer is not visited
    vector<int> targetPeriods;
    for (int t : periods)
    {
        if (t == fromPeriod) continue;

        bool found = false;
        for (int w = 0; w < params.numWarehouses; ++w)
        {
            if (sol_SE_feasible.customerAssignmentToWarehouse[t][w][selectedCustomer - params.numWarehouses] == 1)
            {
                for (const auto &route : sol_SE_feasible.routesWarehouseToCustomer[w][t])
                {
                    if (std::find(route.begin(), route.end(), selectedCustomer) != route.end())
                    {
                        found = true;
                        break;
                    }
                }
            }
        }
        if (!found)
        {
            targetPeriods.push_back(t);
        }
    }

    if (targetPeriods.empty())
    {
        // cout << "No eligible periods to move the customer to." << endl;
        return false;
    }

    int toPeriod = targetPeriods[rand() % targetPeriods.size()];

    // Step 4: Remove the customer from the original warehouse and period
    for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[warehouseFrom][fromPeriod])
    {
        auto it = std::find(route.begin(), route.end(), selectedCustomer);
        if (it != route.end())
        {
            route.erase(it);
            if (route.size() == 2) // Clear the route if only depots are left
            {
                route.clear();
            }
            break;
        }
    }

    // Step 5: Randomly select a warehouse for insertion
    std::uniform_int_distribution<int> warehouseDist(0, params.numWarehouses - 1);
    int warehouseToInsert = warehouseDist(rng);

    // Step 6: Find the best insertion point in the selected warehouse for the target period
    double minInsertionCost = std::numeric_limits<double>::max();
    int minInsertionPos = -1;
    int routeToInsert = -1;
    int routeIndex = 0;

    for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][toPeriod])
    {
        if (route.empty())
        {
            // If the route is empty, insert the customer between the warehouse nodes
            double insertionCost = 2 * params.transportationCost_SecondEchelon[warehouseToInsert][selectedCustomer];
            if (insertionCost < minInsertionCost)
            {
                minInsertionCost = insertionCost;
                minInsertionPos = 0;
                routeToInsert = routeIndex;
            }
        }
        else
        {
            // Calculate insertion cost for non-empty routes
            for (int pos = 1; pos < route.size(); ++pos)
            {
                double insertionCost = params.transportationCost_SecondEchelon[route[pos - 1]][selectedCustomer] +
                                       params.transportationCost_SecondEchelon[selectedCustomer][route[pos]] -
                                       params.transportationCost_SecondEchelon[route[pos - 1]][route[pos]];

                if (insertionCost < minInsertionCost)
                {
                    minInsertionCost = insertionCost;
                    minInsertionPos = pos;
                    routeToInsert = routeIndex;
                }
            }
        }
        routeIndex++;
    }

    // Step 7: Perform the insertion if a valid position is found
    if (routeToInsert != -1)
    {
        // Ensure the customer is no longer assigned to any other warehouse for the target period
        for (int w = 0; w < params.numWarehouses; ++w)
        {
            if (w != warehouseToInsert)
            {
                sol_SE_feasible.customerAssignmentToWarehouse[toPeriod][w][selectedCustomer - params.numWarehouses] = 0;
            }
        }

        if (sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][toPeriod][routeToInsert].empty())
        {
            // Insert into an empty route
            sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][toPeriod][routeToInsert] = {warehouseToInsert, selectedCustomer, warehouseToInsert};
        }
        else
        {
            // Insert into a non-empty route
            sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][toPeriod][routeToInsert].insert(
                sol_SE_feasible.routesWarehouseToCustomer[warehouseToInsert][toPeriod][routeToInsert].begin() + minInsertionPos,
                selectedCustomer);
        }

        // Update the customerAssignmentToWarehouse matrix for the new warehouse
        sol_SE_feasible.customerAssignmentToWarehouse[toPeriod][warehouseToInsert][selectedCustomer - params.numWarehouses] = 1;
    }

    // cout << "Moved customer " << selectedCustomer << " from period " << fromPeriod + 1 << " to period " << toPeriod + 1 << " in warehouse " << warehouseToInsert + 1 << "." << endl;

    // Step 8: Solve the LP to check for improvements
    if (solveLP())
    {
        return true;
    }

    return false;
}

bool LocalSearch_Deterministic::solveLP()
{
	// Define the improvement threshold
	const double improvementThreshold = 1e-2;

	if (params.problemType == "EEV"){
		// solve LP to get the value of the continuous variables for EEV
		LP_SE_Deterministic lpse_eev(params, sol_FE_temp, sol_SE_feasible);
		string status = lpse_eev.solve();
		if (status != "Optimal")
		{
			// cerr << "LP solver (EEV) failed with status (LS): " << status << endl;
			return false;
		}

		objVal_feasible = lpse_eev.getResult().objValue_Total;

		// Determine if the new solution is significantly better than the best known
		if (objVal_feasible < objVal_best_currStart - improvementThreshold)
		{
			// Update the best known solution
			sol_SE_best_currStart = lpse_eev.getSolutionSE();
			objVal_best_currStart = lpse_eev.getResult().objValue_Total;
			return true;
		}
	}
	else {
		// solve LP to get the value of the continuous variables
		LP_SE_Deterministic lpse(params, sol_FE_best_currStart, sol_SE_feasible);
		string status = lpse.solve();
		if (status != "Optimal")
		{
			// cerr << "LP solver failed with status: " << status << endl;
			return false;
		}

		objVal_feasible = lpse.getResult().objValue_Total;

		// Determine if the new solution is significantly better than the best known
		if (objVal_feasible < objVal_best_currStart - improvementThreshold)
		{	
			// Update the best known solution
			sol_FE_best_currStart = lpse.getSolutionFE();
			sol_SE_best_currStart = lpse.getSolutionSE();
			objVal_best_currStart = lpse.getResult().objValue_Total;
			return true;
		}
	}

	// No improvement found
	return false;
}

// Helper function to print a route with a label
void LocalSearch_Deterministic::printRoute(int ware, int per, int rI, const vector<int> &route, const std::string &label) const
{
	cout << label << "[" << ware + 1 << "][" << per + 1 << "][" << rI + 1 << "]: [";
	if (!route.empty()){
		for (size_t i = 0; i < route.size(); ++i)
		{
			cout << route[i];
			if (i != route.size() - 1)
				cout << " -> ";
		}
	}
	cout << "]" << endl;
}

void LocalSearch_Deterministic::printAllRoutes() const
{
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				const auto &route = sol_SE_feasible.routesWarehouseToCustomer[w][t][k];
				printRoute(w, t, k, route, "Route");
			}
		}
	}
	cout << "\n";
}
SolutionFirstEchelon LocalSearch_Deterministic::getSolutionFE()
{
	return sol_FE_best;
}

SolutionSecondEchelon_Deterministic LocalSearch_Deterministic::getSolutionSE()
{
	return sol_SE_best;
}

double LocalSearch_Deterministic::getObjVal()
{
	return objVal_best;
}