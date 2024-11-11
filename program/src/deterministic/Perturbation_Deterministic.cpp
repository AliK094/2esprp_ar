#include "deterministic/Perturbation_Deterministic.h"

Perturbation_Deterministic::Perturbation_Deterministic(const ParameterSetting &parameters,
						   							   const SolutionFirstEchelon &sol_FE,
						   							   const SolutionSecondEchelon_Deterministic &sol_SE,
						   							   const vector<vector<double>> &deterministicDemand,
						  							   bool shortageAllowed,
													   bool isEEV)
	: params(parameters),
	  sol_FE_temp(sol_FE),
	  sol_SE_temp(sol_SE),
	  demand(deterministicDemand),
	  shortageAllowed(shortageAllowed),
	  isEEV(isEEV),
	  rng(std::random_device{}())
{
	// ----------------------------------------------------------------------------------------------------------
	// cout << "\nInitializing Perturbation_Deterministic for " << endl;
	// ----------------------------------------------------------------------------------------------------------
}

bool Perturbation_Deterministic::run()
{
	// --------------------------------------------------------------------------------------------------------------------------
	// Initialize the operators
	vector<std::function<bool()>> perturbOperators = setPerturbOperators();
	int max_perturb = 20;
	int perturbIteration = 0;
	// --------------------------------------------------------------------------------------------------------------------------
	while (perturbIteration < max_perturb)
	{
		// cout << "Perturbation_Deterministic...: " << endl;
		sol_FE_feasible = sol_FE_temp;
		sol_SE_feasible = sol_SE_temp;
		int index = rand() % perturbOperators.size();

		// printAllRoutes();
		// cout << "\n";

		// Execute the operator
		if (!perturbOperators[index]())
		{
			perturbIteration++;
			continue;
		}

		if (isEEV)
		{
			// solve LP to get the value of the continuous variables for EEV
			LP_SE_EEV lpse_eev(params, sol_FE_temp, sol_SE_feasible, demand);
			string status = lpse_eev.solve();
			if (status != "Optimal")
			{
				perturbIteration++;
				continue;
			}
			sol_SE_feasible.clear();
			sol_SE_feasible = lpse_eev.getSolutionSE();
			objVal_feasible = lpse_eev.getResult().objValue_Total;
		}
		else {
			// solve LP to get the value of the continuous variables
			LP_SE_Deterministic lpse_deter(params, sol_FE_temp, sol_SE_feasible,
										demand, shortageAllowed);
			string status = lpse_deter.solve();
			if (status != "Optimal")
			{
				perturbIteration++;
				continue;
			}
			sol_SE_feasible.clear();
			sol_FE_feasible = lpse_deter.getSolutionFE();
			sol_SE_feasible = lpse_deter.getSolutionSE();
			objVal_feasible = lpse_deter.getResult().objValue_Total;
		}

		return true;
	}
	return false;
}

vector<std::function<bool()>> Perturbation_Deterministic::setPerturbOperators()
{
	return {
		[this]()
		{ return randomShift(1); },
		[this]()
		{ return randomSwap(1, 1); },
		[this]()
		{ return randomInsertion(); },
		[this]()
		{ return randomRemoval(); }};
}

// Random Shift Perturbation_Deterministic
bool Perturbation_Deterministic::randomShift(int v)
{
	// cout << "\nRandom Shift Perturbation_Deterministic" << endl;

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

						return true;
					} // End of insertPos loop
				} // End of startPos loop
			} // End of destVehicle loop
		} // End of sourceVehicle loop
	} // End of period loop

	// cerr << "Random Insertion Perturbation_Deterministic failed" << endl;
	return false;
}

bool Perturbation_Deterministic::randomSwap(int v1, int v2)
{
	// cout << "\nRnadom Swap Perturbation_Deterministic" << endl;

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

				if (sourceRoute.size() == static_cast<size_t>(v1 + 2) && destRoute.size() == static_cast<size_t>(v2 + 2) && sourceWarehouse == destWarehouse)
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

				return true;
			} // End of destVehicle loop
		} // End of sourceVehicle loop
	} // End of period loop

	// cerr << "Random Swap Perturbation_Deterministic Failed" << endl;
	return false;
}

bool Perturbation_Deterministic::randomInsertion()
{
	// cout << "\nRandom Insertion Perturbation_Deterministic" << endl;

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

		// Return true if an insertion was successfully made
		return true;
	}

	// cerr << "Random Insertion Perturbation_Deterministic failed." << endl;
	return false;
}

bool Perturbation_Deterministic::randomRemoval()
{
	// cout << "\nRandom Removal Perturbation_Deterministic" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period

	// Randomly choose a warehouse
	std::uniform_int_distribution<int> warehouse_dist(0, static_cast<int>(params.numWarehouses) - 1);
	int selectedWarehouse = warehouse_dist(rng);

	vector<int> candidateRoutes;

	int rtIndex = 0;
	for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t])
	{
		// printRoute(selectedWarehouse, t, rtIndex, route, "Old Route");

		rtIndex++;
	}

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
	if (selectedRoute.size() == 2)
	{
		// Clear the route since it only contains the warehouse start and end nodes
		selectedRoute.clear();
	}

	rtIndex = 0;
	for (auto &route : sol_SE_feasible.routesWarehouseToCustomer[selectedWarehouse][t])
	{
		// printRoute(selectedWarehouse, t, rtIndex, route, "New Route");

		rtIndex++;
	}

	// Log the operation
	// cout << "Removed customer " << removedCustomer << " from period " << t + 1
	// 	 << ", warehouse " << selectedWarehouse + 1 << ", route " << routeIndex + 1 << endl;

	return true;
}

// Helper function to print a route with a label
void Perturbation_Deterministic::printRoute(int ware, int per, int rI, const vector<int> &route, const std::string &label) const
{
	cout << label << "[" << ware + 1 << "][" << per + 1 << "][" << rI + 1 << "]: [";
	if (!route.empty())
	{
		for (size_t i = 0; i < route.size(); ++i)
		{
			cout << route[i];
			if (i != route.size() - 1)
				cout << " -> ";
		}
	}
	cout << "]" << endl;
}

void Perturbation_Deterministic::printAllRoutes() const
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
}

SolutionFirstEchelon Perturbation_Deterministic::getSolutionFE()
{
	return sol_FE_feasible;
}
SolutionSecondEchelon_Deterministic Perturbation_Deterministic::getSolutionSE()
{
	return sol_SE_feasible;
}

double Perturbation_Deterministic::getObjVal()
{
	return objVal_feasible;
}