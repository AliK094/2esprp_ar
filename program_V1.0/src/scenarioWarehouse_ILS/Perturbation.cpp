#include "Perturbation.h"

Perturbation::Perturbation(const int s, const int w, const ParameterSetting &parameters, const Solution &solution)
	: scenario(s), warehouse(warehouse), params(parameters), sol_temp(solution), rng(std::random_device{}())
{
	// ----------------------------------------------------------------------------------------------------------
	cout << "\nInitializing Perturbation for scenario " << scenario + 1 << " and warehouse " << warehouse + 1 << endl;
	// ----------------------------------------------------------------------------------------------------------
}

bool Perturbation::run()
{
	// --------------------------------------------------------------------------------------------------------------------------
	// Initialize the operators
	vector<std::function<bool()>> perturbOperators = setPerturbOperators();
	int max_perturb = 10;
	int perturbIteration = 0;

	while (perturbIteration < max_perturb)
	{
		cout << "Perturbation Iteration: " << perturbIteration << endl;

		sol_feasible = sol_temp;
		int index = rand() % perturbOperators.size();

		printAllRoutes();
		cout << "\n";

		// Execute the operator
		if (!perturbOperators[index]())
		{
			perturbIteration++;
			continue;
		}

		// solve LP to get the value of the continuous variables
		LP_SE lpse(params, sol_feasible);		
		string status = lpse.solve();
		if (status != "Optimal")
		{
			perturbIteration++;
			continue;
		}
		sol_feasible.clear();
		sol_feasible = lpse.getSolution();

		return true;
	}
	return false;
}

vector<std::function<bool()>> Perturbation::setPerturbOperators()
{
	return {
		[this]()
		{ return randomShift(1); },
		[this]()
		{ return randomInsertion(); },
		[this]()
		{ return randomRemoval(); }
	};
}

// Random Shift Perturbation
bool Perturbation::randomShift(int v)
{
	cout << "Random Shift Perturbation" << endl;

	// Collect all eligible periods
	vector<int> periods(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
		periods[t] = t;
	std::shuffle(periods.begin(), periods.end(), rng); // Shuffle to introduce randomness

	// Create random warehouse distributions for destination warehouses
	std::uniform_int_distribution<int> warehouse_dist(0, static_cast<int>(params.numWarehouses) - 1);

	for (const int t : periods)
	{
		// Collect all eligible source routes for the current period in the warehouse
		vector<int> sourceVehicles;
		for (int sourceVehicle = 0; sourceVehicle < params.numVehicles_Warehouse; ++sourceVehicle)
		{
			vector<int> &sourceRoute = sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t][sourceVehicle];

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
			vector<int> &sourceRoute = sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t][sourceVehicle];

			// Select a random destination warehouse (could be the same as the source warehouse)
			int destWarehouse = warehouse_dist(rng);

			// Collect all eligible destination routes (excluding the source route)
			vector<int> destVehicles;
			for (int destVehicle = 0; destVehicle < params.numVehicles_Warehouse; ++destVehicle)
			{
				if (destWarehouse == warehouse && destVehicle == sourceVehicle)
					continue; // Skip same route

				vector<int> &destRoute = sol_feasible.routesWarehouseToCustomer[scenario][destWarehouse][t][destVehicle];

				// No specific size constraints for destination routes, but can add if needed
				destVehicles.push_back(destVehicle);
			}

			if (destVehicles.empty())
				continue; // No eligible destination routes

			// Shuffle destination vehicles to introduce randomness in selection
			std::shuffle(destVehicles.begin(), destVehicles.end(), rng);

			for (const int destVehicle : destVehicles)
			{
				vector<int> &destRoute = sol_feasible.routesWarehouseToCustomer[scenario][destWarehouse][t][destVehicle];

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
						printRoute(scenario, warehouse, t, sourceVehicle, sourceRoute, "Original Source Route");
						printRoute(scenario, destWarehouse, t, destVehicle, destRoute, "Original Dest Route");

						// Perform the shift: remove from source and insert into destination
						sourceRoute.erase(sourceRoute.begin() + startPos, sourceRoute.begin() + endPos);
						destRoute.insert(destRoute.begin() + insertPos, segment.begin(), segment.end());

						// Update the customer assignment matrix: change the warehouse assignment for each customer
						for (const int customer : segment)
						{
							// Set previous warehouse assignment to 0
							sol_feasible.customerAssignmentToWarehouse[scenario][t][warehouse][customer - params.numWarehouses] = 0;

							// Set new warehouse assignment to 1
							sol_feasible.customerAssignmentToWarehouse[scenario][t][destWarehouse][customer - params.numWarehouses] = 1;
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
						cout << "Shifted " << v << " customer(s) from warehouse " << warehouse + 1 
							 << " route " << sourceVehicle + 1 << " to warehouse " << destWarehouse + 1 
							 << " route " << destVehicle + 1 << " in period " << t + 1 << endl;

						// Print updated routes
						printRoute(scenario, warehouse, t, sourceVehicle, sourceRoute, "Updated Source Route");
						printRoute(scenario, destWarehouse, t, destVehicle, destRoute, "Updated Dest Route");

						return true;
					} // End of insertPos loop
				} // End of startPos loop
			} // End of destVehicle loop
		} // End of sourceVehicle loop
	} // End of period loop

	// If no valid shift found that improves the solution
	std::cerr << "No valid shift found that improves the solution." << endl;
	return false;
}

// Random Insertion Perturbation
bool Perturbation::randomInsertion()
{
	cout << "Random Insertion Perturbation" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period

	vector<std::pair<int, int>> visitedNodesWithWarehouse;	 // Stores <node, warehouse>
	vector<std::pair<int, int>> unvisitedNodesWithWarehouse; // Stores <node, assigned warehouse> for unvisited nodes

	// Lambda function to check if a node is in visitedNodesWithWarehouse
	auto isNodeVisited = [&](int node)
	{
		return std::any_of(visitedNodesWithWarehouse.begin(), visitedNodesWithWarehouse.end(),
						   [&](const std::pair<int, int> &p)
						   { return p.first == node; });
	};

	// Loop through all warehouses
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		int rtIndex = 0;
		for (auto &route : sol_feasible.routesWarehouseToCustomer[scenario][w][t])
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
			printRoute(scenario, w, t, rtIndex, route, "Old Route");

			rtIndex++;
		}
	}

	// Identify unvisited nodes for this warehouse
	for (int i = 0; i < params.numCustomers; ++i)
	{
		int node = i + params.numWarehouses;
		if (!isNodeVisited(node))
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				if (sol_feasible.customerAssignmentToWarehouse[scenario][t][w][i] == 1)
				{
					unvisitedNodesWithWarehouse.emplace_back(node, w); // Store the unvisited node with its warehouse
					break;
				}
			}
		}
	}

	if (unvisitedNodesWithWarehouse.empty())
	{
		cout << "No unvisited nodes available for insertion." << endl;
		return false;
	}

	auto randomPair = unvisitedNodesWithWarehouse[rand() % unvisitedNodesWithWarehouse.size()];
	int unvisitedNode = randomPair.first;	   // The node part of the selected pair
	int assignedWarehouse = randomPair.second; // The warehouse part of the selected pair

	// Attempt to insert the unvisited node at the minimum cost position
	bool inserted = false;
	double minInsertionCost = std::numeric_limits<double>::max();
	int minInsertionPos = -1;
	int routeToIns = -1;
	int routeInd = 0;

	for (auto &route : sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t])
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
	if (routeToIns != -1 && !sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t][routeToIns].empty())
	{
		sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t][routeToIns].insert(sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t][routeToIns].begin() + minInsertionPos, unvisitedNode);
		inserted = true;
	}
	else if (routeToIns != -1 && sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t][routeToIns].empty())
	{
		// If the route is empty, add warehouse -> node -> warehouse
		sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t][routeToIns] = {warehouse, unvisitedNode, warehouse};
		inserted = true;
	}

	if (!inserted)
	{
		cout << "Failed to insert unvisited node." << endl;
		return false;
	}

	// Update the customerAssignmentToWarehouse matrix

	if (assignedWarehouse != warehouse)
	{
		sol_feasible.customerAssignmentToWarehouse[scenario][t][assignedWarehouse][unvisitedNode - params.numWarehouses] = 0;
		sol_feasible.customerAssignmentToWarehouse[scenario][t][warehouse][unvisitedNode - params.numWarehouses] = 1;
	}

	int rtIndex = 0;
	for (auto &route : sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t])
	{
		printRoute(scenario, warehouse, t, rtIndex, route, "Old Route");
		
		rtIndex++;
	}
	cout << "Inserted unvisited node " << unvisitedNode << " at period " << t + 1 << " with minimum cost." << endl;

	return true;
}

bool Perturbation::randomRemoval()
{
	cout << "Random Removal Perturbation" << endl;

	int t = rand() % params.numPeriods; // Randomly choose a period
	vector<int> candidateRoutes;

	int rtIndex = 0;
	for (auto &route : sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t])
	{
		cout << "Old Route[" << warehouse + 1 << "][" << t + 1 << "][" << rtIndex + 1 << "]: [";
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
	for (int i = 0; i < sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t].size(); ++i)
	{
		if (sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t][i].size() > 2)
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
	auto &selectedRoute = sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t][routeIndex];

	// Randomly select a customer node to remove, avoiding the warehouse nodes at the start and end if present
	int nodeIndex = rand() % (selectedRoute.size() - 2) + 1; // Avoid first and last positions if they are warehouses
	selectedRoute.erase(selectedRoute.begin() + nodeIndex);
	if (selectedRoute.size() == 2)
	{
		// Clear the route since it only contains the warehouse start and end nodes
		selectedRoute.clear();
	}

	rtIndex = 0;
	for (auto &route : sol_feasible.routesWarehouseToCustomer[scenario][warehouse][t])
	{
		cout << "New Route[" << warehouse + 1 << "][" << t + 1 << "][" << rtIndex + 1 << "]: [";
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
	cout << "Removed a visited node from period " << t + 1 << ", route " << routeIndex + 1 << endl;
	return true;
}

Solution Perturbation::getSolution()
{
	return sol_feasible;
}

// Helper function to print a route with a label
void Perturbation::printRoute(int scen, int ware, int per, int rI, const vector<int> &route, const std::string &label) const
{
	cout << label << "[" << scen + 1 << "][" << ware + 1 << "][" << per + 1 << "][" << rI + 1 << "]: [";
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

void Perturbation::printAllRoutes() const
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					const auto &route = sol_feasible.routesWarehouseToCustomer[s][w][t][k];
					printRoute(s, w, t, k, route, "Route");
				}
			}
		}
	}
}