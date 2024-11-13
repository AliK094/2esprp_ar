#include "deterministic/LP_SE_EEV.h"

// ------------------------------------------------------------------------------------------------------------------------
LP_SE_EEV::LP_SE_EEV(const ParameterSetting &parameters,
			 const SolutionFirstEchelon &solFE,
			 const SolutionSecondEchelon_Deterministic &solSE,
			 const vector<vector<double>> &deterministicDemand)
	: params(parameters),
	  sol_FE_EV(solFE),
	  sol_SE(solSE),
	  demand(deterministicDemand),
	  THRESHOLD(1e-2),
	  save_lpFile(false),
	  save_mpsResultFile(false)
{
}

string LP_SE_EEV::solve()
{
	IloEnv env;
	IloModel model(env);
	IloCplex cplex(model);

	auto startTime = std::chrono::high_resolution_clock::now();

	DefineVariables(env, model);
	DefineObjectiveFunction(env, model);
	DefineConstraints(env, model);

	/* Assure linear mappings between the presolved and original models */
	cplex.setParam(IloCplex::Param::Threads, 1);
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
	cplex.setOut(env.getNullStream());

	cplex.setWarning(env.getNullStream());

	if (save_lpFile)
	{
		string directory = "../cplexFiles/lpModel/";
		string lpFileName;
		lpFileName = directory + "LPSE_EEV_NW" + std::to_string(params.numWarehouses) + 
					"_NR" + std::to_string(params.numCustomers) + 
					"_KP" + std::to_string(params.numVehicles_Plant) + 
					"_KW" + std::to_string(params.numVehicles_Warehouse) + 
					"_T" + std::to_string(params.numPeriods) + 
					"_S" + std::to_string(params.numScenarios) + 
					"_Ins" + params.instance.c_str() + ".lp";

		// Export the model to an LP file
		cplex.exportModel(lpFileName.c_str());
	}

	// Extract model
	cplex.extract(model);

	// cout << "\nSolving LP_SE_EEV..." << endl; 

	// Solve the model
	cplex.solve();

	string status;
	double objValue = 0.0;

	if (cplex.getStatus() == IloAlgorithm::Optimal)
	{
		status = "Optimal";
		objValue = cplex.getObjValue();
		// cout << "Optimal solution found with objective value: " << std::fixed << std::setprecision(1) << objValue << endl;

		if (save_mpsResultFile)
		{
			string directory = "../cplexFiles/solVal/";
			string solFileName;
			solFileName = directory + "LPSE_EEV_NW" + std::to_string(params.numWarehouses) + 
						"_NR" + std::to_string(params.numCustomers) + 
						"_KP" + std::to_string(params.numVehicles_Plant) + 
						"_KW" + std::to_string(params.numVehicles_Warehouse) + 
						"_T" + std::to_string(params.numPeriods) + 
						"_S" + std::to_string(params.numScenarios) + 
						"_Ins" + params.instance.c_str() + ".lp";

			// Export the model to an LP file
			cplex.writeSolution(solFileName.c_str());
		}
	}
	else if (cplex.getStatus() == IloAlgorithm::Infeasible)
	{
		// status = "Infeasible";
		// cout << "Problem is infeasible" << endl;

		// // Iterate through the model to identify constraints
		// IloConstraintArray constraints(env);
		// IloNumArray priorities(env);

		// // Add all constraints to the conflict array with priority 1
		// for (IloModel::Iterator it(model); it.ok(); ++it)
		// {
		// 	IloExtractable extractable = *it;
		// 	if (extractable.isConstraint())  // Check if it's a constraint
		// 	{
		// 		IloConstraint constraint = extractable.asConstraint();
		// 		constraints.add(constraint);
		// 		priorities.add(1.0);  // Assign a priority (1.0 = low penalty)
		// 	}
		// }

		// // Run conflict refinement to identify infeasible subset
		// if (cplex.refineConflict(constraints, priorities))
		// {
		// 	cout << "Constraints in conflict:" << endl;
			
		// 	// Iterate through the constraints and check conflict status
		// 	for (int i = 0; i < constraints.getSize(); ++i)
		// 	{
		// 		if (cplex.getConflict(constraints[i]) == IloCplex::ConflictStatus::ConflictMember)
		// 		{
		// 			cout << " - " << constraints[i].getName() << endl;  // Print the name of the conflicting constraint
		// 		}
		// 	}
		// }
		// else
		// {
		// 	cout << "Could not find an IIS to explain infeasibility." << endl;
		// }

		// // Clean up
		// constraints.end();
		// priorities.end();

		// exit(1);
	}
	else
	{
		status = "Undefined";
		cout << "Solver terminated with status: " << status << endl;
	}

	auto currentTime = std::chrono::high_resolution_clock::now();
	auto totalCPUTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
	// cout << "Timer (seconds): " << std::fixed << std::setprecision(3) << totalCPUTime << endl;

	if (status == "Optimal" || status == "Incumbent")
	{
		// cout << "Retrieving solutions..." << endl;
		// Retrieve the solution
		RetrieveSolutions(cplex);
		CalculateCostsForEachPart();
		// DisplayCosts();

		// Display the solution
		// DisplayProductionSetupVars();
		// DisplayProductionQuantVars();
		// DisplayPlantInventoryVars();
		// DisplayWarehouseInventoryVars();
		// DisplayFirstEchelonRouteVars();
		// DisplayDeliveryQuantityToWarehousesVars();
		// DisplayCustomerInventoryVars();
		// DisplayCustomerUnmetDemandVars();
		// DisplayDeliveryQuantityToCustomersVars();
		// DisplayRoutesWarehouseToCustomersVars();
		// cout << "\n\n\n\n" << endl;
	}

	env.end();

	return status;
}

void LP_SE_EEV::DefineVariables(IloEnv &env, IloModel &model)
{
	/* Define Decision Variables */
	// Initialize Variable Manager
	VariableManager varManager(env);
	// -------------------------------------------------------------------------------------------------------------------------------
	// I_warehouse[w][t] variables (Warehouse w inventory in period t)
	I_warehouse = varManager.create2D(params.numWarehouses, params.numPeriods);
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "I_warehouse[" + std::to_string(w + 1) + "][" + std::to_string(t + 1) + "]";
			I_warehouse[w][t] = IloNumVar(env, 0.0, params.storageCapacity_Warehouse[w], IloNumVar::Float, varName.c_str());
			model.add(I_warehouse[w][t]);
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// I_customer[i][t] variables (Customer i inventory in period t)
	I_customer = varManager.create2D(params.numCustomers, params.numPeriods);
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "I_customer[" + std::to_string(i + 1 + params.numWarehouses) + "][" + std::to_string(t + 1) + "]";
			I_customer[i][t] = IloNumVar(env, 0.0, params.storageCapacity_Customer[i], IloNumVar::Float, varName.c_str());
			model.add(I_customer[i][t]);
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// b_customer[i][t] variables (Customer i unsatisfied demand in period t)
	b_customer = varManager.create2D(params.numCustomers, params.numPeriods);
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "b_customer[" + std::to_string(i + 1 + params.numWarehouses) + "][" + std::to_string(t + 1) + "]";
			b_customer[i][t] = IloNumVar(env, 0.0, demand[i][t], IloNumVar::Float, varName.c_str());
			model.add(b_customer[i][t]);
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// w_customer[i][k][t] variables (Delivery quantity to customer i by vehicle k in period t)
	w_customer = varManager.create3D(params.numCustomers, params.numVehicles_SecondEchelon, params.numPeriods);
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				string varName = "w[" + std::to_string(i + 1 + params.numWarehouses) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "]";
				w_customer[i][k][t] = IloNumVar(env, 0.0, demand[i][t], IloNumVar::Float, varName.c_str());
				model.add(w_customer[i][k][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
}

void LP_SE_EEV::DefineObjectiveFunction(IloEnv &env, IloModel &model)
{
	// Define objective function
	IloExpr obj(env);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			obj += params.unitHoldingCost_Warehouse[w] * I_warehouse[w][t];
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			obj += params.unitHoldingCost_Customer[i] * I_customer[i][t];
			obj += params.unmetDemandPenalty[i] * b_customer[i][t];
		}
	}
	model.add(IloMinimize(env, obj));
}

void LP_SE_EEV::DefineConstraints(IloEnv &env, IloModel &model)
{
	/* Define Constraints */
	vector<vector<vector<double>>> warehouse_delivery(params.numWarehouses, vector<vector<double>>(params.numVehicles_Plant, vector<double>(params.numPeriods, 0.0)));
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Plant; ++k)
		{
			vector<int> route = sol_FE_EV.routesPlantToWarehouse[t][k];
			if (!route.empty())
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					int warehouseIndex = w + 1;
					auto it = std::find(route.begin() + 1, route.end() - 1, warehouseIndex);
					if (it != route.end() - 1)
					{
						warehouse_delivery[w][k][t] += sol_FE_EV.deliveryQuantityToWarehouse[w][t];
					}
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Warehouses Inventory Capacity Constraints:
			I_warehouse[w][t] <= storageCapacity_warehouse 		for all w in W, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			string constraintName = "WarehouseInventoryCapacity(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			expr += I_warehouse[w][t];
			IloConstraint warehouseInventoryCapacityConstraint(expr <= params.storageCapacity_Warehouse[w]);
			expr.end();

			model.add(warehouseInventoryCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Warehouse Inventory Balance Constraints:
			I[w][t] = I[w][t-1] + sum(k in KP) q[w][k][t] - sum(i in N_c) sum(k in K_w) w_customer[i][k][t]	for all w in W, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			string constraintName = "WarehouseInventoryBalance(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			if (t == 0)
			{
				expr += I_warehouse[w][t];

				for (int k = 0; k < params.numVehicles_Plant; ++k)
				{
					expr -= warehouse_delivery[w][k][t];
				}

				for (int k : params.set_WarehouseVehicles[w])
				{
					for (int i = 0; i < params.numCustomers; ++i)
					{
						expr += w_customer[i][k][t];
					}
				}
				IloConstraint WarehouseInventoryBalanceConstraint(expr == params.initialInventory_Warehouse[w]);
				expr.end();

				model.add(WarehouseInventoryBalanceConstraint).setName(constraintName.c_str());
			}
			else
			{
				expr += I_warehouse[w][t];
				expr -= I_warehouse[w][t - 1];

				for (int k = 0; k < params.numVehicles_Plant; ++k)
				{
					expr -= warehouse_delivery[w][k][t];
				}

				for (int k : params.set_WarehouseVehicles[w])
				{
					for (int i = 0; i < params.numCustomers; ++i)
					{
						expr += w_customer[i][k][t];
					}
				}
				IloConstraint WarehouseInventoryBalanceConstraint(expr == 0);
				expr.end();

				model.add(WarehouseInventoryBalanceConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Customers Inventory Balance Constraints:
			I[i][t] = I[i][t-1] + sum(k in K) w_customer[i][k][t] - d[i][t] + b[i][t] 	for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			string constraintName = "CustomerInventoryBalance(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			if (t == 0)
			{
				expr += I_customer[i][t];
				for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
				{
					expr -= w_customer[i][k][t];
				}
				expr -= b_customer[i][t];
				IloConstraint CustomerInventoryBalanceConstraint(expr == params.initialInventory_Customer[i] - demand[i][t]);
				expr.end();

				model.add(CustomerInventoryBalanceConstraint).setName(constraintName.c_str());
			}
			else
			{
				expr += I_customer[i][t];
				expr -= I_customer[i][t - 1];
				for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
				{
					expr -= w_customer[i][k][t];
				}
				expr -= b_customer[i][t];

				IloConstraint CustomerInventoryBalanceConstraint(expr == -demand[i][t]);
				expr.end();

				model.add(CustomerInventoryBalanceConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Customers Inventory Capacity Constraints:
			I[i][t] + d[i][t] <= params.storageCapacity_Customer[i] 		for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			string constraintName = "CustomerInventoryCapacity(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			expr += I_customer[i][t];
			IloConstraint CustomerInventoryCapacityConstraint(expr <= params.storageCapacity_Customer[i] - demand[i][t]);
			expr.end();

			model.add(CustomerInventoryCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Customer Visit Constraints (From Warehouse to Customer):
			w_customer[i][k][t] <= DeliveryUB[i][t]		for all i in N_c, k in union K, t in T

	*/
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				int vehicleIndex = k + (w * params.numVehicles_Warehouse);

				vector<int> route = sol_SE.routesWarehouseToCustomer[w][t][k];
				for (int i = 0; i < params.numCustomers; ++i)
				{
					int customerIndex = i + params.numWarehouses;
					auto it = std::find(route.begin(), route.end(), customerIndex);

					string constraintName = "CustomerVisit(" + std::to_string(customerIndex + 1) + "," + std::to_string(vehicleIndex + 1) + "," + std::to_string(t + 1) + ")";
					IloExpr expr(env);
					expr += w_customer[i][vehicleIndex][t];

					if (it != route.end())
					{
						IloConstraint customerVisitConstraint(expr <= params.storageCapacity_Customer[i]);

						expr.end();

						model.add(customerVisitConstraint).setName(constraintName.c_str());
					}
					else
					{
						IloConstraint customerVisitConstraint(expr == 0.0);

						expr.end();

						model.add(customerVisitConstraint).setName(constraintName.c_str());
					}
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Vehicle Capacity Constraints (From Warehouse to Customer):
			sum(i in N_c) w_customer[i][k][t] <= Q^w		for all w in N_w, k in K_w, t in T

	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k : params.set_WarehouseVehicles[w])
			{
				string constraintName = "VehicleCapacityWarehouse(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

				IloExpr expr(env);
				for (int i = 0; i < params.numCustomers; ++i)
				{
					expr += w_customer[i][k][t];
				}
				expr -= params.vehicleCapacity_Warehouse;
				IloConstraint vehicleCapacityConstraint(expr <= 0);
				expr.end();

				model.add(vehicleCapacityConstraint).setName(constraintName.c_str());
			}
		}
	}
	/* End of Model Constraints */
}

void LP_SE_EEV::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	sol_SE_temp.warehouseInventory.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	sol_SE_temp.customerInventory.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	sol_SE_temp.customerUnmetDemand.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	sol_SE_temp.deliveryQuantityToCustomer.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));

	vector<vector<double>> deliveryQuantityToWarehouse_temp(params.numWarehouses, vector<double>(params.numPeriods, 0.0));

	// Get solution values of decision variables
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			sol_SE_temp.warehouseInventory[w][t] = cplex.getValue(I_warehouse[w][t]);
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			sol_SE_temp.customerInventory[i][t] = cplex.getValue(I_customer[i][t]);
			sol_SE_temp.customerUnmetDemand[i][t] = cplex.getValue(b_customer[i][t]);
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				sol_SE_temp.deliveryQuantityToCustomer[i][t] += cplex.getValue(w_customer[i][k][t]);
			}
		}
	}

	sol_SE_temp.routesWarehouseToCustomer = sol_SE.routesWarehouseToCustomer;
	sol_SE_temp.customerAssignmentToWarehouse = sol_SE.customerAssignmentToWarehouse;
}

void LP_SE_EEV::CalculateCostsForEachPart()
{

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			sol_SE_temp.holdingCostWarehouse += params.unitHoldingCost_Warehouse[w] * sol_SE_temp.warehouseInventory[w][t];
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			sol_SE_temp.holdingCostCustomer += params.unitHoldingCost_Customer[i] * sol_SE_temp.customerInventory[i][t];
			sol_SE_temp.costOfUnmetDemand += params.unmetDemandPenalty[i] * sol_SE_temp.customerUnmetDemand[i][t];
		}
	}

	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				int previousNode = w;
				for (int j = 1; j < sol_SE_temp.routesWarehouseToCustomer[w][t][k].size(); ++j)
				{
					int currentNode = sol_SE_temp.routesWarehouseToCustomer[w][t][k][j];

					sol_SE_temp.transportationCostWarehouseToCustomer += params.transportationCost_SecondEchelon[previousNode][currentNode];

					previousNode = currentNode;
				}
			}
		}
	}

	result.objValue_firstEchelon = sol_FE_EV.setupCost + sol_FE_EV.productionCost + sol_FE_EV.holdingCostPlant + sol_FE_EV.transportationCostPlantToWarehouse;
	

	result.objValue_secondEchelon = sol_SE_temp.holdingCostWarehouse +
									sol_SE_temp.holdingCostCustomer +
									sol_SE_temp.costOfUnmetDemand +
									sol_SE_temp.transportationCostWarehouseToCustomer;

	result.objValue_Total = result.objValue_firstEchelon + result.objValue_secondEchelon;
}

void LP_SE_EEV::DisplayCosts()
{
	cout << "Setup Cost : " << sol_FE_EV.setupCost << endl;
	cout << "Production Cost : " << sol_FE_EV.productionCost << endl;
	cout << "Holding Cost Plant : " << sol_FE_EV.holdingCostPlant << endl;
	cout << "Transportation Cost Plant to Warehouse : " << sol_FE_EV.transportationCostPlantToWarehouse << endl;

	cout << "Holding Cost Warehouse : " << sol_SE_temp.holdingCostWarehouse << endl;
	cout << "Holding Cost Customer : " << sol_SE_temp.holdingCostCustomer << endl;
	cout << "Cost of Unmet Demand : " << sol_SE_temp.costOfUnmetDemand << endl;
	cout << "Transportation Cost Warehouse to Customer : " << sol_SE_temp.transportationCostWarehouseToCustomer << endl;

	cout << "\nObjective value FE : " << result.objValue_firstEchelon << endl;
	cout << "Objective value SE : " << result.objValue_secondEchelon << endl;
	cout << "Objective value Total : " << result.objValue_Total << endl;
}

void LP_SE_EEV::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_EV.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << sol_FE_EV.productionSetup[t] << endl;
		}
	}
}

void LP_SE_EEV::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_EV.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_FE_EV.productionQuantity[t] << endl;
		}
	}
}

void LP_SE_EEV::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_EV.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_FE_EV.plantInventory[t] << endl;
		}
	}
}

void LP_SE_EEV::DisplayWarehouseInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (sol_SE_temp.warehouseInventory[w][t] > THRESHOLD)
			{
				cout << "I_warehouse[" << w + 1 << "][" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_SE_temp.warehouseInventory[w][t] << endl;
			}
		}
	}
}

void LP_SE_EEV::DisplayFirstEchelonRouteVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		int routeInd = 1;
		for (const auto &route : sol_FE_EV.routesPlantToWarehouse[t])
		{
			if (!route.empty())
			{
				cout << "Period: " << t + 1 << ", Route: " << routeInd << " : ";
				for (auto it = route.begin(); it != route.end() - 1; ++it)
				{
					cout << *it << " -> ";
				}
				cout << route.back() << endl;
			}
			++routeInd;
		}
	}
}

void LP_SE_EEV::DisplayDeliveryQuantityToWarehousesVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (sol_FE_EV.deliveryQuantityToWarehouse[w][t] > THRESHOLD)
			{
				cout << "q[" << w + 1 << "][" << t + 1 << "] = " << sol_FE_EV.deliveryQuantityToWarehouse[w][t] << endl;
			}
		}
	}
}

void LP_SE_EEV::DisplayCustomerInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (sol_SE_temp.customerInventory[i][t] > THRESHOLD)
			{
				cout << "I_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "] = " << sol_SE_temp.customerInventory[i][t] << endl;
			}
		}
	}
}

void LP_SE_EEV::DisplayCustomerUnmetDemandVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (sol_SE_temp.customerUnmetDemand[i][t] > THRESHOLD)
			{
				cout << "b_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "] = " << sol_SE_temp.customerUnmetDemand[i][t] << endl;
			}
		}
	}
}

void LP_SE_EEV::DisplayDeliveryQuantityToCustomersVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (sol_SE_temp.deliveryQuantityToCustomer[i][t] > THRESHOLD)
			{
				cout << "deliveryQuantityToCustomer[" << i + params.numWarehouses << "][" << t + 1 << "] = " << sol_SE_temp.deliveryQuantityToCustomer[i][t] << endl;
			}
		}
	}
}

void LP_SE_EEV::DisplayRoutesWarehouseToCustomersVars()
{
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				if (!sol_SE_temp.routesWarehouseToCustomer[w][t][k].empty())
				{
					cout << "route[" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] : [";
					for (auto it = sol_SE_temp.routesWarehouseToCustomer[w][t][k].begin(); it != sol_SE_temp.routesWarehouseToCustomer[w][t][k].end(); ++it)
					{
						if (it != sol_SE_temp.routesWarehouseToCustomer[w][t][k].begin())
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

SolutionSecondEchelon_Deterministic LP_SE_EEV::getSolutionSE()
{
	return sol_SE_temp;
}

Result LP_SE_EEV::getResult()
{
	return result;
}