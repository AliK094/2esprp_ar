#include "LP_SE.h"

// ------------------------------------------------------------------------------------------------------------------------
LP_SE::LP_SE(const ParameterSetting &parameters,
			 const Solution &solution)
	: params(parameters),
	  sol(solution),
	  THRESHOLD(1e-2),
	  save_lpFile(true),
	  save_mpsResultFile(true)
{}

string LP_SE::solve()
{
	IloEnv env;
	IloModel model(env);
	IloCplex cplex(model);

	auto startTime = std::chrono::high_resolution_clock::now();

	DefineVariables(env, model);
	DefineObjectiveFunction(env, model);
	DefineConstraints(env, model);

	/* Assure linear mappings between the presolved and original models */
	cplex.setParam(IloCplex::Param::Threads, 4);
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

	if (save_lpFile)
	{
		string directory = "../cplexFiles/lpModel/";
		string lpFileName = directory + "LPSE_NW" + std::to_string(params.numWarehouses) + "_NC" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_Ins" + params.instance.c_str() + ".lp";

		// Export the model to an LP file
		cplex.exportModel(lpFileName.c_str());
	}

	// Extract model
	cplex.extract(model);

	cout << "Solving LP_SE..." << endl; 
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
			string solFileName = directory + "LPSE_NW" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_Ins" + params.instance.c_str();

			// Export the model to an LP file
			cplex.writeSolution(solFileName.c_str());
		}
	}
	else if (cplex.getStatus() == IloAlgorithm::Infeasible)
	{
		status = "Infeasible";
		cout << "Problem is infeasible" << endl;

		string directory = "../cplexFiles/lpModel/";
		string lpFileName = directory + "LPSE_NW" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_Ins" + params.instance.c_str() + ".lp";

		// Export the model to an LP file
		cplex.exportModel(lpFileName.c_str());
	}
	else
	{
		status = "Undefined";
		cout << "Solver terminated with status: " << status << endl;
	}

	auto currentTime = std::chrono::high_resolution_clock::now();
	auto totalCPUTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
	cout << "Timer (seconds): " << std::fixed << std::setprecision(3) << totalCPUTime << endl;

	if (status == "Optimal" || status == "Incumbent")
	{
		cout << "Retrieving solutions..." << endl;
		// Retrieve the solution
		RetrieveSolutions(cplex);
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
		CalculateCostsForEachPart();
	}

	env.end();

	return status;
}

void LP_SE::DefineVariables(IloEnv &env, IloModel &model)
{
	/* Define Decision Variables */
	// Initialize Variable Manager
	VariableManager varManager(env);
	// -------------------------------------------------------------------------------------------------------------------------------
	// p[t] variables (Production quantity in period t)
	p = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "p[" + std::to_string(t + 1) + "]";
		p[t] = IloNumVar(env, 0.0, params.prodCapacity, IloNumVar::Float, varName.c_str());
		model.add(p[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// I_plant[t] variables (Plant inventory in period t)
	I_plant = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "I_plant[" + std::to_string(t + 1) + "]";
		I_plant[t] = IloNumVar(env, 0.0, params.storageCapacity_Plant, IloNumVar::Float, varName.c_str());
		model.add(I_plant[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// I_warehouse[w][t][s] variables (Warehouse w inventory in period t under scenario s)
	I_warehouse = varManager.create3D(params.numWarehouses, params.numPeriods, params.numScenarios);
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int s = 0; s < params.numScenarios; ++s)
			{
				string varName = "I_warehouse[" + std::to_string(w + 1) + "][" + std::to_string(t + 1) + "][" + std::to_string(s + 1) + "]";
				I_warehouse[w][t][s] = IloNumVar(env, 0.0, params.storageCapacity_Warehouse[w], IloNumVar::Float, varName.c_str());
				model.add(I_warehouse[w][t][s]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// I_customer[i][t][s] variables (Customer i inventory in period t under scenario s)
	I_customer = varManager.create3D(params.numCustomers, params.numPeriods, params.numScenarios);
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int s = 0; s < params.numScenarios; ++s)
			{
				string varName = "I_customer[" + std::to_string(i + 1 + params.numWarehouses) + "][" + std::to_string(t + 1) + "][" + std::to_string(s + 1) + "]";
				I_customer[i][t][s] = IloNumVar(env, 0.0, params.storageCapacity_Customer[i], IloNumVar::Float, varName.c_str());
				model.add(I_customer[i][t][s]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// b_customer[i][t][s] variables (Customer i unsatisfied demand in period t under scenario s)
	b_customer = varManager.create3D(params.numCustomers, params.numPeriods, params.numScenarios);
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int s = 0; s < params.numScenarios; ++s)
			{
				string varName = "b_customer[" + std::to_string(i + 1 + params.numWarehouses) + "][" + std::to_string(t + 1) + "][" + std::to_string(s + 1) + "]";
				b_customer[i][t][s] = IloNumVar(env, 0.0, params.demand[i][t][s], IloNumVar::Float, varName.c_str());
				model.add(b_customer[i][t][s]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// q[w][k][t] variables (Delivery quantity to warehouse w in period t from route r)
	q = varManager.create3D(params.numWarehouses, params.numVehicles_Plant, params.numPeriods);
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int k = 0; k < params.numVehicles_Plant; ++k)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				string varName = "q[" + std::to_string(w + 1) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "]";
				q[w][k][t] = IloNumVar(env, 0.0, params.storageCapacity_Warehouse[w], IloNumVar::Float, varName.c_str());
				model.add(q[w][k][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// w_customer[i][k][t][s] variables (Delivery quantity to customer i by vehicle k in period t under scenario s)
	w_customer = varManager.create4D(params.numCustomers, params.numVehicles_SecondEchelon, params.numPeriods, params.numScenarios);
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int s = 0; s < params.numScenarios; ++s)
				{
					string varName = "w[" + std::to_string(i + 1 + params.numWarehouses) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "][" + std::to_string(s + 1) + "]";
					w_customer[i][k][t][s] = IloNumVar(env, 0.0, params.demand[i][t][s], IloNumVar::Float, varName.c_str());
					model.add(w_customer[i][k][t][s]);
				}
			}
		}
	}
}

void LP_SE::DefineObjectiveFunction(IloEnv &env, IloModel &model)
{
	// Define objective function
	IloExpr obj(env);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		obj += params.unitProdCost * p[t];
		obj += params.unitHoldingCost_Plant * I_plant[t];

		for (int s = 0; s < params.numScenarios; ++s)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				obj += params.probability[s] * params.unitHoldingCost_Warehouse[w] * I_warehouse[w][t][s];
			}

			for (int i = 0; i < params.numCustomers; ++i)
			{
				obj += params.probability[s] * params.unitHoldingCost_Customer[i] * I_customer[i][t][s];
				obj += params.probability[s] * params.unmetDemandPenalty[i] * b_customer[i][t][s];
			}
		}
	}
	model.add(IloMinimize(env, obj));
}

void LP_SE::DefineConstraints(IloEnv &env, IloModel &model)
{
	/* Define Constraints */
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Production Capacity Constraints:
			p[t] <= Capacity * y[t]			for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "ProductionCapacity(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		expr += p[t];
		expr -= params.prodCapacity * sol.productionSetup[t];
		IloConstraint productionCapacityConstraint(expr <= 0);
		expr.end();

		model.add(productionCapacityConstraint).setName(constraintName.c_str());
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Plant Inventory Capacity Constraints
			I_plant[t] <= params.storageCapacity_Plant 		for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "PlantInventoryCapacity(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		expr += I_plant[t];
		IloConstraint plantInventoryCapacityConstraint(expr <= params.storageCapacity_Plant);
		expr.end();

		model.add(plantInventoryCapacityConstraint).setName(constraintName.c_str());
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Plant Inventory Balance Constraints:
			I_plant[t] = I_plant[t-1] + p[t] - sum(k in KP) sum(w in W) q[w][k][t] 		for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "PlantInventoryBalance(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		if (t == 0)
		{
			expr += I_plant[t];
			expr -= p[t];
			for (int k = 0; k < params.numVehicles_Plant; ++k)
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					expr += q[w][k][t];
				}
			}
			IloConstraint plantInventoryBalanceConstraint(expr == params.initialInventory_Plant);
			expr.end();

			model.add(plantInventoryBalanceConstraint).setName(constraintName.c_str());
		}
		else
		{
			expr += I_plant[t];
			expr -= I_plant[t - 1];
			expr -= p[t];
			for (int k = 0; k < params.numVehicles_Plant; ++k)
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					expr += q[w][k][t];
				}
			}
			IloConstraint plantInventoryBalanceConstraint(expr == 0);
			expr.end();

			model.add(plantInventoryBalanceConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Warehouses Inventory Capacity Constraints:
			I_warehouse[w][t][s] <= storageCapacity_warehouse 		for all w in W, t in T, s in S
	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				string constraintName = "WarehouseInventoryCapacity(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				IloExpr expr(env);
				expr += I_warehouse[w][t][s];
				IloConstraint warehouseInventoryCapacityConstraint(expr <= params.storageCapacity_Warehouse[w]);
				expr.end();

				model.add(warehouseInventoryCapacityConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Warehouse Inventory Balance Constraints:
			I[w][t][s] = I[w][t-1][s] + sum(k in KP) q[w][k][t] - sum(i in N_c) sum(k in K_w) w_customer[i][k][t][s]	for all w in W, t in T, s in S
	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				string constraintName = "WarehouseInventoryBalance(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				IloExpr expr(env);
				if (t == 0)
				{
					expr += I_warehouse[w][t][s];

					for (int k = 0; k < params.numVehicles_Plant; ++k)
					{
						expr -= q[w][k][t];
					}

					for (int k : params.set_WarehouseVehicles[w])
					{
						for (int i = 0; i < params.numCustomers; ++i)
						{
							expr += w_customer[i][k][t][s];
						}
					}
					IloConstraint WarehouseInventoryBalanceConstraint(expr == params.initialInventory_Warehouse[w]);
					expr.end();

					model.add(WarehouseInventoryBalanceConstraint).setName(constraintName.c_str());
				}
				else
				{
					expr += I_warehouse[w][t][s];
					expr -= I_warehouse[w][t - 1][s];

					for (int k = 0; k < params.numVehicles_Plant; ++k)
					{
						expr -= q[w][k][t];
					}

					for (int k : params.set_WarehouseVehicles[w])
					{
						for (int i = 0; i < params.numCustomers; ++i)
						{
							expr += w_customer[i][k][t][s];
						}
					}
					IloConstraint WarehouseInventoryBalanceConstraint(expr == 0);
					expr.end();

					model.add(WarehouseInventoryBalanceConstraint).setName(constraintName.c_str());
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Customers Inventory Balance Constraints:
			I[i][t][s] = I[i][t-1][s] + sum(k in K) w_customer[i][k][t][s] - d[i][t][s] + b[i][t][s] 	for all i in N_w, t in T
	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				string constraintName = "CustomerInventoryBalance(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				IloExpr expr(env);
				if (t == 0)
				{
					expr += I_customer[i][t][s];
					for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
					{
						expr -= w_customer[i][k][t][s];
					}
					expr -= b_customer[i][t][s];
					IloConstraint CustomerInventoryBalanceConstraint(expr == params.initialInventory_Customer[i] - params.demand[i][t][s]);
					expr.end();

					model.add(CustomerInventoryBalanceConstraint).setName(constraintName.c_str());
				}
				else
				{
					expr += I_customer[i][t][s];
					expr -= I_customer[i][t - 1][s];
					for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
					{
						expr -= w_customer[i][k][t][s];
					}
					expr -= b_customer[i][t][s];
					IloConstraint CustomerInventoryBalanceConstraint(expr == -params.demand[i][t][s]);
					expr.end();

					model.add(CustomerInventoryBalanceConstraint).setName(constraintName.c_str());
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Customers Inventory Capacity Constraints:
			I[i][t][s] + d[i][t][s] <= params.storageCapacity_Customer[i] 		for all i in N_w, t in T
	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				string constraintName = "CustomerInventoryCapacity(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				IloExpr expr(env);
				expr += I_customer[i][t][s];
				IloConstraint CustomerInventoryCapacityConstraint(expr <= params.storageCapacity_Customer[i] - params.demand[i][t][s]);
				expr.end();

				model.add(CustomerInventoryCapacityConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Each Warehouse must be covered by at most one route.
			q[w][k][t] <= DeliveryUB (if visited)		for all w in w, k in KP and t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Plant; ++k)
		{
			vector<int> route = sol.routesPlantToWarehouse[t][k];
			if (!route.empty())
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					int warehouseIndex = w + 1;
					auto it = std::find(route.begin() + 1, route.end() - 1, warehouseIndex);

					string constraintName = "WarehouseDelivery(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

					IloExpr expr(env);

					expr += q[w][k][t];

					if (it != route.end() - 1)
					{
						expr -= params.storageCapacity_Warehouse[w];

						IloConstraint WarehouseDelivery(expr <= 0);
						expr.end();
					
						model.add(WarehouseDelivery).setName(constraintName.c_str());
					}
					else
					{
						IloConstraint WarehouseDelivery(expr == 0);
						expr.end();
					
						model.add(WarehouseDelivery).setName(constraintName.c_str());
					}

					
				}
			}
			else
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					string constraintName = "WarehouseDelivery(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

					IloExpr expr(env);

					expr += q[w][k][t];
					IloConstraint WarehouseDelivery(expr == 0);
					expr.end();

					model.add(WarehouseDelivery).setName(constraintName.c_str());
				}

			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Vehicle Capacity Constraints (From Plant to Warehouse):
			sum(w in W) q[w][k][t] <= params.vehicleCapacity_Plant 		for all k in KP, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_Plant; ++k)
		{
			string constraintName = "VehicleCapacityPlant(" + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				expr += q[w][k][t];
			}
			expr -= params.vehicleCapacity_Plant;
			IloConstraint vehicleCapacityConstraint(expr <= 0);
			expr.end();

			model.add(vehicleCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Customer Visit Constraints (From Warehouse to Customer):
			w_customer[i][k][t][s] <= DeliveryUB[i][t][s]		for all i in N_c, k in union K, t in T, s in S

	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					int vehicleIndex = k + (w * params.numVehicles_Warehouse);

					vector<int> route = sol.routesWarehouseToCustomer[s][w][t][k];
					for (int i = 0; i < params.numCustomers; ++i)
					{
						int customerIndex = i + params.numWarehouses;
						auto it = std::find(route.begin(), route.end(), customerIndex);

						string constraintName = "CustomerVisit(" + std::to_string(customerIndex + 1) + "," + std::to_string(vehicleIndex + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";
						IloExpr expr(env);
						expr += w_customer[i][vehicleIndex][t][s];

						if (it != route.end())
						{
							// cout << "Customer " << customerIndex << " is visited in route for scenario " << s + 1 << ", warehouse " << w + 1 << ", period " << t + 1 << ", vehicle " << k + 1 << endl;

							expr -= params.DeliveryUB_perCustomer[i][t][s];
							IloConstraint customerVisitConstraint(expr <= 0.0);							
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
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Vehicle Capacity Constraints (From Warehouse to Customer):
			sum(i in N_c) w_customer[i][k][t][s] <= Q^w		for all w in N_w, k in K_w, t in T, s in S

	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k : params.set_WarehouseVehicles[w])
				{
					string constraintName = "VehicleCapacityWarehouse(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

					IloExpr expr(env);
					for (int i = 0; i < params.numCustomers; ++i)
					{
						expr += w_customer[i][k][t][s];
					}
					expr -= params.vehicleCapacity_Warehouse;
					IloConstraint vehicleCapacityConstraint(expr <= 0);
					expr.end();

					model.add(vehicleCapacityConstraint).setName(constraintName.c_str());
				}
			}
		}
	}
	/* End of Model Constraints */
}

void LP_SE::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	sol_temp.productionSetup = sol.productionSetup;
	sol_temp.productionQuantity.assign(params.numPeriods, 0.0);
	sol_temp.plantInventory.assign(params.numPeriods, 0.0);
	sol_temp.warehouseInventory.assign(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	sol_temp.customerInventory.assign(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	sol_temp.customerUnmetDemand.assign(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	sol_temp.deliveryQuantityToWarehouse.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	sol_temp.deliveryQuantityToCustomer.assign(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));	

	vector<vector<vector<double>>> deliveryQuantityToWarehouse_temp(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));

	// Get solution values of decision variables
	for (int t = 0; t < params.numPeriods; ++t)
	{
		sol_temp.productionQuantity[t] = cplex.getValue(p[t]);
		sol_temp.plantInventory[t] = cplex.getValue(I_plant[t]);

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k = 0; k < params.numVehicles_Plant; ++k)
			{
				sol_temp.deliveryQuantityToWarehouse[w][t] += cplex.getValue(q[w][k][t]);
			}

			for (int s = 0; s < params.numScenarios; ++s)
			{
				sol_temp.warehouseInventory[w][t][s] = cplex.getValue(I_warehouse[w][t][s]);
			}
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			for (int s = 0; s < params.numScenarios; ++s)
			{
				sol_temp.customerInventory[i][t][s] = cplex.getValue(I_customer[i][t][s]);
				sol_temp.customerUnmetDemand[i][t][s] = cplex.getValue(b_customer[i][t][s]);
				for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
				{
					sol_temp.deliveryQuantityToCustomer[i][t][s] += cplex.getValue(w_customer[i][k][t][s]);
				}
			}
		}
	}

	sol_temp.routesPlantToWarehouse = sol.routesPlantToWarehouse;
	sol_temp.routesWarehouseToCustomer = sol.routesWarehouseToCustomer;
	sol_temp.customerAssignmentToWarehouse = sol.customerAssignmentToWarehouse;
}

void LP_SE::CalculateCostsForEachPart()
{
	sol_temp.setupCost = sol.setupCost;
	sol_temp.transportationCostPlantToWarehouse += sol.transportationCostPlantToWarehouse;

	for (int t = 0; t < params.numPeriods; ++t)
	{
		sol_temp.productionCost += params.unitProdCost * sol_temp.productionQuantity[t];
		sol_temp.holdingCostPlant += params.unitHoldingCost_Plant * sol_temp.plantInventory[t];

		for (int s = 0; s < params.numScenarios; ++s)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				sol_temp.holdingCostWarehouse_Avg += params.probability[s] * params.unitHoldingCost_Warehouse[w] * sol_temp.warehouseInventory[w][t][s];
			}

			for (int i = 0; i < params.numCustomers; ++i)
			{
				sol_temp.holdingCostCustomer_Avg += params.probability[s] * params.unitHoldingCost_Customer[i] * sol_temp.customerInventory[i][t][s];
				sol_temp.costOfUnmetDemand_Avg += params.probability[s] * params.unmetDemandPenalty[i] * sol_temp.customerUnmetDemand[i][t][s];
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
					int previousNode = w;
					for (int j = 1; j < sol_temp.routesWarehouseToCustomer[s][w][t][k].size(); ++j)
					{
						int currentNode = sol_temp.routesWarehouseToCustomer[s][w][t][k][j];

						sol_temp.transportationCostWarehouseToCustomer_Avg += params.probability[s] * params.transportationCost_SecondEchelon[previousNode][currentNode];

						previousNode = currentNode;
					}
				}
			}
		}
	}

	double objValue_FE = sol_temp.setupCost + sol_temp.productionCost + sol_temp.holdingCostPlant + sol_temp.transportationCostPlantToWarehouse;
	cout << "Setup Cost : " << sol_temp.setupCost << endl;
	cout << "Production Cost : " << sol_temp.productionCost << endl;
	cout << "Holding Cost Plant : " << sol_temp.holdingCostPlant << endl;
	cout << "Transportation Cost Plant to Warehouse : " << sol_temp.transportationCostPlantToWarehouse << endl;

	double objValue_SE = sol_temp.holdingCostWarehouse_Avg + sol_temp.holdingCostCustomer_Avg + sol_temp.costOfUnmetDemand_Avg + sol_temp.transportationCostWarehouseToCustomer_Avg;
	cout << "Holding Cost Warehouse : " << sol_temp.holdingCostWarehouse_Avg << endl;
	cout << "Holding Cost Customer : " << sol_temp.holdingCostCustomer_Avg << endl;
	cout << "Cost of Unmet Demand : " << sol_temp.costOfUnmetDemand_Avg << endl;
	cout << "Transportation Cost Warehouse to Customer : " << sol_temp.transportationCostWarehouseToCustomer_Avg << endl;
	
	sol_temp.totalObjValue = objValue_FE + objValue_SE;

	cout << "\nObjective value FE : " << objValue_FE << endl;
	cout << "Objective value (ILS) SE : " << objValue_SE << endl;
	cout << "Objective value (ILS) Total : " << sol_temp.totalObjValue << endl;
}

void LP_SE::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_temp.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << sol_temp.productionSetup[t] << endl;
		}
	}
}

void LP_SE::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_temp.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_temp.productionQuantity[t] << endl;
		}
	}
}

void LP_SE::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_temp.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_temp.plantInventory[t] << endl;
		}
	}
}

void LP_SE::DisplayWarehouseInventoryVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				if (sol_temp.warehouseInventory[w][t][s] > THRESHOLD)
				{
					cout << "I_warehouse[" << w + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << std::setprecision(0) << std::fixed << sol_temp.warehouseInventory[w][t][s] << endl;
				}
			}
		}
	}
}

void LP_SE::DisplayFirstEchelonRouteVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		int routeInd = 1;
		for (const auto &route : sol_temp.routesPlantToWarehouse[t])
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

void LP_SE::DisplayDeliveryQuantityToWarehousesVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (sol_temp.deliveryQuantityToWarehouse[w][t] > THRESHOLD)
			{
				cout << "q[" << w + 1 << "][" << t + 1 << "] = " << sol_temp.deliveryQuantityToWarehouse[w][t] << endl;
			}
		}
	}
}

void LP_SE::DisplayCustomerInventoryVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (sol_temp.customerInventory[i][t][s] > THRESHOLD)
				{
					cout << "I_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << sol_temp.customerInventory[i][t][s] << endl;
				}
			}
		}
	}
}

void LP_SE::DisplayCustomerUnmetDemandVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (sol_temp.customerUnmetDemand[i][t][s] > THRESHOLD)
				{
					cout << "b_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << sol_temp.customerUnmetDemand[i][t][s] << endl;
				}
			}
		}
	}
}

void LP_SE::DisplayDeliveryQuantityToCustomersVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (sol_temp.deliveryQuantityToCustomer[i][t][s] > THRESHOLD)
				{
					cout << "deliveryQuantityToCustomer[" << i + params.numWarehouses << "][" << t + 1 << "][" << s + 1 << "] = " << sol_temp.deliveryQuantityToCustomer[i][t][s] << endl;
				}
			}
		}
	}
}

void LP_SE::DisplayRoutesWarehouseToCustomersVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
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

Solution LP_SE::getSolution()
{
	return sol_temp;
}