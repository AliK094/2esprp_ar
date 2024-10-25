#include "LP_SE_Scenario.h"

// ------------------------------------------------------------------------------------------------------------------------
LP_SE_Scenario::LP_SE_Scenario(const ParameterSetting &parameters,
			 const SolutionFirstEchelon &sol_FE, 
			 const ScenarioSolutionSecondEchelon &sol_SE_scenario,
			 int s)
	: params(parameters),
	  sol_FE(sol_FE),
	  sol_SE_scenario(sol_SE_scenario),
	  scenario(s),
	  THRESHOLD(1e-2),
	  save_lpFile(false),
	  save_mpsResultFile(false)
{}

string LP_SE_Scenario::solve()
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
		string lpFileName = directory + "LPSE_NW" + std::to_string(params.numWarehouses) + "_NC" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_Ins" + params.instance.c_str() + ".lp";

		// Export the model to an LP file
		cplex.exportModel(lpFileName.c_str());
	}

	// Extract model
	cplex.extract(model);

	// cout << "Solving LP_SE_Scenario...\n" << endl; 
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
			string solFileName = directory + "LPSE_NW" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_Ins" + params.instance.c_str();

			// Export the model to an LP file
			cplex.writeSolution(solFileName.c_str());
		}
	}
	else if (cplex.getStatus() == IloAlgorithm::Infeasible)
	{
		status = "Infeasible";
		// cout << "Problem is infeasible" << endl;

		string directory = "../cplexFiles/lpModel/";
		string lpFileName = directory + "LPSE_NW" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_Ins" + params.instance.c_str() + ".lp";

		// Export the model to an LP file
		cplex.exportModel(lpFileName.c_str());
	}
	else
	{
		status = "Undefined";
		// cout << "Solver terminated with status: " << status << endl;
	}

	auto currentTime = std::chrono::high_resolution_clock::now();
	auto totalCPUTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
	// cout << "Timer (seconds): " << std::fixed << std::setprecision(3) << totalCPUTime << endl;

	if (status == "Optimal" || status == "Incumbent")
	{
		// cout << "Retrieving solutions..." << endl;
		// Retrieve the solution
		RetrieveSolutions(cplex);
		CalculateObjValue_Scenario();
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
		
	}

	env.end();

	return status;
}

void LP_SE_Scenario::DefineVariables(IloEnv &env, IloModel &model)
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

	for (int t = 0; t < params.numPeriods; ++t)
	{
		IloExpr expr(env);
		expr += p[t];
		IloConstraint prodQuantFix(expr == sol_FE.productionQuantity[t]);
		expr.end();
		model.add(prodQuantFix);
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

	for (int t = 0; t < params.numPeriods; ++t)
	{
		IloExpr expr(env);
		expr += I_plant[t];
		IloConstraint plantInvFix(expr == sol_FE.plantInventory[t]);
		expr.end();
		model.add(plantInvFix);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// I_warehouse[w][t]([s]) variables (Warehouse w inventory in period t under scenario s)
	I_warehouse = varManager.create2D(params.numWarehouses, params.numPeriods);
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "I_warehouse[" + std::to_string(w + 1) + "][" + std::to_string(t + 1) + "][" + std::to_string(scenario + 1) + "]";
			I_warehouse[w][t] = IloNumVar(env, 0.0, params.storageCapacity_Warehouse[w], IloNumVar::Float, varName.c_str());
			model.add(I_warehouse[w][t]);
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// I_customer[i][t]([s]) variables (Customer i inventory in period t under scenario s)
	I_customer = varManager.create2D(params.numCustomers, params.numPeriods);
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "I_customer[" + std::to_string(i + 1 + params.numWarehouses) + "][" + std::to_string(t + 1) + "][" + std::to_string(scenario + 1) + "]";
			I_customer[i][t] = IloNumVar(env, 0.0, params.storageCapacity_Customer[i], IloNumVar::Float, varName.c_str());
			model.add(I_customer[i][t]);
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// b_customer[i][t]([s]) variables (Customer i unsatisfied demand in period t under scenario s)
	b_customer = varManager.create2D(params.numCustomers, params.numPeriods);
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "b_customer[" + std::to_string(i + 1 + params.numWarehouses) + "][" + std::to_string(t + 1) + "][" + std::to_string(scenario + 1) + "]";
			b_customer[i][t] = IloNumVar(env, 0.0, params.demand[i][t][scenario], IloNumVar::Float, varName.c_str());
			model.add(b_customer[i][t]);
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

	// for (int k = 0; k < params.numVehicles_Plant; ++k)
	// {
	// 	for (int t = 0; t < params.numPeriods; ++t)
	// 	{
	// 		vector<int>& route = sol_FE.routesPlantToWarehouse[t][k];
	// 		if (!route.empty())
	// 		{
	// 			for (int w = 0; w < params.numWarehouses; ++w)
	// 			{
	// 				int warehouseIndex = w + 1;
	// 				auto it = std::find(route.begin() + 1, route.end() - 1, warehouseIndex);
	// 				IloExpr expr(env);
	// 				expr += q[w][k][t];
	// 				if (it != route.end() - 1)
	// 				{
	// 					IloConstraint delToWareFix(expr == sol_FE.deliveryQuantityToWarehouse[w][t]);
	// 					expr.end();
	// 					model.add(delToWareFix);
	// 				}
	// 				else
	// 				{
	// 					IloConstraint delToWareFix(expr == 0);
	// 					expr.end();
	// 					model.add(delToWareFix);
	// 				}
	// 			}
	// 		}
	// 		for (int w = 0; w < params.numWarehouses; ++w)
	// 		{
	// 			IloExpr expr(env);
	// 			expr += q[w][k][t];
	// 			IloConstraint delToWareFix(expr == 0);
	// 			expr.end();
	// 			model.add(delToWareFix);
	// 		}
	// 	}
	// }

	// -------------------------------------------------------------------------------------------------------------------------------
	// w_customer[i][k][t]([s]) variables (Delivery quantity to customer i by vehicle k in period t under scenario s)
	w_customer = varManager.create3D(params.numCustomers, params.numVehicles_SecondEchelon, params.numPeriods);
	for (int i = 0; i < params.numCustomers; ++i)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				string varName = "w[" + std::to_string(i + 1 + params.numWarehouses) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "][" + std::to_string(scenario + 1) + "]";
				w_customer[i][k][t] = IloNumVar(env, 0.0, params.demand[i][t][scenario], IloNumVar::Float, varName.c_str());
				model.add(w_customer[i][k][t]);
			}
		}
	}
}

void LP_SE_Scenario::DefineObjectiveFunction(IloEnv &env, IloModel &model)
{
	// Define objective function
	IloExpr obj(env);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		obj += params.unitProdCost * p[t];
		obj += params.unitHoldingCost_Plant * I_plant[t];

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

void LP_SE_Scenario::DefineConstraints(IloEnv &env, IloModel &model)
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
		expr -= params.prodCapacity * sol_FE.productionSetup[t];
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
			I_warehouse[w][t]([s]) <= storageCapacity_warehouse 		for all w in W, t in T, s in S
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			string constraintName = "WarehouseInventoryCapacity(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

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
			I[w][t]([s]) = I[w][t-1]([s]) + sum(k in KP) q[w][k][t] - sum(i in N_c) sum(k in K_w) w_customer[i][k][t]([s])	for all w in W, t in T, s in S
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			string constraintName = "WarehouseInventoryBalance(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			if (t == 0)
			{
				expr += I_warehouse[w][t];

				for (int k = 0; k < params.numVehicles_Plant; ++k)
				{
					expr -= q[w][k][t];
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
					expr -= q[w][k][t];
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
			I[i][t]([s]) = I[i][t-1]([s]) + sum(k in K) w_customer[i][k][t]([s]) - d[i][t]([s]) + b[i][t]([s]) 	for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			string constraintName = "CustomerInventoryBalance(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			if (t == 0)
			{
				expr += I_customer[i][t];
				for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
				{
					expr -= w_customer[i][k][t];
				}
				expr -= b_customer[i][t];
				IloConstraint CustomerInventoryBalanceConstraint(expr == params.initialInventory_Customer[i] - params.demand[i][t][scenario]);
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
				IloConstraint CustomerInventoryBalanceConstraint(expr == -params.demand[i][t][scenario]);
				expr.end();

				model.add(CustomerInventoryBalanceConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Customers Inventory Capacity Constraints:
			I[i][t]([s]) + d[i][t]([s]) <= params.storageCapacity_Customer[i] 		for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			string constraintName = "CustomerInventoryCapacity(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			expr += I_customer[i][t];
			IloConstraint CustomerInventoryCapacityConstraint(expr <= params.storageCapacity_Customer[i] - params.demand[i][t][scenario]);
			expr.end();

			model.add(CustomerInventoryCapacityConstraint).setName(constraintName.c_str());
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
			vector<int> route = sol_FE.routesPlantToWarehouse[t][k];
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
			w_customer[i][k][t]([s]) <= DeliveryUB[i][t]([s])		for all i in N_c, k in union K, t in T, s in S

	*/
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				int vehicleIndex = k + (w * params.numVehicles_Warehouse);

				vector<int> route = sol_SE_scenario.routesWarehouseToCustomer_Scenario[w][t][k];
				for (int i = 0; i < params.numCustomers; ++i)
				{
					int customerIndex = i + params.numWarehouses;
					auto it = std::find(route.begin(), route.end(), customerIndex);

					string constraintName = "CustomerVisit(" + std::to_string(customerIndex + 1) + "," + std::to_string(vehicleIndex + 1) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";
					IloExpr expr(env);
					expr += w_customer[i][vehicleIndex][t];

					if (it != route.end())
					{
						// cout << "Customer " << customerIndex << " is visited in route for scenario " << s + 1 << ", warehouse " << w + 1 << ", period " << t + 1 << ", vehicle " << k + 1 << endl;

						expr -= params.DeliveryUB_perCustomer[i][t][scenario];
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
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Vehicle Capacity Constraints (From Warehouse to Customer):
			sum(i in N_c) w_customer[i][k][t]([s]) <= Q^w		for all w in N_w, k in K_w, t in T, s in S

	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k : params.set_WarehouseVehicles[w])
			{
				string constraintName = "VehicleCapacityWarehouse(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

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

void LP_SE_Scenario::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	sol_FE_scenario_temp.productionSetup = sol_FE.productionSetup;
	sol_FE_scenario_temp.productionQuantity.assign(params.numPeriods, 0.0);
	sol_FE_scenario_temp.plantInventory.assign(params.numPeriods, 0.0);
	sol_FE_scenario_temp.deliveryQuantityToWarehouse.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));

	sol_SE_scenario_temp.warehouseInventory_Scenario.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	sol_SE_scenario_temp.customerInventory_Scenario.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	sol_SE_scenario_temp.customerUnmetDemand_Scenario.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	sol_SE_scenario_temp.deliveryQuantityToCustomer_Scenario.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));	

	vector<vector<double>> deliveryQuantityToWarehouse_temp(params.numWarehouses, vector<double>(params.numPeriods, 0.0));

	// Get solution values of decision variables
	for (int t = 0; t < params.numPeriods; ++t)
	{
		sol_FE_scenario_temp.productionQuantity[t] = cplex.getValue(p[t]);
		sol_FE_scenario_temp.plantInventory[t] = cplex.getValue(I_plant[t]);
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k = 0; k < params.numVehicles_Plant; ++k)
			{
				sol_FE_scenario_temp.deliveryQuantityToWarehouse[w][t] += cplex.getValue(q[w][k][t]);
			}
		}

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			sol_SE_scenario_temp.warehouseInventory_Scenario[w][t] = cplex.getValue(I_warehouse[w][t]);
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			sol_SE_scenario_temp.customerInventory_Scenario[i][t] = cplex.getValue(I_customer[i][t]);
			sol_SE_scenario_temp.customerUnmetDemand_Scenario[i][t] = cplex.getValue(b_customer[i][t]);
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				sol_SE_scenario_temp.deliveryQuantityToCustomer_Scenario[i][t] += cplex.getValue(w_customer[i][k][t]);
			}
		}
	}

	sol_FE_scenario_temp.routesPlantToWarehouse = sol_FE.routesPlantToWarehouse;
	sol_SE_scenario_temp.routesWarehouseToCustomer_Scenario = sol_SE_scenario.routesWarehouseToCustomer_Scenario;
	sol_SE_scenario_temp.customerAssignmentToWarehouse_Scenario = sol_SE_scenario.customerAssignmentToWarehouse_Scenario;
}

void LP_SE_Scenario::CalculateObjValue_Scenario()
{
	sol_FE_scenario_temp.setupCost += sol_FE.setupCost;
	sol_FE_scenario_temp.transportationCostPlantToWarehouse += sol_FE.transportationCostPlantToWarehouse;

	for (int t = 0; t < params.numPeriods; ++t)
	{
		sol_FE_scenario_temp.productionCost += params.unitProdCost * sol_FE_scenario_temp.productionQuantity[t];
		sol_FE_scenario_temp.holdingCostPlant += params.unitHoldingCost_Plant * sol_FE_scenario_temp.plantInventory[t];

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			sol_SE_scenario_temp.holdingCostWarehouse_Scenario += params.unitHoldingCost_Warehouse[w] * sol_SE_scenario_temp.warehouseInventory_Scenario[w][t];
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			sol_SE_scenario_temp.holdingCostCustomer_Scenario += params.unitHoldingCost_Customer[i] * sol_SE_scenario_temp.customerInventory_Scenario[i][t];
			sol_SE_scenario_temp.costOfUnmetDemand_Scenario += params.unmetDemandPenalty[i] * sol_SE_scenario_temp.customerUnmetDemand_Scenario[i][t];
		}
	}

	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				int previousNode = w;
				for (int j = 1; j < sol_SE_scenario_temp.routesWarehouseToCustomer_Scenario[w][t][k].size(); ++j)
				{
					int currentNode = sol_SE_scenario_temp.routesWarehouseToCustomer_Scenario[w][t][k][j];

					sol_SE_scenario_temp.transportationCostWarehouseToCustomer_Scenario += params.transportationCost_SecondEchelon[previousNode][currentNode];

					previousNode = currentNode;
				}
			}
		}
	}

	double objValue_FE = sol_FE_scenario_temp.setupCost + sol_FE_scenario_temp.productionCost + sol_FE_scenario_temp.holdingCostPlant + sol_FE_scenario_temp.transportationCostPlantToWarehouse;
	// cout << "Setup Cost : " << sol_FE_scenario_temp.setupCost << endl;
	// cout << "Production Cost : " << sol_FE_scenario_temp.productionCost << endl;
	// cout << "Holding Cost Plant : " << sol_FE_scenario_temp.holdingCostPlant << endl;
	// cout << "Transportation Cost Plant to Warehouse : " << sol_FE_scenario_temp.transportationCostPlantToWarehouse << endl;

	double objValue_SE = sol_SE_scenario_temp.holdingCostWarehouse_Scenario + sol_SE_scenario_temp.holdingCostCustomer_Scenario + sol_SE_scenario_temp.costOfUnmetDemand_Scenario + sol_SE_scenario_temp.transportationCostWarehouseToCustomer_Scenario;
	// cout << "Holding Cost Warehouse : " << sol_SE_scenario_temp.holdingCostWarehouse_Scenario << endl;
	// cout << "Holding Cost Customer : " << sol_SE_scenario_temp.holdingCostCustomer_Scenario << endl;
	// cout << "Cost of Unmet Demand : " << sol_SE_scenario_temp.costOfUnmetDemand_Scenario << endl;
	// cout << "Transportation Cost Warehouse to Customer : " << sol_SE_scenario_temp.transportationCostWarehouseToCustomer_Scenario << endl;
	
	objVal_Scenario = objValue_FE + objValue_SE;

	// cout << "\nObjective value FE : " << objValue_FE << endl;
	// cout << "Objective value (ILS) SE : " << objValue_SE << endl;
	// cout << "Objective value (ILS) Total : " << objVal_Scenario << endl;
}

void LP_SE_Scenario::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_scenario_temp.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << sol_FE_scenario_temp.productionSetup[t] << endl;
		}
	}
}

void LP_SE_Scenario::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_scenario_temp.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_FE_scenario_temp.productionQuantity[t] << endl;
		}
	}
}

void LP_SE_Scenario::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_scenario_temp.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_FE_scenario_temp.plantInventory[t] << endl;
		}
	}
}

void LP_SE_Scenario::DisplayWarehouseInventoryVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				if (sol_SE_scenario_temp.warehouseInventory_Scenario[w][t] > THRESHOLD)
				{
					cout << "I_warehouse[" << w + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << std::setprecision(0) << std::fixed << sol_SE_scenario_temp.warehouseInventory_Scenario[w][t] << endl;
				}
			}
		}
	}
}

void LP_SE_Scenario::DisplayFirstEchelonRouteVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		int routeInd = 1;
		for (const auto &route : sol_FE_scenario_temp.routesPlantToWarehouse[t])
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

void LP_SE_Scenario::DisplayDeliveryQuantityToWarehousesVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (sol_FE_scenario_temp.deliveryQuantityToWarehouse[w][t] > THRESHOLD)
			{
				cout << "q[" << w + 1 << "][" << t + 1 << "] = " << sol_FE_scenario_temp.deliveryQuantityToWarehouse[w][t] << endl;
			}
		}
	}
}

void LP_SE_Scenario::DisplayCustomerInventoryVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (sol_SE_scenario_temp.customerInventory_Scenario[i][t] > THRESHOLD)
				{
					cout << "I_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << sol_SE_scenario_temp.customerInventory_Scenario[i][t] << endl;
				}
			}
		}
	}
}

void LP_SE_Scenario::DisplayCustomerUnmetDemandVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (sol_SE_scenario_temp.customerUnmetDemand_Scenario[i][t] > THRESHOLD)
				{
					cout << "b_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << sol_SE_scenario_temp.customerUnmetDemand_Scenario[i][t] << endl;
				}
			}
		}
	}
}

void LP_SE_Scenario::DisplayDeliveryQuantityToCustomersVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (sol_SE_scenario_temp.deliveryQuantityToCustomer_Scenario[i][t] > THRESHOLD)
				{
					cout << "deliveryQuantityToCustomer[" << i + params.numWarehouses << "][" << t + 1 << "][" << s + 1 << "] = " << sol_SE_scenario_temp.deliveryQuantityToCustomer_Scenario[i][t] << endl;
				}
			}
		}
	}
}

void LP_SE_Scenario::DisplayRoutesWarehouseToCustomersVars()
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
					for (auto it = sol_SE_scenario_temp.routesWarehouseToCustomer_Scenario[w][t][k].begin(); it != sol_SE_scenario_temp.routesWarehouseToCustomer_Scenario[w][t][k].end(); ++it)
					{
						if (it != sol_SE_scenario_temp.routesWarehouseToCustomer_Scenario[w][t][k].begin())
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

ScenarioSolutionSecondEchelon LP_SE_Scenario::getSolutionSE_Scenario()
{
	return sol_SE_scenario_temp;
}


double LP_SE_Scenario::getObjVal_Scenario()
{
	return objVal_Scenario;
}