#include "deterministic/BC_EEV.h"

BC_EEV::BC_EEV(const ParameterSetting &parameters,
				const SolutionFirstEchelon &sol_FE,
				const vector<vector<double>> &deterministicDemand,
				const SolutionWarmStart_Deterministic &warmStartSol)
	: params(parameters),
	  sol_FE_EV(sol_FE),
	  demand(deterministicDemand),
	  warmStart(warmStartSol),
	  THRESHOLD(1e-2),
	  save_lpFile(false),
	  save_mpsResultFile(false)
{}

bool BC_EEV::Solve()
{
	try
	{
		IloEnv env;
		IloModel model(env);
		IloCplex cplex(model);

		auto startTime = std::chrono::high_resolution_clock::now();

		// Set CPLEX Parameters: (DISPLAY LEVEL(0,1,2,3,4), OPTIMALITY GAP, RUN TIME (SECS), THREADS, MEMORY (MB))
		CplexParameterManager parameterManager(cplex);
		parameterManager.setParameters(4, 1e-6, 300, 4, 32000);
		cplex.setParam(IloCplex::Param::Emphasis::MIP, 2);
		cplex.setParam(IloCplex::Param::Conflict::Display, 2);

		DefineVariables(env, model);
		DefineObjectiveFunction(env, model);
		DefineConstraints(env, model);

		/* Assure linear mappings between the presolved and original models */
		cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
		/* Turn on traditional search for use with control callbacks */
		cplex.setParam(IloCplex::Param::MIP::Strategy::Search, CPX_MIPSEARCH_TRADITIONAL);
		/* Let MIP callbacks work on the original model */
		cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);

		cout << "Solving BC_EEV..." << endl;
		SEC_2EPRP LegacyCallback(env, params, x, z);
		cplex.use(&LegacyCallback);

		if (save_lpFile)
		{
			string directory = "../cplexFiles/lpModel/";
			string lpFileName;
			lpFileName = directory + "BC_EEV_NW" + std::to_string(params.numWarehouses) +
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

		// Start From a Warm Start Solution if any is given
		cplex.setParam(IloCplex::Param::MIP::Limits::RepairTries, 1e6);
		cplex.setParam(IloCplex::Param::Advance, 1);
		// Solution warmstart;
		// const string solFrom = "Hybrid-ILS";
		if (!warmStart.empty())
		{
			DefineWarmStartSolution(env, cplex);
		}
		else
		{
			cout << "Couldn't Find A Solution For Warmstart" << endl;
		}
		// DefineValidInequalities(env, model);

		// Solve the model
		cplex.solve();

		if (cplex.getStatus() == IloAlgorithm::Optimal)
		{
			result.status = "Optimal";
			result.objValue_Total = cplex.getObjValue();
			cout << "Optimal solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue_Total << endl;
			result.optimalityGap = cplex.getMIPRelativeGap() * 100;
			result.lowerBound = cplex.getBestObjValue();

			if (save_mpsResultFile)
			{
				string directory = "../cplexFiles/solVal/";
				string solFileName;
				solFileName = directory + "BC_EEV_NW" + std::to_string(params.numWarehouses) +
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
		else if (cplex.getStatus() == IloAlgorithm::Feasible)
		{
			result.status = "Incumbent";
			result.objValue_Total = cplex.getObjValue();
			cout << "Incumbent solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue_Total << endl;
			result.optimalityGap = cplex.getMIPRelativeGap() * 100;
			cout << "Optimality gap: " << result.optimalityGap << "%" << endl;
			result.lowerBound = cplex.getBestObjValue();
			cout << "Lower bound: " << result.lowerBound << endl;

			if (save_mpsResultFile)
			{
				string directory = "../cplexFiles/solVal/";
				string solFileName;
				solFileName = directory + "BC_EEV_NW" + std::to_string(params.numWarehouses) +
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

			result.status = "Infeasible";
			cout << "Problem is infeasible" << endl;
			cout << "Starting Conflict refinement..." << endl;

			IloConstraintArray conflictConstraints(env);
			cplex.getConflict(conflictConstraints);

			IloCplex::ConflictStatusArray conflictStatus(env);
			for (int i = 0; i < conflictConstraints.getSize(); i++)
			{
				conflictStatus.add(cplex.getConflict(conflictConstraints[i]));
			}

			cout << "Conflict constraints:" << endl;
			cout << conflictConstraints << endl;

			cout << "Conflict status:" << endl;
			cout << conflictStatus << endl;

			throw std::runtime_error("Solver terminated with infeasible solution.");
		}
		else
		{
			result.status = "Undefined";
			cout << "Solver terminated with status: " << result.status << endl;
		}

		auto currentTime = std::chrono::high_resolution_clock::now();
		result.totalCPUTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Timer (seconds): " << std::fixed << std::setprecision(4) << result.totalCPUTime << endl;

		if (result.status == "Optimal" || result.status == "Incumbent")
		{
			// Retrieve the solution
			RetrieveSolutions(cplex);
			CalculateCostsForEachPart();
			// Display the solution
			DisplayProductionSetupVars();
			DisplayProductionQuantVars();
			// DisplayPlantInventoryVars();
			// DisplayWarehouseInventoryVars();
			// DisplayFirstEchelonRouteVars();
			// DisplayDeliveryQuantityToWarehousesVars();
			// DisplayCustomerInventoryVars();
			// DisplayCustomerUnmetDemandVars();
			// DisplayDeliveryQuantityToCustomersVars();
			// DisplayRoutesWarehouseToCustomersVars();
		}

		env.end();
	}
	catch (const IloException &e)
	{
		cerr << "Error: " << e << endl;
		return false;
	}
	catch (const std::runtime_error &e)
	{
		cerr << "Runtime Error: " << e.what() << endl;
		return false;
	}
	return true;
}

void BC_EEV::DefineVariables(IloEnv &env, IloModel &model)
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
	// z[i][k][t] variables (Binary variables to indicate whether node i (in Second Echelon) is visited by vehicle k in period t)
	z = varManager.create3D(params.numNodes_SecondEchelon, params.numVehicles_SecondEchelon, params.numPeriods);
	for (int i = 0; i < params.numNodes_SecondEchelon; ++i)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				string varName = "z[" + std::to_string(i + 1) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "]";
				z[i][k][t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
				model.add(z[i][k][t]);
			}
		}
	}

	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			if (std::find(params.set_WarehouseVehicles[w].begin(), params.set_WarehouseVehicles[w].end(), k) == params.set_WarehouseVehicles[w].end())
			{
				for (int t = 0; t < params.numPeriods; ++t)
				{
					model.add(z[w][k][t] == 0);
				}
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// x[i][j][k][t] variables (Binary variables to indicate whether edge (i, j) is traversed by vehicle k in period t)
	x = varManager.create3D(params.numEdges_SecondEchelon, params.numVehicles_SecondEchelon, params.numPeriods);
	for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				if (params.index_i_SecondEchelon[e] < params.numWarehouses || params.index_j_SecondEchelon[e] < params.numWarehouses)
				{
					string varName = "x[" + std::to_string(params.index_i_SecondEchelon[e] + 1) + "][" + std::to_string(params.index_j_SecondEchelon[e] + 1) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "]";
					x[e][k][t] = IloNumVar(env, 0.0, 2.0, IloNumVar::Int, varName.c_str());
					model.add(x[e][k][t]);
				}
				else
				{
					string varName = "x[" + std::to_string(params.index_i_SecondEchelon[e] + 1) + "][" + std::to_string(params.index_j_SecondEchelon[e] + 1) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "]";
					x[e][k][t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
					model.add(x[e][k][t]);
				}
			}
		}
	}

	for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
	{
		if ((params.index_i_SecondEchelon[e] < params.numWarehouses) && (params.index_j_SecondEchelon[e] < params.numWarehouses))
		{
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				for (int t = 0; t < params.numPeriods; ++t)
				{
					model.add(x[e][k][t] == 0);
				}
			}
		}
	}

	for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
	{
		if (params.index_i_SecondEchelon[e] < params.numWarehouses)
		{
			int warehouse = params.index_i_SecondEchelon[e];
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				if (std::find(params.set_WarehouseVehicles[warehouse].begin(), params.set_WarehouseVehicles[warehouse].end(), k) == params.set_WarehouseVehicles[warehouse].end())
				{
					for (int t = 0; t < params.numPeriods; ++t)
					{
						model.add(x[e][k][t] == 0);
					}
				}
			}
		}
		else if (params.index_j_SecondEchelon[e] < params.numWarehouses)
		{
			int warehouse = params.index_j_SecondEchelon[e];
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				if (std::find(params.set_WarehouseVehicles[warehouse].begin(), params.set_WarehouseVehicles[warehouse].end(), k) == params.set_WarehouseVehicles[warehouse].end())
				{
					for (int t = 0; t < params.numPeriods; ++t)
					{
						model.add(x[e][k][t] == 0);
					}
				}
			}
		}
	}
}

void BC_EEV::DefineObjectiveFunction(IloEnv &env, IloModel &model)
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

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k : params.set_WarehouseVehicles[w])
			{
				for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
				{

					obj += params.transportationCost_SecondEchelon[params.index_i_SecondEchelon[e]][params.index_j_SecondEchelon[e]] * x[e][k][t];
				}
			}
		}
	}
	model.add(IloMinimize(env, obj));
}

void BC_EEV::DefineConstraints(IloEnv &env, IloModel &model)
{
	/* Define Constraints */
	// ---------------------------------------------------------------------------------------------------------------------------
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
			I[w][t] = I[w][t-1] + sum(r in R) q[w][r][t] - sum(i in N_c) sum(k in K_w) w_customer[i][k][t]	for all w in W, t in T
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
			w_customer[i][k][t] <= DeliveryUB[i][t] * z[i][k][t] 		for all i in N_c, k in union K, t in T

	*/
	vector<vector<double>> Del_UB(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			Del_UB[i][t] = std::min({demand[i][t], params.vehicleCapacity_Warehouse, params.storageCapacity_Customer[i]});
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				string constraintName = "CustomerVisit(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

				IloExpr expr(env);
				expr += w_customer[i][k][t];
				expr -= Del_UB[i][t] * z[i + params.numWarehouses][k][t];
				IloConstraint customerVisitConstraint(expr <= 0);
				expr.end();

				model.add(customerVisitConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Vehicle Capacity Constraints (From Warehouse to Customer):
			sum(i in N_c) w_customer[i][k][t] <= Q^w * z[w][k][t] 		for all w in N_w, k in K_w, t in T

	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k : params.set_WarehouseVehicles[w])
			{
				string constraintName = "VehicleCapacity(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

				IloExpr expr(env);
				for (int i = 0; i < params.numCustomers; ++i)
				{
					expr += w_customer[i][k][t];
				}
				expr -= params.vehicleCapacity_Warehouse * z[w][k][t];
				IloConstraint vehicleCapacityConstraint(expr <= 0);
				expr.end();

				model.add(vehicleCapacityConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Prevent Split Deliveries:
			sum(k in union K) z[i][k][t] <= 1 		for all i in N_c, k in union K, t in T

	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			string constraintName = "PreventSplitDeliveries(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				expr += z[i + params.numWarehouses][k][t];
			}
			IloConstraint preventSplitDeliveriesConstraint(expr <= 1);
			expr.end();

			model.add(preventSplitDeliveriesConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Degree Constraints:
			sum((j,j') in incident (i)) x[j][j'][k][t] == 2 * z[i][k][t] 		for all i in N_w + N_c, k in union K, t in T

	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int i = 0; i < params.numNodes_SecondEchelon; ++i)
			{
				string constraintName = "DegreeConstraint(" + std::to_string(i + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

				IloExpr expr(env);
				for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
				{
					if (params.index_i_SecondEchelon[e] == i || params.index_j_SecondEchelon[e] == i)
					{
						expr += x[e][k][t];
					}
				}
				expr -= 2 * z[i][k][t];
				IloConstraint degreeConstraint(expr == 0);
				expr.end();

				model.add(degreeConstraint).setName(constraintName.c_str());
			}
		}
	}
	/* End of Model Constraints */
}

void BC_EEV::DefineValidInequalities(IloEnv &env, IloModel &model)
{
	// ---------------------------------------------------------------------------------------------------------------------------
	/* Define Valid Inequalities */
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Logical Inequality (#1):
			i can be visited by vehicle k iff k leaves the warehouse w
			z[i][k][t] <= z[w][k][t] for all i in N_c, w in N_w, k in K_w t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k : params.set_WarehouseVehicles[w])
			{
				for (int i = 0; i < params.numCustomers; ++i)
				{
					string constraintName = "LogicalInequality_One(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

					IloExpr expr(env);
					expr += z[i + params.numWarehouses][k][t];
					expr -= z[w][k][t];
					IloConstraint logicalInequality_One(expr <= 0);
					expr.end();

					model.add(logicalInequality_One).setName(constraintName.c_str());
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Logical Inequality (#2):
		x[i][j][k][t] <= z[i][k][t] for all i in N_w + N_c, j in N_w + N_c, k in K_w, t in T
		x[i][j][k][t] <= z[j][k][t] for all i in N_w + N_c, j in N_w + N_c, k in K_w, t in T
	*/
	//
	// {
	// 	for (int t = 0; t < params.numPeriods; ++t)
	// 	{
	// 		for (int w = 0; w < params.numWarehouses; ++w)
	// 		{
	// 			for (int k : params.set_WarehouseVehicles[w])
	// 			{
	// 				for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
	// 				{
	// 					if (params.index_i_SecondEchelon[e] >= params.numWarehouses && params.index_j_SecondEchelon[e] >= params.numWarehouses)
	// 					{
	// 						string constraintName = "LogicalInequality_TwoSetOne(" + std::to_string(e) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

	// 						IloExpr expr(env);
	// 						expr += x[e][k][t];
	// 						expr -= z[params.index_i_SecondEchelon[e]][k][t];
	// 						IloConstraint logicalInequality_TwoSetOne(expr <= 0);
	// 						expr.end();

	// 						model.add(logicalInequality_TwoSetOne).setName(constraintName.c_str());
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}
	// }

	//
	// {
	// 	for (int t = 0; t < params.numPeriods; ++t)
	// 	{
	// 		for (int w = 0; w < params.numWarehouses; ++w)
	// 		{
	// 			for (int k : params.set_WarehouseVehicles[w])
	// 			{
	// 				for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
	// 				{
	// 					if (params.index_i_SecondEchelon[e] >= params.numWarehouses && params.index_j_SecondEchelon[e] >= params.numWarehouses)
	// 					{
	// 						string constraintName = "LogicalInequality_TwoSetTwo(" + std::to_string(e) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

	// 						IloExpr expr(env);
	// 						expr += x[e][k][t];
	// 						expr -= z[params.index_j_SecondEchelon[e]][k][t];
	// 						IloConstraint logicalInequality_TwoSetTwo(expr <= 0);
	// 						expr.end();

	// 						model.add(logicalInequality_TwoSetTwo).setName(constraintName.c_str());
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}
	// }
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Symmetry Breaking Constraints (#1):
			k can be dispatched iff k - 1 is already dispatched
			z[w][k][t] <= z[w][k - 1][t] for all w in N_w, k in K_w t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (params.set_WarehouseVehicles[w].size() > 1)
			{
				for (int k = params.set_WarehouseVehicles[w][1]; k < params.set_WarehouseVehicles[w][params.set_WarehouseVehicles[w].size() - 1]; ++k)
				{
					string constraintName = "SymmetryBreakingConstraint_One(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

					IloExpr expr(env);
					expr += z[w][k][t];
					expr -= z[w][k - 1][t];
					IloConstraint symmetryBreakingConstraint_One(expr <= 0);
					expr.end();

					model.add(symmetryBreakingConstraint_One).setName(constraintName.c_str());
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Lexicographic Constraints:
			sum(i = 1 to j) 2**(j - i) * z[i][k][t] <= sum(i = 1 to j) 2**(j - i) * z[i][k - 1][t] for all j in N_c, k in K_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k = params.set_WarehouseVehicles[w][1]; k < params.set_WarehouseVehicles[w][params.set_WarehouseVehicles[w].size() - 1]; ++k)
			{
				if (params.set_WarehouseVehicles[w].size() > 1)
				{
					for (int j = 0; j < params.numCustomers; ++j)
					{
						string constraintName = "LexicographicConstraint(" + std::to_string(j + 1 + params.numWarehouses) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

						IloExpr expr(env);
						for (int i = 0; i <= j; i++)
						{
							expr += pow(2, (j - i)) * z[i][k][t];
							expr -= pow(2, (j - i)) * z[i][k - 1][t];
						}

						IloConstraint LexicographicConstraint(expr <= 0);
						expr.end();

						model.add(LexicographicConstraint).setName(constraintName.c_str());
					}
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/* End of Valid Inequalities */
	// ---------------------------------------------------------------------------------------------------------------------------
}

void BC_EEV::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	solSE.warehouseInventory.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	solSE.customerInventory.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	solSE.customerUnmetDemand.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	solSE.deliveryQuantityToCustomer.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	solSE.routesWarehouseToCustomer.assign(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>())));
	solSE.customerAssignmentToWarehouse.assign(params.numPeriods, vector<vector<int>>(params.numWarehouses, vector<int>(params.numCustomers, 0)));

	vector<vector<vector<int>>> visitedNodes_SecondEchelon(params.numNodes_SecondEchelon, vector<vector<int>>(params.numVehicles_SecondEchelon, vector<int>(params.numPeriods, 0)));
	vector<vector<vector<int>>> visitedEdges_SecondEchelon(params.numEdges_SecondEchelon, vector<vector<int>>(params.numVehicles_SecondEchelon, vector<int>(params.numPeriods, 0)));

	// Get solution values of decision variables
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			solSE.warehouseInventory[w][t] = cplex.getValue(I_warehouse[w][t]);
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			solSE.customerInventory[i][t] = cplex.getValue(I_customer[i][t]);
			solSE.customerUnmetDemand[i][t] = cplex.getValue(b_customer[i][t]);
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				solSE.deliveryQuantityToCustomer[i][t] += cplex.getValue(w_customer[i][k][t]);
			}
		}

		for (int i = 0; i < params.numNodes_SecondEchelon; ++i)
		{
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				visitedNodes_SecondEchelon[i][k][t] = cplex.getIntValue(z[i][k][t]);
			}
		}

		for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
		{
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				visitedEdges_SecondEchelon[e][k][t] = cplex.getIntValue(x[e][k][t]);
			}
		}
	}

	// Check split deliveries
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			int sumVisitedVehicles = 0;
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				sumVisitedVehicles += visitedNodes_SecondEchelon[i + params.numWarehouses][k][t];
			}
			if (sumVisitedVehicles > 1)
			{
				cout << "Split deliveries for customer " << i << " at time " << t << endl;
				exit(0);
			}
		}
	}

	auto visitEdge_Copy = visitedEdges_SecondEchelon;
	// find and save the routes for second echelon
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			int vehInd = 0;
			for (int k : params.set_WarehouseVehicles[w])
			{
				vector<int> route;
				if (visitedNodes_SecondEchelon[w][k][t] == 1)
				{
					// int routeSize = 0;
					// for (int i = 0; i < params.numNodes_SecondEchelon; ++i)
					// {
					// 	if (visitedNodes_SecondEchelon[i][k][t] == 1)
					// 	{
					// 		routeSize++;
					// 		cout << "visitedNodes_SecondEchelon[" << i + 1 << "][" << k + 1 << "][" << t + 1 << "] = " << visitedNodes_SecondEchelon[i][k][t] << endl;
					// 	}
					// }

					route.push_back(w);
					int currentNode = w;
					vector<bool> visited(params.numNodes_SecondEchelon, false);
					// cout << "currentNode = " << currentNode + 1 << endl;

					while (!visited[w])
					{
						for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
						{
							if (params.index_i_SecondEchelon[e] == currentNode && !visited[params.index_j_SecondEchelon[e]] && visitEdge_Copy[e][k][t] == 1)
							{
								// cout << "x[" << params.index_i_SecondEchelon[e] + 1 << "][" << params.index_j_SecondEchelon[e] + 1 << "][" << t + 1 << "] = " << visitEdge_Copy[e][k][t] << endl;
								route.push_back(params.index_j_SecondEchelon[e]);
								currentNode = params.index_j_SecondEchelon[e];
								visited[params.index_j_SecondEchelon[e]] = true;
								visitEdge_Copy[e][k][t] = 0;
								break;
							}
							else if (params.index_j_SecondEchelon[e] == currentNode && !visited[params.index_i_SecondEchelon[e]] && visitEdge_Copy[e][k][t] == 1)
							{
								// cout << "x[" << params.index_i_SecondEchelon[e] + 1 << "][" << params.index_j_SecondEchelon[e] + 1 << "][" << t + 1 << "] = " << visitEdge_Copy[e][k][t] << endl;
								route.push_back(params.index_i_SecondEchelon[e]);
								currentNode = params.index_i_SecondEchelon[e];
								visited[params.index_i_SecondEchelon[e]] = true;
								visitEdge_Copy[e][k][t] = 0;
								break;
							}
							else if (params.index_i_SecondEchelon[e] == currentNode && !visited[params.index_j_SecondEchelon[e]] && visitEdge_Copy[e][k][t] == 2)
							{
								// cout << "x[" << params.index_i_SecondEchelon[e] + 1 << "][" << params.index_j_SecondEchelon[e] + 1 << "][" << t + 1 << "] = " << visitEdge_Copy[e][k][t] << endl;
								route.push_back(params.index_j_SecondEchelon[e]);
								currentNode = params.index_j_SecondEchelon[e];
								visited[params.index_j_SecondEchelon[e]] = true;
								visitEdge_Copy[e][k][t] = 1;
								break;
							}
						}
						// cout << "currentNode = " << currentNode + 1 << endl;
					}
					solSE.routesWarehouseToCustomer[w][t][vehInd] = route;
					vehInd++;
				}
			}
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				for (int k : params.set_WarehouseVehicles[w])
				{
					if (visitedNodes_SecondEchelon[i + params.numWarehouses][k][t] == 1)
					{
						// cout << "visitedNodes_SecondEchelon[" << i << "][" << k + 1 << "][" << t + 1 << "] = " << visitedNodes_SecondEchelon[i + params.numWarehouses][k][t] << endl;
						// cout << "Customer " << i + 1 + params.numWarehouses  << " is assigned to Warehouse " << w << " on Period " << t + 1 << " with Vehicle " << k + 1 << endl;
						solSE.customerAssignmentToWarehouse[t][w][i] = 1;
						break;
					}
				}
			}
		}
	}
}

void BC_EEV::CalculateCostsForEachPart()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			solSE.holdingCostWarehouse += params.unitHoldingCost_Warehouse[w] * solSE.warehouseInventory[w][t];
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			solSE.holdingCostCustomer += params.unitHoldingCost_Customer[i] * solSE.customerInventory[i][t];
			solSE.costOfUnmetDemand += params.unmetDemandPenalty[i] * solSE.customerUnmetDemand[i][t];
		}
	}

	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				if (!solSE.routesWarehouseToCustomer[w][t][k].empty())
				{
					int previousNode = w;
					for (int j = 1; j < solSE.routesWarehouseToCustomer[w][t][k].size(); ++j)
					{
						int currentNode = solSE.routesWarehouseToCustomer[w][t][k][j];

						solSE.transportationCostWarehouseToCustomer += params.transportationCost_SecondEchelon[previousNode][currentNode];

						previousNode = currentNode;
					}
				}
			}
		}
	}
	cout << "setupCost = " << sol_FE_EV.setupCost << endl;
	cout << "productionCost = " << sol_FE_EV.productionCost << endl;
	cout << "holdingCostPlant = " << sol_FE_EV.holdingCostPlant << endl;
	cout << "transportationCostPlantToWarehouse = " << sol_FE_EV.transportationCostPlantToWarehouse << endl;

	result.objValue_firstEchelon = sol_FE_EV.setupCost + sol_FE_EV.productionCost + sol_FE_EV.holdingCostPlant + sol_FE_EV.transportationCostPlantToWarehouse;

	cout << "Holding cost warehouse : " << solSE.holdingCostWarehouse << endl;
	cout << "Avg holding cost customer : " << solSE.holdingCostCustomer << endl;
	cout << "Avg cost of unmet demand : " << solSE.costOfUnmetDemand << endl;
	cout << "Avg transportation cost warehouse to customer : " << solSE.transportationCostWarehouseToCustomer << endl;
	result.objValue_secondEchelon = solSE.holdingCostWarehouse + solSE.holdingCostCustomer + solSE.costOfUnmetDemand + solSE.transportationCostWarehouseToCustomer;

	result.objValue_Total = result.objValue_firstEchelon + result.objValue_secondEchelon;

	cout << "objValue_FE = " << result.objValue_firstEchelon << endl;
	cout << "objValue_FE : " << result.objValue_secondEchelon << endl;
	cout << "objValue_Total : " << result.objValue_Total << endl;
}

void BC_EEV::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_EV.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << sol_FE_EV.productionSetup[t] << endl;
		}
	}
}

void BC_EEV::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_EV.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_FE_EV.productionQuantity[t] << endl;
		}
	}
}

void BC_EEV::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_EV.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_FE_EV.plantInventory[t] << endl;
		}
	}
}

void BC_EEV::DisplayWarehouseInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (solSE.warehouseInventory[w][t] > THRESHOLD)
			{
				cout << "I_warehouse[" << w + 1 << "][" << t + 1 << "] = " << std::setprecision(0) << std::fixed << solSE.warehouseInventory[w][t] << endl;
			}
		}
	}
}

void BC_EEV::DisplayFirstEchelonRouteVars()
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

void BC_EEV::DisplayDeliveryQuantityToWarehousesVars()
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

void BC_EEV::DisplayCustomerInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (solSE.customerInventory[i][t] > THRESHOLD)
			{
				cout << "I_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "] = " << solSE.customerInventory[i][t] << endl;
			}
		}
	}
}

void BC_EEV::DisplayCustomerUnmetDemandVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (solSE.customerUnmetDemand[i][t] > THRESHOLD)
			{
				cout << "b_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "] = " << solSE.customerUnmetDemand[i][t] << endl;
			}
		}
	}
}

void BC_EEV::DisplayDeliveryQuantityToCustomersVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (solSE.deliveryQuantityToCustomer[i][t] > THRESHOLD)
			{
				cout << "deliveryQuantityToCustomer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "] = " << solSE.deliveryQuantityToCustomer[i][t] << endl;
			}
		}
	}
}

void BC_EEV::DisplayRoutesWarehouseToCustomersVars()
{
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				if (!solSE.routesWarehouseToCustomer[w][t][k].empty())
				{
					cout << "route[" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] : [";
					for (auto it = solSE.routesWarehouseToCustomer[w][t][k].begin(); it != solSE.routesWarehouseToCustomer[w][t][k].end(); ++it)
					{
						if (it != solSE.routesWarehouseToCustomer[w][t][k].begin())
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

void BC_EEV::DefineWarmStartSolution(IloEnv &env, IloCplex &cplex)
{
	IloNumVarArray startVar(env);
	IloNumArray startVal(env);

	// Define warm start solution
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; k++)
			{
				int vehicleIndex = k + (w * params.numVehicles_Warehouse);
				vector<int> &route = warmStart.routesWarehouseToCustomer_WarmStart[w][t][k];
				if (!route.empty())
				{
					startVar.add(z[w][vehicleIndex][t]);
					startVal.add(1);
					for (int i = 0; i < params.numCustomers; ++i)
					{
						int customerIndex = i + params.numWarehouses;
						auto it = std::find(route.begin(), route.end(), customerIndex);
						if (it != route.end())
						{
							startVar.add(z[customerIndex][vehicleIndex][t]);
							startVal.add(1);
						}
						else
						{
							startVar.add(z[customerIndex][vehicleIndex][t]);
							startVal.add(0);
						}
					}
				}
				else
				{
					startVar.add(z[w][vehicleIndex][t]);
					startVal.add(0);
					for (int i = 0; i < params.numCustomers; ++i)
					{
						int customerIndex = i + params.numWarehouses;
						startVar.add(z[customerIndex][vehicleIndex][t]);
						startVal.add(0);
					}
				}
			}
		}
	}

	vector<vector<vector<int>>> x_warmStart(params.numEdges_SecondEchelon, vector<vector<int>>(params.numVehicles_SecondEchelon, vector<int>(params.numPeriods, 0)));
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; k++)
			{
				int vehicleIndex = k + (w * params.numVehicles_Warehouse);
				vector<int> &route = warmStart.routesWarehouseToCustomer_WarmStart[w][t][k];
				if (route.size() > 3)
				{
					int prevNode = w;
					for (size_t i = 1; i < route.size(); ++i)
					{
						int currentNode = route[i];

						x_warmStart[params.index_e_SecondEchelon[prevNode][currentNode]][vehicleIndex][t] = 1;

						prevNode = currentNode;
					}
				}
				else if (route.size() == 3)
				{
					int prevNode = w;
					int currentNode = route[1];

					x_warmStart[params.index_e_SecondEchelon[prevNode][currentNode]][vehicleIndex][t] = 2;
				}
			}
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; k++)
		{
			for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
			{
				startVar.add(x[e][k][t]);
				startVal.add(x_warmStart[e][k][t]);
			}
		}
	}

	cplex.addMIPStart(startVar, startVal, IloCplex::MIPStartEffort::MIPStartRepair);
	startVal.end();
	startVar.end();
}
