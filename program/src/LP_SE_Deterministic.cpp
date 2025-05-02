#include "LP_SE_Deterministic.h"

// ------------------------------------------------------------------------------------------------------------------------
LP_SE_Deterministic::LP_SE_Deterministic(const ParameterSetting &parameters,
										 const SolutionFirstEchelon &solFE,
										 const SolutionSecondEchelon_Deterministic &solSE,
										 bool saveModel,
										 bool saveSolution)
	: params(parameters),
	  sol_FE(solFE),
	  sol_SE(solSE),
	  saveLP(saveModel),
	  saveSol(saveSolution),
	  THRESHOLD(1e-2)
{
	if (params.problemType == "EV" || params.problemType == "EEV" || params.problemType == "WS")
		shortageAllowed = true;

	if (params.problemType == "EEV")
	{
		warehouse_delivery.resize(params.numWarehouses, vector<vector<double>>(params.numVehicles_Plant, vector<double>(params.numPeriods, 0.0)));
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
						if (it != route.end() - 1)
						{
							warehouse_delivery[w][k][t] += sol_FE.deliveryQuantityToWarehouse[w][t];
						}

						// cout << "warehouse_delivery[" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] = " << warehouse_delivery[w][k][t] << endl;
					}
				}
			}
		}

		// for (int w = 0; w < params.numWarehouses; ++w)
		// {
		// 	for (int t = 0; t < params.numPeriods; ++t)
		// 	{
		// 		for (int k = 0; k < params.numVehicles_Warehouse; ++k)
		// 		{
		// 			vector<int> &route = sol_SE.routesWarehouseToCustomer[w][t][k];
		// 			if (!route.empty())
		// 			{
		// 				cout << "route[" << w << "][" << t << "][" << k << "] : [";
		// 				int previousNode = w;
		// 				for (int j = 1; j < route.size(); ++j)
		// 				{
		// 					int currentNode = route[j];
		// 					cout << previousNode << " -> ";

		// 					previousNode = currentNode;
		// 				}
		// 				cout << previousNode << "]" << endl;
		// 			}
		// 		}
		// 	}
		// }
	}
}

string LP_SE_Deterministic::solve()
{
	IloEnv env;
	IloModel model(env);
	IloCplex cplex(model);

	auto startTime = std::chrono::high_resolution_clock::now();

	configureCplex(cplex, env);

	DefineVariables(env, model);
	DefineObjectiveFunction(env, model);
	DefineConstraints(env, model);

	if (saveLP)
	{
		string directory = "../cplexFiles/lpModel/" + params.problemType + "/";
		if (!fs::exists(directory))
		{
			cout << "Directory does not exist. Creating: " << directory << endl;
			fs::create_directories(directory);
		}

		string lpFileName = generateFileName(directory, ".lp");
		cplex.exportModel(lpFileName.c_str());
	}

	cplex.extract(model);

	cplex.solve();
	handleCplexStatus(cplex, env, model);

	if (result.status == "Optimal")
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

	return result.status;
}

void LP_SE_Deterministic::configureCplex(IloCplex &cplex, IloEnv &env)
{
	cplex.setParam(IloCplex::Param::Threads, 1);
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

	// Set CPLEX parameters: No output or warnings
	cplex.setOut(env.getNullStream());
	cplex.setWarning(env.getNullStream());
}

void LP_SE_Deterministic::handleCplexStatus(IloCplex &cplex, IloEnv &env, IloModel &model)
{
	if (cplex.getStatus() == IloAlgorithm::Optimal)
	{
		result.status = "Optimal";
		result.objValue_Total = cplex.getObjValue();
		// cout << "Optimal solution found. Objective Value = " << result.objValue_Total << endl;
	}
	else if (cplex.getStatus() == IloAlgorithm::Infeasible)
	{
		result.status = "Infeasible";
		// cerr << "Problem is infeasible." << endl;
		// refineConflict(cplex, env, model);
		// throw std::runtime_error("Solver terminated with infeasible solution.");
	}
	else
	{
		result.status = "Undefined";
		cerr << "Solver terminated with undefined status." << endl;
	}

	if (saveSol)
	{
		string directory = "../cplexFiles/solVal/" + params.problemType + "/";
		if (!fs::exists(directory))
		{
			cout << "Directory does not exist. Creating: " << directory << endl;
			fs::create_directories(directory);
		}
		string fileName = generateFileName(directory, ".sol");
		cplex.writeSolution(fileName.c_str());
	}
}

string LP_SE_Deterministic::generateFileName(const string &baseDir, const string &extension)
{
	string fileName = baseDir + "LP_SE_" + params.problemType + "_Ins" + params.instance;

	if (params.problemType == "WS" || params.problemType == "EV" || params.problemType == "EEV")
	{
		fileName += "_S" + std::to_string(params.numScenarios);
	}
	if (params.problemType == "WS" || params.problemType == "EEV")
	{
		fileName += "_s" + std::to_string(params.scenarioIndex);
	}
	fileName += extension;

	return fileName;
}

void LP_SE_Deterministic::refineConflict(IloCplex &cplex, IloEnv &env, IloModel &model)
{
	// Iterate through the model to identify constraints
	IloConstraintArray constraints(env);
	IloNumArray priorities(env);

	// Add all constraints to the conflict array with priority 1
	for (IloModel::Iterator it(model); it.ok(); ++it)
	{
		IloExtractable extractable = *it;
		if (extractable.isConstraint()) // Check if it's a constraint
		{
			IloConstraint constraint = extractable.asConstraint();
			constraints.add(constraint);
			priorities.add(1.0); // Assign a priority (1.0 = low penalty)
		}
	}

	// Run conflict refinement to identify infeasible subset
	if (cplex.refineConflict(constraints, priorities))
	{
		cout << "Constraints in conflict:" << endl;

		// Iterate through the constraints and check conflict status
		for (int i = 0; i < constraints.getSize(); ++i)
		{
			if (cplex.getConflict(constraints[i]) == IloCplex::ConflictStatus::ConflictMember)
			{
				cout << " - " << constraints[i].getName() << endl; // Print the name of the conflicting constraint
			}
		}
	}
	else
	{
		cout << "Could not find an IIS to explain infeasibility." << endl;
	}

	// Clean up
	constraints.end();
	priorities.end();
}

void LP_SE_Deterministic::DefineVariables(IloEnv &env, IloModel &model)
{
	/* Define Decision Variables */
	if (params.problemType != "EEV")
	{
		DefineFirstStageVars(env, model);
	}
	DefineSecondStageVars(env, model);
}

void LP_SE_Deterministic::DefineFirstStageVars(IloEnv &env, IloModel &model)
{
	VariableManager varManager(env);
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
}

void LP_SE_Deterministic::DefineSecondStageVars(IloEnv &env, IloModel &model)
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
			b_customer[i][t] = IloNumVar(env, 0.0, params.demand_Deterministic[i][t], IloNumVar::Float, varName.c_str());
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
				w_customer[i][k][t] = IloNumVar(env, 0.0, params.demand_Deterministic[i][t], IloNumVar::Float, varName.c_str());
				model.add(w_customer[i][k][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
}

void LP_SE_Deterministic::DefineObjectiveFunction(IloEnv &env, IloModel &model)
{
	// Define objective function
	IloExpr obj(env);

	if (params.problemType != "EEV")
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			obj += params.unitProdCost * p[t];
			obj += params.unitHoldingCost_Plant * I_plant[t];
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			obj += params.unitHoldingCost_Warehouse[w] * I_warehouse[w][t];
		}
	}

	if (params.problemType == "2EPRPCS")
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int k : params.set_WarehouseVehicles[w])
			{
				for (int t = 0; t < params.numPeriods; ++t)
				{
					for (int i = 0; i < params.numCustomers; ++i)
					{
						obj += params.unitHandlingCost_Satellite[w] * w_customer[i][k][t];
					}
				}
			}
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			obj += params.unitHoldingCost_Customer[i] * I_customer[i][t];
			obj += params.unmetDemandPenalty[i] * b_customer[i][t];
		}
	}
	model.add(IloMinimize(env, obj));
}

void LP_SE_Deterministic::DefineConstraints(IloEnv &env, IloModel &model)
{
	/* Define Constraints */
	if (params.problemType != "EEV")
	{
		DefCons_ProductionCapacity(env, model);
		DefCons_PlantInventoryCapacity(env, model);
		DefCons_PlantInventoryBalance(env, model);
		DefCons_WarehouseVisit_FirstEchelon(env, model);
		DefCons_VehicleCapacity_FirstEchelon(env, model);
	}

	// if (params.problemType == "2EPRPCS")
	// {
	// 	DefCons_SatelliteInventoryBalance(env, model);
	// }

	DefCons_WarehouseInventoryCapacity(env, model);
	if (params.problemType == "EEV")
	{
		DefCons_WarehouseInventoryBalance_EEV(env, model);
	}
	else
	{
		DefCons_WarehouseInventoryBalance(env, model);
	}

	DefCons_CustomerInventoryCapacity(env, model);
	DefCons_CustomerInventoryBalance(env, model);
	DefCons_CustomerVisit_SecondEchelon(env, model);
	DefCons_VehicleCapacity_SecondEchelon(env, model);
}

void LP_SE_Deterministic::DefCons_ProductionCapacity(IloEnv &env, IloModel &model)
{
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
}

void LP_SE_Deterministic::DefCons_PlantInventoryCapacity(IloEnv &env, IloModel &model)
{
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
}

void LP_SE_Deterministic::DefCons_PlantInventoryBalance(IloEnv &env, IloModel &model)
{
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
}

void LP_SE_Deterministic::DefCons_WarehouseVisit_FirstEchelon(IloEnv &env, IloModel &model)
{
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
						double deliveryUB = params.storageCapacity_Warehouse[w];

						IloConstraint WarehouseDelivery(expr <= deliveryUB);
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
}

void LP_SE_Deterministic::DefCons_VehicleCapacity_FirstEchelon(IloEnv &env, IloModel &model)
{
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
}

void LP_SE_Deterministic::DefCons_WarehouseInventoryCapacity(IloEnv &env, IloModel &model)
{
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
}

void LP_SE_Deterministic::DefCons_WarehouseInventoryBalance(IloEnv &env, IloModel &model)
{
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
}

void LP_SE_Deterministic::DefCons_WarehouseInventoryBalance_EEV(IloEnv &env, IloModel &model)
{
	/*
		Warehouse Inventory Balance Constraints:
			I[w][t] = I[w][t-1] + sum(k in KP) q[w][k][t] - sum(i in N_c) sum(k in K_w) w_customer[i][k][t]	for all w in W, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			string constraintName = "WarehouseInventoryBalance_EEV(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + ")";

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
}

void LP_SE_Deterministic::DefCons_SatelliteInventoryBalance(IloEnv &env, IloModel &model)
{
	/*
		Satellite Inventory Balance Constraints:
			sum(r in R) q[w][r][t] = sum(i in N_c) sum(k in K_w) w_customer[i][k][t]	for all w in W, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			string constraintName = "SatelliteInventoryBalance(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			for (int k = 0; k < params.numVehicles_Plant; ++k)
			{
				expr += q[w][k][t];
			}

			for (int k : params.set_WarehouseVehicles[w])
			{
				for (int i = 0; i < params.numCustomers; ++i)
				{
					expr -= w_customer[i][k][t];
				}
			}
			IloConstraint SatelliteInventoryBalanceConstraint(expr == 0.0);
			expr.end();

			model.add(SatelliteInventoryBalanceConstraint).setName(constraintName.c_str());
		}
	}
}

void LP_SE_Deterministic::DefCons_CustomerInventoryBalance(IloEnv &env, IloModel &model)
{
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

				IloConstraint CustomerInventoryBalanceConstraint(expr == params.initialInventory_Customer[i] - params.demand_Deterministic[i][t]);
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

				IloConstraint CustomerInventoryBalanceConstraint(expr == -params.demand_Deterministic[i][t]);
				expr.end();

				model.add(CustomerInventoryBalanceConstraint).setName(constraintName.c_str());
			}
		}
	}
}

void LP_SE_Deterministic::DefCons_CustomerInventoryCapacity(IloEnv &env, IloModel &model)
{
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
			IloConstraint CustomerInventoryCapacityConstraint(expr <= params.storageCapacity_Customer[i] - params.demand_Deterministic[i][t]);
			expr.end();

			model.add(CustomerInventoryCapacityConstraint).setName(constraintName.c_str());
		}
	}
}

void LP_SE_Deterministic::DefCons_CustomerVisit_SecondEchelon(IloEnv &env, IloModel &model)
{
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

				// if (!route.empty()){
				// 	cout << "Route: " << w + 1 << " Period: " << t + 1 << " Vehicle: " << k + 1 << " : ";
				// 	for (size_t i = 0; i < route.size() - 1; ++i)
				// 	{
				// 		cout << route[i] << "->";
				// 	}
				// 	cout << route.back() << endl;
				// }

				for (int i = 0; i < params.numCustomers; ++i)
				{
					int customerIndex = i + params.numWarehouses;
					auto it = std::find(route.begin(), route.end(), customerIndex);

					string constraintName = "CustomerVisit(" + std::to_string(customerIndex + 1) + "," + std::to_string(vehicleIndex + 1) + "," + std::to_string(t + 1) + ")";
					IloExpr expr(env);
					expr += w_customer[i][vehicleIndex][t];

					if (it != route.end())
					{
						IloConstraint customerVisitConstraint(expr <= params.DelUB_perCus_Det[i][t]);

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
}

void LP_SE_Deterministic::DefCons_VehicleCapacity_SecondEchelon(IloEnv &env, IloModel &model)
{
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
}

void LP_SE_Deterministic::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	if (params.problemType != "EEV")
	{
		sol_FE_temp.productionSetup = sol_FE.productionSetup;
		sol_FE_temp.productionQuantity.assign(params.numPeriods, 0.0);
		sol_FE_temp.plantInventory.assign(params.numPeriods, 0.0);
		sol_FE_temp.deliveryQuantityToWarehouse.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));

		for (int t = 0; t < params.numPeriods; ++t)
		{
			sol_FE_temp.productionQuantity[t] = cplex.getValue(p[t]);
			sol_FE_temp.plantInventory[t] = cplex.getValue(I_plant[t]);

			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k = 0; k < params.numVehicles_Plant; ++k)
				{
					sol_FE_temp.deliveryQuantityToWarehouse[w][t] += cplex.getValue(q[w][k][t]);
				}
			}
		}
		sol_FE_temp.routesPlantToWarehouse = sol_FE.routesPlantToWarehouse;
	}

	sol_SE_temp.warehouseInventory.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			sol_SE_temp.warehouseInventory[w][t] = cplex.getValue(I_warehouse[w][t]);
		}
	}

	sol_SE_temp.customerUnmetDemand.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			sol_SE_temp.customerUnmetDemand[i][t] = cplex.getValue(b_customer[i][t]);
		}
	}

	sol_SE_temp.customerInventory.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	sol_SE_temp.deliveryQuantityToCustomer.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			sol_SE_temp.customerInventory[i][t] = cplex.getValue(I_customer[i][t]);
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				sol_SE_temp.deliveryQuantityToCustomer[i][t] += cplex.getValue(w_customer[i][k][t]);
			}
		}
	}

	sol_SE_temp.routesWarehouseToCustomer = sol_SE.routesWarehouseToCustomer;
	sol_SE_temp.customerAssignmentToWarehouse = sol_SE.customerAssignmentToWarehouse;
}

void LP_SE_Deterministic::CalculateCostsForEachPart()
{
	if (params.problemType != "EEV")
	{
		sol_FE_temp.setupCost = sol_FE.setupCost;
		sol_FE_temp.transportationCostPlantToWarehouse += sol_FE.transportationCostPlantToWarehouse;

		for (int t = 0; t < params.numPeriods; ++t)
		{
			sol_FE_temp.productionCost += params.unitProdCost * sol_FE_temp.productionQuantity[t];
			sol_FE_temp.holdingCostPlant += params.unitHoldingCost_Plant * sol_FE_temp.plantInventory[t];
		}
	}
	else
	{
		sol_FE_temp.setupCost = sol_FE.setupCost;
		sol_FE_temp.productionCost = sol_FE.productionCost;
		sol_FE_temp.holdingCostPlant = sol_FE.holdingCostPlant;
		sol_FE_temp.transportationCostPlantToWarehouse = sol_FE.transportationCostPlantToWarehouse;
	}
	result.objValue_firstEchelon = sol_FE_temp.setupCost + sol_FE_temp.productionCost + sol_FE_temp.holdingCostPlant + sol_FE_temp.transportationCostPlantToWarehouse;

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			sol_SE_temp.holdingCostWarehouse += params.unitHoldingCost_Warehouse[w] * sol_SE_temp.warehouseInventory[w][t];
		}
	}
	result.objValue_secondEchelon += sol_SE_temp.holdingCostWarehouse;

	if (params.problemType == "2EPRPCS")
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				sol_SE_temp.handlingCostSatellite += params.unitHandlingCost_Satellite[w] * sol_FE_temp.deliveryQuantityToWarehouse[w][t];
			}
		}
		result.objValue_secondEchelon += sol_SE_temp.handlingCostSatellite;
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			sol_SE_temp.holdingCostCustomer += params.unitHoldingCost_Customer[i] * sol_SE_temp.customerInventory[i][t];
			sol_SE_temp.costOfUnmetDemand += params.unmetDemandPenalty[i] * sol_SE_temp.customerUnmetDemand[i][t];
		}
	}
	result.objValue_secondEchelon += sol_SE_temp.holdingCostCustomer;
	result.objValue_secondEchelon += sol_SE_temp.costOfUnmetDemand;

	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				vector<int> &route = sol_SE_temp.routesWarehouseToCustomer[w][t][k];
				if (!route.empty())
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
	}

	result.objValue_secondEchelon += sol_SE_temp.transportationCostWarehouseToCustomer;

	result.objValue_Total = result.objValue_firstEchelon + result.objValue_secondEchelon;
	// cout << "Transportation Cost First Echelon : " << sol_FE_temp.transportationCostPlantToWarehouse << endl;
	// cout << "Transportation Cost Second Echelon : " << sol_SE_temp.transportationCostWarehouseToCustomer << endl;
}

void LP_SE_Deterministic::DisplayCosts()
{
	cout << "Setup Cost : " << sol_FE_temp.setupCost << endl;
	cout << "Production Cost : " << sol_FE_temp.productionCost << endl;
	cout << "Holding Cost Plant : " << sol_FE_temp.holdingCostPlant << endl;
	cout << "Transportation Cost Plant to Warehouse : " << sol_FE_temp.transportationCostPlantToWarehouse << endl;

	cout << "Holding Cost Warehouse : " << sol_SE_temp.holdingCostWarehouse << endl;

	if (params.problemType == "2EPRPCS")
		cout << "Handling Cost Satellite : " << sol_SE_temp.handlingCostSatellite << endl;

	cout << "Holding Cost Customer : " << sol_SE_temp.holdingCostCustomer << endl;

	cout << "Cost of Unmet Demand : " << sol_SE_temp.costOfUnmetDemand << endl;

	cout << "Transportation Cost Warehouse to Customer : " << sol_SE_temp.transportationCostWarehouseToCustomer << endl;

	cout << "\nObjective value FE : " << result.objValue_firstEchelon << endl;
	cout << "Objective value SE : " << result.objValue_secondEchelon << endl;
	cout << "Objective value Total : " << result.objValue_Total << endl;
}

void LP_SE_Deterministic::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_temp.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << sol_FE_temp.productionSetup[t] << endl;
		}
	}
}

void LP_SE_Deterministic::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_temp.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_FE_temp.productionQuantity[t] << endl;
		}
	}
}

void LP_SE_Deterministic::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol_FE_temp.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol_FE_temp.plantInventory[t] << endl;
		}
	}
}

void LP_SE_Deterministic::DisplayWarehouseInventoryVars()
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

void LP_SE_Deterministic::DisplayFirstEchelonRouteVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		int routeInd = 1;
		for (const auto &route : sol_FE_temp.routesPlantToWarehouse[t])
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

void LP_SE_Deterministic::DisplayDeliveryQuantityToWarehousesVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (sol_FE_temp.deliveryQuantityToWarehouse[w][t] > THRESHOLD)
			{
				cout << "q[" << w + 1 << "][" << t + 1 << "] = " << sol_FE_temp.deliveryQuantityToWarehouse[w][t] << endl;
			}
		}
	}
}

void LP_SE_Deterministic::DisplayCustomerInventoryVars()
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

void LP_SE_Deterministic::DisplayCustomerUnmetDemandVars()
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

void LP_SE_Deterministic::DisplayDeliveryQuantityToCustomersVars()
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

void LP_SE_Deterministic::DisplayRoutesWarehouseToCustomersVars()
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

SolutionFirstEchelon LP_SE_Deterministic::getSolutionFE()
{
	return sol_FE_temp;
}

SolutionSecondEchelon_Deterministic LP_SE_Deterministic::getSolutionSE()
{
	return sol_SE_temp;
}

Result LP_SE_Deterministic::getResult()
{
	return result;
}