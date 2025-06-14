#include "R2EPRP.h"

R2EPRP::R2EPRP(const ParameterSetting &parameters, const SolutionWarmStart_Deterministic &warmStartSol, bool saveModel, bool saveSolution)
	: params(parameters),
	  warmStart(warmStartSol),
	  saveLP(saveModel),
	  saveSol(saveSolution),
	  THRESHOLD(1e-2)
{
	routeMatrix_FirstEchelon = params.getRouteMatrix();
	numRoutes_FirstEchelon = routeMatrix_FirstEchelon.size();
	optimalRoutes_FirstEchelon = params.getOptimalRoutes();
	routeCosts_FirstEchelon = params.getRouteCosts();

	if (params.problemType == "EV" || params.problemType == "WS")
		shortageAllowed = true;
}

bool R2EPRP::Solve()
{
	try
	{
		IloEnv env;
		IloModel model(env);
		IloCplex cplex(model);

		auto startTime = std::chrono::high_resolution_clock::now();

		configureCplex(cplex, env);

		DefineVariables(env, model);
		DefineObjectiveFunction(env, model);
		DefineConstraints(env, model);

		cout << "Solving R2EPRP-" << params.problemType << "..." << endl;
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

		// Solution warmstart;
		if (!warmStart.empty())
		{
			DefineWarmStartSolution(env, cplex);
		}
		else
		{
			cout << "Couldn't Find A Solution For Warmstart" << endl;
		}

		cplex.solve();
		handleCplexStatus(cplex, env, model);

		auto currentTime = std::chrono::high_resolution_clock::now();
		result.totalCPUTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Timer (seconds): " << std::fixed << std::setprecision(4) << result.totalCPUTime << endl;

		if (result.status == "Optimal" || result.status == "Incumbent")
		{
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
			// DisplayCostsForEachPart();
		}

		env.end();

		return true;
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
	return false;
	// return true;
}

void R2EPRP::configureCplex(IloCplex &cplex, IloEnv &env)
{
	// Set CPLEX Parameters: (DISPLAY LEVEL(0,1,2,3,4), OPTIMALITY GAP, RUN TIME (SECS), THREADS, MEMORY (MB))
	CplexParameterManager parameterManager(cplex);
	parameterManager.setParameters(2, params.R2EPRP_OptimalityGap, params.R2EPRP_TimeLimit, params.R2EPRP_NumThreads, params.R2EPRP_MemoryLimit);

	cplex.setParam(IloCplex::Param::Emphasis::MIP, 2);
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
	cplex.setParam(IloCplex::Param::MIP::Strategy::Search, CPX_MIPSEARCH_TRADITIONAL);
	cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);

	// Set CPLEX parameters: No output or warnings
	cplex.setOut(env.getNullStream());
	cplex.setWarning(env.getNullStream());
}

void R2EPRP::handleCplexStatus(IloCplex &cplex, IloEnv &env, IloModel &model)
{
	if (cplex.getStatus() == IloAlgorithm::Optimal)
	{
		result.status = "Optimal";
		result.objValue_Total = cplex.getObjValue();
		result.optimalityGap = cplex.getMIPRelativeGap() * 100;
		result.lowerBound = cplex.getBestObjValue();
		// cout << "Optimal solution found. Objective Value = " << result.objValue_Total << endl;
	}
	else if (cplex.getStatus() == IloAlgorithm::Feasible)
	{
		result.status = "Incumbent";
		result.objValue_Total = cplex.getObjValue();
		result.optimalityGap = cplex.getMIPRelativeGap() * 100;
		result.lowerBound = cplex.getBestObjValue();
		// cout << "Feasible solution found. Objective Value: " << result.objValue_Total << endl;
		// cout << "LB = " << result.lowerBound;
		// cout << "Optimality Gap (%) = " << result.optimalityGap;
	}
	else if (cplex.getStatus() == IloAlgorithm::Infeasible)
	{
		result.status = "Infeasible";
		// cerr << "Problem is infeasible." << endl;
		// refineConflict(cplex, env, model);
		// saveSol = true;
		// exit(0);
		throw std::runtime_error("Solver terminated with infeasible solution.");
	}
	else
	{
		result.status = "Undefined";
		// cerr << "Solver terminated with undefined status." << endl;
	}

	if (saveSol)
	{
		string directory = "../cplexFiles/solVal/" + params.problemType + "/";
		string fileName = generateFileName(directory, ".sol");
		cplex.writeSolution(fileName.c_str());
	}
}

string R2EPRP::generateFileName(const string &baseDir, const string &extension)
{
	string fileName = baseDir + "R2EPRP_" + params.problemType + "_Ins" + params.instance;

	if (params.problemType == "WS" || params.problemType == "EV")
	{
		fileName += "_S" + std::to_string(params.numScenarios);
	}
	if (params.problemType == "WS")
	{
		fileName += "_s" + std::to_string(params.scenarioIndex);
	}
	fileName += extension;

	return fileName;
}

void R2EPRP::refineConflict(IloCplex &cplex, IloEnv &env, IloModel &model)
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

void R2EPRP::DefineVariables(IloEnv &env, IloModel &model)
{
	/* Define Decision Variables */
	// Initialize Variable Manager
	VariableManager varManager(env);
	// -------------------------------------------------------------------------------------------------------------------------------
	// y[t] variables (Production setup in period t)
	y = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "y[" + std::to_string(t + 1) + "]";
		y[t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
		model.add(y[t]);
	}
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
	// q[r][w][t] variables (Delivery quantity to warehouse w in period t from route r)
	q = varManager.create3D(numRoutes_FirstEchelon, params.numWarehouses, params.numPeriods);
	for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				string varName = "q[" + std::to_string(routeInd + 1) + "][" + std::to_string(w + 1) + "][" + std::to_string(t + 1) + "]";
				q[routeInd][w][t] = IloNumVar(env, 0.0, routeMatrix_FirstEchelon[routeInd][w + 1] * params.storageCapacity_Warehouse[w], IloNumVar::Float, varName.c_str());
				model.add(q[routeInd][w][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// o[r][t] variables (binary variables to indicate whether route r is selected in period t) - from plant to warehouse
	o = varManager.create2D(numRoutes_FirstEchelon, params.numPeriods);
	for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "o[" + std::to_string(routeInd + 1) + "][" + std::to_string(t + 1) + "]";
			o[routeInd][t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
			model.add(o[routeInd][t]);
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
}

void R2EPRP::DefineObjectiveFunction(IloEnv &env, IloModel &model)
{
	// Define objective function
	IloExpr obj(env);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		obj += params.setupCost * y[t];
		obj += params.unitProdCost * p[t];
		obj += params.unitHoldingCost_Plant * I_plant[t];
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			obj += routeCosts_FirstEchelon[routeInd] * o[routeInd][t];
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			obj += params.unitHoldingCost_Customer[i] * I_customer[i][t];
			obj += params.unmetDemandPenalty[i] * b_customer[i][t];
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
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			obj += params.unitHoldingCost_Warehouse[w] * I_warehouse[w][t];
		}
	}
	model.add(IloMinimize(env, obj));
}

void R2EPRP::DefineConstraints(IloEnv &env, IloModel &model)
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
		expr -= params.prodCapacity * y[t];
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
			I_plant[t] = I_plant[t-1] + p[t] - sum(r in R) sum(w in W) q[r][w][t] 		for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "PlantInventoryBalance(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		if (t == 0)
		{
			expr += I_plant[t];
			expr -= p[t];
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					expr += q[routeInd][w][t];
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
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					expr += q[routeInd][w][t];
				}
			}
			IloConstraint plantInventoryBalanceConstraint(expr == 0);
			expr.end();

			model.add(plantInventoryBalanceConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Vehicle Capacity Constraints (From Plant to Warehouse):
			sum(w in W) q[r][w][t] <= params.vehicleCapacity_Plant * o[r][t] 		for all r in R, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			string constraintName = "VehicleCapacityPlant(" + std::to_string(routeInd + 1) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				expr += q[routeInd][w][t];
			}
			expr -= params.vehicleCapacity_Plant * o[routeInd][t];
			IloConstraint vehicleCapacityConstraint(expr <= 0);
			expr.end();

			model.add(vehicleCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Fleet Size Constraints:
			sum (r in R) o[r][t] <= params.numVehicles_Plant 		for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "FleetSize(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			expr += o[routeInd][t];
		}
		IloConstraint fleetSizeConstraint(expr <= params.numVehicles_Plant);
		expr.end();

		model.add(fleetSizeConstraint).setName(constraintName.c_str());
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		WarehouseVisit(r,w,t):
			q[r][w][t] <= o[r][t] * routeMatrix[r][w+1] * storageCapacity_Warehouse[w] 		for all r in R, w in W, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				string constraintName = "WarehouseVisit(" + std::to_string(routeInd + 1) + "," + std::to_string(w + 1) + "," + std::to_string(t + 1) + ")";

				IloExpr expr(env);

				expr += q[routeInd][w][t];

				expr -= o[routeInd][t] * routeMatrix_FirstEchelon[routeInd][w + 1] * params.storageCapacity_Warehouse[w];
				IloConstraint warehouseVisitConstraint(expr <= 0);
				expr.end();

				model.add(warehouseVisitConstraint).setName(constraintName.c_str());
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

				for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
				{
					expr -= q[routeInd][w][t];
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

				for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
				{
					expr -= q[routeInd][w][t];
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
			IloConstraint CustomerInventoryCapacityConstraint(expr <= params.storageCapacity_Customer[i] - params.demand_Deterministic[i][t]);
			expr.end();

			model.add(CustomerInventoryCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Each Warehouse must be covered by at most one route.
			sum(r in R) a[r][w] * o[r][t] <= 1			for all w in w and t in T
	// */
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			string constraintName = "WarehouseCoverage(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				expr += routeMatrix_FirstEchelon[routeInd][w + 1] * o[routeInd][t];
			}
			IloConstraint warehouseCoverageConstraint(expr <= 1);
			expr.end();

			model.add(warehouseCoverageConstraint).setName(constraintName.c_str());
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

				vector<int> route = warmStart.routesWarehouseToCustomer_WarmStart[w][t][k];
				if (!route.empty())
				{
					for (int i = 0; i < params.numCustomers; ++i)
					{
						int customerIndex = i + params.numWarehouses;
						auto it = std::find(route.begin() + 1, route.end() - 1, customerIndex);

						string constraintName = "CustomerVisit(" + std::to_string(customerIndex + 1) + "," + std::to_string(vehicleIndex + 1) + "," + std::to_string(t + 1) + ")";
						IloExpr expr(env);
						expr += w_customer[i][vehicleIndex][t];

						if (it != route.end() - 1)
						{
							IloConstraint customerVisitConstraint(expr <= params.DelUB_perCus_Det[i][t]);
							expr.end();

							model.add(customerVisitConstraint).setName(constraintName.c_str());
						}
						else
						{
							IloConstraint customerVisitConstraint(expr == 0);
							expr.end();

							model.add(customerVisitConstraint).setName(constraintName.c_str());
						}
					}
				}
				else
				{
					for (int i = 0; i < params.numCustomers; ++i)
					{
						int customerIndex = i + params.numWarehouses;

						string constraintName = "CustomerVisit(" + std::to_string(customerIndex + 1) + "," + std::to_string(vehicleIndex + 1) + "," + std::to_string(t + 1) + ")";
						IloExpr expr(env);
						expr += w_customer[i][vehicleIndex][t];

						IloConstraint customerVisitConstraint(expr == 0);
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

void R2EPRP::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	solFE.productionSetup.assign(params.numPeriods, 0.0);
	solFE.productionQuantity.assign(params.numPeriods, 0.0);
	solFE.plantInventory.assign(params.numPeriods, 0.0);
	solSE.warehouseInventory.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	solSE.customerInventory.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	solSE.customerUnmetDemand.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	solFE.deliveryQuantityToWarehouse.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	solFE.routesPlantToWarehouse.assign(params.numPeriods, vector<vector<int>>(params.numVehicles_Plant, vector<int>()));
	solSE.deliveryQuantityToCustomer.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	solSE.routesWarehouseToCustomer.assign(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>())));

	selectedRoute.assign(numRoutes_FirstEchelon, vector<int>(params.numPeriods, 0));

	// Get solution values of decision variables
	for (int t = 0; t < params.numPeriods; ++t)
	{
		solFE.productionSetup[t] = cplex.getIntValue(y[t]);
		solFE.productionQuantity[t] = cplex.getValue(p[t]);
		solFE.plantInventory[t] = cplex.getValue(I_plant[t]);

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				solFE.deliveryQuantityToWarehouse[w][t] += cplex.getValue(q[routeInd][w][t]);
			}

			solSE.warehouseInventory[w][t] = cplex.getValue(I_warehouse[w][t]);
		}

		int vehInd = 0;
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			selectedRoute[routeInd][t] = cplex.getIntValue(o[routeInd][t]);
			if (selectedRoute[routeInd][t] == 1)
			{
				for (int node : optimalRoutes_FirstEchelon[routeInd])
				{
					solFE.routesPlantToWarehouse[t][vehInd].push_back(node);
				}
				vehInd++;
			}
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
	}

	solSE.routesWarehouseToCustomer = warmStart.routesWarehouseToCustomer_WarmStart;
	solSE.customerAssignmentToWarehouse = warmStart.customerAssignmentToWarehouse_WarmStart;
}

void R2EPRP::CalculateCostsForEachPart()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		solFE.setupCost += params.setupCost * solFE.productionSetup[t];
		solFE.productionCost += params.unitProdCost * solFE.productionQuantity[t];
		solFE.holdingCostPlant += params.unitHoldingCost_Plant * solFE.plantInventory[t];

		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			if (selectedRoute[routeInd][t] == 1)
			{
				solFE.transportationCostPlantToWarehouse += routeCosts_FirstEchelon[routeInd];
			}
		}

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			solSE.holdingCostWarehouse += params.unitHoldingCost_Warehouse[w] * solSE.warehouseInventory[w][t];
		}

		if (params.problemType == "2EPRPCS")
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				solSE.handlingCostSatellite += params.unitHandlingCost_Satellite[w] * solFE.deliveryQuantityToWarehouse[w][t];
			}
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

	result.objValue_firstEchelon = solFE.setupCost + solFE.productionCost + solFE.holdingCostPlant + solFE.transportationCostPlantToWarehouse;
	
	result.objValue_secondEchelon += solSE.holdingCostWarehouse;
	if (params.problemType == "2EPRPCS")
	{
		result.objValue_secondEchelon += solSE.handlingCostSatellite;
	}

	result.objValue_secondEchelon += solSE.costOfUnmetDemand;
	result.objValue_secondEchelon += solSE.holdingCostCustomer + solSE.transportationCostWarehouseToCustomer;

	result.objValue_Total = result.objValue_firstEchelon + result.objValue_secondEchelon;
}

void R2EPRP::DisplayCostsForEachPart()
{
	cout << "setupCost (R2EPRP) = " << solFE.setupCost << endl;
	cout << "productionCost (R2EPRP) = " << solFE.productionCost << endl;
	cout << "holdingCostPlant (R2EPRP) = " << solFE.holdingCostPlant << endl;
	cout << "transportationCostPlantToWarehouse (R2EPRP) = " << solFE.transportationCostPlantToWarehouse << endl;
	cout << "objValue_FE (R2EPRP) = " << result.objValue_firstEchelon << endl;
	cout << "holding cost warehouse (R2EPRP) : " << solSE.holdingCostWarehouse << endl;
	if (params.problemType == "2EPRPCS")
	{
		cout << "handling cost satellite (R2EPRP) : " << solSE.handlingCostSatellite << endl;
	}
	
	cout << "holding cost customer (R2EPRP) : " << solSE.holdingCostCustomer << endl;

	cout << "cost of unmet demand (R2EPRP) : " << solSE.costOfUnmetDemand << endl;
	cout << "transportation cost warehouse to customer (R2EPRP) : " << solSE.transportationCostWarehouseToCustomer << endl;

	cout << "Objective value FE : " << result.objValue_firstEchelon << endl;
	cout << "Objective value (ILS) SE : " << result.objValue_secondEchelon << endl;
	cout << "Objective value (ILS) Total : " << result.objValue_Total << endl;
}

void R2EPRP::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << solFE.productionSetup[t] << endl;
		}
	}
}

void R2EPRP::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << solFE.productionQuantity[t] << endl;
		}
	}
}

void R2EPRP::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << solFE.plantInventory[t] << endl;
		}
	}
}

void R2EPRP::DisplayWarehouseInventoryVars()
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

void R2EPRP::DisplayFirstEchelonRouteVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			if (selectedRoute[routeInd][t] == 1)
			{
				cout << "o[" << routeInd + 1 << "][" << t + 1 << "] = " << selectedRoute[routeInd][t] << endl;
			}
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		int routeInd = 1;
		for (const auto &route : solFE.routesPlantToWarehouse[t])
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

void R2EPRP::DisplayDeliveryQuantityToWarehousesVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (solFE.deliveryQuantityToWarehouse[w][t] > THRESHOLD)
			{
				cout << "q[" << w + 1 << "][" << t + 1 << "] = " << solFE.deliveryQuantityToWarehouse[w][t] << endl;
			}
		}
	}
}

void R2EPRP::DisplayCustomerInventoryVars()
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

void R2EPRP::DisplayCustomerUnmetDemandVars()
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

void R2EPRP::DisplayDeliveryQuantityToCustomersVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			if (solSE.deliveryQuantityToCustomer[i][t] > THRESHOLD)
			{
				cout << "deliveryQuantityToCustomer[" << i + params.numWarehouses << "][" << t + 1 << "] = " << solSE.deliveryQuantityToCustomer[i][t] << endl;
			}
		}
	}
}

void R2EPRP::DisplayRoutesWarehouseToCustomersVars()
{
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_Warehouse; ++k)
			{
				cout << "route[" << w << "][" << t + 1 << "][" << k + 1 << "] : [";
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

void R2EPRP::DefineWarmStartSolution(IloEnv &env, IloCplex &cplex)
{
	IloNumVarArray startVar(env);
	IloNumArray startVal(env);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		startVar.add(y[t]);
		startVal.add(warmStart.productionSetup_WarmStart[t]);
	}

	vector<vector<int>> o_warmStart(numRoutes_FirstEchelon, vector<int>(params.numPeriods, 0));
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (const auto &route : warmStart.routesPlantToWarehouse_WarmStart[t])
		{
			auto it = std::find(optimalRoutes_FirstEchelon.begin(), optimalRoutes_FirstEchelon.end(), route);

			if (it != optimalRoutes_FirstEchelon.end())
			{
				int routeIndex = std::distance(optimalRoutes_FirstEchelon.begin(), it);
				o_warmStart[routeIndex][t] = 1;
			}
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int routeIndex = 0; routeIndex < numRoutes_FirstEchelon; ++routeIndex)
		{
			startVar.add(o[routeIndex][t]);
			startVal.add(o_warmStart[routeIndex][t]);
		}
	}

	cplex.addMIPStart(startVar, startVal, IloCplex::MIPStartEffort::MIPStartRepair);
	startVal.end();
	startVar.end();
}
