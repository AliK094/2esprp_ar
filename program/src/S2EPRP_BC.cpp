#include "S2EPRP_BC.h"

S2EPRP_BC::S2EPRP_BC(const ParameterSetting &parameters, const SolutionWarmStart &warmStartSol, bool saveModel, bool saveSolution)
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
}

bool S2EPRP_BC::Solve()
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
		DefineValidInequalities(env, model);

		cout << "Solve S2EPRP_BC..." << endl;
		SEC_S2EPRP LegacyCallback(env, params, x, z);
		cplex.use(&LegacyCallback);

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

		// Extract model
		cplex.extract(model);

		if (!warmStart.empty())
		{
			DefineWarmStartSolution(env, cplex);
		}
		else
		{
			cout << "Couldn't Find A Solution For Warmstart" << endl;
		}

		// Solve the model
		cplex.solve();
		handleCplexStatus(cplex, env, model);

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

		auto currentTime = std::chrono::high_resolution_clock::now();
		result.totalCPUTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Timer (seconds): " << std::fixed << std::setprecision(4) << result.totalCPUTime << endl;

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

void S2EPRP_BC::configureCplex(IloCplex &cplex, IloEnv &env)
{
	// Set CPLEX Parameters: (DISPLAY LEVEL(0,1,2,3,4), OPTIMALITY GAP, RUN TIME (SECS), THREADS, MEMORY (MB))
	CplexParameterManager parameterManager(cplex);
	parameterManager.setParameters(2, params.BC_OptimalityGap, params.BC_TimeLimit, params.BC_NumThreads, params.BC_MemoryLimit);

	cplex.setParam(IloCplex::Param::Emphasis::MIP, 2);
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
	cplex.setParam(IloCplex::Param::MIP::Strategy::Search, CPX_MIPSEARCH_TRADITIONAL);
	cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);
	cplex.setParam(IloCplex::Param::MIP::Limits::RepairTries, 1e3);
	cplex.setParam(IloCplex::Param::Advance, 1);
}

void S2EPRP_BC::handleCplexStatus(IloCplex &cplex, IloEnv &env, IloModel &model)
{
	if (cplex.getStatus() == IloAlgorithm::Optimal)
	{
		result.status = "Optimal";
		result.objValue_Total = cplex.getObjValue();
		result.optimalityGap = cplex.getMIPRelativeGap() * 100;
		result.lowerBound = cplex.getBestObjValue();
		cout << "Optimal solution found. Objective Value = " << result.objValue_Total << endl;
	}
	else if (cplex.getStatus() == IloAlgorithm::Feasible)
	{
		result.status = "Incumbent";
		result.objValue_Total = cplex.getObjValue();
		result.optimalityGap = cplex.getMIPRelativeGap() * 100;
		result.lowerBound = cplex.getBestObjValue();
		cout << "Feasible solution found. Objective Value: " << result.objValue_Total << endl;
		cout << "LB = " << result.lowerBound << endl;
		cout << "Optimality Gap (%) = " << result.optimalityGap << endl;
	}
	else if (cplex.getStatus() == IloAlgorithm::Infeasible)
	{
		result.status = "Infeasible";
		cerr << "Problem is infeasible." << endl;
		refineConflict(cplex, env, model);
		throw std::runtime_error("Solver terminated with infeasible solution.");
	}
	else
	{
		result.status = "Undefined";
		cerr << "Solver terminated with undefined status." << endl;
	}

	if (saveSol)
	{
		string directory = "../cplexFiles/solVal/" + params.problemType + "/";
		string fileName = generateFileName(directory, ".sol");
		cplex.writeSolution(fileName.c_str());
	}
}

string S2EPRP_BC::generateFileName(const string &baseDir, const string &extension)
{
	string fileName = baseDir + "BC_" + params.problemType + "_Ins" + params.instance + "_S" + std::to_string(params.numScenarios);

	fileName += extension;

	return fileName;
}

void S2EPRP_BC::refineConflict(IloCplex &cplex, IloEnv &env, IloModel &model)
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

void S2EPRP_BC::DefineVariables(IloEnv &env, IloModel &model)
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
	// -------------------------------------------------------------------------------------------------------------------------------
	// z[i][k][t][s] variables (Binary variables to indicate whether node i (in Second Echelon) is visited by vehicle k in period t under scenario s)
	z = varManager.create4D(params.numNodes_SecondEchelon, params.numVehicles_SecondEchelon, params.numPeriods, params.numScenarios);
	for (int i = 0; i < params.numNodes_SecondEchelon; ++i)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int s = 0; s < params.numScenarios; ++s)
				{
					string varName = "z[" + std::to_string(i + 1) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "][" + std::to_string(s + 1) + "]";
					z[i][k][t][s] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
					model.add(z[i][k][t][s]);
				}
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
					for (int s = 0; s < params.numScenarios; ++s)
					{
						model.add(z[w][k][t][s] == 0);
					}
				}
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	//
	x = varManager.create4D(params.numEdges_SecondEchelon, params.numVehicles_SecondEchelon, params.numPeriods, params.numScenarios);
	for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int s = 0; s < params.numScenarios; ++s)
				{
					if (params.index_i_SecondEchelon[e] < params.numWarehouses || params.index_j_SecondEchelon[e] < params.numWarehouses)
					{
						string varName = "x[" + std::to_string(params.index_i_SecondEchelon[e] + 1) + "][" + std::to_string(params.index_j_SecondEchelon[e] + 1) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "][" + std::to_string(s + 1) + "]";
						x[e][k][t][s] = IloNumVar(env, 0.0, 2.0, IloNumVar::Int, varName.c_str());
						model.add(x[e][k][t][s]);
					}
					else
					{
						string varName = "x[" + std::to_string(params.index_i_SecondEchelon[e] + 1) + "][" + std::to_string(params.index_j_SecondEchelon[e] + 1) + "][" + std::to_string(k + 1) + "][" + std::to_string(t + 1) + "][" + std::to_string(s + 1) + "]";
						x[e][k][t][s] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
						model.add(x[e][k][t][s]);
					}
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
					for (int s = 0; s < params.numScenarios; ++s)
					{
						model.add(x[e][k][t][s] == 0);
					}
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
						for (int s = 0; s < params.numScenarios; ++s)
						{
							model.add(x[e][k][t][s] == 0);
						}
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
						for (int s = 0; s < params.numScenarios; ++s)
						{
							model.add(x[e][k][t][s] == 0);
						}
					}
				}
			}
		}
	}
}

void S2EPRP_BC::DefineObjectiveFunction(IloEnv &env, IloModel &model)
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

			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k : params.set_WarehouseVehicles[w])
				{
					for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
					{

						obj += params.probability[s] * params.transportationCost_SecondEchelon[params.index_i_SecondEchelon[e]][params.index_j_SecondEchelon[e]] * x[e][k][t][s];
					}
				}
			}
		}
	}
	model.add(IloMinimize(env, obj));
}

void S2EPRP_BC::DefineConstraints(IloEnv &env, IloModel &model)
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
		Vehicle Capacity Constraints (From Plant to Warehouse):
			sum(w in W) q[r][w][t] <= params.vehicleCapacity_Plant * o[r][t] 		for all r in R, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			string constraintName = "VehicleCapacity(" + std::to_string(routeInd + 1) + "," + std::to_string(t + 1) + ")";

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
			I[w][t][s] = I[w][t-1][s] + sum(r in R) q[w][r][t] - sum(i in N_c) sum(k in K_w) w_customer[i][k][t][s]	for all w in W, t in T, s in S
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

					for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
					{
						expr -= q[routeInd][w][t];
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

					for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
					{
						expr -= q[routeInd][w][t];
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
		Customer Visit Constraints (From Warehouse to Customer):
			w_customer[i][k][t][s] <= DeliveryUB[i][t][s] * z[i][k][t][s] 		for all i in N_c, k in union K, t in T, s in S

	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				for (int i = 0; i < params.numCustomers; ++i)
				{
					string constraintName = "CustomerVisit(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

					IloExpr expr(env);
					expr += w_customer[i][k][t][s];
					expr -= params.DeliveryUB_perCustomer[i][t][s] * z[i + params.numWarehouses][k][t][s];
					IloConstraint customerVisitConstraint(expr <= 0);
					expr.end();

					model.add(customerVisitConstraint).setName(constraintName.c_str());
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Vehicle Capacity Constraints (From Warehouse to Customer):
			sum(i in N_c) w_customer[i][k][t][s] <= Q^w * z[w][k][t][s] 		for all w in N_w, k in K_w, t in T, s in S

	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k : params.set_WarehouseVehicles[w])
				{
					string constraintName = "VehicleCapacity(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

					IloExpr expr(env);
					for (int i = 0; i < params.numCustomers; ++i)
					{
						expr += w_customer[i][k][t][s];
					}
					expr -= params.vehicleCapacity_Warehouse * z[w][k][t][s];
					IloConstraint vehicleCapacityConstraint(expr <= 0);
					expr.end();

					model.add(vehicleCapacityConstraint).setName(constraintName.c_str());
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Prevent Split Deliveries:
			sum(k in union K) z[i][k][t][s] <= 1 		for all i in N_c, k in union K, t in T, s in S

	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				string constraintName = "PreventSplitDeliveries(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				IloExpr expr(env);
				for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
				{
					expr += z[i + params.numWarehouses][k][t][s];
				}
				IloConstraint preventSplitDeliveriesConstraint(expr <= 1);
				expr.end();

				model.add(preventSplitDeliveriesConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Degree Constraints:
			sum((j,j') in incident (i)) x[j][j'][k][t][s] == 2 * z[i][k][t][s] 		for all i in N_w + N_c, k in union K, t in T, s in S

	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				for (int i = 0; i < params.numNodes_SecondEchelon; ++i)
				{
					string constraintName = "DegreeConstraint(" + std::to_string(i + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

					IloExpr expr(env);
					for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
					{
						if (params.index_i_SecondEchelon[e] == i || params.index_j_SecondEchelon[e] == i)
						{
							expr += x[e][k][t][s];
						}
					}
					expr -= 2 * z[i][k][t][s];
					IloConstraint degreeConstraint(expr == 0);
					expr.end();

					model.add(degreeConstraint).setName(constraintName.c_str());
				}
			}
		}
	}
	/* End of Model Constraints */
}

void S2EPRP_BC::DefineValidInequalities(IloEnv &env, IloModel &model)
{
	// ---------------------------------------------------------------------------------------------------------------------------
	/* Define Valid Inequalities */
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Logical Inequality (#1):
			i can be visited by vehicle k iff k leaves the warehouse w
			z[i][k][t][s] <= z[w][k][t][s] for all i in N_c, w in N_w, k in K_w t in T, s in S
	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k : params.set_WarehouseVehicles[w])
				{
					for (int i = 0; i < params.numCustomers; ++i)
					{
						string constraintName = "LogicalInequality_One(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

						IloExpr expr(env);
						expr += z[i + params.numWarehouses][k][t][s];
						expr -= z[w][k][t][s];
						IloConstraint logicalInequality_One(expr <= 0);
						expr.end();

						model.add(logicalInequality_One).setName(constraintName.c_str());
					}
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Logical Inequality (#2):
		x[i][j][k][t][s] <= z[i][k][t][s] for all i in N_w + N_c, j in N_w + N_c, k in K_w, t in T, s in S
		x[i][j][k][t][s] <= z[j][k][t][s] for all i in N_w + N_c, j in N_w + N_c, k in K_w, t in T, s in S
	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k : params.set_WarehouseVehicles[w])
				{
					for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
					{
						if (params.index_i_SecondEchelon[e] >= params.numWarehouses && params.index_j_SecondEchelon[e] >= params.numWarehouses)
						{
							string constraintName = "LogicalInequality_TwoSetOne(" + std::to_string(e) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

							IloExpr expr(env);
							expr += x[e][k][t][s];
							expr -= z[params.index_i_SecondEchelon[e]][k][t][s];
							IloConstraint logicalInequality_TwoSetOne(expr <= 0);
							expr.end();

							model.add(logicalInequality_TwoSetOne).setName(constraintName.c_str());
						}
					}
				}
			}
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k : params.set_WarehouseVehicles[w])
				{
					for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
					{
						if (params.index_i_SecondEchelon[e] >= params.numWarehouses && params.index_j_SecondEchelon[e] >= params.numWarehouses)
						{
							string constraintName = "LogicalInequality_TwoSetTwo(" + std::to_string(e) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

							IloExpr expr(env);
							expr += x[e][k][t][s];
							expr -= z[params.index_j_SecondEchelon[e]][k][t][s];
							IloConstraint logicalInequality_TwoSetTwo(expr <= 0);
							expr.end();

							model.add(logicalInequality_TwoSetTwo).setName(constraintName.c_str());
						}
					}
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Symmetry Breaking Constraints (#1):
			k can be dispatched iff k - 1 is already dispatched
			z[w][k][t][s] <= z[w][k - 1][t][s] for all w in N_w, k in K_w t in T, s in S
	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				if (params.set_WarehouseVehicles[w].size() > 1)
				{
					for (int k = params.set_WarehouseVehicles[w][1]; k < params.set_WarehouseVehicles[w][params.set_WarehouseVehicles[w].size() - 1]; ++k)
					{
						string constraintName = "SymmetryBreakingConstraint_One(" + std::to_string(w + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

						IloExpr expr(env);
						expr += z[w][k][t][s];
						expr -= z[w][k - 1][t][s];
						IloConstraint symmetryBreakingConstraint_One(expr <= 0);
						expr.end();

						model.add(symmetryBreakingConstraint_One).setName(constraintName.c_str());
					}
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Lexicographic Constraints:
			sum(i = 1 to j) 2**(j - i) * z[i][k][t][s] <= sum(i = 1 to j) 2**(j - i) * z[i][k - 1][t][s] for all j in N_c, k in K_w, t in T, s in S
	*/
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int k = params.set_WarehouseVehicles[w][1]; k < params.set_WarehouseVehicles[w][params.set_WarehouseVehicles[w].size() - 1]; ++k)
				{
					if (params.set_WarehouseVehicles[w].size() > 1)
					{
						for (int j = params.numWarehouses; j < params.numCustomers + params.numWarehouses; ++j)
						{
							string constraintName = "LexicographicConstraint(" + std::to_string(j + 1) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

							IloExpr expr(env);
							for (int i = params.numWarehouses; i <= j; i++)
							{
								expr += pow(2, (j - i)) * z[i][k][t][s];
								expr -= pow(2, (j - i)) * z[i][k - 1][t][s];
							}

							IloConstraint LexicographicConstraint(expr <= 0);
							expr.end();

							model.add(LexicographicConstraint).setName(constraintName.c_str());
						}
					}
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/* End of Valid Inequalities */
	// ---------------------------------------------------------------------------------------------------------------------------
}

void S2EPRP_BC::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	solFE.productionSetup.assign(params.numPeriods, 0);
	solFE.productionQuantity.assign(params.numPeriods, 0.0);
	solFE.plantInventory.assign(params.numPeriods, 0.0);
	solFE.deliveryQuantityToWarehouse.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	solFE.routesPlantToWarehouse.assign(params.numPeriods, vector<vector<int>>(params.numVehicles_Plant, vector<int>()));

	solSE.warehouseInventory.assign(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	solSE.customerInventory.assign(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	solSE.customerUnmetDemand.assign(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	solSE.deliveryQuantityToCustomer.assign(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	solSE.routesWarehouseToCustomer.assign(params.numScenarios, vector<vector<vector<vector<int>>>>(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>()))));
	solSE.customerAssignmentToWarehouse.assign(params.numScenarios, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numWarehouses, vector<int>(params.numCustomers, 0))));

	selectedRoute.assign(numRoutes_FirstEchelon, vector<int>(params.numPeriods, 0));
	vector<vector<vector<vector<int>>>> visitedNodes_SecondEchelon(params.numNodes_SecondEchelon, vector<vector<vector<int>>>(params.numVehicles_SecondEchelon, vector<vector<int>>(params.numPeriods, vector<int>(params.numScenarios, 0))));
	vector<vector<vector<vector<int>>>> visitedEdges_SecondEchelon(params.numEdges_SecondEchelon, vector<vector<vector<int>>>(params.numVehicles_SecondEchelon, vector<vector<int>>(params.numPeriods, vector<int>(params.numScenarios, 0))));

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
				// solFE.routesPlantToWarehouse[t][routeInd] = optimalRoutes_FirstEchelon[routeInd];
			}
		}

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int s = 0; s < params.numScenarios; ++s)
			{
				solSE.warehouseInventory[w][t][s] = cplex.getValue(I_warehouse[w][t][s]);
			}
		}

		for (int i = 0; i < params.numCustomers; ++i)
		{
			for (int s = 0; s < params.numScenarios; ++s)
			{
				solSE.customerInventory[i][t][s] = cplex.getValue(I_customer[i][t][s]);
				solSE.customerUnmetDemand[i][t][s] = cplex.getValue(b_customer[i][t][s]);
				for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
				{
					solSE.deliveryQuantityToCustomer[i][t][s] += cplex.getValue(w_customer[i][k][t][s]);
				}
			}
		}

		for (int i = 0; i < params.numNodes_SecondEchelon; ++i)
		{
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				for (int s = 0; s < params.numScenarios; ++s)
				{
					visitedNodes_SecondEchelon[i][k][t][s] = cplex.getIntValue(z[i][k][t][s]);
				}
			}
		}

		for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
		{
			for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
			{
				for (int s = 0; s < params.numScenarios; ++s)
				{
					visitedEdges_SecondEchelon[e][k][t][s] = cplex.getIntValue(x[e][k][t][s]);
				}
			}
		}
	}

	// print x
	// for (int s = 0; s < params.numScenarios; ++s)
	// {
	// 	for (int t = 0; t < params.numPeriods; ++t)
	// 	{
	// 		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
	// 		{
	// 			for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
	// 			{
	// 				if (visitedEdges_SecondEchelon[e][k][t][s] > 0)
	// 				{
	// 					cout << "x[" << params.index_i_SecondEchelon[e] << "][" << params.index_j_SecondEchelon[e] << "][" << k + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << visitedEdges_SecondEchelon[e][k][t][s] << endl;
	// 				}
	// 			}
	// 		}
	// 	}
	// }

	// for (int s = 0; s < params.numScenarios; ++s)
	// {
	// 	for (int t = 0; t < params.numPeriods; ++t)
	// 	{
	// 		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
	// 		{
	// 			for (int i = 0; i < params.numNodes_SecondEchelon; ++i)
	// 			{
	// 				if (visitedNodes_SecondEchelon[i][k][t][s] > 0)
	// 				{
	// 					cout << "z[" << i << "][" << k + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << visitedNodes_SecondEchelon[i][k][t][s] << endl;
	// 				}
	// 			}
	// 		}
	// 	}
	// }

	// Check split deliveries
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				int sumVisitedVehicles = 0;
				for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
				{
					sumVisitedVehicles += visitedNodes_SecondEchelon[i + params.numWarehouses][k][t][s];
				}
				if (sumVisitedVehicles > 1)
				{
					cout << "Split deliveries for customer " << i << " at time " << t << " and scenario " << s << endl;
					exit(0);
				}
			}
		}
	}

	auto visitEdge_Copy = visitedEdges_SecondEchelon;
	// find and save the routes for second echelon
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				int vehInd = 0;
				for (int k : params.set_WarehouseVehicles[w])
				{
					vector<int> route;
					if (visitedNodes_SecondEchelon[w][k][t][s] == 1)
					{
						// int routeSize = 0;
						// for (int i = 0; i < params.numNodes_SecondEchelon; ++i)
						// {
						// 	if (visitedNodes_SecondEchelon[i][k][t][s] == 1)
						// 	{
						// 		routeSize++;
						// 		cout << "visitedNodes_SecondEchelon[" << i + 1 << "][" << k + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << visitedNodes_SecondEchelon[i][k][t][s] << endl;
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
								if (params.index_i_SecondEchelon[e] == currentNode && !visited[params.index_j_SecondEchelon[e]] && visitEdge_Copy[e][k][t][s] == 1)
								{
									// cout << "x[" << params.index_i_SecondEchelon[e] + 1 << "][" << params.index_j_SecondEchelon[e] + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << visitEdge_Copy[e][k][t][s] << endl;
									route.push_back(params.index_j_SecondEchelon[e]);
									currentNode = params.index_j_SecondEchelon[e];
									visited[params.index_j_SecondEchelon[e]] = true;
									visitEdge_Copy[e][k][t][s] = 0;
									break;
								}
								else if (params.index_j_SecondEchelon[e] == currentNode && !visited[params.index_i_SecondEchelon[e]] && visitEdge_Copy[e][k][t][s] == 1)
								{
									// cout << "x[" << params.index_i_SecondEchelon[e] + 1 << "][" << params.index_j_SecondEchelon[e] + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << visitEdge_Copy[e][k][t][s] << endl;
									route.push_back(params.index_i_SecondEchelon[e]);
									currentNode = params.index_i_SecondEchelon[e];
									visited[params.index_i_SecondEchelon[e]] = true;
									visitEdge_Copy[e][k][t][s] = 0;
									break;
								}
								else if (params.index_i_SecondEchelon[e] == currentNode && !visited[params.index_j_SecondEchelon[e]] && visitEdge_Copy[e][k][t][s] == 2)
								{
									// cout << "x[" << params.index_i_SecondEchelon[e] + 1 << "][" << params.index_j_SecondEchelon[e] + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << visitEdge_Copy[e][k][t][s] << endl;
									route.push_back(params.index_j_SecondEchelon[e]);
									currentNode = params.index_j_SecondEchelon[e];
									visited[params.index_j_SecondEchelon[e]] = true;
									visitEdge_Copy[e][k][t][s] = 1;
									break;
								}
							}
							// cout << "currentNode = " << currentNode + 1 << endl;
						}
						solSE.routesWarehouseToCustomer[s][w][t][vehInd] = route;
						vehInd++;
					}
				}
			}
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int i = 0; i < params.numCustomers; ++i)
				{
					for (int k : params.set_WarehouseVehicles[w])
					{
						if (visitedNodes_SecondEchelon[i + params.numWarehouses][k][t][s] == 1)
						{
							// cout << "visitedNodes_SecondEchelon[" << i << "][" << k + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << visitedNodes_SecondEchelon[i + params.numWarehouses][k][t][s] << endl;
							// cout << "Customer " << i + 1 + params.numWarehouses  << " is assigned to Warehouse " << w << " on Period " << t + 1 << " with Vehicle " << k + 1 << endl;
							solSE.customerAssignmentToWarehouse[s][t][w][i] = 1;
							break;
						}
					}
				}
			}
		}
	}
}

void S2EPRP_BC::CalculateCostsForEachPart()
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

		for (int s = 0; s < params.numScenarios; ++s)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				solSE.holdingCostWarehouse_Avg += params.probability[s] * params.unitHoldingCost_Warehouse[w] * solSE.warehouseInventory[w][t][s];
			}

			for (int i = 0; i < params.numCustomers; ++i)
			{
				solSE.holdingCostCustomer_Avg += params.probability[s] * params.unitHoldingCost_Customer[i] * solSE.customerInventory[i][t][s];
				solSE.costOfUnmetDemand_Avg += params.probability[s] * params.unmetDemandPenalty[i] * solSE.customerUnmetDemand[i][t][s];
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
					if (!solSE.routesWarehouseToCustomer[s][w][t][k].empty())
					{
						int previousNode = w;
						for (int j = 1; j < solSE.routesWarehouseToCustomer[s][w][t][k].size(); ++j)
						{
							int currentNode = solSE.routesWarehouseToCustomer[s][w][t][k][j];

							solSE.transportationCostWarehouseToCustomer_Avg += params.probability[s] * params.transportationCost_SecondEchelon[previousNode][currentNode];

							previousNode = currentNode;
						}
					}
				}
			}
		}
	}
	cout << "setupCost (BC) = " << solFE.setupCost << endl;
	cout << "productionCost (BC) = " << solFE.productionCost << endl;
	cout << "holdingCostPlant (BC) = " << solFE.holdingCostPlant << endl;
	cout << "transportationCostPlantToWarehouse (BC) = " << solFE.transportationCostPlantToWarehouse << endl;

	result.objValue_firstEchelon = solFE.setupCost + solFE.productionCost + solFE.holdingCostPlant + solFE.transportationCostPlantToWarehouse;

	cout << "Avg holding cost warehouse (BC) : " << solSE.holdingCostWarehouse_Avg << endl;
	cout << "Avg holding cost customer (BC) : " << solSE.holdingCostCustomer_Avg << endl;
	cout << "Avg cost of unmet demand (BC) : " << solSE.costOfUnmetDemand_Avg << endl;
	cout << "Avg transportation cost warehouse to customer (BC) : " << solSE.transportationCostWarehouseToCustomer_Avg << endl;
	result.objValue_secondEchelon = solSE.holdingCostWarehouse_Avg + solSE.holdingCostCustomer_Avg + solSE.costOfUnmetDemand_Avg + solSE.transportationCostWarehouseToCustomer_Avg;

	result.objValue_Total = result.objValue_firstEchelon + result.objValue_secondEchelon;

	cout << "objValue_FE (BC) = " << result.objValue_firstEchelon << endl;
	cout << "objValue_FE (BC) : " << result.objValue_secondEchelon << endl;
	cout << "objValue_Total (BC) : " << result.objValue_Total << endl;
}

void S2EPRP_BC::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << solFE.productionSetup[t] << endl;
		}
	}
}

void S2EPRP_BC::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << solFE.productionQuantity[t] << endl;
		}
	}
}

void S2EPRP_BC::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << solFE.plantInventory[t] << endl;
		}
	}
}

void S2EPRP_BC::DisplayWarehouseInventoryVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				if (solSE.warehouseInventory[w][t][s] > THRESHOLD)
				{
					cout << "I_warehouse[" << w + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << std::setprecision(0) << std::fixed << solSE.warehouseInventory[w][t][s] << endl;
				}
			}
		}
	}
}

void S2EPRP_BC::DisplayFirstEchelonRouteVars()
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

void S2EPRP_BC::DisplayDeliveryQuantityToWarehousesVars()
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

void S2EPRP_BC::DisplayCustomerInventoryVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (solSE.customerInventory[i][t][s] > THRESHOLD)
				{
					cout << "I_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << solSE.customerInventory[i][t][s] << endl;
				}
			}
		}
	}
}

void S2EPRP_BC::DisplayCustomerUnmetDemandVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (solSE.customerUnmetDemand[i][t][s] > THRESHOLD)
				{
					cout << "b_customer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << solSE.customerUnmetDemand[i][t][s] << endl;
				}
			}
		}
	}
}

void S2EPRP_BC::DisplayDeliveryQuantityToCustomersVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (solSE.deliveryQuantityToCustomer[i][t][s] > THRESHOLD)
				{
					cout << "deliveryQuantityToCustomer[" << i + params.numWarehouses + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << solSE.deliveryQuantityToCustomer[i][t][s] << endl;
				}
			}
		}
	}
}

void S2EPRP_BC::DisplayRoutesWarehouseToCustomersVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; ++k)
				{
					if (!solSE.routesWarehouseToCustomer[s][w][t][k].empty())
					{
						cout << "route[" << s + 1 << "][" << w + 1 << "][" << t + 1 << "][" << k + 1 << "] : [";
						for (auto it = solSE.routesWarehouseToCustomer[s][w][t][k].begin(); it != solSE.routesWarehouseToCustomer[s][w][t][k].end(); ++it)
						{
							if (it != solSE.routesWarehouseToCustomer[s][w][t][k].begin())
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
}

void S2EPRP_BC::DefineWarmStartSolution(IloEnv &env, IloCplex &cplex)
{
	IloNumVarArray startVar(env);
	IloNumArray startVal(env);

	// Define warm start solution
	// Production setup
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

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; k++)
				{
					int vehicleIndex = k + (w * params.numVehicles_Warehouse);
					vector<int> &route = warmStart.routesWarehouseToCustomer_WarmStart[s][w][t][k];
					if (!route.empty())
					{
						startVar.add(z[w][vehicleIndex][t][s]);
						startVal.add(1);
						for (int i = 0; i < params.numCustomers; ++i)
						{
							int customerIndex = i + params.numWarehouses;
							auto it = std::find(route.begin(), route.end(), customerIndex);
							if (it != route.end())
							{
								startVar.add(z[customerIndex][vehicleIndex][t][s]);
								startVal.add(1);
							}
							else
							{
								startVar.add(z[customerIndex][vehicleIndex][t][s]);
								startVal.add(0);
							}
						}
					}
					else
					{
						startVar.add(z[w][vehicleIndex][t][s]);
						startVal.add(0);
						for (int i = 0; i < params.numCustomers; ++i)
						{
							int customerIndex = i + params.numWarehouses;
							startVar.add(z[customerIndex][vehicleIndex][t][s]);
							startVal.add(0);
						}
					}
				}
			}
		}
	}

	vector<vector<vector<vector<int>>>> x_warmStart(params.numEdges_SecondEchelon, vector<vector<vector<int>>>(params.numVehicles_SecondEchelon, vector<vector<int>>(params.numPeriods, vector<int>(params.numScenarios, 0))));
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int k = 0; k < params.numVehicles_Warehouse; k++)
				{
					int vehicleIndex = k + (w * params.numVehicles_Warehouse);
					vector<int> &route = warmStart.routesWarehouseToCustomer_WarmStart[s][w][t][k];
					if (route.size() > 3)
					{
						int prevNode = w;
						for (size_t i = 1; i < route.size(); ++i)
						{
							int currentNode = route[i];

							x_warmStart[params.index_e_SecondEchelon[prevNode][currentNode]][vehicleIndex][t][s] = 1;

							prevNode = currentNode;
						}
					}
					else if (route.size() == 3)
					{
						int prevNode = w;
						int currentNode = route[1];

						x_warmStart[params.index_e_SecondEchelon[prevNode][currentNode]][vehicleIndex][t][s] = 2;
					}
				}
			}
		}
	}

	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int k = 0; k < params.numVehicles_SecondEchelon; k++)
			{
				for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
				{
					startVar.add(x[e][k][t][s]);
					startVal.add(x_warmStart[e][k][t][s]);
				}
			}
		}
	}

	cplex.addMIPStart(startVar, startVal, IloCplex::MIPStartEffort::MIPStartRepair);
	startVal.end();
	startVar.end();
}
