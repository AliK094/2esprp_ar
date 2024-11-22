#include "BC_Deterministic.h"

BC_Deterministic::BC_Deterministic(const ParameterSetting &parameters,
								   const SolutionWarmStart_Deterministic &warmStartSol,
								   const SolutionFirstEchelon &sol_FE,
								   bool saveModel,
								   bool saveSolution)
	: params(parameters),
	  warmStart(warmStartSol),
	  sol_FE_EV(sol_FE),
	  saveLP(saveModel),
	  saveSol(saveSolution),
	  THRESHOLD(1e-2)
{
	if (params.problemType != "EEV")
	{
		routeMatrix_FirstEchelon = params.getRouteMatrix();
		numRoutes_FirstEchelon = routeMatrix_FirstEchelon.size();
		optimalRoutes_FirstEchelon = params.getOptimalRoutes();
		routeCosts_FirstEchelon = params.getRouteCosts();
	}
	else if (params.problemType == "EEV")
	{
		warehouse_delivery.resize(params.numWarehouses, vector<vector<double>>(params.numVehicles_Plant, vector<double>(params.numPeriods, 0.0)));
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
	}

	if (params.problemType == "EV" || params.problemType == "EEV" || params.problemType == "WS")
		shortageAllowed = true;
}

bool BC_Deterministic::Solve()
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

		cout << "Solving BC_" << params.problemType << "..." << endl;
		SEC_2EPRP LegacyCallback(env, params, x, z);
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

		cplex.extract(model);

		if (!warmStart.empty())
		{
			DefineWarmStartSolution(env, cplex);
		}
		else
		{
			cout << "Couldn't find a solution for warm start." << endl;
		}

		cplex.solve();
		handleCplexStatus(cplex, env, model);

		if (result.status == "Optimal" || result.status == "Incumbent")
		{
			RetrieveSolutions(cplex);
			CalculateCostsForEachPart();
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
		cout << "Timer (seconds): " << result.totalCPUTime << endl;

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

void BC_Deterministic::configureCplex(IloCplex &cplex, IloEnv &env)
{
	// Set CPLEX Parameters: (DISPLAY LEVEL(0,1,2,3,4), OPTIMALITY GAP, RUN TIME (SECS), THREADS, MEMORY (MB))
	CplexParameterManager parameterManager(cplex);
	parameterManager.setParameters(4, 1e-6, 300, 4, 32000);

	cplex.setParam(IloCplex::Param::Emphasis::MIP, 2);
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
	cplex.setParam(IloCplex::Param::MIP::Strategy::Search, CPX_MIPSEARCH_TRADITIONAL);
	cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);
	cplex.setParam(IloCplex::Param::MIP::Limits::RepairTries, 1e6);
	cplex.setParam(IloCplex::Param::Advance, 1);
}

void BC_Deterministic::handleCplexStatus(IloCplex &cplex, IloEnv &env, IloModel &model)
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
		cout << "LB = " << result.lowerBound;
		cout << "Optimality Gap (%) = " << result.optimalityGap;
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

string BC_Deterministic::generateFileName(const string &baseDir, const string &extension)
{
	string fileName = baseDir + "BC_" + params.problemType + "_Ins" + params.instance;

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

void BC_Deterministic::refineConflict(IloCplex &cplex, IloEnv &env, IloModel &model)
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

void BC_Deterministic::DefineVariables(IloEnv &env, IloModel &model)
{
	/* Define Decision Variables */
	if (params.problemType != "EEV")
	{
		DefineFirstStageVars(env, model);
	}
	DefineSecondStageVars(env, model);
}

void BC_Deterministic::DefineFirstStageVars(IloEnv &env, IloModel &model)
{
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
	// q[r][w][t] variables (Delivery quantity to warehouse w in period t from route r)
	q = varManager.create3D(numRoutes_FirstEchelon, params.numWarehouses, params.numPeriods);
	for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				string varName = "q[" + std::to_string(routeInd + 1) + "][" + std::to_string(w + 1) + "][" + std::to_string(t + 1) + "]";

				double upperBound =routeMatrix_FirstEchelon[routeInd][w + 1] * params.storageCapacity_Warehouse[w];

				q[routeInd][w][t] = IloNumVar(env, 0.0, upperBound, IloNumVar::Float, varName.c_str());

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
}

void BC_Deterministic::DefineSecondStageVars(IloEnv &env, IloModel &model)
{
	// Initialize Variable Manager
	VariableManager varManager(env);
	// -------------------------------------------------------------------------------------------------------------------------------
	if (params.problemType != "2EPRPCS")
	{
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
	if (shortageAllowed)
	{
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

void BC_Deterministic::DefineObjectiveFunction(IloEnv &env, IloModel &model)
{
	// Define objective function
	IloExpr obj(env);

	// First Echelon (not for EEV)
	if (params.problemType != "EEV")
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			obj += params.setupCost * y[t];
			obj += params.unitProdCost * p[t];
			obj += params.unitHoldingCost_Plant * I_plant[t];
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				obj += routeCosts_FirstEchelon[routeInd] * o[routeInd][t];
			}
		}
	}

	if (params.problemType != "2EPRPCS")
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				obj += params.unitHoldingCost_Warehouse[w] * I_warehouse[w][t];
			}
		}
	}
	else if (params.problemType == "2EPRPCS")
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

	if (shortageAllowed)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				obj += params.unmetDemandPenalty[i] * b_customer[i][t];
			}
		}
	}

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			obj += params.unitHoldingCost_Customer[i] * I_customer[i][t];
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

void BC_Deterministic::DefineConstraints(IloEnv &env, IloModel &model)
{
	/* Define Constraints */
	if (params.problemType != "EEV")
	{
		DefCons_ProductionCapacity(env, model);
		DefCons_PlantInventoryCapacity(env, model);
		DefCons_PlantInventoryBalance(env, model);
		DefCons_WarehouseCoverage_FirstEchelon(env, model);
		DefCons_VehicleCapacity_FirstEchelon(env, model);
		DefCons_FleetSize_FirstEchelon(env, model);
		DefCons_WarehouseVisit_FirstEchelon(env, model);
	}

	if (params.problemType == "2EPRPCS")
	{
		DefCons_SatelliteInventoryBalance(env, model);
	}
	else if (params.problemType == "EEV")
	{
		DefCons_WarehouseInventoryBalance_EEV(env, model);
	}
	else
	{
		DefCons_WarehouseInventoryCapacity(env, model);
		DefCons_WarehouseInventoryBalance(env, model);
	}

	DefCons_CustomerInventoryCapacity(env, model);
	DefCons_CustomerInventoryBalance(env, model);
	DefCons_CustomerVisit_SecondEchelon(env, model);
	DefCons_VehicleCapacity_SecondEchelon(env, model);
	DefCons_SplitDeliveries_SecondEchelon(env, model);
	DefCons_Degree_SecondEchelon(env, model);

	/* Define Valid Inequalities */
	DefCons_ValidInequalities_LogicalOne(env, model);
	DefCons_ValidInequalities_LogicalTwo(env, model);
	DefCons_ValidInequalities_SymmetryBreakingOne(env, model);
	DefCons_ValidInequalities_Lexicographic(env, model);
}

void BC_Deterministic::DefCons_ProductionCapacity(IloEnv &env, IloModel &model)
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
		expr -= params.prodCapacity * y[t];
		IloConstraint productionCapacityConstraint(expr <= 0);
		expr.end();

		model.add(productionCapacityConstraint).setName(constraintName.c_str());
	}
}

void BC_Deterministic::DefCons_PlantInventoryCapacity(IloEnv &env, IloModel &model)
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

void BC_Deterministic::DefCons_PlantInventoryBalance(IloEnv &env, IloModel &model)
{
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
}

void BC_Deterministic::DefCons_WarehouseCoverage_FirstEchelon(IloEnv &env, IloModel &model)
{
	/*
		Each Warehouse must be covered by at most one route.
			sum(r in R) a[r][w] * o[r][t] <= 1			for all w in w and t in T
	*/
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
}

void BC_Deterministic::DefCons_WarehouseVisit_FirstEchelon(IloEnv &env, IloModel &model)
{
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
				
				expr -= o[routeInd][t] * routeMatrix_FirstEchelon[routeInd][w + 1] * params.storageCapacity_Warehouse[w] ;
				IloConstraint warehouseVisitConstraint(expr <= 0);
				expr.end();

				model.add(warehouseVisitConstraint).setName(constraintName.c_str());
			}
		}
	}
}

void BC_Deterministic::DefCons_VehicleCapacity_FirstEchelon(IloEnv &env, IloModel &model)
{
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
}

void BC_Deterministic::DefCons_FleetSize_FirstEchelon(IloEnv &env, IloModel &model)
{
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
}

void BC_Deterministic::DefCons_WarehouseInventoryCapacity(IloEnv &env, IloModel &model)
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

void BC_Deterministic::DefCons_WarehouseInventoryBalance(IloEnv &env, IloModel &model)
{
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
}

void BC_Deterministic::DefCons_WarehouseInventoryBalance_EEV(IloEnv &env, IloModel &model)
{
	/*
		Warehouse Inventory Balance Constraints:
			I[w][t] = I[w][t-1] + sum(r in R) q[w][r][t] - sum(i in N_c) sum(k in K_w) w_customer[i][k][t]	for all w in W, t in T
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

void BC_Deterministic::DefCons_SatelliteInventoryBalance(IloEnv &env, IloModel &model)
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
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				expr += q[routeInd][w][t];
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

void BC_Deterministic::DefCons_CustomerInventoryCapacity(IloEnv &env, IloModel &model)
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

void BC_Deterministic::DefCons_CustomerInventoryBalance(IloEnv &env, IloModel &model)
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

				if (shortageAllowed)
				{
					expr -= b_customer[i][t];
				}
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

				if (shortageAllowed)
				{
					expr -= b_customer[i][t];
				}

				IloConstraint CustomerInventoryBalanceConstraint(expr == -params.demand_Deterministic[i][t]);
				expr.end();

				model.add(CustomerInventoryBalanceConstraint).setName(constraintName.c_str());
			}
		}
	}
}

void BC_Deterministic::DefCons_CustomerVisit_SecondEchelon(IloEnv &env, IloModel &model)
{
	/*
		Customer Visit Constraints (From Warehouse to Customer):
			w_customer[i][k][t] <= DeliveryUB[i][t] * z[i][k][t] 		for all i in N_c, k in union K, t in T

	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int k = 0; k < params.numVehicles_SecondEchelon; ++k)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				string constraintName = "CustomerVisit(" + std::to_string(i + 1 + params.numWarehouses) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

				IloExpr expr(env);
				expr += w_customer[i][k][t];
				expr -= params.DelUB_perCus_Det[i][t] * z[i + params.numWarehouses][k][t];
				IloConstraint customerVisitConstraint(expr <= 0);
				expr.end();

				model.add(customerVisitConstraint).setName(constraintName.c_str());
			}
		}
	}
}

void BC_Deterministic::DefCons_VehicleCapacity_SecondEchelon(IloEnv &env, IloModel &model)
{
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
}

void BC_Deterministic::DefCons_SplitDeliveries_SecondEchelon(IloEnv &env, IloModel &model)
{
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
}

void BC_Deterministic::DefCons_Degree_SecondEchelon(IloEnv &env, IloModel &model)
{
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

void BC_Deterministic::DefCons_ValidInequalities_LogicalOne(IloEnv &env, IloModel &model)
{
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
}

void BC_Deterministic::DefCons_ValidInequalities_LogicalTwo(IloEnv &env, IloModel &model)
{
	/*
		Logical Inequality (#2):
		x[i][j][k][t] <= z[i][k][t] for all i in N_w + N_c, j in N_w + N_c, k in K_w, t in T
		x[i][j][k][t] <= z[j][k][t] for all i in N_w + N_c, j in N_w + N_c, k in K_w, t in T
	*/
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
						string constraintName = "LogicalInequality_TwoSetOne(" + std::to_string(e) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

						IloExpr expr(env);
						expr += x[e][k][t];
						expr -= z[params.index_i_SecondEchelon[e]][k][t];
						IloConstraint logicalInequality_TwoSetOne(expr <= 0);
						expr.end();

						model.add(logicalInequality_TwoSetOne).setName(constraintName.c_str());
					}
				}
			}
		}
	}

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
						string constraintName = "LogicalInequality_TwoSetTwo(" + std::to_string(e) + "," + std::to_string(k + 1) + "," + std::to_string(t + 1) + ")";

						IloExpr expr(env);
						expr += x[e][k][t];
						expr -= z[params.index_j_SecondEchelon[e]][k][t];
						IloConstraint logicalInequality_TwoSetTwo(expr <= 0);
						expr.end();

						model.add(logicalInequality_TwoSetTwo).setName(constraintName.c_str());
					}
				}
			}
		}
	}
}

void BC_Deterministic::DefCons_ValidInequalities_SymmetryBreakingOne(IloEnv &env, IloModel &model)
{
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
}

void BC_Deterministic::DefCons_ValidInequalities_Lexicographic(IloEnv &env, IloModel &model)
{
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
}

void BC_Deterministic::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	if (params.problemType != "EEV")
	{
		solFE.productionSetup.assign(params.numPeriods, 0);
		solFE.productionQuantity.assign(params.numPeriods, 0.0);
		solFE.plantInventory.assign(params.numPeriods, 0.0);
		solFE.deliveryQuantityToWarehouse.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
		solFE.routesPlantToWarehouse.assign(params.numPeriods, vector<vector<int>>(params.numVehicles_Plant, vector<int>()));
		selectedRoute.assign(numRoutes_FirstEchelon, vector<int>(params.numPeriods, 0));

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
		}
	}
	else
	{
		solFE.productionSetup = sol_FE_EV.productionSetup;
		solFE.productionQuantity = sol_FE_EV.productionQuantity;
		solFE.plantInventory = sol_FE_EV.plantInventory;
		solFE.deliveryQuantityToWarehouse = sol_FE_EV.deliveryQuantityToWarehouse;
		solFE.routesPlantToWarehouse = sol_FE_EV.routesPlantToWarehouse;
	}

	if (params.problemType != "2EPRPCS")
	{
		solSE.warehouseInventory.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				solSE.warehouseInventory[w][t] = cplex.getValue(I_warehouse[w][t]);
			}
		}
	}

	solSE.customerInventory.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	if (shortageAllowed)
	{
		solSE.customerUnmetDemand.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				solSE.customerUnmetDemand[i][t] = cplex.getValue(b_customer[i][t]);
			}
		}
	}

	solSE.deliveryQuantityToCustomer.assign(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	solSE.routesWarehouseToCustomer.assign(params.numWarehouses, vector<vector<vector<int>>>(params.numPeriods, vector<vector<int>>(params.numVehicles_Warehouse, vector<int>())));
	solSE.customerAssignmentToWarehouse.assign(params.numPeriods, vector<vector<int>>(params.numWarehouses, vector<int>(params.numCustomers, 0)));

	vector<vector<vector<int>>> visitedNodes_SecondEchelon(params.numNodes_SecondEchelon, vector<vector<int>>(params.numVehicles_SecondEchelon, vector<int>(params.numPeriods, 0)));
	vector<vector<vector<int>>> visitedEdges_SecondEchelon(params.numEdges_SecondEchelon, vector<vector<int>>(params.numVehicles_SecondEchelon, vector<int>(params.numPeriods, 0)));

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			solSE.customerInventory[i][t] = cplex.getValue(I_customer[i][t]);
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

	checkSplitDeliveries(visitedNodes_SecondEchelon);
	adjustRoutesSE(visitedNodes_SecondEchelon, visitedEdges_SecondEchelon);
}

void BC_Deterministic::checkSplitDeliveries(vector<vector<vector<int>>>& visitedNodes_SecondEchelon)
{
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
				exit(1);
			}
		}
	}
}

void BC_Deterministic::adjustRoutesSE(vector<vector<vector<int>>>& visitedNodes_SecondEchelon, vector<vector<vector<int>>>& visitedEdges_SecondEchelon)
{
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
					route.push_back(w);
					int currentNode = w;
					vector<bool> visited(params.numNodes_SecondEchelon, false);

					while (!visited[w])
					{
						for (int e = 0; e < params.numEdges_SecondEchelon; ++e)
						{
							if (params.index_i_SecondEchelon[e] == currentNode && !visited[params.index_j_SecondEchelon[e]] && visitEdge_Copy[e][k][t] == 1)
							{
								route.push_back(params.index_j_SecondEchelon[e]);
								currentNode = params.index_j_SecondEchelon[e];
								visited[params.index_j_SecondEchelon[e]] = true;
								visitEdge_Copy[e][k][t] = 0;
								break;
							}
							else if (params.index_j_SecondEchelon[e] == currentNode && !visited[params.index_i_SecondEchelon[e]] && visitEdge_Copy[e][k][t] == 1)
							{
								route.push_back(params.index_i_SecondEchelon[e]);
								currentNode = params.index_i_SecondEchelon[e];
								visited[params.index_i_SecondEchelon[e]] = true;
								visitEdge_Copy[e][k][t] = 0;
								break;
							}
							else if (params.index_i_SecondEchelon[e] == currentNode && !visited[params.index_j_SecondEchelon[e]] && visitEdge_Copy[e][k][t] == 2)
							{
								route.push_back(params.index_j_SecondEchelon[e]);
								currentNode = params.index_j_SecondEchelon[e];
								visited[params.index_j_SecondEchelon[e]] = true;
								visitEdge_Copy[e][k][t] = 1;
								break;
							}
						}
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
						solSE.customerAssignmentToWarehouse[t][w][i] = 1;
						break;
					}
				}
			}
		}
	}
}

void BC_Deterministic::CalculateCostsForEachPart()
{
	if (params.problemType == "EEV")
	{
		cout << "setupCost = " << sol_FE_EV.setupCost << endl;
		cout << "productionCost = " << sol_FE_EV.productionCost << endl;
		cout << "holdingCostPlant = " << sol_FE_EV.holdingCostPlant << endl;
		cout << "transportationCostPlantToWarehouse = " << sol_FE_EV.transportationCostPlantToWarehouse << endl;

		result.objValue_firstEchelon = sol_FE_EV.setupCost + sol_FE_EV.productionCost + sol_FE_EV.holdingCostPlant + sol_FE_EV.transportationCostPlantToWarehouse;
	}
	else
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
		}
		cout << "setupCost = " << solFE.setupCost << endl;
		cout << "productionCost = " << solFE.productionCost << endl;
		cout << "holdingCostPlant = " << solFE.holdingCostPlant << endl;
		cout << "transportationCostPlantToWarehouse = " << solFE.transportationCostPlantToWarehouse << endl;

		result.objValue_firstEchelon = solFE.setupCost + solFE.productionCost + solFE.holdingCostPlant + solFE.transportationCostPlantToWarehouse;
	}

	if (params.problemType != "2EPRPCS")
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				solSE.holdingCostWarehouse += params.unitHoldingCost_Warehouse[w] * solSE.warehouseInventory[w][t];
			}
		}
		cout << "Holding cost warehouse : " << solSE.holdingCostWarehouse << endl;
		result.objValue_secondEchelon += solSE.holdingCostWarehouse;
	}
	else
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				solSE.handlingCostSatellite += params.unitHandlingCost_Satellite[w] * solFE.deliveryQuantityToWarehouse[w][t];
			}
		}
		cout << "Handling cost satellite : " << solSE.handlingCostSatellite << endl;
		result.objValue_secondEchelon += solSE.handlingCostSatellite;
	}

	if (shortageAllowed)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				solSE.costOfUnmetDemand += params.unmetDemandPenalty[i] * solSE.customerUnmetDemand[i][t];
			}
		}
		cout << "Avg cost of unmet demand : " << solSE.costOfUnmetDemand << endl;
		result.objValue_secondEchelon += solSE.costOfUnmetDemand;
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

	cout << "Avg holding cost customer : " << solSE.holdingCostCustomer << endl;
	cout << "Avg transportation cost warehouse to customer : " << solSE.transportationCostWarehouseToCustomer << endl;

	result.objValue_secondEchelon += solSE.holdingCostCustomer + solSE.transportationCostWarehouseToCustomer;

	result.objValue_Total = result.objValue_firstEchelon + result.objValue_secondEchelon;

	cout << "objValue_FE = " << result.objValue_firstEchelon << endl;
	cout << "objValue_FE : " << result.objValue_secondEchelon << endl;
	cout << "objValue_Total : " << result.objValue_Total << endl;
}

void BC_Deterministic::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << solFE.productionSetup[t] << endl;
		}
	}
}

void BC_Deterministic::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << solFE.productionQuantity[t] << endl;
		}
	}
}

void BC_Deterministic::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << solFE.plantInventory[t] << endl;
		}
	}
}

void BC_Deterministic::DisplayWarehouseInventoryVars()
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

void BC_Deterministic::DisplayFirstEchelonRouteVars()
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

void BC_Deterministic::DisplayDeliveryQuantityToWarehousesVars()
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

void BC_Deterministic::DisplayCustomerInventoryVars()
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

void BC_Deterministic::DisplayCustomerUnmetDemandVars()
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

void BC_Deterministic::DisplayDeliveryQuantityToCustomersVars()
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

void BC_Deterministic::DisplayRoutesWarehouseToCustomersVars()
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

void BC_Deterministic::DefineWarmStartSolution(IloEnv &env, IloCplex &cplex)
{
	IloNumVarArray startVar(env);
	IloNumArray startVal(env);

	// Define warm start solution
	if (params.problemType != "EEV")
	{
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
	}

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
