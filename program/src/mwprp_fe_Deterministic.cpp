#include "MWPRP_FE_Deterministic.h"

MWPRP_FE_Deterministic::MWPRP_FE_Deterministic(const ParameterSetting &parameters, bool saveModel, bool saveSolution)
	: params(parameters),
	  saveLP(saveModel),
	  saveSol(saveSolution),
	  THRESHOLD(1e-2)
{
	routeMatrix_FirstEchelon = params.getRouteMatrix();
	numRoutes_FirstEchelon = routeMatrix_FirstEchelon.size();
	optimalRoutes_FirstEchelon = params.getOptimalRoutes();
	routeCosts_FirstEchelon = params.getRouteCosts();

	CATW = params.getCustomersAssignedToWarehouse_det();

	vector<vector<double>> actualDemandCustomers(params.numCustomers, vector<double>(params.numPeriods, 0.0));
	vector<double> remainingInitInv_Custs = params.initialInventory_Customer;

	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 0; i < params.numCustomers; ++i)
		{
			actualDemandCustomers[i][t] = params.demand_Deterministic[i][t];
			if (remainingInitInv_Custs[i] > 0.0)
			{
				actualDemandCustomers[i][t] = std::max(0.0, params.demand_Deterministic[i][t] - remainingInitInv_Custs[i]);
				remainingInitInv_Custs[i] -= std::min(params.demand_Deterministic[i][t], remainingInitInv_Custs[i]);
			}
		}
	}

	demand_warehouse.resize(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				if (CATW[t][w][i] == 1)
				{
					demand_warehouse[w][t] += actualDemandCustomers[i][t];
				}
			}
		}
	}

	if (params.problemType == "EV" || params.problemType == "WS")
		shortageAllowed = true;

	if (shortageAllowed)
	{
		approximatedUnmetDemandPenalty_warehouse.resize(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				double numAssCustomers = 0.0;
				for (int i = 0; i < params.numCustomers; ++i)
				{
					if (CATW[t][w][i] == 1)
					{
						approximatedUnmetDemandPenalty_warehouse[w][t] += params.unmetDemandPenalty[i];
						numAssCustomers += 1.0;
					}
				}
				approximatedUnmetDemandPenalty_warehouse[w][t] /= numAssCustomers;
			}
		}
	}
	else
	{
		approximatedUnmetDemandPenalty_warehouse.resize(params.numWarehouses, vector<double>(params.numPeriods, std::numeric_limits<double>::max()));
	}
}

bool MWPRP_FE_Deterministic::Solve()
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

		cout << "Solving MWPRP_FE_" << params.problemType << "..." << endl;
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
		string status = handleCplexStatus(cplex, env, model);

		auto currentTime = std::chrono::high_resolution_clock::now();
		double CPUtime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Timer (seconds): " << std::fixed << std::setprecision(4) << CPUtime << endl;

		if (status == "Optimal" || status == "Incumbent")
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

void MWPRP_FE_Deterministic::configureCplex(IloCplex &cplex, IloEnv &env)
{
	// Set CPLEX Parameters: (DISPLAY LEVEL(0,1,2,3,4), OPTIMALITY GAP, RUN TIME (SECS), THREADS, MEMORY (MB))
	CplexParameterManager parameterManager(cplex);
	parameterManager.setParameters(4, 1e-6, 600, 8, 32000);

	cplex.setParam(IloCplex::Param::Emphasis::MIP, 2);
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
	cplex.setParam(IloCplex::Param::MIP::Strategy::Search, CPX_MIPSEARCH_TRADITIONAL);
	cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);

	// Set CPLEX parameters: No output or warnings
	cplex.setOut(env.getNullStream());
	cplex.setWarning(env.getNullStream());
}

string MWPRP_FE_Deterministic::handleCplexStatus(IloCplex &cplex, IloEnv &env, IloModel &model)
{
	string status;
	double objValue;
	double optimalityGap;
	double lowerBound;
	if (cplex.getStatus() == IloAlgorithm::Optimal)
	{
		status = "Optimal";
		objValue = cplex.getObjValue();
		optimalityGap = cplex.getMIPRelativeGap() * 100;
		lowerBound = cplex.getBestObjValue();
		cout << "Optimal solution found. Objective Value = " << objValue << endl;
	}
	else if (cplex.getStatus() == IloAlgorithm::Feasible)
	{
		status = "Incumbent";
		objValue = cplex.getObjValue();
		optimalityGap = cplex.getMIPRelativeGap() * 100;
		lowerBound = cplex.getBestObjValue();
		cout << "Feasible solution found. Objective Value: " << objValue << endl;
		cout << "LB = " << lowerBound;
		cout << "Optimality Gap (%) = " << optimalityGap;
	}
	else if (cplex.getStatus() == IloAlgorithm::Infeasible)
	{
		status = "Infeasible";
		cerr << "Problem is infeasible." << endl;
		refineConflict(cplex, env, model);
		throw std::runtime_error("Solver terminated with infeasible solution.");
	}
	else
	{
		status = "Undefined";
		cerr << "Solver terminated with undefined status." << endl;
	}

	if (saveSol)
	{
		string directory = "../cplexFiles/solVal/" + params.problemType + "/";
		string fileName = generateFileName(directory, ".sol");
		cplex.writeSolution(fileName.c_str());
	}

	return status;
}

string MWPRP_FE_Deterministic::generateFileName(const string &baseDir, const string &extension)
{
	string fileName = baseDir + "MWPRP_FE_" + params.problemType + "_Ins" + params.instance;

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

void MWPRP_FE_Deterministic::refineConflict(IloCplex &cplex, IloEnv &env, IloModel &model)
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

void MWPRP_FE_Deterministic::DefineVariables(IloEnv &env, IloModel &model)
{
	// Define Decision Variables
	// Initialize Variable Manager
	VariableManager varManager(env);
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define y[t] variables (Production setup in period t)
	y = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "y[" + std::to_string(t + 1) + "]";
		y[t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
		model.add(y[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define p[t] variables (Production quantity in period t)
	p = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "p[" + std::to_string(t + 1) + "]";
		p[t] = IloNumVar(env, 0.0, params.prodCapacity, IloNumVar::Float, varName.c_str());
		model.add(p[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define I_plant[t] variables (Plant inventory in period t)
	I_plant = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "I_plant[" + std::to_string(t + 1) + "]";
		I_plant[t] = IloNumVar(env, 0.0, params.storageCapacity_Plant, IloNumVar::Float, varName.c_str());
		model.add(I_plant[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	if (params.problemType != "2EPRPCS")
	{
		// Define I_warehouse[w][t] variables (Warehouse w inventory in period t)
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
	// Define b_warehouse[w][t] variables (Warehouse w inventory in period t)
	b_warehouse = varManager.create2D(params.numWarehouses, params.numPeriods);
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "b_warehouse[" + std::to_string(w + 1) + "][" + std::to_string(t + 1) + "]";
			b_warehouse[w][t] = IloNumVar(env, 0.0, demand_warehouse[w][t], IloNumVar::Float, varName.c_str());
			model.add(b_warehouse[w][t]);
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define q[r][w][t] variables (Delivery quantity to warehouse w in period t from route r)
	q = varManager.create3D(numRoutes_FirstEchelon, params.numWarehouses, params.numPeriods);
	for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				string varName = "q[" + std::to_string(routeInd + 1) + "][" + std::to_string(w + 1) + "][" + std::to_string(t + 1) + "]";

				double upperBound = routeMatrix_FirstEchelon[routeInd][w + 1] * params.storageCapacity_Warehouse[w];

				q[routeInd][w][t] = IloNumVar(env, 0.0, upperBound, IloNumVar::Float, varName.c_str());

				model.add(q[routeInd][w][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define o[r][t] variables (binary variables to indicate whether route r is selected in period t) - from plant to warehouse
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
}

void MWPRP_FE_Deterministic::DefineObjectiveFunction(IloEnv &env, IloModel &model)
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

		if (params.problemType != "2EPRPCS")
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				obj += params.unitHoldingCost_Warehouse[w] * I_warehouse[w][t];
			}
		}
		else if (params.problemType == "2EPRPCS")
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int w = 0; w < params.numWarehouses; ++w)
				{
					for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
					{
						obj += params.unitHandlingCost_Satellite[w] * q[routeInd][w][t];
					}
				}
			}
		}

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			obj += approximatedUnmetDemandPenalty_warehouse[w][t] * b_warehouse[w][t];
		}
	}
	model.add(IloMinimize(env, obj));
}

void MWPRP_FE_Deterministic::DefineConstraints(IloEnv &env, IloModel &model)
{
	/* Define Constraints */
	DefCons_ProductionCapacity(env, model);
	DefCons_PlantInventoryCapacity(env, model);
	DefCons_PlantInventoryBalance(env, model);
	DefCons_WarehouseCoverage_FirstEchelon(env, model);
	DefCons_VehicleCapacity_FirstEchelon(env, model);
	DefCons_FleetSize_FirstEchelon(env, model);
	DefCons_WarehouseVisit_FirstEchelon(env, model);
	
	if (params.problemType == "2EPRPCS")
	{
		DefCons_SatelliteInventoryBalance(env, model);
	}
	else 
	{
		DefCons_WarehouseInventoryCapacity(env, model);
		DefCons_WarehouseInventoryBalance(env, model);
	}
}

void MWPRP_FE_Deterministic::DefCons_ProductionCapacity(IloEnv &env, IloModel &model)
{
	/*
		Define Production Capacity Constraints:
			p[t] <= Capacity * y[t]			for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "ProductionCapacity(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		expr += p[t];
		expr += -params.prodCapacity * y[t];
		IloConstraint productionCapacityConstraint(expr <= 0);
		expr.end();

		model.add(productionCapacityConstraint).setName(constraintName.c_str());
	}
}

void MWPRP_FE_Deterministic::DefCons_PlantInventoryCapacity(IloEnv &env, IloModel &model)
{
	/*
		Define Inventory Capacity Constraints (Plant)
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

void MWPRP_FE_Deterministic::DefCons_PlantInventoryBalance(IloEnv &env, IloModel &model)
{
	/*
		Define Inventory Balance Constraints (Plant):
			I_plant[t] = I_plant[t-1] + p[t] - sum(r in R) sum(w in W) q[r][w][t] 		for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "PlantInventoryBalance(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		if (t == 0)
		{
			expr += I_plant[t];
			expr += -p[t];
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
			expr += -I_plant[t - 1];
			expr += -p[t];
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

void MWPRP_FE_Deterministic::DefCons_WarehouseCoverage_FirstEchelon(IloEnv &env, IloModel &model)
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

void MWPRP_FE_Deterministic::DefCons_WarehouseVisit_FirstEchelon(IloEnv &env, IloModel &model)
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

void MWPRP_FE_Deterministic::DefCons_VehicleCapacity_FirstEchelon(IloEnv &env, IloModel &model)
{
	/*
		Define Vehicle Capacity Constraints (From Plant to Warehouse):
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
				// expr += routeMatrix_FirstEchelon[routeInd][w + 1] * q[routeInd][w][t];
				expr += q[routeInd][w][t];
			}
			expr -= params.vehicleCapacity_Plant * o[routeInd][t];
			IloConstraint vehicleCapacityConstraint(expr <= 0);
			expr.end();

			model.add(vehicleCapacityConstraint).setName(constraintName.c_str());
		}
	}
}

void MWPRP_FE_Deterministic::DefCons_FleetSize_FirstEchelon(IloEnv &env, IloModel &model)
{
	/*
		Define Fleet Size Constraints:
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

void MWPRP_FE_Deterministic::DefCons_WarehouseInventoryCapacity(IloEnv &env, IloModel &model)
{
	/*
		Define Inventory Capacity Constraints (Warehouses):
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

void MWPRP_FE_Deterministic::DefCons_WarehouseInventoryBalance(IloEnv &env, IloModel &model)
{
	/*
		Define Inventory Balance Constraints (Warehouses):
			I_warehouse[w][t] = I_warehouse[w][t-1] + q[w][t] - d[w][t] 	for all w in W, t in T
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

				expr += -b_warehouse[w][t];

				for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
				{
					expr += -q[routeInd][w][t];
				}
				IloConstraint warehouseInventoryBalanceConstraint(expr == params.initialInventory_Warehouse[w] - demand_warehouse[w][t]);
				expr.end();

				model.add(warehouseInventoryBalanceConstraint).setName(constraintName.c_str());
			}
			else
			{
				expr += I_warehouse[w][t];
				expr += -b_warehouse[w][t];
				
				expr += -I_warehouse[w][t - 1];
				for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
				{
					expr += -q[routeInd][w][t];
				}
				IloConstraint warehouseInventoryBalanceConstraint(expr == -demand_warehouse[w][t]);
				expr.end();

				model.add(warehouseInventoryBalanceConstraint).setName(constraintName.c_str());
			}
		}
	}
}

void MWPRP_FE_Deterministic::DefCons_SatelliteInventoryBalance(IloEnv &env, IloModel &model)
{
	/*
		Satellite Inventory Balance Constraints:
			sum(r in R) q[w][r][t] = demand_warehouse[w][t]	for all w in W, t in T
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
			IloConstraint warehouseInventoryBalanceConstraint(expr == demand_warehouse[w][t]);
			expr.end();

			model.add(warehouseInventoryBalanceConstraint).setName(constraintName.c_str());
		}
	}
}

void MWPRP_FE_Deterministic::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
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
	}

	if (params.problemType != "2EPRPCS"){
		warehouseInventory.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));

		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{	
				warehouseInventory[w][t] = cplex.getValue(I_warehouse[w][t]);
			}
		}
	}


	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				solFE.deliveryQuantityToWarehouse[w][t] += cplex.getValue(q[routeInd][w][t]);
			}
		}
	}
}

void MWPRP_FE_Deterministic::CalculateCostsForEachPart()
{
	double holdingCostWarehouse;
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
	double totalCost_FE = solFE.setupCost + solFE.productionCost + solFE.holdingCostPlant + solFE.transportationCostPlantToWarehouse;
	
	if (params.problemType != "2EPRPCS"){
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				holdingCostWarehouse += params.unitHoldingCost_Warehouse[w] * warehouseInventory[w][t];
			}
		}
	}
	

	cout << "Setup Cost: " << solFE.setupCost << endl;
	cout << "Production Cost: " << solFE.productionCost << endl;
	cout << "Holding Cost Plant: " << solFE.holdingCostPlant << endl;
	cout << "Transportation Cost Plant to Warehouse: " << solFE.transportationCostPlantToWarehouse << endl;
	cout << "Total Cost_FE (MW-PRP): " << totalCost_FE << endl;
}

void MWPRP_FE_Deterministic::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << solFE.productionSetup[t] << endl;
		}
	}
}

void MWPRP_FE_Deterministic::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << solFE.productionQuantity[t] << endl;
		}
	}
}

void MWPRP_FE_Deterministic::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (solFE.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << solFE.plantInventory[t] << endl;
		}
	}
}

void MWPRP_FE_Deterministic::DisplayWarehouseInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (warehouseInventory[w][t] > THRESHOLD)
			{
				cout << "I_warehouse[" << w + 1 << "][" << t + 1 << "] = " << std::setprecision(0) << std::fixed << warehouseInventory[w][t] << endl;
			}
		}
	}
}

void MWPRP_FE_Deterministic::DisplayFirstEchelonRouteVars()
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

void MWPRP_FE_Deterministic::DisplayDeliveryQuantityToWarehousesVars()
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
