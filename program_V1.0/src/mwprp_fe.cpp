#include "MWPRP_FE.h"

MWPRP_FE::MWPRP_FE(const ParameterSetting &parameters, const vector<vector<vector<double>>> &warehouseInventory_Previous, const vector<vector<vector<double>>> &dualValues_WarehouseInventoryLB)
	: params(parameters),
	  THRESHOLD(1e-2),
	  warehouseInventory_PreviousIter(warehouseInventory_Previous),
	  dualValues_WarehouseInventoryLB(dualValues_WarehouseInventoryLB),
	  save_lpFile(true),
	  save_mpsResultFile(false)
{
	routeMatrix_FirstEchelon = params.getRouteMatrix();
	numRoutes_FirstEchelon = routeMatrix_FirstEchelon.size();
	optimalRoutes_FirstEchelon = params.getOptimalRoutes();
	routeCosts_FirstEchelon = params.getRouteCosts();

	CATW = params.getCustomersAssignedToWarehouse();
	vector<vector<vector<double>>> actualDemandCustomers(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	vector<double> remainingInitInv_Custs = params.initialInventory_Customer;
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int i = 0; i < params.numCustomers; ++i)
			{
				actualDemandCustomers[i][t][s] = params.demand[i][t][s];
				if (remainingInitInv_Custs[i] > 0.0)
				{
					actualDemandCustomers[i][t][s] = std::max(0.0, params.demand[i][t][s] - remainingInitInv_Custs[i]);
					remainingInitInv_Custs[i] -= std::min(params.demand[i][t][s], remainingInitInv_Custs[i]);
				}
			}
		}
	}

	demand_warehouse.resize(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				for (int i = 0; i < params.numCustomers; ++i){
					if (CATW[s][t][w][i] == 1){
						demand_warehouse[w][t][s] += actualDemandCustomers[i][t][s];
					}
				}
			}
		}
	}

	approximatedUnmetDemandPenalty_warehouse.resize(params.numWarehouses, vector<vector<double>>(params.numScenarios, vector<double>(params.numPeriods, 0.0)));
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				double numAssCustomers = 0.0;
				for (int i = 0; i < params.numCustomers; ++i){
					if (CATW[s][t][w][i] == 1){
						approximatedUnmetDemandPenalty_warehouse[w][s][t] += params.unmetDemandPenalty[i];
						numAssCustomers += 1.0;
					}
				}
				approximatedUnmetDemandPenalty_warehouse[w][s][t] /= numAssCustomers;
			}
		}
	}
}

bool MWPRP_FE::Solve()
{
	try
	{
		IloEnv env;
		IloModel model(env);
		IloCplex cplex(model);

		auto startTime = std::chrono::high_resolution_clock::now();

		// Set CPLEX Parameters: (DISPLAY LEVEL(0,1,2,3,4), OPTIMALITY GAP, RUN TIME (SECS), THREADS, MEMORY (MB))
		CplexParameterManager parameterManager(cplex);
		parameterManager.setParameters(1, 1e-6, 600, 8, 16000);
		cplex.setParam(IloCplex::Param::Emphasis::MIP, 2);

		DefineVariables(env, model);
		DefineObjectiveFunction(env, model);
		DefineConstraints(env, model);

		/* Assure linear mappings between the presolved and original models */
		cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
		/* Turn on traditional search for use with control callbacks */
		cplex.setParam(IloCplex::Param::MIP::Strategy::Search, CPX_MIPSEARCH_TRADITIONAL);
		/* Let MIP callbacks work on the original model */
		cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);

		if (save_lpFile)
		{
			string directory = "../cplexFiles/lpModel/";
			string lpFileName = directory + "MWPRP_FE_NW" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_Ins" + params.instance.c_str() + ".lp";

			// Export the model to an LP file
			cplex.exportModel(lpFileName.c_str());
		}

		// Extract model
		cplex.extract(model);

		// Start From a Warm Start Solution if any is given
		// cplex.setParam(IloCplex::Param::MIP::Limits::RepairTries, 1e6);
		// Solution warmstart;
		// const string solFrom = "IMH";
		// params.loadWarmstart(solFrom, warmstart);
		// if (!warmstart.empty())
		// {
		// 	DefineWarmStartSolution(env, cplex, warmstart);
		// }
		// else
		// {
		// 	cout << "Couldn't Find A Solution For Warmstart" << endl;
		// }

		// Solve the model
		cplex.solve();

		if (cplex.getStatus() == IloAlgorithm::Optimal)
		{
			result.status = "Optimal";
			result.objValue = cplex.getObjValue();
			cout << "Optimal solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue << endl;
			result.optimalityGap = cplex.getMIPRelativeGap() * 100;
			result.lowerBound = cplex.getBestObjValue();

			if (save_mpsResultFile)
			{
				string directory = "../cplexFiles/solVal/";
				string solFileName = directory + "MWPRP_FE_NW" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_Ins" + params.instance.c_str();

				// Export the model to an LP file
				cplex.writeSolution(solFileName.c_str());
			}
		}
		else if (cplex.getStatus() == IloAlgorithm::Feasible)
		{
			result.status = "Incumbent";
			result.objValue = cplex.getObjValue();
			cout << "Incumbent solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue << endl;
			result.optimalityGap = cplex.getMIPRelativeGap() * 100;
			cout << "Optimality gap: " << result.optimalityGap << "%" << endl;
			result.lowerBound = cplex.getBestObjValue();
			cout << "Lower bound: " << result.lowerBound << endl;

			if (save_mpsResultFile)
			{
				string directory = "../cplexFiles/solVal/";
				string solFileName = directory + "MWPRP_FE_NW" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numCustomers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_Ins" + params.instance.c_str();

				// Export the model to an LP file
				cplex.writeSolution(solFileName.c_str());
			}
		}
		else if (cplex.getStatus() == IloAlgorithm::Infeasible)
		{
			result.status = "Infeasible";
			cout << "Problem is infeasible" << endl;

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
		result.CPUtime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Timer (seconds): " << std::fixed << std::setprecision(4) << result.CPUtime << endl;

		if (result.status == "Optimal" || result.status == "Incumbent")
		{
			// Retrieve the solution
			RetrieveSolutions(cplex);
			// Display the solution
			DisplayProductionSetupVars();
			DisplayProductionQuantVars();
			// DisplayPlantInventoryVars();
			DisplayWarehouseInventoryVars();
			DisplayFirstEchelonRouteVars();
			DisplayDeliveryQuantityToWarehousesVars();
			CalculateCostsForEachPart();
		}

		// if (!params.saveSolution(algorithm, solution))
		// {
		// 	cerr << "Unable to Save the Solution" << endl;
		// }
		// else
		// {
		// 	bool feasCheck = params.checkFeasibility(algorithm);
		// 	params.saveResultSummaryExact(solution, algorithm, result.objValue, result.lowerBound, result.optimalityGap, result.CPUtime, feasCheck);
		// }

		env.end();
	}
	catch (const IloException &e)
	{
		cerr << "Error: " << e << endl;
		return result.success = false;
	}
	catch (const std::runtime_error &e)
	{
		cerr << "Runtime Error: " << e.what() << endl;
		return result.success = false;
	}
	return result.success = true;
}

void MWPRP_FE::DefineVariables(IloEnv &env, IloModel &model)
{
	// Define Decision Variables

	// Initialize Variable Manager
	VariableManager varManager(env);
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define y[t] variables (Production setup in period t)
	y = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "y[" + std::to_string(t) + "]";
		y[t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
		model.add(y[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define p[t] variables (Production quantity in period t)
	p = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "p[" + std::to_string(t) + "]";
		p[t] = IloNumVar(env, 0.0, params.prodCapacity, IloNumVar::Float, varName.c_str());
		model.add(p[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define I_plant[t] variables (Plant inventory in period t)
	I_plant = varManager.create1D(params.numPeriods);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string varName = "I_plant[" + std::to_string(t) + "]";
		I_plant[t] = IloNumVar(env, 0.0, params.storageCapacity_Plant, IloNumVar::Float, varName.c_str());
		model.add(I_plant[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define I_warehouse[w][t][s] variables (Warehouse w inventory in period t under scenario s)
	I_warehouse = varManager.create3D(params.numWarehouses, params.numPeriods, params.numScenarios);
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int s = 0; s < params.numScenarios; ++s)
			{
				string varName = "I_warehouse[" + std::to_string(w) + "][" + std::to_string(t) + "][" + std::to_string(s) + "]";
				I_warehouse[w][t][s] = IloNumVar(env, 0.0, params.storageCapacity_Warehouse[w], IloNumVar::Float, varName.c_str());
				model.add(I_warehouse[w][t][s]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define b_warehouse[w][t][s] variables (Warehouse w inventory in period t under scenario s)
	b_warehouse = varManager.create3D(params.numWarehouses, params.numPeriods, params.numScenarios);
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int s = 0; s < params.numScenarios; ++s)
			{
				string varName = "b_warehouse[" + std::to_string(w) + "][" + std::to_string(t) + "][" + std::to_string(s) + "]";
				b_warehouse[w][t][s] = IloNumVar(env, 0.0, demand_warehouse[w][t][s], IloNumVar::Float, varName.c_str());
				model.add(b_warehouse[w][t][s]);
			}
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
				string varName = "q[" + std::to_string(routeInd) + "][" + std::to_string(w) + "][" + std::to_string(t) + "]";
				q[routeInd][w][t] = IloNumVar(env, 0.0, routeMatrix_FirstEchelon[routeInd][w + 1] * params.storageCapacity_Warehouse[w], IloNumVar::Float, varName.c_str());
				model.add(q[routeInd][w][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define o[r][t] variables (binary variables to indicate whether route r is selected in period t) - from plant to warehouse
	o = varManager.create2D(numRoutes_FirstEchelon, params.numPeriods);
	for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "o[" + std::to_string(routeInd) + "][" + std::to_string(t) + "]";
			o[routeInd][t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
			model.add(o[routeInd][t]);
		}
	}
}

void MWPRP_FE::DefineObjectiveFunction(IloEnv &env, IloModel &model)
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
				obj += params.probability[s] * approximatedUnmetDemandPenalty_warehouse[w][s][t] * params.unitHoldingCost_Warehouse[w] * b_warehouse[w][t][s];
			}
		}

		// for (int s = 0; s < params.numScenarios; ++s)
		// {
		// 	for (int t = 0; t < params.numPeriods; ++t)
		// 	{
		// 		for (int w = 0; w < params.numWarehouses; ++w)
		// 		{
		// 			if (t == 0)
		// 			{
		// 				obj += params.probability[s] * dualValues_WarehouseInventoryLB[w][t][s] * (params.initialInventory_Warehouse[w]);
		// 			}
		// 			else
		// 			{
		// 				obj += params.probability[s] * dualValues_WarehouseInventoryLB[w][t][s] * (I_warehouse[w][t - 1][s]);
		// 			}

		// 			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		// 			{
		// 				obj += params.probability[s] * dualValues_WarehouseInventoryLB[w][t][s] * q[routeInd][w][t];
		// 			}

		// 			// obj -= 10 * params.probability[s] * dualValues_WarehouseInventoryLB[w][t][s] * warehouseInventory_PreviousIter[w][t][s];
		// 		}
		// 	}
		// }
	}
	model.add(IloMinimize(env, obj));
}

void MWPRP_FE::DefineConstraints(IloEnv &env, IloModel &model)
{
	/* Define Constraints */
	// ---------------------------------------------------------------------------------------------------------------------------
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
	// ---------------------------------------------------------------------------------------------------------------------------
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
	// ---------------------------------------------------------------------------------------------------------------------------
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
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Capacity Constraints (Warehouses):
			I_warehouse[w][t][s] + demand_warehouse[w][t][s] <= storageCapacity_warehouse 		for all w in W, t in T, s in S
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
				// IloConstraint warehouseInventoryCapacityConstraint(expr <= params.storageCapacity_Warehouse[w] - demand_warehouse[w][t][s]);
				IloConstraint warehouseInventoryCapacityConstraint(expr <= params.storageCapacity_Warehouse[w]);
				expr.end();

				model.add(warehouseInventoryCapacityConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Balance Constraints (Warehouses):
			I_warehouse[w][t][s] = I_warehouse[w][t-1][s] + q[w][t] - d[w][t][s] 	for all w in W, t in T, s in S
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
					expr += -b_warehouse[w][t][s];
					for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
					{
						expr += -q[routeInd][w][t];
					}
					IloConstraint warehouseInventoryBalanceConstraint(expr == params.initialInventory_Warehouse[w] - demand_warehouse[w][t][s]);
					expr.end();

					model.add(warehouseInventoryBalanceConstraint).setName(constraintName.c_str());
				}
				else
				{
					expr += I_warehouse[w][t][s];
					expr += -b_warehouse[w][t][s];
					expr += -I_warehouse[w][t - 1][s];
					for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
					{
						expr += -q[routeInd][w][t];
					}
					IloConstraint warehouseInventoryBalanceConstraint(expr == -demand_warehouse[w][t][s]);
					expr.end();

					model.add(warehouseInventoryBalanceConstraint).setName(constraintName.c_str());
				}
			}
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
	// ---------------------------------------------------------------------------------------------------------------------------
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

void MWPRP_FE::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	sol.productionSetup.assign(params.numPeriods, 0);
	sol.productionQuantity.assign(params.numPeriods, 0.0);
	sol.plantInventory.assign(params.numPeriods, 0.0);
	sol.warehouseInventory.assign(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	sol.customerInventory.assign(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	sol.deliveryQuantityToWarehouse.assign(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	sol.routesPlantToWarehouse.assign(params.numPeriods, vector<vector<int>>(params.numVehicles_Plant, vector<int>()));

	selectedRoute.assign(numRoutes_FirstEchelon, vector<int>(params.numPeriods, 0));

	// Get solution values of decision variables
	for (int t = 0; t < params.numPeriods; ++t)
	{
		sol.productionSetup[t] = cplex.getIntValue(y[t]);
		sol.productionQuantity[t] = cplex.getValue(p[t]);
		sol.plantInventory[t] = cplex.getValue(I_plant[t]);

		int vehInd = 0;
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			selectedRoute[routeInd][t] = cplex.getIntValue(o[routeInd][t]);
			if (selectedRoute[routeInd][t] == 1)
			{
				for (int node: optimalRoutes_FirstEchelon[routeInd])
				{
					sol.routesPlantToWarehouse[t][vehInd].push_back(node);
				}
				vehInd++;
				// sol.routesPlantToWarehouse[t][routeInd] = optimalRoutes_FirstEchelon[routeInd];
			}
		}

		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				sol.deliveryQuantityToWarehouse[w][t] += cplex.getValue(q[routeInd][w][t]);
			}

			for (int s = 0; s < params.numScenarios; ++s)
			{
				sol.warehouseInventory[w][t][s] = cplex.getValue(I_warehouse[w][t][s]);
			}
		}
	}
}

void MWPRP_FE::CalculateCostsForEachPart()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		sol.setupCost += params.setupCost * sol.productionSetup[t];
		sol.productionCost += params.unitProdCost * sol.productionQuantity[t];
		sol.holdingCostPlant += params.unitHoldingCost_Plant * sol.plantInventory[t];

		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			if (selectedRoute[routeInd][t] == 1)
			{
				sol.transportationCostPlantToWarehouse += routeCosts_FirstEchelon[routeInd];
			}
		}

		for (int s = 0; s < params.numScenarios; ++s)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				sol.holdingCostWarehouse_Avg += params.probability[s] * params.unitHoldingCost_Warehouse[w] * sol.warehouseInventory[w][t][s];
			}
		}
	}
	double totalCost = sol.setupCost + sol.productionCost + sol.holdingCostPlant + sol.transportationCostPlantToWarehouse + sol.holdingCostWarehouse_Avg;

	cout << "Setup Cost: " << sol.setupCost << endl;
	cout << "Production Cost: " << sol.productionCost << endl;
	cout << "Holding Cost Plant: " << sol.holdingCostPlant << endl;
	cout << "Transportation Cost Plant to Warehouse: " << sol.transportationCostPlantToWarehouse << endl;
	cout << "Total Cost: " << totalCost << endl;
}

void MWPRP_FE::DisplayProductionSetupVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol.productionSetup[t] == 1)
		{
			cout << "y[" << t + 1 << "] = " << sol.productionSetup[t] << endl;
		}
	}
}

void MWPRP_FE::DisplayProductionQuantVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol.productionQuantity[t] > THRESHOLD)
		{
			cout << "p[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol.productionQuantity[t] << endl;
		}
	}
}

void MWPRP_FE::DisplayPlantInventoryVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		if (sol.plantInventory[t] > THRESHOLD)
		{
			cout << "I_plant[" << t + 1 << "] = " << std::setprecision(0) << std::fixed << sol.plantInventory[t] << endl;
		}
	}
}

void MWPRP_FE::DisplayWarehouseInventoryVars()
{
	for (int s = 0; s < params.numScenarios; ++s)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				if (sol.warehouseInventory[w][t][s] > THRESHOLD)
				{
					cout << "I_warehouse[" << w + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << std::setprecision(0) << std::fixed << sol.warehouseInventory[w][t][s] << endl;
				}
			}
		}
	}
}

void MWPRP_FE::DisplayFirstEchelonRouteVars()
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
		for (const auto &route : sol.routesPlantToWarehouse[t])
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

void MWPRP_FE::DisplayDeliveryQuantityToWarehousesVars()
{
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			if (sol.deliveryQuantityToWarehouse[w][t] > THRESHOLD)
			{
				cout << "q[" << w + 1 << "][" << t + 1 << "] = " << sol.deliveryQuantityToWarehouse[w][t] << endl;
			}
		}
	}
}

// void PRPSL_BC::DefineWarmStartSolution(IloEnv &env, IloCplex &cplex, const Solution &warmStart)
// {
// 	IloNumVarArray startVar(env);
// 	IloNumArray startVal(env);
// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		startVar.add(y[t]);
// 		startVal.add(warmStart.productionSetup[t]);
// 	}

// 	for (int s = 0; s < params.numScenarios; ++s)
// 	{
// 		for (int t = 0; t < params.numPeriods; ++t)
// 		{
// 			startVar.add(p[t][s]);
// 			startVal.add(warmStart.productionQuantity[t][s]);
// 		}
// 	}

// 	for (int s = 0; s < params.numScenarios; ++s)
// 	{
// 		for (int t = 0; t < params.numPeriods; ++t)
// 		{
// 			for (int nodeIndex = 0; nodeIndex < params.numNodes; ++nodeIndex)
// 			{
// 				startVar.add(I[nodeIndex][t][s]);
// 				startVal.add(warmStart.inventory[nodeIndex][t][s]);
// 			}
// 		}
// 	}

// 	for (int s = 0; s < params.numScenarios; ++s)
// 	{
// 		for (int t = 0; t < params.numPeriods; ++t)
// 		{
// 			for (int nodeIndex = 1; nodeIndex < params.numNodes; ++nodeIndex)
// 			{
// 				startVar.add(b[nodeIndex][t][s]);
// 				startVal.add(warmStart.backlog[nodeIndex][t][s]);
// 			}
// 		}
// 	}

// 	for (int s = 0; s < params.numScenarios; ++s)
// 	{
// 		for (int t = 0; t < params.numPeriods; ++t)
// 		{
// 			for (int k = 0; k < params.numVehicles; k++)
// 			{
// 				for (int nodeIndex = 0; nodeIndex < params.numNodes; ++nodeIndex)
// 				{
// 					startVar.add(z[nodeIndex][k][t][s]);
// 					startVal.add(warmStart.nodeVisit[nodeIndex][k][t][s]);
// 				}
// 			}
// 		}
// 	}

// 	for (int s = 0; s < params.numScenarios; ++s)
// 	{
// 		for (int t = 0; t < params.numPeriods; ++t)
// 		{
// 			for (int k = 0; k < params.numVehicles; k++)
// 			{
// 				for (int nodeIndex = 0; nodeIndex < params.numNodes; ++nodeIndex)
// 				{
// 					startVar.add(q[nodeIndex][k][t][s]);
// 					startVal.add(warmStart.delivery[nodeIndex][k][t][s]);
// 				}
// 			}
// 		}
// 	}

// 	for (int s = 0; s < params.numScenarios; ++s)
// 	{
// 		for (int t = 0; t < params.numPeriods; ++t)
// 		{
// 			for (int k = 0; k < params.numVehicles; k++)
// 			{
// 				for (int edgeIndex = 0; edgeIndex < params.numEdges; edgeIndex++)
// 				{
// 					startVar.add(x[edgeIndex][k][t][s]);
// 					startVal.add(warmStart.edgeVisit[edgeIndex][k][t][s]);
// 				}
// 			}
// 		}
// 	}

// 	if (params.SLtype == "alpha")
// 	{
// 		for (int s = 0; s < params.numScenarios; ++s)
// 		{
// 			for (int t = 0; t < params.numPeriods; ++t)
// 			{
// 				for (int nodeIndex = 1; nodeIndex < params.numNodes; ++nodeIndex)
// 				{
// 					startVar.add(o[nodeIndex][t][s]);
// 					startVal.add(warmStart.alphaAux[nodeIndex][t][s]);
// 				}
// 			}
// 		}
// 	}
// 	else if (params.SLtype == "beta")
// 	{
// 		for (int s = 0; s < params.numScenarios; ++s)
// 		{
// 			for (int t = 0; t < params.numPeriods; ++t)
// 			{
// 				for (int nodeIndex = 1; nodeIndex < params.numNodes; ++nodeIndex)
// 				{
// 					startVar.add(bo[nodeIndex][t][s]);
// 					startVal.add(warmStart.backorder[nodeIndex][t][s]);
// 				}
// 			}
// 		}
// 	}
// 	cplex.addMIPStart(startVar, startVal, IloCplex::MIPStartEffort::MIPStartRepair);
// 	startVal.end();
// 	startVar.end();
