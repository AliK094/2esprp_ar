#include "MWPRP.h"

MWPRP::MWPRP(const ParameterSetting &parameters)
	: params(parameters),
	  THRESHOLD(1e-2),
	  save_lpFile(true),
	  save_mpsResultFile(false)
{
	solutionAlgorithm = params.solutionAlgorithm;
	instance = params.instance;
	numWarehouses = params.numWarehouses;
	numRetailers = params.numRetailers;
	numPeriods = params.numPeriods;
	numVehiclesPlant = params.numVehiclesPlant;
	numVehiclesWarehouse = params.numVehiclesWarehouse;
	numScenarios = params.numScenarios;
	prodCapacity = params.prodCapacity;

	unitHoldingCostPlant = params.unitHoldingCost[0];
	unitHoldingCostWarehouse.resize(numWarehouses);
	std::copy(params.unitHoldingCost.begin() + 1, params.unitHoldingCost.begin() + 1 + numWarehouses, unitHoldingCostWarehouse.begin());
	unitHoldingCostRetailer.resize(numRetailers);
	std::copy(params.unitHoldingCost.begin() + 1 + numWarehouses, params.unitHoldingCost.begin() + 1 + numWarehouses + numRetailers, unitHoldingCostRetailer.begin());

	storageCapacityPlant = params.storageCapacity[0];
	storageCapacityWarehouse.resize(numWarehouses);
	std::copy(params.storageCapacity.begin() + 1, params.storageCapacity.begin() + 1 + numWarehouses, storageCapacityWarehouse.begin());
	storageCapacityRetailer.resize(numRetailers);
	std::copy(params.storageCapacity.begin() + 1 + numWarehouses, params.storageCapacity.begin() + 1 + numWarehouses + numRetailers, storageCapacityRetailer.begin());

	initialInventoryPlant = params.initialInventory[0];
	initialInventoryWarehouse.resize(numWarehouses);
	std::copy(params.initialInventory.begin() + 1, params.initialInventory.begin() + 1 + numWarehouses, initialInventoryWarehouse.begin());
	initialInventoryRetailer.resize(numRetailers);
	std::copy(params.initialInventory.begin() + 1 + numWarehouses, params.initialInventory.begin() + 1 + numWarehouses + numRetailers, initialInventoryRetailer.begin());

	demand.resize(numRetailers);
	std::copy(params.demand.begin() + 1 + numWarehouses, params.demand.begin() + 1 + numWarehouses + numRetailers, demand.begin());

	unmetDemandPenalty.resize(numRetailers);
	std::copy(params.unmetDemandPenalty.begin() + 1 + numWarehouses, params.unmetDemandPenalty.begin() + 1 + numWarehouses + numRetailers, unmetDemandPenalty.begin());

	vehicleCapacityPlant = params.vehicleCapacityPlant;
	vehicleCapacityWarehouse = params.vehicleCapacityWarehouse;
	probability = params.probability;
	setupCost = params.setupCost;
	unitProdCost = params.unitProdCost;

	routeMatrix_FirstEchelon = params.getRouteMatrix();
	numRoutes_FirstEchelon = routeMatrix_FirstEchelon.size();
	optimalRoutes_FirstEchelon = params.getOptimalRoutes();
	routeCosts_FirstEchelon = params.getRouteCosts();

	auto retailersAssignedToWarehouse = params.getRetailersAssignedToWarehouse();

	routeMatrix_SecondEchelon.resize(numRetailers, std::vector<std::vector<std::vector<int>>>(numWarehouses, std::vector<std::vector<int>>(numPeriods, std::vector<int>(numScenarios, 0))));
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int w = 0; w < numWarehouses; ++w)
			{
				for (int retailer : retailersAssignedToWarehouse[w])
				{
					routeMatrix_SecondEchelon[retailer][w][t][s] = 1;
				}
			}
		}
	}

	// Printing the matrix
	// for (int rI = 0; rI < numRetailers; ++rI)
	// {
	// 	for (int w = 0; w < numWarehouses; ++w)
	// 	{
	// 		for (int k = 0; k < numVehiclesPlant; ++k)
	// 		{
	// 			for (int t = 0; t < numPeriods; ++t)
	// 			{
	// 				for (int s = 0; s < numScenarios; ++s)
	// 				{
	// 					std::cout << "routeMatrix_SecondEchelon[" << rI << "][" << w << "][" << k << "][" << t << "][" << s << "] = "
	// 							  << routeMatrix_SecondEchelon[rI][w][k][t][s] << std::endl;
	// 				}
	// 			}
	// 		}
	// 	}
	// }

	// exit(0);
}

bool MWPRP::Solve()
{
	try
	{
		IloEnv env;
		IloModel model(env);
		IloCplex cplex(model);

		auto startTime = std::chrono::high_resolution_clock::now();

		// Set CPLEX Parameters: (DISPLAY LEVEL(0,1,2,3,4), OPTIMALITY GAP, RUN TIME (SECS), THREADS, MEMORY (MB))
		CplexParameterManager parameterManager(cplex);
		parameterManager.setParameters(4, 1e-6, 7200, 10, 32000);
		cplex.setParam(IloCplex::Param::Emphasis::MIP, 2);

		DefineVariables(env, model);
		DefineObjectiveFunction(env, model);
		DefineConstraints(env, model);
		std::cout << "Define constraints" << std::endl;

		/* Assure linear mappings between the presolved and original models */
		cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
		/* Turn on traditional search for use with control callbacks */
		cplex.setParam(IloCplex::Param::MIP::Strategy::Search, CPX_MIPSEARCH_TRADITIONAL);
		/* Let MIP callbacks work on the original model */
		cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);

		if (save_lpFile)
		{
			std::string directory = "../cplexFiles/lpModel/";
			std::string lpFileName = directory + "MWPRP_FE_NW" + std::to_string(numWarehouses) + "_NR" + std::to_string(numRetailers) + "_KP" + std::to_string(numVehiclesPlant) + "_KW" + std::to_string(numVehiclesWarehouse) + "_T" + std::to_string(numPeriods) + "_S" + std::to_string(numScenarios) + "_Ins" + instance.c_str() + ".lp";

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
		// 	std::cout << "Couldn't Find A Solution For Warmstart" << endl;
		// }

		// Solve the model
		cplex.solve();

		if (cplex.getStatus() == IloAlgorithm::Optimal)
		{
			result.status = "Optimal";
			result.objValue = cplex.getObjValue();
			std::cout << "Optimal solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue << std::endl;
			result.optimalityGap = cplex.getMIPRelativeGap() * 100;
			result.lowerBound = cplex.getBestObjValue();

			if (save_mpsResultFile)
			{
				std::string directory = "../cplexFiles/solVal/";
				std::string solFileName = directory + "MWPRP_FE_NW" + std::to_string(numWarehouses) + "_NR" + std::to_string(numRetailers) + "_KP" + std::to_string(numVehiclesPlant) + "_KW" + std::to_string(numVehiclesWarehouse) + "_T" + std::to_string(numPeriods) + "_S" + std::to_string(numScenarios) + "_Ins" + instance.c_str();

				// Export the model to an LP file
				cplex.writeSolution(solFileName.c_str());
			}
		}
		else if (cplex.getStatus() == IloAlgorithm::Feasible)
		{
			result.status = "Incumbent";
			result.objValue = cplex.getObjValue();
			std::cout << "Incumbent solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue << std::endl;
			result.optimalityGap = cplex.getMIPRelativeGap() * 100;
			std::cout << "Optimality gap: " << result.optimalityGap << "%" << std::endl;
			result.lowerBound = cplex.getBestObjValue();
			std::cout << "Lower bound: " << result.lowerBound << std::endl;

			if (save_mpsResultFile)
			{
				std::string directory = "../cplexFiles/solVal/";
				std::string solFileName = directory + "MWPRP_FE_NW" + std::to_string(numWarehouses) + "_NR" + std::to_string(numRetailers) + "_KP" + std::to_string(numVehiclesPlant) + "_KW" + std::to_string(numVehiclesWarehouse) + "_T" + std::to_string(numPeriods) + "_S" + std::to_string(numScenarios) + "_Ins" + instance.c_str();

				// Export the model to an LP file
				cplex.writeSolution(solFileName.c_str());
			}
		}
		else if (cplex.getStatus() == IloAlgorithm::Infeasible)
		{
			result.status = "Infeasible";
			std::cout << "Problem is infeasible" << std::endl;

			IloConstraintArray conflictConstraints(env);
			cplex.getConflict(conflictConstraints);

			IloCplex::ConflictStatusArray conflictStatus(env);
			for (int i = 0; i < conflictConstraints.getSize(); i++)
			{
				conflictStatus.add(cplex.getConflict(conflictConstraints[i]));
			}

			std::cout << "Conflict constraints:" << std::endl;
			std::cout << conflictConstraints << std::endl;

			std::cout << "Conflict status:" << std::endl;
			std::cout << conflictStatus << std::endl;

			throw std::runtime_error("Solver terminated with infeasible solution.");
		}
		else
		{
			result.status = "Undefined";
			std::cout << "Solver terminated with status: " << result.status << std::endl;
		}

		auto currentTime = std::chrono::high_resolution_clock::now();
		result.CPUtime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		std::cout << "Timer (seconds): " << std::fixed << std::setprecision(4) << result.CPUtime << std::endl;

		RetrieveSolutions(cplex);
		// Display the solution
		DisplayProductionSetupVars();
		DisplayProductionQuantVars();

		// if (!params.saveSolution(algorithm, solution))
		// {
		// 	std::cerr << "Unable to Save the Solution" << endl;
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
		std::cerr << "Error: " << e << std::endl;
		return result.success = false;
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Runtime Error: " << e.what() << std::endl;
		return result.success = false;
	}

	return result.success = true;
}

void MWPRP::DefineVariables(IloEnv &env, IloModel &model)
{
	// Define Decision Variables

	// Initialize Variable Manager
	VariableManager varManager(env);
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define y[t] variables (Production setup in period t)
	y = varManager.create1D(numPeriods);
	for (int t = 0; t < numPeriods; ++t)
	{
		std::string varName = "y[" + std::to_string(t) + "]";
		y[t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
		model.add(y[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define p[t] variables (Production quantity in period t)
	p = varManager.create1D(numPeriods);
	for (int t = 0; t < numPeriods; ++t)
	{
		std::string varName = "p[" + std::to_string(t) + "]";
		p[t] = IloNumVar(env, 0.0, prodCapacity, IloNumVar::Float, varName.c_str());
		model.add(p[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define I_plant[t] variables (Plant inventory in period t)
	I_plant = varManager.create1D(numPeriods);
	for (int t = 0; t < numPeriods; ++t)
	{
		std::string varName = "I_plant[" + std::to_string(t) + "]";
		I_plant[t] = IloNumVar(env, 0.0, storageCapacityPlant, IloNumVar::Float, varName.c_str());
		model.add(I_plant[t]);
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define I_warehouse[w][t][s] variables (Warehouse w inventory in period t under scenario s)
	I_warehouse = varManager.create3D(numWarehouses, numPeriods, numScenarios);
	for (int w = 0; w < numWarehouses; ++w)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int s = 0; s < numScenarios; ++s)
			{
				std::string varName = "I_warehouse[" + std::to_string(w) + "][" + std::to_string(t) + "][" + std::to_string(s) + "]";
				I_warehouse[w][t][s] = IloNumVar(env, 0.0, storageCapacityWarehouse[w], IloNumVar::Float, varName.c_str());
				model.add(I_warehouse[w][t][s]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define I_retailer[rI][t][s] variables (Retailer i inventory in period t under scenario s)
	I_retailer = varManager.create3D(numRetailers, numPeriods, numScenarios);
	for (int rI = 0; rI < numRetailers; ++rI)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int s = 0; s < numScenarios; ++s)
			{
				std::string varName = "I_retailer[" + std::to_string(rI) + "][" + std::to_string(t) + "][" + std::to_string(s) + "]";
				I_retailer[rI][t][s] = IloNumVar(env, 0.0, storageCapacityRetailer[rI], IloNumVar::Float, varName.c_str());
				model.add(I_retailer[rI][t][s]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define q[w][t] variables (Delivery quantity to warehouse w in period t)
	q = varManager.create3D(numRoutes_FirstEchelon, numWarehouses, numPeriods);
	for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
	{
		for (int w = 0; w < numWarehouses; ++w)
		{
			for (int t = 0; t < numPeriods; ++t)
			{
				std::string varName = "q[" + std::to_string(routeInd) + "][" + std::to_string(w) + "][" + std::to_string(t) + "]";
				q[routeInd][w][t] = IloNumVar(env, 0.0, storageCapacityWarehouse[w], IloNumVar::Float, varName.c_str());
				model.add(q[routeInd][w][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define e[rI][w][t]^[s] variables (Delivery quantity from warehouse w to retailer i in period t under scenario s)
	e = varManager.create4D(numRetailers, numWarehouses, numPeriods, numScenarios);
	for (int rI = 0; rI < numRetailers; ++rI)
	{
		for (int w = 0; w < numWarehouses; ++w)
		{
			for (int t = 0; t < numPeriods; ++t)
			{
				for (int s = 0; s < numScenarios; ++s)
				{
					std::string varName = "e[" + std::to_string(rI) + "][" + std::to_string(w) + "][" + std::to_string(t) + "][" + std::to_string(s) + "]";
					e[rI][w][t][s] = IloNumVar(env, 0.0, storageCapacityRetailer[rI], IloNumVar::Float, varName.c_str());
					model.add(e[rI][w][t][s]);
				}
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define o[r][t] variables (binary variables to indicate whether route r is selected in period t) - from plant to warehouse
	o = varManager.create2D(numRoutes_FirstEchelon, numPeriods);
	for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			std::string varName = "o[" + std::to_string(routeInd) + "][" + std::to_string(t) + "]";
			o[routeInd][t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Int, varName.c_str());
			model.add(o[routeInd][t]);
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define b_retailer[rI][t][s] variables (Amount of Unmet demand in retailer i in period t under scenario s)
	b_retailer = varManager.create3D(numRetailers, numPeriods, numScenarios);
	for (int rI = 0; rI < numRetailers; ++rI)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int s = 0; s < numScenarios; ++s)
			{
				std::string varName = "b_retailer[" + std::to_string(rI) + "][" + std::to_string(t) + "][" + std::to_string(s) + "]";
				b_retailer[rI][t][s] = IloNumVar(env, 0.0, demand[rI][t][s], IloNumVar::Float, varName.c_str());
				model.add(b_retailer[rI][t][s]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define z[r][w][t]^[s] variables (binary variables to indicate whether route r is selected for warehouse w in period t under scenario s) - from warehouse to retailer
	// z = varManager.create3D(numRoutes_FirstEchelon_Warehouse, numWarehouses, numPeriods, numScenarios);
	// for (int routerIndex = 0; routerIndex < numRetailers; ++rI)
	// {
	// 	for (int w = 0; w < numWarehouses; ++w)
	// 	{
	// 		for (int t = 0; t < numPeriods; ++t)
	// 		{
	// 			for (int s = 0; s < numScenarios; ++s)
	// 			{
	// 				std::string varName = "z[" + std::to_string(rI) + "][" + std::to_string(w) + "][" + std::to_string(t) + "][" + std::to_string(s) + "]";
	// 				z[rI][w][t][s] = IloNumVar(env, 0.0, 1.0, IloNumVar::Int, varName.c_str());
	// 				model.add(z[rI][w][t][s]);
	// 			}
	// 		}
	// 	}
	// }
}

void MWPRP::DefineObjectiveFunction(IloEnv &env, IloModel &model)
{
	// Define objective function
	IloExpr obj(env);
	for (int t = 0; t < numPeriods; ++t)
	{
		obj += setupCost * y[t];
		obj += unitProdCost * p[t];
		obj += unitHoldingCostPlant * I_plant[t];
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			obj += routeCosts_FirstEchelon[routeInd] * o[routeInd][t];
		}

		for (int s = 0; s < numScenarios; ++s)
		{
			for (int w = 0; w < numWarehouses; ++w)
			{
				obj += probability[s] * unitHoldingCostWarehouse[w] * I_warehouse[w][t][s];
			}

			for (int rI = 0; rI < numRetailers; ++rI)
			{
				obj += probability[s] * unitHoldingCostRetailer[rI] * I_retailer[rI][t][s];
			}
		}
	}
	model.add(IloMinimize(env, obj));
}

void MWPRP::DefineConstraints(IloEnv &env, IloModel &model)
{
	/* Define Constraints */
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Production Capacity Constraints:
			p[t] <= Capacity * y[t]			for all t in T
	*/
	for (int t = 0; t < numPeriods; ++t)
	{
		std::string constraintName = "ProductionCapacity(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		expr += p[t];
		expr += -prodCapacity * y[t];
		IloConstraint productionCapacityConstraint(expr <= 0);
		expr.end();

		model.add(productionCapacityConstraint).setName(constraintName.c_str());
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Capacity Constraints (Plant)
			I_plant[t] <= storageCapacityPlant 		for all t in T
	*/
	for (int t = 0; t < numPeriods; ++t)
	{
		std::string constraintName = "PlantInventoryCapacity(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		expr += I_plant[t];
		IloConstraint plantInventoryCapacityConstraint(expr <= storageCapacityPlant);
		expr.end();

		model.add(plantInventoryCapacityConstraint).setName(constraintName.c_str());
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Balance Constraints (Plant):
			I_plant[t] = I_plant[t-1] + p[t] - sum(w in W) q[w][t] 		for all t in T
	*/
	for (int t = 0; t < numPeriods; ++t)
	{
		std::string constraintName = "PlantInventoryBalance(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		if (t == 0)
		{
			expr += I_plant[t];
			expr += -p[t];
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				for (int w = 0; w < numWarehouses; ++w)
				{
					expr += q[routeInd][w][t];
				}
			}
			IloConstraint plantInventoryBalanceConstraint(expr == initialInventoryPlant);
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
				for (int w = 0; w < numWarehouses; ++w)
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
			I_warehouse[w][t][s] + sum(i in N) e[i][w][t][s] <= storageCapacity_warehouse 		for all w in W, t in T, s in S
	*/
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int w = 0; w < numWarehouses; ++w)
			{
				std::string constraintName = "WarehouseInventoryCapacity(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				IloExpr expr(env);
				expr += I_warehouse[w][t][s];
				for (int rI = 0; rI < numRetailers; ++rI)
				{
					expr += e[rI][w][t][s];
				}
				IloConstraint warehouseInventoryCapacityConstraint(expr <= storageCapacityWarehouse[w]);
				expr.end();

				model.add(warehouseInventoryCapacityConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Balance Constraints (Warehouses):
			I_warehouse[w][t][s] = I_warehouse[w][t-1][s] + q[w][t] - sum(i in N) e[i][w][t][s] 	for all w in W, t in T, s in S
	*/
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int w = 0; w < numWarehouses; ++w)
			{
				std::string constraintName = "WarehouseInventoryBalance(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				IloExpr expr(env);
				if (t == 0)
				{
					expr += I_warehouse[w][t][s];
					for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
					{
						expr += -q[routeInd][w][t];
					}
					for (int rI = 0; rI < numRetailers; ++rI)
					{
						expr += e[rI][w][t][s];
					}
					IloConstraint warehouseInventoryBalanceConstraint(expr == initialInventoryWarehouse[w]);
					expr.end();

					model.add(warehouseInventoryBalanceConstraint).setName(constraintName.c_str());
				}
				else
				{
					expr += I_warehouse[w][t][s];
					expr += -I_warehouse[w][t - 1][s];
					for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
					{
						expr += -q[routeInd][w][t];
					}
					for (int rI = 0; rI < numRetailers; ++rI)
					{
						expr += e[rI][w][t][s];
					}
					IloConstraint warehouseInventoryBalanceConstraint(expr == 0);
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
	*/
	for (int t = 0; t < numPeriods; ++t)
	{
		for (int w = 0; w < numWarehouses; ++w)
		{
			std::string constraintName = "WarehouseCoverage(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				expr += routeMatrix_FirstEchelon[routeInd][w] * o[routeInd][t];
			}
			IloConstraint warehouseCoverageConstraint(expr <= 1);
			expr.end();

			model.add(warehouseCoverageConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Vehicle Capacity Constraints (From Plant to Warehouse):
			sum(w in W) a[r][w] * q[w][t] <= vehicleCapacityPlant * o[r][t] 		for all r in R, t in T
	*/
	for (int t = 0; t < numPeriods; ++t)
	{
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			std::string constraintName = "VehicleCapacity(" + std::to_string(routeInd + 1) + "," + std::to_string(t + 1) + ")";

			IloExpr expr(env);
			for (int w = 0; w < numWarehouses; ++w)
			{
				expr += routeMatrix_FirstEchelon[routeInd][w] * q[routeInd][w][t];
			}
			expr += -vehicleCapacityPlant * o[routeInd][t];
			IloConstraint vehicleCapacityConstraint(expr <= 0);
			expr.end();

			model.add(vehicleCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Fleet Size Constraints:
			sum (r in R) o[r][t] <= numVehiclesPlant 		for all t in T
	*/
	for (int t = 0; t < numPeriods; ++t)
	{
		std::string constraintName = "FleetSize(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			expr += o[routeInd][t];
		}
		IloConstraint fleetSizeConstraint(expr <= numVehiclesPlant);
		expr.end();

		model.add(fleetSizeConstraint).setName(constraintName.c_str());
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Capacity Constraints (Retailers):
			I_retailer[i][t][s] + d[i][t][s] <= storageCapacity_retailer 		for all i in N, t in T, s in S
	*/
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int rI = 0; rI < numRetailers; ++rI)
			{
				std::string constraintName = "RetailerInventoryCapacity(" + std::to_string(rI + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";
				IloExpr expr(env);
				expr += I_retailer[rI][t][s];
				IloConstraint retailerInventoryCapacityConstraint(expr <= storageCapacityRetailer[rI] - demand[rI][t][s]);
				expr.end();

				model.add(retailerInventoryCapacityConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Balance Constraints (Retailers):
			I_retailer[i][t][s] = I_retailer[i][t-1][s] + sum(w in W) e[i][w][t][s] - d[i][t][s] + b[i][t][s] 		for all i in N, t in T, s in S
	*/
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int rI = 0; rI < numRetailers; ++rI)
			{
				std::string constraintName = "RetailerInventoryBalance(" + std::to_string(rI + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				if (t == 0)
				{
					IloExpr expr(env);
					expr += I_retailer[rI][t][s];
					expr += -b_retailer[rI][t][s];
					for (int w = 0; w < numWarehouses; ++w)
					{
						expr += -e[rI][w][t][s];
					}
					IloConstraint retailerInventoryBalanceConstraint(expr == initialInventoryRetailer[rI] - demand[rI][t][s]);
					expr.end();

					model.add(retailerInventoryBalanceConstraint).setName(constraintName.c_str());
				}
				else
				{
					IloExpr expr(env);
					expr += I_retailer[rI][t][s];
					expr += -I_retailer[rI][t - 1][s];
					expr += -b_retailer[rI][t][s];
					for (int w = 0; w < numWarehouses; ++w)
					{
						expr += -e[rI][w][t][s];
					}
					IloConstraint retailerInventoryBalanceConstraint(expr == -demand[rI][t][s]);
					expr.end();

					model.add(retailerInventoryBalanceConstraint).setName(constraintName.c_str());
				}
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Retailer Visit by Vechile from Warehouse:
			e[i][w][t][s] <= storageCapacityRetailer  		for all i in N, w in W, t in T, s in S
	*/

	// not for now

	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Vehicle Capacity Constraints (From Warehouse to Retailer):
			sum (i in N) a[i][w][k][t][s] * e[i][w][t][s] <= vechileCapacityWarehouse		for all w in W, k in K_warehouse, t in T, s in S
	*/
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int w = 0; w < numWarehouses; ++w)
			{
				std::string constraintName = "WarehouseVehicleCapacity(" + std::to_string(w + 1) + "," + std::to_string(t + 1) + "," + std::to_string(s + 1) + ")";

				IloExpr expr(env);
				for (int rI = 0; rI < numRetailers; ++rI)
				{
					expr += routeMatrix_SecondEchelon[rI][w][t][s] * e[rI][w][t][s];
				}
				IloConstraint retailerVehicleCapacityConstraint(expr <= vehicleCapacityWarehouse);
				expr.end();

				model.add(retailerVehicleCapacityConstraint).setName(constraintName.c_str());
			}
		}
	}
}

void MWPRP::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	sol.productionSetup.assign(numPeriods, 0);
	sol.productionQuantity.assign(numPeriods, 0.0);
	sol.plantInventory.assign(numPeriods, 0.0);
	sol.warehouseInventory.assign(numWarehouses, std::vector<std::vector<double>>(numPeriods, std::vector<double>(numScenarios, 0.0)));
	sol.retailerInventory.assign(numRetailers, std::vector<std::vector<double>>(numPeriods, std::vector<double>(numScenarios, 0.0)));
	sol.retailerUnmetDemand.assign(numRetailers, std::vector<std::vector<double>>(numPeriods, std::vector<double>(numScenarios, 0.0)));
	sol.deliveryQuantityToWarehouse.assign(numRoutes_FirstEchelon, std::vector<std::vector<double>>(numWarehouses, std::vector<double>(numPeriods, 0.0)));
	sol.routePlantToWarehouse.resize(numPeriods);
	sol.deliveryQuantityToRetailer.assign(numRetailers,
										  std::vector<std::vector<std::vector<double>>>(numWarehouses,
																						std::vector<std::vector<double>>(numPeriods,
																														 std::vector<double>(numScenarios, 0.0))));
	sol.routeWarehouseToRetailer.assign(numRetailers,
										std::vector<std::vector<std::vector<std::vector<int>>>>(numWarehouses,
																								std::vector<std::vector<std::vector<int>>>(numVehiclesWarehouse,
																																		   std::vector<std::vector<int>>(numPeriods,
																																										 std::vector<int>(numScenarios, 0.0)))));

	selectedRoute.assign(numRoutes_FirstEchelon, std::vector<int>(numPeriods, 0));

	// Get solution values of decision variables
	for (int t = 0; t < numPeriods; ++t)
	{
		sol.productionSetup[t] = cplex.getIntValue(y[t]);
		sol.productionQuantity[t] = cplex.getValue(p[t]);
		sol.plantInventory[t] = cplex.getValue(I_plant[t]);

		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			selectedRoute[routeInd][t] = cplex.getIntValue(o[routeInd][t]);
			if (selectedRoute[routeInd][t] == 1)
			{
				sol.routePlantToWarehouse[t].push_back(optimalRoutes_FirstEchelon[routeInd]);
			}
		}

		for (int w = 0; w < numWarehouses; ++w)
		{
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				sol.deliveryQuantityToWarehouse[routeInd][w][t] = cplex.getValue(q[routeInd][w][t]);
			}
			for (int s = 0; s < numScenarios; ++s)
			{
				sol.warehouseInventory[w][t][s] = cplex.getValue(I_warehouse[w][t][s]);
			}
		}

		for (int rI = 0; rI < numRetailers; ++rI)
		{
			for (int s = 0; s < numScenarios; ++s)
			{
				sol.retailerInventory[rI][t][s] = cplex.getValue(I_retailer[rI][t][s]);
				sol.retailerUnmetDemand[rI][t][s] = cplex.getValue(b_retailer[rI][t][s]);
				for (int w = 0; w < numWarehouses; ++w)
				{
					sol.deliveryQuantityToRetailer[rI][w][t][s] = cplex.getValue(e[rI][w][t][s]);
				}
			}
		}
	}
}

void MWPRP::CalculateCostsForEachPart()
{
	for (int t = 0; t < numPeriods; ++t)
	{
		sol.setupCost += sol.productionSetup[t] * setupCost;
		sol.productionCost += unitProdCost * sol.productionQuantity[t];
		sol.holdingCostPlant += unitHoldingCostPlant * sol.plantInventory[t];

		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
		{
			if (selectedRoute[routeInd][t] == 1)
			{
				sol.transportationCostPlantToWarehouse += routeCosts_FirstEchelon[routeInd];
			}
		}

		for (int s = 0; s < numScenarios; ++s)
		{
			for (int w = 0; w < numWarehouses; ++w)
			{
				sol.holdingCostWarehouse_Avg += probability[s] * unitHoldingCostWarehouse[w] * sol.warehouseInventory[w][t][s];
			}

			for (int rI = 0; rI < numRetailers; ++rI)
			{
				sol.holdingCostRetailer_Avg += probability[s] * unitHoldingCostRetailer[rI] * sol.retailerInventory[rI][t][s];
				sol.costOfUnmetDemand_Avg += probability[s] * unmetDemandPenalty[rI] * sol.retailerUnmetDemand[rI][t][s];
			}
		}
	}
}

// template <typename T>
// void MWPRP::PrintVariable(const std::string &varName, const T &data, double threshold, std::vector<int> indices, int depth)
// {
// 	if (indices.size() == depth)
// 	{
// 		for (int i = 0; i < data.size(); ++i)
// 		{
// 			auto newIndices = indices;
// 			newIndices.push_back(i);
// 			PrintVariable(varName, data[i], threshold, newIndices, depth + 1);
// 		}
// 	}
// 	else
// 	{
// 		for (int i = 0; i < data.size(); ++i)
// 		{
// 			if (data[i] > threshold)
// 			{
// 				std::cout << varName << "[";
// 				for (int j = 0; j < indices.size(); ++j)
// 				{
// 					std::cout << indices[j] + 1 << "][";
// 				}
// 				std::cout << i + 1 << "] = " << data[i] << std::endl;
// 			}
// 		}
// 	}
// }

void MWPRP::DisplayProductionSetupVars()
{
	for (int t = 0; t < numPeriods; ++t)
	{
		std::cout << "y[" << t + 1 << "] = " << sol.productionSetup[t] << std::endl;
	}
}

void MWPRP::DisplayProductionQuantVars()
{
	for (int t = 0; t < numPeriods; ++t)
	{
		std::cout << "p[" << t + 1 << "] = " << sol.productionQuantity[t] << std::endl;
	}
}

void MWPRP::DisplayPlantInventoryVars()
{
	for (int t = 0; t < numPeriods; ++t)
	{
		std::cout << "I_plant[" << t + 1 << "] = " << sol.plantInventory[t] << std::endl;
	}
}

void MWPRP::DisplayWarehouseInventoryVars()
{
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int w = 0; w < numWarehouses; ++w)
			{
				std::cout << "I_warehouse[" << w + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << sol.warehouseInventory[w][t][s] << std::endl;
			}
		}
	}
}

void MWPRP::DisplayRetailerInventoryVars()
{
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int rI = 0; rI < numRetailers; ++rI)
			{
				std::cout << "I_retailer[" << rI + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << sol.retailerInventory[rI][t][s] << std::endl;
			}
		}
	}
}

void MWPRP::DisplayRetailerUnmetDemandVars()
{
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int rI = 0; rI < numRetailers; ++rI)
			{
				std::cout << "b_retailer[" << rI + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << sol.retailerUnmetDemand[rI][t][s] << std::endl;
			}
		}
	}
}

void MWPRP::DisplayFirstEchelonRouteVars()
{
	for (int t = 0; t < numPeriods; ++t)
	{
		int routeInd = 1;
		for (const auto &route : sol.routePlantToWarehouse[t])
		{
			std::cout << "Period: " << t + 1 << ", Route: " << routeInd << " : ";
			if (!route.empty())
			{
				for (auto it = route.begin(); it != route.end() - 1; ++it)
				{
					std::cout << *it << " -> ";
				}
				std::cout << route.back() << std::endl;
			}
			++routeInd;
		}
	}
}

void MWPRP::DisplayDeliveryQuantityToWarehousesVars()
{
	for (int t = 0; t < numPeriods; ++t)
	{
		for (int w = 0; w < numWarehouses; ++w)
		{
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				std::cout << "q[" << routeInd + 1 << "][" << w + 1 << "][" << t + 1 << "] = " << sol.deliveryQuantityToWarehouse[routeInd][w][t] << std::endl;
			}
		}
	}
}

// void MWPRP::DisplaySecondEchelonRouteVars()
// {
// 	for (int s = 0; s < numScenarios; ++s)
// 	{
// 		for (int t = 0; t < numPeriods; ++t)
// 		{
// 			int routeInd = 1;
// 			for (const auto& route : sol.routeWarehouseToRetailer[t][s])
// 			{
// 				std::cout << "Period: " << t + 1 << ", Scenario: " << s + 1 << ", Route: " << routeInd << " : ";
// 				if (!route.empty())
// 				{
// 					for (auto it = route.begin(); it != route.end() - 1; ++it)
// 					{
// 						std::cout << *it << " -> ";
// 					}
// 					std::cout << route.back() << std::endl;
// 				}
// 				++routeInd;
// 			}
// 		}
// 	}
// }

void MWPRP::DisplayDeliveryQuantityToRetailersVars()
{
	for (int s = 0; s < numScenarios; ++s)
	{
		for (int t = 0; t < numPeriods; ++t)
		{
			for (int w = 0; w < numWarehouses; ++w)
			{
				for (int rI = 0; rI < numRetailers; ++rI)
				{
					std::cout << "e[" << w + 1 << "][" << rI + 1 << "][" << t + 1 << "][" << s + 1 << "] = " << sol.deliveryQuantityToRetailer[rI][w][t][s] << std::endl;
				}
			}
		}
	}
}

// void PRPSL_BC::DefineWarmStartSolution(IloEnv &env, IloCplex &cplex, const Solution &warmStart)
// {
// 	IloNumVarArray startVar(env);
// 	IloNumArray startVal(env);
// 	for (int t = 0; t < numPeriods; ++t)
// 	{
// 		startVar.add(y[t]);
// 		startVal.add(warmStart.productionSetup[t]);
// 	}

// 	for (int s = 0; s < numScenarios; ++s)
// 	{
// 		for (int t = 0; t < numPeriods; ++t)
// 		{
// 			startVar.add(p[t][s]);
// 			startVal.add(warmStart.productionQuantity[t][s]);
// 		}
// 	}

// 	for (int s = 0; s < numScenarios; ++s)
// 	{
// 		for (int t = 0; t < numPeriods; ++t)
// 		{
// 			for (int nodeIndex = 0; nodeIndex < params.numNodes; ++nodeIndex)
// 			{
// 				startVar.add(I[nodeIndex][t][s]);
// 				startVal.add(warmStart.inventory[nodeIndex][t][s]);
// 			}
// 		}
// 	}

// 	for (int s = 0; s < numScenarios; ++s)
// 	{
// 		for (int t = 0; t < numPeriods; ++t)
// 		{
// 			for (int nodeIndex = 1; nodeIndex < params.numNodes; ++nodeIndex)
// 			{
// 				startVar.add(b[nodeIndex][t][s]);
// 				startVal.add(warmStart.backlog[nodeIndex][t][s]);
// 			}
// 		}
// 	}

// 	for (int s = 0; s < numScenarios; ++s)
// 	{
// 		for (int t = 0; t < numPeriods; ++t)
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

// 	for (int s = 0; s < numScenarios; ++s)
// 	{
// 		for (int t = 0; t < numPeriods; ++t)
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

// 	for (int s = 0; s < numScenarios; ++s)
// 	{
// 		for (int t = 0; t < numPeriods; ++t)
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
// 		for (int s = 0; s < numScenarios; ++s)
// 		{
// 			for (int t = 0; t < numPeriods; ++t)
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
// 		for (int s = 0; s < numScenarios; ++s)
// 		{
// 			for (int t = 0; t < numPeriods; ++t)
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
