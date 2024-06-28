#include "IRPWS.h"

IRPWS::IRPWS(const ParameterSetting &parameters, const Solution &solution, int w, int s)
	: params(parameters),
	  sol_FE(solution),
	  warehouse(w),
	  scenario(s),
	  THRESHOLD(1e-2),
	  save_lpFile(true),
	  save_mpsResultFile(false)
{
	// ----------------------------------------------------------------------------------------------------------
	RATW = params.getRetailersAssignedToWarehouse();
	numRetailer_w.resize(params.numWarehouses);
	for (int w = 0; w < params.numWarehouses; ++w)
	{
		numRetailer_w[w] = RATW[w].size();
	}

	warehouseInventory.resize(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	warehouseInventory = sol_FE.warehouseInventory;

	numRoutes_FirstEchelon = sol_FE.deliveryQuantityToWarehouse.size();
	sumDeliveredToWarehouse.resize(params.numWarehouses, vector<double>(params.numPeriods, 0.0));
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int w = 0; w < params.numWarehouses; ++w)
		{
			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
			{
				sumDeliveredToWarehouse[w][t] += sol_FE.deliveryQuantityToWarehouse[routeInd][w][t];
			}
		}
	}

	dualValues_WarehouseInventoryLB.resize(params.numPeriods, 0.0);
}

bool IRPWS::Solve()
{
	try
	{
		IloEnv env;
		IloModel model(env);
		IloCplex cplex(model);

		auto startTime = std::chrono::high_resolution_clock::now();

		DefineVariables(env, model);
		DefineObjectiveFunction(env, model);
		DefineConstraints(env, model);

		/* Assure linear mappings between the presolved and original models */
		cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

		if (save_lpFile)
		{
			string directory = "../cplexFiles/lpModel/";
			string lpFileName = directory + "IRPWS_NW" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numRetailers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_scenario" + std::to_string(scenario) + "_warehouse" + std::to_string(warehouse) + "_Ins" + params.instance.c_str() + ".lp";

			// Export the model to an LP file
			cplex.exportModel(lpFileName.c_str());
		}

		// Extract model
		cplex.extract(model);

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
				string solFileName = directory + "IRPWS_NW" + std::to_string(params.numWarehouses) + "_NR" + std::to_string(params.numRetailers) + "_KP" + std::to_string(params.numVehicles_Plant) + "_KW" + std::to_string(params.numVehicles_Warehouse) + "_T" + std::to_string(params.numPeriods) + "_S" + std::to_string(params.numScenarios) + "_scenario" + std::to_string(scenario) + "_warehouse" + std::to_string(warehouse) + "_Ins" + params.instance.c_str();

				// Export the model to an LP file
				cplex.writeSolution(solFileName.c_str());
			}

			for (int t = 0; t < params.numPeriods; ++t)
			{
				dualValues_WarehouseInventoryLB[t] = cplex.getDual(deliveryConstraints[t]);
			}

			RetrieveSolutions(cplex);
		}
		else if (cplex.getStatus() == IloAlgorithm::Infeasible)
		{
			status = "Infeasible";
			cout << "Problem is infeasible" << endl;
		}
		else
		{
			status = "Undefined";
			cout << "Solver terminated with status: " << status << endl;
		}

		auto currentTime = std::chrono::high_resolution_clock::now();
		auto CPUtime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Timer (seconds): " << std::fixed << std::setprecision(4) << CPUtime << endl;

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

void IRPWS::DefineVariables(IloEnv &env, IloModel &model)
{
	// Define Decision Variables

	// Initialize Variable Manager
	VariableManager varManager(env);
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define I[i][t][s] variables (Retailer i inventory at the end of period t under scenario (s))
	//
	I = varManager.create2D(numRetailer_w[warehouse] + 1, params.numPeriods);
	for (int i = 0; i < numRetailer_w[warehouse] + 1; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			if (i == 0)
			{
				string varName = "I_warehouse[" + std::to_string(i) + "][" + std::to_string(t) + "]";
				I[i][t] = IloNumVar(env, 0.0, 0.0, IloNumVar::Float, varName.c_str());
				model.add(I[i][t]);
			}
			else
			{
				string varName = "I_retailer[" + std::to_string(i) + "][" + std::to_string(t) + "]";
				I[i][t] = IloNumVar(env, 0.0, params.storageCapacity_Retailer[RATW[warehouse][i - 1]], IloNumVar::Float, varName.c_str());
				model.add(I[i][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define b[i][t] variables (amount of unmet params.demand in period t (retailers are assigned to warehouses, thus i in N_w))
	b = varManager.create2D(numRetailer_w[warehouse] + 1, params.numPeriods);
	for (int i = 0; i < numRetailer_w[warehouse] + 1; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			if (i == 0)
			{
				string varName = "b_warehouse[" + std::to_string(i) + "][" + std::to_string(t) + "]";
				b[i][t] = IloNumVar(env, 0.0, 0.0, IloNumVar::Float, varName.c_str());
				model.add(b[i][t]);
			}
			else
			{
				string varName = "b_retailer[" + std::to_string(i) + "][" + std::to_string(t) + "]";
				b[i][t] = IloNumVar(env, 0.0, params.demand[RATW[warehouse][i - 1]][t][scenario], IloNumVar::Float, varName.c_str());
				model.add(b[i][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define w[i][t] variables (delivery to retailer i in period t (retailers are assigned to warehouses, thus i in N_w))
	w = varManager.create2D(numRetailer_w[warehouse] + 1, params.numPeriods);
	for (int i = 0; i < numRetailer_w[warehouse] + 1; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			if (i == 0)
			{
				string varName = "w_warehouse[" + std::to_string(i) + "][" + std::to_string(t) + "]";
				w[i][t] = IloNumVar(env, 0.0, 0.0, IloNumVar::Float, varName.c_str());
				model.add(w[i][t]);
			}
			else
			{
				string varName = "w_retailer[" + std::to_string(i) + "][" + std::to_string(t) + "]";
				w[i][t] = IloNumVar(env, 0.0,params.DeliveryUB_perRetailer[RATW[warehouse][i - 1]][t][scenario], IloNumVar::Float, varName.c_str()); // check delivery capacity
				model.add(w[i][t]);
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define z[i][t] variables (load on the vehicle immediately before making delivery to retailer i in period t (retailers are assigned to warehouses, thus i in N_w))
	z = varManager.create2D(numRetailer_w[warehouse] + 1, params.numPeriods);
	for (int i = 0; i < numRetailer_w[warehouse] + 1; ++i)
	{
		for (int t = 0; t < params.numPeriods; ++t)
		{
			string varName = "z[" + std::to_string(i) + "][" + std::to_string(t) + "]";
			z[i][t] = IloNumVar(env, 0.0, params.vehicleCapacity_Warehouse, IloNumVar::Float, varName.c_str());
			model.add(z[i][t]);
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
	// Define x[i][j][t] variables (1 if node i immediately precedes node j on a delivery route in period t; 0 otherwise)
	// Note that node 0 denotes the warehouse and other nodes denote the retailers assigned to warehouse w, Thus we add 1 to retailer nodes!
	x = varManager.create3D(numRetailer_w[warehouse] + 1, numRetailer_w[warehouse] + 1, params.numPeriods);
	for (int i = 0; i < numRetailer_w[warehouse] + 1; ++i)
	{
		for (int j = 0; j < numRetailer_w[warehouse] + 1; ++j)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				string varName = "x[" + std::to_string(i) + "][" + std::to_string(j) + "][" + std::to_string(t) + "]";
				// x[i][j][t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
				x[i][j][t] = IloNumVar(env, 0.0, 1.0, IloNumVar::Float, varName.c_str());
				model.add(x[i][j][t]);

				if (i == j)
				{
					model.add(x[i][j][t] == 0.0);
				}
			}
		}
	}
	// -------------------------------------------------------------------------------------------------------------------------------
}

void IRPWS::DefineObjectiveFunction(IloEnv &env, IloModel &model)
{
	// Define objective function
	IloExpr obj(env);
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 1; i < numRetailer_w[warehouse] + 1; ++i)
		{
			obj += params.probability[scenario] * params.unitHoldingCost_Retailer[RATW[warehouse][i - 1]] * I[i][t];
			obj += params.probability[scenario] * params.unmetDemandPenalty[RATW[warehouse][i - 1]] * b[i][t];
		}

		for (int i = 0; i < numRetailer_w[warehouse] + 1; ++i)
		{
			for (int j = 0; j < numRetailer_w[warehouse] + 1; ++j)
			{
				if (i != j)
				{
					if (i == 0)
					{
						obj += params.probability[scenario] * params.transportationCost_SecondEchelon[warehouse][RATW[warehouse][j - 1] + params.numWarehouses] * x[i][j][t];
					}
					else if (j == 0)
					{
						obj += params.probability[scenario] * params.transportationCost_SecondEchelon[RATW[warehouse][i - 1] + params.numWarehouses][warehouse] * x[i][j][t];
					}
					else
					{
						obj += params.probability[scenario] * params.transportationCost_SecondEchelon[RATW[warehouse][i - 1] + params.numWarehouses][RATW[warehouse][j - 1] + params.numWarehouses] * x[i][j][t];
					}
				}
			}
		}
	}
	model.add(IloMinimize(env, obj));
}

void IRPWS::DefineConstraints(IloEnv &env, IloModel &model)
{
	/* Define Constraints */
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Balance Constraints (Retailers):
			I[i][t]^([s]) = I[i][t-1]^([s]) + w[i][t]^([s]) - d[i][t]^([s]) + b[i][t]^([s]) 	for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 1; i < numRetailer_w[warehouse] + 1; ++i)
		{
			string constraintName = "RetailerInventoryBalance(" + std::to_string(i) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			if (t == 0)
			{
				expr += I[i][t];
				expr += -w[i][t];
				expr += -b[i][t];
				IloConstraint RetailerInventoryBalanceConstraint(expr == params.initialInventory_Retailer[RATW[warehouse][i - 1]] - params.demand[RATW[warehouse][i - 1]][t][scenario]);
				expr.end();

				model.add(RetailerInventoryBalanceConstraint).setName(constraintName.c_str());
			}
			else
			{
				expr += I[i][t];
				expr += -I[i][t - 1];
				expr += -w[i][t];
				expr += -b[i][t];
				IloConstraint RetailerInventoryBalanceConstraint(expr == -params.demand[RATW[warehouse][i - 1]][t][scenario]);
				expr.end();

				model.add(RetailerInventoryBalanceConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Inventory Capacity Constraints (Retailers):
			I[i][t] + d[i][t]^([s]) <= params.storageCapacity_Retailer[i] 		for all i in N_w, t in T
	// */
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 1; i < numRetailer_w[warehouse] + 1; ++i)
		{
			string constraintName = "RetailerInventoryCapacity(" + std::to_string(i) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			expr += I[i][t];
			expr += params.demand[RATW[warehouse][i - 1]][t][scenario];
			IloConstraint RetailerInventoryCapacityConstraint(expr <= params.storageCapacity_Retailer[RATW[warehouse][i - 1]]);
			expr.end();

			model.add(RetailerInventoryCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Ensure that there is at most one successor on a route for each retailer:
			sum(j in N & j != i, t in T) x[i][j][t]^([s]) <= 1			for all i in N_w t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 1; i < numRetailer_w[warehouse] + 1; ++i)
		{
			string constraintName = "OneSuccessor(" + std::to_string(i) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			for (int j = 0; j < numRetailer_w[warehouse] + 1; ++j)
			{
				if (j != i)
				{
					expr += x[i][j][t];
				}
			}
			IloConstraint oneSuccessorConstraint(expr <= 1);
			expr.end();

			model.add(oneSuccessorConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define flow balance constraints:
			sum(i in N & i != j) x[i][j][t]^([s]) = sum(i in N & i != j) x[j][i][t]^([s])			for all j in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int j = 1; j < numRetailer_w[warehouse] + 1; ++j)
		{
			string constraintName = "FlowBalance(" + std::to_string(j) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			for (int i = 0; i < numRetailer_w[warehouse] + 1; ++i)
			{
				if (i != j)
				{
					expr += x[i][j][t];
					expr += -x[j][i][t];
				}
			}
			IloConstraint flowBalanceConstraint(expr == 0);
			expr.end();

			model.add(flowBalanceConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define Delivery limit constraints:
			w[i][t]^([s]) <= M[i][t]^([s]) * sum(j in N) x[i][j][t]^([s])			for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 1; i < numRetailer_w[warehouse] + 1; ++i)
		{
			string constraintName = "DeliveryLimit(" + std::to_string(i) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			expr += w[i][t];
			for (int j = 0; j < numRetailer_w[warehouse] + 1; ++j)
			{
				expr += -params.DeliveryUB_perRetailer[RATW[warehouse][i - 1]][t][scenario] * x[i][j][t];
			}
			IloConstraint deliveryLimitConstraint(expr <= 0);
			expr.end();

			model.add(deliveryLimitConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define the fleet size constraint:
			sum(j in N_w) x[0][j][t]^([s]) <= params.numVehicles_Warehouse			for all t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "FleetSize(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		for (int j = 1; j < numRetailer_w[warehouse] + 1; ++j)
		{
			expr += x[0][j][t];
		}
		IloConstraint fleetSizeConstraint(expr <= params.numVehicles_Warehouse);
		expr.end();

		model.add(fleetSizeConstraint).setName(constraintName.c_str());
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define vehicle load capacity constraint:
			z[i][t]^([s]) <= params.vehicleCapacity_Warehouse			for all i in N_w, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 1; i < numRetailer_w[warehouse] + 1; ++i)
		{
			string constraintName = "VehicleLoadCapacity(" + std::to_string(i) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

			IloExpr expr(env);
			expr += z[i][t];
			IloConstraint vehicleLoadCapacityConstraint(expr <= params.vehicleCapacity_Warehouse);
			expr.end();

			model.add(vehicleLoadCapacityConstraint).setName(constraintName.c_str());
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		MTZ Constraints:
			Guarantee that the load on the vehicle before visiting retailer j on a route must
			be less than or equal to the load just before visiting predecessor retailer i

			z[j][t]^([s]) <= z[i][t]^([s]) - w[i][t]^([s]) + W[t][s] * (1 - x[i][j][t]^([s]))			for all i in N_w, j in N, t in T
	*/
	for (int t = 0; t < params.numPeriods; ++t)
	{
		for (int i = 1; i < numRetailer_w[warehouse] + 1; ++i)
		{
			for (int j = 0; j < numRetailer_w[warehouse] + 1; ++j)
			{
				string constraintName = "MTZ(" + std::to_string(i) + "," + std::to_string(j) + "," + std::to_string(t + 1) + "," + std::to_string(scenario + 1) + ")";

				IloExpr expr(env);
				expr += z[j][t];
				expr += -z[i][t];
				expr += w[i][t];
				expr += -params.DeliveryUB[t][scenario] * (1 - x[i][j][t]);
				IloConstraint MTZConstraint(expr <= 0);
				expr.end();

				model.add(MTZConstraint).setName(constraintName.c_str());
			}
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------
	/*
		Define delivery constraints to limit the deliveries to the available inventory of the warehouse:
			sum(i in N_w) w[i][t]^([s]) <= warehouseInventory[warehouse][t-1]^([s]) + sum(r in R) warehouseDelivery[r][warehouse][t]			for all t in T
	*/
	deliveryConstraints.resize(params.numPeriods);

	for (int t = 0; t < params.numPeriods; ++t)
	{
		string constraintName = "DeliveryLimit(" + std::to_string(t + 1) + ")";

		IloExpr expr(env);
		for (int i = 1; i < numRetailer_w[warehouse] + 1; ++i)
		{
			expr += w[i][t];
		}
		// Store each constraint in the vector with its handle
		if (t == 0)
		{
			deliveryConstraints[t] = IloRange(expr <= params.initialInventory_Warehouse[warehouse] + sumDeliveredToWarehouse[warehouse][t]);
		}
		else
		{
			deliveryConstraints[t] = IloRange(expr <= warehouseInventory[warehouse][t - 1][scenario] + sumDeliveredToWarehouse[warehouse][t]);
		}
		expr.end();

		model.add(deliveryConstraints[t]); // Add constraint to model
	}
}

void IRPWS::RetrieveSolutions(IloCplex &cplex)
{
	// Retrieve solution
	// initilize variables
	retailerInventory_S.assign(params.numRetailers, vector<double>(params.numPeriods, 0.0));
	unmetDemandQuantity_S.assign(params.numRetailers, vector<double>(params.numPeriods, 0.0));
	deliveryQuantityToRetailers_W_S.assign(params.numRetailers, vector<double>(params.numPeriods, 0.0));

	// Get solution values of decision variables
	for (int t = 0; t < params.numPeriods; ++t)
	{
		int retInd = 1;
		for (int i = 0; i < params.numRetailers; ++i)
		{
			if (std::find(RATW[warehouse].begin(), RATW[warehouse].end(), i) != RATW[warehouse].end())
			{
				retailerInventory_S[i][t] = cplex.getValue(I[retInd][t]);
				unmetDemandQuantity_S[i][t] = cplex.getValue(b[retInd][t]);
				deliveryQuantityToRetailers_W_S[i][t] = cplex.getValue(w[retInd++][t]);
			}
		}
	}		
}

vector<double> IRPWS::getDualValues_WarehouseInventoryLB()
{
	return dualValues_WarehouseInventoryLB;
}

vector<vector<double>> IRPWS::getRetailerInventory_S()
{
	return retailerInventory_S;
}

vector<vector<double>> IRPWS::getUnmetDemandQuantity_S()
{
	return unmetDemandQuantity_S;
}

vector<vector<double>> IRPWS::getDeliveryQuantityToRetailers_W_S()
{
	return deliveryQuantityToRetailers_W_S;
}

// void IRPWS::RetrieveSolutions(IloCplex &cplex)
// {
// 	// Retrieve solution
// 	// initilize variables
// 	sol.productionSetup.assign(params.numPeriods, 0);
// 	sol.productionQuantity.assign(params.numPeriods, 0.0);
// 	sol.plantInventory.assign(params.numPeriods, 0.0);
// 	sol.warehouseInventory.assign(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
// 	sol.retailerInventory.assign(params.numRetailers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
// 	sol.deliveryQuantityToWarehouse.assign(numRoutes_FirstEchelon, vector<vector<double>>(params.numWarehouses, vector<double>(params.numPeriods, 0.0)));
// 	sol.routePlantToWarehouse.resize(params.numPeriods);

// 	selectedRoute.assign(numRoutes_FirstEchelon, vector<int>(params.numPeriods, 0));

// 	// Get solution values of decision variables
// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		sol.productionSetup[t] = cplex.getIntValue(y[t]);
// 		sol.productionQuantity[t] = cplex.getValue(p[t]);
// 		sol.plantInventory[t] = cplex.getValue(I_plant[t]);

// 		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
// 		{
// 			selectedRoute[routeInd][t] = cplex.getIntValue(o[routeInd][t]);
// 			if (selectedRoute[routeInd][t] == 1)
// 			{
// 				sol.routePlantToWarehouse[t].push_back(optimalRoutes_FirstEchelon[routeInd]);
// 			}
// 		}

// 		for (int w = 0; w < params.numWarehouses; ++w)
// 		{
// 			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
// 			{
// 				sol.deliveryQuantityToWarehouse[routeInd][warehouse][t] = cplex.getValue(q[routeInd][warehouse][t]);
// 			}

// 			for (int s = 0; s < params.numScenarios; ++s)
// 			{
// 				sol.warehouseInventory[warehouse][t][s] = cplex.getValue(I_warehouse[warehouse][t][s]);
// 			}
// 		}
// 	}
// }

// void IRPWS::CalculateCostsForEachPart()
// {
// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		sol.setupCost += sol.productionSetup[t] * setupCost;
// 		sol.productionCost += unitProdCost * sol.productionQuantity[t];
// 		sol.holdingCostPlant += params.unitHoldingCost_Plant * sol.plantInventory[t];

// 		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
// 		{
// 			if (selectedRoute[routeInd][t] == 1)
// 			{
// 				sol.transportationCostPlantToWarehouse += routeCosts_FirstEchelon[routeInd];
// 			}
// 		}

// 		for (int s = 0; s < params.numScenarios; ++s)
// 		{
// 			for (int w = 0; w < params.numWarehouses; ++w)
// 			{
// 				sol.holdingCostWarehouse_Avg += params.probability[s] * params.unitHoldingCost_Warehouse[warehouse] * sol.warehouseInventory[warehouse][t][s];
// 			}
// 		}
// 	}
// }

// void IRPWS::DisplayProductionSetupVars()
// {
// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		if (sol.productionSetup[t] == 1)
// 		{
// 			cout << "y[" << t + 1 << "] = " << sol.productionSetup[t] << endl;
// 		}
// 	}
// }

// void IRPWS::DisplayProductionQuantVars()
// {
// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		if (sol.productionQuantity[t] > THRESHOLD)
// 		{
// 			cout << "p[" << t + 1 << "] = " << sol.productionQuantity[t] << endl;
// 		}
// 	}
// }

// void IRPWS::DisplayPlantInventoryVars()
// {
// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		if (sol.plantInventory[t] > THRESHOLD)
// 		{
// 			cout << "I_plant[" << t + 1 << "] = " << sol.plantInventory[t] << endl;
// 		}
// 	}
// }

// void IRPWS::DisplayWarehouseInventoryVars()
// {
// 	for (int s = 0; s < params.numScenarios; ++s)
// 	{
// 		for (int t = 0; t < params.numPeriods; ++t)
// 		{
// 			for (int w = 0; w < params.numWarehouses; ++w)
// 			{
// 				if (sol.warehouseInventory[warehouse][t][s] > THRESHOLD)
// 				{
// 					cout << "I_warehouse[" << w + 1 << "][" << t + 1 << "][" << scenario + 1 << "] = " << sol.warehouseInventory[warehouse][t][s] << endl;
// 				}
// 			}
// 		}
// 	}
// }

// void IRPWS::DisplayFirstEchelonRouteVars()
// {
// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
// 		{
// 			if (selectedRoute[routeInd][t] == 1)
// 			{
// 				cout << "o[" << routeInd + 1 << "][" << t + 1 << "] = " << selectedRoute[routeInd][t] << endl;
// 			}
// 		}
// 	}

// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		int routeInd = 1;
// 		for (const auto &route : sol.routePlantToWarehouse[t])
// 		{
// 			cout << "Period: " << t + 1 << ", Route: " << routeInd << " : ";
// 			if (!route.empty())
// 			{
// 				for (auto it = route.begin(); it != route.end() - 1; ++it)
// 				{
// 					cout << *it << " -> ";
// 				}
// 				cout << route.back() << endl;
// 			}
// 			++routeInd;
// 		}
// 	}
// }

// void IRPWS::DisplayDeliveryQuantityToWarehousesVars()
// {
// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		for (int w = 0; w < params.numWarehouses; ++w)
// 		{
// 			for (int routeInd = 0; routeInd < numRoutes_FirstEchelon; ++routeInd)
// 			{
// 				if (sol.deliveryQuantityToWarehouse[routeInd][warehouse][t] > THRESHOLD)
// 				{
// 					cout << "q[" << routeInd + 1 << "][" << w + 1 << "][" << t + 1 << "] = " << sol.deliveryQuantityToWarehouse[routeInd][warehouse][t] << endl;
// 				}
// 			}
// 		}
// 	}
// }
