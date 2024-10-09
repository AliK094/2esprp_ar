#include "headers.h"
#include "ParameterSetting.h"
#include "algorithms.h"

int main(int argc, char *argv[])
{
	string solutionAlgorithm = argv[1];
	if (argc != 13)
	{
		cerr << "Wrong number of arguments" << endl;
		cerr << "Usage: " << argv[0] << " <solutionAlgorithm> <inputFilename> "
			 << "<Number of Warehouses> <Number of Customers> <Planning Horizon> <Number of Vehicles at the plant> "
			 << "<Number of Vehicles Per each Warehouse> <Number of Scenarios> <Penalty Coefficient (For a Unit of Unmet demand)>"
			 << "<Uncertainty Range> <ProbabilityFunction> <instanceName>" << endl;
		return EXIT_FAILURE;
	}

	cout << "\n\n\nSolve The Stochastic Two-Echelon PRP with Adaptive Routing. " << endl;

	ParameterSetting params(argc, argv);
	if (!params.setParameters())
	{
		cerr << "Unable to Set Parameters" << endl;
		return EXIT_FAILURE;
	}

	Algorithms alg(solutionAlgorithm, params);

	if (solutionAlgorithm == "Hybrid-ILS")
	{
		// Solve the problem with Hybrid-ILS
		cout << "\n-------------------------------------------------------------------" << endl;
		bool success = alg.solve_S2EPRP_HILS();
		if (!success)
		{
			return EXIT_FAILURE;
		}
	}
	else if (solutionAlgorithm == "BC")
	{
		bool success = alg.solve_S2EPRP_BC();
		if (!success)
		{
			return EXIT_FAILURE;
		}
	}

	// SolutionManager solMgr_ILS(params, solutionAlgorithm);
	// solMgr_ILS.saveSolution(solFinal);
	// solMgr_ILS.checkFeasibility();

	return EXIT_SUCCESS;
}

// #include "IRPWS.h"
// {
// S2EPRP_BC s2eprp_bc(params);
// if (!s2eprp_bc.Solve())
// {
// 	return EXIT_FAILURE;
// }
// Solution sol_S2E = s2eprp_bc.getSolution();

// SolutionManager solMgr_ILS(params, "ILS");
// 	solMgr_ILS.saveSolution(solFinal);
// 	solMgr_ILS.checkFeasibility();

// SolutionManager solMgr_BC(params, "BC");
// solMgr_BC.saveSolution(sol_S2E);
// solMgr_BC.checkFeasibility();

// exit(0);

// const int maxIteration = 10;
// vector<vector<vector<double>>> dualValues_WarehouseInventoryLB(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
// vector<vector<vector<double>>> warehouseInventory_Previous(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
// // vector<Solution> solPool;
// // vector<Result> resPool;
// for (int iter = 0; iter < maxIteration; ++iter)
// {
// 	cout << "\n\n\nStart Solving The Problem With Hybrid-ILS." << endl;
// 	cout << "\niteration: " << iter + 1 << endl;
// 	Solution sol_Current;
// 	Result res_Current;
// 	// Solve the problem

// 	cout << "Solve The First-Echelon Problem" << endl;
// 	MWPRP_FE mwprp_fe(params, warehouseInventory_Previous, dualValues_WarehouseInventoryLB);
// 	if (!mwprp_fe.Solve())
// 	{
// 		return EXIT_FAILURE;
// 	}
// 	Solution sol_FE = mwprp_fe.getSolution();

// 	// --------------------------------------------------------
// 	// Update the solution
// 	sol_Current.productionSetup = sol_FE.productionSetup;
// 	sol_Current.productionQuantity = sol_FE.productionQuantity;
// 	sol_Current.plantInventory = sol_FE.plantInventory;
// 	sol_Current.warehouseInventory = sol_FE.warehouseInventory;
// 	sol_Current.deliveryQuantityToWarehouse = sol_FE.deliveryQuantityToWarehouse;
// 	sol_Current.routesPlantToWarehouse = sol_FE.routesPlantToWarehouse;
// 	// Update the Costs
// 	sol_Current.setupCost = sol_FE.setupCost;
// 	sol_Current.productionCost = sol_FE.productionCost;
// 	sol_Current.holdingCostPlant = sol_FE.holdingCostPlant;
// 	sol_Current.holdingCostWarehouse_Avg = sol_FE.holdingCostWarehouse_Avg;
// 	sol_Current.transportationCostPlantToWarehouse = sol_FE.transportationCostPlantToWarehouse;

// 	// Update the Result
// 	res_Current.objValue += sol_Current.setupCost + sol_Current.productionCost + sol_Current.holdingCostPlant + sol_Current.holdingCostWarehouse_Avg + sol_Current.transportationCostPlantToWarehouse;

// 	auto custToWarehouse = params.getCustomersAssignedToWarehouse();
// 	auto warehouseToCust = params.getWarehouseAssignedToCustomer();
// 	ILS_SIRP ils_SIRP(params, sol_FE, custToWarehouse, warehouseToCust);
// 	bool status = ils_SIRP.run();
// 	if (!status)
// 	{
// 		return EXIT_FAILURE;
// 	}

// 	solFinal = ils_SIRP.getSolution();

// 	SolutionManager solMgr(params, solutionAlgorithm);
// 	solMgr.saveSolution(solFinal);
// 	solMgr.checkFeasibility();

// 	cout << "Done" << endl;

// S2EPRP_BC s2eprp_bc(params);
// if (!s2eprp_bc.Solve())
// {
// 	return EXIT_FAILURE;
// }
// Solution sol_S2E = s2eprp_bc.getSolution();
// break;
// --------------------------------------------------------
// auto warehouseInventory_Previous = sol_FE.warehouseInventory;

// auto CATW = params.getCustomersAssignedToWarehouse();
// sol_Current.customerInventory.resize(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
// sol_Current.customerUnmetDemand.resize(params.numCustomers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
// sol_Current.deliveryQuantityToCustomer.resize(params.numCustomers, vector<vector<vector<double>>>(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0))));
// dualValues_WarehouseInventoryLB.assign(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
// for (int s = 0; s < params.numScenarios; ++s)
// {
// 	for (int w = 0; w < params.numWarehouses; ++w)
// 	{
// 		cout << "\nSolving Scenario: " << s << ", Warehouse: " << w << endl;
// 		IRPWS irpws(params, sol_FE, w, s);
// 		if (!irpws.Solve())
// 		{
// 			return EXIT_FAILURE;
// 		}
// 		else
// 		{
// 			for (int t = 0; t < params.numPeriods; ++t)
// 			{
// 				dualValues_WarehouseInventoryLB[w][t][s] = irpws.getDualValues_WarehouseInventoryLB()[t];
// 				cout << "Dual Value warehouse " << w << " period " << t << " scenario " << s << " = " << std::setprecision(0) << std::fixed << dualValues_WarehouseInventoryLB[w][t][s] << endl;

// 				for (int i = 0; i < params.numCustomers; ++i)
// 				{
// 					if (std::find(CATW[s][t][w].begin(), CATW[s][t][w].end(), i) != CATW[s][t][w].end())
// 					{
// 						sol_Current.customerInventory[i][t][s] = irpws.getCustomerInventory_S()[i][t];
// 						sol_Current.customerUnmetDemand[i][t][s] = irpws.getUnmetDemandQuantity_S()[i][t];
// 						sol_Current.deliveryQuantityToCustomer[i][w][t][s] = irpws.getDeliveryQuantityToCustomers_W_S()[i][t];
// 					}
// 				}
// 			}
// 		}
// 	}
// }

// // Update the Costs
// for (int s = 0; s < params.numScenarios; ++s)
// {
// 	for (int t = 0; t < params.numPeriods; ++t)
// 	{
// 		for (int i = 0; i < params.numCustomers; ++i)
// 		{
// 			sol_Current.holdingCostCustomer_Avg += params.probability[s] * params.unitHoldingCost_Customer[i] * sol_Current.customerInventory[i][t][s];
// 			sol_Current.costOfUnmetDemand_Avg += params.probability[s] * params.unmetDemandPenalty[i] * sol_Current.customerUnmetDemand[i][t][s];
// 		}
// 	}
// }
// res_Current.objValue += sol_Current.holdingCostCustomer_Avg + sol_Current.costOfUnmetDemand_Avg;
// cout << "Objective Value: " << res_Current.objValue << endl;

// solPool.push_back(sol_Current);

// }

// SolutionManager solMgr(params, solutionAlgorithm);
// solMgr.saveSolution(solFinal);

// return EXIT_SUCCESS;
// }