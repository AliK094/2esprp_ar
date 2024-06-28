#include "headers.h"
#include "ParameterSetting.h"
#include "MWPRP_FE.h"
#include "IRPWS.h"

int main(int argc, char *argv[])
{

	if (argc != 13)
	{
		cerr << "Wrong number of arguments" << endl;
		cerr << "Usage: " << argv[0] << " <solutionAlgorithm> <inputFilename> "
				  << "<Number of Warehouses> <Number of Retailers> <Planning Horizon> <Number of Vehicles at the plant> "
				  << "<Number of Vehicles Per each Warehouse> <Number of Scenarios> <Penalty Coefficient (For a Unit of Unmet demand)>"
				  << "<Uncertainty Range> <ProbabilityFunction> <instanceName>" << endl;
		return EXIT_FAILURE;
	}

	cout << "Solve The Stochastic Two-Echelon PRP with Adaptive Routing. " << endl;

	ParameterSetting params(argc, argv);
	if (!params.setParameters())
	{
		cerr << "Unable to Set Parameters" << endl;
		return EXIT_FAILURE;
	}

	const int maxIteration = 2;
	vector<vector<vector<double>>> dualValues_WarehouseInventoryLB(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	vector<vector<vector<double>>> warehouseInventory_Previous(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
	vector<Solution> solPool;
	vector<Result> resPool;
	for (int iter = 0; iter < maxIteration; ++iter)
	{
		cout << "iteration: " << iter + 1 << endl;
		Solution sol_Current;
		Result res_Current;
		// Solve the problem
		MWPRP_FE mwprp_fe(params, warehouseInventory_Previous, dualValues_WarehouseInventoryLB);
		if (!mwprp_fe.Solve())
		{
			return EXIT_FAILURE;
		}
		Solution sol_FE = mwprp_fe.getSolution();

		// --------------------------------------------------------
		// Update the solution
		sol_Current.productionSetup = sol_FE.productionSetup;
		sol_Current.productionQuantity = sol_FE.productionQuantity;
		sol_Current.plantInventory = sol_FE.plantInventory;
		sol_Current.warehouseInventory = sol_FE.warehouseInventory;
		sol_Current.deliveryQuantityToWarehouse = sol_FE.deliveryQuantityToWarehouse;
		sol_Current.routePlantToWarehouse = sol_FE.routePlantToWarehouse;
		// Update the Costs
		sol_Current.setupCost = sol_FE.setupCost;
		sol_Current.productionCost = sol_FE.productionCost;
		sol_Current.holdingCostPlant = sol_FE.holdingCostPlant;
		sol_Current.holdingCostWarehouse_Avg = sol_FE.holdingCostWarehouse_Avg;
		sol_Current.transportationCostPlantToWarehouse = sol_FE.transportationCostPlantToWarehouse;

		// Update the Result
		res_Current.objValue += sol_Current.setupCost + sol_Current.productionCost + sol_Current.holdingCostPlant + sol_Current.holdingCostWarehouse_Avg + sol_Current.transportationCostPlantToWarehouse;
		// --------------------------------------------------------
		auto warehouseInventory_Previous = sol_FE.warehouseInventory;

		auto RATW = params.getRetailersAssignedToWarehouse();
		sol_Current.retailerInventory.resize(params.numRetailers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
		sol_Current.retailerUnmetDemand.resize(params.numRetailers, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
		sol_Current.deliveryQuantityToRetailer.resize(params.numRetailers, vector<vector<vector<double>>>(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0))));
		dualValues_WarehouseInventoryLB.assign(params.numWarehouses, vector<vector<double>>(params.numPeriods, vector<double>(params.numScenarios, 0.0)));
		for (int s = 0; s < params.numScenarios; ++s)
		{
			for (int w = 0; w < params.numWarehouses; ++w)
			{
				cout << "Solving Scenario: " << s << ", Warehouse: " << w << endl;
				IRPWS irpws(params, sol_FE, w, s);
				if (!irpws.Solve())
				{
					return EXIT_FAILURE;
				}
				else {
					for (int t = 0; t < params.numPeriods; ++t)
					{
						dualValues_WarehouseInventoryLB[w][t][s] = irpws.getDualValues_WarehouseInventoryLB()[t];
						cout << "Dual Value warehouse " << w << " period " << t << " scenario " << s << " = " << dualValues_WarehouseInventoryLB[w][t][s] << endl;

						for (int i = 0; i < params.numRetailers; ++i)
						{
							if (std::find(RATW[w].begin(), RATW[w].end(), i) != RATW[w].end())
							{
								sol_Current.retailerInventory[i][t][s] = irpws.getRetailerInventory_S()[i][t];
								sol_Current.retailerUnmetDemand[i][t][s] = irpws.getUnmetDemandQuantity_S()[i][t];
								sol_Current.deliveryQuantityToRetailer[i][w][t][s] = irpws.getDeliveryQuantityToRetailers_W_S()[i][t];
							}
						}
					}
				}
			}
		}

		// Update the Costs
		for (int s = 0; s < params.numScenarios; ++s)
		{
			for (int t = 0; t < params.numPeriods; ++t)
			{
				for (int i = 0; i < params.numRetailers; ++i)
				{
					sol_Current.holdingCostRetailer_Avg += params.probability[s] * params.unitHoldingCost_Retailer[i] * sol_Current.retailerInventory[i][t][s];
					sol_Current.costOfUnmetDemand_Avg += params.probability[s] * params.unmetDemandPenalty[i] * sol_Current.retailerUnmetDemand[i][t][s];
				}
			}
		}
		res_Current.objValue += sol_Current.holdingCostRetailer_Avg + sol_Current.costOfUnmetDemand_Avg;
		cout << "Objective Value: " << res_Current.objValue << endl;


		solPool.push_back(sol_Current);
	}

	return EXIT_SUCCESS;
}
