#include "SolutionManager.h"

SolutionManager::SolutionManager(const ParameterSetting &parameters, const string solAlg)
    : params(parameters),
      Algorithm(solAlg)
{
}

void SolutionManager::saveSolution(const SolutionFirstEchelon &solFE, const SolutionSecondEchelon &solSE)
{
    string resultsFileName;
    if (Algorithm == "Hybrid-ILS")
    {
        cout << "Saving solution for " << Algorithm << endl;
        resultsFileName = "../Results/Solutions/" + Algorithm + "/" + params.probabilityFunction.c_str() + "/S" + std::to_string(params.numScenarios) + "/Sol_S2EPRPAR_HHA_" + params.probabilityFunction.c_str() + "_" + params.instance.c_str() + "_S" + std::to_string(params.numScenarios)  + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
    }
    else if (Algorithm == "BC")
    {
        cout << "Saving solution for " << Algorithm << endl;
        resultsFileName = "../Results/Solutions/" + Algorithm + "/" + params.probabilityFunction.c_str() + "/S" + std::to_string(params.numScenarios) + "/Sol_S2EPRPAR_BC_" +  params.probabilityFunction.c_str() + "_" + params.instance.c_str() + "_S" + std::to_string(params.numScenarios)  + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
    }
    // }  else if (strcmp(Algorithm, "FR_BC") == 0){
    // 	snprintf(resultsFileName, sizeof(resultsFileName), "../Results/Solutions/FR_BC/%s/%s/S%d/ep%.2lf/Solution_FR_BC_%s_%s_%s_N%d_K%d_T%d_S%d_UR%.2lf_SL%.2lf.txt",
    //           probabilityFunction.c_str(), SLtype.c_str(), numScenarios, uncertaintyRange, instance.c_str(), SLtype.c_str(), probabilityFunction.c_str(), numNodes, numVehicles, numPeriods, numScenarios, uncertaintyRange, ServiceLevel);
    // }

    // Write the data to a file
    std::ofstream file(resultsFileName);
    if (file.is_open())
    {
        // Write Parameters
        file << params.numNodes_Total << " ";
        file << params.numWarehouses << " ";
        file << params.numCustomers << " ";
        file << params.numVehicles_Plant << " ";
        file << params.numVehicles_Warehouse << " ";
        file << params.numPeriods << " ";
        file << params.numScenarios << " " << endl;

        file << params.uncertaintyRange << " ";
        file << params.probabilityFunction << " ";
        file << params.unmetDemandPenaltyCoeff << " " << endl;

        file << std::fixed << std::setprecision(1) << params.unitProdCost << " ";
        file << std::fixed << std::setprecision(1) << params.setupCost << " ";
        file << std::fixed << std::setprecision(1) << params.prodCapacity << " ";
        file << std::fixed << std::setprecision(1) << params.vehicleCapacity_Plant << " ";
        file << std::fixed << std::setprecision(1) << params.vehicleCapacity_Warehouse << " " << endl;

        // Write Coordinate
        file << std::fixed << std::setprecision(1) << params.coordX_Plant << " ";
        file << std::fixed << std::setprecision(1) << params.coordY_Plant << " " << endl;

        for (const auto &d : params.coordX_Warehouse)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;
        for (const auto &d : params.coordY_Warehouse)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        for (const auto &d : params.coordX_Customer)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        for (const auto &d : params.coordY_Customer)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        // Write Holding Cost
        file << params.unitHoldingCost_Plant << endl;

        for (const auto &d : params.unitHoldingCost_Warehouse)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        for (const auto &d : params.unitHoldingCost_Customer)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        // Write Storage Capacity
        file << params.storageCapacity_Plant << endl;

        for (const auto &d : params.storageCapacity_Warehouse)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        for (const auto &d : params.storageCapacity_Customer)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        // Write Initial Inventory
        file << std::fixed << std::setprecision(1) << params.initialInventory_Plant << endl;

        for (const auto &d : params.initialInventory_Warehouse)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        for (const auto &d : params.initialInventory_Customer)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        // Write Unmet Demand Penalty
        for (const auto &d : params.unmetDemandPenalty)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        // Write Consume Rate
        for (const auto &d : params.consumeRate)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        // Write Demand
        for (const auto &d1 : params.demand)
        {
            for (const auto &d2 : d1)
            {
                for (const auto &element : d2)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }
        }

        // Write Probability
        for (const auto &d : params.probability)
        {
            file << std::fixed << std::setprecision(3) << d << " ";
        }
        file << endl;

        // Write Transportation Cost - First Echelon
        for (const auto &d1 : params.transportationCost_FirstEchelon)
        {
            for (const auto &element : d1)
            {
                file << std::fixed << std::setprecision(1) << element << " ";
            }
            file << endl;
        }

        // Write Transportation Cost - Second Echelon
        for (const auto &d1 : params.transportationCost_SecondEchelon)
        {
            for (const auto &element : d1)
            {
                file << std::fixed << std::setprecision(1) << element << " ";
            }
            file << endl;
        }

        // -----------------------------------------------
        // Write Solutions
        for (const auto &d1 : solSE.customerAssignmentToWarehouse)
        {
            for (const auto &d2 : d1)
            {
                for (const auto &d3 : d2)
                {
                    for (const auto &element : d3)
                    {
                        file << std::fixed << std::setprecision(0) << element << " ";
                    }
                    file << endl;
                }
            }
        }

        for (const auto &d : solFE.productionSetup)
        {
            file << std::fixed << std::setprecision(0) << d << " ";
        }
        file << endl;

        for (const auto &d : solFE.productionQuantity)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        for (const auto &d : solFE.plantInventory)
        {
            file << std::fixed << std::setprecision(1) << d << " ";
        }
        file << endl;

        for (const auto &d1 : solSE.warehouseInventory)
        {
            for (const auto &d2 : d1)
            {
                for (const auto &element : d2)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }
        }

        for (const auto &d1 : solSE.customerInventory)
        {
            for (const auto &d2 : d1)
            {
                for (const auto &element : d2)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }
        }

        for (const auto &d1 : solSE.customerUnmetDemand)
        {
            for (const auto &d2 : d1)
            {
                for (const auto &element : d2)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }
        }

        int t_index = 0;
        for (const auto &period : solFE.routesPlantToWarehouse)
        {
            int k_index = 0;
            for (const auto &vehicle : period)
            {
                if (!vehicle.empty())
                {
                    file << t_index << " " << k_index << " : ";
                    for (const auto &node : vehicle)
                    {
                        file << node << " ";
                    }
                    file << endl;
                }
                k_index++;
            }
            t_index++;
        }
        file << "endRoutesPlantToWarehouse" << endl;

        // for (const auto &d1 : solFE.routesPlantToWarehouse)
        // {
        //     for (const auto &d2 : d1)
        //     {
        //         for (const auto &element : d2)
        //         {
        //             file << std::fixed << std::setprecision(0) << element << " ";
        //         }
        //         file << endl;
        //     }
        // }

        for (const auto &d2 : solFE.deliveryQuantityToWarehouse)
        {
            for (const auto &element : d2)
            {
                file << std::fixed << std::setprecision(1) << element << " ";
            }
            file << endl;
        }

        int s_index = 0;
        for (const auto &scenario : solSE.routesWarehouseToCustomer)
        {
            int w_index = 0;
            for (const auto &warehouse : scenario)
            {
                int t_index = 0;
                for (const auto &period : warehouse)
                {
                    int k_index = 0;
                    for (const auto &vehicle : period)
                    {
                        if (!vehicle.empty())
                        {
                            file << s_index << " " << w_index << " " << t_index << " " << k_index << " : ";
                            for (const auto &node : vehicle)
                            { // Innermost level for 'i', actual route
                                file << node << " ";
                            }
                            file << std::endl; // End the line after each route
                        }
                        k_index++;
                    }
                    t_index++;
                }
                w_index++;
            }
            s_index++;
        }
        file << "endRoutesWarehouseToCustomer" << endl;

        for (const auto &d1 : solSE.deliveryQuantityToCustomer)
        {
            for (const auto &d2 : d1)
            {
                for (const auto &element : d2)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }
        }

        file.close();
    }
    else
    {
        cerr << "Failed to open the file for Saving Solution." << endl;
    }
}

bool SolutionManager::checkFeasibility()
{
    string resultsFileName;
    if (Algorithm == "Hybrid-ILS")
    {
        cout << "Checking feasibility for " << Algorithm << endl;
        resultsFileName = "../Results/Solutions/" + Algorithm + "/" + params.probabilityFunction.c_str() + "/S" + std::to_string(params.numScenarios) + "/Sol_S2EPRPAR_HHA_" + params.probabilityFunction.c_str() + "_" + params.instance.c_str() + "_S" + std::to_string(params.numScenarios)  + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
    }
    else if (Algorithm == "BC")
    {
        cout << "Checking feasibility for " << Algorithm << endl;
        resultsFileName = "../Results/Solutions/" + Algorithm + "/" + params.probabilityFunction.c_str() + "/S" + std::to_string(params.numScenarios) + "/Sol_S2EPRPAR_BC_" + params.probabilityFunction.c_str() + "_" + params.instance.c_str() + "_S" + std::to_string(params.numScenarios)  + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
    }

    cout << "Check the solution feasibility using Python" << endl;

    // Call the Python script with the file name as a command line argument
    string command = string("python ../program/src/feasCheck/feasibilityCheck.py ") + resultsFileName + " | tail -1";
    cout << command << endl;

    system(command.c_str());
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe)
    {
        cerr << "Error running command: " << command << endl;
        return 1;
    }

    // Read the output from the pipe
    vector<char> buffer(128);
    string result;
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
    {
        result += buffer.data();
    }

    // Close the pipe
    pclose(pipe);

    // Convert the result to an integer
    int feas_check = std::stoi(result);

    // Check if the output is 0
    if (feas_check == 0)
    {
        cout << "Feasibility Confirmed." << endl;
        return true;
    }
    else
    {
        cout << "The solution is infeasible" << endl;
        cout << "Violated Constraints:" << endl;

        // Run the Python script again to capture the violated constraints
        string constraints_command = string("python ...//Program/src/feasCheck/feasibilityCheck.py ") + resultsFileName;
        FILE *constraints_pipe = popen(constraints_command.c_str(), "r");
        if (!constraints_pipe)
        {
            cerr << "Error running command: " << constraints_command << endl;
            // return 1;
        }

        // Read and print the output (violated constraints)
        while (fgets(buffer.data(), buffer.size(), constraints_pipe) != nullptr)
        {
            cout << buffer.data();
        }

        // Close the pipe for constraints
        pclose(constraints_pipe);

        return false;
    }
}

void SolutionManager::saveResultSummary(const SolutionFirstEchelon &solFE, const SolutionSecondEchelon &solSE, const Result result)
{
    string summaryResultFileName;
    if (Algorithm == "Hybrid-ILS")
    {
        cout << "Saving Results Summary for " << Algorithm << endl;
        summaryResultFileName = "../Results/Summary/" + Algorithm + "/Summary_HHA_" + params.probabilityFunction.c_str() + ".csv";
    }
    else if (Algorithm == "BC")
    {
        cout << "Saving Results Summary for " << Algorithm << endl;
        summaryResultFileName = "../Results/Summary/" + Algorithm + "/Summary_BC_" + params.probabilityFunction.c_str() + ".csv";
    }

    // Save the summary of the results
    std::ofstream file_summary;
    file_summary.open(summaryResultFileName, std::ios::out | std::ios::app);

    if (file_summary.is_open())
    {
        // Write Parameters
        file_summary << params.instance.c_str() << ",";
        file_summary << params.numWarehouses << ",";
        file_summary << params.numCustomers << ",";
        file_summary << params.numPeriods << ",";
        file_summary << params.numScenarios << ",";
        file_summary << params.numVehicles_Plant << ",";
        file_summary << params.numVehicles_Warehouse << ",";
        file_summary << params.unmetDemandPenaltyCoeff << ",";
        file_summary << params.probabilityFunction << ",";
        file_summary << params.uncertaintyRange * 100 << "%,";

        file_summary << std::fixed << std::setprecision(1) << result.objValue_firstEchelon << ",";
        file_summary << std::fixed << std::setprecision(1) << result.objValue_secondEchelon << ",";
        file_summary << std::fixed << std::setprecision(1) << result.objValue_Total << ",";
        file_summary << std::fixed << std::setprecision(3) << result.totalCPUTime << ",";

        if (Algorithm == "BC")
        {
            file_summary << result.status << ",";
            file_summary << std::fixed << std::setprecision(1) << result.lowerBound << ",";
            file_summary << std::fixed << std::setprecision(2) << result.optimalityGap << "%,";
        }

        file_summary << std::fixed << std::setprecision(1) << solFE.setupCost << ",";
        file_summary << std::fixed << std::setprecision(1) << solFE.productionCost << ",";
        file_summary << std::fixed << std::setprecision(1) << solFE.holdingCostPlant << ",";
        file_summary << std::fixed << std::setprecision(1) << solFE.transportationCostPlantToWarehouse << ",";

        file_summary << std::fixed << std::setprecision(1) << solSE.holdingCostWarehouse_Avg << ",";
        file_summary << std::fixed << std::setprecision(1) << solSE.holdingCostCustomer_Avg << ",";
        file_summary << std::fixed << std::setprecision(1) << solSE.costOfUnmetDemand_Avg << ",";
        file_summary << std::fixed << std::setprecision(1) << solSE.transportationCostWarehouseToCustomer_Avg << endl;

        file_summary.close();
        cout << "results summary appended successfully." << endl;
    }
    else
    {
        cerr << "Failed to open the file." << endl;
    }
}

void SolutionManager::saveSolutionEV(const SolutionFirstEchelon &solFE, const SolutionSecondEchelon &solSE)
{
    try 
    {
        cout << "Saving solution for EV..." << endl;
        // Construct the directory path
        string directory = "../Results/Solutions/SolEvaluation/EV/" 
                                + params.probabilityFunction
                                + "/S" + std::to_string(params.numScenarios) 
                                + "/UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100))
                                + "/" + params.instance 
                                + "/" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff));

        // Create the directory if it doesn't exist
        if (!fs::exists(directory)) 
        {
            cout << "Directory does not exist. Creating: " << directory << endl;
            fs::create_directories(directory);
        }

        // Construct the filename
        string filename = "Sol_S2EPRPAR_EV_" + params.probabilityFunction 
                            + "_" + params.instance 
                            + "_S" + std::to_string(params.numScenarios)  
                            + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) 
                            + "%_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";

        // Full path to the file
        string fullPath = directory + "/" + filename;

        std::ofstream file(fullPath);
        if (file.is_open()) 
        {
            // Write Parameters
            file << params.numNodes_Total << " ";
            file << params.numWarehouses << " ";
            file << params.numCustomers << " ";
            file << params.numVehicles_Plant << " ";
            file << params.numVehicles_Warehouse << " ";
            file << params.numPeriods << " ";
            file << params.numScenarios << " " << endl;

            file << params.uncertaintyRange << " ";
            file << params.probabilityFunction << " ";
            file << params.unmetDemandPenaltyCoeff << " " << endl;

            file << std::fixed << std::setprecision(1) << params.unitProdCost << " ";
            file << std::fixed << std::setprecision(1) << params.setupCost << " ";
            file << std::fixed << std::setprecision(1) << params.prodCapacity << " ";
            file << std::fixed << std::setprecision(1) << params.vehicleCapacity_Plant << " ";
            file << std::fixed << std::setprecision(1) << params.vehicleCapacity_Warehouse << " " << endl;

            // Write Coordinate
            file << std::fixed << std::setprecision(1) << params.coordX_Plant << " ";
            file << std::fixed << std::setprecision(1) << params.coordY_Plant << " " << endl;

            for (const auto &d : params.coordX_Warehouse)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;
            for (const auto &d : params.coordY_Warehouse)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            for (const auto &d : params.coordX_Customer)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            for (const auto &d : params.coordY_Customer)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            // Write Holding Cost
            file << params.unitHoldingCost_Plant << endl;

            for (const auto &d : params.unitHoldingCost_Warehouse)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            for (const auto &d : params.unitHoldingCost_Customer)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            // Write Storage Capacity
            file << params.storageCapacity_Plant << endl;

            for (const auto &d : params.storageCapacity_Warehouse)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            for (const auto &d : params.storageCapacity_Customer)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            // Write Initial Inventory
            file << std::fixed << std::setprecision(1) << params.initialInventory_Plant << endl;

            for (const auto &d : params.initialInventory_Warehouse)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            for (const auto &d : params.initialInventory_Customer)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            // Write Unmet Demand Penalty
            for (const auto &d : params.unmetDemandPenalty)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            // Write Consume Rate
            for (const auto &d : params.consumeRate)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            // Write Transportation Cost - First Echelon
            for (const auto &d1 : params.transportationCost_FirstEchelon)
            {
                for (const auto &element : d1)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }

            // Write Transportation Cost - Second Echelon
            for (const auto &d1 : params.transportationCost_SecondEchelon)
            {
                for (const auto &element : d1)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }

            // -------------------------------------------------------------------
            // Write Solutions
            for (const auto &d1 : solSE.customerAssignmentToWarehouse)
            {
                for (const auto &d2 : d1)
                {
                    for (const auto &d3 : d2)
                    {
                        for (const auto &element : d3)
                        {
                            file << std::fixed << std::setprecision(0) << element << " ";
                        }
                        file << endl;
                    }
                }
            }

            for (const auto &d : solFE.productionSetup)
            {
                file << std::fixed << std::setprecision(0) << d << " ";
            }
            file << endl;

            for (const auto &d : solFE.productionQuantity)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            for (const auto &d : solFE.plantInventory)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            for (const auto &d1 : solSE.warehouseInventory)
            {
                for (const auto &d2 : d1)
                {
                    for (const auto &element : d2)
                    {
                        file << std::fixed << std::setprecision(1) << element << " ";
                    }
                    file << endl;
                }
            }

            for (const auto &d1 : solSE.customerInventory)
            {
                for (const auto &d2 : d1)
                {
                    for (const auto &element : d2)
                    {
                        file << std::fixed << std::setprecision(1) << element << " ";
                    }
                    file << endl;
                }
            }

            for (const auto &d1 : solSE.customerUnmetDemand)
            {
                for (const auto &d2 : d1)
                {
                    for (const auto &element : d2)
                    {
                        file << std::fixed << std::setprecision(1) << element << " ";
                    }
                    file << endl;
                }
            }

            int t_index = 0;
            for (const auto &period : solFE.routesPlantToWarehouse)
            {
                int k_index = 0;
                for (const auto &vehicle : period)
                {
                    if (!vehicle.empty())
                    {
                        file << t_index << " " << k_index << " : ";
                        for (const auto &node : vehicle)
                        {
                            file << node << " ";
                        }
                        file << endl;
                    }
                    k_index++;
                }
                t_index++;
            }
            file << "endRoutesPlantToWarehouse" << endl;

            for (const auto &d2 : solFE.deliveryQuantityToWarehouse)
            {
                for (const auto &element : d2)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }

            int s_index = 0;
            for (const auto &scenario : solSE.routesWarehouseToCustomer)
            {
                int w_index = 0;
                for (const auto &warehouse : scenario)
                {
                    int t_index = 0;
                    for (const auto &period : warehouse)
                    {
                        int k_index = 0;
                        for (const auto &vehicle : period)
                        {
                            if (!vehicle.empty())
                            {
                                file << s_index << " " << w_index << " " << t_index << " " << k_index << " : ";
                                for (const auto &node : vehicle)
                                { // Innermost level for 'i', actual route
                                    file << node << " ";
                                }
                                file << std::endl; // End the line after each route
                            }
                            k_index++;
                        }
                        t_index++;
                    }
                    w_index++;
                }
                s_index++;
            }
            file << "endRoutesWarehouseToCustomer" << endl;

            for (const auto &d1 : solSE.deliveryQuantityToCustomer)
            {
                for (const auto &d2 : d1)
                {
                    for (const auto &element : d2)
                    {
                        file << std::fixed << std::setprecision(1) << element << " ";
                    }
                    file << endl;
                }
            }

            file.close();
        }
        else 
        {
            std::cerr << "Unable to open file: " << fullPath << std::endl;
        }
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

bool SolutionManager::checkFeasibilityEV()
{
    // Construct the directory path
    string directory = "../Results/Solutions/SolEvaluation/EV/" 
                            + params.probabilityFunction
                            + "/S" + std::to_string(params.numScenarios) 
                            + "/UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100))
                            + "/" + params.instance 
                            + "/" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff));

    // Construct the filename
    string filename = "Sol_S2EPRPAR_EV_" + params.probabilityFunction 
                        + "_" + params.instance 
                        + "_S" + std::to_string(params.numScenarios)  
                        + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) 
                        + "%_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";

    // Full path to the file
    string fullPath = directory + "/" + filename;

    cout << "Checking Solution Feasibility For EV..." << endl;

    // Call the Python script with the file name as a command line argument
    string command = string("python ../program/src/feasCheck/feasibilityCheckEV.py ") + fullPath + " | tail -1";
    cout << command << endl;

    system(command.c_str());
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe)
    {
        cerr << "Error running command: " << command << endl;
        return 1;
    }

    // Read the output from the pipe
    vector<char> buffer(128);
    string result;
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
    {
        result += buffer.data();
    }

    // Close the pipe
    pclose(pipe);

    // Convert the result to an integer
    int feas_check = std::stoi(result);

    // Check if the output is 0
    if (feas_check == 0)
    {
        cout << "Feasibility Confirmed." << endl;
        return true;
    }
    else
    {
        cout << "The solution is infeasible" << endl;
        cout << "Violated Constraints:" << endl;

        // Run the Python script again to capture the violated constraints
        string constraints_command = string("python ...//Program/src/feasCheck/feasibilityCheckEV.py ") + fullPath;
        FILE *constraints_pipe = popen(constraints_command.c_str(), "r");
        if (!constraints_pipe)
        {
            cerr << "Error running command: " << constraints_command << endl;
            // return 1;
        }

        // Read and print the output (violated constraints)
        while (fgets(buffer.data(), buffer.size(), constraints_pipe) != nullptr)
        {
            cout << buffer.data();
        }

        // Close the pipe for constraints
        pclose(constraints_pipe);

        return false;
    }
}
