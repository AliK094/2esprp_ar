#include "SolutionManager.h"

SolutionManager::SolutionManager(const ParameterSetting &parameters)
    : params(parameters)
{
}

void SolutionManager::saveSolution(const SolutionFirstEchelon &solFE, const SolutionSecondEchelon &solSE)
{
    string directory;
    string filename;
    if (params.problemType == "S2EPRP-AR" && (params.solutionAlgorithm == "Hybrid-ILS" || params.solutionAlgorithm == "BC"))
    {
        cout << "Saving solution for " << params.problemType << " with " << params.solutionAlgorithm << "..." << endl;
        // Construct the directory path
        directory = "../Results/Solutions/" + params.problemType + "/" + params.solutionAlgorithm + "/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios);

        // Construct the filename
        filename = "Sol_" + params.problemType + "_" + params.solutionAlgorithm + "_" + params.probabilityFunction + "_" + params.instance + "_S" + std::to_string(params.numScenarios) + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" + "_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
    }
    else
    {
        cout << "Invalid params.solutionAlgorithm" << endl;
        return;
    }

    // Create the directory if it doesn't exist
    if (!fs::exists(directory))
    {
        cout << "Directory does not exist. Creating: " << directory << endl;
        fs::create_directories(directory);
    }

    // Full path to the file
    string fullPath = directory + "/" + filename;

    // Write the data to a file
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
    string directory;
    string filename;
    if (params.problemType == "S2EPRP-AR" && (params.solutionAlgorithm == "Hybrid-ILS" || params.solutionAlgorithm == "BC"))
    {
        cout << "Check Feasibility for " << params.problemType << " and solutionAlgorithm: " << params.solutionAlgorithm << "..." << endl;
        // Construct the directory path
        directory = "../Results/Solutions/" + params.problemType + "/" + params.solutionAlgorithm + "/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios);

        // Construct the filename
        filename = "Sol_" + params.problemType + "_" + params.solutionAlgorithm + "_" + params.probabilityFunction + "_" + params.instance + "_S" + std::to_string(params.numScenarios) + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" + "_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
    }
    else
    {
        cout << "Invalid params.solutionAlgorithm" << endl;
        return false;
    }
    // Full path to the file
    string fullPath = directory + "/" + filename;

    cout << "Check the solution feasibility using Python" << endl;

    // Call the Python script with the file name as a command line argument
    string command = string("python ../program/src/feasCheck/feasibilityCheck.py ") + fullPath + " | tail -1";
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
        string constraints_command = string("python ...//Program/src/feasCheck/feasibilityCheck.py ") + fullPath;
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
    cout << "Save Solution Summary for " << params.problemType << " and solutionAlgorithm: " << params.solutionAlgorithm << endl;
    if (params.problemType != "S2EPRP-AR")
    {
        cerr << "Invalid problem type." << endl;
        return;
    }

    string directory;
    string filename;
    if (params.solutionAlgorithm == "Hybrid-ILS" || params.solutionAlgorithm == "BC")
    {
        // Construct the directory path
        directory = "../Results/Summary/" + params.problemType + "/" + params.solutionAlgorithm;
        filename = "Sol_" + params.problemType + "_" + params.solutionAlgorithm + "_" + params.probabilityFunction + ".csv";
    }
    else
    {
        cerr << "Invalid params.solutionAlgorithm" << endl;
        return;
    }

    // Create the directory if it doesn't exist
    if (!fs::exists(directory))
    {
        cout << "Directory does not exist. Creating: " << directory << endl;
        fs::create_directories(directory);
    }

    // Full path to the file
    string fullPath = directory + "/" + filename;

    // Check if the file exists
    std::ifstream fileCheck(fullPath);
    bool fileExists = fileCheck.is_open();
    fileCheck.close();
    if (!fileExists)
    {
        std::ofstream outFile(fullPath);
        if (outFile.is_open())
        {
            if (params.solutionAlgorithm == "Hybrid-ILS")
            {
                outFile << "Instance,NumWarehouses,NumCustomers,NumPeriods,NumScenarios,NumVehicles_Plant,NumVehicles_Warehouse,"
                           "UnmetDemandPenaltyCoeff,ProbabilityFunction,UncertaintyRange,ObjectiveValue_FirstEchelon,"
                           "ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,"
                           "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                           "Avg_HoldingCostWarehouse,Avg_HoldingCostCustomer,Avg_ShortageCost,"
                           "Avg_TransportationCostWarehouseToCustomer\n";
                outFile.close();
            }
            else if (params.solutionAlgorithm == "BC")
            {
                outFile << "Instance,NumWarehouses,NumCustomers,NumPeriods,NumScenarios,NumVehicles_Plant,NumVehicles_Warehouse,"
                           "UnmetDemandPenaltyCoeff,ProbabilityFunction,UncertaintyRange,ObjectiveValue_FirstEchelon,"
                           "ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,Status,LowerBound,OptimalityGap,"
                           "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                           "Avg_HoldingCostWarehouse,Avg_HoldingCostCustomer,Avg_ShortageCost,"
                           "Avg_TransportationCostWarehouseToCustomer\n";
                outFile.close();
            }
            cout << "File created: " << fullPath << endl;
        }
        else
        {
            cerr << "Failed to create the file." << endl;
        }
    }

    // Save the summary of the results
    std::ofstream file_summary;
    file_summary.open(fullPath, std::ios::out | std::ios::app);

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

        if (params.solutionAlgorithm == "BC")
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

void SolutionManager::saveSolution_Deterministic(const SolutionFirstEchelon &solFE, const SolutionSecondEchelon_Deterministic &solSE, string solAlg)
{
    try
    {
        string directory;
        string filename;

        if (params.problemType == "S2EPRP-AR")
        {
            cerr << "Invalid problem type." << endl;
            return;
        }

        cout << "Saving solution for " << params.problemType << " and Solution Algorithm " << solAlg << "..." << endl;
        if (params.problemType == "EV")
        {
            // Construct the directory path
            directory = "../Results/Solutions/S2EPRP-AR/SolEvaluation/" + params.problemType + "/" + solAlg + "/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios);

            // Construct the filename
            filename = "Sol_S2EPRP-AR_" + params.problemType + "_" + solAlg + "_" + params.probabilityFunction + "_" + params.instance + "_S" + std::to_string(params.numScenarios) + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" + "_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
        }
        else if (params.problemType == "WS" || params.problemType == "EEV")
        {
            if (params.scenarioIndex == -1)
            {
                cout << "params.scenarioIndex is not valid." << endl;
                exit(1);
            }

            // Construct the directory path
            directory = "../Results/Solutions/S2EPRP-AR/SolEvaluation/" + params.problemType + "/" + solAlg + "/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios) + "/UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" + "/PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + "/" + params.instance;

            // Construct the filename
            filename = "Sol_S2EPRP-AR_" + params.problemType + "_" + solAlg + "_" + params.probabilityFunction + "_" + params.instance + "_s" + std::to_string(params.scenarioIndex) + ".txt";
        }
        else if (params.problemType == "2EPRP" || params.problemType == "2EPRPCS")
        {
            // Construct the directory path
            directory = "../Results/Solutions/" + params.problemType + "/" + solAlg;

            // Construct the filename
            filename = "Sol_" + params.problemType + "_" + solAlg + "_" + params.instance + ".txt";
        }
        else
        {
            cout << "Invalid Problem Type" << endl;
            exit(1);
        }

        // Create the directory if it doesn't exist
        if (!fs::exists(directory))
        {
            cout << "Directory does not exist. Creating: " << directory << endl;
            fs::create_directories(directory);
        }

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

            if (params.problemType != "2EPRP" && params.problemType != "2EPRPCS")
            {
                file << params.numScenarios << " ";
            }

            if (params.problemType == "WS" || params.problemType == "EEV")
            {
                file << params.scenarioIndex << " ";
            }

            if (params.problemType != "2EPRP" && params.problemType != "2EPRPCS")
            {
                file << params.uncertaintyRange << " ";
                file << params.probabilityFunction << " ";
                file << params.unmetDemandPenaltyCoeff << " " << endl;
            }

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

            if (params.problemType == "2EPRPCS")
            {
                for (const auto &d : params.unitHandlingCost_Satellite)
                {
                    file << std::fixed << std::setprecision(1) << d << " ";
                }
                file << endl;
            }
            else
            {
                for (const auto &d : params.unitHoldingCost_Warehouse)
                {
                    file << std::fixed << std::setprecision(1) << d << " ";
                }
                file << endl;
            }

            for (const auto &d : params.unitHoldingCost_Customer)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            // Write Storage Capacity
            file << params.storageCapacity_Plant << endl;

            if (params.problemType != "2EPRPCS")
            {
                for (const auto &d : params.storageCapacity_Warehouse)
                {
                    file << std::fixed << std::setprecision(1) << d << " ";
                }
                file << endl;
            }

            for (const auto &d : params.storageCapacity_Customer)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            // Write Initial Inventory
            file << std::fixed << std::setprecision(1) << params.initialInventory_Plant << endl;

            if (params.problemType != "2EPRPCS")
            {
                for (const auto &d : params.initialInventory_Warehouse)
                {
                    file << std::fixed << std::setprecision(1) << d << " ";
                }
                file << endl;
            }

            for (const auto &d : params.initialInventory_Customer)
            {
                file << std::fixed << std::setprecision(1) << d << " ";
            }
            file << endl;

            if (params.problemType != "2EPRP" && params.problemType != "2EPRPCS")
            {
                // Write Unmet Demand Penalty
                for (const auto &d : params.unmetDemandPenalty)
                {
                    file << std::fixed << std::setprecision(1) << d << " ";
                }
                file << endl;
            }

            // Write Deterministic Demand
            for (const auto &d1 : params.demand_Deterministic)
            {
                for (const auto &d2 : d1)
                {
                    file << std::fixed << std::setprecision(1) << d2 << " ";
                }
                file << endl;
            }

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
                    for (const auto &element : d2)
                    {
                        file << std::fixed << std::setprecision(0) << element << " ";
                    }
                    file << endl;
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

            if (params.problemType != "2EPRPCS")
            {
                for (const auto &d1 : solSE.warehouseInventory)
                {
                    for (const auto &element : d1)
                    {
                        file << std::fixed << std::setprecision(1) << element << " ";
                    }
                    file << endl;
                }
            }

            for (const auto &d1 : solSE.customerInventory)
            {
                for (const auto &element : d1)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }

            if (params.problemType != "2EPRP" && params.problemType != "2EPRPCS")
            {
                for (const auto &d1 : solSE.customerUnmetDemand)
                {
                    for (const auto &element : d1)
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

            int w_index = 0;
            for (const auto &warehouse : solSE.routesWarehouseToCustomer)
            {
                int t_index = 0;
                for (const auto &period : warehouse)
                {
                    int k_index = 0;
                    for (const auto &vehicle : period)
                    {
                        if (!vehicle.empty())
                        {
                            file << w_index << " " << t_index << " " << k_index << " : ";
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
            file << "endRoutesWarehouseToCustomer" << endl;

            for (const auto &d1 : solSE.deliveryQuantityToCustomer)
            {
                for (const auto &element : d1)
                {
                    file << std::fixed << std::setprecision(1) << element << " ";
                }
                file << endl;
            }

            file.close();
        }
        else
        {
            std::cerr << "Unable to open file: " << fullPath << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

bool SolutionManager::checkFeasibility_Deterministic(string solAlg)
{
    string directory;
    string filename;

    cout << "Checking feasibility for " << params.problemType << " and Solution Algorithm " << solAlg << "..." << endl;
    if (params.problemType == "EV")
    {
        directory = "../Results/Solutions/S2EPRP-AR/SolEvaluation/" + params.problemType + "/" + solAlg + "/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios);

        // Construct the filename
        filename = "Sol_S2EPRP-AR_" + params.problemType + "_" + solAlg + "_" + params.probabilityFunction + "_" + params.instance + "_S" + std::to_string(params.numScenarios) + "_UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" + "_PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + ".txt";
    }
    else if (params.problemType == "WS" || params.problemType == "EEV")
    {
        directory = "../Results/Solutions/S2EPRP-AR/SolEvaluation/" + params.problemType + "/" + solAlg + "/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios) + "/UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" + "/PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff)) + "/" + params.instance;

        // Construct the filename
        filename = "Sol_S2EPRP-AR_" + params.problemType + "_" + solAlg + "_" + params.probabilityFunction + "_" + params.instance + "_s" + std::to_string(params.scenarioIndex) + ".txt";
    }
    else if (params.problemType == "2EPRP" || params.problemType == "2EPRPCS")
    {
        directory = "../Results/Solutions/" + params.problemType + "/" + solAlg;

        // Construct the filename
        filename = "Sol_" + params.problemType + "_" + solAlg + "_" + params.instance + ".txt";
    }
    else
    {
        cout << "Invalid Problem Type" << endl;
        return false;
    }

    // Full path to the file
    string fullPath = directory + "/" + filename;

    // Call the Python script with the file name as a command line argument
    string command = string("python ../program/src/feasCheck/feasibilityCheck_deterministic.py ") + fullPath + " " + params.problemType + " | tail -1";
    cout << command << endl;

    system(command.c_str());
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe)
    {
        cerr << "Error running command: " << command << endl;
        return false;
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
        string constraints_command = string("python ...//Program/src/feasCheck/feasibilityCheck_deterministic.py ") + fullPath + " " + params.problemType;
        FILE *constraints_pipe = popen(constraints_command.c_str(), "r");
        if (!constraints_pipe)
        {
            cerr << "Error running command: " << constraints_command << endl;
            return false;
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

void SolutionManager::saveResultSummary_Deterministic(const SolutionFirstEchelon &solFE,
                                                      const SolutionSecondEchelon_Deterministic &solSE,
                                                      const Result result,
                                                      string solAlg)
{
    string directory;
    string filename;
    cout << "Save Solution Summary for " << params.problemType << " and Solution Algorithm " << solAlg << "..." << endl;
    if (params.problemType == "EV")
    {
        // Construct the directory path
        directory = "../Results/Summary/S2EPRP-AR/SolEvaluation/" + params.problemType + "/" + solAlg + "/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios) + "/UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" + "/" + params.instance + "/PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff));
        filename = "Sol_" + params.problemType + "_" + solAlg + "_" + params.instance + ".csv";
    }
    else if (params.problemType == "WS" || params.problemType == "EEV")
    {
        if (params.scenarioIndex == -1)
        {
            cout << "params.scenarioIndex is not valid." << endl;
            exit(1);
        }

        directory = "../Results/Summary/S2EPRP-AR/SolEvaluation/" + params.problemType + "/" + solAlg + "/" + params.probabilityFunction + "/S" + std::to_string(params.numScenarios) + "/UR" + std::to_string(static_cast<int>(params.uncertaintyRange * 100)) + "%" + "/" + params.instance + "/PC" + std::to_string(static_cast<int>(params.unmetDemandPenaltyCoeff));
        filename = "Sol_" + params.problemType + "_" + solAlg + "_" + params.instance + "_S" + std::to_string(params.numScenarios) + ".csv";
    }
    else if (params.problemType == "2EPRP" || params.problemType == "2EPRPCS")
    {
        directory = "../Results/Summary/" + params.problemType + "/" + solAlg;
        filename = "Sol_" + params.problemType + "_" + solAlg + ".csv";
    }
    else
    {
        cout << "Invalid Problem Type" << endl;
        exit(1);
    }

    // Create the directory if it doesn't exist
    if (!fs::exists(directory))
    {
        cout << "Directory does not exist. Creating: " << directory << endl;
        fs::create_directories(directory);
    }

    // Full path to the file
    string fullPath = directory + "/" + filename;

    // Check if the file exists
    std::ifstream fileCheck(fullPath);
    bool fileExists = fileCheck.is_open();
    fileCheck.close();

    // If the file does not exist, create it and write the header
    if (!fileExists)
    {
        std::ofstream outFile(fullPath);
        if (outFile.is_open())
        {
            if (solAlg == "Hybrid-ILS")
            {
                if (params.problemType == "EV")
                {
                    outFile << "Instance,NumWarehouses,NumCustomers,NumPeriods,NumScenarios,NumVehicles_Plant,NumVehicles_Warehouse,"
                            "UnmetDemandPenaltyCoeff,ProbabilityFunction,UncertaintyRange,ObjectiveValue_FirstEchelon,"
                            "ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,"
                            "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                            "HoldingCostWarehouse,HoldingCostCustomer,CostOfUnmetDemand,"
                            "TransportationCostWarehouseToCustomer\n";
                    outFile.close();
                }
                if (params.problemType == "EEV" || params.problemType == "WS")
                {
                    cout << "params.scenarioIndex: " << params.scenarioIndex + 1 << endl;
                    outFile << "Instance,scenarioIndex,NumWarehouses,NumCustomers,NumPeriods,NumScenarios,NumVehicles_Plant,NumVehicles_Warehouse,"
                            "UnmetDemandPenaltyCoeff,ProbabilityFunction,UncertaintyRange,ObjectiveValue_FirstEchelon,"
                            "ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,"
                            "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                            "HoldingCostWarehouse,HoldingCostCustomer,CostOfUnmetDemand,"
                            "TransportationCostWarehouseToCustomer\n";
                    outFile.close();
                }
                else if (params.problemType == "2EPRP")
                {
                    outFile << "Instance,NumWarehouses,NumCustomers,NumPeriods,NumVehicles_Plant,NumVehicles_Warehouse,"
                            "ObjectiveValue_FirstEchelon,ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,"
                            "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                            "HoldingCostWarehouse,HoldingCostCustomer,TransportationCostWarehouseToCustomer\n";
                    outFile.close();
                }
                else if (params.problemType == "2EPRPCS")
                {
                    outFile << "Instance,NumWarehouses,NumCustomers,NumPeriods,NumVehicles_Plant,NumVehicles_Warehouse,"
                            "ObjectiveValue_FirstEchelon,ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,"
                            "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                            "HandlingCostSatelites,HoldingCostCustomer,TransportationCostWarehouseToCustomer\n";
                    outFile.close();
                }
            }
            else if (solAlg == "BC")
            {
                if (params.problemType == "EV")
                {
                    outFile << "Instance,NumWarehouses,NumCustomers,NumPeriods,NumScenarios,NumVehicles_Plant,NumVehicles_Warehouse,"
                            "UnmetDemandPenaltyCoeff,ProbabilityFunction,UncertaintyRange,ObjectiveValue_FirstEchelon,"
                            "ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,Status,LowerBound,OptimalityGap,"
                            "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                            "HoldingCostWarehouse,HoldingCostCustomer,CostOfUnmetDemand,"
                            "TransportationCostWarehouseToCustomer\n";
                    outFile.close();
                }
                if (params.problemType == "EEV" || params.problemType == "WS")
                {
                    cout << "params.scenarioIndex: " << params.scenarioIndex + 1 << endl;
                    outFile << "Instance,scenarioIndex,NumWarehouses,NumCustomers,NumPeriods,NumScenarios,NumVehicles_Plant,NumVehicles_Warehouse,"
                            "UnmetDemandPenaltyCoeff,ProbabilityFunction,UncertaintyRange,ObjectiveValue_FirstEchelon,"
                            "ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,Status,LowerBound,OptimalityGap,"
                            "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                            "HoldingCostWarehouse,HoldingCostCustomer,CostOfUnmetDemand,"
                            "TransportationCostWarehouseToCustomer\n";
                    outFile.close();
                }
                else if (params.problemType == "2EPRP")
                {
                    outFile << "Instance,NumWarehouses,NumCustomers,NumPeriods,NumVehicles_Plant,NumVehicles_Warehouse,"
                            "ObjectiveValue_FirstEchelon,ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,Status,LowerBound,OptimalityGap,"
                            "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                            "HoldingCostWarehouse,HoldingCostCustomer,TransportationCostWarehouseToCustomer\n";
                    outFile.close();
                }
                else if (params.problemType == "2EPRPCS")
                {
                    outFile << "Instance,NumWarehouses,NumCustomers,NumPeriods,NumVehicles_Plant,NumVehicles_Warehouse,"
                            "ObjectiveValue_FirstEchelon,ObjectiveValue_SecondEchelon,ObjectiveValue_Total,TotalCPUTime,Status,LowerBound,OptimalityGap,"
                            "SetupCost,ProductionCost,HoldingCostPlant,TransportationCostPlantToWarehouse,"
                            "HandlingCostSatelites,HoldingCostCustomer,TransportationCostWarehouseToCustomer\n";
                    outFile.close();
                }
            }
            else 
            {
                cout << "Invalid params.solutionAlgorithm" << endl;
                exit(1);
            }
            std::cout << "File created and header added: " << fullPath << std::endl;
        }
        else
        {
            std::cerr << "Error creating file: " << fullPath << std::endl;
        }
    }
    else
    {
        std::cout << "File already exists: " << fullPath << std::endl;
    }

    std::ofstream file_summary(fullPath);
    if (file_summary.is_open())
    {
        // Write Parameters
        file_summary << params.instance.c_str() << ",";

        if (params.problemType == "EEV" || params.problemType == "WS")
        {
            file_summary << params.scenarioIndex + 1 << ",";
        }
        file_summary << params.numWarehouses << ",";
        file_summary << params.numCustomers << ",";
        file_summary << params.numPeriods << ",";
        if (params.problemType == "EV" || params.problemType == "EEV" || params.problemType == "WS")
        {
            file_summary << params.numScenarios << ",";
        }
        file_summary << params.numVehicles_Plant << ",";
        file_summary << params.numVehicles_Warehouse << ",";
        if (params.problemType == "EV" || params.problemType == "EEV" || params.problemType == "WS")
        {
            file_summary << params.unmetDemandPenaltyCoeff << ",";
            file_summary << params.probabilityFunction << ",";
            file_summary << params.uncertaintyRange * 100 << "%,";
        }

        file_summary << std::fixed << std::setprecision(1) << result.objValue_firstEchelon << ",";
        file_summary << std::fixed << std::setprecision(1) << result.objValue_secondEchelon << ",";
        file_summary << std::fixed << std::setprecision(1) << result.objValue_Total << ",";
        file_summary << std::fixed << std::setprecision(3) << result.totalCPUTime << ",";

        if (solAlg == "BC")
        {
            file_summary << result.status << ",";
            file_summary << std::fixed << std::setprecision(1) << result.lowerBound << ",";
            file_summary << std::fixed << std::setprecision(2) << result.optimalityGap << "%,";
        }

        file_summary << std::fixed << std::setprecision(1) << solFE.setupCost << ",";
        file_summary << std::fixed << std::setprecision(1) << solFE.productionCost << ",";
        file_summary << std::fixed << std::setprecision(1) << solFE.holdingCostPlant << ",";
        file_summary << std::fixed << std::setprecision(1) << solFE.transportationCostPlantToWarehouse << ",";

        if (params.problemType == "2EPRPCS")
        {
            file_summary << std::fixed << std::setprecision(1) << solSE.handlingCostSatellite << ",";
        }
        else
        {
            file_summary << std::fixed << std::setprecision(1) << solSE.holdingCostWarehouse << ",";
        }

        file_summary << std::fixed << std::setprecision(1) << solSE.holdingCostCustomer << ",";
        if (params.problemType != "2EPRP" && params.problemType != "2EPRPCS")
        {
            file_summary << std::fixed << std::setprecision(1) << solSE.costOfUnmetDemand << ",";
        }
        file_summary << std::fixed << std::setprecision(1) << solSE.transportationCostWarehouseToCustomer << endl;

        file_summary.close();
        cout << "results summary appended successfully." << endl;
    }
    else
    {
        cerr << "Failed to open the file." << endl;
    }
}