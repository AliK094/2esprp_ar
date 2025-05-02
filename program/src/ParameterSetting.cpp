#include "ParameterSetting.h"

ParameterSetting::ParameterSetting(int argc, char *argv[])
{
    problemType = argv[1];
    // Argument parsing with error handling
    try
    {
        solutionAlgorithm = argv[2];
        inputFile = argv[3];
        numWarehouses = std::stoi(argv[4]);
        numCustomers = std::stoi(argv[5]);
        numPeriods = std::stoi(argv[6]);
        numVehicles_Plant = std::stoi(argv[7]);
        numVehicles_Warehouse = std::stoi(argv[8]);
        if (problemType == "S2EPRP-AR" || problemType == "EV")
        {
            numScenarios = std::stoi(argv[9]);
            unmetDemandPenaltyCoeff = std::stod(argv[10]);
            uncertaintyRange = std::stod(argv[11]);
            probabilityFunction = argv[12];
            instance = argv[13];
        }
        else if (problemType == "EEV" || problemType == "WS")
        {
            numScenarios = std::stoi(argv[9]);
            unmetDemandPenaltyCoeff = std::stod(argv[10]);
            uncertaintyRange = std::stod(argv[11]);
            probabilityFunction = argv[12];
            instance = argv[13];
            scenarioIndex = std::stoi(argv[14]);
        }
        else if (problemType == "2EPRP" || problemType == "2EPRPCS")
        {
            instance = argv[9];
        }
    }
    catch (const std::invalid_argument &e)
    {
        std::cerr << "Error parsing arguments: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }
    catch (const std::out_of_range &e)
    {
        std::cerr << "Argument out of range: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    // Initialize other parameters
    numNodes_Total = numWarehouses + numCustomers + 1;
    numNodes_FirstEchelon = numWarehouses + 1;
    numEdges_FirstEchelon = (numNodes_FirstEchelon * (numNodes_FirstEchelon - 1)) / 2;
    numNodes_SecondEchelon = numCustomers + numWarehouses;
    numEdges_SecondEchelon = (numNodes_SecondEchelon * (numNodes_SecondEchelon - 1)) / 2;
    numVehicles_SecondEchelon = numWarehouses * numVehicles_Warehouse;

    // Reserve space and initialize containers with appropriate sizes
    coordX_Warehouse.resize(numWarehouses, 0);
    coordY_Warehouse.resize(numWarehouses, 0);
    coordX_Customer.resize(numCustomers, 0);
    coordY_Customer.resize(numCustomers, 0);

    if (problemType == "2EPRPCS")
    {
        unitHandlingCost_Satellite.resize(numWarehouses, 0);
    }

    unitHoldingCost_Warehouse.resize(numWarehouses, 0);
    unitHoldingCost_Customer.resize(numCustomers, 0);
    storageCapacity_Warehouse.resize(numWarehouses, 0);
    storageCapacity_Customer.resize(numCustomers, 0);
    initialInventory_Warehouse.resize(numWarehouses, 0);
    initialInventory_Customer.resize(numCustomers, 0);

    consumeRate.assign(numCustomers, 0);
    if (problemType == "S2EPRP-AR")
    {
        probability.assign(numScenarios, 0.0);
        unmetDemandPenalty.assign(numCustomers, std::numeric_limits<double>::infinity());
        demand.assign(numCustomers, vector<vector<double>>(numPeriods, vector<double>(numScenarios, 0.0)));
        DeliveryUB_perCustomer.assign(numCustomers, vector<vector<double>>(numPeriods, vector<double>(numScenarios, std::numeric_limits<double>::infinity())));
        DeliveryUB.assign(numPeriods, vector<double>(numScenarios, std::numeric_limits<double>::infinity()));
    }
    else if (problemType == "EV" || problemType == "EEV" || problemType == "WS")
    {
        probability.assign(numScenarios, 0.0);
        unmetDemandPenalty.assign(numCustomers, std::numeric_limits<double>::infinity());
        demand.assign(numCustomers, vector<vector<double>>(numPeriods, vector<double>(numScenarios, 0.0)));
        DeliveryUB_perCustomer.assign(numCustomers, vector<vector<double>>(numPeriods, vector<double>(numScenarios, std::numeric_limits<double>::infinity())));
        DeliveryUB.assign(numPeriods, vector<double>(numScenarios, std::numeric_limits<double>::infinity()));

        demand_Deterministic.assign(numCustomers, vector<double>(numPeriods, 0.));
        DelUB_perCus_Det.assign(numCustomers, vector<double>(numPeriods, std::numeric_limits<double>::infinity()));
        DelUB_Det.assign(numPeriods, std::numeric_limits<double>::infinity());
    }
    else
    {
        demand_Deterministic.assign(numCustomers, vector<double>(numPeriods, 0.));
        DelUB_perCus_Det.assign(numCustomers, vector<double>(numPeriods, std::numeric_limits<double>::infinity()));
        DelUB_Det.assign(numPeriods, std::numeric_limits<double>::infinity());

        unmetDemandPenalty.assign(numCustomers, std::numeric_limits<double>::infinity());
    }

    initializeIndices();
    setHyperparameters();
}

void ParameterSetting::initializeIndices()
{
    // Initialize first echelon indices and costs
    index_i_FirstEchelon.resize(numEdges_FirstEchelon, 0);
    index_j_FirstEchelon.resize(numEdges_FirstEchelon, 0);
    index_e_FirstEchelon.assign(numNodes_FirstEchelon, vector<int>(numNodes_FirstEchelon, 0));
    transportationCost_FirstEchelon.assign(numNodes_FirstEchelon, vector<double>(numNodes_FirstEchelon, 0));

    int e = 0;
    for (int i = 0; i < numNodes_FirstEchelon - 1; ++i)
    {
        for (int j = i + 1; j < numNodes_FirstEchelon; ++j)
        {
            index_i_FirstEchelon[e] = i;
            index_j_FirstEchelon[e] = j;
            index_e_FirstEchelon[i][j] = index_e_FirstEchelon[j][i] = e;
            e++;
        }
    }
    assert(e == numEdges_FirstEchelon);

    for (int i = 0; i < numNodes_FirstEchelon; ++i)
    {
        index_e_FirstEchelon[i][i] = NONE;
    }
    // -----------------------------------------------------------------
    // Initialize second echelon indices and costs
    index_i_SecondEchelon.resize(numEdges_SecondEchelon, 0);
    index_j_SecondEchelon.resize(numEdges_SecondEchelon, 0);
    index_e_SecondEchelon.assign(numNodes_SecondEchelon, vector<int>(numNodes_SecondEchelon, 0));
    transportationCost_SecondEchelon.assign(numNodes_SecondEchelon, vector<double>(numNodes_SecondEchelon, 0));

    e = 0;
    for (int i = 0; i < numNodes_SecondEchelon - 1; ++i)
    {
        for (int j = i + 1; j < numNodes_SecondEchelon; ++j)
        {
            index_i_SecondEchelon[e] = i;
            index_j_SecondEchelon[e] = j;
            index_e_SecondEchelon[i][j] = index_e_SecondEchelon[j][i] = e;
            e++;
        }
    }
    assert(e == numEdges_SecondEchelon);

    for (int i = 0; i < numNodes_SecondEchelon; ++i)
    {
        index_e_SecondEchelon[i][i] = NONE;
    }

    set_WarehouseVehicles.resize(numWarehouses, vector<int>(numVehicles_Warehouse));
    for (int w = 0; w < numWarehouses; ++w)
    {
        std::iota(set_WarehouseVehicles[w].begin(), set_WarehouseVehicles[w].end(), w * numVehicles_Warehouse);
    }
}

void ParameterSetting::setHyperparameters()
{
    cout << "Set Hyperparameters For The " << problemType << "." << endl;
    if (problemType == "S2EPRP-AR")
    {
        HILS_TimeLimit = 3600.0;
        HILS_MaxIteration = 300;
        HILS_MaxNoImprovement = 20;
        // -----------------------
        ILS_TimeLimit = 150.0;
        ILS_MaxIteration = 20;
        ILS_MaxNoImprovement = 10;
        // -----------------------
        LS_MaxIterRVND = 10;
        // -----------------------
        Perturb_MaxIter = 10;
        // -----------------------
        MWPRP_FE_OptimalityGap = 1e-4;
        MWPRP_FE_TimeLimit = 300.0;
        MWPRP_FE_NumThreads = 20;
        MWPRP_FE_MemoryLimit = 32000.0;
        // -----------------------
        RS2EPRP_OptimalityGap = 1e-2;
        RS2EPRP_TimeLimit = 120.0;
        RS2EPRP_NumThreads = 20;
        RS2EPRP_MemoryLimit = 32000.0;
        // -----------------------
        BC_OptimalityGap = 1e-6;
        BC_TimeLimit = 7200.0;
        BC_NumThreads = 20;
        BC_MemoryLimit = 32000.0;
        // -----------------------
    }
    else
    {
        HILS_TimeLimit = 3600.0;
        HILS_MaxIteration = 200;
        HILS_MaxNoImprovement = 20;
        // -----------------------
        ILS_TimeLimit = 150.0;
        ILS_MaxIteration = 30;
        ILS_MaxNoImprovement = 10;
        // -----------------------
        LS_MaxIterRVND = 10;
        // -----------------------
        Perturb_MaxIter = 10;
        // -----------------------
        MWPRP_FE_OptimalityGap = 1e-4;
        MWPRP_FE_TimeLimit = 300.0;
        MWPRP_FE_NumThreads = 10;
        MWPRP_FE_MemoryLimit = 32000.0;
        // -----------------------
        R2EPRP_OptimalityGap = 1e-2;
        R2EPRP_TimeLimit = 60.0;
        R2EPRP_NumThreads = 10;
        R2EPRP_MemoryLimit = 32000.0;
        // -----------------------
        BC_OptimalityGap = 1e-6;
        BC_TimeLimit = 7200.0;
        BC_NumThreads = 10;
        BC_MemoryLimit = 32000.0;
        // -----------------------
        
    }
}

bool ParameterSetting::setParameters()
{
    try
    {
        cout << "\nSet Parameters For The " << problemType << "." << endl;

        // Read Data From File
        cout << "Read Data From File: " << endl;
        if (!readDataFromFile())
        {
            throw std::runtime_error("Unable to Read Data From File");
        }

        // for (int w = 0; w < numWarehouses; ++w)
        // {
        //     storageCapacity_Warehouse[w] = std::floor(2 * storageCapacity_Warehouse[w]);
        // }
        // // vehicleCapacity_Plant = std::floor(0.6 * vehicleCapacity_Plant);
        // vehicleCapacity_Warehouse = std::floor(1.5 * vehicleCapacity_Warehouse);

        calculateTransportationCost_FirstEchelon();
        calculateTransportationCost_SecondEchelon();
        sort_warehouses_by_distance();

        if (problemType == "S2EPRP-AR")
        {
            // Execute Monte Carlo Simulation
            cout << "Execute Monte-Carlo Simulation (To Obtain Stochastic Demands)." << endl;
            if (!monteCarloSimulation())
            {
                throw std::runtime_error("Unable to Execute Monte Carlo Simulation");
            }
            else
            {
                cout << "Monte Carlo Simulation Completed." << endl;
            }

            checkDemandsDistribution();
            calculateDeliveryUB();
            calculateUnmetDemandPenalty();
            setProbabilities();
            assign_customers_to_warehouse();
        }
        else if (problemType == "EV" || problemType == "EEV" || problemType == "WS")
        {
            // Execute Monte Carlo Simulation
            cout << "Execute Monte-Carlo Simulation (To Obtain Stochastic Demands)." << endl;
            if (!monteCarloSimulation())
            {
                throw std::runtime_error("Unable to Execute Monte Carlo Simulation");
            }
            else
            {
                cout << "Monte Carlo Simulation Completed." << endl;
            }

            checkDemandsDistribution();
            calculateDeliveryUB();
            calculateUnmetDemandPenalty();
            setProbabilities();
            calculateDeterministicDemand();
            if (problemType != "EEV"){
                calculateDeliveryUB_Deterministic();
            }
            else
            {
                if (solutionAlgorithm == "BC")
                {
                    calculateDeliveryUB_Deterministic();
                }
            }
            assign_customers_to_warehouse();
        }
        else
        {
            calculateDeterministicDemand();
            calculateDeliveryUB_Deterministic();
            if (problemType == "2EPRPCS")
            {
                setSatelliteUnitHandlingCost();

                initialInventory_Warehouse.resize(numWarehouses, 0);
                for (int w = 0; w < numWarehouses; ++w)
                {
                    storageCapacity_Warehouse[w] = 1e6 * storageCapacity_Warehouse[w];
                    unitHoldingCost_Warehouse[w] = 1e6 * unitHoldingCost_Warehouse[w];
                }
            }
            assign_customers_to_warehouse();
            unmetDemandPenalty.assign(numCustomers, 1e6 * unitProdCost);
        }

        printParameters();
        // saveInstance();

        generateAllRoutes();
    }
    catch (const std::exception &e)
    {
        cerr << "Error in setParameters: " << e.what() << endl;
        return false;
    }
    return true;
}

bool ParameterSetting::readDataFromFile()
{
    try
    {
        std::ifstream file(inputFile);
        if (!file.is_open())
        {
            throw std::runtime_error("Unable to open file");
        }

        read2ESPRPlDatasetInfo(file);

        file.close();
        cout << "File loaded successfully." << endl;
    }
    catch (const std::exception &e)
    {
        cerr << "Error in readDataFromFile: " << e.what() << endl;
        return false;
    }
    return true;
}

void ParameterSetting::read2ESPRPlDatasetInfo(std::ifstream &file)
{
    std::string line;
    int customerIndex = 0;
    int warehouseIndex = 0;

    // Skip the header lines
    std::getline(file, line); // Skip "THIS IS AN INSTANCE FOR THE TWO-ECHELON PRODUCTION ROUTING PROBLEM"
    std::getline(file, line); // Skip "THIS INSTANCE INCLUDES ONE PRODUCTION PLANT, ..."
    std::getline(file, line); // Skip "THIS INSTANCE HAS 6 PERIODS, ..."

    // Read plant vehicle capacity from the fourth line
    std::getline(file, line);
    sscanf(line.c_str(), "PLANT VEHICLE CAPACITY: %lf", &vehicleCapacity_Plant);

    // Read warehouse vehicle capacity from the fifth line
    std::getline(file, line);
    sscanf(line.c_str(), "WAREHOUSE VEHICLE CAPACITY: %lf", &vehicleCapacity_Warehouse);

    // Read the production plant data
    std::getline(file, line); // Skip "PRODUCTION_PLANT, COORD_X, COORD_Y, ..."
    if (std::getline(file, line))
    {
        sscanf(line.c_str(), "%*d, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
               &coordX_Plant, &coordY_Plant, &initialInventory_Plant,
               &unitHoldingCost_Plant, &storageCapacity_Plant,
               &unitProdCost, &setupCost, &prodCapacity);
    }

    // Read the warehouses data
    std::getline(file, line); // Skip "WAREHOUSE, COORD_X, COORD_Y, ..."
    while (warehouseIndex < numWarehouses && std::getline(file, line))
    {
        sscanf(line.c_str(), "%*d, %lf, %lf, %lf, %lf, %lf",
               &coordX_Warehouse[warehouseIndex], &coordY_Warehouse[warehouseIndex],
               &initialInventory_Warehouse[warehouseIndex],
               &unitHoldingCost_Warehouse[warehouseIndex],
               &storageCapacity_Warehouse[warehouseIndex]);
        warehouseIndex++;
    }

    // Read the customers data
    std::getline(file, line); // Skip "CUSTOMER, COORD_X, COORD_Y, ..."
    while (customerIndex < numCustomers && std::getline(file, line))
    {
        sscanf(line.c_str(), "%*d, %lf, %lf, %lf, %lf, %lf, %lf",
               &coordX_Customer[customerIndex], &coordY_Customer[customerIndex],
               &initialInventory_Customer[customerIndex],
               &unitHoldingCost_Customer[customerIndex],
               &storageCapacity_Customer[customerIndex],
               &consumeRate[customerIndex]);
        customerIndex++;
    }
}

void ParameterSetting::checkDemandsDistribution() const
{
    cout << "Check the demands distribution: " << endl;
    double sum_demands_avg = numPeriods * std::accumulate(consumeRate.begin(), consumeRate.end(), 0.0);
    cout << "Sum of demands in all periods (population): " << sum_demands_avg << endl;

    double sum_demands = 0.0;
    for (const auto &dem_customer : demand)
    {
        for (const auto &dem_period : dem_customer)
        {
            sum_demands += 1. / numScenarios * std::accumulate(dem_period.begin(), dem_period.end(), 0.0);
        }
    }
    cout << "Sum of demands in all periods (sample): " << sum_demands << endl;
}

void ParameterSetting::calculateTransportationCost_FirstEchelon()
{
    for (int i = 0; i < numNodes_FirstEchelon - 1; ++i)
    {
        for (int j = i + 1; j < numNodes_FirstEchelon; ++j)
        {
            if (i == 0)
            {
                transportationCost_FirstEchelon[i][j] = transportationCost_FirstEchelon[j][i] = std::floor(std::hypot(coordX_Plant - coordX_Warehouse[j - 1], coordY_Plant - coordY_Warehouse[j - 1]) + 0.5);
                // cout << "X_Plant: " << coordX_Plant << ", X_Warehouse" << j << ": " << coordX_Warehouse[j - 1] << endl;
                // cout << "Y_Plant: " << coordY_Plant << ", Y_Warehouse" << j << ": " << coordY_Warehouse[j - 1] << endl;
                // cout << "Dist Plant and Warehouse: " << transportationCost_FirstEchelon[i][j] << endl;
            }
            else
            {
                transportationCost_FirstEchelon[i][j] = transportationCost_FirstEchelon[j][i] = std::floor(std::hypot(coordX_Warehouse[i - 1] - coordX_Warehouse[j - 1], coordY_Warehouse[i - 1] - coordY_Warehouse[j - 1]) + 0.5);
                // cout << "X_Warehouse" << i << ": " << coordX_Warehouse[i - 1] << ", X_Warehouse" << j << ": " << coordX_Warehouse[j - 1] << endl;
                // cout << "Y_Warehouse" << i << ": " << coordY_Warehouse[i - 1] << ", Y_Warehouse" << j << ": " << coordY_Warehouse[j - 1] << endl;
                // cout << "Dist Warehouse" << i << " and Warehouse" << j << ": " << transportationCost_FirstEchelon[i][j] << endl;
            }
            // cout << "Cost_FirstE[" << i << "][" << j << "] = " << transportationCost_FirstEchelon[i][j] << endl;
        }
    }
}

void ParameterSetting::calculateTransportationCost_SecondEchelon()
{
    for (int i = 0; i < numNodes_SecondEchelon - 1; ++i)
    {
        for (int j = i + 1; j < numNodes_SecondEchelon; ++j)
        {
            if (i < numWarehouses && j < numWarehouses)
            {
                transportationCost_SecondEchelon[i][j] = transportationCost_SecondEchelon[j][i] = std::floor(std::hypot(coordX_Warehouse[i] - coordX_Warehouse[j], coordY_Warehouse[i] - coordY_Warehouse[j]) + 0.5);
                // cout << "X_Warehouse" << i << ": " << coordX_Warehouse[i] << ", X_Warehouse" << j << ": " << coordX_Warehouse[j] << endl;
                // cout << "Y_Warehouse" << i << ": " << coordY_Warehouse[i] << ", Y_Warehouse" << j << ": " << coordY_Warehouse[j] << endl;
                // cout << "Dist Warehouse" << i << " and Warehouse" << j << ": " << transportationCost_SecondEchelon[i][j] << endl;
            }
            else if (i < numWarehouses && j >= numWarehouses)
            {
                transportationCost_SecondEchelon[i][j] = transportationCost_SecondEchelon[j][i] = std::floor(std::hypot(coordX_Warehouse[i] - coordX_Customer[j - numWarehouses], coordY_Warehouse[i] - coordY_Customer[j - numWarehouses]) + 0.5);
                // cout << "X_Warehouse" << i << ": " << coordX_Warehouse[i] << ", X_Customer" << j - numWarehouses << ": " << coordX_Customer[j - numWarehouses] << endl;
                // cout << "Y_Warehouse" << i << ": " << coordY_Warehouse[i] << ", Y_Customer" << j - numWarehouses << ": " << coordY_Customer[j - numWarehouses] << endl;
                // cout << "Dist Warehouse" << i << " and Customer" << j - numWarehouses << ": " << transportationCost_SecondEchelon[i][j] << endl;
            }
            else
            {
                transportationCost_SecondEchelon[i][j] = transportationCost_SecondEchelon[j][i] = std::floor(std::hypot(coordX_Customer[i - numWarehouses] - coordX_Customer[j - numWarehouses], coordY_Customer[i - numWarehouses] - coordY_Customer[j - numWarehouses]) + 0.5);
                // cout << "X_Customer" << i - numWarehouses << ": " << coordX_Customer[i - numWarehouses] << ", X_Customer" << j - numWarehouses << ": " << coordX_Customer[j - numWarehouses] << endl;
                // cout << "Y_Customer" << i - numWarehouses << ": " << coordY_Customer[i - numWarehouses] << ", Y_Customer" << j - numWarehouses << ": " << coordY_Customer[j - numWarehouses] << endl;
                // cout << "Dist Customer" << i - numWarehouses << " and Customer" << j - numWarehouses << ": " << transportationCost_SecondEchelon[i][j] << endl;
            }
        }
    }
}

void ParameterSetting::calculateUnmetDemandPenalty()
{
    for (int i = 0; i < numCustomers; ++i)
    {
        unmetDemandPenalty[i] = unmetDemandPenaltyCoeff * std::ceil(unitProdCost + (setupCost / prodCapacity) +
                                                                    (2 * ((transportationCost_SecondEchelon[sorted_warehouses_by_distance[i][0]][i + numWarehouses] / vehicleCapacity_Warehouse) +
                                                                          (transportationCost_FirstEchelon[0][sorted_warehouses_by_distance[i][0] + 1] / vehicleCapacity_Plant))));

        // cout << unmetDemandPenalty[i] << endl;
    }
}

void ParameterSetting::setProbabilities()
{
    probability.assign(numScenarios, 1.0 / numScenarios);
}

void ParameterSetting::calculateDeliveryUB()
{
    for (int s = 0; s < numScenarios; ++s)
    {
        for (int t = 0; t < numPeriods; ++t)
        {
            double remainingDemandAllCustomers = 0.0;
            for (int i = 0; i < numCustomers; ++i)
            {
                double remainingDemand = 0.0;
                for (int l = t; l < numPeriods; ++l)
                {
                    remainingDemand += demand[i][l][s];
                    remainingDemandAllCustomers += demand[i][l][s];
                }
                DeliveryUB_perCustomer[i][t][s] = std::min({remainingDemand, vehicleCapacity_Warehouse, storageCapacity_Customer[i]});
            }
            DeliveryUB[t][s] = remainingDemandAllCustomers;
        }
    }
}

void ParameterSetting::calculateDeterministicDemand()
{
    if (problemType == "EEV" || problemType == "WS")
    {
        for (int t = 0; t < numPeriods; ++t)
        {
            for (int i = 0; i < numCustomers; ++i)
            {
                demand_Deterministic[i][t] = demand[i][t][scenarioIndex];
            }
        }
    }
    else if (problemType == "EV" || problemType == "2EPRP" || problemType == "2EPRPCS")
    {
        for (int t = 0; t < numPeriods; ++t)
        {
            for (int i = 0; i < numCustomers; ++i)
            {
                demand_Deterministic[i][t] = consumeRate[i];
            }
        }
    }
}

void ParameterSetting::calculateDeliveryUB_Deterministic()
{

    for (int t = 0; t < numPeriods; ++t)
    {
        double remainingDemandAllCustomers = 0.0;
        for (int i = 0; i < numCustomers; ++i)
        {
            double remainingDemand = 0.0;
            for (int l = t; l < numPeriods; ++l)
            {
                remainingDemand += demand_Deterministic[i][l];
                remainingDemandAllCustomers += demand_Deterministic[i][l];
            }
            DelUB_perCus_Det[i][t] = std::min({remainingDemand, vehicleCapacity_Warehouse, storageCapacity_Customer[i]});
        }
        DelUB_Det[t] = remainingDemandAllCustomers;
    }
}

void ParameterSetting::setSatelliteUnitHandlingCost()
{
    unitHandlingCost_Satellite.assign(numWarehouses, 0.0);
    double satelliteHandlingCostCoeff = 0.1;
    for (int w = 0; w < numWarehouses; ++w)
    {
        unitHandlingCost_Satellite[w] = satelliteHandlingCostCoeff * unitHoldingCost_Plant;
    }
}

void ParameterSetting::printParameters() const
{
    cout << "Instance: " << instance << endl;
    cout << "Number of Warehouses (W): " << numWarehouses << endl;
    cout << "Number of Customers (C): " << numCustomers << endl;
    cout << "Number of Periods (T): " << numPeriods << endl;
    cout << "Fleet Size (Plant) (K_plant): " << numVehicles_Plant << endl;
    cout << "Fleet Capacity (Plant) (Q_plant): " << vehicleCapacity_Plant << endl;
    cout << "Fleet Size (Each Warehouse) (K_warehouse): " << numVehicles_Warehouse << endl;
    cout << "Fleet Capacity (Warehouse) (Q_warehouse): " << vehicleCapacity_Warehouse << endl;
    if (problemType != "2EPRP" && problemType != "2EPRPCS")
    {
        cout << "Number of Scenarios (S): " << numScenarios << endl;
        cout << "Penalty Coefficient for Unit of Unmet Demand (alpha): " << unmetDemandPenaltyCoeff << endl;
        cout << "Probability Function: " << probabilityFunction << endl;
    }

    cout << "\nPlant parameters: " << endl;
    cout << "x_coord = " << coordX_Plant << ", y_coord = " << coordY_Plant << endl;
    cout << "production_capacity = " << prodCapacity << ", storage_capacity = " << storageCapacity_Plant << ", initial_inventory = " << initialInventory_Plant << endl;
    cout << "setup_cost = " << setupCost << ", unit_production_cost = " << unitProdCost << ", unit_holding_cost = " << unitHoldingCost_Plant << endl;

    cout << "\nWarehouses parameters: " << endl;
    for (int i = 0; i < numWarehouses; ++i)
    {
        cout << "x_coord = " << coordX_Warehouse[i] << ", y_coord = " << coordY_Warehouse[i] << endl;
        cout << "storage_capacity = " << storageCapacity_Warehouse[i] << ", initial_inventory = " << initialInventory_Warehouse[i] << ", unit_holding_cost = " << unitHoldingCost_Warehouse[i] << endl;
    }

    cout << "\nCustomers parameters: " << endl;
    for (int i = 0; i < numCustomers; ++i)
    {
        cout << "x_coord = " << coordX_Customer[i] << ", y_coord = " << coordY_Customer[i] << endl;
        cout << "inventory_capacity = " << storageCapacity_Customer[i] << ", initial_inventory = " << initialInventory_Customer[i] << ", unit_holding_cost = " << unitHoldingCost_Customer[i] << endl;
        cout << "nominal_demand = " << consumeRate[i] << endl;
    }
}

void ParameterSetting::saveInstance()
{
    string file_path = "../instances2eprp/" + instance + "_W" + std::to_string(numWarehouses) +
                       "_R" + std::to_string(numCustomers) + "_T" + std::to_string(numPeriods) +
                       "_KP" + std::to_string(numVehicles_Plant) + "_KW" + std::to_string(numVehicles_Warehouse) + ".txt";
    try
    {
        std::ofstream file(file_path);
        if (file.is_open())
        {
            file << "ORIGINAL INSTANCE: " << instance << "\n";
            file << "THIS IS AN INSTANCE FOR THE TWO ECHELON PRODUCTION ROUTING PROBLEM\n";
            file << "THIS INSTANCE INCLUDES ONE PRODUCTION PLANT, " << numWarehouses << " WAREHOUSES, AND " << numCustomers << " RETAILERS.\n";
            file << "THIS INSTANCE HAS " << numPeriods << " PERIODS, " << numVehicles_Plant << " VEHICLES AT THE PRODUCTION PLANT, AND " << numVehicles_Warehouse << " VEHICLES AT EACH WAREHOUSE.\n";
            file << "PLANT VEHICLE CAPACITY: " << std::setprecision(1) << std::fixed << vehicleCapacity_Plant << "\n";
            file << "WAREHOUSE VEHICLE CAPACITY: " << std::setprecision(1) << std::fixed << vehicleCapacity_Warehouse << "\n";

            file << "PRODUCTION_PLANT, COORD_X, COORD_Y, INITIAL_INVENTORY, UNIT_HOLDING_COST, INVENTORY_CAPACITY, UNIT_PRODUCTION_COST, SETUP_COST, PRODUCTION_CAPACITY\n";
            file << "0, " << std::setprecision(1) << std::fixed << coordX_Plant << ", " << coordY_Plant << ", " << initialInventory_Plant << ", " << unitHoldingCost_Plant << ", " << storageCapacity_Plant << ", " << unitProdCost << ", " << setupCost << ", " << prodCapacity << "\n";

            file << "WAREHOUSE, COORD_X, COORD_Y, INITIAL_INVENTORY, UNIT_HOLDING_COST, INVENTORY_CAPACITY\n";
            for (int ind = 0; ind < numWarehouses; ++ind)
            {
                file << ind + 1 << ", " << std::setprecision(1) << std::fixed << coordX_Warehouse[ind] << ", " << coordY_Warehouse[ind] << ", " << initialInventory_Warehouse[ind] << ", " << unitHoldingCost_Warehouse[ind] << ", " << storageCapacity_Warehouse[ind] << "\n";
            }

            file << "RETAILER, COORD_X, COORD_Y, INITIAL_INVENTORY, UNIT_HOLDING_COST, INVENTORY_CAPACITY, DEMAND_AVG\n";
            for (int ind = 0; ind < numCustomers; ++ind)
            {
                file << ind + 1 + numWarehouses << ", " << std::setprecision(1) << std::fixed << coordX_Customer[ind] << ", " << coordY_Customer[ind] << ", " << initialInventory_Customer[ind] << ", " << unitHoldingCost_Customer[ind] << ", " << storageCapacity_Customer[ind] << ", " << consumeRate[ind] << "\n";
            }

            cout << "Instance Data Was Saved Successfully at: " << file_path << endl;
            file.close();
        }
        else
        {
            throw std::runtime_error("Unable to open file: " + file_path);
        }
    }
    catch (const std::exception &e)
    {
        cerr << "Error occurred while saving instance data: " << e.what() << endl;
    }
}

bool ParameterSetting::monteCarloSimulation()
{
    try
    {
        for (int s = 0; s < numScenarios; ++s)
        {
            for (int t = 0; t < numPeriods; ++t)
            {
                generateScenarioDemands(s, t);
            }
        }
    }
    catch (const std::exception &e)
    {
        cerr << "Error in monteCarloSimulation: " << e.what() << endl;
        return false;
    }
    return true;
}

void ParameterSetting::generateScenarioDemands(int scenario, int period)
{
    for (int i = 0; i < numCustomers; ++i)
    {
        double demand = 0.0;
        unsigned int seed = i + (period * numCustomers) + (scenario * numPeriods * numCustomers);

        if (probabilityFunction == "Uniform")
        {
            double demand_range_min = (1 - uncertaintyRange) * consumeRate[i];
            double demand_range_max = (1 + uncertaintyRange) * consumeRate[i];
            demand = std::round(uniformDistribution(demand_range_min, demand_range_max, seed));
        }
        else if (probabilityFunction == "Normal")
        {
            double uniformSD = std::sqrt((std::pow((2 * uncertaintyRange * consumeRate[i]), 2)) / 12);
            demand = std::round(normalDistribution(consumeRate[i], uniformSD, seed));
        }
        else if (probabilityFunction == "Gamma")
        {
            double uniformSD = std::sqrt((std::pow((2 * uncertaintyRange * consumeRate[i]), 2)) / 12);
            demand = std::round(gammaDistribution(consumeRate[i], uniformSD, seed));
        }

        if (demand < 0.01 || std::isnan(demand))
        {
            demand = 0.0;
        }
        this->demand[i][period][scenario] = demand;

        // cout << "Demand for customer " << i << ", Period " << period << ", Scenario " << scenario << " = " << demand << endl;
    }
}

double ParameterSetting::uniformDistribution(double min, double max, unsigned long int seed)
{
    if (min > max)
    {
        throw std::invalid_argument("Minimum value cannot be greater than maximum value.");
    }

    if (seed == 0)
    {
        std::random_device rd; // Will generate a different seed each time
        seed = rd();
    }

    // Seed the random number engine with a fixed value
    std::mt19937 gen(seed);

    // Create a uniform real distribution
    std::uniform_real_distribution<> dis(min, max);

    return dis(gen);
}

double ParameterSetting::normalDistribution(double mean, double sd, unsigned int seed)
{
    // Seed the random number engine with a fixed value
    std::mt19937 gen(seed);

    // Create a normal distribution
    std::normal_distribution<double> dis(mean, sd);

    return dis(gen);
}

double ParameterSetting::gammaDistribution(double mean, double sd, unsigned int seed)
{
    // Calculate the shape and scale parameters based on mean and standard deviation
    double shape = (mean * mean) / (sd * sd);
    double scale = (sd * sd) / mean;

    // Seed the random number engine with a fixed value
    std::mt19937 gen(seed);

    // Create a gamma distribution
    std::gamma_distribution<double> dis(shape, scale);

    return dis(gen);
}

void ParameterSetting::sort_warehouses_by_distance()
{
    try
    {
        sorted_warehouses_by_distance.resize(numCustomers);

        for (int custInd = 0; custInd < numCustomers; ++custInd)
        {
            vector<std::pair<int, double>> distances; // Pair of warehouse ID and distance

            for (int w = 0; w < numWarehouses; ++w)
            {
                double distance = std::hypot(coordX_Customer[custInd] - coordX_Warehouse[w], coordY_Customer[custInd] - coordY_Warehouse[w]);
                distances.emplace_back(w, distance);
            }

            // Sort distances based on the second element (distance)
            std::sort(distances.begin(), distances.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b)
                      { return a.second < b.second; });

            // Extract sorted warehouse IDs
            vector<int> sorted_warehouses;
            for (const auto &pair : distances)
            {
                sorted_warehouses.push_back(pair.first);
            }
            sorted_warehouses_by_distance[custInd] = sorted_warehouses;

            // for (const auto &pair : distances)
            // {
            //     cout << "distance: " << pair.second << ", warehouse: " << pair.first << endl;
            // }

            // cout << "Sorted warehouses by distance for customer " << custInd + numWarehouses << ": [";
            // for (int w = 0; w < numWarehouses; ++w)
            // {
            //     cout << sorted_warehouses_by_distance[custInd][w] << " ";
            // }
            // cout << "]" << endl;
        }
    }
    catch (const std::exception &e)
    {
        cerr << "Error occurred while ordering warehouses by distance: " << e.what() << endl;
        throw;
    }
}

void ParameterSetting::assign_customers_to_warehouse()
{
    try
    {
        if (problemType == "S2EPRP-AR")
        {
            customers_assigned_to_warehouse.resize(numScenarios, vector<vector<vector<int>>>(numPeriods, vector<vector<int>>(numWarehouses,
                                                                                                                             vector<int>(numCustomers, -1))));
            warehouse_assigned_to_customer.resize(numScenarios, vector<vector<int>>(numPeriods, vector<int>(numCustomers, -1)));

            vector<std::pair<int, double>> customer_costRatio; // Pair of customer ID and penalty cost
            for (int i = 0; i < numCustomers; ++i)
            {
                customer_costRatio.emplace_back(i, unmetDemandPenalty[i]);
            }
            // Sort based on the penalty cost (second element of the pair) in descending order
            std::sort(customer_costRatio.begin(), customer_costRatio.end(),
                      [](const std::pair<int, double> &a, const std::pair<int, double> &b)
                      {
                          return a.second > b.second;
                      });

            // Store sorted customer IDs in a new vector
            vector<int> sorted_customer_costRatio;
            for (const auto &pair : customer_costRatio)
            {
                sorted_customer_costRatio.push_back(pair.first);
            }

            const double adjustmentFactor = 0.9;
            for (int s = 0; s < numScenarios; ++s)
            {
                for (int t = 0; t < numPeriods; ++t)
                {
                    vector<double> remainingStorageCapacityWarehouse(numWarehouses, 0.0);
                    vector<double> remainingVehicleCapacityWarehouse(numWarehouses, 0.0);
                    for (int w = 0; w < numWarehouses; ++w)
                    {
                        remainingStorageCapacityWarehouse[w] = storageCapacity_Warehouse[w];
                        remainingVehicleCapacityWarehouse[w] = adjustmentFactor * numVehicles_Warehouse * vehicleCapacity_Warehouse;
                    }

                    for (int custInd : sorted_customer_costRatio)
                    {
                        int assigned_ware = -1;
                        for (int wareInd : sorted_warehouses_by_distance[custInd])
                        {
                            if (remainingStorageCapacityWarehouse[wareInd] - demand[custInd][t][s] >= 0.0 &&
                                remainingVehicleCapacityWarehouse[wareInd] - demand[custInd][t][s] >= 0.0)
                            {
                                remainingStorageCapacityWarehouse[wareInd] -= demand[custInd][t][s];
                                remainingVehicleCapacityWarehouse[wareInd] -= demand[custInd][t][s];
                                assigned_ware = wareInd;
                                break;
                            }
                        }
                        if (assigned_ware != -1)
                        {
                            customers_assigned_to_warehouse[s][t][assigned_ware][custInd] = 1;
                            warehouse_assigned_to_customer[s][t][custInd] = assigned_ware;
                        }
                        else
                        {
                            assigned_ware = sorted_warehouses_by_distance[custInd][0];
                            customers_assigned_to_warehouse[s][t][assigned_ware][custInd] = 1;
                            warehouse_assigned_to_customer[s][t][custInd] = assigned_ware;
                        }

                        for (int w = 0; w < numWarehouses; ++w)
                        {
                            if (w != assigned_ware)
                            {
                                customers_assigned_to_warehouse[s][t][w][custInd] = 0;
                            }
                        }
                    }
                }
            }
        }
        else
        {
            customers_assigned_to_warehouse_det.resize(numPeriods, vector<vector<int>>(numWarehouses, vector<int>(numCustomers, -1)));
            warehouse_assigned_to_customer_det.resize(numPeriods, vector<int>(numCustomers, -1));

            const double adjustmentFactor = 0.9;
            for (int t = 0; t < numPeriods; ++t)
            {
                vector<double> remainingStorageCapacityWarehouse(numWarehouses, 0.0);
                vector<double> remainingVehicleCapacityWarehouse(numWarehouses, 0.0);
                for (int w = 0; w < numWarehouses; ++w)
                {
                    remainingStorageCapacityWarehouse[w] = storageCapacity_Warehouse[w];
                    remainingVehicleCapacityWarehouse[w] = adjustmentFactor * numVehicles_Warehouse * vehicleCapacity_Warehouse;
                }

                for (int i = 0; i < numCustomers; ++i)
                {
                    int assigned_ware = -1;
                    for (int wareInd : sorted_warehouses_by_distance[i])
                    {
                        if (remainingStorageCapacityWarehouse[wareInd] - demand_Deterministic[i][t] >= 0.0 &&
                            remainingVehicleCapacityWarehouse[wareInd] - demand_Deterministic[i][t] >= 0.0)
                        {
                            remainingStorageCapacityWarehouse[wareInd] -= demand_Deterministic[i][t];
                            remainingVehicleCapacityWarehouse[wareInd] -= demand_Deterministic[i][t];
                            assigned_ware = wareInd;
                            break;
                        }
                    }
                    if (assigned_ware != -1)
                    {
                        customers_assigned_to_warehouse_det[t][assigned_ware][i] = 1;
                        warehouse_assigned_to_customer_det[t][i] = assigned_ware;
                    }
                    else
                    {
                        assigned_ware = sorted_warehouses_by_distance[i][0];
                        customers_assigned_to_warehouse_det[t][assigned_ware][i] = 1;
                        warehouse_assigned_to_customer_det[t][i] = assigned_ware;
                    }

                    for (int w = 0; w < numWarehouses; ++w)
                    {
                        if (w != assigned_ware)
                        {
                            customers_assigned_to_warehouse_det[t][w][i] = 0;
                        }
                    }
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        cerr << "Error occurred while assigning customers to warehouses: " << e.what() << endl;
    }
}

void ParameterSetting::generateAllRoutes()
{
    try
    {
        int totalSubsets = pow(2, numWarehouses);
        for (int i = 1; i < totalSubsets; ++i)
        {
            vector<int> subset(numWarehouses + 2, 0);
            subset[0] = 1;
            subset[numWarehouses + 1] = 1;
            for (int j = 0; j < numWarehouses; ++j)
            {
                if (i & (1 << j))
                {
                    subset[j + 1] = 1;
                }
            }
            routeMatrix.push_back(subset);
        }

        // for (const auto &route : routeMatrix)
        // {
        //     for (int value : route)
        //     {
        //         cout << value << " ";
        //     }
        //     cout << endl;
        // }

        cout << "Generating all routes..." << endl;
        solveTSPForRoutes();

        // cout << "\n"
        //      << endl;
        int ind = 1;
        for (const auto &route : optimalRoutes)
        {
            cout << "Optimal Route (" << ind << "): ";
            for (int node : route)
            {
                cout << node << " ";
            }
            cout << "| Cost: " << routeCosts[ind - 1];
            cout << endl;
            ind++;
        }
    }
    catch (const std::exception &e)
    {
        cerr << "Error occurred while generating all routes: " << e.what() << endl;
    }
}

void ParameterSetting::solveTSPForRoutes()
{
    try
    {
        for (const auto &route : routeMatrix)
        {
            // Count the number of 1s in the route
            int numOnes = std::count(route.begin(), route.end(), 1);
            // cout << "route size:" << numOnes - 1 << endl;

            if (numOnes < 3)
            {
                routeCosts.push_back(0.0);
                continue;
            }
            else if (numOnes == 3)
            {
                double routeCost = 0.0;
                vector<int> optRoute = {0};

                for (int i = 1; i <= numWarehouses; ++i)
                {
                    if (route[i] == 1)
                    {
                        optRoute.push_back(i);
                        routeCost += 2 * transportationCost_FirstEchelon[0][i];
                        break;
                    }
                }
                // cout << "route cost:" << routeCost << endl;
                optRoute.push_back(0);

                optimalRoutes.push_back(optRoute);
                routeCosts.push_back(routeCost);
            }
            else if (numOnes > 3)
            {
                // Extract the indices of nodes to visit
                vector<int> nodesToVisit;
                for (int i = 0; i <= numWarehouses; ++i)
                {
                    if (route[i] == 1)
                    {
                        nodesToVisit.push_back(i);
                    }
                }

                // Create a reduced cost matrix
                int size = nodesToVisit.size();
                vector<vector<double>> reducedCostMatrix(size, vector<double>(size));
                for (int i = 0; i < size; ++i)
                {
                    for (int j = 0; j < size; ++j)
                    {
                        reducedCostMatrix[i][j] = transportationCost_FirstEchelon[nodesToVisit[i]][nodesToVisit[j]];
                    }
                }

                // Solve the TSP for the reduced cost matrix
                TSP tsp(reducedCostMatrix);
                bool status = tsp.solve();
                if (!status)
                {
                    throw std::runtime_error("Failed to solve TSP");
                }

                vector<int> tspRoute = tsp.getRouteTSP();

                // Map the TSP solution back to the original nodes
                vector<int> mappedSolution;
                for (int index : tspRoute)
                {
                    mappedSolution.push_back(nodesToVisit[index]);
                }

                optimalRoutes.push_back(mappedSolution);
                routeCosts.push_back(tsp.getObjValue());
            }
        }
    }
    catch (const std::exception &e)
    {
        cerr << "Error occurred while solving TSP for routes: " << e.what() << endl;
    }
}

SolutionWarmStart ParameterSetting::readSolutionWarmStart_Stoch()
{
    cout << "Read Solution For Warm Start..." << endl;
    string directory;
    string filename;
    if (problemType == "S2EPRP-AR" && solutionAlgorithm == "BC")
    {
        cout << "Read Warmstart for " << problemType << " and solutionAlgorithm: " << solutionAlgorithm << "..." << endl;
        // Construct the directory path
        // directory = "../Results/Solutions/" + problemType + "/Preliminary_Test02" + "/Hybrid-ILS/" + probabilityFunction + "/S" + std::to_string(numScenarios) + "/UR" + std::to_string(static_cast<int>(uncertaintyRange * 100)) + "%" + "/PC" + std::to_string(static_cast<int>(unmetDemandPenaltyCoeff));
        directory = "../Results/Solutions/" + problemType + "/Hybrid-ILS/" + probabilityFunction +
                    "/S" + std::to_string(numScenarios) + "/UR" + std::to_string(static_cast<int>(uncertaintyRange * 100)) + "%" +
                    "/PC" + std::to_string(static_cast<int>(unmetDemandPenaltyCoeff));

        // Construct the filename
        filename = "Sol_" + problemType + "_Hybrid-ILS_" + probabilityFunction + "_" + instance + "_S" + std::to_string(numScenarios) + "_UR" + std::to_string(static_cast<int>(uncertaintyRange * 100)) + "%" + "_PC" + std::to_string(static_cast<int>(unmetDemandPenaltyCoeff)) + ".txt";
    }
    else
    {
        cout << "Invalid solutionAlgorithm" << endl;
        return SolutionWarmStart();
    }
    // Full path to the file
    string fullPath = directory + "/" + filename;

    SolutionWarmStart warmstart;

    std::ifstream file(fullPath);
    if (file.is_open())
    {
        int intValue;
        double doubleValue;
        string strValue;

        // Read Parameters
        file >> intValue; // numNodes_Total;
        file >> intValue; // numWarehouses;
        file >> intValue; // numCustomers;
        file >> intValue; // numVehicles_Plant;
        file >> intValue; // numVehicles_Warehouse;
        file >> intValue; // numPeriods;
        file >> intValue; // numScenarios;

        file >> doubleValue; // uncertaintyRange;
        file >> strValue;    // ProbabilityFunction;
        file >> doubleValue; // unmetDemandPenaltyCoeff;

        file >> doubleValue; // unitProdCost;
        file >> doubleValue; // setupCost;
        file >> doubleValue; // prodCapacity;
        file >> doubleValue; // vehicleCapacity_Plant;
        file >> doubleValue; // vehicleCapacity_Warehouse;

        file >> doubleValue; // Plant X Coordinates
        file >> doubleValue; // Plant Y Coordinates

        for (int w = 0; w < numWarehouses; w++)
        {
            file >> doubleValue; // Warehouse X Coordinates
        }
        for (int w = 0; w < numWarehouses; w++)
        {
            file >> doubleValue; // Warehouse Y Coordinates
        }
        for (int i = 0; i < numCustomers; i++)
        {
            file >> doubleValue; // Customers X Coordinates
        }
        for (int i = 0; i < numCustomers; i++)
        {
            file >> doubleValue; // Customers Y Coordinates
        }

        file >> doubleValue; // unit holding cost plant
        for (int w = 0; w < numWarehouses; w++)
        {
            file >> doubleValue; // holding cost warehouse
        }
        for (int i = 0; i < numCustomers; i++)
        {
            file >> doubleValue; // holding cost customer
        }

        file >> doubleValue; // storage capacity plant
        for (int w = 0; w < numWarehouses; w++)
        {
            file >> doubleValue; // storage capacity warehouse
        }
        for (int i = 0; i < numCustomers; i++)
        {
            file >> doubleValue; // storage capacity customer
        }

        file >> doubleValue; // initial inventory plant
        for (int w = 0; w < numWarehouses; w++)
        {
            file >> doubleValue; // initial inventory warehouse
        }
        for (int i = 0; i < numCustomers; i++)
        {
            file >> doubleValue; // initial inventory customer
        }

        for (int i = 0; i < numCustomers; i++)
        {
            file >> doubleValue; // unmet demand penalty
        }

        for (int i = 0; i < numCustomers; i++)
        {
            file >> doubleValue; // consume Rate
        }

        for (int s = 0; s < numScenarios; s++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int i = 0; i < numCustomers; i++)
                {
                    file >> doubleValue;
                }
            }
        }

        for (int s = 0; s < numScenarios; s++)
        {
            file >> doubleValue; // probability
        }

        for (int i = 0; i < numNodes_FirstEchelon; i++)
        {
            for (int j = 0; j < numNodes_FirstEchelon; j++)
            {
                file >> doubleValue; // Transportation Cost - First Echelon
            }
        }

        for (int i = 0; i < numNodes_SecondEchelon; i++)
        {
            for (int j = 0; j < numNodes_SecondEchelon; j++)
            {
                file >> doubleValue; // Transportation Cost - Second Echelon
            }
        }
        cout << "Warmstart file read successfully." << endl;
        // Read Solutions
        warmstart.productionSetup_WarmStart.resize(numPeriods);
        warmstart.routesPlantToWarehouse_WarmStart.resize(numPeriods, vector<vector<int>>(numVehicles_Plant, vector<int>()));
        warmstart.routesWarehouseToCustomer_WarmStart.resize(numScenarios, vector<vector<vector<vector<int>>>>(numWarehouses, vector<vector<vector<int>>>(numPeriods, vector<vector<int>>(numVehicles_Warehouse, vector<int>()))));
        warmstart.customerAssignmentToWarehouse_WarmStart.resize(numScenarios, vector<vector<vector<int>>>(numPeriods, vector<vector<int>>(numWarehouses, vector<int>(numCustomers))));

        // Read Solutions
        for (int s = 0; s < numScenarios; s++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int w = 0; w < numWarehouses; w++)
                {
                    for (int i = 0; i < numCustomers; i++)
                    {
                        file >> warmstart.customerAssignmentToWarehouse_WarmStart[s][t][w][i];
                    }
                }
            }
        }

        for (int t = 0; t < numPeriods; t++)
        {
            file >> warmstart.productionSetup_WarmStart[t];
        }

        for (int t = 0; t < numPeriods; t++)
        {
            file >> doubleValue;
        }

        for (int t = 0; t < numPeriods; t++)
        {
            file >> doubleValue;
        }

        for (int w = 0; w < numWarehouses; w++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int s = 0; s < numScenarios; s++)
                {
                    file >> doubleValue;
                }
            }
        }

        for (int i = 0; i < numCustomers; i++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int s = 0; s < numScenarios; s++)
                {
                    file >> doubleValue;
                }
            }
        }

        for (int i = 0; i < numCustomers; i++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int s = 0; s < numScenarios; s++)
                {
                    file >> doubleValue;
                }
            }
        }

        string line_One;
        while (std::getline(file, line_One))
        {
            if (line_One == "endRoutesPlantToWarehouse")
                break;

            std::istringstream iss(line_One);
            int t_index, k_index;
            char colon;
            if (!(iss >> t_index >> k_index >> colon) || colon != ':')
            {
                cerr << "Error parsing line: " << line_One << std::endl;
                continue;
            }

            int node;
            vector<int> route;
            while (iss >> node)
            {
                route.push_back(node);
            }

            // Assign the route to the appropriate vehicle
            warmstart.routesPlantToWarehouse_WarmStart[t_index][k_index] = route;
        }

        for (int w = 0; w < numWarehouses; w++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                file >> doubleValue;
            }
        }

        string line_Two;
        while (std::getline(file, line_Two))
        {
            if (line_Two == "endRoutesWarehouseToCustomer")
                break;

            std::istringstream iss(line_Two);
            int s_index, w_index, t_index, k_index;
            char colon;
            if (!(iss >> s_index >> w_index >> t_index >> k_index >> colon) || colon != ':')
            {
                cerr << "Error parsing line: " << line_Two << std::endl;
                continue;
            }

            int node;
            vector<int> route;
            while (iss >> node)
            {
                route.push_back(node);
            }

            // Assign the route to the appropriate vehicle
            warmstart.routesWarehouseToCustomer_WarmStart[s_index][w_index][t_index][k_index] = route;
        }

        for (int i = 0; i < numCustomers; i++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int s = 0; s < numScenarios; s++)
                {
                    file >> doubleValue;
                }
            }
        }

        file.close();

        cout << "File loaded successfully." << endl;
        return warmstart;
    }
    else
    {
        cerr << "No Warmstart solution was found!!" << endl;
        warmstart.clear();
        return warmstart;
    }
}


bool ParameterSetting::readSolutionWarmStart_Deter(SolutionWarmStart_Deterministic &sol_WS_Deter)
{
	cout << "Reading Warmstart Solution For the  " << problemType << " ..." << endl;

	string directory;
	string filename;
    string solutionFileName;

    if (problemType == "EV")
    {
	    directory = "../Results/Solutions/S2EPRP-AR/" + problemType + "/Hybrid-ILS/" + probabilityFunction + "/S" + std::to_string(numScenarios) +
				"/UR" + std::to_string(static_cast<int>(uncertaintyRange * 100)) + "%" +
				"/PC" + std::to_string(static_cast<int>(unmetDemandPenaltyCoeff));

        // Construct the filename
        filename = "Sol_S2EPRP-AR_" + problemType + "_Hybrid-ILS_" + probabilityFunction + "_" + instance + "_S" + std::to_string(numScenarios) +
        "_UR" + std::to_string(static_cast<int>(uncertaintyRange * 100)) + "%" +
        "_PC" + std::to_string(static_cast<int>(unmetDemandPenaltyCoeff)) + ".txt";
        solutionFileName = directory + "/" + filename;
    }
    else if (problemType == "WS" || problemType == "EEV")
    {
	    directory = "../Results/Solutions/S2EPRP-AR/" + problemType + "/Hybrid-ILS/" + probabilityFunction + "/S" + std::to_string(numScenarios) +
				"/UR" + std::to_string(static_cast<int>(uncertaintyRange * 100)) + "%" +
				"/PC" + std::to_string(static_cast<int>(unmetDemandPenaltyCoeff)) +
                "/" + instance;

        // Construct the filename
        filename = "Sol_S2EPRP-AR_" + problemType + "_Hybrid-ILS_" + probabilityFunction + "_" + instance + "_s" + std::to_string(scenarioIndex) +".txt";
        solutionFileName = directory + "/" + filename;
    }
    else if (problemType == "2EPRP" || problemType == "2EPRPCS")
    {
        directory = "../Results/Solutions/" + problemType + "/Hybrid-ILS/";

        // Construct the filename
        filename = "Sol_2EPRP_Hybrid-ILS_" + instance + ".txt";
        solutionFileName = directory + "/" + filename;
    }

	if (!fs::exists(solutionFileName))
	{
		cout << "Solution file does not exist for BC, Checking for Hybrid-ILS..." << endl;
        return false;
	}

	std::ifstream file(solutionFileName);
	if (file.is_open())
	{
		int intValue;
		double doubleValue;
		string strValue;

		// Read Parameters
		file >> intValue; // numNodes_Total;
		file >> intValue; // numWarehouses;
		file >> intValue; // numCustomers;
		file >> intValue; // numVehicles_Plant;
		file >> intValue; // numVehicles_Warehouse;
		file >> intValue; // numPeriods;

        if (problemType != "2EPRP" || problemType == "2EPRPCS")
        {
            file >> intValue; // numScenarios;
        }

        if (problemType == "WS" || problemType == "EEV")
        {
            file >> intValue; // scenarioIndex;
        }

        if (problemType != "2EPRP" || problemType == "2EPRPCS")
        {
            file >> doubleValue; // uncertaintyRange;
            file >> strValue;	 // ProbabilityFunction;
            file >> doubleValue; // unmetDemandPenaltyCoeff;
        }

		file >> doubleValue; // unitProdCost;
		file >> doubleValue; // setupCost;
		file >> doubleValue; // prodCapacity;
		file >> doubleValue; // vehicleCapacity_Plant;
		file >> doubleValue; // vehicleCapacity_Warehouse;

		file >> doubleValue; // Plant X Coordinates
		file >> doubleValue; // Plant Y Coordinates

		for (int w = 0; w < numWarehouses; w++)
		{
			file >> doubleValue; // Warehouse X Coordinates
		}
		for (int w = 0; w < numWarehouses; w++)
		{
			file >> doubleValue; // Warehouse Y Coordinates
		}
		for (int i = 0; i < numCustomers; i++)
		{
			file >> doubleValue; // Customers X Coordinates
		}
		for (int i = 0; i < numCustomers; i++)
		{
			file >> doubleValue; // Customers Y Coordinates
		}

		file >> doubleValue; // unit holding cost plant
		for (int w = 0; w < numWarehouses; w++)
		{
			file >> doubleValue; // holding cost warehouse/Handling Cost Sattelite
		}
		for (int i = 0; i < numCustomers; i++)
		{
			file >> doubleValue; // holding cost customer
		}

		file >> doubleValue; // storage capacity plant
        if (problemType != "2EPRPCS")
        {
            for (int w = 0; w < numWarehouses; w++)
            {
                file >> doubleValue; // storage capacity warehouse
            }
        }
		for (int i = 0; i < numCustomers; i++)
		{
			file >> doubleValue; // storage capacity customer
		}

		file >> doubleValue; // initial inventory plant
        if (problemType != "2EPRPCS")
        {
            for (int w = 0; w < numWarehouses; w++)
            {
                file >> doubleValue; // initial inventory warehouse
            }
        }
		for (int i = 0; i < numCustomers; i++)
		{
			file >> doubleValue; // initial inventory customer
		}

		if (problemType != "2EPRP" && problemType != "2EPRPCS")
        {
            for (int i = 0; i < numCustomers; i++)
            {
                file >> doubleValue; // unmet demand penalty
            }
        }

		for (int t = 0; t < numPeriods; t++)
		{
			for (int i = 0; i < numCustomers; i++)
			{
				file >> doubleValue; // Deterministic Demand
			}
		}

		for (int i = 0; i < numNodes_FirstEchelon; i++)
		{
			for (int j = 0; j < numNodes_FirstEchelon; j++)
			{
				file >> doubleValue; // Transportation Cost - First Echelon
			}
		}

		for (int i = 0; i < numNodes_SecondEchelon; i++)
		{
			for (int j = 0; j < numNodes_SecondEchelon; j++)
			{
				file >> doubleValue; // Transportation Cost - Second Echelon
			}
		}

		// Read Solutions
		sol_WS_Deter.productionSetup_WarmStart.resize(numPeriods);
		sol_WS_Deter.routesPlantToWarehouse_WarmStart.resize(numPeriods, vector<vector<int>>(numVehicles_Plant, vector<int>()));
		sol_WS_Deter.customerAssignmentToWarehouse_WarmStart.resize(numPeriods, vector<vector<int>>(numWarehouses, vector<int>(numCustomers)));
		sol_WS_Deter.routesWarehouseToCustomer_WarmStart.resize(numWarehouses, vector<vector<vector<int>>>(numPeriods, vector<vector<int>>(numVehicles_Warehouse, vector<int>())));
		
		// Read Solutions
		for (int t = 0; t < numPeriods; t++)
		{
			for (int w = 0; w < numWarehouses; w++)
			{
				for (int i = 0; i < numCustomers; i++)
				{
					file >> sol_WS_Deter.customerAssignmentToWarehouse_WarmStart[t][w][i]; // Customer Assignment To Warehouse
				}
			}
		}

		for (int t = 0; t < numPeriods; t++)
		{
			file >> sol_WS_Deter.productionSetup_WarmStart[t];
		}

		for (int t = 0; t < numPeriods; t++)
		{
			file >> doubleValue; // Production Quantity
		}

		for (int t = 0; t < numPeriods; t++)
		{
			file >> doubleValue; // Plant Inventory
		}
        
        if (problemType != "2EPRPCS")
        {
            for (int w = 0; w < numWarehouses; w++)
            {
                for (int t = 0; t < numPeriods; t++)
                {
                    file >> doubleValue; // Warehouse Inventory
                }
            }
        }

		for (int i = 0; i < numCustomers; i++)
		{
			for (int t = 0; t < numPeriods; t++)
			{
				file >> doubleValue; // Customer Inventory
			}
		}
        
        if (problemType != "2EPRP" && problemType != "2EPRPCS")
        {
            for (int i = 0; i < numCustomers; i++)
            {
                for (int t = 0; t < numPeriods; t++)
                {
                    file >> doubleValue; // Unmet Demand
                }
            }
        }
		
		string line_One;
		while (std::getline(file, line_One))
		{
			if (line_One == "endRoutesPlantToWarehouse")
				break;

			if (line_One.find(':') == string::npos)
				continue;

			std::istringstream iss(line_One);
			int t_index, k_index;
			char colon;
			if (!(iss >> t_index >> k_index >> colon) || colon != ':')
			{
				cerr << "Error parsing line: " << line_One << std::endl;
				continue;
			}

			int node;
			vector<int> route;
			while (iss >> node)
			{
				route.push_back(node);
			}

			// Assign the route to the appropriate vehicle
			sol_WS_Deter.routesPlantToWarehouse_WarmStart[t_index][k_index] = route;
		}

		for (int w = 0; w < numWarehouses; w++)
		{
			for (int t = 0; t < numPeriods; t++)
			{
				file >> doubleValue; // Delivery Quantity to Warehouse
			}
		}

		string line_Two;
		while (std::getline(file, line_Two))
		{
			if (line_Two == "endRoutesWarehouseToCustomer")
				break;

			if (line_Two.find(':') == string::npos)
				continue;

			std::istringstream iss(line_Two);
			int w_index, t_index, k_index;
			char colon;
			if (!(iss >> w_index >> t_index >> k_index >> colon) || colon != ':')
			{
				cerr << "Error parsing line: " << line_Two << std::endl;
				continue;
			}

			int node;
			vector<int> route;
			while (iss >> node)
			{
				route.push_back(node);
			}

			sol_WS_Deter.routesWarehouseToCustomer_WarmStart[w_index][t_index][k_index] = route;
		}

		for (int i = 0; i < numCustomers; i++)
		{
			for (int t = 0; t < numPeriods; t++)
			{
				file >> doubleValue; // Delivery Quantity to Customer
			}
		}

		file.close();

		cout << "Warm Start Solution For The Deterministic Problem loaded successfully." << endl;
		return true;
	}
	else
	{
		cerr << "Warm Start Solution For The Deterministic Problem not found!!" << endl;
		return false;
	}
}

// Getters
vector<vector<int>> ParameterSetting::getSortedWarehousesByDistance() const
{
    return sorted_warehouses_by_distance;
}

vector<vector<vector<vector<int>>>> ParameterSetting::getCustomersAssignedToWarehouse() const
{
    return customers_assigned_to_warehouse;
}

vector<vector<vector<int>>> ParameterSetting::getWarehouseAssignedToCustomer() const
{
    return warehouse_assigned_to_customer;
}

vector<vector<int>> ParameterSetting::getRouteMatrix() const
{
    return routeMatrix;
}

vector<vector<int>> ParameterSetting::getOptimalRoutes() const
{
    return optimalRoutes;
}

vector<double> ParameterSetting::getRouteCosts() const
{
    return routeCosts;
}

vector<vector<vector<int>>> ParameterSetting::getCustomersAssignedToWarehouse_det() const
{
    return customers_assigned_to_warehouse_det;
}

vector<vector<int>> ParameterSetting::getWarehouseAssignedToCustomer_det() const
{
    return warehouse_assigned_to_customer_det;
}
