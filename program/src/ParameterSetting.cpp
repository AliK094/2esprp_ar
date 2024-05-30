#include "ParameterSetting.h"

ParameterSetting::ParameterSetting(int argc, char *argv[])
    : solutionAlgorithm(argv[2]),
      inp(argv[3]),
      numWarehouses(stoi(argv[4])),
      numRetailers(stoi(argv[5])),
      numPeriods(stoi(argv[6])),
      numVehiclesPlant(stoi(argv[7])),
      numVehiclesWarehouse(stoi(argv[8])),
      numScenarios(stoi(argv[9])),
      unmetDemandPenaltyCoeff(stod(argv[10])),
      uncertaintyRange(stod(argv[11])),
      probabilityFunction(argv[12]),
      instance(argv[13]),
      numNodes(numRetailers + numWarehouses + 1),
      numEdges((numNodes * (numNodes - 1)) / 2),
      coordX(numNodes, 0),
      coordY(numNodes, 0),
      unitHoldingCost(numNodes, 0),
      storageCapacity(numNodes, 0),
      initialInventory(numNodes, 0),
      unmetDemandPenalty(numNodes, numeric_limits<double>::infinity()),
      consumeRate(numNodes, 0),
      demand(numNodes, vector<vector<double>>(numPeriods, vector<double>(numScenarios, 0))),
      index_i(numEdges, 0),
      index_j(numEdges, 0),
      index_e(numNodes, vector<int>(numNodes, 0)),
      probability(numScenarios, 0),
      transportationCost(numNodes, vector<double>(numNodes, 0))
{
    try {
        cout << "\nSolve The Stochastic 2EPRP and Adaptive Routing with " << solutionAlgorithm << endl;
        initializeIndices();
    }
    catch (const exception &e)
    {
        cerr << "Error initializing ParameterSetting: " << e.what() << endl;
        exit(1);
    }
}

void ParameterSetting::initializeIndices()
{
    int e = 0;
    for (int i = 0; i < numNodes - 1; ++i)
    {
        for (int j = i + 1; j < numNodes; ++j)
        {
            index_i[e] = i;
            index_j[e] = j;
            index_e[i][j] = e;
            index_e[j][i] = e;
            e++;
        }
    }
    assert(e == numEdges);

    for (int i = 0; i < numNodes; ++i)
    {
        index_e[i][i] = NONE;
    }
}

bool ParameterSetting::readDataFromFile()
{
    try
    {
        ifstream file(inp);
        if (!file.is_open())
        {
            throw runtime_error("Unable to open file");
        }

        readOriginalDatasetInfo(file);
        readNodeData(file);

        file.close();
        cout << "File loaded successfully." << endl;
    }
    catch (const exception &e)
    {
        cerr << "Error in readDataFromFile: " << e.what() << endl;
        return false;
    }
    return true;
}

void ParameterSetting::readOriginalDatasetInfo(ifstream &file)
{
    string line;
    int i = 0;
    while (getline(file, line) && i < 10)
    {
        if (i == 8)
        {
            sscanf(line.c_str(), "%*s %d", &originalDimension);
            cout << "Original Dimension (Deterministic): " << originalDimension << endl;
        }
        else if (i == 9)
        {
            sscanf(line.c_str(), "%*s %d", &originalHorizon);
            cout << "Original Horizon (Deterministic): " << originalHorizon << endl;
        }
        else if (i == 10)
        {
            sscanf(line.c_str(), "%*s %*s %d", &originalVehicleCapacity);
            cout << "Original Vehicle Capacity (Deterministic): " << originalVehicleCapacity << endl;
        }
        i++;
    }
}

void ParameterSetting::readNodeData(ifstream &file)
{
    string line;
    int i = 0;
    int j = 0;
    while (getline(file, line) && j < numNodes)
    {
        if (i == 13)
        {
            sscanf(line.c_str(), "%*d %lf %lf %lf %lf %lf %lf", &coordX[j], &coordY[j], &initialInventory[j], &unitHoldingCost[j], &unitProdCost, &setupCost);
            j += numWarehouses + 1;
        }
        else if (i >= 15 && j < numNodes)
        {
            sscanf(line.c_str(), "%*d %lf %lf %lf %lf %*f %lf %lf", &coordX[j], &coordY[j], &initialInventory[j], &storageCapacity[j], &consumeRate[j], &unitHoldingCost[j]);
            j++;
        }
        i++;
    }
}

bool ParameterSetting::setParameters()
{
    try
    {
        cout << "\nSet Parameters For the Stochastic Two-Echelon Production Routing Problem With Adaptive Routing: " << endl;

        // Read Data From File
        cout << "Read Data From File: " << endl;
        if (!readDataFromFile())
        {
            throw runtime_error("Unable to Read Data From File");
        }

        // Execute Monte Carlo Simulation
        cout << "Execute Monte Carlo Simulation (To Obtain Stochastic Demands). " << endl;
        if (!monteCarloSimulation())
        {
            throw runtime_error("Unable To Execute Monte Carlo Simulation");
        }
        else
        {
            cout << "Monte Carlo Simulation Completed. " << endl;
        }

        checkDemandsDistribution();


        adjustInitialInventory();
        calculateProductionCapacity();
        setWarehouseParameters();
        calculateVehicleCapacities();
        updateStorageCapacityForRetailers();
        calculateTransportationCost();
        calculateUnmetDemandPenalty();
        setProbabilities();

        sort_warehouses_by_distance();
        assing_retailers_to_warehouse();

        printParameters();

        saveInstance();
    }
    catch (const exception &e)
    {
        cerr << "Error in setParameters: " << e.what() << endl;
        return false;
    }
    return true;
}

void ParameterSetting::checkDemandsDistribution() const
{
    cout << "Check the demands distribution: " << endl;
    double sum_demands_avg = numPeriods * accumulate(consumeRate.begin() + numWarehouses + 1, consumeRate.end(), 0.0);
    cout << "Sum of demands in all periods (population): " << sum_demands_avg << endl;

    double sum_demands = 0.0;
    for (const auto &retailer_demands : demand)
    {
        for (const auto &period_demands : retailer_demands)
        {
            sum_demands += accumulate(period_demands.begin(), period_demands.end(), 0.0);
        }
    }
    cout << "Sum of demands in all periods (sample): " << sum_demands << endl;
}

void ParameterSetting::adjustInitialInventory()
{
    if (numPeriods == 3)
    {
        for (int i = 0; i < numNodes; ++i)
        {
            initialInventory[i] = floor(initialInventory[i] / 2);
        }
    }
}

void ParameterSetting::calculateProductionCapacity()
{
    double totalConsumption = 0.0;
    for (int t = 0; t < numPeriods; ++t)
    {
        for (int i = numWarehouses + 1; i < numNodes; ++i)
        {
            totalConsumption += consumeRate[i];
        }
    }
    prodCapacity = floor(2 * (totalConsumption / numPeriods));
    storageCapacity[0] = prodCapacity / 2;
}

void ParameterSetting::setWarehouseParameters()
{
    double uhc_min = min(unitHoldingCost[0], *min_element(unitHoldingCost.begin() + numWarehouses + 1, unitHoldingCost.end()));
    double uhc_max = *min_element(unitHoldingCost.begin() + numWarehouses + 1, unitHoldingCost.end());

    random_device rd;
    for (int i = 1; i <= numWarehouses; ++i)
    {
        int seed = i;
        mt19937 gen1(int(1e9 + (seed * seed)));
        mt19937 gen2(int(1e6 + (seed * seed)));
        mt19937 gen3(int(1e3 + (seed * seed)));

        uniform_int_distribution<> dis1(0, 2500);
        uniform_int_distribution<> dis2(storageCapacity[0] / 2, storageCapacity[0]);
        uniform_int_distribution<> dis3(uhc_min, uhc_max);

        coordX[i] = floor(dis1(gen1));
        coordY[i] = floor(dis1(gen2));
        initialInventory[i] = 0;
        storageCapacity[i] = floor(dis2(gen3));
        unitHoldingCost[i] = floor(dis3(gen3));
    }
}

void ParameterSetting::calculateVehicleCapacities()
{
    double maxofInvCap_warehouse = *max_element(storageCapacity.begin() + 1, storageCapacity.begin() + numWarehouses + 1);
    double plantVehicleCapacityCoefficient = 2.5 * (floor(numWarehouses / 10) + 1);
    vehicleCapacityPlant = floor(plantVehicleCapacityCoefficient * (maxofInvCap_warehouse / numVehiclesPlant));

    double maxofInvCap_retailer = *max_element(storageCapacity.begin() + numWarehouses + 1, storageCapacity.end());
    double warehouseVehicleCapacityCoefficient = 1.5 * (floor(numRetailers / (10 * numWarehouses)) + 1);
    vehicleCapacityWarehouse = floor(warehouseVehicleCapacityCoefficient * (maxofInvCap_retailer / numVehiclesWarehouse));
}

void ParameterSetting::updateStorageCapacityForRetailers()
{
    for (int i = numWarehouses + 1; i < numNodes; ++i)
    {
        storageCapacity[i] += consumeRate[i];
    }
}

void ParameterSetting::calculateTransportationCost()
{
    for (int i = 0; i < numNodes - 1; ++i)
    {
        for (int j = i + 1; j < numNodes; ++j)
        {
            if ((i == 0 && j <= numWarehouses) || (i != 0))
            {
                transportationCost[i][j] = transportationCost[j][i] = hypot(coordX[i] - coordX[j], coordY[i] - coordY[j]);
            }
            else
            {
                transportationCost[i][j] = transportationCost[j][i] = numeric_limits<double>::infinity();
            }
        }
    }
}

void ParameterSetting::calculateUnmetDemandPenalty()
{
    for (int i = numWarehouses + 1; i < numNodes; ++i)
    {
        unmetDemandPenalty[i] = unmetDemandPenaltyCoeff * ceil(unitProdCost + (setupCost / prodCapacity) +
                                                               (2 * ((transportationCost[nearestWarehouse[i]][i] / vehicleCapacityWarehouse) +
                                                                     (transportationCost[nearestWarehouse[i]][0] / vehicleCapacityPlant))));
    }
}

void ParameterSetting::setProbabilities()
{
    probability.resize(numScenarios, 1.0 / numScenarios);
}

void ParameterSetting::printParameters() const
{
    cout << "Instance: " << instance << endl;
    cout << "Number of Warehouses (W): " << numWarehouses << endl;
    cout << "Number of Retailers (R): " << numRetailers << endl;
    cout << "Number of Periods (T): " << numPeriods << endl;
    cout << "Fleet Size (Plant) (K_plant): " << numVehiclesPlant << endl;
    cout << "Fleet Capacity (Plant) (Q_plant): " << vehicleCapacityPlant << endl;
    cout << "Fleet Size (Each Warehouse) (K_warehouse): " << numVehiclesWarehouse << endl;
    cout << "Fleet Capacity (Warehouse) (Q_warehouse): " << vehicleCapacityWarehouse << endl;
    cout << "Number of Scenarios (S): " << numScenarios << endl;
    cout << "Penalty Coefficient for Unit of Unmet Demand (alpha): " << unmetDemandPenaltyCoeff << endl;
    cout << "Probability Function: " << probabilityFunction << endl;

    cout << "\nPlant parameters: " << endl;
    cout << "x_coord = " << coordX[0] << ", y_coord = " << coordY[0] << endl;
    cout << "production_capacity = " << prodCapacity << ", storage_capacity = " << storageCapacity[0] << ", initial_inventory = " << initialInventory[0] << endl;
    cout << "setup_cost = " << setupCost << ", unit_production_cost = " << unitProdCost << ", unit_holding_cost = " << unitHoldingCost[0] << endl;

    cout << "\nWarehouses parameters: " << endl;
    for (int i = 1; i <= numWarehouses; ++i)
    {
        cout << "x_coord = " << coordX[i] << ", y_coord = " << coordY[i] << endl;
        cout << "storage_capacity = " << storageCapacity[i] << ", initial_inventory = " << initialInventory[i] << ", unit_holding_cost = " << unitHoldingCost[i] << endl;
    }

    cout << "\nRetailers parameters: " << endl;
    for (int i = numWarehouses + 1; i < numNodes; ++i)
    {
        cout << "x_coord = " << coordX[i] << ", y_coord = " << coordY[i] << endl;
        cout << "inventory_capacity = " << storageCapacity[i] << ", initial_inventory = " << initialInventory[i] << ", unit_holding_cost = " << unitHoldingCost[i] << endl;
        cout << "nominal_demand = " << consumeRate[i] << endl;
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
    catch (const exception &e)
    {
        cerr << "Error in monteCarloSimulation: " << e.what() << endl;
        return false;
    }
    return true;
}

void ParameterSetting::generateScenarioDemands(int scenario, int period)
{
    for (int i = numWarehouses + 1; i < numNodes; ++i)
    {
        double demand = 0.0;
        unsigned int seed = i + (period * numRetailers) + (scenario * numPeriods * numRetailers);

        if (probabilityFunction == "Uniform")
        {
            double demand_range_min = (1 - uncertaintyRange) * consumeRate[numWarehouses + 1];
            double demand_range_max = (1 + uncertaintyRange) * consumeRate[numWarehouses + 1];
            demand = round(uniformDistribution(demand_range_min, demand_range_max, seed));
        }
        else if (probabilityFunction == "Normal")
        {
            double uniformSD = sqrt(2 * uncertaintyRange * consumeRate[numWarehouses + 1] * consumeRate[numWarehouses + 1] / 12);
            demand = round(normalDistribution(consumeRate[i], uniformSD, seed));
        }
        else if (probabilityFunction == "Gamma")
        {
            double uniformSD = sqrt((pow((2 * uncertaintyRange * consumeRate[i]), 2)) / 12);
            demand = round(gammaDistribution(consumeRate[i], uniformSD, seed));
        }

        if (demand < 0.01 || isnan(demand))
        {
            demand = 0.0;
        }
        this->demand[i][period][scenario] = demand;
    }
}

void ParameterSetting::saveInstance()
{
    string file_path = "../instances2eprp/" + instance + "_W" + to_string(numWarehouses) +
                       "_R" + to_string(numRetailers) + "_T" + to_string(numPeriods) +
                       "_KP" + to_string(numVehiclesPlant) + "_KW" + to_string(numVehiclesWarehouse) + ".txt";
    try
    {
        ofstream file(file_path);
        if (file.is_open())
        {
            file << "ORIGINAL INSTANCE: " << instance << "\n";
            file << "THIS IS AN INSTANCE FOR THE TWO ECHELON PRODUCTION ROUTING PROBLEM\n";
            file << "THIS INSTANCE INCLUDES ONE PRODUCTION PLANT, " << numWarehouses << " WAREHOUSES, AND " << numRetailers << " RETAILERS.\n";
            file << "THIS INSTANCE HAS " << numPeriods << " PERIODS, " << numVehiclesPlant << " VEHICLES AT THE PRODUCTION PLANT, AND " << numVehiclesWarehouse << " VEHICLES AT EACH WAREHOUSE.\n";
            file << "PLANT VEHICLE CAPACITY: " << setprecision(1) << fixed << vehicleCapacityPlant << "\n";
            file << "WAREHOUSE VEHICLE CAPACITY: " << setprecision(1) << fixed << vehicleCapacityWarehouse << "\n";

            file << "PRODUCTION_PLANT, COORD_X, COORD_Y, INITIAL_INVENTORY, UNIT_HOLDING_COST, INVENTORY_CAPACITY, UNIT_PRODUCTION_COST, SETUP_COST, PRODUCTION_CAPACITY\n";
            file << "0, " << setprecision(1) << fixed << coordX[0] << ", " << coordY[0] << ", " << initialInventory[0] << ", " << unitHoldingCost[0] << ", " << storageCapacity[0] << ", " << unitProdCost << ", " << setupCost << ", " << prodCapacity << "\n";

            file << "WAREHOUSE, COORD_X, COORD_Y, INITIAL_INVENTORY, UNIT_HOLDING_COST, INVENTORY_CAPACITY\n";
            for (int ind = 1; ind <= numWarehouses; ++ind)
            {
                file << ind << ", " << setprecision(1) << fixed << coordX[ind] << ", " << coordY[ind] << ", " << initialInventory[ind] << ", " << unitHoldingCost[ind] << ", " << storageCapacity[ind] << "\n";
            }

            file << "RETAILER, COORD_X, COORD_Y, INITIAL_INVENTORY, UNIT_HOLDING_COST, INVENTORY_CAPACITY, DEMAND_AVG\n";
            for (int ind = numWarehouses + 1; ind <= numWarehouses + numRetailers; ++ind)
            {
                file << ind << ", " << setprecision(1) << fixed << coordX[ind] << ", " << coordY[ind] << ", " << initialInventory[ind] << ", " << unitHoldingCost[ind] << ", " << storageCapacity[ind] << ", " << consumeRate[ind] << "\n";
            }

            cout << "Instance Data Was Saved Successfully at: " << file_path << endl;
            file.close();
        }
        else
        {
            throw runtime_error("Unable to open file: " + file_path);
        }
    }
    catch (const exception &e)
    {
        cerr << "Error occurred while saving instance data: " << e.what() << endl;
    }
}

double ParameterSetting::uniformDistribution(double min, double max, unsigned long int seed)
{
    if (min > max)
    {
        throw invalid_argument("Minimum value cannot be greater than maximum value.");
    }

    // Seed the random number engine with a fixed value
    mt19937 gen(seed);

    // Create a uniform integer distribution
    uniform_real_distribution<> dis(min, max);

    return dis(gen);
}

double ParameterSetting::normalDistribution(double mean, double sd, unsigned int seed)
{
    // Seed the random number engine with a fixed value
    mt19937 gen(seed);

    // Create a normal distribution
    normal_distribution<double> dis(mean, sd);

    return dis(gen);
}

double ParameterSetting::gammaDistribution(double mean, double sd, unsigned int seed)
{
    // Calculate the shape and scale parameters based on mean and standard deviation
    double shape = (mean * mean) / (sd * sd);
    double scale = (sd * sd) / mean;

    // Seed the random number engine with a fixed value
    mt19937 gen(seed);

    // Create a gamma distribution
    gamma_distribution<double> dis(shape, scale);

    return dis(gen);
}

void ParameterSetting::sort_warehouses_by_distance()
{
    try
    {
        sorted_warehouses_by_distance.resize(numNodes);

        for (int retailer_id = numWarehouses + 1; retailer_id < numNodes; ++retailer_id)
        {
            pair<double, double> retailer_coords = {coordX[retailer_id], coordY[retailer_id]};
            vector<pair<int, double>> distances; // Pair of warehouse ID and distance

            for (int ware_id = 1; ware_id <= numWarehouses; ++ware_id)
            {
                double distance = hypot(retailer_coords.first - coordX[ware_id], retailer_coords.second - coordY[ware_id]);
                distances.emplace_back(ware_id, distance);
            }

            // Sort distances based on the second element (distance)
            sort(distances.begin(), distances.end(), [](const pair<int, double> &a, const pair<int, double> &b)
                 { return a.second < b.second; });

            // Extract sorted warehouse IDs
            vector<int> sorted_warehouses;
            for (const auto &pair : distances)
            {
                sorted_warehouses.push_back(pair.first);
            }
            sorted_warehouses_by_distance[retailer_id] = sorted_warehouses;
        }
    }
    catch (const exception &e)
    {
        cerr << "Error occurred while ordering warehouses by distance: " << e.what() << endl;
        throw;
    }
}

vector<vector<int>> ParameterSetting::assign_retailers_to_warehouse()
{
    try
    {
        retailers_assigned_to_warehouse.resize(numNodes);

        vector<double> remainingStorageCapacityWarehouse = storageCapacity;

        double vehicleCapacityAdjustmentCoeff = 0.9;
        vector<double> remainingVehicleCapacityWarehouse(numNodes, 0.0);
        for (int warehouse_id = 1; warehouse_id <= numWarehouses; ++warehouse_id)
        {
            remainingVehicleCapacityWarehouse[warehouse_id] = vehicleCapacityAdjustmentCoeff * vehicleCapacityWarehouse * numVehiclesWarehouse;
        }

        for (int retailer_id = numWarehouses + 1; retailer_id < numNodes; ++retailer_id)
        {
            for (int warehouse_id : sorted_warehouses_by_distance[retailer_id])
            {
                if (remainingStorageCapacityWarehouse[warehouse_id] - consumeRate[retailer_id] >= 0)
                {
                    if (remainingVehicleCapacityWarehouse[warehouse_id] - consumeRate[retailer_id] >= 0)
                    {
                        remainingStorageCapacityWarehouse[warehouse_id] -= consumeRate[retailer_id];
                        remainingVehicleCapacityWarehouse[warehouse_id] -= consumeRate[retailer_id];
                        retailers_assigned_to_warehouse[warehouse_id].push_back(retailer_id);
                        break;
                    }
                }
            }
        }

        cout << "Retailers assigned to warehouses successfully" << endl;
    }
    catch (const exception &e)
    {
        cerr << "Error occurred while assigning retailers to warehouses: " << e.what() << endl;
    }
}

void ParameterSetting::generateAllRoutes() {
    try {
        int totalSubsets = pow(2, numWarehouses);
        for (int i = 1; i < totalSubsets; ++i) {
            vector<int> subset(numWarehouses + 2, 0);
            subset[0] = 1;
            subset[numWarehouses + 1] = 1;
            for (int j = 1; j < numWarehouses; ++j) {
                if (i & (1 << j)) {
                    subset[j + 1] = 1;
                }
            }
            routeMatrix.push_back(subset);
        }


        for (const auto& route : routeMatrix) {
            for (int value : route) {
                cout << value << " ";
            }
            cout << endl;
        }

        solveTSPForRoutes();
    }
    catch (const exception& e) {
        cerr << "Error occurred while generating all routes: " << e.what() << endl;
    }
}

void ParameterSetting::solveTSPForRoutes() {
    try {
        for (const auto& route : routeMatrix) {
            // Count the number of 1s in the route
            int numOnes = count(route.begin(), route.end(), 1);
            if (numOnes < 3) {
                routeCosts.push_back(0.0);
                continue;
            }
            else if (numOnes == 3) {
                double routeCost = 0.0;
                vector<int> optRoute = {0};

                for (int i = 1; i <= numWarehouses; ++i) {
                    if (route[i] == 1) {
                        optRoute.push_back(i);
                        routeCost += 2 * transportationCost[0][i];
                        break;
                    }
                }
                optRoute.push_back(0);
                optimalRoutes.push_back(optRoute);
                routeCosts.push_back(routeCost);
            }   
            else if (numOnes > 3) {
                // Extract the indices of nodes to visit
                vector<int> nodesToVisit;
                for (int i = 0; i <= numWarehouses; ++i) {
                    if (route[i] == 1) {
                        nodesToVisit.push_back(i);
                    }
                }

                // Create a reduced cost matrix
                int size = nodesToVisit.size();
                vector<vector<double>> reducedCostMatrix(size, vector<int>(size));
                for (int i = 0; i < size; ++i) {
                    for (int j = 0; j < size; ++j) {
                        reducedCostMatrix[i][j] = transportationCost[nodesToVisit[i]][nodesToVisit[j]];
                    }
                }

                // Solve the TSP for the reduced cost matrix
                TSP tsp(reducedCostMatrix);
                bool status = tsp.solve();
                if (!status) {
                    throw runtime_error("Failed to solve TSP");
                }

                vector<int> tspRoute = tsp.getRouteTSP();

                // Map the TSP solution back to the original nodes
                vector<int> mappedSolution;
                for (int index : tspRoute) {
                    mappedSolution.push_back(nodesToVisit[index]);
                }

                optimalRoutes.push_back(mappedSolution);
                routeCosts.push_back(tsp.getResult().objValue);

                // Print the mapped solution
                cout << "TSP solution for route: ";
                for (int node : mappedSolution) {
                    cout << node << " ";
                }
                cout << endl;
            }
        }
    }
    catch (const exception& e) {
        cerr << "Error occurred while solving TSP for routes: " << e.what() << endl;
    }
}
 

