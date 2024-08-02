#include "ParameterSetting.h"

ParameterSetting::ParameterSetting(int argc, char *argv[])
    : inputFile(argv[2]),
      numWarehouses(std::stoi(argv[3])),
      numRetailers(std::stoi(argv[4])),
      numPeriods(std::stoi(argv[5])),
      numVehicles_Plant(std::stoi(argv[6])),
      numVehicles_Warehouse(std::stoi(argv[7])),
      numScenarios(std::stoi(argv[8])),
      unmetDemandPenaltyCoeff(std::stod(argv[9])),
      uncertaintyRange(std::stod(argv[10])),
      probabilityFunction(argv[11]),
      instance(argv[12]),
      probability(numScenarios, 0.0),
      numNodes_Total(numWarehouses + numRetailers + 1),
      numNodes_FirstEchelon(numWarehouses + 1),
      numEdges_FirstEchelon((numNodes_FirstEchelon * (numNodes_FirstEchelon - 1)) / 2),
      numNodes_SecondEchelon(numRetailers + numWarehouses),
      numEdges_SecondEchelon((numNodes_SecondEchelon * (numNodes_SecondEchelon - 1)) / 2),
      coordX_Plant(0),
      coordY_Plant(0),
      coordX_Warehouse(numWarehouses, 0),
      coordY_Warehouse(numWarehouses, 0),
      coordX_Retailer(numRetailers, 0),
      coordY_Retailer(numRetailers, 0),
      unitHoldingCost_Plant(0),
      unitHoldingCost_Warehouse(numWarehouses, 0),
      unitHoldingCost_Retailer(numRetailers, 0),
      storageCapacity_Plant(0),
      storageCapacity_Warehouse(numWarehouses, 0),
      storageCapacity_Retailer(numRetailers, 0),
      initialInventory_Plant(0),
      initialInventory_Warehouse(numWarehouses, 0),
      initialInventory_Retailer(numRetailers, 0),
      unmetDemandPenalty(numRetailers, std::numeric_limits<double>::infinity()),
      consumeRate(numRetailers, 0),
      demand(numRetailers, vector<vector<double>>(numPeriods, vector<double>(numScenarios, 0))),
      index_i_FirstEchelon(numEdges_FirstEchelon, 0),
      index_j_FirstEchelon(numEdges_FirstEchelon, 0),
      index_e_FirstEchelon(numNodes_FirstEchelon, vector<int>(numNodes_FirstEchelon, 0)),
      transportationCost_FirstEchelon(numNodes_FirstEchelon, vector<double>(numNodes_FirstEchelon, 0)),
      index_i_SecondEchelon(numEdges_SecondEchelon, 0),
      index_j_SecondEchelon(numEdges_SecondEchelon, 0),
      index_e_SecondEchelon(numNodes_SecondEchelon, vector<int>(numNodes_SecondEchelon, 0)),
      transportationCost_SecondEchelon(numNodes_SecondEchelon, vector<double>(numNodes_SecondEchelon, 0)),
      DeliveryUB_perRetailer(numRetailers, vector<vector<double>>(numPeriods, vector<double>(numScenarios, 0.))),
      DeliveryUB(numPeriods, vector<double>(numScenarios, 0.))
{
    initializeIndices();
}

void ParameterSetting::initializeIndices()
{
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
}

bool ParameterSetting::setParameters()
{
    try
    {
        cout << "\nSet Parameters For The Stochastic Two-Echelon Production Routing Problem With Adaptive Routing: " << endl;

        // Read Data From File
        cout << "Read Data From File: " << endl;
        if (!readDataFromFile())
        {
            throw std::runtime_error("Unable to Read Data From File");
        }

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
        adjustInitialInventory();
        calculateProductionCapacity();
        setWarehouseParameters();
        calculateVehicleCapacities();
        calculateTransportationCost_FirstEchelon();
        calculateTransportationCost_SecondEchelon();
        calculateDeliveryUB();

        sort_warehouses_by_distance();
        calculateUnmetDemandPenalty();
        setProbabilities();

        assign_retailers_to_warehouse();

        printParameters();
        saveInstance();

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

        readOriginalDatasetInfo(file);

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

void ParameterSetting::readOriginalDatasetInfo(std::ifstream &file)
{
    string line;
    int i = 0;
    int j = 0;
    while (std::getline(file, line) && j < numRetailers)
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
        if (i == 13)
        {
            sscanf(line.c_str(), "%*d %lf %lf %lf %lf %lf %lf",
                   &coordX_Plant, &coordY_Plant, &initialInventory_Plant,
                   &unitHoldingCost_Plant, &unitProdCost, &setupCost);
        }
        else if (i >= 15)
        {
            sscanf(line.c_str(), "%*d %lf %lf %lf %lf %*f %lf %lf",
                   &coordX_Retailer[j], &coordY_Retailer[j], &initialInventory_Retailer[j],
                   &storageCapacity_Retailer[j], &consumeRate[j], &unitHoldingCost_Retailer[j]);
            j++;
        }
        i++;
    }
}

void ParameterSetting::checkDemandsDistribution() const
{
    cout << "Check the demands distribution: " << endl;
    double sum_demands_avg = numPeriods * std::accumulate(consumeRate.begin(), consumeRate.end(), 0.0);
    cout << "Sum of demands in all periods (population): " << sum_demands_avg << endl;

    double sum_demands = 0.0;
    for (const auto &dem_retailer : demand)
    {
        for (const auto &dem_period : dem_retailer)
        {
            sum_demands += 1. / numScenarios * std::accumulate(dem_period.begin(), dem_period.end(), 0.0);
        }
    }
    cout << "Sum of demands in all periods (sample): " << sum_demands << endl;
}

void ParameterSetting::adjustInitialInventory()
{
    if (numPeriods == 3)
    {
        initialInventory_Plant = std::floor(initialInventory_Plant / 2);

        for (int i = 0; i < numWarehouses; ++i)
        {
            initialInventory_Warehouse[i] = std::floor(initialInventory_Warehouse[i] / 2);
        }

        for (int i = 0; i < numRetailers; ++i)
        {
            initialInventory_Retailer[i] = std::floor(initialInventory_Retailer[i] / 2);
        }
    }
}

void ParameterSetting::calculateProductionCapacity()
{
    double totalConsumption = 0.0;
    for (int t = 0; t < numPeriods; ++t)
    {
        for (int i = 0; i < numRetailers; ++i)
        {
            totalConsumption += consumeRate[i];
        }
    }
    prodCapacity = std::floor(2 * (totalConsumption / numPeriods));
    storageCapacity_Plant = prodCapacity / 2;
}

void ParameterSetting::setWarehouseParameters()
{
    double uhc_min = std::min(unitHoldingCost_Plant, *std::min_element(unitHoldingCost_Retailer.begin(), unitHoldingCost_Retailer.end()));
    double uhc_max = *std::min_element(unitHoldingCost_Retailer.begin(), unitHoldingCost_Retailer.end());

    std::random_device rd;
    for (int i = 0; i < numWarehouses; ++i)
    {
        int seed = i + 1;
        std::mt19937 gen1(int(1e9 + (seed * seed)));
        std::mt19937 gen2(int(1e6 + (seed * seed)));
        std::mt19937 gen3(int(1e3 + (seed * seed)));

        std::uniform_int_distribution<> dis1(0, 2500);
        // std::uniform_int_distribution<> dis2(storageCapacity_Plant / 2, storageCapacity_Plant);
        std::uniform_int_distribution<> dis3(uhc_min, uhc_max);

        coordX_Warehouse[i] = std::floor(dis1(gen1));
        coordY_Warehouse[i] = std::floor(dis1(gen2));
        initialInventory_Warehouse[i] = 0;
        // storageCapacity_Warehouse[i] = std::floor(dis2(gen3));
        storageCapacity_Warehouse[i] = storageCapacity_Plant;
        unitHoldingCost_Warehouse[i] = std::floor(dis3(gen3));
    }
}

void ParameterSetting::calculateVehicleCapacities()
{
    double maxofInvCap_Warehouse = *std::max_element(storageCapacity_Warehouse.begin(), storageCapacity_Warehouse.end());
    double VehicleCapacityCoefficient_Plant = 2.5 * (std::floor(numWarehouses / 10) + 1);
    vehicleCapacity_Plant = std::floor(VehicleCapacityCoefficient_Plant * (maxofInvCap_Warehouse / numVehicles_Plant));

    double maxofInvCap_Retailer = *std::max_element(storageCapacity_Retailer.begin(), storageCapacity_Retailer.end());
    double VehicleCapacityCoefficient_Warehouse = 1.5 * (std::floor(numRetailers / (10 * numWarehouses)) + 1);
    vehicleCapacity_Warehouse = std::floor(VehicleCapacityCoefficient_Warehouse * (maxofInvCap_Retailer / numVehicles_Warehouse));
}

void ParameterSetting::updateStorageCapacityForRetailers()
{
    for (int i = 0; i < numRetailers; ++i)
    {
        storageCapacity_Retailer[i] += consumeRate[i];
    }
}

void ParameterSetting::calculateTransportationCost_FirstEchelon()
{
    for (int i = 0; i < numNodes_FirstEchelon - 1; ++i)
    {
        for (int j = i + 1; j < numNodes_FirstEchelon; ++j)
        {
            if (i == 0)
            {
                transportationCost_FirstEchelon[i][j] = transportationCost_FirstEchelon[j][i] = std::hypot(coordX_Plant - coordX_Warehouse[j], coordY_Plant - coordY_Warehouse[j]);
            }
            else
            {
                transportationCost_FirstEchelon[i][j] = transportationCost_FirstEchelon[j][i] = std::hypot(coordX_Warehouse[i] - coordX_Warehouse[j], coordY_Warehouse[i] - coordY_Warehouse[j]);
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
                transportationCost_SecondEchelon[i][j] = transportationCost_SecondEchelon[j][i] = std::hypot(coordX_Warehouse[i] - coordX_Warehouse[j], coordY_Warehouse[i] - coordY_Warehouse[j]);
            }
            else if (i < numWarehouses && j >= numWarehouses)
            {
                transportationCost_SecondEchelon[i][j] = transportationCost_SecondEchelon[j][i] = std::hypot(coordX_Warehouse[i] - coordX_Retailer[j], coordY_Warehouse[i] - coordY_Retailer[j]);
            }
            else
            {
                transportationCost_SecondEchelon[i][j] = transportationCost_SecondEchelon[j][i] = std::hypot(coordX_Retailer[i] - coordX_Retailer[j], coordY_Retailer[i] - coordY_Retailer[j]);
            }
            // cout << "Cost_SecondE[" << i << "][" << j << "] = " << transportationCost_SecondEchelon[i][j] << endl;
        }
    }
}

void ParameterSetting::calculateUnmetDemandPenalty()
{
    for (int i = 0; i < numRetailers; ++i)
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
            double remainingDemandAllRetailers = 0.0;
            for (int i = 0; i < numRetailers; ++i)
            {
                double remainingDemand = 0.0;
                for (int l = t; l < numPeriods; ++l)
                {
                    remainingDemand += demand[i][l][s];
                    remainingDemandAllRetailers += demand[i][l][s];
                }
                DeliveryUB_perRetailer[i][t][s] = std::min({remainingDemand, vehicleCapacity_Warehouse, storageCapacity_Retailer[i]});
            }
            DeliveryUB[t][s] = remainingDemandAllRetailers;
        }
    }
}

void ParameterSetting::printParameters() const
{
    cout << "Instance: " << instance << endl;
    cout << "Number of Warehouses (W): " << numWarehouses << endl;
    cout << "Number of Retailers (R): " << numRetailers << endl;
    cout << "Number of Periods (T): " << numPeriods << endl;
    cout << "Fleet Size (Plant) (K_plant): " << numVehicles_Plant << endl;
    cout << "Fleet Capacity (Plant) (Q_plant): " << vehicleCapacity_Plant << endl;
    cout << "Fleet Size (Each Warehouse) (K_warehouse): " << numVehicles_Warehouse << endl;
    cout << "Fleet Capacity (Warehouse) (Q_warehouse): " << vehicleCapacity_Warehouse << endl;
    cout << "Number of Scenarios (S): " << numScenarios << endl;
    cout << "Penalty Coefficient for Unit of Unmet Demand (alpha): " << unmetDemandPenaltyCoeff << endl;
    cout << "Probability Function: " << probabilityFunction << endl;

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

    cout << "\nRetailers parameters: " << endl;
    for (int i = 0; i < numRetailers; ++i)
    {
        cout << "x_coord = " << coordX_Retailer[i] << ", y_coord = " << coordY_Retailer[i] << endl;
        cout << "inventory_capacity = " << storageCapacity_Retailer[i] << ", initial_inventory = " << initialInventory_Retailer[i] << ", unit_holding_cost = " << unitHoldingCost_Retailer[i] << endl;
        cout << "nominal_demand = " << consumeRate[i] << endl;
    }
}

void ParameterSetting::saveInstance()
{
    string file_path = "../instances2eprp/" + instance + "_W" + std::to_string(numWarehouses) +
                       "_R" + std::to_string(numRetailers) + "_T" + std::to_string(numPeriods) +
                       "_KP" + std::to_string(numVehicles_Plant) + "_KW" + std::to_string(numVehicles_Warehouse) + ".txt";
    try
    {
        std::ofstream file(file_path);
        if (file.is_open())
        {
            file << "ORIGINAL INSTANCE: " << instance << "\n";
            file << "THIS IS AN INSTANCE FOR THE TWO ECHELON PRODUCTION ROUTING PROBLEM\n";
            file << "THIS INSTANCE INCLUDES ONE PRODUCTION PLANT, " << numWarehouses << " WAREHOUSES, AND " << numRetailers << " RETAILERS.\n";
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
            for (int ind = 0; ind < numRetailers; ++ind)
            {
                file << ind + 1 + numWarehouses << ", " << std::setprecision(1) << std::fixed << coordX_Retailer[ind] << ", " << coordY_Retailer[ind] << ", " << initialInventory_Retailer[ind] << ", " << unitHoldingCost_Retailer[ind] << ", " << storageCapacity_Retailer[ind] << ", " << consumeRate[ind] << "\n";
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
    for (int i = 0; i < numRetailers; ++i)
    {
        double demand = 0.0;
        unsigned int seed = i + (period * numRetailers) + (scenario * numPeriods * numRetailers);

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

        // cout << "Demand for retailer " << i << ", Period " << period << ", Scenario " << scenario << " = " << demand << endl;
    }
}

double ParameterSetting::uniformDistribution(double min, double max, unsigned long int seed)
{
    if (min > max)
    {
        throw std::invalid_argument("Minimum value cannot be greater than maximum value.");
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
        sorted_warehouses_by_distance.resize(numRetailers);

        for (int retInd = 0; retInd < numRetailers; ++retInd)
        {
            vector<std::pair<int, double>> distances; // Pair of warehouse ID and distance

            for (int w = 0; w < numWarehouses; ++w)
            {
                double distance = std::hypot(coordX_Retailer[retInd] - coordX_Warehouse[w], coordY_Retailer[retInd] - coordY_Warehouse[w]);
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
            sorted_warehouses_by_distance[retInd] = sorted_warehouses;

            // cout << "Sorted warehouses by distance for retailer " << retInd << ": [";
            // for (int w = 0; w < numWarehouses; ++w)
            // {
            //     cout << sorted_warehouses_by_distance[retInd][w] << " ";
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

void ParameterSetting::assign_retailers_to_warehouse()
{
    try
    {
        retailers_assigned_to_warehouse.resize(numWarehouses);
        warehouse_assigned_to_retailer.resize(numRetailers, -1);

        vector<double> remainingStorageCapacityWarehouse(storageCapacity_Warehouse.begin(), storageCapacity_Warehouse.end());

        double lambda = 1.0;
        // if (numVehicles_Warehouse != 1)
        // {
        //     lambda = 0.9;
        // }

        vector<double> remainingVehicleCapacityWarehouse(numWarehouses, lambda * numVehicles_Warehouse * vehicleCapacity_Warehouse);

        int retInd = 0;
        while (retInd < numRetailers)
        {
            bool retailer_assigned = false;

            for (int w : sorted_warehouses_by_distance[retInd])
            {
                if (retailer_assigned)
                    break;

                if (remainingStorageCapacityWarehouse[w] - consumeRate[retInd] >= 0.0 && remainingVehicleCapacityWarehouse[w] - consumeRate[retInd] >= 0.0)
                {
                    remainingStorageCapacityWarehouse[w] -= consumeRate[retInd];
                    remainingVehicleCapacityWarehouse[w] -= consumeRate[retInd];
                    retailers_assigned_to_warehouse[w].push_back(retInd);
                    warehouse_assigned_to_retailer[retInd] = w;
                    retailer_assigned = true;
                    retInd++;
                    break;
                }
            }
        }

        for (int w = 0; w < numWarehouses; ++w) {
            cout << "Retailers assigned to warehouse " << w << ": [";
            bool first = true;
            for (int retInd : retailers_assigned_to_warehouse[w]) {
                if (!first) {
                    cout << " ";
                }
                cout << retInd;
                first = false;
            }
            cout << "]" << endl;
        }

        cout << "Retailers assigned to warehouses successfully" << endl;
    }
    catch (const std::exception &e)
    {
        cerr << "Error occurred while assigning retailers to warehouses: " << e.what() << endl;
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

        for (const auto &route : routeMatrix)
        {
            for (int value : route)
            {
                cout << value << " ";
            }
            cout << endl;
        }

        solveTSPForRoutes();

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
            cout << "route size:" << numOnes - 1 << endl;

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
                cout << "route cost:" << routeCost << endl;
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
                routeCosts.push_back(tsp.getResult().objValue);

                // Print the mapped solution
                cout << "TSP solution for route: ";
                for (int node : mappedSolution)
                {
                    cout << node << " ";
                }
                cout << endl;
                cout << "route cost:" << tsp.getResult().objValue << endl;
                cout << "\n";
            }
        }
    }
    catch (const std::exception &e)
    {
        cerr << "Error occurred while solving TSP for routes: " << e.what() << endl;
    }
}

// Getters
vector<vector<int>> ParameterSetting::getSortedWarehousesByDistance() const
{
    return sorted_warehouses_by_distance;
}

vector<vector<int>> ParameterSetting::getRetailersAssignedToWarehouse() const
{
    return retailers_assigned_to_warehouse;
}

vector<int> ParameterSetting::getWarehouseAssignedToRetailer() const
{
    return warehouse_assigned_to_retailer;
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
