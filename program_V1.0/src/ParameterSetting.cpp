#include "ParameterSetting.h"

ParameterSetting::ParameterSetting(int argc, char *argv[])
    : inputFile(argv[2]),
      numWarehouses(std::stoi(argv[3])),
      numCustomers(std::stoi(argv[4])),
      numPeriods(std::stoi(argv[5])),
      numVehicles_Plant(std::stoi(argv[6])),
      numVehicles_Warehouse(std::stoi(argv[7])),
      numScenarios(std::stoi(argv[8])),
      unmetDemandPenaltyCoeff(std::stod(argv[9])),
      uncertaintyRange(std::stod(argv[10])),
      probabilityFunction(argv[11]),
      instance(argv[12]),
      probability(numScenarios, 0.0),
      numNodes_Total(numWarehouses + numCustomers + 1),
      numNodes_FirstEchelon(numWarehouses + 1),
      numEdges_FirstEchelon((numNodes_FirstEchelon * (numNodes_FirstEchelon - 1)) / 2),
      numNodes_SecondEchelon(numCustomers + numWarehouses),
      numEdges_SecondEchelon((numNodes_SecondEchelon * (numNodes_SecondEchelon - 1)) / 2),
      numVehicles_SecondEchelon(numWarehouses * numVehicles_Warehouse),
      coordX_Plant(0),
      coordY_Plant(0),
      coordX_Warehouse(numWarehouses, 0),
      coordY_Warehouse(numWarehouses, 0),
      coordX_Customer(numCustomers, 0),
      coordY_Customer(numCustomers, 0),
      unitHoldingCost_Plant(0),
      unitHoldingCost_Warehouse(numWarehouses, 0),
      unitHoldingCost_Customer(numCustomers, 0),
      storageCapacity_Plant(0),
      storageCapacity_Warehouse(numWarehouses, 0),
      storageCapacity_Customer(numCustomers, 0),
      initialInventory_Plant(0),
      initialInventory_Warehouse(numWarehouses, 0),
      initialInventory_Customer(numCustomers, 0),
      unmetDemandPenalty(numCustomers, std::numeric_limits<double>::infinity()),
      consumeRate(numCustomers, 0),
      demand(numCustomers, vector<vector<double>>(numPeriods, vector<double>(numScenarios, 0))),
      index_i_FirstEchelon(numEdges_FirstEchelon, 0),
      index_j_FirstEchelon(numEdges_FirstEchelon, 0),
      index_e_FirstEchelon(numNodes_FirstEchelon, vector<int>(numNodes_FirstEchelon, 0)),
      transportationCost_FirstEchelon(numNodes_FirstEchelon, vector<double>(numNodes_FirstEchelon, 0)),
      index_i_SecondEchelon(numEdges_SecondEchelon, 0),
      index_j_SecondEchelon(numEdges_SecondEchelon, 0),
      index_e_SecondEchelon(numNodes_SecondEchelon, vector<int>(numNodes_SecondEchelon, 0)),
      transportationCost_SecondEchelon(numNodes_SecondEchelon, vector<double>(numNodes_SecondEchelon, 0)),
      DeliveryUB_perCustomer(numCustomers, vector<vector<double>>(numPeriods, vector<double>(numScenarios, 0.))),
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

    set_WarehouseVehicles.resize(numWarehouses, vector<int>(numVehicles_Warehouse));
    for (int w = 0; w < numWarehouses; ++w)
    {
        std::iota(set_WarehouseVehicles[w].begin(), set_WarehouseVehicles[w].end(), w * numVehicles_Warehouse);
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

        assign_customers_to_warehouse();

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
    while (std::getline(file, line) && j < numCustomers)
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
                   &coordX_Customer[j], &coordY_Customer[j], &initialInventory_Customer[j],
                   &storageCapacity_Customer[j], &consumeRate[j], &unitHoldingCost_Customer[j]);
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
    for (const auto &dem_customer : demand)
    {
        for (const auto &dem_period : dem_customer)
        {
            sum_demands += 1. / numScenarios * std::accumulate(dem_period.begin(), dem_period.end(), 0.0);
        }
    }
    cout << "Sum of demands in all periods (sample): " << sum_demands << endl;
}

void ParameterSetting::adjustInitialInventory()
{
    if (numPeriods == 3 || numPeriods == 6)
    {
        initialInventory_Plant = std::floor(initialInventory_Plant / 2);

        for (int i = 0; i < numWarehouses; ++i)
        {
            initialInventory_Warehouse[i] = std::floor(initialInventory_Warehouse[i] / 2.0);
        }

        for (int i = 0; i < numCustomers; ++i)
        {
            initialInventory_Customer[i] = std::floor(initialInventory_Customer[i] / 2.0);
        }
    }
}

void ParameterSetting::calculateProductionCapacity()
{
    double totalConsumption = 0.0;
    for (int t = 0; t < numPeriods; ++t)
    {
        for (int i = 0; i < numCustomers; ++i)
        {
            totalConsumption += consumeRate[i];
        }
    }
    prodCapacity = std::floor(2 * (totalConsumption / numPeriods));
    storageCapacity_Plant = prodCapacity / 2;
}

void ParameterSetting::setWarehouseParameters()
{
    double uhc_min = std::min(unitHoldingCost_Plant, *std::min_element(unitHoldingCost_Customer.begin(), unitHoldingCost_Customer.end()));
    double uhc_max = *std::min_element(unitHoldingCost_Customer.begin(), unitHoldingCost_Customer.end());

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
        storageCapacity_Warehouse[i] = 0.9 * storageCapacity_Plant;
        unitHoldingCost_Warehouse[i] = std::floor(dis3(gen3));
    }
}

void ParameterSetting::calculateVehicleCapacities()
{
    // double maxofInvCap_Warehouse = *std::max_element(storageCapacity_Warehouse.begin(), storageCapacity_Warehouse.end());
    // double VehicleCapacityCoefficient_Plant = 2.5 * (std::floor(numWarehouses / 10) + 1);
    // vehicleCapacity_Plant = std::floor(VehicleCapacityCoefficient_Plant * (maxofInvCap_Warehouse / numVehicles_Plant));

    // double maxofInvCap_Customer = *std::max_element(storageCapacity_Customer.begin(), storageCapacity_Customer.end());
    // double VehicleCapacityCoefficient_Warehouse = 1.5 * (std::floor(numCustomers / (10 * numWarehouses)) + 1);
    // vehicleCapacity_Warehouse = std::floor(VehicleCapacityCoefficient_Warehouse * (maxofInvCap_Customer / numVehicles_Warehouse));

    // new
    double maxofInvCap_Warehouse = *std::max_element(storageCapacity_Warehouse.begin(), storageCapacity_Warehouse.end());
    double VehicleCapacityCoefficient_Plant = 1.5;
    vehicleCapacity_Plant = std::floor(VehicleCapacityCoefficient_Plant * (maxofInvCap_Warehouse / numVehicles_Plant));

    double maxofInvCap_Customer = *std::max_element(storageCapacity_Customer.begin(), storageCapacity_Customer.end());
    double VehicleCapacityCoefficient_Warehouse = 1.0 * (std::floor(numCustomers / (10 * numWarehouses)) + 1);
    vehicleCapacity_Warehouse = std::floor(VehicleCapacityCoefficient_Warehouse * (maxofInvCap_Customer / numVehicles_Warehouse));
}

void ParameterSetting::updateStorageCapacityForCustomers()
{
    for (int i = 0; i < numCustomers; ++i)
    {
        storageCapacity_Customer[i] += consumeRate[i];
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
                transportationCost_SecondEchelon[i][j] = transportationCost_SecondEchelon[j][i] = std::hypot(coordX_Warehouse[i] - coordX_Customer[j - numWarehouses], coordY_Warehouse[i] - coordY_Customer[j - numWarehouses]);
            }
            else
            {
                transportationCost_SecondEchelon[i][j] = transportationCost_SecondEchelon[j][i] = std::hypot(coordX_Customer[i - numWarehouses] - coordX_Customer[j - numWarehouses], coordY_Customer[i - numWarehouses] - coordY_Customer[j - numWarehouses]);
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

void ParameterSetting::printParameters() const
{
    cout << "Instance: " << instance << endl;
    cout << "Number of Warehouses (W): " << numWarehouses << endl;
    cout << "Number of Customers (R): " << numCustomers << endl;
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
        customers_assigned_to_warehouse.resize(numScenarios, vector<vector<vector<int>>>(numPeriods, vector<vector<int>>(numWarehouses, vector<int>(numCustomers, -1))));
        warehouse_assigned_to_customer.resize(numScenarios, vector<vector<int>>(numPeriods, vector<int>(numCustomers, -1)));

        for (int s = 0; s < numScenarios; ++s)
        {
            for (int t = 0; t < numPeriods; ++t)
            {
                for (int w = 0; w < numWarehouses; ++w)
                {
                    for (int i = 0; i < numCustomers; ++i)
                    {
                        if (w == sorted_warehouses_by_distance[i][0])
                        {
                            customers_assigned_to_warehouse[s][t][w][i] = 1;
                            warehouse_assigned_to_customer[s][t][i] = w;
                        }
                        else
                        {
                            customers_assigned_to_warehouse[s][t][w][i] = 0;
                        }
                    }
                }
            }
        }

        // int custInd = 0;
        // while (custInd < numCustomers)
        // {
        //     bool customer_assigned = false;

        //     for (int w : sorted_warehouses_by_distance[custInd])
        //     {
        //         if (customer_assigned)
        //             break;

        //         if (remainingStorageCapacityWarehouse[w] - consumeRate[custInd] >= 0.0 && remainingVehicleCapacityWarehouse[w] - consumeRate[custInd] >= 0.0)
        //         {
        //             remainingStorageCapacityWarehouse[w] -= consumeRate[custInd];
        //             remainingVehicleCapacityWarehouse[w] -= consumeRate[custInd];
        //             customers_assigned_to_warehouse[w].push_back(custInd);
        //             warehouse_assigned_to_customer[custInd] = w;
        //             customer_assigned = true;
        //             custInd++;
        //             break;
        //         }
        //     }
        // }

        // for (int s = 0; s < numScenarios; ++s)
        // {
        //     for (int t = 0; t < numPeriods; ++t)
        //     {
        //         for (int w = 0; w < numWarehouses; ++w)
        //         {
        //             cout << "Customers assigned to warehouse " << w << " in scenario " << s << " period " << t << ": [";
        //             for (int i = 0; i < numCustomers; ++i)
        //             {
        //                 if (customers_assigned_to_warehouse[s][t][w][i] == 1)
        //                 {
        //                     cout << i + numWarehouses << " ";
        //                 }
        //             }
        //             cout << "]" << endl;
        //         }

        //         cout << "Customers assigned to warehouses successfully" << endl;
        //     }
        // }
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

        cout << "\n"
             << endl;
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
                routeCosts.push_back(tsp.getResult().objValue);

                // Print the mapped solution
                // cout << "TSP solution for route: ";
                // for (int node : mappedSolution)
                // {
                //     cout << node << " ";
                // }
                // cout << endl;
                // cout << "route cost:" << tsp.getResult().objValue << endl;
                // cout << "\n";
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

Solution ParameterSetting::readSolutionWarmStart()
{
    cout << "Reading Solution For Warm Start..." << endl;
    string resultsFileName = "../Results/Solutions/Hybrid-ILS/" + probabilityFunction + "/Sol_S2EPRPAR_Hybrid-ILS_" + instance + "_PF-" + probabilityFunction + "_W" + std::to_string(numWarehouses) + "_C" + std::to_string(numCustomers) + "_KP" + std::to_string(numVehicles_Plant) + "_KW" + std::to_string(numVehicles_Warehouse) + "_T" + std::to_string(numPeriods) + "_S" + std::to_string(numScenarios) + "_UR" + std::to_string(static_cast<int>(uncertaintyRange * 100)) + "%_PenalyCoeff" + std::to_string(static_cast<int>(unmetDemandPenaltyCoeff)) + ".txt";

    Solution warmstart;
    
    std::ifstream file(resultsFileName);
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
        warmstart.productionSetup.resize(numPeriods);
        warmstart.productionQuantity.resize(numPeriods);
        warmstart.plantInventory.resize(numPeriods);
        warmstart.warehouseInventory.resize(numWarehouses, vector<vector<double>>(numPeriods, vector<double>(numScenarios)));
        warmstart.customerInventory.resize(numCustomers, vector<vector<double>>(numPeriods, vector<double>(numScenarios)));
        warmstart.customerUnmetDemand.resize(numCustomers, vector<vector<double>>(numPeriods, vector<double>(numScenarios)));
        warmstart.deliveryQuantityToWarehouse.resize(numWarehouses, vector<double>(numPeriods));
        warmstart.routesPlantToWarehouse.resize(numPeriods, vector<vector<int>>(numVehicles_Plant, vector<int>()));
        warmstart.deliveryQuantityToCustomer.resize(numCustomers, vector<vector<double>>(numPeriods, vector<double>(numScenarios)));
        warmstart.routesWarehouseToCustomer.resize(numScenarios, vector<vector<vector<vector<int>>>>(numWarehouses, vector<vector<vector<int>>>(numPeriods, vector<vector<int>>(numVehicles_Warehouse, vector<int>()))));
        warmstart.customerAssignmentToWarehouse.resize(numScenarios, vector<vector<vector<int>>>(numPeriods, vector<vector<int>>(numWarehouses, vector<int>(numCustomers))));

        // Read Solutions
        for (int s = 0; s < numScenarios; s++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int w = 0; w < numWarehouses; w++)
                {
                    for (int i = 0; i < numCustomers; i++)
                    {
                        file >> warmstart.customerAssignmentToWarehouse[s][t][w][i];
                    }
                }
            }
        }

        for (int t = 0; t < numPeriods; t++)
        {
            file >> warmstart.productionSetup[t];
        }

        for (int t = 0; t < numPeriods; t++)
        {
            file >> warmstart.productionQuantity[t];
        }

        for (int t = 0; t < numPeriods; t++)
        {
            file >> warmstart.plantInventory[t];
        }

        for (int w = 0; w < numWarehouses; w++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int s = 0; s < numScenarios; s++)
                {
                    file >> warmstart.warehouseInventory[w][t][s];
                }
            }
        }

        for (int i = 0; i < numCustomers; i++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int s = 0; s < numScenarios; s++)
                {
                    file >> warmstart.customerInventory[i][t][s];
                }
            }
        }

        for (int i = 0; i < numCustomers; i++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int s = 0; s < numScenarios; s++)
                {
                    file >> warmstart.customerUnmetDemand[i][t][s];
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
                std::cerr << "Error parsing line: " << line_One << std::endl;
                continue;
            }

            int node;
            std::vector<int> route;
            while (iss >> node)
            {
                route.push_back(node);
            }

            // Assign the route to the appropriate vehicle
            warmstart.routesPlantToWarehouse[t_index][k_index] = route;
        }

        for (int w = 0; w < numWarehouses; w++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                file >> warmstart.deliveryQuantityToWarehouse[w][t];
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
                std::cerr << "Error parsing line: " << line_Two << std::endl;
                continue;
            }

            int node;
            std::vector<int> route;
            while (iss >> node)
            {
                route.push_back(node);
            }

            // Assign the route to the appropriate vehicle
            warmstart.routesWarehouseToCustomer[s_index][w_index][t_index][k_index] = route;
        }

        for (int i = 0; i < numCustomers; i++)
        {
            for (int t = 0; t < numPeriods; t++)
            {
                for (int s = 0; s < numScenarios; s++)
                {
                    file >> warmstart.deliveryQuantityToCustomer[i][t][s];
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
