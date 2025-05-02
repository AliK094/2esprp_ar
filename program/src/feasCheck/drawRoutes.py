import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
import sys

def read_data(file_path, problemType):
    # ----------------------------------------------------------------------------
    # Initialization
    # ----------------------------------------------------------------------------

    # Open the data file and parse values
    with open(file_path, 'r') as fp:
        values = []
        for line in fp:
            ar = line.split()
            for i in ar:
                values.append(i)
    
    # Check problem type for shortage allowance
    shortage_allowed = problemType in {'EV', 'WS', 'EEV'}
    
    index = 0
    # Parse problem parameters
    numNodes_Total = int(values[index])
    index += 1
    numWarehouses = int(values[index])
    index += 1
    numCustomers = int(values[index])
    index += 1
    numVehicles_Plant = int(values[index])
    index += 1
    numVehicles_Warehouse = int(values[index])
    index += 1
    numPeriods = int(values[index])
    index += 1

    if shortage_allowed:
        numScenarios = int(values[index])
        index += 1

    if problemType in {"WS", "EEV"}:
        scenario_Index = int(values[index])
        index += 1

    if shortage_allowed:
        uncertaintyRange = float(values[index])
        index += 1
        probabilityFunction = values[index]
        index += 1
        unmetDemandPenaltyCoeff = float(values[index])
        index += 1

    # Parse production and cost parameters
    unitProdCost = float(values[index])
    index += 1
    setupCost = float(values[index])
    index += 1
    prodCapacity = float(values[index])
    index += 1
    vehicleCapacity_Plant = float(values[index])
    index += 1
    vehicleCapacity_Warehouse = float(values[index])
    index += 1

    coordX_Plant = float(values[index])
    index += 1
    coordY_Plant = float(values[index])
    index += 1

    # Parse coordinates for warehouses and customers
    coordX_Warehouse = np.array([float(values[index + w]) for w in range(numWarehouses)])
    index += numWarehouses
    coordY_Warehouse = np.array([float(values[index + w]) for w in range(numWarehouses)])
    index += numWarehouses

    coordX_Customer = np.array([float(values[index + i]) for i in range(numCustomers)])
    index += numCustomers
    coordY_Customer = np.array([float(values[index + i]) for i in range(numCustomers)])
    index += numCustomers
    
    unitHoldingCost_Plant = float(values[index])
    index += 1
    
    if problemType == "2EPRPCS":
        unitHandlingCost_Satellite = np.zeros((numWarehouses))
        for w in range(numWarehouses):
            unitHandlingCost_Satellite[w] = float(values[index])
            index += 1
    else:
        unitHoldingCost_Warehouse = np.zeros((numWarehouses))
        for w in range(numWarehouses):
            unitHoldingCost_Warehouse[w] = float(values[index])
            index += 1
        
    unitHoldingCost_Customer = np.zeros((numCustomers))
    for i in range(numCustomers):
        unitHoldingCost_Customer[i] = float(values[index])
        index += 1
        
    storageCapacity_Plant = float(values[index])
    index += 1
    
    if problemType != "2EPRPCS":
        storageCapacity_Warehouse = np.zeros((numWarehouses))
        for w in range(numWarehouses):
            storageCapacity_Warehouse[w] = float(values[index])
            index += 1
        
    storageCapacity_Customer = np.zeros((numCustomers))
    for i in range(numCustomers):
        storageCapacity_Customer[i] = float(values[index])
        index += 1
        
    initialInventory_Plant = float(values[index])
    index += 1
    
    if problemType != "2EPRPCS":
        initialInventory_Warehouse = np.zeros((numWarehouses))
        for w in range(numWarehouses):
            initialInventory_Warehouse[w] = float(values[index])
            index += 1
        
    initialInventory_Customer = np.zeros((numCustomers))
    for i in range(numCustomers):
        initialInventory_Customer[i] = float(values[index])
        index += 1
    
    if shortage_allowed:
        unmetDemandPenalty = np.zeros((numCustomers))
        for i in range(numCustomers):
            unmetDemandPenalty[i] = float(values[index])
            index += 1
        
    demand = np.zeros((numCustomers, numPeriods))
    for i in range(numCustomers):
        for t in range(numPeriods):
            demand[i][t] = float(values[index])
            index += 1

    # Parse transportation costs
    transportationCost_FirstEchelon = np.zeros((numWarehouses + 1, numWarehouses + 1))
    for i in range(numWarehouses + 1):
        for j in range(numWarehouses + 1):
            transportationCost_FirstEchelon[i][j] = float(values[index])
            index += 1

    transportationCost_SecondEchelon = np.zeros((numWarehouses + numCustomers, numWarehouses + numCustomers))
    for i in range(numWarehouses + numCustomers):
        for j in range(numWarehouses + numCustomers):
            transportationCost_SecondEchelon[i][j] = float(values[index])
            index += 1
            
    # Solution Values
    customerAssignmentToWarehouse = np.zeros((numPeriods, numWarehouses, numCustomers))
    for t in range(numPeriods):
        for w in range(numWarehouses):
            for i in range(numCustomers):
                customerAssignmentToWarehouse[t][w][i] = int(values[index])
                index += 1

    objValue = 0.0
    
    totalSetupCost = 0.0
    totalProductionCost = 0.0
    totalInventoryCostPlant = 0.0
    totalInventoryCostWarehouse = 0.0
    totalHandlingCostSatellite = 0.0
    totalInventoryCostCustomer = 0.0
    totalTransportationCost_FirstEchelon = 0.0
    totalTransportationCost_SecondEchelon = 0.0
    totalUnmetDemandCost = 0.0
    
    productionSetup = np.zeros((numPeriods))
    for t in range(numPeriods):
        productionSetup[t] = int(values[index])
        objValue += setupCost * productionSetup[t]
        totalSetupCost += setupCost * productionSetup[t]
        index += 1
    
    productionQuantity = np.zeros((numPeriods))
    for t in range(numPeriods):
        productionQuantity[t] = float(values[index])
        objValue += unitProdCost * productionQuantity[t]
        totalProductionCost += unitProdCost * productionQuantity[t]
        index += 1
        
    plantInventory = np.zeros((numPeriods))
    for t in range(numPeriods):
        plantInventory[t] = float(values[index])
        objValue += unitHoldingCost_Plant * plantInventory[t]
        totalInventoryCostPlant += unitHoldingCost_Plant * plantInventory[t]
        index += 1
    
    if problemType != "2EPRPCS":
        warehouseInventory = np.zeros((numWarehouses, numPeriods))
        for w in range(numWarehouses):
            for t in range(numPeriods):
                warehouseInventory[w][t] = float(values[index])
                objValue += unitHoldingCost_Warehouse[w] * warehouseInventory[w][t]
                totalInventoryCostWarehouse += unitHoldingCost_Warehouse[w] * warehouseInventory[w][t]
                index += 1
        
                
    customerInventory = np.zeros((numCustomers, numPeriods))
    for i in range(numCustomers):
        for t in range(numPeriods):
            customerInventory[i][t] = float(values[index])
            objValue += unitHoldingCost_Customer[i] * customerInventory[i][t]
            totalInventoryCostCustomer += unitHoldingCost_Customer[i] * customerInventory[i][t]
            index += 1
    
    if shortage_allowed:       
        customerUnmetDemand = np.zeros((numCustomers, numPeriods))
        for i in range(numCustomers):
            for t in range(numPeriods):
                customerUnmetDemand[i][t] = float(values[index])
                objValue += unmetDemandPenalty[i] * customerUnmetDemand[i][t]
                totalUnmetDemandCost += unmetDemandPenalty[i] * customerUnmetDemand[i][t]
                index += 1
    
    # Parse routes from plant to warehouses
    routesPlantToWarehouse = [[[] for _ in range(numVehicles_Plant)] for _ in range(numPeriods)]
    while values[index] != 'endRoutesPlantToWarehouse':
        period = int(values[index])
        index += 1
        vehicle = int(values[index])
        index += 2  # Skip over vehicle and colon

        route = [int(values[index])]
        index += 1

        visited_node = -1
        while visited_node != route[0]:
            route.append(int(values[index]))
            visited_node = route[-1]
            index += 1

        if period < numPeriods and vehicle < numVehicles_Plant:
            routesPlantToWarehouse[period][vehicle] = route
    index += 1
    
    for t in range(numPeriods):
        for k in range(numVehicles_Plant):
            if len(routesPlantToWarehouse[t][k]) != 0:
                prevNode = routesPlantToWarehouse[t][k][0]
                for i in range(1, len(routesPlantToWarehouse[t][k])):
                    currNode = routesPlantToWarehouse[t][k][i]
                    objValue += transportationCost_FirstEchelon[prevNode][currNode]
                    totalTransportationCost_FirstEchelon += transportationCost_FirstEchelon[prevNode][currNode]
                    prevNode = currNode
                    
    deliveryQuantity_Warehouse = np.zeros((numWarehouses, numPeriods))
    for w in range(numWarehouses):
        for t in range(numPeriods):
            deliveryQuantity_Warehouse[w][t] = float(values[index])
            index += 1

    # Parse routes from warehouses to customers
    routesWarehouseToCustomer = [[[[] for _ in range(numVehicles_Warehouse)] for _ in range(numPeriods)] for _ in range(numWarehouses)]
    while values[index] != 'endRoutesWarehouseToCustomer':
        warehouse = int(values[index])
        index += 1
        period = int(values[index])
        index += 1
        vehicle = int(values[index])
        index += 2  # Skip over vehicle and colon

        route = [int(values[index])]
        index += 1

        visited_node = -1
        while visited_node != route[0]:
            route.append(int(values[index]))
            visited_node = route[-1]
            index += 1

        if warehouse < numWarehouses and period < numPeriods and vehicle < numVehicles_Warehouse:
            routesWarehouseToCustomer[warehouse][period][vehicle] = route
    index += 1
    
    for w in range(numWarehouses):
        for t in range(numPeriods):
            for k in range(numVehicles_Warehouse):
                if len(routesWarehouseToCustomer[w][t][k]) > 2:
                    prevNode = routesWarehouseToCustomer[w][t][k][0]
                    for i in routesWarehouseToCustomer[w][t][k][1:]:
                        currNode = i
                        objValue += transportationCost_SecondEchelon[prevNode][currNode]
                        totalTransportationCost_SecondEchelon += transportationCost_SecondEchelon[prevNode][currNode]
                        prevNode = currNode
    
    for w in range(numWarehouses):
        for t in range(numPeriods):
            for k in range(numVehicles_Warehouse):
                if len(routesWarehouseToCustomer[w][t][k]) > 0:
                    print(f'route for warehouse {w + 1} period {t + 1} vehicle  {k + 1} is:', routesWarehouseToCustomer[w][t][k])
    
            
    deliveryQuantity_Customer = np.zeros((numCustomers, numPeriods))
    for i in range(numCustomers):
        for t in range(numPeriods):
            deliveryQuantity_Customer[i][t] = float(values[index])
            index += 1
    
    if problemType == "2EPRPCS":
        for w in range(numWarehouses):
            for t in range(numPeriods):
                totalHandlingCostSatellite += unitHandlingCost_Satellite[w] * deliveryQuantity_Warehouse[w][t]
                
    # print('\nDelivery Quantity Customer is: \n', deliveryQuantity_Customer)
    # ----------------------------------------------------------------------------

    # print(f'{len(values)} is length of values and {index} is index\n')
    assert len(values) == index

    flag = False
    violated_constraints = []
    print(f'Total Setup Cost is {totalSetupCost:.1f}')
    print(f'Total Production Cost is {totalProductionCost:.1f}')
    print(f'Total Inventory Cost (Plant) is {totalInventoryCostPlant:.1f}')
    print(f'Total Transportation Cost (First echelon) is {totalTransportationCost_FirstEchelon:.1f}')
    if problemType == "2EPRPCS":
        print(f'Total Handling Cost (Satellite) is {totalHandlingCostSatellite:.1f}')
    else:
        print(f'Total Inventory Cost (Warehouse) is {totalInventoryCostWarehouse:.1f}')
    print(f'Total Inventory Cost (Customer) is {totalInventoryCostCustomer:.1f}')
    if shortage_allowed:
        print(f'Total Shortage Cost is {totalUnmetDemandCost:.1f}')
    print(f'Total Transportation Cost (Second echelon) is {totalTransportationCost_SecondEchelon:.1f}')
    print(f'objValue is {objValue:.1f}')
    # ----------------------------------------------------------------------------

    # Return parsed data
    return {
        "numWarehouses": numWarehouses,
        "numCustomers": numCustomers,
        "numPeriods": numPeriods,
        "numVehicles_Plant": numVehicles_Plant,
        "coordX_Warehouse": coordX_Warehouse,
        "coordY_Warehouse": coordY_Warehouse,
        "coordX_Customer": coordX_Customer,
        "coordY_Customer": coordY_Customer,
        "routesPlantToWarehouse": routesPlantToWarehouse,
        "routesWarehouseToCustomer": routesWarehouseToCustomer,
        "transportationCost_FirstEchelon": transportationCost_FirstEchelon,
        "transportationCost_SecondEchelon": transportationCost_SecondEchelon,
    }

def draw_routes(data, file_path):
    # Extract data
    numPeriods = data["numPeriods"]
    numVehicles_Plant = data["numVehicles_Plant"]
    numWarehouses = data["numWarehouses"]
    numVehicles_Warehouse = len(data["routesWarehouseToCustomer"][0][0])  # Assumes at least one vehicle is present
    coordX_Warehouse = data["coordX_Warehouse"]
    coordY_Warehouse = data["coordY_Warehouse"]
    coordX_Customer = data["coordX_Customer"]
    coordY_Customer = data["coordY_Customer"]
    routesPlantToWarehouse = data["routesPlantToWarehouse"]
    routesWarehouseToCustomer = data["routesWarehouseToCustomer"]
    transportationCost_FirstEchelon = data["transportationCost_FirstEchelon"]
    transportationCost_SecondEchelon = data["transportationCost_SecondEchelon"]

    # Plot routes for each period
    for t in range(numPeriods):
        plt.figure(figsize=(12, 8))
        plt.title(f"Routes for Period {t + 1}")
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")

        # Plot plant location
        plt.scatter([0], [0], c='red', label="Plant", zorder=3)
        plt.text(0, 0, "Plant", fontsize=10, color="red", zorder=3)

        # Plot warehouse locations
        plt.scatter(coordX_Warehouse, coordY_Warehouse, c='blue', label="Warehouses", zorder=3)
        for w in range(numWarehouses):
            plt.text(coordX_Warehouse[w], coordY_Warehouse[w], f"W{w}", fontsize=8, color="blue", zorder=3)

        # Plot customer locations
        plt.scatter(coordX_Customer, coordY_Customer, c='green', label="Customers", zorder=3)
        for i in range(len(coordX_Customer)):
            plt.text(coordX_Customer[i], coordY_Customer[i], f"C{i}", fontsize=8, color="green", zorder=3)

        # Draw plant-to-warehouse routes
        for k in range(numVehicles_Plant):
            route = routesPlantToWarehouse[t][k]
            if len(route) > 1:
                for i in range(len(route) - 1):
                    from_node = route[i]
                    to_node = route[i + 1]
                    x1, y1 = (0, 0) if from_node == 0 else (coordX_Warehouse[from_node - 1], coordY_Warehouse[from_node - 1])
                    x2, y2 = coordX_Warehouse[to_node - 1], coordY_Warehouse[to_node - 1]
                    cost = transportationCost_FirstEchelon[from_node][to_node]
                    plt.arrow(x1, y1, x2 - x1, y2 - y1, color='red', length_includes_head=True, head_width=0.5)
                    plt.text((x1 + x2) / 2, (y1 + y2) / 2, f"{cost:.1f}", fontsize=8, color="red")

        # Draw warehouse-to-customer routes
        for w in range(numWarehouses):
            for k in range(numVehicles_Warehouse):
                route = routesWarehouseToCustomer[w][t][k]
                if len(route) > 1:
                    for i in range(len(route) - 1):
                        from_node = route[i]
                        to_node = route[i + 1]
                        x1, y1 = coordX_Warehouse[w] if from_node == 0 else (coordX_Customer[from_node - 1], coordY_Customer[from_node - 1])
                        x2, y2 = coordX_Customer[to_node - 1], coordY_Customer[to_node - 1]
                        cost = transportationCost_SecondEchelon[from_node][to_node]
                        plt.arrow(x1, y1, x2 - x1, y2 - y1, color='blue', length_includes_head=True, head_width=0.5)
                        plt.text((x1 + x2) / 2, (y1 + y2) / 2, f"{cost:.1f}", fontsize=8, color="blue")

        plt.legend()
        plt.grid(True)

        # Save plot to the specified path
        plt.savefig(f"{file_path}/routes_period_{t + 1}.png")
        plt.close()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: script.py <file_path> <problemType> <output_path>")
        sys.exit(1)

    file_path = sys.argv[1]
    problemType = sys.argv[2]
    output_path = sys.argv[3]

    data = read_data(file_path, problemType)
    draw_routes(data, output_path)

