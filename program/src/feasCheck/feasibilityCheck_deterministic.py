"""
In this Problem we aim to check the results obtained from the heuristic problemType to check the feasibility of the results (for the deterministic case)
"""
import numpy as np
import sys

def feasibilityCheck(file_path, problemType):
    # ----------------------------------------------------------------------------
    # Initializization
    # ----------------------------------------------------------------------------
    
    violated_constraints = []
    # We first need to open the data file and assign values
    with open(file_path, 'r') as fp:
        values = []
        for line in fp:
            ar = line.split()
            for i in ar:
                values.append(i)
                
    if problemType == 'EV' or problemType == 'WS' or problemType == 'EEV':
        shortage_allowed = True
    else:
        shortage_allowed = False
    # Load Parameters
    index = 0

    # First Line
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
        index+=1
    
    if problemType == "WS" or problemType == "EEV":
        scenario_Index = int(values[index])
        index+=1
    
    if shortage_allowed:
        # Second Line
        uncertaintyRange = float(values[index])
        index+=1
        probabilityFunction = str(values[index])
        index+=1
        unmetDemandPenaltyCoeff = float(values[index])
        index += 1

    # Third Line
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
    
    # Problem Parameters
    coordX_Plant = float(values[index])
    index += 1
    coordY_Plant = float(values[index])
    index += 1
    
    coordX_Warehouse = np.zeros((numWarehouses))
    for w in range(numWarehouses):
        coordX_Warehouse[w] = float(values[index])
        index += 1
    coordY_Warehouse = np.zeros((numWarehouses))
    for w in range(numWarehouses):
        coordY_Warehouse[w] = float(values[index])
        index += 1
    
    coordX_Customer = np.zeros((numCustomers))
    for i in range(numCustomers):
        coordX_Customer[i] = float(values[index])
        index += 1
    coordY_Customer = np.zeros((numCustomers))
    for i in range(numCustomers):
        coordY_Customer[i] = float(values[index])
        index += 1
        
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
                
    # routesPlantToWarehouse = np.zeros((numPeriods, numVehicles_Plant, 0))
    # Placeholder for routes with variable lengths
    routesPlantToWarehouse = [[[] for _ in range(numVehicles_Plant)] for _ in range(numPeriods)]
    while values[index] != 'endRoutesPlantToWarehouse':
        period = int(values[index])
        index += 1
        vehicle = int(values[index])
        index += 2 # Skip over vehicle, and the colon
        
        route = []
        route.append(int(values[index]))
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
            
    routesWarehouseToCustomer = [[[[] for _ in range(numVehicles_Warehouse)] for _ in range(numPeriods)] for _ in range(numWarehouses)]
    while values[index] != 'endRoutesWarehouseToCustomer':
        warehouse = int(values[index])
        index += 1
        period = int(values[index])
        index += 1
        vehicle = int(values[index])
        index += 2 # Skip over vehicle, and the colon

        route = []
        route.append(int(values[index]))
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

    """This part checks the feasibility of the given solution"""
    tolerance = 1e-2
    
    # Check the Production Capacity Constraint
    for t in range(numPeriods):
        p = productionQuantity[t]
        M = prodCapacity
        y = productionSetup
        
        if p > (M * y[t]) + tolerance:
            print(f'Production Capacity Constraint is not met for t={t + 1}')
            print(f'{p} > {M * y[t]}')
            violated_constraints.append(f'Production Capacity Constraint: t={t + 1}')
            
    # Check Plant Inventory Balance Constrains
    for t in range(numPeriods):
        I_plant = initialInventory_Plant if t == 0 else plantInventory[t - 1]
        I_plant += productionQuantity[t]
        I_plant -= np.sum(deliveryQuantity_Warehouse[:, t], axis=(0))
        
        I_plant_expected = plantInventory[t]
        
        if abs(I_plant - I_plant_expected) > tolerance:
            print(f'Plant Inventory Balance is not met for t={t + 1}')
            if t == 0:
                print(f'{I_plant} != {I_plant_expected} + {initialInventory_Plant} - {np.sum(deliveryQuantity_Warehouse[:, t], axis=(0))}')
            else:
                print(f'{I_plant} != {I_plant_expected} + {plantInventory[t - 1]} - {np.sum(deliveryQuantity_Warehouse[:, t], axis=(0))}')
            violated_constraints.append(f'Plant Inventory Balance: t={t + 1}')
            
    # Check Inventory Capacity Plant Constraint
    for t in range(numPeriods):
        if plantInventory[t] > storageCapacity_Plant + tolerance:
            print(f'Inventory Capacity Plant Constraint is not met for t={t + 1}')
            print(f'{plantInventory[t]} > {storageCapacity_Plant}')
            violated_constraints.append(f'Inventory Capacity Plant Constraint: t={t + 1}')
    
    if problemType != "2EPRPCS":
        # Check Warehouse Inventory Balance Constrains
        for t in range(numPeriods):
            for w in range(numWarehouses):
                I_warehouse = initialInventory_Warehouse[w] if t == 0 else warehouseInventory[w][t - 1]
                I_warehouse += deliveryQuantity_Warehouse[w][t]
                
                deliveryQuantity_CustomerWarehouse = 0.0
                for i in range(numCustomers):
                    if customerAssignmentToWarehouse[t][w][i] == 1:
                        deliveryQuantity_CustomerWarehouse += deliveryQuantity_Customer[i][t]
                
                I_warehouse -= deliveryQuantity_CustomerWarehouse
                
                I_warehouse_expected = warehouseInventory[w][t]
                
                if abs(I_warehouse - I_warehouse_expected) > tolerance:
                    print(f'Warehouse Inventory Balance is not met for w={w + 1}, t={t + 1}')
                    if t == 0:
                        print(f'{I_warehouse_expected} != {initialInventory_Warehouse[w]} + {deliveryQuantity_Warehouse[w][t]} - {deliveryQuantity_CustomerWarehouse}')
                    else:
                        print(f'{I_warehouse_expected} != {warehouseInventory[w][t - 1]} + {deliveryQuantity_Warehouse[w][t]} - {deliveryQuantity_CustomerWarehouse}')
                    violated_constraints.append(f'Warehouse Inventory Balance: w={w + 1}, t={t + 1}')
    else:
        # Check Satellite Balance Constrains
        satellite_Inbound = 0.0
        satellite_Outbound = 0.0
        for t in range(numPeriods):
            for w in range(numWarehouses):
                satellite_Inbound += deliveryQuantity_Warehouse[w][t]
                
                for i in range(numCustomers):
                    if customerAssignmentToWarehouse[t][w][i] == 1:
                        satellite_Outbound += deliveryQuantity_Customer[i][t]
                                                
                if abs(satellite_Inbound - satellite_Outbound) > tolerance:
                    print(f'Satellite Balance is not met for w={w + 1}, t={t + 1}')
                    print(f'{satellite_Inbound} != {satellite_Outbound}')
                    
                    violated_constraints.append(f'Satellite Balance: w={w + 1}, t={t + 1}')
    
    if problemType != "2EPRPCS":
        # Check Warehouse Inventory Capacity Constraint
        for t in range(numPeriods):
            for w in range(numWarehouses):
                if warehouseInventory[w][t] > storageCapacity_Warehouse[w] + tolerance:
                    print(f'Inventory Capacity Warehouse Constraint is not met for w={w + 1}, t={t + 1}')
                    print(f'{warehouseInventory[w][t]} > {storageCapacity_Warehouse[w]}')
                    violated_constraints.append(f'Inventory Capacity Warehouse Constraint: w={w + 1}, t={t + 1}')
                    
    # Check warehouse visit (Only delivered if visited)
    for t in range(numPeriods):
        visited_warehouses = []
        for k in range(numVehicles_Plant):
            if len(routesPlantToWarehouse[t][k]) > 2:
                for w in routesPlantToWarehouse[t][k][1:-1]:
                    warehouse_index = w - 1
                    if warehouse_index not in visited_warehouses:
                        visited_warehouses.append(warehouse_index)
                        
        for w in range(numWarehouses):
            if w not in visited_warehouses and deliveryQuantity_Warehouse[w][t] > tolerance:
                print(f'Warehouse {w + 1} is not visited in period {t + 1} but has a delivery quantity of {deliveryQuantity_Warehouse[w][t]}')
                violated_constraints.append(f'Warehouse visit: Warehouse {w + 1} is not visited in period {t + 1} but has a delivery quantity of {deliveryQuantity_Warehouse[w][t]}')
                    
                    
    # Check split deliveries to warehouses (In each period only one vehicle must visit a warehouse)
    for t in range(numPeriods):
        visited_warehouses = []
        for k in range(numVehicles_Plant):
            if len(routesPlantToWarehouse[t][k]) > 2:
                for w in routesPlantToWarehouse[t][k][1:-1]:
                    warehouse_index = w - 1 
                    if warehouse_index in visited_warehouses:
                        print(f'Warehouse {warehouse_index} is visited more than once in period {t + 1}')
                        print(f'Visited warehouses: {visited_warehouses}')
                        violated_constraints.append(f'Slplit deliveries to warehouses: Warehouse {warehouse_index} is visited more than once in period {t + 1}')
                        break
                    else:
                        visited_warehouses.append(warehouse_index)
                
                
    # Check Vehicle Capacity Constraint (Plant to Warehouse)
    for t in range(numPeriods):
        for k in range(numVehicles_Plant):
            if len(routesPlantToWarehouse[t][k]) > 2:
                vehicle_load = 0
                for w in routesPlantToWarehouse[t][k][1:-1]:
                    warehouse_index = w - 1 
                    vehicle_load += deliveryQuantity_Warehouse[warehouse_index][t]
            
                if vehicle_load > vehicleCapacity_Plant + tolerance:
                    print(f'Vehicle Capacity Constraint is not met for k={k + 1}, t={t + 1}')
                    print(f'{vehicle_load} > {vehicleCapacity_Plant}')
                    violated_constraints.append(f'Vehicle Capacity Constraint: k={k + 1}, t={t + 1}')                
    
    # Check Customer Inventory Balance Constrains
    for t in range(numPeriods):
        for i in range(numCustomers):
            I_customer = initialInventory_Customer[i] if t == 0 else customerInventory[i][t - 1]
            I_customer -= demand[i][t]
            if shortage_allowed:
                I_customer += customerUnmetDemand[i][t]    
            
            deliveryQuantity_CustomerWarehouse = 0.0
            for w in range(numWarehouses):
                if customerAssignmentToWarehouse[t][w][i] == 1:
                    deliveryQuantity_CustomerWarehouse += deliveryQuantity_Customer[i][t]
                            
            I_customer += deliveryQuantity_CustomerWarehouse
            
            I_customer_expected = customerInventory[i][t]
            
            if abs(I_customer - I_customer_expected) > tolerance:
                print(f'Customer Inventory Balance is not met for {i + numWarehouses}, t={t + 1}')
                print(f'{I_customer} != {I_customer_expected}')
                violated_constraints.append(f'Customer Inventory Balance: {i + 1 + numWarehouses}, t={t + 1}')
                    
    # Check Customer Inventory Capacity Constraint
    for t in range(numPeriods):
        for i in range(numCustomers):
            if customerInventory[i][t] + demand[i][t] > storageCapacity_Customer[i] + tolerance:
                print(f'Inventory Capacity is violated for {i + numWarehouses} and t={t + 1}')
                print(f'{customerInventory[i][t]} > {storageCapacity_Customer[i]}')
                violated_constraints.append(f'Inventory Capacity: {i + numWarehouses} and t={t + 1}')
                    
    # Check Customer visit (Only delivered if visited and if is assigned to a warehouse)
    for t in range(numPeriods):
        for w in range(numWarehouses):
            visited_customers = []
            for k in range(numVehicles_Warehouse):
                if len(routesWarehouseToCustomer[w][t][k]) > 2:
                    for i in routesWarehouseToCustomer[w][t][k][1:-1]:
                        customer_index = i - numWarehouses
                        if customer_index not in visited_customers:
                            visited_customers.append(customer_index)
            
            # print(f'Visited customers by warehouse {w + 1} in period {t + 1}: {visited_customers}')            
            for i in range(numCustomers):
                if customerAssignmentToWarehouse[t][w][i] == 1:
                    if i not in visited_customers and deliveryQuantity_Customer[i][t] > tolerance:
                        print(f'Customer {i + numWarehouses} is not visited period {t + 1}, by warehouse {w + 1} and has a delivery of {deliveryQuantity_Customer[i][t]}')
                        violated_constraints.append(f'Customer visit: Customer {i + 1 + numWarehouses} is not visited period {t + 1} by warehouse {w + 1} but has a delivery of {deliveryQuantity_Customer[i][t]}')
                    
            for i in visited_customers:
                if customerAssignmentToWarehouse[t][w][i] == 0:
                    print(f'Customer {i + numWarehouses} is visited in period {t + 1}, by warehouse {w + 1} but is not assigned to warehouse {w + 1}')
                    violated_constraints.append(f'Customer visit: Customer {i + 1 + numWarehouses} is visited period {t + 1} by warehouse {w + 1} but is not assigned to warehouse {w + 1}')
                        
                        
                        
    # Check Split Delivery to Customers (In each period only one vehicle must visit a customer)
    # Also ensure the customer is only visited by one warehouse
    for t in range(numPeriods):
        visited_customers_period = []
        for w in range(numWarehouses):
            visited_customers_warehouse = []
            for k in range(numVehicles_Warehouse):
                if len(routesWarehouseToCustomer[w][t][k]) > 2:
                    for i in routesWarehouseToCustomer[w][t][k][1:-1]:
                        customer_index = i - numWarehouses
                        if customer_index in visited_customers_warehouse:
                            print(f'Customer {customer_index} is visited by more than one vehicle in period {t + 1} in warehouse {w + 1}')
                            violated_constraints.append(f'Customer visit: Customer {customer_index} is visited by more than one vehicle in period {t + 1} in warehouse {w + 1}')
                        else:
                            visited_customers_warehouse.append(customer_index)
                            
                        if customer_index in visited_customers_period:
                            print(f'Customer {customer_index} is visited by more than one warehouse in period {t + 1}')
                            violated_constraints.append(f'Customer visit: Customer {customer_index} is visited by more than one warehouse in period {t + 1}')
                        else:
                            visited_customers_period.append(customer_index)
                                
    # Check Vehicle Capacity Constraint (Warehouse to Customer)
    for w in range(numWarehouses):
        for t in range(numPeriods):
            for k in range(numVehicles_Warehouse):
                if len(routesWarehouseToCustomer[w][t][k]) > 2:
                    vehicle_load = 0
                    for i in routesWarehouseToCustomer[w][t][k][1:-1]:
                        customer_index = i - numWarehouses
                        vehicle_load += deliveryQuantity_Customer[customer_index][t]
                        
                    if vehicle_load > vehicleCapacity_Warehouse + tolerance:
                        print(f'Vehicle Capacity Constraint is violated for w={w + 1}, t={t + 1}, k={k + 1}')
                        print(f'{vehicle_load} > {vehicleCapacity_Warehouse}')
                        violated_constraints.append(f'Vehicle Capacity Constraint: w={w + 1}, t={t + 1}, k={k + 1}')

    if not violated_constraints:
        print('Feasible Solution For')
    else:
        for constraint in violated_constraints:
            print(constraint)
        flag = True

    return(flag)


if __name__ == "__main__":
    
    if len(sys.argv) < 3:
        print("Usage: feasibilityCheck_deterministic.py <file_path> <problemType>")
        sys.exit(1)

    file_path = sys.argv[1]
    problemType = sys.argv[2]
    
    result = feasibilityCheck(file_path, problemType)
    
    if not result:
        print('0')
    else:
        print('1')