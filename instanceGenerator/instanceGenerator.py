import numpy as np
import math
import matplotlib.pyplot as plt
import argparse

class ParameterSettingGenerate:

    def __init__(self, numCustomers, numWarehouses, numPeriods, numVehicles_Plant, numVehicles_Warehouse, insNum):
        self.numCustomers = numCustomers
        self.numWarehouses = numWarehouses
        self.numPeriods = numPeriods
        self.numVehicles_Plant = numVehicles_Plant
        self.numVehicles_Warehouse = numVehicles_Warehouse
        self.insNum = insNum
        self.seed = (self.numCustomers + self.numWarehouses) * self.insNum * 1000
        
        self.coordX_Plant = 0
        self.coordY_Plant = 0
        self.coordX_Customer = np.zeros(self.numCustomers)
        self.coordY_Customer = np.zeros(self.numCustomers)
        self.coordX_Warehouse = np.zeros(self.numWarehouses)
        self.coordY_Warehouse = np.zeros(self.numWarehouses)
        
        self.consumeRate = np.zeros(self.numCustomers)
        self.storageCapacity_Plant = 0
        self.storageCapacity_Customer = np.zeros(self.numCustomers)
        self.storageCapacity_Warehouse = np.zeros(self.numWarehouses)
        self.initialInventory_Plant = 0
        self.initialInventory_Warehouse = np.zeros(self.numWarehouses)
        self.initialInventory_Customer = np.zeros(self.numCustomers)
        self.unitHoldingCost_Plant = 0
        self.unitHoldingCost_Warehouse = np.zeros(self.numWarehouses)
        self.unitHoldingCost_Customer = np.zeros(self.numCustomers)
        
        self.prodCapacity = 0
        self.setupCost = 0
        self.unitProdCost = 0
        self.vehicleCapacity_Plant = 0
        self.vehicleCapacity_Warehouse = 0

    def uniform_distribution(self, min_val, max_val, seed=None):
        if seed is not None:
            rng = np.random.default_rng(seed)
            return rng.uniform(min_val, max_val)
        else:
            return np.random.uniform(min_val, max_val) 
    
    def generate_instance(self, x_min, x_max, y_min, y_max):
        self.generate_customer_nodes(x_min, x_max, y_min, y_max)
        self.generate_warehouse_nodes(x_min, x_max, y_min, y_max, self.numWarehouses)
        self.generate_plant_nodes(x_min, x_max, y_min, y_max)
        self.set_consume_rate()
        self.set_storage_capacity()
        self.set_initial_inventory()
        self.set_costs()
        self.set_vehicle_capacity()

    def generate_customer_nodes(self, x_min, x_max, y_min, y_max):
        m = self.numCustomers / 10.0
        index = 0
        # High-density area (bottom-right, 5m nodes)
        for i in range(math.floor(5 * m)):
            # Generate X coordinate and increment seed
            self.coordX_Customer[index] = round(self.uniform_distribution(x_max / 2.0, x_max, self.seed))
            self.seed += 10 # Increment seed after generating X coordinate
            
            # Generate Y coordinate and increment seed
            self.coordY_Customer[index] = round(self.uniform_distribution(y_min, y_max / 2.0, self.seed))
            self.seed += 10
            
            index += 1

        # Medium-density area (top-right, 3m nodes)
        for i in range(math.floor(3 * m)):
            self.coordX_Customer[index] = round(self.uniform_distribution(x_max / 2.0, x_max, self.seed))
            self.seed += 10
            
            self.coordY_Customer[index] = round(self.uniform_distribution(y_max / 2.0, y_max, self.seed))
            self.seed += 10
            
            index += 1

        # Remaining customers for low-density areas
        remaining_customers = self.numCustomers - index
        low_density_1 = remaining_customers // 2
        low_density_2 = remaining_customers - low_density_1

        # Low-density area 1 (top-left)
        for i in range(low_density_1):
            self.coordX_Customer[index] = round(self.uniform_distribution(x_min, x_max / 2.0, self.seed))
            self.seed += 10
            
            self.coordY_Customer[index] = round(self.uniform_distribution(y_max / 2.0, y_max, self.seed))
            self.seed += 10
            
            index += 1

        # Low-density area 2 (bottom-left)
        for i in range(low_density_2):
            self.coordX_Customer[index] = round(self.uniform_distribution(x_min, x_max / 2.0, self.seed))
            self.seed += 10
            
            self.coordY_Customer[index] = round(self.uniform_distribution(y_min, y_max / 2.0, self.seed))
            self.seed += 10
            
            index += 1

    def generate_warehouse_nodes(self, x_min, x_max, y_min, y_max, numSlices):
        centerX = (x_min + x_max) / 2.0
        centerY = (y_min + y_max) / 2.0
        
        # Calculate semi-major (a) and semi-minor (b) axes of the ellipse
        a = (x_max - x_min) / 2.0  # Semi-major axis (along the x-axis)
        b = (y_max - y_min) / 2.0  # Semi-minor axis (along the y-axis)
        
        a -= 30
        b -= 30
        
        # Divide the ellipse into numSlices
        sliceAngleIncrement = (2 * math.pi) / numSlices
        warehousesPerSlice = self.numWarehouses // numSlices
        warehouseIndex = 0

        for slice in range(numSlices):
            sliceStartAngle = slice * sliceAngleIncrement
            sliceEndAngle = (slice + 1) * sliceAngleIncrement

            for j in range(warehousesPerSlice):
                # Randomly select an angle within this slice
                angle = self.uniform_distribution(sliceStartAngle, sliceEndAngle, self.seed)
                self.seed += 10
                
                # Generate warehouse coordinates on the ellipse using parametric equation of an ellipse
                # X = centerX + a * cos(angle)
                # Y = centerY + b * sin(angle)
                self.coordX_Warehouse[warehouseIndex] = round(centerX + a * math.cos(angle))
                self.coordY_Warehouse[warehouseIndex] = round(centerY + b * math.sin(angle))
                
                # Increment the warehouse index
                warehouseIndex += 1

    def generate_plant_nodes(self, x_min, x_max, y_min, y_max):
        self.coordX_Plant = round(self.uniform_distribution(20.0, 120.0, self.seed))
        self.seed += 10
        
        self.coordY_Plant = round(self.uniform_distribution(780.0, 980.0, self.seed))
        self.seed += 10

    def draw_nodes(self):
        # Plot customer nodes in blue
        plt.scatter(self.coordX_Customer, self.coordY_Customer, s=50, c="blue", label="Customers")

        # Plot warehouse nodes in red
        plt.scatter(self.coordX_Warehouse, self.coordY_Warehouse, s=50, c="red", label="Warehouses")

        # Plot plant nodes in green
        plt.scatter(self.coordX_Plant, self.coordY_Plant, s=50, c="green", label="Plant")

        # Set axis limits and draw quadrants
        plt.xlim(0, 500)
        plt.ylim(0, 1000)
        plt.axhline(500, color="black")
        plt.axvline(250, color="black")

        # Add labels and save the plot to a file (PNG)
        plt.legend()
        figure_name = f"2EPRP_S0{self.insNum}W{self.numWarehouses}C{self.numCustomers}T{self.numPeriods}VP{self.numVehicles_Plant}VW{self.numVehicles_Warehouse}.png"
        plt.savefig(figure_name)

        # Show plot if needed (uncomment if interactive display is preferred)
        # plt.show()
            
    def set_consume_rate(self):
        minConsumeRate_Range = 5.0
        maxConsumeRate_Range = 25.0

        for i in range(self.numCustomers):
            self.consumeRate[i] = round(self.uniform_distribution(minConsumeRate_Range, maxConsumeRate_Range, self.seed))
            self.seed += 10

    def set_storage_capacity(self):
        sumDemandRate = np.sum(self.consumeRate)

        self.prodCapacity = math.floor(2 * sumDemandRate)
        self.storageCapacity_Plant = math.floor(self.prodCapacity / 2.0)

        avgDemWare = (2.0 * sumDemandRate) / self.numWarehouses

        minRange = 0.7 * avgDemWare
        maxRange = 1.0 * avgDemWare

        for i in range(self.numWarehouses):
            self.storageCapacity_Warehouse[i] = round(self.uniform_distribution(minRange, maxRange, self.seed))

        maxLevelRandSet = [2, 3, 6]
        randIndex = round(self.uniform_distribution(0, len(maxLevelRandSet) - 1, self.seed))
        self.seed += 10
        
        maxLevelCoeff = maxLevelRandSet[randIndex]

        for i in range(self.numCustomers):
            self.storageCapacity_Customer[i] = (maxLevelCoeff * self.consumeRate[i]) + self.consumeRate[i]

    def set_initial_inventory(self):
        self.initialInventory_Plant = 0.0

        self.initialInventory_Warehouse = np.zeros(self.numWarehouses)
        
        for i in range(self.numCustomers):
            possibleInitInv = [self.consumeRate[i], 2 * self.consumeRate[i]]
            randIndex = round(self.uniform_distribution(0, len(possibleInitInv) - 1, self.seed))
            self.seed += 10
            
            self.initialInventory_Customer[i] = possibleInitInv[randIndex]

        # if self.numPeriods >= 6:
        #     for i in range(self.numCustomers):
        #         self.initialInventory_Customer[i] = self.storageCapacity_Customer[i] - self.consumeRate[i]
        # elif self.numPeriods == 3:
        #     for i in range(self.numCustomers):
        #         possibleInitInv = [self.consumeRate[i], 2 * self.consumeRate[i]]
        #         randIndex = round(self.uniform_distribution(0, len(possibleInitInv) - 1))
        #         self.initialInventory_Customer[i] = possibleInitInv[randIndex]

    def set_costs(self):
        self.unitHoldingCost_Plant = 3.0
        self.setupCost = 1000 * self.unitHoldingCost_Plant
        self.unitProdCost = 10 * self.unitHoldingCost_Plant

        for i in range(self.numWarehouses):
            self.unitHoldingCost_Warehouse[i] = round(self.uniform_distribution(3.0, 6.0, self.seed))
            self.seed += 10

        for i in range(self.numCustomers):
            self.unitHoldingCost_Customer[i] = round(self.uniform_distribution(6.0, 10.0, self.seed))
            self.seed += 10

    def set_vehicle_capacity(self):
        maxofInvCap_Customer = np.max(self.storageCapacity_Customer)
        VehicleCapacityCoefficient_Warehouse = math.floor(self.numCustomers / 10.0) + 1.0
        self.vehicleCapacity_Warehouse = math.floor((1.5 * VehicleCapacityCoefficient_Warehouse * maxofInvCap_Customer) / self.numVehicles_Warehouse)
        self.vehicleCapacity_Plant = math.floor(2.5 * self.vehicleCapacity_Warehouse)

    def save_solution_to_file(self, filename, plant_data, warehouse_data, customer_data):
        """
        Save the solution to a file in the desired format.

        Arguments:
        - filename: Output file path.
        - plant_data: Tuple containing plant data (coordX, coordY, initial_inventory, unit_holding_cost, inventory_capacity, unit_production_cost, setup_cost, production_capacity).
        - warehouse_data: List of tuples containing warehouse data (coordX, coordY, initial_inventory, unit_holding_cost, inventory_capacity).
        - customer_data: List of tuples containing customer data (coordX, coordY, initial_inventory, unit_holding_cost, inventory_capacity, demand_avg).
        - plant_vehicle_capacity: Capacity of vehicles at the plant.
        - warehouse_vehicle_capacity: Capacity of vehicles at the warehouses.
        - num_periods: Number of periods in the instance.
        - num_vehicles_plant: Number of vehicles at the plant.
        - num_vehicles_warehouse: Number of vehicles at each warehouse.
        """

        with open(filename, 'w') as f:
            # Header information
            f.write("THIS IS AN INSTANCE FOR THE TWO-ECHELON PRODUCTION ROUTING PROBLEM\n")
            f.write(f"THIS INSTANCE INCLUDES ONE PRODUCTION PLANT, {len(warehouse_data)} WAREHOUSES, AND {len(customer_data)} CUSTOMERS.\n")
            f.write(f"THIS INSTANCE HAS {self.numPeriods} PERIODS, {self.numVehicles_Plant} VEHICLES AT THE PRODUCTION PLANT, AND {self.numVehicles_Warehouse} VEHICLES AT EACH WAREHOUSE.\n")
            f.write(f"PLANT VEHICLE CAPACITY: {self.vehicleCapacity_Plant:.1f}\n")
            f.write(f"WAREHOUSE VEHICLE CAPACITY: {self.vehicleCapacity_Warehouse:.1f}\n")

            # Plant data
            f.write("PRODUCTION_PLANT, COORD_X, COORD_Y, INITIAL_INVENTORY, UNIT_HOLDING_COST, INVENTORY_CAPACITY, UNIT_PRODUCTION_COST, SETUP_COST, PRODUCTION_CAPACITY\n")
            f.write(f"0, {plant_data['coordX']:.1f}, {plant_data['coordY']:.1f}, {plant_data['initial_inventory']:.1f}, {plant_data['unit_holding_cost']:.1f}, {plant_data['inventory_capacity']:.1f}, {plant_data['unit_production_cost']:.1f}, {plant_data['setup_cost']:.1f}, {plant_data['production_capacity']:.1f}\n")

            # Warehouse data
            f.write("WAREHOUSE, COORD_X, COORD_Y, INITIAL_INVENTORY, UNIT_HOLDING_COST, INVENTORY_CAPACITY\n")
            for i, warehouse in enumerate(warehouse_data):
                f.write(f"{i+1}, {warehouse['coordX']:.1f}, {warehouse['coordY']:.1f}, {warehouse['initial_inventory']:.1f}, {warehouse['unit_holding_cost']:.1f}, {warehouse['inventory_capacity']:.1f}\n")

            # Retailer data
            f.write("CUSTOMER, COORD_X, COORD_Y, INITIAL_INVENTORY, UNIT_HOLDING_COST, INVENTORY_CAPACITY, DEMAND_AVG\n")
            for i, customer in enumerate(customer_data):
                f.write(f"{i+len(warehouse_data)+1}, {customer['coordX']:.1f}, {customer['coordY']:.1f}, {customer['initial_inventory']:.1f}, {customer['unit_holding_cost']:.1f}, {customer['inventory_capacity']:.1f}, {customer['demand_avg']:.1f}\n")

        print(f"Solution saved to {filename}")

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Two-Echelon Production Routing Problem')
    parser.add_argument('--numWarehouses', type=int, default=2, help='Number of warehouses')
    parser.add_argument('--numCustomers', type=int, default=10, help='Number of customers')
    parser.add_argument('--numPeriods', type=int, default=6, help='Number of periods')
    parser.add_argument('--numVehicles_Plant', type=int, default=1, help='Number of vehicles at the plant')
    parser.add_argument('--numVehicles_Warehouse', type=int, default=1, help='Number of vehicles at the warehouses')
    parser.add_argument('--insNum', type=int, default=1, help='Instance number')

    # Parse arguments
    args = parser.parse_args()

    # Initialize with parsed arguments
    param_gen = ParameterSettingGenerate(
        numWarehouses=args.numWarehouses,
        numCustomers=args.numCustomers,
        numPeriods=args.numPeriods,
        numVehicles_Plant=args.numVehicles_Plant,
        numVehicles_Warehouse=args.numVehicles_Warehouse,
        insNum=args.insNum
    )

    x_min = 0
    x_max = 500
    y_min = 0
    y_max = 1000
    
    param_gen.generate_instance(x_min, x_max, y_min, y_max)

    # Call draw_nodes to generate the plot
    param_gen.draw_nodes()
    
    # Example plant, warehouse, and customer data
    plant_data = {
        'coordX': param_gen.coordX_Plant,
        'coordY': param_gen.coordY_Plant,
        'initial_inventory': param_gen.initialInventory_Plant,
        'unit_holding_cost': param_gen.unitHoldingCost_Plant,
        'inventory_capacity': param_gen.storageCapacity_Plant,
        'unit_production_cost': param_gen.unitProdCost,
        'setup_cost': param_gen.setupCost,
        'production_capacity': param_gen.prodCapacity,        
    }

    warehouse_data = [
        {
            'coordX': param_gen.coordX_Warehouse[i], 
            'coordY': param_gen.coordY_Warehouse[i], 
            'initial_inventory': param_gen.initialInventory_Warehouse[i], 
            'unit_holding_cost': param_gen.unitHoldingCost_Warehouse[i], 
            'inventory_capacity': param_gen.storageCapacity_Warehouse[i]
        }
        for i in range(param_gen.numWarehouses)
    ]

    customer_data = [
        {
            'coordX': param_gen.coordX_Customer[i], 
            'coordY': param_gen.coordY_Customer[i], 
            'initial_inventory': param_gen.initialInventory_Customer[i], 
            'unit_holding_cost': param_gen.unitHoldingCost_Customer[i], 
            'inventory_capacity': param_gen.storageCapacity_Customer[i],
            'demand_avg': param_gen.consumeRate[i]
        }
        for i in range(param_gen.numCustomers)
    ]

    # Save the solution to a file
    param_gen.save_solution_to_file(
        f"../instances2eprp/2EPRP_S0{param_gen.insNum}W{param_gen.numWarehouses}C{param_gen.numCustomers}T{param_gen.numPeriods}VP{param_gen.numVehicles_Plant}VW{param_gen.numVehicles_Warehouse}.txt",
        plant_data,
        warehouse_data,
        customer_data
    )

if __name__ == "__main__":
    # python3 instanceGenerator.py --numWarehouses 2 --numCustomers 15 --numPeriods 3 --numVehicles_Plant 1 --numVehicles_Warehouse 1 --insNum 1
    main()
