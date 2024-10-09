\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 3000 y(0)#0 + 3000 y(1)#1 + 3000 y(2)#2 + 30 p(0)#3 + 30 p(1)#4
       + 30 p(2)#5 + 3 I_plant(0)#6 + 3 I_plant(1)#7 + 3 I_plant(2)#8
       + 4 I_warehouse(0)(0)(0)#9 + 4 I_warehouse(0)(1)(0)#10
       + 4 I_warehouse(0)(2)(0)#11 + 3 I_warehouse(1)(0)(0)#12
       + 3 I_warehouse(1)(1)(0)#13 + 3 I_warehouse(1)(2)(0)#14
       + 3513.33333333333 b_warehouse(0)(0)(0)#15
       + 3513.33333333333 b_warehouse(0)(1)(0)#16
       + 3513.33333333333 b_warehouse(0)(2)(0)#17
       + 2385 b_warehouse(1)(0)(0)#18 + 2385 b_warehouse(1)(1)(0)#19
       + 2385 b_warehouse(1)(2)(0)#20 + 2838.94205647104 o(0)(0)#39
       + 2838.94205647104 o(0)(1)#40 + 2838.94205647104 o(0)(2)#41
       + 1739.25271309261 o(1)(0)#42 + 1739.25271309261 o(1)(1)#43
       + 1739.25271309261 o(1)(2)#44 + 4037.75629949332 o(2)(0)#45
       + 4037.75629949332 o(2)(1)#46 + 4037.75629949332 o(2)(2)#47
Subject To
 ProductionCapacity(1):          - 304 y(0)#0 + p(0)#3 <= 0
 ProductionCapacity(2):          - 304 y(1)#1 + p(1)#4 <= 0
 ProductionCapacity(3):          - 304 y(2)#2 + p(2)#5 <= 0
 PlantInventoryCapacity(1):      I_plant(0)#6 <= 152
 PlantInventoryCapacity(2):      I_plant(1)#7 <= 152
 PlantInventoryCapacity(3):      I_plant(2)#8 <= 152
 PlantInventoryBalance(1):       - p(0)#3 + I_plant(0)#6 + q(0)(0)(0)#21
                                 + q(0)(1)(0)#24 + q(1)(0)(0)#27
                                 + q(1)(1)(0)#30 + q(2)(0)(0)#33
                                 + q(2)(1)(0)#36  = 0
 PlantInventoryBalance(2):       - p(1)#4 - I_plant(0)#6 + I_plant(1)#7
                                 + q(0)(0)(1)#22 + q(0)(1)(1)#25
                                 + q(1)(0)(1)#28 + q(1)(1)(1)#31
                                 + q(2)(0)(1)#34 + q(2)(1)(1)#37  = 0
 PlantInventoryBalance(3):       - p(2)#5 - I_plant(1)#7 + I_plant(2)#8
                                 + q(0)(0)(2)#23 + q(0)(1)(2)#26
                                 + q(1)(0)(2)#29 + q(1)(1)(2)#32
                                 + q(2)(0)(2)#35 + q(2)(1)(2)#38  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(0)(0)(0)#9 <= 136.8
 WarehouseInventoryCapacity(2,1,1): I_warehouse(1)(0)(0)#12 <= 136.8
 WarehouseInventoryCapacity(1,2,1): I_warehouse(0)(1)(0)#10 <= 136.8
 WarehouseInventoryCapacity(2,2,1): I_warehouse(1)(1)(0)#13 <= 136.8
 WarehouseInventoryCapacity(1,3,1): I_warehouse(0)(2)(0)#11 <= 136.8
 WarehouseInventoryCapacity(2,3,1): I_warehouse(1)(2)(0)#14 <= 136.8
 WarehouseInventoryBalance(1,1,1): I_warehouse(0)(0)(0)#9
                                 - b_warehouse(0)(0)(0)#15 - q(0)(0)(0)#21
                                 - q(1)(0)(0)#27 - q(2)(0)(0)#33  = -9
 WarehouseInventoryBalance(2,1,1): I_warehouse(1)(0)(0)#12
                                 - b_warehouse(1)(0)(0)#18 - q(0)(1)(0)#24
                                 - q(1)(1)(0)#30 - q(2)(1)(0)#36  = -10
 WarehouseInventoryBalance(1,2,1): - I_warehouse(0)(0)(0)#9
                                 + I_warehouse(0)(1)(0)#10
                                 - b_warehouse(0)(1)(0)#16 - q(0)(0)(1)#22
                                 - q(1)(0)(1)#28 - q(2)(0)(1)#34  = -31
 WarehouseInventoryBalance(2,2,1): - I_warehouse(1)(0)(0)#12
                                 + I_warehouse(1)(1)(0)#13
                                 - b_warehouse(1)(1)(0)#19 - q(0)(1)(1)#25
                                 - q(1)(1)(1)#31 - q(2)(1)(1)#37  = -36
 WarehouseInventoryBalance(1,3,1): - I_warehouse(0)(1)(0)#10
                                 + I_warehouse(0)(2)(0)#11
                                 - b_warehouse(0)(2)(0)#17 - q(0)(0)(2)#23
                                 - q(1)(0)(2)#29 - q(2)(0)(2)#35  = -74
 WarehouseInventoryBalance(2,3,1): - I_warehouse(1)(1)(0)#13
                                 + I_warehouse(1)(2)(0)#14
                                 - b_warehouse(1)(2)(0)#20 - q(0)(1)(2)#26
                                 - q(1)(1)(2)#32 - q(2)(1)(2)#38  = -49
 WarehouseCoverage(1,1):         o(0)(0)#39 + o(2)(0)#45 <= 1
 WarehouseCoverage(2,1):         o(1)(0)#42 + o(2)(0)#45 <= 1
 WarehouseCoverage(1,2):         o(0)(1)#40 + o(2)(1)#46 <= 1
 WarehouseCoverage(2,2):         o(1)(1)#43 + o(2)(1)#46 <= 1
 WarehouseCoverage(1,3):         o(0)(2)#41 + o(2)(2)#47 <= 1
 WarehouseCoverage(2,3):         o(1)(2)#44 + o(2)(2)#47 <= 1
 VehicleCapacity(1,1):           q(0)(0)(0)#21 + q(0)(1)(0)#24 - 102 o(0)(0)#39
                                 <= 0
 VehicleCapacity(2,1):           q(1)(0)(0)#27 + q(1)(1)(0)#30 - 102 o(1)(0)#42
                                 <= 0
 VehicleCapacity(3,1):           q(2)(0)(0)#33 + q(2)(1)(0)#36 - 102 o(2)(0)#45
                                 <= 0
 VehicleCapacity(1,2):           q(0)(0)(1)#22 + q(0)(1)(1)#25 - 102 o(0)(1)#40
                                 <= 0
 VehicleCapacity(2,2):           q(1)(0)(1)#28 + q(1)(1)(1)#31 - 102 o(1)(1)#43
                                 <= 0
 VehicleCapacity(3,2):           q(2)(0)(1)#34 + q(2)(1)(1)#37 - 102 o(2)(1)#46
                                 <= 0
 VehicleCapacity(1,3):           q(0)(0)(2)#23 + q(0)(1)(2)#26 - 102 o(0)(2)#41
                                 <= 0
 VehicleCapacity(2,3):           q(1)(0)(2)#29 + q(1)(1)(2)#32 - 102 o(1)(2)#44
                                 <= 0
 VehicleCapacity(3,3):           q(2)(0)(2)#35 + q(2)(1)(2)#38 - 102 o(2)(2)#47
                                 <= 0
 FleetSize(1):                   o(0)(0)#39 + o(1)(0)#42 + o(2)(0)#45 <= 2
 FleetSize(2):                   o(0)(1)#40 + o(1)(1)#43 + o(2)(1)#46 <= 2
 FleetSize(3):                   o(0)(2)#41 + o(1)(2)#44 + o(2)(2)#47 <= 2
Bounds
 0 <= y(0)#0 <= 1
 0 <= y(1)#1 <= 1
 0 <= y(2)#2 <= 1
 0 <= p(0)#3 <= 304
 0 <= p(1)#4 <= 304
 0 <= p(2)#5 <= 304
 0 <= I_plant(0)#6 <= 152
 0 <= I_plant(1)#7 <= 152
 0 <= I_plant(2)#8 <= 152
 0 <= I_warehouse(0)(0)(0)#9 <= 136.8
 0 <= I_warehouse(0)(1)(0)#10 <= 136.8
 0 <= I_warehouse(0)(2)(0)#11 <= 136.8
 0 <= I_warehouse(1)(0)(0)#12 <= 136.8
 0 <= I_warehouse(1)(1)(0)#13 <= 136.8
 0 <= I_warehouse(1)(2)(0)#14 <= 136.8
 0 <= b_warehouse(0)(0)(0)#15 <= 9
 0 <= b_warehouse(0)(1)(0)#16 <= 31
 0 <= b_warehouse(0)(2)(0)#17 <= 74
 0 <= b_warehouse(1)(0)(0)#18 <= 10
 0 <= b_warehouse(1)(1)(0)#19 <= 36
 0 <= b_warehouse(1)(2)(0)#20 <= 49
 0 <= q(0)(0)(0)#21 <= 136.8
 0 <= q(0)(0)(1)#22 <= 136.8
 0 <= q(0)(0)(2)#23 <= 136.8
      q(0)(1)(0)#24 = 0
      q(0)(1)(1)#25 = 0
      q(0)(1)(2)#26 = 0
      q(1)(0)(0)#27 = 0
      q(1)(0)(1)#28 = 0
      q(1)(0)(2)#29 = 0
 0 <= q(1)(1)(0)#30 <= 136.8
 0 <= q(1)(1)(1)#31 <= 136.8
 0 <= q(1)(1)(2)#32 <= 136.8
 0 <= q(2)(0)(0)#33 <= 136.8
 0 <= q(2)(0)(1)#34 <= 136.8
 0 <= q(2)(0)(2)#35 <= 136.8
 0 <= q(2)(1)(0)#36 <= 136.8
 0 <= q(2)(1)(1)#37 <= 136.8
 0 <= q(2)(1)(2)#38 <= 136.8
 0 <= o(0)(0)#39 <= 1
 0 <= o(0)(1)#40 <= 1
 0 <= o(0)(2)#41 <= 1
 0 <= o(1)(0)#42 <= 1
 0 <= o(1)(1)#43 <= 1
 0 <= o(1)(2)#44 <= 1
 0 <= o(2)(0)#45 <= 1
 0 <= o(2)(1)#46 <= 1
 0 <= o(2)(2)#47 <= 1
Binaries
 y(0)#0  y(1)#1  y(2)#2  o(0)(0)#39  o(0)(1)#40  o(0)(2)#41  o(1)(0)#42 
 o(1)(1)#43  o(1)(2)#44  o(2)(0)#45  o(2)(1)#46  o(2)(2)#47 
End
