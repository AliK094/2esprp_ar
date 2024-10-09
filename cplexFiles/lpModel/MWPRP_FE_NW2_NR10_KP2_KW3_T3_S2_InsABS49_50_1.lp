\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 3000 y(0)#0 + 3000 y(1)#1 + 3000 y(2)#2 + 30 p(0)#3 + 30 p(1)#4
       + 30 p(2)#5 + 3 I_plant(0)#6 + 3 I_plant(1)#7 + 3 I_plant(2)#8
       + 2 I_warehouse(0)(0)(0)#9 + 2 I_warehouse(0)(0)(1)#10
       + 2 I_warehouse(0)(1)(0)#11 + 2 I_warehouse(0)(1)(1)#12
       + 2 I_warehouse(0)(2)(0)#13 + 2 I_warehouse(0)(2)(1)#14
       + 1.5 I_warehouse(1)(0)(0)#15 + 1.5 I_warehouse(1)(0)(1)#16
       + 1.5 I_warehouse(1)(1)(0)#17 + 1.5 I_warehouse(1)(1)(1)#18
       + 1.5 I_warehouse(1)(2)(0)#19 + 1.5 I_warehouse(1)(2)(1)#20
       + 1953.33333333333 b_warehouse(0)(0)(0)#21
       + 1953.33333333333 b_warehouse(0)(0)(1)#22
       + 1953.33333333333 b_warehouse(0)(1)(0)#23
       + 1953.33333333333 b_warehouse(0)(1)(1)#24
       + 1953.33333333333 b_warehouse(0)(2)(0)#25
       + 1953.33333333333 b_warehouse(0)(2)(1)#26
       + 1361.25 b_warehouse(1)(0)(0)#27 + 1361.25 b_warehouse(1)(0)(1)#28
       + 1361.25 b_warehouse(1)(1)(0)#29 + 1361.25 b_warehouse(1)(1)(1)#30
       + 1361.25 b_warehouse(1)(2)(0)#31 + 1361.25 b_warehouse(1)(2)(1)#32
       + 2838.94205647104 o(0)(0)#51 + 2838.94205647104 o(0)(1)#52
       + 2838.94205647104 o(0)(2)#53 + 1739.25271309261 o(1)(0)#54
       + 1739.25271309261 o(1)(1)#55 + 1739.25271309261 o(1)(2)#56
       + 4037.75629949332 o(2)(0)#57 + 4037.75629949332 o(2)(1)#58
       + 4037.75629949332 o(2)(2)#59
Subject To
 ProductionCapacity(1):          - 304 y(0)#0 + p(0)#3 <= 0
 ProductionCapacity(2):          - 304 y(1)#1 + p(1)#4 <= 0
 ProductionCapacity(3):          - 304 y(2)#2 + p(2)#5 <= 0
 PlantInventoryCapacity(1):      I_plant(0)#6 <= 152
 PlantInventoryCapacity(2):      I_plant(1)#7 <= 152
 PlantInventoryCapacity(3):      I_plant(2)#8 <= 152
 PlantInventoryBalance(1):       - p(0)#3 + I_plant(0)#6 + q(0)(0)(0)#33
                                 + q(0)(1)(0)#36 + q(1)(0)(0)#39
                                 + q(1)(1)(0)#42 + q(2)(0)(0)#45
                                 + q(2)(1)(0)#48  = 0
 PlantInventoryBalance(2):       - p(1)#4 - I_plant(0)#6 + I_plant(1)#7
                                 + q(0)(0)(1)#34 + q(0)(1)(1)#37
                                 + q(1)(0)(1)#40 + q(1)(1)(1)#43
                                 + q(2)(0)(1)#46 + q(2)(1)(1)#49  = 0
 PlantInventoryBalance(3):       - p(2)#5 - I_plant(1)#7 + I_plant(2)#8
                                 + q(0)(0)(2)#35 + q(0)(1)(2)#38
                                 + q(1)(0)(2)#41 + q(1)(1)(2)#44
                                 + q(2)(0)(2)#47 + q(2)(1)(2)#50  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(0)(0)(0)#9 <= 136.8
 WarehouseInventoryCapacity(2,1,1): I_warehouse(1)(0)(0)#15 <= 136.8
 WarehouseInventoryCapacity(1,2,1): I_warehouse(0)(1)(0)#11 <= 136.8
 WarehouseInventoryCapacity(2,2,1): I_warehouse(1)(1)(0)#17 <= 136.8
 WarehouseInventoryCapacity(1,3,1): I_warehouse(0)(2)(0)#13 <= 136.8
 WarehouseInventoryCapacity(2,3,1): I_warehouse(1)(2)(0)#19 <= 136.8
 WarehouseInventoryCapacity(1,1,2): I_warehouse(0)(0)(1)#10 <= 136.8
 WarehouseInventoryCapacity(2,1,2): I_warehouse(1)(0)(1)#16 <= 136.8
 WarehouseInventoryCapacity(1,2,2): I_warehouse(0)(1)(1)#12 <= 136.8
 WarehouseInventoryCapacity(2,2,2): I_warehouse(1)(1)(1)#18 <= 136.8
 WarehouseInventoryCapacity(1,3,2): I_warehouse(0)(2)(1)#14 <= 136.8
 WarehouseInventoryCapacity(2,3,2): I_warehouse(1)(2)(1)#20 <= 136.8
 WarehouseInventoryBalance(1,1,1): I_warehouse(0)(0)(0)#9
                                 - b_warehouse(0)(0)(0)#21 - q(0)(0)(0)#33
                                 - q(1)(0)(0)#39 - q(2)(0)(0)#45  = -9
 WarehouseInventoryBalance(2,1,1): I_warehouse(1)(0)(0)#15
                                 - b_warehouse(1)(0)(0)#27 - q(0)(1)(0)#36
                                 - q(1)(1)(0)#42 - q(2)(1)(0)#48  = -10
 WarehouseInventoryBalance(1,2,1): - I_warehouse(0)(0)(0)#9
                                 + I_warehouse(0)(1)(0)#11
                                 - b_warehouse(0)(1)(0)#23 - q(0)(0)(1)#34
                                 - q(1)(0)(1)#40 - q(2)(0)(1)#46  = -31
 WarehouseInventoryBalance(2,2,1): - I_warehouse(1)(0)(0)#15
                                 + I_warehouse(1)(1)(0)#17
                                 - b_warehouse(1)(1)(0)#29 - q(0)(1)(1)#37
                                 - q(1)(1)(1)#43 - q(2)(1)(1)#49  = -36
 WarehouseInventoryBalance(1,3,1): - I_warehouse(0)(1)(0)#11
                                 + I_warehouse(0)(2)(0)#13
                                 - b_warehouse(0)(2)(0)#25 - q(0)(0)(2)#35
                                 - q(1)(0)(2)#41 - q(2)(0)(2)#47  = -74
 WarehouseInventoryBalance(2,3,1): - I_warehouse(1)(1)(0)#17
                                 + I_warehouse(1)(2)(0)#19
                                 - b_warehouse(1)(2)(0)#31 - q(0)(1)(2)#38
                                 - q(1)(1)(2)#44 - q(2)(1)(2)#50  = -49
 WarehouseInventoryBalance(1,1,2): I_warehouse(0)(0)(1)#10
                                 - b_warehouse(0)(0)(1)#22 - q(0)(0)(0)#33
                                 - q(1)(0)(0)#39 - q(2)(0)(0)#45  = -89
 WarehouseInventoryBalance(2,1,2): I_warehouse(1)(0)(1)#16
                                 - b_warehouse(1)(0)(1)#28 - q(0)(1)(0)#36
                                 - q(1)(1)(0)#42 - q(2)(1)(0)#48  = -51
 WarehouseInventoryBalance(1,2,2): - I_warehouse(0)(0)(1)#10
                                 + I_warehouse(0)(1)(1)#12
                                 - b_warehouse(0)(1)(1)#24 - q(0)(0)(1)#34
                                 - q(1)(0)(1)#40 - q(2)(0)(1)#46  = -99
 WarehouseInventoryBalance(2,2,2): - I_warehouse(1)(0)(1)#16
                                 + I_warehouse(1)(1)(1)#18
                                 - b_warehouse(1)(1)(1)#30 - q(0)(1)(1)#37
                                 - q(1)(1)(1)#43 - q(2)(1)(1)#49  = -55
 WarehouseInventoryBalance(1,3,2): - I_warehouse(0)(1)(1)#12
                                 + I_warehouse(0)(2)(1)#14
                                 - b_warehouse(0)(2)(1)#26 - q(0)(0)(2)#35
                                 - q(1)(0)(2)#41 - q(2)(0)(2)#47  = -104
 WarehouseInventoryBalance(2,3,2): - I_warehouse(1)(1)(1)#18
                                 + I_warehouse(1)(2)(1)#20
                                 - b_warehouse(1)(2)(1)#32 - q(0)(1)(2)#38
                                 - q(1)(1)(2)#44 - q(2)(1)(2)#50  = -51
 WarehouseCoverage(1,1):         o(0)(0)#51 + o(2)(0)#57 <= 1
 WarehouseCoverage(2,1):         o(1)(0)#54 + o(2)(0)#57 <= 1
 WarehouseCoverage(1,2):         o(0)(1)#52 + o(2)(1)#58 <= 1
 WarehouseCoverage(2,2):         o(1)(1)#55 + o(2)(1)#58 <= 1
 WarehouseCoverage(1,3):         o(0)(2)#53 + o(2)(2)#59 <= 1
 WarehouseCoverage(2,3):         o(1)(2)#56 + o(2)(2)#59 <= 1
 VehicleCapacity(1,1):           q(0)(0)(0)#33 + q(0)(1)(0)#36 - 102 o(0)(0)#51
                                 <= 0
 VehicleCapacity(2,1):           q(1)(0)(0)#39 + q(1)(1)(0)#42 - 102 o(1)(0)#54
                                 <= 0
 VehicleCapacity(3,1):           q(2)(0)(0)#45 + q(2)(1)(0)#48 - 102 o(2)(0)#57
                                 <= 0
 VehicleCapacity(1,2):           q(0)(0)(1)#34 + q(0)(1)(1)#37 - 102 o(0)(1)#52
                                 <= 0
 VehicleCapacity(2,2):           q(1)(0)(1)#40 + q(1)(1)(1)#43 - 102 o(1)(1)#55
                                 <= 0
 VehicleCapacity(3,2):           q(2)(0)(1)#46 + q(2)(1)(1)#49 - 102 o(2)(1)#58
                                 <= 0
 VehicleCapacity(1,3):           q(0)(0)(2)#35 + q(0)(1)(2)#38 - 102 o(0)(2)#53
                                 <= 0
 VehicleCapacity(2,3):           q(1)(0)(2)#41 + q(1)(1)(2)#44 - 102 o(1)(2)#56
                                 <= 0
 VehicleCapacity(3,3):           q(2)(0)(2)#47 + q(2)(1)(2)#50 - 102 o(2)(2)#59
                                 <= 0
 FleetSize(1):                   o(0)(0)#51 + o(1)(0)#54 + o(2)(0)#57 <= 2
 FleetSize(2):                   o(0)(1)#52 + o(1)(1)#55 + o(2)(1)#58 <= 2
 FleetSize(3):                   o(0)(2)#53 + o(1)(2)#56 + o(2)(2)#59 <= 2
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
 0 <= I_warehouse(0)(0)(1)#10 <= 136.8
 0 <= I_warehouse(0)(1)(0)#11 <= 136.8
 0 <= I_warehouse(0)(1)(1)#12 <= 136.8
 0 <= I_warehouse(0)(2)(0)#13 <= 136.8
 0 <= I_warehouse(0)(2)(1)#14 <= 136.8
 0 <= I_warehouse(1)(0)(0)#15 <= 136.8
 0 <= I_warehouse(1)(0)(1)#16 <= 136.8
 0 <= I_warehouse(1)(1)(0)#17 <= 136.8
 0 <= I_warehouse(1)(1)(1)#18 <= 136.8
 0 <= I_warehouse(1)(2)(0)#19 <= 136.8
 0 <= I_warehouse(1)(2)(1)#20 <= 136.8
 0 <= b_warehouse(0)(0)(0)#21 <= 9
 0 <= b_warehouse(0)(0)(1)#22 <= 89
 0 <= b_warehouse(0)(1)(0)#23 <= 31
 0 <= b_warehouse(0)(1)(1)#24 <= 99
 0 <= b_warehouse(0)(2)(0)#25 <= 74
 0 <= b_warehouse(0)(2)(1)#26 <= 104
 0 <= b_warehouse(1)(0)(0)#27 <= 10
 0 <= b_warehouse(1)(0)(1)#28 <= 51
 0 <= b_warehouse(1)(1)(0)#29 <= 36
 0 <= b_warehouse(1)(1)(1)#30 <= 55
 0 <= b_warehouse(1)(2)(0)#31 <= 49
 0 <= b_warehouse(1)(2)(1)#32 <= 51
 0 <= q(0)(0)(0)#33 <= 136.8
 0 <= q(0)(0)(1)#34 <= 136.8
 0 <= q(0)(0)(2)#35 <= 136.8
      q(0)(1)(0)#36 = 0
      q(0)(1)(1)#37 = 0
      q(0)(1)(2)#38 = 0
      q(1)(0)(0)#39 = 0
      q(1)(0)(1)#40 = 0
      q(1)(0)(2)#41 = 0
 0 <= q(1)(1)(0)#42 <= 136.8
 0 <= q(1)(1)(1)#43 <= 136.8
 0 <= q(1)(1)(2)#44 <= 136.8
 0 <= q(2)(0)(0)#45 <= 136.8
 0 <= q(2)(0)(1)#46 <= 136.8
 0 <= q(2)(0)(2)#47 <= 136.8
 0 <= q(2)(1)(0)#48 <= 136.8
 0 <= q(2)(1)(1)#49 <= 136.8
 0 <= q(2)(1)(2)#50 <= 136.8
 0 <= o(0)(0)#51 <= 1
 0 <= o(0)(1)#52 <= 1
 0 <= o(0)(2)#53 <= 1
 0 <= o(1)(0)#54 <= 1
 0 <= o(1)(1)#55 <= 1
 0 <= o(1)(2)#56 <= 1
 0 <= o(2)(0)#57 <= 1
 0 <= o(2)(1)#58 <= 1
 0 <= o(2)(2)#59 <= 1
Binaries
 y(0)#0  y(1)#1  y(2)#2  o(0)(0)#51  o(0)(1)#52  o(0)(2)#53  o(1)(0)#54 
 o(1)(1)#55  o(1)(2)#56  o(2)(0)#57  o(2)(1)#58  o(2)(2)#59 
End
