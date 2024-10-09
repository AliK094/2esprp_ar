\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 3000 y(0)#0 + 3000 y(1)#1 + 3000 y(2)#2 + 30 p(0)#3 + 30 p(1)#4
       + 30 p(2)#5 + 3 I_plant(0)#6 + 3 I_plant(1)#7 + 3 I_plant(2)#8
       + 0.8 I_warehouse(0)(0)(0)#9 + 0.8 I_warehouse(0)(0)(1)#10
       + 0.8 I_warehouse(0)(0)(2)#11 + 0.8 I_warehouse(0)(0)(3)#12
       + 0.8 I_warehouse(0)(0)(4)#13 + 0.8 I_warehouse(0)(1)(0)#14
       + 0.8 I_warehouse(0)(1)(1)#15 + 0.8 I_warehouse(0)(1)(2)#16
       + 0.8 I_warehouse(0)(1)(3)#17 + 0.8 I_warehouse(0)(1)(4)#18
       + 0.8 I_warehouse(0)(2)(0)#19 + 0.8 I_warehouse(0)(2)(1)#20
       + 0.8 I_warehouse(0)(2)(2)#21 + 0.8 I_warehouse(0)(2)(3)#22
       + 0.8 I_warehouse(0)(2)(4)#23 + 0.6 I_warehouse(1)(0)(0)#24
       + 0.6 I_warehouse(1)(0)(1)#25 + 0.6 I_warehouse(1)(0)(2)#26
       + 0.6 I_warehouse(1)(0)(3)#27 + 0.6 I_warehouse(1)(0)(4)#28
       + 0.6 I_warehouse(1)(1)(0)#29 + 0.6 I_warehouse(1)(1)(1)#30
       + 0.6 I_warehouse(1)(1)(2)#31 + 0.6 I_warehouse(1)(1)(3)#32
       + 0.6 I_warehouse(1)(1)(4)#33 + 0.6 I_warehouse(1)(2)(0)#34
       + 0.6 I_warehouse(1)(2)(1)#35 + 0.6 I_warehouse(1)(2)(2)#36
       + 0.6 I_warehouse(1)(2)(3)#37 + 0.6 I_warehouse(1)(2)(4)#38
       + 781.333333333333 b_warehouse(0)(0)(0)#39
       + 781.333333333333 b_warehouse(0)(0)(1)#40
       + 781.333333333333 b_warehouse(0)(0)(2)#41
       + 781.333333333333 b_warehouse(0)(0)(3)#42
       + 781.333333333333 b_warehouse(0)(0)(4)#43
       + 781.333333333333 b_warehouse(0)(1)(0)#44
       + 781.333333333333 b_warehouse(0)(1)(1)#45
       + 781.333333333333 b_warehouse(0)(1)(2)#46
       + 781.333333333333 b_warehouse(0)(1)(3)#47
       + 781.333333333333 b_warehouse(0)(1)(4)#48
       + 781.333333333333 b_warehouse(0)(2)(0)#49
       + 781.333333333333 b_warehouse(0)(2)(1)#50
       + 781.333333333333 b_warehouse(0)(2)(2)#51
       + 781.333333333333 b_warehouse(0)(2)(3)#52
       + 781.333333333333 b_warehouse(0)(2)(4)#53
       + 544.5 b_warehouse(1)(0)(0)#54 + 544.5 b_warehouse(1)(0)(1)#55
       + 544.5 b_warehouse(1)(0)(2)#56 + 544.5 b_warehouse(1)(0)(3)#57
       + 544.5 b_warehouse(1)(0)(4)#58 + 544.5 b_warehouse(1)(1)(0)#59
       + 544.5 b_warehouse(1)(1)(1)#60 + 544.5 b_warehouse(1)(1)(2)#61
       + 544.5 b_warehouse(1)(1)(3)#62 + 544.5 b_warehouse(1)(1)(4)#63
       + 544.5 b_warehouse(1)(2)(0)#64 + 544.5 b_warehouse(1)(2)(1)#65
       + 544.5 b_warehouse(1)(2)(2)#66 + 544.5 b_warehouse(1)(2)(3)#67
       + 544.5 b_warehouse(1)(2)(4)#68 + 2838.94205647104 o(0)(0)#87
       + 2838.94205647104 o(0)(1)#88 + 2838.94205647104 o(0)(2)#89
       + 1739.25271309261 o(1)(0)#90 + 1739.25271309261 o(1)(1)#91
       + 1739.25271309261 o(1)(2)#92 + 4037.75629949332 o(2)(0)#93
       + 4037.75629949332 o(2)(1)#94 + 4037.75629949332 o(2)(2)#95
Subject To
 ProductionCapacity(1):          - 304 y(0)#0 + p(0)#3 <= 0
 ProductionCapacity(2):          - 304 y(1)#1 + p(1)#4 <= 0
 ProductionCapacity(3):          - 304 y(2)#2 + p(2)#5 <= 0
 PlantInventoryCapacity(1):      I_plant(0)#6 <= 152
 PlantInventoryCapacity(2):      I_plant(1)#7 <= 152
 PlantInventoryCapacity(3):      I_plant(2)#8 <= 152
 PlantInventoryBalance(1):       - p(0)#3 + I_plant(0)#6 + q(0)(0)(0)#69
                                 + q(0)(1)(0)#72 + q(1)(0)(0)#75
                                 + q(1)(1)(0)#78 + q(2)(0)(0)#81
                                 + q(2)(1)(0)#84  = 0
 PlantInventoryBalance(2):       - p(1)#4 - I_plant(0)#6 + I_plant(1)#7
                                 + q(0)(0)(1)#70 + q(0)(1)(1)#73
                                 + q(1)(0)(1)#76 + q(1)(1)(1)#79
                                 + q(2)(0)(1)#82 + q(2)(1)(1)#85  = 0
 PlantInventoryBalance(3):       - p(2)#5 - I_plant(1)#7 + I_plant(2)#8
                                 + q(0)(0)(2)#71 + q(0)(1)(2)#74
                                 + q(1)(0)(2)#77 + q(1)(1)(2)#80
                                 + q(2)(0)(2)#83 + q(2)(1)(2)#86  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(0)(0)(0)#9 <= 136.8
 WarehouseInventoryCapacity(2,1,1): I_warehouse(1)(0)(0)#24 <= 136.8
 WarehouseInventoryCapacity(1,2,1): I_warehouse(0)(1)(0)#14 <= 136.8
 WarehouseInventoryCapacity(2,2,1): I_warehouse(1)(1)(0)#29 <= 136.8
 WarehouseInventoryCapacity(1,3,1): I_warehouse(0)(2)(0)#19 <= 136.8
 WarehouseInventoryCapacity(2,3,1): I_warehouse(1)(2)(0)#34 <= 136.8
 WarehouseInventoryCapacity(1,1,2): I_warehouse(0)(0)(1)#10 <= 136.8
 WarehouseInventoryCapacity(2,1,2): I_warehouse(1)(0)(1)#25 <= 136.8
 WarehouseInventoryCapacity(1,2,2): I_warehouse(0)(1)(1)#15 <= 136.8
 WarehouseInventoryCapacity(2,2,2): I_warehouse(1)(1)(1)#30 <= 136.8
 WarehouseInventoryCapacity(1,3,2): I_warehouse(0)(2)(1)#20 <= 136.8
 WarehouseInventoryCapacity(2,3,2): I_warehouse(1)(2)(1)#35 <= 136.8
 WarehouseInventoryCapacity(1,1,3): I_warehouse(0)(0)(2)#11 <= 136.8
 WarehouseInventoryCapacity(2,1,3): I_warehouse(1)(0)(2)#26 <= 136.8
 WarehouseInventoryCapacity(1,2,3): I_warehouse(0)(1)(2)#16 <= 136.8
 WarehouseInventoryCapacity(2,2,3): I_warehouse(1)(1)(2)#31 <= 136.8
 WarehouseInventoryCapacity(1,3,3): I_warehouse(0)(2)(2)#21 <= 136.8
 WarehouseInventoryCapacity(2,3,3): I_warehouse(1)(2)(2)#36 <= 136.8
 WarehouseInventoryCapacity(1,1,4): I_warehouse(0)(0)(3)#12 <= 136.8
 WarehouseInventoryCapacity(2,1,4): I_warehouse(1)(0)(3)#27 <= 136.8
 WarehouseInventoryCapacity(1,2,4): I_warehouse(0)(1)(3)#17 <= 136.8
 WarehouseInventoryCapacity(2,2,4): I_warehouse(1)(1)(3)#32 <= 136.8
 WarehouseInventoryCapacity(1,3,4): I_warehouse(0)(2)(3)#22 <= 136.8
 WarehouseInventoryCapacity(2,3,4): I_warehouse(1)(2)(3)#37 <= 136.8
 WarehouseInventoryCapacity(1,1,5): I_warehouse(0)(0)(4)#13 <= 136.8
 WarehouseInventoryCapacity(2,1,5): I_warehouse(1)(0)(4)#28 <= 136.8
 WarehouseInventoryCapacity(1,2,5): I_warehouse(0)(1)(4)#18 <= 136.8
 WarehouseInventoryCapacity(2,2,5): I_warehouse(1)(1)(4)#33 <= 136.8
 WarehouseInventoryCapacity(1,3,5): I_warehouse(0)(2)(4)#23 <= 136.8
 WarehouseInventoryCapacity(2,3,5): I_warehouse(1)(2)(4)#38 <= 136.8
 WarehouseInventoryBalance(1,1,1): I_warehouse(0)(0)(0)#9
                                 - b_warehouse(0)(0)(0)#39 - q(0)(0)(0)#69
                                 - q(1)(0)(0)#75 - q(2)(0)(0)#81  = -9
 WarehouseInventoryBalance(2,1,1): I_warehouse(1)(0)(0)#24
                                 - b_warehouse(1)(0)(0)#54 - q(0)(1)(0)#72
                                 - q(1)(1)(0)#78 - q(2)(1)(0)#84  = -10
 WarehouseInventoryBalance(1,2,1): - I_warehouse(0)(0)(0)#9
                                 + I_warehouse(0)(1)(0)#14
                                 - b_warehouse(0)(1)(0)#44 - q(0)(0)(1)#70
                                 - q(1)(0)(1)#76 - q(2)(0)(1)#82  = -31
 WarehouseInventoryBalance(2,2,1): - I_warehouse(1)(0)(0)#24
                                 + I_warehouse(1)(1)(0)#29
                                 - b_warehouse(1)(1)(0)#59 - q(0)(1)(1)#73
                                 - q(1)(1)(1)#79 - q(2)(1)(1)#85  = -36
 WarehouseInventoryBalance(1,3,1): - I_warehouse(0)(1)(0)#14
                                 + I_warehouse(0)(2)(0)#19
                                 - b_warehouse(0)(2)(0)#49 - q(0)(0)(2)#71
                                 - q(1)(0)(2)#77 - q(2)(0)(2)#83  = -74
 WarehouseInventoryBalance(2,3,1): - I_warehouse(1)(1)(0)#29
                                 + I_warehouse(1)(2)(0)#34
                                 - b_warehouse(1)(2)(0)#64 - q(0)(1)(2)#74
                                 - q(1)(1)(2)#80 - q(2)(1)(2)#86  = -49
 WarehouseInventoryBalance(1,1,2): I_warehouse(0)(0)(1)#10
                                 - b_warehouse(0)(0)(1)#40 - q(0)(0)(0)#69
                                 - q(1)(0)(0)#75 - q(2)(0)(0)#81  = -89
 WarehouseInventoryBalance(2,1,2): I_warehouse(1)(0)(1)#25
                                 - b_warehouse(1)(0)(1)#55 - q(0)(1)(0)#72
                                 - q(1)(1)(0)#78 - q(2)(1)(0)#84  = -51
 WarehouseInventoryBalance(1,2,2): - I_warehouse(0)(0)(1)#10
                                 + I_warehouse(0)(1)(1)#15
                                 - b_warehouse(0)(1)(1)#45 - q(0)(0)(1)#70
                                 - q(1)(0)(1)#76 - q(2)(0)(1)#82  = -99
 WarehouseInventoryBalance(2,2,2): - I_warehouse(1)(0)(1)#25
                                 + I_warehouse(1)(1)(1)#30
                                 - b_warehouse(1)(1)(1)#60 - q(0)(1)(1)#73
                                 - q(1)(1)(1)#79 - q(2)(1)(1)#85  = -55
 WarehouseInventoryBalance(1,3,2): - I_warehouse(0)(1)(1)#15
                                 + I_warehouse(0)(2)(1)#20
                                 - b_warehouse(0)(2)(1)#50 - q(0)(0)(2)#71
                                 - q(1)(0)(2)#77 - q(2)(0)(2)#83  = -104
 WarehouseInventoryBalance(2,3,2): - I_warehouse(1)(1)(1)#30
                                 + I_warehouse(1)(2)(1)#35
                                 - b_warehouse(1)(2)(1)#65 - q(0)(1)(2)#74
                                 - q(1)(1)(2)#80 - q(2)(1)(2)#86  = -51
 WarehouseInventoryBalance(1,1,3): I_warehouse(0)(0)(2)#11
                                 - b_warehouse(0)(0)(2)#41 - q(0)(0)(0)#69
                                 - q(1)(0)(0)#75 - q(2)(0)(0)#81  = -94
 WarehouseInventoryBalance(2,1,3): I_warehouse(1)(0)(2)#26
                                 - b_warehouse(1)(0)(2)#56 - q(0)(1)(0)#72
                                 - q(1)(1)(0)#78 - q(2)(1)(0)#84  = -60
 WarehouseInventoryBalance(1,2,3): - I_warehouse(0)(0)(2)#11
                                 + I_warehouse(0)(1)(2)#16
                                 - b_warehouse(0)(1)(2)#46 - q(0)(0)(1)#70
                                 - q(1)(0)(1)#76 - q(2)(0)(1)#82  = -100
 WarehouseInventoryBalance(2,2,3): - I_warehouse(1)(0)(2)#26
                                 + I_warehouse(1)(1)(2)#31
                                 - b_warehouse(1)(1)(2)#61 - q(0)(1)(1)#73
                                 - q(1)(1)(1)#79 - q(2)(1)(1)#85  = -58
 WarehouseInventoryBalance(1,3,3): - I_warehouse(0)(1)(2)#16
                                 + I_warehouse(0)(2)(2)#21
                                 - b_warehouse(0)(2)(2)#51 - q(0)(0)(2)#71
                                 - q(1)(0)(2)#77 - q(2)(0)(2)#83  = -102
 WarehouseInventoryBalance(2,3,3): - I_warehouse(1)(1)(2)#31
                                 + I_warehouse(1)(2)(2)#36
                                 - b_warehouse(1)(2)(2)#66 - q(0)(1)(2)#74
                                 - q(1)(1)(2)#80 - q(2)(1)(2)#86  = -58
 WarehouseInventoryBalance(1,1,4): I_warehouse(0)(0)(3)#12
                                 - b_warehouse(0)(0)(3)#42 - q(0)(0)(0)#69
                                 - q(1)(0)(0)#75 - q(2)(0)(0)#81  = -103
 WarehouseInventoryBalance(2,1,4): I_warehouse(1)(0)(3)#27
                                 - b_warehouse(1)(0)(3)#57 - q(0)(1)(0)#72
                                 - q(1)(1)(0)#78 - q(2)(1)(0)#84  = -57
 WarehouseInventoryBalance(1,2,4): - I_warehouse(0)(0)(3)#12
                                 + I_warehouse(0)(1)(3)#17
                                 - b_warehouse(0)(1)(3)#47 - q(0)(0)(1)#70
                                 - q(1)(0)(1)#76 - q(2)(0)(1)#82  = -105
 WarehouseInventoryBalance(2,2,4): - I_warehouse(1)(0)(3)#27
                                 + I_warehouse(1)(1)(3)#32
                                 - b_warehouse(1)(1)(3)#62 - q(0)(1)(1)#73
                                 - q(1)(1)(1)#79 - q(2)(1)(1)#85  = -52
 WarehouseInventoryBalance(1,3,4): - I_warehouse(0)(1)(3)#17
                                 + I_warehouse(0)(2)(3)#22
                                 - b_warehouse(0)(2)(3)#52 - q(0)(0)(2)#71
                                 - q(1)(0)(2)#77 - q(2)(0)(2)#83  = -95
 WarehouseInventoryBalance(2,3,4): - I_warehouse(1)(1)(3)#32
                                 + I_warehouse(1)(2)(3)#37
                                 - b_warehouse(1)(2)(3)#67 - q(0)(1)(2)#74
                                 - q(1)(1)(2)#80 - q(2)(1)(2)#86  = -50
 WarehouseInventoryBalance(1,1,5): I_warehouse(0)(0)(4)#13
                                 - b_warehouse(0)(0)(4)#43 - q(0)(0)(0)#69
                                 - q(1)(0)(0)#75 - q(2)(0)(0)#81  = -106
 WarehouseInventoryBalance(2,1,5): I_warehouse(1)(0)(4)#28
                                 - b_warehouse(1)(0)(4)#58 - q(0)(1)(0)#72
                                 - q(1)(1)(0)#78 - q(2)(1)(0)#84  = -59
 WarehouseInventoryBalance(1,2,5): - I_warehouse(0)(0)(4)#13
                                 + I_warehouse(0)(1)(4)#18
                                 - b_warehouse(0)(1)(4)#48 - q(0)(0)(1)#70
                                 - q(1)(0)(1)#76 - q(2)(0)(1)#82  = -98
 WarehouseInventoryBalance(2,2,5): - I_warehouse(1)(0)(4)#28
                                 + I_warehouse(1)(1)(4)#33
                                 - b_warehouse(1)(1)(4)#63 - q(0)(1)(1)#73
                                 - q(1)(1)(1)#79 - q(2)(1)(1)#85  = -55
 WarehouseInventoryBalance(1,3,5): - I_warehouse(0)(1)(4)#18
                                 + I_warehouse(0)(2)(4)#23
                                 - b_warehouse(0)(2)(4)#53 - q(0)(0)(2)#71
                                 - q(1)(0)(2)#77 - q(2)(0)(2)#83  = -92
 WarehouseInventoryBalance(2,3,5): - I_warehouse(1)(1)(4)#33
                                 + I_warehouse(1)(2)(4)#38
                                 - b_warehouse(1)(2)(4)#68 - q(0)(1)(2)#74
                                 - q(1)(1)(2)#80 - q(2)(1)(2)#86  = -49
 WarehouseCoverage(1,1):         o(0)(0)#87 + o(2)(0)#93 <= 1
 WarehouseCoverage(2,1):         o(1)(0)#90 + o(2)(0)#93 <= 1
 WarehouseCoverage(1,2):         o(0)(1)#88 + o(2)(1)#94 <= 1
 WarehouseCoverage(2,2):         o(1)(1)#91 + o(2)(1)#94 <= 1
 WarehouseCoverage(1,3):         o(0)(2)#89 + o(2)(2)#95 <= 1
 WarehouseCoverage(2,3):         o(1)(2)#92 + o(2)(2)#95 <= 1
 VehicleCapacity(1,1):           q(0)(0)(0)#69 + q(0)(1)(0)#72 - 102 o(0)(0)#87
                                 <= 0
 VehicleCapacity(2,1):           q(1)(0)(0)#75 + q(1)(1)(0)#78 - 102 o(1)(0)#90
                                 <= 0
 VehicleCapacity(3,1):           q(2)(0)(0)#81 + q(2)(1)(0)#84 - 102 o(2)(0)#93
                                 <= 0
 VehicleCapacity(1,2):           q(0)(0)(1)#70 + q(0)(1)(1)#73 - 102 o(0)(1)#88
                                 <= 0
 VehicleCapacity(2,2):           q(1)(0)(1)#76 + q(1)(1)(1)#79 - 102 o(1)(1)#91
                                 <= 0
 VehicleCapacity(3,2):           q(2)(0)(1)#82 + q(2)(1)(1)#85 - 102 o(2)(1)#94
                                 <= 0
 VehicleCapacity(1,3):           q(0)(0)(2)#71 + q(0)(1)(2)#74 - 102 o(0)(2)#89
                                 <= 0
 VehicleCapacity(2,3):           q(1)(0)(2)#77 + q(1)(1)(2)#80 - 102 o(1)(2)#92
                                 <= 0
 VehicleCapacity(3,3):           q(2)(0)(2)#83 + q(2)(1)(2)#86 - 102 o(2)(2)#95
                                 <= 0
 FleetSize(1):                   o(0)(0)#87 + o(1)(0)#90 + o(2)(0)#93 <= 2
 FleetSize(2):                   o(0)(1)#88 + o(1)(1)#91 + o(2)(1)#94 <= 2
 FleetSize(3):                   o(0)(2)#89 + o(1)(2)#92 + o(2)(2)#95 <= 2
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
 0 <= I_warehouse(0)(0)(2)#11 <= 136.8
 0 <= I_warehouse(0)(0)(3)#12 <= 136.8
 0 <= I_warehouse(0)(0)(4)#13 <= 136.8
 0 <= I_warehouse(0)(1)(0)#14 <= 136.8
 0 <= I_warehouse(0)(1)(1)#15 <= 136.8
 0 <= I_warehouse(0)(1)(2)#16 <= 136.8
 0 <= I_warehouse(0)(1)(3)#17 <= 136.8
 0 <= I_warehouse(0)(1)(4)#18 <= 136.8
 0 <= I_warehouse(0)(2)(0)#19 <= 136.8
 0 <= I_warehouse(0)(2)(1)#20 <= 136.8
 0 <= I_warehouse(0)(2)(2)#21 <= 136.8
 0 <= I_warehouse(0)(2)(3)#22 <= 136.8
 0 <= I_warehouse(0)(2)(4)#23 <= 136.8
 0 <= I_warehouse(1)(0)(0)#24 <= 136.8
 0 <= I_warehouse(1)(0)(1)#25 <= 136.8
 0 <= I_warehouse(1)(0)(2)#26 <= 136.8
 0 <= I_warehouse(1)(0)(3)#27 <= 136.8
 0 <= I_warehouse(1)(0)(4)#28 <= 136.8
 0 <= I_warehouse(1)(1)(0)#29 <= 136.8
 0 <= I_warehouse(1)(1)(1)#30 <= 136.8
 0 <= I_warehouse(1)(1)(2)#31 <= 136.8
 0 <= I_warehouse(1)(1)(3)#32 <= 136.8
 0 <= I_warehouse(1)(1)(4)#33 <= 136.8
 0 <= I_warehouse(1)(2)(0)#34 <= 136.8
 0 <= I_warehouse(1)(2)(1)#35 <= 136.8
 0 <= I_warehouse(1)(2)(2)#36 <= 136.8
 0 <= I_warehouse(1)(2)(3)#37 <= 136.8
 0 <= I_warehouse(1)(2)(4)#38 <= 136.8
 0 <= b_warehouse(0)(0)(0)#39 <= 9
 0 <= b_warehouse(0)(0)(1)#40 <= 89
 0 <= b_warehouse(0)(0)(2)#41 <= 94
 0 <= b_warehouse(0)(0)(3)#42 <= 103
 0 <= b_warehouse(0)(0)(4)#43 <= 106
 0 <= b_warehouse(0)(1)(0)#44 <= 31
 0 <= b_warehouse(0)(1)(1)#45 <= 99
 0 <= b_warehouse(0)(1)(2)#46 <= 100
 0 <= b_warehouse(0)(1)(3)#47 <= 105
 0 <= b_warehouse(0)(1)(4)#48 <= 98
 0 <= b_warehouse(0)(2)(0)#49 <= 74
 0 <= b_warehouse(0)(2)(1)#50 <= 104
 0 <= b_warehouse(0)(2)(2)#51 <= 102
 0 <= b_warehouse(0)(2)(3)#52 <= 95
 0 <= b_warehouse(0)(2)(4)#53 <= 92
 0 <= b_warehouse(1)(0)(0)#54 <= 10
 0 <= b_warehouse(1)(0)(1)#55 <= 51
 0 <= b_warehouse(1)(0)(2)#56 <= 60
 0 <= b_warehouse(1)(0)(3)#57 <= 57
 0 <= b_warehouse(1)(0)(4)#58 <= 59
 0 <= b_warehouse(1)(1)(0)#59 <= 36
 0 <= b_warehouse(1)(1)(1)#60 <= 55
 0 <= b_warehouse(1)(1)(2)#61 <= 58
 0 <= b_warehouse(1)(1)(3)#62 <= 52
 0 <= b_warehouse(1)(1)(4)#63 <= 55
 0 <= b_warehouse(1)(2)(0)#64 <= 49
 0 <= b_warehouse(1)(2)(1)#65 <= 51
 0 <= b_warehouse(1)(2)(2)#66 <= 58
 0 <= b_warehouse(1)(2)(3)#67 <= 50
 0 <= b_warehouse(1)(2)(4)#68 <= 49
 0 <= q(0)(0)(0)#69 <= 136.8
 0 <= q(0)(0)(1)#70 <= 136.8
 0 <= q(0)(0)(2)#71 <= 136.8
      q(0)(1)(0)#72 = 0
      q(0)(1)(1)#73 = 0
      q(0)(1)(2)#74 = 0
      q(1)(0)(0)#75 = 0
      q(1)(0)(1)#76 = 0
      q(1)(0)(2)#77 = 0
 0 <= q(1)(1)(0)#78 <= 136.8
 0 <= q(1)(1)(1)#79 <= 136.8
 0 <= q(1)(1)(2)#80 <= 136.8
 0 <= q(2)(0)(0)#81 <= 136.8
 0 <= q(2)(0)(1)#82 <= 136.8
 0 <= q(2)(0)(2)#83 <= 136.8
 0 <= q(2)(1)(0)#84 <= 136.8
 0 <= q(2)(1)(1)#85 <= 136.8
 0 <= q(2)(1)(2)#86 <= 136.8
 0 <= o(0)(0)#87 <= 1
 0 <= o(0)(1)#88 <= 1
 0 <= o(0)(2)#89 <= 1
 0 <= o(1)(0)#90 <= 1
 0 <= o(1)(1)#91 <= 1
 0 <= o(1)(2)#92 <= 1
 0 <= o(2)(0)#93 <= 1
 0 <= o(2)(1)#94 <= 1
 0 <= o(2)(2)#95 <= 1
Binaries
 y(0)#0  y(1)#1  y(2)#2  o(0)(0)#87  o(0)(1)#88  o(0)(2)#89  o(1)(0)#90 
 o(1)(1)#91  o(1)(2)#92  o(2)(0)#93  o(2)(1)#94  o(2)(2)#95 
End
