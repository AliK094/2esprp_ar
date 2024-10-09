\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 3000 y(0)#0 + 3000 y(1)#1 + 3000 y(2)#2 + 3000 y(3)#3 + 3000 y(4)#4
       + 3000 y(5)#5 + 30 p(0)#6 + 30 p(1)#7 + 30 p(2)#8 + 30 p(3)#9
       + 30 p(4)#10 + 30 p(5)#11 + 3 I_plant(0)#12 + 3 I_plant(1)#13
       + 3 I_plant(2)#14 + 3 I_plant(3)#15 + 3 I_plant(4)#16 + 3 I_plant(5)#17
       + 2 I_warehouse(0)(0)(0)#18 + 2 I_warehouse(0)(0)(1)#19
       + 2 I_warehouse(0)(1)(0)#20 + 2 I_warehouse(0)(1)(1)#21
       + 2 I_warehouse(0)(2)(0)#22 + 2 I_warehouse(0)(2)(1)#23
       + 2 I_warehouse(0)(3)(0)#24 + 2 I_warehouse(0)(3)(1)#25
       + 2 I_warehouse(0)(4)(0)#26 + 2 I_warehouse(0)(4)(1)#27
       + 2 I_warehouse(0)(5)(0)#28 + 2 I_warehouse(0)(5)(1)#29
       + 1.5 I_warehouse(1)(0)(0)#30 + 1.5 I_warehouse(1)(0)(1)#31
       + 1.5 I_warehouse(1)(1)(0)#32 + 1.5 I_warehouse(1)(1)(1)#33
       + 1.5 I_warehouse(1)(2)(0)#34 + 1.5 I_warehouse(1)(2)(1)#35
       + 1.5 I_warehouse(1)(3)(0)#36 + 1.5 I_warehouse(1)(3)(1)#37
       + 1.5 I_warehouse(1)(4)(0)#38 + 1.5 I_warehouse(1)(4)(1)#39
       + 1.5 I_warehouse(1)(5)(0)#40 + 1.5 I_warehouse(1)(5)(1)#41
       + 1953.33333333333 b_warehouse(0)(0)(0)#42
       + 1953.33333333333 b_warehouse(0)(0)(1)#43
       + 1953.33333333333 b_warehouse(0)(1)(0)#44
       + 1953.33333333333 b_warehouse(0)(1)(1)#45
       + 1953.33333333333 b_warehouse(0)(2)(0)#46
       + 1953.33333333333 b_warehouse(0)(2)(1)#47
       + 1953.33333333333 b_warehouse(0)(3)(0)#48
       + 1953.33333333333 b_warehouse(0)(3)(1)#49
       + 1953.33333333333 b_warehouse(0)(4)(0)#50
       + 1953.33333333333 b_warehouse(0)(4)(1)#51
       + 1953.33333333333 b_warehouse(0)(5)(0)#52
       + 1953.33333333333 b_warehouse(0)(5)(1)#53
       + 1361.25 b_warehouse(1)(0)(0)#54 + 1361.25 b_warehouse(1)(0)(1)#55
       + 1361.25 b_warehouse(1)(1)(0)#56 + 1361.25 b_warehouse(1)(1)(1)#57
       + 1361.25 b_warehouse(1)(2)(0)#58 + 1361.25 b_warehouse(1)(2)(1)#59
       + 1361.25 b_warehouse(1)(3)(0)#60 + 1361.25 b_warehouse(1)(3)(1)#61
       + 1361.25 b_warehouse(1)(4)(0)#62 + 1361.25 b_warehouse(1)(4)(1)#63
       + 1361.25 b_warehouse(1)(5)(0)#64 + 1361.25 b_warehouse(1)(5)(1)#65
       + 2838.94205647104 o(0)(0)#102 + 2838.94205647104 o(0)(1)#103
       + 2838.94205647104 o(0)(2)#104 + 2838.94205647104 o(0)(3)#105
       + 2838.94205647104 o(0)(4)#106 + 2838.94205647104 o(0)(5)#107
       + 1739.25271309261 o(1)(0)#108 + 1739.25271309261 o(1)(1)#109
       + 1739.25271309261 o(1)(2)#110 + 1739.25271309261 o(1)(3)#111
       + 1739.25271309261 o(1)(4)#112 + 1739.25271309261 o(1)(5)#113
       + 4037.75629949332 o(2)(0)#114 + 4037.75629949332 o(2)(1)#115
       + 4037.75629949332 o(2)(2)#116 + 4037.75629949332 o(2)(3)#117
       + 4037.75629949332 o(2)(4)#118 + 4037.75629949332 o(2)(5)#119
Subject To
 ProductionCapacity(1):          - 304 y(0)#0 + p(0)#6 <= 0
 ProductionCapacity(2):          - 304 y(1)#1 + p(1)#7 <= 0
 ProductionCapacity(3):          - 304 y(2)#2 + p(2)#8 <= 0
 ProductionCapacity(4):          - 304 y(3)#3 + p(3)#9 <= 0
 ProductionCapacity(5):          - 304 y(4)#4 + p(4)#10 <= 0
 ProductionCapacity(6):          - 304 y(5)#5 + p(5)#11 <= 0
 PlantInventoryCapacity(1):      I_plant(0)#12 <= 152
 PlantInventoryCapacity(2):      I_plant(1)#13 <= 152
 PlantInventoryCapacity(3):      I_plant(2)#14 <= 152
 PlantInventoryCapacity(4):      I_plant(3)#15 <= 152
 PlantInventoryCapacity(5):      I_plant(4)#16 <= 152
 PlantInventoryCapacity(6):      I_plant(5)#17 <= 152
 PlantInventoryBalance(1):       - p(0)#6 + I_plant(0)#12 + q(0)(0)(0)#66
                                 + q(0)(1)(0)#72 + q(1)(0)(0)#78
                                 + q(1)(1)(0)#84 + q(2)(0)(0)#90
                                 + q(2)(1)(0)#96  = 0
 PlantInventoryBalance(2):       - p(1)#7 - I_plant(0)#12 + I_plant(1)#13
                                 + q(0)(0)(1)#67 + q(0)(1)(1)#73
                                 + q(1)(0)(1)#79 + q(1)(1)(1)#85
                                 + q(2)(0)(1)#91 + q(2)(1)(1)#97  = 0
 PlantInventoryBalance(3):       - p(2)#8 - I_plant(1)#13 + I_plant(2)#14
                                 + q(0)(0)(2)#68 + q(0)(1)(2)#74
                                 + q(1)(0)(2)#80 + q(1)(1)(2)#86
                                 + q(2)(0)(2)#92 + q(2)(1)(2)#98  = 0
 PlantInventoryBalance(4):       - p(3)#9 - I_plant(2)#14 + I_plant(3)#15
                                 + q(0)(0)(3)#69 + q(0)(1)(3)#75
                                 + q(1)(0)(3)#81 + q(1)(1)(3)#87
                                 + q(2)(0)(3)#93 + q(2)(1)(3)#99  = 0
 PlantInventoryBalance(5):       - p(4)#10 - I_plant(3)#15 + I_plant(4)#16
                                 + q(0)(0)(4)#70 + q(0)(1)(4)#76
                                 + q(1)(0)(4)#82 + q(1)(1)(4)#88
                                 + q(2)(0)(4)#94 + q(2)(1)(4)#100  = 0
 PlantInventoryBalance(6):       - p(5)#11 - I_plant(4)#16 + I_plant(5)#17
                                 + q(0)(0)(5)#71 + q(0)(1)(5)#77
                                 + q(1)(0)(5)#83 + q(1)(1)(5)#89
                                 + q(2)(0)(5)#95 + q(2)(1)(5)#101  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(0)(0)(0)#18 <= 136.8
 WarehouseInventoryCapacity(2,1,1): I_warehouse(1)(0)(0)#30 <= 136.8
 WarehouseInventoryCapacity(1,2,1): I_warehouse(0)(1)(0)#20 <= 136.8
 WarehouseInventoryCapacity(2,2,1): I_warehouse(1)(1)(0)#32 <= 136.8
 WarehouseInventoryCapacity(1,3,1): I_warehouse(0)(2)(0)#22 <= 136.8
 WarehouseInventoryCapacity(2,3,1): I_warehouse(1)(2)(0)#34 <= 136.8
 WarehouseInventoryCapacity(1,4,1): I_warehouse(0)(3)(0)#24 <= 136.8
 WarehouseInventoryCapacity(2,4,1): I_warehouse(1)(3)(0)#36 <= 136.8
 WarehouseInventoryCapacity(1,5,1): I_warehouse(0)(4)(0)#26 <= 136.8
 WarehouseInventoryCapacity(2,5,1): I_warehouse(1)(4)(0)#38 <= 136.8
 WarehouseInventoryCapacity(1,6,1): I_warehouse(0)(5)(0)#28 <= 136.8
 WarehouseInventoryCapacity(2,6,1): I_warehouse(1)(5)(0)#40 <= 136.8
 WarehouseInventoryCapacity(1,1,2): I_warehouse(0)(0)(1)#19 <= 136.8
 WarehouseInventoryCapacity(2,1,2): I_warehouse(1)(0)(1)#31 <= 136.8
 WarehouseInventoryCapacity(1,2,2): I_warehouse(0)(1)(1)#21 <= 136.8
 WarehouseInventoryCapacity(2,2,2): I_warehouse(1)(1)(1)#33 <= 136.8
 WarehouseInventoryCapacity(1,3,2): I_warehouse(0)(2)(1)#23 <= 136.8
 WarehouseInventoryCapacity(2,3,2): I_warehouse(1)(2)(1)#35 <= 136.8
 WarehouseInventoryCapacity(1,4,2): I_warehouse(0)(3)(1)#25 <= 136.8
 WarehouseInventoryCapacity(2,4,2): I_warehouse(1)(3)(1)#37 <= 136.8
 WarehouseInventoryCapacity(1,5,2): I_warehouse(0)(4)(1)#27 <= 136.8
 WarehouseInventoryCapacity(2,5,2): I_warehouse(1)(4)(1)#39 <= 136.8
 WarehouseInventoryCapacity(1,6,2): I_warehouse(0)(5)(1)#29 <= 136.8
 WarehouseInventoryCapacity(2,6,2): I_warehouse(1)(5)(1)#41 <= 136.8
 WarehouseInventoryBalance(1,1,1): I_warehouse(0)(0)(0)#18
                                 - b_warehouse(0)(0)(0)#42 - q(0)(0)(0)#66
                                 - q(1)(0)(0)#78 - q(2)(0)(0)#90  = -9
 WarehouseInventoryBalance(2,1,1): I_warehouse(1)(0)(0)#30
                                 - b_warehouse(1)(0)(0)#54 - q(0)(1)(0)#72
                                 - q(1)(1)(0)#84 - q(2)(1)(0)#96  = -10
 WarehouseInventoryBalance(1,2,1): - I_warehouse(0)(0)(0)#18
                                 + I_warehouse(0)(1)(0)#20
                                 - b_warehouse(0)(1)(0)#44 - q(0)(0)(1)#67
                                 - q(1)(0)(1)#79 - q(2)(0)(1)#91  = -31
 WarehouseInventoryBalance(2,2,1): - I_warehouse(1)(0)(0)#30
                                 + I_warehouse(1)(1)(0)#32
                                 - b_warehouse(1)(1)(0)#56 - q(0)(1)(1)#73
                                 - q(1)(1)(1)#85 - q(2)(1)(1)#97  = -36
 WarehouseInventoryBalance(1,3,1): - I_warehouse(0)(1)(0)#20
                                 + I_warehouse(0)(2)(0)#22
                                 - b_warehouse(0)(2)(0)#46 - q(0)(0)(2)#68
                                 - q(1)(0)(2)#80 - q(2)(0)(2)#92  = -74
 WarehouseInventoryBalance(2,3,1): - I_warehouse(1)(1)(0)#32
                                 + I_warehouse(1)(2)(0)#34
                                 - b_warehouse(1)(2)(0)#58 - q(0)(1)(2)#74
                                 - q(1)(1)(2)#86 - q(2)(1)(2)#98  = -49
 WarehouseInventoryBalance(1,4,1): - I_warehouse(0)(2)(0)#22
                                 + I_warehouse(0)(3)(0)#24
                                 - b_warehouse(0)(3)(0)#48 - q(0)(0)(3)#69
                                 - q(1)(0)(3)#81 - q(2)(0)(3)#93  = -89
 WarehouseInventoryBalance(2,4,1): - I_warehouse(1)(2)(0)#34
                                 + I_warehouse(1)(3)(0)#36
                                 - b_warehouse(1)(3)(0)#60 - q(0)(1)(3)#75
                                 - q(1)(1)(3)#87 - q(2)(1)(3)#99  = -51
 WarehouseInventoryBalance(1,5,1): - I_warehouse(0)(3)(0)#24
                                 + I_warehouse(0)(4)(0)#26
                                 - b_warehouse(0)(4)(0)#50 - q(0)(0)(4)#70
                                 - q(1)(0)(4)#82 - q(2)(0)(4)#94  = -99
 WarehouseInventoryBalance(2,5,1): - I_warehouse(1)(3)(0)#36
                                 + I_warehouse(1)(4)(0)#38
                                 - b_warehouse(1)(4)(0)#62 - q(0)(1)(4)#76
                                 - q(1)(1)(4)#88 - q(2)(1)(4)#100  = -55
 WarehouseInventoryBalance(1,6,1): - I_warehouse(0)(4)(0)#26
                                 + I_warehouse(0)(5)(0)#28
                                 - b_warehouse(0)(5)(0)#52 - q(0)(0)(5)#71
                                 - q(1)(0)(5)#83 - q(2)(0)(5)#95  = -104
 WarehouseInventoryBalance(2,6,1): - I_warehouse(1)(4)(0)#38
                                 + I_warehouse(1)(5)(0)#40
                                 - b_warehouse(1)(5)(0)#64 - q(0)(1)(5)#77
                                 - q(1)(1)(5)#89 - q(2)(1)(5)#101  = -51
 WarehouseInventoryBalance(1,1,2): I_warehouse(0)(0)(1)#19
                                 - b_warehouse(0)(0)(1)#43 - q(0)(0)(0)#66
                                 - q(1)(0)(0)#78 - q(2)(0)(0)#90  = -94
 WarehouseInventoryBalance(2,1,2): I_warehouse(1)(0)(1)#31
                                 - b_warehouse(1)(0)(1)#55 - q(0)(1)(0)#72
                                 - q(1)(1)(0)#84 - q(2)(1)(0)#96  = -60
 WarehouseInventoryBalance(1,2,2): - I_warehouse(0)(0)(1)#19
                                 + I_warehouse(0)(1)(1)#21
                                 - b_warehouse(0)(1)(1)#45 - q(0)(0)(1)#67
                                 - q(1)(0)(1)#79 - q(2)(0)(1)#91  = -100
 WarehouseInventoryBalance(2,2,2): - I_warehouse(1)(0)(1)#31
                                 + I_warehouse(1)(1)(1)#33
                                 - b_warehouse(1)(1)(1)#57 - q(0)(1)(1)#73
                                 - q(1)(1)(1)#85 - q(2)(1)(1)#97  = -58
 WarehouseInventoryBalance(1,3,2): - I_warehouse(0)(1)(1)#21
                                 + I_warehouse(0)(2)(1)#23
                                 - b_warehouse(0)(2)(1)#47 - q(0)(0)(2)#68
                                 - q(1)(0)(2)#80 - q(2)(0)(2)#92  = -102
 WarehouseInventoryBalance(2,3,2): - I_warehouse(1)(1)(1)#33
                                 + I_warehouse(1)(2)(1)#35
                                 - b_warehouse(1)(2)(1)#59 - q(0)(1)(2)#74
                                 - q(1)(1)(2)#86 - q(2)(1)(2)#98  = -58
 WarehouseInventoryBalance(1,4,2): - I_warehouse(0)(2)(1)#23
                                 + I_warehouse(0)(3)(1)#25
                                 - b_warehouse(0)(3)(1)#49 - q(0)(0)(3)#69
                                 - q(1)(0)(3)#81 - q(2)(0)(3)#93  = -103
 WarehouseInventoryBalance(2,4,2): - I_warehouse(1)(2)(1)#35
                                 + I_warehouse(1)(3)(1)#37
                                 - b_warehouse(1)(3)(1)#61 - q(0)(1)(3)#75
                                 - q(1)(1)(3)#87 - q(2)(1)(3)#99  = -57
 WarehouseInventoryBalance(1,5,2): - I_warehouse(0)(3)(1)#25
                                 + I_warehouse(0)(4)(1)#27
                                 - b_warehouse(0)(4)(1)#51 - q(0)(0)(4)#70
                                 - q(1)(0)(4)#82 - q(2)(0)(4)#94  = -105
 WarehouseInventoryBalance(2,5,2): - I_warehouse(1)(3)(1)#37
                                 + I_warehouse(1)(4)(1)#39
                                 - b_warehouse(1)(4)(1)#63 - q(0)(1)(4)#76
                                 - q(1)(1)(4)#88 - q(2)(1)(4)#100  = -52
 WarehouseInventoryBalance(1,6,2): - I_warehouse(0)(4)(1)#27
                                 + I_warehouse(0)(5)(1)#29
                                 - b_warehouse(0)(5)(1)#53 - q(0)(0)(5)#71
                                 - q(1)(0)(5)#83 - q(2)(0)(5)#95  = -95
 WarehouseInventoryBalance(2,6,2): - I_warehouse(1)(4)(1)#39
                                 + I_warehouse(1)(5)(1)#41
                                 - b_warehouse(1)(5)(1)#65 - q(0)(1)(5)#77
                                 - q(1)(1)(5)#89 - q(2)(1)(5)#101  = -50
 WarehouseCoverage(1,1):         o(0)(0)#102 + o(2)(0)#114 <= 1
 WarehouseCoverage(2,1):         o(1)(0)#108 + o(2)(0)#114 <= 1
 WarehouseCoverage(1,2):         o(0)(1)#103 + o(2)(1)#115 <= 1
 WarehouseCoverage(2,2):         o(1)(1)#109 + o(2)(1)#115 <= 1
 WarehouseCoverage(1,3):         o(0)(2)#104 + o(2)(2)#116 <= 1
 WarehouseCoverage(2,3):         o(1)(2)#110 + o(2)(2)#116 <= 1
 WarehouseCoverage(1,4):         o(0)(3)#105 + o(2)(3)#117 <= 1
 WarehouseCoverage(2,4):         o(1)(3)#111 + o(2)(3)#117 <= 1
 WarehouseCoverage(1,5):         o(0)(4)#106 + o(2)(4)#118 <= 1
 WarehouseCoverage(2,5):         o(1)(4)#112 + o(2)(4)#118 <= 1
 WarehouseCoverage(1,6):         o(0)(5)#107 + o(2)(5)#119 <= 1
 WarehouseCoverage(2,6):         o(1)(5)#113 + o(2)(5)#119 <= 1
 VehicleCapacity(1,1):           q(0)(0)(0)#66 + q(0)(1)(0)#72
                                 - 102 o(0)(0)#102 <= 0
 VehicleCapacity(2,1):           q(1)(0)(0)#78 + q(1)(1)(0)#84
                                 - 102 o(1)(0)#108 <= 0
 VehicleCapacity(3,1):           q(2)(0)(0)#90 + q(2)(1)(0)#96
                                 - 102 o(2)(0)#114 <= 0
 VehicleCapacity(1,2):           q(0)(0)(1)#67 + q(0)(1)(1)#73
                                 - 102 o(0)(1)#103 <= 0
 VehicleCapacity(2,2):           q(1)(0)(1)#79 + q(1)(1)(1)#85
                                 - 102 o(1)(1)#109 <= 0
 VehicleCapacity(3,2):           q(2)(0)(1)#91 + q(2)(1)(1)#97
                                 - 102 o(2)(1)#115 <= 0
 VehicleCapacity(1,3):           q(0)(0)(2)#68 + q(0)(1)(2)#74
                                 - 102 o(0)(2)#104 <= 0
 VehicleCapacity(2,3):           q(1)(0)(2)#80 + q(1)(1)(2)#86
                                 - 102 o(1)(2)#110 <= 0
 VehicleCapacity(3,3):           q(2)(0)(2)#92 + q(2)(1)(2)#98
                                 - 102 o(2)(2)#116 <= 0
 VehicleCapacity(1,4):           q(0)(0)(3)#69 + q(0)(1)(3)#75
                                 - 102 o(0)(3)#105 <= 0
 VehicleCapacity(2,4):           q(1)(0)(3)#81 + q(1)(1)(3)#87
                                 - 102 o(1)(3)#111 <= 0
 VehicleCapacity(3,4):           q(2)(0)(3)#93 + q(2)(1)(3)#99
                                 - 102 o(2)(3)#117 <= 0
 VehicleCapacity(1,5):           q(0)(0)(4)#70 + q(0)(1)(4)#76
                                 - 102 o(0)(4)#106 <= 0
 VehicleCapacity(2,5):           q(1)(0)(4)#82 + q(1)(1)(4)#88
                                 - 102 o(1)(4)#112 <= 0
 VehicleCapacity(3,5):           q(2)(0)(4)#94 + q(2)(1)(4)#100
                                 - 102 o(2)(4)#118 <= 0
 VehicleCapacity(1,6):           q(0)(0)(5)#71 + q(0)(1)(5)#77
                                 - 102 o(0)(5)#107 <= 0
 VehicleCapacity(2,6):           q(1)(0)(5)#83 + q(1)(1)(5)#89
                                 - 102 o(1)(5)#113 <= 0
 VehicleCapacity(3,6):           q(2)(0)(5)#95 + q(2)(1)(5)#101
                                 - 102 o(2)(5)#119 <= 0
 FleetSize(1):                   o(0)(0)#102 + o(1)(0)#108 + o(2)(0)#114 <= 2
 FleetSize(2):                   o(0)(1)#103 + o(1)(1)#109 + o(2)(1)#115 <= 2
 FleetSize(3):                   o(0)(2)#104 + o(1)(2)#110 + o(2)(2)#116 <= 2
 FleetSize(4):                   o(0)(3)#105 + o(1)(3)#111 + o(2)(3)#117 <= 2
 FleetSize(5):                   o(0)(4)#106 + o(1)(4)#112 + o(2)(4)#118 <= 2
 FleetSize(6):                   o(0)(5)#107 + o(1)(5)#113 + o(2)(5)#119 <= 2
Bounds
 0 <= y(0)#0 <= 1
 0 <= y(1)#1 <= 1
 0 <= y(2)#2 <= 1
 0 <= y(3)#3 <= 1
 0 <= y(4)#4 <= 1
 0 <= y(5)#5 <= 1
 0 <= p(0)#6 <= 304
 0 <= p(1)#7 <= 304
 0 <= p(2)#8 <= 304
 0 <= p(3)#9 <= 304
 0 <= p(4)#10 <= 304
 0 <= p(5)#11 <= 304
 0 <= I_plant(0)#12 <= 152
 0 <= I_plant(1)#13 <= 152
 0 <= I_plant(2)#14 <= 152
 0 <= I_plant(3)#15 <= 152
 0 <= I_plant(4)#16 <= 152
 0 <= I_plant(5)#17 <= 152
 0 <= I_warehouse(0)(0)(0)#18 <= 136.8
 0 <= I_warehouse(0)(0)(1)#19 <= 136.8
 0 <= I_warehouse(0)(1)(0)#20 <= 136.8
 0 <= I_warehouse(0)(1)(1)#21 <= 136.8
 0 <= I_warehouse(0)(2)(0)#22 <= 136.8
 0 <= I_warehouse(0)(2)(1)#23 <= 136.8
 0 <= I_warehouse(0)(3)(0)#24 <= 136.8
 0 <= I_warehouse(0)(3)(1)#25 <= 136.8
 0 <= I_warehouse(0)(4)(0)#26 <= 136.8
 0 <= I_warehouse(0)(4)(1)#27 <= 136.8
 0 <= I_warehouse(0)(5)(0)#28 <= 136.8
 0 <= I_warehouse(0)(5)(1)#29 <= 136.8
 0 <= I_warehouse(1)(0)(0)#30 <= 136.8
 0 <= I_warehouse(1)(0)(1)#31 <= 136.8
 0 <= I_warehouse(1)(1)(0)#32 <= 136.8
 0 <= I_warehouse(1)(1)(1)#33 <= 136.8
 0 <= I_warehouse(1)(2)(0)#34 <= 136.8
 0 <= I_warehouse(1)(2)(1)#35 <= 136.8
 0 <= I_warehouse(1)(3)(0)#36 <= 136.8
 0 <= I_warehouse(1)(3)(1)#37 <= 136.8
 0 <= I_warehouse(1)(4)(0)#38 <= 136.8
 0 <= I_warehouse(1)(4)(1)#39 <= 136.8
 0 <= I_warehouse(1)(5)(0)#40 <= 136.8
 0 <= I_warehouse(1)(5)(1)#41 <= 136.8
 0 <= b_warehouse(0)(0)(0)#42 <= 9
 0 <= b_warehouse(0)(0)(1)#43 <= 94
 0 <= b_warehouse(0)(1)(0)#44 <= 31
 0 <= b_warehouse(0)(1)(1)#45 <= 100
 0 <= b_warehouse(0)(2)(0)#46 <= 74
 0 <= b_warehouse(0)(2)(1)#47 <= 102
 0 <= b_warehouse(0)(3)(0)#48 <= 89
 0 <= b_warehouse(0)(3)(1)#49 <= 103
 0 <= b_warehouse(0)(4)(0)#50 <= 99
 0 <= b_warehouse(0)(4)(1)#51 <= 105
 0 <= b_warehouse(0)(5)(0)#52 <= 104
 0 <= b_warehouse(0)(5)(1)#53 <= 95
 0 <= b_warehouse(1)(0)(0)#54 <= 10
 0 <= b_warehouse(1)(0)(1)#55 <= 60
 0 <= b_warehouse(1)(1)(0)#56 <= 36
 0 <= b_warehouse(1)(1)(1)#57 <= 58
 0 <= b_warehouse(1)(2)(0)#58 <= 49
 0 <= b_warehouse(1)(2)(1)#59 <= 58
 0 <= b_warehouse(1)(3)(0)#60 <= 51
 0 <= b_warehouse(1)(3)(1)#61 <= 57
 0 <= b_warehouse(1)(4)(0)#62 <= 55
 0 <= b_warehouse(1)(4)(1)#63 <= 52
 0 <= b_warehouse(1)(5)(0)#64 <= 51
 0 <= b_warehouse(1)(5)(1)#65 <= 50
 0 <= q(0)(0)(0)#66 <= 136.8
 0 <= q(0)(0)(1)#67 <= 136.8
 0 <= q(0)(0)(2)#68 <= 136.8
 0 <= q(0)(0)(3)#69 <= 136.8
 0 <= q(0)(0)(4)#70 <= 136.8
 0 <= q(0)(0)(5)#71 <= 136.8
      q(0)(1)(0)#72 = 0
      q(0)(1)(1)#73 = 0
      q(0)(1)(2)#74 = 0
      q(0)(1)(3)#75 = 0
      q(0)(1)(4)#76 = 0
      q(0)(1)(5)#77 = 0
      q(1)(0)(0)#78 = 0
      q(1)(0)(1)#79 = 0
      q(1)(0)(2)#80 = 0
      q(1)(0)(3)#81 = 0
      q(1)(0)(4)#82 = 0
      q(1)(0)(5)#83 = 0
 0 <= q(1)(1)(0)#84 <= 136.8
 0 <= q(1)(1)(1)#85 <= 136.8
 0 <= q(1)(1)(2)#86 <= 136.8
 0 <= q(1)(1)(3)#87 <= 136.8
 0 <= q(1)(1)(4)#88 <= 136.8
 0 <= q(1)(1)(5)#89 <= 136.8
 0 <= q(2)(0)(0)#90 <= 136.8
 0 <= q(2)(0)(1)#91 <= 136.8
 0 <= q(2)(0)(2)#92 <= 136.8
 0 <= q(2)(0)(3)#93 <= 136.8
 0 <= q(2)(0)(4)#94 <= 136.8
 0 <= q(2)(0)(5)#95 <= 136.8
 0 <= q(2)(1)(0)#96 <= 136.8
 0 <= q(2)(1)(1)#97 <= 136.8
 0 <= q(2)(1)(2)#98 <= 136.8
 0 <= q(2)(1)(3)#99 <= 136.8
 0 <= q(2)(1)(4)#100 <= 136.8
 0 <= q(2)(1)(5)#101 <= 136.8
 0 <= o(0)(0)#102 <= 1
 0 <= o(0)(1)#103 <= 1
 0 <= o(0)(2)#104 <= 1
 0 <= o(0)(3)#105 <= 1
 0 <= o(0)(4)#106 <= 1
 0 <= o(0)(5)#107 <= 1
 0 <= o(1)(0)#108 <= 1
 0 <= o(1)(1)#109 <= 1
 0 <= o(1)(2)#110 <= 1
 0 <= o(1)(3)#111 <= 1
 0 <= o(1)(4)#112 <= 1
 0 <= o(1)(5)#113 <= 1
 0 <= o(2)(0)#114 <= 1
 0 <= o(2)(1)#115 <= 1
 0 <= o(2)(2)#116 <= 1
 0 <= o(2)(3)#117 <= 1
 0 <= o(2)(4)#118 <= 1
 0 <= o(2)(5)#119 <= 1
Binaries
 y(0)#0  y(1)#1  y(2)#2  y(3)#3  y(4)#4  y(5)#5  o(0)(0)#102  o(0)(1)#103 
 o(0)(2)#104  o(0)(3)#105  o(0)(4)#106  o(0)(5)#107  o(1)(0)#108  o(1)(1)#109 
 o(1)(2)#110  o(1)(3)#111  o(1)(4)#112  o(1)(5)#113  o(2)(0)#114  o(2)(1)#115 
 o(2)(2)#116  o(2)(3)#117  o(2)(4)#118  o(2)(5)#119 
End
