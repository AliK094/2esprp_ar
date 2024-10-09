\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 30 p(1)#0 + 30 p(2)#1 + 30 p(3)#2 + 3 I_plant(1)#3 + 3 I_plant(2)#4
       + 3 I_plant(3)#5 + 5 I_warehouse(1)(1)(2)#6 + 5 I_warehouse(1)(2)(2)#7
       + 5 I_warehouse(1)(3)(2)#8 + 5 I_warehouse(2)(1)(2)#9
       + 5 I_warehouse(2)(2)(2)#10 + 5 I_warehouse(2)(3)(2)#11
       + 9 I_customer(3)(1)(2)#12 + 9 I_customer(3)(2)(2)#13
       + 9 I_customer(3)(3)(2)#14 + 10 I_customer(4)(1)(2)#15
       + 10 I_customer(4)(2)(2)#16 + 10 I_customer(4)(3)(2)#17
       + 10 I_customer(5)(1)(2)#18 + 10 I_customer(5)(2)(2)#19
       + 10 I_customer(5)(3)(2)#20 + 8 I_customer(6)(1)(2)#21
       + 8 I_customer(6)(2)(2)#22 + 8 I_customer(6)(3)(2)#23
       + 8 I_customer(7)(1)(2)#24 + 8 I_customer(7)(2)(2)#25
       + 8 I_customer(7)(3)(2)#26 + 10 I_customer(8)(1)(2)#27
       + 10 I_customer(8)(2)(2)#28 + 10 I_customer(8)(3)(2)#29
       + 10 I_customer(9)(1)(2)#30 + 10 I_customer(9)(2)(2)#31
       + 10 I_customer(9)(3)(2)#32 + 7 I_customer(10)(1)(2)#33
       + 7 I_customer(10)(2)(2)#34 + 7 I_customer(10)(3)(2)#35
       + 9 I_customer(11)(1)(2)#36 + 9 I_customer(11)(2)(2)#37
       + 9 I_customer(11)(3)(2)#38 + 7 I_customer(12)(1)(2)#39
       + 7 I_customer(12)(2)(2)#40 + 7 I_customer(12)(3)(2)#41
       + 4700 b_customer(3)(1)(2)#42 + 4700 b_customer(3)(2)(2)#43
       + 4700 b_customer(3)(3)(2)#44 + 4600 b_customer(4)(1)(2)#45
       + 4600 b_customer(4)(2)(2)#46 + 4600 b_customer(4)(3)(2)#47
       + 4800 b_customer(5)(1)(2)#48 + 4800 b_customer(5)(2)(2)#49
       + 4800 b_customer(5)(3)(2)#50 + 4700 b_customer(6)(1)(2)#51
       + 4700 b_customer(6)(2)(2)#52 + 4700 b_customer(6)(3)(2)#53
       + 4800 b_customer(7)(1)(2)#54 + 4800 b_customer(7)(2)(2)#55
       + 4800 b_customer(7)(3)(2)#56 + 4700 b_customer(8)(1)(2)#57
       + 4700 b_customer(8)(2)(2)#58 + 4700 b_customer(8)(3)(2)#59
       + 4700 b_customer(9)(1)(2)#60 + 4700 b_customer(9)(2)(2)#61
       + 4700 b_customer(9)(3)(2)#62 + 4600 b_customer(10)(1)(2)#63
       + 4600 b_customer(10)(2)(2)#64 + 4600 b_customer(10)(3)(2)#65
       + 4600 b_customer(11)(1)(2)#66 + 4600 b_customer(11)(2)(2)#67
       + 4600 b_customer(11)(3)(2)#68 + 4600 b_customer(12)(1)(2)#69
       + 4600 b_customer(12)(2)(2)#70 + 4600 b_customer(12)(3)(2)#71
Subject To
 c1:                             p(1)#0  = 190
 c2:                             p(2)#1  = 124
 c3:                             p(3)#2  = 0
 c4:                             I_plant(1)#3  = 0
 c5:                             I_plant(2)#4  = 0
 c6:                             I_plant(3)#5  = 0
 ProductionCapacity(1):          p(1)#0 <= 270
 ProductionCapacity(2):          p(2)#1 <= 270
 ProductionCapacity(3):          p(3)#2 <= 0
 PlantInventoryCapacity(1):      I_plant(1)#3 <= 135
 PlantInventoryCapacity(2):      I_plant(2)#4 <= 135
 PlantInventoryCapacity(3):      I_plant(3)#5 <= 135
 PlantInventoryBalance(1):       - p(1)#0 + I_plant(1)#3 + q(1)(1)(1)#72
                                 + q(2)(1)(1)#75  = 0
 PlantInventoryBalance(2):       - p(2)#1 - I_plant(1)#3 + I_plant(2)#4
                                 + q(1)(1)(2)#73 + q(2)(1)(2)#76  = 0
 PlantInventoryBalance(3):       - p(3)#2 - I_plant(2)#4 + I_plant(3)#5
                                 + q(1)(1)(3)#74 + q(2)(1)(3)#77  = 0
 WarehouseInventoryCapacity(1,1,2): I_warehouse(1)(1)(2)#6 <= 124
 WarehouseInventoryCapacity(2,1,2): I_warehouse(2)(1)(2)#9 <= 124
 WarehouseInventoryCapacity(1,2,2): I_warehouse(1)(2)(2)#7 <= 124
 WarehouseInventoryCapacity(2,2,2): I_warehouse(2)(2)(2)#10 <= 124
 WarehouseInventoryCapacity(1,3,2): I_warehouse(1)(3)(2)#8 <= 124
 WarehouseInventoryCapacity(2,3,2): I_warehouse(2)(3)(2)#11 <= 124
 WarehouseInventoryBalance(1,1,2): I_warehouse(1)(1)(2)#6 - q(1)(1)(1)#72
                                 + w(3)(1)(1)(2)#78 + w(4)(1)(1)(2)#84
                                 + w(5)(1)(1)(2)#90 + w(6)(1)(1)(2)#96
                                 + w(7)(1)(1)(2)#102 + w(8)(1)(1)(2)#108
                                 + w(9)(1)(1)(2)#114 + w(10)(1)(1)(2)#120
                                 + w(11)(1)(1)(2)#126 + w(12)(1)(1)(2)#132  = 0
 WarehouseInventoryBalance(2,1,2): I_warehouse(2)(1)(2)#9 - q(2)(1)(1)#75
                                 + w(3)(2)(1)(2)#81 + w(4)(2)(1)(2)#87
                                 + w(5)(2)(1)(2)#93 + w(6)(2)(1)(2)#99
                                 + w(7)(2)(1)(2)#105 + w(8)(2)(1)(2)#111
                                 + w(9)(2)(1)(2)#117 + w(10)(2)(1)(2)#123
                                 + w(11)(2)(1)(2)#129 + w(12)(2)(1)(2)#135  = 0
 WarehouseInventoryBalance(1,2,2): - I_warehouse(1)(1)(2)#6
                                 + I_warehouse(1)(2)(2)#7 - q(1)(1)(2)#73
                                 + w(3)(1)(2)(2)#79 + w(4)(1)(2)(2)#85
                                 + w(5)(1)(2)(2)#91 + w(6)(1)(2)(2)#97
                                 + w(7)(1)(2)(2)#103 + w(8)(1)(2)(2)#109
                                 + w(9)(1)(2)(2)#115 + w(10)(1)(2)(2)#121
                                 + w(11)(1)(2)(2)#127 + w(12)(1)(2)(2)#133  = 0
 WarehouseInventoryBalance(2,2,2): - I_warehouse(2)(1)(2)#9
                                 + I_warehouse(2)(2)(2)#10 - q(2)(1)(2)#76
                                 + w(3)(2)(2)(2)#82 + w(4)(2)(2)(2)#88
                                 + w(5)(2)(2)(2)#94 + w(6)(2)(2)(2)#100
                                 + w(7)(2)(2)(2)#106 + w(8)(2)(2)(2)#112
                                 + w(9)(2)(2)(2)#118 + w(10)(2)(2)(2)#124
                                 + w(11)(2)(2)(2)#130 + w(12)(2)(2)(2)#136  = 0
 WarehouseInventoryBalance(1,3,2): - I_warehouse(1)(2)(2)#7
                                 + I_warehouse(1)(3)(2)#8 - q(1)(1)(3)#74
                                 + w(3)(1)(3)(2)#80 + w(4)(1)(3)(2)#86
                                 + w(5)(1)(3)(2)#92 + w(6)(1)(3)(2)#98
                                 + w(7)(1)(3)(2)#104 + w(8)(1)(3)(2)#110
                                 + w(9)(1)(3)(2)#116 + w(10)(1)(3)(2)#122
                                 + w(11)(1)(3)(2)#128 + w(12)(1)(3)(2)#134  = 0
 WarehouseInventoryBalance(2,3,2): - I_warehouse(2)(2)(2)#10
                                 + I_warehouse(2)(3)(2)#11 - q(2)(1)(3)#77
                                 + w(3)(2)(3)(2)#83 + w(4)(2)(3)(2)#89
                                 + w(5)(2)(3)(2)#95 + w(6)(2)(3)(2)#101
                                 + w(7)(2)(3)(2)#107 + w(8)(2)(3)(2)#113
                                 + w(9)(2)(3)(2)#119 + w(10)(2)(3)(2)#125
                                 + w(11)(2)(3)(2)#131 + w(12)(2)(3)(2)#137  = 0
 CustomerInventoryBalance(3,1,2): I_customer(3)(1)(2)#12
                                 - b_customer(3)(1)(2)#42 - w(3)(1)(1)(2)#78
                                 - w(3)(2)(1)(2)#81  = 16
 CustomerInventoryBalance(4,1,2): I_customer(4)(1)(2)#15
                                 - b_customer(4)(1)(2)#45 - w(4)(1)(1)(2)#84
                                 - w(4)(2)(1)(2)#87  = 1
 CustomerInventoryBalance(5,1,2): I_customer(5)(1)(2)#18
                                 - b_customer(5)(1)(2)#48 - w(5)(1)(1)(2)#90
                                 - w(5)(2)(1)(2)#93  = 1
 CustomerInventoryBalance(6,1,2): I_customer(6)(1)(2)#21
                                 - b_customer(6)(1)(2)#51 - w(6)(1)(1)(2)#96
                                 - w(6)(2)(1)(2)#99  = -1
 CustomerInventoryBalance(7,1,2): I_customer(7)(1)(2)#24
                                 - b_customer(7)(1)(2)#54 - w(7)(1)(1)(2)#102
                                 - w(7)(2)(1)(2)#105  = 8
 CustomerInventoryBalance(8,1,2): I_customer(8)(1)(2)#27
                                 - b_customer(8)(1)(2)#57 - w(8)(1)(1)(2)#108
                                 - w(8)(2)(1)(2)#111  = 0
 CustomerInventoryBalance(9,1,2): I_customer(9)(1)(2)#30
                                 - b_customer(9)(1)(2)#60 - w(9)(1)(1)(2)#114
                                 - w(9)(2)(1)(2)#117  = 19
 CustomerInventoryBalance(10,1,2): I_customer(10)(1)(2)#33
                                 - b_customer(10)(1)(2)#63 - w(10)(1)(1)(2)#120
                                 - w(10)(2)(1)(2)#123  = 2
 CustomerInventoryBalance(11,1,2): I_customer(11)(1)(2)#36
                                 - b_customer(11)(1)(2)#66 - w(11)(1)(1)(2)#126
                                 - w(11)(2)(1)(2)#129  = -3
 CustomerInventoryBalance(12,1,2): I_customer(12)(1)(2)#39
                                 - b_customer(12)(1)(2)#69 - w(12)(1)(1)(2)#132
                                 - w(12)(2)(1)(2)#135  = 17
 CustomerInventoryBalance(3,2,2): - I_customer(3)(1)(2)#12
                                 + I_customer(3)(2)(2)#13
                                 - b_customer(3)(2)(2)#43 - w(3)(1)(2)(2)#79
                                 - w(3)(2)(2)(2)#82  = -20
 CustomerInventoryBalance(4,2,2): - I_customer(4)(1)(2)#15
                                 + I_customer(4)(2)(2)#16
                                 - b_customer(4)(2)(2)#46 - w(4)(1)(2)(2)#85
                                 - w(4)(2)(2)(2)#88  = -8
 CustomerInventoryBalance(5,2,2): - I_customer(5)(1)(2)#18
                                 + I_customer(5)(2)(2)#19
                                 - b_customer(5)(2)(2)#49 - w(5)(1)(2)(2)#91
                                 - w(5)(2)(2)(2)#94  = -7
 CustomerInventoryBalance(6,2,2): - I_customer(6)(1)(2)#21
                                 + I_customer(6)(2)(2)#22
                                 - b_customer(6)(2)(2)#52 - w(6)(1)(2)(2)#97
                                 - w(6)(2)(2)(2)#100  = -16
 CustomerInventoryBalance(7,2,2): - I_customer(7)(1)(2)#24
                                 + I_customer(7)(2)(2)#25
                                 - b_customer(7)(2)(2)#55 - w(7)(1)(2)(2)#103
                                 - w(7)(2)(2)(2)#106  = -7
 CustomerInventoryBalance(8,2,2): - I_customer(8)(1)(2)#27
                                 + I_customer(8)(2)(2)#28
                                 - b_customer(8)(2)(2)#58 - w(8)(1)(2)(2)#109
                                 - w(8)(2)(2)(2)#112  = -17
 CustomerInventoryBalance(9,2,2): - I_customer(9)(1)(2)#30
                                 + I_customer(9)(2)(2)#31
                                 - b_customer(9)(2)(2)#61 - w(9)(1)(2)(2)#115
                                 - w(9)(2)(2)(2)#118  = -14
 CustomerInventoryBalance(10,2,2): - I_customer(10)(1)(2)#33
                                 + I_customer(10)(2)(2)#34
                                 - b_customer(10)(2)(2)#64 - w(10)(1)(2)(2)#121
                                 - w(10)(2)(2)(2)#124  = -18
 CustomerInventoryBalance(11,2,2): - I_customer(11)(1)(2)#36
                                 + I_customer(11)(2)(2)#37
                                 - b_customer(11)(2)(2)#67 - w(11)(1)(2)(2)#127
                                 - w(11)(2)(2)(2)#130  = -16
 CustomerInventoryBalance(12,2,2): - I_customer(12)(1)(2)#39
                                 + I_customer(12)(2)(2)#40
                                 - b_customer(12)(2)(2)#70 - w(12)(1)(2)(2)#133
                                 - w(12)(2)(2)(2)#136  = -14
 CustomerInventoryBalance(3,3,2): - I_customer(3)(2)(2)#13
                                 + I_customer(3)(3)(2)#14
                                 - b_customer(3)(3)(2)#44 - w(3)(1)(3)(2)#80
                                 - w(3)(2)(3)(2)#83  = -16
 CustomerInventoryBalance(4,3,2): - I_customer(4)(2)(2)#16
                                 + I_customer(4)(3)(2)#17
                                 - b_customer(4)(3)(2)#47 - w(4)(1)(3)(2)#86
                                 - w(4)(2)(3)(2)#89  = -7
 CustomerInventoryBalance(5,3,2): - I_customer(5)(2)(2)#19
                                 + I_customer(5)(3)(2)#20
                                 - b_customer(5)(3)(2)#50 - w(5)(1)(3)(2)#92
                                 - w(5)(2)(3)(2)#95  = -7
 CustomerInventoryBalance(6,3,2): - I_customer(6)(2)(2)#22
                                 + I_customer(6)(3)(2)#23
                                 - b_customer(6)(3)(2)#53 - w(6)(1)(3)(2)#98
                                 - w(6)(2)(3)(2)#101  = -17
 CustomerInventoryBalance(7,3,2): - I_customer(7)(2)(2)#25
                                 + I_customer(7)(3)(2)#26
                                 - b_customer(7)(3)(2)#56 - w(7)(1)(3)(2)#104
                                 - w(7)(2)(3)(2)#107  = -6
 CustomerInventoryBalance(8,3,2): - I_customer(8)(2)(2)#28
                                 + I_customer(8)(3)(2)#29
                                 - b_customer(8)(3)(2)#59 - w(8)(1)(3)(2)#110
                                 - w(8)(2)(3)(2)#113  = -19
 CustomerInventoryBalance(9,3,2): - I_customer(9)(2)(2)#31
                                 + I_customer(9)(3)(2)#32
                                 - b_customer(9)(3)(2)#62 - w(9)(1)(3)(2)#116
                                 - w(9)(2)(3)(2)#119  = -13
 CustomerInventoryBalance(10,3,2): - I_customer(10)(2)(2)#34
                                 + I_customer(10)(3)(2)#35
                                 - b_customer(10)(3)(2)#65 - w(10)(1)(3)(2)#122
                                 - w(10)(2)(3)(2)#125  = -17
 CustomerInventoryBalance(11,3,2): - I_customer(11)(2)(2)#37
                                 + I_customer(11)(3)(2)#38
                                 - b_customer(11)(3)(2)#68 - w(11)(1)(3)(2)#128
                                 - w(11)(2)(3)(2)#131  = -18
 CustomerInventoryBalance(12,3,2): - I_customer(12)(2)(2)#40
                                 + I_customer(12)(3)(2)#41
                                 - b_customer(12)(3)(2)#71 - w(12)(1)(3)(2)#134
                                 - w(12)(2)(3)(2)#137  = -16
 CustomerInventoryCapacity(3,1,2): I_customer(3)(1)(2)#12 <= 54
 CustomerInventoryCapacity(4,1,2): I_customer(4)(1)(2)#15 <= 22
 CustomerInventoryCapacity(5,1,2): I_customer(5)(1)(2)#18 <= 19
 CustomerInventoryCapacity(6,1,2): I_customer(6)(1)(2)#21 <= 47
 CustomerInventoryCapacity(7,1,2): I_customer(7)(1)(2)#24 <= 22
 CustomerInventoryCapacity(8,1,2): I_customer(8)(1)(2)#27 <= 54
 CustomerInventoryCapacity(9,1,2): I_customer(9)(1)(2)#30 <= 51
 CustomerInventoryCapacity(10,1,2): I_customer(10)(1)(2)#33 <= 50
 CustomerInventoryCapacity(11,1,2): I_customer(11)(1)(2)#36 <= 45
 CustomerInventoryCapacity(12,1,2): I_customer(12)(1)(2)#39 <= 45
 CustomerInventoryCapacity(3,2,2): I_customer(3)(2)(2)#13 <= 56
 CustomerInventoryCapacity(4,2,2): I_customer(4)(2)(2)#16 <= 20
 CustomerInventoryCapacity(5,2,2): I_customer(5)(2)(2)#19 <= 17
 CustomerInventoryCapacity(6,2,2): I_customer(6)(2)(2)#22 <= 48
 CustomerInventoryCapacity(7,2,2): I_customer(7)(2)(2)#25 <= 21
 CustomerInventoryCapacity(8,2,2): I_customer(8)(2)(2)#28 <= 55
 CustomerInventoryCapacity(9,2,2): I_customer(9)(2)(2)#31 <= 50
 CustomerInventoryCapacity(10,2,2): I_customer(10)(2)(2)#34 <= 46
 CustomerInventoryCapacity(11,2,2): I_customer(11)(2)(2)#37 <= 48
 CustomerInventoryCapacity(12,2,2): I_customer(12)(2)(2)#40 <= 42
 CustomerInventoryCapacity(3,3,2): I_customer(3)(3)(2)#14 <= 60
 CustomerInventoryCapacity(4,3,2): I_customer(4)(3)(2)#17 <= 21
 CustomerInventoryCapacity(5,3,2): I_customer(5)(3)(2)#20 <= 17
 CustomerInventoryCapacity(6,3,2): I_customer(6)(3)(2)#23 <= 47
 CustomerInventoryCapacity(7,3,2): I_customer(7)(3)(2)#26 <= 22
 CustomerInventoryCapacity(8,3,2): I_customer(8)(3)(2)#29 <= 53
 CustomerInventoryCapacity(9,3,2): I_customer(9)(3)(2)#32 <= 51
 CustomerInventoryCapacity(10,3,2): I_customer(10)(3)(2)#35 <= 47
 CustomerInventoryCapacity(11,3,2): I_customer(11)(3)(2)#38 <= 46
 CustomerInventoryCapacity(12,3,2): I_customer(12)(3)(2)#41 <= 40
 WarehouseDelivery(1,1,1):       q(1)(1)(1)#72 <= 124
 WarehouseDelivery(2,1,1):       q(2)(1)(1)#75 <= 124
 WarehouseDelivery(1,1,2):       q(1)(1)(2)#73 <= 124
 WarehouseDelivery(2,1,2):       q(2)(1)(2)#76  = 0
 WarehouseDelivery(1,1,3):       q(1)(1)(3)#74  = 0
 WarehouseDelivery(2,1,3):       q(2)(1)(3)#77  = 0
 VehicleCapacityPlant(1,1):      q(1)(1)(1)#72 + q(2)(1)(1)#75 <= 570
 VehicleCapacityPlant(1,2):      q(1)(1)(2)#73 + q(2)(1)(2)#76 <= 570
 VehicleCapacityPlant(1,3):      q(1)(1)(3)#74 + q(2)(1)(3)#77 <= 570
 CustomerVisit(3,1,1,2):         w(3)(1)(1)(2)#78  = 0
 CustomerVisit(4,1,1,2):         w(4)(1)(1)(2)#84  = 0
 CustomerVisit(5,1,1,2):         w(5)(1)(1)(2)#90  = 0
 CustomerVisit(6,1,1,2):         w(6)(1)(1)(2)#96  = 0
 CustomerVisit(7,1,1,2):         w(7)(1)(1)(2)#102  = 0
 CustomerVisit(8,1,1,2):         w(8)(1)(1)(2)#108  = 0
 CustomerVisit(9,1,1,2):         w(9)(1)(1)(2)#114  = 0
 CustomerVisit(10,1,1,2):        w(10)(1)(1)(2)#120  = 0
 CustomerVisit(11,1,1,2):        w(11)(1)(1)(2)#126  = 0
 CustomerVisit(12,1,1,2):        w(12)(1)(1)(2)#132  = 0
 CustomerVisit(3,1,2,2):         w(3)(1)(2)(2)#79  = 0
 CustomerVisit(5,1,2,2):         w(5)(1)(2)(2)#91  = 0
 CustomerVisit(7,1,2,2):         w(7)(1)(2)(2)#103  = 0
 CustomerVisit(9,1,2,2):         w(9)(1)(2)(2)#115  = 0
 CustomerVisit(11,1,2,2):        w(11)(1)(2)(2)#127  = 0
 CustomerVisit(12,1,2,2):        w(12)(1)(2)(2)#133  = 0
 CustomerVisit(3,2,1,2):         w(3)(2)(1)(2)#81  = 0
 CustomerVisit(4,2,1,2):         w(4)(2)(1)(2)#87  = 0
 CustomerVisit(5,2,1,2):         w(5)(2)(1)(2)#93  = 0
 CustomerVisit(7,2,1,2):         w(7)(2)(1)(2)#105  = 0
 CustomerVisit(8,2,1,2):         w(8)(2)(1)(2)#111  = 0
 CustomerVisit(9,2,1,2):         w(9)(2)(1)(2)#117  = 0
 CustomerVisit(10,2,1,2):        w(10)(2)(1)(2)#123  = 0
 CustomerVisit(11,2,1,2):        w(11)(2)(1)(2)#129  = 0
 CustomerVisit(12,2,1,2):        w(12)(2)(1)(2)#135  = 0
 CustomerVisit(4,2,2,2):         w(4)(2)(2)(2)#88  = 0
 CustomerVisit(6,2,2,2):         w(6)(2)(2)(2)#100  = 0
 CustomerVisit(7,2,2,2):         w(7)(2)(2)(2)#106  = 0
 CustomerVisit(8,2,2,2):         w(8)(2)(2)(2)#112  = 0
 CustomerVisit(9,2,2,2):         w(9)(2)(2)(2)#118  = 0
 CustomerVisit(10,2,2,2):        w(10)(2)(2)(2)#124  = 0
 CustomerVisit(11,2,2,2):        w(11)(2)(2)(2)#130  = 0
 CustomerVisit(12,2,2,2):        w(12)(2)(2)(2)#136  = 0
 CustomerVisit(3,2,3,2):         w(3)(2)(3)(2)#83  = 0
 CustomerVisit(4,2,3,2):         w(4)(2)(3)(2)#89  = 0
 CustomerVisit(5,2,3,2):         w(5)(2)(3)(2)#95  = 0
 CustomerVisit(6,2,3,2):         w(6)(2)(3)(2)#101  = 0
 CustomerVisit(7,2,3,2):         w(7)(2)(3)(2)#107  = 0
 CustomerVisit(8,2,3,2):         w(8)(2)(3)(2)#113  = 0
 CustomerVisit(9,2,3,2):         w(9)(2)(3)(2)#119  = 0
 CustomerVisit(10,2,3,2):        w(10)(2)(3)(2)#125  = 0
 CustomerVisit(11,2,3,2):        w(11)(2)(3)(2)#131  = 0
 CustomerVisit(12,2,3,2):        w(12)(2)(3)(2)#137  = 0
 VehicleCapacityWarehouse(1,1,1,2): w(3)(1)(1)(2)#78 + w(4)(1)(1)(2)#84
                                 + w(5)(1)(1)(2)#90 + w(6)(1)(1)(2)#96
                                 + w(7)(1)(1)(2)#102 + w(8)(1)(1)(2)#108
                                 + w(9)(1)(1)(2)#114 + w(10)(1)(1)(2)#120
                                 + w(11)(1)(1)(2)#126 + w(12)(1)(1)(2)#132
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,2): w(3)(2)(1)(2)#81 + w(4)(2)(1)(2)#87
                                 + w(5)(2)(1)(2)#93 + w(6)(2)(1)(2)#99
                                 + w(7)(2)(1)(2)#105 + w(8)(2)(1)(2)#111
                                 + w(9)(2)(1)(2)#117 + w(10)(2)(1)(2)#123
                                 + w(11)(2)(1)(2)#129 + w(12)(2)(1)(2)#135
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,2): w(3)(1)(2)(2)#79 + w(4)(1)(2)(2)#85
                                 + w(5)(1)(2)(2)#91 + w(6)(1)(2)(2)#97
                                 + w(7)(1)(2)(2)#103 + w(8)(1)(2)(2)#109
                                 + w(9)(1)(2)(2)#115 + w(10)(1)(2)(2)#121
                                 + w(11)(1)(2)(2)#127 + w(12)(1)(2)(2)#133
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,2): w(3)(2)(2)(2)#82 + w(4)(2)(2)(2)#88
                                 + w(5)(2)(2)(2)#94 + w(6)(2)(2)(2)#100
                                 + w(7)(2)(2)(2)#106 + w(8)(2)(2)(2)#112
                                 + w(9)(2)(2)(2)#118 + w(10)(2)(2)(2)#124
                                 + w(11)(2)(2)(2)#130 + w(12)(2)(2)(2)#136
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,2): w(3)(1)(3)(2)#80 + w(4)(1)(3)(2)#86
                                 + w(5)(1)(3)(2)#92 + w(6)(1)(3)(2)#98
                                 + w(7)(1)(3)(2)#104 + w(8)(1)(3)(2)#110
                                 + w(9)(1)(3)(2)#116 + w(10)(1)(3)(2)#122
                                 + w(11)(1)(3)(2)#128 + w(12)(1)(3)(2)#134
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,2): w(3)(2)(3)(2)#83 + w(4)(2)(3)(2)#89
                                 + w(5)(2)(3)(2)#95 + w(6)(2)(3)(2)#101
                                 + w(7)(2)(3)(2)#107 + w(8)(2)(3)(2)#113
                                 + w(9)(2)(3)(2)#119 + w(10)(2)(3)(2)#125
                                 + w(11)(2)(3)(2)#131 + w(12)(2)(3)(2)#137
                                 <= 228
Bounds
 0 <= p(1)#0 <= 270
 0 <= p(2)#1 <= 270
 0 <= p(3)#2 <= 270
 0 <= I_plant(1)#3 <= 135
 0 <= I_plant(2)#4 <= 135
 0 <= I_plant(3)#5 <= 135
 0 <= I_warehouse(1)(1)(2)#6 <= 124
 0 <= I_warehouse(1)(2)(2)#7 <= 124
 0 <= I_warehouse(1)(3)(2)#8 <= 124
 0 <= I_warehouse(2)(1)(2)#9 <= 124
 0 <= I_warehouse(2)(2)(2)#10 <= 124
 0 <= I_warehouse(2)(3)(2)#11 <= 124
 0 <= I_customer(3)(1)(2)#12 <= 76
 0 <= I_customer(3)(2)(2)#13 <= 76
 0 <= I_customer(3)(3)(2)#14 <= 76
 0 <= I_customer(4)(1)(2)#15 <= 28
 0 <= I_customer(4)(2)(2)#16 <= 28
 0 <= I_customer(4)(3)(2)#17 <= 28
 0 <= I_customer(5)(1)(2)#18 <= 24
 0 <= I_customer(5)(2)(2)#19 <= 24
 0 <= I_customer(5)(3)(2)#20 <= 24
 0 <= I_customer(6)(1)(2)#21 <= 64
 0 <= I_customer(6)(2)(2)#22 <= 64
 0 <= I_customer(6)(3)(2)#23 <= 64
 0 <= I_customer(7)(1)(2)#24 <= 28
 0 <= I_customer(7)(2)(2)#25 <= 28
 0 <= I_customer(7)(3)(2)#26 <= 28
 0 <= I_customer(8)(1)(2)#27 <= 72
 0 <= I_customer(8)(2)(2)#28 <= 72
 0 <= I_customer(8)(3)(2)#29 <= 72
 0 <= I_customer(9)(1)(2)#30 <= 64
 0 <= I_customer(9)(2)(2)#31 <= 64
 0 <= I_customer(9)(3)(2)#32 <= 64
 0 <= I_customer(10)(1)(2)#33 <= 64
 0 <= I_customer(10)(2)(2)#34 <= 64
 0 <= I_customer(10)(3)(2)#35 <= 64
 0 <= I_customer(11)(1)(2)#36 <= 64
 0 <= I_customer(11)(2)(2)#37 <= 64
 0 <= I_customer(11)(3)(2)#38 <= 64
 0 <= I_customer(12)(1)(2)#39 <= 56
 0 <= I_customer(12)(2)(2)#40 <= 56
 0 <= I_customer(12)(3)(2)#41 <= 56
 0 <= b_customer(3)(1)(2)#42 <= 22
 0 <= b_customer(3)(2)(2)#43 <= 20
 0 <= b_customer(3)(3)(2)#44 <= 16
 0 <= b_customer(4)(1)(2)#45 <= 6
 0 <= b_customer(4)(2)(2)#46 <= 8
 0 <= b_customer(4)(3)(2)#47 <= 7
 0 <= b_customer(5)(1)(2)#48 <= 5
 0 <= b_customer(5)(2)(2)#49 <= 7
 0 <= b_customer(5)(3)(2)#50 <= 7
 0 <= b_customer(6)(1)(2)#51 <= 17
 0 <= b_customer(6)(2)(2)#52 <= 16
 0 <= b_customer(6)(3)(2)#53 <= 17
 0 <= b_customer(7)(1)(2)#54 <= 6
 0 <= b_customer(7)(2)(2)#55 <= 7
 0 <= b_customer(7)(3)(2)#56 <= 6
 0 <= b_customer(8)(1)(2)#57 <= 18
 0 <= b_customer(8)(2)(2)#58 <= 17
 0 <= b_customer(8)(3)(2)#59 <= 19
 0 <= b_customer(9)(1)(2)#60 <= 13
 0 <= b_customer(9)(2)(2)#61 <= 14
 0 <= b_customer(9)(3)(2)#62 <= 13
 0 <= b_customer(10)(1)(2)#63 <= 14
 0 <= b_customer(10)(2)(2)#64 <= 18
 0 <= b_customer(10)(3)(2)#65 <= 17
 0 <= b_customer(11)(1)(2)#66 <= 19
 0 <= b_customer(11)(2)(2)#67 <= 16
 0 <= b_customer(11)(3)(2)#68 <= 18
 0 <= b_customer(12)(1)(2)#69 <= 11
 0 <= b_customer(12)(2)(2)#70 <= 14
 0 <= b_customer(12)(3)(2)#71 <= 16
 0 <= q(1)(1)(1)#72 <= 124
 0 <= q(1)(1)(2)#73 <= 124
 0 <= q(1)(1)(3)#74 <= 124
 0 <= q(2)(1)(1)#75 <= 124
 0 <= q(2)(1)(2)#76 <= 124
 0 <= q(2)(1)(3)#77 <= 124
 0 <= w(3)(1)(1)(2)#78 <= 22
 0 <= w(3)(1)(2)(2)#79 <= 20
 0 <= w(3)(1)(3)(2)#80 <= 16
 0 <= w(3)(2)(1)(2)#81 <= 22
 0 <= w(3)(2)(2)(2)#82 <= 20
 0 <= w(3)(2)(3)(2)#83 <= 16
 0 <= w(4)(1)(1)(2)#84 <= 6
 0 <= w(4)(1)(2)(2)#85 <= 8
 0 <= w(4)(1)(3)(2)#86 <= 7
 0 <= w(4)(2)(1)(2)#87 <= 6
 0 <= w(4)(2)(2)(2)#88 <= 8
 0 <= w(4)(2)(3)(2)#89 <= 7
 0 <= w(5)(1)(1)(2)#90 <= 5
 0 <= w(5)(1)(2)(2)#91 <= 7
 0 <= w(5)(1)(3)(2)#92 <= 7
 0 <= w(5)(2)(1)(2)#93 <= 5
 0 <= w(5)(2)(2)(2)#94 <= 7
 0 <= w(5)(2)(3)(2)#95 <= 7
 0 <= w(6)(1)(1)(2)#96 <= 17
 0 <= w(6)(1)(2)(2)#97 <= 16
 0 <= w(6)(1)(3)(2)#98 <= 17
 0 <= w(6)(2)(1)(2)#99 <= 17
 0 <= w(6)(2)(2)(2)#100 <= 16
 0 <= w(6)(2)(3)(2)#101 <= 17
 0 <= w(7)(1)(1)(2)#102 <= 6
 0 <= w(7)(1)(2)(2)#103 <= 7
 0 <= w(7)(1)(3)(2)#104 <= 6
 0 <= w(7)(2)(1)(2)#105 <= 6
 0 <= w(7)(2)(2)(2)#106 <= 7
 0 <= w(7)(2)(3)(2)#107 <= 6
 0 <= w(8)(1)(1)(2)#108 <= 18
 0 <= w(8)(1)(2)(2)#109 <= 17
 0 <= w(8)(1)(3)(2)#110 <= 19
 0 <= w(8)(2)(1)(2)#111 <= 18
 0 <= w(8)(2)(2)(2)#112 <= 17
 0 <= w(8)(2)(3)(2)#113 <= 19
 0 <= w(9)(1)(1)(2)#114 <= 13
 0 <= w(9)(1)(2)(2)#115 <= 14
 0 <= w(9)(1)(3)(2)#116 <= 13
 0 <= w(9)(2)(1)(2)#117 <= 13
 0 <= w(9)(2)(2)(2)#118 <= 14
 0 <= w(9)(2)(3)(2)#119 <= 13
 0 <= w(10)(1)(1)(2)#120 <= 14
 0 <= w(10)(1)(2)(2)#121 <= 18
 0 <= w(10)(1)(3)(2)#122 <= 17
 0 <= w(10)(2)(1)(2)#123 <= 14
 0 <= w(10)(2)(2)(2)#124 <= 18
 0 <= w(10)(2)(3)(2)#125 <= 17
 0 <= w(11)(1)(1)(2)#126 <= 19
 0 <= w(11)(1)(2)(2)#127 <= 16
 0 <= w(11)(1)(3)(2)#128 <= 18
 0 <= w(11)(2)(1)(2)#129 <= 19
 0 <= w(11)(2)(2)(2)#130 <= 16
 0 <= w(11)(2)(3)(2)#131 <= 18
 0 <= w(12)(1)(1)(2)#132 <= 11
 0 <= w(12)(1)(2)(2)#133 <= 14
 0 <= w(12)(1)(3)(2)#134 <= 16
 0 <= w(12)(2)(1)(2)#135 <= 11
 0 <= w(12)(2)(2)(2)#136 <= 14
 0 <= w(12)(2)(3)(2)#137 <= 16
End
