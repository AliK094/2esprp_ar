\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 30 p(1)#0 + 30 p(2)#1 + 30 p(3)#2 + 3 I_plant(1)#3 + 3 I_plant(2)#4
       + 3 I_plant(3)#5 + 5 I_warehouse(1)(1)(1)#6 + 5 I_warehouse(1)(2)(1)#7
       + 5 I_warehouse(1)(3)(1)#8 + 5 I_warehouse(2)(1)(1)#9
       + 5 I_warehouse(2)(2)(1)#10 + 5 I_warehouse(2)(3)(1)#11
       + 9 I_customer(3)(1)(1)#12 + 9 I_customer(3)(2)(1)#13
       + 9 I_customer(3)(3)(1)#14 + 10 I_customer(4)(1)(1)#15
       + 10 I_customer(4)(2)(1)#16 + 10 I_customer(4)(3)(1)#17
       + 10 I_customer(5)(1)(1)#18 + 10 I_customer(5)(2)(1)#19
       + 10 I_customer(5)(3)(1)#20 + 8 I_customer(6)(1)(1)#21
       + 8 I_customer(6)(2)(1)#22 + 8 I_customer(6)(3)(1)#23
       + 8 I_customer(7)(1)(1)#24 + 8 I_customer(7)(2)(1)#25
       + 8 I_customer(7)(3)(1)#26 + 10 I_customer(8)(1)(1)#27
       + 10 I_customer(8)(2)(1)#28 + 10 I_customer(8)(3)(1)#29
       + 10 I_customer(9)(1)(1)#30 + 10 I_customer(9)(2)(1)#31
       + 10 I_customer(9)(3)(1)#32 + 7 I_customer(10)(1)(1)#33
       + 7 I_customer(10)(2)(1)#34 + 7 I_customer(10)(3)(1)#35
       + 9 I_customer(11)(1)(1)#36 + 9 I_customer(11)(2)(1)#37
       + 9 I_customer(11)(3)(1)#38 + 7 I_customer(12)(1)(1)#39
       + 7 I_customer(12)(2)(1)#40 + 7 I_customer(12)(3)(1)#41
       + 470 b_customer(3)(1)(1)#42 + 470 b_customer(3)(2)(1)#43
       + 470 b_customer(3)(3)(1)#44 + 460 b_customer(4)(1)(1)#45
       + 460 b_customer(4)(2)(1)#46 + 460 b_customer(4)(3)(1)#47
       + 480 b_customer(5)(1)(1)#48 + 480 b_customer(5)(2)(1)#49
       + 480 b_customer(5)(3)(1)#50 + 470 b_customer(6)(1)(1)#51
       + 470 b_customer(6)(2)(1)#52 + 470 b_customer(6)(3)(1)#53
       + 480 b_customer(7)(1)(1)#54 + 480 b_customer(7)(2)(1)#55
       + 480 b_customer(7)(3)(1)#56 + 470 b_customer(8)(1)(1)#57
       + 470 b_customer(8)(2)(1)#58 + 470 b_customer(8)(3)(1)#59
       + 470 b_customer(9)(1)(1)#60 + 470 b_customer(9)(2)(1)#61
       + 470 b_customer(9)(3)(1)#62 + 460 b_customer(10)(1)(1)#63
       + 460 b_customer(10)(2)(1)#64 + 460 b_customer(10)(3)(1)#65
       + 460 b_customer(11)(1)(1)#66 + 460 b_customer(11)(2)(1)#67
       + 460 b_customer(11)(3)(1)#68 + 460 b_customer(12)(1)(1)#69
       + 460 b_customer(12)(2)(1)#70 + 460 b_customer(12)(3)(1)#71
Subject To
 c1:                             p(1)#0  = 270
 c2:                             p(2)#1  = 0
 c3:                             p(3)#2  = 0
 c4:                             I_plant(1)#3  = 93
 c5:                             I_plant(2)#4  = 0
 c6:                             I_plant(3)#5  = 0
 ProductionCapacity(1):          p(1)#0 <= 270
 ProductionCapacity(2):          p(2)#1 <= 0
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
 WarehouseInventoryCapacity(1,1,1): I_warehouse(1)(1)(1)#6 <= 124
 WarehouseInventoryCapacity(2,1,1): I_warehouse(2)(1)(1)#9 <= 124
 WarehouseInventoryCapacity(1,2,1): I_warehouse(1)(2)(1)#7 <= 124
 WarehouseInventoryCapacity(2,2,1): I_warehouse(2)(2)(1)#10 <= 124
 WarehouseInventoryCapacity(1,3,1): I_warehouse(1)(3)(1)#8 <= 124
 WarehouseInventoryCapacity(2,3,1): I_warehouse(2)(3)(1)#11 <= 124
 WarehouseInventoryBalance(1,1,1): I_warehouse(1)(1)(1)#6 - q(1)(1)(1)#72
                                 + w(3)(1)(1)(1)#78 + w(4)(1)(1)(1)#84
                                 + w(5)(1)(1)(1)#90 + w(6)(1)(1)(1)#96
                                 + w(7)(1)(1)(1)#102 + w(8)(1)(1)(1)#108
                                 + w(9)(1)(1)(1)#114 + w(10)(1)(1)(1)#120
                                 + w(11)(1)(1)(1)#126 + w(12)(1)(1)(1)#132  = 0
 WarehouseInventoryBalance(2,1,1): I_warehouse(2)(1)(1)#9 - q(2)(1)(1)#75
                                 + w(3)(2)(1)(1)#81 + w(4)(2)(1)(1)#87
                                 + w(5)(2)(1)(1)#93 + w(6)(2)(1)(1)#99
                                 + w(7)(2)(1)(1)#105 + w(8)(2)(1)(1)#111
                                 + w(9)(2)(1)(1)#117 + w(10)(2)(1)(1)#123
                                 + w(11)(2)(1)(1)#129 + w(12)(2)(1)(1)#135  = 0
 WarehouseInventoryBalance(1,2,1): - I_warehouse(1)(1)(1)#6
                                 + I_warehouse(1)(2)(1)#7 - q(1)(1)(2)#73
                                 + w(3)(1)(2)(1)#79 + w(4)(1)(2)(1)#85
                                 + w(5)(1)(2)(1)#91 + w(6)(1)(2)(1)#97
                                 + w(7)(1)(2)(1)#103 + w(8)(1)(2)(1)#109
                                 + w(9)(1)(2)(1)#115 + w(10)(1)(2)(1)#121
                                 + w(11)(1)(2)(1)#127 + w(12)(1)(2)(1)#133  = 0
 WarehouseInventoryBalance(2,2,1): - I_warehouse(2)(1)(1)#9
                                 + I_warehouse(2)(2)(1)#10 - q(2)(1)(2)#76
                                 + w(3)(2)(2)(1)#82 + w(4)(2)(2)(1)#88
                                 + w(5)(2)(2)(1)#94 + w(6)(2)(2)(1)#100
                                 + w(7)(2)(2)(1)#106 + w(8)(2)(2)(1)#112
                                 + w(9)(2)(2)(1)#118 + w(10)(2)(2)(1)#124
                                 + w(11)(2)(2)(1)#130 + w(12)(2)(2)(1)#136  = 0
 WarehouseInventoryBalance(1,3,1): - I_warehouse(1)(2)(1)#7
                                 + I_warehouse(1)(3)(1)#8 - q(1)(1)(3)#74
                                 + w(3)(1)(3)(1)#80 + w(4)(1)(3)(1)#86
                                 + w(5)(1)(3)(1)#92 + w(6)(1)(3)(1)#98
                                 + w(7)(1)(3)(1)#104 + w(8)(1)(3)(1)#110
                                 + w(9)(1)(3)(1)#116 + w(10)(1)(3)(1)#122
                                 + w(11)(1)(3)(1)#128 + w(12)(1)(3)(1)#134  = 0
 WarehouseInventoryBalance(2,3,1): - I_warehouse(2)(2)(1)#10
                                 + I_warehouse(2)(3)(1)#11 - q(2)(1)(3)#77
                                 + w(3)(2)(3)(1)#83 + w(4)(2)(3)(1)#89
                                 + w(5)(2)(3)(1)#95 + w(6)(2)(3)(1)#101
                                 + w(7)(2)(3)(1)#107 + w(8)(2)(3)(1)#113
                                 + w(9)(2)(3)(1)#119 + w(10)(2)(3)(1)#125
                                 + w(11)(2)(3)(1)#131 + w(12)(2)(3)(1)#137  = 0
 CustomerInventoryBalance(3,1,1): I_customer(3)(1)(1)#12
                                 - b_customer(3)(1)(1)#42 - w(3)(1)(1)(1)#78
                                 - w(3)(2)(1)(1)#81  = 20
 CustomerInventoryBalance(4,1,1): I_customer(4)(1)(1)#15
                                 - b_customer(4)(1)(1)#45 - w(4)(1)(1)(1)#84
                                 - w(4)(2)(1)(1)#87  = -1
 CustomerInventoryBalance(5,1,1): I_customer(5)(1)(1)#18
                                 - b_customer(5)(1)(1)#48 - w(5)(1)(1)(1)#90
                                 - w(5)(2)(1)(1)#93  = 1
 CustomerInventoryBalance(6,1,1): I_customer(6)(1)(1)#21
                                 - b_customer(6)(1)(1)#51 - w(6)(1)(1)(1)#96
                                 - w(6)(2)(1)(1)#99  = 3
 CustomerInventoryBalance(7,1,1): I_customer(7)(1)(1)#24
                                 - b_customer(7)(1)(1)#54 - w(7)(1)(1)(1)#102
                                 - w(7)(2)(1)(1)#105  = 6
 CustomerInventoryBalance(8,1,1): I_customer(8)(1)(1)#27
                                 - b_customer(8)(1)(1)#57 - w(8)(1)(1)(1)#108
                                 - w(8)(2)(1)(1)#111  = 3
 CustomerInventoryBalance(9,1,1): I_customer(9)(1)(1)#30
                                 - b_customer(9)(1)(1)#60 - w(9)(1)(1)(1)#114
                                 - w(9)(2)(1)(1)#117  = 13
 CustomerInventoryBalance(10,1,1): I_customer(10)(1)(1)#33
                                 - b_customer(10)(1)(1)#63 - w(10)(1)(1)(1)#120
                                 - w(10)(2)(1)(1)#123  = 2
 CustomerInventoryBalance(11,1,1): I_customer(11)(1)(1)#36
                                 - b_customer(11)(1)(1)#66 - w(11)(1)(1)(1)#126
                                 - w(11)(2)(1)(1)#129  = 3
 CustomerInventoryBalance(12,1,1): I_customer(12)(1)(1)#39
                                 - b_customer(12)(1)(1)#69 - w(12)(1)(1)(1)#132
                                 - w(12)(2)(1)(1)#135  = 15
 CustomerInventoryBalance(3,2,1): - I_customer(3)(1)(1)#12
                                 + I_customer(3)(2)(1)#13
                                 - b_customer(3)(2)(1)#43 - w(3)(1)(2)(1)#79
                                 - w(3)(2)(2)(1)#82  = -17
 CustomerInventoryBalance(4,2,1): - I_customer(4)(1)(1)#15
                                 + I_customer(4)(2)(1)#16
                                 - b_customer(4)(2)(1)#46 - w(4)(1)(2)(1)#85
                                 - w(4)(2)(2)(1)#88  = -6
 CustomerInventoryBalance(5,2,1): - I_customer(5)(1)(1)#18
                                 + I_customer(5)(2)(1)#19
                                 - b_customer(5)(2)(1)#49 - w(5)(1)(2)(1)#91
                                 - w(5)(2)(2)(1)#94  = -6
 CustomerInventoryBalance(6,2,1): - I_customer(6)(1)(1)#21
                                 + I_customer(6)(2)(1)#22
                                 - b_customer(6)(2)(1)#52 - w(6)(1)(2)(1)#97
                                 - w(6)(2)(2)(1)#100  = -17
 CustomerInventoryBalance(7,2,1): - I_customer(7)(1)(1)#24
                                 + I_customer(7)(2)(1)#25
                                 - b_customer(7)(2)(1)#55 - w(7)(1)(2)(1)#103
                                 - w(7)(2)(2)(1)#106  = -8
 CustomerInventoryBalance(8,2,1): - I_customer(8)(1)(1)#27
                                 + I_customer(8)(2)(1)#28
                                 - b_customer(8)(2)(1)#58 - w(8)(1)(2)(1)#109
                                 - w(8)(2)(2)(1)#112  = -20
 CustomerInventoryBalance(9,2,1): - I_customer(9)(1)(1)#30
                                 + I_customer(9)(2)(1)#31
                                 - b_customer(9)(2)(1)#61 - w(9)(1)(2)(1)#115
                                 - w(9)(2)(2)(1)#118  = -15
 CustomerInventoryBalance(10,2,1): - I_customer(10)(1)(1)#33
                                 + I_customer(10)(2)(1)#34
                                 - b_customer(10)(2)(1)#64 - w(10)(1)(2)(1)#121
                                 - w(10)(2)(2)(1)#124  = -14
 CustomerInventoryBalance(11,2,1): - I_customer(11)(1)(1)#36
                                 + I_customer(11)(2)(1)#37
                                 - b_customer(11)(2)(1)#67 - w(11)(1)(2)(1)#127
                                 - w(11)(2)(2)(1)#130  = -13
 CustomerInventoryBalance(12,2,1): - I_customer(12)(1)(1)#39
                                 + I_customer(12)(2)(1)#40
                                 - b_customer(12)(2)(1)#70 - w(12)(1)(2)(1)#133
                                 - w(12)(2)(2)(1)#136  = -14
 CustomerInventoryBalance(3,3,1): - I_customer(3)(2)(1)#13
                                 + I_customer(3)(3)(1)#14
                                 - b_customer(3)(3)(1)#44 - w(3)(1)(3)(1)#80
                                 - w(3)(2)(3)(1)#83  = -22
 CustomerInventoryBalance(4,3,1): - I_customer(4)(2)(1)#16
                                 + I_customer(4)(3)(1)#17
                                 - b_customer(4)(3)(1)#47 - w(4)(1)(3)(1)#86
                                 - w(4)(2)(3)(1)#89  = -8
 CustomerInventoryBalance(5,3,1): - I_customer(5)(2)(1)#19
                                 + I_customer(5)(3)(1)#20
                                 - b_customer(5)(3)(1)#50 - w(5)(1)(3)(1)#92
                                 - w(5)(2)(3)(1)#95  = -6
 CustomerInventoryBalance(6,3,1): - I_customer(6)(2)(1)#22
                                 + I_customer(6)(3)(1)#23
                                 - b_customer(6)(3)(1)#53 - w(6)(1)(3)(1)#98
                                 - w(6)(2)(3)(1)#101  = -17
 CustomerInventoryBalance(7,3,1): - I_customer(7)(2)(1)#25
                                 + I_customer(7)(3)(1)#26
                                 - b_customer(7)(3)(1)#56 - w(7)(1)(3)(1)#104
                                 - w(7)(2)(3)(1)#107  = -6
 CustomerInventoryBalance(8,3,1): - I_customer(8)(2)(1)#28
                                 + I_customer(8)(3)(1)#29
                                 - b_customer(8)(3)(1)#59 - w(8)(1)(3)(1)#110
                                 - w(8)(2)(3)(1)#113  = -19
 CustomerInventoryBalance(9,3,1): - I_customer(9)(2)(1)#31
                                 + I_customer(9)(3)(1)#32
                                 - b_customer(9)(3)(1)#62 - w(9)(1)(3)(1)#116
                                 - w(9)(2)(3)(1)#119  = -17
 CustomerInventoryBalance(10,3,1): - I_customer(10)(2)(1)#34
                                 + I_customer(10)(3)(1)#35
                                 - b_customer(10)(3)(1)#65 - w(10)(1)(3)(1)#122
                                 - w(10)(2)(3)(1)#125  = -16
 CustomerInventoryBalance(11,3,1): - I_customer(11)(2)(1)#37
                                 + I_customer(11)(3)(1)#38
                                 - b_customer(11)(3)(1)#68 - w(11)(1)(3)(1)#128
                                 - w(11)(2)(3)(1)#131  = -14
 CustomerInventoryBalance(12,3,1): - I_customer(12)(2)(1)#40
                                 + I_customer(12)(3)(1)#41
                                 - b_customer(12)(3)(1)#71 - w(12)(1)(3)(1)#134
                                 - w(12)(2)(3)(1)#137  = -16
 CustomerInventoryCapacity(3,1,1): I_customer(3)(1)(1)#12 <= 58
 CustomerInventoryCapacity(4,1,1): I_customer(4)(1)(1)#15 <= 20
 CustomerInventoryCapacity(5,1,1): I_customer(5)(1)(1)#18 <= 19
 CustomerInventoryCapacity(6,1,1): I_customer(6)(1)(1)#21 <= 51
 CustomerInventoryCapacity(7,1,1): I_customer(7)(1)(1)#24 <= 20
 CustomerInventoryCapacity(8,1,1): I_customer(8)(1)(1)#27 <= 57
 CustomerInventoryCapacity(9,1,1): I_customer(9)(1)(1)#30 <= 45
 CustomerInventoryCapacity(10,1,1): I_customer(10)(1)(1)#33 <= 50
 CustomerInventoryCapacity(11,1,1): I_customer(11)(1)(1)#36 <= 51
 CustomerInventoryCapacity(12,1,1): I_customer(12)(1)(1)#39 <= 43
 CustomerInventoryCapacity(3,2,1): I_customer(3)(2)(1)#13 <= 59
 CustomerInventoryCapacity(4,2,1): I_customer(4)(2)(1)#16 <= 22
 CustomerInventoryCapacity(5,2,1): I_customer(5)(2)(1)#19 <= 18
 CustomerInventoryCapacity(6,2,1): I_customer(6)(2)(1)#22 <= 47
 CustomerInventoryCapacity(7,2,1): I_customer(7)(2)(1)#25 <= 20
 CustomerInventoryCapacity(8,2,1): I_customer(8)(2)(1)#28 <= 52
 CustomerInventoryCapacity(9,2,1): I_customer(9)(2)(1)#31 <= 49
 CustomerInventoryCapacity(10,2,1): I_customer(10)(2)(1)#34 <= 50
 CustomerInventoryCapacity(11,2,1): I_customer(11)(2)(1)#37 <= 51
 CustomerInventoryCapacity(12,2,1): I_customer(12)(2)(1)#40 <= 42
 CustomerInventoryCapacity(3,3,1): I_customer(3)(3)(1)#14 <= 54
 CustomerInventoryCapacity(4,3,1): I_customer(4)(3)(1)#17 <= 20
 CustomerInventoryCapacity(5,3,1): I_customer(5)(3)(1)#20 <= 18
 CustomerInventoryCapacity(6,3,1): I_customer(6)(3)(1)#23 <= 47
 CustomerInventoryCapacity(7,3,1): I_customer(7)(3)(1)#26 <= 22
 CustomerInventoryCapacity(8,3,1): I_customer(8)(3)(1)#29 <= 53
 CustomerInventoryCapacity(9,3,1): I_customer(9)(3)(1)#32 <= 47
 CustomerInventoryCapacity(10,3,1): I_customer(10)(3)(1)#35 <= 48
 CustomerInventoryCapacity(11,3,1): I_customer(11)(3)(1)#38 <= 50
 CustomerInventoryCapacity(12,3,1): I_customer(12)(3)(1)#41 <= 40
 WarehouseDelivery(1,1,1):       q(1)(1)(1)#72 <= 124
 WarehouseDelivery(2,1,1):       q(2)(1)(1)#75 <= 124
 WarehouseDelivery(1,1,2):       q(1)(1)(2)#73 <= 124
 WarehouseDelivery(2,1,2):       q(2)(1)(2)#76  = 0
 WarehouseDelivery(1,1,3):       q(1)(1)(3)#74  = 0
 WarehouseDelivery(2,1,3):       q(2)(1)(3)#77  = 0
 VehicleCapacityPlant(1,1):      q(1)(1)(1)#72 + q(2)(1)(1)#75 <= 570
 VehicleCapacityPlant(1,2):      q(1)(1)(2)#73 + q(2)(1)(2)#76 <= 570
 VehicleCapacityPlant(1,3):      q(1)(1)(3)#74 + q(2)(1)(3)#77 <= 570
 CustomerVisit(3,1,1,1):         w(3)(1)(1)(1)#78  = 0
 CustomerVisit(4,1,1,1):         w(4)(1)(1)(1)#84  = 0
 CustomerVisit(5,1,1,1):         w(5)(1)(1)(1)#90  = 0
 CustomerVisit(7,1,1,1):         w(7)(1)(1)(1)#102  = 0
 CustomerVisit(8,1,1,1):         w(8)(1)(1)(1)#108  = 0
 CustomerVisit(9,1,1,1):         w(9)(1)(1)(1)#114  = 0
 CustomerVisit(10,1,1,1):        w(10)(1)(1)(1)#120  = 0
 CustomerVisit(11,1,1,1):        w(11)(1)(1)(1)#126  = 0
 CustomerVisit(12,1,1,1):        w(12)(1)(1)(1)#132  = 0
 CustomerVisit(3,1,2,1):         w(3)(1)(2)(1)#79  = 0
 CustomerVisit(4,1,2,1):         w(4)(1)(2)(1)#85  = 0
 CustomerVisit(5,1,2,1):         w(5)(1)(2)(1)#91  = 0
 CustomerVisit(6,1,2,1):         w(6)(1)(2)(1)#97  = 0
 CustomerVisit(7,1,2,1):         w(7)(1)(2)(1)#103  = 0
 CustomerVisit(8,1,2,1):         w(8)(1)(2)(1)#109  = 0
 CustomerVisit(9,1,2,1):         w(9)(1)(2)(1)#115  = 0
 CustomerVisit(10,1,2,1):        w(10)(1)(2)(1)#121  = 0
 CustomerVisit(11,1,2,1):        w(11)(1)(2)(1)#127  = 0
 CustomerVisit(12,1,2,1):        w(12)(1)(2)(1)#133  = 0
 CustomerVisit(3,1,3,1):         w(3)(1)(3)(1)#80  = 0
 CustomerVisit(5,1,3,1):         w(5)(1)(3)(1)#92  = 0
 CustomerVisit(7,1,3,1):         w(7)(1)(3)(1)#104  = 0
 CustomerVisit(9,1,3,1):         w(9)(1)(3)(1)#116  = 0
 CustomerVisit(6,2,1,1):         w(6)(2)(1)(1)#99  = 0
 CustomerVisit(7,2,1,1):         w(7)(2)(1)(1)#105  = 0
 CustomerVisit(8,2,1,1):         w(8)(2)(1)(1)#111  = 0
 CustomerVisit(12,2,1,1):        w(12)(2)(1)(1)#135  = 0
 CustomerVisit(9,2,2,1):         w(9)(2)(2)(1)#118  = 0
 CustomerVisit(10,2,2,1):        w(10)(2)(2)(1)#124  = 0
 CustomerVisit(11,2,2,1):        w(11)(2)(2)(1)#130  = 0
 CustomerVisit(12,2,2,1):        w(12)(2)(2)(1)#136  = 0
 CustomerVisit(3,2,3,1):         w(3)(2)(3)(1)#83  = 0
 CustomerVisit(4,2,3,1):         w(4)(2)(3)(1)#89  = 0
 CustomerVisit(5,2,3,1):         w(5)(2)(3)(1)#95  = 0
 CustomerVisit(6,2,3,1):         w(6)(2)(3)(1)#101  = 0
 CustomerVisit(7,2,3,1):         w(7)(2)(3)(1)#107  = 0
 CustomerVisit(8,2,3,1):         w(8)(2)(3)(1)#113  = 0
 CustomerVisit(9,2,3,1):         w(9)(2)(3)(1)#119  = 0
 CustomerVisit(10,2,3,1):        w(10)(2)(3)(1)#125  = 0
 CustomerVisit(11,2,3,1):        w(11)(2)(3)(1)#131  = 0
 CustomerVisit(12,2,3,1):        w(12)(2)(3)(1)#137  = 0
 VehicleCapacityWarehouse(1,1,1,1): w(3)(1)(1)(1)#78 + w(4)(1)(1)(1)#84
                                 + w(5)(1)(1)(1)#90 + w(6)(1)(1)(1)#96
                                 + w(7)(1)(1)(1)#102 + w(8)(1)(1)(1)#108
                                 + w(9)(1)(1)(1)#114 + w(10)(1)(1)(1)#120
                                 + w(11)(1)(1)(1)#126 + w(12)(1)(1)(1)#132
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,1): w(3)(2)(1)(1)#81 + w(4)(2)(1)(1)#87
                                 + w(5)(2)(1)(1)#93 + w(6)(2)(1)(1)#99
                                 + w(7)(2)(1)(1)#105 + w(8)(2)(1)(1)#111
                                 + w(9)(2)(1)(1)#117 + w(10)(2)(1)(1)#123
                                 + w(11)(2)(1)(1)#129 + w(12)(2)(1)(1)#135
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,1): w(3)(1)(2)(1)#79 + w(4)(1)(2)(1)#85
                                 + w(5)(1)(2)(1)#91 + w(6)(1)(2)(1)#97
                                 + w(7)(1)(2)(1)#103 + w(8)(1)(2)(1)#109
                                 + w(9)(1)(2)(1)#115 + w(10)(1)(2)(1)#121
                                 + w(11)(1)(2)(1)#127 + w(12)(1)(2)(1)#133
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,1): w(3)(2)(2)(1)#82 + w(4)(2)(2)(1)#88
                                 + w(5)(2)(2)(1)#94 + w(6)(2)(2)(1)#100
                                 + w(7)(2)(2)(1)#106 + w(8)(2)(2)(1)#112
                                 + w(9)(2)(2)(1)#118 + w(10)(2)(2)(1)#124
                                 + w(11)(2)(2)(1)#130 + w(12)(2)(2)(1)#136
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,1): w(3)(1)(3)(1)#80 + w(4)(1)(3)(1)#86
                                 + w(5)(1)(3)(1)#92 + w(6)(1)(3)(1)#98
                                 + w(7)(1)(3)(1)#104 + w(8)(1)(3)(1)#110
                                 + w(9)(1)(3)(1)#116 + w(10)(1)(3)(1)#122
                                 + w(11)(1)(3)(1)#128 + w(12)(1)(3)(1)#134
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,1): w(3)(2)(3)(1)#83 + w(4)(2)(3)(1)#89
                                 + w(5)(2)(3)(1)#95 + w(6)(2)(3)(1)#101
                                 + w(7)(2)(3)(1)#107 + w(8)(2)(3)(1)#113
                                 + w(9)(2)(3)(1)#119 + w(10)(2)(3)(1)#125
                                 + w(11)(2)(3)(1)#131 + w(12)(2)(3)(1)#137
                                 <= 228
Bounds
 0 <= p(1)#0 <= 270
 0 <= p(2)#1 <= 270
 0 <= p(3)#2 <= 270
 0 <= I_plant(1)#3 <= 135
 0 <= I_plant(2)#4 <= 135
 0 <= I_plant(3)#5 <= 135
 0 <= I_warehouse(1)(1)(1)#6 <= 124
 0 <= I_warehouse(1)(2)(1)#7 <= 124
 0 <= I_warehouse(1)(3)(1)#8 <= 124
 0 <= I_warehouse(2)(1)(1)#9 <= 124
 0 <= I_warehouse(2)(2)(1)#10 <= 124
 0 <= I_warehouse(2)(3)(1)#11 <= 124
 0 <= I_customer(3)(1)(1)#12 <= 76
 0 <= I_customer(3)(2)(1)#13 <= 76
 0 <= I_customer(3)(3)(1)#14 <= 76
 0 <= I_customer(4)(1)(1)#15 <= 28
 0 <= I_customer(4)(2)(1)#16 <= 28
 0 <= I_customer(4)(3)(1)#17 <= 28
 0 <= I_customer(5)(1)(1)#18 <= 24
 0 <= I_customer(5)(2)(1)#19 <= 24
 0 <= I_customer(5)(3)(1)#20 <= 24
 0 <= I_customer(6)(1)(1)#21 <= 64
 0 <= I_customer(6)(2)(1)#22 <= 64
 0 <= I_customer(6)(3)(1)#23 <= 64
 0 <= I_customer(7)(1)(1)#24 <= 28
 0 <= I_customer(7)(2)(1)#25 <= 28
 0 <= I_customer(7)(3)(1)#26 <= 28
 0 <= I_customer(8)(1)(1)#27 <= 72
 0 <= I_customer(8)(2)(1)#28 <= 72
 0 <= I_customer(8)(3)(1)#29 <= 72
 0 <= I_customer(9)(1)(1)#30 <= 64
 0 <= I_customer(9)(2)(1)#31 <= 64
 0 <= I_customer(9)(3)(1)#32 <= 64
 0 <= I_customer(10)(1)(1)#33 <= 64
 0 <= I_customer(10)(2)(1)#34 <= 64
 0 <= I_customer(10)(3)(1)#35 <= 64
 0 <= I_customer(11)(1)(1)#36 <= 64
 0 <= I_customer(11)(2)(1)#37 <= 64
 0 <= I_customer(11)(3)(1)#38 <= 64
 0 <= I_customer(12)(1)(1)#39 <= 56
 0 <= I_customer(12)(2)(1)#40 <= 56
 0 <= I_customer(12)(3)(1)#41 <= 56
 0 <= b_customer(3)(1)(1)#42 <= 18
 0 <= b_customer(3)(2)(1)#43 <= 17
 0 <= b_customer(3)(3)(1)#44 <= 22
 0 <= b_customer(4)(1)(1)#45 <= 8
 0 <= b_customer(4)(2)(1)#46 <= 6
 0 <= b_customer(4)(3)(1)#47 <= 8
 0 <= b_customer(5)(1)(1)#48 <= 5
 0 <= b_customer(5)(2)(1)#49 <= 6
 0 <= b_customer(5)(3)(1)#50 <= 6
 0 <= b_customer(6)(1)(1)#51 <= 13
 0 <= b_customer(6)(2)(1)#52 <= 17
 0 <= b_customer(6)(3)(1)#53 <= 17
 0 <= b_customer(7)(1)(1)#54 <= 8
 0 <= b_customer(7)(2)(1)#55 <= 8
 0 <= b_customer(7)(3)(1)#56 <= 6
 0 <= b_customer(8)(1)(1)#57 <= 15
 0 <= b_customer(8)(2)(1)#58 <= 20
 0 <= b_customer(8)(3)(1)#59 <= 19
 0 <= b_customer(9)(1)(1)#60 <= 19
 0 <= b_customer(9)(2)(1)#61 <= 15
 0 <= b_customer(9)(3)(1)#62 <= 17
 0 <= b_customer(10)(1)(1)#63 <= 14
 0 <= b_customer(10)(2)(1)#64 <= 14
 0 <= b_customer(10)(3)(1)#65 <= 16
 0 <= b_customer(11)(1)(1)#66 <= 13
 0 <= b_customer(11)(2)(1)#67 <= 13
 0 <= b_customer(11)(3)(1)#68 <= 14
 0 <= b_customer(12)(1)(1)#69 <= 13
 0 <= b_customer(12)(2)(1)#70 <= 14
 0 <= b_customer(12)(3)(1)#71 <= 16
 0 <= q(1)(1)(1)#72 <= 124
 0 <= q(1)(1)(2)#73 <= 124
 0 <= q(1)(1)(3)#74 <= 124
 0 <= q(2)(1)(1)#75 <= 124
 0 <= q(2)(1)(2)#76 <= 124
 0 <= q(2)(1)(3)#77 <= 124
 0 <= w(3)(1)(1)(1)#78 <= 18
 0 <= w(3)(1)(2)(1)#79 <= 17
 0 <= w(3)(1)(3)(1)#80 <= 22
 0 <= w(3)(2)(1)(1)#81 <= 18
 0 <= w(3)(2)(2)(1)#82 <= 17
 0 <= w(3)(2)(3)(1)#83 <= 22
 0 <= w(4)(1)(1)(1)#84 <= 8
 0 <= w(4)(1)(2)(1)#85 <= 6
 0 <= w(4)(1)(3)(1)#86 <= 8
 0 <= w(4)(2)(1)(1)#87 <= 8
 0 <= w(4)(2)(2)(1)#88 <= 6
 0 <= w(4)(2)(3)(1)#89 <= 8
 0 <= w(5)(1)(1)(1)#90 <= 5
 0 <= w(5)(1)(2)(1)#91 <= 6
 0 <= w(5)(1)(3)(1)#92 <= 6
 0 <= w(5)(2)(1)(1)#93 <= 5
 0 <= w(5)(2)(2)(1)#94 <= 6
 0 <= w(5)(2)(3)(1)#95 <= 6
 0 <= w(6)(1)(1)(1)#96 <= 13
 0 <= w(6)(1)(2)(1)#97 <= 17
 0 <= w(6)(1)(3)(1)#98 <= 17
 0 <= w(6)(2)(1)(1)#99 <= 13
 0 <= w(6)(2)(2)(1)#100 <= 17
 0 <= w(6)(2)(3)(1)#101 <= 17
 0 <= w(7)(1)(1)(1)#102 <= 8
 0 <= w(7)(1)(2)(1)#103 <= 8
 0 <= w(7)(1)(3)(1)#104 <= 6
 0 <= w(7)(2)(1)(1)#105 <= 8
 0 <= w(7)(2)(2)(1)#106 <= 8
 0 <= w(7)(2)(3)(1)#107 <= 6
 0 <= w(8)(1)(1)(1)#108 <= 15
 0 <= w(8)(1)(2)(1)#109 <= 20
 0 <= w(8)(1)(3)(1)#110 <= 19
 0 <= w(8)(2)(1)(1)#111 <= 15
 0 <= w(8)(2)(2)(1)#112 <= 20
 0 <= w(8)(2)(3)(1)#113 <= 19
 0 <= w(9)(1)(1)(1)#114 <= 19
 0 <= w(9)(1)(2)(1)#115 <= 15
 0 <= w(9)(1)(3)(1)#116 <= 17
 0 <= w(9)(2)(1)(1)#117 <= 19
 0 <= w(9)(2)(2)(1)#118 <= 15
 0 <= w(9)(2)(3)(1)#119 <= 17
 0 <= w(10)(1)(1)(1)#120 <= 14
 0 <= w(10)(1)(2)(1)#121 <= 14
 0 <= w(10)(1)(3)(1)#122 <= 16
 0 <= w(10)(2)(1)(1)#123 <= 14
 0 <= w(10)(2)(2)(1)#124 <= 14
 0 <= w(10)(2)(3)(1)#125 <= 16
 0 <= w(11)(1)(1)(1)#126 <= 13
 0 <= w(11)(1)(2)(1)#127 <= 13
 0 <= w(11)(1)(3)(1)#128 <= 14
 0 <= w(11)(2)(1)(1)#129 <= 13
 0 <= w(11)(2)(2)(1)#130 <= 13
 0 <= w(11)(2)(3)(1)#131 <= 14
 0 <= w(12)(1)(1)(1)#132 <= 13
 0 <= w(12)(1)(2)(1)#133 <= 14
 0 <= w(12)(1)(3)(1)#134 <= 16
 0 <= w(12)(2)(1)(1)#135 <= 13
 0 <= w(12)(2)(2)(1)#136 <= 14
 0 <= w(12)(2)(3)(1)#137 <= 16
End
