\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 30 p(1)#0 + 30 p(2)#1 + 30 p(3)#2 + 3 I_plant(1)#3 + 3 I_plant(2)#4
       + 3 I_plant(3)#5 + 5 I_warehouse(1)(1)(78)#6 + 5 I_warehouse(1)(2)(78)#7
       + 5 I_warehouse(1)(3)(78)#8 + 5 I_warehouse(2)(1)(78)#9
       + 5 I_warehouse(2)(2)(78)#10 + 5 I_warehouse(2)(3)(78)#11
       + 9 I_customer(3)(1)(78)#12 + 9 I_customer(3)(2)(78)#13
       + 9 I_customer(3)(3)(78)#14 + 10 I_customer(4)(1)(78)#15
       + 10 I_customer(4)(2)(78)#16 + 10 I_customer(4)(3)(78)#17
       + 10 I_customer(5)(1)(78)#18 + 10 I_customer(5)(2)(78)#19
       + 10 I_customer(5)(3)(78)#20 + 8 I_customer(6)(1)(78)#21
       + 8 I_customer(6)(2)(78)#22 + 8 I_customer(6)(3)(78)#23
       + 8 I_customer(7)(1)(78)#24 + 8 I_customer(7)(2)(78)#25
       + 8 I_customer(7)(3)(78)#26 + 10 I_customer(8)(1)(78)#27
       + 10 I_customer(8)(2)(78)#28 + 10 I_customer(8)(3)(78)#29
       + 10 I_customer(9)(1)(78)#30 + 10 I_customer(9)(2)(78)#31
       + 10 I_customer(9)(3)(78)#32 + 7 I_customer(10)(1)(78)#33
       + 7 I_customer(10)(2)(78)#34 + 7 I_customer(10)(3)(78)#35
       + 9 I_customer(11)(1)(78)#36 + 9 I_customer(11)(2)(78)#37
       + 9 I_customer(11)(3)(78)#38 + 7 I_customer(12)(1)(78)#39
       + 7 I_customer(12)(2)(78)#40 + 7 I_customer(12)(3)(78)#41
       + 470 b_customer(3)(1)(78)#42 + 470 b_customer(3)(2)(78)#43
       + 470 b_customer(3)(3)(78)#44 + 460 b_customer(4)(1)(78)#45
       + 460 b_customer(4)(2)(78)#46 + 460 b_customer(4)(3)(78)#47
       + 480 b_customer(5)(1)(78)#48 + 480 b_customer(5)(2)(78)#49
       + 480 b_customer(5)(3)(78)#50 + 470 b_customer(6)(1)(78)#51
       + 470 b_customer(6)(2)(78)#52 + 470 b_customer(6)(3)(78)#53
       + 480 b_customer(7)(1)(78)#54 + 480 b_customer(7)(2)(78)#55
       + 480 b_customer(7)(3)(78)#56 + 470 b_customer(8)(1)(78)#57
       + 470 b_customer(8)(2)(78)#58 + 470 b_customer(8)(3)(78)#59
       + 470 b_customer(9)(1)(78)#60 + 470 b_customer(9)(2)(78)#61
       + 470 b_customer(9)(3)(78)#62 + 460 b_customer(10)(1)(78)#63
       + 460 b_customer(10)(2)(78)#64 + 460 b_customer(10)(3)(78)#65
       + 460 b_customer(11)(1)(78)#66 + 460 b_customer(11)(2)(78)#67
       + 460 b_customer(11)(3)(78)#68 + 460 b_customer(12)(1)(78)#69
       + 460 b_customer(12)(2)(78)#70 + 460 b_customer(12)(3)(78)#71
Subject To
 c1:                             p(1)#0  = 270
 c2:                             p(2)#1  = 0
 c3:                             p(3)#2  = 0
 c4:                             I_plant(1)#3  = 40
 c5:                             I_plant(2)#4  = 40
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
 WarehouseInventoryCapacity(1,1,78): I_warehouse(1)(1)(78)#6 <= 124
 WarehouseInventoryCapacity(2,1,78): I_warehouse(2)(1)(78)#9 <= 124
 WarehouseInventoryCapacity(1,2,78): I_warehouse(1)(2)(78)#7 <= 124
 WarehouseInventoryCapacity(2,2,78): I_warehouse(2)(2)(78)#10 <= 124
 WarehouseInventoryCapacity(1,3,78): I_warehouse(1)(3)(78)#8 <= 124
 WarehouseInventoryCapacity(2,3,78): I_warehouse(2)(3)(78)#11 <= 124
 WarehouseInventoryBalance(1,1,78): I_warehouse(1)(1)(78)#6 - q(1)(1)(1)#72
                                 + w(3)(1)(1)(78)#78 + w(4)(1)(1)(78)#84
                                 + w(5)(1)(1)(78)#90 + w(6)(1)(1)(78)#96
                                 + w(7)(1)(1)(78)#102 + w(8)(1)(1)(78)#108
                                 + w(9)(1)(1)(78)#114 + w(10)(1)(1)(78)#120
                                 + w(11)(1)(1)(78)#126 + w(12)(1)(1)(78)#132
                                  = 0
 WarehouseInventoryBalance(2,1,78): I_warehouse(2)(1)(78)#9 - q(2)(1)(1)#75
                                 + w(3)(2)(1)(78)#81 + w(4)(2)(1)(78)#87
                                 + w(5)(2)(1)(78)#93 + w(6)(2)(1)(78)#99
                                 + w(7)(2)(1)(78)#105 + w(8)(2)(1)(78)#111
                                 + w(9)(2)(1)(78)#117 + w(10)(2)(1)(78)#123
                                 + w(11)(2)(1)(78)#129 + w(12)(2)(1)(78)#135
                                  = 0
 WarehouseInventoryBalance(1,2,78): - I_warehouse(1)(1)(78)#6
                                 + I_warehouse(1)(2)(78)#7 - q(1)(1)(2)#73
                                 + w(3)(1)(2)(78)#79 + w(4)(1)(2)(78)#85
                                 + w(5)(1)(2)(78)#91 + w(6)(1)(2)(78)#97
                                 + w(7)(1)(2)(78)#103 + w(8)(1)(2)(78)#109
                                 + w(9)(1)(2)(78)#115 + w(10)(1)(2)(78)#121
                                 + w(11)(1)(2)(78)#127 + w(12)(1)(2)(78)#133
                                  = 0
 WarehouseInventoryBalance(2,2,78): - I_warehouse(2)(1)(78)#9
                                 + I_warehouse(2)(2)(78)#10 - q(2)(1)(2)#76
                                 + w(3)(2)(2)(78)#82 + w(4)(2)(2)(78)#88
                                 + w(5)(2)(2)(78)#94 + w(6)(2)(2)(78)#100
                                 + w(7)(2)(2)(78)#106 + w(8)(2)(2)(78)#112
                                 + w(9)(2)(2)(78)#118 + w(10)(2)(2)(78)#124
                                 + w(11)(2)(2)(78)#130 + w(12)(2)(2)(78)#136
                                  = 0
 WarehouseInventoryBalance(1,3,78): - I_warehouse(1)(2)(78)#7
                                 + I_warehouse(1)(3)(78)#8 - q(1)(1)(3)#74
                                 + w(3)(1)(3)(78)#80 + w(4)(1)(3)(78)#86
                                 + w(5)(1)(3)(78)#92 + w(6)(1)(3)(78)#98
                                 + w(7)(1)(3)(78)#104 + w(8)(1)(3)(78)#110
                                 + w(9)(1)(3)(78)#116 + w(10)(1)(3)(78)#122
                                 + w(11)(1)(3)(78)#128 + w(12)(1)(3)(78)#134
                                  = 0
 WarehouseInventoryBalance(2,3,78): - I_warehouse(2)(2)(78)#10
                                 + I_warehouse(2)(3)(78)#11 - q(2)(1)(3)#77
                                 + w(3)(2)(3)(78)#83 + w(4)(2)(3)(78)#89
                                 + w(5)(2)(3)(78)#95 + w(6)(2)(3)(78)#101
                                 + w(7)(2)(3)(78)#107 + w(8)(2)(3)(78)#113
                                 + w(9)(2)(3)(78)#119 + w(10)(2)(3)(78)#125
                                 + w(11)(2)(3)(78)#131 + w(12)(2)(3)(78)#137
                                  = 0
 CustomerInventoryBalance(3,1,78): I_customer(3)(1)(78)#12
                                 - b_customer(3)(1)(78)#42 - w(3)(1)(1)(78)#78
                                 - w(3)(2)(1)(78)#81  = 22
 CustomerInventoryBalance(4,1,78): I_customer(4)(1)(78)#15
                                 - b_customer(4)(1)(78)#45 - w(4)(1)(1)(78)#84
                                 - w(4)(2)(1)(78)#87  = 0
 CustomerInventoryBalance(5,1,78): I_customer(5)(1)(78)#18
                                 - b_customer(5)(1)(78)#48 - w(5)(1)(1)(78)#90
                                 - w(5)(2)(1)(78)#93  = -1
 CustomerInventoryBalance(6,1,78): I_customer(6)(1)(78)#21
                                 - b_customer(6)(1)(78)#51 - w(6)(1)(1)(78)#96
                                 - w(6)(2)(1)(78)#99  = 2
 CustomerInventoryBalance(7,1,78): I_customer(7)(1)(78)#24
                                 - b_customer(7)(1)(78)#54 - w(7)(1)(1)(78)#102
                                 - w(7)(2)(1)(78)#105  = 6
 CustomerInventoryBalance(8,1,78): I_customer(8)(1)(78)#27
                                 - b_customer(8)(1)(78)#57 - w(8)(1)(1)(78)#108
                                 - w(8)(2)(1)(78)#111  = 3
 CustomerInventoryBalance(9,1,78): I_customer(9)(1)(78)#30
                                 - b_customer(9)(1)(78)#60 - w(9)(1)(1)(78)#114
                                 - w(9)(2)(1)(78)#117  = 19
 CustomerInventoryBalance(10,1,78): I_customer(10)(1)(78)#33
                                 - b_customer(10)(1)(78)#63
                                 - w(10)(1)(1)(78)#120 - w(10)(2)(1)(78)#123
                                  = -2
 CustomerInventoryBalance(11,1,78): I_customer(11)(1)(78)#36
                                 - b_customer(11)(1)(78)#66
                                 - w(11)(1)(1)(78)#126 - w(11)(2)(1)(78)#129
                                  = -1
 CustomerInventoryBalance(12,1,78): I_customer(12)(1)(78)#39
                                 - b_customer(12)(1)(78)#69
                                 - w(12)(1)(1)(78)#132 - w(12)(2)(1)(78)#135
                                  = 17
 CustomerInventoryBalance(3,2,78): - I_customer(3)(1)(78)#12
                                 + I_customer(3)(2)(78)#13
                                 - b_customer(3)(2)(78)#43 - w(3)(1)(2)(78)#79
                                 - w(3)(2)(2)(78)#82  = -15
 CustomerInventoryBalance(4,2,78): - I_customer(4)(1)(78)#15
                                 + I_customer(4)(2)(78)#16
                                 - b_customer(4)(2)(78)#46 - w(4)(1)(2)(78)#85
                                 - w(4)(2)(2)(78)#88  = -6
 CustomerInventoryBalance(5,2,78): - I_customer(5)(1)(78)#18
                                 + I_customer(5)(2)(78)#19
                                 - b_customer(5)(2)(78)#49 - w(5)(1)(2)(78)#91
                                 - w(5)(2)(2)(78)#94  = -7
 CustomerInventoryBalance(6,2,78): - I_customer(6)(1)(78)#21
                                 + I_customer(6)(2)(78)#22
                                 - b_customer(6)(2)(78)#52 - w(6)(1)(2)(78)#97
                                 - w(6)(2)(2)(78)#100  = -19
 CustomerInventoryBalance(7,2,78): - I_customer(7)(1)(78)#24
                                 + I_customer(7)(2)(78)#25
                                 - b_customer(7)(2)(78)#55 - w(7)(1)(2)(78)#103
                                 - w(7)(2)(2)(78)#106  = -7
 CustomerInventoryBalance(8,2,78): - I_customer(8)(1)(78)#27
                                 + I_customer(8)(2)(78)#28
                                 - b_customer(8)(2)(78)#58 - w(8)(1)(2)(78)#109
                                 - w(8)(2)(2)(78)#112  = -15
 CustomerInventoryBalance(9,2,78): - I_customer(9)(1)(78)#30
                                 + I_customer(9)(2)(78)#31
                                 - b_customer(9)(2)(78)#61 - w(9)(1)(2)(78)#115
                                 - w(9)(2)(2)(78)#118  = -15
 CustomerInventoryBalance(10,2,78): - I_customer(10)(1)(78)#33
                                 + I_customer(10)(2)(78)#34
                                 - b_customer(10)(2)(78)#64
                                 - w(10)(1)(2)(78)#121 - w(10)(2)(2)(78)#124
                                  = -13
 CustomerInventoryBalance(11,2,78): - I_customer(11)(1)(78)#36
                                 + I_customer(11)(2)(78)#37
                                 - b_customer(11)(2)(78)#67
                                 - w(11)(1)(2)(78)#127 - w(11)(2)(2)(78)#130
                                  = -19
 CustomerInventoryBalance(12,2,78): - I_customer(12)(1)(78)#39
                                 + I_customer(12)(2)(78)#40
                                 - b_customer(12)(2)(78)#70
                                 - w(12)(1)(2)(78)#133 - w(12)(2)(2)(78)#136
                                  = -14
 CustomerInventoryBalance(3,3,78): - I_customer(3)(2)(78)#13
                                 + I_customer(3)(3)(78)#14
                                 - b_customer(3)(3)(78)#44 - w(3)(1)(3)(78)#80
                                 - w(3)(2)(3)(78)#83  = -19
 CustomerInventoryBalance(4,3,78): - I_customer(4)(2)(78)#16
                                 + I_customer(4)(3)(78)#17
                                 - b_customer(4)(3)(78)#47 - w(4)(1)(3)(78)#86
                                 - w(4)(2)(3)(78)#89  = -7
 CustomerInventoryBalance(5,3,78): - I_customer(5)(2)(78)#19
                                 + I_customer(5)(3)(78)#20
                                 - b_customer(5)(3)(78)#50 - w(5)(1)(3)(78)#92
                                 - w(5)(2)(3)(78)#95  = -5
 CustomerInventoryBalance(6,3,78): - I_customer(6)(2)(78)#22
                                 + I_customer(6)(3)(78)#23
                                 - b_customer(6)(3)(78)#53 - w(6)(1)(3)(78)#98
                                 - w(6)(2)(3)(78)#101  = -16
 CustomerInventoryBalance(7,3,78): - I_customer(7)(2)(78)#25
                                 + I_customer(7)(3)(78)#26
                                 - b_customer(7)(3)(78)#56 - w(7)(1)(3)(78)#104
                                 - w(7)(2)(3)(78)#107  = -8
 CustomerInventoryBalance(8,3,78): - I_customer(8)(2)(78)#28
                                 + I_customer(8)(3)(78)#29
                                 - b_customer(8)(3)(78)#59 - w(8)(1)(3)(78)#110
                                 - w(8)(2)(3)(78)#113  = -18
 CustomerInventoryBalance(9,3,78): - I_customer(9)(2)(78)#31
                                 + I_customer(9)(3)(78)#32
                                 - b_customer(9)(3)(78)#62 - w(9)(1)(3)(78)#116
                                 - w(9)(2)(3)(78)#119  = -13
 CustomerInventoryBalance(10,3,78): - I_customer(10)(2)(78)#34
                                 + I_customer(10)(3)(78)#35
                                 - b_customer(10)(3)(78)#65
                                 - w(10)(1)(3)(78)#122 - w(10)(2)(3)(78)#125
                                  = -16
 CustomerInventoryBalance(11,3,78): - I_customer(11)(2)(78)#37
                                 + I_customer(11)(3)(78)#38
                                 - b_customer(11)(3)(78)#68
                                 - w(11)(1)(3)(78)#128 - w(11)(2)(3)(78)#131
                                  = -16
 CustomerInventoryBalance(12,3,78): - I_customer(12)(2)(78)#40
                                 + I_customer(12)(3)(78)#41
                                 - b_customer(12)(3)(78)#71
                                 - w(12)(1)(3)(78)#134 - w(12)(2)(3)(78)#137
                                  = -15
 CustomerInventoryCapacity(3,1,78): I_customer(3)(1)(78)#12 <= 60
 CustomerInventoryCapacity(4,1,78): I_customer(4)(1)(78)#15 <= 21
 CustomerInventoryCapacity(5,1,78): I_customer(5)(1)(78)#18 <= 17
 CustomerInventoryCapacity(6,1,78): I_customer(6)(1)(78)#21 <= 50
 CustomerInventoryCapacity(7,1,78): I_customer(7)(1)(78)#24 <= 20
 CustomerInventoryCapacity(8,1,78): I_customer(8)(1)(78)#27 <= 57
 CustomerInventoryCapacity(9,1,78): I_customer(9)(1)(78)#30 <= 51
 CustomerInventoryCapacity(10,1,78): I_customer(10)(1)(78)#33 <= 46
 CustomerInventoryCapacity(11,1,78): I_customer(11)(1)(78)#36 <= 47
 CustomerInventoryCapacity(12,1,78): I_customer(12)(1)(78)#39 <= 45
 CustomerInventoryCapacity(3,2,78): I_customer(3)(2)(78)#13 <= 61
 CustomerInventoryCapacity(4,2,78): I_customer(4)(2)(78)#16 <= 22
 CustomerInventoryCapacity(5,2,78): I_customer(5)(2)(78)#19 <= 17
 CustomerInventoryCapacity(6,2,78): I_customer(6)(2)(78)#22 <= 45
 CustomerInventoryCapacity(7,2,78): I_customer(7)(2)(78)#25 <= 21
 CustomerInventoryCapacity(8,2,78): I_customer(8)(2)(78)#28 <= 57
 CustomerInventoryCapacity(9,2,78): I_customer(9)(2)(78)#31 <= 49
 CustomerInventoryCapacity(10,2,78): I_customer(10)(2)(78)#34 <= 51
 CustomerInventoryCapacity(11,2,78): I_customer(11)(2)(78)#37 <= 45
 CustomerInventoryCapacity(12,2,78): I_customer(12)(2)(78)#40 <= 42
 CustomerInventoryCapacity(3,3,78): I_customer(3)(3)(78)#14 <= 57
 CustomerInventoryCapacity(4,3,78): I_customer(4)(3)(78)#17 <= 21
 CustomerInventoryCapacity(5,3,78): I_customer(5)(3)(78)#20 <= 19
 CustomerInventoryCapacity(6,3,78): I_customer(6)(3)(78)#23 <= 48
 CustomerInventoryCapacity(7,3,78): I_customer(7)(3)(78)#26 <= 20
 CustomerInventoryCapacity(8,3,78): I_customer(8)(3)(78)#29 <= 54
 CustomerInventoryCapacity(9,3,78): I_customer(9)(3)(78)#32 <= 51
 CustomerInventoryCapacity(10,3,78): I_customer(10)(3)(78)#35 <= 48
 CustomerInventoryCapacity(11,3,78): I_customer(11)(3)(78)#38 <= 48
 CustomerInventoryCapacity(12,3,78): I_customer(12)(3)(78)#41 <= 41
 WarehouseDelivery(1,1,1):       q(1)(1)(1)#72 <= 124
 WarehouseDelivery(2,1,1):       q(2)(1)(1)#75 <= 124
 WarehouseDelivery(1,1,2):       q(1)(1)(2)#73  = 0
 WarehouseDelivery(2,1,2):       q(2)(1)(2)#76  = 0
 WarehouseDelivery(1,1,3):       q(1)(1)(3)#74 <= 124
 WarehouseDelivery(2,1,3):       q(2)(1)(3)#77  = 0
 VehicleCapacityPlant(1,1):      q(1)(1)(1)#72 + q(2)(1)(1)#75 <= 570
 VehicleCapacityPlant(1,2):      q(1)(1)(2)#73 + q(2)(1)(2)#76 <= 570
 VehicleCapacityPlant(1,3):      q(1)(1)(3)#74 + q(2)(1)(3)#77 <= 570
 CustomerVisit(3,1,1,78):        w(3)(1)(1)(78)#78  = 0
 CustomerVisit(4,1,1,78):        w(4)(1)(1)(78)#84  = 0
 CustomerVisit(5,1,1,78):        w(5)(1)(1)(78)#90  = 0
 CustomerVisit(6,1,1,78):        w(6)(1)(1)(78)#96  = 0
 CustomerVisit(7,1,1,78):        w(7)(1)(1)(78)#102  = 0
 CustomerVisit(8,1,1,78):        w(8)(1)(1)(78)#108  = 0
 CustomerVisit(9,1,1,78):        w(9)(1)(1)(78)#114  = 0
 CustomerVisit(11,1,1,78):       w(11)(1)(1)(78)#126  = 0
 CustomerVisit(12,1,1,78):       w(12)(1)(1)(78)#132  = 0
 CustomerVisit(3,1,2,78):        w(3)(1)(2)(78)#79  = 0
 CustomerVisit(4,1,2,78):        w(4)(1)(2)(78)#85  = 0
 CustomerVisit(5,1,2,78):        w(5)(1)(2)(78)#91  = 0
 CustomerVisit(6,1,2,78):        w(6)(1)(2)(78)#97  = 0
 CustomerVisit(7,1,2,78):        w(7)(1)(2)(78)#103  = 0
 CustomerVisit(8,1,2,78):        w(8)(1)(2)(78)#109  = 0
 CustomerVisit(9,1,2,78):        w(9)(1)(2)(78)#115  = 0
 CustomerVisit(10,1,2,78):       w(10)(1)(2)(78)#121  = 0
 CustomerVisit(11,1,2,78):       w(11)(1)(2)(78)#127  = 0
 CustomerVisit(12,1,2,78):       w(12)(1)(2)(78)#133  = 0
 CustomerVisit(3,1,3,78):        w(3)(1)(3)(78)#80  = 0
 CustomerVisit(4,1,3,78):        w(4)(1)(3)(78)#86  = 0
 CustomerVisit(5,1,3,78):        w(5)(1)(3)(78)#92  = 0
 CustomerVisit(6,1,3,78):        w(6)(1)(3)(78)#98  = 0
 CustomerVisit(7,1,3,78):        w(7)(1)(3)(78)#104  = 0
 CustomerVisit(8,1,3,78):        w(8)(1)(3)(78)#110  = 0
 CustomerVisit(9,1,3,78):        w(9)(1)(3)(78)#116  = 0
 CustomerVisit(10,1,3,78):       w(10)(1)(3)(78)#122  = 0
 CustomerVisit(11,1,3,78):       w(11)(1)(3)(78)#128  = 0
 CustomerVisit(12,1,3,78):       w(12)(1)(3)(78)#134  = 0
 CustomerVisit(3,2,1,78):        w(3)(2)(1)(78)#81  = 0
 CustomerVisit(9,2,1,78):        w(9)(2)(1)(78)#117  = 0
 CustomerVisit(10,2,1,78):       w(10)(2)(1)(78)#123  = 0
 CustomerVisit(11,2,1,78):       w(11)(2)(1)(78)#129  = 0
 CustomerVisit(12,2,1,78):       w(12)(2)(1)(78)#135  = 0
 CustomerVisit(3,2,3,78):        w(3)(2)(3)(78)#83  = 0
 CustomerVisit(4,2,3,78):        w(4)(2)(3)(78)#89  = 0
 CustomerVisit(5,2,3,78):        w(5)(2)(3)(78)#95  = 0
 CustomerVisit(6,2,3,78):        w(6)(2)(3)(78)#101  = 0
 CustomerVisit(7,2,3,78):        w(7)(2)(3)(78)#107  = 0
 CustomerVisit(8,2,3,78):        w(8)(2)(3)(78)#113  = 0
 CustomerVisit(9,2,3,78):        w(9)(2)(3)(78)#119  = 0
 CustomerVisit(10,2,3,78):       w(10)(2)(3)(78)#125  = 0
 CustomerVisit(11,2,3,78):       w(11)(2)(3)(78)#131  = 0
 CustomerVisit(12,2,3,78):       w(12)(2)(3)(78)#137  = 0
 VehicleCapacityWarehouse(1,1,1,78): w(3)(1)(1)(78)#78 + w(4)(1)(1)(78)#84
                                 + w(5)(1)(1)(78)#90 + w(6)(1)(1)(78)#96
                                 + w(7)(1)(1)(78)#102 + w(8)(1)(1)(78)#108
                                 + w(9)(1)(1)(78)#114 + w(10)(1)(1)(78)#120
                                 + w(11)(1)(1)(78)#126 + w(12)(1)(1)(78)#132
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,78): w(3)(2)(1)(78)#81 + w(4)(2)(1)(78)#87
                                 + w(5)(2)(1)(78)#93 + w(6)(2)(1)(78)#99
                                 + w(7)(2)(1)(78)#105 + w(8)(2)(1)(78)#111
                                 + w(9)(2)(1)(78)#117 + w(10)(2)(1)(78)#123
                                 + w(11)(2)(1)(78)#129 + w(12)(2)(1)(78)#135
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,78): w(3)(1)(2)(78)#79 + w(4)(1)(2)(78)#85
                                 + w(5)(1)(2)(78)#91 + w(6)(1)(2)(78)#97
                                 + w(7)(1)(2)(78)#103 + w(8)(1)(2)(78)#109
                                 + w(9)(1)(2)(78)#115 + w(10)(1)(2)(78)#121
                                 + w(11)(1)(2)(78)#127 + w(12)(1)(2)(78)#133
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,78): w(3)(2)(2)(78)#82 + w(4)(2)(2)(78)#88
                                 + w(5)(2)(2)(78)#94 + w(6)(2)(2)(78)#100
                                 + w(7)(2)(2)(78)#106 + w(8)(2)(2)(78)#112
                                 + w(9)(2)(2)(78)#118 + w(10)(2)(2)(78)#124
                                 + w(11)(2)(2)(78)#130 + w(12)(2)(2)(78)#136
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,78): w(3)(1)(3)(78)#80 + w(4)(1)(3)(78)#86
                                 + w(5)(1)(3)(78)#92 + w(6)(1)(3)(78)#98
                                 + w(7)(1)(3)(78)#104 + w(8)(1)(3)(78)#110
                                 + w(9)(1)(3)(78)#116 + w(10)(1)(3)(78)#122
                                 + w(11)(1)(3)(78)#128 + w(12)(1)(3)(78)#134
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,78): w(3)(2)(3)(78)#83 + w(4)(2)(3)(78)#89
                                 + w(5)(2)(3)(78)#95 + w(6)(2)(3)(78)#101
                                 + w(7)(2)(3)(78)#107 + w(8)(2)(3)(78)#113
                                 + w(9)(2)(3)(78)#119 + w(10)(2)(3)(78)#125
                                 + w(11)(2)(3)(78)#131 + w(12)(2)(3)(78)#137
                                 <= 228
Bounds
 0 <= p(1)#0 <= 270
 0 <= p(2)#1 <= 270
 0 <= p(3)#2 <= 270
 0 <= I_plant(1)#3 <= 135
 0 <= I_plant(2)#4 <= 135
 0 <= I_plant(3)#5 <= 135
 0 <= I_warehouse(1)(1)(78)#6 <= 124
 0 <= I_warehouse(1)(2)(78)#7 <= 124
 0 <= I_warehouse(1)(3)(78)#8 <= 124
 0 <= I_warehouse(2)(1)(78)#9 <= 124
 0 <= I_warehouse(2)(2)(78)#10 <= 124
 0 <= I_warehouse(2)(3)(78)#11 <= 124
 0 <= I_customer(3)(1)(78)#12 <= 76
 0 <= I_customer(3)(2)(78)#13 <= 76
 0 <= I_customer(3)(3)(78)#14 <= 76
 0 <= I_customer(4)(1)(78)#15 <= 28
 0 <= I_customer(4)(2)(78)#16 <= 28
 0 <= I_customer(4)(3)(78)#17 <= 28
 0 <= I_customer(5)(1)(78)#18 <= 24
 0 <= I_customer(5)(2)(78)#19 <= 24
 0 <= I_customer(5)(3)(78)#20 <= 24
 0 <= I_customer(6)(1)(78)#21 <= 64
 0 <= I_customer(6)(2)(78)#22 <= 64
 0 <= I_customer(6)(3)(78)#23 <= 64
 0 <= I_customer(7)(1)(78)#24 <= 28
 0 <= I_customer(7)(2)(78)#25 <= 28
 0 <= I_customer(7)(3)(78)#26 <= 28
 0 <= I_customer(8)(1)(78)#27 <= 72
 0 <= I_customer(8)(2)(78)#28 <= 72
 0 <= I_customer(8)(3)(78)#29 <= 72
 0 <= I_customer(9)(1)(78)#30 <= 64
 0 <= I_customer(9)(2)(78)#31 <= 64
 0 <= I_customer(9)(3)(78)#32 <= 64
 0 <= I_customer(10)(1)(78)#33 <= 64
 0 <= I_customer(10)(2)(78)#34 <= 64
 0 <= I_customer(10)(3)(78)#35 <= 64
 0 <= I_customer(11)(1)(78)#36 <= 64
 0 <= I_customer(11)(2)(78)#37 <= 64
 0 <= I_customer(11)(3)(78)#38 <= 64
 0 <= I_customer(12)(1)(78)#39 <= 56
 0 <= I_customer(12)(2)(78)#40 <= 56
 0 <= I_customer(12)(3)(78)#41 <= 56
 0 <= b_customer(3)(1)(78)#42 <= 16
 0 <= b_customer(3)(2)(78)#43 <= 15
 0 <= b_customer(3)(3)(78)#44 <= 19
 0 <= b_customer(4)(1)(78)#45 <= 7
 0 <= b_customer(4)(2)(78)#46 <= 6
 0 <= b_customer(4)(3)(78)#47 <= 7
 0 <= b_customer(5)(1)(78)#48 <= 7
 0 <= b_customer(5)(2)(78)#49 <= 7
 0 <= b_customer(5)(3)(78)#50 <= 5
 0 <= b_customer(6)(1)(78)#51 <= 14
 0 <= b_customer(6)(2)(78)#52 <= 19
 0 <= b_customer(6)(3)(78)#53 <= 16
 0 <= b_customer(7)(1)(78)#54 <= 8
 0 <= b_customer(7)(2)(78)#55 <= 7
 0 <= b_customer(7)(3)(78)#56 <= 8
 0 <= b_customer(8)(1)(78)#57 <= 15
 0 <= b_customer(8)(2)(78)#58 <= 15
 0 <= b_customer(8)(3)(78)#59 <= 18
 0 <= b_customer(9)(1)(78)#60 <= 13
 0 <= b_customer(9)(2)(78)#61 <= 15
 0 <= b_customer(9)(3)(78)#62 <= 13
 0 <= b_customer(10)(1)(78)#63 <= 18
 0 <= b_customer(10)(2)(78)#64 <= 13
 0 <= b_customer(10)(3)(78)#65 <= 16
 0 <= b_customer(11)(1)(78)#66 <= 17
 0 <= b_customer(11)(2)(78)#67 <= 19
 0 <= b_customer(11)(3)(78)#68 <= 16
 0 <= b_customer(12)(1)(78)#69 <= 11
 0 <= b_customer(12)(2)(78)#70 <= 14
 0 <= b_customer(12)(3)(78)#71 <= 15
 0 <= q(1)(1)(1)#72 <= 124
 0 <= q(1)(1)(2)#73 <= 124
 0 <= q(1)(1)(3)#74 <= 124
 0 <= q(2)(1)(1)#75 <= 124
 0 <= q(2)(1)(2)#76 <= 124
 0 <= q(2)(1)(3)#77 <= 124
 0 <= w(3)(1)(1)(78)#78 <= 16
 0 <= w(3)(1)(2)(78)#79 <= 15
 0 <= w(3)(1)(3)(78)#80 <= 19
 0 <= w(3)(2)(1)(78)#81 <= 16
 0 <= w(3)(2)(2)(78)#82 <= 15
 0 <= w(3)(2)(3)(78)#83 <= 19
 0 <= w(4)(1)(1)(78)#84 <= 7
 0 <= w(4)(1)(2)(78)#85 <= 6
 0 <= w(4)(1)(3)(78)#86 <= 7
 0 <= w(4)(2)(1)(78)#87 <= 7
 0 <= w(4)(2)(2)(78)#88 <= 6
 0 <= w(4)(2)(3)(78)#89 <= 7
 0 <= w(5)(1)(1)(78)#90 <= 7
 0 <= w(5)(1)(2)(78)#91 <= 7
 0 <= w(5)(1)(3)(78)#92 <= 5
 0 <= w(5)(2)(1)(78)#93 <= 7
 0 <= w(5)(2)(2)(78)#94 <= 7
 0 <= w(5)(2)(3)(78)#95 <= 5
 0 <= w(6)(1)(1)(78)#96 <= 14
 0 <= w(6)(1)(2)(78)#97 <= 19
 0 <= w(6)(1)(3)(78)#98 <= 16
 0 <= w(6)(2)(1)(78)#99 <= 14
 0 <= w(6)(2)(2)(78)#100 <= 19
 0 <= w(6)(2)(3)(78)#101 <= 16
 0 <= w(7)(1)(1)(78)#102 <= 8
 0 <= w(7)(1)(2)(78)#103 <= 7
 0 <= w(7)(1)(3)(78)#104 <= 8
 0 <= w(7)(2)(1)(78)#105 <= 8
 0 <= w(7)(2)(2)(78)#106 <= 7
 0 <= w(7)(2)(3)(78)#107 <= 8
 0 <= w(8)(1)(1)(78)#108 <= 15
 0 <= w(8)(1)(2)(78)#109 <= 15
 0 <= w(8)(1)(3)(78)#110 <= 18
 0 <= w(8)(2)(1)(78)#111 <= 15
 0 <= w(8)(2)(2)(78)#112 <= 15
 0 <= w(8)(2)(3)(78)#113 <= 18
 0 <= w(9)(1)(1)(78)#114 <= 13
 0 <= w(9)(1)(2)(78)#115 <= 15
 0 <= w(9)(1)(3)(78)#116 <= 13
 0 <= w(9)(2)(1)(78)#117 <= 13
 0 <= w(9)(2)(2)(78)#118 <= 15
 0 <= w(9)(2)(3)(78)#119 <= 13
 0 <= w(10)(1)(1)(78)#120 <= 18
 0 <= w(10)(1)(2)(78)#121 <= 13
 0 <= w(10)(1)(3)(78)#122 <= 16
 0 <= w(10)(2)(1)(78)#123 <= 18
 0 <= w(10)(2)(2)(78)#124 <= 13
 0 <= w(10)(2)(3)(78)#125 <= 16
 0 <= w(11)(1)(1)(78)#126 <= 17
 0 <= w(11)(1)(2)(78)#127 <= 19
 0 <= w(11)(1)(3)(78)#128 <= 16
 0 <= w(11)(2)(1)(78)#129 <= 17
 0 <= w(11)(2)(2)(78)#130 <= 19
 0 <= w(11)(2)(3)(78)#131 <= 16
 0 <= w(12)(1)(1)(78)#132 <= 11
 0 <= w(12)(1)(2)(78)#133 <= 14
 0 <= w(12)(1)(3)(78)#134 <= 15
 0 <= w(12)(2)(1)(78)#135 <= 11
 0 <= w(12)(2)(2)(78)#136 <= 14
 0 <= w(12)(2)(3)(78)#137 <= 15
End
