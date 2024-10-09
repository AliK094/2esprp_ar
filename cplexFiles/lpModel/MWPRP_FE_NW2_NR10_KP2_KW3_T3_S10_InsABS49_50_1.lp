\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 3000 y(0)#0 + 3000 y(1)#1 + 3000 y(2)#2 + 30 p(0)#3 + 30 p(1)#4
       + 30 p(2)#5 + 3 I_plant(0)#6 + 3 I_plant(1)#7 + 3 I_plant(2)#8
       + 0.4 I_warehouse(0)(0)(0)#9 + 0.4 I_warehouse(0)(0)(1)#10
       + 0.4 I_warehouse(0)(0)(2)#11 + 0.4 I_warehouse(0)(0)(3)#12
       + 0.4 I_warehouse(0)(0)(4)#13 + 0.4 I_warehouse(0)(0)(5)#14
       + 0.4 I_warehouse(0)(0)(6)#15 + 0.4 I_warehouse(0)(0)(7)#16
       + 0.4 I_warehouse(0)(0)(8)#17 + 0.4 I_warehouse(0)(0)(9)#18
       + 0.4 I_warehouse(0)(1)(0)#19 + 0.4 I_warehouse(0)(1)(1)#20
       + 0.4 I_warehouse(0)(1)(2)#21 + 0.4 I_warehouse(0)(1)(3)#22
       + 0.4 I_warehouse(0)(1)(4)#23 + 0.4 I_warehouse(0)(1)(5)#24
       + 0.4 I_warehouse(0)(1)(6)#25 + 0.4 I_warehouse(0)(1)(7)#26
       + 0.4 I_warehouse(0)(1)(8)#27 + 0.4 I_warehouse(0)(1)(9)#28
       + 0.4 I_warehouse(0)(2)(0)#29 + 0.4 I_warehouse(0)(2)(1)#30
       + 0.4 I_warehouse(0)(2)(2)#31 + 0.4 I_warehouse(0)(2)(3)#32
       + 0.4 I_warehouse(0)(2)(4)#33 + 0.4 I_warehouse(0)(2)(5)#34
       + 0.4 I_warehouse(0)(2)(6)#35 + 0.4 I_warehouse(0)(2)(7)#36
       + 0.4 I_warehouse(0)(2)(8)#37 + 0.4 I_warehouse(0)(2)(9)#38
       + 0.3 I_warehouse(1)(0)(0)#39 + 0.3 I_warehouse(1)(0)(1)#40
       + 0.3 I_warehouse(1)(0)(2)#41 + 0.3 I_warehouse(1)(0)(3)#42
       + 0.3 I_warehouse(1)(0)(4)#43 + 0.3 I_warehouse(1)(0)(5)#44
       + 0.3 I_warehouse(1)(0)(6)#45 + 0.3 I_warehouse(1)(0)(7)#46
       + 0.3 I_warehouse(1)(0)(8)#47 + 0.3 I_warehouse(1)(0)(9)#48
       + 0.3 I_warehouse(1)(1)(0)#49 + 0.3 I_warehouse(1)(1)(1)#50
       + 0.3 I_warehouse(1)(1)(2)#51 + 0.3 I_warehouse(1)(1)(3)#52
       + 0.3 I_warehouse(1)(1)(4)#53 + 0.3 I_warehouse(1)(1)(5)#54
       + 0.3 I_warehouse(1)(1)(6)#55 + 0.3 I_warehouse(1)(1)(7)#56
       + 0.3 I_warehouse(1)(1)(8)#57 + 0.3 I_warehouse(1)(1)(9)#58
       + 0.3 I_warehouse(1)(2)(0)#59 + 0.3 I_warehouse(1)(2)(1)#60
       + 0.3 I_warehouse(1)(2)(2)#61 + 0.3 I_warehouse(1)(2)(3)#62
       + 0.3 I_warehouse(1)(2)(4)#63 + 0.3 I_warehouse(1)(2)(5)#64
       + 0.3 I_warehouse(1)(2)(6)#65 + 0.3 I_warehouse(1)(2)(7)#66
       + 0.3 I_warehouse(1)(2)(8)#67 + 0.3 I_warehouse(1)(2)(9)#68
       + 390.666666666667 b_warehouse(0)(0)(0)#69
       + 390.666666666667 b_warehouse(0)(0)(1)#70
       + 390.666666666667 b_warehouse(0)(0)(2)#71
       + 390.666666666667 b_warehouse(0)(0)(3)#72
       + 390.666666666667 b_warehouse(0)(0)(4)#73
       + 390.666666666667 b_warehouse(0)(0)(5)#74
       + 390.666666666667 b_warehouse(0)(0)(6)#75
       + 390.666666666667 b_warehouse(0)(0)(7)#76
       + 390.666666666667 b_warehouse(0)(0)(8)#77
       + 390.666666666667 b_warehouse(0)(0)(9)#78
       + 390.666666666667 b_warehouse(0)(1)(0)#79
       + 390.666666666667 b_warehouse(0)(1)(1)#80
       + 390.666666666667 b_warehouse(0)(1)(2)#81
       + 390.666666666667 b_warehouse(0)(1)(3)#82
       + 390.666666666667 b_warehouse(0)(1)(4)#83
       + 390.666666666667 b_warehouse(0)(1)(5)#84
       + 390.666666666667 b_warehouse(0)(1)(6)#85
       + 390.666666666667 b_warehouse(0)(1)(7)#86
       + 390.666666666667 b_warehouse(0)(1)(8)#87
       + 390.666666666667 b_warehouse(0)(1)(9)#88
       + 390.666666666667 b_warehouse(0)(2)(0)#89
       + 390.666666666667 b_warehouse(0)(2)(1)#90
       + 390.666666666667 b_warehouse(0)(2)(2)#91
       + 390.666666666667 b_warehouse(0)(2)(3)#92
       + 390.666666666667 b_warehouse(0)(2)(4)#93
       + 390.666666666667 b_warehouse(0)(2)(5)#94
       + 390.666666666667 b_warehouse(0)(2)(6)#95
       + 390.666666666667 b_warehouse(0)(2)(7)#96
       + 390.666666666667 b_warehouse(0)(2)(8)#97
       + 390.666666666667 b_warehouse(0)(2)(9)#98
       + 272.25 b_warehouse(1)(0)(0)#99 + 272.25 b_warehouse(1)(0)(1)#100
       + 272.25 b_warehouse(1)(0)(2)#101 + 272.25 b_warehouse(1)(0)(3)#102
       + 272.25 b_warehouse(1)(0)(4)#103 + 272.25 b_warehouse(1)(0)(5)#104
       + 272.25 b_warehouse(1)(0)(6)#105 + 272.25 b_warehouse(1)(0)(7)#106
       + 272.25 b_warehouse(1)(0)(8)#107 + 272.25 b_warehouse(1)(0)(9)#108
       + 272.25 b_warehouse(1)(1)(0)#109 + 272.25 b_warehouse(1)(1)(1)#110
       + 272.25 b_warehouse(1)(1)(2)#111 + 272.25 b_warehouse(1)(1)(3)#112
       + 272.25 b_warehouse(1)(1)(4)#113 + 272.25 b_warehouse(1)(1)(5)#114
       + 272.25 b_warehouse(1)(1)(6)#115 + 272.25 b_warehouse(1)(1)(7)#116
       + 272.25 b_warehouse(1)(1)(8)#117 + 272.25 b_warehouse(1)(1)(9)#118
       + 272.25 b_warehouse(1)(2)(0)#119 + 272.25 b_warehouse(1)(2)(1)#120
       + 272.25 b_warehouse(1)(2)(2)#121 + 272.25 b_warehouse(1)(2)(3)#122
       + 272.25 b_warehouse(1)(2)(4)#123 + 272.25 b_warehouse(1)(2)(5)#124
       + 272.25 b_warehouse(1)(2)(6)#125 + 272.25 b_warehouse(1)(2)(7)#126
       + 272.25 b_warehouse(1)(2)(8)#127 + 272.25 b_warehouse(1)(2)(9)#128
       + 2838.94205647104 o(0)(0)#147 + 2838.94205647104 o(0)(1)#148
       + 2838.94205647104 o(0)(2)#149 + 1739.25271309261 o(1)(0)#150
       + 1739.25271309261 o(1)(1)#151 + 1739.25271309261 o(1)(2)#152
       + 4037.75629949332 o(2)(0)#153 + 4037.75629949332 o(2)(1)#154
       + 4037.75629949332 o(2)(2)#155
Subject To
 ProductionCapacity(1):          - 304 y(0)#0 + p(0)#3 <= 0
 ProductionCapacity(2):          - 304 y(1)#1 + p(1)#4 <= 0
 ProductionCapacity(3):          - 304 y(2)#2 + p(2)#5 <= 0
 PlantInventoryCapacity(1):      I_plant(0)#6 <= 152
 PlantInventoryCapacity(2):      I_plant(1)#7 <= 152
 PlantInventoryCapacity(3):      I_plant(2)#8 <= 152
 PlantInventoryBalance(1):       - p(0)#3 + I_plant(0)#6 + q(0)(0)(0)#129
                                 + q(0)(1)(0)#132 + q(1)(0)(0)#135
                                 + q(1)(1)(0)#138 + q(2)(0)(0)#141
                                 + q(2)(1)(0)#144  = 0
 PlantInventoryBalance(2):       - p(1)#4 - I_plant(0)#6 + I_plant(1)#7
                                 + q(0)(0)(1)#130 + q(0)(1)(1)#133
                                 + q(1)(0)(1)#136 + q(1)(1)(1)#139
                                 + q(2)(0)(1)#142 + q(2)(1)(1)#145  = 0
 PlantInventoryBalance(3):       - p(2)#5 - I_plant(1)#7 + I_plant(2)#8
                                 + q(0)(0)(2)#131 + q(0)(1)(2)#134
                                 + q(1)(0)(2)#137 + q(1)(1)(2)#140
                                 + q(2)(0)(2)#143 + q(2)(1)(2)#146  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(0)(0)(0)#9 <= 136.8
 WarehouseInventoryCapacity(2,1,1): I_warehouse(1)(0)(0)#39 <= 136.8
 WarehouseInventoryCapacity(1,2,1): I_warehouse(0)(1)(0)#19 <= 136.8
 WarehouseInventoryCapacity(2,2,1): I_warehouse(1)(1)(0)#49 <= 136.8
 WarehouseInventoryCapacity(1,3,1): I_warehouse(0)(2)(0)#29 <= 136.8
 WarehouseInventoryCapacity(2,3,1): I_warehouse(1)(2)(0)#59 <= 136.8
 WarehouseInventoryCapacity(1,1,2): I_warehouse(0)(0)(1)#10 <= 136.8
 WarehouseInventoryCapacity(2,1,2): I_warehouse(1)(0)(1)#40 <= 136.8
 WarehouseInventoryCapacity(1,2,2): I_warehouse(0)(1)(1)#20 <= 136.8
 WarehouseInventoryCapacity(2,2,2): I_warehouse(1)(1)(1)#50 <= 136.8
 WarehouseInventoryCapacity(1,3,2): I_warehouse(0)(2)(1)#30 <= 136.8
 WarehouseInventoryCapacity(2,3,2): I_warehouse(1)(2)(1)#60 <= 136.8
 WarehouseInventoryCapacity(1,1,3): I_warehouse(0)(0)(2)#11 <= 136.8
 WarehouseInventoryCapacity(2,1,3): I_warehouse(1)(0)(2)#41 <= 136.8
 WarehouseInventoryCapacity(1,2,3): I_warehouse(0)(1)(2)#21 <= 136.8
 WarehouseInventoryCapacity(2,2,3): I_warehouse(1)(1)(2)#51 <= 136.8
 WarehouseInventoryCapacity(1,3,3): I_warehouse(0)(2)(2)#31 <= 136.8
 WarehouseInventoryCapacity(2,3,3): I_warehouse(1)(2)(2)#61 <= 136.8
 WarehouseInventoryCapacity(1,1,4): I_warehouse(0)(0)(3)#12 <= 136.8
 WarehouseInventoryCapacity(2,1,4): I_warehouse(1)(0)(3)#42 <= 136.8
 WarehouseInventoryCapacity(1,2,4): I_warehouse(0)(1)(3)#22 <= 136.8
 WarehouseInventoryCapacity(2,2,4): I_warehouse(1)(1)(3)#52 <= 136.8
 WarehouseInventoryCapacity(1,3,4): I_warehouse(0)(2)(3)#32 <= 136.8
 WarehouseInventoryCapacity(2,3,4): I_warehouse(1)(2)(3)#62 <= 136.8
 WarehouseInventoryCapacity(1,1,5): I_warehouse(0)(0)(4)#13 <= 136.8
 WarehouseInventoryCapacity(2,1,5): I_warehouse(1)(0)(4)#43 <= 136.8
 WarehouseInventoryCapacity(1,2,5): I_warehouse(0)(1)(4)#23 <= 136.8
 WarehouseInventoryCapacity(2,2,5): I_warehouse(1)(1)(4)#53 <= 136.8
 WarehouseInventoryCapacity(1,3,5): I_warehouse(0)(2)(4)#33 <= 136.8
 WarehouseInventoryCapacity(2,3,5): I_warehouse(1)(2)(4)#63 <= 136.8
 WarehouseInventoryCapacity(1,1,6): I_warehouse(0)(0)(5)#14 <= 136.8
 WarehouseInventoryCapacity(2,1,6): I_warehouse(1)(0)(5)#44 <= 136.8
 WarehouseInventoryCapacity(1,2,6): I_warehouse(0)(1)(5)#24 <= 136.8
 WarehouseInventoryCapacity(2,2,6): I_warehouse(1)(1)(5)#54 <= 136.8
 WarehouseInventoryCapacity(1,3,6): I_warehouse(0)(2)(5)#34 <= 136.8
 WarehouseInventoryCapacity(2,3,6): I_warehouse(1)(2)(5)#64 <= 136.8
 WarehouseInventoryCapacity(1,1,7): I_warehouse(0)(0)(6)#15 <= 136.8
 WarehouseInventoryCapacity(2,1,7): I_warehouse(1)(0)(6)#45 <= 136.8
 WarehouseInventoryCapacity(1,2,7): I_warehouse(0)(1)(6)#25 <= 136.8
 WarehouseInventoryCapacity(2,2,7): I_warehouse(1)(1)(6)#55 <= 136.8
 WarehouseInventoryCapacity(1,3,7): I_warehouse(0)(2)(6)#35 <= 136.8
 WarehouseInventoryCapacity(2,3,7): I_warehouse(1)(2)(6)#65 <= 136.8
 WarehouseInventoryCapacity(1,1,8): I_warehouse(0)(0)(7)#16 <= 136.8
 WarehouseInventoryCapacity(2,1,8): I_warehouse(1)(0)(7)#46 <= 136.8
 WarehouseInventoryCapacity(1,2,8): I_warehouse(0)(1)(7)#26 <= 136.8
 WarehouseInventoryCapacity(2,2,8): I_warehouse(1)(1)(7)#56 <= 136.8
 WarehouseInventoryCapacity(1,3,8): I_warehouse(0)(2)(7)#36 <= 136.8
 WarehouseInventoryCapacity(2,3,8): I_warehouse(1)(2)(7)#66 <= 136.8
 WarehouseInventoryCapacity(1,1,9): I_warehouse(0)(0)(8)#17 <= 136.8
 WarehouseInventoryCapacity(2,1,9): I_warehouse(1)(0)(8)#47 <= 136.8
 WarehouseInventoryCapacity(1,2,9): I_warehouse(0)(1)(8)#27 <= 136.8
 WarehouseInventoryCapacity(2,2,9): I_warehouse(1)(1)(8)#57 <= 136.8
 WarehouseInventoryCapacity(1,3,9): I_warehouse(0)(2)(8)#37 <= 136.8
 WarehouseInventoryCapacity(2,3,9): I_warehouse(1)(2)(8)#67 <= 136.8
 WarehouseInventoryCapacity(1,1,10): I_warehouse(0)(0)(9)#18 <= 136.8
 WarehouseInventoryCapacity(2,1,10): I_warehouse(1)(0)(9)#48 <= 136.8
 WarehouseInventoryCapacity(1,2,10): I_warehouse(0)(1)(9)#28 <= 136.8
 WarehouseInventoryCapacity(2,2,10): I_warehouse(1)(1)(9)#58 <= 136.8
 WarehouseInventoryCapacity(1,3,10): I_warehouse(0)(2)(9)#38 <= 136.8
 WarehouseInventoryCapacity(2,3,10): I_warehouse(1)(2)(9)#68 <= 136.8
 WarehouseInventoryBalance(1,1,1): I_warehouse(0)(0)(0)#9
                                 - b_warehouse(0)(0)(0)#69 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -9
 WarehouseInventoryBalance(2,1,1): I_warehouse(1)(0)(0)#39
                                 - b_warehouse(1)(0)(0)#99 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -10
 WarehouseInventoryBalance(1,2,1): - I_warehouse(0)(0)(0)#9
                                 + I_warehouse(0)(1)(0)#19
                                 - b_warehouse(0)(1)(0)#79 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -31
 WarehouseInventoryBalance(2,2,1): - I_warehouse(1)(0)(0)#39
                                 + I_warehouse(1)(1)(0)#49
                                 - b_warehouse(1)(1)(0)#109 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -36
 WarehouseInventoryBalance(1,3,1): - I_warehouse(0)(1)(0)#19
                                 + I_warehouse(0)(2)(0)#29
                                 - b_warehouse(0)(2)(0)#89 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -74
 WarehouseInventoryBalance(2,3,1): - I_warehouse(1)(1)(0)#49
                                 + I_warehouse(1)(2)(0)#59
                                 - b_warehouse(1)(2)(0)#119 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -49
 WarehouseInventoryBalance(1,1,2): I_warehouse(0)(0)(1)#10
                                 - b_warehouse(0)(0)(1)#70 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -89
 WarehouseInventoryBalance(2,1,2): I_warehouse(1)(0)(1)#40
                                 - b_warehouse(1)(0)(1)#100 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -51
 WarehouseInventoryBalance(1,2,2): - I_warehouse(0)(0)(1)#10
                                 + I_warehouse(0)(1)(1)#20
                                 - b_warehouse(0)(1)(1)#80 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -99
 WarehouseInventoryBalance(2,2,2): - I_warehouse(1)(0)(1)#40
                                 + I_warehouse(1)(1)(1)#50
                                 - b_warehouse(1)(1)(1)#110 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -55
 WarehouseInventoryBalance(1,3,2): - I_warehouse(0)(1)(1)#20
                                 + I_warehouse(0)(2)(1)#30
                                 - b_warehouse(0)(2)(1)#90 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -104
 WarehouseInventoryBalance(2,3,2): - I_warehouse(1)(1)(1)#50
                                 + I_warehouse(1)(2)(1)#60
                                 - b_warehouse(1)(2)(1)#120 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -51
 WarehouseInventoryBalance(1,1,3): I_warehouse(0)(0)(2)#11
                                 - b_warehouse(0)(0)(2)#71 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -94
 WarehouseInventoryBalance(2,1,3): I_warehouse(1)(0)(2)#41
                                 - b_warehouse(1)(0)(2)#101 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -60
 WarehouseInventoryBalance(1,2,3): - I_warehouse(0)(0)(2)#11
                                 + I_warehouse(0)(1)(2)#21
                                 - b_warehouse(0)(1)(2)#81 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -100
 WarehouseInventoryBalance(2,2,3): - I_warehouse(1)(0)(2)#41
                                 + I_warehouse(1)(1)(2)#51
                                 - b_warehouse(1)(1)(2)#111 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -58
 WarehouseInventoryBalance(1,3,3): - I_warehouse(0)(1)(2)#21
                                 + I_warehouse(0)(2)(2)#31
                                 - b_warehouse(0)(2)(2)#91 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -102
 WarehouseInventoryBalance(2,3,3): - I_warehouse(1)(1)(2)#51
                                 + I_warehouse(1)(2)(2)#61
                                 - b_warehouse(1)(2)(2)#121 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -58
 WarehouseInventoryBalance(1,1,4): I_warehouse(0)(0)(3)#12
                                 - b_warehouse(0)(0)(3)#72 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -103
 WarehouseInventoryBalance(2,1,4): I_warehouse(1)(0)(3)#42
                                 - b_warehouse(1)(0)(3)#102 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -57
 WarehouseInventoryBalance(1,2,4): - I_warehouse(0)(0)(3)#12
                                 + I_warehouse(0)(1)(3)#22
                                 - b_warehouse(0)(1)(3)#82 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -105
 WarehouseInventoryBalance(2,2,4): - I_warehouse(1)(0)(3)#42
                                 + I_warehouse(1)(1)(3)#52
                                 - b_warehouse(1)(1)(3)#112 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -52
 WarehouseInventoryBalance(1,3,4): - I_warehouse(0)(1)(3)#22
                                 + I_warehouse(0)(2)(3)#32
                                 - b_warehouse(0)(2)(3)#92 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -95
 WarehouseInventoryBalance(2,3,4): - I_warehouse(1)(1)(3)#52
                                 + I_warehouse(1)(2)(3)#62
                                 - b_warehouse(1)(2)(3)#122 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -50
 WarehouseInventoryBalance(1,1,5): I_warehouse(0)(0)(4)#13
                                 - b_warehouse(0)(0)(4)#73 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -106
 WarehouseInventoryBalance(2,1,5): I_warehouse(1)(0)(4)#43
                                 - b_warehouse(1)(0)(4)#103 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -59
 WarehouseInventoryBalance(1,2,5): - I_warehouse(0)(0)(4)#13
                                 + I_warehouse(0)(1)(4)#23
                                 - b_warehouse(0)(1)(4)#83 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -98
 WarehouseInventoryBalance(2,2,5): - I_warehouse(1)(0)(4)#43
                                 + I_warehouse(1)(1)(4)#53
                                 - b_warehouse(1)(1)(4)#113 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -55
 WarehouseInventoryBalance(1,3,5): - I_warehouse(0)(1)(4)#23
                                 + I_warehouse(0)(2)(4)#33
                                 - b_warehouse(0)(2)(4)#93 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -92
 WarehouseInventoryBalance(2,3,5): - I_warehouse(1)(1)(4)#53
                                 + I_warehouse(1)(2)(4)#63
                                 - b_warehouse(1)(2)(4)#123 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -49
 WarehouseInventoryBalance(1,1,6): I_warehouse(0)(0)(5)#14
                                 - b_warehouse(0)(0)(5)#74 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -98
 WarehouseInventoryBalance(2,1,6): I_warehouse(1)(0)(5)#44
                                 - b_warehouse(1)(0)(5)#104 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -54
 WarehouseInventoryBalance(1,2,6): - I_warehouse(0)(0)(5)#14
                                 + I_warehouse(0)(1)(5)#24
                                 - b_warehouse(0)(1)(5)#84 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -91
 WarehouseInventoryBalance(2,2,6): - I_warehouse(1)(0)(5)#44
                                 + I_warehouse(1)(1)(5)#54
                                 - b_warehouse(1)(1)(5)#114 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -53
 WarehouseInventoryBalance(1,3,6): - I_warehouse(0)(1)(5)#24
                                 + I_warehouse(0)(2)(5)#34
                                 - b_warehouse(0)(2)(5)#94 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -101
 WarehouseInventoryBalance(2,3,6): - I_warehouse(1)(1)(5)#54
                                 + I_warehouse(1)(2)(5)#64
                                 - b_warehouse(1)(2)(5)#124 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -54
 WarehouseInventoryBalance(1,1,7): I_warehouse(0)(0)(6)#15
                                 - b_warehouse(0)(0)(6)#75 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -94
 WarehouseInventoryBalance(2,1,7): I_warehouse(1)(0)(6)#45
                                 - b_warehouse(1)(0)(6)#105 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -55
 WarehouseInventoryBalance(1,2,7): - I_warehouse(0)(0)(6)#15
                                 + I_warehouse(0)(1)(6)#25
                                 - b_warehouse(0)(1)(6)#85 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -97
 WarehouseInventoryBalance(2,2,7): - I_warehouse(1)(0)(6)#45
                                 + I_warehouse(1)(1)(6)#55
                                 - b_warehouse(1)(1)(6)#115 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -59
 WarehouseInventoryBalance(1,3,7): - I_warehouse(0)(1)(6)#25
                                 + I_warehouse(0)(2)(6)#35
                                 - b_warehouse(0)(2)(6)#95 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -106
 WarehouseInventoryBalance(2,3,7): - I_warehouse(1)(1)(6)#55
                                 + I_warehouse(1)(2)(6)#65
                                 - b_warehouse(1)(2)(6)#125 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -53
 WarehouseInventoryBalance(1,1,8): I_warehouse(0)(0)(7)#16
                                 - b_warehouse(0)(0)(7)#76 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -92
 WarehouseInventoryBalance(2,1,8): I_warehouse(1)(0)(7)#46
                                 - b_warehouse(1)(0)(7)#106 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -51
 WarehouseInventoryBalance(1,2,8): - I_warehouse(0)(0)(7)#16
                                 + I_warehouse(0)(1)(7)#26
                                 - b_warehouse(0)(1)(7)#86 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -95
 WarehouseInventoryBalance(2,2,8): - I_warehouse(1)(0)(7)#46
                                 + I_warehouse(1)(1)(7)#56
                                 - b_warehouse(1)(1)(7)#116 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -53
 WarehouseInventoryBalance(1,3,8): - I_warehouse(0)(1)(7)#26
                                 + I_warehouse(0)(2)(7)#36
                                 - b_warehouse(0)(2)(7)#96 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -104
 WarehouseInventoryBalance(2,3,8): - I_warehouse(1)(1)(7)#56
                                 + I_warehouse(1)(2)(7)#66
                                 - b_warehouse(1)(2)(7)#126 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -56
 WarehouseInventoryBalance(1,1,9): I_warehouse(0)(0)(8)#17
                                 - b_warehouse(0)(0)(8)#77 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -94
 WarehouseInventoryBalance(2,1,9): I_warehouse(1)(0)(8)#47
                                 - b_warehouse(1)(0)(8)#107 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -61
 WarehouseInventoryBalance(1,2,9): - I_warehouse(0)(0)(8)#17
                                 + I_warehouse(0)(1)(8)#27
                                 - b_warehouse(0)(1)(8)#87 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -100
 WarehouseInventoryBalance(2,2,9): - I_warehouse(1)(0)(8)#47
                                 + I_warehouse(1)(1)(8)#57
                                 - b_warehouse(1)(1)(8)#117 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -53
 WarehouseInventoryBalance(1,3,9): - I_warehouse(0)(1)(8)#27
                                 + I_warehouse(0)(2)(8)#37
                                 - b_warehouse(0)(2)(8)#97 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -98
 WarehouseInventoryBalance(2,3,9): - I_warehouse(1)(1)(8)#57
                                 + I_warehouse(1)(2)(8)#67
                                 - b_warehouse(1)(2)(8)#127 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -52
 WarehouseInventoryBalance(1,1,10): I_warehouse(0)(0)(9)#18
                                 - b_warehouse(0)(0)(9)#78 - q(0)(0)(0)#129
                                 - q(1)(0)(0)#135 - q(2)(0)(0)#141  = -90
 WarehouseInventoryBalance(2,1,10): I_warehouse(1)(0)(9)#48
                                 - b_warehouse(1)(0)(9)#108 - q(0)(1)(0)#132
                                 - q(1)(1)(0)#138 - q(2)(1)(0)#144  = -56
 WarehouseInventoryBalance(1,2,10): - I_warehouse(0)(0)(9)#18
                                 + I_warehouse(0)(1)(9)#28
                                 - b_warehouse(0)(1)(9)#88 - q(0)(0)(1)#130
                                 - q(1)(0)(1)#136 - q(2)(0)(1)#142  = -95
 WarehouseInventoryBalance(2,2,10): - I_warehouse(1)(0)(9)#48
                                 + I_warehouse(1)(1)(9)#58
                                 - b_warehouse(1)(1)(9)#118 - q(0)(1)(1)#133
                                 - q(1)(1)(1)#139 - q(2)(1)(1)#145  = -52
 WarehouseInventoryBalance(1,3,10): - I_warehouse(0)(1)(9)#28
                                 + I_warehouse(0)(2)(9)#38
                                 - b_warehouse(0)(2)(9)#98 - q(0)(0)(2)#131
                                 - q(1)(0)(2)#137 - q(2)(0)(2)#143  = -104
 WarehouseInventoryBalance(2,3,10): - I_warehouse(1)(1)(9)#58
                                 + I_warehouse(1)(2)(9)#68
                                 - b_warehouse(1)(2)(9)#128 - q(0)(1)(2)#134
                                 - q(1)(1)(2)#140 - q(2)(1)(2)#146  = -52
 WarehouseCoverage(1,1):         o(0)(0)#147 + o(2)(0)#153 <= 1
 WarehouseCoverage(2,1):         o(1)(0)#150 + o(2)(0)#153 <= 1
 WarehouseCoverage(1,2):         o(0)(1)#148 + o(2)(1)#154 <= 1
 WarehouseCoverage(2,2):         o(1)(1)#151 + o(2)(1)#154 <= 1
 WarehouseCoverage(1,3):         o(0)(2)#149 + o(2)(2)#155 <= 1
 WarehouseCoverage(2,3):         o(1)(2)#152 + o(2)(2)#155 <= 1
 VehicleCapacity(1,1):           q(0)(0)(0)#129 + q(0)(1)(0)#132
                                 - 102 o(0)(0)#147 <= 0
 VehicleCapacity(2,1):           q(1)(0)(0)#135 + q(1)(1)(0)#138
                                 - 102 o(1)(0)#150 <= 0
 VehicleCapacity(3,1):           q(2)(0)(0)#141 + q(2)(1)(0)#144
                                 - 102 o(2)(0)#153 <= 0
 VehicleCapacity(1,2):           q(0)(0)(1)#130 + q(0)(1)(1)#133
                                 - 102 o(0)(1)#148 <= 0
 VehicleCapacity(2,2):           q(1)(0)(1)#136 + q(1)(1)(1)#139
                                 - 102 o(1)(1)#151 <= 0
 VehicleCapacity(3,2):           q(2)(0)(1)#142 + q(2)(1)(1)#145
                                 - 102 o(2)(1)#154 <= 0
 VehicleCapacity(1,3):           q(0)(0)(2)#131 + q(0)(1)(2)#134
                                 - 102 o(0)(2)#149 <= 0
 VehicleCapacity(2,3):           q(1)(0)(2)#137 + q(1)(1)(2)#140
                                 - 102 o(1)(2)#152 <= 0
 VehicleCapacity(3,3):           q(2)(0)(2)#143 + q(2)(1)(2)#146
                                 - 102 o(2)(2)#155 <= 0
 FleetSize(1):                   o(0)(0)#147 + o(1)(0)#150 + o(2)(0)#153 <= 2
 FleetSize(2):                   o(0)(1)#148 + o(1)(1)#151 + o(2)(1)#154 <= 2
 FleetSize(3):                   o(0)(2)#149 + o(1)(2)#152 + o(2)(2)#155 <= 2
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
 0 <= I_warehouse(0)(0)(5)#14 <= 136.8
 0 <= I_warehouse(0)(0)(6)#15 <= 136.8
 0 <= I_warehouse(0)(0)(7)#16 <= 136.8
 0 <= I_warehouse(0)(0)(8)#17 <= 136.8
 0 <= I_warehouse(0)(0)(9)#18 <= 136.8
 0 <= I_warehouse(0)(1)(0)#19 <= 136.8
 0 <= I_warehouse(0)(1)(1)#20 <= 136.8
 0 <= I_warehouse(0)(1)(2)#21 <= 136.8
 0 <= I_warehouse(0)(1)(3)#22 <= 136.8
 0 <= I_warehouse(0)(1)(4)#23 <= 136.8
 0 <= I_warehouse(0)(1)(5)#24 <= 136.8
 0 <= I_warehouse(0)(1)(6)#25 <= 136.8
 0 <= I_warehouse(0)(1)(7)#26 <= 136.8
 0 <= I_warehouse(0)(1)(8)#27 <= 136.8
 0 <= I_warehouse(0)(1)(9)#28 <= 136.8
 0 <= I_warehouse(0)(2)(0)#29 <= 136.8
 0 <= I_warehouse(0)(2)(1)#30 <= 136.8
 0 <= I_warehouse(0)(2)(2)#31 <= 136.8
 0 <= I_warehouse(0)(2)(3)#32 <= 136.8
 0 <= I_warehouse(0)(2)(4)#33 <= 136.8
 0 <= I_warehouse(0)(2)(5)#34 <= 136.8
 0 <= I_warehouse(0)(2)(6)#35 <= 136.8
 0 <= I_warehouse(0)(2)(7)#36 <= 136.8
 0 <= I_warehouse(0)(2)(8)#37 <= 136.8
 0 <= I_warehouse(0)(2)(9)#38 <= 136.8
 0 <= I_warehouse(1)(0)(0)#39 <= 136.8
 0 <= I_warehouse(1)(0)(1)#40 <= 136.8
 0 <= I_warehouse(1)(0)(2)#41 <= 136.8
 0 <= I_warehouse(1)(0)(3)#42 <= 136.8
 0 <= I_warehouse(1)(0)(4)#43 <= 136.8
 0 <= I_warehouse(1)(0)(5)#44 <= 136.8
 0 <= I_warehouse(1)(0)(6)#45 <= 136.8
 0 <= I_warehouse(1)(0)(7)#46 <= 136.8
 0 <= I_warehouse(1)(0)(8)#47 <= 136.8
 0 <= I_warehouse(1)(0)(9)#48 <= 136.8
 0 <= I_warehouse(1)(1)(0)#49 <= 136.8
 0 <= I_warehouse(1)(1)(1)#50 <= 136.8
 0 <= I_warehouse(1)(1)(2)#51 <= 136.8
 0 <= I_warehouse(1)(1)(3)#52 <= 136.8
 0 <= I_warehouse(1)(1)(4)#53 <= 136.8
 0 <= I_warehouse(1)(1)(5)#54 <= 136.8
 0 <= I_warehouse(1)(1)(6)#55 <= 136.8
 0 <= I_warehouse(1)(1)(7)#56 <= 136.8
 0 <= I_warehouse(1)(1)(8)#57 <= 136.8
 0 <= I_warehouse(1)(1)(9)#58 <= 136.8
 0 <= I_warehouse(1)(2)(0)#59 <= 136.8
 0 <= I_warehouse(1)(2)(1)#60 <= 136.8
 0 <= I_warehouse(1)(2)(2)#61 <= 136.8
 0 <= I_warehouse(1)(2)(3)#62 <= 136.8
 0 <= I_warehouse(1)(2)(4)#63 <= 136.8
 0 <= I_warehouse(1)(2)(5)#64 <= 136.8
 0 <= I_warehouse(1)(2)(6)#65 <= 136.8
 0 <= I_warehouse(1)(2)(7)#66 <= 136.8
 0 <= I_warehouse(1)(2)(8)#67 <= 136.8
 0 <= I_warehouse(1)(2)(9)#68 <= 136.8
 0 <= b_warehouse(0)(0)(0)#69 <= 9
 0 <= b_warehouse(0)(0)(1)#70 <= 89
 0 <= b_warehouse(0)(0)(2)#71 <= 94
 0 <= b_warehouse(0)(0)(3)#72 <= 103
 0 <= b_warehouse(0)(0)(4)#73 <= 106
 0 <= b_warehouse(0)(0)(5)#74 <= 98
 0 <= b_warehouse(0)(0)(6)#75 <= 94
 0 <= b_warehouse(0)(0)(7)#76 <= 92
 0 <= b_warehouse(0)(0)(8)#77 <= 94
 0 <= b_warehouse(0)(0)(9)#78 <= 90
 0 <= b_warehouse(0)(1)(0)#79 <= 31
 0 <= b_warehouse(0)(1)(1)#80 <= 99
 0 <= b_warehouse(0)(1)(2)#81 <= 100
 0 <= b_warehouse(0)(1)(3)#82 <= 105
 0 <= b_warehouse(0)(1)(4)#83 <= 98
 0 <= b_warehouse(0)(1)(5)#84 <= 91
 0 <= b_warehouse(0)(1)(6)#85 <= 97
 0 <= b_warehouse(0)(1)(7)#86 <= 95
 0 <= b_warehouse(0)(1)(8)#87 <= 100
 0 <= b_warehouse(0)(1)(9)#88 <= 95
 0 <= b_warehouse(0)(2)(0)#89 <= 74
 0 <= b_warehouse(0)(2)(1)#90 <= 104
 0 <= b_warehouse(0)(2)(2)#91 <= 102
 0 <= b_warehouse(0)(2)(3)#92 <= 95
 0 <= b_warehouse(0)(2)(4)#93 <= 92
 0 <= b_warehouse(0)(2)(5)#94 <= 101
 0 <= b_warehouse(0)(2)(6)#95 <= 106
 0 <= b_warehouse(0)(2)(7)#96 <= 104
 0 <= b_warehouse(0)(2)(8)#97 <= 98
 0 <= b_warehouse(0)(2)(9)#98 <= 104
 0 <= b_warehouse(1)(0)(0)#99 <= 10
 0 <= b_warehouse(1)(0)(1)#100 <= 51
 0 <= b_warehouse(1)(0)(2)#101 <= 60
 0 <= b_warehouse(1)(0)(3)#102 <= 57
 0 <= b_warehouse(1)(0)(4)#103 <= 59
 0 <= b_warehouse(1)(0)(5)#104 <= 54
 0 <= b_warehouse(1)(0)(6)#105 <= 55
 0 <= b_warehouse(1)(0)(7)#106 <= 51
 0 <= b_warehouse(1)(0)(8)#107 <= 61
 0 <= b_warehouse(1)(0)(9)#108 <= 56
 0 <= b_warehouse(1)(1)(0)#109 <= 36
 0 <= b_warehouse(1)(1)(1)#110 <= 55
 0 <= b_warehouse(1)(1)(2)#111 <= 58
 0 <= b_warehouse(1)(1)(3)#112 <= 52
 0 <= b_warehouse(1)(1)(4)#113 <= 55
 0 <= b_warehouse(1)(1)(5)#114 <= 53
 0 <= b_warehouse(1)(1)(6)#115 <= 59
 0 <= b_warehouse(1)(1)(7)#116 <= 53
 0 <= b_warehouse(1)(1)(8)#117 <= 53
 0 <= b_warehouse(1)(1)(9)#118 <= 52
 0 <= b_warehouse(1)(2)(0)#119 <= 49
 0 <= b_warehouse(1)(2)(1)#120 <= 51
 0 <= b_warehouse(1)(2)(2)#121 <= 58
 0 <= b_warehouse(1)(2)(3)#122 <= 50
 0 <= b_warehouse(1)(2)(4)#123 <= 49
 0 <= b_warehouse(1)(2)(5)#124 <= 54
 0 <= b_warehouse(1)(2)(6)#125 <= 53
 0 <= b_warehouse(1)(2)(7)#126 <= 56
 0 <= b_warehouse(1)(2)(8)#127 <= 52
 0 <= b_warehouse(1)(2)(9)#128 <= 52
 0 <= q(0)(0)(0)#129 <= 136.8
 0 <= q(0)(0)(1)#130 <= 136.8
 0 <= q(0)(0)(2)#131 <= 136.8
      q(0)(1)(0)#132 = 0
      q(0)(1)(1)#133 = 0
      q(0)(1)(2)#134 = 0
      q(1)(0)(0)#135 = 0
      q(1)(0)(1)#136 = 0
      q(1)(0)(2)#137 = 0
 0 <= q(1)(1)(0)#138 <= 136.8
 0 <= q(1)(1)(1)#139 <= 136.8
 0 <= q(1)(1)(2)#140 <= 136.8
 0 <= q(2)(0)(0)#141 <= 136.8
 0 <= q(2)(0)(1)#142 <= 136.8
 0 <= q(2)(0)(2)#143 <= 136.8
 0 <= q(2)(1)(0)#144 <= 136.8
 0 <= q(2)(1)(1)#145 <= 136.8
 0 <= q(2)(1)(2)#146 <= 136.8
 0 <= o(0)(0)#147 <= 1
 0 <= o(0)(1)#148 <= 1
 0 <= o(0)(2)#149 <= 1
 0 <= o(1)(0)#150 <= 1
 0 <= o(1)(1)#151 <= 1
 0 <= o(1)(2)#152 <= 1
 0 <= o(2)(0)#153 <= 1
 0 <= o(2)(1)#154 <= 1
 0 <= o(2)(2)#155 <= 1
Binaries
 y(0)#0  y(1)#1  y(2)#2  o(0)(0)#147  o(0)(1)#148  o(0)(2)#149  o(1)(0)#150 
 o(1)(1)#151  o(1)(2)#152  o(2)(0)#153  o(2)(1)#154  o(2)(2)#155 
End
