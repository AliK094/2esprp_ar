\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 3000 y(0)#0 + 3000 y(1)#1 + 3000 y(2)#2 + 3000 y(3)#3 + 3000 y(4)#4
       + 3000 y(5)#5 + 30 p(0)#6 + 30 p(1)#7 + 30 p(2)#8 + 30 p(3)#9
       + 30 p(4)#10 + 30 p(5)#11 + 3 I_plant(0)#12 + 3 I_plant(1)#13
       + 3 I_plant(2)#14 + 3 I_plant(3)#15 + 3 I_plant(4)#16 + 3 I_plant(5)#17
       + 0.4 I_warehouse(0)(0)(0)#18 + 0.4 I_warehouse(0)(0)(1)#19
       + 0.4 I_warehouse(0)(0)(2)#20 + 0.4 I_warehouse(0)(0)(3)#21
       + 0.4 I_warehouse(0)(0)(4)#22 + 0.4 I_warehouse(0)(0)(5)#23
       + 0.4 I_warehouse(0)(0)(6)#24 + 0.4 I_warehouse(0)(0)(7)#25
       + 0.4 I_warehouse(0)(0)(8)#26 + 0.4 I_warehouse(0)(0)(9)#27
       + 0.4 I_warehouse(0)(1)(0)#28 + 0.4 I_warehouse(0)(1)(1)#29
       + 0.4 I_warehouse(0)(1)(2)#30 + 0.4 I_warehouse(0)(1)(3)#31
       + 0.4 I_warehouse(0)(1)(4)#32 + 0.4 I_warehouse(0)(1)(5)#33
       + 0.4 I_warehouse(0)(1)(6)#34 + 0.4 I_warehouse(0)(1)(7)#35
       + 0.4 I_warehouse(0)(1)(8)#36 + 0.4 I_warehouse(0)(1)(9)#37
       + 0.4 I_warehouse(0)(2)(0)#38 + 0.4 I_warehouse(0)(2)(1)#39
       + 0.4 I_warehouse(0)(2)(2)#40 + 0.4 I_warehouse(0)(2)(3)#41
       + 0.4 I_warehouse(0)(2)(4)#42 + 0.4 I_warehouse(0)(2)(5)#43
       + 0.4 I_warehouse(0)(2)(6)#44 + 0.4 I_warehouse(0)(2)(7)#45
       + 0.4 I_warehouse(0)(2)(8)#46 + 0.4 I_warehouse(0)(2)(9)#47
       + 0.4 I_warehouse(0)(3)(0)#48 + 0.4 I_warehouse(0)(3)(1)#49
       + 0.4 I_warehouse(0)(3)(2)#50 + 0.4 I_warehouse(0)(3)(3)#51
       + 0.4 I_warehouse(0)(3)(4)#52 + 0.4 I_warehouse(0)(3)(5)#53
       + 0.4 I_warehouse(0)(3)(6)#54 + 0.4 I_warehouse(0)(3)(7)#55
       + 0.4 I_warehouse(0)(3)(8)#56 + 0.4 I_warehouse(0)(3)(9)#57
       + 0.4 I_warehouse(0)(4)(0)#58 + 0.4 I_warehouse(0)(4)(1)#59
       + 0.4 I_warehouse(0)(4)(2)#60 + 0.4 I_warehouse(0)(4)(3)#61
       + 0.4 I_warehouse(0)(4)(4)#62 + 0.4 I_warehouse(0)(4)(5)#63
       + 0.4 I_warehouse(0)(4)(6)#64 + 0.4 I_warehouse(0)(4)(7)#65
       + 0.4 I_warehouse(0)(4)(8)#66 + 0.4 I_warehouse(0)(4)(9)#67
       + 0.4 I_warehouse(0)(5)(0)#68 + 0.4 I_warehouse(0)(5)(1)#69
       + 0.4 I_warehouse(0)(5)(2)#70 + 0.4 I_warehouse(0)(5)(3)#71
       + 0.4 I_warehouse(0)(5)(4)#72 + 0.4 I_warehouse(0)(5)(5)#73
       + 0.4 I_warehouse(0)(5)(6)#74 + 0.4 I_warehouse(0)(5)(7)#75
       + 0.4 I_warehouse(0)(5)(8)#76 + 0.4 I_warehouse(0)(5)(9)#77
       + 0.3 I_warehouse(1)(0)(0)#78 + 0.3 I_warehouse(1)(0)(1)#79
       + 0.3 I_warehouse(1)(0)(2)#80 + 0.3 I_warehouse(1)(0)(3)#81
       + 0.3 I_warehouse(1)(0)(4)#82 + 0.3 I_warehouse(1)(0)(5)#83
       + 0.3 I_warehouse(1)(0)(6)#84 + 0.3 I_warehouse(1)(0)(7)#85
       + 0.3 I_warehouse(1)(0)(8)#86 + 0.3 I_warehouse(1)(0)(9)#87
       + 0.3 I_warehouse(1)(1)(0)#88 + 0.3 I_warehouse(1)(1)(1)#89
       + 0.3 I_warehouse(1)(1)(2)#90 + 0.3 I_warehouse(1)(1)(3)#91
       + 0.3 I_warehouse(1)(1)(4)#92 + 0.3 I_warehouse(1)(1)(5)#93
       + 0.3 I_warehouse(1)(1)(6)#94 + 0.3 I_warehouse(1)(1)(7)#95
       + 0.3 I_warehouse(1)(1)(8)#96 + 0.3 I_warehouse(1)(1)(9)#97
       + 0.3 I_warehouse(1)(2)(0)#98 + 0.3 I_warehouse(1)(2)(1)#99
       + 0.3 I_warehouse(1)(2)(2)#100 + 0.3 I_warehouse(1)(2)(3)#101
       + 0.3 I_warehouse(1)(2)(4)#102 + 0.3 I_warehouse(1)(2)(5)#103
       + 0.3 I_warehouse(1)(2)(6)#104 + 0.3 I_warehouse(1)(2)(7)#105
       + 0.3 I_warehouse(1)(2)(8)#106 + 0.3 I_warehouse(1)(2)(9)#107
       + 0.3 I_warehouse(1)(3)(0)#108 + 0.3 I_warehouse(1)(3)(1)#109
       + 0.3 I_warehouse(1)(3)(2)#110 + 0.3 I_warehouse(1)(3)(3)#111
       + 0.3 I_warehouse(1)(3)(4)#112 + 0.3 I_warehouse(1)(3)(5)#113
       + 0.3 I_warehouse(1)(3)(6)#114 + 0.3 I_warehouse(1)(3)(7)#115
       + 0.3 I_warehouse(1)(3)(8)#116 + 0.3 I_warehouse(1)(3)(9)#117
       + 0.3 I_warehouse(1)(4)(0)#118 + 0.3 I_warehouse(1)(4)(1)#119
       + 0.3 I_warehouse(1)(4)(2)#120 + 0.3 I_warehouse(1)(4)(3)#121
       + 0.3 I_warehouse(1)(4)(4)#122 + 0.3 I_warehouse(1)(4)(5)#123
       + 0.3 I_warehouse(1)(4)(6)#124 + 0.3 I_warehouse(1)(4)(7)#125
       + 0.3 I_warehouse(1)(4)(8)#126 + 0.3 I_warehouse(1)(4)(9)#127
       + 0.3 I_warehouse(1)(5)(0)#128 + 0.3 I_warehouse(1)(5)(1)#129
       + 0.3 I_warehouse(1)(5)(2)#130 + 0.3 I_warehouse(1)(5)(3)#131
       + 0.3 I_warehouse(1)(5)(4)#132 + 0.3 I_warehouse(1)(5)(5)#133
       + 0.3 I_warehouse(1)(5)(6)#134 + 0.3 I_warehouse(1)(5)(7)#135
       + 0.3 I_warehouse(1)(5)(8)#136 + 0.3 I_warehouse(1)(5)(9)#137
       + 390.666666666667 b_warehouse(0)(0)(0)#138
       + 390.666666666667 b_warehouse(0)(0)(1)#139
       + 390.666666666667 b_warehouse(0)(0)(2)#140
       + 390.666666666667 b_warehouse(0)(0)(3)#141
       + 390.666666666667 b_warehouse(0)(0)(4)#142
       + 390.666666666667 b_warehouse(0)(0)(5)#143
       + 390.666666666667 b_warehouse(0)(0)(6)#144
       + 390.666666666667 b_warehouse(0)(0)(7)#145
       + 390.666666666667 b_warehouse(0)(0)(8)#146
       + 390.666666666667 b_warehouse(0)(0)(9)#147
       + 390.666666666667 b_warehouse(0)(1)(0)#148
       + 390.666666666667 b_warehouse(0)(1)(1)#149
       + 390.666666666667 b_warehouse(0)(1)(2)#150
       + 390.666666666667 b_warehouse(0)(1)(3)#151
       + 390.666666666667 b_warehouse(0)(1)(4)#152
       + 390.666666666667 b_warehouse(0)(1)(5)#153
       + 390.666666666667 b_warehouse(0)(1)(6)#154
       + 390.666666666667 b_warehouse(0)(1)(7)#155
       + 390.666666666667 b_warehouse(0)(1)(8)#156
       + 390.666666666667 b_warehouse(0)(1)(9)#157
       + 390.666666666667 b_warehouse(0)(2)(0)#158
       + 390.666666666667 b_warehouse(0)(2)(1)#159
       + 390.666666666667 b_warehouse(0)(2)(2)#160
       + 390.666666666667 b_warehouse(0)(2)(3)#161
       + 390.666666666667 b_warehouse(0)(2)(4)#162
       + 390.666666666667 b_warehouse(0)(2)(5)#163
       + 390.666666666667 b_warehouse(0)(2)(6)#164
       + 390.666666666667 b_warehouse(0)(2)(7)#165
       + 390.666666666667 b_warehouse(0)(2)(8)#166
       + 390.666666666667 b_warehouse(0)(2)(9)#167
       + 390.666666666667 b_warehouse(0)(3)(0)#168
       + 390.666666666667 b_warehouse(0)(3)(1)#169
       + 390.666666666667 b_warehouse(0)(3)(2)#170
       + 390.666666666667 b_warehouse(0)(3)(3)#171
       + 390.666666666667 b_warehouse(0)(3)(4)#172
       + 390.666666666667 b_warehouse(0)(3)(5)#173
       + 390.666666666667 b_warehouse(0)(3)(6)#174
       + 390.666666666667 b_warehouse(0)(3)(7)#175
       + 390.666666666667 b_warehouse(0)(3)(8)#176
       + 390.666666666667 b_warehouse(0)(3)(9)#177
       + 390.666666666667 b_warehouse(0)(4)(0)#178
       + 390.666666666667 b_warehouse(0)(4)(1)#179
       + 390.666666666667 b_warehouse(0)(4)(2)#180
       + 390.666666666667 b_warehouse(0)(4)(3)#181
       + 390.666666666667 b_warehouse(0)(4)(4)#182
       + 390.666666666667 b_warehouse(0)(4)(5)#183
       + 390.666666666667 b_warehouse(0)(4)(6)#184
       + 390.666666666667 b_warehouse(0)(4)(7)#185
       + 390.666666666667 b_warehouse(0)(4)(8)#186
       + 390.666666666667 b_warehouse(0)(4)(9)#187
       + 390.666666666667 b_warehouse(0)(5)(0)#188
       + 390.666666666667 b_warehouse(0)(5)(1)#189
       + 390.666666666667 b_warehouse(0)(5)(2)#190
       + 390.666666666667 b_warehouse(0)(5)(3)#191
       + 390.666666666667 b_warehouse(0)(5)(4)#192
       + 390.666666666667 b_warehouse(0)(5)(5)#193
       + 390.666666666667 b_warehouse(0)(5)(6)#194
       + 390.666666666667 b_warehouse(0)(5)(7)#195
       + 390.666666666667 b_warehouse(0)(5)(8)#196
       + 390.666666666667 b_warehouse(0)(5)(9)#197
       + 272.25 b_warehouse(1)(0)(0)#198 + 272.25 b_warehouse(1)(0)(1)#199
       + 272.25 b_warehouse(1)(0)(2)#200 + 272.25 b_warehouse(1)(0)(3)#201
       + 272.25 b_warehouse(1)(0)(4)#202 + 272.25 b_warehouse(1)(0)(5)#203
       + 272.25 b_warehouse(1)(0)(6)#204 + 272.25 b_warehouse(1)(0)(7)#205
       + 272.25 b_warehouse(1)(0)(8)#206 + 272.25 b_warehouse(1)(0)(9)#207
       + 272.25 b_warehouse(1)(1)(0)#208 + 272.25 b_warehouse(1)(1)(1)#209
       + 272.25 b_warehouse(1)(1)(2)#210 + 272.25 b_warehouse(1)(1)(3)#211
       + 272.25 b_warehouse(1)(1)(4)#212 + 272.25 b_warehouse(1)(1)(5)#213
       + 272.25 b_warehouse(1)(1)(6)#214 + 272.25 b_warehouse(1)(1)(7)#215
       + 272.25 b_warehouse(1)(1)(8)#216 + 272.25 b_warehouse(1)(1)(9)#217
       + 272.25 b_warehouse(1)(2)(0)#218 + 272.25 b_warehouse(1)(2)(1)#219
       + 272.25 b_warehouse(1)(2)(2)#220 + 272.25 b_warehouse(1)(2)(3)#221
       + 272.25 b_warehouse(1)(2)(4)#222 + 272.25 b_warehouse(1)(2)(5)#223
       + 272.25 b_warehouse(1)(2)(6)#224 + 272.25 b_warehouse(1)(2)(7)#225
       + 272.25 b_warehouse(1)(2)(8)#226 + 272.25 b_warehouse(1)(2)(9)#227
       + 272.25 b_warehouse(1)(3)(0)#228 + 272.25 b_warehouse(1)(3)(1)#229
       + 272.25 b_warehouse(1)(3)(2)#230 + 272.25 b_warehouse(1)(3)(3)#231
       + 272.25 b_warehouse(1)(3)(4)#232 + 272.25 b_warehouse(1)(3)(5)#233
       + 272.25 b_warehouse(1)(3)(6)#234 + 272.25 b_warehouse(1)(3)(7)#235
       + 272.25 b_warehouse(1)(3)(8)#236 + 272.25 b_warehouse(1)(3)(9)#237
       + 272.25 b_warehouse(1)(4)(0)#238 + 272.25 b_warehouse(1)(4)(1)#239
       + 272.25 b_warehouse(1)(4)(2)#240 + 272.25 b_warehouse(1)(4)(3)#241
       + 272.25 b_warehouse(1)(4)(4)#242 + 272.25 b_warehouse(1)(4)(5)#243
       + 272.25 b_warehouse(1)(4)(6)#244 + 272.25 b_warehouse(1)(4)(7)#245
       + 272.25 b_warehouse(1)(4)(8)#246 + 272.25 b_warehouse(1)(4)(9)#247
       + 272.25 b_warehouse(1)(5)(0)#248 + 272.25 b_warehouse(1)(5)(1)#249
       + 272.25 b_warehouse(1)(5)(2)#250 + 272.25 b_warehouse(1)(5)(3)#251
       + 272.25 b_warehouse(1)(5)(4)#252 + 272.25 b_warehouse(1)(5)(5)#253
       + 272.25 b_warehouse(1)(5)(6)#254 + 272.25 b_warehouse(1)(5)(7)#255
       + 272.25 b_warehouse(1)(5)(8)#256 + 272.25 b_warehouse(1)(5)(9)#257
       + 2838.94205647104 o(0)(0)#294 + 2838.94205647104 o(0)(1)#295
       + 2838.94205647104 o(0)(2)#296 + 2838.94205647104 o(0)(3)#297
       + 2838.94205647104 o(0)(4)#298 + 2838.94205647104 o(0)(5)#299
       + 1739.25271309261 o(1)(0)#300 + 1739.25271309261 o(1)(1)#301
       + 1739.25271309261 o(1)(2)#302 + 1739.25271309261 o(1)(3)#303
       + 1739.25271309261 o(1)(4)#304 + 1739.25271309261 o(1)(5)#305
       + 4037.75629949332 o(2)(0)#306 + 4037.75629949332 o(2)(1)#307
       + 4037.75629949332 o(2)(2)#308 + 4037.75629949332 o(2)(3)#309
       + 4037.75629949332 o(2)(4)#310 + 4037.75629949332 o(2)(5)#311
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
 PlantInventoryBalance(1):       - p(0)#6 + I_plant(0)#12 + q(0)(0)(0)#258
                                 + q(0)(1)(0)#264 + q(1)(0)(0)#270
                                 + q(1)(1)(0)#276 + q(2)(0)(0)#282
                                 + q(2)(1)(0)#288  = 0
 PlantInventoryBalance(2):       - p(1)#7 - I_plant(0)#12 + I_plant(1)#13
                                 + q(0)(0)(1)#259 + q(0)(1)(1)#265
                                 + q(1)(0)(1)#271 + q(1)(1)(1)#277
                                 + q(2)(0)(1)#283 + q(2)(1)(1)#289  = 0
 PlantInventoryBalance(3):       - p(2)#8 - I_plant(1)#13 + I_plant(2)#14
                                 + q(0)(0)(2)#260 + q(0)(1)(2)#266
                                 + q(1)(0)(2)#272 + q(1)(1)(2)#278
                                 + q(2)(0)(2)#284 + q(2)(1)(2)#290  = 0
 PlantInventoryBalance(4):       - p(3)#9 - I_plant(2)#14 + I_plant(3)#15
                                 + q(0)(0)(3)#261 + q(0)(1)(3)#267
                                 + q(1)(0)(3)#273 + q(1)(1)(3)#279
                                 + q(2)(0)(3)#285 + q(2)(1)(3)#291  = 0
 PlantInventoryBalance(5):       - p(4)#10 - I_plant(3)#15 + I_plant(4)#16
                                 + q(0)(0)(4)#262 + q(0)(1)(4)#268
                                 + q(1)(0)(4)#274 + q(1)(1)(4)#280
                                 + q(2)(0)(4)#286 + q(2)(1)(4)#292  = 0
 PlantInventoryBalance(6):       - p(5)#11 - I_plant(4)#16 + I_plant(5)#17
                                 + q(0)(0)(5)#263 + q(0)(1)(5)#269
                                 + q(1)(0)(5)#275 + q(1)(1)(5)#281
                                 + q(2)(0)(5)#287 + q(2)(1)(5)#293  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(0)(0)(0)#18 <= 136.8
 WarehouseInventoryCapacity(2,1,1): I_warehouse(1)(0)(0)#78 <= 136.8
 WarehouseInventoryCapacity(1,2,1): I_warehouse(0)(1)(0)#28 <= 136.8
 WarehouseInventoryCapacity(2,2,1): I_warehouse(1)(1)(0)#88 <= 136.8
 WarehouseInventoryCapacity(1,3,1): I_warehouse(0)(2)(0)#38 <= 136.8
 WarehouseInventoryCapacity(2,3,1): I_warehouse(1)(2)(0)#98 <= 136.8
 WarehouseInventoryCapacity(1,4,1): I_warehouse(0)(3)(0)#48 <= 136.8
 WarehouseInventoryCapacity(2,4,1): I_warehouse(1)(3)(0)#108 <= 136.8
 WarehouseInventoryCapacity(1,5,1): I_warehouse(0)(4)(0)#58 <= 136.8
 WarehouseInventoryCapacity(2,5,1): I_warehouse(1)(4)(0)#118 <= 136.8
 WarehouseInventoryCapacity(1,6,1): I_warehouse(0)(5)(0)#68 <= 136.8
 WarehouseInventoryCapacity(2,6,1): I_warehouse(1)(5)(0)#128 <= 136.8
 WarehouseInventoryCapacity(1,1,2): I_warehouse(0)(0)(1)#19 <= 136.8
 WarehouseInventoryCapacity(2,1,2): I_warehouse(1)(0)(1)#79 <= 136.8
 WarehouseInventoryCapacity(1,2,2): I_warehouse(0)(1)(1)#29 <= 136.8
 WarehouseInventoryCapacity(2,2,2): I_warehouse(1)(1)(1)#89 <= 136.8
 WarehouseInventoryCapacity(1,3,2): I_warehouse(0)(2)(1)#39 <= 136.8
 WarehouseInventoryCapacity(2,3,2): I_warehouse(1)(2)(1)#99 <= 136.8
 WarehouseInventoryCapacity(1,4,2): I_warehouse(0)(3)(1)#49 <= 136.8
 WarehouseInventoryCapacity(2,4,2): I_warehouse(1)(3)(1)#109 <= 136.8
 WarehouseInventoryCapacity(1,5,2): I_warehouse(0)(4)(1)#59 <= 136.8
 WarehouseInventoryCapacity(2,5,2): I_warehouse(1)(4)(1)#119 <= 136.8
 WarehouseInventoryCapacity(1,6,2): I_warehouse(0)(5)(1)#69 <= 136.8
 WarehouseInventoryCapacity(2,6,2): I_warehouse(1)(5)(1)#129 <= 136.8
 WarehouseInventoryCapacity(1,1,3): I_warehouse(0)(0)(2)#20 <= 136.8
 WarehouseInventoryCapacity(2,1,3): I_warehouse(1)(0)(2)#80 <= 136.8
 WarehouseInventoryCapacity(1,2,3): I_warehouse(0)(1)(2)#30 <= 136.8
 WarehouseInventoryCapacity(2,2,3): I_warehouse(1)(1)(2)#90 <= 136.8
 WarehouseInventoryCapacity(1,3,3): I_warehouse(0)(2)(2)#40 <= 136.8
 WarehouseInventoryCapacity(2,3,3): I_warehouse(1)(2)(2)#100 <= 136.8
 WarehouseInventoryCapacity(1,4,3): I_warehouse(0)(3)(2)#50 <= 136.8
 WarehouseInventoryCapacity(2,4,3): I_warehouse(1)(3)(2)#110 <= 136.8
 WarehouseInventoryCapacity(1,5,3): I_warehouse(0)(4)(2)#60 <= 136.8
 WarehouseInventoryCapacity(2,5,3): I_warehouse(1)(4)(2)#120 <= 136.8
 WarehouseInventoryCapacity(1,6,3): I_warehouse(0)(5)(2)#70 <= 136.8
 WarehouseInventoryCapacity(2,6,3): I_warehouse(1)(5)(2)#130 <= 136.8
 WarehouseInventoryCapacity(1,1,4): I_warehouse(0)(0)(3)#21 <= 136.8
 WarehouseInventoryCapacity(2,1,4): I_warehouse(1)(0)(3)#81 <= 136.8
 WarehouseInventoryCapacity(1,2,4): I_warehouse(0)(1)(3)#31 <= 136.8
 WarehouseInventoryCapacity(2,2,4): I_warehouse(1)(1)(3)#91 <= 136.8
 WarehouseInventoryCapacity(1,3,4): I_warehouse(0)(2)(3)#41 <= 136.8
 WarehouseInventoryCapacity(2,3,4): I_warehouse(1)(2)(3)#101 <= 136.8
 WarehouseInventoryCapacity(1,4,4): I_warehouse(0)(3)(3)#51 <= 136.8
 WarehouseInventoryCapacity(2,4,4): I_warehouse(1)(3)(3)#111 <= 136.8
 WarehouseInventoryCapacity(1,5,4): I_warehouse(0)(4)(3)#61 <= 136.8
 WarehouseInventoryCapacity(2,5,4): I_warehouse(1)(4)(3)#121 <= 136.8
 WarehouseInventoryCapacity(1,6,4): I_warehouse(0)(5)(3)#71 <= 136.8
 WarehouseInventoryCapacity(2,6,4): I_warehouse(1)(5)(3)#131 <= 136.8
 WarehouseInventoryCapacity(1,1,5): I_warehouse(0)(0)(4)#22 <= 136.8
 WarehouseInventoryCapacity(2,1,5): I_warehouse(1)(0)(4)#82 <= 136.8
 WarehouseInventoryCapacity(1,2,5): I_warehouse(0)(1)(4)#32 <= 136.8
 WarehouseInventoryCapacity(2,2,5): I_warehouse(1)(1)(4)#92 <= 136.8
 WarehouseInventoryCapacity(1,3,5): I_warehouse(0)(2)(4)#42 <= 136.8
 WarehouseInventoryCapacity(2,3,5): I_warehouse(1)(2)(4)#102 <= 136.8
 WarehouseInventoryCapacity(1,4,5): I_warehouse(0)(3)(4)#52 <= 136.8
 WarehouseInventoryCapacity(2,4,5): I_warehouse(1)(3)(4)#112 <= 136.8
 WarehouseInventoryCapacity(1,5,5): I_warehouse(0)(4)(4)#62 <= 136.8
 WarehouseInventoryCapacity(2,5,5): I_warehouse(1)(4)(4)#122 <= 136.8
 WarehouseInventoryCapacity(1,6,5): I_warehouse(0)(5)(4)#72 <= 136.8
 WarehouseInventoryCapacity(2,6,5): I_warehouse(1)(5)(4)#132 <= 136.8
 WarehouseInventoryCapacity(1,1,6): I_warehouse(0)(0)(5)#23 <= 136.8
 WarehouseInventoryCapacity(2,1,6): I_warehouse(1)(0)(5)#83 <= 136.8
 WarehouseInventoryCapacity(1,2,6): I_warehouse(0)(1)(5)#33 <= 136.8
 WarehouseInventoryCapacity(2,2,6): I_warehouse(1)(1)(5)#93 <= 136.8
 WarehouseInventoryCapacity(1,3,6): I_warehouse(0)(2)(5)#43 <= 136.8
 WarehouseInventoryCapacity(2,3,6): I_warehouse(1)(2)(5)#103 <= 136.8
 WarehouseInventoryCapacity(1,4,6): I_warehouse(0)(3)(5)#53 <= 136.8
 WarehouseInventoryCapacity(2,4,6): I_warehouse(1)(3)(5)#113 <= 136.8
 WarehouseInventoryCapacity(1,5,6): I_warehouse(0)(4)(5)#63 <= 136.8
 WarehouseInventoryCapacity(2,5,6): I_warehouse(1)(4)(5)#123 <= 136.8
 WarehouseInventoryCapacity(1,6,6): I_warehouse(0)(5)(5)#73 <= 136.8
 WarehouseInventoryCapacity(2,6,6): I_warehouse(1)(5)(5)#133 <= 136.8
 WarehouseInventoryCapacity(1,1,7): I_warehouse(0)(0)(6)#24 <= 136.8
 WarehouseInventoryCapacity(2,1,7): I_warehouse(1)(0)(6)#84 <= 136.8
 WarehouseInventoryCapacity(1,2,7): I_warehouse(0)(1)(6)#34 <= 136.8
 WarehouseInventoryCapacity(2,2,7): I_warehouse(1)(1)(6)#94 <= 136.8
 WarehouseInventoryCapacity(1,3,7): I_warehouse(0)(2)(6)#44 <= 136.8
 WarehouseInventoryCapacity(2,3,7): I_warehouse(1)(2)(6)#104 <= 136.8
 WarehouseInventoryCapacity(1,4,7): I_warehouse(0)(3)(6)#54 <= 136.8
 WarehouseInventoryCapacity(2,4,7): I_warehouse(1)(3)(6)#114 <= 136.8
 WarehouseInventoryCapacity(1,5,7): I_warehouse(0)(4)(6)#64 <= 136.8
 WarehouseInventoryCapacity(2,5,7): I_warehouse(1)(4)(6)#124 <= 136.8
 WarehouseInventoryCapacity(1,6,7): I_warehouse(0)(5)(6)#74 <= 136.8
 WarehouseInventoryCapacity(2,6,7): I_warehouse(1)(5)(6)#134 <= 136.8
 WarehouseInventoryCapacity(1,1,8): I_warehouse(0)(0)(7)#25 <= 136.8
 WarehouseInventoryCapacity(2,1,8): I_warehouse(1)(0)(7)#85 <= 136.8
 WarehouseInventoryCapacity(1,2,8): I_warehouse(0)(1)(7)#35 <= 136.8
 WarehouseInventoryCapacity(2,2,8): I_warehouse(1)(1)(7)#95 <= 136.8
 WarehouseInventoryCapacity(1,3,8): I_warehouse(0)(2)(7)#45 <= 136.8
 WarehouseInventoryCapacity(2,3,8): I_warehouse(1)(2)(7)#105 <= 136.8
 WarehouseInventoryCapacity(1,4,8): I_warehouse(0)(3)(7)#55 <= 136.8
 WarehouseInventoryCapacity(2,4,8): I_warehouse(1)(3)(7)#115 <= 136.8
 WarehouseInventoryCapacity(1,5,8): I_warehouse(0)(4)(7)#65 <= 136.8
 WarehouseInventoryCapacity(2,5,8): I_warehouse(1)(4)(7)#125 <= 136.8
 WarehouseInventoryCapacity(1,6,8): I_warehouse(0)(5)(7)#75 <= 136.8
 WarehouseInventoryCapacity(2,6,8): I_warehouse(1)(5)(7)#135 <= 136.8
 WarehouseInventoryCapacity(1,1,9): I_warehouse(0)(0)(8)#26 <= 136.8
 WarehouseInventoryCapacity(2,1,9): I_warehouse(1)(0)(8)#86 <= 136.8
 WarehouseInventoryCapacity(1,2,9): I_warehouse(0)(1)(8)#36 <= 136.8
 WarehouseInventoryCapacity(2,2,9): I_warehouse(1)(1)(8)#96 <= 136.8
 WarehouseInventoryCapacity(1,3,9): I_warehouse(0)(2)(8)#46 <= 136.8
 WarehouseInventoryCapacity(2,3,9): I_warehouse(1)(2)(8)#106 <= 136.8
 WarehouseInventoryCapacity(1,4,9): I_warehouse(0)(3)(8)#56 <= 136.8
 WarehouseInventoryCapacity(2,4,9): I_warehouse(1)(3)(8)#116 <= 136.8
 WarehouseInventoryCapacity(1,5,9): I_warehouse(0)(4)(8)#66 <= 136.8
 WarehouseInventoryCapacity(2,5,9): I_warehouse(1)(4)(8)#126 <= 136.8
 WarehouseInventoryCapacity(1,6,9): I_warehouse(0)(5)(8)#76 <= 136.8
 WarehouseInventoryCapacity(2,6,9): I_warehouse(1)(5)(8)#136 <= 136.8
 WarehouseInventoryCapacity(1,1,10): I_warehouse(0)(0)(9)#27 <= 136.8
 WarehouseInventoryCapacity(2,1,10): I_warehouse(1)(0)(9)#87 <= 136.8
 WarehouseInventoryCapacity(1,2,10): I_warehouse(0)(1)(9)#37 <= 136.8
 WarehouseInventoryCapacity(2,2,10): I_warehouse(1)(1)(9)#97 <= 136.8
 WarehouseInventoryCapacity(1,3,10): I_warehouse(0)(2)(9)#47 <= 136.8
 WarehouseInventoryCapacity(2,3,10): I_warehouse(1)(2)(9)#107 <= 136.8
 WarehouseInventoryCapacity(1,4,10): I_warehouse(0)(3)(9)#57 <= 136.8
 WarehouseInventoryCapacity(2,4,10): I_warehouse(1)(3)(9)#117 <= 136.8
 WarehouseInventoryCapacity(1,5,10): I_warehouse(0)(4)(9)#67 <= 136.8
 WarehouseInventoryCapacity(2,5,10): I_warehouse(1)(4)(9)#127 <= 136.8
 WarehouseInventoryCapacity(1,6,10): I_warehouse(0)(5)(9)#77 <= 136.8
 WarehouseInventoryCapacity(2,6,10): I_warehouse(1)(5)(9)#137 <= 136.8
 WarehouseInventoryBalance(1,1,1): I_warehouse(0)(0)(0)#18
                                 - b_warehouse(0)(0)(0)#138 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -9
 WarehouseInventoryBalance(2,1,1): I_warehouse(1)(0)(0)#78
                                 - b_warehouse(1)(0)(0)#198 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -10
 WarehouseInventoryBalance(1,2,1): - I_warehouse(0)(0)(0)#18
                                 + I_warehouse(0)(1)(0)#28
                                 - b_warehouse(0)(1)(0)#148 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -31
 WarehouseInventoryBalance(2,2,1): - I_warehouse(1)(0)(0)#78
                                 + I_warehouse(1)(1)(0)#88
                                 - b_warehouse(1)(1)(0)#208 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -36
 WarehouseInventoryBalance(1,3,1): - I_warehouse(0)(1)(0)#28
                                 + I_warehouse(0)(2)(0)#38
                                 - b_warehouse(0)(2)(0)#158 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -74
 WarehouseInventoryBalance(2,3,1): - I_warehouse(1)(1)(0)#88
                                 + I_warehouse(1)(2)(0)#98
                                 - b_warehouse(1)(2)(0)#218 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -49
 WarehouseInventoryBalance(1,4,1): - I_warehouse(0)(2)(0)#38
                                 + I_warehouse(0)(3)(0)#48
                                 - b_warehouse(0)(3)(0)#168 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -89
 WarehouseInventoryBalance(2,4,1): - I_warehouse(1)(2)(0)#98
                                 + I_warehouse(1)(3)(0)#108
                                 - b_warehouse(1)(3)(0)#228 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -51
 WarehouseInventoryBalance(1,5,1): - I_warehouse(0)(3)(0)#48
                                 + I_warehouse(0)(4)(0)#58
                                 - b_warehouse(0)(4)(0)#178 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -99
 WarehouseInventoryBalance(2,5,1): - I_warehouse(1)(3)(0)#108
                                 + I_warehouse(1)(4)(0)#118
                                 - b_warehouse(1)(4)(0)#238 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -55
 WarehouseInventoryBalance(1,6,1): - I_warehouse(0)(4)(0)#58
                                 + I_warehouse(0)(5)(0)#68
                                 - b_warehouse(0)(5)(0)#188 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -104
 WarehouseInventoryBalance(2,6,1): - I_warehouse(1)(4)(0)#118
                                 + I_warehouse(1)(5)(0)#128
                                 - b_warehouse(1)(5)(0)#248 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -51
 WarehouseInventoryBalance(1,1,2): I_warehouse(0)(0)(1)#19
                                 - b_warehouse(0)(0)(1)#139 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -94
 WarehouseInventoryBalance(2,1,2): I_warehouse(1)(0)(1)#79
                                 - b_warehouse(1)(0)(1)#199 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -60
 WarehouseInventoryBalance(1,2,2): - I_warehouse(0)(0)(1)#19
                                 + I_warehouse(0)(1)(1)#29
                                 - b_warehouse(0)(1)(1)#149 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -100
 WarehouseInventoryBalance(2,2,2): - I_warehouse(1)(0)(1)#79
                                 + I_warehouse(1)(1)(1)#89
                                 - b_warehouse(1)(1)(1)#209 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -58
 WarehouseInventoryBalance(1,3,2): - I_warehouse(0)(1)(1)#29
                                 + I_warehouse(0)(2)(1)#39
                                 - b_warehouse(0)(2)(1)#159 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -102
 WarehouseInventoryBalance(2,3,2): - I_warehouse(1)(1)(1)#89
                                 + I_warehouse(1)(2)(1)#99
                                 - b_warehouse(1)(2)(1)#219 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -58
 WarehouseInventoryBalance(1,4,2): - I_warehouse(0)(2)(1)#39
                                 + I_warehouse(0)(3)(1)#49
                                 - b_warehouse(0)(3)(1)#169 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -103
 WarehouseInventoryBalance(2,4,2): - I_warehouse(1)(2)(1)#99
                                 + I_warehouse(1)(3)(1)#109
                                 - b_warehouse(1)(3)(1)#229 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -57
 WarehouseInventoryBalance(1,5,2): - I_warehouse(0)(3)(1)#49
                                 + I_warehouse(0)(4)(1)#59
                                 - b_warehouse(0)(4)(1)#179 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -105
 WarehouseInventoryBalance(2,5,2): - I_warehouse(1)(3)(1)#109
                                 + I_warehouse(1)(4)(1)#119
                                 - b_warehouse(1)(4)(1)#239 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -52
 WarehouseInventoryBalance(1,6,2): - I_warehouse(0)(4)(1)#59
                                 + I_warehouse(0)(5)(1)#69
                                 - b_warehouse(0)(5)(1)#189 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -95
 WarehouseInventoryBalance(2,6,2): - I_warehouse(1)(4)(1)#119
                                 + I_warehouse(1)(5)(1)#129
                                 - b_warehouse(1)(5)(1)#249 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -50
 WarehouseInventoryBalance(1,1,3): I_warehouse(0)(0)(2)#20
                                 - b_warehouse(0)(0)(2)#140 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -106
 WarehouseInventoryBalance(2,1,3): I_warehouse(1)(0)(2)#80
                                 - b_warehouse(1)(0)(2)#200 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -59
 WarehouseInventoryBalance(1,2,3): - I_warehouse(0)(0)(2)#20
                                 + I_warehouse(0)(1)(2)#30
                                 - b_warehouse(0)(1)(2)#150 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -98
 WarehouseInventoryBalance(2,2,3): - I_warehouse(1)(0)(2)#80
                                 + I_warehouse(1)(1)(2)#90
                                 - b_warehouse(1)(1)(2)#210 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -55
 WarehouseInventoryBalance(1,3,3): - I_warehouse(0)(1)(2)#30
                                 + I_warehouse(0)(2)(2)#40
                                 - b_warehouse(0)(2)(2)#160 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -92
 WarehouseInventoryBalance(2,3,3): - I_warehouse(1)(1)(2)#90
                                 + I_warehouse(1)(2)(2)#100
                                 - b_warehouse(1)(2)(2)#220 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -49
 WarehouseInventoryBalance(1,4,3): - I_warehouse(0)(2)(2)#40
                                 + I_warehouse(0)(3)(2)#50
                                 - b_warehouse(0)(3)(2)#170 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -98
 WarehouseInventoryBalance(2,4,3): - I_warehouse(1)(2)(2)#100
                                 + I_warehouse(1)(3)(2)#110
                                 - b_warehouse(1)(3)(2)#230 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -54
 WarehouseInventoryBalance(1,5,3): - I_warehouse(0)(3)(2)#50
                                 + I_warehouse(0)(4)(2)#60
                                 - b_warehouse(0)(4)(2)#180 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -91
 WarehouseInventoryBalance(2,5,3): - I_warehouse(1)(3)(2)#110
                                 + I_warehouse(1)(4)(2)#120
                                 - b_warehouse(1)(4)(2)#240 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -53
 WarehouseInventoryBalance(1,6,3): - I_warehouse(0)(4)(2)#60
                                 + I_warehouse(0)(5)(2)#70
                                 - b_warehouse(0)(5)(2)#190 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -101
 WarehouseInventoryBalance(2,6,3): - I_warehouse(1)(4)(2)#120
                                 + I_warehouse(1)(5)(2)#130
                                 - b_warehouse(1)(5)(2)#250 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -54
 WarehouseInventoryBalance(1,1,4): I_warehouse(0)(0)(3)#21
                                 - b_warehouse(0)(0)(3)#141 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -94
 WarehouseInventoryBalance(2,1,4): I_warehouse(1)(0)(3)#81
                                 - b_warehouse(1)(0)(3)#201 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -55
 WarehouseInventoryBalance(1,2,4): - I_warehouse(0)(0)(3)#21
                                 + I_warehouse(0)(1)(3)#31
                                 - b_warehouse(0)(1)(3)#151 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -97
 WarehouseInventoryBalance(2,2,4): - I_warehouse(1)(0)(3)#81
                                 + I_warehouse(1)(1)(3)#91
                                 - b_warehouse(1)(1)(3)#211 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -59
 WarehouseInventoryBalance(1,3,4): - I_warehouse(0)(1)(3)#31
                                 + I_warehouse(0)(2)(3)#41
                                 - b_warehouse(0)(2)(3)#161 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -106
 WarehouseInventoryBalance(2,3,4): - I_warehouse(1)(1)(3)#91
                                 + I_warehouse(1)(2)(3)#101
                                 - b_warehouse(1)(2)(3)#221 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -53
 WarehouseInventoryBalance(1,4,4): - I_warehouse(0)(2)(3)#41
                                 + I_warehouse(0)(3)(3)#51
                                 - b_warehouse(0)(3)(3)#171 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -92
 WarehouseInventoryBalance(2,4,4): - I_warehouse(1)(2)(3)#101
                                 + I_warehouse(1)(3)(3)#111
                                 - b_warehouse(1)(3)(3)#231 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -51
 WarehouseInventoryBalance(1,5,4): - I_warehouse(0)(3)(3)#51
                                 + I_warehouse(0)(4)(3)#61
                                 - b_warehouse(0)(4)(3)#181 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -95
 WarehouseInventoryBalance(2,5,4): - I_warehouse(1)(3)(3)#111
                                 + I_warehouse(1)(4)(3)#121
                                 - b_warehouse(1)(4)(3)#241 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -53
 WarehouseInventoryBalance(1,6,4): - I_warehouse(0)(4)(3)#61
                                 + I_warehouse(0)(5)(3)#71
                                 - b_warehouse(0)(5)(3)#191 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -104
 WarehouseInventoryBalance(2,6,4): - I_warehouse(1)(4)(3)#121
                                 + I_warehouse(1)(5)(3)#131
                                 - b_warehouse(1)(5)(3)#251 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -56
 WarehouseInventoryBalance(1,1,5): I_warehouse(0)(0)(4)#22
                                 - b_warehouse(0)(0)(4)#142 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -94
 WarehouseInventoryBalance(2,1,5): I_warehouse(1)(0)(4)#82
                                 - b_warehouse(1)(0)(4)#202 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -61
 WarehouseInventoryBalance(1,2,5): - I_warehouse(0)(0)(4)#22
                                 + I_warehouse(0)(1)(4)#32
                                 - b_warehouse(0)(1)(4)#152 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -100
 WarehouseInventoryBalance(2,2,5): - I_warehouse(1)(0)(4)#82
                                 + I_warehouse(1)(1)(4)#92
                                 - b_warehouse(1)(1)(4)#212 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -53
 WarehouseInventoryBalance(1,3,5): - I_warehouse(0)(1)(4)#32
                                 + I_warehouse(0)(2)(4)#42
                                 - b_warehouse(0)(2)(4)#162 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -98
 WarehouseInventoryBalance(2,3,5): - I_warehouse(1)(1)(4)#92
                                 + I_warehouse(1)(2)(4)#102
                                 - b_warehouse(1)(2)(4)#222 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -52
 WarehouseInventoryBalance(1,4,5): - I_warehouse(0)(2)(4)#42
                                 + I_warehouse(0)(3)(4)#52
                                 - b_warehouse(0)(3)(4)#172 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -90
 WarehouseInventoryBalance(2,4,5): - I_warehouse(1)(2)(4)#102
                                 + I_warehouse(1)(3)(4)#112
                                 - b_warehouse(1)(3)(4)#232 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -56
 WarehouseInventoryBalance(1,5,5): - I_warehouse(0)(3)(4)#52
                                 + I_warehouse(0)(4)(4)#62
                                 - b_warehouse(0)(4)(4)#182 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -95
 WarehouseInventoryBalance(2,5,5): - I_warehouse(1)(3)(4)#112
                                 + I_warehouse(1)(4)(4)#122
                                 - b_warehouse(1)(4)(4)#242 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -52
 WarehouseInventoryBalance(1,6,5): - I_warehouse(0)(4)(4)#62
                                 + I_warehouse(0)(5)(4)#72
                                 - b_warehouse(0)(5)(4)#192 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -104
 WarehouseInventoryBalance(2,6,5): - I_warehouse(1)(4)(4)#122
                                 + I_warehouse(1)(5)(4)#132
                                 - b_warehouse(1)(5)(4)#252 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -52
 WarehouseInventoryBalance(1,1,6): I_warehouse(0)(0)(5)#23
                                 - b_warehouse(0)(0)(5)#143 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -104
 WarehouseInventoryBalance(2,1,6): I_warehouse(1)(0)(5)#83
                                 - b_warehouse(1)(0)(5)#203 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -52
 WarehouseInventoryBalance(1,2,6): - I_warehouse(0)(0)(5)#23
                                 + I_warehouse(0)(1)(5)#33
                                 - b_warehouse(0)(1)(5)#153 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -93
 WarehouseInventoryBalance(2,2,6): - I_warehouse(1)(0)(5)#83
                                 + I_warehouse(1)(1)(5)#93
                                 - b_warehouse(1)(1)(5)#213 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -54
 WarehouseInventoryBalance(1,3,6): - I_warehouse(0)(1)(5)#33
                                 + I_warehouse(0)(2)(5)#43
                                 - b_warehouse(0)(2)(5)#163 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -95
 WarehouseInventoryBalance(2,3,6): - I_warehouse(1)(1)(5)#93
                                 + I_warehouse(1)(2)(5)#103
                                 - b_warehouse(1)(2)(5)#223 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -58
 WarehouseInventoryBalance(1,4,6): - I_warehouse(0)(2)(5)#43
                                 + I_warehouse(0)(3)(5)#53
                                 - b_warehouse(0)(3)(5)#173 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -92
 WarehouseInventoryBalance(2,4,6): - I_warehouse(1)(2)(5)#103
                                 + I_warehouse(1)(3)(5)#113
                                 - b_warehouse(1)(3)(5)#233 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -54
 WarehouseInventoryBalance(1,5,6): - I_warehouse(0)(3)(5)#53
                                 + I_warehouse(0)(4)(5)#63
                                 - b_warehouse(0)(4)(5)#183 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -104
 WarehouseInventoryBalance(2,5,6): - I_warehouse(1)(3)(5)#113
                                 + I_warehouse(1)(4)(5)#123
                                 - b_warehouse(1)(4)(5)#243 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -54
 WarehouseInventoryBalance(1,6,6): - I_warehouse(0)(4)(5)#63
                                 + I_warehouse(0)(5)(5)#73
                                 - b_warehouse(0)(5)(5)#193 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -101
 WarehouseInventoryBalance(2,6,6): - I_warehouse(1)(4)(5)#123
                                 + I_warehouse(1)(5)(5)#133
                                 - b_warehouse(1)(5)(5)#253 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -52
 WarehouseInventoryBalance(1,1,7): I_warehouse(0)(0)(6)#24
                                 - b_warehouse(0)(0)(6)#144 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -96
 WarehouseInventoryBalance(2,1,7): I_warehouse(1)(0)(6)#84
                                 - b_warehouse(1)(0)(6)#204 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -55
 WarehouseInventoryBalance(1,2,7): - I_warehouse(0)(0)(6)#24
                                 + I_warehouse(0)(1)(6)#34
                                 - b_warehouse(0)(1)(6)#154 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -92
 WarehouseInventoryBalance(2,2,7): - I_warehouse(1)(0)(6)#84
                                 + I_warehouse(1)(1)(6)#94
                                 - b_warehouse(1)(1)(6)#214 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -54
 WarehouseInventoryBalance(1,3,7): - I_warehouse(0)(1)(6)#34
                                 + I_warehouse(0)(2)(6)#44
                                 - b_warehouse(0)(2)(6)#164 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -104
 WarehouseInventoryBalance(2,3,7): - I_warehouse(1)(1)(6)#94
                                 + I_warehouse(1)(2)(6)#104
                                 - b_warehouse(1)(2)(6)#224 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -50
 WarehouseInventoryBalance(1,4,7): - I_warehouse(0)(2)(6)#44
                                 + I_warehouse(0)(3)(6)#54
                                 - b_warehouse(0)(3)(6)#174 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -88
 WarehouseInventoryBalance(2,4,7): - I_warehouse(1)(2)(6)#104
                                 + I_warehouse(1)(3)(6)#114
                                 - b_warehouse(1)(3)(6)#234 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -49
 WarehouseInventoryBalance(1,5,7): - I_warehouse(0)(3)(6)#54
                                 + I_warehouse(0)(4)(6)#64
                                 - b_warehouse(0)(4)(6)#184 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -95
 WarehouseInventoryBalance(2,5,7): - I_warehouse(1)(3)(6)#114
                                 + I_warehouse(1)(4)(6)#124
                                 - b_warehouse(1)(4)(6)#244 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -54
 WarehouseInventoryBalance(1,6,7): - I_warehouse(0)(4)(6)#64
                                 + I_warehouse(0)(5)(6)#74
                                 - b_warehouse(0)(5)(6)#194 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -101
 WarehouseInventoryBalance(2,6,7): - I_warehouse(1)(4)(6)#124
                                 + I_warehouse(1)(5)(6)#134
                                 - b_warehouse(1)(5)(6)#254 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -57
 WarehouseInventoryBalance(1,1,8): I_warehouse(0)(0)(7)#25
                                 - b_warehouse(0)(0)(7)#145 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -104
 WarehouseInventoryBalance(2,1,8): I_warehouse(1)(0)(7)#85
                                 - b_warehouse(1)(0)(7)#205 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -52
 WarehouseInventoryBalance(1,2,8): - I_warehouse(0)(0)(7)#25
                                 + I_warehouse(0)(1)(7)#35
                                 - b_warehouse(0)(1)(7)#155 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -100
 WarehouseInventoryBalance(2,2,8): - I_warehouse(1)(0)(7)#85
                                 + I_warehouse(1)(1)(7)#95
                                 - b_warehouse(1)(1)(7)#215 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -51
 WarehouseInventoryBalance(1,3,8): - I_warehouse(0)(1)(7)#35
                                 + I_warehouse(0)(2)(7)#45
                                 - b_warehouse(0)(2)(7)#165 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -104
 WarehouseInventoryBalance(2,3,8): - I_warehouse(1)(1)(7)#95
                                 + I_warehouse(1)(2)(7)#105
                                 - b_warehouse(1)(2)(7)#225 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -54
 WarehouseInventoryBalance(1,4,8): - I_warehouse(0)(2)(7)#45
                                 + I_warehouse(0)(3)(7)#55
                                 - b_warehouse(0)(3)(7)#175 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -101
 WarehouseInventoryBalance(2,4,8): - I_warehouse(1)(2)(7)#105
                                 + I_warehouse(1)(3)(7)#115
                                 - b_warehouse(1)(3)(7)#235 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -55
 WarehouseInventoryBalance(1,5,8): - I_warehouse(0)(3)(7)#55
                                 + I_warehouse(0)(4)(7)#65
                                 - b_warehouse(0)(4)(7)#185 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -92
 WarehouseInventoryBalance(2,5,8): - I_warehouse(1)(3)(7)#115
                                 + I_warehouse(1)(4)(7)#125
                                 - b_warehouse(1)(4)(7)#245 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -57
 WarehouseInventoryBalance(1,6,8): - I_warehouse(0)(4)(7)#65
                                 + I_warehouse(0)(5)(7)#75
                                 - b_warehouse(0)(5)(7)#195 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -103
 WarehouseInventoryBalance(2,6,8): - I_warehouse(1)(4)(7)#125
                                 + I_warehouse(1)(5)(7)#135
                                 - b_warehouse(1)(5)(7)#255 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -51
 WarehouseInventoryBalance(1,1,9): I_warehouse(0)(0)(8)#26
                                 - b_warehouse(0)(0)(8)#146 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -100
 WarehouseInventoryBalance(2,1,9): I_warehouse(1)(0)(8)#86
                                 - b_warehouse(1)(0)(8)#206 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -53
 WarehouseInventoryBalance(1,2,9): - I_warehouse(0)(0)(8)#26
                                 + I_warehouse(0)(1)(8)#36
                                 - b_warehouse(0)(1)(8)#156 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -92
 WarehouseInventoryBalance(2,2,9): - I_warehouse(1)(0)(8)#86
                                 + I_warehouse(1)(1)(8)#96
                                 - b_warehouse(1)(1)(8)#216 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -51
 WarehouseInventoryBalance(1,3,9): - I_warehouse(0)(1)(8)#36
                                 + I_warehouse(0)(2)(8)#46
                                 - b_warehouse(0)(2)(8)#166 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -90
 WarehouseInventoryBalance(2,3,9): - I_warehouse(1)(1)(8)#96
                                 + I_warehouse(1)(2)(8)#106
                                 - b_warehouse(1)(2)(8)#226 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -52
 WarehouseInventoryBalance(1,4,9): - I_warehouse(0)(2)(8)#46
                                 + I_warehouse(0)(3)(8)#56
                                 - b_warehouse(0)(3)(8)#176 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -93
 WarehouseInventoryBalance(2,4,9): - I_warehouse(1)(2)(8)#106
                                 + I_warehouse(1)(3)(8)#116
                                 - b_warehouse(1)(3)(8)#236 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -55
 WarehouseInventoryBalance(1,5,9): - I_warehouse(0)(3)(8)#56
                                 + I_warehouse(0)(4)(8)#66
                                 - b_warehouse(0)(4)(8)#186 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -101
 WarehouseInventoryBalance(2,5,9): - I_warehouse(1)(3)(8)#116
                                 + I_warehouse(1)(4)(8)#126
                                 - b_warehouse(1)(4)(8)#246 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -59
 WarehouseInventoryBalance(1,6,9): - I_warehouse(0)(4)(8)#66
                                 + I_warehouse(0)(5)(8)#76
                                 - b_warehouse(0)(5)(8)#196 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -102
 WarehouseInventoryBalance(2,6,9): - I_warehouse(1)(4)(8)#126
                                 + I_warehouse(1)(5)(8)#136
                                 - b_warehouse(1)(5)(8)#256 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -54
 WarehouseInventoryBalance(1,1,10): I_warehouse(0)(0)(9)#27
                                 - b_warehouse(0)(0)(9)#147 - q(0)(0)(0)#258
                                 - q(1)(0)(0)#270 - q(2)(0)(0)#282  = -92
 WarehouseInventoryBalance(2,1,10): I_warehouse(1)(0)(9)#87
                                 - b_warehouse(1)(0)(9)#207 - q(0)(1)(0)#264
                                 - q(1)(1)(0)#276 - q(2)(1)(0)#288  = -58
 WarehouseInventoryBalance(1,2,10): - I_warehouse(0)(0)(9)#27
                                 + I_warehouse(0)(1)(9)#37
                                 - b_warehouse(0)(1)(9)#157 - q(0)(0)(1)#259
                                 - q(1)(0)(1)#271 - q(2)(0)(1)#283  = -102
 WarehouseInventoryBalance(2,2,10): - I_warehouse(1)(0)(9)#87
                                 + I_warehouse(1)(1)(9)#97
                                 - b_warehouse(1)(1)(9)#217 - q(0)(1)(1)#265
                                 - q(1)(1)(1)#277 - q(2)(1)(1)#289  = -55
 WarehouseInventoryBalance(1,3,10): - I_warehouse(0)(1)(9)#37
                                 + I_warehouse(0)(2)(9)#47
                                 - b_warehouse(0)(2)(9)#167 - q(0)(0)(2)#260
                                 - q(1)(0)(2)#272 - q(2)(0)(2)#284  = -95
 WarehouseInventoryBalance(2,3,10): - I_warehouse(1)(1)(9)#97
                                 + I_warehouse(1)(2)(9)#107
                                 - b_warehouse(1)(2)(9)#227 - q(0)(1)(2)#266
                                 - q(1)(1)(2)#278 - q(2)(1)(2)#290  = -54
 WarehouseInventoryBalance(1,4,10): - I_warehouse(0)(2)(9)#47
                                 + I_warehouse(0)(3)(9)#57
                                 - b_warehouse(0)(3)(9)#177 - q(0)(0)(3)#261
                                 - q(1)(0)(3)#273 - q(2)(0)(3)#285  = -97
 WarehouseInventoryBalance(2,4,10): - I_warehouse(1)(2)(9)#107
                                 + I_warehouse(1)(3)(9)#117
                                 - b_warehouse(1)(3)(9)#237 - q(0)(1)(3)#267
                                 - q(1)(1)(3)#279 - q(2)(1)(3)#291  = -53
 WarehouseInventoryBalance(1,5,10): - I_warehouse(0)(3)(9)#57
                                 + I_warehouse(0)(4)(9)#67
                                 - b_warehouse(0)(4)(9)#187 - q(0)(0)(4)#262
                                 - q(1)(0)(4)#274 - q(2)(0)(4)#286  = -99
 WarehouseInventoryBalance(2,5,10): - I_warehouse(1)(3)(9)#117
                                 + I_warehouse(1)(4)(9)#127
                                 - b_warehouse(1)(4)(9)#247 - q(0)(1)(4)#268
                                 - q(1)(1)(4)#280 - q(2)(1)(4)#292  = -53
 WarehouseInventoryBalance(1,6,10): - I_warehouse(0)(4)(9)#67
                                 + I_warehouse(0)(5)(9)#77
                                 - b_warehouse(0)(5)(9)#197 - q(0)(0)(5)#263
                                 - q(1)(0)(5)#275 - q(2)(0)(5)#287  = -102
 WarehouseInventoryBalance(2,6,10): - I_warehouse(1)(4)(9)#127
                                 + I_warehouse(1)(5)(9)#137
                                 - b_warehouse(1)(5)(9)#257 - q(0)(1)(5)#269
                                 - q(1)(1)(5)#281 - q(2)(1)(5)#293  = -49
 WarehouseCoverage(1,1):         o(0)(0)#294 + o(2)(0)#306 <= 1
 WarehouseCoverage(2,1):         o(1)(0)#300 + o(2)(0)#306 <= 1
 WarehouseCoverage(1,2):         o(0)(1)#295 + o(2)(1)#307 <= 1
 WarehouseCoverage(2,2):         o(1)(1)#301 + o(2)(1)#307 <= 1
 WarehouseCoverage(1,3):         o(0)(2)#296 + o(2)(2)#308 <= 1
 WarehouseCoverage(2,3):         o(1)(2)#302 + o(2)(2)#308 <= 1
 WarehouseCoverage(1,4):         o(0)(3)#297 + o(2)(3)#309 <= 1
 WarehouseCoverage(2,4):         o(1)(3)#303 + o(2)(3)#309 <= 1
 WarehouseCoverage(1,5):         o(0)(4)#298 + o(2)(4)#310 <= 1
 WarehouseCoverage(2,5):         o(1)(4)#304 + o(2)(4)#310 <= 1
 WarehouseCoverage(1,6):         o(0)(5)#299 + o(2)(5)#311 <= 1
 WarehouseCoverage(2,6):         o(1)(5)#305 + o(2)(5)#311 <= 1
 VehicleCapacity(1,1):           q(0)(0)(0)#258 + q(0)(1)(0)#264
                                 - 102 o(0)(0)#294 <= 0
 VehicleCapacity(2,1):           q(1)(0)(0)#270 + q(1)(1)(0)#276
                                 - 102 o(1)(0)#300 <= 0
 VehicleCapacity(3,1):           q(2)(0)(0)#282 + q(2)(1)(0)#288
                                 - 102 o(2)(0)#306 <= 0
 VehicleCapacity(1,2):           q(0)(0)(1)#259 + q(0)(1)(1)#265
                                 - 102 o(0)(1)#295 <= 0
 VehicleCapacity(2,2):           q(1)(0)(1)#271 + q(1)(1)(1)#277
                                 - 102 o(1)(1)#301 <= 0
 VehicleCapacity(3,2):           q(2)(0)(1)#283 + q(2)(1)(1)#289
                                 - 102 o(2)(1)#307 <= 0
 VehicleCapacity(1,3):           q(0)(0)(2)#260 + q(0)(1)(2)#266
                                 - 102 o(0)(2)#296 <= 0
 VehicleCapacity(2,3):           q(1)(0)(2)#272 + q(1)(1)(2)#278
                                 - 102 o(1)(2)#302 <= 0
 VehicleCapacity(3,3):           q(2)(0)(2)#284 + q(2)(1)(2)#290
                                 - 102 o(2)(2)#308 <= 0
 VehicleCapacity(1,4):           q(0)(0)(3)#261 + q(0)(1)(3)#267
                                 - 102 o(0)(3)#297 <= 0
 VehicleCapacity(2,4):           q(1)(0)(3)#273 + q(1)(1)(3)#279
                                 - 102 o(1)(3)#303 <= 0
 VehicleCapacity(3,4):           q(2)(0)(3)#285 + q(2)(1)(3)#291
                                 - 102 o(2)(3)#309 <= 0
 VehicleCapacity(1,5):           q(0)(0)(4)#262 + q(0)(1)(4)#268
                                 - 102 o(0)(4)#298 <= 0
 VehicleCapacity(2,5):           q(1)(0)(4)#274 + q(1)(1)(4)#280
                                 - 102 o(1)(4)#304 <= 0
 VehicleCapacity(3,5):           q(2)(0)(4)#286 + q(2)(1)(4)#292
                                 - 102 o(2)(4)#310 <= 0
 VehicleCapacity(1,6):           q(0)(0)(5)#263 + q(0)(1)(5)#269
                                 - 102 o(0)(5)#299 <= 0
 VehicleCapacity(2,6):           q(1)(0)(5)#275 + q(1)(1)(5)#281
                                 - 102 o(1)(5)#305 <= 0
 VehicleCapacity(3,6):           q(2)(0)(5)#287 + q(2)(1)(5)#293
                                 - 102 o(2)(5)#311 <= 0
 FleetSize(1):                   o(0)(0)#294 + o(1)(0)#300 + o(2)(0)#306 <= 2
 FleetSize(2):                   o(0)(1)#295 + o(1)(1)#301 + o(2)(1)#307 <= 2
 FleetSize(3):                   o(0)(2)#296 + o(1)(2)#302 + o(2)(2)#308 <= 2
 FleetSize(4):                   o(0)(3)#297 + o(1)(3)#303 + o(2)(3)#309 <= 2
 FleetSize(5):                   o(0)(4)#298 + o(1)(4)#304 + o(2)(4)#310 <= 2
 FleetSize(6):                   o(0)(5)#299 + o(1)(5)#305 + o(2)(5)#311 <= 2
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
 0 <= I_warehouse(0)(0)(2)#20 <= 136.8
 0 <= I_warehouse(0)(0)(3)#21 <= 136.8
 0 <= I_warehouse(0)(0)(4)#22 <= 136.8
 0 <= I_warehouse(0)(0)(5)#23 <= 136.8
 0 <= I_warehouse(0)(0)(6)#24 <= 136.8
 0 <= I_warehouse(0)(0)(7)#25 <= 136.8
 0 <= I_warehouse(0)(0)(8)#26 <= 136.8
 0 <= I_warehouse(0)(0)(9)#27 <= 136.8
 0 <= I_warehouse(0)(1)(0)#28 <= 136.8
 0 <= I_warehouse(0)(1)(1)#29 <= 136.8
 0 <= I_warehouse(0)(1)(2)#30 <= 136.8
 0 <= I_warehouse(0)(1)(3)#31 <= 136.8
 0 <= I_warehouse(0)(1)(4)#32 <= 136.8
 0 <= I_warehouse(0)(1)(5)#33 <= 136.8
 0 <= I_warehouse(0)(1)(6)#34 <= 136.8
 0 <= I_warehouse(0)(1)(7)#35 <= 136.8
 0 <= I_warehouse(0)(1)(8)#36 <= 136.8
 0 <= I_warehouse(0)(1)(9)#37 <= 136.8
 0 <= I_warehouse(0)(2)(0)#38 <= 136.8
 0 <= I_warehouse(0)(2)(1)#39 <= 136.8
 0 <= I_warehouse(0)(2)(2)#40 <= 136.8
 0 <= I_warehouse(0)(2)(3)#41 <= 136.8
 0 <= I_warehouse(0)(2)(4)#42 <= 136.8
 0 <= I_warehouse(0)(2)(5)#43 <= 136.8
 0 <= I_warehouse(0)(2)(6)#44 <= 136.8
 0 <= I_warehouse(0)(2)(7)#45 <= 136.8
 0 <= I_warehouse(0)(2)(8)#46 <= 136.8
 0 <= I_warehouse(0)(2)(9)#47 <= 136.8
 0 <= I_warehouse(0)(3)(0)#48 <= 136.8
 0 <= I_warehouse(0)(3)(1)#49 <= 136.8
 0 <= I_warehouse(0)(3)(2)#50 <= 136.8
 0 <= I_warehouse(0)(3)(3)#51 <= 136.8
 0 <= I_warehouse(0)(3)(4)#52 <= 136.8
 0 <= I_warehouse(0)(3)(5)#53 <= 136.8
 0 <= I_warehouse(0)(3)(6)#54 <= 136.8
 0 <= I_warehouse(0)(3)(7)#55 <= 136.8
 0 <= I_warehouse(0)(3)(8)#56 <= 136.8
 0 <= I_warehouse(0)(3)(9)#57 <= 136.8
 0 <= I_warehouse(0)(4)(0)#58 <= 136.8
 0 <= I_warehouse(0)(4)(1)#59 <= 136.8
 0 <= I_warehouse(0)(4)(2)#60 <= 136.8
 0 <= I_warehouse(0)(4)(3)#61 <= 136.8
 0 <= I_warehouse(0)(4)(4)#62 <= 136.8
 0 <= I_warehouse(0)(4)(5)#63 <= 136.8
 0 <= I_warehouse(0)(4)(6)#64 <= 136.8
 0 <= I_warehouse(0)(4)(7)#65 <= 136.8
 0 <= I_warehouse(0)(4)(8)#66 <= 136.8
 0 <= I_warehouse(0)(4)(9)#67 <= 136.8
 0 <= I_warehouse(0)(5)(0)#68 <= 136.8
 0 <= I_warehouse(0)(5)(1)#69 <= 136.8
 0 <= I_warehouse(0)(5)(2)#70 <= 136.8
 0 <= I_warehouse(0)(5)(3)#71 <= 136.8
 0 <= I_warehouse(0)(5)(4)#72 <= 136.8
 0 <= I_warehouse(0)(5)(5)#73 <= 136.8
 0 <= I_warehouse(0)(5)(6)#74 <= 136.8
 0 <= I_warehouse(0)(5)(7)#75 <= 136.8
 0 <= I_warehouse(0)(5)(8)#76 <= 136.8
 0 <= I_warehouse(0)(5)(9)#77 <= 136.8
 0 <= I_warehouse(1)(0)(0)#78 <= 136.8
 0 <= I_warehouse(1)(0)(1)#79 <= 136.8
 0 <= I_warehouse(1)(0)(2)#80 <= 136.8
 0 <= I_warehouse(1)(0)(3)#81 <= 136.8
 0 <= I_warehouse(1)(0)(4)#82 <= 136.8
 0 <= I_warehouse(1)(0)(5)#83 <= 136.8
 0 <= I_warehouse(1)(0)(6)#84 <= 136.8
 0 <= I_warehouse(1)(0)(7)#85 <= 136.8
 0 <= I_warehouse(1)(0)(8)#86 <= 136.8
 0 <= I_warehouse(1)(0)(9)#87 <= 136.8
 0 <= I_warehouse(1)(1)(0)#88 <= 136.8
 0 <= I_warehouse(1)(1)(1)#89 <= 136.8
 0 <= I_warehouse(1)(1)(2)#90 <= 136.8
 0 <= I_warehouse(1)(1)(3)#91 <= 136.8
 0 <= I_warehouse(1)(1)(4)#92 <= 136.8
 0 <= I_warehouse(1)(1)(5)#93 <= 136.8
 0 <= I_warehouse(1)(1)(6)#94 <= 136.8
 0 <= I_warehouse(1)(1)(7)#95 <= 136.8
 0 <= I_warehouse(1)(1)(8)#96 <= 136.8
 0 <= I_warehouse(1)(1)(9)#97 <= 136.8
 0 <= I_warehouse(1)(2)(0)#98 <= 136.8
 0 <= I_warehouse(1)(2)(1)#99 <= 136.8
 0 <= I_warehouse(1)(2)(2)#100 <= 136.8
 0 <= I_warehouse(1)(2)(3)#101 <= 136.8
 0 <= I_warehouse(1)(2)(4)#102 <= 136.8
 0 <= I_warehouse(1)(2)(5)#103 <= 136.8
 0 <= I_warehouse(1)(2)(6)#104 <= 136.8
 0 <= I_warehouse(1)(2)(7)#105 <= 136.8
 0 <= I_warehouse(1)(2)(8)#106 <= 136.8
 0 <= I_warehouse(1)(2)(9)#107 <= 136.8
 0 <= I_warehouse(1)(3)(0)#108 <= 136.8
 0 <= I_warehouse(1)(3)(1)#109 <= 136.8
 0 <= I_warehouse(1)(3)(2)#110 <= 136.8
 0 <= I_warehouse(1)(3)(3)#111 <= 136.8
 0 <= I_warehouse(1)(3)(4)#112 <= 136.8
 0 <= I_warehouse(1)(3)(5)#113 <= 136.8
 0 <= I_warehouse(1)(3)(6)#114 <= 136.8
 0 <= I_warehouse(1)(3)(7)#115 <= 136.8
 0 <= I_warehouse(1)(3)(8)#116 <= 136.8
 0 <= I_warehouse(1)(3)(9)#117 <= 136.8
 0 <= I_warehouse(1)(4)(0)#118 <= 136.8
 0 <= I_warehouse(1)(4)(1)#119 <= 136.8
 0 <= I_warehouse(1)(4)(2)#120 <= 136.8
 0 <= I_warehouse(1)(4)(3)#121 <= 136.8
 0 <= I_warehouse(1)(4)(4)#122 <= 136.8
 0 <= I_warehouse(1)(4)(5)#123 <= 136.8
 0 <= I_warehouse(1)(4)(6)#124 <= 136.8
 0 <= I_warehouse(1)(4)(7)#125 <= 136.8
 0 <= I_warehouse(1)(4)(8)#126 <= 136.8
 0 <= I_warehouse(1)(4)(9)#127 <= 136.8
 0 <= I_warehouse(1)(5)(0)#128 <= 136.8
 0 <= I_warehouse(1)(5)(1)#129 <= 136.8
 0 <= I_warehouse(1)(5)(2)#130 <= 136.8
 0 <= I_warehouse(1)(5)(3)#131 <= 136.8
 0 <= I_warehouse(1)(5)(4)#132 <= 136.8
 0 <= I_warehouse(1)(5)(5)#133 <= 136.8
 0 <= I_warehouse(1)(5)(6)#134 <= 136.8
 0 <= I_warehouse(1)(5)(7)#135 <= 136.8
 0 <= I_warehouse(1)(5)(8)#136 <= 136.8
 0 <= I_warehouse(1)(5)(9)#137 <= 136.8
 0 <= b_warehouse(0)(0)(0)#138 <= 9
 0 <= b_warehouse(0)(0)(1)#139 <= 94
 0 <= b_warehouse(0)(0)(2)#140 <= 106
 0 <= b_warehouse(0)(0)(3)#141 <= 94
 0 <= b_warehouse(0)(0)(4)#142 <= 94
 0 <= b_warehouse(0)(0)(5)#143 <= 104
 0 <= b_warehouse(0)(0)(6)#144 <= 96
 0 <= b_warehouse(0)(0)(7)#145 <= 104
 0 <= b_warehouse(0)(0)(8)#146 <= 100
 0 <= b_warehouse(0)(0)(9)#147 <= 92
 0 <= b_warehouse(0)(1)(0)#148 <= 31
 0 <= b_warehouse(0)(1)(1)#149 <= 100
 0 <= b_warehouse(0)(1)(2)#150 <= 98
 0 <= b_warehouse(0)(1)(3)#151 <= 97
 0 <= b_warehouse(0)(1)(4)#152 <= 100
 0 <= b_warehouse(0)(1)(5)#153 <= 93
 0 <= b_warehouse(0)(1)(6)#154 <= 92
 0 <= b_warehouse(0)(1)(7)#155 <= 100
 0 <= b_warehouse(0)(1)(8)#156 <= 92
 0 <= b_warehouse(0)(1)(9)#157 <= 102
 0 <= b_warehouse(0)(2)(0)#158 <= 74
 0 <= b_warehouse(0)(2)(1)#159 <= 102
 0 <= b_warehouse(0)(2)(2)#160 <= 92
 0 <= b_warehouse(0)(2)(3)#161 <= 106
 0 <= b_warehouse(0)(2)(4)#162 <= 98
 0 <= b_warehouse(0)(2)(5)#163 <= 95
 0 <= b_warehouse(0)(2)(6)#164 <= 104
 0 <= b_warehouse(0)(2)(7)#165 <= 104
 0 <= b_warehouse(0)(2)(8)#166 <= 90
 0 <= b_warehouse(0)(2)(9)#167 <= 95
 0 <= b_warehouse(0)(3)(0)#168 <= 89
 0 <= b_warehouse(0)(3)(1)#169 <= 103
 0 <= b_warehouse(0)(3)(2)#170 <= 98
 0 <= b_warehouse(0)(3)(3)#171 <= 92
 0 <= b_warehouse(0)(3)(4)#172 <= 90
 0 <= b_warehouse(0)(3)(5)#173 <= 92
 0 <= b_warehouse(0)(3)(6)#174 <= 88
 0 <= b_warehouse(0)(3)(7)#175 <= 101
 0 <= b_warehouse(0)(3)(8)#176 <= 93
 0 <= b_warehouse(0)(3)(9)#177 <= 97
 0 <= b_warehouse(0)(4)(0)#178 <= 99
 0 <= b_warehouse(0)(4)(1)#179 <= 105
 0 <= b_warehouse(0)(4)(2)#180 <= 91
 0 <= b_warehouse(0)(4)(3)#181 <= 95
 0 <= b_warehouse(0)(4)(4)#182 <= 95
 0 <= b_warehouse(0)(4)(5)#183 <= 104
 0 <= b_warehouse(0)(4)(6)#184 <= 95
 0 <= b_warehouse(0)(4)(7)#185 <= 92
 0 <= b_warehouse(0)(4)(8)#186 <= 101
 0 <= b_warehouse(0)(4)(9)#187 <= 99
 0 <= b_warehouse(0)(5)(0)#188 <= 104
 0 <= b_warehouse(0)(5)(1)#189 <= 95
 0 <= b_warehouse(0)(5)(2)#190 <= 101
 0 <= b_warehouse(0)(5)(3)#191 <= 104
 0 <= b_warehouse(0)(5)(4)#192 <= 104
 0 <= b_warehouse(0)(5)(5)#193 <= 101
 0 <= b_warehouse(0)(5)(6)#194 <= 101
 0 <= b_warehouse(0)(5)(7)#195 <= 103
 0 <= b_warehouse(0)(5)(8)#196 <= 102
 0 <= b_warehouse(0)(5)(9)#197 <= 102
 0 <= b_warehouse(1)(0)(0)#198 <= 10
 0 <= b_warehouse(1)(0)(1)#199 <= 60
 0 <= b_warehouse(1)(0)(2)#200 <= 59
 0 <= b_warehouse(1)(0)(3)#201 <= 55
 0 <= b_warehouse(1)(0)(4)#202 <= 61
 0 <= b_warehouse(1)(0)(5)#203 <= 52
 0 <= b_warehouse(1)(0)(6)#204 <= 55
 0 <= b_warehouse(1)(0)(7)#205 <= 52
 0 <= b_warehouse(1)(0)(8)#206 <= 53
 0 <= b_warehouse(1)(0)(9)#207 <= 58
 0 <= b_warehouse(1)(1)(0)#208 <= 36
 0 <= b_warehouse(1)(1)(1)#209 <= 58
 0 <= b_warehouse(1)(1)(2)#210 <= 55
 0 <= b_warehouse(1)(1)(3)#211 <= 59
 0 <= b_warehouse(1)(1)(4)#212 <= 53
 0 <= b_warehouse(1)(1)(5)#213 <= 54
 0 <= b_warehouse(1)(1)(6)#214 <= 54
 0 <= b_warehouse(1)(1)(7)#215 <= 51
 0 <= b_warehouse(1)(1)(8)#216 <= 51
 0 <= b_warehouse(1)(1)(9)#217 <= 55
 0 <= b_warehouse(1)(2)(0)#218 <= 49
 0 <= b_warehouse(1)(2)(1)#219 <= 58
 0 <= b_warehouse(1)(2)(2)#220 <= 49
 0 <= b_warehouse(1)(2)(3)#221 <= 53
 0 <= b_warehouse(1)(2)(4)#222 <= 52
 0 <= b_warehouse(1)(2)(5)#223 <= 58
 0 <= b_warehouse(1)(2)(6)#224 <= 50
 0 <= b_warehouse(1)(2)(7)#225 <= 54
 0 <= b_warehouse(1)(2)(8)#226 <= 52
 0 <= b_warehouse(1)(2)(9)#227 <= 54
 0 <= b_warehouse(1)(3)(0)#228 <= 51
 0 <= b_warehouse(1)(3)(1)#229 <= 57
 0 <= b_warehouse(1)(3)(2)#230 <= 54
 0 <= b_warehouse(1)(3)(3)#231 <= 51
 0 <= b_warehouse(1)(3)(4)#232 <= 56
 0 <= b_warehouse(1)(3)(5)#233 <= 54
 0 <= b_warehouse(1)(3)(6)#234 <= 49
 0 <= b_warehouse(1)(3)(7)#235 <= 55
 0 <= b_warehouse(1)(3)(8)#236 <= 55
 0 <= b_warehouse(1)(3)(9)#237 <= 53
 0 <= b_warehouse(1)(4)(0)#238 <= 55
 0 <= b_warehouse(1)(4)(1)#239 <= 52
 0 <= b_warehouse(1)(4)(2)#240 <= 53
 0 <= b_warehouse(1)(4)(3)#241 <= 53
 0 <= b_warehouse(1)(4)(4)#242 <= 52
 0 <= b_warehouse(1)(4)(5)#243 <= 54
 0 <= b_warehouse(1)(4)(6)#244 <= 54
 0 <= b_warehouse(1)(4)(7)#245 <= 57
 0 <= b_warehouse(1)(4)(8)#246 <= 59
 0 <= b_warehouse(1)(4)(9)#247 <= 53
 0 <= b_warehouse(1)(5)(0)#248 <= 51
 0 <= b_warehouse(1)(5)(1)#249 <= 50
 0 <= b_warehouse(1)(5)(2)#250 <= 54
 0 <= b_warehouse(1)(5)(3)#251 <= 56
 0 <= b_warehouse(1)(5)(4)#252 <= 52
 0 <= b_warehouse(1)(5)(5)#253 <= 52
 0 <= b_warehouse(1)(5)(6)#254 <= 57
 0 <= b_warehouse(1)(5)(7)#255 <= 51
 0 <= b_warehouse(1)(5)(8)#256 <= 54
 0 <= b_warehouse(1)(5)(9)#257 <= 49
 0 <= q(0)(0)(0)#258 <= 136.8
 0 <= q(0)(0)(1)#259 <= 136.8
 0 <= q(0)(0)(2)#260 <= 136.8
 0 <= q(0)(0)(3)#261 <= 136.8
 0 <= q(0)(0)(4)#262 <= 136.8
 0 <= q(0)(0)(5)#263 <= 136.8
      q(0)(1)(0)#264 = 0
      q(0)(1)(1)#265 = 0
      q(0)(1)(2)#266 = 0
      q(0)(1)(3)#267 = 0
      q(0)(1)(4)#268 = 0
      q(0)(1)(5)#269 = 0
      q(1)(0)(0)#270 = 0
      q(1)(0)(1)#271 = 0
      q(1)(0)(2)#272 = 0
      q(1)(0)(3)#273 = 0
      q(1)(0)(4)#274 = 0
      q(1)(0)(5)#275 = 0
 0 <= q(1)(1)(0)#276 <= 136.8
 0 <= q(1)(1)(1)#277 <= 136.8
 0 <= q(1)(1)(2)#278 <= 136.8
 0 <= q(1)(1)(3)#279 <= 136.8
 0 <= q(1)(1)(4)#280 <= 136.8
 0 <= q(1)(1)(5)#281 <= 136.8
 0 <= q(2)(0)(0)#282 <= 136.8
 0 <= q(2)(0)(1)#283 <= 136.8
 0 <= q(2)(0)(2)#284 <= 136.8
 0 <= q(2)(0)(3)#285 <= 136.8
 0 <= q(2)(0)(4)#286 <= 136.8
 0 <= q(2)(0)(5)#287 <= 136.8
 0 <= q(2)(1)(0)#288 <= 136.8
 0 <= q(2)(1)(1)#289 <= 136.8
 0 <= q(2)(1)(2)#290 <= 136.8
 0 <= q(2)(1)(3)#291 <= 136.8
 0 <= q(2)(1)(4)#292 <= 136.8
 0 <= q(2)(1)(5)#293 <= 136.8
 0 <= o(0)(0)#294 <= 1
 0 <= o(0)(1)#295 <= 1
 0 <= o(0)(2)#296 <= 1
 0 <= o(0)(3)#297 <= 1
 0 <= o(0)(4)#298 <= 1
 0 <= o(0)(5)#299 <= 1
 0 <= o(1)(0)#300 <= 1
 0 <= o(1)(1)#301 <= 1
 0 <= o(1)(2)#302 <= 1
 0 <= o(1)(3)#303 <= 1
 0 <= o(1)(4)#304 <= 1
 0 <= o(1)(5)#305 <= 1
 0 <= o(2)(0)#306 <= 1
 0 <= o(2)(1)#307 <= 1
 0 <= o(2)(2)#308 <= 1
 0 <= o(2)(3)#309 <= 1
 0 <= o(2)(4)#310 <= 1
 0 <= o(2)(5)#311 <= 1
Binaries
 y(0)#0  y(1)#1  y(2)#2  y(3)#3  y(4)#4  y(5)#5  o(0)(0)#294  o(0)(1)#295 
 o(0)(2)#296  o(0)(3)#297  o(0)(4)#298  o(0)(5)#299  o(1)(0)#300  o(1)(1)#301 
 o(1)(2)#302  o(1)(3)#303  o(1)(4)#304  o(1)(5)#305  o(2)(0)#306  o(2)(1)#307 
 o(2)(2)#308  o(2)(3)#309  o(2)(4)#310  o(2)(5)#311 
End
