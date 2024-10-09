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
       + 0.5 I_warehouse(2)(0)(0)#69 + 0.5 I_warehouse(2)(0)(1)#70
       + 0.5 I_warehouse(2)(0)(2)#71 + 0.5 I_warehouse(2)(0)(3)#72
       + 0.5 I_warehouse(2)(0)(4)#73 + 0.5 I_warehouse(2)(0)(5)#74
       + 0.5 I_warehouse(2)(0)(6)#75 + 0.5 I_warehouse(2)(0)(7)#76
       + 0.5 I_warehouse(2)(0)(8)#77 + 0.5 I_warehouse(2)(0)(9)#78
       + 0.5 I_warehouse(2)(1)(0)#79 + 0.5 I_warehouse(2)(1)(1)#80
       + 0.5 I_warehouse(2)(1)(2)#81 + 0.5 I_warehouse(2)(1)(3)#82
       + 0.5 I_warehouse(2)(1)(4)#83 + 0.5 I_warehouse(2)(1)(5)#84
       + 0.5 I_warehouse(2)(1)(6)#85 + 0.5 I_warehouse(2)(1)(7)#86
       + 0.5 I_warehouse(2)(1)(8)#87 + 0.5 I_warehouse(2)(1)(9)#88
       + 0.5 I_warehouse(2)(2)(0)#89 + 0.5 I_warehouse(2)(2)(1)#90
       + 0.5 I_warehouse(2)(2)(2)#91 + 0.5 I_warehouse(2)(2)(3)#92
       + 0.5 I_warehouse(2)(2)(4)#93 + 0.5 I_warehouse(2)(2)(5)#94
       + 0.5 I_warehouse(2)(2)(6)#95 + 0.5 I_warehouse(2)(2)(7)#96
       + 0.5 I_warehouse(2)(2)(8)#97 + 0.5 I_warehouse(2)(2)(9)#98
       + 0.6 I_warehouse(3)(0)(0)#99 + 0.6 I_warehouse(3)(0)(1)#100
       + 0.6 I_warehouse(3)(0)(2)#101 + 0.6 I_warehouse(3)(0)(3)#102
       + 0.6 I_warehouse(3)(0)(4)#103 + 0.6 I_warehouse(3)(0)(5)#104
       + 0.6 I_warehouse(3)(0)(6)#105 + 0.6 I_warehouse(3)(0)(7)#106
       + 0.6 I_warehouse(3)(0)(8)#107 + 0.6 I_warehouse(3)(0)(9)#108
       + 0.6 I_warehouse(3)(1)(0)#109 + 0.6 I_warehouse(3)(1)(1)#110
       + 0.6 I_warehouse(3)(1)(2)#111 + 0.6 I_warehouse(3)(1)(3)#112
       + 0.6 I_warehouse(3)(1)(4)#113 + 0.6 I_warehouse(3)(1)(5)#114
       + 0.6 I_warehouse(3)(1)(6)#115 + 0.6 I_warehouse(3)(1)(7)#116
       + 0.6 I_warehouse(3)(1)(8)#117 + 0.6 I_warehouse(3)(1)(9)#118
       + 0.6 I_warehouse(3)(2)(0)#119 + 0.6 I_warehouse(3)(2)(1)#120
       + 0.6 I_warehouse(3)(2)(2)#121 + 0.6 I_warehouse(3)(2)(3)#122
       + 0.6 I_warehouse(3)(2)(4)#123 + 0.6 I_warehouse(3)(2)(5)#124
       + 0.6 I_warehouse(3)(2)(6)#125 + 0.6 I_warehouse(3)(2)(7)#126
       + 0.6 I_warehouse(3)(2)(8)#127 + 0.6 I_warehouse(3)(2)(9)#128
       + 0.5 I_warehouse(4)(0)(0)#129 + 0.5 I_warehouse(4)(0)(1)#130
       + 0.5 I_warehouse(4)(0)(2)#131 + 0.5 I_warehouse(4)(0)(3)#132
       + 0.5 I_warehouse(4)(0)(4)#133 + 0.5 I_warehouse(4)(0)(5)#134
       + 0.5 I_warehouse(4)(0)(6)#135 + 0.5 I_warehouse(4)(0)(7)#136
       + 0.5 I_warehouse(4)(0)(8)#137 + 0.5 I_warehouse(4)(0)(9)#138
       + 0.5 I_warehouse(4)(1)(0)#139 + 0.5 I_warehouse(4)(1)(1)#140
       + 0.5 I_warehouse(4)(1)(2)#141 + 0.5 I_warehouse(4)(1)(3)#142
       + 0.5 I_warehouse(4)(1)(4)#143 + 0.5 I_warehouse(4)(1)(5)#144
       + 0.5 I_warehouse(4)(1)(6)#145 + 0.5 I_warehouse(4)(1)(7)#146
       + 0.5 I_warehouse(4)(1)(8)#147 + 0.5 I_warehouse(4)(1)(9)#148
       + 0.5 I_warehouse(4)(2)(0)#149 + 0.5 I_warehouse(4)(2)(1)#150
       + 0.5 I_warehouse(4)(2)(2)#151 + 0.5 I_warehouse(4)(2)(3)#152
       + 0.5 I_warehouse(4)(2)(4)#153 + 0.5 I_warehouse(4)(2)(5)#154
       + 0.5 I_warehouse(4)(2)(6)#155 + 0.5 I_warehouse(4)(2)(7)#156
       + 0.5 I_warehouse(4)(2)(8)#157 + 0.5 I_warehouse(4)(2)(9)#158
       + 254 b_warehouse(0)(0)(0)#159 + 254 b_warehouse(0)(0)(1)#160
       + 254 b_warehouse(0)(0)(2)#161 + 254 b_warehouse(0)(0)(3)#162
       + 254 b_warehouse(0)(0)(4)#163 + 254 b_warehouse(0)(0)(5)#164
       + 254 b_warehouse(0)(0)(6)#165 + 254 b_warehouse(0)(0)(7)#166
       + 254 b_warehouse(0)(0)(8)#167 + 254 b_warehouse(0)(0)(9)#168
       + 254 b_warehouse(0)(1)(0)#169 + 254 b_warehouse(0)(1)(1)#170
       + 254 b_warehouse(0)(1)(2)#171 + 254 b_warehouse(0)(1)(3)#172
       + 254 b_warehouse(0)(1)(4)#173 + 254 b_warehouse(0)(1)(5)#174
       + 254 b_warehouse(0)(1)(6)#175 + 254 b_warehouse(0)(1)(7)#176
       + 254 b_warehouse(0)(1)(8)#177 + 254 b_warehouse(0)(1)(9)#178
       + 254 b_warehouse(0)(2)(0)#179 + 254 b_warehouse(0)(2)(1)#180
       + 254 b_warehouse(0)(2)(2)#181 + 254 b_warehouse(0)(2)(3)#182
       + 254 b_warehouse(0)(2)(4)#183 + 254 b_warehouse(0)(2)(5)#184
       + 254 b_warehouse(0)(2)(6)#185 + 254 b_warehouse(0)(2)(7)#186
       + 254 b_warehouse(0)(2)(8)#187 + 254 b_warehouse(0)(2)(9)#188
       + 171 b_warehouse(1)(0)(0)#189 + 171 b_warehouse(1)(0)(1)#190
       + 171 b_warehouse(1)(0)(2)#191 + 171 b_warehouse(1)(0)(3)#192
       + 171 b_warehouse(1)(0)(4)#193 + 171 b_warehouse(1)(0)(5)#194
       + 171 b_warehouse(1)(0)(6)#195 + 171 b_warehouse(1)(0)(7)#196
       + 171 b_warehouse(1)(0)(8)#197 + 171 b_warehouse(1)(0)(9)#198
       + 171 b_warehouse(1)(1)(0)#199 + 171 b_warehouse(1)(1)(1)#200
       + 171 b_warehouse(1)(1)(2)#201 + 171 b_warehouse(1)(1)(3)#202
       + 171 b_warehouse(1)(1)(4)#203 + 171 b_warehouse(1)(1)(5)#204
       + 171 b_warehouse(1)(1)(6)#205 + 171 b_warehouse(1)(1)(7)#206
       + 171 b_warehouse(1)(1)(8)#207 + 171 b_warehouse(1)(1)(9)#208
       + 171 b_warehouse(1)(2)(0)#209 + 171 b_warehouse(1)(2)(1)#210
       + 171 b_warehouse(1)(2)(2)#211 + 171 b_warehouse(1)(2)(3)#212
       + 171 b_warehouse(1)(2)(4)#213 + 171 b_warehouse(1)(2)(5)#214
       + 171 b_warehouse(1)(2)(6)#215 + 171 b_warehouse(1)(2)(7)#216
       + 171 b_warehouse(1)(2)(8)#217 + 171 b_warehouse(1)(2)(9)#218
       + 311.666666666667 b_warehouse(2)(0)(0)#219
       + 311.666666666667 b_warehouse(2)(0)(1)#220
       + 311.666666666667 b_warehouse(2)(0)(2)#221
       + 311.666666666667 b_warehouse(2)(0)(3)#222
       + 311.666666666667 b_warehouse(2)(0)(4)#223
       + 311.666666666667 b_warehouse(2)(0)(5)#224
       + 311.666666666667 b_warehouse(2)(0)(6)#225
       + 311.666666666667 b_warehouse(2)(0)(7)#226
       + 311.666666666667 b_warehouse(2)(0)(8)#227
       + 311.666666666667 b_warehouse(2)(0)(9)#228
       + 311.666666666667 b_warehouse(2)(1)(0)#229
       + 311.666666666667 b_warehouse(2)(1)(1)#230
       + 311.666666666667 b_warehouse(2)(1)(2)#231
       + 311.666666666667 b_warehouse(2)(1)(3)#232
       + 311.666666666667 b_warehouse(2)(1)(4)#233
       + 311.666666666667 b_warehouse(2)(1)(5)#234
       + 311.666666666667 b_warehouse(2)(1)(6)#235
       + 311.666666666667 b_warehouse(2)(1)(7)#236
       + 311.666666666667 b_warehouse(2)(1)(8)#237
       + 311.666666666667 b_warehouse(2)(1)(9)#238
       + 311.666666666667 b_warehouse(2)(2)(0)#239
       + 311.666666666667 b_warehouse(2)(2)(1)#240
       + 311.666666666667 b_warehouse(2)(2)(2)#241
       + 311.666666666667 b_warehouse(2)(2)(3)#242
       + 311.666666666667 b_warehouse(2)(2)(4)#243
       + 311.666666666667 b_warehouse(2)(2)(5)#244
       + 311.666666666667 b_warehouse(2)(2)(6)#245
       + 311.666666666667 b_warehouse(2)(2)(7)#246
       + 311.666666666667 b_warehouse(2)(2)(8)#247
       + 311.666666666667 b_warehouse(2)(2)(9)#248
       + 301.5 b_warehouse(3)(0)(0)#249 + 301.5 b_warehouse(3)(0)(1)#250
       + 301.5 b_warehouse(3)(0)(2)#251 + 301.5 b_warehouse(3)(0)(3)#252
       + 301.5 b_warehouse(3)(0)(4)#253 + 301.5 b_warehouse(3)(0)(5)#254
       + 301.5 b_warehouse(3)(0)(6)#255 + 301.5 b_warehouse(3)(0)(7)#256
       + 301.5 b_warehouse(3)(0)(8)#257 + 301.5 b_warehouse(3)(0)(9)#258
       + 301.5 b_warehouse(3)(1)(0)#259 + 301.5 b_warehouse(3)(1)(1)#260
       + 301.5 b_warehouse(3)(1)(2)#261 + 301.5 b_warehouse(3)(1)(3)#262
       + 301.5 b_warehouse(3)(1)(4)#263 + 301.5 b_warehouse(3)(1)(5)#264
       + 301.5 b_warehouse(3)(1)(6)#265 + 301.5 b_warehouse(3)(1)(7)#266
       + 301.5 b_warehouse(3)(1)(8)#267 + 301.5 b_warehouse(3)(1)(9)#268
       + 301.5 b_warehouse(3)(2)(0)#269 + 301.5 b_warehouse(3)(2)(1)#270
       + 301.5 b_warehouse(3)(2)(2)#271 + 301.5 b_warehouse(3)(2)(3)#272
       + 301.5 b_warehouse(3)(2)(4)#273 + 301.5 b_warehouse(3)(2)(5)#274
       + 301.5 b_warehouse(3)(2)(6)#275 + 301.5 b_warehouse(3)(2)(7)#276
       + 301.5 b_warehouse(3)(2)(8)#277 + 301.5 b_warehouse(3)(2)(9)#278
       + 375 b_warehouse(4)(0)(0)#279 + 375 b_warehouse(4)(0)(1)#280
       + 375 b_warehouse(4)(0)(2)#281 + 375 b_warehouse(4)(0)(3)#282
       + 375 b_warehouse(4)(0)(4)#283 + 375 b_warehouse(4)(0)(5)#284
       + 375 b_warehouse(4)(0)(6)#285 + 375 b_warehouse(4)(0)(7)#286
       + 375 b_warehouse(4)(0)(8)#287 + 375 b_warehouse(4)(0)(9)#288
       + 375 b_warehouse(4)(1)(0)#289 + 375 b_warehouse(4)(1)(1)#290
       + 375 b_warehouse(4)(1)(2)#291 + 375 b_warehouse(4)(1)(3)#292
       + 375 b_warehouse(4)(1)(4)#293 + 375 b_warehouse(4)(1)(5)#294
       + 375 b_warehouse(4)(1)(6)#295 + 375 b_warehouse(4)(1)(7)#296
       + 375 b_warehouse(4)(1)(8)#297 + 375 b_warehouse(4)(1)(9)#298
       + 375 b_warehouse(4)(2)(0)#299 + 375 b_warehouse(4)(2)(1)#300
       + 375 b_warehouse(4)(2)(2)#301 + 375 b_warehouse(4)(2)(3)#302
       + 375 b_warehouse(4)(2)(4)#303 + 375 b_warehouse(4)(2)(5)#304
       + 375 b_warehouse(4)(2)(6)#305 + 375 b_warehouse(4)(2)(7)#306
       + 375 b_warehouse(4)(2)(8)#307 + 375 b_warehouse(4)(2)(9)#308
       + 2838.94205647104 o(0)(0)#774 + 2838.94205647104 o(0)(1)#775
       + 2838.94205647104 o(0)(2)#776 + 1306.88790644033 o(1)(0)#777
       + 1306.88790644033 o(1)(1)#778 + 1306.88790644033 o(1)(2)#779
       + 3393.02444661378 o(2)(0)#780 + 3393.02444661378 o(2)(1)#781
       + 3393.02444661378 o(2)(2)#782 + 3753.07553880814 o(3)(0)#783
       + 3753.07553880814 o(3)(1)#784 + 3753.07553880814 o(3)(2)#785
       + 4049.31086188739 o(4)(0)#786 + 4049.31086188739 o(4)(1)#787
       + 4049.31086188739 o(4)(2)#788 + 4506.22183961407 o(5)(0)#789
       + 4506.22183961407 o(5)(1)#790 + 4506.22183961407 o(5)(2)#791
       + 4603.39325203014 o(6)(0)#792 + 4603.39325203014 o(6)(1)#793
       + 4603.39325203014 o(6)(2)#794 + 2141.16416932472 o(7)(0)#795
       + 2141.16416932472 o(7)(1)#796 + 2141.16416932472 o(7)(2)#797
       + 3739.86109815001 o(8)(0)#798 + 3739.86109815001 o(8)(1)#799
       + 3739.86109815001 o(8)(2)#800 + 3272.50432316779 o(9)(0)#801
       + 3272.50432316779 o(9)(1)#802 + 3272.50432316779 o(9)(2)#803
       + 4293.94348829276 o(10)(0)#804 + 4293.94348829276 o(10)(1)#805
       + 4293.94348829276 o(10)(2)#806 + 4156.88842697614 o(11)(0)#807
       + 4156.88842697614 o(11)(1)#808 + 4156.88842697614 o(11)(2)#809
       + 4453.12375005539 o(12)(0)#810 + 4453.12375005539 o(12)(1)#811
       + 4453.12375005539 o(12)(2)#812 + 4910.03472778207 o(13)(0)#813
       + 4910.03472778207 o(13)(1)#814 + 4910.03472778207 o(13)(2)#815
       + 5007.20614019814 o(14)(0)#816 + 5007.20614019814 o(14)(1)#817
       + 5007.20614019814 o(14)(2)#818 + 1739.25271309261 o(15)(0)#819
       + 1739.25271309261 o(15)(1)#820 + 1739.25271309261 o(15)(2)#821
       + 4037.75629949332 o(16)(0)#822 + 4037.75629949332 o(16)(1)#823
       + 4037.75629949332 o(16)(2)#824 + 1955.97792115717 o(17)(0)#825
       + 1955.97792115717 o(17)(1)#826 + 1955.97792115717 o(17)(2)#827
       + 4042.11446133062 o(18)(0)#828 + 4042.11446133062 o(18)(1)#829
       + 4042.11446133062 o(18)(2)#830 + 5154.53785110185 o(19)(0)#831
       + 5154.53785110185 o(19)(1)#832 + 5154.53785110185 o(19)(2)#833
       + 5248.12510490968 o(20)(0)#834 + 5248.12510490968 o(20)(1)#835
       + 5248.12510490968 o(20)(2)#836 + 5155.31185433091 o(21)(0)#837
       + 5155.31185433091 o(21)(1)#838 + 5155.31185433091 o(21)(2)#839
       + 5252.48326674697 o(22)(0)#840 + 5252.48326674697 o(22)(1)#841
       + 5252.48326674697 o(22)(2)#842 + 3834.65189510218 o(23)(0)#843
       + 3834.65189510218 o(23)(1)#844 + 3834.65189510218 o(23)(2)#845
       + 4938.6753411723 o(24)(0)#846 + 4938.6753411723 o(24)(1)#847
       + 4938.6753411723 o(24)(2)#848 + 3921.59433788463 o(25)(0)#849
       + 3921.59433788463 o(25)(1)#850 + 3921.59433788463 o(25)(2)#851
       + 4943.03350300959 o(26)(0)#852 + 4943.03350300959 o(26)(1)#853
       + 4943.03350300959 o(26)(2)#854 + 5558.35073926985 o(27)(0)#855
       + 5558.35073926985 o(27)(1)#856 + 5558.35073926985 o(27)(2)#857
       + 5651.93799307768 o(28)(0)#858 + 5651.93799307768 o(28)(1)#859
       + 5651.93799307768 o(28)(2)#860 + 5559.12474249891 o(29)(0)#861
       + 5559.12474249891 o(29)(1)#862 + 5559.12474249891 o(29)(2)#863
       + 5656.29615491497 o(30)(0)#864 + 5656.29615491497 o(30)(1)#865
       + 5656.29615491497 o(30)(2)#866
Subject To
 ProductionCapacity(1):          - 944 y(0)#0 + p(0)#3 <= 0
 ProductionCapacity(2):          - 944 y(1)#1 + p(1)#4 <= 0
 ProductionCapacity(3):          - 944 y(2)#2 + p(2)#5 <= 0
 PlantInventoryCapacity(1):      I_plant(0)#6 <= 472
 PlantInventoryCapacity(2):      I_plant(1)#7 <= 472
 PlantInventoryCapacity(3):      I_plant(2)#8 <= 472
 PlantInventoryBalance(1):       - p(0)#3 + I_plant(0)#6 + q(0)(0)(0)#309
                                 + q(0)(1)(0)#312 + q(0)(2)(0)#315
                                 + q(0)(3)(0)#318 + q(0)(4)(0)#321
                                 + q(1)(0)(0)#324 + q(1)(1)(0)#327
                                 + q(1)(2)(0)#330 + q(1)(3)(0)#333
                                 + q(1)(4)(0)#336 + q(2)(0)(0)#339
                                 + q(2)(1)(0)#342 + q(2)(2)(0)#345
                                 + q(2)(3)(0)#348 + q(2)(4)(0)#351
                                 + q(3)(0)(0)#354 + q(3)(1)(0)#357
                                 + q(3)(2)(0)#360 + q(3)(3)(0)#363
                                 + q(3)(4)(0)#366 + q(4)(0)(0)#369
                                 + q(4)(1)(0)#372 + q(4)(2)(0)#375
                                 + q(4)(3)(0)#378 + q(4)(4)(0)#381
                                 + q(5)(0)(0)#384 + q(5)(1)(0)#387
                                 + q(5)(2)(0)#390 + q(5)(3)(0)#393
                                 + q(5)(4)(0)#396 + q(6)(0)(0)#399
                                 + q(6)(1)(0)#402 + q(6)(2)(0)#405
                                 + q(6)(3)(0)#408 + q(6)(4)(0)#411
                                 + q(7)(0)(0)#414 + q(7)(1)(0)#417
                                 + q(7)(2)(0)#420 + q(7)(3)(0)#423
                                 + q(7)(4)(0)#426 + q(8)(0)(0)#429
                                 + q(8)(1)(0)#432 + q(8)(2)(0)#435
                                 + q(8)(3)(0)#438 + q(8)(4)(0)#441
                                 + q(9)(0)(0)#444 + q(9)(1)(0)#447
                                 + q(9)(2)(0)#450 + q(9)(3)(0)#453
                                 + q(9)(4)(0)#456 + q(10)(0)(0)#459
                                 + q(10)(1)(0)#462 + q(10)(2)(0)#465
                                 + q(10)(3)(0)#468 + q(10)(4)(0)#471
                                 + q(11)(0)(0)#474 + q(11)(1)(0)#477
                                 + q(11)(2)(0)#480 + q(11)(3)(0)#483
                                 + q(11)(4)(0)#486 + q(12)(0)(0)#489
                                 + q(12)(1)(0)#492 + q(12)(2)(0)#495
                                 + q(12)(3)(0)#498 + q(12)(4)(0)#501
                                 + q(13)(0)(0)#504 + q(13)(1)(0)#507
                                 + q(13)(2)(0)#510 + q(13)(3)(0)#513
                                 + q(13)(4)(0)#516 + q(14)(0)(0)#519
                                 + q(14)(1)(0)#522 + q(14)(2)(0)#525
                                 + q(14)(3)(0)#528 + q(14)(4)(0)#531
                                 + q(15)(0)(0)#534 + q(15)(1)(0)#537
                                 + q(15)(2)(0)#540 + q(15)(3)(0)#543
                                 + q(15)(4)(0)#546 + q(16)(0)(0)#549
                                 + q(16)(1)(0)#552 + q(16)(2)(0)#555
                                 + q(16)(3)(0)#558 + q(16)(4)(0)#561
                                 + q(17)(0)(0)#564 + q(17)(1)(0)#567
                                 + q(17)(2)(0)#570 + q(17)(3)(0)#573
                                 + q(17)(4)(0)#576 + q(18)(0)(0)#579
                                 + q(18)(1)(0)#582 + q(18)(2)(0)#585
                                 + q(18)(3)(0)#588 + q(18)(4)(0)#591
                                 + q(19)(0)(0)#594 + q(19)(1)(0)#597
                                 + q(19)(2)(0)#600 + q(19)(3)(0)#603
                                 + q(19)(4)(0)#606 + q(20)(0)(0)#609
                                 + q(20)(1)(0)#612 + q(20)(2)(0)#615
                                 + q(20)(3)(0)#618 + q(20)(4)(0)#621
                                 + q(21)(0)(0)#624 + q(21)(1)(0)#627
                                 + q(21)(2)(0)#630 + q(21)(3)(0)#633
                                 + q(21)(4)(0)#636 + q(22)(0)(0)#639
                                 + q(22)(1)(0)#642 + q(22)(2)(0)#645
                                 + q(22)(3)(0)#648 + q(22)(4)(0)#651
                                 + q(23)(0)(0)#654 + q(23)(1)(0)#657
                                 + q(23)(2)(0)#660 + q(23)(3)(0)#663
                                 + q(23)(4)(0)#666 + q(24)(0)(0)#669
                                 + q(24)(1)(0)#672 + q(24)(2)(0)#675
                                 + q(24)(3)(0)#678 + q(24)(4)(0)#681
                                 + q(25)(0)(0)#684 + q(25)(1)(0)#687
                                 + q(25)(2)(0)#690 + q(25)(3)(0)#693
                                 + q(25)(4)(0)#696 + q(26)(0)(0)#699
                                 + q(26)(1)(0)#702 + q(26)(2)(0)#705
                                 + q(26)(3)(0)#708 + q(26)(4)(0)#711
                                 + q(27)(0)(0)#714 + q(27)(1)(0)#717
                                 + q(27)(2)(0)#720 + q(27)(3)(0)#723
                                 + q(27)(4)(0)#726 + q(28)(0)(0)#729
                                 + q(28)(1)(0)#732 + q(28)(2)(0)#735
                                 + q(28)(3)(0)#738 + q(28)(4)(0)#741
                                 + q(29)(0)(0)#744 + q(29)(1)(0)#747
                                 + q(29)(2)(0)#750 + q(29)(3)(0)#753
                                 + q(29)(4)(0)#756 + q(30)(0)(0)#759
                                 + q(30)(1)(0)#762 + q(30)(2)(0)#765
                                 + q(30)(3)(0)#768 + q(30)(4)(0)#771  = 0
 PlantInventoryBalance(2):       - p(1)#4 - I_plant(0)#6 + I_plant(1)#7
                                 + q(0)(0)(1)#310 + q(0)(1)(1)#313
                                 + q(0)(2)(1)#316 + q(0)(3)(1)#319
                                 + q(0)(4)(1)#322 + q(1)(0)(1)#325
                                 + q(1)(1)(1)#328 + q(1)(2)(1)#331
                                 + q(1)(3)(1)#334 + q(1)(4)(1)#337
                                 + q(2)(0)(1)#340 + q(2)(1)(1)#343
                                 + q(2)(2)(1)#346 + q(2)(3)(1)#349
                                 + q(2)(4)(1)#352 + q(3)(0)(1)#355
                                 + q(3)(1)(1)#358 + q(3)(2)(1)#361
                                 + q(3)(3)(1)#364 + q(3)(4)(1)#367
                                 + q(4)(0)(1)#370 + q(4)(1)(1)#373
                                 + q(4)(2)(1)#376 + q(4)(3)(1)#379
                                 + q(4)(4)(1)#382 + q(5)(0)(1)#385
                                 + q(5)(1)(1)#388 + q(5)(2)(1)#391
                                 + q(5)(3)(1)#394 + q(5)(4)(1)#397
                                 + q(6)(0)(1)#400 + q(6)(1)(1)#403
                                 + q(6)(2)(1)#406 + q(6)(3)(1)#409
                                 + q(6)(4)(1)#412 + q(7)(0)(1)#415
                                 + q(7)(1)(1)#418 + q(7)(2)(1)#421
                                 + q(7)(3)(1)#424 + q(7)(4)(1)#427
                                 + q(8)(0)(1)#430 + q(8)(1)(1)#433
                                 + q(8)(2)(1)#436 + q(8)(3)(1)#439
                                 + q(8)(4)(1)#442 + q(9)(0)(1)#445
                                 + q(9)(1)(1)#448 + q(9)(2)(1)#451
                                 + q(9)(3)(1)#454 + q(9)(4)(1)#457
                                 + q(10)(0)(1)#460 + q(10)(1)(1)#463
                                 + q(10)(2)(1)#466 + q(10)(3)(1)#469
                                 + q(10)(4)(1)#472 + q(11)(0)(1)#475
                                 + q(11)(1)(1)#478 + q(11)(2)(1)#481
                                 + q(11)(3)(1)#484 + q(11)(4)(1)#487
                                 + q(12)(0)(1)#490 + q(12)(1)(1)#493
                                 + q(12)(2)(1)#496 + q(12)(3)(1)#499
                                 + q(12)(4)(1)#502 + q(13)(0)(1)#505
                                 + q(13)(1)(1)#508 + q(13)(2)(1)#511
                                 + q(13)(3)(1)#514 + q(13)(4)(1)#517
                                 + q(14)(0)(1)#520 + q(14)(1)(1)#523
                                 + q(14)(2)(1)#526 + q(14)(3)(1)#529
                                 + q(14)(4)(1)#532 + q(15)(0)(1)#535
                                 + q(15)(1)(1)#538 + q(15)(2)(1)#541
                                 + q(15)(3)(1)#544 + q(15)(4)(1)#547
                                 + q(16)(0)(1)#550 + q(16)(1)(1)#553
                                 + q(16)(2)(1)#556 + q(16)(3)(1)#559
                                 + q(16)(4)(1)#562 + q(17)(0)(1)#565
                                 + q(17)(1)(1)#568 + q(17)(2)(1)#571
                                 + q(17)(3)(1)#574 + q(17)(4)(1)#577
                                 + q(18)(0)(1)#580 + q(18)(1)(1)#583
                                 + q(18)(2)(1)#586 + q(18)(3)(1)#589
                                 + q(18)(4)(1)#592 + q(19)(0)(1)#595
                                 + q(19)(1)(1)#598 + q(19)(2)(1)#601
                                 + q(19)(3)(1)#604 + q(19)(4)(1)#607
                                 + q(20)(0)(1)#610 + q(20)(1)(1)#613
                                 + q(20)(2)(1)#616 + q(20)(3)(1)#619
                                 + q(20)(4)(1)#622 + q(21)(0)(1)#625
                                 + q(21)(1)(1)#628 + q(21)(2)(1)#631
                                 + q(21)(3)(1)#634 + q(21)(4)(1)#637
                                 + q(22)(0)(1)#640 + q(22)(1)(1)#643
                                 + q(22)(2)(1)#646 + q(22)(3)(1)#649
                                 + q(22)(4)(1)#652 + q(23)(0)(1)#655
                                 + q(23)(1)(1)#658 + q(23)(2)(1)#661
                                 + q(23)(3)(1)#664 + q(23)(4)(1)#667
                                 + q(24)(0)(1)#670 + q(24)(1)(1)#673
                                 + q(24)(2)(1)#676 + q(24)(3)(1)#679
                                 + q(24)(4)(1)#682 + q(25)(0)(1)#685
                                 + q(25)(1)(1)#688 + q(25)(2)(1)#691
                                 + q(25)(3)(1)#694 + q(25)(4)(1)#697
                                 + q(26)(0)(1)#700 + q(26)(1)(1)#703
                                 + q(26)(2)(1)#706 + q(26)(3)(1)#709
                                 + q(26)(4)(1)#712 + q(27)(0)(1)#715
                                 + q(27)(1)(1)#718 + q(27)(2)(1)#721
                                 + q(27)(3)(1)#724 + q(27)(4)(1)#727
                                 + q(28)(0)(1)#730 + q(28)(1)(1)#733
                                 + q(28)(2)(1)#736 + q(28)(3)(1)#739
                                 + q(28)(4)(1)#742 + q(29)(0)(1)#745
                                 + q(29)(1)(1)#748 + q(29)(2)(1)#751
                                 + q(29)(3)(1)#754 + q(29)(4)(1)#757
                                 + q(30)(0)(1)#760 + q(30)(1)(1)#763
                                 + q(30)(2)(1)#766 + q(30)(3)(1)#769
                                 + q(30)(4)(1)#772  = 0
 PlantInventoryBalance(3):       - p(2)#5 - I_plant(1)#7 + I_plant(2)#8
                                 + q(0)(0)(2)#311 + q(0)(1)(2)#314
                                 + q(0)(2)(2)#317 + q(0)(3)(2)#320
                                 + q(0)(4)(2)#323 + q(1)(0)(2)#326
                                 + q(1)(1)(2)#329 + q(1)(2)(2)#332
                                 + q(1)(3)(2)#335 + q(1)(4)(2)#338
                                 + q(2)(0)(2)#341 + q(2)(1)(2)#344
                                 + q(2)(2)(2)#347 + q(2)(3)(2)#350
                                 + q(2)(4)(2)#353 + q(3)(0)(2)#356
                                 + q(3)(1)(2)#359 + q(3)(2)(2)#362
                                 + q(3)(3)(2)#365 + q(3)(4)(2)#368
                                 + q(4)(0)(2)#371 + q(4)(1)(2)#374
                                 + q(4)(2)(2)#377 + q(4)(3)(2)#380
                                 + q(4)(4)(2)#383 + q(5)(0)(2)#386
                                 + q(5)(1)(2)#389 + q(5)(2)(2)#392
                                 + q(5)(3)(2)#395 + q(5)(4)(2)#398
                                 + q(6)(0)(2)#401 + q(6)(1)(2)#404
                                 + q(6)(2)(2)#407 + q(6)(3)(2)#410
                                 + q(6)(4)(2)#413 + q(7)(0)(2)#416
                                 + q(7)(1)(2)#419 + q(7)(2)(2)#422
                                 + q(7)(3)(2)#425 + q(7)(4)(2)#428
                                 + q(8)(0)(2)#431 + q(8)(1)(2)#434
                                 + q(8)(2)(2)#437 + q(8)(3)(2)#440
                                 + q(8)(4)(2)#443 + q(9)(0)(2)#446
                                 + q(9)(1)(2)#449 + q(9)(2)(2)#452
                                 + q(9)(3)(2)#455 + q(9)(4)(2)#458
                                 + q(10)(0)(2)#461 + q(10)(1)(2)#464
                                 + q(10)(2)(2)#467 + q(10)(3)(2)#470
                                 + q(10)(4)(2)#473 + q(11)(0)(2)#476
                                 + q(11)(1)(2)#479 + q(11)(2)(2)#482
                                 + q(11)(3)(2)#485 + q(11)(4)(2)#488
                                 + q(12)(0)(2)#491 + q(12)(1)(2)#494
                                 + q(12)(2)(2)#497 + q(12)(3)(2)#500
                                 + q(12)(4)(2)#503 + q(13)(0)(2)#506
                                 + q(13)(1)(2)#509 + q(13)(2)(2)#512
                                 + q(13)(3)(2)#515 + q(13)(4)(2)#518
                                 + q(14)(0)(2)#521 + q(14)(1)(2)#524
                                 + q(14)(2)(2)#527 + q(14)(3)(2)#530
                                 + q(14)(4)(2)#533 + q(15)(0)(2)#536
                                 + q(15)(1)(2)#539 + q(15)(2)(2)#542
                                 + q(15)(3)(2)#545 + q(15)(4)(2)#548
                                 + q(16)(0)(2)#551 + q(16)(1)(2)#554
                                 + q(16)(2)(2)#557 + q(16)(3)(2)#560
                                 + q(16)(4)(2)#563 + q(17)(0)(2)#566
                                 + q(17)(1)(2)#569 + q(17)(2)(2)#572
                                 + q(17)(3)(2)#575 + q(17)(4)(2)#578
                                 + q(18)(0)(2)#581 + q(18)(1)(2)#584
                                 + q(18)(2)(2)#587 + q(18)(3)(2)#590
                                 + q(18)(4)(2)#593 + q(19)(0)(2)#596
                                 + q(19)(1)(2)#599 + q(19)(2)(2)#602
                                 + q(19)(3)(2)#605 + q(19)(4)(2)#608
                                 + q(20)(0)(2)#611 + q(20)(1)(2)#614
                                 + q(20)(2)(2)#617 + q(20)(3)(2)#620
                                 + q(20)(4)(2)#623 + q(21)(0)(2)#626
                                 + q(21)(1)(2)#629 + q(21)(2)(2)#632
                                 + q(21)(3)(2)#635 + q(21)(4)(2)#638
                                 + q(22)(0)(2)#641 + q(22)(1)(2)#644
                                 + q(22)(2)(2)#647 + q(22)(3)(2)#650
                                 + q(22)(4)(2)#653 + q(23)(0)(2)#656
                                 + q(23)(1)(2)#659 + q(23)(2)(2)#662
                                 + q(23)(3)(2)#665 + q(23)(4)(2)#668
                                 + q(24)(0)(2)#671 + q(24)(1)(2)#674
                                 + q(24)(2)(2)#677 + q(24)(3)(2)#680
                                 + q(24)(4)(2)#683 + q(25)(0)(2)#686
                                 + q(25)(1)(2)#689 + q(25)(2)(2)#692
                                 + q(25)(3)(2)#695 + q(25)(4)(2)#698
                                 + q(26)(0)(2)#701 + q(26)(1)(2)#704
                                 + q(26)(2)(2)#707 + q(26)(3)(2)#710
                                 + q(26)(4)(2)#713 + q(27)(0)(2)#716
                                 + q(27)(1)(2)#719 + q(27)(2)(2)#722
                                 + q(27)(3)(2)#725 + q(27)(4)(2)#728
                                 + q(28)(0)(2)#731 + q(28)(1)(2)#734
                                 + q(28)(2)(2)#737 + q(28)(3)(2)#740
                                 + q(28)(4)(2)#743 + q(29)(0)(2)#746
                                 + q(29)(1)(2)#749 + q(29)(2)(2)#752
                                 + q(29)(3)(2)#755 + q(29)(4)(2)#758
                                 + q(30)(0)(2)#761 + q(30)(1)(2)#764
                                 + q(30)(2)(2)#767 + q(30)(3)(2)#770
                                 + q(30)(4)(2)#773  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(0)(0)(0)#9 <= 424.8
 WarehouseInventoryCapacity(2,1,1): I_warehouse(1)(0)(0)#39 <= 424.8
 WarehouseInventoryCapacity(3,1,1): I_warehouse(2)(0)(0)#69 <= 424.8
 WarehouseInventoryCapacity(4,1,1): I_warehouse(3)(0)(0)#99 <= 424.8
 WarehouseInventoryCapacity(5,1,1): I_warehouse(4)(0)(0)#129 <= 424.8
 WarehouseInventoryCapacity(1,2,1): I_warehouse(0)(1)(0)#19 <= 424.8
 WarehouseInventoryCapacity(2,2,1): I_warehouse(1)(1)(0)#49 <= 424.8
 WarehouseInventoryCapacity(3,2,1): I_warehouse(2)(1)(0)#79 <= 424.8
 WarehouseInventoryCapacity(4,2,1): I_warehouse(3)(1)(0)#109 <= 424.8
 WarehouseInventoryCapacity(5,2,1): I_warehouse(4)(1)(0)#139 <= 424.8
 WarehouseInventoryCapacity(1,3,1): I_warehouse(0)(2)(0)#29 <= 424.8
 WarehouseInventoryCapacity(2,3,1): I_warehouse(1)(2)(0)#59 <= 424.8
 WarehouseInventoryCapacity(3,3,1): I_warehouse(2)(2)(0)#89 <= 424.8
 WarehouseInventoryCapacity(4,3,1): I_warehouse(3)(2)(0)#119 <= 424.8
 WarehouseInventoryCapacity(5,3,1): I_warehouse(4)(2)(0)#149 <= 424.8
 WarehouseInventoryCapacity(1,1,2): I_warehouse(0)(0)(1)#10 <= 424.8
 WarehouseInventoryCapacity(2,1,2): I_warehouse(1)(0)(1)#40 <= 424.8
 WarehouseInventoryCapacity(3,1,2): I_warehouse(2)(0)(1)#70 <= 424.8
 WarehouseInventoryCapacity(4,1,2): I_warehouse(3)(0)(1)#100 <= 424.8
 WarehouseInventoryCapacity(5,1,2): I_warehouse(4)(0)(1)#130 <= 424.8
 WarehouseInventoryCapacity(1,2,2): I_warehouse(0)(1)(1)#20 <= 424.8
 WarehouseInventoryCapacity(2,2,2): I_warehouse(1)(1)(1)#50 <= 424.8
 WarehouseInventoryCapacity(3,2,2): I_warehouse(2)(1)(1)#80 <= 424.8
 WarehouseInventoryCapacity(4,2,2): I_warehouse(3)(1)(1)#110 <= 424.8
 WarehouseInventoryCapacity(5,2,2): I_warehouse(4)(1)(1)#140 <= 424.8
 WarehouseInventoryCapacity(1,3,2): I_warehouse(0)(2)(1)#30 <= 424.8
 WarehouseInventoryCapacity(2,3,2): I_warehouse(1)(2)(1)#60 <= 424.8
 WarehouseInventoryCapacity(3,3,2): I_warehouse(2)(2)(1)#90 <= 424.8
 WarehouseInventoryCapacity(4,3,2): I_warehouse(3)(2)(1)#120 <= 424.8
 WarehouseInventoryCapacity(5,3,2): I_warehouse(4)(2)(1)#150 <= 424.8
 WarehouseInventoryCapacity(1,1,3): I_warehouse(0)(0)(2)#11 <= 424.8
 WarehouseInventoryCapacity(2,1,3): I_warehouse(1)(0)(2)#41 <= 424.8
 WarehouseInventoryCapacity(3,1,3): I_warehouse(2)(0)(2)#71 <= 424.8
 WarehouseInventoryCapacity(4,1,3): I_warehouse(3)(0)(2)#101 <= 424.8
 WarehouseInventoryCapacity(5,1,3): I_warehouse(4)(0)(2)#131 <= 424.8
 WarehouseInventoryCapacity(1,2,3): I_warehouse(0)(1)(2)#21 <= 424.8
 WarehouseInventoryCapacity(2,2,3): I_warehouse(1)(1)(2)#51 <= 424.8
 WarehouseInventoryCapacity(3,2,3): I_warehouse(2)(1)(2)#81 <= 424.8
 WarehouseInventoryCapacity(4,2,3): I_warehouse(3)(1)(2)#111 <= 424.8
 WarehouseInventoryCapacity(5,2,3): I_warehouse(4)(1)(2)#141 <= 424.8
 WarehouseInventoryCapacity(1,3,3): I_warehouse(0)(2)(2)#31 <= 424.8
 WarehouseInventoryCapacity(2,3,3): I_warehouse(1)(2)(2)#61 <= 424.8
 WarehouseInventoryCapacity(3,3,3): I_warehouse(2)(2)(2)#91 <= 424.8
 WarehouseInventoryCapacity(4,3,3): I_warehouse(3)(2)(2)#121 <= 424.8
 WarehouseInventoryCapacity(5,3,3): I_warehouse(4)(2)(2)#151 <= 424.8
 WarehouseInventoryCapacity(1,1,4): I_warehouse(0)(0)(3)#12 <= 424.8
 WarehouseInventoryCapacity(2,1,4): I_warehouse(1)(0)(3)#42 <= 424.8
 WarehouseInventoryCapacity(3,1,4): I_warehouse(2)(0)(3)#72 <= 424.8
 WarehouseInventoryCapacity(4,1,4): I_warehouse(3)(0)(3)#102 <= 424.8
 WarehouseInventoryCapacity(5,1,4): I_warehouse(4)(0)(3)#132 <= 424.8
 WarehouseInventoryCapacity(1,2,4): I_warehouse(0)(1)(3)#22 <= 424.8
 WarehouseInventoryCapacity(2,2,4): I_warehouse(1)(1)(3)#52 <= 424.8
 WarehouseInventoryCapacity(3,2,4): I_warehouse(2)(1)(3)#82 <= 424.8
 WarehouseInventoryCapacity(4,2,4): I_warehouse(3)(1)(3)#112 <= 424.8
 WarehouseInventoryCapacity(5,2,4): I_warehouse(4)(1)(3)#142 <= 424.8
 WarehouseInventoryCapacity(1,3,4): I_warehouse(0)(2)(3)#32 <= 424.8
 WarehouseInventoryCapacity(2,3,4): I_warehouse(1)(2)(3)#62 <= 424.8
 WarehouseInventoryCapacity(3,3,4): I_warehouse(2)(2)(3)#92 <= 424.8
 WarehouseInventoryCapacity(4,3,4): I_warehouse(3)(2)(3)#122 <= 424.8
 WarehouseInventoryCapacity(5,3,4): I_warehouse(4)(2)(3)#152 <= 424.8
 WarehouseInventoryCapacity(1,1,5): I_warehouse(0)(0)(4)#13 <= 424.8
 WarehouseInventoryCapacity(2,1,5): I_warehouse(1)(0)(4)#43 <= 424.8
 WarehouseInventoryCapacity(3,1,5): I_warehouse(2)(0)(4)#73 <= 424.8
 WarehouseInventoryCapacity(4,1,5): I_warehouse(3)(0)(4)#103 <= 424.8
 WarehouseInventoryCapacity(5,1,5): I_warehouse(4)(0)(4)#133 <= 424.8
 WarehouseInventoryCapacity(1,2,5): I_warehouse(0)(1)(4)#23 <= 424.8
 WarehouseInventoryCapacity(2,2,5): I_warehouse(1)(1)(4)#53 <= 424.8
 WarehouseInventoryCapacity(3,2,5): I_warehouse(2)(1)(4)#83 <= 424.8
 WarehouseInventoryCapacity(4,2,5): I_warehouse(3)(1)(4)#113 <= 424.8
 WarehouseInventoryCapacity(5,2,5): I_warehouse(4)(1)(4)#143 <= 424.8
 WarehouseInventoryCapacity(1,3,5): I_warehouse(0)(2)(4)#33 <= 424.8
 WarehouseInventoryCapacity(2,3,5): I_warehouse(1)(2)(4)#63 <= 424.8
 WarehouseInventoryCapacity(3,3,5): I_warehouse(2)(2)(4)#93 <= 424.8
 WarehouseInventoryCapacity(4,3,5): I_warehouse(3)(2)(4)#123 <= 424.8
 WarehouseInventoryCapacity(5,3,5): I_warehouse(4)(2)(4)#153 <= 424.8
 WarehouseInventoryCapacity(1,1,6): I_warehouse(0)(0)(5)#14 <= 424.8
 WarehouseInventoryCapacity(2,1,6): I_warehouse(1)(0)(5)#44 <= 424.8
 WarehouseInventoryCapacity(3,1,6): I_warehouse(2)(0)(5)#74 <= 424.8
 WarehouseInventoryCapacity(4,1,6): I_warehouse(3)(0)(5)#104 <= 424.8
 WarehouseInventoryCapacity(5,1,6): I_warehouse(4)(0)(5)#134 <= 424.8
 WarehouseInventoryCapacity(1,2,6): I_warehouse(0)(1)(5)#24 <= 424.8
 WarehouseInventoryCapacity(2,2,6): I_warehouse(1)(1)(5)#54 <= 424.8
 WarehouseInventoryCapacity(3,2,6): I_warehouse(2)(1)(5)#84 <= 424.8
 WarehouseInventoryCapacity(4,2,6): I_warehouse(3)(1)(5)#114 <= 424.8
 WarehouseInventoryCapacity(5,2,6): I_warehouse(4)(1)(5)#144 <= 424.8
 WarehouseInventoryCapacity(1,3,6): I_warehouse(0)(2)(5)#34 <= 424.8
 WarehouseInventoryCapacity(2,3,6): I_warehouse(1)(2)(5)#64 <= 424.8
 WarehouseInventoryCapacity(3,3,6): I_warehouse(2)(2)(5)#94 <= 424.8
 WarehouseInventoryCapacity(4,3,6): I_warehouse(3)(2)(5)#124 <= 424.8
 WarehouseInventoryCapacity(5,3,6): I_warehouse(4)(2)(5)#154 <= 424.8
 WarehouseInventoryCapacity(1,1,7): I_warehouse(0)(0)(6)#15 <= 424.8
 WarehouseInventoryCapacity(2,1,7): I_warehouse(1)(0)(6)#45 <= 424.8
 WarehouseInventoryCapacity(3,1,7): I_warehouse(2)(0)(6)#75 <= 424.8
 WarehouseInventoryCapacity(4,1,7): I_warehouse(3)(0)(6)#105 <= 424.8
 WarehouseInventoryCapacity(5,1,7): I_warehouse(4)(0)(6)#135 <= 424.8
 WarehouseInventoryCapacity(1,2,7): I_warehouse(0)(1)(6)#25 <= 424.8
 WarehouseInventoryCapacity(2,2,7): I_warehouse(1)(1)(6)#55 <= 424.8
 WarehouseInventoryCapacity(3,2,7): I_warehouse(2)(1)(6)#85 <= 424.8
 WarehouseInventoryCapacity(4,2,7): I_warehouse(3)(1)(6)#115 <= 424.8
 WarehouseInventoryCapacity(5,2,7): I_warehouse(4)(1)(6)#145 <= 424.8
 WarehouseInventoryCapacity(1,3,7): I_warehouse(0)(2)(6)#35 <= 424.8
 WarehouseInventoryCapacity(2,3,7): I_warehouse(1)(2)(6)#65 <= 424.8
 WarehouseInventoryCapacity(3,3,7): I_warehouse(2)(2)(6)#95 <= 424.8
 WarehouseInventoryCapacity(4,3,7): I_warehouse(3)(2)(6)#125 <= 424.8
 WarehouseInventoryCapacity(5,3,7): I_warehouse(4)(2)(6)#155 <= 424.8
 WarehouseInventoryCapacity(1,1,8): I_warehouse(0)(0)(7)#16 <= 424.8
 WarehouseInventoryCapacity(2,1,8): I_warehouse(1)(0)(7)#46 <= 424.8
 WarehouseInventoryCapacity(3,1,8): I_warehouse(2)(0)(7)#76 <= 424.8
 WarehouseInventoryCapacity(4,1,8): I_warehouse(3)(0)(7)#106 <= 424.8
 WarehouseInventoryCapacity(5,1,8): I_warehouse(4)(0)(7)#136 <= 424.8
 WarehouseInventoryCapacity(1,2,8): I_warehouse(0)(1)(7)#26 <= 424.8
 WarehouseInventoryCapacity(2,2,8): I_warehouse(1)(1)(7)#56 <= 424.8
 WarehouseInventoryCapacity(3,2,8): I_warehouse(2)(1)(7)#86 <= 424.8
 WarehouseInventoryCapacity(4,2,8): I_warehouse(3)(1)(7)#116 <= 424.8
 WarehouseInventoryCapacity(5,2,8): I_warehouse(4)(1)(7)#146 <= 424.8
 WarehouseInventoryCapacity(1,3,8): I_warehouse(0)(2)(7)#36 <= 424.8
 WarehouseInventoryCapacity(2,3,8): I_warehouse(1)(2)(7)#66 <= 424.8
 WarehouseInventoryCapacity(3,3,8): I_warehouse(2)(2)(7)#96 <= 424.8
 WarehouseInventoryCapacity(4,3,8): I_warehouse(3)(2)(7)#126 <= 424.8
 WarehouseInventoryCapacity(5,3,8): I_warehouse(4)(2)(7)#156 <= 424.8
 WarehouseInventoryCapacity(1,1,9): I_warehouse(0)(0)(8)#17 <= 424.8
 WarehouseInventoryCapacity(2,1,9): I_warehouse(1)(0)(8)#47 <= 424.8
 WarehouseInventoryCapacity(3,1,9): I_warehouse(2)(0)(8)#77 <= 424.8
 WarehouseInventoryCapacity(4,1,9): I_warehouse(3)(0)(8)#107 <= 424.8
 WarehouseInventoryCapacity(5,1,9): I_warehouse(4)(0)(8)#137 <= 424.8
 WarehouseInventoryCapacity(1,2,9): I_warehouse(0)(1)(8)#27 <= 424.8
 WarehouseInventoryCapacity(2,2,9): I_warehouse(1)(1)(8)#57 <= 424.8
 WarehouseInventoryCapacity(3,2,9): I_warehouse(2)(1)(8)#87 <= 424.8
 WarehouseInventoryCapacity(4,2,9): I_warehouse(3)(1)(8)#117 <= 424.8
 WarehouseInventoryCapacity(5,2,9): I_warehouse(4)(1)(8)#147 <= 424.8
 WarehouseInventoryCapacity(1,3,9): I_warehouse(0)(2)(8)#37 <= 424.8
 WarehouseInventoryCapacity(2,3,9): I_warehouse(1)(2)(8)#67 <= 424.8
 WarehouseInventoryCapacity(3,3,9): I_warehouse(2)(2)(8)#97 <= 424.8
 WarehouseInventoryCapacity(4,3,9): I_warehouse(3)(2)(8)#127 <= 424.8
 WarehouseInventoryCapacity(5,3,9): I_warehouse(4)(2)(8)#157 <= 424.8
 WarehouseInventoryCapacity(1,1,10): I_warehouse(0)(0)(9)#18 <= 424.8
 WarehouseInventoryCapacity(2,1,10): I_warehouse(1)(0)(9)#48 <= 424.8
 WarehouseInventoryCapacity(3,1,10): I_warehouse(2)(0)(9)#78 <= 424.8
 WarehouseInventoryCapacity(4,1,10): I_warehouse(3)(0)(9)#108 <= 424.8
 WarehouseInventoryCapacity(5,1,10): I_warehouse(4)(0)(9)#138 <= 424.8
 WarehouseInventoryCapacity(1,2,10): I_warehouse(0)(1)(9)#28 <= 424.8
 WarehouseInventoryCapacity(2,2,10): I_warehouse(1)(1)(9)#58 <= 424.8
 WarehouseInventoryCapacity(3,2,10): I_warehouse(2)(1)(9)#88 <= 424.8
 WarehouseInventoryCapacity(4,2,10): I_warehouse(3)(1)(9)#118 <= 424.8
 WarehouseInventoryCapacity(5,2,10): I_warehouse(4)(1)(9)#148 <= 424.8
 WarehouseInventoryCapacity(1,3,10): I_warehouse(0)(2)(9)#38 <= 424.8
 WarehouseInventoryCapacity(2,3,10): I_warehouse(1)(2)(9)#68 <= 424.8
 WarehouseInventoryCapacity(3,3,10): I_warehouse(2)(2)(9)#98 <= 424.8
 WarehouseInventoryCapacity(4,3,10): I_warehouse(3)(2)(9)#128 <= 424.8
 WarehouseInventoryCapacity(5,3,10): I_warehouse(4)(2)(9)#158 <= 424.8
 WarehouseInventoryBalance(1,1,1): I_warehouse(0)(0)(0)#9
                                 - b_warehouse(0)(0)(0)#159 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -3
 WarehouseInventoryBalance(2,1,1): I_warehouse(1)(0)(0)#39
                                 - b_warehouse(1)(0)(0)#189 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -18
 WarehouseInventoryBalance(3,1,1): I_warehouse(2)(0)(0)#69
                                 - b_warehouse(2)(0)(0)#219 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -6
 WarehouseInventoryBalance(4,1,1): I_warehouse(3)(0)(0)#99
                                 - b_warehouse(3)(0)(0)#249 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -15
 WarehouseInventoryBalance(5,1,1): I_warehouse(4)(0)(0)#129
                                 - b_warehouse(4)(0)(0)#279 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -23
 WarehouseInventoryBalance(1,2,1): - I_warehouse(0)(0)(0)#9
                                 + I_warehouse(0)(1)(0)#19
                                 - b_warehouse(0)(1)(0)#169 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -34
 WarehouseInventoryBalance(2,2,1): - I_warehouse(1)(0)(0)#39
                                 + I_warehouse(1)(1)(0)#49
                                 - b_warehouse(1)(1)(0)#199 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -47
 WarehouseInventoryBalance(3,2,1): - I_warehouse(2)(0)(0)#69
                                 + I_warehouse(2)(1)(0)#79
                                 - b_warehouse(2)(1)(0)#229 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -49
 WarehouseInventoryBalance(4,2,1): - I_warehouse(3)(0)(0)#99
                                 + I_warehouse(3)(1)(0)#109
                                 - b_warehouse(3)(1)(0)#259 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -37
 WarehouseInventoryBalance(5,2,1): - I_warehouse(4)(0)(0)#129
                                 + I_warehouse(4)(1)(0)#139
                                 - b_warehouse(4)(1)(0)#289 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -58
 WarehouseInventoryBalance(1,3,1): - I_warehouse(0)(1)(0)#19
                                 + I_warehouse(0)(2)(0)#29
                                 - b_warehouse(0)(2)(0)#179 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -104
 WarehouseInventoryBalance(2,3,1): - I_warehouse(1)(1)(0)#49
                                 + I_warehouse(1)(2)(0)#59
                                 - b_warehouse(1)(2)(0)#209 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -57
 WarehouseInventoryBalance(3,3,1): - I_warehouse(2)(1)(0)#79
                                 + I_warehouse(2)(2)(0)#89
                                 - b_warehouse(2)(2)(0)#239 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -69
 WarehouseInventoryBalance(4,3,1): - I_warehouse(3)(1)(0)#109
                                 + I_warehouse(3)(2)(0)#119
                                 - b_warehouse(3)(2)(0)#269 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -37
 WarehouseInventoryBalance(5,3,1): - I_warehouse(4)(1)(0)#139
                                 + I_warehouse(4)(2)(0)#149
                                 - b_warehouse(4)(2)(0)#299 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -119
 WarehouseInventoryBalance(1,1,2): I_warehouse(0)(0)(1)#10
                                 - b_warehouse(0)(0)(1)#160 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -143
 WarehouseInventoryBalance(2,1,2): I_warehouse(1)(0)(1)#40
                                 - b_warehouse(1)(0)(1)#190 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -52
 WarehouseInventoryBalance(3,1,2): I_warehouse(2)(0)(1)#70
                                 - b_warehouse(2)(0)(1)#220 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -86
 WarehouseInventoryBalance(4,1,2): I_warehouse(3)(0)(1)#100
                                 - b_warehouse(3)(0)(1)#250 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -53
 WarehouseInventoryBalance(5,1,2): I_warehouse(4)(0)(1)#130
                                 - b_warehouse(4)(0)(1)#280 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -141
 WarehouseInventoryBalance(1,2,2): - I_warehouse(0)(0)(1)#10
                                 + I_warehouse(0)(1)(1)#20
                                 - b_warehouse(0)(1)(1)#170 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -147
 WarehouseInventoryBalance(2,2,2): - I_warehouse(1)(0)(1)#40
                                 + I_warehouse(1)(1)(1)#50
                                 - b_warehouse(1)(1)(1)#200 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -50
 WarehouseInventoryBalance(3,2,2): - I_warehouse(2)(0)(1)#70
                                 + I_warehouse(2)(1)(1)#80
                                 - b_warehouse(2)(1)(1)#230 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -86
 WarehouseInventoryBalance(4,2,2): - I_warehouse(3)(0)(1)#100
                                 + I_warehouse(3)(1)(1)#110
                                 - b_warehouse(3)(1)(1)#260 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -52
 WarehouseInventoryBalance(5,2,2): - I_warehouse(4)(0)(1)#130
                                 + I_warehouse(4)(1)(1)#140
                                 - b_warehouse(4)(1)(1)#290 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -139
 WarehouseInventoryBalance(1,3,2): - I_warehouse(0)(1)(1)#20
                                 + I_warehouse(0)(2)(1)#30
                                 - b_warehouse(0)(2)(1)#180 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -130
 WarehouseInventoryBalance(2,3,2): - I_warehouse(1)(1)(1)#50
                                 + I_warehouse(1)(2)(1)#60
                                 - b_warehouse(1)(2)(1)#210 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -48
 WarehouseInventoryBalance(3,3,2): - I_warehouse(2)(1)(1)#80
                                 + I_warehouse(2)(2)(1)#90
                                 - b_warehouse(2)(2)(1)#240 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -82
 WarehouseInventoryBalance(4,3,2): - I_warehouse(3)(1)(1)#110
                                 + I_warehouse(3)(2)(1)#120
                                 - b_warehouse(3)(2)(1)#270 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -60
 WarehouseInventoryBalance(5,3,2): - I_warehouse(4)(1)(1)#140
                                 + I_warehouse(4)(2)(1)#150
                                 - b_warehouse(4)(2)(1)#300 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -150
 WarehouseInventoryBalance(1,1,3): I_warehouse(0)(0)(2)#11
                                 - b_warehouse(0)(0)(2)#161 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -132
 WarehouseInventoryBalance(2,1,3): I_warehouse(1)(0)(2)#41
                                 - b_warehouse(1)(0)(2)#191 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -44
 WarehouseInventoryBalance(3,1,3): I_warehouse(2)(0)(2)#71
                                 - b_warehouse(2)(0)(2)#221 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -87
 WarehouseInventoryBalance(4,1,3): I_warehouse(3)(0)(2)#101
                                 - b_warehouse(3)(0)(2)#251 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -60
 WarehouseInventoryBalance(5,1,3): I_warehouse(4)(0)(2)#131
                                 - b_warehouse(4)(0)(2)#281 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -154
 WarehouseInventoryBalance(1,2,3): - I_warehouse(0)(0)(2)#11
                                 + I_warehouse(0)(1)(2)#21
                                 - b_warehouse(0)(1)(2)#171 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -133
 WarehouseInventoryBalance(2,2,3): - I_warehouse(1)(0)(2)#41
                                 + I_warehouse(1)(1)(2)#51
                                 - b_warehouse(1)(1)(2)#201 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -54
 WarehouseInventoryBalance(3,2,3): - I_warehouse(2)(0)(2)#71
                                 + I_warehouse(2)(1)(2)#81
                                 - b_warehouse(2)(1)(2)#231 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -92
 WarehouseInventoryBalance(4,2,3): - I_warehouse(3)(0)(2)#101
                                 + I_warehouse(3)(1)(2)#111
                                 - b_warehouse(3)(1)(2)#261 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -54
 WarehouseInventoryBalance(5,2,3): - I_warehouse(4)(0)(2)#131
                                 + I_warehouse(4)(1)(2)#141
                                 - b_warehouse(4)(1)(2)#291 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -136
 WarehouseInventoryBalance(1,3,3): - I_warehouse(0)(1)(2)#21
                                 + I_warehouse(0)(2)(2)#31
                                 - b_warehouse(0)(2)(2)#181 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -128
 WarehouseInventoryBalance(2,3,3): - I_warehouse(1)(1)(2)#51
                                 + I_warehouse(1)(2)(2)#61
                                 - b_warehouse(1)(2)(2)#211 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -50
 WarehouseInventoryBalance(3,3,3): - I_warehouse(2)(1)(2)#81
                                 + I_warehouse(2)(2)(2)#91
                                 - b_warehouse(2)(2)(2)#241 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -96
 WarehouseInventoryBalance(4,3,3): - I_warehouse(3)(1)(2)#111
                                 + I_warehouse(3)(2)(2)#121
                                 - b_warehouse(3)(2)(2)#271 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -53
 WarehouseInventoryBalance(5,3,3): - I_warehouse(4)(1)(2)#141
                                 + I_warehouse(4)(2)(2)#151
                                 - b_warehouse(4)(2)(2)#301 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -148
 WarehouseInventoryBalance(1,1,4): I_warehouse(0)(0)(3)#12
                                 - b_warehouse(0)(0)(3)#162 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -135
 WarehouseInventoryBalance(2,1,4): I_warehouse(1)(0)(3)#42
                                 - b_warehouse(1)(0)(3)#192 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -50
 WarehouseInventoryBalance(3,1,4): I_warehouse(2)(0)(3)#72
                                 - b_warehouse(2)(0)(3)#222 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -88
 WarehouseInventoryBalance(4,1,4): I_warehouse(3)(0)(3)#102
                                 - b_warehouse(3)(0)(3)#252 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -57
 WarehouseInventoryBalance(5,1,4): I_warehouse(4)(0)(3)#132
                                 - b_warehouse(4)(0)(3)#282 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -135
 WarehouseInventoryBalance(1,2,4): - I_warehouse(0)(0)(3)#12
                                 + I_warehouse(0)(1)(3)#22
                                 - b_warehouse(0)(1)(3)#172 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -148
 WarehouseInventoryBalance(2,2,4): - I_warehouse(1)(0)(3)#42
                                 + I_warehouse(1)(1)(3)#52
                                 - b_warehouse(1)(1)(3)#202 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -50
 WarehouseInventoryBalance(3,2,4): - I_warehouse(2)(0)(3)#72
                                 + I_warehouse(2)(1)(3)#82
                                 - b_warehouse(2)(1)(3)#232 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -84
 WarehouseInventoryBalance(4,2,4): - I_warehouse(3)(0)(3)#102
                                 + I_warehouse(3)(1)(3)#112
                                 - b_warehouse(3)(1)(3)#262 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -54
 WarehouseInventoryBalance(5,2,4): - I_warehouse(4)(0)(3)#132
                                 + I_warehouse(4)(1)(3)#142
                                 - b_warehouse(4)(1)(3)#292 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -136
 WarehouseInventoryBalance(1,3,4): - I_warehouse(0)(1)(3)#22
                                 + I_warehouse(0)(2)(3)#32
                                 - b_warehouse(0)(2)(3)#182 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -135
 WarehouseInventoryBalance(2,3,4): - I_warehouse(1)(1)(3)#52
                                 + I_warehouse(1)(2)(3)#62
                                 - b_warehouse(1)(2)(3)#212 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -48
 WarehouseInventoryBalance(3,3,4): - I_warehouse(2)(1)(3)#82
                                 + I_warehouse(2)(2)(3)#92
                                 - b_warehouse(2)(2)(3)#242 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -88
 WarehouseInventoryBalance(4,3,4): - I_warehouse(3)(1)(3)#112
                                 + I_warehouse(3)(2)(3)#122
                                 - b_warehouse(3)(2)(3)#272 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -56
 WarehouseInventoryBalance(5,3,4): - I_warehouse(4)(1)(3)#142
                                 + I_warehouse(4)(2)(3)#152
                                 - b_warehouse(4)(2)(3)#302 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -150
 WarehouseInventoryBalance(1,1,5): I_warehouse(0)(0)(4)#13
                                 - b_warehouse(0)(0)(4)#163 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -138
 WarehouseInventoryBalance(2,1,5): I_warehouse(1)(0)(4)#43
                                 - b_warehouse(1)(0)(4)#193 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -44
 WarehouseInventoryBalance(3,1,5): I_warehouse(2)(0)(4)#73
                                 - b_warehouse(2)(0)(4)#223 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -84
 WarehouseInventoryBalance(4,1,5): I_warehouse(3)(0)(4)#103
                                 - b_warehouse(3)(0)(4)#253 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -54
 WarehouseInventoryBalance(5,1,5): I_warehouse(4)(0)(4)#133
                                 - b_warehouse(4)(0)(4)#283 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -145
 WarehouseInventoryBalance(1,2,5): - I_warehouse(0)(0)(4)#13
                                 + I_warehouse(0)(1)(4)#23
                                 - b_warehouse(0)(1)(4)#173 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -126
 WarehouseInventoryBalance(2,2,5): - I_warehouse(1)(0)(4)#43
                                 + I_warehouse(1)(1)(4)#53
                                 - b_warehouse(1)(1)(4)#203 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -50
 WarehouseInventoryBalance(3,2,5): - I_warehouse(2)(0)(4)#73
                                 + I_warehouse(2)(1)(4)#83
                                 - b_warehouse(2)(1)(4)#233 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -92
 WarehouseInventoryBalance(4,2,5): - I_warehouse(3)(0)(4)#103
                                 + I_warehouse(3)(1)(4)#113
                                 - b_warehouse(3)(1)(4)#263 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -51
 WarehouseInventoryBalance(5,2,5): - I_warehouse(4)(0)(4)#133
                                 + I_warehouse(4)(1)(4)#143
                                 - b_warehouse(4)(1)(4)#293 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -142
 WarehouseInventoryBalance(1,3,5): - I_warehouse(0)(1)(4)#23
                                 + I_warehouse(0)(2)(4)#33
                                 - b_warehouse(0)(2)(4)#183 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -139
 WarehouseInventoryBalance(2,3,5): - I_warehouse(1)(1)(4)#53
                                 + I_warehouse(1)(2)(4)#63
                                 - b_warehouse(1)(2)(4)#213 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -53
 WarehouseInventoryBalance(3,3,5): - I_warehouse(2)(1)(4)#83
                                 + I_warehouse(2)(2)(4)#93
                                 - b_warehouse(2)(2)(4)#243 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -84
 WarehouseInventoryBalance(4,3,5): - I_warehouse(3)(1)(4)#113
                                 + I_warehouse(3)(2)(4)#123
                                 - b_warehouse(3)(2)(4)#273 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -64
 WarehouseInventoryBalance(5,3,5): - I_warehouse(4)(1)(4)#143
                                 + I_warehouse(4)(2)(4)#153
                                 - b_warehouse(4)(2)(4)#303 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -143
 WarehouseInventoryBalance(1,1,6): I_warehouse(0)(0)(5)#14
                                 - b_warehouse(0)(0)(5)#164 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -141
 WarehouseInventoryBalance(2,1,6): I_warehouse(1)(0)(5)#44
                                 - b_warehouse(1)(0)(5)#194 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -45
 WarehouseInventoryBalance(3,1,6): I_warehouse(2)(0)(5)#74
                                 - b_warehouse(2)(0)(5)#224 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -85
 WarehouseInventoryBalance(4,1,6): I_warehouse(3)(0)(5)#104
                                 - b_warehouse(3)(0)(5)#254 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -58
 WarehouseInventoryBalance(5,1,6): I_warehouse(4)(0)(5)#134
                                 - b_warehouse(4)(0)(5)#284 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -150
 WarehouseInventoryBalance(1,2,6): - I_warehouse(0)(0)(5)#14
                                 + I_warehouse(0)(1)(5)#24
                                 - b_warehouse(0)(1)(5)#174 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -134
 WarehouseInventoryBalance(2,2,6): - I_warehouse(1)(0)(5)#44
                                 + I_warehouse(1)(1)(5)#54
                                 - b_warehouse(1)(1)(5)#204 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -55
 WarehouseInventoryBalance(3,2,6): - I_warehouse(2)(0)(5)#74
                                 + I_warehouse(2)(1)(5)#84
                                 - b_warehouse(2)(1)(5)#234 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -81
 WarehouseInventoryBalance(4,2,6): - I_warehouse(3)(0)(5)#104
                                 + I_warehouse(3)(1)(5)#114
                                 - b_warehouse(3)(1)(5)#264 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -51
 WarehouseInventoryBalance(5,2,6): - I_warehouse(4)(0)(5)#134
                                 + I_warehouse(4)(1)(5)#144
                                 - b_warehouse(4)(1)(5)#294 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -135
 WarehouseInventoryBalance(1,3,6): - I_warehouse(0)(1)(5)#24
                                 + I_warehouse(0)(2)(5)#34
                                 - b_warehouse(0)(2)(5)#184 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -131
 WarehouseInventoryBalance(2,3,6): - I_warehouse(1)(1)(5)#54
                                 + I_warehouse(1)(2)(5)#64
                                 - b_warehouse(1)(2)(5)#214 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -54
 WarehouseInventoryBalance(3,3,6): - I_warehouse(2)(1)(5)#84
                                 + I_warehouse(2)(2)(5)#94
                                 - b_warehouse(2)(2)(5)#244 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -89
 WarehouseInventoryBalance(4,3,6): - I_warehouse(3)(1)(5)#114
                                 + I_warehouse(3)(2)(5)#124
                                 - b_warehouse(3)(2)(5)#274 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -51
 WarehouseInventoryBalance(5,3,6): - I_warehouse(4)(1)(5)#144
                                 + I_warehouse(4)(2)(5)#154
                                 - b_warehouse(4)(2)(5)#304 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -153
 WarehouseInventoryBalance(1,1,7): I_warehouse(0)(0)(6)#15
                                 - b_warehouse(0)(0)(6)#165 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -131
 WarehouseInventoryBalance(2,1,7): I_warehouse(1)(0)(6)#45
                                 - b_warehouse(1)(0)(6)#195 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -52
 WarehouseInventoryBalance(3,1,7): I_warehouse(2)(0)(6)#75
                                 - b_warehouse(2)(0)(6)#225 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -93
 WarehouseInventoryBalance(4,1,7): I_warehouse(3)(0)(6)#105
                                 - b_warehouse(3)(0)(6)#255 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -56
 WarehouseInventoryBalance(5,1,7): I_warehouse(4)(0)(6)#135
                                 - b_warehouse(4)(0)(6)#285 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -142
 WarehouseInventoryBalance(1,2,7): - I_warehouse(0)(0)(6)#15
                                 + I_warehouse(0)(1)(6)#25
                                 - b_warehouse(0)(1)(6)#175 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -129
 WarehouseInventoryBalance(2,2,7): - I_warehouse(1)(0)(6)#45
                                 + I_warehouse(1)(1)(6)#55
                                 - b_warehouse(1)(1)(6)#205 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -48
 WarehouseInventoryBalance(3,2,7): - I_warehouse(2)(0)(6)#75
                                 + I_warehouse(2)(1)(6)#85
                                 - b_warehouse(2)(1)(6)#235 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -90
 WarehouseInventoryBalance(4,2,7): - I_warehouse(3)(0)(6)#105
                                 + I_warehouse(3)(1)(6)#115
                                 - b_warehouse(3)(1)(6)#265 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -49
 WarehouseInventoryBalance(5,2,7): - I_warehouse(4)(0)(6)#135
                                 + I_warehouse(4)(1)(6)#145
                                 - b_warehouse(4)(1)(6)#295 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -150
 WarehouseInventoryBalance(1,3,7): - I_warehouse(0)(1)(6)#25
                                 + I_warehouse(0)(2)(6)#35
                                 - b_warehouse(0)(2)(6)#185 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -142
 WarehouseInventoryBalance(2,3,7): - I_warehouse(1)(1)(6)#55
                                 + I_warehouse(1)(2)(6)#65
                                 - b_warehouse(1)(2)(6)#215 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -53
 WarehouseInventoryBalance(3,3,7): - I_warehouse(2)(1)(6)#85
                                 + I_warehouse(2)(2)(6)#95
                                 - b_warehouse(2)(2)(6)#245 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -89
 WarehouseInventoryBalance(4,3,7): - I_warehouse(3)(1)(6)#115
                                 + I_warehouse(3)(2)(6)#125
                                 - b_warehouse(3)(2)(6)#275 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -54
 WarehouseInventoryBalance(5,3,7): - I_warehouse(4)(1)(6)#145
                                 + I_warehouse(4)(2)(6)#155
                                 - b_warehouse(4)(2)(6)#305 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -148
 WarehouseInventoryBalance(1,1,8): I_warehouse(0)(0)(7)#16
                                 - b_warehouse(0)(0)(7)#166 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -139
 WarehouseInventoryBalance(2,1,8): I_warehouse(1)(0)(7)#46
                                 - b_warehouse(1)(0)(7)#196 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -48
 WarehouseInventoryBalance(3,1,8): I_warehouse(2)(0)(7)#76
                                 - b_warehouse(2)(0)(7)#226 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -90
 WarehouseInventoryBalance(4,1,8): I_warehouse(3)(0)(7)#106
                                 - b_warehouse(3)(0)(7)#256 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -56
 WarehouseInventoryBalance(5,1,8): I_warehouse(4)(0)(7)#136
                                 - b_warehouse(4)(0)(7)#286 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -138
 WarehouseInventoryBalance(1,2,8): - I_warehouse(0)(0)(7)#16
                                 + I_warehouse(0)(1)(7)#26
                                 - b_warehouse(0)(1)(7)#176 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -129
 WarehouseInventoryBalance(2,2,8): - I_warehouse(1)(0)(7)#46
                                 + I_warehouse(1)(1)(7)#56
                                 - b_warehouse(1)(1)(7)#206 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -51
 WarehouseInventoryBalance(3,2,8): - I_warehouse(2)(0)(7)#76
                                 + I_warehouse(2)(1)(7)#86
                                 - b_warehouse(2)(1)(7)#236 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -87
 WarehouseInventoryBalance(4,2,8): - I_warehouse(3)(0)(7)#106
                                 + I_warehouse(3)(1)(7)#116
                                 - b_warehouse(3)(1)(7)#266 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -50
 WarehouseInventoryBalance(5,2,8): - I_warehouse(4)(0)(7)#136
                                 + I_warehouse(4)(1)(7)#146
                                 - b_warehouse(4)(1)(7)#296 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -132
 WarehouseInventoryBalance(1,3,8): - I_warehouse(0)(1)(7)#26
                                 + I_warehouse(0)(2)(7)#36
                                 - b_warehouse(0)(2)(7)#186 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -132
 WarehouseInventoryBalance(2,3,8): - I_warehouse(1)(1)(7)#56
                                 + I_warehouse(1)(2)(7)#66
                                 - b_warehouse(1)(2)(7)#216 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -48
 WarehouseInventoryBalance(3,3,8): - I_warehouse(2)(1)(7)#86
                                 + I_warehouse(2)(2)(7)#96
                                 - b_warehouse(2)(2)(7)#246 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -88
 WarehouseInventoryBalance(4,3,8): - I_warehouse(3)(1)(7)#116
                                 + I_warehouse(3)(2)(7)#126
                                 - b_warehouse(3)(2)(7)#276 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -63
 WarehouseInventoryBalance(5,3,8): - I_warehouse(4)(1)(7)#146
                                 + I_warehouse(4)(2)(7)#156
                                 - b_warehouse(4)(2)(7)#306 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -136
 WarehouseInventoryBalance(1,1,9): I_warehouse(0)(0)(8)#17
                                 - b_warehouse(0)(0)(8)#167 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -128
 WarehouseInventoryBalance(2,1,9): I_warehouse(1)(0)(8)#47
                                 - b_warehouse(1)(0)(8)#197 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -45
 WarehouseInventoryBalance(3,1,9): I_warehouse(2)(0)(8)#77
                                 - b_warehouse(2)(0)(8)#227 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -95
 WarehouseInventoryBalance(4,1,9): I_warehouse(3)(0)(8)#107
                                 - b_warehouse(3)(0)(8)#257 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -50
 WarehouseInventoryBalance(5,1,9): I_warehouse(4)(0)(8)#137
                                 - b_warehouse(4)(0)(8)#287 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -150
 WarehouseInventoryBalance(1,2,9): - I_warehouse(0)(0)(8)#17
                                 + I_warehouse(0)(1)(8)#27
                                 - b_warehouse(0)(1)(8)#177 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -130
 WarehouseInventoryBalance(2,2,9): - I_warehouse(1)(0)(8)#47
                                 + I_warehouse(1)(1)(8)#57
                                 - b_warehouse(1)(1)(8)#207 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -52
 WarehouseInventoryBalance(3,2,9): - I_warehouse(2)(0)(8)#77
                                 + I_warehouse(2)(1)(8)#87
                                 - b_warehouse(2)(1)(8)#237 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -87
 WarehouseInventoryBalance(4,2,9): - I_warehouse(3)(0)(8)#107
                                 + I_warehouse(3)(1)(8)#117
                                 - b_warehouse(3)(1)(8)#267 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -59
 WarehouseInventoryBalance(5,2,9): - I_warehouse(4)(0)(8)#137
                                 + I_warehouse(4)(1)(8)#147
                                 - b_warehouse(4)(1)(8)#297 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -143
 WarehouseInventoryBalance(1,3,9): - I_warehouse(0)(1)(8)#27
                                 + I_warehouse(0)(2)(8)#37
                                 - b_warehouse(0)(2)(8)#187 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -132
 WarehouseInventoryBalance(2,3,9): - I_warehouse(1)(1)(8)#57
                                 + I_warehouse(1)(2)(8)#67
                                 - b_warehouse(1)(2)(8)#217 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -53
 WarehouseInventoryBalance(3,3,9): - I_warehouse(2)(1)(8)#87
                                 + I_warehouse(2)(2)(8)#97
                                 - b_warehouse(2)(2)(8)#247 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -83
 WarehouseInventoryBalance(4,3,9): - I_warehouse(3)(1)(8)#117
                                 + I_warehouse(3)(2)(8)#127
                                 - b_warehouse(3)(2)(8)#277 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -50
 WarehouseInventoryBalance(5,3,9): - I_warehouse(4)(1)(8)#147
                                 + I_warehouse(4)(2)(8)#157
                                 - b_warehouse(4)(2)(8)#307 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -143
 WarehouseInventoryBalance(1,1,10): I_warehouse(0)(0)(9)#18
                                 - b_warehouse(0)(0)(9)#168 - q(0)(0)(0)#309
                                 - q(1)(0)(0)#324 - q(2)(0)(0)#339
                                 - q(3)(0)(0)#354 - q(4)(0)(0)#369
                                 - q(5)(0)(0)#384 - q(6)(0)(0)#399
                                 - q(7)(0)(0)#414 - q(8)(0)(0)#429
                                 - q(9)(0)(0)#444 - q(10)(0)(0)#459
                                 - q(11)(0)(0)#474 - q(12)(0)(0)#489
                                 - q(13)(0)(0)#504 - q(14)(0)(0)#519
                                 - q(15)(0)(0)#534 - q(16)(0)(0)#549
                                 - q(17)(0)(0)#564 - q(18)(0)(0)#579
                                 - q(19)(0)(0)#594 - q(20)(0)(0)#609
                                 - q(21)(0)(0)#624 - q(22)(0)(0)#639
                                 - q(23)(0)(0)#654 - q(24)(0)(0)#669
                                 - q(25)(0)(0)#684 - q(26)(0)(0)#699
                                 - q(27)(0)(0)#714 - q(28)(0)(0)#729
                                 - q(29)(0)(0)#744 - q(30)(0)(0)#759  = -137
 WarehouseInventoryBalance(2,1,10): I_warehouse(1)(0)(9)#48
                                 - b_warehouse(1)(0)(9)#198 - q(0)(1)(0)#312
                                 - q(1)(1)(0)#327 - q(2)(1)(0)#342
                                 - q(3)(1)(0)#357 - q(4)(1)(0)#372
                                 - q(5)(1)(0)#387 - q(6)(1)(0)#402
                                 - q(7)(1)(0)#417 - q(8)(1)(0)#432
                                 - q(9)(1)(0)#447 - q(10)(1)(0)#462
                                 - q(11)(1)(0)#477 - q(12)(1)(0)#492
                                 - q(13)(1)(0)#507 - q(14)(1)(0)#522
                                 - q(15)(1)(0)#537 - q(16)(1)(0)#552
                                 - q(17)(1)(0)#567 - q(18)(1)(0)#582
                                 - q(19)(1)(0)#597 - q(20)(1)(0)#612
                                 - q(21)(1)(0)#627 - q(22)(1)(0)#642
                                 - q(23)(1)(0)#657 - q(24)(1)(0)#672
                                 - q(25)(1)(0)#687 - q(26)(1)(0)#702
                                 - q(27)(1)(0)#717 - q(28)(1)(0)#732
                                 - q(29)(1)(0)#747 - q(30)(1)(0)#762  = -45
 WarehouseInventoryBalance(3,1,10): I_warehouse(2)(0)(9)#78
                                 - b_warehouse(2)(0)(9)#228 - q(0)(2)(0)#315
                                 - q(1)(2)(0)#330 - q(2)(2)(0)#345
                                 - q(3)(2)(0)#360 - q(4)(2)(0)#375
                                 - q(5)(2)(0)#390 - q(6)(2)(0)#405
                                 - q(7)(2)(0)#420 - q(8)(2)(0)#435
                                 - q(9)(2)(0)#450 - q(10)(2)(0)#465
                                 - q(11)(2)(0)#480 - q(12)(2)(0)#495
                                 - q(13)(2)(0)#510 - q(14)(2)(0)#525
                                 - q(15)(2)(0)#540 - q(16)(2)(0)#555
                                 - q(17)(2)(0)#570 - q(18)(2)(0)#585
                                 - q(19)(2)(0)#600 - q(20)(2)(0)#615
                                 - q(21)(2)(0)#630 - q(22)(2)(0)#645
                                 - q(23)(2)(0)#660 - q(24)(2)(0)#675
                                 - q(25)(2)(0)#690 - q(26)(2)(0)#705
                                 - q(27)(2)(0)#720 - q(28)(2)(0)#735
                                 - q(29)(2)(0)#750 - q(30)(2)(0)#765  = -87
 WarehouseInventoryBalance(4,1,10): I_warehouse(3)(0)(9)#108
                                 - b_warehouse(3)(0)(9)#258 - q(0)(3)(0)#318
                                 - q(1)(3)(0)#333 - q(2)(3)(0)#348
                                 - q(3)(3)(0)#363 - q(4)(3)(0)#378
                                 - q(5)(3)(0)#393 - q(6)(3)(0)#408
                                 - q(7)(3)(0)#423 - q(8)(3)(0)#438
                                 - q(9)(3)(0)#453 - q(10)(3)(0)#468
                                 - q(11)(3)(0)#483 - q(12)(3)(0)#498
                                 - q(13)(3)(0)#513 - q(14)(3)(0)#528
                                 - q(15)(3)(0)#543 - q(16)(3)(0)#558
                                 - q(17)(3)(0)#573 - q(18)(3)(0)#588
                                 - q(19)(3)(0)#603 - q(20)(3)(0)#618
                                 - q(21)(3)(0)#633 - q(22)(3)(0)#648
                                 - q(23)(3)(0)#663 - q(24)(3)(0)#678
                                 - q(25)(3)(0)#693 - q(26)(3)(0)#708
                                 - q(27)(3)(0)#723 - q(28)(3)(0)#738
                                 - q(29)(3)(0)#753 - q(30)(3)(0)#768  = -56
 WarehouseInventoryBalance(5,1,10): I_warehouse(4)(0)(9)#138
                                 - b_warehouse(4)(0)(9)#288 - q(0)(4)(0)#321
                                 - q(1)(4)(0)#336 - q(2)(4)(0)#351
                                 - q(3)(4)(0)#366 - q(4)(4)(0)#381
                                 - q(5)(4)(0)#396 - q(6)(4)(0)#411
                                 - q(7)(4)(0)#426 - q(8)(4)(0)#441
                                 - q(9)(4)(0)#456 - q(10)(4)(0)#471
                                 - q(11)(4)(0)#486 - q(12)(4)(0)#501
                                 - q(13)(4)(0)#516 - q(14)(4)(0)#531
                                 - q(15)(4)(0)#546 - q(16)(4)(0)#561
                                 - q(17)(4)(0)#576 - q(18)(4)(0)#591
                                 - q(19)(4)(0)#606 - q(20)(4)(0)#621
                                 - q(21)(4)(0)#636 - q(22)(4)(0)#651
                                 - q(23)(4)(0)#666 - q(24)(4)(0)#681
                                 - q(25)(4)(0)#696 - q(26)(4)(0)#711
                                 - q(27)(4)(0)#726 - q(28)(4)(0)#741
                                 - q(29)(4)(0)#756 - q(30)(4)(0)#771  = -147
 WarehouseInventoryBalance(1,2,10): - I_warehouse(0)(0)(9)#18
                                 + I_warehouse(0)(1)(9)#28
                                 - b_warehouse(0)(1)(9)#178 - q(0)(0)(1)#310
                                 - q(1)(0)(1)#325 - q(2)(0)(1)#340
                                 - q(3)(0)(1)#355 - q(4)(0)(1)#370
                                 - q(5)(0)(1)#385 - q(6)(0)(1)#400
                                 - q(7)(0)(1)#415 - q(8)(0)(1)#430
                                 - q(9)(0)(1)#445 - q(10)(0)(1)#460
                                 - q(11)(0)(1)#475 - q(12)(0)(1)#490
                                 - q(13)(0)(1)#505 - q(14)(0)(1)#520
                                 - q(15)(0)(1)#535 - q(16)(0)(1)#550
                                 - q(17)(0)(1)#565 - q(18)(0)(1)#580
                                 - q(19)(0)(1)#595 - q(20)(0)(1)#610
                                 - q(21)(0)(1)#625 - q(22)(0)(1)#640
                                 - q(23)(0)(1)#655 - q(24)(0)(1)#670
                                 - q(25)(0)(1)#685 - q(26)(0)(1)#700
                                 - q(27)(0)(1)#715 - q(28)(0)(1)#730
                                 - q(29)(0)(1)#745 - q(30)(0)(1)#760  = -137
 WarehouseInventoryBalance(2,2,10): - I_warehouse(1)(0)(9)#48
                                 + I_warehouse(1)(1)(9)#58
                                 - b_warehouse(1)(1)(9)#208 - q(0)(1)(1)#313
                                 - q(1)(1)(1)#328 - q(2)(1)(1)#343
                                 - q(3)(1)(1)#358 - q(4)(1)(1)#373
                                 - q(5)(1)(1)#388 - q(6)(1)(1)#403
                                 - q(7)(1)(1)#418 - q(8)(1)(1)#433
                                 - q(9)(1)(1)#448 - q(10)(1)(1)#463
                                 - q(11)(1)(1)#478 - q(12)(1)(1)#493
                                 - q(13)(1)(1)#508 - q(14)(1)(1)#523
                                 - q(15)(1)(1)#538 - q(16)(1)(1)#553
                                 - q(17)(1)(1)#568 - q(18)(1)(1)#583
                                 - q(19)(1)(1)#598 - q(20)(1)(1)#613
                                 - q(21)(1)(1)#628 - q(22)(1)(1)#643
                                 - q(23)(1)(1)#658 - q(24)(1)(1)#673
                                 - q(25)(1)(1)#688 - q(26)(1)(1)#703
                                 - q(27)(1)(1)#718 - q(28)(1)(1)#733
                                 - q(29)(1)(1)#748 - q(30)(1)(1)#763  = -49
 WarehouseInventoryBalance(3,2,10): - I_warehouse(2)(0)(9)#78
                                 + I_warehouse(2)(1)(9)#88
                                 - b_warehouse(2)(1)(9)#238 - q(0)(2)(1)#316
                                 - q(1)(2)(1)#331 - q(2)(2)(1)#346
                                 - q(3)(2)(1)#361 - q(4)(2)(1)#376
                                 - q(5)(2)(1)#391 - q(6)(2)(1)#406
                                 - q(7)(2)(1)#421 - q(8)(2)(1)#436
                                 - q(9)(2)(1)#451 - q(10)(2)(1)#466
                                 - q(11)(2)(1)#481 - q(12)(2)(1)#496
                                 - q(13)(2)(1)#511 - q(14)(2)(1)#526
                                 - q(15)(2)(1)#541 - q(16)(2)(1)#556
                                 - q(17)(2)(1)#571 - q(18)(2)(1)#586
                                 - q(19)(2)(1)#601 - q(20)(2)(1)#616
                                 - q(21)(2)(1)#631 - q(22)(2)(1)#646
                                 - q(23)(2)(1)#661 - q(24)(2)(1)#676
                                 - q(25)(2)(1)#691 - q(26)(2)(1)#706
                                 - q(27)(2)(1)#721 - q(28)(2)(1)#736
                                 - q(29)(2)(1)#751 - q(30)(2)(1)#766  = -98
 WarehouseInventoryBalance(4,2,10): - I_warehouse(3)(0)(9)#108
                                 + I_warehouse(3)(1)(9)#118
                                 - b_warehouse(3)(1)(9)#268 - q(0)(3)(1)#319
                                 - q(1)(3)(1)#334 - q(2)(3)(1)#349
                                 - q(3)(3)(1)#364 - q(4)(3)(1)#379
                                 - q(5)(3)(1)#394 - q(6)(3)(1)#409
                                 - q(7)(3)(1)#424 - q(8)(3)(1)#439
                                 - q(9)(3)(1)#454 - q(10)(3)(1)#469
                                 - q(11)(3)(1)#484 - q(12)(3)(1)#499
                                 - q(13)(3)(1)#514 - q(14)(3)(1)#529
                                 - q(15)(3)(1)#544 - q(16)(3)(1)#559
                                 - q(17)(3)(1)#574 - q(18)(3)(1)#589
                                 - q(19)(3)(1)#604 - q(20)(3)(1)#619
                                 - q(21)(3)(1)#634 - q(22)(3)(1)#649
                                 - q(23)(3)(1)#664 - q(24)(3)(1)#679
                                 - q(25)(3)(1)#694 - q(26)(3)(1)#709
                                 - q(27)(3)(1)#724 - q(28)(3)(1)#739
                                 - q(29)(3)(1)#754 - q(30)(3)(1)#769  = -59
 WarehouseInventoryBalance(5,2,10): - I_warehouse(4)(0)(9)#138
                                 + I_warehouse(4)(1)(9)#148
                                 - b_warehouse(4)(1)(9)#298 - q(0)(4)(1)#322
                                 - q(1)(4)(1)#337 - q(2)(4)(1)#352
                                 - q(3)(4)(1)#367 - q(4)(4)(1)#382
                                 - q(5)(4)(1)#397 - q(6)(4)(1)#412
                                 - q(7)(4)(1)#427 - q(8)(4)(1)#442
                                 - q(9)(4)(1)#457 - q(10)(4)(1)#472
                                 - q(11)(4)(1)#487 - q(12)(4)(1)#502
                                 - q(13)(4)(1)#517 - q(14)(4)(1)#532
                                 - q(15)(4)(1)#547 - q(16)(4)(1)#562
                                 - q(17)(4)(1)#577 - q(18)(4)(1)#592
                                 - q(19)(4)(1)#607 - q(20)(4)(1)#622
                                 - q(21)(4)(1)#637 - q(22)(4)(1)#652
                                 - q(23)(4)(1)#667 - q(24)(4)(1)#682
                                 - q(25)(4)(1)#697 - q(26)(4)(1)#712
                                 - q(27)(4)(1)#727 - q(28)(4)(1)#742
                                 - q(29)(4)(1)#757 - q(30)(4)(1)#772  = -148
 WarehouseInventoryBalance(1,3,10): - I_warehouse(0)(1)(9)#28
                                 + I_warehouse(0)(2)(9)#38
                                 - b_warehouse(0)(2)(9)#188 - q(0)(0)(2)#311
                                 - q(1)(0)(2)#326 - q(2)(0)(2)#341
                                 - q(3)(0)(2)#356 - q(4)(0)(2)#371
                                 - q(5)(0)(2)#386 - q(6)(0)(2)#401
                                 - q(7)(0)(2)#416 - q(8)(0)(2)#431
                                 - q(9)(0)(2)#446 - q(10)(0)(2)#461
                                 - q(11)(0)(2)#476 - q(12)(0)(2)#491
                                 - q(13)(0)(2)#506 - q(14)(0)(2)#521
                                 - q(15)(0)(2)#536 - q(16)(0)(2)#551
                                 - q(17)(0)(2)#566 - q(18)(0)(2)#581
                                 - q(19)(0)(2)#596 - q(20)(0)(2)#611
                                 - q(21)(0)(2)#626 - q(22)(0)(2)#641
                                 - q(23)(0)(2)#656 - q(24)(0)(2)#671
                                 - q(25)(0)(2)#686 - q(26)(0)(2)#701
                                 - q(27)(0)(2)#716 - q(28)(0)(2)#731
                                 - q(29)(0)(2)#746 - q(30)(0)(2)#761  = -139
 WarehouseInventoryBalance(2,3,10): - I_warehouse(1)(1)(9)#58
                                 + I_warehouse(1)(2)(9)#68
                                 - b_warehouse(1)(2)(9)#218 - q(0)(1)(2)#314
                                 - q(1)(1)(2)#329 - q(2)(1)(2)#344
                                 - q(3)(1)(2)#359 - q(4)(1)(2)#374
                                 - q(5)(1)(2)#389 - q(6)(1)(2)#404
                                 - q(7)(1)(2)#419 - q(8)(1)(2)#434
                                 - q(9)(1)(2)#449 - q(10)(1)(2)#464
                                 - q(11)(1)(2)#479 - q(12)(1)(2)#494
                                 - q(13)(1)(2)#509 - q(14)(1)(2)#524
                                 - q(15)(1)(2)#539 - q(16)(1)(2)#554
                                 - q(17)(1)(2)#569 - q(18)(1)(2)#584
                                 - q(19)(1)(2)#599 - q(20)(1)(2)#614
                                 - q(21)(1)(2)#629 - q(22)(1)(2)#644
                                 - q(23)(1)(2)#659 - q(24)(1)(2)#674
                                 - q(25)(1)(2)#689 - q(26)(1)(2)#704
                                 - q(27)(1)(2)#719 - q(28)(1)(2)#734
                                 - q(29)(1)(2)#749 - q(30)(1)(2)#764  = -50
 WarehouseInventoryBalance(3,3,10): - I_warehouse(2)(1)(9)#88
                                 + I_warehouse(2)(2)(9)#98
                                 - b_warehouse(2)(2)(9)#248 - q(0)(2)(2)#317
                                 - q(1)(2)(2)#332 - q(2)(2)(2)#347
                                 - q(3)(2)(2)#362 - q(4)(2)(2)#377
                                 - q(5)(2)(2)#392 - q(6)(2)(2)#407
                                 - q(7)(2)(2)#422 - q(8)(2)(2)#437
                                 - q(9)(2)(2)#452 - q(10)(2)(2)#467
                                 - q(11)(2)(2)#482 - q(12)(2)(2)#497
                                 - q(13)(2)(2)#512 - q(14)(2)(2)#527
                                 - q(15)(2)(2)#542 - q(16)(2)(2)#557
                                 - q(17)(2)(2)#572 - q(18)(2)(2)#587
                                 - q(19)(2)(2)#602 - q(20)(2)(2)#617
                                 - q(21)(2)(2)#632 - q(22)(2)(2)#647
                                 - q(23)(2)(2)#662 - q(24)(2)(2)#677
                                 - q(25)(2)(2)#692 - q(26)(2)(2)#707
                                 - q(27)(2)(2)#722 - q(28)(2)(2)#737
                                 - q(29)(2)(2)#752 - q(30)(2)(2)#767  = -85
 WarehouseInventoryBalance(4,3,10): - I_warehouse(3)(1)(9)#118
                                 + I_warehouse(3)(2)(9)#128
                                 - b_warehouse(3)(2)(9)#278 - q(0)(3)(2)#320
                                 - q(1)(3)(2)#335 - q(2)(3)(2)#350
                                 - q(3)(3)(2)#365 - q(4)(3)(2)#380
                                 - q(5)(3)(2)#395 - q(6)(3)(2)#410
                                 - q(7)(3)(2)#425 - q(8)(3)(2)#440
                                 - q(9)(3)(2)#455 - q(10)(3)(2)#470
                                 - q(11)(3)(2)#485 - q(12)(3)(2)#500
                                 - q(13)(3)(2)#515 - q(14)(3)(2)#530
                                 - q(15)(3)(2)#545 - q(16)(3)(2)#560
                                 - q(17)(3)(2)#575 - q(18)(3)(2)#590
                                 - q(19)(3)(2)#605 - q(20)(3)(2)#620
                                 - q(21)(3)(2)#635 - q(22)(3)(2)#650
                                 - q(23)(3)(2)#665 - q(24)(3)(2)#680
                                 - q(25)(3)(2)#695 - q(26)(3)(2)#710
                                 - q(27)(3)(2)#725 - q(28)(3)(2)#740
                                 - q(29)(3)(2)#755 - q(30)(3)(2)#770  = -56
 WarehouseInventoryBalance(5,3,10): - I_warehouse(4)(1)(9)#148
                                 + I_warehouse(4)(2)(9)#158
                                 - b_warehouse(4)(2)(9)#308 - q(0)(4)(2)#323
                                 - q(1)(4)(2)#338 - q(2)(4)(2)#353
                                 - q(3)(4)(2)#368 - q(4)(4)(2)#383
                                 - q(5)(4)(2)#398 - q(6)(4)(2)#413
                                 - q(7)(4)(2)#428 - q(8)(4)(2)#443
                                 - q(9)(4)(2)#458 - q(10)(4)(2)#473
                                 - q(11)(4)(2)#488 - q(12)(4)(2)#503
                                 - q(13)(4)(2)#518 - q(14)(4)(2)#533
                                 - q(15)(4)(2)#548 - q(16)(4)(2)#563
                                 - q(17)(4)(2)#578 - q(18)(4)(2)#593
                                 - q(19)(4)(2)#608 - q(20)(4)(2)#623
                                 - q(21)(4)(2)#638 - q(22)(4)(2)#653
                                 - q(23)(4)(2)#668 - q(24)(4)(2)#683
                                 - q(25)(4)(2)#698 - q(26)(4)(2)#713
                                 - q(27)(4)(2)#728 - q(28)(4)(2)#743
                                 - q(29)(4)(2)#758 - q(30)(4)(2)#773  = -132
 WarehouseCoverage(1,1):         o(0)(0)#774 + o(2)(0)#780 + o(4)(0)#786
                                 + o(6)(0)#792 + o(8)(0)#798 + o(10)(0)#804
                                 + o(12)(0)#810 + o(14)(0)#816 + o(16)(0)#822
                                 + o(18)(0)#828 + o(20)(0)#834 + o(22)(0)#840
                                 + o(24)(0)#846 + o(26)(0)#852 + o(28)(0)#858
                                 + o(30)(0)#864 <= 1
 WarehouseCoverage(2,1):         o(1)(0)#777 + o(2)(0)#780 + o(5)(0)#789
                                 + o(6)(0)#792 + o(9)(0)#801 + o(10)(0)#804
                                 + o(13)(0)#813 + o(14)(0)#816 + o(17)(0)#825
                                 + o(18)(0)#828 + o(21)(0)#837 + o(22)(0)#840
                                 + o(25)(0)#849 + o(26)(0)#852 + o(29)(0)#861
                                 + o(30)(0)#864 <= 1
 WarehouseCoverage(3,1):         o(3)(0)#783 + o(4)(0)#786 + o(5)(0)#789
                                 + o(6)(0)#792 + o(11)(0)#807 + o(12)(0)#810
                                 + o(13)(0)#813 + o(14)(0)#816 + o(19)(0)#831
                                 + o(20)(0)#834 + o(21)(0)#837 + o(22)(0)#840
                                 + o(27)(0)#855 + o(28)(0)#858 + o(29)(0)#861
                                 + o(30)(0)#864 <= 1
 WarehouseCoverage(4,1):         o(7)(0)#795 + o(8)(0)#798 + o(9)(0)#801
                                 + o(10)(0)#804 + o(11)(0)#807 + o(12)(0)#810
                                 + o(13)(0)#813 + o(14)(0)#816 + o(23)(0)#843
                                 + o(24)(0)#846 + o(25)(0)#849 + o(26)(0)#852
                                 + o(27)(0)#855 + o(28)(0)#858 + o(29)(0)#861
                                 + o(30)(0)#864 <= 1
 WarehouseCoverage(5,1):         o(15)(0)#819 + o(16)(0)#822 + o(17)(0)#825
                                 + o(18)(0)#828 + o(19)(0)#831 + o(20)(0)#834
                                 + o(21)(0)#837 + o(22)(0)#840 + o(23)(0)#843
                                 + o(24)(0)#846 + o(25)(0)#849 + o(26)(0)#852
                                 + o(27)(0)#855 + o(28)(0)#858 + o(29)(0)#861
                                 + o(30)(0)#864 <= 1
 WarehouseCoverage(1,2):         o(0)(1)#775 + o(2)(1)#781 + o(4)(1)#787
                                 + o(6)(1)#793 + o(8)(1)#799 + o(10)(1)#805
                                 + o(12)(1)#811 + o(14)(1)#817 + o(16)(1)#823
                                 + o(18)(1)#829 + o(20)(1)#835 + o(22)(1)#841
                                 + o(24)(1)#847 + o(26)(1)#853 + o(28)(1)#859
                                 + o(30)(1)#865 <= 1
 WarehouseCoverage(2,2):         o(1)(1)#778 + o(2)(1)#781 + o(5)(1)#790
                                 + o(6)(1)#793 + o(9)(1)#802 + o(10)(1)#805
                                 + o(13)(1)#814 + o(14)(1)#817 + o(17)(1)#826
                                 + o(18)(1)#829 + o(21)(1)#838 + o(22)(1)#841
                                 + o(25)(1)#850 + o(26)(1)#853 + o(29)(1)#862
                                 + o(30)(1)#865 <= 1
 WarehouseCoverage(3,2):         o(3)(1)#784 + o(4)(1)#787 + o(5)(1)#790
                                 + o(6)(1)#793 + o(11)(1)#808 + o(12)(1)#811
                                 + o(13)(1)#814 + o(14)(1)#817 + o(19)(1)#832
                                 + o(20)(1)#835 + o(21)(1)#838 + o(22)(1)#841
                                 + o(27)(1)#856 + o(28)(1)#859 + o(29)(1)#862
                                 + o(30)(1)#865 <= 1
 WarehouseCoverage(4,2):         o(7)(1)#796 + o(8)(1)#799 + o(9)(1)#802
                                 + o(10)(1)#805 + o(11)(1)#808 + o(12)(1)#811
                                 + o(13)(1)#814 + o(14)(1)#817 + o(23)(1)#844
                                 + o(24)(1)#847 + o(25)(1)#850 + o(26)(1)#853
                                 + o(27)(1)#856 + o(28)(1)#859 + o(29)(1)#862
                                 + o(30)(1)#865 <= 1
 WarehouseCoverage(5,2):         o(15)(1)#820 + o(16)(1)#823 + o(17)(1)#826
                                 + o(18)(1)#829 + o(19)(1)#832 + o(20)(1)#835
                                 + o(21)(1)#838 + o(22)(1)#841 + o(23)(1)#844
                                 + o(24)(1)#847 + o(25)(1)#850 + o(26)(1)#853
                                 + o(27)(1)#856 + o(28)(1)#859 + o(29)(1)#862
                                 + o(30)(1)#865 <= 1
 WarehouseCoverage(1,3):         o(0)(2)#776 + o(2)(2)#782 + o(4)(2)#788
                                 + o(6)(2)#794 + o(8)(2)#800 + o(10)(2)#806
                                 + o(12)(2)#812 + o(14)(2)#818 + o(16)(2)#824
                                 + o(18)(2)#830 + o(20)(2)#836 + o(22)(2)#842
                                 + o(24)(2)#848 + o(26)(2)#854 + o(28)(2)#860
                                 + o(30)(2)#866 <= 1
 WarehouseCoverage(2,3):         o(1)(2)#779 + o(2)(2)#782 + o(5)(2)#791
                                 + o(6)(2)#794 + o(9)(2)#803 + o(10)(2)#806
                                 + o(13)(2)#815 + o(14)(2)#818 + o(17)(2)#827
                                 + o(18)(2)#830 + o(21)(2)#839 + o(22)(2)#842
                                 + o(25)(2)#851 + o(26)(2)#854 + o(29)(2)#863
                                 + o(30)(2)#866 <= 1
 WarehouseCoverage(3,3):         o(3)(2)#785 + o(4)(2)#788 + o(5)(2)#791
                                 + o(6)(2)#794 + o(11)(2)#809 + o(12)(2)#812
                                 + o(13)(2)#815 + o(14)(2)#818 + o(19)(2)#833
                                 + o(20)(2)#836 + o(21)(2)#839 + o(22)(2)#842
                                 + o(27)(2)#857 + o(28)(2)#860 + o(29)(2)#863
                                 + o(30)(2)#866 <= 1
 WarehouseCoverage(4,3):         o(7)(2)#797 + o(8)(2)#800 + o(9)(2)#803
                                 + o(10)(2)#806 + o(11)(2)#809 + o(12)(2)#812
                                 + o(13)(2)#815 + o(14)(2)#818 + o(23)(2)#845
                                 + o(24)(2)#848 + o(25)(2)#851 + o(26)(2)#854
                                 + o(27)(2)#857 + o(28)(2)#860 + o(29)(2)#863
                                 + o(30)(2)#866 <= 1
 WarehouseCoverage(5,3):         o(15)(2)#821 + o(16)(2)#824 + o(17)(2)#827
                                 + o(18)(2)#830 + o(19)(2)#833 + o(20)(2)#836
                                 + o(21)(2)#839 + o(22)(2)#842 + o(23)(2)#845
                                 + o(24)(2)#848 + o(25)(2)#851 + o(26)(2)#854
                                 + o(27)(2)#857 + o(28)(2)#860 + o(29)(2)#863
                                 + o(30)(2)#866 <= 1
 VehicleCapacity(1,1):           q(0)(0)(0)#309 + q(0)(1)(0)#312
                                 + q(0)(2)(0)#315 + q(0)(3)(0)#318
                                 + q(0)(4)(0)#321 - 318 o(0)(0)#774 <= 0
 VehicleCapacity(2,1):           q(1)(0)(0)#324 + q(1)(1)(0)#327
                                 + q(1)(2)(0)#330 + q(1)(3)(0)#333
                                 + q(1)(4)(0)#336 - 318 o(1)(0)#777 <= 0
 VehicleCapacity(3,1):           q(2)(0)(0)#339 + q(2)(1)(0)#342
                                 + q(2)(2)(0)#345 + q(2)(3)(0)#348
                                 + q(2)(4)(0)#351 - 318 o(2)(0)#780 <= 0
 VehicleCapacity(4,1):           q(3)(0)(0)#354 + q(3)(1)(0)#357
                                 + q(3)(2)(0)#360 + q(3)(3)(0)#363
                                 + q(3)(4)(0)#366 - 318 o(3)(0)#783 <= 0
 VehicleCapacity(5,1):           q(4)(0)(0)#369 + q(4)(1)(0)#372
                                 + q(4)(2)(0)#375 + q(4)(3)(0)#378
                                 + q(4)(4)(0)#381 - 318 o(4)(0)#786 <= 0
 VehicleCapacity(6,1):           q(5)(0)(0)#384 + q(5)(1)(0)#387
                                 + q(5)(2)(0)#390 + q(5)(3)(0)#393
                                 + q(5)(4)(0)#396 - 318 o(5)(0)#789 <= 0
 VehicleCapacity(7,1):           q(6)(0)(0)#399 + q(6)(1)(0)#402
                                 + q(6)(2)(0)#405 + q(6)(3)(0)#408
                                 + q(6)(4)(0)#411 - 318 o(6)(0)#792 <= 0
 VehicleCapacity(8,1):           q(7)(0)(0)#414 + q(7)(1)(0)#417
                                 + q(7)(2)(0)#420 + q(7)(3)(0)#423
                                 + q(7)(4)(0)#426 - 318 o(7)(0)#795 <= 0
 VehicleCapacity(9,1):           q(8)(0)(0)#429 + q(8)(1)(0)#432
                                 + q(8)(2)(0)#435 + q(8)(3)(0)#438
                                 + q(8)(4)(0)#441 - 318 o(8)(0)#798 <= 0
 VehicleCapacity(10,1):          q(9)(0)(0)#444 + q(9)(1)(0)#447
                                 + q(9)(2)(0)#450 + q(9)(3)(0)#453
                                 + q(9)(4)(0)#456 - 318 o(9)(0)#801 <= 0
 VehicleCapacity(11,1):          q(10)(0)(0)#459 + q(10)(1)(0)#462
                                 + q(10)(2)(0)#465 + q(10)(3)(0)#468
                                 + q(10)(4)(0)#471 - 318 o(10)(0)#804 <= 0
 VehicleCapacity(12,1):          q(11)(0)(0)#474 + q(11)(1)(0)#477
                                 + q(11)(2)(0)#480 + q(11)(3)(0)#483
                                 + q(11)(4)(0)#486 - 318 o(11)(0)#807 <= 0
 VehicleCapacity(13,1):          q(12)(0)(0)#489 + q(12)(1)(0)#492
                                 + q(12)(2)(0)#495 + q(12)(3)(0)#498
                                 + q(12)(4)(0)#501 - 318 o(12)(0)#810 <= 0
 VehicleCapacity(14,1):          q(13)(0)(0)#504 + q(13)(1)(0)#507
                                 + q(13)(2)(0)#510 + q(13)(3)(0)#513
                                 + q(13)(4)(0)#516 - 318 o(13)(0)#813 <= 0
 VehicleCapacity(15,1):          q(14)(0)(0)#519 + q(14)(1)(0)#522
                                 + q(14)(2)(0)#525 + q(14)(3)(0)#528
                                 + q(14)(4)(0)#531 - 318 o(14)(0)#816 <= 0
 VehicleCapacity(16,1):          q(15)(0)(0)#534 + q(15)(1)(0)#537
                                 + q(15)(2)(0)#540 + q(15)(3)(0)#543
                                 + q(15)(4)(0)#546 - 318 o(15)(0)#819 <= 0
 VehicleCapacity(17,1):          q(16)(0)(0)#549 + q(16)(1)(0)#552
                                 + q(16)(2)(0)#555 + q(16)(3)(0)#558
                                 + q(16)(4)(0)#561 - 318 o(16)(0)#822 <= 0
 VehicleCapacity(18,1):          q(17)(0)(0)#564 + q(17)(1)(0)#567
                                 + q(17)(2)(0)#570 + q(17)(3)(0)#573
                                 + q(17)(4)(0)#576 - 318 o(17)(0)#825 <= 0
 VehicleCapacity(19,1):          q(18)(0)(0)#579 + q(18)(1)(0)#582
                                 + q(18)(2)(0)#585 + q(18)(3)(0)#588
                                 + q(18)(4)(0)#591 - 318 o(18)(0)#828 <= 0
 VehicleCapacity(20,1):          q(19)(0)(0)#594 + q(19)(1)(0)#597
                                 + q(19)(2)(0)#600 + q(19)(3)(0)#603
                                 + q(19)(4)(0)#606 - 318 o(19)(0)#831 <= 0
 VehicleCapacity(21,1):          q(20)(0)(0)#609 + q(20)(1)(0)#612
                                 + q(20)(2)(0)#615 + q(20)(3)(0)#618
                                 + q(20)(4)(0)#621 - 318 o(20)(0)#834 <= 0
 VehicleCapacity(22,1):          q(21)(0)(0)#624 + q(21)(1)(0)#627
                                 + q(21)(2)(0)#630 + q(21)(3)(0)#633
                                 + q(21)(4)(0)#636 - 318 o(21)(0)#837 <= 0
 VehicleCapacity(23,1):          q(22)(0)(0)#639 + q(22)(1)(0)#642
                                 + q(22)(2)(0)#645 + q(22)(3)(0)#648
                                 + q(22)(4)(0)#651 - 318 o(22)(0)#840 <= 0
 VehicleCapacity(24,1):          q(23)(0)(0)#654 + q(23)(1)(0)#657
                                 + q(23)(2)(0)#660 + q(23)(3)(0)#663
                                 + q(23)(4)(0)#666 - 318 o(23)(0)#843 <= 0
 VehicleCapacity(25,1):          q(24)(0)(0)#669 + q(24)(1)(0)#672
                                 + q(24)(2)(0)#675 + q(24)(3)(0)#678
                                 + q(24)(4)(0)#681 - 318 o(24)(0)#846 <= 0
 VehicleCapacity(26,1):          q(25)(0)(0)#684 + q(25)(1)(0)#687
                                 + q(25)(2)(0)#690 + q(25)(3)(0)#693
                                 + q(25)(4)(0)#696 - 318 o(25)(0)#849 <= 0
 VehicleCapacity(27,1):          q(26)(0)(0)#699 + q(26)(1)(0)#702
                                 + q(26)(2)(0)#705 + q(26)(3)(0)#708
                                 + q(26)(4)(0)#711 - 318 o(26)(0)#852 <= 0
 VehicleCapacity(28,1):          q(27)(0)(0)#714 + q(27)(1)(0)#717
                                 + q(27)(2)(0)#720 + q(27)(3)(0)#723
                                 + q(27)(4)(0)#726 - 318 o(27)(0)#855 <= 0
 VehicleCapacity(29,1):          q(28)(0)(0)#729 + q(28)(1)(0)#732
                                 + q(28)(2)(0)#735 + q(28)(3)(0)#738
                                 + q(28)(4)(0)#741 - 318 o(28)(0)#858 <= 0
 VehicleCapacity(30,1):          q(29)(0)(0)#744 + q(29)(1)(0)#747
                                 + q(29)(2)(0)#750 + q(29)(3)(0)#753
                                 + q(29)(4)(0)#756 - 318 o(29)(0)#861 <= 0
 VehicleCapacity(31,1):          q(30)(0)(0)#759 + q(30)(1)(0)#762
                                 + q(30)(2)(0)#765 + q(30)(3)(0)#768
                                 + q(30)(4)(0)#771 - 318 o(30)(0)#864 <= 0
 VehicleCapacity(1,2):           q(0)(0)(1)#310 + q(0)(1)(1)#313
                                 + q(0)(2)(1)#316 + q(0)(3)(1)#319
                                 + q(0)(4)(1)#322 - 318 o(0)(1)#775 <= 0
 VehicleCapacity(2,2):           q(1)(0)(1)#325 + q(1)(1)(1)#328
                                 + q(1)(2)(1)#331 + q(1)(3)(1)#334
                                 + q(1)(4)(1)#337 - 318 o(1)(1)#778 <= 0
 VehicleCapacity(3,2):           q(2)(0)(1)#340 + q(2)(1)(1)#343
                                 + q(2)(2)(1)#346 + q(2)(3)(1)#349
                                 + q(2)(4)(1)#352 - 318 o(2)(1)#781 <= 0
 VehicleCapacity(4,2):           q(3)(0)(1)#355 + q(3)(1)(1)#358
                                 + q(3)(2)(1)#361 + q(3)(3)(1)#364
                                 + q(3)(4)(1)#367 - 318 o(3)(1)#784 <= 0
 VehicleCapacity(5,2):           q(4)(0)(1)#370 + q(4)(1)(1)#373
                                 + q(4)(2)(1)#376 + q(4)(3)(1)#379
                                 + q(4)(4)(1)#382 - 318 o(4)(1)#787 <= 0
 VehicleCapacity(6,2):           q(5)(0)(1)#385 + q(5)(1)(1)#388
                                 + q(5)(2)(1)#391 + q(5)(3)(1)#394
                                 + q(5)(4)(1)#397 - 318 o(5)(1)#790 <= 0
 VehicleCapacity(7,2):           q(6)(0)(1)#400 + q(6)(1)(1)#403
                                 + q(6)(2)(1)#406 + q(6)(3)(1)#409
                                 + q(6)(4)(1)#412 - 318 o(6)(1)#793 <= 0
 VehicleCapacity(8,2):           q(7)(0)(1)#415 + q(7)(1)(1)#418
                                 + q(7)(2)(1)#421 + q(7)(3)(1)#424
                                 + q(7)(4)(1)#427 - 318 o(7)(1)#796 <= 0
 VehicleCapacity(9,2):           q(8)(0)(1)#430 + q(8)(1)(1)#433
                                 + q(8)(2)(1)#436 + q(8)(3)(1)#439
                                 + q(8)(4)(1)#442 - 318 o(8)(1)#799 <= 0
 VehicleCapacity(10,2):          q(9)(0)(1)#445 + q(9)(1)(1)#448
                                 + q(9)(2)(1)#451 + q(9)(3)(1)#454
                                 + q(9)(4)(1)#457 - 318 o(9)(1)#802 <= 0
 VehicleCapacity(11,2):          q(10)(0)(1)#460 + q(10)(1)(1)#463
                                 + q(10)(2)(1)#466 + q(10)(3)(1)#469
                                 + q(10)(4)(1)#472 - 318 o(10)(1)#805 <= 0
 VehicleCapacity(12,2):          q(11)(0)(1)#475 + q(11)(1)(1)#478
                                 + q(11)(2)(1)#481 + q(11)(3)(1)#484
                                 + q(11)(4)(1)#487 - 318 o(11)(1)#808 <= 0
 VehicleCapacity(13,2):          q(12)(0)(1)#490 + q(12)(1)(1)#493
                                 + q(12)(2)(1)#496 + q(12)(3)(1)#499
                                 + q(12)(4)(1)#502 - 318 o(12)(1)#811 <= 0
 VehicleCapacity(14,2):          q(13)(0)(1)#505 + q(13)(1)(1)#508
                                 + q(13)(2)(1)#511 + q(13)(3)(1)#514
                                 + q(13)(4)(1)#517 - 318 o(13)(1)#814 <= 0
 VehicleCapacity(15,2):          q(14)(0)(1)#520 + q(14)(1)(1)#523
                                 + q(14)(2)(1)#526 + q(14)(3)(1)#529
                                 + q(14)(4)(1)#532 - 318 o(14)(1)#817 <= 0
 VehicleCapacity(16,2):          q(15)(0)(1)#535 + q(15)(1)(1)#538
                                 + q(15)(2)(1)#541 + q(15)(3)(1)#544
                                 + q(15)(4)(1)#547 - 318 o(15)(1)#820 <= 0
 VehicleCapacity(17,2):          q(16)(0)(1)#550 + q(16)(1)(1)#553
                                 + q(16)(2)(1)#556 + q(16)(3)(1)#559
                                 + q(16)(4)(1)#562 - 318 o(16)(1)#823 <= 0
 VehicleCapacity(18,2):          q(17)(0)(1)#565 + q(17)(1)(1)#568
                                 + q(17)(2)(1)#571 + q(17)(3)(1)#574
                                 + q(17)(4)(1)#577 - 318 o(17)(1)#826 <= 0
 VehicleCapacity(19,2):          q(18)(0)(1)#580 + q(18)(1)(1)#583
                                 + q(18)(2)(1)#586 + q(18)(3)(1)#589
                                 + q(18)(4)(1)#592 - 318 o(18)(1)#829 <= 0
 VehicleCapacity(20,2):          q(19)(0)(1)#595 + q(19)(1)(1)#598
                                 + q(19)(2)(1)#601 + q(19)(3)(1)#604
                                 + q(19)(4)(1)#607 - 318 o(19)(1)#832 <= 0
 VehicleCapacity(21,2):          q(20)(0)(1)#610 + q(20)(1)(1)#613
                                 + q(20)(2)(1)#616 + q(20)(3)(1)#619
                                 + q(20)(4)(1)#622 - 318 o(20)(1)#835 <= 0
 VehicleCapacity(22,2):          q(21)(0)(1)#625 + q(21)(1)(1)#628
                                 + q(21)(2)(1)#631 + q(21)(3)(1)#634
                                 + q(21)(4)(1)#637 - 318 o(21)(1)#838 <= 0
 VehicleCapacity(23,2):          q(22)(0)(1)#640 + q(22)(1)(1)#643
                                 + q(22)(2)(1)#646 + q(22)(3)(1)#649
                                 + q(22)(4)(1)#652 - 318 o(22)(1)#841 <= 0
 VehicleCapacity(24,2):          q(23)(0)(1)#655 + q(23)(1)(1)#658
                                 + q(23)(2)(1)#661 + q(23)(3)(1)#664
                                 + q(23)(4)(1)#667 - 318 o(23)(1)#844 <= 0
 VehicleCapacity(25,2):          q(24)(0)(1)#670 + q(24)(1)(1)#673
                                 + q(24)(2)(1)#676 + q(24)(3)(1)#679
                                 + q(24)(4)(1)#682 - 318 o(24)(1)#847 <= 0
 VehicleCapacity(26,2):          q(25)(0)(1)#685 + q(25)(1)(1)#688
                                 + q(25)(2)(1)#691 + q(25)(3)(1)#694
                                 + q(25)(4)(1)#697 - 318 o(25)(1)#850 <= 0
 VehicleCapacity(27,2):          q(26)(0)(1)#700 + q(26)(1)(1)#703
                                 + q(26)(2)(1)#706 + q(26)(3)(1)#709
                                 + q(26)(4)(1)#712 - 318 o(26)(1)#853 <= 0
 VehicleCapacity(28,2):          q(27)(0)(1)#715 + q(27)(1)(1)#718
                                 + q(27)(2)(1)#721 + q(27)(3)(1)#724
                                 + q(27)(4)(1)#727 - 318 o(27)(1)#856 <= 0
 VehicleCapacity(29,2):          q(28)(0)(1)#730 + q(28)(1)(1)#733
                                 + q(28)(2)(1)#736 + q(28)(3)(1)#739
                                 + q(28)(4)(1)#742 - 318 o(28)(1)#859 <= 0
 VehicleCapacity(30,2):          q(29)(0)(1)#745 + q(29)(1)(1)#748
                                 + q(29)(2)(1)#751 + q(29)(3)(1)#754
                                 + q(29)(4)(1)#757 - 318 o(29)(1)#862 <= 0
 VehicleCapacity(31,2):          q(30)(0)(1)#760 + q(30)(1)(1)#763
                                 + q(30)(2)(1)#766 + q(30)(3)(1)#769
                                 + q(30)(4)(1)#772 - 318 o(30)(1)#865 <= 0
 VehicleCapacity(1,3):           q(0)(0)(2)#311 + q(0)(1)(2)#314
                                 + q(0)(2)(2)#317 + q(0)(3)(2)#320
                                 + q(0)(4)(2)#323 - 318 o(0)(2)#776 <= 0
 VehicleCapacity(2,3):           q(1)(0)(2)#326 + q(1)(1)(2)#329
                                 + q(1)(2)(2)#332 + q(1)(3)(2)#335
                                 + q(1)(4)(2)#338 - 318 o(1)(2)#779 <= 0
 VehicleCapacity(3,3):           q(2)(0)(2)#341 + q(2)(1)(2)#344
                                 + q(2)(2)(2)#347 + q(2)(3)(2)#350
                                 + q(2)(4)(2)#353 - 318 o(2)(2)#782 <= 0
 VehicleCapacity(4,3):           q(3)(0)(2)#356 + q(3)(1)(2)#359
                                 + q(3)(2)(2)#362 + q(3)(3)(2)#365
                                 + q(3)(4)(2)#368 - 318 o(3)(2)#785 <= 0
 VehicleCapacity(5,3):           q(4)(0)(2)#371 + q(4)(1)(2)#374
                                 + q(4)(2)(2)#377 + q(4)(3)(2)#380
                                 + q(4)(4)(2)#383 - 318 o(4)(2)#788 <= 0
 VehicleCapacity(6,3):           q(5)(0)(2)#386 + q(5)(1)(2)#389
                                 + q(5)(2)(2)#392 + q(5)(3)(2)#395
                                 + q(5)(4)(2)#398 - 318 o(5)(2)#791 <= 0
 VehicleCapacity(7,3):           q(6)(0)(2)#401 + q(6)(1)(2)#404
                                 + q(6)(2)(2)#407 + q(6)(3)(2)#410
                                 + q(6)(4)(2)#413 - 318 o(6)(2)#794 <= 0
 VehicleCapacity(8,3):           q(7)(0)(2)#416 + q(7)(1)(2)#419
                                 + q(7)(2)(2)#422 + q(7)(3)(2)#425
                                 + q(7)(4)(2)#428 - 318 o(7)(2)#797 <= 0
 VehicleCapacity(9,3):           q(8)(0)(2)#431 + q(8)(1)(2)#434
                                 + q(8)(2)(2)#437 + q(8)(3)(2)#440
                                 + q(8)(4)(2)#443 - 318 o(8)(2)#800 <= 0
 VehicleCapacity(10,3):          q(9)(0)(2)#446 + q(9)(1)(2)#449
                                 + q(9)(2)(2)#452 + q(9)(3)(2)#455
                                 + q(9)(4)(2)#458 - 318 o(9)(2)#803 <= 0
 VehicleCapacity(11,3):          q(10)(0)(2)#461 + q(10)(1)(2)#464
                                 + q(10)(2)(2)#467 + q(10)(3)(2)#470
                                 + q(10)(4)(2)#473 - 318 o(10)(2)#806 <= 0
 VehicleCapacity(12,3):          q(11)(0)(2)#476 + q(11)(1)(2)#479
                                 + q(11)(2)(2)#482 + q(11)(3)(2)#485
                                 + q(11)(4)(2)#488 - 318 o(11)(2)#809 <= 0
 VehicleCapacity(13,3):          q(12)(0)(2)#491 + q(12)(1)(2)#494
                                 + q(12)(2)(2)#497 + q(12)(3)(2)#500
                                 + q(12)(4)(2)#503 - 318 o(12)(2)#812 <= 0
 VehicleCapacity(14,3):          q(13)(0)(2)#506 + q(13)(1)(2)#509
                                 + q(13)(2)(2)#512 + q(13)(3)(2)#515
                                 + q(13)(4)(2)#518 - 318 o(13)(2)#815 <= 0
 VehicleCapacity(15,3):          q(14)(0)(2)#521 + q(14)(1)(2)#524
                                 + q(14)(2)(2)#527 + q(14)(3)(2)#530
                                 + q(14)(4)(2)#533 - 318 o(14)(2)#818 <= 0
 VehicleCapacity(16,3):          q(15)(0)(2)#536 + q(15)(1)(2)#539
                                 + q(15)(2)(2)#542 + q(15)(3)(2)#545
                                 + q(15)(4)(2)#548 - 318 o(15)(2)#821 <= 0
 VehicleCapacity(17,3):          q(16)(0)(2)#551 + q(16)(1)(2)#554
                                 + q(16)(2)(2)#557 + q(16)(3)(2)#560
                                 + q(16)(4)(2)#563 - 318 o(16)(2)#824 <= 0
 VehicleCapacity(18,3):          q(17)(0)(2)#566 + q(17)(1)(2)#569
                                 + q(17)(2)(2)#572 + q(17)(3)(2)#575
                                 + q(17)(4)(2)#578 - 318 o(17)(2)#827 <= 0
 VehicleCapacity(19,3):          q(18)(0)(2)#581 + q(18)(1)(2)#584
                                 + q(18)(2)(2)#587 + q(18)(3)(2)#590
                                 + q(18)(4)(2)#593 - 318 o(18)(2)#830 <= 0
 VehicleCapacity(20,3):          q(19)(0)(2)#596 + q(19)(1)(2)#599
                                 + q(19)(2)(2)#602 + q(19)(3)(2)#605
                                 + q(19)(4)(2)#608 - 318 o(19)(2)#833 <= 0
 VehicleCapacity(21,3):          q(20)(0)(2)#611 + q(20)(1)(2)#614
                                 + q(20)(2)(2)#617 + q(20)(3)(2)#620
                                 + q(20)(4)(2)#623 - 318 o(20)(2)#836 <= 0
 VehicleCapacity(22,3):          q(21)(0)(2)#626 + q(21)(1)(2)#629
                                 + q(21)(2)(2)#632 + q(21)(3)(2)#635
                                 + q(21)(4)(2)#638 - 318 o(21)(2)#839 <= 0
 VehicleCapacity(23,3):          q(22)(0)(2)#641 + q(22)(1)(2)#644
                                 + q(22)(2)(2)#647 + q(22)(3)(2)#650
                                 + q(22)(4)(2)#653 - 318 o(22)(2)#842 <= 0
 VehicleCapacity(24,3):          q(23)(0)(2)#656 + q(23)(1)(2)#659
                                 + q(23)(2)(2)#662 + q(23)(3)(2)#665
                                 + q(23)(4)(2)#668 - 318 o(23)(2)#845 <= 0
 VehicleCapacity(25,3):          q(24)(0)(2)#671 + q(24)(1)(2)#674
                                 + q(24)(2)(2)#677 + q(24)(3)(2)#680
                                 + q(24)(4)(2)#683 - 318 o(24)(2)#848 <= 0
 VehicleCapacity(26,3):          q(25)(0)(2)#686 + q(25)(1)(2)#689
                                 + q(25)(2)(2)#692 + q(25)(3)(2)#695
                                 + q(25)(4)(2)#698 - 318 o(25)(2)#851 <= 0
 VehicleCapacity(27,3):          q(26)(0)(2)#701 + q(26)(1)(2)#704
                                 + q(26)(2)(2)#707 + q(26)(3)(2)#710
                                 + q(26)(4)(2)#713 - 318 o(26)(2)#854 <= 0
 VehicleCapacity(28,3):          q(27)(0)(2)#716 + q(27)(1)(2)#719
                                 + q(27)(2)(2)#722 + q(27)(3)(2)#725
                                 + q(27)(4)(2)#728 - 318 o(27)(2)#857 <= 0
 VehicleCapacity(29,3):          q(28)(0)(2)#731 + q(28)(1)(2)#734
                                 + q(28)(2)(2)#737 + q(28)(3)(2)#740
                                 + q(28)(4)(2)#743 - 318 o(28)(2)#860 <= 0
 VehicleCapacity(30,3):          q(29)(0)(2)#746 + q(29)(1)(2)#749
                                 + q(29)(2)(2)#752 + q(29)(3)(2)#755
                                 + q(29)(4)(2)#758 - 318 o(29)(2)#863 <= 0
 VehicleCapacity(31,3):          q(30)(0)(2)#761 + q(30)(1)(2)#764
                                 + q(30)(2)(2)#767 + q(30)(3)(2)#770
                                 + q(30)(4)(2)#773 - 318 o(30)(2)#866 <= 0
 FleetSize(1):                   o(0)(0)#774 + o(1)(0)#777 + o(2)(0)#780
                                 + o(3)(0)#783 + o(4)(0)#786 + o(5)(0)#789
                                 + o(6)(0)#792 + o(7)(0)#795 + o(8)(0)#798
                                 + o(9)(0)#801 + o(10)(0)#804 + o(11)(0)#807
                                 + o(12)(0)#810 + o(13)(0)#813 + o(14)(0)#816
                                 + o(15)(0)#819 + o(16)(0)#822 + o(17)(0)#825
                                 + o(18)(0)#828 + o(19)(0)#831 + o(20)(0)#834
                                 + o(21)(0)#837 + o(22)(0)#840 + o(23)(0)#843
                                 + o(24)(0)#846 + o(25)(0)#849 + o(26)(0)#852
                                 + o(27)(0)#855 + o(28)(0)#858 + o(29)(0)#861
                                 + o(30)(0)#864 <= 2
 FleetSize(2):                   o(0)(1)#775 + o(1)(1)#778 + o(2)(1)#781
                                 + o(3)(1)#784 + o(4)(1)#787 + o(5)(1)#790
                                 + o(6)(1)#793 + o(7)(1)#796 + o(8)(1)#799
                                 + o(9)(1)#802 + o(10)(1)#805 + o(11)(1)#808
                                 + o(12)(1)#811 + o(13)(1)#814 + o(14)(1)#817
                                 + o(15)(1)#820 + o(16)(1)#823 + o(17)(1)#826
                                 + o(18)(1)#829 + o(19)(1)#832 + o(20)(1)#835
                                 + o(21)(1)#838 + o(22)(1)#841 + o(23)(1)#844
                                 + o(24)(1)#847 + o(25)(1)#850 + o(26)(1)#853
                                 + o(27)(1)#856 + o(28)(1)#859 + o(29)(1)#862
                                 + o(30)(1)#865 <= 2
 FleetSize(3):                   o(0)(2)#776 + o(1)(2)#779 + o(2)(2)#782
                                 + o(3)(2)#785 + o(4)(2)#788 + o(5)(2)#791
                                 + o(6)(2)#794 + o(7)(2)#797 + o(8)(2)#800
                                 + o(9)(2)#803 + o(10)(2)#806 + o(11)(2)#809
                                 + o(12)(2)#812 + o(13)(2)#815 + o(14)(2)#818
                                 + o(15)(2)#821 + o(16)(2)#824 + o(17)(2)#827
                                 + o(18)(2)#830 + o(19)(2)#833 + o(20)(2)#836
                                 + o(21)(2)#839 + o(22)(2)#842 + o(23)(2)#845
                                 + o(24)(2)#848 + o(25)(2)#851 + o(26)(2)#854
                                 + o(27)(2)#857 + o(28)(2)#860 + o(29)(2)#863
                                 + o(30)(2)#866 <= 2
Bounds
 0 <= y(0)#0 <= 1
 0 <= y(1)#1 <= 1
 0 <= y(2)#2 <= 1
 0 <= p(0)#3 <= 944
 0 <= p(1)#4 <= 944
 0 <= p(2)#5 <= 944
 0 <= I_plant(0)#6 <= 472
 0 <= I_plant(1)#7 <= 472
 0 <= I_plant(2)#8 <= 472
 0 <= I_warehouse(0)(0)(0)#9 <= 424.8
 0 <= I_warehouse(0)(0)(1)#10 <= 424.8
 0 <= I_warehouse(0)(0)(2)#11 <= 424.8
 0 <= I_warehouse(0)(0)(3)#12 <= 424.8
 0 <= I_warehouse(0)(0)(4)#13 <= 424.8
 0 <= I_warehouse(0)(0)(5)#14 <= 424.8
 0 <= I_warehouse(0)(0)(6)#15 <= 424.8
 0 <= I_warehouse(0)(0)(7)#16 <= 424.8
 0 <= I_warehouse(0)(0)(8)#17 <= 424.8
 0 <= I_warehouse(0)(0)(9)#18 <= 424.8
 0 <= I_warehouse(0)(1)(0)#19 <= 424.8
 0 <= I_warehouse(0)(1)(1)#20 <= 424.8
 0 <= I_warehouse(0)(1)(2)#21 <= 424.8
 0 <= I_warehouse(0)(1)(3)#22 <= 424.8
 0 <= I_warehouse(0)(1)(4)#23 <= 424.8
 0 <= I_warehouse(0)(1)(5)#24 <= 424.8
 0 <= I_warehouse(0)(1)(6)#25 <= 424.8
 0 <= I_warehouse(0)(1)(7)#26 <= 424.8
 0 <= I_warehouse(0)(1)(8)#27 <= 424.8
 0 <= I_warehouse(0)(1)(9)#28 <= 424.8
 0 <= I_warehouse(0)(2)(0)#29 <= 424.8
 0 <= I_warehouse(0)(2)(1)#30 <= 424.8
 0 <= I_warehouse(0)(2)(2)#31 <= 424.8
 0 <= I_warehouse(0)(2)(3)#32 <= 424.8
 0 <= I_warehouse(0)(2)(4)#33 <= 424.8
 0 <= I_warehouse(0)(2)(5)#34 <= 424.8
 0 <= I_warehouse(0)(2)(6)#35 <= 424.8
 0 <= I_warehouse(0)(2)(7)#36 <= 424.8
 0 <= I_warehouse(0)(2)(8)#37 <= 424.8
 0 <= I_warehouse(0)(2)(9)#38 <= 424.8
 0 <= I_warehouse(1)(0)(0)#39 <= 424.8
 0 <= I_warehouse(1)(0)(1)#40 <= 424.8
 0 <= I_warehouse(1)(0)(2)#41 <= 424.8
 0 <= I_warehouse(1)(0)(3)#42 <= 424.8
 0 <= I_warehouse(1)(0)(4)#43 <= 424.8
 0 <= I_warehouse(1)(0)(5)#44 <= 424.8
 0 <= I_warehouse(1)(0)(6)#45 <= 424.8
 0 <= I_warehouse(1)(0)(7)#46 <= 424.8
 0 <= I_warehouse(1)(0)(8)#47 <= 424.8
 0 <= I_warehouse(1)(0)(9)#48 <= 424.8
 0 <= I_warehouse(1)(1)(0)#49 <= 424.8
 0 <= I_warehouse(1)(1)(1)#50 <= 424.8
 0 <= I_warehouse(1)(1)(2)#51 <= 424.8
 0 <= I_warehouse(1)(1)(3)#52 <= 424.8
 0 <= I_warehouse(1)(1)(4)#53 <= 424.8
 0 <= I_warehouse(1)(1)(5)#54 <= 424.8
 0 <= I_warehouse(1)(1)(6)#55 <= 424.8
 0 <= I_warehouse(1)(1)(7)#56 <= 424.8
 0 <= I_warehouse(1)(1)(8)#57 <= 424.8
 0 <= I_warehouse(1)(1)(9)#58 <= 424.8
 0 <= I_warehouse(1)(2)(0)#59 <= 424.8
 0 <= I_warehouse(1)(2)(1)#60 <= 424.8
 0 <= I_warehouse(1)(2)(2)#61 <= 424.8
 0 <= I_warehouse(1)(2)(3)#62 <= 424.8
 0 <= I_warehouse(1)(2)(4)#63 <= 424.8
 0 <= I_warehouse(1)(2)(5)#64 <= 424.8
 0 <= I_warehouse(1)(2)(6)#65 <= 424.8
 0 <= I_warehouse(1)(2)(7)#66 <= 424.8
 0 <= I_warehouse(1)(2)(8)#67 <= 424.8
 0 <= I_warehouse(1)(2)(9)#68 <= 424.8
 0 <= I_warehouse(2)(0)(0)#69 <= 424.8
 0 <= I_warehouse(2)(0)(1)#70 <= 424.8
 0 <= I_warehouse(2)(0)(2)#71 <= 424.8
 0 <= I_warehouse(2)(0)(3)#72 <= 424.8
 0 <= I_warehouse(2)(0)(4)#73 <= 424.8
 0 <= I_warehouse(2)(0)(5)#74 <= 424.8
 0 <= I_warehouse(2)(0)(6)#75 <= 424.8
 0 <= I_warehouse(2)(0)(7)#76 <= 424.8
 0 <= I_warehouse(2)(0)(8)#77 <= 424.8
 0 <= I_warehouse(2)(0)(9)#78 <= 424.8
 0 <= I_warehouse(2)(1)(0)#79 <= 424.8
 0 <= I_warehouse(2)(1)(1)#80 <= 424.8
 0 <= I_warehouse(2)(1)(2)#81 <= 424.8
 0 <= I_warehouse(2)(1)(3)#82 <= 424.8
 0 <= I_warehouse(2)(1)(4)#83 <= 424.8
 0 <= I_warehouse(2)(1)(5)#84 <= 424.8
 0 <= I_warehouse(2)(1)(6)#85 <= 424.8
 0 <= I_warehouse(2)(1)(7)#86 <= 424.8
 0 <= I_warehouse(2)(1)(8)#87 <= 424.8
 0 <= I_warehouse(2)(1)(9)#88 <= 424.8
 0 <= I_warehouse(2)(2)(0)#89 <= 424.8
 0 <= I_warehouse(2)(2)(1)#90 <= 424.8
 0 <= I_warehouse(2)(2)(2)#91 <= 424.8
 0 <= I_warehouse(2)(2)(3)#92 <= 424.8
 0 <= I_warehouse(2)(2)(4)#93 <= 424.8
 0 <= I_warehouse(2)(2)(5)#94 <= 424.8
 0 <= I_warehouse(2)(2)(6)#95 <= 424.8
 0 <= I_warehouse(2)(2)(7)#96 <= 424.8
 0 <= I_warehouse(2)(2)(8)#97 <= 424.8
 0 <= I_warehouse(2)(2)(9)#98 <= 424.8
 0 <= I_warehouse(3)(0)(0)#99 <= 424.8
 0 <= I_warehouse(3)(0)(1)#100 <= 424.8
 0 <= I_warehouse(3)(0)(2)#101 <= 424.8
 0 <= I_warehouse(3)(0)(3)#102 <= 424.8
 0 <= I_warehouse(3)(0)(4)#103 <= 424.8
 0 <= I_warehouse(3)(0)(5)#104 <= 424.8
 0 <= I_warehouse(3)(0)(6)#105 <= 424.8
 0 <= I_warehouse(3)(0)(7)#106 <= 424.8
 0 <= I_warehouse(3)(0)(8)#107 <= 424.8
 0 <= I_warehouse(3)(0)(9)#108 <= 424.8
 0 <= I_warehouse(3)(1)(0)#109 <= 424.8
 0 <= I_warehouse(3)(1)(1)#110 <= 424.8
 0 <= I_warehouse(3)(1)(2)#111 <= 424.8
 0 <= I_warehouse(3)(1)(3)#112 <= 424.8
 0 <= I_warehouse(3)(1)(4)#113 <= 424.8
 0 <= I_warehouse(3)(1)(5)#114 <= 424.8
 0 <= I_warehouse(3)(1)(6)#115 <= 424.8
 0 <= I_warehouse(3)(1)(7)#116 <= 424.8
 0 <= I_warehouse(3)(1)(8)#117 <= 424.8
 0 <= I_warehouse(3)(1)(9)#118 <= 424.8
 0 <= I_warehouse(3)(2)(0)#119 <= 424.8
 0 <= I_warehouse(3)(2)(1)#120 <= 424.8
 0 <= I_warehouse(3)(2)(2)#121 <= 424.8
 0 <= I_warehouse(3)(2)(3)#122 <= 424.8
 0 <= I_warehouse(3)(2)(4)#123 <= 424.8
 0 <= I_warehouse(3)(2)(5)#124 <= 424.8
 0 <= I_warehouse(3)(2)(6)#125 <= 424.8
 0 <= I_warehouse(3)(2)(7)#126 <= 424.8
 0 <= I_warehouse(3)(2)(8)#127 <= 424.8
 0 <= I_warehouse(3)(2)(9)#128 <= 424.8
 0 <= I_warehouse(4)(0)(0)#129 <= 424.8
 0 <= I_warehouse(4)(0)(1)#130 <= 424.8
 0 <= I_warehouse(4)(0)(2)#131 <= 424.8
 0 <= I_warehouse(4)(0)(3)#132 <= 424.8
 0 <= I_warehouse(4)(0)(4)#133 <= 424.8
 0 <= I_warehouse(4)(0)(5)#134 <= 424.8
 0 <= I_warehouse(4)(0)(6)#135 <= 424.8
 0 <= I_warehouse(4)(0)(7)#136 <= 424.8
 0 <= I_warehouse(4)(0)(8)#137 <= 424.8
 0 <= I_warehouse(4)(0)(9)#138 <= 424.8
 0 <= I_warehouse(4)(1)(0)#139 <= 424.8
 0 <= I_warehouse(4)(1)(1)#140 <= 424.8
 0 <= I_warehouse(4)(1)(2)#141 <= 424.8
 0 <= I_warehouse(4)(1)(3)#142 <= 424.8
 0 <= I_warehouse(4)(1)(4)#143 <= 424.8
 0 <= I_warehouse(4)(1)(5)#144 <= 424.8
 0 <= I_warehouse(4)(1)(6)#145 <= 424.8
 0 <= I_warehouse(4)(1)(7)#146 <= 424.8
 0 <= I_warehouse(4)(1)(8)#147 <= 424.8
 0 <= I_warehouse(4)(1)(9)#148 <= 424.8
 0 <= I_warehouse(4)(2)(0)#149 <= 424.8
 0 <= I_warehouse(4)(2)(1)#150 <= 424.8
 0 <= I_warehouse(4)(2)(2)#151 <= 424.8
 0 <= I_warehouse(4)(2)(3)#152 <= 424.8
 0 <= I_warehouse(4)(2)(4)#153 <= 424.8
 0 <= I_warehouse(4)(2)(5)#154 <= 424.8
 0 <= I_warehouse(4)(2)(6)#155 <= 424.8
 0 <= I_warehouse(4)(2)(7)#156 <= 424.8
 0 <= I_warehouse(4)(2)(8)#157 <= 424.8
 0 <= I_warehouse(4)(2)(9)#158 <= 424.8
 0 <= b_warehouse(0)(0)(0)#159 <= 3
 0 <= b_warehouse(0)(0)(1)#160 <= 143
 0 <= b_warehouse(0)(0)(2)#161 <= 132
 0 <= b_warehouse(0)(0)(3)#162 <= 135
 0 <= b_warehouse(0)(0)(4)#163 <= 138
 0 <= b_warehouse(0)(0)(5)#164 <= 141
 0 <= b_warehouse(0)(0)(6)#165 <= 131
 0 <= b_warehouse(0)(0)(7)#166 <= 139
 0 <= b_warehouse(0)(0)(8)#167 <= 128
 0 <= b_warehouse(0)(0)(9)#168 <= 137
 0 <= b_warehouse(0)(1)(0)#169 <= 34
 0 <= b_warehouse(0)(1)(1)#170 <= 147
 0 <= b_warehouse(0)(1)(2)#171 <= 133
 0 <= b_warehouse(0)(1)(3)#172 <= 148
 0 <= b_warehouse(0)(1)(4)#173 <= 126
 0 <= b_warehouse(0)(1)(5)#174 <= 134
 0 <= b_warehouse(0)(1)(6)#175 <= 129
 0 <= b_warehouse(0)(1)(7)#176 <= 129
 0 <= b_warehouse(0)(1)(8)#177 <= 130
 0 <= b_warehouse(0)(1)(9)#178 <= 137
 0 <= b_warehouse(0)(2)(0)#179 <= 104
 0 <= b_warehouse(0)(2)(1)#180 <= 130
 0 <= b_warehouse(0)(2)(2)#181 <= 128
 0 <= b_warehouse(0)(2)(3)#182 <= 135
 0 <= b_warehouse(0)(2)(4)#183 <= 139
 0 <= b_warehouse(0)(2)(5)#184 <= 131
 0 <= b_warehouse(0)(2)(6)#185 <= 142
 0 <= b_warehouse(0)(2)(7)#186 <= 132
 0 <= b_warehouse(0)(2)(8)#187 <= 132
 0 <= b_warehouse(0)(2)(9)#188 <= 139
 0 <= b_warehouse(1)(0)(0)#189 <= 18
 0 <= b_warehouse(1)(0)(1)#190 <= 52
 0 <= b_warehouse(1)(0)(2)#191 <= 44
 0 <= b_warehouse(1)(0)(3)#192 <= 50
 0 <= b_warehouse(1)(0)(4)#193 <= 44
 0 <= b_warehouse(1)(0)(5)#194 <= 45
 0 <= b_warehouse(1)(0)(6)#195 <= 52
 0 <= b_warehouse(1)(0)(7)#196 <= 48
 0 <= b_warehouse(1)(0)(8)#197 <= 45
 0 <= b_warehouse(1)(0)(9)#198 <= 45
 0 <= b_warehouse(1)(1)(0)#199 <= 47
 0 <= b_warehouse(1)(1)(1)#200 <= 50
 0 <= b_warehouse(1)(1)(2)#201 <= 54
 0 <= b_warehouse(1)(1)(3)#202 <= 50
 0 <= b_warehouse(1)(1)(4)#203 <= 50
 0 <= b_warehouse(1)(1)(5)#204 <= 55
 0 <= b_warehouse(1)(1)(6)#205 <= 48
 0 <= b_warehouse(1)(1)(7)#206 <= 51
 0 <= b_warehouse(1)(1)(8)#207 <= 52
 0 <= b_warehouse(1)(1)(9)#208 <= 49
 0 <= b_warehouse(1)(2)(0)#209 <= 57
 0 <= b_warehouse(1)(2)(1)#210 <= 48
 0 <= b_warehouse(1)(2)(2)#211 <= 50
 0 <= b_warehouse(1)(2)(3)#212 <= 48
 0 <= b_warehouse(1)(2)(4)#213 <= 53
 0 <= b_warehouse(1)(2)(5)#214 <= 54
 0 <= b_warehouse(1)(2)(6)#215 <= 53
 0 <= b_warehouse(1)(2)(7)#216 <= 48
 0 <= b_warehouse(1)(2)(8)#217 <= 53
 0 <= b_warehouse(1)(2)(9)#218 <= 50
 0 <= b_warehouse(2)(0)(0)#219 <= 6
 0 <= b_warehouse(2)(0)(1)#220 <= 86
 0 <= b_warehouse(2)(0)(2)#221 <= 87
 0 <= b_warehouse(2)(0)(3)#222 <= 88
 0 <= b_warehouse(2)(0)(4)#223 <= 84
 0 <= b_warehouse(2)(0)(5)#224 <= 85
 0 <= b_warehouse(2)(0)(6)#225 <= 93
 0 <= b_warehouse(2)(0)(7)#226 <= 90
 0 <= b_warehouse(2)(0)(8)#227 <= 95
 0 <= b_warehouse(2)(0)(9)#228 <= 87
 0 <= b_warehouse(2)(1)(0)#229 <= 49
 0 <= b_warehouse(2)(1)(1)#230 <= 86
 0 <= b_warehouse(2)(1)(2)#231 <= 92
 0 <= b_warehouse(2)(1)(3)#232 <= 84
 0 <= b_warehouse(2)(1)(4)#233 <= 92
 0 <= b_warehouse(2)(1)(5)#234 <= 81
 0 <= b_warehouse(2)(1)(6)#235 <= 90
 0 <= b_warehouse(2)(1)(7)#236 <= 87
 0 <= b_warehouse(2)(1)(8)#237 <= 87
 0 <= b_warehouse(2)(1)(9)#238 <= 98
 0 <= b_warehouse(2)(2)(0)#239 <= 69
 0 <= b_warehouse(2)(2)(1)#240 <= 82
 0 <= b_warehouse(2)(2)(2)#241 <= 96
 0 <= b_warehouse(2)(2)(3)#242 <= 88
 0 <= b_warehouse(2)(2)(4)#243 <= 84
 0 <= b_warehouse(2)(2)(5)#244 <= 89
 0 <= b_warehouse(2)(2)(6)#245 <= 89
 0 <= b_warehouse(2)(2)(7)#246 <= 88
 0 <= b_warehouse(2)(2)(8)#247 <= 83
 0 <= b_warehouse(2)(2)(9)#248 <= 85
 0 <= b_warehouse(3)(0)(0)#249 <= 15
 0 <= b_warehouse(3)(0)(1)#250 <= 53
 0 <= b_warehouse(3)(0)(2)#251 <= 60
 0 <= b_warehouse(3)(0)(3)#252 <= 57
 0 <= b_warehouse(3)(0)(4)#253 <= 54
 0 <= b_warehouse(3)(0)(5)#254 <= 58
 0 <= b_warehouse(3)(0)(6)#255 <= 56
 0 <= b_warehouse(3)(0)(7)#256 <= 56
 0 <= b_warehouse(3)(0)(8)#257 <= 50
 0 <= b_warehouse(3)(0)(9)#258 <= 56
 0 <= b_warehouse(3)(1)(0)#259 <= 37
 0 <= b_warehouse(3)(1)(1)#260 <= 52
 0 <= b_warehouse(3)(1)(2)#261 <= 54
 0 <= b_warehouse(3)(1)(3)#262 <= 54
 0 <= b_warehouse(3)(1)(4)#263 <= 51
 0 <= b_warehouse(3)(1)(5)#264 <= 51
 0 <= b_warehouse(3)(1)(6)#265 <= 49
 0 <= b_warehouse(3)(1)(7)#266 <= 50
 0 <= b_warehouse(3)(1)(8)#267 <= 59
 0 <= b_warehouse(3)(1)(9)#268 <= 59
 0 <= b_warehouse(3)(2)(0)#269 <= 37
 0 <= b_warehouse(3)(2)(1)#270 <= 60
 0 <= b_warehouse(3)(2)(2)#271 <= 53
 0 <= b_warehouse(3)(2)(3)#272 <= 56
 0 <= b_warehouse(3)(2)(4)#273 <= 64
 0 <= b_warehouse(3)(2)(5)#274 <= 51
 0 <= b_warehouse(3)(2)(6)#275 <= 54
 0 <= b_warehouse(3)(2)(7)#276 <= 63
 0 <= b_warehouse(3)(2)(8)#277 <= 50
 0 <= b_warehouse(3)(2)(9)#278 <= 56
 0 <= b_warehouse(4)(0)(0)#279 <= 23
 0 <= b_warehouse(4)(0)(1)#280 <= 141
 0 <= b_warehouse(4)(0)(2)#281 <= 154
 0 <= b_warehouse(4)(0)(3)#282 <= 135
 0 <= b_warehouse(4)(0)(4)#283 <= 145
 0 <= b_warehouse(4)(0)(5)#284 <= 150
 0 <= b_warehouse(4)(0)(6)#285 <= 142
 0 <= b_warehouse(4)(0)(7)#286 <= 138
 0 <= b_warehouse(4)(0)(8)#287 <= 150
 0 <= b_warehouse(4)(0)(9)#288 <= 147
 0 <= b_warehouse(4)(1)(0)#289 <= 58
 0 <= b_warehouse(4)(1)(1)#290 <= 139
 0 <= b_warehouse(4)(1)(2)#291 <= 136
 0 <= b_warehouse(4)(1)(3)#292 <= 136
 0 <= b_warehouse(4)(1)(4)#293 <= 142
 0 <= b_warehouse(4)(1)(5)#294 <= 135
 0 <= b_warehouse(4)(1)(6)#295 <= 150
 0 <= b_warehouse(4)(1)(7)#296 <= 132
 0 <= b_warehouse(4)(1)(8)#297 <= 143
 0 <= b_warehouse(4)(1)(9)#298 <= 148
 0 <= b_warehouse(4)(2)(0)#299 <= 119
 0 <= b_warehouse(4)(2)(1)#300 <= 150
 0 <= b_warehouse(4)(2)(2)#301 <= 148
 0 <= b_warehouse(4)(2)(3)#302 <= 150
 0 <= b_warehouse(4)(2)(4)#303 <= 143
 0 <= b_warehouse(4)(2)(5)#304 <= 153
 0 <= b_warehouse(4)(2)(6)#305 <= 148
 0 <= b_warehouse(4)(2)(7)#306 <= 136
 0 <= b_warehouse(4)(2)(8)#307 <= 143
 0 <= b_warehouse(4)(2)(9)#308 <= 132
 0 <= q(0)(0)(0)#309 <= 424.8
 0 <= q(0)(0)(1)#310 <= 424.8
 0 <= q(0)(0)(2)#311 <= 424.8
      q(0)(1)(0)#312 = 0
      q(0)(1)(1)#313 = 0
      q(0)(1)(2)#314 = 0
      q(0)(2)(0)#315 = 0
      q(0)(2)(1)#316 = 0
      q(0)(2)(2)#317 = 0
      q(0)(3)(0)#318 = 0
      q(0)(3)(1)#319 = 0
      q(0)(3)(2)#320 = 0
      q(0)(4)(0)#321 = 0
      q(0)(4)(1)#322 = 0
      q(0)(4)(2)#323 = 0
      q(1)(0)(0)#324 = 0
      q(1)(0)(1)#325 = 0
      q(1)(0)(2)#326 = 0
 0 <= q(1)(1)(0)#327 <= 424.8
 0 <= q(1)(1)(1)#328 <= 424.8
 0 <= q(1)(1)(2)#329 <= 424.8
      q(1)(2)(0)#330 = 0
      q(1)(2)(1)#331 = 0
      q(1)(2)(2)#332 = 0
      q(1)(3)(0)#333 = 0
      q(1)(3)(1)#334 = 0
      q(1)(3)(2)#335 = 0
      q(1)(4)(0)#336 = 0
      q(1)(4)(1)#337 = 0
      q(1)(4)(2)#338 = 0
 0 <= q(2)(0)(0)#339 <= 424.8
 0 <= q(2)(0)(1)#340 <= 424.8
 0 <= q(2)(0)(2)#341 <= 424.8
 0 <= q(2)(1)(0)#342 <= 424.8
 0 <= q(2)(1)(1)#343 <= 424.8
 0 <= q(2)(1)(2)#344 <= 424.8
      q(2)(2)(0)#345 = 0
      q(2)(2)(1)#346 = 0
      q(2)(2)(2)#347 = 0
      q(2)(3)(0)#348 = 0
      q(2)(3)(1)#349 = 0
      q(2)(3)(2)#350 = 0
      q(2)(4)(0)#351 = 0
      q(2)(4)(1)#352 = 0
      q(2)(4)(2)#353 = 0
      q(3)(0)(0)#354 = 0
      q(3)(0)(1)#355 = 0
      q(3)(0)(2)#356 = 0
      q(3)(1)(0)#357 = 0
      q(3)(1)(1)#358 = 0
      q(3)(1)(2)#359 = 0
 0 <= q(3)(2)(0)#360 <= 424.8
 0 <= q(3)(2)(1)#361 <= 424.8
 0 <= q(3)(2)(2)#362 <= 424.8
      q(3)(3)(0)#363 = 0
      q(3)(3)(1)#364 = 0
      q(3)(3)(2)#365 = 0
      q(3)(4)(0)#366 = 0
      q(3)(4)(1)#367 = 0
      q(3)(4)(2)#368 = 0
 0 <= q(4)(0)(0)#369 <= 424.8
 0 <= q(4)(0)(1)#370 <= 424.8
 0 <= q(4)(0)(2)#371 <= 424.8
      q(4)(1)(0)#372 = 0
      q(4)(1)(1)#373 = 0
      q(4)(1)(2)#374 = 0
 0 <= q(4)(2)(0)#375 <= 424.8
 0 <= q(4)(2)(1)#376 <= 424.8
 0 <= q(4)(2)(2)#377 <= 424.8
      q(4)(3)(0)#378 = 0
      q(4)(3)(1)#379 = 0
      q(4)(3)(2)#380 = 0
      q(4)(4)(0)#381 = 0
      q(4)(4)(1)#382 = 0
      q(4)(4)(2)#383 = 0
      q(5)(0)(0)#384 = 0
      q(5)(0)(1)#385 = 0
      q(5)(0)(2)#386 = 0
 0 <= q(5)(1)(0)#387 <= 424.8
 0 <= q(5)(1)(1)#388 <= 424.8
 0 <= q(5)(1)(2)#389 <= 424.8
 0 <= q(5)(2)(0)#390 <= 424.8
 0 <= q(5)(2)(1)#391 <= 424.8
 0 <= q(5)(2)(2)#392 <= 424.8
      q(5)(3)(0)#393 = 0
      q(5)(3)(1)#394 = 0
      q(5)(3)(2)#395 = 0
      q(5)(4)(0)#396 = 0
      q(5)(4)(1)#397 = 0
      q(5)(4)(2)#398 = 0
 0 <= q(6)(0)(0)#399 <= 424.8
 0 <= q(6)(0)(1)#400 <= 424.8
 0 <= q(6)(0)(2)#401 <= 424.8
 0 <= q(6)(1)(0)#402 <= 424.8
 0 <= q(6)(1)(1)#403 <= 424.8
 0 <= q(6)(1)(2)#404 <= 424.8
 0 <= q(6)(2)(0)#405 <= 424.8
 0 <= q(6)(2)(1)#406 <= 424.8
 0 <= q(6)(2)(2)#407 <= 424.8
      q(6)(3)(0)#408 = 0
      q(6)(3)(1)#409 = 0
      q(6)(3)(2)#410 = 0
      q(6)(4)(0)#411 = 0
      q(6)(4)(1)#412 = 0
      q(6)(4)(2)#413 = 0
      q(7)(0)(0)#414 = 0
      q(7)(0)(1)#415 = 0
      q(7)(0)(2)#416 = 0
      q(7)(1)(0)#417 = 0
      q(7)(1)(1)#418 = 0
      q(7)(1)(2)#419 = 0
      q(7)(2)(0)#420 = 0
      q(7)(2)(1)#421 = 0
      q(7)(2)(2)#422 = 0
 0 <= q(7)(3)(0)#423 <= 424.8
 0 <= q(7)(3)(1)#424 <= 424.8
 0 <= q(7)(3)(2)#425 <= 424.8
      q(7)(4)(0)#426 = 0
      q(7)(4)(1)#427 = 0
      q(7)(4)(2)#428 = 0
 0 <= q(8)(0)(0)#429 <= 424.8
 0 <= q(8)(0)(1)#430 <= 424.8
 0 <= q(8)(0)(2)#431 <= 424.8
      q(8)(1)(0)#432 = 0
      q(8)(1)(1)#433 = 0
      q(8)(1)(2)#434 = 0
      q(8)(2)(0)#435 = 0
      q(8)(2)(1)#436 = 0
      q(8)(2)(2)#437 = 0
 0 <= q(8)(3)(0)#438 <= 424.8
 0 <= q(8)(3)(1)#439 <= 424.8
 0 <= q(8)(3)(2)#440 <= 424.8
      q(8)(4)(0)#441 = 0
      q(8)(4)(1)#442 = 0
      q(8)(4)(2)#443 = 0
      q(9)(0)(0)#444 = 0
      q(9)(0)(1)#445 = 0
      q(9)(0)(2)#446 = 0
 0 <= q(9)(1)(0)#447 <= 424.8
 0 <= q(9)(1)(1)#448 <= 424.8
 0 <= q(9)(1)(2)#449 <= 424.8
      q(9)(2)(0)#450 = 0
      q(9)(2)(1)#451 = 0
      q(9)(2)(2)#452 = 0
 0 <= q(9)(3)(0)#453 <= 424.8
 0 <= q(9)(3)(1)#454 <= 424.8
 0 <= q(9)(3)(2)#455 <= 424.8
      q(9)(4)(0)#456 = 0
      q(9)(4)(1)#457 = 0
      q(9)(4)(2)#458 = 0
 0 <= q(10)(0)(0)#459 <= 424.8
 0 <= q(10)(0)(1)#460 <= 424.8
 0 <= q(10)(0)(2)#461 <= 424.8
 0 <= q(10)(1)(0)#462 <= 424.8
 0 <= q(10)(1)(1)#463 <= 424.8
 0 <= q(10)(1)(2)#464 <= 424.8
      q(10)(2)(0)#465 = 0
      q(10)(2)(1)#466 = 0
      q(10)(2)(2)#467 = 0
 0 <= q(10)(3)(0)#468 <= 424.8
 0 <= q(10)(3)(1)#469 <= 424.8
 0 <= q(10)(3)(2)#470 <= 424.8
      q(10)(4)(0)#471 = 0
      q(10)(4)(1)#472 = 0
      q(10)(4)(2)#473 = 0
      q(11)(0)(0)#474 = 0
      q(11)(0)(1)#475 = 0
      q(11)(0)(2)#476 = 0
      q(11)(1)(0)#477 = 0
      q(11)(1)(1)#478 = 0
      q(11)(1)(2)#479 = 0
 0 <= q(11)(2)(0)#480 <= 424.8
 0 <= q(11)(2)(1)#481 <= 424.8
 0 <= q(11)(2)(2)#482 <= 424.8
 0 <= q(11)(3)(0)#483 <= 424.8
 0 <= q(11)(3)(1)#484 <= 424.8
 0 <= q(11)(3)(2)#485 <= 424.8
      q(11)(4)(0)#486 = 0
      q(11)(4)(1)#487 = 0
      q(11)(4)(2)#488 = 0
 0 <= q(12)(0)(0)#489 <= 424.8
 0 <= q(12)(0)(1)#490 <= 424.8
 0 <= q(12)(0)(2)#491 <= 424.8
      q(12)(1)(0)#492 = 0
      q(12)(1)(1)#493 = 0
      q(12)(1)(2)#494 = 0
 0 <= q(12)(2)(0)#495 <= 424.8
 0 <= q(12)(2)(1)#496 <= 424.8
 0 <= q(12)(2)(2)#497 <= 424.8
 0 <= q(12)(3)(0)#498 <= 424.8
 0 <= q(12)(3)(1)#499 <= 424.8
 0 <= q(12)(3)(2)#500 <= 424.8
      q(12)(4)(0)#501 = 0
      q(12)(4)(1)#502 = 0
      q(12)(4)(2)#503 = 0
      q(13)(0)(0)#504 = 0
      q(13)(0)(1)#505 = 0
      q(13)(0)(2)#506 = 0
 0 <= q(13)(1)(0)#507 <= 424.8
 0 <= q(13)(1)(1)#508 <= 424.8
 0 <= q(13)(1)(2)#509 <= 424.8
 0 <= q(13)(2)(0)#510 <= 424.8
 0 <= q(13)(2)(1)#511 <= 424.8
 0 <= q(13)(2)(2)#512 <= 424.8
 0 <= q(13)(3)(0)#513 <= 424.8
 0 <= q(13)(3)(1)#514 <= 424.8
 0 <= q(13)(3)(2)#515 <= 424.8
      q(13)(4)(0)#516 = 0
      q(13)(4)(1)#517 = 0
      q(13)(4)(2)#518 = 0
 0 <= q(14)(0)(0)#519 <= 424.8
 0 <= q(14)(0)(1)#520 <= 424.8
 0 <= q(14)(0)(2)#521 <= 424.8
 0 <= q(14)(1)(0)#522 <= 424.8
 0 <= q(14)(1)(1)#523 <= 424.8
 0 <= q(14)(1)(2)#524 <= 424.8
 0 <= q(14)(2)(0)#525 <= 424.8
 0 <= q(14)(2)(1)#526 <= 424.8
 0 <= q(14)(2)(2)#527 <= 424.8
 0 <= q(14)(3)(0)#528 <= 424.8
 0 <= q(14)(3)(1)#529 <= 424.8
 0 <= q(14)(3)(2)#530 <= 424.8
      q(14)(4)(0)#531 = 0
      q(14)(4)(1)#532 = 0
      q(14)(4)(2)#533 = 0
      q(15)(0)(0)#534 = 0
      q(15)(0)(1)#535 = 0
      q(15)(0)(2)#536 = 0
      q(15)(1)(0)#537 = 0
      q(15)(1)(1)#538 = 0
      q(15)(1)(2)#539 = 0
      q(15)(2)(0)#540 = 0
      q(15)(2)(1)#541 = 0
      q(15)(2)(2)#542 = 0
      q(15)(3)(0)#543 = 0
      q(15)(3)(1)#544 = 0
      q(15)(3)(2)#545 = 0
 0 <= q(15)(4)(0)#546 <= 424.8
 0 <= q(15)(4)(1)#547 <= 424.8
 0 <= q(15)(4)(2)#548 <= 424.8
 0 <= q(16)(0)(0)#549 <= 424.8
 0 <= q(16)(0)(1)#550 <= 424.8
 0 <= q(16)(0)(2)#551 <= 424.8
      q(16)(1)(0)#552 = 0
      q(16)(1)(1)#553 = 0
      q(16)(1)(2)#554 = 0
      q(16)(2)(0)#555 = 0
      q(16)(2)(1)#556 = 0
      q(16)(2)(2)#557 = 0
      q(16)(3)(0)#558 = 0
      q(16)(3)(1)#559 = 0
      q(16)(3)(2)#560 = 0
 0 <= q(16)(4)(0)#561 <= 424.8
 0 <= q(16)(4)(1)#562 <= 424.8
 0 <= q(16)(4)(2)#563 <= 424.8
      q(17)(0)(0)#564 = 0
      q(17)(0)(1)#565 = 0
      q(17)(0)(2)#566 = 0
 0 <= q(17)(1)(0)#567 <= 424.8
 0 <= q(17)(1)(1)#568 <= 424.8
 0 <= q(17)(1)(2)#569 <= 424.8
      q(17)(2)(0)#570 = 0
      q(17)(2)(1)#571 = 0
      q(17)(2)(2)#572 = 0
      q(17)(3)(0)#573 = 0
      q(17)(3)(1)#574 = 0
      q(17)(3)(2)#575 = 0
 0 <= q(17)(4)(0)#576 <= 424.8
 0 <= q(17)(4)(1)#577 <= 424.8
 0 <= q(17)(4)(2)#578 <= 424.8
 0 <= q(18)(0)(0)#579 <= 424.8
 0 <= q(18)(0)(1)#580 <= 424.8
 0 <= q(18)(0)(2)#581 <= 424.8
 0 <= q(18)(1)(0)#582 <= 424.8
 0 <= q(18)(1)(1)#583 <= 424.8
 0 <= q(18)(1)(2)#584 <= 424.8
      q(18)(2)(0)#585 = 0
      q(18)(2)(1)#586 = 0
      q(18)(2)(2)#587 = 0
      q(18)(3)(0)#588 = 0
      q(18)(3)(1)#589 = 0
      q(18)(3)(2)#590 = 0
 0 <= q(18)(4)(0)#591 <= 424.8
 0 <= q(18)(4)(1)#592 <= 424.8
 0 <= q(18)(4)(2)#593 <= 424.8
      q(19)(0)(0)#594 = 0
      q(19)(0)(1)#595 = 0
      q(19)(0)(2)#596 = 0
      q(19)(1)(0)#597 = 0
      q(19)(1)(1)#598 = 0
      q(19)(1)(2)#599 = 0
 0 <= q(19)(2)(0)#600 <= 424.8
 0 <= q(19)(2)(1)#601 <= 424.8
 0 <= q(19)(2)(2)#602 <= 424.8
      q(19)(3)(0)#603 = 0
      q(19)(3)(1)#604 = 0
      q(19)(3)(2)#605 = 0
 0 <= q(19)(4)(0)#606 <= 424.8
 0 <= q(19)(4)(1)#607 <= 424.8
 0 <= q(19)(4)(2)#608 <= 424.8
 0 <= q(20)(0)(0)#609 <= 424.8
 0 <= q(20)(0)(1)#610 <= 424.8
 0 <= q(20)(0)(2)#611 <= 424.8
      q(20)(1)(0)#612 = 0
      q(20)(1)(1)#613 = 0
      q(20)(1)(2)#614 = 0
 0 <= q(20)(2)(0)#615 <= 424.8
 0 <= q(20)(2)(1)#616 <= 424.8
 0 <= q(20)(2)(2)#617 <= 424.8
      q(20)(3)(0)#618 = 0
      q(20)(3)(1)#619 = 0
      q(20)(3)(2)#620 = 0
 0 <= q(20)(4)(0)#621 <= 424.8
 0 <= q(20)(4)(1)#622 <= 424.8
 0 <= q(20)(4)(2)#623 <= 424.8
      q(21)(0)(0)#624 = 0
      q(21)(0)(1)#625 = 0
      q(21)(0)(2)#626 = 0
 0 <= q(21)(1)(0)#627 <= 424.8
 0 <= q(21)(1)(1)#628 <= 424.8
 0 <= q(21)(1)(2)#629 <= 424.8
 0 <= q(21)(2)(0)#630 <= 424.8
 0 <= q(21)(2)(1)#631 <= 424.8
 0 <= q(21)(2)(2)#632 <= 424.8
      q(21)(3)(0)#633 = 0
      q(21)(3)(1)#634 = 0
      q(21)(3)(2)#635 = 0
 0 <= q(21)(4)(0)#636 <= 424.8
 0 <= q(21)(4)(1)#637 <= 424.8
 0 <= q(21)(4)(2)#638 <= 424.8
 0 <= q(22)(0)(0)#639 <= 424.8
 0 <= q(22)(0)(1)#640 <= 424.8
 0 <= q(22)(0)(2)#641 <= 424.8
 0 <= q(22)(1)(0)#642 <= 424.8
 0 <= q(22)(1)(1)#643 <= 424.8
 0 <= q(22)(1)(2)#644 <= 424.8
 0 <= q(22)(2)(0)#645 <= 424.8
 0 <= q(22)(2)(1)#646 <= 424.8
 0 <= q(22)(2)(2)#647 <= 424.8
      q(22)(3)(0)#648 = 0
      q(22)(3)(1)#649 = 0
      q(22)(3)(2)#650 = 0
 0 <= q(22)(4)(0)#651 <= 424.8
 0 <= q(22)(4)(1)#652 <= 424.8
 0 <= q(22)(4)(2)#653 <= 424.8
      q(23)(0)(0)#654 = 0
      q(23)(0)(1)#655 = 0
      q(23)(0)(2)#656 = 0
      q(23)(1)(0)#657 = 0
      q(23)(1)(1)#658 = 0
      q(23)(1)(2)#659 = 0
      q(23)(2)(0)#660 = 0
      q(23)(2)(1)#661 = 0
      q(23)(2)(2)#662 = 0
 0 <= q(23)(3)(0)#663 <= 424.8
 0 <= q(23)(3)(1)#664 <= 424.8
 0 <= q(23)(3)(2)#665 <= 424.8
 0 <= q(23)(4)(0)#666 <= 424.8
 0 <= q(23)(4)(1)#667 <= 424.8
 0 <= q(23)(4)(2)#668 <= 424.8
 0 <= q(24)(0)(0)#669 <= 424.8
 0 <= q(24)(0)(1)#670 <= 424.8
 0 <= q(24)(0)(2)#671 <= 424.8
      q(24)(1)(0)#672 = 0
      q(24)(1)(1)#673 = 0
      q(24)(1)(2)#674 = 0
      q(24)(2)(0)#675 = 0
      q(24)(2)(1)#676 = 0
      q(24)(2)(2)#677 = 0
 0 <= q(24)(3)(0)#678 <= 424.8
 0 <= q(24)(3)(1)#679 <= 424.8
 0 <= q(24)(3)(2)#680 <= 424.8
 0 <= q(24)(4)(0)#681 <= 424.8
 0 <= q(24)(4)(1)#682 <= 424.8
 0 <= q(24)(4)(2)#683 <= 424.8
      q(25)(0)(0)#684 = 0
      q(25)(0)(1)#685 = 0
      q(25)(0)(2)#686 = 0
 0 <= q(25)(1)(0)#687 <= 424.8
 0 <= q(25)(1)(1)#688 <= 424.8
 0 <= q(25)(1)(2)#689 <= 424.8
      q(25)(2)(0)#690 = 0
      q(25)(2)(1)#691 = 0
      q(25)(2)(2)#692 = 0
 0 <= q(25)(3)(0)#693 <= 424.8
 0 <= q(25)(3)(1)#694 <= 424.8
 0 <= q(25)(3)(2)#695 <= 424.8
 0 <= q(25)(4)(0)#696 <= 424.8
 0 <= q(25)(4)(1)#697 <= 424.8
 0 <= q(25)(4)(2)#698 <= 424.8
 0 <= q(26)(0)(0)#699 <= 424.8
 0 <= q(26)(0)(1)#700 <= 424.8
 0 <= q(26)(0)(2)#701 <= 424.8
 0 <= q(26)(1)(0)#702 <= 424.8
 0 <= q(26)(1)(1)#703 <= 424.8
 0 <= q(26)(1)(2)#704 <= 424.8
      q(26)(2)(0)#705 = 0
      q(26)(2)(1)#706 = 0
      q(26)(2)(2)#707 = 0
 0 <= q(26)(3)(0)#708 <= 424.8
 0 <= q(26)(3)(1)#709 <= 424.8
 0 <= q(26)(3)(2)#710 <= 424.8
 0 <= q(26)(4)(0)#711 <= 424.8
 0 <= q(26)(4)(1)#712 <= 424.8
 0 <= q(26)(4)(2)#713 <= 424.8
      q(27)(0)(0)#714 = 0
      q(27)(0)(1)#715 = 0
      q(27)(0)(2)#716 = 0
      q(27)(1)(0)#717 = 0
      q(27)(1)(1)#718 = 0
      q(27)(1)(2)#719 = 0
 0 <= q(27)(2)(0)#720 <= 424.8
 0 <= q(27)(2)(1)#721 <= 424.8
 0 <= q(27)(2)(2)#722 <= 424.8
 0 <= q(27)(3)(0)#723 <= 424.8
 0 <= q(27)(3)(1)#724 <= 424.8
 0 <= q(27)(3)(2)#725 <= 424.8
 0 <= q(27)(4)(0)#726 <= 424.8
 0 <= q(27)(4)(1)#727 <= 424.8
 0 <= q(27)(4)(2)#728 <= 424.8
 0 <= q(28)(0)(0)#729 <= 424.8
 0 <= q(28)(0)(1)#730 <= 424.8
 0 <= q(28)(0)(2)#731 <= 424.8
      q(28)(1)(0)#732 = 0
      q(28)(1)(1)#733 = 0
      q(28)(1)(2)#734 = 0
 0 <= q(28)(2)(0)#735 <= 424.8
 0 <= q(28)(2)(1)#736 <= 424.8
 0 <= q(28)(2)(2)#737 <= 424.8
 0 <= q(28)(3)(0)#738 <= 424.8
 0 <= q(28)(3)(1)#739 <= 424.8
 0 <= q(28)(3)(2)#740 <= 424.8
 0 <= q(28)(4)(0)#741 <= 424.8
 0 <= q(28)(4)(1)#742 <= 424.8
 0 <= q(28)(4)(2)#743 <= 424.8
      q(29)(0)(0)#744 = 0
      q(29)(0)(1)#745 = 0
      q(29)(0)(2)#746 = 0
 0 <= q(29)(1)(0)#747 <= 424.8
 0 <= q(29)(1)(1)#748 <= 424.8
 0 <= q(29)(1)(2)#749 <= 424.8
 0 <= q(29)(2)(0)#750 <= 424.8
 0 <= q(29)(2)(1)#751 <= 424.8
 0 <= q(29)(2)(2)#752 <= 424.8
 0 <= q(29)(3)(0)#753 <= 424.8
 0 <= q(29)(3)(1)#754 <= 424.8
 0 <= q(29)(3)(2)#755 <= 424.8
 0 <= q(29)(4)(0)#756 <= 424.8
 0 <= q(29)(4)(1)#757 <= 424.8
 0 <= q(29)(4)(2)#758 <= 424.8
 0 <= q(30)(0)(0)#759 <= 424.8
 0 <= q(30)(0)(1)#760 <= 424.8
 0 <= q(30)(0)(2)#761 <= 424.8
 0 <= q(30)(1)(0)#762 <= 424.8
 0 <= q(30)(1)(1)#763 <= 424.8
 0 <= q(30)(1)(2)#764 <= 424.8
 0 <= q(30)(2)(0)#765 <= 424.8
 0 <= q(30)(2)(1)#766 <= 424.8
 0 <= q(30)(2)(2)#767 <= 424.8
 0 <= q(30)(3)(0)#768 <= 424.8
 0 <= q(30)(3)(1)#769 <= 424.8
 0 <= q(30)(3)(2)#770 <= 424.8
 0 <= q(30)(4)(0)#771 <= 424.8
 0 <= q(30)(4)(1)#772 <= 424.8
 0 <= q(30)(4)(2)#773 <= 424.8
 0 <= o(0)(0)#774 <= 1
 0 <= o(0)(1)#775 <= 1
 0 <= o(0)(2)#776 <= 1
 0 <= o(1)(0)#777 <= 1
 0 <= o(1)(1)#778 <= 1
 0 <= o(1)(2)#779 <= 1
 0 <= o(2)(0)#780 <= 1
 0 <= o(2)(1)#781 <= 1
 0 <= o(2)(2)#782 <= 1
 0 <= o(3)(0)#783 <= 1
 0 <= o(3)(1)#784 <= 1
 0 <= o(3)(2)#785 <= 1
 0 <= o(4)(0)#786 <= 1
 0 <= o(4)(1)#787 <= 1
 0 <= o(4)(2)#788 <= 1
 0 <= o(5)(0)#789 <= 1
 0 <= o(5)(1)#790 <= 1
 0 <= o(5)(2)#791 <= 1
 0 <= o(6)(0)#792 <= 1
 0 <= o(6)(1)#793 <= 1
 0 <= o(6)(2)#794 <= 1
 0 <= o(7)(0)#795 <= 1
 0 <= o(7)(1)#796 <= 1
 0 <= o(7)(2)#797 <= 1
 0 <= o(8)(0)#798 <= 1
 0 <= o(8)(1)#799 <= 1
 0 <= o(8)(2)#800 <= 1
 0 <= o(9)(0)#801 <= 1
 0 <= o(9)(1)#802 <= 1
 0 <= o(9)(2)#803 <= 1
 0 <= o(10)(0)#804 <= 1
 0 <= o(10)(1)#805 <= 1
 0 <= o(10)(2)#806 <= 1
 0 <= o(11)(0)#807 <= 1
 0 <= o(11)(1)#808 <= 1
 0 <= o(11)(2)#809 <= 1
 0 <= o(12)(0)#810 <= 1
 0 <= o(12)(1)#811 <= 1
 0 <= o(12)(2)#812 <= 1
 0 <= o(13)(0)#813 <= 1
 0 <= o(13)(1)#814 <= 1
 0 <= o(13)(2)#815 <= 1
 0 <= o(14)(0)#816 <= 1
 0 <= o(14)(1)#817 <= 1
 0 <= o(14)(2)#818 <= 1
 0 <= o(15)(0)#819 <= 1
 0 <= o(15)(1)#820 <= 1
 0 <= o(15)(2)#821 <= 1
 0 <= o(16)(0)#822 <= 1
 0 <= o(16)(1)#823 <= 1
 0 <= o(16)(2)#824 <= 1
 0 <= o(17)(0)#825 <= 1
 0 <= o(17)(1)#826 <= 1
 0 <= o(17)(2)#827 <= 1
 0 <= o(18)(0)#828 <= 1
 0 <= o(18)(1)#829 <= 1
 0 <= o(18)(2)#830 <= 1
 0 <= o(19)(0)#831 <= 1
 0 <= o(19)(1)#832 <= 1
 0 <= o(19)(2)#833 <= 1
 0 <= o(20)(0)#834 <= 1
 0 <= o(20)(1)#835 <= 1
 0 <= o(20)(2)#836 <= 1
 0 <= o(21)(0)#837 <= 1
 0 <= o(21)(1)#838 <= 1
 0 <= o(21)(2)#839 <= 1
 0 <= o(22)(0)#840 <= 1
 0 <= o(22)(1)#841 <= 1
 0 <= o(22)(2)#842 <= 1
 0 <= o(23)(0)#843 <= 1
 0 <= o(23)(1)#844 <= 1
 0 <= o(23)(2)#845 <= 1
 0 <= o(24)(0)#846 <= 1
 0 <= o(24)(1)#847 <= 1
 0 <= o(24)(2)#848 <= 1
 0 <= o(25)(0)#849 <= 1
 0 <= o(25)(1)#850 <= 1
 0 <= o(25)(2)#851 <= 1
 0 <= o(26)(0)#852 <= 1
 0 <= o(26)(1)#853 <= 1
 0 <= o(26)(2)#854 <= 1
 0 <= o(27)(0)#855 <= 1
 0 <= o(27)(1)#856 <= 1
 0 <= o(27)(2)#857 <= 1
 0 <= o(28)(0)#858 <= 1
 0 <= o(28)(1)#859 <= 1
 0 <= o(28)(2)#860 <= 1
 0 <= o(29)(0)#861 <= 1
 0 <= o(29)(1)#862 <= 1
 0 <= o(29)(2)#863 <= 1
 0 <= o(30)(0)#864 <= 1
 0 <= o(30)(1)#865 <= 1
 0 <= o(30)(2)#866 <= 1
Binaries
 y(0)#0  y(1)#1  y(2)#2  o(0)(0)#774  o(0)(1)#775  o(0)(2)#776  o(1)(0)#777 
 o(1)(1)#778  o(1)(2)#779  o(2)(0)#780  o(2)(1)#781  o(2)(2)#782  o(3)(0)#783 
 o(3)(1)#784  o(3)(2)#785  o(4)(0)#786  o(4)(1)#787  o(4)(2)#788  o(5)(0)#789 
 o(5)(1)#790  o(5)(2)#791  o(6)(0)#792  o(6)(1)#793  o(6)(2)#794  o(7)(0)#795 
 o(7)(1)#796  o(7)(2)#797  o(8)(0)#798  o(8)(1)#799  o(8)(2)#800  o(9)(0)#801 
 o(9)(1)#802  o(9)(2)#803  o(10)(0)#804  o(10)(1)#805  o(10)(2)#806 
 o(11)(0)#807  o(11)(1)#808  o(11)(2)#809  o(12)(0)#810  o(12)(1)#811 
 o(12)(2)#812  o(13)(0)#813  o(13)(1)#814  o(13)(2)#815  o(14)(0)#816 
 o(14)(1)#817  o(14)(2)#818  o(15)(0)#819  o(15)(1)#820  o(15)(2)#821 
 o(16)(0)#822  o(16)(1)#823  o(16)(2)#824  o(17)(0)#825  o(17)(1)#826 
 o(17)(2)#827  o(18)(0)#828  o(18)(1)#829  o(18)(2)#830  o(19)(0)#831 
 o(19)(1)#832  o(19)(2)#833  o(20)(0)#834  o(20)(1)#835  o(20)(2)#836 
 o(21)(0)#837  o(21)(1)#838  o(21)(2)#839  o(22)(0)#840  o(22)(1)#841 
 o(22)(2)#842  o(23)(0)#843  o(23)(1)#844  o(23)(2)#845  o(24)(0)#846 
 o(24)(1)#847  o(24)(2)#848  o(25)(0)#849  o(25)(1)#850  o(25)(2)#851 
 o(26)(0)#852  o(26)(1)#853  o(26)(2)#854  o(27)(0)#855  o(27)(1)#856 
 o(27)(2)#857  o(28)(0)#858  o(28)(1)#859  o(28)(2)#860  o(29)(0)#861 
 o(29)(1)#862  o(29)(2)#863  o(30)(0)#864  o(30)(1)#865  o(30)(2)#866 
End
