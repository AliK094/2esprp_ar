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
       + 324.444444444444 b_warehouse(0)(0)(0)#129
       + 324.444444444444 b_warehouse(0)(0)(1)#130
       + 324.444444444444 b_warehouse(0)(0)(2)#131
       + 324.444444444444 b_warehouse(0)(0)(3)#132
       + 324.444444444444 b_warehouse(0)(0)(4)#133
       + 324.444444444444 b_warehouse(0)(0)(5)#134
       + 324.444444444444 b_warehouse(0)(0)(6)#135
       + 324.444444444444 b_warehouse(0)(0)(7)#136
       + 324.444444444444 b_warehouse(0)(0)(8)#137
       + 324.444444444444 b_warehouse(0)(0)(9)#138
       + 324.444444444444 b_warehouse(0)(1)(0)#139
       + 324.444444444444 b_warehouse(0)(1)(1)#140
       + 324.444444444444 b_warehouse(0)(1)(2)#141
       + 324.444444444444 b_warehouse(0)(1)(3)#142
       + 324.444444444444 b_warehouse(0)(1)(4)#143
       + 324.444444444444 b_warehouse(0)(1)(5)#144
       + 324.444444444444 b_warehouse(0)(1)(6)#145
       + 324.444444444444 b_warehouse(0)(1)(7)#146
       + 324.444444444444 b_warehouse(0)(1)(8)#147
       + 324.444444444444 b_warehouse(0)(1)(9)#148
       + 324.444444444444 b_warehouse(0)(2)(0)#149
       + 324.444444444444 b_warehouse(0)(2)(1)#150
       + 324.444444444444 b_warehouse(0)(2)(2)#151
       + 324.444444444444 b_warehouse(0)(2)(3)#152
       + 324.444444444444 b_warehouse(0)(2)(4)#153
       + 324.444444444444 b_warehouse(0)(2)(5)#154
       + 324.444444444444 b_warehouse(0)(2)(6)#155
       + 324.444444444444 b_warehouse(0)(2)(7)#156
       + 324.444444444444 b_warehouse(0)(2)(8)#157
       + 324.444444444444 b_warehouse(0)(2)(9)#158
       + 165 b_warehouse(1)(0)(0)#159 + 165 b_warehouse(1)(0)(1)#160
       + 165 b_warehouse(1)(0)(2)#161 + 165 b_warehouse(1)(0)(3)#162
       + 165 b_warehouse(1)(0)(4)#163 + 165 b_warehouse(1)(0)(5)#164
       + 165 b_warehouse(1)(0)(6)#165 + 165 b_warehouse(1)(0)(7)#166
       + 165 b_warehouse(1)(0)(8)#167 + 165 b_warehouse(1)(0)(9)#168
       + 165 b_warehouse(1)(1)(0)#169 + 165 b_warehouse(1)(1)(1)#170
       + 165 b_warehouse(1)(1)(2)#171 + 165 b_warehouse(1)(1)(3)#172
       + 165 b_warehouse(1)(1)(4)#173 + 165 b_warehouse(1)(1)(5)#174
       + 165 b_warehouse(1)(1)(6)#175 + 165 b_warehouse(1)(1)(7)#176
       + 165 b_warehouse(1)(1)(8)#177 + 165 b_warehouse(1)(1)(9)#178
       + 165 b_warehouse(1)(2)(0)#179 + 165 b_warehouse(1)(2)(1)#180
       + 165 b_warehouse(1)(2)(2)#181 + 165 b_warehouse(1)(2)(3)#182
       + 165 b_warehouse(1)(2)(4)#183 + 165 b_warehouse(1)(2)(5)#184
       + 165 b_warehouse(1)(2)(6)#185 + 165 b_warehouse(1)(2)(7)#186
       + 165 b_warehouse(1)(2)(8)#187 + 165 b_warehouse(1)(2)(9)#188
       + 351 b_warehouse(2)(0)(0)#189 + 351 b_warehouse(2)(0)(1)#190
       + 351 b_warehouse(2)(0)(2)#191 + 351 b_warehouse(2)(0)(3)#192
       + 351 b_warehouse(2)(0)(4)#193 + 351 b_warehouse(2)(0)(5)#194
       + 351 b_warehouse(2)(0)(6)#195 + 351 b_warehouse(2)(0)(7)#196
       + 351 b_warehouse(2)(0)(8)#197 + 351 b_warehouse(2)(0)(9)#198
       + 351 b_warehouse(2)(1)(0)#199 + 351 b_warehouse(2)(1)(1)#200
       + 351 b_warehouse(2)(1)(2)#201 + 351 b_warehouse(2)(1)(3)#202
       + 351 b_warehouse(2)(1)(4)#203 + 351 b_warehouse(2)(1)(5)#204
       + 351 b_warehouse(2)(1)(6)#205 + 351 b_warehouse(2)(1)(7)#206
       + 351 b_warehouse(2)(1)(8)#207 + 351 b_warehouse(2)(1)(9)#208
       + 351 b_warehouse(2)(2)(0)#209 + 351 b_warehouse(2)(2)(1)#210
       + 351 b_warehouse(2)(2)(2)#211 + 351 b_warehouse(2)(2)(3)#212
       + 351 b_warehouse(2)(2)(4)#213 + 351 b_warehouse(2)(2)(5)#214
       + 351 b_warehouse(2)(2)(6)#215 + 351 b_warehouse(2)(2)(7)#216
       + 351 b_warehouse(2)(2)(8)#217 + 351 b_warehouse(2)(2)(9)#218
       + 420 b_warehouse(3)(0)(0)#219 + 420 b_warehouse(3)(0)(1)#220
       + 420 b_warehouse(3)(0)(2)#221 + 420 b_warehouse(3)(0)(3)#222
       + 420 b_warehouse(3)(0)(4)#223 + 420 b_warehouse(3)(0)(5)#224
       + 420 b_warehouse(3)(0)(6)#225 + 420 b_warehouse(3)(0)(7)#226
       + 420 b_warehouse(3)(0)(8)#227 + 420 b_warehouse(3)(0)(9)#228
       + 420 b_warehouse(3)(1)(0)#229 + 420 b_warehouse(3)(1)(1)#230
       + 420 b_warehouse(3)(1)(2)#231 + 420 b_warehouse(3)(1)(3)#232
       + 420 b_warehouse(3)(1)(4)#233 + 420 b_warehouse(3)(1)(5)#234
       + 420 b_warehouse(3)(1)(6)#235 + 420 b_warehouse(3)(1)(7)#236
       + 420 b_warehouse(3)(1)(8)#237 + 420 b_warehouse(3)(1)(9)#238
       + 420 b_warehouse(3)(2)(0)#239 + 420 b_warehouse(3)(2)(1)#240
       + 420 b_warehouse(3)(2)(2)#241 + 420 b_warehouse(3)(2)(3)#242
       + 420 b_warehouse(3)(2)(4)#243 + 420 b_warehouse(3)(2)(5)#244
       + 420 b_warehouse(3)(2)(6)#245 + 420 b_warehouse(3)(2)(7)#246
       + 420 b_warehouse(3)(2)(8)#247 + 420 b_warehouse(3)(2)(9)#248
       + 2838.94205647104 o(0)(0)#429 + 2838.94205647104 o(0)(1)#430
       + 2838.94205647104 o(0)(2)#431 + 1306.88790644033 o(1)(0)#432
       + 1306.88790644033 o(1)(1)#433 + 1306.88790644033 o(1)(2)#434
       + 3393.02444661378 o(2)(0)#435 + 3393.02444661378 o(2)(1)#436
       + 3393.02444661378 o(2)(2)#437 + 3753.07553880814 o(3)(0)#438
       + 3753.07553880814 o(3)(1)#439 + 3753.07553880814 o(3)(2)#440
       + 4049.31086188739 o(4)(0)#441 + 4049.31086188739 o(4)(1)#442
       + 4049.31086188739 o(4)(2)#443 + 4506.22183961407 o(5)(0)#444
       + 4506.22183961407 o(5)(1)#445 + 4506.22183961407 o(5)(2)#446
       + 4603.39325203014 o(6)(0)#447 + 4603.39325203014 o(6)(1)#448
       + 4603.39325203014 o(6)(2)#449 + 1739.25271309261 o(7)(0)#450
       + 1739.25271309261 o(7)(1)#451 + 1739.25271309261 o(7)(2)#452
       + 4037.75629949332 o(8)(0)#453 + 4037.75629949332 o(8)(1)#454
       + 4037.75629949332 o(8)(2)#455 + 1955.97792115717 o(9)(0)#456
       + 1955.97792115717 o(9)(1)#457 + 1955.97792115717 o(9)(2)#458
       + 4042.11446133062 o(10)(0)#459 + 4042.11446133062 o(10)(1)#460
       + 4042.11446133062 o(10)(2)#461 + 5154.53785110185 o(11)(0)#462
       + 5154.53785110185 o(11)(1)#463 + 5154.53785110185 o(11)(2)#464
       + 5248.12510490968 o(12)(0)#465 + 5248.12510490968 o(12)(1)#466
       + 5248.12510490968 o(12)(2)#467 + 5155.31185433091 o(13)(0)#468
       + 5155.31185433091 o(13)(1)#469 + 5155.31185433091 o(13)(2)#470
       + 5252.48326674697 o(14)(0)#471 + 5252.48326674697 o(14)(1)#472
       + 5252.48326674697 o(14)(2)#473
Subject To
 ProductionCapacity(1):          - 622 y(0)#0 + p(0)#3 <= 0
 ProductionCapacity(2):          - 622 y(1)#1 + p(1)#4 <= 0
 ProductionCapacity(3):          - 622 y(2)#2 + p(2)#5 <= 0
 PlantInventoryCapacity(1):      I_plant(0)#6 <= 311
 PlantInventoryCapacity(2):      I_plant(1)#7 <= 311
 PlantInventoryCapacity(3):      I_plant(2)#8 <= 311
 PlantInventoryBalance(1):       - p(0)#3 + I_plant(0)#6 + q(0)(0)(0)#249
                                 + q(0)(1)(0)#252 + q(0)(2)(0)#255
                                 + q(0)(3)(0)#258 + q(1)(0)(0)#261
                                 + q(1)(1)(0)#264 + q(1)(2)(0)#267
                                 + q(1)(3)(0)#270 + q(2)(0)(0)#273
                                 + q(2)(1)(0)#276 + q(2)(2)(0)#279
                                 + q(2)(3)(0)#282 + q(3)(0)(0)#285
                                 + q(3)(1)(0)#288 + q(3)(2)(0)#291
                                 + q(3)(3)(0)#294 + q(4)(0)(0)#297
                                 + q(4)(1)(0)#300 + q(4)(2)(0)#303
                                 + q(4)(3)(0)#306 + q(5)(0)(0)#309
                                 + q(5)(1)(0)#312 + q(5)(2)(0)#315
                                 + q(5)(3)(0)#318 + q(6)(0)(0)#321
                                 + q(6)(1)(0)#324 + q(6)(2)(0)#327
                                 + q(6)(3)(0)#330 + q(7)(0)(0)#333
                                 + q(7)(1)(0)#336 + q(7)(2)(0)#339
                                 + q(7)(3)(0)#342 + q(8)(0)(0)#345
                                 + q(8)(1)(0)#348 + q(8)(2)(0)#351
                                 + q(8)(3)(0)#354 + q(9)(0)(0)#357
                                 + q(9)(1)(0)#360 + q(9)(2)(0)#363
                                 + q(9)(3)(0)#366 + q(10)(0)(0)#369
                                 + q(10)(1)(0)#372 + q(10)(2)(0)#375
                                 + q(10)(3)(0)#378 + q(11)(0)(0)#381
                                 + q(11)(1)(0)#384 + q(11)(2)(0)#387
                                 + q(11)(3)(0)#390 + q(12)(0)(0)#393
                                 + q(12)(1)(0)#396 + q(12)(2)(0)#399
                                 + q(12)(3)(0)#402 + q(13)(0)(0)#405
                                 + q(13)(1)(0)#408 + q(13)(2)(0)#411
                                 + q(13)(3)(0)#414 + q(14)(0)(0)#417
                                 + q(14)(1)(0)#420 + q(14)(2)(0)#423
                                 + q(14)(3)(0)#426  = 0
 PlantInventoryBalance(2):       - p(1)#4 - I_plant(0)#6 + I_plant(1)#7
                                 + q(0)(0)(1)#250 + q(0)(1)(1)#253
                                 + q(0)(2)(1)#256 + q(0)(3)(1)#259
                                 + q(1)(0)(1)#262 + q(1)(1)(1)#265
                                 + q(1)(2)(1)#268 + q(1)(3)(1)#271
                                 + q(2)(0)(1)#274 + q(2)(1)(1)#277
                                 + q(2)(2)(1)#280 + q(2)(3)(1)#283
                                 + q(3)(0)(1)#286 + q(3)(1)(1)#289
                                 + q(3)(2)(1)#292 + q(3)(3)(1)#295
                                 + q(4)(0)(1)#298 + q(4)(1)(1)#301
                                 + q(4)(2)(1)#304 + q(4)(3)(1)#307
                                 + q(5)(0)(1)#310 + q(5)(1)(1)#313
                                 + q(5)(2)(1)#316 + q(5)(3)(1)#319
                                 + q(6)(0)(1)#322 + q(6)(1)(1)#325
                                 + q(6)(2)(1)#328 + q(6)(3)(1)#331
                                 + q(7)(0)(1)#334 + q(7)(1)(1)#337
                                 + q(7)(2)(1)#340 + q(7)(3)(1)#343
                                 + q(8)(0)(1)#346 + q(8)(1)(1)#349
                                 + q(8)(2)(1)#352 + q(8)(3)(1)#355
                                 + q(9)(0)(1)#358 + q(9)(1)(1)#361
                                 + q(9)(2)(1)#364 + q(9)(3)(1)#367
                                 + q(10)(0)(1)#370 + q(10)(1)(1)#373
                                 + q(10)(2)(1)#376 + q(10)(3)(1)#379
                                 + q(11)(0)(1)#382 + q(11)(1)(1)#385
                                 + q(11)(2)(1)#388 + q(11)(3)(1)#391
                                 + q(12)(0)(1)#394 + q(12)(1)(1)#397
                                 + q(12)(2)(1)#400 + q(12)(3)(1)#403
                                 + q(13)(0)(1)#406 + q(13)(1)(1)#409
                                 + q(13)(2)(1)#412 + q(13)(3)(1)#415
                                 + q(14)(0)(1)#418 + q(14)(1)(1)#421
                                 + q(14)(2)(1)#424 + q(14)(3)(1)#427  = 0
 PlantInventoryBalance(3):       - p(2)#5 - I_plant(1)#7 + I_plant(2)#8
                                 + q(0)(0)(2)#251 + q(0)(1)(2)#254
                                 + q(0)(2)(2)#257 + q(0)(3)(2)#260
                                 + q(1)(0)(2)#263 + q(1)(1)(2)#266
                                 + q(1)(2)(2)#269 + q(1)(3)(2)#272
                                 + q(2)(0)(2)#275 + q(2)(1)(2)#278
                                 + q(2)(2)(2)#281 + q(2)(3)(2)#284
                                 + q(3)(0)(2)#287 + q(3)(1)(2)#290
                                 + q(3)(2)(2)#293 + q(3)(3)(2)#296
                                 + q(4)(0)(2)#299 + q(4)(1)(2)#302
                                 + q(4)(2)(2)#305 + q(4)(3)(2)#308
                                 + q(5)(0)(2)#311 + q(5)(1)(2)#314
                                 + q(5)(2)(2)#317 + q(5)(3)(2)#320
                                 + q(6)(0)(2)#323 + q(6)(1)(2)#326
                                 + q(6)(2)(2)#329 + q(6)(3)(2)#332
                                 + q(7)(0)(2)#335 + q(7)(1)(2)#338
                                 + q(7)(2)(2)#341 + q(7)(3)(2)#344
                                 + q(8)(0)(2)#347 + q(8)(1)(2)#350
                                 + q(8)(2)(2)#353 + q(8)(3)(2)#356
                                 + q(9)(0)(2)#359 + q(9)(1)(2)#362
                                 + q(9)(2)(2)#365 + q(9)(3)(2)#368
                                 + q(10)(0)(2)#371 + q(10)(1)(2)#374
                                 + q(10)(2)(2)#377 + q(10)(3)(2)#380
                                 + q(11)(0)(2)#383 + q(11)(1)(2)#386
                                 + q(11)(2)(2)#389 + q(11)(3)(2)#392
                                 + q(12)(0)(2)#395 + q(12)(1)(2)#398
                                 + q(12)(2)(2)#401 + q(12)(3)(2)#404
                                 + q(13)(0)(2)#407 + q(13)(1)(2)#410
                                 + q(13)(2)(2)#413 + q(13)(3)(2)#416
                                 + q(14)(0)(2)#419 + q(14)(1)(2)#422
                                 + q(14)(2)(2)#425 + q(14)(3)(2)#428  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(0)(0)(0)#9 <= 279.9
 WarehouseInventoryCapacity(2,1,1): I_warehouse(1)(0)(0)#39 <= 279.9
 WarehouseInventoryCapacity(3,1,1): I_warehouse(2)(0)(0)#69 <= 279.9
 WarehouseInventoryCapacity(4,1,1): I_warehouse(3)(0)(0)#99 <= 279.9
 WarehouseInventoryCapacity(1,2,1): I_warehouse(0)(1)(0)#19 <= 279.9
 WarehouseInventoryCapacity(2,2,1): I_warehouse(1)(1)(0)#49 <= 279.9
 WarehouseInventoryCapacity(3,2,1): I_warehouse(2)(1)(0)#79 <= 279.9
 WarehouseInventoryCapacity(4,2,1): I_warehouse(3)(1)(0)#109 <= 279.9
 WarehouseInventoryCapacity(1,3,1): I_warehouse(0)(2)(0)#29 <= 279.9
 WarehouseInventoryCapacity(2,3,1): I_warehouse(1)(2)(0)#59 <= 279.9
 WarehouseInventoryCapacity(3,3,1): I_warehouse(2)(2)(0)#89 <= 279.9
 WarehouseInventoryCapacity(4,3,1): I_warehouse(3)(2)(0)#119 <= 279.9
 WarehouseInventoryCapacity(1,1,2): I_warehouse(0)(0)(1)#10 <= 279.9
 WarehouseInventoryCapacity(2,1,2): I_warehouse(1)(0)(1)#40 <= 279.9
 WarehouseInventoryCapacity(3,1,2): I_warehouse(2)(0)(1)#70 <= 279.9
 WarehouseInventoryCapacity(4,1,2): I_warehouse(3)(0)(1)#100 <= 279.9
 WarehouseInventoryCapacity(1,2,2): I_warehouse(0)(1)(1)#20 <= 279.9
 WarehouseInventoryCapacity(2,2,2): I_warehouse(1)(1)(1)#50 <= 279.9
 WarehouseInventoryCapacity(3,2,2): I_warehouse(2)(1)(1)#80 <= 279.9
 WarehouseInventoryCapacity(4,2,2): I_warehouse(3)(1)(1)#110 <= 279.9
 WarehouseInventoryCapacity(1,3,2): I_warehouse(0)(2)(1)#30 <= 279.9
 WarehouseInventoryCapacity(2,3,2): I_warehouse(1)(2)(1)#60 <= 279.9
 WarehouseInventoryCapacity(3,3,2): I_warehouse(2)(2)(1)#90 <= 279.9
 WarehouseInventoryCapacity(4,3,2): I_warehouse(3)(2)(1)#120 <= 279.9
 WarehouseInventoryCapacity(1,1,3): I_warehouse(0)(0)(2)#11 <= 279.9
 WarehouseInventoryCapacity(2,1,3): I_warehouse(1)(0)(2)#41 <= 279.9
 WarehouseInventoryCapacity(3,1,3): I_warehouse(2)(0)(2)#71 <= 279.9
 WarehouseInventoryCapacity(4,1,3): I_warehouse(3)(0)(2)#101 <= 279.9
 WarehouseInventoryCapacity(1,2,3): I_warehouse(0)(1)(2)#21 <= 279.9
 WarehouseInventoryCapacity(2,2,3): I_warehouse(1)(1)(2)#51 <= 279.9
 WarehouseInventoryCapacity(3,2,3): I_warehouse(2)(1)(2)#81 <= 279.9
 WarehouseInventoryCapacity(4,2,3): I_warehouse(3)(1)(2)#111 <= 279.9
 WarehouseInventoryCapacity(1,3,3): I_warehouse(0)(2)(2)#31 <= 279.9
 WarehouseInventoryCapacity(2,3,3): I_warehouse(1)(2)(2)#61 <= 279.9
 WarehouseInventoryCapacity(3,3,3): I_warehouse(2)(2)(2)#91 <= 279.9
 WarehouseInventoryCapacity(4,3,3): I_warehouse(3)(2)(2)#121 <= 279.9
 WarehouseInventoryCapacity(1,1,4): I_warehouse(0)(0)(3)#12 <= 279.9
 WarehouseInventoryCapacity(2,1,4): I_warehouse(1)(0)(3)#42 <= 279.9
 WarehouseInventoryCapacity(3,1,4): I_warehouse(2)(0)(3)#72 <= 279.9
 WarehouseInventoryCapacity(4,1,4): I_warehouse(3)(0)(3)#102 <= 279.9
 WarehouseInventoryCapacity(1,2,4): I_warehouse(0)(1)(3)#22 <= 279.9
 WarehouseInventoryCapacity(2,2,4): I_warehouse(1)(1)(3)#52 <= 279.9
 WarehouseInventoryCapacity(3,2,4): I_warehouse(2)(1)(3)#82 <= 279.9
 WarehouseInventoryCapacity(4,2,4): I_warehouse(3)(1)(3)#112 <= 279.9
 WarehouseInventoryCapacity(1,3,4): I_warehouse(0)(2)(3)#32 <= 279.9
 WarehouseInventoryCapacity(2,3,4): I_warehouse(1)(2)(3)#62 <= 279.9
 WarehouseInventoryCapacity(3,3,4): I_warehouse(2)(2)(3)#92 <= 279.9
 WarehouseInventoryCapacity(4,3,4): I_warehouse(3)(2)(3)#122 <= 279.9
 WarehouseInventoryCapacity(1,1,5): I_warehouse(0)(0)(4)#13 <= 279.9
 WarehouseInventoryCapacity(2,1,5): I_warehouse(1)(0)(4)#43 <= 279.9
 WarehouseInventoryCapacity(3,1,5): I_warehouse(2)(0)(4)#73 <= 279.9
 WarehouseInventoryCapacity(4,1,5): I_warehouse(3)(0)(4)#103 <= 279.9
 WarehouseInventoryCapacity(1,2,5): I_warehouse(0)(1)(4)#23 <= 279.9
 WarehouseInventoryCapacity(2,2,5): I_warehouse(1)(1)(4)#53 <= 279.9
 WarehouseInventoryCapacity(3,2,5): I_warehouse(2)(1)(4)#83 <= 279.9
 WarehouseInventoryCapacity(4,2,5): I_warehouse(3)(1)(4)#113 <= 279.9
 WarehouseInventoryCapacity(1,3,5): I_warehouse(0)(2)(4)#33 <= 279.9
 WarehouseInventoryCapacity(2,3,5): I_warehouse(1)(2)(4)#63 <= 279.9
 WarehouseInventoryCapacity(3,3,5): I_warehouse(2)(2)(4)#93 <= 279.9
 WarehouseInventoryCapacity(4,3,5): I_warehouse(3)(2)(4)#123 <= 279.9
 WarehouseInventoryCapacity(1,1,6): I_warehouse(0)(0)(5)#14 <= 279.9
 WarehouseInventoryCapacity(2,1,6): I_warehouse(1)(0)(5)#44 <= 279.9
 WarehouseInventoryCapacity(3,1,6): I_warehouse(2)(0)(5)#74 <= 279.9
 WarehouseInventoryCapacity(4,1,6): I_warehouse(3)(0)(5)#104 <= 279.9
 WarehouseInventoryCapacity(1,2,6): I_warehouse(0)(1)(5)#24 <= 279.9
 WarehouseInventoryCapacity(2,2,6): I_warehouse(1)(1)(5)#54 <= 279.9
 WarehouseInventoryCapacity(3,2,6): I_warehouse(2)(1)(5)#84 <= 279.9
 WarehouseInventoryCapacity(4,2,6): I_warehouse(3)(1)(5)#114 <= 279.9
 WarehouseInventoryCapacity(1,3,6): I_warehouse(0)(2)(5)#34 <= 279.9
 WarehouseInventoryCapacity(2,3,6): I_warehouse(1)(2)(5)#64 <= 279.9
 WarehouseInventoryCapacity(3,3,6): I_warehouse(2)(2)(5)#94 <= 279.9
 WarehouseInventoryCapacity(4,3,6): I_warehouse(3)(2)(5)#124 <= 279.9
 WarehouseInventoryCapacity(1,1,7): I_warehouse(0)(0)(6)#15 <= 279.9
 WarehouseInventoryCapacity(2,1,7): I_warehouse(1)(0)(6)#45 <= 279.9
 WarehouseInventoryCapacity(3,1,7): I_warehouse(2)(0)(6)#75 <= 279.9
 WarehouseInventoryCapacity(4,1,7): I_warehouse(3)(0)(6)#105 <= 279.9
 WarehouseInventoryCapacity(1,2,7): I_warehouse(0)(1)(6)#25 <= 279.9
 WarehouseInventoryCapacity(2,2,7): I_warehouse(1)(1)(6)#55 <= 279.9
 WarehouseInventoryCapacity(3,2,7): I_warehouse(2)(1)(6)#85 <= 279.9
 WarehouseInventoryCapacity(4,2,7): I_warehouse(3)(1)(6)#115 <= 279.9
 WarehouseInventoryCapacity(1,3,7): I_warehouse(0)(2)(6)#35 <= 279.9
 WarehouseInventoryCapacity(2,3,7): I_warehouse(1)(2)(6)#65 <= 279.9
 WarehouseInventoryCapacity(3,3,7): I_warehouse(2)(2)(6)#95 <= 279.9
 WarehouseInventoryCapacity(4,3,7): I_warehouse(3)(2)(6)#125 <= 279.9
 WarehouseInventoryCapacity(1,1,8): I_warehouse(0)(0)(7)#16 <= 279.9
 WarehouseInventoryCapacity(2,1,8): I_warehouse(1)(0)(7)#46 <= 279.9
 WarehouseInventoryCapacity(3,1,8): I_warehouse(2)(0)(7)#76 <= 279.9
 WarehouseInventoryCapacity(4,1,8): I_warehouse(3)(0)(7)#106 <= 279.9
 WarehouseInventoryCapacity(1,2,8): I_warehouse(0)(1)(7)#26 <= 279.9
 WarehouseInventoryCapacity(2,2,8): I_warehouse(1)(1)(7)#56 <= 279.9
 WarehouseInventoryCapacity(3,2,8): I_warehouse(2)(1)(7)#86 <= 279.9
 WarehouseInventoryCapacity(4,2,8): I_warehouse(3)(1)(7)#116 <= 279.9
 WarehouseInventoryCapacity(1,3,8): I_warehouse(0)(2)(7)#36 <= 279.9
 WarehouseInventoryCapacity(2,3,8): I_warehouse(1)(2)(7)#66 <= 279.9
 WarehouseInventoryCapacity(3,3,8): I_warehouse(2)(2)(7)#96 <= 279.9
 WarehouseInventoryCapacity(4,3,8): I_warehouse(3)(2)(7)#126 <= 279.9
 WarehouseInventoryCapacity(1,1,9): I_warehouse(0)(0)(8)#17 <= 279.9
 WarehouseInventoryCapacity(2,1,9): I_warehouse(1)(0)(8)#47 <= 279.9
 WarehouseInventoryCapacity(3,1,9): I_warehouse(2)(0)(8)#77 <= 279.9
 WarehouseInventoryCapacity(4,1,9): I_warehouse(3)(0)(8)#107 <= 279.9
 WarehouseInventoryCapacity(1,2,9): I_warehouse(0)(1)(8)#27 <= 279.9
 WarehouseInventoryCapacity(2,2,9): I_warehouse(1)(1)(8)#57 <= 279.9
 WarehouseInventoryCapacity(3,2,9): I_warehouse(2)(1)(8)#87 <= 279.9
 WarehouseInventoryCapacity(4,2,9): I_warehouse(3)(1)(8)#117 <= 279.9
 WarehouseInventoryCapacity(1,3,9): I_warehouse(0)(2)(8)#37 <= 279.9
 WarehouseInventoryCapacity(2,3,9): I_warehouse(1)(2)(8)#67 <= 279.9
 WarehouseInventoryCapacity(3,3,9): I_warehouse(2)(2)(8)#97 <= 279.9
 WarehouseInventoryCapacity(4,3,9): I_warehouse(3)(2)(8)#127 <= 279.9
 WarehouseInventoryCapacity(1,1,10): I_warehouse(0)(0)(9)#18 <= 279.9
 WarehouseInventoryCapacity(2,1,10): I_warehouse(1)(0)(9)#48 <= 279.9
 WarehouseInventoryCapacity(3,1,10): I_warehouse(2)(0)(9)#78 <= 279.9
 WarehouseInventoryCapacity(4,1,10): I_warehouse(3)(0)(9)#108 <= 279.9
 WarehouseInventoryCapacity(1,2,10): I_warehouse(0)(1)(9)#28 <= 279.9
 WarehouseInventoryCapacity(2,2,10): I_warehouse(1)(1)(9)#58 <= 279.9
 WarehouseInventoryCapacity(3,2,10): I_warehouse(2)(1)(9)#88 <= 279.9
 WarehouseInventoryCapacity(4,2,10): I_warehouse(3)(1)(9)#118 <= 279.9
 WarehouseInventoryCapacity(1,3,10): I_warehouse(0)(2)(9)#38 <= 279.9
 WarehouseInventoryCapacity(2,3,10): I_warehouse(1)(2)(9)#68 <= 279.9
 WarehouseInventoryCapacity(3,3,10): I_warehouse(2)(2)(9)#98 <= 279.9
 WarehouseInventoryCapacity(4,3,10): I_warehouse(3)(2)(9)#128 <= 279.9
 WarehouseInventoryBalance(1,1,1): I_warehouse(0)(0)(0)#9
                                 - b_warehouse(0)(0)(0)#129 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -23
 WarehouseInventoryBalance(2,1,1): I_warehouse(1)(0)(0)#39
                                 - b_warehouse(1)(0)(0)#159 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -5
 WarehouseInventoryBalance(3,1,1): I_warehouse(2)(0)(0)#69
                                 - b_warehouse(2)(0)(0)#189 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -6
 WarehouseInventoryBalance(4,1,1): I_warehouse(3)(0)(0)#99
                                 - b_warehouse(3)(0)(0)#219 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -6
 WarehouseInventoryBalance(1,2,1): - I_warehouse(0)(0)(0)#9
                                 + I_warehouse(0)(1)(0)#19
                                 - b_warehouse(0)(1)(0)#139 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -59
 WarehouseInventoryBalance(2,2,1): - I_warehouse(1)(0)(0)#39
                                 + I_warehouse(1)(1)(0)#49
                                 - b_warehouse(1)(1)(0)#169 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -29
 WarehouseInventoryBalance(3,2,1): - I_warehouse(2)(0)(0)#69
                                 + I_warehouse(2)(1)(0)#79
                                 - b_warehouse(2)(1)(0)#199 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -46
 WarehouseInventoryBalance(4,2,1): - I_warehouse(3)(0)(0)#99
                                 + I_warehouse(3)(1)(0)#109
                                 - b_warehouse(3)(1)(0)#229 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -29
 WarehouseInventoryBalance(1,3,1): - I_warehouse(0)(1)(0)#19
                                 + I_warehouse(0)(2)(0)#29
                                 - b_warehouse(0)(2)(0)#149 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -127
 WarehouseInventoryBalance(2,3,1): - I_warehouse(1)(1)(0)#49
                                 + I_warehouse(1)(2)(0)#59
                                 - b_warehouse(1)(2)(0)#179 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -29
 WarehouseInventoryBalance(3,3,1): - I_warehouse(2)(1)(0)#79
                                 + I_warehouse(2)(2)(0)#89
                                 - b_warehouse(2)(2)(0)#209 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -62
 WarehouseInventoryBalance(4,3,1): - I_warehouse(3)(1)(0)#109
                                 + I_warehouse(3)(2)(0)#119
                                 - b_warehouse(3)(2)(0)#239 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -44
 WarehouseInventoryBalance(1,1,2): I_warehouse(0)(0)(1)#10
                                 - b_warehouse(0)(0)(1)#130 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -161
 WarehouseInventoryBalance(2,1,2): I_warehouse(1)(0)(1)#40
                                 - b_warehouse(1)(0)(1)#160 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -31
 WarehouseInventoryBalance(3,1,2): I_warehouse(2)(0)(1)#70
                                 - b_warehouse(2)(0)(1)#190 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -69
 WarehouseInventoryBalance(4,1,2): I_warehouse(3)(0)(1)#100
                                 - b_warehouse(3)(0)(1)#220 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -57
 WarehouseInventoryBalance(1,2,2): - I_warehouse(0)(0)(1)#10
                                 + I_warehouse(0)(1)(1)#20
                                 - b_warehouse(0)(1)(1)#140 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -165
 WarehouseInventoryBalance(2,2,2): - I_warehouse(1)(0)(1)#40
                                 + I_warehouse(1)(1)(1)#50
                                 - b_warehouse(1)(1)(1)#170 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -28
 WarehouseInventoryBalance(3,2,2): - I_warehouse(2)(0)(1)#70
                                 + I_warehouse(2)(1)(1)#80
                                 - b_warehouse(2)(1)(1)#200 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -73
 WarehouseInventoryBalance(4,2,2): - I_warehouse(3)(0)(1)#100
                                 + I_warehouse(3)(1)(1)#110
                                 - b_warehouse(3)(1)(1)#230 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -66
 WarehouseInventoryBalance(1,3,2): - I_warehouse(0)(1)(1)#20
                                 + I_warehouse(0)(2)(1)#30
                                 - b_warehouse(0)(2)(1)#150 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -167
 WarehouseInventoryBalance(2,3,2): - I_warehouse(1)(1)(1)#50
                                 + I_warehouse(1)(2)(1)#60
                                 - b_warehouse(1)(2)(1)#180 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -26
 WarehouseInventoryBalance(3,3,2): - I_warehouse(2)(1)(1)#80
                                 + I_warehouse(2)(2)(1)#90
                                 - b_warehouse(2)(2)(1)#210 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -63
 WarehouseInventoryBalance(4,3,2): - I_warehouse(3)(1)(1)#110
                                 + I_warehouse(3)(2)(1)#120
                                 - b_warehouse(3)(2)(1)#240 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -53
 WarehouseInventoryBalance(1,1,3): I_warehouse(0)(0)(2)#11
                                 - b_warehouse(0)(0)(2)#131 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -163
 WarehouseInventoryBalance(2,1,3): I_warehouse(1)(0)(2)#41
                                 - b_warehouse(1)(0)(2)#161 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -31
 WarehouseInventoryBalance(3,1,3): I_warehouse(2)(0)(2)#71
                                 - b_warehouse(2)(0)(2)#191 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -69
 WarehouseInventoryBalance(4,1,3): I_warehouse(3)(0)(2)#101
                                 - b_warehouse(3)(0)(2)#221 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -60
 WarehouseInventoryBalance(1,2,3): - I_warehouse(0)(0)(2)#11
                                 + I_warehouse(0)(1)(2)#21
                                 - b_warehouse(0)(1)(2)#141 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -152
 WarehouseInventoryBalance(2,2,3): - I_warehouse(1)(0)(2)#41
                                 + I_warehouse(1)(1)(2)#51
                                 - b_warehouse(1)(1)(2)#171 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -26
 WarehouseInventoryBalance(3,2,3): - I_warehouse(2)(0)(2)#71
                                 + I_warehouse(2)(1)(2)#81
                                 - b_warehouse(2)(1)(2)#201 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -68
 WarehouseInventoryBalance(4,2,3): - I_warehouse(3)(0)(2)#101
                                 + I_warehouse(3)(1)(2)#111
                                 - b_warehouse(3)(1)(2)#231 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -51
 WarehouseInventoryBalance(1,3,3): - I_warehouse(0)(1)(2)#21
                                 + I_warehouse(0)(2)(2)#31
                                 - b_warehouse(0)(2)(2)#151 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -146
 WarehouseInventoryBalance(2,3,3): - I_warehouse(1)(1)(2)#51
                                 + I_warehouse(1)(2)(2)#61
                                 - b_warehouse(1)(2)(2)#181 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -24
 WarehouseInventoryBalance(3,3,3): - I_warehouse(2)(1)(2)#81
                                 + I_warehouse(2)(2)(2)#91
                                 - b_warehouse(2)(2)(2)#211 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -70
 WarehouseInventoryBalance(4,3,3): - I_warehouse(3)(1)(2)#111
                                 + I_warehouse(3)(2)(2)#121
                                 - b_warehouse(3)(2)(2)#241 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -62
 WarehouseInventoryBalance(1,1,4): I_warehouse(0)(0)(3)#12
                                 - b_warehouse(0)(0)(3)#132 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -155
 WarehouseInventoryBalance(2,1,4): I_warehouse(1)(0)(3)#42
                                 - b_warehouse(1)(0)(3)#162 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -25
 WarehouseInventoryBalance(3,1,4): I_warehouse(2)(0)(3)#72
                                 - b_warehouse(2)(0)(3)#192 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -69
 WarehouseInventoryBalance(4,1,4): I_warehouse(3)(0)(3)#102
                                 - b_warehouse(3)(0)(3)#222 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -62
 WarehouseInventoryBalance(1,2,4): - I_warehouse(0)(0)(3)#12
                                 + I_warehouse(0)(1)(3)#22
                                 - b_warehouse(0)(1)(3)#142 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -159
 WarehouseInventoryBalance(2,2,4): - I_warehouse(1)(0)(3)#42
                                 + I_warehouse(1)(1)(3)#52
                                 - b_warehouse(1)(1)(3)#172 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -29
 WarehouseInventoryBalance(3,2,4): - I_warehouse(2)(0)(3)#72
                                 + I_warehouse(2)(1)(3)#82
                                 - b_warehouse(2)(1)(3)#202 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -67
 WarehouseInventoryBalance(4,2,4): - I_warehouse(3)(0)(3)#102
                                 + I_warehouse(3)(1)(3)#112
                                 - b_warehouse(3)(1)(3)#232 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -49
 WarehouseInventoryBalance(1,3,4): - I_warehouse(0)(1)(3)#22
                                 + I_warehouse(0)(2)(3)#32
                                 - b_warehouse(0)(2)(3)#152 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -157
 WarehouseInventoryBalance(2,3,4): - I_warehouse(1)(1)(3)#52
                                 + I_warehouse(1)(2)(3)#62
                                 - b_warehouse(1)(2)(3)#182 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -26
 WarehouseInventoryBalance(3,3,4): - I_warehouse(2)(1)(3)#82
                                 + I_warehouse(2)(2)(3)#92
                                 - b_warehouse(2)(2)(3)#212 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -71
 WarehouseInventoryBalance(4,3,4): - I_warehouse(3)(1)(3)#112
                                 + I_warehouse(3)(2)(3)#122
                                 - b_warehouse(3)(2)(3)#242 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -59
 WarehouseInventoryBalance(1,1,5): I_warehouse(0)(0)(4)#13
                                 - b_warehouse(0)(0)(4)#133 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -155
 WarehouseInventoryBalance(2,1,5): I_warehouse(1)(0)(4)#43
                                 - b_warehouse(1)(0)(4)#163 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -32
 WarehouseInventoryBalance(3,1,5): I_warehouse(2)(0)(4)#73
                                 - b_warehouse(2)(0)(4)#193 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -75
 WarehouseInventoryBalance(4,1,5): I_warehouse(3)(0)(4)#103
                                 - b_warehouse(3)(0)(4)#223 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -55
 WarehouseInventoryBalance(1,2,5): - I_warehouse(0)(0)(4)#13
                                 + I_warehouse(0)(1)(4)#23
                                 - b_warehouse(0)(1)(4)#143 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -159
 WarehouseInventoryBalance(2,2,5): - I_warehouse(1)(0)(4)#43
                                 + I_warehouse(1)(1)(4)#53
                                 - b_warehouse(1)(1)(4)#173 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -31
 WarehouseInventoryBalance(3,2,5): - I_warehouse(2)(0)(4)#73
                                 + I_warehouse(2)(1)(4)#83
                                 - b_warehouse(2)(1)(4)#203 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -62
 WarehouseInventoryBalance(4,2,5): - I_warehouse(3)(0)(4)#103
                                 + I_warehouse(3)(1)(4)#113
                                 - b_warehouse(3)(1)(4)#233 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -49
 WarehouseInventoryBalance(1,3,5): - I_warehouse(0)(1)(4)#23
                                 + I_warehouse(0)(2)(4)#33
                                 - b_warehouse(0)(2)(4)#153 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -157
 WarehouseInventoryBalance(2,3,5): - I_warehouse(1)(1)(4)#53
                                 + I_warehouse(1)(2)(4)#63
                                 - b_warehouse(1)(2)(4)#183 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -27
 WarehouseInventoryBalance(3,3,5): - I_warehouse(2)(1)(4)#83
                                 + I_warehouse(2)(2)(4)#93
                                 - b_warehouse(2)(2)(4)#213 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -74
 WarehouseInventoryBalance(4,3,5): - I_warehouse(3)(1)(4)#113
                                 + I_warehouse(3)(2)(4)#123
                                 - b_warehouse(3)(2)(4)#243 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -54
 WarehouseInventoryBalance(1,1,6): I_warehouse(0)(0)(5)#14
                                 - b_warehouse(0)(0)(5)#134 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -164
 WarehouseInventoryBalance(2,1,6): I_warehouse(1)(0)(5)#44
                                 - b_warehouse(1)(0)(5)#164 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -27
 WarehouseInventoryBalance(3,1,6): I_warehouse(2)(0)(5)#74
                                 - b_warehouse(2)(0)(5)#194 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -64
 WarehouseInventoryBalance(4,1,6): I_warehouse(3)(0)(5)#104
                                 - b_warehouse(3)(0)(5)#224 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -54
 WarehouseInventoryBalance(1,2,6): - I_warehouse(0)(0)(5)#14
                                 + I_warehouse(0)(1)(5)#24
                                 - b_warehouse(0)(1)(5)#144 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -156
 WarehouseInventoryBalance(2,2,6): - I_warehouse(1)(0)(5)#44
                                 + I_warehouse(1)(1)(5)#54
                                 - b_warehouse(1)(1)(5)#174 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -29
 WarehouseInventoryBalance(3,2,6): - I_warehouse(2)(0)(5)#74
                                 + I_warehouse(2)(1)(5)#84
                                 - b_warehouse(2)(1)(5)#204 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -66
 WarehouseInventoryBalance(4,2,6): - I_warehouse(3)(0)(5)#104
                                 + I_warehouse(3)(1)(5)#114
                                 - b_warehouse(3)(1)(5)#234 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -55
 WarehouseInventoryBalance(1,3,6): - I_warehouse(0)(1)(5)#24
                                 + I_warehouse(0)(2)(5)#34
                                 - b_warehouse(0)(2)(5)#154 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -167
 WarehouseInventoryBalance(2,3,6): - I_warehouse(1)(1)(5)#54
                                 + I_warehouse(1)(2)(5)#64
                                 - b_warehouse(1)(2)(5)#184 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -28
 WarehouseInventoryBalance(3,3,6): - I_warehouse(2)(1)(5)#84
                                 + I_warehouse(2)(2)(5)#94
                                 - b_warehouse(2)(2)(5)#214 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -71
 WarehouseInventoryBalance(4,3,6): - I_warehouse(3)(1)(5)#114
                                 + I_warehouse(3)(2)(5)#124
                                 - b_warehouse(3)(2)(5)#244 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -56
 WarehouseInventoryBalance(1,1,7): I_warehouse(0)(0)(6)#15
                                 - b_warehouse(0)(0)(6)#135 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -160
 WarehouseInventoryBalance(2,1,7): I_warehouse(1)(0)(6)#45
                                 - b_warehouse(1)(0)(6)#165 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -25
 WarehouseInventoryBalance(3,1,7): I_warehouse(2)(0)(6)#75
                                 - b_warehouse(2)(0)(6)#195 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -67
 WarehouseInventoryBalance(4,1,7): I_warehouse(3)(0)(6)#105
                                 - b_warehouse(3)(0)(6)#225 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -55
 WarehouseInventoryBalance(1,2,7): - I_warehouse(0)(0)(6)#15
                                 + I_warehouse(0)(1)(6)#25
                                 - b_warehouse(0)(1)(6)#145 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -159
 WarehouseInventoryBalance(2,2,7): - I_warehouse(1)(0)(6)#45
                                 + I_warehouse(1)(1)(6)#55
                                 - b_warehouse(1)(1)(6)#175 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -26
 WarehouseInventoryBalance(3,2,7): - I_warehouse(2)(0)(6)#75
                                 + I_warehouse(2)(1)(6)#85
                                 - b_warehouse(2)(1)(6)#205 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -61
 WarehouseInventoryBalance(4,2,7): - I_warehouse(3)(0)(6)#105
                                 + I_warehouse(3)(1)(6)#115
                                 - b_warehouse(3)(1)(6)#235 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -51
 WarehouseInventoryBalance(1,3,7): - I_warehouse(0)(1)(6)#25
                                 + I_warehouse(0)(2)(6)#35
                                 - b_warehouse(0)(2)(6)#155 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -156
 WarehouseInventoryBalance(2,3,7): - I_warehouse(1)(1)(6)#55
                                 + I_warehouse(1)(2)(6)#65
                                 - b_warehouse(1)(2)(6)#185 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -29
 WarehouseInventoryBalance(3,3,7): - I_warehouse(2)(1)(6)#85
                                 + I_warehouse(2)(2)(6)#95
                                 - b_warehouse(2)(2)(6)#215 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -66
 WarehouseInventoryBalance(4,3,7): - I_warehouse(3)(1)(6)#115
                                 + I_warehouse(3)(2)(6)#125
                                 - b_warehouse(3)(2)(6)#245 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -58
 WarehouseInventoryBalance(1,1,8): I_warehouse(0)(0)(7)#16
                                 - b_warehouse(0)(0)(7)#136 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -160
 WarehouseInventoryBalance(2,1,8): I_warehouse(1)(0)(7)#46
                                 - b_warehouse(1)(0)(7)#166 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -28
 WarehouseInventoryBalance(3,1,8): I_warehouse(2)(0)(7)#76
                                 - b_warehouse(2)(0)(7)#196 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -69
 WarehouseInventoryBalance(4,1,8): I_warehouse(3)(0)(7)#106
                                 - b_warehouse(3)(0)(7)#226 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -58
 WarehouseInventoryBalance(1,2,8): - I_warehouse(0)(0)(7)#16
                                 + I_warehouse(0)(1)(7)#26
                                 - b_warehouse(0)(1)(7)#146 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -165
 WarehouseInventoryBalance(2,2,8): - I_warehouse(1)(0)(7)#46
                                 + I_warehouse(1)(1)(7)#56
                                 - b_warehouse(1)(1)(7)#176 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -26
 WarehouseInventoryBalance(3,2,8): - I_warehouse(2)(0)(7)#76
                                 + I_warehouse(2)(1)(7)#86
                                 - b_warehouse(2)(1)(7)#206 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -68
 WarehouseInventoryBalance(4,2,8): - I_warehouse(3)(0)(7)#106
                                 + I_warehouse(3)(1)(7)#116
                                 - b_warehouse(3)(1)(7)#236 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -59
 WarehouseInventoryBalance(1,3,8): - I_warehouse(0)(1)(7)#26
                                 + I_warehouse(0)(2)(7)#36
                                 - b_warehouse(0)(2)(7)#156 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -147
 WarehouseInventoryBalance(2,3,8): - I_warehouse(1)(1)(7)#56
                                 + I_warehouse(1)(2)(7)#66
                                 - b_warehouse(1)(2)(7)#186 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -30
 WarehouseInventoryBalance(3,3,8): - I_warehouse(2)(1)(7)#86
                                 + I_warehouse(2)(2)(7)#96
                                 - b_warehouse(2)(2)(7)#216 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -73
 WarehouseInventoryBalance(4,3,8): - I_warehouse(3)(1)(7)#116
                                 + I_warehouse(3)(2)(7)#126
                                 - b_warehouse(3)(2)(7)#246 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -59
 WarehouseInventoryBalance(1,1,9): I_warehouse(0)(0)(8)#17
                                 - b_warehouse(0)(0)(8)#137 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -154
 WarehouseInventoryBalance(2,1,9): I_warehouse(1)(0)(8)#47
                                 - b_warehouse(1)(0)(8)#167 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -29
 WarehouseInventoryBalance(3,1,9): I_warehouse(2)(0)(8)#77
                                 - b_warehouse(2)(0)(8)#197 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -66
 WarehouseInventoryBalance(4,1,9): I_warehouse(3)(0)(8)#107
                                 - b_warehouse(3)(0)(8)#227 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -52
 WarehouseInventoryBalance(1,2,9): - I_warehouse(0)(0)(8)#17
                                 + I_warehouse(0)(1)(8)#27
                                 - b_warehouse(0)(1)(8)#147 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -149
 WarehouseInventoryBalance(2,2,9): - I_warehouse(1)(0)(8)#47
                                 + I_warehouse(1)(1)(8)#57
                                 - b_warehouse(1)(1)(8)#177 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -25
 WarehouseInventoryBalance(3,2,9): - I_warehouse(2)(0)(8)#77
                                 + I_warehouse(2)(1)(8)#87
                                 - b_warehouse(2)(1)(8)#207 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -64
 WarehouseInventoryBalance(4,2,9): - I_warehouse(3)(0)(8)#107
                                 + I_warehouse(3)(1)(8)#117
                                 - b_warehouse(3)(1)(8)#237 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -61
 WarehouseInventoryBalance(1,3,9): - I_warehouse(0)(1)(8)#27
                                 + I_warehouse(0)(2)(8)#37
                                 - b_warehouse(0)(2)(8)#157 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -163
 WarehouseInventoryBalance(2,3,9): - I_warehouse(1)(1)(8)#57
                                 + I_warehouse(1)(2)(8)#67
                                 - b_warehouse(1)(2)(8)#187 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -30
 WarehouseInventoryBalance(3,3,9): - I_warehouse(2)(1)(8)#87
                                 + I_warehouse(2)(2)(8)#97
                                 - b_warehouse(2)(2)(8)#217 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -71
 WarehouseInventoryBalance(4,3,9): - I_warehouse(3)(1)(8)#117
                                 + I_warehouse(3)(2)(8)#127
                                 - b_warehouse(3)(2)(8)#247 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -58
 WarehouseInventoryBalance(1,1,10): I_warehouse(0)(0)(9)#18
                                 - b_warehouse(0)(0)(9)#138 - q(0)(0)(0)#249
                                 - q(1)(0)(0)#261 - q(2)(0)(0)#273
                                 - q(3)(0)(0)#285 - q(4)(0)(0)#297
                                 - q(5)(0)(0)#309 - q(6)(0)(0)#321
                                 - q(7)(0)(0)#333 - q(8)(0)(0)#345
                                 - q(9)(0)(0)#357 - q(10)(0)(0)#369
                                 - q(11)(0)(0)#381 - q(12)(0)(0)#393
                                 - q(13)(0)(0)#405 - q(14)(0)(0)#417  = -154
 WarehouseInventoryBalance(2,1,10): I_warehouse(1)(0)(9)#48
                                 - b_warehouse(1)(0)(9)#168 - q(0)(1)(0)#252
                                 - q(1)(1)(0)#264 - q(2)(1)(0)#276
                                 - q(3)(1)(0)#288 - q(4)(1)(0)#300
                                 - q(5)(1)(0)#312 - q(6)(1)(0)#324
                                 - q(7)(1)(0)#336 - q(8)(1)(0)#348
                                 - q(9)(1)(0)#360 - q(10)(1)(0)#372
                                 - q(11)(1)(0)#384 - q(12)(1)(0)#396
                                 - q(13)(1)(0)#408 - q(14)(1)(0)#420  = -30
 WarehouseInventoryBalance(3,1,10): I_warehouse(2)(0)(9)#78
                                 - b_warehouse(2)(0)(9)#198 - q(0)(2)(0)#255
                                 - q(1)(2)(0)#267 - q(2)(2)(0)#279
                                 - q(3)(2)(0)#291 - q(4)(2)(0)#303
                                 - q(5)(2)(0)#315 - q(6)(2)(0)#327
                                 - q(7)(2)(0)#339 - q(8)(2)(0)#351
                                 - q(9)(2)(0)#363 - q(10)(2)(0)#375
                                 - q(11)(2)(0)#387 - q(12)(2)(0)#399
                                 - q(13)(2)(0)#411 - q(14)(2)(0)#423  = -72
 WarehouseInventoryBalance(4,1,10): I_warehouse(3)(0)(9)#108
                                 - b_warehouse(3)(0)(9)#228 - q(0)(3)(0)#258
                                 - q(1)(3)(0)#270 - q(2)(3)(0)#282
                                 - q(3)(3)(0)#294 - q(4)(3)(0)#306
                                 - q(5)(3)(0)#318 - q(6)(3)(0)#330
                                 - q(7)(3)(0)#342 - q(8)(3)(0)#354
                                 - q(9)(3)(0)#366 - q(10)(3)(0)#378
                                 - q(11)(3)(0)#390 - q(12)(3)(0)#402
                                 - q(13)(3)(0)#414 - q(14)(3)(0)#426  = -57
 WarehouseInventoryBalance(1,2,10): - I_warehouse(0)(0)(9)#18
                                 + I_warehouse(0)(1)(9)#28
                                 - b_warehouse(0)(1)(9)#148 - q(0)(0)(1)#250
                                 - q(1)(0)(1)#262 - q(2)(0)(1)#274
                                 - q(3)(0)(1)#286 - q(4)(0)(1)#298
                                 - q(5)(0)(1)#310 - q(6)(0)(1)#322
                                 - q(7)(0)(1)#334 - q(8)(0)(1)#346
                                 - q(9)(0)(1)#358 - q(10)(0)(1)#370
                                 - q(11)(0)(1)#382 - q(12)(0)(1)#394
                                 - q(13)(0)(1)#406 - q(14)(0)(1)#418  = -150
 WarehouseInventoryBalance(2,2,10): - I_warehouse(1)(0)(9)#48
                                 + I_warehouse(1)(1)(9)#58
                                 - b_warehouse(1)(1)(9)#178 - q(0)(1)(1)#253
                                 - q(1)(1)(1)#265 - q(2)(1)(1)#277
                                 - q(3)(1)(1)#289 - q(4)(1)(1)#301
                                 - q(5)(1)(1)#313 - q(6)(1)(1)#325
                                 - q(7)(1)(1)#337 - q(8)(1)(1)#349
                                 - q(9)(1)(1)#361 - q(10)(1)(1)#373
                                 - q(11)(1)(1)#385 - q(12)(1)(1)#397
                                 - q(13)(1)(1)#409 - q(14)(1)(1)#421  = -30
 WarehouseInventoryBalance(3,2,10): - I_warehouse(2)(0)(9)#78
                                 + I_warehouse(2)(1)(9)#88
                                 - b_warehouse(2)(1)(9)#208 - q(0)(2)(1)#256
                                 - q(1)(2)(1)#268 - q(2)(2)(1)#280
                                 - q(3)(2)(1)#292 - q(4)(2)(1)#304
                                 - q(5)(2)(1)#316 - q(6)(2)(1)#328
                                 - q(7)(2)(1)#340 - q(8)(2)(1)#352
                                 - q(9)(2)(1)#364 - q(10)(2)(1)#376
                                 - q(11)(2)(1)#388 - q(12)(2)(1)#400
                                 - q(13)(2)(1)#412 - q(14)(2)(1)#424  = -65
 WarehouseInventoryBalance(4,2,10): - I_warehouse(3)(0)(9)#108
                                 + I_warehouse(3)(1)(9)#118
                                 - b_warehouse(3)(1)(9)#238 - q(0)(3)(1)#259
                                 - q(1)(3)(1)#271 - q(2)(3)(1)#283
                                 - q(3)(3)(1)#295 - q(4)(3)(1)#307
                                 - q(5)(3)(1)#319 - q(6)(3)(1)#331
                                 - q(7)(3)(1)#343 - q(8)(3)(1)#355
                                 - q(9)(3)(1)#367 - q(10)(3)(1)#379
                                 - q(11)(3)(1)#391 - q(12)(3)(1)#403
                                 - q(13)(3)(1)#415 - q(14)(3)(1)#427  = -57
 WarehouseInventoryBalance(1,3,10): - I_warehouse(0)(1)(9)#28
                                 + I_warehouse(0)(2)(9)#38
                                 - b_warehouse(0)(2)(9)#158 - q(0)(0)(2)#251
                                 - q(1)(0)(2)#263 - q(2)(0)(2)#275
                                 - q(3)(0)(2)#287 - q(4)(0)(2)#299
                                 - q(5)(0)(2)#311 - q(6)(0)(2)#323
                                 - q(7)(0)(2)#335 - q(8)(0)(2)#347
                                 - q(9)(0)(2)#359 - q(10)(0)(2)#371
                                 - q(11)(0)(2)#383 - q(12)(0)(2)#395
                                 - q(13)(0)(2)#407 - q(14)(0)(2)#419  = -150
 WarehouseInventoryBalance(2,3,10): - I_warehouse(1)(1)(9)#58
                                 + I_warehouse(1)(2)(9)#68
                                 - b_warehouse(1)(2)(9)#188 - q(0)(1)(2)#254
                                 - q(1)(1)(2)#266 - q(2)(1)(2)#278
                                 - q(3)(1)(2)#290 - q(4)(1)(2)#302
                                 - q(5)(1)(2)#314 - q(6)(1)(2)#326
                                 - q(7)(1)(2)#338 - q(8)(1)(2)#350
                                 - q(9)(1)(2)#362 - q(10)(1)(2)#374
                                 - q(11)(1)(2)#386 - q(12)(1)(2)#398
                                 - q(13)(1)(2)#410 - q(14)(1)(2)#422  = -28
 WarehouseInventoryBalance(3,3,10): - I_warehouse(2)(1)(9)#88
                                 + I_warehouse(2)(2)(9)#98
                                 - b_warehouse(2)(2)(9)#218 - q(0)(2)(2)#257
                                 - q(1)(2)(2)#269 - q(2)(2)(2)#281
                                 - q(3)(2)(2)#293 - q(4)(2)(2)#305
                                 - q(5)(2)(2)#317 - q(6)(2)(2)#329
                                 - q(7)(2)(2)#341 - q(8)(2)(2)#353
                                 - q(9)(2)(2)#365 - q(10)(2)(2)#377
                                 - q(11)(2)(2)#389 - q(12)(2)(2)#401
                                 - q(13)(2)(2)#413 - q(14)(2)(2)#425  = -71
 WarehouseInventoryBalance(4,3,10): - I_warehouse(3)(1)(9)#118
                                 + I_warehouse(3)(2)(9)#128
                                 - b_warehouse(3)(2)(9)#248 - q(0)(3)(2)#260
                                 - q(1)(3)(2)#272 - q(2)(3)(2)#284
                                 - q(3)(3)(2)#296 - q(4)(3)(2)#308
                                 - q(5)(3)(2)#320 - q(6)(3)(2)#332
                                 - q(7)(3)(2)#344 - q(8)(3)(2)#356
                                 - q(9)(3)(2)#368 - q(10)(3)(2)#380
                                 - q(11)(3)(2)#392 - q(12)(3)(2)#404
                                 - q(13)(3)(2)#416 - q(14)(3)(2)#428  = -57
 WarehouseCoverage(1,1):         o(0)(0)#429 + o(2)(0)#435 + o(4)(0)#441
                                 + o(6)(0)#447 + o(8)(0)#453 + o(10)(0)#459
                                 + o(12)(0)#465 + o(14)(0)#471 <= 1
 WarehouseCoverage(2,1):         o(1)(0)#432 + o(2)(0)#435 + o(5)(0)#444
                                 + o(6)(0)#447 + o(9)(0)#456 + o(10)(0)#459
                                 + o(13)(0)#468 + o(14)(0)#471 <= 1
 WarehouseCoverage(3,1):         o(3)(0)#438 + o(4)(0)#441 + o(5)(0)#444
                                 + o(6)(0)#447 + o(11)(0)#462 + o(12)(0)#465
                                 + o(13)(0)#468 + o(14)(0)#471 <= 1
 WarehouseCoverage(4,1):         o(7)(0)#450 + o(8)(0)#453 + o(9)(0)#456
                                 + o(10)(0)#459 + o(11)(0)#462 + o(12)(0)#465
                                 + o(13)(0)#468 + o(14)(0)#471 <= 1
 WarehouseCoverage(1,2):         o(0)(1)#430 + o(2)(1)#436 + o(4)(1)#442
                                 + o(6)(1)#448 + o(8)(1)#454 + o(10)(1)#460
                                 + o(12)(1)#466 + o(14)(1)#472 <= 1
 WarehouseCoverage(2,2):         o(1)(1)#433 + o(2)(1)#436 + o(5)(1)#445
                                 + o(6)(1)#448 + o(9)(1)#457 + o(10)(1)#460
                                 + o(13)(1)#469 + o(14)(1)#472 <= 1
 WarehouseCoverage(3,2):         o(3)(1)#439 + o(4)(1)#442 + o(5)(1)#445
                                 + o(6)(1)#448 + o(11)(1)#463 + o(12)(1)#466
                                 + o(13)(1)#469 + o(14)(1)#472 <= 1
 WarehouseCoverage(4,2):         o(7)(1)#451 + o(8)(1)#454 + o(9)(1)#457
                                 + o(10)(1)#460 + o(11)(1)#463 + o(12)(1)#466
                                 + o(13)(1)#469 + o(14)(1)#472 <= 1
 WarehouseCoverage(1,3):         o(0)(2)#431 + o(2)(2)#437 + o(4)(2)#443
                                 + o(6)(2)#449 + o(8)(2)#455 + o(10)(2)#461
                                 + o(12)(2)#467 + o(14)(2)#473 <= 1
 WarehouseCoverage(2,3):         o(1)(2)#434 + o(2)(2)#437 + o(5)(2)#446
                                 + o(6)(2)#449 + o(9)(2)#458 + o(10)(2)#461
                                 + o(13)(2)#470 + o(14)(2)#473 <= 1
 WarehouseCoverage(3,3):         o(3)(2)#440 + o(4)(2)#443 + o(5)(2)#446
                                 + o(6)(2)#449 + o(11)(2)#464 + o(12)(2)#467
                                 + o(13)(2)#470 + o(14)(2)#473 <= 1
 WarehouseCoverage(4,3):         o(7)(2)#452 + o(8)(2)#455 + o(9)(2)#458
                                 + o(10)(2)#461 + o(11)(2)#464 + o(12)(2)#467
                                 + o(13)(2)#470 + o(14)(2)#473 <= 1
 VehicleCapacity(1,1):           q(0)(0)(0)#249 + q(0)(1)(0)#252
                                 + q(0)(2)(0)#255 + q(0)(3)(0)#258
                                 - 209 o(0)(0)#429 <= 0
 VehicleCapacity(2,1):           q(1)(0)(0)#261 + q(1)(1)(0)#264
                                 + q(1)(2)(0)#267 + q(1)(3)(0)#270
                                 - 209 o(1)(0)#432 <= 0
 VehicleCapacity(3,1):           q(2)(0)(0)#273 + q(2)(1)(0)#276
                                 + q(2)(2)(0)#279 + q(2)(3)(0)#282
                                 - 209 o(2)(0)#435 <= 0
 VehicleCapacity(4,1):           q(3)(0)(0)#285 + q(3)(1)(0)#288
                                 + q(3)(2)(0)#291 + q(3)(3)(0)#294
                                 - 209 o(3)(0)#438 <= 0
 VehicleCapacity(5,1):           q(4)(0)(0)#297 + q(4)(1)(0)#300
                                 + q(4)(2)(0)#303 + q(4)(3)(0)#306
                                 - 209 o(4)(0)#441 <= 0
 VehicleCapacity(6,1):           q(5)(0)(0)#309 + q(5)(1)(0)#312
                                 + q(5)(2)(0)#315 + q(5)(3)(0)#318
                                 - 209 o(5)(0)#444 <= 0
 VehicleCapacity(7,1):           q(6)(0)(0)#321 + q(6)(1)(0)#324
                                 + q(6)(2)(0)#327 + q(6)(3)(0)#330
                                 - 209 o(6)(0)#447 <= 0
 VehicleCapacity(8,1):           q(7)(0)(0)#333 + q(7)(1)(0)#336
                                 + q(7)(2)(0)#339 + q(7)(3)(0)#342
                                 - 209 o(7)(0)#450 <= 0
 VehicleCapacity(9,1):           q(8)(0)(0)#345 + q(8)(1)(0)#348
                                 + q(8)(2)(0)#351 + q(8)(3)(0)#354
                                 - 209 o(8)(0)#453 <= 0
 VehicleCapacity(10,1):          q(9)(0)(0)#357 + q(9)(1)(0)#360
                                 + q(9)(2)(0)#363 + q(9)(3)(0)#366
                                 - 209 o(9)(0)#456 <= 0
 VehicleCapacity(11,1):          q(10)(0)(0)#369 + q(10)(1)(0)#372
                                 + q(10)(2)(0)#375 + q(10)(3)(0)#378
                                 - 209 o(10)(0)#459 <= 0
 VehicleCapacity(12,1):          q(11)(0)(0)#381 + q(11)(1)(0)#384
                                 + q(11)(2)(0)#387 + q(11)(3)(0)#390
                                 - 209 o(11)(0)#462 <= 0
 VehicleCapacity(13,1):          q(12)(0)(0)#393 + q(12)(1)(0)#396
                                 + q(12)(2)(0)#399 + q(12)(3)(0)#402
                                 - 209 o(12)(0)#465 <= 0
 VehicleCapacity(14,1):          q(13)(0)(0)#405 + q(13)(1)(0)#408
                                 + q(13)(2)(0)#411 + q(13)(3)(0)#414
                                 - 209 o(13)(0)#468 <= 0
 VehicleCapacity(15,1):          q(14)(0)(0)#417 + q(14)(1)(0)#420
                                 + q(14)(2)(0)#423 + q(14)(3)(0)#426
                                 - 209 o(14)(0)#471 <= 0
 VehicleCapacity(1,2):           q(0)(0)(1)#250 + q(0)(1)(1)#253
                                 + q(0)(2)(1)#256 + q(0)(3)(1)#259
                                 - 209 o(0)(1)#430 <= 0
 VehicleCapacity(2,2):           q(1)(0)(1)#262 + q(1)(1)(1)#265
                                 + q(1)(2)(1)#268 + q(1)(3)(1)#271
                                 - 209 o(1)(1)#433 <= 0
 VehicleCapacity(3,2):           q(2)(0)(1)#274 + q(2)(1)(1)#277
                                 + q(2)(2)(1)#280 + q(2)(3)(1)#283
                                 - 209 o(2)(1)#436 <= 0
 VehicleCapacity(4,2):           q(3)(0)(1)#286 + q(3)(1)(1)#289
                                 + q(3)(2)(1)#292 + q(3)(3)(1)#295
                                 - 209 o(3)(1)#439 <= 0
 VehicleCapacity(5,2):           q(4)(0)(1)#298 + q(4)(1)(1)#301
                                 + q(4)(2)(1)#304 + q(4)(3)(1)#307
                                 - 209 o(4)(1)#442 <= 0
 VehicleCapacity(6,2):           q(5)(0)(1)#310 + q(5)(1)(1)#313
                                 + q(5)(2)(1)#316 + q(5)(3)(1)#319
                                 - 209 o(5)(1)#445 <= 0
 VehicleCapacity(7,2):           q(6)(0)(1)#322 + q(6)(1)(1)#325
                                 + q(6)(2)(1)#328 + q(6)(3)(1)#331
                                 - 209 o(6)(1)#448 <= 0
 VehicleCapacity(8,2):           q(7)(0)(1)#334 + q(7)(1)(1)#337
                                 + q(7)(2)(1)#340 + q(7)(3)(1)#343
                                 - 209 o(7)(1)#451 <= 0
 VehicleCapacity(9,2):           q(8)(0)(1)#346 + q(8)(1)(1)#349
                                 + q(8)(2)(1)#352 + q(8)(3)(1)#355
                                 - 209 o(8)(1)#454 <= 0
 VehicleCapacity(10,2):          q(9)(0)(1)#358 + q(9)(1)(1)#361
                                 + q(9)(2)(1)#364 + q(9)(3)(1)#367
                                 - 209 o(9)(1)#457 <= 0
 VehicleCapacity(11,2):          q(10)(0)(1)#370 + q(10)(1)(1)#373
                                 + q(10)(2)(1)#376 + q(10)(3)(1)#379
                                 - 209 o(10)(1)#460 <= 0
 VehicleCapacity(12,2):          q(11)(0)(1)#382 + q(11)(1)(1)#385
                                 + q(11)(2)(1)#388 + q(11)(3)(1)#391
                                 - 209 o(11)(1)#463 <= 0
 VehicleCapacity(13,2):          q(12)(0)(1)#394 + q(12)(1)(1)#397
                                 + q(12)(2)(1)#400 + q(12)(3)(1)#403
                                 - 209 o(12)(1)#466 <= 0
 VehicleCapacity(14,2):          q(13)(0)(1)#406 + q(13)(1)(1)#409
                                 + q(13)(2)(1)#412 + q(13)(3)(1)#415
                                 - 209 o(13)(1)#469 <= 0
 VehicleCapacity(15,2):          q(14)(0)(1)#418 + q(14)(1)(1)#421
                                 + q(14)(2)(1)#424 + q(14)(3)(1)#427
                                 - 209 o(14)(1)#472 <= 0
 VehicleCapacity(1,3):           q(0)(0)(2)#251 + q(0)(1)(2)#254
                                 + q(0)(2)(2)#257 + q(0)(3)(2)#260
                                 - 209 o(0)(2)#431 <= 0
 VehicleCapacity(2,3):           q(1)(0)(2)#263 + q(1)(1)(2)#266
                                 + q(1)(2)(2)#269 + q(1)(3)(2)#272
                                 - 209 o(1)(2)#434 <= 0
 VehicleCapacity(3,3):           q(2)(0)(2)#275 + q(2)(1)(2)#278
                                 + q(2)(2)(2)#281 + q(2)(3)(2)#284
                                 - 209 o(2)(2)#437 <= 0
 VehicleCapacity(4,3):           q(3)(0)(2)#287 + q(3)(1)(2)#290
                                 + q(3)(2)(2)#293 + q(3)(3)(2)#296
                                 - 209 o(3)(2)#440 <= 0
 VehicleCapacity(5,3):           q(4)(0)(2)#299 + q(4)(1)(2)#302
                                 + q(4)(2)(2)#305 + q(4)(3)(2)#308
                                 - 209 o(4)(2)#443 <= 0
 VehicleCapacity(6,3):           q(5)(0)(2)#311 + q(5)(1)(2)#314
                                 + q(5)(2)(2)#317 + q(5)(3)(2)#320
                                 - 209 o(5)(2)#446 <= 0
 VehicleCapacity(7,3):           q(6)(0)(2)#323 + q(6)(1)(2)#326
                                 + q(6)(2)(2)#329 + q(6)(3)(2)#332
                                 - 209 o(6)(2)#449 <= 0
 VehicleCapacity(8,3):           q(7)(0)(2)#335 + q(7)(1)(2)#338
                                 + q(7)(2)(2)#341 + q(7)(3)(2)#344
                                 - 209 o(7)(2)#452 <= 0
 VehicleCapacity(9,3):           q(8)(0)(2)#347 + q(8)(1)(2)#350
                                 + q(8)(2)(2)#353 + q(8)(3)(2)#356
                                 - 209 o(8)(2)#455 <= 0
 VehicleCapacity(10,3):          q(9)(0)(2)#359 + q(9)(1)(2)#362
                                 + q(9)(2)(2)#365 + q(9)(3)(2)#368
                                 - 209 o(9)(2)#458 <= 0
 VehicleCapacity(11,3):          q(10)(0)(2)#371 + q(10)(1)(2)#374
                                 + q(10)(2)(2)#377 + q(10)(3)(2)#380
                                 - 209 o(10)(2)#461 <= 0
 VehicleCapacity(12,3):          q(11)(0)(2)#383 + q(11)(1)(2)#386
                                 + q(11)(2)(2)#389 + q(11)(3)(2)#392
                                 - 209 o(11)(2)#464 <= 0
 VehicleCapacity(13,3):          q(12)(0)(2)#395 + q(12)(1)(2)#398
                                 + q(12)(2)(2)#401 + q(12)(3)(2)#404
                                 - 209 o(12)(2)#467 <= 0
 VehicleCapacity(14,3):          q(13)(0)(2)#407 + q(13)(1)(2)#410
                                 + q(13)(2)(2)#413 + q(13)(3)(2)#416
                                 - 209 o(13)(2)#470 <= 0
 VehicleCapacity(15,3):          q(14)(0)(2)#419 + q(14)(1)(2)#422
                                 + q(14)(2)(2)#425 + q(14)(3)(2)#428
                                 - 209 o(14)(2)#473 <= 0
 FleetSize(1):                   o(0)(0)#429 + o(1)(0)#432 + o(2)(0)#435
                                 + o(3)(0)#438 + o(4)(0)#441 + o(5)(0)#444
                                 + o(6)(0)#447 + o(7)(0)#450 + o(8)(0)#453
                                 + o(9)(0)#456 + o(10)(0)#459 + o(11)(0)#462
                                 + o(12)(0)#465 + o(13)(0)#468 + o(14)(0)#471
                                 <= 2
 FleetSize(2):                   o(0)(1)#430 + o(1)(1)#433 + o(2)(1)#436
                                 + o(3)(1)#439 + o(4)(1)#442 + o(5)(1)#445
                                 + o(6)(1)#448 + o(7)(1)#451 + o(8)(1)#454
                                 + o(9)(1)#457 + o(10)(1)#460 + o(11)(1)#463
                                 + o(12)(1)#466 + o(13)(1)#469 + o(14)(1)#472
                                 <= 2
 FleetSize(3):                   o(0)(2)#431 + o(1)(2)#434 + o(2)(2)#437
                                 + o(3)(2)#440 + o(4)(2)#443 + o(5)(2)#446
                                 + o(6)(2)#449 + o(7)(2)#452 + o(8)(2)#455
                                 + o(9)(2)#458 + o(10)(2)#461 + o(11)(2)#464
                                 + o(12)(2)#467 + o(13)(2)#470 + o(14)(2)#473
                                 <= 2
Bounds
 0 <= y(0)#0 <= 1
 0 <= y(1)#1 <= 1
 0 <= y(2)#2 <= 1
 0 <= p(0)#3 <= 622
 0 <= p(1)#4 <= 622
 0 <= p(2)#5 <= 622
 0 <= I_plant(0)#6 <= 311
 0 <= I_plant(1)#7 <= 311
 0 <= I_plant(2)#8 <= 311
 0 <= I_warehouse(0)(0)(0)#9 <= 279.9
 0 <= I_warehouse(0)(0)(1)#10 <= 279.9
 0 <= I_warehouse(0)(0)(2)#11 <= 279.9
 0 <= I_warehouse(0)(0)(3)#12 <= 279.9
 0 <= I_warehouse(0)(0)(4)#13 <= 279.9
 0 <= I_warehouse(0)(0)(5)#14 <= 279.9
 0 <= I_warehouse(0)(0)(6)#15 <= 279.9
 0 <= I_warehouse(0)(0)(7)#16 <= 279.9
 0 <= I_warehouse(0)(0)(8)#17 <= 279.9
 0 <= I_warehouse(0)(0)(9)#18 <= 279.9
 0 <= I_warehouse(0)(1)(0)#19 <= 279.9
 0 <= I_warehouse(0)(1)(1)#20 <= 279.9
 0 <= I_warehouse(0)(1)(2)#21 <= 279.9
 0 <= I_warehouse(0)(1)(3)#22 <= 279.9
 0 <= I_warehouse(0)(1)(4)#23 <= 279.9
 0 <= I_warehouse(0)(1)(5)#24 <= 279.9
 0 <= I_warehouse(0)(1)(6)#25 <= 279.9
 0 <= I_warehouse(0)(1)(7)#26 <= 279.9
 0 <= I_warehouse(0)(1)(8)#27 <= 279.9
 0 <= I_warehouse(0)(1)(9)#28 <= 279.9
 0 <= I_warehouse(0)(2)(0)#29 <= 279.9
 0 <= I_warehouse(0)(2)(1)#30 <= 279.9
 0 <= I_warehouse(0)(2)(2)#31 <= 279.9
 0 <= I_warehouse(0)(2)(3)#32 <= 279.9
 0 <= I_warehouse(0)(2)(4)#33 <= 279.9
 0 <= I_warehouse(0)(2)(5)#34 <= 279.9
 0 <= I_warehouse(0)(2)(6)#35 <= 279.9
 0 <= I_warehouse(0)(2)(7)#36 <= 279.9
 0 <= I_warehouse(0)(2)(8)#37 <= 279.9
 0 <= I_warehouse(0)(2)(9)#38 <= 279.9
 0 <= I_warehouse(1)(0)(0)#39 <= 279.9
 0 <= I_warehouse(1)(0)(1)#40 <= 279.9
 0 <= I_warehouse(1)(0)(2)#41 <= 279.9
 0 <= I_warehouse(1)(0)(3)#42 <= 279.9
 0 <= I_warehouse(1)(0)(4)#43 <= 279.9
 0 <= I_warehouse(1)(0)(5)#44 <= 279.9
 0 <= I_warehouse(1)(0)(6)#45 <= 279.9
 0 <= I_warehouse(1)(0)(7)#46 <= 279.9
 0 <= I_warehouse(1)(0)(8)#47 <= 279.9
 0 <= I_warehouse(1)(0)(9)#48 <= 279.9
 0 <= I_warehouse(1)(1)(0)#49 <= 279.9
 0 <= I_warehouse(1)(1)(1)#50 <= 279.9
 0 <= I_warehouse(1)(1)(2)#51 <= 279.9
 0 <= I_warehouse(1)(1)(3)#52 <= 279.9
 0 <= I_warehouse(1)(1)(4)#53 <= 279.9
 0 <= I_warehouse(1)(1)(5)#54 <= 279.9
 0 <= I_warehouse(1)(1)(6)#55 <= 279.9
 0 <= I_warehouse(1)(1)(7)#56 <= 279.9
 0 <= I_warehouse(1)(1)(8)#57 <= 279.9
 0 <= I_warehouse(1)(1)(9)#58 <= 279.9
 0 <= I_warehouse(1)(2)(0)#59 <= 279.9
 0 <= I_warehouse(1)(2)(1)#60 <= 279.9
 0 <= I_warehouse(1)(2)(2)#61 <= 279.9
 0 <= I_warehouse(1)(2)(3)#62 <= 279.9
 0 <= I_warehouse(1)(2)(4)#63 <= 279.9
 0 <= I_warehouse(1)(2)(5)#64 <= 279.9
 0 <= I_warehouse(1)(2)(6)#65 <= 279.9
 0 <= I_warehouse(1)(2)(7)#66 <= 279.9
 0 <= I_warehouse(1)(2)(8)#67 <= 279.9
 0 <= I_warehouse(1)(2)(9)#68 <= 279.9
 0 <= I_warehouse(2)(0)(0)#69 <= 279.9
 0 <= I_warehouse(2)(0)(1)#70 <= 279.9
 0 <= I_warehouse(2)(0)(2)#71 <= 279.9
 0 <= I_warehouse(2)(0)(3)#72 <= 279.9
 0 <= I_warehouse(2)(0)(4)#73 <= 279.9
 0 <= I_warehouse(2)(0)(5)#74 <= 279.9
 0 <= I_warehouse(2)(0)(6)#75 <= 279.9
 0 <= I_warehouse(2)(0)(7)#76 <= 279.9
 0 <= I_warehouse(2)(0)(8)#77 <= 279.9
 0 <= I_warehouse(2)(0)(9)#78 <= 279.9
 0 <= I_warehouse(2)(1)(0)#79 <= 279.9
 0 <= I_warehouse(2)(1)(1)#80 <= 279.9
 0 <= I_warehouse(2)(1)(2)#81 <= 279.9
 0 <= I_warehouse(2)(1)(3)#82 <= 279.9
 0 <= I_warehouse(2)(1)(4)#83 <= 279.9
 0 <= I_warehouse(2)(1)(5)#84 <= 279.9
 0 <= I_warehouse(2)(1)(6)#85 <= 279.9
 0 <= I_warehouse(2)(1)(7)#86 <= 279.9
 0 <= I_warehouse(2)(1)(8)#87 <= 279.9
 0 <= I_warehouse(2)(1)(9)#88 <= 279.9
 0 <= I_warehouse(2)(2)(0)#89 <= 279.9
 0 <= I_warehouse(2)(2)(1)#90 <= 279.9
 0 <= I_warehouse(2)(2)(2)#91 <= 279.9
 0 <= I_warehouse(2)(2)(3)#92 <= 279.9
 0 <= I_warehouse(2)(2)(4)#93 <= 279.9
 0 <= I_warehouse(2)(2)(5)#94 <= 279.9
 0 <= I_warehouse(2)(2)(6)#95 <= 279.9
 0 <= I_warehouse(2)(2)(7)#96 <= 279.9
 0 <= I_warehouse(2)(2)(8)#97 <= 279.9
 0 <= I_warehouse(2)(2)(9)#98 <= 279.9
 0 <= I_warehouse(3)(0)(0)#99 <= 279.9
 0 <= I_warehouse(3)(0)(1)#100 <= 279.9
 0 <= I_warehouse(3)(0)(2)#101 <= 279.9
 0 <= I_warehouse(3)(0)(3)#102 <= 279.9
 0 <= I_warehouse(3)(0)(4)#103 <= 279.9
 0 <= I_warehouse(3)(0)(5)#104 <= 279.9
 0 <= I_warehouse(3)(0)(6)#105 <= 279.9
 0 <= I_warehouse(3)(0)(7)#106 <= 279.9
 0 <= I_warehouse(3)(0)(8)#107 <= 279.9
 0 <= I_warehouse(3)(0)(9)#108 <= 279.9
 0 <= I_warehouse(3)(1)(0)#109 <= 279.9
 0 <= I_warehouse(3)(1)(1)#110 <= 279.9
 0 <= I_warehouse(3)(1)(2)#111 <= 279.9
 0 <= I_warehouse(3)(1)(3)#112 <= 279.9
 0 <= I_warehouse(3)(1)(4)#113 <= 279.9
 0 <= I_warehouse(3)(1)(5)#114 <= 279.9
 0 <= I_warehouse(3)(1)(6)#115 <= 279.9
 0 <= I_warehouse(3)(1)(7)#116 <= 279.9
 0 <= I_warehouse(3)(1)(8)#117 <= 279.9
 0 <= I_warehouse(3)(1)(9)#118 <= 279.9
 0 <= I_warehouse(3)(2)(0)#119 <= 279.9
 0 <= I_warehouse(3)(2)(1)#120 <= 279.9
 0 <= I_warehouse(3)(2)(2)#121 <= 279.9
 0 <= I_warehouse(3)(2)(3)#122 <= 279.9
 0 <= I_warehouse(3)(2)(4)#123 <= 279.9
 0 <= I_warehouse(3)(2)(5)#124 <= 279.9
 0 <= I_warehouse(3)(2)(6)#125 <= 279.9
 0 <= I_warehouse(3)(2)(7)#126 <= 279.9
 0 <= I_warehouse(3)(2)(8)#127 <= 279.9
 0 <= I_warehouse(3)(2)(9)#128 <= 279.9
 0 <= b_warehouse(0)(0)(0)#129 <= 23
 0 <= b_warehouse(0)(0)(1)#130 <= 161
 0 <= b_warehouse(0)(0)(2)#131 <= 163
 0 <= b_warehouse(0)(0)(3)#132 <= 155
 0 <= b_warehouse(0)(0)(4)#133 <= 155
 0 <= b_warehouse(0)(0)(5)#134 <= 164
 0 <= b_warehouse(0)(0)(6)#135 <= 160
 0 <= b_warehouse(0)(0)(7)#136 <= 160
 0 <= b_warehouse(0)(0)(8)#137 <= 154
 0 <= b_warehouse(0)(0)(9)#138 <= 154
 0 <= b_warehouse(0)(1)(0)#139 <= 59
 0 <= b_warehouse(0)(1)(1)#140 <= 165
 0 <= b_warehouse(0)(1)(2)#141 <= 152
 0 <= b_warehouse(0)(1)(3)#142 <= 159
 0 <= b_warehouse(0)(1)(4)#143 <= 159
 0 <= b_warehouse(0)(1)(5)#144 <= 156
 0 <= b_warehouse(0)(1)(6)#145 <= 159
 0 <= b_warehouse(0)(1)(7)#146 <= 165
 0 <= b_warehouse(0)(1)(8)#147 <= 149
 0 <= b_warehouse(0)(1)(9)#148 <= 150
 0 <= b_warehouse(0)(2)(0)#149 <= 127
 0 <= b_warehouse(0)(2)(1)#150 <= 167
 0 <= b_warehouse(0)(2)(2)#151 <= 146
 0 <= b_warehouse(0)(2)(3)#152 <= 157
 0 <= b_warehouse(0)(2)(4)#153 <= 157
 0 <= b_warehouse(0)(2)(5)#154 <= 167
 0 <= b_warehouse(0)(2)(6)#155 <= 156
 0 <= b_warehouse(0)(2)(7)#156 <= 147
 0 <= b_warehouse(0)(2)(8)#157 <= 163
 0 <= b_warehouse(0)(2)(9)#158 <= 150
 0 <= b_warehouse(1)(0)(0)#159 <= 5
 0 <= b_warehouse(1)(0)(1)#160 <= 31
 0 <= b_warehouse(1)(0)(2)#161 <= 31
 0 <= b_warehouse(1)(0)(3)#162 <= 25
 0 <= b_warehouse(1)(0)(4)#163 <= 32
 0 <= b_warehouse(1)(0)(5)#164 <= 27
 0 <= b_warehouse(1)(0)(6)#165 <= 25
 0 <= b_warehouse(1)(0)(7)#166 <= 28
 0 <= b_warehouse(1)(0)(8)#167 <= 29
 0 <= b_warehouse(1)(0)(9)#168 <= 30
 0 <= b_warehouse(1)(1)(0)#169 <= 29
 0 <= b_warehouse(1)(1)(1)#170 <= 28
 0 <= b_warehouse(1)(1)(2)#171 <= 26
 0 <= b_warehouse(1)(1)(3)#172 <= 29
 0 <= b_warehouse(1)(1)(4)#173 <= 31
 0 <= b_warehouse(1)(1)(5)#174 <= 29
 0 <= b_warehouse(1)(1)(6)#175 <= 26
 0 <= b_warehouse(1)(1)(7)#176 <= 26
 0 <= b_warehouse(1)(1)(8)#177 <= 25
 0 <= b_warehouse(1)(1)(9)#178 <= 30
 0 <= b_warehouse(1)(2)(0)#179 <= 29
 0 <= b_warehouse(1)(2)(1)#180 <= 26
 0 <= b_warehouse(1)(2)(2)#181 <= 24
 0 <= b_warehouse(1)(2)(3)#182 <= 26
 0 <= b_warehouse(1)(2)(4)#183 <= 27
 0 <= b_warehouse(1)(2)(5)#184 <= 28
 0 <= b_warehouse(1)(2)(6)#185 <= 29
 0 <= b_warehouse(1)(2)(7)#186 <= 30
 0 <= b_warehouse(1)(2)(8)#187 <= 30
 0 <= b_warehouse(1)(2)(9)#188 <= 28
 0 <= b_warehouse(2)(0)(0)#189 <= 6
 0 <= b_warehouse(2)(0)(1)#190 <= 69
 0 <= b_warehouse(2)(0)(2)#191 <= 69
 0 <= b_warehouse(2)(0)(3)#192 <= 69
 0 <= b_warehouse(2)(0)(4)#193 <= 75
 0 <= b_warehouse(2)(0)(5)#194 <= 64
 0 <= b_warehouse(2)(0)(6)#195 <= 67
 0 <= b_warehouse(2)(0)(7)#196 <= 69
 0 <= b_warehouse(2)(0)(8)#197 <= 66
 0 <= b_warehouse(2)(0)(9)#198 <= 72
 0 <= b_warehouse(2)(1)(0)#199 <= 46
 0 <= b_warehouse(2)(1)(1)#200 <= 73
 0 <= b_warehouse(2)(1)(2)#201 <= 68
 0 <= b_warehouse(2)(1)(3)#202 <= 67
 0 <= b_warehouse(2)(1)(4)#203 <= 62
 0 <= b_warehouse(2)(1)(5)#204 <= 66
 0 <= b_warehouse(2)(1)(6)#205 <= 61
 0 <= b_warehouse(2)(1)(7)#206 <= 68
 0 <= b_warehouse(2)(1)(8)#207 <= 64
 0 <= b_warehouse(2)(1)(9)#208 <= 65
 0 <= b_warehouse(2)(2)(0)#209 <= 62
 0 <= b_warehouse(2)(2)(1)#210 <= 63
 0 <= b_warehouse(2)(2)(2)#211 <= 70
 0 <= b_warehouse(2)(2)(3)#212 <= 71
 0 <= b_warehouse(2)(2)(4)#213 <= 74
 0 <= b_warehouse(2)(2)(5)#214 <= 71
 0 <= b_warehouse(2)(2)(6)#215 <= 66
 0 <= b_warehouse(2)(2)(7)#216 <= 73
 0 <= b_warehouse(2)(2)(8)#217 <= 71
 0 <= b_warehouse(2)(2)(9)#218 <= 71
 0 <= b_warehouse(3)(0)(0)#219 <= 6
 0 <= b_warehouse(3)(0)(1)#220 <= 57
 0 <= b_warehouse(3)(0)(2)#221 <= 60
 0 <= b_warehouse(3)(0)(3)#222 <= 62
 0 <= b_warehouse(3)(0)(4)#223 <= 55
 0 <= b_warehouse(3)(0)(5)#224 <= 54
 0 <= b_warehouse(3)(0)(6)#225 <= 55
 0 <= b_warehouse(3)(0)(7)#226 <= 58
 0 <= b_warehouse(3)(0)(8)#227 <= 52
 0 <= b_warehouse(3)(0)(9)#228 <= 57
 0 <= b_warehouse(3)(1)(0)#229 <= 29
 0 <= b_warehouse(3)(1)(1)#230 <= 66
 0 <= b_warehouse(3)(1)(2)#231 <= 51
 0 <= b_warehouse(3)(1)(3)#232 <= 49
 0 <= b_warehouse(3)(1)(4)#233 <= 49
 0 <= b_warehouse(3)(1)(5)#234 <= 55
 0 <= b_warehouse(3)(1)(6)#235 <= 51
 0 <= b_warehouse(3)(1)(7)#236 <= 59
 0 <= b_warehouse(3)(1)(8)#237 <= 61
 0 <= b_warehouse(3)(1)(9)#238 <= 57
 0 <= b_warehouse(3)(2)(0)#239 <= 44
 0 <= b_warehouse(3)(2)(1)#240 <= 53
 0 <= b_warehouse(3)(2)(2)#241 <= 62
 0 <= b_warehouse(3)(2)(3)#242 <= 59
 0 <= b_warehouse(3)(2)(4)#243 <= 54
 0 <= b_warehouse(3)(2)(5)#244 <= 56
 0 <= b_warehouse(3)(2)(6)#245 <= 58
 0 <= b_warehouse(3)(2)(7)#246 <= 59
 0 <= b_warehouse(3)(2)(8)#247 <= 58
 0 <= b_warehouse(3)(2)(9)#248 <= 57
 0 <= q(0)(0)(0)#249 <= 279.9
 0 <= q(0)(0)(1)#250 <= 279.9
 0 <= q(0)(0)(2)#251 <= 279.9
      q(0)(1)(0)#252 = 0
      q(0)(1)(1)#253 = 0
      q(0)(1)(2)#254 = 0
      q(0)(2)(0)#255 = 0
      q(0)(2)(1)#256 = 0
      q(0)(2)(2)#257 = 0
      q(0)(3)(0)#258 = 0
      q(0)(3)(1)#259 = 0
      q(0)(3)(2)#260 = 0
      q(1)(0)(0)#261 = 0
      q(1)(0)(1)#262 = 0
      q(1)(0)(2)#263 = 0
 0 <= q(1)(1)(0)#264 <= 279.9
 0 <= q(1)(1)(1)#265 <= 279.9
 0 <= q(1)(1)(2)#266 <= 279.9
      q(1)(2)(0)#267 = 0
      q(1)(2)(1)#268 = 0
      q(1)(2)(2)#269 = 0
      q(1)(3)(0)#270 = 0
      q(1)(3)(1)#271 = 0
      q(1)(3)(2)#272 = 0
 0 <= q(2)(0)(0)#273 <= 279.9
 0 <= q(2)(0)(1)#274 <= 279.9
 0 <= q(2)(0)(2)#275 <= 279.9
 0 <= q(2)(1)(0)#276 <= 279.9
 0 <= q(2)(1)(1)#277 <= 279.9
 0 <= q(2)(1)(2)#278 <= 279.9
      q(2)(2)(0)#279 = 0
      q(2)(2)(1)#280 = 0
      q(2)(2)(2)#281 = 0
      q(2)(3)(0)#282 = 0
      q(2)(3)(1)#283 = 0
      q(2)(3)(2)#284 = 0
      q(3)(0)(0)#285 = 0
      q(3)(0)(1)#286 = 0
      q(3)(0)(2)#287 = 0
      q(3)(1)(0)#288 = 0
      q(3)(1)(1)#289 = 0
      q(3)(1)(2)#290 = 0
 0 <= q(3)(2)(0)#291 <= 279.9
 0 <= q(3)(2)(1)#292 <= 279.9
 0 <= q(3)(2)(2)#293 <= 279.9
      q(3)(3)(0)#294 = 0
      q(3)(3)(1)#295 = 0
      q(3)(3)(2)#296 = 0
 0 <= q(4)(0)(0)#297 <= 279.9
 0 <= q(4)(0)(1)#298 <= 279.9
 0 <= q(4)(0)(2)#299 <= 279.9
      q(4)(1)(0)#300 = 0
      q(4)(1)(1)#301 = 0
      q(4)(1)(2)#302 = 0
 0 <= q(4)(2)(0)#303 <= 279.9
 0 <= q(4)(2)(1)#304 <= 279.9
 0 <= q(4)(2)(2)#305 <= 279.9
      q(4)(3)(0)#306 = 0
      q(4)(3)(1)#307 = 0
      q(4)(3)(2)#308 = 0
      q(5)(0)(0)#309 = 0
      q(5)(0)(1)#310 = 0
      q(5)(0)(2)#311 = 0
 0 <= q(5)(1)(0)#312 <= 279.9
 0 <= q(5)(1)(1)#313 <= 279.9
 0 <= q(5)(1)(2)#314 <= 279.9
 0 <= q(5)(2)(0)#315 <= 279.9
 0 <= q(5)(2)(1)#316 <= 279.9
 0 <= q(5)(2)(2)#317 <= 279.9
      q(5)(3)(0)#318 = 0
      q(5)(3)(1)#319 = 0
      q(5)(3)(2)#320 = 0
 0 <= q(6)(0)(0)#321 <= 279.9
 0 <= q(6)(0)(1)#322 <= 279.9
 0 <= q(6)(0)(2)#323 <= 279.9
 0 <= q(6)(1)(0)#324 <= 279.9
 0 <= q(6)(1)(1)#325 <= 279.9
 0 <= q(6)(1)(2)#326 <= 279.9
 0 <= q(6)(2)(0)#327 <= 279.9
 0 <= q(6)(2)(1)#328 <= 279.9
 0 <= q(6)(2)(2)#329 <= 279.9
      q(6)(3)(0)#330 = 0
      q(6)(3)(1)#331 = 0
      q(6)(3)(2)#332 = 0
      q(7)(0)(0)#333 = 0
      q(7)(0)(1)#334 = 0
      q(7)(0)(2)#335 = 0
      q(7)(1)(0)#336 = 0
      q(7)(1)(1)#337 = 0
      q(7)(1)(2)#338 = 0
      q(7)(2)(0)#339 = 0
      q(7)(2)(1)#340 = 0
      q(7)(2)(2)#341 = 0
 0 <= q(7)(3)(0)#342 <= 279.9
 0 <= q(7)(3)(1)#343 <= 279.9
 0 <= q(7)(3)(2)#344 <= 279.9
 0 <= q(8)(0)(0)#345 <= 279.9
 0 <= q(8)(0)(1)#346 <= 279.9
 0 <= q(8)(0)(2)#347 <= 279.9
      q(8)(1)(0)#348 = 0
      q(8)(1)(1)#349 = 0
      q(8)(1)(2)#350 = 0
      q(8)(2)(0)#351 = 0
      q(8)(2)(1)#352 = 0
      q(8)(2)(2)#353 = 0
 0 <= q(8)(3)(0)#354 <= 279.9
 0 <= q(8)(3)(1)#355 <= 279.9
 0 <= q(8)(3)(2)#356 <= 279.9
      q(9)(0)(0)#357 = 0
      q(9)(0)(1)#358 = 0
      q(9)(0)(2)#359 = 0
 0 <= q(9)(1)(0)#360 <= 279.9
 0 <= q(9)(1)(1)#361 <= 279.9
 0 <= q(9)(1)(2)#362 <= 279.9
      q(9)(2)(0)#363 = 0
      q(9)(2)(1)#364 = 0
      q(9)(2)(2)#365 = 0
 0 <= q(9)(3)(0)#366 <= 279.9
 0 <= q(9)(3)(1)#367 <= 279.9
 0 <= q(9)(3)(2)#368 <= 279.9
 0 <= q(10)(0)(0)#369 <= 279.9
 0 <= q(10)(0)(1)#370 <= 279.9
 0 <= q(10)(0)(2)#371 <= 279.9
 0 <= q(10)(1)(0)#372 <= 279.9
 0 <= q(10)(1)(1)#373 <= 279.9
 0 <= q(10)(1)(2)#374 <= 279.9
      q(10)(2)(0)#375 = 0
      q(10)(2)(1)#376 = 0
      q(10)(2)(2)#377 = 0
 0 <= q(10)(3)(0)#378 <= 279.9
 0 <= q(10)(3)(1)#379 <= 279.9
 0 <= q(10)(3)(2)#380 <= 279.9
      q(11)(0)(0)#381 = 0
      q(11)(0)(1)#382 = 0
      q(11)(0)(2)#383 = 0
      q(11)(1)(0)#384 = 0
      q(11)(1)(1)#385 = 0
      q(11)(1)(2)#386 = 0
 0 <= q(11)(2)(0)#387 <= 279.9
 0 <= q(11)(2)(1)#388 <= 279.9
 0 <= q(11)(2)(2)#389 <= 279.9
 0 <= q(11)(3)(0)#390 <= 279.9
 0 <= q(11)(3)(1)#391 <= 279.9
 0 <= q(11)(3)(2)#392 <= 279.9
 0 <= q(12)(0)(0)#393 <= 279.9
 0 <= q(12)(0)(1)#394 <= 279.9
 0 <= q(12)(0)(2)#395 <= 279.9
      q(12)(1)(0)#396 = 0
      q(12)(1)(1)#397 = 0
      q(12)(1)(2)#398 = 0
 0 <= q(12)(2)(0)#399 <= 279.9
 0 <= q(12)(2)(1)#400 <= 279.9
 0 <= q(12)(2)(2)#401 <= 279.9
 0 <= q(12)(3)(0)#402 <= 279.9
 0 <= q(12)(3)(1)#403 <= 279.9
 0 <= q(12)(3)(2)#404 <= 279.9
      q(13)(0)(0)#405 = 0
      q(13)(0)(1)#406 = 0
      q(13)(0)(2)#407 = 0
 0 <= q(13)(1)(0)#408 <= 279.9
 0 <= q(13)(1)(1)#409 <= 279.9
 0 <= q(13)(1)(2)#410 <= 279.9
 0 <= q(13)(2)(0)#411 <= 279.9
 0 <= q(13)(2)(1)#412 <= 279.9
 0 <= q(13)(2)(2)#413 <= 279.9
 0 <= q(13)(3)(0)#414 <= 279.9
 0 <= q(13)(3)(1)#415 <= 279.9
 0 <= q(13)(3)(2)#416 <= 279.9
 0 <= q(14)(0)(0)#417 <= 279.9
 0 <= q(14)(0)(1)#418 <= 279.9
 0 <= q(14)(0)(2)#419 <= 279.9
 0 <= q(14)(1)(0)#420 <= 279.9
 0 <= q(14)(1)(1)#421 <= 279.9
 0 <= q(14)(1)(2)#422 <= 279.9
 0 <= q(14)(2)(0)#423 <= 279.9
 0 <= q(14)(2)(1)#424 <= 279.9
 0 <= q(14)(2)(2)#425 <= 279.9
 0 <= q(14)(3)(0)#426 <= 279.9
 0 <= q(14)(3)(1)#427 <= 279.9
 0 <= q(14)(3)(2)#428 <= 279.9
 0 <= o(0)(0)#429 <= 1
 0 <= o(0)(1)#430 <= 1
 0 <= o(0)(2)#431 <= 1
 0 <= o(1)(0)#432 <= 1
 0 <= o(1)(1)#433 <= 1
 0 <= o(1)(2)#434 <= 1
 0 <= o(2)(0)#435 <= 1
 0 <= o(2)(1)#436 <= 1
 0 <= o(2)(2)#437 <= 1
 0 <= o(3)(0)#438 <= 1
 0 <= o(3)(1)#439 <= 1
 0 <= o(3)(2)#440 <= 1
 0 <= o(4)(0)#441 <= 1
 0 <= o(4)(1)#442 <= 1
 0 <= o(4)(2)#443 <= 1
 0 <= o(5)(0)#444 <= 1
 0 <= o(5)(1)#445 <= 1
 0 <= o(5)(2)#446 <= 1
 0 <= o(6)(0)#447 <= 1
 0 <= o(6)(1)#448 <= 1
 0 <= o(6)(2)#449 <= 1
 0 <= o(7)(0)#450 <= 1
 0 <= o(7)(1)#451 <= 1
 0 <= o(7)(2)#452 <= 1
 0 <= o(8)(0)#453 <= 1
 0 <= o(8)(1)#454 <= 1
 0 <= o(8)(2)#455 <= 1
 0 <= o(9)(0)#456 <= 1
 0 <= o(9)(1)#457 <= 1
 0 <= o(9)(2)#458 <= 1
 0 <= o(10)(0)#459 <= 1
 0 <= o(10)(1)#460 <= 1
 0 <= o(10)(2)#461 <= 1
 0 <= o(11)(0)#462 <= 1
 0 <= o(11)(1)#463 <= 1
 0 <= o(11)(2)#464 <= 1
 0 <= o(12)(0)#465 <= 1
 0 <= o(12)(1)#466 <= 1
 0 <= o(12)(2)#467 <= 1
 0 <= o(13)(0)#468 <= 1
 0 <= o(13)(1)#469 <= 1
 0 <= o(13)(2)#470 <= 1
 0 <= o(14)(0)#471 <= 1
 0 <= o(14)(1)#472 <= 1
 0 <= o(14)(2)#473 <= 1
Binaries
 y(0)#0  y(1)#1  y(2)#2  o(0)(0)#429  o(0)(1)#430  o(0)(2)#431  o(1)(0)#432 
 o(1)(1)#433  o(1)(2)#434  o(2)(0)#435  o(2)(1)#436  o(2)(2)#437  o(3)(0)#438 
 o(3)(1)#439  o(3)(2)#440  o(4)(0)#441  o(4)(1)#442  o(4)(2)#443  o(5)(0)#444 
 o(5)(1)#445  o(5)(2)#446  o(6)(0)#447  o(6)(1)#448  o(6)(2)#449  o(7)(0)#450 
 o(7)(1)#451  o(7)(2)#452  o(8)(0)#453  o(8)(1)#454  o(8)(2)#455  o(9)(0)#456 
 o(9)(1)#457  o(9)(2)#458  o(10)(0)#459  o(10)(1)#460  o(10)(2)#461 
 o(11)(0)#462  o(11)(1)#463  o(11)(2)#464  o(12)(0)#465  o(12)(1)#466 
 o(12)(2)#467  o(13)(0)#468  o(13)(1)#469  o(13)(2)#470  o(14)(0)#471 
 o(14)(1)#472  o(14)(2)#473 
End
