\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 30 p(1)#0 + 30 p(2)#1 + 30 p(3)#2 + 3 I_plant(1)#3 + 3 I_plant(2)#4
       + 3 I_plant(3)#5 + 0.4 I_warehouse(1)(1)(1)#6
       + 0.4 I_warehouse(1)(1)(2)#7 + 0.4 I_warehouse(1)(1)(3)#8
       + 0.4 I_warehouse(1)(1)(4)#9 + 0.4 I_warehouse(1)(1)(5)#10
       + 0.4 I_warehouse(1)(1)(6)#11 + 0.4 I_warehouse(1)(1)(7)#12
       + 0.4 I_warehouse(1)(1)(8)#13 + 0.4 I_warehouse(1)(1)(9)#14
       + 0.4 I_warehouse(1)(1)(10)#15 + 0.4 I_warehouse(1)(2)(1)#16
       + 0.4 I_warehouse(1)(2)(2)#17 + 0.4 I_warehouse(1)(2)(3)#18
       + 0.4 I_warehouse(1)(2)(4)#19 + 0.4 I_warehouse(1)(2)(5)#20
       + 0.4 I_warehouse(1)(2)(6)#21 + 0.4 I_warehouse(1)(2)(7)#22
       + 0.4 I_warehouse(1)(2)(8)#23 + 0.4 I_warehouse(1)(2)(9)#24
       + 0.4 I_warehouse(1)(2)(10)#25 + 0.4 I_warehouse(1)(3)(1)#26
       + 0.4 I_warehouse(1)(3)(2)#27 + 0.4 I_warehouse(1)(3)(3)#28
       + 0.4 I_warehouse(1)(3)(4)#29 + 0.4 I_warehouse(1)(3)(5)#30
       + 0.4 I_warehouse(1)(3)(6)#31 + 0.4 I_warehouse(1)(3)(7)#32
       + 0.4 I_warehouse(1)(3)(8)#33 + 0.4 I_warehouse(1)(3)(9)#34
       + 0.4 I_warehouse(1)(3)(10)#35 + 0.3 I_warehouse(2)(1)(1)#36
       + 0.3 I_warehouse(2)(1)(2)#37 + 0.3 I_warehouse(2)(1)(3)#38
       + 0.3 I_warehouse(2)(1)(4)#39 + 0.3 I_warehouse(2)(1)(5)#40
       + 0.3 I_warehouse(2)(1)(6)#41 + 0.3 I_warehouse(2)(1)(7)#42
       + 0.3 I_warehouse(2)(1)(8)#43 + 0.3 I_warehouse(2)(1)(9)#44
       + 0.3 I_warehouse(2)(1)(10)#45 + 0.3 I_warehouse(2)(2)(1)#46
       + 0.3 I_warehouse(2)(2)(2)#47 + 0.3 I_warehouse(2)(2)(3)#48
       + 0.3 I_warehouse(2)(2)(4)#49 + 0.3 I_warehouse(2)(2)(5)#50
       + 0.3 I_warehouse(2)(2)(6)#51 + 0.3 I_warehouse(2)(2)(7)#52
       + 0.3 I_warehouse(2)(2)(8)#53 + 0.3 I_warehouse(2)(2)(9)#54
       + 0.3 I_warehouse(2)(2)(10)#55 + 0.3 I_warehouse(2)(3)(1)#56
       + 0.3 I_warehouse(2)(3)(2)#57 + 0.3 I_warehouse(2)(3)(3)#58
       + 0.3 I_warehouse(2)(3)(4)#59 + 0.3 I_warehouse(2)(3)(5)#60
       + 0.3 I_warehouse(2)(3)(6)#61 + 0.3 I_warehouse(2)(3)(7)#62
       + 0.3 I_warehouse(2)(3)(8)#63 + 0.3 I_warehouse(2)(3)(9)#64
       + 0.3 I_warehouse(2)(3)(10)#65 + 0.6 I_customer(3)(1)(1)#66
       + 0.6 I_customer(3)(1)(2)#67 + 0.6 I_customer(3)(1)(3)#68
       + 0.6 I_customer(3)(1)(4)#69 + 0.6 I_customer(3)(1)(5)#70
       + 0.6 I_customer(3)(1)(6)#71 + 0.6 I_customer(3)(1)(7)#72
       + 0.6 I_customer(3)(1)(8)#73 + 0.6 I_customer(3)(1)(9)#74
       + 0.6 I_customer(3)(1)(10)#75 + 0.6 I_customer(3)(2)(1)#76
       + 0.6 I_customer(3)(2)(2)#77 + 0.6 I_customer(3)(2)(3)#78
       + 0.6 I_customer(3)(2)(4)#79 + 0.6 I_customer(3)(2)(5)#80
       + 0.6 I_customer(3)(2)(6)#81 + 0.6 I_customer(3)(2)(7)#82
       + 0.6 I_customer(3)(2)(8)#83 + 0.6 I_customer(3)(2)(9)#84
       + 0.6 I_customer(3)(2)(10)#85 + 0.6 I_customer(3)(3)(1)#86
       + 0.6 I_customer(3)(3)(2)#87 + 0.6 I_customer(3)(3)(3)#88
       + 0.6 I_customer(3)(3)(4)#89 + 0.6 I_customer(3)(3)(5)#90
       + 0.6 I_customer(3)(3)(6)#91 + 0.6 I_customer(3)(3)(7)#92
       + 0.6 I_customer(3)(3)(8)#93 + 0.6 I_customer(3)(3)(9)#94
       + 0.6 I_customer(3)(3)(10)#95 + 0.9 I_customer(4)(1)(1)#96
       + 0.9 I_customer(4)(1)(2)#97 + 0.9 I_customer(4)(1)(3)#98
       + 0.9 I_customer(4)(1)(4)#99 + 0.9 I_customer(4)(1)(5)#100
       + 0.9 I_customer(4)(1)(6)#101 + 0.9 I_customer(4)(1)(7)#102
       + 0.9 I_customer(4)(1)(8)#103 + 0.9 I_customer(4)(1)(9)#104
       + 0.9 I_customer(4)(1)(10)#105 + 0.9 I_customer(4)(2)(1)#106
       + 0.9 I_customer(4)(2)(2)#107 + 0.9 I_customer(4)(2)(3)#108
       + 0.9 I_customer(4)(2)(4)#109 + 0.9 I_customer(4)(2)(5)#110
       + 0.9 I_customer(4)(2)(6)#111 + 0.9 I_customer(4)(2)(7)#112
       + 0.9 I_customer(4)(2)(8)#113 + 0.9 I_customer(4)(2)(9)#114
       + 0.9 I_customer(4)(2)(10)#115 + 0.9 I_customer(4)(3)(1)#116
       + 0.9 I_customer(4)(3)(2)#117 + 0.9 I_customer(4)(3)(3)#118
       + 0.9 I_customer(4)(3)(4)#119 + 0.9 I_customer(4)(3)(5)#120
       + 0.9 I_customer(4)(3)(6)#121 + 0.9 I_customer(4)(3)(7)#122
       + 0.9 I_customer(4)(3)(8)#123 + 0.9 I_customer(4)(3)(9)#124
       + 0.9 I_customer(4)(3)(10)#125 + 0.7 I_customer(5)(1)(1)#126
       + 0.7 I_customer(5)(1)(2)#127 + 0.7 I_customer(5)(1)(3)#128
       + 0.7 I_customer(5)(1)(4)#129 + 0.7 I_customer(5)(1)(5)#130
       + 0.7 I_customer(5)(1)(6)#131 + 0.7 I_customer(5)(1)(7)#132
       + 0.7 I_customer(5)(1)(8)#133 + 0.7 I_customer(5)(1)(9)#134
       + 0.7 I_customer(5)(1)(10)#135 + 0.7 I_customer(5)(2)(1)#136
       + 0.7 I_customer(5)(2)(2)#137 + 0.7 I_customer(5)(2)(3)#138
       + 0.7 I_customer(5)(2)(4)#139 + 0.7 I_customer(5)(2)(5)#140
       + 0.7 I_customer(5)(2)(6)#141 + 0.7 I_customer(5)(2)(7)#142
       + 0.7 I_customer(5)(2)(8)#143 + 0.7 I_customer(5)(2)(9)#144
       + 0.7 I_customer(5)(2)(10)#145 + 0.7 I_customer(5)(3)(1)#146
       + 0.7 I_customer(5)(3)(2)#147 + 0.7 I_customer(5)(3)(3)#148
       + 0.7 I_customer(5)(3)(4)#149 + 0.7 I_customer(5)(3)(5)#150
       + 0.7 I_customer(5)(3)(6)#151 + 0.7 I_customer(5)(3)(7)#152
       + 0.7 I_customer(5)(3)(8)#153 + 0.7 I_customer(5)(3)(9)#154
       + 0.7 I_customer(5)(3)(10)#155 + 0.8 I_customer(6)(1)(1)#156
       + 0.8 I_customer(6)(1)(2)#157 + 0.8 I_customer(6)(1)(3)#158
       + 0.8 I_customer(6)(1)(4)#159 + 0.8 I_customer(6)(1)(5)#160
       + 0.8 I_customer(6)(1)(6)#161 + 0.8 I_customer(6)(1)(7)#162
       + 0.8 I_customer(6)(1)(8)#163 + 0.8 I_customer(6)(1)(9)#164
       + 0.8 I_customer(6)(1)(10)#165 + 0.8 I_customer(6)(2)(1)#166
       + 0.8 I_customer(6)(2)(2)#167 + 0.8 I_customer(6)(2)(3)#168
       + 0.8 I_customer(6)(2)(4)#169 + 0.8 I_customer(6)(2)(5)#170
       + 0.8 I_customer(6)(2)(6)#171 + 0.8 I_customer(6)(2)(7)#172
       + 0.8 I_customer(6)(2)(8)#173 + 0.8 I_customer(6)(2)(9)#174
       + 0.8 I_customer(6)(2)(10)#175 + 0.8 I_customer(6)(3)(1)#176
       + 0.8 I_customer(6)(3)(2)#177 + 0.8 I_customer(6)(3)(3)#178
       + 0.8 I_customer(6)(3)(4)#179 + 0.8 I_customer(6)(3)(5)#180
       + 0.8 I_customer(6)(3)(6)#181 + 0.8 I_customer(6)(3)(7)#182
       + 0.8 I_customer(6)(3)(8)#183 + 0.8 I_customer(6)(3)(9)#184
       + 0.8 I_customer(6)(3)(10)#185 + 0.6 I_customer(7)(1)(1)#186
       + 0.6 I_customer(7)(1)(2)#187 + 0.6 I_customer(7)(1)(3)#188
       + 0.6 I_customer(7)(1)(4)#189 + 0.6 I_customer(7)(1)(5)#190
       + 0.6 I_customer(7)(1)(6)#191 + 0.6 I_customer(7)(1)(7)#192
       + 0.6 I_customer(7)(1)(8)#193 + 0.6 I_customer(7)(1)(9)#194
       + 0.6 I_customer(7)(1)(10)#195 + 0.6 I_customer(7)(2)(1)#196
       + 0.6 I_customer(7)(2)(2)#197 + 0.6 I_customer(7)(2)(3)#198
       + 0.6 I_customer(7)(2)(4)#199 + 0.6 I_customer(7)(2)(5)#200
       + 0.6 I_customer(7)(2)(6)#201 + 0.6 I_customer(7)(2)(7)#202
       + 0.6 I_customer(7)(2)(8)#203 + 0.6 I_customer(7)(2)(9)#204
       + 0.6 I_customer(7)(2)(10)#205 + 0.6 I_customer(7)(3)(1)#206
       + 0.6 I_customer(7)(3)(2)#207 + 0.6 I_customer(7)(3)(3)#208
       + 0.6 I_customer(7)(3)(4)#209 + 0.6 I_customer(7)(3)(5)#210
       + 0.6 I_customer(7)(3)(6)#211 + 0.6 I_customer(7)(3)(7)#212
       + 0.6 I_customer(7)(3)(8)#213 + 0.6 I_customer(7)(3)(9)#214
       + 0.6 I_customer(7)(3)(10)#215 + 0.6 I_customer(8)(1)(1)#216
       + 0.6 I_customer(8)(1)(2)#217 + 0.6 I_customer(8)(1)(3)#218
       + 0.6 I_customer(8)(1)(4)#219 + 0.6 I_customer(8)(1)(5)#220
       + 0.6 I_customer(8)(1)(6)#221 + 0.6 I_customer(8)(1)(7)#222
       + 0.6 I_customer(8)(1)(8)#223 + 0.6 I_customer(8)(1)(9)#224
       + 0.6 I_customer(8)(1)(10)#225 + 0.6 I_customer(8)(2)(1)#226
       + 0.6 I_customer(8)(2)(2)#227 + 0.6 I_customer(8)(2)(3)#228
       + 0.6 I_customer(8)(2)(4)#229 + 0.6 I_customer(8)(2)(5)#230
       + 0.6 I_customer(8)(2)(6)#231 + 0.6 I_customer(8)(2)(7)#232
       + 0.6 I_customer(8)(2)(8)#233 + 0.6 I_customer(8)(2)(9)#234
       + 0.6 I_customer(8)(2)(10)#235 + 0.6 I_customer(8)(3)(1)#236
       + 0.6 I_customer(8)(3)(2)#237 + 0.6 I_customer(8)(3)(3)#238
       + 0.6 I_customer(8)(3)(4)#239 + 0.6 I_customer(8)(3)(5)#240
       + 0.6 I_customer(8)(3)(6)#241 + 0.6 I_customer(8)(3)(7)#242
       + 0.6 I_customer(8)(3)(8)#243 + 0.6 I_customer(8)(3)(9)#244
       + 0.6 I_customer(8)(3)(10)#245 + 0.8 I_customer(9)(1)(1)#246
       + 0.8 I_customer(9)(1)(2)#247 + 0.8 I_customer(9)(1)(3)#248
       + 0.8 I_customer(9)(1)(4)#249 + 0.8 I_customer(9)(1)(5)#250
       + 0.8 I_customer(9)(1)(6)#251 + 0.8 I_customer(9)(1)(7)#252
       + 0.8 I_customer(9)(1)(8)#253 + 0.8 I_customer(9)(1)(9)#254
       + 0.8 I_customer(9)(1)(10)#255 + 0.8 I_customer(9)(2)(1)#256
       + 0.8 I_customer(9)(2)(2)#257 + 0.8 I_customer(9)(2)(3)#258
       + 0.8 I_customer(9)(2)(4)#259 + 0.8 I_customer(9)(2)(5)#260
       + 0.8 I_customer(9)(2)(6)#261 + 0.8 I_customer(9)(2)(7)#262
       + 0.8 I_customer(9)(2)(8)#263 + 0.8 I_customer(9)(2)(9)#264
       + 0.8 I_customer(9)(2)(10)#265 + 0.8 I_customer(9)(3)(1)#266
       + 0.8 I_customer(9)(3)(2)#267 + 0.8 I_customer(9)(3)(3)#268
       + 0.8 I_customer(9)(3)(4)#269 + 0.8 I_customer(9)(3)(5)#270
       + 0.8 I_customer(9)(3)(6)#271 + 0.8 I_customer(9)(3)(7)#272
       + 0.8 I_customer(9)(3)(8)#273 + 0.8 I_customer(9)(3)(9)#274
       + 0.8 I_customer(9)(3)(10)#275 + 0.8 I_customer(10)(1)(1)#276
       + 0.8 I_customer(10)(1)(2)#277 + 0.8 I_customer(10)(1)(3)#278
       + 0.8 I_customer(10)(1)(4)#279 + 0.8 I_customer(10)(1)(5)#280
       + 0.8 I_customer(10)(1)(6)#281 + 0.8 I_customer(10)(1)(7)#282
       + 0.8 I_customer(10)(1)(8)#283 + 0.8 I_customer(10)(1)(9)#284
       + 0.8 I_customer(10)(1)(10)#285 + 0.8 I_customer(10)(2)(1)#286
       + 0.8 I_customer(10)(2)(2)#287 + 0.8 I_customer(10)(2)(3)#288
       + 0.8 I_customer(10)(2)(4)#289 + 0.8 I_customer(10)(2)(5)#290
       + 0.8 I_customer(10)(2)(6)#291 + 0.8 I_customer(10)(2)(7)#292
       + 0.8 I_customer(10)(2)(8)#293 + 0.8 I_customer(10)(2)(9)#294
       + 0.8 I_customer(10)(2)(10)#295 + 0.8 I_customer(10)(3)(1)#296
       + 0.8 I_customer(10)(3)(2)#297 + 0.8 I_customer(10)(3)(3)#298
       + 0.8 I_customer(10)(3)(4)#299 + 0.8 I_customer(10)(3)(5)#300
       + 0.8 I_customer(10)(3)(6)#301 + 0.8 I_customer(10)(3)(7)#302
       + 0.8 I_customer(10)(3)(8)#303 + 0.8 I_customer(10)(3)(9)#304
       + 0.8 I_customer(10)(3)(10)#305 + 0.9 I_customer(11)(1)(1)#306
       + 0.9 I_customer(11)(1)(2)#307 + 0.9 I_customer(11)(1)(3)#308
       + 0.9 I_customer(11)(1)(4)#309 + 0.9 I_customer(11)(1)(5)#310
       + 0.9 I_customer(11)(1)(6)#311 + 0.9 I_customer(11)(1)(7)#312
       + 0.9 I_customer(11)(1)(8)#313 + 0.9 I_customer(11)(1)(9)#314
       + 0.9 I_customer(11)(1)(10)#315 + 0.9 I_customer(11)(2)(1)#316
       + 0.9 I_customer(11)(2)(2)#317 + 0.9 I_customer(11)(2)(3)#318
       + 0.9 I_customer(11)(2)(4)#319 + 0.9 I_customer(11)(2)(5)#320
       + 0.9 I_customer(11)(2)(6)#321 + 0.9 I_customer(11)(2)(7)#322
       + 0.9 I_customer(11)(2)(8)#323 + 0.9 I_customer(11)(2)(9)#324
       + 0.9 I_customer(11)(2)(10)#325 + 0.9 I_customer(11)(3)(1)#326
       + 0.9 I_customer(11)(3)(2)#327 + 0.9 I_customer(11)(3)(3)#328
       + 0.9 I_customer(11)(3)(4)#329 + 0.9 I_customer(11)(3)(5)#330
       + 0.9 I_customer(11)(3)(6)#331 + 0.9 I_customer(11)(3)(7)#332
       + 0.9 I_customer(11)(3)(8)#333 + 0.9 I_customer(11)(3)(9)#334
       + 0.9 I_customer(11)(3)(10)#335 + 0.7 I_customer(12)(1)(1)#336
       + 0.7 I_customer(12)(1)(2)#337 + 0.7 I_customer(12)(1)(3)#338
       + 0.7 I_customer(12)(1)(4)#339 + 0.7 I_customer(12)(1)(5)#340
       + 0.7 I_customer(12)(1)(6)#341 + 0.7 I_customer(12)(1)(7)#342
       + 0.7 I_customer(12)(1)(8)#343 + 0.7 I_customer(12)(1)(9)#344
       + 0.7 I_customer(12)(1)(10)#345 + 0.7 I_customer(12)(2)(1)#346
       + 0.7 I_customer(12)(2)(2)#347 + 0.7 I_customer(12)(2)(3)#348
       + 0.7 I_customer(12)(2)(4)#349 + 0.7 I_customer(12)(2)(5)#350
       + 0.7 I_customer(12)(2)(6)#351 + 0.7 I_customer(12)(2)(7)#352
       + 0.7 I_customer(12)(2)(8)#353 + 0.7 I_customer(12)(2)(9)#354
       + 0.7 I_customer(12)(2)(10)#355 + 0.7 I_customer(12)(3)(1)#356
       + 0.7 I_customer(12)(3)(2)#357 + 0.7 I_customer(12)(3)(3)#358
       + 0.7 I_customer(12)(3)(4)#359 + 0.7 I_customer(12)(3)(5)#360
       + 0.7 I_customer(12)(3)(6)#361 + 0.7 I_customer(12)(3)(7)#362
       + 0.7 I_customer(12)(3)(8)#363 + 0.7 I_customer(12)(3)(9)#364
       + 0.7 I_customer(12)(3)(10)#365 + 104 b_customer(3)(1)(1)#366
       + 104 b_customer(3)(1)(2)#367 + 104 b_customer(3)(1)(3)#368
       + 104 b_customer(3)(1)(4)#369 + 104 b_customer(3)(1)(5)#370
       + 104 b_customer(3)(1)(6)#371 + 104 b_customer(3)(1)(7)#372
       + 104 b_customer(3)(1)(8)#373 + 104 b_customer(3)(1)(9)#374
       + 104 b_customer(3)(1)(10)#375 + 104 b_customer(3)(2)(1)#376
       + 104 b_customer(3)(2)(2)#377 + 104 b_customer(3)(2)(3)#378
       + 104 b_customer(3)(2)(4)#379 + 104 b_customer(3)(2)(5)#380
       + 104 b_customer(3)(2)(6)#381 + 104 b_customer(3)(2)(7)#382
       + 104 b_customer(3)(2)(8)#383 + 104 b_customer(3)(2)(9)#384
       + 104 b_customer(3)(2)(10)#385 + 104 b_customer(3)(3)(1)#386
       + 104 b_customer(3)(3)(2)#387 + 104 b_customer(3)(3)(3)#388
       + 104 b_customer(3)(3)(4)#389 + 104 b_customer(3)(3)(5)#390
       + 104 b_customer(3)(3)(6)#391 + 104 b_customer(3)(3)(7)#392
       + 104 b_customer(3)(3)(8)#393 + 104 b_customer(3)(3)(9)#394
       + 104 b_customer(3)(3)(10)#395 + 75 b_customer(4)(1)(1)#396
       + 75 b_customer(4)(1)(2)#397 + 75 b_customer(4)(1)(3)#398
       + 75 b_customer(4)(1)(4)#399 + 75 b_customer(4)(1)(5)#400
       + 75 b_customer(4)(1)(6)#401 + 75 b_customer(4)(1)(7)#402
       + 75 b_customer(4)(1)(8)#403 + 75 b_customer(4)(1)(9)#404
       + 75 b_customer(4)(1)(10)#405 + 75 b_customer(4)(2)(1)#406
       + 75 b_customer(4)(2)(2)#407 + 75 b_customer(4)(2)(3)#408
       + 75 b_customer(4)(2)(4)#409 + 75 b_customer(4)(2)(5)#410
       + 75 b_customer(4)(2)(6)#411 + 75 b_customer(4)(2)(7)#412
       + 75 b_customer(4)(2)(8)#413 + 75 b_customer(4)(2)(9)#414
       + 75 b_customer(4)(2)(10)#415 + 75 b_customer(4)(3)(1)#416
       + 75 b_customer(4)(3)(2)#417 + 75 b_customer(4)(3)(3)#418
       + 75 b_customer(4)(3)(4)#419 + 75 b_customer(4)(3)(5)#420
       + 75 b_customer(4)(3)(6)#421 + 75 b_customer(4)(3)(7)#422
       + 75 b_customer(4)(3)(8)#423 + 75 b_customer(4)(3)(9)#424
       + 75 b_customer(4)(3)(10)#425 + 71 b_customer(5)(1)(1)#426
       + 71 b_customer(5)(1)(2)#427 + 71 b_customer(5)(1)(3)#428
       + 71 b_customer(5)(1)(4)#429 + 71 b_customer(5)(1)(5)#430
       + 71 b_customer(5)(1)(6)#431 + 71 b_customer(5)(1)(7)#432
       + 71 b_customer(5)(1)(8)#433 + 71 b_customer(5)(1)(9)#434
       + 71 b_customer(5)(1)(10)#435 + 71 b_customer(5)(2)(1)#436
       + 71 b_customer(5)(2)(2)#437 + 71 b_customer(5)(2)(3)#438
       + 71 b_customer(5)(2)(4)#439 + 71 b_customer(5)(2)(5)#440
       + 71 b_customer(5)(2)(6)#441 + 71 b_customer(5)(2)(7)#442
       + 71 b_customer(5)(2)(8)#443 + 71 b_customer(5)(2)(9)#444
       + 71 b_customer(5)(2)(10)#445 + 71 b_customer(5)(3)(1)#446
       + 71 b_customer(5)(3)(2)#447 + 71 b_customer(5)(3)(3)#448
       + 71 b_customer(5)(3)(4)#449 + 71 b_customer(5)(3)(5)#450
       + 71 b_customer(5)(3)(6)#451 + 71 b_customer(5)(3)(7)#452
       + 71 b_customer(5)(3)(8)#453 + 71 b_customer(5)(3)(9)#454
       + 71 b_customer(5)(3)(10)#455 + 130 b_customer(6)(1)(1)#456
       + 130 b_customer(6)(1)(2)#457 + 130 b_customer(6)(1)(3)#458
       + 130 b_customer(6)(1)(4)#459 + 130 b_customer(6)(1)(5)#460
       + 130 b_customer(6)(1)(6)#461 + 130 b_customer(6)(1)(7)#462
       + 130 b_customer(6)(1)(8)#463 + 130 b_customer(6)(1)(9)#464
       + 130 b_customer(6)(1)(10)#465 + 130 b_customer(6)(2)(1)#466
       + 130 b_customer(6)(2)(2)#467 + 130 b_customer(6)(2)(3)#468
       + 130 b_customer(6)(2)(4)#469 + 130 b_customer(6)(2)(5)#470
       + 130 b_customer(6)(2)(6)#471 + 130 b_customer(6)(2)(7)#472
       + 130 b_customer(6)(2)(8)#473 + 130 b_customer(6)(2)(9)#474
       + 130 b_customer(6)(2)(10)#475 + 130 b_customer(6)(3)(1)#476
       + 130 b_customer(6)(3)(2)#477 + 130 b_customer(6)(3)(3)#478
       + 130 b_customer(6)(3)(4)#479 + 130 b_customer(6)(3)(5)#480
       + 130 b_customer(6)(3)(6)#481 + 130 b_customer(6)(3)(7)#482
       + 130 b_customer(6)(3)(8)#483 + 130 b_customer(6)(3)(9)#484
       + 130 b_customer(6)(3)(10)#485 + 67 b_customer(7)(1)(1)#486
       + 67 b_customer(7)(1)(2)#487 + 67 b_customer(7)(1)(3)#488
       + 67 b_customer(7)(1)(4)#489 + 67 b_customer(7)(1)(5)#490
       + 67 b_customer(7)(1)(6)#491 + 67 b_customer(7)(1)(7)#492
       + 67 b_customer(7)(1)(8)#493 + 67 b_customer(7)(1)(9)#494
       + 67 b_customer(7)(1)(10)#495 + 67 b_customer(7)(2)(1)#496
       + 67 b_customer(7)(2)(2)#497 + 67 b_customer(7)(2)(3)#498
       + 67 b_customer(7)(2)(4)#499 + 67 b_customer(7)(2)(5)#500
       + 67 b_customer(7)(2)(6)#501 + 67 b_customer(7)(2)(7)#502
       + 67 b_customer(7)(2)(8)#503 + 67 b_customer(7)(2)(9)#504
       + 67 b_customer(7)(2)(10)#505 + 67 b_customer(7)(3)(1)#506
       + 67 b_customer(7)(3)(2)#507 + 67 b_customer(7)(3)(3)#508
       + 67 b_customer(7)(3)(4)#509 + 67 b_customer(7)(3)(5)#510
       + 67 b_customer(7)(3)(6)#511 + 67 b_customer(7)(3)(7)#512
       + 67 b_customer(7)(3)(8)#513 + 67 b_customer(7)(3)(9)#514
       + 67 b_customer(7)(3)(10)#515 + 117 b_customer(8)(1)(1)#516
       + 117 b_customer(8)(1)(2)#517 + 117 b_customer(8)(1)(3)#518
       + 117 b_customer(8)(1)(4)#519 + 117 b_customer(8)(1)(5)#520
       + 117 b_customer(8)(1)(6)#521 + 117 b_customer(8)(1)(7)#522
       + 117 b_customer(8)(1)(8)#523 + 117 b_customer(8)(1)(9)#524
       + 117 b_customer(8)(1)(10)#525 + 117 b_customer(8)(2)(1)#526
       + 117 b_customer(8)(2)(2)#527 + 117 b_customer(8)(2)(3)#528
       + 117 b_customer(8)(2)(4)#529 + 117 b_customer(8)(2)(5)#530
       + 117 b_customer(8)(2)(6)#531 + 117 b_customer(8)(2)(7)#532
       + 117 b_customer(8)(2)(8)#533 + 117 b_customer(8)(2)(9)#534
       + 117 b_customer(8)(2)(10)#535 + 117 b_customer(8)(3)(1)#536
       + 117 b_customer(8)(3)(2)#537 + 117 b_customer(8)(3)(3)#538
       + 117 b_customer(8)(3)(4)#539 + 117 b_customer(8)(3)(5)#540
       + 117 b_customer(8)(3)(6)#541 + 117 b_customer(8)(3)(7)#542
       + 117 b_customer(8)(3)(8)#543 + 117 b_customer(8)(3)(9)#544
       + 117 b_customer(8)(3)(10)#545 + 84 b_customer(9)(1)(1)#546
       + 84 b_customer(9)(1)(2)#547 + 84 b_customer(9)(1)(3)#548
       + 84 b_customer(9)(1)(4)#549 + 84 b_customer(9)(1)(5)#550
       + 84 b_customer(9)(1)(6)#551 + 84 b_customer(9)(1)(7)#552
       + 84 b_customer(9)(1)(8)#553 + 84 b_customer(9)(1)(9)#554
       + 84 b_customer(9)(1)(10)#555 + 84 b_customer(9)(2)(1)#556
       + 84 b_customer(9)(2)(2)#557 + 84 b_customer(9)(2)(3)#558
       + 84 b_customer(9)(2)(4)#559 + 84 b_customer(9)(2)(5)#560
       + 84 b_customer(9)(2)(6)#561 + 84 b_customer(9)(2)(7)#562
       + 84 b_customer(9)(2)(8)#563 + 84 b_customer(9)(2)(9)#564
       + 84 b_customer(9)(2)(10)#565 + 84 b_customer(9)(3)(1)#566
       + 84 b_customer(9)(3)(2)#567 + 84 b_customer(9)(3)(3)#568
       + 84 b_customer(9)(3)(4)#569 + 84 b_customer(9)(3)(5)#570
       + 84 b_customer(9)(3)(6)#571 + 84 b_customer(9)(3)(7)#572
       + 84 b_customer(9)(3)(8)#573 + 84 b_customer(9)(3)(9)#574
       + 84 b_customer(9)(3)(10)#575 + 124 b_customer(10)(1)(1)#576
       + 124 b_customer(10)(1)(2)#577 + 124 b_customer(10)(1)(3)#578
       + 124 b_customer(10)(1)(4)#579 + 124 b_customer(10)(1)(5)#580
       + 124 b_customer(10)(1)(6)#581 + 124 b_customer(10)(1)(7)#582
       + 124 b_customer(10)(1)(8)#583 + 124 b_customer(10)(1)(9)#584
       + 124 b_customer(10)(1)(10)#585 + 124 b_customer(10)(2)(1)#586
       + 124 b_customer(10)(2)(2)#587 + 124 b_customer(10)(2)(3)#588
       + 124 b_customer(10)(2)(4)#589 + 124 b_customer(10)(2)(5)#590
       + 124 b_customer(10)(2)(6)#591 + 124 b_customer(10)(2)(7)#592
       + 124 b_customer(10)(2)(8)#593 + 124 b_customer(10)(2)(9)#594
       + 124 b_customer(10)(2)(10)#595 + 124 b_customer(10)(3)(1)#596
       + 124 b_customer(10)(3)(2)#597 + 124 b_customer(10)(3)(3)#598
       + 124 b_customer(10)(3)(4)#599 + 124 b_customer(10)(3)(5)#600
       + 124 b_customer(10)(3)(6)#601 + 124 b_customer(10)(3)(7)#602
       + 124 b_customer(10)(3)(8)#603 + 124 b_customer(10)(3)(9)#604
       + 124 b_customer(10)(3)(10)#605 + 101 b_customer(11)(1)(1)#606
       + 101 b_customer(11)(1)(2)#607 + 101 b_customer(11)(1)(3)#608
       + 101 b_customer(11)(1)(4)#609 + 101 b_customer(11)(1)(5)#610
       + 101 b_customer(11)(1)(6)#611 + 101 b_customer(11)(1)(7)#612
       + 101 b_customer(11)(1)(8)#613 + 101 b_customer(11)(1)(9)#614
       + 101 b_customer(11)(1)(10)#615 + 101 b_customer(11)(2)(1)#616
       + 101 b_customer(11)(2)(2)#617 + 101 b_customer(11)(2)(3)#618
       + 101 b_customer(11)(2)(4)#619 + 101 b_customer(11)(2)(5)#620
       + 101 b_customer(11)(2)(6)#621 + 101 b_customer(11)(2)(7)#622
       + 101 b_customer(11)(2)(8)#623 + 101 b_customer(11)(2)(9)#624
       + 101 b_customer(11)(2)(10)#625 + 101 b_customer(11)(3)(1)#626
       + 101 b_customer(11)(3)(2)#627 + 101 b_customer(11)(3)(3)#628
       + 101 b_customer(11)(3)(4)#629 + 101 b_customer(11)(3)(5)#630
       + 101 b_customer(11)(3)(6)#631 + 101 b_customer(11)(3)(7)#632
       + 101 b_customer(11)(3)(8)#633 + 101 b_customer(11)(3)(9)#634
       + 101 b_customer(11)(3)(10)#635 + 76 b_customer(12)(1)(1)#636
       + 76 b_customer(12)(1)(2)#637 + 76 b_customer(12)(1)(3)#638
       + 76 b_customer(12)(1)(4)#639 + 76 b_customer(12)(1)(5)#640
       + 76 b_customer(12)(1)(6)#641 + 76 b_customer(12)(1)(7)#642
       + 76 b_customer(12)(1)(8)#643 + 76 b_customer(12)(1)(9)#644
       + 76 b_customer(12)(1)(10)#645 + 76 b_customer(12)(2)(1)#646
       + 76 b_customer(12)(2)(2)#647 + 76 b_customer(12)(2)(3)#648
       + 76 b_customer(12)(2)(4)#649 + 76 b_customer(12)(2)(5)#650
       + 76 b_customer(12)(2)(6)#651 + 76 b_customer(12)(2)(7)#652
       + 76 b_customer(12)(2)(8)#653 + 76 b_customer(12)(2)(9)#654
       + 76 b_customer(12)(2)(10)#655 + 76 b_customer(12)(3)(1)#656
       + 76 b_customer(12)(3)(2)#657 + 76 b_customer(12)(3)(3)#658
       + 76 b_customer(12)(3)(4)#659 + 76 b_customer(12)(3)(5)#660
       + 76 b_customer(12)(3)(6)#661 + 76 b_customer(12)(3)(7)#662
       + 76 b_customer(12)(3)(8)#663 + 76 b_customer(12)(3)(9)#664
       + 76 b_customer(12)(3)(10)#665
Subject To
 ProductionCapacity(1):          p(1)#0 <= 304
 ProductionCapacity(2):          p(2)#1 <= 0
 ProductionCapacity(3):          p(3)#2 <= 0
 PlantInventoryCapacity(1):      I_plant(1)#3 <= 152
 PlantInventoryCapacity(2):      I_plant(2)#4 <= 152
 PlantInventoryCapacity(3):      I_plant(3)#5 <= 152
 PlantInventoryBalance(1):       - p(1)#0 + I_plant(1)#3 + q(1)(1)(1)#666
                                 + q(1)(2)(1)#669 + q(2)(1)(1)#672
                                 + q(2)(2)(1)#675  = 0
 PlantInventoryBalance(2):       - p(2)#1 - I_plant(1)#3 + I_plant(2)#4
                                 + q(1)(1)(2)#667 + q(1)(2)(2)#670
                                 + q(2)(1)(2)#673 + q(2)(2)(2)#676  = 0
 PlantInventoryBalance(3):       - p(3)#2 - I_plant(2)#4 + I_plant(3)#5
                                 + q(1)(1)(3)#668 + q(1)(2)(3)#671
                                 + q(2)(1)(3)#674 + q(2)(2)(3)#677  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(1)(1)(1)#6 <= 136.8
 WarehouseInventoryCapacity(2,1,1): I_warehouse(2)(1)(1)#36 <= 136.8
 WarehouseInventoryCapacity(1,2,1): I_warehouse(1)(2)(1)#16 <= 136.8
 WarehouseInventoryCapacity(2,2,1): I_warehouse(2)(2)(1)#46 <= 136.8
 WarehouseInventoryCapacity(1,3,1): I_warehouse(1)(3)(1)#26 <= 136.8
 WarehouseInventoryCapacity(2,3,1): I_warehouse(2)(3)(1)#56 <= 136.8
 WarehouseInventoryCapacity(1,1,2): I_warehouse(1)(1)(2)#7 <= 136.8
 WarehouseInventoryCapacity(2,1,2): I_warehouse(2)(1)(2)#37 <= 136.8
 WarehouseInventoryCapacity(1,2,2): I_warehouse(1)(2)(2)#17 <= 136.8
 WarehouseInventoryCapacity(2,2,2): I_warehouse(2)(2)(2)#47 <= 136.8
 WarehouseInventoryCapacity(1,3,2): I_warehouse(1)(3)(2)#27 <= 136.8
 WarehouseInventoryCapacity(2,3,2): I_warehouse(2)(3)(2)#57 <= 136.8
 WarehouseInventoryCapacity(1,1,3): I_warehouse(1)(1)(3)#8 <= 136.8
 WarehouseInventoryCapacity(2,1,3): I_warehouse(2)(1)(3)#38 <= 136.8
 WarehouseInventoryCapacity(1,2,3): I_warehouse(1)(2)(3)#18 <= 136.8
 WarehouseInventoryCapacity(2,2,3): I_warehouse(2)(2)(3)#48 <= 136.8
 WarehouseInventoryCapacity(1,3,3): I_warehouse(1)(3)(3)#28 <= 136.8
 WarehouseInventoryCapacity(2,3,3): I_warehouse(2)(3)(3)#58 <= 136.8
 WarehouseInventoryCapacity(1,1,4): I_warehouse(1)(1)(4)#9 <= 136.8
 WarehouseInventoryCapacity(2,1,4): I_warehouse(2)(1)(4)#39 <= 136.8
 WarehouseInventoryCapacity(1,2,4): I_warehouse(1)(2)(4)#19 <= 136.8
 WarehouseInventoryCapacity(2,2,4): I_warehouse(2)(2)(4)#49 <= 136.8
 WarehouseInventoryCapacity(1,3,4): I_warehouse(1)(3)(4)#29 <= 136.8
 WarehouseInventoryCapacity(2,3,4): I_warehouse(2)(3)(4)#59 <= 136.8
 WarehouseInventoryCapacity(1,1,5): I_warehouse(1)(1)(5)#10 <= 136.8
 WarehouseInventoryCapacity(2,1,5): I_warehouse(2)(1)(5)#40 <= 136.8
 WarehouseInventoryCapacity(1,2,5): I_warehouse(1)(2)(5)#20 <= 136.8
 WarehouseInventoryCapacity(2,2,5): I_warehouse(2)(2)(5)#50 <= 136.8
 WarehouseInventoryCapacity(1,3,5): I_warehouse(1)(3)(5)#30 <= 136.8
 WarehouseInventoryCapacity(2,3,5): I_warehouse(2)(3)(5)#60 <= 136.8
 WarehouseInventoryCapacity(1,1,6): I_warehouse(1)(1)(6)#11 <= 136.8
 WarehouseInventoryCapacity(2,1,6): I_warehouse(2)(1)(6)#41 <= 136.8
 WarehouseInventoryCapacity(1,2,6): I_warehouse(1)(2)(6)#21 <= 136.8
 WarehouseInventoryCapacity(2,2,6): I_warehouse(2)(2)(6)#51 <= 136.8
 WarehouseInventoryCapacity(1,3,6): I_warehouse(1)(3)(6)#31 <= 136.8
 WarehouseInventoryCapacity(2,3,6): I_warehouse(2)(3)(6)#61 <= 136.8
 WarehouseInventoryCapacity(1,1,7): I_warehouse(1)(1)(7)#12 <= 136.8
 WarehouseInventoryCapacity(2,1,7): I_warehouse(2)(1)(7)#42 <= 136.8
 WarehouseInventoryCapacity(1,2,7): I_warehouse(1)(2)(7)#22 <= 136.8
 WarehouseInventoryCapacity(2,2,7): I_warehouse(2)(2)(7)#52 <= 136.8
 WarehouseInventoryCapacity(1,3,7): I_warehouse(1)(3)(7)#32 <= 136.8
 WarehouseInventoryCapacity(2,3,7): I_warehouse(2)(3)(7)#62 <= 136.8
 WarehouseInventoryCapacity(1,1,8): I_warehouse(1)(1)(8)#13 <= 136.8
 WarehouseInventoryCapacity(2,1,8): I_warehouse(2)(1)(8)#43 <= 136.8
 WarehouseInventoryCapacity(1,2,8): I_warehouse(1)(2)(8)#23 <= 136.8
 WarehouseInventoryCapacity(2,2,8): I_warehouse(2)(2)(8)#53 <= 136.8
 WarehouseInventoryCapacity(1,3,8): I_warehouse(1)(3)(8)#33 <= 136.8
 WarehouseInventoryCapacity(2,3,8): I_warehouse(2)(3)(8)#63 <= 136.8
 WarehouseInventoryCapacity(1,1,9): I_warehouse(1)(1)(9)#14 <= 136.8
 WarehouseInventoryCapacity(2,1,9): I_warehouse(2)(1)(9)#44 <= 136.8
 WarehouseInventoryCapacity(1,2,9): I_warehouse(1)(2)(9)#24 <= 136.8
 WarehouseInventoryCapacity(2,2,9): I_warehouse(2)(2)(9)#54 <= 136.8
 WarehouseInventoryCapacity(1,3,9): I_warehouse(1)(3)(9)#34 <= 136.8
 WarehouseInventoryCapacity(2,3,9): I_warehouse(2)(3)(9)#64 <= 136.8
 WarehouseInventoryCapacity(1,1,10): I_warehouse(1)(1)(10)#15 <= 136.8
 WarehouseInventoryCapacity(2,1,10): I_warehouse(2)(1)(10)#45 <= 136.8
 WarehouseInventoryCapacity(1,2,10): I_warehouse(1)(2)(10)#25 <= 136.8
 WarehouseInventoryCapacity(2,2,10): I_warehouse(2)(2)(10)#55 <= 136.8
 WarehouseInventoryCapacity(1,3,10): I_warehouse(1)(3)(10)#35 <= 136.8
 WarehouseInventoryCapacity(2,3,10): I_warehouse(2)(3)(10)#65 <= 136.8
 WarehouseInventoryBalance(1,1,1): I_warehouse(1)(1)(1)#6 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(1)#678
                                 + w(3)(2)(1)(1)#708 + w(3)(3)(1)(1)#738
                                 + w(4)(1)(1)(1)#858 + w(4)(2)(1)(1)#888
                                 + w(4)(3)(1)(1)#918 + w(5)(1)(1)(1)#1038
                                 + w(5)(2)(1)(1)#1068 + w(5)(3)(1)(1)#1098
                                 + w(6)(1)(1)(1)#1218 + w(6)(2)(1)(1)#1248
                                 + w(6)(3)(1)(1)#1278 + w(7)(1)(1)(1)#1398
                                 + w(7)(2)(1)(1)#1428 + w(7)(3)(1)(1)#1458
                                 + w(8)(1)(1)(1)#1578 + w(8)(2)(1)(1)#1608
                                 + w(8)(3)(1)(1)#1638 + w(9)(1)(1)(1)#1758
                                 + w(9)(2)(1)(1)#1788 + w(9)(3)(1)(1)#1818
                                 + w(10)(1)(1)(1)#1938 + w(10)(2)(1)(1)#1968
                                 + w(10)(3)(1)(1)#1998 + w(11)(1)(1)(1)#2118
                                 + w(11)(2)(1)(1)#2148 + w(11)(3)(1)(1)#2178
                                 + w(12)(1)(1)(1)#2298 + w(12)(2)(1)(1)#2328
                                 + w(12)(3)(1)(1)#2358  = 0
 WarehouseInventoryBalance(2,1,1): I_warehouse(2)(1)(1)#36 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(1)#768
                                 + w(3)(5)(1)(1)#798 + w(3)(6)(1)(1)#828
                                 + w(4)(4)(1)(1)#948 + w(4)(5)(1)(1)#978
                                 + w(4)(6)(1)(1)#1008 + w(5)(4)(1)(1)#1128
                                 + w(5)(5)(1)(1)#1158 + w(5)(6)(1)(1)#1188
                                 + w(6)(4)(1)(1)#1308 + w(6)(5)(1)(1)#1338
                                 + w(6)(6)(1)(1)#1368 + w(7)(4)(1)(1)#1488
                                 + w(7)(5)(1)(1)#1518 + w(7)(6)(1)(1)#1548
                                 + w(8)(4)(1)(1)#1668 + w(8)(5)(1)(1)#1698
                                 + w(8)(6)(1)(1)#1728 + w(9)(4)(1)(1)#1848
                                 + w(9)(5)(1)(1)#1878 + w(9)(6)(1)(1)#1908
                                 + w(10)(4)(1)(1)#2028 + w(10)(5)(1)(1)#2058
                                 + w(10)(6)(1)(1)#2088 + w(11)(4)(1)(1)#2208
                                 + w(11)(5)(1)(1)#2238 + w(11)(6)(1)(1)#2268
                                 + w(12)(4)(1)(1)#2388 + w(12)(5)(1)(1)#2418
                                 + w(12)(6)(1)(1)#2448  = 0
 WarehouseInventoryBalance(1,2,1): - I_warehouse(1)(1)(1)#6
                                 + I_warehouse(1)(2)(1)#16 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(1)#688
                                 + w(3)(2)(2)(1)#718 + w(3)(3)(2)(1)#748
                                 + w(4)(1)(2)(1)#868 + w(4)(2)(2)(1)#898
                                 + w(4)(3)(2)(1)#928 + w(5)(1)(2)(1)#1048
                                 + w(5)(2)(2)(1)#1078 + w(5)(3)(2)(1)#1108
                                 + w(6)(1)(2)(1)#1228 + w(6)(2)(2)(1)#1258
                                 + w(6)(3)(2)(1)#1288 + w(7)(1)(2)(1)#1408
                                 + w(7)(2)(2)(1)#1438 + w(7)(3)(2)(1)#1468
                                 + w(8)(1)(2)(1)#1588 + w(8)(2)(2)(1)#1618
                                 + w(8)(3)(2)(1)#1648 + w(9)(1)(2)(1)#1768
                                 + w(9)(2)(2)(1)#1798 + w(9)(3)(2)(1)#1828
                                 + w(10)(1)(2)(1)#1948 + w(10)(2)(2)(1)#1978
                                 + w(10)(3)(2)(1)#2008 + w(11)(1)(2)(1)#2128
                                 + w(11)(2)(2)(1)#2158 + w(11)(3)(2)(1)#2188
                                 + w(12)(1)(2)(1)#2308 + w(12)(2)(2)(1)#2338
                                 + w(12)(3)(2)(1)#2368  = 0
 WarehouseInventoryBalance(2,2,1): - I_warehouse(2)(1)(1)#36
                                 + I_warehouse(2)(2)(1)#46 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(1)#778
                                 + w(3)(5)(2)(1)#808 + w(3)(6)(2)(1)#838
                                 + w(4)(4)(2)(1)#958 + w(4)(5)(2)(1)#988
                                 + w(4)(6)(2)(1)#1018 + w(5)(4)(2)(1)#1138
                                 + w(5)(5)(2)(1)#1168 + w(5)(6)(2)(1)#1198
                                 + w(6)(4)(2)(1)#1318 + w(6)(5)(2)(1)#1348
                                 + w(6)(6)(2)(1)#1378 + w(7)(4)(2)(1)#1498
                                 + w(7)(5)(2)(1)#1528 + w(7)(6)(2)(1)#1558
                                 + w(8)(4)(2)(1)#1678 + w(8)(5)(2)(1)#1708
                                 + w(8)(6)(2)(1)#1738 + w(9)(4)(2)(1)#1858
                                 + w(9)(5)(2)(1)#1888 + w(9)(6)(2)(1)#1918
                                 + w(10)(4)(2)(1)#2038 + w(10)(5)(2)(1)#2068
                                 + w(10)(6)(2)(1)#2098 + w(11)(4)(2)(1)#2218
                                 + w(11)(5)(2)(1)#2248 + w(11)(6)(2)(1)#2278
                                 + w(12)(4)(2)(1)#2398 + w(12)(5)(2)(1)#2428
                                 + w(12)(6)(2)(1)#2458  = 0
 WarehouseInventoryBalance(1,3,1): - I_warehouse(1)(2)(1)#16
                                 + I_warehouse(1)(3)(1)#26 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(1)#698
                                 + w(3)(2)(3)(1)#728 + w(3)(3)(3)(1)#758
                                 + w(4)(1)(3)(1)#878 + w(4)(2)(3)(1)#908
                                 + w(4)(3)(3)(1)#938 + w(5)(1)(3)(1)#1058
                                 + w(5)(2)(3)(1)#1088 + w(5)(3)(3)(1)#1118
                                 + w(6)(1)(3)(1)#1238 + w(6)(2)(3)(1)#1268
                                 + w(6)(3)(3)(1)#1298 + w(7)(1)(3)(1)#1418
                                 + w(7)(2)(3)(1)#1448 + w(7)(3)(3)(1)#1478
                                 + w(8)(1)(3)(1)#1598 + w(8)(2)(3)(1)#1628
                                 + w(8)(3)(3)(1)#1658 + w(9)(1)(3)(1)#1778
                                 + w(9)(2)(3)(1)#1808 + w(9)(3)(3)(1)#1838
                                 + w(10)(1)(3)(1)#1958 + w(10)(2)(3)(1)#1988
                                 + w(10)(3)(3)(1)#2018 + w(11)(1)(3)(1)#2138
                                 + w(11)(2)(3)(1)#2168 + w(11)(3)(3)(1)#2198
                                 + w(12)(1)(3)(1)#2318 + w(12)(2)(3)(1)#2348
                                 + w(12)(3)(3)(1)#2378  = 0
 WarehouseInventoryBalance(2,3,1): - I_warehouse(2)(2)(1)#46
                                 + I_warehouse(2)(3)(1)#56 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(1)#788
                                 + w(3)(5)(3)(1)#818 + w(3)(6)(3)(1)#848
                                 + w(4)(4)(3)(1)#968 + w(4)(5)(3)(1)#998
                                 + w(4)(6)(3)(1)#1028 + w(5)(4)(3)(1)#1148
                                 + w(5)(5)(3)(1)#1178 + w(5)(6)(3)(1)#1208
                                 + w(6)(4)(3)(1)#1328 + w(6)(5)(3)(1)#1358
                                 + w(6)(6)(3)(1)#1388 + w(7)(4)(3)(1)#1508
                                 + w(7)(5)(3)(1)#1538 + w(7)(6)(3)(1)#1568
                                 + w(8)(4)(3)(1)#1688 + w(8)(5)(3)(1)#1718
                                 + w(8)(6)(3)(1)#1748 + w(9)(4)(3)(1)#1868
                                 + w(9)(5)(3)(1)#1898 + w(9)(6)(3)(1)#1928
                                 + w(10)(4)(3)(1)#2048 + w(10)(5)(3)(1)#2078
                                 + w(10)(6)(3)(1)#2108 + w(11)(4)(3)(1)#2228
                                 + w(11)(5)(3)(1)#2258 + w(11)(6)(3)(1)#2288
                                 + w(12)(4)(3)(1)#2408 + w(12)(5)(3)(1)#2438
                                 + w(12)(6)(3)(1)#2468  = 0
 WarehouseInventoryBalance(1,1,2): I_warehouse(1)(1)(2)#7 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(2)#679
                                 + w(3)(2)(1)(2)#709 + w(3)(3)(1)(2)#739
                                 + w(4)(1)(1)(2)#859 + w(4)(2)(1)(2)#889
                                 + w(4)(3)(1)(2)#919 + w(5)(1)(1)(2)#1039
                                 + w(5)(2)(1)(2)#1069 + w(5)(3)(1)(2)#1099
                                 + w(6)(1)(1)(2)#1219 + w(6)(2)(1)(2)#1249
                                 + w(6)(3)(1)(2)#1279 + w(7)(1)(1)(2)#1399
                                 + w(7)(2)(1)(2)#1429 + w(7)(3)(1)(2)#1459
                                 + w(8)(1)(1)(2)#1579 + w(8)(2)(1)(2)#1609
                                 + w(8)(3)(1)(2)#1639 + w(9)(1)(1)(2)#1759
                                 + w(9)(2)(1)(2)#1789 + w(9)(3)(1)(2)#1819
                                 + w(10)(1)(1)(2)#1939 + w(10)(2)(1)(2)#1969
                                 + w(10)(3)(1)(2)#1999 + w(11)(1)(1)(2)#2119
                                 + w(11)(2)(1)(2)#2149 + w(11)(3)(1)(2)#2179
                                 + w(12)(1)(1)(2)#2299 + w(12)(2)(1)(2)#2329
                                 + w(12)(3)(1)(2)#2359  = 0
 WarehouseInventoryBalance(2,1,2): I_warehouse(2)(1)(2)#37 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(2)#769
                                 + w(3)(5)(1)(2)#799 + w(3)(6)(1)(2)#829
                                 + w(4)(4)(1)(2)#949 + w(4)(5)(1)(2)#979
                                 + w(4)(6)(1)(2)#1009 + w(5)(4)(1)(2)#1129
                                 + w(5)(5)(1)(2)#1159 + w(5)(6)(1)(2)#1189
                                 + w(6)(4)(1)(2)#1309 + w(6)(5)(1)(2)#1339
                                 + w(6)(6)(1)(2)#1369 + w(7)(4)(1)(2)#1489
                                 + w(7)(5)(1)(2)#1519 + w(7)(6)(1)(2)#1549
                                 + w(8)(4)(1)(2)#1669 + w(8)(5)(1)(2)#1699
                                 + w(8)(6)(1)(2)#1729 + w(9)(4)(1)(2)#1849
                                 + w(9)(5)(1)(2)#1879 + w(9)(6)(1)(2)#1909
                                 + w(10)(4)(1)(2)#2029 + w(10)(5)(1)(2)#2059
                                 + w(10)(6)(1)(2)#2089 + w(11)(4)(1)(2)#2209
                                 + w(11)(5)(1)(2)#2239 + w(11)(6)(1)(2)#2269
                                 + w(12)(4)(1)(2)#2389 + w(12)(5)(1)(2)#2419
                                 + w(12)(6)(1)(2)#2449  = 0
 WarehouseInventoryBalance(1,2,2): - I_warehouse(1)(1)(2)#7
                                 + I_warehouse(1)(2)(2)#17 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(2)#689
                                 + w(3)(2)(2)(2)#719 + w(3)(3)(2)(2)#749
                                 + w(4)(1)(2)(2)#869 + w(4)(2)(2)(2)#899
                                 + w(4)(3)(2)(2)#929 + w(5)(1)(2)(2)#1049
                                 + w(5)(2)(2)(2)#1079 + w(5)(3)(2)(2)#1109
                                 + w(6)(1)(2)(2)#1229 + w(6)(2)(2)(2)#1259
                                 + w(6)(3)(2)(2)#1289 + w(7)(1)(2)(2)#1409
                                 + w(7)(2)(2)(2)#1439 + w(7)(3)(2)(2)#1469
                                 + w(8)(1)(2)(2)#1589 + w(8)(2)(2)(2)#1619
                                 + w(8)(3)(2)(2)#1649 + w(9)(1)(2)(2)#1769
                                 + w(9)(2)(2)(2)#1799 + w(9)(3)(2)(2)#1829
                                 + w(10)(1)(2)(2)#1949 + w(10)(2)(2)(2)#1979
                                 + w(10)(3)(2)(2)#2009 + w(11)(1)(2)(2)#2129
                                 + w(11)(2)(2)(2)#2159 + w(11)(3)(2)(2)#2189
                                 + w(12)(1)(2)(2)#2309 + w(12)(2)(2)(2)#2339
                                 + w(12)(3)(2)(2)#2369  = 0
 WarehouseInventoryBalance(2,2,2): - I_warehouse(2)(1)(2)#37
                                 + I_warehouse(2)(2)(2)#47 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(2)#779
                                 + w(3)(5)(2)(2)#809 + w(3)(6)(2)(2)#839
                                 + w(4)(4)(2)(2)#959 + w(4)(5)(2)(2)#989
                                 + w(4)(6)(2)(2)#1019 + w(5)(4)(2)(2)#1139
                                 + w(5)(5)(2)(2)#1169 + w(5)(6)(2)(2)#1199
                                 + w(6)(4)(2)(2)#1319 + w(6)(5)(2)(2)#1349
                                 + w(6)(6)(2)(2)#1379 + w(7)(4)(2)(2)#1499
                                 + w(7)(5)(2)(2)#1529 + w(7)(6)(2)(2)#1559
                                 + w(8)(4)(2)(2)#1679 + w(8)(5)(2)(2)#1709
                                 + w(8)(6)(2)(2)#1739 + w(9)(4)(2)(2)#1859
                                 + w(9)(5)(2)(2)#1889 + w(9)(6)(2)(2)#1919
                                 + w(10)(4)(2)(2)#2039 + w(10)(5)(2)(2)#2069
                                 + w(10)(6)(2)(2)#2099 + w(11)(4)(2)(2)#2219
                                 + w(11)(5)(2)(2)#2249 + w(11)(6)(2)(2)#2279
                                 + w(12)(4)(2)(2)#2399 + w(12)(5)(2)(2)#2429
                                 + w(12)(6)(2)(2)#2459  = 0
 WarehouseInventoryBalance(1,3,2): - I_warehouse(1)(2)(2)#17
                                 + I_warehouse(1)(3)(2)#27 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(2)#699
                                 + w(3)(2)(3)(2)#729 + w(3)(3)(3)(2)#759
                                 + w(4)(1)(3)(2)#879 + w(4)(2)(3)(2)#909
                                 + w(4)(3)(3)(2)#939 + w(5)(1)(3)(2)#1059
                                 + w(5)(2)(3)(2)#1089 + w(5)(3)(3)(2)#1119
                                 + w(6)(1)(3)(2)#1239 + w(6)(2)(3)(2)#1269
                                 + w(6)(3)(3)(2)#1299 + w(7)(1)(3)(2)#1419
                                 + w(7)(2)(3)(2)#1449 + w(7)(3)(3)(2)#1479
                                 + w(8)(1)(3)(2)#1599 + w(8)(2)(3)(2)#1629
                                 + w(8)(3)(3)(2)#1659 + w(9)(1)(3)(2)#1779
                                 + w(9)(2)(3)(2)#1809 + w(9)(3)(3)(2)#1839
                                 + w(10)(1)(3)(2)#1959 + w(10)(2)(3)(2)#1989
                                 + w(10)(3)(3)(2)#2019 + w(11)(1)(3)(2)#2139
                                 + w(11)(2)(3)(2)#2169 + w(11)(3)(3)(2)#2199
                                 + w(12)(1)(3)(2)#2319 + w(12)(2)(3)(2)#2349
                                 + w(12)(3)(3)(2)#2379  = 0
 WarehouseInventoryBalance(2,3,2): - I_warehouse(2)(2)(2)#47
                                 + I_warehouse(2)(3)(2)#57 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(2)#789
                                 + w(3)(5)(3)(2)#819 + w(3)(6)(3)(2)#849
                                 + w(4)(4)(3)(2)#969 + w(4)(5)(3)(2)#999
                                 + w(4)(6)(3)(2)#1029 + w(5)(4)(3)(2)#1149
                                 + w(5)(5)(3)(2)#1179 + w(5)(6)(3)(2)#1209
                                 + w(6)(4)(3)(2)#1329 + w(6)(5)(3)(2)#1359
                                 + w(6)(6)(3)(2)#1389 + w(7)(4)(3)(2)#1509
                                 + w(7)(5)(3)(2)#1539 + w(7)(6)(3)(2)#1569
                                 + w(8)(4)(3)(2)#1689 + w(8)(5)(3)(2)#1719
                                 + w(8)(6)(3)(2)#1749 + w(9)(4)(3)(2)#1869
                                 + w(9)(5)(3)(2)#1899 + w(9)(6)(3)(2)#1929
                                 + w(10)(4)(3)(2)#2049 + w(10)(5)(3)(2)#2079
                                 + w(10)(6)(3)(2)#2109 + w(11)(4)(3)(2)#2229
                                 + w(11)(5)(3)(2)#2259 + w(11)(6)(3)(2)#2289
                                 + w(12)(4)(3)(2)#2409 + w(12)(5)(3)(2)#2439
                                 + w(12)(6)(3)(2)#2469  = 0
 WarehouseInventoryBalance(1,1,3): I_warehouse(1)(1)(3)#8 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(3)#680
                                 + w(3)(2)(1)(3)#710 + w(3)(3)(1)(3)#740
                                 + w(4)(1)(1)(3)#860 + w(4)(2)(1)(3)#890
                                 + w(4)(3)(1)(3)#920 + w(5)(1)(1)(3)#1040
                                 + w(5)(2)(1)(3)#1070 + w(5)(3)(1)(3)#1100
                                 + w(6)(1)(1)(3)#1220 + w(6)(2)(1)(3)#1250
                                 + w(6)(3)(1)(3)#1280 + w(7)(1)(1)(3)#1400
                                 + w(7)(2)(1)(3)#1430 + w(7)(3)(1)(3)#1460
                                 + w(8)(1)(1)(3)#1580 + w(8)(2)(1)(3)#1610
                                 + w(8)(3)(1)(3)#1640 + w(9)(1)(1)(3)#1760
                                 + w(9)(2)(1)(3)#1790 + w(9)(3)(1)(3)#1820
                                 + w(10)(1)(1)(3)#1940 + w(10)(2)(1)(3)#1970
                                 + w(10)(3)(1)(3)#2000 + w(11)(1)(1)(3)#2120
                                 + w(11)(2)(1)(3)#2150 + w(11)(3)(1)(3)#2180
                                 + w(12)(1)(1)(3)#2300 + w(12)(2)(1)(3)#2330
                                 + w(12)(3)(1)(3)#2360  = 0
 WarehouseInventoryBalance(2,1,3): I_warehouse(2)(1)(3)#38 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(3)#770
                                 + w(3)(5)(1)(3)#800 + w(3)(6)(1)(3)#830
                                 + w(4)(4)(1)(3)#950 + w(4)(5)(1)(3)#980
                                 + w(4)(6)(1)(3)#1010 + w(5)(4)(1)(3)#1130
                                 + w(5)(5)(1)(3)#1160 + w(5)(6)(1)(3)#1190
                                 + w(6)(4)(1)(3)#1310 + w(6)(5)(1)(3)#1340
                                 + w(6)(6)(1)(3)#1370 + w(7)(4)(1)(3)#1490
                                 + w(7)(5)(1)(3)#1520 + w(7)(6)(1)(3)#1550
                                 + w(8)(4)(1)(3)#1670 + w(8)(5)(1)(3)#1700
                                 + w(8)(6)(1)(3)#1730 + w(9)(4)(1)(3)#1850
                                 + w(9)(5)(1)(3)#1880 + w(9)(6)(1)(3)#1910
                                 + w(10)(4)(1)(3)#2030 + w(10)(5)(1)(3)#2060
                                 + w(10)(6)(1)(3)#2090 + w(11)(4)(1)(3)#2210
                                 + w(11)(5)(1)(3)#2240 + w(11)(6)(1)(3)#2270
                                 + w(12)(4)(1)(3)#2390 + w(12)(5)(1)(3)#2420
                                 + w(12)(6)(1)(3)#2450  = 0
 WarehouseInventoryBalance(1,2,3): - I_warehouse(1)(1)(3)#8
                                 + I_warehouse(1)(2)(3)#18 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(3)#690
                                 + w(3)(2)(2)(3)#720 + w(3)(3)(2)(3)#750
                                 + w(4)(1)(2)(3)#870 + w(4)(2)(2)(3)#900
                                 + w(4)(3)(2)(3)#930 + w(5)(1)(2)(3)#1050
                                 + w(5)(2)(2)(3)#1080 + w(5)(3)(2)(3)#1110
                                 + w(6)(1)(2)(3)#1230 + w(6)(2)(2)(3)#1260
                                 + w(6)(3)(2)(3)#1290 + w(7)(1)(2)(3)#1410
                                 + w(7)(2)(2)(3)#1440 + w(7)(3)(2)(3)#1470
                                 + w(8)(1)(2)(3)#1590 + w(8)(2)(2)(3)#1620
                                 + w(8)(3)(2)(3)#1650 + w(9)(1)(2)(3)#1770
                                 + w(9)(2)(2)(3)#1800 + w(9)(3)(2)(3)#1830
                                 + w(10)(1)(2)(3)#1950 + w(10)(2)(2)(3)#1980
                                 + w(10)(3)(2)(3)#2010 + w(11)(1)(2)(3)#2130
                                 + w(11)(2)(2)(3)#2160 + w(11)(3)(2)(3)#2190
                                 + w(12)(1)(2)(3)#2310 + w(12)(2)(2)(3)#2340
                                 + w(12)(3)(2)(3)#2370  = 0
 WarehouseInventoryBalance(2,2,3): - I_warehouse(2)(1)(3)#38
                                 + I_warehouse(2)(2)(3)#48 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(3)#780
                                 + w(3)(5)(2)(3)#810 + w(3)(6)(2)(3)#840
                                 + w(4)(4)(2)(3)#960 + w(4)(5)(2)(3)#990
                                 + w(4)(6)(2)(3)#1020 + w(5)(4)(2)(3)#1140
                                 + w(5)(5)(2)(3)#1170 + w(5)(6)(2)(3)#1200
                                 + w(6)(4)(2)(3)#1320 + w(6)(5)(2)(3)#1350
                                 + w(6)(6)(2)(3)#1380 + w(7)(4)(2)(3)#1500
                                 + w(7)(5)(2)(3)#1530 + w(7)(6)(2)(3)#1560
                                 + w(8)(4)(2)(3)#1680 + w(8)(5)(2)(3)#1710
                                 + w(8)(6)(2)(3)#1740 + w(9)(4)(2)(3)#1860
                                 + w(9)(5)(2)(3)#1890 + w(9)(6)(2)(3)#1920
                                 + w(10)(4)(2)(3)#2040 + w(10)(5)(2)(3)#2070
                                 + w(10)(6)(2)(3)#2100 + w(11)(4)(2)(3)#2220
                                 + w(11)(5)(2)(3)#2250 + w(11)(6)(2)(3)#2280
                                 + w(12)(4)(2)(3)#2400 + w(12)(5)(2)(3)#2430
                                 + w(12)(6)(2)(3)#2460  = 0
 WarehouseInventoryBalance(1,3,3): - I_warehouse(1)(2)(3)#18
                                 + I_warehouse(1)(3)(3)#28 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(3)#700
                                 + w(3)(2)(3)(3)#730 + w(3)(3)(3)(3)#760
                                 + w(4)(1)(3)(3)#880 + w(4)(2)(3)(3)#910
                                 + w(4)(3)(3)(3)#940 + w(5)(1)(3)(3)#1060
                                 + w(5)(2)(3)(3)#1090 + w(5)(3)(3)(3)#1120
                                 + w(6)(1)(3)(3)#1240 + w(6)(2)(3)(3)#1270
                                 + w(6)(3)(3)(3)#1300 + w(7)(1)(3)(3)#1420
                                 + w(7)(2)(3)(3)#1450 + w(7)(3)(3)(3)#1480
                                 + w(8)(1)(3)(3)#1600 + w(8)(2)(3)(3)#1630
                                 + w(8)(3)(3)(3)#1660 + w(9)(1)(3)(3)#1780
                                 + w(9)(2)(3)(3)#1810 + w(9)(3)(3)(3)#1840
                                 + w(10)(1)(3)(3)#1960 + w(10)(2)(3)(3)#1990
                                 + w(10)(3)(3)(3)#2020 + w(11)(1)(3)(3)#2140
                                 + w(11)(2)(3)(3)#2170 + w(11)(3)(3)(3)#2200
                                 + w(12)(1)(3)(3)#2320 + w(12)(2)(3)(3)#2350
                                 + w(12)(3)(3)(3)#2380  = 0
 WarehouseInventoryBalance(2,3,3): - I_warehouse(2)(2)(3)#48
                                 + I_warehouse(2)(3)(3)#58 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(3)#790
                                 + w(3)(5)(3)(3)#820 + w(3)(6)(3)(3)#850
                                 + w(4)(4)(3)(3)#970 + w(4)(5)(3)(3)#1000
                                 + w(4)(6)(3)(3)#1030 + w(5)(4)(3)(3)#1150
                                 + w(5)(5)(3)(3)#1180 + w(5)(6)(3)(3)#1210
                                 + w(6)(4)(3)(3)#1330 + w(6)(5)(3)(3)#1360
                                 + w(6)(6)(3)(3)#1390 + w(7)(4)(3)(3)#1510
                                 + w(7)(5)(3)(3)#1540 + w(7)(6)(3)(3)#1570
                                 + w(8)(4)(3)(3)#1690 + w(8)(5)(3)(3)#1720
                                 + w(8)(6)(3)(3)#1750 + w(9)(4)(3)(3)#1870
                                 + w(9)(5)(3)(3)#1900 + w(9)(6)(3)(3)#1930
                                 + w(10)(4)(3)(3)#2050 + w(10)(5)(3)(3)#2080
                                 + w(10)(6)(3)(3)#2110 + w(11)(4)(3)(3)#2230
                                 + w(11)(5)(3)(3)#2260 + w(11)(6)(3)(3)#2290
                                 + w(12)(4)(3)(3)#2410 + w(12)(5)(3)(3)#2440
                                 + w(12)(6)(3)(3)#2470  = 0
 WarehouseInventoryBalance(1,1,4): I_warehouse(1)(1)(4)#9 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(4)#681
                                 + w(3)(2)(1)(4)#711 + w(3)(3)(1)(4)#741
                                 + w(4)(1)(1)(4)#861 + w(4)(2)(1)(4)#891
                                 + w(4)(3)(1)(4)#921 + w(5)(1)(1)(4)#1041
                                 + w(5)(2)(1)(4)#1071 + w(5)(3)(1)(4)#1101
                                 + w(6)(1)(1)(4)#1221 + w(6)(2)(1)(4)#1251
                                 + w(6)(3)(1)(4)#1281 + w(7)(1)(1)(4)#1401
                                 + w(7)(2)(1)(4)#1431 + w(7)(3)(1)(4)#1461
                                 + w(8)(1)(1)(4)#1581 + w(8)(2)(1)(4)#1611
                                 + w(8)(3)(1)(4)#1641 + w(9)(1)(1)(4)#1761
                                 + w(9)(2)(1)(4)#1791 + w(9)(3)(1)(4)#1821
                                 + w(10)(1)(1)(4)#1941 + w(10)(2)(1)(4)#1971
                                 + w(10)(3)(1)(4)#2001 + w(11)(1)(1)(4)#2121
                                 + w(11)(2)(1)(4)#2151 + w(11)(3)(1)(4)#2181
                                 + w(12)(1)(1)(4)#2301 + w(12)(2)(1)(4)#2331
                                 + w(12)(3)(1)(4)#2361  = 0
 WarehouseInventoryBalance(2,1,4): I_warehouse(2)(1)(4)#39 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(4)#771
                                 + w(3)(5)(1)(4)#801 + w(3)(6)(1)(4)#831
                                 + w(4)(4)(1)(4)#951 + w(4)(5)(1)(4)#981
                                 + w(4)(6)(1)(4)#1011 + w(5)(4)(1)(4)#1131
                                 + w(5)(5)(1)(4)#1161 + w(5)(6)(1)(4)#1191
                                 + w(6)(4)(1)(4)#1311 + w(6)(5)(1)(4)#1341
                                 + w(6)(6)(1)(4)#1371 + w(7)(4)(1)(4)#1491
                                 + w(7)(5)(1)(4)#1521 + w(7)(6)(1)(4)#1551
                                 + w(8)(4)(1)(4)#1671 + w(8)(5)(1)(4)#1701
                                 + w(8)(6)(1)(4)#1731 + w(9)(4)(1)(4)#1851
                                 + w(9)(5)(1)(4)#1881 + w(9)(6)(1)(4)#1911
                                 + w(10)(4)(1)(4)#2031 + w(10)(5)(1)(4)#2061
                                 + w(10)(6)(1)(4)#2091 + w(11)(4)(1)(4)#2211
                                 + w(11)(5)(1)(4)#2241 + w(11)(6)(1)(4)#2271
                                 + w(12)(4)(1)(4)#2391 + w(12)(5)(1)(4)#2421
                                 + w(12)(6)(1)(4)#2451  = 0
 WarehouseInventoryBalance(1,2,4): - I_warehouse(1)(1)(4)#9
                                 + I_warehouse(1)(2)(4)#19 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(4)#691
                                 + w(3)(2)(2)(4)#721 + w(3)(3)(2)(4)#751
                                 + w(4)(1)(2)(4)#871 + w(4)(2)(2)(4)#901
                                 + w(4)(3)(2)(4)#931 + w(5)(1)(2)(4)#1051
                                 + w(5)(2)(2)(4)#1081 + w(5)(3)(2)(4)#1111
                                 + w(6)(1)(2)(4)#1231 + w(6)(2)(2)(4)#1261
                                 + w(6)(3)(2)(4)#1291 + w(7)(1)(2)(4)#1411
                                 + w(7)(2)(2)(4)#1441 + w(7)(3)(2)(4)#1471
                                 + w(8)(1)(2)(4)#1591 + w(8)(2)(2)(4)#1621
                                 + w(8)(3)(2)(4)#1651 + w(9)(1)(2)(4)#1771
                                 + w(9)(2)(2)(4)#1801 + w(9)(3)(2)(4)#1831
                                 + w(10)(1)(2)(4)#1951 + w(10)(2)(2)(4)#1981
                                 + w(10)(3)(2)(4)#2011 + w(11)(1)(2)(4)#2131
                                 + w(11)(2)(2)(4)#2161 + w(11)(3)(2)(4)#2191
                                 + w(12)(1)(2)(4)#2311 + w(12)(2)(2)(4)#2341
                                 + w(12)(3)(2)(4)#2371  = 0
 WarehouseInventoryBalance(2,2,4): - I_warehouse(2)(1)(4)#39
                                 + I_warehouse(2)(2)(4)#49 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(4)#781
                                 + w(3)(5)(2)(4)#811 + w(3)(6)(2)(4)#841
                                 + w(4)(4)(2)(4)#961 + w(4)(5)(2)(4)#991
                                 + w(4)(6)(2)(4)#1021 + w(5)(4)(2)(4)#1141
                                 + w(5)(5)(2)(4)#1171 + w(5)(6)(2)(4)#1201
                                 + w(6)(4)(2)(4)#1321 + w(6)(5)(2)(4)#1351
                                 + w(6)(6)(2)(4)#1381 + w(7)(4)(2)(4)#1501
                                 + w(7)(5)(2)(4)#1531 + w(7)(6)(2)(4)#1561
                                 + w(8)(4)(2)(4)#1681 + w(8)(5)(2)(4)#1711
                                 + w(8)(6)(2)(4)#1741 + w(9)(4)(2)(4)#1861
                                 + w(9)(5)(2)(4)#1891 + w(9)(6)(2)(4)#1921
                                 + w(10)(4)(2)(4)#2041 + w(10)(5)(2)(4)#2071
                                 + w(10)(6)(2)(4)#2101 + w(11)(4)(2)(4)#2221
                                 + w(11)(5)(2)(4)#2251 + w(11)(6)(2)(4)#2281
                                 + w(12)(4)(2)(4)#2401 + w(12)(5)(2)(4)#2431
                                 + w(12)(6)(2)(4)#2461  = 0
 WarehouseInventoryBalance(1,3,4): - I_warehouse(1)(2)(4)#19
                                 + I_warehouse(1)(3)(4)#29 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(4)#701
                                 + w(3)(2)(3)(4)#731 + w(3)(3)(3)(4)#761
                                 + w(4)(1)(3)(4)#881 + w(4)(2)(3)(4)#911
                                 + w(4)(3)(3)(4)#941 + w(5)(1)(3)(4)#1061
                                 + w(5)(2)(3)(4)#1091 + w(5)(3)(3)(4)#1121
                                 + w(6)(1)(3)(4)#1241 + w(6)(2)(3)(4)#1271
                                 + w(6)(3)(3)(4)#1301 + w(7)(1)(3)(4)#1421
                                 + w(7)(2)(3)(4)#1451 + w(7)(3)(3)(4)#1481
                                 + w(8)(1)(3)(4)#1601 + w(8)(2)(3)(4)#1631
                                 + w(8)(3)(3)(4)#1661 + w(9)(1)(3)(4)#1781
                                 + w(9)(2)(3)(4)#1811 + w(9)(3)(3)(4)#1841
                                 + w(10)(1)(3)(4)#1961 + w(10)(2)(3)(4)#1991
                                 + w(10)(3)(3)(4)#2021 + w(11)(1)(3)(4)#2141
                                 + w(11)(2)(3)(4)#2171 + w(11)(3)(3)(4)#2201
                                 + w(12)(1)(3)(4)#2321 + w(12)(2)(3)(4)#2351
                                 + w(12)(3)(3)(4)#2381  = 0
 WarehouseInventoryBalance(2,3,4): - I_warehouse(2)(2)(4)#49
                                 + I_warehouse(2)(3)(4)#59 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(4)#791
                                 + w(3)(5)(3)(4)#821 + w(3)(6)(3)(4)#851
                                 + w(4)(4)(3)(4)#971 + w(4)(5)(3)(4)#1001
                                 + w(4)(6)(3)(4)#1031 + w(5)(4)(3)(4)#1151
                                 + w(5)(5)(3)(4)#1181 + w(5)(6)(3)(4)#1211
                                 + w(6)(4)(3)(4)#1331 + w(6)(5)(3)(4)#1361
                                 + w(6)(6)(3)(4)#1391 + w(7)(4)(3)(4)#1511
                                 + w(7)(5)(3)(4)#1541 + w(7)(6)(3)(4)#1571
                                 + w(8)(4)(3)(4)#1691 + w(8)(5)(3)(4)#1721
                                 + w(8)(6)(3)(4)#1751 + w(9)(4)(3)(4)#1871
                                 + w(9)(5)(3)(4)#1901 + w(9)(6)(3)(4)#1931
                                 + w(10)(4)(3)(4)#2051 + w(10)(5)(3)(4)#2081
                                 + w(10)(6)(3)(4)#2111 + w(11)(4)(3)(4)#2231
                                 + w(11)(5)(3)(4)#2261 + w(11)(6)(3)(4)#2291
                                 + w(12)(4)(3)(4)#2411 + w(12)(5)(3)(4)#2441
                                 + w(12)(6)(3)(4)#2471  = 0
 WarehouseInventoryBalance(1,1,5): I_warehouse(1)(1)(5)#10 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(5)#682
                                 + w(3)(2)(1)(5)#712 + w(3)(3)(1)(5)#742
                                 + w(4)(1)(1)(5)#862 + w(4)(2)(1)(5)#892
                                 + w(4)(3)(1)(5)#922 + w(5)(1)(1)(5)#1042
                                 + w(5)(2)(1)(5)#1072 + w(5)(3)(1)(5)#1102
                                 + w(6)(1)(1)(5)#1222 + w(6)(2)(1)(5)#1252
                                 + w(6)(3)(1)(5)#1282 + w(7)(1)(1)(5)#1402
                                 + w(7)(2)(1)(5)#1432 + w(7)(3)(1)(5)#1462
                                 + w(8)(1)(1)(5)#1582 + w(8)(2)(1)(5)#1612
                                 + w(8)(3)(1)(5)#1642 + w(9)(1)(1)(5)#1762
                                 + w(9)(2)(1)(5)#1792 + w(9)(3)(1)(5)#1822
                                 + w(10)(1)(1)(5)#1942 + w(10)(2)(1)(5)#1972
                                 + w(10)(3)(1)(5)#2002 + w(11)(1)(1)(5)#2122
                                 + w(11)(2)(1)(5)#2152 + w(11)(3)(1)(5)#2182
                                 + w(12)(1)(1)(5)#2302 + w(12)(2)(1)(5)#2332
                                 + w(12)(3)(1)(5)#2362  = 0
 WarehouseInventoryBalance(2,1,5): I_warehouse(2)(1)(5)#40 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(5)#772
                                 + w(3)(5)(1)(5)#802 + w(3)(6)(1)(5)#832
                                 + w(4)(4)(1)(5)#952 + w(4)(5)(1)(5)#982
                                 + w(4)(6)(1)(5)#1012 + w(5)(4)(1)(5)#1132
                                 + w(5)(5)(1)(5)#1162 + w(5)(6)(1)(5)#1192
                                 + w(6)(4)(1)(5)#1312 + w(6)(5)(1)(5)#1342
                                 + w(6)(6)(1)(5)#1372 + w(7)(4)(1)(5)#1492
                                 + w(7)(5)(1)(5)#1522 + w(7)(6)(1)(5)#1552
                                 + w(8)(4)(1)(5)#1672 + w(8)(5)(1)(5)#1702
                                 + w(8)(6)(1)(5)#1732 + w(9)(4)(1)(5)#1852
                                 + w(9)(5)(1)(5)#1882 + w(9)(6)(1)(5)#1912
                                 + w(10)(4)(1)(5)#2032 + w(10)(5)(1)(5)#2062
                                 + w(10)(6)(1)(5)#2092 + w(11)(4)(1)(5)#2212
                                 + w(11)(5)(1)(5)#2242 + w(11)(6)(1)(5)#2272
                                 + w(12)(4)(1)(5)#2392 + w(12)(5)(1)(5)#2422
                                 + w(12)(6)(1)(5)#2452  = 0
 WarehouseInventoryBalance(1,2,5): - I_warehouse(1)(1)(5)#10
                                 + I_warehouse(1)(2)(5)#20 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(5)#692
                                 + w(3)(2)(2)(5)#722 + w(3)(3)(2)(5)#752
                                 + w(4)(1)(2)(5)#872 + w(4)(2)(2)(5)#902
                                 + w(4)(3)(2)(5)#932 + w(5)(1)(2)(5)#1052
                                 + w(5)(2)(2)(5)#1082 + w(5)(3)(2)(5)#1112
                                 + w(6)(1)(2)(5)#1232 + w(6)(2)(2)(5)#1262
                                 + w(6)(3)(2)(5)#1292 + w(7)(1)(2)(5)#1412
                                 + w(7)(2)(2)(5)#1442 + w(7)(3)(2)(5)#1472
                                 + w(8)(1)(2)(5)#1592 + w(8)(2)(2)(5)#1622
                                 + w(8)(3)(2)(5)#1652 + w(9)(1)(2)(5)#1772
                                 + w(9)(2)(2)(5)#1802 + w(9)(3)(2)(5)#1832
                                 + w(10)(1)(2)(5)#1952 + w(10)(2)(2)(5)#1982
                                 + w(10)(3)(2)(5)#2012 + w(11)(1)(2)(5)#2132
                                 + w(11)(2)(2)(5)#2162 + w(11)(3)(2)(5)#2192
                                 + w(12)(1)(2)(5)#2312 + w(12)(2)(2)(5)#2342
                                 + w(12)(3)(2)(5)#2372  = 0
 WarehouseInventoryBalance(2,2,5): - I_warehouse(2)(1)(5)#40
                                 + I_warehouse(2)(2)(5)#50 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(5)#782
                                 + w(3)(5)(2)(5)#812 + w(3)(6)(2)(5)#842
                                 + w(4)(4)(2)(5)#962 + w(4)(5)(2)(5)#992
                                 + w(4)(6)(2)(5)#1022 + w(5)(4)(2)(5)#1142
                                 + w(5)(5)(2)(5)#1172 + w(5)(6)(2)(5)#1202
                                 + w(6)(4)(2)(5)#1322 + w(6)(5)(2)(5)#1352
                                 + w(6)(6)(2)(5)#1382 + w(7)(4)(2)(5)#1502
                                 + w(7)(5)(2)(5)#1532 + w(7)(6)(2)(5)#1562
                                 + w(8)(4)(2)(5)#1682 + w(8)(5)(2)(5)#1712
                                 + w(8)(6)(2)(5)#1742 + w(9)(4)(2)(5)#1862
                                 + w(9)(5)(2)(5)#1892 + w(9)(6)(2)(5)#1922
                                 + w(10)(4)(2)(5)#2042 + w(10)(5)(2)(5)#2072
                                 + w(10)(6)(2)(5)#2102 + w(11)(4)(2)(5)#2222
                                 + w(11)(5)(2)(5)#2252 + w(11)(6)(2)(5)#2282
                                 + w(12)(4)(2)(5)#2402 + w(12)(5)(2)(5)#2432
                                 + w(12)(6)(2)(5)#2462  = 0
 WarehouseInventoryBalance(1,3,5): - I_warehouse(1)(2)(5)#20
                                 + I_warehouse(1)(3)(5)#30 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(5)#702
                                 + w(3)(2)(3)(5)#732 + w(3)(3)(3)(5)#762
                                 + w(4)(1)(3)(5)#882 + w(4)(2)(3)(5)#912
                                 + w(4)(3)(3)(5)#942 + w(5)(1)(3)(5)#1062
                                 + w(5)(2)(3)(5)#1092 + w(5)(3)(3)(5)#1122
                                 + w(6)(1)(3)(5)#1242 + w(6)(2)(3)(5)#1272
                                 + w(6)(3)(3)(5)#1302 + w(7)(1)(3)(5)#1422
                                 + w(7)(2)(3)(5)#1452 + w(7)(3)(3)(5)#1482
                                 + w(8)(1)(3)(5)#1602 + w(8)(2)(3)(5)#1632
                                 + w(8)(3)(3)(5)#1662 + w(9)(1)(3)(5)#1782
                                 + w(9)(2)(3)(5)#1812 + w(9)(3)(3)(5)#1842
                                 + w(10)(1)(3)(5)#1962 + w(10)(2)(3)(5)#1992
                                 + w(10)(3)(3)(5)#2022 + w(11)(1)(3)(5)#2142
                                 + w(11)(2)(3)(5)#2172 + w(11)(3)(3)(5)#2202
                                 + w(12)(1)(3)(5)#2322 + w(12)(2)(3)(5)#2352
                                 + w(12)(3)(3)(5)#2382  = 0
 WarehouseInventoryBalance(2,3,5): - I_warehouse(2)(2)(5)#50
                                 + I_warehouse(2)(3)(5)#60 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(5)#792
                                 + w(3)(5)(3)(5)#822 + w(3)(6)(3)(5)#852
                                 + w(4)(4)(3)(5)#972 + w(4)(5)(3)(5)#1002
                                 + w(4)(6)(3)(5)#1032 + w(5)(4)(3)(5)#1152
                                 + w(5)(5)(3)(5)#1182 + w(5)(6)(3)(5)#1212
                                 + w(6)(4)(3)(5)#1332 + w(6)(5)(3)(5)#1362
                                 + w(6)(6)(3)(5)#1392 + w(7)(4)(3)(5)#1512
                                 + w(7)(5)(3)(5)#1542 + w(7)(6)(3)(5)#1572
                                 + w(8)(4)(3)(5)#1692 + w(8)(5)(3)(5)#1722
                                 + w(8)(6)(3)(5)#1752 + w(9)(4)(3)(5)#1872
                                 + w(9)(5)(3)(5)#1902 + w(9)(6)(3)(5)#1932
                                 + w(10)(4)(3)(5)#2052 + w(10)(5)(3)(5)#2082
                                 + w(10)(6)(3)(5)#2112 + w(11)(4)(3)(5)#2232
                                 + w(11)(5)(3)(5)#2262 + w(11)(6)(3)(5)#2292
                                 + w(12)(4)(3)(5)#2412 + w(12)(5)(3)(5)#2442
                                 + w(12)(6)(3)(5)#2472  = 0
 WarehouseInventoryBalance(1,1,6): I_warehouse(1)(1)(6)#11 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(6)#683
                                 + w(3)(2)(1)(6)#713 + w(3)(3)(1)(6)#743
                                 + w(4)(1)(1)(6)#863 + w(4)(2)(1)(6)#893
                                 + w(4)(3)(1)(6)#923 + w(5)(1)(1)(6)#1043
                                 + w(5)(2)(1)(6)#1073 + w(5)(3)(1)(6)#1103
                                 + w(6)(1)(1)(6)#1223 + w(6)(2)(1)(6)#1253
                                 + w(6)(3)(1)(6)#1283 + w(7)(1)(1)(6)#1403
                                 + w(7)(2)(1)(6)#1433 + w(7)(3)(1)(6)#1463
                                 + w(8)(1)(1)(6)#1583 + w(8)(2)(1)(6)#1613
                                 + w(8)(3)(1)(6)#1643 + w(9)(1)(1)(6)#1763
                                 + w(9)(2)(1)(6)#1793 + w(9)(3)(1)(6)#1823
                                 + w(10)(1)(1)(6)#1943 + w(10)(2)(1)(6)#1973
                                 + w(10)(3)(1)(6)#2003 + w(11)(1)(1)(6)#2123
                                 + w(11)(2)(1)(6)#2153 + w(11)(3)(1)(6)#2183
                                 + w(12)(1)(1)(6)#2303 + w(12)(2)(1)(6)#2333
                                 + w(12)(3)(1)(6)#2363  = 0
 WarehouseInventoryBalance(2,1,6): I_warehouse(2)(1)(6)#41 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(6)#773
                                 + w(3)(5)(1)(6)#803 + w(3)(6)(1)(6)#833
                                 + w(4)(4)(1)(6)#953 + w(4)(5)(1)(6)#983
                                 + w(4)(6)(1)(6)#1013 + w(5)(4)(1)(6)#1133
                                 + w(5)(5)(1)(6)#1163 + w(5)(6)(1)(6)#1193
                                 + w(6)(4)(1)(6)#1313 + w(6)(5)(1)(6)#1343
                                 + w(6)(6)(1)(6)#1373 + w(7)(4)(1)(6)#1493
                                 + w(7)(5)(1)(6)#1523 + w(7)(6)(1)(6)#1553
                                 + w(8)(4)(1)(6)#1673 + w(8)(5)(1)(6)#1703
                                 + w(8)(6)(1)(6)#1733 + w(9)(4)(1)(6)#1853
                                 + w(9)(5)(1)(6)#1883 + w(9)(6)(1)(6)#1913
                                 + w(10)(4)(1)(6)#2033 + w(10)(5)(1)(6)#2063
                                 + w(10)(6)(1)(6)#2093 + w(11)(4)(1)(6)#2213
                                 + w(11)(5)(1)(6)#2243 + w(11)(6)(1)(6)#2273
                                 + w(12)(4)(1)(6)#2393 + w(12)(5)(1)(6)#2423
                                 + w(12)(6)(1)(6)#2453  = 0
 WarehouseInventoryBalance(1,2,6): - I_warehouse(1)(1)(6)#11
                                 + I_warehouse(1)(2)(6)#21 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(6)#693
                                 + w(3)(2)(2)(6)#723 + w(3)(3)(2)(6)#753
                                 + w(4)(1)(2)(6)#873 + w(4)(2)(2)(6)#903
                                 + w(4)(3)(2)(6)#933 + w(5)(1)(2)(6)#1053
                                 + w(5)(2)(2)(6)#1083 + w(5)(3)(2)(6)#1113
                                 + w(6)(1)(2)(6)#1233 + w(6)(2)(2)(6)#1263
                                 + w(6)(3)(2)(6)#1293 + w(7)(1)(2)(6)#1413
                                 + w(7)(2)(2)(6)#1443 + w(7)(3)(2)(6)#1473
                                 + w(8)(1)(2)(6)#1593 + w(8)(2)(2)(6)#1623
                                 + w(8)(3)(2)(6)#1653 + w(9)(1)(2)(6)#1773
                                 + w(9)(2)(2)(6)#1803 + w(9)(3)(2)(6)#1833
                                 + w(10)(1)(2)(6)#1953 + w(10)(2)(2)(6)#1983
                                 + w(10)(3)(2)(6)#2013 + w(11)(1)(2)(6)#2133
                                 + w(11)(2)(2)(6)#2163 + w(11)(3)(2)(6)#2193
                                 + w(12)(1)(2)(6)#2313 + w(12)(2)(2)(6)#2343
                                 + w(12)(3)(2)(6)#2373  = 0
 WarehouseInventoryBalance(2,2,6): - I_warehouse(2)(1)(6)#41
                                 + I_warehouse(2)(2)(6)#51 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(6)#783
                                 + w(3)(5)(2)(6)#813 + w(3)(6)(2)(6)#843
                                 + w(4)(4)(2)(6)#963 + w(4)(5)(2)(6)#993
                                 + w(4)(6)(2)(6)#1023 + w(5)(4)(2)(6)#1143
                                 + w(5)(5)(2)(6)#1173 + w(5)(6)(2)(6)#1203
                                 + w(6)(4)(2)(6)#1323 + w(6)(5)(2)(6)#1353
                                 + w(6)(6)(2)(6)#1383 + w(7)(4)(2)(6)#1503
                                 + w(7)(5)(2)(6)#1533 + w(7)(6)(2)(6)#1563
                                 + w(8)(4)(2)(6)#1683 + w(8)(5)(2)(6)#1713
                                 + w(8)(6)(2)(6)#1743 + w(9)(4)(2)(6)#1863
                                 + w(9)(5)(2)(6)#1893 + w(9)(6)(2)(6)#1923
                                 + w(10)(4)(2)(6)#2043 + w(10)(5)(2)(6)#2073
                                 + w(10)(6)(2)(6)#2103 + w(11)(4)(2)(6)#2223
                                 + w(11)(5)(2)(6)#2253 + w(11)(6)(2)(6)#2283
                                 + w(12)(4)(2)(6)#2403 + w(12)(5)(2)(6)#2433
                                 + w(12)(6)(2)(6)#2463  = 0
 WarehouseInventoryBalance(1,3,6): - I_warehouse(1)(2)(6)#21
                                 + I_warehouse(1)(3)(6)#31 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(6)#703
                                 + w(3)(2)(3)(6)#733 + w(3)(3)(3)(6)#763
                                 + w(4)(1)(3)(6)#883 + w(4)(2)(3)(6)#913
                                 + w(4)(3)(3)(6)#943 + w(5)(1)(3)(6)#1063
                                 + w(5)(2)(3)(6)#1093 + w(5)(3)(3)(6)#1123
                                 + w(6)(1)(3)(6)#1243 + w(6)(2)(3)(6)#1273
                                 + w(6)(3)(3)(6)#1303 + w(7)(1)(3)(6)#1423
                                 + w(7)(2)(3)(6)#1453 + w(7)(3)(3)(6)#1483
                                 + w(8)(1)(3)(6)#1603 + w(8)(2)(3)(6)#1633
                                 + w(8)(3)(3)(6)#1663 + w(9)(1)(3)(6)#1783
                                 + w(9)(2)(3)(6)#1813 + w(9)(3)(3)(6)#1843
                                 + w(10)(1)(3)(6)#1963 + w(10)(2)(3)(6)#1993
                                 + w(10)(3)(3)(6)#2023 + w(11)(1)(3)(6)#2143
                                 + w(11)(2)(3)(6)#2173 + w(11)(3)(3)(6)#2203
                                 + w(12)(1)(3)(6)#2323 + w(12)(2)(3)(6)#2353
                                 + w(12)(3)(3)(6)#2383  = 0
 WarehouseInventoryBalance(2,3,6): - I_warehouse(2)(2)(6)#51
                                 + I_warehouse(2)(3)(6)#61 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(6)#793
                                 + w(3)(5)(3)(6)#823 + w(3)(6)(3)(6)#853
                                 + w(4)(4)(3)(6)#973 + w(4)(5)(3)(6)#1003
                                 + w(4)(6)(3)(6)#1033 + w(5)(4)(3)(6)#1153
                                 + w(5)(5)(3)(6)#1183 + w(5)(6)(3)(6)#1213
                                 + w(6)(4)(3)(6)#1333 + w(6)(5)(3)(6)#1363
                                 + w(6)(6)(3)(6)#1393 + w(7)(4)(3)(6)#1513
                                 + w(7)(5)(3)(6)#1543 + w(7)(6)(3)(6)#1573
                                 + w(8)(4)(3)(6)#1693 + w(8)(5)(3)(6)#1723
                                 + w(8)(6)(3)(6)#1753 + w(9)(4)(3)(6)#1873
                                 + w(9)(5)(3)(6)#1903 + w(9)(6)(3)(6)#1933
                                 + w(10)(4)(3)(6)#2053 + w(10)(5)(3)(6)#2083
                                 + w(10)(6)(3)(6)#2113 + w(11)(4)(3)(6)#2233
                                 + w(11)(5)(3)(6)#2263 + w(11)(6)(3)(6)#2293
                                 + w(12)(4)(3)(6)#2413 + w(12)(5)(3)(6)#2443
                                 + w(12)(6)(3)(6)#2473  = 0
 WarehouseInventoryBalance(1,1,7): I_warehouse(1)(1)(7)#12 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(7)#684
                                 + w(3)(2)(1)(7)#714 + w(3)(3)(1)(7)#744
                                 + w(4)(1)(1)(7)#864 + w(4)(2)(1)(7)#894
                                 + w(4)(3)(1)(7)#924 + w(5)(1)(1)(7)#1044
                                 + w(5)(2)(1)(7)#1074 + w(5)(3)(1)(7)#1104
                                 + w(6)(1)(1)(7)#1224 + w(6)(2)(1)(7)#1254
                                 + w(6)(3)(1)(7)#1284 + w(7)(1)(1)(7)#1404
                                 + w(7)(2)(1)(7)#1434 + w(7)(3)(1)(7)#1464
                                 + w(8)(1)(1)(7)#1584 + w(8)(2)(1)(7)#1614
                                 + w(8)(3)(1)(7)#1644 + w(9)(1)(1)(7)#1764
                                 + w(9)(2)(1)(7)#1794 + w(9)(3)(1)(7)#1824
                                 + w(10)(1)(1)(7)#1944 + w(10)(2)(1)(7)#1974
                                 + w(10)(3)(1)(7)#2004 + w(11)(1)(1)(7)#2124
                                 + w(11)(2)(1)(7)#2154 + w(11)(3)(1)(7)#2184
                                 + w(12)(1)(1)(7)#2304 + w(12)(2)(1)(7)#2334
                                 + w(12)(3)(1)(7)#2364  = 0
 WarehouseInventoryBalance(2,1,7): I_warehouse(2)(1)(7)#42 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(7)#774
                                 + w(3)(5)(1)(7)#804 + w(3)(6)(1)(7)#834
                                 + w(4)(4)(1)(7)#954 + w(4)(5)(1)(7)#984
                                 + w(4)(6)(1)(7)#1014 + w(5)(4)(1)(7)#1134
                                 + w(5)(5)(1)(7)#1164 + w(5)(6)(1)(7)#1194
                                 + w(6)(4)(1)(7)#1314 + w(6)(5)(1)(7)#1344
                                 + w(6)(6)(1)(7)#1374 + w(7)(4)(1)(7)#1494
                                 + w(7)(5)(1)(7)#1524 + w(7)(6)(1)(7)#1554
                                 + w(8)(4)(1)(7)#1674 + w(8)(5)(1)(7)#1704
                                 + w(8)(6)(1)(7)#1734 + w(9)(4)(1)(7)#1854
                                 + w(9)(5)(1)(7)#1884 + w(9)(6)(1)(7)#1914
                                 + w(10)(4)(1)(7)#2034 + w(10)(5)(1)(7)#2064
                                 + w(10)(6)(1)(7)#2094 + w(11)(4)(1)(7)#2214
                                 + w(11)(5)(1)(7)#2244 + w(11)(6)(1)(7)#2274
                                 + w(12)(4)(1)(7)#2394 + w(12)(5)(1)(7)#2424
                                 + w(12)(6)(1)(7)#2454  = 0
 WarehouseInventoryBalance(1,2,7): - I_warehouse(1)(1)(7)#12
                                 + I_warehouse(1)(2)(7)#22 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(7)#694
                                 + w(3)(2)(2)(7)#724 + w(3)(3)(2)(7)#754
                                 + w(4)(1)(2)(7)#874 + w(4)(2)(2)(7)#904
                                 + w(4)(3)(2)(7)#934 + w(5)(1)(2)(7)#1054
                                 + w(5)(2)(2)(7)#1084 + w(5)(3)(2)(7)#1114
                                 + w(6)(1)(2)(7)#1234 + w(6)(2)(2)(7)#1264
                                 + w(6)(3)(2)(7)#1294 + w(7)(1)(2)(7)#1414
                                 + w(7)(2)(2)(7)#1444 + w(7)(3)(2)(7)#1474
                                 + w(8)(1)(2)(7)#1594 + w(8)(2)(2)(7)#1624
                                 + w(8)(3)(2)(7)#1654 + w(9)(1)(2)(7)#1774
                                 + w(9)(2)(2)(7)#1804 + w(9)(3)(2)(7)#1834
                                 + w(10)(1)(2)(7)#1954 + w(10)(2)(2)(7)#1984
                                 + w(10)(3)(2)(7)#2014 + w(11)(1)(2)(7)#2134
                                 + w(11)(2)(2)(7)#2164 + w(11)(3)(2)(7)#2194
                                 + w(12)(1)(2)(7)#2314 + w(12)(2)(2)(7)#2344
                                 + w(12)(3)(2)(7)#2374  = 0
 WarehouseInventoryBalance(2,2,7): - I_warehouse(2)(1)(7)#42
                                 + I_warehouse(2)(2)(7)#52 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(7)#784
                                 + w(3)(5)(2)(7)#814 + w(3)(6)(2)(7)#844
                                 + w(4)(4)(2)(7)#964 + w(4)(5)(2)(7)#994
                                 + w(4)(6)(2)(7)#1024 + w(5)(4)(2)(7)#1144
                                 + w(5)(5)(2)(7)#1174 + w(5)(6)(2)(7)#1204
                                 + w(6)(4)(2)(7)#1324 + w(6)(5)(2)(7)#1354
                                 + w(6)(6)(2)(7)#1384 + w(7)(4)(2)(7)#1504
                                 + w(7)(5)(2)(7)#1534 + w(7)(6)(2)(7)#1564
                                 + w(8)(4)(2)(7)#1684 + w(8)(5)(2)(7)#1714
                                 + w(8)(6)(2)(7)#1744 + w(9)(4)(2)(7)#1864
                                 + w(9)(5)(2)(7)#1894 + w(9)(6)(2)(7)#1924
                                 + w(10)(4)(2)(7)#2044 + w(10)(5)(2)(7)#2074
                                 + w(10)(6)(2)(7)#2104 + w(11)(4)(2)(7)#2224
                                 + w(11)(5)(2)(7)#2254 + w(11)(6)(2)(7)#2284
                                 + w(12)(4)(2)(7)#2404 + w(12)(5)(2)(7)#2434
                                 + w(12)(6)(2)(7)#2464  = 0
 WarehouseInventoryBalance(1,3,7): - I_warehouse(1)(2)(7)#22
                                 + I_warehouse(1)(3)(7)#32 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(7)#704
                                 + w(3)(2)(3)(7)#734 + w(3)(3)(3)(7)#764
                                 + w(4)(1)(3)(7)#884 + w(4)(2)(3)(7)#914
                                 + w(4)(3)(3)(7)#944 + w(5)(1)(3)(7)#1064
                                 + w(5)(2)(3)(7)#1094 + w(5)(3)(3)(7)#1124
                                 + w(6)(1)(3)(7)#1244 + w(6)(2)(3)(7)#1274
                                 + w(6)(3)(3)(7)#1304 + w(7)(1)(3)(7)#1424
                                 + w(7)(2)(3)(7)#1454 + w(7)(3)(3)(7)#1484
                                 + w(8)(1)(3)(7)#1604 + w(8)(2)(3)(7)#1634
                                 + w(8)(3)(3)(7)#1664 + w(9)(1)(3)(7)#1784
                                 + w(9)(2)(3)(7)#1814 + w(9)(3)(3)(7)#1844
                                 + w(10)(1)(3)(7)#1964 + w(10)(2)(3)(7)#1994
                                 + w(10)(3)(3)(7)#2024 + w(11)(1)(3)(7)#2144
                                 + w(11)(2)(3)(7)#2174 + w(11)(3)(3)(7)#2204
                                 + w(12)(1)(3)(7)#2324 + w(12)(2)(3)(7)#2354
                                 + w(12)(3)(3)(7)#2384  = 0
 WarehouseInventoryBalance(2,3,7): - I_warehouse(2)(2)(7)#52
                                 + I_warehouse(2)(3)(7)#62 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(7)#794
                                 + w(3)(5)(3)(7)#824 + w(3)(6)(3)(7)#854
                                 + w(4)(4)(3)(7)#974 + w(4)(5)(3)(7)#1004
                                 + w(4)(6)(3)(7)#1034 + w(5)(4)(3)(7)#1154
                                 + w(5)(5)(3)(7)#1184 + w(5)(6)(3)(7)#1214
                                 + w(6)(4)(3)(7)#1334 + w(6)(5)(3)(7)#1364
                                 + w(6)(6)(3)(7)#1394 + w(7)(4)(3)(7)#1514
                                 + w(7)(5)(3)(7)#1544 + w(7)(6)(3)(7)#1574
                                 + w(8)(4)(3)(7)#1694 + w(8)(5)(3)(7)#1724
                                 + w(8)(6)(3)(7)#1754 + w(9)(4)(3)(7)#1874
                                 + w(9)(5)(3)(7)#1904 + w(9)(6)(3)(7)#1934
                                 + w(10)(4)(3)(7)#2054 + w(10)(5)(3)(7)#2084
                                 + w(10)(6)(3)(7)#2114 + w(11)(4)(3)(7)#2234
                                 + w(11)(5)(3)(7)#2264 + w(11)(6)(3)(7)#2294
                                 + w(12)(4)(3)(7)#2414 + w(12)(5)(3)(7)#2444
                                 + w(12)(6)(3)(7)#2474  = 0
 WarehouseInventoryBalance(1,1,8): I_warehouse(1)(1)(8)#13 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(8)#685
                                 + w(3)(2)(1)(8)#715 + w(3)(3)(1)(8)#745
                                 + w(4)(1)(1)(8)#865 + w(4)(2)(1)(8)#895
                                 + w(4)(3)(1)(8)#925 + w(5)(1)(1)(8)#1045
                                 + w(5)(2)(1)(8)#1075 + w(5)(3)(1)(8)#1105
                                 + w(6)(1)(1)(8)#1225 + w(6)(2)(1)(8)#1255
                                 + w(6)(3)(1)(8)#1285 + w(7)(1)(1)(8)#1405
                                 + w(7)(2)(1)(8)#1435 + w(7)(3)(1)(8)#1465
                                 + w(8)(1)(1)(8)#1585 + w(8)(2)(1)(8)#1615
                                 + w(8)(3)(1)(8)#1645 + w(9)(1)(1)(8)#1765
                                 + w(9)(2)(1)(8)#1795 + w(9)(3)(1)(8)#1825
                                 + w(10)(1)(1)(8)#1945 + w(10)(2)(1)(8)#1975
                                 + w(10)(3)(1)(8)#2005 + w(11)(1)(1)(8)#2125
                                 + w(11)(2)(1)(8)#2155 + w(11)(3)(1)(8)#2185
                                 + w(12)(1)(1)(8)#2305 + w(12)(2)(1)(8)#2335
                                 + w(12)(3)(1)(8)#2365  = 0
 WarehouseInventoryBalance(2,1,8): I_warehouse(2)(1)(8)#43 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(8)#775
                                 + w(3)(5)(1)(8)#805 + w(3)(6)(1)(8)#835
                                 + w(4)(4)(1)(8)#955 + w(4)(5)(1)(8)#985
                                 + w(4)(6)(1)(8)#1015 + w(5)(4)(1)(8)#1135
                                 + w(5)(5)(1)(8)#1165 + w(5)(6)(1)(8)#1195
                                 + w(6)(4)(1)(8)#1315 + w(6)(5)(1)(8)#1345
                                 + w(6)(6)(1)(8)#1375 + w(7)(4)(1)(8)#1495
                                 + w(7)(5)(1)(8)#1525 + w(7)(6)(1)(8)#1555
                                 + w(8)(4)(1)(8)#1675 + w(8)(5)(1)(8)#1705
                                 + w(8)(6)(1)(8)#1735 + w(9)(4)(1)(8)#1855
                                 + w(9)(5)(1)(8)#1885 + w(9)(6)(1)(8)#1915
                                 + w(10)(4)(1)(8)#2035 + w(10)(5)(1)(8)#2065
                                 + w(10)(6)(1)(8)#2095 + w(11)(4)(1)(8)#2215
                                 + w(11)(5)(1)(8)#2245 + w(11)(6)(1)(8)#2275
                                 + w(12)(4)(1)(8)#2395 + w(12)(5)(1)(8)#2425
                                 + w(12)(6)(1)(8)#2455  = 0
 WarehouseInventoryBalance(1,2,8): - I_warehouse(1)(1)(8)#13
                                 + I_warehouse(1)(2)(8)#23 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(8)#695
                                 + w(3)(2)(2)(8)#725 + w(3)(3)(2)(8)#755
                                 + w(4)(1)(2)(8)#875 + w(4)(2)(2)(8)#905
                                 + w(4)(3)(2)(8)#935 + w(5)(1)(2)(8)#1055
                                 + w(5)(2)(2)(8)#1085 + w(5)(3)(2)(8)#1115
                                 + w(6)(1)(2)(8)#1235 + w(6)(2)(2)(8)#1265
                                 + w(6)(3)(2)(8)#1295 + w(7)(1)(2)(8)#1415
                                 + w(7)(2)(2)(8)#1445 + w(7)(3)(2)(8)#1475
                                 + w(8)(1)(2)(8)#1595 + w(8)(2)(2)(8)#1625
                                 + w(8)(3)(2)(8)#1655 + w(9)(1)(2)(8)#1775
                                 + w(9)(2)(2)(8)#1805 + w(9)(3)(2)(8)#1835
                                 + w(10)(1)(2)(8)#1955 + w(10)(2)(2)(8)#1985
                                 + w(10)(3)(2)(8)#2015 + w(11)(1)(2)(8)#2135
                                 + w(11)(2)(2)(8)#2165 + w(11)(3)(2)(8)#2195
                                 + w(12)(1)(2)(8)#2315 + w(12)(2)(2)(8)#2345
                                 + w(12)(3)(2)(8)#2375  = 0
 WarehouseInventoryBalance(2,2,8): - I_warehouse(2)(1)(8)#43
                                 + I_warehouse(2)(2)(8)#53 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(8)#785
                                 + w(3)(5)(2)(8)#815 + w(3)(6)(2)(8)#845
                                 + w(4)(4)(2)(8)#965 + w(4)(5)(2)(8)#995
                                 + w(4)(6)(2)(8)#1025 + w(5)(4)(2)(8)#1145
                                 + w(5)(5)(2)(8)#1175 + w(5)(6)(2)(8)#1205
                                 + w(6)(4)(2)(8)#1325 + w(6)(5)(2)(8)#1355
                                 + w(6)(6)(2)(8)#1385 + w(7)(4)(2)(8)#1505
                                 + w(7)(5)(2)(8)#1535 + w(7)(6)(2)(8)#1565
                                 + w(8)(4)(2)(8)#1685 + w(8)(5)(2)(8)#1715
                                 + w(8)(6)(2)(8)#1745 + w(9)(4)(2)(8)#1865
                                 + w(9)(5)(2)(8)#1895 + w(9)(6)(2)(8)#1925
                                 + w(10)(4)(2)(8)#2045 + w(10)(5)(2)(8)#2075
                                 + w(10)(6)(2)(8)#2105 + w(11)(4)(2)(8)#2225
                                 + w(11)(5)(2)(8)#2255 + w(11)(6)(2)(8)#2285
                                 + w(12)(4)(2)(8)#2405 + w(12)(5)(2)(8)#2435
                                 + w(12)(6)(2)(8)#2465  = 0
 WarehouseInventoryBalance(1,3,8): - I_warehouse(1)(2)(8)#23
                                 + I_warehouse(1)(3)(8)#33 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(8)#705
                                 + w(3)(2)(3)(8)#735 + w(3)(3)(3)(8)#765
                                 + w(4)(1)(3)(8)#885 + w(4)(2)(3)(8)#915
                                 + w(4)(3)(3)(8)#945 + w(5)(1)(3)(8)#1065
                                 + w(5)(2)(3)(8)#1095 + w(5)(3)(3)(8)#1125
                                 + w(6)(1)(3)(8)#1245 + w(6)(2)(3)(8)#1275
                                 + w(6)(3)(3)(8)#1305 + w(7)(1)(3)(8)#1425
                                 + w(7)(2)(3)(8)#1455 + w(7)(3)(3)(8)#1485
                                 + w(8)(1)(3)(8)#1605 + w(8)(2)(3)(8)#1635
                                 + w(8)(3)(3)(8)#1665 + w(9)(1)(3)(8)#1785
                                 + w(9)(2)(3)(8)#1815 + w(9)(3)(3)(8)#1845
                                 + w(10)(1)(3)(8)#1965 + w(10)(2)(3)(8)#1995
                                 + w(10)(3)(3)(8)#2025 + w(11)(1)(3)(8)#2145
                                 + w(11)(2)(3)(8)#2175 + w(11)(3)(3)(8)#2205
                                 + w(12)(1)(3)(8)#2325 + w(12)(2)(3)(8)#2355
                                 + w(12)(3)(3)(8)#2385  = 0
 WarehouseInventoryBalance(2,3,8): - I_warehouse(2)(2)(8)#53
                                 + I_warehouse(2)(3)(8)#63 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(8)#795
                                 + w(3)(5)(3)(8)#825 + w(3)(6)(3)(8)#855
                                 + w(4)(4)(3)(8)#975 + w(4)(5)(3)(8)#1005
                                 + w(4)(6)(3)(8)#1035 + w(5)(4)(3)(8)#1155
                                 + w(5)(5)(3)(8)#1185 + w(5)(6)(3)(8)#1215
                                 + w(6)(4)(3)(8)#1335 + w(6)(5)(3)(8)#1365
                                 + w(6)(6)(3)(8)#1395 + w(7)(4)(3)(8)#1515
                                 + w(7)(5)(3)(8)#1545 + w(7)(6)(3)(8)#1575
                                 + w(8)(4)(3)(8)#1695 + w(8)(5)(3)(8)#1725
                                 + w(8)(6)(3)(8)#1755 + w(9)(4)(3)(8)#1875
                                 + w(9)(5)(3)(8)#1905 + w(9)(6)(3)(8)#1935
                                 + w(10)(4)(3)(8)#2055 + w(10)(5)(3)(8)#2085
                                 + w(10)(6)(3)(8)#2115 + w(11)(4)(3)(8)#2235
                                 + w(11)(5)(3)(8)#2265 + w(11)(6)(3)(8)#2295
                                 + w(12)(4)(3)(8)#2415 + w(12)(5)(3)(8)#2445
                                 + w(12)(6)(3)(8)#2475  = 0
 WarehouseInventoryBalance(1,1,9): I_warehouse(1)(1)(9)#14 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(9)#686
                                 + w(3)(2)(1)(9)#716 + w(3)(3)(1)(9)#746
                                 + w(4)(1)(1)(9)#866 + w(4)(2)(1)(9)#896
                                 + w(4)(3)(1)(9)#926 + w(5)(1)(1)(9)#1046
                                 + w(5)(2)(1)(9)#1076 + w(5)(3)(1)(9)#1106
                                 + w(6)(1)(1)(9)#1226 + w(6)(2)(1)(9)#1256
                                 + w(6)(3)(1)(9)#1286 + w(7)(1)(1)(9)#1406
                                 + w(7)(2)(1)(9)#1436 + w(7)(3)(1)(9)#1466
                                 + w(8)(1)(1)(9)#1586 + w(8)(2)(1)(9)#1616
                                 + w(8)(3)(1)(9)#1646 + w(9)(1)(1)(9)#1766
                                 + w(9)(2)(1)(9)#1796 + w(9)(3)(1)(9)#1826
                                 + w(10)(1)(1)(9)#1946 + w(10)(2)(1)(9)#1976
                                 + w(10)(3)(1)(9)#2006 + w(11)(1)(1)(9)#2126
                                 + w(11)(2)(1)(9)#2156 + w(11)(3)(1)(9)#2186
                                 + w(12)(1)(1)(9)#2306 + w(12)(2)(1)(9)#2336
                                 + w(12)(3)(1)(9)#2366  = 0
 WarehouseInventoryBalance(2,1,9): I_warehouse(2)(1)(9)#44 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(9)#776
                                 + w(3)(5)(1)(9)#806 + w(3)(6)(1)(9)#836
                                 + w(4)(4)(1)(9)#956 + w(4)(5)(1)(9)#986
                                 + w(4)(6)(1)(9)#1016 + w(5)(4)(1)(9)#1136
                                 + w(5)(5)(1)(9)#1166 + w(5)(6)(1)(9)#1196
                                 + w(6)(4)(1)(9)#1316 + w(6)(5)(1)(9)#1346
                                 + w(6)(6)(1)(9)#1376 + w(7)(4)(1)(9)#1496
                                 + w(7)(5)(1)(9)#1526 + w(7)(6)(1)(9)#1556
                                 + w(8)(4)(1)(9)#1676 + w(8)(5)(1)(9)#1706
                                 + w(8)(6)(1)(9)#1736 + w(9)(4)(1)(9)#1856
                                 + w(9)(5)(1)(9)#1886 + w(9)(6)(1)(9)#1916
                                 + w(10)(4)(1)(9)#2036 + w(10)(5)(1)(9)#2066
                                 + w(10)(6)(1)(9)#2096 + w(11)(4)(1)(9)#2216
                                 + w(11)(5)(1)(9)#2246 + w(11)(6)(1)(9)#2276
                                 + w(12)(4)(1)(9)#2396 + w(12)(5)(1)(9)#2426
                                 + w(12)(6)(1)(9)#2456  = 0
 WarehouseInventoryBalance(1,2,9): - I_warehouse(1)(1)(9)#14
                                 + I_warehouse(1)(2)(9)#24 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(9)#696
                                 + w(3)(2)(2)(9)#726 + w(3)(3)(2)(9)#756
                                 + w(4)(1)(2)(9)#876 + w(4)(2)(2)(9)#906
                                 + w(4)(3)(2)(9)#936 + w(5)(1)(2)(9)#1056
                                 + w(5)(2)(2)(9)#1086 + w(5)(3)(2)(9)#1116
                                 + w(6)(1)(2)(9)#1236 + w(6)(2)(2)(9)#1266
                                 + w(6)(3)(2)(9)#1296 + w(7)(1)(2)(9)#1416
                                 + w(7)(2)(2)(9)#1446 + w(7)(3)(2)(9)#1476
                                 + w(8)(1)(2)(9)#1596 + w(8)(2)(2)(9)#1626
                                 + w(8)(3)(2)(9)#1656 + w(9)(1)(2)(9)#1776
                                 + w(9)(2)(2)(9)#1806 + w(9)(3)(2)(9)#1836
                                 + w(10)(1)(2)(9)#1956 + w(10)(2)(2)(9)#1986
                                 + w(10)(3)(2)(9)#2016 + w(11)(1)(2)(9)#2136
                                 + w(11)(2)(2)(9)#2166 + w(11)(3)(2)(9)#2196
                                 + w(12)(1)(2)(9)#2316 + w(12)(2)(2)(9)#2346
                                 + w(12)(3)(2)(9)#2376  = 0
 WarehouseInventoryBalance(2,2,9): - I_warehouse(2)(1)(9)#44
                                 + I_warehouse(2)(2)(9)#54 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(9)#786
                                 + w(3)(5)(2)(9)#816 + w(3)(6)(2)(9)#846
                                 + w(4)(4)(2)(9)#966 + w(4)(5)(2)(9)#996
                                 + w(4)(6)(2)(9)#1026 + w(5)(4)(2)(9)#1146
                                 + w(5)(5)(2)(9)#1176 + w(5)(6)(2)(9)#1206
                                 + w(6)(4)(2)(9)#1326 + w(6)(5)(2)(9)#1356
                                 + w(6)(6)(2)(9)#1386 + w(7)(4)(2)(9)#1506
                                 + w(7)(5)(2)(9)#1536 + w(7)(6)(2)(9)#1566
                                 + w(8)(4)(2)(9)#1686 + w(8)(5)(2)(9)#1716
                                 + w(8)(6)(2)(9)#1746 + w(9)(4)(2)(9)#1866
                                 + w(9)(5)(2)(9)#1896 + w(9)(6)(2)(9)#1926
                                 + w(10)(4)(2)(9)#2046 + w(10)(5)(2)(9)#2076
                                 + w(10)(6)(2)(9)#2106 + w(11)(4)(2)(9)#2226
                                 + w(11)(5)(2)(9)#2256 + w(11)(6)(2)(9)#2286
                                 + w(12)(4)(2)(9)#2406 + w(12)(5)(2)(9)#2436
                                 + w(12)(6)(2)(9)#2466  = 0
 WarehouseInventoryBalance(1,3,9): - I_warehouse(1)(2)(9)#24
                                 + I_warehouse(1)(3)(9)#34 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(9)#706
                                 + w(3)(2)(3)(9)#736 + w(3)(3)(3)(9)#766
                                 + w(4)(1)(3)(9)#886 + w(4)(2)(3)(9)#916
                                 + w(4)(3)(3)(9)#946 + w(5)(1)(3)(9)#1066
                                 + w(5)(2)(3)(9)#1096 + w(5)(3)(3)(9)#1126
                                 + w(6)(1)(3)(9)#1246 + w(6)(2)(3)(9)#1276
                                 + w(6)(3)(3)(9)#1306 + w(7)(1)(3)(9)#1426
                                 + w(7)(2)(3)(9)#1456 + w(7)(3)(3)(9)#1486
                                 + w(8)(1)(3)(9)#1606 + w(8)(2)(3)(9)#1636
                                 + w(8)(3)(3)(9)#1666 + w(9)(1)(3)(9)#1786
                                 + w(9)(2)(3)(9)#1816 + w(9)(3)(3)(9)#1846
                                 + w(10)(1)(3)(9)#1966 + w(10)(2)(3)(9)#1996
                                 + w(10)(3)(3)(9)#2026 + w(11)(1)(3)(9)#2146
                                 + w(11)(2)(3)(9)#2176 + w(11)(3)(3)(9)#2206
                                 + w(12)(1)(3)(9)#2326 + w(12)(2)(3)(9)#2356
                                 + w(12)(3)(3)(9)#2386  = 0
 WarehouseInventoryBalance(2,3,9): - I_warehouse(2)(2)(9)#54
                                 + I_warehouse(2)(3)(9)#64 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(9)#796
                                 + w(3)(5)(3)(9)#826 + w(3)(6)(3)(9)#856
                                 + w(4)(4)(3)(9)#976 + w(4)(5)(3)(9)#1006
                                 + w(4)(6)(3)(9)#1036 + w(5)(4)(3)(9)#1156
                                 + w(5)(5)(3)(9)#1186 + w(5)(6)(3)(9)#1216
                                 + w(6)(4)(3)(9)#1336 + w(6)(5)(3)(9)#1366
                                 + w(6)(6)(3)(9)#1396 + w(7)(4)(3)(9)#1516
                                 + w(7)(5)(3)(9)#1546 + w(7)(6)(3)(9)#1576
                                 + w(8)(4)(3)(9)#1696 + w(8)(5)(3)(9)#1726
                                 + w(8)(6)(3)(9)#1756 + w(9)(4)(3)(9)#1876
                                 + w(9)(5)(3)(9)#1906 + w(9)(6)(3)(9)#1936
                                 + w(10)(4)(3)(9)#2056 + w(10)(5)(3)(9)#2086
                                 + w(10)(6)(3)(9)#2116 + w(11)(4)(3)(9)#2236
                                 + w(11)(5)(3)(9)#2266 + w(11)(6)(3)(9)#2296
                                 + w(12)(4)(3)(9)#2416 + w(12)(5)(3)(9)#2446
                                 + w(12)(6)(3)(9)#2476  = 0
 WarehouseInventoryBalance(1,1,10): I_warehouse(1)(1)(10)#15 - q(1)(1)(1)#666
                                 - q(1)(2)(1)#669 + w(3)(1)(1)(10)#687
                                 + w(3)(2)(1)(10)#717 + w(3)(3)(1)(10)#747
                                 + w(4)(1)(1)(10)#867 + w(4)(2)(1)(10)#897
                                 + w(4)(3)(1)(10)#927 + w(5)(1)(1)(10)#1047
                                 + w(5)(2)(1)(10)#1077 + w(5)(3)(1)(10)#1107
                                 + w(6)(1)(1)(10)#1227 + w(6)(2)(1)(10)#1257
                                 + w(6)(3)(1)(10)#1287 + w(7)(1)(1)(10)#1407
                                 + w(7)(2)(1)(10)#1437 + w(7)(3)(1)(10)#1467
                                 + w(8)(1)(1)(10)#1587 + w(8)(2)(1)(10)#1617
                                 + w(8)(3)(1)(10)#1647 + w(9)(1)(1)(10)#1767
                                 + w(9)(2)(1)(10)#1797 + w(9)(3)(1)(10)#1827
                                 + w(10)(1)(1)(10)#1947 + w(10)(2)(1)(10)#1977
                                 + w(10)(3)(1)(10)#2007 + w(11)(1)(1)(10)#2127
                                 + w(11)(2)(1)(10)#2157 + w(11)(3)(1)(10)#2187
                                 + w(12)(1)(1)(10)#2307 + w(12)(2)(1)(10)#2337
                                 + w(12)(3)(1)(10)#2367  = 0
 WarehouseInventoryBalance(2,1,10): I_warehouse(2)(1)(10)#45 - q(2)(1)(1)#672
                                 - q(2)(2)(1)#675 + w(3)(4)(1)(10)#777
                                 + w(3)(5)(1)(10)#807 + w(3)(6)(1)(10)#837
                                 + w(4)(4)(1)(10)#957 + w(4)(5)(1)(10)#987
                                 + w(4)(6)(1)(10)#1017 + w(5)(4)(1)(10)#1137
                                 + w(5)(5)(1)(10)#1167 + w(5)(6)(1)(10)#1197
                                 + w(6)(4)(1)(10)#1317 + w(6)(5)(1)(10)#1347
                                 + w(6)(6)(1)(10)#1377 + w(7)(4)(1)(10)#1497
                                 + w(7)(5)(1)(10)#1527 + w(7)(6)(1)(10)#1557
                                 + w(8)(4)(1)(10)#1677 + w(8)(5)(1)(10)#1707
                                 + w(8)(6)(1)(10)#1737 + w(9)(4)(1)(10)#1857
                                 + w(9)(5)(1)(10)#1887 + w(9)(6)(1)(10)#1917
                                 + w(10)(4)(1)(10)#2037 + w(10)(5)(1)(10)#2067
                                 + w(10)(6)(1)(10)#2097 + w(11)(4)(1)(10)#2217
                                 + w(11)(5)(1)(10)#2247 + w(11)(6)(1)(10)#2277
                                 + w(12)(4)(1)(10)#2397 + w(12)(5)(1)(10)#2427
                                 + w(12)(6)(1)(10)#2457  = 0
 WarehouseInventoryBalance(1,2,10): - I_warehouse(1)(1)(10)#15
                                 + I_warehouse(1)(2)(10)#25 - q(1)(1)(2)#667
                                 - q(1)(2)(2)#670 + w(3)(1)(2)(10)#697
                                 + w(3)(2)(2)(10)#727 + w(3)(3)(2)(10)#757
                                 + w(4)(1)(2)(10)#877 + w(4)(2)(2)(10)#907
                                 + w(4)(3)(2)(10)#937 + w(5)(1)(2)(10)#1057
                                 + w(5)(2)(2)(10)#1087 + w(5)(3)(2)(10)#1117
                                 + w(6)(1)(2)(10)#1237 + w(6)(2)(2)(10)#1267
                                 + w(6)(3)(2)(10)#1297 + w(7)(1)(2)(10)#1417
                                 + w(7)(2)(2)(10)#1447 + w(7)(3)(2)(10)#1477
                                 + w(8)(1)(2)(10)#1597 + w(8)(2)(2)(10)#1627
                                 + w(8)(3)(2)(10)#1657 + w(9)(1)(2)(10)#1777
                                 + w(9)(2)(2)(10)#1807 + w(9)(3)(2)(10)#1837
                                 + w(10)(1)(2)(10)#1957 + w(10)(2)(2)(10)#1987
                                 + w(10)(3)(2)(10)#2017 + w(11)(1)(2)(10)#2137
                                 + w(11)(2)(2)(10)#2167 + w(11)(3)(2)(10)#2197
                                 + w(12)(1)(2)(10)#2317 + w(12)(2)(2)(10)#2347
                                 + w(12)(3)(2)(10)#2377  = 0
 WarehouseInventoryBalance(2,2,10): - I_warehouse(2)(1)(10)#45
                                 + I_warehouse(2)(2)(10)#55 - q(2)(1)(2)#673
                                 - q(2)(2)(2)#676 + w(3)(4)(2)(10)#787
                                 + w(3)(5)(2)(10)#817 + w(3)(6)(2)(10)#847
                                 + w(4)(4)(2)(10)#967 + w(4)(5)(2)(10)#997
                                 + w(4)(6)(2)(10)#1027 + w(5)(4)(2)(10)#1147
                                 + w(5)(5)(2)(10)#1177 + w(5)(6)(2)(10)#1207
                                 + w(6)(4)(2)(10)#1327 + w(6)(5)(2)(10)#1357
                                 + w(6)(6)(2)(10)#1387 + w(7)(4)(2)(10)#1507
                                 + w(7)(5)(2)(10)#1537 + w(7)(6)(2)(10)#1567
                                 + w(8)(4)(2)(10)#1687 + w(8)(5)(2)(10)#1717
                                 + w(8)(6)(2)(10)#1747 + w(9)(4)(2)(10)#1867
                                 + w(9)(5)(2)(10)#1897 + w(9)(6)(2)(10)#1927
                                 + w(10)(4)(2)(10)#2047 + w(10)(5)(2)(10)#2077
                                 + w(10)(6)(2)(10)#2107 + w(11)(4)(2)(10)#2227
                                 + w(11)(5)(2)(10)#2257 + w(11)(6)(2)(10)#2287
                                 + w(12)(4)(2)(10)#2407 + w(12)(5)(2)(10)#2437
                                 + w(12)(6)(2)(10)#2467  = 0
 WarehouseInventoryBalance(1,3,10): - I_warehouse(1)(2)(10)#25
                                 + I_warehouse(1)(3)(10)#35 - q(1)(1)(3)#668
                                 - q(1)(2)(3)#671 + w(3)(1)(3)(10)#707
                                 + w(3)(2)(3)(10)#737 + w(3)(3)(3)(10)#767
                                 + w(4)(1)(3)(10)#887 + w(4)(2)(3)(10)#917
                                 + w(4)(3)(3)(10)#947 + w(5)(1)(3)(10)#1067
                                 + w(5)(2)(3)(10)#1097 + w(5)(3)(3)(10)#1127
                                 + w(6)(1)(3)(10)#1247 + w(6)(2)(3)(10)#1277
                                 + w(6)(3)(3)(10)#1307 + w(7)(1)(3)(10)#1427
                                 + w(7)(2)(3)(10)#1457 + w(7)(3)(3)(10)#1487
                                 + w(8)(1)(3)(10)#1607 + w(8)(2)(3)(10)#1637
                                 + w(8)(3)(3)(10)#1667 + w(9)(1)(3)(10)#1787
                                 + w(9)(2)(3)(10)#1817 + w(9)(3)(3)(10)#1847
                                 + w(10)(1)(3)(10)#1967 + w(10)(2)(3)(10)#1997
                                 + w(10)(3)(3)(10)#2027 + w(11)(1)(3)(10)#2147
                                 + w(11)(2)(3)(10)#2177 + w(11)(3)(3)(10)#2207
                                 + w(12)(1)(3)(10)#2327 + w(12)(2)(3)(10)#2357
                                 + w(12)(3)(3)(10)#2387  = 0
 WarehouseInventoryBalance(2,3,10): - I_warehouse(2)(2)(10)#55
                                 + I_warehouse(2)(3)(10)#65 - q(2)(1)(3)#674
                                 - q(2)(2)(3)#677 + w(3)(4)(3)(10)#797
                                 + w(3)(5)(3)(10)#827 + w(3)(6)(3)(10)#857
                                 + w(4)(4)(3)(10)#977 + w(4)(5)(3)(10)#1007
                                 + w(4)(6)(3)(10)#1037 + w(5)(4)(3)(10)#1157
                                 + w(5)(5)(3)(10)#1187 + w(5)(6)(3)(10)#1217
                                 + w(6)(4)(3)(10)#1337 + w(6)(5)(3)(10)#1367
                                 + w(6)(6)(3)(10)#1397 + w(7)(4)(3)(10)#1517
                                 + w(7)(5)(3)(10)#1547 + w(7)(6)(3)(10)#1577
                                 + w(8)(4)(3)(10)#1697 + w(8)(5)(3)(10)#1727
                                 + w(8)(6)(3)(10)#1757 + w(9)(4)(3)(10)#1877
                                 + w(9)(5)(3)(10)#1907 + w(9)(6)(3)(10)#1937
                                 + w(10)(4)(3)(10)#2057 + w(10)(5)(3)(10)#2087
                                 + w(10)(6)(3)(10)#2117 + w(11)(4)(3)(10)#2237
                                 + w(11)(5)(3)(10)#2267 + w(11)(6)(3)(10)#2297
                                 + w(12)(4)(3)(10)#2417 + w(12)(5)(3)(10)#2447
                                 + w(12)(6)(3)(10)#2477  = 0
 CustomerInventoryBalance(3,1,1): I_customer(3)(1)(1)#66
                                 - b_customer(3)(1)(1)#366 - w(3)(1)(1)(1)#678
                                 - w(3)(2)(1)(1)#708 - w(3)(3)(1)(1)#738
                                 - w(3)(4)(1)(1)#768 - w(3)(5)(1)(1)#798
                                 - w(3)(6)(1)(1)#828  = -5
 CustomerInventoryBalance(4,1,1): I_customer(4)(1)(1)#96
                                 - b_customer(4)(1)(1)#396 - w(4)(1)(1)(1)#858
                                 - w(4)(2)(1)(1)#888 - w(4)(3)(1)(1)#918
                                 - w(4)(4)(1)(1)#948 - w(4)(5)(1)(1)#978
                                 - w(4)(6)(1)(1)#1008  = -3
 CustomerInventoryBalance(5,1,1): I_customer(5)(1)(1)#126
                                 - b_customer(5)(1)(1)#426 - w(5)(1)(1)(1)#1038
                                 - w(5)(2)(1)(1)#1068 - w(5)(3)(1)(1)#1098
                                 - w(5)(4)(1)(1)#1128 - w(5)(5)(1)(1)#1158
                                 - w(5)(6)(1)(1)#1188  = 2
 CustomerInventoryBalance(6,1,1): I_customer(6)(1)(1)#156
                                 - b_customer(6)(1)(1)#456 - w(6)(1)(1)(1)#1218
                                 - w(6)(2)(1)(1)#1248 - w(6)(3)(1)(1)#1278
                                 - w(6)(4)(1)(1)#1308 - w(6)(5)(1)(1)#1338
                                 - w(6)(6)(1)(1)#1368  = -3
 CustomerInventoryBalance(7,1,1): I_customer(7)(1)(1)#186
                                 - b_customer(7)(1)(1)#486 - w(7)(1)(1)(1)#1398
                                 - w(7)(2)(1)(1)#1428 - w(7)(3)(1)(1)#1458
                                 - w(7)(4)(1)(1)#1488 - w(7)(5)(1)(1)#1518
                                 - w(7)(6)(1)(1)#1548  = -2
 CustomerInventoryBalance(8,1,1): I_customer(8)(1)(1)#216
                                 - b_customer(8)(1)(1)#516 - w(8)(1)(1)(1)#1578
                                 - w(8)(2)(1)(1)#1608 - w(8)(3)(1)(1)#1638
                                 - w(8)(4)(1)(1)#1668 - w(8)(5)(1)(1)#1698
                                 - w(8)(6)(1)(1)#1728  = 27
 CustomerInventoryBalance(9,1,1): I_customer(9)(1)(1)#246
                                 - b_customer(9)(1)(1)#546 - w(9)(1)(1)(1)#1758
                                 - w(9)(2)(1)(1)#1788 - w(9)(3)(1)(1)#1818
                                 - w(9)(4)(1)(1)#1848 - w(9)(5)(1)(1)#1878
                                 - w(9)(6)(1)(1)#1908  = 29
 CustomerInventoryBalance(10,1,1): I_customer(10)(1)(1)#276
                                 - b_customer(10)(1)(1)#576
                                 - w(10)(1)(1)(1)#1938 - w(10)(2)(1)(1)#1968
                                 - w(10)(3)(1)(1)#1998 - w(10)(4)(1)(1)#2028
                                 - w(10)(5)(1)(1)#2058 - w(10)(6)(1)(1)#2088
                                  = -6
 CustomerInventoryBalance(11,1,1): I_customer(11)(1)(1)#306
                                 - b_customer(11)(1)(1)#606
                                 - w(11)(1)(1)(1)#2118 - w(11)(2)(1)(1)#2148
                                 - w(11)(3)(1)(1)#2178 - w(11)(4)(1)(1)#2208
                                 - w(11)(5)(1)(1)#2238 - w(11)(6)(1)(1)#2268
                                  = 32
 CustomerInventoryBalance(12,1,1): I_customer(12)(1)(1)#336
                                 - b_customer(12)(1)(1)#636
                                 - w(12)(1)(1)(1)#2298 - w(12)(2)(1)(1)#2328
                                 - w(12)(3)(1)(1)#2358 - w(12)(4)(1)(1)#2388
                                 - w(12)(5)(1)(1)#2418 - w(12)(6)(1)(1)#2448
                                  = 23
 CustomerInventoryBalance(3,2,1): - I_customer(3)(1)(1)#66
                                 + I_customer(3)(2)(1)#76
                                 - b_customer(3)(2)(1)#376 - w(3)(1)(2)(1)#688
                                 - w(3)(2)(2)(1)#718 - w(3)(3)(2)(1)#748
                                 - w(3)(4)(2)(1)#778 - w(3)(5)(2)(1)#808
                                 - w(3)(6)(2)(1)#838  = -9
 CustomerInventoryBalance(4,2,1): - I_customer(4)(1)(1)#96
                                 + I_customer(4)(2)(1)#106
                                 - b_customer(4)(2)(1)#406 - w(4)(1)(2)(1)#868
                                 - w(4)(2)(2)(1)#898 - w(4)(3)(2)(1)#928
                                 - w(4)(4)(2)(1)#958 - w(4)(5)(2)(1)#988
                                 - w(4)(6)(2)(1)#1018  = -12
 CustomerInventoryBalance(5,2,1): - I_customer(5)(1)(1)#126
                                 + I_customer(5)(2)(1)#136
                                 - b_customer(5)(2)(1)#436 - w(5)(1)(2)(1)#1048
                                 - w(5)(2)(2)(1)#1078 - w(5)(3)(2)(1)#1108
                                 - w(5)(4)(2)(1)#1138 - w(5)(5)(2)(1)#1168
                                 - w(5)(6)(2)(1)#1198  = -15
 CustomerInventoryBalance(6,2,1): - I_customer(6)(1)(1)#156
                                 + I_customer(6)(2)(1)#166
                                 - b_customer(6)(2)(1)#466 - w(6)(1)(2)(1)#1228
                                 - w(6)(2)(2)(1)#1258 - w(6)(3)(2)(1)#1288
                                 - w(6)(4)(2)(1)#1318 - w(6)(5)(2)(1)#1348
                                 - w(6)(6)(2)(1)#1378  = -7
 CustomerInventoryBalance(7,2,1): - I_customer(7)(1)(1)#186
                                 + I_customer(7)(2)(1)#196
                                 - b_customer(7)(2)(1)#496 - w(7)(1)(2)(1)#1408
                                 - w(7)(2)(2)(1)#1438 - w(7)(3)(2)(1)#1468
                                 - w(7)(4)(2)(1)#1498 - w(7)(5)(2)(1)#1528
                                 - w(7)(6)(2)(1)#1558  = -15
 CustomerInventoryBalance(8,2,1): - I_customer(8)(1)(1)#216
                                 + I_customer(8)(2)(1)#226
                                 - b_customer(8)(2)(1)#526 - w(8)(1)(2)(1)#1588
                                 - w(8)(2)(2)(1)#1618 - w(8)(3)(2)(1)#1648
                                 - w(8)(4)(2)(1)#1678 - w(8)(5)(2)(1)#1708
                                 - w(8)(6)(2)(1)#1738  = -18
 CustomerInventoryBalance(9,2,1): - I_customer(9)(1)(1)#246
                                 + I_customer(9)(2)(1)#256
                                 - b_customer(9)(2)(1)#556 - w(9)(1)(2)(1)#1768
                                 - w(9)(2)(2)(1)#1798 - w(9)(3)(2)(1)#1828
                                 - w(9)(4)(2)(1)#1858 - w(9)(5)(2)(1)#1888
                                 - w(9)(6)(2)(1)#1918  = -20
 CustomerInventoryBalance(10,2,1): - I_customer(10)(1)(1)#276
                                 + I_customer(10)(2)(1)#286
                                 - b_customer(10)(2)(1)#586
                                 - w(10)(1)(2)(1)#1948 - w(10)(2)(2)(1)#1978
                                 - w(10)(3)(2)(1)#2008 - w(10)(4)(2)(1)#2038
                                 - w(10)(5)(2)(1)#2068 - w(10)(6)(2)(1)#2098
                                  = -11
 CustomerInventoryBalance(11,2,1): - I_customer(11)(1)(1)#306
                                 + I_customer(11)(2)(1)#316
                                 - b_customer(11)(2)(1)#616
                                 - w(11)(1)(2)(1)#2128 - w(11)(2)(2)(1)#2158
                                 - w(11)(3)(2)(1)#2188 - w(11)(4)(2)(1)#2218
                                 - w(11)(5)(2)(1)#2248 - w(11)(6)(2)(1)#2278
                                  = -16
 CustomerInventoryBalance(12,2,1): - I_customer(12)(1)(1)#336
                                 + I_customer(12)(2)(1)#346
                                 - b_customer(12)(2)(1)#646
                                 - w(12)(1)(2)(1)#2308 - w(12)(2)(2)(1)#2338
                                 - w(12)(3)(2)(1)#2368 - w(12)(4)(2)(1)#2398
                                 - w(12)(5)(2)(1)#2428 - w(12)(6)(2)(1)#2458
                                  = -21
 CustomerInventoryBalance(3,3,1): - I_customer(3)(2)(1)#76
                                 + I_customer(3)(3)(1)#86
                                 - b_customer(3)(3)(1)#386 - w(3)(1)(3)(1)#698
                                 - w(3)(2)(3)(1)#728 - w(3)(3)(3)(1)#758
                                 - w(3)(4)(3)(1)#788 - w(3)(5)(3)(1)#818
                                 - w(3)(6)(3)(1)#848  = -12
 CustomerInventoryBalance(4,3,1): - I_customer(4)(2)(1)#106
                                 + I_customer(4)(3)(1)#116
                                 - b_customer(4)(3)(1)#416 - w(4)(1)(3)(1)#878
                                 - w(4)(2)(3)(1)#908 - w(4)(3)(3)(1)#938
                                 - w(4)(4)(3)(1)#968 - w(4)(5)(3)(1)#998
                                 - w(4)(6)(3)(1)#1028  = -17
 CustomerInventoryBalance(5,3,1): - I_customer(5)(2)(1)#136
                                 + I_customer(5)(3)(1)#146
                                 - b_customer(5)(3)(1)#446 - w(5)(1)(3)(1)#1058
                                 - w(5)(2)(3)(1)#1088 - w(5)(3)(3)(1)#1118
                                 - w(5)(4)(3)(1)#1148 - w(5)(5)(3)(1)#1178
                                 - w(5)(6)(3)(1)#1208  = -16
 CustomerInventoryBalance(6,3,1): - I_customer(6)(2)(1)#166
                                 + I_customer(6)(3)(1)#176
                                 - b_customer(6)(3)(1)#476 - w(6)(1)(3)(1)#1238
                                 - w(6)(2)(3)(1)#1268 - w(6)(3)(3)(1)#1298
                                 - w(6)(4)(3)(1)#1328 - w(6)(5)(3)(1)#1358
                                 - w(6)(6)(3)(1)#1388  = -7
 CustomerInventoryBalance(7,3,1): - I_customer(7)(2)(1)#196
                                 + I_customer(7)(3)(1)#206
                                 - b_customer(7)(3)(1)#506 - w(7)(1)(3)(1)#1418
                                 - w(7)(2)(3)(1)#1448 - w(7)(3)(3)(1)#1478
                                 - w(7)(4)(3)(1)#1508 - w(7)(5)(3)(1)#1538
                                 - w(7)(6)(3)(1)#1568  = -12
 CustomerInventoryBalance(8,3,1): - I_customer(8)(2)(1)#226
                                 + I_customer(8)(3)(1)#236
                                 - b_customer(8)(3)(1)#536 - w(8)(1)(3)(1)#1598
                                 - w(8)(2)(3)(1)#1628 - w(8)(3)(3)(1)#1658
                                 - w(8)(4)(3)(1)#1688 - w(8)(5)(3)(1)#1718
                                 - w(8)(6)(3)(1)#1748  = -17
 CustomerInventoryBalance(9,3,1): - I_customer(9)(2)(1)#256
                                 + I_customer(9)(3)(1)#266
                                 - b_customer(9)(3)(1)#566 - w(9)(1)(3)(1)#1778
                                 - w(9)(2)(3)(1)#1808 - w(9)(3)(3)(1)#1838
                                 - w(9)(4)(3)(1)#1868 - w(9)(5)(3)(1)#1898
                                 - w(9)(6)(3)(1)#1928  = -24
 CustomerInventoryBalance(10,3,1): - I_customer(10)(2)(1)#286
                                 + I_customer(10)(3)(1)#296
                                 - b_customer(10)(3)(1)#596
                                 - w(10)(1)(3)(1)#1958 - w(10)(2)(3)(1)#1988
                                 - w(10)(3)(3)(1)#2018 - w(10)(4)(3)(1)#2048
                                 - w(10)(5)(3)(1)#2078 - w(10)(6)(3)(1)#2108
                                  = -13
 CustomerInventoryBalance(11,3,1): - I_customer(11)(2)(1)#316
                                 + I_customer(11)(3)(1)#326
                                 - b_customer(11)(3)(1)#626
                                 - w(11)(1)(3)(1)#2138 - w(11)(2)(3)(1)#2168
                                 - w(11)(3)(3)(1)#2198 - w(11)(4)(3)(1)#2228
                                 - w(11)(5)(3)(1)#2258 - w(11)(6)(3)(1)#2288
                                  = -16
 CustomerInventoryBalance(12,3,1): - I_customer(12)(2)(1)#346
                                 + I_customer(12)(3)(1)#356
                                 - b_customer(12)(3)(1)#656
                                 - w(12)(1)(3)(1)#2318 - w(12)(2)(3)(1)#2348
                                 - w(12)(3)(3)(1)#2378 - w(12)(4)(3)(1)#2408
                                 - w(12)(5)(3)(1)#2438 - w(12)(6)(3)(1)#2468
                                  = -25
 CustomerInventoryBalance(3,1,2): I_customer(3)(1)(2)#67
                                 - b_customer(3)(1)(2)#367 - w(3)(1)(1)(2)#679
                                 - w(3)(2)(1)(2)#709 - w(3)(3)(1)(2)#739
                                 - w(3)(4)(1)(2)#769 - w(3)(5)(1)(2)#799
                                 - w(3)(6)(1)(2)#829  = -7
 CustomerInventoryBalance(4,1,2): I_customer(4)(1)(2)#97
                                 - b_customer(4)(1)(2)#397 - w(4)(1)(1)(2)#859
                                 - w(4)(2)(1)(2)#889 - w(4)(3)(1)(2)#919
                                 - w(4)(4)(1)(2)#949 - w(4)(5)(1)(2)#979
                                 - w(4)(6)(1)(2)#1009  = 3
 CustomerInventoryBalance(5,1,2): I_customer(5)(1)(2)#127
                                 - b_customer(5)(1)(2)#427 - w(5)(1)(1)(2)#1039
                                 - w(5)(2)(1)(2)#1069 - w(5)(3)(1)(2)#1099
                                 - w(5)(4)(1)(2)#1129 - w(5)(5)(1)(2)#1159
                                 - w(5)(6)(1)(2)#1189  = 3
 CustomerInventoryBalance(6,1,2): I_customer(6)(1)(2)#157
                                 - b_customer(6)(1)(2)#457 - w(6)(1)(1)(2)#1219
                                 - w(6)(2)(1)(2)#1249 - w(6)(3)(1)(2)#1279
                                 - w(6)(4)(1)(2)#1309 - w(6)(5)(1)(2)#1339
                                 - w(6)(6)(1)(2)#1369  = -5
 CustomerInventoryBalance(7,1,2): I_customer(7)(1)(2)#187
                                 - b_customer(7)(1)(2)#487 - w(7)(1)(1)(2)#1399
                                 - w(7)(2)(1)(2)#1429 - w(7)(3)(1)(2)#1459
                                 - w(7)(4)(1)(2)#1489 - w(7)(5)(1)(2)#1519
                                 - w(7)(6)(1)(2)#1549  = 2
 CustomerInventoryBalance(8,1,2): I_customer(8)(1)(2)#217
                                 - b_customer(8)(1)(2)#517 - w(8)(1)(1)(2)#1579
                                 - w(8)(2)(1)(2)#1609 - w(8)(3)(1)(2)#1639
                                 - w(8)(4)(1)(2)#1669 - w(8)(5)(1)(2)#1699
                                 - w(8)(6)(1)(2)#1729  = 24
 CustomerInventoryBalance(9,1,2): I_customer(9)(1)(2)#247
                                 - b_customer(9)(1)(2)#547 - w(9)(1)(1)(2)#1759
                                 - w(9)(2)(1)(2)#1789 - w(9)(3)(1)(2)#1819
                                 - w(9)(4)(1)(2)#1849 - w(9)(5)(1)(2)#1879
                                 - w(9)(6)(1)(2)#1909  = 37
 CustomerInventoryBalance(10,1,2): I_customer(10)(1)(2)#277
                                 - b_customer(10)(1)(2)#577
                                 - w(10)(1)(1)(2)#1939 - w(10)(2)(1)(2)#1969
                                 - w(10)(3)(1)(2)#1999 - w(10)(4)(1)(2)#2029
                                 - w(10)(5)(1)(2)#2059 - w(10)(6)(1)(2)#2089
                                  = -5
 CustomerInventoryBalance(11,1,2): I_customer(11)(1)(2)#307
                                 - b_customer(11)(1)(2)#607
                                 - w(11)(1)(1)(2)#2119 - w(11)(2)(1)(2)#2149
                                 - w(11)(3)(1)(2)#2179 - w(11)(4)(1)(2)#2209
                                 - w(11)(5)(1)(2)#2239 - w(11)(6)(1)(2)#2269
                                  = 25
 CustomerInventoryBalance(12,1,2): I_customer(12)(1)(2)#337
                                 - b_customer(12)(1)(2)#637
                                 - w(12)(1)(1)(2)#2299 - w(12)(2)(1)(2)#2329
                                 - w(12)(3)(1)(2)#2359 - w(12)(4)(1)(2)#2389
                                 - w(12)(5)(1)(2)#2419 - w(12)(6)(1)(2)#2449
                                  = 26
 CustomerInventoryBalance(3,2,2): - I_customer(3)(1)(2)#67
                                 + I_customer(3)(2)(2)#77
                                 - b_customer(3)(2)(2)#377 - w(3)(1)(2)(2)#689
                                 - w(3)(2)(2)(2)#719 - w(3)(3)(2)(2)#749
                                 - w(3)(4)(2)(2)#779 - w(3)(5)(2)(2)#809
                                 - w(3)(6)(2)(2)#839  = -11
 CustomerInventoryBalance(4,2,2): - I_customer(4)(1)(2)#97
                                 + I_customer(4)(2)(2)#107
                                 - b_customer(4)(2)(2)#407 - w(4)(1)(2)(2)#869
                                 - w(4)(2)(2)(2)#899 - w(4)(3)(2)(2)#929
                                 - w(4)(4)(2)(2)#959 - w(4)(5)(2)(2)#989
                                 - w(4)(6)(2)(2)#1019  = -16
 CustomerInventoryBalance(5,2,2): - I_customer(5)(1)(2)#127
                                 + I_customer(5)(2)(2)#137
                                 - b_customer(5)(2)(2)#437 - w(5)(1)(2)(2)#1049
                                 - w(5)(2)(2)(2)#1079 - w(5)(3)(2)(2)#1109
                                 - w(5)(4)(2)(2)#1139 - w(5)(5)(2)(2)#1169
                                 - w(5)(6)(2)(2)#1199  = -17
 CustomerInventoryBalance(6,2,2): - I_customer(6)(1)(2)#157
                                 + I_customer(6)(2)(2)#167
                                 - b_customer(6)(2)(2)#467 - w(6)(1)(2)(2)#1229
                                 - w(6)(2)(2)(2)#1259 - w(6)(3)(2)(2)#1289
                                 - w(6)(4)(2)(2)#1319 - w(6)(5)(2)(2)#1349
                                 - w(6)(6)(2)(2)#1379  = -7
 CustomerInventoryBalance(7,2,2): - I_customer(7)(1)(2)#187
                                 + I_customer(7)(2)(2)#197
                                 - b_customer(7)(2)(2)#497 - w(7)(1)(2)(2)#1409
                                 - w(7)(2)(2)(2)#1439 - w(7)(3)(2)(2)#1469
                                 - w(7)(4)(2)(2)#1499 - w(7)(5)(2)(2)#1529
                                 - w(7)(6)(2)(2)#1559  = -13
 CustomerInventoryBalance(8,2,2): - I_customer(8)(1)(2)#217
                                 + I_customer(8)(2)(2)#227
                                 - b_customer(8)(2)(2)#527 - w(8)(1)(2)(2)#1589
                                 - w(8)(2)(2)(2)#1619 - w(8)(3)(2)(2)#1649
                                 - w(8)(4)(2)(2)#1679 - w(8)(5)(2)(2)#1709
                                 - w(8)(6)(2)(2)#1739  = -15
 CustomerInventoryBalance(9,2,2): - I_customer(9)(1)(2)#247
                                 + I_customer(9)(2)(2)#257
                                 - b_customer(9)(2)(2)#557 - w(9)(1)(2)(2)#1769
                                 - w(9)(2)(2)(2)#1799 - w(9)(3)(2)(2)#1829
                                 - w(9)(4)(2)(2)#1859 - w(9)(5)(2)(2)#1889
                                 - w(9)(6)(2)(2)#1919  = -20
 CustomerInventoryBalance(10,2,2): - I_customer(10)(1)(2)#277
                                 + I_customer(10)(2)(2)#287
                                 - b_customer(10)(2)(2)#587
                                 - w(10)(1)(2)(2)#1949 - w(10)(2)(2)(2)#1979
                                 - w(10)(3)(2)(2)#2009 - w(10)(4)(2)(2)#2039
                                 - w(10)(5)(2)(2)#2069 - w(10)(6)(2)(2)#2099
                                  = -15
 CustomerInventoryBalance(11,2,2): - I_customer(11)(1)(2)#307
                                 + I_customer(11)(2)(2)#317
                                 - b_customer(11)(2)(2)#617
                                 - w(11)(1)(2)(2)#2129 - w(11)(2)(2)(2)#2159
                                 - w(11)(3)(2)(2)#2189 - w(11)(4)(2)(2)#2219
                                 - w(11)(5)(2)(2)#2249 - w(11)(6)(2)(2)#2279
                                  = -18
 CustomerInventoryBalance(12,2,2): - I_customer(12)(1)(2)#337
                                 + I_customer(12)(2)(2)#347
                                 - b_customer(12)(2)(2)#647
                                 - w(12)(1)(2)(2)#2309 - w(12)(2)(2)(2)#2339
                                 - w(12)(3)(2)(2)#2369 - w(12)(4)(2)(2)#2399
                                 - w(12)(5)(2)(2)#2429 - w(12)(6)(2)(2)#2459
                                  = -22
 CustomerInventoryBalance(3,3,2): - I_customer(3)(2)(2)#77
                                 + I_customer(3)(3)(2)#87
                                 - b_customer(3)(3)(2)#387 - w(3)(1)(3)(2)#699
                                 - w(3)(2)(3)(2)#729 - w(3)(3)(3)(2)#759
                                 - w(3)(4)(3)(2)#789 - w(3)(5)(3)(2)#819
                                 - w(3)(6)(3)(2)#849  = -9
 CustomerInventoryBalance(4,3,2): - I_customer(4)(2)(2)#107
                                 + I_customer(4)(3)(2)#117
                                 - b_customer(4)(3)(2)#417 - w(4)(1)(3)(2)#879
                                 - w(4)(2)(3)(2)#909 - w(4)(3)(3)(2)#939
                                 - w(4)(4)(3)(2)#969 - w(4)(5)(3)(2)#999
                                 - w(4)(6)(3)(2)#1029  = -14
 CustomerInventoryBalance(5,3,2): - I_customer(5)(2)(2)#137
                                 + I_customer(5)(3)(2)#147
                                 - b_customer(5)(3)(2)#447 - w(5)(1)(3)(2)#1059
                                 - w(5)(2)(3)(2)#1089 - w(5)(3)(3)(2)#1119
                                 - w(5)(4)(3)(2)#1149 - w(5)(5)(3)(2)#1179
                                 - w(5)(6)(3)(2)#1209  = -17
 CustomerInventoryBalance(6,3,2): - I_customer(6)(2)(2)#167
                                 + I_customer(6)(3)(2)#177
                                 - b_customer(6)(3)(2)#477 - w(6)(1)(3)(2)#1239
                                 - w(6)(2)(3)(2)#1269 - w(6)(3)(3)(2)#1299
                                 - w(6)(4)(3)(2)#1329 - w(6)(5)(3)(2)#1359
                                 - w(6)(6)(3)(2)#1389  = -8
 CustomerInventoryBalance(7,3,2): - I_customer(7)(2)(2)#197
                                 + I_customer(7)(3)(2)#207
                                 - b_customer(7)(3)(2)#507 - w(7)(1)(3)(2)#1419
                                 - w(7)(2)(3)(2)#1449 - w(7)(3)(3)(2)#1479
                                 - w(7)(4)(3)(2)#1509 - w(7)(5)(3)(2)#1539
                                 - w(7)(6)(3)(2)#1569  = -11
 CustomerInventoryBalance(8,3,2): - I_customer(8)(2)(2)#227
                                 + I_customer(8)(3)(2)#237
                                 - b_customer(8)(3)(2)#537 - w(8)(1)(3)(2)#1599
                                 - w(8)(2)(3)(2)#1629 - w(8)(3)(3)(2)#1659
                                 - w(8)(4)(3)(2)#1689 - w(8)(5)(3)(2)#1719
                                 - w(8)(6)(3)(2)#1749  = -17
 CustomerInventoryBalance(9,3,2): - I_customer(9)(2)(2)#257
                                 + I_customer(9)(3)(2)#267
                                 - b_customer(9)(3)(2)#567 - w(9)(1)(3)(2)#1779
                                 - w(9)(2)(3)(2)#1809 - w(9)(3)(3)(2)#1839
                                 - w(9)(4)(3)(2)#1869 - w(9)(5)(3)(2)#1899
                                 - w(9)(6)(3)(2)#1929  = -18
 CustomerInventoryBalance(10,3,2): - I_customer(10)(2)(2)#287
                                 + I_customer(10)(3)(2)#297
                                 - b_customer(10)(3)(2)#597
                                 - w(10)(1)(3)(2)#1959 - w(10)(2)(3)(2)#1989
                                 - w(10)(3)(3)(2)#2019 - w(10)(4)(3)(2)#2049
                                 - w(10)(5)(3)(2)#2079 - w(10)(6)(3)(2)#2109
                                  = -14
 CustomerInventoryBalance(11,3,2): - I_customer(11)(2)(2)#317
                                 + I_customer(11)(3)(2)#327
                                 - b_customer(11)(3)(2)#627
                                 - w(11)(1)(3)(2)#2139 - w(11)(2)(3)(2)#2169
                                 - w(11)(3)(3)(2)#2199 - w(11)(4)(3)(2)#2229
                                 - w(11)(5)(3)(2)#2259 - w(11)(6)(3)(2)#2289
                                  = -22
 CustomerInventoryBalance(12,3,2): - I_customer(12)(2)(2)#347
                                 + I_customer(12)(3)(2)#357
                                 - b_customer(12)(3)(2)#657
                                 - w(12)(1)(3)(2)#2319 - w(12)(2)(3)(2)#2349
                                 - w(12)(3)(3)(2)#2379 - w(12)(4)(3)(2)#2409
                                 - w(12)(5)(3)(2)#2439 - w(12)(6)(3)(2)#2469
                                  = -25
 CustomerInventoryBalance(3,1,3): I_customer(3)(1)(3)#68
                                 - b_customer(3)(1)(3)#368 - w(3)(1)(1)(3)#680
                                 - w(3)(2)(1)(3)#710 - w(3)(3)(1)(3)#740
                                 - w(3)(4)(1)(3)#770 - w(3)(5)(1)(3)#800
                                 - w(3)(6)(1)(3)#830  = -6
 CustomerInventoryBalance(4,1,3): I_customer(4)(1)(3)#98
                                 - b_customer(4)(1)(3)#398 - w(4)(1)(1)(3)#860
                                 - w(4)(2)(1)(3)#890 - w(4)(3)(1)(3)#920
                                 - w(4)(4)(1)(3)#950 - w(4)(5)(1)(3)#980
                                 - w(4)(6)(1)(3)#1010  = -2
 CustomerInventoryBalance(5,1,3): I_customer(5)(1)(3)#128
                                 - b_customer(5)(1)(3)#428 - w(5)(1)(1)(3)#1040
                                 - w(5)(2)(1)(3)#1070 - w(5)(3)(1)(3)#1100
                                 - w(5)(4)(1)(3)#1130 - w(5)(5)(1)(3)#1160
                                 - w(5)(6)(1)(3)#1190  = 0
 CustomerInventoryBalance(6,1,3): I_customer(6)(1)(3)#158
                                 - b_customer(6)(1)(3)#458 - w(6)(1)(1)(3)#1220
                                 - w(6)(2)(1)(3)#1250 - w(6)(3)(1)(3)#1280
                                 - w(6)(4)(1)(3)#1310 - w(6)(5)(1)(3)#1340
                                 - w(6)(6)(1)(3)#1370  = -4
 CustomerInventoryBalance(7,1,3): I_customer(7)(1)(3)#188
                                 - b_customer(7)(1)(3)#488 - w(7)(1)(1)(3)#1400
                                 - w(7)(2)(1)(3)#1430 - w(7)(3)(1)(3)#1460
                                 - w(7)(4)(1)(3)#1490 - w(7)(5)(1)(3)#1520
                                 - w(7)(6)(1)(3)#1550  = -1
 CustomerInventoryBalance(8,1,3): I_customer(8)(1)(3)#218
                                 - b_customer(8)(1)(3)#518 - w(8)(1)(1)(3)#1580
                                 - w(8)(2)(1)(3)#1610 - w(8)(3)(1)(3)#1640
                                 - w(8)(4)(1)(3)#1670 - w(8)(5)(1)(3)#1700
                                 - w(8)(6)(1)(3)#1730  = 22
 CustomerInventoryBalance(9,1,3): I_customer(9)(1)(3)#248
                                 - b_customer(9)(1)(3)#548 - w(9)(1)(1)(3)#1760
                                 - w(9)(2)(1)(3)#1790 - w(9)(3)(1)(3)#1820
                                 - w(9)(4)(1)(3)#1850 - w(9)(5)(1)(3)#1880
                                 - w(9)(6)(1)(3)#1910  = 36
 CustomerInventoryBalance(10,1,3): I_customer(10)(1)(3)#278
                                 - b_customer(10)(1)(3)#578
                                 - w(10)(1)(1)(3)#1940 - w(10)(2)(1)(3)#1970
                                 - w(10)(3)(1)(3)#2000 - w(10)(4)(1)(3)#2030
                                 - w(10)(5)(1)(3)#2060 - w(10)(6)(1)(3)#2090
                                  = -8
 CustomerInventoryBalance(11,1,3): I_customer(11)(1)(3)#308
                                 - b_customer(11)(1)(3)#608
                                 - w(11)(1)(1)(3)#2120 - w(11)(2)(1)(3)#2150
                                 - w(11)(3)(1)(3)#2180 - w(11)(4)(1)(3)#2210
                                 - w(11)(5)(1)(3)#2240 - w(11)(6)(1)(3)#2270
                                  = 29
 CustomerInventoryBalance(12,1,3): I_customer(12)(1)(3)#338
                                 - b_customer(12)(1)(3)#638
                                 - w(12)(1)(1)(3)#2300 - w(12)(2)(1)(3)#2330
                                 - w(12)(3)(1)(3)#2360 - w(12)(4)(1)(3)#2390
                                 - w(12)(5)(1)(3)#2420 - w(12)(6)(1)(3)#2450
                                  = 23
 CustomerInventoryBalance(3,2,3): - I_customer(3)(1)(3)#68
                                 + I_customer(3)(2)(3)#78
                                 - b_customer(3)(2)(3)#378 - w(3)(1)(2)(3)#690
                                 - w(3)(2)(2)(3)#720 - w(3)(3)(2)(3)#750
                                 - w(3)(4)(2)(3)#780 - w(3)(5)(2)(3)#810
                                 - w(3)(6)(2)(3)#840  = -11
 CustomerInventoryBalance(4,2,3): - I_customer(4)(1)(3)#98
                                 + I_customer(4)(2)(3)#108
                                 - b_customer(4)(2)(3)#408 - w(4)(1)(2)(3)#870
                                 - w(4)(2)(2)(3)#900 - w(4)(3)(2)(3)#930
                                 - w(4)(4)(2)(3)#960 - w(4)(5)(2)(3)#990
                                 - w(4)(6)(2)(3)#1020  = -17
 CustomerInventoryBalance(5,2,3): - I_customer(5)(1)(3)#128
                                 + I_customer(5)(2)(3)#138
                                 - b_customer(5)(2)(3)#438 - w(5)(1)(2)(3)#1050
                                 - w(5)(2)(2)(3)#1080 - w(5)(3)(2)(3)#1110
                                 - w(5)(4)(2)(3)#1140 - w(5)(5)(2)(3)#1170
                                 - w(5)(6)(2)(3)#1200  = -18
 CustomerInventoryBalance(6,2,3): - I_customer(6)(1)(3)#158
                                 + I_customer(6)(2)(3)#168
                                 - b_customer(6)(2)(3)#468 - w(6)(1)(2)(3)#1230
                                 - w(6)(2)(2)(3)#1260 - w(6)(3)(2)(3)#1290
                                 - w(6)(4)(2)(3)#1320 - w(6)(5)(2)(3)#1350
                                 - w(6)(6)(2)(3)#1380  = -6
 CustomerInventoryBalance(7,2,3): - I_customer(7)(1)(3)#188
                                 + I_customer(7)(2)(3)#198
                                 - b_customer(7)(2)(3)#498 - w(7)(1)(2)(3)#1410
                                 - w(7)(2)(2)(3)#1440 - w(7)(3)(2)(3)#1470
                                 - w(7)(4)(2)(3)#1500 - w(7)(5)(2)(3)#1530
                                 - w(7)(6)(2)(3)#1560  = -13
 CustomerInventoryBalance(8,2,3): - I_customer(8)(1)(3)#218
                                 + I_customer(8)(2)(3)#228
                                 - b_customer(8)(2)(3)#528 - w(8)(1)(2)(3)#1590
                                 - w(8)(2)(2)(3)#1620 - w(8)(3)(2)(3)#1650
                                 - w(8)(4)(2)(3)#1680 - w(8)(5)(2)(3)#1710
                                 - w(8)(6)(2)(3)#1740  = -17
 CustomerInventoryBalance(9,2,3): - I_customer(9)(1)(3)#248
                                 + I_customer(9)(2)(3)#258
                                 - b_customer(9)(2)(3)#558 - w(9)(1)(2)(3)#1770
                                 - w(9)(2)(2)(3)#1800 - w(9)(3)(2)(3)#1830
                                 - w(9)(4)(2)(3)#1860 - w(9)(5)(2)(3)#1890
                                 - w(9)(6)(2)(3)#1920  = -22
 CustomerInventoryBalance(10,2,3): - I_customer(10)(1)(3)#278
                                 + I_customer(10)(2)(3)#288
                                 - b_customer(10)(2)(3)#588
                                 - w(10)(1)(2)(3)#1950 - w(10)(2)(2)(3)#1980
                                 - w(10)(3)(2)(3)#2010 - w(10)(4)(2)(3)#2040
                                 - w(10)(5)(2)(3)#2070 - w(10)(6)(2)(3)#2100
                                  = -11
 CustomerInventoryBalance(11,2,3): - I_customer(11)(1)(3)#308
                                 + I_customer(11)(2)(3)#318
                                 - b_customer(11)(2)(3)#618
                                 - w(11)(1)(2)(3)#2130 - w(11)(2)(2)(3)#2160
                                 - w(11)(3)(2)(3)#2190 - w(11)(4)(2)(3)#2220
                                 - w(11)(5)(2)(3)#2250 - w(11)(6)(2)(3)#2280
                                  = -22
 CustomerInventoryBalance(12,2,3): - I_customer(12)(1)(3)#338
                                 + I_customer(12)(2)(3)#348
                                 - b_customer(12)(2)(3)#648
                                 - w(12)(1)(2)(3)#2310 - w(12)(2)(2)(3)#2340
                                 - w(12)(3)(2)(3)#2370 - w(12)(4)(2)(3)#2400
                                 - w(12)(5)(2)(3)#2430 - w(12)(6)(2)(3)#2460
                                  = -21
 CustomerInventoryBalance(3,3,3): - I_customer(3)(2)(3)#78
                                 + I_customer(3)(3)(3)#88
                                 - b_customer(3)(3)(3)#388 - w(3)(1)(3)(3)#700
                                 - w(3)(2)(3)(3)#730 - w(3)(3)(3)(3)#760
                                 - w(3)(4)(3)(3)#790 - w(3)(5)(3)(3)#820
                                 - w(3)(6)(3)(3)#850  = -11
 CustomerInventoryBalance(4,3,3): - I_customer(4)(2)(3)#108
                                 + I_customer(4)(3)(3)#118
                                 - b_customer(4)(3)(3)#418 - w(4)(1)(3)(3)#880
                                 - w(4)(2)(3)(3)#910 - w(4)(3)(3)(3)#940
                                 - w(4)(4)(3)(3)#970 - w(4)(5)(3)(3)#1000
                                 - w(4)(6)(3)(3)#1030  = -15
 CustomerInventoryBalance(5,3,3): - I_customer(5)(2)(3)#138
                                 + I_customer(5)(3)(3)#148
                                 - b_customer(5)(3)(3)#448 - w(5)(1)(3)(3)#1060
                                 - w(5)(2)(3)(3)#1090 - w(5)(3)(3)(3)#1120
                                 - w(5)(4)(3)(3)#1150 - w(5)(5)(3)(3)#1180
                                 - w(5)(6)(3)(3)#1210  = -13
 CustomerInventoryBalance(6,3,3): - I_customer(6)(2)(3)#168
                                 + I_customer(6)(3)(3)#178
                                 - b_customer(6)(3)(3)#478 - w(6)(1)(3)(3)#1240
                                 - w(6)(2)(3)(3)#1270 - w(6)(3)(3)(3)#1300
                                 - w(6)(4)(3)(3)#1330 - w(6)(5)(3)(3)#1360
                                 - w(6)(6)(3)(3)#1390  = -7
 CustomerInventoryBalance(7,3,3): - I_customer(7)(2)(3)#198
                                 + I_customer(7)(3)(3)#208
                                 - b_customer(7)(3)(3)#508 - w(7)(1)(3)(3)#1420
                                 - w(7)(2)(3)(3)#1450 - w(7)(3)(3)(3)#1480
                                 - w(7)(4)(3)(3)#1510 - w(7)(5)(3)(3)#1540
                                 - w(7)(6)(3)(3)#1570  = -13
 CustomerInventoryBalance(8,3,3): - I_customer(8)(2)(3)#228
                                 + I_customer(8)(3)(3)#238
                                 - b_customer(8)(3)(3)#538 - w(8)(1)(3)(3)#1600
                                 - w(8)(2)(3)(3)#1630 - w(8)(3)(3)(3)#1660
                                 - w(8)(4)(3)(3)#1690 - w(8)(5)(3)(3)#1720
                                 - w(8)(6)(3)(3)#1750  = -19
 CustomerInventoryBalance(9,3,3): - I_customer(9)(2)(3)#258
                                 + I_customer(9)(3)(3)#268
                                 - b_customer(9)(3)(3)#568 - w(9)(1)(3)(3)#1780
                                 - w(9)(2)(3)(3)#1810 - w(9)(3)(3)(3)#1840
                                 - w(9)(4)(3)(3)#1870 - w(9)(5)(3)(3)#1900
                                 - w(9)(6)(3)(3)#1930  = -26
 CustomerInventoryBalance(10,3,3): - I_customer(10)(2)(3)#288
                                 + I_customer(10)(3)(3)#298
                                 - b_customer(10)(3)(3)#598
                                 - w(10)(1)(3)(3)#1960 - w(10)(2)(3)(3)#1990
                                 - w(10)(3)(3)(3)#2020 - w(10)(4)(3)(3)#2050
                                 - w(10)(5)(3)(3)#2080 - w(10)(6)(3)(3)#2110
                                  = -15
 CustomerInventoryBalance(11,3,3): - I_customer(11)(2)(3)#318
                                 + I_customer(11)(3)(3)#328
                                 - b_customer(11)(3)(3)#628
                                 - w(11)(1)(3)(3)#2140 - w(11)(2)(3)(3)#2170
                                 - w(11)(3)(3)(3)#2200 - w(11)(4)(3)(3)#2230
                                 - w(11)(5)(3)(3)#2260 - w(11)(6)(3)(3)#2290
                                  = -20
 CustomerInventoryBalance(12,3,3): - I_customer(12)(2)(3)#348
                                 + I_customer(12)(3)(3)#358
                                 - b_customer(12)(3)(3)#658
                                 - w(12)(1)(3)(3)#2320 - w(12)(2)(3)(3)#2350
                                 - w(12)(3)(3)(3)#2380 - w(12)(4)(3)(3)#2410
                                 - w(12)(5)(3)(3)#2440 - w(12)(6)(3)(3)#2470
                                  = -21
 CustomerInventoryBalance(3,1,4): I_customer(3)(1)(4)#69
                                 - b_customer(3)(1)(4)#369 - w(3)(1)(1)(4)#681
                                 - w(3)(2)(1)(4)#711 - w(3)(3)(1)(4)#741
                                 - w(3)(4)(1)(4)#771 - w(3)(5)(1)(4)#801
                                 - w(3)(6)(1)(4)#831  = -6
 CustomerInventoryBalance(4,1,4): I_customer(4)(1)(4)#99
                                 - b_customer(4)(1)(4)#399 - w(4)(1)(1)(4)#861
                                 - w(4)(2)(1)(4)#891 - w(4)(3)(1)(4)#921
                                 - w(4)(4)(1)(4)#951 - w(4)(5)(1)(4)#981
                                 - w(4)(6)(1)(4)#1011  = -3
 CustomerInventoryBalance(5,1,4): I_customer(5)(1)(4)#129
                                 - b_customer(5)(1)(4)#429 - w(5)(1)(1)(4)#1041
                                 - w(5)(2)(1)(4)#1071 - w(5)(3)(1)(4)#1101
                                 - w(5)(4)(1)(4)#1131 - w(5)(5)(1)(4)#1161
                                 - w(5)(6)(1)(4)#1191  = -2
 CustomerInventoryBalance(6,1,4): I_customer(6)(1)(4)#159
                                 - b_customer(6)(1)(4)#459 - w(6)(1)(1)(4)#1221
                                 - w(6)(2)(1)(4)#1251 - w(6)(3)(1)(4)#1281
                                 - w(6)(4)(1)(4)#1311 - w(6)(5)(1)(4)#1341
                                 - w(6)(6)(1)(4)#1371  = -4
 CustomerInventoryBalance(7,1,4): I_customer(7)(1)(4)#189
                                 - b_customer(7)(1)(4)#489 - w(7)(1)(1)(4)#1401
                                 - w(7)(2)(1)(4)#1431 - w(7)(3)(1)(4)#1461
                                 - w(7)(4)(1)(4)#1491 - w(7)(5)(1)(4)#1521
                                 - w(7)(6)(1)(4)#1551  = -1
 CustomerInventoryBalance(8,1,4): I_customer(8)(1)(4)#219
                                 - b_customer(8)(1)(4)#519 - w(8)(1)(1)(4)#1581
                                 - w(8)(2)(1)(4)#1611 - w(8)(3)(1)(4)#1641
                                 - w(8)(4)(1)(4)#1671 - w(8)(5)(1)(4)#1701
                                 - w(8)(6)(1)(4)#1731  = 26
 CustomerInventoryBalance(9,1,4): I_customer(9)(1)(4)#249
                                 - b_customer(9)(1)(4)#549 - w(9)(1)(1)(4)#1761
                                 - w(9)(2)(1)(4)#1791 - w(9)(3)(1)(4)#1821
                                 - w(9)(4)(1)(4)#1851 - w(9)(5)(1)(4)#1881
                                 - w(9)(6)(1)(4)#1911  = 33
 CustomerInventoryBalance(10,1,4): I_customer(10)(1)(4)#279
                                 - b_customer(10)(1)(4)#579
                                 - w(10)(1)(1)(4)#1941 - w(10)(2)(1)(4)#1971
                                 - w(10)(3)(1)(4)#2001 - w(10)(4)(1)(4)#2031
                                 - w(10)(5)(1)(4)#2061 - w(10)(6)(1)(4)#2091
                                  = -9
 CustomerInventoryBalance(11,1,4): I_customer(11)(1)(4)#309
                                 - b_customer(11)(1)(4)#609
                                 - w(11)(1)(1)(4)#2121 - w(11)(2)(1)(4)#2151
                                 - w(11)(3)(1)(4)#2181 - w(11)(4)(1)(4)#2211
                                 - w(11)(5)(1)(4)#2241 - w(11)(6)(1)(4)#2271
                                  = 29
 CustomerInventoryBalance(12,1,4): I_customer(12)(1)(4)#339
                                 - b_customer(12)(1)(4)#639
                                 - w(12)(1)(1)(4)#2301 - w(12)(2)(1)(4)#2331
                                 - w(12)(3)(1)(4)#2361 - w(12)(4)(1)(4)#2391
                                 - w(12)(5)(1)(4)#2421 - w(12)(6)(1)(4)#2451
                                  = 20
 CustomerInventoryBalance(3,2,4): - I_customer(3)(1)(4)#69
                                 + I_customer(3)(2)(4)#79
                                 - b_customer(3)(2)(4)#379 - w(3)(1)(2)(4)#691
                                 - w(3)(2)(2)(4)#721 - w(3)(3)(2)(4)#751
                                 - w(3)(4)(2)(4)#781 - w(3)(5)(2)(4)#811
                                 - w(3)(6)(2)(4)#841  = -11
 CustomerInventoryBalance(4,2,4): - I_customer(4)(1)(4)#99
                                 + I_customer(4)(2)(4)#109
                                 - b_customer(4)(2)(4)#409 - w(4)(1)(2)(4)#871
                                 - w(4)(2)(2)(4)#901 - w(4)(3)(2)(4)#931
                                 - w(4)(4)(2)(4)#961 - w(4)(5)(2)(4)#991
                                 - w(4)(6)(2)(4)#1021  = -15
 CustomerInventoryBalance(5,2,4): - I_customer(5)(1)(4)#129
                                 + I_customer(5)(2)(4)#139
                                 - b_customer(5)(2)(4)#439 - w(5)(1)(2)(4)#1051
                                 - w(5)(2)(2)(4)#1081 - w(5)(3)(2)(4)#1111
                                 - w(5)(4)(2)(4)#1141 - w(5)(5)(2)(4)#1171
                                 - w(5)(6)(2)(4)#1201  = -14
 CustomerInventoryBalance(6,2,4): - I_customer(6)(1)(4)#159
                                 + I_customer(6)(2)(4)#169
                                 - b_customer(6)(2)(4)#469 - w(6)(1)(2)(4)#1231
                                 - w(6)(2)(2)(4)#1261 - w(6)(3)(2)(4)#1291
                                 - w(6)(4)(2)(4)#1321 - w(6)(5)(2)(4)#1351
                                 - w(6)(6)(2)(4)#1381  = -6
 CustomerInventoryBalance(7,2,4): - I_customer(7)(1)(4)#189
                                 + I_customer(7)(2)(4)#199
                                 - b_customer(7)(2)(4)#499 - w(7)(1)(2)(4)#1411
                                 - w(7)(2)(2)(4)#1441 - w(7)(3)(2)(4)#1471
                                 - w(7)(4)(2)(4)#1501 - w(7)(5)(2)(4)#1531
                                 - w(7)(6)(2)(4)#1561  = -11
 CustomerInventoryBalance(8,2,4): - I_customer(8)(1)(4)#219
                                 + I_customer(8)(2)(4)#229
                                 - b_customer(8)(2)(4)#529 - w(8)(1)(2)(4)#1591
                                 - w(8)(2)(2)(4)#1621 - w(8)(3)(2)(4)#1651
                                 - w(8)(4)(2)(4)#1681 - w(8)(5)(2)(4)#1711
                                 - w(8)(6)(2)(4)#1741  = -15
 CustomerInventoryBalance(9,2,4): - I_customer(9)(1)(4)#249
                                 + I_customer(9)(2)(4)#259
                                 - b_customer(9)(2)(4)#559 - w(9)(1)(2)(4)#1771
                                 - w(9)(2)(2)(4)#1801 - w(9)(3)(2)(4)#1831
                                 - w(9)(4)(2)(4)#1861 - w(9)(5)(2)(4)#1891
                                 - w(9)(6)(2)(4)#1921  = -24
 CustomerInventoryBalance(10,2,4): - I_customer(10)(1)(4)#279
                                 + I_customer(10)(2)(4)#289
                                 - b_customer(10)(2)(4)#589
                                 - w(10)(1)(2)(4)#1951 - w(10)(2)(2)(4)#1981
                                 - w(10)(3)(2)(4)#2011 - w(10)(4)(2)(4)#2041
                                 - w(10)(5)(2)(4)#2071 - w(10)(6)(2)(4)#2101
                                  = -16
 CustomerInventoryBalance(11,2,4): - I_customer(11)(1)(4)#309
                                 + I_customer(11)(2)(4)#319
                                 - b_customer(11)(2)(4)#619
                                 - w(11)(1)(2)(4)#2131 - w(11)(2)(2)(4)#2161
                                 - w(11)(3)(2)(4)#2191 - w(11)(4)(2)(4)#2221
                                 - w(11)(5)(2)(4)#2251 - w(11)(6)(2)(4)#2281
                                  = -22
 CustomerInventoryBalance(12,2,4): - I_customer(12)(1)(4)#339
                                 + I_customer(12)(2)(4)#349
                                 - b_customer(12)(2)(4)#649
                                 - w(12)(1)(2)(4)#2311 - w(12)(2)(2)(4)#2341
                                 - w(12)(3)(2)(4)#2371 - w(12)(4)(2)(4)#2401
                                 - w(12)(5)(2)(4)#2431 - w(12)(6)(2)(4)#2461
                                  = -23
 CustomerInventoryBalance(3,3,4): - I_customer(3)(2)(4)#79
                                 + I_customer(3)(3)(4)#89
                                 - b_customer(3)(3)(4)#389 - w(3)(1)(3)(4)#701
                                 - w(3)(2)(3)(4)#731 - w(3)(3)(3)(4)#761
                                 - w(3)(4)(3)(4)#791 - w(3)(5)(3)(4)#821
                                 - w(3)(6)(3)(4)#851  = -11
 CustomerInventoryBalance(4,3,4): - I_customer(4)(2)(4)#109
                                 + I_customer(4)(3)(4)#119
                                 - b_customer(4)(3)(4)#419 - w(4)(1)(3)(4)#881
                                 - w(4)(2)(3)(4)#911 - w(4)(3)(3)(4)#941
                                 - w(4)(4)(3)(4)#971 - w(4)(5)(3)(4)#1001
                                 - w(4)(6)(3)(4)#1031  = -13
 CustomerInventoryBalance(5,3,4): - I_customer(5)(2)(4)#139
                                 + I_customer(5)(3)(4)#149
                                 - b_customer(5)(3)(4)#449 - w(5)(1)(3)(4)#1061
                                 - w(5)(2)(3)(4)#1091 - w(5)(3)(3)(4)#1121
                                 - w(5)(4)(3)(4)#1151 - w(5)(5)(3)(4)#1181
                                 - w(5)(6)(3)(4)#1211  = -16
 CustomerInventoryBalance(6,3,4): - I_customer(6)(2)(4)#169
                                 + I_customer(6)(3)(4)#179
                                 - b_customer(6)(3)(4)#479 - w(6)(1)(3)(4)#1241
                                 - w(6)(2)(3)(4)#1271 - w(6)(3)(3)(4)#1301
                                 - w(6)(4)(3)(4)#1331 - w(6)(5)(3)(4)#1361
                                 - w(6)(6)(3)(4)#1391  = -6
 CustomerInventoryBalance(7,3,4): - I_customer(7)(2)(4)#199
                                 + I_customer(7)(3)(4)#209
                                 - b_customer(7)(3)(4)#509 - w(7)(1)(3)(4)#1421
                                 - w(7)(2)(3)(4)#1451 - w(7)(3)(3)(4)#1481
                                 - w(7)(4)(3)(4)#1511 - w(7)(5)(3)(4)#1541
                                 - w(7)(6)(3)(4)#1571  = -11
 CustomerInventoryBalance(8,3,4): - I_customer(8)(2)(4)#229
                                 + I_customer(8)(3)(4)#239
                                 - b_customer(8)(3)(4)#539 - w(8)(1)(3)(4)#1601
                                 - w(8)(2)(3)(4)#1631 - w(8)(3)(3)(4)#1661
                                 - w(8)(4)(3)(4)#1691 - w(8)(5)(3)(4)#1721
                                 - w(8)(6)(3)(4)#1751  = -15
 CustomerInventoryBalance(9,3,4): - I_customer(9)(2)(4)#259
                                 + I_customer(9)(3)(4)#269
                                 - b_customer(9)(3)(4)#569 - w(9)(1)(3)(4)#1781
                                 - w(9)(2)(3)(4)#1811 - w(9)(3)(3)(4)#1841
                                 - w(9)(4)(3)(4)#1871 - w(9)(5)(3)(4)#1901
                                 - w(9)(6)(3)(4)#1931  = -23
 CustomerInventoryBalance(10,3,4): - I_customer(10)(2)(4)#289
                                 + I_customer(10)(3)(4)#299
                                 - b_customer(10)(3)(4)#599
                                 - w(10)(1)(3)(4)#1961 - w(10)(2)(3)(4)#1991
                                 - w(10)(3)(3)(4)#2021 - w(10)(4)(3)(4)#2051
                                 - w(10)(5)(3)(4)#2081 - w(10)(6)(3)(4)#2111
                                  = -15
 CustomerInventoryBalance(11,3,4): - I_customer(11)(2)(4)#319
                                 + I_customer(11)(3)(4)#329
                                 - b_customer(11)(3)(4)#629
                                 - w(11)(1)(3)(4)#2141 - w(11)(2)(3)(4)#2171
                                 - w(11)(3)(3)(4)#2201 - w(11)(4)(3)(4)#2231
                                 - w(11)(5)(3)(4)#2261 - w(11)(6)(3)(4)#2291
                                  = -16
 CustomerInventoryBalance(12,3,4): - I_customer(12)(2)(4)#349
                                 + I_customer(12)(3)(4)#359
                                 - b_customer(12)(3)(4)#659
                                 - w(12)(1)(3)(4)#2321 - w(12)(2)(3)(4)#2351
                                 - w(12)(3)(3)(4)#2381 - w(12)(4)(3)(4)#2411
                                 - w(12)(5)(3)(4)#2441 - w(12)(6)(3)(4)#2471
                                  = -19
 CustomerInventoryBalance(3,1,5): I_customer(3)(1)(5)#70
                                 - b_customer(3)(1)(5)#370 - w(3)(1)(1)(5)#682
                                 - w(3)(2)(1)(5)#712 - w(3)(3)(1)(5)#742
                                 - w(3)(4)(1)(5)#772 - w(3)(5)(1)(5)#802
                                 - w(3)(6)(1)(5)#832  = -4
 CustomerInventoryBalance(4,1,5): I_customer(4)(1)(5)#100
                                 - b_customer(4)(1)(5)#400 - w(4)(1)(1)(5)#862
                                 - w(4)(2)(1)(5)#892 - w(4)(3)(1)(5)#922
                                 - w(4)(4)(1)(5)#952 - w(4)(5)(1)(5)#982
                                 - w(4)(6)(1)(5)#1012  = -2
 CustomerInventoryBalance(5,1,5): I_customer(5)(1)(5)#130
                                 - b_customer(5)(1)(5)#430 - w(5)(1)(1)(5)#1042
                                 - w(5)(2)(1)(5)#1072 - w(5)(3)(1)(5)#1102
                                 - w(5)(4)(1)(5)#1132 - w(5)(5)(1)(5)#1162
                                 - w(5)(6)(1)(5)#1192  = -2
 CustomerInventoryBalance(6,1,5): I_customer(6)(1)(5)#160
                                 - b_customer(6)(1)(5)#460 - w(6)(1)(1)(5)#1222
                                 - w(6)(2)(1)(5)#1252 - w(6)(3)(1)(5)#1282
                                 - w(6)(4)(1)(5)#1312 - w(6)(5)(1)(5)#1342
                                 - w(6)(6)(1)(5)#1372  = -5
 CustomerInventoryBalance(7,1,5): I_customer(7)(1)(5)#190
                                 - b_customer(7)(1)(5)#490 - w(7)(1)(1)(5)#1402
                                 - w(7)(2)(1)(5)#1432 - w(7)(3)(1)(5)#1462
                                 - w(7)(4)(1)(5)#1492 - w(7)(5)(1)(5)#1522
                                 - w(7)(6)(1)(5)#1552  = -1
 CustomerInventoryBalance(8,1,5): I_customer(8)(1)(5)#220
                                 - b_customer(8)(1)(5)#520 - w(8)(1)(1)(5)#1582
                                 - w(8)(2)(1)(5)#1612 - w(8)(3)(1)(5)#1642
                                 - w(8)(4)(1)(5)#1672 - w(8)(5)(1)(5)#1702
                                 - w(8)(6)(1)(5)#1732  = 21
 CustomerInventoryBalance(9,1,5): I_customer(9)(1)(5)#250
                                 - b_customer(9)(1)(5)#550 - w(9)(1)(1)(5)#1762
                                 - w(9)(2)(1)(5)#1792 - w(9)(3)(1)(5)#1822
                                 - w(9)(4)(1)(5)#1852 - w(9)(5)(1)(5)#1882
                                 - w(9)(6)(1)(5)#1912  = 29
 CustomerInventoryBalance(10,1,5): I_customer(10)(1)(5)#280
                                 - b_customer(10)(1)(5)#580
                                 - w(10)(1)(1)(5)#1942 - w(10)(2)(1)(5)#1972
                                 - w(10)(3)(1)(5)#2002 - w(10)(4)(1)(5)#2032
                                 - w(10)(5)(1)(5)#2062 - w(10)(6)(1)(5)#2092
                                  = -5
 CustomerInventoryBalance(11,1,5): I_customer(11)(1)(5)#310
                                 - b_customer(11)(1)(5)#610
                                 - w(11)(1)(1)(5)#2122 - w(11)(2)(1)(5)#2152
                                 - w(11)(3)(1)(5)#2182 - w(11)(4)(1)(5)#2212
                                 - w(11)(5)(1)(5)#2242 - w(11)(6)(1)(5)#2272
                                  = 25
 CustomerInventoryBalance(12,1,5): I_customer(12)(1)(5)#340
                                 - b_customer(12)(1)(5)#640
                                 - w(12)(1)(1)(5)#2302 - w(12)(2)(1)(5)#2332
                                 - w(12)(3)(1)(5)#2362 - w(12)(4)(1)(5)#2392
                                 - w(12)(5)(1)(5)#2422 - w(12)(6)(1)(5)#2452
                                  = 22
 CustomerInventoryBalance(3,2,5): - I_customer(3)(1)(5)#70
                                 + I_customer(3)(2)(5)#80
                                 - b_customer(3)(2)(5)#380 - w(3)(1)(2)(5)#692
                                 - w(3)(2)(2)(5)#722 - w(3)(3)(2)(5)#752
                                 - w(3)(4)(2)(5)#782 - w(3)(5)(2)(5)#812
                                 - w(3)(6)(2)(5)#842  = -12
 CustomerInventoryBalance(4,2,5): - I_customer(4)(1)(5)#100
                                 + I_customer(4)(2)(5)#110
                                 - b_customer(4)(2)(5)#410 - w(4)(1)(2)(5)#872
                                 - w(4)(2)(2)(5)#902 - w(4)(3)(2)(5)#932
                                 - w(4)(4)(2)(5)#962 - w(4)(5)(2)(5)#992
                                 - w(4)(6)(2)(5)#1022  = -17
 CustomerInventoryBalance(5,2,5): - I_customer(5)(1)(5)#130
                                 + I_customer(5)(2)(5)#140
                                 - b_customer(5)(2)(5)#440 - w(5)(1)(2)(5)#1052
                                 - w(5)(2)(2)(5)#1082 - w(5)(3)(2)(5)#1112
                                 - w(5)(4)(2)(5)#1142 - w(5)(5)(2)(5)#1172
                                 - w(5)(6)(2)(5)#1202  = -12
 CustomerInventoryBalance(6,2,5): - I_customer(6)(1)(5)#160
                                 + I_customer(6)(2)(5)#170
                                 - b_customer(6)(2)(5)#470 - w(6)(1)(2)(5)#1232
                                 - w(6)(2)(2)(5)#1262 - w(6)(3)(2)(5)#1292
                                 - w(6)(4)(2)(5)#1322 - w(6)(5)(2)(5)#1352
                                 - w(6)(6)(2)(5)#1382  = -7
 CustomerInventoryBalance(7,2,5): - I_customer(7)(1)(5)#190
                                 + I_customer(7)(2)(5)#200
                                 - b_customer(7)(2)(5)#500 - w(7)(1)(2)(5)#1412
                                 - w(7)(2)(2)(5)#1442 - w(7)(3)(2)(5)#1472
                                 - w(7)(4)(2)(5)#1502 - w(7)(5)(2)(5)#1532
                                 - w(7)(6)(2)(5)#1562  = -12
 CustomerInventoryBalance(8,2,5): - I_customer(8)(1)(5)#220
                                 + I_customer(8)(2)(5)#230
                                 - b_customer(8)(2)(5)#530 - w(8)(1)(2)(5)#1592
                                 - w(8)(2)(2)(5)#1622 - w(8)(3)(2)(5)#1652
                                 - w(8)(4)(2)(5)#1682 - w(8)(5)(2)(5)#1712
                                 - w(8)(6)(2)(5)#1742  = -14
 CustomerInventoryBalance(9,2,5): - I_customer(9)(1)(5)#250
                                 + I_customer(9)(2)(5)#260
                                 - b_customer(9)(2)(5)#560 - w(9)(1)(2)(5)#1772
                                 - w(9)(2)(2)(5)#1802 - w(9)(3)(2)(5)#1832
                                 - w(9)(4)(2)(5)#1862 - w(9)(5)(2)(5)#1892
                                 - w(9)(6)(2)(5)#1922  = -26
 CustomerInventoryBalance(10,2,5): - I_customer(10)(1)(5)#280
                                 + I_customer(10)(2)(5)#290
                                 - b_customer(10)(2)(5)#590
                                 - w(10)(1)(2)(5)#1952 - w(10)(2)(2)(5)#1982
                                 - w(10)(3)(2)(5)#2012 - w(10)(4)(2)(5)#2042
                                 - w(10)(5)(2)(5)#2072 - w(10)(6)(2)(5)#2102
                                  = -13
 CustomerInventoryBalance(11,2,5): - I_customer(11)(1)(5)#310
                                 + I_customer(11)(2)(5)#320
                                 - b_customer(11)(2)(5)#620
                                 - w(11)(1)(2)(5)#2132 - w(11)(2)(2)(5)#2162
                                 - w(11)(3)(2)(5)#2192 - w(11)(4)(2)(5)#2222
                                 - w(11)(5)(2)(5)#2252 - w(11)(6)(2)(5)#2282
                                  = -21
 CustomerInventoryBalance(12,2,5): - I_customer(12)(1)(5)#340
                                 + I_customer(12)(2)(5)#350
                                 - b_customer(12)(2)(5)#650
                                 - w(12)(1)(2)(5)#2312 - w(12)(2)(2)(5)#2342
                                 - w(12)(3)(2)(5)#2372 - w(12)(4)(2)(5)#2402
                                 - w(12)(5)(2)(5)#2432 - w(12)(6)(2)(5)#2462
                                  = -19
 CustomerInventoryBalance(3,3,5): - I_customer(3)(2)(5)#80
                                 + I_customer(3)(3)(5)#90
                                 - b_customer(3)(3)(5)#390 - w(3)(1)(3)(5)#702
                                 - w(3)(2)(3)(5)#732 - w(3)(3)(3)(5)#762
                                 - w(3)(4)(3)(5)#792 - w(3)(5)(3)(5)#822
                                 - w(3)(6)(3)(5)#852  = -9
 CustomerInventoryBalance(4,3,5): - I_customer(4)(2)(5)#110
                                 + I_customer(4)(3)(5)#120
                                 - b_customer(4)(3)(5)#420 - w(4)(1)(3)(5)#882
                                 - w(4)(2)(3)(5)#912 - w(4)(3)(3)(5)#942
                                 - w(4)(4)(3)(5)#972 - w(4)(5)(3)(5)#1002
                                 - w(4)(6)(3)(5)#1032  = -14
 CustomerInventoryBalance(5,3,5): - I_customer(5)(2)(5)#140
                                 + I_customer(5)(3)(5)#150
                                 - b_customer(5)(3)(5)#450 - w(5)(1)(3)(5)#1062
                                 - w(5)(2)(3)(5)#1092 - w(5)(3)(3)(5)#1122
                                 - w(5)(4)(3)(5)#1152 - w(5)(5)(3)(5)#1182
                                 - w(5)(6)(3)(5)#1212  = -14
 CustomerInventoryBalance(6,3,5): - I_customer(6)(2)(5)#170
                                 + I_customer(6)(3)(5)#180
                                 - b_customer(6)(3)(5)#480 - w(6)(1)(3)(5)#1242
                                 - w(6)(2)(3)(5)#1272 - w(6)(3)(3)(5)#1302
                                 - w(6)(4)(3)(5)#1332 - w(6)(5)(3)(5)#1362
                                 - w(6)(6)(3)(5)#1392  = -8
 CustomerInventoryBalance(7,3,5): - I_customer(7)(2)(5)#200
                                 + I_customer(7)(3)(5)#210
                                 - b_customer(7)(3)(5)#510 - w(7)(1)(3)(5)#1422
                                 - w(7)(2)(3)(5)#1452 - w(7)(3)(3)(5)#1482
                                 - w(7)(4)(3)(5)#1512 - w(7)(5)(3)(5)#1542
                                 - w(7)(6)(3)(5)#1572  = -12
 CustomerInventoryBalance(8,3,5): - I_customer(8)(2)(5)#230
                                 + I_customer(8)(3)(5)#240
                                 - b_customer(8)(3)(5)#540 - w(8)(1)(3)(5)#1602
                                 - w(8)(2)(3)(5)#1632 - w(8)(3)(3)(5)#1662
                                 - w(8)(4)(3)(5)#1692 - w(8)(5)(3)(5)#1722
                                 - w(8)(6)(3)(5)#1752  = -14
 CustomerInventoryBalance(9,3,5): - I_customer(9)(2)(5)#260
                                 + I_customer(9)(3)(5)#270
                                 - b_customer(9)(3)(5)#570 - w(9)(1)(3)(5)#1782
                                 - w(9)(2)(3)(5)#1812 - w(9)(3)(3)(5)#1842
                                 - w(9)(4)(3)(5)#1872 - w(9)(5)(3)(5)#1902
                                 - w(9)(6)(3)(5)#1932  = -24
 CustomerInventoryBalance(10,3,5): - I_customer(10)(2)(5)#290
                                 + I_customer(10)(3)(5)#300
                                 - b_customer(10)(3)(5)#600
                                 - w(10)(1)(3)(5)#1962 - w(10)(2)(3)(5)#1992
                                 - w(10)(3)(3)(5)#2022 - w(10)(4)(3)(5)#2052
                                 - w(10)(5)(3)(5)#2082 - w(10)(6)(3)(5)#2112
                                  = -11
 CustomerInventoryBalance(11,3,5): - I_customer(11)(2)(5)#320
                                 + I_customer(11)(3)(5)#330
                                 - b_customer(11)(3)(5)#630
                                 - w(11)(1)(3)(5)#2142 - w(11)(2)(3)(5)#2172
                                 - w(11)(3)(3)(5)#2202 - w(11)(4)(3)(5)#2232
                                 - w(11)(5)(3)(5)#2262 - w(11)(6)(3)(5)#2292
                                  = -16
 CustomerInventoryBalance(12,3,5): - I_customer(12)(2)(5)#350
                                 + I_customer(12)(3)(5)#360
                                 - b_customer(12)(3)(5)#660
                                 - w(12)(1)(3)(5)#2322 - w(12)(2)(3)(5)#2352
                                 - w(12)(3)(3)(5)#2382 - w(12)(4)(3)(5)#2412
                                 - w(12)(5)(3)(5)#2442 - w(12)(6)(3)(5)#2472
                                  = -19
 CustomerInventoryBalance(3,1,6): I_customer(3)(1)(6)#71
                                 - b_customer(3)(1)(6)#371 - w(3)(1)(1)(6)#683
                                 - w(3)(2)(1)(6)#713 - w(3)(3)(1)(6)#743
                                 - w(3)(4)(1)(6)#773 - w(3)(5)(1)(6)#803
                                 - w(3)(6)(1)(6)#833  = -7
 CustomerInventoryBalance(4,1,6): I_customer(4)(1)(6)#101
                                 - b_customer(4)(1)(6)#401 - w(4)(1)(1)(6)#863
                                 - w(4)(2)(1)(6)#893 - w(4)(3)(1)(6)#923
                                 - w(4)(4)(1)(6)#953 - w(4)(5)(1)(6)#983
                                 - w(4)(6)(1)(6)#1013  = 2
 CustomerInventoryBalance(5,1,6): I_customer(5)(1)(6)#131
                                 - b_customer(5)(1)(6)#431 - w(5)(1)(1)(6)#1043
                                 - w(5)(2)(1)(6)#1073 - w(5)(3)(1)(6)#1103
                                 - w(5)(4)(1)(6)#1133 - w(5)(5)(1)(6)#1163
                                 - w(5)(6)(1)(6)#1193  = 1
 CustomerInventoryBalance(6,1,6): I_customer(6)(1)(6)#161
                                 - b_customer(6)(1)(6)#461 - w(6)(1)(1)(6)#1223
                                 - w(6)(2)(1)(6)#1253 - w(6)(3)(1)(6)#1283
                                 - w(6)(4)(1)(6)#1313 - w(6)(5)(1)(6)#1343
                                 - w(6)(6)(1)(6)#1373  = -5
 CustomerInventoryBalance(7,1,6): I_customer(7)(1)(6)#191
                                 - b_customer(7)(1)(6)#491 - w(7)(1)(1)(6)#1403
                                 - w(7)(2)(1)(6)#1433 - w(7)(3)(1)(6)#1463
                                 - w(7)(4)(1)(6)#1493 - w(7)(5)(1)(6)#1523
                                 - w(7)(6)(1)(6)#1553  = -1
 CustomerInventoryBalance(8,1,6): I_customer(8)(1)(6)#221
                                 - b_customer(8)(1)(6)#521 - w(8)(1)(1)(6)#1583
                                 - w(8)(2)(1)(6)#1613 - w(8)(3)(1)(6)#1643
                                 - w(8)(4)(1)(6)#1673 - w(8)(5)(1)(6)#1703
                                 - w(8)(6)(1)(6)#1733  = 25
 CustomerInventoryBalance(9,1,6): I_customer(9)(1)(6)#251
                                 - b_customer(9)(1)(6)#551 - w(9)(1)(1)(6)#1763
                                 - w(9)(2)(1)(6)#1793 - w(9)(3)(1)(6)#1823
                                 - w(9)(4)(1)(6)#1853 - w(9)(5)(1)(6)#1883
                                 - w(9)(6)(1)(6)#1913  = 29
 CustomerInventoryBalance(10,1,6): I_customer(10)(1)(6)#281
                                 - b_customer(10)(1)(6)#581
                                 - w(10)(1)(1)(6)#1943 - w(10)(2)(1)(6)#1973
                                 - w(10)(3)(1)(6)#2003 - w(10)(4)(1)(6)#2033
                                 - w(10)(5)(1)(6)#2063 - w(10)(6)(1)(6)#2093
                                  = -7
 CustomerInventoryBalance(11,1,6): I_customer(11)(1)(6)#311
                                 - b_customer(11)(1)(6)#611
                                 - w(11)(1)(1)(6)#2123 - w(11)(2)(1)(6)#2153
                                 - w(11)(3)(1)(6)#2183 - w(11)(4)(1)(6)#2213
                                 - w(11)(5)(1)(6)#2243 - w(11)(6)(1)(6)#2273
                                  = 30
 CustomerInventoryBalance(12,1,6): I_customer(12)(1)(6)#341
                                 - b_customer(12)(1)(6)#641
                                 - w(12)(1)(1)(6)#2303 - w(12)(2)(1)(6)#2333
                                 - w(12)(3)(1)(6)#2363 - w(12)(4)(1)(6)#2393
                                 - w(12)(5)(1)(6)#2423 - w(12)(6)(1)(6)#2453
                                  = 24
 CustomerInventoryBalance(3,2,6): - I_customer(3)(1)(6)#71
                                 + I_customer(3)(2)(6)#81
                                 - b_customer(3)(2)(6)#381 - w(3)(1)(2)(6)#693
                                 - w(3)(2)(2)(6)#723 - w(3)(3)(2)(6)#753
                                 - w(3)(4)(2)(6)#783 - w(3)(5)(2)(6)#813
                                 - w(3)(6)(2)(6)#843  = -10
 CustomerInventoryBalance(4,2,6): - I_customer(4)(1)(6)#101
                                 + I_customer(4)(2)(6)#111
                                 - b_customer(4)(2)(6)#411 - w(4)(1)(2)(6)#873
                                 - w(4)(2)(2)(6)#903 - w(4)(3)(2)(6)#933
                                 - w(4)(4)(2)(6)#963 - w(4)(5)(2)(6)#993
                                 - w(4)(6)(2)(6)#1023  = -13
 CustomerInventoryBalance(5,2,6): - I_customer(5)(1)(6)#131
                                 + I_customer(5)(2)(6)#141
                                 - b_customer(5)(2)(6)#441 - w(5)(1)(2)(6)#1053
                                 - w(5)(2)(2)(6)#1083 - w(5)(3)(2)(6)#1113
                                 - w(5)(4)(2)(6)#1143 - w(5)(5)(2)(6)#1173
                                 - w(5)(6)(2)(6)#1203  = -17
 CustomerInventoryBalance(6,2,6): - I_customer(6)(1)(6)#161
                                 + I_customer(6)(2)(6)#171
                                 - b_customer(6)(2)(6)#471 - w(6)(1)(2)(6)#1233
                                 - w(6)(2)(2)(6)#1263 - w(6)(3)(2)(6)#1293
                                 - w(6)(4)(2)(6)#1323 - w(6)(5)(2)(6)#1353
                                 - w(6)(6)(2)(6)#1383  = -6
 CustomerInventoryBalance(7,2,6): - I_customer(7)(1)(6)#191
                                 + I_customer(7)(2)(6)#201
                                 - b_customer(7)(2)(6)#501 - w(7)(1)(2)(6)#1413
                                 - w(7)(2)(2)(6)#1443 - w(7)(3)(2)(6)#1473
                                 - w(7)(4)(2)(6)#1503 - w(7)(5)(2)(6)#1533
                                 - w(7)(6)(2)(6)#1563  = -11
 CustomerInventoryBalance(8,2,6): - I_customer(8)(1)(6)#221
                                 + I_customer(8)(2)(6)#231
                                 - b_customer(8)(2)(6)#531 - w(8)(1)(2)(6)#1593
                                 - w(8)(2)(2)(6)#1623 - w(8)(3)(2)(6)#1653
                                 - w(8)(4)(2)(6)#1683 - w(8)(5)(2)(6)#1713
                                 - w(8)(6)(2)(6)#1743  = -19
 CustomerInventoryBalance(9,2,6): - I_customer(9)(1)(6)#251
                                 + I_customer(9)(2)(6)#261
                                 - b_customer(9)(2)(6)#561 - w(9)(1)(2)(6)#1773
                                 - w(9)(2)(2)(6)#1803 - w(9)(3)(2)(6)#1833
                                 - w(9)(4)(2)(6)#1863 - w(9)(5)(2)(6)#1893
                                 - w(9)(6)(2)(6)#1923  = -19
 CustomerInventoryBalance(10,2,6): - I_customer(10)(1)(6)#281
                                 + I_customer(10)(2)(6)#291
                                 - b_customer(10)(2)(6)#591
                                 - w(10)(1)(2)(6)#1953 - w(10)(2)(2)(6)#1983
                                 - w(10)(3)(2)(6)#2013 - w(10)(4)(2)(6)#2043
                                 - w(10)(5)(2)(6)#2073 - w(10)(6)(2)(6)#2103
                                  = -14
 CustomerInventoryBalance(11,2,6): - I_customer(11)(1)(6)#311
                                 + I_customer(11)(2)(6)#321
                                 - b_customer(11)(2)(6)#621
                                 - w(11)(1)(2)(6)#2133 - w(11)(2)(2)(6)#2163
                                 - w(11)(3)(2)(6)#2193 - w(11)(4)(2)(6)#2223
                                 - w(11)(5)(2)(6)#2253 - w(11)(6)(2)(6)#2283
                                  = -16
 CustomerInventoryBalance(12,2,6): - I_customer(12)(1)(6)#341
                                 + I_customer(12)(2)(6)#351
                                 - b_customer(12)(2)(6)#651
                                 - w(12)(1)(2)(6)#2313 - w(12)(2)(2)(6)#2343
                                 - w(12)(3)(2)(6)#2373 - w(12)(4)(2)(6)#2403
                                 - w(12)(5)(2)(6)#2433 - w(12)(6)(2)(6)#2463
                                  = -19
 CustomerInventoryBalance(3,3,6): - I_customer(3)(2)(6)#81
                                 + I_customer(3)(3)(6)#91
                                 - b_customer(3)(3)(6)#391 - w(3)(1)(3)(6)#703
                                 - w(3)(2)(3)(6)#733 - w(3)(3)(3)(6)#763
                                 - w(3)(4)(3)(6)#793 - w(3)(5)(3)(6)#823
                                 - w(3)(6)(3)(6)#853  = -9
 CustomerInventoryBalance(4,3,6): - I_customer(4)(2)(6)#111
                                 + I_customer(4)(3)(6)#121
                                 - b_customer(4)(3)(6)#421 - w(4)(1)(3)(6)#883
                                 - w(4)(2)(3)(6)#913 - w(4)(3)(3)(6)#943
                                 - w(4)(4)(3)(6)#973 - w(4)(5)(3)(6)#1003
                                 - w(4)(6)(3)(6)#1033  = -16
 CustomerInventoryBalance(5,3,6): - I_customer(5)(2)(6)#141
                                 + I_customer(5)(3)(6)#151
                                 - b_customer(5)(3)(6)#451 - w(5)(1)(3)(6)#1063
                                 - w(5)(2)(3)(6)#1093 - w(5)(3)(3)(6)#1123
                                 - w(5)(4)(3)(6)#1153 - w(5)(5)(3)(6)#1183
                                 - w(5)(6)(3)(6)#1213  = -13
 CustomerInventoryBalance(6,3,6): - I_customer(6)(2)(6)#171
                                 + I_customer(6)(3)(6)#181
                                 - b_customer(6)(3)(6)#481 - w(6)(1)(3)(6)#1243
                                 - w(6)(2)(3)(6)#1273 - w(6)(3)(3)(6)#1303
                                 - w(6)(4)(3)(6)#1333 - w(6)(5)(3)(6)#1363
                                 - w(6)(6)(3)(6)#1393  = -6
 CustomerInventoryBalance(7,3,6): - I_customer(7)(2)(6)#201
                                 + I_customer(7)(3)(6)#211
                                 - b_customer(7)(3)(6)#511 - w(7)(1)(3)(6)#1423
                                 - w(7)(2)(3)(6)#1453 - w(7)(3)(3)(6)#1483
                                 - w(7)(4)(3)(6)#1513 - w(7)(5)(3)(6)#1543
                                 - w(7)(6)(3)(6)#1573  = -14
 CustomerInventoryBalance(8,3,6): - I_customer(8)(2)(6)#231
                                 + I_customer(8)(3)(6)#241
                                 - b_customer(8)(3)(6)#541 - w(8)(1)(3)(6)#1603
                                 - w(8)(2)(3)(6)#1633 - w(8)(3)(3)(6)#1663
                                 - w(8)(4)(3)(6)#1693 - w(8)(5)(3)(6)#1723
                                 - w(8)(6)(3)(6)#1753  = -15
 CustomerInventoryBalance(9,3,6): - I_customer(9)(2)(6)#261
                                 + I_customer(9)(3)(6)#271
                                 - b_customer(9)(3)(6)#571 - w(9)(1)(3)(6)#1783
                                 - w(9)(2)(3)(6)#1813 - w(9)(3)(3)(6)#1843
                                 - w(9)(4)(3)(6)#1873 - w(9)(5)(3)(6)#1903
                                 - w(9)(6)(3)(6)#1933  = -26
 CustomerInventoryBalance(10,3,6): - I_customer(10)(2)(6)#291
                                 + I_customer(10)(3)(6)#301
                                 - b_customer(10)(3)(6)#601
                                 - w(10)(1)(3)(6)#1963 - w(10)(2)(3)(6)#1993
                                 - w(10)(3)(3)(6)#2023 - w(10)(4)(3)(6)#2053
                                 - w(10)(5)(3)(6)#2083 - w(10)(6)(3)(6)#2113
                                  = -14
 CustomerInventoryBalance(11,3,6): - I_customer(11)(2)(6)#321
                                 + I_customer(11)(3)(6)#331
                                 - b_customer(11)(3)(6)#631
                                 - w(11)(1)(3)(6)#2143 - w(11)(2)(3)(6)#2173
                                 - w(11)(3)(3)(6)#2203 - w(11)(4)(3)(6)#2233
                                 - w(11)(5)(3)(6)#2263 - w(11)(6)(3)(6)#2293
                                  = -16
 CustomerInventoryBalance(12,3,6): - I_customer(12)(2)(6)#351
                                 + I_customer(12)(3)(6)#361
                                 - b_customer(12)(3)(6)#661
                                 - w(12)(1)(3)(6)#2323 - w(12)(2)(3)(6)#2353
                                 - w(12)(3)(3)(6)#2383 - w(12)(4)(3)(6)#2413
                                 - w(12)(5)(3)(6)#2443 - w(12)(6)(3)(6)#2473
                                  = -26
 CustomerInventoryBalance(3,1,7): I_customer(3)(1)(7)#72
                                 - b_customer(3)(1)(7)#372 - w(3)(1)(1)(7)#684
                                 - w(3)(2)(1)(7)#714 - w(3)(3)(1)(7)#744
                                 - w(3)(4)(1)(7)#774 - w(3)(5)(1)(7)#804
                                 - w(3)(6)(1)(7)#834  = -7
 CustomerInventoryBalance(4,1,7): I_customer(4)(1)(7)#102
                                 - b_customer(4)(1)(7)#402 - w(4)(1)(1)(7)#864
                                 - w(4)(2)(1)(7)#894 - w(4)(3)(1)(7)#924
                                 - w(4)(4)(1)(7)#954 - w(4)(5)(1)(7)#984
                                 - w(4)(6)(1)(7)#1014  = 2
 CustomerInventoryBalance(5,1,7): I_customer(5)(1)(7)#132
                                 - b_customer(5)(1)(7)#432 - w(5)(1)(1)(7)#1044
                                 - w(5)(2)(1)(7)#1074 - w(5)(3)(1)(7)#1104
                                 - w(5)(4)(1)(7)#1134 - w(5)(5)(1)(7)#1164
                                 - w(5)(6)(1)(7)#1194  = 0
 CustomerInventoryBalance(6,1,7): I_customer(6)(1)(7)#162
                                 - b_customer(6)(1)(7)#462 - w(6)(1)(1)(7)#1224
                                 - w(6)(2)(1)(7)#1254 - w(6)(3)(1)(7)#1284
                                 - w(6)(4)(1)(7)#1314 - w(6)(5)(1)(7)#1344
                                 - w(6)(6)(1)(7)#1374  = -5
 CustomerInventoryBalance(7,1,7): I_customer(7)(1)(7)#192
                                 - b_customer(7)(1)(7)#492 - w(7)(1)(1)(7)#1404
                                 - w(7)(2)(1)(7)#1434 - w(7)(3)(1)(7)#1464
                                 - w(7)(4)(1)(7)#1494 - w(7)(5)(1)(7)#1524
                                 - w(7)(6)(1)(7)#1554  = 1
 CustomerInventoryBalance(8,1,7): I_customer(8)(1)(7)#222
                                 - b_customer(8)(1)(7)#522 - w(8)(1)(1)(7)#1584
                                 - w(8)(2)(1)(7)#1614 - w(8)(3)(1)(7)#1644
                                 - w(8)(4)(1)(7)#1674 - w(8)(5)(1)(7)#1704
                                 - w(8)(6)(1)(7)#1734  = 22
 CustomerInventoryBalance(9,1,7): I_customer(9)(1)(7)#252
                                 - b_customer(9)(1)(7)#552 - w(9)(1)(1)(7)#1764
                                 - w(9)(2)(1)(7)#1794 - w(9)(3)(1)(7)#1824
                                 - w(9)(4)(1)(7)#1854 - w(9)(5)(1)(7)#1884
                                 - w(9)(6)(1)(7)#1914  = 37
 CustomerInventoryBalance(10,1,7): I_customer(10)(1)(7)#282
                                 - b_customer(10)(1)(7)#582
                                 - w(10)(1)(1)(7)#1944 - w(10)(2)(1)(7)#1974
                                 - w(10)(3)(1)(7)#2004 - w(10)(4)(1)(7)#2034
                                 - w(10)(5)(1)(7)#2064 - w(10)(6)(1)(7)#2094
                                  = -7
 CustomerInventoryBalance(11,1,7): I_customer(11)(1)(7)#312
                                 - b_customer(11)(1)(7)#612
                                 - w(11)(1)(1)(7)#2124 - w(11)(2)(1)(7)#2154
                                 - w(11)(3)(1)(7)#2184 - w(11)(4)(1)(7)#2214
                                 - w(11)(5)(1)(7)#2244 - w(11)(6)(1)(7)#2274
                                  = 27
 CustomerInventoryBalance(12,1,7): I_customer(12)(1)(7)#342
                                 - b_customer(12)(1)(7)#642
                                 - w(12)(1)(1)(7)#2304 - w(12)(2)(1)(7)#2334
                                 - w(12)(3)(1)(7)#2364 - w(12)(4)(1)(7)#2394
                                 - w(12)(5)(1)(7)#2424 - w(12)(6)(1)(7)#2454
                                  = 24
 CustomerInventoryBalance(3,2,7): - I_customer(3)(1)(7)#72
                                 + I_customer(3)(2)(7)#82
                                 - b_customer(3)(2)(7)#382 - w(3)(1)(2)(7)#694
                                 - w(3)(2)(2)(7)#724 - w(3)(3)(2)(7)#754
                                 - w(3)(4)(2)(7)#784 - w(3)(5)(2)(7)#814
                                 - w(3)(6)(2)(7)#844  = -11
 CustomerInventoryBalance(4,2,7): - I_customer(4)(1)(7)#102
                                 + I_customer(4)(2)(7)#112
                                 - b_customer(4)(2)(7)#412 - w(4)(1)(2)(7)#874
                                 - w(4)(2)(2)(7)#904 - w(4)(3)(2)(7)#934
                                 - w(4)(4)(2)(7)#964 - w(4)(5)(2)(7)#994
                                 - w(4)(6)(2)(7)#1024  = -17
 CustomerInventoryBalance(5,2,7): - I_customer(5)(1)(7)#132
                                 + I_customer(5)(2)(7)#142
                                 - b_customer(5)(2)(7)#442 - w(5)(1)(2)(7)#1054
                                 - w(5)(2)(2)(7)#1084 - w(5)(3)(2)(7)#1114
                                 - w(5)(4)(2)(7)#1144 - w(5)(5)(2)(7)#1174
                                 - w(5)(6)(2)(7)#1204  = -13
 CustomerInventoryBalance(6,2,7): - I_customer(6)(1)(7)#162
                                 + I_customer(6)(2)(7)#172
                                 - b_customer(6)(2)(7)#472 - w(6)(1)(2)(7)#1234
                                 - w(6)(2)(2)(7)#1264 - w(6)(3)(2)(7)#1294
                                 - w(6)(4)(2)(7)#1324 - w(6)(5)(2)(7)#1354
                                 - w(6)(6)(2)(7)#1384  = -6
 CustomerInventoryBalance(7,2,7): - I_customer(7)(1)(7)#192
                                 + I_customer(7)(2)(7)#202
                                 - b_customer(7)(2)(7)#502 - w(7)(1)(2)(7)#1414
                                 - w(7)(2)(2)(7)#1444 - w(7)(3)(2)(7)#1474
                                 - w(7)(4)(2)(7)#1504 - w(7)(5)(2)(7)#1534
                                 - w(7)(6)(2)(7)#1564  = -13
 CustomerInventoryBalance(8,2,7): - I_customer(8)(1)(7)#222
                                 + I_customer(8)(2)(7)#232
                                 - b_customer(8)(2)(7)#532 - w(8)(1)(2)(7)#1594
                                 - w(8)(2)(2)(7)#1624 - w(8)(3)(2)(7)#1654
                                 - w(8)(4)(2)(7)#1684 - w(8)(5)(2)(7)#1714
                                 - w(8)(6)(2)(7)#1744  = -18
 CustomerInventoryBalance(9,2,7): - I_customer(9)(1)(7)#252
                                 + I_customer(9)(2)(7)#262
                                 - b_customer(9)(2)(7)#562 - w(9)(1)(2)(7)#1774
                                 - w(9)(2)(2)(7)#1804 - w(9)(3)(2)(7)#1834
                                 - w(9)(4)(2)(7)#1864 - w(9)(5)(2)(7)#1894
                                 - w(9)(6)(2)(7)#1924  = -21
 CustomerInventoryBalance(10,2,7): - I_customer(10)(1)(7)#282
                                 + I_customer(10)(2)(7)#292
                                 - b_customer(10)(2)(7)#592
                                 - w(10)(1)(2)(7)#1954 - w(10)(2)(2)(7)#1984
                                 - w(10)(3)(2)(7)#2014 - w(10)(4)(2)(7)#2044
                                 - w(10)(5)(2)(7)#2074 - w(10)(6)(2)(7)#2104
                                  = -14
 CustomerInventoryBalance(11,2,7): - I_customer(11)(1)(7)#312
                                 + I_customer(11)(2)(7)#322
                                 - b_customer(11)(2)(7)#622
                                 - w(11)(1)(2)(7)#2134 - w(11)(2)(2)(7)#2164
                                 - w(11)(3)(2)(7)#2194 - w(11)(4)(2)(7)#2224
                                 - w(11)(5)(2)(7)#2254 - w(11)(6)(2)(7)#2284
                                  = -20
 CustomerInventoryBalance(12,2,7): - I_customer(12)(1)(7)#342
                                 + I_customer(12)(2)(7)#352
                                 - b_customer(12)(2)(7)#652
                                 - w(12)(1)(2)(7)#2314 - w(12)(2)(2)(7)#2344
                                 - w(12)(3)(2)(7)#2374 - w(12)(4)(2)(7)#2404
                                 - w(12)(5)(2)(7)#2434 - w(12)(6)(2)(7)#2464
                                  = -23
 CustomerInventoryBalance(3,3,7): - I_customer(3)(2)(7)#82
                                 + I_customer(3)(3)(7)#92
                                 - b_customer(3)(3)(7)#392 - w(3)(1)(3)(7)#704
                                 - w(3)(2)(3)(7)#734 - w(3)(3)(3)(7)#764
                                 - w(3)(4)(3)(7)#794 - w(3)(5)(3)(7)#824
                                 - w(3)(6)(3)(7)#854  = -10
 CustomerInventoryBalance(4,3,7): - I_customer(4)(2)(7)#112
                                 + I_customer(4)(3)(7)#122
                                 - b_customer(4)(3)(7)#422 - w(4)(1)(3)(7)#884
                                 - w(4)(2)(3)(7)#914 - w(4)(3)(3)(7)#944
                                 - w(4)(4)(3)(7)#974 - w(4)(5)(3)(7)#1004
                                 - w(4)(6)(3)(7)#1034  = -15
 CustomerInventoryBalance(5,3,7): - I_customer(5)(2)(7)#142
                                 + I_customer(5)(3)(7)#152
                                 - b_customer(5)(3)(7)#452 - w(5)(1)(3)(7)#1064
                                 - w(5)(2)(3)(7)#1094 - w(5)(3)(3)(7)#1124
                                 - w(5)(4)(3)(7)#1154 - w(5)(5)(3)(7)#1184
                                 - w(5)(6)(3)(7)#1214  = -12
 CustomerInventoryBalance(6,3,7): - I_customer(6)(2)(7)#172
                                 + I_customer(6)(3)(7)#182
                                 - b_customer(6)(3)(7)#482 - w(6)(1)(3)(7)#1244
                                 - w(6)(2)(3)(7)#1274 - w(6)(3)(3)(7)#1304
                                 - w(6)(4)(3)(7)#1334 - w(6)(5)(3)(7)#1364
                                 - w(6)(6)(3)(7)#1394  = -7
 CustomerInventoryBalance(7,3,7): - I_customer(7)(2)(7)#202
                                 + I_customer(7)(3)(7)#212
                                 - b_customer(7)(3)(7)#512 - w(7)(1)(3)(7)#1424
                                 - w(7)(2)(3)(7)#1454 - w(7)(3)(3)(7)#1484
                                 - w(7)(4)(3)(7)#1514 - w(7)(5)(3)(7)#1544
                                 - w(7)(6)(3)(7)#1574  = -14
 CustomerInventoryBalance(8,3,7): - I_customer(8)(2)(7)#232
                                 + I_customer(8)(3)(7)#242
                                 - b_customer(8)(3)(7)#542 - w(8)(1)(3)(7)#1604
                                 - w(8)(2)(3)(7)#1634 - w(8)(3)(3)(7)#1664
                                 - w(8)(4)(3)(7)#1694 - w(8)(5)(3)(7)#1724
                                 - w(8)(6)(3)(7)#1754  = -14
 CustomerInventoryBalance(9,3,7): - I_customer(9)(2)(7)#262
                                 + I_customer(9)(3)(7)#272
                                 - b_customer(9)(3)(7)#572 - w(9)(1)(3)(7)#1784
                                 - w(9)(2)(3)(7)#1814 - w(9)(3)(3)(7)#1844
                                 - w(9)(4)(3)(7)#1874 - w(9)(5)(3)(7)#1904
                                 - w(9)(6)(3)(7)#1934  = -26
 CustomerInventoryBalance(10,3,7): - I_customer(10)(2)(7)#292
                                 + I_customer(10)(3)(7)#302
                                 - b_customer(10)(3)(7)#602
                                 - w(10)(1)(3)(7)#1964 - w(10)(2)(3)(7)#1994
                                 - w(10)(3)(3)(7)#2024 - w(10)(4)(3)(7)#2054
                                 - w(10)(5)(3)(7)#2084 - w(10)(6)(3)(7)#2114
                                  = -15
 CustomerInventoryBalance(11,3,7): - I_customer(11)(2)(7)#322
                                 + I_customer(11)(3)(7)#332
                                 - b_customer(11)(3)(7)#632
                                 - w(11)(1)(3)(7)#2144 - w(11)(2)(3)(7)#2174
                                 - w(11)(3)(3)(7)#2204 - w(11)(4)(3)(7)#2234
                                 - w(11)(5)(3)(7)#2264 - w(11)(6)(3)(7)#2294
                                  = -21
 CustomerInventoryBalance(12,3,7): - I_customer(12)(2)(7)#352
                                 + I_customer(12)(3)(7)#362
                                 - b_customer(12)(3)(7)#662
                                 - w(12)(1)(3)(7)#2324 - w(12)(2)(3)(7)#2354
                                 - w(12)(3)(3)(7)#2384 - w(12)(4)(3)(7)#2414
                                 - w(12)(5)(3)(7)#2444 - w(12)(6)(3)(7)#2474
                                  = -25
 CustomerInventoryBalance(3,1,8): I_customer(3)(1)(8)#73
                                 - b_customer(3)(1)(8)#373 - w(3)(1)(1)(8)#685
                                 - w(3)(2)(1)(8)#715 - w(3)(3)(1)(8)#745
                                 - w(3)(4)(1)(8)#775 - w(3)(5)(1)(8)#805
                                 - w(3)(6)(1)(8)#835  = -5
 CustomerInventoryBalance(4,1,8): I_customer(4)(1)(8)#103
                                 - b_customer(4)(1)(8)#403 - w(4)(1)(1)(8)#865
                                 - w(4)(2)(1)(8)#895 - w(4)(3)(1)(8)#925
                                 - w(4)(4)(1)(8)#955 - w(4)(5)(1)(8)#985
                                 - w(4)(6)(1)(8)#1015  = 2
 CustomerInventoryBalance(5,1,8): I_customer(5)(1)(8)#133
                                 - b_customer(5)(1)(8)#433 - w(5)(1)(1)(8)#1045
                                 - w(5)(2)(1)(8)#1075 - w(5)(3)(1)(8)#1105
                                 - w(5)(4)(1)(8)#1135 - w(5)(5)(1)(8)#1165
                                 - w(5)(6)(1)(8)#1195  = 2
 CustomerInventoryBalance(6,1,8): I_customer(6)(1)(8)#163
                                 - b_customer(6)(1)(8)#463 - w(6)(1)(1)(8)#1225
                                 - w(6)(2)(1)(8)#1255 - w(6)(3)(1)(8)#1285
                                 - w(6)(4)(1)(8)#1315 - w(6)(5)(1)(8)#1345
                                 - w(6)(6)(1)(8)#1375  = -4
 CustomerInventoryBalance(7,1,8): I_customer(7)(1)(8)#193
                                 - b_customer(7)(1)(8)#493 - w(7)(1)(1)(8)#1405
                                 - w(7)(2)(1)(8)#1435 - w(7)(3)(1)(8)#1465
                                 - w(7)(4)(1)(8)#1495 - w(7)(5)(1)(8)#1525
                                 - w(7)(6)(1)(8)#1555  = -2
 CustomerInventoryBalance(8,1,8): I_customer(8)(1)(8)#223
                                 - b_customer(8)(1)(8)#523 - w(8)(1)(1)(8)#1585
                                 - w(8)(2)(1)(8)#1615 - w(8)(3)(1)(8)#1645
                                 - w(8)(4)(1)(8)#1675 - w(8)(5)(1)(8)#1705
                                 - w(8)(6)(1)(8)#1735  = 27
 CustomerInventoryBalance(9,1,8): I_customer(9)(1)(8)#253
                                 - b_customer(9)(1)(8)#553 - w(9)(1)(1)(8)#1765
                                 - w(9)(2)(1)(8)#1795 - w(9)(3)(1)(8)#1825
                                 - w(9)(4)(1)(8)#1855 - w(9)(5)(1)(8)#1885
                                 - w(9)(6)(1)(8)#1915  = 34
 CustomerInventoryBalance(10,1,8): I_customer(10)(1)(8)#283
                                 - b_customer(10)(1)(8)#583
                                 - w(10)(1)(1)(8)#1945 - w(10)(2)(1)(8)#1975
                                 - w(10)(3)(1)(8)#2005 - w(10)(4)(1)(8)#2035
                                 - w(10)(5)(1)(8)#2065 - w(10)(6)(1)(8)#2095
                                  = -4
 CustomerInventoryBalance(11,1,8): I_customer(11)(1)(8)#313
                                 - b_customer(11)(1)(8)#613
                                 - w(11)(1)(1)(8)#2125 - w(11)(2)(1)(8)#2155
                                 - w(11)(3)(1)(8)#2185 - w(11)(4)(1)(8)#2215
                                 - w(11)(5)(1)(8)#2245 - w(11)(6)(1)(8)#2275
                                  = 25
 CustomerInventoryBalance(12,1,8): I_customer(12)(1)(8)#343
                                 - b_customer(12)(1)(8)#643
                                 - w(12)(1)(1)(8)#2305 - w(12)(2)(1)(8)#2335
                                 - w(12)(3)(1)(8)#2365 - w(12)(4)(1)(8)#2395
                                 - w(12)(5)(1)(8)#2425 - w(12)(6)(1)(8)#2455
                                  = 25
 CustomerInventoryBalance(3,2,8): - I_customer(3)(1)(8)#73
                                 + I_customer(3)(2)(8)#83
                                 - b_customer(3)(2)(8)#383 - w(3)(1)(2)(8)#695
                                 - w(3)(2)(2)(8)#725 - w(3)(3)(2)(8)#755
                                 - w(3)(4)(2)(8)#785 - w(3)(5)(2)(8)#815
                                 - w(3)(6)(2)(8)#845  = -10
 CustomerInventoryBalance(4,2,8): - I_customer(4)(1)(8)#103
                                 + I_customer(4)(2)(8)#113
                                 - b_customer(4)(2)(8)#413 - w(4)(1)(2)(8)#875
                                 - w(4)(2)(2)(8)#905 - w(4)(3)(2)(8)#935
                                 - w(4)(4)(2)(8)#965 - w(4)(5)(2)(8)#995
                                 - w(4)(6)(2)(8)#1025  = -13
 CustomerInventoryBalance(5,2,8): - I_customer(5)(1)(8)#133
                                 + I_customer(5)(2)(8)#143
                                 - b_customer(5)(2)(8)#443 - w(5)(1)(2)(8)#1055
                                 - w(5)(2)(2)(8)#1085 - w(5)(3)(2)(8)#1115
                                 - w(5)(4)(2)(8)#1145 - w(5)(5)(2)(8)#1175
                                 - w(5)(6)(2)(8)#1205  = -13
 CustomerInventoryBalance(6,2,8): - I_customer(6)(1)(8)#163
                                 + I_customer(6)(2)(8)#173
                                 - b_customer(6)(2)(8)#473 - w(6)(1)(2)(8)#1235
                                 - w(6)(2)(2)(8)#1265 - w(6)(3)(2)(8)#1295
                                 - w(6)(4)(2)(8)#1325 - w(6)(5)(2)(8)#1355
                                 - w(6)(6)(2)(8)#1385  = -8
 CustomerInventoryBalance(7,2,8): - I_customer(7)(1)(8)#193
                                 + I_customer(7)(2)(8)#203
                                 - b_customer(7)(2)(8)#503 - w(7)(1)(2)(8)#1415
                                 - w(7)(2)(2)(8)#1445 - w(7)(3)(2)(8)#1475
                                 - w(7)(4)(2)(8)#1505 - w(7)(5)(2)(8)#1535
                                 - w(7)(6)(2)(8)#1565  = -13
 CustomerInventoryBalance(8,2,8): - I_customer(8)(1)(8)#223
                                 + I_customer(8)(2)(8)#233
                                 - b_customer(8)(2)(8)#533 - w(8)(1)(2)(8)#1595
                                 - w(8)(2)(2)(8)#1625 - w(8)(3)(2)(8)#1655
                                 - w(8)(4)(2)(8)#1685 - w(8)(5)(2)(8)#1715
                                 - w(8)(6)(2)(8)#1745  = -17
 CustomerInventoryBalance(9,2,8): - I_customer(9)(1)(8)#253
                                 + I_customer(9)(2)(8)#263
                                 - b_customer(9)(2)(8)#563 - w(9)(1)(2)(8)#1775
                                 - w(9)(2)(2)(8)#1805 - w(9)(3)(2)(8)#1835
                                 - w(9)(4)(2)(8)#1865 - w(9)(5)(2)(8)#1895
                                 - w(9)(6)(2)(8)#1925  = -20
 CustomerInventoryBalance(10,2,8): - I_customer(10)(1)(8)#283
                                 + I_customer(10)(2)(8)#293
                                 - b_customer(10)(2)(8)#593
                                 - w(10)(1)(2)(8)#1955 - w(10)(2)(2)(8)#1985
                                 - w(10)(3)(2)(8)#2015 - w(10)(4)(2)(8)#2045
                                 - w(10)(5)(2)(8)#2075 - w(10)(6)(2)(8)#2105
                                  = -11
 CustomerInventoryBalance(11,2,8): - I_customer(11)(1)(8)#313
                                 + I_customer(11)(2)(8)#323
                                 - b_customer(11)(2)(8)#623
                                 - w(11)(1)(2)(8)#2135 - w(11)(2)(2)(8)#2165
                                 - w(11)(3)(2)(8)#2195 - w(11)(4)(2)(8)#2225
                                 - w(11)(5)(2)(8)#2255 - w(11)(6)(2)(8)#2285
                                  = -17
 CustomerInventoryBalance(12,2,8): - I_customer(12)(1)(8)#343
                                 + I_customer(12)(2)(8)#353
                                 - b_customer(12)(2)(8)#653
                                 - w(12)(1)(2)(8)#2315 - w(12)(2)(2)(8)#2345
                                 - w(12)(3)(2)(8)#2375 - w(12)(4)(2)(8)#2405
                                 - w(12)(5)(2)(8)#2435 - w(12)(6)(2)(8)#2465
                                  = -26
 CustomerInventoryBalance(3,3,8): - I_customer(3)(2)(8)#83
                                 + I_customer(3)(3)(8)#93
                                 - b_customer(3)(3)(8)#393 - w(3)(1)(3)(8)#705
                                 - w(3)(2)(3)(8)#735 - w(3)(3)(3)(8)#765
                                 - w(3)(4)(3)(8)#795 - w(3)(5)(3)(8)#825
                                 - w(3)(6)(3)(8)#855  = -8
 CustomerInventoryBalance(4,3,8): - I_customer(4)(2)(8)#113
                                 + I_customer(4)(3)(8)#123
                                 - b_customer(4)(3)(8)#423 - w(4)(1)(3)(8)#885
                                 - w(4)(2)(3)(8)#915 - w(4)(3)(3)(8)#945
                                 - w(4)(4)(3)(8)#975 - w(4)(5)(3)(8)#1005
                                 - w(4)(6)(3)(8)#1035  = -16
 CustomerInventoryBalance(5,3,8): - I_customer(5)(2)(8)#143
                                 + I_customer(5)(3)(8)#153
                                 - b_customer(5)(3)(8)#453 - w(5)(1)(3)(8)#1065
                                 - w(5)(2)(3)(8)#1095 - w(5)(3)(3)(8)#1125
                                 - w(5)(4)(3)(8)#1155 - w(5)(5)(3)(8)#1185
                                 - w(5)(6)(3)(8)#1215  = -17
 CustomerInventoryBalance(6,3,8): - I_customer(6)(2)(8)#173
                                 + I_customer(6)(3)(8)#183
                                 - b_customer(6)(3)(8)#483 - w(6)(1)(3)(8)#1245
                                 - w(6)(2)(3)(8)#1275 - w(6)(3)(3)(8)#1305
                                 - w(6)(4)(3)(8)#1335 - w(6)(5)(3)(8)#1365
                                 - w(6)(6)(3)(8)#1395  = -7
 CustomerInventoryBalance(7,3,8): - I_customer(7)(2)(8)#203
                                 + I_customer(7)(3)(8)#213
                                 - b_customer(7)(3)(8)#513 - w(7)(1)(3)(8)#1425
                                 - w(7)(2)(3)(8)#1455 - w(7)(3)(3)(8)#1485
                                 - w(7)(4)(3)(8)#1515 - w(7)(5)(3)(8)#1545
                                 - w(7)(6)(3)(8)#1575  = -13
 CustomerInventoryBalance(8,3,8): - I_customer(8)(2)(8)#233
                                 + I_customer(8)(3)(8)#243
                                 - b_customer(8)(3)(8)#543 - w(8)(1)(3)(8)#1605
                                 - w(8)(2)(3)(8)#1635 - w(8)(3)(3)(8)#1665
                                 - w(8)(4)(3)(8)#1695 - w(8)(5)(3)(8)#1725
                                 - w(8)(6)(3)(8)#1755  = -19
 CustomerInventoryBalance(9,3,8): - I_customer(9)(2)(8)#263
                                 + I_customer(9)(3)(8)#273
                                 - b_customer(9)(3)(8)#573 - w(9)(1)(3)(8)#1785
                                 - w(9)(2)(3)(8)#1815 - w(9)(3)(3)(8)#1845
                                 - w(9)(4)(3)(8)#1875 - w(9)(5)(3)(8)#1905
                                 - w(9)(6)(3)(8)#1935  = -25
 CustomerInventoryBalance(10,3,8): - I_customer(10)(2)(8)#293
                                 + I_customer(10)(3)(8)#303
                                 - b_customer(10)(3)(8)#603
                                 - w(10)(1)(3)(8)#1965 - w(10)(2)(3)(8)#1995
                                 - w(10)(3)(3)(8)#2025 - w(10)(4)(3)(8)#2055
                                 - w(10)(5)(3)(8)#2085 - w(10)(6)(3)(8)#2115
                                  = -14
 CustomerInventoryBalance(11,3,8): - I_customer(11)(2)(8)#323
                                 + I_customer(11)(3)(8)#333
                                 - b_customer(11)(3)(8)#633
                                 - w(11)(1)(3)(8)#2145 - w(11)(2)(3)(8)#2175
                                 - w(11)(3)(3)(8)#2205 - w(11)(4)(3)(8)#2235
                                 - w(11)(5)(3)(8)#2265 - w(11)(6)(3)(8)#2295
                                  = -19
 CustomerInventoryBalance(12,3,8): - I_customer(12)(2)(8)#353
                                 + I_customer(12)(3)(8)#363
                                 - b_customer(12)(3)(8)#663
                                 - w(12)(1)(3)(8)#2325 - w(12)(2)(3)(8)#2355
                                 - w(12)(3)(3)(8)#2385 - w(12)(4)(3)(8)#2415
                                 - w(12)(5)(3)(8)#2445 - w(12)(6)(3)(8)#2475
                                  = -22
 CustomerInventoryBalance(3,1,9): I_customer(3)(1)(9)#74
                                 - b_customer(3)(1)(9)#374 - w(3)(1)(1)(9)#686
                                 - w(3)(2)(1)(9)#716 - w(3)(3)(1)(9)#746
                                 - w(3)(4)(1)(9)#776 - w(3)(5)(1)(9)#806
                                 - w(3)(6)(1)(9)#836  = -7
 CustomerInventoryBalance(4,1,9): I_customer(4)(1)(9)#104
                                 - b_customer(4)(1)(9)#404 - w(4)(1)(1)(9)#866
                                 - w(4)(2)(1)(9)#896 - w(4)(3)(1)(9)#926
                                 - w(4)(4)(1)(9)#956 - w(4)(5)(1)(9)#986
                                 - w(4)(6)(1)(9)#1016  = -3
 CustomerInventoryBalance(5,1,9): I_customer(5)(1)(9)#134
                                 - b_customer(5)(1)(9)#434 - w(5)(1)(1)(9)#1046
                                 - w(5)(2)(1)(9)#1076 - w(5)(3)(1)(9)#1106
                                 - w(5)(4)(1)(9)#1136 - w(5)(5)(1)(9)#1166
                                 - w(5)(6)(1)(9)#1196  = 2
 CustomerInventoryBalance(6,1,9): I_customer(6)(1)(9)#164
                                 - b_customer(6)(1)(9)#464 - w(6)(1)(1)(9)#1226
                                 - w(6)(2)(1)(9)#1256 - w(6)(3)(1)(9)#1286
                                 - w(6)(4)(1)(9)#1316 - w(6)(5)(1)(9)#1346
                                 - w(6)(6)(1)(9)#1376  = -4
 CustomerInventoryBalance(7,1,9): I_customer(7)(1)(9)#194
                                 - b_customer(7)(1)(9)#494 - w(7)(1)(1)(9)#1406
                                 - w(7)(2)(1)(9)#1436 - w(7)(3)(1)(9)#1466
                                 - w(7)(4)(1)(9)#1496 - w(7)(5)(1)(9)#1526
                                 - w(7)(6)(1)(9)#1556  = -1
 CustomerInventoryBalance(8,1,9): I_customer(8)(1)(9)#224
                                 - b_customer(8)(1)(9)#524 - w(8)(1)(1)(9)#1586
                                 - w(8)(2)(1)(9)#1616 - w(8)(3)(1)(9)#1646
                                 - w(8)(4)(1)(9)#1676 - w(8)(5)(1)(9)#1706
                                 - w(8)(6)(1)(9)#1736  = 23
 CustomerInventoryBalance(9,1,9): I_customer(9)(1)(9)#254
                                 - b_customer(9)(1)(9)#554 - w(9)(1)(1)(9)#1766
                                 - w(9)(2)(1)(9)#1796 - w(9)(3)(1)(9)#1826
                                 - w(9)(4)(1)(9)#1856 - w(9)(5)(1)(9)#1886
                                 - w(9)(6)(1)(9)#1916  = 31
 CustomerInventoryBalance(10,1,9): I_customer(10)(1)(9)#284
                                 - b_customer(10)(1)(9)#584
                                 - w(10)(1)(1)(9)#1946 - w(10)(2)(1)(9)#1976
                                 - w(10)(3)(1)(9)#2006 - w(10)(4)(1)(9)#2036
                                 - w(10)(5)(1)(9)#2066 - w(10)(6)(1)(9)#2096
                                  = -5
 CustomerInventoryBalance(11,1,9): I_customer(11)(1)(9)#314
                                 - b_customer(11)(1)(9)#614
                                 - w(11)(1)(1)(9)#2126 - w(11)(2)(1)(9)#2156
                                 - w(11)(3)(1)(9)#2186 - w(11)(4)(1)(9)#2216
                                 - w(11)(5)(1)(9)#2246 - w(11)(6)(1)(9)#2276
                                  = 26
 CustomerInventoryBalance(12,1,9): I_customer(12)(1)(9)#344
                                 - b_customer(12)(1)(9)#644
                                 - w(12)(1)(1)(9)#2306 - w(12)(2)(1)(9)#2336
                                 - w(12)(3)(1)(9)#2366 - w(12)(4)(1)(9)#2396
                                 - w(12)(5)(1)(9)#2426 - w(12)(6)(1)(9)#2456
                                  = 26
 CustomerInventoryBalance(3,2,9): - I_customer(3)(1)(9)#74
                                 + I_customer(3)(2)(9)#84
                                 - b_customer(3)(2)(9)#384 - w(3)(1)(2)(9)#696
                                 - w(3)(2)(2)(9)#726 - w(3)(3)(2)(9)#756
                                 - w(3)(4)(2)(9)#786 - w(3)(5)(2)(9)#816
                                 - w(3)(6)(2)(9)#846  = -10
 CustomerInventoryBalance(4,2,9): - I_customer(4)(1)(9)#104
                                 + I_customer(4)(2)(9)#114
                                 - b_customer(4)(2)(9)#414 - w(4)(1)(2)(9)#876
                                 - w(4)(2)(2)(9)#906 - w(4)(3)(2)(9)#936
                                 - w(4)(4)(2)(9)#966 - w(4)(5)(2)(9)#996
                                 - w(4)(6)(2)(9)#1026  = -15
 CustomerInventoryBalance(5,2,9): - I_customer(5)(1)(9)#134
                                 + I_customer(5)(2)(9)#144
                                 - b_customer(5)(2)(9)#444 - w(5)(1)(2)(9)#1056
                                 - w(5)(2)(2)(9)#1086 - w(5)(3)(2)(9)#1116
                                 - w(5)(4)(2)(9)#1146 - w(5)(5)(2)(9)#1176
                                 - w(5)(6)(2)(9)#1206  = -17
 CustomerInventoryBalance(6,2,9): - I_customer(6)(1)(9)#164
                                 + I_customer(6)(2)(9)#174
                                 - b_customer(6)(2)(9)#474 - w(6)(1)(2)(9)#1236
                                 - w(6)(2)(2)(9)#1266 - w(6)(3)(2)(9)#1296
                                 - w(6)(4)(2)(9)#1326 - w(6)(5)(2)(9)#1356
                                 - w(6)(6)(2)(9)#1386  = -7
 CustomerInventoryBalance(7,2,9): - I_customer(7)(1)(9)#194
                                 + I_customer(7)(2)(9)#204
                                 - b_customer(7)(2)(9)#504 - w(7)(1)(2)(9)#1416
                                 - w(7)(2)(2)(9)#1446 - w(7)(3)(2)(9)#1476
                                 - w(7)(4)(2)(9)#1506 - w(7)(5)(2)(9)#1536
                                 - w(7)(6)(2)(9)#1566  = -13
 CustomerInventoryBalance(8,2,9): - I_customer(8)(1)(9)#224
                                 + I_customer(8)(2)(9)#234
                                 - b_customer(8)(2)(9)#534 - w(8)(1)(2)(9)#1596
                                 - w(8)(2)(2)(9)#1626 - w(8)(3)(2)(9)#1656
                                 - w(8)(4)(2)(9)#1686 - w(8)(5)(2)(9)#1716
                                 - w(8)(6)(2)(9)#1746  = -15
 CustomerInventoryBalance(9,2,9): - I_customer(9)(1)(9)#254
                                 + I_customer(9)(2)(9)#264
                                 - b_customer(9)(2)(9)#564 - w(9)(1)(2)(9)#1776
                                 - w(9)(2)(2)(9)#1806 - w(9)(3)(2)(9)#1836
                                 - w(9)(4)(2)(9)#1866 - w(9)(5)(2)(9)#1896
                                 - w(9)(6)(2)(9)#1926  = -25
 CustomerInventoryBalance(10,2,9): - I_customer(10)(1)(9)#284
                                 + I_customer(10)(2)(9)#294
                                 - b_customer(10)(2)(9)#594
                                 - w(10)(1)(2)(9)#1956 - w(10)(2)(2)(9)#1986
                                 - w(10)(3)(2)(9)#2016 - w(10)(4)(2)(9)#2046
                                 - w(10)(5)(2)(9)#2076 - w(10)(6)(2)(9)#2106
                                  = -11
 CustomerInventoryBalance(11,2,9): - I_customer(11)(1)(9)#314
                                 + I_customer(11)(2)(9)#324
                                 - b_customer(11)(2)(9)#624
                                 - w(11)(1)(2)(9)#2136 - w(11)(2)(2)(9)#2166
                                 - w(11)(3)(2)(9)#2196 - w(11)(4)(2)(9)#2226
                                 - w(11)(5)(2)(9)#2256 - w(11)(6)(2)(9)#2286
                                  = -15
 CustomerInventoryBalance(12,2,9): - I_customer(12)(1)(9)#344
                                 + I_customer(12)(2)(9)#354
                                 - b_customer(12)(2)(9)#654
                                 - w(12)(1)(2)(9)#2316 - w(12)(2)(2)(9)#2346
                                 - w(12)(3)(2)(9)#2376 - w(12)(4)(2)(9)#2406
                                 - w(12)(5)(2)(9)#2436 - w(12)(6)(2)(9)#2466
                                  = -25
 CustomerInventoryBalance(3,3,9): - I_customer(3)(2)(9)#84
                                 + I_customer(3)(3)(9)#94
                                 - b_customer(3)(3)(9)#394 - w(3)(1)(3)(9)#706
                                 - w(3)(2)(3)(9)#736 - w(3)(3)(3)(9)#766
                                 - w(3)(4)(3)(9)#796 - w(3)(5)(3)(9)#826
                                 - w(3)(6)(3)(9)#856  = -8
 CustomerInventoryBalance(4,3,9): - I_customer(4)(2)(9)#114
                                 + I_customer(4)(3)(9)#124
                                 - b_customer(4)(3)(9)#424 - w(4)(1)(3)(9)#886
                                 - w(4)(2)(3)(9)#916 - w(4)(3)(3)(9)#946
                                 - w(4)(4)(3)(9)#976 - w(4)(5)(3)(9)#1006
                                 - w(4)(6)(3)(9)#1036  = -18
 CustomerInventoryBalance(5,3,9): - I_customer(5)(2)(9)#144
                                 + I_customer(5)(3)(9)#154
                                 - b_customer(5)(3)(9)#454 - w(5)(1)(3)(9)#1066
                                 - w(5)(2)(3)(9)#1096 - w(5)(3)(3)(9)#1126
                                 - w(5)(4)(3)(9)#1156 - w(5)(5)(3)(9)#1186
                                 - w(5)(6)(3)(9)#1216  = -13
 CustomerInventoryBalance(6,3,9): - I_customer(6)(2)(9)#174
                                 + I_customer(6)(3)(9)#184
                                 - b_customer(6)(3)(9)#484 - w(6)(1)(3)(9)#1246
                                 - w(6)(2)(3)(9)#1276 - w(6)(3)(3)(9)#1306
                                 - w(6)(4)(3)(9)#1336 - w(6)(5)(3)(9)#1366
                                 - w(6)(6)(3)(9)#1396  = -6
 CustomerInventoryBalance(7,3,9): - I_customer(7)(2)(9)#204
                                 + I_customer(7)(3)(9)#214
                                 - b_customer(7)(3)(9)#514 - w(7)(1)(3)(9)#1426
                                 - w(7)(2)(3)(9)#1456 - w(7)(3)(3)(9)#1486
                                 - w(7)(4)(3)(9)#1516 - w(7)(5)(3)(9)#1546
                                 - w(7)(6)(3)(9)#1576  = -13
 CustomerInventoryBalance(8,3,9): - I_customer(8)(2)(9)#234
                                 + I_customer(8)(3)(9)#244
                                 - b_customer(8)(3)(9)#544 - w(8)(1)(3)(9)#1606
                                 - w(8)(2)(3)(9)#1636 - w(8)(3)(3)(9)#1666
                                 - w(8)(4)(3)(9)#1696 - w(8)(5)(3)(9)#1726
                                 - w(8)(6)(3)(9)#1756  = -13
 CustomerInventoryBalance(9,3,9): - I_customer(9)(2)(9)#264
                                 + I_customer(9)(3)(9)#274
                                 - b_customer(9)(3)(9)#574 - w(9)(1)(3)(9)#1786
                                 - w(9)(2)(3)(9)#1816 - w(9)(3)(3)(9)#1846
                                 - w(9)(4)(3)(9)#1876 - w(9)(5)(3)(9)#1906
                                 - w(9)(6)(3)(9)#1936  = -24
 CustomerInventoryBalance(10,3,9): - I_customer(10)(2)(9)#294
                                 + I_customer(10)(3)(9)#304
                                 - b_customer(10)(3)(9)#604
                                 - w(10)(1)(3)(9)#1966 - w(10)(2)(3)(9)#1996
                                 - w(10)(3)(3)(9)#2026 - w(10)(4)(3)(9)#2056
                                 - w(10)(5)(3)(9)#2086 - w(10)(6)(3)(9)#2116
                                  = -15
 CustomerInventoryBalance(11,3,9): - I_customer(11)(2)(9)#324
                                 + I_customer(11)(3)(9)#334
                                 - b_customer(11)(3)(9)#634
                                 - w(11)(1)(3)(9)#2146 - w(11)(2)(3)(9)#2176
                                 - w(11)(3)(3)(9)#2206 - w(11)(4)(3)(9)#2236
                                 - w(11)(5)(3)(9)#2266 - w(11)(6)(3)(9)#2296
                                  = -19
 CustomerInventoryBalance(12,3,9): - I_customer(12)(2)(9)#354
                                 + I_customer(12)(3)(9)#364
                                 - b_customer(12)(3)(9)#664
                                 - w(12)(1)(3)(9)#2326 - w(12)(2)(3)(9)#2356
                                 - w(12)(3)(3)(9)#2386 - w(12)(4)(3)(9)#2416
                                 - w(12)(5)(3)(9)#2446 - w(12)(6)(3)(9)#2476
                                  = -21
 CustomerInventoryBalance(3,1,10): I_customer(3)(1)(10)#75
                                 - b_customer(3)(1)(10)#375
                                 - w(3)(1)(1)(10)#687 - w(3)(2)(1)(10)#717
                                 - w(3)(3)(1)(10)#747 - w(3)(4)(1)(10)#777
                                 - w(3)(5)(1)(10)#807 - w(3)(6)(1)(10)#837
                                  = -6
 CustomerInventoryBalance(4,1,10): I_customer(4)(1)(10)#105
                                 - b_customer(4)(1)(10)#405
                                 - w(4)(1)(1)(10)#867 - w(4)(2)(1)(10)#897
                                 - w(4)(3)(1)(10)#927 - w(4)(4)(1)(10)#957
                                 - w(4)(5)(1)(10)#987 - w(4)(6)(1)(10)#1017
                                  = 1
 CustomerInventoryBalance(5,1,10): I_customer(5)(1)(10)#135
                                 - b_customer(5)(1)(10)#435
                                 - w(5)(1)(1)(10)#1047 - w(5)(2)(1)(10)#1077
                                 - w(5)(3)(1)(10)#1107 - w(5)(4)(1)(10)#1137
                                 - w(5)(5)(1)(10)#1167 - w(5)(6)(1)(10)#1197
                                  = 0
 CustomerInventoryBalance(6,1,10): I_customer(6)(1)(10)#165
                                 - b_customer(6)(1)(10)#465
                                 - w(6)(1)(1)(10)#1227 - w(6)(2)(1)(10)#1257
                                 - w(6)(3)(1)(10)#1287 - w(6)(4)(1)(10)#1317
                                 - w(6)(5)(1)(10)#1347 - w(6)(6)(1)(10)#1377
                                  = -4
 CustomerInventoryBalance(7,1,10): I_customer(7)(1)(10)#195
                                 - b_customer(7)(1)(10)#495
                                 - w(7)(1)(1)(10)#1407 - w(7)(2)(1)(10)#1437
                                 - w(7)(3)(1)(10)#1467 - w(7)(4)(1)(10)#1497
                                 - w(7)(5)(1)(10)#1527 - w(7)(6)(1)(10)#1557
                                  = -2
 CustomerInventoryBalance(8,1,10): I_customer(8)(1)(10)#225
                                 - b_customer(8)(1)(10)#525
                                 - w(8)(1)(1)(10)#1587 - w(8)(2)(1)(10)#1617
                                 - w(8)(3)(1)(10)#1647 - w(8)(4)(1)(10)#1677
                                 - w(8)(5)(1)(10)#1707 - w(8)(6)(1)(10)#1737
                                  = 24
 CustomerInventoryBalance(9,1,10): I_customer(9)(1)(10)#255
                                 - b_customer(9)(1)(10)#555
                                 - w(9)(1)(1)(10)#1767 - w(9)(2)(1)(10)#1797
                                 - w(9)(3)(1)(10)#1827 - w(9)(4)(1)(10)#1857
                                 - w(9)(5)(1)(10)#1887 - w(9)(6)(1)(10)#1917
                                  = 35
 CustomerInventoryBalance(10,1,10): I_customer(10)(1)(10)#285
                                 - b_customer(10)(1)(10)#585
                                 - w(10)(1)(1)(10)#1947 - w(10)(2)(1)(10)#1977
                                 - w(10)(3)(1)(10)#2007 - w(10)(4)(1)(10)#2037
                                 - w(10)(5)(1)(10)#2067 - w(10)(6)(1)(10)#2097
                                  = -5
 CustomerInventoryBalance(11,1,10): I_customer(11)(1)(10)#315
                                 - b_customer(11)(1)(10)#615
                                 - w(11)(1)(1)(10)#2127 - w(11)(2)(1)(10)#2157
                                 - w(11)(3)(1)(10)#2187 - w(11)(4)(1)(10)#2217
                                 - w(11)(5)(1)(10)#2247 - w(11)(6)(1)(10)#2277
                                  = 28
 CustomerInventoryBalance(12,1,10): I_customer(12)(1)(10)#345
                                 - b_customer(12)(1)(10)#645
                                 - w(12)(1)(1)(10)#2307 - w(12)(2)(1)(10)#2337
                                 - w(12)(3)(1)(10)#2367 - w(12)(4)(1)(10)#2397
                                 - w(12)(5)(1)(10)#2427 - w(12)(6)(1)(10)#2457
                                  = 26
 CustomerInventoryBalance(3,2,10): - I_customer(3)(1)(10)#75
                                 + I_customer(3)(2)(10)#85
                                 - b_customer(3)(2)(10)#385
                                 - w(3)(1)(2)(10)#697 - w(3)(2)(2)(10)#727
                                 - w(3)(3)(2)(10)#757 - w(3)(4)(2)(10)#787
                                 - w(3)(5)(2)(10)#817 - w(3)(6)(2)(10)#847
                                  = -10
 CustomerInventoryBalance(4,2,10): - I_customer(4)(1)(10)#105
                                 + I_customer(4)(2)(10)#115
                                 - b_customer(4)(2)(10)#415
                                 - w(4)(1)(2)(10)#877 - w(4)(2)(2)(10)#907
                                 - w(4)(3)(2)(10)#937 - w(4)(4)(2)(10)#967
                                 - w(4)(5)(2)(10)#997 - w(4)(6)(2)(10)#1027
                                  = -14
 CustomerInventoryBalance(5,2,10): - I_customer(5)(1)(10)#135
                                 + I_customer(5)(2)(10)#145
                                 - b_customer(5)(2)(10)#445
                                 - w(5)(1)(2)(10)#1057 - w(5)(2)(2)(10)#1087
                                 - w(5)(3)(2)(10)#1117 - w(5)(4)(2)(10)#1147
                                 - w(5)(5)(2)(10)#1177 - w(5)(6)(2)(10)#1207
                                  = -13
 CustomerInventoryBalance(6,2,10): - I_customer(6)(1)(10)#165
                                 + I_customer(6)(2)(10)#175
                                 - b_customer(6)(2)(10)#475
                                 - w(6)(1)(2)(10)#1237 - w(6)(2)(2)(10)#1267
                                 - w(6)(3)(2)(10)#1297 - w(6)(4)(2)(10)#1327
                                 - w(6)(5)(2)(10)#1357 - w(6)(6)(2)(10)#1387
                                  = -8
 CustomerInventoryBalance(7,2,10): - I_customer(7)(1)(10)#195
                                 + I_customer(7)(2)(10)#205
                                 - b_customer(7)(2)(10)#505
                                 - w(7)(1)(2)(10)#1417 - w(7)(2)(2)(10)#1447
                                 - w(7)(3)(2)(10)#1477 - w(7)(4)(2)(10)#1507
                                 - w(7)(5)(2)(10)#1537 - w(7)(6)(2)(10)#1567
                                  = -13
 CustomerInventoryBalance(8,2,10): - I_customer(8)(1)(10)#225
                                 + I_customer(8)(2)(10)#235
                                 - b_customer(8)(2)(10)#535
                                 - w(8)(1)(2)(10)#1597 - w(8)(2)(2)(10)#1627
                                 - w(8)(3)(2)(10)#1657 - w(8)(4)(2)(10)#1687
                                 - w(8)(5)(2)(10)#1717 - w(8)(6)(2)(10)#1747
                                  = -15
 CustomerInventoryBalance(9,2,10): - I_customer(9)(1)(10)#255
                                 + I_customer(9)(2)(10)#265
                                 - b_customer(9)(2)(10)#565
                                 - w(9)(1)(2)(10)#1777 - w(9)(2)(2)(10)#1807
                                 - w(9)(3)(2)(10)#1837 - w(9)(4)(2)(10)#1867
                                 - w(9)(5)(2)(10)#1897 - w(9)(6)(2)(10)#1927
                                  = -25
 CustomerInventoryBalance(10,2,10): - I_customer(10)(1)(10)#285
                                 + I_customer(10)(2)(10)#295
                                 - b_customer(10)(2)(10)#595
                                 - w(10)(1)(2)(10)#1957 - w(10)(2)(2)(10)#1987
                                 - w(10)(3)(2)(10)#2017 - w(10)(4)(2)(10)#2047
                                 - w(10)(5)(2)(10)#2077 - w(10)(6)(2)(10)#2107
                                  = -13
 CustomerInventoryBalance(11,2,10): - I_customer(11)(1)(10)#315
                                 + I_customer(11)(2)(10)#325
                                 - b_customer(11)(2)(10)#625
                                 - w(11)(1)(2)(10)#2137 - w(11)(2)(2)(10)#2167
                                 - w(11)(3)(2)(10)#2197 - w(11)(4)(2)(10)#2227
                                 - w(11)(5)(2)(10)#2257 - w(11)(6)(2)(10)#2287
                                  = -18
 CustomerInventoryBalance(12,2,10): - I_customer(12)(1)(10)#345
                                 + I_customer(12)(2)(10)#355
                                 - b_customer(12)(2)(10)#655
                                 - w(12)(1)(2)(10)#2317 - w(12)(2)(2)(10)#2347
                                 - w(12)(3)(2)(10)#2377 - w(12)(4)(2)(10)#2407
                                 - w(12)(5)(2)(10)#2437 - w(12)(6)(2)(10)#2467
                                  = -18
 CustomerInventoryBalance(3,3,10): - I_customer(3)(2)(10)#85
                                 + I_customer(3)(3)(10)#95
                                 - b_customer(3)(3)(10)#395
                                 - w(3)(1)(3)(10)#707 - w(3)(2)(3)(10)#737
                                 - w(3)(3)(3)(10)#767 - w(3)(4)(3)(10)#797
                                 - w(3)(5)(3)(10)#827 - w(3)(6)(3)(10)#857
                                  = -11
 CustomerInventoryBalance(4,3,10): - I_customer(4)(2)(10)#115
                                 + I_customer(4)(3)(10)#125
                                 - b_customer(4)(3)(10)#425
                                 - w(4)(1)(3)(10)#887 - w(4)(2)(3)(10)#917
                                 - w(4)(3)(3)(10)#947 - w(4)(4)(3)(10)#977
                                 - w(4)(5)(3)(10)#1007 - w(4)(6)(3)(10)#1037
                                  = -14
 CustomerInventoryBalance(5,3,10): - I_customer(5)(2)(10)#145
                                 + I_customer(5)(3)(10)#155
                                 - b_customer(5)(3)(10)#455
                                 - w(5)(1)(3)(10)#1067 - w(5)(2)(3)(10)#1097
                                 - w(5)(3)(3)(10)#1127 - w(5)(4)(3)(10)#1157
                                 - w(5)(5)(3)(10)#1187 - w(5)(6)(3)(10)#1217
                                  = -16
 CustomerInventoryBalance(6,3,10): - I_customer(6)(2)(10)#175
                                 + I_customer(6)(3)(10)#185
                                 - b_customer(6)(3)(10)#485
                                 - w(6)(1)(3)(10)#1247 - w(6)(2)(3)(10)#1277
                                 - w(6)(3)(3)(10)#1307 - w(6)(4)(3)(10)#1337
                                 - w(6)(5)(3)(10)#1367 - w(6)(6)(3)(10)#1397
                                  = -8
 CustomerInventoryBalance(7,3,10): - I_customer(7)(2)(10)#205
                                 + I_customer(7)(3)(10)#215
                                 - b_customer(7)(3)(10)#515
                                 - w(7)(1)(3)(10)#1427 - w(7)(2)(3)(10)#1457
                                 - w(7)(3)(3)(10)#1487 - w(7)(4)(3)(10)#1517
                                 - w(7)(5)(3)(10)#1547 - w(7)(6)(3)(10)#1577
                                  = -12
 CustomerInventoryBalance(8,3,10): - I_customer(8)(2)(10)#235
                                 + I_customer(8)(3)(10)#245
                                 - b_customer(8)(3)(10)#545
                                 - w(8)(1)(3)(10)#1607 - w(8)(2)(3)(10)#1637
                                 - w(8)(3)(3)(10)#1667 - w(8)(4)(3)(10)#1697
                                 - w(8)(5)(3)(10)#1727 - w(8)(6)(3)(10)#1757
                                  = -15
 CustomerInventoryBalance(9,3,10): - I_customer(9)(2)(10)#265
                                 + I_customer(9)(3)(10)#275
                                 - b_customer(9)(3)(10)#575
                                 - w(9)(1)(3)(10)#1787 - w(9)(2)(3)(10)#1817
                                 - w(9)(3)(3)(10)#1847 - w(9)(4)(3)(10)#1877
                                 - w(9)(5)(3)(10)#1907 - w(9)(6)(3)(10)#1937
                                  = -25
 CustomerInventoryBalance(10,3,10): - I_customer(10)(2)(10)#295
                                 + I_customer(10)(3)(10)#305
                                 - b_customer(10)(3)(10)#605
                                 - w(10)(1)(3)(10)#1967 - w(10)(2)(3)(10)#1997
                                 - w(10)(3)(3)(10)#2027 - w(10)(4)(3)(10)#2057
                                 - w(10)(5)(3)(10)#2087 - w(10)(6)(3)(10)#2117
                                  = -13
 CustomerInventoryBalance(11,3,10): - I_customer(11)(2)(10)#325
                                 + I_customer(11)(3)(10)#335
                                 - b_customer(11)(3)(10)#635
                                 - w(11)(1)(3)(10)#2147 - w(11)(2)(3)(10)#2177
                                 - w(11)(3)(3)(10)#2207 - w(11)(4)(3)(10)#2237
                                 - w(11)(5)(3)(10)#2267 - w(11)(6)(3)(10)#2297
                                  = -17
 CustomerInventoryBalance(12,3,10): - I_customer(12)(2)(10)#355
                                 + I_customer(12)(3)(10)#365
                                 - b_customer(12)(3)(10)#665
                                 - w(12)(1)(3)(10)#2327 - w(12)(2)(3)(10)#2357
                                 - w(12)(3)(3)(10)#2387 - w(12)(4)(3)(10)#2417
                                 - w(12)(5)(3)(10)#2447 - w(12)(6)(3)(10)#2477
                                  = -25
 CustomerInventoryCapacity(3,1,1): I_customer(3)(1)(1)#66 <= 10
 CustomerInventoryCapacity(4,1,1): I_customer(4)(1)(1)#96 <= 27
 CustomerInventoryCapacity(5,1,1): I_customer(5)(1)(1)#126 <= 32
 CustomerInventoryCapacity(6,1,1): I_customer(6)(1)(1)#156 <= 8
 CustomerInventoryCapacity(7,1,1): I_customer(7)(1)(1)#186 <= 24
 CustomerInventoryCapacity(8,1,1): I_customer(8)(1)(1)#216 <= 83
 CustomerInventoryCapacity(9,1,1): I_customer(9)(1)(1)#246 <= 106
 CustomerInventoryCapacity(10,1,1): I_customer(10)(1)(1)#276 <= 14
 CustomerInventoryCapacity(11,1,1): I_customer(11)(1)(1)#306 <= 99
 CustomerInventoryCapacity(12,1,1): I_customer(12)(1)(1)#336 <= 89
 CustomerInventoryCapacity(3,2,1): I_customer(3)(2)(1)#76 <= 11
 CustomerInventoryCapacity(4,2,1): I_customer(4)(2)(1)#106 <= 33
 CustomerInventoryCapacity(5,2,1): I_customer(5)(2)(1)#136 <= 30
 CustomerInventoryCapacity(6,2,1): I_customer(6)(2)(1)#166 <= 7
 CustomerInventoryCapacity(7,2,1): I_customer(7)(2)(1)#196 <= 24
 CustomerInventoryCapacity(8,2,1): I_customer(8)(2)(1)#226 <= 78
 CustomerInventoryCapacity(9,2,1): I_customer(9)(2)(1)#256 <= 112
 CustomerInventoryCapacity(10,2,1): I_customer(10)(2)(1)#286 <= 15
 CustomerInventoryCapacity(11,2,1): I_customer(11)(2)(1)#316 <= 98
 CustomerInventoryCapacity(12,2,1): I_customer(12)(2)(1)#346 <= 89
 CustomerInventoryCapacity(3,3,1): I_customer(3)(3)(1)#86 <= 8
 CustomerInventoryCapacity(4,3,1): I_customer(4)(3)(1)#116 <= 28
 CustomerInventoryCapacity(5,3,1): I_customer(5)(3)(1)#146 <= 29
 CustomerInventoryCapacity(6,3,1): I_customer(6)(3)(1)#176 <= 7
 CustomerInventoryCapacity(7,3,1): I_customer(7)(3)(1)#206 <= 27
 CustomerInventoryCapacity(8,3,1): I_customer(8)(3)(1)#236 <= 79
 CustomerInventoryCapacity(9,3,1): I_customer(9)(3)(1)#266 <= 108
 CustomerInventoryCapacity(10,3,1): I_customer(10)(3)(1)#296 <= 13
 CustomerInventoryCapacity(11,3,1): I_customer(11)(3)(1)#326 <= 98
 CustomerInventoryCapacity(12,3,1): I_customer(12)(3)(1)#356 <= 85
 CustomerInventoryCapacity(3,1,2): I_customer(3)(1)(2)#67 <= 8
 CustomerInventoryCapacity(4,1,2): I_customer(4)(1)(2)#97 <= 33
 CustomerInventoryCapacity(5,1,2): I_customer(5)(1)(2)#127 <= 33
 CustomerInventoryCapacity(6,1,2): I_customer(6)(1)(2)#157 <= 6
 CustomerInventoryCapacity(7,1,2): I_customer(7)(1)(2)#187 <= 28
 CustomerInventoryCapacity(8,1,2): I_customer(8)(1)(2)#217 <= 80
 CustomerInventoryCapacity(9,1,2): I_customer(9)(1)(2)#247 <= 114
 CustomerInventoryCapacity(10,1,2): I_customer(10)(1)(2)#277 <= 15
 CustomerInventoryCapacity(11,1,2): I_customer(11)(1)(2)#307 <= 92
 CustomerInventoryCapacity(12,1,2): I_customer(12)(1)(2)#337 <= 92
 CustomerInventoryCapacity(3,2,2): I_customer(3)(2)(2)#77 <= 9
 CustomerInventoryCapacity(4,2,2): I_customer(4)(2)(2)#107 <= 29
 CustomerInventoryCapacity(5,2,2): I_customer(5)(2)(2)#137 <= 28
 CustomerInventoryCapacity(6,2,2): I_customer(6)(2)(2)#167 <= 7
 CustomerInventoryCapacity(7,2,2): I_customer(7)(2)(2)#197 <= 26
 CustomerInventoryCapacity(8,2,2): I_customer(8)(2)(2)#227 <= 81
 CustomerInventoryCapacity(9,2,2): I_customer(9)(2)(2)#257 <= 112
 CustomerInventoryCapacity(10,2,2): I_customer(10)(2)(2)#287 <= 11
 CustomerInventoryCapacity(11,2,2): I_customer(11)(2)(2)#317 <= 96
 CustomerInventoryCapacity(12,2,2): I_customer(12)(2)(2)#347 <= 88
 CustomerInventoryCapacity(3,3,2): I_customer(3)(3)(2)#87 <= 11
 CustomerInventoryCapacity(4,3,2): I_customer(4)(3)(2)#117 <= 31
 CustomerInventoryCapacity(5,3,2): I_customer(5)(3)(2)#147 <= 28
 CustomerInventoryCapacity(6,3,2): I_customer(6)(3)(2)#177 <= 6
 CustomerInventoryCapacity(7,3,2): I_customer(7)(3)(2)#207 <= 28
 CustomerInventoryCapacity(8,3,2): I_customer(8)(3)(2)#237 <= 79
 CustomerInventoryCapacity(9,3,2): I_customer(9)(3)(2)#267 <= 114
 CustomerInventoryCapacity(10,3,2): I_customer(10)(3)(2)#297 <= 12
 CustomerInventoryCapacity(11,3,2): I_customer(11)(3)(2)#327 <= 92
 CustomerInventoryCapacity(12,3,2): I_customer(12)(3)(2)#357 <= 85
 CustomerInventoryCapacity(3,1,3): I_customer(3)(1)(3)#68 <= 9
 CustomerInventoryCapacity(4,1,3): I_customer(4)(1)(3)#98 <= 28
 CustomerInventoryCapacity(5,1,3): I_customer(5)(1)(3)#128 <= 30
 CustomerInventoryCapacity(6,1,3): I_customer(6)(1)(3)#158 <= 7
 CustomerInventoryCapacity(7,1,3): I_customer(7)(1)(3)#188 <= 25
 CustomerInventoryCapacity(8,1,3): I_customer(8)(1)(3)#218 <= 78
 CustomerInventoryCapacity(9,1,3): I_customer(9)(1)(3)#248 <= 113
 CustomerInventoryCapacity(10,1,3): I_customer(10)(1)(3)#278 <= 12
 CustomerInventoryCapacity(11,1,3): I_customer(11)(1)(3)#308 <= 96
 CustomerInventoryCapacity(12,1,3): I_customer(12)(1)(3)#338 <= 89
 CustomerInventoryCapacity(3,2,3): I_customer(3)(2)(3)#78 <= 9
 CustomerInventoryCapacity(4,2,3): I_customer(4)(2)(3)#108 <= 28
 CustomerInventoryCapacity(5,2,3): I_customer(5)(2)(3)#138 <= 27
 CustomerInventoryCapacity(6,2,3): I_customer(6)(2)(3)#168 <= 8
 CustomerInventoryCapacity(7,2,3): I_customer(7)(2)(3)#198 <= 26
 CustomerInventoryCapacity(8,2,3): I_customer(8)(2)(3)#228 <= 79
 CustomerInventoryCapacity(9,2,3): I_customer(9)(2)(3)#258 <= 110
 CustomerInventoryCapacity(10,2,3): I_customer(10)(2)(3)#288 <= 15
 CustomerInventoryCapacity(11,2,3): I_customer(11)(2)(3)#318 <= 92
 CustomerInventoryCapacity(12,2,3): I_customer(12)(2)(3)#348 <= 89
 CustomerInventoryCapacity(3,3,3): I_customer(3)(3)(3)#88 <= 9
 CustomerInventoryCapacity(4,3,3): I_customer(4)(3)(3)#118 <= 30
 CustomerInventoryCapacity(5,3,3): I_customer(5)(3)(3)#148 <= 32
 CustomerInventoryCapacity(6,3,3): I_customer(6)(3)(3)#178 <= 7
 CustomerInventoryCapacity(7,3,3): I_customer(7)(3)(3)#208 <= 26
 CustomerInventoryCapacity(8,3,3): I_customer(8)(3)(3)#238 <= 77
 CustomerInventoryCapacity(9,3,3): I_customer(9)(3)(3)#268 <= 106
 CustomerInventoryCapacity(10,3,3): I_customer(10)(3)(3)#298 <= 11
 CustomerInventoryCapacity(11,3,3): I_customer(11)(3)(3)#328 <= 94
 CustomerInventoryCapacity(12,3,3): I_customer(12)(3)(3)#358 <= 89
 CustomerInventoryCapacity(3,1,4): I_customer(3)(1)(4)#69 <= 9
 CustomerInventoryCapacity(4,1,4): I_customer(4)(1)(4)#99 <= 27
 CustomerInventoryCapacity(5,1,4): I_customer(5)(1)(4)#129 <= 28
 CustomerInventoryCapacity(6,1,4): I_customer(6)(1)(4)#159 <= 7
 CustomerInventoryCapacity(7,1,4): I_customer(7)(1)(4)#189 <= 25
 CustomerInventoryCapacity(8,1,4): I_customer(8)(1)(4)#219 <= 82
 CustomerInventoryCapacity(9,1,4): I_customer(9)(1)(4)#249 <= 110
 CustomerInventoryCapacity(10,1,4): I_customer(10)(1)(4)#279 <= 11
 CustomerInventoryCapacity(11,1,4): I_customer(11)(1)(4)#309 <= 96
 CustomerInventoryCapacity(12,1,4): I_customer(12)(1)(4)#339 <= 86
 CustomerInventoryCapacity(3,2,4): I_customer(3)(2)(4)#79 <= 9
 CustomerInventoryCapacity(4,2,4): I_customer(4)(2)(4)#109 <= 30
 CustomerInventoryCapacity(5,2,4): I_customer(5)(2)(4)#139 <= 31
 CustomerInventoryCapacity(6,2,4): I_customer(6)(2)(4)#169 <= 8
 CustomerInventoryCapacity(7,2,4): I_customer(7)(2)(4)#199 <= 28
 CustomerInventoryCapacity(8,2,4): I_customer(8)(2)(4)#229 <= 81
 CustomerInventoryCapacity(9,2,4): I_customer(9)(2)(4)#259 <= 108
 CustomerInventoryCapacity(10,2,4): I_customer(10)(2)(4)#289 <= 10
 CustomerInventoryCapacity(11,2,4): I_customer(11)(2)(4)#319 <= 92
 CustomerInventoryCapacity(12,2,4): I_customer(12)(2)(4)#349 <= 87
 CustomerInventoryCapacity(3,3,4): I_customer(3)(3)(4)#89 <= 9
 CustomerInventoryCapacity(4,3,4): I_customer(4)(3)(4)#119 <= 32
 CustomerInventoryCapacity(5,3,4): I_customer(5)(3)(4)#149 <= 29
 CustomerInventoryCapacity(6,3,4): I_customer(6)(3)(4)#179 <= 8
 CustomerInventoryCapacity(7,3,4): I_customer(7)(3)(4)#209 <= 28
 CustomerInventoryCapacity(8,3,4): I_customer(8)(3)(4)#239 <= 81
 CustomerInventoryCapacity(9,3,4): I_customer(9)(3)(4)#269 <= 109
 CustomerInventoryCapacity(10,3,4): I_customer(10)(3)(4)#299 <= 11
 CustomerInventoryCapacity(11,3,4): I_customer(11)(3)(4)#329 <= 98
 CustomerInventoryCapacity(12,3,4): I_customer(12)(3)(4)#359 <= 91
 CustomerInventoryCapacity(3,1,5): I_customer(3)(1)(5)#70 <= 11
 CustomerInventoryCapacity(4,1,5): I_customer(4)(1)(5)#100 <= 28
 CustomerInventoryCapacity(5,1,5): I_customer(5)(1)(5)#130 <= 28
 CustomerInventoryCapacity(6,1,5): I_customer(6)(1)(5)#160 <= 6
 CustomerInventoryCapacity(7,1,5): I_customer(7)(1)(5)#190 <= 25
 CustomerInventoryCapacity(8,1,5): I_customer(8)(1)(5)#220 <= 77
 CustomerInventoryCapacity(9,1,5): I_customer(9)(1)(5)#250 <= 106
 CustomerInventoryCapacity(10,1,5): I_customer(10)(1)(5)#280 <= 15
 CustomerInventoryCapacity(11,1,5): I_customer(11)(1)(5)#310 <= 92
 CustomerInventoryCapacity(12,1,5): I_customer(12)(1)(5)#340 <= 88
 CustomerInventoryCapacity(3,2,5): I_customer(3)(2)(5)#80 <= 8
 CustomerInventoryCapacity(4,2,5): I_customer(4)(2)(5)#110 <= 28
 CustomerInventoryCapacity(5,2,5): I_customer(5)(2)(5)#140 <= 33
 CustomerInventoryCapacity(6,2,5): I_customer(6)(2)(5)#170 <= 7
 CustomerInventoryCapacity(7,2,5): I_customer(7)(2)(5)#200 <= 27
 CustomerInventoryCapacity(8,2,5): I_customer(8)(2)(5)#230 <= 82
 CustomerInventoryCapacity(9,2,5): I_customer(9)(2)(5)#260 <= 106
 CustomerInventoryCapacity(10,2,5): I_customer(10)(2)(5)#290 <= 13
 CustomerInventoryCapacity(11,2,5): I_customer(11)(2)(5)#320 <= 93
 CustomerInventoryCapacity(12,2,5): I_customer(12)(2)(5)#350 <= 91
 CustomerInventoryCapacity(3,3,5): I_customer(3)(3)(5)#90 <= 11
 CustomerInventoryCapacity(4,3,5): I_customer(4)(3)(5)#120 <= 31
 CustomerInventoryCapacity(5,3,5): I_customer(5)(3)(5)#150 <= 31
 CustomerInventoryCapacity(6,3,5): I_customer(6)(3)(5)#180 <= 6
 CustomerInventoryCapacity(7,3,5): I_customer(7)(3)(5)#210 <= 27
 CustomerInventoryCapacity(8,3,5): I_customer(8)(3)(5)#240 <= 82
 CustomerInventoryCapacity(9,3,5): I_customer(9)(3)(5)#270 <= 108
 CustomerInventoryCapacity(10,3,5): I_customer(10)(3)(5)#300 <= 15
 CustomerInventoryCapacity(11,3,5): I_customer(11)(3)(5)#330 <= 98
 CustomerInventoryCapacity(12,3,5): I_customer(12)(3)(5)#360 <= 91
 CustomerInventoryCapacity(3,1,6): I_customer(3)(1)(6)#71 <= 8
 CustomerInventoryCapacity(4,1,6): I_customer(4)(1)(6)#101 <= 32
 CustomerInventoryCapacity(5,1,6): I_customer(5)(1)(6)#131 <= 31
 CustomerInventoryCapacity(6,1,6): I_customer(6)(1)(6)#161 <= 6
 CustomerInventoryCapacity(7,1,6): I_customer(7)(1)(6)#191 <= 25
 CustomerInventoryCapacity(8,1,6): I_customer(8)(1)(6)#221 <= 81
 CustomerInventoryCapacity(9,1,6): I_customer(9)(1)(6)#251 <= 106
 CustomerInventoryCapacity(10,1,6): I_customer(10)(1)(6)#281 <= 13
 CustomerInventoryCapacity(11,1,6): I_customer(11)(1)(6)#311 <= 97
 CustomerInventoryCapacity(12,1,6): I_customer(12)(1)(6)#341 <= 90
 CustomerInventoryCapacity(3,2,6): I_customer(3)(2)(6)#81 <= 10
 CustomerInventoryCapacity(4,2,6): I_customer(4)(2)(6)#111 <= 32
 CustomerInventoryCapacity(5,2,6): I_customer(5)(2)(6)#141 <= 28
 CustomerInventoryCapacity(6,2,6): I_customer(6)(2)(6)#171 <= 8
 CustomerInventoryCapacity(7,2,6): I_customer(7)(2)(6)#201 <= 28
 CustomerInventoryCapacity(8,2,6): I_customer(8)(2)(6)#231 <= 77
 CustomerInventoryCapacity(9,2,6): I_customer(9)(2)(6)#261 <= 113
 CustomerInventoryCapacity(10,2,6): I_customer(10)(2)(6)#291 <= 12
 CustomerInventoryCapacity(11,2,6): I_customer(11)(2)(6)#321 <= 98
 CustomerInventoryCapacity(12,2,6): I_customer(12)(2)(6)#351 <= 91
 CustomerInventoryCapacity(3,3,6): I_customer(3)(3)(6)#91 <= 11
 CustomerInventoryCapacity(4,3,6): I_customer(4)(3)(6)#121 <= 29
 CustomerInventoryCapacity(5,3,6): I_customer(5)(3)(6)#151 <= 32
 CustomerInventoryCapacity(6,3,6): I_customer(6)(3)(6)#181 <= 8
 CustomerInventoryCapacity(7,3,6): I_customer(7)(3)(6)#211 <= 25
 CustomerInventoryCapacity(8,3,6): I_customer(8)(3)(6)#241 <= 81
 CustomerInventoryCapacity(9,3,6): I_customer(9)(3)(6)#271 <= 106
 CustomerInventoryCapacity(10,3,6): I_customer(10)(3)(6)#301 <= 12
 CustomerInventoryCapacity(11,3,6): I_customer(11)(3)(6)#331 <= 98
 CustomerInventoryCapacity(12,3,6): I_customer(12)(3)(6)#361 <= 84
 CustomerInventoryCapacity(3,1,7): I_customer(3)(1)(7)#72 <= 8
 CustomerInventoryCapacity(4,1,7): I_customer(4)(1)(7)#102 <= 32
 CustomerInventoryCapacity(5,1,7): I_customer(5)(1)(7)#132 <= 30
 CustomerInventoryCapacity(6,1,7): I_customer(6)(1)(7)#162 <= 6
 CustomerInventoryCapacity(7,1,7): I_customer(7)(1)(7)#192 <= 27
 CustomerInventoryCapacity(8,1,7): I_customer(8)(1)(7)#222 <= 78
 CustomerInventoryCapacity(9,1,7): I_customer(9)(1)(7)#252 <= 114
 CustomerInventoryCapacity(10,1,7): I_customer(10)(1)(7)#282 <= 13
 CustomerInventoryCapacity(11,1,7): I_customer(11)(1)(7)#312 <= 94
 CustomerInventoryCapacity(12,1,7): I_customer(12)(1)(7)#342 <= 90
 CustomerInventoryCapacity(3,2,7): I_customer(3)(2)(7)#82 <= 9
 CustomerInventoryCapacity(4,2,7): I_customer(4)(2)(7)#112 <= 28
 CustomerInventoryCapacity(5,2,7): I_customer(5)(2)(7)#142 <= 32
 CustomerInventoryCapacity(6,2,7): I_customer(6)(2)(7)#172 <= 8
 CustomerInventoryCapacity(7,2,7): I_customer(7)(2)(7)#202 <= 26
 CustomerInventoryCapacity(8,2,7): I_customer(8)(2)(7)#232 <= 78
 CustomerInventoryCapacity(9,2,7): I_customer(9)(2)(7)#262 <= 111
 CustomerInventoryCapacity(10,2,7): I_customer(10)(2)(7)#292 <= 12
 CustomerInventoryCapacity(11,2,7): I_customer(11)(2)(7)#322 <= 94
 CustomerInventoryCapacity(12,2,7): I_customer(12)(2)(7)#352 <= 87
 CustomerInventoryCapacity(3,3,7): I_customer(3)(3)(7)#92 <= 10
 CustomerInventoryCapacity(4,3,7): I_customer(4)(3)(7)#122 <= 30
 CustomerInventoryCapacity(5,3,7): I_customer(5)(3)(7)#152 <= 33
 CustomerInventoryCapacity(6,3,7): I_customer(6)(3)(7)#182 <= 7
 CustomerInventoryCapacity(7,3,7): I_customer(7)(3)(7)#212 <= 25
 CustomerInventoryCapacity(8,3,7): I_customer(8)(3)(7)#242 <= 82
 CustomerInventoryCapacity(9,3,7): I_customer(9)(3)(7)#272 <= 106
 CustomerInventoryCapacity(10,3,7): I_customer(10)(3)(7)#302 <= 11
 CustomerInventoryCapacity(11,3,7): I_customer(11)(3)(7)#332 <= 93
 CustomerInventoryCapacity(12,3,7): I_customer(12)(3)(7)#362 <= 85
 CustomerInventoryCapacity(3,1,8): I_customer(3)(1)(8)#73 <= 10
 CustomerInventoryCapacity(4,1,8): I_customer(4)(1)(8)#103 <= 32
 CustomerInventoryCapacity(5,1,8): I_customer(5)(1)(8)#133 <= 32
 CustomerInventoryCapacity(6,1,8): I_customer(6)(1)(8)#163 <= 7
 CustomerInventoryCapacity(7,1,8): I_customer(7)(1)(8)#193 <= 24
 CustomerInventoryCapacity(8,1,8): I_customer(8)(1)(8)#223 <= 83
 CustomerInventoryCapacity(9,1,8): I_customer(9)(1)(8)#253 <= 111
 CustomerInventoryCapacity(10,1,8): I_customer(10)(1)(8)#283 <= 16
 CustomerInventoryCapacity(11,1,8): I_customer(11)(1)(8)#313 <= 92
 CustomerInventoryCapacity(12,1,8): I_customer(12)(1)(8)#343 <= 91
 CustomerInventoryCapacity(3,2,8): I_customer(3)(2)(8)#83 <= 10
 CustomerInventoryCapacity(4,2,8): I_customer(4)(2)(8)#113 <= 32
 CustomerInventoryCapacity(5,2,8): I_customer(5)(2)(8)#143 <= 32
 CustomerInventoryCapacity(6,2,8): I_customer(6)(2)(8)#173 <= 6
 CustomerInventoryCapacity(7,2,8): I_customer(7)(2)(8)#203 <= 26
 CustomerInventoryCapacity(8,2,8): I_customer(8)(2)(8)#233 <= 79
 CustomerInventoryCapacity(9,2,8): I_customer(9)(2)(8)#263 <= 112
 CustomerInventoryCapacity(10,2,8): I_customer(10)(2)(8)#293 <= 15
 CustomerInventoryCapacity(11,2,8): I_customer(11)(2)(8)#323 <= 97
 CustomerInventoryCapacity(12,2,8): I_customer(12)(2)(8)#353 <= 84
 CustomerInventoryCapacity(3,3,8): I_customer(3)(3)(8)#93 <= 12
 CustomerInventoryCapacity(4,3,8): I_customer(4)(3)(8)#123 <= 29
 CustomerInventoryCapacity(5,3,8): I_customer(5)(3)(8)#153 <= 28
 CustomerInventoryCapacity(6,3,8): I_customer(6)(3)(8)#183 <= 7
 CustomerInventoryCapacity(7,3,8): I_customer(7)(3)(8)#213 <= 26
 CustomerInventoryCapacity(8,3,8): I_customer(8)(3)(8)#243 <= 77
 CustomerInventoryCapacity(9,3,8): I_customer(9)(3)(8)#273 <= 107
 CustomerInventoryCapacity(10,3,8): I_customer(10)(3)(8)#303 <= 12
 CustomerInventoryCapacity(11,3,8): I_customer(11)(3)(8)#333 <= 95
 CustomerInventoryCapacity(12,3,8): I_customer(12)(3)(8)#363 <= 88
 CustomerInventoryCapacity(3,1,9): I_customer(3)(1)(9)#74 <= 8
 CustomerInventoryCapacity(4,1,9): I_customer(4)(1)(9)#104 <= 27
 CustomerInventoryCapacity(5,1,9): I_customer(5)(1)(9)#134 <= 32
 CustomerInventoryCapacity(6,1,9): I_customer(6)(1)(9)#164 <= 7
 CustomerInventoryCapacity(7,1,9): I_customer(7)(1)(9)#194 <= 25
 CustomerInventoryCapacity(8,1,9): I_customer(8)(1)(9)#224 <= 79
 CustomerInventoryCapacity(9,1,9): I_customer(9)(1)(9)#254 <= 108
 CustomerInventoryCapacity(10,1,9): I_customer(10)(1)(9)#284 <= 15
 CustomerInventoryCapacity(11,1,9): I_customer(11)(1)(9)#314 <= 93
 CustomerInventoryCapacity(12,1,9): I_customer(12)(1)(9)#344 <= 92
 CustomerInventoryCapacity(3,2,9): I_customer(3)(2)(9)#84 <= 10
 CustomerInventoryCapacity(4,2,9): I_customer(4)(2)(9)#114 <= 30
 CustomerInventoryCapacity(5,2,9): I_customer(5)(2)(9)#144 <= 28
 CustomerInventoryCapacity(6,2,9): I_customer(6)(2)(9)#174 <= 7
 CustomerInventoryCapacity(7,2,9): I_customer(7)(2)(9)#204 <= 26
 CustomerInventoryCapacity(8,2,9): I_customer(8)(2)(9)#234 <= 81
 CustomerInventoryCapacity(9,2,9): I_customer(9)(2)(9)#264 <= 107
 CustomerInventoryCapacity(10,2,9): I_customer(10)(2)(9)#294 <= 15
 CustomerInventoryCapacity(11,2,9): I_customer(11)(2)(9)#324 <= 99
 CustomerInventoryCapacity(12,2,9): I_customer(12)(2)(9)#354 <= 85
 CustomerInventoryCapacity(3,3,9): I_customer(3)(3)(9)#94 <= 12
 CustomerInventoryCapacity(4,3,9): I_customer(4)(3)(9)#124 <= 27
 CustomerInventoryCapacity(5,3,9): I_customer(5)(3)(9)#154 <= 32
 CustomerInventoryCapacity(6,3,9): I_customer(6)(3)(9)#184 <= 8
 CustomerInventoryCapacity(7,3,9): I_customer(7)(3)(9)#214 <= 26
 CustomerInventoryCapacity(8,3,9): I_customer(8)(3)(9)#244 <= 83
 CustomerInventoryCapacity(9,3,9): I_customer(9)(3)(9)#274 <= 108
 CustomerInventoryCapacity(10,3,9): I_customer(10)(3)(9)#304 <= 11
 CustomerInventoryCapacity(11,3,9): I_customer(11)(3)(9)#334 <= 95
 CustomerInventoryCapacity(12,3,9): I_customer(12)(3)(9)#364 <= 89
 CustomerInventoryCapacity(3,1,10): I_customer(3)(1)(10)#75 <= 9
 CustomerInventoryCapacity(4,1,10): I_customer(4)(1)(10)#105 <= 31
 CustomerInventoryCapacity(5,1,10): I_customer(5)(1)(10)#135 <= 30
 CustomerInventoryCapacity(6,1,10): I_customer(6)(1)(10)#165 <= 7
 CustomerInventoryCapacity(7,1,10): I_customer(7)(1)(10)#195 <= 24
 CustomerInventoryCapacity(8,1,10): I_customer(8)(1)(10)#225 <= 80
 CustomerInventoryCapacity(9,1,10): I_customer(9)(1)(10)#255 <= 112
 CustomerInventoryCapacity(10,1,10): I_customer(10)(1)(10)#285 <= 15
 CustomerInventoryCapacity(11,1,10): I_customer(11)(1)(10)#315 <= 95
 CustomerInventoryCapacity(12,1,10): I_customer(12)(1)(10)#345 <= 92
 CustomerInventoryCapacity(3,2,10): I_customer(3)(2)(10)#85 <= 10
 CustomerInventoryCapacity(4,2,10): I_customer(4)(2)(10)#115 <= 31
 CustomerInventoryCapacity(5,2,10): I_customer(5)(2)(10)#145 <= 32
 CustomerInventoryCapacity(6,2,10): I_customer(6)(2)(10)#175 <= 6
 CustomerInventoryCapacity(7,2,10): I_customer(7)(2)(10)#205 <= 26
 CustomerInventoryCapacity(8,2,10): I_customer(8)(2)(10)#235 <= 81
 CustomerInventoryCapacity(9,2,10): I_customer(9)(2)(10)#265 <= 107
 CustomerInventoryCapacity(10,2,10): I_customer(10)(2)(10)#295 <= 13
 CustomerInventoryCapacity(11,2,10): I_customer(11)(2)(10)#325 <= 96
 CustomerInventoryCapacity(12,2,10): I_customer(12)(2)(10)#355 <= 92
 CustomerInventoryCapacity(3,3,10): I_customer(3)(3)(10)#95 <= 9
 CustomerInventoryCapacity(4,3,10): I_customer(4)(3)(10)#125 <= 31
 CustomerInventoryCapacity(5,3,10): I_customer(5)(3)(10)#155 <= 29
 CustomerInventoryCapacity(6,3,10): I_customer(6)(3)(10)#185 <= 6
 CustomerInventoryCapacity(7,3,10): I_customer(7)(3)(10)#215 <= 27
 CustomerInventoryCapacity(8,3,10): I_customer(8)(3)(10)#245 <= 81
 CustomerInventoryCapacity(9,3,10): I_customer(9)(3)(10)#275 <= 107
 CustomerInventoryCapacity(10,3,10): I_customer(10)(3)(10)#305 <= 13
 CustomerInventoryCapacity(11,3,10): I_customer(11)(3)(10)#335 <= 97
 CustomerInventoryCapacity(12,3,10): I_customer(12)(3)(10)#365 <= 85
 WarehouseDelivery(1,1,1):       q(1)(1)(1)#666 <= 136.8
 WarehouseDelivery(2,1,1):       q(2)(1)(1)#672  = 0
 WarehouseDelivery(1,2,1):       q(1)(2)(1)#669  = 0
 WarehouseDelivery(2,2,1):       q(2)(2)(1)#675 <= 136.8
 WarehouseDelivery(1,1,2):       q(1)(1)(2)#667  = 0
 WarehouseDelivery(2,1,2):       q(2)(1)(2)#673  = 0
 WarehouseDelivery(1,2,2):       q(1)(2)(2)#670  = 0
 WarehouseDelivery(2,2,2):       q(2)(2)(2)#676  = 0
 WarehouseDelivery(1,1,3):       q(1)(1)(3)#668 <= 136.8
 WarehouseDelivery(2,1,3):       q(2)(1)(3)#674  = 0
 WarehouseDelivery(1,2,3):       q(1)(2)(3)#671  = 0
 WarehouseDelivery(2,2,3):       q(2)(2)(3)#677  = 0
 VehicleCapacityPlant(1,1):      q(1)(1)(1)#666 + q(2)(1)(1)#672 <= 102
 VehicleCapacityPlant(2,1):      q(1)(2)(1)#669 + q(2)(2)(1)#675 <= 102
 VehicleCapacityPlant(1,2):      q(1)(1)(2)#667 + q(2)(1)(2)#673 <= 102
 VehicleCapacityPlant(2,2):      q(1)(2)(2)#670 + q(2)(2)(2)#676 <= 102
 VehicleCapacityPlant(1,3):      q(1)(1)(3)#668 + q(2)(1)(3)#674 <= 102
 VehicleCapacityPlant(2,3):      q(1)(2)(3)#671 + q(2)(2)(3)#677 <= 102
 CustomerVisit(3,1,1,1):         w(3)(1)(1)(1)#678  = 0
 CustomerVisit(4,1,1,1):         w(4)(1)(1)(1)#858  = 0
 CustomerVisit(5,1,1,1):         w(5)(1)(1)(1)#1038  = 0
 CustomerVisit(6,1,1,1):         w(6)(1)(1)(1)#1218  = 0
 CustomerVisit(7,1,1,1):         w(7)(1)(1)(1)#1398  = 0
 CustomerVisit(8,1,1,1):         w(8)(1)(1)(1)#1578  = 0
 CustomerVisit(9,1,1,1):         w(9)(1)(1)(1)#1758  = 0
 CustomerVisit(10,1,1,1):        w(10)(1)(1)(1)#1938  = 0
 CustomerVisit(11,1,1,1):        w(11)(1)(1)(1)#2118  = 0
 CustomerVisit(12,1,1,1):        w(12)(1)(1)(1)#2298  = 0
 CustomerVisit(3,2,1,1):         w(3)(2)(1)(1)#708  = 0
 CustomerVisit(4,2,1,1):         w(4)(2)(1)(1)#888  = 0
 CustomerVisit(5,2,1,1):         w(5)(2)(1)(1)#1068  = 0
 CustomerVisit(6,2,1,1):         w(6)(2)(1)(1)#1248  = 0
 CustomerVisit(7,2,1,1):         w(7)(2)(1)(1)#1428  = 0
 CustomerVisit(8,2,1,1):         w(8)(2)(1)(1)#1608  = 0
 CustomerVisit(9,2,1,1):         w(9)(2)(1)(1)#1788  = 0
 CustomerVisit(10,2,1,1):        w(10)(2)(1)(1)#1968  = 0
 CustomerVisit(11,2,1,1):        w(11)(2)(1)(1)#2148  = 0
 CustomerVisit(12,2,1,1):        w(12)(2)(1)(1)#2328  = 0
 CustomerVisit(3,3,1,1):         w(3)(3)(1)(1)#738  = 0
 CustomerVisit(4,3,1,1):         w(4)(3)(1)(1)#918  = 0
 CustomerVisit(5,3,1,1):         w(5)(3)(1)(1)#1098  = 0
 CustomerVisit(7,3,1,1):         w(7)(3)(1)(1)#1458  = 0
 CustomerVisit(8,3,1,1):         w(8)(3)(1)(1)#1638  = 0
 CustomerVisit(9,3,1,1):         w(9)(3)(1)(1)#1818  = 0
 CustomerVisit(11,3,1,1):        w(11)(3)(1)(1)#2178  = 0
 CustomerVisit(3,1,2,1):         w(3)(1)(2)(1)#688  = 0
 CustomerVisit(4,1,2,1):         w(4)(1)(2)(1)#868  = 0
 CustomerVisit(5,1,2,1):         w(5)(1)(2)(1)#1048  = 0
 CustomerVisit(6,1,2,1):         w(6)(1)(2)(1)#1228  = 0
 CustomerVisit(7,1,2,1):         w(7)(1)(2)(1)#1408  = 0
 CustomerVisit(8,1,2,1):         w(8)(1)(2)(1)#1588  = 0
 CustomerVisit(9,1,2,1):         w(9)(1)(2)(1)#1768  = 0
 CustomerVisit(10,1,2,1):        w(10)(1)(2)(1)#1948  = 0
 CustomerVisit(11,1,2,1):        w(11)(1)(2)(1)#2128  = 0
 CustomerVisit(12,1,2,1):        w(12)(1)(2)(1)#2308  = 0
 CustomerVisit(4,2,2,1):         w(4)(2)(2)(1)#898  = 0
 CustomerVisit(5,2,2,1):         w(5)(2)(2)(1)#1078  = 0
 CustomerVisit(7,2,2,1):         w(7)(2)(2)(1)#1438  = 0
 CustomerVisit(8,2,2,1):         w(8)(2)(2)(1)#1618  = 0
 CustomerVisit(11,2,2,1):        w(11)(2)(2)(1)#2158  = 0
 CustomerVisit(3,3,2,1):         w(3)(3)(2)(1)#748  = 0
 CustomerVisit(4,3,2,1):         w(4)(3)(2)(1)#928  = 0
 CustomerVisit(6,3,2,1):         w(6)(3)(2)(1)#1288  = 0
 CustomerVisit(7,3,2,1):         w(7)(3)(2)(1)#1468  = 0
 CustomerVisit(8,3,2,1):         w(8)(3)(2)(1)#1648  = 0
 CustomerVisit(9,3,2,1):         w(9)(3)(2)(1)#1828  = 0
 CustomerVisit(10,3,2,1):        w(10)(3)(2)(1)#2008  = 0
 CustomerVisit(11,3,2,1):        w(11)(3)(2)(1)#2188  = 0
 CustomerVisit(12,3,2,1):        w(12)(3)(2)(1)#2368  = 0
 CustomerVisit(5,1,3,1):         w(5)(1)(3)(1)#1058  = 0
 CustomerVisit(7,1,3,1):         w(7)(1)(3)(1)#1418  = 0
 CustomerVisit(9,1,3,1):         w(9)(1)(3)(1)#1778  = 0
 CustomerVisit(11,1,3,1):        w(11)(1)(3)(1)#2138  = 0
 CustomerVisit(12,1,3,1):        w(12)(1)(3)(1)#2318  = 0
 CustomerVisit(3,2,3,1):         w(3)(2)(3)(1)#728  = 0
 CustomerVisit(4,2,3,1):         w(4)(2)(3)(1)#908  = 0
 CustomerVisit(5,2,3,1):         w(5)(2)(3)(1)#1088  = 0
 CustomerVisit(6,2,3,1):         w(6)(2)(3)(1)#1268  = 0
 CustomerVisit(7,2,3,1):         w(7)(2)(3)(1)#1448  = 0
 CustomerVisit(8,2,3,1):         w(8)(2)(3)(1)#1628  = 0
 CustomerVisit(9,2,3,1):         w(9)(2)(3)(1)#1808  = 0
 CustomerVisit(10,2,3,1):        w(10)(2)(3)(1)#1988  = 0
 CustomerVisit(11,2,3,1):        w(11)(2)(3)(1)#2168  = 0
 CustomerVisit(12,2,3,1):        w(12)(2)(3)(1)#2348  = 0
 CustomerVisit(3,3,3,1):         w(3)(3)(3)(1)#758  = 0
 CustomerVisit(4,3,3,1):         w(4)(3)(3)(1)#938  = 0
 CustomerVisit(6,3,3,1):         w(6)(3)(3)(1)#1298  = 0
 CustomerVisit(7,3,3,1):         w(7)(3)(3)(1)#1478  = 0
 CustomerVisit(8,3,3,1):         w(8)(3)(3)(1)#1658  = 0
 CustomerVisit(9,3,3,1):         w(9)(3)(3)(1)#1838  = 0
 CustomerVisit(10,3,3,1):        w(10)(3)(3)(1)#2018  = 0
 CustomerVisit(11,3,3,1):        w(11)(3)(3)(1)#2198  = 0
 CustomerVisit(12,3,3,1):        w(12)(3)(3)(1)#2378  = 0
 CustomerVisit(5,4,1,1):         w(5)(4)(1)(1)#1128  = 0
 CustomerVisit(6,4,1,1):         w(6)(4)(1)(1)#1308  = 0
 CustomerVisit(8,4,1,1):         w(8)(4)(1)(1)#1668  = 0
 CustomerVisit(9,4,1,1):         w(9)(4)(1)(1)#1848  = 0
 CustomerVisit(10,4,1,1):        w(10)(4)(1)(1)#2028  = 0
 CustomerVisit(11,4,1,1):        w(11)(4)(1)(1)#2208  = 0
 CustomerVisit(12,4,1,1):        w(12)(4)(1)(1)#2388  = 0
 CustomerVisit(3,5,1,1):         w(3)(5)(1)(1)#798  = 0
 CustomerVisit(4,5,1,1):         w(4)(5)(1)(1)#978  = 0
 CustomerVisit(5,5,1,1):         w(5)(5)(1)(1)#1158  = 0
 CustomerVisit(6,5,1,1):         w(6)(5)(1)(1)#1338  = 0
 CustomerVisit(7,5,1,1):         w(7)(5)(1)(1)#1518  = 0
 CustomerVisit(8,5,1,1):         w(8)(5)(1)(1)#1698  = 0
 CustomerVisit(9,5,1,1):         w(9)(5)(1)(1)#1878  = 0
 CustomerVisit(10,5,1,1):        w(10)(5)(1)(1)#2058  = 0
 CustomerVisit(11,5,1,1):        w(11)(5)(1)(1)#2238  = 0
 CustomerVisit(12,5,1,1):        w(12)(5)(1)(1)#2418  = 0
 CustomerVisit(3,6,1,1):         w(3)(6)(1)(1)#828  = 0
 CustomerVisit(4,6,1,1):         w(4)(6)(1)(1)#1008  = 0
 CustomerVisit(5,6,1,1):         w(5)(6)(1)(1)#1188  = 0
 CustomerVisit(6,6,1,1):         w(6)(6)(1)(1)#1368  = 0
 CustomerVisit(7,6,1,1):         w(7)(6)(1)(1)#1548  = 0
 CustomerVisit(8,6,1,1):         w(8)(6)(1)(1)#1728  = 0
 CustomerVisit(9,6,1,1):         w(9)(6)(1)(1)#1908  = 0
 CustomerVisit(10,6,1,1):        w(10)(6)(1)(1)#2088  = 0
 CustomerVisit(11,6,1,1):        w(11)(6)(1)(1)#2268  = 0
 CustomerVisit(12,6,1,1):        w(12)(6)(1)(1)#2448  = 0
 CustomerVisit(3,4,2,1):         w(3)(4)(2)(1)#778  = 0
 CustomerVisit(4,4,2,1):         w(4)(4)(2)(1)#958  = 0
 CustomerVisit(5,4,2,1):         w(5)(4)(2)(1)#1138  = 0
 CustomerVisit(6,4,2,1):         w(6)(4)(2)(1)#1318  = 0
 CustomerVisit(7,4,2,1):         w(7)(4)(2)(1)#1498  = 0
 CustomerVisit(8,4,2,1):         w(8)(4)(2)(1)#1678  = 0
 CustomerVisit(9,4,2,1):         w(9)(4)(2)(1)#1858  = 0
 CustomerVisit(10,4,2,1):        w(10)(4)(2)(1)#2038  = 0
 CustomerVisit(11,4,2,1):        w(11)(4)(2)(1)#2218  = 0
 CustomerVisit(12,4,2,1):        w(12)(4)(2)(1)#2398  = 0
 CustomerVisit(3,5,2,1):         w(3)(5)(2)(1)#808  = 0
 CustomerVisit(4,5,2,1):         w(4)(5)(2)(1)#988  = 0
 CustomerVisit(5,5,2,1):         w(5)(5)(2)(1)#1168  = 0
 CustomerVisit(6,5,2,1):         w(6)(5)(2)(1)#1348  = 0
 CustomerVisit(7,5,2,1):         w(7)(5)(2)(1)#1528  = 0
 CustomerVisit(8,5,2,1):         w(8)(5)(2)(1)#1708  = 0
 CustomerVisit(9,5,2,1):         w(9)(5)(2)(1)#1888  = 0
 CustomerVisit(10,5,2,1):        w(10)(5)(2)(1)#2068  = 0
 CustomerVisit(11,5,2,1):        w(11)(5)(2)(1)#2248  = 0
 CustomerVisit(12,5,2,1):        w(12)(5)(2)(1)#2428  = 0
 CustomerVisit(3,6,2,1):         w(3)(6)(2)(1)#838  = 0
 CustomerVisit(4,6,2,1):         w(4)(6)(2)(1)#1018  = 0
 CustomerVisit(5,6,2,1):         w(5)(6)(2)(1)#1198  = 0
 CustomerVisit(6,6,2,1):         w(6)(6)(2)(1)#1378  = 0
 CustomerVisit(8,6,2,1):         w(8)(6)(2)(1)#1738  = 0
 CustomerVisit(9,6,2,1):         w(9)(6)(2)(1)#1918  = 0
 CustomerVisit(10,6,2,1):        w(10)(6)(2)(1)#2098  = 0
 CustomerVisit(11,6,2,1):        w(11)(6)(2)(1)#2278  = 0
 CustomerVisit(12,6,2,1):        w(12)(6)(2)(1)#2458  = 0
 CustomerVisit(3,4,3,1):         w(3)(4)(3)(1)#788  = 0
 CustomerVisit(4,4,3,1):         w(4)(4)(3)(1)#968  = 0
 CustomerVisit(5,4,3,1):         w(5)(4)(3)(1)#1148  = 0
 CustomerVisit(6,4,3,1):         w(6)(4)(3)(1)#1328  = 0
 CustomerVisit(7,4,3,1):         w(7)(4)(3)(1)#1508  = 0
 CustomerVisit(8,4,3,1):         w(8)(4)(3)(1)#1688  = 0
 CustomerVisit(9,4,3,1):         w(9)(4)(3)(1)#1868  = 0
 CustomerVisit(10,4,3,1):        w(10)(4)(3)(1)#2048  = 0
 CustomerVisit(11,4,3,1):        w(11)(4)(3)(1)#2228  = 0
 CustomerVisit(12,4,3,1):        w(12)(4)(3)(1)#2408  = 0
 CustomerVisit(3,5,3,1):         w(3)(5)(3)(1)#818  = 0
 CustomerVisit(4,5,3,1):         w(4)(5)(3)(1)#998  = 0
 CustomerVisit(5,5,3,1):         w(5)(5)(3)(1)#1178  = 0
 CustomerVisit(6,5,3,1):         w(6)(5)(3)(1)#1358  = 0
 CustomerVisit(7,5,3,1):         w(7)(5)(3)(1)#1538  = 0
 CustomerVisit(8,5,3,1):         w(8)(5)(3)(1)#1718  = 0
 CustomerVisit(9,5,3,1):         w(9)(5)(3)(1)#1898  = 0
 CustomerVisit(10,5,3,1):        w(10)(5)(3)(1)#2078  = 0
 CustomerVisit(11,5,3,1):        w(11)(5)(3)(1)#2258  = 0
 CustomerVisit(12,5,3,1):        w(12)(5)(3)(1)#2438  = 0
 CustomerVisit(3,6,3,1):         w(3)(6)(3)(1)#848  = 0
 CustomerVisit(4,6,3,1):         w(4)(6)(3)(1)#1028  = 0
 CustomerVisit(5,6,3,1):         w(5)(6)(3)(1)#1208  = 0
 CustomerVisit(6,6,3,1):         w(6)(6)(3)(1)#1388  = 0
 CustomerVisit(7,6,3,1):         w(7)(6)(3)(1)#1568  = 0
 CustomerVisit(8,6,3,1):         w(8)(6)(3)(1)#1748  = 0
 CustomerVisit(9,6,3,1):         w(9)(6)(3)(1)#1928  = 0
 CustomerVisit(10,6,3,1):        w(10)(6)(3)(1)#2108  = 0
 CustomerVisit(11,6,3,1):        w(11)(6)(3)(1)#2288  = 0
 CustomerVisit(12,6,3,1):        w(12)(6)(3)(1)#2468  = 0
 CustomerVisit(3,1,1,2):         w(3)(1)(1)(2)#679  = 0
 CustomerVisit(4,1,1,2):         w(4)(1)(1)(2)#859  = 0
 CustomerVisit(5,1,1,2):         w(5)(1)(1)(2)#1039  = 0
 CustomerVisit(6,1,1,2):         w(6)(1)(1)(2)#1219  = 0
 CustomerVisit(7,1,1,2):         w(7)(1)(1)(2)#1399  = 0
 CustomerVisit(8,1,1,2):         w(8)(1)(1)(2)#1579  = 0
 CustomerVisit(9,1,1,2):         w(9)(1)(1)(2)#1759  = 0
 CustomerVisit(10,1,1,2):        w(10)(1)(1)(2)#1939  = 0
 CustomerVisit(11,1,1,2):        w(11)(1)(1)(2)#2119  = 0
 CustomerVisit(12,1,1,2):        w(12)(1)(1)(2)#2299  = 0
 CustomerVisit(3,2,1,2):         w(3)(2)(1)(2)#709  = 0
 CustomerVisit(4,2,1,2):         w(4)(2)(1)(2)#889  = 0
 CustomerVisit(5,2,1,2):         w(5)(2)(1)(2)#1069  = 0
 CustomerVisit(6,2,1,2):         w(6)(2)(1)(2)#1249  = 0
 CustomerVisit(7,2,1,2):         w(7)(2)(1)(2)#1429  = 0
 CustomerVisit(8,2,1,2):         w(8)(2)(1)(2)#1609  = 0
 CustomerVisit(9,2,1,2):         w(9)(2)(1)(2)#1789  = 0
 CustomerVisit(10,2,1,2):        w(10)(2)(1)(2)#1969  = 0
 CustomerVisit(11,2,1,2):        w(11)(2)(1)(2)#2149  = 0
 CustomerVisit(12,2,1,2):        w(12)(2)(1)(2)#2329  = 0
 CustomerVisit(3,3,1,2):         w(3)(3)(1)(2)#739  = 0
 CustomerVisit(4,3,1,2):         w(4)(3)(1)(2)#919  = 0
 CustomerVisit(5,3,1,2):         w(5)(3)(1)(2)#1099  = 0
 CustomerVisit(7,3,1,2):         w(7)(3)(1)(2)#1459  = 0
 CustomerVisit(8,3,1,2):         w(8)(3)(1)(2)#1639  = 0
 CustomerVisit(9,3,1,2):         w(9)(3)(1)(2)#1819  = 0
 CustomerVisit(11,3,1,2):        w(11)(3)(1)(2)#2179  = 0
 CustomerVisit(12,3,1,2):        w(12)(3)(1)(2)#2359  = 0
 CustomerVisit(3,1,2,2):         w(3)(1)(2)(2)#689  = 0
 CustomerVisit(4,1,2,2):         w(4)(1)(2)(2)#869  = 0
 CustomerVisit(5,1,2,2):         w(5)(1)(2)(2)#1049  = 0
 CustomerVisit(7,1,2,2):         w(7)(1)(2)(2)#1409  = 0
 CustomerVisit(8,1,2,2):         w(8)(1)(2)(2)#1589  = 0
 CustomerVisit(9,1,2,2):         w(9)(1)(2)(2)#1769  = 0
 CustomerVisit(11,1,2,2):        w(11)(1)(2)(2)#2129  = 0
 CustomerVisit(3,2,2,2):         w(3)(2)(2)(2)#719  = 0
 CustomerVisit(4,2,2,2):         w(4)(2)(2)(2)#899  = 0
 CustomerVisit(6,2,2,2):         w(6)(2)(2)(2)#1259  = 0
 CustomerVisit(7,2,2,2):         w(7)(2)(2)(2)#1439  = 0
 CustomerVisit(8,2,2,2):         w(8)(2)(2)(2)#1619  = 0
 CustomerVisit(9,2,2,2):         w(9)(2)(2)(2)#1799  = 0
 CustomerVisit(10,2,2,2):        w(10)(2)(2)(2)#1979  = 0
 CustomerVisit(11,2,2,2):        w(11)(2)(2)(2)#2159  = 0
 CustomerVisit(12,2,2,2):        w(12)(2)(2)(2)#2339  = 0
 CustomerVisit(3,3,2,2):         w(3)(3)(2)(2)#749  = 0
 CustomerVisit(4,3,2,2):         w(4)(3)(2)(2)#929  = 0
 CustomerVisit(5,3,2,2):         w(5)(3)(2)(2)#1109  = 0
 CustomerVisit(6,3,2,2):         w(6)(3)(2)(2)#1289  = 0
 CustomerVisit(7,3,2,2):         w(7)(3)(2)(2)#1469  = 0
 CustomerVisit(8,3,2,2):         w(8)(3)(2)(2)#1649  = 0
 CustomerVisit(9,3,2,2):         w(9)(3)(2)(2)#1829  = 0
 CustomerVisit(10,3,2,2):        w(10)(3)(2)(2)#2009  = 0
 CustomerVisit(11,3,2,2):        w(11)(3)(2)(2)#2189  = 0
 CustomerVisit(12,3,2,2):        w(12)(3)(2)(2)#2369  = 0
 CustomerVisit(3,1,3,2):         w(3)(1)(3)(2)#699  = 0
 CustomerVisit(4,1,3,2):         w(4)(1)(3)(2)#879  = 0
 CustomerVisit(5,1,3,2):         w(5)(1)(3)(2)#1059  = 0
 CustomerVisit(6,1,3,2):         w(6)(1)(3)(2)#1239  = 0
 CustomerVisit(7,1,3,2):         w(7)(1)(3)(2)#1419  = 0
 CustomerVisit(8,1,3,2):         w(8)(1)(3)(2)#1599  = 0
 CustomerVisit(9,1,3,2):         w(9)(1)(3)(2)#1779  = 0
 CustomerVisit(10,1,3,2):        w(10)(1)(3)(2)#1959  = 0
 CustomerVisit(11,1,3,2):        w(11)(1)(3)(2)#2139  = 0
 CustomerVisit(12,1,3,2):        w(12)(1)(3)(2)#2319  = 0
 CustomerVisit(4,2,3,2):         w(4)(2)(3)(2)#909  = 0
 CustomerVisit(5,2,3,2):         w(5)(2)(3)(2)#1089  = 0
 CustomerVisit(7,2,3,2):         w(7)(2)(3)(2)#1449  = 0
 CustomerVisit(3,3,3,2):         w(3)(3)(3)(2)#759  = 0
 CustomerVisit(4,3,3,2):         w(4)(3)(3)(2)#939  = 0
 CustomerVisit(6,3,3,2):         w(6)(3)(3)(2)#1299  = 0
 CustomerVisit(7,3,3,2):         w(7)(3)(3)(2)#1479  = 0
 CustomerVisit(8,3,3,2):         w(8)(3)(3)(2)#1659  = 0
 CustomerVisit(9,3,3,2):         w(9)(3)(3)(2)#1839  = 0
 CustomerVisit(10,3,3,2):        w(10)(3)(3)(2)#2019  = 0
 CustomerVisit(11,3,3,2):        w(11)(3)(3)(2)#2199  = 0
 CustomerVisit(12,3,3,2):        w(12)(3)(3)(2)#2379  = 0
 CustomerVisit(3,4,1,2):         w(3)(4)(1)(2)#769  = 0
 CustomerVisit(4,4,1,2):         w(4)(4)(1)(2)#949  = 0
 CustomerVisit(5,4,1,2):         w(5)(4)(1)(2)#1129  = 0
 CustomerVisit(6,4,1,2):         w(6)(4)(1)(2)#1309  = 0
 CustomerVisit(7,4,1,2):         w(7)(4)(1)(2)#1489  = 0
 CustomerVisit(8,4,1,2):         w(8)(4)(1)(2)#1669  = 0
 CustomerVisit(9,4,1,2):         w(9)(4)(1)(2)#1849  = 0
 CustomerVisit(10,4,1,2):        w(10)(4)(1)(2)#2029  = 0
 CustomerVisit(11,4,1,2):        w(11)(4)(1)(2)#2209  = 0
 CustomerVisit(12,4,1,2):        w(12)(4)(1)(2)#2389  = 0
 CustomerVisit(3,5,1,2):         w(3)(5)(1)(2)#799  = 0
 CustomerVisit(4,5,1,2):         w(4)(5)(1)(2)#979  = 0
 CustomerVisit(5,5,1,2):         w(5)(5)(1)(2)#1159  = 0
 CustomerVisit(6,5,1,2):         w(6)(5)(1)(2)#1339  = 0
 CustomerVisit(7,5,1,2):         w(7)(5)(1)(2)#1519  = 0
 CustomerVisit(8,5,1,2):         w(8)(5)(1)(2)#1699  = 0
 CustomerVisit(9,5,1,2):         w(9)(5)(1)(2)#1879  = 0
 CustomerVisit(10,5,1,2):        w(10)(5)(1)(2)#2059  = 0
 CustomerVisit(11,5,1,2):        w(11)(5)(1)(2)#2239  = 0
 CustomerVisit(12,5,1,2):        w(12)(5)(1)(2)#2419  = 0
 CustomerVisit(5,6,1,2):         w(5)(6)(1)(2)#1189  = 0
 CustomerVisit(6,6,1,2):         w(6)(6)(1)(2)#1369  = 0
 CustomerVisit(8,6,1,2):         w(8)(6)(1)(2)#1729  = 0
 CustomerVisit(9,6,1,2):         w(9)(6)(1)(2)#1909  = 0
 CustomerVisit(10,6,1,2):        w(10)(6)(1)(2)#2089  = 0
 CustomerVisit(11,6,1,2):        w(11)(6)(1)(2)#2269  = 0
 CustomerVisit(12,6,1,2):        w(12)(6)(1)(2)#2449  = 0
 CustomerVisit(5,4,2,2):         w(5)(4)(2)(2)#1139  = 0
 CustomerVisit(6,4,2,2):         w(6)(4)(2)(2)#1319  = 0
 CustomerVisit(8,4,2,2):         w(8)(4)(2)(2)#1679  = 0
 CustomerVisit(9,4,2,2):         w(9)(4)(2)(2)#1859  = 0
 CustomerVisit(10,4,2,2):        w(10)(4)(2)(2)#2039  = 0
 CustomerVisit(11,4,2,2):        w(11)(4)(2)(2)#2219  = 0
 CustomerVisit(12,4,2,2):        w(12)(4)(2)(2)#2399  = 0
 CustomerVisit(3,5,2,2):         w(3)(5)(2)(2)#809  = 0
 CustomerVisit(4,5,2,2):         w(4)(5)(2)(2)#989  = 0
 CustomerVisit(5,5,2,2):         w(5)(5)(2)(2)#1169  = 0
 CustomerVisit(6,5,2,2):         w(6)(5)(2)(2)#1349  = 0
 CustomerVisit(7,5,2,2):         w(7)(5)(2)(2)#1529  = 0
 CustomerVisit(8,5,2,2):         w(8)(5)(2)(2)#1709  = 0
 CustomerVisit(9,5,2,2):         w(9)(5)(2)(2)#1889  = 0
 CustomerVisit(10,5,2,2):        w(10)(5)(2)(2)#2069  = 0
 CustomerVisit(11,5,2,2):        w(11)(5)(2)(2)#2249  = 0
 CustomerVisit(12,5,2,2):        w(12)(5)(2)(2)#2429  = 0
 CustomerVisit(3,6,2,2):         w(3)(6)(2)(2)#839  = 0
 CustomerVisit(4,6,2,2):         w(4)(6)(2)(2)#1019  = 0
 CustomerVisit(5,6,2,2):         w(5)(6)(2)(2)#1199  = 0
 CustomerVisit(6,6,2,2):         w(6)(6)(2)(2)#1379  = 0
 CustomerVisit(7,6,2,2):         w(7)(6)(2)(2)#1559  = 0
 CustomerVisit(8,6,2,2):         w(8)(6)(2)(2)#1739  = 0
 CustomerVisit(9,6,2,2):         w(9)(6)(2)(2)#1919  = 0
 CustomerVisit(10,6,2,2):        w(10)(6)(2)(2)#2099  = 0
 CustomerVisit(11,6,2,2):        w(11)(6)(2)(2)#2279  = 0
 CustomerVisit(12,6,2,2):        w(12)(6)(2)(2)#2459  = 0
 CustomerVisit(3,4,3,2):         w(3)(4)(3)(2)#789  = 0
 CustomerVisit(4,4,3,2):         w(4)(4)(3)(2)#969  = 0
 CustomerVisit(5,4,3,2):         w(5)(4)(3)(2)#1149  = 0
 CustomerVisit(6,4,3,2):         w(6)(4)(3)(2)#1329  = 0
 CustomerVisit(7,4,3,2):         w(7)(4)(3)(2)#1509  = 0
 CustomerVisit(8,4,3,2):         w(8)(4)(3)(2)#1689  = 0
 CustomerVisit(9,4,3,2):         w(9)(4)(3)(2)#1869  = 0
 CustomerVisit(10,4,3,2):        w(10)(4)(3)(2)#2049  = 0
 CustomerVisit(11,4,3,2):        w(11)(4)(3)(2)#2229  = 0
 CustomerVisit(12,4,3,2):        w(12)(4)(3)(2)#2409  = 0
 CustomerVisit(3,5,3,2):         w(3)(5)(3)(2)#819  = 0
 CustomerVisit(4,5,3,2):         w(4)(5)(3)(2)#999  = 0
 CustomerVisit(5,5,3,2):         w(5)(5)(3)(2)#1179  = 0
 CustomerVisit(6,5,3,2):         w(6)(5)(3)(2)#1359  = 0
 CustomerVisit(7,5,3,2):         w(7)(5)(3)(2)#1539  = 0
 CustomerVisit(8,5,3,2):         w(8)(5)(3)(2)#1719  = 0
 CustomerVisit(9,5,3,2):         w(9)(5)(3)(2)#1899  = 0
 CustomerVisit(10,5,3,2):        w(10)(5)(3)(2)#2079  = 0
 CustomerVisit(11,5,3,2):        w(11)(5)(3)(2)#2259  = 0
 CustomerVisit(12,5,3,2):        w(12)(5)(3)(2)#2439  = 0
 CustomerVisit(3,6,3,2):         w(3)(6)(3)(2)#849  = 0
 CustomerVisit(4,6,3,2):         w(4)(6)(3)(2)#1029  = 0
 CustomerVisit(5,6,3,2):         w(5)(6)(3)(2)#1209  = 0
 CustomerVisit(6,6,3,2):         w(6)(6)(3)(2)#1389  = 0
 CustomerVisit(7,6,3,2):         w(7)(6)(3)(2)#1569  = 0
 CustomerVisit(8,6,3,2):         w(8)(6)(3)(2)#1749  = 0
 CustomerVisit(9,6,3,2):         w(9)(6)(3)(2)#1929  = 0
 CustomerVisit(10,6,3,2):        w(10)(6)(3)(2)#2109  = 0
 CustomerVisit(11,6,3,2):        w(11)(6)(3)(2)#2289  = 0
 CustomerVisit(12,6,3,2):        w(12)(6)(3)(2)#2469  = 0
 CustomerVisit(3,1,1,3):         w(3)(1)(1)(3)#680  = 0
 CustomerVisit(4,1,1,3):         w(4)(1)(1)(3)#860  = 0
 CustomerVisit(5,1,1,3):         w(5)(1)(1)(3)#1040  = 0
 CustomerVisit(6,1,1,3):         w(6)(1)(1)(3)#1220  = 0
 CustomerVisit(7,1,1,3):         w(7)(1)(1)(3)#1400  = 0
 CustomerVisit(8,1,1,3):         w(8)(1)(1)(3)#1580  = 0
 CustomerVisit(9,1,1,3):         w(9)(1)(1)(3)#1760  = 0
 CustomerVisit(10,1,1,3):        w(10)(1)(1)(3)#1940  = 0
 CustomerVisit(11,1,1,3):        w(11)(1)(1)(3)#2120  = 0
 CustomerVisit(12,1,1,3):        w(12)(1)(1)(3)#2300  = 0
 CustomerVisit(3,2,1,3):         w(3)(2)(1)(3)#710  = 0
 CustomerVisit(4,2,1,3):         w(4)(2)(1)(3)#890  = 0
 CustomerVisit(6,2,1,3):         w(6)(2)(1)(3)#1250  = 0
 CustomerVisit(7,2,1,3):         w(7)(2)(1)(3)#1430  = 0
 CustomerVisit(8,2,1,3):         w(8)(2)(1)(3)#1610  = 0
 CustomerVisit(9,2,1,3):         w(9)(2)(1)(3)#1790  = 0
 CustomerVisit(10,2,1,3):        w(10)(2)(1)(3)#1970  = 0
 CustomerVisit(11,2,1,3):        w(11)(2)(1)(3)#2150  = 0
 CustomerVisit(12,2,1,3):        w(12)(2)(1)(3)#2330  = 0
 CustomerVisit(3,3,1,3):         w(3)(3)(1)(3)#740  = 0
 CustomerVisit(4,3,1,3):         w(4)(3)(1)(3)#920  = 0
 CustomerVisit(5,3,1,3):         w(5)(3)(1)(3)#1100  = 0
 CustomerVisit(7,3,1,3):         w(7)(3)(1)(3)#1460  = 0
 CustomerVisit(8,3,1,3):         w(8)(3)(1)(3)#1640  = 0
 CustomerVisit(9,3,1,3):         w(9)(3)(1)(3)#1820  = 0
 CustomerVisit(12,3,1,3):        w(12)(3)(1)(3)#2360  = 0
 CustomerVisit(3,1,2,3):         w(3)(1)(2)(3)#690  = 0
 CustomerVisit(4,1,2,3):         w(4)(1)(2)(3)#870  = 0
 CustomerVisit(5,1,2,3):         w(5)(1)(2)(3)#1050  = 0
 CustomerVisit(6,1,2,3):         w(6)(1)(2)(3)#1230  = 0
 CustomerVisit(7,1,2,3):         w(7)(1)(2)(3)#1410  = 0
 CustomerVisit(8,1,2,3):         w(8)(1)(2)(3)#1590  = 0
 CustomerVisit(9,1,2,3):         w(9)(1)(2)(3)#1770  = 0
 CustomerVisit(10,1,2,3):        w(10)(1)(2)(3)#1950  = 0
 CustomerVisit(11,1,2,3):        w(11)(1)(2)(3)#2130  = 0
 CustomerVisit(12,1,2,3):        w(12)(1)(2)(3)#2310  = 0
 CustomerVisit(3,2,2,3):         w(3)(2)(2)(3)#720  = 0
 CustomerVisit(4,2,2,3):         w(4)(2)(2)(3)#900  = 0
 CustomerVisit(6,2,2,3):         w(6)(2)(2)(3)#1260  = 0
 CustomerVisit(7,2,2,3):         w(7)(2)(2)(3)#1440  = 0
 CustomerVisit(8,2,2,3):         w(8)(2)(2)(3)#1620  = 0
 CustomerVisit(9,2,2,3):         w(9)(2)(2)(3)#1800  = 0
 CustomerVisit(10,2,2,3):        w(10)(2)(2)(3)#1980  = 0
 CustomerVisit(11,2,2,3):        w(11)(2)(2)(3)#2160  = 0
 CustomerVisit(3,3,2,3):         w(3)(3)(2)(3)#750  = 0
 CustomerVisit(4,3,2,3):         w(4)(3)(2)(3)#930  = 0
 CustomerVisit(5,3,2,3):         w(5)(3)(2)(3)#1110  = 0
 CustomerVisit(6,3,2,3):         w(6)(3)(2)(3)#1290  = 0
 CustomerVisit(7,3,2,3):         w(7)(3)(2)(3)#1470  = 0
 CustomerVisit(8,3,2,3):         w(8)(3)(2)(3)#1650  = 0
 CustomerVisit(9,3,2,3):         w(9)(3)(2)(3)#1830  = 0
 CustomerVisit(10,3,2,3):        w(10)(3)(2)(3)#2010  = 0
 CustomerVisit(11,3,2,3):        w(11)(3)(2)(3)#2190  = 0
 CustomerVisit(12,3,2,3):        w(12)(3)(2)(3)#2370  = 0
 CustomerVisit(4,1,3,3):         w(4)(1)(3)(3)#880  = 0
 CustomerVisit(5,1,3,3):         w(5)(1)(3)(3)#1060  = 0
 CustomerVisit(7,1,3,3):         w(7)(1)(3)(3)#1420  = 0
 CustomerVisit(8,1,3,3):         w(8)(1)(3)(3)#1600  = 0
 CustomerVisit(11,1,3,3):        w(11)(1)(3)(3)#2140  = 0
 CustomerVisit(12,1,3,3):        w(12)(1)(3)(3)#2320  = 0
 CustomerVisit(3,2,3,3):         w(3)(2)(3)(3)#730  = 0
 CustomerVisit(4,2,3,3):         w(4)(2)(3)(3)#910  = 0
 CustomerVisit(5,2,3,3):         w(5)(2)(3)(3)#1090  = 0
 CustomerVisit(6,2,3,3):         w(6)(2)(3)(3)#1270  = 0
 CustomerVisit(7,2,3,3):         w(7)(2)(3)(3)#1450  = 0
 CustomerVisit(8,2,3,3):         w(8)(2)(3)(3)#1630  = 0
 CustomerVisit(9,2,3,3):         w(9)(2)(3)(3)#1810  = 0
 CustomerVisit(10,2,3,3):        w(10)(2)(3)(3)#1990  = 0
 CustomerVisit(11,2,3,3):        w(11)(2)(3)(3)#2170  = 0
 CustomerVisit(12,2,3,3):        w(12)(2)(3)(3)#2350  = 0
 CustomerVisit(3,3,3,3):         w(3)(3)(3)(3)#760  = 0
 CustomerVisit(4,3,3,3):         w(4)(3)(3)(3)#940  = 0
 CustomerVisit(5,3,3,3):         w(5)(3)(3)(3)#1120  = 0
 CustomerVisit(6,3,3,3):         w(6)(3)(3)(3)#1300  = 0
 CustomerVisit(7,3,3,3):         w(7)(3)(3)(3)#1480  = 0
 CustomerVisit(8,3,3,3):         w(8)(3)(3)(3)#1660  = 0
 CustomerVisit(9,3,3,3):         w(9)(3)(3)(3)#1840  = 0
 CustomerVisit(10,3,3,3):        w(10)(3)(3)(3)#2020  = 0
 CustomerVisit(11,3,3,3):        w(11)(3)(3)(3)#2200  = 0
 CustomerVisit(12,3,3,3):        w(12)(3)(3)(3)#2380  = 0
 CustomerVisit(3,4,1,3):         w(3)(4)(1)(3)#770  = 0
 CustomerVisit(4,4,1,3):         w(4)(4)(1)(3)#950  = 0
 CustomerVisit(5,4,1,3):         w(5)(4)(1)(3)#1130  = 0
 CustomerVisit(6,4,1,3):         w(6)(4)(1)(3)#1310  = 0
 CustomerVisit(7,4,1,3):         w(7)(4)(1)(3)#1490  = 0
 CustomerVisit(8,4,1,3):         w(8)(4)(1)(3)#1670  = 0
 CustomerVisit(9,4,1,3):         w(9)(4)(1)(3)#1850  = 0
 CustomerVisit(10,4,1,3):        w(10)(4)(1)(3)#2030  = 0
 CustomerVisit(11,4,1,3):        w(11)(4)(1)(3)#2210  = 0
 CustomerVisit(12,4,1,3):        w(12)(4)(1)(3)#2390  = 0
 CustomerVisit(5,5,1,3):         w(5)(5)(1)(3)#1160  = 0
 CustomerVisit(6,5,1,3):         w(6)(5)(1)(3)#1340  = 0
 CustomerVisit(8,5,1,3):         w(8)(5)(1)(3)#1700  = 0
 CustomerVisit(9,5,1,3):         w(9)(5)(1)(3)#1880  = 0
 CustomerVisit(10,5,1,3):        w(10)(5)(1)(3)#2060  = 0
 CustomerVisit(11,5,1,3):        w(11)(5)(1)(3)#2240  = 0
 CustomerVisit(12,5,1,3):        w(12)(5)(1)(3)#2420  = 0
 CustomerVisit(3,6,1,3):         w(3)(6)(1)(3)#830  = 0
 CustomerVisit(4,6,1,3):         w(4)(6)(1)(3)#1010  = 0
 CustomerVisit(5,6,1,3):         w(5)(6)(1)(3)#1190  = 0
 CustomerVisit(6,6,1,3):         w(6)(6)(1)(3)#1370  = 0
 CustomerVisit(7,6,1,3):         w(7)(6)(1)(3)#1550  = 0
 CustomerVisit(8,6,1,3):         w(8)(6)(1)(3)#1730  = 0
 CustomerVisit(9,6,1,3):         w(9)(6)(1)(3)#1910  = 0
 CustomerVisit(10,6,1,3):        w(10)(6)(1)(3)#2090  = 0
 CustomerVisit(11,6,1,3):        w(11)(6)(1)(3)#2270  = 0
 CustomerVisit(12,6,1,3):        w(12)(6)(1)(3)#2450  = 0
 CustomerVisit(3,4,2,3):         w(3)(4)(2)(3)#780  = 0
 CustomerVisit(5,4,2,3):         w(5)(4)(2)(3)#1140  = 0
 CustomerVisit(6,4,2,3):         w(6)(4)(2)(3)#1320  = 0
 CustomerVisit(7,4,2,3):         w(7)(4)(2)(3)#1500  = 0
 CustomerVisit(8,4,2,3):         w(8)(4)(2)(3)#1680  = 0
 CustomerVisit(9,4,2,3):         w(9)(4)(2)(3)#1860  = 0
 CustomerVisit(10,4,2,3):        w(10)(4)(2)(3)#2040  = 0
 CustomerVisit(11,4,2,3):        w(11)(4)(2)(3)#2220  = 0
 CustomerVisit(12,4,2,3):        w(12)(4)(2)(3)#2400  = 0
 CustomerVisit(3,5,2,3):         w(3)(5)(2)(3)#810  = 0
 CustomerVisit(4,5,2,3):         w(4)(5)(2)(3)#990  = 0
 CustomerVisit(5,5,2,3):         w(5)(5)(2)(3)#1170  = 0
 CustomerVisit(6,5,2,3):         w(6)(5)(2)(3)#1350  = 0
 CustomerVisit(7,5,2,3):         w(7)(5)(2)(3)#1530  = 0
 CustomerVisit(8,5,2,3):         w(8)(5)(2)(3)#1710  = 0
 CustomerVisit(9,5,2,3):         w(9)(5)(2)(3)#1890  = 0
 CustomerVisit(10,5,2,3):        w(10)(5)(2)(3)#2070  = 0
 CustomerVisit(11,5,2,3):        w(11)(5)(2)(3)#2250  = 0
 CustomerVisit(12,5,2,3):        w(12)(5)(2)(3)#2430  = 0
 CustomerVisit(4,6,2,3):         w(4)(6)(2)(3)#1020  = 0
 CustomerVisit(5,6,2,3):         w(5)(6)(2)(3)#1200  = 0
 CustomerVisit(9,6,2,3):         w(9)(6)(2)(3)#1920  = 0
 CustomerVisit(11,6,2,3):        w(11)(6)(2)(3)#2280  = 0
 CustomerVisit(12,6,2,3):        w(12)(6)(2)(3)#2460  = 0
 CustomerVisit(3,4,3,3):         w(3)(4)(3)(3)#790  = 0
 CustomerVisit(4,4,3,3):         w(4)(4)(3)(3)#970  = 0
 CustomerVisit(5,4,3,3):         w(5)(4)(3)(3)#1150  = 0
 CustomerVisit(6,4,3,3):         w(6)(4)(3)(3)#1330  = 0
 CustomerVisit(7,4,3,3):         w(7)(4)(3)(3)#1510  = 0
 CustomerVisit(8,4,3,3):         w(8)(4)(3)(3)#1690  = 0
 CustomerVisit(9,4,3,3):         w(9)(4)(3)(3)#1870  = 0
 CustomerVisit(10,4,3,3):        w(10)(4)(3)(3)#2050  = 0
 CustomerVisit(11,4,3,3):        w(11)(4)(3)(3)#2230  = 0
 CustomerVisit(12,4,3,3):        w(12)(4)(3)(3)#2410  = 0
 CustomerVisit(3,5,3,3):         w(3)(5)(3)(3)#820  = 0
 CustomerVisit(4,5,3,3):         w(4)(5)(3)(3)#1000  = 0
 CustomerVisit(5,5,3,3):         w(5)(5)(3)(3)#1180  = 0
 CustomerVisit(6,5,3,3):         w(6)(5)(3)(3)#1360  = 0
 CustomerVisit(7,5,3,3):         w(7)(5)(3)(3)#1540  = 0
 CustomerVisit(8,5,3,3):         w(8)(5)(3)(3)#1720  = 0
 CustomerVisit(9,5,3,3):         w(9)(5)(3)(3)#1900  = 0
 CustomerVisit(10,5,3,3):        w(10)(5)(3)(3)#2080  = 0
 CustomerVisit(11,5,3,3):        w(11)(5)(3)(3)#2260  = 0
 CustomerVisit(12,5,3,3):        w(12)(5)(3)(3)#2440  = 0
 CustomerVisit(3,6,3,3):         w(3)(6)(3)(3)#850  = 0
 CustomerVisit(4,6,3,3):         w(4)(6)(3)(3)#1030  = 0
 CustomerVisit(5,6,3,3):         w(5)(6)(3)(3)#1210  = 0
 CustomerVisit(6,6,3,3):         w(6)(6)(3)(3)#1390  = 0
 CustomerVisit(7,6,3,3):         w(7)(6)(3)(3)#1570  = 0
 CustomerVisit(8,6,3,3):         w(8)(6)(3)(3)#1750  = 0
 CustomerVisit(9,6,3,3):         w(9)(6)(3)(3)#1930  = 0
 CustomerVisit(10,6,3,3):        w(10)(6)(3)(3)#2110  = 0
 CustomerVisit(11,6,3,3):        w(11)(6)(3)(3)#2290  = 0
 CustomerVisit(12,6,3,3):        w(12)(6)(3)(3)#2470  = 0
 CustomerVisit(3,1,1,4):         w(3)(1)(1)(4)#681  = 0
 CustomerVisit(4,1,1,4):         w(4)(1)(1)(4)#861  = 0
 CustomerVisit(7,1,1,4):         w(7)(1)(1)(4)#1401  = 0
 CustomerVisit(8,1,1,4):         w(8)(1)(1)(4)#1581  = 0
 CustomerVisit(9,1,1,4):         w(9)(1)(1)(4)#1761  = 0
 CustomerVisit(11,1,1,4):        w(11)(1)(1)(4)#2121  = 0
 CustomerVisit(3,2,1,4):         w(3)(2)(1)(4)#711  = 0
 CustomerVisit(4,2,1,4):         w(4)(2)(1)(4)#891  = 0
 CustomerVisit(5,2,1,4):         w(5)(2)(1)(4)#1071  = 0
 CustomerVisit(6,2,1,4):         w(6)(2)(1)(4)#1251  = 0
 CustomerVisit(7,2,1,4):         w(7)(2)(1)(4)#1431  = 0
 CustomerVisit(8,2,1,4):         w(8)(2)(1)(4)#1611  = 0
 CustomerVisit(9,2,1,4):         w(9)(2)(1)(4)#1791  = 0
 CustomerVisit(10,2,1,4):        w(10)(2)(1)(4)#1971  = 0
 CustomerVisit(11,2,1,4):        w(11)(2)(1)(4)#2151  = 0
 CustomerVisit(12,2,1,4):        w(12)(2)(1)(4)#2331  = 0
 CustomerVisit(3,3,1,4):         w(3)(3)(1)(4)#741  = 0
 CustomerVisit(4,3,1,4):         w(4)(3)(1)(4)#921  = 0
 CustomerVisit(5,3,1,4):         w(5)(3)(1)(4)#1101  = 0
 CustomerVisit(6,3,1,4):         w(6)(3)(1)(4)#1281  = 0
 CustomerVisit(7,3,1,4):         w(7)(3)(1)(4)#1461  = 0
 CustomerVisit(8,3,1,4):         w(8)(3)(1)(4)#1641  = 0
 CustomerVisit(9,3,1,4):         w(9)(3)(1)(4)#1821  = 0
 CustomerVisit(10,3,1,4):        w(10)(3)(1)(4)#2001  = 0
 CustomerVisit(11,3,1,4):        w(11)(3)(1)(4)#2181  = 0
 CustomerVisit(12,3,1,4):        w(12)(3)(1)(4)#2361  = 0
 CustomerVisit(4,1,2,4):         w(4)(1)(2)(4)#871  = 0
 CustomerVisit(5,1,2,4):         w(5)(1)(2)(4)#1051  = 0
 CustomerVisit(7,1,2,4):         w(7)(1)(2)(4)#1411  = 0
 CustomerVisit(11,1,2,4):        w(11)(1)(2)(4)#2131  = 0
 CustomerVisit(12,1,2,4):        w(12)(1)(2)(4)#2311  = 0
 CustomerVisit(3,2,2,4):         w(3)(2)(2)(4)#721  = 0
 CustomerVisit(4,2,2,4):         w(4)(2)(2)(4)#901  = 0
 CustomerVisit(5,2,2,4):         w(5)(2)(2)(4)#1081  = 0
 CustomerVisit(6,2,2,4):         w(6)(2)(2)(4)#1261  = 0
 CustomerVisit(7,2,2,4):         w(7)(2)(2)(4)#1441  = 0
 CustomerVisit(8,2,2,4):         w(8)(2)(2)(4)#1621  = 0
 CustomerVisit(9,2,2,4):         w(9)(2)(2)(4)#1801  = 0
 CustomerVisit(10,2,2,4):        w(10)(2)(2)(4)#1981  = 0
 CustomerVisit(11,2,2,4):        w(11)(2)(2)(4)#2161  = 0
 CustomerVisit(12,2,2,4):        w(12)(2)(2)(4)#2341  = 0
 CustomerVisit(3,3,2,4):         w(3)(3)(2)(4)#751  = 0
 CustomerVisit(4,3,2,4):         w(4)(3)(2)(4)#931  = 0
 CustomerVisit(5,3,2,4):         w(5)(3)(2)(4)#1111  = 0
 CustomerVisit(6,3,2,4):         w(6)(3)(2)(4)#1291  = 0
 CustomerVisit(7,3,2,4):         w(7)(3)(2)(4)#1471  = 0
 CustomerVisit(8,3,2,4):         w(8)(3)(2)(4)#1651  = 0
 CustomerVisit(9,3,2,4):         w(9)(3)(2)(4)#1831  = 0
 CustomerVisit(10,3,2,4):        w(10)(3)(2)(4)#2011  = 0
 CustomerVisit(11,3,2,4):        w(11)(3)(2)(4)#2191  = 0
 CustomerVisit(12,3,2,4):        w(12)(3)(2)(4)#2371  = 0
 CustomerVisit(3,1,3,4):         w(3)(1)(3)(4)#701  = 0
 CustomerVisit(4,1,3,4):         w(4)(1)(3)(4)#881  = 0
 CustomerVisit(5,1,3,4):         w(5)(1)(3)(4)#1061  = 0
 CustomerVisit(6,1,3,4):         w(6)(1)(3)(4)#1241  = 0
 CustomerVisit(7,1,3,4):         w(7)(1)(3)(4)#1421  = 0
 CustomerVisit(8,1,3,4):         w(8)(1)(3)(4)#1601  = 0
 CustomerVisit(9,1,3,4):         w(9)(1)(3)(4)#1781  = 0
 CustomerVisit(10,1,3,4):        w(10)(1)(3)(4)#1961  = 0
 CustomerVisit(11,1,3,4):        w(11)(1)(3)(4)#2141  = 0
 CustomerVisit(12,1,3,4):        w(12)(1)(3)(4)#2321  = 0
 CustomerVisit(3,2,3,4):         w(3)(2)(3)(4)#731  = 0
 CustomerVisit(4,2,3,4):         w(4)(2)(3)(4)#911  = 0
 CustomerVisit(5,2,3,4):         w(5)(2)(3)(4)#1091  = 0
 CustomerVisit(7,2,3,4):         w(7)(2)(3)(4)#1451  = 0
 CustomerVisit(8,2,3,4):         w(8)(2)(3)(4)#1631  = 0
 CustomerVisit(9,2,3,4):         w(9)(2)(3)(4)#1811  = 0
 CustomerVisit(3,3,3,4):         w(3)(3)(3)(4)#761  = 0
 CustomerVisit(4,3,3,4):         w(4)(3)(3)(4)#941  = 0
 CustomerVisit(6,3,3,4):         w(6)(3)(3)(4)#1301  = 0
 CustomerVisit(7,3,3,4):         w(7)(3)(3)(4)#1481  = 0
 CustomerVisit(8,3,3,4):         w(8)(3)(3)(4)#1661  = 0
 CustomerVisit(9,3,3,4):         w(9)(3)(3)(4)#1841  = 0
 CustomerVisit(10,3,3,4):        w(10)(3)(3)(4)#2021  = 0
 CustomerVisit(11,3,3,4):        w(11)(3)(3)(4)#2201  = 0
 CustomerVisit(12,3,3,4):        w(12)(3)(3)(4)#2381  = 0
 CustomerVisit(5,4,1,4):         w(5)(4)(1)(4)#1131  = 0
 CustomerVisit(6,4,1,4):         w(6)(4)(1)(4)#1311  = 0
 CustomerVisit(8,4,1,4):         w(8)(4)(1)(4)#1671  = 0
 CustomerVisit(9,4,1,4):         w(9)(4)(1)(4)#1851  = 0
 CustomerVisit(10,4,1,4):        w(10)(4)(1)(4)#2031  = 0
 CustomerVisit(11,4,1,4):        w(11)(4)(1)(4)#2211  = 0
 CustomerVisit(12,4,1,4):        w(12)(4)(1)(4)#2391  = 0
 CustomerVisit(3,5,1,4):         w(3)(5)(1)(4)#801  = 0
 CustomerVisit(4,5,1,4):         w(4)(5)(1)(4)#981  = 0
 CustomerVisit(5,5,1,4):         w(5)(5)(1)(4)#1161  = 0
 CustomerVisit(6,5,1,4):         w(6)(5)(1)(4)#1341  = 0
 CustomerVisit(7,5,1,4):         w(7)(5)(1)(4)#1521  = 0
 CustomerVisit(8,5,1,4):         w(8)(5)(1)(4)#1701  = 0
 CustomerVisit(9,5,1,4):         w(9)(5)(1)(4)#1881  = 0
 CustomerVisit(10,5,1,4):        w(10)(5)(1)(4)#2061  = 0
 CustomerVisit(11,5,1,4):        w(11)(5)(1)(4)#2241  = 0
 CustomerVisit(12,5,1,4):        w(12)(5)(1)(4)#2421  = 0
 CustomerVisit(3,6,1,4):         w(3)(6)(1)(4)#831  = 0
 CustomerVisit(4,6,1,4):         w(4)(6)(1)(4)#1011  = 0
 CustomerVisit(5,6,1,4):         w(5)(6)(1)(4)#1191  = 0
 CustomerVisit(6,6,1,4):         w(6)(6)(1)(4)#1371  = 0
 CustomerVisit(7,6,1,4):         w(7)(6)(1)(4)#1551  = 0
 CustomerVisit(8,6,1,4):         w(8)(6)(1)(4)#1731  = 0
 CustomerVisit(9,6,1,4):         w(9)(6)(1)(4)#1911  = 0
 CustomerVisit(10,6,1,4):        w(10)(6)(1)(4)#2091  = 0
 CustomerVisit(11,6,1,4):        w(11)(6)(1)(4)#2271  = 0
 CustomerVisit(12,6,1,4):        w(12)(6)(1)(4)#2451  = 0
 CustomerVisit(3,4,2,4):         w(3)(4)(2)(4)#781  = 0
 CustomerVisit(4,4,2,4):         w(4)(4)(2)(4)#961  = 0
 CustomerVisit(5,4,2,4):         w(5)(4)(2)(4)#1141  = 0
 CustomerVisit(6,4,2,4):         w(6)(4)(2)(4)#1321  = 0
 CustomerVisit(7,4,2,4):         w(7)(4)(2)(4)#1501  = 0
 CustomerVisit(8,4,2,4):         w(8)(4)(2)(4)#1681  = 0
 CustomerVisit(9,4,2,4):         w(9)(4)(2)(4)#1861  = 0
 CustomerVisit(10,4,2,4):        w(10)(4)(2)(4)#2041  = 0
 CustomerVisit(11,4,2,4):        w(11)(4)(2)(4)#2221  = 0
 CustomerVisit(12,4,2,4):        w(12)(4)(2)(4)#2401  = 0
 CustomerVisit(3,5,2,4):         w(3)(5)(2)(4)#811  = 0
 CustomerVisit(4,5,2,4):         w(4)(5)(2)(4)#991  = 0
 CustomerVisit(5,5,2,4):         w(5)(5)(2)(4)#1171  = 0
 CustomerVisit(6,5,2,4):         w(6)(5)(2)(4)#1351  = 0
 CustomerVisit(7,5,2,4):         w(7)(5)(2)(4)#1531  = 0
 CustomerVisit(8,5,2,4):         w(8)(5)(2)(4)#1711  = 0
 CustomerVisit(9,5,2,4):         w(9)(5)(2)(4)#1891  = 0
 CustomerVisit(10,5,2,4):        w(10)(5)(2)(4)#2071  = 0
 CustomerVisit(11,5,2,4):        w(11)(5)(2)(4)#2251  = 0
 CustomerVisit(12,5,2,4):        w(12)(5)(2)(4)#2431  = 0
 CustomerVisit(3,6,2,4):         w(3)(6)(2)(4)#841  = 0
 CustomerVisit(4,6,2,4):         w(4)(6)(2)(4)#1021  = 0
 CustomerVisit(5,6,2,4):         w(5)(6)(2)(4)#1201  = 0
 CustomerVisit(6,6,2,4):         w(6)(6)(2)(4)#1381  = 0
 CustomerVisit(7,6,2,4):         w(7)(6)(2)(4)#1561  = 0
 CustomerVisit(8,6,2,4):         w(8)(6)(2)(4)#1741  = 0
 CustomerVisit(9,6,2,4):         w(9)(6)(2)(4)#1921  = 0
 CustomerVisit(10,6,2,4):        w(10)(6)(2)(4)#2101  = 0
 CustomerVisit(11,6,2,4):        w(11)(6)(2)(4)#2281  = 0
 CustomerVisit(12,6,2,4):        w(12)(6)(2)(4)#2461  = 0
 CustomerVisit(5,4,3,4):         w(5)(4)(3)(4)#1151  = 0
 CustomerVisit(6,4,3,4):         w(6)(4)(3)(4)#1331  = 0
 CustomerVisit(8,4,3,4):         w(8)(4)(3)(4)#1691  = 0
 CustomerVisit(9,4,3,4):         w(9)(4)(3)(4)#1871  = 0
 CustomerVisit(10,4,3,4):        w(10)(4)(3)(4)#2051  = 0
 CustomerVisit(11,4,3,4):        w(11)(4)(3)(4)#2231  = 0
 CustomerVisit(12,4,3,4):        w(12)(4)(3)(4)#2411  = 0
 CustomerVisit(3,5,3,4):         w(3)(5)(3)(4)#821  = 0
 CustomerVisit(4,5,3,4):         w(4)(5)(3)(4)#1001  = 0
 CustomerVisit(5,5,3,4):         w(5)(5)(3)(4)#1181  = 0
 CustomerVisit(6,5,3,4):         w(6)(5)(3)(4)#1361  = 0
 CustomerVisit(7,5,3,4):         w(7)(5)(3)(4)#1541  = 0
 CustomerVisit(8,5,3,4):         w(8)(5)(3)(4)#1721  = 0
 CustomerVisit(9,5,3,4):         w(9)(5)(3)(4)#1901  = 0
 CustomerVisit(10,5,3,4):        w(10)(5)(3)(4)#2081  = 0
 CustomerVisit(11,5,3,4):        w(11)(5)(3)(4)#2261  = 0
 CustomerVisit(12,5,3,4):        w(12)(5)(3)(4)#2441  = 0
 CustomerVisit(3,6,3,4):         w(3)(6)(3)(4)#851  = 0
 CustomerVisit(4,6,3,4):         w(4)(6)(3)(4)#1031  = 0
 CustomerVisit(5,6,3,4):         w(5)(6)(3)(4)#1211  = 0
 CustomerVisit(6,6,3,4):         w(6)(6)(3)(4)#1391  = 0
 CustomerVisit(7,6,3,4):         w(7)(6)(3)(4)#1571  = 0
 CustomerVisit(8,6,3,4):         w(8)(6)(3)(4)#1751  = 0
 CustomerVisit(9,6,3,4):         w(9)(6)(3)(4)#1931  = 0
 CustomerVisit(10,6,3,4):        w(10)(6)(3)(4)#2111  = 0
 CustomerVisit(11,6,3,4):        w(11)(6)(3)(4)#2291  = 0
 CustomerVisit(12,6,3,4):        w(12)(6)(3)(4)#2471  = 0
 CustomerVisit(3,1,1,5):         w(3)(1)(1)(5)#682  = 0
 CustomerVisit(4,1,1,5):         w(4)(1)(1)(5)#862  = 0
 CustomerVisit(7,1,1,5):         w(7)(1)(1)(5)#1402  = 0
 CustomerVisit(8,1,1,5):         w(8)(1)(1)(5)#1582  = 0
 CustomerVisit(9,1,1,5):         w(9)(1)(1)(5)#1762  = 0
 CustomerVisit(12,1,1,5):        w(12)(1)(1)(5)#2302  = 0
 CustomerVisit(3,2,1,5):         w(3)(2)(1)(5)#712  = 0
 CustomerVisit(4,2,1,5):         w(4)(2)(1)(5)#892  = 0
 CustomerVisit(5,2,1,5):         w(5)(2)(1)(5)#1072  = 0
 CustomerVisit(6,2,1,5):         w(6)(2)(1)(5)#1252  = 0
 CustomerVisit(7,2,1,5):         w(7)(2)(1)(5)#1432  = 0
 CustomerVisit(8,2,1,5):         w(8)(2)(1)(5)#1612  = 0
 CustomerVisit(9,2,1,5):         w(9)(2)(1)(5)#1792  = 0
 CustomerVisit(10,2,1,5):        w(10)(2)(1)(5)#1972  = 0
 CustomerVisit(11,2,1,5):        w(11)(2)(1)(5)#2152  = 0
 CustomerVisit(12,2,1,5):        w(12)(2)(1)(5)#2332  = 0
 CustomerVisit(3,3,1,5):         w(3)(3)(1)(5)#742  = 0
 CustomerVisit(4,3,1,5):         w(4)(3)(1)(5)#922  = 0
 CustomerVisit(5,3,1,5):         w(5)(3)(1)(5)#1102  = 0
 CustomerVisit(6,3,1,5):         w(6)(3)(1)(5)#1282  = 0
 CustomerVisit(7,3,1,5):         w(7)(3)(1)(5)#1462  = 0
 CustomerVisit(8,3,1,5):         w(8)(3)(1)(5)#1642  = 0
 CustomerVisit(9,3,1,5):         w(9)(3)(1)(5)#1822  = 0
 CustomerVisit(10,3,1,5):        w(10)(3)(1)(5)#2002  = 0
 CustomerVisit(11,3,1,5):        w(11)(3)(1)(5)#2182  = 0
 CustomerVisit(12,3,1,5):        w(12)(3)(1)(5)#2362  = 0
 CustomerVisit(3,1,2,5):         w(3)(1)(2)(5)#692  = 0
 CustomerVisit(4,1,2,5):         w(4)(1)(2)(5)#872  = 0
 CustomerVisit(5,1,2,5):         w(5)(1)(2)(5)#1052  = 0
 CustomerVisit(7,1,2,5):         w(7)(1)(2)(5)#1412  = 0
 CustomerVisit(8,1,2,5):         w(8)(1)(2)(5)#1592  = 0
 CustomerVisit(9,1,2,5):         w(9)(1)(2)(5)#1772  = 0
 CustomerVisit(11,1,2,5):        w(11)(1)(2)(5)#2132  = 0
 CustomerVisit(3,2,2,5):         w(3)(2)(2)(5)#722  = 0
 CustomerVisit(4,2,2,5):         w(4)(2)(2)(5)#902  = 0
 CustomerVisit(5,2,2,5):         w(5)(2)(2)(5)#1082  = 0
 CustomerVisit(6,2,2,5):         w(6)(2)(2)(5)#1262  = 0
 CustomerVisit(7,2,2,5):         w(7)(2)(2)(5)#1442  = 0
 CustomerVisit(8,2,2,5):         w(8)(2)(2)(5)#1622  = 0
 CustomerVisit(9,2,2,5):         w(9)(2)(2)(5)#1802  = 0
 CustomerVisit(10,2,2,5):        w(10)(2)(2)(5)#1982  = 0
 CustomerVisit(11,2,2,5):        w(11)(2)(2)(5)#2162  = 0
 CustomerVisit(12,2,2,5):        w(12)(2)(2)(5)#2342  = 0
 CustomerVisit(3,3,2,5):         w(3)(3)(2)(5)#752  = 0
 CustomerVisit(4,3,2,5):         w(4)(3)(2)(5)#932  = 0
 CustomerVisit(5,3,2,5):         w(5)(3)(2)(5)#1112  = 0
 CustomerVisit(6,3,2,5):         w(6)(3)(2)(5)#1292  = 0
 CustomerVisit(7,3,2,5):         w(7)(3)(2)(5)#1472  = 0
 CustomerVisit(8,3,2,5):         w(8)(3)(2)(5)#1652  = 0
 CustomerVisit(9,3,2,5):         w(9)(3)(2)(5)#1832  = 0
 CustomerVisit(10,3,2,5):        w(10)(3)(2)(5)#2012  = 0
 CustomerVisit(11,3,2,5):        w(11)(3)(2)(5)#2192  = 0
 CustomerVisit(12,3,2,5):        w(12)(3)(2)(5)#2372  = 0
 CustomerVisit(3,1,3,5):         w(3)(1)(3)(5)#702  = 0
 CustomerVisit(4,1,3,5):         w(4)(1)(3)(5)#882  = 0
 CustomerVisit(6,1,3,5):         w(6)(1)(3)(5)#1242  = 0
 CustomerVisit(7,1,3,5):         w(7)(1)(3)(5)#1422  = 0
 CustomerVisit(8,1,3,5):         w(8)(1)(3)(5)#1602  = 0
 CustomerVisit(9,1,3,5):         w(9)(1)(3)(5)#1782  = 0
 CustomerVisit(10,1,3,5):        w(10)(1)(3)(5)#1962  = 0
 CustomerVisit(11,1,3,5):        w(11)(1)(3)(5)#2142  = 0
 CustomerVisit(12,1,3,5):        w(12)(1)(3)(5)#2322  = 0
 CustomerVisit(4,2,3,5):         w(4)(2)(3)(5)#912  = 0
 CustomerVisit(5,2,3,5):         w(5)(2)(3)(5)#1092  = 0
 CustomerVisit(7,2,3,5):         w(7)(2)(3)(5)#1452  = 0
 CustomerVisit(11,2,3,5):        w(11)(2)(3)(5)#2172  = 0
 CustomerVisit(12,2,3,5):        w(12)(2)(3)(5)#2352  = 0
 CustomerVisit(3,3,3,5):         w(3)(3)(3)(5)#762  = 0
 CustomerVisit(4,3,3,5):         w(4)(3)(3)(5)#942  = 0
 CustomerVisit(5,3,3,5):         w(5)(3)(3)(5)#1122  = 0
 CustomerVisit(6,3,3,5):         w(6)(3)(3)(5)#1302  = 0
 CustomerVisit(7,3,3,5):         w(7)(3)(3)(5)#1482  = 0
 CustomerVisit(8,3,3,5):         w(8)(3)(3)(5)#1662  = 0
 CustomerVisit(9,3,3,5):         w(9)(3)(3)(5)#1842  = 0
 CustomerVisit(10,3,3,5):        w(10)(3)(3)(5)#2022  = 0
 CustomerVisit(11,3,3,5):        w(11)(3)(3)(5)#2202  = 0
 CustomerVisit(12,3,3,5):        w(12)(3)(3)(5)#2382  = 0
 CustomerVisit(5,4,1,5):         w(5)(4)(1)(5)#1132  = 0
 CustomerVisit(6,4,1,5):         w(6)(4)(1)(5)#1312  = 0
 CustomerVisit(8,4,1,5):         w(8)(4)(1)(5)#1672  = 0
 CustomerVisit(9,4,1,5):         w(9)(4)(1)(5)#1852  = 0
 CustomerVisit(10,4,1,5):        w(10)(4)(1)(5)#2032  = 0
 CustomerVisit(11,4,1,5):        w(11)(4)(1)(5)#2212  = 0
 CustomerVisit(12,4,1,5):        w(12)(4)(1)(5)#2392  = 0
 CustomerVisit(3,5,1,5):         w(3)(5)(1)(5)#802  = 0
 CustomerVisit(4,5,1,5):         w(4)(5)(1)(5)#982  = 0
 CustomerVisit(5,5,1,5):         w(5)(5)(1)(5)#1162  = 0
 CustomerVisit(6,5,1,5):         w(6)(5)(1)(5)#1342  = 0
 CustomerVisit(7,5,1,5):         w(7)(5)(1)(5)#1522  = 0
 CustomerVisit(8,5,1,5):         w(8)(5)(1)(5)#1702  = 0
 CustomerVisit(9,5,1,5):         w(9)(5)(1)(5)#1882  = 0
 CustomerVisit(10,5,1,5):        w(10)(5)(1)(5)#2062  = 0
 CustomerVisit(11,5,1,5):        w(11)(5)(1)(5)#2242  = 0
 CustomerVisit(12,5,1,5):        w(12)(5)(1)(5)#2422  = 0
 CustomerVisit(3,6,1,5):         w(3)(6)(1)(5)#832  = 0
 CustomerVisit(4,6,1,5):         w(4)(6)(1)(5)#1012  = 0
 CustomerVisit(5,6,1,5):         w(5)(6)(1)(5)#1192  = 0
 CustomerVisit(6,6,1,5):         w(6)(6)(1)(5)#1372  = 0
 CustomerVisit(7,6,1,5):         w(7)(6)(1)(5)#1552  = 0
 CustomerVisit(8,6,1,5):         w(8)(6)(1)(5)#1732  = 0
 CustomerVisit(9,6,1,5):         w(9)(6)(1)(5)#1912  = 0
 CustomerVisit(10,6,1,5):        w(10)(6)(1)(5)#2092  = 0
 CustomerVisit(11,6,1,5):        w(11)(6)(1)(5)#2272  = 0
 CustomerVisit(12,6,1,5):        w(12)(6)(1)(5)#2452  = 0
 CustomerVisit(5,4,2,5):         w(5)(4)(2)(5)#1142  = 0
 CustomerVisit(6,4,2,5):         w(6)(4)(2)(5)#1322  = 0
 CustomerVisit(8,4,2,5):         w(8)(4)(2)(5)#1682  = 0
 CustomerVisit(9,4,2,5):         w(9)(4)(2)(5)#1862  = 0
 CustomerVisit(10,4,2,5):        w(10)(4)(2)(5)#2042  = 0
 CustomerVisit(11,4,2,5):        w(11)(4)(2)(5)#2222  = 0
 CustomerVisit(12,4,2,5):        w(12)(4)(2)(5)#2402  = 0
 CustomerVisit(3,5,2,5):         w(3)(5)(2)(5)#812  = 0
 CustomerVisit(4,5,2,5):         w(4)(5)(2)(5)#992  = 0
 CustomerVisit(5,5,2,5):         w(5)(5)(2)(5)#1172  = 0
 CustomerVisit(6,5,2,5):         w(6)(5)(2)(5)#1352  = 0
 CustomerVisit(7,5,2,5):         w(7)(5)(2)(5)#1532  = 0
 CustomerVisit(8,5,2,5):         w(8)(5)(2)(5)#1712  = 0
 CustomerVisit(9,5,2,5):         w(9)(5)(2)(5)#1892  = 0
 CustomerVisit(10,5,2,5):        w(10)(5)(2)(5)#2072  = 0
 CustomerVisit(11,5,2,5):        w(11)(5)(2)(5)#2252  = 0
 CustomerVisit(12,5,2,5):        w(12)(5)(2)(5)#2432  = 0
 CustomerVisit(3,6,2,5):         w(3)(6)(2)(5)#842  = 0
 CustomerVisit(4,6,2,5):         w(4)(6)(2)(5)#1022  = 0
 CustomerVisit(5,6,2,5):         w(5)(6)(2)(5)#1202  = 0
 CustomerVisit(6,6,2,5):         w(6)(6)(2)(5)#1382  = 0
 CustomerVisit(7,6,2,5):         w(7)(6)(2)(5)#1562  = 0
 CustomerVisit(8,6,2,5):         w(8)(6)(2)(5)#1742  = 0
 CustomerVisit(9,6,2,5):         w(9)(6)(2)(5)#1922  = 0
 CustomerVisit(10,6,2,5):        w(10)(6)(2)(5)#2102  = 0
 CustomerVisit(11,6,2,5):        w(11)(6)(2)(5)#2282  = 0
 CustomerVisit(12,6,2,5):        w(12)(6)(2)(5)#2462  = 0
 CustomerVisit(3,4,3,5):         w(3)(4)(3)(5)#792  = 0
 CustomerVisit(4,4,3,5):         w(4)(4)(3)(5)#972  = 0
 CustomerVisit(5,4,3,5):         w(5)(4)(3)(5)#1152  = 0
 CustomerVisit(6,4,3,5):         w(6)(4)(3)(5)#1332  = 0
 CustomerVisit(7,4,3,5):         w(7)(4)(3)(5)#1512  = 0
 CustomerVisit(8,4,3,5):         w(8)(4)(3)(5)#1692  = 0
 CustomerVisit(9,4,3,5):         w(9)(4)(3)(5)#1872  = 0
 CustomerVisit(10,4,3,5):        w(10)(4)(3)(5)#2052  = 0
 CustomerVisit(11,4,3,5):        w(11)(4)(3)(5)#2232  = 0
 CustomerVisit(12,4,3,5):        w(12)(4)(3)(5)#2412  = 0
 CustomerVisit(3,5,3,5):         w(3)(5)(3)(5)#822  = 0
 CustomerVisit(4,5,3,5):         w(4)(5)(3)(5)#1002  = 0
 CustomerVisit(5,5,3,5):         w(5)(5)(3)(5)#1182  = 0
 CustomerVisit(6,5,3,5):         w(6)(5)(3)(5)#1362  = 0
 CustomerVisit(7,5,3,5):         w(7)(5)(3)(5)#1542  = 0
 CustomerVisit(8,5,3,5):         w(8)(5)(3)(5)#1722  = 0
 CustomerVisit(9,5,3,5):         w(9)(5)(3)(5)#1902  = 0
 CustomerVisit(10,5,3,5):        w(10)(5)(3)(5)#2082  = 0
 CustomerVisit(11,5,3,5):        w(11)(5)(3)(5)#2262  = 0
 CustomerVisit(12,5,3,5):        w(12)(5)(3)(5)#2442  = 0
 CustomerVisit(3,6,3,5):         w(3)(6)(3)(5)#852  = 0
 CustomerVisit(4,6,3,5):         w(4)(6)(3)(5)#1032  = 0
 CustomerVisit(5,6,3,5):         w(5)(6)(3)(5)#1212  = 0
 CustomerVisit(6,6,3,5):         w(6)(6)(3)(5)#1392  = 0
 CustomerVisit(7,6,3,5):         w(7)(6)(3)(5)#1572  = 0
 CustomerVisit(8,6,3,5):         w(8)(6)(3)(5)#1752  = 0
 CustomerVisit(9,6,3,5):         w(9)(6)(3)(5)#1932  = 0
 CustomerVisit(10,6,3,5):        w(10)(6)(3)(5)#2112  = 0
 CustomerVisit(11,6,3,5):        w(11)(6)(3)(5)#2292  = 0
 CustomerVisit(12,6,3,5):        w(12)(6)(3)(5)#2472  = 0
 CustomerVisit(4,1,1,6):         w(4)(1)(1)(6)#863  = 0
 CustomerVisit(5,1,1,6):         w(5)(1)(1)(6)#1043  = 0
 CustomerVisit(7,1,1,6):         w(7)(1)(1)(6)#1403  = 0
 CustomerVisit(11,1,1,6):        w(11)(1)(1)(6)#2123  = 0
 CustomerVisit(12,1,1,6):        w(12)(1)(1)(6)#2303  = 0
 CustomerVisit(3,2,1,6):         w(3)(2)(1)(6)#713  = 0
 CustomerVisit(4,2,1,6):         w(4)(2)(1)(6)#893  = 0
 CustomerVisit(5,2,1,6):         w(5)(2)(1)(6)#1073  = 0
 CustomerVisit(6,2,1,6):         w(6)(2)(1)(6)#1253  = 0
 CustomerVisit(7,2,1,6):         w(7)(2)(1)(6)#1433  = 0
 CustomerVisit(8,2,1,6):         w(8)(2)(1)(6)#1613  = 0
 CustomerVisit(9,2,1,6):         w(9)(2)(1)(6)#1793  = 0
 CustomerVisit(10,2,1,6):        w(10)(2)(1)(6)#1973  = 0
 CustomerVisit(11,2,1,6):        w(11)(2)(1)(6)#2153  = 0
 CustomerVisit(12,2,1,6):        w(12)(2)(1)(6)#2333  = 0
 CustomerVisit(3,3,1,6):         w(3)(3)(1)(6)#743  = 0
 CustomerVisit(4,3,1,6):         w(4)(3)(1)(6)#923  = 0
 CustomerVisit(5,3,1,6):         w(5)(3)(1)(6)#1103  = 0
 CustomerVisit(6,3,1,6):         w(6)(3)(1)(6)#1283  = 0
 CustomerVisit(7,3,1,6):         w(7)(3)(1)(6)#1463  = 0
 CustomerVisit(8,3,1,6):         w(8)(3)(1)(6)#1643  = 0
 CustomerVisit(9,3,1,6):         w(9)(3)(1)(6)#1823  = 0
 CustomerVisit(10,3,1,6):        w(10)(3)(1)(6)#2003  = 0
 CustomerVisit(11,3,1,6):        w(11)(3)(1)(6)#2183  = 0
 CustomerVisit(12,3,1,6):        w(12)(3)(1)(6)#2363  = 0
 CustomerVisit(3,1,2,6):         w(3)(1)(2)(6)#693  = 0
 CustomerVisit(4,1,2,6):         w(4)(1)(2)(6)#873  = 0
 CustomerVisit(7,1,2,6):         w(7)(1)(2)(6)#1413  = 0
 CustomerVisit(8,1,2,6):         w(8)(1)(2)(6)#1593  = 0
 CustomerVisit(9,1,2,6):         w(9)(1)(2)(6)#1773  = 0
 CustomerVisit(12,1,2,6):        w(12)(1)(2)(6)#2313  = 0
 CustomerVisit(3,2,2,6):         w(3)(2)(2)(6)#723  = 0
 CustomerVisit(4,2,2,6):         w(4)(2)(2)(6)#903  = 0
 CustomerVisit(5,2,2,6):         w(5)(2)(2)(6)#1083  = 0
 CustomerVisit(6,2,2,6):         w(6)(2)(2)(6)#1263  = 0
 CustomerVisit(7,2,2,6):         w(7)(2)(2)(6)#1443  = 0
 CustomerVisit(8,2,2,6):         w(8)(2)(2)(6)#1623  = 0
 CustomerVisit(9,2,2,6):         w(9)(2)(2)(6)#1803  = 0
 CustomerVisit(10,2,2,6):        w(10)(2)(2)(6)#1983  = 0
 CustomerVisit(11,2,2,6):        w(11)(2)(2)(6)#2163  = 0
 CustomerVisit(12,2,2,6):        w(12)(2)(2)(6)#2343  = 0
 CustomerVisit(3,3,2,6):         w(3)(3)(2)(6)#753  = 0
 CustomerVisit(4,3,2,6):         w(4)(3)(2)(6)#933  = 0
 CustomerVisit(5,3,2,6):         w(5)(3)(2)(6)#1113  = 0
 CustomerVisit(6,3,2,6):         w(6)(3)(2)(6)#1293  = 0
 CustomerVisit(7,3,2,6):         w(7)(3)(2)(6)#1473  = 0
 CustomerVisit(8,3,2,6):         w(8)(3)(2)(6)#1653  = 0
 CustomerVisit(9,3,2,6):         w(9)(3)(2)(6)#1833  = 0
 CustomerVisit(10,3,2,6):        w(10)(3)(2)(6)#2013  = 0
 CustomerVisit(11,3,2,6):        w(11)(3)(2)(6)#2193  = 0
 CustomerVisit(12,3,2,6):        w(12)(3)(2)(6)#2373  = 0
 CustomerVisit(3,1,3,6):         w(3)(1)(3)(6)#703  = 0
 CustomerVisit(4,1,3,6):         w(4)(1)(3)(6)#883  = 0
 CustomerVisit(5,1,3,6):         w(5)(1)(3)(6)#1063  = 0
 CustomerVisit(7,1,3,6):         w(7)(1)(3)(6)#1423  = 0
 CustomerVisit(8,1,3,6):         w(8)(1)(3)(6)#1603  = 0
 CustomerVisit(9,1,3,6):         w(9)(1)(3)(6)#1783  = 0
 CustomerVisit(11,1,3,6):        w(11)(1)(3)(6)#2143  = 0
 CustomerVisit(3,2,3,6):         w(3)(2)(3)(6)#733  = 0
 CustomerVisit(4,2,3,6):         w(4)(2)(3)(6)#913  = 0
 CustomerVisit(5,2,3,6):         w(5)(2)(3)(6)#1093  = 0
 CustomerVisit(6,2,3,6):         w(6)(2)(3)(6)#1273  = 0
 CustomerVisit(7,2,3,6):         w(7)(2)(3)(6)#1453  = 0
 CustomerVisit(8,2,3,6):         w(8)(2)(3)(6)#1633  = 0
 CustomerVisit(9,2,3,6):         w(9)(2)(3)(6)#1813  = 0
 CustomerVisit(10,2,3,6):        w(10)(2)(3)(6)#1993  = 0
 CustomerVisit(11,2,3,6):        w(11)(2)(3)(6)#2173  = 0
 CustomerVisit(12,2,3,6):        w(12)(2)(3)(6)#2353  = 0
 CustomerVisit(3,3,3,6):         w(3)(3)(3)(6)#763  = 0
 CustomerVisit(4,3,3,6):         w(4)(3)(3)(6)#943  = 0
 CustomerVisit(6,3,3,6):         w(6)(3)(3)(6)#1303  = 0
 CustomerVisit(7,3,3,6):         w(7)(3)(3)(6)#1483  = 0
 CustomerVisit(8,3,3,6):         w(8)(3)(3)(6)#1663  = 0
 CustomerVisit(9,3,3,6):         w(9)(3)(3)(6)#1843  = 0
 CustomerVisit(10,3,3,6):        w(10)(3)(3)(6)#2023  = 0
 CustomerVisit(11,3,3,6):        w(11)(3)(3)(6)#2203  = 0
 CustomerVisit(12,3,3,6):        w(12)(3)(3)(6)#2383  = 0
 CustomerVisit(3,4,1,6):         w(3)(4)(1)(6)#773  = 0
 CustomerVisit(4,4,1,6):         w(4)(4)(1)(6)#953  = 0
 CustomerVisit(5,4,1,6):         w(5)(4)(1)(6)#1133  = 0
 CustomerVisit(6,4,1,6):         w(6)(4)(1)(6)#1313  = 0
 CustomerVisit(8,4,1,6):         w(8)(4)(1)(6)#1673  = 0
 CustomerVisit(9,4,1,6):         w(9)(4)(1)(6)#1853  = 0
 CustomerVisit(10,4,1,6):        w(10)(4)(1)(6)#2033  = 0
 CustomerVisit(11,4,1,6):        w(11)(4)(1)(6)#2213  = 0
 CustomerVisit(12,4,1,6):        w(12)(4)(1)(6)#2393  = 0
 CustomerVisit(3,5,1,6):         w(3)(5)(1)(6)#803  = 0
 CustomerVisit(4,5,1,6):         w(4)(5)(1)(6)#983  = 0
 CustomerVisit(5,5,1,6):         w(5)(5)(1)(6)#1163  = 0
 CustomerVisit(6,5,1,6):         w(6)(5)(1)(6)#1343  = 0
 CustomerVisit(7,5,1,6):         w(7)(5)(1)(6)#1523  = 0
 CustomerVisit(8,5,1,6):         w(8)(5)(1)(6)#1703  = 0
 CustomerVisit(9,5,1,6):         w(9)(5)(1)(6)#1883  = 0
 CustomerVisit(10,5,1,6):        w(10)(5)(1)(6)#2063  = 0
 CustomerVisit(11,5,1,6):        w(11)(5)(1)(6)#2243  = 0
 CustomerVisit(12,5,1,6):        w(12)(5)(1)(6)#2423  = 0
 CustomerVisit(3,6,1,6):         w(3)(6)(1)(6)#833  = 0
 CustomerVisit(4,6,1,6):         w(4)(6)(1)(6)#1013  = 0
 CustomerVisit(5,6,1,6):         w(5)(6)(1)(6)#1193  = 0
 CustomerVisit(6,6,1,6):         w(6)(6)(1)(6)#1373  = 0
 CustomerVisit(7,6,1,6):         w(7)(6)(1)(6)#1553  = 0
 CustomerVisit(8,6,1,6):         w(8)(6)(1)(6)#1733  = 0
 CustomerVisit(9,6,1,6):         w(9)(6)(1)(6)#1913  = 0
 CustomerVisit(10,6,1,6):        w(10)(6)(1)(6)#2093  = 0
 CustomerVisit(11,6,1,6):        w(11)(6)(1)(6)#2273  = 0
 CustomerVisit(12,6,1,6):        w(12)(6)(1)(6)#2453  = 0
 CustomerVisit(3,4,2,6):         w(3)(4)(2)(6)#783  = 0
 CustomerVisit(4,4,2,6):         w(4)(4)(2)(6)#963  = 0
 CustomerVisit(5,4,2,6):         w(5)(4)(2)(6)#1143  = 0
 CustomerVisit(6,4,2,6):         w(6)(4)(2)(6)#1323  = 0
 CustomerVisit(7,4,2,6):         w(7)(4)(2)(6)#1503  = 0
 CustomerVisit(8,4,2,6):         w(8)(4)(2)(6)#1683  = 0
 CustomerVisit(9,4,2,6):         w(9)(4)(2)(6)#1863  = 0
 CustomerVisit(10,4,2,6):        w(10)(4)(2)(6)#2043  = 0
 CustomerVisit(11,4,2,6):        w(11)(4)(2)(6)#2223  = 0
 CustomerVisit(12,4,2,6):        w(12)(4)(2)(6)#2403  = 0
 CustomerVisit(3,5,2,6):         w(3)(5)(2)(6)#813  = 0
 CustomerVisit(4,5,2,6):         w(4)(5)(2)(6)#993  = 0
 CustomerVisit(5,5,2,6):         w(5)(5)(2)(6)#1173  = 0
 CustomerVisit(6,5,2,6):         w(6)(5)(2)(6)#1353  = 0
 CustomerVisit(7,5,2,6):         w(7)(5)(2)(6)#1533  = 0
 CustomerVisit(8,5,2,6):         w(8)(5)(2)(6)#1713  = 0
 CustomerVisit(9,5,2,6):         w(9)(5)(2)(6)#1893  = 0
 CustomerVisit(10,5,2,6):        w(10)(5)(2)(6)#2073  = 0
 CustomerVisit(11,5,2,6):        w(11)(5)(2)(6)#2253  = 0
 CustomerVisit(12,5,2,6):        w(12)(5)(2)(6)#2433  = 0
 CustomerVisit(5,6,2,6):         w(5)(6)(2)(6)#1203  = 0
 CustomerVisit(6,6,2,6):         w(6)(6)(2)(6)#1383  = 0
 CustomerVisit(8,6,2,6):         w(8)(6)(2)(6)#1743  = 0
 CustomerVisit(9,6,2,6):         w(9)(6)(2)(6)#1923  = 0
 CustomerVisit(10,6,2,6):        w(10)(6)(2)(6)#2103  = 0
 CustomerVisit(11,6,2,6):        w(11)(6)(2)(6)#2283  = 0
 CustomerVisit(12,6,2,6):        w(12)(6)(2)(6)#2463  = 0
 CustomerVisit(5,4,3,6):         w(5)(4)(3)(6)#1153  = 0
 CustomerVisit(6,4,3,6):         w(6)(4)(3)(6)#1333  = 0
 CustomerVisit(8,4,3,6):         w(8)(4)(3)(6)#1693  = 0
 CustomerVisit(9,4,3,6):         w(9)(4)(3)(6)#1873  = 0
 CustomerVisit(10,4,3,6):        w(10)(4)(3)(6)#2053  = 0
 CustomerVisit(11,4,3,6):        w(11)(4)(3)(6)#2233  = 0
 CustomerVisit(12,4,3,6):        w(12)(4)(3)(6)#2413  = 0
 CustomerVisit(3,5,3,6):         w(3)(5)(3)(6)#823  = 0
 CustomerVisit(4,5,3,6):         w(4)(5)(3)(6)#1003  = 0
 CustomerVisit(5,5,3,6):         w(5)(5)(3)(6)#1183  = 0
 CustomerVisit(6,5,3,6):         w(6)(5)(3)(6)#1363  = 0
 CustomerVisit(7,5,3,6):         w(7)(5)(3)(6)#1543  = 0
 CustomerVisit(8,5,3,6):         w(8)(5)(3)(6)#1723  = 0
 CustomerVisit(9,5,3,6):         w(9)(5)(3)(6)#1903  = 0
 CustomerVisit(10,5,3,6):        w(10)(5)(3)(6)#2083  = 0
 CustomerVisit(11,5,3,6):        w(11)(5)(3)(6)#2263  = 0
 CustomerVisit(12,5,3,6):        w(12)(5)(3)(6)#2443  = 0
 CustomerVisit(3,6,3,6):         w(3)(6)(3)(6)#853  = 0
 CustomerVisit(4,6,3,6):         w(4)(6)(3)(6)#1033  = 0
 CustomerVisit(5,6,3,6):         w(5)(6)(3)(6)#1213  = 0
 CustomerVisit(6,6,3,6):         w(6)(6)(3)(6)#1393  = 0
 CustomerVisit(7,6,3,6):         w(7)(6)(3)(6)#1573  = 0
 CustomerVisit(8,6,3,6):         w(8)(6)(3)(6)#1753  = 0
 CustomerVisit(9,6,3,6):         w(9)(6)(3)(6)#1933  = 0
 CustomerVisit(10,6,3,6):        w(10)(6)(3)(6)#2113  = 0
 CustomerVisit(11,6,3,6):        w(11)(6)(3)(6)#2293  = 0
 CustomerVisit(12,6,3,6):        w(12)(6)(3)(6)#2473  = 0
 CustomerVisit(4,1,1,7):         w(4)(1)(1)(7)#864  = 0
 CustomerVisit(5,1,1,7):         w(5)(1)(1)(7)#1044  = 0
 CustomerVisit(7,1,1,7):         w(7)(1)(1)(7)#1404  = 0
 CustomerVisit(11,1,1,7):        w(11)(1)(1)(7)#2124  = 0
 CustomerVisit(12,1,1,7):        w(12)(1)(1)(7)#2304  = 0
 CustomerVisit(3,2,1,7):         w(3)(2)(1)(7)#714  = 0
 CustomerVisit(4,2,1,7):         w(4)(2)(1)(7)#894  = 0
 CustomerVisit(5,2,1,7):         w(5)(2)(1)(7)#1074  = 0
 CustomerVisit(6,2,1,7):         w(6)(2)(1)(7)#1254  = 0
 CustomerVisit(7,2,1,7):         w(7)(2)(1)(7)#1434  = 0
 CustomerVisit(8,2,1,7):         w(8)(2)(1)(7)#1614  = 0
 CustomerVisit(9,2,1,7):         w(9)(2)(1)(7)#1794  = 0
 CustomerVisit(10,2,1,7):        w(10)(2)(1)(7)#1974  = 0
 CustomerVisit(11,2,1,7):        w(11)(2)(1)(7)#2154  = 0
 CustomerVisit(12,2,1,7):        w(12)(2)(1)(7)#2334  = 0
 CustomerVisit(3,3,1,7):         w(3)(3)(1)(7)#744  = 0
 CustomerVisit(4,3,1,7):         w(4)(3)(1)(7)#924  = 0
 CustomerVisit(5,3,1,7):         w(5)(3)(1)(7)#1104  = 0
 CustomerVisit(6,3,1,7):         w(6)(3)(1)(7)#1284  = 0
 CustomerVisit(7,3,1,7):         w(7)(3)(1)(7)#1464  = 0
 CustomerVisit(8,3,1,7):         w(8)(3)(1)(7)#1644  = 0
 CustomerVisit(9,3,1,7):         w(9)(3)(1)(7)#1824  = 0
 CustomerVisit(10,3,1,7):        w(10)(3)(1)(7)#2004  = 0
 CustomerVisit(11,3,1,7):        w(11)(3)(1)(7)#2184  = 0
 CustomerVisit(12,3,1,7):        w(12)(3)(1)(7)#2364  = 0
 CustomerVisit(3,1,2,7):         w(3)(1)(2)(7)#694  = 0
 CustomerVisit(4,1,2,7):         w(4)(1)(2)(7)#874  = 0
 CustomerVisit(6,1,2,7):         w(6)(1)(2)(7)#1234  = 0
 CustomerVisit(7,1,2,7):         w(7)(1)(2)(7)#1414  = 0
 CustomerVisit(8,1,2,7):         w(8)(1)(2)(7)#1594  = 0
 CustomerVisit(9,1,2,7):         w(9)(1)(2)(7)#1774  = 0
 CustomerVisit(10,1,2,7):        w(10)(1)(2)(7)#1954  = 0
 CustomerVisit(11,1,2,7):        w(11)(1)(2)(7)#2134  = 0
 CustomerVisit(12,1,2,7):        w(12)(1)(2)(7)#2314  = 0
 CustomerVisit(3,2,2,7):         w(3)(2)(2)(7)#724  = 0
 CustomerVisit(4,2,2,7):         w(4)(2)(2)(7)#904  = 0
 CustomerVisit(5,2,2,7):         w(5)(2)(2)(7)#1084  = 0
 CustomerVisit(7,2,2,7):         w(7)(2)(2)(7)#1444  = 0
 CustomerVisit(8,2,2,7):         w(8)(2)(2)(7)#1624  = 0
 CustomerVisit(9,2,2,7):         w(9)(2)(2)(7)#1804  = 0
 CustomerVisit(12,2,2,7):        w(12)(2)(2)(7)#2344  = 0
 CustomerVisit(3,3,2,7):         w(3)(3)(2)(7)#754  = 0
 CustomerVisit(4,3,2,7):         w(4)(3)(2)(7)#934  = 0
 CustomerVisit(5,3,2,7):         w(5)(3)(2)(7)#1114  = 0
 CustomerVisit(6,3,2,7):         w(6)(3)(2)(7)#1294  = 0
 CustomerVisit(7,3,2,7):         w(7)(3)(2)(7)#1474  = 0
 CustomerVisit(8,3,2,7):         w(8)(3)(2)(7)#1654  = 0
 CustomerVisit(9,3,2,7):         w(9)(3)(2)(7)#1834  = 0
 CustomerVisit(10,3,2,7):        w(10)(3)(2)(7)#2014  = 0
 CustomerVisit(11,3,2,7):        w(11)(3)(2)(7)#2194  = 0
 CustomerVisit(12,3,2,7):        w(12)(3)(2)(7)#2374  = 0
 CustomerVisit(3,1,3,7):         w(3)(1)(3)(7)#704  = 0
 CustomerVisit(4,1,3,7):         w(4)(1)(3)(7)#884  = 0
 CustomerVisit(5,1,3,7):         w(5)(1)(3)(7)#1064  = 0
 CustomerVisit(7,1,3,7):         w(7)(1)(3)(7)#1424  = 0
 CustomerVisit(8,1,3,7):         w(8)(1)(3)(7)#1604  = 0
 CustomerVisit(9,1,3,7):         w(9)(1)(3)(7)#1784  = 0
 CustomerVisit(11,1,3,7):        w(11)(1)(3)(7)#2144  = 0
 CustomerVisit(3,2,3,7):         w(3)(2)(3)(7)#734  = 0
 CustomerVisit(4,2,3,7):         w(4)(2)(3)(7)#914  = 0
 CustomerVisit(5,2,3,7):         w(5)(2)(3)(7)#1094  = 0
 CustomerVisit(6,2,3,7):         w(6)(2)(3)(7)#1274  = 0
 CustomerVisit(7,2,3,7):         w(7)(2)(3)(7)#1454  = 0
 CustomerVisit(8,2,3,7):         w(8)(2)(3)(7)#1634  = 0
 CustomerVisit(9,2,3,7):         w(9)(2)(3)(7)#1814  = 0
 CustomerVisit(10,2,3,7):        w(10)(2)(3)(7)#1994  = 0
 CustomerVisit(11,2,3,7):        w(11)(2)(3)(7)#2174  = 0
 CustomerVisit(12,2,3,7):        w(12)(2)(3)(7)#2354  = 0
 CustomerVisit(3,3,3,7):         w(3)(3)(3)(7)#764  = 0
 CustomerVisit(4,3,3,7):         w(4)(3)(3)(7)#944  = 0
 CustomerVisit(6,3,3,7):         w(6)(3)(3)(7)#1304  = 0
 CustomerVisit(7,3,3,7):         w(7)(3)(3)(7)#1484  = 0
 CustomerVisit(8,3,3,7):         w(8)(3)(3)(7)#1664  = 0
 CustomerVisit(9,3,3,7):         w(9)(3)(3)(7)#1844  = 0
 CustomerVisit(10,3,3,7):        w(10)(3)(3)(7)#2024  = 0
 CustomerVisit(11,3,3,7):        w(11)(3)(3)(7)#2204  = 0
 CustomerVisit(12,3,3,7):        w(12)(3)(3)(7)#2384  = 0
 CustomerVisit(3,4,1,7):         w(3)(4)(1)(7)#774  = 0
 CustomerVisit(4,4,1,7):         w(4)(4)(1)(7)#954  = 0
 CustomerVisit(5,4,1,7):         w(5)(4)(1)(7)#1134  = 0
 CustomerVisit(6,4,1,7):         w(6)(4)(1)(7)#1314  = 0
 CustomerVisit(7,4,1,7):         w(7)(4)(1)(7)#1494  = 0
 CustomerVisit(8,4,1,7):         w(8)(4)(1)(7)#1674  = 0
 CustomerVisit(9,4,1,7):         w(9)(4)(1)(7)#1854  = 0
 CustomerVisit(10,4,1,7):        w(10)(4)(1)(7)#2034  = 0
 CustomerVisit(11,4,1,7):        w(11)(4)(1)(7)#2214  = 0
 CustomerVisit(12,4,1,7):        w(12)(4)(1)(7)#2394  = 0
 CustomerVisit(3,5,1,7):         w(3)(5)(1)(7)#804  = 0
 CustomerVisit(4,5,1,7):         w(4)(5)(1)(7)#984  = 0
 CustomerVisit(5,5,1,7):         w(5)(5)(1)(7)#1164  = 0
 CustomerVisit(6,5,1,7):         w(6)(5)(1)(7)#1344  = 0
 CustomerVisit(7,5,1,7):         w(7)(5)(1)(7)#1524  = 0
 CustomerVisit(8,5,1,7):         w(8)(5)(1)(7)#1704  = 0
 CustomerVisit(9,5,1,7):         w(9)(5)(1)(7)#1884  = 0
 CustomerVisit(10,5,1,7):        w(10)(5)(1)(7)#2064  = 0
 CustomerVisit(11,5,1,7):        w(11)(5)(1)(7)#2244  = 0
 CustomerVisit(12,5,1,7):        w(12)(5)(1)(7)#2424  = 0
 CustomerVisit(3,6,1,7):         w(3)(6)(1)(7)#834  = 0
 CustomerVisit(4,6,1,7):         w(4)(6)(1)(7)#1014  = 0
 CustomerVisit(5,6,1,7):         w(5)(6)(1)(7)#1194  = 0
 CustomerVisit(6,6,1,7):         w(6)(6)(1)(7)#1374  = 0
 CustomerVisit(7,6,1,7):         w(7)(6)(1)(7)#1554  = 0
 CustomerVisit(8,6,1,7):         w(8)(6)(1)(7)#1734  = 0
 CustomerVisit(9,6,1,7):         w(9)(6)(1)(7)#1914  = 0
 CustomerVisit(10,6,1,7):        w(10)(6)(1)(7)#2094  = 0
 CustomerVisit(11,6,1,7):        w(11)(6)(1)(7)#2274  = 0
 CustomerVisit(12,6,1,7):        w(12)(6)(1)(7)#2454  = 0
 CustomerVisit(5,4,2,7):         w(5)(4)(2)(7)#1144  = 0
 CustomerVisit(6,4,2,7):         w(6)(4)(2)(7)#1324  = 0
 CustomerVisit(8,4,2,7):         w(8)(4)(2)(7)#1684  = 0
 CustomerVisit(9,4,2,7):         w(9)(4)(2)(7)#1864  = 0
 CustomerVisit(10,4,2,7):        w(10)(4)(2)(7)#2044  = 0
 CustomerVisit(11,4,2,7):        w(11)(4)(2)(7)#2224  = 0
 CustomerVisit(12,4,2,7):        w(12)(4)(2)(7)#2404  = 0
 CustomerVisit(3,5,2,7):         w(3)(5)(2)(7)#814  = 0
 CustomerVisit(4,5,2,7):         w(4)(5)(2)(7)#994  = 0
 CustomerVisit(5,5,2,7):         w(5)(5)(2)(7)#1174  = 0
 CustomerVisit(6,5,2,7):         w(6)(5)(2)(7)#1354  = 0
 CustomerVisit(7,5,2,7):         w(7)(5)(2)(7)#1534  = 0
 CustomerVisit(8,5,2,7):         w(8)(5)(2)(7)#1714  = 0
 CustomerVisit(9,5,2,7):         w(9)(5)(2)(7)#1894  = 0
 CustomerVisit(10,5,2,7):        w(10)(5)(2)(7)#2074  = 0
 CustomerVisit(11,5,2,7):        w(11)(5)(2)(7)#2254  = 0
 CustomerVisit(12,5,2,7):        w(12)(5)(2)(7)#2434  = 0
 CustomerVisit(3,6,2,7):         w(3)(6)(2)(7)#844  = 0
 CustomerVisit(4,6,2,7):         w(4)(6)(2)(7)#1024  = 0
 CustomerVisit(5,6,2,7):         w(5)(6)(2)(7)#1204  = 0
 CustomerVisit(6,6,2,7):         w(6)(6)(2)(7)#1384  = 0
 CustomerVisit(7,6,2,7):         w(7)(6)(2)(7)#1564  = 0
 CustomerVisit(8,6,2,7):         w(8)(6)(2)(7)#1744  = 0
 CustomerVisit(9,6,2,7):         w(9)(6)(2)(7)#1924  = 0
 CustomerVisit(10,6,2,7):        w(10)(6)(2)(7)#2104  = 0
 CustomerVisit(11,6,2,7):        w(11)(6)(2)(7)#2284  = 0
 CustomerVisit(12,6,2,7):        w(12)(6)(2)(7)#2464  = 0
 CustomerVisit(5,4,3,7):         w(5)(4)(3)(7)#1154  = 0
 CustomerVisit(6,4,3,7):         w(6)(4)(3)(7)#1334  = 0
 CustomerVisit(8,4,3,7):         w(8)(4)(3)(7)#1694  = 0
 CustomerVisit(9,4,3,7):         w(9)(4)(3)(7)#1874  = 0
 CustomerVisit(10,4,3,7):        w(10)(4)(3)(7)#2054  = 0
 CustomerVisit(11,4,3,7):        w(11)(4)(3)(7)#2234  = 0
 CustomerVisit(12,4,3,7):        w(12)(4)(3)(7)#2414  = 0
 CustomerVisit(3,5,3,7):         w(3)(5)(3)(7)#824  = 0
 CustomerVisit(4,5,3,7):         w(4)(5)(3)(7)#1004  = 0
 CustomerVisit(5,5,3,7):         w(5)(5)(3)(7)#1184  = 0
 CustomerVisit(6,5,3,7):         w(6)(5)(3)(7)#1364  = 0
 CustomerVisit(7,5,3,7):         w(7)(5)(3)(7)#1544  = 0
 CustomerVisit(8,5,3,7):         w(8)(5)(3)(7)#1724  = 0
 CustomerVisit(9,5,3,7):         w(9)(5)(3)(7)#1904  = 0
 CustomerVisit(10,5,3,7):        w(10)(5)(3)(7)#2084  = 0
 CustomerVisit(11,5,3,7):        w(11)(5)(3)(7)#2264  = 0
 CustomerVisit(12,5,3,7):        w(12)(5)(3)(7)#2444  = 0
 CustomerVisit(3,6,3,7):         w(3)(6)(3)(7)#854  = 0
 CustomerVisit(4,6,3,7):         w(4)(6)(3)(7)#1034  = 0
 CustomerVisit(5,6,3,7):         w(5)(6)(3)(7)#1214  = 0
 CustomerVisit(6,6,3,7):         w(6)(6)(3)(7)#1394  = 0
 CustomerVisit(7,6,3,7):         w(7)(6)(3)(7)#1574  = 0
 CustomerVisit(8,6,3,7):         w(8)(6)(3)(7)#1754  = 0
 CustomerVisit(9,6,3,7):         w(9)(6)(3)(7)#1934  = 0
 CustomerVisit(10,6,3,7):        w(10)(6)(3)(7)#2114  = 0
 CustomerVisit(11,6,3,7):        w(11)(6)(3)(7)#2294  = 0
 CustomerVisit(12,6,3,7):        w(12)(6)(3)(7)#2474  = 0
 CustomerVisit(4,1,1,8):         w(4)(1)(1)(8)#865  = 0
 CustomerVisit(7,1,1,8):         w(7)(1)(1)(8)#1405  = 0
 CustomerVisit(11,1,1,8):        w(11)(1)(1)(8)#2125  = 0
 CustomerVisit(12,1,1,8):        w(12)(1)(1)(8)#2305  = 0
 CustomerVisit(3,2,1,8):         w(3)(2)(1)(8)#715  = 0
 CustomerVisit(4,2,1,8):         w(4)(2)(1)(8)#895  = 0
 CustomerVisit(5,2,1,8):         w(5)(2)(1)(8)#1075  = 0
 CustomerVisit(6,2,1,8):         w(6)(2)(1)(8)#1255  = 0
 CustomerVisit(7,2,1,8):         w(7)(2)(1)(8)#1435  = 0
 CustomerVisit(8,2,1,8):         w(8)(2)(1)(8)#1615  = 0
 CustomerVisit(9,2,1,8):         w(9)(2)(1)(8)#1795  = 0
 CustomerVisit(10,2,1,8):        w(10)(2)(1)(8)#1975  = 0
 CustomerVisit(11,2,1,8):        w(11)(2)(1)(8)#2155  = 0
 CustomerVisit(12,2,1,8):        w(12)(2)(1)(8)#2335  = 0
 CustomerVisit(3,3,1,8):         w(3)(3)(1)(8)#745  = 0
 CustomerVisit(4,3,1,8):         w(4)(3)(1)(8)#925  = 0
 CustomerVisit(5,3,1,8):         w(5)(3)(1)(8)#1105  = 0
 CustomerVisit(6,3,1,8):         w(6)(3)(1)(8)#1285  = 0
 CustomerVisit(7,3,1,8):         w(7)(3)(1)(8)#1465  = 0
 CustomerVisit(8,3,1,8):         w(8)(3)(1)(8)#1645  = 0
 CustomerVisit(9,3,1,8):         w(9)(3)(1)(8)#1825  = 0
 CustomerVisit(10,3,1,8):        w(10)(3)(1)(8)#2005  = 0
 CustomerVisit(11,3,1,8):        w(11)(3)(1)(8)#2185  = 0
 CustomerVisit(12,3,1,8):        w(12)(3)(1)(8)#2365  = 0
 CustomerVisit(3,1,2,8):         w(3)(1)(2)(8)#695  = 0
 CustomerVisit(4,1,2,8):         w(4)(1)(2)(8)#875  = 0
 CustomerVisit(5,1,2,8):         w(5)(1)(2)(8)#1055  = 0
 CustomerVisit(6,1,2,8):         w(6)(1)(2)(8)#1235  = 0
 CustomerVisit(7,1,2,8):         w(7)(1)(2)(8)#1415  = 0
 CustomerVisit(8,1,2,8):         w(8)(1)(2)(8)#1595  = 0
 CustomerVisit(9,1,2,8):         w(9)(1)(2)(8)#1775  = 0
 CustomerVisit(10,1,2,8):        w(10)(1)(2)(8)#1955  = 0
 CustomerVisit(11,1,2,8):        w(11)(1)(2)(8)#2135  = 0
 CustomerVisit(12,1,2,8):        w(12)(1)(2)(8)#2315  = 0
 CustomerVisit(3,2,2,8):         w(3)(2)(2)(8)#725  = 0
 CustomerVisit(4,2,2,8):         w(4)(2)(2)(8)#905  = 0
 CustomerVisit(5,2,2,8):         w(5)(2)(2)(8)#1085  = 0
 CustomerVisit(7,2,2,8):         w(7)(2)(2)(8)#1445  = 0
 CustomerVisit(8,2,2,8):         w(8)(2)(2)(8)#1625  = 0
 CustomerVisit(9,2,2,8):         w(9)(2)(2)(8)#1805  = 0
 CustomerVisit(11,2,2,8):        w(11)(2)(2)(8)#2165  = 0
 CustomerVisit(3,3,2,8):         w(3)(3)(2)(8)#755  = 0
 CustomerVisit(4,3,2,8):         w(4)(3)(2)(8)#935  = 0
 CustomerVisit(5,3,2,8):         w(5)(3)(2)(8)#1115  = 0
 CustomerVisit(6,3,2,8):         w(6)(3)(2)(8)#1295  = 0
 CustomerVisit(7,3,2,8):         w(7)(3)(2)(8)#1475  = 0
 CustomerVisit(8,3,2,8):         w(8)(3)(2)(8)#1655  = 0
 CustomerVisit(9,3,2,8):         w(9)(3)(2)(8)#1835  = 0
 CustomerVisit(10,3,2,8):        w(10)(3)(2)(8)#2015  = 0
 CustomerVisit(11,3,2,8):        w(11)(3)(2)(8)#2195  = 0
 CustomerVisit(12,3,2,8):        w(12)(3)(2)(8)#2375  = 0
 CustomerVisit(3,1,3,8):         w(3)(1)(3)(8)#705  = 0
 CustomerVisit(4,1,3,8):         w(4)(1)(3)(8)#885  = 0
 CustomerVisit(5,1,3,8):         w(5)(1)(3)(8)#1065  = 0
 CustomerVisit(7,1,3,8):         w(7)(1)(3)(8)#1425  = 0
 CustomerVisit(8,1,3,8):         w(8)(1)(3)(8)#1605  = 0
 CustomerVisit(9,1,3,8):         w(9)(1)(3)(8)#1785  = 0
 CustomerVisit(12,1,3,8):        w(12)(1)(3)(8)#2325  = 0
 CustomerVisit(3,2,3,8):         w(3)(2)(3)(8)#735  = 0
 CustomerVisit(4,2,3,8):         w(4)(2)(3)(8)#915  = 0
 CustomerVisit(5,2,3,8):         w(5)(2)(3)(8)#1095  = 0
 CustomerVisit(6,2,3,8):         w(6)(2)(3)(8)#1275  = 0
 CustomerVisit(7,2,3,8):         w(7)(2)(3)(8)#1455  = 0
 CustomerVisit(8,2,3,8):         w(8)(2)(3)(8)#1635  = 0
 CustomerVisit(9,2,3,8):         w(9)(2)(3)(8)#1815  = 0
 CustomerVisit(10,2,3,8):        w(10)(2)(3)(8)#1995  = 0
 CustomerVisit(11,2,3,8):        w(11)(2)(3)(8)#2175  = 0
 CustomerVisit(12,2,3,8):        w(12)(2)(3)(8)#2355  = 0
 CustomerVisit(3,3,3,8):         w(3)(3)(3)(8)#765  = 0
 CustomerVisit(4,3,3,8):         w(4)(3)(3)(8)#945  = 0
 CustomerVisit(6,3,3,8):         w(6)(3)(3)(8)#1305  = 0
 CustomerVisit(7,3,3,8):         w(7)(3)(3)(8)#1485  = 0
 CustomerVisit(8,3,3,8):         w(8)(3)(3)(8)#1665  = 0
 CustomerVisit(9,3,3,8):         w(9)(3)(3)(8)#1845  = 0
 CustomerVisit(10,3,3,8):        w(10)(3)(3)(8)#2025  = 0
 CustomerVisit(11,3,3,8):        w(11)(3)(3)(8)#2205  = 0
 CustomerVisit(12,3,3,8):        w(12)(3)(3)(8)#2385  = 0
 CustomerVisit(3,4,1,8):         w(3)(4)(1)(8)#775  = 0
 CustomerVisit(4,4,1,8):         w(4)(4)(1)(8)#955  = 0
 CustomerVisit(5,4,1,8):         w(5)(4)(1)(8)#1135  = 0
 CustomerVisit(6,4,1,8):         w(6)(4)(1)(8)#1315  = 0
 CustomerVisit(8,4,1,8):         w(8)(4)(1)(8)#1675  = 0
 CustomerVisit(9,4,1,8):         w(9)(4)(1)(8)#1855  = 0
 CustomerVisit(10,4,1,8):        w(10)(4)(1)(8)#2035  = 0
 CustomerVisit(11,4,1,8):        w(11)(4)(1)(8)#2215  = 0
 CustomerVisit(12,4,1,8):        w(12)(4)(1)(8)#2395  = 0
 CustomerVisit(3,5,1,8):         w(3)(5)(1)(8)#805  = 0
 CustomerVisit(4,5,1,8):         w(4)(5)(1)(8)#985  = 0
 CustomerVisit(5,5,1,8):         w(5)(5)(1)(8)#1165  = 0
 CustomerVisit(6,5,1,8):         w(6)(5)(1)(8)#1345  = 0
 CustomerVisit(7,5,1,8):         w(7)(5)(1)(8)#1525  = 0
 CustomerVisit(8,5,1,8):         w(8)(5)(1)(8)#1705  = 0
 CustomerVisit(9,5,1,8):         w(9)(5)(1)(8)#1885  = 0
 CustomerVisit(10,5,1,8):        w(10)(5)(1)(8)#2065  = 0
 CustomerVisit(11,5,1,8):        w(11)(5)(1)(8)#2245  = 0
 CustomerVisit(12,5,1,8):        w(12)(5)(1)(8)#2425  = 0
 CustomerVisit(3,6,1,8):         w(3)(6)(1)(8)#835  = 0
 CustomerVisit(4,6,1,8):         w(4)(6)(1)(8)#1015  = 0
 CustomerVisit(5,6,1,8):         w(5)(6)(1)(8)#1195  = 0
 CustomerVisit(6,6,1,8):         w(6)(6)(1)(8)#1375  = 0
 CustomerVisit(7,6,1,8):         w(7)(6)(1)(8)#1555  = 0
 CustomerVisit(8,6,1,8):         w(8)(6)(1)(8)#1735  = 0
 CustomerVisit(9,6,1,8):         w(9)(6)(1)(8)#1915  = 0
 CustomerVisit(10,6,1,8):        w(10)(6)(1)(8)#2095  = 0
 CustomerVisit(11,6,1,8):        w(11)(6)(1)(8)#2275  = 0
 CustomerVisit(12,6,1,8):        w(12)(6)(1)(8)#2455  = 0
 CustomerVisit(5,4,2,8):         w(5)(4)(2)(8)#1145  = 0
 CustomerVisit(6,4,2,8):         w(6)(4)(2)(8)#1325  = 0
 CustomerVisit(8,4,2,8):         w(8)(4)(2)(8)#1685  = 0
 CustomerVisit(9,4,2,8):         w(9)(4)(2)(8)#1865  = 0
 CustomerVisit(10,4,2,8):        w(10)(4)(2)(8)#2045  = 0
 CustomerVisit(11,4,2,8):        w(11)(4)(2)(8)#2225  = 0
 CustomerVisit(12,4,2,8):        w(12)(4)(2)(8)#2405  = 0
 CustomerVisit(3,5,2,8):         w(3)(5)(2)(8)#815  = 0
 CustomerVisit(4,5,2,8):         w(4)(5)(2)(8)#995  = 0
 CustomerVisit(5,5,2,8):         w(5)(5)(2)(8)#1175  = 0
 CustomerVisit(6,5,2,8):         w(6)(5)(2)(8)#1355  = 0
 CustomerVisit(7,5,2,8):         w(7)(5)(2)(8)#1535  = 0
 CustomerVisit(8,5,2,8):         w(8)(5)(2)(8)#1715  = 0
 CustomerVisit(9,5,2,8):         w(9)(5)(2)(8)#1895  = 0
 CustomerVisit(10,5,2,8):        w(10)(5)(2)(8)#2075  = 0
 CustomerVisit(11,5,2,8):        w(11)(5)(2)(8)#2255  = 0
 CustomerVisit(12,5,2,8):        w(12)(5)(2)(8)#2435  = 0
 CustomerVisit(3,6,2,8):         w(3)(6)(2)(8)#845  = 0
 CustomerVisit(4,6,2,8):         w(4)(6)(2)(8)#1025  = 0
 CustomerVisit(5,6,2,8):         w(5)(6)(2)(8)#1205  = 0
 CustomerVisit(6,6,2,8):         w(6)(6)(2)(8)#1385  = 0
 CustomerVisit(7,6,2,8):         w(7)(6)(2)(8)#1565  = 0
 CustomerVisit(8,6,2,8):         w(8)(6)(2)(8)#1745  = 0
 CustomerVisit(9,6,2,8):         w(9)(6)(2)(8)#1925  = 0
 CustomerVisit(10,6,2,8):        w(10)(6)(2)(8)#2105  = 0
 CustomerVisit(11,6,2,8):        w(11)(6)(2)(8)#2285  = 0
 CustomerVisit(12,6,2,8):        w(12)(6)(2)(8)#2465  = 0
 CustomerVisit(3,4,3,8):         w(3)(4)(3)(8)#795  = 0
 CustomerVisit(4,4,3,8):         w(4)(4)(3)(8)#975  = 0
 CustomerVisit(5,4,3,8):         w(5)(4)(3)(8)#1155  = 0
 CustomerVisit(6,4,3,8):         w(6)(4)(3)(8)#1335  = 0
 CustomerVisit(7,4,3,8):         w(7)(4)(3)(8)#1515  = 0
 CustomerVisit(8,4,3,8):         w(8)(4)(3)(8)#1695  = 0
 CustomerVisit(9,4,3,8):         w(9)(4)(3)(8)#1875  = 0
 CustomerVisit(10,4,3,8):        w(10)(4)(3)(8)#2055  = 0
 CustomerVisit(11,4,3,8):        w(11)(4)(3)(8)#2235  = 0
 CustomerVisit(12,4,3,8):        w(12)(4)(3)(8)#2415  = 0
 CustomerVisit(5,5,3,8):         w(5)(5)(3)(8)#1185  = 0
 CustomerVisit(6,5,3,8):         w(6)(5)(3)(8)#1365  = 0
 CustomerVisit(8,5,3,8):         w(8)(5)(3)(8)#1725  = 0
 CustomerVisit(9,5,3,8):         w(9)(5)(3)(8)#1905  = 0
 CustomerVisit(10,5,3,8):        w(10)(5)(3)(8)#2085  = 0
 CustomerVisit(11,5,3,8):        w(11)(5)(3)(8)#2265  = 0
 CustomerVisit(12,5,3,8):        w(12)(5)(3)(8)#2445  = 0
 CustomerVisit(3,6,3,8):         w(3)(6)(3)(8)#855  = 0
 CustomerVisit(4,6,3,8):         w(4)(6)(3)(8)#1035  = 0
 CustomerVisit(5,6,3,8):         w(5)(6)(3)(8)#1215  = 0
 CustomerVisit(6,6,3,8):         w(6)(6)(3)(8)#1395  = 0
 CustomerVisit(7,6,3,8):         w(7)(6)(3)(8)#1575  = 0
 CustomerVisit(8,6,3,8):         w(8)(6)(3)(8)#1755  = 0
 CustomerVisit(9,6,3,8):         w(9)(6)(3)(8)#1935  = 0
 CustomerVisit(10,6,3,8):        w(10)(6)(3)(8)#2115  = 0
 CustomerVisit(11,6,3,8):        w(11)(6)(3)(8)#2295  = 0
 CustomerVisit(12,6,3,8):        w(12)(6)(3)(8)#2475  = 0
 CustomerVisit(3,1,1,9):         w(3)(1)(1)(9)#686  = 0
 CustomerVisit(4,1,1,9):         w(4)(1)(1)(9)#866  = 0
 CustomerVisit(5,1,1,9):         w(5)(1)(1)(9)#1046  = 0
 CustomerVisit(6,1,1,9):         w(6)(1)(1)(9)#1226  = 0
 CustomerVisit(7,1,1,9):         w(7)(1)(1)(9)#1406  = 0
 CustomerVisit(8,1,1,9):         w(8)(1)(1)(9)#1586  = 0
 CustomerVisit(9,1,1,9):         w(9)(1)(1)(9)#1766  = 0
 CustomerVisit(10,1,1,9):        w(10)(1)(1)(9)#1946  = 0
 CustomerVisit(11,1,1,9):        w(11)(1)(1)(9)#2126  = 0
 CustomerVisit(12,1,1,9):        w(12)(1)(1)(9)#2306  = 0
 CustomerVisit(3,2,1,9):         w(3)(2)(1)(9)#716  = 0
 CustomerVisit(4,2,1,9):         w(4)(2)(1)(9)#896  = 0
 CustomerVisit(5,2,1,9):         w(5)(2)(1)(9)#1076  = 0
 CustomerVisit(7,2,1,9):         w(7)(2)(1)(9)#1436  = 0
 CustomerVisit(8,2,1,9):         w(8)(2)(1)(9)#1616  = 0
 CustomerVisit(9,2,1,9):         w(9)(2)(1)(9)#1796  = 0
 CustomerVisit(3,3,1,9):         w(3)(3)(1)(9)#746  = 0
 CustomerVisit(4,3,1,9):         w(4)(3)(1)(9)#926  = 0
 CustomerVisit(5,3,1,9):         w(5)(3)(1)(9)#1106  = 0
 CustomerVisit(6,3,1,9):         w(6)(3)(1)(9)#1286  = 0
 CustomerVisit(7,3,1,9):         w(7)(3)(1)(9)#1466  = 0
 CustomerVisit(8,3,1,9):         w(8)(3)(1)(9)#1646  = 0
 CustomerVisit(9,3,1,9):         w(9)(3)(1)(9)#1826  = 0
 CustomerVisit(10,3,1,9):        w(10)(3)(1)(9)#2006  = 0
 CustomerVisit(11,3,1,9):        w(11)(3)(1)(9)#2186  = 0
 CustomerVisit(12,3,1,9):        w(12)(3)(1)(9)#2366  = 0
 CustomerVisit(3,1,2,9):         w(3)(1)(2)(9)#696  = 0
 CustomerVisit(4,1,2,9):         w(4)(1)(2)(9)#876  = 0
 CustomerVisit(6,1,2,9):         w(6)(1)(2)(9)#1236  = 0
 CustomerVisit(7,1,2,9):         w(7)(1)(2)(9)#1416  = 0
 CustomerVisit(8,1,2,9):         w(8)(1)(2)(9)#1596  = 0
 CustomerVisit(9,1,2,9):         w(9)(1)(2)(9)#1776  = 0
 CustomerVisit(10,1,2,9):        w(10)(1)(2)(9)#1956  = 0
 CustomerVisit(11,1,2,9):        w(11)(1)(2)(9)#2136  = 0
 CustomerVisit(12,1,2,9):        w(12)(1)(2)(9)#2316  = 0
 CustomerVisit(3,2,2,9):         w(3)(2)(2)(9)#726  = 0
 CustomerVisit(4,2,2,9):         w(4)(2)(2)(9)#906  = 0
 CustomerVisit(5,2,2,9):         w(5)(2)(2)(9)#1086  = 0
 CustomerVisit(6,2,2,9):         w(6)(2)(2)(9)#1266  = 0
 CustomerVisit(7,2,2,9):         w(7)(2)(2)(9)#1446  = 0
 CustomerVisit(8,2,2,9):         w(8)(2)(2)(9)#1626  = 0
 CustomerVisit(9,2,2,9):         w(9)(2)(2)(9)#1806  = 0
 CustomerVisit(10,2,2,9):        w(10)(2)(2)(9)#1986  = 0
 CustomerVisit(11,2,2,9):        w(11)(2)(2)(9)#2166  = 0
 CustomerVisit(12,2,2,9):        w(12)(2)(2)(9)#2346  = 0
 CustomerVisit(3,3,2,9):         w(3)(3)(2)(9)#756  = 0
 CustomerVisit(4,3,2,9):         w(4)(3)(2)(9)#936  = 0
 CustomerVisit(5,3,2,9):         w(5)(3)(2)(9)#1116  = 0
 CustomerVisit(6,3,2,9):         w(6)(3)(2)(9)#1296  = 0
 CustomerVisit(7,3,2,9):         w(7)(3)(2)(9)#1476  = 0
 CustomerVisit(8,3,2,9):         w(8)(3)(2)(9)#1656  = 0
 CustomerVisit(9,3,2,9):         w(9)(3)(2)(9)#1836  = 0
 CustomerVisit(10,3,2,9):        w(10)(3)(2)(9)#2016  = 0
 CustomerVisit(11,3,2,9):        w(11)(3)(2)(9)#2196  = 0
 CustomerVisit(12,3,2,9):        w(12)(3)(2)(9)#2376  = 0
 CustomerVisit(4,1,3,9):         w(4)(1)(3)(9)#886  = 0
 CustomerVisit(5,1,3,9):         w(5)(1)(3)(9)#1066  = 0
 CustomerVisit(7,1,3,9):         w(7)(1)(3)(9)#1426  = 0
 CustomerVisit(8,1,3,9):         w(8)(1)(3)(9)#1606  = 0
 CustomerVisit(11,1,3,9):        w(11)(1)(3)(9)#2146  = 0
 CustomerVisit(3,2,3,9):         w(3)(2)(3)(9)#736  = 0
 CustomerVisit(4,2,3,9):         w(4)(2)(3)(9)#916  = 0
 CustomerVisit(5,2,3,9):         w(5)(2)(3)(9)#1096  = 0
 CustomerVisit(6,2,3,9):         w(6)(2)(3)(9)#1276  = 0
 CustomerVisit(7,2,3,9):         w(7)(2)(3)(9)#1456  = 0
 CustomerVisit(8,2,3,9):         w(8)(2)(3)(9)#1636  = 0
 CustomerVisit(9,2,3,9):         w(9)(2)(3)(9)#1816  = 0
 CustomerVisit(10,2,3,9):        w(10)(2)(3)(9)#1996  = 0
 CustomerVisit(11,2,3,9):        w(11)(2)(3)(9)#2176  = 0
 CustomerVisit(12,2,3,9):        w(12)(2)(3)(9)#2356  = 0
 CustomerVisit(3,3,3,9):         w(3)(3)(3)(9)#766  = 0
 CustomerVisit(4,3,3,9):         w(4)(3)(3)(9)#946  = 0
 CustomerVisit(6,3,3,9):         w(6)(3)(3)(9)#1306  = 0
 CustomerVisit(7,3,3,9):         w(7)(3)(3)(9)#1486  = 0
 CustomerVisit(8,3,3,9):         w(8)(3)(3)(9)#1666  = 0
 CustomerVisit(9,3,3,9):         w(9)(3)(3)(9)#1846  = 0
 CustomerVisit(10,3,3,9):        w(10)(3)(3)(9)#2026  = 0
 CustomerVisit(11,3,3,9):        w(11)(3)(3)(9)#2206  = 0
 CustomerVisit(12,3,3,9):        w(12)(3)(3)(9)#2386  = 0
 CustomerVisit(3,4,1,9):         w(3)(4)(1)(9)#776  = 0
 CustomerVisit(4,4,1,9):         w(4)(4)(1)(9)#956  = 0
 CustomerVisit(5,4,1,9):         w(5)(4)(1)(9)#1136  = 0
 CustomerVisit(6,4,1,9):         w(6)(4)(1)(9)#1316  = 0
 CustomerVisit(7,4,1,9):         w(7)(4)(1)(9)#1496  = 0
 CustomerVisit(8,4,1,9):         w(8)(4)(1)(9)#1676  = 0
 CustomerVisit(9,4,1,9):         w(9)(4)(1)(9)#1856  = 0
 CustomerVisit(10,4,1,9):        w(10)(4)(1)(9)#2036  = 0
 CustomerVisit(11,4,1,9):        w(11)(4)(1)(9)#2216  = 0
 CustomerVisit(12,4,1,9):        w(12)(4)(1)(9)#2396  = 0
 CustomerVisit(5,5,1,9):         w(5)(5)(1)(9)#1166  = 0
 CustomerVisit(6,5,1,9):         w(6)(5)(1)(9)#1346  = 0
 CustomerVisit(8,5,1,9):         w(8)(5)(1)(9)#1706  = 0
 CustomerVisit(9,5,1,9):         w(9)(5)(1)(9)#1886  = 0
 CustomerVisit(10,5,1,9):        w(10)(5)(1)(9)#2066  = 0
 CustomerVisit(11,5,1,9):        w(11)(5)(1)(9)#2246  = 0
 CustomerVisit(12,5,1,9):        w(12)(5)(1)(9)#2426  = 0
 CustomerVisit(3,6,1,9):         w(3)(6)(1)(9)#836  = 0
 CustomerVisit(4,6,1,9):         w(4)(6)(1)(9)#1016  = 0
 CustomerVisit(5,6,1,9):         w(5)(6)(1)(9)#1196  = 0
 CustomerVisit(6,6,1,9):         w(6)(6)(1)(9)#1376  = 0
 CustomerVisit(7,6,1,9):         w(7)(6)(1)(9)#1556  = 0
 CustomerVisit(8,6,1,9):         w(8)(6)(1)(9)#1736  = 0
 CustomerVisit(9,6,1,9):         w(9)(6)(1)(9)#1916  = 0
 CustomerVisit(10,6,1,9):        w(10)(6)(1)(9)#2096  = 0
 CustomerVisit(11,6,1,9):        w(11)(6)(1)(9)#2276  = 0
 CustomerVisit(12,6,1,9):        w(12)(6)(1)(9)#2456  = 0
 CustomerVisit(4,4,2,9):         w(4)(4)(2)(9)#966  = 0
 CustomerVisit(5,4,2,9):         w(5)(4)(2)(9)#1146  = 0
 CustomerVisit(9,4,2,9):         w(9)(4)(2)(9)#1866  = 0
 CustomerVisit(11,4,2,9):        w(11)(4)(2)(9)#2226  = 0
 CustomerVisit(12,4,2,9):        w(12)(4)(2)(9)#2406  = 0
 CustomerVisit(3,5,2,9):         w(3)(5)(2)(9)#816  = 0
 CustomerVisit(4,5,2,9):         w(4)(5)(2)(9)#996  = 0
 CustomerVisit(5,5,2,9):         w(5)(5)(2)(9)#1176  = 0
 CustomerVisit(6,5,2,9):         w(6)(5)(2)(9)#1356  = 0
 CustomerVisit(7,5,2,9):         w(7)(5)(2)(9)#1536  = 0
 CustomerVisit(8,5,2,9):         w(8)(5)(2)(9)#1716  = 0
 CustomerVisit(9,5,2,9):         w(9)(5)(2)(9)#1896  = 0
 CustomerVisit(10,5,2,9):        w(10)(5)(2)(9)#2076  = 0
 CustomerVisit(11,5,2,9):        w(11)(5)(2)(9)#2256  = 0
 CustomerVisit(12,5,2,9):        w(12)(5)(2)(9)#2436  = 0
 CustomerVisit(3,6,2,9):         w(3)(6)(2)(9)#846  = 0
 CustomerVisit(4,6,2,9):         w(4)(6)(2)(9)#1026  = 0
 CustomerVisit(5,6,2,9):         w(5)(6)(2)(9)#1206  = 0
 CustomerVisit(6,6,2,9):         w(6)(6)(2)(9)#1386  = 0
 CustomerVisit(7,6,2,9):         w(7)(6)(2)(9)#1566  = 0
 CustomerVisit(8,6,2,9):         w(8)(6)(2)(9)#1746  = 0
 CustomerVisit(9,6,2,9):         w(9)(6)(2)(9)#1926  = 0
 CustomerVisit(10,6,2,9):        w(10)(6)(2)(9)#2106  = 0
 CustomerVisit(11,6,2,9):        w(11)(6)(2)(9)#2286  = 0
 CustomerVisit(12,6,2,9):        w(12)(6)(2)(9)#2466  = 0
 CustomerVisit(3,4,3,9):         w(3)(4)(3)(9)#796  = 0
 CustomerVisit(5,4,3,9):         w(5)(4)(3)(9)#1156  = 0
 CustomerVisit(6,4,3,9):         w(6)(4)(3)(9)#1336  = 0
 CustomerVisit(7,4,3,9):         w(7)(4)(3)(9)#1516  = 0
 CustomerVisit(8,4,3,9):         w(8)(4)(3)(9)#1696  = 0
 CustomerVisit(9,4,3,9):         w(9)(4)(3)(9)#1876  = 0
 CustomerVisit(10,4,3,9):        w(10)(4)(3)(9)#2056  = 0
 CustomerVisit(11,4,3,9):        w(11)(4)(3)(9)#2236  = 0
 CustomerVisit(12,4,3,9):        w(12)(4)(3)(9)#2416  = 0
 CustomerVisit(3,5,3,9):         w(3)(5)(3)(9)#826  = 0
 CustomerVisit(4,5,3,9):         w(4)(5)(3)(9)#1006  = 0
 CustomerVisit(5,5,3,9):         w(5)(5)(3)(9)#1186  = 0
 CustomerVisit(6,5,3,9):         w(6)(5)(3)(9)#1366  = 0
 CustomerVisit(7,5,3,9):         w(7)(5)(3)(9)#1546  = 0
 CustomerVisit(8,5,3,9):         w(8)(5)(3)(9)#1726  = 0
 CustomerVisit(9,5,3,9):         w(9)(5)(3)(9)#1906  = 0
 CustomerVisit(10,5,3,9):        w(10)(5)(3)(9)#2086  = 0
 CustomerVisit(11,5,3,9):        w(11)(5)(3)(9)#2266  = 0
 CustomerVisit(12,5,3,9):        w(12)(5)(3)(9)#2446  = 0
 CustomerVisit(3,6,3,9):         w(3)(6)(3)(9)#856  = 0
 CustomerVisit(4,6,3,9):         w(4)(6)(3)(9)#1036  = 0
 CustomerVisit(5,6,3,9):         w(5)(6)(3)(9)#1216  = 0
 CustomerVisit(6,6,3,9):         w(6)(6)(3)(9)#1396  = 0
 CustomerVisit(7,6,3,9):         w(7)(6)(3)(9)#1576  = 0
 CustomerVisit(8,6,3,9):         w(8)(6)(3)(9)#1756  = 0
 CustomerVisit(9,6,3,9):         w(9)(6)(3)(9)#1936  = 0
 CustomerVisit(10,6,3,9):        w(10)(6)(3)(9)#2116  = 0
 CustomerVisit(11,6,3,9):        w(11)(6)(3)(9)#2296  = 0
 CustomerVisit(12,6,3,9):        w(12)(6)(3)(9)#2476  = 0
 CustomerVisit(3,1,1,10):        w(3)(1)(1)(10)#687  = 0
 CustomerVisit(4,1,1,10):        w(4)(1)(1)(10)#867  = 0
 CustomerVisit(5,1,1,10):        w(5)(1)(1)(10)#1047  = 0
 CustomerVisit(6,1,1,10):        w(6)(1)(1)(10)#1227  = 0
 CustomerVisit(7,1,1,10):        w(7)(1)(1)(10)#1407  = 0
 CustomerVisit(8,1,1,10):        w(8)(1)(1)(10)#1587  = 0
 CustomerVisit(9,1,1,10):        w(9)(1)(1)(10)#1767  = 0
 CustomerVisit(10,1,1,10):       w(10)(1)(1)(10)#1947  = 0
 CustomerVisit(11,1,1,10):       w(11)(1)(1)(10)#2127  = 0
 CustomerVisit(12,1,1,10):       w(12)(1)(1)(10)#2307  = 0
 CustomerVisit(3,2,1,10):        w(3)(2)(1)(10)#717  = 0
 CustomerVisit(4,2,1,10):        w(4)(2)(1)(10)#897  = 0
 CustomerVisit(5,2,1,10):        w(5)(2)(1)(10)#1077  = 0
 CustomerVisit(6,2,1,10):        w(6)(2)(1)(10)#1257  = 0
 CustomerVisit(7,2,1,10):        w(7)(2)(1)(10)#1437  = 0
 CustomerVisit(8,2,1,10):        w(8)(2)(1)(10)#1617  = 0
 CustomerVisit(9,2,1,10):        w(9)(2)(1)(10)#1797  = 0
 CustomerVisit(10,2,1,10):       w(10)(2)(1)(10)#1977  = 0
 CustomerVisit(11,2,1,10):       w(11)(2)(1)(10)#2157  = 0
 CustomerVisit(12,2,1,10):       w(12)(2)(1)(10)#2337  = 0
 CustomerVisit(4,3,1,10):        w(4)(3)(1)(10)#927  = 0
 CustomerVisit(5,3,1,10):        w(5)(3)(1)(10)#1107  = 0
 CustomerVisit(7,3,1,10):        w(7)(3)(1)(10)#1467  = 0
 CustomerVisit(11,3,1,10):       w(11)(3)(1)(10)#2187  = 0
 CustomerVisit(3,1,2,10):        w(3)(1)(2)(10)#697  = 0
 CustomerVisit(4,1,2,10):        w(4)(1)(2)(10)#877  = 0
 CustomerVisit(7,1,2,10):        w(7)(1)(2)(10)#1417  = 0
 CustomerVisit(8,1,2,10):        w(8)(1)(2)(10)#1597  = 0
 CustomerVisit(9,1,2,10):        w(9)(1)(2)(10)#1777  = 0
 CustomerVisit(11,1,2,10):       w(11)(1)(2)(10)#2137  = 0
 CustomerVisit(3,2,2,10):        w(3)(2)(2)(10)#727  = 0
 CustomerVisit(4,2,2,10):        w(4)(2)(2)(10)#907  = 0
 CustomerVisit(5,2,2,10):        w(5)(2)(2)(10)#1087  = 0
 CustomerVisit(6,2,2,10):        w(6)(2)(2)(10)#1267  = 0
 CustomerVisit(7,2,2,10):        w(7)(2)(2)(10)#1447  = 0
 CustomerVisit(8,2,2,10):        w(8)(2)(2)(10)#1627  = 0
 CustomerVisit(9,2,2,10):        w(9)(2)(2)(10)#1807  = 0
 CustomerVisit(10,2,2,10):       w(10)(2)(2)(10)#1987  = 0
 CustomerVisit(11,2,2,10):       w(11)(2)(2)(10)#2167  = 0
 CustomerVisit(12,2,2,10):       w(12)(2)(2)(10)#2347  = 0
 CustomerVisit(3,3,2,10):        w(3)(3)(2)(10)#757  = 0
 CustomerVisit(4,3,2,10):        w(4)(3)(2)(10)#937  = 0
 CustomerVisit(5,3,2,10):        w(5)(3)(2)(10)#1117  = 0
 CustomerVisit(6,3,2,10):        w(6)(3)(2)(10)#1297  = 0
 CustomerVisit(7,3,2,10):        w(7)(3)(2)(10)#1477  = 0
 CustomerVisit(8,3,2,10):        w(8)(3)(2)(10)#1657  = 0
 CustomerVisit(9,3,2,10):        w(9)(3)(2)(10)#1837  = 0
 CustomerVisit(10,3,2,10):       w(10)(3)(2)(10)#2017  = 0
 CustomerVisit(11,3,2,10):       w(11)(3)(2)(10)#2197  = 0
 CustomerVisit(12,3,2,10):       w(12)(3)(2)(10)#2377  = 0
 CustomerVisit(3,1,3,10):        w(3)(1)(3)(10)#707  = 0
 CustomerVisit(4,1,3,10):        w(4)(1)(3)(10)#887  = 0
 CustomerVisit(5,1,3,10):        w(5)(1)(3)(10)#1067  = 0
 CustomerVisit(6,1,3,10):        w(6)(1)(3)(10)#1247  = 0
 CustomerVisit(7,1,3,10):        w(7)(1)(3)(10)#1427  = 0
 CustomerVisit(8,1,3,10):        w(8)(1)(3)(10)#1607  = 0
 CustomerVisit(9,1,3,10):        w(9)(1)(3)(10)#1787  = 0
 CustomerVisit(10,1,3,10):       w(10)(1)(3)(10)#1967  = 0
 CustomerVisit(11,1,3,10):       w(11)(1)(3)(10)#2147  = 0
 CustomerVisit(12,1,3,10):       w(12)(1)(3)(10)#2327  = 0
 CustomerVisit(3,2,3,10):        w(3)(2)(3)(10)#737  = 0
 CustomerVisit(4,2,3,10):        w(4)(2)(3)(10)#917  = 0
 CustomerVisit(7,2,3,10):        w(7)(2)(3)(10)#1457  = 0
 CustomerVisit(8,2,3,10):        w(8)(2)(3)(10)#1637  = 0
 CustomerVisit(9,2,3,10):        w(9)(2)(3)(10)#1817  = 0
 CustomerVisit(12,2,3,10):       w(12)(2)(3)(10)#2357  = 0
 CustomerVisit(3,3,3,10):        w(3)(3)(3)(10)#767  = 0
 CustomerVisit(4,3,3,10):        w(4)(3)(3)(10)#947  = 0
 CustomerVisit(5,3,3,10):        w(5)(3)(3)(10)#1127  = 0
 CustomerVisit(6,3,3,10):        w(6)(3)(3)(10)#1307  = 0
 CustomerVisit(7,3,3,10):        w(7)(3)(3)(10)#1487  = 0
 CustomerVisit(8,3,3,10):        w(8)(3)(3)(10)#1667  = 0
 CustomerVisit(9,3,3,10):        w(9)(3)(3)(10)#1847  = 0
 CustomerVisit(10,3,3,10):       w(10)(3)(3)(10)#2027  = 0
 CustomerVisit(11,3,3,10):       w(11)(3)(3)(10)#2207  = 0
 CustomerVisit(12,3,3,10):       w(12)(3)(3)(10)#2387  = 0
 CustomerVisit(3,4,1,10):        w(3)(4)(1)(10)#777  = 0
 CustomerVisit(4,4,1,10):        w(4)(4)(1)(10)#957  = 0
 CustomerVisit(5,4,1,10):        w(5)(4)(1)(10)#1137  = 0
 CustomerVisit(6,4,1,10):        w(6)(4)(1)(10)#1317  = 0
 CustomerVisit(7,4,1,10):        w(7)(4)(1)(10)#1497  = 0
 CustomerVisit(8,4,1,10):        w(8)(4)(1)(10)#1677  = 0
 CustomerVisit(9,4,1,10):        w(9)(4)(1)(10)#1857  = 0
 CustomerVisit(10,4,1,10):       w(10)(4)(1)(10)#2037  = 0
 CustomerVisit(11,4,1,10):       w(11)(4)(1)(10)#2217  = 0
 CustomerVisit(12,4,1,10):       w(12)(4)(1)(10)#2397  = 0
 CustomerVisit(3,5,1,10):        w(3)(5)(1)(10)#807  = 0
 CustomerVisit(4,5,1,10):        w(4)(5)(1)(10)#987  = 0
 CustomerVisit(5,5,1,10):        w(5)(5)(1)(10)#1167  = 0
 CustomerVisit(6,5,1,10):        w(6)(5)(1)(10)#1347  = 0
 CustomerVisit(7,5,1,10):        w(7)(5)(1)(10)#1527  = 0
 CustomerVisit(8,5,1,10):        w(8)(5)(1)(10)#1707  = 0
 CustomerVisit(9,5,1,10):        w(9)(5)(1)(10)#1887  = 0
 CustomerVisit(10,5,1,10):       w(10)(5)(1)(10)#2067  = 0
 CustomerVisit(11,5,1,10):       w(11)(5)(1)(10)#2247  = 0
 CustomerVisit(12,5,1,10):       w(12)(5)(1)(10)#2427  = 0
 CustomerVisit(3,6,1,10):        w(3)(6)(1)(10)#837  = 0
 CustomerVisit(4,6,1,10):        w(4)(6)(1)(10)#1017  = 0
 CustomerVisit(5,6,1,10):        w(5)(6)(1)(10)#1197  = 0
 CustomerVisit(6,6,1,10):        w(6)(6)(1)(10)#1377  = 0
 CustomerVisit(8,6,1,10):        w(8)(6)(1)(10)#1737  = 0
 CustomerVisit(9,6,1,10):        w(9)(6)(1)(10)#1917  = 0
 CustomerVisit(10,6,1,10):       w(10)(6)(1)(10)#2097  = 0
 CustomerVisit(11,6,1,10):       w(11)(6)(1)(10)#2277  = 0
 CustomerVisit(12,6,1,10):       w(12)(6)(1)(10)#2457  = 0
 CustomerVisit(3,4,2,10):        w(3)(4)(2)(10)#787  = 0
 CustomerVisit(4,4,2,10):        w(4)(4)(2)(10)#967  = 0
 CustomerVisit(5,4,2,10):        w(5)(4)(2)(10)#1147  = 0
 CustomerVisit(6,4,2,10):        w(6)(4)(2)(10)#1327  = 0
 CustomerVisit(7,4,2,10):        w(7)(4)(2)(10)#1507  = 0
 CustomerVisit(8,4,2,10):        w(8)(4)(2)(10)#1687  = 0
 CustomerVisit(9,4,2,10):        w(9)(4)(2)(10)#1867  = 0
 CustomerVisit(10,4,2,10):       w(10)(4)(2)(10)#2047  = 0
 CustomerVisit(11,4,2,10):       w(11)(4)(2)(10)#2227  = 0
 CustomerVisit(12,4,2,10):       w(12)(4)(2)(10)#2407  = 0
 CustomerVisit(5,5,2,10):        w(5)(5)(2)(10)#1177  = 0
 CustomerVisit(6,5,2,10):        w(6)(5)(2)(10)#1357  = 0
 CustomerVisit(8,5,2,10):        w(8)(5)(2)(10)#1717  = 0
 CustomerVisit(9,5,2,10):        w(9)(5)(2)(10)#1897  = 0
 CustomerVisit(10,5,2,10):       w(10)(5)(2)(10)#2077  = 0
 CustomerVisit(11,5,2,10):       w(11)(5)(2)(10)#2257  = 0
 CustomerVisit(12,5,2,10):       w(12)(5)(2)(10)#2437  = 0
 CustomerVisit(3,6,2,10):        w(3)(6)(2)(10)#847  = 0
 CustomerVisit(4,6,2,10):        w(4)(6)(2)(10)#1027  = 0
 CustomerVisit(5,6,2,10):        w(5)(6)(2)(10)#1207  = 0
 CustomerVisit(6,6,2,10):        w(6)(6)(2)(10)#1387  = 0
 CustomerVisit(7,6,2,10):        w(7)(6)(2)(10)#1567  = 0
 CustomerVisit(8,6,2,10):        w(8)(6)(2)(10)#1747  = 0
 CustomerVisit(9,6,2,10):        w(9)(6)(2)(10)#1927  = 0
 CustomerVisit(10,6,2,10):       w(10)(6)(2)(10)#2107  = 0
 CustomerVisit(11,6,2,10):       w(11)(6)(2)(10)#2287  = 0
 CustomerVisit(12,6,2,10):       w(12)(6)(2)(10)#2467  = 0
 CustomerVisit(5,4,3,10):        w(5)(4)(3)(10)#1157  = 0
 CustomerVisit(6,4,3,10):        w(6)(4)(3)(10)#1337  = 0
 CustomerVisit(8,4,3,10):        w(8)(4)(3)(10)#1697  = 0
 CustomerVisit(9,4,3,10):        w(9)(4)(3)(10)#1877  = 0
 CustomerVisit(10,4,3,10):       w(10)(4)(3)(10)#2057  = 0
 CustomerVisit(11,4,3,10):       w(11)(4)(3)(10)#2237  = 0
 CustomerVisit(12,4,3,10):       w(12)(4)(3)(10)#2417  = 0
 CustomerVisit(3,5,3,10):        w(3)(5)(3)(10)#827  = 0
 CustomerVisit(4,5,3,10):        w(4)(5)(3)(10)#1007  = 0
 CustomerVisit(5,5,3,10):        w(5)(5)(3)(10)#1187  = 0
 CustomerVisit(6,5,3,10):        w(6)(5)(3)(10)#1367  = 0
 CustomerVisit(7,5,3,10):        w(7)(5)(3)(10)#1547  = 0
 CustomerVisit(8,5,3,10):        w(8)(5)(3)(10)#1727  = 0
 CustomerVisit(9,5,3,10):        w(9)(5)(3)(10)#1907  = 0
 CustomerVisit(10,5,3,10):       w(10)(5)(3)(10)#2087  = 0
 CustomerVisit(11,5,3,10):       w(11)(5)(3)(10)#2267  = 0
 CustomerVisit(12,5,3,10):       w(12)(5)(3)(10)#2447  = 0
 CustomerVisit(3,6,3,10):        w(3)(6)(3)(10)#857  = 0
 CustomerVisit(4,6,3,10):        w(4)(6)(3)(10)#1037  = 0
 CustomerVisit(5,6,3,10):        w(5)(6)(3)(10)#1217  = 0
 CustomerVisit(6,6,3,10):        w(6)(6)(3)(10)#1397  = 0
 CustomerVisit(7,6,3,10):        w(7)(6)(3)(10)#1577  = 0
 CustomerVisit(8,6,3,10):        w(8)(6)(3)(10)#1757  = 0
 CustomerVisit(9,6,3,10):        w(9)(6)(3)(10)#1937  = 0
 CustomerVisit(10,6,3,10):       w(10)(6)(3)(10)#2117  = 0
 CustomerVisit(11,6,3,10):       w(11)(6)(3)(10)#2297  = 0
 CustomerVisit(12,6,3,10):       w(12)(6)(3)(10)#2477  = 0
 VehicleCapacityWarehouse(1,1,1,1): w(3)(1)(1)(1)#678 + w(4)(1)(1)(1)#858
                                 + w(5)(1)(1)(1)#1038 + w(6)(1)(1)(1)#1218
                                 + w(7)(1)(1)(1)#1398 + w(8)(1)(1)(1)#1578
                                 + w(9)(1)(1)(1)#1758 + w(10)(1)(1)(1)#1938
                                 + w(11)(1)(1)(1)#2118 + w(12)(1)(1)(1)#2298
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,1): w(3)(2)(1)(1)#708 + w(4)(2)(1)(1)#888
                                 + w(5)(2)(1)(1)#1068 + w(6)(2)(1)(1)#1248
                                 + w(7)(2)(1)(1)#1428 + w(8)(2)(1)(1)#1608
                                 + w(9)(2)(1)(1)#1788 + w(10)(2)(1)(1)#1968
                                 + w(11)(2)(1)(1)#2148 + w(12)(2)(1)(1)#2328
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,1): w(3)(3)(1)(1)#738 + w(4)(3)(1)(1)#918
                                 + w(5)(3)(1)(1)#1098 + w(6)(3)(1)(1)#1278
                                 + w(7)(3)(1)(1)#1458 + w(8)(3)(1)(1)#1638
                                 + w(9)(3)(1)(1)#1818 + w(10)(3)(1)(1)#1998
                                 + w(11)(3)(1)(1)#2178 + w(12)(3)(1)(1)#2358
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,1): w(3)(4)(1)(1)#768 + w(4)(4)(1)(1)#948
                                 + w(5)(4)(1)(1)#1128 + w(6)(4)(1)(1)#1308
                                 + w(7)(4)(1)(1)#1488 + w(8)(4)(1)(1)#1668
                                 + w(9)(4)(1)(1)#1848 + w(10)(4)(1)(1)#2028
                                 + w(11)(4)(1)(1)#2208 + w(12)(4)(1)(1)#2388
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,1): w(3)(5)(1)(1)#798 + w(4)(5)(1)(1)#978
                                 + w(5)(5)(1)(1)#1158 + w(6)(5)(1)(1)#1338
                                 + w(7)(5)(1)(1)#1518 + w(8)(5)(1)(1)#1698
                                 + w(9)(5)(1)(1)#1878 + w(10)(5)(1)(1)#2058
                                 + w(11)(5)(1)(1)#2238 + w(12)(5)(1)(1)#2418
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,1): w(3)(6)(1)(1)#828 + w(4)(6)(1)(1)#1008
                                 + w(5)(6)(1)(1)#1188 + w(6)(6)(1)(1)#1368
                                 + w(7)(6)(1)(1)#1548 + w(8)(6)(1)(1)#1728
                                 + w(9)(6)(1)(1)#1908 + w(10)(6)(1)(1)#2088
                                 + w(11)(6)(1)(1)#2268 + w(12)(6)(1)(1)#2448
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,1): w(3)(1)(2)(1)#688 + w(4)(1)(2)(1)#868
                                 + w(5)(1)(2)(1)#1048 + w(6)(1)(2)(1)#1228
                                 + w(7)(1)(2)(1)#1408 + w(8)(1)(2)(1)#1588
                                 + w(9)(1)(2)(1)#1768 + w(10)(1)(2)(1)#1948
                                 + w(11)(1)(2)(1)#2128 + w(12)(1)(2)(1)#2308
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,1): w(3)(2)(2)(1)#718 + w(4)(2)(2)(1)#898
                                 + w(5)(2)(2)(1)#1078 + w(6)(2)(2)(1)#1258
                                 + w(7)(2)(2)(1)#1438 + w(8)(2)(2)(1)#1618
                                 + w(9)(2)(2)(1)#1798 + w(10)(2)(2)(1)#1978
                                 + w(11)(2)(2)(1)#2158 + w(12)(2)(2)(1)#2338
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,1): w(3)(3)(2)(1)#748 + w(4)(3)(2)(1)#928
                                 + w(5)(3)(2)(1)#1108 + w(6)(3)(2)(1)#1288
                                 + w(7)(3)(2)(1)#1468 + w(8)(3)(2)(1)#1648
                                 + w(9)(3)(2)(1)#1828 + w(10)(3)(2)(1)#2008
                                 + w(11)(3)(2)(1)#2188 + w(12)(3)(2)(1)#2368
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,1): w(3)(4)(2)(1)#778 + w(4)(4)(2)(1)#958
                                 + w(5)(4)(2)(1)#1138 + w(6)(4)(2)(1)#1318
                                 + w(7)(4)(2)(1)#1498 + w(8)(4)(2)(1)#1678
                                 + w(9)(4)(2)(1)#1858 + w(10)(4)(2)(1)#2038
                                 + w(11)(4)(2)(1)#2218 + w(12)(4)(2)(1)#2398
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,1): w(3)(5)(2)(1)#808 + w(4)(5)(2)(1)#988
                                 + w(5)(5)(2)(1)#1168 + w(6)(5)(2)(1)#1348
                                 + w(7)(5)(2)(1)#1528 + w(8)(5)(2)(1)#1708
                                 + w(9)(5)(2)(1)#1888 + w(10)(5)(2)(1)#2068
                                 + w(11)(5)(2)(1)#2248 + w(12)(5)(2)(1)#2428
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,1): w(3)(6)(2)(1)#838 + w(4)(6)(2)(1)#1018
                                 + w(5)(6)(2)(1)#1198 + w(6)(6)(2)(1)#1378
                                 + w(7)(6)(2)(1)#1558 + w(8)(6)(2)(1)#1738
                                 + w(9)(6)(2)(1)#1918 + w(10)(6)(2)(1)#2098
                                 + w(11)(6)(2)(1)#2278 + w(12)(6)(2)(1)#2458
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,1): w(3)(1)(3)(1)#698 + w(4)(1)(3)(1)#878
                                 + w(5)(1)(3)(1)#1058 + w(6)(1)(3)(1)#1238
                                 + w(7)(1)(3)(1)#1418 + w(8)(1)(3)(1)#1598
                                 + w(9)(1)(3)(1)#1778 + w(10)(1)(3)(1)#1958
                                 + w(11)(1)(3)(1)#2138 + w(12)(1)(3)(1)#2318
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,1): w(3)(2)(3)(1)#728 + w(4)(2)(3)(1)#908
                                 + w(5)(2)(3)(1)#1088 + w(6)(2)(3)(1)#1268
                                 + w(7)(2)(3)(1)#1448 + w(8)(2)(3)(1)#1628
                                 + w(9)(2)(3)(1)#1808 + w(10)(2)(3)(1)#1988
                                 + w(11)(2)(3)(1)#2168 + w(12)(2)(3)(1)#2348
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,1): w(3)(3)(3)(1)#758 + w(4)(3)(3)(1)#938
                                 + w(5)(3)(3)(1)#1118 + w(6)(3)(3)(1)#1298
                                 + w(7)(3)(3)(1)#1478 + w(8)(3)(3)(1)#1658
                                 + w(9)(3)(3)(1)#1838 + w(10)(3)(3)(1)#2018
                                 + w(11)(3)(3)(1)#2198 + w(12)(3)(3)(1)#2378
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,1): w(3)(4)(3)(1)#788 + w(4)(4)(3)(1)#968
                                 + w(5)(4)(3)(1)#1148 + w(6)(4)(3)(1)#1328
                                 + w(7)(4)(3)(1)#1508 + w(8)(4)(3)(1)#1688
                                 + w(9)(4)(3)(1)#1868 + w(10)(4)(3)(1)#2048
                                 + w(11)(4)(3)(1)#2228 + w(12)(4)(3)(1)#2408
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,1): w(3)(5)(3)(1)#818 + w(4)(5)(3)(1)#998
                                 + w(5)(5)(3)(1)#1178 + w(6)(5)(3)(1)#1358
                                 + w(7)(5)(3)(1)#1538 + w(8)(5)(3)(1)#1718
                                 + w(9)(5)(3)(1)#1898 + w(10)(5)(3)(1)#2078
                                 + w(11)(5)(3)(1)#2258 + w(12)(5)(3)(1)#2438
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,1): w(3)(6)(3)(1)#848 + w(4)(6)(3)(1)#1028
                                 + w(5)(6)(3)(1)#1208 + w(6)(6)(3)(1)#1388
                                 + w(7)(6)(3)(1)#1568 + w(8)(6)(3)(1)#1748
                                 + w(9)(6)(3)(1)#1928 + w(10)(6)(3)(1)#2108
                                 + w(11)(6)(3)(1)#2288 + w(12)(6)(3)(1)#2468
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,2): w(3)(1)(1)(2)#679 + w(4)(1)(1)(2)#859
                                 + w(5)(1)(1)(2)#1039 + w(6)(1)(1)(2)#1219
                                 + w(7)(1)(1)(2)#1399 + w(8)(1)(1)(2)#1579
                                 + w(9)(1)(1)(2)#1759 + w(10)(1)(1)(2)#1939
                                 + w(11)(1)(1)(2)#2119 + w(12)(1)(1)(2)#2299
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,2): w(3)(2)(1)(2)#709 + w(4)(2)(1)(2)#889
                                 + w(5)(2)(1)(2)#1069 + w(6)(2)(1)(2)#1249
                                 + w(7)(2)(1)(2)#1429 + w(8)(2)(1)(2)#1609
                                 + w(9)(2)(1)(2)#1789 + w(10)(2)(1)(2)#1969
                                 + w(11)(2)(1)(2)#2149 + w(12)(2)(1)(2)#2329
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,2): w(3)(3)(1)(2)#739 + w(4)(3)(1)(2)#919
                                 + w(5)(3)(1)(2)#1099 + w(6)(3)(1)(2)#1279
                                 + w(7)(3)(1)(2)#1459 + w(8)(3)(1)(2)#1639
                                 + w(9)(3)(1)(2)#1819 + w(10)(3)(1)(2)#1999
                                 + w(11)(3)(1)(2)#2179 + w(12)(3)(1)(2)#2359
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,2): w(3)(4)(1)(2)#769 + w(4)(4)(1)(2)#949
                                 + w(5)(4)(1)(2)#1129 + w(6)(4)(1)(2)#1309
                                 + w(7)(4)(1)(2)#1489 + w(8)(4)(1)(2)#1669
                                 + w(9)(4)(1)(2)#1849 + w(10)(4)(1)(2)#2029
                                 + w(11)(4)(1)(2)#2209 + w(12)(4)(1)(2)#2389
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,2): w(3)(5)(1)(2)#799 + w(4)(5)(1)(2)#979
                                 + w(5)(5)(1)(2)#1159 + w(6)(5)(1)(2)#1339
                                 + w(7)(5)(1)(2)#1519 + w(8)(5)(1)(2)#1699
                                 + w(9)(5)(1)(2)#1879 + w(10)(5)(1)(2)#2059
                                 + w(11)(5)(1)(2)#2239 + w(12)(5)(1)(2)#2419
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,2): w(3)(6)(1)(2)#829 + w(4)(6)(1)(2)#1009
                                 + w(5)(6)(1)(2)#1189 + w(6)(6)(1)(2)#1369
                                 + w(7)(6)(1)(2)#1549 + w(8)(6)(1)(2)#1729
                                 + w(9)(6)(1)(2)#1909 + w(10)(6)(1)(2)#2089
                                 + w(11)(6)(1)(2)#2269 + w(12)(6)(1)(2)#2449
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,2): w(3)(1)(2)(2)#689 + w(4)(1)(2)(2)#869
                                 + w(5)(1)(2)(2)#1049 + w(6)(1)(2)(2)#1229
                                 + w(7)(1)(2)(2)#1409 + w(8)(1)(2)(2)#1589
                                 + w(9)(1)(2)(2)#1769 + w(10)(1)(2)(2)#1949
                                 + w(11)(1)(2)(2)#2129 + w(12)(1)(2)(2)#2309
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,2): w(3)(2)(2)(2)#719 + w(4)(2)(2)(2)#899
                                 + w(5)(2)(2)(2)#1079 + w(6)(2)(2)(2)#1259
                                 + w(7)(2)(2)(2)#1439 + w(8)(2)(2)(2)#1619
                                 + w(9)(2)(2)(2)#1799 + w(10)(2)(2)(2)#1979
                                 + w(11)(2)(2)(2)#2159 + w(12)(2)(2)(2)#2339
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,2): w(3)(3)(2)(2)#749 + w(4)(3)(2)(2)#929
                                 + w(5)(3)(2)(2)#1109 + w(6)(3)(2)(2)#1289
                                 + w(7)(3)(2)(2)#1469 + w(8)(3)(2)(2)#1649
                                 + w(9)(3)(2)(2)#1829 + w(10)(3)(2)(2)#2009
                                 + w(11)(3)(2)(2)#2189 + w(12)(3)(2)(2)#2369
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,2): w(3)(4)(2)(2)#779 + w(4)(4)(2)(2)#959
                                 + w(5)(4)(2)(2)#1139 + w(6)(4)(2)(2)#1319
                                 + w(7)(4)(2)(2)#1499 + w(8)(4)(2)(2)#1679
                                 + w(9)(4)(2)(2)#1859 + w(10)(4)(2)(2)#2039
                                 + w(11)(4)(2)(2)#2219 + w(12)(4)(2)(2)#2399
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,2): w(3)(5)(2)(2)#809 + w(4)(5)(2)(2)#989
                                 + w(5)(5)(2)(2)#1169 + w(6)(5)(2)(2)#1349
                                 + w(7)(5)(2)(2)#1529 + w(8)(5)(2)(2)#1709
                                 + w(9)(5)(2)(2)#1889 + w(10)(5)(2)(2)#2069
                                 + w(11)(5)(2)(2)#2249 + w(12)(5)(2)(2)#2429
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,2): w(3)(6)(2)(2)#839 + w(4)(6)(2)(2)#1019
                                 + w(5)(6)(2)(2)#1199 + w(6)(6)(2)(2)#1379
                                 + w(7)(6)(2)(2)#1559 + w(8)(6)(2)(2)#1739
                                 + w(9)(6)(2)(2)#1919 + w(10)(6)(2)(2)#2099
                                 + w(11)(6)(2)(2)#2279 + w(12)(6)(2)(2)#2459
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,2): w(3)(1)(3)(2)#699 + w(4)(1)(3)(2)#879
                                 + w(5)(1)(3)(2)#1059 + w(6)(1)(3)(2)#1239
                                 + w(7)(1)(3)(2)#1419 + w(8)(1)(3)(2)#1599
                                 + w(9)(1)(3)(2)#1779 + w(10)(1)(3)(2)#1959
                                 + w(11)(1)(3)(2)#2139 + w(12)(1)(3)(2)#2319
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,2): w(3)(2)(3)(2)#729 + w(4)(2)(3)(2)#909
                                 + w(5)(2)(3)(2)#1089 + w(6)(2)(3)(2)#1269
                                 + w(7)(2)(3)(2)#1449 + w(8)(2)(3)(2)#1629
                                 + w(9)(2)(3)(2)#1809 + w(10)(2)(3)(2)#1989
                                 + w(11)(2)(3)(2)#2169 + w(12)(2)(3)(2)#2349
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,2): w(3)(3)(3)(2)#759 + w(4)(3)(3)(2)#939
                                 + w(5)(3)(3)(2)#1119 + w(6)(3)(3)(2)#1299
                                 + w(7)(3)(3)(2)#1479 + w(8)(3)(3)(2)#1659
                                 + w(9)(3)(3)(2)#1839 + w(10)(3)(3)(2)#2019
                                 + w(11)(3)(3)(2)#2199 + w(12)(3)(3)(2)#2379
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,2): w(3)(4)(3)(2)#789 + w(4)(4)(3)(2)#969
                                 + w(5)(4)(3)(2)#1149 + w(6)(4)(3)(2)#1329
                                 + w(7)(4)(3)(2)#1509 + w(8)(4)(3)(2)#1689
                                 + w(9)(4)(3)(2)#1869 + w(10)(4)(3)(2)#2049
                                 + w(11)(4)(3)(2)#2229 + w(12)(4)(3)(2)#2409
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,2): w(3)(5)(3)(2)#819 + w(4)(5)(3)(2)#999
                                 + w(5)(5)(3)(2)#1179 + w(6)(5)(3)(2)#1359
                                 + w(7)(5)(3)(2)#1539 + w(8)(5)(3)(2)#1719
                                 + w(9)(5)(3)(2)#1899 + w(10)(5)(3)(2)#2079
                                 + w(11)(5)(3)(2)#2259 + w(12)(5)(3)(2)#2439
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,2): w(3)(6)(3)(2)#849 + w(4)(6)(3)(2)#1029
                                 + w(5)(6)(3)(2)#1209 + w(6)(6)(3)(2)#1389
                                 + w(7)(6)(3)(2)#1569 + w(8)(6)(3)(2)#1749
                                 + w(9)(6)(3)(2)#1929 + w(10)(6)(3)(2)#2109
                                 + w(11)(6)(3)(2)#2289 + w(12)(6)(3)(2)#2469
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,3): w(3)(1)(1)(3)#680 + w(4)(1)(1)(3)#860
                                 + w(5)(1)(1)(3)#1040 + w(6)(1)(1)(3)#1220
                                 + w(7)(1)(1)(3)#1400 + w(8)(1)(1)(3)#1580
                                 + w(9)(1)(1)(3)#1760 + w(10)(1)(1)(3)#1940
                                 + w(11)(1)(1)(3)#2120 + w(12)(1)(1)(3)#2300
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,3): w(3)(2)(1)(3)#710 + w(4)(2)(1)(3)#890
                                 + w(5)(2)(1)(3)#1070 + w(6)(2)(1)(3)#1250
                                 + w(7)(2)(1)(3)#1430 + w(8)(2)(1)(3)#1610
                                 + w(9)(2)(1)(3)#1790 + w(10)(2)(1)(3)#1970
                                 + w(11)(2)(1)(3)#2150 + w(12)(2)(1)(3)#2330
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,3): w(3)(3)(1)(3)#740 + w(4)(3)(1)(3)#920
                                 + w(5)(3)(1)(3)#1100 + w(6)(3)(1)(3)#1280
                                 + w(7)(3)(1)(3)#1460 + w(8)(3)(1)(3)#1640
                                 + w(9)(3)(1)(3)#1820 + w(10)(3)(1)(3)#2000
                                 + w(11)(3)(1)(3)#2180 + w(12)(3)(1)(3)#2360
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,3): w(3)(4)(1)(3)#770 + w(4)(4)(1)(3)#950
                                 + w(5)(4)(1)(3)#1130 + w(6)(4)(1)(3)#1310
                                 + w(7)(4)(1)(3)#1490 + w(8)(4)(1)(3)#1670
                                 + w(9)(4)(1)(3)#1850 + w(10)(4)(1)(3)#2030
                                 + w(11)(4)(1)(3)#2210 + w(12)(4)(1)(3)#2390
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,3): w(3)(5)(1)(3)#800 + w(4)(5)(1)(3)#980
                                 + w(5)(5)(1)(3)#1160 + w(6)(5)(1)(3)#1340
                                 + w(7)(5)(1)(3)#1520 + w(8)(5)(1)(3)#1700
                                 + w(9)(5)(1)(3)#1880 + w(10)(5)(1)(3)#2060
                                 + w(11)(5)(1)(3)#2240 + w(12)(5)(1)(3)#2420
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,3): w(3)(6)(1)(3)#830 + w(4)(6)(1)(3)#1010
                                 + w(5)(6)(1)(3)#1190 + w(6)(6)(1)(3)#1370
                                 + w(7)(6)(1)(3)#1550 + w(8)(6)(1)(3)#1730
                                 + w(9)(6)(1)(3)#1910 + w(10)(6)(1)(3)#2090
                                 + w(11)(6)(1)(3)#2270 + w(12)(6)(1)(3)#2450
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,3): w(3)(1)(2)(3)#690 + w(4)(1)(2)(3)#870
                                 + w(5)(1)(2)(3)#1050 + w(6)(1)(2)(3)#1230
                                 + w(7)(1)(2)(3)#1410 + w(8)(1)(2)(3)#1590
                                 + w(9)(1)(2)(3)#1770 + w(10)(1)(2)(3)#1950
                                 + w(11)(1)(2)(3)#2130 + w(12)(1)(2)(3)#2310
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,3): w(3)(2)(2)(3)#720 + w(4)(2)(2)(3)#900
                                 + w(5)(2)(2)(3)#1080 + w(6)(2)(2)(3)#1260
                                 + w(7)(2)(2)(3)#1440 + w(8)(2)(2)(3)#1620
                                 + w(9)(2)(2)(3)#1800 + w(10)(2)(2)(3)#1980
                                 + w(11)(2)(2)(3)#2160 + w(12)(2)(2)(3)#2340
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,3): w(3)(3)(2)(3)#750 + w(4)(3)(2)(3)#930
                                 + w(5)(3)(2)(3)#1110 + w(6)(3)(2)(3)#1290
                                 + w(7)(3)(2)(3)#1470 + w(8)(3)(2)(3)#1650
                                 + w(9)(3)(2)(3)#1830 + w(10)(3)(2)(3)#2010
                                 + w(11)(3)(2)(3)#2190 + w(12)(3)(2)(3)#2370
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,3): w(3)(4)(2)(3)#780 + w(4)(4)(2)(3)#960
                                 + w(5)(4)(2)(3)#1140 + w(6)(4)(2)(3)#1320
                                 + w(7)(4)(2)(3)#1500 + w(8)(4)(2)(3)#1680
                                 + w(9)(4)(2)(3)#1860 + w(10)(4)(2)(3)#2040
                                 + w(11)(4)(2)(3)#2220 + w(12)(4)(2)(3)#2400
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,3): w(3)(5)(2)(3)#810 + w(4)(5)(2)(3)#990
                                 + w(5)(5)(2)(3)#1170 + w(6)(5)(2)(3)#1350
                                 + w(7)(5)(2)(3)#1530 + w(8)(5)(2)(3)#1710
                                 + w(9)(5)(2)(3)#1890 + w(10)(5)(2)(3)#2070
                                 + w(11)(5)(2)(3)#2250 + w(12)(5)(2)(3)#2430
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,3): w(3)(6)(2)(3)#840 + w(4)(6)(2)(3)#1020
                                 + w(5)(6)(2)(3)#1200 + w(6)(6)(2)(3)#1380
                                 + w(7)(6)(2)(3)#1560 + w(8)(6)(2)(3)#1740
                                 + w(9)(6)(2)(3)#1920 + w(10)(6)(2)(3)#2100
                                 + w(11)(6)(2)(3)#2280 + w(12)(6)(2)(3)#2460
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,3): w(3)(1)(3)(3)#700 + w(4)(1)(3)(3)#880
                                 + w(5)(1)(3)(3)#1060 + w(6)(1)(3)(3)#1240
                                 + w(7)(1)(3)(3)#1420 + w(8)(1)(3)(3)#1600
                                 + w(9)(1)(3)(3)#1780 + w(10)(1)(3)(3)#1960
                                 + w(11)(1)(3)(3)#2140 + w(12)(1)(3)(3)#2320
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,3): w(3)(2)(3)(3)#730 + w(4)(2)(3)(3)#910
                                 + w(5)(2)(3)(3)#1090 + w(6)(2)(3)(3)#1270
                                 + w(7)(2)(3)(3)#1450 + w(8)(2)(3)(3)#1630
                                 + w(9)(2)(3)(3)#1810 + w(10)(2)(3)(3)#1990
                                 + w(11)(2)(3)(3)#2170 + w(12)(2)(3)(3)#2350
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,3): w(3)(3)(3)(3)#760 + w(4)(3)(3)(3)#940
                                 + w(5)(3)(3)(3)#1120 + w(6)(3)(3)(3)#1300
                                 + w(7)(3)(3)(3)#1480 + w(8)(3)(3)(3)#1660
                                 + w(9)(3)(3)(3)#1840 + w(10)(3)(3)(3)#2020
                                 + w(11)(3)(3)(3)#2200 + w(12)(3)(3)(3)#2380
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,3): w(3)(4)(3)(3)#790 + w(4)(4)(3)(3)#970
                                 + w(5)(4)(3)(3)#1150 + w(6)(4)(3)(3)#1330
                                 + w(7)(4)(3)(3)#1510 + w(8)(4)(3)(3)#1690
                                 + w(9)(4)(3)(3)#1870 + w(10)(4)(3)(3)#2050
                                 + w(11)(4)(3)(3)#2230 + w(12)(4)(3)(3)#2410
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,3): w(3)(5)(3)(3)#820 + w(4)(5)(3)(3)#1000
                                 + w(5)(5)(3)(3)#1180 + w(6)(5)(3)(3)#1360
                                 + w(7)(5)(3)(3)#1540 + w(8)(5)(3)(3)#1720
                                 + w(9)(5)(3)(3)#1900 + w(10)(5)(3)(3)#2080
                                 + w(11)(5)(3)(3)#2260 + w(12)(5)(3)(3)#2440
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,3): w(3)(6)(3)(3)#850 + w(4)(6)(3)(3)#1030
                                 + w(5)(6)(3)(3)#1210 + w(6)(6)(3)(3)#1390
                                 + w(7)(6)(3)(3)#1570 + w(8)(6)(3)(3)#1750
                                 + w(9)(6)(3)(3)#1930 + w(10)(6)(3)(3)#2110
                                 + w(11)(6)(3)(3)#2290 + w(12)(6)(3)(3)#2470
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,4): w(3)(1)(1)(4)#681 + w(4)(1)(1)(4)#861
                                 + w(5)(1)(1)(4)#1041 + w(6)(1)(1)(4)#1221
                                 + w(7)(1)(1)(4)#1401 + w(8)(1)(1)(4)#1581
                                 + w(9)(1)(1)(4)#1761 + w(10)(1)(1)(4)#1941
                                 + w(11)(1)(1)(4)#2121 + w(12)(1)(1)(4)#2301
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,4): w(3)(2)(1)(4)#711 + w(4)(2)(1)(4)#891
                                 + w(5)(2)(1)(4)#1071 + w(6)(2)(1)(4)#1251
                                 + w(7)(2)(1)(4)#1431 + w(8)(2)(1)(4)#1611
                                 + w(9)(2)(1)(4)#1791 + w(10)(2)(1)(4)#1971
                                 + w(11)(2)(1)(4)#2151 + w(12)(2)(1)(4)#2331
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,4): w(3)(3)(1)(4)#741 + w(4)(3)(1)(4)#921
                                 + w(5)(3)(1)(4)#1101 + w(6)(3)(1)(4)#1281
                                 + w(7)(3)(1)(4)#1461 + w(8)(3)(1)(4)#1641
                                 + w(9)(3)(1)(4)#1821 + w(10)(3)(1)(4)#2001
                                 + w(11)(3)(1)(4)#2181 + w(12)(3)(1)(4)#2361
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,4): w(3)(4)(1)(4)#771 + w(4)(4)(1)(4)#951
                                 + w(5)(4)(1)(4)#1131 + w(6)(4)(1)(4)#1311
                                 + w(7)(4)(1)(4)#1491 + w(8)(4)(1)(4)#1671
                                 + w(9)(4)(1)(4)#1851 + w(10)(4)(1)(4)#2031
                                 + w(11)(4)(1)(4)#2211 + w(12)(4)(1)(4)#2391
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,4): w(3)(5)(1)(4)#801 + w(4)(5)(1)(4)#981
                                 + w(5)(5)(1)(4)#1161 + w(6)(5)(1)(4)#1341
                                 + w(7)(5)(1)(4)#1521 + w(8)(5)(1)(4)#1701
                                 + w(9)(5)(1)(4)#1881 + w(10)(5)(1)(4)#2061
                                 + w(11)(5)(1)(4)#2241 + w(12)(5)(1)(4)#2421
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,4): w(3)(6)(1)(4)#831 + w(4)(6)(1)(4)#1011
                                 + w(5)(6)(1)(4)#1191 + w(6)(6)(1)(4)#1371
                                 + w(7)(6)(1)(4)#1551 + w(8)(6)(1)(4)#1731
                                 + w(9)(6)(1)(4)#1911 + w(10)(6)(1)(4)#2091
                                 + w(11)(6)(1)(4)#2271 + w(12)(6)(1)(4)#2451
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,4): w(3)(1)(2)(4)#691 + w(4)(1)(2)(4)#871
                                 + w(5)(1)(2)(4)#1051 + w(6)(1)(2)(4)#1231
                                 + w(7)(1)(2)(4)#1411 + w(8)(1)(2)(4)#1591
                                 + w(9)(1)(2)(4)#1771 + w(10)(1)(2)(4)#1951
                                 + w(11)(1)(2)(4)#2131 + w(12)(1)(2)(4)#2311
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,4): w(3)(2)(2)(4)#721 + w(4)(2)(2)(4)#901
                                 + w(5)(2)(2)(4)#1081 + w(6)(2)(2)(4)#1261
                                 + w(7)(2)(2)(4)#1441 + w(8)(2)(2)(4)#1621
                                 + w(9)(2)(2)(4)#1801 + w(10)(2)(2)(4)#1981
                                 + w(11)(2)(2)(4)#2161 + w(12)(2)(2)(4)#2341
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,4): w(3)(3)(2)(4)#751 + w(4)(3)(2)(4)#931
                                 + w(5)(3)(2)(4)#1111 + w(6)(3)(2)(4)#1291
                                 + w(7)(3)(2)(4)#1471 + w(8)(3)(2)(4)#1651
                                 + w(9)(3)(2)(4)#1831 + w(10)(3)(2)(4)#2011
                                 + w(11)(3)(2)(4)#2191 + w(12)(3)(2)(4)#2371
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,4): w(3)(4)(2)(4)#781 + w(4)(4)(2)(4)#961
                                 + w(5)(4)(2)(4)#1141 + w(6)(4)(2)(4)#1321
                                 + w(7)(4)(2)(4)#1501 + w(8)(4)(2)(4)#1681
                                 + w(9)(4)(2)(4)#1861 + w(10)(4)(2)(4)#2041
                                 + w(11)(4)(2)(4)#2221 + w(12)(4)(2)(4)#2401
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,4): w(3)(5)(2)(4)#811 + w(4)(5)(2)(4)#991
                                 + w(5)(5)(2)(4)#1171 + w(6)(5)(2)(4)#1351
                                 + w(7)(5)(2)(4)#1531 + w(8)(5)(2)(4)#1711
                                 + w(9)(5)(2)(4)#1891 + w(10)(5)(2)(4)#2071
                                 + w(11)(5)(2)(4)#2251 + w(12)(5)(2)(4)#2431
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,4): w(3)(6)(2)(4)#841 + w(4)(6)(2)(4)#1021
                                 + w(5)(6)(2)(4)#1201 + w(6)(6)(2)(4)#1381
                                 + w(7)(6)(2)(4)#1561 + w(8)(6)(2)(4)#1741
                                 + w(9)(6)(2)(4)#1921 + w(10)(6)(2)(4)#2101
                                 + w(11)(6)(2)(4)#2281 + w(12)(6)(2)(4)#2461
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,4): w(3)(1)(3)(4)#701 + w(4)(1)(3)(4)#881
                                 + w(5)(1)(3)(4)#1061 + w(6)(1)(3)(4)#1241
                                 + w(7)(1)(3)(4)#1421 + w(8)(1)(3)(4)#1601
                                 + w(9)(1)(3)(4)#1781 + w(10)(1)(3)(4)#1961
                                 + w(11)(1)(3)(4)#2141 + w(12)(1)(3)(4)#2321
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,4): w(3)(2)(3)(4)#731 + w(4)(2)(3)(4)#911
                                 + w(5)(2)(3)(4)#1091 + w(6)(2)(3)(4)#1271
                                 + w(7)(2)(3)(4)#1451 + w(8)(2)(3)(4)#1631
                                 + w(9)(2)(3)(4)#1811 + w(10)(2)(3)(4)#1991
                                 + w(11)(2)(3)(4)#2171 + w(12)(2)(3)(4)#2351
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,4): w(3)(3)(3)(4)#761 + w(4)(3)(3)(4)#941
                                 + w(5)(3)(3)(4)#1121 + w(6)(3)(3)(4)#1301
                                 + w(7)(3)(3)(4)#1481 + w(8)(3)(3)(4)#1661
                                 + w(9)(3)(3)(4)#1841 + w(10)(3)(3)(4)#2021
                                 + w(11)(3)(3)(4)#2201 + w(12)(3)(3)(4)#2381
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,4): w(3)(4)(3)(4)#791 + w(4)(4)(3)(4)#971
                                 + w(5)(4)(3)(4)#1151 + w(6)(4)(3)(4)#1331
                                 + w(7)(4)(3)(4)#1511 + w(8)(4)(3)(4)#1691
                                 + w(9)(4)(3)(4)#1871 + w(10)(4)(3)(4)#2051
                                 + w(11)(4)(3)(4)#2231 + w(12)(4)(3)(4)#2411
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,4): w(3)(5)(3)(4)#821 + w(4)(5)(3)(4)#1001
                                 + w(5)(5)(3)(4)#1181 + w(6)(5)(3)(4)#1361
                                 + w(7)(5)(3)(4)#1541 + w(8)(5)(3)(4)#1721
                                 + w(9)(5)(3)(4)#1901 + w(10)(5)(3)(4)#2081
                                 + w(11)(5)(3)(4)#2261 + w(12)(5)(3)(4)#2441
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,4): w(3)(6)(3)(4)#851 + w(4)(6)(3)(4)#1031
                                 + w(5)(6)(3)(4)#1211 + w(6)(6)(3)(4)#1391
                                 + w(7)(6)(3)(4)#1571 + w(8)(6)(3)(4)#1751
                                 + w(9)(6)(3)(4)#1931 + w(10)(6)(3)(4)#2111
                                 + w(11)(6)(3)(4)#2291 + w(12)(6)(3)(4)#2471
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,5): w(3)(1)(1)(5)#682 + w(4)(1)(1)(5)#862
                                 + w(5)(1)(1)(5)#1042 + w(6)(1)(1)(5)#1222
                                 + w(7)(1)(1)(5)#1402 + w(8)(1)(1)(5)#1582
                                 + w(9)(1)(1)(5)#1762 + w(10)(1)(1)(5)#1942
                                 + w(11)(1)(1)(5)#2122 + w(12)(1)(1)(5)#2302
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,5): w(3)(2)(1)(5)#712 + w(4)(2)(1)(5)#892
                                 + w(5)(2)(1)(5)#1072 + w(6)(2)(1)(5)#1252
                                 + w(7)(2)(1)(5)#1432 + w(8)(2)(1)(5)#1612
                                 + w(9)(2)(1)(5)#1792 + w(10)(2)(1)(5)#1972
                                 + w(11)(2)(1)(5)#2152 + w(12)(2)(1)(5)#2332
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,5): w(3)(3)(1)(5)#742 + w(4)(3)(1)(5)#922
                                 + w(5)(3)(1)(5)#1102 + w(6)(3)(1)(5)#1282
                                 + w(7)(3)(1)(5)#1462 + w(8)(3)(1)(5)#1642
                                 + w(9)(3)(1)(5)#1822 + w(10)(3)(1)(5)#2002
                                 + w(11)(3)(1)(5)#2182 + w(12)(3)(1)(5)#2362
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,5): w(3)(4)(1)(5)#772 + w(4)(4)(1)(5)#952
                                 + w(5)(4)(1)(5)#1132 + w(6)(4)(1)(5)#1312
                                 + w(7)(4)(1)(5)#1492 + w(8)(4)(1)(5)#1672
                                 + w(9)(4)(1)(5)#1852 + w(10)(4)(1)(5)#2032
                                 + w(11)(4)(1)(5)#2212 + w(12)(4)(1)(5)#2392
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,5): w(3)(5)(1)(5)#802 + w(4)(5)(1)(5)#982
                                 + w(5)(5)(1)(5)#1162 + w(6)(5)(1)(5)#1342
                                 + w(7)(5)(1)(5)#1522 + w(8)(5)(1)(5)#1702
                                 + w(9)(5)(1)(5)#1882 + w(10)(5)(1)(5)#2062
                                 + w(11)(5)(1)(5)#2242 + w(12)(5)(1)(5)#2422
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,5): w(3)(6)(1)(5)#832 + w(4)(6)(1)(5)#1012
                                 + w(5)(6)(1)(5)#1192 + w(6)(6)(1)(5)#1372
                                 + w(7)(6)(1)(5)#1552 + w(8)(6)(1)(5)#1732
                                 + w(9)(6)(1)(5)#1912 + w(10)(6)(1)(5)#2092
                                 + w(11)(6)(1)(5)#2272 + w(12)(6)(1)(5)#2452
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,5): w(3)(1)(2)(5)#692 + w(4)(1)(2)(5)#872
                                 + w(5)(1)(2)(5)#1052 + w(6)(1)(2)(5)#1232
                                 + w(7)(1)(2)(5)#1412 + w(8)(1)(2)(5)#1592
                                 + w(9)(1)(2)(5)#1772 + w(10)(1)(2)(5)#1952
                                 + w(11)(1)(2)(5)#2132 + w(12)(1)(2)(5)#2312
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,5): w(3)(2)(2)(5)#722 + w(4)(2)(2)(5)#902
                                 + w(5)(2)(2)(5)#1082 + w(6)(2)(2)(5)#1262
                                 + w(7)(2)(2)(5)#1442 + w(8)(2)(2)(5)#1622
                                 + w(9)(2)(2)(5)#1802 + w(10)(2)(2)(5)#1982
                                 + w(11)(2)(2)(5)#2162 + w(12)(2)(2)(5)#2342
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,5): w(3)(3)(2)(5)#752 + w(4)(3)(2)(5)#932
                                 + w(5)(3)(2)(5)#1112 + w(6)(3)(2)(5)#1292
                                 + w(7)(3)(2)(5)#1472 + w(8)(3)(2)(5)#1652
                                 + w(9)(3)(2)(5)#1832 + w(10)(3)(2)(5)#2012
                                 + w(11)(3)(2)(5)#2192 + w(12)(3)(2)(5)#2372
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,5): w(3)(4)(2)(5)#782 + w(4)(4)(2)(5)#962
                                 + w(5)(4)(2)(5)#1142 + w(6)(4)(2)(5)#1322
                                 + w(7)(4)(2)(5)#1502 + w(8)(4)(2)(5)#1682
                                 + w(9)(4)(2)(5)#1862 + w(10)(4)(2)(5)#2042
                                 + w(11)(4)(2)(5)#2222 + w(12)(4)(2)(5)#2402
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,5): w(3)(5)(2)(5)#812 + w(4)(5)(2)(5)#992
                                 + w(5)(5)(2)(5)#1172 + w(6)(5)(2)(5)#1352
                                 + w(7)(5)(2)(5)#1532 + w(8)(5)(2)(5)#1712
                                 + w(9)(5)(2)(5)#1892 + w(10)(5)(2)(5)#2072
                                 + w(11)(5)(2)(5)#2252 + w(12)(5)(2)(5)#2432
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,5): w(3)(6)(2)(5)#842 + w(4)(6)(2)(5)#1022
                                 + w(5)(6)(2)(5)#1202 + w(6)(6)(2)(5)#1382
                                 + w(7)(6)(2)(5)#1562 + w(8)(6)(2)(5)#1742
                                 + w(9)(6)(2)(5)#1922 + w(10)(6)(2)(5)#2102
                                 + w(11)(6)(2)(5)#2282 + w(12)(6)(2)(5)#2462
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,5): w(3)(1)(3)(5)#702 + w(4)(1)(3)(5)#882
                                 + w(5)(1)(3)(5)#1062 + w(6)(1)(3)(5)#1242
                                 + w(7)(1)(3)(5)#1422 + w(8)(1)(3)(5)#1602
                                 + w(9)(1)(3)(5)#1782 + w(10)(1)(3)(5)#1962
                                 + w(11)(1)(3)(5)#2142 + w(12)(1)(3)(5)#2322
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,5): w(3)(2)(3)(5)#732 + w(4)(2)(3)(5)#912
                                 + w(5)(2)(3)(5)#1092 + w(6)(2)(3)(5)#1272
                                 + w(7)(2)(3)(5)#1452 + w(8)(2)(3)(5)#1632
                                 + w(9)(2)(3)(5)#1812 + w(10)(2)(3)(5)#1992
                                 + w(11)(2)(3)(5)#2172 + w(12)(2)(3)(5)#2352
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,5): w(3)(3)(3)(5)#762 + w(4)(3)(3)(5)#942
                                 + w(5)(3)(3)(5)#1122 + w(6)(3)(3)(5)#1302
                                 + w(7)(3)(3)(5)#1482 + w(8)(3)(3)(5)#1662
                                 + w(9)(3)(3)(5)#1842 + w(10)(3)(3)(5)#2022
                                 + w(11)(3)(3)(5)#2202 + w(12)(3)(3)(5)#2382
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,5): w(3)(4)(3)(5)#792 + w(4)(4)(3)(5)#972
                                 + w(5)(4)(3)(5)#1152 + w(6)(4)(3)(5)#1332
                                 + w(7)(4)(3)(5)#1512 + w(8)(4)(3)(5)#1692
                                 + w(9)(4)(3)(5)#1872 + w(10)(4)(3)(5)#2052
                                 + w(11)(4)(3)(5)#2232 + w(12)(4)(3)(5)#2412
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,5): w(3)(5)(3)(5)#822 + w(4)(5)(3)(5)#1002
                                 + w(5)(5)(3)(5)#1182 + w(6)(5)(3)(5)#1362
                                 + w(7)(5)(3)(5)#1542 + w(8)(5)(3)(5)#1722
                                 + w(9)(5)(3)(5)#1902 + w(10)(5)(3)(5)#2082
                                 + w(11)(5)(3)(5)#2262 + w(12)(5)(3)(5)#2442
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,5): w(3)(6)(3)(5)#852 + w(4)(6)(3)(5)#1032
                                 + w(5)(6)(3)(5)#1212 + w(6)(6)(3)(5)#1392
                                 + w(7)(6)(3)(5)#1572 + w(8)(6)(3)(5)#1752
                                 + w(9)(6)(3)(5)#1932 + w(10)(6)(3)(5)#2112
                                 + w(11)(6)(3)(5)#2292 + w(12)(6)(3)(5)#2472
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,6): w(3)(1)(1)(6)#683 + w(4)(1)(1)(6)#863
                                 + w(5)(1)(1)(6)#1043 + w(6)(1)(1)(6)#1223
                                 + w(7)(1)(1)(6)#1403 + w(8)(1)(1)(6)#1583
                                 + w(9)(1)(1)(6)#1763 + w(10)(1)(1)(6)#1943
                                 + w(11)(1)(1)(6)#2123 + w(12)(1)(1)(6)#2303
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,6): w(3)(2)(1)(6)#713 + w(4)(2)(1)(6)#893
                                 + w(5)(2)(1)(6)#1073 + w(6)(2)(1)(6)#1253
                                 + w(7)(2)(1)(6)#1433 + w(8)(2)(1)(6)#1613
                                 + w(9)(2)(1)(6)#1793 + w(10)(2)(1)(6)#1973
                                 + w(11)(2)(1)(6)#2153 + w(12)(2)(1)(6)#2333
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,6): w(3)(3)(1)(6)#743 + w(4)(3)(1)(6)#923
                                 + w(5)(3)(1)(6)#1103 + w(6)(3)(1)(6)#1283
                                 + w(7)(3)(1)(6)#1463 + w(8)(3)(1)(6)#1643
                                 + w(9)(3)(1)(6)#1823 + w(10)(3)(1)(6)#2003
                                 + w(11)(3)(1)(6)#2183 + w(12)(3)(1)(6)#2363
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,6): w(3)(4)(1)(6)#773 + w(4)(4)(1)(6)#953
                                 + w(5)(4)(1)(6)#1133 + w(6)(4)(1)(6)#1313
                                 + w(7)(4)(1)(6)#1493 + w(8)(4)(1)(6)#1673
                                 + w(9)(4)(1)(6)#1853 + w(10)(4)(1)(6)#2033
                                 + w(11)(4)(1)(6)#2213 + w(12)(4)(1)(6)#2393
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,6): w(3)(5)(1)(6)#803 + w(4)(5)(1)(6)#983
                                 + w(5)(5)(1)(6)#1163 + w(6)(5)(1)(6)#1343
                                 + w(7)(5)(1)(6)#1523 + w(8)(5)(1)(6)#1703
                                 + w(9)(5)(1)(6)#1883 + w(10)(5)(1)(6)#2063
                                 + w(11)(5)(1)(6)#2243 + w(12)(5)(1)(6)#2423
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,6): w(3)(6)(1)(6)#833 + w(4)(6)(1)(6)#1013
                                 + w(5)(6)(1)(6)#1193 + w(6)(6)(1)(6)#1373
                                 + w(7)(6)(1)(6)#1553 + w(8)(6)(1)(6)#1733
                                 + w(9)(6)(1)(6)#1913 + w(10)(6)(1)(6)#2093
                                 + w(11)(6)(1)(6)#2273 + w(12)(6)(1)(6)#2453
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,6): w(3)(1)(2)(6)#693 + w(4)(1)(2)(6)#873
                                 + w(5)(1)(2)(6)#1053 + w(6)(1)(2)(6)#1233
                                 + w(7)(1)(2)(6)#1413 + w(8)(1)(2)(6)#1593
                                 + w(9)(1)(2)(6)#1773 + w(10)(1)(2)(6)#1953
                                 + w(11)(1)(2)(6)#2133 + w(12)(1)(2)(6)#2313
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,6): w(3)(2)(2)(6)#723 + w(4)(2)(2)(6)#903
                                 + w(5)(2)(2)(6)#1083 + w(6)(2)(2)(6)#1263
                                 + w(7)(2)(2)(6)#1443 + w(8)(2)(2)(6)#1623
                                 + w(9)(2)(2)(6)#1803 + w(10)(2)(2)(6)#1983
                                 + w(11)(2)(2)(6)#2163 + w(12)(2)(2)(6)#2343
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,6): w(3)(3)(2)(6)#753 + w(4)(3)(2)(6)#933
                                 + w(5)(3)(2)(6)#1113 + w(6)(3)(2)(6)#1293
                                 + w(7)(3)(2)(6)#1473 + w(8)(3)(2)(6)#1653
                                 + w(9)(3)(2)(6)#1833 + w(10)(3)(2)(6)#2013
                                 + w(11)(3)(2)(6)#2193 + w(12)(3)(2)(6)#2373
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,6): w(3)(4)(2)(6)#783 + w(4)(4)(2)(6)#963
                                 + w(5)(4)(2)(6)#1143 + w(6)(4)(2)(6)#1323
                                 + w(7)(4)(2)(6)#1503 + w(8)(4)(2)(6)#1683
                                 + w(9)(4)(2)(6)#1863 + w(10)(4)(2)(6)#2043
                                 + w(11)(4)(2)(6)#2223 + w(12)(4)(2)(6)#2403
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,6): w(3)(5)(2)(6)#813 + w(4)(5)(2)(6)#993
                                 + w(5)(5)(2)(6)#1173 + w(6)(5)(2)(6)#1353
                                 + w(7)(5)(2)(6)#1533 + w(8)(5)(2)(6)#1713
                                 + w(9)(5)(2)(6)#1893 + w(10)(5)(2)(6)#2073
                                 + w(11)(5)(2)(6)#2253 + w(12)(5)(2)(6)#2433
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,6): w(3)(6)(2)(6)#843 + w(4)(6)(2)(6)#1023
                                 + w(5)(6)(2)(6)#1203 + w(6)(6)(2)(6)#1383
                                 + w(7)(6)(2)(6)#1563 + w(8)(6)(2)(6)#1743
                                 + w(9)(6)(2)(6)#1923 + w(10)(6)(2)(6)#2103
                                 + w(11)(6)(2)(6)#2283 + w(12)(6)(2)(6)#2463
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,6): w(3)(1)(3)(6)#703 + w(4)(1)(3)(6)#883
                                 + w(5)(1)(3)(6)#1063 + w(6)(1)(3)(6)#1243
                                 + w(7)(1)(3)(6)#1423 + w(8)(1)(3)(6)#1603
                                 + w(9)(1)(3)(6)#1783 + w(10)(1)(3)(6)#1963
                                 + w(11)(1)(3)(6)#2143 + w(12)(1)(3)(6)#2323
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,6): w(3)(2)(3)(6)#733 + w(4)(2)(3)(6)#913
                                 + w(5)(2)(3)(6)#1093 + w(6)(2)(3)(6)#1273
                                 + w(7)(2)(3)(6)#1453 + w(8)(2)(3)(6)#1633
                                 + w(9)(2)(3)(6)#1813 + w(10)(2)(3)(6)#1993
                                 + w(11)(2)(3)(6)#2173 + w(12)(2)(3)(6)#2353
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,6): w(3)(3)(3)(6)#763 + w(4)(3)(3)(6)#943
                                 + w(5)(3)(3)(6)#1123 + w(6)(3)(3)(6)#1303
                                 + w(7)(3)(3)(6)#1483 + w(8)(3)(3)(6)#1663
                                 + w(9)(3)(3)(6)#1843 + w(10)(3)(3)(6)#2023
                                 + w(11)(3)(3)(6)#2203 + w(12)(3)(3)(6)#2383
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,6): w(3)(4)(3)(6)#793 + w(4)(4)(3)(6)#973
                                 + w(5)(4)(3)(6)#1153 + w(6)(4)(3)(6)#1333
                                 + w(7)(4)(3)(6)#1513 + w(8)(4)(3)(6)#1693
                                 + w(9)(4)(3)(6)#1873 + w(10)(4)(3)(6)#2053
                                 + w(11)(4)(3)(6)#2233 + w(12)(4)(3)(6)#2413
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,6): w(3)(5)(3)(6)#823 + w(4)(5)(3)(6)#1003
                                 + w(5)(5)(3)(6)#1183 + w(6)(5)(3)(6)#1363
                                 + w(7)(5)(3)(6)#1543 + w(8)(5)(3)(6)#1723
                                 + w(9)(5)(3)(6)#1903 + w(10)(5)(3)(6)#2083
                                 + w(11)(5)(3)(6)#2263 + w(12)(5)(3)(6)#2443
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,6): w(3)(6)(3)(6)#853 + w(4)(6)(3)(6)#1033
                                 + w(5)(6)(3)(6)#1213 + w(6)(6)(3)(6)#1393
                                 + w(7)(6)(3)(6)#1573 + w(8)(6)(3)(6)#1753
                                 + w(9)(6)(3)(6)#1933 + w(10)(6)(3)(6)#2113
                                 + w(11)(6)(3)(6)#2293 + w(12)(6)(3)(6)#2473
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,7): w(3)(1)(1)(7)#684 + w(4)(1)(1)(7)#864
                                 + w(5)(1)(1)(7)#1044 + w(6)(1)(1)(7)#1224
                                 + w(7)(1)(1)(7)#1404 + w(8)(1)(1)(7)#1584
                                 + w(9)(1)(1)(7)#1764 + w(10)(1)(1)(7)#1944
                                 + w(11)(1)(1)(7)#2124 + w(12)(1)(1)(7)#2304
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,7): w(3)(2)(1)(7)#714 + w(4)(2)(1)(7)#894
                                 + w(5)(2)(1)(7)#1074 + w(6)(2)(1)(7)#1254
                                 + w(7)(2)(1)(7)#1434 + w(8)(2)(1)(7)#1614
                                 + w(9)(2)(1)(7)#1794 + w(10)(2)(1)(7)#1974
                                 + w(11)(2)(1)(7)#2154 + w(12)(2)(1)(7)#2334
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,7): w(3)(3)(1)(7)#744 + w(4)(3)(1)(7)#924
                                 + w(5)(3)(1)(7)#1104 + w(6)(3)(1)(7)#1284
                                 + w(7)(3)(1)(7)#1464 + w(8)(3)(1)(7)#1644
                                 + w(9)(3)(1)(7)#1824 + w(10)(3)(1)(7)#2004
                                 + w(11)(3)(1)(7)#2184 + w(12)(3)(1)(7)#2364
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,7): w(3)(4)(1)(7)#774 + w(4)(4)(1)(7)#954
                                 + w(5)(4)(1)(7)#1134 + w(6)(4)(1)(7)#1314
                                 + w(7)(4)(1)(7)#1494 + w(8)(4)(1)(7)#1674
                                 + w(9)(4)(1)(7)#1854 + w(10)(4)(1)(7)#2034
                                 + w(11)(4)(1)(7)#2214 + w(12)(4)(1)(7)#2394
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,7): w(3)(5)(1)(7)#804 + w(4)(5)(1)(7)#984
                                 + w(5)(5)(1)(7)#1164 + w(6)(5)(1)(7)#1344
                                 + w(7)(5)(1)(7)#1524 + w(8)(5)(1)(7)#1704
                                 + w(9)(5)(1)(7)#1884 + w(10)(5)(1)(7)#2064
                                 + w(11)(5)(1)(7)#2244 + w(12)(5)(1)(7)#2424
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,7): w(3)(6)(1)(7)#834 + w(4)(6)(1)(7)#1014
                                 + w(5)(6)(1)(7)#1194 + w(6)(6)(1)(7)#1374
                                 + w(7)(6)(1)(7)#1554 + w(8)(6)(1)(7)#1734
                                 + w(9)(6)(1)(7)#1914 + w(10)(6)(1)(7)#2094
                                 + w(11)(6)(1)(7)#2274 + w(12)(6)(1)(7)#2454
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,7): w(3)(1)(2)(7)#694 + w(4)(1)(2)(7)#874
                                 + w(5)(1)(2)(7)#1054 + w(6)(1)(2)(7)#1234
                                 + w(7)(1)(2)(7)#1414 + w(8)(1)(2)(7)#1594
                                 + w(9)(1)(2)(7)#1774 + w(10)(1)(2)(7)#1954
                                 + w(11)(1)(2)(7)#2134 + w(12)(1)(2)(7)#2314
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,7): w(3)(2)(2)(7)#724 + w(4)(2)(2)(7)#904
                                 + w(5)(2)(2)(7)#1084 + w(6)(2)(2)(7)#1264
                                 + w(7)(2)(2)(7)#1444 + w(8)(2)(2)(7)#1624
                                 + w(9)(2)(2)(7)#1804 + w(10)(2)(2)(7)#1984
                                 + w(11)(2)(2)(7)#2164 + w(12)(2)(2)(7)#2344
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,7): w(3)(3)(2)(7)#754 + w(4)(3)(2)(7)#934
                                 + w(5)(3)(2)(7)#1114 + w(6)(3)(2)(7)#1294
                                 + w(7)(3)(2)(7)#1474 + w(8)(3)(2)(7)#1654
                                 + w(9)(3)(2)(7)#1834 + w(10)(3)(2)(7)#2014
                                 + w(11)(3)(2)(7)#2194 + w(12)(3)(2)(7)#2374
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,7): w(3)(4)(2)(7)#784 + w(4)(4)(2)(7)#964
                                 + w(5)(4)(2)(7)#1144 + w(6)(4)(2)(7)#1324
                                 + w(7)(4)(2)(7)#1504 + w(8)(4)(2)(7)#1684
                                 + w(9)(4)(2)(7)#1864 + w(10)(4)(2)(7)#2044
                                 + w(11)(4)(2)(7)#2224 + w(12)(4)(2)(7)#2404
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,7): w(3)(5)(2)(7)#814 + w(4)(5)(2)(7)#994
                                 + w(5)(5)(2)(7)#1174 + w(6)(5)(2)(7)#1354
                                 + w(7)(5)(2)(7)#1534 + w(8)(5)(2)(7)#1714
                                 + w(9)(5)(2)(7)#1894 + w(10)(5)(2)(7)#2074
                                 + w(11)(5)(2)(7)#2254 + w(12)(5)(2)(7)#2434
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,7): w(3)(6)(2)(7)#844 + w(4)(6)(2)(7)#1024
                                 + w(5)(6)(2)(7)#1204 + w(6)(6)(2)(7)#1384
                                 + w(7)(6)(2)(7)#1564 + w(8)(6)(2)(7)#1744
                                 + w(9)(6)(2)(7)#1924 + w(10)(6)(2)(7)#2104
                                 + w(11)(6)(2)(7)#2284 + w(12)(6)(2)(7)#2464
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,7): w(3)(1)(3)(7)#704 + w(4)(1)(3)(7)#884
                                 + w(5)(1)(3)(7)#1064 + w(6)(1)(3)(7)#1244
                                 + w(7)(1)(3)(7)#1424 + w(8)(1)(3)(7)#1604
                                 + w(9)(1)(3)(7)#1784 + w(10)(1)(3)(7)#1964
                                 + w(11)(1)(3)(7)#2144 + w(12)(1)(3)(7)#2324
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,7): w(3)(2)(3)(7)#734 + w(4)(2)(3)(7)#914
                                 + w(5)(2)(3)(7)#1094 + w(6)(2)(3)(7)#1274
                                 + w(7)(2)(3)(7)#1454 + w(8)(2)(3)(7)#1634
                                 + w(9)(2)(3)(7)#1814 + w(10)(2)(3)(7)#1994
                                 + w(11)(2)(3)(7)#2174 + w(12)(2)(3)(7)#2354
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,7): w(3)(3)(3)(7)#764 + w(4)(3)(3)(7)#944
                                 + w(5)(3)(3)(7)#1124 + w(6)(3)(3)(7)#1304
                                 + w(7)(3)(3)(7)#1484 + w(8)(3)(3)(7)#1664
                                 + w(9)(3)(3)(7)#1844 + w(10)(3)(3)(7)#2024
                                 + w(11)(3)(3)(7)#2204 + w(12)(3)(3)(7)#2384
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,7): w(3)(4)(3)(7)#794 + w(4)(4)(3)(7)#974
                                 + w(5)(4)(3)(7)#1154 + w(6)(4)(3)(7)#1334
                                 + w(7)(4)(3)(7)#1514 + w(8)(4)(3)(7)#1694
                                 + w(9)(4)(3)(7)#1874 + w(10)(4)(3)(7)#2054
                                 + w(11)(4)(3)(7)#2234 + w(12)(4)(3)(7)#2414
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,7): w(3)(5)(3)(7)#824 + w(4)(5)(3)(7)#1004
                                 + w(5)(5)(3)(7)#1184 + w(6)(5)(3)(7)#1364
                                 + w(7)(5)(3)(7)#1544 + w(8)(5)(3)(7)#1724
                                 + w(9)(5)(3)(7)#1904 + w(10)(5)(3)(7)#2084
                                 + w(11)(5)(3)(7)#2264 + w(12)(5)(3)(7)#2444
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,7): w(3)(6)(3)(7)#854 + w(4)(6)(3)(7)#1034
                                 + w(5)(6)(3)(7)#1214 + w(6)(6)(3)(7)#1394
                                 + w(7)(6)(3)(7)#1574 + w(8)(6)(3)(7)#1754
                                 + w(9)(6)(3)(7)#1934 + w(10)(6)(3)(7)#2114
                                 + w(11)(6)(3)(7)#2294 + w(12)(6)(3)(7)#2474
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,8): w(3)(1)(1)(8)#685 + w(4)(1)(1)(8)#865
                                 + w(5)(1)(1)(8)#1045 + w(6)(1)(1)(8)#1225
                                 + w(7)(1)(1)(8)#1405 + w(8)(1)(1)(8)#1585
                                 + w(9)(1)(1)(8)#1765 + w(10)(1)(1)(8)#1945
                                 + w(11)(1)(1)(8)#2125 + w(12)(1)(1)(8)#2305
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,8): w(3)(2)(1)(8)#715 + w(4)(2)(1)(8)#895
                                 + w(5)(2)(1)(8)#1075 + w(6)(2)(1)(8)#1255
                                 + w(7)(2)(1)(8)#1435 + w(8)(2)(1)(8)#1615
                                 + w(9)(2)(1)(8)#1795 + w(10)(2)(1)(8)#1975
                                 + w(11)(2)(1)(8)#2155 + w(12)(2)(1)(8)#2335
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,8): w(3)(3)(1)(8)#745 + w(4)(3)(1)(8)#925
                                 + w(5)(3)(1)(8)#1105 + w(6)(3)(1)(8)#1285
                                 + w(7)(3)(1)(8)#1465 + w(8)(3)(1)(8)#1645
                                 + w(9)(3)(1)(8)#1825 + w(10)(3)(1)(8)#2005
                                 + w(11)(3)(1)(8)#2185 + w(12)(3)(1)(8)#2365
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,8): w(3)(4)(1)(8)#775 + w(4)(4)(1)(8)#955
                                 + w(5)(4)(1)(8)#1135 + w(6)(4)(1)(8)#1315
                                 + w(7)(4)(1)(8)#1495 + w(8)(4)(1)(8)#1675
                                 + w(9)(4)(1)(8)#1855 + w(10)(4)(1)(8)#2035
                                 + w(11)(4)(1)(8)#2215 + w(12)(4)(1)(8)#2395
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,8): w(3)(5)(1)(8)#805 + w(4)(5)(1)(8)#985
                                 + w(5)(5)(1)(8)#1165 + w(6)(5)(1)(8)#1345
                                 + w(7)(5)(1)(8)#1525 + w(8)(5)(1)(8)#1705
                                 + w(9)(5)(1)(8)#1885 + w(10)(5)(1)(8)#2065
                                 + w(11)(5)(1)(8)#2245 + w(12)(5)(1)(8)#2425
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,8): w(3)(6)(1)(8)#835 + w(4)(6)(1)(8)#1015
                                 + w(5)(6)(1)(8)#1195 + w(6)(6)(1)(8)#1375
                                 + w(7)(6)(1)(8)#1555 + w(8)(6)(1)(8)#1735
                                 + w(9)(6)(1)(8)#1915 + w(10)(6)(1)(8)#2095
                                 + w(11)(6)(1)(8)#2275 + w(12)(6)(1)(8)#2455
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,8): w(3)(1)(2)(8)#695 + w(4)(1)(2)(8)#875
                                 + w(5)(1)(2)(8)#1055 + w(6)(1)(2)(8)#1235
                                 + w(7)(1)(2)(8)#1415 + w(8)(1)(2)(8)#1595
                                 + w(9)(1)(2)(8)#1775 + w(10)(1)(2)(8)#1955
                                 + w(11)(1)(2)(8)#2135 + w(12)(1)(2)(8)#2315
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,8): w(3)(2)(2)(8)#725 + w(4)(2)(2)(8)#905
                                 + w(5)(2)(2)(8)#1085 + w(6)(2)(2)(8)#1265
                                 + w(7)(2)(2)(8)#1445 + w(8)(2)(2)(8)#1625
                                 + w(9)(2)(2)(8)#1805 + w(10)(2)(2)(8)#1985
                                 + w(11)(2)(2)(8)#2165 + w(12)(2)(2)(8)#2345
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,8): w(3)(3)(2)(8)#755 + w(4)(3)(2)(8)#935
                                 + w(5)(3)(2)(8)#1115 + w(6)(3)(2)(8)#1295
                                 + w(7)(3)(2)(8)#1475 + w(8)(3)(2)(8)#1655
                                 + w(9)(3)(2)(8)#1835 + w(10)(3)(2)(8)#2015
                                 + w(11)(3)(2)(8)#2195 + w(12)(3)(2)(8)#2375
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,8): w(3)(4)(2)(8)#785 + w(4)(4)(2)(8)#965
                                 + w(5)(4)(2)(8)#1145 + w(6)(4)(2)(8)#1325
                                 + w(7)(4)(2)(8)#1505 + w(8)(4)(2)(8)#1685
                                 + w(9)(4)(2)(8)#1865 + w(10)(4)(2)(8)#2045
                                 + w(11)(4)(2)(8)#2225 + w(12)(4)(2)(8)#2405
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,8): w(3)(5)(2)(8)#815 + w(4)(5)(2)(8)#995
                                 + w(5)(5)(2)(8)#1175 + w(6)(5)(2)(8)#1355
                                 + w(7)(5)(2)(8)#1535 + w(8)(5)(2)(8)#1715
                                 + w(9)(5)(2)(8)#1895 + w(10)(5)(2)(8)#2075
                                 + w(11)(5)(2)(8)#2255 + w(12)(5)(2)(8)#2435
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,8): w(3)(6)(2)(8)#845 + w(4)(6)(2)(8)#1025
                                 + w(5)(6)(2)(8)#1205 + w(6)(6)(2)(8)#1385
                                 + w(7)(6)(2)(8)#1565 + w(8)(6)(2)(8)#1745
                                 + w(9)(6)(2)(8)#1925 + w(10)(6)(2)(8)#2105
                                 + w(11)(6)(2)(8)#2285 + w(12)(6)(2)(8)#2465
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,8): w(3)(1)(3)(8)#705 + w(4)(1)(3)(8)#885
                                 + w(5)(1)(3)(8)#1065 + w(6)(1)(3)(8)#1245
                                 + w(7)(1)(3)(8)#1425 + w(8)(1)(3)(8)#1605
                                 + w(9)(1)(3)(8)#1785 + w(10)(1)(3)(8)#1965
                                 + w(11)(1)(3)(8)#2145 + w(12)(1)(3)(8)#2325
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,8): w(3)(2)(3)(8)#735 + w(4)(2)(3)(8)#915
                                 + w(5)(2)(3)(8)#1095 + w(6)(2)(3)(8)#1275
                                 + w(7)(2)(3)(8)#1455 + w(8)(2)(3)(8)#1635
                                 + w(9)(2)(3)(8)#1815 + w(10)(2)(3)(8)#1995
                                 + w(11)(2)(3)(8)#2175 + w(12)(2)(3)(8)#2355
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,8): w(3)(3)(3)(8)#765 + w(4)(3)(3)(8)#945
                                 + w(5)(3)(3)(8)#1125 + w(6)(3)(3)(8)#1305
                                 + w(7)(3)(3)(8)#1485 + w(8)(3)(3)(8)#1665
                                 + w(9)(3)(3)(8)#1845 + w(10)(3)(3)(8)#2025
                                 + w(11)(3)(3)(8)#2205 + w(12)(3)(3)(8)#2385
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,8): w(3)(4)(3)(8)#795 + w(4)(4)(3)(8)#975
                                 + w(5)(4)(3)(8)#1155 + w(6)(4)(3)(8)#1335
                                 + w(7)(4)(3)(8)#1515 + w(8)(4)(3)(8)#1695
                                 + w(9)(4)(3)(8)#1875 + w(10)(4)(3)(8)#2055
                                 + w(11)(4)(3)(8)#2235 + w(12)(4)(3)(8)#2415
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,8): w(3)(5)(3)(8)#825 + w(4)(5)(3)(8)#1005
                                 + w(5)(5)(3)(8)#1185 + w(6)(5)(3)(8)#1365
                                 + w(7)(5)(3)(8)#1545 + w(8)(5)(3)(8)#1725
                                 + w(9)(5)(3)(8)#1905 + w(10)(5)(3)(8)#2085
                                 + w(11)(5)(3)(8)#2265 + w(12)(5)(3)(8)#2445
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,8): w(3)(6)(3)(8)#855 + w(4)(6)(3)(8)#1035
                                 + w(5)(6)(3)(8)#1215 + w(6)(6)(3)(8)#1395
                                 + w(7)(6)(3)(8)#1575 + w(8)(6)(3)(8)#1755
                                 + w(9)(6)(3)(8)#1935 + w(10)(6)(3)(8)#2115
                                 + w(11)(6)(3)(8)#2295 + w(12)(6)(3)(8)#2475
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,9): w(3)(1)(1)(9)#686 + w(4)(1)(1)(9)#866
                                 + w(5)(1)(1)(9)#1046 + w(6)(1)(1)(9)#1226
                                 + w(7)(1)(1)(9)#1406 + w(8)(1)(1)(9)#1586
                                 + w(9)(1)(1)(9)#1766 + w(10)(1)(1)(9)#1946
                                 + w(11)(1)(1)(9)#2126 + w(12)(1)(1)(9)#2306
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,9): w(3)(2)(1)(9)#716 + w(4)(2)(1)(9)#896
                                 + w(5)(2)(1)(9)#1076 + w(6)(2)(1)(9)#1256
                                 + w(7)(2)(1)(9)#1436 + w(8)(2)(1)(9)#1616
                                 + w(9)(2)(1)(9)#1796 + w(10)(2)(1)(9)#1976
                                 + w(11)(2)(1)(9)#2156 + w(12)(2)(1)(9)#2336
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,9): w(3)(3)(1)(9)#746 + w(4)(3)(1)(9)#926
                                 + w(5)(3)(1)(9)#1106 + w(6)(3)(1)(9)#1286
                                 + w(7)(3)(1)(9)#1466 + w(8)(3)(1)(9)#1646
                                 + w(9)(3)(1)(9)#1826 + w(10)(3)(1)(9)#2006
                                 + w(11)(3)(1)(9)#2186 + w(12)(3)(1)(9)#2366
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,9): w(3)(4)(1)(9)#776 + w(4)(4)(1)(9)#956
                                 + w(5)(4)(1)(9)#1136 + w(6)(4)(1)(9)#1316
                                 + w(7)(4)(1)(9)#1496 + w(8)(4)(1)(9)#1676
                                 + w(9)(4)(1)(9)#1856 + w(10)(4)(1)(9)#2036
                                 + w(11)(4)(1)(9)#2216 + w(12)(4)(1)(9)#2396
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,9): w(3)(5)(1)(9)#806 + w(4)(5)(1)(9)#986
                                 + w(5)(5)(1)(9)#1166 + w(6)(5)(1)(9)#1346
                                 + w(7)(5)(1)(9)#1526 + w(8)(5)(1)(9)#1706
                                 + w(9)(5)(1)(9)#1886 + w(10)(5)(1)(9)#2066
                                 + w(11)(5)(1)(9)#2246 + w(12)(5)(1)(9)#2426
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,9): w(3)(6)(1)(9)#836 + w(4)(6)(1)(9)#1016
                                 + w(5)(6)(1)(9)#1196 + w(6)(6)(1)(9)#1376
                                 + w(7)(6)(1)(9)#1556 + w(8)(6)(1)(9)#1736
                                 + w(9)(6)(1)(9)#1916 + w(10)(6)(1)(9)#2096
                                 + w(11)(6)(1)(9)#2276 + w(12)(6)(1)(9)#2456
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,9): w(3)(1)(2)(9)#696 + w(4)(1)(2)(9)#876
                                 + w(5)(1)(2)(9)#1056 + w(6)(1)(2)(9)#1236
                                 + w(7)(1)(2)(9)#1416 + w(8)(1)(2)(9)#1596
                                 + w(9)(1)(2)(9)#1776 + w(10)(1)(2)(9)#1956
                                 + w(11)(1)(2)(9)#2136 + w(12)(1)(2)(9)#2316
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,9): w(3)(2)(2)(9)#726 + w(4)(2)(2)(9)#906
                                 + w(5)(2)(2)(9)#1086 + w(6)(2)(2)(9)#1266
                                 + w(7)(2)(2)(9)#1446 + w(8)(2)(2)(9)#1626
                                 + w(9)(2)(2)(9)#1806 + w(10)(2)(2)(9)#1986
                                 + w(11)(2)(2)(9)#2166 + w(12)(2)(2)(9)#2346
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,9): w(3)(3)(2)(9)#756 + w(4)(3)(2)(9)#936
                                 + w(5)(3)(2)(9)#1116 + w(6)(3)(2)(9)#1296
                                 + w(7)(3)(2)(9)#1476 + w(8)(3)(2)(9)#1656
                                 + w(9)(3)(2)(9)#1836 + w(10)(3)(2)(9)#2016
                                 + w(11)(3)(2)(9)#2196 + w(12)(3)(2)(9)#2376
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,9): w(3)(4)(2)(9)#786 + w(4)(4)(2)(9)#966
                                 + w(5)(4)(2)(9)#1146 + w(6)(4)(2)(9)#1326
                                 + w(7)(4)(2)(9)#1506 + w(8)(4)(2)(9)#1686
                                 + w(9)(4)(2)(9)#1866 + w(10)(4)(2)(9)#2046
                                 + w(11)(4)(2)(9)#2226 + w(12)(4)(2)(9)#2406
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,9): w(3)(5)(2)(9)#816 + w(4)(5)(2)(9)#996
                                 + w(5)(5)(2)(9)#1176 + w(6)(5)(2)(9)#1356
                                 + w(7)(5)(2)(9)#1536 + w(8)(5)(2)(9)#1716
                                 + w(9)(5)(2)(9)#1896 + w(10)(5)(2)(9)#2076
                                 + w(11)(5)(2)(9)#2256 + w(12)(5)(2)(9)#2436
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,9): w(3)(6)(2)(9)#846 + w(4)(6)(2)(9)#1026
                                 + w(5)(6)(2)(9)#1206 + w(6)(6)(2)(9)#1386
                                 + w(7)(6)(2)(9)#1566 + w(8)(6)(2)(9)#1746
                                 + w(9)(6)(2)(9)#1926 + w(10)(6)(2)(9)#2106
                                 + w(11)(6)(2)(9)#2286 + w(12)(6)(2)(9)#2466
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,9): w(3)(1)(3)(9)#706 + w(4)(1)(3)(9)#886
                                 + w(5)(1)(3)(9)#1066 + w(6)(1)(3)(9)#1246
                                 + w(7)(1)(3)(9)#1426 + w(8)(1)(3)(9)#1606
                                 + w(9)(1)(3)(9)#1786 + w(10)(1)(3)(9)#1966
                                 + w(11)(1)(3)(9)#2146 + w(12)(1)(3)(9)#2326
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,9): w(3)(2)(3)(9)#736 + w(4)(2)(3)(9)#916
                                 + w(5)(2)(3)(9)#1096 + w(6)(2)(3)(9)#1276
                                 + w(7)(2)(3)(9)#1456 + w(8)(2)(3)(9)#1636
                                 + w(9)(2)(3)(9)#1816 + w(10)(2)(3)(9)#1996
                                 + w(11)(2)(3)(9)#2176 + w(12)(2)(3)(9)#2356
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,9): w(3)(3)(3)(9)#766 + w(4)(3)(3)(9)#946
                                 + w(5)(3)(3)(9)#1126 + w(6)(3)(3)(9)#1306
                                 + w(7)(3)(3)(9)#1486 + w(8)(3)(3)(9)#1666
                                 + w(9)(3)(3)(9)#1846 + w(10)(3)(3)(9)#2026
                                 + w(11)(3)(3)(9)#2206 + w(12)(3)(3)(9)#2386
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,9): w(3)(4)(3)(9)#796 + w(4)(4)(3)(9)#976
                                 + w(5)(4)(3)(9)#1156 + w(6)(4)(3)(9)#1336
                                 + w(7)(4)(3)(9)#1516 + w(8)(4)(3)(9)#1696
                                 + w(9)(4)(3)(9)#1876 + w(10)(4)(3)(9)#2056
                                 + w(11)(4)(3)(9)#2236 + w(12)(4)(3)(9)#2416
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,9): w(3)(5)(3)(9)#826 + w(4)(5)(3)(9)#1006
                                 + w(5)(5)(3)(9)#1186 + w(6)(5)(3)(9)#1366
                                 + w(7)(5)(3)(9)#1546 + w(8)(5)(3)(9)#1726
                                 + w(9)(5)(3)(9)#1906 + w(10)(5)(3)(9)#2086
                                 + w(11)(5)(3)(9)#2266 + w(12)(5)(3)(9)#2446
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,9): w(3)(6)(3)(9)#856 + w(4)(6)(3)(9)#1036
                                 + w(5)(6)(3)(9)#1216 + w(6)(6)(3)(9)#1396
                                 + w(7)(6)(3)(9)#1576 + w(8)(6)(3)(9)#1756
                                 + w(9)(6)(3)(9)#1936 + w(10)(6)(3)(9)#2116
                                 + w(11)(6)(3)(9)#2296 + w(12)(6)(3)(9)#2476
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,10): w(3)(1)(1)(10)#687 + w(4)(1)(1)(10)#867
                                 + w(5)(1)(1)(10)#1047 + w(6)(1)(1)(10)#1227
                                 + w(7)(1)(1)(10)#1407 + w(8)(1)(1)(10)#1587
                                 + w(9)(1)(1)(10)#1767 + w(10)(1)(1)(10)#1947
                                 + w(11)(1)(1)(10)#2127 + w(12)(1)(1)(10)#2307
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,10): w(3)(2)(1)(10)#717 + w(4)(2)(1)(10)#897
                                 + w(5)(2)(1)(10)#1077 + w(6)(2)(1)(10)#1257
                                 + w(7)(2)(1)(10)#1437 + w(8)(2)(1)(10)#1617
                                 + w(9)(2)(1)(10)#1797 + w(10)(2)(1)(10)#1977
                                 + w(11)(2)(1)(10)#2157 + w(12)(2)(1)(10)#2337
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,10): w(3)(3)(1)(10)#747 + w(4)(3)(1)(10)#927
                                 + w(5)(3)(1)(10)#1107 + w(6)(3)(1)(10)#1287
                                 + w(7)(3)(1)(10)#1467 + w(8)(3)(1)(10)#1647
                                 + w(9)(3)(1)(10)#1827 + w(10)(3)(1)(10)#2007
                                 + w(11)(3)(1)(10)#2187 + w(12)(3)(1)(10)#2367
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,10): w(3)(4)(1)(10)#777 + w(4)(4)(1)(10)#957
                                 + w(5)(4)(1)(10)#1137 + w(6)(4)(1)(10)#1317
                                 + w(7)(4)(1)(10)#1497 + w(8)(4)(1)(10)#1677
                                 + w(9)(4)(1)(10)#1857 + w(10)(4)(1)(10)#2037
                                 + w(11)(4)(1)(10)#2217 + w(12)(4)(1)(10)#2397
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,10): w(3)(5)(1)(10)#807 + w(4)(5)(1)(10)#987
                                 + w(5)(5)(1)(10)#1167 + w(6)(5)(1)(10)#1347
                                 + w(7)(5)(1)(10)#1527 + w(8)(5)(1)(10)#1707
                                 + w(9)(5)(1)(10)#1887 + w(10)(5)(1)(10)#2067
                                 + w(11)(5)(1)(10)#2247 + w(12)(5)(1)(10)#2427
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,10): w(3)(6)(1)(10)#837 + w(4)(6)(1)(10)#1017
                                 + w(5)(6)(1)(10)#1197 + w(6)(6)(1)(10)#1377
                                 + w(7)(6)(1)(10)#1557 + w(8)(6)(1)(10)#1737
                                 + w(9)(6)(1)(10)#1917 + w(10)(6)(1)(10)#2097
                                 + w(11)(6)(1)(10)#2277 + w(12)(6)(1)(10)#2457
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,10): w(3)(1)(2)(10)#697 + w(4)(1)(2)(10)#877
                                 + w(5)(1)(2)(10)#1057 + w(6)(1)(2)(10)#1237
                                 + w(7)(1)(2)(10)#1417 + w(8)(1)(2)(10)#1597
                                 + w(9)(1)(2)(10)#1777 + w(10)(1)(2)(10)#1957
                                 + w(11)(1)(2)(10)#2137 + w(12)(1)(2)(10)#2317
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,10): w(3)(2)(2)(10)#727 + w(4)(2)(2)(10)#907
                                 + w(5)(2)(2)(10)#1087 + w(6)(2)(2)(10)#1267
                                 + w(7)(2)(2)(10)#1447 + w(8)(2)(2)(10)#1627
                                 + w(9)(2)(2)(10)#1807 + w(10)(2)(2)(10)#1987
                                 + w(11)(2)(2)(10)#2167 + w(12)(2)(2)(10)#2347
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,10): w(3)(3)(2)(10)#757 + w(4)(3)(2)(10)#937
                                 + w(5)(3)(2)(10)#1117 + w(6)(3)(2)(10)#1297
                                 + w(7)(3)(2)(10)#1477 + w(8)(3)(2)(10)#1657
                                 + w(9)(3)(2)(10)#1837 + w(10)(3)(2)(10)#2017
                                 + w(11)(3)(2)(10)#2197 + w(12)(3)(2)(10)#2377
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,10): w(3)(4)(2)(10)#787 + w(4)(4)(2)(10)#967
                                 + w(5)(4)(2)(10)#1147 + w(6)(4)(2)(10)#1327
                                 + w(7)(4)(2)(10)#1507 + w(8)(4)(2)(10)#1687
                                 + w(9)(4)(2)(10)#1867 + w(10)(4)(2)(10)#2047
                                 + w(11)(4)(2)(10)#2227 + w(12)(4)(2)(10)#2407
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,10): w(3)(5)(2)(10)#817 + w(4)(5)(2)(10)#997
                                 + w(5)(5)(2)(10)#1177 + w(6)(5)(2)(10)#1357
                                 + w(7)(5)(2)(10)#1537 + w(8)(5)(2)(10)#1717
                                 + w(9)(5)(2)(10)#1897 + w(10)(5)(2)(10)#2077
                                 + w(11)(5)(2)(10)#2257 + w(12)(5)(2)(10)#2437
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,10): w(3)(6)(2)(10)#847 + w(4)(6)(2)(10)#1027
                                 + w(5)(6)(2)(10)#1207 + w(6)(6)(2)(10)#1387
                                 + w(7)(6)(2)(10)#1567 + w(8)(6)(2)(10)#1747
                                 + w(9)(6)(2)(10)#1927 + w(10)(6)(2)(10)#2107
                                 + w(11)(6)(2)(10)#2287 + w(12)(6)(2)(10)#2467
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,10): w(3)(1)(3)(10)#707 + w(4)(1)(3)(10)#887
                                 + w(5)(1)(3)(10)#1067 + w(6)(1)(3)(10)#1247
                                 + w(7)(1)(3)(10)#1427 + w(8)(1)(3)(10)#1607
                                 + w(9)(1)(3)(10)#1787 + w(10)(1)(3)(10)#1967
                                 + w(11)(1)(3)(10)#2147 + w(12)(1)(3)(10)#2327
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,10): w(3)(2)(3)(10)#737 + w(4)(2)(3)(10)#917
                                 + w(5)(2)(3)(10)#1097 + w(6)(2)(3)(10)#1277
                                 + w(7)(2)(3)(10)#1457 + w(8)(2)(3)(10)#1637
                                 + w(9)(2)(3)(10)#1817 + w(10)(2)(3)(10)#1997
                                 + w(11)(2)(3)(10)#2177 + w(12)(2)(3)(10)#2357
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,10): w(3)(3)(3)(10)#767 + w(4)(3)(3)(10)#947
                                 + w(5)(3)(3)(10)#1127 + w(6)(3)(3)(10)#1307
                                 + w(7)(3)(3)(10)#1487 + w(8)(3)(3)(10)#1667
                                 + w(9)(3)(3)(10)#1847 + w(10)(3)(3)(10)#2027
                                 + w(11)(3)(3)(10)#2207 + w(12)(3)(3)(10)#2387
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,10): w(3)(4)(3)(10)#797 + w(4)(4)(3)(10)#977
                                 + w(5)(4)(3)(10)#1157 + w(6)(4)(3)(10)#1337
                                 + w(7)(4)(3)(10)#1517 + w(8)(4)(3)(10)#1697
                                 + w(9)(4)(3)(10)#1877 + w(10)(4)(3)(10)#2057
                                 + w(11)(4)(3)(10)#2237 + w(12)(4)(3)(10)#2417
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,10): w(3)(5)(3)(10)#827 + w(4)(5)(3)(10)#1007
                                 + w(5)(5)(3)(10)#1187 + w(6)(5)(3)(10)#1367
                                 + w(7)(5)(3)(10)#1547 + w(8)(5)(3)(10)#1727
                                 + w(9)(5)(3)(10)#1907 + w(10)(5)(3)(10)#2087
                                 + w(11)(5)(3)(10)#2267 + w(12)(5)(3)(10)#2447
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,10): w(3)(6)(3)(10)#857 + w(4)(6)(3)(10)#1037
                                 + w(5)(6)(3)(10)#1217 + w(6)(6)(3)(10)#1397
                                 + w(7)(6)(3)(10)#1577 + w(8)(6)(3)(10)#1757
                                 + w(9)(6)(3)(10)#1937 + w(10)(6)(3)(10)#2117
                                 + w(11)(6)(3)(10)#2297 + w(12)(6)(3)(10)#2477
                                 <= 44
Bounds
 0 <= p(1)#0 <= 304
 0 <= p(2)#1 <= 304
 0 <= p(3)#2 <= 304
 0 <= I_plant(1)#3 <= 152
 0 <= I_plant(2)#4 <= 152
 0 <= I_plant(3)#5 <= 152
 0 <= I_warehouse(1)(1)(1)#6 <= 136.8
 0 <= I_warehouse(1)(1)(2)#7 <= 136.8
 0 <= I_warehouse(1)(1)(3)#8 <= 136.8
 0 <= I_warehouse(1)(1)(4)#9 <= 136.8
 0 <= I_warehouse(1)(1)(5)#10 <= 136.8
 0 <= I_warehouse(1)(1)(6)#11 <= 136.8
 0 <= I_warehouse(1)(1)(7)#12 <= 136.8
 0 <= I_warehouse(1)(1)(8)#13 <= 136.8
 0 <= I_warehouse(1)(1)(9)#14 <= 136.8
 0 <= I_warehouse(1)(1)(10)#15 <= 136.8
 0 <= I_warehouse(1)(2)(1)#16 <= 136.8
 0 <= I_warehouse(1)(2)(2)#17 <= 136.8
 0 <= I_warehouse(1)(2)(3)#18 <= 136.8
 0 <= I_warehouse(1)(2)(4)#19 <= 136.8
 0 <= I_warehouse(1)(2)(5)#20 <= 136.8
 0 <= I_warehouse(1)(2)(6)#21 <= 136.8
 0 <= I_warehouse(1)(2)(7)#22 <= 136.8
 0 <= I_warehouse(1)(2)(8)#23 <= 136.8
 0 <= I_warehouse(1)(2)(9)#24 <= 136.8
 0 <= I_warehouse(1)(2)(10)#25 <= 136.8
 0 <= I_warehouse(1)(3)(1)#26 <= 136.8
 0 <= I_warehouse(1)(3)(2)#27 <= 136.8
 0 <= I_warehouse(1)(3)(3)#28 <= 136.8
 0 <= I_warehouse(1)(3)(4)#29 <= 136.8
 0 <= I_warehouse(1)(3)(5)#30 <= 136.8
 0 <= I_warehouse(1)(3)(6)#31 <= 136.8
 0 <= I_warehouse(1)(3)(7)#32 <= 136.8
 0 <= I_warehouse(1)(3)(8)#33 <= 136.8
 0 <= I_warehouse(1)(3)(9)#34 <= 136.8
 0 <= I_warehouse(1)(3)(10)#35 <= 136.8
 0 <= I_warehouse(2)(1)(1)#36 <= 136.8
 0 <= I_warehouse(2)(1)(2)#37 <= 136.8
 0 <= I_warehouse(2)(1)(3)#38 <= 136.8
 0 <= I_warehouse(2)(1)(4)#39 <= 136.8
 0 <= I_warehouse(2)(1)(5)#40 <= 136.8
 0 <= I_warehouse(2)(1)(6)#41 <= 136.8
 0 <= I_warehouse(2)(1)(7)#42 <= 136.8
 0 <= I_warehouse(2)(1)(8)#43 <= 136.8
 0 <= I_warehouse(2)(1)(9)#44 <= 136.8
 0 <= I_warehouse(2)(1)(10)#45 <= 136.8
 0 <= I_warehouse(2)(2)(1)#46 <= 136.8
 0 <= I_warehouse(2)(2)(2)#47 <= 136.8
 0 <= I_warehouse(2)(2)(3)#48 <= 136.8
 0 <= I_warehouse(2)(2)(4)#49 <= 136.8
 0 <= I_warehouse(2)(2)(5)#50 <= 136.8
 0 <= I_warehouse(2)(2)(6)#51 <= 136.8
 0 <= I_warehouse(2)(2)(7)#52 <= 136.8
 0 <= I_warehouse(2)(2)(8)#53 <= 136.8
 0 <= I_warehouse(2)(2)(9)#54 <= 136.8
 0 <= I_warehouse(2)(2)(10)#55 <= 136.8
 0 <= I_warehouse(2)(3)(1)#56 <= 136.8
 0 <= I_warehouse(2)(3)(2)#57 <= 136.8
 0 <= I_warehouse(2)(3)(3)#58 <= 136.8
 0 <= I_warehouse(2)(3)(4)#59 <= 136.8
 0 <= I_warehouse(2)(3)(5)#60 <= 136.8
 0 <= I_warehouse(2)(3)(6)#61 <= 136.8
 0 <= I_warehouse(2)(3)(7)#62 <= 136.8
 0 <= I_warehouse(2)(3)(8)#63 <= 136.8
 0 <= I_warehouse(2)(3)(9)#64 <= 136.8
 0 <= I_warehouse(2)(3)(10)#65 <= 136.8
 0 <= I_customer(3)(1)(1)#66 <= 20
 0 <= I_customer(3)(1)(2)#67 <= 20
 0 <= I_customer(3)(1)(3)#68 <= 20
 0 <= I_customer(3)(1)(4)#69 <= 20
 0 <= I_customer(3)(1)(5)#70 <= 20
 0 <= I_customer(3)(1)(6)#71 <= 20
 0 <= I_customer(3)(1)(7)#72 <= 20
 0 <= I_customer(3)(1)(8)#73 <= 20
 0 <= I_customer(3)(1)(9)#74 <= 20
 0 <= I_customer(3)(1)(10)#75 <= 20
 0 <= I_customer(3)(2)(1)#76 <= 20
 0 <= I_customer(3)(2)(2)#77 <= 20
 0 <= I_customer(3)(2)(3)#78 <= 20
 0 <= I_customer(3)(2)(4)#79 <= 20
 0 <= I_customer(3)(2)(5)#80 <= 20
 0 <= I_customer(3)(2)(6)#81 <= 20
 0 <= I_customer(3)(2)(7)#82 <= 20
 0 <= I_customer(3)(2)(8)#83 <= 20
 0 <= I_customer(3)(2)(9)#84 <= 20
 0 <= I_customer(3)(2)(10)#85 <= 20
 0 <= I_customer(3)(3)(1)#86 <= 20
 0 <= I_customer(3)(3)(2)#87 <= 20
 0 <= I_customer(3)(3)(3)#88 <= 20
 0 <= I_customer(3)(3)(4)#89 <= 20
 0 <= I_customer(3)(3)(5)#90 <= 20
 0 <= I_customer(3)(3)(6)#91 <= 20
 0 <= I_customer(3)(3)(7)#92 <= 20
 0 <= I_customer(3)(3)(8)#93 <= 20
 0 <= I_customer(3)(3)(9)#94 <= 20
 0 <= I_customer(3)(3)(10)#95 <= 20
 0 <= I_customer(4)(1)(1)#96 <= 45
 0 <= I_customer(4)(1)(2)#97 <= 45
 0 <= I_customer(4)(1)(3)#98 <= 45
 0 <= I_customer(4)(1)(4)#99 <= 45
 0 <= I_customer(4)(1)(5)#100 <= 45
 0 <= I_customer(4)(1)(6)#101 <= 45
 0 <= I_customer(4)(1)(7)#102 <= 45
 0 <= I_customer(4)(1)(8)#103 <= 45
 0 <= I_customer(4)(1)(9)#104 <= 45
 0 <= I_customer(4)(1)(10)#105 <= 45
 0 <= I_customer(4)(2)(1)#106 <= 45
 0 <= I_customer(4)(2)(2)#107 <= 45
 0 <= I_customer(4)(2)(3)#108 <= 45
 0 <= I_customer(4)(2)(4)#109 <= 45
 0 <= I_customer(4)(2)(5)#110 <= 45
 0 <= I_customer(4)(2)(6)#111 <= 45
 0 <= I_customer(4)(2)(7)#112 <= 45
 0 <= I_customer(4)(2)(8)#113 <= 45
 0 <= I_customer(4)(2)(9)#114 <= 45
 0 <= I_customer(4)(2)(10)#115 <= 45
 0 <= I_customer(4)(3)(1)#116 <= 45
 0 <= I_customer(4)(3)(2)#117 <= 45
 0 <= I_customer(4)(3)(3)#118 <= 45
 0 <= I_customer(4)(3)(4)#119 <= 45
 0 <= I_customer(4)(3)(5)#120 <= 45
 0 <= I_customer(4)(3)(6)#121 <= 45
 0 <= I_customer(4)(3)(7)#122 <= 45
 0 <= I_customer(4)(3)(8)#123 <= 45
 0 <= I_customer(4)(3)(9)#124 <= 45
 0 <= I_customer(4)(3)(10)#125 <= 45
 0 <= I_customer(5)(1)(1)#126 <= 45
 0 <= I_customer(5)(1)(2)#127 <= 45
 0 <= I_customer(5)(1)(3)#128 <= 45
 0 <= I_customer(5)(1)(4)#129 <= 45
 0 <= I_customer(5)(1)(5)#130 <= 45
 0 <= I_customer(5)(1)(6)#131 <= 45
 0 <= I_customer(5)(1)(7)#132 <= 45
 0 <= I_customer(5)(1)(8)#133 <= 45
 0 <= I_customer(5)(1)(9)#134 <= 45
 0 <= I_customer(5)(1)(10)#135 <= 45
 0 <= I_customer(5)(2)(1)#136 <= 45
 0 <= I_customer(5)(2)(2)#137 <= 45
 0 <= I_customer(5)(2)(3)#138 <= 45
 0 <= I_customer(5)(2)(4)#139 <= 45
 0 <= I_customer(5)(2)(5)#140 <= 45
 0 <= I_customer(5)(2)(6)#141 <= 45
 0 <= I_customer(5)(2)(7)#142 <= 45
 0 <= I_customer(5)(2)(8)#143 <= 45
 0 <= I_customer(5)(2)(9)#144 <= 45
 0 <= I_customer(5)(2)(10)#145 <= 45
 0 <= I_customer(5)(3)(1)#146 <= 45
 0 <= I_customer(5)(3)(2)#147 <= 45
 0 <= I_customer(5)(3)(3)#148 <= 45
 0 <= I_customer(5)(3)(4)#149 <= 45
 0 <= I_customer(5)(3)(5)#150 <= 45
 0 <= I_customer(5)(3)(6)#151 <= 45
 0 <= I_customer(5)(3)(7)#152 <= 45
 0 <= I_customer(5)(3)(8)#153 <= 45
 0 <= I_customer(5)(3)(9)#154 <= 45
 0 <= I_customer(5)(3)(10)#155 <= 45
 0 <= I_customer(6)(1)(1)#156 <= 14
 0 <= I_customer(6)(1)(2)#157 <= 14
 0 <= I_customer(6)(1)(3)#158 <= 14
 0 <= I_customer(6)(1)(4)#159 <= 14
 0 <= I_customer(6)(1)(5)#160 <= 14
 0 <= I_customer(6)(1)(6)#161 <= 14
 0 <= I_customer(6)(1)(7)#162 <= 14
 0 <= I_customer(6)(1)(8)#163 <= 14
 0 <= I_customer(6)(1)(9)#164 <= 14
 0 <= I_customer(6)(1)(10)#165 <= 14
 0 <= I_customer(6)(2)(1)#166 <= 14
 0 <= I_customer(6)(2)(2)#167 <= 14
 0 <= I_customer(6)(2)(3)#168 <= 14
 0 <= I_customer(6)(2)(4)#169 <= 14
 0 <= I_customer(6)(2)(5)#170 <= 14
 0 <= I_customer(6)(2)(6)#171 <= 14
 0 <= I_customer(6)(2)(7)#172 <= 14
 0 <= I_customer(6)(2)(8)#173 <= 14
 0 <= I_customer(6)(2)(9)#174 <= 14
 0 <= I_customer(6)(2)(10)#175 <= 14
 0 <= I_customer(6)(3)(1)#176 <= 14
 0 <= I_customer(6)(3)(2)#177 <= 14
 0 <= I_customer(6)(3)(3)#178 <= 14
 0 <= I_customer(6)(3)(4)#179 <= 14
 0 <= I_customer(6)(3)(5)#180 <= 14
 0 <= I_customer(6)(3)(6)#181 <= 14
 0 <= I_customer(6)(3)(7)#182 <= 14
 0 <= I_customer(6)(3)(8)#183 <= 14
 0 <= I_customer(6)(3)(9)#184 <= 14
 0 <= I_customer(6)(3)(10)#185 <= 14
 0 <= I_customer(7)(1)(1)#186 <= 39
 0 <= I_customer(7)(1)(2)#187 <= 39
 0 <= I_customer(7)(1)(3)#188 <= 39
 0 <= I_customer(7)(1)(4)#189 <= 39
 0 <= I_customer(7)(1)(5)#190 <= 39
 0 <= I_customer(7)(1)(6)#191 <= 39
 0 <= I_customer(7)(1)(7)#192 <= 39
 0 <= I_customer(7)(1)(8)#193 <= 39
 0 <= I_customer(7)(1)(9)#194 <= 39
 0 <= I_customer(7)(1)(10)#195 <= 39
 0 <= I_customer(7)(2)(1)#196 <= 39
 0 <= I_customer(7)(2)(2)#197 <= 39
 0 <= I_customer(7)(2)(3)#198 <= 39
 0 <= I_customer(7)(2)(4)#199 <= 39
 0 <= I_customer(7)(2)(5)#200 <= 39
 0 <= I_customer(7)(2)(6)#201 <= 39
 0 <= I_customer(7)(2)(7)#202 <= 39
 0 <= I_customer(7)(2)(8)#203 <= 39
 0 <= I_customer(7)(2)(9)#204 <= 39
 0 <= I_customer(7)(2)(10)#205 <= 39
 0 <= I_customer(7)(3)(1)#206 <= 39
 0 <= I_customer(7)(3)(2)#207 <= 39
 0 <= I_customer(7)(3)(3)#208 <= 39
 0 <= I_customer(7)(3)(4)#209 <= 39
 0 <= I_customer(7)(3)(5)#210 <= 39
 0 <= I_customer(7)(3)(6)#211 <= 39
 0 <= I_customer(7)(3)(7)#212 <= 39
 0 <= I_customer(7)(3)(8)#213 <= 39
 0 <= I_customer(7)(3)(9)#214 <= 39
 0 <= I_customer(7)(3)(10)#215 <= 39
 0 <= I_customer(8)(1)(1)#216 <= 96
 0 <= I_customer(8)(1)(2)#217 <= 96
 0 <= I_customer(8)(1)(3)#218 <= 96
 0 <= I_customer(8)(1)(4)#219 <= 96
 0 <= I_customer(8)(1)(5)#220 <= 96
 0 <= I_customer(8)(1)(6)#221 <= 96
 0 <= I_customer(8)(1)(7)#222 <= 96
 0 <= I_customer(8)(1)(8)#223 <= 96
 0 <= I_customer(8)(1)(9)#224 <= 96
 0 <= I_customer(8)(1)(10)#225 <= 96
 0 <= I_customer(8)(2)(1)#226 <= 96
 0 <= I_customer(8)(2)(2)#227 <= 96
 0 <= I_customer(8)(2)(3)#228 <= 96
 0 <= I_customer(8)(2)(4)#229 <= 96
 0 <= I_customer(8)(2)(5)#230 <= 96
 0 <= I_customer(8)(2)(6)#231 <= 96
 0 <= I_customer(8)(2)(7)#232 <= 96
 0 <= I_customer(8)(2)(8)#233 <= 96
 0 <= I_customer(8)(2)(9)#234 <= 96
 0 <= I_customer(8)(2)(10)#235 <= 96
 0 <= I_customer(8)(3)(1)#236 <= 96
 0 <= I_customer(8)(3)(2)#237 <= 96
 0 <= I_customer(8)(3)(3)#238 <= 96
 0 <= I_customer(8)(3)(4)#239 <= 96
 0 <= I_customer(8)(3)(5)#240 <= 96
 0 <= I_customer(8)(3)(6)#241 <= 96
 0 <= I_customer(8)(3)(7)#242 <= 96
 0 <= I_customer(8)(3)(8)#243 <= 96
 0 <= I_customer(8)(3)(9)#244 <= 96
 0 <= I_customer(8)(3)(10)#245 <= 96
 0 <= I_customer(9)(1)(1)#246 <= 132
 0 <= I_customer(9)(1)(2)#247 <= 132
 0 <= I_customer(9)(1)(3)#248 <= 132
 0 <= I_customer(9)(1)(4)#249 <= 132
 0 <= I_customer(9)(1)(5)#250 <= 132
 0 <= I_customer(9)(1)(6)#251 <= 132
 0 <= I_customer(9)(1)(7)#252 <= 132
 0 <= I_customer(9)(1)(8)#253 <= 132
 0 <= I_customer(9)(1)(9)#254 <= 132
 0 <= I_customer(9)(1)(10)#255 <= 132
 0 <= I_customer(9)(2)(1)#256 <= 132
 0 <= I_customer(9)(2)(2)#257 <= 132
 0 <= I_customer(9)(2)(3)#258 <= 132
 0 <= I_customer(9)(2)(4)#259 <= 132
 0 <= I_customer(9)(2)(5)#260 <= 132
 0 <= I_customer(9)(2)(6)#261 <= 132
 0 <= I_customer(9)(2)(7)#262 <= 132
 0 <= I_customer(9)(2)(8)#263 <= 132
 0 <= I_customer(9)(2)(9)#264 <= 132
 0 <= I_customer(9)(2)(10)#265 <= 132
 0 <= I_customer(9)(3)(1)#266 <= 132
 0 <= I_customer(9)(3)(2)#267 <= 132
 0 <= I_customer(9)(3)(3)#268 <= 132
 0 <= I_customer(9)(3)(4)#269 <= 132
 0 <= I_customer(9)(3)(5)#270 <= 132
 0 <= I_customer(9)(3)(6)#271 <= 132
 0 <= I_customer(9)(3)(7)#272 <= 132
 0 <= I_customer(9)(3)(8)#273 <= 132
 0 <= I_customer(9)(3)(9)#274 <= 132
 0 <= I_customer(9)(3)(10)#275 <= 132
 0 <= I_customer(10)(1)(1)#276 <= 26
 0 <= I_customer(10)(1)(2)#277 <= 26
 0 <= I_customer(10)(1)(3)#278 <= 26
 0 <= I_customer(10)(1)(4)#279 <= 26
 0 <= I_customer(10)(1)(5)#280 <= 26
 0 <= I_customer(10)(1)(6)#281 <= 26
 0 <= I_customer(10)(1)(7)#282 <= 26
 0 <= I_customer(10)(1)(8)#283 <= 26
 0 <= I_customer(10)(1)(9)#284 <= 26
 0 <= I_customer(10)(1)(10)#285 <= 26
 0 <= I_customer(10)(2)(1)#286 <= 26
 0 <= I_customer(10)(2)(2)#287 <= 26
 0 <= I_customer(10)(2)(3)#288 <= 26
 0 <= I_customer(10)(2)(4)#289 <= 26
 0 <= I_customer(10)(2)(5)#290 <= 26
 0 <= I_customer(10)(2)(6)#291 <= 26
 0 <= I_customer(10)(2)(7)#292 <= 26
 0 <= I_customer(10)(2)(8)#293 <= 26
 0 <= I_customer(10)(2)(9)#294 <= 26
 0 <= I_customer(10)(2)(10)#295 <= 26
 0 <= I_customer(10)(3)(1)#296 <= 26
 0 <= I_customer(10)(3)(2)#297 <= 26
 0 <= I_customer(10)(3)(3)#298 <= 26
 0 <= I_customer(10)(3)(4)#299 <= 26
 0 <= I_customer(10)(3)(5)#300 <= 26
 0 <= I_customer(10)(3)(6)#301 <= 26
 0 <= I_customer(10)(3)(7)#302 <= 26
 0 <= I_customer(10)(3)(8)#303 <= 26
 0 <= I_customer(10)(3)(9)#304 <= 26
 0 <= I_customer(10)(3)(10)#305 <= 26
 0 <= I_customer(11)(1)(1)#306 <= 114
 0 <= I_customer(11)(1)(2)#307 <= 114
 0 <= I_customer(11)(1)(3)#308 <= 114
 0 <= I_customer(11)(1)(4)#309 <= 114
 0 <= I_customer(11)(1)(5)#310 <= 114
 0 <= I_customer(11)(1)(6)#311 <= 114
 0 <= I_customer(11)(1)(7)#312 <= 114
 0 <= I_customer(11)(1)(8)#313 <= 114
 0 <= I_customer(11)(1)(9)#314 <= 114
 0 <= I_customer(11)(1)(10)#315 <= 114
 0 <= I_customer(11)(2)(1)#316 <= 114
 0 <= I_customer(11)(2)(2)#317 <= 114
 0 <= I_customer(11)(2)(3)#318 <= 114
 0 <= I_customer(11)(2)(4)#319 <= 114
 0 <= I_customer(11)(2)(5)#320 <= 114
 0 <= I_customer(11)(2)(6)#321 <= 114
 0 <= I_customer(11)(2)(7)#322 <= 114
 0 <= I_customer(11)(2)(8)#323 <= 114
 0 <= I_customer(11)(2)(9)#324 <= 114
 0 <= I_customer(11)(2)(10)#325 <= 114
 0 <= I_customer(11)(3)(1)#326 <= 114
 0 <= I_customer(11)(3)(2)#327 <= 114
 0 <= I_customer(11)(3)(3)#328 <= 114
 0 <= I_customer(11)(3)(4)#329 <= 114
 0 <= I_customer(11)(3)(5)#330 <= 114
 0 <= I_customer(11)(3)(6)#331 <= 114
 0 <= I_customer(11)(3)(7)#332 <= 114
 0 <= I_customer(11)(3)(8)#333 <= 114
 0 <= I_customer(11)(3)(9)#334 <= 114
 0 <= I_customer(11)(3)(10)#335 <= 114
 0 <= I_customer(12)(1)(1)#336 <= 110
 0 <= I_customer(12)(1)(2)#337 <= 110
 0 <= I_customer(12)(1)(3)#338 <= 110
 0 <= I_customer(12)(1)(4)#339 <= 110
 0 <= I_customer(12)(1)(5)#340 <= 110
 0 <= I_customer(12)(1)(6)#341 <= 110
 0 <= I_customer(12)(1)(7)#342 <= 110
 0 <= I_customer(12)(1)(8)#343 <= 110
 0 <= I_customer(12)(1)(9)#344 <= 110
 0 <= I_customer(12)(1)(10)#345 <= 110
 0 <= I_customer(12)(2)(1)#346 <= 110
 0 <= I_customer(12)(2)(2)#347 <= 110
 0 <= I_customer(12)(2)(3)#348 <= 110
 0 <= I_customer(12)(2)(4)#349 <= 110
 0 <= I_customer(12)(2)(5)#350 <= 110
 0 <= I_customer(12)(2)(6)#351 <= 110
 0 <= I_customer(12)(2)(7)#352 <= 110
 0 <= I_customer(12)(2)(8)#353 <= 110
 0 <= I_customer(12)(2)(9)#354 <= 110
 0 <= I_customer(12)(2)(10)#355 <= 110
 0 <= I_customer(12)(3)(1)#356 <= 110
 0 <= I_customer(12)(3)(2)#357 <= 110
 0 <= I_customer(12)(3)(3)#358 <= 110
 0 <= I_customer(12)(3)(4)#359 <= 110
 0 <= I_customer(12)(3)(5)#360 <= 110
 0 <= I_customer(12)(3)(6)#361 <= 110
 0 <= I_customer(12)(3)(7)#362 <= 110
 0 <= I_customer(12)(3)(8)#363 <= 110
 0 <= I_customer(12)(3)(9)#364 <= 110
 0 <= I_customer(12)(3)(10)#365 <= 110
 0 <= b_customer(3)(1)(1)#366 <= 10
 0 <= b_customer(3)(1)(2)#367 <= 12
 0 <= b_customer(3)(1)(3)#368 <= 11
 0 <= b_customer(3)(1)(4)#369 <= 11
 0 <= b_customer(3)(1)(5)#370 <= 9
 0 <= b_customer(3)(1)(6)#371 <= 12
 0 <= b_customer(3)(1)(7)#372 <= 12
 0 <= b_customer(3)(1)(8)#373 <= 10
 0 <= b_customer(3)(1)(9)#374 <= 12
 0 <= b_customer(3)(1)(10)#375 <= 11
 0 <= b_customer(3)(2)(1)#376 <= 9
 0 <= b_customer(3)(2)(2)#377 <= 11
 0 <= b_customer(3)(2)(3)#378 <= 11
 0 <= b_customer(3)(2)(4)#379 <= 11
 0 <= b_customer(3)(2)(5)#380 <= 12
 0 <= b_customer(3)(2)(6)#381 <= 10
 0 <= b_customer(3)(2)(7)#382 <= 11
 0 <= b_customer(3)(2)(8)#383 <= 10
 0 <= b_customer(3)(2)(9)#384 <= 10
 0 <= b_customer(3)(2)(10)#385 <= 10
 0 <= b_customer(3)(3)(1)#386 <= 12
 0 <= b_customer(3)(3)(2)#387 <= 9
 0 <= b_customer(3)(3)(3)#388 <= 11
 0 <= b_customer(3)(3)(4)#389 <= 11
 0 <= b_customer(3)(3)(5)#390 <= 9
 0 <= b_customer(3)(3)(6)#391 <= 9
 0 <= b_customer(3)(3)(7)#392 <= 10
 0 <= b_customer(3)(3)(8)#393 <= 8
 0 <= b_customer(3)(3)(9)#394 <= 8
 0 <= b_customer(3)(3)(10)#395 <= 11
 0 <= b_customer(4)(1)(1)#396 <= 18
 0 <= b_customer(4)(1)(2)#397 <= 12
 0 <= b_customer(4)(1)(3)#398 <= 17
 0 <= b_customer(4)(1)(4)#399 <= 18
 0 <= b_customer(4)(1)(5)#400 <= 17
 0 <= b_customer(4)(1)(6)#401 <= 13
 0 <= b_customer(4)(1)(7)#402 <= 13
 0 <= b_customer(4)(1)(8)#403 <= 13
 0 <= b_customer(4)(1)(9)#404 <= 18
 0 <= b_customer(4)(1)(10)#405 <= 14
 0 <= b_customer(4)(2)(1)#406 <= 12
 0 <= b_customer(4)(2)(2)#407 <= 16
 0 <= b_customer(4)(2)(3)#408 <= 17
 0 <= b_customer(4)(2)(4)#409 <= 15
 0 <= b_customer(4)(2)(5)#410 <= 17
 0 <= b_customer(4)(2)(6)#411 <= 13
 0 <= b_customer(4)(2)(7)#412 <= 17
 0 <= b_customer(4)(2)(8)#413 <= 13
 0 <= b_customer(4)(2)(9)#414 <= 15
 0 <= b_customer(4)(2)(10)#415 <= 14
 0 <= b_customer(4)(3)(1)#416 <= 17
 0 <= b_customer(4)(3)(2)#417 <= 14
 0 <= b_customer(4)(3)(3)#418 <= 15
 0 <= b_customer(4)(3)(4)#419 <= 13
 0 <= b_customer(4)(3)(5)#420 <= 14
 0 <= b_customer(4)(3)(6)#421 <= 16
 0 <= b_customer(4)(3)(7)#422 <= 15
 0 <= b_customer(4)(3)(8)#423 <= 16
 0 <= b_customer(4)(3)(9)#424 <= 18
 0 <= b_customer(4)(3)(10)#425 <= 14
 0 <= b_customer(5)(1)(1)#426 <= 13
 0 <= b_customer(5)(1)(2)#427 <= 12
 0 <= b_customer(5)(1)(3)#428 <= 15
 0 <= b_customer(5)(1)(4)#429 <= 17
 0 <= b_customer(5)(1)(5)#430 <= 17
 0 <= b_customer(5)(1)(6)#431 <= 14
 0 <= b_customer(5)(1)(7)#432 <= 15
 0 <= b_customer(5)(1)(8)#433 <= 13
 0 <= b_customer(5)(1)(9)#434 <= 13
 0 <= b_customer(5)(1)(10)#435 <= 15
 0 <= b_customer(5)(2)(1)#436 <= 15
 0 <= b_customer(5)(2)(2)#437 <= 17
 0 <= b_customer(5)(2)(3)#438 <= 18
 0 <= b_customer(5)(2)(4)#439 <= 14
 0 <= b_customer(5)(2)(5)#440 <= 12
 0 <= b_customer(5)(2)(6)#441 <= 17
 0 <= b_customer(5)(2)(7)#442 <= 13
 0 <= b_customer(5)(2)(8)#443 <= 13
 0 <= b_customer(5)(2)(9)#444 <= 17
 0 <= b_customer(5)(2)(10)#445 <= 13
 0 <= b_customer(5)(3)(1)#446 <= 16
 0 <= b_customer(5)(3)(2)#447 <= 17
 0 <= b_customer(5)(3)(3)#448 <= 13
 0 <= b_customer(5)(3)(4)#449 <= 16
 0 <= b_customer(5)(3)(5)#450 <= 14
 0 <= b_customer(5)(3)(6)#451 <= 13
 0 <= b_customer(5)(3)(7)#452 <= 12
 0 <= b_customer(5)(3)(8)#453 <= 17
 0 <= b_customer(5)(3)(9)#454 <= 13
 0 <= b_customer(5)(3)(10)#455 <= 16
 0 <= b_customer(6)(1)(1)#456 <= 6
 0 <= b_customer(6)(1)(2)#457 <= 8
 0 <= b_customer(6)(1)(3)#458 <= 7
 0 <= b_customer(6)(1)(4)#459 <= 7
 0 <= b_customer(6)(1)(5)#460 <= 8
 0 <= b_customer(6)(1)(6)#461 <= 8
 0 <= b_customer(6)(1)(7)#462 <= 8
 0 <= b_customer(6)(1)(8)#463 <= 7
 0 <= b_customer(6)(1)(9)#464 <= 7
 0 <= b_customer(6)(1)(10)#465 <= 7
 0 <= b_customer(6)(2)(1)#466 <= 7
 0 <= b_customer(6)(2)(2)#467 <= 7
 0 <= b_customer(6)(2)(3)#468 <= 6
 0 <= b_customer(6)(2)(4)#469 <= 6
 0 <= b_customer(6)(2)(5)#470 <= 7
 0 <= b_customer(6)(2)(6)#471 <= 6
 0 <= b_customer(6)(2)(7)#472 <= 6
 0 <= b_customer(6)(2)(8)#473 <= 8
 0 <= b_customer(6)(2)(9)#474 <= 7
 0 <= b_customer(6)(2)(10)#475 <= 8
 0 <= b_customer(6)(3)(1)#476 <= 7
 0 <= b_customer(6)(3)(2)#477 <= 8
 0 <= b_customer(6)(3)(3)#478 <= 7
 0 <= b_customer(6)(3)(4)#479 <= 6
 0 <= b_customer(6)(3)(5)#480 <= 8
 0 <= b_customer(6)(3)(6)#481 <= 6
 0 <= b_customer(6)(3)(7)#482 <= 7
 0 <= b_customer(6)(3)(8)#483 <= 7
 0 <= b_customer(6)(3)(9)#484 <= 6
 0 <= b_customer(6)(3)(10)#485 <= 8
 0 <= b_customer(7)(1)(1)#486 <= 15
 0 <= b_customer(7)(1)(2)#487 <= 11
 0 <= b_customer(7)(1)(3)#488 <= 14
 0 <= b_customer(7)(1)(4)#489 <= 14
 0 <= b_customer(7)(1)(5)#490 <= 14
 0 <= b_customer(7)(1)(6)#491 <= 14
 0 <= b_customer(7)(1)(7)#492 <= 12
 0 <= b_customer(7)(1)(8)#493 <= 15
 0 <= b_customer(7)(1)(9)#494 <= 14
 0 <= b_customer(7)(1)(10)#495 <= 15
 0 <= b_customer(7)(2)(1)#496 <= 15
 0 <= b_customer(7)(2)(2)#497 <= 13
 0 <= b_customer(7)(2)(3)#498 <= 13
 0 <= b_customer(7)(2)(4)#499 <= 11
 0 <= b_customer(7)(2)(5)#500 <= 12
 0 <= b_customer(7)(2)(6)#501 <= 11
 0 <= b_customer(7)(2)(7)#502 <= 13
 0 <= b_customer(7)(2)(8)#503 <= 13
 0 <= b_customer(7)(2)(9)#504 <= 13
 0 <= b_customer(7)(2)(10)#505 <= 13
 0 <= b_customer(7)(3)(1)#506 <= 12
 0 <= b_customer(7)(3)(2)#507 <= 11
 0 <= b_customer(7)(3)(3)#508 <= 13
 0 <= b_customer(7)(3)(4)#509 <= 11
 0 <= b_customer(7)(3)(5)#510 <= 12
 0 <= b_customer(7)(3)(6)#511 <= 14
 0 <= b_customer(7)(3)(7)#512 <= 14
 0 <= b_customer(7)(3)(8)#513 <= 13
 0 <= b_customer(7)(3)(9)#514 <= 13
 0 <= b_customer(7)(3)(10)#515 <= 12
 0 <= b_customer(8)(1)(1)#516 <= 13
 0 <= b_customer(8)(1)(2)#517 <= 16
 0 <= b_customer(8)(1)(3)#518 <= 18
 0 <= b_customer(8)(1)(4)#519 <= 14
 0 <= b_customer(8)(1)(5)#520 <= 19
 0 <= b_customer(8)(1)(6)#521 <= 15
 0 <= b_customer(8)(1)(7)#522 <= 18
 0 <= b_customer(8)(1)(8)#523 <= 13
 0 <= b_customer(8)(1)(9)#524 <= 17
 0 <= b_customer(8)(1)(10)#525 <= 16
 0 <= b_customer(8)(2)(1)#526 <= 18
 0 <= b_customer(8)(2)(2)#527 <= 15
 0 <= b_customer(8)(2)(3)#528 <= 17
 0 <= b_customer(8)(2)(4)#529 <= 15
 0 <= b_customer(8)(2)(5)#530 <= 14
 0 <= b_customer(8)(2)(6)#531 <= 19
 0 <= b_customer(8)(2)(7)#532 <= 18
 0 <= b_customer(8)(2)(8)#533 <= 17
 0 <= b_customer(8)(2)(9)#534 <= 15
 0 <= b_customer(8)(2)(10)#535 <= 15
 0 <= b_customer(8)(3)(1)#536 <= 17
 0 <= b_customer(8)(3)(2)#537 <= 17
 0 <= b_customer(8)(3)(3)#538 <= 19
 0 <= b_customer(8)(3)(4)#539 <= 15
 0 <= b_customer(8)(3)(5)#540 <= 14
 0 <= b_customer(8)(3)(6)#541 <= 15
 0 <= b_customer(8)(3)(7)#542 <= 14
 0 <= b_customer(8)(3)(8)#543 <= 19
 0 <= b_customer(8)(3)(9)#544 <= 13
 0 <= b_customer(8)(3)(10)#545 <= 15
 0 <= b_customer(9)(1)(1)#546 <= 26
 0 <= b_customer(9)(1)(2)#547 <= 18
 0 <= b_customer(9)(1)(3)#548 <= 19
 0 <= b_customer(9)(1)(4)#549 <= 22
 0 <= b_customer(9)(1)(5)#550 <= 26
 0 <= b_customer(9)(1)(6)#551 <= 26
 0 <= b_customer(9)(1)(7)#552 <= 18
 0 <= b_customer(9)(1)(8)#553 <= 21
 0 <= b_customer(9)(1)(9)#554 <= 24
 0 <= b_customer(9)(1)(10)#555 <= 20
 0 <= b_customer(9)(2)(1)#556 <= 20
 0 <= b_customer(9)(2)(2)#557 <= 20
 0 <= b_customer(9)(2)(3)#558 <= 22
 0 <= b_customer(9)(2)(4)#559 <= 24
 0 <= b_customer(9)(2)(5)#560 <= 26
 0 <= b_customer(9)(2)(6)#561 <= 19
 0 <= b_customer(9)(2)(7)#562 <= 21
 0 <= b_customer(9)(2)(8)#563 <= 20
 0 <= b_customer(9)(2)(9)#564 <= 25
 0 <= b_customer(9)(2)(10)#565 <= 25
 0 <= b_customer(9)(3)(1)#566 <= 24
 0 <= b_customer(9)(3)(2)#567 <= 18
 0 <= b_customer(9)(3)(3)#568 <= 26
 0 <= b_customer(9)(3)(4)#569 <= 23
 0 <= b_customer(9)(3)(5)#570 <= 24
 0 <= b_customer(9)(3)(6)#571 <= 26
 0 <= b_customer(9)(3)(7)#572 <= 26
 0 <= b_customer(9)(3)(8)#573 <= 25
 0 <= b_customer(9)(3)(9)#574 <= 24
 0 <= b_customer(9)(3)(10)#575 <= 25
 0 <= b_customer(10)(1)(1)#576 <= 12
 0 <= b_customer(10)(1)(2)#577 <= 11
 0 <= b_customer(10)(1)(3)#578 <= 14
 0 <= b_customer(10)(1)(4)#579 <= 15
 0 <= b_customer(10)(1)(5)#580 <= 11
 0 <= b_customer(10)(1)(6)#581 <= 13
 0 <= b_customer(10)(1)(7)#582 <= 13
 0 <= b_customer(10)(1)(8)#583 <= 10
 0 <= b_customer(10)(1)(9)#584 <= 11
 0 <= b_customer(10)(1)(10)#585 <= 11
 0 <= b_customer(10)(2)(1)#586 <= 11
 0 <= b_customer(10)(2)(2)#587 <= 15
 0 <= b_customer(10)(2)(3)#588 <= 11
 0 <= b_customer(10)(2)(4)#589 <= 16
 0 <= b_customer(10)(2)(5)#590 <= 13
 0 <= b_customer(10)(2)(6)#591 <= 14
 0 <= b_customer(10)(2)(7)#592 <= 14
 0 <= b_customer(10)(2)(8)#593 <= 11
 0 <= b_customer(10)(2)(9)#594 <= 11
 0 <= b_customer(10)(2)(10)#595 <= 13
 0 <= b_customer(10)(3)(1)#596 <= 13
 0 <= b_customer(10)(3)(2)#597 <= 14
 0 <= b_customer(10)(3)(3)#598 <= 15
 0 <= b_customer(10)(3)(4)#599 <= 15
 0 <= b_customer(10)(3)(5)#600 <= 11
 0 <= b_customer(10)(3)(6)#601 <= 14
 0 <= b_customer(10)(3)(7)#602 <= 15
 0 <= b_customer(10)(3)(8)#603 <= 14
 0 <= b_customer(10)(3)(9)#604 <= 15
 0 <= b_customer(10)(3)(10)#605 <= 13
 0 <= b_customer(11)(1)(1)#606 <= 15
 0 <= b_customer(11)(1)(2)#607 <= 22
 0 <= b_customer(11)(1)(3)#608 <= 18
 0 <= b_customer(11)(1)(4)#609 <= 18
 0 <= b_customer(11)(1)(5)#610 <= 22
 0 <= b_customer(11)(1)(6)#611 <= 17
 0 <= b_customer(11)(1)(7)#612 <= 20
 0 <= b_customer(11)(1)(8)#613 <= 22
 0 <= b_customer(11)(1)(9)#614 <= 21
 0 <= b_customer(11)(1)(10)#615 <= 19
 0 <= b_customer(11)(2)(1)#616 <= 16
 0 <= b_customer(11)(2)(2)#617 <= 18
 0 <= b_customer(11)(2)(3)#618 <= 22
 0 <= b_customer(11)(2)(4)#619 <= 22
 0 <= b_customer(11)(2)(5)#620 <= 21
 0 <= b_customer(11)(2)(6)#621 <= 16
 0 <= b_customer(11)(2)(7)#622 <= 20
 0 <= b_customer(11)(2)(8)#623 <= 17
 0 <= b_customer(11)(2)(9)#624 <= 15
 0 <= b_customer(11)(2)(10)#625 <= 18
 0 <= b_customer(11)(3)(1)#626 <= 16
 0 <= b_customer(11)(3)(2)#627 <= 22
 0 <= b_customer(11)(3)(3)#628 <= 20
 0 <= b_customer(11)(3)(4)#629 <= 16
 0 <= b_customer(11)(3)(5)#630 <= 16
 0 <= b_customer(11)(3)(6)#631 <= 16
 0 <= b_customer(11)(3)(7)#632 <= 21
 0 <= b_customer(11)(3)(8)#633 <= 19
 0 <= b_customer(11)(3)(9)#634 <= 19
 0 <= b_customer(11)(3)(10)#635 <= 17
 0 <= b_customer(12)(1)(1)#636 <= 21
 0 <= b_customer(12)(1)(2)#637 <= 18
 0 <= b_customer(12)(1)(3)#638 <= 21
 0 <= b_customer(12)(1)(4)#639 <= 24
 0 <= b_customer(12)(1)(5)#640 <= 22
 0 <= b_customer(12)(1)(6)#641 <= 20
 0 <= b_customer(12)(1)(7)#642 <= 20
 0 <= b_customer(12)(1)(8)#643 <= 19
 0 <= b_customer(12)(1)(9)#644 <= 18
 0 <= b_customer(12)(1)(10)#645 <= 18
 0 <= b_customer(12)(2)(1)#646 <= 21
 0 <= b_customer(12)(2)(2)#647 <= 22
 0 <= b_customer(12)(2)(3)#648 <= 21
 0 <= b_customer(12)(2)(4)#649 <= 23
 0 <= b_customer(12)(2)(5)#650 <= 19
 0 <= b_customer(12)(2)(6)#651 <= 19
 0 <= b_customer(12)(2)(7)#652 <= 23
 0 <= b_customer(12)(2)(8)#653 <= 26
 0 <= b_customer(12)(2)(9)#654 <= 25
 0 <= b_customer(12)(2)(10)#655 <= 18
 0 <= b_customer(12)(3)(1)#656 <= 25
 0 <= b_customer(12)(3)(2)#657 <= 25
 0 <= b_customer(12)(3)(3)#658 <= 21
 0 <= b_customer(12)(3)(4)#659 <= 19
 0 <= b_customer(12)(3)(5)#660 <= 19
 0 <= b_customer(12)(3)(6)#661 <= 26
 0 <= b_customer(12)(3)(7)#662 <= 25
 0 <= b_customer(12)(3)(8)#663 <= 22
 0 <= b_customer(12)(3)(9)#664 <= 21
 0 <= b_customer(12)(3)(10)#665 <= 25
 0 <= q(1)(1)(1)#666 <= 136.8
 0 <= q(1)(1)(2)#667 <= 136.8
 0 <= q(1)(1)(3)#668 <= 136.8
 0 <= q(1)(2)(1)#669 <= 136.8
 0 <= q(1)(2)(2)#670 <= 136.8
 0 <= q(1)(2)(3)#671 <= 136.8
 0 <= q(2)(1)(1)#672 <= 136.8
 0 <= q(2)(1)(2)#673 <= 136.8
 0 <= q(2)(1)(3)#674 <= 136.8
 0 <= q(2)(2)(1)#675 <= 136.8
 0 <= q(2)(2)(2)#676 <= 136.8
 0 <= q(2)(2)(3)#677 <= 136.8
 0 <= w(3)(1)(1)(1)#678 <= 10
 0 <= w(3)(1)(1)(2)#679 <= 12
 0 <= w(3)(1)(1)(3)#680 <= 11
 0 <= w(3)(1)(1)(4)#681 <= 11
 0 <= w(3)(1)(1)(5)#682 <= 9
 0 <= w(3)(1)(1)(6)#683 <= 12
 0 <= w(3)(1)(1)(7)#684 <= 12
 0 <= w(3)(1)(1)(8)#685 <= 10
 0 <= w(3)(1)(1)(9)#686 <= 12
 0 <= w(3)(1)(1)(10)#687 <= 11
 0 <= w(3)(1)(2)(1)#688 <= 9
 0 <= w(3)(1)(2)(2)#689 <= 11
 0 <= w(3)(1)(2)(3)#690 <= 11
 0 <= w(3)(1)(2)(4)#691 <= 11
 0 <= w(3)(1)(2)(5)#692 <= 12
 0 <= w(3)(1)(2)(6)#693 <= 10
 0 <= w(3)(1)(2)(7)#694 <= 11
 0 <= w(3)(1)(2)(8)#695 <= 10
 0 <= w(3)(1)(2)(9)#696 <= 10
 0 <= w(3)(1)(2)(10)#697 <= 10
 0 <= w(3)(1)(3)(1)#698 <= 12
 0 <= w(3)(1)(3)(2)#699 <= 9
 0 <= w(3)(1)(3)(3)#700 <= 11
 0 <= w(3)(1)(3)(4)#701 <= 11
 0 <= w(3)(1)(3)(5)#702 <= 9
 0 <= w(3)(1)(3)(6)#703 <= 9
 0 <= w(3)(1)(3)(7)#704 <= 10
 0 <= w(3)(1)(3)(8)#705 <= 8
 0 <= w(3)(1)(3)(9)#706 <= 8
 0 <= w(3)(1)(3)(10)#707 <= 11
 0 <= w(3)(2)(1)(1)#708 <= 10
 0 <= w(3)(2)(1)(2)#709 <= 12
 0 <= w(3)(2)(1)(3)#710 <= 11
 0 <= w(3)(2)(1)(4)#711 <= 11
 0 <= w(3)(2)(1)(5)#712 <= 9
 0 <= w(3)(2)(1)(6)#713 <= 12
 0 <= w(3)(2)(1)(7)#714 <= 12
 0 <= w(3)(2)(1)(8)#715 <= 10
 0 <= w(3)(2)(1)(9)#716 <= 12
 0 <= w(3)(2)(1)(10)#717 <= 11
 0 <= w(3)(2)(2)(1)#718 <= 9
 0 <= w(3)(2)(2)(2)#719 <= 11
 0 <= w(3)(2)(2)(3)#720 <= 11
 0 <= w(3)(2)(2)(4)#721 <= 11
 0 <= w(3)(2)(2)(5)#722 <= 12
 0 <= w(3)(2)(2)(6)#723 <= 10
 0 <= w(3)(2)(2)(7)#724 <= 11
 0 <= w(3)(2)(2)(8)#725 <= 10
 0 <= w(3)(2)(2)(9)#726 <= 10
 0 <= w(3)(2)(2)(10)#727 <= 10
 0 <= w(3)(2)(3)(1)#728 <= 12
 0 <= w(3)(2)(3)(2)#729 <= 9
 0 <= w(3)(2)(3)(3)#730 <= 11
 0 <= w(3)(2)(3)(4)#731 <= 11
 0 <= w(3)(2)(3)(5)#732 <= 9
 0 <= w(3)(2)(3)(6)#733 <= 9
 0 <= w(3)(2)(3)(7)#734 <= 10
 0 <= w(3)(2)(3)(8)#735 <= 8
 0 <= w(3)(2)(3)(9)#736 <= 8
 0 <= w(3)(2)(3)(10)#737 <= 11
 0 <= w(3)(3)(1)(1)#738 <= 10
 0 <= w(3)(3)(1)(2)#739 <= 12
 0 <= w(3)(3)(1)(3)#740 <= 11
 0 <= w(3)(3)(1)(4)#741 <= 11
 0 <= w(3)(3)(1)(5)#742 <= 9
 0 <= w(3)(3)(1)(6)#743 <= 12
 0 <= w(3)(3)(1)(7)#744 <= 12
 0 <= w(3)(3)(1)(8)#745 <= 10
 0 <= w(3)(3)(1)(9)#746 <= 12
 0 <= w(3)(3)(1)(10)#747 <= 11
 0 <= w(3)(3)(2)(1)#748 <= 9
 0 <= w(3)(3)(2)(2)#749 <= 11
 0 <= w(3)(3)(2)(3)#750 <= 11
 0 <= w(3)(3)(2)(4)#751 <= 11
 0 <= w(3)(3)(2)(5)#752 <= 12
 0 <= w(3)(3)(2)(6)#753 <= 10
 0 <= w(3)(3)(2)(7)#754 <= 11
 0 <= w(3)(3)(2)(8)#755 <= 10
 0 <= w(3)(3)(2)(9)#756 <= 10
 0 <= w(3)(3)(2)(10)#757 <= 10
 0 <= w(3)(3)(3)(1)#758 <= 12
 0 <= w(3)(3)(3)(2)#759 <= 9
 0 <= w(3)(3)(3)(3)#760 <= 11
 0 <= w(3)(3)(3)(4)#761 <= 11
 0 <= w(3)(3)(3)(5)#762 <= 9
 0 <= w(3)(3)(3)(6)#763 <= 9
 0 <= w(3)(3)(3)(7)#764 <= 10
 0 <= w(3)(3)(3)(8)#765 <= 8
 0 <= w(3)(3)(3)(9)#766 <= 8
 0 <= w(3)(3)(3)(10)#767 <= 11
 0 <= w(3)(4)(1)(1)#768 <= 10
 0 <= w(3)(4)(1)(2)#769 <= 12
 0 <= w(3)(4)(1)(3)#770 <= 11
 0 <= w(3)(4)(1)(4)#771 <= 11
 0 <= w(3)(4)(1)(5)#772 <= 9
 0 <= w(3)(4)(1)(6)#773 <= 12
 0 <= w(3)(4)(1)(7)#774 <= 12
 0 <= w(3)(4)(1)(8)#775 <= 10
 0 <= w(3)(4)(1)(9)#776 <= 12
 0 <= w(3)(4)(1)(10)#777 <= 11
 0 <= w(3)(4)(2)(1)#778 <= 9
 0 <= w(3)(4)(2)(2)#779 <= 11
 0 <= w(3)(4)(2)(3)#780 <= 11
 0 <= w(3)(4)(2)(4)#781 <= 11
 0 <= w(3)(4)(2)(5)#782 <= 12
 0 <= w(3)(4)(2)(6)#783 <= 10
 0 <= w(3)(4)(2)(7)#784 <= 11
 0 <= w(3)(4)(2)(8)#785 <= 10
 0 <= w(3)(4)(2)(9)#786 <= 10
 0 <= w(3)(4)(2)(10)#787 <= 10
 0 <= w(3)(4)(3)(1)#788 <= 12
 0 <= w(3)(4)(3)(2)#789 <= 9
 0 <= w(3)(4)(3)(3)#790 <= 11
 0 <= w(3)(4)(3)(4)#791 <= 11
 0 <= w(3)(4)(3)(5)#792 <= 9
 0 <= w(3)(4)(3)(6)#793 <= 9
 0 <= w(3)(4)(3)(7)#794 <= 10
 0 <= w(3)(4)(3)(8)#795 <= 8
 0 <= w(3)(4)(3)(9)#796 <= 8
 0 <= w(3)(4)(3)(10)#797 <= 11
 0 <= w(3)(5)(1)(1)#798 <= 10
 0 <= w(3)(5)(1)(2)#799 <= 12
 0 <= w(3)(5)(1)(3)#800 <= 11
 0 <= w(3)(5)(1)(4)#801 <= 11
 0 <= w(3)(5)(1)(5)#802 <= 9
 0 <= w(3)(5)(1)(6)#803 <= 12
 0 <= w(3)(5)(1)(7)#804 <= 12
 0 <= w(3)(5)(1)(8)#805 <= 10
 0 <= w(3)(5)(1)(9)#806 <= 12
 0 <= w(3)(5)(1)(10)#807 <= 11
 0 <= w(3)(5)(2)(1)#808 <= 9
 0 <= w(3)(5)(2)(2)#809 <= 11
 0 <= w(3)(5)(2)(3)#810 <= 11
 0 <= w(3)(5)(2)(4)#811 <= 11
 0 <= w(3)(5)(2)(5)#812 <= 12
 0 <= w(3)(5)(2)(6)#813 <= 10
 0 <= w(3)(5)(2)(7)#814 <= 11
 0 <= w(3)(5)(2)(8)#815 <= 10
 0 <= w(3)(5)(2)(9)#816 <= 10
 0 <= w(3)(5)(2)(10)#817 <= 10
 0 <= w(3)(5)(3)(1)#818 <= 12
 0 <= w(3)(5)(3)(2)#819 <= 9
 0 <= w(3)(5)(3)(3)#820 <= 11
 0 <= w(3)(5)(3)(4)#821 <= 11
 0 <= w(3)(5)(3)(5)#822 <= 9
 0 <= w(3)(5)(3)(6)#823 <= 9
 0 <= w(3)(5)(3)(7)#824 <= 10
 0 <= w(3)(5)(3)(8)#825 <= 8
 0 <= w(3)(5)(3)(9)#826 <= 8
 0 <= w(3)(5)(3)(10)#827 <= 11
 0 <= w(3)(6)(1)(1)#828 <= 10
 0 <= w(3)(6)(1)(2)#829 <= 12
 0 <= w(3)(6)(1)(3)#830 <= 11
 0 <= w(3)(6)(1)(4)#831 <= 11
 0 <= w(3)(6)(1)(5)#832 <= 9
 0 <= w(3)(6)(1)(6)#833 <= 12
 0 <= w(3)(6)(1)(7)#834 <= 12
 0 <= w(3)(6)(1)(8)#835 <= 10
 0 <= w(3)(6)(1)(9)#836 <= 12
 0 <= w(3)(6)(1)(10)#837 <= 11
 0 <= w(3)(6)(2)(1)#838 <= 9
 0 <= w(3)(6)(2)(2)#839 <= 11
 0 <= w(3)(6)(2)(3)#840 <= 11
 0 <= w(3)(6)(2)(4)#841 <= 11
 0 <= w(3)(6)(2)(5)#842 <= 12
 0 <= w(3)(6)(2)(6)#843 <= 10
 0 <= w(3)(6)(2)(7)#844 <= 11
 0 <= w(3)(6)(2)(8)#845 <= 10
 0 <= w(3)(6)(2)(9)#846 <= 10
 0 <= w(3)(6)(2)(10)#847 <= 10
 0 <= w(3)(6)(3)(1)#848 <= 12
 0 <= w(3)(6)(3)(2)#849 <= 9
 0 <= w(3)(6)(3)(3)#850 <= 11
 0 <= w(3)(6)(3)(4)#851 <= 11
 0 <= w(3)(6)(3)(5)#852 <= 9
 0 <= w(3)(6)(3)(6)#853 <= 9
 0 <= w(3)(6)(3)(7)#854 <= 10
 0 <= w(3)(6)(3)(8)#855 <= 8
 0 <= w(3)(6)(3)(9)#856 <= 8
 0 <= w(3)(6)(3)(10)#857 <= 11
 0 <= w(4)(1)(1)(1)#858 <= 18
 0 <= w(4)(1)(1)(2)#859 <= 12
 0 <= w(4)(1)(1)(3)#860 <= 17
 0 <= w(4)(1)(1)(4)#861 <= 18
 0 <= w(4)(1)(1)(5)#862 <= 17
 0 <= w(4)(1)(1)(6)#863 <= 13
 0 <= w(4)(1)(1)(7)#864 <= 13
 0 <= w(4)(1)(1)(8)#865 <= 13
 0 <= w(4)(1)(1)(9)#866 <= 18
 0 <= w(4)(1)(1)(10)#867 <= 14
 0 <= w(4)(1)(2)(1)#868 <= 12
 0 <= w(4)(1)(2)(2)#869 <= 16
 0 <= w(4)(1)(2)(3)#870 <= 17
 0 <= w(4)(1)(2)(4)#871 <= 15
 0 <= w(4)(1)(2)(5)#872 <= 17
 0 <= w(4)(1)(2)(6)#873 <= 13
 0 <= w(4)(1)(2)(7)#874 <= 17
 0 <= w(4)(1)(2)(8)#875 <= 13
 0 <= w(4)(1)(2)(9)#876 <= 15
 0 <= w(4)(1)(2)(10)#877 <= 14
 0 <= w(4)(1)(3)(1)#878 <= 17
 0 <= w(4)(1)(3)(2)#879 <= 14
 0 <= w(4)(1)(3)(3)#880 <= 15
 0 <= w(4)(1)(3)(4)#881 <= 13
 0 <= w(4)(1)(3)(5)#882 <= 14
 0 <= w(4)(1)(3)(6)#883 <= 16
 0 <= w(4)(1)(3)(7)#884 <= 15
 0 <= w(4)(1)(3)(8)#885 <= 16
 0 <= w(4)(1)(3)(9)#886 <= 18
 0 <= w(4)(1)(3)(10)#887 <= 14
 0 <= w(4)(2)(1)(1)#888 <= 18
 0 <= w(4)(2)(1)(2)#889 <= 12
 0 <= w(4)(2)(1)(3)#890 <= 17
 0 <= w(4)(2)(1)(4)#891 <= 18
 0 <= w(4)(2)(1)(5)#892 <= 17
 0 <= w(4)(2)(1)(6)#893 <= 13
 0 <= w(4)(2)(1)(7)#894 <= 13
 0 <= w(4)(2)(1)(8)#895 <= 13
 0 <= w(4)(2)(1)(9)#896 <= 18
 0 <= w(4)(2)(1)(10)#897 <= 14
 0 <= w(4)(2)(2)(1)#898 <= 12
 0 <= w(4)(2)(2)(2)#899 <= 16
 0 <= w(4)(2)(2)(3)#900 <= 17
 0 <= w(4)(2)(2)(4)#901 <= 15
 0 <= w(4)(2)(2)(5)#902 <= 17
 0 <= w(4)(2)(2)(6)#903 <= 13
 0 <= w(4)(2)(2)(7)#904 <= 17
 0 <= w(4)(2)(2)(8)#905 <= 13
 0 <= w(4)(2)(2)(9)#906 <= 15
 0 <= w(4)(2)(2)(10)#907 <= 14
 0 <= w(4)(2)(3)(1)#908 <= 17
 0 <= w(4)(2)(3)(2)#909 <= 14
 0 <= w(4)(2)(3)(3)#910 <= 15
 0 <= w(4)(2)(3)(4)#911 <= 13
 0 <= w(4)(2)(3)(5)#912 <= 14
 0 <= w(4)(2)(3)(6)#913 <= 16
 0 <= w(4)(2)(3)(7)#914 <= 15
 0 <= w(4)(2)(3)(8)#915 <= 16
 0 <= w(4)(2)(3)(9)#916 <= 18
 0 <= w(4)(2)(3)(10)#917 <= 14
 0 <= w(4)(3)(1)(1)#918 <= 18
 0 <= w(4)(3)(1)(2)#919 <= 12
 0 <= w(4)(3)(1)(3)#920 <= 17
 0 <= w(4)(3)(1)(4)#921 <= 18
 0 <= w(4)(3)(1)(5)#922 <= 17
 0 <= w(4)(3)(1)(6)#923 <= 13
 0 <= w(4)(3)(1)(7)#924 <= 13
 0 <= w(4)(3)(1)(8)#925 <= 13
 0 <= w(4)(3)(1)(9)#926 <= 18
 0 <= w(4)(3)(1)(10)#927 <= 14
 0 <= w(4)(3)(2)(1)#928 <= 12
 0 <= w(4)(3)(2)(2)#929 <= 16
 0 <= w(4)(3)(2)(3)#930 <= 17
 0 <= w(4)(3)(2)(4)#931 <= 15
 0 <= w(4)(3)(2)(5)#932 <= 17
 0 <= w(4)(3)(2)(6)#933 <= 13
 0 <= w(4)(3)(2)(7)#934 <= 17
 0 <= w(4)(3)(2)(8)#935 <= 13
 0 <= w(4)(3)(2)(9)#936 <= 15
 0 <= w(4)(3)(2)(10)#937 <= 14
 0 <= w(4)(3)(3)(1)#938 <= 17
 0 <= w(4)(3)(3)(2)#939 <= 14
 0 <= w(4)(3)(3)(3)#940 <= 15
 0 <= w(4)(3)(3)(4)#941 <= 13
 0 <= w(4)(3)(3)(5)#942 <= 14
 0 <= w(4)(3)(3)(6)#943 <= 16
 0 <= w(4)(3)(3)(7)#944 <= 15
 0 <= w(4)(3)(3)(8)#945 <= 16
 0 <= w(4)(3)(3)(9)#946 <= 18
 0 <= w(4)(3)(3)(10)#947 <= 14
 0 <= w(4)(4)(1)(1)#948 <= 18
 0 <= w(4)(4)(1)(2)#949 <= 12
 0 <= w(4)(4)(1)(3)#950 <= 17
 0 <= w(4)(4)(1)(4)#951 <= 18
 0 <= w(4)(4)(1)(5)#952 <= 17
 0 <= w(4)(4)(1)(6)#953 <= 13
 0 <= w(4)(4)(1)(7)#954 <= 13
 0 <= w(4)(4)(1)(8)#955 <= 13
 0 <= w(4)(4)(1)(9)#956 <= 18
 0 <= w(4)(4)(1)(10)#957 <= 14
 0 <= w(4)(4)(2)(1)#958 <= 12
 0 <= w(4)(4)(2)(2)#959 <= 16
 0 <= w(4)(4)(2)(3)#960 <= 17
 0 <= w(4)(4)(2)(4)#961 <= 15
 0 <= w(4)(4)(2)(5)#962 <= 17
 0 <= w(4)(4)(2)(6)#963 <= 13
 0 <= w(4)(4)(2)(7)#964 <= 17
 0 <= w(4)(4)(2)(8)#965 <= 13
 0 <= w(4)(4)(2)(9)#966 <= 15
 0 <= w(4)(4)(2)(10)#967 <= 14
 0 <= w(4)(4)(3)(1)#968 <= 17
 0 <= w(4)(4)(3)(2)#969 <= 14
 0 <= w(4)(4)(3)(3)#970 <= 15
 0 <= w(4)(4)(3)(4)#971 <= 13
 0 <= w(4)(4)(3)(5)#972 <= 14
 0 <= w(4)(4)(3)(6)#973 <= 16
 0 <= w(4)(4)(3)(7)#974 <= 15
 0 <= w(4)(4)(3)(8)#975 <= 16
 0 <= w(4)(4)(3)(9)#976 <= 18
 0 <= w(4)(4)(3)(10)#977 <= 14
 0 <= w(4)(5)(1)(1)#978 <= 18
 0 <= w(4)(5)(1)(2)#979 <= 12
 0 <= w(4)(5)(1)(3)#980 <= 17
 0 <= w(4)(5)(1)(4)#981 <= 18
 0 <= w(4)(5)(1)(5)#982 <= 17
 0 <= w(4)(5)(1)(6)#983 <= 13
 0 <= w(4)(5)(1)(7)#984 <= 13
 0 <= w(4)(5)(1)(8)#985 <= 13
 0 <= w(4)(5)(1)(9)#986 <= 18
 0 <= w(4)(5)(1)(10)#987 <= 14
 0 <= w(4)(5)(2)(1)#988 <= 12
 0 <= w(4)(5)(2)(2)#989 <= 16
 0 <= w(4)(5)(2)(3)#990 <= 17
 0 <= w(4)(5)(2)(4)#991 <= 15
 0 <= w(4)(5)(2)(5)#992 <= 17
 0 <= w(4)(5)(2)(6)#993 <= 13
 0 <= w(4)(5)(2)(7)#994 <= 17
 0 <= w(4)(5)(2)(8)#995 <= 13
 0 <= w(4)(5)(2)(9)#996 <= 15
 0 <= w(4)(5)(2)(10)#997 <= 14
 0 <= w(4)(5)(3)(1)#998 <= 17
 0 <= w(4)(5)(3)(2)#999 <= 14
 0 <= w(4)(5)(3)(3)#1000 <= 15
 0 <= w(4)(5)(3)(4)#1001 <= 13
 0 <= w(4)(5)(3)(5)#1002 <= 14
 0 <= w(4)(5)(3)(6)#1003 <= 16
 0 <= w(4)(5)(3)(7)#1004 <= 15
 0 <= w(4)(5)(3)(8)#1005 <= 16
 0 <= w(4)(5)(3)(9)#1006 <= 18
 0 <= w(4)(5)(3)(10)#1007 <= 14
 0 <= w(4)(6)(1)(1)#1008 <= 18
 0 <= w(4)(6)(1)(2)#1009 <= 12
 0 <= w(4)(6)(1)(3)#1010 <= 17
 0 <= w(4)(6)(1)(4)#1011 <= 18
 0 <= w(4)(6)(1)(5)#1012 <= 17
 0 <= w(4)(6)(1)(6)#1013 <= 13
 0 <= w(4)(6)(1)(7)#1014 <= 13
 0 <= w(4)(6)(1)(8)#1015 <= 13
 0 <= w(4)(6)(1)(9)#1016 <= 18
 0 <= w(4)(6)(1)(10)#1017 <= 14
 0 <= w(4)(6)(2)(1)#1018 <= 12
 0 <= w(4)(6)(2)(2)#1019 <= 16
 0 <= w(4)(6)(2)(3)#1020 <= 17
 0 <= w(4)(6)(2)(4)#1021 <= 15
 0 <= w(4)(6)(2)(5)#1022 <= 17
 0 <= w(4)(6)(2)(6)#1023 <= 13
 0 <= w(4)(6)(2)(7)#1024 <= 17
 0 <= w(4)(6)(2)(8)#1025 <= 13
 0 <= w(4)(6)(2)(9)#1026 <= 15
 0 <= w(4)(6)(2)(10)#1027 <= 14
 0 <= w(4)(6)(3)(1)#1028 <= 17
 0 <= w(4)(6)(3)(2)#1029 <= 14
 0 <= w(4)(6)(3)(3)#1030 <= 15
 0 <= w(4)(6)(3)(4)#1031 <= 13
 0 <= w(4)(6)(3)(5)#1032 <= 14
 0 <= w(4)(6)(3)(6)#1033 <= 16
 0 <= w(4)(6)(3)(7)#1034 <= 15
 0 <= w(4)(6)(3)(8)#1035 <= 16
 0 <= w(4)(6)(3)(9)#1036 <= 18
 0 <= w(4)(6)(3)(10)#1037 <= 14
 0 <= w(5)(1)(1)(1)#1038 <= 13
 0 <= w(5)(1)(1)(2)#1039 <= 12
 0 <= w(5)(1)(1)(3)#1040 <= 15
 0 <= w(5)(1)(1)(4)#1041 <= 17
 0 <= w(5)(1)(1)(5)#1042 <= 17
 0 <= w(5)(1)(1)(6)#1043 <= 14
 0 <= w(5)(1)(1)(7)#1044 <= 15
 0 <= w(5)(1)(1)(8)#1045 <= 13
 0 <= w(5)(1)(1)(9)#1046 <= 13
 0 <= w(5)(1)(1)(10)#1047 <= 15
 0 <= w(5)(1)(2)(1)#1048 <= 15
 0 <= w(5)(1)(2)(2)#1049 <= 17
 0 <= w(5)(1)(2)(3)#1050 <= 18
 0 <= w(5)(1)(2)(4)#1051 <= 14
 0 <= w(5)(1)(2)(5)#1052 <= 12
 0 <= w(5)(1)(2)(6)#1053 <= 17
 0 <= w(5)(1)(2)(7)#1054 <= 13
 0 <= w(5)(1)(2)(8)#1055 <= 13
 0 <= w(5)(1)(2)(9)#1056 <= 17
 0 <= w(5)(1)(2)(10)#1057 <= 13
 0 <= w(5)(1)(3)(1)#1058 <= 16
 0 <= w(5)(1)(3)(2)#1059 <= 17
 0 <= w(5)(1)(3)(3)#1060 <= 13
 0 <= w(5)(1)(3)(4)#1061 <= 16
 0 <= w(5)(1)(3)(5)#1062 <= 14
 0 <= w(5)(1)(3)(6)#1063 <= 13
 0 <= w(5)(1)(3)(7)#1064 <= 12
 0 <= w(5)(1)(3)(8)#1065 <= 17
 0 <= w(5)(1)(3)(9)#1066 <= 13
 0 <= w(5)(1)(3)(10)#1067 <= 16
 0 <= w(5)(2)(1)(1)#1068 <= 13
 0 <= w(5)(2)(1)(2)#1069 <= 12
 0 <= w(5)(2)(1)(3)#1070 <= 15
 0 <= w(5)(2)(1)(4)#1071 <= 17
 0 <= w(5)(2)(1)(5)#1072 <= 17
 0 <= w(5)(2)(1)(6)#1073 <= 14
 0 <= w(5)(2)(1)(7)#1074 <= 15
 0 <= w(5)(2)(1)(8)#1075 <= 13
 0 <= w(5)(2)(1)(9)#1076 <= 13
 0 <= w(5)(2)(1)(10)#1077 <= 15
 0 <= w(5)(2)(2)(1)#1078 <= 15
 0 <= w(5)(2)(2)(2)#1079 <= 17
 0 <= w(5)(2)(2)(3)#1080 <= 18
 0 <= w(5)(2)(2)(4)#1081 <= 14
 0 <= w(5)(2)(2)(5)#1082 <= 12
 0 <= w(5)(2)(2)(6)#1083 <= 17
 0 <= w(5)(2)(2)(7)#1084 <= 13
 0 <= w(5)(2)(2)(8)#1085 <= 13
 0 <= w(5)(2)(2)(9)#1086 <= 17
 0 <= w(5)(2)(2)(10)#1087 <= 13
 0 <= w(5)(2)(3)(1)#1088 <= 16
 0 <= w(5)(2)(3)(2)#1089 <= 17
 0 <= w(5)(2)(3)(3)#1090 <= 13
 0 <= w(5)(2)(3)(4)#1091 <= 16
 0 <= w(5)(2)(3)(5)#1092 <= 14
 0 <= w(5)(2)(3)(6)#1093 <= 13
 0 <= w(5)(2)(3)(7)#1094 <= 12
 0 <= w(5)(2)(3)(8)#1095 <= 17
 0 <= w(5)(2)(3)(9)#1096 <= 13
 0 <= w(5)(2)(3)(10)#1097 <= 16
 0 <= w(5)(3)(1)(1)#1098 <= 13
 0 <= w(5)(3)(1)(2)#1099 <= 12
 0 <= w(5)(3)(1)(3)#1100 <= 15
 0 <= w(5)(3)(1)(4)#1101 <= 17
 0 <= w(5)(3)(1)(5)#1102 <= 17
 0 <= w(5)(3)(1)(6)#1103 <= 14
 0 <= w(5)(3)(1)(7)#1104 <= 15
 0 <= w(5)(3)(1)(8)#1105 <= 13
 0 <= w(5)(3)(1)(9)#1106 <= 13
 0 <= w(5)(3)(1)(10)#1107 <= 15
 0 <= w(5)(3)(2)(1)#1108 <= 15
 0 <= w(5)(3)(2)(2)#1109 <= 17
 0 <= w(5)(3)(2)(3)#1110 <= 18
 0 <= w(5)(3)(2)(4)#1111 <= 14
 0 <= w(5)(3)(2)(5)#1112 <= 12
 0 <= w(5)(3)(2)(6)#1113 <= 17
 0 <= w(5)(3)(2)(7)#1114 <= 13
 0 <= w(5)(3)(2)(8)#1115 <= 13
 0 <= w(5)(3)(2)(9)#1116 <= 17
 0 <= w(5)(3)(2)(10)#1117 <= 13
 0 <= w(5)(3)(3)(1)#1118 <= 16
 0 <= w(5)(3)(3)(2)#1119 <= 17
 0 <= w(5)(3)(3)(3)#1120 <= 13
 0 <= w(5)(3)(3)(4)#1121 <= 16
 0 <= w(5)(3)(3)(5)#1122 <= 14
 0 <= w(5)(3)(3)(6)#1123 <= 13
 0 <= w(5)(3)(3)(7)#1124 <= 12
 0 <= w(5)(3)(3)(8)#1125 <= 17
 0 <= w(5)(3)(3)(9)#1126 <= 13
 0 <= w(5)(3)(3)(10)#1127 <= 16
 0 <= w(5)(4)(1)(1)#1128 <= 13
 0 <= w(5)(4)(1)(2)#1129 <= 12
 0 <= w(5)(4)(1)(3)#1130 <= 15
 0 <= w(5)(4)(1)(4)#1131 <= 17
 0 <= w(5)(4)(1)(5)#1132 <= 17
 0 <= w(5)(4)(1)(6)#1133 <= 14
 0 <= w(5)(4)(1)(7)#1134 <= 15
 0 <= w(5)(4)(1)(8)#1135 <= 13
 0 <= w(5)(4)(1)(9)#1136 <= 13
 0 <= w(5)(4)(1)(10)#1137 <= 15
 0 <= w(5)(4)(2)(1)#1138 <= 15
 0 <= w(5)(4)(2)(2)#1139 <= 17
 0 <= w(5)(4)(2)(3)#1140 <= 18
 0 <= w(5)(4)(2)(4)#1141 <= 14
 0 <= w(5)(4)(2)(5)#1142 <= 12
 0 <= w(5)(4)(2)(6)#1143 <= 17
 0 <= w(5)(4)(2)(7)#1144 <= 13
 0 <= w(5)(4)(2)(8)#1145 <= 13
 0 <= w(5)(4)(2)(9)#1146 <= 17
 0 <= w(5)(4)(2)(10)#1147 <= 13
 0 <= w(5)(4)(3)(1)#1148 <= 16
 0 <= w(5)(4)(3)(2)#1149 <= 17
 0 <= w(5)(4)(3)(3)#1150 <= 13
 0 <= w(5)(4)(3)(4)#1151 <= 16
 0 <= w(5)(4)(3)(5)#1152 <= 14
 0 <= w(5)(4)(3)(6)#1153 <= 13
 0 <= w(5)(4)(3)(7)#1154 <= 12
 0 <= w(5)(4)(3)(8)#1155 <= 17
 0 <= w(5)(4)(3)(9)#1156 <= 13
 0 <= w(5)(4)(3)(10)#1157 <= 16
 0 <= w(5)(5)(1)(1)#1158 <= 13
 0 <= w(5)(5)(1)(2)#1159 <= 12
 0 <= w(5)(5)(1)(3)#1160 <= 15
 0 <= w(5)(5)(1)(4)#1161 <= 17
 0 <= w(5)(5)(1)(5)#1162 <= 17
 0 <= w(5)(5)(1)(6)#1163 <= 14
 0 <= w(5)(5)(1)(7)#1164 <= 15
 0 <= w(5)(5)(1)(8)#1165 <= 13
 0 <= w(5)(5)(1)(9)#1166 <= 13
 0 <= w(5)(5)(1)(10)#1167 <= 15
 0 <= w(5)(5)(2)(1)#1168 <= 15
 0 <= w(5)(5)(2)(2)#1169 <= 17
 0 <= w(5)(5)(2)(3)#1170 <= 18
 0 <= w(5)(5)(2)(4)#1171 <= 14
 0 <= w(5)(5)(2)(5)#1172 <= 12
 0 <= w(5)(5)(2)(6)#1173 <= 17
 0 <= w(5)(5)(2)(7)#1174 <= 13
 0 <= w(5)(5)(2)(8)#1175 <= 13
 0 <= w(5)(5)(2)(9)#1176 <= 17
 0 <= w(5)(5)(2)(10)#1177 <= 13
 0 <= w(5)(5)(3)(1)#1178 <= 16
 0 <= w(5)(5)(3)(2)#1179 <= 17
 0 <= w(5)(5)(3)(3)#1180 <= 13
 0 <= w(5)(5)(3)(4)#1181 <= 16
 0 <= w(5)(5)(3)(5)#1182 <= 14
 0 <= w(5)(5)(3)(6)#1183 <= 13
 0 <= w(5)(5)(3)(7)#1184 <= 12
 0 <= w(5)(5)(3)(8)#1185 <= 17
 0 <= w(5)(5)(3)(9)#1186 <= 13
 0 <= w(5)(5)(3)(10)#1187 <= 16
 0 <= w(5)(6)(1)(1)#1188 <= 13
 0 <= w(5)(6)(1)(2)#1189 <= 12
 0 <= w(5)(6)(1)(3)#1190 <= 15
 0 <= w(5)(6)(1)(4)#1191 <= 17
 0 <= w(5)(6)(1)(5)#1192 <= 17
 0 <= w(5)(6)(1)(6)#1193 <= 14
 0 <= w(5)(6)(1)(7)#1194 <= 15
 0 <= w(5)(6)(1)(8)#1195 <= 13
 0 <= w(5)(6)(1)(9)#1196 <= 13
 0 <= w(5)(6)(1)(10)#1197 <= 15
 0 <= w(5)(6)(2)(1)#1198 <= 15
 0 <= w(5)(6)(2)(2)#1199 <= 17
 0 <= w(5)(6)(2)(3)#1200 <= 18
 0 <= w(5)(6)(2)(4)#1201 <= 14
 0 <= w(5)(6)(2)(5)#1202 <= 12
 0 <= w(5)(6)(2)(6)#1203 <= 17
 0 <= w(5)(6)(2)(7)#1204 <= 13
 0 <= w(5)(6)(2)(8)#1205 <= 13
 0 <= w(5)(6)(2)(9)#1206 <= 17
 0 <= w(5)(6)(2)(10)#1207 <= 13
 0 <= w(5)(6)(3)(1)#1208 <= 16
 0 <= w(5)(6)(3)(2)#1209 <= 17
 0 <= w(5)(6)(3)(3)#1210 <= 13
 0 <= w(5)(6)(3)(4)#1211 <= 16
 0 <= w(5)(6)(3)(5)#1212 <= 14
 0 <= w(5)(6)(3)(6)#1213 <= 13
 0 <= w(5)(6)(3)(7)#1214 <= 12
 0 <= w(5)(6)(3)(8)#1215 <= 17
 0 <= w(5)(6)(3)(9)#1216 <= 13
 0 <= w(5)(6)(3)(10)#1217 <= 16
 0 <= w(6)(1)(1)(1)#1218 <= 6
 0 <= w(6)(1)(1)(2)#1219 <= 8
 0 <= w(6)(1)(1)(3)#1220 <= 7
 0 <= w(6)(1)(1)(4)#1221 <= 7
 0 <= w(6)(1)(1)(5)#1222 <= 8
 0 <= w(6)(1)(1)(6)#1223 <= 8
 0 <= w(6)(1)(1)(7)#1224 <= 8
 0 <= w(6)(1)(1)(8)#1225 <= 7
 0 <= w(6)(1)(1)(9)#1226 <= 7
 0 <= w(6)(1)(1)(10)#1227 <= 7
 0 <= w(6)(1)(2)(1)#1228 <= 7
 0 <= w(6)(1)(2)(2)#1229 <= 7
 0 <= w(6)(1)(2)(3)#1230 <= 6
 0 <= w(6)(1)(2)(4)#1231 <= 6
 0 <= w(6)(1)(2)(5)#1232 <= 7
 0 <= w(6)(1)(2)(6)#1233 <= 6
 0 <= w(6)(1)(2)(7)#1234 <= 6
 0 <= w(6)(1)(2)(8)#1235 <= 8
 0 <= w(6)(1)(2)(9)#1236 <= 7
 0 <= w(6)(1)(2)(10)#1237 <= 8
 0 <= w(6)(1)(3)(1)#1238 <= 7
 0 <= w(6)(1)(3)(2)#1239 <= 8
 0 <= w(6)(1)(3)(3)#1240 <= 7
 0 <= w(6)(1)(3)(4)#1241 <= 6
 0 <= w(6)(1)(3)(5)#1242 <= 8
 0 <= w(6)(1)(3)(6)#1243 <= 6
 0 <= w(6)(1)(3)(7)#1244 <= 7
 0 <= w(6)(1)(3)(8)#1245 <= 7
 0 <= w(6)(1)(3)(9)#1246 <= 6
 0 <= w(6)(1)(3)(10)#1247 <= 8
 0 <= w(6)(2)(1)(1)#1248 <= 6
 0 <= w(6)(2)(1)(2)#1249 <= 8
 0 <= w(6)(2)(1)(3)#1250 <= 7
 0 <= w(6)(2)(1)(4)#1251 <= 7
 0 <= w(6)(2)(1)(5)#1252 <= 8
 0 <= w(6)(2)(1)(6)#1253 <= 8
 0 <= w(6)(2)(1)(7)#1254 <= 8
 0 <= w(6)(2)(1)(8)#1255 <= 7
 0 <= w(6)(2)(1)(9)#1256 <= 7
 0 <= w(6)(2)(1)(10)#1257 <= 7
 0 <= w(6)(2)(2)(1)#1258 <= 7
 0 <= w(6)(2)(2)(2)#1259 <= 7
 0 <= w(6)(2)(2)(3)#1260 <= 6
 0 <= w(6)(2)(2)(4)#1261 <= 6
 0 <= w(6)(2)(2)(5)#1262 <= 7
 0 <= w(6)(2)(2)(6)#1263 <= 6
 0 <= w(6)(2)(2)(7)#1264 <= 6
 0 <= w(6)(2)(2)(8)#1265 <= 8
 0 <= w(6)(2)(2)(9)#1266 <= 7
 0 <= w(6)(2)(2)(10)#1267 <= 8
 0 <= w(6)(2)(3)(1)#1268 <= 7
 0 <= w(6)(2)(3)(2)#1269 <= 8
 0 <= w(6)(2)(3)(3)#1270 <= 7
 0 <= w(6)(2)(3)(4)#1271 <= 6
 0 <= w(6)(2)(3)(5)#1272 <= 8
 0 <= w(6)(2)(3)(6)#1273 <= 6
 0 <= w(6)(2)(3)(7)#1274 <= 7
 0 <= w(6)(2)(3)(8)#1275 <= 7
 0 <= w(6)(2)(3)(9)#1276 <= 6
 0 <= w(6)(2)(3)(10)#1277 <= 8
 0 <= w(6)(3)(1)(1)#1278 <= 6
 0 <= w(6)(3)(1)(2)#1279 <= 8
 0 <= w(6)(3)(1)(3)#1280 <= 7
 0 <= w(6)(3)(1)(4)#1281 <= 7
 0 <= w(6)(3)(1)(5)#1282 <= 8
 0 <= w(6)(3)(1)(6)#1283 <= 8
 0 <= w(6)(3)(1)(7)#1284 <= 8
 0 <= w(6)(3)(1)(8)#1285 <= 7
 0 <= w(6)(3)(1)(9)#1286 <= 7
 0 <= w(6)(3)(1)(10)#1287 <= 7
 0 <= w(6)(3)(2)(1)#1288 <= 7
 0 <= w(6)(3)(2)(2)#1289 <= 7
 0 <= w(6)(3)(2)(3)#1290 <= 6
 0 <= w(6)(3)(2)(4)#1291 <= 6
 0 <= w(6)(3)(2)(5)#1292 <= 7
 0 <= w(6)(3)(2)(6)#1293 <= 6
 0 <= w(6)(3)(2)(7)#1294 <= 6
 0 <= w(6)(3)(2)(8)#1295 <= 8
 0 <= w(6)(3)(2)(9)#1296 <= 7
 0 <= w(6)(3)(2)(10)#1297 <= 8
 0 <= w(6)(3)(3)(1)#1298 <= 7
 0 <= w(6)(3)(3)(2)#1299 <= 8
 0 <= w(6)(3)(3)(3)#1300 <= 7
 0 <= w(6)(3)(3)(4)#1301 <= 6
 0 <= w(6)(3)(3)(5)#1302 <= 8
 0 <= w(6)(3)(3)(6)#1303 <= 6
 0 <= w(6)(3)(3)(7)#1304 <= 7
 0 <= w(6)(3)(3)(8)#1305 <= 7
 0 <= w(6)(3)(3)(9)#1306 <= 6
 0 <= w(6)(3)(3)(10)#1307 <= 8
 0 <= w(6)(4)(1)(1)#1308 <= 6
 0 <= w(6)(4)(1)(2)#1309 <= 8
 0 <= w(6)(4)(1)(3)#1310 <= 7
 0 <= w(6)(4)(1)(4)#1311 <= 7
 0 <= w(6)(4)(1)(5)#1312 <= 8
 0 <= w(6)(4)(1)(6)#1313 <= 8
 0 <= w(6)(4)(1)(7)#1314 <= 8
 0 <= w(6)(4)(1)(8)#1315 <= 7
 0 <= w(6)(4)(1)(9)#1316 <= 7
 0 <= w(6)(4)(1)(10)#1317 <= 7
 0 <= w(6)(4)(2)(1)#1318 <= 7
 0 <= w(6)(4)(2)(2)#1319 <= 7
 0 <= w(6)(4)(2)(3)#1320 <= 6
 0 <= w(6)(4)(2)(4)#1321 <= 6
 0 <= w(6)(4)(2)(5)#1322 <= 7
 0 <= w(6)(4)(2)(6)#1323 <= 6
 0 <= w(6)(4)(2)(7)#1324 <= 6
 0 <= w(6)(4)(2)(8)#1325 <= 8
 0 <= w(6)(4)(2)(9)#1326 <= 7
 0 <= w(6)(4)(2)(10)#1327 <= 8
 0 <= w(6)(4)(3)(1)#1328 <= 7
 0 <= w(6)(4)(3)(2)#1329 <= 8
 0 <= w(6)(4)(3)(3)#1330 <= 7
 0 <= w(6)(4)(3)(4)#1331 <= 6
 0 <= w(6)(4)(3)(5)#1332 <= 8
 0 <= w(6)(4)(3)(6)#1333 <= 6
 0 <= w(6)(4)(3)(7)#1334 <= 7
 0 <= w(6)(4)(3)(8)#1335 <= 7
 0 <= w(6)(4)(3)(9)#1336 <= 6
 0 <= w(6)(4)(3)(10)#1337 <= 8
 0 <= w(6)(5)(1)(1)#1338 <= 6
 0 <= w(6)(5)(1)(2)#1339 <= 8
 0 <= w(6)(5)(1)(3)#1340 <= 7
 0 <= w(6)(5)(1)(4)#1341 <= 7
 0 <= w(6)(5)(1)(5)#1342 <= 8
 0 <= w(6)(5)(1)(6)#1343 <= 8
 0 <= w(6)(5)(1)(7)#1344 <= 8
 0 <= w(6)(5)(1)(8)#1345 <= 7
 0 <= w(6)(5)(1)(9)#1346 <= 7
 0 <= w(6)(5)(1)(10)#1347 <= 7
 0 <= w(6)(5)(2)(1)#1348 <= 7
 0 <= w(6)(5)(2)(2)#1349 <= 7
 0 <= w(6)(5)(2)(3)#1350 <= 6
 0 <= w(6)(5)(2)(4)#1351 <= 6
 0 <= w(6)(5)(2)(5)#1352 <= 7
 0 <= w(6)(5)(2)(6)#1353 <= 6
 0 <= w(6)(5)(2)(7)#1354 <= 6
 0 <= w(6)(5)(2)(8)#1355 <= 8
 0 <= w(6)(5)(2)(9)#1356 <= 7
 0 <= w(6)(5)(2)(10)#1357 <= 8
 0 <= w(6)(5)(3)(1)#1358 <= 7
 0 <= w(6)(5)(3)(2)#1359 <= 8
 0 <= w(6)(5)(3)(3)#1360 <= 7
 0 <= w(6)(5)(3)(4)#1361 <= 6
 0 <= w(6)(5)(3)(5)#1362 <= 8
 0 <= w(6)(5)(3)(6)#1363 <= 6
 0 <= w(6)(5)(3)(7)#1364 <= 7
 0 <= w(6)(5)(3)(8)#1365 <= 7
 0 <= w(6)(5)(3)(9)#1366 <= 6
 0 <= w(6)(5)(3)(10)#1367 <= 8
 0 <= w(6)(6)(1)(1)#1368 <= 6
 0 <= w(6)(6)(1)(2)#1369 <= 8
 0 <= w(6)(6)(1)(3)#1370 <= 7
 0 <= w(6)(6)(1)(4)#1371 <= 7
 0 <= w(6)(6)(1)(5)#1372 <= 8
 0 <= w(6)(6)(1)(6)#1373 <= 8
 0 <= w(6)(6)(1)(7)#1374 <= 8
 0 <= w(6)(6)(1)(8)#1375 <= 7
 0 <= w(6)(6)(1)(9)#1376 <= 7
 0 <= w(6)(6)(1)(10)#1377 <= 7
 0 <= w(6)(6)(2)(1)#1378 <= 7
 0 <= w(6)(6)(2)(2)#1379 <= 7
 0 <= w(6)(6)(2)(3)#1380 <= 6
 0 <= w(6)(6)(2)(4)#1381 <= 6
 0 <= w(6)(6)(2)(5)#1382 <= 7
 0 <= w(6)(6)(2)(6)#1383 <= 6
 0 <= w(6)(6)(2)(7)#1384 <= 6
 0 <= w(6)(6)(2)(8)#1385 <= 8
 0 <= w(6)(6)(2)(9)#1386 <= 7
 0 <= w(6)(6)(2)(10)#1387 <= 8
 0 <= w(6)(6)(3)(1)#1388 <= 7
 0 <= w(6)(6)(3)(2)#1389 <= 8
 0 <= w(6)(6)(3)(3)#1390 <= 7
 0 <= w(6)(6)(3)(4)#1391 <= 6
 0 <= w(6)(6)(3)(5)#1392 <= 8
 0 <= w(6)(6)(3)(6)#1393 <= 6
 0 <= w(6)(6)(3)(7)#1394 <= 7
 0 <= w(6)(6)(3)(8)#1395 <= 7
 0 <= w(6)(6)(3)(9)#1396 <= 6
 0 <= w(6)(6)(3)(10)#1397 <= 8
 0 <= w(7)(1)(1)(1)#1398 <= 15
 0 <= w(7)(1)(1)(2)#1399 <= 11
 0 <= w(7)(1)(1)(3)#1400 <= 14
 0 <= w(7)(1)(1)(4)#1401 <= 14
 0 <= w(7)(1)(1)(5)#1402 <= 14
 0 <= w(7)(1)(1)(6)#1403 <= 14
 0 <= w(7)(1)(1)(7)#1404 <= 12
 0 <= w(7)(1)(1)(8)#1405 <= 15
 0 <= w(7)(1)(1)(9)#1406 <= 14
 0 <= w(7)(1)(1)(10)#1407 <= 15
 0 <= w(7)(1)(2)(1)#1408 <= 15
 0 <= w(7)(1)(2)(2)#1409 <= 13
 0 <= w(7)(1)(2)(3)#1410 <= 13
 0 <= w(7)(1)(2)(4)#1411 <= 11
 0 <= w(7)(1)(2)(5)#1412 <= 12
 0 <= w(7)(1)(2)(6)#1413 <= 11
 0 <= w(7)(1)(2)(7)#1414 <= 13
 0 <= w(7)(1)(2)(8)#1415 <= 13
 0 <= w(7)(1)(2)(9)#1416 <= 13
 0 <= w(7)(1)(2)(10)#1417 <= 13
 0 <= w(7)(1)(3)(1)#1418 <= 12
 0 <= w(7)(1)(3)(2)#1419 <= 11
 0 <= w(7)(1)(3)(3)#1420 <= 13
 0 <= w(7)(1)(3)(4)#1421 <= 11
 0 <= w(7)(1)(3)(5)#1422 <= 12
 0 <= w(7)(1)(3)(6)#1423 <= 14
 0 <= w(7)(1)(3)(7)#1424 <= 14
 0 <= w(7)(1)(3)(8)#1425 <= 13
 0 <= w(7)(1)(3)(9)#1426 <= 13
 0 <= w(7)(1)(3)(10)#1427 <= 12
 0 <= w(7)(2)(1)(1)#1428 <= 15
 0 <= w(7)(2)(1)(2)#1429 <= 11
 0 <= w(7)(2)(1)(3)#1430 <= 14
 0 <= w(7)(2)(1)(4)#1431 <= 14
 0 <= w(7)(2)(1)(5)#1432 <= 14
 0 <= w(7)(2)(1)(6)#1433 <= 14
 0 <= w(7)(2)(1)(7)#1434 <= 12
 0 <= w(7)(2)(1)(8)#1435 <= 15
 0 <= w(7)(2)(1)(9)#1436 <= 14
 0 <= w(7)(2)(1)(10)#1437 <= 15
 0 <= w(7)(2)(2)(1)#1438 <= 15
 0 <= w(7)(2)(2)(2)#1439 <= 13
 0 <= w(7)(2)(2)(3)#1440 <= 13
 0 <= w(7)(2)(2)(4)#1441 <= 11
 0 <= w(7)(2)(2)(5)#1442 <= 12
 0 <= w(7)(2)(2)(6)#1443 <= 11
 0 <= w(7)(2)(2)(7)#1444 <= 13
 0 <= w(7)(2)(2)(8)#1445 <= 13
 0 <= w(7)(2)(2)(9)#1446 <= 13
 0 <= w(7)(2)(2)(10)#1447 <= 13
 0 <= w(7)(2)(3)(1)#1448 <= 12
 0 <= w(7)(2)(3)(2)#1449 <= 11
 0 <= w(7)(2)(3)(3)#1450 <= 13
 0 <= w(7)(2)(3)(4)#1451 <= 11
 0 <= w(7)(2)(3)(5)#1452 <= 12
 0 <= w(7)(2)(3)(6)#1453 <= 14
 0 <= w(7)(2)(3)(7)#1454 <= 14
 0 <= w(7)(2)(3)(8)#1455 <= 13
 0 <= w(7)(2)(3)(9)#1456 <= 13
 0 <= w(7)(2)(3)(10)#1457 <= 12
 0 <= w(7)(3)(1)(1)#1458 <= 15
 0 <= w(7)(3)(1)(2)#1459 <= 11
 0 <= w(7)(3)(1)(3)#1460 <= 14
 0 <= w(7)(3)(1)(4)#1461 <= 14
 0 <= w(7)(3)(1)(5)#1462 <= 14
 0 <= w(7)(3)(1)(6)#1463 <= 14
 0 <= w(7)(3)(1)(7)#1464 <= 12
 0 <= w(7)(3)(1)(8)#1465 <= 15
 0 <= w(7)(3)(1)(9)#1466 <= 14
 0 <= w(7)(3)(1)(10)#1467 <= 15
 0 <= w(7)(3)(2)(1)#1468 <= 15
 0 <= w(7)(3)(2)(2)#1469 <= 13
 0 <= w(7)(3)(2)(3)#1470 <= 13
 0 <= w(7)(3)(2)(4)#1471 <= 11
 0 <= w(7)(3)(2)(5)#1472 <= 12
 0 <= w(7)(3)(2)(6)#1473 <= 11
 0 <= w(7)(3)(2)(7)#1474 <= 13
 0 <= w(7)(3)(2)(8)#1475 <= 13
 0 <= w(7)(3)(2)(9)#1476 <= 13
 0 <= w(7)(3)(2)(10)#1477 <= 13
 0 <= w(7)(3)(3)(1)#1478 <= 12
 0 <= w(7)(3)(3)(2)#1479 <= 11
 0 <= w(7)(3)(3)(3)#1480 <= 13
 0 <= w(7)(3)(3)(4)#1481 <= 11
 0 <= w(7)(3)(3)(5)#1482 <= 12
 0 <= w(7)(3)(3)(6)#1483 <= 14
 0 <= w(7)(3)(3)(7)#1484 <= 14
 0 <= w(7)(3)(3)(8)#1485 <= 13
 0 <= w(7)(3)(3)(9)#1486 <= 13
 0 <= w(7)(3)(3)(10)#1487 <= 12
 0 <= w(7)(4)(1)(1)#1488 <= 15
 0 <= w(7)(4)(1)(2)#1489 <= 11
 0 <= w(7)(4)(1)(3)#1490 <= 14
 0 <= w(7)(4)(1)(4)#1491 <= 14
 0 <= w(7)(4)(1)(5)#1492 <= 14
 0 <= w(7)(4)(1)(6)#1493 <= 14
 0 <= w(7)(4)(1)(7)#1494 <= 12
 0 <= w(7)(4)(1)(8)#1495 <= 15
 0 <= w(7)(4)(1)(9)#1496 <= 14
 0 <= w(7)(4)(1)(10)#1497 <= 15
 0 <= w(7)(4)(2)(1)#1498 <= 15
 0 <= w(7)(4)(2)(2)#1499 <= 13
 0 <= w(7)(4)(2)(3)#1500 <= 13
 0 <= w(7)(4)(2)(4)#1501 <= 11
 0 <= w(7)(4)(2)(5)#1502 <= 12
 0 <= w(7)(4)(2)(6)#1503 <= 11
 0 <= w(7)(4)(2)(7)#1504 <= 13
 0 <= w(7)(4)(2)(8)#1505 <= 13
 0 <= w(7)(4)(2)(9)#1506 <= 13
 0 <= w(7)(4)(2)(10)#1507 <= 13
 0 <= w(7)(4)(3)(1)#1508 <= 12
 0 <= w(7)(4)(3)(2)#1509 <= 11
 0 <= w(7)(4)(3)(3)#1510 <= 13
 0 <= w(7)(4)(3)(4)#1511 <= 11
 0 <= w(7)(4)(3)(5)#1512 <= 12
 0 <= w(7)(4)(3)(6)#1513 <= 14
 0 <= w(7)(4)(3)(7)#1514 <= 14
 0 <= w(7)(4)(3)(8)#1515 <= 13
 0 <= w(7)(4)(3)(9)#1516 <= 13
 0 <= w(7)(4)(3)(10)#1517 <= 12
 0 <= w(7)(5)(1)(1)#1518 <= 15
 0 <= w(7)(5)(1)(2)#1519 <= 11
 0 <= w(7)(5)(1)(3)#1520 <= 14
 0 <= w(7)(5)(1)(4)#1521 <= 14
 0 <= w(7)(5)(1)(5)#1522 <= 14
 0 <= w(7)(5)(1)(6)#1523 <= 14
 0 <= w(7)(5)(1)(7)#1524 <= 12
 0 <= w(7)(5)(1)(8)#1525 <= 15
 0 <= w(7)(5)(1)(9)#1526 <= 14
 0 <= w(7)(5)(1)(10)#1527 <= 15
 0 <= w(7)(5)(2)(1)#1528 <= 15
 0 <= w(7)(5)(2)(2)#1529 <= 13
 0 <= w(7)(5)(2)(3)#1530 <= 13
 0 <= w(7)(5)(2)(4)#1531 <= 11
 0 <= w(7)(5)(2)(5)#1532 <= 12
 0 <= w(7)(5)(2)(6)#1533 <= 11
 0 <= w(7)(5)(2)(7)#1534 <= 13
 0 <= w(7)(5)(2)(8)#1535 <= 13
 0 <= w(7)(5)(2)(9)#1536 <= 13
 0 <= w(7)(5)(2)(10)#1537 <= 13
 0 <= w(7)(5)(3)(1)#1538 <= 12
 0 <= w(7)(5)(3)(2)#1539 <= 11
 0 <= w(7)(5)(3)(3)#1540 <= 13
 0 <= w(7)(5)(3)(4)#1541 <= 11
 0 <= w(7)(5)(3)(5)#1542 <= 12
 0 <= w(7)(5)(3)(6)#1543 <= 14
 0 <= w(7)(5)(3)(7)#1544 <= 14
 0 <= w(7)(5)(3)(8)#1545 <= 13
 0 <= w(7)(5)(3)(9)#1546 <= 13
 0 <= w(7)(5)(3)(10)#1547 <= 12
 0 <= w(7)(6)(1)(1)#1548 <= 15
 0 <= w(7)(6)(1)(2)#1549 <= 11
 0 <= w(7)(6)(1)(3)#1550 <= 14
 0 <= w(7)(6)(1)(4)#1551 <= 14
 0 <= w(7)(6)(1)(5)#1552 <= 14
 0 <= w(7)(6)(1)(6)#1553 <= 14
 0 <= w(7)(6)(1)(7)#1554 <= 12
 0 <= w(7)(6)(1)(8)#1555 <= 15
 0 <= w(7)(6)(1)(9)#1556 <= 14
 0 <= w(7)(6)(1)(10)#1557 <= 15
 0 <= w(7)(6)(2)(1)#1558 <= 15
 0 <= w(7)(6)(2)(2)#1559 <= 13
 0 <= w(7)(6)(2)(3)#1560 <= 13
 0 <= w(7)(6)(2)(4)#1561 <= 11
 0 <= w(7)(6)(2)(5)#1562 <= 12
 0 <= w(7)(6)(2)(6)#1563 <= 11
 0 <= w(7)(6)(2)(7)#1564 <= 13
 0 <= w(7)(6)(2)(8)#1565 <= 13
 0 <= w(7)(6)(2)(9)#1566 <= 13
 0 <= w(7)(6)(2)(10)#1567 <= 13
 0 <= w(7)(6)(3)(1)#1568 <= 12
 0 <= w(7)(6)(3)(2)#1569 <= 11
 0 <= w(7)(6)(3)(3)#1570 <= 13
 0 <= w(7)(6)(3)(4)#1571 <= 11
 0 <= w(7)(6)(3)(5)#1572 <= 12
 0 <= w(7)(6)(3)(6)#1573 <= 14
 0 <= w(7)(6)(3)(7)#1574 <= 14
 0 <= w(7)(6)(3)(8)#1575 <= 13
 0 <= w(7)(6)(3)(9)#1576 <= 13
 0 <= w(7)(6)(3)(10)#1577 <= 12
 0 <= w(8)(1)(1)(1)#1578 <= 13
 0 <= w(8)(1)(1)(2)#1579 <= 16
 0 <= w(8)(1)(1)(3)#1580 <= 18
 0 <= w(8)(1)(1)(4)#1581 <= 14
 0 <= w(8)(1)(1)(5)#1582 <= 19
 0 <= w(8)(1)(1)(6)#1583 <= 15
 0 <= w(8)(1)(1)(7)#1584 <= 18
 0 <= w(8)(1)(1)(8)#1585 <= 13
 0 <= w(8)(1)(1)(9)#1586 <= 17
 0 <= w(8)(1)(1)(10)#1587 <= 16
 0 <= w(8)(1)(2)(1)#1588 <= 18
 0 <= w(8)(1)(2)(2)#1589 <= 15
 0 <= w(8)(1)(2)(3)#1590 <= 17
 0 <= w(8)(1)(2)(4)#1591 <= 15
 0 <= w(8)(1)(2)(5)#1592 <= 14
 0 <= w(8)(1)(2)(6)#1593 <= 19
 0 <= w(8)(1)(2)(7)#1594 <= 18
 0 <= w(8)(1)(2)(8)#1595 <= 17
 0 <= w(8)(1)(2)(9)#1596 <= 15
 0 <= w(8)(1)(2)(10)#1597 <= 15
 0 <= w(8)(1)(3)(1)#1598 <= 17
 0 <= w(8)(1)(3)(2)#1599 <= 17
 0 <= w(8)(1)(3)(3)#1600 <= 19
 0 <= w(8)(1)(3)(4)#1601 <= 15
 0 <= w(8)(1)(3)(5)#1602 <= 14
 0 <= w(8)(1)(3)(6)#1603 <= 15
 0 <= w(8)(1)(3)(7)#1604 <= 14
 0 <= w(8)(1)(3)(8)#1605 <= 19
 0 <= w(8)(1)(3)(9)#1606 <= 13
 0 <= w(8)(1)(3)(10)#1607 <= 15
 0 <= w(8)(2)(1)(1)#1608 <= 13
 0 <= w(8)(2)(1)(2)#1609 <= 16
 0 <= w(8)(2)(1)(3)#1610 <= 18
 0 <= w(8)(2)(1)(4)#1611 <= 14
 0 <= w(8)(2)(1)(5)#1612 <= 19
 0 <= w(8)(2)(1)(6)#1613 <= 15
 0 <= w(8)(2)(1)(7)#1614 <= 18
 0 <= w(8)(2)(1)(8)#1615 <= 13
 0 <= w(8)(2)(1)(9)#1616 <= 17
 0 <= w(8)(2)(1)(10)#1617 <= 16
 0 <= w(8)(2)(2)(1)#1618 <= 18
 0 <= w(8)(2)(2)(2)#1619 <= 15
 0 <= w(8)(2)(2)(3)#1620 <= 17
 0 <= w(8)(2)(2)(4)#1621 <= 15
 0 <= w(8)(2)(2)(5)#1622 <= 14
 0 <= w(8)(2)(2)(6)#1623 <= 19
 0 <= w(8)(2)(2)(7)#1624 <= 18
 0 <= w(8)(2)(2)(8)#1625 <= 17
 0 <= w(8)(2)(2)(9)#1626 <= 15
 0 <= w(8)(2)(2)(10)#1627 <= 15
 0 <= w(8)(2)(3)(1)#1628 <= 17
 0 <= w(8)(2)(3)(2)#1629 <= 17
 0 <= w(8)(2)(3)(3)#1630 <= 19
 0 <= w(8)(2)(3)(4)#1631 <= 15
 0 <= w(8)(2)(3)(5)#1632 <= 14
 0 <= w(8)(2)(3)(6)#1633 <= 15
 0 <= w(8)(2)(3)(7)#1634 <= 14
 0 <= w(8)(2)(3)(8)#1635 <= 19
 0 <= w(8)(2)(3)(9)#1636 <= 13
 0 <= w(8)(2)(3)(10)#1637 <= 15
 0 <= w(8)(3)(1)(1)#1638 <= 13
 0 <= w(8)(3)(1)(2)#1639 <= 16
 0 <= w(8)(3)(1)(3)#1640 <= 18
 0 <= w(8)(3)(1)(4)#1641 <= 14
 0 <= w(8)(3)(1)(5)#1642 <= 19
 0 <= w(8)(3)(1)(6)#1643 <= 15
 0 <= w(8)(3)(1)(7)#1644 <= 18
 0 <= w(8)(3)(1)(8)#1645 <= 13
 0 <= w(8)(3)(1)(9)#1646 <= 17
 0 <= w(8)(3)(1)(10)#1647 <= 16
 0 <= w(8)(3)(2)(1)#1648 <= 18
 0 <= w(8)(3)(2)(2)#1649 <= 15
 0 <= w(8)(3)(2)(3)#1650 <= 17
 0 <= w(8)(3)(2)(4)#1651 <= 15
 0 <= w(8)(3)(2)(5)#1652 <= 14
 0 <= w(8)(3)(2)(6)#1653 <= 19
 0 <= w(8)(3)(2)(7)#1654 <= 18
 0 <= w(8)(3)(2)(8)#1655 <= 17
 0 <= w(8)(3)(2)(9)#1656 <= 15
 0 <= w(8)(3)(2)(10)#1657 <= 15
 0 <= w(8)(3)(3)(1)#1658 <= 17
 0 <= w(8)(3)(3)(2)#1659 <= 17
 0 <= w(8)(3)(3)(3)#1660 <= 19
 0 <= w(8)(3)(3)(4)#1661 <= 15
 0 <= w(8)(3)(3)(5)#1662 <= 14
 0 <= w(8)(3)(3)(6)#1663 <= 15
 0 <= w(8)(3)(3)(7)#1664 <= 14
 0 <= w(8)(3)(3)(8)#1665 <= 19
 0 <= w(8)(3)(3)(9)#1666 <= 13
 0 <= w(8)(3)(3)(10)#1667 <= 15
 0 <= w(8)(4)(1)(1)#1668 <= 13
 0 <= w(8)(4)(1)(2)#1669 <= 16
 0 <= w(8)(4)(1)(3)#1670 <= 18
 0 <= w(8)(4)(1)(4)#1671 <= 14
 0 <= w(8)(4)(1)(5)#1672 <= 19
 0 <= w(8)(4)(1)(6)#1673 <= 15
 0 <= w(8)(4)(1)(7)#1674 <= 18
 0 <= w(8)(4)(1)(8)#1675 <= 13
 0 <= w(8)(4)(1)(9)#1676 <= 17
 0 <= w(8)(4)(1)(10)#1677 <= 16
 0 <= w(8)(4)(2)(1)#1678 <= 18
 0 <= w(8)(4)(2)(2)#1679 <= 15
 0 <= w(8)(4)(2)(3)#1680 <= 17
 0 <= w(8)(4)(2)(4)#1681 <= 15
 0 <= w(8)(4)(2)(5)#1682 <= 14
 0 <= w(8)(4)(2)(6)#1683 <= 19
 0 <= w(8)(4)(2)(7)#1684 <= 18
 0 <= w(8)(4)(2)(8)#1685 <= 17
 0 <= w(8)(4)(2)(9)#1686 <= 15
 0 <= w(8)(4)(2)(10)#1687 <= 15
 0 <= w(8)(4)(3)(1)#1688 <= 17
 0 <= w(8)(4)(3)(2)#1689 <= 17
 0 <= w(8)(4)(3)(3)#1690 <= 19
 0 <= w(8)(4)(3)(4)#1691 <= 15
 0 <= w(8)(4)(3)(5)#1692 <= 14
 0 <= w(8)(4)(3)(6)#1693 <= 15
 0 <= w(8)(4)(3)(7)#1694 <= 14
 0 <= w(8)(4)(3)(8)#1695 <= 19
 0 <= w(8)(4)(3)(9)#1696 <= 13
 0 <= w(8)(4)(3)(10)#1697 <= 15
 0 <= w(8)(5)(1)(1)#1698 <= 13
 0 <= w(8)(5)(1)(2)#1699 <= 16
 0 <= w(8)(5)(1)(3)#1700 <= 18
 0 <= w(8)(5)(1)(4)#1701 <= 14
 0 <= w(8)(5)(1)(5)#1702 <= 19
 0 <= w(8)(5)(1)(6)#1703 <= 15
 0 <= w(8)(5)(1)(7)#1704 <= 18
 0 <= w(8)(5)(1)(8)#1705 <= 13
 0 <= w(8)(5)(1)(9)#1706 <= 17
 0 <= w(8)(5)(1)(10)#1707 <= 16
 0 <= w(8)(5)(2)(1)#1708 <= 18
 0 <= w(8)(5)(2)(2)#1709 <= 15
 0 <= w(8)(5)(2)(3)#1710 <= 17
 0 <= w(8)(5)(2)(4)#1711 <= 15
 0 <= w(8)(5)(2)(5)#1712 <= 14
 0 <= w(8)(5)(2)(6)#1713 <= 19
 0 <= w(8)(5)(2)(7)#1714 <= 18
 0 <= w(8)(5)(2)(8)#1715 <= 17
 0 <= w(8)(5)(2)(9)#1716 <= 15
 0 <= w(8)(5)(2)(10)#1717 <= 15
 0 <= w(8)(5)(3)(1)#1718 <= 17
 0 <= w(8)(5)(3)(2)#1719 <= 17
 0 <= w(8)(5)(3)(3)#1720 <= 19
 0 <= w(8)(5)(3)(4)#1721 <= 15
 0 <= w(8)(5)(3)(5)#1722 <= 14
 0 <= w(8)(5)(3)(6)#1723 <= 15
 0 <= w(8)(5)(3)(7)#1724 <= 14
 0 <= w(8)(5)(3)(8)#1725 <= 19
 0 <= w(8)(5)(3)(9)#1726 <= 13
 0 <= w(8)(5)(3)(10)#1727 <= 15
 0 <= w(8)(6)(1)(1)#1728 <= 13
 0 <= w(8)(6)(1)(2)#1729 <= 16
 0 <= w(8)(6)(1)(3)#1730 <= 18
 0 <= w(8)(6)(1)(4)#1731 <= 14
 0 <= w(8)(6)(1)(5)#1732 <= 19
 0 <= w(8)(6)(1)(6)#1733 <= 15
 0 <= w(8)(6)(1)(7)#1734 <= 18
 0 <= w(8)(6)(1)(8)#1735 <= 13
 0 <= w(8)(6)(1)(9)#1736 <= 17
 0 <= w(8)(6)(1)(10)#1737 <= 16
 0 <= w(8)(6)(2)(1)#1738 <= 18
 0 <= w(8)(6)(2)(2)#1739 <= 15
 0 <= w(8)(6)(2)(3)#1740 <= 17
 0 <= w(8)(6)(2)(4)#1741 <= 15
 0 <= w(8)(6)(2)(5)#1742 <= 14
 0 <= w(8)(6)(2)(6)#1743 <= 19
 0 <= w(8)(6)(2)(7)#1744 <= 18
 0 <= w(8)(6)(2)(8)#1745 <= 17
 0 <= w(8)(6)(2)(9)#1746 <= 15
 0 <= w(8)(6)(2)(10)#1747 <= 15
 0 <= w(8)(6)(3)(1)#1748 <= 17
 0 <= w(8)(6)(3)(2)#1749 <= 17
 0 <= w(8)(6)(3)(3)#1750 <= 19
 0 <= w(8)(6)(3)(4)#1751 <= 15
 0 <= w(8)(6)(3)(5)#1752 <= 14
 0 <= w(8)(6)(3)(6)#1753 <= 15
 0 <= w(8)(6)(3)(7)#1754 <= 14
 0 <= w(8)(6)(3)(8)#1755 <= 19
 0 <= w(8)(6)(3)(9)#1756 <= 13
 0 <= w(8)(6)(3)(10)#1757 <= 15
 0 <= w(9)(1)(1)(1)#1758 <= 26
 0 <= w(9)(1)(1)(2)#1759 <= 18
 0 <= w(9)(1)(1)(3)#1760 <= 19
 0 <= w(9)(1)(1)(4)#1761 <= 22
 0 <= w(9)(1)(1)(5)#1762 <= 26
 0 <= w(9)(1)(1)(6)#1763 <= 26
 0 <= w(9)(1)(1)(7)#1764 <= 18
 0 <= w(9)(1)(1)(8)#1765 <= 21
 0 <= w(9)(1)(1)(9)#1766 <= 24
 0 <= w(9)(1)(1)(10)#1767 <= 20
 0 <= w(9)(1)(2)(1)#1768 <= 20
 0 <= w(9)(1)(2)(2)#1769 <= 20
 0 <= w(9)(1)(2)(3)#1770 <= 22
 0 <= w(9)(1)(2)(4)#1771 <= 24
 0 <= w(9)(1)(2)(5)#1772 <= 26
 0 <= w(9)(1)(2)(6)#1773 <= 19
 0 <= w(9)(1)(2)(7)#1774 <= 21
 0 <= w(9)(1)(2)(8)#1775 <= 20
 0 <= w(9)(1)(2)(9)#1776 <= 25
 0 <= w(9)(1)(2)(10)#1777 <= 25
 0 <= w(9)(1)(3)(1)#1778 <= 24
 0 <= w(9)(1)(3)(2)#1779 <= 18
 0 <= w(9)(1)(3)(3)#1780 <= 26
 0 <= w(9)(1)(3)(4)#1781 <= 23
 0 <= w(9)(1)(3)(5)#1782 <= 24
 0 <= w(9)(1)(3)(6)#1783 <= 26
 0 <= w(9)(1)(3)(7)#1784 <= 26
 0 <= w(9)(1)(3)(8)#1785 <= 25
 0 <= w(9)(1)(3)(9)#1786 <= 24
 0 <= w(9)(1)(3)(10)#1787 <= 25
 0 <= w(9)(2)(1)(1)#1788 <= 26
 0 <= w(9)(2)(1)(2)#1789 <= 18
 0 <= w(9)(2)(1)(3)#1790 <= 19
 0 <= w(9)(2)(1)(4)#1791 <= 22
 0 <= w(9)(2)(1)(5)#1792 <= 26
 0 <= w(9)(2)(1)(6)#1793 <= 26
 0 <= w(9)(2)(1)(7)#1794 <= 18
 0 <= w(9)(2)(1)(8)#1795 <= 21
 0 <= w(9)(2)(1)(9)#1796 <= 24
 0 <= w(9)(2)(1)(10)#1797 <= 20
 0 <= w(9)(2)(2)(1)#1798 <= 20
 0 <= w(9)(2)(2)(2)#1799 <= 20
 0 <= w(9)(2)(2)(3)#1800 <= 22
 0 <= w(9)(2)(2)(4)#1801 <= 24
 0 <= w(9)(2)(2)(5)#1802 <= 26
 0 <= w(9)(2)(2)(6)#1803 <= 19
 0 <= w(9)(2)(2)(7)#1804 <= 21
 0 <= w(9)(2)(2)(8)#1805 <= 20
 0 <= w(9)(2)(2)(9)#1806 <= 25
 0 <= w(9)(2)(2)(10)#1807 <= 25
 0 <= w(9)(2)(3)(1)#1808 <= 24
 0 <= w(9)(2)(3)(2)#1809 <= 18
 0 <= w(9)(2)(3)(3)#1810 <= 26
 0 <= w(9)(2)(3)(4)#1811 <= 23
 0 <= w(9)(2)(3)(5)#1812 <= 24
 0 <= w(9)(2)(3)(6)#1813 <= 26
 0 <= w(9)(2)(3)(7)#1814 <= 26
 0 <= w(9)(2)(3)(8)#1815 <= 25
 0 <= w(9)(2)(3)(9)#1816 <= 24
 0 <= w(9)(2)(3)(10)#1817 <= 25
 0 <= w(9)(3)(1)(1)#1818 <= 26
 0 <= w(9)(3)(1)(2)#1819 <= 18
 0 <= w(9)(3)(1)(3)#1820 <= 19
 0 <= w(9)(3)(1)(4)#1821 <= 22
 0 <= w(9)(3)(1)(5)#1822 <= 26
 0 <= w(9)(3)(1)(6)#1823 <= 26
 0 <= w(9)(3)(1)(7)#1824 <= 18
 0 <= w(9)(3)(1)(8)#1825 <= 21
 0 <= w(9)(3)(1)(9)#1826 <= 24
 0 <= w(9)(3)(1)(10)#1827 <= 20
 0 <= w(9)(3)(2)(1)#1828 <= 20
 0 <= w(9)(3)(2)(2)#1829 <= 20
 0 <= w(9)(3)(2)(3)#1830 <= 22
 0 <= w(9)(3)(2)(4)#1831 <= 24
 0 <= w(9)(3)(2)(5)#1832 <= 26
 0 <= w(9)(3)(2)(6)#1833 <= 19
 0 <= w(9)(3)(2)(7)#1834 <= 21
 0 <= w(9)(3)(2)(8)#1835 <= 20
 0 <= w(9)(3)(2)(9)#1836 <= 25
 0 <= w(9)(3)(2)(10)#1837 <= 25
 0 <= w(9)(3)(3)(1)#1838 <= 24
 0 <= w(9)(3)(3)(2)#1839 <= 18
 0 <= w(9)(3)(3)(3)#1840 <= 26
 0 <= w(9)(3)(3)(4)#1841 <= 23
 0 <= w(9)(3)(3)(5)#1842 <= 24
 0 <= w(9)(3)(3)(6)#1843 <= 26
 0 <= w(9)(3)(3)(7)#1844 <= 26
 0 <= w(9)(3)(3)(8)#1845 <= 25
 0 <= w(9)(3)(3)(9)#1846 <= 24
 0 <= w(9)(3)(3)(10)#1847 <= 25
 0 <= w(9)(4)(1)(1)#1848 <= 26
 0 <= w(9)(4)(1)(2)#1849 <= 18
 0 <= w(9)(4)(1)(3)#1850 <= 19
 0 <= w(9)(4)(1)(4)#1851 <= 22
 0 <= w(9)(4)(1)(5)#1852 <= 26
 0 <= w(9)(4)(1)(6)#1853 <= 26
 0 <= w(9)(4)(1)(7)#1854 <= 18
 0 <= w(9)(4)(1)(8)#1855 <= 21
 0 <= w(9)(4)(1)(9)#1856 <= 24
 0 <= w(9)(4)(1)(10)#1857 <= 20
 0 <= w(9)(4)(2)(1)#1858 <= 20
 0 <= w(9)(4)(2)(2)#1859 <= 20
 0 <= w(9)(4)(2)(3)#1860 <= 22
 0 <= w(9)(4)(2)(4)#1861 <= 24
 0 <= w(9)(4)(2)(5)#1862 <= 26
 0 <= w(9)(4)(2)(6)#1863 <= 19
 0 <= w(9)(4)(2)(7)#1864 <= 21
 0 <= w(9)(4)(2)(8)#1865 <= 20
 0 <= w(9)(4)(2)(9)#1866 <= 25
 0 <= w(9)(4)(2)(10)#1867 <= 25
 0 <= w(9)(4)(3)(1)#1868 <= 24
 0 <= w(9)(4)(3)(2)#1869 <= 18
 0 <= w(9)(4)(3)(3)#1870 <= 26
 0 <= w(9)(4)(3)(4)#1871 <= 23
 0 <= w(9)(4)(3)(5)#1872 <= 24
 0 <= w(9)(4)(3)(6)#1873 <= 26
 0 <= w(9)(4)(3)(7)#1874 <= 26
 0 <= w(9)(4)(3)(8)#1875 <= 25
 0 <= w(9)(4)(3)(9)#1876 <= 24
 0 <= w(9)(4)(3)(10)#1877 <= 25
 0 <= w(9)(5)(1)(1)#1878 <= 26
 0 <= w(9)(5)(1)(2)#1879 <= 18
 0 <= w(9)(5)(1)(3)#1880 <= 19
 0 <= w(9)(5)(1)(4)#1881 <= 22
 0 <= w(9)(5)(1)(5)#1882 <= 26
 0 <= w(9)(5)(1)(6)#1883 <= 26
 0 <= w(9)(5)(1)(7)#1884 <= 18
 0 <= w(9)(5)(1)(8)#1885 <= 21
 0 <= w(9)(5)(1)(9)#1886 <= 24
 0 <= w(9)(5)(1)(10)#1887 <= 20
 0 <= w(9)(5)(2)(1)#1888 <= 20
 0 <= w(9)(5)(2)(2)#1889 <= 20
 0 <= w(9)(5)(2)(3)#1890 <= 22
 0 <= w(9)(5)(2)(4)#1891 <= 24
 0 <= w(9)(5)(2)(5)#1892 <= 26
 0 <= w(9)(5)(2)(6)#1893 <= 19
 0 <= w(9)(5)(2)(7)#1894 <= 21
 0 <= w(9)(5)(2)(8)#1895 <= 20
 0 <= w(9)(5)(2)(9)#1896 <= 25
 0 <= w(9)(5)(2)(10)#1897 <= 25
 0 <= w(9)(5)(3)(1)#1898 <= 24
 0 <= w(9)(5)(3)(2)#1899 <= 18
 0 <= w(9)(5)(3)(3)#1900 <= 26
 0 <= w(9)(5)(3)(4)#1901 <= 23
 0 <= w(9)(5)(3)(5)#1902 <= 24
 0 <= w(9)(5)(3)(6)#1903 <= 26
 0 <= w(9)(5)(3)(7)#1904 <= 26
 0 <= w(9)(5)(3)(8)#1905 <= 25
 0 <= w(9)(5)(3)(9)#1906 <= 24
 0 <= w(9)(5)(3)(10)#1907 <= 25
 0 <= w(9)(6)(1)(1)#1908 <= 26
 0 <= w(9)(6)(1)(2)#1909 <= 18
 0 <= w(9)(6)(1)(3)#1910 <= 19
 0 <= w(9)(6)(1)(4)#1911 <= 22
 0 <= w(9)(6)(1)(5)#1912 <= 26
 0 <= w(9)(6)(1)(6)#1913 <= 26
 0 <= w(9)(6)(1)(7)#1914 <= 18
 0 <= w(9)(6)(1)(8)#1915 <= 21
 0 <= w(9)(6)(1)(9)#1916 <= 24
 0 <= w(9)(6)(1)(10)#1917 <= 20
 0 <= w(9)(6)(2)(1)#1918 <= 20
 0 <= w(9)(6)(2)(2)#1919 <= 20
 0 <= w(9)(6)(2)(3)#1920 <= 22
 0 <= w(9)(6)(2)(4)#1921 <= 24
 0 <= w(9)(6)(2)(5)#1922 <= 26
 0 <= w(9)(6)(2)(6)#1923 <= 19
 0 <= w(9)(6)(2)(7)#1924 <= 21
 0 <= w(9)(6)(2)(8)#1925 <= 20
 0 <= w(9)(6)(2)(9)#1926 <= 25
 0 <= w(9)(6)(2)(10)#1927 <= 25
 0 <= w(9)(6)(3)(1)#1928 <= 24
 0 <= w(9)(6)(3)(2)#1929 <= 18
 0 <= w(9)(6)(3)(3)#1930 <= 26
 0 <= w(9)(6)(3)(4)#1931 <= 23
 0 <= w(9)(6)(3)(5)#1932 <= 24
 0 <= w(9)(6)(3)(6)#1933 <= 26
 0 <= w(9)(6)(3)(7)#1934 <= 26
 0 <= w(9)(6)(3)(8)#1935 <= 25
 0 <= w(9)(6)(3)(9)#1936 <= 24
 0 <= w(9)(6)(3)(10)#1937 <= 25
 0 <= w(10)(1)(1)(1)#1938 <= 12
 0 <= w(10)(1)(1)(2)#1939 <= 11
 0 <= w(10)(1)(1)(3)#1940 <= 14
 0 <= w(10)(1)(1)(4)#1941 <= 15
 0 <= w(10)(1)(1)(5)#1942 <= 11
 0 <= w(10)(1)(1)(6)#1943 <= 13
 0 <= w(10)(1)(1)(7)#1944 <= 13
 0 <= w(10)(1)(1)(8)#1945 <= 10
 0 <= w(10)(1)(1)(9)#1946 <= 11
 0 <= w(10)(1)(1)(10)#1947 <= 11
 0 <= w(10)(1)(2)(1)#1948 <= 11
 0 <= w(10)(1)(2)(2)#1949 <= 15
 0 <= w(10)(1)(2)(3)#1950 <= 11
 0 <= w(10)(1)(2)(4)#1951 <= 16
 0 <= w(10)(1)(2)(5)#1952 <= 13
 0 <= w(10)(1)(2)(6)#1953 <= 14
 0 <= w(10)(1)(2)(7)#1954 <= 14
 0 <= w(10)(1)(2)(8)#1955 <= 11
 0 <= w(10)(1)(2)(9)#1956 <= 11
 0 <= w(10)(1)(2)(10)#1957 <= 13
 0 <= w(10)(1)(3)(1)#1958 <= 13
 0 <= w(10)(1)(3)(2)#1959 <= 14
 0 <= w(10)(1)(3)(3)#1960 <= 15
 0 <= w(10)(1)(3)(4)#1961 <= 15
 0 <= w(10)(1)(3)(5)#1962 <= 11
 0 <= w(10)(1)(3)(6)#1963 <= 14
 0 <= w(10)(1)(3)(7)#1964 <= 15
 0 <= w(10)(1)(3)(8)#1965 <= 14
 0 <= w(10)(1)(3)(9)#1966 <= 15
 0 <= w(10)(1)(3)(10)#1967 <= 13
 0 <= w(10)(2)(1)(1)#1968 <= 12
 0 <= w(10)(2)(1)(2)#1969 <= 11
 0 <= w(10)(2)(1)(3)#1970 <= 14
 0 <= w(10)(2)(1)(4)#1971 <= 15
 0 <= w(10)(2)(1)(5)#1972 <= 11
 0 <= w(10)(2)(1)(6)#1973 <= 13
 0 <= w(10)(2)(1)(7)#1974 <= 13
 0 <= w(10)(2)(1)(8)#1975 <= 10
 0 <= w(10)(2)(1)(9)#1976 <= 11
 0 <= w(10)(2)(1)(10)#1977 <= 11
 0 <= w(10)(2)(2)(1)#1978 <= 11
 0 <= w(10)(2)(2)(2)#1979 <= 15
 0 <= w(10)(2)(2)(3)#1980 <= 11
 0 <= w(10)(2)(2)(4)#1981 <= 16
 0 <= w(10)(2)(2)(5)#1982 <= 13
 0 <= w(10)(2)(2)(6)#1983 <= 14
 0 <= w(10)(2)(2)(7)#1984 <= 14
 0 <= w(10)(2)(2)(8)#1985 <= 11
 0 <= w(10)(2)(2)(9)#1986 <= 11
 0 <= w(10)(2)(2)(10)#1987 <= 13
 0 <= w(10)(2)(3)(1)#1988 <= 13
 0 <= w(10)(2)(3)(2)#1989 <= 14
 0 <= w(10)(2)(3)(3)#1990 <= 15
 0 <= w(10)(2)(3)(4)#1991 <= 15
 0 <= w(10)(2)(3)(5)#1992 <= 11
 0 <= w(10)(2)(3)(6)#1993 <= 14
 0 <= w(10)(2)(3)(7)#1994 <= 15
 0 <= w(10)(2)(3)(8)#1995 <= 14
 0 <= w(10)(2)(3)(9)#1996 <= 15
 0 <= w(10)(2)(3)(10)#1997 <= 13
 0 <= w(10)(3)(1)(1)#1998 <= 12
 0 <= w(10)(3)(1)(2)#1999 <= 11
 0 <= w(10)(3)(1)(3)#2000 <= 14
 0 <= w(10)(3)(1)(4)#2001 <= 15
 0 <= w(10)(3)(1)(5)#2002 <= 11
 0 <= w(10)(3)(1)(6)#2003 <= 13
 0 <= w(10)(3)(1)(7)#2004 <= 13
 0 <= w(10)(3)(1)(8)#2005 <= 10
 0 <= w(10)(3)(1)(9)#2006 <= 11
 0 <= w(10)(3)(1)(10)#2007 <= 11
 0 <= w(10)(3)(2)(1)#2008 <= 11
 0 <= w(10)(3)(2)(2)#2009 <= 15
 0 <= w(10)(3)(2)(3)#2010 <= 11
 0 <= w(10)(3)(2)(4)#2011 <= 16
 0 <= w(10)(3)(2)(5)#2012 <= 13
 0 <= w(10)(3)(2)(6)#2013 <= 14
 0 <= w(10)(3)(2)(7)#2014 <= 14
 0 <= w(10)(3)(2)(8)#2015 <= 11
 0 <= w(10)(3)(2)(9)#2016 <= 11
 0 <= w(10)(3)(2)(10)#2017 <= 13
 0 <= w(10)(3)(3)(1)#2018 <= 13
 0 <= w(10)(3)(3)(2)#2019 <= 14
 0 <= w(10)(3)(3)(3)#2020 <= 15
 0 <= w(10)(3)(3)(4)#2021 <= 15
 0 <= w(10)(3)(3)(5)#2022 <= 11
 0 <= w(10)(3)(3)(6)#2023 <= 14
 0 <= w(10)(3)(3)(7)#2024 <= 15
 0 <= w(10)(3)(3)(8)#2025 <= 14
 0 <= w(10)(3)(3)(9)#2026 <= 15
 0 <= w(10)(3)(3)(10)#2027 <= 13
 0 <= w(10)(4)(1)(1)#2028 <= 12
 0 <= w(10)(4)(1)(2)#2029 <= 11
 0 <= w(10)(4)(1)(3)#2030 <= 14
 0 <= w(10)(4)(1)(4)#2031 <= 15
 0 <= w(10)(4)(1)(5)#2032 <= 11
 0 <= w(10)(4)(1)(6)#2033 <= 13
 0 <= w(10)(4)(1)(7)#2034 <= 13
 0 <= w(10)(4)(1)(8)#2035 <= 10
 0 <= w(10)(4)(1)(9)#2036 <= 11
 0 <= w(10)(4)(1)(10)#2037 <= 11
 0 <= w(10)(4)(2)(1)#2038 <= 11
 0 <= w(10)(4)(2)(2)#2039 <= 15
 0 <= w(10)(4)(2)(3)#2040 <= 11
 0 <= w(10)(4)(2)(4)#2041 <= 16
 0 <= w(10)(4)(2)(5)#2042 <= 13
 0 <= w(10)(4)(2)(6)#2043 <= 14
 0 <= w(10)(4)(2)(7)#2044 <= 14
 0 <= w(10)(4)(2)(8)#2045 <= 11
 0 <= w(10)(4)(2)(9)#2046 <= 11
 0 <= w(10)(4)(2)(10)#2047 <= 13
 0 <= w(10)(4)(3)(1)#2048 <= 13
 0 <= w(10)(4)(3)(2)#2049 <= 14
 0 <= w(10)(4)(3)(3)#2050 <= 15
 0 <= w(10)(4)(3)(4)#2051 <= 15
 0 <= w(10)(4)(3)(5)#2052 <= 11
 0 <= w(10)(4)(3)(6)#2053 <= 14
 0 <= w(10)(4)(3)(7)#2054 <= 15
 0 <= w(10)(4)(3)(8)#2055 <= 14
 0 <= w(10)(4)(3)(9)#2056 <= 15
 0 <= w(10)(4)(3)(10)#2057 <= 13
 0 <= w(10)(5)(1)(1)#2058 <= 12
 0 <= w(10)(5)(1)(2)#2059 <= 11
 0 <= w(10)(5)(1)(3)#2060 <= 14
 0 <= w(10)(5)(1)(4)#2061 <= 15
 0 <= w(10)(5)(1)(5)#2062 <= 11
 0 <= w(10)(5)(1)(6)#2063 <= 13
 0 <= w(10)(5)(1)(7)#2064 <= 13
 0 <= w(10)(5)(1)(8)#2065 <= 10
 0 <= w(10)(5)(1)(9)#2066 <= 11
 0 <= w(10)(5)(1)(10)#2067 <= 11
 0 <= w(10)(5)(2)(1)#2068 <= 11
 0 <= w(10)(5)(2)(2)#2069 <= 15
 0 <= w(10)(5)(2)(3)#2070 <= 11
 0 <= w(10)(5)(2)(4)#2071 <= 16
 0 <= w(10)(5)(2)(5)#2072 <= 13
 0 <= w(10)(5)(2)(6)#2073 <= 14
 0 <= w(10)(5)(2)(7)#2074 <= 14
 0 <= w(10)(5)(2)(8)#2075 <= 11
 0 <= w(10)(5)(2)(9)#2076 <= 11
 0 <= w(10)(5)(2)(10)#2077 <= 13
 0 <= w(10)(5)(3)(1)#2078 <= 13
 0 <= w(10)(5)(3)(2)#2079 <= 14
 0 <= w(10)(5)(3)(3)#2080 <= 15
 0 <= w(10)(5)(3)(4)#2081 <= 15
 0 <= w(10)(5)(3)(5)#2082 <= 11
 0 <= w(10)(5)(3)(6)#2083 <= 14
 0 <= w(10)(5)(3)(7)#2084 <= 15
 0 <= w(10)(5)(3)(8)#2085 <= 14
 0 <= w(10)(5)(3)(9)#2086 <= 15
 0 <= w(10)(5)(3)(10)#2087 <= 13
 0 <= w(10)(6)(1)(1)#2088 <= 12
 0 <= w(10)(6)(1)(2)#2089 <= 11
 0 <= w(10)(6)(1)(3)#2090 <= 14
 0 <= w(10)(6)(1)(4)#2091 <= 15
 0 <= w(10)(6)(1)(5)#2092 <= 11
 0 <= w(10)(6)(1)(6)#2093 <= 13
 0 <= w(10)(6)(1)(7)#2094 <= 13
 0 <= w(10)(6)(1)(8)#2095 <= 10
 0 <= w(10)(6)(1)(9)#2096 <= 11
 0 <= w(10)(6)(1)(10)#2097 <= 11
 0 <= w(10)(6)(2)(1)#2098 <= 11
 0 <= w(10)(6)(2)(2)#2099 <= 15
 0 <= w(10)(6)(2)(3)#2100 <= 11
 0 <= w(10)(6)(2)(4)#2101 <= 16
 0 <= w(10)(6)(2)(5)#2102 <= 13
 0 <= w(10)(6)(2)(6)#2103 <= 14
 0 <= w(10)(6)(2)(7)#2104 <= 14
 0 <= w(10)(6)(2)(8)#2105 <= 11
 0 <= w(10)(6)(2)(9)#2106 <= 11
 0 <= w(10)(6)(2)(10)#2107 <= 13
 0 <= w(10)(6)(3)(1)#2108 <= 13
 0 <= w(10)(6)(3)(2)#2109 <= 14
 0 <= w(10)(6)(3)(3)#2110 <= 15
 0 <= w(10)(6)(3)(4)#2111 <= 15
 0 <= w(10)(6)(3)(5)#2112 <= 11
 0 <= w(10)(6)(3)(6)#2113 <= 14
 0 <= w(10)(6)(3)(7)#2114 <= 15
 0 <= w(10)(6)(3)(8)#2115 <= 14
 0 <= w(10)(6)(3)(9)#2116 <= 15
 0 <= w(10)(6)(3)(10)#2117 <= 13
 0 <= w(11)(1)(1)(1)#2118 <= 15
 0 <= w(11)(1)(1)(2)#2119 <= 22
 0 <= w(11)(1)(1)(3)#2120 <= 18
 0 <= w(11)(1)(1)(4)#2121 <= 18
 0 <= w(11)(1)(1)(5)#2122 <= 22
 0 <= w(11)(1)(1)(6)#2123 <= 17
 0 <= w(11)(1)(1)(7)#2124 <= 20
 0 <= w(11)(1)(1)(8)#2125 <= 22
 0 <= w(11)(1)(1)(9)#2126 <= 21
 0 <= w(11)(1)(1)(10)#2127 <= 19
 0 <= w(11)(1)(2)(1)#2128 <= 16
 0 <= w(11)(1)(2)(2)#2129 <= 18
 0 <= w(11)(1)(2)(3)#2130 <= 22
 0 <= w(11)(1)(2)(4)#2131 <= 22
 0 <= w(11)(1)(2)(5)#2132 <= 21
 0 <= w(11)(1)(2)(6)#2133 <= 16
 0 <= w(11)(1)(2)(7)#2134 <= 20
 0 <= w(11)(1)(2)(8)#2135 <= 17
 0 <= w(11)(1)(2)(9)#2136 <= 15
 0 <= w(11)(1)(2)(10)#2137 <= 18
 0 <= w(11)(1)(3)(1)#2138 <= 16
 0 <= w(11)(1)(3)(2)#2139 <= 22
 0 <= w(11)(1)(3)(3)#2140 <= 20
 0 <= w(11)(1)(3)(4)#2141 <= 16
 0 <= w(11)(1)(3)(5)#2142 <= 16
 0 <= w(11)(1)(3)(6)#2143 <= 16
 0 <= w(11)(1)(3)(7)#2144 <= 21
 0 <= w(11)(1)(3)(8)#2145 <= 19
 0 <= w(11)(1)(3)(9)#2146 <= 19
 0 <= w(11)(1)(3)(10)#2147 <= 17
 0 <= w(11)(2)(1)(1)#2148 <= 15
 0 <= w(11)(2)(1)(2)#2149 <= 22
 0 <= w(11)(2)(1)(3)#2150 <= 18
 0 <= w(11)(2)(1)(4)#2151 <= 18
 0 <= w(11)(2)(1)(5)#2152 <= 22
 0 <= w(11)(2)(1)(6)#2153 <= 17
 0 <= w(11)(2)(1)(7)#2154 <= 20
 0 <= w(11)(2)(1)(8)#2155 <= 22
 0 <= w(11)(2)(1)(9)#2156 <= 21
 0 <= w(11)(2)(1)(10)#2157 <= 19
 0 <= w(11)(2)(2)(1)#2158 <= 16
 0 <= w(11)(2)(2)(2)#2159 <= 18
 0 <= w(11)(2)(2)(3)#2160 <= 22
 0 <= w(11)(2)(2)(4)#2161 <= 22
 0 <= w(11)(2)(2)(5)#2162 <= 21
 0 <= w(11)(2)(2)(6)#2163 <= 16
 0 <= w(11)(2)(2)(7)#2164 <= 20
 0 <= w(11)(2)(2)(8)#2165 <= 17
 0 <= w(11)(2)(2)(9)#2166 <= 15
 0 <= w(11)(2)(2)(10)#2167 <= 18
 0 <= w(11)(2)(3)(1)#2168 <= 16
 0 <= w(11)(2)(3)(2)#2169 <= 22
 0 <= w(11)(2)(3)(3)#2170 <= 20
 0 <= w(11)(2)(3)(4)#2171 <= 16
 0 <= w(11)(2)(3)(5)#2172 <= 16
 0 <= w(11)(2)(3)(6)#2173 <= 16
 0 <= w(11)(2)(3)(7)#2174 <= 21
 0 <= w(11)(2)(3)(8)#2175 <= 19
 0 <= w(11)(2)(3)(9)#2176 <= 19
 0 <= w(11)(2)(3)(10)#2177 <= 17
 0 <= w(11)(3)(1)(1)#2178 <= 15
 0 <= w(11)(3)(1)(2)#2179 <= 22
 0 <= w(11)(3)(1)(3)#2180 <= 18
 0 <= w(11)(3)(1)(4)#2181 <= 18
 0 <= w(11)(3)(1)(5)#2182 <= 22
 0 <= w(11)(3)(1)(6)#2183 <= 17
 0 <= w(11)(3)(1)(7)#2184 <= 20
 0 <= w(11)(3)(1)(8)#2185 <= 22
 0 <= w(11)(3)(1)(9)#2186 <= 21
 0 <= w(11)(3)(1)(10)#2187 <= 19
 0 <= w(11)(3)(2)(1)#2188 <= 16
 0 <= w(11)(3)(2)(2)#2189 <= 18
 0 <= w(11)(3)(2)(3)#2190 <= 22
 0 <= w(11)(3)(2)(4)#2191 <= 22
 0 <= w(11)(3)(2)(5)#2192 <= 21
 0 <= w(11)(3)(2)(6)#2193 <= 16
 0 <= w(11)(3)(2)(7)#2194 <= 20
 0 <= w(11)(3)(2)(8)#2195 <= 17
 0 <= w(11)(3)(2)(9)#2196 <= 15
 0 <= w(11)(3)(2)(10)#2197 <= 18
 0 <= w(11)(3)(3)(1)#2198 <= 16
 0 <= w(11)(3)(3)(2)#2199 <= 22
 0 <= w(11)(3)(3)(3)#2200 <= 20
 0 <= w(11)(3)(3)(4)#2201 <= 16
 0 <= w(11)(3)(3)(5)#2202 <= 16
 0 <= w(11)(3)(3)(6)#2203 <= 16
 0 <= w(11)(3)(3)(7)#2204 <= 21
 0 <= w(11)(3)(3)(8)#2205 <= 19
 0 <= w(11)(3)(3)(9)#2206 <= 19
 0 <= w(11)(3)(3)(10)#2207 <= 17
 0 <= w(11)(4)(1)(1)#2208 <= 15
 0 <= w(11)(4)(1)(2)#2209 <= 22
 0 <= w(11)(4)(1)(3)#2210 <= 18
 0 <= w(11)(4)(1)(4)#2211 <= 18
 0 <= w(11)(4)(1)(5)#2212 <= 22
 0 <= w(11)(4)(1)(6)#2213 <= 17
 0 <= w(11)(4)(1)(7)#2214 <= 20
 0 <= w(11)(4)(1)(8)#2215 <= 22
 0 <= w(11)(4)(1)(9)#2216 <= 21
 0 <= w(11)(4)(1)(10)#2217 <= 19
 0 <= w(11)(4)(2)(1)#2218 <= 16
 0 <= w(11)(4)(2)(2)#2219 <= 18
 0 <= w(11)(4)(2)(3)#2220 <= 22
 0 <= w(11)(4)(2)(4)#2221 <= 22
 0 <= w(11)(4)(2)(5)#2222 <= 21
 0 <= w(11)(4)(2)(6)#2223 <= 16
 0 <= w(11)(4)(2)(7)#2224 <= 20
 0 <= w(11)(4)(2)(8)#2225 <= 17
 0 <= w(11)(4)(2)(9)#2226 <= 15
 0 <= w(11)(4)(2)(10)#2227 <= 18
 0 <= w(11)(4)(3)(1)#2228 <= 16
 0 <= w(11)(4)(3)(2)#2229 <= 22
 0 <= w(11)(4)(3)(3)#2230 <= 20
 0 <= w(11)(4)(3)(4)#2231 <= 16
 0 <= w(11)(4)(3)(5)#2232 <= 16
 0 <= w(11)(4)(3)(6)#2233 <= 16
 0 <= w(11)(4)(3)(7)#2234 <= 21
 0 <= w(11)(4)(3)(8)#2235 <= 19
 0 <= w(11)(4)(3)(9)#2236 <= 19
 0 <= w(11)(4)(3)(10)#2237 <= 17
 0 <= w(11)(5)(1)(1)#2238 <= 15
 0 <= w(11)(5)(1)(2)#2239 <= 22
 0 <= w(11)(5)(1)(3)#2240 <= 18
 0 <= w(11)(5)(1)(4)#2241 <= 18
 0 <= w(11)(5)(1)(5)#2242 <= 22
 0 <= w(11)(5)(1)(6)#2243 <= 17
 0 <= w(11)(5)(1)(7)#2244 <= 20
 0 <= w(11)(5)(1)(8)#2245 <= 22
 0 <= w(11)(5)(1)(9)#2246 <= 21
 0 <= w(11)(5)(1)(10)#2247 <= 19
 0 <= w(11)(5)(2)(1)#2248 <= 16
 0 <= w(11)(5)(2)(2)#2249 <= 18
 0 <= w(11)(5)(2)(3)#2250 <= 22
 0 <= w(11)(5)(2)(4)#2251 <= 22
 0 <= w(11)(5)(2)(5)#2252 <= 21
 0 <= w(11)(5)(2)(6)#2253 <= 16
 0 <= w(11)(5)(2)(7)#2254 <= 20
 0 <= w(11)(5)(2)(8)#2255 <= 17
 0 <= w(11)(5)(2)(9)#2256 <= 15
 0 <= w(11)(5)(2)(10)#2257 <= 18
 0 <= w(11)(5)(3)(1)#2258 <= 16
 0 <= w(11)(5)(3)(2)#2259 <= 22
 0 <= w(11)(5)(3)(3)#2260 <= 20
 0 <= w(11)(5)(3)(4)#2261 <= 16
 0 <= w(11)(5)(3)(5)#2262 <= 16
 0 <= w(11)(5)(3)(6)#2263 <= 16
 0 <= w(11)(5)(3)(7)#2264 <= 21
 0 <= w(11)(5)(3)(8)#2265 <= 19
 0 <= w(11)(5)(3)(9)#2266 <= 19
 0 <= w(11)(5)(3)(10)#2267 <= 17
 0 <= w(11)(6)(1)(1)#2268 <= 15
 0 <= w(11)(6)(1)(2)#2269 <= 22
 0 <= w(11)(6)(1)(3)#2270 <= 18
 0 <= w(11)(6)(1)(4)#2271 <= 18
 0 <= w(11)(6)(1)(5)#2272 <= 22
 0 <= w(11)(6)(1)(6)#2273 <= 17
 0 <= w(11)(6)(1)(7)#2274 <= 20
 0 <= w(11)(6)(1)(8)#2275 <= 22
 0 <= w(11)(6)(1)(9)#2276 <= 21
 0 <= w(11)(6)(1)(10)#2277 <= 19
 0 <= w(11)(6)(2)(1)#2278 <= 16
 0 <= w(11)(6)(2)(2)#2279 <= 18
 0 <= w(11)(6)(2)(3)#2280 <= 22
 0 <= w(11)(6)(2)(4)#2281 <= 22
 0 <= w(11)(6)(2)(5)#2282 <= 21
 0 <= w(11)(6)(2)(6)#2283 <= 16
 0 <= w(11)(6)(2)(7)#2284 <= 20
 0 <= w(11)(6)(2)(8)#2285 <= 17
 0 <= w(11)(6)(2)(9)#2286 <= 15
 0 <= w(11)(6)(2)(10)#2287 <= 18
 0 <= w(11)(6)(3)(1)#2288 <= 16
 0 <= w(11)(6)(3)(2)#2289 <= 22
 0 <= w(11)(6)(3)(3)#2290 <= 20
 0 <= w(11)(6)(3)(4)#2291 <= 16
 0 <= w(11)(6)(3)(5)#2292 <= 16
 0 <= w(11)(6)(3)(6)#2293 <= 16
 0 <= w(11)(6)(3)(7)#2294 <= 21
 0 <= w(11)(6)(3)(8)#2295 <= 19
 0 <= w(11)(6)(3)(9)#2296 <= 19
 0 <= w(11)(6)(3)(10)#2297 <= 17
 0 <= w(12)(1)(1)(1)#2298 <= 21
 0 <= w(12)(1)(1)(2)#2299 <= 18
 0 <= w(12)(1)(1)(3)#2300 <= 21
 0 <= w(12)(1)(1)(4)#2301 <= 24
 0 <= w(12)(1)(1)(5)#2302 <= 22
 0 <= w(12)(1)(1)(6)#2303 <= 20
 0 <= w(12)(1)(1)(7)#2304 <= 20
 0 <= w(12)(1)(1)(8)#2305 <= 19
 0 <= w(12)(1)(1)(9)#2306 <= 18
 0 <= w(12)(1)(1)(10)#2307 <= 18
 0 <= w(12)(1)(2)(1)#2308 <= 21
 0 <= w(12)(1)(2)(2)#2309 <= 22
 0 <= w(12)(1)(2)(3)#2310 <= 21
 0 <= w(12)(1)(2)(4)#2311 <= 23
 0 <= w(12)(1)(2)(5)#2312 <= 19
 0 <= w(12)(1)(2)(6)#2313 <= 19
 0 <= w(12)(1)(2)(7)#2314 <= 23
 0 <= w(12)(1)(2)(8)#2315 <= 26
 0 <= w(12)(1)(2)(9)#2316 <= 25
 0 <= w(12)(1)(2)(10)#2317 <= 18
 0 <= w(12)(1)(3)(1)#2318 <= 25
 0 <= w(12)(1)(3)(2)#2319 <= 25
 0 <= w(12)(1)(3)(3)#2320 <= 21
 0 <= w(12)(1)(3)(4)#2321 <= 19
 0 <= w(12)(1)(3)(5)#2322 <= 19
 0 <= w(12)(1)(3)(6)#2323 <= 26
 0 <= w(12)(1)(3)(7)#2324 <= 25
 0 <= w(12)(1)(3)(8)#2325 <= 22
 0 <= w(12)(1)(3)(9)#2326 <= 21
 0 <= w(12)(1)(3)(10)#2327 <= 25
 0 <= w(12)(2)(1)(1)#2328 <= 21
 0 <= w(12)(2)(1)(2)#2329 <= 18
 0 <= w(12)(2)(1)(3)#2330 <= 21
 0 <= w(12)(2)(1)(4)#2331 <= 24
 0 <= w(12)(2)(1)(5)#2332 <= 22
 0 <= w(12)(2)(1)(6)#2333 <= 20
 0 <= w(12)(2)(1)(7)#2334 <= 20
 0 <= w(12)(2)(1)(8)#2335 <= 19
 0 <= w(12)(2)(1)(9)#2336 <= 18
 0 <= w(12)(2)(1)(10)#2337 <= 18
 0 <= w(12)(2)(2)(1)#2338 <= 21
 0 <= w(12)(2)(2)(2)#2339 <= 22
 0 <= w(12)(2)(2)(3)#2340 <= 21
 0 <= w(12)(2)(2)(4)#2341 <= 23
 0 <= w(12)(2)(2)(5)#2342 <= 19
 0 <= w(12)(2)(2)(6)#2343 <= 19
 0 <= w(12)(2)(2)(7)#2344 <= 23
 0 <= w(12)(2)(2)(8)#2345 <= 26
 0 <= w(12)(2)(2)(9)#2346 <= 25
 0 <= w(12)(2)(2)(10)#2347 <= 18
 0 <= w(12)(2)(3)(1)#2348 <= 25
 0 <= w(12)(2)(3)(2)#2349 <= 25
 0 <= w(12)(2)(3)(3)#2350 <= 21
 0 <= w(12)(2)(3)(4)#2351 <= 19
 0 <= w(12)(2)(3)(5)#2352 <= 19
 0 <= w(12)(2)(3)(6)#2353 <= 26
 0 <= w(12)(2)(3)(7)#2354 <= 25
 0 <= w(12)(2)(3)(8)#2355 <= 22
 0 <= w(12)(2)(3)(9)#2356 <= 21
 0 <= w(12)(2)(3)(10)#2357 <= 25
 0 <= w(12)(3)(1)(1)#2358 <= 21
 0 <= w(12)(3)(1)(2)#2359 <= 18
 0 <= w(12)(3)(1)(3)#2360 <= 21
 0 <= w(12)(3)(1)(4)#2361 <= 24
 0 <= w(12)(3)(1)(5)#2362 <= 22
 0 <= w(12)(3)(1)(6)#2363 <= 20
 0 <= w(12)(3)(1)(7)#2364 <= 20
 0 <= w(12)(3)(1)(8)#2365 <= 19
 0 <= w(12)(3)(1)(9)#2366 <= 18
 0 <= w(12)(3)(1)(10)#2367 <= 18
 0 <= w(12)(3)(2)(1)#2368 <= 21
 0 <= w(12)(3)(2)(2)#2369 <= 22
 0 <= w(12)(3)(2)(3)#2370 <= 21
 0 <= w(12)(3)(2)(4)#2371 <= 23
 0 <= w(12)(3)(2)(5)#2372 <= 19
 0 <= w(12)(3)(2)(6)#2373 <= 19
 0 <= w(12)(3)(2)(7)#2374 <= 23
 0 <= w(12)(3)(2)(8)#2375 <= 26
 0 <= w(12)(3)(2)(9)#2376 <= 25
 0 <= w(12)(3)(2)(10)#2377 <= 18
 0 <= w(12)(3)(3)(1)#2378 <= 25
 0 <= w(12)(3)(3)(2)#2379 <= 25
 0 <= w(12)(3)(3)(3)#2380 <= 21
 0 <= w(12)(3)(3)(4)#2381 <= 19
 0 <= w(12)(3)(3)(5)#2382 <= 19
 0 <= w(12)(3)(3)(6)#2383 <= 26
 0 <= w(12)(3)(3)(7)#2384 <= 25
 0 <= w(12)(3)(3)(8)#2385 <= 22
 0 <= w(12)(3)(3)(9)#2386 <= 21
 0 <= w(12)(3)(3)(10)#2387 <= 25
 0 <= w(12)(4)(1)(1)#2388 <= 21
 0 <= w(12)(4)(1)(2)#2389 <= 18
 0 <= w(12)(4)(1)(3)#2390 <= 21
 0 <= w(12)(4)(1)(4)#2391 <= 24
 0 <= w(12)(4)(1)(5)#2392 <= 22
 0 <= w(12)(4)(1)(6)#2393 <= 20
 0 <= w(12)(4)(1)(7)#2394 <= 20
 0 <= w(12)(4)(1)(8)#2395 <= 19
 0 <= w(12)(4)(1)(9)#2396 <= 18
 0 <= w(12)(4)(1)(10)#2397 <= 18
 0 <= w(12)(4)(2)(1)#2398 <= 21
 0 <= w(12)(4)(2)(2)#2399 <= 22
 0 <= w(12)(4)(2)(3)#2400 <= 21
 0 <= w(12)(4)(2)(4)#2401 <= 23
 0 <= w(12)(4)(2)(5)#2402 <= 19
 0 <= w(12)(4)(2)(6)#2403 <= 19
 0 <= w(12)(4)(2)(7)#2404 <= 23
 0 <= w(12)(4)(2)(8)#2405 <= 26
 0 <= w(12)(4)(2)(9)#2406 <= 25
 0 <= w(12)(4)(2)(10)#2407 <= 18
 0 <= w(12)(4)(3)(1)#2408 <= 25
 0 <= w(12)(4)(3)(2)#2409 <= 25
 0 <= w(12)(4)(3)(3)#2410 <= 21
 0 <= w(12)(4)(3)(4)#2411 <= 19
 0 <= w(12)(4)(3)(5)#2412 <= 19
 0 <= w(12)(4)(3)(6)#2413 <= 26
 0 <= w(12)(4)(3)(7)#2414 <= 25
 0 <= w(12)(4)(3)(8)#2415 <= 22
 0 <= w(12)(4)(3)(9)#2416 <= 21
 0 <= w(12)(4)(3)(10)#2417 <= 25
 0 <= w(12)(5)(1)(1)#2418 <= 21
 0 <= w(12)(5)(1)(2)#2419 <= 18
 0 <= w(12)(5)(1)(3)#2420 <= 21
 0 <= w(12)(5)(1)(4)#2421 <= 24
 0 <= w(12)(5)(1)(5)#2422 <= 22
 0 <= w(12)(5)(1)(6)#2423 <= 20
 0 <= w(12)(5)(1)(7)#2424 <= 20
 0 <= w(12)(5)(1)(8)#2425 <= 19
 0 <= w(12)(5)(1)(9)#2426 <= 18
 0 <= w(12)(5)(1)(10)#2427 <= 18
 0 <= w(12)(5)(2)(1)#2428 <= 21
 0 <= w(12)(5)(2)(2)#2429 <= 22
 0 <= w(12)(5)(2)(3)#2430 <= 21
 0 <= w(12)(5)(2)(4)#2431 <= 23
 0 <= w(12)(5)(2)(5)#2432 <= 19
 0 <= w(12)(5)(2)(6)#2433 <= 19
 0 <= w(12)(5)(2)(7)#2434 <= 23
 0 <= w(12)(5)(2)(8)#2435 <= 26
 0 <= w(12)(5)(2)(9)#2436 <= 25
 0 <= w(12)(5)(2)(10)#2437 <= 18
 0 <= w(12)(5)(3)(1)#2438 <= 25
 0 <= w(12)(5)(3)(2)#2439 <= 25
 0 <= w(12)(5)(3)(3)#2440 <= 21
 0 <= w(12)(5)(3)(4)#2441 <= 19
 0 <= w(12)(5)(3)(5)#2442 <= 19
 0 <= w(12)(5)(3)(6)#2443 <= 26
 0 <= w(12)(5)(3)(7)#2444 <= 25
 0 <= w(12)(5)(3)(8)#2445 <= 22
 0 <= w(12)(5)(3)(9)#2446 <= 21
 0 <= w(12)(5)(3)(10)#2447 <= 25
 0 <= w(12)(6)(1)(1)#2448 <= 21
 0 <= w(12)(6)(1)(2)#2449 <= 18
 0 <= w(12)(6)(1)(3)#2450 <= 21
 0 <= w(12)(6)(1)(4)#2451 <= 24
 0 <= w(12)(6)(1)(5)#2452 <= 22
 0 <= w(12)(6)(1)(6)#2453 <= 20
 0 <= w(12)(6)(1)(7)#2454 <= 20
 0 <= w(12)(6)(1)(8)#2455 <= 19
 0 <= w(12)(6)(1)(9)#2456 <= 18
 0 <= w(12)(6)(1)(10)#2457 <= 18
 0 <= w(12)(6)(2)(1)#2458 <= 21
 0 <= w(12)(6)(2)(2)#2459 <= 22
 0 <= w(12)(6)(2)(3)#2460 <= 21
 0 <= w(12)(6)(2)(4)#2461 <= 23
 0 <= w(12)(6)(2)(5)#2462 <= 19
 0 <= w(12)(6)(2)(6)#2463 <= 19
 0 <= w(12)(6)(2)(7)#2464 <= 23
 0 <= w(12)(6)(2)(8)#2465 <= 26
 0 <= w(12)(6)(2)(9)#2466 <= 25
 0 <= w(12)(6)(2)(10)#2467 <= 18
 0 <= w(12)(6)(3)(1)#2468 <= 25
 0 <= w(12)(6)(3)(2)#2469 <= 25
 0 <= w(12)(6)(3)(3)#2470 <= 21
 0 <= w(12)(6)(3)(4)#2471 <= 19
 0 <= w(12)(6)(3)(5)#2472 <= 19
 0 <= w(12)(6)(3)(6)#2473 <= 26
 0 <= w(12)(6)(3)(7)#2474 <= 25
 0 <= w(12)(6)(3)(8)#2475 <= 22
 0 <= w(12)(6)(3)(9)#2476 <= 21
 0 <= w(12)(6)(3)(10)#2477 <= 25
End
