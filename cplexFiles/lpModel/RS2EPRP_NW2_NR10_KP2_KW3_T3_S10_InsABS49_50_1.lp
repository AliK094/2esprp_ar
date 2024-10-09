\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 3000 y(1)#0 + 3000 y(2)#1 + 3000 y(3)#2 + 30 p(1)#3 + 30 p(2)#4
       + 30 p(3)#5 + 3 I_plant(1)#6 + 3 I_plant(2)#7 + 3 I_plant(3)#8
       + 0.4 I_warehouse(1)(1)(1)#9 + 0.4 I_warehouse(1)(1)(2)#10
       + 0.4 I_warehouse(1)(1)(3)#11 + 0.4 I_warehouse(1)(1)(4)#12
       + 0.4 I_warehouse(1)(1)(5)#13 + 0.4 I_warehouse(1)(1)(6)#14
       + 0.4 I_warehouse(1)(1)(7)#15 + 0.4 I_warehouse(1)(1)(8)#16
       + 0.4 I_warehouse(1)(1)(9)#17 + 0.4 I_warehouse(1)(1)(10)#18
       + 0.4 I_warehouse(1)(2)(1)#19 + 0.4 I_warehouse(1)(2)(2)#20
       + 0.4 I_warehouse(1)(2)(3)#21 + 0.4 I_warehouse(1)(2)(4)#22
       + 0.4 I_warehouse(1)(2)(5)#23 + 0.4 I_warehouse(1)(2)(6)#24
       + 0.4 I_warehouse(1)(2)(7)#25 + 0.4 I_warehouse(1)(2)(8)#26
       + 0.4 I_warehouse(1)(2)(9)#27 + 0.4 I_warehouse(1)(2)(10)#28
       + 0.4 I_warehouse(1)(3)(1)#29 + 0.4 I_warehouse(1)(3)(2)#30
       + 0.4 I_warehouse(1)(3)(3)#31 + 0.4 I_warehouse(1)(3)(4)#32
       + 0.4 I_warehouse(1)(3)(5)#33 + 0.4 I_warehouse(1)(3)(6)#34
       + 0.4 I_warehouse(1)(3)(7)#35 + 0.4 I_warehouse(1)(3)(8)#36
       + 0.4 I_warehouse(1)(3)(9)#37 + 0.4 I_warehouse(1)(3)(10)#38
       + 0.3 I_warehouse(2)(1)(1)#39 + 0.3 I_warehouse(2)(1)(2)#40
       + 0.3 I_warehouse(2)(1)(3)#41 + 0.3 I_warehouse(2)(1)(4)#42
       + 0.3 I_warehouse(2)(1)(5)#43 + 0.3 I_warehouse(2)(1)(6)#44
       + 0.3 I_warehouse(2)(1)(7)#45 + 0.3 I_warehouse(2)(1)(8)#46
       + 0.3 I_warehouse(2)(1)(9)#47 + 0.3 I_warehouse(2)(1)(10)#48
       + 0.3 I_warehouse(2)(2)(1)#49 + 0.3 I_warehouse(2)(2)(2)#50
       + 0.3 I_warehouse(2)(2)(3)#51 + 0.3 I_warehouse(2)(2)(4)#52
       + 0.3 I_warehouse(2)(2)(5)#53 + 0.3 I_warehouse(2)(2)(6)#54
       + 0.3 I_warehouse(2)(2)(7)#55 + 0.3 I_warehouse(2)(2)(8)#56
       + 0.3 I_warehouse(2)(2)(9)#57 + 0.3 I_warehouse(2)(2)(10)#58
       + 0.3 I_warehouse(2)(3)(1)#59 + 0.3 I_warehouse(2)(3)(2)#60
       + 0.3 I_warehouse(2)(3)(3)#61 + 0.3 I_warehouse(2)(3)(4)#62
       + 0.3 I_warehouse(2)(3)(5)#63 + 0.3 I_warehouse(2)(3)(6)#64
       + 0.3 I_warehouse(2)(3)(7)#65 + 0.3 I_warehouse(2)(3)(8)#66
       + 0.3 I_warehouse(2)(3)(9)#67 + 0.3 I_warehouse(2)(3)(10)#68
       + 0.6 I_customer(3)(1)(1)#69 + 0.6 I_customer(3)(1)(2)#70
       + 0.6 I_customer(3)(1)(3)#71 + 0.6 I_customer(3)(1)(4)#72
       + 0.6 I_customer(3)(1)(5)#73 + 0.6 I_customer(3)(1)(6)#74
       + 0.6 I_customer(3)(1)(7)#75 + 0.6 I_customer(3)(1)(8)#76
       + 0.6 I_customer(3)(1)(9)#77 + 0.6 I_customer(3)(1)(10)#78
       + 0.6 I_customer(3)(2)(1)#79 + 0.6 I_customer(3)(2)(2)#80
       + 0.6 I_customer(3)(2)(3)#81 + 0.6 I_customer(3)(2)(4)#82
       + 0.6 I_customer(3)(2)(5)#83 + 0.6 I_customer(3)(2)(6)#84
       + 0.6 I_customer(3)(2)(7)#85 + 0.6 I_customer(3)(2)(8)#86
       + 0.6 I_customer(3)(2)(9)#87 + 0.6 I_customer(3)(2)(10)#88
       + 0.6 I_customer(3)(3)(1)#89 + 0.6 I_customer(3)(3)(2)#90
       + 0.6 I_customer(3)(3)(3)#91 + 0.6 I_customer(3)(3)(4)#92
       + 0.6 I_customer(3)(3)(5)#93 + 0.6 I_customer(3)(3)(6)#94
       + 0.6 I_customer(3)(3)(7)#95 + 0.6 I_customer(3)(3)(8)#96
       + 0.6 I_customer(3)(3)(9)#97 + 0.6 I_customer(3)(3)(10)#98
       + 0.9 I_customer(4)(1)(1)#99 + 0.9 I_customer(4)(1)(2)#100
       + 0.9 I_customer(4)(1)(3)#101 + 0.9 I_customer(4)(1)(4)#102
       + 0.9 I_customer(4)(1)(5)#103 + 0.9 I_customer(4)(1)(6)#104
       + 0.9 I_customer(4)(1)(7)#105 + 0.9 I_customer(4)(1)(8)#106
       + 0.9 I_customer(4)(1)(9)#107 + 0.9 I_customer(4)(1)(10)#108
       + 0.9 I_customer(4)(2)(1)#109 + 0.9 I_customer(4)(2)(2)#110
       + 0.9 I_customer(4)(2)(3)#111 + 0.9 I_customer(4)(2)(4)#112
       + 0.9 I_customer(4)(2)(5)#113 + 0.9 I_customer(4)(2)(6)#114
       + 0.9 I_customer(4)(2)(7)#115 + 0.9 I_customer(4)(2)(8)#116
       + 0.9 I_customer(4)(2)(9)#117 + 0.9 I_customer(4)(2)(10)#118
       + 0.9 I_customer(4)(3)(1)#119 + 0.9 I_customer(4)(3)(2)#120
       + 0.9 I_customer(4)(3)(3)#121 + 0.9 I_customer(4)(3)(4)#122
       + 0.9 I_customer(4)(3)(5)#123 + 0.9 I_customer(4)(3)(6)#124
       + 0.9 I_customer(4)(3)(7)#125 + 0.9 I_customer(4)(3)(8)#126
       + 0.9 I_customer(4)(3)(9)#127 + 0.9 I_customer(4)(3)(10)#128
       + 0.7 I_customer(5)(1)(1)#129 + 0.7 I_customer(5)(1)(2)#130
       + 0.7 I_customer(5)(1)(3)#131 + 0.7 I_customer(5)(1)(4)#132
       + 0.7 I_customer(5)(1)(5)#133 + 0.7 I_customer(5)(1)(6)#134
       + 0.7 I_customer(5)(1)(7)#135 + 0.7 I_customer(5)(1)(8)#136
       + 0.7 I_customer(5)(1)(9)#137 + 0.7 I_customer(5)(1)(10)#138
       + 0.7 I_customer(5)(2)(1)#139 + 0.7 I_customer(5)(2)(2)#140
       + 0.7 I_customer(5)(2)(3)#141 + 0.7 I_customer(5)(2)(4)#142
       + 0.7 I_customer(5)(2)(5)#143 + 0.7 I_customer(5)(2)(6)#144
       + 0.7 I_customer(5)(2)(7)#145 + 0.7 I_customer(5)(2)(8)#146
       + 0.7 I_customer(5)(2)(9)#147 + 0.7 I_customer(5)(2)(10)#148
       + 0.7 I_customer(5)(3)(1)#149 + 0.7 I_customer(5)(3)(2)#150
       + 0.7 I_customer(5)(3)(3)#151 + 0.7 I_customer(5)(3)(4)#152
       + 0.7 I_customer(5)(3)(5)#153 + 0.7 I_customer(5)(3)(6)#154
       + 0.7 I_customer(5)(3)(7)#155 + 0.7 I_customer(5)(3)(8)#156
       + 0.7 I_customer(5)(3)(9)#157 + 0.7 I_customer(5)(3)(10)#158
       + 0.8 I_customer(6)(1)(1)#159 + 0.8 I_customer(6)(1)(2)#160
       + 0.8 I_customer(6)(1)(3)#161 + 0.8 I_customer(6)(1)(4)#162
       + 0.8 I_customer(6)(1)(5)#163 + 0.8 I_customer(6)(1)(6)#164
       + 0.8 I_customer(6)(1)(7)#165 + 0.8 I_customer(6)(1)(8)#166
       + 0.8 I_customer(6)(1)(9)#167 + 0.8 I_customer(6)(1)(10)#168
       + 0.8 I_customer(6)(2)(1)#169 + 0.8 I_customer(6)(2)(2)#170
       + 0.8 I_customer(6)(2)(3)#171 + 0.8 I_customer(6)(2)(4)#172
       + 0.8 I_customer(6)(2)(5)#173 + 0.8 I_customer(6)(2)(6)#174
       + 0.8 I_customer(6)(2)(7)#175 + 0.8 I_customer(6)(2)(8)#176
       + 0.8 I_customer(6)(2)(9)#177 + 0.8 I_customer(6)(2)(10)#178
       + 0.8 I_customer(6)(3)(1)#179 + 0.8 I_customer(6)(3)(2)#180
       + 0.8 I_customer(6)(3)(3)#181 + 0.8 I_customer(6)(3)(4)#182
       + 0.8 I_customer(6)(3)(5)#183 + 0.8 I_customer(6)(3)(6)#184
       + 0.8 I_customer(6)(3)(7)#185 + 0.8 I_customer(6)(3)(8)#186
       + 0.8 I_customer(6)(3)(9)#187 + 0.8 I_customer(6)(3)(10)#188
       + 0.6 I_customer(7)(1)(1)#189 + 0.6 I_customer(7)(1)(2)#190
       + 0.6 I_customer(7)(1)(3)#191 + 0.6 I_customer(7)(1)(4)#192
       + 0.6 I_customer(7)(1)(5)#193 + 0.6 I_customer(7)(1)(6)#194
       + 0.6 I_customer(7)(1)(7)#195 + 0.6 I_customer(7)(1)(8)#196
       + 0.6 I_customer(7)(1)(9)#197 + 0.6 I_customer(7)(1)(10)#198
       + 0.6 I_customer(7)(2)(1)#199 + 0.6 I_customer(7)(2)(2)#200
       + 0.6 I_customer(7)(2)(3)#201 + 0.6 I_customer(7)(2)(4)#202
       + 0.6 I_customer(7)(2)(5)#203 + 0.6 I_customer(7)(2)(6)#204
       + 0.6 I_customer(7)(2)(7)#205 + 0.6 I_customer(7)(2)(8)#206
       + 0.6 I_customer(7)(2)(9)#207 + 0.6 I_customer(7)(2)(10)#208
       + 0.6 I_customer(7)(3)(1)#209 + 0.6 I_customer(7)(3)(2)#210
       + 0.6 I_customer(7)(3)(3)#211 + 0.6 I_customer(7)(3)(4)#212
       + 0.6 I_customer(7)(3)(5)#213 + 0.6 I_customer(7)(3)(6)#214
       + 0.6 I_customer(7)(3)(7)#215 + 0.6 I_customer(7)(3)(8)#216
       + 0.6 I_customer(7)(3)(9)#217 + 0.6 I_customer(7)(3)(10)#218
       + 0.6 I_customer(8)(1)(1)#219 + 0.6 I_customer(8)(1)(2)#220
       + 0.6 I_customer(8)(1)(3)#221 + 0.6 I_customer(8)(1)(4)#222
       + 0.6 I_customer(8)(1)(5)#223 + 0.6 I_customer(8)(1)(6)#224
       + 0.6 I_customer(8)(1)(7)#225 + 0.6 I_customer(8)(1)(8)#226
       + 0.6 I_customer(8)(1)(9)#227 + 0.6 I_customer(8)(1)(10)#228
       + 0.6 I_customer(8)(2)(1)#229 + 0.6 I_customer(8)(2)(2)#230
       + 0.6 I_customer(8)(2)(3)#231 + 0.6 I_customer(8)(2)(4)#232
       + 0.6 I_customer(8)(2)(5)#233 + 0.6 I_customer(8)(2)(6)#234
       + 0.6 I_customer(8)(2)(7)#235 + 0.6 I_customer(8)(2)(8)#236
       + 0.6 I_customer(8)(2)(9)#237 + 0.6 I_customer(8)(2)(10)#238
       + 0.6 I_customer(8)(3)(1)#239 + 0.6 I_customer(8)(3)(2)#240
       + 0.6 I_customer(8)(3)(3)#241 + 0.6 I_customer(8)(3)(4)#242
       + 0.6 I_customer(8)(3)(5)#243 + 0.6 I_customer(8)(3)(6)#244
       + 0.6 I_customer(8)(3)(7)#245 + 0.6 I_customer(8)(3)(8)#246
       + 0.6 I_customer(8)(3)(9)#247 + 0.6 I_customer(8)(3)(10)#248
       + 0.8 I_customer(9)(1)(1)#249 + 0.8 I_customer(9)(1)(2)#250
       + 0.8 I_customer(9)(1)(3)#251 + 0.8 I_customer(9)(1)(4)#252
       + 0.8 I_customer(9)(1)(5)#253 + 0.8 I_customer(9)(1)(6)#254
       + 0.8 I_customer(9)(1)(7)#255 + 0.8 I_customer(9)(1)(8)#256
       + 0.8 I_customer(9)(1)(9)#257 + 0.8 I_customer(9)(1)(10)#258
       + 0.8 I_customer(9)(2)(1)#259 + 0.8 I_customer(9)(2)(2)#260
       + 0.8 I_customer(9)(2)(3)#261 + 0.8 I_customer(9)(2)(4)#262
       + 0.8 I_customer(9)(2)(5)#263 + 0.8 I_customer(9)(2)(6)#264
       + 0.8 I_customer(9)(2)(7)#265 + 0.8 I_customer(9)(2)(8)#266
       + 0.8 I_customer(9)(2)(9)#267 + 0.8 I_customer(9)(2)(10)#268
       + 0.8 I_customer(9)(3)(1)#269 + 0.8 I_customer(9)(3)(2)#270
       + 0.8 I_customer(9)(3)(3)#271 + 0.8 I_customer(9)(3)(4)#272
       + 0.8 I_customer(9)(3)(5)#273 + 0.8 I_customer(9)(3)(6)#274
       + 0.8 I_customer(9)(3)(7)#275 + 0.8 I_customer(9)(3)(8)#276
       + 0.8 I_customer(9)(3)(9)#277 + 0.8 I_customer(9)(3)(10)#278
       + 0.8 I_customer(10)(1)(1)#279 + 0.8 I_customer(10)(1)(2)#280
       + 0.8 I_customer(10)(1)(3)#281 + 0.8 I_customer(10)(1)(4)#282
       + 0.8 I_customer(10)(1)(5)#283 + 0.8 I_customer(10)(1)(6)#284
       + 0.8 I_customer(10)(1)(7)#285 + 0.8 I_customer(10)(1)(8)#286
       + 0.8 I_customer(10)(1)(9)#287 + 0.8 I_customer(10)(1)(10)#288
       + 0.8 I_customer(10)(2)(1)#289 + 0.8 I_customer(10)(2)(2)#290
       + 0.8 I_customer(10)(2)(3)#291 + 0.8 I_customer(10)(2)(4)#292
       + 0.8 I_customer(10)(2)(5)#293 + 0.8 I_customer(10)(2)(6)#294
       + 0.8 I_customer(10)(2)(7)#295 + 0.8 I_customer(10)(2)(8)#296
       + 0.8 I_customer(10)(2)(9)#297 + 0.8 I_customer(10)(2)(10)#298
       + 0.8 I_customer(10)(3)(1)#299 + 0.8 I_customer(10)(3)(2)#300
       + 0.8 I_customer(10)(3)(3)#301 + 0.8 I_customer(10)(3)(4)#302
       + 0.8 I_customer(10)(3)(5)#303 + 0.8 I_customer(10)(3)(6)#304
       + 0.8 I_customer(10)(3)(7)#305 + 0.8 I_customer(10)(3)(8)#306
       + 0.8 I_customer(10)(3)(9)#307 + 0.8 I_customer(10)(3)(10)#308
       + 0.9 I_customer(11)(1)(1)#309 + 0.9 I_customer(11)(1)(2)#310
       + 0.9 I_customer(11)(1)(3)#311 + 0.9 I_customer(11)(1)(4)#312
       + 0.9 I_customer(11)(1)(5)#313 + 0.9 I_customer(11)(1)(6)#314
       + 0.9 I_customer(11)(1)(7)#315 + 0.9 I_customer(11)(1)(8)#316
       + 0.9 I_customer(11)(1)(9)#317 + 0.9 I_customer(11)(1)(10)#318
       + 0.9 I_customer(11)(2)(1)#319 + 0.9 I_customer(11)(2)(2)#320
       + 0.9 I_customer(11)(2)(3)#321 + 0.9 I_customer(11)(2)(4)#322
       + 0.9 I_customer(11)(2)(5)#323 + 0.9 I_customer(11)(2)(6)#324
       + 0.9 I_customer(11)(2)(7)#325 + 0.9 I_customer(11)(2)(8)#326
       + 0.9 I_customer(11)(2)(9)#327 + 0.9 I_customer(11)(2)(10)#328
       + 0.9 I_customer(11)(3)(1)#329 + 0.9 I_customer(11)(3)(2)#330
       + 0.9 I_customer(11)(3)(3)#331 + 0.9 I_customer(11)(3)(4)#332
       + 0.9 I_customer(11)(3)(5)#333 + 0.9 I_customer(11)(3)(6)#334
       + 0.9 I_customer(11)(3)(7)#335 + 0.9 I_customer(11)(3)(8)#336
       + 0.9 I_customer(11)(3)(9)#337 + 0.9 I_customer(11)(3)(10)#338
       + 0.7 I_customer(12)(1)(1)#339 + 0.7 I_customer(12)(1)(2)#340
       + 0.7 I_customer(12)(1)(3)#341 + 0.7 I_customer(12)(1)(4)#342
       + 0.7 I_customer(12)(1)(5)#343 + 0.7 I_customer(12)(1)(6)#344
       + 0.7 I_customer(12)(1)(7)#345 + 0.7 I_customer(12)(1)(8)#346
       + 0.7 I_customer(12)(1)(9)#347 + 0.7 I_customer(12)(1)(10)#348
       + 0.7 I_customer(12)(2)(1)#349 + 0.7 I_customer(12)(2)(2)#350
       + 0.7 I_customer(12)(2)(3)#351 + 0.7 I_customer(12)(2)(4)#352
       + 0.7 I_customer(12)(2)(5)#353 + 0.7 I_customer(12)(2)(6)#354
       + 0.7 I_customer(12)(2)(7)#355 + 0.7 I_customer(12)(2)(8)#356
       + 0.7 I_customer(12)(2)(9)#357 + 0.7 I_customer(12)(2)(10)#358
       + 0.7 I_customer(12)(3)(1)#359 + 0.7 I_customer(12)(3)(2)#360
       + 0.7 I_customer(12)(3)(3)#361 + 0.7 I_customer(12)(3)(4)#362
       + 0.7 I_customer(12)(3)(5)#363 + 0.7 I_customer(12)(3)(6)#364
       + 0.7 I_customer(12)(3)(7)#365 + 0.7 I_customer(12)(3)(8)#366
       + 0.7 I_customer(12)(3)(9)#367 + 0.7 I_customer(12)(3)(10)#368
       + 104 b_customer(3)(1)(1)#369 + 104 b_customer(3)(1)(2)#370
       + 104 b_customer(3)(1)(3)#371 + 104 b_customer(3)(1)(4)#372
       + 104 b_customer(3)(1)(5)#373 + 104 b_customer(3)(1)(6)#374
       + 104 b_customer(3)(1)(7)#375 + 104 b_customer(3)(1)(8)#376
       + 104 b_customer(3)(1)(9)#377 + 104 b_customer(3)(1)(10)#378
       + 104 b_customer(3)(2)(1)#379 + 104 b_customer(3)(2)(2)#380
       + 104 b_customer(3)(2)(3)#381 + 104 b_customer(3)(2)(4)#382
       + 104 b_customer(3)(2)(5)#383 + 104 b_customer(3)(2)(6)#384
       + 104 b_customer(3)(2)(7)#385 + 104 b_customer(3)(2)(8)#386
       + 104 b_customer(3)(2)(9)#387 + 104 b_customer(3)(2)(10)#388
       + 104 b_customer(3)(3)(1)#389 + 104 b_customer(3)(3)(2)#390
       + 104 b_customer(3)(3)(3)#391 + 104 b_customer(3)(3)(4)#392
       + 104 b_customer(3)(3)(5)#393 + 104 b_customer(3)(3)(6)#394
       + 104 b_customer(3)(3)(7)#395 + 104 b_customer(3)(3)(8)#396
       + 104 b_customer(3)(3)(9)#397 + 104 b_customer(3)(3)(10)#398
       + 75 b_customer(4)(1)(1)#399 + 75 b_customer(4)(1)(2)#400
       + 75 b_customer(4)(1)(3)#401 + 75 b_customer(4)(1)(4)#402
       + 75 b_customer(4)(1)(5)#403 + 75 b_customer(4)(1)(6)#404
       + 75 b_customer(4)(1)(7)#405 + 75 b_customer(4)(1)(8)#406
       + 75 b_customer(4)(1)(9)#407 + 75 b_customer(4)(1)(10)#408
       + 75 b_customer(4)(2)(1)#409 + 75 b_customer(4)(2)(2)#410
       + 75 b_customer(4)(2)(3)#411 + 75 b_customer(4)(2)(4)#412
       + 75 b_customer(4)(2)(5)#413 + 75 b_customer(4)(2)(6)#414
       + 75 b_customer(4)(2)(7)#415 + 75 b_customer(4)(2)(8)#416
       + 75 b_customer(4)(2)(9)#417 + 75 b_customer(4)(2)(10)#418
       + 75 b_customer(4)(3)(1)#419 + 75 b_customer(4)(3)(2)#420
       + 75 b_customer(4)(3)(3)#421 + 75 b_customer(4)(3)(4)#422
       + 75 b_customer(4)(3)(5)#423 + 75 b_customer(4)(3)(6)#424
       + 75 b_customer(4)(3)(7)#425 + 75 b_customer(4)(3)(8)#426
       + 75 b_customer(4)(3)(9)#427 + 75 b_customer(4)(3)(10)#428
       + 71 b_customer(5)(1)(1)#429 + 71 b_customer(5)(1)(2)#430
       + 71 b_customer(5)(1)(3)#431 + 71 b_customer(5)(1)(4)#432
       + 71 b_customer(5)(1)(5)#433 + 71 b_customer(5)(1)(6)#434
       + 71 b_customer(5)(1)(7)#435 + 71 b_customer(5)(1)(8)#436
       + 71 b_customer(5)(1)(9)#437 + 71 b_customer(5)(1)(10)#438
       + 71 b_customer(5)(2)(1)#439 + 71 b_customer(5)(2)(2)#440
       + 71 b_customer(5)(2)(3)#441 + 71 b_customer(5)(2)(4)#442
       + 71 b_customer(5)(2)(5)#443 + 71 b_customer(5)(2)(6)#444
       + 71 b_customer(5)(2)(7)#445 + 71 b_customer(5)(2)(8)#446
       + 71 b_customer(5)(2)(9)#447 + 71 b_customer(5)(2)(10)#448
       + 71 b_customer(5)(3)(1)#449 + 71 b_customer(5)(3)(2)#450
       + 71 b_customer(5)(3)(3)#451 + 71 b_customer(5)(3)(4)#452
       + 71 b_customer(5)(3)(5)#453 + 71 b_customer(5)(3)(6)#454
       + 71 b_customer(5)(3)(7)#455 + 71 b_customer(5)(3)(8)#456
       + 71 b_customer(5)(3)(9)#457 + 71 b_customer(5)(3)(10)#458
       + 130 b_customer(6)(1)(1)#459 + 130 b_customer(6)(1)(2)#460
       + 130 b_customer(6)(1)(3)#461 + 130 b_customer(6)(1)(4)#462
       + 130 b_customer(6)(1)(5)#463 + 130 b_customer(6)(1)(6)#464
       + 130 b_customer(6)(1)(7)#465 + 130 b_customer(6)(1)(8)#466
       + 130 b_customer(6)(1)(9)#467 + 130 b_customer(6)(1)(10)#468
       + 130 b_customer(6)(2)(1)#469 + 130 b_customer(6)(2)(2)#470
       + 130 b_customer(6)(2)(3)#471 + 130 b_customer(6)(2)(4)#472
       + 130 b_customer(6)(2)(5)#473 + 130 b_customer(6)(2)(6)#474
       + 130 b_customer(6)(2)(7)#475 + 130 b_customer(6)(2)(8)#476
       + 130 b_customer(6)(2)(9)#477 + 130 b_customer(6)(2)(10)#478
       + 130 b_customer(6)(3)(1)#479 + 130 b_customer(6)(3)(2)#480
       + 130 b_customer(6)(3)(3)#481 + 130 b_customer(6)(3)(4)#482
       + 130 b_customer(6)(3)(5)#483 + 130 b_customer(6)(3)(6)#484
       + 130 b_customer(6)(3)(7)#485 + 130 b_customer(6)(3)(8)#486
       + 130 b_customer(6)(3)(9)#487 + 130 b_customer(6)(3)(10)#488
       + 67 b_customer(7)(1)(1)#489 + 67 b_customer(7)(1)(2)#490
       + 67 b_customer(7)(1)(3)#491 + 67 b_customer(7)(1)(4)#492
       + 67 b_customer(7)(1)(5)#493 + 67 b_customer(7)(1)(6)#494
       + 67 b_customer(7)(1)(7)#495 + 67 b_customer(7)(1)(8)#496
       + 67 b_customer(7)(1)(9)#497 + 67 b_customer(7)(1)(10)#498
       + 67 b_customer(7)(2)(1)#499 + 67 b_customer(7)(2)(2)#500
       + 67 b_customer(7)(2)(3)#501 + 67 b_customer(7)(2)(4)#502
       + 67 b_customer(7)(2)(5)#503 + 67 b_customer(7)(2)(6)#504
       + 67 b_customer(7)(2)(7)#505 + 67 b_customer(7)(2)(8)#506
       + 67 b_customer(7)(2)(9)#507 + 67 b_customer(7)(2)(10)#508
       + 67 b_customer(7)(3)(1)#509 + 67 b_customer(7)(3)(2)#510
       + 67 b_customer(7)(3)(3)#511 + 67 b_customer(7)(3)(4)#512
       + 67 b_customer(7)(3)(5)#513 + 67 b_customer(7)(3)(6)#514
       + 67 b_customer(7)(3)(7)#515 + 67 b_customer(7)(3)(8)#516
       + 67 b_customer(7)(3)(9)#517 + 67 b_customer(7)(3)(10)#518
       + 117 b_customer(8)(1)(1)#519 + 117 b_customer(8)(1)(2)#520
       + 117 b_customer(8)(1)(3)#521 + 117 b_customer(8)(1)(4)#522
       + 117 b_customer(8)(1)(5)#523 + 117 b_customer(8)(1)(6)#524
       + 117 b_customer(8)(1)(7)#525 + 117 b_customer(8)(1)(8)#526
       + 117 b_customer(8)(1)(9)#527 + 117 b_customer(8)(1)(10)#528
       + 117 b_customer(8)(2)(1)#529 + 117 b_customer(8)(2)(2)#530
       + 117 b_customer(8)(2)(3)#531 + 117 b_customer(8)(2)(4)#532
       + 117 b_customer(8)(2)(5)#533 + 117 b_customer(8)(2)(6)#534
       + 117 b_customer(8)(2)(7)#535 + 117 b_customer(8)(2)(8)#536
       + 117 b_customer(8)(2)(9)#537 + 117 b_customer(8)(2)(10)#538
       + 117 b_customer(8)(3)(1)#539 + 117 b_customer(8)(3)(2)#540
       + 117 b_customer(8)(3)(3)#541 + 117 b_customer(8)(3)(4)#542
       + 117 b_customer(8)(3)(5)#543 + 117 b_customer(8)(3)(6)#544
       + 117 b_customer(8)(3)(7)#545 + 117 b_customer(8)(3)(8)#546
       + 117 b_customer(8)(3)(9)#547 + 117 b_customer(8)(3)(10)#548
       + 84 b_customer(9)(1)(1)#549 + 84 b_customer(9)(1)(2)#550
       + 84 b_customer(9)(1)(3)#551 + 84 b_customer(9)(1)(4)#552
       + 84 b_customer(9)(1)(5)#553 + 84 b_customer(9)(1)(6)#554
       + 84 b_customer(9)(1)(7)#555 + 84 b_customer(9)(1)(8)#556
       + 84 b_customer(9)(1)(9)#557 + 84 b_customer(9)(1)(10)#558
       + 84 b_customer(9)(2)(1)#559 + 84 b_customer(9)(2)(2)#560
       + 84 b_customer(9)(2)(3)#561 + 84 b_customer(9)(2)(4)#562
       + 84 b_customer(9)(2)(5)#563 + 84 b_customer(9)(2)(6)#564
       + 84 b_customer(9)(2)(7)#565 + 84 b_customer(9)(2)(8)#566
       + 84 b_customer(9)(2)(9)#567 + 84 b_customer(9)(2)(10)#568
       + 84 b_customer(9)(3)(1)#569 + 84 b_customer(9)(3)(2)#570
       + 84 b_customer(9)(3)(3)#571 + 84 b_customer(9)(3)(4)#572
       + 84 b_customer(9)(3)(5)#573 + 84 b_customer(9)(3)(6)#574
       + 84 b_customer(9)(3)(7)#575 + 84 b_customer(9)(3)(8)#576
       + 84 b_customer(9)(3)(9)#577 + 84 b_customer(9)(3)(10)#578
       + 124 b_customer(10)(1)(1)#579 + 124 b_customer(10)(1)(2)#580
       + 124 b_customer(10)(1)(3)#581 + 124 b_customer(10)(1)(4)#582
       + 124 b_customer(10)(1)(5)#583 + 124 b_customer(10)(1)(6)#584
       + 124 b_customer(10)(1)(7)#585 + 124 b_customer(10)(1)(8)#586
       + 124 b_customer(10)(1)(9)#587 + 124 b_customer(10)(1)(10)#588
       + 124 b_customer(10)(2)(1)#589 + 124 b_customer(10)(2)(2)#590
       + 124 b_customer(10)(2)(3)#591 + 124 b_customer(10)(2)(4)#592
       + 124 b_customer(10)(2)(5)#593 + 124 b_customer(10)(2)(6)#594
       + 124 b_customer(10)(2)(7)#595 + 124 b_customer(10)(2)(8)#596
       + 124 b_customer(10)(2)(9)#597 + 124 b_customer(10)(2)(10)#598
       + 124 b_customer(10)(3)(1)#599 + 124 b_customer(10)(3)(2)#600
       + 124 b_customer(10)(3)(3)#601 + 124 b_customer(10)(3)(4)#602
       + 124 b_customer(10)(3)(5)#603 + 124 b_customer(10)(3)(6)#604
       + 124 b_customer(10)(3)(7)#605 + 124 b_customer(10)(3)(8)#606
       + 124 b_customer(10)(3)(9)#607 + 124 b_customer(10)(3)(10)#608
       + 101 b_customer(11)(1)(1)#609 + 101 b_customer(11)(1)(2)#610
       + 101 b_customer(11)(1)(3)#611 + 101 b_customer(11)(1)(4)#612
       + 101 b_customer(11)(1)(5)#613 + 101 b_customer(11)(1)(6)#614
       + 101 b_customer(11)(1)(7)#615 + 101 b_customer(11)(1)(8)#616
       + 101 b_customer(11)(1)(9)#617 + 101 b_customer(11)(1)(10)#618
       + 101 b_customer(11)(2)(1)#619 + 101 b_customer(11)(2)(2)#620
       + 101 b_customer(11)(2)(3)#621 + 101 b_customer(11)(2)(4)#622
       + 101 b_customer(11)(2)(5)#623 + 101 b_customer(11)(2)(6)#624
       + 101 b_customer(11)(2)(7)#625 + 101 b_customer(11)(2)(8)#626
       + 101 b_customer(11)(2)(9)#627 + 101 b_customer(11)(2)(10)#628
       + 101 b_customer(11)(3)(1)#629 + 101 b_customer(11)(3)(2)#630
       + 101 b_customer(11)(3)(3)#631 + 101 b_customer(11)(3)(4)#632
       + 101 b_customer(11)(3)(5)#633 + 101 b_customer(11)(3)(6)#634
       + 101 b_customer(11)(3)(7)#635 + 101 b_customer(11)(3)(8)#636
       + 101 b_customer(11)(3)(9)#637 + 101 b_customer(11)(3)(10)#638
       + 76 b_customer(12)(1)(1)#639 + 76 b_customer(12)(1)(2)#640
       + 76 b_customer(12)(1)(3)#641 + 76 b_customer(12)(1)(4)#642
       + 76 b_customer(12)(1)(5)#643 + 76 b_customer(12)(1)(6)#644
       + 76 b_customer(12)(1)(7)#645 + 76 b_customer(12)(1)(8)#646
       + 76 b_customer(12)(1)(9)#647 + 76 b_customer(12)(1)(10)#648
       + 76 b_customer(12)(2)(1)#649 + 76 b_customer(12)(2)(2)#650
       + 76 b_customer(12)(2)(3)#651 + 76 b_customer(12)(2)(4)#652
       + 76 b_customer(12)(2)(5)#653 + 76 b_customer(12)(2)(6)#654
       + 76 b_customer(12)(2)(7)#655 + 76 b_customer(12)(2)(8)#656
       + 76 b_customer(12)(2)(9)#657 + 76 b_customer(12)(2)(10)#658
       + 76 b_customer(12)(3)(1)#659 + 76 b_customer(12)(3)(2)#660
       + 76 b_customer(12)(3)(3)#661 + 76 b_customer(12)(3)(4)#662
       + 76 b_customer(12)(3)(5)#663 + 76 b_customer(12)(3)(6)#664
       + 76 b_customer(12)(3)(7)#665 + 76 b_customer(12)(3)(8)#666
       + 76 b_customer(12)(3)(9)#667 + 76 b_customer(12)(3)(10)#668
       + 2838.94205647104 o(1)(1)#687 + 2838.94205647104 o(1)(2)#688
       + 2838.94205647104 o(1)(3)#689 + 1739.25271309261 o(2)(1)#690
       + 1739.25271309261 o(2)(2)#691 + 1739.25271309261 o(2)(3)#692
       + 4037.75629949332 o(3)(1)#693 + 4037.75629949332 o(3)(2)#694
       + 4037.75629949332 o(3)(3)#695
Subject To
 ProductionCapacity(1):          - 304 y(1)#0 + p(1)#3 <= 0
 ProductionCapacity(2):          - 304 y(2)#1 + p(2)#4 <= 0
 ProductionCapacity(3):          - 304 y(3)#2 + p(3)#5 <= 0
 PlantInventoryCapacity(1):      I_plant(1)#6 <= 152
 PlantInventoryCapacity(2):      I_plant(2)#7 <= 152
 PlantInventoryCapacity(3):      I_plant(3)#8 <= 152
 PlantInventoryBalance(1):       - p(1)#3 + I_plant(1)#6 + q(1)(1)(1)#669
                                 + q(1)(2)(1)#672 + q(2)(1)(1)#675
                                 + q(2)(2)(1)#678 + q(3)(1)(1)#681
                                 + q(3)(2)(1)#684  = 0
 PlantInventoryBalance(2):       - p(2)#4 - I_plant(1)#6 + I_plant(2)#7
                                 + q(1)(1)(2)#670 + q(1)(2)(2)#673
                                 + q(2)(1)(2)#676 + q(2)(2)(2)#679
                                 + q(3)(1)(2)#682 + q(3)(2)(2)#685  = 0
 PlantInventoryBalance(3):       - p(3)#5 - I_plant(2)#7 + I_plant(3)#8
                                 + q(1)(1)(3)#671 + q(1)(2)(3)#674
                                 + q(2)(1)(3)#677 + q(2)(2)(3)#680
                                 + q(3)(1)(3)#683 + q(3)(2)(3)#686  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(1)(1)(1)#9 <= 136.8
 WarehouseInventoryCapacity(2,1,1): I_warehouse(2)(1)(1)#39 <= 136.8
 WarehouseInventoryCapacity(1,2,1): I_warehouse(1)(2)(1)#19 <= 136.8
 WarehouseInventoryCapacity(2,2,1): I_warehouse(2)(2)(1)#49 <= 136.8
 WarehouseInventoryCapacity(1,3,1): I_warehouse(1)(3)(1)#29 <= 136.8
 WarehouseInventoryCapacity(2,3,1): I_warehouse(2)(3)(1)#59 <= 136.8
 WarehouseInventoryCapacity(1,1,2): I_warehouse(1)(1)(2)#10 <= 136.8
 WarehouseInventoryCapacity(2,1,2): I_warehouse(2)(1)(2)#40 <= 136.8
 WarehouseInventoryCapacity(1,2,2): I_warehouse(1)(2)(2)#20 <= 136.8
 WarehouseInventoryCapacity(2,2,2): I_warehouse(2)(2)(2)#50 <= 136.8
 WarehouseInventoryCapacity(1,3,2): I_warehouse(1)(3)(2)#30 <= 136.8
 WarehouseInventoryCapacity(2,3,2): I_warehouse(2)(3)(2)#60 <= 136.8
 WarehouseInventoryCapacity(1,1,3): I_warehouse(1)(1)(3)#11 <= 136.8
 WarehouseInventoryCapacity(2,1,3): I_warehouse(2)(1)(3)#41 <= 136.8
 WarehouseInventoryCapacity(1,2,3): I_warehouse(1)(2)(3)#21 <= 136.8
 WarehouseInventoryCapacity(2,2,3): I_warehouse(2)(2)(3)#51 <= 136.8
 WarehouseInventoryCapacity(1,3,3): I_warehouse(1)(3)(3)#31 <= 136.8
 WarehouseInventoryCapacity(2,3,3): I_warehouse(2)(3)(3)#61 <= 136.8
 WarehouseInventoryCapacity(1,1,4): I_warehouse(1)(1)(4)#12 <= 136.8
 WarehouseInventoryCapacity(2,1,4): I_warehouse(2)(1)(4)#42 <= 136.8
 WarehouseInventoryCapacity(1,2,4): I_warehouse(1)(2)(4)#22 <= 136.8
 WarehouseInventoryCapacity(2,2,4): I_warehouse(2)(2)(4)#52 <= 136.8
 WarehouseInventoryCapacity(1,3,4): I_warehouse(1)(3)(4)#32 <= 136.8
 WarehouseInventoryCapacity(2,3,4): I_warehouse(2)(3)(4)#62 <= 136.8
 WarehouseInventoryCapacity(1,1,5): I_warehouse(1)(1)(5)#13 <= 136.8
 WarehouseInventoryCapacity(2,1,5): I_warehouse(2)(1)(5)#43 <= 136.8
 WarehouseInventoryCapacity(1,2,5): I_warehouse(1)(2)(5)#23 <= 136.8
 WarehouseInventoryCapacity(2,2,5): I_warehouse(2)(2)(5)#53 <= 136.8
 WarehouseInventoryCapacity(1,3,5): I_warehouse(1)(3)(5)#33 <= 136.8
 WarehouseInventoryCapacity(2,3,5): I_warehouse(2)(3)(5)#63 <= 136.8
 WarehouseInventoryCapacity(1,1,6): I_warehouse(1)(1)(6)#14 <= 136.8
 WarehouseInventoryCapacity(2,1,6): I_warehouse(2)(1)(6)#44 <= 136.8
 WarehouseInventoryCapacity(1,2,6): I_warehouse(1)(2)(6)#24 <= 136.8
 WarehouseInventoryCapacity(2,2,6): I_warehouse(2)(2)(6)#54 <= 136.8
 WarehouseInventoryCapacity(1,3,6): I_warehouse(1)(3)(6)#34 <= 136.8
 WarehouseInventoryCapacity(2,3,6): I_warehouse(2)(3)(6)#64 <= 136.8
 WarehouseInventoryCapacity(1,1,7): I_warehouse(1)(1)(7)#15 <= 136.8
 WarehouseInventoryCapacity(2,1,7): I_warehouse(2)(1)(7)#45 <= 136.8
 WarehouseInventoryCapacity(1,2,7): I_warehouse(1)(2)(7)#25 <= 136.8
 WarehouseInventoryCapacity(2,2,7): I_warehouse(2)(2)(7)#55 <= 136.8
 WarehouseInventoryCapacity(1,3,7): I_warehouse(1)(3)(7)#35 <= 136.8
 WarehouseInventoryCapacity(2,3,7): I_warehouse(2)(3)(7)#65 <= 136.8
 WarehouseInventoryCapacity(1,1,8): I_warehouse(1)(1)(8)#16 <= 136.8
 WarehouseInventoryCapacity(2,1,8): I_warehouse(2)(1)(8)#46 <= 136.8
 WarehouseInventoryCapacity(1,2,8): I_warehouse(1)(2)(8)#26 <= 136.8
 WarehouseInventoryCapacity(2,2,8): I_warehouse(2)(2)(8)#56 <= 136.8
 WarehouseInventoryCapacity(1,3,8): I_warehouse(1)(3)(8)#36 <= 136.8
 WarehouseInventoryCapacity(2,3,8): I_warehouse(2)(3)(8)#66 <= 136.8
 WarehouseInventoryCapacity(1,1,9): I_warehouse(1)(1)(9)#17 <= 136.8
 WarehouseInventoryCapacity(2,1,9): I_warehouse(2)(1)(9)#47 <= 136.8
 WarehouseInventoryCapacity(1,2,9): I_warehouse(1)(2)(9)#27 <= 136.8
 WarehouseInventoryCapacity(2,2,9): I_warehouse(2)(2)(9)#57 <= 136.8
 WarehouseInventoryCapacity(1,3,9): I_warehouse(1)(3)(9)#37 <= 136.8
 WarehouseInventoryCapacity(2,3,9): I_warehouse(2)(3)(9)#67 <= 136.8
 WarehouseInventoryCapacity(1,1,10): I_warehouse(1)(1)(10)#18 <= 136.8
 WarehouseInventoryCapacity(2,1,10): I_warehouse(2)(1)(10)#48 <= 136.8
 WarehouseInventoryCapacity(1,2,10): I_warehouse(1)(2)(10)#28 <= 136.8
 WarehouseInventoryCapacity(2,2,10): I_warehouse(2)(2)(10)#58 <= 136.8
 WarehouseInventoryCapacity(1,3,10): I_warehouse(1)(3)(10)#38 <= 136.8
 WarehouseInventoryCapacity(2,3,10): I_warehouse(2)(3)(10)#68 <= 136.8
 WarehouseInventoryBalance(1,1,1): I_warehouse(1)(1)(1)#9 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(1)#696 + w(3)(2)(1)(1)#726
                                 + w(3)(3)(1)(1)#756 + w(4)(1)(1)(1)#876
                                 + w(4)(2)(1)(1)#906 + w(4)(3)(1)(1)#936
                                 + w(5)(1)(1)(1)#1056 + w(5)(2)(1)(1)#1086
                                 + w(5)(3)(1)(1)#1116 + w(6)(1)(1)(1)#1236
                                 + w(6)(2)(1)(1)#1266 + w(6)(3)(1)(1)#1296
                                 + w(7)(1)(1)(1)#1416 + w(7)(2)(1)(1)#1446
                                 + w(7)(3)(1)(1)#1476 + w(8)(1)(1)(1)#1596
                                 + w(8)(2)(1)(1)#1626 + w(8)(3)(1)(1)#1656
                                 + w(9)(1)(1)(1)#1776 + w(9)(2)(1)(1)#1806
                                 + w(9)(3)(1)(1)#1836 + w(10)(1)(1)(1)#1956
                                 + w(10)(2)(1)(1)#1986 + w(10)(3)(1)(1)#2016
                                 + w(11)(1)(1)(1)#2136 + w(11)(2)(1)(1)#2166
                                 + w(11)(3)(1)(1)#2196 + w(12)(1)(1)(1)#2316
                                 + w(12)(2)(1)(1)#2346 + w(12)(3)(1)(1)#2376
                                  = 0
 WarehouseInventoryBalance(2,1,1): I_warehouse(2)(1)(1)#39 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(1)#786 + w(3)(5)(1)(1)#816
                                 + w(3)(6)(1)(1)#846 + w(4)(4)(1)(1)#966
                                 + w(4)(5)(1)(1)#996 + w(4)(6)(1)(1)#1026
                                 + w(5)(4)(1)(1)#1146 + w(5)(5)(1)(1)#1176
                                 + w(5)(6)(1)(1)#1206 + w(6)(4)(1)(1)#1326
                                 + w(6)(5)(1)(1)#1356 + w(6)(6)(1)(1)#1386
                                 + w(7)(4)(1)(1)#1506 + w(7)(5)(1)(1)#1536
                                 + w(7)(6)(1)(1)#1566 + w(8)(4)(1)(1)#1686
                                 + w(8)(5)(1)(1)#1716 + w(8)(6)(1)(1)#1746
                                 + w(9)(4)(1)(1)#1866 + w(9)(5)(1)(1)#1896
                                 + w(9)(6)(1)(1)#1926 + w(10)(4)(1)(1)#2046
                                 + w(10)(5)(1)(1)#2076 + w(10)(6)(1)(1)#2106
                                 + w(11)(4)(1)(1)#2226 + w(11)(5)(1)(1)#2256
                                 + w(11)(6)(1)(1)#2286 + w(12)(4)(1)(1)#2406
                                 + w(12)(5)(1)(1)#2436 + w(12)(6)(1)(1)#2466
                                  = 0
 WarehouseInventoryBalance(1,2,1): - I_warehouse(1)(1)(1)#9
                                 + I_warehouse(1)(2)(1)#19 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(1)#706 + w(3)(2)(2)(1)#736
                                 + w(3)(3)(2)(1)#766 + w(4)(1)(2)(1)#886
                                 + w(4)(2)(2)(1)#916 + w(4)(3)(2)(1)#946
                                 + w(5)(1)(2)(1)#1066 + w(5)(2)(2)(1)#1096
                                 + w(5)(3)(2)(1)#1126 + w(6)(1)(2)(1)#1246
                                 + w(6)(2)(2)(1)#1276 + w(6)(3)(2)(1)#1306
                                 + w(7)(1)(2)(1)#1426 + w(7)(2)(2)(1)#1456
                                 + w(7)(3)(2)(1)#1486 + w(8)(1)(2)(1)#1606
                                 + w(8)(2)(2)(1)#1636 + w(8)(3)(2)(1)#1666
                                 + w(9)(1)(2)(1)#1786 + w(9)(2)(2)(1)#1816
                                 + w(9)(3)(2)(1)#1846 + w(10)(1)(2)(1)#1966
                                 + w(10)(2)(2)(1)#1996 + w(10)(3)(2)(1)#2026
                                 + w(11)(1)(2)(1)#2146 + w(11)(2)(2)(1)#2176
                                 + w(11)(3)(2)(1)#2206 + w(12)(1)(2)(1)#2326
                                 + w(12)(2)(2)(1)#2356 + w(12)(3)(2)(1)#2386
                                  = 0
 WarehouseInventoryBalance(2,2,1): - I_warehouse(2)(1)(1)#39
                                 + I_warehouse(2)(2)(1)#49 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(1)#796 + w(3)(5)(2)(1)#826
                                 + w(3)(6)(2)(1)#856 + w(4)(4)(2)(1)#976
                                 + w(4)(5)(2)(1)#1006 + w(4)(6)(2)(1)#1036
                                 + w(5)(4)(2)(1)#1156 + w(5)(5)(2)(1)#1186
                                 + w(5)(6)(2)(1)#1216 + w(6)(4)(2)(1)#1336
                                 + w(6)(5)(2)(1)#1366 + w(6)(6)(2)(1)#1396
                                 + w(7)(4)(2)(1)#1516 + w(7)(5)(2)(1)#1546
                                 + w(7)(6)(2)(1)#1576 + w(8)(4)(2)(1)#1696
                                 + w(8)(5)(2)(1)#1726 + w(8)(6)(2)(1)#1756
                                 + w(9)(4)(2)(1)#1876 + w(9)(5)(2)(1)#1906
                                 + w(9)(6)(2)(1)#1936 + w(10)(4)(2)(1)#2056
                                 + w(10)(5)(2)(1)#2086 + w(10)(6)(2)(1)#2116
                                 + w(11)(4)(2)(1)#2236 + w(11)(5)(2)(1)#2266
                                 + w(11)(6)(2)(1)#2296 + w(12)(4)(2)(1)#2416
                                 + w(12)(5)(2)(1)#2446 + w(12)(6)(2)(1)#2476
                                  = 0
 WarehouseInventoryBalance(1,3,1): - I_warehouse(1)(2)(1)#19
                                 + I_warehouse(1)(3)(1)#29 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(1)#716 + w(3)(2)(3)(1)#746
                                 + w(3)(3)(3)(1)#776 + w(4)(1)(3)(1)#896
                                 + w(4)(2)(3)(1)#926 + w(4)(3)(3)(1)#956
                                 + w(5)(1)(3)(1)#1076 + w(5)(2)(3)(1)#1106
                                 + w(5)(3)(3)(1)#1136 + w(6)(1)(3)(1)#1256
                                 + w(6)(2)(3)(1)#1286 + w(6)(3)(3)(1)#1316
                                 + w(7)(1)(3)(1)#1436 + w(7)(2)(3)(1)#1466
                                 + w(7)(3)(3)(1)#1496 + w(8)(1)(3)(1)#1616
                                 + w(8)(2)(3)(1)#1646 + w(8)(3)(3)(1)#1676
                                 + w(9)(1)(3)(1)#1796 + w(9)(2)(3)(1)#1826
                                 + w(9)(3)(3)(1)#1856 + w(10)(1)(3)(1)#1976
                                 + w(10)(2)(3)(1)#2006 + w(10)(3)(3)(1)#2036
                                 + w(11)(1)(3)(1)#2156 + w(11)(2)(3)(1)#2186
                                 + w(11)(3)(3)(1)#2216 + w(12)(1)(3)(1)#2336
                                 + w(12)(2)(3)(1)#2366 + w(12)(3)(3)(1)#2396
                                  = 0
 WarehouseInventoryBalance(2,3,1): - I_warehouse(2)(2)(1)#49
                                 + I_warehouse(2)(3)(1)#59 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(1)#806 + w(3)(5)(3)(1)#836
                                 + w(3)(6)(3)(1)#866 + w(4)(4)(3)(1)#986
                                 + w(4)(5)(3)(1)#1016 + w(4)(6)(3)(1)#1046
                                 + w(5)(4)(3)(1)#1166 + w(5)(5)(3)(1)#1196
                                 + w(5)(6)(3)(1)#1226 + w(6)(4)(3)(1)#1346
                                 + w(6)(5)(3)(1)#1376 + w(6)(6)(3)(1)#1406
                                 + w(7)(4)(3)(1)#1526 + w(7)(5)(3)(1)#1556
                                 + w(7)(6)(3)(1)#1586 + w(8)(4)(3)(1)#1706
                                 + w(8)(5)(3)(1)#1736 + w(8)(6)(3)(1)#1766
                                 + w(9)(4)(3)(1)#1886 + w(9)(5)(3)(1)#1916
                                 + w(9)(6)(3)(1)#1946 + w(10)(4)(3)(1)#2066
                                 + w(10)(5)(3)(1)#2096 + w(10)(6)(3)(1)#2126
                                 + w(11)(4)(3)(1)#2246 + w(11)(5)(3)(1)#2276
                                 + w(11)(6)(3)(1)#2306 + w(12)(4)(3)(1)#2426
                                 + w(12)(5)(3)(1)#2456 + w(12)(6)(3)(1)#2486
                                  = 0
 WarehouseInventoryBalance(1,1,2): I_warehouse(1)(1)(2)#10 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(2)#697 + w(3)(2)(1)(2)#727
                                 + w(3)(3)(1)(2)#757 + w(4)(1)(1)(2)#877
                                 + w(4)(2)(1)(2)#907 + w(4)(3)(1)(2)#937
                                 + w(5)(1)(1)(2)#1057 + w(5)(2)(1)(2)#1087
                                 + w(5)(3)(1)(2)#1117 + w(6)(1)(1)(2)#1237
                                 + w(6)(2)(1)(2)#1267 + w(6)(3)(1)(2)#1297
                                 + w(7)(1)(1)(2)#1417 + w(7)(2)(1)(2)#1447
                                 + w(7)(3)(1)(2)#1477 + w(8)(1)(1)(2)#1597
                                 + w(8)(2)(1)(2)#1627 + w(8)(3)(1)(2)#1657
                                 + w(9)(1)(1)(2)#1777 + w(9)(2)(1)(2)#1807
                                 + w(9)(3)(1)(2)#1837 + w(10)(1)(1)(2)#1957
                                 + w(10)(2)(1)(2)#1987 + w(10)(3)(1)(2)#2017
                                 + w(11)(1)(1)(2)#2137 + w(11)(2)(1)(2)#2167
                                 + w(11)(3)(1)(2)#2197 + w(12)(1)(1)(2)#2317
                                 + w(12)(2)(1)(2)#2347 + w(12)(3)(1)(2)#2377
                                  = 0
 WarehouseInventoryBalance(2,1,2): I_warehouse(2)(1)(2)#40 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(2)#787 + w(3)(5)(1)(2)#817
                                 + w(3)(6)(1)(2)#847 + w(4)(4)(1)(2)#967
                                 + w(4)(5)(1)(2)#997 + w(4)(6)(1)(2)#1027
                                 + w(5)(4)(1)(2)#1147 + w(5)(5)(1)(2)#1177
                                 + w(5)(6)(1)(2)#1207 + w(6)(4)(1)(2)#1327
                                 + w(6)(5)(1)(2)#1357 + w(6)(6)(1)(2)#1387
                                 + w(7)(4)(1)(2)#1507 + w(7)(5)(1)(2)#1537
                                 + w(7)(6)(1)(2)#1567 + w(8)(4)(1)(2)#1687
                                 + w(8)(5)(1)(2)#1717 + w(8)(6)(1)(2)#1747
                                 + w(9)(4)(1)(2)#1867 + w(9)(5)(1)(2)#1897
                                 + w(9)(6)(1)(2)#1927 + w(10)(4)(1)(2)#2047
                                 + w(10)(5)(1)(2)#2077 + w(10)(6)(1)(2)#2107
                                 + w(11)(4)(1)(2)#2227 + w(11)(5)(1)(2)#2257
                                 + w(11)(6)(1)(2)#2287 + w(12)(4)(1)(2)#2407
                                 + w(12)(5)(1)(2)#2437 + w(12)(6)(1)(2)#2467
                                  = 0
 WarehouseInventoryBalance(1,2,2): - I_warehouse(1)(1)(2)#10
                                 + I_warehouse(1)(2)(2)#20 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(2)#707 + w(3)(2)(2)(2)#737
                                 + w(3)(3)(2)(2)#767 + w(4)(1)(2)(2)#887
                                 + w(4)(2)(2)(2)#917 + w(4)(3)(2)(2)#947
                                 + w(5)(1)(2)(2)#1067 + w(5)(2)(2)(2)#1097
                                 + w(5)(3)(2)(2)#1127 + w(6)(1)(2)(2)#1247
                                 + w(6)(2)(2)(2)#1277 + w(6)(3)(2)(2)#1307
                                 + w(7)(1)(2)(2)#1427 + w(7)(2)(2)(2)#1457
                                 + w(7)(3)(2)(2)#1487 + w(8)(1)(2)(2)#1607
                                 + w(8)(2)(2)(2)#1637 + w(8)(3)(2)(2)#1667
                                 + w(9)(1)(2)(2)#1787 + w(9)(2)(2)(2)#1817
                                 + w(9)(3)(2)(2)#1847 + w(10)(1)(2)(2)#1967
                                 + w(10)(2)(2)(2)#1997 + w(10)(3)(2)(2)#2027
                                 + w(11)(1)(2)(2)#2147 + w(11)(2)(2)(2)#2177
                                 + w(11)(3)(2)(2)#2207 + w(12)(1)(2)(2)#2327
                                 + w(12)(2)(2)(2)#2357 + w(12)(3)(2)(2)#2387
                                  = 0
 WarehouseInventoryBalance(2,2,2): - I_warehouse(2)(1)(2)#40
                                 + I_warehouse(2)(2)(2)#50 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(2)#797 + w(3)(5)(2)(2)#827
                                 + w(3)(6)(2)(2)#857 + w(4)(4)(2)(2)#977
                                 + w(4)(5)(2)(2)#1007 + w(4)(6)(2)(2)#1037
                                 + w(5)(4)(2)(2)#1157 + w(5)(5)(2)(2)#1187
                                 + w(5)(6)(2)(2)#1217 + w(6)(4)(2)(2)#1337
                                 + w(6)(5)(2)(2)#1367 + w(6)(6)(2)(2)#1397
                                 + w(7)(4)(2)(2)#1517 + w(7)(5)(2)(2)#1547
                                 + w(7)(6)(2)(2)#1577 + w(8)(4)(2)(2)#1697
                                 + w(8)(5)(2)(2)#1727 + w(8)(6)(2)(2)#1757
                                 + w(9)(4)(2)(2)#1877 + w(9)(5)(2)(2)#1907
                                 + w(9)(6)(2)(2)#1937 + w(10)(4)(2)(2)#2057
                                 + w(10)(5)(2)(2)#2087 + w(10)(6)(2)(2)#2117
                                 + w(11)(4)(2)(2)#2237 + w(11)(5)(2)(2)#2267
                                 + w(11)(6)(2)(2)#2297 + w(12)(4)(2)(2)#2417
                                 + w(12)(5)(2)(2)#2447 + w(12)(6)(2)(2)#2477
                                  = 0
 WarehouseInventoryBalance(1,3,2): - I_warehouse(1)(2)(2)#20
                                 + I_warehouse(1)(3)(2)#30 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(2)#717 + w(3)(2)(3)(2)#747
                                 + w(3)(3)(3)(2)#777 + w(4)(1)(3)(2)#897
                                 + w(4)(2)(3)(2)#927 + w(4)(3)(3)(2)#957
                                 + w(5)(1)(3)(2)#1077 + w(5)(2)(3)(2)#1107
                                 + w(5)(3)(3)(2)#1137 + w(6)(1)(3)(2)#1257
                                 + w(6)(2)(3)(2)#1287 + w(6)(3)(3)(2)#1317
                                 + w(7)(1)(3)(2)#1437 + w(7)(2)(3)(2)#1467
                                 + w(7)(3)(3)(2)#1497 + w(8)(1)(3)(2)#1617
                                 + w(8)(2)(3)(2)#1647 + w(8)(3)(3)(2)#1677
                                 + w(9)(1)(3)(2)#1797 + w(9)(2)(3)(2)#1827
                                 + w(9)(3)(3)(2)#1857 + w(10)(1)(3)(2)#1977
                                 + w(10)(2)(3)(2)#2007 + w(10)(3)(3)(2)#2037
                                 + w(11)(1)(3)(2)#2157 + w(11)(2)(3)(2)#2187
                                 + w(11)(3)(3)(2)#2217 + w(12)(1)(3)(2)#2337
                                 + w(12)(2)(3)(2)#2367 + w(12)(3)(3)(2)#2397
                                  = 0
 WarehouseInventoryBalance(2,3,2): - I_warehouse(2)(2)(2)#50
                                 + I_warehouse(2)(3)(2)#60 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(2)#807 + w(3)(5)(3)(2)#837
                                 + w(3)(6)(3)(2)#867 + w(4)(4)(3)(2)#987
                                 + w(4)(5)(3)(2)#1017 + w(4)(6)(3)(2)#1047
                                 + w(5)(4)(3)(2)#1167 + w(5)(5)(3)(2)#1197
                                 + w(5)(6)(3)(2)#1227 + w(6)(4)(3)(2)#1347
                                 + w(6)(5)(3)(2)#1377 + w(6)(6)(3)(2)#1407
                                 + w(7)(4)(3)(2)#1527 + w(7)(5)(3)(2)#1557
                                 + w(7)(6)(3)(2)#1587 + w(8)(4)(3)(2)#1707
                                 + w(8)(5)(3)(2)#1737 + w(8)(6)(3)(2)#1767
                                 + w(9)(4)(3)(2)#1887 + w(9)(5)(3)(2)#1917
                                 + w(9)(6)(3)(2)#1947 + w(10)(4)(3)(2)#2067
                                 + w(10)(5)(3)(2)#2097 + w(10)(6)(3)(2)#2127
                                 + w(11)(4)(3)(2)#2247 + w(11)(5)(3)(2)#2277
                                 + w(11)(6)(3)(2)#2307 + w(12)(4)(3)(2)#2427
                                 + w(12)(5)(3)(2)#2457 + w(12)(6)(3)(2)#2487
                                  = 0
 WarehouseInventoryBalance(1,1,3): I_warehouse(1)(1)(3)#11 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(3)#698 + w(3)(2)(1)(3)#728
                                 + w(3)(3)(1)(3)#758 + w(4)(1)(1)(3)#878
                                 + w(4)(2)(1)(3)#908 + w(4)(3)(1)(3)#938
                                 + w(5)(1)(1)(3)#1058 + w(5)(2)(1)(3)#1088
                                 + w(5)(3)(1)(3)#1118 + w(6)(1)(1)(3)#1238
                                 + w(6)(2)(1)(3)#1268 + w(6)(3)(1)(3)#1298
                                 + w(7)(1)(1)(3)#1418 + w(7)(2)(1)(3)#1448
                                 + w(7)(3)(1)(3)#1478 + w(8)(1)(1)(3)#1598
                                 + w(8)(2)(1)(3)#1628 + w(8)(3)(1)(3)#1658
                                 + w(9)(1)(1)(3)#1778 + w(9)(2)(1)(3)#1808
                                 + w(9)(3)(1)(3)#1838 + w(10)(1)(1)(3)#1958
                                 + w(10)(2)(1)(3)#1988 + w(10)(3)(1)(3)#2018
                                 + w(11)(1)(1)(3)#2138 + w(11)(2)(1)(3)#2168
                                 + w(11)(3)(1)(3)#2198 + w(12)(1)(1)(3)#2318
                                 + w(12)(2)(1)(3)#2348 + w(12)(3)(1)(3)#2378
                                  = 0
 WarehouseInventoryBalance(2,1,3): I_warehouse(2)(1)(3)#41 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(3)#788 + w(3)(5)(1)(3)#818
                                 + w(3)(6)(1)(3)#848 + w(4)(4)(1)(3)#968
                                 + w(4)(5)(1)(3)#998 + w(4)(6)(1)(3)#1028
                                 + w(5)(4)(1)(3)#1148 + w(5)(5)(1)(3)#1178
                                 + w(5)(6)(1)(3)#1208 + w(6)(4)(1)(3)#1328
                                 + w(6)(5)(1)(3)#1358 + w(6)(6)(1)(3)#1388
                                 + w(7)(4)(1)(3)#1508 + w(7)(5)(1)(3)#1538
                                 + w(7)(6)(1)(3)#1568 + w(8)(4)(1)(3)#1688
                                 + w(8)(5)(1)(3)#1718 + w(8)(6)(1)(3)#1748
                                 + w(9)(4)(1)(3)#1868 + w(9)(5)(1)(3)#1898
                                 + w(9)(6)(1)(3)#1928 + w(10)(4)(1)(3)#2048
                                 + w(10)(5)(1)(3)#2078 + w(10)(6)(1)(3)#2108
                                 + w(11)(4)(1)(3)#2228 + w(11)(5)(1)(3)#2258
                                 + w(11)(6)(1)(3)#2288 + w(12)(4)(1)(3)#2408
                                 + w(12)(5)(1)(3)#2438 + w(12)(6)(1)(3)#2468
                                  = 0
 WarehouseInventoryBalance(1,2,3): - I_warehouse(1)(1)(3)#11
                                 + I_warehouse(1)(2)(3)#21 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(3)#708 + w(3)(2)(2)(3)#738
                                 + w(3)(3)(2)(3)#768 + w(4)(1)(2)(3)#888
                                 + w(4)(2)(2)(3)#918 + w(4)(3)(2)(3)#948
                                 + w(5)(1)(2)(3)#1068 + w(5)(2)(2)(3)#1098
                                 + w(5)(3)(2)(3)#1128 + w(6)(1)(2)(3)#1248
                                 + w(6)(2)(2)(3)#1278 + w(6)(3)(2)(3)#1308
                                 + w(7)(1)(2)(3)#1428 + w(7)(2)(2)(3)#1458
                                 + w(7)(3)(2)(3)#1488 + w(8)(1)(2)(3)#1608
                                 + w(8)(2)(2)(3)#1638 + w(8)(3)(2)(3)#1668
                                 + w(9)(1)(2)(3)#1788 + w(9)(2)(2)(3)#1818
                                 + w(9)(3)(2)(3)#1848 + w(10)(1)(2)(3)#1968
                                 + w(10)(2)(2)(3)#1998 + w(10)(3)(2)(3)#2028
                                 + w(11)(1)(2)(3)#2148 + w(11)(2)(2)(3)#2178
                                 + w(11)(3)(2)(3)#2208 + w(12)(1)(2)(3)#2328
                                 + w(12)(2)(2)(3)#2358 + w(12)(3)(2)(3)#2388
                                  = 0
 WarehouseInventoryBalance(2,2,3): - I_warehouse(2)(1)(3)#41
                                 + I_warehouse(2)(2)(3)#51 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(3)#798 + w(3)(5)(2)(3)#828
                                 + w(3)(6)(2)(3)#858 + w(4)(4)(2)(3)#978
                                 + w(4)(5)(2)(3)#1008 + w(4)(6)(2)(3)#1038
                                 + w(5)(4)(2)(3)#1158 + w(5)(5)(2)(3)#1188
                                 + w(5)(6)(2)(3)#1218 + w(6)(4)(2)(3)#1338
                                 + w(6)(5)(2)(3)#1368 + w(6)(6)(2)(3)#1398
                                 + w(7)(4)(2)(3)#1518 + w(7)(5)(2)(3)#1548
                                 + w(7)(6)(2)(3)#1578 + w(8)(4)(2)(3)#1698
                                 + w(8)(5)(2)(3)#1728 + w(8)(6)(2)(3)#1758
                                 + w(9)(4)(2)(3)#1878 + w(9)(5)(2)(3)#1908
                                 + w(9)(6)(2)(3)#1938 + w(10)(4)(2)(3)#2058
                                 + w(10)(5)(2)(3)#2088 + w(10)(6)(2)(3)#2118
                                 + w(11)(4)(2)(3)#2238 + w(11)(5)(2)(3)#2268
                                 + w(11)(6)(2)(3)#2298 + w(12)(4)(2)(3)#2418
                                 + w(12)(5)(2)(3)#2448 + w(12)(6)(2)(3)#2478
                                  = 0
 WarehouseInventoryBalance(1,3,3): - I_warehouse(1)(2)(3)#21
                                 + I_warehouse(1)(3)(3)#31 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(3)#718 + w(3)(2)(3)(3)#748
                                 + w(3)(3)(3)(3)#778 + w(4)(1)(3)(3)#898
                                 + w(4)(2)(3)(3)#928 + w(4)(3)(3)(3)#958
                                 + w(5)(1)(3)(3)#1078 + w(5)(2)(3)(3)#1108
                                 + w(5)(3)(3)(3)#1138 + w(6)(1)(3)(3)#1258
                                 + w(6)(2)(3)(3)#1288 + w(6)(3)(3)(3)#1318
                                 + w(7)(1)(3)(3)#1438 + w(7)(2)(3)(3)#1468
                                 + w(7)(3)(3)(3)#1498 + w(8)(1)(3)(3)#1618
                                 + w(8)(2)(3)(3)#1648 + w(8)(3)(3)(3)#1678
                                 + w(9)(1)(3)(3)#1798 + w(9)(2)(3)(3)#1828
                                 + w(9)(3)(3)(3)#1858 + w(10)(1)(3)(3)#1978
                                 + w(10)(2)(3)(3)#2008 + w(10)(3)(3)(3)#2038
                                 + w(11)(1)(3)(3)#2158 + w(11)(2)(3)(3)#2188
                                 + w(11)(3)(3)(3)#2218 + w(12)(1)(3)(3)#2338
                                 + w(12)(2)(3)(3)#2368 + w(12)(3)(3)(3)#2398
                                  = 0
 WarehouseInventoryBalance(2,3,3): - I_warehouse(2)(2)(3)#51
                                 + I_warehouse(2)(3)(3)#61 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(3)#808 + w(3)(5)(3)(3)#838
                                 + w(3)(6)(3)(3)#868 + w(4)(4)(3)(3)#988
                                 + w(4)(5)(3)(3)#1018 + w(4)(6)(3)(3)#1048
                                 + w(5)(4)(3)(3)#1168 + w(5)(5)(3)(3)#1198
                                 + w(5)(6)(3)(3)#1228 + w(6)(4)(3)(3)#1348
                                 + w(6)(5)(3)(3)#1378 + w(6)(6)(3)(3)#1408
                                 + w(7)(4)(3)(3)#1528 + w(7)(5)(3)(3)#1558
                                 + w(7)(6)(3)(3)#1588 + w(8)(4)(3)(3)#1708
                                 + w(8)(5)(3)(3)#1738 + w(8)(6)(3)(3)#1768
                                 + w(9)(4)(3)(3)#1888 + w(9)(5)(3)(3)#1918
                                 + w(9)(6)(3)(3)#1948 + w(10)(4)(3)(3)#2068
                                 + w(10)(5)(3)(3)#2098 + w(10)(6)(3)(3)#2128
                                 + w(11)(4)(3)(3)#2248 + w(11)(5)(3)(3)#2278
                                 + w(11)(6)(3)(3)#2308 + w(12)(4)(3)(3)#2428
                                 + w(12)(5)(3)(3)#2458 + w(12)(6)(3)(3)#2488
                                  = 0
 WarehouseInventoryBalance(1,1,4): I_warehouse(1)(1)(4)#12 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(4)#699 + w(3)(2)(1)(4)#729
                                 + w(3)(3)(1)(4)#759 + w(4)(1)(1)(4)#879
                                 + w(4)(2)(1)(4)#909 + w(4)(3)(1)(4)#939
                                 + w(5)(1)(1)(4)#1059 + w(5)(2)(1)(4)#1089
                                 + w(5)(3)(1)(4)#1119 + w(6)(1)(1)(4)#1239
                                 + w(6)(2)(1)(4)#1269 + w(6)(3)(1)(4)#1299
                                 + w(7)(1)(1)(4)#1419 + w(7)(2)(1)(4)#1449
                                 + w(7)(3)(1)(4)#1479 + w(8)(1)(1)(4)#1599
                                 + w(8)(2)(1)(4)#1629 + w(8)(3)(1)(4)#1659
                                 + w(9)(1)(1)(4)#1779 + w(9)(2)(1)(4)#1809
                                 + w(9)(3)(1)(4)#1839 + w(10)(1)(1)(4)#1959
                                 + w(10)(2)(1)(4)#1989 + w(10)(3)(1)(4)#2019
                                 + w(11)(1)(1)(4)#2139 + w(11)(2)(1)(4)#2169
                                 + w(11)(3)(1)(4)#2199 + w(12)(1)(1)(4)#2319
                                 + w(12)(2)(1)(4)#2349 + w(12)(3)(1)(4)#2379
                                  = 0
 WarehouseInventoryBalance(2,1,4): I_warehouse(2)(1)(4)#42 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(4)#789 + w(3)(5)(1)(4)#819
                                 + w(3)(6)(1)(4)#849 + w(4)(4)(1)(4)#969
                                 + w(4)(5)(1)(4)#999 + w(4)(6)(1)(4)#1029
                                 + w(5)(4)(1)(4)#1149 + w(5)(5)(1)(4)#1179
                                 + w(5)(6)(1)(4)#1209 + w(6)(4)(1)(4)#1329
                                 + w(6)(5)(1)(4)#1359 + w(6)(6)(1)(4)#1389
                                 + w(7)(4)(1)(4)#1509 + w(7)(5)(1)(4)#1539
                                 + w(7)(6)(1)(4)#1569 + w(8)(4)(1)(4)#1689
                                 + w(8)(5)(1)(4)#1719 + w(8)(6)(1)(4)#1749
                                 + w(9)(4)(1)(4)#1869 + w(9)(5)(1)(4)#1899
                                 + w(9)(6)(1)(4)#1929 + w(10)(4)(1)(4)#2049
                                 + w(10)(5)(1)(4)#2079 + w(10)(6)(1)(4)#2109
                                 + w(11)(4)(1)(4)#2229 + w(11)(5)(1)(4)#2259
                                 + w(11)(6)(1)(4)#2289 + w(12)(4)(1)(4)#2409
                                 + w(12)(5)(1)(4)#2439 + w(12)(6)(1)(4)#2469
                                  = 0
 WarehouseInventoryBalance(1,2,4): - I_warehouse(1)(1)(4)#12
                                 + I_warehouse(1)(2)(4)#22 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(4)#709 + w(3)(2)(2)(4)#739
                                 + w(3)(3)(2)(4)#769 + w(4)(1)(2)(4)#889
                                 + w(4)(2)(2)(4)#919 + w(4)(3)(2)(4)#949
                                 + w(5)(1)(2)(4)#1069 + w(5)(2)(2)(4)#1099
                                 + w(5)(3)(2)(4)#1129 + w(6)(1)(2)(4)#1249
                                 + w(6)(2)(2)(4)#1279 + w(6)(3)(2)(4)#1309
                                 + w(7)(1)(2)(4)#1429 + w(7)(2)(2)(4)#1459
                                 + w(7)(3)(2)(4)#1489 + w(8)(1)(2)(4)#1609
                                 + w(8)(2)(2)(4)#1639 + w(8)(3)(2)(4)#1669
                                 + w(9)(1)(2)(4)#1789 + w(9)(2)(2)(4)#1819
                                 + w(9)(3)(2)(4)#1849 + w(10)(1)(2)(4)#1969
                                 + w(10)(2)(2)(4)#1999 + w(10)(3)(2)(4)#2029
                                 + w(11)(1)(2)(4)#2149 + w(11)(2)(2)(4)#2179
                                 + w(11)(3)(2)(4)#2209 + w(12)(1)(2)(4)#2329
                                 + w(12)(2)(2)(4)#2359 + w(12)(3)(2)(4)#2389
                                  = 0
 WarehouseInventoryBalance(2,2,4): - I_warehouse(2)(1)(4)#42
                                 + I_warehouse(2)(2)(4)#52 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(4)#799 + w(3)(5)(2)(4)#829
                                 + w(3)(6)(2)(4)#859 + w(4)(4)(2)(4)#979
                                 + w(4)(5)(2)(4)#1009 + w(4)(6)(2)(4)#1039
                                 + w(5)(4)(2)(4)#1159 + w(5)(5)(2)(4)#1189
                                 + w(5)(6)(2)(4)#1219 + w(6)(4)(2)(4)#1339
                                 + w(6)(5)(2)(4)#1369 + w(6)(6)(2)(4)#1399
                                 + w(7)(4)(2)(4)#1519 + w(7)(5)(2)(4)#1549
                                 + w(7)(6)(2)(4)#1579 + w(8)(4)(2)(4)#1699
                                 + w(8)(5)(2)(4)#1729 + w(8)(6)(2)(4)#1759
                                 + w(9)(4)(2)(4)#1879 + w(9)(5)(2)(4)#1909
                                 + w(9)(6)(2)(4)#1939 + w(10)(4)(2)(4)#2059
                                 + w(10)(5)(2)(4)#2089 + w(10)(6)(2)(4)#2119
                                 + w(11)(4)(2)(4)#2239 + w(11)(5)(2)(4)#2269
                                 + w(11)(6)(2)(4)#2299 + w(12)(4)(2)(4)#2419
                                 + w(12)(5)(2)(4)#2449 + w(12)(6)(2)(4)#2479
                                  = 0
 WarehouseInventoryBalance(1,3,4): - I_warehouse(1)(2)(4)#22
                                 + I_warehouse(1)(3)(4)#32 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(4)#719 + w(3)(2)(3)(4)#749
                                 + w(3)(3)(3)(4)#779 + w(4)(1)(3)(4)#899
                                 + w(4)(2)(3)(4)#929 + w(4)(3)(3)(4)#959
                                 + w(5)(1)(3)(4)#1079 + w(5)(2)(3)(4)#1109
                                 + w(5)(3)(3)(4)#1139 + w(6)(1)(3)(4)#1259
                                 + w(6)(2)(3)(4)#1289 + w(6)(3)(3)(4)#1319
                                 + w(7)(1)(3)(4)#1439 + w(7)(2)(3)(4)#1469
                                 + w(7)(3)(3)(4)#1499 + w(8)(1)(3)(4)#1619
                                 + w(8)(2)(3)(4)#1649 + w(8)(3)(3)(4)#1679
                                 + w(9)(1)(3)(4)#1799 + w(9)(2)(3)(4)#1829
                                 + w(9)(3)(3)(4)#1859 + w(10)(1)(3)(4)#1979
                                 + w(10)(2)(3)(4)#2009 + w(10)(3)(3)(4)#2039
                                 + w(11)(1)(3)(4)#2159 + w(11)(2)(3)(4)#2189
                                 + w(11)(3)(3)(4)#2219 + w(12)(1)(3)(4)#2339
                                 + w(12)(2)(3)(4)#2369 + w(12)(3)(3)(4)#2399
                                  = 0
 WarehouseInventoryBalance(2,3,4): - I_warehouse(2)(2)(4)#52
                                 + I_warehouse(2)(3)(4)#62 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(4)#809 + w(3)(5)(3)(4)#839
                                 + w(3)(6)(3)(4)#869 + w(4)(4)(3)(4)#989
                                 + w(4)(5)(3)(4)#1019 + w(4)(6)(3)(4)#1049
                                 + w(5)(4)(3)(4)#1169 + w(5)(5)(3)(4)#1199
                                 + w(5)(6)(3)(4)#1229 + w(6)(4)(3)(4)#1349
                                 + w(6)(5)(3)(4)#1379 + w(6)(6)(3)(4)#1409
                                 + w(7)(4)(3)(4)#1529 + w(7)(5)(3)(4)#1559
                                 + w(7)(6)(3)(4)#1589 + w(8)(4)(3)(4)#1709
                                 + w(8)(5)(3)(4)#1739 + w(8)(6)(3)(4)#1769
                                 + w(9)(4)(3)(4)#1889 + w(9)(5)(3)(4)#1919
                                 + w(9)(6)(3)(4)#1949 + w(10)(4)(3)(4)#2069
                                 + w(10)(5)(3)(4)#2099 + w(10)(6)(3)(4)#2129
                                 + w(11)(4)(3)(4)#2249 + w(11)(5)(3)(4)#2279
                                 + w(11)(6)(3)(4)#2309 + w(12)(4)(3)(4)#2429
                                 + w(12)(5)(3)(4)#2459 + w(12)(6)(3)(4)#2489
                                  = 0
 WarehouseInventoryBalance(1,1,5): I_warehouse(1)(1)(5)#13 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(5)#700 + w(3)(2)(1)(5)#730
                                 + w(3)(3)(1)(5)#760 + w(4)(1)(1)(5)#880
                                 + w(4)(2)(1)(5)#910 + w(4)(3)(1)(5)#940
                                 + w(5)(1)(1)(5)#1060 + w(5)(2)(1)(5)#1090
                                 + w(5)(3)(1)(5)#1120 + w(6)(1)(1)(5)#1240
                                 + w(6)(2)(1)(5)#1270 + w(6)(3)(1)(5)#1300
                                 + w(7)(1)(1)(5)#1420 + w(7)(2)(1)(5)#1450
                                 + w(7)(3)(1)(5)#1480 + w(8)(1)(1)(5)#1600
                                 + w(8)(2)(1)(5)#1630 + w(8)(3)(1)(5)#1660
                                 + w(9)(1)(1)(5)#1780 + w(9)(2)(1)(5)#1810
                                 + w(9)(3)(1)(5)#1840 + w(10)(1)(1)(5)#1960
                                 + w(10)(2)(1)(5)#1990 + w(10)(3)(1)(5)#2020
                                 + w(11)(1)(1)(5)#2140 + w(11)(2)(1)(5)#2170
                                 + w(11)(3)(1)(5)#2200 + w(12)(1)(1)(5)#2320
                                 + w(12)(2)(1)(5)#2350 + w(12)(3)(1)(5)#2380
                                  = 0
 WarehouseInventoryBalance(2,1,5): I_warehouse(2)(1)(5)#43 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(5)#790 + w(3)(5)(1)(5)#820
                                 + w(3)(6)(1)(5)#850 + w(4)(4)(1)(5)#970
                                 + w(4)(5)(1)(5)#1000 + w(4)(6)(1)(5)#1030
                                 + w(5)(4)(1)(5)#1150 + w(5)(5)(1)(5)#1180
                                 + w(5)(6)(1)(5)#1210 + w(6)(4)(1)(5)#1330
                                 + w(6)(5)(1)(5)#1360 + w(6)(6)(1)(5)#1390
                                 + w(7)(4)(1)(5)#1510 + w(7)(5)(1)(5)#1540
                                 + w(7)(6)(1)(5)#1570 + w(8)(4)(1)(5)#1690
                                 + w(8)(5)(1)(5)#1720 + w(8)(6)(1)(5)#1750
                                 + w(9)(4)(1)(5)#1870 + w(9)(5)(1)(5)#1900
                                 + w(9)(6)(1)(5)#1930 + w(10)(4)(1)(5)#2050
                                 + w(10)(5)(1)(5)#2080 + w(10)(6)(1)(5)#2110
                                 + w(11)(4)(1)(5)#2230 + w(11)(5)(1)(5)#2260
                                 + w(11)(6)(1)(5)#2290 + w(12)(4)(1)(5)#2410
                                 + w(12)(5)(1)(5)#2440 + w(12)(6)(1)(5)#2470
                                  = 0
 WarehouseInventoryBalance(1,2,5): - I_warehouse(1)(1)(5)#13
                                 + I_warehouse(1)(2)(5)#23 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(5)#710 + w(3)(2)(2)(5)#740
                                 + w(3)(3)(2)(5)#770 + w(4)(1)(2)(5)#890
                                 + w(4)(2)(2)(5)#920 + w(4)(3)(2)(5)#950
                                 + w(5)(1)(2)(5)#1070 + w(5)(2)(2)(5)#1100
                                 + w(5)(3)(2)(5)#1130 + w(6)(1)(2)(5)#1250
                                 + w(6)(2)(2)(5)#1280 + w(6)(3)(2)(5)#1310
                                 + w(7)(1)(2)(5)#1430 + w(7)(2)(2)(5)#1460
                                 + w(7)(3)(2)(5)#1490 + w(8)(1)(2)(5)#1610
                                 + w(8)(2)(2)(5)#1640 + w(8)(3)(2)(5)#1670
                                 + w(9)(1)(2)(5)#1790 + w(9)(2)(2)(5)#1820
                                 + w(9)(3)(2)(5)#1850 + w(10)(1)(2)(5)#1970
                                 + w(10)(2)(2)(5)#2000 + w(10)(3)(2)(5)#2030
                                 + w(11)(1)(2)(5)#2150 + w(11)(2)(2)(5)#2180
                                 + w(11)(3)(2)(5)#2210 + w(12)(1)(2)(5)#2330
                                 + w(12)(2)(2)(5)#2360 + w(12)(3)(2)(5)#2390
                                  = 0
 WarehouseInventoryBalance(2,2,5): - I_warehouse(2)(1)(5)#43
                                 + I_warehouse(2)(2)(5)#53 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(5)#800 + w(3)(5)(2)(5)#830
                                 + w(3)(6)(2)(5)#860 + w(4)(4)(2)(5)#980
                                 + w(4)(5)(2)(5)#1010 + w(4)(6)(2)(5)#1040
                                 + w(5)(4)(2)(5)#1160 + w(5)(5)(2)(5)#1190
                                 + w(5)(6)(2)(5)#1220 + w(6)(4)(2)(5)#1340
                                 + w(6)(5)(2)(5)#1370 + w(6)(6)(2)(5)#1400
                                 + w(7)(4)(2)(5)#1520 + w(7)(5)(2)(5)#1550
                                 + w(7)(6)(2)(5)#1580 + w(8)(4)(2)(5)#1700
                                 + w(8)(5)(2)(5)#1730 + w(8)(6)(2)(5)#1760
                                 + w(9)(4)(2)(5)#1880 + w(9)(5)(2)(5)#1910
                                 + w(9)(6)(2)(5)#1940 + w(10)(4)(2)(5)#2060
                                 + w(10)(5)(2)(5)#2090 + w(10)(6)(2)(5)#2120
                                 + w(11)(4)(2)(5)#2240 + w(11)(5)(2)(5)#2270
                                 + w(11)(6)(2)(5)#2300 + w(12)(4)(2)(5)#2420
                                 + w(12)(5)(2)(5)#2450 + w(12)(6)(2)(5)#2480
                                  = 0
 WarehouseInventoryBalance(1,3,5): - I_warehouse(1)(2)(5)#23
                                 + I_warehouse(1)(3)(5)#33 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(5)#720 + w(3)(2)(3)(5)#750
                                 + w(3)(3)(3)(5)#780 + w(4)(1)(3)(5)#900
                                 + w(4)(2)(3)(5)#930 + w(4)(3)(3)(5)#960
                                 + w(5)(1)(3)(5)#1080 + w(5)(2)(3)(5)#1110
                                 + w(5)(3)(3)(5)#1140 + w(6)(1)(3)(5)#1260
                                 + w(6)(2)(3)(5)#1290 + w(6)(3)(3)(5)#1320
                                 + w(7)(1)(3)(5)#1440 + w(7)(2)(3)(5)#1470
                                 + w(7)(3)(3)(5)#1500 + w(8)(1)(3)(5)#1620
                                 + w(8)(2)(3)(5)#1650 + w(8)(3)(3)(5)#1680
                                 + w(9)(1)(3)(5)#1800 + w(9)(2)(3)(5)#1830
                                 + w(9)(3)(3)(5)#1860 + w(10)(1)(3)(5)#1980
                                 + w(10)(2)(3)(5)#2010 + w(10)(3)(3)(5)#2040
                                 + w(11)(1)(3)(5)#2160 + w(11)(2)(3)(5)#2190
                                 + w(11)(3)(3)(5)#2220 + w(12)(1)(3)(5)#2340
                                 + w(12)(2)(3)(5)#2370 + w(12)(3)(3)(5)#2400
                                  = 0
 WarehouseInventoryBalance(2,3,5): - I_warehouse(2)(2)(5)#53
                                 + I_warehouse(2)(3)(5)#63 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(5)#810 + w(3)(5)(3)(5)#840
                                 + w(3)(6)(3)(5)#870 + w(4)(4)(3)(5)#990
                                 + w(4)(5)(3)(5)#1020 + w(4)(6)(3)(5)#1050
                                 + w(5)(4)(3)(5)#1170 + w(5)(5)(3)(5)#1200
                                 + w(5)(6)(3)(5)#1230 + w(6)(4)(3)(5)#1350
                                 + w(6)(5)(3)(5)#1380 + w(6)(6)(3)(5)#1410
                                 + w(7)(4)(3)(5)#1530 + w(7)(5)(3)(5)#1560
                                 + w(7)(6)(3)(5)#1590 + w(8)(4)(3)(5)#1710
                                 + w(8)(5)(3)(5)#1740 + w(8)(6)(3)(5)#1770
                                 + w(9)(4)(3)(5)#1890 + w(9)(5)(3)(5)#1920
                                 + w(9)(6)(3)(5)#1950 + w(10)(4)(3)(5)#2070
                                 + w(10)(5)(3)(5)#2100 + w(10)(6)(3)(5)#2130
                                 + w(11)(4)(3)(5)#2250 + w(11)(5)(3)(5)#2280
                                 + w(11)(6)(3)(5)#2310 + w(12)(4)(3)(5)#2430
                                 + w(12)(5)(3)(5)#2460 + w(12)(6)(3)(5)#2490
                                  = 0
 WarehouseInventoryBalance(1,1,6): I_warehouse(1)(1)(6)#14 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(6)#701 + w(3)(2)(1)(6)#731
                                 + w(3)(3)(1)(6)#761 + w(4)(1)(1)(6)#881
                                 + w(4)(2)(1)(6)#911 + w(4)(3)(1)(6)#941
                                 + w(5)(1)(1)(6)#1061 + w(5)(2)(1)(6)#1091
                                 + w(5)(3)(1)(6)#1121 + w(6)(1)(1)(6)#1241
                                 + w(6)(2)(1)(6)#1271 + w(6)(3)(1)(6)#1301
                                 + w(7)(1)(1)(6)#1421 + w(7)(2)(1)(6)#1451
                                 + w(7)(3)(1)(6)#1481 + w(8)(1)(1)(6)#1601
                                 + w(8)(2)(1)(6)#1631 + w(8)(3)(1)(6)#1661
                                 + w(9)(1)(1)(6)#1781 + w(9)(2)(1)(6)#1811
                                 + w(9)(3)(1)(6)#1841 + w(10)(1)(1)(6)#1961
                                 + w(10)(2)(1)(6)#1991 + w(10)(3)(1)(6)#2021
                                 + w(11)(1)(1)(6)#2141 + w(11)(2)(1)(6)#2171
                                 + w(11)(3)(1)(6)#2201 + w(12)(1)(1)(6)#2321
                                 + w(12)(2)(1)(6)#2351 + w(12)(3)(1)(6)#2381
                                  = 0
 WarehouseInventoryBalance(2,1,6): I_warehouse(2)(1)(6)#44 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(6)#791 + w(3)(5)(1)(6)#821
                                 + w(3)(6)(1)(6)#851 + w(4)(4)(1)(6)#971
                                 + w(4)(5)(1)(6)#1001 + w(4)(6)(1)(6)#1031
                                 + w(5)(4)(1)(6)#1151 + w(5)(5)(1)(6)#1181
                                 + w(5)(6)(1)(6)#1211 + w(6)(4)(1)(6)#1331
                                 + w(6)(5)(1)(6)#1361 + w(6)(6)(1)(6)#1391
                                 + w(7)(4)(1)(6)#1511 + w(7)(5)(1)(6)#1541
                                 + w(7)(6)(1)(6)#1571 + w(8)(4)(1)(6)#1691
                                 + w(8)(5)(1)(6)#1721 + w(8)(6)(1)(6)#1751
                                 + w(9)(4)(1)(6)#1871 + w(9)(5)(1)(6)#1901
                                 + w(9)(6)(1)(6)#1931 + w(10)(4)(1)(6)#2051
                                 + w(10)(5)(1)(6)#2081 + w(10)(6)(1)(6)#2111
                                 + w(11)(4)(1)(6)#2231 + w(11)(5)(1)(6)#2261
                                 + w(11)(6)(1)(6)#2291 + w(12)(4)(1)(6)#2411
                                 + w(12)(5)(1)(6)#2441 + w(12)(6)(1)(6)#2471
                                  = 0
 WarehouseInventoryBalance(1,2,6): - I_warehouse(1)(1)(6)#14
                                 + I_warehouse(1)(2)(6)#24 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(6)#711 + w(3)(2)(2)(6)#741
                                 + w(3)(3)(2)(6)#771 + w(4)(1)(2)(6)#891
                                 + w(4)(2)(2)(6)#921 + w(4)(3)(2)(6)#951
                                 + w(5)(1)(2)(6)#1071 + w(5)(2)(2)(6)#1101
                                 + w(5)(3)(2)(6)#1131 + w(6)(1)(2)(6)#1251
                                 + w(6)(2)(2)(6)#1281 + w(6)(3)(2)(6)#1311
                                 + w(7)(1)(2)(6)#1431 + w(7)(2)(2)(6)#1461
                                 + w(7)(3)(2)(6)#1491 + w(8)(1)(2)(6)#1611
                                 + w(8)(2)(2)(6)#1641 + w(8)(3)(2)(6)#1671
                                 + w(9)(1)(2)(6)#1791 + w(9)(2)(2)(6)#1821
                                 + w(9)(3)(2)(6)#1851 + w(10)(1)(2)(6)#1971
                                 + w(10)(2)(2)(6)#2001 + w(10)(3)(2)(6)#2031
                                 + w(11)(1)(2)(6)#2151 + w(11)(2)(2)(6)#2181
                                 + w(11)(3)(2)(6)#2211 + w(12)(1)(2)(6)#2331
                                 + w(12)(2)(2)(6)#2361 + w(12)(3)(2)(6)#2391
                                  = 0
 WarehouseInventoryBalance(2,2,6): - I_warehouse(2)(1)(6)#44
                                 + I_warehouse(2)(2)(6)#54 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(6)#801 + w(3)(5)(2)(6)#831
                                 + w(3)(6)(2)(6)#861 + w(4)(4)(2)(6)#981
                                 + w(4)(5)(2)(6)#1011 + w(4)(6)(2)(6)#1041
                                 + w(5)(4)(2)(6)#1161 + w(5)(5)(2)(6)#1191
                                 + w(5)(6)(2)(6)#1221 + w(6)(4)(2)(6)#1341
                                 + w(6)(5)(2)(6)#1371 + w(6)(6)(2)(6)#1401
                                 + w(7)(4)(2)(6)#1521 + w(7)(5)(2)(6)#1551
                                 + w(7)(6)(2)(6)#1581 + w(8)(4)(2)(6)#1701
                                 + w(8)(5)(2)(6)#1731 + w(8)(6)(2)(6)#1761
                                 + w(9)(4)(2)(6)#1881 + w(9)(5)(2)(6)#1911
                                 + w(9)(6)(2)(6)#1941 + w(10)(4)(2)(6)#2061
                                 + w(10)(5)(2)(6)#2091 + w(10)(6)(2)(6)#2121
                                 + w(11)(4)(2)(6)#2241 + w(11)(5)(2)(6)#2271
                                 + w(11)(6)(2)(6)#2301 + w(12)(4)(2)(6)#2421
                                 + w(12)(5)(2)(6)#2451 + w(12)(6)(2)(6)#2481
                                  = 0
 WarehouseInventoryBalance(1,3,6): - I_warehouse(1)(2)(6)#24
                                 + I_warehouse(1)(3)(6)#34 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(6)#721 + w(3)(2)(3)(6)#751
                                 + w(3)(3)(3)(6)#781 + w(4)(1)(3)(6)#901
                                 + w(4)(2)(3)(6)#931 + w(4)(3)(3)(6)#961
                                 + w(5)(1)(3)(6)#1081 + w(5)(2)(3)(6)#1111
                                 + w(5)(3)(3)(6)#1141 + w(6)(1)(3)(6)#1261
                                 + w(6)(2)(3)(6)#1291 + w(6)(3)(3)(6)#1321
                                 + w(7)(1)(3)(6)#1441 + w(7)(2)(3)(6)#1471
                                 + w(7)(3)(3)(6)#1501 + w(8)(1)(3)(6)#1621
                                 + w(8)(2)(3)(6)#1651 + w(8)(3)(3)(6)#1681
                                 + w(9)(1)(3)(6)#1801 + w(9)(2)(3)(6)#1831
                                 + w(9)(3)(3)(6)#1861 + w(10)(1)(3)(6)#1981
                                 + w(10)(2)(3)(6)#2011 + w(10)(3)(3)(6)#2041
                                 + w(11)(1)(3)(6)#2161 + w(11)(2)(3)(6)#2191
                                 + w(11)(3)(3)(6)#2221 + w(12)(1)(3)(6)#2341
                                 + w(12)(2)(3)(6)#2371 + w(12)(3)(3)(6)#2401
                                  = 0
 WarehouseInventoryBalance(2,3,6): - I_warehouse(2)(2)(6)#54
                                 + I_warehouse(2)(3)(6)#64 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(6)#811 + w(3)(5)(3)(6)#841
                                 + w(3)(6)(3)(6)#871 + w(4)(4)(3)(6)#991
                                 + w(4)(5)(3)(6)#1021 + w(4)(6)(3)(6)#1051
                                 + w(5)(4)(3)(6)#1171 + w(5)(5)(3)(6)#1201
                                 + w(5)(6)(3)(6)#1231 + w(6)(4)(3)(6)#1351
                                 + w(6)(5)(3)(6)#1381 + w(6)(6)(3)(6)#1411
                                 + w(7)(4)(3)(6)#1531 + w(7)(5)(3)(6)#1561
                                 + w(7)(6)(3)(6)#1591 + w(8)(4)(3)(6)#1711
                                 + w(8)(5)(3)(6)#1741 + w(8)(6)(3)(6)#1771
                                 + w(9)(4)(3)(6)#1891 + w(9)(5)(3)(6)#1921
                                 + w(9)(6)(3)(6)#1951 + w(10)(4)(3)(6)#2071
                                 + w(10)(5)(3)(6)#2101 + w(10)(6)(3)(6)#2131
                                 + w(11)(4)(3)(6)#2251 + w(11)(5)(3)(6)#2281
                                 + w(11)(6)(3)(6)#2311 + w(12)(4)(3)(6)#2431
                                 + w(12)(5)(3)(6)#2461 + w(12)(6)(3)(6)#2491
                                  = 0
 WarehouseInventoryBalance(1,1,7): I_warehouse(1)(1)(7)#15 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(7)#702 + w(3)(2)(1)(7)#732
                                 + w(3)(3)(1)(7)#762 + w(4)(1)(1)(7)#882
                                 + w(4)(2)(1)(7)#912 + w(4)(3)(1)(7)#942
                                 + w(5)(1)(1)(7)#1062 + w(5)(2)(1)(7)#1092
                                 + w(5)(3)(1)(7)#1122 + w(6)(1)(1)(7)#1242
                                 + w(6)(2)(1)(7)#1272 + w(6)(3)(1)(7)#1302
                                 + w(7)(1)(1)(7)#1422 + w(7)(2)(1)(7)#1452
                                 + w(7)(3)(1)(7)#1482 + w(8)(1)(1)(7)#1602
                                 + w(8)(2)(1)(7)#1632 + w(8)(3)(1)(7)#1662
                                 + w(9)(1)(1)(7)#1782 + w(9)(2)(1)(7)#1812
                                 + w(9)(3)(1)(7)#1842 + w(10)(1)(1)(7)#1962
                                 + w(10)(2)(1)(7)#1992 + w(10)(3)(1)(7)#2022
                                 + w(11)(1)(1)(7)#2142 + w(11)(2)(1)(7)#2172
                                 + w(11)(3)(1)(7)#2202 + w(12)(1)(1)(7)#2322
                                 + w(12)(2)(1)(7)#2352 + w(12)(3)(1)(7)#2382
                                  = 0
 WarehouseInventoryBalance(2,1,7): I_warehouse(2)(1)(7)#45 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(7)#792 + w(3)(5)(1)(7)#822
                                 + w(3)(6)(1)(7)#852 + w(4)(4)(1)(7)#972
                                 + w(4)(5)(1)(7)#1002 + w(4)(6)(1)(7)#1032
                                 + w(5)(4)(1)(7)#1152 + w(5)(5)(1)(7)#1182
                                 + w(5)(6)(1)(7)#1212 + w(6)(4)(1)(7)#1332
                                 + w(6)(5)(1)(7)#1362 + w(6)(6)(1)(7)#1392
                                 + w(7)(4)(1)(7)#1512 + w(7)(5)(1)(7)#1542
                                 + w(7)(6)(1)(7)#1572 + w(8)(4)(1)(7)#1692
                                 + w(8)(5)(1)(7)#1722 + w(8)(6)(1)(7)#1752
                                 + w(9)(4)(1)(7)#1872 + w(9)(5)(1)(7)#1902
                                 + w(9)(6)(1)(7)#1932 + w(10)(4)(1)(7)#2052
                                 + w(10)(5)(1)(7)#2082 + w(10)(6)(1)(7)#2112
                                 + w(11)(4)(1)(7)#2232 + w(11)(5)(1)(7)#2262
                                 + w(11)(6)(1)(7)#2292 + w(12)(4)(1)(7)#2412
                                 + w(12)(5)(1)(7)#2442 + w(12)(6)(1)(7)#2472
                                  = 0
 WarehouseInventoryBalance(1,2,7): - I_warehouse(1)(1)(7)#15
                                 + I_warehouse(1)(2)(7)#25 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(7)#712 + w(3)(2)(2)(7)#742
                                 + w(3)(3)(2)(7)#772 + w(4)(1)(2)(7)#892
                                 + w(4)(2)(2)(7)#922 + w(4)(3)(2)(7)#952
                                 + w(5)(1)(2)(7)#1072 + w(5)(2)(2)(7)#1102
                                 + w(5)(3)(2)(7)#1132 + w(6)(1)(2)(7)#1252
                                 + w(6)(2)(2)(7)#1282 + w(6)(3)(2)(7)#1312
                                 + w(7)(1)(2)(7)#1432 + w(7)(2)(2)(7)#1462
                                 + w(7)(3)(2)(7)#1492 + w(8)(1)(2)(7)#1612
                                 + w(8)(2)(2)(7)#1642 + w(8)(3)(2)(7)#1672
                                 + w(9)(1)(2)(7)#1792 + w(9)(2)(2)(7)#1822
                                 + w(9)(3)(2)(7)#1852 + w(10)(1)(2)(7)#1972
                                 + w(10)(2)(2)(7)#2002 + w(10)(3)(2)(7)#2032
                                 + w(11)(1)(2)(7)#2152 + w(11)(2)(2)(7)#2182
                                 + w(11)(3)(2)(7)#2212 + w(12)(1)(2)(7)#2332
                                 + w(12)(2)(2)(7)#2362 + w(12)(3)(2)(7)#2392
                                  = 0
 WarehouseInventoryBalance(2,2,7): - I_warehouse(2)(1)(7)#45
                                 + I_warehouse(2)(2)(7)#55 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(7)#802 + w(3)(5)(2)(7)#832
                                 + w(3)(6)(2)(7)#862 + w(4)(4)(2)(7)#982
                                 + w(4)(5)(2)(7)#1012 + w(4)(6)(2)(7)#1042
                                 + w(5)(4)(2)(7)#1162 + w(5)(5)(2)(7)#1192
                                 + w(5)(6)(2)(7)#1222 + w(6)(4)(2)(7)#1342
                                 + w(6)(5)(2)(7)#1372 + w(6)(6)(2)(7)#1402
                                 + w(7)(4)(2)(7)#1522 + w(7)(5)(2)(7)#1552
                                 + w(7)(6)(2)(7)#1582 + w(8)(4)(2)(7)#1702
                                 + w(8)(5)(2)(7)#1732 + w(8)(6)(2)(7)#1762
                                 + w(9)(4)(2)(7)#1882 + w(9)(5)(2)(7)#1912
                                 + w(9)(6)(2)(7)#1942 + w(10)(4)(2)(7)#2062
                                 + w(10)(5)(2)(7)#2092 + w(10)(6)(2)(7)#2122
                                 + w(11)(4)(2)(7)#2242 + w(11)(5)(2)(7)#2272
                                 + w(11)(6)(2)(7)#2302 + w(12)(4)(2)(7)#2422
                                 + w(12)(5)(2)(7)#2452 + w(12)(6)(2)(7)#2482
                                  = 0
 WarehouseInventoryBalance(1,3,7): - I_warehouse(1)(2)(7)#25
                                 + I_warehouse(1)(3)(7)#35 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(7)#722 + w(3)(2)(3)(7)#752
                                 + w(3)(3)(3)(7)#782 + w(4)(1)(3)(7)#902
                                 + w(4)(2)(3)(7)#932 + w(4)(3)(3)(7)#962
                                 + w(5)(1)(3)(7)#1082 + w(5)(2)(3)(7)#1112
                                 + w(5)(3)(3)(7)#1142 + w(6)(1)(3)(7)#1262
                                 + w(6)(2)(3)(7)#1292 + w(6)(3)(3)(7)#1322
                                 + w(7)(1)(3)(7)#1442 + w(7)(2)(3)(7)#1472
                                 + w(7)(3)(3)(7)#1502 + w(8)(1)(3)(7)#1622
                                 + w(8)(2)(3)(7)#1652 + w(8)(3)(3)(7)#1682
                                 + w(9)(1)(3)(7)#1802 + w(9)(2)(3)(7)#1832
                                 + w(9)(3)(3)(7)#1862 + w(10)(1)(3)(7)#1982
                                 + w(10)(2)(3)(7)#2012 + w(10)(3)(3)(7)#2042
                                 + w(11)(1)(3)(7)#2162 + w(11)(2)(3)(7)#2192
                                 + w(11)(3)(3)(7)#2222 + w(12)(1)(3)(7)#2342
                                 + w(12)(2)(3)(7)#2372 + w(12)(3)(3)(7)#2402
                                  = 0
 WarehouseInventoryBalance(2,3,7): - I_warehouse(2)(2)(7)#55
                                 + I_warehouse(2)(3)(7)#65 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(7)#812 + w(3)(5)(3)(7)#842
                                 + w(3)(6)(3)(7)#872 + w(4)(4)(3)(7)#992
                                 + w(4)(5)(3)(7)#1022 + w(4)(6)(3)(7)#1052
                                 + w(5)(4)(3)(7)#1172 + w(5)(5)(3)(7)#1202
                                 + w(5)(6)(3)(7)#1232 + w(6)(4)(3)(7)#1352
                                 + w(6)(5)(3)(7)#1382 + w(6)(6)(3)(7)#1412
                                 + w(7)(4)(3)(7)#1532 + w(7)(5)(3)(7)#1562
                                 + w(7)(6)(3)(7)#1592 + w(8)(4)(3)(7)#1712
                                 + w(8)(5)(3)(7)#1742 + w(8)(6)(3)(7)#1772
                                 + w(9)(4)(3)(7)#1892 + w(9)(5)(3)(7)#1922
                                 + w(9)(6)(3)(7)#1952 + w(10)(4)(3)(7)#2072
                                 + w(10)(5)(3)(7)#2102 + w(10)(6)(3)(7)#2132
                                 + w(11)(4)(3)(7)#2252 + w(11)(5)(3)(7)#2282
                                 + w(11)(6)(3)(7)#2312 + w(12)(4)(3)(7)#2432
                                 + w(12)(5)(3)(7)#2462 + w(12)(6)(3)(7)#2492
                                  = 0
 WarehouseInventoryBalance(1,1,8): I_warehouse(1)(1)(8)#16 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(8)#703 + w(3)(2)(1)(8)#733
                                 + w(3)(3)(1)(8)#763 + w(4)(1)(1)(8)#883
                                 + w(4)(2)(1)(8)#913 + w(4)(3)(1)(8)#943
                                 + w(5)(1)(1)(8)#1063 + w(5)(2)(1)(8)#1093
                                 + w(5)(3)(1)(8)#1123 + w(6)(1)(1)(8)#1243
                                 + w(6)(2)(1)(8)#1273 + w(6)(3)(1)(8)#1303
                                 + w(7)(1)(1)(8)#1423 + w(7)(2)(1)(8)#1453
                                 + w(7)(3)(1)(8)#1483 + w(8)(1)(1)(8)#1603
                                 + w(8)(2)(1)(8)#1633 + w(8)(3)(1)(8)#1663
                                 + w(9)(1)(1)(8)#1783 + w(9)(2)(1)(8)#1813
                                 + w(9)(3)(1)(8)#1843 + w(10)(1)(1)(8)#1963
                                 + w(10)(2)(1)(8)#1993 + w(10)(3)(1)(8)#2023
                                 + w(11)(1)(1)(8)#2143 + w(11)(2)(1)(8)#2173
                                 + w(11)(3)(1)(8)#2203 + w(12)(1)(1)(8)#2323
                                 + w(12)(2)(1)(8)#2353 + w(12)(3)(1)(8)#2383
                                  = 0
 WarehouseInventoryBalance(2,1,8): I_warehouse(2)(1)(8)#46 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(8)#793 + w(3)(5)(1)(8)#823
                                 + w(3)(6)(1)(8)#853 + w(4)(4)(1)(8)#973
                                 + w(4)(5)(1)(8)#1003 + w(4)(6)(1)(8)#1033
                                 + w(5)(4)(1)(8)#1153 + w(5)(5)(1)(8)#1183
                                 + w(5)(6)(1)(8)#1213 + w(6)(4)(1)(8)#1333
                                 + w(6)(5)(1)(8)#1363 + w(6)(6)(1)(8)#1393
                                 + w(7)(4)(1)(8)#1513 + w(7)(5)(1)(8)#1543
                                 + w(7)(6)(1)(8)#1573 + w(8)(4)(1)(8)#1693
                                 + w(8)(5)(1)(8)#1723 + w(8)(6)(1)(8)#1753
                                 + w(9)(4)(1)(8)#1873 + w(9)(5)(1)(8)#1903
                                 + w(9)(6)(1)(8)#1933 + w(10)(4)(1)(8)#2053
                                 + w(10)(5)(1)(8)#2083 + w(10)(6)(1)(8)#2113
                                 + w(11)(4)(1)(8)#2233 + w(11)(5)(1)(8)#2263
                                 + w(11)(6)(1)(8)#2293 + w(12)(4)(1)(8)#2413
                                 + w(12)(5)(1)(8)#2443 + w(12)(6)(1)(8)#2473
                                  = 0
 WarehouseInventoryBalance(1,2,8): - I_warehouse(1)(1)(8)#16
                                 + I_warehouse(1)(2)(8)#26 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(8)#713 + w(3)(2)(2)(8)#743
                                 + w(3)(3)(2)(8)#773 + w(4)(1)(2)(8)#893
                                 + w(4)(2)(2)(8)#923 + w(4)(3)(2)(8)#953
                                 + w(5)(1)(2)(8)#1073 + w(5)(2)(2)(8)#1103
                                 + w(5)(3)(2)(8)#1133 + w(6)(1)(2)(8)#1253
                                 + w(6)(2)(2)(8)#1283 + w(6)(3)(2)(8)#1313
                                 + w(7)(1)(2)(8)#1433 + w(7)(2)(2)(8)#1463
                                 + w(7)(3)(2)(8)#1493 + w(8)(1)(2)(8)#1613
                                 + w(8)(2)(2)(8)#1643 + w(8)(3)(2)(8)#1673
                                 + w(9)(1)(2)(8)#1793 + w(9)(2)(2)(8)#1823
                                 + w(9)(3)(2)(8)#1853 + w(10)(1)(2)(8)#1973
                                 + w(10)(2)(2)(8)#2003 + w(10)(3)(2)(8)#2033
                                 + w(11)(1)(2)(8)#2153 + w(11)(2)(2)(8)#2183
                                 + w(11)(3)(2)(8)#2213 + w(12)(1)(2)(8)#2333
                                 + w(12)(2)(2)(8)#2363 + w(12)(3)(2)(8)#2393
                                  = 0
 WarehouseInventoryBalance(2,2,8): - I_warehouse(2)(1)(8)#46
                                 + I_warehouse(2)(2)(8)#56 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(8)#803 + w(3)(5)(2)(8)#833
                                 + w(3)(6)(2)(8)#863 + w(4)(4)(2)(8)#983
                                 + w(4)(5)(2)(8)#1013 + w(4)(6)(2)(8)#1043
                                 + w(5)(4)(2)(8)#1163 + w(5)(5)(2)(8)#1193
                                 + w(5)(6)(2)(8)#1223 + w(6)(4)(2)(8)#1343
                                 + w(6)(5)(2)(8)#1373 + w(6)(6)(2)(8)#1403
                                 + w(7)(4)(2)(8)#1523 + w(7)(5)(2)(8)#1553
                                 + w(7)(6)(2)(8)#1583 + w(8)(4)(2)(8)#1703
                                 + w(8)(5)(2)(8)#1733 + w(8)(6)(2)(8)#1763
                                 + w(9)(4)(2)(8)#1883 + w(9)(5)(2)(8)#1913
                                 + w(9)(6)(2)(8)#1943 + w(10)(4)(2)(8)#2063
                                 + w(10)(5)(2)(8)#2093 + w(10)(6)(2)(8)#2123
                                 + w(11)(4)(2)(8)#2243 + w(11)(5)(2)(8)#2273
                                 + w(11)(6)(2)(8)#2303 + w(12)(4)(2)(8)#2423
                                 + w(12)(5)(2)(8)#2453 + w(12)(6)(2)(8)#2483
                                  = 0
 WarehouseInventoryBalance(1,3,8): - I_warehouse(1)(2)(8)#26
                                 + I_warehouse(1)(3)(8)#36 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(8)#723 + w(3)(2)(3)(8)#753
                                 + w(3)(3)(3)(8)#783 + w(4)(1)(3)(8)#903
                                 + w(4)(2)(3)(8)#933 + w(4)(3)(3)(8)#963
                                 + w(5)(1)(3)(8)#1083 + w(5)(2)(3)(8)#1113
                                 + w(5)(3)(3)(8)#1143 + w(6)(1)(3)(8)#1263
                                 + w(6)(2)(3)(8)#1293 + w(6)(3)(3)(8)#1323
                                 + w(7)(1)(3)(8)#1443 + w(7)(2)(3)(8)#1473
                                 + w(7)(3)(3)(8)#1503 + w(8)(1)(3)(8)#1623
                                 + w(8)(2)(3)(8)#1653 + w(8)(3)(3)(8)#1683
                                 + w(9)(1)(3)(8)#1803 + w(9)(2)(3)(8)#1833
                                 + w(9)(3)(3)(8)#1863 + w(10)(1)(3)(8)#1983
                                 + w(10)(2)(3)(8)#2013 + w(10)(3)(3)(8)#2043
                                 + w(11)(1)(3)(8)#2163 + w(11)(2)(3)(8)#2193
                                 + w(11)(3)(3)(8)#2223 + w(12)(1)(3)(8)#2343
                                 + w(12)(2)(3)(8)#2373 + w(12)(3)(3)(8)#2403
                                  = 0
 WarehouseInventoryBalance(2,3,8): - I_warehouse(2)(2)(8)#56
                                 + I_warehouse(2)(3)(8)#66 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(8)#813 + w(3)(5)(3)(8)#843
                                 + w(3)(6)(3)(8)#873 + w(4)(4)(3)(8)#993
                                 + w(4)(5)(3)(8)#1023 + w(4)(6)(3)(8)#1053
                                 + w(5)(4)(3)(8)#1173 + w(5)(5)(3)(8)#1203
                                 + w(5)(6)(3)(8)#1233 + w(6)(4)(3)(8)#1353
                                 + w(6)(5)(3)(8)#1383 + w(6)(6)(3)(8)#1413
                                 + w(7)(4)(3)(8)#1533 + w(7)(5)(3)(8)#1563
                                 + w(7)(6)(3)(8)#1593 + w(8)(4)(3)(8)#1713
                                 + w(8)(5)(3)(8)#1743 + w(8)(6)(3)(8)#1773
                                 + w(9)(4)(3)(8)#1893 + w(9)(5)(3)(8)#1923
                                 + w(9)(6)(3)(8)#1953 + w(10)(4)(3)(8)#2073
                                 + w(10)(5)(3)(8)#2103 + w(10)(6)(3)(8)#2133
                                 + w(11)(4)(3)(8)#2253 + w(11)(5)(3)(8)#2283
                                 + w(11)(6)(3)(8)#2313 + w(12)(4)(3)(8)#2433
                                 + w(12)(5)(3)(8)#2463 + w(12)(6)(3)(8)#2493
                                  = 0
 WarehouseInventoryBalance(1,1,9): I_warehouse(1)(1)(9)#17 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(9)#704 + w(3)(2)(1)(9)#734
                                 + w(3)(3)(1)(9)#764 + w(4)(1)(1)(9)#884
                                 + w(4)(2)(1)(9)#914 + w(4)(3)(1)(9)#944
                                 + w(5)(1)(1)(9)#1064 + w(5)(2)(1)(9)#1094
                                 + w(5)(3)(1)(9)#1124 + w(6)(1)(1)(9)#1244
                                 + w(6)(2)(1)(9)#1274 + w(6)(3)(1)(9)#1304
                                 + w(7)(1)(1)(9)#1424 + w(7)(2)(1)(9)#1454
                                 + w(7)(3)(1)(9)#1484 + w(8)(1)(1)(9)#1604
                                 + w(8)(2)(1)(9)#1634 + w(8)(3)(1)(9)#1664
                                 + w(9)(1)(1)(9)#1784 + w(9)(2)(1)(9)#1814
                                 + w(9)(3)(1)(9)#1844 + w(10)(1)(1)(9)#1964
                                 + w(10)(2)(1)(9)#1994 + w(10)(3)(1)(9)#2024
                                 + w(11)(1)(1)(9)#2144 + w(11)(2)(1)(9)#2174
                                 + w(11)(3)(1)(9)#2204 + w(12)(1)(1)(9)#2324
                                 + w(12)(2)(1)(9)#2354 + w(12)(3)(1)(9)#2384
                                  = 0
 WarehouseInventoryBalance(2,1,9): I_warehouse(2)(1)(9)#47 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(9)#794 + w(3)(5)(1)(9)#824
                                 + w(3)(6)(1)(9)#854 + w(4)(4)(1)(9)#974
                                 + w(4)(5)(1)(9)#1004 + w(4)(6)(1)(9)#1034
                                 + w(5)(4)(1)(9)#1154 + w(5)(5)(1)(9)#1184
                                 + w(5)(6)(1)(9)#1214 + w(6)(4)(1)(9)#1334
                                 + w(6)(5)(1)(9)#1364 + w(6)(6)(1)(9)#1394
                                 + w(7)(4)(1)(9)#1514 + w(7)(5)(1)(9)#1544
                                 + w(7)(6)(1)(9)#1574 + w(8)(4)(1)(9)#1694
                                 + w(8)(5)(1)(9)#1724 + w(8)(6)(1)(9)#1754
                                 + w(9)(4)(1)(9)#1874 + w(9)(5)(1)(9)#1904
                                 + w(9)(6)(1)(9)#1934 + w(10)(4)(1)(9)#2054
                                 + w(10)(5)(1)(9)#2084 + w(10)(6)(1)(9)#2114
                                 + w(11)(4)(1)(9)#2234 + w(11)(5)(1)(9)#2264
                                 + w(11)(6)(1)(9)#2294 + w(12)(4)(1)(9)#2414
                                 + w(12)(5)(1)(9)#2444 + w(12)(6)(1)(9)#2474
                                  = 0
 WarehouseInventoryBalance(1,2,9): - I_warehouse(1)(1)(9)#17
                                 + I_warehouse(1)(2)(9)#27 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(9)#714 + w(3)(2)(2)(9)#744
                                 + w(3)(3)(2)(9)#774 + w(4)(1)(2)(9)#894
                                 + w(4)(2)(2)(9)#924 + w(4)(3)(2)(9)#954
                                 + w(5)(1)(2)(9)#1074 + w(5)(2)(2)(9)#1104
                                 + w(5)(3)(2)(9)#1134 + w(6)(1)(2)(9)#1254
                                 + w(6)(2)(2)(9)#1284 + w(6)(3)(2)(9)#1314
                                 + w(7)(1)(2)(9)#1434 + w(7)(2)(2)(9)#1464
                                 + w(7)(3)(2)(9)#1494 + w(8)(1)(2)(9)#1614
                                 + w(8)(2)(2)(9)#1644 + w(8)(3)(2)(9)#1674
                                 + w(9)(1)(2)(9)#1794 + w(9)(2)(2)(9)#1824
                                 + w(9)(3)(2)(9)#1854 + w(10)(1)(2)(9)#1974
                                 + w(10)(2)(2)(9)#2004 + w(10)(3)(2)(9)#2034
                                 + w(11)(1)(2)(9)#2154 + w(11)(2)(2)(9)#2184
                                 + w(11)(3)(2)(9)#2214 + w(12)(1)(2)(9)#2334
                                 + w(12)(2)(2)(9)#2364 + w(12)(3)(2)(9)#2394
                                  = 0
 WarehouseInventoryBalance(2,2,9): - I_warehouse(2)(1)(9)#47
                                 + I_warehouse(2)(2)(9)#57 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(9)#804 + w(3)(5)(2)(9)#834
                                 + w(3)(6)(2)(9)#864 + w(4)(4)(2)(9)#984
                                 + w(4)(5)(2)(9)#1014 + w(4)(6)(2)(9)#1044
                                 + w(5)(4)(2)(9)#1164 + w(5)(5)(2)(9)#1194
                                 + w(5)(6)(2)(9)#1224 + w(6)(4)(2)(9)#1344
                                 + w(6)(5)(2)(9)#1374 + w(6)(6)(2)(9)#1404
                                 + w(7)(4)(2)(9)#1524 + w(7)(5)(2)(9)#1554
                                 + w(7)(6)(2)(9)#1584 + w(8)(4)(2)(9)#1704
                                 + w(8)(5)(2)(9)#1734 + w(8)(6)(2)(9)#1764
                                 + w(9)(4)(2)(9)#1884 + w(9)(5)(2)(9)#1914
                                 + w(9)(6)(2)(9)#1944 + w(10)(4)(2)(9)#2064
                                 + w(10)(5)(2)(9)#2094 + w(10)(6)(2)(9)#2124
                                 + w(11)(4)(2)(9)#2244 + w(11)(5)(2)(9)#2274
                                 + w(11)(6)(2)(9)#2304 + w(12)(4)(2)(9)#2424
                                 + w(12)(5)(2)(9)#2454 + w(12)(6)(2)(9)#2484
                                  = 0
 WarehouseInventoryBalance(1,3,9): - I_warehouse(1)(2)(9)#27
                                 + I_warehouse(1)(3)(9)#37 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(9)#724 + w(3)(2)(3)(9)#754
                                 + w(3)(3)(3)(9)#784 + w(4)(1)(3)(9)#904
                                 + w(4)(2)(3)(9)#934 + w(4)(3)(3)(9)#964
                                 + w(5)(1)(3)(9)#1084 + w(5)(2)(3)(9)#1114
                                 + w(5)(3)(3)(9)#1144 + w(6)(1)(3)(9)#1264
                                 + w(6)(2)(3)(9)#1294 + w(6)(3)(3)(9)#1324
                                 + w(7)(1)(3)(9)#1444 + w(7)(2)(3)(9)#1474
                                 + w(7)(3)(3)(9)#1504 + w(8)(1)(3)(9)#1624
                                 + w(8)(2)(3)(9)#1654 + w(8)(3)(3)(9)#1684
                                 + w(9)(1)(3)(9)#1804 + w(9)(2)(3)(9)#1834
                                 + w(9)(3)(3)(9)#1864 + w(10)(1)(3)(9)#1984
                                 + w(10)(2)(3)(9)#2014 + w(10)(3)(3)(9)#2044
                                 + w(11)(1)(3)(9)#2164 + w(11)(2)(3)(9)#2194
                                 + w(11)(3)(3)(9)#2224 + w(12)(1)(3)(9)#2344
                                 + w(12)(2)(3)(9)#2374 + w(12)(3)(3)(9)#2404
                                  = 0
 WarehouseInventoryBalance(2,3,9): - I_warehouse(2)(2)(9)#57
                                 + I_warehouse(2)(3)(9)#67 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(9)#814 + w(3)(5)(3)(9)#844
                                 + w(3)(6)(3)(9)#874 + w(4)(4)(3)(9)#994
                                 + w(4)(5)(3)(9)#1024 + w(4)(6)(3)(9)#1054
                                 + w(5)(4)(3)(9)#1174 + w(5)(5)(3)(9)#1204
                                 + w(5)(6)(3)(9)#1234 + w(6)(4)(3)(9)#1354
                                 + w(6)(5)(3)(9)#1384 + w(6)(6)(3)(9)#1414
                                 + w(7)(4)(3)(9)#1534 + w(7)(5)(3)(9)#1564
                                 + w(7)(6)(3)(9)#1594 + w(8)(4)(3)(9)#1714
                                 + w(8)(5)(3)(9)#1744 + w(8)(6)(3)(9)#1774
                                 + w(9)(4)(3)(9)#1894 + w(9)(5)(3)(9)#1924
                                 + w(9)(6)(3)(9)#1954 + w(10)(4)(3)(9)#2074
                                 + w(10)(5)(3)(9)#2104 + w(10)(6)(3)(9)#2134
                                 + w(11)(4)(3)(9)#2254 + w(11)(5)(3)(9)#2284
                                 + w(11)(6)(3)(9)#2314 + w(12)(4)(3)(9)#2434
                                 + w(12)(5)(3)(9)#2464 + w(12)(6)(3)(9)#2494
                                  = 0
 WarehouseInventoryBalance(1,1,10): I_warehouse(1)(1)(10)#18 - q(1)(1)(1)#669
                                 - q(2)(1)(1)#675 - q(3)(1)(1)#681
                                 + w(3)(1)(1)(10)#705 + w(3)(2)(1)(10)#735
                                 + w(3)(3)(1)(10)#765 + w(4)(1)(1)(10)#885
                                 + w(4)(2)(1)(10)#915 + w(4)(3)(1)(10)#945
                                 + w(5)(1)(1)(10)#1065 + w(5)(2)(1)(10)#1095
                                 + w(5)(3)(1)(10)#1125 + w(6)(1)(1)(10)#1245
                                 + w(6)(2)(1)(10)#1275 + w(6)(3)(1)(10)#1305
                                 + w(7)(1)(1)(10)#1425 + w(7)(2)(1)(10)#1455
                                 + w(7)(3)(1)(10)#1485 + w(8)(1)(1)(10)#1605
                                 + w(8)(2)(1)(10)#1635 + w(8)(3)(1)(10)#1665
                                 + w(9)(1)(1)(10)#1785 + w(9)(2)(1)(10)#1815
                                 + w(9)(3)(1)(10)#1845 + w(10)(1)(1)(10)#1965
                                 + w(10)(2)(1)(10)#1995 + w(10)(3)(1)(10)#2025
                                 + w(11)(1)(1)(10)#2145 + w(11)(2)(1)(10)#2175
                                 + w(11)(3)(1)(10)#2205 + w(12)(1)(1)(10)#2325
                                 + w(12)(2)(1)(10)#2355 + w(12)(3)(1)(10)#2385
                                  = 0
 WarehouseInventoryBalance(2,1,10): I_warehouse(2)(1)(10)#48 - q(1)(2)(1)#672
                                 - q(2)(2)(1)#678 - q(3)(2)(1)#684
                                 + w(3)(4)(1)(10)#795 + w(3)(5)(1)(10)#825
                                 + w(3)(6)(1)(10)#855 + w(4)(4)(1)(10)#975
                                 + w(4)(5)(1)(10)#1005 + w(4)(6)(1)(10)#1035
                                 + w(5)(4)(1)(10)#1155 + w(5)(5)(1)(10)#1185
                                 + w(5)(6)(1)(10)#1215 + w(6)(4)(1)(10)#1335
                                 + w(6)(5)(1)(10)#1365 + w(6)(6)(1)(10)#1395
                                 + w(7)(4)(1)(10)#1515 + w(7)(5)(1)(10)#1545
                                 + w(7)(6)(1)(10)#1575 + w(8)(4)(1)(10)#1695
                                 + w(8)(5)(1)(10)#1725 + w(8)(6)(1)(10)#1755
                                 + w(9)(4)(1)(10)#1875 + w(9)(5)(1)(10)#1905
                                 + w(9)(6)(1)(10)#1935 + w(10)(4)(1)(10)#2055
                                 + w(10)(5)(1)(10)#2085 + w(10)(6)(1)(10)#2115
                                 + w(11)(4)(1)(10)#2235 + w(11)(5)(1)(10)#2265
                                 + w(11)(6)(1)(10)#2295 + w(12)(4)(1)(10)#2415
                                 + w(12)(5)(1)(10)#2445 + w(12)(6)(1)(10)#2475
                                  = 0
 WarehouseInventoryBalance(1,2,10): - I_warehouse(1)(1)(10)#18
                                 + I_warehouse(1)(2)(10)#28 - q(1)(1)(2)#670
                                 - q(2)(1)(2)#676 - q(3)(1)(2)#682
                                 + w(3)(1)(2)(10)#715 + w(3)(2)(2)(10)#745
                                 + w(3)(3)(2)(10)#775 + w(4)(1)(2)(10)#895
                                 + w(4)(2)(2)(10)#925 + w(4)(3)(2)(10)#955
                                 + w(5)(1)(2)(10)#1075 + w(5)(2)(2)(10)#1105
                                 + w(5)(3)(2)(10)#1135 + w(6)(1)(2)(10)#1255
                                 + w(6)(2)(2)(10)#1285 + w(6)(3)(2)(10)#1315
                                 + w(7)(1)(2)(10)#1435 + w(7)(2)(2)(10)#1465
                                 + w(7)(3)(2)(10)#1495 + w(8)(1)(2)(10)#1615
                                 + w(8)(2)(2)(10)#1645 + w(8)(3)(2)(10)#1675
                                 + w(9)(1)(2)(10)#1795 + w(9)(2)(2)(10)#1825
                                 + w(9)(3)(2)(10)#1855 + w(10)(1)(2)(10)#1975
                                 + w(10)(2)(2)(10)#2005 + w(10)(3)(2)(10)#2035
                                 + w(11)(1)(2)(10)#2155 + w(11)(2)(2)(10)#2185
                                 + w(11)(3)(2)(10)#2215 + w(12)(1)(2)(10)#2335
                                 + w(12)(2)(2)(10)#2365 + w(12)(3)(2)(10)#2395
                                  = 0
 WarehouseInventoryBalance(2,2,10): - I_warehouse(2)(1)(10)#48
                                 + I_warehouse(2)(2)(10)#58 - q(1)(2)(2)#673
                                 - q(2)(2)(2)#679 - q(3)(2)(2)#685
                                 + w(3)(4)(2)(10)#805 + w(3)(5)(2)(10)#835
                                 + w(3)(6)(2)(10)#865 + w(4)(4)(2)(10)#985
                                 + w(4)(5)(2)(10)#1015 + w(4)(6)(2)(10)#1045
                                 + w(5)(4)(2)(10)#1165 + w(5)(5)(2)(10)#1195
                                 + w(5)(6)(2)(10)#1225 + w(6)(4)(2)(10)#1345
                                 + w(6)(5)(2)(10)#1375 + w(6)(6)(2)(10)#1405
                                 + w(7)(4)(2)(10)#1525 + w(7)(5)(2)(10)#1555
                                 + w(7)(6)(2)(10)#1585 + w(8)(4)(2)(10)#1705
                                 + w(8)(5)(2)(10)#1735 + w(8)(6)(2)(10)#1765
                                 + w(9)(4)(2)(10)#1885 + w(9)(5)(2)(10)#1915
                                 + w(9)(6)(2)(10)#1945 + w(10)(4)(2)(10)#2065
                                 + w(10)(5)(2)(10)#2095 + w(10)(6)(2)(10)#2125
                                 + w(11)(4)(2)(10)#2245 + w(11)(5)(2)(10)#2275
                                 + w(11)(6)(2)(10)#2305 + w(12)(4)(2)(10)#2425
                                 + w(12)(5)(2)(10)#2455 + w(12)(6)(2)(10)#2485
                                  = 0
 WarehouseInventoryBalance(1,3,10): - I_warehouse(1)(2)(10)#28
                                 + I_warehouse(1)(3)(10)#38 - q(1)(1)(3)#671
                                 - q(2)(1)(3)#677 - q(3)(1)(3)#683
                                 + w(3)(1)(3)(10)#725 + w(3)(2)(3)(10)#755
                                 + w(3)(3)(3)(10)#785 + w(4)(1)(3)(10)#905
                                 + w(4)(2)(3)(10)#935 + w(4)(3)(3)(10)#965
                                 + w(5)(1)(3)(10)#1085 + w(5)(2)(3)(10)#1115
                                 + w(5)(3)(3)(10)#1145 + w(6)(1)(3)(10)#1265
                                 + w(6)(2)(3)(10)#1295 + w(6)(3)(3)(10)#1325
                                 + w(7)(1)(3)(10)#1445 + w(7)(2)(3)(10)#1475
                                 + w(7)(3)(3)(10)#1505 + w(8)(1)(3)(10)#1625
                                 + w(8)(2)(3)(10)#1655 + w(8)(3)(3)(10)#1685
                                 + w(9)(1)(3)(10)#1805 + w(9)(2)(3)(10)#1835
                                 + w(9)(3)(3)(10)#1865 + w(10)(1)(3)(10)#1985
                                 + w(10)(2)(3)(10)#2015 + w(10)(3)(3)(10)#2045
                                 + w(11)(1)(3)(10)#2165 + w(11)(2)(3)(10)#2195
                                 + w(11)(3)(3)(10)#2225 + w(12)(1)(3)(10)#2345
                                 + w(12)(2)(3)(10)#2375 + w(12)(3)(3)(10)#2405
                                  = 0
 WarehouseInventoryBalance(2,3,10): - I_warehouse(2)(2)(10)#58
                                 + I_warehouse(2)(3)(10)#68 - q(1)(2)(3)#674
                                 - q(2)(2)(3)#680 - q(3)(2)(3)#686
                                 + w(3)(4)(3)(10)#815 + w(3)(5)(3)(10)#845
                                 + w(3)(6)(3)(10)#875 + w(4)(4)(3)(10)#995
                                 + w(4)(5)(3)(10)#1025 + w(4)(6)(3)(10)#1055
                                 + w(5)(4)(3)(10)#1175 + w(5)(5)(3)(10)#1205
                                 + w(5)(6)(3)(10)#1235 + w(6)(4)(3)(10)#1355
                                 + w(6)(5)(3)(10)#1385 + w(6)(6)(3)(10)#1415
                                 + w(7)(4)(3)(10)#1535 + w(7)(5)(3)(10)#1565
                                 + w(7)(6)(3)(10)#1595 + w(8)(4)(3)(10)#1715
                                 + w(8)(5)(3)(10)#1745 + w(8)(6)(3)(10)#1775
                                 + w(9)(4)(3)(10)#1895 + w(9)(5)(3)(10)#1925
                                 + w(9)(6)(3)(10)#1955 + w(10)(4)(3)(10)#2075
                                 + w(10)(5)(3)(10)#2105 + w(10)(6)(3)(10)#2135
                                 + w(11)(4)(3)(10)#2255 + w(11)(5)(3)(10)#2285
                                 + w(11)(6)(3)(10)#2315 + w(12)(4)(3)(10)#2435
                                 + w(12)(5)(3)(10)#2465 + w(12)(6)(3)(10)#2495
                                  = 0
 CustomerInventoryBalance(3,1,1): I_customer(3)(1)(1)#69
                                 - b_customer(3)(1)(1)#369 - w(3)(1)(1)(1)#696
                                 - w(3)(2)(1)(1)#726 - w(3)(3)(1)(1)#756
                                 - w(3)(4)(1)(1)#786 - w(3)(5)(1)(1)#816
                                 - w(3)(6)(1)(1)#846  = -5
 CustomerInventoryBalance(4,1,1): I_customer(4)(1)(1)#99
                                 - b_customer(4)(1)(1)#399 - w(4)(1)(1)(1)#876
                                 - w(4)(2)(1)(1)#906 - w(4)(3)(1)(1)#936
                                 - w(4)(4)(1)(1)#966 - w(4)(5)(1)(1)#996
                                 - w(4)(6)(1)(1)#1026  = -3
 CustomerInventoryBalance(5,1,1): I_customer(5)(1)(1)#129
                                 - b_customer(5)(1)(1)#429 - w(5)(1)(1)(1)#1056
                                 - w(5)(2)(1)(1)#1086 - w(5)(3)(1)(1)#1116
                                 - w(5)(4)(1)(1)#1146 - w(5)(5)(1)(1)#1176
                                 - w(5)(6)(1)(1)#1206  = 2
 CustomerInventoryBalance(6,1,1): I_customer(6)(1)(1)#159
                                 - b_customer(6)(1)(1)#459 - w(6)(1)(1)(1)#1236
                                 - w(6)(2)(1)(1)#1266 - w(6)(3)(1)(1)#1296
                                 - w(6)(4)(1)(1)#1326 - w(6)(5)(1)(1)#1356
                                 - w(6)(6)(1)(1)#1386  = -3
 CustomerInventoryBalance(7,1,1): I_customer(7)(1)(1)#189
                                 - b_customer(7)(1)(1)#489 - w(7)(1)(1)(1)#1416
                                 - w(7)(2)(1)(1)#1446 - w(7)(3)(1)(1)#1476
                                 - w(7)(4)(1)(1)#1506 - w(7)(5)(1)(1)#1536
                                 - w(7)(6)(1)(1)#1566  = -2
 CustomerInventoryBalance(8,1,1): I_customer(8)(1)(1)#219
                                 - b_customer(8)(1)(1)#519 - w(8)(1)(1)(1)#1596
                                 - w(8)(2)(1)(1)#1626 - w(8)(3)(1)(1)#1656
                                 - w(8)(4)(1)(1)#1686 - w(8)(5)(1)(1)#1716
                                 - w(8)(6)(1)(1)#1746  = 27
 CustomerInventoryBalance(9,1,1): I_customer(9)(1)(1)#249
                                 - b_customer(9)(1)(1)#549 - w(9)(1)(1)(1)#1776
                                 - w(9)(2)(1)(1)#1806 - w(9)(3)(1)(1)#1836
                                 - w(9)(4)(1)(1)#1866 - w(9)(5)(1)(1)#1896
                                 - w(9)(6)(1)(1)#1926  = 29
 CustomerInventoryBalance(10,1,1): I_customer(10)(1)(1)#279
                                 - b_customer(10)(1)(1)#579
                                 - w(10)(1)(1)(1)#1956 - w(10)(2)(1)(1)#1986
                                 - w(10)(3)(1)(1)#2016 - w(10)(4)(1)(1)#2046
                                 - w(10)(5)(1)(1)#2076 - w(10)(6)(1)(1)#2106
                                  = -6
 CustomerInventoryBalance(11,1,1): I_customer(11)(1)(1)#309
                                 - b_customer(11)(1)(1)#609
                                 - w(11)(1)(1)(1)#2136 - w(11)(2)(1)(1)#2166
                                 - w(11)(3)(1)(1)#2196 - w(11)(4)(1)(1)#2226
                                 - w(11)(5)(1)(1)#2256 - w(11)(6)(1)(1)#2286
                                  = 32
 CustomerInventoryBalance(12,1,1): I_customer(12)(1)(1)#339
                                 - b_customer(12)(1)(1)#639
                                 - w(12)(1)(1)(1)#2316 - w(12)(2)(1)(1)#2346
                                 - w(12)(3)(1)(1)#2376 - w(12)(4)(1)(1)#2406
                                 - w(12)(5)(1)(1)#2436 - w(12)(6)(1)(1)#2466
                                  = 23
 CustomerInventoryBalance(3,2,1): - I_customer(3)(1)(1)#69
                                 + I_customer(3)(2)(1)#79
                                 - b_customer(3)(2)(1)#379 - w(3)(1)(2)(1)#706
                                 - w(3)(2)(2)(1)#736 - w(3)(3)(2)(1)#766
                                 - w(3)(4)(2)(1)#796 - w(3)(5)(2)(1)#826
                                 - w(3)(6)(2)(1)#856  = -9
 CustomerInventoryBalance(4,2,1): - I_customer(4)(1)(1)#99
                                 + I_customer(4)(2)(1)#109
                                 - b_customer(4)(2)(1)#409 - w(4)(1)(2)(1)#886
                                 - w(4)(2)(2)(1)#916 - w(4)(3)(2)(1)#946
                                 - w(4)(4)(2)(1)#976 - w(4)(5)(2)(1)#1006
                                 - w(4)(6)(2)(1)#1036  = -12
 CustomerInventoryBalance(5,2,1): - I_customer(5)(1)(1)#129
                                 + I_customer(5)(2)(1)#139
                                 - b_customer(5)(2)(1)#439 - w(5)(1)(2)(1)#1066
                                 - w(5)(2)(2)(1)#1096 - w(5)(3)(2)(1)#1126
                                 - w(5)(4)(2)(1)#1156 - w(5)(5)(2)(1)#1186
                                 - w(5)(6)(2)(1)#1216  = -15
 CustomerInventoryBalance(6,2,1): - I_customer(6)(1)(1)#159
                                 + I_customer(6)(2)(1)#169
                                 - b_customer(6)(2)(1)#469 - w(6)(1)(2)(1)#1246
                                 - w(6)(2)(2)(1)#1276 - w(6)(3)(2)(1)#1306
                                 - w(6)(4)(2)(1)#1336 - w(6)(5)(2)(1)#1366
                                 - w(6)(6)(2)(1)#1396  = -7
 CustomerInventoryBalance(7,2,1): - I_customer(7)(1)(1)#189
                                 + I_customer(7)(2)(1)#199
                                 - b_customer(7)(2)(1)#499 - w(7)(1)(2)(1)#1426
                                 - w(7)(2)(2)(1)#1456 - w(7)(3)(2)(1)#1486
                                 - w(7)(4)(2)(1)#1516 - w(7)(5)(2)(1)#1546
                                 - w(7)(6)(2)(1)#1576  = -15
 CustomerInventoryBalance(8,2,1): - I_customer(8)(1)(1)#219
                                 + I_customer(8)(2)(1)#229
                                 - b_customer(8)(2)(1)#529 - w(8)(1)(2)(1)#1606
                                 - w(8)(2)(2)(1)#1636 - w(8)(3)(2)(1)#1666
                                 - w(8)(4)(2)(1)#1696 - w(8)(5)(2)(1)#1726
                                 - w(8)(6)(2)(1)#1756  = -18
 CustomerInventoryBalance(9,2,1): - I_customer(9)(1)(1)#249
                                 + I_customer(9)(2)(1)#259
                                 - b_customer(9)(2)(1)#559 - w(9)(1)(2)(1)#1786
                                 - w(9)(2)(2)(1)#1816 - w(9)(3)(2)(1)#1846
                                 - w(9)(4)(2)(1)#1876 - w(9)(5)(2)(1)#1906
                                 - w(9)(6)(2)(1)#1936  = -20
 CustomerInventoryBalance(10,2,1): - I_customer(10)(1)(1)#279
                                 + I_customer(10)(2)(1)#289
                                 - b_customer(10)(2)(1)#589
                                 - w(10)(1)(2)(1)#1966 - w(10)(2)(2)(1)#1996
                                 - w(10)(3)(2)(1)#2026 - w(10)(4)(2)(1)#2056
                                 - w(10)(5)(2)(1)#2086 - w(10)(6)(2)(1)#2116
                                  = -11
 CustomerInventoryBalance(11,2,1): - I_customer(11)(1)(1)#309
                                 + I_customer(11)(2)(1)#319
                                 - b_customer(11)(2)(1)#619
                                 - w(11)(1)(2)(1)#2146 - w(11)(2)(2)(1)#2176
                                 - w(11)(3)(2)(1)#2206 - w(11)(4)(2)(1)#2236
                                 - w(11)(5)(2)(1)#2266 - w(11)(6)(2)(1)#2296
                                  = -16
 CustomerInventoryBalance(12,2,1): - I_customer(12)(1)(1)#339
                                 + I_customer(12)(2)(1)#349
                                 - b_customer(12)(2)(1)#649
                                 - w(12)(1)(2)(1)#2326 - w(12)(2)(2)(1)#2356
                                 - w(12)(3)(2)(1)#2386 - w(12)(4)(2)(1)#2416
                                 - w(12)(5)(2)(1)#2446 - w(12)(6)(2)(1)#2476
                                  = -21
 CustomerInventoryBalance(3,3,1): - I_customer(3)(2)(1)#79
                                 + I_customer(3)(3)(1)#89
                                 - b_customer(3)(3)(1)#389 - w(3)(1)(3)(1)#716
                                 - w(3)(2)(3)(1)#746 - w(3)(3)(3)(1)#776
                                 - w(3)(4)(3)(1)#806 - w(3)(5)(3)(1)#836
                                 - w(3)(6)(3)(1)#866  = -12
 CustomerInventoryBalance(4,3,1): - I_customer(4)(2)(1)#109
                                 + I_customer(4)(3)(1)#119
                                 - b_customer(4)(3)(1)#419 - w(4)(1)(3)(1)#896
                                 - w(4)(2)(3)(1)#926 - w(4)(3)(3)(1)#956
                                 - w(4)(4)(3)(1)#986 - w(4)(5)(3)(1)#1016
                                 - w(4)(6)(3)(1)#1046  = -17
 CustomerInventoryBalance(5,3,1): - I_customer(5)(2)(1)#139
                                 + I_customer(5)(3)(1)#149
                                 - b_customer(5)(3)(1)#449 - w(5)(1)(3)(1)#1076
                                 - w(5)(2)(3)(1)#1106 - w(5)(3)(3)(1)#1136
                                 - w(5)(4)(3)(1)#1166 - w(5)(5)(3)(1)#1196
                                 - w(5)(6)(3)(1)#1226  = -16
 CustomerInventoryBalance(6,3,1): - I_customer(6)(2)(1)#169
                                 + I_customer(6)(3)(1)#179
                                 - b_customer(6)(3)(1)#479 - w(6)(1)(3)(1)#1256
                                 - w(6)(2)(3)(1)#1286 - w(6)(3)(3)(1)#1316
                                 - w(6)(4)(3)(1)#1346 - w(6)(5)(3)(1)#1376
                                 - w(6)(6)(3)(1)#1406  = -7
 CustomerInventoryBalance(7,3,1): - I_customer(7)(2)(1)#199
                                 + I_customer(7)(3)(1)#209
                                 - b_customer(7)(3)(1)#509 - w(7)(1)(3)(1)#1436
                                 - w(7)(2)(3)(1)#1466 - w(7)(3)(3)(1)#1496
                                 - w(7)(4)(3)(1)#1526 - w(7)(5)(3)(1)#1556
                                 - w(7)(6)(3)(1)#1586  = -12
 CustomerInventoryBalance(8,3,1): - I_customer(8)(2)(1)#229
                                 + I_customer(8)(3)(1)#239
                                 - b_customer(8)(3)(1)#539 - w(8)(1)(3)(1)#1616
                                 - w(8)(2)(3)(1)#1646 - w(8)(3)(3)(1)#1676
                                 - w(8)(4)(3)(1)#1706 - w(8)(5)(3)(1)#1736
                                 - w(8)(6)(3)(1)#1766  = -17
 CustomerInventoryBalance(9,3,1): - I_customer(9)(2)(1)#259
                                 + I_customer(9)(3)(1)#269
                                 - b_customer(9)(3)(1)#569 - w(9)(1)(3)(1)#1796
                                 - w(9)(2)(3)(1)#1826 - w(9)(3)(3)(1)#1856
                                 - w(9)(4)(3)(1)#1886 - w(9)(5)(3)(1)#1916
                                 - w(9)(6)(3)(1)#1946  = -24
 CustomerInventoryBalance(10,3,1): - I_customer(10)(2)(1)#289
                                 + I_customer(10)(3)(1)#299
                                 - b_customer(10)(3)(1)#599
                                 - w(10)(1)(3)(1)#1976 - w(10)(2)(3)(1)#2006
                                 - w(10)(3)(3)(1)#2036 - w(10)(4)(3)(1)#2066
                                 - w(10)(5)(3)(1)#2096 - w(10)(6)(3)(1)#2126
                                  = -13
 CustomerInventoryBalance(11,3,1): - I_customer(11)(2)(1)#319
                                 + I_customer(11)(3)(1)#329
                                 - b_customer(11)(3)(1)#629
                                 - w(11)(1)(3)(1)#2156 - w(11)(2)(3)(1)#2186
                                 - w(11)(3)(3)(1)#2216 - w(11)(4)(3)(1)#2246
                                 - w(11)(5)(3)(1)#2276 - w(11)(6)(3)(1)#2306
                                  = -16
 CustomerInventoryBalance(12,3,1): - I_customer(12)(2)(1)#349
                                 + I_customer(12)(3)(1)#359
                                 - b_customer(12)(3)(1)#659
                                 - w(12)(1)(3)(1)#2336 - w(12)(2)(3)(1)#2366
                                 - w(12)(3)(3)(1)#2396 - w(12)(4)(3)(1)#2426
                                 - w(12)(5)(3)(1)#2456 - w(12)(6)(3)(1)#2486
                                  = -25
 CustomerInventoryBalance(3,1,2): I_customer(3)(1)(2)#70
                                 - b_customer(3)(1)(2)#370 - w(3)(1)(1)(2)#697
                                 - w(3)(2)(1)(2)#727 - w(3)(3)(1)(2)#757
                                 - w(3)(4)(1)(2)#787 - w(3)(5)(1)(2)#817
                                 - w(3)(6)(1)(2)#847  = -7
 CustomerInventoryBalance(4,1,2): I_customer(4)(1)(2)#100
                                 - b_customer(4)(1)(2)#400 - w(4)(1)(1)(2)#877
                                 - w(4)(2)(1)(2)#907 - w(4)(3)(1)(2)#937
                                 - w(4)(4)(1)(2)#967 - w(4)(5)(1)(2)#997
                                 - w(4)(6)(1)(2)#1027  = 3
 CustomerInventoryBalance(5,1,2): I_customer(5)(1)(2)#130
                                 - b_customer(5)(1)(2)#430 - w(5)(1)(1)(2)#1057
                                 - w(5)(2)(1)(2)#1087 - w(5)(3)(1)(2)#1117
                                 - w(5)(4)(1)(2)#1147 - w(5)(5)(1)(2)#1177
                                 - w(5)(6)(1)(2)#1207  = 3
 CustomerInventoryBalance(6,1,2): I_customer(6)(1)(2)#160
                                 - b_customer(6)(1)(2)#460 - w(6)(1)(1)(2)#1237
                                 - w(6)(2)(1)(2)#1267 - w(6)(3)(1)(2)#1297
                                 - w(6)(4)(1)(2)#1327 - w(6)(5)(1)(2)#1357
                                 - w(6)(6)(1)(2)#1387  = -5
 CustomerInventoryBalance(7,1,2): I_customer(7)(1)(2)#190
                                 - b_customer(7)(1)(2)#490 - w(7)(1)(1)(2)#1417
                                 - w(7)(2)(1)(2)#1447 - w(7)(3)(1)(2)#1477
                                 - w(7)(4)(1)(2)#1507 - w(7)(5)(1)(2)#1537
                                 - w(7)(6)(1)(2)#1567  = 2
 CustomerInventoryBalance(8,1,2): I_customer(8)(1)(2)#220
                                 - b_customer(8)(1)(2)#520 - w(8)(1)(1)(2)#1597
                                 - w(8)(2)(1)(2)#1627 - w(8)(3)(1)(2)#1657
                                 - w(8)(4)(1)(2)#1687 - w(8)(5)(1)(2)#1717
                                 - w(8)(6)(1)(2)#1747  = 24
 CustomerInventoryBalance(9,1,2): I_customer(9)(1)(2)#250
                                 - b_customer(9)(1)(2)#550 - w(9)(1)(1)(2)#1777
                                 - w(9)(2)(1)(2)#1807 - w(9)(3)(1)(2)#1837
                                 - w(9)(4)(1)(2)#1867 - w(9)(5)(1)(2)#1897
                                 - w(9)(6)(1)(2)#1927  = 37
 CustomerInventoryBalance(10,1,2): I_customer(10)(1)(2)#280
                                 - b_customer(10)(1)(2)#580
                                 - w(10)(1)(1)(2)#1957 - w(10)(2)(1)(2)#1987
                                 - w(10)(3)(1)(2)#2017 - w(10)(4)(1)(2)#2047
                                 - w(10)(5)(1)(2)#2077 - w(10)(6)(1)(2)#2107
                                  = -5
 CustomerInventoryBalance(11,1,2): I_customer(11)(1)(2)#310
                                 - b_customer(11)(1)(2)#610
                                 - w(11)(1)(1)(2)#2137 - w(11)(2)(1)(2)#2167
                                 - w(11)(3)(1)(2)#2197 - w(11)(4)(1)(2)#2227
                                 - w(11)(5)(1)(2)#2257 - w(11)(6)(1)(2)#2287
                                  = 25
 CustomerInventoryBalance(12,1,2): I_customer(12)(1)(2)#340
                                 - b_customer(12)(1)(2)#640
                                 - w(12)(1)(1)(2)#2317 - w(12)(2)(1)(2)#2347
                                 - w(12)(3)(1)(2)#2377 - w(12)(4)(1)(2)#2407
                                 - w(12)(5)(1)(2)#2437 - w(12)(6)(1)(2)#2467
                                  = 26
 CustomerInventoryBalance(3,2,2): - I_customer(3)(1)(2)#70
                                 + I_customer(3)(2)(2)#80
                                 - b_customer(3)(2)(2)#380 - w(3)(1)(2)(2)#707
                                 - w(3)(2)(2)(2)#737 - w(3)(3)(2)(2)#767
                                 - w(3)(4)(2)(2)#797 - w(3)(5)(2)(2)#827
                                 - w(3)(6)(2)(2)#857  = -11
 CustomerInventoryBalance(4,2,2): - I_customer(4)(1)(2)#100
                                 + I_customer(4)(2)(2)#110
                                 - b_customer(4)(2)(2)#410 - w(4)(1)(2)(2)#887
                                 - w(4)(2)(2)(2)#917 - w(4)(3)(2)(2)#947
                                 - w(4)(4)(2)(2)#977 - w(4)(5)(2)(2)#1007
                                 - w(4)(6)(2)(2)#1037  = -16
 CustomerInventoryBalance(5,2,2): - I_customer(5)(1)(2)#130
                                 + I_customer(5)(2)(2)#140
                                 - b_customer(5)(2)(2)#440 - w(5)(1)(2)(2)#1067
                                 - w(5)(2)(2)(2)#1097 - w(5)(3)(2)(2)#1127
                                 - w(5)(4)(2)(2)#1157 - w(5)(5)(2)(2)#1187
                                 - w(5)(6)(2)(2)#1217  = -17
 CustomerInventoryBalance(6,2,2): - I_customer(6)(1)(2)#160
                                 + I_customer(6)(2)(2)#170
                                 - b_customer(6)(2)(2)#470 - w(6)(1)(2)(2)#1247
                                 - w(6)(2)(2)(2)#1277 - w(6)(3)(2)(2)#1307
                                 - w(6)(4)(2)(2)#1337 - w(6)(5)(2)(2)#1367
                                 - w(6)(6)(2)(2)#1397  = -7
 CustomerInventoryBalance(7,2,2): - I_customer(7)(1)(2)#190
                                 + I_customer(7)(2)(2)#200
                                 - b_customer(7)(2)(2)#500 - w(7)(1)(2)(2)#1427
                                 - w(7)(2)(2)(2)#1457 - w(7)(3)(2)(2)#1487
                                 - w(7)(4)(2)(2)#1517 - w(7)(5)(2)(2)#1547
                                 - w(7)(6)(2)(2)#1577  = -13
 CustomerInventoryBalance(8,2,2): - I_customer(8)(1)(2)#220
                                 + I_customer(8)(2)(2)#230
                                 - b_customer(8)(2)(2)#530 - w(8)(1)(2)(2)#1607
                                 - w(8)(2)(2)(2)#1637 - w(8)(3)(2)(2)#1667
                                 - w(8)(4)(2)(2)#1697 - w(8)(5)(2)(2)#1727
                                 - w(8)(6)(2)(2)#1757  = -15
 CustomerInventoryBalance(9,2,2): - I_customer(9)(1)(2)#250
                                 + I_customer(9)(2)(2)#260
                                 - b_customer(9)(2)(2)#560 - w(9)(1)(2)(2)#1787
                                 - w(9)(2)(2)(2)#1817 - w(9)(3)(2)(2)#1847
                                 - w(9)(4)(2)(2)#1877 - w(9)(5)(2)(2)#1907
                                 - w(9)(6)(2)(2)#1937  = -20
 CustomerInventoryBalance(10,2,2): - I_customer(10)(1)(2)#280
                                 + I_customer(10)(2)(2)#290
                                 - b_customer(10)(2)(2)#590
                                 - w(10)(1)(2)(2)#1967 - w(10)(2)(2)(2)#1997
                                 - w(10)(3)(2)(2)#2027 - w(10)(4)(2)(2)#2057
                                 - w(10)(5)(2)(2)#2087 - w(10)(6)(2)(2)#2117
                                  = -15
 CustomerInventoryBalance(11,2,2): - I_customer(11)(1)(2)#310
                                 + I_customer(11)(2)(2)#320
                                 - b_customer(11)(2)(2)#620
                                 - w(11)(1)(2)(2)#2147 - w(11)(2)(2)(2)#2177
                                 - w(11)(3)(2)(2)#2207 - w(11)(4)(2)(2)#2237
                                 - w(11)(5)(2)(2)#2267 - w(11)(6)(2)(2)#2297
                                  = -18
 CustomerInventoryBalance(12,2,2): - I_customer(12)(1)(2)#340
                                 + I_customer(12)(2)(2)#350
                                 - b_customer(12)(2)(2)#650
                                 - w(12)(1)(2)(2)#2327 - w(12)(2)(2)(2)#2357
                                 - w(12)(3)(2)(2)#2387 - w(12)(4)(2)(2)#2417
                                 - w(12)(5)(2)(2)#2447 - w(12)(6)(2)(2)#2477
                                  = -22
 CustomerInventoryBalance(3,3,2): - I_customer(3)(2)(2)#80
                                 + I_customer(3)(3)(2)#90
                                 - b_customer(3)(3)(2)#390 - w(3)(1)(3)(2)#717
                                 - w(3)(2)(3)(2)#747 - w(3)(3)(3)(2)#777
                                 - w(3)(4)(3)(2)#807 - w(3)(5)(3)(2)#837
                                 - w(3)(6)(3)(2)#867  = -9
 CustomerInventoryBalance(4,3,2): - I_customer(4)(2)(2)#110
                                 + I_customer(4)(3)(2)#120
                                 - b_customer(4)(3)(2)#420 - w(4)(1)(3)(2)#897
                                 - w(4)(2)(3)(2)#927 - w(4)(3)(3)(2)#957
                                 - w(4)(4)(3)(2)#987 - w(4)(5)(3)(2)#1017
                                 - w(4)(6)(3)(2)#1047  = -14
 CustomerInventoryBalance(5,3,2): - I_customer(5)(2)(2)#140
                                 + I_customer(5)(3)(2)#150
                                 - b_customer(5)(3)(2)#450 - w(5)(1)(3)(2)#1077
                                 - w(5)(2)(3)(2)#1107 - w(5)(3)(3)(2)#1137
                                 - w(5)(4)(3)(2)#1167 - w(5)(5)(3)(2)#1197
                                 - w(5)(6)(3)(2)#1227  = -17
 CustomerInventoryBalance(6,3,2): - I_customer(6)(2)(2)#170
                                 + I_customer(6)(3)(2)#180
                                 - b_customer(6)(3)(2)#480 - w(6)(1)(3)(2)#1257
                                 - w(6)(2)(3)(2)#1287 - w(6)(3)(3)(2)#1317
                                 - w(6)(4)(3)(2)#1347 - w(6)(5)(3)(2)#1377
                                 - w(6)(6)(3)(2)#1407  = -8
 CustomerInventoryBalance(7,3,2): - I_customer(7)(2)(2)#200
                                 + I_customer(7)(3)(2)#210
                                 - b_customer(7)(3)(2)#510 - w(7)(1)(3)(2)#1437
                                 - w(7)(2)(3)(2)#1467 - w(7)(3)(3)(2)#1497
                                 - w(7)(4)(3)(2)#1527 - w(7)(5)(3)(2)#1557
                                 - w(7)(6)(3)(2)#1587  = -11
 CustomerInventoryBalance(8,3,2): - I_customer(8)(2)(2)#230
                                 + I_customer(8)(3)(2)#240
                                 - b_customer(8)(3)(2)#540 - w(8)(1)(3)(2)#1617
                                 - w(8)(2)(3)(2)#1647 - w(8)(3)(3)(2)#1677
                                 - w(8)(4)(3)(2)#1707 - w(8)(5)(3)(2)#1737
                                 - w(8)(6)(3)(2)#1767  = -17
 CustomerInventoryBalance(9,3,2): - I_customer(9)(2)(2)#260
                                 + I_customer(9)(3)(2)#270
                                 - b_customer(9)(3)(2)#570 - w(9)(1)(3)(2)#1797
                                 - w(9)(2)(3)(2)#1827 - w(9)(3)(3)(2)#1857
                                 - w(9)(4)(3)(2)#1887 - w(9)(5)(3)(2)#1917
                                 - w(9)(6)(3)(2)#1947  = -18
 CustomerInventoryBalance(10,3,2): - I_customer(10)(2)(2)#290
                                 + I_customer(10)(3)(2)#300
                                 - b_customer(10)(3)(2)#600
                                 - w(10)(1)(3)(2)#1977 - w(10)(2)(3)(2)#2007
                                 - w(10)(3)(3)(2)#2037 - w(10)(4)(3)(2)#2067
                                 - w(10)(5)(3)(2)#2097 - w(10)(6)(3)(2)#2127
                                  = -14
 CustomerInventoryBalance(11,3,2): - I_customer(11)(2)(2)#320
                                 + I_customer(11)(3)(2)#330
                                 - b_customer(11)(3)(2)#630
                                 - w(11)(1)(3)(2)#2157 - w(11)(2)(3)(2)#2187
                                 - w(11)(3)(3)(2)#2217 - w(11)(4)(3)(2)#2247
                                 - w(11)(5)(3)(2)#2277 - w(11)(6)(3)(2)#2307
                                  = -22
 CustomerInventoryBalance(12,3,2): - I_customer(12)(2)(2)#350
                                 + I_customer(12)(3)(2)#360
                                 - b_customer(12)(3)(2)#660
                                 - w(12)(1)(3)(2)#2337 - w(12)(2)(3)(2)#2367
                                 - w(12)(3)(3)(2)#2397 - w(12)(4)(3)(2)#2427
                                 - w(12)(5)(3)(2)#2457 - w(12)(6)(3)(2)#2487
                                  = -25
 CustomerInventoryBalance(3,1,3): I_customer(3)(1)(3)#71
                                 - b_customer(3)(1)(3)#371 - w(3)(1)(1)(3)#698
                                 - w(3)(2)(1)(3)#728 - w(3)(3)(1)(3)#758
                                 - w(3)(4)(1)(3)#788 - w(3)(5)(1)(3)#818
                                 - w(3)(6)(1)(3)#848  = -6
 CustomerInventoryBalance(4,1,3): I_customer(4)(1)(3)#101
                                 - b_customer(4)(1)(3)#401 - w(4)(1)(1)(3)#878
                                 - w(4)(2)(1)(3)#908 - w(4)(3)(1)(3)#938
                                 - w(4)(4)(1)(3)#968 - w(4)(5)(1)(3)#998
                                 - w(4)(6)(1)(3)#1028  = -2
 CustomerInventoryBalance(5,1,3): I_customer(5)(1)(3)#131
                                 - b_customer(5)(1)(3)#431 - w(5)(1)(1)(3)#1058
                                 - w(5)(2)(1)(3)#1088 - w(5)(3)(1)(3)#1118
                                 - w(5)(4)(1)(3)#1148 - w(5)(5)(1)(3)#1178
                                 - w(5)(6)(1)(3)#1208  = 0
 CustomerInventoryBalance(6,1,3): I_customer(6)(1)(3)#161
                                 - b_customer(6)(1)(3)#461 - w(6)(1)(1)(3)#1238
                                 - w(6)(2)(1)(3)#1268 - w(6)(3)(1)(3)#1298
                                 - w(6)(4)(1)(3)#1328 - w(6)(5)(1)(3)#1358
                                 - w(6)(6)(1)(3)#1388  = -4
 CustomerInventoryBalance(7,1,3): I_customer(7)(1)(3)#191
                                 - b_customer(7)(1)(3)#491 - w(7)(1)(1)(3)#1418
                                 - w(7)(2)(1)(3)#1448 - w(7)(3)(1)(3)#1478
                                 - w(7)(4)(1)(3)#1508 - w(7)(5)(1)(3)#1538
                                 - w(7)(6)(1)(3)#1568  = -1
 CustomerInventoryBalance(8,1,3): I_customer(8)(1)(3)#221
                                 - b_customer(8)(1)(3)#521 - w(8)(1)(1)(3)#1598
                                 - w(8)(2)(1)(3)#1628 - w(8)(3)(1)(3)#1658
                                 - w(8)(4)(1)(3)#1688 - w(8)(5)(1)(3)#1718
                                 - w(8)(6)(1)(3)#1748  = 22
 CustomerInventoryBalance(9,1,3): I_customer(9)(1)(3)#251
                                 - b_customer(9)(1)(3)#551 - w(9)(1)(1)(3)#1778
                                 - w(9)(2)(1)(3)#1808 - w(9)(3)(1)(3)#1838
                                 - w(9)(4)(1)(3)#1868 - w(9)(5)(1)(3)#1898
                                 - w(9)(6)(1)(3)#1928  = 36
 CustomerInventoryBalance(10,1,3): I_customer(10)(1)(3)#281
                                 - b_customer(10)(1)(3)#581
                                 - w(10)(1)(1)(3)#1958 - w(10)(2)(1)(3)#1988
                                 - w(10)(3)(1)(3)#2018 - w(10)(4)(1)(3)#2048
                                 - w(10)(5)(1)(3)#2078 - w(10)(6)(1)(3)#2108
                                  = -8
 CustomerInventoryBalance(11,1,3): I_customer(11)(1)(3)#311
                                 - b_customer(11)(1)(3)#611
                                 - w(11)(1)(1)(3)#2138 - w(11)(2)(1)(3)#2168
                                 - w(11)(3)(1)(3)#2198 - w(11)(4)(1)(3)#2228
                                 - w(11)(5)(1)(3)#2258 - w(11)(6)(1)(3)#2288
                                  = 29
 CustomerInventoryBalance(12,1,3): I_customer(12)(1)(3)#341
                                 - b_customer(12)(1)(3)#641
                                 - w(12)(1)(1)(3)#2318 - w(12)(2)(1)(3)#2348
                                 - w(12)(3)(1)(3)#2378 - w(12)(4)(1)(3)#2408
                                 - w(12)(5)(1)(3)#2438 - w(12)(6)(1)(3)#2468
                                  = 23
 CustomerInventoryBalance(3,2,3): - I_customer(3)(1)(3)#71
                                 + I_customer(3)(2)(3)#81
                                 - b_customer(3)(2)(3)#381 - w(3)(1)(2)(3)#708
                                 - w(3)(2)(2)(3)#738 - w(3)(3)(2)(3)#768
                                 - w(3)(4)(2)(3)#798 - w(3)(5)(2)(3)#828
                                 - w(3)(6)(2)(3)#858  = -11
 CustomerInventoryBalance(4,2,3): - I_customer(4)(1)(3)#101
                                 + I_customer(4)(2)(3)#111
                                 - b_customer(4)(2)(3)#411 - w(4)(1)(2)(3)#888
                                 - w(4)(2)(2)(3)#918 - w(4)(3)(2)(3)#948
                                 - w(4)(4)(2)(3)#978 - w(4)(5)(2)(3)#1008
                                 - w(4)(6)(2)(3)#1038  = -17
 CustomerInventoryBalance(5,2,3): - I_customer(5)(1)(3)#131
                                 + I_customer(5)(2)(3)#141
                                 - b_customer(5)(2)(3)#441 - w(5)(1)(2)(3)#1068
                                 - w(5)(2)(2)(3)#1098 - w(5)(3)(2)(3)#1128
                                 - w(5)(4)(2)(3)#1158 - w(5)(5)(2)(3)#1188
                                 - w(5)(6)(2)(3)#1218  = -18
 CustomerInventoryBalance(6,2,3): - I_customer(6)(1)(3)#161
                                 + I_customer(6)(2)(3)#171
                                 - b_customer(6)(2)(3)#471 - w(6)(1)(2)(3)#1248
                                 - w(6)(2)(2)(3)#1278 - w(6)(3)(2)(3)#1308
                                 - w(6)(4)(2)(3)#1338 - w(6)(5)(2)(3)#1368
                                 - w(6)(6)(2)(3)#1398  = -6
 CustomerInventoryBalance(7,2,3): - I_customer(7)(1)(3)#191
                                 + I_customer(7)(2)(3)#201
                                 - b_customer(7)(2)(3)#501 - w(7)(1)(2)(3)#1428
                                 - w(7)(2)(2)(3)#1458 - w(7)(3)(2)(3)#1488
                                 - w(7)(4)(2)(3)#1518 - w(7)(5)(2)(3)#1548
                                 - w(7)(6)(2)(3)#1578  = -13
 CustomerInventoryBalance(8,2,3): - I_customer(8)(1)(3)#221
                                 + I_customer(8)(2)(3)#231
                                 - b_customer(8)(2)(3)#531 - w(8)(1)(2)(3)#1608
                                 - w(8)(2)(2)(3)#1638 - w(8)(3)(2)(3)#1668
                                 - w(8)(4)(2)(3)#1698 - w(8)(5)(2)(3)#1728
                                 - w(8)(6)(2)(3)#1758  = -17
 CustomerInventoryBalance(9,2,3): - I_customer(9)(1)(3)#251
                                 + I_customer(9)(2)(3)#261
                                 - b_customer(9)(2)(3)#561 - w(9)(1)(2)(3)#1788
                                 - w(9)(2)(2)(3)#1818 - w(9)(3)(2)(3)#1848
                                 - w(9)(4)(2)(3)#1878 - w(9)(5)(2)(3)#1908
                                 - w(9)(6)(2)(3)#1938  = -22
 CustomerInventoryBalance(10,2,3): - I_customer(10)(1)(3)#281
                                 + I_customer(10)(2)(3)#291
                                 - b_customer(10)(2)(3)#591
                                 - w(10)(1)(2)(3)#1968 - w(10)(2)(2)(3)#1998
                                 - w(10)(3)(2)(3)#2028 - w(10)(4)(2)(3)#2058
                                 - w(10)(5)(2)(3)#2088 - w(10)(6)(2)(3)#2118
                                  = -11
 CustomerInventoryBalance(11,2,3): - I_customer(11)(1)(3)#311
                                 + I_customer(11)(2)(3)#321
                                 - b_customer(11)(2)(3)#621
                                 - w(11)(1)(2)(3)#2148 - w(11)(2)(2)(3)#2178
                                 - w(11)(3)(2)(3)#2208 - w(11)(4)(2)(3)#2238
                                 - w(11)(5)(2)(3)#2268 - w(11)(6)(2)(3)#2298
                                  = -22
 CustomerInventoryBalance(12,2,3): - I_customer(12)(1)(3)#341
                                 + I_customer(12)(2)(3)#351
                                 - b_customer(12)(2)(3)#651
                                 - w(12)(1)(2)(3)#2328 - w(12)(2)(2)(3)#2358
                                 - w(12)(3)(2)(3)#2388 - w(12)(4)(2)(3)#2418
                                 - w(12)(5)(2)(3)#2448 - w(12)(6)(2)(3)#2478
                                  = -21
 CustomerInventoryBalance(3,3,3): - I_customer(3)(2)(3)#81
                                 + I_customer(3)(3)(3)#91
                                 - b_customer(3)(3)(3)#391 - w(3)(1)(3)(3)#718
                                 - w(3)(2)(3)(3)#748 - w(3)(3)(3)(3)#778
                                 - w(3)(4)(3)(3)#808 - w(3)(5)(3)(3)#838
                                 - w(3)(6)(3)(3)#868  = -11
 CustomerInventoryBalance(4,3,3): - I_customer(4)(2)(3)#111
                                 + I_customer(4)(3)(3)#121
                                 - b_customer(4)(3)(3)#421 - w(4)(1)(3)(3)#898
                                 - w(4)(2)(3)(3)#928 - w(4)(3)(3)(3)#958
                                 - w(4)(4)(3)(3)#988 - w(4)(5)(3)(3)#1018
                                 - w(4)(6)(3)(3)#1048  = -15
 CustomerInventoryBalance(5,3,3): - I_customer(5)(2)(3)#141
                                 + I_customer(5)(3)(3)#151
                                 - b_customer(5)(3)(3)#451 - w(5)(1)(3)(3)#1078
                                 - w(5)(2)(3)(3)#1108 - w(5)(3)(3)(3)#1138
                                 - w(5)(4)(3)(3)#1168 - w(5)(5)(3)(3)#1198
                                 - w(5)(6)(3)(3)#1228  = -13
 CustomerInventoryBalance(6,3,3): - I_customer(6)(2)(3)#171
                                 + I_customer(6)(3)(3)#181
                                 - b_customer(6)(3)(3)#481 - w(6)(1)(3)(3)#1258
                                 - w(6)(2)(3)(3)#1288 - w(6)(3)(3)(3)#1318
                                 - w(6)(4)(3)(3)#1348 - w(6)(5)(3)(3)#1378
                                 - w(6)(6)(3)(3)#1408  = -7
 CustomerInventoryBalance(7,3,3): - I_customer(7)(2)(3)#201
                                 + I_customer(7)(3)(3)#211
                                 - b_customer(7)(3)(3)#511 - w(7)(1)(3)(3)#1438
                                 - w(7)(2)(3)(3)#1468 - w(7)(3)(3)(3)#1498
                                 - w(7)(4)(3)(3)#1528 - w(7)(5)(3)(3)#1558
                                 - w(7)(6)(3)(3)#1588  = -13
 CustomerInventoryBalance(8,3,3): - I_customer(8)(2)(3)#231
                                 + I_customer(8)(3)(3)#241
                                 - b_customer(8)(3)(3)#541 - w(8)(1)(3)(3)#1618
                                 - w(8)(2)(3)(3)#1648 - w(8)(3)(3)(3)#1678
                                 - w(8)(4)(3)(3)#1708 - w(8)(5)(3)(3)#1738
                                 - w(8)(6)(3)(3)#1768  = -19
 CustomerInventoryBalance(9,3,3): - I_customer(9)(2)(3)#261
                                 + I_customer(9)(3)(3)#271
                                 - b_customer(9)(3)(3)#571 - w(9)(1)(3)(3)#1798
                                 - w(9)(2)(3)(3)#1828 - w(9)(3)(3)(3)#1858
                                 - w(9)(4)(3)(3)#1888 - w(9)(5)(3)(3)#1918
                                 - w(9)(6)(3)(3)#1948  = -26
 CustomerInventoryBalance(10,3,3): - I_customer(10)(2)(3)#291
                                 + I_customer(10)(3)(3)#301
                                 - b_customer(10)(3)(3)#601
                                 - w(10)(1)(3)(3)#1978 - w(10)(2)(3)(3)#2008
                                 - w(10)(3)(3)(3)#2038 - w(10)(4)(3)(3)#2068
                                 - w(10)(5)(3)(3)#2098 - w(10)(6)(3)(3)#2128
                                  = -15
 CustomerInventoryBalance(11,3,3): - I_customer(11)(2)(3)#321
                                 + I_customer(11)(3)(3)#331
                                 - b_customer(11)(3)(3)#631
                                 - w(11)(1)(3)(3)#2158 - w(11)(2)(3)(3)#2188
                                 - w(11)(3)(3)(3)#2218 - w(11)(4)(3)(3)#2248
                                 - w(11)(5)(3)(3)#2278 - w(11)(6)(3)(3)#2308
                                  = -20
 CustomerInventoryBalance(12,3,3): - I_customer(12)(2)(3)#351
                                 + I_customer(12)(3)(3)#361
                                 - b_customer(12)(3)(3)#661
                                 - w(12)(1)(3)(3)#2338 - w(12)(2)(3)(3)#2368
                                 - w(12)(3)(3)(3)#2398 - w(12)(4)(3)(3)#2428
                                 - w(12)(5)(3)(3)#2458 - w(12)(6)(3)(3)#2488
                                  = -21
 CustomerInventoryBalance(3,1,4): I_customer(3)(1)(4)#72
                                 - b_customer(3)(1)(4)#372 - w(3)(1)(1)(4)#699
                                 - w(3)(2)(1)(4)#729 - w(3)(3)(1)(4)#759
                                 - w(3)(4)(1)(4)#789 - w(3)(5)(1)(4)#819
                                 - w(3)(6)(1)(4)#849  = -6
 CustomerInventoryBalance(4,1,4): I_customer(4)(1)(4)#102
                                 - b_customer(4)(1)(4)#402 - w(4)(1)(1)(4)#879
                                 - w(4)(2)(1)(4)#909 - w(4)(3)(1)(4)#939
                                 - w(4)(4)(1)(4)#969 - w(4)(5)(1)(4)#999
                                 - w(4)(6)(1)(4)#1029  = -3
 CustomerInventoryBalance(5,1,4): I_customer(5)(1)(4)#132
                                 - b_customer(5)(1)(4)#432 - w(5)(1)(1)(4)#1059
                                 - w(5)(2)(1)(4)#1089 - w(5)(3)(1)(4)#1119
                                 - w(5)(4)(1)(4)#1149 - w(5)(5)(1)(4)#1179
                                 - w(5)(6)(1)(4)#1209  = -2
 CustomerInventoryBalance(6,1,4): I_customer(6)(1)(4)#162
                                 - b_customer(6)(1)(4)#462 - w(6)(1)(1)(4)#1239
                                 - w(6)(2)(1)(4)#1269 - w(6)(3)(1)(4)#1299
                                 - w(6)(4)(1)(4)#1329 - w(6)(5)(1)(4)#1359
                                 - w(6)(6)(1)(4)#1389  = -4
 CustomerInventoryBalance(7,1,4): I_customer(7)(1)(4)#192
                                 - b_customer(7)(1)(4)#492 - w(7)(1)(1)(4)#1419
                                 - w(7)(2)(1)(4)#1449 - w(7)(3)(1)(4)#1479
                                 - w(7)(4)(1)(4)#1509 - w(7)(5)(1)(4)#1539
                                 - w(7)(6)(1)(4)#1569  = -1
 CustomerInventoryBalance(8,1,4): I_customer(8)(1)(4)#222
                                 - b_customer(8)(1)(4)#522 - w(8)(1)(1)(4)#1599
                                 - w(8)(2)(1)(4)#1629 - w(8)(3)(1)(4)#1659
                                 - w(8)(4)(1)(4)#1689 - w(8)(5)(1)(4)#1719
                                 - w(8)(6)(1)(4)#1749  = 26
 CustomerInventoryBalance(9,1,4): I_customer(9)(1)(4)#252
                                 - b_customer(9)(1)(4)#552 - w(9)(1)(1)(4)#1779
                                 - w(9)(2)(1)(4)#1809 - w(9)(3)(1)(4)#1839
                                 - w(9)(4)(1)(4)#1869 - w(9)(5)(1)(4)#1899
                                 - w(9)(6)(1)(4)#1929  = 33
 CustomerInventoryBalance(10,1,4): I_customer(10)(1)(4)#282
                                 - b_customer(10)(1)(4)#582
                                 - w(10)(1)(1)(4)#1959 - w(10)(2)(1)(4)#1989
                                 - w(10)(3)(1)(4)#2019 - w(10)(4)(1)(4)#2049
                                 - w(10)(5)(1)(4)#2079 - w(10)(6)(1)(4)#2109
                                  = -9
 CustomerInventoryBalance(11,1,4): I_customer(11)(1)(4)#312
                                 - b_customer(11)(1)(4)#612
                                 - w(11)(1)(1)(4)#2139 - w(11)(2)(1)(4)#2169
                                 - w(11)(3)(1)(4)#2199 - w(11)(4)(1)(4)#2229
                                 - w(11)(5)(1)(4)#2259 - w(11)(6)(1)(4)#2289
                                  = 29
 CustomerInventoryBalance(12,1,4): I_customer(12)(1)(4)#342
                                 - b_customer(12)(1)(4)#642
                                 - w(12)(1)(1)(4)#2319 - w(12)(2)(1)(4)#2349
                                 - w(12)(3)(1)(4)#2379 - w(12)(4)(1)(4)#2409
                                 - w(12)(5)(1)(4)#2439 - w(12)(6)(1)(4)#2469
                                  = 20
 CustomerInventoryBalance(3,2,4): - I_customer(3)(1)(4)#72
                                 + I_customer(3)(2)(4)#82
                                 - b_customer(3)(2)(4)#382 - w(3)(1)(2)(4)#709
                                 - w(3)(2)(2)(4)#739 - w(3)(3)(2)(4)#769
                                 - w(3)(4)(2)(4)#799 - w(3)(5)(2)(4)#829
                                 - w(3)(6)(2)(4)#859  = -11
 CustomerInventoryBalance(4,2,4): - I_customer(4)(1)(4)#102
                                 + I_customer(4)(2)(4)#112
                                 - b_customer(4)(2)(4)#412 - w(4)(1)(2)(4)#889
                                 - w(4)(2)(2)(4)#919 - w(4)(3)(2)(4)#949
                                 - w(4)(4)(2)(4)#979 - w(4)(5)(2)(4)#1009
                                 - w(4)(6)(2)(4)#1039  = -15
 CustomerInventoryBalance(5,2,4): - I_customer(5)(1)(4)#132
                                 + I_customer(5)(2)(4)#142
                                 - b_customer(5)(2)(4)#442 - w(5)(1)(2)(4)#1069
                                 - w(5)(2)(2)(4)#1099 - w(5)(3)(2)(4)#1129
                                 - w(5)(4)(2)(4)#1159 - w(5)(5)(2)(4)#1189
                                 - w(5)(6)(2)(4)#1219  = -14
 CustomerInventoryBalance(6,2,4): - I_customer(6)(1)(4)#162
                                 + I_customer(6)(2)(4)#172
                                 - b_customer(6)(2)(4)#472 - w(6)(1)(2)(4)#1249
                                 - w(6)(2)(2)(4)#1279 - w(6)(3)(2)(4)#1309
                                 - w(6)(4)(2)(4)#1339 - w(6)(5)(2)(4)#1369
                                 - w(6)(6)(2)(4)#1399  = -6
 CustomerInventoryBalance(7,2,4): - I_customer(7)(1)(4)#192
                                 + I_customer(7)(2)(4)#202
                                 - b_customer(7)(2)(4)#502 - w(7)(1)(2)(4)#1429
                                 - w(7)(2)(2)(4)#1459 - w(7)(3)(2)(4)#1489
                                 - w(7)(4)(2)(4)#1519 - w(7)(5)(2)(4)#1549
                                 - w(7)(6)(2)(4)#1579  = -11
 CustomerInventoryBalance(8,2,4): - I_customer(8)(1)(4)#222
                                 + I_customer(8)(2)(4)#232
                                 - b_customer(8)(2)(4)#532 - w(8)(1)(2)(4)#1609
                                 - w(8)(2)(2)(4)#1639 - w(8)(3)(2)(4)#1669
                                 - w(8)(4)(2)(4)#1699 - w(8)(5)(2)(4)#1729
                                 - w(8)(6)(2)(4)#1759  = -15
 CustomerInventoryBalance(9,2,4): - I_customer(9)(1)(4)#252
                                 + I_customer(9)(2)(4)#262
                                 - b_customer(9)(2)(4)#562 - w(9)(1)(2)(4)#1789
                                 - w(9)(2)(2)(4)#1819 - w(9)(3)(2)(4)#1849
                                 - w(9)(4)(2)(4)#1879 - w(9)(5)(2)(4)#1909
                                 - w(9)(6)(2)(4)#1939  = -24
 CustomerInventoryBalance(10,2,4): - I_customer(10)(1)(4)#282
                                 + I_customer(10)(2)(4)#292
                                 - b_customer(10)(2)(4)#592
                                 - w(10)(1)(2)(4)#1969 - w(10)(2)(2)(4)#1999
                                 - w(10)(3)(2)(4)#2029 - w(10)(4)(2)(4)#2059
                                 - w(10)(5)(2)(4)#2089 - w(10)(6)(2)(4)#2119
                                  = -16
 CustomerInventoryBalance(11,2,4): - I_customer(11)(1)(4)#312
                                 + I_customer(11)(2)(4)#322
                                 - b_customer(11)(2)(4)#622
                                 - w(11)(1)(2)(4)#2149 - w(11)(2)(2)(4)#2179
                                 - w(11)(3)(2)(4)#2209 - w(11)(4)(2)(4)#2239
                                 - w(11)(5)(2)(4)#2269 - w(11)(6)(2)(4)#2299
                                  = -22
 CustomerInventoryBalance(12,2,4): - I_customer(12)(1)(4)#342
                                 + I_customer(12)(2)(4)#352
                                 - b_customer(12)(2)(4)#652
                                 - w(12)(1)(2)(4)#2329 - w(12)(2)(2)(4)#2359
                                 - w(12)(3)(2)(4)#2389 - w(12)(4)(2)(4)#2419
                                 - w(12)(5)(2)(4)#2449 - w(12)(6)(2)(4)#2479
                                  = -23
 CustomerInventoryBalance(3,3,4): - I_customer(3)(2)(4)#82
                                 + I_customer(3)(3)(4)#92
                                 - b_customer(3)(3)(4)#392 - w(3)(1)(3)(4)#719
                                 - w(3)(2)(3)(4)#749 - w(3)(3)(3)(4)#779
                                 - w(3)(4)(3)(4)#809 - w(3)(5)(3)(4)#839
                                 - w(3)(6)(3)(4)#869  = -11
 CustomerInventoryBalance(4,3,4): - I_customer(4)(2)(4)#112
                                 + I_customer(4)(3)(4)#122
                                 - b_customer(4)(3)(4)#422 - w(4)(1)(3)(4)#899
                                 - w(4)(2)(3)(4)#929 - w(4)(3)(3)(4)#959
                                 - w(4)(4)(3)(4)#989 - w(4)(5)(3)(4)#1019
                                 - w(4)(6)(3)(4)#1049  = -13
 CustomerInventoryBalance(5,3,4): - I_customer(5)(2)(4)#142
                                 + I_customer(5)(3)(4)#152
                                 - b_customer(5)(3)(4)#452 - w(5)(1)(3)(4)#1079
                                 - w(5)(2)(3)(4)#1109 - w(5)(3)(3)(4)#1139
                                 - w(5)(4)(3)(4)#1169 - w(5)(5)(3)(4)#1199
                                 - w(5)(6)(3)(4)#1229  = -16
 CustomerInventoryBalance(6,3,4): - I_customer(6)(2)(4)#172
                                 + I_customer(6)(3)(4)#182
                                 - b_customer(6)(3)(4)#482 - w(6)(1)(3)(4)#1259
                                 - w(6)(2)(3)(4)#1289 - w(6)(3)(3)(4)#1319
                                 - w(6)(4)(3)(4)#1349 - w(6)(5)(3)(4)#1379
                                 - w(6)(6)(3)(4)#1409  = -6
 CustomerInventoryBalance(7,3,4): - I_customer(7)(2)(4)#202
                                 + I_customer(7)(3)(4)#212
                                 - b_customer(7)(3)(4)#512 - w(7)(1)(3)(4)#1439
                                 - w(7)(2)(3)(4)#1469 - w(7)(3)(3)(4)#1499
                                 - w(7)(4)(3)(4)#1529 - w(7)(5)(3)(4)#1559
                                 - w(7)(6)(3)(4)#1589  = -11
 CustomerInventoryBalance(8,3,4): - I_customer(8)(2)(4)#232
                                 + I_customer(8)(3)(4)#242
                                 - b_customer(8)(3)(4)#542 - w(8)(1)(3)(4)#1619
                                 - w(8)(2)(3)(4)#1649 - w(8)(3)(3)(4)#1679
                                 - w(8)(4)(3)(4)#1709 - w(8)(5)(3)(4)#1739
                                 - w(8)(6)(3)(4)#1769  = -15
 CustomerInventoryBalance(9,3,4): - I_customer(9)(2)(4)#262
                                 + I_customer(9)(3)(4)#272
                                 - b_customer(9)(3)(4)#572 - w(9)(1)(3)(4)#1799
                                 - w(9)(2)(3)(4)#1829 - w(9)(3)(3)(4)#1859
                                 - w(9)(4)(3)(4)#1889 - w(9)(5)(3)(4)#1919
                                 - w(9)(6)(3)(4)#1949  = -23
 CustomerInventoryBalance(10,3,4): - I_customer(10)(2)(4)#292
                                 + I_customer(10)(3)(4)#302
                                 - b_customer(10)(3)(4)#602
                                 - w(10)(1)(3)(4)#1979 - w(10)(2)(3)(4)#2009
                                 - w(10)(3)(3)(4)#2039 - w(10)(4)(3)(4)#2069
                                 - w(10)(5)(3)(4)#2099 - w(10)(6)(3)(4)#2129
                                  = -15
 CustomerInventoryBalance(11,3,4): - I_customer(11)(2)(4)#322
                                 + I_customer(11)(3)(4)#332
                                 - b_customer(11)(3)(4)#632
                                 - w(11)(1)(3)(4)#2159 - w(11)(2)(3)(4)#2189
                                 - w(11)(3)(3)(4)#2219 - w(11)(4)(3)(4)#2249
                                 - w(11)(5)(3)(4)#2279 - w(11)(6)(3)(4)#2309
                                  = -16
 CustomerInventoryBalance(12,3,4): - I_customer(12)(2)(4)#352
                                 + I_customer(12)(3)(4)#362
                                 - b_customer(12)(3)(4)#662
                                 - w(12)(1)(3)(4)#2339 - w(12)(2)(3)(4)#2369
                                 - w(12)(3)(3)(4)#2399 - w(12)(4)(3)(4)#2429
                                 - w(12)(5)(3)(4)#2459 - w(12)(6)(3)(4)#2489
                                  = -19
 CustomerInventoryBalance(3,1,5): I_customer(3)(1)(5)#73
                                 - b_customer(3)(1)(5)#373 - w(3)(1)(1)(5)#700
                                 - w(3)(2)(1)(5)#730 - w(3)(3)(1)(5)#760
                                 - w(3)(4)(1)(5)#790 - w(3)(5)(1)(5)#820
                                 - w(3)(6)(1)(5)#850  = -4
 CustomerInventoryBalance(4,1,5): I_customer(4)(1)(5)#103
                                 - b_customer(4)(1)(5)#403 - w(4)(1)(1)(5)#880
                                 - w(4)(2)(1)(5)#910 - w(4)(3)(1)(5)#940
                                 - w(4)(4)(1)(5)#970 - w(4)(5)(1)(5)#1000
                                 - w(4)(6)(1)(5)#1030  = -2
 CustomerInventoryBalance(5,1,5): I_customer(5)(1)(5)#133
                                 - b_customer(5)(1)(5)#433 - w(5)(1)(1)(5)#1060
                                 - w(5)(2)(1)(5)#1090 - w(5)(3)(1)(5)#1120
                                 - w(5)(4)(1)(5)#1150 - w(5)(5)(1)(5)#1180
                                 - w(5)(6)(1)(5)#1210  = -2
 CustomerInventoryBalance(6,1,5): I_customer(6)(1)(5)#163
                                 - b_customer(6)(1)(5)#463 - w(6)(1)(1)(5)#1240
                                 - w(6)(2)(1)(5)#1270 - w(6)(3)(1)(5)#1300
                                 - w(6)(4)(1)(5)#1330 - w(6)(5)(1)(5)#1360
                                 - w(6)(6)(1)(5)#1390  = -5
 CustomerInventoryBalance(7,1,5): I_customer(7)(1)(5)#193
                                 - b_customer(7)(1)(5)#493 - w(7)(1)(1)(5)#1420
                                 - w(7)(2)(1)(5)#1450 - w(7)(3)(1)(5)#1480
                                 - w(7)(4)(1)(5)#1510 - w(7)(5)(1)(5)#1540
                                 - w(7)(6)(1)(5)#1570  = -1
 CustomerInventoryBalance(8,1,5): I_customer(8)(1)(5)#223
                                 - b_customer(8)(1)(5)#523 - w(8)(1)(1)(5)#1600
                                 - w(8)(2)(1)(5)#1630 - w(8)(3)(1)(5)#1660
                                 - w(8)(4)(1)(5)#1690 - w(8)(5)(1)(5)#1720
                                 - w(8)(6)(1)(5)#1750  = 21
 CustomerInventoryBalance(9,1,5): I_customer(9)(1)(5)#253
                                 - b_customer(9)(1)(5)#553 - w(9)(1)(1)(5)#1780
                                 - w(9)(2)(1)(5)#1810 - w(9)(3)(1)(5)#1840
                                 - w(9)(4)(1)(5)#1870 - w(9)(5)(1)(5)#1900
                                 - w(9)(6)(1)(5)#1930  = 29
 CustomerInventoryBalance(10,1,5): I_customer(10)(1)(5)#283
                                 - b_customer(10)(1)(5)#583
                                 - w(10)(1)(1)(5)#1960 - w(10)(2)(1)(5)#1990
                                 - w(10)(3)(1)(5)#2020 - w(10)(4)(1)(5)#2050
                                 - w(10)(5)(1)(5)#2080 - w(10)(6)(1)(5)#2110
                                  = -5
 CustomerInventoryBalance(11,1,5): I_customer(11)(1)(5)#313
                                 - b_customer(11)(1)(5)#613
                                 - w(11)(1)(1)(5)#2140 - w(11)(2)(1)(5)#2170
                                 - w(11)(3)(1)(5)#2200 - w(11)(4)(1)(5)#2230
                                 - w(11)(5)(1)(5)#2260 - w(11)(6)(1)(5)#2290
                                  = 25
 CustomerInventoryBalance(12,1,5): I_customer(12)(1)(5)#343
                                 - b_customer(12)(1)(5)#643
                                 - w(12)(1)(1)(5)#2320 - w(12)(2)(1)(5)#2350
                                 - w(12)(3)(1)(5)#2380 - w(12)(4)(1)(5)#2410
                                 - w(12)(5)(1)(5)#2440 - w(12)(6)(1)(5)#2470
                                  = 22
 CustomerInventoryBalance(3,2,5): - I_customer(3)(1)(5)#73
                                 + I_customer(3)(2)(5)#83
                                 - b_customer(3)(2)(5)#383 - w(3)(1)(2)(5)#710
                                 - w(3)(2)(2)(5)#740 - w(3)(3)(2)(5)#770
                                 - w(3)(4)(2)(5)#800 - w(3)(5)(2)(5)#830
                                 - w(3)(6)(2)(5)#860  = -12
 CustomerInventoryBalance(4,2,5): - I_customer(4)(1)(5)#103
                                 + I_customer(4)(2)(5)#113
                                 - b_customer(4)(2)(5)#413 - w(4)(1)(2)(5)#890
                                 - w(4)(2)(2)(5)#920 - w(4)(3)(2)(5)#950
                                 - w(4)(4)(2)(5)#980 - w(4)(5)(2)(5)#1010
                                 - w(4)(6)(2)(5)#1040  = -17
 CustomerInventoryBalance(5,2,5): - I_customer(5)(1)(5)#133
                                 + I_customer(5)(2)(5)#143
                                 - b_customer(5)(2)(5)#443 - w(5)(1)(2)(5)#1070
                                 - w(5)(2)(2)(5)#1100 - w(5)(3)(2)(5)#1130
                                 - w(5)(4)(2)(5)#1160 - w(5)(5)(2)(5)#1190
                                 - w(5)(6)(2)(5)#1220  = -12
 CustomerInventoryBalance(6,2,5): - I_customer(6)(1)(5)#163
                                 + I_customer(6)(2)(5)#173
                                 - b_customer(6)(2)(5)#473 - w(6)(1)(2)(5)#1250
                                 - w(6)(2)(2)(5)#1280 - w(6)(3)(2)(5)#1310
                                 - w(6)(4)(2)(5)#1340 - w(6)(5)(2)(5)#1370
                                 - w(6)(6)(2)(5)#1400  = -7
 CustomerInventoryBalance(7,2,5): - I_customer(7)(1)(5)#193
                                 + I_customer(7)(2)(5)#203
                                 - b_customer(7)(2)(5)#503 - w(7)(1)(2)(5)#1430
                                 - w(7)(2)(2)(5)#1460 - w(7)(3)(2)(5)#1490
                                 - w(7)(4)(2)(5)#1520 - w(7)(5)(2)(5)#1550
                                 - w(7)(6)(2)(5)#1580  = -12
 CustomerInventoryBalance(8,2,5): - I_customer(8)(1)(5)#223
                                 + I_customer(8)(2)(5)#233
                                 - b_customer(8)(2)(5)#533 - w(8)(1)(2)(5)#1610
                                 - w(8)(2)(2)(5)#1640 - w(8)(3)(2)(5)#1670
                                 - w(8)(4)(2)(5)#1700 - w(8)(5)(2)(5)#1730
                                 - w(8)(6)(2)(5)#1760  = -14
 CustomerInventoryBalance(9,2,5): - I_customer(9)(1)(5)#253
                                 + I_customer(9)(2)(5)#263
                                 - b_customer(9)(2)(5)#563 - w(9)(1)(2)(5)#1790
                                 - w(9)(2)(2)(5)#1820 - w(9)(3)(2)(5)#1850
                                 - w(9)(4)(2)(5)#1880 - w(9)(5)(2)(5)#1910
                                 - w(9)(6)(2)(5)#1940  = -26
 CustomerInventoryBalance(10,2,5): - I_customer(10)(1)(5)#283
                                 + I_customer(10)(2)(5)#293
                                 - b_customer(10)(2)(5)#593
                                 - w(10)(1)(2)(5)#1970 - w(10)(2)(2)(5)#2000
                                 - w(10)(3)(2)(5)#2030 - w(10)(4)(2)(5)#2060
                                 - w(10)(5)(2)(5)#2090 - w(10)(6)(2)(5)#2120
                                  = -13
 CustomerInventoryBalance(11,2,5): - I_customer(11)(1)(5)#313
                                 + I_customer(11)(2)(5)#323
                                 - b_customer(11)(2)(5)#623
                                 - w(11)(1)(2)(5)#2150 - w(11)(2)(2)(5)#2180
                                 - w(11)(3)(2)(5)#2210 - w(11)(4)(2)(5)#2240
                                 - w(11)(5)(2)(5)#2270 - w(11)(6)(2)(5)#2300
                                  = -21
 CustomerInventoryBalance(12,2,5): - I_customer(12)(1)(5)#343
                                 + I_customer(12)(2)(5)#353
                                 - b_customer(12)(2)(5)#653
                                 - w(12)(1)(2)(5)#2330 - w(12)(2)(2)(5)#2360
                                 - w(12)(3)(2)(5)#2390 - w(12)(4)(2)(5)#2420
                                 - w(12)(5)(2)(5)#2450 - w(12)(6)(2)(5)#2480
                                  = -19
 CustomerInventoryBalance(3,3,5): - I_customer(3)(2)(5)#83
                                 + I_customer(3)(3)(5)#93
                                 - b_customer(3)(3)(5)#393 - w(3)(1)(3)(5)#720
                                 - w(3)(2)(3)(5)#750 - w(3)(3)(3)(5)#780
                                 - w(3)(4)(3)(5)#810 - w(3)(5)(3)(5)#840
                                 - w(3)(6)(3)(5)#870  = -9
 CustomerInventoryBalance(4,3,5): - I_customer(4)(2)(5)#113
                                 + I_customer(4)(3)(5)#123
                                 - b_customer(4)(3)(5)#423 - w(4)(1)(3)(5)#900
                                 - w(4)(2)(3)(5)#930 - w(4)(3)(3)(5)#960
                                 - w(4)(4)(3)(5)#990 - w(4)(5)(3)(5)#1020
                                 - w(4)(6)(3)(5)#1050  = -14
 CustomerInventoryBalance(5,3,5): - I_customer(5)(2)(5)#143
                                 + I_customer(5)(3)(5)#153
                                 - b_customer(5)(3)(5)#453 - w(5)(1)(3)(5)#1080
                                 - w(5)(2)(3)(5)#1110 - w(5)(3)(3)(5)#1140
                                 - w(5)(4)(3)(5)#1170 - w(5)(5)(3)(5)#1200
                                 - w(5)(6)(3)(5)#1230  = -14
 CustomerInventoryBalance(6,3,5): - I_customer(6)(2)(5)#173
                                 + I_customer(6)(3)(5)#183
                                 - b_customer(6)(3)(5)#483 - w(6)(1)(3)(5)#1260
                                 - w(6)(2)(3)(5)#1290 - w(6)(3)(3)(5)#1320
                                 - w(6)(4)(3)(5)#1350 - w(6)(5)(3)(5)#1380
                                 - w(6)(6)(3)(5)#1410  = -8
 CustomerInventoryBalance(7,3,5): - I_customer(7)(2)(5)#203
                                 + I_customer(7)(3)(5)#213
                                 - b_customer(7)(3)(5)#513 - w(7)(1)(3)(5)#1440
                                 - w(7)(2)(3)(5)#1470 - w(7)(3)(3)(5)#1500
                                 - w(7)(4)(3)(5)#1530 - w(7)(5)(3)(5)#1560
                                 - w(7)(6)(3)(5)#1590  = -12
 CustomerInventoryBalance(8,3,5): - I_customer(8)(2)(5)#233
                                 + I_customer(8)(3)(5)#243
                                 - b_customer(8)(3)(5)#543 - w(8)(1)(3)(5)#1620
                                 - w(8)(2)(3)(5)#1650 - w(8)(3)(3)(5)#1680
                                 - w(8)(4)(3)(5)#1710 - w(8)(5)(3)(5)#1740
                                 - w(8)(6)(3)(5)#1770  = -14
 CustomerInventoryBalance(9,3,5): - I_customer(9)(2)(5)#263
                                 + I_customer(9)(3)(5)#273
                                 - b_customer(9)(3)(5)#573 - w(9)(1)(3)(5)#1800
                                 - w(9)(2)(3)(5)#1830 - w(9)(3)(3)(5)#1860
                                 - w(9)(4)(3)(5)#1890 - w(9)(5)(3)(5)#1920
                                 - w(9)(6)(3)(5)#1950  = -24
 CustomerInventoryBalance(10,3,5): - I_customer(10)(2)(5)#293
                                 + I_customer(10)(3)(5)#303
                                 - b_customer(10)(3)(5)#603
                                 - w(10)(1)(3)(5)#1980 - w(10)(2)(3)(5)#2010
                                 - w(10)(3)(3)(5)#2040 - w(10)(4)(3)(5)#2070
                                 - w(10)(5)(3)(5)#2100 - w(10)(6)(3)(5)#2130
                                  = -11
 CustomerInventoryBalance(11,3,5): - I_customer(11)(2)(5)#323
                                 + I_customer(11)(3)(5)#333
                                 - b_customer(11)(3)(5)#633
                                 - w(11)(1)(3)(5)#2160 - w(11)(2)(3)(5)#2190
                                 - w(11)(3)(3)(5)#2220 - w(11)(4)(3)(5)#2250
                                 - w(11)(5)(3)(5)#2280 - w(11)(6)(3)(5)#2310
                                  = -16
 CustomerInventoryBalance(12,3,5): - I_customer(12)(2)(5)#353
                                 + I_customer(12)(3)(5)#363
                                 - b_customer(12)(3)(5)#663
                                 - w(12)(1)(3)(5)#2340 - w(12)(2)(3)(5)#2370
                                 - w(12)(3)(3)(5)#2400 - w(12)(4)(3)(5)#2430
                                 - w(12)(5)(3)(5)#2460 - w(12)(6)(3)(5)#2490
                                  = -19
 CustomerInventoryBalance(3,1,6): I_customer(3)(1)(6)#74
                                 - b_customer(3)(1)(6)#374 - w(3)(1)(1)(6)#701
                                 - w(3)(2)(1)(6)#731 - w(3)(3)(1)(6)#761
                                 - w(3)(4)(1)(6)#791 - w(3)(5)(1)(6)#821
                                 - w(3)(6)(1)(6)#851  = -7
 CustomerInventoryBalance(4,1,6): I_customer(4)(1)(6)#104
                                 - b_customer(4)(1)(6)#404 - w(4)(1)(1)(6)#881
                                 - w(4)(2)(1)(6)#911 - w(4)(3)(1)(6)#941
                                 - w(4)(4)(1)(6)#971 - w(4)(5)(1)(6)#1001
                                 - w(4)(6)(1)(6)#1031  = 2
 CustomerInventoryBalance(5,1,6): I_customer(5)(1)(6)#134
                                 - b_customer(5)(1)(6)#434 - w(5)(1)(1)(6)#1061
                                 - w(5)(2)(1)(6)#1091 - w(5)(3)(1)(6)#1121
                                 - w(5)(4)(1)(6)#1151 - w(5)(5)(1)(6)#1181
                                 - w(5)(6)(1)(6)#1211  = 1
 CustomerInventoryBalance(6,1,6): I_customer(6)(1)(6)#164
                                 - b_customer(6)(1)(6)#464 - w(6)(1)(1)(6)#1241
                                 - w(6)(2)(1)(6)#1271 - w(6)(3)(1)(6)#1301
                                 - w(6)(4)(1)(6)#1331 - w(6)(5)(1)(6)#1361
                                 - w(6)(6)(1)(6)#1391  = -5
 CustomerInventoryBalance(7,1,6): I_customer(7)(1)(6)#194
                                 - b_customer(7)(1)(6)#494 - w(7)(1)(1)(6)#1421
                                 - w(7)(2)(1)(6)#1451 - w(7)(3)(1)(6)#1481
                                 - w(7)(4)(1)(6)#1511 - w(7)(5)(1)(6)#1541
                                 - w(7)(6)(1)(6)#1571  = -1
 CustomerInventoryBalance(8,1,6): I_customer(8)(1)(6)#224
                                 - b_customer(8)(1)(6)#524 - w(8)(1)(1)(6)#1601
                                 - w(8)(2)(1)(6)#1631 - w(8)(3)(1)(6)#1661
                                 - w(8)(4)(1)(6)#1691 - w(8)(5)(1)(6)#1721
                                 - w(8)(6)(1)(6)#1751  = 25
 CustomerInventoryBalance(9,1,6): I_customer(9)(1)(6)#254
                                 - b_customer(9)(1)(6)#554 - w(9)(1)(1)(6)#1781
                                 - w(9)(2)(1)(6)#1811 - w(9)(3)(1)(6)#1841
                                 - w(9)(4)(1)(6)#1871 - w(9)(5)(1)(6)#1901
                                 - w(9)(6)(1)(6)#1931  = 29
 CustomerInventoryBalance(10,1,6): I_customer(10)(1)(6)#284
                                 - b_customer(10)(1)(6)#584
                                 - w(10)(1)(1)(6)#1961 - w(10)(2)(1)(6)#1991
                                 - w(10)(3)(1)(6)#2021 - w(10)(4)(1)(6)#2051
                                 - w(10)(5)(1)(6)#2081 - w(10)(6)(1)(6)#2111
                                  = -7
 CustomerInventoryBalance(11,1,6): I_customer(11)(1)(6)#314
                                 - b_customer(11)(1)(6)#614
                                 - w(11)(1)(1)(6)#2141 - w(11)(2)(1)(6)#2171
                                 - w(11)(3)(1)(6)#2201 - w(11)(4)(1)(6)#2231
                                 - w(11)(5)(1)(6)#2261 - w(11)(6)(1)(6)#2291
                                  = 30
 CustomerInventoryBalance(12,1,6): I_customer(12)(1)(6)#344
                                 - b_customer(12)(1)(6)#644
                                 - w(12)(1)(1)(6)#2321 - w(12)(2)(1)(6)#2351
                                 - w(12)(3)(1)(6)#2381 - w(12)(4)(1)(6)#2411
                                 - w(12)(5)(1)(6)#2441 - w(12)(6)(1)(6)#2471
                                  = 24
 CustomerInventoryBalance(3,2,6): - I_customer(3)(1)(6)#74
                                 + I_customer(3)(2)(6)#84
                                 - b_customer(3)(2)(6)#384 - w(3)(1)(2)(6)#711
                                 - w(3)(2)(2)(6)#741 - w(3)(3)(2)(6)#771
                                 - w(3)(4)(2)(6)#801 - w(3)(5)(2)(6)#831
                                 - w(3)(6)(2)(6)#861  = -10
 CustomerInventoryBalance(4,2,6): - I_customer(4)(1)(6)#104
                                 + I_customer(4)(2)(6)#114
                                 - b_customer(4)(2)(6)#414 - w(4)(1)(2)(6)#891
                                 - w(4)(2)(2)(6)#921 - w(4)(3)(2)(6)#951
                                 - w(4)(4)(2)(6)#981 - w(4)(5)(2)(6)#1011
                                 - w(4)(6)(2)(6)#1041  = -13
 CustomerInventoryBalance(5,2,6): - I_customer(5)(1)(6)#134
                                 + I_customer(5)(2)(6)#144
                                 - b_customer(5)(2)(6)#444 - w(5)(1)(2)(6)#1071
                                 - w(5)(2)(2)(6)#1101 - w(5)(3)(2)(6)#1131
                                 - w(5)(4)(2)(6)#1161 - w(5)(5)(2)(6)#1191
                                 - w(5)(6)(2)(6)#1221  = -17
 CustomerInventoryBalance(6,2,6): - I_customer(6)(1)(6)#164
                                 + I_customer(6)(2)(6)#174
                                 - b_customer(6)(2)(6)#474 - w(6)(1)(2)(6)#1251
                                 - w(6)(2)(2)(6)#1281 - w(6)(3)(2)(6)#1311
                                 - w(6)(4)(2)(6)#1341 - w(6)(5)(2)(6)#1371
                                 - w(6)(6)(2)(6)#1401  = -6
 CustomerInventoryBalance(7,2,6): - I_customer(7)(1)(6)#194
                                 + I_customer(7)(2)(6)#204
                                 - b_customer(7)(2)(6)#504 - w(7)(1)(2)(6)#1431
                                 - w(7)(2)(2)(6)#1461 - w(7)(3)(2)(6)#1491
                                 - w(7)(4)(2)(6)#1521 - w(7)(5)(2)(6)#1551
                                 - w(7)(6)(2)(6)#1581  = -11
 CustomerInventoryBalance(8,2,6): - I_customer(8)(1)(6)#224
                                 + I_customer(8)(2)(6)#234
                                 - b_customer(8)(2)(6)#534 - w(8)(1)(2)(6)#1611
                                 - w(8)(2)(2)(6)#1641 - w(8)(3)(2)(6)#1671
                                 - w(8)(4)(2)(6)#1701 - w(8)(5)(2)(6)#1731
                                 - w(8)(6)(2)(6)#1761  = -19
 CustomerInventoryBalance(9,2,6): - I_customer(9)(1)(6)#254
                                 + I_customer(9)(2)(6)#264
                                 - b_customer(9)(2)(6)#564 - w(9)(1)(2)(6)#1791
                                 - w(9)(2)(2)(6)#1821 - w(9)(3)(2)(6)#1851
                                 - w(9)(4)(2)(6)#1881 - w(9)(5)(2)(6)#1911
                                 - w(9)(6)(2)(6)#1941  = -19
 CustomerInventoryBalance(10,2,6): - I_customer(10)(1)(6)#284
                                 + I_customer(10)(2)(6)#294
                                 - b_customer(10)(2)(6)#594
                                 - w(10)(1)(2)(6)#1971 - w(10)(2)(2)(6)#2001
                                 - w(10)(3)(2)(6)#2031 - w(10)(4)(2)(6)#2061
                                 - w(10)(5)(2)(6)#2091 - w(10)(6)(2)(6)#2121
                                  = -14
 CustomerInventoryBalance(11,2,6): - I_customer(11)(1)(6)#314
                                 + I_customer(11)(2)(6)#324
                                 - b_customer(11)(2)(6)#624
                                 - w(11)(1)(2)(6)#2151 - w(11)(2)(2)(6)#2181
                                 - w(11)(3)(2)(6)#2211 - w(11)(4)(2)(6)#2241
                                 - w(11)(5)(2)(6)#2271 - w(11)(6)(2)(6)#2301
                                  = -16
 CustomerInventoryBalance(12,2,6): - I_customer(12)(1)(6)#344
                                 + I_customer(12)(2)(6)#354
                                 - b_customer(12)(2)(6)#654
                                 - w(12)(1)(2)(6)#2331 - w(12)(2)(2)(6)#2361
                                 - w(12)(3)(2)(6)#2391 - w(12)(4)(2)(6)#2421
                                 - w(12)(5)(2)(6)#2451 - w(12)(6)(2)(6)#2481
                                  = -19
 CustomerInventoryBalance(3,3,6): - I_customer(3)(2)(6)#84
                                 + I_customer(3)(3)(6)#94
                                 - b_customer(3)(3)(6)#394 - w(3)(1)(3)(6)#721
                                 - w(3)(2)(3)(6)#751 - w(3)(3)(3)(6)#781
                                 - w(3)(4)(3)(6)#811 - w(3)(5)(3)(6)#841
                                 - w(3)(6)(3)(6)#871  = -9
 CustomerInventoryBalance(4,3,6): - I_customer(4)(2)(6)#114
                                 + I_customer(4)(3)(6)#124
                                 - b_customer(4)(3)(6)#424 - w(4)(1)(3)(6)#901
                                 - w(4)(2)(3)(6)#931 - w(4)(3)(3)(6)#961
                                 - w(4)(4)(3)(6)#991 - w(4)(5)(3)(6)#1021
                                 - w(4)(6)(3)(6)#1051  = -16
 CustomerInventoryBalance(5,3,6): - I_customer(5)(2)(6)#144
                                 + I_customer(5)(3)(6)#154
                                 - b_customer(5)(3)(6)#454 - w(5)(1)(3)(6)#1081
                                 - w(5)(2)(3)(6)#1111 - w(5)(3)(3)(6)#1141
                                 - w(5)(4)(3)(6)#1171 - w(5)(5)(3)(6)#1201
                                 - w(5)(6)(3)(6)#1231  = -13
 CustomerInventoryBalance(6,3,6): - I_customer(6)(2)(6)#174
                                 + I_customer(6)(3)(6)#184
                                 - b_customer(6)(3)(6)#484 - w(6)(1)(3)(6)#1261
                                 - w(6)(2)(3)(6)#1291 - w(6)(3)(3)(6)#1321
                                 - w(6)(4)(3)(6)#1351 - w(6)(5)(3)(6)#1381
                                 - w(6)(6)(3)(6)#1411  = -6
 CustomerInventoryBalance(7,3,6): - I_customer(7)(2)(6)#204
                                 + I_customer(7)(3)(6)#214
                                 - b_customer(7)(3)(6)#514 - w(7)(1)(3)(6)#1441
                                 - w(7)(2)(3)(6)#1471 - w(7)(3)(3)(6)#1501
                                 - w(7)(4)(3)(6)#1531 - w(7)(5)(3)(6)#1561
                                 - w(7)(6)(3)(6)#1591  = -14
 CustomerInventoryBalance(8,3,6): - I_customer(8)(2)(6)#234
                                 + I_customer(8)(3)(6)#244
                                 - b_customer(8)(3)(6)#544 - w(8)(1)(3)(6)#1621
                                 - w(8)(2)(3)(6)#1651 - w(8)(3)(3)(6)#1681
                                 - w(8)(4)(3)(6)#1711 - w(8)(5)(3)(6)#1741
                                 - w(8)(6)(3)(6)#1771  = -15
 CustomerInventoryBalance(9,3,6): - I_customer(9)(2)(6)#264
                                 + I_customer(9)(3)(6)#274
                                 - b_customer(9)(3)(6)#574 - w(9)(1)(3)(6)#1801
                                 - w(9)(2)(3)(6)#1831 - w(9)(3)(3)(6)#1861
                                 - w(9)(4)(3)(6)#1891 - w(9)(5)(3)(6)#1921
                                 - w(9)(6)(3)(6)#1951  = -26
 CustomerInventoryBalance(10,3,6): - I_customer(10)(2)(6)#294
                                 + I_customer(10)(3)(6)#304
                                 - b_customer(10)(3)(6)#604
                                 - w(10)(1)(3)(6)#1981 - w(10)(2)(3)(6)#2011
                                 - w(10)(3)(3)(6)#2041 - w(10)(4)(3)(6)#2071
                                 - w(10)(5)(3)(6)#2101 - w(10)(6)(3)(6)#2131
                                  = -14
 CustomerInventoryBalance(11,3,6): - I_customer(11)(2)(6)#324
                                 + I_customer(11)(3)(6)#334
                                 - b_customer(11)(3)(6)#634
                                 - w(11)(1)(3)(6)#2161 - w(11)(2)(3)(6)#2191
                                 - w(11)(3)(3)(6)#2221 - w(11)(4)(3)(6)#2251
                                 - w(11)(5)(3)(6)#2281 - w(11)(6)(3)(6)#2311
                                  = -16
 CustomerInventoryBalance(12,3,6): - I_customer(12)(2)(6)#354
                                 + I_customer(12)(3)(6)#364
                                 - b_customer(12)(3)(6)#664
                                 - w(12)(1)(3)(6)#2341 - w(12)(2)(3)(6)#2371
                                 - w(12)(3)(3)(6)#2401 - w(12)(4)(3)(6)#2431
                                 - w(12)(5)(3)(6)#2461 - w(12)(6)(3)(6)#2491
                                  = -26
 CustomerInventoryBalance(3,1,7): I_customer(3)(1)(7)#75
                                 - b_customer(3)(1)(7)#375 - w(3)(1)(1)(7)#702
                                 - w(3)(2)(1)(7)#732 - w(3)(3)(1)(7)#762
                                 - w(3)(4)(1)(7)#792 - w(3)(5)(1)(7)#822
                                 - w(3)(6)(1)(7)#852  = -7
 CustomerInventoryBalance(4,1,7): I_customer(4)(1)(7)#105
                                 - b_customer(4)(1)(7)#405 - w(4)(1)(1)(7)#882
                                 - w(4)(2)(1)(7)#912 - w(4)(3)(1)(7)#942
                                 - w(4)(4)(1)(7)#972 - w(4)(5)(1)(7)#1002
                                 - w(4)(6)(1)(7)#1032  = 2
 CustomerInventoryBalance(5,1,7): I_customer(5)(1)(7)#135
                                 - b_customer(5)(1)(7)#435 - w(5)(1)(1)(7)#1062
                                 - w(5)(2)(1)(7)#1092 - w(5)(3)(1)(7)#1122
                                 - w(5)(4)(1)(7)#1152 - w(5)(5)(1)(7)#1182
                                 - w(5)(6)(1)(7)#1212  = 0
 CustomerInventoryBalance(6,1,7): I_customer(6)(1)(7)#165
                                 - b_customer(6)(1)(7)#465 - w(6)(1)(1)(7)#1242
                                 - w(6)(2)(1)(7)#1272 - w(6)(3)(1)(7)#1302
                                 - w(6)(4)(1)(7)#1332 - w(6)(5)(1)(7)#1362
                                 - w(6)(6)(1)(7)#1392  = -5
 CustomerInventoryBalance(7,1,7): I_customer(7)(1)(7)#195
                                 - b_customer(7)(1)(7)#495 - w(7)(1)(1)(7)#1422
                                 - w(7)(2)(1)(7)#1452 - w(7)(3)(1)(7)#1482
                                 - w(7)(4)(1)(7)#1512 - w(7)(5)(1)(7)#1542
                                 - w(7)(6)(1)(7)#1572  = 1
 CustomerInventoryBalance(8,1,7): I_customer(8)(1)(7)#225
                                 - b_customer(8)(1)(7)#525 - w(8)(1)(1)(7)#1602
                                 - w(8)(2)(1)(7)#1632 - w(8)(3)(1)(7)#1662
                                 - w(8)(4)(1)(7)#1692 - w(8)(5)(1)(7)#1722
                                 - w(8)(6)(1)(7)#1752  = 22
 CustomerInventoryBalance(9,1,7): I_customer(9)(1)(7)#255
                                 - b_customer(9)(1)(7)#555 - w(9)(1)(1)(7)#1782
                                 - w(9)(2)(1)(7)#1812 - w(9)(3)(1)(7)#1842
                                 - w(9)(4)(1)(7)#1872 - w(9)(5)(1)(7)#1902
                                 - w(9)(6)(1)(7)#1932  = 37
 CustomerInventoryBalance(10,1,7): I_customer(10)(1)(7)#285
                                 - b_customer(10)(1)(7)#585
                                 - w(10)(1)(1)(7)#1962 - w(10)(2)(1)(7)#1992
                                 - w(10)(3)(1)(7)#2022 - w(10)(4)(1)(7)#2052
                                 - w(10)(5)(1)(7)#2082 - w(10)(6)(1)(7)#2112
                                  = -7
 CustomerInventoryBalance(11,1,7): I_customer(11)(1)(7)#315
                                 - b_customer(11)(1)(7)#615
                                 - w(11)(1)(1)(7)#2142 - w(11)(2)(1)(7)#2172
                                 - w(11)(3)(1)(7)#2202 - w(11)(4)(1)(7)#2232
                                 - w(11)(5)(1)(7)#2262 - w(11)(6)(1)(7)#2292
                                  = 27
 CustomerInventoryBalance(12,1,7): I_customer(12)(1)(7)#345
                                 - b_customer(12)(1)(7)#645
                                 - w(12)(1)(1)(7)#2322 - w(12)(2)(1)(7)#2352
                                 - w(12)(3)(1)(7)#2382 - w(12)(4)(1)(7)#2412
                                 - w(12)(5)(1)(7)#2442 - w(12)(6)(1)(7)#2472
                                  = 24
 CustomerInventoryBalance(3,2,7): - I_customer(3)(1)(7)#75
                                 + I_customer(3)(2)(7)#85
                                 - b_customer(3)(2)(7)#385 - w(3)(1)(2)(7)#712
                                 - w(3)(2)(2)(7)#742 - w(3)(3)(2)(7)#772
                                 - w(3)(4)(2)(7)#802 - w(3)(5)(2)(7)#832
                                 - w(3)(6)(2)(7)#862  = -11
 CustomerInventoryBalance(4,2,7): - I_customer(4)(1)(7)#105
                                 + I_customer(4)(2)(7)#115
                                 - b_customer(4)(2)(7)#415 - w(4)(1)(2)(7)#892
                                 - w(4)(2)(2)(7)#922 - w(4)(3)(2)(7)#952
                                 - w(4)(4)(2)(7)#982 - w(4)(5)(2)(7)#1012
                                 - w(4)(6)(2)(7)#1042  = -17
 CustomerInventoryBalance(5,2,7): - I_customer(5)(1)(7)#135
                                 + I_customer(5)(2)(7)#145
                                 - b_customer(5)(2)(7)#445 - w(5)(1)(2)(7)#1072
                                 - w(5)(2)(2)(7)#1102 - w(5)(3)(2)(7)#1132
                                 - w(5)(4)(2)(7)#1162 - w(5)(5)(2)(7)#1192
                                 - w(5)(6)(2)(7)#1222  = -13
 CustomerInventoryBalance(6,2,7): - I_customer(6)(1)(7)#165
                                 + I_customer(6)(2)(7)#175
                                 - b_customer(6)(2)(7)#475 - w(6)(1)(2)(7)#1252
                                 - w(6)(2)(2)(7)#1282 - w(6)(3)(2)(7)#1312
                                 - w(6)(4)(2)(7)#1342 - w(6)(5)(2)(7)#1372
                                 - w(6)(6)(2)(7)#1402  = -6
 CustomerInventoryBalance(7,2,7): - I_customer(7)(1)(7)#195
                                 + I_customer(7)(2)(7)#205
                                 - b_customer(7)(2)(7)#505 - w(7)(1)(2)(7)#1432
                                 - w(7)(2)(2)(7)#1462 - w(7)(3)(2)(7)#1492
                                 - w(7)(4)(2)(7)#1522 - w(7)(5)(2)(7)#1552
                                 - w(7)(6)(2)(7)#1582  = -13
 CustomerInventoryBalance(8,2,7): - I_customer(8)(1)(7)#225
                                 + I_customer(8)(2)(7)#235
                                 - b_customer(8)(2)(7)#535 - w(8)(1)(2)(7)#1612
                                 - w(8)(2)(2)(7)#1642 - w(8)(3)(2)(7)#1672
                                 - w(8)(4)(2)(7)#1702 - w(8)(5)(2)(7)#1732
                                 - w(8)(6)(2)(7)#1762  = -18
 CustomerInventoryBalance(9,2,7): - I_customer(9)(1)(7)#255
                                 + I_customer(9)(2)(7)#265
                                 - b_customer(9)(2)(7)#565 - w(9)(1)(2)(7)#1792
                                 - w(9)(2)(2)(7)#1822 - w(9)(3)(2)(7)#1852
                                 - w(9)(4)(2)(7)#1882 - w(9)(5)(2)(7)#1912
                                 - w(9)(6)(2)(7)#1942  = -21
 CustomerInventoryBalance(10,2,7): - I_customer(10)(1)(7)#285
                                 + I_customer(10)(2)(7)#295
                                 - b_customer(10)(2)(7)#595
                                 - w(10)(1)(2)(7)#1972 - w(10)(2)(2)(7)#2002
                                 - w(10)(3)(2)(7)#2032 - w(10)(4)(2)(7)#2062
                                 - w(10)(5)(2)(7)#2092 - w(10)(6)(2)(7)#2122
                                  = -14
 CustomerInventoryBalance(11,2,7): - I_customer(11)(1)(7)#315
                                 + I_customer(11)(2)(7)#325
                                 - b_customer(11)(2)(7)#625
                                 - w(11)(1)(2)(7)#2152 - w(11)(2)(2)(7)#2182
                                 - w(11)(3)(2)(7)#2212 - w(11)(4)(2)(7)#2242
                                 - w(11)(5)(2)(7)#2272 - w(11)(6)(2)(7)#2302
                                  = -20
 CustomerInventoryBalance(12,2,7): - I_customer(12)(1)(7)#345
                                 + I_customer(12)(2)(7)#355
                                 - b_customer(12)(2)(7)#655
                                 - w(12)(1)(2)(7)#2332 - w(12)(2)(2)(7)#2362
                                 - w(12)(3)(2)(7)#2392 - w(12)(4)(2)(7)#2422
                                 - w(12)(5)(2)(7)#2452 - w(12)(6)(2)(7)#2482
                                  = -23
 CustomerInventoryBalance(3,3,7): - I_customer(3)(2)(7)#85
                                 + I_customer(3)(3)(7)#95
                                 - b_customer(3)(3)(7)#395 - w(3)(1)(3)(7)#722
                                 - w(3)(2)(3)(7)#752 - w(3)(3)(3)(7)#782
                                 - w(3)(4)(3)(7)#812 - w(3)(5)(3)(7)#842
                                 - w(3)(6)(3)(7)#872  = -10
 CustomerInventoryBalance(4,3,7): - I_customer(4)(2)(7)#115
                                 + I_customer(4)(3)(7)#125
                                 - b_customer(4)(3)(7)#425 - w(4)(1)(3)(7)#902
                                 - w(4)(2)(3)(7)#932 - w(4)(3)(3)(7)#962
                                 - w(4)(4)(3)(7)#992 - w(4)(5)(3)(7)#1022
                                 - w(4)(6)(3)(7)#1052  = -15
 CustomerInventoryBalance(5,3,7): - I_customer(5)(2)(7)#145
                                 + I_customer(5)(3)(7)#155
                                 - b_customer(5)(3)(7)#455 - w(5)(1)(3)(7)#1082
                                 - w(5)(2)(3)(7)#1112 - w(5)(3)(3)(7)#1142
                                 - w(5)(4)(3)(7)#1172 - w(5)(5)(3)(7)#1202
                                 - w(5)(6)(3)(7)#1232  = -12
 CustomerInventoryBalance(6,3,7): - I_customer(6)(2)(7)#175
                                 + I_customer(6)(3)(7)#185
                                 - b_customer(6)(3)(7)#485 - w(6)(1)(3)(7)#1262
                                 - w(6)(2)(3)(7)#1292 - w(6)(3)(3)(7)#1322
                                 - w(6)(4)(3)(7)#1352 - w(6)(5)(3)(7)#1382
                                 - w(6)(6)(3)(7)#1412  = -7
 CustomerInventoryBalance(7,3,7): - I_customer(7)(2)(7)#205
                                 + I_customer(7)(3)(7)#215
                                 - b_customer(7)(3)(7)#515 - w(7)(1)(3)(7)#1442
                                 - w(7)(2)(3)(7)#1472 - w(7)(3)(3)(7)#1502
                                 - w(7)(4)(3)(7)#1532 - w(7)(5)(3)(7)#1562
                                 - w(7)(6)(3)(7)#1592  = -14
 CustomerInventoryBalance(8,3,7): - I_customer(8)(2)(7)#235
                                 + I_customer(8)(3)(7)#245
                                 - b_customer(8)(3)(7)#545 - w(8)(1)(3)(7)#1622
                                 - w(8)(2)(3)(7)#1652 - w(8)(3)(3)(7)#1682
                                 - w(8)(4)(3)(7)#1712 - w(8)(5)(3)(7)#1742
                                 - w(8)(6)(3)(7)#1772  = -14
 CustomerInventoryBalance(9,3,7): - I_customer(9)(2)(7)#265
                                 + I_customer(9)(3)(7)#275
                                 - b_customer(9)(3)(7)#575 - w(9)(1)(3)(7)#1802
                                 - w(9)(2)(3)(7)#1832 - w(9)(3)(3)(7)#1862
                                 - w(9)(4)(3)(7)#1892 - w(9)(5)(3)(7)#1922
                                 - w(9)(6)(3)(7)#1952  = -26
 CustomerInventoryBalance(10,3,7): - I_customer(10)(2)(7)#295
                                 + I_customer(10)(3)(7)#305
                                 - b_customer(10)(3)(7)#605
                                 - w(10)(1)(3)(7)#1982 - w(10)(2)(3)(7)#2012
                                 - w(10)(3)(3)(7)#2042 - w(10)(4)(3)(7)#2072
                                 - w(10)(5)(3)(7)#2102 - w(10)(6)(3)(7)#2132
                                  = -15
 CustomerInventoryBalance(11,3,7): - I_customer(11)(2)(7)#325
                                 + I_customer(11)(3)(7)#335
                                 - b_customer(11)(3)(7)#635
                                 - w(11)(1)(3)(7)#2162 - w(11)(2)(3)(7)#2192
                                 - w(11)(3)(3)(7)#2222 - w(11)(4)(3)(7)#2252
                                 - w(11)(5)(3)(7)#2282 - w(11)(6)(3)(7)#2312
                                  = -21
 CustomerInventoryBalance(12,3,7): - I_customer(12)(2)(7)#355
                                 + I_customer(12)(3)(7)#365
                                 - b_customer(12)(3)(7)#665
                                 - w(12)(1)(3)(7)#2342 - w(12)(2)(3)(7)#2372
                                 - w(12)(3)(3)(7)#2402 - w(12)(4)(3)(7)#2432
                                 - w(12)(5)(3)(7)#2462 - w(12)(6)(3)(7)#2492
                                  = -25
 CustomerInventoryBalance(3,1,8): I_customer(3)(1)(8)#76
                                 - b_customer(3)(1)(8)#376 - w(3)(1)(1)(8)#703
                                 - w(3)(2)(1)(8)#733 - w(3)(3)(1)(8)#763
                                 - w(3)(4)(1)(8)#793 - w(3)(5)(1)(8)#823
                                 - w(3)(6)(1)(8)#853  = -5
 CustomerInventoryBalance(4,1,8): I_customer(4)(1)(8)#106
                                 - b_customer(4)(1)(8)#406 - w(4)(1)(1)(8)#883
                                 - w(4)(2)(1)(8)#913 - w(4)(3)(1)(8)#943
                                 - w(4)(4)(1)(8)#973 - w(4)(5)(1)(8)#1003
                                 - w(4)(6)(1)(8)#1033  = 2
 CustomerInventoryBalance(5,1,8): I_customer(5)(1)(8)#136
                                 - b_customer(5)(1)(8)#436 - w(5)(1)(1)(8)#1063
                                 - w(5)(2)(1)(8)#1093 - w(5)(3)(1)(8)#1123
                                 - w(5)(4)(1)(8)#1153 - w(5)(5)(1)(8)#1183
                                 - w(5)(6)(1)(8)#1213  = 2
 CustomerInventoryBalance(6,1,8): I_customer(6)(1)(8)#166
                                 - b_customer(6)(1)(8)#466 - w(6)(1)(1)(8)#1243
                                 - w(6)(2)(1)(8)#1273 - w(6)(3)(1)(8)#1303
                                 - w(6)(4)(1)(8)#1333 - w(6)(5)(1)(8)#1363
                                 - w(6)(6)(1)(8)#1393  = -4
 CustomerInventoryBalance(7,1,8): I_customer(7)(1)(8)#196
                                 - b_customer(7)(1)(8)#496 - w(7)(1)(1)(8)#1423
                                 - w(7)(2)(1)(8)#1453 - w(7)(3)(1)(8)#1483
                                 - w(7)(4)(1)(8)#1513 - w(7)(5)(1)(8)#1543
                                 - w(7)(6)(1)(8)#1573  = -2
 CustomerInventoryBalance(8,1,8): I_customer(8)(1)(8)#226
                                 - b_customer(8)(1)(8)#526 - w(8)(1)(1)(8)#1603
                                 - w(8)(2)(1)(8)#1633 - w(8)(3)(1)(8)#1663
                                 - w(8)(4)(1)(8)#1693 - w(8)(5)(1)(8)#1723
                                 - w(8)(6)(1)(8)#1753  = 27
 CustomerInventoryBalance(9,1,8): I_customer(9)(1)(8)#256
                                 - b_customer(9)(1)(8)#556 - w(9)(1)(1)(8)#1783
                                 - w(9)(2)(1)(8)#1813 - w(9)(3)(1)(8)#1843
                                 - w(9)(4)(1)(8)#1873 - w(9)(5)(1)(8)#1903
                                 - w(9)(6)(1)(8)#1933  = 34
 CustomerInventoryBalance(10,1,8): I_customer(10)(1)(8)#286
                                 - b_customer(10)(1)(8)#586
                                 - w(10)(1)(1)(8)#1963 - w(10)(2)(1)(8)#1993
                                 - w(10)(3)(1)(8)#2023 - w(10)(4)(1)(8)#2053
                                 - w(10)(5)(1)(8)#2083 - w(10)(6)(1)(8)#2113
                                  = -4
 CustomerInventoryBalance(11,1,8): I_customer(11)(1)(8)#316
                                 - b_customer(11)(1)(8)#616
                                 - w(11)(1)(1)(8)#2143 - w(11)(2)(1)(8)#2173
                                 - w(11)(3)(1)(8)#2203 - w(11)(4)(1)(8)#2233
                                 - w(11)(5)(1)(8)#2263 - w(11)(6)(1)(8)#2293
                                  = 25
 CustomerInventoryBalance(12,1,8): I_customer(12)(1)(8)#346
                                 - b_customer(12)(1)(8)#646
                                 - w(12)(1)(1)(8)#2323 - w(12)(2)(1)(8)#2353
                                 - w(12)(3)(1)(8)#2383 - w(12)(4)(1)(8)#2413
                                 - w(12)(5)(1)(8)#2443 - w(12)(6)(1)(8)#2473
                                  = 25
 CustomerInventoryBalance(3,2,8): - I_customer(3)(1)(8)#76
                                 + I_customer(3)(2)(8)#86
                                 - b_customer(3)(2)(8)#386 - w(3)(1)(2)(8)#713
                                 - w(3)(2)(2)(8)#743 - w(3)(3)(2)(8)#773
                                 - w(3)(4)(2)(8)#803 - w(3)(5)(2)(8)#833
                                 - w(3)(6)(2)(8)#863  = -10
 CustomerInventoryBalance(4,2,8): - I_customer(4)(1)(8)#106
                                 + I_customer(4)(2)(8)#116
                                 - b_customer(4)(2)(8)#416 - w(4)(1)(2)(8)#893
                                 - w(4)(2)(2)(8)#923 - w(4)(3)(2)(8)#953
                                 - w(4)(4)(2)(8)#983 - w(4)(5)(2)(8)#1013
                                 - w(4)(6)(2)(8)#1043  = -13
 CustomerInventoryBalance(5,2,8): - I_customer(5)(1)(8)#136
                                 + I_customer(5)(2)(8)#146
                                 - b_customer(5)(2)(8)#446 - w(5)(1)(2)(8)#1073
                                 - w(5)(2)(2)(8)#1103 - w(5)(3)(2)(8)#1133
                                 - w(5)(4)(2)(8)#1163 - w(5)(5)(2)(8)#1193
                                 - w(5)(6)(2)(8)#1223  = -13
 CustomerInventoryBalance(6,2,8): - I_customer(6)(1)(8)#166
                                 + I_customer(6)(2)(8)#176
                                 - b_customer(6)(2)(8)#476 - w(6)(1)(2)(8)#1253
                                 - w(6)(2)(2)(8)#1283 - w(6)(3)(2)(8)#1313
                                 - w(6)(4)(2)(8)#1343 - w(6)(5)(2)(8)#1373
                                 - w(6)(6)(2)(8)#1403  = -8
 CustomerInventoryBalance(7,2,8): - I_customer(7)(1)(8)#196
                                 + I_customer(7)(2)(8)#206
                                 - b_customer(7)(2)(8)#506 - w(7)(1)(2)(8)#1433
                                 - w(7)(2)(2)(8)#1463 - w(7)(3)(2)(8)#1493
                                 - w(7)(4)(2)(8)#1523 - w(7)(5)(2)(8)#1553
                                 - w(7)(6)(2)(8)#1583  = -13
 CustomerInventoryBalance(8,2,8): - I_customer(8)(1)(8)#226
                                 + I_customer(8)(2)(8)#236
                                 - b_customer(8)(2)(8)#536 - w(8)(1)(2)(8)#1613
                                 - w(8)(2)(2)(8)#1643 - w(8)(3)(2)(8)#1673
                                 - w(8)(4)(2)(8)#1703 - w(8)(5)(2)(8)#1733
                                 - w(8)(6)(2)(8)#1763  = -17
 CustomerInventoryBalance(9,2,8): - I_customer(9)(1)(8)#256
                                 + I_customer(9)(2)(8)#266
                                 - b_customer(9)(2)(8)#566 - w(9)(1)(2)(8)#1793
                                 - w(9)(2)(2)(8)#1823 - w(9)(3)(2)(8)#1853
                                 - w(9)(4)(2)(8)#1883 - w(9)(5)(2)(8)#1913
                                 - w(9)(6)(2)(8)#1943  = -20
 CustomerInventoryBalance(10,2,8): - I_customer(10)(1)(8)#286
                                 + I_customer(10)(2)(8)#296
                                 - b_customer(10)(2)(8)#596
                                 - w(10)(1)(2)(8)#1973 - w(10)(2)(2)(8)#2003
                                 - w(10)(3)(2)(8)#2033 - w(10)(4)(2)(8)#2063
                                 - w(10)(5)(2)(8)#2093 - w(10)(6)(2)(8)#2123
                                  = -11
 CustomerInventoryBalance(11,2,8): - I_customer(11)(1)(8)#316
                                 + I_customer(11)(2)(8)#326
                                 - b_customer(11)(2)(8)#626
                                 - w(11)(1)(2)(8)#2153 - w(11)(2)(2)(8)#2183
                                 - w(11)(3)(2)(8)#2213 - w(11)(4)(2)(8)#2243
                                 - w(11)(5)(2)(8)#2273 - w(11)(6)(2)(8)#2303
                                  = -17
 CustomerInventoryBalance(12,2,8): - I_customer(12)(1)(8)#346
                                 + I_customer(12)(2)(8)#356
                                 - b_customer(12)(2)(8)#656
                                 - w(12)(1)(2)(8)#2333 - w(12)(2)(2)(8)#2363
                                 - w(12)(3)(2)(8)#2393 - w(12)(4)(2)(8)#2423
                                 - w(12)(5)(2)(8)#2453 - w(12)(6)(2)(8)#2483
                                  = -26
 CustomerInventoryBalance(3,3,8): - I_customer(3)(2)(8)#86
                                 + I_customer(3)(3)(8)#96
                                 - b_customer(3)(3)(8)#396 - w(3)(1)(3)(8)#723
                                 - w(3)(2)(3)(8)#753 - w(3)(3)(3)(8)#783
                                 - w(3)(4)(3)(8)#813 - w(3)(5)(3)(8)#843
                                 - w(3)(6)(3)(8)#873  = -8
 CustomerInventoryBalance(4,3,8): - I_customer(4)(2)(8)#116
                                 + I_customer(4)(3)(8)#126
                                 - b_customer(4)(3)(8)#426 - w(4)(1)(3)(8)#903
                                 - w(4)(2)(3)(8)#933 - w(4)(3)(3)(8)#963
                                 - w(4)(4)(3)(8)#993 - w(4)(5)(3)(8)#1023
                                 - w(4)(6)(3)(8)#1053  = -16
 CustomerInventoryBalance(5,3,8): - I_customer(5)(2)(8)#146
                                 + I_customer(5)(3)(8)#156
                                 - b_customer(5)(3)(8)#456 - w(5)(1)(3)(8)#1083
                                 - w(5)(2)(3)(8)#1113 - w(5)(3)(3)(8)#1143
                                 - w(5)(4)(3)(8)#1173 - w(5)(5)(3)(8)#1203
                                 - w(5)(6)(3)(8)#1233  = -17
 CustomerInventoryBalance(6,3,8): - I_customer(6)(2)(8)#176
                                 + I_customer(6)(3)(8)#186
                                 - b_customer(6)(3)(8)#486 - w(6)(1)(3)(8)#1263
                                 - w(6)(2)(3)(8)#1293 - w(6)(3)(3)(8)#1323
                                 - w(6)(4)(3)(8)#1353 - w(6)(5)(3)(8)#1383
                                 - w(6)(6)(3)(8)#1413  = -7
 CustomerInventoryBalance(7,3,8): - I_customer(7)(2)(8)#206
                                 + I_customer(7)(3)(8)#216
                                 - b_customer(7)(3)(8)#516 - w(7)(1)(3)(8)#1443
                                 - w(7)(2)(3)(8)#1473 - w(7)(3)(3)(8)#1503
                                 - w(7)(4)(3)(8)#1533 - w(7)(5)(3)(8)#1563
                                 - w(7)(6)(3)(8)#1593  = -13
 CustomerInventoryBalance(8,3,8): - I_customer(8)(2)(8)#236
                                 + I_customer(8)(3)(8)#246
                                 - b_customer(8)(3)(8)#546 - w(8)(1)(3)(8)#1623
                                 - w(8)(2)(3)(8)#1653 - w(8)(3)(3)(8)#1683
                                 - w(8)(4)(3)(8)#1713 - w(8)(5)(3)(8)#1743
                                 - w(8)(6)(3)(8)#1773  = -19
 CustomerInventoryBalance(9,3,8): - I_customer(9)(2)(8)#266
                                 + I_customer(9)(3)(8)#276
                                 - b_customer(9)(3)(8)#576 - w(9)(1)(3)(8)#1803
                                 - w(9)(2)(3)(8)#1833 - w(9)(3)(3)(8)#1863
                                 - w(9)(4)(3)(8)#1893 - w(9)(5)(3)(8)#1923
                                 - w(9)(6)(3)(8)#1953  = -25
 CustomerInventoryBalance(10,3,8): - I_customer(10)(2)(8)#296
                                 + I_customer(10)(3)(8)#306
                                 - b_customer(10)(3)(8)#606
                                 - w(10)(1)(3)(8)#1983 - w(10)(2)(3)(8)#2013
                                 - w(10)(3)(3)(8)#2043 - w(10)(4)(3)(8)#2073
                                 - w(10)(5)(3)(8)#2103 - w(10)(6)(3)(8)#2133
                                  = -14
 CustomerInventoryBalance(11,3,8): - I_customer(11)(2)(8)#326
                                 + I_customer(11)(3)(8)#336
                                 - b_customer(11)(3)(8)#636
                                 - w(11)(1)(3)(8)#2163 - w(11)(2)(3)(8)#2193
                                 - w(11)(3)(3)(8)#2223 - w(11)(4)(3)(8)#2253
                                 - w(11)(5)(3)(8)#2283 - w(11)(6)(3)(8)#2313
                                  = -19
 CustomerInventoryBalance(12,3,8): - I_customer(12)(2)(8)#356
                                 + I_customer(12)(3)(8)#366
                                 - b_customer(12)(3)(8)#666
                                 - w(12)(1)(3)(8)#2343 - w(12)(2)(3)(8)#2373
                                 - w(12)(3)(3)(8)#2403 - w(12)(4)(3)(8)#2433
                                 - w(12)(5)(3)(8)#2463 - w(12)(6)(3)(8)#2493
                                  = -22
 CustomerInventoryBalance(3,1,9): I_customer(3)(1)(9)#77
                                 - b_customer(3)(1)(9)#377 - w(3)(1)(1)(9)#704
                                 - w(3)(2)(1)(9)#734 - w(3)(3)(1)(9)#764
                                 - w(3)(4)(1)(9)#794 - w(3)(5)(1)(9)#824
                                 - w(3)(6)(1)(9)#854  = -7
 CustomerInventoryBalance(4,1,9): I_customer(4)(1)(9)#107
                                 - b_customer(4)(1)(9)#407 - w(4)(1)(1)(9)#884
                                 - w(4)(2)(1)(9)#914 - w(4)(3)(1)(9)#944
                                 - w(4)(4)(1)(9)#974 - w(4)(5)(1)(9)#1004
                                 - w(4)(6)(1)(9)#1034  = -3
 CustomerInventoryBalance(5,1,9): I_customer(5)(1)(9)#137
                                 - b_customer(5)(1)(9)#437 - w(5)(1)(1)(9)#1064
                                 - w(5)(2)(1)(9)#1094 - w(5)(3)(1)(9)#1124
                                 - w(5)(4)(1)(9)#1154 - w(5)(5)(1)(9)#1184
                                 - w(5)(6)(1)(9)#1214  = 2
 CustomerInventoryBalance(6,1,9): I_customer(6)(1)(9)#167
                                 - b_customer(6)(1)(9)#467 - w(6)(1)(1)(9)#1244
                                 - w(6)(2)(1)(9)#1274 - w(6)(3)(1)(9)#1304
                                 - w(6)(4)(1)(9)#1334 - w(6)(5)(1)(9)#1364
                                 - w(6)(6)(1)(9)#1394  = -4
 CustomerInventoryBalance(7,1,9): I_customer(7)(1)(9)#197
                                 - b_customer(7)(1)(9)#497 - w(7)(1)(1)(9)#1424
                                 - w(7)(2)(1)(9)#1454 - w(7)(3)(1)(9)#1484
                                 - w(7)(4)(1)(9)#1514 - w(7)(5)(1)(9)#1544
                                 - w(7)(6)(1)(9)#1574  = -1
 CustomerInventoryBalance(8,1,9): I_customer(8)(1)(9)#227
                                 - b_customer(8)(1)(9)#527 - w(8)(1)(1)(9)#1604
                                 - w(8)(2)(1)(9)#1634 - w(8)(3)(1)(9)#1664
                                 - w(8)(4)(1)(9)#1694 - w(8)(5)(1)(9)#1724
                                 - w(8)(6)(1)(9)#1754  = 23
 CustomerInventoryBalance(9,1,9): I_customer(9)(1)(9)#257
                                 - b_customer(9)(1)(9)#557 - w(9)(1)(1)(9)#1784
                                 - w(9)(2)(1)(9)#1814 - w(9)(3)(1)(9)#1844
                                 - w(9)(4)(1)(9)#1874 - w(9)(5)(1)(9)#1904
                                 - w(9)(6)(1)(9)#1934  = 31
 CustomerInventoryBalance(10,1,9): I_customer(10)(1)(9)#287
                                 - b_customer(10)(1)(9)#587
                                 - w(10)(1)(1)(9)#1964 - w(10)(2)(1)(9)#1994
                                 - w(10)(3)(1)(9)#2024 - w(10)(4)(1)(9)#2054
                                 - w(10)(5)(1)(9)#2084 - w(10)(6)(1)(9)#2114
                                  = -5
 CustomerInventoryBalance(11,1,9): I_customer(11)(1)(9)#317
                                 - b_customer(11)(1)(9)#617
                                 - w(11)(1)(1)(9)#2144 - w(11)(2)(1)(9)#2174
                                 - w(11)(3)(1)(9)#2204 - w(11)(4)(1)(9)#2234
                                 - w(11)(5)(1)(9)#2264 - w(11)(6)(1)(9)#2294
                                  = 26
 CustomerInventoryBalance(12,1,9): I_customer(12)(1)(9)#347
                                 - b_customer(12)(1)(9)#647
                                 - w(12)(1)(1)(9)#2324 - w(12)(2)(1)(9)#2354
                                 - w(12)(3)(1)(9)#2384 - w(12)(4)(1)(9)#2414
                                 - w(12)(5)(1)(9)#2444 - w(12)(6)(1)(9)#2474
                                  = 26
 CustomerInventoryBalance(3,2,9): - I_customer(3)(1)(9)#77
                                 + I_customer(3)(2)(9)#87
                                 - b_customer(3)(2)(9)#387 - w(3)(1)(2)(9)#714
                                 - w(3)(2)(2)(9)#744 - w(3)(3)(2)(9)#774
                                 - w(3)(4)(2)(9)#804 - w(3)(5)(2)(9)#834
                                 - w(3)(6)(2)(9)#864  = -10
 CustomerInventoryBalance(4,2,9): - I_customer(4)(1)(9)#107
                                 + I_customer(4)(2)(9)#117
                                 - b_customer(4)(2)(9)#417 - w(4)(1)(2)(9)#894
                                 - w(4)(2)(2)(9)#924 - w(4)(3)(2)(9)#954
                                 - w(4)(4)(2)(9)#984 - w(4)(5)(2)(9)#1014
                                 - w(4)(6)(2)(9)#1044  = -15
 CustomerInventoryBalance(5,2,9): - I_customer(5)(1)(9)#137
                                 + I_customer(5)(2)(9)#147
                                 - b_customer(5)(2)(9)#447 - w(5)(1)(2)(9)#1074
                                 - w(5)(2)(2)(9)#1104 - w(5)(3)(2)(9)#1134
                                 - w(5)(4)(2)(9)#1164 - w(5)(5)(2)(9)#1194
                                 - w(5)(6)(2)(9)#1224  = -17
 CustomerInventoryBalance(6,2,9): - I_customer(6)(1)(9)#167
                                 + I_customer(6)(2)(9)#177
                                 - b_customer(6)(2)(9)#477 - w(6)(1)(2)(9)#1254
                                 - w(6)(2)(2)(9)#1284 - w(6)(3)(2)(9)#1314
                                 - w(6)(4)(2)(9)#1344 - w(6)(5)(2)(9)#1374
                                 - w(6)(6)(2)(9)#1404  = -7
 CustomerInventoryBalance(7,2,9): - I_customer(7)(1)(9)#197
                                 + I_customer(7)(2)(9)#207
                                 - b_customer(7)(2)(9)#507 - w(7)(1)(2)(9)#1434
                                 - w(7)(2)(2)(9)#1464 - w(7)(3)(2)(9)#1494
                                 - w(7)(4)(2)(9)#1524 - w(7)(5)(2)(9)#1554
                                 - w(7)(6)(2)(9)#1584  = -13
 CustomerInventoryBalance(8,2,9): - I_customer(8)(1)(9)#227
                                 + I_customer(8)(2)(9)#237
                                 - b_customer(8)(2)(9)#537 - w(8)(1)(2)(9)#1614
                                 - w(8)(2)(2)(9)#1644 - w(8)(3)(2)(9)#1674
                                 - w(8)(4)(2)(9)#1704 - w(8)(5)(2)(9)#1734
                                 - w(8)(6)(2)(9)#1764  = -15
 CustomerInventoryBalance(9,2,9): - I_customer(9)(1)(9)#257
                                 + I_customer(9)(2)(9)#267
                                 - b_customer(9)(2)(9)#567 - w(9)(1)(2)(9)#1794
                                 - w(9)(2)(2)(9)#1824 - w(9)(3)(2)(9)#1854
                                 - w(9)(4)(2)(9)#1884 - w(9)(5)(2)(9)#1914
                                 - w(9)(6)(2)(9)#1944  = -25
 CustomerInventoryBalance(10,2,9): - I_customer(10)(1)(9)#287
                                 + I_customer(10)(2)(9)#297
                                 - b_customer(10)(2)(9)#597
                                 - w(10)(1)(2)(9)#1974 - w(10)(2)(2)(9)#2004
                                 - w(10)(3)(2)(9)#2034 - w(10)(4)(2)(9)#2064
                                 - w(10)(5)(2)(9)#2094 - w(10)(6)(2)(9)#2124
                                  = -11
 CustomerInventoryBalance(11,2,9): - I_customer(11)(1)(9)#317
                                 + I_customer(11)(2)(9)#327
                                 - b_customer(11)(2)(9)#627
                                 - w(11)(1)(2)(9)#2154 - w(11)(2)(2)(9)#2184
                                 - w(11)(3)(2)(9)#2214 - w(11)(4)(2)(9)#2244
                                 - w(11)(5)(2)(9)#2274 - w(11)(6)(2)(9)#2304
                                  = -15
 CustomerInventoryBalance(12,2,9): - I_customer(12)(1)(9)#347
                                 + I_customer(12)(2)(9)#357
                                 - b_customer(12)(2)(9)#657
                                 - w(12)(1)(2)(9)#2334 - w(12)(2)(2)(9)#2364
                                 - w(12)(3)(2)(9)#2394 - w(12)(4)(2)(9)#2424
                                 - w(12)(5)(2)(9)#2454 - w(12)(6)(2)(9)#2484
                                  = -25
 CustomerInventoryBalance(3,3,9): - I_customer(3)(2)(9)#87
                                 + I_customer(3)(3)(9)#97
                                 - b_customer(3)(3)(9)#397 - w(3)(1)(3)(9)#724
                                 - w(3)(2)(3)(9)#754 - w(3)(3)(3)(9)#784
                                 - w(3)(4)(3)(9)#814 - w(3)(5)(3)(9)#844
                                 - w(3)(6)(3)(9)#874  = -8
 CustomerInventoryBalance(4,3,9): - I_customer(4)(2)(9)#117
                                 + I_customer(4)(3)(9)#127
                                 - b_customer(4)(3)(9)#427 - w(4)(1)(3)(9)#904
                                 - w(4)(2)(3)(9)#934 - w(4)(3)(3)(9)#964
                                 - w(4)(4)(3)(9)#994 - w(4)(5)(3)(9)#1024
                                 - w(4)(6)(3)(9)#1054  = -18
 CustomerInventoryBalance(5,3,9): - I_customer(5)(2)(9)#147
                                 + I_customer(5)(3)(9)#157
                                 - b_customer(5)(3)(9)#457 - w(5)(1)(3)(9)#1084
                                 - w(5)(2)(3)(9)#1114 - w(5)(3)(3)(9)#1144
                                 - w(5)(4)(3)(9)#1174 - w(5)(5)(3)(9)#1204
                                 - w(5)(6)(3)(9)#1234  = -13
 CustomerInventoryBalance(6,3,9): - I_customer(6)(2)(9)#177
                                 + I_customer(6)(3)(9)#187
                                 - b_customer(6)(3)(9)#487 - w(6)(1)(3)(9)#1264
                                 - w(6)(2)(3)(9)#1294 - w(6)(3)(3)(9)#1324
                                 - w(6)(4)(3)(9)#1354 - w(6)(5)(3)(9)#1384
                                 - w(6)(6)(3)(9)#1414  = -6
 CustomerInventoryBalance(7,3,9): - I_customer(7)(2)(9)#207
                                 + I_customer(7)(3)(9)#217
                                 - b_customer(7)(3)(9)#517 - w(7)(1)(3)(9)#1444
                                 - w(7)(2)(3)(9)#1474 - w(7)(3)(3)(9)#1504
                                 - w(7)(4)(3)(9)#1534 - w(7)(5)(3)(9)#1564
                                 - w(7)(6)(3)(9)#1594  = -13
 CustomerInventoryBalance(8,3,9): - I_customer(8)(2)(9)#237
                                 + I_customer(8)(3)(9)#247
                                 - b_customer(8)(3)(9)#547 - w(8)(1)(3)(9)#1624
                                 - w(8)(2)(3)(9)#1654 - w(8)(3)(3)(9)#1684
                                 - w(8)(4)(3)(9)#1714 - w(8)(5)(3)(9)#1744
                                 - w(8)(6)(3)(9)#1774  = -13
 CustomerInventoryBalance(9,3,9): - I_customer(9)(2)(9)#267
                                 + I_customer(9)(3)(9)#277
                                 - b_customer(9)(3)(9)#577 - w(9)(1)(3)(9)#1804
                                 - w(9)(2)(3)(9)#1834 - w(9)(3)(3)(9)#1864
                                 - w(9)(4)(3)(9)#1894 - w(9)(5)(3)(9)#1924
                                 - w(9)(6)(3)(9)#1954  = -24
 CustomerInventoryBalance(10,3,9): - I_customer(10)(2)(9)#297
                                 + I_customer(10)(3)(9)#307
                                 - b_customer(10)(3)(9)#607
                                 - w(10)(1)(3)(9)#1984 - w(10)(2)(3)(9)#2014
                                 - w(10)(3)(3)(9)#2044 - w(10)(4)(3)(9)#2074
                                 - w(10)(5)(3)(9)#2104 - w(10)(6)(3)(9)#2134
                                  = -15
 CustomerInventoryBalance(11,3,9): - I_customer(11)(2)(9)#327
                                 + I_customer(11)(3)(9)#337
                                 - b_customer(11)(3)(9)#637
                                 - w(11)(1)(3)(9)#2164 - w(11)(2)(3)(9)#2194
                                 - w(11)(3)(3)(9)#2224 - w(11)(4)(3)(9)#2254
                                 - w(11)(5)(3)(9)#2284 - w(11)(6)(3)(9)#2314
                                  = -19
 CustomerInventoryBalance(12,3,9): - I_customer(12)(2)(9)#357
                                 + I_customer(12)(3)(9)#367
                                 - b_customer(12)(3)(9)#667
                                 - w(12)(1)(3)(9)#2344 - w(12)(2)(3)(9)#2374
                                 - w(12)(3)(3)(9)#2404 - w(12)(4)(3)(9)#2434
                                 - w(12)(5)(3)(9)#2464 - w(12)(6)(3)(9)#2494
                                  = -21
 CustomerInventoryBalance(3,1,10): I_customer(3)(1)(10)#78
                                 - b_customer(3)(1)(10)#378
                                 - w(3)(1)(1)(10)#705 - w(3)(2)(1)(10)#735
                                 - w(3)(3)(1)(10)#765 - w(3)(4)(1)(10)#795
                                 - w(3)(5)(1)(10)#825 - w(3)(6)(1)(10)#855
                                  = -6
 CustomerInventoryBalance(4,1,10): I_customer(4)(1)(10)#108
                                 - b_customer(4)(1)(10)#408
                                 - w(4)(1)(1)(10)#885 - w(4)(2)(1)(10)#915
                                 - w(4)(3)(1)(10)#945 - w(4)(4)(1)(10)#975
                                 - w(4)(5)(1)(10)#1005 - w(4)(6)(1)(10)#1035
                                  = 1
 CustomerInventoryBalance(5,1,10): I_customer(5)(1)(10)#138
                                 - b_customer(5)(1)(10)#438
                                 - w(5)(1)(1)(10)#1065 - w(5)(2)(1)(10)#1095
                                 - w(5)(3)(1)(10)#1125 - w(5)(4)(1)(10)#1155
                                 - w(5)(5)(1)(10)#1185 - w(5)(6)(1)(10)#1215
                                  = 0
 CustomerInventoryBalance(6,1,10): I_customer(6)(1)(10)#168
                                 - b_customer(6)(1)(10)#468
                                 - w(6)(1)(1)(10)#1245 - w(6)(2)(1)(10)#1275
                                 - w(6)(3)(1)(10)#1305 - w(6)(4)(1)(10)#1335
                                 - w(6)(5)(1)(10)#1365 - w(6)(6)(1)(10)#1395
                                  = -4
 CustomerInventoryBalance(7,1,10): I_customer(7)(1)(10)#198
                                 - b_customer(7)(1)(10)#498
                                 - w(7)(1)(1)(10)#1425 - w(7)(2)(1)(10)#1455
                                 - w(7)(3)(1)(10)#1485 - w(7)(4)(1)(10)#1515
                                 - w(7)(5)(1)(10)#1545 - w(7)(6)(1)(10)#1575
                                  = -2
 CustomerInventoryBalance(8,1,10): I_customer(8)(1)(10)#228
                                 - b_customer(8)(1)(10)#528
                                 - w(8)(1)(1)(10)#1605 - w(8)(2)(1)(10)#1635
                                 - w(8)(3)(1)(10)#1665 - w(8)(4)(1)(10)#1695
                                 - w(8)(5)(1)(10)#1725 - w(8)(6)(1)(10)#1755
                                  = 24
 CustomerInventoryBalance(9,1,10): I_customer(9)(1)(10)#258
                                 - b_customer(9)(1)(10)#558
                                 - w(9)(1)(1)(10)#1785 - w(9)(2)(1)(10)#1815
                                 - w(9)(3)(1)(10)#1845 - w(9)(4)(1)(10)#1875
                                 - w(9)(5)(1)(10)#1905 - w(9)(6)(1)(10)#1935
                                  = 35
 CustomerInventoryBalance(10,1,10): I_customer(10)(1)(10)#288
                                 - b_customer(10)(1)(10)#588
                                 - w(10)(1)(1)(10)#1965 - w(10)(2)(1)(10)#1995
                                 - w(10)(3)(1)(10)#2025 - w(10)(4)(1)(10)#2055
                                 - w(10)(5)(1)(10)#2085 - w(10)(6)(1)(10)#2115
                                  = -5
 CustomerInventoryBalance(11,1,10): I_customer(11)(1)(10)#318
                                 - b_customer(11)(1)(10)#618
                                 - w(11)(1)(1)(10)#2145 - w(11)(2)(1)(10)#2175
                                 - w(11)(3)(1)(10)#2205 - w(11)(4)(1)(10)#2235
                                 - w(11)(5)(1)(10)#2265 - w(11)(6)(1)(10)#2295
                                  = 28
 CustomerInventoryBalance(12,1,10): I_customer(12)(1)(10)#348
                                 - b_customer(12)(1)(10)#648
                                 - w(12)(1)(1)(10)#2325 - w(12)(2)(1)(10)#2355
                                 - w(12)(3)(1)(10)#2385 - w(12)(4)(1)(10)#2415
                                 - w(12)(5)(1)(10)#2445 - w(12)(6)(1)(10)#2475
                                  = 26
 CustomerInventoryBalance(3,2,10): - I_customer(3)(1)(10)#78
                                 + I_customer(3)(2)(10)#88
                                 - b_customer(3)(2)(10)#388
                                 - w(3)(1)(2)(10)#715 - w(3)(2)(2)(10)#745
                                 - w(3)(3)(2)(10)#775 - w(3)(4)(2)(10)#805
                                 - w(3)(5)(2)(10)#835 - w(3)(6)(2)(10)#865
                                  = -10
 CustomerInventoryBalance(4,2,10): - I_customer(4)(1)(10)#108
                                 + I_customer(4)(2)(10)#118
                                 - b_customer(4)(2)(10)#418
                                 - w(4)(1)(2)(10)#895 - w(4)(2)(2)(10)#925
                                 - w(4)(3)(2)(10)#955 - w(4)(4)(2)(10)#985
                                 - w(4)(5)(2)(10)#1015 - w(4)(6)(2)(10)#1045
                                  = -14
 CustomerInventoryBalance(5,2,10): - I_customer(5)(1)(10)#138
                                 + I_customer(5)(2)(10)#148
                                 - b_customer(5)(2)(10)#448
                                 - w(5)(1)(2)(10)#1075 - w(5)(2)(2)(10)#1105
                                 - w(5)(3)(2)(10)#1135 - w(5)(4)(2)(10)#1165
                                 - w(5)(5)(2)(10)#1195 - w(5)(6)(2)(10)#1225
                                  = -13
 CustomerInventoryBalance(6,2,10): - I_customer(6)(1)(10)#168
                                 + I_customer(6)(2)(10)#178
                                 - b_customer(6)(2)(10)#478
                                 - w(6)(1)(2)(10)#1255 - w(6)(2)(2)(10)#1285
                                 - w(6)(3)(2)(10)#1315 - w(6)(4)(2)(10)#1345
                                 - w(6)(5)(2)(10)#1375 - w(6)(6)(2)(10)#1405
                                  = -8
 CustomerInventoryBalance(7,2,10): - I_customer(7)(1)(10)#198
                                 + I_customer(7)(2)(10)#208
                                 - b_customer(7)(2)(10)#508
                                 - w(7)(1)(2)(10)#1435 - w(7)(2)(2)(10)#1465
                                 - w(7)(3)(2)(10)#1495 - w(7)(4)(2)(10)#1525
                                 - w(7)(5)(2)(10)#1555 - w(7)(6)(2)(10)#1585
                                  = -13
 CustomerInventoryBalance(8,2,10): - I_customer(8)(1)(10)#228
                                 + I_customer(8)(2)(10)#238
                                 - b_customer(8)(2)(10)#538
                                 - w(8)(1)(2)(10)#1615 - w(8)(2)(2)(10)#1645
                                 - w(8)(3)(2)(10)#1675 - w(8)(4)(2)(10)#1705
                                 - w(8)(5)(2)(10)#1735 - w(8)(6)(2)(10)#1765
                                  = -15
 CustomerInventoryBalance(9,2,10): - I_customer(9)(1)(10)#258
                                 + I_customer(9)(2)(10)#268
                                 - b_customer(9)(2)(10)#568
                                 - w(9)(1)(2)(10)#1795 - w(9)(2)(2)(10)#1825
                                 - w(9)(3)(2)(10)#1855 - w(9)(4)(2)(10)#1885
                                 - w(9)(5)(2)(10)#1915 - w(9)(6)(2)(10)#1945
                                  = -25
 CustomerInventoryBalance(10,2,10): - I_customer(10)(1)(10)#288
                                 + I_customer(10)(2)(10)#298
                                 - b_customer(10)(2)(10)#598
                                 - w(10)(1)(2)(10)#1975 - w(10)(2)(2)(10)#2005
                                 - w(10)(3)(2)(10)#2035 - w(10)(4)(2)(10)#2065
                                 - w(10)(5)(2)(10)#2095 - w(10)(6)(2)(10)#2125
                                  = -13
 CustomerInventoryBalance(11,2,10): - I_customer(11)(1)(10)#318
                                 + I_customer(11)(2)(10)#328
                                 - b_customer(11)(2)(10)#628
                                 - w(11)(1)(2)(10)#2155 - w(11)(2)(2)(10)#2185
                                 - w(11)(3)(2)(10)#2215 - w(11)(4)(2)(10)#2245
                                 - w(11)(5)(2)(10)#2275 - w(11)(6)(2)(10)#2305
                                  = -18
 CustomerInventoryBalance(12,2,10): - I_customer(12)(1)(10)#348
                                 + I_customer(12)(2)(10)#358
                                 - b_customer(12)(2)(10)#658
                                 - w(12)(1)(2)(10)#2335 - w(12)(2)(2)(10)#2365
                                 - w(12)(3)(2)(10)#2395 - w(12)(4)(2)(10)#2425
                                 - w(12)(5)(2)(10)#2455 - w(12)(6)(2)(10)#2485
                                  = -18
 CustomerInventoryBalance(3,3,10): - I_customer(3)(2)(10)#88
                                 + I_customer(3)(3)(10)#98
                                 - b_customer(3)(3)(10)#398
                                 - w(3)(1)(3)(10)#725 - w(3)(2)(3)(10)#755
                                 - w(3)(3)(3)(10)#785 - w(3)(4)(3)(10)#815
                                 - w(3)(5)(3)(10)#845 - w(3)(6)(3)(10)#875
                                  = -11
 CustomerInventoryBalance(4,3,10): - I_customer(4)(2)(10)#118
                                 + I_customer(4)(3)(10)#128
                                 - b_customer(4)(3)(10)#428
                                 - w(4)(1)(3)(10)#905 - w(4)(2)(3)(10)#935
                                 - w(4)(3)(3)(10)#965 - w(4)(4)(3)(10)#995
                                 - w(4)(5)(3)(10)#1025 - w(4)(6)(3)(10)#1055
                                  = -14
 CustomerInventoryBalance(5,3,10): - I_customer(5)(2)(10)#148
                                 + I_customer(5)(3)(10)#158
                                 - b_customer(5)(3)(10)#458
                                 - w(5)(1)(3)(10)#1085 - w(5)(2)(3)(10)#1115
                                 - w(5)(3)(3)(10)#1145 - w(5)(4)(3)(10)#1175
                                 - w(5)(5)(3)(10)#1205 - w(5)(6)(3)(10)#1235
                                  = -16
 CustomerInventoryBalance(6,3,10): - I_customer(6)(2)(10)#178
                                 + I_customer(6)(3)(10)#188
                                 - b_customer(6)(3)(10)#488
                                 - w(6)(1)(3)(10)#1265 - w(6)(2)(3)(10)#1295
                                 - w(6)(3)(3)(10)#1325 - w(6)(4)(3)(10)#1355
                                 - w(6)(5)(3)(10)#1385 - w(6)(6)(3)(10)#1415
                                  = -8
 CustomerInventoryBalance(7,3,10): - I_customer(7)(2)(10)#208
                                 + I_customer(7)(3)(10)#218
                                 - b_customer(7)(3)(10)#518
                                 - w(7)(1)(3)(10)#1445 - w(7)(2)(3)(10)#1475
                                 - w(7)(3)(3)(10)#1505 - w(7)(4)(3)(10)#1535
                                 - w(7)(5)(3)(10)#1565 - w(7)(6)(3)(10)#1595
                                  = -12
 CustomerInventoryBalance(8,3,10): - I_customer(8)(2)(10)#238
                                 + I_customer(8)(3)(10)#248
                                 - b_customer(8)(3)(10)#548
                                 - w(8)(1)(3)(10)#1625 - w(8)(2)(3)(10)#1655
                                 - w(8)(3)(3)(10)#1685 - w(8)(4)(3)(10)#1715
                                 - w(8)(5)(3)(10)#1745 - w(8)(6)(3)(10)#1775
                                  = -15
 CustomerInventoryBalance(9,3,10): - I_customer(9)(2)(10)#268
                                 + I_customer(9)(3)(10)#278
                                 - b_customer(9)(3)(10)#578
                                 - w(9)(1)(3)(10)#1805 - w(9)(2)(3)(10)#1835
                                 - w(9)(3)(3)(10)#1865 - w(9)(4)(3)(10)#1895
                                 - w(9)(5)(3)(10)#1925 - w(9)(6)(3)(10)#1955
                                  = -25
 CustomerInventoryBalance(10,3,10): - I_customer(10)(2)(10)#298
                                 + I_customer(10)(3)(10)#308
                                 - b_customer(10)(3)(10)#608
                                 - w(10)(1)(3)(10)#1985 - w(10)(2)(3)(10)#2015
                                 - w(10)(3)(3)(10)#2045 - w(10)(4)(3)(10)#2075
                                 - w(10)(5)(3)(10)#2105 - w(10)(6)(3)(10)#2135
                                  = -13
 CustomerInventoryBalance(11,3,10): - I_customer(11)(2)(10)#328
                                 + I_customer(11)(3)(10)#338
                                 - b_customer(11)(3)(10)#638
                                 - w(11)(1)(3)(10)#2165 - w(11)(2)(3)(10)#2195
                                 - w(11)(3)(3)(10)#2225 - w(11)(4)(3)(10)#2255
                                 - w(11)(5)(3)(10)#2285 - w(11)(6)(3)(10)#2315
                                  = -17
 CustomerInventoryBalance(12,3,10): - I_customer(12)(2)(10)#358
                                 + I_customer(12)(3)(10)#368
                                 - b_customer(12)(3)(10)#668
                                 - w(12)(1)(3)(10)#2345 - w(12)(2)(3)(10)#2375
                                 - w(12)(3)(3)(10)#2405 - w(12)(4)(3)(10)#2435
                                 - w(12)(5)(3)(10)#2465 - w(12)(6)(3)(10)#2495
                                  = -25
 CustomerInventoryCapacity(3,1,1): I_customer(3)(1)(1)#69 <= 10
 CustomerInventoryCapacity(4,1,1): I_customer(4)(1)(1)#99 <= 27
 CustomerInventoryCapacity(5,1,1): I_customer(5)(1)(1)#129 <= 32
 CustomerInventoryCapacity(6,1,1): I_customer(6)(1)(1)#159 <= 8
 CustomerInventoryCapacity(7,1,1): I_customer(7)(1)(1)#189 <= 24
 CustomerInventoryCapacity(8,1,1): I_customer(8)(1)(1)#219 <= 83
 CustomerInventoryCapacity(9,1,1): I_customer(9)(1)(1)#249 <= 106
 CustomerInventoryCapacity(10,1,1): I_customer(10)(1)(1)#279 <= 14
 CustomerInventoryCapacity(11,1,1): I_customer(11)(1)(1)#309 <= 99
 CustomerInventoryCapacity(12,1,1): I_customer(12)(1)(1)#339 <= 89
 CustomerInventoryCapacity(3,2,1): I_customer(3)(2)(1)#79 <= 11
 CustomerInventoryCapacity(4,2,1): I_customer(4)(2)(1)#109 <= 33
 CustomerInventoryCapacity(5,2,1): I_customer(5)(2)(1)#139 <= 30
 CustomerInventoryCapacity(6,2,1): I_customer(6)(2)(1)#169 <= 7
 CustomerInventoryCapacity(7,2,1): I_customer(7)(2)(1)#199 <= 24
 CustomerInventoryCapacity(8,2,1): I_customer(8)(2)(1)#229 <= 78
 CustomerInventoryCapacity(9,2,1): I_customer(9)(2)(1)#259 <= 112
 CustomerInventoryCapacity(10,2,1): I_customer(10)(2)(1)#289 <= 15
 CustomerInventoryCapacity(11,2,1): I_customer(11)(2)(1)#319 <= 98
 CustomerInventoryCapacity(12,2,1): I_customer(12)(2)(1)#349 <= 89
 CustomerInventoryCapacity(3,3,1): I_customer(3)(3)(1)#89 <= 8
 CustomerInventoryCapacity(4,3,1): I_customer(4)(3)(1)#119 <= 28
 CustomerInventoryCapacity(5,3,1): I_customer(5)(3)(1)#149 <= 29
 CustomerInventoryCapacity(6,3,1): I_customer(6)(3)(1)#179 <= 7
 CustomerInventoryCapacity(7,3,1): I_customer(7)(3)(1)#209 <= 27
 CustomerInventoryCapacity(8,3,1): I_customer(8)(3)(1)#239 <= 79
 CustomerInventoryCapacity(9,3,1): I_customer(9)(3)(1)#269 <= 108
 CustomerInventoryCapacity(10,3,1): I_customer(10)(3)(1)#299 <= 13
 CustomerInventoryCapacity(11,3,1): I_customer(11)(3)(1)#329 <= 98
 CustomerInventoryCapacity(12,3,1): I_customer(12)(3)(1)#359 <= 85
 CustomerInventoryCapacity(3,1,2): I_customer(3)(1)(2)#70 <= 8
 CustomerInventoryCapacity(4,1,2): I_customer(4)(1)(2)#100 <= 33
 CustomerInventoryCapacity(5,1,2): I_customer(5)(1)(2)#130 <= 33
 CustomerInventoryCapacity(6,1,2): I_customer(6)(1)(2)#160 <= 6
 CustomerInventoryCapacity(7,1,2): I_customer(7)(1)(2)#190 <= 28
 CustomerInventoryCapacity(8,1,2): I_customer(8)(1)(2)#220 <= 80
 CustomerInventoryCapacity(9,1,2): I_customer(9)(1)(2)#250 <= 114
 CustomerInventoryCapacity(10,1,2): I_customer(10)(1)(2)#280 <= 15
 CustomerInventoryCapacity(11,1,2): I_customer(11)(1)(2)#310 <= 92
 CustomerInventoryCapacity(12,1,2): I_customer(12)(1)(2)#340 <= 92
 CustomerInventoryCapacity(3,2,2): I_customer(3)(2)(2)#80 <= 9
 CustomerInventoryCapacity(4,2,2): I_customer(4)(2)(2)#110 <= 29
 CustomerInventoryCapacity(5,2,2): I_customer(5)(2)(2)#140 <= 28
 CustomerInventoryCapacity(6,2,2): I_customer(6)(2)(2)#170 <= 7
 CustomerInventoryCapacity(7,2,2): I_customer(7)(2)(2)#200 <= 26
 CustomerInventoryCapacity(8,2,2): I_customer(8)(2)(2)#230 <= 81
 CustomerInventoryCapacity(9,2,2): I_customer(9)(2)(2)#260 <= 112
 CustomerInventoryCapacity(10,2,2): I_customer(10)(2)(2)#290 <= 11
 CustomerInventoryCapacity(11,2,2): I_customer(11)(2)(2)#320 <= 96
 CustomerInventoryCapacity(12,2,2): I_customer(12)(2)(2)#350 <= 88
 CustomerInventoryCapacity(3,3,2): I_customer(3)(3)(2)#90 <= 11
 CustomerInventoryCapacity(4,3,2): I_customer(4)(3)(2)#120 <= 31
 CustomerInventoryCapacity(5,3,2): I_customer(5)(3)(2)#150 <= 28
 CustomerInventoryCapacity(6,3,2): I_customer(6)(3)(2)#180 <= 6
 CustomerInventoryCapacity(7,3,2): I_customer(7)(3)(2)#210 <= 28
 CustomerInventoryCapacity(8,3,2): I_customer(8)(3)(2)#240 <= 79
 CustomerInventoryCapacity(9,3,2): I_customer(9)(3)(2)#270 <= 114
 CustomerInventoryCapacity(10,3,2): I_customer(10)(3)(2)#300 <= 12
 CustomerInventoryCapacity(11,3,2): I_customer(11)(3)(2)#330 <= 92
 CustomerInventoryCapacity(12,3,2): I_customer(12)(3)(2)#360 <= 85
 CustomerInventoryCapacity(3,1,3): I_customer(3)(1)(3)#71 <= 9
 CustomerInventoryCapacity(4,1,3): I_customer(4)(1)(3)#101 <= 28
 CustomerInventoryCapacity(5,1,3): I_customer(5)(1)(3)#131 <= 30
 CustomerInventoryCapacity(6,1,3): I_customer(6)(1)(3)#161 <= 7
 CustomerInventoryCapacity(7,1,3): I_customer(7)(1)(3)#191 <= 25
 CustomerInventoryCapacity(8,1,3): I_customer(8)(1)(3)#221 <= 78
 CustomerInventoryCapacity(9,1,3): I_customer(9)(1)(3)#251 <= 113
 CustomerInventoryCapacity(10,1,3): I_customer(10)(1)(3)#281 <= 12
 CustomerInventoryCapacity(11,1,3): I_customer(11)(1)(3)#311 <= 96
 CustomerInventoryCapacity(12,1,3): I_customer(12)(1)(3)#341 <= 89
 CustomerInventoryCapacity(3,2,3): I_customer(3)(2)(3)#81 <= 9
 CustomerInventoryCapacity(4,2,3): I_customer(4)(2)(3)#111 <= 28
 CustomerInventoryCapacity(5,2,3): I_customer(5)(2)(3)#141 <= 27
 CustomerInventoryCapacity(6,2,3): I_customer(6)(2)(3)#171 <= 8
 CustomerInventoryCapacity(7,2,3): I_customer(7)(2)(3)#201 <= 26
 CustomerInventoryCapacity(8,2,3): I_customer(8)(2)(3)#231 <= 79
 CustomerInventoryCapacity(9,2,3): I_customer(9)(2)(3)#261 <= 110
 CustomerInventoryCapacity(10,2,3): I_customer(10)(2)(3)#291 <= 15
 CustomerInventoryCapacity(11,2,3): I_customer(11)(2)(3)#321 <= 92
 CustomerInventoryCapacity(12,2,3): I_customer(12)(2)(3)#351 <= 89
 CustomerInventoryCapacity(3,3,3): I_customer(3)(3)(3)#91 <= 9
 CustomerInventoryCapacity(4,3,3): I_customer(4)(3)(3)#121 <= 30
 CustomerInventoryCapacity(5,3,3): I_customer(5)(3)(3)#151 <= 32
 CustomerInventoryCapacity(6,3,3): I_customer(6)(3)(3)#181 <= 7
 CustomerInventoryCapacity(7,3,3): I_customer(7)(3)(3)#211 <= 26
 CustomerInventoryCapacity(8,3,3): I_customer(8)(3)(3)#241 <= 77
 CustomerInventoryCapacity(9,3,3): I_customer(9)(3)(3)#271 <= 106
 CustomerInventoryCapacity(10,3,3): I_customer(10)(3)(3)#301 <= 11
 CustomerInventoryCapacity(11,3,3): I_customer(11)(3)(3)#331 <= 94
 CustomerInventoryCapacity(12,3,3): I_customer(12)(3)(3)#361 <= 89
 CustomerInventoryCapacity(3,1,4): I_customer(3)(1)(4)#72 <= 9
 CustomerInventoryCapacity(4,1,4): I_customer(4)(1)(4)#102 <= 27
 CustomerInventoryCapacity(5,1,4): I_customer(5)(1)(4)#132 <= 28
 CustomerInventoryCapacity(6,1,4): I_customer(6)(1)(4)#162 <= 7
 CustomerInventoryCapacity(7,1,4): I_customer(7)(1)(4)#192 <= 25
 CustomerInventoryCapacity(8,1,4): I_customer(8)(1)(4)#222 <= 82
 CustomerInventoryCapacity(9,1,4): I_customer(9)(1)(4)#252 <= 110
 CustomerInventoryCapacity(10,1,4): I_customer(10)(1)(4)#282 <= 11
 CustomerInventoryCapacity(11,1,4): I_customer(11)(1)(4)#312 <= 96
 CustomerInventoryCapacity(12,1,4): I_customer(12)(1)(4)#342 <= 86
 CustomerInventoryCapacity(3,2,4): I_customer(3)(2)(4)#82 <= 9
 CustomerInventoryCapacity(4,2,4): I_customer(4)(2)(4)#112 <= 30
 CustomerInventoryCapacity(5,2,4): I_customer(5)(2)(4)#142 <= 31
 CustomerInventoryCapacity(6,2,4): I_customer(6)(2)(4)#172 <= 8
 CustomerInventoryCapacity(7,2,4): I_customer(7)(2)(4)#202 <= 28
 CustomerInventoryCapacity(8,2,4): I_customer(8)(2)(4)#232 <= 81
 CustomerInventoryCapacity(9,2,4): I_customer(9)(2)(4)#262 <= 108
 CustomerInventoryCapacity(10,2,4): I_customer(10)(2)(4)#292 <= 10
 CustomerInventoryCapacity(11,2,4): I_customer(11)(2)(4)#322 <= 92
 CustomerInventoryCapacity(12,2,4): I_customer(12)(2)(4)#352 <= 87
 CustomerInventoryCapacity(3,3,4): I_customer(3)(3)(4)#92 <= 9
 CustomerInventoryCapacity(4,3,4): I_customer(4)(3)(4)#122 <= 32
 CustomerInventoryCapacity(5,3,4): I_customer(5)(3)(4)#152 <= 29
 CustomerInventoryCapacity(6,3,4): I_customer(6)(3)(4)#182 <= 8
 CustomerInventoryCapacity(7,3,4): I_customer(7)(3)(4)#212 <= 28
 CustomerInventoryCapacity(8,3,4): I_customer(8)(3)(4)#242 <= 81
 CustomerInventoryCapacity(9,3,4): I_customer(9)(3)(4)#272 <= 109
 CustomerInventoryCapacity(10,3,4): I_customer(10)(3)(4)#302 <= 11
 CustomerInventoryCapacity(11,3,4): I_customer(11)(3)(4)#332 <= 98
 CustomerInventoryCapacity(12,3,4): I_customer(12)(3)(4)#362 <= 91
 CustomerInventoryCapacity(3,1,5): I_customer(3)(1)(5)#73 <= 11
 CustomerInventoryCapacity(4,1,5): I_customer(4)(1)(5)#103 <= 28
 CustomerInventoryCapacity(5,1,5): I_customer(5)(1)(5)#133 <= 28
 CustomerInventoryCapacity(6,1,5): I_customer(6)(1)(5)#163 <= 6
 CustomerInventoryCapacity(7,1,5): I_customer(7)(1)(5)#193 <= 25
 CustomerInventoryCapacity(8,1,5): I_customer(8)(1)(5)#223 <= 77
 CustomerInventoryCapacity(9,1,5): I_customer(9)(1)(5)#253 <= 106
 CustomerInventoryCapacity(10,1,5): I_customer(10)(1)(5)#283 <= 15
 CustomerInventoryCapacity(11,1,5): I_customer(11)(1)(5)#313 <= 92
 CustomerInventoryCapacity(12,1,5): I_customer(12)(1)(5)#343 <= 88
 CustomerInventoryCapacity(3,2,5): I_customer(3)(2)(5)#83 <= 8
 CustomerInventoryCapacity(4,2,5): I_customer(4)(2)(5)#113 <= 28
 CustomerInventoryCapacity(5,2,5): I_customer(5)(2)(5)#143 <= 33
 CustomerInventoryCapacity(6,2,5): I_customer(6)(2)(5)#173 <= 7
 CustomerInventoryCapacity(7,2,5): I_customer(7)(2)(5)#203 <= 27
 CustomerInventoryCapacity(8,2,5): I_customer(8)(2)(5)#233 <= 82
 CustomerInventoryCapacity(9,2,5): I_customer(9)(2)(5)#263 <= 106
 CustomerInventoryCapacity(10,2,5): I_customer(10)(2)(5)#293 <= 13
 CustomerInventoryCapacity(11,2,5): I_customer(11)(2)(5)#323 <= 93
 CustomerInventoryCapacity(12,2,5): I_customer(12)(2)(5)#353 <= 91
 CustomerInventoryCapacity(3,3,5): I_customer(3)(3)(5)#93 <= 11
 CustomerInventoryCapacity(4,3,5): I_customer(4)(3)(5)#123 <= 31
 CustomerInventoryCapacity(5,3,5): I_customer(5)(3)(5)#153 <= 31
 CustomerInventoryCapacity(6,3,5): I_customer(6)(3)(5)#183 <= 6
 CustomerInventoryCapacity(7,3,5): I_customer(7)(3)(5)#213 <= 27
 CustomerInventoryCapacity(8,3,5): I_customer(8)(3)(5)#243 <= 82
 CustomerInventoryCapacity(9,3,5): I_customer(9)(3)(5)#273 <= 108
 CustomerInventoryCapacity(10,3,5): I_customer(10)(3)(5)#303 <= 15
 CustomerInventoryCapacity(11,3,5): I_customer(11)(3)(5)#333 <= 98
 CustomerInventoryCapacity(12,3,5): I_customer(12)(3)(5)#363 <= 91
 CustomerInventoryCapacity(3,1,6): I_customer(3)(1)(6)#74 <= 8
 CustomerInventoryCapacity(4,1,6): I_customer(4)(1)(6)#104 <= 32
 CustomerInventoryCapacity(5,1,6): I_customer(5)(1)(6)#134 <= 31
 CustomerInventoryCapacity(6,1,6): I_customer(6)(1)(6)#164 <= 6
 CustomerInventoryCapacity(7,1,6): I_customer(7)(1)(6)#194 <= 25
 CustomerInventoryCapacity(8,1,6): I_customer(8)(1)(6)#224 <= 81
 CustomerInventoryCapacity(9,1,6): I_customer(9)(1)(6)#254 <= 106
 CustomerInventoryCapacity(10,1,6): I_customer(10)(1)(6)#284 <= 13
 CustomerInventoryCapacity(11,1,6): I_customer(11)(1)(6)#314 <= 97
 CustomerInventoryCapacity(12,1,6): I_customer(12)(1)(6)#344 <= 90
 CustomerInventoryCapacity(3,2,6): I_customer(3)(2)(6)#84 <= 10
 CustomerInventoryCapacity(4,2,6): I_customer(4)(2)(6)#114 <= 32
 CustomerInventoryCapacity(5,2,6): I_customer(5)(2)(6)#144 <= 28
 CustomerInventoryCapacity(6,2,6): I_customer(6)(2)(6)#174 <= 8
 CustomerInventoryCapacity(7,2,6): I_customer(7)(2)(6)#204 <= 28
 CustomerInventoryCapacity(8,2,6): I_customer(8)(2)(6)#234 <= 77
 CustomerInventoryCapacity(9,2,6): I_customer(9)(2)(6)#264 <= 113
 CustomerInventoryCapacity(10,2,6): I_customer(10)(2)(6)#294 <= 12
 CustomerInventoryCapacity(11,2,6): I_customer(11)(2)(6)#324 <= 98
 CustomerInventoryCapacity(12,2,6): I_customer(12)(2)(6)#354 <= 91
 CustomerInventoryCapacity(3,3,6): I_customer(3)(3)(6)#94 <= 11
 CustomerInventoryCapacity(4,3,6): I_customer(4)(3)(6)#124 <= 29
 CustomerInventoryCapacity(5,3,6): I_customer(5)(3)(6)#154 <= 32
 CustomerInventoryCapacity(6,3,6): I_customer(6)(3)(6)#184 <= 8
 CustomerInventoryCapacity(7,3,6): I_customer(7)(3)(6)#214 <= 25
 CustomerInventoryCapacity(8,3,6): I_customer(8)(3)(6)#244 <= 81
 CustomerInventoryCapacity(9,3,6): I_customer(9)(3)(6)#274 <= 106
 CustomerInventoryCapacity(10,3,6): I_customer(10)(3)(6)#304 <= 12
 CustomerInventoryCapacity(11,3,6): I_customer(11)(3)(6)#334 <= 98
 CustomerInventoryCapacity(12,3,6): I_customer(12)(3)(6)#364 <= 84
 CustomerInventoryCapacity(3,1,7): I_customer(3)(1)(7)#75 <= 8
 CustomerInventoryCapacity(4,1,7): I_customer(4)(1)(7)#105 <= 32
 CustomerInventoryCapacity(5,1,7): I_customer(5)(1)(7)#135 <= 30
 CustomerInventoryCapacity(6,1,7): I_customer(6)(1)(7)#165 <= 6
 CustomerInventoryCapacity(7,1,7): I_customer(7)(1)(7)#195 <= 27
 CustomerInventoryCapacity(8,1,7): I_customer(8)(1)(7)#225 <= 78
 CustomerInventoryCapacity(9,1,7): I_customer(9)(1)(7)#255 <= 114
 CustomerInventoryCapacity(10,1,7): I_customer(10)(1)(7)#285 <= 13
 CustomerInventoryCapacity(11,1,7): I_customer(11)(1)(7)#315 <= 94
 CustomerInventoryCapacity(12,1,7): I_customer(12)(1)(7)#345 <= 90
 CustomerInventoryCapacity(3,2,7): I_customer(3)(2)(7)#85 <= 9
 CustomerInventoryCapacity(4,2,7): I_customer(4)(2)(7)#115 <= 28
 CustomerInventoryCapacity(5,2,7): I_customer(5)(2)(7)#145 <= 32
 CustomerInventoryCapacity(6,2,7): I_customer(6)(2)(7)#175 <= 8
 CustomerInventoryCapacity(7,2,7): I_customer(7)(2)(7)#205 <= 26
 CustomerInventoryCapacity(8,2,7): I_customer(8)(2)(7)#235 <= 78
 CustomerInventoryCapacity(9,2,7): I_customer(9)(2)(7)#265 <= 111
 CustomerInventoryCapacity(10,2,7): I_customer(10)(2)(7)#295 <= 12
 CustomerInventoryCapacity(11,2,7): I_customer(11)(2)(7)#325 <= 94
 CustomerInventoryCapacity(12,2,7): I_customer(12)(2)(7)#355 <= 87
 CustomerInventoryCapacity(3,3,7): I_customer(3)(3)(7)#95 <= 10
 CustomerInventoryCapacity(4,3,7): I_customer(4)(3)(7)#125 <= 30
 CustomerInventoryCapacity(5,3,7): I_customer(5)(3)(7)#155 <= 33
 CustomerInventoryCapacity(6,3,7): I_customer(6)(3)(7)#185 <= 7
 CustomerInventoryCapacity(7,3,7): I_customer(7)(3)(7)#215 <= 25
 CustomerInventoryCapacity(8,3,7): I_customer(8)(3)(7)#245 <= 82
 CustomerInventoryCapacity(9,3,7): I_customer(9)(3)(7)#275 <= 106
 CustomerInventoryCapacity(10,3,7): I_customer(10)(3)(7)#305 <= 11
 CustomerInventoryCapacity(11,3,7): I_customer(11)(3)(7)#335 <= 93
 CustomerInventoryCapacity(12,3,7): I_customer(12)(3)(7)#365 <= 85
 CustomerInventoryCapacity(3,1,8): I_customer(3)(1)(8)#76 <= 10
 CustomerInventoryCapacity(4,1,8): I_customer(4)(1)(8)#106 <= 32
 CustomerInventoryCapacity(5,1,8): I_customer(5)(1)(8)#136 <= 32
 CustomerInventoryCapacity(6,1,8): I_customer(6)(1)(8)#166 <= 7
 CustomerInventoryCapacity(7,1,8): I_customer(7)(1)(8)#196 <= 24
 CustomerInventoryCapacity(8,1,8): I_customer(8)(1)(8)#226 <= 83
 CustomerInventoryCapacity(9,1,8): I_customer(9)(1)(8)#256 <= 111
 CustomerInventoryCapacity(10,1,8): I_customer(10)(1)(8)#286 <= 16
 CustomerInventoryCapacity(11,1,8): I_customer(11)(1)(8)#316 <= 92
 CustomerInventoryCapacity(12,1,8): I_customer(12)(1)(8)#346 <= 91
 CustomerInventoryCapacity(3,2,8): I_customer(3)(2)(8)#86 <= 10
 CustomerInventoryCapacity(4,2,8): I_customer(4)(2)(8)#116 <= 32
 CustomerInventoryCapacity(5,2,8): I_customer(5)(2)(8)#146 <= 32
 CustomerInventoryCapacity(6,2,8): I_customer(6)(2)(8)#176 <= 6
 CustomerInventoryCapacity(7,2,8): I_customer(7)(2)(8)#206 <= 26
 CustomerInventoryCapacity(8,2,8): I_customer(8)(2)(8)#236 <= 79
 CustomerInventoryCapacity(9,2,8): I_customer(9)(2)(8)#266 <= 112
 CustomerInventoryCapacity(10,2,8): I_customer(10)(2)(8)#296 <= 15
 CustomerInventoryCapacity(11,2,8): I_customer(11)(2)(8)#326 <= 97
 CustomerInventoryCapacity(12,2,8): I_customer(12)(2)(8)#356 <= 84
 CustomerInventoryCapacity(3,3,8): I_customer(3)(3)(8)#96 <= 12
 CustomerInventoryCapacity(4,3,8): I_customer(4)(3)(8)#126 <= 29
 CustomerInventoryCapacity(5,3,8): I_customer(5)(3)(8)#156 <= 28
 CustomerInventoryCapacity(6,3,8): I_customer(6)(3)(8)#186 <= 7
 CustomerInventoryCapacity(7,3,8): I_customer(7)(3)(8)#216 <= 26
 CustomerInventoryCapacity(8,3,8): I_customer(8)(3)(8)#246 <= 77
 CustomerInventoryCapacity(9,3,8): I_customer(9)(3)(8)#276 <= 107
 CustomerInventoryCapacity(10,3,8): I_customer(10)(3)(8)#306 <= 12
 CustomerInventoryCapacity(11,3,8): I_customer(11)(3)(8)#336 <= 95
 CustomerInventoryCapacity(12,3,8): I_customer(12)(3)(8)#366 <= 88
 CustomerInventoryCapacity(3,1,9): I_customer(3)(1)(9)#77 <= 8
 CustomerInventoryCapacity(4,1,9): I_customer(4)(1)(9)#107 <= 27
 CustomerInventoryCapacity(5,1,9): I_customer(5)(1)(9)#137 <= 32
 CustomerInventoryCapacity(6,1,9): I_customer(6)(1)(9)#167 <= 7
 CustomerInventoryCapacity(7,1,9): I_customer(7)(1)(9)#197 <= 25
 CustomerInventoryCapacity(8,1,9): I_customer(8)(1)(9)#227 <= 79
 CustomerInventoryCapacity(9,1,9): I_customer(9)(1)(9)#257 <= 108
 CustomerInventoryCapacity(10,1,9): I_customer(10)(1)(9)#287 <= 15
 CustomerInventoryCapacity(11,1,9): I_customer(11)(1)(9)#317 <= 93
 CustomerInventoryCapacity(12,1,9): I_customer(12)(1)(9)#347 <= 92
 CustomerInventoryCapacity(3,2,9): I_customer(3)(2)(9)#87 <= 10
 CustomerInventoryCapacity(4,2,9): I_customer(4)(2)(9)#117 <= 30
 CustomerInventoryCapacity(5,2,9): I_customer(5)(2)(9)#147 <= 28
 CustomerInventoryCapacity(6,2,9): I_customer(6)(2)(9)#177 <= 7
 CustomerInventoryCapacity(7,2,9): I_customer(7)(2)(9)#207 <= 26
 CustomerInventoryCapacity(8,2,9): I_customer(8)(2)(9)#237 <= 81
 CustomerInventoryCapacity(9,2,9): I_customer(9)(2)(9)#267 <= 107
 CustomerInventoryCapacity(10,2,9): I_customer(10)(2)(9)#297 <= 15
 CustomerInventoryCapacity(11,2,9): I_customer(11)(2)(9)#327 <= 99
 CustomerInventoryCapacity(12,2,9): I_customer(12)(2)(9)#357 <= 85
 CustomerInventoryCapacity(3,3,9): I_customer(3)(3)(9)#97 <= 12
 CustomerInventoryCapacity(4,3,9): I_customer(4)(3)(9)#127 <= 27
 CustomerInventoryCapacity(5,3,9): I_customer(5)(3)(9)#157 <= 32
 CustomerInventoryCapacity(6,3,9): I_customer(6)(3)(9)#187 <= 8
 CustomerInventoryCapacity(7,3,9): I_customer(7)(3)(9)#217 <= 26
 CustomerInventoryCapacity(8,3,9): I_customer(8)(3)(9)#247 <= 83
 CustomerInventoryCapacity(9,3,9): I_customer(9)(3)(9)#277 <= 108
 CustomerInventoryCapacity(10,3,9): I_customer(10)(3)(9)#307 <= 11
 CustomerInventoryCapacity(11,3,9): I_customer(11)(3)(9)#337 <= 95
 CustomerInventoryCapacity(12,3,9): I_customer(12)(3)(9)#367 <= 89
 CustomerInventoryCapacity(3,1,10): I_customer(3)(1)(10)#78 <= 9
 CustomerInventoryCapacity(4,1,10): I_customer(4)(1)(10)#108 <= 31
 CustomerInventoryCapacity(5,1,10): I_customer(5)(1)(10)#138 <= 30
 CustomerInventoryCapacity(6,1,10): I_customer(6)(1)(10)#168 <= 7
 CustomerInventoryCapacity(7,1,10): I_customer(7)(1)(10)#198 <= 24
 CustomerInventoryCapacity(8,1,10): I_customer(8)(1)(10)#228 <= 80
 CustomerInventoryCapacity(9,1,10): I_customer(9)(1)(10)#258 <= 112
 CustomerInventoryCapacity(10,1,10): I_customer(10)(1)(10)#288 <= 15
 CustomerInventoryCapacity(11,1,10): I_customer(11)(1)(10)#318 <= 95
 CustomerInventoryCapacity(12,1,10): I_customer(12)(1)(10)#348 <= 92
 CustomerInventoryCapacity(3,2,10): I_customer(3)(2)(10)#88 <= 10
 CustomerInventoryCapacity(4,2,10): I_customer(4)(2)(10)#118 <= 31
 CustomerInventoryCapacity(5,2,10): I_customer(5)(2)(10)#148 <= 32
 CustomerInventoryCapacity(6,2,10): I_customer(6)(2)(10)#178 <= 6
 CustomerInventoryCapacity(7,2,10): I_customer(7)(2)(10)#208 <= 26
 CustomerInventoryCapacity(8,2,10): I_customer(8)(2)(10)#238 <= 81
 CustomerInventoryCapacity(9,2,10): I_customer(9)(2)(10)#268 <= 107
 CustomerInventoryCapacity(10,2,10): I_customer(10)(2)(10)#298 <= 13
 CustomerInventoryCapacity(11,2,10): I_customer(11)(2)(10)#328 <= 96
 CustomerInventoryCapacity(12,2,10): I_customer(12)(2)(10)#358 <= 92
 CustomerInventoryCapacity(3,3,10): I_customer(3)(3)(10)#98 <= 9
 CustomerInventoryCapacity(4,3,10): I_customer(4)(3)(10)#128 <= 31
 CustomerInventoryCapacity(5,3,10): I_customer(5)(3)(10)#158 <= 29
 CustomerInventoryCapacity(6,3,10): I_customer(6)(3)(10)#188 <= 6
 CustomerInventoryCapacity(7,3,10): I_customer(7)(3)(10)#218 <= 27
 CustomerInventoryCapacity(8,3,10): I_customer(8)(3)(10)#248 <= 81
 CustomerInventoryCapacity(9,3,10): I_customer(9)(3)(10)#278 <= 107
 CustomerInventoryCapacity(10,3,10): I_customer(10)(3)(10)#308 <= 13
 CustomerInventoryCapacity(11,3,10): I_customer(11)(3)(10)#338 <= 97
 CustomerInventoryCapacity(12,3,10): I_customer(12)(3)(10)#368 <= 85
 WarehouseCoverage(1,1):         o(1)(1)#687 + o(3)(1)#693 <= 1
 WarehouseCoverage(2,1):         o(2)(1)#690 + o(3)(1)#693 <= 1
 WarehouseCoverage(1,2):         o(1)(2)#688 + o(3)(2)#694 <= 1
 WarehouseCoverage(2,2):         o(2)(2)#691 + o(3)(2)#694 <= 1
 WarehouseCoverage(1,3):         o(1)(3)#689 + o(3)(3)#695 <= 1
 WarehouseCoverage(2,3):         o(2)(3)#692 + o(3)(3)#695 <= 1
 VehicleCapacityPlant(1,1):      q(1)(1)(1)#669 + q(1)(2)(1)#672
                                 - 102 o(1)(1)#687 <= 0
 VehicleCapacityPlant(2,1):      q(2)(1)(1)#675 + q(2)(2)(1)#678
                                 - 102 o(2)(1)#690 <= 0
 VehicleCapacityPlant(3,1):      q(3)(1)(1)#681 + q(3)(2)(1)#684
                                 - 102 o(3)(1)#693 <= 0
 VehicleCapacityPlant(1,2):      q(1)(1)(2)#670 + q(1)(2)(2)#673
                                 - 102 o(1)(2)#688 <= 0
 VehicleCapacityPlant(2,2):      q(2)(1)(2)#676 + q(2)(2)(2)#679
                                 - 102 o(2)(2)#691 <= 0
 VehicleCapacityPlant(3,2):      q(3)(1)(2)#682 + q(3)(2)(2)#685
                                 - 102 o(3)(2)#694 <= 0
 VehicleCapacityPlant(1,3):      q(1)(1)(3)#671 + q(1)(2)(3)#674
                                 - 102 o(1)(3)#689 <= 0
 VehicleCapacityPlant(2,3):      q(2)(1)(3)#677 + q(2)(2)(3)#680
                                 - 102 o(2)(3)#692 <= 0
 VehicleCapacityPlant(3,3):      q(3)(1)(3)#683 + q(3)(2)(3)#686
                                 - 102 o(3)(3)#695 <= 0
 FleetSize(1):                   o(1)(1)#687 + o(2)(1)#690 + o(3)(1)#693 <= 2
 FleetSize(2):                   o(1)(2)#688 + o(2)(2)#691 + o(3)(2)#694 <= 2
 FleetSize(3):                   o(1)(3)#689 + o(2)(3)#692 + o(3)(3)#695 <= 2
 CustomerVisit(3,1,1,1):         w(3)(1)(1)(1)#696  = 0
 CustomerVisit(4,1,1,1):         w(4)(1)(1)(1)#876  = 0
 CustomerVisit(5,1,1,1):         w(5)(1)(1)(1)#1056  = 0
 CustomerVisit(6,1,1,1):         w(6)(1)(1)(1)#1236  = 0
 CustomerVisit(7,1,1,1):         w(7)(1)(1)(1)#1416  = 0
 CustomerVisit(8,1,1,1):         w(8)(1)(1)(1)#1596  = 0
 CustomerVisit(9,1,1,1):         w(9)(1)(1)(1)#1776  = 0
 CustomerVisit(10,1,1,1):        w(10)(1)(1)(1)#1956  = 0
 CustomerVisit(11,1,1,1):        w(11)(1)(1)(1)#2136  = 0
 CustomerVisit(12,1,1,1):        w(12)(1)(1)(1)#2316  = 0
 CustomerVisit(3,2,1,1):         w(3)(2)(1)(1)#726  = 0
 CustomerVisit(4,2,1,1):         w(4)(2)(1)(1)#906  = 0
 CustomerVisit(5,2,1,1):         w(5)(2)(1)(1)#1086  = 0
 CustomerVisit(6,2,1,1):         w(6)(2)(1)(1)#1266  = 0
 CustomerVisit(7,2,1,1):         w(7)(2)(1)(1)#1446  = 0
 CustomerVisit(8,2,1,1):         w(8)(2)(1)(1)#1626  = 0
 CustomerVisit(9,2,1,1):         w(9)(2)(1)(1)#1806  = 0
 CustomerVisit(10,2,1,1):        w(10)(2)(1)(1)#1986  = 0
 CustomerVisit(11,2,1,1):        w(11)(2)(1)(1)#2166  = 0
 CustomerVisit(12,2,1,1):        w(12)(2)(1)(1)#2346  = 0
 CustomerVisit(3,3,1,1):         w(3)(3)(1)(1)#756  = 0
 CustomerVisit(4,3,1,1):         w(4)(3)(1)(1)#936  = 0
 CustomerVisit(5,3,1,1):         w(5)(3)(1)(1)#1116  = 0
 CustomerVisit(6,3,1,1):         w(6)(3)(1)(1)#1296  = 0
 CustomerVisit(7,3,1,1):         w(7)(3)(1)(1)#1476  = 0
 CustomerVisit(8,3,1,1):         w(8)(3)(1)(1)#1656  = 0
 CustomerVisit(9,3,1,1):         w(9)(3)(1)(1)#1836  = 0
 CustomerVisit(10,3,1,1):        w(10)(3)(1)(1)#2016  = 0
 CustomerVisit(11,3,1,1):        w(11)(3)(1)(1)#2196  = 0
 CustomerVisit(12,3,1,1):        w(12)(3)(1)(1)#2376  = 0
 CustomerVisit(3,1,2,1):         w(3)(1)(2)(1)#706  = 0
 CustomerVisit(4,1,2,1):         w(4)(1)(2)(1)#886  = 0
 CustomerVisit(5,1,2,1):         w(5)(1)(2)(1)#1066 <= 31
 CustomerVisit(6,1,2,1):         w(6)(1)(2)(1)#1246  = 0
 CustomerVisit(7,1,2,1):         w(7)(1)(2)(1)#1426  = 0
 CustomerVisit(8,1,2,1):         w(8)(1)(2)(1)#1606  = 0
 CustomerVisit(9,1,2,1):         w(9)(1)(2)(1)#1786  = 0
 CustomerVisit(10,1,2,1):        w(10)(1)(2)(1)#1966  = 0
 CustomerVisit(11,1,2,1):        w(11)(1)(2)(1)#2146  = 0
 CustomerVisit(12,1,2,1):        w(12)(1)(2)(1)#2326  = 0
 CustomerVisit(3,2,2,1):         w(3)(2)(2)(1)#736  = 0
 CustomerVisit(4,2,2,1):         w(4)(2)(2)(1)#916  = 0
 CustomerVisit(5,2,2,1):         w(5)(2)(2)(1)#1096  = 0
 CustomerVisit(6,2,2,1):         w(6)(2)(2)(1)#1276  = 0
 CustomerVisit(7,2,2,1):         w(7)(2)(2)(1)#1456  = 0
 CustomerVisit(8,2,2,1):         w(8)(2)(2)(1)#1636  = 0
 CustomerVisit(9,2,2,1):         w(9)(2)(2)(1)#1816  = 0
 CustomerVisit(10,2,2,1):        w(10)(2)(2)(1)#1996  = 0
 CustomerVisit(11,2,2,1):        w(11)(2)(2)(1)#2176  = 0
 CustomerVisit(12,2,2,1):        w(12)(2)(2)(1)#2356  = 0
 CustomerVisit(3,3,2,1):         w(3)(3)(2)(1)#766 <= 20
 CustomerVisit(4,3,2,1):         w(4)(3)(2)(1)#946  = 0
 CustomerVisit(5,3,2,1):         w(5)(3)(2)(1)#1126  = 0
 CustomerVisit(6,3,2,1):         w(6)(3)(2)(1)#1306 <= 14
 CustomerVisit(7,3,2,1):         w(7)(3)(2)(1)#1486  = 0
 CustomerVisit(8,3,2,1):         w(8)(3)(2)(1)#1666  = 0
 CustomerVisit(9,3,2,1):         w(9)(3)(2)(1)#1846  = 0
 CustomerVisit(10,3,2,1):        w(10)(3)(2)(1)#2026 <= 24
 CustomerVisit(11,3,2,1):        w(11)(3)(2)(1)#2206 <= 32
 CustomerVisit(12,3,2,1):        w(12)(3)(2)(1)#2386 <= 44
 CustomerVisit(3,1,3,1):         w(3)(1)(3)(1)#716  = 0
 CustomerVisit(4,1,3,1):         w(4)(1)(3)(1)#896  = 0
 CustomerVisit(5,1,3,1):         w(5)(1)(3)(1)#1076  = 0
 CustomerVisit(6,1,3,1):         w(6)(1)(3)(1)#1256  = 0
 CustomerVisit(7,1,3,1):         w(7)(1)(3)(1)#1436  = 0
 CustomerVisit(8,1,3,1):         w(8)(1)(3)(1)#1616  = 0
 CustomerVisit(9,1,3,1):         w(9)(1)(3)(1)#1796 <= 24
 CustomerVisit(10,1,3,1):        w(10)(1)(3)(1)#1976  = 0
 CustomerVisit(11,1,3,1):        w(11)(1)(3)(1)#2156  = 0
 CustomerVisit(12,1,3,1):        w(12)(1)(3)(1)#2336 <= 25
 CustomerVisit(3,2,3,1):         w(3)(2)(3)(1)#746  = 0
 CustomerVisit(4,2,3,1):         w(4)(2)(3)(1)#926  = 0
 CustomerVisit(5,2,3,1):         w(5)(2)(3)(1)#1106 <= 16
 CustomerVisit(6,2,3,1):         w(6)(2)(3)(1)#1286 <= 7
 CustomerVisit(7,2,3,1):         w(7)(2)(3)(1)#1466  = 0
 CustomerVisit(8,2,3,1):         w(8)(2)(3)(1)#1646 <= 17
 CustomerVisit(9,2,3,1):         w(9)(2)(3)(1)#1826  = 0
 CustomerVisit(10,2,3,1):        w(10)(2)(3)(1)#2006 <= 13
 CustomerVisit(11,2,3,1):        w(11)(2)(3)(1)#2186  = 0
 CustomerVisit(12,2,3,1):        w(12)(2)(3)(1)#2366  = 0
 CustomerVisit(3,3,3,1):         w(3)(3)(3)(1)#776  = 0
 CustomerVisit(4,3,3,1):         w(4)(3)(3)(1)#956  = 0
 CustomerVisit(5,3,3,1):         w(5)(3)(3)(1)#1136  = 0
 CustomerVisit(6,3,3,1):         w(6)(3)(3)(1)#1316  = 0
 CustomerVisit(7,3,3,1):         w(7)(3)(3)(1)#1496  = 0
 CustomerVisit(8,3,3,1):         w(8)(3)(3)(1)#1676  = 0
 CustomerVisit(9,3,3,1):         w(9)(3)(3)(1)#1856  = 0
 CustomerVisit(10,3,3,1):        w(10)(3)(3)(1)#2036  = 0
 CustomerVisit(11,3,3,1):        w(11)(3)(3)(1)#2216  = 0
 CustomerVisit(12,3,3,1):        w(12)(3)(3)(1)#2396  = 0
 CustomerVisit(3,4,1,1):         w(3)(4)(1)(1)#786  = 0
 CustomerVisit(4,4,1,1):         w(4)(4)(1)(1)#966  = 0
 CustomerVisit(5,4,1,1):         w(5)(4)(1)(1)#1146  = 0
 CustomerVisit(6,4,1,1):         w(6)(4)(1)(1)#1326  = 0
 CustomerVisit(7,4,1,1):         w(7)(4)(1)(1)#1506  = 0
 CustomerVisit(8,4,1,1):         w(8)(4)(1)(1)#1686  = 0
 CustomerVisit(9,4,1,1):         w(9)(4)(1)(1)#1866  = 0
 CustomerVisit(10,4,1,1):        w(10)(4)(1)(1)#2046  = 0
 CustomerVisit(11,4,1,1):        w(11)(4)(1)(1)#2226  = 0
 CustomerVisit(12,4,1,1):        w(12)(4)(1)(1)#2406  = 0
 CustomerVisit(3,5,1,1):         w(3)(5)(1)(1)#816 <= 20
 CustomerVisit(4,5,1,1):         w(4)(5)(1)(1)#996 <= 44
 CustomerVisit(5,5,1,1):         w(5)(5)(1)(1)#1176  = 0
 CustomerVisit(6,5,1,1):         w(6)(5)(1)(1)#1356 <= 14
 CustomerVisit(7,5,1,1):         w(7)(5)(1)(1)#1536 <= 39
 CustomerVisit(8,5,1,1):         w(8)(5)(1)(1)#1716  = 0
 CustomerVisit(9,5,1,1):         w(9)(5)(1)(1)#1896  = 0
 CustomerVisit(10,5,1,1):        w(10)(5)(1)(1)#2076 <= 26
 CustomerVisit(11,5,1,1):        w(11)(5)(1)(1)#2256 <= 44
 CustomerVisit(12,5,1,1):        w(12)(5)(1)(1)#2436  = 0
 CustomerVisit(3,6,1,1):         w(3)(6)(1)(1)#846  = 0
 CustomerVisit(4,6,1,1):         w(4)(6)(1)(1)#1026  = 0
 CustomerVisit(5,6,1,1):         w(5)(6)(1)(1)#1206  = 0
 CustomerVisit(6,6,1,1):         w(6)(6)(1)(1)#1386  = 0
 CustomerVisit(7,6,1,1):         w(7)(6)(1)(1)#1566  = 0
 CustomerVisit(8,6,1,1):         w(8)(6)(1)(1)#1746  = 0
 CustomerVisit(9,6,1,1):         w(9)(6)(1)(1)#1926  = 0
 CustomerVisit(10,6,1,1):        w(10)(6)(1)(1)#2106  = 0
 CustomerVisit(11,6,1,1):        w(11)(6)(1)(1)#2286  = 0
 CustomerVisit(12,6,1,1):        w(12)(6)(1)(1)#2466  = 0
 CustomerVisit(3,4,2,1):         w(3)(4)(2)(1)#796  = 0
 CustomerVisit(4,4,2,1):         w(4)(4)(2)(1)#976  = 0
 CustomerVisit(5,4,2,1):         w(5)(4)(2)(1)#1156  = 0
 CustomerVisit(6,4,2,1):         w(6)(4)(2)(1)#1336  = 0
 CustomerVisit(7,4,2,1):         w(7)(4)(2)(1)#1516 <= 27
 CustomerVisit(8,4,2,1):         w(8)(4)(2)(1)#1696  = 0
 CustomerVisit(9,4,2,1):         w(9)(4)(2)(1)#1876  = 0
 CustomerVisit(10,4,2,1):        w(10)(4)(2)(1)#2056  = 0
 CustomerVisit(11,4,2,1):        w(11)(4)(2)(1)#2236  = 0
 CustomerVisit(12,4,2,1):        w(12)(4)(2)(1)#2416  = 0
 CustomerVisit(3,5,2,1):         w(3)(5)(2)(1)#826  = 0
 CustomerVisit(4,5,2,1):         w(4)(5)(2)(1)#1006  = 0
 CustomerVisit(5,5,2,1):         w(5)(5)(2)(1)#1186  = 0
 CustomerVisit(6,5,2,1):         w(6)(5)(2)(1)#1366  = 0
 CustomerVisit(7,5,2,1):         w(7)(5)(2)(1)#1546  = 0
 CustomerVisit(8,5,2,1):         w(8)(5)(2)(1)#1726  = 0
 CustomerVisit(9,5,2,1):         w(9)(5)(2)(1)#1906  = 0
 CustomerVisit(10,5,2,1):        w(10)(5)(2)(1)#2086  = 0
 CustomerVisit(11,5,2,1):        w(11)(5)(2)(1)#2266  = 0
 CustomerVisit(12,5,2,1):        w(12)(5)(2)(1)#2446  = 0
 CustomerVisit(3,6,2,1):         w(3)(6)(2)(1)#856  = 0
 CustomerVisit(4,6,2,1):         w(4)(6)(2)(1)#1036  = 0
 CustomerVisit(5,6,2,1):         w(5)(6)(2)(1)#1216  = 0
 CustomerVisit(6,6,2,1):         w(6)(6)(2)(1)#1396  = 0
 CustomerVisit(7,6,2,1):         w(7)(6)(2)(1)#1576  = 0
 CustomerVisit(8,6,2,1):         w(8)(6)(2)(1)#1756  = 0
 CustomerVisit(9,6,2,1):         w(9)(6)(2)(1)#1936  = 0
 CustomerVisit(10,6,2,1):        w(10)(6)(2)(1)#2116  = 0
 CustomerVisit(11,6,2,1):        w(11)(6)(2)(1)#2296  = 0
 CustomerVisit(12,6,2,1):        w(12)(6)(2)(1)#2476  = 0
 CustomerVisit(3,4,3,1):         w(3)(4)(3)(1)#806 <= 12
 CustomerVisit(4,4,3,1):         w(4)(4)(3)(1)#986 <= 17
 CustomerVisit(5,4,3,1):         w(5)(4)(3)(1)#1166  = 0
 CustomerVisit(6,4,3,1):         w(6)(4)(3)(1)#1346  = 0
 CustomerVisit(7,4,3,1):         w(7)(4)(3)(1)#1526  = 0
 CustomerVisit(8,4,3,1):         w(8)(4)(3)(1)#1706  = 0
 CustomerVisit(9,4,3,1):         w(9)(4)(3)(1)#1886  = 0
 CustomerVisit(10,4,3,1):        w(10)(4)(3)(1)#2066  = 0
 CustomerVisit(11,4,3,1):        w(11)(4)(3)(1)#2246  = 0
 CustomerVisit(12,4,3,1):        w(12)(4)(3)(1)#2426  = 0
 CustomerVisit(3,5,3,1):         w(3)(5)(3)(1)#836  = 0
 CustomerVisit(4,5,3,1):         w(4)(5)(3)(1)#1016  = 0
 CustomerVisit(5,5,3,1):         w(5)(5)(3)(1)#1196  = 0
 CustomerVisit(6,5,3,1):         w(6)(5)(3)(1)#1376  = 0
 CustomerVisit(7,5,3,1):         w(7)(5)(3)(1)#1556 <= 12
 CustomerVisit(8,5,3,1):         w(8)(5)(3)(1)#1736  = 0
 CustomerVisit(9,5,3,1):         w(9)(5)(3)(1)#1916  = 0
 CustomerVisit(10,5,3,1):        w(10)(5)(3)(1)#2096  = 0
 CustomerVisit(11,5,3,1):        w(11)(5)(3)(1)#2276  = 0
 CustomerVisit(12,5,3,1):        w(12)(5)(3)(1)#2456  = 0
 CustomerVisit(3,6,3,1):         w(3)(6)(3)(1)#866  = 0
 CustomerVisit(4,6,3,1):         w(4)(6)(3)(1)#1046  = 0
 CustomerVisit(5,6,3,1):         w(5)(6)(3)(1)#1226  = 0
 CustomerVisit(6,6,3,1):         w(6)(6)(3)(1)#1406  = 0
 CustomerVisit(7,6,3,1):         w(7)(6)(3)(1)#1586  = 0
 CustomerVisit(8,6,3,1):         w(8)(6)(3)(1)#1766  = 0
 CustomerVisit(9,6,3,1):         w(9)(6)(3)(1)#1946  = 0
 CustomerVisit(10,6,3,1):        w(10)(6)(3)(1)#2126  = 0
 CustomerVisit(11,6,3,1):        w(11)(6)(3)(1)#2306  = 0
 CustomerVisit(12,6,3,1):        w(12)(6)(3)(1)#2486  = 0
 CustomerVisit(3,1,1,2):         w(3)(1)(1)(2)#697  = 0
 CustomerVisit(4,1,1,2):         w(4)(1)(1)(2)#877  = 0
 CustomerVisit(5,1,1,2):         w(5)(1)(1)(2)#1057  = 0
 CustomerVisit(6,1,1,2):         w(6)(1)(1)(2)#1237  = 0
 CustomerVisit(7,1,1,2):         w(7)(1)(1)(2)#1417  = 0
 CustomerVisit(8,1,1,2):         w(8)(1)(1)(2)#1597  = 0
 CustomerVisit(9,1,1,2):         w(9)(1)(1)(2)#1777  = 0
 CustomerVisit(10,1,1,2):        w(10)(1)(1)(2)#1957  = 0
 CustomerVisit(11,1,1,2):        w(11)(1)(1)(2)#2137  = 0
 CustomerVisit(12,1,1,2):        w(12)(1)(1)(2)#2317  = 0
 CustomerVisit(3,2,1,2):         w(3)(2)(1)(2)#727  = 0
 CustomerVisit(4,2,1,2):         w(4)(2)(1)(2)#907  = 0
 CustomerVisit(5,2,1,2):         w(5)(2)(1)(2)#1087  = 0
 CustomerVisit(6,2,1,2):         w(6)(2)(1)(2)#1267 <= 14
 CustomerVisit(7,2,1,2):         w(7)(2)(1)(2)#1447  = 0
 CustomerVisit(8,2,1,2):         w(8)(2)(1)(2)#1627  = 0
 CustomerVisit(9,2,1,2):         w(9)(2)(1)(2)#1807 <= 44
 CustomerVisit(10,2,1,2):        w(10)(2)(1)(2)#1987 <= 26
 CustomerVisit(11,2,1,2):        w(11)(2)(1)(2)#2167  = 0
 CustomerVisit(12,2,1,2):        w(12)(2)(1)(2)#2347 <= 44
 CustomerVisit(3,3,1,2):         w(3)(3)(1)(2)#757  = 0
 CustomerVisit(4,3,1,2):         w(4)(3)(1)(2)#937  = 0
 CustomerVisit(5,3,1,2):         w(5)(3)(1)(2)#1117  = 0
 CustomerVisit(6,3,1,2):         w(6)(3)(1)(2)#1297  = 0
 CustomerVisit(7,3,1,2):         w(7)(3)(1)(2)#1477  = 0
 CustomerVisit(8,3,1,2):         w(8)(3)(1)(2)#1657  = 0
 CustomerVisit(9,3,1,2):         w(9)(3)(1)(2)#1837  = 0
 CustomerVisit(10,3,1,2):        w(10)(3)(1)(2)#2017  = 0
 CustomerVisit(11,3,1,2):        w(11)(3)(1)(2)#2197  = 0
 CustomerVisit(12,3,1,2):        w(12)(3)(1)(2)#2377  = 0
 CustomerVisit(3,1,2,2):         w(3)(1)(2)(2)#707  = 0
 CustomerVisit(4,1,2,2):         w(4)(1)(2)(2)#887  = 0
 CustomerVisit(5,1,2,2):         w(5)(1)(2)(2)#1067 <= 34
 CustomerVisit(6,1,2,2):         w(6)(1)(2)(2)#1247  = 0
 CustomerVisit(7,1,2,2):         w(7)(1)(2)(2)#1427  = 0
 CustomerVisit(8,1,2,2):         w(8)(1)(2)(2)#1607  = 0
 CustomerVisit(9,1,2,2):         w(9)(1)(2)(2)#1787  = 0
 CustomerVisit(10,1,2,2):        w(10)(1)(2)(2)#1967  = 0
 CustomerVisit(11,1,2,2):        w(11)(1)(2)(2)#2147  = 0
 CustomerVisit(12,1,2,2):        w(12)(1)(2)(2)#2327  = 0
 CustomerVisit(3,2,2,2):         w(3)(2)(2)(2)#737 <= 20
 CustomerVisit(4,2,2,2):         w(4)(2)(2)(2)#917  = 0
 CustomerVisit(5,2,2,2):         w(5)(2)(2)(2)#1097  = 0
 CustomerVisit(6,2,2,2):         w(6)(2)(2)(2)#1277 <= 14
 CustomerVisit(7,2,2,2):         w(7)(2)(2)(2)#1457  = 0
 CustomerVisit(8,2,2,2):         w(8)(2)(2)(2)#1637  = 0
 CustomerVisit(9,2,2,2):         w(9)(2)(2)(2)#1817 <= 38
 CustomerVisit(10,2,2,2):        w(10)(2)(2)(2)#1997 <= 26
 CustomerVisit(11,2,2,2):        w(11)(2)(2)(2)#2177  = 0
 CustomerVisit(12,2,2,2):        w(12)(2)(2)(2)#2357 <= 44
 CustomerVisit(3,3,2,2):         w(3)(3)(2)(2)#767  = 0
 CustomerVisit(4,3,2,2):         w(4)(3)(2)(2)#947  = 0
 CustomerVisit(5,3,2,2):         w(5)(3)(2)(2)#1127  = 0
 CustomerVisit(6,3,2,2):         w(6)(3)(2)(2)#1307  = 0
 CustomerVisit(7,3,2,2):         w(7)(3)(2)(2)#1487  = 0
 CustomerVisit(8,3,2,2):         w(8)(3)(2)(2)#1667  = 0
 CustomerVisit(9,3,2,2):         w(9)(3)(2)(2)#1847  = 0
 CustomerVisit(10,3,2,2):        w(10)(3)(2)(2)#2027  = 0
 CustomerVisit(11,3,2,2):        w(11)(3)(2)(2)#2207  = 0
 CustomerVisit(12,3,2,2):        w(12)(3)(2)(2)#2387  = 0
 CustomerVisit(3,1,3,2):         w(3)(1)(3)(2)#717  = 0
 CustomerVisit(4,1,3,2):         w(4)(1)(3)(2)#897  = 0
 CustomerVisit(5,1,3,2):         w(5)(1)(3)(2)#1077 <= 17
 CustomerVisit(6,1,3,2):         w(6)(1)(3)(2)#1257  = 0
 CustomerVisit(7,1,3,2):         w(7)(1)(3)(2)#1437  = 0
 CustomerVisit(8,1,3,2):         w(8)(1)(3)(2)#1617  = 0
 CustomerVisit(9,1,3,2):         w(9)(1)(3)(2)#1797  = 0
 CustomerVisit(10,1,3,2):        w(10)(1)(3)(2)#1977  = 0
 CustomerVisit(11,1,3,2):        w(11)(1)(3)(2)#2157  = 0
 CustomerVisit(12,1,3,2):        w(12)(1)(3)(2)#2337  = 0
 CustomerVisit(3,2,3,2):         w(3)(2)(3)(2)#747  = 0
 CustomerVisit(4,2,3,2):         w(4)(2)(3)(2)#927  = 0
 CustomerVisit(5,2,3,2):         w(5)(2)(3)(2)#1107  = 0
 CustomerVisit(6,2,3,2):         w(6)(2)(3)(2)#1287 <= 8
 CustomerVisit(7,2,3,2):         w(7)(2)(3)(2)#1467  = 0
 CustomerVisit(8,2,3,2):         w(8)(2)(3)(2)#1647 <= 17
 CustomerVisit(9,2,3,2):         w(9)(2)(3)(2)#1827 <= 18
 CustomerVisit(10,2,3,2):        w(10)(2)(3)(2)#2007 <= 14
 CustomerVisit(11,2,3,2):        w(11)(2)(3)(2)#2187 <= 22
 CustomerVisit(12,2,3,2):        w(12)(2)(3)(2)#2367  = 0
 CustomerVisit(3,3,3,2):         w(3)(3)(3)(2)#777  = 0
 CustomerVisit(4,3,3,2):         w(4)(3)(3)(2)#957  = 0
 CustomerVisit(5,3,3,2):         w(5)(3)(3)(2)#1137  = 0
 CustomerVisit(6,3,3,2):         w(6)(3)(3)(2)#1317  = 0
 CustomerVisit(7,3,3,2):         w(7)(3)(3)(2)#1497  = 0
 CustomerVisit(8,3,3,2):         w(8)(3)(3)(2)#1677  = 0
 CustomerVisit(9,3,3,2):         w(9)(3)(3)(2)#1857  = 0
 CustomerVisit(10,3,3,2):        w(10)(3)(3)(2)#2037  = 0
 CustomerVisit(11,3,3,2):        w(11)(3)(3)(2)#2217  = 0
 CustomerVisit(12,3,3,2):        w(12)(3)(3)(2)#2397  = 0
 CustomerVisit(3,4,1,2):         w(3)(4)(1)(2)#787  = 0
 CustomerVisit(4,4,1,2):         w(4)(4)(1)(2)#967  = 0
 CustomerVisit(5,4,1,2):         w(5)(4)(1)(2)#1147  = 0
 CustomerVisit(6,4,1,2):         w(6)(4)(1)(2)#1327  = 0
 CustomerVisit(7,4,1,2):         w(7)(4)(1)(2)#1507  = 0
 CustomerVisit(8,4,1,2):         w(8)(4)(1)(2)#1687  = 0
 CustomerVisit(9,4,1,2):         w(9)(4)(1)(2)#1867  = 0
 CustomerVisit(10,4,1,2):        w(10)(4)(1)(2)#2047  = 0
 CustomerVisit(11,4,1,2):        w(11)(4)(1)(2)#2227  = 0
 CustomerVisit(12,4,1,2):        w(12)(4)(1)(2)#2407  = 0
 CustomerVisit(3,5,1,2):         w(3)(5)(1)(2)#817  = 0
 CustomerVisit(4,5,1,2):         w(4)(5)(1)(2)#997  = 0
 CustomerVisit(5,5,1,2):         w(5)(5)(1)(2)#1177  = 0
 CustomerVisit(6,5,1,2):         w(6)(5)(1)(2)#1357  = 0
 CustomerVisit(7,5,1,2):         w(7)(5)(1)(2)#1537  = 0
 CustomerVisit(8,5,1,2):         w(8)(5)(1)(2)#1717  = 0
 CustomerVisit(9,5,1,2):         w(9)(5)(1)(2)#1897  = 0
 CustomerVisit(10,5,1,2):        w(10)(5)(1)(2)#2077  = 0
 CustomerVisit(11,5,1,2):        w(11)(5)(1)(2)#2257  = 0
 CustomerVisit(12,5,1,2):        w(12)(5)(1)(2)#2437  = 0
 CustomerVisit(3,6,1,2):         w(3)(6)(1)(2)#847 <= 20
 CustomerVisit(4,6,1,2):         w(4)(6)(1)(2)#1027  = 0
 CustomerVisit(5,6,1,2):         w(5)(6)(1)(2)#1207  = 0
 CustomerVisit(6,6,1,2):         w(6)(6)(1)(2)#1387  = 0
 CustomerVisit(7,6,1,2):         w(7)(6)(1)(2)#1567 <= 35
 CustomerVisit(8,6,1,2):         w(8)(6)(1)(2)#1747  = 0
 CustomerVisit(9,6,1,2):         w(9)(6)(1)(2)#1927  = 0
 CustomerVisit(10,6,1,2):        w(10)(6)(1)(2)#2107  = 0
 CustomerVisit(11,6,1,2):        w(11)(6)(1)(2)#2287  = 0
 CustomerVisit(12,6,1,2):        w(12)(6)(1)(2)#2467  = 0
 CustomerVisit(3,4,2,2):         w(3)(4)(2)(2)#797  = 0
 CustomerVisit(4,4,2,2):         w(4)(4)(2)(2)#977  = 0
 CustomerVisit(5,4,2,2):         w(5)(4)(2)(2)#1157  = 0
 CustomerVisit(6,4,2,2):         w(6)(4)(2)(2)#1337  = 0
 CustomerVisit(7,4,2,2):         w(7)(4)(2)(2)#1517  = 0
 CustomerVisit(8,4,2,2):         w(8)(4)(2)(2)#1697  = 0
 CustomerVisit(9,4,2,2):         w(9)(4)(2)(2)#1877  = 0
 CustomerVisit(10,4,2,2):        w(10)(4)(2)(2)#2057  = 0
 CustomerVisit(11,4,2,2):        w(11)(4)(2)(2)#2237  = 0
 CustomerVisit(12,4,2,2):        w(12)(4)(2)(2)#2417  = 0
 CustomerVisit(3,5,2,2):         w(3)(5)(2)(2)#827  = 0
 CustomerVisit(4,5,2,2):         w(4)(5)(2)(2)#1007  = 0
 CustomerVisit(5,5,2,2):         w(5)(5)(2)(2)#1187  = 0
 CustomerVisit(6,5,2,2):         w(6)(5)(2)(2)#1367  = 0
 CustomerVisit(7,5,2,2):         w(7)(5)(2)(2)#1547  = 0
 CustomerVisit(8,5,2,2):         w(8)(5)(2)(2)#1727  = 0
 CustomerVisit(9,5,2,2):         w(9)(5)(2)(2)#1907  = 0
 CustomerVisit(10,5,2,2):        w(10)(5)(2)(2)#2087  = 0
 CustomerVisit(11,5,2,2):        w(11)(5)(2)(2)#2267  = 0
 CustomerVisit(12,5,2,2):        w(12)(5)(2)(2)#2447  = 0
 CustomerVisit(3,6,2,2):         w(3)(6)(2)(2)#857  = 0
 CustomerVisit(4,6,2,2):         w(4)(6)(2)(2)#1037 <= 30
 CustomerVisit(5,6,2,2):         w(5)(6)(2)(2)#1217  = 0
 CustomerVisit(6,6,2,2):         w(6)(6)(2)(2)#1397  = 0
 CustomerVisit(7,6,2,2):         w(7)(6)(2)(2)#1577  = 0
 CustomerVisit(8,6,2,2):         w(8)(6)(2)(2)#1757  = 0
 CustomerVisit(9,6,2,2):         w(9)(6)(2)(2)#1937  = 0
 CustomerVisit(10,6,2,2):        w(10)(6)(2)(2)#2117  = 0
 CustomerVisit(11,6,2,2):        w(11)(6)(2)(2)#2297  = 0
 CustomerVisit(12,6,2,2):        w(12)(6)(2)(2)#2477  = 0
 CustomerVisit(3,4,3,2):         w(3)(4)(3)(2)#807 <= 9
 CustomerVisit(4,4,3,2):         w(4)(4)(3)(2)#987 <= 14
 CustomerVisit(5,4,3,2):         w(5)(4)(3)(2)#1167  = 0
 CustomerVisit(6,4,3,2):         w(6)(4)(3)(2)#1347  = 0
 CustomerVisit(7,4,3,2):         w(7)(4)(3)(2)#1527  = 0
 CustomerVisit(8,4,3,2):         w(8)(4)(3)(2)#1707  = 0
 CustomerVisit(9,4,3,2):         w(9)(4)(3)(2)#1887  = 0
 CustomerVisit(10,4,3,2):        w(10)(4)(3)(2)#2067  = 0
 CustomerVisit(11,4,3,2):        w(11)(4)(3)(2)#2247  = 0
 CustomerVisit(12,4,3,2):        w(12)(4)(3)(2)#2427  = 0
 CustomerVisit(3,5,3,2):         w(3)(5)(3)(2)#837  = 0
 CustomerVisit(4,5,3,2):         w(4)(5)(3)(2)#1017  = 0
 CustomerVisit(5,5,3,2):         w(5)(5)(3)(2)#1197  = 0
 CustomerVisit(6,5,3,2):         w(6)(5)(3)(2)#1377  = 0
 CustomerVisit(7,5,3,2):         w(7)(5)(3)(2)#1557 <= 11
 CustomerVisit(8,5,3,2):         w(8)(5)(3)(2)#1737  = 0
 CustomerVisit(9,5,3,2):         w(9)(5)(3)(2)#1917  = 0
 CustomerVisit(10,5,3,2):        w(10)(5)(3)(2)#2097  = 0
 CustomerVisit(11,5,3,2):        w(11)(5)(3)(2)#2277  = 0
 CustomerVisit(12,5,3,2):        w(12)(5)(3)(2)#2457  = 0
 CustomerVisit(3,6,3,2):         w(3)(6)(3)(2)#867  = 0
 CustomerVisit(4,6,3,2):         w(4)(6)(3)(2)#1047  = 0
 CustomerVisit(5,6,3,2):         w(5)(6)(3)(2)#1227  = 0
 CustomerVisit(6,6,3,2):         w(6)(6)(3)(2)#1407  = 0
 CustomerVisit(7,6,3,2):         w(7)(6)(3)(2)#1587  = 0
 CustomerVisit(8,6,3,2):         w(8)(6)(3)(2)#1767  = 0
 CustomerVisit(9,6,3,2):         w(9)(6)(3)(2)#1947  = 0
 CustomerVisit(10,6,3,2):        w(10)(6)(3)(2)#2127  = 0
 CustomerVisit(11,6,3,2):        w(11)(6)(3)(2)#2307  = 0
 CustomerVisit(12,6,3,2):        w(12)(6)(3)(2)#2487  = 0
 CustomerVisit(3,1,1,3):         w(3)(1)(1)(3)#698  = 0
 CustomerVisit(4,1,1,3):         w(4)(1)(1)(3)#878  = 0
 CustomerVisit(5,1,1,3):         w(5)(1)(1)(3)#1058  = 0
 CustomerVisit(6,1,1,3):         w(6)(1)(1)(3)#1238  = 0
 CustomerVisit(7,1,1,3):         w(7)(1)(1)(3)#1418  = 0
 CustomerVisit(8,1,1,3):         w(8)(1)(1)(3)#1598  = 0
 CustomerVisit(9,1,1,3):         w(9)(1)(1)(3)#1778  = 0
 CustomerVisit(10,1,1,3):        w(10)(1)(1)(3)#1958  = 0
 CustomerVisit(11,1,1,3):        w(11)(1)(1)(3)#2138  = 0
 CustomerVisit(12,1,1,3):        w(12)(1)(1)(3)#2318  = 0
 CustomerVisit(3,2,1,3):         w(3)(2)(1)(3)#728 <= 20
 CustomerVisit(4,2,1,3):         w(4)(2)(1)(3)#908  = 0
 CustomerVisit(5,2,1,3):         w(5)(2)(1)(3)#1088 <= 44
 CustomerVisit(6,2,1,3):         w(6)(2)(1)(3)#1268 <= 14
 CustomerVisit(7,2,1,3):         w(7)(2)(1)(3)#1448  = 0
 CustomerVisit(8,2,1,3):         w(8)(2)(1)(3)#1628  = 0
 CustomerVisit(9,2,1,3):         w(9)(2)(1)(3)#1808 <= 44
 CustomerVisit(10,2,1,3):        w(10)(2)(1)(3)#1988 <= 26
 CustomerVisit(11,2,1,3):        w(11)(2)(1)(3)#2168 <= 44
 CustomerVisit(12,2,1,3):        w(12)(2)(1)(3)#2348  = 0
 CustomerVisit(3,3,1,3):         w(3)(3)(1)(3)#758  = 0
 CustomerVisit(4,3,1,3):         w(4)(3)(1)(3)#938  = 0
 CustomerVisit(5,3,1,3):         w(5)(3)(1)(3)#1118  = 0
 CustomerVisit(6,3,1,3):         w(6)(3)(1)(3)#1298  = 0
 CustomerVisit(7,3,1,3):         w(7)(3)(1)(3)#1478  = 0
 CustomerVisit(8,3,1,3):         w(8)(3)(1)(3)#1658  = 0
 CustomerVisit(9,3,1,3):         w(9)(3)(1)(3)#1838  = 0
 CustomerVisit(10,3,1,3):        w(10)(3)(1)(3)#2018  = 0
 CustomerVisit(11,3,1,3):        w(11)(3)(1)(3)#2198  = 0
 CustomerVisit(12,3,1,3):        w(12)(3)(1)(3)#2378 <= 44
 CustomerVisit(3,1,2,3):         w(3)(1)(2)(3)#708  = 0
 CustomerVisit(4,1,2,3):         w(4)(1)(2)(3)#888  = 0
 CustomerVisit(5,1,2,3):         w(5)(1)(2)(3)#1068 <= 31
 CustomerVisit(6,1,2,3):         w(6)(1)(2)(3)#1248  = 0
 CustomerVisit(7,1,2,3):         w(7)(1)(2)(3)#1428  = 0
 CustomerVisit(8,1,2,3):         w(8)(1)(2)(3)#1608  = 0
 CustomerVisit(9,1,2,3):         w(9)(1)(2)(3)#1788  = 0
 CustomerVisit(10,1,2,3):        w(10)(1)(2)(3)#1968  = 0
 CustomerVisit(11,1,2,3):        w(11)(1)(2)(3)#2148  = 0
 CustomerVisit(12,1,2,3):        w(12)(1)(2)(3)#2328  = 0
 CustomerVisit(3,2,2,3):         w(3)(2)(2)(3)#738  = 0
 CustomerVisit(4,2,2,3):         w(4)(2)(2)(3)#918  = 0
 CustomerVisit(5,2,2,3):         w(5)(2)(2)(3)#1098  = 0
 CustomerVisit(6,2,2,3):         w(6)(2)(2)(3)#1278  = 0
 CustomerVisit(7,2,2,3):         w(7)(2)(2)(3)#1458  = 0
 CustomerVisit(8,2,2,3):         w(8)(2)(2)(3)#1638  = 0
 CustomerVisit(9,2,2,3):         w(9)(2)(2)(3)#1818  = 0
 CustomerVisit(10,2,2,3):        w(10)(2)(2)(3)#1998  = 0
 CustomerVisit(11,2,2,3):        w(11)(2)(2)(3)#2178  = 0
 CustomerVisit(12,2,2,3):        w(12)(2)(2)(3)#2358  = 0
 CustomerVisit(3,3,2,3):         w(3)(3)(2)(3)#768  = 0
 CustomerVisit(4,3,2,3):         w(4)(3)(2)(3)#948  = 0
 CustomerVisit(5,3,2,3):         w(5)(3)(2)(3)#1128  = 0
 CustomerVisit(6,3,2,3):         w(6)(3)(2)(3)#1308 <= 13
 CustomerVisit(7,3,2,3):         w(7)(3)(2)(3)#1488  = 0
 CustomerVisit(8,3,2,3):         w(8)(3)(2)(3)#1668  = 0
 CustomerVisit(9,3,2,3):         w(9)(3)(2)(3)#1848  = 0
 CustomerVisit(10,3,2,3):        w(10)(3)(2)(3)#2028 <= 26
 CustomerVisit(11,3,2,3):        w(11)(3)(2)(3)#2208 <= 42
 CustomerVisit(12,3,2,3):        w(12)(3)(2)(3)#2388  = 0
 CustomerVisit(3,1,3,3):         w(3)(1)(3)(3)#718  = 0
 CustomerVisit(4,1,3,3):         w(4)(1)(3)(3)#898  = 0
 CustomerVisit(5,1,3,3):         w(5)(1)(3)(3)#1078  = 0
 CustomerVisit(6,1,3,3):         w(6)(1)(3)(3)#1258 <= 7
 CustomerVisit(7,1,3,3):         w(7)(1)(3)(3)#1438  = 0
 CustomerVisit(8,1,3,3):         w(8)(1)(3)(3)#1618  = 0
 CustomerVisit(9,1,3,3):         w(9)(1)(3)(3)#1798 <= 26
 CustomerVisit(10,1,3,3):        w(10)(1)(3)(3)#1978 <= 15
 CustomerVisit(11,1,3,3):        w(11)(1)(3)(3)#2158 <= 20
 CustomerVisit(12,1,3,3):        w(12)(1)(3)(3)#2338 <= 21
 CustomerVisit(3,2,3,3):         w(3)(2)(3)(3)#748  = 0
 CustomerVisit(4,2,3,3):         w(4)(2)(3)(3)#928  = 0
 CustomerVisit(5,2,3,3):         w(5)(2)(3)(3)#1108  = 0
 CustomerVisit(6,2,3,3):         w(6)(2)(3)(3)#1288  = 0
 CustomerVisit(7,2,3,3):         w(7)(2)(3)(3)#1468  = 0
 CustomerVisit(8,2,3,3):         w(8)(2)(3)(3)#1648  = 0
 CustomerVisit(9,2,3,3):         w(9)(2)(3)(3)#1828  = 0
 CustomerVisit(10,2,3,3):        w(10)(2)(3)(3)#2008  = 0
 CustomerVisit(11,2,3,3):        w(11)(2)(3)(3)#2188  = 0
 CustomerVisit(12,2,3,3):        w(12)(2)(3)(3)#2368  = 0
 CustomerVisit(3,3,3,3):         w(3)(3)(3)(3)#778  = 0
 CustomerVisit(4,3,3,3):         w(4)(3)(3)(3)#958  = 0
 CustomerVisit(5,3,3,3):         w(5)(3)(3)(3)#1138  = 0
 CustomerVisit(6,3,3,3):         w(6)(3)(3)(3)#1318  = 0
 CustomerVisit(7,3,3,3):         w(7)(3)(3)(3)#1498  = 0
 CustomerVisit(8,3,3,3):         w(8)(3)(3)(3)#1678  = 0
 CustomerVisit(9,3,3,3):         w(9)(3)(3)(3)#1858  = 0
 CustomerVisit(10,3,3,3):        w(10)(3)(3)(3)#2038  = 0
 CustomerVisit(11,3,3,3):        w(11)(3)(3)(3)#2218  = 0
 CustomerVisit(12,3,3,3):        w(12)(3)(3)(3)#2398  = 0
 CustomerVisit(3,4,1,3):         w(3)(4)(1)(3)#788  = 0
 CustomerVisit(4,4,1,3):         w(4)(4)(1)(3)#968  = 0
 CustomerVisit(5,4,1,3):         w(5)(4)(1)(3)#1148  = 0
 CustomerVisit(6,4,1,3):         w(6)(4)(1)(3)#1328  = 0
 CustomerVisit(7,4,1,3):         w(7)(4)(1)(3)#1508 <= 39
 CustomerVisit(8,4,1,3):         w(8)(4)(1)(3)#1688  = 0
 CustomerVisit(9,4,1,3):         w(9)(4)(1)(3)#1868  = 0
 CustomerVisit(10,4,1,3):        w(10)(4)(1)(3)#2048  = 0
 CustomerVisit(11,4,1,3):        w(11)(4)(1)(3)#2228  = 0
 CustomerVisit(12,4,1,3):        w(12)(4)(1)(3)#2408  = 0
 CustomerVisit(3,5,1,3):         w(3)(5)(1)(3)#818  = 0
 CustomerVisit(4,5,1,3):         w(4)(5)(1)(3)#998 <= 44
 CustomerVisit(5,5,1,3):         w(5)(5)(1)(3)#1178  = 0
 CustomerVisit(6,5,1,3):         w(6)(5)(1)(3)#1358  = 0
 CustomerVisit(7,5,1,3):         w(7)(5)(1)(3)#1538  = 0
 CustomerVisit(8,5,1,3):         w(8)(5)(1)(3)#1718 <= 44
 CustomerVisit(9,5,1,3):         w(9)(5)(1)(3)#1898  = 0
 CustomerVisit(10,5,1,3):        w(10)(5)(1)(3)#2078  = 0
 CustomerVisit(11,5,1,3):        w(11)(5)(1)(3)#2258  = 0
 CustomerVisit(12,5,1,3):        w(12)(5)(1)(3)#2438  = 0
 CustomerVisit(3,6,1,3):         w(3)(6)(1)(3)#848  = 0
 CustomerVisit(4,6,1,3):         w(4)(6)(1)(3)#1028  = 0
 CustomerVisit(5,6,1,3):         w(5)(6)(1)(3)#1208  = 0
 CustomerVisit(6,6,1,3):         w(6)(6)(1)(3)#1388  = 0
 CustomerVisit(7,6,1,3):         w(7)(6)(1)(3)#1568  = 0
 CustomerVisit(8,6,1,3):         w(8)(6)(1)(3)#1748  = 0
 CustomerVisit(9,6,1,3):         w(9)(6)(1)(3)#1928  = 0
 CustomerVisit(10,6,1,3):        w(10)(6)(1)(3)#2108  = 0
 CustomerVisit(11,6,1,3):        w(11)(6)(1)(3)#2288  = 0
 CustomerVisit(12,6,1,3):        w(12)(6)(1)(3)#2468  = 0
 CustomerVisit(3,4,2,3):         w(3)(4)(2)(3)#798 <= 20
 CustomerVisit(4,4,2,3):         w(4)(4)(2)(3)#978 <= 32
 CustomerVisit(5,4,2,3):         w(5)(4)(2)(3)#1158  = 0
 CustomerVisit(6,4,2,3):         w(6)(4)(2)(3)#1338  = 0
 CustomerVisit(7,4,2,3):         w(7)(4)(2)(3)#1518  = 0
 CustomerVisit(8,4,2,3):         w(8)(4)(2)(3)#1698  = 0
 CustomerVisit(9,4,2,3):         w(9)(4)(2)(3)#1878  = 0
 CustomerVisit(10,4,2,3):        w(10)(4)(2)(3)#2058  = 0
 CustomerVisit(11,4,2,3):        w(11)(4)(2)(3)#2238  = 0
 CustomerVisit(12,4,2,3):        w(12)(4)(2)(3)#2418  = 0
 CustomerVisit(3,5,2,3):         w(3)(5)(2)(3)#828  = 0
 CustomerVisit(4,5,2,3):         w(4)(5)(2)(3)#1008  = 0
 CustomerVisit(5,5,2,3):         w(5)(5)(2)(3)#1188  = 0
 CustomerVisit(6,5,2,3):         w(6)(5)(2)(3)#1368  = 0
 CustomerVisit(7,5,2,3):         w(7)(5)(2)(3)#1548  = 0
 CustomerVisit(8,5,2,3):         w(8)(5)(2)(3)#1728  = 0
 CustomerVisit(9,5,2,3):         w(9)(5)(2)(3)#1908  = 0
 CustomerVisit(10,5,2,3):        w(10)(5)(2)(3)#2088  = 0
 CustomerVisit(11,5,2,3):        w(11)(5)(2)(3)#2268  = 0
 CustomerVisit(12,5,2,3):        w(12)(5)(2)(3)#2448  = 0
 CustomerVisit(3,6,2,3):         w(3)(6)(2)(3)#858  = 0
 CustomerVisit(4,6,2,3):         w(4)(6)(2)(3)#1038  = 0
 CustomerVisit(5,6,2,3):         w(5)(6)(2)(3)#1218  = 0
 CustomerVisit(6,6,2,3):         w(6)(6)(2)(3)#1398  = 0
 CustomerVisit(7,6,2,3):         w(7)(6)(2)(3)#1578  = 0
 CustomerVisit(8,6,2,3):         w(8)(6)(2)(3)#1758  = 0
 CustomerVisit(9,6,2,3):         w(9)(6)(2)(3)#1938  = 0
 CustomerVisit(10,6,2,3):        w(10)(6)(2)(3)#2118  = 0
 CustomerVisit(11,6,2,3):        w(11)(6)(2)(3)#2298  = 0
 CustomerVisit(12,6,2,3):        w(12)(6)(2)(3)#2478  = 0
 CustomerVisit(3,4,3,3):         w(3)(4)(3)(3)#808 <= 11
 CustomerVisit(4,4,3,3):         w(4)(4)(3)(3)#988 <= 15
 CustomerVisit(5,4,3,3):         w(5)(4)(3)(3)#1168  = 0
 CustomerVisit(6,4,3,3):         w(6)(4)(3)(3)#1348  = 0
 CustomerVisit(7,4,3,3):         w(7)(4)(3)(3)#1528 <= 13
 CustomerVisit(8,4,3,3):         w(8)(4)(3)(3)#1708  = 0
 CustomerVisit(9,4,3,3):         w(9)(4)(3)(3)#1888  = 0
 CustomerVisit(10,4,3,3):        w(10)(4)(3)(3)#2068  = 0
 CustomerVisit(11,4,3,3):        w(11)(4)(3)(3)#2248  = 0
 CustomerVisit(12,4,3,3):        w(12)(4)(3)(3)#2428  = 0
 CustomerVisit(3,5,3,3):         w(3)(5)(3)(3)#838  = 0
 CustomerVisit(4,5,3,3):         w(4)(5)(3)(3)#1018  = 0
 CustomerVisit(5,5,3,3):         w(5)(5)(3)(3)#1198  = 0
 CustomerVisit(6,5,3,3):         w(6)(5)(3)(3)#1378  = 0
 CustomerVisit(7,5,3,3):         w(7)(5)(3)(3)#1558  = 0
 CustomerVisit(8,5,3,3):         w(8)(5)(3)(3)#1738  = 0
 CustomerVisit(9,5,3,3):         w(9)(5)(3)(3)#1918  = 0
 CustomerVisit(10,5,3,3):        w(10)(5)(3)(3)#2098  = 0
 CustomerVisit(11,5,3,3):        w(11)(5)(3)(3)#2278  = 0
 CustomerVisit(12,5,3,3):        w(12)(5)(3)(3)#2458  = 0
 CustomerVisit(3,6,3,3):         w(3)(6)(3)(3)#868  = 0
 CustomerVisit(4,6,3,3):         w(4)(6)(3)(3)#1048  = 0
 CustomerVisit(5,6,3,3):         w(5)(6)(3)(3)#1228  = 0
 CustomerVisit(6,6,3,3):         w(6)(6)(3)(3)#1408  = 0
 CustomerVisit(7,6,3,3):         w(7)(6)(3)(3)#1588  = 0
 CustomerVisit(8,6,3,3):         w(8)(6)(3)(3)#1768  = 0
 CustomerVisit(9,6,3,3):         w(9)(6)(3)(3)#1948  = 0
 CustomerVisit(10,6,3,3):        w(10)(6)(3)(3)#2128  = 0
 CustomerVisit(11,6,3,3):        w(11)(6)(3)(3)#2308  = 0
 CustomerVisit(12,6,3,3):        w(12)(6)(3)(3)#2488  = 0
 CustomerVisit(3,1,1,4):         w(3)(1)(1)(4)#699  = 0
 CustomerVisit(4,1,1,4):         w(4)(1)(1)(4)#879  = 0
 CustomerVisit(5,1,1,4):         w(5)(1)(1)(4)#1059 <= 44
 CustomerVisit(6,1,1,4):         w(6)(1)(1)(4)#1239 <= 14
 CustomerVisit(7,1,1,4):         w(7)(1)(1)(4)#1419  = 0
 CustomerVisit(8,1,1,4):         w(8)(1)(1)(4)#1599  = 0
 CustomerVisit(9,1,1,4):         w(9)(1)(1)(4)#1779 <= 44
 CustomerVisit(10,1,1,4):        w(10)(1)(1)(4)#1959 <= 26
 CustomerVisit(11,1,1,4):        w(11)(1)(1)(4)#2139  = 0
 CustomerVisit(12,1,1,4):        w(12)(1)(1)(4)#2319  = 0
 CustomerVisit(3,2,1,4):         w(3)(2)(1)(4)#729  = 0
 CustomerVisit(4,2,1,4):         w(4)(2)(1)(4)#909  = 0
 CustomerVisit(5,2,1,4):         w(5)(2)(1)(4)#1089  = 0
 CustomerVisit(6,2,1,4):         w(6)(2)(1)(4)#1269  = 0
 CustomerVisit(7,2,1,4):         w(7)(2)(1)(4)#1449  = 0
 CustomerVisit(8,2,1,4):         w(8)(2)(1)(4)#1629  = 0
 CustomerVisit(9,2,1,4):         w(9)(2)(1)(4)#1809  = 0
 CustomerVisit(10,2,1,4):        w(10)(2)(1)(4)#1989  = 0
 CustomerVisit(11,2,1,4):        w(11)(2)(1)(4)#2169  = 0
 CustomerVisit(12,2,1,4):        w(12)(2)(1)(4)#2349  = 0
 CustomerVisit(3,3,1,4):         w(3)(3)(1)(4)#759  = 0
 CustomerVisit(4,3,1,4):         w(4)(3)(1)(4)#939  = 0
 CustomerVisit(5,3,1,4):         w(5)(3)(1)(4)#1119  = 0
 CustomerVisit(6,3,1,4):         w(6)(3)(1)(4)#1299  = 0
 CustomerVisit(7,3,1,4):         w(7)(3)(1)(4)#1479  = 0
 CustomerVisit(8,3,1,4):         w(8)(3)(1)(4)#1659  = 0
 CustomerVisit(9,3,1,4):         w(9)(3)(1)(4)#1839  = 0
 CustomerVisit(10,3,1,4):        w(10)(3)(1)(4)#2019  = 0
 CustomerVisit(11,3,1,4):        w(11)(3)(1)(4)#2199  = 0
 CustomerVisit(12,3,1,4):        w(12)(3)(1)(4)#2379  = 0
 CustomerVisit(3,1,2,4):         w(3)(1)(2)(4)#709  = 0
 CustomerVisit(4,1,2,4):         w(4)(1)(2)(4)#889  = 0
 CustomerVisit(5,1,2,4):         w(5)(1)(2)(4)#1069  = 0
 CustomerVisit(6,1,2,4):         w(6)(1)(2)(4)#1249 <= 12
 CustomerVisit(7,1,2,4):         w(7)(1)(2)(4)#1429  = 0
 CustomerVisit(8,1,2,4):         w(8)(1)(2)(4)#1609  = 0
 CustomerVisit(9,1,2,4):         w(9)(1)(2)(4)#1789  = 0
 CustomerVisit(10,1,2,4):        w(10)(1)(2)(4)#1969 <= 26
 CustomerVisit(11,1,2,4):        w(11)(1)(2)(4)#2149 <= 38
 CustomerVisit(12,1,2,4):        w(12)(1)(2)(4)#2329  = 0
 CustomerVisit(3,2,2,4):         w(3)(2)(2)(4)#739  = 0
 CustomerVisit(4,2,2,4):         w(4)(2)(2)(4)#919  = 0
 CustomerVisit(5,2,2,4):         w(5)(2)(2)(4)#1099 <= 30
 CustomerVisit(6,2,2,4):         w(6)(2)(2)(4)#1279  = 0
 CustomerVisit(7,2,2,4):         w(7)(2)(2)(4)#1459  = 0
 CustomerVisit(8,2,2,4):         w(8)(2)(2)(4)#1639  = 0
 CustomerVisit(9,2,2,4):         w(9)(2)(2)(4)#1819  = 0
 CustomerVisit(10,2,2,4):        w(10)(2)(2)(4)#1999  = 0
 CustomerVisit(11,2,2,4):        w(11)(2)(2)(4)#2179  = 0
 CustomerVisit(12,2,2,4):        w(12)(2)(2)(4)#2359 <= 42
 CustomerVisit(3,3,2,4):         w(3)(3)(2)(4)#769  = 0
 CustomerVisit(4,3,2,4):         w(4)(3)(2)(4)#949  = 0
 CustomerVisit(5,3,2,4):         w(5)(3)(2)(4)#1129  = 0
 CustomerVisit(6,3,2,4):         w(6)(3)(2)(4)#1309  = 0
 CustomerVisit(7,3,2,4):         w(7)(3)(2)(4)#1489  = 0
 CustomerVisit(8,3,2,4):         w(8)(3)(2)(4)#1669  = 0
 CustomerVisit(9,3,2,4):         w(9)(3)(2)(4)#1849  = 0
 CustomerVisit(10,3,2,4):        w(10)(3)(2)(4)#2029  = 0
 CustomerVisit(11,3,2,4):        w(11)(3)(2)(4)#2209  = 0
 CustomerVisit(12,3,2,4):        w(12)(3)(2)(4)#2389  = 0
 CustomerVisit(3,1,3,4):         w(3)(1)(3)(4)#719  = 0
 CustomerVisit(4,1,3,4):         w(4)(1)(3)(4)#899  = 0
 CustomerVisit(5,1,3,4):         w(5)(1)(3)(4)#1079 <= 16
 CustomerVisit(6,1,3,4):         w(6)(1)(3)(4)#1259 <= 6
 CustomerVisit(7,1,3,4):         w(7)(1)(3)(4)#1439  = 0
 CustomerVisit(8,1,3,4):         w(8)(1)(3)(4)#1619  = 0
 CustomerVisit(9,1,3,4):         w(9)(1)(3)(4)#1799  = 0
 CustomerVisit(10,1,3,4):        w(10)(1)(3)(4)#1979 <= 15
 CustomerVisit(11,1,3,4):        w(11)(1)(3)(4)#2159  = 0
 CustomerVisit(12,1,3,4):        w(12)(1)(3)(4)#2339  = 0
 CustomerVisit(3,2,3,4):         w(3)(2)(3)(4)#749  = 0
 CustomerVisit(4,2,3,4):         w(4)(2)(3)(4)#929  = 0
 CustomerVisit(5,2,3,4):         w(5)(2)(3)(4)#1109  = 0
 CustomerVisit(6,2,3,4):         w(6)(2)(3)(4)#1289  = 0
 CustomerVisit(7,2,3,4):         w(7)(2)(3)(4)#1469  = 0
 CustomerVisit(8,2,3,4):         w(8)(2)(3)(4)#1649  = 0
 CustomerVisit(9,2,3,4):         w(9)(2)(3)(4)#1829  = 0
 CustomerVisit(10,2,3,4):        w(10)(2)(3)(4)#2009  = 0
 CustomerVisit(11,2,3,4):        w(11)(2)(3)(4)#2189  = 0
 CustomerVisit(12,2,3,4):        w(12)(2)(3)(4)#2369  = 0
 CustomerVisit(3,3,3,4):         w(3)(3)(3)(4)#779  = 0
 CustomerVisit(4,3,3,4):         w(4)(3)(3)(4)#959  = 0
 CustomerVisit(5,3,3,4):         w(5)(3)(3)(4)#1139  = 0
 CustomerVisit(6,3,3,4):         w(6)(3)(3)(4)#1319  = 0
 CustomerVisit(7,3,3,4):         w(7)(3)(3)(4)#1499  = 0
 CustomerVisit(8,3,3,4):         w(8)(3)(3)(4)#1679  = 0
 CustomerVisit(9,3,3,4):         w(9)(3)(3)(4)#1859  = 0
 CustomerVisit(10,3,3,4):        w(10)(3)(3)(4)#2039  = 0
 CustomerVisit(11,3,3,4):        w(11)(3)(3)(4)#2219  = 0
 CustomerVisit(12,3,3,4):        w(12)(3)(3)(4)#2399  = 0
 CustomerVisit(3,4,1,4):         w(3)(4)(1)(4)#789 <= 20
 CustomerVisit(4,4,1,4):         w(4)(4)(1)(4)#969 <= 44
 CustomerVisit(5,4,1,4):         w(5)(4)(1)(4)#1149  = 0
 CustomerVisit(6,4,1,4):         w(6)(4)(1)(4)#1329  = 0
 CustomerVisit(7,4,1,4):         w(7)(4)(1)(4)#1509 <= 36
 CustomerVisit(8,4,1,4):         w(8)(4)(1)(4)#1689  = 0
 CustomerVisit(9,4,1,4):         w(9)(4)(1)(4)#1869  = 0
 CustomerVisit(10,4,1,4):        w(10)(4)(1)(4)#2049  = 0
 CustomerVisit(11,4,1,4):        w(11)(4)(1)(4)#2229  = 0
 CustomerVisit(12,4,1,4):        w(12)(4)(1)(4)#2409  = 0
 CustomerVisit(3,5,1,4):         w(3)(5)(1)(4)#819  = 0
 CustomerVisit(4,5,1,4):         w(4)(5)(1)(4)#999  = 0
 CustomerVisit(5,5,1,4):         w(5)(5)(1)(4)#1179  = 0
 CustomerVisit(6,5,1,4):         w(6)(5)(1)(4)#1359  = 0
 CustomerVisit(7,5,1,4):         w(7)(5)(1)(4)#1539  = 0
 CustomerVisit(8,5,1,4):         w(8)(5)(1)(4)#1719  = 0
 CustomerVisit(9,5,1,4):         w(9)(5)(1)(4)#1899  = 0
 CustomerVisit(10,5,1,4):        w(10)(5)(1)(4)#2079  = 0
 CustomerVisit(11,5,1,4):        w(11)(5)(1)(4)#2259  = 0
 CustomerVisit(12,5,1,4):        w(12)(5)(1)(4)#2439  = 0
 CustomerVisit(3,6,1,4):         w(3)(6)(1)(4)#849  = 0
 CustomerVisit(4,6,1,4):         w(4)(6)(1)(4)#1029  = 0
 CustomerVisit(5,6,1,4):         w(5)(6)(1)(4)#1209  = 0
 CustomerVisit(6,6,1,4):         w(6)(6)(1)(4)#1389  = 0
 CustomerVisit(7,6,1,4):         w(7)(6)(1)(4)#1569  = 0
 CustomerVisit(8,6,1,4):         w(8)(6)(1)(4)#1749  = 0
 CustomerVisit(9,6,1,4):         w(9)(6)(1)(4)#1929  = 0
 CustomerVisit(10,6,1,4):        w(10)(6)(1)(4)#2109  = 0
 CustomerVisit(11,6,1,4):        w(11)(6)(1)(4)#2289  = 0
 CustomerVisit(12,6,1,4):        w(12)(6)(1)(4)#2469  = 0
 CustomerVisit(3,4,2,4):         w(3)(4)(2)(4)#799 <= 20
 CustomerVisit(4,4,2,4):         w(4)(4)(2)(4)#979 <= 28
 CustomerVisit(5,4,2,4):         w(5)(4)(2)(4)#1159  = 0
 CustomerVisit(6,4,2,4):         w(6)(4)(2)(4)#1339  = 0
 CustomerVisit(7,4,2,4):         w(7)(4)(2)(4)#1519 <= 22
 CustomerVisit(8,4,2,4):         w(8)(4)(2)(4)#1699 <= 30
 CustomerVisit(9,4,2,4):         w(9)(4)(2)(4)#1879  = 0
 CustomerVisit(10,4,2,4):        w(10)(4)(2)(4)#2059  = 0
 CustomerVisit(11,4,2,4):        w(11)(4)(2)(4)#2239  = 0
 CustomerVisit(12,4,2,4):        w(12)(4)(2)(4)#2419  = 0
 CustomerVisit(3,5,2,4):         w(3)(5)(2)(4)#829  = 0
 CustomerVisit(4,5,2,4):         w(4)(5)(2)(4)#1009  = 0
 CustomerVisit(5,5,2,4):         w(5)(5)(2)(4)#1189  = 0
 CustomerVisit(6,5,2,4):         w(6)(5)(2)(4)#1369  = 0
 CustomerVisit(7,5,2,4):         w(7)(5)(2)(4)#1549  = 0
 CustomerVisit(8,5,2,4):         w(8)(5)(2)(4)#1729  = 0
 CustomerVisit(9,5,2,4):         w(9)(5)(2)(4)#1909  = 0
 CustomerVisit(10,5,2,4):        w(10)(5)(2)(4)#2089  = 0
 CustomerVisit(11,5,2,4):        w(11)(5)(2)(4)#2269  = 0
 CustomerVisit(12,5,2,4):        w(12)(5)(2)(4)#2449  = 0
 CustomerVisit(3,6,2,4):         w(3)(6)(2)(4)#859  = 0
 CustomerVisit(4,6,2,4):         w(4)(6)(2)(4)#1039  = 0
 CustomerVisit(5,6,2,4):         w(5)(6)(2)(4)#1219  = 0
 CustomerVisit(6,6,2,4):         w(6)(6)(2)(4)#1399  = 0
 CustomerVisit(7,6,2,4):         w(7)(6)(2)(4)#1579  = 0
 CustomerVisit(8,6,2,4):         w(8)(6)(2)(4)#1759  = 0
 CustomerVisit(9,6,2,4):         w(9)(6)(2)(4)#1939  = 0
 CustomerVisit(10,6,2,4):        w(10)(6)(2)(4)#2119  = 0
 CustomerVisit(11,6,2,4):        w(11)(6)(2)(4)#2299  = 0
 CustomerVisit(12,6,2,4):        w(12)(6)(2)(4)#2479  = 0
 CustomerVisit(3,4,3,4):         w(3)(4)(3)(4)#809 <= 11
 CustomerVisit(4,4,3,4):         w(4)(4)(3)(4)#989 <= 13
 CustomerVisit(5,4,3,4):         w(5)(4)(3)(4)#1169  = 0
 CustomerVisit(6,4,3,4):         w(6)(4)(3)(4)#1349  = 0
 CustomerVisit(7,4,3,4):         w(7)(4)(3)(4)#1529 <= 11
 CustomerVisit(8,4,3,4):         w(8)(4)(3)(4)#1709  = 0
 CustomerVisit(9,4,3,4):         w(9)(4)(3)(4)#1889  = 0
 CustomerVisit(10,4,3,4):        w(10)(4)(3)(4)#2069  = 0
 CustomerVisit(11,4,3,4):        w(11)(4)(3)(4)#2249  = 0
 CustomerVisit(12,4,3,4):        w(12)(4)(3)(4)#2429  = 0
 CustomerVisit(3,5,3,4):         w(3)(5)(3)(4)#839  = 0
 CustomerVisit(4,5,3,4):         w(4)(5)(3)(4)#1019  = 0
 CustomerVisit(5,5,3,4):         w(5)(5)(3)(4)#1199  = 0
 CustomerVisit(6,5,3,4):         w(6)(5)(3)(4)#1379  = 0
 CustomerVisit(7,5,3,4):         w(7)(5)(3)(4)#1559  = 0
 CustomerVisit(8,5,3,4):         w(8)(5)(3)(4)#1739  = 0
 CustomerVisit(9,5,3,4):         w(9)(5)(3)(4)#1919  = 0
 CustomerVisit(10,5,3,4):        w(10)(5)(3)(4)#2099  = 0
 CustomerVisit(11,5,3,4):        w(11)(5)(3)(4)#2279  = 0
 CustomerVisit(12,5,3,4):        w(12)(5)(3)(4)#2459  = 0
 CustomerVisit(3,6,3,4):         w(3)(6)(3)(4)#869  = 0
 CustomerVisit(4,6,3,4):         w(4)(6)(3)(4)#1049  = 0
 CustomerVisit(5,6,3,4):         w(5)(6)(3)(4)#1229  = 0
 CustomerVisit(6,6,3,4):         w(6)(6)(3)(4)#1409  = 0
 CustomerVisit(7,6,3,4):         w(7)(6)(3)(4)#1589  = 0
 CustomerVisit(8,6,3,4):         w(8)(6)(3)(4)#1769  = 0
 CustomerVisit(9,6,3,4):         w(9)(6)(3)(4)#1949  = 0
 CustomerVisit(10,6,3,4):        w(10)(6)(3)(4)#2129  = 0
 CustomerVisit(11,6,3,4):        w(11)(6)(3)(4)#2309  = 0
 CustomerVisit(12,6,3,4):        w(12)(6)(3)(4)#2489  = 0
 CustomerVisit(3,1,1,5):         w(3)(1)(1)(5)#700  = 0
 CustomerVisit(4,1,1,5):         w(4)(1)(1)(5)#880  = 0
 CustomerVisit(5,1,1,5):         w(5)(1)(1)(5)#1060 <= 43
 CustomerVisit(6,1,1,5):         w(6)(1)(1)(5)#1240 <= 14
 CustomerVisit(7,1,1,5):         w(7)(1)(1)(5)#1420  = 0
 CustomerVisit(8,1,1,5):         w(8)(1)(1)(5)#1600  = 0
 CustomerVisit(9,1,1,5):         w(9)(1)(1)(5)#1780  = 0
 CustomerVisit(10,1,1,5):        w(10)(1)(1)(5)#1960 <= 26
 CustomerVisit(11,1,1,5):        w(11)(1)(1)(5)#2140 <= 44
 CustomerVisit(12,1,1,5):        w(12)(1)(1)(5)#2320  = 0
 CustomerVisit(3,2,1,5):         w(3)(2)(1)(5)#730  = 0
 CustomerVisit(4,2,1,5):         w(4)(2)(1)(5)#910  = 0
 CustomerVisit(5,2,1,5):         w(5)(2)(1)(5)#1090  = 0
 CustomerVisit(6,2,1,5):         w(6)(2)(1)(5)#1270  = 0
 CustomerVisit(7,2,1,5):         w(7)(2)(1)(5)#1450  = 0
 CustomerVisit(8,2,1,5):         w(8)(2)(1)(5)#1630  = 0
 CustomerVisit(9,2,1,5):         w(9)(2)(1)(5)#1810  = 0
 CustomerVisit(10,2,1,5):        w(10)(2)(1)(5)#1990  = 0
 CustomerVisit(11,2,1,5):        w(11)(2)(1)(5)#2170  = 0
 CustomerVisit(12,2,1,5):        w(12)(2)(1)(5)#2350  = 0
 CustomerVisit(3,3,1,5):         w(3)(3)(1)(5)#760  = 0
 CustomerVisit(4,3,1,5):         w(4)(3)(1)(5)#940  = 0
 CustomerVisit(5,3,1,5):         w(5)(3)(1)(5)#1120  = 0
 CustomerVisit(6,3,1,5):         w(6)(3)(1)(5)#1300  = 0
 CustomerVisit(7,3,1,5):         w(7)(3)(1)(5)#1480  = 0
 CustomerVisit(8,3,1,5):         w(8)(3)(1)(5)#1660  = 0
 CustomerVisit(9,3,1,5):         w(9)(3)(1)(5)#1840  = 0
 CustomerVisit(10,3,1,5):        w(10)(3)(1)(5)#2020  = 0
 CustomerVisit(11,3,1,5):        w(11)(3)(1)(5)#2200  = 0
 CustomerVisit(12,3,1,5):        w(12)(3)(1)(5)#2380  = 0
 CustomerVisit(3,1,2,5):         w(3)(1)(2)(5)#710  = 0
 CustomerVisit(4,1,2,5):         w(4)(1)(2)(5)#890  = 0
 CustomerVisit(5,1,2,5):         w(5)(1)(2)(5)#1070  = 0
 CustomerVisit(6,1,2,5):         w(6)(1)(2)(5)#1250  = 0
 CustomerVisit(7,1,2,5):         w(7)(1)(2)(5)#1430  = 0
 CustomerVisit(8,1,2,5):         w(8)(1)(2)(5)#1610  = 0
 CustomerVisit(9,1,2,5):         w(9)(1)(2)(5)#1790  = 0
 CustomerVisit(10,1,2,5):        w(10)(1)(2)(5)#1970  = 0
 CustomerVisit(11,1,2,5):        w(11)(1)(2)(5)#2150  = 0
 CustomerVisit(12,1,2,5):        w(12)(1)(2)(5)#2330  = 0
 CustomerVisit(3,2,2,5):         w(3)(2)(2)(5)#740  = 0
 CustomerVisit(4,2,2,5):         w(4)(2)(2)(5)#920  = 0
 CustomerVisit(5,2,2,5):         w(5)(2)(2)(5)#1100  = 0
 CustomerVisit(6,2,2,5):         w(6)(2)(2)(5)#1280  = 0
 CustomerVisit(7,2,2,5):         w(7)(2)(2)(5)#1460  = 0
 CustomerVisit(8,2,2,5):         w(8)(2)(2)(5)#1640  = 0
 CustomerVisit(9,2,2,5):         w(9)(2)(2)(5)#1820 <= 44
 CustomerVisit(10,2,2,5):        w(10)(2)(2)(5)#2000  = 0
 CustomerVisit(11,2,2,5):        w(11)(2)(2)(5)#2180  = 0
 CustomerVisit(12,2,2,5):        w(12)(2)(2)(5)#2360 <= 38
 CustomerVisit(3,3,2,5):         w(3)(3)(2)(5)#770  = 0
 CustomerVisit(4,3,2,5):         w(4)(3)(2)(5)#950  = 0
 CustomerVisit(5,3,2,5):         w(5)(3)(2)(5)#1130 <= 26
 CustomerVisit(6,3,2,5):         w(6)(3)(2)(5)#1310 <= 14
 CustomerVisit(7,3,2,5):         w(7)(3)(2)(5)#1490  = 0
 CustomerVisit(8,3,2,5):         w(8)(3)(2)(5)#1670  = 0
 CustomerVisit(9,3,2,5):         w(9)(3)(2)(5)#1850  = 0
 CustomerVisit(10,3,2,5):        w(10)(3)(2)(5)#2030 <= 24
 CustomerVisit(11,3,2,5):        w(11)(3)(2)(5)#2210  = 0
 CustomerVisit(12,3,2,5):        w(12)(3)(2)(5)#2390  = 0
 CustomerVisit(3,1,3,5):         w(3)(1)(3)(5)#720  = 0
 CustomerVisit(4,1,3,5):         w(4)(1)(3)(5)#900  = 0
 CustomerVisit(5,1,3,5):         w(5)(1)(3)(5)#1080 <= 14
 CustomerVisit(6,1,3,5):         w(6)(1)(3)(5)#1260 <= 8
 CustomerVisit(7,1,3,5):         w(7)(1)(3)(5)#1440  = 0
 CustomerVisit(8,1,3,5):         w(8)(1)(3)(5)#1620  = 0
 CustomerVisit(9,1,3,5):         w(9)(1)(3)(5)#1800  = 0
 CustomerVisit(10,1,3,5):        w(10)(1)(3)(5)#1980 <= 11
 CustomerVisit(11,1,3,5):        w(11)(1)(3)(5)#2160 <= 16
 CustomerVisit(12,1,3,5):        w(12)(1)(3)(5)#2340  = 0
 CustomerVisit(3,2,3,5):         w(3)(2)(3)(5)#750  = 0
 CustomerVisit(4,2,3,5):         w(4)(2)(3)(5)#930  = 0
 CustomerVisit(5,2,3,5):         w(5)(2)(3)(5)#1110  = 0
 CustomerVisit(6,2,3,5):         w(6)(2)(3)(5)#1290  = 0
 CustomerVisit(7,2,3,5):         w(7)(2)(3)(5)#1470  = 0
 CustomerVisit(8,2,3,5):         w(8)(2)(3)(5)#1650  = 0
 CustomerVisit(9,2,3,5):         w(9)(2)(3)(5)#1830  = 0
 CustomerVisit(10,2,3,5):        w(10)(2)(3)(5)#2010  = 0
 CustomerVisit(11,2,3,5):        w(11)(2)(3)(5)#2190  = 0
 CustomerVisit(12,2,3,5):        w(12)(2)(3)(5)#2370  = 0
 CustomerVisit(3,3,3,5):         w(3)(3)(3)(5)#780  = 0
 CustomerVisit(4,3,3,5):         w(4)(3)(3)(5)#960  = 0
 CustomerVisit(5,3,3,5):         w(5)(3)(3)(5)#1140  = 0
 CustomerVisit(6,3,3,5):         w(6)(3)(3)(5)#1320  = 0
 CustomerVisit(7,3,3,5):         w(7)(3)(3)(5)#1500  = 0
 CustomerVisit(8,3,3,5):         w(8)(3)(3)(5)#1680  = 0
 CustomerVisit(9,3,3,5):         w(9)(3)(3)(5)#1860  = 0
 CustomerVisit(10,3,3,5):        w(10)(3)(3)(5)#2040  = 0
 CustomerVisit(11,3,3,5):        w(11)(3)(3)(5)#2220  = 0
 CustomerVisit(12,3,3,5):        w(12)(3)(3)(5)#2400  = 0
 CustomerVisit(3,4,1,5):         w(3)(4)(1)(5)#790 <= 20
 CustomerVisit(4,4,1,5):         w(4)(4)(1)(5)#970 <= 44
 CustomerVisit(5,4,1,5):         w(5)(4)(1)(5)#1150  = 0
 CustomerVisit(6,4,1,5):         w(6)(4)(1)(5)#1330  = 0
 CustomerVisit(7,4,1,5):         w(7)(4)(1)(5)#1510 <= 38
 CustomerVisit(8,4,1,5):         w(8)(4)(1)(5)#1690  = 0
 CustomerVisit(9,4,1,5):         w(9)(4)(1)(5)#1870  = 0
 CustomerVisit(10,4,1,5):        w(10)(4)(1)(5)#2050  = 0
 CustomerVisit(11,4,1,5):        w(11)(4)(1)(5)#2230  = 0
 CustomerVisit(12,4,1,5):        w(12)(4)(1)(5)#2410  = 0
 CustomerVisit(3,5,1,5):         w(3)(5)(1)(5)#820  = 0
 CustomerVisit(4,5,1,5):         w(4)(5)(1)(5)#1000  = 0
 CustomerVisit(5,5,1,5):         w(5)(5)(1)(5)#1180  = 0
 CustomerVisit(6,5,1,5):         w(6)(5)(1)(5)#1360  = 0
 CustomerVisit(7,5,1,5):         w(7)(5)(1)(5)#1540  = 0
 CustomerVisit(8,5,1,5):         w(8)(5)(1)(5)#1720  = 0
 CustomerVisit(9,5,1,5):         w(9)(5)(1)(5)#1900  = 0
 CustomerVisit(10,5,1,5):        w(10)(5)(1)(5)#2080  = 0
 CustomerVisit(11,5,1,5):        w(11)(5)(1)(5)#2260  = 0
 CustomerVisit(12,5,1,5):        w(12)(5)(1)(5)#2440  = 0
 CustomerVisit(3,6,1,5):         w(3)(6)(1)(5)#850  = 0
 CustomerVisit(4,6,1,5):         w(4)(6)(1)(5)#1030  = 0
 CustomerVisit(5,6,1,5):         w(5)(6)(1)(5)#1210  = 0
 CustomerVisit(6,6,1,5):         w(6)(6)(1)(5)#1390  = 0
 CustomerVisit(7,6,1,5):         w(7)(6)(1)(5)#1570  = 0
 CustomerVisit(8,6,1,5):         w(8)(6)(1)(5)#1750  = 0
 CustomerVisit(9,6,1,5):         w(9)(6)(1)(5)#1930  = 0
 CustomerVisit(10,6,1,5):        w(10)(6)(1)(5)#2110  = 0
 CustomerVisit(11,6,1,5):        w(11)(6)(1)(5)#2290  = 0
 CustomerVisit(12,6,1,5):        w(12)(6)(1)(5)#2470  = 0
 CustomerVisit(3,4,2,5):         w(3)(4)(2)(5)#800 <= 20
 CustomerVisit(4,4,2,5):         w(4)(4)(2)(5)#980 <= 31
 CustomerVisit(5,4,2,5):         w(5)(4)(2)(5)#1160  = 0
 CustomerVisit(6,4,2,5):         w(6)(4)(2)(5)#1340  = 0
 CustomerVisit(7,4,2,5):         w(7)(4)(2)(5)#1520 <= 24
 CustomerVisit(8,4,2,5):         w(8)(4)(2)(5)#1700 <= 28
 CustomerVisit(9,4,2,5):         w(9)(4)(2)(5)#1880  = 0
 CustomerVisit(10,4,2,5):        w(10)(4)(2)(5)#2060  = 0
 CustomerVisit(11,4,2,5):        w(11)(4)(2)(5)#2240  = 0
 CustomerVisit(12,4,2,5):        w(12)(4)(2)(5)#2420  = 0
 CustomerVisit(3,5,2,5):         w(3)(5)(2)(5)#830  = 0
 CustomerVisit(4,5,2,5):         w(4)(5)(2)(5)#1010  = 0
 CustomerVisit(5,5,2,5):         w(5)(5)(2)(5)#1190  = 0
 CustomerVisit(6,5,2,5):         w(6)(5)(2)(5)#1370  = 0
 CustomerVisit(7,5,2,5):         w(7)(5)(2)(5)#1550  = 0
 CustomerVisit(8,5,2,5):         w(8)(5)(2)(5)#1730  = 0
 CustomerVisit(9,5,2,5):         w(9)(5)(2)(5)#1910  = 0
 CustomerVisit(10,5,2,5):        w(10)(5)(2)(5)#2090  = 0
 CustomerVisit(11,5,2,5):        w(11)(5)(2)(5)#2270  = 0
 CustomerVisit(12,5,2,5):        w(12)(5)(2)(5)#2450  = 0
 CustomerVisit(3,6,2,5):         w(3)(6)(2)(5)#860  = 0
 CustomerVisit(4,6,2,5):         w(4)(6)(2)(5)#1040  = 0
 CustomerVisit(5,6,2,5):         w(5)(6)(2)(5)#1220  = 0
 CustomerVisit(6,6,2,5):         w(6)(6)(2)(5)#1400  = 0
 CustomerVisit(7,6,2,5):         w(7)(6)(2)(5)#1580  = 0
 CustomerVisit(8,6,2,5):         w(8)(6)(2)(5)#1760  = 0
 CustomerVisit(9,6,2,5):         w(9)(6)(2)(5)#1940  = 0
 CustomerVisit(10,6,2,5):        w(10)(6)(2)(5)#2120  = 0
 CustomerVisit(11,6,2,5):        w(11)(6)(2)(5)#2300  = 0
 CustomerVisit(12,6,2,5):        w(12)(6)(2)(5)#2480  = 0
 CustomerVisit(3,4,3,5):         w(3)(4)(3)(5)#810 <= 9
 CustomerVisit(4,4,3,5):         w(4)(4)(3)(5)#990  = 0
 CustomerVisit(5,4,3,5):         w(5)(4)(3)(5)#1170  = 0
 CustomerVisit(6,4,3,5):         w(6)(4)(3)(5)#1350  = 0
 CustomerVisit(7,4,3,5):         w(7)(4)(3)(5)#1530 <= 12
 CustomerVisit(8,4,3,5):         w(8)(4)(3)(5)#1710  = 0
 CustomerVisit(9,4,3,5):         w(9)(4)(3)(5)#1890  = 0
 CustomerVisit(10,4,3,5):        w(10)(4)(3)(5)#2070  = 0
 CustomerVisit(11,4,3,5):        w(11)(4)(3)(5)#2250  = 0
 CustomerVisit(12,4,3,5):        w(12)(4)(3)(5)#2430  = 0
 CustomerVisit(3,5,3,5):         w(3)(5)(3)(5)#840  = 0
 CustomerVisit(4,5,3,5):         w(4)(5)(3)(5)#1020  = 0
 CustomerVisit(5,5,3,5):         w(5)(5)(3)(5)#1200  = 0
 CustomerVisit(6,5,3,5):         w(6)(5)(3)(5)#1380  = 0
 CustomerVisit(7,5,3,5):         w(7)(5)(3)(5)#1560  = 0
 CustomerVisit(8,5,3,5):         w(8)(5)(3)(5)#1740  = 0
 CustomerVisit(9,5,3,5):         w(9)(5)(3)(5)#1920  = 0
 CustomerVisit(10,5,3,5):        w(10)(5)(3)(5)#2100  = 0
 CustomerVisit(11,5,3,5):        w(11)(5)(3)(5)#2280  = 0
 CustomerVisit(12,5,3,5):        w(12)(5)(3)(5)#2460  = 0
 CustomerVisit(3,6,3,5):         w(3)(6)(3)(5)#870  = 0
 CustomerVisit(4,6,3,5):         w(4)(6)(3)(5)#1050  = 0
 CustomerVisit(5,6,3,5):         w(5)(6)(3)(5)#1230  = 0
 CustomerVisit(6,6,3,5):         w(6)(6)(3)(5)#1410  = 0
 CustomerVisit(7,6,3,5):         w(7)(6)(3)(5)#1590  = 0
 CustomerVisit(8,6,3,5):         w(8)(6)(3)(5)#1770  = 0
 CustomerVisit(9,6,3,5):         w(9)(6)(3)(5)#1950  = 0
 CustomerVisit(10,6,3,5):        w(10)(6)(3)(5)#2130  = 0
 CustomerVisit(11,6,3,5):        w(11)(6)(3)(5)#2310  = 0
 CustomerVisit(12,6,3,5):        w(12)(6)(3)(5)#2490  = 0
 CustomerVisit(3,1,1,6):         w(3)(1)(1)(6)#701  = 0
 CustomerVisit(4,1,1,6):         w(4)(1)(1)(6)#881  = 0
 CustomerVisit(5,1,1,6):         w(5)(1)(1)(6)#1061  = 0
 CustomerVisit(6,1,1,6):         w(6)(1)(1)(6)#1241  = 0
 CustomerVisit(7,1,1,6):         w(7)(1)(1)(6)#1421  = 0
 CustomerVisit(8,1,1,6):         w(8)(1)(1)(6)#1601  = 0
 CustomerVisit(9,1,1,6):         w(9)(1)(1)(6)#1781  = 0
 CustomerVisit(10,1,1,6):        w(10)(1)(1)(6)#1961  = 0
 CustomerVisit(11,1,1,6):        w(11)(1)(1)(6)#2141  = 0
 CustomerVisit(12,1,1,6):        w(12)(1)(1)(6)#2321  = 0
 CustomerVisit(3,2,1,6):         w(3)(2)(1)(6)#731 <= 20
 CustomerVisit(4,2,1,6):         w(4)(2)(1)(6)#911  = 0
 CustomerVisit(5,2,1,6):         w(5)(2)(1)(6)#1091  = 0
 CustomerVisit(6,2,1,6):         w(6)(2)(1)(6)#1271 <= 14
 CustomerVisit(7,2,1,6):         w(7)(2)(1)(6)#1451  = 0
 CustomerVisit(8,2,1,6):         w(8)(2)(1)(6)#1631  = 0
 CustomerVisit(9,2,1,6):         w(9)(2)(1)(6)#1811  = 0
 CustomerVisit(10,2,1,6):        w(10)(2)(1)(6)#1991 <= 26
 CustomerVisit(11,2,1,6):        w(11)(2)(1)(6)#2171  = 0
 CustomerVisit(12,2,1,6):        w(12)(2)(1)(6)#2351 <= 44
 CustomerVisit(3,3,1,6):         w(3)(3)(1)(6)#761  = 0
 CustomerVisit(4,3,1,6):         w(4)(3)(1)(6)#941  = 0
 CustomerVisit(5,3,1,6):         w(5)(3)(1)(6)#1121  = 0
 CustomerVisit(6,3,1,6):         w(6)(3)(1)(6)#1301  = 0
 CustomerVisit(7,3,1,6):         w(7)(3)(1)(6)#1481  = 0
 CustomerVisit(8,3,1,6):         w(8)(3)(1)(6)#1661  = 0
 CustomerVisit(9,3,1,6):         w(9)(3)(1)(6)#1841  = 0
 CustomerVisit(10,3,1,6):        w(10)(3)(1)(6)#2021  = 0
 CustomerVisit(11,3,1,6):        w(11)(3)(1)(6)#2201  = 0
 CustomerVisit(12,3,1,6):        w(12)(3)(1)(6)#2381  = 0
 CustomerVisit(3,1,2,6):         w(3)(1)(2)(6)#711  = 0
 CustomerVisit(4,1,2,6):         w(4)(1)(2)(6)#891  = 0
 CustomerVisit(5,1,2,6):         w(5)(1)(2)(6)#1071 <= 30
 CustomerVisit(6,1,2,6):         w(6)(1)(2)(6)#1251  = 0
 CustomerVisit(7,1,2,6):         w(7)(1)(2)(6)#1431  = 0
 CustomerVisit(8,1,2,6):         w(8)(1)(2)(6)#1611  = 0
 CustomerVisit(9,1,2,6):         w(9)(1)(2)(6)#1791  = 0
 CustomerVisit(10,1,2,6):        w(10)(1)(2)(6)#1971  = 0
 CustomerVisit(11,1,2,6):        w(11)(1)(2)(6)#2151  = 0
 CustomerVisit(12,1,2,6):        w(12)(1)(2)(6)#2331  = 0
 CustomerVisit(3,2,2,6):         w(3)(2)(2)(6)#741 <= 19
 CustomerVisit(4,2,2,6):         w(4)(2)(2)(6)#921  = 0
 CustomerVisit(5,2,2,6):         w(5)(2)(2)(6)#1101  = 0
 CustomerVisit(6,2,2,6):         w(6)(2)(2)(6)#1281 <= 12
 CustomerVisit(7,2,2,6):         w(7)(2)(2)(6)#1461  = 0
 CustomerVisit(8,2,2,6):         w(8)(2)(2)(6)#1641 <= 34
 CustomerVisit(9,2,2,6):         w(9)(2)(2)(6)#1821 <= 44
 CustomerVisit(10,2,2,6):        w(10)(2)(2)(6)#2001 <= 26
 CustomerVisit(11,2,2,6):        w(11)(2)(2)(6)#2181  = 0
 CustomerVisit(12,2,2,6):        w(12)(2)(2)(6)#2361  = 0
 CustomerVisit(3,3,2,6):         w(3)(3)(2)(6)#771  = 0
 CustomerVisit(4,3,2,6):         w(4)(3)(2)(6)#951  = 0
 CustomerVisit(5,3,2,6):         w(5)(3)(2)(6)#1131  = 0
 CustomerVisit(6,3,2,6):         w(6)(3)(2)(6)#1311  = 0
 CustomerVisit(7,3,2,6):         w(7)(3)(2)(6)#1491  = 0
 CustomerVisit(8,3,2,6):         w(8)(3)(2)(6)#1671  = 0
 CustomerVisit(9,3,2,6):         w(9)(3)(2)(6)#1851  = 0
 CustomerVisit(10,3,2,6):        w(10)(3)(2)(6)#2031  = 0
 CustomerVisit(11,3,2,6):        w(11)(3)(2)(6)#2211  = 0
 CustomerVisit(12,3,2,6):        w(12)(3)(2)(6)#2391 <= 44
 CustomerVisit(3,1,3,6):         w(3)(1)(3)(6)#721 <= 9
 CustomerVisit(4,1,3,6):         w(4)(1)(3)(6)#901  = 0
 CustomerVisit(5,1,3,6):         w(5)(1)(3)(6)#1081 <= 13
 CustomerVisit(6,1,3,6):         w(6)(1)(3)(6)#1261  = 0
 CustomerVisit(7,1,3,6):         w(7)(1)(3)(6)#1441  = 0
 CustomerVisit(8,1,3,6):         w(8)(1)(3)(6)#1621  = 0
 CustomerVisit(9,1,3,6):         w(9)(1)(3)(6)#1801 <= 26
 CustomerVisit(10,1,3,6):        w(10)(1)(3)(6)#1981  = 0
 CustomerVisit(11,1,3,6):        w(11)(1)(3)(6)#2161  = 0
 CustomerVisit(12,1,3,6):        w(12)(1)(3)(6)#2341  = 0
 CustomerVisit(3,2,3,6):         w(3)(2)(3)(6)#751  = 0
 CustomerVisit(4,2,3,6):         w(4)(2)(3)(6)#931  = 0
 CustomerVisit(5,2,3,6):         w(5)(2)(3)(6)#1111  = 0
 CustomerVisit(6,2,3,6):         w(6)(2)(3)(6)#1291  = 0
 CustomerVisit(7,2,3,6):         w(7)(2)(3)(6)#1471  = 0
 CustomerVisit(8,2,3,6):         w(8)(2)(3)(6)#1651  = 0
 CustomerVisit(9,2,3,6):         w(9)(2)(3)(6)#1831  = 0
 CustomerVisit(10,2,3,6):        w(10)(2)(3)(6)#2011  = 0
 CustomerVisit(11,2,3,6):        w(11)(2)(3)(6)#2191  = 0
 CustomerVisit(12,2,3,6):        w(12)(2)(3)(6)#2371  = 0
 CustomerVisit(3,3,3,6):         w(3)(3)(3)(6)#781  = 0
 CustomerVisit(4,3,3,6):         w(4)(3)(3)(6)#961  = 0
 CustomerVisit(5,3,3,6):         w(5)(3)(3)(6)#1141  = 0
 CustomerVisit(6,3,3,6):         w(6)(3)(3)(6)#1321  = 0
 CustomerVisit(7,3,3,6):         w(7)(3)(3)(6)#1501  = 0
 CustomerVisit(8,3,3,6):         w(8)(3)(3)(6)#1681  = 0
 CustomerVisit(9,3,3,6):         w(9)(3)(3)(6)#1861  = 0
 CustomerVisit(10,3,3,6):        w(10)(3)(3)(6)#2041  = 0
 CustomerVisit(11,3,3,6):        w(11)(3)(3)(6)#2221  = 0
 CustomerVisit(12,3,3,6):        w(12)(3)(3)(6)#2401  = 0
 CustomerVisit(3,4,1,6):         w(3)(4)(1)(6)#791  = 0
 CustomerVisit(4,4,1,6):         w(4)(4)(1)(6)#971 <= 42
 CustomerVisit(5,4,1,6):         w(5)(4)(1)(6)#1151  = 0
 CustomerVisit(6,4,1,6):         w(6)(4)(1)(6)#1331  = 0
 CustomerVisit(7,4,1,6):         w(7)(4)(1)(6)#1511 <= 39
 CustomerVisit(8,4,1,6):         w(8)(4)(1)(6)#1691  = 0
 CustomerVisit(9,4,1,6):         w(9)(4)(1)(6)#1871  = 0
 CustomerVisit(10,4,1,6):        w(10)(4)(1)(6)#2051  = 0
 CustomerVisit(11,4,1,6):        w(11)(4)(1)(6)#2231  = 0
 CustomerVisit(12,4,1,6):        w(12)(4)(1)(6)#2411  = 0
 CustomerVisit(3,5,1,6):         w(3)(5)(1)(6)#821  = 0
 CustomerVisit(4,5,1,6):         w(4)(5)(1)(6)#1001  = 0
 CustomerVisit(5,5,1,6):         w(5)(5)(1)(6)#1181  = 0
 CustomerVisit(6,5,1,6):         w(6)(5)(1)(6)#1361  = 0
 CustomerVisit(7,5,1,6):         w(7)(5)(1)(6)#1541  = 0
 CustomerVisit(8,5,1,6):         w(8)(5)(1)(6)#1721  = 0
 CustomerVisit(9,5,1,6):         w(9)(5)(1)(6)#1901  = 0
 CustomerVisit(10,5,1,6):        w(10)(5)(1)(6)#2081  = 0
 CustomerVisit(11,5,1,6):        w(11)(5)(1)(6)#2261  = 0
 CustomerVisit(12,5,1,6):        w(12)(5)(1)(6)#2441  = 0
 CustomerVisit(3,6,1,6):         w(3)(6)(1)(6)#851  = 0
 CustomerVisit(4,6,1,6):         w(4)(6)(1)(6)#1031  = 0
 CustomerVisit(5,6,1,6):         w(5)(6)(1)(6)#1211  = 0
 CustomerVisit(6,6,1,6):         w(6)(6)(1)(6)#1391  = 0
 CustomerVisit(7,6,1,6):         w(7)(6)(1)(6)#1571  = 0
 CustomerVisit(8,6,1,6):         w(8)(6)(1)(6)#1751  = 0
 CustomerVisit(9,6,1,6):         w(9)(6)(1)(6)#1931  = 0
 CustomerVisit(10,6,1,6):        w(10)(6)(1)(6)#2111  = 0
 CustomerVisit(11,6,1,6):        w(11)(6)(1)(6)#2291  = 0
 CustomerVisit(12,6,1,6):        w(12)(6)(1)(6)#2471  = 0
 CustomerVisit(3,4,2,6):         w(3)(4)(2)(6)#801  = 0
 CustomerVisit(4,4,2,6):         w(4)(4)(2)(6)#981  = 0
 CustomerVisit(5,4,2,6):         w(5)(4)(2)(6)#1161  = 0
 CustomerVisit(6,4,2,6):         w(6)(4)(2)(6)#1341  = 0
 CustomerVisit(7,4,2,6):         w(7)(4)(2)(6)#1521  = 0
 CustomerVisit(8,4,2,6):         w(8)(4)(2)(6)#1701  = 0
 CustomerVisit(9,4,2,6):         w(9)(4)(2)(6)#1881  = 0
 CustomerVisit(10,4,2,6):        w(10)(4)(2)(6)#2061  = 0
 CustomerVisit(11,4,2,6):        w(11)(4)(2)(6)#2241  = 0
 CustomerVisit(12,4,2,6):        w(12)(4)(2)(6)#2421  = 0
 CustomerVisit(3,5,2,6):         w(3)(5)(2)(6)#831  = 0
 CustomerVisit(4,5,2,6):         w(4)(5)(2)(6)#1011  = 0
 CustomerVisit(5,5,2,6):         w(5)(5)(2)(6)#1191  = 0
 CustomerVisit(6,5,2,6):         w(6)(5)(2)(6)#1371  = 0
 CustomerVisit(7,5,2,6):         w(7)(5)(2)(6)#1551  = 0
 CustomerVisit(8,5,2,6):         w(8)(5)(2)(6)#1731  = 0
 CustomerVisit(9,5,2,6):         w(9)(5)(2)(6)#1911  = 0
 CustomerVisit(10,5,2,6):        w(10)(5)(2)(6)#2091  = 0
 CustomerVisit(11,5,2,6):        w(11)(5)(2)(6)#2271  = 0
 CustomerVisit(12,5,2,6):        w(12)(5)(2)(6)#2451  = 0
 CustomerVisit(3,6,2,6):         w(3)(6)(2)(6)#861  = 0
 CustomerVisit(4,6,2,6):         w(4)(6)(2)(6)#1041  = 0
 CustomerVisit(5,6,2,6):         w(5)(6)(2)(6)#1221  = 0
 CustomerVisit(6,6,2,6):         w(6)(6)(2)(6)#1401  = 0
 CustomerVisit(7,6,2,6):         w(7)(6)(2)(6)#1581  = 0
 CustomerVisit(8,6,2,6):         w(8)(6)(2)(6)#1761  = 0
 CustomerVisit(9,6,2,6):         w(9)(6)(2)(6)#1941  = 0
 CustomerVisit(10,6,2,6):        w(10)(6)(2)(6)#2121  = 0
 CustomerVisit(11,6,2,6):        w(11)(6)(2)(6)#2301  = 0
 CustomerVisit(12,6,2,6):        w(12)(6)(2)(6)#2481  = 0
 CustomerVisit(3,4,3,6):         w(3)(4)(3)(6)#811  = 0
 CustomerVisit(4,4,3,6):         w(4)(4)(3)(6)#991 <= 16
 CustomerVisit(5,4,3,6):         w(5)(4)(3)(6)#1171  = 0
 CustomerVisit(6,4,3,6):         w(6)(4)(3)(6)#1351 <= 6
 CustomerVisit(7,4,3,6):         w(7)(4)(3)(6)#1531 <= 14
 CustomerVisit(8,4,3,6):         w(8)(4)(3)(6)#1711  = 0
 CustomerVisit(9,4,3,6):         w(9)(4)(3)(6)#1891  = 0
 CustomerVisit(10,4,3,6):        w(10)(4)(3)(6)#2071 <= 14
 CustomerVisit(11,4,3,6):        w(11)(4)(3)(6)#2251 <= 16
 CustomerVisit(12,4,3,6):        w(12)(4)(3)(6)#2431  = 0
 CustomerVisit(3,5,3,6):         w(3)(5)(3)(6)#841  = 0
 CustomerVisit(4,5,3,6):         w(4)(5)(3)(6)#1021  = 0
 CustomerVisit(5,5,3,6):         w(5)(5)(3)(6)#1201  = 0
 CustomerVisit(6,5,3,6):         w(6)(5)(3)(6)#1381  = 0
 CustomerVisit(7,5,3,6):         w(7)(5)(3)(6)#1561  = 0
 CustomerVisit(8,5,3,6):         w(8)(5)(3)(6)#1741  = 0
 CustomerVisit(9,5,3,6):         w(9)(5)(3)(6)#1921  = 0
 CustomerVisit(10,5,3,6):        w(10)(5)(3)(6)#2101  = 0
 CustomerVisit(11,5,3,6):        w(11)(5)(3)(6)#2281  = 0
 CustomerVisit(12,5,3,6):        w(12)(5)(3)(6)#2461  = 0
 CustomerVisit(3,6,3,6):         w(3)(6)(3)(6)#871  = 0
 CustomerVisit(4,6,3,6):         w(4)(6)(3)(6)#1051  = 0
 CustomerVisit(5,6,3,6):         w(5)(6)(3)(6)#1231  = 0
 CustomerVisit(6,6,3,6):         w(6)(6)(3)(6)#1411  = 0
 CustomerVisit(7,6,3,6):         w(7)(6)(3)(6)#1591  = 0
 CustomerVisit(8,6,3,6):         w(8)(6)(3)(6)#1771  = 0
 CustomerVisit(9,6,3,6):         w(9)(6)(3)(6)#1951  = 0
 CustomerVisit(10,6,3,6):        w(10)(6)(3)(6)#2131  = 0
 CustomerVisit(11,6,3,6):        w(11)(6)(3)(6)#2311  = 0
 CustomerVisit(12,6,3,6):        w(12)(6)(3)(6)#2491  = 0
 CustomerVisit(3,1,1,7):         w(3)(1)(1)(7)#702  = 0
 CustomerVisit(4,1,1,7):         w(4)(1)(1)(7)#882  = 0
 CustomerVisit(5,1,1,7):         w(5)(1)(1)(7)#1062  = 0
 CustomerVisit(6,1,1,7):         w(6)(1)(1)(7)#1242  = 0
 CustomerVisit(7,1,1,7):         w(7)(1)(1)(7)#1422  = 0
 CustomerVisit(8,1,1,7):         w(8)(1)(1)(7)#1602  = 0
 CustomerVisit(9,1,1,7):         w(9)(1)(1)(7)#1782  = 0
 CustomerVisit(10,1,1,7):        w(10)(1)(1)(7)#1962  = 0
 CustomerVisit(11,1,1,7):        w(11)(1)(1)(7)#2142  = 0
 CustomerVisit(12,1,1,7):        w(12)(1)(1)(7)#2322  = 0
 CustomerVisit(3,2,1,7):         w(3)(2)(1)(7)#732  = 0
 CustomerVisit(4,2,1,7):         w(4)(2)(1)(7)#912  = 0
 CustomerVisit(5,2,1,7):         w(5)(2)(1)(7)#1092  = 0
 CustomerVisit(6,2,1,7):         w(6)(2)(1)(7)#1272  = 0
 CustomerVisit(7,2,1,7):         w(7)(2)(1)(7)#1452  = 0
 CustomerVisit(8,2,1,7):         w(8)(2)(1)(7)#1632  = 0
 CustomerVisit(9,2,1,7):         w(9)(2)(1)(7)#1812  = 0
 CustomerVisit(10,2,1,7):        w(10)(2)(1)(7)#1992  = 0
 CustomerVisit(11,2,1,7):        w(11)(2)(1)(7)#2172  = 0
 CustomerVisit(12,2,1,7):        w(12)(2)(1)(7)#2352  = 0
 CustomerVisit(3,3,1,7):         w(3)(3)(1)(7)#762  = 0
 CustomerVisit(4,3,1,7):         w(4)(3)(1)(7)#942  = 0
 CustomerVisit(5,3,1,7):         w(5)(3)(1)(7)#1122 <= 40
 CustomerVisit(6,3,1,7):         w(6)(3)(1)(7)#1302 <= 14
 CustomerVisit(7,3,1,7):         w(7)(3)(1)(7)#1482  = 0
 CustomerVisit(8,3,1,7):         w(8)(3)(1)(7)#1662  = 0
 CustomerVisit(9,3,1,7):         w(9)(3)(1)(7)#1842  = 0
 CustomerVisit(10,3,1,7):        w(10)(3)(1)(7)#2022 <= 26
 CustomerVisit(11,3,1,7):        w(11)(3)(1)(7)#2202 <= 44
 CustomerVisit(12,3,1,7):        w(12)(3)(1)(7)#2382  = 0
 CustomerVisit(3,1,2,7):         w(3)(1)(2)(7)#712  = 0
 CustomerVisit(4,1,2,7):         w(4)(1)(2)(7)#892  = 0
 CustomerVisit(5,1,2,7):         w(5)(1)(2)(7)#1072  = 0
 CustomerVisit(6,1,2,7):         w(6)(1)(2)(7)#1252  = 0
 CustomerVisit(7,1,2,7):         w(7)(1)(2)(7)#1432  = 0
 CustomerVisit(8,1,2,7):         w(8)(1)(2)(7)#1612  = 0
 CustomerVisit(9,1,2,7):         w(9)(1)(2)(7)#1792  = 0
 CustomerVisit(10,1,2,7):        w(10)(1)(2)(7)#1972  = 0
 CustomerVisit(11,1,2,7):        w(11)(1)(2)(7)#2152  = 0
 CustomerVisit(12,1,2,7):        w(12)(1)(2)(7)#2332  = 0
 CustomerVisit(3,2,2,7):         w(3)(2)(2)(7)#742  = 0
 CustomerVisit(4,2,2,7):         w(4)(2)(2)(7)#922  = 0
 CustomerVisit(5,2,2,7):         w(5)(2)(2)(7)#1102  = 0
 CustomerVisit(6,2,2,7):         w(6)(2)(2)(7)#1282  = 0
 CustomerVisit(7,2,2,7):         w(7)(2)(2)(7)#1462  = 0
 CustomerVisit(8,2,2,7):         w(8)(2)(2)(7)#1642  = 0
 CustomerVisit(9,2,2,7):         w(9)(2)(2)(7)#1822  = 0
 CustomerVisit(10,2,2,7):        w(10)(2)(2)(7)#2002  = 0
 CustomerVisit(11,2,2,7):        w(11)(2)(2)(7)#2182  = 0
 CustomerVisit(12,2,2,7):        w(12)(2)(2)(7)#2362  = 0
 CustomerVisit(3,3,2,7):         w(3)(3)(2)(7)#772  = 0
 CustomerVisit(4,3,2,7):         w(4)(3)(2)(7)#952  = 0
 CustomerVisit(5,3,2,7):         w(5)(3)(2)(7)#1132  = 0
 CustomerVisit(6,3,2,7):         w(6)(3)(2)(7)#1312  = 0
 CustomerVisit(7,3,2,7):         w(7)(3)(2)(7)#1492  = 0
 CustomerVisit(8,3,2,7):         w(8)(3)(2)(7)#1672  = 0
 CustomerVisit(9,3,2,7):         w(9)(3)(2)(7)#1852  = 0
 CustomerVisit(10,3,2,7):        w(10)(3)(2)(7)#2032  = 0
 CustomerVisit(11,3,2,7):        w(11)(3)(2)(7)#2212  = 0
 CustomerVisit(12,3,2,7):        w(12)(3)(2)(7)#2392  = 0
 CustomerVisit(3,1,3,7):         w(3)(1)(3)(7)#722  = 0
 CustomerVisit(4,1,3,7):         w(4)(1)(3)(7)#902  = 0
 CustomerVisit(5,1,3,7):         w(5)(1)(3)(7)#1082  = 0
 CustomerVisit(6,1,3,7):         w(6)(1)(3)(7)#1262  = 0
 CustomerVisit(7,1,3,7):         w(7)(1)(3)(7)#1442  = 0
 CustomerVisit(8,1,3,7):         w(8)(1)(3)(7)#1622  = 0
 CustomerVisit(9,1,3,7):         w(9)(1)(3)(7)#1802  = 0
 CustomerVisit(10,1,3,7):        w(10)(1)(3)(7)#1982  = 0
 CustomerVisit(11,1,3,7):        w(11)(1)(3)(7)#2162  = 0
 CustomerVisit(12,1,3,7):        w(12)(1)(3)(7)#2342  = 0
 CustomerVisit(3,2,3,7):         w(3)(2)(3)(7)#752 <= 10
 CustomerVisit(4,2,3,7):         w(4)(2)(3)(7)#932  = 0
 CustomerVisit(5,2,3,7):         w(5)(2)(3)(7)#1112  = 0
 CustomerVisit(6,2,3,7):         w(6)(2)(3)(7)#1292 <= 7
 CustomerVisit(7,2,3,7):         w(7)(2)(3)(7)#1472  = 0
 CustomerVisit(8,2,3,7):         w(8)(2)(3)(7)#1652  = 0
 CustomerVisit(9,2,3,7):         w(9)(2)(3)(7)#1832  = 0
 CustomerVisit(10,2,3,7):        w(10)(2)(3)(7)#2012 <= 15
 CustomerVisit(11,2,3,7):        w(11)(2)(3)(7)#2192 <= 21
 CustomerVisit(12,2,3,7):        w(12)(2)(3)(7)#2372  = 0
 CustomerVisit(3,3,3,7):         w(3)(3)(3)(7)#782  = 0
 CustomerVisit(4,3,3,7):         w(4)(3)(3)(7)#962  = 0
 CustomerVisit(5,3,3,7):         w(5)(3)(3)(7)#1142 <= 12
 CustomerVisit(6,3,3,7):         w(6)(3)(3)(7)#1322  = 0
 CustomerVisit(7,3,3,7):         w(7)(3)(3)(7)#1502  = 0
 CustomerVisit(8,3,3,7):         w(8)(3)(3)(7)#1682  = 0
 CustomerVisit(9,3,3,7):         w(9)(3)(3)(7)#1862 <= 26
 CustomerVisit(10,3,3,7):        w(10)(3)(3)(7)#2042  = 0
 CustomerVisit(11,3,3,7):        w(11)(3)(3)(7)#2222  = 0
 CustomerVisit(12,3,3,7):        w(12)(3)(3)(7)#2402 <= 25
 CustomerVisit(3,4,1,7):         w(3)(4)(1)(7)#792 <= 20
 CustomerVisit(4,4,1,7):         w(4)(4)(1)(7)#972 <= 44
 CustomerVisit(5,4,1,7):         w(5)(4)(1)(7)#1152  = 0
 CustomerVisit(6,4,1,7):         w(6)(4)(1)(7)#1332  = 0
 CustomerVisit(7,4,1,7):         w(7)(4)(1)(7)#1512 <= 39
 CustomerVisit(8,4,1,7):         w(8)(4)(1)(7)#1692  = 0
 CustomerVisit(9,4,1,7):         w(9)(4)(1)(7)#1872  = 0
 CustomerVisit(10,4,1,7):        w(10)(4)(1)(7)#2052  = 0
 CustomerVisit(11,4,1,7):        w(11)(4)(1)(7)#2232  = 0
 CustomerVisit(12,4,1,7):        w(12)(4)(1)(7)#2412  = 0
 CustomerVisit(3,5,1,7):         w(3)(5)(1)(7)#822  = 0
 CustomerVisit(4,5,1,7):         w(4)(5)(1)(7)#1002  = 0
 CustomerVisit(5,5,1,7):         w(5)(5)(1)(7)#1182  = 0
 CustomerVisit(6,5,1,7):         w(6)(5)(1)(7)#1362  = 0
 CustomerVisit(7,5,1,7):         w(7)(5)(1)(7)#1542  = 0
 CustomerVisit(8,5,1,7):         w(8)(5)(1)(7)#1722  = 0
 CustomerVisit(9,5,1,7):         w(9)(5)(1)(7)#1902  = 0
 CustomerVisit(10,5,1,7):        w(10)(5)(1)(7)#2082  = 0
 CustomerVisit(11,5,1,7):        w(11)(5)(1)(7)#2262  = 0
 CustomerVisit(12,5,1,7):        w(12)(5)(1)(7)#2442  = 0
 CustomerVisit(3,6,1,7):         w(3)(6)(1)(7)#852  = 0
 CustomerVisit(4,6,1,7):         w(4)(6)(1)(7)#1032  = 0
 CustomerVisit(5,6,1,7):         w(5)(6)(1)(7)#1212  = 0
 CustomerVisit(6,6,1,7):         w(6)(6)(1)(7)#1392  = 0
 CustomerVisit(7,6,1,7):         w(7)(6)(1)(7)#1572  = 0
 CustomerVisit(8,6,1,7):         w(8)(6)(1)(7)#1752  = 0
 CustomerVisit(9,6,1,7):         w(9)(6)(1)(7)#1932  = 0
 CustomerVisit(10,6,1,7):        w(10)(6)(1)(7)#2112  = 0
 CustomerVisit(11,6,1,7):        w(11)(6)(1)(7)#2292  = 0
 CustomerVisit(12,6,1,7):        w(12)(6)(1)(7)#2472  = 0
 CustomerVisit(3,4,2,7):         w(3)(4)(2)(7)#802  = 0
 CustomerVisit(4,4,2,7):         w(4)(4)(2)(7)#982  = 0
 CustomerVisit(5,4,2,7):         w(5)(4)(2)(7)#1162  = 0
 CustomerVisit(6,4,2,7):         w(6)(4)(2)(7)#1342  = 0
 CustomerVisit(7,4,2,7):         w(7)(4)(2)(7)#1522  = 0
 CustomerVisit(8,4,2,7):         w(8)(4)(2)(7)#1702  = 0
 CustomerVisit(9,4,2,7):         w(9)(4)(2)(7)#1882  = 0
 CustomerVisit(10,4,2,7):        w(10)(4)(2)(7)#2062  = 0
 CustomerVisit(11,4,2,7):        w(11)(4)(2)(7)#2242  = 0
 CustomerVisit(12,4,2,7):        w(12)(4)(2)(7)#2422  = 0
 CustomerVisit(3,5,2,7):         w(3)(5)(2)(7)#832 <= 20
 CustomerVisit(4,5,2,7):         w(4)(5)(2)(7)#1012 <= 32
 CustomerVisit(5,5,2,7):         w(5)(5)(2)(7)#1192  = 0
 CustomerVisit(6,5,2,7):         w(6)(5)(2)(7)#1372 <= 13
 CustomerVisit(7,5,2,7):         w(7)(5)(2)(7)#1552 <= 27
 CustomerVisit(8,5,2,7):         w(8)(5)(2)(7)#1732 <= 32
 CustomerVisit(9,5,2,7):         w(9)(5)(2)(7)#1912  = 0
 CustomerVisit(10,5,2,7):        w(10)(5)(2)(7)#2092 <= 26
 CustomerVisit(11,5,2,7):        w(11)(5)(2)(7)#2272  = 0
 CustomerVisit(12,5,2,7):        w(12)(5)(2)(7)#2452  = 0
 CustomerVisit(3,6,2,7):         w(3)(6)(2)(7)#862  = 0
 CustomerVisit(4,6,2,7):         w(4)(6)(2)(7)#1042  = 0
 CustomerVisit(5,6,2,7):         w(5)(6)(2)(7)#1222  = 0
 CustomerVisit(6,6,2,7):         w(6)(6)(2)(7)#1402  = 0
 CustomerVisit(7,6,2,7):         w(7)(6)(2)(7)#1582  = 0
 CustomerVisit(8,6,2,7):         w(8)(6)(2)(7)#1762  = 0
 CustomerVisit(9,6,2,7):         w(9)(6)(2)(7)#1942  = 0
 CustomerVisit(10,6,2,7):        w(10)(6)(2)(7)#2122  = 0
 CustomerVisit(11,6,2,7):        w(11)(6)(2)(7)#2302  = 0
 CustomerVisit(12,6,2,7):        w(12)(6)(2)(7)#2482  = 0
 CustomerVisit(3,4,3,7):         w(3)(4)(3)(7)#812  = 0
 CustomerVisit(4,4,3,7):         w(4)(4)(3)(7)#992  = 0
 CustomerVisit(5,4,3,7):         w(5)(4)(3)(7)#1172  = 0
 CustomerVisit(6,4,3,7):         w(6)(4)(3)(7)#1352  = 0
 CustomerVisit(7,4,3,7):         w(7)(4)(3)(7)#1532  = 0
 CustomerVisit(8,4,3,7):         w(8)(4)(3)(7)#1712  = 0
 CustomerVisit(9,4,3,7):         w(9)(4)(3)(7)#1892  = 0
 CustomerVisit(10,4,3,7):        w(10)(4)(3)(7)#2072  = 0
 CustomerVisit(11,4,3,7):        w(11)(4)(3)(7)#2252  = 0
 CustomerVisit(12,4,3,7):        w(12)(4)(3)(7)#2432  = 0
 CustomerVisit(3,5,3,7):         w(3)(5)(3)(7)#842  = 0
 CustomerVisit(4,5,3,7):         w(4)(5)(3)(7)#1022  = 0
 CustomerVisit(5,5,3,7):         w(5)(5)(3)(7)#1202  = 0
 CustomerVisit(6,5,3,7):         w(6)(5)(3)(7)#1382  = 0
 CustomerVisit(7,5,3,7):         w(7)(5)(3)(7)#1562  = 0
 CustomerVisit(8,5,3,7):         w(8)(5)(3)(7)#1742  = 0
 CustomerVisit(9,5,3,7):         w(9)(5)(3)(7)#1922  = 0
 CustomerVisit(10,5,3,7):        w(10)(5)(3)(7)#2102  = 0
 CustomerVisit(11,5,3,7):        w(11)(5)(3)(7)#2282  = 0
 CustomerVisit(12,5,3,7):        w(12)(5)(3)(7)#2462  = 0
 CustomerVisit(3,6,3,7):         w(3)(6)(3)(7)#872  = 0
 CustomerVisit(4,6,3,7):         w(4)(6)(3)(7)#1052  = 0
 CustomerVisit(5,6,3,7):         w(5)(6)(3)(7)#1232  = 0
 CustomerVisit(6,6,3,7):         w(6)(6)(3)(7)#1412  = 0
 CustomerVisit(7,6,3,7):         w(7)(6)(3)(7)#1592 <= 14
 CustomerVisit(8,6,3,7):         w(8)(6)(3)(7)#1772  = 0
 CustomerVisit(9,6,3,7):         w(9)(6)(3)(7)#1952  = 0
 CustomerVisit(10,6,3,7):        w(10)(6)(3)(7)#2132  = 0
 CustomerVisit(11,6,3,7):        w(11)(6)(3)(7)#2312  = 0
 CustomerVisit(12,6,3,7):        w(12)(6)(3)(7)#2492  = 0
 CustomerVisit(3,1,1,8):         w(3)(1)(1)(8)#703  = 0
 CustomerVisit(4,1,1,8):         w(4)(1)(1)(8)#883  = 0
 CustomerVisit(5,1,1,8):         w(5)(1)(1)(8)#1063  = 0
 CustomerVisit(6,1,1,8):         w(6)(1)(1)(8)#1243  = 0
 CustomerVisit(7,1,1,8):         w(7)(1)(1)(8)#1423  = 0
 CustomerVisit(8,1,1,8):         w(8)(1)(1)(8)#1603  = 0
 CustomerVisit(9,1,1,8):         w(9)(1)(1)(8)#1783  = 0
 CustomerVisit(10,1,1,8):        w(10)(1)(1)(8)#1963  = 0
 CustomerVisit(11,1,1,8):        w(11)(1)(1)(8)#2143  = 0
 CustomerVisit(12,1,1,8):        w(12)(1)(1)(8)#2323  = 0
 CustomerVisit(3,2,1,8):         w(3)(2)(1)(8)#733 <= 20
 CustomerVisit(4,2,1,8):         w(4)(2)(1)(8)#913  = 0
 CustomerVisit(5,2,1,8):         w(5)(2)(1)(8)#1093  = 0
 CustomerVisit(6,2,1,8):         w(6)(2)(1)(8)#1273  = 0
 CustomerVisit(7,2,1,8):         w(7)(2)(1)(8)#1453  = 0
 CustomerVisit(8,2,1,8):         w(8)(2)(1)(8)#1633  = 0
 CustomerVisit(9,2,1,8):         w(9)(2)(1)(8)#1813  = 0
 CustomerVisit(10,2,1,8):        w(10)(2)(1)(8)#1993  = 0
 CustomerVisit(11,2,1,8):        w(11)(2)(1)(8)#2173  = 0
 CustomerVisit(12,2,1,8):        w(12)(2)(1)(8)#2353  = 0
 CustomerVisit(3,3,1,8):         w(3)(3)(1)(8)#763  = 0
 CustomerVisit(4,3,1,8):         w(4)(3)(1)(8)#943  = 0
 CustomerVisit(5,3,1,8):         w(5)(3)(1)(8)#1123  = 0
 CustomerVisit(6,3,1,8):         w(6)(3)(1)(8)#1303 <= 14
 CustomerVisit(7,3,1,8):         w(7)(3)(1)(8)#1483  = 0
 CustomerVisit(8,3,1,8):         w(8)(3)(1)(8)#1663 <= 44
 CustomerVisit(9,3,1,8):         w(9)(3)(1)(8)#1843  = 0
 CustomerVisit(10,3,1,8):        w(10)(3)(1)(8)#2023 <= 26
 CustomerVisit(11,3,1,8):        w(11)(3)(1)(8)#2203 <= 44
 CustomerVisit(12,3,1,8):        w(12)(3)(1)(8)#2383 <= 44
 CustomerVisit(3,1,2,8):         w(3)(1)(2)(8)#713 <= 18
 CustomerVisit(4,1,2,8):         w(4)(1)(2)(8)#893  = 0
 CustomerVisit(5,1,2,8):         w(5)(1)(2)(8)#1073 <= 30
 CustomerVisit(6,1,2,8):         w(6)(1)(2)(8)#1253 <= 14
 CustomerVisit(7,1,2,8):         w(7)(1)(2)(8)#1433  = 0
 CustomerVisit(8,1,2,8):         w(8)(1)(2)(8)#1613  = 0
 CustomerVisit(9,1,2,8):         w(9)(1)(2)(8)#1793 <= 44
 CustomerVisit(10,1,2,8):        w(10)(1)(2)(8)#1973 <= 25
 CustomerVisit(11,1,2,8):        w(11)(1)(2)(8)#2153  = 0
 CustomerVisit(12,1,2,8):        w(12)(1)(2)(8)#2333  = 0
 CustomerVisit(3,2,2,8):         w(3)(2)(2)(8)#743  = 0
 CustomerVisit(4,2,2,8):         w(4)(2)(2)(8)#923  = 0
 CustomerVisit(5,2,2,8):         w(5)(2)(2)(8)#1103  = 0
 CustomerVisit(6,2,2,8):         w(6)(2)(2)(8)#1283  = 0
 CustomerVisit(7,2,2,8):         w(7)(2)(2)(8)#1463  = 0
 CustomerVisit(8,2,2,8):         w(8)(2)(2)(8)#1643  = 0
 CustomerVisit(9,2,2,8):         w(9)(2)(2)(8)#1823  = 0
 CustomerVisit(10,2,2,8):        w(10)(2)(2)(8)#2003  = 0
 CustomerVisit(11,2,2,8):        w(11)(2)(2)(8)#2183  = 0
 CustomerVisit(12,2,2,8):        w(12)(2)(2)(8)#2363  = 0
 CustomerVisit(3,3,2,8):         w(3)(3)(2)(8)#773  = 0
 CustomerVisit(4,3,2,8):         w(4)(3)(2)(8)#953  = 0
 CustomerVisit(5,3,2,8):         w(5)(3)(2)(8)#1133  = 0
 CustomerVisit(6,3,2,8):         w(6)(3)(2)(8)#1313  = 0
 CustomerVisit(7,3,2,8):         w(7)(3)(2)(8)#1493  = 0
 CustomerVisit(8,3,2,8):         w(8)(3)(2)(8)#1673  = 0
 CustomerVisit(9,3,2,8):         w(9)(3)(2)(8)#1853  = 0
 CustomerVisit(10,3,2,8):        w(10)(3)(2)(8)#2033  = 0
 CustomerVisit(11,3,2,8):        w(11)(3)(2)(8)#2213  = 0
 CustomerVisit(12,3,2,8):        w(12)(3)(2)(8)#2393  = 0
 CustomerVisit(3,1,3,8):         w(3)(1)(3)(8)#723  = 0
 CustomerVisit(4,1,3,8):         w(4)(1)(3)(8)#903  = 0
 CustomerVisit(5,1,3,8):         w(5)(1)(3)(8)#1083 <= 17
 CustomerVisit(6,1,3,8):         w(6)(1)(3)(8)#1263  = 0
 CustomerVisit(7,1,3,8):         w(7)(1)(3)(8)#1443  = 0
 CustomerVisit(8,1,3,8):         w(8)(1)(3)(8)#1623  = 0
 CustomerVisit(9,1,3,8):         w(9)(1)(3)(8)#1803  = 0
 CustomerVisit(10,1,3,8):        w(10)(1)(3)(8)#1983  = 0
 CustomerVisit(11,1,3,8):        w(11)(1)(3)(8)#2163  = 0
 CustomerVisit(12,1,3,8):        w(12)(1)(3)(8)#2343 <= 22
 CustomerVisit(3,2,3,8):         w(3)(2)(3)(8)#753  = 0
 CustomerVisit(4,2,3,8):         w(4)(2)(3)(8)#933  = 0
 CustomerVisit(5,2,3,8):         w(5)(2)(3)(8)#1113  = 0
 CustomerVisit(6,2,3,8):         w(6)(2)(3)(8)#1293  = 0
 CustomerVisit(7,2,3,8):         w(7)(2)(3)(8)#1473  = 0
 CustomerVisit(8,2,3,8):         w(8)(2)(3)(8)#1653  = 0
 CustomerVisit(9,2,3,8):         w(9)(2)(3)(8)#1833  = 0
 CustomerVisit(10,2,3,8):        w(10)(2)(3)(8)#2013  = 0
 CustomerVisit(11,2,3,8):        w(11)(2)(3)(8)#2193  = 0
 CustomerVisit(12,2,3,8):        w(12)(2)(3)(8)#2373  = 0
 CustomerVisit(3,3,3,8):         w(3)(3)(3)(8)#783  = 0
 CustomerVisit(4,3,3,8):         w(4)(3)(3)(8)#963  = 0
 CustomerVisit(5,3,3,8):         w(5)(3)(3)(8)#1143  = 0
 CustomerVisit(6,3,3,8):         w(6)(3)(3)(8)#1323  = 0
 CustomerVisit(7,3,3,8):         w(7)(3)(3)(8)#1503  = 0
 CustomerVisit(8,3,3,8):         w(8)(3)(3)(8)#1683  = 0
 CustomerVisit(9,3,3,8):         w(9)(3)(3)(8)#1863  = 0
 CustomerVisit(10,3,3,8):        w(10)(3)(3)(8)#2043  = 0
 CustomerVisit(11,3,3,8):        w(11)(3)(3)(8)#2223  = 0
 CustomerVisit(12,3,3,8):        w(12)(3)(3)(8)#2403  = 0
 CustomerVisit(3,4,1,8):         w(3)(4)(1)(8)#793  = 0
 CustomerVisit(4,4,1,8):         w(4)(4)(1)(8)#973  = 0
 CustomerVisit(5,4,1,8):         w(5)(4)(1)(8)#1153  = 0
 CustomerVisit(6,4,1,8):         w(6)(4)(1)(8)#1333  = 0
 CustomerVisit(7,4,1,8):         w(7)(4)(1)(8)#1513 <= 39
 CustomerVisit(8,4,1,8):         w(8)(4)(1)(8)#1693  = 0
 CustomerVisit(9,4,1,8):         w(9)(4)(1)(8)#1873  = 0
 CustomerVisit(10,4,1,8):        w(10)(4)(1)(8)#2053  = 0
 CustomerVisit(11,4,1,8):        w(11)(4)(1)(8)#2233  = 0
 CustomerVisit(12,4,1,8):        w(12)(4)(1)(8)#2413  = 0
 CustomerVisit(3,5,1,8):         w(3)(5)(1)(8)#823  = 0
 CustomerVisit(4,5,1,8):         w(4)(5)(1)(8)#1003  = 0
 CustomerVisit(5,5,1,8):         w(5)(5)(1)(8)#1183  = 0
 CustomerVisit(6,5,1,8):         w(6)(5)(1)(8)#1363  = 0
 CustomerVisit(7,5,1,8):         w(7)(5)(1)(8)#1543  = 0
 CustomerVisit(8,5,1,8):         w(8)(5)(1)(8)#1723  = 0
 CustomerVisit(9,5,1,8):         w(9)(5)(1)(8)#1903  = 0
 CustomerVisit(10,5,1,8):        w(10)(5)(1)(8)#2083  = 0
 CustomerVisit(11,5,1,8):        w(11)(5)(1)(8)#2263  = 0
 CustomerVisit(12,5,1,8):        w(12)(5)(1)(8)#2443  = 0
 CustomerVisit(3,6,1,8):         w(3)(6)(1)(8)#853  = 0
 CustomerVisit(4,6,1,8):         w(4)(6)(1)(8)#1033  = 0
 CustomerVisit(5,6,1,8):         w(5)(6)(1)(8)#1213  = 0
 CustomerVisit(6,6,1,8):         w(6)(6)(1)(8)#1393  = 0
 CustomerVisit(7,6,1,8):         w(7)(6)(1)(8)#1573  = 0
 CustomerVisit(8,6,1,8):         w(8)(6)(1)(8)#1753  = 0
 CustomerVisit(9,6,1,8):         w(9)(6)(1)(8)#1933  = 0
 CustomerVisit(10,6,1,8):        w(10)(6)(1)(8)#2113  = 0
 CustomerVisit(11,6,1,8):        w(11)(6)(1)(8)#2293  = 0
 CustomerVisit(12,6,1,8):        w(12)(6)(1)(8)#2473  = 0
 CustomerVisit(3,4,2,8):         w(3)(4)(2)(8)#803  = 0
 CustomerVisit(4,4,2,8):         w(4)(4)(2)(8)#983 <= 29
 CustomerVisit(5,4,2,8):         w(5)(4)(2)(8)#1163  = 0
 CustomerVisit(6,4,2,8):         w(6)(4)(2)(8)#1343  = 0
 CustomerVisit(7,4,2,8):         w(7)(4)(2)(8)#1523 <= 26
 CustomerVisit(8,4,2,8):         w(8)(4)(2)(8)#1703  = 0
 CustomerVisit(9,4,2,8):         w(9)(4)(2)(8)#1883  = 0
 CustomerVisit(10,4,2,8):        w(10)(4)(2)(8)#2063  = 0
 CustomerVisit(11,4,2,8):        w(11)(4)(2)(8)#2243  = 0
 CustomerVisit(12,4,2,8):        w(12)(4)(2)(8)#2423  = 0
 CustomerVisit(3,5,2,8):         w(3)(5)(2)(8)#833  = 0
 CustomerVisit(4,5,2,8):         w(4)(5)(2)(8)#1013  = 0
 CustomerVisit(5,5,2,8):         w(5)(5)(2)(8)#1193  = 0
 CustomerVisit(6,5,2,8):         w(6)(5)(2)(8)#1373  = 0
 CustomerVisit(7,5,2,8):         w(7)(5)(2)(8)#1553  = 0
 CustomerVisit(8,5,2,8):         w(8)(5)(2)(8)#1733  = 0
 CustomerVisit(9,5,2,8):         w(9)(5)(2)(8)#1913  = 0
 CustomerVisit(10,5,2,8):        w(10)(5)(2)(8)#2093  = 0
 CustomerVisit(11,5,2,8):        w(11)(5)(2)(8)#2273  = 0
 CustomerVisit(12,5,2,8):        w(12)(5)(2)(8)#2453  = 0
 CustomerVisit(3,6,2,8):         w(3)(6)(2)(8)#863  = 0
 CustomerVisit(4,6,2,8):         w(4)(6)(2)(8)#1043  = 0
 CustomerVisit(5,6,2,8):         w(5)(6)(2)(8)#1223  = 0
 CustomerVisit(6,6,2,8):         w(6)(6)(2)(8)#1403  = 0
 CustomerVisit(7,6,2,8):         w(7)(6)(2)(8)#1583  = 0
 CustomerVisit(8,6,2,8):         w(8)(6)(2)(8)#1763  = 0
 CustomerVisit(9,6,2,8):         w(9)(6)(2)(8)#1943  = 0
 CustomerVisit(10,6,2,8):        w(10)(6)(2)(8)#2123  = 0
 CustomerVisit(11,6,2,8):        w(11)(6)(2)(8)#2303  = 0
 CustomerVisit(12,6,2,8):        w(12)(6)(2)(8)#2483  = 0
 CustomerVisit(3,4,3,8):         w(3)(4)(3)(8)#813 <= 8
 CustomerVisit(4,4,3,8):         w(4)(4)(3)(8)#993 <= 16
 CustomerVisit(5,4,3,8):         w(5)(4)(3)(8)#1173  = 0
 CustomerVisit(6,4,3,8):         w(6)(4)(3)(8)#1353 <= 7
 CustomerVisit(7,4,3,8):         w(7)(4)(3)(8)#1533  = 0
 CustomerVisit(8,4,3,8):         w(8)(4)(3)(8)#1713  = 0
 CustomerVisit(9,4,3,8):         w(9)(4)(3)(8)#1893  = 0
 CustomerVisit(10,4,3,8):        w(10)(4)(3)(8)#2073 <= 14
 CustomerVisit(11,4,3,8):        w(11)(4)(3)(8)#2253 <= 19
 CustomerVisit(12,4,3,8):        w(12)(4)(3)(8)#2433  = 0
 CustomerVisit(3,5,3,8):         w(3)(5)(3)(8)#843  = 0
 CustomerVisit(4,5,3,8):         w(4)(5)(3)(8)#1023  = 0
 CustomerVisit(5,5,3,8):         w(5)(5)(3)(8)#1203  = 0
 CustomerVisit(6,5,3,8):         w(6)(5)(3)(8)#1383  = 0
 CustomerVisit(7,5,3,8):         w(7)(5)(3)(8)#1563  = 0
 CustomerVisit(8,5,3,8):         w(8)(5)(3)(8)#1743  = 0
 CustomerVisit(9,5,3,8):         w(9)(5)(3)(8)#1923  = 0
 CustomerVisit(10,5,3,8):        w(10)(5)(3)(8)#2103  = 0
 CustomerVisit(11,5,3,8):        w(11)(5)(3)(8)#2283  = 0
 CustomerVisit(12,5,3,8):        w(12)(5)(3)(8)#2463  = 0
 CustomerVisit(3,6,3,8):         w(3)(6)(3)(8)#873  = 0
 CustomerVisit(4,6,3,8):         w(4)(6)(3)(8)#1053  = 0
 CustomerVisit(5,6,3,8):         w(5)(6)(3)(8)#1233  = 0
 CustomerVisit(6,6,3,8):         w(6)(6)(3)(8)#1413  = 0
 CustomerVisit(7,6,3,8):         w(7)(6)(3)(8)#1593  = 0
 CustomerVisit(8,6,3,8):         w(8)(6)(3)(8)#1773  = 0
 CustomerVisit(9,6,3,8):         w(9)(6)(3)(8)#1953  = 0
 CustomerVisit(10,6,3,8):        w(10)(6)(3)(8)#2133  = 0
 CustomerVisit(11,6,3,8):        w(11)(6)(3)(8)#2313  = 0
 CustomerVisit(12,6,3,8):        w(12)(6)(3)(8)#2493  = 0
 CustomerVisit(3,1,1,9):         w(3)(1)(1)(9)#704  = 0
 CustomerVisit(4,1,1,9):         w(4)(1)(1)(9)#884  = 0
 CustomerVisit(5,1,1,9):         w(5)(1)(1)(9)#1064  = 0
 CustomerVisit(6,1,1,9):         w(6)(1)(1)(9)#1244 <= 14
 CustomerVisit(7,1,1,9):         w(7)(1)(1)(9)#1424  = 0
 CustomerVisit(8,1,1,9):         w(8)(1)(1)(9)#1604  = 0
 CustomerVisit(9,1,1,9):         w(9)(1)(1)(9)#1784  = 0
 CustomerVisit(10,1,1,9):        w(10)(1)(1)(9)#1964 <= 26
 CustomerVisit(11,1,1,9):        w(11)(1)(1)(9)#2144  = 0
 CustomerVisit(12,1,1,9):        w(12)(1)(1)(9)#2324 <= 44
 CustomerVisit(3,2,1,9):         w(3)(2)(1)(9)#734  = 0
 CustomerVisit(4,2,1,9):         w(4)(2)(1)(9)#914  = 0
 CustomerVisit(5,2,1,9):         w(5)(2)(1)(9)#1094  = 0
 CustomerVisit(6,2,1,9):         w(6)(2)(1)(9)#1274  = 0
 CustomerVisit(7,2,1,9):         w(7)(2)(1)(9)#1454  = 0
 CustomerVisit(8,2,1,9):         w(8)(2)(1)(9)#1634  = 0
 CustomerVisit(9,2,1,9):         w(9)(2)(1)(9)#1814  = 0
 CustomerVisit(10,2,1,9):        w(10)(2)(1)(9)#1994  = 0
 CustomerVisit(11,2,1,9):        w(11)(2)(1)(9)#2174  = 0
 CustomerVisit(12,2,1,9):        w(12)(2)(1)(9)#2354  = 0
 CustomerVisit(3,3,1,9):         w(3)(3)(1)(9)#764  = 0
 CustomerVisit(4,3,1,9):         w(4)(3)(1)(9)#944  = 0
 CustomerVisit(5,3,1,9):         w(5)(3)(1)(9)#1124  = 0
 CustomerVisit(6,3,1,9):         w(6)(3)(1)(9)#1304  = 0
 CustomerVisit(7,3,1,9):         w(7)(3)(1)(9)#1484  = 0
 CustomerVisit(8,3,1,9):         w(8)(3)(1)(9)#1664  = 0
 CustomerVisit(9,3,1,9):         w(9)(3)(1)(9)#1844  = 0
 CustomerVisit(10,3,1,9):        w(10)(3)(1)(9)#2024  = 0
 CustomerVisit(11,3,1,9):        w(11)(3)(1)(9)#2204  = 0
 CustomerVisit(12,3,1,9):        w(12)(3)(1)(9)#2384  = 0
 CustomerVisit(3,1,2,9):         w(3)(1)(2)(9)#714  = 0
 CustomerVisit(4,1,2,9):         w(4)(1)(2)(9)#894  = 0
 CustomerVisit(5,1,2,9):         w(5)(1)(2)(9)#1074  = 0
 CustomerVisit(6,1,2,9):         w(6)(1)(2)(9)#1254  = 0
 CustomerVisit(7,1,2,9):         w(7)(1)(2)(9)#1434  = 0
 CustomerVisit(8,1,2,9):         w(8)(1)(2)(9)#1614  = 0
 CustomerVisit(9,1,2,9):         w(9)(1)(2)(9)#1794  = 0
 CustomerVisit(10,1,2,9):        w(10)(1)(2)(9)#1974  = 0
 CustomerVisit(11,1,2,9):        w(11)(1)(2)(9)#2154  = 0
 CustomerVisit(12,1,2,9):        w(12)(1)(2)(9)#2334  = 0
 CustomerVisit(3,2,2,9):         w(3)(2)(2)(9)#744  = 0
 CustomerVisit(4,2,2,9):         w(4)(2)(2)(9)#924  = 0
 CustomerVisit(5,2,2,9):         w(5)(2)(2)(9)#1104  = 0
 CustomerVisit(6,2,2,9):         w(6)(2)(2)(9)#1284 <= 13
 CustomerVisit(7,2,2,9):         w(7)(2)(2)(9)#1464  = 0
 CustomerVisit(8,2,2,9):         w(8)(2)(2)(9)#1644  = 0
 CustomerVisit(9,2,2,9):         w(9)(2)(2)(9)#1824  = 0
 CustomerVisit(10,2,2,9):        w(10)(2)(2)(9)#2004 <= 26
 CustomerVisit(11,2,2,9):        w(11)(2)(2)(9)#2184  = 0
 CustomerVisit(12,2,2,9):        w(12)(2)(2)(9)#2364  = 0
 CustomerVisit(3,3,2,9):         w(3)(3)(2)(9)#774 <= 18
 CustomerVisit(4,3,2,9):         w(4)(3)(2)(9)#954  = 0
 CustomerVisit(5,3,2,9):         w(5)(3)(2)(9)#1134 <= 30
 CustomerVisit(6,3,2,9):         w(6)(3)(2)(9)#1314  = 0
 CustomerVisit(7,3,2,9):         w(7)(3)(2)(9)#1494  = 0
 CustomerVisit(8,3,2,9):         w(8)(3)(2)(9)#1674  = 0
 CustomerVisit(9,3,2,9):         w(9)(3)(2)(9)#1854 <= 44
 CustomerVisit(10,3,2,9):        w(10)(3)(2)(9)#2034  = 0
 CustomerVisit(11,3,2,9):        w(11)(3)(2)(9)#2214  = 0
 CustomerVisit(12,3,2,9):        w(12)(3)(2)(9)#2394 <= 44
 CustomerVisit(3,1,3,9):         w(3)(1)(3)(9)#724 <= 8
 CustomerVisit(4,1,3,9):         w(4)(1)(3)(9)#904  = 0
 CustomerVisit(5,1,3,9):         w(5)(1)(3)(9)#1084 <= 13
 CustomerVisit(6,1,3,9):         w(6)(1)(3)(9)#1264 <= 6
 CustomerVisit(7,1,3,9):         w(7)(1)(3)(9)#1444  = 0
 CustomerVisit(8,1,3,9):         w(8)(1)(3)(9)#1624  = 0
 CustomerVisit(9,1,3,9):         w(9)(1)(3)(9)#1804  = 0
 CustomerVisit(10,1,3,9):        w(10)(1)(3)(9)#1984 <= 15
 CustomerVisit(11,1,3,9):        w(11)(1)(3)(9)#2164 <= 19
 CustomerVisit(12,1,3,9):        w(12)(1)(3)(9)#2344  = 0
 CustomerVisit(3,2,3,9):         w(3)(2)(3)(9)#754  = 0
 CustomerVisit(4,2,3,9):         w(4)(2)(3)(9)#934  = 0
 CustomerVisit(5,2,3,9):         w(5)(2)(3)(9)#1114  = 0
 CustomerVisit(6,2,3,9):         w(6)(2)(3)(9)#1294  = 0
 CustomerVisit(7,2,3,9):         w(7)(2)(3)(9)#1474  = 0
 CustomerVisit(8,2,3,9):         w(8)(2)(3)(9)#1654  = 0
 CustomerVisit(9,2,3,9):         w(9)(2)(3)(9)#1834  = 0
 CustomerVisit(10,2,3,9):        w(10)(2)(3)(9)#2014  = 0
 CustomerVisit(11,2,3,9):        w(11)(2)(3)(9)#2194  = 0
 CustomerVisit(12,2,3,9):        w(12)(2)(3)(9)#2374  = 0
 CustomerVisit(3,3,3,9):         w(3)(3)(3)(9)#784  = 0
 CustomerVisit(4,3,3,9):         w(4)(3)(3)(9)#964  = 0
 CustomerVisit(5,3,3,9):         w(5)(3)(3)(9)#1144  = 0
 CustomerVisit(6,3,3,9):         w(6)(3)(3)(9)#1324  = 0
 CustomerVisit(7,3,3,9):         w(7)(3)(3)(9)#1504  = 0
 CustomerVisit(8,3,3,9):         w(8)(3)(3)(9)#1684  = 0
 CustomerVisit(9,3,3,9):         w(9)(3)(3)(9)#1864  = 0
 CustomerVisit(10,3,3,9):        w(10)(3)(3)(9)#2044  = 0
 CustomerVisit(11,3,3,9):        w(11)(3)(3)(9)#2224  = 0
 CustomerVisit(12,3,3,9):        w(12)(3)(3)(9)#2404  = 0
 CustomerVisit(3,4,1,9):         w(3)(4)(1)(9)#794  = 0
 CustomerVisit(4,4,1,9):         w(4)(4)(1)(9)#974  = 0
 CustomerVisit(5,4,1,9):         w(5)(4)(1)(9)#1154  = 0
 CustomerVisit(6,4,1,9):         w(6)(4)(1)(9)#1334  = 0
 CustomerVisit(7,4,1,9):         w(7)(4)(1)(9)#1514  = 0
 CustomerVisit(8,4,1,9):         w(8)(4)(1)(9)#1694  = 0
 CustomerVisit(9,4,1,9):         w(9)(4)(1)(9)#1874  = 0
 CustomerVisit(10,4,1,9):        w(10)(4)(1)(9)#2054  = 0
 CustomerVisit(11,4,1,9):        w(11)(4)(1)(9)#2234  = 0
 CustomerVisit(12,4,1,9):        w(12)(4)(1)(9)#2414  = 0
 CustomerVisit(3,5,1,9):         w(3)(5)(1)(9)#824 <= 20
 CustomerVisit(4,5,1,9):         w(4)(5)(1)(9)#1004 <= 44
 CustomerVisit(5,5,1,9):         w(5)(5)(1)(9)#1184  = 0
 CustomerVisit(6,5,1,9):         w(6)(5)(1)(9)#1364  = 0
 CustomerVisit(7,5,1,9):         w(7)(5)(1)(9)#1544 <= 39
 CustomerVisit(8,5,1,9):         w(8)(5)(1)(9)#1724  = 0
 CustomerVisit(9,5,1,9):         w(9)(5)(1)(9)#1904  = 0
 CustomerVisit(10,5,1,9):        w(10)(5)(1)(9)#2084  = 0
 CustomerVisit(11,5,1,9):        w(11)(5)(1)(9)#2264  = 0
 CustomerVisit(12,5,1,9):        w(12)(5)(1)(9)#2444  = 0
 CustomerVisit(3,6,1,9):         w(3)(6)(1)(9)#854  = 0
 CustomerVisit(4,6,1,9):         w(4)(6)(1)(9)#1034  = 0
 CustomerVisit(5,6,1,9):         w(5)(6)(1)(9)#1214  = 0
 CustomerVisit(6,6,1,9):         w(6)(6)(1)(9)#1394  = 0
 CustomerVisit(7,6,1,9):         w(7)(6)(1)(9)#1574  = 0
 CustomerVisit(8,6,1,9):         w(8)(6)(1)(9)#1754  = 0
 CustomerVisit(9,6,1,9):         w(9)(6)(1)(9)#1934  = 0
 CustomerVisit(10,6,1,9):        w(10)(6)(1)(9)#2114  = 0
 CustomerVisit(11,6,1,9):        w(11)(6)(1)(9)#2294  = 0
 CustomerVisit(12,6,1,9):        w(12)(6)(1)(9)#2474  = 0
 CustomerVisit(3,4,2,9):         w(3)(4)(2)(9)#804  = 0
 CustomerVisit(4,4,2,9):         w(4)(4)(2)(9)#984  = 0
 CustomerVisit(5,4,2,9):         w(5)(4)(2)(9)#1164  = 0
 CustomerVisit(6,4,2,9):         w(6)(4)(2)(9)#1344  = 0
 CustomerVisit(7,4,2,9):         w(7)(4)(2)(9)#1524  = 0
 CustomerVisit(8,4,2,9):         w(8)(4)(2)(9)#1704  = 0
 CustomerVisit(9,4,2,9):         w(9)(4)(2)(9)#1884  = 0
 CustomerVisit(10,4,2,9):        w(10)(4)(2)(9)#2064  = 0
 CustomerVisit(11,4,2,9):        w(11)(4)(2)(9)#2244  = 0
 CustomerVisit(12,4,2,9):        w(12)(4)(2)(9)#2424  = 0
 CustomerVisit(3,5,2,9):         w(3)(5)(2)(9)#834  = 0
 CustomerVisit(4,5,2,9):         w(4)(5)(2)(9)#1014  = 0
 CustomerVisit(5,5,2,9):         w(5)(5)(2)(9)#1194  = 0
 CustomerVisit(6,5,2,9):         w(6)(5)(2)(9)#1374  = 0
 CustomerVisit(7,5,2,9):         w(7)(5)(2)(9)#1554 <= 26
 CustomerVisit(8,5,2,9):         w(8)(5)(2)(9)#1734  = 0
 CustomerVisit(9,5,2,9):         w(9)(5)(2)(9)#1914  = 0
 CustomerVisit(10,5,2,9):        w(10)(5)(2)(9)#2094  = 0
 CustomerVisit(11,5,2,9):        w(11)(5)(2)(9)#2274  = 0
 CustomerVisit(12,5,2,9):        w(12)(5)(2)(9)#2454  = 0
 CustomerVisit(3,6,2,9):         w(3)(6)(2)(9)#864  = 0
 CustomerVisit(4,6,2,9):         w(4)(6)(2)(9)#1044  = 0
 CustomerVisit(5,6,2,9):         w(5)(6)(2)(9)#1224  = 0
 CustomerVisit(6,6,2,9):         w(6)(6)(2)(9)#1404  = 0
 CustomerVisit(7,6,2,9):         w(7)(6)(2)(9)#1584  = 0
 CustomerVisit(8,6,2,9):         w(8)(6)(2)(9)#1764  = 0
 CustomerVisit(9,6,2,9):         w(9)(6)(2)(9)#1944  = 0
 CustomerVisit(10,6,2,9):        w(10)(6)(2)(9)#2124  = 0
 CustomerVisit(11,6,2,9):        w(11)(6)(2)(9)#2304  = 0
 CustomerVisit(12,6,2,9):        w(12)(6)(2)(9)#2484  = 0
 CustomerVisit(3,4,3,9):         w(3)(4)(3)(9)#814  = 0
 CustomerVisit(4,4,3,9):         w(4)(4)(3)(9)#994  = 0
 CustomerVisit(5,4,3,9):         w(5)(4)(3)(9)#1174  = 0
 CustomerVisit(6,4,3,9):         w(6)(4)(3)(9)#1354  = 0
 CustomerVisit(7,4,3,9):         w(7)(4)(3)(9)#1534  = 0
 CustomerVisit(8,4,3,9):         w(8)(4)(3)(9)#1714  = 0
 CustomerVisit(9,4,3,9):         w(9)(4)(3)(9)#1894  = 0
 CustomerVisit(10,4,3,9):        w(10)(4)(3)(9)#2074  = 0
 CustomerVisit(11,4,3,9):        w(11)(4)(3)(9)#2254  = 0
 CustomerVisit(12,4,3,9):        w(12)(4)(3)(9)#2434  = 0
 CustomerVisit(3,5,3,9):         w(3)(5)(3)(9)#844  = 0
 CustomerVisit(4,5,3,9):         w(4)(5)(3)(9)#1024 <= 18
 CustomerVisit(5,5,3,9):         w(5)(5)(3)(9)#1204  = 0
 CustomerVisit(6,5,3,9):         w(6)(5)(3)(9)#1384  = 0
 CustomerVisit(7,5,3,9):         w(7)(5)(3)(9)#1564  = 0
 CustomerVisit(8,5,3,9):         w(8)(5)(3)(9)#1744  = 0
 CustomerVisit(9,5,3,9):         w(9)(5)(3)(9)#1924  = 0
 CustomerVisit(10,5,3,9):        w(10)(5)(3)(9)#2104  = 0
 CustomerVisit(11,5,3,9):        w(11)(5)(3)(9)#2284  = 0
 CustomerVisit(12,5,3,9):        w(12)(5)(3)(9)#2464  = 0
 CustomerVisit(3,6,3,9):         w(3)(6)(3)(9)#874  = 0
 CustomerVisit(4,6,3,9):         w(4)(6)(3)(9)#1054  = 0
 CustomerVisit(5,6,3,9):         w(5)(6)(3)(9)#1234  = 0
 CustomerVisit(6,6,3,9):         w(6)(6)(3)(9)#1414  = 0
 CustomerVisit(7,6,3,9):         w(7)(6)(3)(9)#1594  = 0
 CustomerVisit(8,6,3,9):         w(8)(6)(3)(9)#1774 <= 13
 CustomerVisit(9,6,3,9):         w(9)(6)(3)(9)#1954  = 0
 CustomerVisit(10,6,3,9):        w(10)(6)(3)(9)#2134  = 0
 CustomerVisit(11,6,3,9):        w(11)(6)(3)(9)#2314  = 0
 CustomerVisit(12,6,3,9):        w(12)(6)(3)(9)#2494  = 0
 CustomerVisit(3,1,1,10):        w(3)(1)(1)(10)#705 <= 20
 CustomerVisit(4,1,1,10):        w(4)(1)(1)(10)#885  = 0
 CustomerVisit(5,1,1,10):        w(5)(1)(1)(10)#1065  = 0
 CustomerVisit(6,1,1,10):        w(6)(1)(1)(10)#1245 <= 14
 CustomerVisit(7,1,1,10):        w(7)(1)(1)(10)#1425  = 0
 CustomerVisit(8,1,1,10):        w(8)(1)(1)(10)#1605  = 0
 CustomerVisit(9,1,1,10):        w(9)(1)(1)(10)#1785  = 0
 CustomerVisit(10,1,1,10):       w(10)(1)(1)(10)#1965 <= 26
 CustomerVisit(11,1,1,10):       w(11)(1)(1)(10)#2145  = 0
 CustomerVisit(12,1,1,10):       w(12)(1)(1)(10)#2325  = 0
 CustomerVisit(3,2,1,10):        w(3)(2)(1)(10)#735  = 0
 CustomerVisit(4,2,1,10):        w(4)(2)(1)(10)#915  = 0
 CustomerVisit(5,2,1,10):        w(5)(2)(1)(10)#1095  = 0
 CustomerVisit(6,2,1,10):        w(6)(2)(1)(10)#1275  = 0
 CustomerVisit(7,2,1,10):        w(7)(2)(1)(10)#1455  = 0
 CustomerVisit(8,2,1,10):        w(8)(2)(1)(10)#1635  = 0
 CustomerVisit(9,2,1,10):        w(9)(2)(1)(10)#1815  = 0
 CustomerVisit(10,2,1,10):       w(10)(2)(1)(10)#1995  = 0
 CustomerVisit(11,2,1,10):       w(11)(2)(1)(10)#2175  = 0
 CustomerVisit(12,2,1,10):       w(12)(2)(1)(10)#2355  = 0
 CustomerVisit(3,3,1,10):        w(3)(3)(1)(10)#765  = 0
 CustomerVisit(4,3,1,10):        w(4)(3)(1)(10)#945  = 0
 CustomerVisit(5,3,1,10):        w(5)(3)(1)(10)#1125  = 0
 CustomerVisit(6,3,1,10):        w(6)(3)(1)(10)#1305  = 0
 CustomerVisit(7,3,1,10):        w(7)(3)(1)(10)#1485  = 0
 CustomerVisit(8,3,1,10):        w(8)(3)(1)(10)#1665  = 0
 CustomerVisit(9,3,1,10):        w(9)(3)(1)(10)#1845  = 0
 CustomerVisit(10,3,1,10):       w(10)(3)(1)(10)#2025  = 0
 CustomerVisit(11,3,1,10):       w(11)(3)(1)(10)#2205  = 0
 CustomerVisit(12,3,1,10):       w(12)(3)(1)(10)#2385  = 0
 CustomerVisit(3,1,2,10):        w(3)(1)(2)(10)#715  = 0
 CustomerVisit(4,1,2,10):        w(4)(1)(2)(10)#895  = 0
 CustomerVisit(5,1,2,10):        w(5)(1)(2)(10)#1075  = 0
 CustomerVisit(6,1,2,10):        w(6)(1)(2)(10)#1255 <= 14
 CustomerVisit(7,1,2,10):        w(7)(1)(2)(10)#1435  = 0
 CustomerVisit(8,1,2,10):        w(8)(1)(2)(10)#1615 <= 30
 CustomerVisit(9,1,2,10):        w(9)(1)(2)(10)#1795 <= 44
 CustomerVisit(10,1,2,10):       w(10)(1)(2)(10)#1975 <= 26
 CustomerVisit(11,1,2,10):       w(11)(1)(2)(10)#2155 <= 35
 CustomerVisit(12,1,2,10):       w(12)(1)(2)(10)#2335  = 0
 CustomerVisit(3,2,2,10):        w(3)(2)(2)(10)#745  = 0
 CustomerVisit(4,2,2,10):        w(4)(2)(2)(10)#925  = 0
 CustomerVisit(5,2,2,10):        w(5)(2)(2)(10)#1105  = 0
 CustomerVisit(6,2,2,10):        w(6)(2)(2)(10)#1285  = 0
 CustomerVisit(7,2,2,10):        w(7)(2)(2)(10)#1465  = 0
 CustomerVisit(8,2,2,10):        w(8)(2)(2)(10)#1645  = 0
 CustomerVisit(9,2,2,10):        w(9)(2)(2)(10)#1825  = 0
 CustomerVisit(10,2,2,10):       w(10)(2)(2)(10)#2005  = 0
 CustomerVisit(11,2,2,10):       w(11)(2)(2)(10)#2185  = 0
 CustomerVisit(12,2,2,10):       w(12)(2)(2)(10)#2365  = 0
 CustomerVisit(3,3,2,10):        w(3)(3)(2)(10)#775  = 0
 CustomerVisit(4,3,2,10):        w(4)(3)(2)(10)#955  = 0
 CustomerVisit(5,3,2,10):        w(5)(3)(2)(10)#1135 <= 29
 CustomerVisit(6,3,2,10):        w(6)(3)(2)(10)#1315  = 0
 CustomerVisit(7,3,2,10):        w(7)(3)(2)(10)#1495  = 0
 CustomerVisit(8,3,2,10):        w(8)(3)(2)(10)#1675  = 0
 CustomerVisit(9,3,2,10):        w(9)(3)(2)(10)#1855  = 0
 CustomerVisit(10,3,2,10):       w(10)(3)(2)(10)#2035  = 0
 CustomerVisit(11,3,2,10):       w(11)(3)(2)(10)#2215  = 0
 CustomerVisit(12,3,2,10):       w(12)(3)(2)(10)#2395  = 0
 CustomerVisit(3,1,3,10):        w(3)(1)(3)(10)#725  = 0
 CustomerVisit(4,1,3,10):        w(4)(1)(3)(10)#905  = 0
 CustomerVisit(5,1,3,10):        w(5)(1)(3)(10)#1085 <= 16
 CustomerVisit(6,1,3,10):        w(6)(1)(3)(10)#1265  = 0
 CustomerVisit(7,1,3,10):        w(7)(1)(3)(10)#1445  = 0
 CustomerVisit(8,1,3,10):        w(8)(1)(3)(10)#1625  = 0
 CustomerVisit(9,1,3,10):        w(9)(1)(3)(10)#1805  = 0
 CustomerVisit(10,1,3,10):       w(10)(1)(3)(10)#1985  = 0
 CustomerVisit(11,1,3,10):       w(11)(1)(3)(10)#2165  = 0
 CustomerVisit(12,1,3,10):       w(12)(1)(3)(10)#2345 <= 25
 CustomerVisit(3,2,3,10):        w(3)(2)(3)(10)#755  = 0
 CustomerVisit(4,2,3,10):        w(4)(2)(3)(10)#935  = 0
 CustomerVisit(5,2,3,10):        w(5)(2)(3)(10)#1115  = 0
 CustomerVisit(6,2,3,10):        w(6)(2)(3)(10)#1295  = 0
 CustomerVisit(7,2,3,10):        w(7)(2)(3)(10)#1475  = 0
 CustomerVisit(8,2,3,10):        w(8)(2)(3)(10)#1655  = 0
 CustomerVisit(9,2,3,10):        w(9)(2)(3)(10)#1835  = 0
 CustomerVisit(10,2,3,10):       w(10)(2)(3)(10)#2015  = 0
 CustomerVisit(11,2,3,10):       w(11)(2)(3)(10)#2195  = 0
 CustomerVisit(12,2,3,10):       w(12)(2)(3)(10)#2375  = 0
 CustomerVisit(3,3,3,10):        w(3)(3)(3)(10)#785  = 0
 CustomerVisit(4,3,3,10):        w(4)(3)(3)(10)#965  = 0
 CustomerVisit(5,3,3,10):        w(5)(3)(3)(10)#1145  = 0
 CustomerVisit(6,3,3,10):        w(6)(3)(3)(10)#1325  = 0
 CustomerVisit(7,3,3,10):        w(7)(3)(3)(10)#1505  = 0
 CustomerVisit(8,3,3,10):        w(8)(3)(3)(10)#1685  = 0
 CustomerVisit(9,3,3,10):        w(9)(3)(3)(10)#1865  = 0
 CustomerVisit(10,3,3,10):       w(10)(3)(3)(10)#2045  = 0
 CustomerVisit(11,3,3,10):       w(11)(3)(3)(10)#2225  = 0
 CustomerVisit(12,3,3,10):       w(12)(3)(3)(10)#2405  = 0
 CustomerVisit(3,4,1,10):        w(3)(4)(1)(10)#795  = 0
 CustomerVisit(4,4,1,10):        w(4)(4)(1)(10)#975 <= 42
 CustomerVisit(5,4,1,10):        w(5)(4)(1)(10)#1155  = 0
 CustomerVisit(6,4,1,10):        w(6)(4)(1)(10)#1335  = 0
 CustomerVisit(7,4,1,10):        w(7)(4)(1)(10)#1515 <= 39
 CustomerVisit(8,4,1,10):        w(8)(4)(1)(10)#1695  = 0
 CustomerVisit(9,4,1,10):        w(9)(4)(1)(10)#1875  = 0
 CustomerVisit(10,4,1,10):       w(10)(4)(1)(10)#2055  = 0
 CustomerVisit(11,4,1,10):       w(11)(4)(1)(10)#2235  = 0
 CustomerVisit(12,4,1,10):       w(12)(4)(1)(10)#2415  = 0
 CustomerVisit(3,5,1,10):        w(3)(5)(1)(10)#825  = 0
 CustomerVisit(4,5,1,10):        w(4)(5)(1)(10)#1005  = 0
 CustomerVisit(5,5,1,10):        w(5)(5)(1)(10)#1185  = 0
 CustomerVisit(6,5,1,10):        w(6)(5)(1)(10)#1365  = 0
 CustomerVisit(7,5,1,10):        w(7)(5)(1)(10)#1545  = 0
 CustomerVisit(8,5,1,10):        w(8)(5)(1)(10)#1725  = 0
 CustomerVisit(9,5,1,10):        w(9)(5)(1)(10)#1905  = 0
 CustomerVisit(10,5,1,10):       w(10)(5)(1)(10)#2085  = 0
 CustomerVisit(11,5,1,10):       w(11)(5)(1)(10)#2265  = 0
 CustomerVisit(12,5,1,10):       w(12)(5)(1)(10)#2445  = 0
 CustomerVisit(3,6,1,10):        w(3)(6)(1)(10)#855  = 0
 CustomerVisit(4,6,1,10):        w(4)(6)(1)(10)#1035  = 0
 CustomerVisit(5,6,1,10):        w(5)(6)(1)(10)#1215  = 0
 CustomerVisit(6,6,1,10):        w(6)(6)(1)(10)#1395  = 0
 CustomerVisit(7,6,1,10):        w(7)(6)(1)(10)#1575  = 0
 CustomerVisit(8,6,1,10):        w(8)(6)(1)(10)#1755  = 0
 CustomerVisit(9,6,1,10):        w(9)(6)(1)(10)#1935  = 0
 CustomerVisit(10,6,1,10):       w(10)(6)(1)(10)#2115  = 0
 CustomerVisit(11,6,1,10):       w(11)(6)(1)(10)#2295  = 0
 CustomerVisit(12,6,1,10):       w(12)(6)(1)(10)#2475  = 0
 CustomerVisit(3,4,2,10):        w(3)(4)(2)(10)#805  = 0
 CustomerVisit(4,4,2,10):        w(4)(4)(2)(10)#985  = 0
 CustomerVisit(5,4,2,10):        w(5)(4)(2)(10)#1165  = 0
 CustomerVisit(6,4,2,10):        w(6)(4)(2)(10)#1345  = 0
 CustomerVisit(7,4,2,10):        w(7)(4)(2)(10)#1525  = 0
 CustomerVisit(8,4,2,10):        w(8)(4)(2)(10)#1705  = 0
 CustomerVisit(9,4,2,10):        w(9)(4)(2)(10)#1885  = 0
 CustomerVisit(10,4,2,10):       w(10)(4)(2)(10)#2065  = 0
 CustomerVisit(11,4,2,10):       w(11)(4)(2)(10)#2245  = 0
 CustomerVisit(12,4,2,10):       w(12)(4)(2)(10)#2425  = 0
 CustomerVisit(3,5,2,10):        w(3)(5)(2)(10)#835  = 0
 CustomerVisit(4,5,2,10):        w(4)(5)(2)(10)#1015  = 0
 CustomerVisit(5,5,2,10):        w(5)(5)(2)(10)#1195  = 0
 CustomerVisit(6,5,2,10):        w(6)(5)(2)(10)#1375  = 0
 CustomerVisit(7,5,2,10):        w(7)(5)(2)(10)#1555  = 0
 CustomerVisit(8,5,2,10):        w(8)(5)(2)(10)#1735  = 0
 CustomerVisit(9,5,2,10):        w(9)(5)(2)(10)#1915  = 0
 CustomerVisit(10,5,2,10):       w(10)(5)(2)(10)#2095  = 0
 CustomerVisit(11,5,2,10):       w(11)(5)(2)(10)#2275  = 0
 CustomerVisit(12,5,2,10):       w(12)(5)(2)(10)#2455  = 0
 CustomerVisit(3,6,2,10):        w(3)(6)(2)(10)#865 <= 20
 CustomerVisit(4,6,2,10):        w(4)(6)(2)(10)#1045 <= 28
 CustomerVisit(5,6,2,10):        w(5)(6)(2)(10)#1225  = 0
 CustomerVisit(6,6,2,10):        w(6)(6)(2)(10)#1405  = 0
 CustomerVisit(7,6,2,10):        w(7)(6)(2)(10)#1585  = 0
 CustomerVisit(8,6,2,10):        w(8)(6)(2)(10)#1765  = 0
 CustomerVisit(9,6,2,10):        w(9)(6)(2)(10)#1945  = 0
 CustomerVisit(10,6,2,10):       w(10)(6)(2)(10)#2125  = 0
 CustomerVisit(11,6,2,10):       w(11)(6)(2)(10)#2305  = 0
 CustomerVisit(12,6,2,10):       w(12)(6)(2)(10)#2485  = 0
 CustomerVisit(3,4,3,10):        w(3)(4)(3)(10)#815 <= 11
 CustomerVisit(4,4,3,10):        w(4)(4)(3)(10)#995  = 0
 CustomerVisit(5,4,3,10):        w(5)(4)(3)(10)#1175  = 0
 CustomerVisit(6,4,3,10):        w(6)(4)(3)(10)#1355 <= 8
 CustomerVisit(7,4,3,10):        w(7)(4)(3)(10)#1535 <= 12
 CustomerVisit(8,4,3,10):        w(8)(4)(3)(10)#1715  = 0
 CustomerVisit(9,4,3,10):        w(9)(4)(3)(10)#1895  = 0
 CustomerVisit(10,4,3,10):       w(10)(4)(3)(10)#2075 <= 13
 CustomerVisit(11,4,3,10):       w(11)(4)(3)(10)#2255  = 0
 CustomerVisit(12,4,3,10):       w(12)(4)(3)(10)#2435  = 0
 CustomerVisit(3,5,3,10):        w(3)(5)(3)(10)#845  = 0
 CustomerVisit(4,5,3,10):        w(4)(5)(3)(10)#1025  = 0
 CustomerVisit(5,5,3,10):        w(5)(5)(3)(10)#1205  = 0
 CustomerVisit(6,5,3,10):        w(6)(5)(3)(10)#1385  = 0
 CustomerVisit(7,5,3,10):        w(7)(5)(3)(10)#1565  = 0
 CustomerVisit(8,5,3,10):        w(8)(5)(3)(10)#1745  = 0
 CustomerVisit(9,5,3,10):        w(9)(5)(3)(10)#1925  = 0
 CustomerVisit(10,5,3,10):       w(10)(5)(3)(10)#2105  = 0
 CustomerVisit(11,5,3,10):       w(11)(5)(3)(10)#2285  = 0
 CustomerVisit(12,5,3,10):       w(12)(5)(3)(10)#2465  = 0
 CustomerVisit(3,6,3,10):        w(3)(6)(3)(10)#875  = 0
 CustomerVisit(4,6,3,10):        w(4)(6)(3)(10)#1055  = 0
 CustomerVisit(5,6,3,10):        w(5)(6)(3)(10)#1235  = 0
 CustomerVisit(6,6,3,10):        w(6)(6)(3)(10)#1415  = 0
 CustomerVisit(7,6,3,10):        w(7)(6)(3)(10)#1595  = 0
 CustomerVisit(8,6,3,10):        w(8)(6)(3)(10)#1775  = 0
 CustomerVisit(9,6,3,10):        w(9)(6)(3)(10)#1955  = 0
 CustomerVisit(10,6,3,10):       w(10)(6)(3)(10)#2135  = 0
 CustomerVisit(11,6,3,10):       w(11)(6)(3)(10)#2315  = 0
 CustomerVisit(12,6,3,10):       w(12)(6)(3)(10)#2495  = 0
 VehicleCapacityWarehouse(1,1,1,1): w(3)(1)(1)(1)#696 + w(4)(1)(1)(1)#876
                                 + w(5)(1)(1)(1)#1056 + w(6)(1)(1)(1)#1236
                                 + w(7)(1)(1)(1)#1416 + w(8)(1)(1)(1)#1596
                                 + w(9)(1)(1)(1)#1776 + w(10)(1)(1)(1)#1956
                                 + w(11)(1)(1)(1)#2136 + w(12)(1)(1)(1)#2316
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,1): w(3)(2)(1)(1)#726 + w(4)(2)(1)(1)#906
                                 + w(5)(2)(1)(1)#1086 + w(6)(2)(1)(1)#1266
                                 + w(7)(2)(1)(1)#1446 + w(8)(2)(1)(1)#1626
                                 + w(9)(2)(1)(1)#1806 + w(10)(2)(1)(1)#1986
                                 + w(11)(2)(1)(1)#2166 + w(12)(2)(1)(1)#2346
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,1): w(3)(3)(1)(1)#756 + w(4)(3)(1)(1)#936
                                 + w(5)(3)(1)(1)#1116 + w(6)(3)(1)(1)#1296
                                 + w(7)(3)(1)(1)#1476 + w(8)(3)(1)(1)#1656
                                 + w(9)(3)(1)(1)#1836 + w(10)(3)(1)(1)#2016
                                 + w(11)(3)(1)(1)#2196 + w(12)(3)(1)(1)#2376
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,1): w(3)(4)(1)(1)#786 + w(4)(4)(1)(1)#966
                                 + w(5)(4)(1)(1)#1146 + w(6)(4)(1)(1)#1326
                                 + w(7)(4)(1)(1)#1506 + w(8)(4)(1)(1)#1686
                                 + w(9)(4)(1)(1)#1866 + w(10)(4)(1)(1)#2046
                                 + w(11)(4)(1)(1)#2226 + w(12)(4)(1)(1)#2406
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,1): w(3)(5)(1)(1)#816 + w(4)(5)(1)(1)#996
                                 + w(5)(5)(1)(1)#1176 + w(6)(5)(1)(1)#1356
                                 + w(7)(5)(1)(1)#1536 + w(8)(5)(1)(1)#1716
                                 + w(9)(5)(1)(1)#1896 + w(10)(5)(1)(1)#2076
                                 + w(11)(5)(1)(1)#2256 + w(12)(5)(1)(1)#2436
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,1): w(3)(6)(1)(1)#846 + w(4)(6)(1)(1)#1026
                                 + w(5)(6)(1)(1)#1206 + w(6)(6)(1)(1)#1386
                                 + w(7)(6)(1)(1)#1566 + w(8)(6)(1)(1)#1746
                                 + w(9)(6)(1)(1)#1926 + w(10)(6)(1)(1)#2106
                                 + w(11)(6)(1)(1)#2286 + w(12)(6)(1)(1)#2466
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,1): w(3)(1)(2)(1)#706 + w(4)(1)(2)(1)#886
                                 + w(5)(1)(2)(1)#1066 + w(6)(1)(2)(1)#1246
                                 + w(7)(1)(2)(1)#1426 + w(8)(1)(2)(1)#1606
                                 + w(9)(1)(2)(1)#1786 + w(10)(1)(2)(1)#1966
                                 + w(11)(1)(2)(1)#2146 + w(12)(1)(2)(1)#2326
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,1): w(3)(2)(2)(1)#736 + w(4)(2)(2)(1)#916
                                 + w(5)(2)(2)(1)#1096 + w(6)(2)(2)(1)#1276
                                 + w(7)(2)(2)(1)#1456 + w(8)(2)(2)(1)#1636
                                 + w(9)(2)(2)(1)#1816 + w(10)(2)(2)(1)#1996
                                 + w(11)(2)(2)(1)#2176 + w(12)(2)(2)(1)#2356
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,1): w(3)(3)(2)(1)#766 + w(4)(3)(2)(1)#946
                                 + w(5)(3)(2)(1)#1126 + w(6)(3)(2)(1)#1306
                                 + w(7)(3)(2)(1)#1486 + w(8)(3)(2)(1)#1666
                                 + w(9)(3)(2)(1)#1846 + w(10)(3)(2)(1)#2026
                                 + w(11)(3)(2)(1)#2206 + w(12)(3)(2)(1)#2386
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,1): w(3)(4)(2)(1)#796 + w(4)(4)(2)(1)#976
                                 + w(5)(4)(2)(1)#1156 + w(6)(4)(2)(1)#1336
                                 + w(7)(4)(2)(1)#1516 + w(8)(4)(2)(1)#1696
                                 + w(9)(4)(2)(1)#1876 + w(10)(4)(2)(1)#2056
                                 + w(11)(4)(2)(1)#2236 + w(12)(4)(2)(1)#2416
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,1): w(3)(5)(2)(1)#826 + w(4)(5)(2)(1)#1006
                                 + w(5)(5)(2)(1)#1186 + w(6)(5)(2)(1)#1366
                                 + w(7)(5)(2)(1)#1546 + w(8)(5)(2)(1)#1726
                                 + w(9)(5)(2)(1)#1906 + w(10)(5)(2)(1)#2086
                                 + w(11)(5)(2)(1)#2266 + w(12)(5)(2)(1)#2446
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,1): w(3)(6)(2)(1)#856 + w(4)(6)(2)(1)#1036
                                 + w(5)(6)(2)(1)#1216 + w(6)(6)(2)(1)#1396
                                 + w(7)(6)(2)(1)#1576 + w(8)(6)(2)(1)#1756
                                 + w(9)(6)(2)(1)#1936 + w(10)(6)(2)(1)#2116
                                 + w(11)(6)(2)(1)#2296 + w(12)(6)(2)(1)#2476
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,1): w(3)(1)(3)(1)#716 + w(4)(1)(3)(1)#896
                                 + w(5)(1)(3)(1)#1076 + w(6)(1)(3)(1)#1256
                                 + w(7)(1)(3)(1)#1436 + w(8)(1)(3)(1)#1616
                                 + w(9)(1)(3)(1)#1796 + w(10)(1)(3)(1)#1976
                                 + w(11)(1)(3)(1)#2156 + w(12)(1)(3)(1)#2336
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,1): w(3)(2)(3)(1)#746 + w(4)(2)(3)(1)#926
                                 + w(5)(2)(3)(1)#1106 + w(6)(2)(3)(1)#1286
                                 + w(7)(2)(3)(1)#1466 + w(8)(2)(3)(1)#1646
                                 + w(9)(2)(3)(1)#1826 + w(10)(2)(3)(1)#2006
                                 + w(11)(2)(3)(1)#2186 + w(12)(2)(3)(1)#2366
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,1): w(3)(3)(3)(1)#776 + w(4)(3)(3)(1)#956
                                 + w(5)(3)(3)(1)#1136 + w(6)(3)(3)(1)#1316
                                 + w(7)(3)(3)(1)#1496 + w(8)(3)(3)(1)#1676
                                 + w(9)(3)(3)(1)#1856 + w(10)(3)(3)(1)#2036
                                 + w(11)(3)(3)(1)#2216 + w(12)(3)(3)(1)#2396
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,1): w(3)(4)(3)(1)#806 + w(4)(4)(3)(1)#986
                                 + w(5)(4)(3)(1)#1166 + w(6)(4)(3)(1)#1346
                                 + w(7)(4)(3)(1)#1526 + w(8)(4)(3)(1)#1706
                                 + w(9)(4)(3)(1)#1886 + w(10)(4)(3)(1)#2066
                                 + w(11)(4)(3)(1)#2246 + w(12)(4)(3)(1)#2426
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,1): w(3)(5)(3)(1)#836 + w(4)(5)(3)(1)#1016
                                 + w(5)(5)(3)(1)#1196 + w(6)(5)(3)(1)#1376
                                 + w(7)(5)(3)(1)#1556 + w(8)(5)(3)(1)#1736
                                 + w(9)(5)(3)(1)#1916 + w(10)(5)(3)(1)#2096
                                 + w(11)(5)(3)(1)#2276 + w(12)(5)(3)(1)#2456
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,1): w(3)(6)(3)(1)#866 + w(4)(6)(3)(1)#1046
                                 + w(5)(6)(3)(1)#1226 + w(6)(6)(3)(1)#1406
                                 + w(7)(6)(3)(1)#1586 + w(8)(6)(3)(1)#1766
                                 + w(9)(6)(3)(1)#1946 + w(10)(6)(3)(1)#2126
                                 + w(11)(6)(3)(1)#2306 + w(12)(6)(3)(1)#2486
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,2): w(3)(1)(1)(2)#697 + w(4)(1)(1)(2)#877
                                 + w(5)(1)(1)(2)#1057 + w(6)(1)(1)(2)#1237
                                 + w(7)(1)(1)(2)#1417 + w(8)(1)(1)(2)#1597
                                 + w(9)(1)(1)(2)#1777 + w(10)(1)(1)(2)#1957
                                 + w(11)(1)(1)(2)#2137 + w(12)(1)(1)(2)#2317
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,2): w(3)(2)(1)(2)#727 + w(4)(2)(1)(2)#907
                                 + w(5)(2)(1)(2)#1087 + w(6)(2)(1)(2)#1267
                                 + w(7)(2)(1)(2)#1447 + w(8)(2)(1)(2)#1627
                                 + w(9)(2)(1)(2)#1807 + w(10)(2)(1)(2)#1987
                                 + w(11)(2)(1)(2)#2167 + w(12)(2)(1)(2)#2347
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,2): w(3)(3)(1)(2)#757 + w(4)(3)(1)(2)#937
                                 + w(5)(3)(1)(2)#1117 + w(6)(3)(1)(2)#1297
                                 + w(7)(3)(1)(2)#1477 + w(8)(3)(1)(2)#1657
                                 + w(9)(3)(1)(2)#1837 + w(10)(3)(1)(2)#2017
                                 + w(11)(3)(1)(2)#2197 + w(12)(3)(1)(2)#2377
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,2): w(3)(4)(1)(2)#787 + w(4)(4)(1)(2)#967
                                 + w(5)(4)(1)(2)#1147 + w(6)(4)(1)(2)#1327
                                 + w(7)(4)(1)(2)#1507 + w(8)(4)(1)(2)#1687
                                 + w(9)(4)(1)(2)#1867 + w(10)(4)(1)(2)#2047
                                 + w(11)(4)(1)(2)#2227 + w(12)(4)(1)(2)#2407
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,2): w(3)(5)(1)(2)#817 + w(4)(5)(1)(2)#997
                                 + w(5)(5)(1)(2)#1177 + w(6)(5)(1)(2)#1357
                                 + w(7)(5)(1)(2)#1537 + w(8)(5)(1)(2)#1717
                                 + w(9)(5)(1)(2)#1897 + w(10)(5)(1)(2)#2077
                                 + w(11)(5)(1)(2)#2257 + w(12)(5)(1)(2)#2437
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,2): w(3)(6)(1)(2)#847 + w(4)(6)(1)(2)#1027
                                 + w(5)(6)(1)(2)#1207 + w(6)(6)(1)(2)#1387
                                 + w(7)(6)(1)(2)#1567 + w(8)(6)(1)(2)#1747
                                 + w(9)(6)(1)(2)#1927 + w(10)(6)(1)(2)#2107
                                 + w(11)(6)(1)(2)#2287 + w(12)(6)(1)(2)#2467
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,2): w(3)(1)(2)(2)#707 + w(4)(1)(2)(2)#887
                                 + w(5)(1)(2)(2)#1067 + w(6)(1)(2)(2)#1247
                                 + w(7)(1)(2)(2)#1427 + w(8)(1)(2)(2)#1607
                                 + w(9)(1)(2)(2)#1787 + w(10)(1)(2)(2)#1967
                                 + w(11)(1)(2)(2)#2147 + w(12)(1)(2)(2)#2327
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,2): w(3)(2)(2)(2)#737 + w(4)(2)(2)(2)#917
                                 + w(5)(2)(2)(2)#1097 + w(6)(2)(2)(2)#1277
                                 + w(7)(2)(2)(2)#1457 + w(8)(2)(2)(2)#1637
                                 + w(9)(2)(2)(2)#1817 + w(10)(2)(2)(2)#1997
                                 + w(11)(2)(2)(2)#2177 + w(12)(2)(2)(2)#2357
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,2): w(3)(3)(2)(2)#767 + w(4)(3)(2)(2)#947
                                 + w(5)(3)(2)(2)#1127 + w(6)(3)(2)(2)#1307
                                 + w(7)(3)(2)(2)#1487 + w(8)(3)(2)(2)#1667
                                 + w(9)(3)(2)(2)#1847 + w(10)(3)(2)(2)#2027
                                 + w(11)(3)(2)(2)#2207 + w(12)(3)(2)(2)#2387
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,2): w(3)(4)(2)(2)#797 + w(4)(4)(2)(2)#977
                                 + w(5)(4)(2)(2)#1157 + w(6)(4)(2)(2)#1337
                                 + w(7)(4)(2)(2)#1517 + w(8)(4)(2)(2)#1697
                                 + w(9)(4)(2)(2)#1877 + w(10)(4)(2)(2)#2057
                                 + w(11)(4)(2)(2)#2237 + w(12)(4)(2)(2)#2417
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,2): w(3)(5)(2)(2)#827 + w(4)(5)(2)(2)#1007
                                 + w(5)(5)(2)(2)#1187 + w(6)(5)(2)(2)#1367
                                 + w(7)(5)(2)(2)#1547 + w(8)(5)(2)(2)#1727
                                 + w(9)(5)(2)(2)#1907 + w(10)(5)(2)(2)#2087
                                 + w(11)(5)(2)(2)#2267 + w(12)(5)(2)(2)#2447
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,2): w(3)(6)(2)(2)#857 + w(4)(6)(2)(2)#1037
                                 + w(5)(6)(2)(2)#1217 + w(6)(6)(2)(2)#1397
                                 + w(7)(6)(2)(2)#1577 + w(8)(6)(2)(2)#1757
                                 + w(9)(6)(2)(2)#1937 + w(10)(6)(2)(2)#2117
                                 + w(11)(6)(2)(2)#2297 + w(12)(6)(2)(2)#2477
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,2): w(3)(1)(3)(2)#717 + w(4)(1)(3)(2)#897
                                 + w(5)(1)(3)(2)#1077 + w(6)(1)(3)(2)#1257
                                 + w(7)(1)(3)(2)#1437 + w(8)(1)(3)(2)#1617
                                 + w(9)(1)(3)(2)#1797 + w(10)(1)(3)(2)#1977
                                 + w(11)(1)(3)(2)#2157 + w(12)(1)(3)(2)#2337
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,2): w(3)(2)(3)(2)#747 + w(4)(2)(3)(2)#927
                                 + w(5)(2)(3)(2)#1107 + w(6)(2)(3)(2)#1287
                                 + w(7)(2)(3)(2)#1467 + w(8)(2)(3)(2)#1647
                                 + w(9)(2)(3)(2)#1827 + w(10)(2)(3)(2)#2007
                                 + w(11)(2)(3)(2)#2187 + w(12)(2)(3)(2)#2367
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,2): w(3)(3)(3)(2)#777 + w(4)(3)(3)(2)#957
                                 + w(5)(3)(3)(2)#1137 + w(6)(3)(3)(2)#1317
                                 + w(7)(3)(3)(2)#1497 + w(8)(3)(3)(2)#1677
                                 + w(9)(3)(3)(2)#1857 + w(10)(3)(3)(2)#2037
                                 + w(11)(3)(3)(2)#2217 + w(12)(3)(3)(2)#2397
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,2): w(3)(4)(3)(2)#807 + w(4)(4)(3)(2)#987
                                 + w(5)(4)(3)(2)#1167 + w(6)(4)(3)(2)#1347
                                 + w(7)(4)(3)(2)#1527 + w(8)(4)(3)(2)#1707
                                 + w(9)(4)(3)(2)#1887 + w(10)(4)(3)(2)#2067
                                 + w(11)(4)(3)(2)#2247 + w(12)(4)(3)(2)#2427
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,2): w(3)(5)(3)(2)#837 + w(4)(5)(3)(2)#1017
                                 + w(5)(5)(3)(2)#1197 + w(6)(5)(3)(2)#1377
                                 + w(7)(5)(3)(2)#1557 + w(8)(5)(3)(2)#1737
                                 + w(9)(5)(3)(2)#1917 + w(10)(5)(3)(2)#2097
                                 + w(11)(5)(3)(2)#2277 + w(12)(5)(3)(2)#2457
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,2): w(3)(6)(3)(2)#867 + w(4)(6)(3)(2)#1047
                                 + w(5)(6)(3)(2)#1227 + w(6)(6)(3)(2)#1407
                                 + w(7)(6)(3)(2)#1587 + w(8)(6)(3)(2)#1767
                                 + w(9)(6)(3)(2)#1947 + w(10)(6)(3)(2)#2127
                                 + w(11)(6)(3)(2)#2307 + w(12)(6)(3)(2)#2487
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,3): w(3)(1)(1)(3)#698 + w(4)(1)(1)(3)#878
                                 + w(5)(1)(1)(3)#1058 + w(6)(1)(1)(3)#1238
                                 + w(7)(1)(1)(3)#1418 + w(8)(1)(1)(3)#1598
                                 + w(9)(1)(1)(3)#1778 + w(10)(1)(1)(3)#1958
                                 + w(11)(1)(1)(3)#2138 + w(12)(1)(1)(3)#2318
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,3): w(3)(2)(1)(3)#728 + w(4)(2)(1)(3)#908
                                 + w(5)(2)(1)(3)#1088 + w(6)(2)(1)(3)#1268
                                 + w(7)(2)(1)(3)#1448 + w(8)(2)(1)(3)#1628
                                 + w(9)(2)(1)(3)#1808 + w(10)(2)(1)(3)#1988
                                 + w(11)(2)(1)(3)#2168 + w(12)(2)(1)(3)#2348
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,3): w(3)(3)(1)(3)#758 + w(4)(3)(1)(3)#938
                                 + w(5)(3)(1)(3)#1118 + w(6)(3)(1)(3)#1298
                                 + w(7)(3)(1)(3)#1478 + w(8)(3)(1)(3)#1658
                                 + w(9)(3)(1)(3)#1838 + w(10)(3)(1)(3)#2018
                                 + w(11)(3)(1)(3)#2198 + w(12)(3)(1)(3)#2378
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,3): w(3)(4)(1)(3)#788 + w(4)(4)(1)(3)#968
                                 + w(5)(4)(1)(3)#1148 + w(6)(4)(1)(3)#1328
                                 + w(7)(4)(1)(3)#1508 + w(8)(4)(1)(3)#1688
                                 + w(9)(4)(1)(3)#1868 + w(10)(4)(1)(3)#2048
                                 + w(11)(4)(1)(3)#2228 + w(12)(4)(1)(3)#2408
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,3): w(3)(5)(1)(3)#818 + w(4)(5)(1)(3)#998
                                 + w(5)(5)(1)(3)#1178 + w(6)(5)(1)(3)#1358
                                 + w(7)(5)(1)(3)#1538 + w(8)(5)(1)(3)#1718
                                 + w(9)(5)(1)(3)#1898 + w(10)(5)(1)(3)#2078
                                 + w(11)(5)(1)(3)#2258 + w(12)(5)(1)(3)#2438
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,3): w(3)(6)(1)(3)#848 + w(4)(6)(1)(3)#1028
                                 + w(5)(6)(1)(3)#1208 + w(6)(6)(1)(3)#1388
                                 + w(7)(6)(1)(3)#1568 + w(8)(6)(1)(3)#1748
                                 + w(9)(6)(1)(3)#1928 + w(10)(6)(1)(3)#2108
                                 + w(11)(6)(1)(3)#2288 + w(12)(6)(1)(3)#2468
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,3): w(3)(1)(2)(3)#708 + w(4)(1)(2)(3)#888
                                 + w(5)(1)(2)(3)#1068 + w(6)(1)(2)(3)#1248
                                 + w(7)(1)(2)(3)#1428 + w(8)(1)(2)(3)#1608
                                 + w(9)(1)(2)(3)#1788 + w(10)(1)(2)(3)#1968
                                 + w(11)(1)(2)(3)#2148 + w(12)(1)(2)(3)#2328
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,3): w(3)(2)(2)(3)#738 + w(4)(2)(2)(3)#918
                                 + w(5)(2)(2)(3)#1098 + w(6)(2)(2)(3)#1278
                                 + w(7)(2)(2)(3)#1458 + w(8)(2)(2)(3)#1638
                                 + w(9)(2)(2)(3)#1818 + w(10)(2)(2)(3)#1998
                                 + w(11)(2)(2)(3)#2178 + w(12)(2)(2)(3)#2358
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,3): w(3)(3)(2)(3)#768 + w(4)(3)(2)(3)#948
                                 + w(5)(3)(2)(3)#1128 + w(6)(3)(2)(3)#1308
                                 + w(7)(3)(2)(3)#1488 + w(8)(3)(2)(3)#1668
                                 + w(9)(3)(2)(3)#1848 + w(10)(3)(2)(3)#2028
                                 + w(11)(3)(2)(3)#2208 + w(12)(3)(2)(3)#2388
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,3): w(3)(4)(2)(3)#798 + w(4)(4)(2)(3)#978
                                 + w(5)(4)(2)(3)#1158 + w(6)(4)(2)(3)#1338
                                 + w(7)(4)(2)(3)#1518 + w(8)(4)(2)(3)#1698
                                 + w(9)(4)(2)(3)#1878 + w(10)(4)(2)(3)#2058
                                 + w(11)(4)(2)(3)#2238 + w(12)(4)(2)(3)#2418
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,3): w(3)(5)(2)(3)#828 + w(4)(5)(2)(3)#1008
                                 + w(5)(5)(2)(3)#1188 + w(6)(5)(2)(3)#1368
                                 + w(7)(5)(2)(3)#1548 + w(8)(5)(2)(3)#1728
                                 + w(9)(5)(2)(3)#1908 + w(10)(5)(2)(3)#2088
                                 + w(11)(5)(2)(3)#2268 + w(12)(5)(2)(3)#2448
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,3): w(3)(6)(2)(3)#858 + w(4)(6)(2)(3)#1038
                                 + w(5)(6)(2)(3)#1218 + w(6)(6)(2)(3)#1398
                                 + w(7)(6)(2)(3)#1578 + w(8)(6)(2)(3)#1758
                                 + w(9)(6)(2)(3)#1938 + w(10)(6)(2)(3)#2118
                                 + w(11)(6)(2)(3)#2298 + w(12)(6)(2)(3)#2478
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,3): w(3)(1)(3)(3)#718 + w(4)(1)(3)(3)#898
                                 + w(5)(1)(3)(3)#1078 + w(6)(1)(3)(3)#1258
                                 + w(7)(1)(3)(3)#1438 + w(8)(1)(3)(3)#1618
                                 + w(9)(1)(3)(3)#1798 + w(10)(1)(3)(3)#1978
                                 + w(11)(1)(3)(3)#2158 + w(12)(1)(3)(3)#2338
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,3): w(3)(2)(3)(3)#748 + w(4)(2)(3)(3)#928
                                 + w(5)(2)(3)(3)#1108 + w(6)(2)(3)(3)#1288
                                 + w(7)(2)(3)(3)#1468 + w(8)(2)(3)(3)#1648
                                 + w(9)(2)(3)(3)#1828 + w(10)(2)(3)(3)#2008
                                 + w(11)(2)(3)(3)#2188 + w(12)(2)(3)(3)#2368
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,3): w(3)(3)(3)(3)#778 + w(4)(3)(3)(3)#958
                                 + w(5)(3)(3)(3)#1138 + w(6)(3)(3)(3)#1318
                                 + w(7)(3)(3)(3)#1498 + w(8)(3)(3)(3)#1678
                                 + w(9)(3)(3)(3)#1858 + w(10)(3)(3)(3)#2038
                                 + w(11)(3)(3)(3)#2218 + w(12)(3)(3)(3)#2398
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,3): w(3)(4)(3)(3)#808 + w(4)(4)(3)(3)#988
                                 + w(5)(4)(3)(3)#1168 + w(6)(4)(3)(3)#1348
                                 + w(7)(4)(3)(3)#1528 + w(8)(4)(3)(3)#1708
                                 + w(9)(4)(3)(3)#1888 + w(10)(4)(3)(3)#2068
                                 + w(11)(4)(3)(3)#2248 + w(12)(4)(3)(3)#2428
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,3): w(3)(5)(3)(3)#838 + w(4)(5)(3)(3)#1018
                                 + w(5)(5)(3)(3)#1198 + w(6)(5)(3)(3)#1378
                                 + w(7)(5)(3)(3)#1558 + w(8)(5)(3)(3)#1738
                                 + w(9)(5)(3)(3)#1918 + w(10)(5)(3)(3)#2098
                                 + w(11)(5)(3)(3)#2278 + w(12)(5)(3)(3)#2458
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,3): w(3)(6)(3)(3)#868 + w(4)(6)(3)(3)#1048
                                 + w(5)(6)(3)(3)#1228 + w(6)(6)(3)(3)#1408
                                 + w(7)(6)(3)(3)#1588 + w(8)(6)(3)(3)#1768
                                 + w(9)(6)(3)(3)#1948 + w(10)(6)(3)(3)#2128
                                 + w(11)(6)(3)(3)#2308 + w(12)(6)(3)(3)#2488
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,4): w(3)(1)(1)(4)#699 + w(4)(1)(1)(4)#879
                                 + w(5)(1)(1)(4)#1059 + w(6)(1)(1)(4)#1239
                                 + w(7)(1)(1)(4)#1419 + w(8)(1)(1)(4)#1599
                                 + w(9)(1)(1)(4)#1779 + w(10)(1)(1)(4)#1959
                                 + w(11)(1)(1)(4)#2139 + w(12)(1)(1)(4)#2319
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,4): w(3)(2)(1)(4)#729 + w(4)(2)(1)(4)#909
                                 + w(5)(2)(1)(4)#1089 + w(6)(2)(1)(4)#1269
                                 + w(7)(2)(1)(4)#1449 + w(8)(2)(1)(4)#1629
                                 + w(9)(2)(1)(4)#1809 + w(10)(2)(1)(4)#1989
                                 + w(11)(2)(1)(4)#2169 + w(12)(2)(1)(4)#2349
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,4): w(3)(3)(1)(4)#759 + w(4)(3)(1)(4)#939
                                 + w(5)(3)(1)(4)#1119 + w(6)(3)(1)(4)#1299
                                 + w(7)(3)(1)(4)#1479 + w(8)(3)(1)(4)#1659
                                 + w(9)(3)(1)(4)#1839 + w(10)(3)(1)(4)#2019
                                 + w(11)(3)(1)(4)#2199 + w(12)(3)(1)(4)#2379
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,4): w(3)(4)(1)(4)#789 + w(4)(4)(1)(4)#969
                                 + w(5)(4)(1)(4)#1149 + w(6)(4)(1)(4)#1329
                                 + w(7)(4)(1)(4)#1509 + w(8)(4)(1)(4)#1689
                                 + w(9)(4)(1)(4)#1869 + w(10)(4)(1)(4)#2049
                                 + w(11)(4)(1)(4)#2229 + w(12)(4)(1)(4)#2409
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,4): w(3)(5)(1)(4)#819 + w(4)(5)(1)(4)#999
                                 + w(5)(5)(1)(4)#1179 + w(6)(5)(1)(4)#1359
                                 + w(7)(5)(1)(4)#1539 + w(8)(5)(1)(4)#1719
                                 + w(9)(5)(1)(4)#1899 + w(10)(5)(1)(4)#2079
                                 + w(11)(5)(1)(4)#2259 + w(12)(5)(1)(4)#2439
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,4): w(3)(6)(1)(4)#849 + w(4)(6)(1)(4)#1029
                                 + w(5)(6)(1)(4)#1209 + w(6)(6)(1)(4)#1389
                                 + w(7)(6)(1)(4)#1569 + w(8)(6)(1)(4)#1749
                                 + w(9)(6)(1)(4)#1929 + w(10)(6)(1)(4)#2109
                                 + w(11)(6)(1)(4)#2289 + w(12)(6)(1)(4)#2469
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,4): w(3)(1)(2)(4)#709 + w(4)(1)(2)(4)#889
                                 + w(5)(1)(2)(4)#1069 + w(6)(1)(2)(4)#1249
                                 + w(7)(1)(2)(4)#1429 + w(8)(1)(2)(4)#1609
                                 + w(9)(1)(2)(4)#1789 + w(10)(1)(2)(4)#1969
                                 + w(11)(1)(2)(4)#2149 + w(12)(1)(2)(4)#2329
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,4): w(3)(2)(2)(4)#739 + w(4)(2)(2)(4)#919
                                 + w(5)(2)(2)(4)#1099 + w(6)(2)(2)(4)#1279
                                 + w(7)(2)(2)(4)#1459 + w(8)(2)(2)(4)#1639
                                 + w(9)(2)(2)(4)#1819 + w(10)(2)(2)(4)#1999
                                 + w(11)(2)(2)(4)#2179 + w(12)(2)(2)(4)#2359
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,4): w(3)(3)(2)(4)#769 + w(4)(3)(2)(4)#949
                                 + w(5)(3)(2)(4)#1129 + w(6)(3)(2)(4)#1309
                                 + w(7)(3)(2)(4)#1489 + w(8)(3)(2)(4)#1669
                                 + w(9)(3)(2)(4)#1849 + w(10)(3)(2)(4)#2029
                                 + w(11)(3)(2)(4)#2209 + w(12)(3)(2)(4)#2389
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,4): w(3)(4)(2)(4)#799 + w(4)(4)(2)(4)#979
                                 + w(5)(4)(2)(4)#1159 + w(6)(4)(2)(4)#1339
                                 + w(7)(4)(2)(4)#1519 + w(8)(4)(2)(4)#1699
                                 + w(9)(4)(2)(4)#1879 + w(10)(4)(2)(4)#2059
                                 + w(11)(4)(2)(4)#2239 + w(12)(4)(2)(4)#2419
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,4): w(3)(5)(2)(4)#829 + w(4)(5)(2)(4)#1009
                                 + w(5)(5)(2)(4)#1189 + w(6)(5)(2)(4)#1369
                                 + w(7)(5)(2)(4)#1549 + w(8)(5)(2)(4)#1729
                                 + w(9)(5)(2)(4)#1909 + w(10)(5)(2)(4)#2089
                                 + w(11)(5)(2)(4)#2269 + w(12)(5)(2)(4)#2449
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,4): w(3)(6)(2)(4)#859 + w(4)(6)(2)(4)#1039
                                 + w(5)(6)(2)(4)#1219 + w(6)(6)(2)(4)#1399
                                 + w(7)(6)(2)(4)#1579 + w(8)(6)(2)(4)#1759
                                 + w(9)(6)(2)(4)#1939 + w(10)(6)(2)(4)#2119
                                 + w(11)(6)(2)(4)#2299 + w(12)(6)(2)(4)#2479
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,4): w(3)(1)(3)(4)#719 + w(4)(1)(3)(4)#899
                                 + w(5)(1)(3)(4)#1079 + w(6)(1)(3)(4)#1259
                                 + w(7)(1)(3)(4)#1439 + w(8)(1)(3)(4)#1619
                                 + w(9)(1)(3)(4)#1799 + w(10)(1)(3)(4)#1979
                                 + w(11)(1)(3)(4)#2159 + w(12)(1)(3)(4)#2339
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,4): w(3)(2)(3)(4)#749 + w(4)(2)(3)(4)#929
                                 + w(5)(2)(3)(4)#1109 + w(6)(2)(3)(4)#1289
                                 + w(7)(2)(3)(4)#1469 + w(8)(2)(3)(4)#1649
                                 + w(9)(2)(3)(4)#1829 + w(10)(2)(3)(4)#2009
                                 + w(11)(2)(3)(4)#2189 + w(12)(2)(3)(4)#2369
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,4): w(3)(3)(3)(4)#779 + w(4)(3)(3)(4)#959
                                 + w(5)(3)(3)(4)#1139 + w(6)(3)(3)(4)#1319
                                 + w(7)(3)(3)(4)#1499 + w(8)(3)(3)(4)#1679
                                 + w(9)(3)(3)(4)#1859 + w(10)(3)(3)(4)#2039
                                 + w(11)(3)(3)(4)#2219 + w(12)(3)(3)(4)#2399
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,4): w(3)(4)(3)(4)#809 + w(4)(4)(3)(4)#989
                                 + w(5)(4)(3)(4)#1169 + w(6)(4)(3)(4)#1349
                                 + w(7)(4)(3)(4)#1529 + w(8)(4)(3)(4)#1709
                                 + w(9)(4)(3)(4)#1889 + w(10)(4)(3)(4)#2069
                                 + w(11)(4)(3)(4)#2249 + w(12)(4)(3)(4)#2429
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,4): w(3)(5)(3)(4)#839 + w(4)(5)(3)(4)#1019
                                 + w(5)(5)(3)(4)#1199 + w(6)(5)(3)(4)#1379
                                 + w(7)(5)(3)(4)#1559 + w(8)(5)(3)(4)#1739
                                 + w(9)(5)(3)(4)#1919 + w(10)(5)(3)(4)#2099
                                 + w(11)(5)(3)(4)#2279 + w(12)(5)(3)(4)#2459
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,4): w(3)(6)(3)(4)#869 + w(4)(6)(3)(4)#1049
                                 + w(5)(6)(3)(4)#1229 + w(6)(6)(3)(4)#1409
                                 + w(7)(6)(3)(4)#1589 + w(8)(6)(3)(4)#1769
                                 + w(9)(6)(3)(4)#1949 + w(10)(6)(3)(4)#2129
                                 + w(11)(6)(3)(4)#2309 + w(12)(6)(3)(4)#2489
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,5): w(3)(1)(1)(5)#700 + w(4)(1)(1)(5)#880
                                 + w(5)(1)(1)(5)#1060 + w(6)(1)(1)(5)#1240
                                 + w(7)(1)(1)(5)#1420 + w(8)(1)(1)(5)#1600
                                 + w(9)(1)(1)(5)#1780 + w(10)(1)(1)(5)#1960
                                 + w(11)(1)(1)(5)#2140 + w(12)(1)(1)(5)#2320
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,5): w(3)(2)(1)(5)#730 + w(4)(2)(1)(5)#910
                                 + w(5)(2)(1)(5)#1090 + w(6)(2)(1)(5)#1270
                                 + w(7)(2)(1)(5)#1450 + w(8)(2)(1)(5)#1630
                                 + w(9)(2)(1)(5)#1810 + w(10)(2)(1)(5)#1990
                                 + w(11)(2)(1)(5)#2170 + w(12)(2)(1)(5)#2350
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,5): w(3)(3)(1)(5)#760 + w(4)(3)(1)(5)#940
                                 + w(5)(3)(1)(5)#1120 + w(6)(3)(1)(5)#1300
                                 + w(7)(3)(1)(5)#1480 + w(8)(3)(1)(5)#1660
                                 + w(9)(3)(1)(5)#1840 + w(10)(3)(1)(5)#2020
                                 + w(11)(3)(1)(5)#2200 + w(12)(3)(1)(5)#2380
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,5): w(3)(4)(1)(5)#790 + w(4)(4)(1)(5)#970
                                 + w(5)(4)(1)(5)#1150 + w(6)(4)(1)(5)#1330
                                 + w(7)(4)(1)(5)#1510 + w(8)(4)(1)(5)#1690
                                 + w(9)(4)(1)(5)#1870 + w(10)(4)(1)(5)#2050
                                 + w(11)(4)(1)(5)#2230 + w(12)(4)(1)(5)#2410
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,5): w(3)(5)(1)(5)#820 + w(4)(5)(1)(5)#1000
                                 + w(5)(5)(1)(5)#1180 + w(6)(5)(1)(5)#1360
                                 + w(7)(5)(1)(5)#1540 + w(8)(5)(1)(5)#1720
                                 + w(9)(5)(1)(5)#1900 + w(10)(5)(1)(5)#2080
                                 + w(11)(5)(1)(5)#2260 + w(12)(5)(1)(5)#2440
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,5): w(3)(6)(1)(5)#850 + w(4)(6)(1)(5)#1030
                                 + w(5)(6)(1)(5)#1210 + w(6)(6)(1)(5)#1390
                                 + w(7)(6)(1)(5)#1570 + w(8)(6)(1)(5)#1750
                                 + w(9)(6)(1)(5)#1930 + w(10)(6)(1)(5)#2110
                                 + w(11)(6)(1)(5)#2290 + w(12)(6)(1)(5)#2470
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,5): w(3)(1)(2)(5)#710 + w(4)(1)(2)(5)#890
                                 + w(5)(1)(2)(5)#1070 + w(6)(1)(2)(5)#1250
                                 + w(7)(1)(2)(5)#1430 + w(8)(1)(2)(5)#1610
                                 + w(9)(1)(2)(5)#1790 + w(10)(1)(2)(5)#1970
                                 + w(11)(1)(2)(5)#2150 + w(12)(1)(2)(5)#2330
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,5): w(3)(2)(2)(5)#740 + w(4)(2)(2)(5)#920
                                 + w(5)(2)(2)(5)#1100 + w(6)(2)(2)(5)#1280
                                 + w(7)(2)(2)(5)#1460 + w(8)(2)(2)(5)#1640
                                 + w(9)(2)(2)(5)#1820 + w(10)(2)(2)(5)#2000
                                 + w(11)(2)(2)(5)#2180 + w(12)(2)(2)(5)#2360
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,5): w(3)(3)(2)(5)#770 + w(4)(3)(2)(5)#950
                                 + w(5)(3)(2)(5)#1130 + w(6)(3)(2)(5)#1310
                                 + w(7)(3)(2)(5)#1490 + w(8)(3)(2)(5)#1670
                                 + w(9)(3)(2)(5)#1850 + w(10)(3)(2)(5)#2030
                                 + w(11)(3)(2)(5)#2210 + w(12)(3)(2)(5)#2390
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,5): w(3)(4)(2)(5)#800 + w(4)(4)(2)(5)#980
                                 + w(5)(4)(2)(5)#1160 + w(6)(4)(2)(5)#1340
                                 + w(7)(4)(2)(5)#1520 + w(8)(4)(2)(5)#1700
                                 + w(9)(4)(2)(5)#1880 + w(10)(4)(2)(5)#2060
                                 + w(11)(4)(2)(5)#2240 + w(12)(4)(2)(5)#2420
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,5): w(3)(5)(2)(5)#830 + w(4)(5)(2)(5)#1010
                                 + w(5)(5)(2)(5)#1190 + w(6)(5)(2)(5)#1370
                                 + w(7)(5)(2)(5)#1550 + w(8)(5)(2)(5)#1730
                                 + w(9)(5)(2)(5)#1910 + w(10)(5)(2)(5)#2090
                                 + w(11)(5)(2)(5)#2270 + w(12)(5)(2)(5)#2450
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,5): w(3)(6)(2)(5)#860 + w(4)(6)(2)(5)#1040
                                 + w(5)(6)(2)(5)#1220 + w(6)(6)(2)(5)#1400
                                 + w(7)(6)(2)(5)#1580 + w(8)(6)(2)(5)#1760
                                 + w(9)(6)(2)(5)#1940 + w(10)(6)(2)(5)#2120
                                 + w(11)(6)(2)(5)#2300 + w(12)(6)(2)(5)#2480
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,5): w(3)(1)(3)(5)#720 + w(4)(1)(3)(5)#900
                                 + w(5)(1)(3)(5)#1080 + w(6)(1)(3)(5)#1260
                                 + w(7)(1)(3)(5)#1440 + w(8)(1)(3)(5)#1620
                                 + w(9)(1)(3)(5)#1800 + w(10)(1)(3)(5)#1980
                                 + w(11)(1)(3)(5)#2160 + w(12)(1)(3)(5)#2340
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,5): w(3)(2)(3)(5)#750 + w(4)(2)(3)(5)#930
                                 + w(5)(2)(3)(5)#1110 + w(6)(2)(3)(5)#1290
                                 + w(7)(2)(3)(5)#1470 + w(8)(2)(3)(5)#1650
                                 + w(9)(2)(3)(5)#1830 + w(10)(2)(3)(5)#2010
                                 + w(11)(2)(3)(5)#2190 + w(12)(2)(3)(5)#2370
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,5): w(3)(3)(3)(5)#780 + w(4)(3)(3)(5)#960
                                 + w(5)(3)(3)(5)#1140 + w(6)(3)(3)(5)#1320
                                 + w(7)(3)(3)(5)#1500 + w(8)(3)(3)(5)#1680
                                 + w(9)(3)(3)(5)#1860 + w(10)(3)(3)(5)#2040
                                 + w(11)(3)(3)(5)#2220 + w(12)(3)(3)(5)#2400
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,5): w(3)(4)(3)(5)#810 + w(4)(4)(3)(5)#990
                                 + w(5)(4)(3)(5)#1170 + w(6)(4)(3)(5)#1350
                                 + w(7)(4)(3)(5)#1530 + w(8)(4)(3)(5)#1710
                                 + w(9)(4)(3)(5)#1890 + w(10)(4)(3)(5)#2070
                                 + w(11)(4)(3)(5)#2250 + w(12)(4)(3)(5)#2430
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,5): w(3)(5)(3)(5)#840 + w(4)(5)(3)(5)#1020
                                 + w(5)(5)(3)(5)#1200 + w(6)(5)(3)(5)#1380
                                 + w(7)(5)(3)(5)#1560 + w(8)(5)(3)(5)#1740
                                 + w(9)(5)(3)(5)#1920 + w(10)(5)(3)(5)#2100
                                 + w(11)(5)(3)(5)#2280 + w(12)(5)(3)(5)#2460
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,5): w(3)(6)(3)(5)#870 + w(4)(6)(3)(5)#1050
                                 + w(5)(6)(3)(5)#1230 + w(6)(6)(3)(5)#1410
                                 + w(7)(6)(3)(5)#1590 + w(8)(6)(3)(5)#1770
                                 + w(9)(6)(3)(5)#1950 + w(10)(6)(3)(5)#2130
                                 + w(11)(6)(3)(5)#2310 + w(12)(6)(3)(5)#2490
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,6): w(3)(1)(1)(6)#701 + w(4)(1)(1)(6)#881
                                 + w(5)(1)(1)(6)#1061 + w(6)(1)(1)(6)#1241
                                 + w(7)(1)(1)(6)#1421 + w(8)(1)(1)(6)#1601
                                 + w(9)(1)(1)(6)#1781 + w(10)(1)(1)(6)#1961
                                 + w(11)(1)(1)(6)#2141 + w(12)(1)(1)(6)#2321
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,6): w(3)(2)(1)(6)#731 + w(4)(2)(1)(6)#911
                                 + w(5)(2)(1)(6)#1091 + w(6)(2)(1)(6)#1271
                                 + w(7)(2)(1)(6)#1451 + w(8)(2)(1)(6)#1631
                                 + w(9)(2)(1)(6)#1811 + w(10)(2)(1)(6)#1991
                                 + w(11)(2)(1)(6)#2171 + w(12)(2)(1)(6)#2351
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,6): w(3)(3)(1)(6)#761 + w(4)(3)(1)(6)#941
                                 + w(5)(3)(1)(6)#1121 + w(6)(3)(1)(6)#1301
                                 + w(7)(3)(1)(6)#1481 + w(8)(3)(1)(6)#1661
                                 + w(9)(3)(1)(6)#1841 + w(10)(3)(1)(6)#2021
                                 + w(11)(3)(1)(6)#2201 + w(12)(3)(1)(6)#2381
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,6): w(3)(4)(1)(6)#791 + w(4)(4)(1)(6)#971
                                 + w(5)(4)(1)(6)#1151 + w(6)(4)(1)(6)#1331
                                 + w(7)(4)(1)(6)#1511 + w(8)(4)(1)(6)#1691
                                 + w(9)(4)(1)(6)#1871 + w(10)(4)(1)(6)#2051
                                 + w(11)(4)(1)(6)#2231 + w(12)(4)(1)(6)#2411
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,6): w(3)(5)(1)(6)#821 + w(4)(5)(1)(6)#1001
                                 + w(5)(5)(1)(6)#1181 + w(6)(5)(1)(6)#1361
                                 + w(7)(5)(1)(6)#1541 + w(8)(5)(1)(6)#1721
                                 + w(9)(5)(1)(6)#1901 + w(10)(5)(1)(6)#2081
                                 + w(11)(5)(1)(6)#2261 + w(12)(5)(1)(6)#2441
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,6): w(3)(6)(1)(6)#851 + w(4)(6)(1)(6)#1031
                                 + w(5)(6)(1)(6)#1211 + w(6)(6)(1)(6)#1391
                                 + w(7)(6)(1)(6)#1571 + w(8)(6)(1)(6)#1751
                                 + w(9)(6)(1)(6)#1931 + w(10)(6)(1)(6)#2111
                                 + w(11)(6)(1)(6)#2291 + w(12)(6)(1)(6)#2471
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,6): w(3)(1)(2)(6)#711 + w(4)(1)(2)(6)#891
                                 + w(5)(1)(2)(6)#1071 + w(6)(1)(2)(6)#1251
                                 + w(7)(1)(2)(6)#1431 + w(8)(1)(2)(6)#1611
                                 + w(9)(1)(2)(6)#1791 + w(10)(1)(2)(6)#1971
                                 + w(11)(1)(2)(6)#2151 + w(12)(1)(2)(6)#2331
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,6): w(3)(2)(2)(6)#741 + w(4)(2)(2)(6)#921
                                 + w(5)(2)(2)(6)#1101 + w(6)(2)(2)(6)#1281
                                 + w(7)(2)(2)(6)#1461 + w(8)(2)(2)(6)#1641
                                 + w(9)(2)(2)(6)#1821 + w(10)(2)(2)(6)#2001
                                 + w(11)(2)(2)(6)#2181 + w(12)(2)(2)(6)#2361
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,6): w(3)(3)(2)(6)#771 + w(4)(3)(2)(6)#951
                                 + w(5)(3)(2)(6)#1131 + w(6)(3)(2)(6)#1311
                                 + w(7)(3)(2)(6)#1491 + w(8)(3)(2)(6)#1671
                                 + w(9)(3)(2)(6)#1851 + w(10)(3)(2)(6)#2031
                                 + w(11)(3)(2)(6)#2211 + w(12)(3)(2)(6)#2391
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,6): w(3)(4)(2)(6)#801 + w(4)(4)(2)(6)#981
                                 + w(5)(4)(2)(6)#1161 + w(6)(4)(2)(6)#1341
                                 + w(7)(4)(2)(6)#1521 + w(8)(4)(2)(6)#1701
                                 + w(9)(4)(2)(6)#1881 + w(10)(4)(2)(6)#2061
                                 + w(11)(4)(2)(6)#2241 + w(12)(4)(2)(6)#2421
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,6): w(3)(5)(2)(6)#831 + w(4)(5)(2)(6)#1011
                                 + w(5)(5)(2)(6)#1191 + w(6)(5)(2)(6)#1371
                                 + w(7)(5)(2)(6)#1551 + w(8)(5)(2)(6)#1731
                                 + w(9)(5)(2)(6)#1911 + w(10)(5)(2)(6)#2091
                                 + w(11)(5)(2)(6)#2271 + w(12)(5)(2)(6)#2451
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,6): w(3)(6)(2)(6)#861 + w(4)(6)(2)(6)#1041
                                 + w(5)(6)(2)(6)#1221 + w(6)(6)(2)(6)#1401
                                 + w(7)(6)(2)(6)#1581 + w(8)(6)(2)(6)#1761
                                 + w(9)(6)(2)(6)#1941 + w(10)(6)(2)(6)#2121
                                 + w(11)(6)(2)(6)#2301 + w(12)(6)(2)(6)#2481
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,6): w(3)(1)(3)(6)#721 + w(4)(1)(3)(6)#901
                                 + w(5)(1)(3)(6)#1081 + w(6)(1)(3)(6)#1261
                                 + w(7)(1)(3)(6)#1441 + w(8)(1)(3)(6)#1621
                                 + w(9)(1)(3)(6)#1801 + w(10)(1)(3)(6)#1981
                                 + w(11)(1)(3)(6)#2161 + w(12)(1)(3)(6)#2341
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,6): w(3)(2)(3)(6)#751 + w(4)(2)(3)(6)#931
                                 + w(5)(2)(3)(6)#1111 + w(6)(2)(3)(6)#1291
                                 + w(7)(2)(3)(6)#1471 + w(8)(2)(3)(6)#1651
                                 + w(9)(2)(3)(6)#1831 + w(10)(2)(3)(6)#2011
                                 + w(11)(2)(3)(6)#2191 + w(12)(2)(3)(6)#2371
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,6): w(3)(3)(3)(6)#781 + w(4)(3)(3)(6)#961
                                 + w(5)(3)(3)(6)#1141 + w(6)(3)(3)(6)#1321
                                 + w(7)(3)(3)(6)#1501 + w(8)(3)(3)(6)#1681
                                 + w(9)(3)(3)(6)#1861 + w(10)(3)(3)(6)#2041
                                 + w(11)(3)(3)(6)#2221 + w(12)(3)(3)(6)#2401
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,6): w(3)(4)(3)(6)#811 + w(4)(4)(3)(6)#991
                                 + w(5)(4)(3)(6)#1171 + w(6)(4)(3)(6)#1351
                                 + w(7)(4)(3)(6)#1531 + w(8)(4)(3)(6)#1711
                                 + w(9)(4)(3)(6)#1891 + w(10)(4)(3)(6)#2071
                                 + w(11)(4)(3)(6)#2251 + w(12)(4)(3)(6)#2431
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,6): w(3)(5)(3)(6)#841 + w(4)(5)(3)(6)#1021
                                 + w(5)(5)(3)(6)#1201 + w(6)(5)(3)(6)#1381
                                 + w(7)(5)(3)(6)#1561 + w(8)(5)(3)(6)#1741
                                 + w(9)(5)(3)(6)#1921 + w(10)(5)(3)(6)#2101
                                 + w(11)(5)(3)(6)#2281 + w(12)(5)(3)(6)#2461
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,6): w(3)(6)(3)(6)#871 + w(4)(6)(3)(6)#1051
                                 + w(5)(6)(3)(6)#1231 + w(6)(6)(3)(6)#1411
                                 + w(7)(6)(3)(6)#1591 + w(8)(6)(3)(6)#1771
                                 + w(9)(6)(3)(6)#1951 + w(10)(6)(3)(6)#2131
                                 + w(11)(6)(3)(6)#2311 + w(12)(6)(3)(6)#2491
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,7): w(3)(1)(1)(7)#702 + w(4)(1)(1)(7)#882
                                 + w(5)(1)(1)(7)#1062 + w(6)(1)(1)(7)#1242
                                 + w(7)(1)(1)(7)#1422 + w(8)(1)(1)(7)#1602
                                 + w(9)(1)(1)(7)#1782 + w(10)(1)(1)(7)#1962
                                 + w(11)(1)(1)(7)#2142 + w(12)(1)(1)(7)#2322
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,7): w(3)(2)(1)(7)#732 + w(4)(2)(1)(7)#912
                                 + w(5)(2)(1)(7)#1092 + w(6)(2)(1)(7)#1272
                                 + w(7)(2)(1)(7)#1452 + w(8)(2)(1)(7)#1632
                                 + w(9)(2)(1)(7)#1812 + w(10)(2)(1)(7)#1992
                                 + w(11)(2)(1)(7)#2172 + w(12)(2)(1)(7)#2352
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,7): w(3)(3)(1)(7)#762 + w(4)(3)(1)(7)#942
                                 + w(5)(3)(1)(7)#1122 + w(6)(3)(1)(7)#1302
                                 + w(7)(3)(1)(7)#1482 + w(8)(3)(1)(7)#1662
                                 + w(9)(3)(1)(7)#1842 + w(10)(3)(1)(7)#2022
                                 + w(11)(3)(1)(7)#2202 + w(12)(3)(1)(7)#2382
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,7): w(3)(4)(1)(7)#792 + w(4)(4)(1)(7)#972
                                 + w(5)(4)(1)(7)#1152 + w(6)(4)(1)(7)#1332
                                 + w(7)(4)(1)(7)#1512 + w(8)(4)(1)(7)#1692
                                 + w(9)(4)(1)(7)#1872 + w(10)(4)(1)(7)#2052
                                 + w(11)(4)(1)(7)#2232 + w(12)(4)(1)(7)#2412
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,7): w(3)(5)(1)(7)#822 + w(4)(5)(1)(7)#1002
                                 + w(5)(5)(1)(7)#1182 + w(6)(5)(1)(7)#1362
                                 + w(7)(5)(1)(7)#1542 + w(8)(5)(1)(7)#1722
                                 + w(9)(5)(1)(7)#1902 + w(10)(5)(1)(7)#2082
                                 + w(11)(5)(1)(7)#2262 + w(12)(5)(1)(7)#2442
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,7): w(3)(6)(1)(7)#852 + w(4)(6)(1)(7)#1032
                                 + w(5)(6)(1)(7)#1212 + w(6)(6)(1)(7)#1392
                                 + w(7)(6)(1)(7)#1572 + w(8)(6)(1)(7)#1752
                                 + w(9)(6)(1)(7)#1932 + w(10)(6)(1)(7)#2112
                                 + w(11)(6)(1)(7)#2292 + w(12)(6)(1)(7)#2472
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,7): w(3)(1)(2)(7)#712 + w(4)(1)(2)(7)#892
                                 + w(5)(1)(2)(7)#1072 + w(6)(1)(2)(7)#1252
                                 + w(7)(1)(2)(7)#1432 + w(8)(1)(2)(7)#1612
                                 + w(9)(1)(2)(7)#1792 + w(10)(1)(2)(7)#1972
                                 + w(11)(1)(2)(7)#2152 + w(12)(1)(2)(7)#2332
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,7): w(3)(2)(2)(7)#742 + w(4)(2)(2)(7)#922
                                 + w(5)(2)(2)(7)#1102 + w(6)(2)(2)(7)#1282
                                 + w(7)(2)(2)(7)#1462 + w(8)(2)(2)(7)#1642
                                 + w(9)(2)(2)(7)#1822 + w(10)(2)(2)(7)#2002
                                 + w(11)(2)(2)(7)#2182 + w(12)(2)(2)(7)#2362
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,7): w(3)(3)(2)(7)#772 + w(4)(3)(2)(7)#952
                                 + w(5)(3)(2)(7)#1132 + w(6)(3)(2)(7)#1312
                                 + w(7)(3)(2)(7)#1492 + w(8)(3)(2)(7)#1672
                                 + w(9)(3)(2)(7)#1852 + w(10)(3)(2)(7)#2032
                                 + w(11)(3)(2)(7)#2212 + w(12)(3)(2)(7)#2392
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,7): w(3)(4)(2)(7)#802 + w(4)(4)(2)(7)#982
                                 + w(5)(4)(2)(7)#1162 + w(6)(4)(2)(7)#1342
                                 + w(7)(4)(2)(7)#1522 + w(8)(4)(2)(7)#1702
                                 + w(9)(4)(2)(7)#1882 + w(10)(4)(2)(7)#2062
                                 + w(11)(4)(2)(7)#2242 + w(12)(4)(2)(7)#2422
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,7): w(3)(5)(2)(7)#832 + w(4)(5)(2)(7)#1012
                                 + w(5)(5)(2)(7)#1192 + w(6)(5)(2)(7)#1372
                                 + w(7)(5)(2)(7)#1552 + w(8)(5)(2)(7)#1732
                                 + w(9)(5)(2)(7)#1912 + w(10)(5)(2)(7)#2092
                                 + w(11)(5)(2)(7)#2272 + w(12)(5)(2)(7)#2452
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,7): w(3)(6)(2)(7)#862 + w(4)(6)(2)(7)#1042
                                 + w(5)(6)(2)(7)#1222 + w(6)(6)(2)(7)#1402
                                 + w(7)(6)(2)(7)#1582 + w(8)(6)(2)(7)#1762
                                 + w(9)(6)(2)(7)#1942 + w(10)(6)(2)(7)#2122
                                 + w(11)(6)(2)(7)#2302 + w(12)(6)(2)(7)#2482
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,7): w(3)(1)(3)(7)#722 + w(4)(1)(3)(7)#902
                                 + w(5)(1)(3)(7)#1082 + w(6)(1)(3)(7)#1262
                                 + w(7)(1)(3)(7)#1442 + w(8)(1)(3)(7)#1622
                                 + w(9)(1)(3)(7)#1802 + w(10)(1)(3)(7)#1982
                                 + w(11)(1)(3)(7)#2162 + w(12)(1)(3)(7)#2342
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,7): w(3)(2)(3)(7)#752 + w(4)(2)(3)(7)#932
                                 + w(5)(2)(3)(7)#1112 + w(6)(2)(3)(7)#1292
                                 + w(7)(2)(3)(7)#1472 + w(8)(2)(3)(7)#1652
                                 + w(9)(2)(3)(7)#1832 + w(10)(2)(3)(7)#2012
                                 + w(11)(2)(3)(7)#2192 + w(12)(2)(3)(7)#2372
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,7): w(3)(3)(3)(7)#782 + w(4)(3)(3)(7)#962
                                 + w(5)(3)(3)(7)#1142 + w(6)(3)(3)(7)#1322
                                 + w(7)(3)(3)(7)#1502 + w(8)(3)(3)(7)#1682
                                 + w(9)(3)(3)(7)#1862 + w(10)(3)(3)(7)#2042
                                 + w(11)(3)(3)(7)#2222 + w(12)(3)(3)(7)#2402
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,7): w(3)(4)(3)(7)#812 + w(4)(4)(3)(7)#992
                                 + w(5)(4)(3)(7)#1172 + w(6)(4)(3)(7)#1352
                                 + w(7)(4)(3)(7)#1532 + w(8)(4)(3)(7)#1712
                                 + w(9)(4)(3)(7)#1892 + w(10)(4)(3)(7)#2072
                                 + w(11)(4)(3)(7)#2252 + w(12)(4)(3)(7)#2432
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,7): w(3)(5)(3)(7)#842 + w(4)(5)(3)(7)#1022
                                 + w(5)(5)(3)(7)#1202 + w(6)(5)(3)(7)#1382
                                 + w(7)(5)(3)(7)#1562 + w(8)(5)(3)(7)#1742
                                 + w(9)(5)(3)(7)#1922 + w(10)(5)(3)(7)#2102
                                 + w(11)(5)(3)(7)#2282 + w(12)(5)(3)(7)#2462
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,7): w(3)(6)(3)(7)#872 + w(4)(6)(3)(7)#1052
                                 + w(5)(6)(3)(7)#1232 + w(6)(6)(3)(7)#1412
                                 + w(7)(6)(3)(7)#1592 + w(8)(6)(3)(7)#1772
                                 + w(9)(6)(3)(7)#1952 + w(10)(6)(3)(7)#2132
                                 + w(11)(6)(3)(7)#2312 + w(12)(6)(3)(7)#2492
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,8): w(3)(1)(1)(8)#703 + w(4)(1)(1)(8)#883
                                 + w(5)(1)(1)(8)#1063 + w(6)(1)(1)(8)#1243
                                 + w(7)(1)(1)(8)#1423 + w(8)(1)(1)(8)#1603
                                 + w(9)(1)(1)(8)#1783 + w(10)(1)(1)(8)#1963
                                 + w(11)(1)(1)(8)#2143 + w(12)(1)(1)(8)#2323
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,8): w(3)(2)(1)(8)#733 + w(4)(2)(1)(8)#913
                                 + w(5)(2)(1)(8)#1093 + w(6)(2)(1)(8)#1273
                                 + w(7)(2)(1)(8)#1453 + w(8)(2)(1)(8)#1633
                                 + w(9)(2)(1)(8)#1813 + w(10)(2)(1)(8)#1993
                                 + w(11)(2)(1)(8)#2173 + w(12)(2)(1)(8)#2353
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,8): w(3)(3)(1)(8)#763 + w(4)(3)(1)(8)#943
                                 + w(5)(3)(1)(8)#1123 + w(6)(3)(1)(8)#1303
                                 + w(7)(3)(1)(8)#1483 + w(8)(3)(1)(8)#1663
                                 + w(9)(3)(1)(8)#1843 + w(10)(3)(1)(8)#2023
                                 + w(11)(3)(1)(8)#2203 + w(12)(3)(1)(8)#2383
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,8): w(3)(4)(1)(8)#793 + w(4)(4)(1)(8)#973
                                 + w(5)(4)(1)(8)#1153 + w(6)(4)(1)(8)#1333
                                 + w(7)(4)(1)(8)#1513 + w(8)(4)(1)(8)#1693
                                 + w(9)(4)(1)(8)#1873 + w(10)(4)(1)(8)#2053
                                 + w(11)(4)(1)(8)#2233 + w(12)(4)(1)(8)#2413
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,8): w(3)(5)(1)(8)#823 + w(4)(5)(1)(8)#1003
                                 + w(5)(5)(1)(8)#1183 + w(6)(5)(1)(8)#1363
                                 + w(7)(5)(1)(8)#1543 + w(8)(5)(1)(8)#1723
                                 + w(9)(5)(1)(8)#1903 + w(10)(5)(1)(8)#2083
                                 + w(11)(5)(1)(8)#2263 + w(12)(5)(1)(8)#2443
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,8): w(3)(6)(1)(8)#853 + w(4)(6)(1)(8)#1033
                                 + w(5)(6)(1)(8)#1213 + w(6)(6)(1)(8)#1393
                                 + w(7)(6)(1)(8)#1573 + w(8)(6)(1)(8)#1753
                                 + w(9)(6)(1)(8)#1933 + w(10)(6)(1)(8)#2113
                                 + w(11)(6)(1)(8)#2293 + w(12)(6)(1)(8)#2473
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,8): w(3)(1)(2)(8)#713 + w(4)(1)(2)(8)#893
                                 + w(5)(1)(2)(8)#1073 + w(6)(1)(2)(8)#1253
                                 + w(7)(1)(2)(8)#1433 + w(8)(1)(2)(8)#1613
                                 + w(9)(1)(2)(8)#1793 + w(10)(1)(2)(8)#1973
                                 + w(11)(1)(2)(8)#2153 + w(12)(1)(2)(8)#2333
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,8): w(3)(2)(2)(8)#743 + w(4)(2)(2)(8)#923
                                 + w(5)(2)(2)(8)#1103 + w(6)(2)(2)(8)#1283
                                 + w(7)(2)(2)(8)#1463 + w(8)(2)(2)(8)#1643
                                 + w(9)(2)(2)(8)#1823 + w(10)(2)(2)(8)#2003
                                 + w(11)(2)(2)(8)#2183 + w(12)(2)(2)(8)#2363
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,8): w(3)(3)(2)(8)#773 + w(4)(3)(2)(8)#953
                                 + w(5)(3)(2)(8)#1133 + w(6)(3)(2)(8)#1313
                                 + w(7)(3)(2)(8)#1493 + w(8)(3)(2)(8)#1673
                                 + w(9)(3)(2)(8)#1853 + w(10)(3)(2)(8)#2033
                                 + w(11)(3)(2)(8)#2213 + w(12)(3)(2)(8)#2393
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,8): w(3)(4)(2)(8)#803 + w(4)(4)(2)(8)#983
                                 + w(5)(4)(2)(8)#1163 + w(6)(4)(2)(8)#1343
                                 + w(7)(4)(2)(8)#1523 + w(8)(4)(2)(8)#1703
                                 + w(9)(4)(2)(8)#1883 + w(10)(4)(2)(8)#2063
                                 + w(11)(4)(2)(8)#2243 + w(12)(4)(2)(8)#2423
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,8): w(3)(5)(2)(8)#833 + w(4)(5)(2)(8)#1013
                                 + w(5)(5)(2)(8)#1193 + w(6)(5)(2)(8)#1373
                                 + w(7)(5)(2)(8)#1553 + w(8)(5)(2)(8)#1733
                                 + w(9)(5)(2)(8)#1913 + w(10)(5)(2)(8)#2093
                                 + w(11)(5)(2)(8)#2273 + w(12)(5)(2)(8)#2453
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,8): w(3)(6)(2)(8)#863 + w(4)(6)(2)(8)#1043
                                 + w(5)(6)(2)(8)#1223 + w(6)(6)(2)(8)#1403
                                 + w(7)(6)(2)(8)#1583 + w(8)(6)(2)(8)#1763
                                 + w(9)(6)(2)(8)#1943 + w(10)(6)(2)(8)#2123
                                 + w(11)(6)(2)(8)#2303 + w(12)(6)(2)(8)#2483
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,8): w(3)(1)(3)(8)#723 + w(4)(1)(3)(8)#903
                                 + w(5)(1)(3)(8)#1083 + w(6)(1)(3)(8)#1263
                                 + w(7)(1)(3)(8)#1443 + w(8)(1)(3)(8)#1623
                                 + w(9)(1)(3)(8)#1803 + w(10)(1)(3)(8)#1983
                                 + w(11)(1)(3)(8)#2163 + w(12)(1)(3)(8)#2343
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,8): w(3)(2)(3)(8)#753 + w(4)(2)(3)(8)#933
                                 + w(5)(2)(3)(8)#1113 + w(6)(2)(3)(8)#1293
                                 + w(7)(2)(3)(8)#1473 + w(8)(2)(3)(8)#1653
                                 + w(9)(2)(3)(8)#1833 + w(10)(2)(3)(8)#2013
                                 + w(11)(2)(3)(8)#2193 + w(12)(2)(3)(8)#2373
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,8): w(3)(3)(3)(8)#783 + w(4)(3)(3)(8)#963
                                 + w(5)(3)(3)(8)#1143 + w(6)(3)(3)(8)#1323
                                 + w(7)(3)(3)(8)#1503 + w(8)(3)(3)(8)#1683
                                 + w(9)(3)(3)(8)#1863 + w(10)(3)(3)(8)#2043
                                 + w(11)(3)(3)(8)#2223 + w(12)(3)(3)(8)#2403
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,8): w(3)(4)(3)(8)#813 + w(4)(4)(3)(8)#993
                                 + w(5)(4)(3)(8)#1173 + w(6)(4)(3)(8)#1353
                                 + w(7)(4)(3)(8)#1533 + w(8)(4)(3)(8)#1713
                                 + w(9)(4)(3)(8)#1893 + w(10)(4)(3)(8)#2073
                                 + w(11)(4)(3)(8)#2253 + w(12)(4)(3)(8)#2433
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,8): w(3)(5)(3)(8)#843 + w(4)(5)(3)(8)#1023
                                 + w(5)(5)(3)(8)#1203 + w(6)(5)(3)(8)#1383
                                 + w(7)(5)(3)(8)#1563 + w(8)(5)(3)(8)#1743
                                 + w(9)(5)(3)(8)#1923 + w(10)(5)(3)(8)#2103
                                 + w(11)(5)(3)(8)#2283 + w(12)(5)(3)(8)#2463
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,8): w(3)(6)(3)(8)#873 + w(4)(6)(3)(8)#1053
                                 + w(5)(6)(3)(8)#1233 + w(6)(6)(3)(8)#1413
                                 + w(7)(6)(3)(8)#1593 + w(8)(6)(3)(8)#1773
                                 + w(9)(6)(3)(8)#1953 + w(10)(6)(3)(8)#2133
                                 + w(11)(6)(3)(8)#2313 + w(12)(6)(3)(8)#2493
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,9): w(3)(1)(1)(9)#704 + w(4)(1)(1)(9)#884
                                 + w(5)(1)(1)(9)#1064 + w(6)(1)(1)(9)#1244
                                 + w(7)(1)(1)(9)#1424 + w(8)(1)(1)(9)#1604
                                 + w(9)(1)(1)(9)#1784 + w(10)(1)(1)(9)#1964
                                 + w(11)(1)(1)(9)#2144 + w(12)(1)(1)(9)#2324
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,9): w(3)(2)(1)(9)#734 + w(4)(2)(1)(9)#914
                                 + w(5)(2)(1)(9)#1094 + w(6)(2)(1)(9)#1274
                                 + w(7)(2)(1)(9)#1454 + w(8)(2)(1)(9)#1634
                                 + w(9)(2)(1)(9)#1814 + w(10)(2)(1)(9)#1994
                                 + w(11)(2)(1)(9)#2174 + w(12)(2)(1)(9)#2354
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,9): w(3)(3)(1)(9)#764 + w(4)(3)(1)(9)#944
                                 + w(5)(3)(1)(9)#1124 + w(6)(3)(1)(9)#1304
                                 + w(7)(3)(1)(9)#1484 + w(8)(3)(1)(9)#1664
                                 + w(9)(3)(1)(9)#1844 + w(10)(3)(1)(9)#2024
                                 + w(11)(3)(1)(9)#2204 + w(12)(3)(1)(9)#2384
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,9): w(3)(4)(1)(9)#794 + w(4)(4)(1)(9)#974
                                 + w(5)(4)(1)(9)#1154 + w(6)(4)(1)(9)#1334
                                 + w(7)(4)(1)(9)#1514 + w(8)(4)(1)(9)#1694
                                 + w(9)(4)(1)(9)#1874 + w(10)(4)(1)(9)#2054
                                 + w(11)(4)(1)(9)#2234 + w(12)(4)(1)(9)#2414
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,9): w(3)(5)(1)(9)#824 + w(4)(5)(1)(9)#1004
                                 + w(5)(5)(1)(9)#1184 + w(6)(5)(1)(9)#1364
                                 + w(7)(5)(1)(9)#1544 + w(8)(5)(1)(9)#1724
                                 + w(9)(5)(1)(9)#1904 + w(10)(5)(1)(9)#2084
                                 + w(11)(5)(1)(9)#2264 + w(12)(5)(1)(9)#2444
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,9): w(3)(6)(1)(9)#854 + w(4)(6)(1)(9)#1034
                                 + w(5)(6)(1)(9)#1214 + w(6)(6)(1)(9)#1394
                                 + w(7)(6)(1)(9)#1574 + w(8)(6)(1)(9)#1754
                                 + w(9)(6)(1)(9)#1934 + w(10)(6)(1)(9)#2114
                                 + w(11)(6)(1)(9)#2294 + w(12)(6)(1)(9)#2474
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,9): w(3)(1)(2)(9)#714 + w(4)(1)(2)(9)#894
                                 + w(5)(1)(2)(9)#1074 + w(6)(1)(2)(9)#1254
                                 + w(7)(1)(2)(9)#1434 + w(8)(1)(2)(9)#1614
                                 + w(9)(1)(2)(9)#1794 + w(10)(1)(2)(9)#1974
                                 + w(11)(1)(2)(9)#2154 + w(12)(1)(2)(9)#2334
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,9): w(3)(2)(2)(9)#744 + w(4)(2)(2)(9)#924
                                 + w(5)(2)(2)(9)#1104 + w(6)(2)(2)(9)#1284
                                 + w(7)(2)(2)(9)#1464 + w(8)(2)(2)(9)#1644
                                 + w(9)(2)(2)(9)#1824 + w(10)(2)(2)(9)#2004
                                 + w(11)(2)(2)(9)#2184 + w(12)(2)(2)(9)#2364
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,9): w(3)(3)(2)(9)#774 + w(4)(3)(2)(9)#954
                                 + w(5)(3)(2)(9)#1134 + w(6)(3)(2)(9)#1314
                                 + w(7)(3)(2)(9)#1494 + w(8)(3)(2)(9)#1674
                                 + w(9)(3)(2)(9)#1854 + w(10)(3)(2)(9)#2034
                                 + w(11)(3)(2)(9)#2214 + w(12)(3)(2)(9)#2394
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,9): w(3)(4)(2)(9)#804 + w(4)(4)(2)(9)#984
                                 + w(5)(4)(2)(9)#1164 + w(6)(4)(2)(9)#1344
                                 + w(7)(4)(2)(9)#1524 + w(8)(4)(2)(9)#1704
                                 + w(9)(4)(2)(9)#1884 + w(10)(4)(2)(9)#2064
                                 + w(11)(4)(2)(9)#2244 + w(12)(4)(2)(9)#2424
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,9): w(3)(5)(2)(9)#834 + w(4)(5)(2)(9)#1014
                                 + w(5)(5)(2)(9)#1194 + w(6)(5)(2)(9)#1374
                                 + w(7)(5)(2)(9)#1554 + w(8)(5)(2)(9)#1734
                                 + w(9)(5)(2)(9)#1914 + w(10)(5)(2)(9)#2094
                                 + w(11)(5)(2)(9)#2274 + w(12)(5)(2)(9)#2454
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,9): w(3)(6)(2)(9)#864 + w(4)(6)(2)(9)#1044
                                 + w(5)(6)(2)(9)#1224 + w(6)(6)(2)(9)#1404
                                 + w(7)(6)(2)(9)#1584 + w(8)(6)(2)(9)#1764
                                 + w(9)(6)(2)(9)#1944 + w(10)(6)(2)(9)#2124
                                 + w(11)(6)(2)(9)#2304 + w(12)(6)(2)(9)#2484
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,9): w(3)(1)(3)(9)#724 + w(4)(1)(3)(9)#904
                                 + w(5)(1)(3)(9)#1084 + w(6)(1)(3)(9)#1264
                                 + w(7)(1)(3)(9)#1444 + w(8)(1)(3)(9)#1624
                                 + w(9)(1)(3)(9)#1804 + w(10)(1)(3)(9)#1984
                                 + w(11)(1)(3)(9)#2164 + w(12)(1)(3)(9)#2344
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,9): w(3)(2)(3)(9)#754 + w(4)(2)(3)(9)#934
                                 + w(5)(2)(3)(9)#1114 + w(6)(2)(3)(9)#1294
                                 + w(7)(2)(3)(9)#1474 + w(8)(2)(3)(9)#1654
                                 + w(9)(2)(3)(9)#1834 + w(10)(2)(3)(9)#2014
                                 + w(11)(2)(3)(9)#2194 + w(12)(2)(3)(9)#2374
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,9): w(3)(3)(3)(9)#784 + w(4)(3)(3)(9)#964
                                 + w(5)(3)(3)(9)#1144 + w(6)(3)(3)(9)#1324
                                 + w(7)(3)(3)(9)#1504 + w(8)(3)(3)(9)#1684
                                 + w(9)(3)(3)(9)#1864 + w(10)(3)(3)(9)#2044
                                 + w(11)(3)(3)(9)#2224 + w(12)(3)(3)(9)#2404
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,9): w(3)(4)(3)(9)#814 + w(4)(4)(3)(9)#994
                                 + w(5)(4)(3)(9)#1174 + w(6)(4)(3)(9)#1354
                                 + w(7)(4)(3)(9)#1534 + w(8)(4)(3)(9)#1714
                                 + w(9)(4)(3)(9)#1894 + w(10)(4)(3)(9)#2074
                                 + w(11)(4)(3)(9)#2254 + w(12)(4)(3)(9)#2434
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,9): w(3)(5)(3)(9)#844 + w(4)(5)(3)(9)#1024
                                 + w(5)(5)(3)(9)#1204 + w(6)(5)(3)(9)#1384
                                 + w(7)(5)(3)(9)#1564 + w(8)(5)(3)(9)#1744
                                 + w(9)(5)(3)(9)#1924 + w(10)(5)(3)(9)#2104
                                 + w(11)(5)(3)(9)#2284 + w(12)(5)(3)(9)#2464
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,9): w(3)(6)(3)(9)#874 + w(4)(6)(3)(9)#1054
                                 + w(5)(6)(3)(9)#1234 + w(6)(6)(3)(9)#1414
                                 + w(7)(6)(3)(9)#1594 + w(8)(6)(3)(9)#1774
                                 + w(9)(6)(3)(9)#1954 + w(10)(6)(3)(9)#2134
                                 + w(11)(6)(3)(9)#2314 + w(12)(6)(3)(9)#2494
                                 <= 44
 VehicleCapacityWarehouse(1,1,1,10): w(3)(1)(1)(10)#705 + w(4)(1)(1)(10)#885
                                 + w(5)(1)(1)(10)#1065 + w(6)(1)(1)(10)#1245
                                 + w(7)(1)(1)(10)#1425 + w(8)(1)(1)(10)#1605
                                 + w(9)(1)(1)(10)#1785 + w(10)(1)(1)(10)#1965
                                 + w(11)(1)(1)(10)#2145 + w(12)(1)(1)(10)#2325
                                 <= 44
 VehicleCapacityWarehouse(1,2,1,10): w(3)(2)(1)(10)#735 + w(4)(2)(1)(10)#915
                                 + w(5)(2)(1)(10)#1095 + w(6)(2)(1)(10)#1275
                                 + w(7)(2)(1)(10)#1455 + w(8)(2)(1)(10)#1635
                                 + w(9)(2)(1)(10)#1815 + w(10)(2)(1)(10)#1995
                                 + w(11)(2)(1)(10)#2175 + w(12)(2)(1)(10)#2355
                                 <= 44
 VehicleCapacityWarehouse(1,3,1,10): w(3)(3)(1)(10)#765 + w(4)(3)(1)(10)#945
                                 + w(5)(3)(1)(10)#1125 + w(6)(3)(1)(10)#1305
                                 + w(7)(3)(1)(10)#1485 + w(8)(3)(1)(10)#1665
                                 + w(9)(3)(1)(10)#1845 + w(10)(3)(1)(10)#2025
                                 + w(11)(3)(1)(10)#2205 + w(12)(3)(1)(10)#2385
                                 <= 44
 VehicleCapacityWarehouse(2,4,1,10): w(3)(4)(1)(10)#795 + w(4)(4)(1)(10)#975
                                 + w(5)(4)(1)(10)#1155 + w(6)(4)(1)(10)#1335
                                 + w(7)(4)(1)(10)#1515 + w(8)(4)(1)(10)#1695
                                 + w(9)(4)(1)(10)#1875 + w(10)(4)(1)(10)#2055
                                 + w(11)(4)(1)(10)#2235 + w(12)(4)(1)(10)#2415
                                 <= 44
 VehicleCapacityWarehouse(2,5,1,10): w(3)(5)(1)(10)#825 + w(4)(5)(1)(10)#1005
                                 + w(5)(5)(1)(10)#1185 + w(6)(5)(1)(10)#1365
                                 + w(7)(5)(1)(10)#1545 + w(8)(5)(1)(10)#1725
                                 + w(9)(5)(1)(10)#1905 + w(10)(5)(1)(10)#2085
                                 + w(11)(5)(1)(10)#2265 + w(12)(5)(1)(10)#2445
                                 <= 44
 VehicleCapacityWarehouse(2,6,1,10): w(3)(6)(1)(10)#855 + w(4)(6)(1)(10)#1035
                                 + w(5)(6)(1)(10)#1215 + w(6)(6)(1)(10)#1395
                                 + w(7)(6)(1)(10)#1575 + w(8)(6)(1)(10)#1755
                                 + w(9)(6)(1)(10)#1935 + w(10)(6)(1)(10)#2115
                                 + w(11)(6)(1)(10)#2295 + w(12)(6)(1)(10)#2475
                                 <= 44
 VehicleCapacityWarehouse(1,1,2,10): w(3)(1)(2)(10)#715 + w(4)(1)(2)(10)#895
                                 + w(5)(1)(2)(10)#1075 + w(6)(1)(2)(10)#1255
                                 + w(7)(1)(2)(10)#1435 + w(8)(1)(2)(10)#1615
                                 + w(9)(1)(2)(10)#1795 + w(10)(1)(2)(10)#1975
                                 + w(11)(1)(2)(10)#2155 + w(12)(1)(2)(10)#2335
                                 <= 44
 VehicleCapacityWarehouse(1,2,2,10): w(3)(2)(2)(10)#745 + w(4)(2)(2)(10)#925
                                 + w(5)(2)(2)(10)#1105 + w(6)(2)(2)(10)#1285
                                 + w(7)(2)(2)(10)#1465 + w(8)(2)(2)(10)#1645
                                 + w(9)(2)(2)(10)#1825 + w(10)(2)(2)(10)#2005
                                 + w(11)(2)(2)(10)#2185 + w(12)(2)(2)(10)#2365
                                 <= 44
 VehicleCapacityWarehouse(1,3,2,10): w(3)(3)(2)(10)#775 + w(4)(3)(2)(10)#955
                                 + w(5)(3)(2)(10)#1135 + w(6)(3)(2)(10)#1315
                                 + w(7)(3)(2)(10)#1495 + w(8)(3)(2)(10)#1675
                                 + w(9)(3)(2)(10)#1855 + w(10)(3)(2)(10)#2035
                                 + w(11)(3)(2)(10)#2215 + w(12)(3)(2)(10)#2395
                                 <= 44
 VehicleCapacityWarehouse(2,4,2,10): w(3)(4)(2)(10)#805 + w(4)(4)(2)(10)#985
                                 + w(5)(4)(2)(10)#1165 + w(6)(4)(2)(10)#1345
                                 + w(7)(4)(2)(10)#1525 + w(8)(4)(2)(10)#1705
                                 + w(9)(4)(2)(10)#1885 + w(10)(4)(2)(10)#2065
                                 + w(11)(4)(2)(10)#2245 + w(12)(4)(2)(10)#2425
                                 <= 44
 VehicleCapacityWarehouse(2,5,2,10): w(3)(5)(2)(10)#835 + w(4)(5)(2)(10)#1015
                                 + w(5)(5)(2)(10)#1195 + w(6)(5)(2)(10)#1375
                                 + w(7)(5)(2)(10)#1555 + w(8)(5)(2)(10)#1735
                                 + w(9)(5)(2)(10)#1915 + w(10)(5)(2)(10)#2095
                                 + w(11)(5)(2)(10)#2275 + w(12)(5)(2)(10)#2455
                                 <= 44
 VehicleCapacityWarehouse(2,6,2,10): w(3)(6)(2)(10)#865 + w(4)(6)(2)(10)#1045
                                 + w(5)(6)(2)(10)#1225 + w(6)(6)(2)(10)#1405
                                 + w(7)(6)(2)(10)#1585 + w(8)(6)(2)(10)#1765
                                 + w(9)(6)(2)(10)#1945 + w(10)(6)(2)(10)#2125
                                 + w(11)(6)(2)(10)#2305 + w(12)(6)(2)(10)#2485
                                 <= 44
 VehicleCapacityWarehouse(1,1,3,10): w(3)(1)(3)(10)#725 + w(4)(1)(3)(10)#905
                                 + w(5)(1)(3)(10)#1085 + w(6)(1)(3)(10)#1265
                                 + w(7)(1)(3)(10)#1445 + w(8)(1)(3)(10)#1625
                                 + w(9)(1)(3)(10)#1805 + w(10)(1)(3)(10)#1985
                                 + w(11)(1)(3)(10)#2165 + w(12)(1)(3)(10)#2345
                                 <= 44
 VehicleCapacityWarehouse(1,2,3,10): w(3)(2)(3)(10)#755 + w(4)(2)(3)(10)#935
                                 + w(5)(2)(3)(10)#1115 + w(6)(2)(3)(10)#1295
                                 + w(7)(2)(3)(10)#1475 + w(8)(2)(3)(10)#1655
                                 + w(9)(2)(3)(10)#1835 + w(10)(2)(3)(10)#2015
                                 + w(11)(2)(3)(10)#2195 + w(12)(2)(3)(10)#2375
                                 <= 44
 VehicleCapacityWarehouse(1,3,3,10): w(3)(3)(3)(10)#785 + w(4)(3)(3)(10)#965
                                 + w(5)(3)(3)(10)#1145 + w(6)(3)(3)(10)#1325
                                 + w(7)(3)(3)(10)#1505 + w(8)(3)(3)(10)#1685
                                 + w(9)(3)(3)(10)#1865 + w(10)(3)(3)(10)#2045
                                 + w(11)(3)(3)(10)#2225 + w(12)(3)(3)(10)#2405
                                 <= 44
 VehicleCapacityWarehouse(2,4,3,10): w(3)(4)(3)(10)#815 + w(4)(4)(3)(10)#995
                                 + w(5)(4)(3)(10)#1175 + w(6)(4)(3)(10)#1355
                                 + w(7)(4)(3)(10)#1535 + w(8)(4)(3)(10)#1715
                                 + w(9)(4)(3)(10)#1895 + w(10)(4)(3)(10)#2075
                                 + w(11)(4)(3)(10)#2255 + w(12)(4)(3)(10)#2435
                                 <= 44
 VehicleCapacityWarehouse(2,5,3,10): w(3)(5)(3)(10)#845 + w(4)(5)(3)(10)#1025
                                 + w(5)(5)(3)(10)#1205 + w(6)(5)(3)(10)#1385
                                 + w(7)(5)(3)(10)#1565 + w(8)(5)(3)(10)#1745
                                 + w(9)(5)(3)(10)#1925 + w(10)(5)(3)(10)#2105
                                 + w(11)(5)(3)(10)#2285 + w(12)(5)(3)(10)#2465
                                 <= 44
 VehicleCapacityWarehouse(2,6,3,10): w(3)(6)(3)(10)#875 + w(4)(6)(3)(10)#1055
                                 + w(5)(6)(3)(10)#1235 + w(6)(6)(3)(10)#1415
                                 + w(7)(6)(3)(10)#1595 + w(8)(6)(3)(10)#1775
                                 + w(9)(6)(3)(10)#1955 + w(10)(6)(3)(10)#2135
                                 + w(11)(6)(3)(10)#2315 + w(12)(6)(3)(10)#2495
                                 <= 44
Bounds
 0 <= y(1)#0 <= 1
 0 <= y(2)#1 <= 1
 0 <= y(3)#2 <= 1
 0 <= p(1)#3 <= 304
 0 <= p(2)#4 <= 304
 0 <= p(3)#5 <= 304
 0 <= I_plant(1)#6 <= 152
 0 <= I_plant(2)#7 <= 152
 0 <= I_plant(3)#8 <= 152
 0 <= I_warehouse(1)(1)(1)#9 <= 136.8
 0 <= I_warehouse(1)(1)(2)#10 <= 136.8
 0 <= I_warehouse(1)(1)(3)#11 <= 136.8
 0 <= I_warehouse(1)(1)(4)#12 <= 136.8
 0 <= I_warehouse(1)(1)(5)#13 <= 136.8
 0 <= I_warehouse(1)(1)(6)#14 <= 136.8
 0 <= I_warehouse(1)(1)(7)#15 <= 136.8
 0 <= I_warehouse(1)(1)(8)#16 <= 136.8
 0 <= I_warehouse(1)(1)(9)#17 <= 136.8
 0 <= I_warehouse(1)(1)(10)#18 <= 136.8
 0 <= I_warehouse(1)(2)(1)#19 <= 136.8
 0 <= I_warehouse(1)(2)(2)#20 <= 136.8
 0 <= I_warehouse(1)(2)(3)#21 <= 136.8
 0 <= I_warehouse(1)(2)(4)#22 <= 136.8
 0 <= I_warehouse(1)(2)(5)#23 <= 136.8
 0 <= I_warehouse(1)(2)(6)#24 <= 136.8
 0 <= I_warehouse(1)(2)(7)#25 <= 136.8
 0 <= I_warehouse(1)(2)(8)#26 <= 136.8
 0 <= I_warehouse(1)(2)(9)#27 <= 136.8
 0 <= I_warehouse(1)(2)(10)#28 <= 136.8
 0 <= I_warehouse(1)(3)(1)#29 <= 136.8
 0 <= I_warehouse(1)(3)(2)#30 <= 136.8
 0 <= I_warehouse(1)(3)(3)#31 <= 136.8
 0 <= I_warehouse(1)(3)(4)#32 <= 136.8
 0 <= I_warehouse(1)(3)(5)#33 <= 136.8
 0 <= I_warehouse(1)(3)(6)#34 <= 136.8
 0 <= I_warehouse(1)(3)(7)#35 <= 136.8
 0 <= I_warehouse(1)(3)(8)#36 <= 136.8
 0 <= I_warehouse(1)(3)(9)#37 <= 136.8
 0 <= I_warehouse(1)(3)(10)#38 <= 136.8
 0 <= I_warehouse(2)(1)(1)#39 <= 136.8
 0 <= I_warehouse(2)(1)(2)#40 <= 136.8
 0 <= I_warehouse(2)(1)(3)#41 <= 136.8
 0 <= I_warehouse(2)(1)(4)#42 <= 136.8
 0 <= I_warehouse(2)(1)(5)#43 <= 136.8
 0 <= I_warehouse(2)(1)(6)#44 <= 136.8
 0 <= I_warehouse(2)(1)(7)#45 <= 136.8
 0 <= I_warehouse(2)(1)(8)#46 <= 136.8
 0 <= I_warehouse(2)(1)(9)#47 <= 136.8
 0 <= I_warehouse(2)(1)(10)#48 <= 136.8
 0 <= I_warehouse(2)(2)(1)#49 <= 136.8
 0 <= I_warehouse(2)(2)(2)#50 <= 136.8
 0 <= I_warehouse(2)(2)(3)#51 <= 136.8
 0 <= I_warehouse(2)(2)(4)#52 <= 136.8
 0 <= I_warehouse(2)(2)(5)#53 <= 136.8
 0 <= I_warehouse(2)(2)(6)#54 <= 136.8
 0 <= I_warehouse(2)(2)(7)#55 <= 136.8
 0 <= I_warehouse(2)(2)(8)#56 <= 136.8
 0 <= I_warehouse(2)(2)(9)#57 <= 136.8
 0 <= I_warehouse(2)(2)(10)#58 <= 136.8
 0 <= I_warehouse(2)(3)(1)#59 <= 136.8
 0 <= I_warehouse(2)(3)(2)#60 <= 136.8
 0 <= I_warehouse(2)(3)(3)#61 <= 136.8
 0 <= I_warehouse(2)(3)(4)#62 <= 136.8
 0 <= I_warehouse(2)(3)(5)#63 <= 136.8
 0 <= I_warehouse(2)(3)(6)#64 <= 136.8
 0 <= I_warehouse(2)(3)(7)#65 <= 136.8
 0 <= I_warehouse(2)(3)(8)#66 <= 136.8
 0 <= I_warehouse(2)(3)(9)#67 <= 136.8
 0 <= I_warehouse(2)(3)(10)#68 <= 136.8
 0 <= I_customer(3)(1)(1)#69 <= 20
 0 <= I_customer(3)(1)(2)#70 <= 20
 0 <= I_customer(3)(1)(3)#71 <= 20
 0 <= I_customer(3)(1)(4)#72 <= 20
 0 <= I_customer(3)(1)(5)#73 <= 20
 0 <= I_customer(3)(1)(6)#74 <= 20
 0 <= I_customer(3)(1)(7)#75 <= 20
 0 <= I_customer(3)(1)(8)#76 <= 20
 0 <= I_customer(3)(1)(9)#77 <= 20
 0 <= I_customer(3)(1)(10)#78 <= 20
 0 <= I_customer(3)(2)(1)#79 <= 20
 0 <= I_customer(3)(2)(2)#80 <= 20
 0 <= I_customer(3)(2)(3)#81 <= 20
 0 <= I_customer(3)(2)(4)#82 <= 20
 0 <= I_customer(3)(2)(5)#83 <= 20
 0 <= I_customer(3)(2)(6)#84 <= 20
 0 <= I_customer(3)(2)(7)#85 <= 20
 0 <= I_customer(3)(2)(8)#86 <= 20
 0 <= I_customer(3)(2)(9)#87 <= 20
 0 <= I_customer(3)(2)(10)#88 <= 20
 0 <= I_customer(3)(3)(1)#89 <= 20
 0 <= I_customer(3)(3)(2)#90 <= 20
 0 <= I_customer(3)(3)(3)#91 <= 20
 0 <= I_customer(3)(3)(4)#92 <= 20
 0 <= I_customer(3)(3)(5)#93 <= 20
 0 <= I_customer(3)(3)(6)#94 <= 20
 0 <= I_customer(3)(3)(7)#95 <= 20
 0 <= I_customer(3)(3)(8)#96 <= 20
 0 <= I_customer(3)(3)(9)#97 <= 20
 0 <= I_customer(3)(3)(10)#98 <= 20
 0 <= I_customer(4)(1)(1)#99 <= 45
 0 <= I_customer(4)(1)(2)#100 <= 45
 0 <= I_customer(4)(1)(3)#101 <= 45
 0 <= I_customer(4)(1)(4)#102 <= 45
 0 <= I_customer(4)(1)(5)#103 <= 45
 0 <= I_customer(4)(1)(6)#104 <= 45
 0 <= I_customer(4)(1)(7)#105 <= 45
 0 <= I_customer(4)(1)(8)#106 <= 45
 0 <= I_customer(4)(1)(9)#107 <= 45
 0 <= I_customer(4)(1)(10)#108 <= 45
 0 <= I_customer(4)(2)(1)#109 <= 45
 0 <= I_customer(4)(2)(2)#110 <= 45
 0 <= I_customer(4)(2)(3)#111 <= 45
 0 <= I_customer(4)(2)(4)#112 <= 45
 0 <= I_customer(4)(2)(5)#113 <= 45
 0 <= I_customer(4)(2)(6)#114 <= 45
 0 <= I_customer(4)(2)(7)#115 <= 45
 0 <= I_customer(4)(2)(8)#116 <= 45
 0 <= I_customer(4)(2)(9)#117 <= 45
 0 <= I_customer(4)(2)(10)#118 <= 45
 0 <= I_customer(4)(3)(1)#119 <= 45
 0 <= I_customer(4)(3)(2)#120 <= 45
 0 <= I_customer(4)(3)(3)#121 <= 45
 0 <= I_customer(4)(3)(4)#122 <= 45
 0 <= I_customer(4)(3)(5)#123 <= 45
 0 <= I_customer(4)(3)(6)#124 <= 45
 0 <= I_customer(4)(3)(7)#125 <= 45
 0 <= I_customer(4)(3)(8)#126 <= 45
 0 <= I_customer(4)(3)(9)#127 <= 45
 0 <= I_customer(4)(3)(10)#128 <= 45
 0 <= I_customer(5)(1)(1)#129 <= 45
 0 <= I_customer(5)(1)(2)#130 <= 45
 0 <= I_customer(5)(1)(3)#131 <= 45
 0 <= I_customer(5)(1)(4)#132 <= 45
 0 <= I_customer(5)(1)(5)#133 <= 45
 0 <= I_customer(5)(1)(6)#134 <= 45
 0 <= I_customer(5)(1)(7)#135 <= 45
 0 <= I_customer(5)(1)(8)#136 <= 45
 0 <= I_customer(5)(1)(9)#137 <= 45
 0 <= I_customer(5)(1)(10)#138 <= 45
 0 <= I_customer(5)(2)(1)#139 <= 45
 0 <= I_customer(5)(2)(2)#140 <= 45
 0 <= I_customer(5)(2)(3)#141 <= 45
 0 <= I_customer(5)(2)(4)#142 <= 45
 0 <= I_customer(5)(2)(5)#143 <= 45
 0 <= I_customer(5)(2)(6)#144 <= 45
 0 <= I_customer(5)(2)(7)#145 <= 45
 0 <= I_customer(5)(2)(8)#146 <= 45
 0 <= I_customer(5)(2)(9)#147 <= 45
 0 <= I_customer(5)(2)(10)#148 <= 45
 0 <= I_customer(5)(3)(1)#149 <= 45
 0 <= I_customer(5)(3)(2)#150 <= 45
 0 <= I_customer(5)(3)(3)#151 <= 45
 0 <= I_customer(5)(3)(4)#152 <= 45
 0 <= I_customer(5)(3)(5)#153 <= 45
 0 <= I_customer(5)(3)(6)#154 <= 45
 0 <= I_customer(5)(3)(7)#155 <= 45
 0 <= I_customer(5)(3)(8)#156 <= 45
 0 <= I_customer(5)(3)(9)#157 <= 45
 0 <= I_customer(5)(3)(10)#158 <= 45
 0 <= I_customer(6)(1)(1)#159 <= 14
 0 <= I_customer(6)(1)(2)#160 <= 14
 0 <= I_customer(6)(1)(3)#161 <= 14
 0 <= I_customer(6)(1)(4)#162 <= 14
 0 <= I_customer(6)(1)(5)#163 <= 14
 0 <= I_customer(6)(1)(6)#164 <= 14
 0 <= I_customer(6)(1)(7)#165 <= 14
 0 <= I_customer(6)(1)(8)#166 <= 14
 0 <= I_customer(6)(1)(9)#167 <= 14
 0 <= I_customer(6)(1)(10)#168 <= 14
 0 <= I_customer(6)(2)(1)#169 <= 14
 0 <= I_customer(6)(2)(2)#170 <= 14
 0 <= I_customer(6)(2)(3)#171 <= 14
 0 <= I_customer(6)(2)(4)#172 <= 14
 0 <= I_customer(6)(2)(5)#173 <= 14
 0 <= I_customer(6)(2)(6)#174 <= 14
 0 <= I_customer(6)(2)(7)#175 <= 14
 0 <= I_customer(6)(2)(8)#176 <= 14
 0 <= I_customer(6)(2)(9)#177 <= 14
 0 <= I_customer(6)(2)(10)#178 <= 14
 0 <= I_customer(6)(3)(1)#179 <= 14
 0 <= I_customer(6)(3)(2)#180 <= 14
 0 <= I_customer(6)(3)(3)#181 <= 14
 0 <= I_customer(6)(3)(4)#182 <= 14
 0 <= I_customer(6)(3)(5)#183 <= 14
 0 <= I_customer(6)(3)(6)#184 <= 14
 0 <= I_customer(6)(3)(7)#185 <= 14
 0 <= I_customer(6)(3)(8)#186 <= 14
 0 <= I_customer(6)(3)(9)#187 <= 14
 0 <= I_customer(6)(3)(10)#188 <= 14
 0 <= I_customer(7)(1)(1)#189 <= 39
 0 <= I_customer(7)(1)(2)#190 <= 39
 0 <= I_customer(7)(1)(3)#191 <= 39
 0 <= I_customer(7)(1)(4)#192 <= 39
 0 <= I_customer(7)(1)(5)#193 <= 39
 0 <= I_customer(7)(1)(6)#194 <= 39
 0 <= I_customer(7)(1)(7)#195 <= 39
 0 <= I_customer(7)(1)(8)#196 <= 39
 0 <= I_customer(7)(1)(9)#197 <= 39
 0 <= I_customer(7)(1)(10)#198 <= 39
 0 <= I_customer(7)(2)(1)#199 <= 39
 0 <= I_customer(7)(2)(2)#200 <= 39
 0 <= I_customer(7)(2)(3)#201 <= 39
 0 <= I_customer(7)(2)(4)#202 <= 39
 0 <= I_customer(7)(2)(5)#203 <= 39
 0 <= I_customer(7)(2)(6)#204 <= 39
 0 <= I_customer(7)(2)(7)#205 <= 39
 0 <= I_customer(7)(2)(8)#206 <= 39
 0 <= I_customer(7)(2)(9)#207 <= 39
 0 <= I_customer(7)(2)(10)#208 <= 39
 0 <= I_customer(7)(3)(1)#209 <= 39
 0 <= I_customer(7)(3)(2)#210 <= 39
 0 <= I_customer(7)(3)(3)#211 <= 39
 0 <= I_customer(7)(3)(4)#212 <= 39
 0 <= I_customer(7)(3)(5)#213 <= 39
 0 <= I_customer(7)(3)(6)#214 <= 39
 0 <= I_customer(7)(3)(7)#215 <= 39
 0 <= I_customer(7)(3)(8)#216 <= 39
 0 <= I_customer(7)(3)(9)#217 <= 39
 0 <= I_customer(7)(3)(10)#218 <= 39
 0 <= I_customer(8)(1)(1)#219 <= 96
 0 <= I_customer(8)(1)(2)#220 <= 96
 0 <= I_customer(8)(1)(3)#221 <= 96
 0 <= I_customer(8)(1)(4)#222 <= 96
 0 <= I_customer(8)(1)(5)#223 <= 96
 0 <= I_customer(8)(1)(6)#224 <= 96
 0 <= I_customer(8)(1)(7)#225 <= 96
 0 <= I_customer(8)(1)(8)#226 <= 96
 0 <= I_customer(8)(1)(9)#227 <= 96
 0 <= I_customer(8)(1)(10)#228 <= 96
 0 <= I_customer(8)(2)(1)#229 <= 96
 0 <= I_customer(8)(2)(2)#230 <= 96
 0 <= I_customer(8)(2)(3)#231 <= 96
 0 <= I_customer(8)(2)(4)#232 <= 96
 0 <= I_customer(8)(2)(5)#233 <= 96
 0 <= I_customer(8)(2)(6)#234 <= 96
 0 <= I_customer(8)(2)(7)#235 <= 96
 0 <= I_customer(8)(2)(8)#236 <= 96
 0 <= I_customer(8)(2)(9)#237 <= 96
 0 <= I_customer(8)(2)(10)#238 <= 96
 0 <= I_customer(8)(3)(1)#239 <= 96
 0 <= I_customer(8)(3)(2)#240 <= 96
 0 <= I_customer(8)(3)(3)#241 <= 96
 0 <= I_customer(8)(3)(4)#242 <= 96
 0 <= I_customer(8)(3)(5)#243 <= 96
 0 <= I_customer(8)(3)(6)#244 <= 96
 0 <= I_customer(8)(3)(7)#245 <= 96
 0 <= I_customer(8)(3)(8)#246 <= 96
 0 <= I_customer(8)(3)(9)#247 <= 96
 0 <= I_customer(8)(3)(10)#248 <= 96
 0 <= I_customer(9)(1)(1)#249 <= 132
 0 <= I_customer(9)(1)(2)#250 <= 132
 0 <= I_customer(9)(1)(3)#251 <= 132
 0 <= I_customer(9)(1)(4)#252 <= 132
 0 <= I_customer(9)(1)(5)#253 <= 132
 0 <= I_customer(9)(1)(6)#254 <= 132
 0 <= I_customer(9)(1)(7)#255 <= 132
 0 <= I_customer(9)(1)(8)#256 <= 132
 0 <= I_customer(9)(1)(9)#257 <= 132
 0 <= I_customer(9)(1)(10)#258 <= 132
 0 <= I_customer(9)(2)(1)#259 <= 132
 0 <= I_customer(9)(2)(2)#260 <= 132
 0 <= I_customer(9)(2)(3)#261 <= 132
 0 <= I_customer(9)(2)(4)#262 <= 132
 0 <= I_customer(9)(2)(5)#263 <= 132
 0 <= I_customer(9)(2)(6)#264 <= 132
 0 <= I_customer(9)(2)(7)#265 <= 132
 0 <= I_customer(9)(2)(8)#266 <= 132
 0 <= I_customer(9)(2)(9)#267 <= 132
 0 <= I_customer(9)(2)(10)#268 <= 132
 0 <= I_customer(9)(3)(1)#269 <= 132
 0 <= I_customer(9)(3)(2)#270 <= 132
 0 <= I_customer(9)(3)(3)#271 <= 132
 0 <= I_customer(9)(3)(4)#272 <= 132
 0 <= I_customer(9)(3)(5)#273 <= 132
 0 <= I_customer(9)(3)(6)#274 <= 132
 0 <= I_customer(9)(3)(7)#275 <= 132
 0 <= I_customer(9)(3)(8)#276 <= 132
 0 <= I_customer(9)(3)(9)#277 <= 132
 0 <= I_customer(9)(3)(10)#278 <= 132
 0 <= I_customer(10)(1)(1)#279 <= 26
 0 <= I_customer(10)(1)(2)#280 <= 26
 0 <= I_customer(10)(1)(3)#281 <= 26
 0 <= I_customer(10)(1)(4)#282 <= 26
 0 <= I_customer(10)(1)(5)#283 <= 26
 0 <= I_customer(10)(1)(6)#284 <= 26
 0 <= I_customer(10)(1)(7)#285 <= 26
 0 <= I_customer(10)(1)(8)#286 <= 26
 0 <= I_customer(10)(1)(9)#287 <= 26
 0 <= I_customer(10)(1)(10)#288 <= 26
 0 <= I_customer(10)(2)(1)#289 <= 26
 0 <= I_customer(10)(2)(2)#290 <= 26
 0 <= I_customer(10)(2)(3)#291 <= 26
 0 <= I_customer(10)(2)(4)#292 <= 26
 0 <= I_customer(10)(2)(5)#293 <= 26
 0 <= I_customer(10)(2)(6)#294 <= 26
 0 <= I_customer(10)(2)(7)#295 <= 26
 0 <= I_customer(10)(2)(8)#296 <= 26
 0 <= I_customer(10)(2)(9)#297 <= 26
 0 <= I_customer(10)(2)(10)#298 <= 26
 0 <= I_customer(10)(3)(1)#299 <= 26
 0 <= I_customer(10)(3)(2)#300 <= 26
 0 <= I_customer(10)(3)(3)#301 <= 26
 0 <= I_customer(10)(3)(4)#302 <= 26
 0 <= I_customer(10)(3)(5)#303 <= 26
 0 <= I_customer(10)(3)(6)#304 <= 26
 0 <= I_customer(10)(3)(7)#305 <= 26
 0 <= I_customer(10)(3)(8)#306 <= 26
 0 <= I_customer(10)(3)(9)#307 <= 26
 0 <= I_customer(10)(3)(10)#308 <= 26
 0 <= I_customer(11)(1)(1)#309 <= 114
 0 <= I_customer(11)(1)(2)#310 <= 114
 0 <= I_customer(11)(1)(3)#311 <= 114
 0 <= I_customer(11)(1)(4)#312 <= 114
 0 <= I_customer(11)(1)(5)#313 <= 114
 0 <= I_customer(11)(1)(6)#314 <= 114
 0 <= I_customer(11)(1)(7)#315 <= 114
 0 <= I_customer(11)(1)(8)#316 <= 114
 0 <= I_customer(11)(1)(9)#317 <= 114
 0 <= I_customer(11)(1)(10)#318 <= 114
 0 <= I_customer(11)(2)(1)#319 <= 114
 0 <= I_customer(11)(2)(2)#320 <= 114
 0 <= I_customer(11)(2)(3)#321 <= 114
 0 <= I_customer(11)(2)(4)#322 <= 114
 0 <= I_customer(11)(2)(5)#323 <= 114
 0 <= I_customer(11)(2)(6)#324 <= 114
 0 <= I_customer(11)(2)(7)#325 <= 114
 0 <= I_customer(11)(2)(8)#326 <= 114
 0 <= I_customer(11)(2)(9)#327 <= 114
 0 <= I_customer(11)(2)(10)#328 <= 114
 0 <= I_customer(11)(3)(1)#329 <= 114
 0 <= I_customer(11)(3)(2)#330 <= 114
 0 <= I_customer(11)(3)(3)#331 <= 114
 0 <= I_customer(11)(3)(4)#332 <= 114
 0 <= I_customer(11)(3)(5)#333 <= 114
 0 <= I_customer(11)(3)(6)#334 <= 114
 0 <= I_customer(11)(3)(7)#335 <= 114
 0 <= I_customer(11)(3)(8)#336 <= 114
 0 <= I_customer(11)(3)(9)#337 <= 114
 0 <= I_customer(11)(3)(10)#338 <= 114
 0 <= I_customer(12)(1)(1)#339 <= 110
 0 <= I_customer(12)(1)(2)#340 <= 110
 0 <= I_customer(12)(1)(3)#341 <= 110
 0 <= I_customer(12)(1)(4)#342 <= 110
 0 <= I_customer(12)(1)(5)#343 <= 110
 0 <= I_customer(12)(1)(6)#344 <= 110
 0 <= I_customer(12)(1)(7)#345 <= 110
 0 <= I_customer(12)(1)(8)#346 <= 110
 0 <= I_customer(12)(1)(9)#347 <= 110
 0 <= I_customer(12)(1)(10)#348 <= 110
 0 <= I_customer(12)(2)(1)#349 <= 110
 0 <= I_customer(12)(2)(2)#350 <= 110
 0 <= I_customer(12)(2)(3)#351 <= 110
 0 <= I_customer(12)(2)(4)#352 <= 110
 0 <= I_customer(12)(2)(5)#353 <= 110
 0 <= I_customer(12)(2)(6)#354 <= 110
 0 <= I_customer(12)(2)(7)#355 <= 110
 0 <= I_customer(12)(2)(8)#356 <= 110
 0 <= I_customer(12)(2)(9)#357 <= 110
 0 <= I_customer(12)(2)(10)#358 <= 110
 0 <= I_customer(12)(3)(1)#359 <= 110
 0 <= I_customer(12)(3)(2)#360 <= 110
 0 <= I_customer(12)(3)(3)#361 <= 110
 0 <= I_customer(12)(3)(4)#362 <= 110
 0 <= I_customer(12)(3)(5)#363 <= 110
 0 <= I_customer(12)(3)(6)#364 <= 110
 0 <= I_customer(12)(3)(7)#365 <= 110
 0 <= I_customer(12)(3)(8)#366 <= 110
 0 <= I_customer(12)(3)(9)#367 <= 110
 0 <= I_customer(12)(3)(10)#368 <= 110
 0 <= b_customer(3)(1)(1)#369 <= 10
 0 <= b_customer(3)(1)(2)#370 <= 12
 0 <= b_customer(3)(1)(3)#371 <= 11
 0 <= b_customer(3)(1)(4)#372 <= 11
 0 <= b_customer(3)(1)(5)#373 <= 9
 0 <= b_customer(3)(1)(6)#374 <= 12
 0 <= b_customer(3)(1)(7)#375 <= 12
 0 <= b_customer(3)(1)(8)#376 <= 10
 0 <= b_customer(3)(1)(9)#377 <= 12
 0 <= b_customer(3)(1)(10)#378 <= 11
 0 <= b_customer(3)(2)(1)#379 <= 9
 0 <= b_customer(3)(2)(2)#380 <= 11
 0 <= b_customer(3)(2)(3)#381 <= 11
 0 <= b_customer(3)(2)(4)#382 <= 11
 0 <= b_customer(3)(2)(5)#383 <= 12
 0 <= b_customer(3)(2)(6)#384 <= 10
 0 <= b_customer(3)(2)(7)#385 <= 11
 0 <= b_customer(3)(2)(8)#386 <= 10
 0 <= b_customer(3)(2)(9)#387 <= 10
 0 <= b_customer(3)(2)(10)#388 <= 10
 0 <= b_customer(3)(3)(1)#389 <= 12
 0 <= b_customer(3)(3)(2)#390 <= 9
 0 <= b_customer(3)(3)(3)#391 <= 11
 0 <= b_customer(3)(3)(4)#392 <= 11
 0 <= b_customer(3)(3)(5)#393 <= 9
 0 <= b_customer(3)(3)(6)#394 <= 9
 0 <= b_customer(3)(3)(7)#395 <= 10
 0 <= b_customer(3)(3)(8)#396 <= 8
 0 <= b_customer(3)(3)(9)#397 <= 8
 0 <= b_customer(3)(3)(10)#398 <= 11
 0 <= b_customer(4)(1)(1)#399 <= 18
 0 <= b_customer(4)(1)(2)#400 <= 12
 0 <= b_customer(4)(1)(3)#401 <= 17
 0 <= b_customer(4)(1)(4)#402 <= 18
 0 <= b_customer(4)(1)(5)#403 <= 17
 0 <= b_customer(4)(1)(6)#404 <= 13
 0 <= b_customer(4)(1)(7)#405 <= 13
 0 <= b_customer(4)(1)(8)#406 <= 13
 0 <= b_customer(4)(1)(9)#407 <= 18
 0 <= b_customer(4)(1)(10)#408 <= 14
 0 <= b_customer(4)(2)(1)#409 <= 12
 0 <= b_customer(4)(2)(2)#410 <= 16
 0 <= b_customer(4)(2)(3)#411 <= 17
 0 <= b_customer(4)(2)(4)#412 <= 15
 0 <= b_customer(4)(2)(5)#413 <= 17
 0 <= b_customer(4)(2)(6)#414 <= 13
 0 <= b_customer(4)(2)(7)#415 <= 17
 0 <= b_customer(4)(2)(8)#416 <= 13
 0 <= b_customer(4)(2)(9)#417 <= 15
 0 <= b_customer(4)(2)(10)#418 <= 14
 0 <= b_customer(4)(3)(1)#419 <= 17
 0 <= b_customer(4)(3)(2)#420 <= 14
 0 <= b_customer(4)(3)(3)#421 <= 15
 0 <= b_customer(4)(3)(4)#422 <= 13
 0 <= b_customer(4)(3)(5)#423 <= 14
 0 <= b_customer(4)(3)(6)#424 <= 16
 0 <= b_customer(4)(3)(7)#425 <= 15
 0 <= b_customer(4)(3)(8)#426 <= 16
 0 <= b_customer(4)(3)(9)#427 <= 18
 0 <= b_customer(4)(3)(10)#428 <= 14
 0 <= b_customer(5)(1)(1)#429 <= 13
 0 <= b_customer(5)(1)(2)#430 <= 12
 0 <= b_customer(5)(1)(3)#431 <= 15
 0 <= b_customer(5)(1)(4)#432 <= 17
 0 <= b_customer(5)(1)(5)#433 <= 17
 0 <= b_customer(5)(1)(6)#434 <= 14
 0 <= b_customer(5)(1)(7)#435 <= 15
 0 <= b_customer(5)(1)(8)#436 <= 13
 0 <= b_customer(5)(1)(9)#437 <= 13
 0 <= b_customer(5)(1)(10)#438 <= 15
 0 <= b_customer(5)(2)(1)#439 <= 15
 0 <= b_customer(5)(2)(2)#440 <= 17
 0 <= b_customer(5)(2)(3)#441 <= 18
 0 <= b_customer(5)(2)(4)#442 <= 14
 0 <= b_customer(5)(2)(5)#443 <= 12
 0 <= b_customer(5)(2)(6)#444 <= 17
 0 <= b_customer(5)(2)(7)#445 <= 13
 0 <= b_customer(5)(2)(8)#446 <= 13
 0 <= b_customer(5)(2)(9)#447 <= 17
 0 <= b_customer(5)(2)(10)#448 <= 13
 0 <= b_customer(5)(3)(1)#449 <= 16
 0 <= b_customer(5)(3)(2)#450 <= 17
 0 <= b_customer(5)(3)(3)#451 <= 13
 0 <= b_customer(5)(3)(4)#452 <= 16
 0 <= b_customer(5)(3)(5)#453 <= 14
 0 <= b_customer(5)(3)(6)#454 <= 13
 0 <= b_customer(5)(3)(7)#455 <= 12
 0 <= b_customer(5)(3)(8)#456 <= 17
 0 <= b_customer(5)(3)(9)#457 <= 13
 0 <= b_customer(5)(3)(10)#458 <= 16
 0 <= b_customer(6)(1)(1)#459 <= 6
 0 <= b_customer(6)(1)(2)#460 <= 8
 0 <= b_customer(6)(1)(3)#461 <= 7
 0 <= b_customer(6)(1)(4)#462 <= 7
 0 <= b_customer(6)(1)(5)#463 <= 8
 0 <= b_customer(6)(1)(6)#464 <= 8
 0 <= b_customer(6)(1)(7)#465 <= 8
 0 <= b_customer(6)(1)(8)#466 <= 7
 0 <= b_customer(6)(1)(9)#467 <= 7
 0 <= b_customer(6)(1)(10)#468 <= 7
 0 <= b_customer(6)(2)(1)#469 <= 7
 0 <= b_customer(6)(2)(2)#470 <= 7
 0 <= b_customer(6)(2)(3)#471 <= 6
 0 <= b_customer(6)(2)(4)#472 <= 6
 0 <= b_customer(6)(2)(5)#473 <= 7
 0 <= b_customer(6)(2)(6)#474 <= 6
 0 <= b_customer(6)(2)(7)#475 <= 6
 0 <= b_customer(6)(2)(8)#476 <= 8
 0 <= b_customer(6)(2)(9)#477 <= 7
 0 <= b_customer(6)(2)(10)#478 <= 8
 0 <= b_customer(6)(3)(1)#479 <= 7
 0 <= b_customer(6)(3)(2)#480 <= 8
 0 <= b_customer(6)(3)(3)#481 <= 7
 0 <= b_customer(6)(3)(4)#482 <= 6
 0 <= b_customer(6)(3)(5)#483 <= 8
 0 <= b_customer(6)(3)(6)#484 <= 6
 0 <= b_customer(6)(3)(7)#485 <= 7
 0 <= b_customer(6)(3)(8)#486 <= 7
 0 <= b_customer(6)(3)(9)#487 <= 6
 0 <= b_customer(6)(3)(10)#488 <= 8
 0 <= b_customer(7)(1)(1)#489 <= 15
 0 <= b_customer(7)(1)(2)#490 <= 11
 0 <= b_customer(7)(1)(3)#491 <= 14
 0 <= b_customer(7)(1)(4)#492 <= 14
 0 <= b_customer(7)(1)(5)#493 <= 14
 0 <= b_customer(7)(1)(6)#494 <= 14
 0 <= b_customer(7)(1)(7)#495 <= 12
 0 <= b_customer(7)(1)(8)#496 <= 15
 0 <= b_customer(7)(1)(9)#497 <= 14
 0 <= b_customer(7)(1)(10)#498 <= 15
 0 <= b_customer(7)(2)(1)#499 <= 15
 0 <= b_customer(7)(2)(2)#500 <= 13
 0 <= b_customer(7)(2)(3)#501 <= 13
 0 <= b_customer(7)(2)(4)#502 <= 11
 0 <= b_customer(7)(2)(5)#503 <= 12
 0 <= b_customer(7)(2)(6)#504 <= 11
 0 <= b_customer(7)(2)(7)#505 <= 13
 0 <= b_customer(7)(2)(8)#506 <= 13
 0 <= b_customer(7)(2)(9)#507 <= 13
 0 <= b_customer(7)(2)(10)#508 <= 13
 0 <= b_customer(7)(3)(1)#509 <= 12
 0 <= b_customer(7)(3)(2)#510 <= 11
 0 <= b_customer(7)(3)(3)#511 <= 13
 0 <= b_customer(7)(3)(4)#512 <= 11
 0 <= b_customer(7)(3)(5)#513 <= 12
 0 <= b_customer(7)(3)(6)#514 <= 14
 0 <= b_customer(7)(3)(7)#515 <= 14
 0 <= b_customer(7)(3)(8)#516 <= 13
 0 <= b_customer(7)(3)(9)#517 <= 13
 0 <= b_customer(7)(3)(10)#518 <= 12
 0 <= b_customer(8)(1)(1)#519 <= 13
 0 <= b_customer(8)(1)(2)#520 <= 16
 0 <= b_customer(8)(1)(3)#521 <= 18
 0 <= b_customer(8)(1)(4)#522 <= 14
 0 <= b_customer(8)(1)(5)#523 <= 19
 0 <= b_customer(8)(1)(6)#524 <= 15
 0 <= b_customer(8)(1)(7)#525 <= 18
 0 <= b_customer(8)(1)(8)#526 <= 13
 0 <= b_customer(8)(1)(9)#527 <= 17
 0 <= b_customer(8)(1)(10)#528 <= 16
 0 <= b_customer(8)(2)(1)#529 <= 18
 0 <= b_customer(8)(2)(2)#530 <= 15
 0 <= b_customer(8)(2)(3)#531 <= 17
 0 <= b_customer(8)(2)(4)#532 <= 15
 0 <= b_customer(8)(2)(5)#533 <= 14
 0 <= b_customer(8)(2)(6)#534 <= 19
 0 <= b_customer(8)(2)(7)#535 <= 18
 0 <= b_customer(8)(2)(8)#536 <= 17
 0 <= b_customer(8)(2)(9)#537 <= 15
 0 <= b_customer(8)(2)(10)#538 <= 15
 0 <= b_customer(8)(3)(1)#539 <= 17
 0 <= b_customer(8)(3)(2)#540 <= 17
 0 <= b_customer(8)(3)(3)#541 <= 19
 0 <= b_customer(8)(3)(4)#542 <= 15
 0 <= b_customer(8)(3)(5)#543 <= 14
 0 <= b_customer(8)(3)(6)#544 <= 15
 0 <= b_customer(8)(3)(7)#545 <= 14
 0 <= b_customer(8)(3)(8)#546 <= 19
 0 <= b_customer(8)(3)(9)#547 <= 13
 0 <= b_customer(8)(3)(10)#548 <= 15
 0 <= b_customer(9)(1)(1)#549 <= 26
 0 <= b_customer(9)(1)(2)#550 <= 18
 0 <= b_customer(9)(1)(3)#551 <= 19
 0 <= b_customer(9)(1)(4)#552 <= 22
 0 <= b_customer(9)(1)(5)#553 <= 26
 0 <= b_customer(9)(1)(6)#554 <= 26
 0 <= b_customer(9)(1)(7)#555 <= 18
 0 <= b_customer(9)(1)(8)#556 <= 21
 0 <= b_customer(9)(1)(9)#557 <= 24
 0 <= b_customer(9)(1)(10)#558 <= 20
 0 <= b_customer(9)(2)(1)#559 <= 20
 0 <= b_customer(9)(2)(2)#560 <= 20
 0 <= b_customer(9)(2)(3)#561 <= 22
 0 <= b_customer(9)(2)(4)#562 <= 24
 0 <= b_customer(9)(2)(5)#563 <= 26
 0 <= b_customer(9)(2)(6)#564 <= 19
 0 <= b_customer(9)(2)(7)#565 <= 21
 0 <= b_customer(9)(2)(8)#566 <= 20
 0 <= b_customer(9)(2)(9)#567 <= 25
 0 <= b_customer(9)(2)(10)#568 <= 25
 0 <= b_customer(9)(3)(1)#569 <= 24
 0 <= b_customer(9)(3)(2)#570 <= 18
 0 <= b_customer(9)(3)(3)#571 <= 26
 0 <= b_customer(9)(3)(4)#572 <= 23
 0 <= b_customer(9)(3)(5)#573 <= 24
 0 <= b_customer(9)(3)(6)#574 <= 26
 0 <= b_customer(9)(3)(7)#575 <= 26
 0 <= b_customer(9)(3)(8)#576 <= 25
 0 <= b_customer(9)(3)(9)#577 <= 24
 0 <= b_customer(9)(3)(10)#578 <= 25
 0 <= b_customer(10)(1)(1)#579 <= 12
 0 <= b_customer(10)(1)(2)#580 <= 11
 0 <= b_customer(10)(1)(3)#581 <= 14
 0 <= b_customer(10)(1)(4)#582 <= 15
 0 <= b_customer(10)(1)(5)#583 <= 11
 0 <= b_customer(10)(1)(6)#584 <= 13
 0 <= b_customer(10)(1)(7)#585 <= 13
 0 <= b_customer(10)(1)(8)#586 <= 10
 0 <= b_customer(10)(1)(9)#587 <= 11
 0 <= b_customer(10)(1)(10)#588 <= 11
 0 <= b_customer(10)(2)(1)#589 <= 11
 0 <= b_customer(10)(2)(2)#590 <= 15
 0 <= b_customer(10)(2)(3)#591 <= 11
 0 <= b_customer(10)(2)(4)#592 <= 16
 0 <= b_customer(10)(2)(5)#593 <= 13
 0 <= b_customer(10)(2)(6)#594 <= 14
 0 <= b_customer(10)(2)(7)#595 <= 14
 0 <= b_customer(10)(2)(8)#596 <= 11
 0 <= b_customer(10)(2)(9)#597 <= 11
 0 <= b_customer(10)(2)(10)#598 <= 13
 0 <= b_customer(10)(3)(1)#599 <= 13
 0 <= b_customer(10)(3)(2)#600 <= 14
 0 <= b_customer(10)(3)(3)#601 <= 15
 0 <= b_customer(10)(3)(4)#602 <= 15
 0 <= b_customer(10)(3)(5)#603 <= 11
 0 <= b_customer(10)(3)(6)#604 <= 14
 0 <= b_customer(10)(3)(7)#605 <= 15
 0 <= b_customer(10)(3)(8)#606 <= 14
 0 <= b_customer(10)(3)(9)#607 <= 15
 0 <= b_customer(10)(3)(10)#608 <= 13
 0 <= b_customer(11)(1)(1)#609 <= 15
 0 <= b_customer(11)(1)(2)#610 <= 22
 0 <= b_customer(11)(1)(3)#611 <= 18
 0 <= b_customer(11)(1)(4)#612 <= 18
 0 <= b_customer(11)(1)(5)#613 <= 22
 0 <= b_customer(11)(1)(6)#614 <= 17
 0 <= b_customer(11)(1)(7)#615 <= 20
 0 <= b_customer(11)(1)(8)#616 <= 22
 0 <= b_customer(11)(1)(9)#617 <= 21
 0 <= b_customer(11)(1)(10)#618 <= 19
 0 <= b_customer(11)(2)(1)#619 <= 16
 0 <= b_customer(11)(2)(2)#620 <= 18
 0 <= b_customer(11)(2)(3)#621 <= 22
 0 <= b_customer(11)(2)(4)#622 <= 22
 0 <= b_customer(11)(2)(5)#623 <= 21
 0 <= b_customer(11)(2)(6)#624 <= 16
 0 <= b_customer(11)(2)(7)#625 <= 20
 0 <= b_customer(11)(2)(8)#626 <= 17
 0 <= b_customer(11)(2)(9)#627 <= 15
 0 <= b_customer(11)(2)(10)#628 <= 18
 0 <= b_customer(11)(3)(1)#629 <= 16
 0 <= b_customer(11)(3)(2)#630 <= 22
 0 <= b_customer(11)(3)(3)#631 <= 20
 0 <= b_customer(11)(3)(4)#632 <= 16
 0 <= b_customer(11)(3)(5)#633 <= 16
 0 <= b_customer(11)(3)(6)#634 <= 16
 0 <= b_customer(11)(3)(7)#635 <= 21
 0 <= b_customer(11)(3)(8)#636 <= 19
 0 <= b_customer(11)(3)(9)#637 <= 19
 0 <= b_customer(11)(3)(10)#638 <= 17
 0 <= b_customer(12)(1)(1)#639 <= 21
 0 <= b_customer(12)(1)(2)#640 <= 18
 0 <= b_customer(12)(1)(3)#641 <= 21
 0 <= b_customer(12)(1)(4)#642 <= 24
 0 <= b_customer(12)(1)(5)#643 <= 22
 0 <= b_customer(12)(1)(6)#644 <= 20
 0 <= b_customer(12)(1)(7)#645 <= 20
 0 <= b_customer(12)(1)(8)#646 <= 19
 0 <= b_customer(12)(1)(9)#647 <= 18
 0 <= b_customer(12)(1)(10)#648 <= 18
 0 <= b_customer(12)(2)(1)#649 <= 21
 0 <= b_customer(12)(2)(2)#650 <= 22
 0 <= b_customer(12)(2)(3)#651 <= 21
 0 <= b_customer(12)(2)(4)#652 <= 23
 0 <= b_customer(12)(2)(5)#653 <= 19
 0 <= b_customer(12)(2)(6)#654 <= 19
 0 <= b_customer(12)(2)(7)#655 <= 23
 0 <= b_customer(12)(2)(8)#656 <= 26
 0 <= b_customer(12)(2)(9)#657 <= 25
 0 <= b_customer(12)(2)(10)#658 <= 18
 0 <= b_customer(12)(3)(1)#659 <= 25
 0 <= b_customer(12)(3)(2)#660 <= 25
 0 <= b_customer(12)(3)(3)#661 <= 21
 0 <= b_customer(12)(3)(4)#662 <= 19
 0 <= b_customer(12)(3)(5)#663 <= 19
 0 <= b_customer(12)(3)(6)#664 <= 26
 0 <= b_customer(12)(3)(7)#665 <= 25
 0 <= b_customer(12)(3)(8)#666 <= 22
 0 <= b_customer(12)(3)(9)#667 <= 21
 0 <= b_customer(12)(3)(10)#668 <= 25
 0 <= q(1)(1)(1)#669 <= 136.8
 0 <= q(1)(1)(2)#670 <= 136.8
 0 <= q(1)(1)(3)#671 <= 136.8
      q(1)(2)(1)#672 = 0
      q(1)(2)(2)#673 = 0
      q(1)(2)(3)#674 = 0
      q(2)(1)(1)#675 = 0
      q(2)(1)(2)#676 = 0
      q(2)(1)(3)#677 = 0
 0 <= q(2)(2)(1)#678 <= 136.8
 0 <= q(2)(2)(2)#679 <= 136.8
 0 <= q(2)(2)(3)#680 <= 136.8
 0 <= q(3)(1)(1)#681 <= 136.8
 0 <= q(3)(1)(2)#682 <= 136.8
 0 <= q(3)(1)(3)#683 <= 136.8
 0 <= q(3)(2)(1)#684 <= 136.8
 0 <= q(3)(2)(2)#685 <= 136.8
 0 <= q(3)(2)(3)#686 <= 136.8
 0 <= o(1)(1)#687 <= 1
 0 <= o(1)(2)#688 <= 1
 0 <= o(1)(3)#689 <= 1
 0 <= o(2)(1)#690 <= 1
 0 <= o(2)(2)#691 <= 1
 0 <= o(2)(3)#692 <= 1
 0 <= o(3)(1)#693 <= 1
 0 <= o(3)(2)#694 <= 1
 0 <= o(3)(3)#695 <= 1
 0 <= w(3)(1)(1)(1)#696 <= 10
 0 <= w(3)(1)(1)(2)#697 <= 12
 0 <= w(3)(1)(1)(3)#698 <= 11
 0 <= w(3)(1)(1)(4)#699 <= 11
 0 <= w(3)(1)(1)(5)#700 <= 9
 0 <= w(3)(1)(1)(6)#701 <= 12
 0 <= w(3)(1)(1)(7)#702 <= 12
 0 <= w(3)(1)(1)(8)#703 <= 10
 0 <= w(3)(1)(1)(9)#704 <= 12
 0 <= w(3)(1)(1)(10)#705 <= 11
 0 <= w(3)(1)(2)(1)#706 <= 9
 0 <= w(3)(1)(2)(2)#707 <= 11
 0 <= w(3)(1)(2)(3)#708 <= 11
 0 <= w(3)(1)(2)(4)#709 <= 11
 0 <= w(3)(1)(2)(5)#710 <= 12
 0 <= w(3)(1)(2)(6)#711 <= 10
 0 <= w(3)(1)(2)(7)#712 <= 11
 0 <= w(3)(1)(2)(8)#713 <= 10
 0 <= w(3)(1)(2)(9)#714 <= 10
 0 <= w(3)(1)(2)(10)#715 <= 10
 0 <= w(3)(1)(3)(1)#716 <= 12
 0 <= w(3)(1)(3)(2)#717 <= 9
 0 <= w(3)(1)(3)(3)#718 <= 11
 0 <= w(3)(1)(3)(4)#719 <= 11
 0 <= w(3)(1)(3)(5)#720 <= 9
 0 <= w(3)(1)(3)(6)#721 <= 9
 0 <= w(3)(1)(3)(7)#722 <= 10
 0 <= w(3)(1)(3)(8)#723 <= 8
 0 <= w(3)(1)(3)(9)#724 <= 8
 0 <= w(3)(1)(3)(10)#725 <= 11
 0 <= w(3)(2)(1)(1)#726 <= 10
 0 <= w(3)(2)(1)(2)#727 <= 12
 0 <= w(3)(2)(1)(3)#728 <= 11
 0 <= w(3)(2)(1)(4)#729 <= 11
 0 <= w(3)(2)(1)(5)#730 <= 9
 0 <= w(3)(2)(1)(6)#731 <= 12
 0 <= w(3)(2)(1)(7)#732 <= 12
 0 <= w(3)(2)(1)(8)#733 <= 10
 0 <= w(3)(2)(1)(9)#734 <= 12
 0 <= w(3)(2)(1)(10)#735 <= 11
 0 <= w(3)(2)(2)(1)#736 <= 9
 0 <= w(3)(2)(2)(2)#737 <= 11
 0 <= w(3)(2)(2)(3)#738 <= 11
 0 <= w(3)(2)(2)(4)#739 <= 11
 0 <= w(3)(2)(2)(5)#740 <= 12
 0 <= w(3)(2)(2)(6)#741 <= 10
 0 <= w(3)(2)(2)(7)#742 <= 11
 0 <= w(3)(2)(2)(8)#743 <= 10
 0 <= w(3)(2)(2)(9)#744 <= 10
 0 <= w(3)(2)(2)(10)#745 <= 10
 0 <= w(3)(2)(3)(1)#746 <= 12
 0 <= w(3)(2)(3)(2)#747 <= 9
 0 <= w(3)(2)(3)(3)#748 <= 11
 0 <= w(3)(2)(3)(4)#749 <= 11
 0 <= w(3)(2)(3)(5)#750 <= 9
 0 <= w(3)(2)(3)(6)#751 <= 9
 0 <= w(3)(2)(3)(7)#752 <= 10
 0 <= w(3)(2)(3)(8)#753 <= 8
 0 <= w(3)(2)(3)(9)#754 <= 8
 0 <= w(3)(2)(3)(10)#755 <= 11
 0 <= w(3)(3)(1)(1)#756 <= 10
 0 <= w(3)(3)(1)(2)#757 <= 12
 0 <= w(3)(3)(1)(3)#758 <= 11
 0 <= w(3)(3)(1)(4)#759 <= 11
 0 <= w(3)(3)(1)(5)#760 <= 9
 0 <= w(3)(3)(1)(6)#761 <= 12
 0 <= w(3)(3)(1)(7)#762 <= 12
 0 <= w(3)(3)(1)(8)#763 <= 10
 0 <= w(3)(3)(1)(9)#764 <= 12
 0 <= w(3)(3)(1)(10)#765 <= 11
 0 <= w(3)(3)(2)(1)#766 <= 9
 0 <= w(3)(3)(2)(2)#767 <= 11
 0 <= w(3)(3)(2)(3)#768 <= 11
 0 <= w(3)(3)(2)(4)#769 <= 11
 0 <= w(3)(3)(2)(5)#770 <= 12
 0 <= w(3)(3)(2)(6)#771 <= 10
 0 <= w(3)(3)(2)(7)#772 <= 11
 0 <= w(3)(3)(2)(8)#773 <= 10
 0 <= w(3)(3)(2)(9)#774 <= 10
 0 <= w(3)(3)(2)(10)#775 <= 10
 0 <= w(3)(3)(3)(1)#776 <= 12
 0 <= w(3)(3)(3)(2)#777 <= 9
 0 <= w(3)(3)(3)(3)#778 <= 11
 0 <= w(3)(3)(3)(4)#779 <= 11
 0 <= w(3)(3)(3)(5)#780 <= 9
 0 <= w(3)(3)(3)(6)#781 <= 9
 0 <= w(3)(3)(3)(7)#782 <= 10
 0 <= w(3)(3)(3)(8)#783 <= 8
 0 <= w(3)(3)(3)(9)#784 <= 8
 0 <= w(3)(3)(3)(10)#785 <= 11
 0 <= w(3)(4)(1)(1)#786 <= 10
 0 <= w(3)(4)(1)(2)#787 <= 12
 0 <= w(3)(4)(1)(3)#788 <= 11
 0 <= w(3)(4)(1)(4)#789 <= 11
 0 <= w(3)(4)(1)(5)#790 <= 9
 0 <= w(3)(4)(1)(6)#791 <= 12
 0 <= w(3)(4)(1)(7)#792 <= 12
 0 <= w(3)(4)(1)(8)#793 <= 10
 0 <= w(3)(4)(1)(9)#794 <= 12
 0 <= w(3)(4)(1)(10)#795 <= 11
 0 <= w(3)(4)(2)(1)#796 <= 9
 0 <= w(3)(4)(2)(2)#797 <= 11
 0 <= w(3)(4)(2)(3)#798 <= 11
 0 <= w(3)(4)(2)(4)#799 <= 11
 0 <= w(3)(4)(2)(5)#800 <= 12
 0 <= w(3)(4)(2)(6)#801 <= 10
 0 <= w(3)(4)(2)(7)#802 <= 11
 0 <= w(3)(4)(2)(8)#803 <= 10
 0 <= w(3)(4)(2)(9)#804 <= 10
 0 <= w(3)(4)(2)(10)#805 <= 10
 0 <= w(3)(4)(3)(1)#806 <= 12
 0 <= w(3)(4)(3)(2)#807 <= 9
 0 <= w(3)(4)(3)(3)#808 <= 11
 0 <= w(3)(4)(3)(4)#809 <= 11
 0 <= w(3)(4)(3)(5)#810 <= 9
 0 <= w(3)(4)(3)(6)#811 <= 9
 0 <= w(3)(4)(3)(7)#812 <= 10
 0 <= w(3)(4)(3)(8)#813 <= 8
 0 <= w(3)(4)(3)(9)#814 <= 8
 0 <= w(3)(4)(3)(10)#815 <= 11
 0 <= w(3)(5)(1)(1)#816 <= 10
 0 <= w(3)(5)(1)(2)#817 <= 12
 0 <= w(3)(5)(1)(3)#818 <= 11
 0 <= w(3)(5)(1)(4)#819 <= 11
 0 <= w(3)(5)(1)(5)#820 <= 9
 0 <= w(3)(5)(1)(6)#821 <= 12
 0 <= w(3)(5)(1)(7)#822 <= 12
 0 <= w(3)(5)(1)(8)#823 <= 10
 0 <= w(3)(5)(1)(9)#824 <= 12
 0 <= w(3)(5)(1)(10)#825 <= 11
 0 <= w(3)(5)(2)(1)#826 <= 9
 0 <= w(3)(5)(2)(2)#827 <= 11
 0 <= w(3)(5)(2)(3)#828 <= 11
 0 <= w(3)(5)(2)(4)#829 <= 11
 0 <= w(3)(5)(2)(5)#830 <= 12
 0 <= w(3)(5)(2)(6)#831 <= 10
 0 <= w(3)(5)(2)(7)#832 <= 11
 0 <= w(3)(5)(2)(8)#833 <= 10
 0 <= w(3)(5)(2)(9)#834 <= 10
 0 <= w(3)(5)(2)(10)#835 <= 10
 0 <= w(3)(5)(3)(1)#836 <= 12
 0 <= w(3)(5)(3)(2)#837 <= 9
 0 <= w(3)(5)(3)(3)#838 <= 11
 0 <= w(3)(5)(3)(4)#839 <= 11
 0 <= w(3)(5)(3)(5)#840 <= 9
 0 <= w(3)(5)(3)(6)#841 <= 9
 0 <= w(3)(5)(3)(7)#842 <= 10
 0 <= w(3)(5)(3)(8)#843 <= 8
 0 <= w(3)(5)(3)(9)#844 <= 8
 0 <= w(3)(5)(3)(10)#845 <= 11
 0 <= w(3)(6)(1)(1)#846 <= 10
 0 <= w(3)(6)(1)(2)#847 <= 12
 0 <= w(3)(6)(1)(3)#848 <= 11
 0 <= w(3)(6)(1)(4)#849 <= 11
 0 <= w(3)(6)(1)(5)#850 <= 9
 0 <= w(3)(6)(1)(6)#851 <= 12
 0 <= w(3)(6)(1)(7)#852 <= 12
 0 <= w(3)(6)(1)(8)#853 <= 10
 0 <= w(3)(6)(1)(9)#854 <= 12
 0 <= w(3)(6)(1)(10)#855 <= 11
 0 <= w(3)(6)(2)(1)#856 <= 9
 0 <= w(3)(6)(2)(2)#857 <= 11
 0 <= w(3)(6)(2)(3)#858 <= 11
 0 <= w(3)(6)(2)(4)#859 <= 11
 0 <= w(3)(6)(2)(5)#860 <= 12
 0 <= w(3)(6)(2)(6)#861 <= 10
 0 <= w(3)(6)(2)(7)#862 <= 11
 0 <= w(3)(6)(2)(8)#863 <= 10
 0 <= w(3)(6)(2)(9)#864 <= 10
 0 <= w(3)(6)(2)(10)#865 <= 10
 0 <= w(3)(6)(3)(1)#866 <= 12
 0 <= w(3)(6)(3)(2)#867 <= 9
 0 <= w(3)(6)(3)(3)#868 <= 11
 0 <= w(3)(6)(3)(4)#869 <= 11
 0 <= w(3)(6)(3)(5)#870 <= 9
 0 <= w(3)(6)(3)(6)#871 <= 9
 0 <= w(3)(6)(3)(7)#872 <= 10
 0 <= w(3)(6)(3)(8)#873 <= 8
 0 <= w(3)(6)(3)(9)#874 <= 8
 0 <= w(3)(6)(3)(10)#875 <= 11
 0 <= w(4)(1)(1)(1)#876 <= 18
 0 <= w(4)(1)(1)(2)#877 <= 12
 0 <= w(4)(1)(1)(3)#878 <= 17
 0 <= w(4)(1)(1)(4)#879 <= 18
 0 <= w(4)(1)(1)(5)#880 <= 17
 0 <= w(4)(1)(1)(6)#881 <= 13
 0 <= w(4)(1)(1)(7)#882 <= 13
 0 <= w(4)(1)(1)(8)#883 <= 13
 0 <= w(4)(1)(1)(9)#884 <= 18
 0 <= w(4)(1)(1)(10)#885 <= 14
 0 <= w(4)(1)(2)(1)#886 <= 12
 0 <= w(4)(1)(2)(2)#887 <= 16
 0 <= w(4)(1)(2)(3)#888 <= 17
 0 <= w(4)(1)(2)(4)#889 <= 15
 0 <= w(4)(1)(2)(5)#890 <= 17
 0 <= w(4)(1)(2)(6)#891 <= 13
 0 <= w(4)(1)(2)(7)#892 <= 17
 0 <= w(4)(1)(2)(8)#893 <= 13
 0 <= w(4)(1)(2)(9)#894 <= 15
 0 <= w(4)(1)(2)(10)#895 <= 14
 0 <= w(4)(1)(3)(1)#896 <= 17
 0 <= w(4)(1)(3)(2)#897 <= 14
 0 <= w(4)(1)(3)(3)#898 <= 15
 0 <= w(4)(1)(3)(4)#899 <= 13
 0 <= w(4)(1)(3)(5)#900 <= 14
 0 <= w(4)(1)(3)(6)#901 <= 16
 0 <= w(4)(1)(3)(7)#902 <= 15
 0 <= w(4)(1)(3)(8)#903 <= 16
 0 <= w(4)(1)(3)(9)#904 <= 18
 0 <= w(4)(1)(3)(10)#905 <= 14
 0 <= w(4)(2)(1)(1)#906 <= 18
 0 <= w(4)(2)(1)(2)#907 <= 12
 0 <= w(4)(2)(1)(3)#908 <= 17
 0 <= w(4)(2)(1)(4)#909 <= 18
 0 <= w(4)(2)(1)(5)#910 <= 17
 0 <= w(4)(2)(1)(6)#911 <= 13
 0 <= w(4)(2)(1)(7)#912 <= 13
 0 <= w(4)(2)(1)(8)#913 <= 13
 0 <= w(4)(2)(1)(9)#914 <= 18
 0 <= w(4)(2)(1)(10)#915 <= 14
 0 <= w(4)(2)(2)(1)#916 <= 12
 0 <= w(4)(2)(2)(2)#917 <= 16
 0 <= w(4)(2)(2)(3)#918 <= 17
 0 <= w(4)(2)(2)(4)#919 <= 15
 0 <= w(4)(2)(2)(5)#920 <= 17
 0 <= w(4)(2)(2)(6)#921 <= 13
 0 <= w(4)(2)(2)(7)#922 <= 17
 0 <= w(4)(2)(2)(8)#923 <= 13
 0 <= w(4)(2)(2)(9)#924 <= 15
 0 <= w(4)(2)(2)(10)#925 <= 14
 0 <= w(4)(2)(3)(1)#926 <= 17
 0 <= w(4)(2)(3)(2)#927 <= 14
 0 <= w(4)(2)(3)(3)#928 <= 15
 0 <= w(4)(2)(3)(4)#929 <= 13
 0 <= w(4)(2)(3)(5)#930 <= 14
 0 <= w(4)(2)(3)(6)#931 <= 16
 0 <= w(4)(2)(3)(7)#932 <= 15
 0 <= w(4)(2)(3)(8)#933 <= 16
 0 <= w(4)(2)(3)(9)#934 <= 18
 0 <= w(4)(2)(3)(10)#935 <= 14
 0 <= w(4)(3)(1)(1)#936 <= 18
 0 <= w(4)(3)(1)(2)#937 <= 12
 0 <= w(4)(3)(1)(3)#938 <= 17
 0 <= w(4)(3)(1)(4)#939 <= 18
 0 <= w(4)(3)(1)(5)#940 <= 17
 0 <= w(4)(3)(1)(6)#941 <= 13
 0 <= w(4)(3)(1)(7)#942 <= 13
 0 <= w(4)(3)(1)(8)#943 <= 13
 0 <= w(4)(3)(1)(9)#944 <= 18
 0 <= w(4)(3)(1)(10)#945 <= 14
 0 <= w(4)(3)(2)(1)#946 <= 12
 0 <= w(4)(3)(2)(2)#947 <= 16
 0 <= w(4)(3)(2)(3)#948 <= 17
 0 <= w(4)(3)(2)(4)#949 <= 15
 0 <= w(4)(3)(2)(5)#950 <= 17
 0 <= w(4)(3)(2)(6)#951 <= 13
 0 <= w(4)(3)(2)(7)#952 <= 17
 0 <= w(4)(3)(2)(8)#953 <= 13
 0 <= w(4)(3)(2)(9)#954 <= 15
 0 <= w(4)(3)(2)(10)#955 <= 14
 0 <= w(4)(3)(3)(1)#956 <= 17
 0 <= w(4)(3)(3)(2)#957 <= 14
 0 <= w(4)(3)(3)(3)#958 <= 15
 0 <= w(4)(3)(3)(4)#959 <= 13
 0 <= w(4)(3)(3)(5)#960 <= 14
 0 <= w(4)(3)(3)(6)#961 <= 16
 0 <= w(4)(3)(3)(7)#962 <= 15
 0 <= w(4)(3)(3)(8)#963 <= 16
 0 <= w(4)(3)(3)(9)#964 <= 18
 0 <= w(4)(3)(3)(10)#965 <= 14
 0 <= w(4)(4)(1)(1)#966 <= 18
 0 <= w(4)(4)(1)(2)#967 <= 12
 0 <= w(4)(4)(1)(3)#968 <= 17
 0 <= w(4)(4)(1)(4)#969 <= 18
 0 <= w(4)(4)(1)(5)#970 <= 17
 0 <= w(4)(4)(1)(6)#971 <= 13
 0 <= w(4)(4)(1)(7)#972 <= 13
 0 <= w(4)(4)(1)(8)#973 <= 13
 0 <= w(4)(4)(1)(9)#974 <= 18
 0 <= w(4)(4)(1)(10)#975 <= 14
 0 <= w(4)(4)(2)(1)#976 <= 12
 0 <= w(4)(4)(2)(2)#977 <= 16
 0 <= w(4)(4)(2)(3)#978 <= 17
 0 <= w(4)(4)(2)(4)#979 <= 15
 0 <= w(4)(4)(2)(5)#980 <= 17
 0 <= w(4)(4)(2)(6)#981 <= 13
 0 <= w(4)(4)(2)(7)#982 <= 17
 0 <= w(4)(4)(2)(8)#983 <= 13
 0 <= w(4)(4)(2)(9)#984 <= 15
 0 <= w(4)(4)(2)(10)#985 <= 14
 0 <= w(4)(4)(3)(1)#986 <= 17
 0 <= w(4)(4)(3)(2)#987 <= 14
 0 <= w(4)(4)(3)(3)#988 <= 15
 0 <= w(4)(4)(3)(4)#989 <= 13
 0 <= w(4)(4)(3)(5)#990 <= 14
 0 <= w(4)(4)(3)(6)#991 <= 16
 0 <= w(4)(4)(3)(7)#992 <= 15
 0 <= w(4)(4)(3)(8)#993 <= 16
 0 <= w(4)(4)(3)(9)#994 <= 18
 0 <= w(4)(4)(3)(10)#995 <= 14
 0 <= w(4)(5)(1)(1)#996 <= 18
 0 <= w(4)(5)(1)(2)#997 <= 12
 0 <= w(4)(5)(1)(3)#998 <= 17
 0 <= w(4)(5)(1)(4)#999 <= 18
 0 <= w(4)(5)(1)(5)#1000 <= 17
 0 <= w(4)(5)(1)(6)#1001 <= 13
 0 <= w(4)(5)(1)(7)#1002 <= 13
 0 <= w(4)(5)(1)(8)#1003 <= 13
 0 <= w(4)(5)(1)(9)#1004 <= 18
 0 <= w(4)(5)(1)(10)#1005 <= 14
 0 <= w(4)(5)(2)(1)#1006 <= 12
 0 <= w(4)(5)(2)(2)#1007 <= 16
 0 <= w(4)(5)(2)(3)#1008 <= 17
 0 <= w(4)(5)(2)(4)#1009 <= 15
 0 <= w(4)(5)(2)(5)#1010 <= 17
 0 <= w(4)(5)(2)(6)#1011 <= 13
 0 <= w(4)(5)(2)(7)#1012 <= 17
 0 <= w(4)(5)(2)(8)#1013 <= 13
 0 <= w(4)(5)(2)(9)#1014 <= 15
 0 <= w(4)(5)(2)(10)#1015 <= 14
 0 <= w(4)(5)(3)(1)#1016 <= 17
 0 <= w(4)(5)(3)(2)#1017 <= 14
 0 <= w(4)(5)(3)(3)#1018 <= 15
 0 <= w(4)(5)(3)(4)#1019 <= 13
 0 <= w(4)(5)(3)(5)#1020 <= 14
 0 <= w(4)(5)(3)(6)#1021 <= 16
 0 <= w(4)(5)(3)(7)#1022 <= 15
 0 <= w(4)(5)(3)(8)#1023 <= 16
 0 <= w(4)(5)(3)(9)#1024 <= 18
 0 <= w(4)(5)(3)(10)#1025 <= 14
 0 <= w(4)(6)(1)(1)#1026 <= 18
 0 <= w(4)(6)(1)(2)#1027 <= 12
 0 <= w(4)(6)(1)(3)#1028 <= 17
 0 <= w(4)(6)(1)(4)#1029 <= 18
 0 <= w(4)(6)(1)(5)#1030 <= 17
 0 <= w(4)(6)(1)(6)#1031 <= 13
 0 <= w(4)(6)(1)(7)#1032 <= 13
 0 <= w(4)(6)(1)(8)#1033 <= 13
 0 <= w(4)(6)(1)(9)#1034 <= 18
 0 <= w(4)(6)(1)(10)#1035 <= 14
 0 <= w(4)(6)(2)(1)#1036 <= 12
 0 <= w(4)(6)(2)(2)#1037 <= 16
 0 <= w(4)(6)(2)(3)#1038 <= 17
 0 <= w(4)(6)(2)(4)#1039 <= 15
 0 <= w(4)(6)(2)(5)#1040 <= 17
 0 <= w(4)(6)(2)(6)#1041 <= 13
 0 <= w(4)(6)(2)(7)#1042 <= 17
 0 <= w(4)(6)(2)(8)#1043 <= 13
 0 <= w(4)(6)(2)(9)#1044 <= 15
 0 <= w(4)(6)(2)(10)#1045 <= 14
 0 <= w(4)(6)(3)(1)#1046 <= 17
 0 <= w(4)(6)(3)(2)#1047 <= 14
 0 <= w(4)(6)(3)(3)#1048 <= 15
 0 <= w(4)(6)(3)(4)#1049 <= 13
 0 <= w(4)(6)(3)(5)#1050 <= 14
 0 <= w(4)(6)(3)(6)#1051 <= 16
 0 <= w(4)(6)(3)(7)#1052 <= 15
 0 <= w(4)(6)(3)(8)#1053 <= 16
 0 <= w(4)(6)(3)(9)#1054 <= 18
 0 <= w(4)(6)(3)(10)#1055 <= 14
 0 <= w(5)(1)(1)(1)#1056 <= 13
 0 <= w(5)(1)(1)(2)#1057 <= 12
 0 <= w(5)(1)(1)(3)#1058 <= 15
 0 <= w(5)(1)(1)(4)#1059 <= 17
 0 <= w(5)(1)(1)(5)#1060 <= 17
 0 <= w(5)(1)(1)(6)#1061 <= 14
 0 <= w(5)(1)(1)(7)#1062 <= 15
 0 <= w(5)(1)(1)(8)#1063 <= 13
 0 <= w(5)(1)(1)(9)#1064 <= 13
 0 <= w(5)(1)(1)(10)#1065 <= 15
 0 <= w(5)(1)(2)(1)#1066 <= 15
 0 <= w(5)(1)(2)(2)#1067 <= 17
 0 <= w(5)(1)(2)(3)#1068 <= 18
 0 <= w(5)(1)(2)(4)#1069 <= 14
 0 <= w(5)(1)(2)(5)#1070 <= 12
 0 <= w(5)(1)(2)(6)#1071 <= 17
 0 <= w(5)(1)(2)(7)#1072 <= 13
 0 <= w(5)(1)(2)(8)#1073 <= 13
 0 <= w(5)(1)(2)(9)#1074 <= 17
 0 <= w(5)(1)(2)(10)#1075 <= 13
 0 <= w(5)(1)(3)(1)#1076 <= 16
 0 <= w(5)(1)(3)(2)#1077 <= 17
 0 <= w(5)(1)(3)(3)#1078 <= 13
 0 <= w(5)(1)(3)(4)#1079 <= 16
 0 <= w(5)(1)(3)(5)#1080 <= 14
 0 <= w(5)(1)(3)(6)#1081 <= 13
 0 <= w(5)(1)(3)(7)#1082 <= 12
 0 <= w(5)(1)(3)(8)#1083 <= 17
 0 <= w(5)(1)(3)(9)#1084 <= 13
 0 <= w(5)(1)(3)(10)#1085 <= 16
 0 <= w(5)(2)(1)(1)#1086 <= 13
 0 <= w(5)(2)(1)(2)#1087 <= 12
 0 <= w(5)(2)(1)(3)#1088 <= 15
 0 <= w(5)(2)(1)(4)#1089 <= 17
 0 <= w(5)(2)(1)(5)#1090 <= 17
 0 <= w(5)(2)(1)(6)#1091 <= 14
 0 <= w(5)(2)(1)(7)#1092 <= 15
 0 <= w(5)(2)(1)(8)#1093 <= 13
 0 <= w(5)(2)(1)(9)#1094 <= 13
 0 <= w(5)(2)(1)(10)#1095 <= 15
 0 <= w(5)(2)(2)(1)#1096 <= 15
 0 <= w(5)(2)(2)(2)#1097 <= 17
 0 <= w(5)(2)(2)(3)#1098 <= 18
 0 <= w(5)(2)(2)(4)#1099 <= 14
 0 <= w(5)(2)(2)(5)#1100 <= 12
 0 <= w(5)(2)(2)(6)#1101 <= 17
 0 <= w(5)(2)(2)(7)#1102 <= 13
 0 <= w(5)(2)(2)(8)#1103 <= 13
 0 <= w(5)(2)(2)(9)#1104 <= 17
 0 <= w(5)(2)(2)(10)#1105 <= 13
 0 <= w(5)(2)(3)(1)#1106 <= 16
 0 <= w(5)(2)(3)(2)#1107 <= 17
 0 <= w(5)(2)(3)(3)#1108 <= 13
 0 <= w(5)(2)(3)(4)#1109 <= 16
 0 <= w(5)(2)(3)(5)#1110 <= 14
 0 <= w(5)(2)(3)(6)#1111 <= 13
 0 <= w(5)(2)(3)(7)#1112 <= 12
 0 <= w(5)(2)(3)(8)#1113 <= 17
 0 <= w(5)(2)(3)(9)#1114 <= 13
 0 <= w(5)(2)(3)(10)#1115 <= 16
 0 <= w(5)(3)(1)(1)#1116 <= 13
 0 <= w(5)(3)(1)(2)#1117 <= 12
 0 <= w(5)(3)(1)(3)#1118 <= 15
 0 <= w(5)(3)(1)(4)#1119 <= 17
 0 <= w(5)(3)(1)(5)#1120 <= 17
 0 <= w(5)(3)(1)(6)#1121 <= 14
 0 <= w(5)(3)(1)(7)#1122 <= 15
 0 <= w(5)(3)(1)(8)#1123 <= 13
 0 <= w(5)(3)(1)(9)#1124 <= 13
 0 <= w(5)(3)(1)(10)#1125 <= 15
 0 <= w(5)(3)(2)(1)#1126 <= 15
 0 <= w(5)(3)(2)(2)#1127 <= 17
 0 <= w(5)(3)(2)(3)#1128 <= 18
 0 <= w(5)(3)(2)(4)#1129 <= 14
 0 <= w(5)(3)(2)(5)#1130 <= 12
 0 <= w(5)(3)(2)(6)#1131 <= 17
 0 <= w(5)(3)(2)(7)#1132 <= 13
 0 <= w(5)(3)(2)(8)#1133 <= 13
 0 <= w(5)(3)(2)(9)#1134 <= 17
 0 <= w(5)(3)(2)(10)#1135 <= 13
 0 <= w(5)(3)(3)(1)#1136 <= 16
 0 <= w(5)(3)(3)(2)#1137 <= 17
 0 <= w(5)(3)(3)(3)#1138 <= 13
 0 <= w(5)(3)(3)(4)#1139 <= 16
 0 <= w(5)(3)(3)(5)#1140 <= 14
 0 <= w(5)(3)(3)(6)#1141 <= 13
 0 <= w(5)(3)(3)(7)#1142 <= 12
 0 <= w(5)(3)(3)(8)#1143 <= 17
 0 <= w(5)(3)(3)(9)#1144 <= 13
 0 <= w(5)(3)(3)(10)#1145 <= 16
 0 <= w(5)(4)(1)(1)#1146 <= 13
 0 <= w(5)(4)(1)(2)#1147 <= 12
 0 <= w(5)(4)(1)(3)#1148 <= 15
 0 <= w(5)(4)(1)(4)#1149 <= 17
 0 <= w(5)(4)(1)(5)#1150 <= 17
 0 <= w(5)(4)(1)(6)#1151 <= 14
 0 <= w(5)(4)(1)(7)#1152 <= 15
 0 <= w(5)(4)(1)(8)#1153 <= 13
 0 <= w(5)(4)(1)(9)#1154 <= 13
 0 <= w(5)(4)(1)(10)#1155 <= 15
 0 <= w(5)(4)(2)(1)#1156 <= 15
 0 <= w(5)(4)(2)(2)#1157 <= 17
 0 <= w(5)(4)(2)(3)#1158 <= 18
 0 <= w(5)(4)(2)(4)#1159 <= 14
 0 <= w(5)(4)(2)(5)#1160 <= 12
 0 <= w(5)(4)(2)(6)#1161 <= 17
 0 <= w(5)(4)(2)(7)#1162 <= 13
 0 <= w(5)(4)(2)(8)#1163 <= 13
 0 <= w(5)(4)(2)(9)#1164 <= 17
 0 <= w(5)(4)(2)(10)#1165 <= 13
 0 <= w(5)(4)(3)(1)#1166 <= 16
 0 <= w(5)(4)(3)(2)#1167 <= 17
 0 <= w(5)(4)(3)(3)#1168 <= 13
 0 <= w(5)(4)(3)(4)#1169 <= 16
 0 <= w(5)(4)(3)(5)#1170 <= 14
 0 <= w(5)(4)(3)(6)#1171 <= 13
 0 <= w(5)(4)(3)(7)#1172 <= 12
 0 <= w(5)(4)(3)(8)#1173 <= 17
 0 <= w(5)(4)(3)(9)#1174 <= 13
 0 <= w(5)(4)(3)(10)#1175 <= 16
 0 <= w(5)(5)(1)(1)#1176 <= 13
 0 <= w(5)(5)(1)(2)#1177 <= 12
 0 <= w(5)(5)(1)(3)#1178 <= 15
 0 <= w(5)(5)(1)(4)#1179 <= 17
 0 <= w(5)(5)(1)(5)#1180 <= 17
 0 <= w(5)(5)(1)(6)#1181 <= 14
 0 <= w(5)(5)(1)(7)#1182 <= 15
 0 <= w(5)(5)(1)(8)#1183 <= 13
 0 <= w(5)(5)(1)(9)#1184 <= 13
 0 <= w(5)(5)(1)(10)#1185 <= 15
 0 <= w(5)(5)(2)(1)#1186 <= 15
 0 <= w(5)(5)(2)(2)#1187 <= 17
 0 <= w(5)(5)(2)(3)#1188 <= 18
 0 <= w(5)(5)(2)(4)#1189 <= 14
 0 <= w(5)(5)(2)(5)#1190 <= 12
 0 <= w(5)(5)(2)(6)#1191 <= 17
 0 <= w(5)(5)(2)(7)#1192 <= 13
 0 <= w(5)(5)(2)(8)#1193 <= 13
 0 <= w(5)(5)(2)(9)#1194 <= 17
 0 <= w(5)(5)(2)(10)#1195 <= 13
 0 <= w(5)(5)(3)(1)#1196 <= 16
 0 <= w(5)(5)(3)(2)#1197 <= 17
 0 <= w(5)(5)(3)(3)#1198 <= 13
 0 <= w(5)(5)(3)(4)#1199 <= 16
 0 <= w(5)(5)(3)(5)#1200 <= 14
 0 <= w(5)(5)(3)(6)#1201 <= 13
 0 <= w(5)(5)(3)(7)#1202 <= 12
 0 <= w(5)(5)(3)(8)#1203 <= 17
 0 <= w(5)(5)(3)(9)#1204 <= 13
 0 <= w(5)(5)(3)(10)#1205 <= 16
 0 <= w(5)(6)(1)(1)#1206 <= 13
 0 <= w(5)(6)(1)(2)#1207 <= 12
 0 <= w(5)(6)(1)(3)#1208 <= 15
 0 <= w(5)(6)(1)(4)#1209 <= 17
 0 <= w(5)(6)(1)(5)#1210 <= 17
 0 <= w(5)(6)(1)(6)#1211 <= 14
 0 <= w(5)(6)(1)(7)#1212 <= 15
 0 <= w(5)(6)(1)(8)#1213 <= 13
 0 <= w(5)(6)(1)(9)#1214 <= 13
 0 <= w(5)(6)(1)(10)#1215 <= 15
 0 <= w(5)(6)(2)(1)#1216 <= 15
 0 <= w(5)(6)(2)(2)#1217 <= 17
 0 <= w(5)(6)(2)(3)#1218 <= 18
 0 <= w(5)(6)(2)(4)#1219 <= 14
 0 <= w(5)(6)(2)(5)#1220 <= 12
 0 <= w(5)(6)(2)(6)#1221 <= 17
 0 <= w(5)(6)(2)(7)#1222 <= 13
 0 <= w(5)(6)(2)(8)#1223 <= 13
 0 <= w(5)(6)(2)(9)#1224 <= 17
 0 <= w(5)(6)(2)(10)#1225 <= 13
 0 <= w(5)(6)(3)(1)#1226 <= 16
 0 <= w(5)(6)(3)(2)#1227 <= 17
 0 <= w(5)(6)(3)(3)#1228 <= 13
 0 <= w(5)(6)(3)(4)#1229 <= 16
 0 <= w(5)(6)(3)(5)#1230 <= 14
 0 <= w(5)(6)(3)(6)#1231 <= 13
 0 <= w(5)(6)(3)(7)#1232 <= 12
 0 <= w(5)(6)(3)(8)#1233 <= 17
 0 <= w(5)(6)(3)(9)#1234 <= 13
 0 <= w(5)(6)(3)(10)#1235 <= 16
 0 <= w(6)(1)(1)(1)#1236 <= 6
 0 <= w(6)(1)(1)(2)#1237 <= 8
 0 <= w(6)(1)(1)(3)#1238 <= 7
 0 <= w(6)(1)(1)(4)#1239 <= 7
 0 <= w(6)(1)(1)(5)#1240 <= 8
 0 <= w(6)(1)(1)(6)#1241 <= 8
 0 <= w(6)(1)(1)(7)#1242 <= 8
 0 <= w(6)(1)(1)(8)#1243 <= 7
 0 <= w(6)(1)(1)(9)#1244 <= 7
 0 <= w(6)(1)(1)(10)#1245 <= 7
 0 <= w(6)(1)(2)(1)#1246 <= 7
 0 <= w(6)(1)(2)(2)#1247 <= 7
 0 <= w(6)(1)(2)(3)#1248 <= 6
 0 <= w(6)(1)(2)(4)#1249 <= 6
 0 <= w(6)(1)(2)(5)#1250 <= 7
 0 <= w(6)(1)(2)(6)#1251 <= 6
 0 <= w(6)(1)(2)(7)#1252 <= 6
 0 <= w(6)(1)(2)(8)#1253 <= 8
 0 <= w(6)(1)(2)(9)#1254 <= 7
 0 <= w(6)(1)(2)(10)#1255 <= 8
 0 <= w(6)(1)(3)(1)#1256 <= 7
 0 <= w(6)(1)(3)(2)#1257 <= 8
 0 <= w(6)(1)(3)(3)#1258 <= 7
 0 <= w(6)(1)(3)(4)#1259 <= 6
 0 <= w(6)(1)(3)(5)#1260 <= 8
 0 <= w(6)(1)(3)(6)#1261 <= 6
 0 <= w(6)(1)(3)(7)#1262 <= 7
 0 <= w(6)(1)(3)(8)#1263 <= 7
 0 <= w(6)(1)(3)(9)#1264 <= 6
 0 <= w(6)(1)(3)(10)#1265 <= 8
 0 <= w(6)(2)(1)(1)#1266 <= 6
 0 <= w(6)(2)(1)(2)#1267 <= 8
 0 <= w(6)(2)(1)(3)#1268 <= 7
 0 <= w(6)(2)(1)(4)#1269 <= 7
 0 <= w(6)(2)(1)(5)#1270 <= 8
 0 <= w(6)(2)(1)(6)#1271 <= 8
 0 <= w(6)(2)(1)(7)#1272 <= 8
 0 <= w(6)(2)(1)(8)#1273 <= 7
 0 <= w(6)(2)(1)(9)#1274 <= 7
 0 <= w(6)(2)(1)(10)#1275 <= 7
 0 <= w(6)(2)(2)(1)#1276 <= 7
 0 <= w(6)(2)(2)(2)#1277 <= 7
 0 <= w(6)(2)(2)(3)#1278 <= 6
 0 <= w(6)(2)(2)(4)#1279 <= 6
 0 <= w(6)(2)(2)(5)#1280 <= 7
 0 <= w(6)(2)(2)(6)#1281 <= 6
 0 <= w(6)(2)(2)(7)#1282 <= 6
 0 <= w(6)(2)(2)(8)#1283 <= 8
 0 <= w(6)(2)(2)(9)#1284 <= 7
 0 <= w(6)(2)(2)(10)#1285 <= 8
 0 <= w(6)(2)(3)(1)#1286 <= 7
 0 <= w(6)(2)(3)(2)#1287 <= 8
 0 <= w(6)(2)(3)(3)#1288 <= 7
 0 <= w(6)(2)(3)(4)#1289 <= 6
 0 <= w(6)(2)(3)(5)#1290 <= 8
 0 <= w(6)(2)(3)(6)#1291 <= 6
 0 <= w(6)(2)(3)(7)#1292 <= 7
 0 <= w(6)(2)(3)(8)#1293 <= 7
 0 <= w(6)(2)(3)(9)#1294 <= 6
 0 <= w(6)(2)(3)(10)#1295 <= 8
 0 <= w(6)(3)(1)(1)#1296 <= 6
 0 <= w(6)(3)(1)(2)#1297 <= 8
 0 <= w(6)(3)(1)(3)#1298 <= 7
 0 <= w(6)(3)(1)(4)#1299 <= 7
 0 <= w(6)(3)(1)(5)#1300 <= 8
 0 <= w(6)(3)(1)(6)#1301 <= 8
 0 <= w(6)(3)(1)(7)#1302 <= 8
 0 <= w(6)(3)(1)(8)#1303 <= 7
 0 <= w(6)(3)(1)(9)#1304 <= 7
 0 <= w(6)(3)(1)(10)#1305 <= 7
 0 <= w(6)(3)(2)(1)#1306 <= 7
 0 <= w(6)(3)(2)(2)#1307 <= 7
 0 <= w(6)(3)(2)(3)#1308 <= 6
 0 <= w(6)(3)(2)(4)#1309 <= 6
 0 <= w(6)(3)(2)(5)#1310 <= 7
 0 <= w(6)(3)(2)(6)#1311 <= 6
 0 <= w(6)(3)(2)(7)#1312 <= 6
 0 <= w(6)(3)(2)(8)#1313 <= 8
 0 <= w(6)(3)(2)(9)#1314 <= 7
 0 <= w(6)(3)(2)(10)#1315 <= 8
 0 <= w(6)(3)(3)(1)#1316 <= 7
 0 <= w(6)(3)(3)(2)#1317 <= 8
 0 <= w(6)(3)(3)(3)#1318 <= 7
 0 <= w(6)(3)(3)(4)#1319 <= 6
 0 <= w(6)(3)(3)(5)#1320 <= 8
 0 <= w(6)(3)(3)(6)#1321 <= 6
 0 <= w(6)(3)(3)(7)#1322 <= 7
 0 <= w(6)(3)(3)(8)#1323 <= 7
 0 <= w(6)(3)(3)(9)#1324 <= 6
 0 <= w(6)(3)(3)(10)#1325 <= 8
 0 <= w(6)(4)(1)(1)#1326 <= 6
 0 <= w(6)(4)(1)(2)#1327 <= 8
 0 <= w(6)(4)(1)(3)#1328 <= 7
 0 <= w(6)(4)(1)(4)#1329 <= 7
 0 <= w(6)(4)(1)(5)#1330 <= 8
 0 <= w(6)(4)(1)(6)#1331 <= 8
 0 <= w(6)(4)(1)(7)#1332 <= 8
 0 <= w(6)(4)(1)(8)#1333 <= 7
 0 <= w(6)(4)(1)(9)#1334 <= 7
 0 <= w(6)(4)(1)(10)#1335 <= 7
 0 <= w(6)(4)(2)(1)#1336 <= 7
 0 <= w(6)(4)(2)(2)#1337 <= 7
 0 <= w(6)(4)(2)(3)#1338 <= 6
 0 <= w(6)(4)(2)(4)#1339 <= 6
 0 <= w(6)(4)(2)(5)#1340 <= 7
 0 <= w(6)(4)(2)(6)#1341 <= 6
 0 <= w(6)(4)(2)(7)#1342 <= 6
 0 <= w(6)(4)(2)(8)#1343 <= 8
 0 <= w(6)(4)(2)(9)#1344 <= 7
 0 <= w(6)(4)(2)(10)#1345 <= 8
 0 <= w(6)(4)(3)(1)#1346 <= 7
 0 <= w(6)(4)(3)(2)#1347 <= 8
 0 <= w(6)(4)(3)(3)#1348 <= 7
 0 <= w(6)(4)(3)(4)#1349 <= 6
 0 <= w(6)(4)(3)(5)#1350 <= 8
 0 <= w(6)(4)(3)(6)#1351 <= 6
 0 <= w(6)(4)(3)(7)#1352 <= 7
 0 <= w(6)(4)(3)(8)#1353 <= 7
 0 <= w(6)(4)(3)(9)#1354 <= 6
 0 <= w(6)(4)(3)(10)#1355 <= 8
 0 <= w(6)(5)(1)(1)#1356 <= 6
 0 <= w(6)(5)(1)(2)#1357 <= 8
 0 <= w(6)(5)(1)(3)#1358 <= 7
 0 <= w(6)(5)(1)(4)#1359 <= 7
 0 <= w(6)(5)(1)(5)#1360 <= 8
 0 <= w(6)(5)(1)(6)#1361 <= 8
 0 <= w(6)(5)(1)(7)#1362 <= 8
 0 <= w(6)(5)(1)(8)#1363 <= 7
 0 <= w(6)(5)(1)(9)#1364 <= 7
 0 <= w(6)(5)(1)(10)#1365 <= 7
 0 <= w(6)(5)(2)(1)#1366 <= 7
 0 <= w(6)(5)(2)(2)#1367 <= 7
 0 <= w(6)(5)(2)(3)#1368 <= 6
 0 <= w(6)(5)(2)(4)#1369 <= 6
 0 <= w(6)(5)(2)(5)#1370 <= 7
 0 <= w(6)(5)(2)(6)#1371 <= 6
 0 <= w(6)(5)(2)(7)#1372 <= 6
 0 <= w(6)(5)(2)(8)#1373 <= 8
 0 <= w(6)(5)(2)(9)#1374 <= 7
 0 <= w(6)(5)(2)(10)#1375 <= 8
 0 <= w(6)(5)(3)(1)#1376 <= 7
 0 <= w(6)(5)(3)(2)#1377 <= 8
 0 <= w(6)(5)(3)(3)#1378 <= 7
 0 <= w(6)(5)(3)(4)#1379 <= 6
 0 <= w(6)(5)(3)(5)#1380 <= 8
 0 <= w(6)(5)(3)(6)#1381 <= 6
 0 <= w(6)(5)(3)(7)#1382 <= 7
 0 <= w(6)(5)(3)(8)#1383 <= 7
 0 <= w(6)(5)(3)(9)#1384 <= 6
 0 <= w(6)(5)(3)(10)#1385 <= 8
 0 <= w(6)(6)(1)(1)#1386 <= 6
 0 <= w(6)(6)(1)(2)#1387 <= 8
 0 <= w(6)(6)(1)(3)#1388 <= 7
 0 <= w(6)(6)(1)(4)#1389 <= 7
 0 <= w(6)(6)(1)(5)#1390 <= 8
 0 <= w(6)(6)(1)(6)#1391 <= 8
 0 <= w(6)(6)(1)(7)#1392 <= 8
 0 <= w(6)(6)(1)(8)#1393 <= 7
 0 <= w(6)(6)(1)(9)#1394 <= 7
 0 <= w(6)(6)(1)(10)#1395 <= 7
 0 <= w(6)(6)(2)(1)#1396 <= 7
 0 <= w(6)(6)(2)(2)#1397 <= 7
 0 <= w(6)(6)(2)(3)#1398 <= 6
 0 <= w(6)(6)(2)(4)#1399 <= 6
 0 <= w(6)(6)(2)(5)#1400 <= 7
 0 <= w(6)(6)(2)(6)#1401 <= 6
 0 <= w(6)(6)(2)(7)#1402 <= 6
 0 <= w(6)(6)(2)(8)#1403 <= 8
 0 <= w(6)(6)(2)(9)#1404 <= 7
 0 <= w(6)(6)(2)(10)#1405 <= 8
 0 <= w(6)(6)(3)(1)#1406 <= 7
 0 <= w(6)(6)(3)(2)#1407 <= 8
 0 <= w(6)(6)(3)(3)#1408 <= 7
 0 <= w(6)(6)(3)(4)#1409 <= 6
 0 <= w(6)(6)(3)(5)#1410 <= 8
 0 <= w(6)(6)(3)(6)#1411 <= 6
 0 <= w(6)(6)(3)(7)#1412 <= 7
 0 <= w(6)(6)(3)(8)#1413 <= 7
 0 <= w(6)(6)(3)(9)#1414 <= 6
 0 <= w(6)(6)(3)(10)#1415 <= 8
 0 <= w(7)(1)(1)(1)#1416 <= 15
 0 <= w(7)(1)(1)(2)#1417 <= 11
 0 <= w(7)(1)(1)(3)#1418 <= 14
 0 <= w(7)(1)(1)(4)#1419 <= 14
 0 <= w(7)(1)(1)(5)#1420 <= 14
 0 <= w(7)(1)(1)(6)#1421 <= 14
 0 <= w(7)(1)(1)(7)#1422 <= 12
 0 <= w(7)(1)(1)(8)#1423 <= 15
 0 <= w(7)(1)(1)(9)#1424 <= 14
 0 <= w(7)(1)(1)(10)#1425 <= 15
 0 <= w(7)(1)(2)(1)#1426 <= 15
 0 <= w(7)(1)(2)(2)#1427 <= 13
 0 <= w(7)(1)(2)(3)#1428 <= 13
 0 <= w(7)(1)(2)(4)#1429 <= 11
 0 <= w(7)(1)(2)(5)#1430 <= 12
 0 <= w(7)(1)(2)(6)#1431 <= 11
 0 <= w(7)(1)(2)(7)#1432 <= 13
 0 <= w(7)(1)(2)(8)#1433 <= 13
 0 <= w(7)(1)(2)(9)#1434 <= 13
 0 <= w(7)(1)(2)(10)#1435 <= 13
 0 <= w(7)(1)(3)(1)#1436 <= 12
 0 <= w(7)(1)(3)(2)#1437 <= 11
 0 <= w(7)(1)(3)(3)#1438 <= 13
 0 <= w(7)(1)(3)(4)#1439 <= 11
 0 <= w(7)(1)(3)(5)#1440 <= 12
 0 <= w(7)(1)(3)(6)#1441 <= 14
 0 <= w(7)(1)(3)(7)#1442 <= 14
 0 <= w(7)(1)(3)(8)#1443 <= 13
 0 <= w(7)(1)(3)(9)#1444 <= 13
 0 <= w(7)(1)(3)(10)#1445 <= 12
 0 <= w(7)(2)(1)(1)#1446 <= 15
 0 <= w(7)(2)(1)(2)#1447 <= 11
 0 <= w(7)(2)(1)(3)#1448 <= 14
 0 <= w(7)(2)(1)(4)#1449 <= 14
 0 <= w(7)(2)(1)(5)#1450 <= 14
 0 <= w(7)(2)(1)(6)#1451 <= 14
 0 <= w(7)(2)(1)(7)#1452 <= 12
 0 <= w(7)(2)(1)(8)#1453 <= 15
 0 <= w(7)(2)(1)(9)#1454 <= 14
 0 <= w(7)(2)(1)(10)#1455 <= 15
 0 <= w(7)(2)(2)(1)#1456 <= 15
 0 <= w(7)(2)(2)(2)#1457 <= 13
 0 <= w(7)(2)(2)(3)#1458 <= 13
 0 <= w(7)(2)(2)(4)#1459 <= 11
 0 <= w(7)(2)(2)(5)#1460 <= 12
 0 <= w(7)(2)(2)(6)#1461 <= 11
 0 <= w(7)(2)(2)(7)#1462 <= 13
 0 <= w(7)(2)(2)(8)#1463 <= 13
 0 <= w(7)(2)(2)(9)#1464 <= 13
 0 <= w(7)(2)(2)(10)#1465 <= 13
 0 <= w(7)(2)(3)(1)#1466 <= 12
 0 <= w(7)(2)(3)(2)#1467 <= 11
 0 <= w(7)(2)(3)(3)#1468 <= 13
 0 <= w(7)(2)(3)(4)#1469 <= 11
 0 <= w(7)(2)(3)(5)#1470 <= 12
 0 <= w(7)(2)(3)(6)#1471 <= 14
 0 <= w(7)(2)(3)(7)#1472 <= 14
 0 <= w(7)(2)(3)(8)#1473 <= 13
 0 <= w(7)(2)(3)(9)#1474 <= 13
 0 <= w(7)(2)(3)(10)#1475 <= 12
 0 <= w(7)(3)(1)(1)#1476 <= 15
 0 <= w(7)(3)(1)(2)#1477 <= 11
 0 <= w(7)(3)(1)(3)#1478 <= 14
 0 <= w(7)(3)(1)(4)#1479 <= 14
 0 <= w(7)(3)(1)(5)#1480 <= 14
 0 <= w(7)(3)(1)(6)#1481 <= 14
 0 <= w(7)(3)(1)(7)#1482 <= 12
 0 <= w(7)(3)(1)(8)#1483 <= 15
 0 <= w(7)(3)(1)(9)#1484 <= 14
 0 <= w(7)(3)(1)(10)#1485 <= 15
 0 <= w(7)(3)(2)(1)#1486 <= 15
 0 <= w(7)(3)(2)(2)#1487 <= 13
 0 <= w(7)(3)(2)(3)#1488 <= 13
 0 <= w(7)(3)(2)(4)#1489 <= 11
 0 <= w(7)(3)(2)(5)#1490 <= 12
 0 <= w(7)(3)(2)(6)#1491 <= 11
 0 <= w(7)(3)(2)(7)#1492 <= 13
 0 <= w(7)(3)(2)(8)#1493 <= 13
 0 <= w(7)(3)(2)(9)#1494 <= 13
 0 <= w(7)(3)(2)(10)#1495 <= 13
 0 <= w(7)(3)(3)(1)#1496 <= 12
 0 <= w(7)(3)(3)(2)#1497 <= 11
 0 <= w(7)(3)(3)(3)#1498 <= 13
 0 <= w(7)(3)(3)(4)#1499 <= 11
 0 <= w(7)(3)(3)(5)#1500 <= 12
 0 <= w(7)(3)(3)(6)#1501 <= 14
 0 <= w(7)(3)(3)(7)#1502 <= 14
 0 <= w(7)(3)(3)(8)#1503 <= 13
 0 <= w(7)(3)(3)(9)#1504 <= 13
 0 <= w(7)(3)(3)(10)#1505 <= 12
 0 <= w(7)(4)(1)(1)#1506 <= 15
 0 <= w(7)(4)(1)(2)#1507 <= 11
 0 <= w(7)(4)(1)(3)#1508 <= 14
 0 <= w(7)(4)(1)(4)#1509 <= 14
 0 <= w(7)(4)(1)(5)#1510 <= 14
 0 <= w(7)(4)(1)(6)#1511 <= 14
 0 <= w(7)(4)(1)(7)#1512 <= 12
 0 <= w(7)(4)(1)(8)#1513 <= 15
 0 <= w(7)(4)(1)(9)#1514 <= 14
 0 <= w(7)(4)(1)(10)#1515 <= 15
 0 <= w(7)(4)(2)(1)#1516 <= 15
 0 <= w(7)(4)(2)(2)#1517 <= 13
 0 <= w(7)(4)(2)(3)#1518 <= 13
 0 <= w(7)(4)(2)(4)#1519 <= 11
 0 <= w(7)(4)(2)(5)#1520 <= 12
 0 <= w(7)(4)(2)(6)#1521 <= 11
 0 <= w(7)(4)(2)(7)#1522 <= 13
 0 <= w(7)(4)(2)(8)#1523 <= 13
 0 <= w(7)(4)(2)(9)#1524 <= 13
 0 <= w(7)(4)(2)(10)#1525 <= 13
 0 <= w(7)(4)(3)(1)#1526 <= 12
 0 <= w(7)(4)(3)(2)#1527 <= 11
 0 <= w(7)(4)(3)(3)#1528 <= 13
 0 <= w(7)(4)(3)(4)#1529 <= 11
 0 <= w(7)(4)(3)(5)#1530 <= 12
 0 <= w(7)(4)(3)(6)#1531 <= 14
 0 <= w(7)(4)(3)(7)#1532 <= 14
 0 <= w(7)(4)(3)(8)#1533 <= 13
 0 <= w(7)(4)(3)(9)#1534 <= 13
 0 <= w(7)(4)(3)(10)#1535 <= 12
 0 <= w(7)(5)(1)(1)#1536 <= 15
 0 <= w(7)(5)(1)(2)#1537 <= 11
 0 <= w(7)(5)(1)(3)#1538 <= 14
 0 <= w(7)(5)(1)(4)#1539 <= 14
 0 <= w(7)(5)(1)(5)#1540 <= 14
 0 <= w(7)(5)(1)(6)#1541 <= 14
 0 <= w(7)(5)(1)(7)#1542 <= 12
 0 <= w(7)(5)(1)(8)#1543 <= 15
 0 <= w(7)(5)(1)(9)#1544 <= 14
 0 <= w(7)(5)(1)(10)#1545 <= 15
 0 <= w(7)(5)(2)(1)#1546 <= 15
 0 <= w(7)(5)(2)(2)#1547 <= 13
 0 <= w(7)(5)(2)(3)#1548 <= 13
 0 <= w(7)(5)(2)(4)#1549 <= 11
 0 <= w(7)(5)(2)(5)#1550 <= 12
 0 <= w(7)(5)(2)(6)#1551 <= 11
 0 <= w(7)(5)(2)(7)#1552 <= 13
 0 <= w(7)(5)(2)(8)#1553 <= 13
 0 <= w(7)(5)(2)(9)#1554 <= 13
 0 <= w(7)(5)(2)(10)#1555 <= 13
 0 <= w(7)(5)(3)(1)#1556 <= 12
 0 <= w(7)(5)(3)(2)#1557 <= 11
 0 <= w(7)(5)(3)(3)#1558 <= 13
 0 <= w(7)(5)(3)(4)#1559 <= 11
 0 <= w(7)(5)(3)(5)#1560 <= 12
 0 <= w(7)(5)(3)(6)#1561 <= 14
 0 <= w(7)(5)(3)(7)#1562 <= 14
 0 <= w(7)(5)(3)(8)#1563 <= 13
 0 <= w(7)(5)(3)(9)#1564 <= 13
 0 <= w(7)(5)(3)(10)#1565 <= 12
 0 <= w(7)(6)(1)(1)#1566 <= 15
 0 <= w(7)(6)(1)(2)#1567 <= 11
 0 <= w(7)(6)(1)(3)#1568 <= 14
 0 <= w(7)(6)(1)(4)#1569 <= 14
 0 <= w(7)(6)(1)(5)#1570 <= 14
 0 <= w(7)(6)(1)(6)#1571 <= 14
 0 <= w(7)(6)(1)(7)#1572 <= 12
 0 <= w(7)(6)(1)(8)#1573 <= 15
 0 <= w(7)(6)(1)(9)#1574 <= 14
 0 <= w(7)(6)(1)(10)#1575 <= 15
 0 <= w(7)(6)(2)(1)#1576 <= 15
 0 <= w(7)(6)(2)(2)#1577 <= 13
 0 <= w(7)(6)(2)(3)#1578 <= 13
 0 <= w(7)(6)(2)(4)#1579 <= 11
 0 <= w(7)(6)(2)(5)#1580 <= 12
 0 <= w(7)(6)(2)(6)#1581 <= 11
 0 <= w(7)(6)(2)(7)#1582 <= 13
 0 <= w(7)(6)(2)(8)#1583 <= 13
 0 <= w(7)(6)(2)(9)#1584 <= 13
 0 <= w(7)(6)(2)(10)#1585 <= 13
 0 <= w(7)(6)(3)(1)#1586 <= 12
 0 <= w(7)(6)(3)(2)#1587 <= 11
 0 <= w(7)(6)(3)(3)#1588 <= 13
 0 <= w(7)(6)(3)(4)#1589 <= 11
 0 <= w(7)(6)(3)(5)#1590 <= 12
 0 <= w(7)(6)(3)(6)#1591 <= 14
 0 <= w(7)(6)(3)(7)#1592 <= 14
 0 <= w(7)(6)(3)(8)#1593 <= 13
 0 <= w(7)(6)(3)(9)#1594 <= 13
 0 <= w(7)(6)(3)(10)#1595 <= 12
 0 <= w(8)(1)(1)(1)#1596 <= 13
 0 <= w(8)(1)(1)(2)#1597 <= 16
 0 <= w(8)(1)(1)(3)#1598 <= 18
 0 <= w(8)(1)(1)(4)#1599 <= 14
 0 <= w(8)(1)(1)(5)#1600 <= 19
 0 <= w(8)(1)(1)(6)#1601 <= 15
 0 <= w(8)(1)(1)(7)#1602 <= 18
 0 <= w(8)(1)(1)(8)#1603 <= 13
 0 <= w(8)(1)(1)(9)#1604 <= 17
 0 <= w(8)(1)(1)(10)#1605 <= 16
 0 <= w(8)(1)(2)(1)#1606 <= 18
 0 <= w(8)(1)(2)(2)#1607 <= 15
 0 <= w(8)(1)(2)(3)#1608 <= 17
 0 <= w(8)(1)(2)(4)#1609 <= 15
 0 <= w(8)(1)(2)(5)#1610 <= 14
 0 <= w(8)(1)(2)(6)#1611 <= 19
 0 <= w(8)(1)(2)(7)#1612 <= 18
 0 <= w(8)(1)(2)(8)#1613 <= 17
 0 <= w(8)(1)(2)(9)#1614 <= 15
 0 <= w(8)(1)(2)(10)#1615 <= 15
 0 <= w(8)(1)(3)(1)#1616 <= 17
 0 <= w(8)(1)(3)(2)#1617 <= 17
 0 <= w(8)(1)(3)(3)#1618 <= 19
 0 <= w(8)(1)(3)(4)#1619 <= 15
 0 <= w(8)(1)(3)(5)#1620 <= 14
 0 <= w(8)(1)(3)(6)#1621 <= 15
 0 <= w(8)(1)(3)(7)#1622 <= 14
 0 <= w(8)(1)(3)(8)#1623 <= 19
 0 <= w(8)(1)(3)(9)#1624 <= 13
 0 <= w(8)(1)(3)(10)#1625 <= 15
 0 <= w(8)(2)(1)(1)#1626 <= 13
 0 <= w(8)(2)(1)(2)#1627 <= 16
 0 <= w(8)(2)(1)(3)#1628 <= 18
 0 <= w(8)(2)(1)(4)#1629 <= 14
 0 <= w(8)(2)(1)(5)#1630 <= 19
 0 <= w(8)(2)(1)(6)#1631 <= 15
 0 <= w(8)(2)(1)(7)#1632 <= 18
 0 <= w(8)(2)(1)(8)#1633 <= 13
 0 <= w(8)(2)(1)(9)#1634 <= 17
 0 <= w(8)(2)(1)(10)#1635 <= 16
 0 <= w(8)(2)(2)(1)#1636 <= 18
 0 <= w(8)(2)(2)(2)#1637 <= 15
 0 <= w(8)(2)(2)(3)#1638 <= 17
 0 <= w(8)(2)(2)(4)#1639 <= 15
 0 <= w(8)(2)(2)(5)#1640 <= 14
 0 <= w(8)(2)(2)(6)#1641 <= 19
 0 <= w(8)(2)(2)(7)#1642 <= 18
 0 <= w(8)(2)(2)(8)#1643 <= 17
 0 <= w(8)(2)(2)(9)#1644 <= 15
 0 <= w(8)(2)(2)(10)#1645 <= 15
 0 <= w(8)(2)(3)(1)#1646 <= 17
 0 <= w(8)(2)(3)(2)#1647 <= 17
 0 <= w(8)(2)(3)(3)#1648 <= 19
 0 <= w(8)(2)(3)(4)#1649 <= 15
 0 <= w(8)(2)(3)(5)#1650 <= 14
 0 <= w(8)(2)(3)(6)#1651 <= 15
 0 <= w(8)(2)(3)(7)#1652 <= 14
 0 <= w(8)(2)(3)(8)#1653 <= 19
 0 <= w(8)(2)(3)(9)#1654 <= 13
 0 <= w(8)(2)(3)(10)#1655 <= 15
 0 <= w(8)(3)(1)(1)#1656 <= 13
 0 <= w(8)(3)(1)(2)#1657 <= 16
 0 <= w(8)(3)(1)(3)#1658 <= 18
 0 <= w(8)(3)(1)(4)#1659 <= 14
 0 <= w(8)(3)(1)(5)#1660 <= 19
 0 <= w(8)(3)(1)(6)#1661 <= 15
 0 <= w(8)(3)(1)(7)#1662 <= 18
 0 <= w(8)(3)(1)(8)#1663 <= 13
 0 <= w(8)(3)(1)(9)#1664 <= 17
 0 <= w(8)(3)(1)(10)#1665 <= 16
 0 <= w(8)(3)(2)(1)#1666 <= 18
 0 <= w(8)(3)(2)(2)#1667 <= 15
 0 <= w(8)(3)(2)(3)#1668 <= 17
 0 <= w(8)(3)(2)(4)#1669 <= 15
 0 <= w(8)(3)(2)(5)#1670 <= 14
 0 <= w(8)(3)(2)(6)#1671 <= 19
 0 <= w(8)(3)(2)(7)#1672 <= 18
 0 <= w(8)(3)(2)(8)#1673 <= 17
 0 <= w(8)(3)(2)(9)#1674 <= 15
 0 <= w(8)(3)(2)(10)#1675 <= 15
 0 <= w(8)(3)(3)(1)#1676 <= 17
 0 <= w(8)(3)(3)(2)#1677 <= 17
 0 <= w(8)(3)(3)(3)#1678 <= 19
 0 <= w(8)(3)(3)(4)#1679 <= 15
 0 <= w(8)(3)(3)(5)#1680 <= 14
 0 <= w(8)(3)(3)(6)#1681 <= 15
 0 <= w(8)(3)(3)(7)#1682 <= 14
 0 <= w(8)(3)(3)(8)#1683 <= 19
 0 <= w(8)(3)(3)(9)#1684 <= 13
 0 <= w(8)(3)(3)(10)#1685 <= 15
 0 <= w(8)(4)(1)(1)#1686 <= 13
 0 <= w(8)(4)(1)(2)#1687 <= 16
 0 <= w(8)(4)(1)(3)#1688 <= 18
 0 <= w(8)(4)(1)(4)#1689 <= 14
 0 <= w(8)(4)(1)(5)#1690 <= 19
 0 <= w(8)(4)(1)(6)#1691 <= 15
 0 <= w(8)(4)(1)(7)#1692 <= 18
 0 <= w(8)(4)(1)(8)#1693 <= 13
 0 <= w(8)(4)(1)(9)#1694 <= 17
 0 <= w(8)(4)(1)(10)#1695 <= 16
 0 <= w(8)(4)(2)(1)#1696 <= 18
 0 <= w(8)(4)(2)(2)#1697 <= 15
 0 <= w(8)(4)(2)(3)#1698 <= 17
 0 <= w(8)(4)(2)(4)#1699 <= 15
 0 <= w(8)(4)(2)(5)#1700 <= 14
 0 <= w(8)(4)(2)(6)#1701 <= 19
 0 <= w(8)(4)(2)(7)#1702 <= 18
 0 <= w(8)(4)(2)(8)#1703 <= 17
 0 <= w(8)(4)(2)(9)#1704 <= 15
 0 <= w(8)(4)(2)(10)#1705 <= 15
 0 <= w(8)(4)(3)(1)#1706 <= 17
 0 <= w(8)(4)(3)(2)#1707 <= 17
 0 <= w(8)(4)(3)(3)#1708 <= 19
 0 <= w(8)(4)(3)(4)#1709 <= 15
 0 <= w(8)(4)(3)(5)#1710 <= 14
 0 <= w(8)(4)(3)(6)#1711 <= 15
 0 <= w(8)(4)(3)(7)#1712 <= 14
 0 <= w(8)(4)(3)(8)#1713 <= 19
 0 <= w(8)(4)(3)(9)#1714 <= 13
 0 <= w(8)(4)(3)(10)#1715 <= 15
 0 <= w(8)(5)(1)(1)#1716 <= 13
 0 <= w(8)(5)(1)(2)#1717 <= 16
 0 <= w(8)(5)(1)(3)#1718 <= 18
 0 <= w(8)(5)(1)(4)#1719 <= 14
 0 <= w(8)(5)(1)(5)#1720 <= 19
 0 <= w(8)(5)(1)(6)#1721 <= 15
 0 <= w(8)(5)(1)(7)#1722 <= 18
 0 <= w(8)(5)(1)(8)#1723 <= 13
 0 <= w(8)(5)(1)(9)#1724 <= 17
 0 <= w(8)(5)(1)(10)#1725 <= 16
 0 <= w(8)(5)(2)(1)#1726 <= 18
 0 <= w(8)(5)(2)(2)#1727 <= 15
 0 <= w(8)(5)(2)(3)#1728 <= 17
 0 <= w(8)(5)(2)(4)#1729 <= 15
 0 <= w(8)(5)(2)(5)#1730 <= 14
 0 <= w(8)(5)(2)(6)#1731 <= 19
 0 <= w(8)(5)(2)(7)#1732 <= 18
 0 <= w(8)(5)(2)(8)#1733 <= 17
 0 <= w(8)(5)(2)(9)#1734 <= 15
 0 <= w(8)(5)(2)(10)#1735 <= 15
 0 <= w(8)(5)(3)(1)#1736 <= 17
 0 <= w(8)(5)(3)(2)#1737 <= 17
 0 <= w(8)(5)(3)(3)#1738 <= 19
 0 <= w(8)(5)(3)(4)#1739 <= 15
 0 <= w(8)(5)(3)(5)#1740 <= 14
 0 <= w(8)(5)(3)(6)#1741 <= 15
 0 <= w(8)(5)(3)(7)#1742 <= 14
 0 <= w(8)(5)(3)(8)#1743 <= 19
 0 <= w(8)(5)(3)(9)#1744 <= 13
 0 <= w(8)(5)(3)(10)#1745 <= 15
 0 <= w(8)(6)(1)(1)#1746 <= 13
 0 <= w(8)(6)(1)(2)#1747 <= 16
 0 <= w(8)(6)(1)(3)#1748 <= 18
 0 <= w(8)(6)(1)(4)#1749 <= 14
 0 <= w(8)(6)(1)(5)#1750 <= 19
 0 <= w(8)(6)(1)(6)#1751 <= 15
 0 <= w(8)(6)(1)(7)#1752 <= 18
 0 <= w(8)(6)(1)(8)#1753 <= 13
 0 <= w(8)(6)(1)(9)#1754 <= 17
 0 <= w(8)(6)(1)(10)#1755 <= 16
 0 <= w(8)(6)(2)(1)#1756 <= 18
 0 <= w(8)(6)(2)(2)#1757 <= 15
 0 <= w(8)(6)(2)(3)#1758 <= 17
 0 <= w(8)(6)(2)(4)#1759 <= 15
 0 <= w(8)(6)(2)(5)#1760 <= 14
 0 <= w(8)(6)(2)(6)#1761 <= 19
 0 <= w(8)(6)(2)(7)#1762 <= 18
 0 <= w(8)(6)(2)(8)#1763 <= 17
 0 <= w(8)(6)(2)(9)#1764 <= 15
 0 <= w(8)(6)(2)(10)#1765 <= 15
 0 <= w(8)(6)(3)(1)#1766 <= 17
 0 <= w(8)(6)(3)(2)#1767 <= 17
 0 <= w(8)(6)(3)(3)#1768 <= 19
 0 <= w(8)(6)(3)(4)#1769 <= 15
 0 <= w(8)(6)(3)(5)#1770 <= 14
 0 <= w(8)(6)(3)(6)#1771 <= 15
 0 <= w(8)(6)(3)(7)#1772 <= 14
 0 <= w(8)(6)(3)(8)#1773 <= 19
 0 <= w(8)(6)(3)(9)#1774 <= 13
 0 <= w(8)(6)(3)(10)#1775 <= 15
 0 <= w(9)(1)(1)(1)#1776 <= 26
 0 <= w(9)(1)(1)(2)#1777 <= 18
 0 <= w(9)(1)(1)(3)#1778 <= 19
 0 <= w(9)(1)(1)(4)#1779 <= 22
 0 <= w(9)(1)(1)(5)#1780 <= 26
 0 <= w(9)(1)(1)(6)#1781 <= 26
 0 <= w(9)(1)(1)(7)#1782 <= 18
 0 <= w(9)(1)(1)(8)#1783 <= 21
 0 <= w(9)(1)(1)(9)#1784 <= 24
 0 <= w(9)(1)(1)(10)#1785 <= 20
 0 <= w(9)(1)(2)(1)#1786 <= 20
 0 <= w(9)(1)(2)(2)#1787 <= 20
 0 <= w(9)(1)(2)(3)#1788 <= 22
 0 <= w(9)(1)(2)(4)#1789 <= 24
 0 <= w(9)(1)(2)(5)#1790 <= 26
 0 <= w(9)(1)(2)(6)#1791 <= 19
 0 <= w(9)(1)(2)(7)#1792 <= 21
 0 <= w(9)(1)(2)(8)#1793 <= 20
 0 <= w(9)(1)(2)(9)#1794 <= 25
 0 <= w(9)(1)(2)(10)#1795 <= 25
 0 <= w(9)(1)(3)(1)#1796 <= 24
 0 <= w(9)(1)(3)(2)#1797 <= 18
 0 <= w(9)(1)(3)(3)#1798 <= 26
 0 <= w(9)(1)(3)(4)#1799 <= 23
 0 <= w(9)(1)(3)(5)#1800 <= 24
 0 <= w(9)(1)(3)(6)#1801 <= 26
 0 <= w(9)(1)(3)(7)#1802 <= 26
 0 <= w(9)(1)(3)(8)#1803 <= 25
 0 <= w(9)(1)(3)(9)#1804 <= 24
 0 <= w(9)(1)(3)(10)#1805 <= 25
 0 <= w(9)(2)(1)(1)#1806 <= 26
 0 <= w(9)(2)(1)(2)#1807 <= 18
 0 <= w(9)(2)(1)(3)#1808 <= 19
 0 <= w(9)(2)(1)(4)#1809 <= 22
 0 <= w(9)(2)(1)(5)#1810 <= 26
 0 <= w(9)(2)(1)(6)#1811 <= 26
 0 <= w(9)(2)(1)(7)#1812 <= 18
 0 <= w(9)(2)(1)(8)#1813 <= 21
 0 <= w(9)(2)(1)(9)#1814 <= 24
 0 <= w(9)(2)(1)(10)#1815 <= 20
 0 <= w(9)(2)(2)(1)#1816 <= 20
 0 <= w(9)(2)(2)(2)#1817 <= 20
 0 <= w(9)(2)(2)(3)#1818 <= 22
 0 <= w(9)(2)(2)(4)#1819 <= 24
 0 <= w(9)(2)(2)(5)#1820 <= 26
 0 <= w(9)(2)(2)(6)#1821 <= 19
 0 <= w(9)(2)(2)(7)#1822 <= 21
 0 <= w(9)(2)(2)(8)#1823 <= 20
 0 <= w(9)(2)(2)(9)#1824 <= 25
 0 <= w(9)(2)(2)(10)#1825 <= 25
 0 <= w(9)(2)(3)(1)#1826 <= 24
 0 <= w(9)(2)(3)(2)#1827 <= 18
 0 <= w(9)(2)(3)(3)#1828 <= 26
 0 <= w(9)(2)(3)(4)#1829 <= 23
 0 <= w(9)(2)(3)(5)#1830 <= 24
 0 <= w(9)(2)(3)(6)#1831 <= 26
 0 <= w(9)(2)(3)(7)#1832 <= 26
 0 <= w(9)(2)(3)(8)#1833 <= 25
 0 <= w(9)(2)(3)(9)#1834 <= 24
 0 <= w(9)(2)(3)(10)#1835 <= 25
 0 <= w(9)(3)(1)(1)#1836 <= 26
 0 <= w(9)(3)(1)(2)#1837 <= 18
 0 <= w(9)(3)(1)(3)#1838 <= 19
 0 <= w(9)(3)(1)(4)#1839 <= 22
 0 <= w(9)(3)(1)(5)#1840 <= 26
 0 <= w(9)(3)(1)(6)#1841 <= 26
 0 <= w(9)(3)(1)(7)#1842 <= 18
 0 <= w(9)(3)(1)(8)#1843 <= 21
 0 <= w(9)(3)(1)(9)#1844 <= 24
 0 <= w(9)(3)(1)(10)#1845 <= 20
 0 <= w(9)(3)(2)(1)#1846 <= 20
 0 <= w(9)(3)(2)(2)#1847 <= 20
 0 <= w(9)(3)(2)(3)#1848 <= 22
 0 <= w(9)(3)(2)(4)#1849 <= 24
 0 <= w(9)(3)(2)(5)#1850 <= 26
 0 <= w(9)(3)(2)(6)#1851 <= 19
 0 <= w(9)(3)(2)(7)#1852 <= 21
 0 <= w(9)(3)(2)(8)#1853 <= 20
 0 <= w(9)(3)(2)(9)#1854 <= 25
 0 <= w(9)(3)(2)(10)#1855 <= 25
 0 <= w(9)(3)(3)(1)#1856 <= 24
 0 <= w(9)(3)(3)(2)#1857 <= 18
 0 <= w(9)(3)(3)(3)#1858 <= 26
 0 <= w(9)(3)(3)(4)#1859 <= 23
 0 <= w(9)(3)(3)(5)#1860 <= 24
 0 <= w(9)(3)(3)(6)#1861 <= 26
 0 <= w(9)(3)(3)(7)#1862 <= 26
 0 <= w(9)(3)(3)(8)#1863 <= 25
 0 <= w(9)(3)(3)(9)#1864 <= 24
 0 <= w(9)(3)(3)(10)#1865 <= 25
 0 <= w(9)(4)(1)(1)#1866 <= 26
 0 <= w(9)(4)(1)(2)#1867 <= 18
 0 <= w(9)(4)(1)(3)#1868 <= 19
 0 <= w(9)(4)(1)(4)#1869 <= 22
 0 <= w(9)(4)(1)(5)#1870 <= 26
 0 <= w(9)(4)(1)(6)#1871 <= 26
 0 <= w(9)(4)(1)(7)#1872 <= 18
 0 <= w(9)(4)(1)(8)#1873 <= 21
 0 <= w(9)(4)(1)(9)#1874 <= 24
 0 <= w(9)(4)(1)(10)#1875 <= 20
 0 <= w(9)(4)(2)(1)#1876 <= 20
 0 <= w(9)(4)(2)(2)#1877 <= 20
 0 <= w(9)(4)(2)(3)#1878 <= 22
 0 <= w(9)(4)(2)(4)#1879 <= 24
 0 <= w(9)(4)(2)(5)#1880 <= 26
 0 <= w(9)(4)(2)(6)#1881 <= 19
 0 <= w(9)(4)(2)(7)#1882 <= 21
 0 <= w(9)(4)(2)(8)#1883 <= 20
 0 <= w(9)(4)(2)(9)#1884 <= 25
 0 <= w(9)(4)(2)(10)#1885 <= 25
 0 <= w(9)(4)(3)(1)#1886 <= 24
 0 <= w(9)(4)(3)(2)#1887 <= 18
 0 <= w(9)(4)(3)(3)#1888 <= 26
 0 <= w(9)(4)(3)(4)#1889 <= 23
 0 <= w(9)(4)(3)(5)#1890 <= 24
 0 <= w(9)(4)(3)(6)#1891 <= 26
 0 <= w(9)(4)(3)(7)#1892 <= 26
 0 <= w(9)(4)(3)(8)#1893 <= 25
 0 <= w(9)(4)(3)(9)#1894 <= 24
 0 <= w(9)(4)(3)(10)#1895 <= 25
 0 <= w(9)(5)(1)(1)#1896 <= 26
 0 <= w(9)(5)(1)(2)#1897 <= 18
 0 <= w(9)(5)(1)(3)#1898 <= 19
 0 <= w(9)(5)(1)(4)#1899 <= 22
 0 <= w(9)(5)(1)(5)#1900 <= 26
 0 <= w(9)(5)(1)(6)#1901 <= 26
 0 <= w(9)(5)(1)(7)#1902 <= 18
 0 <= w(9)(5)(1)(8)#1903 <= 21
 0 <= w(9)(5)(1)(9)#1904 <= 24
 0 <= w(9)(5)(1)(10)#1905 <= 20
 0 <= w(9)(5)(2)(1)#1906 <= 20
 0 <= w(9)(5)(2)(2)#1907 <= 20
 0 <= w(9)(5)(2)(3)#1908 <= 22
 0 <= w(9)(5)(2)(4)#1909 <= 24
 0 <= w(9)(5)(2)(5)#1910 <= 26
 0 <= w(9)(5)(2)(6)#1911 <= 19
 0 <= w(9)(5)(2)(7)#1912 <= 21
 0 <= w(9)(5)(2)(8)#1913 <= 20
 0 <= w(9)(5)(2)(9)#1914 <= 25
 0 <= w(9)(5)(2)(10)#1915 <= 25
 0 <= w(9)(5)(3)(1)#1916 <= 24
 0 <= w(9)(5)(3)(2)#1917 <= 18
 0 <= w(9)(5)(3)(3)#1918 <= 26
 0 <= w(9)(5)(3)(4)#1919 <= 23
 0 <= w(9)(5)(3)(5)#1920 <= 24
 0 <= w(9)(5)(3)(6)#1921 <= 26
 0 <= w(9)(5)(3)(7)#1922 <= 26
 0 <= w(9)(5)(3)(8)#1923 <= 25
 0 <= w(9)(5)(3)(9)#1924 <= 24
 0 <= w(9)(5)(3)(10)#1925 <= 25
 0 <= w(9)(6)(1)(1)#1926 <= 26
 0 <= w(9)(6)(1)(2)#1927 <= 18
 0 <= w(9)(6)(1)(3)#1928 <= 19
 0 <= w(9)(6)(1)(4)#1929 <= 22
 0 <= w(9)(6)(1)(5)#1930 <= 26
 0 <= w(9)(6)(1)(6)#1931 <= 26
 0 <= w(9)(6)(1)(7)#1932 <= 18
 0 <= w(9)(6)(1)(8)#1933 <= 21
 0 <= w(9)(6)(1)(9)#1934 <= 24
 0 <= w(9)(6)(1)(10)#1935 <= 20
 0 <= w(9)(6)(2)(1)#1936 <= 20
 0 <= w(9)(6)(2)(2)#1937 <= 20
 0 <= w(9)(6)(2)(3)#1938 <= 22
 0 <= w(9)(6)(2)(4)#1939 <= 24
 0 <= w(9)(6)(2)(5)#1940 <= 26
 0 <= w(9)(6)(2)(6)#1941 <= 19
 0 <= w(9)(6)(2)(7)#1942 <= 21
 0 <= w(9)(6)(2)(8)#1943 <= 20
 0 <= w(9)(6)(2)(9)#1944 <= 25
 0 <= w(9)(6)(2)(10)#1945 <= 25
 0 <= w(9)(6)(3)(1)#1946 <= 24
 0 <= w(9)(6)(3)(2)#1947 <= 18
 0 <= w(9)(6)(3)(3)#1948 <= 26
 0 <= w(9)(6)(3)(4)#1949 <= 23
 0 <= w(9)(6)(3)(5)#1950 <= 24
 0 <= w(9)(6)(3)(6)#1951 <= 26
 0 <= w(9)(6)(3)(7)#1952 <= 26
 0 <= w(9)(6)(3)(8)#1953 <= 25
 0 <= w(9)(6)(3)(9)#1954 <= 24
 0 <= w(9)(6)(3)(10)#1955 <= 25
 0 <= w(10)(1)(1)(1)#1956 <= 12
 0 <= w(10)(1)(1)(2)#1957 <= 11
 0 <= w(10)(1)(1)(3)#1958 <= 14
 0 <= w(10)(1)(1)(4)#1959 <= 15
 0 <= w(10)(1)(1)(5)#1960 <= 11
 0 <= w(10)(1)(1)(6)#1961 <= 13
 0 <= w(10)(1)(1)(7)#1962 <= 13
 0 <= w(10)(1)(1)(8)#1963 <= 10
 0 <= w(10)(1)(1)(9)#1964 <= 11
 0 <= w(10)(1)(1)(10)#1965 <= 11
 0 <= w(10)(1)(2)(1)#1966 <= 11
 0 <= w(10)(1)(2)(2)#1967 <= 15
 0 <= w(10)(1)(2)(3)#1968 <= 11
 0 <= w(10)(1)(2)(4)#1969 <= 16
 0 <= w(10)(1)(2)(5)#1970 <= 13
 0 <= w(10)(1)(2)(6)#1971 <= 14
 0 <= w(10)(1)(2)(7)#1972 <= 14
 0 <= w(10)(1)(2)(8)#1973 <= 11
 0 <= w(10)(1)(2)(9)#1974 <= 11
 0 <= w(10)(1)(2)(10)#1975 <= 13
 0 <= w(10)(1)(3)(1)#1976 <= 13
 0 <= w(10)(1)(3)(2)#1977 <= 14
 0 <= w(10)(1)(3)(3)#1978 <= 15
 0 <= w(10)(1)(3)(4)#1979 <= 15
 0 <= w(10)(1)(3)(5)#1980 <= 11
 0 <= w(10)(1)(3)(6)#1981 <= 14
 0 <= w(10)(1)(3)(7)#1982 <= 15
 0 <= w(10)(1)(3)(8)#1983 <= 14
 0 <= w(10)(1)(3)(9)#1984 <= 15
 0 <= w(10)(1)(3)(10)#1985 <= 13
 0 <= w(10)(2)(1)(1)#1986 <= 12
 0 <= w(10)(2)(1)(2)#1987 <= 11
 0 <= w(10)(2)(1)(3)#1988 <= 14
 0 <= w(10)(2)(1)(4)#1989 <= 15
 0 <= w(10)(2)(1)(5)#1990 <= 11
 0 <= w(10)(2)(1)(6)#1991 <= 13
 0 <= w(10)(2)(1)(7)#1992 <= 13
 0 <= w(10)(2)(1)(8)#1993 <= 10
 0 <= w(10)(2)(1)(9)#1994 <= 11
 0 <= w(10)(2)(1)(10)#1995 <= 11
 0 <= w(10)(2)(2)(1)#1996 <= 11
 0 <= w(10)(2)(2)(2)#1997 <= 15
 0 <= w(10)(2)(2)(3)#1998 <= 11
 0 <= w(10)(2)(2)(4)#1999 <= 16
 0 <= w(10)(2)(2)(5)#2000 <= 13
 0 <= w(10)(2)(2)(6)#2001 <= 14
 0 <= w(10)(2)(2)(7)#2002 <= 14
 0 <= w(10)(2)(2)(8)#2003 <= 11
 0 <= w(10)(2)(2)(9)#2004 <= 11
 0 <= w(10)(2)(2)(10)#2005 <= 13
 0 <= w(10)(2)(3)(1)#2006 <= 13
 0 <= w(10)(2)(3)(2)#2007 <= 14
 0 <= w(10)(2)(3)(3)#2008 <= 15
 0 <= w(10)(2)(3)(4)#2009 <= 15
 0 <= w(10)(2)(3)(5)#2010 <= 11
 0 <= w(10)(2)(3)(6)#2011 <= 14
 0 <= w(10)(2)(3)(7)#2012 <= 15
 0 <= w(10)(2)(3)(8)#2013 <= 14
 0 <= w(10)(2)(3)(9)#2014 <= 15
 0 <= w(10)(2)(3)(10)#2015 <= 13
 0 <= w(10)(3)(1)(1)#2016 <= 12
 0 <= w(10)(3)(1)(2)#2017 <= 11
 0 <= w(10)(3)(1)(3)#2018 <= 14
 0 <= w(10)(3)(1)(4)#2019 <= 15
 0 <= w(10)(3)(1)(5)#2020 <= 11
 0 <= w(10)(3)(1)(6)#2021 <= 13
 0 <= w(10)(3)(1)(7)#2022 <= 13
 0 <= w(10)(3)(1)(8)#2023 <= 10
 0 <= w(10)(3)(1)(9)#2024 <= 11
 0 <= w(10)(3)(1)(10)#2025 <= 11
 0 <= w(10)(3)(2)(1)#2026 <= 11
 0 <= w(10)(3)(2)(2)#2027 <= 15
 0 <= w(10)(3)(2)(3)#2028 <= 11
 0 <= w(10)(3)(2)(4)#2029 <= 16
 0 <= w(10)(3)(2)(5)#2030 <= 13
 0 <= w(10)(3)(2)(6)#2031 <= 14
 0 <= w(10)(3)(2)(7)#2032 <= 14
 0 <= w(10)(3)(2)(8)#2033 <= 11
 0 <= w(10)(3)(2)(9)#2034 <= 11
 0 <= w(10)(3)(2)(10)#2035 <= 13
 0 <= w(10)(3)(3)(1)#2036 <= 13
 0 <= w(10)(3)(3)(2)#2037 <= 14
 0 <= w(10)(3)(3)(3)#2038 <= 15
 0 <= w(10)(3)(3)(4)#2039 <= 15
 0 <= w(10)(3)(3)(5)#2040 <= 11
 0 <= w(10)(3)(3)(6)#2041 <= 14
 0 <= w(10)(3)(3)(7)#2042 <= 15
 0 <= w(10)(3)(3)(8)#2043 <= 14
 0 <= w(10)(3)(3)(9)#2044 <= 15
 0 <= w(10)(3)(3)(10)#2045 <= 13
 0 <= w(10)(4)(1)(1)#2046 <= 12
 0 <= w(10)(4)(1)(2)#2047 <= 11
 0 <= w(10)(4)(1)(3)#2048 <= 14
 0 <= w(10)(4)(1)(4)#2049 <= 15
 0 <= w(10)(4)(1)(5)#2050 <= 11
 0 <= w(10)(4)(1)(6)#2051 <= 13
 0 <= w(10)(4)(1)(7)#2052 <= 13
 0 <= w(10)(4)(1)(8)#2053 <= 10
 0 <= w(10)(4)(1)(9)#2054 <= 11
 0 <= w(10)(4)(1)(10)#2055 <= 11
 0 <= w(10)(4)(2)(1)#2056 <= 11
 0 <= w(10)(4)(2)(2)#2057 <= 15
 0 <= w(10)(4)(2)(3)#2058 <= 11
 0 <= w(10)(4)(2)(4)#2059 <= 16
 0 <= w(10)(4)(2)(5)#2060 <= 13
 0 <= w(10)(4)(2)(6)#2061 <= 14
 0 <= w(10)(4)(2)(7)#2062 <= 14
 0 <= w(10)(4)(2)(8)#2063 <= 11
 0 <= w(10)(4)(2)(9)#2064 <= 11
 0 <= w(10)(4)(2)(10)#2065 <= 13
 0 <= w(10)(4)(3)(1)#2066 <= 13
 0 <= w(10)(4)(3)(2)#2067 <= 14
 0 <= w(10)(4)(3)(3)#2068 <= 15
 0 <= w(10)(4)(3)(4)#2069 <= 15
 0 <= w(10)(4)(3)(5)#2070 <= 11
 0 <= w(10)(4)(3)(6)#2071 <= 14
 0 <= w(10)(4)(3)(7)#2072 <= 15
 0 <= w(10)(4)(3)(8)#2073 <= 14
 0 <= w(10)(4)(3)(9)#2074 <= 15
 0 <= w(10)(4)(3)(10)#2075 <= 13
 0 <= w(10)(5)(1)(1)#2076 <= 12
 0 <= w(10)(5)(1)(2)#2077 <= 11
 0 <= w(10)(5)(1)(3)#2078 <= 14
 0 <= w(10)(5)(1)(4)#2079 <= 15
 0 <= w(10)(5)(1)(5)#2080 <= 11
 0 <= w(10)(5)(1)(6)#2081 <= 13
 0 <= w(10)(5)(1)(7)#2082 <= 13
 0 <= w(10)(5)(1)(8)#2083 <= 10
 0 <= w(10)(5)(1)(9)#2084 <= 11
 0 <= w(10)(5)(1)(10)#2085 <= 11
 0 <= w(10)(5)(2)(1)#2086 <= 11
 0 <= w(10)(5)(2)(2)#2087 <= 15
 0 <= w(10)(5)(2)(3)#2088 <= 11
 0 <= w(10)(5)(2)(4)#2089 <= 16
 0 <= w(10)(5)(2)(5)#2090 <= 13
 0 <= w(10)(5)(2)(6)#2091 <= 14
 0 <= w(10)(5)(2)(7)#2092 <= 14
 0 <= w(10)(5)(2)(8)#2093 <= 11
 0 <= w(10)(5)(2)(9)#2094 <= 11
 0 <= w(10)(5)(2)(10)#2095 <= 13
 0 <= w(10)(5)(3)(1)#2096 <= 13
 0 <= w(10)(5)(3)(2)#2097 <= 14
 0 <= w(10)(5)(3)(3)#2098 <= 15
 0 <= w(10)(5)(3)(4)#2099 <= 15
 0 <= w(10)(5)(3)(5)#2100 <= 11
 0 <= w(10)(5)(3)(6)#2101 <= 14
 0 <= w(10)(5)(3)(7)#2102 <= 15
 0 <= w(10)(5)(3)(8)#2103 <= 14
 0 <= w(10)(5)(3)(9)#2104 <= 15
 0 <= w(10)(5)(3)(10)#2105 <= 13
 0 <= w(10)(6)(1)(1)#2106 <= 12
 0 <= w(10)(6)(1)(2)#2107 <= 11
 0 <= w(10)(6)(1)(3)#2108 <= 14
 0 <= w(10)(6)(1)(4)#2109 <= 15
 0 <= w(10)(6)(1)(5)#2110 <= 11
 0 <= w(10)(6)(1)(6)#2111 <= 13
 0 <= w(10)(6)(1)(7)#2112 <= 13
 0 <= w(10)(6)(1)(8)#2113 <= 10
 0 <= w(10)(6)(1)(9)#2114 <= 11
 0 <= w(10)(6)(1)(10)#2115 <= 11
 0 <= w(10)(6)(2)(1)#2116 <= 11
 0 <= w(10)(6)(2)(2)#2117 <= 15
 0 <= w(10)(6)(2)(3)#2118 <= 11
 0 <= w(10)(6)(2)(4)#2119 <= 16
 0 <= w(10)(6)(2)(5)#2120 <= 13
 0 <= w(10)(6)(2)(6)#2121 <= 14
 0 <= w(10)(6)(2)(7)#2122 <= 14
 0 <= w(10)(6)(2)(8)#2123 <= 11
 0 <= w(10)(6)(2)(9)#2124 <= 11
 0 <= w(10)(6)(2)(10)#2125 <= 13
 0 <= w(10)(6)(3)(1)#2126 <= 13
 0 <= w(10)(6)(3)(2)#2127 <= 14
 0 <= w(10)(6)(3)(3)#2128 <= 15
 0 <= w(10)(6)(3)(4)#2129 <= 15
 0 <= w(10)(6)(3)(5)#2130 <= 11
 0 <= w(10)(6)(3)(6)#2131 <= 14
 0 <= w(10)(6)(3)(7)#2132 <= 15
 0 <= w(10)(6)(3)(8)#2133 <= 14
 0 <= w(10)(6)(3)(9)#2134 <= 15
 0 <= w(10)(6)(3)(10)#2135 <= 13
 0 <= w(11)(1)(1)(1)#2136 <= 15
 0 <= w(11)(1)(1)(2)#2137 <= 22
 0 <= w(11)(1)(1)(3)#2138 <= 18
 0 <= w(11)(1)(1)(4)#2139 <= 18
 0 <= w(11)(1)(1)(5)#2140 <= 22
 0 <= w(11)(1)(1)(6)#2141 <= 17
 0 <= w(11)(1)(1)(7)#2142 <= 20
 0 <= w(11)(1)(1)(8)#2143 <= 22
 0 <= w(11)(1)(1)(9)#2144 <= 21
 0 <= w(11)(1)(1)(10)#2145 <= 19
 0 <= w(11)(1)(2)(1)#2146 <= 16
 0 <= w(11)(1)(2)(2)#2147 <= 18
 0 <= w(11)(1)(2)(3)#2148 <= 22
 0 <= w(11)(1)(2)(4)#2149 <= 22
 0 <= w(11)(1)(2)(5)#2150 <= 21
 0 <= w(11)(1)(2)(6)#2151 <= 16
 0 <= w(11)(1)(2)(7)#2152 <= 20
 0 <= w(11)(1)(2)(8)#2153 <= 17
 0 <= w(11)(1)(2)(9)#2154 <= 15
 0 <= w(11)(1)(2)(10)#2155 <= 18
 0 <= w(11)(1)(3)(1)#2156 <= 16
 0 <= w(11)(1)(3)(2)#2157 <= 22
 0 <= w(11)(1)(3)(3)#2158 <= 20
 0 <= w(11)(1)(3)(4)#2159 <= 16
 0 <= w(11)(1)(3)(5)#2160 <= 16
 0 <= w(11)(1)(3)(6)#2161 <= 16
 0 <= w(11)(1)(3)(7)#2162 <= 21
 0 <= w(11)(1)(3)(8)#2163 <= 19
 0 <= w(11)(1)(3)(9)#2164 <= 19
 0 <= w(11)(1)(3)(10)#2165 <= 17
 0 <= w(11)(2)(1)(1)#2166 <= 15
 0 <= w(11)(2)(1)(2)#2167 <= 22
 0 <= w(11)(2)(1)(3)#2168 <= 18
 0 <= w(11)(2)(1)(4)#2169 <= 18
 0 <= w(11)(2)(1)(5)#2170 <= 22
 0 <= w(11)(2)(1)(6)#2171 <= 17
 0 <= w(11)(2)(1)(7)#2172 <= 20
 0 <= w(11)(2)(1)(8)#2173 <= 22
 0 <= w(11)(2)(1)(9)#2174 <= 21
 0 <= w(11)(2)(1)(10)#2175 <= 19
 0 <= w(11)(2)(2)(1)#2176 <= 16
 0 <= w(11)(2)(2)(2)#2177 <= 18
 0 <= w(11)(2)(2)(3)#2178 <= 22
 0 <= w(11)(2)(2)(4)#2179 <= 22
 0 <= w(11)(2)(2)(5)#2180 <= 21
 0 <= w(11)(2)(2)(6)#2181 <= 16
 0 <= w(11)(2)(2)(7)#2182 <= 20
 0 <= w(11)(2)(2)(8)#2183 <= 17
 0 <= w(11)(2)(2)(9)#2184 <= 15
 0 <= w(11)(2)(2)(10)#2185 <= 18
 0 <= w(11)(2)(3)(1)#2186 <= 16
 0 <= w(11)(2)(3)(2)#2187 <= 22
 0 <= w(11)(2)(3)(3)#2188 <= 20
 0 <= w(11)(2)(3)(4)#2189 <= 16
 0 <= w(11)(2)(3)(5)#2190 <= 16
 0 <= w(11)(2)(3)(6)#2191 <= 16
 0 <= w(11)(2)(3)(7)#2192 <= 21
 0 <= w(11)(2)(3)(8)#2193 <= 19
 0 <= w(11)(2)(3)(9)#2194 <= 19
 0 <= w(11)(2)(3)(10)#2195 <= 17
 0 <= w(11)(3)(1)(1)#2196 <= 15
 0 <= w(11)(3)(1)(2)#2197 <= 22
 0 <= w(11)(3)(1)(3)#2198 <= 18
 0 <= w(11)(3)(1)(4)#2199 <= 18
 0 <= w(11)(3)(1)(5)#2200 <= 22
 0 <= w(11)(3)(1)(6)#2201 <= 17
 0 <= w(11)(3)(1)(7)#2202 <= 20
 0 <= w(11)(3)(1)(8)#2203 <= 22
 0 <= w(11)(3)(1)(9)#2204 <= 21
 0 <= w(11)(3)(1)(10)#2205 <= 19
 0 <= w(11)(3)(2)(1)#2206 <= 16
 0 <= w(11)(3)(2)(2)#2207 <= 18
 0 <= w(11)(3)(2)(3)#2208 <= 22
 0 <= w(11)(3)(2)(4)#2209 <= 22
 0 <= w(11)(3)(2)(5)#2210 <= 21
 0 <= w(11)(3)(2)(6)#2211 <= 16
 0 <= w(11)(3)(2)(7)#2212 <= 20
 0 <= w(11)(3)(2)(8)#2213 <= 17
 0 <= w(11)(3)(2)(9)#2214 <= 15
 0 <= w(11)(3)(2)(10)#2215 <= 18
 0 <= w(11)(3)(3)(1)#2216 <= 16
 0 <= w(11)(3)(3)(2)#2217 <= 22
 0 <= w(11)(3)(3)(3)#2218 <= 20
 0 <= w(11)(3)(3)(4)#2219 <= 16
 0 <= w(11)(3)(3)(5)#2220 <= 16
 0 <= w(11)(3)(3)(6)#2221 <= 16
 0 <= w(11)(3)(3)(7)#2222 <= 21
 0 <= w(11)(3)(3)(8)#2223 <= 19
 0 <= w(11)(3)(3)(9)#2224 <= 19
 0 <= w(11)(3)(3)(10)#2225 <= 17
 0 <= w(11)(4)(1)(1)#2226 <= 15
 0 <= w(11)(4)(1)(2)#2227 <= 22
 0 <= w(11)(4)(1)(3)#2228 <= 18
 0 <= w(11)(4)(1)(4)#2229 <= 18
 0 <= w(11)(4)(1)(5)#2230 <= 22
 0 <= w(11)(4)(1)(6)#2231 <= 17
 0 <= w(11)(4)(1)(7)#2232 <= 20
 0 <= w(11)(4)(1)(8)#2233 <= 22
 0 <= w(11)(4)(1)(9)#2234 <= 21
 0 <= w(11)(4)(1)(10)#2235 <= 19
 0 <= w(11)(4)(2)(1)#2236 <= 16
 0 <= w(11)(4)(2)(2)#2237 <= 18
 0 <= w(11)(4)(2)(3)#2238 <= 22
 0 <= w(11)(4)(2)(4)#2239 <= 22
 0 <= w(11)(4)(2)(5)#2240 <= 21
 0 <= w(11)(4)(2)(6)#2241 <= 16
 0 <= w(11)(4)(2)(7)#2242 <= 20
 0 <= w(11)(4)(2)(8)#2243 <= 17
 0 <= w(11)(4)(2)(9)#2244 <= 15
 0 <= w(11)(4)(2)(10)#2245 <= 18
 0 <= w(11)(4)(3)(1)#2246 <= 16
 0 <= w(11)(4)(3)(2)#2247 <= 22
 0 <= w(11)(4)(3)(3)#2248 <= 20
 0 <= w(11)(4)(3)(4)#2249 <= 16
 0 <= w(11)(4)(3)(5)#2250 <= 16
 0 <= w(11)(4)(3)(6)#2251 <= 16
 0 <= w(11)(4)(3)(7)#2252 <= 21
 0 <= w(11)(4)(3)(8)#2253 <= 19
 0 <= w(11)(4)(3)(9)#2254 <= 19
 0 <= w(11)(4)(3)(10)#2255 <= 17
 0 <= w(11)(5)(1)(1)#2256 <= 15
 0 <= w(11)(5)(1)(2)#2257 <= 22
 0 <= w(11)(5)(1)(3)#2258 <= 18
 0 <= w(11)(5)(1)(4)#2259 <= 18
 0 <= w(11)(5)(1)(5)#2260 <= 22
 0 <= w(11)(5)(1)(6)#2261 <= 17
 0 <= w(11)(5)(1)(7)#2262 <= 20
 0 <= w(11)(5)(1)(8)#2263 <= 22
 0 <= w(11)(5)(1)(9)#2264 <= 21
 0 <= w(11)(5)(1)(10)#2265 <= 19
 0 <= w(11)(5)(2)(1)#2266 <= 16
 0 <= w(11)(5)(2)(2)#2267 <= 18
 0 <= w(11)(5)(2)(3)#2268 <= 22
 0 <= w(11)(5)(2)(4)#2269 <= 22
 0 <= w(11)(5)(2)(5)#2270 <= 21
 0 <= w(11)(5)(2)(6)#2271 <= 16
 0 <= w(11)(5)(2)(7)#2272 <= 20
 0 <= w(11)(5)(2)(8)#2273 <= 17
 0 <= w(11)(5)(2)(9)#2274 <= 15
 0 <= w(11)(5)(2)(10)#2275 <= 18
 0 <= w(11)(5)(3)(1)#2276 <= 16
 0 <= w(11)(5)(3)(2)#2277 <= 22
 0 <= w(11)(5)(3)(3)#2278 <= 20
 0 <= w(11)(5)(3)(4)#2279 <= 16
 0 <= w(11)(5)(3)(5)#2280 <= 16
 0 <= w(11)(5)(3)(6)#2281 <= 16
 0 <= w(11)(5)(3)(7)#2282 <= 21
 0 <= w(11)(5)(3)(8)#2283 <= 19
 0 <= w(11)(5)(3)(9)#2284 <= 19
 0 <= w(11)(5)(3)(10)#2285 <= 17
 0 <= w(11)(6)(1)(1)#2286 <= 15
 0 <= w(11)(6)(1)(2)#2287 <= 22
 0 <= w(11)(6)(1)(3)#2288 <= 18
 0 <= w(11)(6)(1)(4)#2289 <= 18
 0 <= w(11)(6)(1)(5)#2290 <= 22
 0 <= w(11)(6)(1)(6)#2291 <= 17
 0 <= w(11)(6)(1)(7)#2292 <= 20
 0 <= w(11)(6)(1)(8)#2293 <= 22
 0 <= w(11)(6)(1)(9)#2294 <= 21
 0 <= w(11)(6)(1)(10)#2295 <= 19
 0 <= w(11)(6)(2)(1)#2296 <= 16
 0 <= w(11)(6)(2)(2)#2297 <= 18
 0 <= w(11)(6)(2)(3)#2298 <= 22
 0 <= w(11)(6)(2)(4)#2299 <= 22
 0 <= w(11)(6)(2)(5)#2300 <= 21
 0 <= w(11)(6)(2)(6)#2301 <= 16
 0 <= w(11)(6)(2)(7)#2302 <= 20
 0 <= w(11)(6)(2)(8)#2303 <= 17
 0 <= w(11)(6)(2)(9)#2304 <= 15
 0 <= w(11)(6)(2)(10)#2305 <= 18
 0 <= w(11)(6)(3)(1)#2306 <= 16
 0 <= w(11)(6)(3)(2)#2307 <= 22
 0 <= w(11)(6)(3)(3)#2308 <= 20
 0 <= w(11)(6)(3)(4)#2309 <= 16
 0 <= w(11)(6)(3)(5)#2310 <= 16
 0 <= w(11)(6)(3)(6)#2311 <= 16
 0 <= w(11)(6)(3)(7)#2312 <= 21
 0 <= w(11)(6)(3)(8)#2313 <= 19
 0 <= w(11)(6)(3)(9)#2314 <= 19
 0 <= w(11)(6)(3)(10)#2315 <= 17
 0 <= w(12)(1)(1)(1)#2316 <= 21
 0 <= w(12)(1)(1)(2)#2317 <= 18
 0 <= w(12)(1)(1)(3)#2318 <= 21
 0 <= w(12)(1)(1)(4)#2319 <= 24
 0 <= w(12)(1)(1)(5)#2320 <= 22
 0 <= w(12)(1)(1)(6)#2321 <= 20
 0 <= w(12)(1)(1)(7)#2322 <= 20
 0 <= w(12)(1)(1)(8)#2323 <= 19
 0 <= w(12)(1)(1)(9)#2324 <= 18
 0 <= w(12)(1)(1)(10)#2325 <= 18
 0 <= w(12)(1)(2)(1)#2326 <= 21
 0 <= w(12)(1)(2)(2)#2327 <= 22
 0 <= w(12)(1)(2)(3)#2328 <= 21
 0 <= w(12)(1)(2)(4)#2329 <= 23
 0 <= w(12)(1)(2)(5)#2330 <= 19
 0 <= w(12)(1)(2)(6)#2331 <= 19
 0 <= w(12)(1)(2)(7)#2332 <= 23
 0 <= w(12)(1)(2)(8)#2333 <= 26
 0 <= w(12)(1)(2)(9)#2334 <= 25
 0 <= w(12)(1)(2)(10)#2335 <= 18
 0 <= w(12)(1)(3)(1)#2336 <= 25
 0 <= w(12)(1)(3)(2)#2337 <= 25
 0 <= w(12)(1)(3)(3)#2338 <= 21
 0 <= w(12)(1)(3)(4)#2339 <= 19
 0 <= w(12)(1)(3)(5)#2340 <= 19
 0 <= w(12)(1)(3)(6)#2341 <= 26
 0 <= w(12)(1)(3)(7)#2342 <= 25
 0 <= w(12)(1)(3)(8)#2343 <= 22
 0 <= w(12)(1)(3)(9)#2344 <= 21
 0 <= w(12)(1)(3)(10)#2345 <= 25
 0 <= w(12)(2)(1)(1)#2346 <= 21
 0 <= w(12)(2)(1)(2)#2347 <= 18
 0 <= w(12)(2)(1)(3)#2348 <= 21
 0 <= w(12)(2)(1)(4)#2349 <= 24
 0 <= w(12)(2)(1)(5)#2350 <= 22
 0 <= w(12)(2)(1)(6)#2351 <= 20
 0 <= w(12)(2)(1)(7)#2352 <= 20
 0 <= w(12)(2)(1)(8)#2353 <= 19
 0 <= w(12)(2)(1)(9)#2354 <= 18
 0 <= w(12)(2)(1)(10)#2355 <= 18
 0 <= w(12)(2)(2)(1)#2356 <= 21
 0 <= w(12)(2)(2)(2)#2357 <= 22
 0 <= w(12)(2)(2)(3)#2358 <= 21
 0 <= w(12)(2)(2)(4)#2359 <= 23
 0 <= w(12)(2)(2)(5)#2360 <= 19
 0 <= w(12)(2)(2)(6)#2361 <= 19
 0 <= w(12)(2)(2)(7)#2362 <= 23
 0 <= w(12)(2)(2)(8)#2363 <= 26
 0 <= w(12)(2)(2)(9)#2364 <= 25
 0 <= w(12)(2)(2)(10)#2365 <= 18
 0 <= w(12)(2)(3)(1)#2366 <= 25
 0 <= w(12)(2)(3)(2)#2367 <= 25
 0 <= w(12)(2)(3)(3)#2368 <= 21
 0 <= w(12)(2)(3)(4)#2369 <= 19
 0 <= w(12)(2)(3)(5)#2370 <= 19
 0 <= w(12)(2)(3)(6)#2371 <= 26
 0 <= w(12)(2)(3)(7)#2372 <= 25
 0 <= w(12)(2)(3)(8)#2373 <= 22
 0 <= w(12)(2)(3)(9)#2374 <= 21
 0 <= w(12)(2)(3)(10)#2375 <= 25
 0 <= w(12)(3)(1)(1)#2376 <= 21
 0 <= w(12)(3)(1)(2)#2377 <= 18
 0 <= w(12)(3)(1)(3)#2378 <= 21
 0 <= w(12)(3)(1)(4)#2379 <= 24
 0 <= w(12)(3)(1)(5)#2380 <= 22
 0 <= w(12)(3)(1)(6)#2381 <= 20
 0 <= w(12)(3)(1)(7)#2382 <= 20
 0 <= w(12)(3)(1)(8)#2383 <= 19
 0 <= w(12)(3)(1)(9)#2384 <= 18
 0 <= w(12)(3)(1)(10)#2385 <= 18
 0 <= w(12)(3)(2)(1)#2386 <= 21
 0 <= w(12)(3)(2)(2)#2387 <= 22
 0 <= w(12)(3)(2)(3)#2388 <= 21
 0 <= w(12)(3)(2)(4)#2389 <= 23
 0 <= w(12)(3)(2)(5)#2390 <= 19
 0 <= w(12)(3)(2)(6)#2391 <= 19
 0 <= w(12)(3)(2)(7)#2392 <= 23
 0 <= w(12)(3)(2)(8)#2393 <= 26
 0 <= w(12)(3)(2)(9)#2394 <= 25
 0 <= w(12)(3)(2)(10)#2395 <= 18
 0 <= w(12)(3)(3)(1)#2396 <= 25
 0 <= w(12)(3)(3)(2)#2397 <= 25
 0 <= w(12)(3)(3)(3)#2398 <= 21
 0 <= w(12)(3)(3)(4)#2399 <= 19
 0 <= w(12)(3)(3)(5)#2400 <= 19
 0 <= w(12)(3)(3)(6)#2401 <= 26
 0 <= w(12)(3)(3)(7)#2402 <= 25
 0 <= w(12)(3)(3)(8)#2403 <= 22
 0 <= w(12)(3)(3)(9)#2404 <= 21
 0 <= w(12)(3)(3)(10)#2405 <= 25
 0 <= w(12)(4)(1)(1)#2406 <= 21
 0 <= w(12)(4)(1)(2)#2407 <= 18
 0 <= w(12)(4)(1)(3)#2408 <= 21
 0 <= w(12)(4)(1)(4)#2409 <= 24
 0 <= w(12)(4)(1)(5)#2410 <= 22
 0 <= w(12)(4)(1)(6)#2411 <= 20
 0 <= w(12)(4)(1)(7)#2412 <= 20
 0 <= w(12)(4)(1)(8)#2413 <= 19
 0 <= w(12)(4)(1)(9)#2414 <= 18
 0 <= w(12)(4)(1)(10)#2415 <= 18
 0 <= w(12)(4)(2)(1)#2416 <= 21
 0 <= w(12)(4)(2)(2)#2417 <= 22
 0 <= w(12)(4)(2)(3)#2418 <= 21
 0 <= w(12)(4)(2)(4)#2419 <= 23
 0 <= w(12)(4)(2)(5)#2420 <= 19
 0 <= w(12)(4)(2)(6)#2421 <= 19
 0 <= w(12)(4)(2)(7)#2422 <= 23
 0 <= w(12)(4)(2)(8)#2423 <= 26
 0 <= w(12)(4)(2)(9)#2424 <= 25
 0 <= w(12)(4)(2)(10)#2425 <= 18
 0 <= w(12)(4)(3)(1)#2426 <= 25
 0 <= w(12)(4)(3)(2)#2427 <= 25
 0 <= w(12)(4)(3)(3)#2428 <= 21
 0 <= w(12)(4)(3)(4)#2429 <= 19
 0 <= w(12)(4)(3)(5)#2430 <= 19
 0 <= w(12)(4)(3)(6)#2431 <= 26
 0 <= w(12)(4)(3)(7)#2432 <= 25
 0 <= w(12)(4)(3)(8)#2433 <= 22
 0 <= w(12)(4)(3)(9)#2434 <= 21
 0 <= w(12)(4)(3)(10)#2435 <= 25
 0 <= w(12)(5)(1)(1)#2436 <= 21
 0 <= w(12)(5)(1)(2)#2437 <= 18
 0 <= w(12)(5)(1)(3)#2438 <= 21
 0 <= w(12)(5)(1)(4)#2439 <= 24
 0 <= w(12)(5)(1)(5)#2440 <= 22
 0 <= w(12)(5)(1)(6)#2441 <= 20
 0 <= w(12)(5)(1)(7)#2442 <= 20
 0 <= w(12)(5)(1)(8)#2443 <= 19
 0 <= w(12)(5)(1)(9)#2444 <= 18
 0 <= w(12)(5)(1)(10)#2445 <= 18
 0 <= w(12)(5)(2)(1)#2446 <= 21
 0 <= w(12)(5)(2)(2)#2447 <= 22
 0 <= w(12)(5)(2)(3)#2448 <= 21
 0 <= w(12)(5)(2)(4)#2449 <= 23
 0 <= w(12)(5)(2)(5)#2450 <= 19
 0 <= w(12)(5)(2)(6)#2451 <= 19
 0 <= w(12)(5)(2)(7)#2452 <= 23
 0 <= w(12)(5)(2)(8)#2453 <= 26
 0 <= w(12)(5)(2)(9)#2454 <= 25
 0 <= w(12)(5)(2)(10)#2455 <= 18
 0 <= w(12)(5)(3)(1)#2456 <= 25
 0 <= w(12)(5)(3)(2)#2457 <= 25
 0 <= w(12)(5)(3)(3)#2458 <= 21
 0 <= w(12)(5)(3)(4)#2459 <= 19
 0 <= w(12)(5)(3)(5)#2460 <= 19
 0 <= w(12)(5)(3)(6)#2461 <= 26
 0 <= w(12)(5)(3)(7)#2462 <= 25
 0 <= w(12)(5)(3)(8)#2463 <= 22
 0 <= w(12)(5)(3)(9)#2464 <= 21
 0 <= w(12)(5)(3)(10)#2465 <= 25
 0 <= w(12)(6)(1)(1)#2466 <= 21
 0 <= w(12)(6)(1)(2)#2467 <= 18
 0 <= w(12)(6)(1)(3)#2468 <= 21
 0 <= w(12)(6)(1)(4)#2469 <= 24
 0 <= w(12)(6)(1)(5)#2470 <= 22
 0 <= w(12)(6)(1)(6)#2471 <= 20
 0 <= w(12)(6)(1)(7)#2472 <= 20
 0 <= w(12)(6)(1)(8)#2473 <= 19
 0 <= w(12)(6)(1)(9)#2474 <= 18
 0 <= w(12)(6)(1)(10)#2475 <= 18
 0 <= w(12)(6)(2)(1)#2476 <= 21
 0 <= w(12)(6)(2)(2)#2477 <= 22
 0 <= w(12)(6)(2)(3)#2478 <= 21
 0 <= w(12)(6)(2)(4)#2479 <= 23
 0 <= w(12)(6)(2)(5)#2480 <= 19
 0 <= w(12)(6)(2)(6)#2481 <= 19
 0 <= w(12)(6)(2)(7)#2482 <= 23
 0 <= w(12)(6)(2)(8)#2483 <= 26
 0 <= w(12)(6)(2)(9)#2484 <= 25
 0 <= w(12)(6)(2)(10)#2485 <= 18
 0 <= w(12)(6)(3)(1)#2486 <= 25
 0 <= w(12)(6)(3)(2)#2487 <= 25
 0 <= w(12)(6)(3)(3)#2488 <= 21
 0 <= w(12)(6)(3)(4)#2489 <= 19
 0 <= w(12)(6)(3)(5)#2490 <= 19
 0 <= w(12)(6)(3)(6)#2491 <= 26
 0 <= w(12)(6)(3)(7)#2492 <= 25
 0 <= w(12)(6)(3)(8)#2493 <= 22
 0 <= w(12)(6)(3)(9)#2494 <= 21
 0 <= w(12)(6)(3)(10)#2495 <= 25
Binaries
 y(1)#0  y(2)#1  y(3)#2  o(1)(1)#687  o(1)(2)#688  o(1)(3)#689  o(2)(1)#690 
 o(2)(2)#691  o(2)(3)#692  o(3)(1)#693  o(3)(2)#694  o(3)(3)#695 
End
