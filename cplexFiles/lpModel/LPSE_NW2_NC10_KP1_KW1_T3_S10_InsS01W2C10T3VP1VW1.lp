\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 30 p(1)#0 + 30 p(2)#1 + 30 p(3)#2 + 3 I_plant(1)#3 + 3 I_plant(2)#4
       + 3 I_plant(3)#5 + 0.5 I_warehouse(1)(1)(1)#6
       + 0.5 I_warehouse(1)(1)(2)#7 + 0.5 I_warehouse(1)(1)(3)#8
       + 0.5 I_warehouse(1)(1)(4)#9 + 0.5 I_warehouse(1)(1)(5)#10
       + 0.5 I_warehouse(1)(1)(6)#11 + 0.5 I_warehouse(1)(1)(7)#12
       + 0.5 I_warehouse(1)(1)(8)#13 + 0.5 I_warehouse(1)(1)(9)#14
       + 0.5 I_warehouse(1)(1)(10)#15 + 0.5 I_warehouse(1)(2)(1)#16
       + 0.5 I_warehouse(1)(2)(2)#17 + 0.5 I_warehouse(1)(2)(3)#18
       + 0.5 I_warehouse(1)(2)(4)#19 + 0.5 I_warehouse(1)(2)(5)#20
       + 0.5 I_warehouse(1)(2)(6)#21 + 0.5 I_warehouse(1)(2)(7)#22
       + 0.5 I_warehouse(1)(2)(8)#23 + 0.5 I_warehouse(1)(2)(9)#24
       + 0.5 I_warehouse(1)(2)(10)#25 + 0.5 I_warehouse(1)(3)(1)#26
       + 0.5 I_warehouse(1)(3)(2)#27 + 0.5 I_warehouse(1)(3)(3)#28
       + 0.5 I_warehouse(1)(3)(4)#29 + 0.5 I_warehouse(1)(3)(5)#30
       + 0.5 I_warehouse(1)(3)(6)#31 + 0.5 I_warehouse(1)(3)(7)#32
       + 0.5 I_warehouse(1)(3)(8)#33 + 0.5 I_warehouse(1)(3)(9)#34
       + 0.5 I_warehouse(1)(3)(10)#35 + 0.5 I_warehouse(2)(1)(1)#36
       + 0.5 I_warehouse(2)(1)(2)#37 + 0.5 I_warehouse(2)(1)(3)#38
       + 0.5 I_warehouse(2)(1)(4)#39 + 0.5 I_warehouse(2)(1)(5)#40
       + 0.5 I_warehouse(2)(1)(6)#41 + 0.5 I_warehouse(2)(1)(7)#42
       + 0.5 I_warehouse(2)(1)(8)#43 + 0.5 I_warehouse(2)(1)(9)#44
       + 0.5 I_warehouse(2)(1)(10)#45 + 0.5 I_warehouse(2)(2)(1)#46
       + 0.5 I_warehouse(2)(2)(2)#47 + 0.5 I_warehouse(2)(2)(3)#48
       + 0.5 I_warehouse(2)(2)(4)#49 + 0.5 I_warehouse(2)(2)(5)#50
       + 0.5 I_warehouse(2)(2)(6)#51 + 0.5 I_warehouse(2)(2)(7)#52
       + 0.5 I_warehouse(2)(2)(8)#53 + 0.5 I_warehouse(2)(2)(9)#54
       + 0.5 I_warehouse(2)(2)(10)#55 + 0.5 I_warehouse(2)(3)(1)#56
       + 0.5 I_warehouse(2)(3)(2)#57 + 0.5 I_warehouse(2)(3)(3)#58
       + 0.5 I_warehouse(2)(3)(4)#59 + 0.5 I_warehouse(2)(3)(5)#60
       + 0.5 I_warehouse(2)(3)(6)#61 + 0.5 I_warehouse(2)(3)(7)#62
       + 0.5 I_warehouse(2)(3)(8)#63 + 0.5 I_warehouse(2)(3)(9)#64
       + 0.5 I_warehouse(2)(3)(10)#65 + 0.9 I_customer(3)(1)(1)#66
       + 0.9 I_customer(3)(1)(2)#67 + 0.9 I_customer(3)(1)(3)#68
       + 0.9 I_customer(3)(1)(4)#69 + 0.9 I_customer(3)(1)(5)#70
       + 0.9 I_customer(3)(1)(6)#71 + 0.9 I_customer(3)(1)(7)#72
       + 0.9 I_customer(3)(1)(8)#73 + 0.9 I_customer(3)(1)(9)#74
       + 0.9 I_customer(3)(1)(10)#75 + 0.9 I_customer(3)(2)(1)#76
       + 0.9 I_customer(3)(2)(2)#77 + 0.9 I_customer(3)(2)(3)#78
       + 0.9 I_customer(3)(2)(4)#79 + 0.9 I_customer(3)(2)(5)#80
       + 0.9 I_customer(3)(2)(6)#81 + 0.9 I_customer(3)(2)(7)#82
       + 0.9 I_customer(3)(2)(8)#83 + 0.9 I_customer(3)(2)(9)#84
       + 0.9 I_customer(3)(2)(10)#85 + 0.9 I_customer(3)(3)(1)#86
       + 0.9 I_customer(3)(3)(2)#87 + 0.9 I_customer(3)(3)(3)#88
       + 0.9 I_customer(3)(3)(4)#89 + 0.9 I_customer(3)(3)(5)#90
       + 0.9 I_customer(3)(3)(6)#91 + 0.9 I_customer(3)(3)(7)#92
       + 0.9 I_customer(3)(3)(8)#93 + 0.9 I_customer(3)(3)(9)#94
       + 0.9 I_customer(3)(3)(10)#95 + I_customer(4)(1)(1)#96
       + I_customer(4)(1)(2)#97 + I_customer(4)(1)(3)#98
       + I_customer(4)(1)(4)#99 + I_customer(4)(1)(5)#100
       + I_customer(4)(1)(6)#101 + I_customer(4)(1)(7)#102
       + I_customer(4)(1)(8)#103 + I_customer(4)(1)(9)#104
       + I_customer(4)(1)(10)#105 + I_customer(4)(2)(1)#106
       + I_customer(4)(2)(2)#107 + I_customer(4)(2)(3)#108
       + I_customer(4)(2)(4)#109 + I_customer(4)(2)(5)#110
       + I_customer(4)(2)(6)#111 + I_customer(4)(2)(7)#112
       + I_customer(4)(2)(8)#113 + I_customer(4)(2)(9)#114
       + I_customer(4)(2)(10)#115 + I_customer(4)(3)(1)#116
       + I_customer(4)(3)(2)#117 + I_customer(4)(3)(3)#118
       + I_customer(4)(3)(4)#119 + I_customer(4)(3)(5)#120
       + I_customer(4)(3)(6)#121 + I_customer(4)(3)(7)#122
       + I_customer(4)(3)(8)#123 + I_customer(4)(3)(9)#124
       + I_customer(4)(3)(10)#125 + I_customer(5)(1)(1)#126
       + I_customer(5)(1)(2)#127 + I_customer(5)(1)(3)#128
       + I_customer(5)(1)(4)#129 + I_customer(5)(1)(5)#130
       + I_customer(5)(1)(6)#131 + I_customer(5)(1)(7)#132
       + I_customer(5)(1)(8)#133 + I_customer(5)(1)(9)#134
       + I_customer(5)(1)(10)#135 + I_customer(5)(2)(1)#136
       + I_customer(5)(2)(2)#137 + I_customer(5)(2)(3)#138
       + I_customer(5)(2)(4)#139 + I_customer(5)(2)(5)#140
       + I_customer(5)(2)(6)#141 + I_customer(5)(2)(7)#142
       + I_customer(5)(2)(8)#143 + I_customer(5)(2)(9)#144
       + I_customer(5)(2)(10)#145 + I_customer(5)(3)(1)#146
       + I_customer(5)(3)(2)#147 + I_customer(5)(3)(3)#148
       + I_customer(5)(3)(4)#149 + I_customer(5)(3)(5)#150
       + I_customer(5)(3)(6)#151 + I_customer(5)(3)(7)#152
       + I_customer(5)(3)(8)#153 + I_customer(5)(3)(9)#154
       + I_customer(5)(3)(10)#155 + 0.8 I_customer(6)(1)(1)#156
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
       + 0.8 I_customer(6)(3)(10)#185 + 0.8 I_customer(7)(1)(1)#186
       + 0.8 I_customer(7)(1)(2)#187 + 0.8 I_customer(7)(1)(3)#188
       + 0.8 I_customer(7)(1)(4)#189 + 0.8 I_customer(7)(1)(5)#190
       + 0.8 I_customer(7)(1)(6)#191 + 0.8 I_customer(7)(1)(7)#192
       + 0.8 I_customer(7)(1)(8)#193 + 0.8 I_customer(7)(1)(9)#194
       + 0.8 I_customer(7)(1)(10)#195 + 0.8 I_customer(7)(2)(1)#196
       + 0.8 I_customer(7)(2)(2)#197 + 0.8 I_customer(7)(2)(3)#198
       + 0.8 I_customer(7)(2)(4)#199 + 0.8 I_customer(7)(2)(5)#200
       + 0.8 I_customer(7)(2)(6)#201 + 0.8 I_customer(7)(2)(7)#202
       + 0.8 I_customer(7)(2)(8)#203 + 0.8 I_customer(7)(2)(9)#204
       + 0.8 I_customer(7)(2)(10)#205 + 0.8 I_customer(7)(3)(1)#206
       + 0.8 I_customer(7)(3)(2)#207 + 0.8 I_customer(7)(3)(3)#208
       + 0.8 I_customer(7)(3)(4)#209 + 0.8 I_customer(7)(3)(5)#210
       + 0.8 I_customer(7)(3)(6)#211 + 0.8 I_customer(7)(3)(7)#212
       + 0.8 I_customer(7)(3)(8)#213 + 0.8 I_customer(7)(3)(9)#214
       + 0.8 I_customer(7)(3)(10)#215 + I_customer(8)(1)(1)#216
       + I_customer(8)(1)(2)#217 + I_customer(8)(1)(3)#218
       + I_customer(8)(1)(4)#219 + I_customer(8)(1)(5)#220
       + I_customer(8)(1)(6)#221 + I_customer(8)(1)(7)#222
       + I_customer(8)(1)(8)#223 + I_customer(8)(1)(9)#224
       + I_customer(8)(1)(10)#225 + I_customer(8)(2)(1)#226
       + I_customer(8)(2)(2)#227 + I_customer(8)(2)(3)#228
       + I_customer(8)(2)(4)#229 + I_customer(8)(2)(5)#230
       + I_customer(8)(2)(6)#231 + I_customer(8)(2)(7)#232
       + I_customer(8)(2)(8)#233 + I_customer(8)(2)(9)#234
       + I_customer(8)(2)(10)#235 + I_customer(8)(3)(1)#236
       + I_customer(8)(3)(2)#237 + I_customer(8)(3)(3)#238
       + I_customer(8)(3)(4)#239 + I_customer(8)(3)(5)#240
       + I_customer(8)(3)(6)#241 + I_customer(8)(3)(7)#242
       + I_customer(8)(3)(8)#243 + I_customer(8)(3)(9)#244
       + I_customer(8)(3)(10)#245 + I_customer(9)(1)(1)#246
       + I_customer(9)(1)(2)#247 + I_customer(9)(1)(3)#248
       + I_customer(9)(1)(4)#249 + I_customer(9)(1)(5)#250
       + I_customer(9)(1)(6)#251 + I_customer(9)(1)(7)#252
       + I_customer(9)(1)(8)#253 + I_customer(9)(1)(9)#254
       + I_customer(9)(1)(10)#255 + I_customer(9)(2)(1)#256
       + I_customer(9)(2)(2)#257 + I_customer(9)(2)(3)#258
       + I_customer(9)(2)(4)#259 + I_customer(9)(2)(5)#260
       + I_customer(9)(2)(6)#261 + I_customer(9)(2)(7)#262
       + I_customer(9)(2)(8)#263 + I_customer(9)(2)(9)#264
       + I_customer(9)(2)(10)#265 + I_customer(9)(3)(1)#266
       + I_customer(9)(3)(2)#267 + I_customer(9)(3)(3)#268
       + I_customer(9)(3)(4)#269 + I_customer(9)(3)(5)#270
       + I_customer(9)(3)(6)#271 + I_customer(9)(3)(7)#272
       + I_customer(9)(3)(8)#273 + I_customer(9)(3)(9)#274
       + I_customer(9)(3)(10)#275 + 0.7 I_customer(10)(1)(1)#276
       + 0.7 I_customer(10)(1)(2)#277 + 0.7 I_customer(10)(1)(3)#278
       + 0.7 I_customer(10)(1)(4)#279 + 0.7 I_customer(10)(1)(5)#280
       + 0.7 I_customer(10)(1)(6)#281 + 0.7 I_customer(10)(1)(7)#282
       + 0.7 I_customer(10)(1)(8)#283 + 0.7 I_customer(10)(1)(9)#284
       + 0.7 I_customer(10)(1)(10)#285 + 0.7 I_customer(10)(2)(1)#286
       + 0.7 I_customer(10)(2)(2)#287 + 0.7 I_customer(10)(2)(3)#288
       + 0.7 I_customer(10)(2)(4)#289 + 0.7 I_customer(10)(2)(5)#290
       + 0.7 I_customer(10)(2)(6)#291 + 0.7 I_customer(10)(2)(7)#292
       + 0.7 I_customer(10)(2)(8)#293 + 0.7 I_customer(10)(2)(9)#294
       + 0.7 I_customer(10)(2)(10)#295 + 0.7 I_customer(10)(3)(1)#296
       + 0.7 I_customer(10)(3)(2)#297 + 0.7 I_customer(10)(3)(3)#298
       + 0.7 I_customer(10)(3)(4)#299 + 0.7 I_customer(10)(3)(5)#300
       + 0.7 I_customer(10)(3)(6)#301 + 0.7 I_customer(10)(3)(7)#302
       + 0.7 I_customer(10)(3)(8)#303 + 0.7 I_customer(10)(3)(9)#304
       + 0.7 I_customer(10)(3)(10)#305 + 0.9 I_customer(11)(1)(1)#306
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
       + 0.7 I_customer(12)(3)(10)#365 + 470 b_customer(3)(1)(1)#366
       + 470 b_customer(3)(1)(2)#367 + 470 b_customer(3)(1)(3)#368
       + 470 b_customer(3)(1)(4)#369 + 470 b_customer(3)(1)(5)#370
       + 470 b_customer(3)(1)(6)#371 + 470 b_customer(3)(1)(7)#372
       + 470 b_customer(3)(1)(8)#373 + 470 b_customer(3)(1)(9)#374
       + 470 b_customer(3)(1)(10)#375 + 470 b_customer(3)(2)(1)#376
       + 470 b_customer(3)(2)(2)#377 + 470 b_customer(3)(2)(3)#378
       + 470 b_customer(3)(2)(4)#379 + 470 b_customer(3)(2)(5)#380
       + 470 b_customer(3)(2)(6)#381 + 470 b_customer(3)(2)(7)#382
       + 470 b_customer(3)(2)(8)#383 + 470 b_customer(3)(2)(9)#384
       + 470 b_customer(3)(2)(10)#385 + 470 b_customer(3)(3)(1)#386
       + 470 b_customer(3)(3)(2)#387 + 470 b_customer(3)(3)(3)#388
       + 470 b_customer(3)(3)(4)#389 + 470 b_customer(3)(3)(5)#390
       + 470 b_customer(3)(3)(6)#391 + 470 b_customer(3)(3)(7)#392
       + 470 b_customer(3)(3)(8)#393 + 470 b_customer(3)(3)(9)#394
       + 470 b_customer(3)(3)(10)#395 + 460 b_customer(4)(1)(1)#396
       + 460 b_customer(4)(1)(2)#397 + 460 b_customer(4)(1)(3)#398
       + 460 b_customer(4)(1)(4)#399 + 460 b_customer(4)(1)(5)#400
       + 460 b_customer(4)(1)(6)#401 + 460 b_customer(4)(1)(7)#402
       + 460 b_customer(4)(1)(8)#403 + 460 b_customer(4)(1)(9)#404
       + 460 b_customer(4)(1)(10)#405 + 460 b_customer(4)(2)(1)#406
       + 460 b_customer(4)(2)(2)#407 + 460 b_customer(4)(2)(3)#408
       + 460 b_customer(4)(2)(4)#409 + 460 b_customer(4)(2)(5)#410
       + 460 b_customer(4)(2)(6)#411 + 460 b_customer(4)(2)(7)#412
       + 460 b_customer(4)(2)(8)#413 + 460 b_customer(4)(2)(9)#414
       + 460 b_customer(4)(2)(10)#415 + 460 b_customer(4)(3)(1)#416
       + 460 b_customer(4)(3)(2)#417 + 460 b_customer(4)(3)(3)#418
       + 460 b_customer(4)(3)(4)#419 + 460 b_customer(4)(3)(5)#420
       + 460 b_customer(4)(3)(6)#421 + 460 b_customer(4)(3)(7)#422
       + 460 b_customer(4)(3)(8)#423 + 460 b_customer(4)(3)(9)#424
       + 460 b_customer(4)(3)(10)#425 + 480 b_customer(5)(1)(1)#426
       + 480 b_customer(5)(1)(2)#427 + 480 b_customer(5)(1)(3)#428
       + 480 b_customer(5)(1)(4)#429 + 480 b_customer(5)(1)(5)#430
       + 480 b_customer(5)(1)(6)#431 + 480 b_customer(5)(1)(7)#432
       + 480 b_customer(5)(1)(8)#433 + 480 b_customer(5)(1)(9)#434
       + 480 b_customer(5)(1)(10)#435 + 480 b_customer(5)(2)(1)#436
       + 480 b_customer(5)(2)(2)#437 + 480 b_customer(5)(2)(3)#438
       + 480 b_customer(5)(2)(4)#439 + 480 b_customer(5)(2)(5)#440
       + 480 b_customer(5)(2)(6)#441 + 480 b_customer(5)(2)(7)#442
       + 480 b_customer(5)(2)(8)#443 + 480 b_customer(5)(2)(9)#444
       + 480 b_customer(5)(2)(10)#445 + 480 b_customer(5)(3)(1)#446
       + 480 b_customer(5)(3)(2)#447 + 480 b_customer(5)(3)(3)#448
       + 480 b_customer(5)(3)(4)#449 + 480 b_customer(5)(3)(5)#450
       + 480 b_customer(5)(3)(6)#451 + 480 b_customer(5)(3)(7)#452
       + 480 b_customer(5)(3)(8)#453 + 480 b_customer(5)(3)(9)#454
       + 480 b_customer(5)(3)(10)#455 + 470 b_customer(6)(1)(1)#456
       + 470 b_customer(6)(1)(2)#457 + 470 b_customer(6)(1)(3)#458
       + 470 b_customer(6)(1)(4)#459 + 470 b_customer(6)(1)(5)#460
       + 470 b_customer(6)(1)(6)#461 + 470 b_customer(6)(1)(7)#462
       + 470 b_customer(6)(1)(8)#463 + 470 b_customer(6)(1)(9)#464
       + 470 b_customer(6)(1)(10)#465 + 470 b_customer(6)(2)(1)#466
       + 470 b_customer(6)(2)(2)#467 + 470 b_customer(6)(2)(3)#468
       + 470 b_customer(6)(2)(4)#469 + 470 b_customer(6)(2)(5)#470
       + 470 b_customer(6)(2)(6)#471 + 470 b_customer(6)(2)(7)#472
       + 470 b_customer(6)(2)(8)#473 + 470 b_customer(6)(2)(9)#474
       + 470 b_customer(6)(2)(10)#475 + 470 b_customer(6)(3)(1)#476
       + 470 b_customer(6)(3)(2)#477 + 470 b_customer(6)(3)(3)#478
       + 470 b_customer(6)(3)(4)#479 + 470 b_customer(6)(3)(5)#480
       + 470 b_customer(6)(3)(6)#481 + 470 b_customer(6)(3)(7)#482
       + 470 b_customer(6)(3)(8)#483 + 470 b_customer(6)(3)(9)#484
       + 470 b_customer(6)(3)(10)#485 + 480 b_customer(7)(1)(1)#486
       + 480 b_customer(7)(1)(2)#487 + 480 b_customer(7)(1)(3)#488
       + 480 b_customer(7)(1)(4)#489 + 480 b_customer(7)(1)(5)#490
       + 480 b_customer(7)(1)(6)#491 + 480 b_customer(7)(1)(7)#492
       + 480 b_customer(7)(1)(8)#493 + 480 b_customer(7)(1)(9)#494
       + 480 b_customer(7)(1)(10)#495 + 480 b_customer(7)(2)(1)#496
       + 480 b_customer(7)(2)(2)#497 + 480 b_customer(7)(2)(3)#498
       + 480 b_customer(7)(2)(4)#499 + 480 b_customer(7)(2)(5)#500
       + 480 b_customer(7)(2)(6)#501 + 480 b_customer(7)(2)(7)#502
       + 480 b_customer(7)(2)(8)#503 + 480 b_customer(7)(2)(9)#504
       + 480 b_customer(7)(2)(10)#505 + 480 b_customer(7)(3)(1)#506
       + 480 b_customer(7)(3)(2)#507 + 480 b_customer(7)(3)(3)#508
       + 480 b_customer(7)(3)(4)#509 + 480 b_customer(7)(3)(5)#510
       + 480 b_customer(7)(3)(6)#511 + 480 b_customer(7)(3)(7)#512
       + 480 b_customer(7)(3)(8)#513 + 480 b_customer(7)(3)(9)#514
       + 480 b_customer(7)(3)(10)#515 + 470 b_customer(8)(1)(1)#516
       + 470 b_customer(8)(1)(2)#517 + 470 b_customer(8)(1)(3)#518
       + 470 b_customer(8)(1)(4)#519 + 470 b_customer(8)(1)(5)#520
       + 470 b_customer(8)(1)(6)#521 + 470 b_customer(8)(1)(7)#522
       + 470 b_customer(8)(1)(8)#523 + 470 b_customer(8)(1)(9)#524
       + 470 b_customer(8)(1)(10)#525 + 470 b_customer(8)(2)(1)#526
       + 470 b_customer(8)(2)(2)#527 + 470 b_customer(8)(2)(3)#528
       + 470 b_customer(8)(2)(4)#529 + 470 b_customer(8)(2)(5)#530
       + 470 b_customer(8)(2)(6)#531 + 470 b_customer(8)(2)(7)#532
       + 470 b_customer(8)(2)(8)#533 + 470 b_customer(8)(2)(9)#534
       + 470 b_customer(8)(2)(10)#535 + 470 b_customer(8)(3)(1)#536
       + 470 b_customer(8)(3)(2)#537 + 470 b_customer(8)(3)(3)#538
       + 470 b_customer(8)(3)(4)#539 + 470 b_customer(8)(3)(5)#540
       + 470 b_customer(8)(3)(6)#541 + 470 b_customer(8)(3)(7)#542
       + 470 b_customer(8)(3)(8)#543 + 470 b_customer(8)(3)(9)#544
       + 470 b_customer(8)(3)(10)#545 + 470 b_customer(9)(1)(1)#546
       + 470 b_customer(9)(1)(2)#547 + 470 b_customer(9)(1)(3)#548
       + 470 b_customer(9)(1)(4)#549 + 470 b_customer(9)(1)(5)#550
       + 470 b_customer(9)(1)(6)#551 + 470 b_customer(9)(1)(7)#552
       + 470 b_customer(9)(1)(8)#553 + 470 b_customer(9)(1)(9)#554
       + 470 b_customer(9)(1)(10)#555 + 470 b_customer(9)(2)(1)#556
       + 470 b_customer(9)(2)(2)#557 + 470 b_customer(9)(2)(3)#558
       + 470 b_customer(9)(2)(4)#559 + 470 b_customer(9)(2)(5)#560
       + 470 b_customer(9)(2)(6)#561 + 470 b_customer(9)(2)(7)#562
       + 470 b_customer(9)(2)(8)#563 + 470 b_customer(9)(2)(9)#564
       + 470 b_customer(9)(2)(10)#565 + 470 b_customer(9)(3)(1)#566
       + 470 b_customer(9)(3)(2)#567 + 470 b_customer(9)(3)(3)#568
       + 470 b_customer(9)(3)(4)#569 + 470 b_customer(9)(3)(5)#570
       + 470 b_customer(9)(3)(6)#571 + 470 b_customer(9)(3)(7)#572
       + 470 b_customer(9)(3)(8)#573 + 470 b_customer(9)(3)(9)#574
       + 470 b_customer(9)(3)(10)#575 + 460 b_customer(10)(1)(1)#576
       + 460 b_customer(10)(1)(2)#577 + 460 b_customer(10)(1)(3)#578
       + 460 b_customer(10)(1)(4)#579 + 460 b_customer(10)(1)(5)#580
       + 460 b_customer(10)(1)(6)#581 + 460 b_customer(10)(1)(7)#582
       + 460 b_customer(10)(1)(8)#583 + 460 b_customer(10)(1)(9)#584
       + 460 b_customer(10)(1)(10)#585 + 460 b_customer(10)(2)(1)#586
       + 460 b_customer(10)(2)(2)#587 + 460 b_customer(10)(2)(3)#588
       + 460 b_customer(10)(2)(4)#589 + 460 b_customer(10)(2)(5)#590
       + 460 b_customer(10)(2)(6)#591 + 460 b_customer(10)(2)(7)#592
       + 460 b_customer(10)(2)(8)#593 + 460 b_customer(10)(2)(9)#594
       + 460 b_customer(10)(2)(10)#595 + 460 b_customer(10)(3)(1)#596
       + 460 b_customer(10)(3)(2)#597 + 460 b_customer(10)(3)(3)#598
       + 460 b_customer(10)(3)(4)#599 + 460 b_customer(10)(3)(5)#600
       + 460 b_customer(10)(3)(6)#601 + 460 b_customer(10)(3)(7)#602
       + 460 b_customer(10)(3)(8)#603 + 460 b_customer(10)(3)(9)#604
       + 460 b_customer(10)(3)(10)#605 + 460 b_customer(11)(1)(1)#606
       + 460 b_customer(11)(1)(2)#607 + 460 b_customer(11)(1)(3)#608
       + 460 b_customer(11)(1)(4)#609 + 460 b_customer(11)(1)(5)#610
       + 460 b_customer(11)(1)(6)#611 + 460 b_customer(11)(1)(7)#612
       + 460 b_customer(11)(1)(8)#613 + 460 b_customer(11)(1)(9)#614
       + 460 b_customer(11)(1)(10)#615 + 460 b_customer(11)(2)(1)#616
       + 460 b_customer(11)(2)(2)#617 + 460 b_customer(11)(2)(3)#618
       + 460 b_customer(11)(2)(4)#619 + 460 b_customer(11)(2)(5)#620
       + 460 b_customer(11)(2)(6)#621 + 460 b_customer(11)(2)(7)#622
       + 460 b_customer(11)(2)(8)#623 + 460 b_customer(11)(2)(9)#624
       + 460 b_customer(11)(2)(10)#625 + 460 b_customer(11)(3)(1)#626
       + 460 b_customer(11)(3)(2)#627 + 460 b_customer(11)(3)(3)#628
       + 460 b_customer(11)(3)(4)#629 + 460 b_customer(11)(3)(5)#630
       + 460 b_customer(11)(3)(6)#631 + 460 b_customer(11)(3)(7)#632
       + 460 b_customer(11)(3)(8)#633 + 460 b_customer(11)(3)(9)#634
       + 460 b_customer(11)(3)(10)#635 + 460 b_customer(12)(1)(1)#636
       + 460 b_customer(12)(1)(2)#637 + 460 b_customer(12)(1)(3)#638
       + 460 b_customer(12)(1)(4)#639 + 460 b_customer(12)(1)(5)#640
       + 460 b_customer(12)(1)(6)#641 + 460 b_customer(12)(1)(7)#642
       + 460 b_customer(12)(1)(8)#643 + 460 b_customer(12)(1)(9)#644
       + 460 b_customer(12)(1)(10)#645 + 460 b_customer(12)(2)(1)#646
       + 460 b_customer(12)(2)(2)#647 + 460 b_customer(12)(2)(3)#648
       + 460 b_customer(12)(2)(4)#649 + 460 b_customer(12)(2)(5)#650
       + 460 b_customer(12)(2)(6)#651 + 460 b_customer(12)(2)(7)#652
       + 460 b_customer(12)(2)(8)#653 + 460 b_customer(12)(2)(9)#654
       + 460 b_customer(12)(2)(10)#655 + 460 b_customer(12)(3)(1)#656
       + 460 b_customer(12)(3)(2)#657 + 460 b_customer(12)(3)(3)#658
       + 460 b_customer(12)(3)(4)#659 + 460 b_customer(12)(3)(5)#660
       + 460 b_customer(12)(3)(6)#661 + 460 b_customer(12)(3)(7)#662
       + 460 b_customer(12)(3)(8)#663 + 460 b_customer(12)(3)(9)#664
       + 460 b_customer(12)(3)(10)#665
Subject To
 ProductionCapacity(1):          p(1)#0 <= 270
 ProductionCapacity(2):          p(2)#1 <= 270
 ProductionCapacity(3):          p(3)#2 <= 0
 PlantInventoryCapacity(1):      I_plant(1)#3 <= 135
 PlantInventoryCapacity(2):      I_plant(2)#4 <= 135
 PlantInventoryCapacity(3):      I_plant(3)#5 <= 135
 PlantInventoryBalance(1):       - p(1)#0 + I_plant(1)#3 + q(1)(1)(1)#666
                                 + q(2)(1)(1)#669  = 0
 PlantInventoryBalance(2):       - p(2)#1 - I_plant(1)#3 + I_plant(2)#4
                                 + q(1)(1)(2)#667 + q(2)(1)(2)#670  = 0
 PlantInventoryBalance(3):       - p(3)#2 - I_plant(2)#4 + I_plant(3)#5
                                 + q(1)(1)(3)#668 + q(2)(1)(3)#671  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(1)(1)(1)#6 <= 124
 WarehouseInventoryCapacity(2,1,1): I_warehouse(2)(1)(1)#36 <= 124
 WarehouseInventoryCapacity(1,2,1): I_warehouse(1)(2)(1)#16 <= 124
 WarehouseInventoryCapacity(2,2,1): I_warehouse(2)(2)(1)#46 <= 124
 WarehouseInventoryCapacity(1,3,1): I_warehouse(1)(3)(1)#26 <= 124
 WarehouseInventoryCapacity(2,3,1): I_warehouse(2)(3)(1)#56 <= 124
 WarehouseInventoryCapacity(1,1,2): I_warehouse(1)(1)(2)#7 <= 124
 WarehouseInventoryCapacity(2,1,2): I_warehouse(2)(1)(2)#37 <= 124
 WarehouseInventoryCapacity(1,2,2): I_warehouse(1)(2)(2)#17 <= 124
 WarehouseInventoryCapacity(2,2,2): I_warehouse(2)(2)(2)#47 <= 124
 WarehouseInventoryCapacity(1,3,2): I_warehouse(1)(3)(2)#27 <= 124
 WarehouseInventoryCapacity(2,3,2): I_warehouse(2)(3)(2)#57 <= 124
 WarehouseInventoryCapacity(1,1,3): I_warehouse(1)(1)(3)#8 <= 124
 WarehouseInventoryCapacity(2,1,3): I_warehouse(2)(1)(3)#38 <= 124
 WarehouseInventoryCapacity(1,2,3): I_warehouse(1)(2)(3)#18 <= 124
 WarehouseInventoryCapacity(2,2,3): I_warehouse(2)(2)(3)#48 <= 124
 WarehouseInventoryCapacity(1,3,3): I_warehouse(1)(3)(3)#28 <= 124
 WarehouseInventoryCapacity(2,3,3): I_warehouse(2)(3)(3)#58 <= 124
 WarehouseInventoryCapacity(1,1,4): I_warehouse(1)(1)(4)#9 <= 124
 WarehouseInventoryCapacity(2,1,4): I_warehouse(2)(1)(4)#39 <= 124
 WarehouseInventoryCapacity(1,2,4): I_warehouse(1)(2)(4)#19 <= 124
 WarehouseInventoryCapacity(2,2,4): I_warehouse(2)(2)(4)#49 <= 124
 WarehouseInventoryCapacity(1,3,4): I_warehouse(1)(3)(4)#29 <= 124
 WarehouseInventoryCapacity(2,3,4): I_warehouse(2)(3)(4)#59 <= 124
 WarehouseInventoryCapacity(1,1,5): I_warehouse(1)(1)(5)#10 <= 124
 WarehouseInventoryCapacity(2,1,5): I_warehouse(2)(1)(5)#40 <= 124
 WarehouseInventoryCapacity(1,2,5): I_warehouse(1)(2)(5)#20 <= 124
 WarehouseInventoryCapacity(2,2,5): I_warehouse(2)(2)(5)#50 <= 124
 WarehouseInventoryCapacity(1,3,5): I_warehouse(1)(3)(5)#30 <= 124
 WarehouseInventoryCapacity(2,3,5): I_warehouse(2)(3)(5)#60 <= 124
 WarehouseInventoryCapacity(1,1,6): I_warehouse(1)(1)(6)#11 <= 124
 WarehouseInventoryCapacity(2,1,6): I_warehouse(2)(1)(6)#41 <= 124
 WarehouseInventoryCapacity(1,2,6): I_warehouse(1)(2)(6)#21 <= 124
 WarehouseInventoryCapacity(2,2,6): I_warehouse(2)(2)(6)#51 <= 124
 WarehouseInventoryCapacity(1,3,6): I_warehouse(1)(3)(6)#31 <= 124
 WarehouseInventoryCapacity(2,3,6): I_warehouse(2)(3)(6)#61 <= 124
 WarehouseInventoryCapacity(1,1,7): I_warehouse(1)(1)(7)#12 <= 124
 WarehouseInventoryCapacity(2,1,7): I_warehouse(2)(1)(7)#42 <= 124
 WarehouseInventoryCapacity(1,2,7): I_warehouse(1)(2)(7)#22 <= 124
 WarehouseInventoryCapacity(2,2,7): I_warehouse(2)(2)(7)#52 <= 124
 WarehouseInventoryCapacity(1,3,7): I_warehouse(1)(3)(7)#32 <= 124
 WarehouseInventoryCapacity(2,3,7): I_warehouse(2)(3)(7)#62 <= 124
 WarehouseInventoryCapacity(1,1,8): I_warehouse(1)(1)(8)#13 <= 124
 WarehouseInventoryCapacity(2,1,8): I_warehouse(2)(1)(8)#43 <= 124
 WarehouseInventoryCapacity(1,2,8): I_warehouse(1)(2)(8)#23 <= 124
 WarehouseInventoryCapacity(2,2,8): I_warehouse(2)(2)(8)#53 <= 124
 WarehouseInventoryCapacity(1,3,8): I_warehouse(1)(3)(8)#33 <= 124
 WarehouseInventoryCapacity(2,3,8): I_warehouse(2)(3)(8)#63 <= 124
 WarehouseInventoryCapacity(1,1,9): I_warehouse(1)(1)(9)#14 <= 124
 WarehouseInventoryCapacity(2,1,9): I_warehouse(2)(1)(9)#44 <= 124
 WarehouseInventoryCapacity(1,2,9): I_warehouse(1)(2)(9)#24 <= 124
 WarehouseInventoryCapacity(2,2,9): I_warehouse(2)(2)(9)#54 <= 124
 WarehouseInventoryCapacity(1,3,9): I_warehouse(1)(3)(9)#34 <= 124
 WarehouseInventoryCapacity(2,3,9): I_warehouse(2)(3)(9)#64 <= 124
 WarehouseInventoryCapacity(1,1,10): I_warehouse(1)(1)(10)#15 <= 124
 WarehouseInventoryCapacity(2,1,10): I_warehouse(2)(1)(10)#45 <= 124
 WarehouseInventoryCapacity(1,2,10): I_warehouse(1)(2)(10)#25 <= 124
 WarehouseInventoryCapacity(2,2,10): I_warehouse(2)(2)(10)#55 <= 124
 WarehouseInventoryCapacity(1,3,10): I_warehouse(1)(3)(10)#35 <= 124
 WarehouseInventoryCapacity(2,3,10): I_warehouse(2)(3)(10)#65 <= 124
 WarehouseInventoryBalance(1,1,1): I_warehouse(1)(1)(1)#6 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(1)#672 + w(4)(1)(1)(1)#732
                                 + w(5)(1)(1)(1)#792 + w(6)(1)(1)(1)#852
                                 + w(7)(1)(1)(1)#912 + w(8)(1)(1)(1)#972
                                 + w(9)(1)(1)(1)#1032 + w(10)(1)(1)(1)#1092
                                 + w(11)(1)(1)(1)#1152 + w(12)(1)(1)(1)#1212
                                  = 0
 WarehouseInventoryBalance(2,1,1): I_warehouse(2)(1)(1)#36 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(1)#702 + w(4)(2)(1)(1)#762
                                 + w(5)(2)(1)(1)#822 + w(6)(2)(1)(1)#882
                                 + w(7)(2)(1)(1)#942 + w(8)(2)(1)(1)#1002
                                 + w(9)(2)(1)(1)#1062 + w(10)(2)(1)(1)#1122
                                 + w(11)(2)(1)(1)#1182 + w(12)(2)(1)(1)#1242
                                  = 0
 WarehouseInventoryBalance(1,2,1): - I_warehouse(1)(1)(1)#6
                                 + I_warehouse(1)(2)(1)#16 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(1)#682 + w(4)(1)(2)(1)#742
                                 + w(5)(1)(2)(1)#802 + w(6)(1)(2)(1)#862
                                 + w(7)(1)(2)(1)#922 + w(8)(1)(2)(1)#982
                                 + w(9)(1)(2)(1)#1042 + w(10)(1)(2)(1)#1102
                                 + w(11)(1)(2)(1)#1162 + w(12)(1)(2)(1)#1222
                                  = 0
 WarehouseInventoryBalance(2,2,1): - I_warehouse(2)(1)(1)#36
                                 + I_warehouse(2)(2)(1)#46 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(1)#712 + w(4)(2)(2)(1)#772
                                 + w(5)(2)(2)(1)#832 + w(6)(2)(2)(1)#892
                                 + w(7)(2)(2)(1)#952 + w(8)(2)(2)(1)#1012
                                 + w(9)(2)(2)(1)#1072 + w(10)(2)(2)(1)#1132
                                 + w(11)(2)(2)(1)#1192 + w(12)(2)(2)(1)#1252
                                  = 0
 WarehouseInventoryBalance(1,3,1): - I_warehouse(1)(2)(1)#16
                                 + I_warehouse(1)(3)(1)#26 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(1)#692 + w(4)(1)(3)(1)#752
                                 + w(5)(1)(3)(1)#812 + w(6)(1)(3)(1)#872
                                 + w(7)(1)(3)(1)#932 + w(8)(1)(3)(1)#992
                                 + w(9)(1)(3)(1)#1052 + w(10)(1)(3)(1)#1112
                                 + w(11)(1)(3)(1)#1172 + w(12)(1)(3)(1)#1232
                                  = 0
 WarehouseInventoryBalance(2,3,1): - I_warehouse(2)(2)(1)#46
                                 + I_warehouse(2)(3)(1)#56 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(1)#722 + w(4)(2)(3)(1)#782
                                 + w(5)(2)(3)(1)#842 + w(6)(2)(3)(1)#902
                                 + w(7)(2)(3)(1)#962 + w(8)(2)(3)(1)#1022
                                 + w(9)(2)(3)(1)#1082 + w(10)(2)(3)(1)#1142
                                 + w(11)(2)(3)(1)#1202 + w(12)(2)(3)(1)#1262
                                  = 0
 WarehouseInventoryBalance(1,1,2): I_warehouse(1)(1)(2)#7 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(2)#673 + w(4)(1)(1)(2)#733
                                 + w(5)(1)(1)(2)#793 + w(6)(1)(1)(2)#853
                                 + w(7)(1)(1)(2)#913 + w(8)(1)(1)(2)#973
                                 + w(9)(1)(1)(2)#1033 + w(10)(1)(1)(2)#1093
                                 + w(11)(1)(1)(2)#1153 + w(12)(1)(1)(2)#1213
                                  = 0
 WarehouseInventoryBalance(2,1,2): I_warehouse(2)(1)(2)#37 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(2)#703 + w(4)(2)(1)(2)#763
                                 + w(5)(2)(1)(2)#823 + w(6)(2)(1)(2)#883
                                 + w(7)(2)(1)(2)#943 + w(8)(2)(1)(2)#1003
                                 + w(9)(2)(1)(2)#1063 + w(10)(2)(1)(2)#1123
                                 + w(11)(2)(1)(2)#1183 + w(12)(2)(1)(2)#1243
                                  = 0
 WarehouseInventoryBalance(1,2,2): - I_warehouse(1)(1)(2)#7
                                 + I_warehouse(1)(2)(2)#17 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(2)#683 + w(4)(1)(2)(2)#743
                                 + w(5)(1)(2)(2)#803 + w(6)(1)(2)(2)#863
                                 + w(7)(1)(2)(2)#923 + w(8)(1)(2)(2)#983
                                 + w(9)(1)(2)(2)#1043 + w(10)(1)(2)(2)#1103
                                 + w(11)(1)(2)(2)#1163 + w(12)(1)(2)(2)#1223
                                  = 0
 WarehouseInventoryBalance(2,2,2): - I_warehouse(2)(1)(2)#37
                                 + I_warehouse(2)(2)(2)#47 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(2)#713 + w(4)(2)(2)(2)#773
                                 + w(5)(2)(2)(2)#833 + w(6)(2)(2)(2)#893
                                 + w(7)(2)(2)(2)#953 + w(8)(2)(2)(2)#1013
                                 + w(9)(2)(2)(2)#1073 + w(10)(2)(2)(2)#1133
                                 + w(11)(2)(2)(2)#1193 + w(12)(2)(2)(2)#1253
                                  = 0
 WarehouseInventoryBalance(1,3,2): - I_warehouse(1)(2)(2)#17
                                 + I_warehouse(1)(3)(2)#27 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(2)#693 + w(4)(1)(3)(2)#753
                                 + w(5)(1)(3)(2)#813 + w(6)(1)(3)(2)#873
                                 + w(7)(1)(3)(2)#933 + w(8)(1)(3)(2)#993
                                 + w(9)(1)(3)(2)#1053 + w(10)(1)(3)(2)#1113
                                 + w(11)(1)(3)(2)#1173 + w(12)(1)(3)(2)#1233
                                  = 0
 WarehouseInventoryBalance(2,3,2): - I_warehouse(2)(2)(2)#47
                                 + I_warehouse(2)(3)(2)#57 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(2)#723 + w(4)(2)(3)(2)#783
                                 + w(5)(2)(3)(2)#843 + w(6)(2)(3)(2)#903
                                 + w(7)(2)(3)(2)#963 + w(8)(2)(3)(2)#1023
                                 + w(9)(2)(3)(2)#1083 + w(10)(2)(3)(2)#1143
                                 + w(11)(2)(3)(2)#1203 + w(12)(2)(3)(2)#1263
                                  = 0
 WarehouseInventoryBalance(1,1,3): I_warehouse(1)(1)(3)#8 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(3)#674 + w(4)(1)(1)(3)#734
                                 + w(5)(1)(1)(3)#794 + w(6)(1)(1)(3)#854
                                 + w(7)(1)(1)(3)#914 + w(8)(1)(1)(3)#974
                                 + w(9)(1)(1)(3)#1034 + w(10)(1)(1)(3)#1094
                                 + w(11)(1)(1)(3)#1154 + w(12)(1)(1)(3)#1214
                                  = 0
 WarehouseInventoryBalance(2,1,3): I_warehouse(2)(1)(3)#38 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(3)#704 + w(4)(2)(1)(3)#764
                                 + w(5)(2)(1)(3)#824 + w(6)(2)(1)(3)#884
                                 + w(7)(2)(1)(3)#944 + w(8)(2)(1)(3)#1004
                                 + w(9)(2)(1)(3)#1064 + w(10)(2)(1)(3)#1124
                                 + w(11)(2)(1)(3)#1184 + w(12)(2)(1)(3)#1244
                                  = 0
 WarehouseInventoryBalance(1,2,3): - I_warehouse(1)(1)(3)#8
                                 + I_warehouse(1)(2)(3)#18 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(3)#684 + w(4)(1)(2)(3)#744
                                 + w(5)(1)(2)(3)#804 + w(6)(1)(2)(3)#864
                                 + w(7)(1)(2)(3)#924 + w(8)(1)(2)(3)#984
                                 + w(9)(1)(2)(3)#1044 + w(10)(1)(2)(3)#1104
                                 + w(11)(1)(2)(3)#1164 + w(12)(1)(2)(3)#1224
                                  = 0
 WarehouseInventoryBalance(2,2,3): - I_warehouse(2)(1)(3)#38
                                 + I_warehouse(2)(2)(3)#48 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(3)#714 + w(4)(2)(2)(3)#774
                                 + w(5)(2)(2)(3)#834 + w(6)(2)(2)(3)#894
                                 + w(7)(2)(2)(3)#954 + w(8)(2)(2)(3)#1014
                                 + w(9)(2)(2)(3)#1074 + w(10)(2)(2)(3)#1134
                                 + w(11)(2)(2)(3)#1194 + w(12)(2)(2)(3)#1254
                                  = 0
 WarehouseInventoryBalance(1,3,3): - I_warehouse(1)(2)(3)#18
                                 + I_warehouse(1)(3)(3)#28 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(3)#694 + w(4)(1)(3)(3)#754
                                 + w(5)(1)(3)(3)#814 + w(6)(1)(3)(3)#874
                                 + w(7)(1)(3)(3)#934 + w(8)(1)(3)(3)#994
                                 + w(9)(1)(3)(3)#1054 + w(10)(1)(3)(3)#1114
                                 + w(11)(1)(3)(3)#1174 + w(12)(1)(3)(3)#1234
                                  = 0
 WarehouseInventoryBalance(2,3,3): - I_warehouse(2)(2)(3)#48
                                 + I_warehouse(2)(3)(3)#58 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(3)#724 + w(4)(2)(3)(3)#784
                                 + w(5)(2)(3)(3)#844 + w(6)(2)(3)(3)#904
                                 + w(7)(2)(3)(3)#964 + w(8)(2)(3)(3)#1024
                                 + w(9)(2)(3)(3)#1084 + w(10)(2)(3)(3)#1144
                                 + w(11)(2)(3)(3)#1204 + w(12)(2)(3)(3)#1264
                                  = 0
 WarehouseInventoryBalance(1,1,4): I_warehouse(1)(1)(4)#9 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(4)#675 + w(4)(1)(1)(4)#735
                                 + w(5)(1)(1)(4)#795 + w(6)(1)(1)(4)#855
                                 + w(7)(1)(1)(4)#915 + w(8)(1)(1)(4)#975
                                 + w(9)(1)(1)(4)#1035 + w(10)(1)(1)(4)#1095
                                 + w(11)(1)(1)(4)#1155 + w(12)(1)(1)(4)#1215
                                  = 0
 WarehouseInventoryBalance(2,1,4): I_warehouse(2)(1)(4)#39 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(4)#705 + w(4)(2)(1)(4)#765
                                 + w(5)(2)(1)(4)#825 + w(6)(2)(1)(4)#885
                                 + w(7)(2)(1)(4)#945 + w(8)(2)(1)(4)#1005
                                 + w(9)(2)(1)(4)#1065 + w(10)(2)(1)(4)#1125
                                 + w(11)(2)(1)(4)#1185 + w(12)(2)(1)(4)#1245
                                  = 0
 WarehouseInventoryBalance(1,2,4): - I_warehouse(1)(1)(4)#9
                                 + I_warehouse(1)(2)(4)#19 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(4)#685 + w(4)(1)(2)(4)#745
                                 + w(5)(1)(2)(4)#805 + w(6)(1)(2)(4)#865
                                 + w(7)(1)(2)(4)#925 + w(8)(1)(2)(4)#985
                                 + w(9)(1)(2)(4)#1045 + w(10)(1)(2)(4)#1105
                                 + w(11)(1)(2)(4)#1165 + w(12)(1)(2)(4)#1225
                                  = 0
 WarehouseInventoryBalance(2,2,4): - I_warehouse(2)(1)(4)#39
                                 + I_warehouse(2)(2)(4)#49 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(4)#715 + w(4)(2)(2)(4)#775
                                 + w(5)(2)(2)(4)#835 + w(6)(2)(2)(4)#895
                                 + w(7)(2)(2)(4)#955 + w(8)(2)(2)(4)#1015
                                 + w(9)(2)(2)(4)#1075 + w(10)(2)(2)(4)#1135
                                 + w(11)(2)(2)(4)#1195 + w(12)(2)(2)(4)#1255
                                  = 0
 WarehouseInventoryBalance(1,3,4): - I_warehouse(1)(2)(4)#19
                                 + I_warehouse(1)(3)(4)#29 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(4)#695 + w(4)(1)(3)(4)#755
                                 + w(5)(1)(3)(4)#815 + w(6)(1)(3)(4)#875
                                 + w(7)(1)(3)(4)#935 + w(8)(1)(3)(4)#995
                                 + w(9)(1)(3)(4)#1055 + w(10)(1)(3)(4)#1115
                                 + w(11)(1)(3)(4)#1175 + w(12)(1)(3)(4)#1235
                                  = 0
 WarehouseInventoryBalance(2,3,4): - I_warehouse(2)(2)(4)#49
                                 + I_warehouse(2)(3)(4)#59 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(4)#725 + w(4)(2)(3)(4)#785
                                 + w(5)(2)(3)(4)#845 + w(6)(2)(3)(4)#905
                                 + w(7)(2)(3)(4)#965 + w(8)(2)(3)(4)#1025
                                 + w(9)(2)(3)(4)#1085 + w(10)(2)(3)(4)#1145
                                 + w(11)(2)(3)(4)#1205 + w(12)(2)(3)(4)#1265
                                  = 0
 WarehouseInventoryBalance(1,1,5): I_warehouse(1)(1)(5)#10 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(5)#676 + w(4)(1)(1)(5)#736
                                 + w(5)(1)(1)(5)#796 + w(6)(1)(1)(5)#856
                                 + w(7)(1)(1)(5)#916 + w(8)(1)(1)(5)#976
                                 + w(9)(1)(1)(5)#1036 + w(10)(1)(1)(5)#1096
                                 + w(11)(1)(1)(5)#1156 + w(12)(1)(1)(5)#1216
                                  = 0
 WarehouseInventoryBalance(2,1,5): I_warehouse(2)(1)(5)#40 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(5)#706 + w(4)(2)(1)(5)#766
                                 + w(5)(2)(1)(5)#826 + w(6)(2)(1)(5)#886
                                 + w(7)(2)(1)(5)#946 + w(8)(2)(1)(5)#1006
                                 + w(9)(2)(1)(5)#1066 + w(10)(2)(1)(5)#1126
                                 + w(11)(2)(1)(5)#1186 + w(12)(2)(1)(5)#1246
                                  = 0
 WarehouseInventoryBalance(1,2,5): - I_warehouse(1)(1)(5)#10
                                 + I_warehouse(1)(2)(5)#20 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(5)#686 + w(4)(1)(2)(5)#746
                                 + w(5)(1)(2)(5)#806 + w(6)(1)(2)(5)#866
                                 + w(7)(1)(2)(5)#926 + w(8)(1)(2)(5)#986
                                 + w(9)(1)(2)(5)#1046 + w(10)(1)(2)(5)#1106
                                 + w(11)(1)(2)(5)#1166 + w(12)(1)(2)(5)#1226
                                  = 0
 WarehouseInventoryBalance(2,2,5): - I_warehouse(2)(1)(5)#40
                                 + I_warehouse(2)(2)(5)#50 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(5)#716 + w(4)(2)(2)(5)#776
                                 + w(5)(2)(2)(5)#836 + w(6)(2)(2)(5)#896
                                 + w(7)(2)(2)(5)#956 + w(8)(2)(2)(5)#1016
                                 + w(9)(2)(2)(5)#1076 + w(10)(2)(2)(5)#1136
                                 + w(11)(2)(2)(5)#1196 + w(12)(2)(2)(5)#1256
                                  = 0
 WarehouseInventoryBalance(1,3,5): - I_warehouse(1)(2)(5)#20
                                 + I_warehouse(1)(3)(5)#30 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(5)#696 + w(4)(1)(3)(5)#756
                                 + w(5)(1)(3)(5)#816 + w(6)(1)(3)(5)#876
                                 + w(7)(1)(3)(5)#936 + w(8)(1)(3)(5)#996
                                 + w(9)(1)(3)(5)#1056 + w(10)(1)(3)(5)#1116
                                 + w(11)(1)(3)(5)#1176 + w(12)(1)(3)(5)#1236
                                  = 0
 WarehouseInventoryBalance(2,3,5): - I_warehouse(2)(2)(5)#50
                                 + I_warehouse(2)(3)(5)#60 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(5)#726 + w(4)(2)(3)(5)#786
                                 + w(5)(2)(3)(5)#846 + w(6)(2)(3)(5)#906
                                 + w(7)(2)(3)(5)#966 + w(8)(2)(3)(5)#1026
                                 + w(9)(2)(3)(5)#1086 + w(10)(2)(3)(5)#1146
                                 + w(11)(2)(3)(5)#1206 + w(12)(2)(3)(5)#1266
                                  = 0
 WarehouseInventoryBalance(1,1,6): I_warehouse(1)(1)(6)#11 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(6)#677 + w(4)(1)(1)(6)#737
                                 + w(5)(1)(1)(6)#797 + w(6)(1)(1)(6)#857
                                 + w(7)(1)(1)(6)#917 + w(8)(1)(1)(6)#977
                                 + w(9)(1)(1)(6)#1037 + w(10)(1)(1)(6)#1097
                                 + w(11)(1)(1)(6)#1157 + w(12)(1)(1)(6)#1217
                                  = 0
 WarehouseInventoryBalance(2,1,6): I_warehouse(2)(1)(6)#41 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(6)#707 + w(4)(2)(1)(6)#767
                                 + w(5)(2)(1)(6)#827 + w(6)(2)(1)(6)#887
                                 + w(7)(2)(1)(6)#947 + w(8)(2)(1)(6)#1007
                                 + w(9)(2)(1)(6)#1067 + w(10)(2)(1)(6)#1127
                                 + w(11)(2)(1)(6)#1187 + w(12)(2)(1)(6)#1247
                                  = 0
 WarehouseInventoryBalance(1,2,6): - I_warehouse(1)(1)(6)#11
                                 + I_warehouse(1)(2)(6)#21 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(6)#687 + w(4)(1)(2)(6)#747
                                 + w(5)(1)(2)(6)#807 + w(6)(1)(2)(6)#867
                                 + w(7)(1)(2)(6)#927 + w(8)(1)(2)(6)#987
                                 + w(9)(1)(2)(6)#1047 + w(10)(1)(2)(6)#1107
                                 + w(11)(1)(2)(6)#1167 + w(12)(1)(2)(6)#1227
                                  = 0
 WarehouseInventoryBalance(2,2,6): - I_warehouse(2)(1)(6)#41
                                 + I_warehouse(2)(2)(6)#51 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(6)#717 + w(4)(2)(2)(6)#777
                                 + w(5)(2)(2)(6)#837 + w(6)(2)(2)(6)#897
                                 + w(7)(2)(2)(6)#957 + w(8)(2)(2)(6)#1017
                                 + w(9)(2)(2)(6)#1077 + w(10)(2)(2)(6)#1137
                                 + w(11)(2)(2)(6)#1197 + w(12)(2)(2)(6)#1257
                                  = 0
 WarehouseInventoryBalance(1,3,6): - I_warehouse(1)(2)(6)#21
                                 + I_warehouse(1)(3)(6)#31 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(6)#697 + w(4)(1)(3)(6)#757
                                 + w(5)(1)(3)(6)#817 + w(6)(1)(3)(6)#877
                                 + w(7)(1)(3)(6)#937 + w(8)(1)(3)(6)#997
                                 + w(9)(1)(3)(6)#1057 + w(10)(1)(3)(6)#1117
                                 + w(11)(1)(3)(6)#1177 + w(12)(1)(3)(6)#1237
                                  = 0
 WarehouseInventoryBalance(2,3,6): - I_warehouse(2)(2)(6)#51
                                 + I_warehouse(2)(3)(6)#61 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(6)#727 + w(4)(2)(3)(6)#787
                                 + w(5)(2)(3)(6)#847 + w(6)(2)(3)(6)#907
                                 + w(7)(2)(3)(6)#967 + w(8)(2)(3)(6)#1027
                                 + w(9)(2)(3)(6)#1087 + w(10)(2)(3)(6)#1147
                                 + w(11)(2)(3)(6)#1207 + w(12)(2)(3)(6)#1267
                                  = 0
 WarehouseInventoryBalance(1,1,7): I_warehouse(1)(1)(7)#12 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(7)#678 + w(4)(1)(1)(7)#738
                                 + w(5)(1)(1)(7)#798 + w(6)(1)(1)(7)#858
                                 + w(7)(1)(1)(7)#918 + w(8)(1)(1)(7)#978
                                 + w(9)(1)(1)(7)#1038 + w(10)(1)(1)(7)#1098
                                 + w(11)(1)(1)(7)#1158 + w(12)(1)(1)(7)#1218
                                  = 0
 WarehouseInventoryBalance(2,1,7): I_warehouse(2)(1)(7)#42 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(7)#708 + w(4)(2)(1)(7)#768
                                 + w(5)(2)(1)(7)#828 + w(6)(2)(1)(7)#888
                                 + w(7)(2)(1)(7)#948 + w(8)(2)(1)(7)#1008
                                 + w(9)(2)(1)(7)#1068 + w(10)(2)(1)(7)#1128
                                 + w(11)(2)(1)(7)#1188 + w(12)(2)(1)(7)#1248
                                  = 0
 WarehouseInventoryBalance(1,2,7): - I_warehouse(1)(1)(7)#12
                                 + I_warehouse(1)(2)(7)#22 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(7)#688 + w(4)(1)(2)(7)#748
                                 + w(5)(1)(2)(7)#808 + w(6)(1)(2)(7)#868
                                 + w(7)(1)(2)(7)#928 + w(8)(1)(2)(7)#988
                                 + w(9)(1)(2)(7)#1048 + w(10)(1)(2)(7)#1108
                                 + w(11)(1)(2)(7)#1168 + w(12)(1)(2)(7)#1228
                                  = 0
 WarehouseInventoryBalance(2,2,7): - I_warehouse(2)(1)(7)#42
                                 + I_warehouse(2)(2)(7)#52 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(7)#718 + w(4)(2)(2)(7)#778
                                 + w(5)(2)(2)(7)#838 + w(6)(2)(2)(7)#898
                                 + w(7)(2)(2)(7)#958 + w(8)(2)(2)(7)#1018
                                 + w(9)(2)(2)(7)#1078 + w(10)(2)(2)(7)#1138
                                 + w(11)(2)(2)(7)#1198 + w(12)(2)(2)(7)#1258
                                  = 0
 WarehouseInventoryBalance(1,3,7): - I_warehouse(1)(2)(7)#22
                                 + I_warehouse(1)(3)(7)#32 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(7)#698 + w(4)(1)(3)(7)#758
                                 + w(5)(1)(3)(7)#818 + w(6)(1)(3)(7)#878
                                 + w(7)(1)(3)(7)#938 + w(8)(1)(3)(7)#998
                                 + w(9)(1)(3)(7)#1058 + w(10)(1)(3)(7)#1118
                                 + w(11)(1)(3)(7)#1178 + w(12)(1)(3)(7)#1238
                                  = 0
 WarehouseInventoryBalance(2,3,7): - I_warehouse(2)(2)(7)#52
                                 + I_warehouse(2)(3)(7)#62 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(7)#728 + w(4)(2)(3)(7)#788
                                 + w(5)(2)(3)(7)#848 + w(6)(2)(3)(7)#908
                                 + w(7)(2)(3)(7)#968 + w(8)(2)(3)(7)#1028
                                 + w(9)(2)(3)(7)#1088 + w(10)(2)(3)(7)#1148
                                 + w(11)(2)(3)(7)#1208 + w(12)(2)(3)(7)#1268
                                  = 0
 WarehouseInventoryBalance(1,1,8): I_warehouse(1)(1)(8)#13 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(8)#679 + w(4)(1)(1)(8)#739
                                 + w(5)(1)(1)(8)#799 + w(6)(1)(1)(8)#859
                                 + w(7)(1)(1)(8)#919 + w(8)(1)(1)(8)#979
                                 + w(9)(1)(1)(8)#1039 + w(10)(1)(1)(8)#1099
                                 + w(11)(1)(1)(8)#1159 + w(12)(1)(1)(8)#1219
                                  = 0
 WarehouseInventoryBalance(2,1,8): I_warehouse(2)(1)(8)#43 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(8)#709 + w(4)(2)(1)(8)#769
                                 + w(5)(2)(1)(8)#829 + w(6)(2)(1)(8)#889
                                 + w(7)(2)(1)(8)#949 + w(8)(2)(1)(8)#1009
                                 + w(9)(2)(1)(8)#1069 + w(10)(2)(1)(8)#1129
                                 + w(11)(2)(1)(8)#1189 + w(12)(2)(1)(8)#1249
                                  = 0
 WarehouseInventoryBalance(1,2,8): - I_warehouse(1)(1)(8)#13
                                 + I_warehouse(1)(2)(8)#23 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(8)#689 + w(4)(1)(2)(8)#749
                                 + w(5)(1)(2)(8)#809 + w(6)(1)(2)(8)#869
                                 + w(7)(1)(2)(8)#929 + w(8)(1)(2)(8)#989
                                 + w(9)(1)(2)(8)#1049 + w(10)(1)(2)(8)#1109
                                 + w(11)(1)(2)(8)#1169 + w(12)(1)(2)(8)#1229
                                  = 0
 WarehouseInventoryBalance(2,2,8): - I_warehouse(2)(1)(8)#43
                                 + I_warehouse(2)(2)(8)#53 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(8)#719 + w(4)(2)(2)(8)#779
                                 + w(5)(2)(2)(8)#839 + w(6)(2)(2)(8)#899
                                 + w(7)(2)(2)(8)#959 + w(8)(2)(2)(8)#1019
                                 + w(9)(2)(2)(8)#1079 + w(10)(2)(2)(8)#1139
                                 + w(11)(2)(2)(8)#1199 + w(12)(2)(2)(8)#1259
                                  = 0
 WarehouseInventoryBalance(1,3,8): - I_warehouse(1)(2)(8)#23
                                 + I_warehouse(1)(3)(8)#33 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(8)#699 + w(4)(1)(3)(8)#759
                                 + w(5)(1)(3)(8)#819 + w(6)(1)(3)(8)#879
                                 + w(7)(1)(3)(8)#939 + w(8)(1)(3)(8)#999
                                 + w(9)(1)(3)(8)#1059 + w(10)(1)(3)(8)#1119
                                 + w(11)(1)(3)(8)#1179 + w(12)(1)(3)(8)#1239
                                  = 0
 WarehouseInventoryBalance(2,3,8): - I_warehouse(2)(2)(8)#53
                                 + I_warehouse(2)(3)(8)#63 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(8)#729 + w(4)(2)(3)(8)#789
                                 + w(5)(2)(3)(8)#849 + w(6)(2)(3)(8)#909
                                 + w(7)(2)(3)(8)#969 + w(8)(2)(3)(8)#1029
                                 + w(9)(2)(3)(8)#1089 + w(10)(2)(3)(8)#1149
                                 + w(11)(2)(3)(8)#1209 + w(12)(2)(3)(8)#1269
                                  = 0
 WarehouseInventoryBalance(1,1,9): I_warehouse(1)(1)(9)#14 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(9)#680 + w(4)(1)(1)(9)#740
                                 + w(5)(1)(1)(9)#800 + w(6)(1)(1)(9)#860
                                 + w(7)(1)(1)(9)#920 + w(8)(1)(1)(9)#980
                                 + w(9)(1)(1)(9)#1040 + w(10)(1)(1)(9)#1100
                                 + w(11)(1)(1)(9)#1160 + w(12)(1)(1)(9)#1220
                                  = 0
 WarehouseInventoryBalance(2,1,9): I_warehouse(2)(1)(9)#44 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(9)#710 + w(4)(2)(1)(9)#770
                                 + w(5)(2)(1)(9)#830 + w(6)(2)(1)(9)#890
                                 + w(7)(2)(1)(9)#950 + w(8)(2)(1)(9)#1010
                                 + w(9)(2)(1)(9)#1070 + w(10)(2)(1)(9)#1130
                                 + w(11)(2)(1)(9)#1190 + w(12)(2)(1)(9)#1250
                                  = 0
 WarehouseInventoryBalance(1,2,9): - I_warehouse(1)(1)(9)#14
                                 + I_warehouse(1)(2)(9)#24 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(9)#690 + w(4)(1)(2)(9)#750
                                 + w(5)(1)(2)(9)#810 + w(6)(1)(2)(9)#870
                                 + w(7)(1)(2)(9)#930 + w(8)(1)(2)(9)#990
                                 + w(9)(1)(2)(9)#1050 + w(10)(1)(2)(9)#1110
                                 + w(11)(1)(2)(9)#1170 + w(12)(1)(2)(9)#1230
                                  = 0
 WarehouseInventoryBalance(2,2,9): - I_warehouse(2)(1)(9)#44
                                 + I_warehouse(2)(2)(9)#54 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(9)#720 + w(4)(2)(2)(9)#780
                                 + w(5)(2)(2)(9)#840 + w(6)(2)(2)(9)#900
                                 + w(7)(2)(2)(9)#960 + w(8)(2)(2)(9)#1020
                                 + w(9)(2)(2)(9)#1080 + w(10)(2)(2)(9)#1140
                                 + w(11)(2)(2)(9)#1200 + w(12)(2)(2)(9)#1260
                                  = 0
 WarehouseInventoryBalance(1,3,9): - I_warehouse(1)(2)(9)#24
                                 + I_warehouse(1)(3)(9)#34 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(9)#700 + w(4)(1)(3)(9)#760
                                 + w(5)(1)(3)(9)#820 + w(6)(1)(3)(9)#880
                                 + w(7)(1)(3)(9)#940 + w(8)(1)(3)(9)#1000
                                 + w(9)(1)(3)(9)#1060 + w(10)(1)(3)(9)#1120
                                 + w(11)(1)(3)(9)#1180 + w(12)(1)(3)(9)#1240
                                  = 0
 WarehouseInventoryBalance(2,3,9): - I_warehouse(2)(2)(9)#54
                                 + I_warehouse(2)(3)(9)#64 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(9)#730 + w(4)(2)(3)(9)#790
                                 + w(5)(2)(3)(9)#850 + w(6)(2)(3)(9)#910
                                 + w(7)(2)(3)(9)#970 + w(8)(2)(3)(9)#1030
                                 + w(9)(2)(3)(9)#1090 + w(10)(2)(3)(9)#1150
                                 + w(11)(2)(3)(9)#1210 + w(12)(2)(3)(9)#1270
                                  = 0
 WarehouseInventoryBalance(1,1,10): I_warehouse(1)(1)(10)#15 - q(1)(1)(1)#666
                                 + w(3)(1)(1)(10)#681 + w(4)(1)(1)(10)#741
                                 + w(5)(1)(1)(10)#801 + w(6)(1)(1)(10)#861
                                 + w(7)(1)(1)(10)#921 + w(8)(1)(1)(10)#981
                                 + w(9)(1)(1)(10)#1041 + w(10)(1)(1)(10)#1101
                                 + w(11)(1)(1)(10)#1161 + w(12)(1)(1)(10)#1221
                                  = 0
 WarehouseInventoryBalance(2,1,10): I_warehouse(2)(1)(10)#45 - q(2)(1)(1)#669
                                 + w(3)(2)(1)(10)#711 + w(4)(2)(1)(10)#771
                                 + w(5)(2)(1)(10)#831 + w(6)(2)(1)(10)#891
                                 + w(7)(2)(1)(10)#951 + w(8)(2)(1)(10)#1011
                                 + w(9)(2)(1)(10)#1071 + w(10)(2)(1)(10)#1131
                                 + w(11)(2)(1)(10)#1191 + w(12)(2)(1)(10)#1251
                                  = 0
 WarehouseInventoryBalance(1,2,10): - I_warehouse(1)(1)(10)#15
                                 + I_warehouse(1)(2)(10)#25 - q(1)(1)(2)#667
                                 + w(3)(1)(2)(10)#691 + w(4)(1)(2)(10)#751
                                 + w(5)(1)(2)(10)#811 + w(6)(1)(2)(10)#871
                                 + w(7)(1)(2)(10)#931 + w(8)(1)(2)(10)#991
                                 + w(9)(1)(2)(10)#1051 + w(10)(1)(2)(10)#1111
                                 + w(11)(1)(2)(10)#1171 + w(12)(1)(2)(10)#1231
                                  = 0
 WarehouseInventoryBalance(2,2,10): - I_warehouse(2)(1)(10)#45
                                 + I_warehouse(2)(2)(10)#55 - q(2)(1)(2)#670
                                 + w(3)(2)(2)(10)#721 + w(4)(2)(2)(10)#781
                                 + w(5)(2)(2)(10)#841 + w(6)(2)(2)(10)#901
                                 + w(7)(2)(2)(10)#961 + w(8)(2)(2)(10)#1021
                                 + w(9)(2)(2)(10)#1081 + w(10)(2)(2)(10)#1141
                                 + w(11)(2)(2)(10)#1201 + w(12)(2)(2)(10)#1261
                                  = 0
 WarehouseInventoryBalance(1,3,10): - I_warehouse(1)(2)(10)#25
                                 + I_warehouse(1)(3)(10)#35 - q(1)(1)(3)#668
                                 + w(3)(1)(3)(10)#701 + w(4)(1)(3)(10)#761
                                 + w(5)(1)(3)(10)#821 + w(6)(1)(3)(10)#881
                                 + w(7)(1)(3)(10)#941 + w(8)(1)(3)(10)#1001
                                 + w(9)(1)(3)(10)#1061 + w(10)(1)(3)(10)#1121
                                 + w(11)(1)(3)(10)#1181 + w(12)(1)(3)(10)#1241
                                  = 0
 WarehouseInventoryBalance(2,3,10): - I_warehouse(2)(2)(10)#55
                                 + I_warehouse(2)(3)(10)#65 - q(2)(1)(3)#671
                                 + w(3)(2)(3)(10)#731 + w(4)(2)(3)(10)#791
                                 + w(5)(2)(3)(10)#851 + w(6)(2)(3)(10)#911
                                 + w(7)(2)(3)(10)#971 + w(8)(2)(3)(10)#1031
                                 + w(9)(2)(3)(10)#1091 + w(10)(2)(3)(10)#1151
                                 + w(11)(2)(3)(10)#1211 + w(12)(2)(3)(10)#1271
                                  = 0
 CustomerInventoryBalance(3,1,1): I_customer(3)(1)(1)#66
                                 - b_customer(3)(1)(1)#366 - w(3)(1)(1)(1)#672
                                 - w(3)(2)(1)(1)#702  = 17
 CustomerInventoryBalance(4,1,1): I_customer(4)(1)(1)#96
                                 - b_customer(4)(1)(1)#396 - w(4)(1)(1)(1)#732
                                 - w(4)(2)(1)(1)#762  = -1
 CustomerInventoryBalance(5,1,1): I_customer(5)(1)(1)#126
                                 - b_customer(5)(1)(1)#426 - w(5)(1)(1)(1)#792
                                 - w(5)(2)(1)(1)#822  = 1
 CustomerInventoryBalance(6,1,1): I_customer(6)(1)(1)#156
                                 - b_customer(6)(1)(1)#456 - w(6)(1)(1)(1)#852
                                 - w(6)(2)(1)(1)#882  = 3
 CustomerInventoryBalance(7,1,1): I_customer(7)(1)(1)#186
                                 - b_customer(7)(1)(1)#486 - w(7)(1)(1)(1)#912
                                 - w(7)(2)(1)(1)#942  = 6
 CustomerInventoryBalance(8,1,1): I_customer(8)(1)(1)#216
                                 - b_customer(8)(1)(1)#516 - w(8)(1)(1)(1)#972
                                 - w(8)(2)(1)(1)#1002  = 3
 CustomerInventoryBalance(9,1,1): I_customer(9)(1)(1)#246
                                 - b_customer(9)(1)(1)#546 - w(9)(1)(1)(1)#1032
                                 - w(9)(2)(1)(1)#1062  = 13
 CustomerInventoryBalance(10,1,1): I_customer(10)(1)(1)#276
                                 - b_customer(10)(1)(1)#576
                                 - w(10)(1)(1)(1)#1092 - w(10)(2)(1)(1)#1122
                                  = 2
 CustomerInventoryBalance(11,1,1): I_customer(11)(1)(1)#306
                                 - b_customer(11)(1)(1)#606
                                 - w(11)(1)(1)(1)#1152 - w(11)(2)(1)(1)#1182
                                  = 3
 CustomerInventoryBalance(12,1,1): I_customer(12)(1)(1)#336
                                 - b_customer(12)(1)(1)#636
                                 - w(12)(1)(1)(1)#1212 - w(12)(2)(1)(1)#1242
                                  = 15
 CustomerInventoryBalance(3,2,1): - I_customer(3)(1)(1)#66
                                 + I_customer(3)(2)(1)#76
                                 - b_customer(3)(2)(1)#376 - w(3)(1)(2)(1)#682
                                 - w(3)(2)(2)(1)#712  = -17
 CustomerInventoryBalance(4,2,1): - I_customer(4)(1)(1)#96
                                 + I_customer(4)(2)(1)#106
                                 - b_customer(4)(2)(1)#406 - w(4)(1)(2)(1)#742
                                 - w(4)(2)(2)(1)#772  = -6
 CustomerInventoryBalance(5,2,1): - I_customer(5)(1)(1)#126
                                 + I_customer(5)(2)(1)#136
                                 - b_customer(5)(2)(1)#436 - w(5)(1)(2)(1)#802
                                 - w(5)(2)(2)(1)#832  = -6
 CustomerInventoryBalance(6,2,1): - I_customer(6)(1)(1)#156
                                 + I_customer(6)(2)(1)#166
                                 - b_customer(6)(2)(1)#466 - w(6)(1)(2)(1)#862
                                 - w(6)(2)(2)(1)#892  = -17
 CustomerInventoryBalance(7,2,1): - I_customer(7)(1)(1)#186
                                 + I_customer(7)(2)(1)#196
                                 - b_customer(7)(2)(1)#496 - w(7)(1)(2)(1)#922
                                 - w(7)(2)(2)(1)#952  = -8
 CustomerInventoryBalance(8,2,1): - I_customer(8)(1)(1)#216
                                 + I_customer(8)(2)(1)#226
                                 - b_customer(8)(2)(1)#526 - w(8)(1)(2)(1)#982
                                 - w(8)(2)(2)(1)#1012  = -20
 CustomerInventoryBalance(9,2,1): - I_customer(9)(1)(1)#246
                                 + I_customer(9)(2)(1)#256
                                 - b_customer(9)(2)(1)#556 - w(9)(1)(2)(1)#1042
                                 - w(9)(2)(2)(1)#1072  = -15
 CustomerInventoryBalance(10,2,1): - I_customer(10)(1)(1)#276
                                 + I_customer(10)(2)(1)#286
                                 - b_customer(10)(2)(1)#586
                                 - w(10)(1)(2)(1)#1102 - w(10)(2)(2)(1)#1132
                                  = -14
 CustomerInventoryBalance(11,2,1): - I_customer(11)(1)(1)#306
                                 + I_customer(11)(2)(1)#316
                                 - b_customer(11)(2)(1)#616
                                 - w(11)(1)(2)(1)#1162 - w(11)(2)(2)(1)#1192
                                  = -13
 CustomerInventoryBalance(12,2,1): - I_customer(12)(1)(1)#336
                                 + I_customer(12)(2)(1)#346
                                 - b_customer(12)(2)(1)#646
                                 - w(12)(1)(2)(1)#1222 - w(12)(2)(2)(1)#1252
                                  = -14
 CustomerInventoryBalance(3,3,1): - I_customer(3)(2)(1)#76
                                 + I_customer(3)(3)(1)#86
                                 - b_customer(3)(3)(1)#386 - w(3)(1)(3)(1)#692
                                 - w(3)(2)(3)(1)#722  = -22
 CustomerInventoryBalance(4,3,1): - I_customer(4)(2)(1)#106
                                 + I_customer(4)(3)(1)#116
                                 - b_customer(4)(3)(1)#416 - w(4)(1)(3)(1)#752
                                 - w(4)(2)(3)(1)#782  = -8
 CustomerInventoryBalance(5,3,1): - I_customer(5)(2)(1)#136
                                 + I_customer(5)(3)(1)#146
                                 - b_customer(5)(3)(1)#446 - w(5)(1)(3)(1)#812
                                 - w(5)(2)(3)(1)#842  = -6
 CustomerInventoryBalance(6,3,1): - I_customer(6)(2)(1)#166
                                 + I_customer(6)(3)(1)#176
                                 - b_customer(6)(3)(1)#476 - w(6)(1)(3)(1)#872
                                 - w(6)(2)(3)(1)#902  = -17
 CustomerInventoryBalance(7,3,1): - I_customer(7)(2)(1)#196
                                 + I_customer(7)(3)(1)#206
                                 - b_customer(7)(3)(1)#506 - w(7)(1)(3)(1)#932
                                 - w(7)(2)(3)(1)#962  = -6
 CustomerInventoryBalance(8,3,1): - I_customer(8)(2)(1)#226
                                 + I_customer(8)(3)(1)#236
                                 - b_customer(8)(3)(1)#536 - w(8)(1)(3)(1)#992
                                 - w(8)(2)(3)(1)#1022  = -19
 CustomerInventoryBalance(9,3,1): - I_customer(9)(2)(1)#256
                                 + I_customer(9)(3)(1)#266
                                 - b_customer(9)(3)(1)#566 - w(9)(1)(3)(1)#1052
                                 - w(9)(2)(3)(1)#1082  = -17
 CustomerInventoryBalance(10,3,1): - I_customer(10)(2)(1)#286
                                 + I_customer(10)(3)(1)#296
                                 - b_customer(10)(3)(1)#596
                                 - w(10)(1)(3)(1)#1112 - w(10)(2)(3)(1)#1142
                                  = -16
 CustomerInventoryBalance(11,3,1): - I_customer(11)(2)(1)#316
                                 + I_customer(11)(3)(1)#326
                                 - b_customer(11)(3)(1)#626
                                 - w(11)(1)(3)(1)#1172 - w(11)(2)(3)(1)#1202
                                  = -14
 CustomerInventoryBalance(12,3,1): - I_customer(12)(2)(1)#346
                                 + I_customer(12)(3)(1)#356
                                 - b_customer(12)(3)(1)#656
                                 - w(12)(1)(3)(1)#1232 - w(12)(2)(3)(1)#1262
                                  = -16
 CustomerInventoryBalance(3,1,2): I_customer(3)(1)(2)#67
                                 - b_customer(3)(1)(2)#367 - w(3)(1)(1)(2)#673
                                 - w(3)(2)(1)(2)#703  = 16
 CustomerInventoryBalance(4,1,2): I_customer(4)(1)(2)#97
                                 - b_customer(4)(1)(2)#397 - w(4)(1)(1)(2)#733
                                 - w(4)(2)(1)(2)#763  = 1
 CustomerInventoryBalance(5,1,2): I_customer(5)(1)(2)#127
                                 - b_customer(5)(1)(2)#427 - w(5)(1)(1)(2)#793
                                 - w(5)(2)(1)(2)#823  = 1
 CustomerInventoryBalance(6,1,2): I_customer(6)(1)(2)#157
                                 - b_customer(6)(1)(2)#457 - w(6)(1)(1)(2)#853
                                 - w(6)(2)(1)(2)#883  = -1
 CustomerInventoryBalance(7,1,2): I_customer(7)(1)(2)#187
                                 - b_customer(7)(1)(2)#487 - w(7)(1)(1)(2)#913
                                 - w(7)(2)(1)(2)#943  = 8
 CustomerInventoryBalance(8,1,2): I_customer(8)(1)(2)#217
                                 - b_customer(8)(1)(2)#517 - w(8)(1)(1)(2)#973
                                 - w(8)(2)(1)(2)#1003  = 0
 CustomerInventoryBalance(9,1,2): I_customer(9)(1)(2)#247
                                 - b_customer(9)(1)(2)#547 - w(9)(1)(1)(2)#1033
                                 - w(9)(2)(1)(2)#1063  = 19
 CustomerInventoryBalance(10,1,2): I_customer(10)(1)(2)#277
                                 - b_customer(10)(1)(2)#577
                                 - w(10)(1)(1)(2)#1093 - w(10)(2)(1)(2)#1123
                                  = 2
 CustomerInventoryBalance(11,1,2): I_customer(11)(1)(2)#307
                                 - b_customer(11)(1)(2)#607
                                 - w(11)(1)(1)(2)#1153 - w(11)(2)(1)(2)#1183
                                  = -3
 CustomerInventoryBalance(12,1,2): I_customer(12)(1)(2)#337
                                 - b_customer(12)(1)(2)#637
                                 - w(12)(1)(1)(2)#1213 - w(12)(2)(1)(2)#1243
                                  = 17
 CustomerInventoryBalance(3,2,2): - I_customer(3)(1)(2)#67
                                 + I_customer(3)(2)(2)#77
                                 - b_customer(3)(2)(2)#377 - w(3)(1)(2)(2)#683
                                 - w(3)(2)(2)(2)#713  = -20
 CustomerInventoryBalance(4,2,2): - I_customer(4)(1)(2)#97
                                 + I_customer(4)(2)(2)#107
                                 - b_customer(4)(2)(2)#407 - w(4)(1)(2)(2)#743
                                 - w(4)(2)(2)(2)#773  = -8
 CustomerInventoryBalance(5,2,2): - I_customer(5)(1)(2)#127
                                 + I_customer(5)(2)(2)#137
                                 - b_customer(5)(2)(2)#437 - w(5)(1)(2)(2)#803
                                 - w(5)(2)(2)(2)#833  = -7
 CustomerInventoryBalance(6,2,2): - I_customer(6)(1)(2)#157
                                 + I_customer(6)(2)(2)#167
                                 - b_customer(6)(2)(2)#467 - w(6)(1)(2)(2)#863
                                 - w(6)(2)(2)(2)#893  = -16
 CustomerInventoryBalance(7,2,2): - I_customer(7)(1)(2)#187
                                 + I_customer(7)(2)(2)#197
                                 - b_customer(7)(2)(2)#497 - w(7)(1)(2)(2)#923
                                 - w(7)(2)(2)(2)#953  = -7
 CustomerInventoryBalance(8,2,2): - I_customer(8)(1)(2)#217
                                 + I_customer(8)(2)(2)#227
                                 - b_customer(8)(2)(2)#527 - w(8)(1)(2)(2)#983
                                 - w(8)(2)(2)(2)#1013  = -17
 CustomerInventoryBalance(9,2,2): - I_customer(9)(1)(2)#247
                                 + I_customer(9)(2)(2)#257
                                 - b_customer(9)(2)(2)#557 - w(9)(1)(2)(2)#1043
                                 - w(9)(2)(2)(2)#1073  = -14
 CustomerInventoryBalance(10,2,2): - I_customer(10)(1)(2)#277
                                 + I_customer(10)(2)(2)#287
                                 - b_customer(10)(2)(2)#587
                                 - w(10)(1)(2)(2)#1103 - w(10)(2)(2)(2)#1133
                                  = -18
 CustomerInventoryBalance(11,2,2): - I_customer(11)(1)(2)#307
                                 + I_customer(11)(2)(2)#317
                                 - b_customer(11)(2)(2)#617
                                 - w(11)(1)(2)(2)#1163 - w(11)(2)(2)(2)#1193
                                  = -16
 CustomerInventoryBalance(12,2,2): - I_customer(12)(1)(2)#337
                                 + I_customer(12)(2)(2)#347
                                 - b_customer(12)(2)(2)#647
                                 - w(12)(1)(2)(2)#1223 - w(12)(2)(2)(2)#1253
                                  = -14
 CustomerInventoryBalance(3,3,2): - I_customer(3)(2)(2)#77
                                 + I_customer(3)(3)(2)#87
                                 - b_customer(3)(3)(2)#387 - w(3)(1)(3)(2)#693
                                 - w(3)(2)(3)(2)#723  = -16
 CustomerInventoryBalance(4,3,2): - I_customer(4)(2)(2)#107
                                 + I_customer(4)(3)(2)#117
                                 - b_customer(4)(3)(2)#417 - w(4)(1)(3)(2)#753
                                 - w(4)(2)(3)(2)#783  = -7
 CustomerInventoryBalance(5,3,2): - I_customer(5)(2)(2)#137
                                 + I_customer(5)(3)(2)#147
                                 - b_customer(5)(3)(2)#447 - w(5)(1)(3)(2)#813
                                 - w(5)(2)(3)(2)#843  = -7
 CustomerInventoryBalance(6,3,2): - I_customer(6)(2)(2)#167
                                 + I_customer(6)(3)(2)#177
                                 - b_customer(6)(3)(2)#477 - w(6)(1)(3)(2)#873
                                 - w(6)(2)(3)(2)#903  = -17
 CustomerInventoryBalance(7,3,2): - I_customer(7)(2)(2)#197
                                 + I_customer(7)(3)(2)#207
                                 - b_customer(7)(3)(2)#507 - w(7)(1)(3)(2)#933
                                 - w(7)(2)(3)(2)#963  = -6
 CustomerInventoryBalance(8,3,2): - I_customer(8)(2)(2)#227
                                 + I_customer(8)(3)(2)#237
                                 - b_customer(8)(3)(2)#537 - w(8)(1)(3)(2)#993
                                 - w(8)(2)(3)(2)#1023  = -19
 CustomerInventoryBalance(9,3,2): - I_customer(9)(2)(2)#257
                                 + I_customer(9)(3)(2)#267
                                 - b_customer(9)(3)(2)#567 - w(9)(1)(3)(2)#1053
                                 - w(9)(2)(3)(2)#1083  = -13
 CustomerInventoryBalance(10,3,2): - I_customer(10)(2)(2)#287
                                 + I_customer(10)(3)(2)#297
                                 - b_customer(10)(3)(2)#597
                                 - w(10)(1)(3)(2)#1113 - w(10)(2)(3)(2)#1143
                                  = -17
 CustomerInventoryBalance(11,3,2): - I_customer(11)(2)(2)#317
                                 + I_customer(11)(3)(2)#327
                                 - b_customer(11)(3)(2)#627
                                 - w(11)(1)(3)(2)#1173 - w(11)(2)(3)(2)#1203
                                  = -18
 CustomerInventoryBalance(12,3,2): - I_customer(12)(2)(2)#347
                                 + I_customer(12)(3)(2)#357
                                 - b_customer(12)(3)(2)#657
                                 - w(12)(1)(3)(2)#1233 - w(12)(2)(3)(2)#1263
                                  = -16
 CustomerInventoryBalance(3,1,3): I_customer(3)(1)(3)#68
                                 - b_customer(3)(1)(3)#368 - w(3)(1)(1)(3)#674
                                 - w(3)(2)(1)(3)#704  = 17
 CustomerInventoryBalance(4,1,3): I_customer(4)(1)(3)#98
                                 - b_customer(4)(1)(3)#398 - w(4)(1)(1)(3)#734
                                 - w(4)(2)(1)(3)#764  = -1
 CustomerInventoryBalance(5,1,3): I_customer(5)(1)(3)#128
                                 - b_customer(5)(1)(3)#428 - w(5)(1)(1)(3)#794
                                 - w(5)(2)(1)(3)#824  = 0
 CustomerInventoryBalance(6,1,3): I_customer(6)(1)(3)#158
                                 - b_customer(6)(1)(3)#458 - w(6)(1)(1)(3)#854
                                 - w(6)(2)(1)(3)#884  = 0
 CustomerInventoryBalance(7,1,3): I_customer(7)(1)(3)#188
                                 - b_customer(7)(1)(3)#488 - w(7)(1)(1)(3)#914
                                 - w(7)(2)(1)(3)#944  = 6
 CustomerInventoryBalance(8,1,3): I_customer(8)(1)(3)#218
                                 - b_customer(8)(1)(3)#518 - w(8)(1)(1)(3)#974
                                 - w(8)(2)(1)(3)#1004  = -3
 CustomerInventoryBalance(9,1,3): I_customer(9)(1)(3)#248
                                 - b_customer(9)(1)(3)#548 - w(9)(1)(1)(3)#1034
                                 - w(9)(2)(1)(3)#1064  = 18
 CustomerInventoryBalance(10,1,3): I_customer(10)(1)(3)#278
                                 - b_customer(10)(1)(3)#578
                                 - w(10)(1)(1)(3)#1094 - w(10)(2)(1)(3)#1124
                                  = -2
 CustomerInventoryBalance(11,1,3): I_customer(11)(1)(3)#308
                                 - b_customer(11)(1)(3)#608
                                 - w(11)(1)(1)(3)#1154 - w(11)(2)(1)(3)#1184
                                  = 1
 CustomerInventoryBalance(12,1,3): I_customer(12)(1)(3)#338
                                 - b_customer(12)(1)(3)#638
                                 - w(12)(1)(1)(3)#1214 - w(12)(2)(1)(3)#1244
                                  = 15
 CustomerInventoryBalance(3,2,3): - I_customer(3)(1)(3)#68
                                 + I_customer(3)(2)(3)#78
                                 - b_customer(3)(2)(3)#378 - w(3)(1)(2)(3)#684
                                 - w(3)(2)(2)(3)#714  = -20
 CustomerInventoryBalance(4,2,3): - I_customer(4)(1)(3)#98
                                 + I_customer(4)(2)(3)#108
                                 - b_customer(4)(2)(3)#408 - w(4)(1)(2)(3)#744
                                 - w(4)(2)(2)(3)#774  = -8
 CustomerInventoryBalance(5,2,3): - I_customer(5)(1)(3)#128
                                 + I_customer(5)(2)(3)#138
                                 - b_customer(5)(2)(3)#438 - w(5)(1)(2)(3)#804
                                 - w(5)(2)(2)(3)#834  = -7
 CustomerInventoryBalance(6,2,3): - I_customer(6)(1)(3)#158
                                 + I_customer(6)(2)(3)#168
                                 - b_customer(6)(2)(3)#468 - w(6)(1)(2)(3)#864
                                 - w(6)(2)(2)(3)#894  = -15
 CustomerInventoryBalance(7,2,3): - I_customer(7)(1)(3)#188
                                 + I_customer(7)(2)(3)#198
                                 - b_customer(7)(2)(3)#498 - w(7)(1)(2)(3)#924
                                 - w(7)(2)(2)(3)#954  = -7
 CustomerInventoryBalance(8,2,3): - I_customer(8)(1)(3)#218
                                 + I_customer(8)(2)(3)#228
                                 - b_customer(8)(2)(3)#528 - w(8)(1)(2)(3)#984
                                 - w(8)(2)(2)(3)#1014  = -20
 CustomerInventoryBalance(9,2,3): - I_customer(9)(1)(3)#248
                                 + I_customer(9)(2)(3)#258
                                 - b_customer(9)(2)(3)#558 - w(9)(1)(2)(3)#1044
                                 - w(9)(2)(2)(3)#1074  = -16
 CustomerInventoryBalance(10,2,3): - I_customer(10)(1)(3)#278
                                 + I_customer(10)(2)(3)#288
                                 - b_customer(10)(2)(3)#588
                                 - w(10)(1)(2)(3)#1104 - w(10)(2)(2)(3)#1134
                                  = -13
 CustomerInventoryBalance(11,2,3): - I_customer(11)(1)(3)#308
                                 + I_customer(11)(2)(3)#318
                                 - b_customer(11)(2)(3)#618
                                 - w(11)(1)(2)(3)#1164 - w(11)(2)(2)(3)#1194
                                  = -18
 CustomerInventoryBalance(12,2,3): - I_customer(12)(1)(3)#338
                                 + I_customer(12)(2)(3)#348
                                 - b_customer(12)(2)(3)#648
                                 - w(12)(1)(2)(3)#1224 - w(12)(2)(2)(3)#1254
                                  = -13
 CustomerInventoryBalance(3,3,3): - I_customer(3)(2)(3)#78
                                 + I_customer(3)(3)(3)#88
                                 - b_customer(3)(3)(3)#388 - w(3)(1)(3)(3)#694
                                 - w(3)(2)(3)(3)#724  = -20
 CustomerInventoryBalance(4,3,3): - I_customer(4)(2)(3)#108
                                 + I_customer(4)(3)(3)#118
                                 - b_customer(4)(3)(3)#418 - w(4)(1)(3)(3)#754
                                 - w(4)(2)(3)(3)#784  = -7
 CustomerInventoryBalance(5,3,3): - I_customer(5)(2)(3)#138
                                 + I_customer(5)(3)(3)#148
                                 - b_customer(5)(3)(3)#448 - w(5)(1)(3)(3)#814
                                 - w(5)(2)(3)(3)#844  = -5
 CustomerInventoryBalance(6,3,3): - I_customer(6)(2)(3)#168
                                 + I_customer(6)(3)(3)#178
                                 - b_customer(6)(3)(3)#478 - w(6)(1)(3)(3)#874
                                 - w(6)(2)(3)(3)#904  = -16
 CustomerInventoryBalance(7,3,3): - I_customer(7)(2)(3)#198
                                 + I_customer(7)(3)(3)#208
                                 - b_customer(7)(3)(3)#508 - w(7)(1)(3)(3)#934
                                 - w(7)(2)(3)(3)#964  = -7
 CustomerInventoryBalance(8,3,3): - I_customer(8)(2)(3)#228
                                 + I_customer(8)(3)(3)#238
                                 - b_customer(8)(3)(3)#538 - w(8)(1)(3)(3)#994
                                 - w(8)(2)(3)(3)#1024  = -21
 CustomerInventoryBalance(9,3,3): - I_customer(9)(2)(3)#258
                                 + I_customer(9)(3)(3)#268
                                 - b_customer(9)(3)(3)#568 - w(9)(1)(3)(3)#1054
                                 - w(9)(2)(3)(3)#1084  = -19
 CustomerInventoryBalance(10,3,3): - I_customer(10)(2)(3)#288
                                 + I_customer(10)(3)(3)#298
                                 - b_customer(10)(3)(3)#598
                                 - w(10)(1)(3)(3)#1114 - w(10)(2)(3)(3)#1144
                                  = -19
 CustomerInventoryBalance(11,3,3): - I_customer(11)(2)(3)#318
                                 + I_customer(11)(3)(3)#328
                                 - b_customer(11)(3)(3)#628
                                 - w(11)(1)(3)(3)#1174 - w(11)(2)(3)(3)#1204
                                  = -17
 CustomerInventoryBalance(12,3,3): - I_customer(12)(2)(3)#348
                                 + I_customer(12)(3)(3)#358
                                 - b_customer(12)(3)(3)#658
                                 - w(12)(1)(3)(3)#1234 - w(12)(2)(3)(3)#1264
                                  = -13
 CustomerInventoryBalance(3,1,4): I_customer(3)(1)(4)#69
                                 - b_customer(3)(1)(4)#369 - w(3)(1)(1)(4)#675
                                 - w(3)(2)(1)(4)#705  = 17
 CustomerInventoryBalance(4,1,4): I_customer(4)(1)(4)#99
                                 - b_customer(4)(1)(4)#399 - w(4)(1)(1)(4)#735
                                 - w(4)(2)(1)(4)#765  = -1
 CustomerInventoryBalance(5,1,4): I_customer(5)(1)(4)#129
                                 - b_customer(5)(1)(4)#429 - w(5)(1)(1)(4)#795
                                 - w(5)(2)(1)(4)#825  = -1
 CustomerInventoryBalance(6,1,4): I_customer(6)(1)(4)#159
                                 - b_customer(6)(1)(4)#459 - w(6)(1)(1)(4)#855
                                 - w(6)(2)(1)(4)#885  = -1
 CustomerInventoryBalance(7,1,4): I_customer(7)(1)(4)#189
                                 - b_customer(7)(1)(4)#489 - w(7)(1)(1)(4)#915
                                 - w(7)(2)(1)(4)#945  = 6
 CustomerInventoryBalance(8,1,4): I_customer(8)(1)(4)#219
                                 - b_customer(8)(1)(4)#519 - w(8)(1)(1)(4)#975
                                 - w(8)(2)(1)(4)#1005  = 2
 CustomerInventoryBalance(9,1,4): I_customer(9)(1)(4)#249
                                 - b_customer(9)(1)(4)#549 - w(9)(1)(1)(4)#1035
                                 - w(9)(2)(1)(4)#1065  = 16
 CustomerInventoryBalance(10,1,4): I_customer(10)(1)(4)#279
                                 - b_customer(10)(1)(4)#579
                                 - w(10)(1)(1)(4)#1095 - w(10)(2)(1)(4)#1125
                                  = -2
 CustomerInventoryBalance(11,1,4): I_customer(11)(1)(4)#309
                                 - b_customer(11)(1)(4)#609
                                 - w(11)(1)(1)(4)#1155 - w(11)(2)(1)(4)#1185
                                  = 1
 CustomerInventoryBalance(12,1,4): I_customer(12)(1)(4)#339
                                 - b_customer(12)(1)(4)#639
                                 - w(12)(1)(1)(4)#1215 - w(12)(2)(1)(4)#1245
                                  = 13
 CustomerInventoryBalance(3,2,4): - I_customer(3)(1)(4)#69
                                 + I_customer(3)(2)(4)#79
                                 - b_customer(3)(2)(4)#379 - w(3)(1)(2)(4)#685
                                 - w(3)(2)(2)(4)#715  = -20
 CustomerInventoryBalance(4,2,4): - I_customer(4)(1)(4)#99
                                 + I_customer(4)(2)(4)#109
                                 - b_customer(4)(2)(4)#409 - w(4)(1)(2)(4)#745
                                 - w(4)(2)(2)(4)#775  = -7
 CustomerInventoryBalance(5,2,4): - I_customer(5)(1)(4)#129
                                 + I_customer(5)(2)(4)#139
                                 - b_customer(5)(2)(4)#439 - w(5)(1)(2)(4)#805
                                 - w(5)(2)(2)(4)#835  = -6
 CustomerInventoryBalance(6,2,4): - I_customer(6)(1)(4)#159
                                 + I_customer(6)(2)(4)#169
                                 - b_customer(6)(2)(4)#469 - w(6)(1)(2)(4)#865
                                 - w(6)(2)(2)(4)#895  = -14
 CustomerInventoryBalance(7,2,4): - I_customer(7)(1)(4)#189
                                 + I_customer(7)(2)(4)#199
                                 - b_customer(7)(2)(4)#499 - w(7)(1)(2)(4)#925
                                 - w(7)(2)(2)(4)#955  = -6
 CustomerInventoryBalance(8,2,4): - I_customer(8)(1)(4)#219
                                 + I_customer(8)(2)(4)#229
                                 - b_customer(8)(2)(4)#529 - w(8)(1)(2)(4)#985
                                 - w(8)(2)(2)(4)#1015  = -17
 CustomerInventoryBalance(9,2,4): - I_customer(9)(1)(4)#249
                                 + I_customer(9)(2)(4)#259
                                 - b_customer(9)(2)(4)#559 - w(9)(1)(2)(4)#1045
                                 - w(9)(2)(2)(4)#1075  = -17
 CustomerInventoryBalance(10,2,4): - I_customer(10)(1)(4)#279
                                 + I_customer(10)(2)(4)#289
                                 - b_customer(10)(2)(4)#589
                                 - w(10)(1)(2)(4)#1105 - w(10)(2)(2)(4)#1135
                                  = -19
 CustomerInventoryBalance(11,2,4): - I_customer(11)(1)(4)#309
                                 + I_customer(11)(2)(4)#319
                                 - b_customer(11)(2)(4)#619
                                 - w(11)(1)(2)(4)#1165 - w(11)(2)(2)(4)#1195
                                  = -19
 CustomerInventoryBalance(12,2,4): - I_customer(12)(1)(4)#339
                                 + I_customer(12)(2)(4)#349
                                 - b_customer(12)(2)(4)#649
                                 - w(12)(1)(2)(4)#1225 - w(12)(2)(2)(4)#1255
                                  = -15
 CustomerInventoryBalance(3,3,4): - I_customer(3)(2)(4)#79
                                 + I_customer(3)(3)(4)#89
                                 - b_customer(3)(3)(4)#389 - w(3)(1)(3)(4)#695
                                 - w(3)(2)(3)(4)#725  = -21
 CustomerInventoryBalance(4,3,4): - I_customer(4)(2)(4)#109
                                 + I_customer(4)(3)(4)#119
                                 - b_customer(4)(3)(4)#419 - w(4)(1)(3)(4)#755
                                 - w(4)(2)(3)(4)#785  = -6
 CustomerInventoryBalance(5,3,4): - I_customer(5)(2)(4)#139
                                 + I_customer(5)(3)(4)#149
                                 - b_customer(5)(3)(4)#449 - w(5)(1)(3)(4)#815
                                 - w(5)(2)(3)(4)#845  = -6
 CustomerInventoryBalance(6,3,4): - I_customer(6)(2)(4)#169
                                 + I_customer(6)(3)(4)#179
                                 - b_customer(6)(3)(4)#479 - w(6)(1)(3)(4)#875
                                 - w(6)(2)(3)(4)#905  = -14
 CustomerInventoryBalance(7,3,4): - I_customer(7)(2)(4)#199
                                 + I_customer(7)(3)(4)#209
                                 - b_customer(7)(3)(4)#509 - w(7)(1)(3)(4)#935
                                 - w(7)(2)(3)(4)#965  = -6
 CustomerInventoryBalance(8,3,4): - I_customer(8)(2)(4)#229
                                 + I_customer(8)(3)(4)#239
                                 - b_customer(8)(3)(4)#539 - w(8)(1)(3)(4)#995
                                 - w(8)(2)(3)(4)#1025  = -16
 CustomerInventoryBalance(9,3,4): - I_customer(9)(2)(4)#259
                                 + I_customer(9)(3)(4)#269
                                 - b_customer(9)(3)(4)#569 - w(9)(1)(3)(4)#1055
                                 - w(9)(2)(3)(4)#1085  = -16
 CustomerInventoryBalance(10,3,4): - I_customer(10)(2)(4)#289
                                 + I_customer(10)(3)(4)#299
                                 - b_customer(10)(3)(4)#599
                                 - w(10)(1)(3)(4)#1115 - w(10)(2)(3)(4)#1145
                                  = -18
 CustomerInventoryBalance(11,3,4): - I_customer(11)(2)(4)#319
                                 + I_customer(11)(3)(4)#329
                                 - b_customer(11)(3)(4)#629
                                 - w(11)(1)(3)(4)#1175 - w(11)(2)(3)(4)#1205
                                  = -14
 CustomerInventoryBalance(12,3,4): - I_customer(12)(2)(4)#349
                                 + I_customer(12)(3)(4)#359
                                 - b_customer(12)(3)(4)#659
                                 - w(12)(1)(3)(4)#1235 - w(12)(2)(3)(4)#1265
                                  = -12
 CustomerInventoryBalance(3,1,5): I_customer(3)(1)(5)#70
                                 - b_customer(3)(1)(5)#370 - w(3)(1)(1)(5)#676
                                 - w(3)(2)(1)(5)#706  = 20
 CustomerInventoryBalance(4,1,5): I_customer(4)(1)(5)#100
                                 - b_customer(4)(1)(5)#400 - w(4)(1)(1)(5)#736
                                 - w(4)(2)(1)(5)#766  = -1
 CustomerInventoryBalance(5,1,5): I_customer(5)(1)(5)#130
                                 - b_customer(5)(1)(5)#430 - w(5)(1)(1)(5)#796
                                 - w(5)(2)(1)(5)#826  = -1
 CustomerInventoryBalance(6,1,5): I_customer(6)(1)(5)#160
                                 - b_customer(6)(1)(5)#460 - w(6)(1)(1)(5)#856
                                 - w(6)(2)(1)(5)#886  = -1
 CustomerInventoryBalance(7,1,5): I_customer(7)(1)(5)#190
                                 - b_customer(7)(1)(5)#490 - w(7)(1)(1)(5)#916
                                 - w(7)(2)(1)(5)#946  = 6
 CustomerInventoryBalance(8,1,5): I_customer(8)(1)(5)#220
                                 - b_customer(8)(1)(5)#520 - w(8)(1)(1)(5)#976
                                 - w(8)(2)(1)(5)#1006  = -3
 CustomerInventoryBalance(9,1,5): I_customer(9)(1)(5)#250
                                 - b_customer(9)(1)(5)#550 - w(9)(1)(1)(5)#1036
                                 - w(9)(2)(1)(5)#1066  = 13
 CustomerInventoryBalance(10,1,5): I_customer(10)(1)(5)#280
                                 - b_customer(10)(1)(5)#580
                                 - w(10)(1)(1)(5)#1096 - w(10)(2)(1)(5)#1126
                                  = 3
 CustomerInventoryBalance(11,1,5): I_customer(11)(1)(5)#310
                                 - b_customer(11)(1)(5)#610
                                 - w(11)(1)(1)(5)#1156 - w(11)(2)(1)(5)#1186
                                  = -3
 CustomerInventoryBalance(12,1,5): I_customer(12)(1)(5)#340
                                 - b_customer(12)(1)(5)#640
                                 - w(12)(1)(1)(5)#1216 - w(12)(2)(1)(5)#1246
                                  = 14
 CustomerInventoryBalance(3,2,5): - I_customer(3)(1)(5)#70
                                 + I_customer(3)(2)(5)#80
                                 - b_customer(3)(2)(5)#380 - w(3)(1)(2)(5)#686
                                 - w(3)(2)(2)(5)#716  = -22
 CustomerInventoryBalance(4,2,5): - I_customer(4)(1)(5)#100
                                 + I_customer(4)(2)(5)#110
                                 - b_customer(4)(2)(5)#410 - w(4)(1)(2)(5)#746
                                 - w(4)(2)(2)(5)#776  = -8
 CustomerInventoryBalance(5,2,5): - I_customer(5)(1)(5)#130
                                 + I_customer(5)(2)(5)#140
                                 - b_customer(5)(2)(5)#440 - w(5)(1)(2)(5)#806
                                 - w(5)(2)(2)(5)#836  = -5
 CustomerInventoryBalance(6,2,5): - I_customer(6)(1)(5)#160
                                 + I_customer(6)(2)(5)#170
                                 - b_customer(6)(2)(5)#470 - w(6)(1)(2)(5)#866
                                 - w(6)(2)(2)(5)#896  = -15
 CustomerInventoryBalance(7,2,5): - I_customer(7)(1)(5)#190
                                 + I_customer(7)(2)(5)#200
                                 - b_customer(7)(2)(5)#500 - w(7)(1)(2)(5)#926
                                 - w(7)(2)(2)(5)#956  = -6
 CustomerInventoryBalance(8,2,5): - I_customer(8)(1)(5)#220
                                 + I_customer(8)(2)(5)#230
                                 - b_customer(8)(2)(5)#530 - w(8)(1)(2)(5)#986
                                 - w(8)(2)(2)(5)#1016  = -15
 CustomerInventoryBalance(9,2,5): - I_customer(9)(1)(5)#250
                                 + I_customer(9)(2)(5)#260
                                 - b_customer(9)(2)(5)#560 - w(9)(1)(2)(5)#1046
                                 - w(9)(2)(2)(5)#1076  = -19
 CustomerInventoryBalance(10,2,5): - I_customer(10)(1)(5)#280
                                 + I_customer(10)(2)(5)#290
                                 - b_customer(10)(2)(5)#590
                                 - w(10)(1)(2)(5)#1106 - w(10)(2)(2)(5)#1136
                                  = -16
 CustomerInventoryBalance(11,2,5): - I_customer(11)(1)(5)#310
                                 + I_customer(11)(2)(5)#320
                                 - b_customer(11)(2)(5)#620
                                 - w(11)(1)(2)(5)#1166 - w(11)(2)(2)(5)#1196
                                  = -18
 CustomerInventoryBalance(12,2,5): - I_customer(12)(1)(5)#340
                                 + I_customer(12)(2)(5)#350
                                 - b_customer(12)(2)(5)#650
                                 - w(12)(1)(2)(5)#1226 - w(12)(2)(2)(5)#1256
                                  = -12
 CustomerInventoryBalance(3,3,5): - I_customer(3)(2)(5)#80
                                 + I_customer(3)(3)(5)#90
                                 - b_customer(3)(3)(5)#390 - w(3)(1)(3)(5)#696
                                 - w(3)(2)(3)(5)#726  = -17
 CustomerInventoryBalance(4,3,5): - I_customer(4)(2)(5)#110
                                 + I_customer(4)(3)(5)#120
                                 - b_customer(4)(3)(5)#420 - w(4)(1)(3)(5)#756
                                 - w(4)(2)(3)(5)#786  = -7
 CustomerInventoryBalance(5,3,5): - I_customer(5)(2)(5)#140
                                 + I_customer(5)(3)(5)#150
                                 - b_customer(5)(3)(5)#450 - w(5)(1)(3)(5)#816
                                 - w(5)(2)(3)(5)#846  = -6
 CustomerInventoryBalance(6,3,5): - I_customer(6)(2)(5)#170
                                 + I_customer(6)(3)(5)#180
                                 - b_customer(6)(3)(5)#480 - w(6)(1)(3)(5)#876
                                 - w(6)(2)(3)(5)#906  = -18
 CustomerInventoryBalance(7,3,5): - I_customer(7)(2)(5)#200
                                 + I_customer(7)(3)(5)#210
                                 - b_customer(7)(3)(5)#510 - w(7)(1)(3)(5)#936
                                 - w(7)(2)(3)(5)#966  = -7
 CustomerInventoryBalance(8,3,5): - I_customer(8)(2)(5)#230
                                 + I_customer(8)(3)(5)#240
                                 - b_customer(8)(3)(5)#540 - w(8)(1)(3)(5)#996
                                 - w(8)(2)(3)(5)#1026  = -15
 CustomerInventoryBalance(9,3,5): - I_customer(9)(2)(5)#260
                                 + I_customer(9)(3)(5)#270
                                 - b_customer(9)(3)(5)#570 - w(9)(1)(3)(5)#1056
                                 - w(9)(2)(3)(5)#1086  = -17
 CustomerInventoryBalance(10,3,5): - I_customer(10)(2)(5)#290
                                 + I_customer(10)(3)(5)#300
                                 - b_customer(10)(3)(5)#600
                                 - w(10)(1)(3)(5)#1116 - w(10)(2)(3)(5)#1146
                                  = -14
 CustomerInventoryBalance(11,3,5): - I_customer(11)(2)(5)#320
                                 + I_customer(11)(3)(5)#330
                                 - b_customer(11)(3)(5)#630
                                 - w(11)(1)(3)(5)#1176 - w(11)(2)(3)(5)#1206
                                  = -14
 CustomerInventoryBalance(12,3,5): - I_customer(12)(2)(5)#350
                                 + I_customer(12)(3)(5)#360
                                 - b_customer(12)(3)(5)#660
                                 - w(12)(1)(3)(5)#1236 - w(12)(2)(3)(5)#1266
                                  = -12
 CustomerInventoryBalance(3,1,6): I_customer(3)(1)(6)#71
                                 - b_customer(3)(1)(6)#371 - w(3)(1)(1)(6)#677
                                 - w(3)(2)(1)(6)#707  = 16
 CustomerInventoryBalance(4,1,6): I_customer(4)(1)(6)#101
                                 - b_customer(4)(1)(6)#401 - w(4)(1)(1)(6)#737
                                 - w(4)(2)(1)(6)#767  = 1
 CustomerInventoryBalance(5,1,6): I_customer(5)(1)(6)#131
                                 - b_customer(5)(1)(6)#431 - w(5)(1)(1)(6)#797
                                 - w(5)(2)(1)(6)#827  = 0
 CustomerInventoryBalance(6,1,6): I_customer(6)(1)(6)#161
                                 - b_customer(6)(1)(6)#461 - w(6)(1)(1)(6)#857
                                 - w(6)(2)(1)(6)#887  = -1
 CustomerInventoryBalance(7,1,6): I_customer(7)(1)(6)#191
                                 - b_customer(7)(1)(6)#491 - w(7)(1)(1)(6)#917
                                 - w(7)(2)(1)(6)#947  = 7
 CustomerInventoryBalance(8,1,6): I_customer(8)(1)(6)#221
                                 - b_customer(8)(1)(6)#521 - w(8)(1)(1)(6)#977
                                 - w(8)(2)(1)(6)#1007  = 1
 CustomerInventoryBalance(9,1,6): I_customer(9)(1)(6)#251
                                 - b_customer(9)(1)(6)#551 - w(9)(1)(1)(6)#1037
                                 - w(9)(2)(1)(6)#1067  = 13
 CustomerInventoryBalance(10,1,6): I_customer(10)(1)(6)#281
                                 - b_customer(10)(1)(6)#581
                                 - w(10)(1)(1)(6)#1097 - w(10)(2)(1)(6)#1127
                                  = 0
 CustomerInventoryBalance(11,1,6): I_customer(11)(1)(6)#311
                                 - b_customer(11)(1)(6)#611
                                 - w(11)(1)(1)(6)#1157 - w(11)(2)(1)(6)#1187
                                  = 2
 CustomerInventoryBalance(12,1,6): I_customer(12)(1)(6)#341
                                 - b_customer(12)(1)(6)#641
                                 - w(12)(1)(1)(6)#1217 - w(12)(2)(1)(6)#1247
                                  = 16
 CustomerInventoryBalance(3,2,6): - I_customer(3)(1)(6)#71
                                 + I_customer(3)(2)(6)#81
                                 - b_customer(3)(2)(6)#381 - w(3)(1)(2)(6)#687
                                 - w(3)(2)(2)(6)#717  = -19
 CustomerInventoryBalance(4,2,6): - I_customer(4)(1)(6)#101
                                 + I_customer(4)(2)(6)#111
                                 - b_customer(4)(2)(6)#411 - w(4)(1)(2)(6)#747
                                 - w(4)(2)(2)(6)#777  = -6
 CustomerInventoryBalance(5,2,6): - I_customer(5)(1)(6)#131
                                 + I_customer(5)(2)(6)#141
                                 - b_customer(5)(2)(6)#441 - w(5)(1)(2)(6)#807
                                 - w(5)(2)(2)(6)#837  = -7
 CustomerInventoryBalance(6,2,6): - I_customer(6)(1)(6)#161
                                 + I_customer(6)(2)(6)#171
                                 - b_customer(6)(2)(6)#471 - w(6)(1)(2)(6)#867
                                 - w(6)(2)(2)(6)#897  = -15
 CustomerInventoryBalance(7,2,6): - I_customer(7)(1)(6)#191
                                 + I_customer(7)(2)(6)#201
                                 - b_customer(7)(2)(6)#501 - w(7)(1)(2)(6)#927
                                 - w(7)(2)(2)(6)#957  = -6
 CustomerInventoryBalance(8,2,6): - I_customer(8)(1)(6)#221
                                 + I_customer(8)(2)(6)#231
                                 - b_customer(8)(2)(6)#531 - w(8)(1)(2)(6)#987
                                 - w(8)(2)(2)(6)#1017  = -21
 CustomerInventoryBalance(9,2,6): - I_customer(9)(1)(6)#251
                                 + I_customer(9)(2)(6)#261
                                 - b_customer(9)(2)(6)#561 - w(9)(1)(2)(6)#1047
                                 - w(9)(2)(2)(6)#1077  = -14
 CustomerInventoryBalance(10,2,6): - I_customer(10)(1)(6)#281
                                 + I_customer(10)(2)(6)#291
                                 - b_customer(10)(2)(6)#591
                                 - w(10)(1)(2)(6)#1107 - w(10)(2)(2)(6)#1137
                                  = -18
 CustomerInventoryBalance(11,2,6): - I_customer(11)(1)(6)#311
                                 + I_customer(11)(2)(6)#321
                                 - b_customer(11)(2)(6)#621
                                 - w(11)(1)(2)(6)#1167 - w(11)(2)(2)(6)#1197
                                  = -13
 CustomerInventoryBalance(12,2,6): - I_customer(12)(1)(6)#341
                                 + I_customer(12)(2)(6)#351
                                 - b_customer(12)(2)(6)#651
                                 - w(12)(1)(2)(6)#1227 - w(12)(2)(2)(6)#1257
                                  = -12
 CustomerInventoryBalance(3,3,6): - I_customer(3)(2)(6)#81
                                 + I_customer(3)(3)(6)#91
                                 - b_customer(3)(3)(6)#391 - w(3)(1)(3)(6)#697
                                 - w(3)(2)(3)(6)#727  = -18
 CustomerInventoryBalance(4,3,6): - I_customer(4)(2)(6)#111
                                 + I_customer(4)(3)(6)#121
                                 - b_customer(4)(3)(6)#421 - w(4)(1)(3)(6)#757
                                 - w(4)(2)(3)(6)#787  = -7
 CustomerInventoryBalance(5,3,6): - I_customer(5)(2)(6)#141
                                 + I_customer(5)(3)(6)#151
                                 - b_customer(5)(3)(6)#451 - w(5)(1)(3)(6)#817
                                 - w(5)(2)(3)(6)#847  = -5
 CustomerInventoryBalance(6,3,6): - I_customer(6)(2)(6)#171
                                 + I_customer(6)(3)(6)#181
                                 - b_customer(6)(3)(6)#481 - w(6)(1)(3)(6)#877
                                 - w(6)(2)(3)(6)#907  = -13
 CustomerInventoryBalance(7,3,6): - I_customer(7)(2)(6)#201
                                 + I_customer(7)(3)(6)#211
                                 - b_customer(7)(3)(6)#511 - w(7)(1)(3)(6)#937
                                 - w(7)(2)(3)(6)#967  = -7
 CustomerInventoryBalance(8,3,6): - I_customer(8)(2)(6)#231
                                 + I_customer(8)(3)(6)#241
                                 - b_customer(8)(3)(6)#541 - w(8)(1)(3)(6)#997
                                 - w(8)(2)(3)(6)#1027  = -17
 CustomerInventoryBalance(9,3,6): - I_customer(9)(2)(6)#261
                                 + I_customer(9)(3)(6)#271
                                 - b_customer(9)(3)(6)#571 - w(9)(1)(3)(6)#1057
                                 - w(9)(2)(3)(6)#1087  = -19
 CustomerInventoryBalance(10,3,6): - I_customer(10)(2)(6)#291
                                 + I_customer(10)(3)(6)#301
                                 - b_customer(10)(3)(6)#601
                                 - w(10)(1)(3)(6)#1117 - w(10)(2)(3)(6)#1147
                                  = -18
 CustomerInventoryBalance(11,3,6): - I_customer(11)(2)(6)#321
                                 + I_customer(11)(3)(6)#331
                                 - b_customer(11)(3)(6)#631
                                 - w(11)(1)(3)(6)#1177 - w(11)(2)(3)(6)#1207
                                  = -14
 CustomerInventoryBalance(12,3,6): - I_customer(12)(2)(6)#351
                                 + I_customer(12)(3)(6)#361
                                 - b_customer(12)(3)(6)#661
                                 - w(12)(1)(3)(6)#1237 - w(12)(2)(3)(6)#1267
                                  = -16
 CustomerInventoryBalance(3,1,7): I_customer(3)(1)(7)#72
                                 - b_customer(3)(1)(7)#372 - w(3)(1)(1)(7)#678
                                 - w(3)(2)(1)(7)#708  = 15
 CustomerInventoryBalance(4,1,7): I_customer(4)(1)(7)#102
                                 - b_customer(4)(1)(7)#402 - w(4)(1)(1)(7)#738
                                 - w(4)(2)(1)(7)#768  = 1
 CustomerInventoryBalance(5,1,7): I_customer(5)(1)(7)#132
                                 - b_customer(5)(1)(7)#432 - w(5)(1)(1)(7)#798
                                 - w(5)(2)(1)(7)#828  = 0
 CustomerInventoryBalance(6,1,7): I_customer(6)(1)(7)#162
                                 - b_customer(6)(1)(7)#462 - w(6)(1)(1)(7)#858
                                 - w(6)(2)(1)(7)#888  = -1
 CustomerInventoryBalance(7,1,7): I_customer(7)(1)(7)#192
                                 - b_customer(7)(1)(7)#492 - w(7)(1)(1)(7)#918
                                 - w(7)(2)(1)(7)#948  = 8
 CustomerInventoryBalance(8,1,7): I_customer(8)(1)(7)#222
                                 - b_customer(8)(1)(7)#522 - w(8)(1)(1)(7)#978
                                 - w(8)(2)(1)(7)#1008  = -2
 CustomerInventoryBalance(9,1,7): I_customer(9)(1)(7)#252
                                 - b_customer(9)(1)(7)#552 - w(9)(1)(1)(7)#1038
                                 - w(9)(2)(1)(7)#1068  = 19
 CustomerInventoryBalance(10,1,7): I_customer(10)(1)(7)#282
                                 - b_customer(10)(1)(7)#582
                                 - w(10)(1)(1)(7)#1098 - w(10)(2)(1)(7)#1128
                                  = 0
 CustomerInventoryBalance(11,1,7): I_customer(11)(1)(7)#312
                                 - b_customer(11)(1)(7)#612
                                 - w(11)(1)(1)(7)#1158 - w(11)(2)(1)(7)#1188
                                  = -1
 CustomerInventoryBalance(12,1,7): I_customer(12)(1)(7)#342
                                 - b_customer(12)(1)(7)#642
                                 - w(12)(1)(1)(7)#1218 - w(12)(2)(1)(7)#1248
                                  = 16
 CustomerInventoryBalance(3,2,7): - I_customer(3)(1)(7)#72
                                 + I_customer(3)(2)(7)#82
                                 - b_customer(3)(2)(7)#382 - w(3)(1)(2)(7)#688
                                 - w(3)(2)(2)(7)#718  = -21
 CustomerInventoryBalance(4,2,7): - I_customer(4)(1)(7)#102
                                 + I_customer(4)(2)(7)#112
                                 - b_customer(4)(2)(7)#412 - w(4)(1)(2)(7)#748
                                 - w(4)(2)(2)(7)#778  = -8
 CustomerInventoryBalance(5,2,7): - I_customer(5)(1)(7)#132
                                 + I_customer(5)(2)(7)#142
                                 - b_customer(5)(2)(7)#442 - w(5)(1)(2)(7)#808
                                 - w(5)(2)(2)(7)#838  = -5
 CustomerInventoryBalance(6,2,7): - I_customer(6)(1)(7)#162
                                 + I_customer(6)(2)(7)#172
                                 - b_customer(6)(2)(7)#472 - w(6)(1)(2)(7)#868
                                 - w(6)(2)(2)(7)#898  = -15
 CustomerInventoryBalance(7,2,7): - I_customer(7)(1)(7)#192
                                 + I_customer(7)(2)(7)#202
                                 - b_customer(7)(2)(7)#502 - w(7)(1)(2)(7)#928
                                 - w(7)(2)(2)(7)#958  = -7
 CustomerInventoryBalance(8,2,7): - I_customer(8)(1)(7)#222
                                 + I_customer(8)(2)(7)#232
                                 - b_customer(8)(2)(7)#532 - w(8)(1)(2)(7)#988
                                 - w(8)(2)(2)(7)#1018  = -20
 CustomerInventoryBalance(9,2,7): - I_customer(9)(1)(7)#252
                                 + I_customer(9)(2)(7)#262
                                 - b_customer(9)(2)(7)#562 - w(9)(1)(2)(7)#1048
                                 - w(9)(2)(2)(7)#1078  = -15
 CustomerInventoryBalance(10,2,7): - I_customer(10)(1)(7)#282
                                 + I_customer(10)(2)(7)#292
                                 - b_customer(10)(2)(7)#592
                                 - w(10)(1)(2)(7)#1108 - w(10)(2)(2)(7)#1138
                                  = -17
 CustomerInventoryBalance(11,2,7): - I_customer(11)(1)(7)#312
                                 + I_customer(11)(2)(7)#322
                                 - b_customer(11)(2)(7)#622
                                 - w(11)(1)(2)(7)#1168 - w(11)(2)(2)(7)#1198
                                  = -17
 CustomerInventoryBalance(12,2,7): - I_customer(12)(1)(7)#342
                                 + I_customer(12)(2)(7)#352
                                 - b_customer(12)(2)(7)#652
                                 - w(12)(1)(2)(7)#1228 - w(12)(2)(2)(7)#1258
                                  = -14
 CustomerInventoryBalance(3,3,7): - I_customer(3)(2)(7)#82
                                 + I_customer(3)(3)(7)#92
                                 - b_customer(3)(3)(7)#392 - w(3)(1)(3)(7)#698
                                 - w(3)(2)(3)(7)#728  = -19
 CustomerInventoryBalance(4,3,7): - I_customer(4)(2)(7)#112
                                 + I_customer(4)(3)(7)#122
                                 - b_customer(4)(3)(7)#422 - w(4)(1)(3)(7)#758
                                 - w(4)(2)(3)(7)#788  = -7
 CustomerInventoryBalance(5,3,7): - I_customer(5)(2)(7)#142
                                 + I_customer(5)(3)(7)#152
                                 - b_customer(5)(3)(7)#452 - w(5)(1)(3)(7)#818
                                 - w(5)(2)(3)(7)#848  = -5
 CustomerInventoryBalance(6,3,7): - I_customer(6)(2)(7)#172
                                 + I_customer(6)(3)(7)#182
                                 - b_customer(6)(3)(7)#482 - w(6)(1)(3)(7)#878
                                 - w(6)(2)(3)(7)#908  = -17
 CustomerInventoryBalance(7,3,7): - I_customer(7)(2)(7)#202
                                 + I_customer(7)(3)(7)#212
                                 - b_customer(7)(3)(7)#512 - w(7)(1)(3)(7)#938
                                 - w(7)(2)(3)(7)#968  = -7
 CustomerInventoryBalance(8,3,7): - I_customer(8)(2)(7)#232
                                 + I_customer(8)(3)(7)#242
                                 - b_customer(8)(3)(7)#542 - w(8)(1)(3)(7)#998
                                 - w(8)(2)(3)(7)#1028  = -16
 CustomerInventoryBalance(9,3,7): - I_customer(9)(2)(7)#262
                                 + I_customer(9)(3)(7)#272
                                 - b_customer(9)(3)(7)#572 - w(9)(1)(3)(7)#1058
                                 - w(9)(2)(3)(7)#1088  = -19
 CustomerInventoryBalance(10,3,7): - I_customer(10)(2)(7)#292
                                 + I_customer(10)(3)(7)#302
                                 - b_customer(10)(3)(7)#602
                                 - w(10)(1)(3)(7)#1118 - w(10)(2)(3)(7)#1148
                                  = -18
 CustomerInventoryBalance(11,3,7): - I_customer(11)(2)(7)#322
                                 + I_customer(11)(3)(7)#332
                                 - b_customer(11)(3)(7)#632
                                 - w(11)(1)(3)(7)#1178 - w(11)(2)(3)(7)#1208
                                  = -17
 CustomerInventoryBalance(12,3,7): - I_customer(12)(2)(7)#352
                                 + I_customer(12)(3)(7)#362
                                 - b_customer(12)(3)(7)#662
                                 - w(12)(1)(3)(7)#1238 - w(12)(2)(3)(7)#1268
                                  = -16
 CustomerInventoryBalance(3,1,8): I_customer(3)(1)(8)#73
                                 - b_customer(3)(1)(8)#373 - w(3)(1)(1)(8)#679
                                 - w(3)(2)(1)(8)#709  = 19
 CustomerInventoryBalance(4,1,8): I_customer(4)(1)(8)#103
                                 - b_customer(4)(1)(8)#403 - w(4)(1)(1)(8)#739
                                 - w(4)(2)(1)(8)#769  = 1
 CustomerInventoryBalance(5,1,8): I_customer(5)(1)(8)#133
                                 - b_customer(5)(1)(8)#433 - w(5)(1)(1)(8)#799
                                 - w(5)(2)(1)(8)#829  = 1
 CustomerInventoryBalance(6,1,8): I_customer(6)(1)(8)#163
                                 - b_customer(6)(1)(8)#463 - w(6)(1)(1)(8)#859
                                 - w(6)(2)(1)(8)#889  = -1
 CustomerInventoryBalance(7,1,8): I_customer(7)(1)(8)#193
                                 - b_customer(7)(1)(8)#493 - w(7)(1)(1)(8)#919
                                 - w(7)(2)(1)(8)#949  = 6
 CustomerInventoryBalance(8,1,8): I_customer(8)(1)(8)#223
                                 - b_customer(8)(1)(8)#523 - w(8)(1)(1)(8)#979
                                 - w(8)(2)(1)(8)#1009  = 3
 CustomerInventoryBalance(9,1,8): I_customer(9)(1)(8)#253
                                 - b_customer(9)(1)(8)#553 - w(9)(1)(1)(8)#1039
                                 - w(9)(2)(1)(8)#1069  = 17
 CustomerInventoryBalance(10,1,8): I_customer(10)(1)(8)#283
                                 - b_customer(10)(1)(8)#583
                                 - w(10)(1)(1)(8)#1099 - w(10)(2)(1)(8)#1129
                                  = 3
 CustomerInventoryBalance(11,1,8): I_customer(11)(1)(8)#313
                                 - b_customer(11)(1)(8)#613
                                 - w(11)(1)(1)(8)#1159 - w(11)(2)(1)(8)#1189
                                  = -3
 CustomerInventoryBalance(12,1,8): I_customer(12)(1)(8)#343
                                 - b_customer(12)(1)(8)#643
                                 - w(12)(1)(1)(8)#1219 - w(12)(2)(1)(8)#1249
                                  = 16
 CustomerInventoryBalance(3,2,8): - I_customer(3)(1)(8)#73
                                 + I_customer(3)(2)(8)#83
                                 - b_customer(3)(2)(8)#383 - w(3)(1)(2)(8)#689
                                 - w(3)(2)(2)(8)#719  = -19
 CustomerInventoryBalance(4,2,8): - I_customer(4)(1)(8)#103
                                 + I_customer(4)(2)(8)#113
                                 - b_customer(4)(2)(8)#413 - w(4)(1)(2)(8)#749
                                 - w(4)(2)(2)(8)#779  = -6
 CustomerInventoryBalance(5,2,8): - I_customer(5)(1)(8)#133
                                 + I_customer(5)(2)(8)#143
                                 - b_customer(5)(2)(8)#443 - w(5)(1)(2)(8)#809
                                 - w(5)(2)(2)(8)#839  = -5
 CustomerInventoryBalance(6,2,8): - I_customer(6)(1)(8)#163
                                 + I_customer(6)(2)(8)#173
                                 - b_customer(6)(2)(8)#473 - w(6)(1)(2)(8)#869
                                 - w(6)(2)(2)(8)#899  = -17
 CustomerInventoryBalance(7,2,8): - I_customer(7)(1)(8)#193
                                 + I_customer(7)(2)(8)#203
                                 - b_customer(7)(2)(8)#503 - w(7)(1)(2)(8)#929
                                 - w(7)(2)(2)(8)#959  = -7
 CustomerInventoryBalance(8,2,8): - I_customer(8)(1)(8)#223
                                 + I_customer(8)(2)(8)#233
                                 - b_customer(8)(2)(8)#533 - w(8)(1)(2)(8)#989
                                 - w(8)(2)(2)(8)#1019  = -19
 CustomerInventoryBalance(9,2,8): - I_customer(9)(1)(8)#253
                                 + I_customer(9)(2)(8)#263
                                 - b_customer(9)(2)(8)#563 - w(9)(1)(2)(8)#1049
                                 - w(9)(2)(2)(8)#1079  = -15
 CustomerInventoryBalance(10,2,8): - I_customer(10)(1)(8)#283
                                 + I_customer(10)(2)(8)#293
                                 - b_customer(10)(2)(8)#593
                                 - w(10)(1)(2)(8)#1109 - w(10)(2)(2)(8)#1139
                                  = -13
 CustomerInventoryBalance(11,2,8): - I_customer(11)(1)(8)#313
                                 + I_customer(11)(2)(8)#323
                                 - b_customer(11)(2)(8)#623
                                 - w(11)(1)(2)(8)#1169 - w(11)(2)(2)(8)#1199
                                  = -14
 CustomerInventoryBalance(12,2,8): - I_customer(12)(1)(8)#343
                                 + I_customer(12)(2)(8)#353
                                 - b_customer(12)(2)(8)#653
                                 - w(12)(1)(2)(8)#1229 - w(12)(2)(2)(8)#1259
                                  = -17
 CustomerInventoryBalance(3,3,8): - I_customer(3)(2)(8)#83
                                 + I_customer(3)(3)(8)#93
                                 - b_customer(3)(3)(8)#393 - w(3)(1)(3)(8)#699
                                 - w(3)(2)(3)(8)#729  = -15
 CustomerInventoryBalance(4,3,8): - I_customer(4)(2)(8)#113
                                 + I_customer(4)(3)(8)#123
                                 - b_customer(4)(3)(8)#423 - w(4)(1)(3)(8)#759
                                 - w(4)(2)(3)(8)#789  = -7
 CustomerInventoryBalance(5,3,8): - I_customer(5)(2)(8)#143
                                 + I_customer(5)(3)(8)#153
                                 - b_customer(5)(3)(8)#453 - w(5)(1)(3)(8)#819
                                 - w(5)(2)(3)(8)#849  = -7
 CustomerInventoryBalance(6,3,8): - I_customer(6)(2)(8)#173
                                 + I_customer(6)(3)(8)#183
                                 - b_customer(6)(3)(8)#483 - w(6)(1)(3)(8)#879
                                 - w(6)(2)(3)(8)#909  = -16
 CustomerInventoryBalance(7,3,8): - I_customer(7)(2)(8)#203
                                 + I_customer(7)(3)(8)#213
                                 - b_customer(7)(3)(8)#513 - w(7)(1)(3)(8)#939
                                 - w(7)(2)(3)(8)#969  = -7
 CustomerInventoryBalance(8,3,8): - I_customer(8)(2)(8)#233
                                 + I_customer(8)(3)(8)#243
                                 - b_customer(8)(3)(8)#543 - w(8)(1)(3)(8)#999
                                 - w(8)(2)(3)(8)#1029  = -21
 CustomerInventoryBalance(9,3,8): - I_customer(9)(2)(8)#263
                                 + I_customer(9)(3)(8)#273
                                 - b_customer(9)(3)(8)#573 - w(9)(1)(3)(8)#1059
                                 - w(9)(2)(3)(8)#1089  = -18
 CustomerInventoryBalance(10,3,8): - I_customer(10)(2)(8)#293
                                 + I_customer(10)(3)(8)#303
                                 - b_customer(10)(3)(8)#603
                                 - w(10)(1)(3)(8)#1119 - w(10)(2)(3)(8)#1149
                                  = -17
 CustomerInventoryBalance(11,3,8): - I_customer(11)(2)(8)#323
                                 + I_customer(11)(3)(8)#333
                                 - b_customer(11)(3)(8)#633
                                 - w(11)(1)(3)(8)#1179 - w(11)(2)(3)(8)#1209
                                  = -16
 CustomerInventoryBalance(12,3,8): - I_customer(12)(2)(8)#353
                                 + I_customer(12)(3)(8)#363
                                 - b_customer(12)(3)(8)#663
                                 - w(12)(1)(3)(8)#1239 - w(12)(2)(3)(8)#1269
                                  = -14
 CustomerInventoryBalance(3,1,9): I_customer(3)(1)(9)#74
                                 - b_customer(3)(1)(9)#374 - w(3)(1)(1)(9)#680
                                 - w(3)(2)(1)(9)#710  = 15
 CustomerInventoryBalance(4,1,9): I_customer(4)(1)(9)#104
                                 - b_customer(4)(1)(9)#404 - w(4)(1)(1)(9)#740
                                 - w(4)(2)(1)(9)#770  = -1
 CustomerInventoryBalance(5,1,9): I_customer(5)(1)(9)#134
                                 - b_customer(5)(1)(9)#434 - w(5)(1)(1)(9)#800
                                 - w(5)(2)(1)(9)#830  = 1
 CustomerInventoryBalance(6,1,9): I_customer(6)(1)(9)#164
                                 - b_customer(6)(1)(9)#464 - w(6)(1)(1)(9)#860
                                 - w(6)(2)(1)(9)#890  = 0
 CustomerInventoryBalance(7,1,9): I_customer(7)(1)(9)#194
                                 - b_customer(7)(1)(9)#494 - w(7)(1)(1)(9)#920
                                 - w(7)(2)(1)(9)#950  = 7
 CustomerInventoryBalance(8,1,9): I_customer(8)(1)(9)#224
                                 - b_customer(8)(1)(9)#524 - w(8)(1)(1)(9)#980
                                 - w(8)(2)(1)(9)#1010  = -1
 CustomerInventoryBalance(9,1,9): I_customer(9)(1)(9)#254
                                 - b_customer(9)(1)(9)#554 - w(9)(1)(1)(9)#1040
                                 - w(9)(2)(1)(9)#1070  = 15
 CustomerInventoryBalance(10,1,9): I_customer(10)(1)(9)#284
                                 - b_customer(10)(1)(9)#584
                                 - w(10)(1)(1)(9)#1100 - w(10)(2)(1)(9)#1130
                                  = 2
 CustomerInventoryBalance(11,1,9): I_customer(11)(1)(9)#314
                                 - b_customer(11)(1)(9)#614
                                 - w(11)(1)(1)(9)#1160 - w(11)(2)(1)(9)#1190
                                  = -2
 CustomerInventoryBalance(12,1,9): I_customer(12)(1)(9)#344
                                 - b_customer(12)(1)(9)#644
                                 - w(12)(1)(1)(9)#1220 - w(12)(2)(1)(9)#1250
                                  = 17
 CustomerInventoryBalance(3,2,9): - I_customer(3)(1)(9)#74
                                 + I_customer(3)(2)(9)#84
                                 - b_customer(3)(2)(9)#384 - w(3)(1)(2)(9)#690
                                 - w(3)(2)(2)(9)#720  = -18
 CustomerInventoryBalance(4,2,9): - I_customer(4)(1)(9)#104
                                 + I_customer(4)(2)(9)#114
                                 - b_customer(4)(2)(9)#414 - w(4)(1)(2)(9)#750
                                 - w(4)(2)(2)(9)#780  = -7
 CustomerInventoryBalance(5,2,9): - I_customer(5)(1)(9)#134
                                 + I_customer(5)(2)(9)#144
                                 - b_customer(5)(2)(9)#444 - w(5)(1)(2)(9)#810
                                 - w(5)(2)(2)(9)#840  = -7
 CustomerInventoryBalance(6,2,9): - I_customer(6)(1)(9)#164
                                 + I_customer(6)(2)(9)#174
                                 - b_customer(6)(2)(9)#474 - w(6)(1)(2)(9)#870
                                 - w(6)(2)(2)(9)#900  = -16
 CustomerInventoryBalance(7,2,9): - I_customer(7)(1)(9)#194
                                 + I_customer(7)(2)(9)#204
                                 - b_customer(7)(2)(9)#504 - w(7)(1)(2)(9)#930
                                 - w(7)(2)(2)(9)#960  = -7
 CustomerInventoryBalance(8,2,9): - I_customer(8)(1)(9)#224
                                 + I_customer(8)(2)(9)#234
                                 - b_customer(8)(2)(9)#534 - w(8)(1)(2)(9)#990
                                 - w(8)(2)(2)(9)#1020  = -17
 CustomerInventoryBalance(9,2,9): - I_customer(9)(1)(9)#254
                                 + I_customer(9)(2)(9)#264
                                 - b_customer(9)(2)(9)#564 - w(9)(1)(2)(9)#1050
                                 - w(9)(2)(2)(9)#1080  = -18
 CustomerInventoryBalance(10,2,9): - I_customer(10)(1)(9)#284
                                 + I_customer(10)(2)(9)#294
                                 - b_customer(10)(2)(9)#594
                                 - w(10)(1)(2)(9)#1110 - w(10)(2)(2)(9)#1140
                                  = -14
 CustomerInventoryBalance(11,2,9): - I_customer(11)(1)(9)#314
                                 + I_customer(11)(2)(9)#324
                                 - b_customer(11)(2)(9)#624
                                 - w(11)(1)(2)(9)#1170 - w(11)(2)(2)(9)#1200
                                  = -13
 CustomerInventoryBalance(12,2,9): - I_customer(12)(1)(9)#344
                                 + I_customer(12)(2)(9)#354
                                 - b_customer(12)(2)(9)#654
                                 - w(12)(1)(2)(9)#1230 - w(12)(2)(2)(9)#1260
                                  = -16
 CustomerInventoryBalance(3,3,9): - I_customer(3)(2)(9)#84
                                 + I_customer(3)(3)(9)#94
                                 - b_customer(3)(3)(9)#394 - w(3)(1)(3)(9)#700
                                 - w(3)(2)(3)(9)#730  = -16
 CustomerInventoryBalance(4,3,9): - I_customer(4)(2)(9)#114
                                 + I_customer(4)(3)(9)#124
                                 - b_customer(4)(3)(9)#424 - w(4)(1)(3)(9)#760
                                 - w(4)(2)(3)(9)#790  = -8
 CustomerInventoryBalance(5,3,9): - I_customer(5)(2)(9)#144
                                 + I_customer(5)(3)(9)#154
                                 - b_customer(5)(3)(9)#454 - w(5)(1)(3)(9)#820
                                 - w(5)(2)(3)(9)#850  = -5
 CustomerInventoryBalance(6,3,9): - I_customer(6)(2)(9)#174
                                 + I_customer(6)(3)(9)#184
                                 - b_customer(6)(3)(9)#484 - w(6)(1)(3)(9)#880
                                 - w(6)(2)(3)(9)#910  = -14
 CustomerInventoryBalance(7,3,9): - I_customer(7)(2)(9)#204
                                 + I_customer(7)(3)(9)#214
                                 - b_customer(7)(3)(9)#514 - w(7)(1)(3)(9)#940
                                 - w(7)(2)(3)(9)#970  = -7
 CustomerInventoryBalance(8,3,9): - I_customer(8)(2)(9)#234
                                 + I_customer(8)(3)(9)#244
                                 - b_customer(8)(3)(9)#544 - w(8)(1)(3)(9)#1000
                                 - w(8)(2)(3)(9)#1030  = -15
 CustomerInventoryBalance(9,3,9): - I_customer(9)(2)(9)#264
                                 + I_customer(9)(3)(9)#274
                                 - b_customer(9)(3)(9)#574 - w(9)(1)(3)(9)#1060
                                 - w(9)(2)(3)(9)#1090  = -17
 CustomerInventoryBalance(10,3,9): - I_customer(10)(2)(9)#294
                                 + I_customer(10)(3)(9)#304
                                 - b_customer(10)(3)(9)#604
                                 - w(10)(1)(3)(9)#1120 - w(10)(2)(3)(9)#1150
                                  = -19
 CustomerInventoryBalance(11,3,9): - I_customer(11)(2)(9)#324
                                 + I_customer(11)(3)(9)#334
                                 - b_customer(11)(3)(9)#634
                                 - w(11)(1)(3)(9)#1180 - w(11)(2)(3)(9)#1210
                                  = -16
 CustomerInventoryBalance(12,3,9): - I_customer(12)(2)(9)#354
                                 + I_customer(12)(3)(9)#364
                                 - b_customer(12)(3)(9)#664
                                 - w(12)(1)(3)(9)#1240 - w(12)(2)(3)(9)#1270
                                  = -13
 CustomerInventoryBalance(3,1,10): I_customer(3)(1)(10)#75
                                 - b_customer(3)(1)(10)#375
                                 - w(3)(1)(1)(10)#681 - w(3)(2)(1)(10)#711
                                  = 17
 CustomerInventoryBalance(4,1,10): I_customer(4)(1)(10)#105
                                 - b_customer(4)(1)(10)#405
                                 - w(4)(1)(1)(10)#741 - w(4)(2)(1)(10)#771  = 1
 CustomerInventoryBalance(5,1,10): I_customer(5)(1)(10)#135
                                 - b_customer(5)(1)(10)#435
                                 - w(5)(1)(1)(10)#801 - w(5)(2)(1)(10)#831  = 0
 CustomerInventoryBalance(6,1,10): I_customer(6)(1)(10)#165
                                 - b_customer(6)(1)(10)#465
                                 - w(6)(1)(1)(10)#861 - w(6)(2)(1)(10)#891  = 1
 CustomerInventoryBalance(7,1,10): I_customer(7)(1)(10)#195
                                 - b_customer(7)(1)(10)#495
                                 - w(7)(1)(1)(10)#921 - w(7)(2)(1)(10)#951  = 6
 CustomerInventoryBalance(8,1,10): I_customer(8)(1)(10)#225
                                 - b_customer(8)(1)(10)#525
                                 - w(8)(1)(1)(10)#981 - w(8)(2)(1)(10)#1011
                                  = 0
 CustomerInventoryBalance(9,1,10): I_customer(9)(1)(10)#255
                                 - b_customer(9)(1)(10)#555
                                 - w(9)(1)(1)(10)#1041 - w(9)(2)(1)(10)#1071
                                  = 17
 CustomerInventoryBalance(10,1,10): I_customer(10)(1)(10)#285
                                 - b_customer(10)(1)(10)#585
                                 - w(10)(1)(1)(10)#1101 - w(10)(2)(1)(10)#1131
                                  = 3
 CustomerInventoryBalance(11,1,10): I_customer(11)(1)(10)#315
                                 - b_customer(11)(1)(10)#615
                                 - w(11)(1)(1)(10)#1161 - w(11)(2)(1)(10)#1191
                                  = 0
 CustomerInventoryBalance(12,1,10): I_customer(12)(1)(10)#345
                                 - b_customer(12)(1)(10)#645
                                 - w(12)(1)(1)(10)#1221 - w(12)(2)(1)(10)#1251
                                  = 16
 CustomerInventoryBalance(3,2,10): - I_customer(3)(1)(10)#75
                                 + I_customer(3)(2)(10)#85
                                 - b_customer(3)(2)(10)#385
                                 - w(3)(1)(2)(10)#691 - w(3)(2)(2)(10)#721
                                  = -19
 CustomerInventoryBalance(4,2,10): - I_customer(4)(1)(10)#105
                                 + I_customer(4)(2)(10)#115
                                 - b_customer(4)(2)(10)#415
                                 - w(4)(1)(2)(10)#751 - w(4)(2)(2)(10)#781
                                  = -6
 CustomerInventoryBalance(5,2,10): - I_customer(5)(1)(10)#135
                                 + I_customer(5)(2)(10)#145
                                 - b_customer(5)(2)(10)#445
                                 - w(5)(1)(2)(10)#811 - w(5)(2)(2)(10)#841
                                  = -5
 CustomerInventoryBalance(6,2,10): - I_customer(6)(1)(10)#165
                                 + I_customer(6)(2)(10)#175
                                 - b_customer(6)(2)(10)#475
                                 - w(6)(1)(2)(10)#871 - w(6)(2)(2)(10)#901
                                  = -18
 CustomerInventoryBalance(7,2,10): - I_customer(7)(1)(10)#195
                                 + I_customer(7)(2)(10)#205
                                 - b_customer(7)(2)(10)#505
                                 - w(7)(1)(2)(10)#931 - w(7)(2)(2)(10)#961
                                  = -7
 CustomerInventoryBalance(8,2,10): - I_customer(8)(1)(10)#225
                                 + I_customer(8)(2)(10)#235
                                 - b_customer(8)(2)(10)#535
                                 - w(8)(1)(2)(10)#991 - w(8)(2)(2)(10)#1021
                                  = -16
 CustomerInventoryBalance(9,2,10): - I_customer(9)(1)(10)#255
                                 + I_customer(9)(2)(10)#265
                                 - b_customer(9)(2)(10)#565
                                 - w(9)(1)(2)(10)#1051 - w(9)(2)(2)(10)#1081
                                  = -18
 CustomerInventoryBalance(10,2,10): - I_customer(10)(1)(10)#285
                                 + I_customer(10)(2)(10)#295
                                 - b_customer(10)(2)(10)#595
                                 - w(10)(1)(2)(10)#1111 - w(10)(2)(2)(10)#1141
                                  = -16
 CustomerInventoryBalance(11,2,10): - I_customer(11)(1)(10)#315
                                 + I_customer(11)(2)(10)#325
                                 - b_customer(11)(2)(10)#625
                                 - w(11)(1)(2)(10)#1171 - w(11)(2)(2)(10)#1201
                                  = -15
 CustomerInventoryBalance(12,2,10): - I_customer(12)(1)(10)#345
                                 + I_customer(12)(2)(10)#355
                                 - b_customer(12)(2)(10)#655
                                 - w(12)(1)(2)(10)#1231 - w(12)(2)(2)(10)#1261
                                  = -11
 CustomerInventoryBalance(3,3,10): - I_customer(3)(2)(10)#85
                                 + I_customer(3)(3)(10)#95
                                 - b_customer(3)(3)(10)#395
                                 - w(3)(1)(3)(10)#701 - w(3)(2)(3)(10)#731
                                  = -21
 CustomerInventoryBalance(4,3,10): - I_customer(4)(2)(10)#115
                                 + I_customer(4)(3)(10)#125
                                 - b_customer(4)(3)(10)#425
                                 - w(4)(1)(3)(10)#761 - w(4)(2)(3)(10)#791
                                  = -7
 CustomerInventoryBalance(5,3,10): - I_customer(5)(2)(10)#145
                                 + I_customer(5)(3)(10)#155
                                 - b_customer(5)(3)(10)#455
                                 - w(5)(1)(3)(10)#821 - w(5)(2)(3)(10)#851
                                  = -6
 CustomerInventoryBalance(6,3,10): - I_customer(6)(2)(10)#175
                                 + I_customer(6)(3)(10)#185
                                 - b_customer(6)(3)(10)#485
                                 - w(6)(1)(3)(10)#881 - w(6)(2)(3)(10)#911
                                  = -17
 CustomerInventoryBalance(7,3,10): - I_customer(7)(2)(10)#205
                                 + I_customer(7)(3)(10)#215
                                 - b_customer(7)(3)(10)#515
                                 - w(7)(1)(3)(10)#941 - w(7)(2)(3)(10)#971
                                  = -7
 CustomerInventoryBalance(8,3,10): - I_customer(8)(2)(10)#235
                                 + I_customer(8)(3)(10)#245
                                 - b_customer(8)(3)(10)#545
                                 - w(8)(1)(3)(10)#1001 - w(8)(2)(3)(10)#1031
                                  = -17
 CustomerInventoryBalance(9,3,10): - I_customer(9)(2)(10)#265
                                 + I_customer(9)(3)(10)#275
                                 - b_customer(9)(3)(10)#575
                                 - w(9)(1)(3)(10)#1061 - w(9)(2)(3)(10)#1091
                                  = -18
 CustomerInventoryBalance(10,3,10): - I_customer(10)(2)(10)#295
                                 + I_customer(10)(3)(10)#305
                                 - b_customer(10)(3)(10)#605
                                 - w(10)(1)(3)(10)#1121 - w(10)(2)(3)(10)#1151
                                  = -16
 CustomerInventoryBalance(11,3,10): - I_customer(11)(2)(10)#325
                                 + I_customer(11)(3)(10)#335
                                 - b_customer(11)(3)(10)#635
                                 - w(11)(1)(3)(10)#1181 - w(11)(2)(3)(10)#1211
                                  = -15
 CustomerInventoryBalance(12,3,10): - I_customer(12)(2)(10)#355
                                 + I_customer(12)(3)(10)#365
                                 - b_customer(12)(3)(10)#665
                                 - w(12)(1)(3)(10)#1241 - w(12)(2)(3)(10)#1271
                                  = -16
 CustomerInventoryCapacity(3,1,1): I_customer(3)(1)(1)#66 <= 55
 CustomerInventoryCapacity(4,1,1): I_customer(4)(1)(1)#96 <= 20
 CustomerInventoryCapacity(5,1,1): I_customer(5)(1)(1)#126 <= 19
 CustomerInventoryCapacity(6,1,1): I_customer(6)(1)(1)#156 <= 51
 CustomerInventoryCapacity(7,1,1): I_customer(7)(1)(1)#186 <= 20
 CustomerInventoryCapacity(8,1,1): I_customer(8)(1)(1)#216 <= 57
 CustomerInventoryCapacity(9,1,1): I_customer(9)(1)(1)#246 <= 45
 CustomerInventoryCapacity(10,1,1): I_customer(10)(1)(1)#276 <= 50
 CustomerInventoryCapacity(11,1,1): I_customer(11)(1)(1)#306 <= 51
 CustomerInventoryCapacity(12,1,1): I_customer(12)(1)(1)#336 <= 43
 CustomerInventoryCapacity(3,2,1): I_customer(3)(2)(1)#76 <= 59
 CustomerInventoryCapacity(4,2,1): I_customer(4)(2)(1)#106 <= 22
 CustomerInventoryCapacity(5,2,1): I_customer(5)(2)(1)#136 <= 18
 CustomerInventoryCapacity(6,2,1): I_customer(6)(2)(1)#166 <= 47
 CustomerInventoryCapacity(7,2,1): I_customer(7)(2)(1)#196 <= 20
 CustomerInventoryCapacity(8,2,1): I_customer(8)(2)(1)#226 <= 52
 CustomerInventoryCapacity(9,2,1): I_customer(9)(2)(1)#256 <= 49
 CustomerInventoryCapacity(10,2,1): I_customer(10)(2)(1)#286 <= 50
 CustomerInventoryCapacity(11,2,1): I_customer(11)(2)(1)#316 <= 51
 CustomerInventoryCapacity(12,2,1): I_customer(12)(2)(1)#346 <= 42
 CustomerInventoryCapacity(3,3,1): I_customer(3)(3)(1)#86 <= 54
 CustomerInventoryCapacity(4,3,1): I_customer(4)(3)(1)#116 <= 20
 CustomerInventoryCapacity(5,3,1): I_customer(5)(3)(1)#146 <= 18
 CustomerInventoryCapacity(6,3,1): I_customer(6)(3)(1)#176 <= 47
 CustomerInventoryCapacity(7,3,1): I_customer(7)(3)(1)#206 <= 22
 CustomerInventoryCapacity(8,3,1): I_customer(8)(3)(1)#236 <= 53
 CustomerInventoryCapacity(9,3,1): I_customer(9)(3)(1)#266 <= 47
 CustomerInventoryCapacity(10,3,1): I_customer(10)(3)(1)#296 <= 48
 CustomerInventoryCapacity(11,3,1): I_customer(11)(3)(1)#326 <= 50
 CustomerInventoryCapacity(12,3,1): I_customer(12)(3)(1)#356 <= 40
 CustomerInventoryCapacity(3,1,2): I_customer(3)(1)(2)#67 <= 54
 CustomerInventoryCapacity(4,1,2): I_customer(4)(1)(2)#97 <= 22
 CustomerInventoryCapacity(5,1,2): I_customer(5)(1)(2)#127 <= 19
 CustomerInventoryCapacity(6,1,2): I_customer(6)(1)(2)#157 <= 47
 CustomerInventoryCapacity(7,1,2): I_customer(7)(1)(2)#187 <= 22
 CustomerInventoryCapacity(8,1,2): I_customer(8)(1)(2)#217 <= 54
 CustomerInventoryCapacity(9,1,2): I_customer(9)(1)(2)#247 <= 51
 CustomerInventoryCapacity(10,1,2): I_customer(10)(1)(2)#277 <= 50
 CustomerInventoryCapacity(11,1,2): I_customer(11)(1)(2)#307 <= 45
 CustomerInventoryCapacity(12,1,2): I_customer(12)(1)(2)#337 <= 45
 CustomerInventoryCapacity(3,2,2): I_customer(3)(2)(2)#77 <= 56
 CustomerInventoryCapacity(4,2,2): I_customer(4)(2)(2)#107 <= 20
 CustomerInventoryCapacity(5,2,2): I_customer(5)(2)(2)#137 <= 17
 CustomerInventoryCapacity(6,2,2): I_customer(6)(2)(2)#167 <= 48
 CustomerInventoryCapacity(7,2,2): I_customer(7)(2)(2)#197 <= 21
 CustomerInventoryCapacity(8,2,2): I_customer(8)(2)(2)#227 <= 55
 CustomerInventoryCapacity(9,2,2): I_customer(9)(2)(2)#257 <= 50
 CustomerInventoryCapacity(10,2,2): I_customer(10)(2)(2)#287 <= 46
 CustomerInventoryCapacity(11,2,2): I_customer(11)(2)(2)#317 <= 48
 CustomerInventoryCapacity(12,2,2): I_customer(12)(2)(2)#347 <= 42
 CustomerInventoryCapacity(3,3,2): I_customer(3)(3)(2)#87 <= 60
 CustomerInventoryCapacity(4,3,2): I_customer(4)(3)(2)#117 <= 21
 CustomerInventoryCapacity(5,3,2): I_customer(5)(3)(2)#147 <= 17
 CustomerInventoryCapacity(6,3,2): I_customer(6)(3)(2)#177 <= 47
 CustomerInventoryCapacity(7,3,2): I_customer(7)(3)(2)#207 <= 22
 CustomerInventoryCapacity(8,3,2): I_customer(8)(3)(2)#237 <= 53
 CustomerInventoryCapacity(9,3,2): I_customer(9)(3)(2)#267 <= 51
 CustomerInventoryCapacity(10,3,2): I_customer(10)(3)(2)#297 <= 47
 CustomerInventoryCapacity(11,3,2): I_customer(11)(3)(2)#327 <= 46
 CustomerInventoryCapacity(12,3,2): I_customer(12)(3)(2)#357 <= 40
 CustomerInventoryCapacity(3,1,3): I_customer(3)(1)(3)#68 <= 55
 CustomerInventoryCapacity(4,1,3): I_customer(4)(1)(3)#98 <= 20
 CustomerInventoryCapacity(5,1,3): I_customer(5)(1)(3)#128 <= 18
 CustomerInventoryCapacity(6,1,3): I_customer(6)(1)(3)#158 <= 48
 CustomerInventoryCapacity(7,1,3): I_customer(7)(1)(3)#188 <= 20
 CustomerInventoryCapacity(8,1,3): I_customer(8)(1)(3)#218 <= 51
 CustomerInventoryCapacity(9,1,3): I_customer(9)(1)(3)#248 <= 50
 CustomerInventoryCapacity(10,1,3): I_customer(10)(1)(3)#278 <= 46
 CustomerInventoryCapacity(11,1,3): I_customer(11)(1)(3)#308 <= 49
 CustomerInventoryCapacity(12,1,3): I_customer(12)(1)(3)#338 <= 43
 CustomerInventoryCapacity(3,2,3): I_customer(3)(2)(3)#78 <= 56
 CustomerInventoryCapacity(4,2,3): I_customer(4)(2)(3)#108 <= 20
 CustomerInventoryCapacity(5,2,3): I_customer(5)(2)(3)#138 <= 17
 CustomerInventoryCapacity(6,2,3): I_customer(6)(2)(3)#168 <= 49
 CustomerInventoryCapacity(7,2,3): I_customer(7)(2)(3)#198 <= 21
 CustomerInventoryCapacity(8,2,3): I_customer(8)(2)(3)#228 <= 52
 CustomerInventoryCapacity(9,2,3): I_customer(9)(2)(3)#258 <= 48
 CustomerInventoryCapacity(10,2,3): I_customer(10)(2)(3)#288 <= 51
 CustomerInventoryCapacity(11,2,3): I_customer(11)(2)(3)#318 <= 46
 CustomerInventoryCapacity(12,2,3): I_customer(12)(2)(3)#348 <= 43
 CustomerInventoryCapacity(3,3,3): I_customer(3)(3)(3)#88 <= 56
 CustomerInventoryCapacity(4,3,3): I_customer(4)(3)(3)#118 <= 21
 CustomerInventoryCapacity(5,3,3): I_customer(5)(3)(3)#148 <= 19
 CustomerInventoryCapacity(6,3,3): I_customer(6)(3)(3)#178 <= 48
 CustomerInventoryCapacity(7,3,3): I_customer(7)(3)(3)#208 <= 21
 CustomerInventoryCapacity(8,3,3): I_customer(8)(3)(3)#238 <= 51
 CustomerInventoryCapacity(9,3,3): I_customer(9)(3)(3)#268 <= 45
 CustomerInventoryCapacity(10,3,3): I_customer(10)(3)(3)#298 <= 45
 CustomerInventoryCapacity(11,3,3): I_customer(11)(3)(3)#328 <= 47
 CustomerInventoryCapacity(12,3,3): I_customer(12)(3)(3)#358 <= 43
 CustomerInventoryCapacity(3,1,4): I_customer(3)(1)(4)#69 <= 55
 CustomerInventoryCapacity(4,1,4): I_customer(4)(1)(4)#99 <= 20
 CustomerInventoryCapacity(5,1,4): I_customer(5)(1)(4)#129 <= 17
 CustomerInventoryCapacity(6,1,4): I_customer(6)(1)(4)#159 <= 47
 CustomerInventoryCapacity(7,1,4): I_customer(7)(1)(4)#189 <= 20
 CustomerInventoryCapacity(8,1,4): I_customer(8)(1)(4)#219 <= 56
 CustomerInventoryCapacity(9,1,4): I_customer(9)(1)(4)#249 <= 48
 CustomerInventoryCapacity(10,1,4): I_customer(10)(1)(4)#279 <= 46
 CustomerInventoryCapacity(11,1,4): I_customer(11)(1)(4)#309 <= 49
 CustomerInventoryCapacity(12,1,4): I_customer(12)(1)(4)#339 <= 41
 CustomerInventoryCapacity(3,2,4): I_customer(3)(2)(4)#79 <= 56
 CustomerInventoryCapacity(4,2,4): I_customer(4)(2)(4)#109 <= 21
 CustomerInventoryCapacity(5,2,4): I_customer(5)(2)(4)#139 <= 18
 CustomerInventoryCapacity(6,2,4): I_customer(6)(2)(4)#169 <= 50
 CustomerInventoryCapacity(7,2,4): I_customer(7)(2)(4)#199 <= 22
 CustomerInventoryCapacity(8,2,4): I_customer(8)(2)(4)#229 <= 55
 CustomerInventoryCapacity(9,2,4): I_customer(9)(2)(4)#259 <= 47
 CustomerInventoryCapacity(10,2,4): I_customer(10)(2)(4)#289 <= 45
 CustomerInventoryCapacity(11,2,4): I_customer(11)(2)(4)#319 <= 45
 CustomerInventoryCapacity(12,2,4): I_customer(12)(2)(4)#349 <= 41
 CustomerInventoryCapacity(3,3,4): I_customer(3)(3)(4)#89 <= 55
 CustomerInventoryCapacity(4,3,4): I_customer(4)(3)(4)#119 <= 22
 CustomerInventoryCapacity(5,3,4): I_customer(5)(3)(4)#149 <= 18
 CustomerInventoryCapacity(6,3,4): I_customer(6)(3)(4)#179 <= 50
 CustomerInventoryCapacity(7,3,4): I_customer(7)(3)(4)#209 <= 22
 CustomerInventoryCapacity(8,3,4): I_customer(8)(3)(4)#239 <= 56
 CustomerInventoryCapacity(9,3,4): I_customer(9)(3)(4)#269 <= 48
 CustomerInventoryCapacity(10,3,4): I_customer(10)(3)(4)#299 <= 46
 CustomerInventoryCapacity(11,3,4): I_customer(11)(3)(4)#329 <= 50
 CustomerInventoryCapacity(12,3,4): I_customer(12)(3)(4)#359 <= 44
 CustomerInventoryCapacity(3,1,5): I_customer(3)(1)(5)#70 <= 58
 CustomerInventoryCapacity(4,1,5): I_customer(4)(1)(5)#100 <= 20
 CustomerInventoryCapacity(5,1,5): I_customer(5)(1)(5)#130 <= 17
 CustomerInventoryCapacity(6,1,5): I_customer(6)(1)(5)#160 <= 47
 CustomerInventoryCapacity(7,1,5): I_customer(7)(1)(5)#190 <= 20
 CustomerInventoryCapacity(8,1,5): I_customer(8)(1)(5)#220 <= 51
 CustomerInventoryCapacity(9,1,5): I_customer(9)(1)(5)#250 <= 45
 CustomerInventoryCapacity(10,1,5): I_customer(10)(1)(5)#280 <= 51
 CustomerInventoryCapacity(11,1,5): I_customer(11)(1)(5)#310 <= 45
 CustomerInventoryCapacity(12,1,5): I_customer(12)(1)(5)#340 <= 42
 CustomerInventoryCapacity(3,2,5): I_customer(3)(2)(5)#80 <= 54
 CustomerInventoryCapacity(4,2,5): I_customer(4)(2)(5)#110 <= 20
 CustomerInventoryCapacity(5,2,5): I_customer(5)(2)(5)#140 <= 19
 CustomerInventoryCapacity(6,2,5): I_customer(6)(2)(5)#170 <= 49
 CustomerInventoryCapacity(7,2,5): I_customer(7)(2)(5)#200 <= 22
 CustomerInventoryCapacity(8,2,5): I_customer(8)(2)(5)#230 <= 57
 CustomerInventoryCapacity(9,2,5): I_customer(9)(2)(5)#260 <= 45
 CustomerInventoryCapacity(10,2,5): I_customer(10)(2)(5)#290 <= 48
 CustomerInventoryCapacity(11,2,5): I_customer(11)(2)(5)#320 <= 46
 CustomerInventoryCapacity(12,2,5): I_customer(12)(2)(5)#350 <= 44
 CustomerInventoryCapacity(3,3,5): I_customer(3)(3)(5)#90 <= 59
 CustomerInventoryCapacity(4,3,5): I_customer(4)(3)(5)#120 <= 21
 CustomerInventoryCapacity(5,3,5): I_customer(5)(3)(5)#150 <= 18
 CustomerInventoryCapacity(6,3,5): I_customer(6)(3)(5)#180 <= 46
 CustomerInventoryCapacity(7,3,5): I_customer(7)(3)(5)#210 <= 21
 CustomerInventoryCapacity(8,3,5): I_customer(8)(3)(5)#240 <= 57
 CustomerInventoryCapacity(9,3,5): I_customer(9)(3)(5)#270 <= 47
 CustomerInventoryCapacity(10,3,5): I_customer(10)(3)(5)#300 <= 50
 CustomerInventoryCapacity(11,3,5): I_customer(11)(3)(5)#330 <= 50
 CustomerInventoryCapacity(12,3,5): I_customer(12)(3)(5)#360 <= 44
 CustomerInventoryCapacity(3,1,6): I_customer(3)(1)(6)#71 <= 54
 CustomerInventoryCapacity(4,1,6): I_customer(4)(1)(6)#101 <= 22
 CustomerInventoryCapacity(5,1,6): I_customer(5)(1)(6)#131 <= 18
 CustomerInventoryCapacity(6,1,6): I_customer(6)(1)(6)#161 <= 47
 CustomerInventoryCapacity(7,1,6): I_customer(7)(1)(6)#191 <= 21
 CustomerInventoryCapacity(8,1,6): I_customer(8)(1)(6)#221 <= 55
 CustomerInventoryCapacity(9,1,6): I_customer(9)(1)(6)#251 <= 45
 CustomerInventoryCapacity(10,1,6): I_customer(10)(1)(6)#281 <= 48
 CustomerInventoryCapacity(11,1,6): I_customer(11)(1)(6)#311 <= 50
 CustomerInventoryCapacity(12,1,6): I_customer(12)(1)(6)#341 <= 44
 CustomerInventoryCapacity(3,2,6): I_customer(3)(2)(6)#81 <= 57
 CustomerInventoryCapacity(4,2,6): I_customer(4)(2)(6)#111 <= 22
 CustomerInventoryCapacity(5,2,6): I_customer(5)(2)(6)#141 <= 17
 CustomerInventoryCapacity(6,2,6): I_customer(6)(2)(6)#171 <= 49
 CustomerInventoryCapacity(7,2,6): I_customer(7)(2)(6)#201 <= 22
 CustomerInventoryCapacity(8,2,6): I_customer(8)(2)(6)#231 <= 51
 CustomerInventoryCapacity(9,2,6): I_customer(9)(2)(6)#261 <= 50
 CustomerInventoryCapacity(10,2,6): I_customer(10)(2)(6)#291 <= 46
 CustomerInventoryCapacity(11,2,6): I_customer(11)(2)(6)#321 <= 51
 CustomerInventoryCapacity(12,2,6): I_customer(12)(2)(6)#351 <= 44
 CustomerInventoryCapacity(3,3,6): I_customer(3)(3)(6)#91 <= 58
 CustomerInventoryCapacity(4,3,6): I_customer(4)(3)(6)#121 <= 21
 CustomerInventoryCapacity(5,3,6): I_customer(5)(3)(6)#151 <= 19
 CustomerInventoryCapacity(6,3,6): I_customer(6)(3)(6)#181 <= 51
 CustomerInventoryCapacity(7,3,6): I_customer(7)(3)(6)#211 <= 21
 CustomerInventoryCapacity(8,3,6): I_customer(8)(3)(6)#241 <= 55
 CustomerInventoryCapacity(9,3,6): I_customer(9)(3)(6)#271 <= 45
 CustomerInventoryCapacity(10,3,6): I_customer(10)(3)(6)#301 <= 46
 CustomerInventoryCapacity(11,3,6): I_customer(11)(3)(6)#331 <= 50
 CustomerInventoryCapacity(12,3,6): I_customer(12)(3)(6)#361 <= 40
 CustomerInventoryCapacity(3,1,7): I_customer(3)(1)(7)#72 <= 53
 CustomerInventoryCapacity(4,1,7): I_customer(4)(1)(7)#102 <= 22
 CustomerInventoryCapacity(5,1,7): I_customer(5)(1)(7)#132 <= 18
 CustomerInventoryCapacity(6,1,7): I_customer(6)(1)(7)#162 <= 47
 CustomerInventoryCapacity(7,1,7): I_customer(7)(1)(7)#192 <= 22
 CustomerInventoryCapacity(8,1,7): I_customer(8)(1)(7)#222 <= 52
 CustomerInventoryCapacity(9,1,7): I_customer(9)(1)(7)#252 <= 51
 CustomerInventoryCapacity(10,1,7): I_customer(10)(1)(7)#282 <= 48
 CustomerInventoryCapacity(11,1,7): I_customer(11)(1)(7)#312 <= 47
 CustomerInventoryCapacity(12,1,7): I_customer(12)(1)(7)#342 <= 44
 CustomerInventoryCapacity(3,2,7): I_customer(3)(2)(7)#82 <= 55
 CustomerInventoryCapacity(4,2,7): I_customer(4)(2)(7)#112 <= 20
 CustomerInventoryCapacity(5,2,7): I_customer(5)(2)(7)#142 <= 19
 CustomerInventoryCapacity(6,2,7): I_customer(6)(2)(7)#172 <= 49
 CustomerInventoryCapacity(7,2,7): I_customer(7)(2)(7)#202 <= 21
 CustomerInventoryCapacity(8,2,7): I_customer(8)(2)(7)#232 <= 52
 CustomerInventoryCapacity(9,2,7): I_customer(9)(2)(7)#262 <= 49
 CustomerInventoryCapacity(10,2,7): I_customer(10)(2)(7)#292 <= 47
 CustomerInventoryCapacity(11,2,7): I_customer(11)(2)(7)#322 <= 47
 CustomerInventoryCapacity(12,2,7): I_customer(12)(2)(7)#352 <= 42
 CustomerInventoryCapacity(3,3,7): I_customer(3)(3)(7)#92 <= 57
 CustomerInventoryCapacity(4,3,7): I_customer(4)(3)(7)#122 <= 21
 CustomerInventoryCapacity(5,3,7): I_customer(5)(3)(7)#152 <= 19
 CustomerInventoryCapacity(6,3,7): I_customer(6)(3)(7)#182 <= 47
 CustomerInventoryCapacity(7,3,7): I_customer(7)(3)(7)#212 <= 21
 CustomerInventoryCapacity(8,3,7): I_customer(8)(3)(7)#242 <= 56
 CustomerInventoryCapacity(9,3,7): I_customer(9)(3)(7)#272 <= 45
 CustomerInventoryCapacity(10,3,7): I_customer(10)(3)(7)#302 <= 46
 CustomerInventoryCapacity(11,3,7): I_customer(11)(3)(7)#332 <= 47
 CustomerInventoryCapacity(12,3,7): I_customer(12)(3)(7)#362 <= 40
 CustomerInventoryCapacity(3,1,8): I_customer(3)(1)(8)#73 <= 57
 CustomerInventoryCapacity(4,1,8): I_customer(4)(1)(8)#103 <= 22
 CustomerInventoryCapacity(5,1,8): I_customer(5)(1)(8)#133 <= 19
 CustomerInventoryCapacity(6,1,8): I_customer(6)(1)(8)#163 <= 47
 CustomerInventoryCapacity(7,1,8): I_customer(7)(1)(8)#193 <= 20
 CustomerInventoryCapacity(8,1,8): I_customer(8)(1)(8)#223 <= 57
 CustomerInventoryCapacity(9,1,8): I_customer(9)(1)(8)#253 <= 49
 CustomerInventoryCapacity(10,1,8): I_customer(10)(1)(8)#283 <= 51
 CustomerInventoryCapacity(11,1,8): I_customer(11)(1)(8)#313 <= 45
 CustomerInventoryCapacity(12,1,8): I_customer(12)(1)(8)#343 <= 44
 CustomerInventoryCapacity(3,2,8): I_customer(3)(2)(8)#83 <= 57
 CustomerInventoryCapacity(4,2,8): I_customer(4)(2)(8)#113 <= 22
 CustomerInventoryCapacity(5,2,8): I_customer(5)(2)(8)#143 <= 19
 CustomerInventoryCapacity(6,2,8): I_customer(6)(2)(8)#173 <= 47
 CustomerInventoryCapacity(7,2,8): I_customer(7)(2)(8)#203 <= 21
 CustomerInventoryCapacity(8,2,8): I_customer(8)(2)(8)#233 <= 53
 CustomerInventoryCapacity(9,2,8): I_customer(9)(2)(8)#263 <= 49
 CustomerInventoryCapacity(10,2,8): I_customer(10)(2)(8)#293 <= 51
 CustomerInventoryCapacity(11,2,8): I_customer(11)(2)(8)#323 <= 50
 CustomerInventoryCapacity(12,2,8): I_customer(12)(2)(8)#353 <= 39
 CustomerInventoryCapacity(3,3,8): I_customer(3)(3)(8)#93 <= 61
 CustomerInventoryCapacity(4,3,8): I_customer(4)(3)(8)#123 <= 21
 CustomerInventoryCapacity(5,3,8): I_customer(5)(3)(8)#153 <= 17
 CustomerInventoryCapacity(6,3,8): I_customer(6)(3)(8)#183 <= 48
 CustomerInventoryCapacity(7,3,8): I_customer(7)(3)(8)#213 <= 21
 CustomerInventoryCapacity(8,3,8): I_customer(8)(3)(8)#243 <= 51
 CustomerInventoryCapacity(9,3,8): I_customer(9)(3)(8)#273 <= 46
 CustomerInventoryCapacity(10,3,8): I_customer(10)(3)(8)#303 <= 47
 CustomerInventoryCapacity(11,3,8): I_customer(11)(3)(8)#333 <= 48
 CustomerInventoryCapacity(12,3,8): I_customer(12)(3)(8)#363 <= 42
 CustomerInventoryCapacity(3,1,9): I_customer(3)(1)(9)#74 <= 53
 CustomerInventoryCapacity(4,1,9): I_customer(4)(1)(9)#104 <= 20
 CustomerInventoryCapacity(5,1,9): I_customer(5)(1)(9)#134 <= 19
 CustomerInventoryCapacity(6,1,9): I_customer(6)(1)(9)#164 <= 48
 CustomerInventoryCapacity(7,1,9): I_customer(7)(1)(9)#194 <= 21
 CustomerInventoryCapacity(8,1,9): I_customer(8)(1)(9)#224 <= 53
 CustomerInventoryCapacity(9,1,9): I_customer(9)(1)(9)#254 <= 47
 CustomerInventoryCapacity(10,1,9): I_customer(10)(1)(9)#284 <= 50
 CustomerInventoryCapacity(11,1,9): I_customer(11)(1)(9)#314 <= 46
 CustomerInventoryCapacity(12,1,9): I_customer(12)(1)(9)#344 <= 45
 CustomerInventoryCapacity(3,2,9): I_customer(3)(2)(9)#84 <= 58
 CustomerInventoryCapacity(4,2,9): I_customer(4)(2)(9)#114 <= 21
 CustomerInventoryCapacity(5,2,9): I_customer(5)(2)(9)#144 <= 17
 CustomerInventoryCapacity(6,2,9): I_customer(6)(2)(9)#174 <= 48
 CustomerInventoryCapacity(7,2,9): I_customer(7)(2)(9)#204 <= 21
 CustomerInventoryCapacity(8,2,9): I_customer(8)(2)(9)#234 <= 55
 CustomerInventoryCapacity(9,2,9): I_customer(9)(2)(9)#264 <= 46
 CustomerInventoryCapacity(10,2,9): I_customer(10)(2)(9)#294 <= 50
 CustomerInventoryCapacity(11,2,9): I_customer(11)(2)(9)#324 <= 51
 CustomerInventoryCapacity(12,2,9): I_customer(12)(2)(9)#354 <= 40
 CustomerInventoryCapacity(3,3,9): I_customer(3)(3)(9)#94 <= 60
 CustomerInventoryCapacity(4,3,9): I_customer(4)(3)(9)#124 <= 20
 CustomerInventoryCapacity(5,3,9): I_customer(5)(3)(9)#154 <= 19
 CustomerInventoryCapacity(6,3,9): I_customer(6)(3)(9)#184 <= 50
 CustomerInventoryCapacity(7,3,9): I_customer(7)(3)(9)#214 <= 21
 CustomerInventoryCapacity(8,3,9): I_customer(8)(3)(9)#244 <= 57
 CustomerInventoryCapacity(9,3,9): I_customer(9)(3)(9)#274 <= 47
 CustomerInventoryCapacity(10,3,9): I_customer(10)(3)(9)#304 <= 45
 CustomerInventoryCapacity(11,3,9): I_customer(11)(3)(9)#334 <= 48
 CustomerInventoryCapacity(12,3,9): I_customer(12)(3)(9)#364 <= 43
 CustomerInventoryCapacity(3,1,10): I_customer(3)(1)(10)#75 <= 55
 CustomerInventoryCapacity(4,1,10): I_customer(4)(1)(10)#105 <= 22
 CustomerInventoryCapacity(5,1,10): I_customer(5)(1)(10)#135 <= 18
 CustomerInventoryCapacity(6,1,10): I_customer(6)(1)(10)#165 <= 49
 CustomerInventoryCapacity(7,1,10): I_customer(7)(1)(10)#195 <= 20
 CustomerInventoryCapacity(8,1,10): I_customer(8)(1)(10)#225 <= 54
 CustomerInventoryCapacity(9,1,10): I_customer(9)(1)(10)#255 <= 49
 CustomerInventoryCapacity(10,1,10): I_customer(10)(1)(10)#285 <= 51
 CustomerInventoryCapacity(11,1,10): I_customer(11)(1)(10)#315 <= 48
 CustomerInventoryCapacity(12,1,10): I_customer(12)(1)(10)#345 <= 44
 CustomerInventoryCapacity(3,2,10): I_customer(3)(2)(10)#85 <= 57
 CustomerInventoryCapacity(4,2,10): I_customer(4)(2)(10)#115 <= 22
 CustomerInventoryCapacity(5,2,10): I_customer(5)(2)(10)#145 <= 19
 CustomerInventoryCapacity(6,2,10): I_customer(6)(2)(10)#175 <= 46
 CustomerInventoryCapacity(7,2,10): I_customer(7)(2)(10)#205 <= 21
 CustomerInventoryCapacity(8,2,10): I_customer(8)(2)(10)#235 <= 56
 CustomerInventoryCapacity(9,2,10): I_customer(9)(2)(10)#265 <= 46
 CustomerInventoryCapacity(10,2,10): I_customer(10)(2)(10)#295 <= 48
 CustomerInventoryCapacity(11,2,10): I_customer(11)(2)(10)#325 <= 49
 CustomerInventoryCapacity(12,2,10): I_customer(12)(2)(10)#355 <= 45
 CustomerInventoryCapacity(3,3,10): I_customer(3)(3)(10)#95 <= 55
 CustomerInventoryCapacity(4,3,10): I_customer(4)(3)(10)#125 <= 21
 CustomerInventoryCapacity(5,3,10): I_customer(5)(3)(10)#155 <= 18
 CustomerInventoryCapacity(6,3,10): I_customer(6)(3)(10)#185 <= 47
 CustomerInventoryCapacity(7,3,10): I_customer(7)(3)(10)#215 <= 21
 CustomerInventoryCapacity(8,3,10): I_customer(8)(3)(10)#245 <= 55
 CustomerInventoryCapacity(9,3,10): I_customer(9)(3)(10)#275 <= 46
 CustomerInventoryCapacity(10,3,10): I_customer(10)(3)(10)#305 <= 48
 CustomerInventoryCapacity(11,3,10): I_customer(11)(3)(10)#335 <= 49
 CustomerInventoryCapacity(12,3,10): I_customer(12)(3)(10)#365 <= 40
 WarehouseDelivery(1,1,1):       q(1)(1)(1)#666 <= 124
 WarehouseDelivery(2,1,1):       q(2)(1)(1)#669 <= 124
 WarehouseDelivery(1,1,2):       q(1)(1)(2)#667 <= 124
 WarehouseDelivery(2,1,2):       q(2)(1)(2)#670  = 0
 WarehouseDelivery(1,1,3):       q(1)(1)(3)#668  = 0
 WarehouseDelivery(2,1,3):       q(2)(1)(3)#671  = 0
 VehicleCapacityPlant(1,1):      q(1)(1)(1)#666 + q(2)(1)(1)#669 <= 570
 VehicleCapacityPlant(1,2):      q(1)(1)(2)#667 + q(2)(1)(2)#670 <= 570
 VehicleCapacityPlant(1,3):      q(1)(1)(3)#668 + q(2)(1)(3)#671 <= 570
 CustomerVisit(3,1,1,1):         w(3)(1)(1)(1)#672  = 0
 CustomerVisit(4,1,1,1):         w(4)(1)(1)(1)#732  = 0
 CustomerVisit(5,1,1,1):         w(5)(1)(1)(1)#792  = 0
 CustomerVisit(6,1,1,1):         w(6)(1)(1)(1)#852  = 0
 CustomerVisit(7,1,1,1):         w(7)(1)(1)(1)#912  = 0
 CustomerVisit(8,1,1,1):         w(8)(1)(1)(1)#972  = 0
 CustomerVisit(12,1,1,1):        w(12)(1)(1)(1)#1212  = 0
 CustomerVisit(3,1,2,1):         w(3)(1)(2)(1)#682  = 0
 CustomerVisit(4,1,2,1):         w(4)(1)(2)(1)#742  = 0
 CustomerVisit(9,1,2,1):         w(9)(1)(2)(1)#1042  = 0
 CustomerVisit(12,1,2,1):        w(12)(1)(2)(1)#1222  = 0
 CustomerVisit(3,1,3,1):         w(3)(1)(3)(1)#692  = 0
 CustomerVisit(4,1,3,1):         w(4)(1)(3)(1)#752  = 0
 CustomerVisit(5,1,3,1):         w(5)(1)(3)(1)#812  = 0
 CustomerVisit(6,1,3,1):         w(6)(1)(3)(1)#872  = 0
 CustomerVisit(7,1,3,1):         w(7)(1)(3)(1)#932  = 0
 CustomerVisit(8,1,3,1):         w(8)(1)(3)(1)#992  = 0
 CustomerVisit(9,1,3,1):         w(9)(1)(3)(1)#1052  = 0
 CustomerVisit(10,1,3,1):        w(10)(1)(3)(1)#1112  = 0
 CustomerVisit(11,1,3,1):        w(11)(1)(3)(1)#1172  = 0
 CustomerVisit(12,1,3,1):        w(12)(1)(3)(1)#1232  = 0
 CustomerVisit(3,2,1,1):         w(3)(2)(1)(1)#702  = 0
 CustomerVisit(5,2,1,1):         w(5)(2)(1)(1)#822  = 0
 CustomerVisit(6,2,1,1):         w(6)(2)(1)(1)#882  = 0
 CustomerVisit(7,2,1,1):         w(7)(2)(1)(1)#942  = 0
 CustomerVisit(8,2,1,1):         w(8)(2)(1)(1)#1002  = 0
 CustomerVisit(9,2,1,1):         w(9)(2)(1)(1)#1062  = 0
 CustomerVisit(10,2,1,1):        w(10)(2)(1)(1)#1122  = 0
 CustomerVisit(11,2,1,1):        w(11)(2)(1)(1)#1182  = 0
 CustomerVisit(12,2,1,1):        w(12)(2)(1)(1)#1242  = 0
 CustomerVisit(3,2,2,1):         w(3)(2)(2)(1)#712  = 0
 CustomerVisit(4,2,2,1):         w(4)(2)(2)(1)#772  = 0
 CustomerVisit(5,2,2,1):         w(5)(2)(2)(1)#832  = 0
 CustomerVisit(6,2,2,1):         w(6)(2)(2)(1)#892  = 0
 CustomerVisit(7,2,2,1):         w(7)(2)(2)(1)#952  = 0
 CustomerVisit(8,2,2,1):         w(8)(2)(2)(1)#1012  = 0
 CustomerVisit(9,2,2,1):         w(9)(2)(2)(1)#1072  = 0
 CustomerVisit(10,2,2,1):        w(10)(2)(2)(1)#1132  = 0
 CustomerVisit(11,2,2,1):        w(11)(2)(2)(1)#1192  = 0
 CustomerVisit(12,2,2,1):        w(12)(2)(2)(1)#1252  = 0
 CustomerVisit(7,2,3,1):         w(7)(2)(3)(1)#962  = 0
 CustomerVisit(9,2,3,1):         w(9)(2)(3)(1)#1082  = 0
 CustomerVisit(10,2,3,1):        w(10)(2)(3)(1)#1142  = 0
 CustomerVisit(11,2,3,1):        w(11)(2)(3)(1)#1202  = 0
 CustomerVisit(3,1,1,2):         w(3)(1)(1)(2)#673  = 0
 CustomerVisit(4,1,1,2):         w(4)(1)(1)(2)#733  = 0
 CustomerVisit(5,1,1,2):         w(5)(1)(1)(2)#793  = 0
 CustomerVisit(6,1,1,2):         w(6)(1)(1)(2)#853  = 0
 CustomerVisit(7,1,1,2):         w(7)(1)(1)(2)#913  = 0
 CustomerVisit(8,1,1,2):         w(8)(1)(1)(2)#973  = 0
 CustomerVisit(9,1,1,2):         w(9)(1)(1)(2)#1033  = 0
 CustomerVisit(10,1,1,2):        w(10)(1)(1)(2)#1093  = 0
 CustomerVisit(12,1,1,2):        w(12)(1)(1)(2)#1213  = 0
 CustomerVisit(7,1,2,2):         w(7)(1)(2)(2)#923  = 0
 CustomerVisit(9,1,2,2):         w(9)(1)(2)(2)#1043  = 0
 CustomerVisit(11,1,2,2):        w(11)(1)(2)(2)#1163  = 0
 CustomerVisit(12,1,2,2):        w(12)(1)(2)(2)#1223  = 0
 CustomerVisit(3,2,1,2):         w(3)(2)(1)(2)#703  = 0
 CustomerVisit(4,2,1,2):         w(4)(2)(1)(2)#763  = 0
 CustomerVisit(5,2,1,2):         w(5)(2)(1)(2)#823  = 0
 CustomerVisit(7,2,1,2):         w(7)(2)(1)(2)#943  = 0
 CustomerVisit(8,2,1,2):         w(8)(2)(1)(2)#1003  = 0
 CustomerVisit(9,2,1,2):         w(9)(2)(1)(2)#1063  = 0
 CustomerVisit(10,2,1,2):        w(10)(2)(1)(2)#1123  = 0
 CustomerVisit(11,2,1,2):        w(11)(2)(1)(2)#1183  = 0
 CustomerVisit(12,2,1,2):        w(12)(2)(1)(2)#1243  = 0
 CustomerVisit(3,2,2,2):         w(3)(2)(2)(2)#713  = 0
 CustomerVisit(4,2,2,2):         w(4)(2)(2)(2)#773  = 0
 CustomerVisit(5,2,2,2):         w(5)(2)(2)(2)#833  = 0
 CustomerVisit(6,2,2,2):         w(6)(2)(2)(2)#893  = 0
 CustomerVisit(7,2,2,2):         w(7)(2)(2)(2)#953  = 0
 CustomerVisit(8,2,2,2):         w(8)(2)(2)(2)#1013  = 0
 CustomerVisit(9,2,2,2):         w(9)(2)(2)(2)#1073  = 0
 CustomerVisit(10,2,2,2):        w(10)(2)(2)(2)#1133  = 0
 CustomerVisit(11,2,2,2):        w(11)(2)(2)(2)#1193  = 0
 CustomerVisit(12,2,2,2):        w(12)(2)(2)(2)#1253  = 0
 CustomerVisit(3,2,3,2):         w(3)(2)(3)(2)#723  = 0
 CustomerVisit(4,2,3,2):         w(4)(2)(3)(2)#783  = 0
 CustomerVisit(5,2,3,2):         w(5)(2)(3)(2)#843  = 0
 CustomerVisit(6,2,3,2):         w(6)(2)(3)(2)#903  = 0
 CustomerVisit(7,2,3,2):         w(7)(2)(3)(2)#963  = 0
 CustomerVisit(8,2,3,2):         w(8)(2)(3)(2)#1023  = 0
 CustomerVisit(9,2,3,2):         w(9)(2)(3)(2)#1083  = 0
 CustomerVisit(10,2,3,2):        w(10)(2)(3)(2)#1143  = 0
 CustomerVisit(11,2,3,2):        w(11)(2)(3)(2)#1203  = 0
 CustomerVisit(12,2,3,2):        w(12)(2)(3)(2)#1263  = 0
 CustomerVisit(3,1,1,3):         w(3)(1)(1)(3)#674  = 0
 CustomerVisit(4,1,1,3):         w(4)(1)(1)(3)#734  = 0
 CustomerVisit(5,1,1,3):         w(5)(1)(1)(3)#794  = 0
 CustomerVisit(6,1,1,3):         w(6)(1)(1)(3)#854  = 0
 CustomerVisit(7,1,1,3):         w(7)(1)(1)(3)#914  = 0
 CustomerVisit(9,1,1,3):         w(9)(1)(1)(3)#1034  = 0
 CustomerVisit(11,1,1,3):        w(11)(1)(1)(3)#1154  = 0
 CustomerVisit(12,1,1,3):        w(12)(1)(1)(3)#1214  = 0
 CustomerVisit(6,1,2,3):         w(6)(1)(2)(3)#864  = 0
 CustomerVisit(9,1,2,3):         w(9)(1)(2)(3)#1044  = 0
 CustomerVisit(4,1,3,3):         w(4)(1)(3)(3)#754  = 0
 CustomerVisit(12,1,3,3):        w(12)(1)(3)(3)#1234  = 0
 CustomerVisit(3,2,1,3):         w(3)(2)(1)(3)#704  = 0
 CustomerVisit(5,2,1,3):         w(5)(2)(1)(3)#824  = 0
 CustomerVisit(7,2,1,3):         w(7)(2)(1)(3)#944  = 0
 CustomerVisit(8,2,1,3):         w(8)(2)(1)(3)#1004  = 0
 CustomerVisit(9,2,1,3):         w(9)(2)(1)(3)#1064  = 0
 CustomerVisit(10,2,1,3):        w(10)(2)(1)(3)#1124  = 0
 CustomerVisit(11,2,1,3):        w(11)(2)(1)(3)#1184  = 0
 CustomerVisit(12,2,1,3):        w(12)(2)(1)(3)#1244  = 0
 CustomerVisit(3,2,2,3):         w(3)(2)(2)(3)#714  = 0
 CustomerVisit(4,2,2,3):         w(4)(2)(2)(3)#774  = 0
 CustomerVisit(5,2,2,3):         w(5)(2)(2)(3)#834  = 0
 CustomerVisit(6,2,2,3):         w(6)(2)(2)(3)#894  = 0
 CustomerVisit(7,2,2,3):         w(7)(2)(2)(3)#954  = 0
 CustomerVisit(8,2,2,3):         w(8)(2)(2)(3)#1014  = 0
 CustomerVisit(9,2,2,3):         w(9)(2)(2)(3)#1074  = 0
 CustomerVisit(10,2,2,3):        w(10)(2)(2)(3)#1134  = 0
 CustomerVisit(11,2,2,3):        w(11)(2)(2)(3)#1194  = 0
 CustomerVisit(12,2,2,3):        w(12)(2)(2)(3)#1254  = 0
 CustomerVisit(3,2,3,3):         w(3)(2)(3)(3)#724  = 0
 CustomerVisit(4,2,3,3):         w(4)(2)(3)(3)#784  = 0
 CustomerVisit(5,2,3,3):         w(5)(2)(3)(3)#844  = 0
 CustomerVisit(6,2,3,3):         w(6)(2)(3)(3)#904  = 0
 CustomerVisit(7,2,3,3):         w(7)(2)(3)(3)#964  = 0
 CustomerVisit(8,2,3,3):         w(8)(2)(3)(3)#1024  = 0
 CustomerVisit(9,2,3,3):         w(9)(2)(3)(3)#1084  = 0
 CustomerVisit(10,2,3,3):        w(10)(2)(3)(3)#1144  = 0
 CustomerVisit(11,2,3,3):        w(11)(2)(3)(3)#1204  = 0
 CustomerVisit(12,2,3,3):        w(12)(2)(3)(3)#1264  = 0
 CustomerVisit(3,1,1,4):         w(3)(1)(1)(4)#675  = 0
 CustomerVisit(4,1,1,4):         w(4)(1)(1)(4)#735  = 0
 CustomerVisit(5,1,1,4):         w(5)(1)(1)(4)#795  = 0
 CustomerVisit(6,1,1,4):         w(6)(1)(1)(4)#855  = 0
 CustomerVisit(7,1,1,4):         w(7)(1)(1)(4)#915  = 0
 CustomerVisit(8,1,1,4):         w(8)(1)(1)(4)#975  = 0
 CustomerVisit(9,1,1,4):         w(9)(1)(1)(4)#1035  = 0
 CustomerVisit(11,1,1,4):        w(11)(1)(1)(4)#1155  = 0
 CustomerVisit(12,1,1,4):        w(12)(1)(1)(4)#1215  = 0
 CustomerVisit(3,1,2,4):         w(3)(1)(2)(4)#685  = 0
 CustomerVisit(4,1,2,4):         w(4)(1)(2)(4)#745  = 0
 CustomerVisit(5,1,2,4):         w(5)(1)(2)(4)#805  = 0
 CustomerVisit(6,1,2,4):         w(6)(1)(2)(4)#865  = 0
 CustomerVisit(3,1,3,4):         w(3)(1)(3)(4)#695  = 0
 CustomerVisit(4,1,3,4):         w(4)(1)(3)(4)#755  = 0
 CustomerVisit(5,1,3,4):         w(5)(1)(3)(4)#815  = 0
 CustomerVisit(6,1,3,4):         w(6)(1)(3)(4)#875  = 0
 CustomerVisit(7,1,3,4):         w(7)(1)(3)(4)#935  = 0
 CustomerVisit(9,1,3,4):         w(9)(1)(3)(4)#1055  = 0
 CustomerVisit(12,1,3,4):        w(12)(1)(3)(4)#1235  = 0
 CustomerVisit(7,2,1,4):         w(7)(2)(1)(4)#945  = 0
 CustomerVisit(8,2,1,4):         w(8)(2)(1)(4)#1005  = 0
 CustomerVisit(9,2,1,4):         w(9)(2)(1)(4)#1065  = 0
 CustomerVisit(10,2,1,4):        w(10)(2)(1)(4)#1125  = 0
 CustomerVisit(11,2,1,4):        w(11)(2)(1)(4)#1185  = 0
 CustomerVisit(12,2,1,4):        w(12)(2)(1)(4)#1245  = 0
 CustomerVisit(7,2,2,4):         w(7)(2)(2)(4)#955  = 0
 CustomerVisit(8,2,2,4):         w(8)(2)(2)(4)#1015  = 0
 CustomerVisit(9,2,2,4):         w(9)(2)(2)(4)#1075  = 0
 CustomerVisit(10,2,2,4):        w(10)(2)(2)(4)#1135  = 0
 CustomerVisit(11,2,2,4):        w(11)(2)(2)(4)#1195  = 0
 CustomerVisit(12,2,2,4):        w(12)(2)(2)(4)#1255  = 0
 CustomerVisit(3,2,3,4):         w(3)(2)(3)(4)#725  = 0
 CustomerVisit(4,2,3,4):         w(4)(2)(3)(4)#785  = 0
 CustomerVisit(5,2,3,4):         w(5)(2)(3)(4)#845  = 0
 CustomerVisit(6,2,3,4):         w(6)(2)(3)(4)#905  = 0
 CustomerVisit(7,2,3,4):         w(7)(2)(3)(4)#965  = 0
 CustomerVisit(8,2,3,4):         w(8)(2)(3)(4)#1025  = 0
 CustomerVisit(9,2,3,4):         w(9)(2)(3)(4)#1085  = 0
 CustomerVisit(10,2,3,4):        w(10)(2)(3)(4)#1145  = 0
 CustomerVisit(11,2,3,4):        w(11)(2)(3)(4)#1205  = 0
 CustomerVisit(12,2,3,4):        w(12)(2)(3)(4)#1265  = 0
 CustomerVisit(7,1,1,5):         w(7)(1)(1)(5)#916  = 0
 CustomerVisit(12,1,1,5):        w(12)(1)(1)(5)#1216  = 0
 CustomerVisit(3,1,2,5):         w(3)(1)(2)(5)#686  = 0
 CustomerVisit(4,1,2,5):         w(4)(1)(2)(5)#746  = 0
 CustomerVisit(5,1,2,5):         w(5)(1)(2)(5)#806  = 0
 CustomerVisit(6,1,2,5):         w(6)(1)(2)(5)#866  = 0
 CustomerVisit(7,1,2,5):         w(7)(1)(2)(5)#926  = 0
 CustomerVisit(8,1,2,5):         w(8)(1)(2)(5)#986  = 0
 CustomerVisit(12,1,2,5):        w(12)(1)(2)(5)#1226  = 0
 CustomerVisit(3,1,3,5):         w(3)(1)(3)(5)#696  = 0
 CustomerVisit(4,1,3,5):         w(4)(1)(3)(5)#756  = 0
 CustomerVisit(5,1,3,5):         w(5)(1)(3)(5)#816  = 0
 CustomerVisit(9,1,3,5):         w(9)(1)(3)(5)#1056  = 0
 CustomerVisit(10,1,3,5):        w(10)(1)(3)(5)#1116  = 0
 CustomerVisit(11,1,3,5):        w(11)(1)(3)(5)#1176  = 0
 CustomerVisit(3,2,1,5):         w(3)(2)(1)(5)#706  = 0
 CustomerVisit(4,2,1,5):         w(4)(2)(1)(5)#766  = 0
 CustomerVisit(5,2,1,5):         w(5)(2)(1)(5)#826  = 0
 CustomerVisit(6,2,1,5):         w(6)(2)(1)(5)#886  = 0
 CustomerVisit(7,2,1,5):         w(7)(2)(1)(5)#946  = 0
 CustomerVisit(8,2,1,5):         w(8)(2)(1)(5)#1006  = 0
 CustomerVisit(9,2,1,5):         w(9)(2)(1)(5)#1066  = 0
 CustomerVisit(10,2,1,5):        w(10)(2)(1)(5)#1126  = 0
 CustomerVisit(11,2,1,5):        w(11)(2)(1)(5)#1186  = 0
 CustomerVisit(12,2,1,5):        w(12)(2)(1)(5)#1246  = 0
 CustomerVisit(6,2,2,5):         w(6)(2)(2)(5)#896  = 0
 CustomerVisit(7,2,2,5):         w(7)(2)(2)(5)#956  = 0
 CustomerVisit(8,2,2,5):         w(8)(2)(2)(5)#1016  = 0
 CustomerVisit(9,2,2,5):         w(9)(2)(2)(5)#1076  = 0
 CustomerVisit(10,2,2,5):        w(10)(2)(2)(5)#1136  = 0
 CustomerVisit(11,2,2,5):        w(11)(2)(2)(5)#1196  = 0
 CustomerVisit(12,2,2,5):        w(12)(2)(2)(5)#1256  = 0
 CustomerVisit(3,2,3,5):         w(3)(2)(3)(5)#726  = 0
 CustomerVisit(4,2,3,5):         w(4)(2)(3)(5)#786  = 0
 CustomerVisit(5,2,3,5):         w(5)(2)(3)(5)#846  = 0
 CustomerVisit(6,2,3,5):         w(6)(2)(3)(5)#906  = 0
 CustomerVisit(7,2,3,5):         w(7)(2)(3)(5)#966  = 0
 CustomerVisit(8,2,3,5):         w(8)(2)(3)(5)#1026  = 0
 CustomerVisit(9,2,3,5):         w(9)(2)(3)(5)#1086  = 0
 CustomerVisit(10,2,3,5):        w(10)(2)(3)(5)#1146  = 0
 CustomerVisit(11,2,3,5):        w(11)(2)(3)(5)#1206  = 0
 CustomerVisit(12,2,3,5):        w(12)(2)(3)(5)#1266  = 0
 CustomerVisit(3,1,1,6):         w(3)(1)(1)(6)#677  = 0
 CustomerVisit(4,1,1,6):         w(4)(1)(1)(6)#737  = 0
 CustomerVisit(5,1,1,6):         w(5)(1)(1)(6)#797  = 0
 CustomerVisit(6,1,1,6):         w(6)(1)(1)(6)#857  = 0
 CustomerVisit(7,1,1,6):         w(7)(1)(1)(6)#917  = 0
 CustomerVisit(8,1,1,6):         w(8)(1)(1)(6)#977  = 0
 CustomerVisit(9,1,1,6):         w(9)(1)(1)(6)#1037  = 0
 CustomerVisit(10,1,1,6):        w(10)(1)(1)(6)#1097  = 0
 CustomerVisit(11,1,1,6):        w(11)(1)(1)(6)#1157  = 0
 CustomerVisit(12,1,1,6):        w(12)(1)(1)(6)#1217  = 0
 CustomerVisit(3,1,3,6):         w(3)(1)(3)(6)#697  = 0
 CustomerVisit(4,1,3,6):         w(4)(1)(3)(6)#757  = 0
 CustomerVisit(5,1,3,6):         w(5)(1)(3)(6)#817  = 0
 CustomerVisit(6,1,3,6):         w(6)(1)(3)(6)#877  = 0
 CustomerVisit(7,1,3,6):         w(7)(1)(3)(6)#937  = 0
 CustomerVisit(12,1,3,6):        w(12)(1)(3)(6)#1237  = 0
 CustomerVisit(7,2,1,6):         w(7)(2)(1)(6)#947  = 0
 CustomerVisit(8,2,1,6):         w(8)(2)(1)(6)#1007  = 0
 CustomerVisit(9,2,1,6):         w(9)(2)(1)(6)#1067  = 0
 CustomerVisit(10,2,1,6):        w(10)(2)(1)(6)#1127  = 0
 CustomerVisit(11,2,1,6):        w(11)(2)(1)(6)#1187  = 0
 CustomerVisit(12,2,1,6):        w(12)(2)(1)(6)#1247  = 0
 CustomerVisit(3,2,2,6):         w(3)(2)(2)(6)#717  = 0
 CustomerVisit(4,2,2,6):         w(4)(2)(2)(6)#777  = 0
 CustomerVisit(5,2,2,6):         w(5)(2)(2)(6)#837  = 0
 CustomerVisit(6,2,2,6):         w(6)(2)(2)(6)#897  = 0
 CustomerVisit(7,2,2,6):         w(7)(2)(2)(6)#957  = 0
 CustomerVisit(8,2,2,6):         w(8)(2)(2)(6)#1017  = 0
 CustomerVisit(9,2,2,6):         w(9)(2)(2)(6)#1077  = 0
 CustomerVisit(10,2,2,6):        w(10)(2)(2)(6)#1137  = 0
 CustomerVisit(11,2,2,6):        w(11)(2)(2)(6)#1197  = 0
 CustomerVisit(12,2,2,6):        w(12)(2)(2)(6)#1257  = 0
 CustomerVisit(3,2,3,6):         w(3)(2)(3)(6)#727  = 0
 CustomerVisit(4,2,3,6):         w(4)(2)(3)(6)#787  = 0
 CustomerVisit(5,2,3,6):         w(5)(2)(3)(6)#847  = 0
 CustomerVisit(6,2,3,6):         w(6)(2)(3)(6)#907  = 0
 CustomerVisit(7,2,3,6):         w(7)(2)(3)(6)#967  = 0
 CustomerVisit(8,2,3,6):         w(8)(2)(3)(6)#1027  = 0
 CustomerVisit(9,2,3,6):         w(9)(2)(3)(6)#1087  = 0
 CustomerVisit(10,2,3,6):        w(10)(2)(3)(6)#1147  = 0
 CustomerVisit(11,2,3,6):        w(11)(2)(3)(6)#1207  = 0
 CustomerVisit(12,2,3,6):        w(12)(2)(3)(6)#1267  = 0
 CustomerVisit(3,1,1,7):         w(3)(1)(1)(7)#678  = 0
 CustomerVisit(4,1,1,7):         w(4)(1)(1)(7)#738  = 0
 CustomerVisit(5,1,1,7):         w(5)(1)(1)(7)#798  = 0
 CustomerVisit(7,1,1,7):         w(7)(1)(1)(7)#918  = 0
 CustomerVisit(9,1,1,7):         w(9)(1)(1)(7)#1038  = 0
 CustomerVisit(10,1,1,7):        w(10)(1)(1)(7)#1098  = 0
 CustomerVisit(12,1,1,7):        w(12)(1)(1)(7)#1218  = 0
 CustomerVisit(12,1,2,7):        w(12)(1)(2)(7)#1228  = 0
 CustomerVisit(7,1,3,7):         w(7)(1)(3)(7)#938  = 0
 CustomerVisit(9,1,3,7):         w(9)(1)(3)(7)#1058  = 0
 CustomerVisit(3,2,1,7):         w(3)(2)(1)(7)#708  = 0
 CustomerVisit(4,2,1,7):         w(4)(2)(1)(7)#768  = 0
 CustomerVisit(5,2,1,7):         w(5)(2)(1)(7)#828  = 0
 CustomerVisit(6,2,1,7):         w(6)(2)(1)(7)#888  = 0
 CustomerVisit(7,2,1,7):         w(7)(2)(1)(7)#948  = 0
 CustomerVisit(8,2,1,7):         w(8)(2)(1)(7)#1008  = 0
 CustomerVisit(9,2,1,7):         w(9)(2)(1)(7)#1068  = 0
 CustomerVisit(10,2,1,7):        w(10)(2)(1)(7)#1128  = 0
 CustomerVisit(11,2,1,7):        w(11)(2)(1)(7)#1188  = 0
 CustomerVisit(12,2,1,7):        w(12)(2)(1)(7)#1248  = 0
 CustomerVisit(3,2,2,7):         w(3)(2)(2)(7)#718  = 0
 CustomerVisit(4,2,2,7):         w(4)(2)(2)(7)#778  = 0
 CustomerVisit(5,2,2,7):         w(5)(2)(2)(7)#838  = 0
 CustomerVisit(6,2,2,7):         w(6)(2)(2)(7)#898  = 0
 CustomerVisit(7,2,2,7):         w(7)(2)(2)(7)#958  = 0
 CustomerVisit(8,2,2,7):         w(8)(2)(2)(7)#1018  = 0
 CustomerVisit(9,2,2,7):         w(9)(2)(2)(7)#1078  = 0
 CustomerVisit(10,2,2,7):        w(10)(2)(2)(7)#1138  = 0
 CustomerVisit(11,2,2,7):        w(11)(2)(2)(7)#1198  = 0
 CustomerVisit(12,2,2,7):        w(12)(2)(2)(7)#1258  = 0
 CustomerVisit(3,2,3,7):         w(3)(2)(3)(7)#728  = 0
 CustomerVisit(4,2,3,7):         w(4)(2)(3)(7)#788  = 0
 CustomerVisit(5,2,3,7):         w(5)(2)(3)(7)#848  = 0
 CustomerVisit(6,2,3,7):         w(6)(2)(3)(7)#908  = 0
 CustomerVisit(7,2,3,7):         w(7)(2)(3)(7)#968  = 0
 CustomerVisit(8,2,3,7):         w(8)(2)(3)(7)#1028  = 0
 CustomerVisit(9,2,3,7):         w(9)(2)(3)(7)#1088  = 0
 CustomerVisit(10,2,3,7):        w(10)(2)(3)(7)#1148  = 0
 CustomerVisit(11,2,3,7):        w(11)(2)(3)(7)#1208  = 0
 CustomerVisit(12,2,3,7):        w(12)(2)(3)(7)#1268  = 0
 CustomerVisit(3,1,1,8):         w(3)(1)(1)(8)#679  = 0
 CustomerVisit(4,1,1,8):         w(4)(1)(1)(8)#739  = 0
 CustomerVisit(5,1,1,8):         w(5)(1)(1)(8)#799  = 0
 CustomerVisit(8,1,1,8):         w(8)(1)(1)(8)#979  = 0
 CustomerVisit(9,1,1,8):         w(9)(1)(1)(8)#1039  = 0
 CustomerVisit(12,1,1,8):        w(12)(1)(1)(8)#1219  = 0
 CustomerVisit(3,1,2,8):         w(3)(1)(2)(8)#689  = 0
 CustomerVisit(7,1,2,8):         w(7)(1)(2)(8)#929  = 0
 CustomerVisit(9,1,2,8):         w(9)(1)(2)(8)#1049  = 0
 CustomerVisit(10,1,2,8):        w(10)(1)(2)(8)#1109  = 0
 CustomerVisit(11,1,2,8):        w(11)(1)(2)(8)#1169  = 0
 CustomerVisit(3,1,3,8):         w(3)(1)(3)(8)#699  = 0
 CustomerVisit(4,1,3,8):         w(4)(1)(3)(8)#759  = 0
 CustomerVisit(5,1,3,8):         w(5)(1)(3)(8)#819  = 0
 CustomerVisit(6,1,3,8):         w(6)(1)(3)(8)#879  = 0
 CustomerVisit(7,1,3,8):         w(7)(1)(3)(8)#939  = 0
 CustomerVisit(8,1,3,8):         w(8)(1)(3)(8)#999  = 0
 CustomerVisit(9,1,3,8):         w(9)(1)(3)(8)#1059  = 0
 CustomerVisit(10,1,3,8):        w(10)(1)(3)(8)#1119  = 0
 CustomerVisit(11,1,3,8):        w(11)(1)(3)(8)#1179  = 0
 CustomerVisit(12,1,3,8):        w(12)(1)(3)(8)#1239  = 0
 CustomerVisit(3,2,1,8):         w(3)(2)(1)(8)#709  = 0
 CustomerVisit(4,2,1,8):         w(4)(2)(1)(8)#769  = 0
 CustomerVisit(5,2,1,8):         w(5)(2)(1)(8)#829  = 0
 CustomerVisit(6,2,1,8):         w(6)(2)(1)(8)#889  = 0
 CustomerVisit(7,2,1,8):         w(7)(2)(1)(8)#949  = 0
 CustomerVisit(8,2,1,8):         w(8)(2)(1)(8)#1009  = 0
 CustomerVisit(9,2,1,8):         w(9)(2)(1)(8)#1069  = 0
 CustomerVisit(10,2,1,8):        w(10)(2)(1)(8)#1129  = 0
 CustomerVisit(11,2,1,8):        w(11)(2)(1)(8)#1189  = 0
 CustomerVisit(12,2,1,8):        w(12)(2)(1)(8)#1249  = 0
 CustomerVisit(3,2,2,8):         w(3)(2)(2)(8)#719  = 0
 CustomerVisit(4,2,2,8):         w(4)(2)(2)(8)#779  = 0
 CustomerVisit(5,2,2,8):         w(5)(2)(2)(8)#839  = 0
 CustomerVisit(6,2,2,8):         w(6)(2)(2)(8)#899  = 0
 CustomerVisit(7,2,2,8):         w(7)(2)(2)(8)#959  = 0
 CustomerVisit(8,2,2,8):         w(8)(2)(2)(8)#1019  = 0
 CustomerVisit(9,2,2,8):         w(9)(2)(2)(8)#1079  = 0
 CustomerVisit(10,2,2,8):        w(10)(2)(2)(8)#1139  = 0
 CustomerVisit(11,2,2,8):        w(11)(2)(2)(8)#1199  = 0
 CustomerVisit(12,2,2,8):        w(12)(2)(2)(8)#1259  = 0
 CustomerVisit(7,2,3,8):         w(7)(2)(3)(8)#969  = 0
 CustomerVisit(3,1,1,9):         w(3)(1)(1)(9)#680  = 0
 CustomerVisit(4,1,1,9):         w(4)(1)(1)(9)#740  = 0
 CustomerVisit(5,1,1,9):         w(5)(1)(1)(9)#800  = 0
 CustomerVisit(6,1,1,9):         w(6)(1)(1)(9)#860  = 0
 CustomerVisit(7,1,1,9):         w(7)(1)(1)(9)#920  = 0
 CustomerVisit(10,1,1,9):        w(10)(1)(1)(9)#1100  = 0
 CustomerVisit(12,1,1,9):        w(12)(1)(1)(9)#1220  = 0
 CustomerVisit(4,1,2,9):         w(4)(1)(2)(9)#750  = 0
 CustomerVisit(9,1,2,9):         w(9)(1)(2)(9)#1050  = 0
 CustomerVisit(11,1,2,9):        w(11)(1)(2)(9)#1170  = 0
 CustomerVisit(3,1,3,9):         w(3)(1)(3)(9)#700  = 0
 CustomerVisit(4,1,3,9):         w(4)(1)(3)(9)#760  = 0
 CustomerVisit(5,1,3,9):         w(5)(1)(3)(9)#820  = 0
 CustomerVisit(6,1,3,9):         w(6)(1)(3)(9)#880  = 0
 CustomerVisit(7,1,3,9):         w(7)(1)(3)(9)#940  = 0
 CustomerVisit(8,1,3,9):         w(8)(1)(3)(9)#1000  = 0
 CustomerVisit(12,1,3,9):        w(12)(1)(3)(9)#1240  = 0
 CustomerVisit(6,2,1,9):         w(6)(2)(1)(9)#890  = 0
 CustomerVisit(7,2,1,9):         w(7)(2)(1)(9)#950  = 0
 CustomerVisit(8,2,1,9):         w(8)(2)(1)(9)#1010  = 0
 CustomerVisit(9,2,1,9):         w(9)(2)(1)(9)#1070  = 0
 CustomerVisit(10,2,1,9):        w(10)(2)(1)(9)#1130  = 0
 CustomerVisit(11,2,1,9):        w(11)(2)(1)(9)#1190  = 0
 CustomerVisit(12,2,1,9):        w(12)(2)(1)(9)#1250  = 0
 CustomerVisit(3,2,2,9):         w(3)(2)(2)(9)#720  = 0
 CustomerVisit(4,2,2,9):         w(4)(2)(2)(9)#780  = 0
 CustomerVisit(5,2,2,9):         w(5)(2)(2)(9)#840  = 0
 CustomerVisit(6,2,2,9):         w(6)(2)(2)(9)#900  = 0
 CustomerVisit(7,2,2,9):         w(7)(2)(2)(9)#960  = 0
 CustomerVisit(8,2,2,9):         w(8)(2)(2)(9)#1020  = 0
 CustomerVisit(9,2,2,9):         w(9)(2)(2)(9)#1080  = 0
 CustomerVisit(10,2,2,9):        w(10)(2)(2)(9)#1140  = 0
 CustomerVisit(11,2,2,9):        w(11)(2)(2)(9)#1200  = 0
 CustomerVisit(12,2,2,9):        w(12)(2)(2)(9)#1260  = 0
 CustomerVisit(3,2,3,9):         w(3)(2)(3)(9)#730  = 0
 CustomerVisit(5,2,3,9):         w(5)(2)(3)(9)#850  = 0
 CustomerVisit(7,2,3,9):         w(7)(2)(3)(9)#970  = 0
 CustomerVisit(8,2,3,9):         w(8)(2)(3)(9)#1030  = 0
 CustomerVisit(9,2,3,9):         w(9)(2)(3)(9)#1090  = 0
 CustomerVisit(10,2,3,9):        w(10)(2)(3)(9)#1150  = 0
 CustomerVisit(11,2,3,9):        w(11)(2)(3)(9)#1210  = 0
 CustomerVisit(12,2,3,9):        w(12)(2)(3)(9)#1270  = 0
 CustomerVisit(4,1,1,10):        w(4)(1)(1)(10)#741  = 0
 CustomerVisit(6,1,1,10):        w(6)(1)(1)(10)#861  = 0
 CustomerVisit(12,1,1,10):       w(12)(1)(1)(10)#1221  = 0
 CustomerVisit(7,1,2,10):        w(7)(1)(2)(10)#931  = 0
 CustomerVisit(3,1,3,10):        w(3)(1)(3)(10)#701  = 0
 CustomerVisit(4,1,3,10):        w(4)(1)(3)(10)#761  = 0
 CustomerVisit(5,1,3,10):        w(5)(1)(3)(10)#821  = 0
 CustomerVisit(6,1,3,10):        w(6)(1)(3)(10)#881  = 0
 CustomerVisit(7,1,3,10):        w(7)(1)(3)(10)#941  = 0
 CustomerVisit(8,1,3,10):        w(8)(1)(3)(10)#1001  = 0
 CustomerVisit(9,1,3,10):        w(9)(1)(3)(10)#1061  = 0
 CustomerVisit(10,1,3,10):       w(10)(1)(3)(10)#1121  = 0
 CustomerVisit(11,1,3,10):       w(11)(1)(3)(10)#1181  = 0
 CustomerVisit(12,1,3,10):       w(12)(1)(3)(10)#1241  = 0
 CustomerVisit(3,2,1,10):        w(3)(2)(1)(10)#711  = 0
 CustomerVisit(4,2,1,10):        w(4)(2)(1)(10)#771  = 0
 CustomerVisit(5,2,1,10):        w(5)(2)(1)(10)#831  = 0
 CustomerVisit(6,2,1,10):        w(6)(2)(1)(10)#891  = 0
 CustomerVisit(7,2,1,10):        w(7)(2)(1)(10)#951  = 0
 CustomerVisit(8,2,1,10):        w(8)(2)(1)(10)#1011  = 0
 CustomerVisit(9,2,1,10):        w(9)(2)(1)(10)#1071  = 0
 CustomerVisit(10,2,1,10):       w(10)(2)(1)(10)#1131  = 0
 CustomerVisit(11,2,1,10):       w(11)(2)(1)(10)#1191  = 0
 CustomerVisit(12,2,1,10):       w(12)(2)(1)(10)#1251  = 0
 CustomerVisit(3,2,2,10):        w(3)(2)(2)(10)#721  = 0
 CustomerVisit(4,2,2,10):        w(4)(2)(2)(10)#781  = 0
 CustomerVisit(5,2,2,10):        w(5)(2)(2)(10)#841  = 0
 CustomerVisit(6,2,2,10):        w(6)(2)(2)(10)#901  = 0
 CustomerVisit(7,2,2,10):        w(7)(2)(2)(10)#961  = 0
 CustomerVisit(8,2,2,10):        w(8)(2)(2)(10)#1021  = 0
 CustomerVisit(9,2,2,10):        w(9)(2)(2)(10)#1081  = 0
 CustomerVisit(10,2,2,10):       w(10)(2)(2)(10)#1141  = 0
 CustomerVisit(11,2,2,10):       w(11)(2)(2)(10)#1201  = 0
 CustomerVisit(12,2,2,10):       w(12)(2)(2)(10)#1261  = 0
 CustomerVisit(3,2,3,10):        w(3)(2)(3)(10)#731  = 0
 CustomerVisit(5,2,3,10):        w(5)(2)(3)(10)#851  = 0
 CustomerVisit(7,2,3,10):        w(7)(2)(3)(10)#971  = 0
 CustomerVisit(8,2,3,10):        w(8)(2)(3)(10)#1031  = 0
 CustomerVisit(9,2,3,10):        w(9)(2)(3)(10)#1091  = 0
 CustomerVisit(10,2,3,10):       w(10)(2)(3)(10)#1151  = 0
 CustomerVisit(11,2,3,10):       w(11)(2)(3)(10)#1211  = 0
 CustomerVisit(12,2,3,10):       w(12)(2)(3)(10)#1271  = 0
 VehicleCapacityWarehouse(1,1,1,1): w(3)(1)(1)(1)#672 + w(4)(1)(1)(1)#732
                                 + w(5)(1)(1)(1)#792 + w(6)(1)(1)(1)#852
                                 + w(7)(1)(1)(1)#912 + w(8)(1)(1)(1)#972
                                 + w(9)(1)(1)(1)#1032 + w(10)(1)(1)(1)#1092
                                 + w(11)(1)(1)(1)#1152 + w(12)(1)(1)(1)#1212
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,1): w(3)(2)(1)(1)#702 + w(4)(2)(1)(1)#762
                                 + w(5)(2)(1)(1)#822 + w(6)(2)(1)(1)#882
                                 + w(7)(2)(1)(1)#942 + w(8)(2)(1)(1)#1002
                                 + w(9)(2)(1)(1)#1062 + w(10)(2)(1)(1)#1122
                                 + w(11)(2)(1)(1)#1182 + w(12)(2)(1)(1)#1242
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,1): w(3)(1)(2)(1)#682 + w(4)(1)(2)(1)#742
                                 + w(5)(1)(2)(1)#802 + w(6)(1)(2)(1)#862
                                 + w(7)(1)(2)(1)#922 + w(8)(1)(2)(1)#982
                                 + w(9)(1)(2)(1)#1042 + w(10)(1)(2)(1)#1102
                                 + w(11)(1)(2)(1)#1162 + w(12)(1)(2)(1)#1222
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,1): w(3)(2)(2)(1)#712 + w(4)(2)(2)(1)#772
                                 + w(5)(2)(2)(1)#832 + w(6)(2)(2)(1)#892
                                 + w(7)(2)(2)(1)#952 + w(8)(2)(2)(1)#1012
                                 + w(9)(2)(2)(1)#1072 + w(10)(2)(2)(1)#1132
                                 + w(11)(2)(2)(1)#1192 + w(12)(2)(2)(1)#1252
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,1): w(3)(1)(3)(1)#692 + w(4)(1)(3)(1)#752
                                 + w(5)(1)(3)(1)#812 + w(6)(1)(3)(1)#872
                                 + w(7)(1)(3)(1)#932 + w(8)(1)(3)(1)#992
                                 + w(9)(1)(3)(1)#1052 + w(10)(1)(3)(1)#1112
                                 + w(11)(1)(3)(1)#1172 + w(12)(1)(3)(1)#1232
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,1): w(3)(2)(3)(1)#722 + w(4)(2)(3)(1)#782
                                 + w(5)(2)(3)(1)#842 + w(6)(2)(3)(1)#902
                                 + w(7)(2)(3)(1)#962 + w(8)(2)(3)(1)#1022
                                 + w(9)(2)(3)(1)#1082 + w(10)(2)(3)(1)#1142
                                 + w(11)(2)(3)(1)#1202 + w(12)(2)(3)(1)#1262
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,2): w(3)(1)(1)(2)#673 + w(4)(1)(1)(2)#733
                                 + w(5)(1)(1)(2)#793 + w(6)(1)(1)(2)#853
                                 + w(7)(1)(1)(2)#913 + w(8)(1)(1)(2)#973
                                 + w(9)(1)(1)(2)#1033 + w(10)(1)(1)(2)#1093
                                 + w(11)(1)(1)(2)#1153 + w(12)(1)(1)(2)#1213
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,2): w(3)(2)(1)(2)#703 + w(4)(2)(1)(2)#763
                                 + w(5)(2)(1)(2)#823 + w(6)(2)(1)(2)#883
                                 + w(7)(2)(1)(2)#943 + w(8)(2)(1)(2)#1003
                                 + w(9)(2)(1)(2)#1063 + w(10)(2)(1)(2)#1123
                                 + w(11)(2)(1)(2)#1183 + w(12)(2)(1)(2)#1243
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,2): w(3)(1)(2)(2)#683 + w(4)(1)(2)(2)#743
                                 + w(5)(1)(2)(2)#803 + w(6)(1)(2)(2)#863
                                 + w(7)(1)(2)(2)#923 + w(8)(1)(2)(2)#983
                                 + w(9)(1)(2)(2)#1043 + w(10)(1)(2)(2)#1103
                                 + w(11)(1)(2)(2)#1163 + w(12)(1)(2)(2)#1223
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,2): w(3)(2)(2)(2)#713 + w(4)(2)(2)(2)#773
                                 + w(5)(2)(2)(2)#833 + w(6)(2)(2)(2)#893
                                 + w(7)(2)(2)(2)#953 + w(8)(2)(2)(2)#1013
                                 + w(9)(2)(2)(2)#1073 + w(10)(2)(2)(2)#1133
                                 + w(11)(2)(2)(2)#1193 + w(12)(2)(2)(2)#1253
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,2): w(3)(1)(3)(2)#693 + w(4)(1)(3)(2)#753
                                 + w(5)(1)(3)(2)#813 + w(6)(1)(3)(2)#873
                                 + w(7)(1)(3)(2)#933 + w(8)(1)(3)(2)#993
                                 + w(9)(1)(3)(2)#1053 + w(10)(1)(3)(2)#1113
                                 + w(11)(1)(3)(2)#1173 + w(12)(1)(3)(2)#1233
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,2): w(3)(2)(3)(2)#723 + w(4)(2)(3)(2)#783
                                 + w(5)(2)(3)(2)#843 + w(6)(2)(3)(2)#903
                                 + w(7)(2)(3)(2)#963 + w(8)(2)(3)(2)#1023
                                 + w(9)(2)(3)(2)#1083 + w(10)(2)(3)(2)#1143
                                 + w(11)(2)(3)(2)#1203 + w(12)(2)(3)(2)#1263
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,3): w(3)(1)(1)(3)#674 + w(4)(1)(1)(3)#734
                                 + w(5)(1)(1)(3)#794 + w(6)(1)(1)(3)#854
                                 + w(7)(1)(1)(3)#914 + w(8)(1)(1)(3)#974
                                 + w(9)(1)(1)(3)#1034 + w(10)(1)(1)(3)#1094
                                 + w(11)(1)(1)(3)#1154 + w(12)(1)(1)(3)#1214
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,3): w(3)(2)(1)(3)#704 + w(4)(2)(1)(3)#764
                                 + w(5)(2)(1)(3)#824 + w(6)(2)(1)(3)#884
                                 + w(7)(2)(1)(3)#944 + w(8)(2)(1)(3)#1004
                                 + w(9)(2)(1)(3)#1064 + w(10)(2)(1)(3)#1124
                                 + w(11)(2)(1)(3)#1184 + w(12)(2)(1)(3)#1244
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,3): w(3)(1)(2)(3)#684 + w(4)(1)(2)(3)#744
                                 + w(5)(1)(2)(3)#804 + w(6)(1)(2)(3)#864
                                 + w(7)(1)(2)(3)#924 + w(8)(1)(2)(3)#984
                                 + w(9)(1)(2)(3)#1044 + w(10)(1)(2)(3)#1104
                                 + w(11)(1)(2)(3)#1164 + w(12)(1)(2)(3)#1224
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,3): w(3)(2)(2)(3)#714 + w(4)(2)(2)(3)#774
                                 + w(5)(2)(2)(3)#834 + w(6)(2)(2)(3)#894
                                 + w(7)(2)(2)(3)#954 + w(8)(2)(2)(3)#1014
                                 + w(9)(2)(2)(3)#1074 + w(10)(2)(2)(3)#1134
                                 + w(11)(2)(2)(3)#1194 + w(12)(2)(2)(3)#1254
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,3): w(3)(1)(3)(3)#694 + w(4)(1)(3)(3)#754
                                 + w(5)(1)(3)(3)#814 + w(6)(1)(3)(3)#874
                                 + w(7)(1)(3)(3)#934 + w(8)(1)(3)(3)#994
                                 + w(9)(1)(3)(3)#1054 + w(10)(1)(3)(3)#1114
                                 + w(11)(1)(3)(3)#1174 + w(12)(1)(3)(3)#1234
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,3): w(3)(2)(3)(3)#724 + w(4)(2)(3)(3)#784
                                 + w(5)(2)(3)(3)#844 + w(6)(2)(3)(3)#904
                                 + w(7)(2)(3)(3)#964 + w(8)(2)(3)(3)#1024
                                 + w(9)(2)(3)(3)#1084 + w(10)(2)(3)(3)#1144
                                 + w(11)(2)(3)(3)#1204 + w(12)(2)(3)(3)#1264
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,4): w(3)(1)(1)(4)#675 + w(4)(1)(1)(4)#735
                                 + w(5)(1)(1)(4)#795 + w(6)(1)(1)(4)#855
                                 + w(7)(1)(1)(4)#915 + w(8)(1)(1)(4)#975
                                 + w(9)(1)(1)(4)#1035 + w(10)(1)(1)(4)#1095
                                 + w(11)(1)(1)(4)#1155 + w(12)(1)(1)(4)#1215
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,4): w(3)(2)(1)(4)#705 + w(4)(2)(1)(4)#765
                                 + w(5)(2)(1)(4)#825 + w(6)(2)(1)(4)#885
                                 + w(7)(2)(1)(4)#945 + w(8)(2)(1)(4)#1005
                                 + w(9)(2)(1)(4)#1065 + w(10)(2)(1)(4)#1125
                                 + w(11)(2)(1)(4)#1185 + w(12)(2)(1)(4)#1245
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,4): w(3)(1)(2)(4)#685 + w(4)(1)(2)(4)#745
                                 + w(5)(1)(2)(4)#805 + w(6)(1)(2)(4)#865
                                 + w(7)(1)(2)(4)#925 + w(8)(1)(2)(4)#985
                                 + w(9)(1)(2)(4)#1045 + w(10)(1)(2)(4)#1105
                                 + w(11)(1)(2)(4)#1165 + w(12)(1)(2)(4)#1225
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,4): w(3)(2)(2)(4)#715 + w(4)(2)(2)(4)#775
                                 + w(5)(2)(2)(4)#835 + w(6)(2)(2)(4)#895
                                 + w(7)(2)(2)(4)#955 + w(8)(2)(2)(4)#1015
                                 + w(9)(2)(2)(4)#1075 + w(10)(2)(2)(4)#1135
                                 + w(11)(2)(2)(4)#1195 + w(12)(2)(2)(4)#1255
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,4): w(3)(1)(3)(4)#695 + w(4)(1)(3)(4)#755
                                 + w(5)(1)(3)(4)#815 + w(6)(1)(3)(4)#875
                                 + w(7)(1)(3)(4)#935 + w(8)(1)(3)(4)#995
                                 + w(9)(1)(3)(4)#1055 + w(10)(1)(3)(4)#1115
                                 + w(11)(1)(3)(4)#1175 + w(12)(1)(3)(4)#1235
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,4): w(3)(2)(3)(4)#725 + w(4)(2)(3)(4)#785
                                 + w(5)(2)(3)(4)#845 + w(6)(2)(3)(4)#905
                                 + w(7)(2)(3)(4)#965 + w(8)(2)(3)(4)#1025
                                 + w(9)(2)(3)(4)#1085 + w(10)(2)(3)(4)#1145
                                 + w(11)(2)(3)(4)#1205 + w(12)(2)(3)(4)#1265
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,5): w(3)(1)(1)(5)#676 + w(4)(1)(1)(5)#736
                                 + w(5)(1)(1)(5)#796 + w(6)(1)(1)(5)#856
                                 + w(7)(1)(1)(5)#916 + w(8)(1)(1)(5)#976
                                 + w(9)(1)(1)(5)#1036 + w(10)(1)(1)(5)#1096
                                 + w(11)(1)(1)(5)#1156 + w(12)(1)(1)(5)#1216
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,5): w(3)(2)(1)(5)#706 + w(4)(2)(1)(5)#766
                                 + w(5)(2)(1)(5)#826 + w(6)(2)(1)(5)#886
                                 + w(7)(2)(1)(5)#946 + w(8)(2)(1)(5)#1006
                                 + w(9)(2)(1)(5)#1066 + w(10)(2)(1)(5)#1126
                                 + w(11)(2)(1)(5)#1186 + w(12)(2)(1)(5)#1246
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,5): w(3)(1)(2)(5)#686 + w(4)(1)(2)(5)#746
                                 + w(5)(1)(2)(5)#806 + w(6)(1)(2)(5)#866
                                 + w(7)(1)(2)(5)#926 + w(8)(1)(2)(5)#986
                                 + w(9)(1)(2)(5)#1046 + w(10)(1)(2)(5)#1106
                                 + w(11)(1)(2)(5)#1166 + w(12)(1)(2)(5)#1226
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,5): w(3)(2)(2)(5)#716 + w(4)(2)(2)(5)#776
                                 + w(5)(2)(2)(5)#836 + w(6)(2)(2)(5)#896
                                 + w(7)(2)(2)(5)#956 + w(8)(2)(2)(5)#1016
                                 + w(9)(2)(2)(5)#1076 + w(10)(2)(2)(5)#1136
                                 + w(11)(2)(2)(5)#1196 + w(12)(2)(2)(5)#1256
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,5): w(3)(1)(3)(5)#696 + w(4)(1)(3)(5)#756
                                 + w(5)(1)(3)(5)#816 + w(6)(1)(3)(5)#876
                                 + w(7)(1)(3)(5)#936 + w(8)(1)(3)(5)#996
                                 + w(9)(1)(3)(5)#1056 + w(10)(1)(3)(5)#1116
                                 + w(11)(1)(3)(5)#1176 + w(12)(1)(3)(5)#1236
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,5): w(3)(2)(3)(5)#726 + w(4)(2)(3)(5)#786
                                 + w(5)(2)(3)(5)#846 + w(6)(2)(3)(5)#906
                                 + w(7)(2)(3)(5)#966 + w(8)(2)(3)(5)#1026
                                 + w(9)(2)(3)(5)#1086 + w(10)(2)(3)(5)#1146
                                 + w(11)(2)(3)(5)#1206 + w(12)(2)(3)(5)#1266
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,6): w(3)(1)(1)(6)#677 + w(4)(1)(1)(6)#737
                                 + w(5)(1)(1)(6)#797 + w(6)(1)(1)(6)#857
                                 + w(7)(1)(1)(6)#917 + w(8)(1)(1)(6)#977
                                 + w(9)(1)(1)(6)#1037 + w(10)(1)(1)(6)#1097
                                 + w(11)(1)(1)(6)#1157 + w(12)(1)(1)(6)#1217
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,6): w(3)(2)(1)(6)#707 + w(4)(2)(1)(6)#767
                                 + w(5)(2)(1)(6)#827 + w(6)(2)(1)(6)#887
                                 + w(7)(2)(1)(6)#947 + w(8)(2)(1)(6)#1007
                                 + w(9)(2)(1)(6)#1067 + w(10)(2)(1)(6)#1127
                                 + w(11)(2)(1)(6)#1187 + w(12)(2)(1)(6)#1247
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,6): w(3)(1)(2)(6)#687 + w(4)(1)(2)(6)#747
                                 + w(5)(1)(2)(6)#807 + w(6)(1)(2)(6)#867
                                 + w(7)(1)(2)(6)#927 + w(8)(1)(2)(6)#987
                                 + w(9)(1)(2)(6)#1047 + w(10)(1)(2)(6)#1107
                                 + w(11)(1)(2)(6)#1167 + w(12)(1)(2)(6)#1227
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,6): w(3)(2)(2)(6)#717 + w(4)(2)(2)(6)#777
                                 + w(5)(2)(2)(6)#837 + w(6)(2)(2)(6)#897
                                 + w(7)(2)(2)(6)#957 + w(8)(2)(2)(6)#1017
                                 + w(9)(2)(2)(6)#1077 + w(10)(2)(2)(6)#1137
                                 + w(11)(2)(2)(6)#1197 + w(12)(2)(2)(6)#1257
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,6): w(3)(1)(3)(6)#697 + w(4)(1)(3)(6)#757
                                 + w(5)(1)(3)(6)#817 + w(6)(1)(3)(6)#877
                                 + w(7)(1)(3)(6)#937 + w(8)(1)(3)(6)#997
                                 + w(9)(1)(3)(6)#1057 + w(10)(1)(3)(6)#1117
                                 + w(11)(1)(3)(6)#1177 + w(12)(1)(3)(6)#1237
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,6): w(3)(2)(3)(6)#727 + w(4)(2)(3)(6)#787
                                 + w(5)(2)(3)(6)#847 + w(6)(2)(3)(6)#907
                                 + w(7)(2)(3)(6)#967 + w(8)(2)(3)(6)#1027
                                 + w(9)(2)(3)(6)#1087 + w(10)(2)(3)(6)#1147
                                 + w(11)(2)(3)(6)#1207 + w(12)(2)(3)(6)#1267
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,7): w(3)(1)(1)(7)#678 + w(4)(1)(1)(7)#738
                                 + w(5)(1)(1)(7)#798 + w(6)(1)(1)(7)#858
                                 + w(7)(1)(1)(7)#918 + w(8)(1)(1)(7)#978
                                 + w(9)(1)(1)(7)#1038 + w(10)(1)(1)(7)#1098
                                 + w(11)(1)(1)(7)#1158 + w(12)(1)(1)(7)#1218
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,7): w(3)(2)(1)(7)#708 + w(4)(2)(1)(7)#768
                                 + w(5)(2)(1)(7)#828 + w(6)(2)(1)(7)#888
                                 + w(7)(2)(1)(7)#948 + w(8)(2)(1)(7)#1008
                                 + w(9)(2)(1)(7)#1068 + w(10)(2)(1)(7)#1128
                                 + w(11)(2)(1)(7)#1188 + w(12)(2)(1)(7)#1248
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,7): w(3)(1)(2)(7)#688 + w(4)(1)(2)(7)#748
                                 + w(5)(1)(2)(7)#808 + w(6)(1)(2)(7)#868
                                 + w(7)(1)(2)(7)#928 + w(8)(1)(2)(7)#988
                                 + w(9)(1)(2)(7)#1048 + w(10)(1)(2)(7)#1108
                                 + w(11)(1)(2)(7)#1168 + w(12)(1)(2)(7)#1228
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,7): w(3)(2)(2)(7)#718 + w(4)(2)(2)(7)#778
                                 + w(5)(2)(2)(7)#838 + w(6)(2)(2)(7)#898
                                 + w(7)(2)(2)(7)#958 + w(8)(2)(2)(7)#1018
                                 + w(9)(2)(2)(7)#1078 + w(10)(2)(2)(7)#1138
                                 + w(11)(2)(2)(7)#1198 + w(12)(2)(2)(7)#1258
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,7): w(3)(1)(3)(7)#698 + w(4)(1)(3)(7)#758
                                 + w(5)(1)(3)(7)#818 + w(6)(1)(3)(7)#878
                                 + w(7)(1)(3)(7)#938 + w(8)(1)(3)(7)#998
                                 + w(9)(1)(3)(7)#1058 + w(10)(1)(3)(7)#1118
                                 + w(11)(1)(3)(7)#1178 + w(12)(1)(3)(7)#1238
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,7): w(3)(2)(3)(7)#728 + w(4)(2)(3)(7)#788
                                 + w(5)(2)(3)(7)#848 + w(6)(2)(3)(7)#908
                                 + w(7)(2)(3)(7)#968 + w(8)(2)(3)(7)#1028
                                 + w(9)(2)(3)(7)#1088 + w(10)(2)(3)(7)#1148
                                 + w(11)(2)(3)(7)#1208 + w(12)(2)(3)(7)#1268
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,8): w(3)(1)(1)(8)#679 + w(4)(1)(1)(8)#739
                                 + w(5)(1)(1)(8)#799 + w(6)(1)(1)(8)#859
                                 + w(7)(1)(1)(8)#919 + w(8)(1)(1)(8)#979
                                 + w(9)(1)(1)(8)#1039 + w(10)(1)(1)(8)#1099
                                 + w(11)(1)(1)(8)#1159 + w(12)(1)(1)(8)#1219
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,8): w(3)(2)(1)(8)#709 + w(4)(2)(1)(8)#769
                                 + w(5)(2)(1)(8)#829 + w(6)(2)(1)(8)#889
                                 + w(7)(2)(1)(8)#949 + w(8)(2)(1)(8)#1009
                                 + w(9)(2)(1)(8)#1069 + w(10)(2)(1)(8)#1129
                                 + w(11)(2)(1)(8)#1189 + w(12)(2)(1)(8)#1249
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,8): w(3)(1)(2)(8)#689 + w(4)(1)(2)(8)#749
                                 + w(5)(1)(2)(8)#809 + w(6)(1)(2)(8)#869
                                 + w(7)(1)(2)(8)#929 + w(8)(1)(2)(8)#989
                                 + w(9)(1)(2)(8)#1049 + w(10)(1)(2)(8)#1109
                                 + w(11)(1)(2)(8)#1169 + w(12)(1)(2)(8)#1229
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,8): w(3)(2)(2)(8)#719 + w(4)(2)(2)(8)#779
                                 + w(5)(2)(2)(8)#839 + w(6)(2)(2)(8)#899
                                 + w(7)(2)(2)(8)#959 + w(8)(2)(2)(8)#1019
                                 + w(9)(2)(2)(8)#1079 + w(10)(2)(2)(8)#1139
                                 + w(11)(2)(2)(8)#1199 + w(12)(2)(2)(8)#1259
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,8): w(3)(1)(3)(8)#699 + w(4)(1)(3)(8)#759
                                 + w(5)(1)(3)(8)#819 + w(6)(1)(3)(8)#879
                                 + w(7)(1)(3)(8)#939 + w(8)(1)(3)(8)#999
                                 + w(9)(1)(3)(8)#1059 + w(10)(1)(3)(8)#1119
                                 + w(11)(1)(3)(8)#1179 + w(12)(1)(3)(8)#1239
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,8): w(3)(2)(3)(8)#729 + w(4)(2)(3)(8)#789
                                 + w(5)(2)(3)(8)#849 + w(6)(2)(3)(8)#909
                                 + w(7)(2)(3)(8)#969 + w(8)(2)(3)(8)#1029
                                 + w(9)(2)(3)(8)#1089 + w(10)(2)(3)(8)#1149
                                 + w(11)(2)(3)(8)#1209 + w(12)(2)(3)(8)#1269
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,9): w(3)(1)(1)(9)#680 + w(4)(1)(1)(9)#740
                                 + w(5)(1)(1)(9)#800 + w(6)(1)(1)(9)#860
                                 + w(7)(1)(1)(9)#920 + w(8)(1)(1)(9)#980
                                 + w(9)(1)(1)(9)#1040 + w(10)(1)(1)(9)#1100
                                 + w(11)(1)(1)(9)#1160 + w(12)(1)(1)(9)#1220
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,9): w(3)(2)(1)(9)#710 + w(4)(2)(1)(9)#770
                                 + w(5)(2)(1)(9)#830 + w(6)(2)(1)(9)#890
                                 + w(7)(2)(1)(9)#950 + w(8)(2)(1)(9)#1010
                                 + w(9)(2)(1)(9)#1070 + w(10)(2)(1)(9)#1130
                                 + w(11)(2)(1)(9)#1190 + w(12)(2)(1)(9)#1250
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,9): w(3)(1)(2)(9)#690 + w(4)(1)(2)(9)#750
                                 + w(5)(1)(2)(9)#810 + w(6)(1)(2)(9)#870
                                 + w(7)(1)(2)(9)#930 + w(8)(1)(2)(9)#990
                                 + w(9)(1)(2)(9)#1050 + w(10)(1)(2)(9)#1110
                                 + w(11)(1)(2)(9)#1170 + w(12)(1)(2)(9)#1230
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,9): w(3)(2)(2)(9)#720 + w(4)(2)(2)(9)#780
                                 + w(5)(2)(2)(9)#840 + w(6)(2)(2)(9)#900
                                 + w(7)(2)(2)(9)#960 + w(8)(2)(2)(9)#1020
                                 + w(9)(2)(2)(9)#1080 + w(10)(2)(2)(9)#1140
                                 + w(11)(2)(2)(9)#1200 + w(12)(2)(2)(9)#1260
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,9): w(3)(1)(3)(9)#700 + w(4)(1)(3)(9)#760
                                 + w(5)(1)(3)(9)#820 + w(6)(1)(3)(9)#880
                                 + w(7)(1)(3)(9)#940 + w(8)(1)(3)(9)#1000
                                 + w(9)(1)(3)(9)#1060 + w(10)(1)(3)(9)#1120
                                 + w(11)(1)(3)(9)#1180 + w(12)(1)(3)(9)#1240
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,9): w(3)(2)(3)(9)#730 + w(4)(2)(3)(9)#790
                                 + w(5)(2)(3)(9)#850 + w(6)(2)(3)(9)#910
                                 + w(7)(2)(3)(9)#970 + w(8)(2)(3)(9)#1030
                                 + w(9)(2)(3)(9)#1090 + w(10)(2)(3)(9)#1150
                                 + w(11)(2)(3)(9)#1210 + w(12)(2)(3)(9)#1270
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,10): w(3)(1)(1)(10)#681 + w(4)(1)(1)(10)#741
                                 + w(5)(1)(1)(10)#801 + w(6)(1)(1)(10)#861
                                 + w(7)(1)(1)(10)#921 + w(8)(1)(1)(10)#981
                                 + w(9)(1)(1)(10)#1041 + w(10)(1)(1)(10)#1101
                                 + w(11)(1)(1)(10)#1161 + w(12)(1)(1)(10)#1221
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,10): w(3)(2)(1)(10)#711 + w(4)(2)(1)(10)#771
                                 + w(5)(2)(1)(10)#831 + w(6)(2)(1)(10)#891
                                 + w(7)(2)(1)(10)#951 + w(8)(2)(1)(10)#1011
                                 + w(9)(2)(1)(10)#1071 + w(10)(2)(1)(10)#1131
                                 + w(11)(2)(1)(10)#1191 + w(12)(2)(1)(10)#1251
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,10): w(3)(1)(2)(10)#691 + w(4)(1)(2)(10)#751
                                 + w(5)(1)(2)(10)#811 + w(6)(1)(2)(10)#871
                                 + w(7)(1)(2)(10)#931 + w(8)(1)(2)(10)#991
                                 + w(9)(1)(2)(10)#1051 + w(10)(1)(2)(10)#1111
                                 + w(11)(1)(2)(10)#1171 + w(12)(1)(2)(10)#1231
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,10): w(3)(2)(2)(10)#721 + w(4)(2)(2)(10)#781
                                 + w(5)(2)(2)(10)#841 + w(6)(2)(2)(10)#901
                                 + w(7)(2)(2)(10)#961 + w(8)(2)(2)(10)#1021
                                 + w(9)(2)(2)(10)#1081 + w(10)(2)(2)(10)#1141
                                 + w(11)(2)(2)(10)#1201 + w(12)(2)(2)(10)#1261
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,10): w(3)(1)(3)(10)#701 + w(4)(1)(3)(10)#761
                                 + w(5)(1)(3)(10)#821 + w(6)(1)(3)(10)#881
                                 + w(7)(1)(3)(10)#941 + w(8)(1)(3)(10)#1001
                                 + w(9)(1)(3)(10)#1061 + w(10)(1)(3)(10)#1121
                                 + w(11)(1)(3)(10)#1181 + w(12)(1)(3)(10)#1241
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,10): w(3)(2)(3)(10)#731 + w(4)(2)(3)(10)#791
                                 + w(5)(2)(3)(10)#851 + w(6)(2)(3)(10)#911
                                 + w(7)(2)(3)(10)#971 + w(8)(2)(3)(10)#1031
                                 + w(9)(2)(3)(10)#1091 + w(10)(2)(3)(10)#1151
                                 + w(11)(2)(3)(10)#1211 + w(12)(2)(3)(10)#1271
                                 <= 228
Bounds
 0 <= p(1)#0 <= 270
 0 <= p(2)#1 <= 270
 0 <= p(3)#2 <= 270
 0 <= I_plant(1)#3 <= 135
 0 <= I_plant(2)#4 <= 135
 0 <= I_plant(3)#5 <= 135
 0 <= I_warehouse(1)(1)(1)#6 <= 124
 0 <= I_warehouse(1)(1)(2)#7 <= 124
 0 <= I_warehouse(1)(1)(3)#8 <= 124
 0 <= I_warehouse(1)(1)(4)#9 <= 124
 0 <= I_warehouse(1)(1)(5)#10 <= 124
 0 <= I_warehouse(1)(1)(6)#11 <= 124
 0 <= I_warehouse(1)(1)(7)#12 <= 124
 0 <= I_warehouse(1)(1)(8)#13 <= 124
 0 <= I_warehouse(1)(1)(9)#14 <= 124
 0 <= I_warehouse(1)(1)(10)#15 <= 124
 0 <= I_warehouse(1)(2)(1)#16 <= 124
 0 <= I_warehouse(1)(2)(2)#17 <= 124
 0 <= I_warehouse(1)(2)(3)#18 <= 124
 0 <= I_warehouse(1)(2)(4)#19 <= 124
 0 <= I_warehouse(1)(2)(5)#20 <= 124
 0 <= I_warehouse(1)(2)(6)#21 <= 124
 0 <= I_warehouse(1)(2)(7)#22 <= 124
 0 <= I_warehouse(1)(2)(8)#23 <= 124
 0 <= I_warehouse(1)(2)(9)#24 <= 124
 0 <= I_warehouse(1)(2)(10)#25 <= 124
 0 <= I_warehouse(1)(3)(1)#26 <= 124
 0 <= I_warehouse(1)(3)(2)#27 <= 124
 0 <= I_warehouse(1)(3)(3)#28 <= 124
 0 <= I_warehouse(1)(3)(4)#29 <= 124
 0 <= I_warehouse(1)(3)(5)#30 <= 124
 0 <= I_warehouse(1)(3)(6)#31 <= 124
 0 <= I_warehouse(1)(3)(7)#32 <= 124
 0 <= I_warehouse(1)(3)(8)#33 <= 124
 0 <= I_warehouse(1)(3)(9)#34 <= 124
 0 <= I_warehouse(1)(3)(10)#35 <= 124
 0 <= I_warehouse(2)(1)(1)#36 <= 124
 0 <= I_warehouse(2)(1)(2)#37 <= 124
 0 <= I_warehouse(2)(1)(3)#38 <= 124
 0 <= I_warehouse(2)(1)(4)#39 <= 124
 0 <= I_warehouse(2)(1)(5)#40 <= 124
 0 <= I_warehouse(2)(1)(6)#41 <= 124
 0 <= I_warehouse(2)(1)(7)#42 <= 124
 0 <= I_warehouse(2)(1)(8)#43 <= 124
 0 <= I_warehouse(2)(1)(9)#44 <= 124
 0 <= I_warehouse(2)(1)(10)#45 <= 124
 0 <= I_warehouse(2)(2)(1)#46 <= 124
 0 <= I_warehouse(2)(2)(2)#47 <= 124
 0 <= I_warehouse(2)(2)(3)#48 <= 124
 0 <= I_warehouse(2)(2)(4)#49 <= 124
 0 <= I_warehouse(2)(2)(5)#50 <= 124
 0 <= I_warehouse(2)(2)(6)#51 <= 124
 0 <= I_warehouse(2)(2)(7)#52 <= 124
 0 <= I_warehouse(2)(2)(8)#53 <= 124
 0 <= I_warehouse(2)(2)(9)#54 <= 124
 0 <= I_warehouse(2)(2)(10)#55 <= 124
 0 <= I_warehouse(2)(3)(1)#56 <= 124
 0 <= I_warehouse(2)(3)(2)#57 <= 124
 0 <= I_warehouse(2)(3)(3)#58 <= 124
 0 <= I_warehouse(2)(3)(4)#59 <= 124
 0 <= I_warehouse(2)(3)(5)#60 <= 124
 0 <= I_warehouse(2)(3)(6)#61 <= 124
 0 <= I_warehouse(2)(3)(7)#62 <= 124
 0 <= I_warehouse(2)(3)(8)#63 <= 124
 0 <= I_warehouse(2)(3)(9)#64 <= 124
 0 <= I_warehouse(2)(3)(10)#65 <= 124
 0 <= I_customer(3)(1)(1)#66 <= 76
 0 <= I_customer(3)(1)(2)#67 <= 76
 0 <= I_customer(3)(1)(3)#68 <= 76
 0 <= I_customer(3)(1)(4)#69 <= 76
 0 <= I_customer(3)(1)(5)#70 <= 76
 0 <= I_customer(3)(1)(6)#71 <= 76
 0 <= I_customer(3)(1)(7)#72 <= 76
 0 <= I_customer(3)(1)(8)#73 <= 76
 0 <= I_customer(3)(1)(9)#74 <= 76
 0 <= I_customer(3)(1)(10)#75 <= 76
 0 <= I_customer(3)(2)(1)#76 <= 76
 0 <= I_customer(3)(2)(2)#77 <= 76
 0 <= I_customer(3)(2)(3)#78 <= 76
 0 <= I_customer(3)(2)(4)#79 <= 76
 0 <= I_customer(3)(2)(5)#80 <= 76
 0 <= I_customer(3)(2)(6)#81 <= 76
 0 <= I_customer(3)(2)(7)#82 <= 76
 0 <= I_customer(3)(2)(8)#83 <= 76
 0 <= I_customer(3)(2)(9)#84 <= 76
 0 <= I_customer(3)(2)(10)#85 <= 76
 0 <= I_customer(3)(3)(1)#86 <= 76
 0 <= I_customer(3)(3)(2)#87 <= 76
 0 <= I_customer(3)(3)(3)#88 <= 76
 0 <= I_customer(3)(3)(4)#89 <= 76
 0 <= I_customer(3)(3)(5)#90 <= 76
 0 <= I_customer(3)(3)(6)#91 <= 76
 0 <= I_customer(3)(3)(7)#92 <= 76
 0 <= I_customer(3)(3)(8)#93 <= 76
 0 <= I_customer(3)(3)(9)#94 <= 76
 0 <= I_customer(3)(3)(10)#95 <= 76
 0 <= I_customer(4)(1)(1)#96 <= 28
 0 <= I_customer(4)(1)(2)#97 <= 28
 0 <= I_customer(4)(1)(3)#98 <= 28
 0 <= I_customer(4)(1)(4)#99 <= 28
 0 <= I_customer(4)(1)(5)#100 <= 28
 0 <= I_customer(4)(1)(6)#101 <= 28
 0 <= I_customer(4)(1)(7)#102 <= 28
 0 <= I_customer(4)(1)(8)#103 <= 28
 0 <= I_customer(4)(1)(9)#104 <= 28
 0 <= I_customer(4)(1)(10)#105 <= 28
 0 <= I_customer(4)(2)(1)#106 <= 28
 0 <= I_customer(4)(2)(2)#107 <= 28
 0 <= I_customer(4)(2)(3)#108 <= 28
 0 <= I_customer(4)(2)(4)#109 <= 28
 0 <= I_customer(4)(2)(5)#110 <= 28
 0 <= I_customer(4)(2)(6)#111 <= 28
 0 <= I_customer(4)(2)(7)#112 <= 28
 0 <= I_customer(4)(2)(8)#113 <= 28
 0 <= I_customer(4)(2)(9)#114 <= 28
 0 <= I_customer(4)(2)(10)#115 <= 28
 0 <= I_customer(4)(3)(1)#116 <= 28
 0 <= I_customer(4)(3)(2)#117 <= 28
 0 <= I_customer(4)(3)(3)#118 <= 28
 0 <= I_customer(4)(3)(4)#119 <= 28
 0 <= I_customer(4)(3)(5)#120 <= 28
 0 <= I_customer(4)(3)(6)#121 <= 28
 0 <= I_customer(4)(3)(7)#122 <= 28
 0 <= I_customer(4)(3)(8)#123 <= 28
 0 <= I_customer(4)(3)(9)#124 <= 28
 0 <= I_customer(4)(3)(10)#125 <= 28
 0 <= I_customer(5)(1)(1)#126 <= 24
 0 <= I_customer(5)(1)(2)#127 <= 24
 0 <= I_customer(5)(1)(3)#128 <= 24
 0 <= I_customer(5)(1)(4)#129 <= 24
 0 <= I_customer(5)(1)(5)#130 <= 24
 0 <= I_customer(5)(1)(6)#131 <= 24
 0 <= I_customer(5)(1)(7)#132 <= 24
 0 <= I_customer(5)(1)(8)#133 <= 24
 0 <= I_customer(5)(1)(9)#134 <= 24
 0 <= I_customer(5)(1)(10)#135 <= 24
 0 <= I_customer(5)(2)(1)#136 <= 24
 0 <= I_customer(5)(2)(2)#137 <= 24
 0 <= I_customer(5)(2)(3)#138 <= 24
 0 <= I_customer(5)(2)(4)#139 <= 24
 0 <= I_customer(5)(2)(5)#140 <= 24
 0 <= I_customer(5)(2)(6)#141 <= 24
 0 <= I_customer(5)(2)(7)#142 <= 24
 0 <= I_customer(5)(2)(8)#143 <= 24
 0 <= I_customer(5)(2)(9)#144 <= 24
 0 <= I_customer(5)(2)(10)#145 <= 24
 0 <= I_customer(5)(3)(1)#146 <= 24
 0 <= I_customer(5)(3)(2)#147 <= 24
 0 <= I_customer(5)(3)(3)#148 <= 24
 0 <= I_customer(5)(3)(4)#149 <= 24
 0 <= I_customer(5)(3)(5)#150 <= 24
 0 <= I_customer(5)(3)(6)#151 <= 24
 0 <= I_customer(5)(3)(7)#152 <= 24
 0 <= I_customer(5)(3)(8)#153 <= 24
 0 <= I_customer(5)(3)(9)#154 <= 24
 0 <= I_customer(5)(3)(10)#155 <= 24
 0 <= I_customer(6)(1)(1)#156 <= 64
 0 <= I_customer(6)(1)(2)#157 <= 64
 0 <= I_customer(6)(1)(3)#158 <= 64
 0 <= I_customer(6)(1)(4)#159 <= 64
 0 <= I_customer(6)(1)(5)#160 <= 64
 0 <= I_customer(6)(1)(6)#161 <= 64
 0 <= I_customer(6)(1)(7)#162 <= 64
 0 <= I_customer(6)(1)(8)#163 <= 64
 0 <= I_customer(6)(1)(9)#164 <= 64
 0 <= I_customer(6)(1)(10)#165 <= 64
 0 <= I_customer(6)(2)(1)#166 <= 64
 0 <= I_customer(6)(2)(2)#167 <= 64
 0 <= I_customer(6)(2)(3)#168 <= 64
 0 <= I_customer(6)(2)(4)#169 <= 64
 0 <= I_customer(6)(2)(5)#170 <= 64
 0 <= I_customer(6)(2)(6)#171 <= 64
 0 <= I_customer(6)(2)(7)#172 <= 64
 0 <= I_customer(6)(2)(8)#173 <= 64
 0 <= I_customer(6)(2)(9)#174 <= 64
 0 <= I_customer(6)(2)(10)#175 <= 64
 0 <= I_customer(6)(3)(1)#176 <= 64
 0 <= I_customer(6)(3)(2)#177 <= 64
 0 <= I_customer(6)(3)(3)#178 <= 64
 0 <= I_customer(6)(3)(4)#179 <= 64
 0 <= I_customer(6)(3)(5)#180 <= 64
 0 <= I_customer(6)(3)(6)#181 <= 64
 0 <= I_customer(6)(3)(7)#182 <= 64
 0 <= I_customer(6)(3)(8)#183 <= 64
 0 <= I_customer(6)(3)(9)#184 <= 64
 0 <= I_customer(6)(3)(10)#185 <= 64
 0 <= I_customer(7)(1)(1)#186 <= 28
 0 <= I_customer(7)(1)(2)#187 <= 28
 0 <= I_customer(7)(1)(3)#188 <= 28
 0 <= I_customer(7)(1)(4)#189 <= 28
 0 <= I_customer(7)(1)(5)#190 <= 28
 0 <= I_customer(7)(1)(6)#191 <= 28
 0 <= I_customer(7)(1)(7)#192 <= 28
 0 <= I_customer(7)(1)(8)#193 <= 28
 0 <= I_customer(7)(1)(9)#194 <= 28
 0 <= I_customer(7)(1)(10)#195 <= 28
 0 <= I_customer(7)(2)(1)#196 <= 28
 0 <= I_customer(7)(2)(2)#197 <= 28
 0 <= I_customer(7)(2)(3)#198 <= 28
 0 <= I_customer(7)(2)(4)#199 <= 28
 0 <= I_customer(7)(2)(5)#200 <= 28
 0 <= I_customer(7)(2)(6)#201 <= 28
 0 <= I_customer(7)(2)(7)#202 <= 28
 0 <= I_customer(7)(2)(8)#203 <= 28
 0 <= I_customer(7)(2)(9)#204 <= 28
 0 <= I_customer(7)(2)(10)#205 <= 28
 0 <= I_customer(7)(3)(1)#206 <= 28
 0 <= I_customer(7)(3)(2)#207 <= 28
 0 <= I_customer(7)(3)(3)#208 <= 28
 0 <= I_customer(7)(3)(4)#209 <= 28
 0 <= I_customer(7)(3)(5)#210 <= 28
 0 <= I_customer(7)(3)(6)#211 <= 28
 0 <= I_customer(7)(3)(7)#212 <= 28
 0 <= I_customer(7)(3)(8)#213 <= 28
 0 <= I_customer(7)(3)(9)#214 <= 28
 0 <= I_customer(7)(3)(10)#215 <= 28
 0 <= I_customer(8)(1)(1)#216 <= 72
 0 <= I_customer(8)(1)(2)#217 <= 72
 0 <= I_customer(8)(1)(3)#218 <= 72
 0 <= I_customer(8)(1)(4)#219 <= 72
 0 <= I_customer(8)(1)(5)#220 <= 72
 0 <= I_customer(8)(1)(6)#221 <= 72
 0 <= I_customer(8)(1)(7)#222 <= 72
 0 <= I_customer(8)(1)(8)#223 <= 72
 0 <= I_customer(8)(1)(9)#224 <= 72
 0 <= I_customer(8)(1)(10)#225 <= 72
 0 <= I_customer(8)(2)(1)#226 <= 72
 0 <= I_customer(8)(2)(2)#227 <= 72
 0 <= I_customer(8)(2)(3)#228 <= 72
 0 <= I_customer(8)(2)(4)#229 <= 72
 0 <= I_customer(8)(2)(5)#230 <= 72
 0 <= I_customer(8)(2)(6)#231 <= 72
 0 <= I_customer(8)(2)(7)#232 <= 72
 0 <= I_customer(8)(2)(8)#233 <= 72
 0 <= I_customer(8)(2)(9)#234 <= 72
 0 <= I_customer(8)(2)(10)#235 <= 72
 0 <= I_customer(8)(3)(1)#236 <= 72
 0 <= I_customer(8)(3)(2)#237 <= 72
 0 <= I_customer(8)(3)(3)#238 <= 72
 0 <= I_customer(8)(3)(4)#239 <= 72
 0 <= I_customer(8)(3)(5)#240 <= 72
 0 <= I_customer(8)(3)(6)#241 <= 72
 0 <= I_customer(8)(3)(7)#242 <= 72
 0 <= I_customer(8)(3)(8)#243 <= 72
 0 <= I_customer(8)(3)(9)#244 <= 72
 0 <= I_customer(8)(3)(10)#245 <= 72
 0 <= I_customer(9)(1)(1)#246 <= 64
 0 <= I_customer(9)(1)(2)#247 <= 64
 0 <= I_customer(9)(1)(3)#248 <= 64
 0 <= I_customer(9)(1)(4)#249 <= 64
 0 <= I_customer(9)(1)(5)#250 <= 64
 0 <= I_customer(9)(1)(6)#251 <= 64
 0 <= I_customer(9)(1)(7)#252 <= 64
 0 <= I_customer(9)(1)(8)#253 <= 64
 0 <= I_customer(9)(1)(9)#254 <= 64
 0 <= I_customer(9)(1)(10)#255 <= 64
 0 <= I_customer(9)(2)(1)#256 <= 64
 0 <= I_customer(9)(2)(2)#257 <= 64
 0 <= I_customer(9)(2)(3)#258 <= 64
 0 <= I_customer(9)(2)(4)#259 <= 64
 0 <= I_customer(9)(2)(5)#260 <= 64
 0 <= I_customer(9)(2)(6)#261 <= 64
 0 <= I_customer(9)(2)(7)#262 <= 64
 0 <= I_customer(9)(2)(8)#263 <= 64
 0 <= I_customer(9)(2)(9)#264 <= 64
 0 <= I_customer(9)(2)(10)#265 <= 64
 0 <= I_customer(9)(3)(1)#266 <= 64
 0 <= I_customer(9)(3)(2)#267 <= 64
 0 <= I_customer(9)(3)(3)#268 <= 64
 0 <= I_customer(9)(3)(4)#269 <= 64
 0 <= I_customer(9)(3)(5)#270 <= 64
 0 <= I_customer(9)(3)(6)#271 <= 64
 0 <= I_customer(9)(3)(7)#272 <= 64
 0 <= I_customer(9)(3)(8)#273 <= 64
 0 <= I_customer(9)(3)(9)#274 <= 64
 0 <= I_customer(9)(3)(10)#275 <= 64
 0 <= I_customer(10)(1)(1)#276 <= 64
 0 <= I_customer(10)(1)(2)#277 <= 64
 0 <= I_customer(10)(1)(3)#278 <= 64
 0 <= I_customer(10)(1)(4)#279 <= 64
 0 <= I_customer(10)(1)(5)#280 <= 64
 0 <= I_customer(10)(1)(6)#281 <= 64
 0 <= I_customer(10)(1)(7)#282 <= 64
 0 <= I_customer(10)(1)(8)#283 <= 64
 0 <= I_customer(10)(1)(9)#284 <= 64
 0 <= I_customer(10)(1)(10)#285 <= 64
 0 <= I_customer(10)(2)(1)#286 <= 64
 0 <= I_customer(10)(2)(2)#287 <= 64
 0 <= I_customer(10)(2)(3)#288 <= 64
 0 <= I_customer(10)(2)(4)#289 <= 64
 0 <= I_customer(10)(2)(5)#290 <= 64
 0 <= I_customer(10)(2)(6)#291 <= 64
 0 <= I_customer(10)(2)(7)#292 <= 64
 0 <= I_customer(10)(2)(8)#293 <= 64
 0 <= I_customer(10)(2)(9)#294 <= 64
 0 <= I_customer(10)(2)(10)#295 <= 64
 0 <= I_customer(10)(3)(1)#296 <= 64
 0 <= I_customer(10)(3)(2)#297 <= 64
 0 <= I_customer(10)(3)(3)#298 <= 64
 0 <= I_customer(10)(3)(4)#299 <= 64
 0 <= I_customer(10)(3)(5)#300 <= 64
 0 <= I_customer(10)(3)(6)#301 <= 64
 0 <= I_customer(10)(3)(7)#302 <= 64
 0 <= I_customer(10)(3)(8)#303 <= 64
 0 <= I_customer(10)(3)(9)#304 <= 64
 0 <= I_customer(10)(3)(10)#305 <= 64
 0 <= I_customer(11)(1)(1)#306 <= 64
 0 <= I_customer(11)(1)(2)#307 <= 64
 0 <= I_customer(11)(1)(3)#308 <= 64
 0 <= I_customer(11)(1)(4)#309 <= 64
 0 <= I_customer(11)(1)(5)#310 <= 64
 0 <= I_customer(11)(1)(6)#311 <= 64
 0 <= I_customer(11)(1)(7)#312 <= 64
 0 <= I_customer(11)(1)(8)#313 <= 64
 0 <= I_customer(11)(1)(9)#314 <= 64
 0 <= I_customer(11)(1)(10)#315 <= 64
 0 <= I_customer(11)(2)(1)#316 <= 64
 0 <= I_customer(11)(2)(2)#317 <= 64
 0 <= I_customer(11)(2)(3)#318 <= 64
 0 <= I_customer(11)(2)(4)#319 <= 64
 0 <= I_customer(11)(2)(5)#320 <= 64
 0 <= I_customer(11)(2)(6)#321 <= 64
 0 <= I_customer(11)(2)(7)#322 <= 64
 0 <= I_customer(11)(2)(8)#323 <= 64
 0 <= I_customer(11)(2)(9)#324 <= 64
 0 <= I_customer(11)(2)(10)#325 <= 64
 0 <= I_customer(11)(3)(1)#326 <= 64
 0 <= I_customer(11)(3)(2)#327 <= 64
 0 <= I_customer(11)(3)(3)#328 <= 64
 0 <= I_customer(11)(3)(4)#329 <= 64
 0 <= I_customer(11)(3)(5)#330 <= 64
 0 <= I_customer(11)(3)(6)#331 <= 64
 0 <= I_customer(11)(3)(7)#332 <= 64
 0 <= I_customer(11)(3)(8)#333 <= 64
 0 <= I_customer(11)(3)(9)#334 <= 64
 0 <= I_customer(11)(3)(10)#335 <= 64
 0 <= I_customer(12)(1)(1)#336 <= 56
 0 <= I_customer(12)(1)(2)#337 <= 56
 0 <= I_customer(12)(1)(3)#338 <= 56
 0 <= I_customer(12)(1)(4)#339 <= 56
 0 <= I_customer(12)(1)(5)#340 <= 56
 0 <= I_customer(12)(1)(6)#341 <= 56
 0 <= I_customer(12)(1)(7)#342 <= 56
 0 <= I_customer(12)(1)(8)#343 <= 56
 0 <= I_customer(12)(1)(9)#344 <= 56
 0 <= I_customer(12)(1)(10)#345 <= 56
 0 <= I_customer(12)(2)(1)#346 <= 56
 0 <= I_customer(12)(2)(2)#347 <= 56
 0 <= I_customer(12)(2)(3)#348 <= 56
 0 <= I_customer(12)(2)(4)#349 <= 56
 0 <= I_customer(12)(2)(5)#350 <= 56
 0 <= I_customer(12)(2)(6)#351 <= 56
 0 <= I_customer(12)(2)(7)#352 <= 56
 0 <= I_customer(12)(2)(8)#353 <= 56
 0 <= I_customer(12)(2)(9)#354 <= 56
 0 <= I_customer(12)(2)(10)#355 <= 56
 0 <= I_customer(12)(3)(1)#356 <= 56
 0 <= I_customer(12)(3)(2)#357 <= 56
 0 <= I_customer(12)(3)(3)#358 <= 56
 0 <= I_customer(12)(3)(4)#359 <= 56
 0 <= I_customer(12)(3)(5)#360 <= 56
 0 <= I_customer(12)(3)(6)#361 <= 56
 0 <= I_customer(12)(3)(7)#362 <= 56
 0 <= I_customer(12)(3)(8)#363 <= 56
 0 <= I_customer(12)(3)(9)#364 <= 56
 0 <= I_customer(12)(3)(10)#365 <= 56
 0 <= b_customer(3)(1)(1)#366 <= 21
 0 <= b_customer(3)(1)(2)#367 <= 22
 0 <= b_customer(3)(1)(3)#368 <= 21
 0 <= b_customer(3)(1)(4)#369 <= 21
 0 <= b_customer(3)(1)(5)#370 <= 18
 0 <= b_customer(3)(1)(6)#371 <= 22
 0 <= b_customer(3)(1)(7)#372 <= 23
 0 <= b_customer(3)(1)(8)#373 <= 19
 0 <= b_customer(3)(1)(9)#374 <= 23
 0 <= b_customer(3)(1)(10)#375 <= 21
 0 <= b_customer(3)(2)(1)#376 <= 17
 0 <= b_customer(3)(2)(2)#377 <= 20
 0 <= b_customer(3)(2)(3)#378 <= 20
 0 <= b_customer(3)(2)(4)#379 <= 20
 0 <= b_customer(3)(2)(5)#380 <= 22
 0 <= b_customer(3)(2)(6)#381 <= 19
 0 <= b_customer(3)(2)(7)#382 <= 21
 0 <= b_customer(3)(2)(8)#383 <= 19
 0 <= b_customer(3)(2)(9)#384 <= 18
 0 <= b_customer(3)(2)(10)#385 <= 19
 0 <= b_customer(3)(3)(1)#386 <= 22
 0 <= b_customer(3)(3)(2)#387 <= 16
 0 <= b_customer(3)(3)(3)#388 <= 20
 0 <= b_customer(3)(3)(4)#389 <= 21
 0 <= b_customer(3)(3)(5)#390 <= 17
 0 <= b_customer(3)(3)(6)#391 <= 18
 0 <= b_customer(3)(3)(7)#392 <= 19
 0 <= b_customer(3)(3)(8)#393 <= 15
 0 <= b_customer(3)(3)(9)#394 <= 16
 0 <= b_customer(3)(3)(10)#395 <= 21
 0 <= b_customer(4)(1)(1)#396 <= 8
 0 <= b_customer(4)(1)(2)#397 <= 6
 0 <= b_customer(4)(1)(3)#398 <= 8
 0 <= b_customer(4)(1)(4)#399 <= 8
 0 <= b_customer(4)(1)(5)#400 <= 8
 0 <= b_customer(4)(1)(6)#401 <= 6
 0 <= b_customer(4)(1)(7)#402 <= 6
 0 <= b_customer(4)(1)(8)#403 <= 6
 0 <= b_customer(4)(1)(9)#404 <= 8
 0 <= b_customer(4)(1)(10)#405 <= 6
 0 <= b_customer(4)(2)(1)#406 <= 6
 0 <= b_customer(4)(2)(2)#407 <= 8
 0 <= b_customer(4)(2)(3)#408 <= 8
 0 <= b_customer(4)(2)(4)#409 <= 7
 0 <= b_customer(4)(2)(5)#410 <= 8
 0 <= b_customer(4)(2)(6)#411 <= 6
 0 <= b_customer(4)(2)(7)#412 <= 8
 0 <= b_customer(4)(2)(8)#413 <= 6
 0 <= b_customer(4)(2)(9)#414 <= 7
 0 <= b_customer(4)(2)(10)#415 <= 6
 0 <= b_customer(4)(3)(1)#416 <= 8
 0 <= b_customer(4)(3)(2)#417 <= 7
 0 <= b_customer(4)(3)(3)#418 <= 7
 0 <= b_customer(4)(3)(4)#419 <= 6
 0 <= b_customer(4)(3)(5)#420 <= 7
 0 <= b_customer(4)(3)(6)#421 <= 7
 0 <= b_customer(4)(3)(7)#422 <= 7
 0 <= b_customer(4)(3)(8)#423 <= 7
 0 <= b_customer(4)(3)(9)#424 <= 8
 0 <= b_customer(4)(3)(10)#425 <= 7
 0 <= b_customer(5)(1)(1)#426 <= 5
 0 <= b_customer(5)(1)(2)#427 <= 5
 0 <= b_customer(5)(1)(3)#428 <= 6
 0 <= b_customer(5)(1)(4)#429 <= 7
 0 <= b_customer(5)(1)(5)#430 <= 7
 0 <= b_customer(5)(1)(6)#431 <= 6
 0 <= b_customer(5)(1)(7)#432 <= 6
 0 <= b_customer(5)(1)(8)#433 <= 5
 0 <= b_customer(5)(1)(9)#434 <= 5
 0 <= b_customer(5)(1)(10)#435 <= 6
 0 <= b_customer(5)(2)(1)#436 <= 6
 0 <= b_customer(5)(2)(2)#437 <= 7
 0 <= b_customer(5)(2)(3)#438 <= 7
 0 <= b_customer(5)(2)(4)#439 <= 6
 0 <= b_customer(5)(2)(5)#440 <= 5
 0 <= b_customer(5)(2)(6)#441 <= 7
 0 <= b_customer(5)(2)(7)#442 <= 5
 0 <= b_customer(5)(2)(8)#443 <= 5
 0 <= b_customer(5)(2)(9)#444 <= 7
 0 <= b_customer(5)(2)(10)#445 <= 5
 0 <= b_customer(5)(3)(1)#446 <= 6
 0 <= b_customer(5)(3)(2)#447 <= 7
 0 <= b_customer(5)(3)(3)#448 <= 5
 0 <= b_customer(5)(3)(4)#449 <= 6
 0 <= b_customer(5)(3)(5)#450 <= 6
 0 <= b_customer(5)(3)(6)#451 <= 5
 0 <= b_customer(5)(3)(7)#452 <= 5
 0 <= b_customer(5)(3)(8)#453 <= 7
 0 <= b_customer(5)(3)(9)#454 <= 5
 0 <= b_customer(5)(3)(10)#455 <= 6
 0 <= b_customer(6)(1)(1)#456 <= 13
 0 <= b_customer(6)(1)(2)#457 <= 17
 0 <= b_customer(6)(1)(3)#458 <= 16
 0 <= b_customer(6)(1)(4)#459 <= 17
 0 <= b_customer(6)(1)(5)#460 <= 17
 0 <= b_customer(6)(1)(6)#461 <= 17
 0 <= b_customer(6)(1)(7)#462 <= 17
 0 <= b_customer(6)(1)(8)#463 <= 17
 0 <= b_customer(6)(1)(9)#464 <= 16
 0 <= b_customer(6)(1)(10)#465 <= 15
 0 <= b_customer(6)(2)(1)#466 <= 17
 0 <= b_customer(6)(2)(2)#467 <= 16
 0 <= b_customer(6)(2)(3)#468 <= 15
 0 <= b_customer(6)(2)(4)#469 <= 14
 0 <= b_customer(6)(2)(5)#470 <= 15
 0 <= b_customer(6)(2)(6)#471 <= 15
 0 <= b_customer(6)(2)(7)#472 <= 15
 0 <= b_customer(6)(2)(8)#473 <= 17
 0 <= b_customer(6)(2)(9)#474 <= 16
 0 <= b_customer(6)(2)(10)#475 <= 18
 0 <= b_customer(6)(3)(1)#476 <= 17
 0 <= b_customer(6)(3)(2)#477 <= 17
 0 <= b_customer(6)(3)(3)#478 <= 16
 0 <= b_customer(6)(3)(4)#479 <= 14
 0 <= b_customer(6)(3)(5)#480 <= 18
 0 <= b_customer(6)(3)(6)#481 <= 13
 0 <= b_customer(6)(3)(7)#482 <= 17
 0 <= b_customer(6)(3)(8)#483 <= 16
 0 <= b_customer(6)(3)(9)#484 <= 14
 0 <= b_customer(6)(3)(10)#485 <= 17
 0 <= b_customer(7)(1)(1)#486 <= 8
 0 <= b_customer(7)(1)(2)#487 <= 6
 0 <= b_customer(7)(1)(3)#488 <= 8
 0 <= b_customer(7)(1)(4)#489 <= 8
 0 <= b_customer(7)(1)(5)#490 <= 8
 0 <= b_customer(7)(1)(6)#491 <= 7
 0 <= b_customer(7)(1)(7)#492 <= 6
 0 <= b_customer(7)(1)(8)#493 <= 8
 0 <= b_customer(7)(1)(9)#494 <= 7
 0 <= b_customer(7)(1)(10)#495 <= 8
 0 <= b_customer(7)(2)(1)#496 <= 8
 0 <= b_customer(7)(2)(2)#497 <= 7
 0 <= b_customer(7)(2)(3)#498 <= 7
 0 <= b_customer(7)(2)(4)#499 <= 6
 0 <= b_customer(7)(2)(5)#500 <= 6
 0 <= b_customer(7)(2)(6)#501 <= 6
 0 <= b_customer(7)(2)(7)#502 <= 7
 0 <= b_customer(7)(2)(8)#503 <= 7
 0 <= b_customer(7)(2)(9)#504 <= 7
 0 <= b_customer(7)(2)(10)#505 <= 7
 0 <= b_customer(7)(3)(1)#506 <= 6
 0 <= b_customer(7)(3)(2)#507 <= 6
 0 <= b_customer(7)(3)(3)#508 <= 7
 0 <= b_customer(7)(3)(4)#509 <= 6
 0 <= b_customer(7)(3)(5)#510 <= 7
 0 <= b_customer(7)(3)(6)#511 <= 7
 0 <= b_customer(7)(3)(7)#512 <= 7
 0 <= b_customer(7)(3)(8)#513 <= 7
 0 <= b_customer(7)(3)(9)#514 <= 7
 0 <= b_customer(7)(3)(10)#515 <= 7
 0 <= b_customer(8)(1)(1)#516 <= 15
 0 <= b_customer(8)(1)(2)#517 <= 18
 0 <= b_customer(8)(1)(3)#518 <= 21
 0 <= b_customer(8)(1)(4)#519 <= 16
 0 <= b_customer(8)(1)(5)#520 <= 21
 0 <= b_customer(8)(1)(6)#521 <= 17
 0 <= b_customer(8)(1)(7)#522 <= 20
 0 <= b_customer(8)(1)(8)#523 <= 15
 0 <= b_customer(8)(1)(9)#524 <= 19
 0 <= b_customer(8)(1)(10)#525 <= 18
 0 <= b_customer(8)(2)(1)#526 <= 20
 0 <= b_customer(8)(2)(2)#527 <= 17
 0 <= b_customer(8)(2)(3)#528 <= 20
 0 <= b_customer(8)(2)(4)#529 <= 17
 0 <= b_customer(8)(2)(5)#530 <= 15
 0 <= b_customer(8)(2)(6)#531 <= 21
 0 <= b_customer(8)(2)(7)#532 <= 20
 0 <= b_customer(8)(2)(8)#533 <= 19
 0 <= b_customer(8)(2)(9)#534 <= 17
 0 <= b_customer(8)(2)(10)#535 <= 16
 0 <= b_customer(8)(3)(1)#536 <= 19
 0 <= b_customer(8)(3)(2)#537 <= 19
 0 <= b_customer(8)(3)(3)#538 <= 21
 0 <= b_customer(8)(3)(4)#539 <= 16
 0 <= b_customer(8)(3)(5)#540 <= 15
 0 <= b_customer(8)(3)(6)#541 <= 17
 0 <= b_customer(8)(3)(7)#542 <= 16
 0 <= b_customer(8)(3)(8)#543 <= 21
 0 <= b_customer(8)(3)(9)#544 <= 15
 0 <= b_customer(8)(3)(10)#545 <= 17
 0 <= b_customer(9)(1)(1)#546 <= 19
 0 <= b_customer(9)(1)(2)#547 <= 13
 0 <= b_customer(9)(1)(3)#548 <= 14
 0 <= b_customer(9)(1)(4)#549 <= 16
 0 <= b_customer(9)(1)(5)#550 <= 19
 0 <= b_customer(9)(1)(6)#551 <= 19
 0 <= b_customer(9)(1)(7)#552 <= 13
 0 <= b_customer(9)(1)(8)#553 <= 15
 0 <= b_customer(9)(1)(9)#554 <= 17
 0 <= b_customer(9)(1)(10)#555 <= 15
 0 <= b_customer(9)(2)(1)#556 <= 15
 0 <= b_customer(9)(2)(2)#557 <= 14
 0 <= b_customer(9)(2)(3)#558 <= 16
 0 <= b_customer(9)(2)(4)#559 <= 17
 0 <= b_customer(9)(2)(5)#560 <= 19
 0 <= b_customer(9)(2)(6)#561 <= 14
 0 <= b_customer(9)(2)(7)#562 <= 15
 0 <= b_customer(9)(2)(8)#563 <= 15
 0 <= b_customer(9)(2)(9)#564 <= 18
 0 <= b_customer(9)(2)(10)#565 <= 18
 0 <= b_customer(9)(3)(1)#566 <= 17
 0 <= b_customer(9)(3)(2)#567 <= 13
 0 <= b_customer(9)(3)(3)#568 <= 19
 0 <= b_customer(9)(3)(4)#569 <= 16
 0 <= b_customer(9)(3)(5)#570 <= 17
 0 <= b_customer(9)(3)(6)#571 <= 19
 0 <= b_customer(9)(3)(7)#572 <= 19
 0 <= b_customer(9)(3)(8)#573 <= 18
 0 <= b_customer(9)(3)(9)#574 <= 17
 0 <= b_customer(9)(3)(10)#575 <= 18
 0 <= b_customer(10)(1)(1)#576 <= 14
 0 <= b_customer(10)(1)(2)#577 <= 14
 0 <= b_customer(10)(1)(3)#578 <= 18
 0 <= b_customer(10)(1)(4)#579 <= 18
 0 <= b_customer(10)(1)(5)#580 <= 13
 0 <= b_customer(10)(1)(6)#581 <= 16
 0 <= b_customer(10)(1)(7)#582 <= 16
 0 <= b_customer(10)(1)(8)#583 <= 13
 0 <= b_customer(10)(1)(9)#584 <= 14
 0 <= b_customer(10)(1)(10)#585 <= 13
 0 <= b_customer(10)(2)(1)#586 <= 14
 0 <= b_customer(10)(2)(2)#587 <= 18
 0 <= b_customer(10)(2)(3)#588 <= 13
 0 <= b_customer(10)(2)(4)#589 <= 19
 0 <= b_customer(10)(2)(5)#590 <= 16
 0 <= b_customer(10)(2)(6)#591 <= 18
 0 <= b_customer(10)(2)(7)#592 <= 17
 0 <= b_customer(10)(2)(8)#593 <= 13
 0 <= b_customer(10)(2)(9)#594 <= 14
 0 <= b_customer(10)(2)(10)#595 <= 16
 0 <= b_customer(10)(3)(1)#596 <= 16
 0 <= b_customer(10)(3)(2)#597 <= 17
 0 <= b_customer(10)(3)(3)#598 <= 19
 0 <= b_customer(10)(3)(4)#599 <= 18
 0 <= b_customer(10)(3)(5)#600 <= 14
 0 <= b_customer(10)(3)(6)#601 <= 18
 0 <= b_customer(10)(3)(7)#602 <= 18
 0 <= b_customer(10)(3)(8)#603 <= 17
 0 <= b_customer(10)(3)(9)#604 <= 19
 0 <= b_customer(10)(3)(10)#605 <= 16
 0 <= b_customer(11)(1)(1)#606 <= 13
 0 <= b_customer(11)(1)(2)#607 <= 19
 0 <= b_customer(11)(1)(3)#608 <= 15
 0 <= b_customer(11)(1)(4)#609 <= 15
 0 <= b_customer(11)(1)(5)#610 <= 19
 0 <= b_customer(11)(1)(6)#611 <= 14
 0 <= b_customer(11)(1)(7)#612 <= 17
 0 <= b_customer(11)(1)(8)#613 <= 19
 0 <= b_customer(11)(1)(9)#614 <= 18
 0 <= b_customer(11)(1)(10)#615 <= 16
 0 <= b_customer(11)(2)(1)#616 <= 13
 0 <= b_customer(11)(2)(2)#617 <= 16
 0 <= b_customer(11)(2)(3)#618 <= 18
 0 <= b_customer(11)(2)(4)#619 <= 19
 0 <= b_customer(11)(2)(5)#620 <= 18
 0 <= b_customer(11)(2)(6)#621 <= 13
 0 <= b_customer(11)(2)(7)#622 <= 17
 0 <= b_customer(11)(2)(8)#623 <= 14
 0 <= b_customer(11)(2)(9)#624 <= 13
 0 <= b_customer(11)(2)(10)#625 <= 15
 0 <= b_customer(11)(3)(1)#626 <= 14
 0 <= b_customer(11)(3)(2)#627 <= 18
 0 <= b_customer(11)(3)(3)#628 <= 17
 0 <= b_customer(11)(3)(4)#629 <= 14
 0 <= b_customer(11)(3)(5)#630 <= 14
 0 <= b_customer(11)(3)(6)#631 <= 14
 0 <= b_customer(11)(3)(7)#632 <= 17
 0 <= b_customer(11)(3)(8)#633 <= 16
 0 <= b_customer(11)(3)(9)#634 <= 16
 0 <= b_customer(11)(3)(10)#635 <= 15
 0 <= b_customer(12)(1)(1)#636 <= 13
 0 <= b_customer(12)(1)(2)#637 <= 11
 0 <= b_customer(12)(1)(3)#638 <= 13
 0 <= b_customer(12)(1)(4)#639 <= 15
 0 <= b_customer(12)(1)(5)#640 <= 14
 0 <= b_customer(12)(1)(6)#641 <= 12
 0 <= b_customer(12)(1)(7)#642 <= 12
 0 <= b_customer(12)(1)(8)#643 <= 12
 0 <= b_customer(12)(1)(9)#644 <= 11
 0 <= b_customer(12)(1)(10)#645 <= 12
 0 <= b_customer(12)(2)(1)#646 <= 14
 0 <= b_customer(12)(2)(2)#647 <= 14
 0 <= b_customer(12)(2)(3)#648 <= 13
 0 <= b_customer(12)(2)(4)#649 <= 15
 0 <= b_customer(12)(2)(5)#650 <= 12
 0 <= b_customer(12)(2)(6)#651 <= 12
 0 <= b_customer(12)(2)(7)#652 <= 14
 0 <= b_customer(12)(2)(8)#653 <= 17
 0 <= b_customer(12)(2)(9)#654 <= 16
 0 <= b_customer(12)(2)(10)#655 <= 11
 0 <= b_customer(12)(3)(1)#656 <= 16
 0 <= b_customer(12)(3)(2)#657 <= 16
 0 <= b_customer(12)(3)(3)#658 <= 13
 0 <= b_customer(12)(3)(4)#659 <= 12
 0 <= b_customer(12)(3)(5)#660 <= 12
 0 <= b_customer(12)(3)(6)#661 <= 16
 0 <= b_customer(12)(3)(7)#662 <= 16
 0 <= b_customer(12)(3)(8)#663 <= 14
 0 <= b_customer(12)(3)(9)#664 <= 13
 0 <= b_customer(12)(3)(10)#665 <= 16
 0 <= q(1)(1)(1)#666 <= 124
 0 <= q(1)(1)(2)#667 <= 124
 0 <= q(1)(1)(3)#668 <= 124
 0 <= q(2)(1)(1)#669 <= 124
 0 <= q(2)(1)(2)#670 <= 124
 0 <= q(2)(1)(3)#671 <= 124
 0 <= w(3)(1)(1)(1)#672 <= 21
 0 <= w(3)(1)(1)(2)#673 <= 22
 0 <= w(3)(1)(1)(3)#674 <= 21
 0 <= w(3)(1)(1)(4)#675 <= 21
 0 <= w(3)(1)(1)(5)#676 <= 18
 0 <= w(3)(1)(1)(6)#677 <= 22
 0 <= w(3)(1)(1)(7)#678 <= 23
 0 <= w(3)(1)(1)(8)#679 <= 19
 0 <= w(3)(1)(1)(9)#680 <= 23
 0 <= w(3)(1)(1)(10)#681 <= 21
 0 <= w(3)(1)(2)(1)#682 <= 17
 0 <= w(3)(1)(2)(2)#683 <= 20
 0 <= w(3)(1)(2)(3)#684 <= 20
 0 <= w(3)(1)(2)(4)#685 <= 20
 0 <= w(3)(1)(2)(5)#686 <= 22
 0 <= w(3)(1)(2)(6)#687 <= 19
 0 <= w(3)(1)(2)(7)#688 <= 21
 0 <= w(3)(1)(2)(8)#689 <= 19
 0 <= w(3)(1)(2)(9)#690 <= 18
 0 <= w(3)(1)(2)(10)#691 <= 19
 0 <= w(3)(1)(3)(1)#692 <= 22
 0 <= w(3)(1)(3)(2)#693 <= 16
 0 <= w(3)(1)(3)(3)#694 <= 20
 0 <= w(3)(1)(3)(4)#695 <= 21
 0 <= w(3)(1)(3)(5)#696 <= 17
 0 <= w(3)(1)(3)(6)#697 <= 18
 0 <= w(3)(1)(3)(7)#698 <= 19
 0 <= w(3)(1)(3)(8)#699 <= 15
 0 <= w(3)(1)(3)(9)#700 <= 16
 0 <= w(3)(1)(3)(10)#701 <= 21
 0 <= w(3)(2)(1)(1)#702 <= 21
 0 <= w(3)(2)(1)(2)#703 <= 22
 0 <= w(3)(2)(1)(3)#704 <= 21
 0 <= w(3)(2)(1)(4)#705 <= 21
 0 <= w(3)(2)(1)(5)#706 <= 18
 0 <= w(3)(2)(1)(6)#707 <= 22
 0 <= w(3)(2)(1)(7)#708 <= 23
 0 <= w(3)(2)(1)(8)#709 <= 19
 0 <= w(3)(2)(1)(9)#710 <= 23
 0 <= w(3)(2)(1)(10)#711 <= 21
 0 <= w(3)(2)(2)(1)#712 <= 17
 0 <= w(3)(2)(2)(2)#713 <= 20
 0 <= w(3)(2)(2)(3)#714 <= 20
 0 <= w(3)(2)(2)(4)#715 <= 20
 0 <= w(3)(2)(2)(5)#716 <= 22
 0 <= w(3)(2)(2)(6)#717 <= 19
 0 <= w(3)(2)(2)(7)#718 <= 21
 0 <= w(3)(2)(2)(8)#719 <= 19
 0 <= w(3)(2)(2)(9)#720 <= 18
 0 <= w(3)(2)(2)(10)#721 <= 19
 0 <= w(3)(2)(3)(1)#722 <= 22
 0 <= w(3)(2)(3)(2)#723 <= 16
 0 <= w(3)(2)(3)(3)#724 <= 20
 0 <= w(3)(2)(3)(4)#725 <= 21
 0 <= w(3)(2)(3)(5)#726 <= 17
 0 <= w(3)(2)(3)(6)#727 <= 18
 0 <= w(3)(2)(3)(7)#728 <= 19
 0 <= w(3)(2)(3)(8)#729 <= 15
 0 <= w(3)(2)(3)(9)#730 <= 16
 0 <= w(3)(2)(3)(10)#731 <= 21
 0 <= w(4)(1)(1)(1)#732 <= 8
 0 <= w(4)(1)(1)(2)#733 <= 6
 0 <= w(4)(1)(1)(3)#734 <= 8
 0 <= w(4)(1)(1)(4)#735 <= 8
 0 <= w(4)(1)(1)(5)#736 <= 8
 0 <= w(4)(1)(1)(6)#737 <= 6
 0 <= w(4)(1)(1)(7)#738 <= 6
 0 <= w(4)(1)(1)(8)#739 <= 6
 0 <= w(4)(1)(1)(9)#740 <= 8
 0 <= w(4)(1)(1)(10)#741 <= 6
 0 <= w(4)(1)(2)(1)#742 <= 6
 0 <= w(4)(1)(2)(2)#743 <= 8
 0 <= w(4)(1)(2)(3)#744 <= 8
 0 <= w(4)(1)(2)(4)#745 <= 7
 0 <= w(4)(1)(2)(5)#746 <= 8
 0 <= w(4)(1)(2)(6)#747 <= 6
 0 <= w(4)(1)(2)(7)#748 <= 8
 0 <= w(4)(1)(2)(8)#749 <= 6
 0 <= w(4)(1)(2)(9)#750 <= 7
 0 <= w(4)(1)(2)(10)#751 <= 6
 0 <= w(4)(1)(3)(1)#752 <= 8
 0 <= w(4)(1)(3)(2)#753 <= 7
 0 <= w(4)(1)(3)(3)#754 <= 7
 0 <= w(4)(1)(3)(4)#755 <= 6
 0 <= w(4)(1)(3)(5)#756 <= 7
 0 <= w(4)(1)(3)(6)#757 <= 7
 0 <= w(4)(1)(3)(7)#758 <= 7
 0 <= w(4)(1)(3)(8)#759 <= 7
 0 <= w(4)(1)(3)(9)#760 <= 8
 0 <= w(4)(1)(3)(10)#761 <= 7
 0 <= w(4)(2)(1)(1)#762 <= 8
 0 <= w(4)(2)(1)(2)#763 <= 6
 0 <= w(4)(2)(1)(3)#764 <= 8
 0 <= w(4)(2)(1)(4)#765 <= 8
 0 <= w(4)(2)(1)(5)#766 <= 8
 0 <= w(4)(2)(1)(6)#767 <= 6
 0 <= w(4)(2)(1)(7)#768 <= 6
 0 <= w(4)(2)(1)(8)#769 <= 6
 0 <= w(4)(2)(1)(9)#770 <= 8
 0 <= w(4)(2)(1)(10)#771 <= 6
 0 <= w(4)(2)(2)(1)#772 <= 6
 0 <= w(4)(2)(2)(2)#773 <= 8
 0 <= w(4)(2)(2)(3)#774 <= 8
 0 <= w(4)(2)(2)(4)#775 <= 7
 0 <= w(4)(2)(2)(5)#776 <= 8
 0 <= w(4)(2)(2)(6)#777 <= 6
 0 <= w(4)(2)(2)(7)#778 <= 8
 0 <= w(4)(2)(2)(8)#779 <= 6
 0 <= w(4)(2)(2)(9)#780 <= 7
 0 <= w(4)(2)(2)(10)#781 <= 6
 0 <= w(4)(2)(3)(1)#782 <= 8
 0 <= w(4)(2)(3)(2)#783 <= 7
 0 <= w(4)(2)(3)(3)#784 <= 7
 0 <= w(4)(2)(3)(4)#785 <= 6
 0 <= w(4)(2)(3)(5)#786 <= 7
 0 <= w(4)(2)(3)(6)#787 <= 7
 0 <= w(4)(2)(3)(7)#788 <= 7
 0 <= w(4)(2)(3)(8)#789 <= 7
 0 <= w(4)(2)(3)(9)#790 <= 8
 0 <= w(4)(2)(3)(10)#791 <= 7
 0 <= w(5)(1)(1)(1)#792 <= 5
 0 <= w(5)(1)(1)(2)#793 <= 5
 0 <= w(5)(1)(1)(3)#794 <= 6
 0 <= w(5)(1)(1)(4)#795 <= 7
 0 <= w(5)(1)(1)(5)#796 <= 7
 0 <= w(5)(1)(1)(6)#797 <= 6
 0 <= w(5)(1)(1)(7)#798 <= 6
 0 <= w(5)(1)(1)(8)#799 <= 5
 0 <= w(5)(1)(1)(9)#800 <= 5
 0 <= w(5)(1)(1)(10)#801 <= 6
 0 <= w(5)(1)(2)(1)#802 <= 6
 0 <= w(5)(1)(2)(2)#803 <= 7
 0 <= w(5)(1)(2)(3)#804 <= 7
 0 <= w(5)(1)(2)(4)#805 <= 6
 0 <= w(5)(1)(2)(5)#806 <= 5
 0 <= w(5)(1)(2)(6)#807 <= 7
 0 <= w(5)(1)(2)(7)#808 <= 5
 0 <= w(5)(1)(2)(8)#809 <= 5
 0 <= w(5)(1)(2)(9)#810 <= 7
 0 <= w(5)(1)(2)(10)#811 <= 5
 0 <= w(5)(1)(3)(1)#812 <= 6
 0 <= w(5)(1)(3)(2)#813 <= 7
 0 <= w(5)(1)(3)(3)#814 <= 5
 0 <= w(5)(1)(3)(4)#815 <= 6
 0 <= w(5)(1)(3)(5)#816 <= 6
 0 <= w(5)(1)(3)(6)#817 <= 5
 0 <= w(5)(1)(3)(7)#818 <= 5
 0 <= w(5)(1)(3)(8)#819 <= 7
 0 <= w(5)(1)(3)(9)#820 <= 5
 0 <= w(5)(1)(3)(10)#821 <= 6
 0 <= w(5)(2)(1)(1)#822 <= 5
 0 <= w(5)(2)(1)(2)#823 <= 5
 0 <= w(5)(2)(1)(3)#824 <= 6
 0 <= w(5)(2)(1)(4)#825 <= 7
 0 <= w(5)(2)(1)(5)#826 <= 7
 0 <= w(5)(2)(1)(6)#827 <= 6
 0 <= w(5)(2)(1)(7)#828 <= 6
 0 <= w(5)(2)(1)(8)#829 <= 5
 0 <= w(5)(2)(1)(9)#830 <= 5
 0 <= w(5)(2)(1)(10)#831 <= 6
 0 <= w(5)(2)(2)(1)#832 <= 6
 0 <= w(5)(2)(2)(2)#833 <= 7
 0 <= w(5)(2)(2)(3)#834 <= 7
 0 <= w(5)(2)(2)(4)#835 <= 6
 0 <= w(5)(2)(2)(5)#836 <= 5
 0 <= w(5)(2)(2)(6)#837 <= 7
 0 <= w(5)(2)(2)(7)#838 <= 5
 0 <= w(5)(2)(2)(8)#839 <= 5
 0 <= w(5)(2)(2)(9)#840 <= 7
 0 <= w(5)(2)(2)(10)#841 <= 5
 0 <= w(5)(2)(3)(1)#842 <= 6
 0 <= w(5)(2)(3)(2)#843 <= 7
 0 <= w(5)(2)(3)(3)#844 <= 5
 0 <= w(5)(2)(3)(4)#845 <= 6
 0 <= w(5)(2)(3)(5)#846 <= 6
 0 <= w(5)(2)(3)(6)#847 <= 5
 0 <= w(5)(2)(3)(7)#848 <= 5
 0 <= w(5)(2)(3)(8)#849 <= 7
 0 <= w(5)(2)(3)(9)#850 <= 5
 0 <= w(5)(2)(3)(10)#851 <= 6
 0 <= w(6)(1)(1)(1)#852 <= 13
 0 <= w(6)(1)(1)(2)#853 <= 17
 0 <= w(6)(1)(1)(3)#854 <= 16
 0 <= w(6)(1)(1)(4)#855 <= 17
 0 <= w(6)(1)(1)(5)#856 <= 17
 0 <= w(6)(1)(1)(6)#857 <= 17
 0 <= w(6)(1)(1)(7)#858 <= 17
 0 <= w(6)(1)(1)(8)#859 <= 17
 0 <= w(6)(1)(1)(9)#860 <= 16
 0 <= w(6)(1)(1)(10)#861 <= 15
 0 <= w(6)(1)(2)(1)#862 <= 17
 0 <= w(6)(1)(2)(2)#863 <= 16
 0 <= w(6)(1)(2)(3)#864 <= 15
 0 <= w(6)(1)(2)(4)#865 <= 14
 0 <= w(6)(1)(2)(5)#866 <= 15
 0 <= w(6)(1)(2)(6)#867 <= 15
 0 <= w(6)(1)(2)(7)#868 <= 15
 0 <= w(6)(1)(2)(8)#869 <= 17
 0 <= w(6)(1)(2)(9)#870 <= 16
 0 <= w(6)(1)(2)(10)#871 <= 18
 0 <= w(6)(1)(3)(1)#872 <= 17
 0 <= w(6)(1)(3)(2)#873 <= 17
 0 <= w(6)(1)(3)(3)#874 <= 16
 0 <= w(6)(1)(3)(4)#875 <= 14
 0 <= w(6)(1)(3)(5)#876 <= 18
 0 <= w(6)(1)(3)(6)#877 <= 13
 0 <= w(6)(1)(3)(7)#878 <= 17
 0 <= w(6)(1)(3)(8)#879 <= 16
 0 <= w(6)(1)(3)(9)#880 <= 14
 0 <= w(6)(1)(3)(10)#881 <= 17
 0 <= w(6)(2)(1)(1)#882 <= 13
 0 <= w(6)(2)(1)(2)#883 <= 17
 0 <= w(6)(2)(1)(3)#884 <= 16
 0 <= w(6)(2)(1)(4)#885 <= 17
 0 <= w(6)(2)(1)(5)#886 <= 17
 0 <= w(6)(2)(1)(6)#887 <= 17
 0 <= w(6)(2)(1)(7)#888 <= 17
 0 <= w(6)(2)(1)(8)#889 <= 17
 0 <= w(6)(2)(1)(9)#890 <= 16
 0 <= w(6)(2)(1)(10)#891 <= 15
 0 <= w(6)(2)(2)(1)#892 <= 17
 0 <= w(6)(2)(2)(2)#893 <= 16
 0 <= w(6)(2)(2)(3)#894 <= 15
 0 <= w(6)(2)(2)(4)#895 <= 14
 0 <= w(6)(2)(2)(5)#896 <= 15
 0 <= w(6)(2)(2)(6)#897 <= 15
 0 <= w(6)(2)(2)(7)#898 <= 15
 0 <= w(6)(2)(2)(8)#899 <= 17
 0 <= w(6)(2)(2)(9)#900 <= 16
 0 <= w(6)(2)(2)(10)#901 <= 18
 0 <= w(6)(2)(3)(1)#902 <= 17
 0 <= w(6)(2)(3)(2)#903 <= 17
 0 <= w(6)(2)(3)(3)#904 <= 16
 0 <= w(6)(2)(3)(4)#905 <= 14
 0 <= w(6)(2)(3)(5)#906 <= 18
 0 <= w(6)(2)(3)(6)#907 <= 13
 0 <= w(6)(2)(3)(7)#908 <= 17
 0 <= w(6)(2)(3)(8)#909 <= 16
 0 <= w(6)(2)(3)(9)#910 <= 14
 0 <= w(6)(2)(3)(10)#911 <= 17
 0 <= w(7)(1)(1)(1)#912 <= 8
 0 <= w(7)(1)(1)(2)#913 <= 6
 0 <= w(7)(1)(1)(3)#914 <= 8
 0 <= w(7)(1)(1)(4)#915 <= 8
 0 <= w(7)(1)(1)(5)#916 <= 8
 0 <= w(7)(1)(1)(6)#917 <= 7
 0 <= w(7)(1)(1)(7)#918 <= 6
 0 <= w(7)(1)(1)(8)#919 <= 8
 0 <= w(7)(1)(1)(9)#920 <= 7
 0 <= w(7)(1)(1)(10)#921 <= 8
 0 <= w(7)(1)(2)(1)#922 <= 8
 0 <= w(7)(1)(2)(2)#923 <= 7
 0 <= w(7)(1)(2)(3)#924 <= 7
 0 <= w(7)(1)(2)(4)#925 <= 6
 0 <= w(7)(1)(2)(5)#926 <= 6
 0 <= w(7)(1)(2)(6)#927 <= 6
 0 <= w(7)(1)(2)(7)#928 <= 7
 0 <= w(7)(1)(2)(8)#929 <= 7
 0 <= w(7)(1)(2)(9)#930 <= 7
 0 <= w(7)(1)(2)(10)#931 <= 7
 0 <= w(7)(1)(3)(1)#932 <= 6
 0 <= w(7)(1)(3)(2)#933 <= 6
 0 <= w(7)(1)(3)(3)#934 <= 7
 0 <= w(7)(1)(3)(4)#935 <= 6
 0 <= w(7)(1)(3)(5)#936 <= 7
 0 <= w(7)(1)(3)(6)#937 <= 7
 0 <= w(7)(1)(3)(7)#938 <= 7
 0 <= w(7)(1)(3)(8)#939 <= 7
 0 <= w(7)(1)(3)(9)#940 <= 7
 0 <= w(7)(1)(3)(10)#941 <= 7
 0 <= w(7)(2)(1)(1)#942 <= 8
 0 <= w(7)(2)(1)(2)#943 <= 6
 0 <= w(7)(2)(1)(3)#944 <= 8
 0 <= w(7)(2)(1)(4)#945 <= 8
 0 <= w(7)(2)(1)(5)#946 <= 8
 0 <= w(7)(2)(1)(6)#947 <= 7
 0 <= w(7)(2)(1)(7)#948 <= 6
 0 <= w(7)(2)(1)(8)#949 <= 8
 0 <= w(7)(2)(1)(9)#950 <= 7
 0 <= w(7)(2)(1)(10)#951 <= 8
 0 <= w(7)(2)(2)(1)#952 <= 8
 0 <= w(7)(2)(2)(2)#953 <= 7
 0 <= w(7)(2)(2)(3)#954 <= 7
 0 <= w(7)(2)(2)(4)#955 <= 6
 0 <= w(7)(2)(2)(5)#956 <= 6
 0 <= w(7)(2)(2)(6)#957 <= 6
 0 <= w(7)(2)(2)(7)#958 <= 7
 0 <= w(7)(2)(2)(8)#959 <= 7
 0 <= w(7)(2)(2)(9)#960 <= 7
 0 <= w(7)(2)(2)(10)#961 <= 7
 0 <= w(7)(2)(3)(1)#962 <= 6
 0 <= w(7)(2)(3)(2)#963 <= 6
 0 <= w(7)(2)(3)(3)#964 <= 7
 0 <= w(7)(2)(3)(4)#965 <= 6
 0 <= w(7)(2)(3)(5)#966 <= 7
 0 <= w(7)(2)(3)(6)#967 <= 7
 0 <= w(7)(2)(3)(7)#968 <= 7
 0 <= w(7)(2)(3)(8)#969 <= 7
 0 <= w(7)(2)(3)(9)#970 <= 7
 0 <= w(7)(2)(3)(10)#971 <= 7
 0 <= w(8)(1)(1)(1)#972 <= 15
 0 <= w(8)(1)(1)(2)#973 <= 18
 0 <= w(8)(1)(1)(3)#974 <= 21
 0 <= w(8)(1)(1)(4)#975 <= 16
 0 <= w(8)(1)(1)(5)#976 <= 21
 0 <= w(8)(1)(1)(6)#977 <= 17
 0 <= w(8)(1)(1)(7)#978 <= 20
 0 <= w(8)(1)(1)(8)#979 <= 15
 0 <= w(8)(1)(1)(9)#980 <= 19
 0 <= w(8)(1)(1)(10)#981 <= 18
 0 <= w(8)(1)(2)(1)#982 <= 20
 0 <= w(8)(1)(2)(2)#983 <= 17
 0 <= w(8)(1)(2)(3)#984 <= 20
 0 <= w(8)(1)(2)(4)#985 <= 17
 0 <= w(8)(1)(2)(5)#986 <= 15
 0 <= w(8)(1)(2)(6)#987 <= 21
 0 <= w(8)(1)(2)(7)#988 <= 20
 0 <= w(8)(1)(2)(8)#989 <= 19
 0 <= w(8)(1)(2)(9)#990 <= 17
 0 <= w(8)(1)(2)(10)#991 <= 16
 0 <= w(8)(1)(3)(1)#992 <= 19
 0 <= w(8)(1)(3)(2)#993 <= 19
 0 <= w(8)(1)(3)(3)#994 <= 21
 0 <= w(8)(1)(3)(4)#995 <= 16
 0 <= w(8)(1)(3)(5)#996 <= 15
 0 <= w(8)(1)(3)(6)#997 <= 17
 0 <= w(8)(1)(3)(7)#998 <= 16
 0 <= w(8)(1)(3)(8)#999 <= 21
 0 <= w(8)(1)(3)(9)#1000 <= 15
 0 <= w(8)(1)(3)(10)#1001 <= 17
 0 <= w(8)(2)(1)(1)#1002 <= 15
 0 <= w(8)(2)(1)(2)#1003 <= 18
 0 <= w(8)(2)(1)(3)#1004 <= 21
 0 <= w(8)(2)(1)(4)#1005 <= 16
 0 <= w(8)(2)(1)(5)#1006 <= 21
 0 <= w(8)(2)(1)(6)#1007 <= 17
 0 <= w(8)(2)(1)(7)#1008 <= 20
 0 <= w(8)(2)(1)(8)#1009 <= 15
 0 <= w(8)(2)(1)(9)#1010 <= 19
 0 <= w(8)(2)(1)(10)#1011 <= 18
 0 <= w(8)(2)(2)(1)#1012 <= 20
 0 <= w(8)(2)(2)(2)#1013 <= 17
 0 <= w(8)(2)(2)(3)#1014 <= 20
 0 <= w(8)(2)(2)(4)#1015 <= 17
 0 <= w(8)(2)(2)(5)#1016 <= 15
 0 <= w(8)(2)(2)(6)#1017 <= 21
 0 <= w(8)(2)(2)(7)#1018 <= 20
 0 <= w(8)(2)(2)(8)#1019 <= 19
 0 <= w(8)(2)(2)(9)#1020 <= 17
 0 <= w(8)(2)(2)(10)#1021 <= 16
 0 <= w(8)(2)(3)(1)#1022 <= 19
 0 <= w(8)(2)(3)(2)#1023 <= 19
 0 <= w(8)(2)(3)(3)#1024 <= 21
 0 <= w(8)(2)(3)(4)#1025 <= 16
 0 <= w(8)(2)(3)(5)#1026 <= 15
 0 <= w(8)(2)(3)(6)#1027 <= 17
 0 <= w(8)(2)(3)(7)#1028 <= 16
 0 <= w(8)(2)(3)(8)#1029 <= 21
 0 <= w(8)(2)(3)(9)#1030 <= 15
 0 <= w(8)(2)(3)(10)#1031 <= 17
 0 <= w(9)(1)(1)(1)#1032 <= 19
 0 <= w(9)(1)(1)(2)#1033 <= 13
 0 <= w(9)(1)(1)(3)#1034 <= 14
 0 <= w(9)(1)(1)(4)#1035 <= 16
 0 <= w(9)(1)(1)(5)#1036 <= 19
 0 <= w(9)(1)(1)(6)#1037 <= 19
 0 <= w(9)(1)(1)(7)#1038 <= 13
 0 <= w(9)(1)(1)(8)#1039 <= 15
 0 <= w(9)(1)(1)(9)#1040 <= 17
 0 <= w(9)(1)(1)(10)#1041 <= 15
 0 <= w(9)(1)(2)(1)#1042 <= 15
 0 <= w(9)(1)(2)(2)#1043 <= 14
 0 <= w(9)(1)(2)(3)#1044 <= 16
 0 <= w(9)(1)(2)(4)#1045 <= 17
 0 <= w(9)(1)(2)(5)#1046 <= 19
 0 <= w(9)(1)(2)(6)#1047 <= 14
 0 <= w(9)(1)(2)(7)#1048 <= 15
 0 <= w(9)(1)(2)(8)#1049 <= 15
 0 <= w(9)(1)(2)(9)#1050 <= 18
 0 <= w(9)(1)(2)(10)#1051 <= 18
 0 <= w(9)(1)(3)(1)#1052 <= 17
 0 <= w(9)(1)(3)(2)#1053 <= 13
 0 <= w(9)(1)(3)(3)#1054 <= 19
 0 <= w(9)(1)(3)(4)#1055 <= 16
 0 <= w(9)(1)(3)(5)#1056 <= 17
 0 <= w(9)(1)(3)(6)#1057 <= 19
 0 <= w(9)(1)(3)(7)#1058 <= 19
 0 <= w(9)(1)(3)(8)#1059 <= 18
 0 <= w(9)(1)(3)(9)#1060 <= 17
 0 <= w(9)(1)(3)(10)#1061 <= 18
 0 <= w(9)(2)(1)(1)#1062 <= 19
 0 <= w(9)(2)(1)(2)#1063 <= 13
 0 <= w(9)(2)(1)(3)#1064 <= 14
 0 <= w(9)(2)(1)(4)#1065 <= 16
 0 <= w(9)(2)(1)(5)#1066 <= 19
 0 <= w(9)(2)(1)(6)#1067 <= 19
 0 <= w(9)(2)(1)(7)#1068 <= 13
 0 <= w(9)(2)(1)(8)#1069 <= 15
 0 <= w(9)(2)(1)(9)#1070 <= 17
 0 <= w(9)(2)(1)(10)#1071 <= 15
 0 <= w(9)(2)(2)(1)#1072 <= 15
 0 <= w(9)(2)(2)(2)#1073 <= 14
 0 <= w(9)(2)(2)(3)#1074 <= 16
 0 <= w(9)(2)(2)(4)#1075 <= 17
 0 <= w(9)(2)(2)(5)#1076 <= 19
 0 <= w(9)(2)(2)(6)#1077 <= 14
 0 <= w(9)(2)(2)(7)#1078 <= 15
 0 <= w(9)(2)(2)(8)#1079 <= 15
 0 <= w(9)(2)(2)(9)#1080 <= 18
 0 <= w(9)(2)(2)(10)#1081 <= 18
 0 <= w(9)(2)(3)(1)#1082 <= 17
 0 <= w(9)(2)(3)(2)#1083 <= 13
 0 <= w(9)(2)(3)(3)#1084 <= 19
 0 <= w(9)(2)(3)(4)#1085 <= 16
 0 <= w(9)(2)(3)(5)#1086 <= 17
 0 <= w(9)(2)(3)(6)#1087 <= 19
 0 <= w(9)(2)(3)(7)#1088 <= 19
 0 <= w(9)(2)(3)(8)#1089 <= 18
 0 <= w(9)(2)(3)(9)#1090 <= 17
 0 <= w(9)(2)(3)(10)#1091 <= 18
 0 <= w(10)(1)(1)(1)#1092 <= 14
 0 <= w(10)(1)(1)(2)#1093 <= 14
 0 <= w(10)(1)(1)(3)#1094 <= 18
 0 <= w(10)(1)(1)(4)#1095 <= 18
 0 <= w(10)(1)(1)(5)#1096 <= 13
 0 <= w(10)(1)(1)(6)#1097 <= 16
 0 <= w(10)(1)(1)(7)#1098 <= 16
 0 <= w(10)(1)(1)(8)#1099 <= 13
 0 <= w(10)(1)(1)(9)#1100 <= 14
 0 <= w(10)(1)(1)(10)#1101 <= 13
 0 <= w(10)(1)(2)(1)#1102 <= 14
 0 <= w(10)(1)(2)(2)#1103 <= 18
 0 <= w(10)(1)(2)(3)#1104 <= 13
 0 <= w(10)(1)(2)(4)#1105 <= 19
 0 <= w(10)(1)(2)(5)#1106 <= 16
 0 <= w(10)(1)(2)(6)#1107 <= 18
 0 <= w(10)(1)(2)(7)#1108 <= 17
 0 <= w(10)(1)(2)(8)#1109 <= 13
 0 <= w(10)(1)(2)(9)#1110 <= 14
 0 <= w(10)(1)(2)(10)#1111 <= 16
 0 <= w(10)(1)(3)(1)#1112 <= 16
 0 <= w(10)(1)(3)(2)#1113 <= 17
 0 <= w(10)(1)(3)(3)#1114 <= 19
 0 <= w(10)(1)(3)(4)#1115 <= 18
 0 <= w(10)(1)(3)(5)#1116 <= 14
 0 <= w(10)(1)(3)(6)#1117 <= 18
 0 <= w(10)(1)(3)(7)#1118 <= 18
 0 <= w(10)(1)(3)(8)#1119 <= 17
 0 <= w(10)(1)(3)(9)#1120 <= 19
 0 <= w(10)(1)(3)(10)#1121 <= 16
 0 <= w(10)(2)(1)(1)#1122 <= 14
 0 <= w(10)(2)(1)(2)#1123 <= 14
 0 <= w(10)(2)(1)(3)#1124 <= 18
 0 <= w(10)(2)(1)(4)#1125 <= 18
 0 <= w(10)(2)(1)(5)#1126 <= 13
 0 <= w(10)(2)(1)(6)#1127 <= 16
 0 <= w(10)(2)(1)(7)#1128 <= 16
 0 <= w(10)(2)(1)(8)#1129 <= 13
 0 <= w(10)(2)(1)(9)#1130 <= 14
 0 <= w(10)(2)(1)(10)#1131 <= 13
 0 <= w(10)(2)(2)(1)#1132 <= 14
 0 <= w(10)(2)(2)(2)#1133 <= 18
 0 <= w(10)(2)(2)(3)#1134 <= 13
 0 <= w(10)(2)(2)(4)#1135 <= 19
 0 <= w(10)(2)(2)(5)#1136 <= 16
 0 <= w(10)(2)(2)(6)#1137 <= 18
 0 <= w(10)(2)(2)(7)#1138 <= 17
 0 <= w(10)(2)(2)(8)#1139 <= 13
 0 <= w(10)(2)(2)(9)#1140 <= 14
 0 <= w(10)(2)(2)(10)#1141 <= 16
 0 <= w(10)(2)(3)(1)#1142 <= 16
 0 <= w(10)(2)(3)(2)#1143 <= 17
 0 <= w(10)(2)(3)(3)#1144 <= 19
 0 <= w(10)(2)(3)(4)#1145 <= 18
 0 <= w(10)(2)(3)(5)#1146 <= 14
 0 <= w(10)(2)(3)(6)#1147 <= 18
 0 <= w(10)(2)(3)(7)#1148 <= 18
 0 <= w(10)(2)(3)(8)#1149 <= 17
 0 <= w(10)(2)(3)(9)#1150 <= 19
 0 <= w(10)(2)(3)(10)#1151 <= 16
 0 <= w(11)(1)(1)(1)#1152 <= 13
 0 <= w(11)(1)(1)(2)#1153 <= 19
 0 <= w(11)(1)(1)(3)#1154 <= 15
 0 <= w(11)(1)(1)(4)#1155 <= 15
 0 <= w(11)(1)(1)(5)#1156 <= 19
 0 <= w(11)(1)(1)(6)#1157 <= 14
 0 <= w(11)(1)(1)(7)#1158 <= 17
 0 <= w(11)(1)(1)(8)#1159 <= 19
 0 <= w(11)(1)(1)(9)#1160 <= 18
 0 <= w(11)(1)(1)(10)#1161 <= 16
 0 <= w(11)(1)(2)(1)#1162 <= 13
 0 <= w(11)(1)(2)(2)#1163 <= 16
 0 <= w(11)(1)(2)(3)#1164 <= 18
 0 <= w(11)(1)(2)(4)#1165 <= 19
 0 <= w(11)(1)(2)(5)#1166 <= 18
 0 <= w(11)(1)(2)(6)#1167 <= 13
 0 <= w(11)(1)(2)(7)#1168 <= 17
 0 <= w(11)(1)(2)(8)#1169 <= 14
 0 <= w(11)(1)(2)(9)#1170 <= 13
 0 <= w(11)(1)(2)(10)#1171 <= 15
 0 <= w(11)(1)(3)(1)#1172 <= 14
 0 <= w(11)(1)(3)(2)#1173 <= 18
 0 <= w(11)(1)(3)(3)#1174 <= 17
 0 <= w(11)(1)(3)(4)#1175 <= 14
 0 <= w(11)(1)(3)(5)#1176 <= 14
 0 <= w(11)(1)(3)(6)#1177 <= 14
 0 <= w(11)(1)(3)(7)#1178 <= 17
 0 <= w(11)(1)(3)(8)#1179 <= 16
 0 <= w(11)(1)(3)(9)#1180 <= 16
 0 <= w(11)(1)(3)(10)#1181 <= 15
 0 <= w(11)(2)(1)(1)#1182 <= 13
 0 <= w(11)(2)(1)(2)#1183 <= 19
 0 <= w(11)(2)(1)(3)#1184 <= 15
 0 <= w(11)(2)(1)(4)#1185 <= 15
 0 <= w(11)(2)(1)(5)#1186 <= 19
 0 <= w(11)(2)(1)(6)#1187 <= 14
 0 <= w(11)(2)(1)(7)#1188 <= 17
 0 <= w(11)(2)(1)(8)#1189 <= 19
 0 <= w(11)(2)(1)(9)#1190 <= 18
 0 <= w(11)(2)(1)(10)#1191 <= 16
 0 <= w(11)(2)(2)(1)#1192 <= 13
 0 <= w(11)(2)(2)(2)#1193 <= 16
 0 <= w(11)(2)(2)(3)#1194 <= 18
 0 <= w(11)(2)(2)(4)#1195 <= 19
 0 <= w(11)(2)(2)(5)#1196 <= 18
 0 <= w(11)(2)(2)(6)#1197 <= 13
 0 <= w(11)(2)(2)(7)#1198 <= 17
 0 <= w(11)(2)(2)(8)#1199 <= 14
 0 <= w(11)(2)(2)(9)#1200 <= 13
 0 <= w(11)(2)(2)(10)#1201 <= 15
 0 <= w(11)(2)(3)(1)#1202 <= 14
 0 <= w(11)(2)(3)(2)#1203 <= 18
 0 <= w(11)(2)(3)(3)#1204 <= 17
 0 <= w(11)(2)(3)(4)#1205 <= 14
 0 <= w(11)(2)(3)(5)#1206 <= 14
 0 <= w(11)(2)(3)(6)#1207 <= 14
 0 <= w(11)(2)(3)(7)#1208 <= 17
 0 <= w(11)(2)(3)(8)#1209 <= 16
 0 <= w(11)(2)(3)(9)#1210 <= 16
 0 <= w(11)(2)(3)(10)#1211 <= 15
 0 <= w(12)(1)(1)(1)#1212 <= 13
 0 <= w(12)(1)(1)(2)#1213 <= 11
 0 <= w(12)(1)(1)(3)#1214 <= 13
 0 <= w(12)(1)(1)(4)#1215 <= 15
 0 <= w(12)(1)(1)(5)#1216 <= 14
 0 <= w(12)(1)(1)(6)#1217 <= 12
 0 <= w(12)(1)(1)(7)#1218 <= 12
 0 <= w(12)(1)(1)(8)#1219 <= 12
 0 <= w(12)(1)(1)(9)#1220 <= 11
 0 <= w(12)(1)(1)(10)#1221 <= 12
 0 <= w(12)(1)(2)(1)#1222 <= 14
 0 <= w(12)(1)(2)(2)#1223 <= 14
 0 <= w(12)(1)(2)(3)#1224 <= 13
 0 <= w(12)(1)(2)(4)#1225 <= 15
 0 <= w(12)(1)(2)(5)#1226 <= 12
 0 <= w(12)(1)(2)(6)#1227 <= 12
 0 <= w(12)(1)(2)(7)#1228 <= 14
 0 <= w(12)(1)(2)(8)#1229 <= 17
 0 <= w(12)(1)(2)(9)#1230 <= 16
 0 <= w(12)(1)(2)(10)#1231 <= 11
 0 <= w(12)(1)(3)(1)#1232 <= 16
 0 <= w(12)(1)(3)(2)#1233 <= 16
 0 <= w(12)(1)(3)(3)#1234 <= 13
 0 <= w(12)(1)(3)(4)#1235 <= 12
 0 <= w(12)(1)(3)(5)#1236 <= 12
 0 <= w(12)(1)(3)(6)#1237 <= 16
 0 <= w(12)(1)(3)(7)#1238 <= 16
 0 <= w(12)(1)(3)(8)#1239 <= 14
 0 <= w(12)(1)(3)(9)#1240 <= 13
 0 <= w(12)(1)(3)(10)#1241 <= 16
 0 <= w(12)(2)(1)(1)#1242 <= 13
 0 <= w(12)(2)(1)(2)#1243 <= 11
 0 <= w(12)(2)(1)(3)#1244 <= 13
 0 <= w(12)(2)(1)(4)#1245 <= 15
 0 <= w(12)(2)(1)(5)#1246 <= 14
 0 <= w(12)(2)(1)(6)#1247 <= 12
 0 <= w(12)(2)(1)(7)#1248 <= 12
 0 <= w(12)(2)(1)(8)#1249 <= 12
 0 <= w(12)(2)(1)(9)#1250 <= 11
 0 <= w(12)(2)(1)(10)#1251 <= 12
 0 <= w(12)(2)(2)(1)#1252 <= 14
 0 <= w(12)(2)(2)(2)#1253 <= 14
 0 <= w(12)(2)(2)(3)#1254 <= 13
 0 <= w(12)(2)(2)(4)#1255 <= 15
 0 <= w(12)(2)(2)(5)#1256 <= 12
 0 <= w(12)(2)(2)(6)#1257 <= 12
 0 <= w(12)(2)(2)(7)#1258 <= 14
 0 <= w(12)(2)(2)(8)#1259 <= 17
 0 <= w(12)(2)(2)(9)#1260 <= 16
 0 <= w(12)(2)(2)(10)#1261 <= 11
 0 <= w(12)(2)(3)(1)#1262 <= 16
 0 <= w(12)(2)(3)(2)#1263 <= 16
 0 <= w(12)(2)(3)(3)#1264 <= 13
 0 <= w(12)(2)(3)(4)#1265 <= 12
 0 <= w(12)(2)(3)(5)#1266 <= 12
 0 <= w(12)(2)(3)(6)#1267 <= 16
 0 <= w(12)(2)(3)(7)#1268 <= 16
 0 <= w(12)(2)(3)(8)#1269 <= 14
 0 <= w(12)(2)(3)(9)#1270 <= 13
 0 <= w(12)(2)(3)(10)#1271 <= 16
End
