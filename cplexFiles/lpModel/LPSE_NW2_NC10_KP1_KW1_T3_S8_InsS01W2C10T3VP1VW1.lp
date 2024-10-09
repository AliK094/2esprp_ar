\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 30 p(1)#0 + 30 p(2)#1 + 30 p(3)#2 + 3 I_plant(1)#3 + 3 I_plant(2)#4
       + 3 I_plant(3)#5 + 0.625 I_warehouse(1)(1)(1)#6
       + 0.625 I_warehouse(1)(1)(2)#7 + 0.625 I_warehouse(1)(1)(3)#8
       + 0.625 I_warehouse(1)(1)(4)#9 + 0.625 I_warehouse(1)(1)(5)#10
       + 0.625 I_warehouse(1)(1)(6)#11 + 0.625 I_warehouse(1)(1)(7)#12
       + 0.625 I_warehouse(1)(1)(8)#13 + 0.625 I_warehouse(1)(2)(1)#14
       + 0.625 I_warehouse(1)(2)(2)#15 + 0.625 I_warehouse(1)(2)(3)#16
       + 0.625 I_warehouse(1)(2)(4)#17 + 0.625 I_warehouse(1)(2)(5)#18
       + 0.625 I_warehouse(1)(2)(6)#19 + 0.625 I_warehouse(1)(2)(7)#20
       + 0.625 I_warehouse(1)(2)(8)#21 + 0.625 I_warehouse(1)(3)(1)#22
       + 0.625 I_warehouse(1)(3)(2)#23 + 0.625 I_warehouse(1)(3)(3)#24
       + 0.625 I_warehouse(1)(3)(4)#25 + 0.625 I_warehouse(1)(3)(5)#26
       + 0.625 I_warehouse(1)(3)(6)#27 + 0.625 I_warehouse(1)(3)(7)#28
       + 0.625 I_warehouse(1)(3)(8)#29 + 0.625 I_warehouse(2)(1)(1)#30
       + 0.625 I_warehouse(2)(1)(2)#31 + 0.625 I_warehouse(2)(1)(3)#32
       + 0.625 I_warehouse(2)(1)(4)#33 + 0.625 I_warehouse(2)(1)(5)#34
       + 0.625 I_warehouse(2)(1)(6)#35 + 0.625 I_warehouse(2)(1)(7)#36
       + 0.625 I_warehouse(2)(1)(8)#37 + 0.625 I_warehouse(2)(2)(1)#38
       + 0.625 I_warehouse(2)(2)(2)#39 + 0.625 I_warehouse(2)(2)(3)#40
       + 0.625 I_warehouse(2)(2)(4)#41 + 0.625 I_warehouse(2)(2)(5)#42
       + 0.625 I_warehouse(2)(2)(6)#43 + 0.625 I_warehouse(2)(2)(7)#44
       + 0.625 I_warehouse(2)(2)(8)#45 + 0.625 I_warehouse(2)(3)(1)#46
       + 0.625 I_warehouse(2)(3)(2)#47 + 0.625 I_warehouse(2)(3)(3)#48
       + 0.625 I_warehouse(2)(3)(4)#49 + 0.625 I_warehouse(2)(3)(5)#50
       + 0.625 I_warehouse(2)(3)(6)#51 + 0.625 I_warehouse(2)(3)(7)#52
       + 0.625 I_warehouse(2)(3)(8)#53 + 1.125 I_customer(3)(1)(1)#54
       + 1.125 I_customer(3)(1)(2)#55 + 1.125 I_customer(3)(1)(3)#56
       + 1.125 I_customer(3)(1)(4)#57 + 1.125 I_customer(3)(1)(5)#58
       + 1.125 I_customer(3)(1)(6)#59 + 1.125 I_customer(3)(1)(7)#60
       + 1.125 I_customer(3)(1)(8)#61 + 1.125 I_customer(3)(2)(1)#62
       + 1.125 I_customer(3)(2)(2)#63 + 1.125 I_customer(3)(2)(3)#64
       + 1.125 I_customer(3)(2)(4)#65 + 1.125 I_customer(3)(2)(5)#66
       + 1.125 I_customer(3)(2)(6)#67 + 1.125 I_customer(3)(2)(7)#68
       + 1.125 I_customer(3)(2)(8)#69 + 1.125 I_customer(3)(3)(1)#70
       + 1.125 I_customer(3)(3)(2)#71 + 1.125 I_customer(3)(3)(3)#72
       + 1.125 I_customer(3)(3)(4)#73 + 1.125 I_customer(3)(3)(5)#74
       + 1.125 I_customer(3)(3)(6)#75 + 1.125 I_customer(3)(3)(7)#76
       + 1.125 I_customer(3)(3)(8)#77 + 1.25 I_customer(4)(1)(1)#78
       + 1.25 I_customer(4)(1)(2)#79 + 1.25 I_customer(4)(1)(3)#80
       + 1.25 I_customer(4)(1)(4)#81 + 1.25 I_customer(4)(1)(5)#82
       + 1.25 I_customer(4)(1)(6)#83 + 1.25 I_customer(4)(1)(7)#84
       + 1.25 I_customer(4)(1)(8)#85 + 1.25 I_customer(4)(2)(1)#86
       + 1.25 I_customer(4)(2)(2)#87 + 1.25 I_customer(4)(2)(3)#88
       + 1.25 I_customer(4)(2)(4)#89 + 1.25 I_customer(4)(2)(5)#90
       + 1.25 I_customer(4)(2)(6)#91 + 1.25 I_customer(4)(2)(7)#92
       + 1.25 I_customer(4)(2)(8)#93 + 1.25 I_customer(4)(3)(1)#94
       + 1.25 I_customer(4)(3)(2)#95 + 1.25 I_customer(4)(3)(3)#96
       + 1.25 I_customer(4)(3)(4)#97 + 1.25 I_customer(4)(3)(5)#98
       + 1.25 I_customer(4)(3)(6)#99 + 1.25 I_customer(4)(3)(7)#100
       + 1.25 I_customer(4)(3)(8)#101 + 1.25 I_customer(5)(1)(1)#102
       + 1.25 I_customer(5)(1)(2)#103 + 1.25 I_customer(5)(1)(3)#104
       + 1.25 I_customer(5)(1)(4)#105 + 1.25 I_customer(5)(1)(5)#106
       + 1.25 I_customer(5)(1)(6)#107 + 1.25 I_customer(5)(1)(7)#108
       + 1.25 I_customer(5)(1)(8)#109 + 1.25 I_customer(5)(2)(1)#110
       + 1.25 I_customer(5)(2)(2)#111 + 1.25 I_customer(5)(2)(3)#112
       + 1.25 I_customer(5)(2)(4)#113 + 1.25 I_customer(5)(2)(5)#114
       + 1.25 I_customer(5)(2)(6)#115 + 1.25 I_customer(5)(2)(7)#116
       + 1.25 I_customer(5)(2)(8)#117 + 1.25 I_customer(5)(3)(1)#118
       + 1.25 I_customer(5)(3)(2)#119 + 1.25 I_customer(5)(3)(3)#120
       + 1.25 I_customer(5)(3)(4)#121 + 1.25 I_customer(5)(3)(5)#122
       + 1.25 I_customer(5)(3)(6)#123 + 1.25 I_customer(5)(3)(7)#124
       + 1.25 I_customer(5)(3)(8)#125 + I_customer(6)(1)(1)#126
       + I_customer(6)(1)(2)#127 + I_customer(6)(1)(3)#128
       + I_customer(6)(1)(4)#129 + I_customer(6)(1)(5)#130
       + I_customer(6)(1)(6)#131 + I_customer(6)(1)(7)#132
       + I_customer(6)(1)(8)#133 + I_customer(6)(2)(1)#134
       + I_customer(6)(2)(2)#135 + I_customer(6)(2)(3)#136
       + I_customer(6)(2)(4)#137 + I_customer(6)(2)(5)#138
       + I_customer(6)(2)(6)#139 + I_customer(6)(2)(7)#140
       + I_customer(6)(2)(8)#141 + I_customer(6)(3)(1)#142
       + I_customer(6)(3)(2)#143 + I_customer(6)(3)(3)#144
       + I_customer(6)(3)(4)#145 + I_customer(6)(3)(5)#146
       + I_customer(6)(3)(6)#147 + I_customer(6)(3)(7)#148
       + I_customer(6)(3)(8)#149 + I_customer(7)(1)(1)#150
       + I_customer(7)(1)(2)#151 + I_customer(7)(1)(3)#152
       + I_customer(7)(1)(4)#153 + I_customer(7)(1)(5)#154
       + I_customer(7)(1)(6)#155 + I_customer(7)(1)(7)#156
       + I_customer(7)(1)(8)#157 + I_customer(7)(2)(1)#158
       + I_customer(7)(2)(2)#159 + I_customer(7)(2)(3)#160
       + I_customer(7)(2)(4)#161 + I_customer(7)(2)(5)#162
       + I_customer(7)(2)(6)#163 + I_customer(7)(2)(7)#164
       + I_customer(7)(2)(8)#165 + I_customer(7)(3)(1)#166
       + I_customer(7)(3)(2)#167 + I_customer(7)(3)(3)#168
       + I_customer(7)(3)(4)#169 + I_customer(7)(3)(5)#170
       + I_customer(7)(3)(6)#171 + I_customer(7)(3)(7)#172
       + I_customer(7)(3)(8)#173 + 1.25 I_customer(8)(1)(1)#174
       + 1.25 I_customer(8)(1)(2)#175 + 1.25 I_customer(8)(1)(3)#176
       + 1.25 I_customer(8)(1)(4)#177 + 1.25 I_customer(8)(1)(5)#178
       + 1.25 I_customer(8)(1)(6)#179 + 1.25 I_customer(8)(1)(7)#180
       + 1.25 I_customer(8)(1)(8)#181 + 1.25 I_customer(8)(2)(1)#182
       + 1.25 I_customer(8)(2)(2)#183 + 1.25 I_customer(8)(2)(3)#184
       + 1.25 I_customer(8)(2)(4)#185 + 1.25 I_customer(8)(2)(5)#186
       + 1.25 I_customer(8)(2)(6)#187 + 1.25 I_customer(8)(2)(7)#188
       + 1.25 I_customer(8)(2)(8)#189 + 1.25 I_customer(8)(3)(1)#190
       + 1.25 I_customer(8)(3)(2)#191 + 1.25 I_customer(8)(3)(3)#192
       + 1.25 I_customer(8)(3)(4)#193 + 1.25 I_customer(8)(3)(5)#194
       + 1.25 I_customer(8)(3)(6)#195 + 1.25 I_customer(8)(3)(7)#196
       + 1.25 I_customer(8)(3)(8)#197 + 1.25 I_customer(9)(1)(1)#198
       + 1.25 I_customer(9)(1)(2)#199 + 1.25 I_customer(9)(1)(3)#200
       + 1.25 I_customer(9)(1)(4)#201 + 1.25 I_customer(9)(1)(5)#202
       + 1.25 I_customer(9)(1)(6)#203 + 1.25 I_customer(9)(1)(7)#204
       + 1.25 I_customer(9)(1)(8)#205 + 1.25 I_customer(9)(2)(1)#206
       + 1.25 I_customer(9)(2)(2)#207 + 1.25 I_customer(9)(2)(3)#208
       + 1.25 I_customer(9)(2)(4)#209 + 1.25 I_customer(9)(2)(5)#210
       + 1.25 I_customer(9)(2)(6)#211 + 1.25 I_customer(9)(2)(7)#212
       + 1.25 I_customer(9)(2)(8)#213 + 1.25 I_customer(9)(3)(1)#214
       + 1.25 I_customer(9)(3)(2)#215 + 1.25 I_customer(9)(3)(3)#216
       + 1.25 I_customer(9)(3)(4)#217 + 1.25 I_customer(9)(3)(5)#218
       + 1.25 I_customer(9)(3)(6)#219 + 1.25 I_customer(9)(3)(7)#220
       + 1.25 I_customer(9)(3)(8)#221 + 0.875 I_customer(10)(1)(1)#222
       + 0.875 I_customer(10)(1)(2)#223 + 0.875 I_customer(10)(1)(3)#224
       + 0.875 I_customer(10)(1)(4)#225 + 0.875 I_customer(10)(1)(5)#226
       + 0.875 I_customer(10)(1)(6)#227 + 0.875 I_customer(10)(1)(7)#228
       + 0.875 I_customer(10)(1)(8)#229 + 0.875 I_customer(10)(2)(1)#230
       + 0.875 I_customer(10)(2)(2)#231 + 0.875 I_customer(10)(2)(3)#232
       + 0.875 I_customer(10)(2)(4)#233 + 0.875 I_customer(10)(2)(5)#234
       + 0.875 I_customer(10)(2)(6)#235 + 0.875 I_customer(10)(2)(7)#236
       + 0.875 I_customer(10)(2)(8)#237 + 0.875 I_customer(10)(3)(1)#238
       + 0.875 I_customer(10)(3)(2)#239 + 0.875 I_customer(10)(3)(3)#240
       + 0.875 I_customer(10)(3)(4)#241 + 0.875 I_customer(10)(3)(5)#242
       + 0.875 I_customer(10)(3)(6)#243 + 0.875 I_customer(10)(3)(7)#244
       + 0.875 I_customer(10)(3)(8)#245 + 1.125 I_customer(11)(1)(1)#246
       + 1.125 I_customer(11)(1)(2)#247 + 1.125 I_customer(11)(1)(3)#248
       + 1.125 I_customer(11)(1)(4)#249 + 1.125 I_customer(11)(1)(5)#250
       + 1.125 I_customer(11)(1)(6)#251 + 1.125 I_customer(11)(1)(7)#252
       + 1.125 I_customer(11)(1)(8)#253 + 1.125 I_customer(11)(2)(1)#254
       + 1.125 I_customer(11)(2)(2)#255 + 1.125 I_customer(11)(2)(3)#256
       + 1.125 I_customer(11)(2)(4)#257 + 1.125 I_customer(11)(2)(5)#258
       + 1.125 I_customer(11)(2)(6)#259 + 1.125 I_customer(11)(2)(7)#260
       + 1.125 I_customer(11)(2)(8)#261 + 1.125 I_customer(11)(3)(1)#262
       + 1.125 I_customer(11)(3)(2)#263 + 1.125 I_customer(11)(3)(3)#264
       + 1.125 I_customer(11)(3)(4)#265 + 1.125 I_customer(11)(3)(5)#266
       + 1.125 I_customer(11)(3)(6)#267 + 1.125 I_customer(11)(3)(7)#268
       + 1.125 I_customer(11)(3)(8)#269 + 0.875 I_customer(12)(1)(1)#270
       + 0.875 I_customer(12)(1)(2)#271 + 0.875 I_customer(12)(1)(3)#272
       + 0.875 I_customer(12)(1)(4)#273 + 0.875 I_customer(12)(1)(5)#274
       + 0.875 I_customer(12)(1)(6)#275 + 0.875 I_customer(12)(1)(7)#276
       + 0.875 I_customer(12)(1)(8)#277 + 0.875 I_customer(12)(2)(1)#278
       + 0.875 I_customer(12)(2)(2)#279 + 0.875 I_customer(12)(2)(3)#280
       + 0.875 I_customer(12)(2)(4)#281 + 0.875 I_customer(12)(2)(5)#282
       + 0.875 I_customer(12)(2)(6)#283 + 0.875 I_customer(12)(2)(7)#284
       + 0.875 I_customer(12)(2)(8)#285 + 0.875 I_customer(12)(3)(1)#286
       + 0.875 I_customer(12)(3)(2)#287 + 0.875 I_customer(12)(3)(3)#288
       + 0.875 I_customer(12)(3)(4)#289 + 0.875 I_customer(12)(3)(5)#290
       + 0.875 I_customer(12)(3)(6)#291 + 0.875 I_customer(12)(3)(7)#292
       + 0.875 I_customer(12)(3)(8)#293 + 58.75 b_customer(3)(1)(1)#294
       + 58.75 b_customer(3)(1)(2)#295 + 58.75 b_customer(3)(1)(3)#296
       + 58.75 b_customer(3)(1)(4)#297 + 58.75 b_customer(3)(1)(5)#298
       + 58.75 b_customer(3)(1)(6)#299 + 58.75 b_customer(3)(1)(7)#300
       + 58.75 b_customer(3)(1)(8)#301 + 58.75 b_customer(3)(2)(1)#302
       + 58.75 b_customer(3)(2)(2)#303 + 58.75 b_customer(3)(2)(3)#304
       + 58.75 b_customer(3)(2)(4)#305 + 58.75 b_customer(3)(2)(5)#306
       + 58.75 b_customer(3)(2)(6)#307 + 58.75 b_customer(3)(2)(7)#308
       + 58.75 b_customer(3)(2)(8)#309 + 58.75 b_customer(3)(3)(1)#310
       + 58.75 b_customer(3)(3)(2)#311 + 58.75 b_customer(3)(3)(3)#312
       + 58.75 b_customer(3)(3)(4)#313 + 58.75 b_customer(3)(3)(5)#314
       + 58.75 b_customer(3)(3)(6)#315 + 58.75 b_customer(3)(3)(7)#316
       + 58.75 b_customer(3)(3)(8)#317 + 57.5 b_customer(4)(1)(1)#318
       + 57.5 b_customer(4)(1)(2)#319 + 57.5 b_customer(4)(1)(3)#320
       + 57.5 b_customer(4)(1)(4)#321 + 57.5 b_customer(4)(1)(5)#322
       + 57.5 b_customer(4)(1)(6)#323 + 57.5 b_customer(4)(1)(7)#324
       + 57.5 b_customer(4)(1)(8)#325 + 57.5 b_customer(4)(2)(1)#326
       + 57.5 b_customer(4)(2)(2)#327 + 57.5 b_customer(4)(2)(3)#328
       + 57.5 b_customer(4)(2)(4)#329 + 57.5 b_customer(4)(2)(5)#330
       + 57.5 b_customer(4)(2)(6)#331 + 57.5 b_customer(4)(2)(7)#332
       + 57.5 b_customer(4)(2)(8)#333 + 57.5 b_customer(4)(3)(1)#334
       + 57.5 b_customer(4)(3)(2)#335 + 57.5 b_customer(4)(3)(3)#336
       + 57.5 b_customer(4)(3)(4)#337 + 57.5 b_customer(4)(3)(5)#338
       + 57.5 b_customer(4)(3)(6)#339 + 57.5 b_customer(4)(3)(7)#340
       + 57.5 b_customer(4)(3)(8)#341 + 60 b_customer(5)(1)(1)#342
       + 60 b_customer(5)(1)(2)#343 + 60 b_customer(5)(1)(3)#344
       + 60 b_customer(5)(1)(4)#345 + 60 b_customer(5)(1)(5)#346
       + 60 b_customer(5)(1)(6)#347 + 60 b_customer(5)(1)(7)#348
       + 60 b_customer(5)(1)(8)#349 + 60 b_customer(5)(2)(1)#350
       + 60 b_customer(5)(2)(2)#351 + 60 b_customer(5)(2)(3)#352
       + 60 b_customer(5)(2)(4)#353 + 60 b_customer(5)(2)(5)#354
       + 60 b_customer(5)(2)(6)#355 + 60 b_customer(5)(2)(7)#356
       + 60 b_customer(5)(2)(8)#357 + 60 b_customer(5)(3)(1)#358
       + 60 b_customer(5)(3)(2)#359 + 60 b_customer(5)(3)(3)#360
       + 60 b_customer(5)(3)(4)#361 + 60 b_customer(5)(3)(5)#362
       + 60 b_customer(5)(3)(6)#363 + 60 b_customer(5)(3)(7)#364
       + 60 b_customer(5)(3)(8)#365 + 58.75 b_customer(6)(1)(1)#366
       + 58.75 b_customer(6)(1)(2)#367 + 58.75 b_customer(6)(1)(3)#368
       + 58.75 b_customer(6)(1)(4)#369 + 58.75 b_customer(6)(1)(5)#370
       + 58.75 b_customer(6)(1)(6)#371 + 58.75 b_customer(6)(1)(7)#372
       + 58.75 b_customer(6)(1)(8)#373 + 58.75 b_customer(6)(2)(1)#374
       + 58.75 b_customer(6)(2)(2)#375 + 58.75 b_customer(6)(2)(3)#376
       + 58.75 b_customer(6)(2)(4)#377 + 58.75 b_customer(6)(2)(5)#378
       + 58.75 b_customer(6)(2)(6)#379 + 58.75 b_customer(6)(2)(7)#380
       + 58.75 b_customer(6)(2)(8)#381 + 58.75 b_customer(6)(3)(1)#382
       + 58.75 b_customer(6)(3)(2)#383 + 58.75 b_customer(6)(3)(3)#384
       + 58.75 b_customer(6)(3)(4)#385 + 58.75 b_customer(6)(3)(5)#386
       + 58.75 b_customer(6)(3)(6)#387 + 58.75 b_customer(6)(3)(7)#388
       + 58.75 b_customer(6)(3)(8)#389 + 60 b_customer(7)(1)(1)#390
       + 60 b_customer(7)(1)(2)#391 + 60 b_customer(7)(1)(3)#392
       + 60 b_customer(7)(1)(4)#393 + 60 b_customer(7)(1)(5)#394
       + 60 b_customer(7)(1)(6)#395 + 60 b_customer(7)(1)(7)#396
       + 60 b_customer(7)(1)(8)#397 + 60 b_customer(7)(2)(1)#398
       + 60 b_customer(7)(2)(2)#399 + 60 b_customer(7)(2)(3)#400
       + 60 b_customer(7)(2)(4)#401 + 60 b_customer(7)(2)(5)#402
       + 60 b_customer(7)(2)(6)#403 + 60 b_customer(7)(2)(7)#404
       + 60 b_customer(7)(2)(8)#405 + 60 b_customer(7)(3)(1)#406
       + 60 b_customer(7)(3)(2)#407 + 60 b_customer(7)(3)(3)#408
       + 60 b_customer(7)(3)(4)#409 + 60 b_customer(7)(3)(5)#410
       + 60 b_customer(7)(3)(6)#411 + 60 b_customer(7)(3)(7)#412
       + 60 b_customer(7)(3)(8)#413 + 58.75 b_customer(8)(1)(1)#414
       + 58.75 b_customer(8)(1)(2)#415 + 58.75 b_customer(8)(1)(3)#416
       + 58.75 b_customer(8)(1)(4)#417 + 58.75 b_customer(8)(1)(5)#418
       + 58.75 b_customer(8)(1)(6)#419 + 58.75 b_customer(8)(1)(7)#420
       + 58.75 b_customer(8)(1)(8)#421 + 58.75 b_customer(8)(2)(1)#422
       + 58.75 b_customer(8)(2)(2)#423 + 58.75 b_customer(8)(2)(3)#424
       + 58.75 b_customer(8)(2)(4)#425 + 58.75 b_customer(8)(2)(5)#426
       + 58.75 b_customer(8)(2)(6)#427 + 58.75 b_customer(8)(2)(7)#428
       + 58.75 b_customer(8)(2)(8)#429 + 58.75 b_customer(8)(3)(1)#430
       + 58.75 b_customer(8)(3)(2)#431 + 58.75 b_customer(8)(3)(3)#432
       + 58.75 b_customer(8)(3)(4)#433 + 58.75 b_customer(8)(3)(5)#434
       + 58.75 b_customer(8)(3)(6)#435 + 58.75 b_customer(8)(3)(7)#436
       + 58.75 b_customer(8)(3)(8)#437 + 58.75 b_customer(9)(1)(1)#438
       + 58.75 b_customer(9)(1)(2)#439 + 58.75 b_customer(9)(1)(3)#440
       + 58.75 b_customer(9)(1)(4)#441 + 58.75 b_customer(9)(1)(5)#442
       + 58.75 b_customer(9)(1)(6)#443 + 58.75 b_customer(9)(1)(7)#444
       + 58.75 b_customer(9)(1)(8)#445 + 58.75 b_customer(9)(2)(1)#446
       + 58.75 b_customer(9)(2)(2)#447 + 58.75 b_customer(9)(2)(3)#448
       + 58.75 b_customer(9)(2)(4)#449 + 58.75 b_customer(9)(2)(5)#450
       + 58.75 b_customer(9)(2)(6)#451 + 58.75 b_customer(9)(2)(7)#452
       + 58.75 b_customer(9)(2)(8)#453 + 58.75 b_customer(9)(3)(1)#454
       + 58.75 b_customer(9)(3)(2)#455 + 58.75 b_customer(9)(3)(3)#456
       + 58.75 b_customer(9)(3)(4)#457 + 58.75 b_customer(9)(3)(5)#458
       + 58.75 b_customer(9)(3)(6)#459 + 58.75 b_customer(9)(3)(7)#460
       + 58.75 b_customer(9)(3)(8)#461 + 57.5 b_customer(10)(1)(1)#462
       + 57.5 b_customer(10)(1)(2)#463 + 57.5 b_customer(10)(1)(3)#464
       + 57.5 b_customer(10)(1)(4)#465 + 57.5 b_customer(10)(1)(5)#466
       + 57.5 b_customer(10)(1)(6)#467 + 57.5 b_customer(10)(1)(7)#468
       + 57.5 b_customer(10)(1)(8)#469 + 57.5 b_customer(10)(2)(1)#470
       + 57.5 b_customer(10)(2)(2)#471 + 57.5 b_customer(10)(2)(3)#472
       + 57.5 b_customer(10)(2)(4)#473 + 57.5 b_customer(10)(2)(5)#474
       + 57.5 b_customer(10)(2)(6)#475 + 57.5 b_customer(10)(2)(7)#476
       + 57.5 b_customer(10)(2)(8)#477 + 57.5 b_customer(10)(3)(1)#478
       + 57.5 b_customer(10)(3)(2)#479 + 57.5 b_customer(10)(3)(3)#480
       + 57.5 b_customer(10)(3)(4)#481 + 57.5 b_customer(10)(3)(5)#482
       + 57.5 b_customer(10)(3)(6)#483 + 57.5 b_customer(10)(3)(7)#484
       + 57.5 b_customer(10)(3)(8)#485 + 57.5 b_customer(11)(1)(1)#486
       + 57.5 b_customer(11)(1)(2)#487 + 57.5 b_customer(11)(1)(3)#488
       + 57.5 b_customer(11)(1)(4)#489 + 57.5 b_customer(11)(1)(5)#490
       + 57.5 b_customer(11)(1)(6)#491 + 57.5 b_customer(11)(1)(7)#492
       + 57.5 b_customer(11)(1)(8)#493 + 57.5 b_customer(11)(2)(1)#494
       + 57.5 b_customer(11)(2)(2)#495 + 57.5 b_customer(11)(2)(3)#496
       + 57.5 b_customer(11)(2)(4)#497 + 57.5 b_customer(11)(2)(5)#498
       + 57.5 b_customer(11)(2)(6)#499 + 57.5 b_customer(11)(2)(7)#500
       + 57.5 b_customer(11)(2)(8)#501 + 57.5 b_customer(11)(3)(1)#502
       + 57.5 b_customer(11)(3)(2)#503 + 57.5 b_customer(11)(3)(3)#504
       + 57.5 b_customer(11)(3)(4)#505 + 57.5 b_customer(11)(3)(5)#506
       + 57.5 b_customer(11)(3)(6)#507 + 57.5 b_customer(11)(3)(7)#508
       + 57.5 b_customer(11)(3)(8)#509 + 57.5 b_customer(12)(1)(1)#510
       + 57.5 b_customer(12)(1)(2)#511 + 57.5 b_customer(12)(1)(3)#512
       + 57.5 b_customer(12)(1)(4)#513 + 57.5 b_customer(12)(1)(5)#514
       + 57.5 b_customer(12)(1)(6)#515 + 57.5 b_customer(12)(1)(7)#516
       + 57.5 b_customer(12)(1)(8)#517 + 57.5 b_customer(12)(2)(1)#518
       + 57.5 b_customer(12)(2)(2)#519 + 57.5 b_customer(12)(2)(3)#520
       + 57.5 b_customer(12)(2)(4)#521 + 57.5 b_customer(12)(2)(5)#522
       + 57.5 b_customer(12)(2)(6)#523 + 57.5 b_customer(12)(2)(7)#524
       + 57.5 b_customer(12)(2)(8)#525 + 57.5 b_customer(12)(3)(1)#526
       + 57.5 b_customer(12)(3)(2)#527 + 57.5 b_customer(12)(3)(3)#528
       + 57.5 b_customer(12)(3)(4)#529 + 57.5 b_customer(12)(3)(5)#530
       + 57.5 b_customer(12)(3)(6)#531 + 57.5 b_customer(12)(3)(7)#532
       + 57.5 b_customer(12)(3)(8)#533
Subject To
 ProductionCapacity(1):          p(1)#0 <= 270
 ProductionCapacity(2):          p(2)#1 <= 0
 ProductionCapacity(3):          p(3)#2 <= 0
 PlantInventoryCapacity(1):      I_plant(1)#3 <= 135
 PlantInventoryCapacity(2):      I_plant(2)#4 <= 135
 PlantInventoryCapacity(3):      I_plant(3)#5 <= 135
 PlantInventoryBalance(1):       - p(1)#0 + I_plant(1)#3 + q(1)(1)(1)#534
                                 + q(2)(1)(1)#537  = 0
 PlantInventoryBalance(2):       - p(2)#1 - I_plant(1)#3 + I_plant(2)#4
                                 + q(1)(1)(2)#535 + q(2)(1)(2)#538  = 0
 PlantInventoryBalance(3):       - p(3)#2 - I_plant(2)#4 + I_plant(3)#5
                                 + q(1)(1)(3)#536 + q(2)(1)(3)#539  = 0
 WarehouseInventoryCapacity(1,1,1): I_warehouse(1)(1)(1)#6 <= 124
 WarehouseInventoryCapacity(2,1,1): I_warehouse(2)(1)(1)#30 <= 124
 WarehouseInventoryCapacity(1,2,1): I_warehouse(1)(2)(1)#14 <= 124
 WarehouseInventoryCapacity(2,2,1): I_warehouse(2)(2)(1)#38 <= 124
 WarehouseInventoryCapacity(1,3,1): I_warehouse(1)(3)(1)#22 <= 124
 WarehouseInventoryCapacity(2,3,1): I_warehouse(2)(3)(1)#46 <= 124
 WarehouseInventoryCapacity(1,1,2): I_warehouse(1)(1)(2)#7 <= 124
 WarehouseInventoryCapacity(2,1,2): I_warehouse(2)(1)(2)#31 <= 124
 WarehouseInventoryCapacity(1,2,2): I_warehouse(1)(2)(2)#15 <= 124
 WarehouseInventoryCapacity(2,2,2): I_warehouse(2)(2)(2)#39 <= 124
 WarehouseInventoryCapacity(1,3,2): I_warehouse(1)(3)(2)#23 <= 124
 WarehouseInventoryCapacity(2,3,2): I_warehouse(2)(3)(2)#47 <= 124
 WarehouseInventoryCapacity(1,1,3): I_warehouse(1)(1)(3)#8 <= 124
 WarehouseInventoryCapacity(2,1,3): I_warehouse(2)(1)(3)#32 <= 124
 WarehouseInventoryCapacity(1,2,3): I_warehouse(1)(2)(3)#16 <= 124
 WarehouseInventoryCapacity(2,2,3): I_warehouse(2)(2)(3)#40 <= 124
 WarehouseInventoryCapacity(1,3,3): I_warehouse(1)(3)(3)#24 <= 124
 WarehouseInventoryCapacity(2,3,3): I_warehouse(2)(3)(3)#48 <= 124
 WarehouseInventoryCapacity(1,1,4): I_warehouse(1)(1)(4)#9 <= 124
 WarehouseInventoryCapacity(2,1,4): I_warehouse(2)(1)(4)#33 <= 124
 WarehouseInventoryCapacity(1,2,4): I_warehouse(1)(2)(4)#17 <= 124
 WarehouseInventoryCapacity(2,2,4): I_warehouse(2)(2)(4)#41 <= 124
 WarehouseInventoryCapacity(1,3,4): I_warehouse(1)(3)(4)#25 <= 124
 WarehouseInventoryCapacity(2,3,4): I_warehouse(2)(3)(4)#49 <= 124
 WarehouseInventoryCapacity(1,1,5): I_warehouse(1)(1)(5)#10 <= 124
 WarehouseInventoryCapacity(2,1,5): I_warehouse(2)(1)(5)#34 <= 124
 WarehouseInventoryCapacity(1,2,5): I_warehouse(1)(2)(5)#18 <= 124
 WarehouseInventoryCapacity(2,2,5): I_warehouse(2)(2)(5)#42 <= 124
 WarehouseInventoryCapacity(1,3,5): I_warehouse(1)(3)(5)#26 <= 124
 WarehouseInventoryCapacity(2,3,5): I_warehouse(2)(3)(5)#50 <= 124
 WarehouseInventoryCapacity(1,1,6): I_warehouse(1)(1)(6)#11 <= 124
 WarehouseInventoryCapacity(2,1,6): I_warehouse(2)(1)(6)#35 <= 124
 WarehouseInventoryCapacity(1,2,6): I_warehouse(1)(2)(6)#19 <= 124
 WarehouseInventoryCapacity(2,2,6): I_warehouse(2)(2)(6)#43 <= 124
 WarehouseInventoryCapacity(1,3,6): I_warehouse(1)(3)(6)#27 <= 124
 WarehouseInventoryCapacity(2,3,6): I_warehouse(2)(3)(6)#51 <= 124
 WarehouseInventoryCapacity(1,1,7): I_warehouse(1)(1)(7)#12 <= 124
 WarehouseInventoryCapacity(2,1,7): I_warehouse(2)(1)(7)#36 <= 124
 WarehouseInventoryCapacity(1,2,7): I_warehouse(1)(2)(7)#20 <= 124
 WarehouseInventoryCapacity(2,2,7): I_warehouse(2)(2)(7)#44 <= 124
 WarehouseInventoryCapacity(1,3,7): I_warehouse(1)(3)(7)#28 <= 124
 WarehouseInventoryCapacity(2,3,7): I_warehouse(2)(3)(7)#52 <= 124
 WarehouseInventoryCapacity(1,1,8): I_warehouse(1)(1)(8)#13 <= 124
 WarehouseInventoryCapacity(2,1,8): I_warehouse(2)(1)(8)#37 <= 124
 WarehouseInventoryCapacity(1,2,8): I_warehouse(1)(2)(8)#21 <= 124
 WarehouseInventoryCapacity(2,2,8): I_warehouse(2)(2)(8)#45 <= 124
 WarehouseInventoryCapacity(1,3,8): I_warehouse(1)(3)(8)#29 <= 124
 WarehouseInventoryCapacity(2,3,8): I_warehouse(2)(3)(8)#53 <= 124
 WarehouseInventoryBalance(1,1,1): I_warehouse(1)(1)(1)#6 - q(1)(1)(1)#534
                                 + w(3)(1)(1)(1)#540 + w(4)(1)(1)(1)#588
                                 + w(5)(1)(1)(1)#636 + w(6)(1)(1)(1)#684
                                 + w(7)(1)(1)(1)#732 + w(8)(1)(1)(1)#780
                                 + w(9)(1)(1)(1)#828 + w(10)(1)(1)(1)#876
                                 + w(11)(1)(1)(1)#924 + w(12)(1)(1)(1)#972  = 0
 WarehouseInventoryBalance(2,1,1): I_warehouse(2)(1)(1)#30 - q(2)(1)(1)#537
                                 + w(3)(2)(1)(1)#564 + w(4)(2)(1)(1)#612
                                 + w(5)(2)(1)(1)#660 + w(6)(2)(1)(1)#708
                                 + w(7)(2)(1)(1)#756 + w(8)(2)(1)(1)#804
                                 + w(9)(2)(1)(1)#852 + w(10)(2)(1)(1)#900
                                 + w(11)(2)(1)(1)#948 + w(12)(2)(1)(1)#996  = 0
 WarehouseInventoryBalance(1,2,1): - I_warehouse(1)(1)(1)#6
                                 + I_warehouse(1)(2)(1)#14 - q(1)(1)(2)#535
                                 + w(3)(1)(2)(1)#548 + w(4)(1)(2)(1)#596
                                 + w(5)(1)(2)(1)#644 + w(6)(1)(2)(1)#692
                                 + w(7)(1)(2)(1)#740 + w(8)(1)(2)(1)#788
                                 + w(9)(1)(2)(1)#836 + w(10)(1)(2)(1)#884
                                 + w(11)(1)(2)(1)#932 + w(12)(1)(2)(1)#980  = 0
 WarehouseInventoryBalance(2,2,1): - I_warehouse(2)(1)(1)#30
                                 + I_warehouse(2)(2)(1)#38 - q(2)(1)(2)#538
                                 + w(3)(2)(2)(1)#572 + w(4)(2)(2)(1)#620
                                 + w(5)(2)(2)(1)#668 + w(6)(2)(2)(1)#716
                                 + w(7)(2)(2)(1)#764 + w(8)(2)(2)(1)#812
                                 + w(9)(2)(2)(1)#860 + w(10)(2)(2)(1)#908
                                 + w(11)(2)(2)(1)#956 + w(12)(2)(2)(1)#1004
                                  = 0
 WarehouseInventoryBalance(1,3,1): - I_warehouse(1)(2)(1)#14
                                 + I_warehouse(1)(3)(1)#22 - q(1)(1)(3)#536
                                 + w(3)(1)(3)(1)#556 + w(4)(1)(3)(1)#604
                                 + w(5)(1)(3)(1)#652 + w(6)(1)(3)(1)#700
                                 + w(7)(1)(3)(1)#748 + w(8)(1)(3)(1)#796
                                 + w(9)(1)(3)(1)#844 + w(10)(1)(3)(1)#892
                                 + w(11)(1)(3)(1)#940 + w(12)(1)(3)(1)#988  = 0
 WarehouseInventoryBalance(2,3,1): - I_warehouse(2)(2)(1)#38
                                 + I_warehouse(2)(3)(1)#46 - q(2)(1)(3)#539
                                 + w(3)(2)(3)(1)#580 + w(4)(2)(3)(1)#628
                                 + w(5)(2)(3)(1)#676 + w(6)(2)(3)(1)#724
                                 + w(7)(2)(3)(1)#772 + w(8)(2)(3)(1)#820
                                 + w(9)(2)(3)(1)#868 + w(10)(2)(3)(1)#916
                                 + w(11)(2)(3)(1)#964 + w(12)(2)(3)(1)#1012
                                  = 0
 WarehouseInventoryBalance(1,1,2): I_warehouse(1)(1)(2)#7 - q(1)(1)(1)#534
                                 + w(3)(1)(1)(2)#541 + w(4)(1)(1)(2)#589
                                 + w(5)(1)(1)(2)#637 + w(6)(1)(1)(2)#685
                                 + w(7)(1)(1)(2)#733 + w(8)(1)(1)(2)#781
                                 + w(9)(1)(1)(2)#829 + w(10)(1)(1)(2)#877
                                 + w(11)(1)(1)(2)#925 + w(12)(1)(1)(2)#973  = 0
 WarehouseInventoryBalance(2,1,2): I_warehouse(2)(1)(2)#31 - q(2)(1)(1)#537
                                 + w(3)(2)(1)(2)#565 + w(4)(2)(1)(2)#613
                                 + w(5)(2)(1)(2)#661 + w(6)(2)(1)(2)#709
                                 + w(7)(2)(1)(2)#757 + w(8)(2)(1)(2)#805
                                 + w(9)(2)(1)(2)#853 + w(10)(2)(1)(2)#901
                                 + w(11)(2)(1)(2)#949 + w(12)(2)(1)(2)#997  = 0
 WarehouseInventoryBalance(1,2,2): - I_warehouse(1)(1)(2)#7
                                 + I_warehouse(1)(2)(2)#15 - q(1)(1)(2)#535
                                 + w(3)(1)(2)(2)#549 + w(4)(1)(2)(2)#597
                                 + w(5)(1)(2)(2)#645 + w(6)(1)(2)(2)#693
                                 + w(7)(1)(2)(2)#741 + w(8)(1)(2)(2)#789
                                 + w(9)(1)(2)(2)#837 + w(10)(1)(2)(2)#885
                                 + w(11)(1)(2)(2)#933 + w(12)(1)(2)(2)#981  = 0
 WarehouseInventoryBalance(2,2,2): - I_warehouse(2)(1)(2)#31
                                 + I_warehouse(2)(2)(2)#39 - q(2)(1)(2)#538
                                 + w(3)(2)(2)(2)#573 + w(4)(2)(2)(2)#621
                                 + w(5)(2)(2)(2)#669 + w(6)(2)(2)(2)#717
                                 + w(7)(2)(2)(2)#765 + w(8)(2)(2)(2)#813
                                 + w(9)(2)(2)(2)#861 + w(10)(2)(2)(2)#909
                                 + w(11)(2)(2)(2)#957 + w(12)(2)(2)(2)#1005
                                  = 0
 WarehouseInventoryBalance(1,3,2): - I_warehouse(1)(2)(2)#15
                                 + I_warehouse(1)(3)(2)#23 - q(1)(1)(3)#536
                                 + w(3)(1)(3)(2)#557 + w(4)(1)(3)(2)#605
                                 + w(5)(1)(3)(2)#653 + w(6)(1)(3)(2)#701
                                 + w(7)(1)(3)(2)#749 + w(8)(1)(3)(2)#797
                                 + w(9)(1)(3)(2)#845 + w(10)(1)(3)(2)#893
                                 + w(11)(1)(3)(2)#941 + w(12)(1)(3)(2)#989  = 0
 WarehouseInventoryBalance(2,3,2): - I_warehouse(2)(2)(2)#39
                                 + I_warehouse(2)(3)(2)#47 - q(2)(1)(3)#539
                                 + w(3)(2)(3)(2)#581 + w(4)(2)(3)(2)#629
                                 + w(5)(2)(3)(2)#677 + w(6)(2)(3)(2)#725
                                 + w(7)(2)(3)(2)#773 + w(8)(2)(3)(2)#821
                                 + w(9)(2)(3)(2)#869 + w(10)(2)(3)(2)#917
                                 + w(11)(2)(3)(2)#965 + w(12)(2)(3)(2)#1013
                                  = 0
 WarehouseInventoryBalance(1,1,3): I_warehouse(1)(1)(3)#8 - q(1)(1)(1)#534
                                 + w(3)(1)(1)(3)#542 + w(4)(1)(1)(3)#590
                                 + w(5)(1)(1)(3)#638 + w(6)(1)(1)(3)#686
                                 + w(7)(1)(1)(3)#734 + w(8)(1)(1)(3)#782
                                 + w(9)(1)(1)(3)#830 + w(10)(1)(1)(3)#878
                                 + w(11)(1)(1)(3)#926 + w(12)(1)(1)(3)#974  = 0
 WarehouseInventoryBalance(2,1,3): I_warehouse(2)(1)(3)#32 - q(2)(1)(1)#537
                                 + w(3)(2)(1)(3)#566 + w(4)(2)(1)(3)#614
                                 + w(5)(2)(1)(3)#662 + w(6)(2)(1)(3)#710
                                 + w(7)(2)(1)(3)#758 + w(8)(2)(1)(3)#806
                                 + w(9)(2)(1)(3)#854 + w(10)(2)(1)(3)#902
                                 + w(11)(2)(1)(3)#950 + w(12)(2)(1)(3)#998  = 0
 WarehouseInventoryBalance(1,2,3): - I_warehouse(1)(1)(3)#8
                                 + I_warehouse(1)(2)(3)#16 - q(1)(1)(2)#535
                                 + w(3)(1)(2)(3)#550 + w(4)(1)(2)(3)#598
                                 + w(5)(1)(2)(3)#646 + w(6)(1)(2)(3)#694
                                 + w(7)(1)(2)(3)#742 + w(8)(1)(2)(3)#790
                                 + w(9)(1)(2)(3)#838 + w(10)(1)(2)(3)#886
                                 + w(11)(1)(2)(3)#934 + w(12)(1)(2)(3)#982  = 0
 WarehouseInventoryBalance(2,2,3): - I_warehouse(2)(1)(3)#32
                                 + I_warehouse(2)(2)(3)#40 - q(2)(1)(2)#538
                                 + w(3)(2)(2)(3)#574 + w(4)(2)(2)(3)#622
                                 + w(5)(2)(2)(3)#670 + w(6)(2)(2)(3)#718
                                 + w(7)(2)(2)(3)#766 + w(8)(2)(2)(3)#814
                                 + w(9)(2)(2)(3)#862 + w(10)(2)(2)(3)#910
                                 + w(11)(2)(2)(3)#958 + w(12)(2)(2)(3)#1006
                                  = 0
 WarehouseInventoryBalance(1,3,3): - I_warehouse(1)(2)(3)#16
                                 + I_warehouse(1)(3)(3)#24 - q(1)(1)(3)#536
                                 + w(3)(1)(3)(3)#558 + w(4)(1)(3)(3)#606
                                 + w(5)(1)(3)(3)#654 + w(6)(1)(3)(3)#702
                                 + w(7)(1)(3)(3)#750 + w(8)(1)(3)(3)#798
                                 + w(9)(1)(3)(3)#846 + w(10)(1)(3)(3)#894
                                 + w(11)(1)(3)(3)#942 + w(12)(1)(3)(3)#990  = 0
 WarehouseInventoryBalance(2,3,3): - I_warehouse(2)(2)(3)#40
                                 + I_warehouse(2)(3)(3)#48 - q(2)(1)(3)#539
                                 + w(3)(2)(3)(3)#582 + w(4)(2)(3)(3)#630
                                 + w(5)(2)(3)(3)#678 + w(6)(2)(3)(3)#726
                                 + w(7)(2)(3)(3)#774 + w(8)(2)(3)(3)#822
                                 + w(9)(2)(3)(3)#870 + w(10)(2)(3)(3)#918
                                 + w(11)(2)(3)(3)#966 + w(12)(2)(3)(3)#1014
                                  = 0
 WarehouseInventoryBalance(1,1,4): I_warehouse(1)(1)(4)#9 - q(1)(1)(1)#534
                                 + w(3)(1)(1)(4)#543 + w(4)(1)(1)(4)#591
                                 + w(5)(1)(1)(4)#639 + w(6)(1)(1)(4)#687
                                 + w(7)(1)(1)(4)#735 + w(8)(1)(1)(4)#783
                                 + w(9)(1)(1)(4)#831 + w(10)(1)(1)(4)#879
                                 + w(11)(1)(1)(4)#927 + w(12)(1)(1)(4)#975  = 0
 WarehouseInventoryBalance(2,1,4): I_warehouse(2)(1)(4)#33 - q(2)(1)(1)#537
                                 + w(3)(2)(1)(4)#567 + w(4)(2)(1)(4)#615
                                 + w(5)(2)(1)(4)#663 + w(6)(2)(1)(4)#711
                                 + w(7)(2)(1)(4)#759 + w(8)(2)(1)(4)#807
                                 + w(9)(2)(1)(4)#855 + w(10)(2)(1)(4)#903
                                 + w(11)(2)(1)(4)#951 + w(12)(2)(1)(4)#999  = 0
 WarehouseInventoryBalance(1,2,4): - I_warehouse(1)(1)(4)#9
                                 + I_warehouse(1)(2)(4)#17 - q(1)(1)(2)#535
                                 + w(3)(1)(2)(4)#551 + w(4)(1)(2)(4)#599
                                 + w(5)(1)(2)(4)#647 + w(6)(1)(2)(4)#695
                                 + w(7)(1)(2)(4)#743 + w(8)(1)(2)(4)#791
                                 + w(9)(1)(2)(4)#839 + w(10)(1)(2)(4)#887
                                 + w(11)(1)(2)(4)#935 + w(12)(1)(2)(4)#983  = 0
 WarehouseInventoryBalance(2,2,4): - I_warehouse(2)(1)(4)#33
                                 + I_warehouse(2)(2)(4)#41 - q(2)(1)(2)#538
                                 + w(3)(2)(2)(4)#575 + w(4)(2)(2)(4)#623
                                 + w(5)(2)(2)(4)#671 + w(6)(2)(2)(4)#719
                                 + w(7)(2)(2)(4)#767 + w(8)(2)(2)(4)#815
                                 + w(9)(2)(2)(4)#863 + w(10)(2)(2)(4)#911
                                 + w(11)(2)(2)(4)#959 + w(12)(2)(2)(4)#1007
                                  = 0
 WarehouseInventoryBalance(1,3,4): - I_warehouse(1)(2)(4)#17
                                 + I_warehouse(1)(3)(4)#25 - q(1)(1)(3)#536
                                 + w(3)(1)(3)(4)#559 + w(4)(1)(3)(4)#607
                                 + w(5)(1)(3)(4)#655 + w(6)(1)(3)(4)#703
                                 + w(7)(1)(3)(4)#751 + w(8)(1)(3)(4)#799
                                 + w(9)(1)(3)(4)#847 + w(10)(1)(3)(4)#895
                                 + w(11)(1)(3)(4)#943 + w(12)(1)(3)(4)#991  = 0
 WarehouseInventoryBalance(2,3,4): - I_warehouse(2)(2)(4)#41
                                 + I_warehouse(2)(3)(4)#49 - q(2)(1)(3)#539
                                 + w(3)(2)(3)(4)#583 + w(4)(2)(3)(4)#631
                                 + w(5)(2)(3)(4)#679 + w(6)(2)(3)(4)#727
                                 + w(7)(2)(3)(4)#775 + w(8)(2)(3)(4)#823
                                 + w(9)(2)(3)(4)#871 + w(10)(2)(3)(4)#919
                                 + w(11)(2)(3)(4)#967 + w(12)(2)(3)(4)#1015
                                  = 0
 WarehouseInventoryBalance(1,1,5): I_warehouse(1)(1)(5)#10 - q(1)(1)(1)#534
                                 + w(3)(1)(1)(5)#544 + w(4)(1)(1)(5)#592
                                 + w(5)(1)(1)(5)#640 + w(6)(1)(1)(5)#688
                                 + w(7)(1)(1)(5)#736 + w(8)(1)(1)(5)#784
                                 + w(9)(1)(1)(5)#832 + w(10)(1)(1)(5)#880
                                 + w(11)(1)(1)(5)#928 + w(12)(1)(1)(5)#976  = 0
 WarehouseInventoryBalance(2,1,5): I_warehouse(2)(1)(5)#34 - q(2)(1)(1)#537
                                 + w(3)(2)(1)(5)#568 + w(4)(2)(1)(5)#616
                                 + w(5)(2)(1)(5)#664 + w(6)(2)(1)(5)#712
                                 + w(7)(2)(1)(5)#760 + w(8)(2)(1)(5)#808
                                 + w(9)(2)(1)(5)#856 + w(10)(2)(1)(5)#904
                                 + w(11)(2)(1)(5)#952 + w(12)(2)(1)(5)#1000
                                  = 0
 WarehouseInventoryBalance(1,2,5): - I_warehouse(1)(1)(5)#10
                                 + I_warehouse(1)(2)(5)#18 - q(1)(1)(2)#535
                                 + w(3)(1)(2)(5)#552 + w(4)(1)(2)(5)#600
                                 + w(5)(1)(2)(5)#648 + w(6)(1)(2)(5)#696
                                 + w(7)(1)(2)(5)#744 + w(8)(1)(2)(5)#792
                                 + w(9)(1)(2)(5)#840 + w(10)(1)(2)(5)#888
                                 + w(11)(1)(2)(5)#936 + w(12)(1)(2)(5)#984  = 0
 WarehouseInventoryBalance(2,2,5): - I_warehouse(2)(1)(5)#34
                                 + I_warehouse(2)(2)(5)#42 - q(2)(1)(2)#538
                                 + w(3)(2)(2)(5)#576 + w(4)(2)(2)(5)#624
                                 + w(5)(2)(2)(5)#672 + w(6)(2)(2)(5)#720
                                 + w(7)(2)(2)(5)#768 + w(8)(2)(2)(5)#816
                                 + w(9)(2)(2)(5)#864 + w(10)(2)(2)(5)#912
                                 + w(11)(2)(2)(5)#960 + w(12)(2)(2)(5)#1008
                                  = 0
 WarehouseInventoryBalance(1,3,5): - I_warehouse(1)(2)(5)#18
                                 + I_warehouse(1)(3)(5)#26 - q(1)(1)(3)#536
                                 + w(3)(1)(3)(5)#560 + w(4)(1)(3)(5)#608
                                 + w(5)(1)(3)(5)#656 + w(6)(1)(3)(5)#704
                                 + w(7)(1)(3)(5)#752 + w(8)(1)(3)(5)#800
                                 + w(9)(1)(3)(5)#848 + w(10)(1)(3)(5)#896
                                 + w(11)(1)(3)(5)#944 + w(12)(1)(3)(5)#992  = 0
 WarehouseInventoryBalance(2,3,5): - I_warehouse(2)(2)(5)#42
                                 + I_warehouse(2)(3)(5)#50 - q(2)(1)(3)#539
                                 + w(3)(2)(3)(5)#584 + w(4)(2)(3)(5)#632
                                 + w(5)(2)(3)(5)#680 + w(6)(2)(3)(5)#728
                                 + w(7)(2)(3)(5)#776 + w(8)(2)(3)(5)#824
                                 + w(9)(2)(3)(5)#872 + w(10)(2)(3)(5)#920
                                 + w(11)(2)(3)(5)#968 + w(12)(2)(3)(5)#1016
                                  = 0
 WarehouseInventoryBalance(1,1,6): I_warehouse(1)(1)(6)#11 - q(1)(1)(1)#534
                                 + w(3)(1)(1)(6)#545 + w(4)(1)(1)(6)#593
                                 + w(5)(1)(1)(6)#641 + w(6)(1)(1)(6)#689
                                 + w(7)(1)(1)(6)#737 + w(8)(1)(1)(6)#785
                                 + w(9)(1)(1)(6)#833 + w(10)(1)(1)(6)#881
                                 + w(11)(1)(1)(6)#929 + w(12)(1)(1)(6)#977  = 0
 WarehouseInventoryBalance(2,1,6): I_warehouse(2)(1)(6)#35 - q(2)(1)(1)#537
                                 + w(3)(2)(1)(6)#569 + w(4)(2)(1)(6)#617
                                 + w(5)(2)(1)(6)#665 + w(6)(2)(1)(6)#713
                                 + w(7)(2)(1)(6)#761 + w(8)(2)(1)(6)#809
                                 + w(9)(2)(1)(6)#857 + w(10)(2)(1)(6)#905
                                 + w(11)(2)(1)(6)#953 + w(12)(2)(1)(6)#1001
                                  = 0
 WarehouseInventoryBalance(1,2,6): - I_warehouse(1)(1)(6)#11
                                 + I_warehouse(1)(2)(6)#19 - q(1)(1)(2)#535
                                 + w(3)(1)(2)(6)#553 + w(4)(1)(2)(6)#601
                                 + w(5)(1)(2)(6)#649 + w(6)(1)(2)(6)#697
                                 + w(7)(1)(2)(6)#745 + w(8)(1)(2)(6)#793
                                 + w(9)(1)(2)(6)#841 + w(10)(1)(2)(6)#889
                                 + w(11)(1)(2)(6)#937 + w(12)(1)(2)(6)#985  = 0
 WarehouseInventoryBalance(2,2,6): - I_warehouse(2)(1)(6)#35
                                 + I_warehouse(2)(2)(6)#43 - q(2)(1)(2)#538
                                 + w(3)(2)(2)(6)#577 + w(4)(2)(2)(6)#625
                                 + w(5)(2)(2)(6)#673 + w(6)(2)(2)(6)#721
                                 + w(7)(2)(2)(6)#769 + w(8)(2)(2)(6)#817
                                 + w(9)(2)(2)(6)#865 + w(10)(2)(2)(6)#913
                                 + w(11)(2)(2)(6)#961 + w(12)(2)(2)(6)#1009
                                  = 0
 WarehouseInventoryBalance(1,3,6): - I_warehouse(1)(2)(6)#19
                                 + I_warehouse(1)(3)(6)#27 - q(1)(1)(3)#536
                                 + w(3)(1)(3)(6)#561 + w(4)(1)(3)(6)#609
                                 + w(5)(1)(3)(6)#657 + w(6)(1)(3)(6)#705
                                 + w(7)(1)(3)(6)#753 + w(8)(1)(3)(6)#801
                                 + w(9)(1)(3)(6)#849 + w(10)(1)(3)(6)#897
                                 + w(11)(1)(3)(6)#945 + w(12)(1)(3)(6)#993  = 0
 WarehouseInventoryBalance(2,3,6): - I_warehouse(2)(2)(6)#43
                                 + I_warehouse(2)(3)(6)#51 - q(2)(1)(3)#539
                                 + w(3)(2)(3)(6)#585 + w(4)(2)(3)(6)#633
                                 + w(5)(2)(3)(6)#681 + w(6)(2)(3)(6)#729
                                 + w(7)(2)(3)(6)#777 + w(8)(2)(3)(6)#825
                                 + w(9)(2)(3)(6)#873 + w(10)(2)(3)(6)#921
                                 + w(11)(2)(3)(6)#969 + w(12)(2)(3)(6)#1017
                                  = 0
 WarehouseInventoryBalance(1,1,7): I_warehouse(1)(1)(7)#12 - q(1)(1)(1)#534
                                 + w(3)(1)(1)(7)#546 + w(4)(1)(1)(7)#594
                                 + w(5)(1)(1)(7)#642 + w(6)(1)(1)(7)#690
                                 + w(7)(1)(1)(7)#738 + w(8)(1)(1)(7)#786
                                 + w(9)(1)(1)(7)#834 + w(10)(1)(1)(7)#882
                                 + w(11)(1)(1)(7)#930 + w(12)(1)(1)(7)#978  = 0
 WarehouseInventoryBalance(2,1,7): I_warehouse(2)(1)(7)#36 - q(2)(1)(1)#537
                                 + w(3)(2)(1)(7)#570 + w(4)(2)(1)(7)#618
                                 + w(5)(2)(1)(7)#666 + w(6)(2)(1)(7)#714
                                 + w(7)(2)(1)(7)#762 + w(8)(2)(1)(7)#810
                                 + w(9)(2)(1)(7)#858 + w(10)(2)(1)(7)#906
                                 + w(11)(2)(1)(7)#954 + w(12)(2)(1)(7)#1002
                                  = 0
 WarehouseInventoryBalance(1,2,7): - I_warehouse(1)(1)(7)#12
                                 + I_warehouse(1)(2)(7)#20 - q(1)(1)(2)#535
                                 + w(3)(1)(2)(7)#554 + w(4)(1)(2)(7)#602
                                 + w(5)(1)(2)(7)#650 + w(6)(1)(2)(7)#698
                                 + w(7)(1)(2)(7)#746 + w(8)(1)(2)(7)#794
                                 + w(9)(1)(2)(7)#842 + w(10)(1)(2)(7)#890
                                 + w(11)(1)(2)(7)#938 + w(12)(1)(2)(7)#986  = 0
 WarehouseInventoryBalance(2,2,7): - I_warehouse(2)(1)(7)#36
                                 + I_warehouse(2)(2)(7)#44 - q(2)(1)(2)#538
                                 + w(3)(2)(2)(7)#578 + w(4)(2)(2)(7)#626
                                 + w(5)(2)(2)(7)#674 + w(6)(2)(2)(7)#722
                                 + w(7)(2)(2)(7)#770 + w(8)(2)(2)(7)#818
                                 + w(9)(2)(2)(7)#866 + w(10)(2)(2)(7)#914
                                 + w(11)(2)(2)(7)#962 + w(12)(2)(2)(7)#1010
                                  = 0
 WarehouseInventoryBalance(1,3,7): - I_warehouse(1)(2)(7)#20
                                 + I_warehouse(1)(3)(7)#28 - q(1)(1)(3)#536
                                 + w(3)(1)(3)(7)#562 + w(4)(1)(3)(7)#610
                                 + w(5)(1)(3)(7)#658 + w(6)(1)(3)(7)#706
                                 + w(7)(1)(3)(7)#754 + w(8)(1)(3)(7)#802
                                 + w(9)(1)(3)(7)#850 + w(10)(1)(3)(7)#898
                                 + w(11)(1)(3)(7)#946 + w(12)(1)(3)(7)#994  = 0
 WarehouseInventoryBalance(2,3,7): - I_warehouse(2)(2)(7)#44
                                 + I_warehouse(2)(3)(7)#52 - q(2)(1)(3)#539
                                 + w(3)(2)(3)(7)#586 + w(4)(2)(3)(7)#634
                                 + w(5)(2)(3)(7)#682 + w(6)(2)(3)(7)#730
                                 + w(7)(2)(3)(7)#778 + w(8)(2)(3)(7)#826
                                 + w(9)(2)(3)(7)#874 + w(10)(2)(3)(7)#922
                                 + w(11)(2)(3)(7)#970 + w(12)(2)(3)(7)#1018
                                  = 0
 WarehouseInventoryBalance(1,1,8): I_warehouse(1)(1)(8)#13 - q(1)(1)(1)#534
                                 + w(3)(1)(1)(8)#547 + w(4)(1)(1)(8)#595
                                 + w(5)(1)(1)(8)#643 + w(6)(1)(1)(8)#691
                                 + w(7)(1)(1)(8)#739 + w(8)(1)(1)(8)#787
                                 + w(9)(1)(1)(8)#835 + w(10)(1)(1)(8)#883
                                 + w(11)(1)(1)(8)#931 + w(12)(1)(1)(8)#979  = 0
 WarehouseInventoryBalance(2,1,8): I_warehouse(2)(1)(8)#37 - q(2)(1)(1)#537
                                 + w(3)(2)(1)(8)#571 + w(4)(2)(1)(8)#619
                                 + w(5)(2)(1)(8)#667 + w(6)(2)(1)(8)#715
                                 + w(7)(2)(1)(8)#763 + w(8)(2)(1)(8)#811
                                 + w(9)(2)(1)(8)#859 + w(10)(2)(1)(8)#907
                                 + w(11)(2)(1)(8)#955 + w(12)(2)(1)(8)#1003
                                  = 0
 WarehouseInventoryBalance(1,2,8): - I_warehouse(1)(1)(8)#13
                                 + I_warehouse(1)(2)(8)#21 - q(1)(1)(2)#535
                                 + w(3)(1)(2)(8)#555 + w(4)(1)(2)(8)#603
                                 + w(5)(1)(2)(8)#651 + w(6)(1)(2)(8)#699
                                 + w(7)(1)(2)(8)#747 + w(8)(1)(2)(8)#795
                                 + w(9)(1)(2)(8)#843 + w(10)(1)(2)(8)#891
                                 + w(11)(1)(2)(8)#939 + w(12)(1)(2)(8)#987  = 0
 WarehouseInventoryBalance(2,2,8): - I_warehouse(2)(1)(8)#37
                                 + I_warehouse(2)(2)(8)#45 - q(2)(1)(2)#538
                                 + w(3)(2)(2)(8)#579 + w(4)(2)(2)(8)#627
                                 + w(5)(2)(2)(8)#675 + w(6)(2)(2)(8)#723
                                 + w(7)(2)(2)(8)#771 + w(8)(2)(2)(8)#819
                                 + w(9)(2)(2)(8)#867 + w(10)(2)(2)(8)#915
                                 + w(11)(2)(2)(8)#963 + w(12)(2)(2)(8)#1011
                                  = 0
 WarehouseInventoryBalance(1,3,8): - I_warehouse(1)(2)(8)#21
                                 + I_warehouse(1)(3)(8)#29 - q(1)(1)(3)#536
                                 + w(3)(1)(3)(8)#563 + w(4)(1)(3)(8)#611
                                 + w(5)(1)(3)(8)#659 + w(6)(1)(3)(8)#707
                                 + w(7)(1)(3)(8)#755 + w(8)(1)(3)(8)#803
                                 + w(9)(1)(3)(8)#851 + w(10)(1)(3)(8)#899
                                 + w(11)(1)(3)(8)#947 + w(12)(1)(3)(8)#995  = 0
 WarehouseInventoryBalance(2,3,8): - I_warehouse(2)(2)(8)#45
                                 + I_warehouse(2)(3)(8)#53 - q(2)(1)(3)#539
                                 + w(3)(2)(3)(8)#587 + w(4)(2)(3)(8)#635
                                 + w(5)(2)(3)(8)#683 + w(6)(2)(3)(8)#731
                                 + w(7)(2)(3)(8)#779 + w(8)(2)(3)(8)#827
                                 + w(9)(2)(3)(8)#875 + w(10)(2)(3)(8)#923
                                 + w(11)(2)(3)(8)#971 + w(12)(2)(3)(8)#1019
                                  = 0
 CustomerInventoryBalance(3,1,1): I_customer(3)(1)(1)#54
                                 - b_customer(3)(1)(1)#294 - w(3)(1)(1)(1)#540
                                 - w(3)(2)(1)(1)#564  = 20
 CustomerInventoryBalance(4,1,1): I_customer(4)(1)(1)#78
                                 - b_customer(4)(1)(1)#318 - w(4)(1)(1)(1)#588
                                 - w(4)(2)(1)(1)#612  = -1
 CustomerInventoryBalance(5,1,1): I_customer(5)(1)(1)#102
                                 - b_customer(5)(1)(1)#342 - w(5)(1)(1)(1)#636
                                 - w(5)(2)(1)(1)#660  = 1
 CustomerInventoryBalance(6,1,1): I_customer(6)(1)(1)#126
                                 - b_customer(6)(1)(1)#366 - w(6)(1)(1)(1)#684
                                 - w(6)(2)(1)(1)#708  = 3
 CustomerInventoryBalance(7,1,1): I_customer(7)(1)(1)#150
                                 - b_customer(7)(1)(1)#390 - w(7)(1)(1)(1)#732
                                 - w(7)(2)(1)(1)#756  = 6
 CustomerInventoryBalance(8,1,1): I_customer(8)(1)(1)#174
                                 - b_customer(8)(1)(1)#414 - w(8)(1)(1)(1)#780
                                 - w(8)(2)(1)(1)#804  = 3
 CustomerInventoryBalance(9,1,1): I_customer(9)(1)(1)#198
                                 - b_customer(9)(1)(1)#438 - w(9)(1)(1)(1)#828
                                 - w(9)(2)(1)(1)#852  = 13
 CustomerInventoryBalance(10,1,1): I_customer(10)(1)(1)#222
                                 - b_customer(10)(1)(1)#462
                                 - w(10)(1)(1)(1)#876 - w(10)(2)(1)(1)#900  = 2
 CustomerInventoryBalance(11,1,1): I_customer(11)(1)(1)#246
                                 - b_customer(11)(1)(1)#486
                                 - w(11)(1)(1)(1)#924 - w(11)(2)(1)(1)#948  = 3
 CustomerInventoryBalance(12,1,1): I_customer(12)(1)(1)#270
                                 - b_customer(12)(1)(1)#510
                                 - w(12)(1)(1)(1)#972 - w(12)(2)(1)(1)#996
                                  = 15
 CustomerInventoryBalance(3,2,1): - I_customer(3)(1)(1)#54
                                 + I_customer(3)(2)(1)#62
                                 - b_customer(3)(2)(1)#302 - w(3)(1)(2)(1)#548
                                 - w(3)(2)(2)(1)#572  = -17
 CustomerInventoryBalance(4,2,1): - I_customer(4)(1)(1)#78
                                 + I_customer(4)(2)(1)#86
                                 - b_customer(4)(2)(1)#326 - w(4)(1)(2)(1)#596
                                 - w(4)(2)(2)(1)#620  = -6
 CustomerInventoryBalance(5,2,1): - I_customer(5)(1)(1)#102
                                 + I_customer(5)(2)(1)#110
                                 - b_customer(5)(2)(1)#350 - w(5)(1)(2)(1)#644
                                 - w(5)(2)(2)(1)#668  = -6
 CustomerInventoryBalance(6,2,1): - I_customer(6)(1)(1)#126
                                 + I_customer(6)(2)(1)#134
                                 - b_customer(6)(2)(1)#374 - w(6)(1)(2)(1)#692
                                 - w(6)(2)(2)(1)#716  = -17
 CustomerInventoryBalance(7,2,1): - I_customer(7)(1)(1)#150
                                 + I_customer(7)(2)(1)#158
                                 - b_customer(7)(2)(1)#398 - w(7)(1)(2)(1)#740
                                 - w(7)(2)(2)(1)#764  = -8
 CustomerInventoryBalance(8,2,1): - I_customer(8)(1)(1)#174
                                 + I_customer(8)(2)(1)#182
                                 - b_customer(8)(2)(1)#422 - w(8)(1)(2)(1)#788
                                 - w(8)(2)(2)(1)#812  = -20
 CustomerInventoryBalance(9,2,1): - I_customer(9)(1)(1)#198
                                 + I_customer(9)(2)(1)#206
                                 - b_customer(9)(2)(1)#446 - w(9)(1)(2)(1)#836
                                 - w(9)(2)(2)(1)#860  = -15
 CustomerInventoryBalance(10,2,1): - I_customer(10)(1)(1)#222
                                 + I_customer(10)(2)(1)#230
                                 - b_customer(10)(2)(1)#470
                                 - w(10)(1)(2)(1)#884 - w(10)(2)(2)(1)#908
                                  = -14
 CustomerInventoryBalance(11,2,1): - I_customer(11)(1)(1)#246
                                 + I_customer(11)(2)(1)#254
                                 - b_customer(11)(2)(1)#494
                                 - w(11)(1)(2)(1)#932 - w(11)(2)(2)(1)#956
                                  = -13
 CustomerInventoryBalance(12,2,1): - I_customer(12)(1)(1)#270
                                 + I_customer(12)(2)(1)#278
                                 - b_customer(12)(2)(1)#518
                                 - w(12)(1)(2)(1)#980 - w(12)(2)(2)(1)#1004
                                  = -14
 CustomerInventoryBalance(3,3,1): - I_customer(3)(2)(1)#62
                                 + I_customer(3)(3)(1)#70
                                 - b_customer(3)(3)(1)#310 - w(3)(1)(3)(1)#556
                                 - w(3)(2)(3)(1)#580  = -22
 CustomerInventoryBalance(4,3,1): - I_customer(4)(2)(1)#86
                                 + I_customer(4)(3)(1)#94
                                 - b_customer(4)(3)(1)#334 - w(4)(1)(3)(1)#604
                                 - w(4)(2)(3)(1)#628  = -8
 CustomerInventoryBalance(5,3,1): - I_customer(5)(2)(1)#110
                                 + I_customer(5)(3)(1)#118
                                 - b_customer(5)(3)(1)#358 - w(5)(1)(3)(1)#652
                                 - w(5)(2)(3)(1)#676  = -6
 CustomerInventoryBalance(6,3,1): - I_customer(6)(2)(1)#134
                                 + I_customer(6)(3)(1)#142
                                 - b_customer(6)(3)(1)#382 - w(6)(1)(3)(1)#700
                                 - w(6)(2)(3)(1)#724  = -17
 CustomerInventoryBalance(7,3,1): - I_customer(7)(2)(1)#158
                                 + I_customer(7)(3)(1)#166
                                 - b_customer(7)(3)(1)#406 - w(7)(1)(3)(1)#748
                                 - w(7)(2)(3)(1)#772  = -6
 CustomerInventoryBalance(8,3,1): - I_customer(8)(2)(1)#182
                                 + I_customer(8)(3)(1)#190
                                 - b_customer(8)(3)(1)#430 - w(8)(1)(3)(1)#796
                                 - w(8)(2)(3)(1)#820  = -19
 CustomerInventoryBalance(9,3,1): - I_customer(9)(2)(1)#206
                                 + I_customer(9)(3)(1)#214
                                 - b_customer(9)(3)(1)#454 - w(9)(1)(3)(1)#844
                                 - w(9)(2)(3)(1)#868  = -17
 CustomerInventoryBalance(10,3,1): - I_customer(10)(2)(1)#230
                                 + I_customer(10)(3)(1)#238
                                 - b_customer(10)(3)(1)#478
                                 - w(10)(1)(3)(1)#892 - w(10)(2)(3)(1)#916
                                  = -16
 CustomerInventoryBalance(11,3,1): - I_customer(11)(2)(1)#254
                                 + I_customer(11)(3)(1)#262
                                 - b_customer(11)(3)(1)#502
                                 - w(11)(1)(3)(1)#940 - w(11)(2)(3)(1)#964
                                  = -14
 CustomerInventoryBalance(12,3,1): - I_customer(12)(2)(1)#278
                                 + I_customer(12)(3)(1)#286
                                 - b_customer(12)(3)(1)#526
                                 - w(12)(1)(3)(1)#988 - w(12)(2)(3)(1)#1012
                                  = -16
 CustomerInventoryBalance(3,1,2): I_customer(3)(1)(2)#55
                                 - b_customer(3)(1)(2)#295 - w(3)(1)(1)(2)#541
                                 - w(3)(2)(1)(2)#565  = 16
 CustomerInventoryBalance(4,1,2): I_customer(4)(1)(2)#79
                                 - b_customer(4)(1)(2)#319 - w(4)(1)(1)(2)#589
                                 - w(4)(2)(1)(2)#613  = 1
 CustomerInventoryBalance(5,1,2): I_customer(5)(1)(2)#103
                                 - b_customer(5)(1)(2)#343 - w(5)(1)(1)(2)#637
                                 - w(5)(2)(1)(2)#661  = 1
 CustomerInventoryBalance(6,1,2): I_customer(6)(1)(2)#127
                                 - b_customer(6)(1)(2)#367 - w(6)(1)(1)(2)#685
                                 - w(6)(2)(1)(2)#709  = -1
 CustomerInventoryBalance(7,1,2): I_customer(7)(1)(2)#151
                                 - b_customer(7)(1)(2)#391 - w(7)(1)(1)(2)#733
                                 - w(7)(2)(1)(2)#757  = 8
 CustomerInventoryBalance(8,1,2): I_customer(8)(1)(2)#175
                                 - b_customer(8)(1)(2)#415 - w(8)(1)(1)(2)#781
                                 - w(8)(2)(1)(2)#805  = 0
 CustomerInventoryBalance(9,1,2): I_customer(9)(1)(2)#199
                                 - b_customer(9)(1)(2)#439 - w(9)(1)(1)(2)#829
                                 - w(9)(2)(1)(2)#853  = 19
 CustomerInventoryBalance(10,1,2): I_customer(10)(1)(2)#223
                                 - b_customer(10)(1)(2)#463
                                 - w(10)(1)(1)(2)#877 - w(10)(2)(1)(2)#901  = 2
 CustomerInventoryBalance(11,1,2): I_customer(11)(1)(2)#247
                                 - b_customer(11)(1)(2)#487
                                 - w(11)(1)(1)(2)#925 - w(11)(2)(1)(2)#949
                                  = -3
 CustomerInventoryBalance(12,1,2): I_customer(12)(1)(2)#271
                                 - b_customer(12)(1)(2)#511
                                 - w(12)(1)(1)(2)#973 - w(12)(2)(1)(2)#997
                                  = 17
 CustomerInventoryBalance(3,2,2): - I_customer(3)(1)(2)#55
                                 + I_customer(3)(2)(2)#63
                                 - b_customer(3)(2)(2)#303 - w(3)(1)(2)(2)#549
                                 - w(3)(2)(2)(2)#573  = -20
 CustomerInventoryBalance(4,2,2): - I_customer(4)(1)(2)#79
                                 + I_customer(4)(2)(2)#87
                                 - b_customer(4)(2)(2)#327 - w(4)(1)(2)(2)#597
                                 - w(4)(2)(2)(2)#621  = -8
 CustomerInventoryBalance(5,2,2): - I_customer(5)(1)(2)#103
                                 + I_customer(5)(2)(2)#111
                                 - b_customer(5)(2)(2)#351 - w(5)(1)(2)(2)#645
                                 - w(5)(2)(2)(2)#669  = -7
 CustomerInventoryBalance(6,2,2): - I_customer(6)(1)(2)#127
                                 + I_customer(6)(2)(2)#135
                                 - b_customer(6)(2)(2)#375 - w(6)(1)(2)(2)#693
                                 - w(6)(2)(2)(2)#717  = -16
 CustomerInventoryBalance(7,2,2): - I_customer(7)(1)(2)#151
                                 + I_customer(7)(2)(2)#159
                                 - b_customer(7)(2)(2)#399 - w(7)(1)(2)(2)#741
                                 - w(7)(2)(2)(2)#765  = -7
 CustomerInventoryBalance(8,2,2): - I_customer(8)(1)(2)#175
                                 + I_customer(8)(2)(2)#183
                                 - b_customer(8)(2)(2)#423 - w(8)(1)(2)(2)#789
                                 - w(8)(2)(2)(2)#813  = -17
 CustomerInventoryBalance(9,2,2): - I_customer(9)(1)(2)#199
                                 + I_customer(9)(2)(2)#207
                                 - b_customer(9)(2)(2)#447 - w(9)(1)(2)(2)#837
                                 - w(9)(2)(2)(2)#861  = -14
 CustomerInventoryBalance(10,2,2): - I_customer(10)(1)(2)#223
                                 + I_customer(10)(2)(2)#231
                                 - b_customer(10)(2)(2)#471
                                 - w(10)(1)(2)(2)#885 - w(10)(2)(2)(2)#909
                                  = -18
 CustomerInventoryBalance(11,2,2): - I_customer(11)(1)(2)#247
                                 + I_customer(11)(2)(2)#255
                                 - b_customer(11)(2)(2)#495
                                 - w(11)(1)(2)(2)#933 - w(11)(2)(2)(2)#957
                                  = -16
 CustomerInventoryBalance(12,2,2): - I_customer(12)(1)(2)#271
                                 + I_customer(12)(2)(2)#279
                                 - b_customer(12)(2)(2)#519
                                 - w(12)(1)(2)(2)#981 - w(12)(2)(2)(2)#1005
                                  = -14
 CustomerInventoryBalance(3,3,2): - I_customer(3)(2)(2)#63
                                 + I_customer(3)(3)(2)#71
                                 - b_customer(3)(3)(2)#311 - w(3)(1)(3)(2)#557
                                 - w(3)(2)(3)(2)#581  = -16
 CustomerInventoryBalance(4,3,2): - I_customer(4)(2)(2)#87
                                 + I_customer(4)(3)(2)#95
                                 - b_customer(4)(3)(2)#335 - w(4)(1)(3)(2)#605
                                 - w(4)(2)(3)(2)#629  = -7
 CustomerInventoryBalance(5,3,2): - I_customer(5)(2)(2)#111
                                 + I_customer(5)(3)(2)#119
                                 - b_customer(5)(3)(2)#359 - w(5)(1)(3)(2)#653
                                 - w(5)(2)(3)(2)#677  = -7
 CustomerInventoryBalance(6,3,2): - I_customer(6)(2)(2)#135
                                 + I_customer(6)(3)(2)#143
                                 - b_customer(6)(3)(2)#383 - w(6)(1)(3)(2)#701
                                 - w(6)(2)(3)(2)#725  = -17
 CustomerInventoryBalance(7,3,2): - I_customer(7)(2)(2)#159
                                 + I_customer(7)(3)(2)#167
                                 - b_customer(7)(3)(2)#407 - w(7)(1)(3)(2)#749
                                 - w(7)(2)(3)(2)#773  = -6
 CustomerInventoryBalance(8,3,2): - I_customer(8)(2)(2)#183
                                 + I_customer(8)(3)(2)#191
                                 - b_customer(8)(3)(2)#431 - w(8)(1)(3)(2)#797
                                 - w(8)(2)(3)(2)#821  = -19
 CustomerInventoryBalance(9,3,2): - I_customer(9)(2)(2)#207
                                 + I_customer(9)(3)(2)#215
                                 - b_customer(9)(3)(2)#455 - w(9)(1)(3)(2)#845
                                 - w(9)(2)(3)(2)#869  = -13
 CustomerInventoryBalance(10,3,2): - I_customer(10)(2)(2)#231
                                 + I_customer(10)(3)(2)#239
                                 - b_customer(10)(3)(2)#479
                                 - w(10)(1)(3)(2)#893 - w(10)(2)(3)(2)#917
                                  = -17
 CustomerInventoryBalance(11,3,2): - I_customer(11)(2)(2)#255
                                 + I_customer(11)(3)(2)#263
                                 - b_customer(11)(3)(2)#503
                                 - w(11)(1)(3)(2)#941 - w(11)(2)(3)(2)#965
                                  = -18
 CustomerInventoryBalance(12,3,2): - I_customer(12)(2)(2)#279
                                 + I_customer(12)(3)(2)#287
                                 - b_customer(12)(3)(2)#527
                                 - w(12)(1)(3)(2)#989 - w(12)(2)(3)(2)#1013
                                  = -16
 CustomerInventoryBalance(3,1,3): I_customer(3)(1)(3)#56
                                 - b_customer(3)(1)(3)#296 - w(3)(1)(1)(3)#542
                                 - w(3)(2)(1)(3)#566  = 17
 CustomerInventoryBalance(4,1,3): I_customer(4)(1)(3)#80
                                 - b_customer(4)(1)(3)#320 - w(4)(1)(1)(3)#590
                                 - w(4)(2)(1)(3)#614  = -1
 CustomerInventoryBalance(5,1,3): I_customer(5)(1)(3)#104
                                 - b_customer(5)(1)(3)#344 - w(5)(1)(1)(3)#638
                                 - w(5)(2)(1)(3)#662  = 0
 CustomerInventoryBalance(6,1,3): I_customer(6)(1)(3)#128
                                 - b_customer(6)(1)(3)#368 - w(6)(1)(1)(3)#686
                                 - w(6)(2)(1)(3)#710  = 0
 CustomerInventoryBalance(7,1,3): I_customer(7)(1)(3)#152
                                 - b_customer(7)(1)(3)#392 - w(7)(1)(1)(3)#734
                                 - w(7)(2)(1)(3)#758  = 6
 CustomerInventoryBalance(8,1,3): I_customer(8)(1)(3)#176
                                 - b_customer(8)(1)(3)#416 - w(8)(1)(1)(3)#782
                                 - w(8)(2)(1)(3)#806  = -3
 CustomerInventoryBalance(9,1,3): I_customer(9)(1)(3)#200
                                 - b_customer(9)(1)(3)#440 - w(9)(1)(1)(3)#830
                                 - w(9)(2)(1)(3)#854  = 18
 CustomerInventoryBalance(10,1,3): I_customer(10)(1)(3)#224
                                 - b_customer(10)(1)(3)#464
                                 - w(10)(1)(1)(3)#878 - w(10)(2)(1)(3)#902
                                  = -2
 CustomerInventoryBalance(11,1,3): I_customer(11)(1)(3)#248
                                 - b_customer(11)(1)(3)#488
                                 - w(11)(1)(1)(3)#926 - w(11)(2)(1)(3)#950  = 1
 CustomerInventoryBalance(12,1,3): I_customer(12)(1)(3)#272
                                 - b_customer(12)(1)(3)#512
                                 - w(12)(1)(1)(3)#974 - w(12)(2)(1)(3)#998
                                  = 15
 CustomerInventoryBalance(3,2,3): - I_customer(3)(1)(3)#56
                                 + I_customer(3)(2)(3)#64
                                 - b_customer(3)(2)(3)#304 - w(3)(1)(2)(3)#550
                                 - w(3)(2)(2)(3)#574  = -20
 CustomerInventoryBalance(4,2,3): - I_customer(4)(1)(3)#80
                                 + I_customer(4)(2)(3)#88
                                 - b_customer(4)(2)(3)#328 - w(4)(1)(2)(3)#598
                                 - w(4)(2)(2)(3)#622  = -8
 CustomerInventoryBalance(5,2,3): - I_customer(5)(1)(3)#104
                                 + I_customer(5)(2)(3)#112
                                 - b_customer(5)(2)(3)#352 - w(5)(1)(2)(3)#646
                                 - w(5)(2)(2)(3)#670  = -7
 CustomerInventoryBalance(6,2,3): - I_customer(6)(1)(3)#128
                                 + I_customer(6)(2)(3)#136
                                 - b_customer(6)(2)(3)#376 - w(6)(1)(2)(3)#694
                                 - w(6)(2)(2)(3)#718  = -15
 CustomerInventoryBalance(7,2,3): - I_customer(7)(1)(3)#152
                                 + I_customer(7)(2)(3)#160
                                 - b_customer(7)(2)(3)#400 - w(7)(1)(2)(3)#742
                                 - w(7)(2)(2)(3)#766  = -7
 CustomerInventoryBalance(8,2,3): - I_customer(8)(1)(3)#176
                                 + I_customer(8)(2)(3)#184
                                 - b_customer(8)(2)(3)#424 - w(8)(1)(2)(3)#790
                                 - w(8)(2)(2)(3)#814  = -20
 CustomerInventoryBalance(9,2,3): - I_customer(9)(1)(3)#200
                                 + I_customer(9)(2)(3)#208
                                 - b_customer(9)(2)(3)#448 - w(9)(1)(2)(3)#838
                                 - w(9)(2)(2)(3)#862  = -16
 CustomerInventoryBalance(10,2,3): - I_customer(10)(1)(3)#224
                                 + I_customer(10)(2)(3)#232
                                 - b_customer(10)(2)(3)#472
                                 - w(10)(1)(2)(3)#886 - w(10)(2)(2)(3)#910
                                  = -13
 CustomerInventoryBalance(11,2,3): - I_customer(11)(1)(3)#248
                                 + I_customer(11)(2)(3)#256
                                 - b_customer(11)(2)(3)#496
                                 - w(11)(1)(2)(3)#934 - w(11)(2)(2)(3)#958
                                  = -18
 CustomerInventoryBalance(12,2,3): - I_customer(12)(1)(3)#272
                                 + I_customer(12)(2)(3)#280
                                 - b_customer(12)(2)(3)#520
                                 - w(12)(1)(2)(3)#982 - w(12)(2)(2)(3)#1006
                                  = -13
 CustomerInventoryBalance(3,3,3): - I_customer(3)(2)(3)#64
                                 + I_customer(3)(3)(3)#72
                                 - b_customer(3)(3)(3)#312 - w(3)(1)(3)(3)#558
                                 - w(3)(2)(3)(3)#582  = -20
 CustomerInventoryBalance(4,3,3): - I_customer(4)(2)(3)#88
                                 + I_customer(4)(3)(3)#96
                                 - b_customer(4)(3)(3)#336 - w(4)(1)(3)(3)#606
                                 - w(4)(2)(3)(3)#630  = -7
 CustomerInventoryBalance(5,3,3): - I_customer(5)(2)(3)#112
                                 + I_customer(5)(3)(3)#120
                                 - b_customer(5)(3)(3)#360 - w(5)(1)(3)(3)#654
                                 - w(5)(2)(3)(3)#678  = -5
 CustomerInventoryBalance(6,3,3): - I_customer(6)(2)(3)#136
                                 + I_customer(6)(3)(3)#144
                                 - b_customer(6)(3)(3)#384 - w(6)(1)(3)(3)#702
                                 - w(6)(2)(3)(3)#726  = -16
 CustomerInventoryBalance(7,3,3): - I_customer(7)(2)(3)#160
                                 + I_customer(7)(3)(3)#168
                                 - b_customer(7)(3)(3)#408 - w(7)(1)(3)(3)#750
                                 - w(7)(2)(3)(3)#774  = -7
 CustomerInventoryBalance(8,3,3): - I_customer(8)(2)(3)#184
                                 + I_customer(8)(3)(3)#192
                                 - b_customer(8)(3)(3)#432 - w(8)(1)(3)(3)#798
                                 - w(8)(2)(3)(3)#822  = -21
 CustomerInventoryBalance(9,3,3): - I_customer(9)(2)(3)#208
                                 + I_customer(9)(3)(3)#216
                                 - b_customer(9)(3)(3)#456 - w(9)(1)(3)(3)#846
                                 - w(9)(2)(3)(3)#870  = -19
 CustomerInventoryBalance(10,3,3): - I_customer(10)(2)(3)#232
                                 + I_customer(10)(3)(3)#240
                                 - b_customer(10)(3)(3)#480
                                 - w(10)(1)(3)(3)#894 - w(10)(2)(3)(3)#918
                                  = -19
 CustomerInventoryBalance(11,3,3): - I_customer(11)(2)(3)#256
                                 + I_customer(11)(3)(3)#264
                                 - b_customer(11)(3)(3)#504
                                 - w(11)(1)(3)(3)#942 - w(11)(2)(3)(3)#966
                                  = -17
 CustomerInventoryBalance(12,3,3): - I_customer(12)(2)(3)#280
                                 + I_customer(12)(3)(3)#288
                                 - b_customer(12)(3)(3)#528
                                 - w(12)(1)(3)(3)#990 - w(12)(2)(3)(3)#1014
                                  = -13
 CustomerInventoryBalance(3,1,4): I_customer(3)(1)(4)#57
                                 - b_customer(3)(1)(4)#297 - w(3)(1)(1)(4)#543
                                 - w(3)(2)(1)(4)#567  = 17
 CustomerInventoryBalance(4,1,4): I_customer(4)(1)(4)#81
                                 - b_customer(4)(1)(4)#321 - w(4)(1)(1)(4)#591
                                 - w(4)(2)(1)(4)#615  = -1
 CustomerInventoryBalance(5,1,4): I_customer(5)(1)(4)#105
                                 - b_customer(5)(1)(4)#345 - w(5)(1)(1)(4)#639
                                 - w(5)(2)(1)(4)#663  = -1
 CustomerInventoryBalance(6,1,4): I_customer(6)(1)(4)#129
                                 - b_customer(6)(1)(4)#369 - w(6)(1)(1)(4)#687
                                 - w(6)(2)(1)(4)#711  = -1
 CustomerInventoryBalance(7,1,4): I_customer(7)(1)(4)#153
                                 - b_customer(7)(1)(4)#393 - w(7)(1)(1)(4)#735
                                 - w(7)(2)(1)(4)#759  = 6
 CustomerInventoryBalance(8,1,4): I_customer(8)(1)(4)#177
                                 - b_customer(8)(1)(4)#417 - w(8)(1)(1)(4)#783
                                 - w(8)(2)(1)(4)#807  = 2
 CustomerInventoryBalance(9,1,4): I_customer(9)(1)(4)#201
                                 - b_customer(9)(1)(4)#441 - w(9)(1)(1)(4)#831
                                 - w(9)(2)(1)(4)#855  = 16
 CustomerInventoryBalance(10,1,4): I_customer(10)(1)(4)#225
                                 - b_customer(10)(1)(4)#465
                                 - w(10)(1)(1)(4)#879 - w(10)(2)(1)(4)#903
                                  = -2
 CustomerInventoryBalance(11,1,4): I_customer(11)(1)(4)#249
                                 - b_customer(11)(1)(4)#489
                                 - w(11)(1)(1)(4)#927 - w(11)(2)(1)(4)#951  = 1
 CustomerInventoryBalance(12,1,4): I_customer(12)(1)(4)#273
                                 - b_customer(12)(1)(4)#513
                                 - w(12)(1)(1)(4)#975 - w(12)(2)(1)(4)#999
                                  = 13
 CustomerInventoryBalance(3,2,4): - I_customer(3)(1)(4)#57
                                 + I_customer(3)(2)(4)#65
                                 - b_customer(3)(2)(4)#305 - w(3)(1)(2)(4)#551
                                 - w(3)(2)(2)(4)#575  = -20
 CustomerInventoryBalance(4,2,4): - I_customer(4)(1)(4)#81
                                 + I_customer(4)(2)(4)#89
                                 - b_customer(4)(2)(4)#329 - w(4)(1)(2)(4)#599
                                 - w(4)(2)(2)(4)#623  = -7
 CustomerInventoryBalance(5,2,4): - I_customer(5)(1)(4)#105
                                 + I_customer(5)(2)(4)#113
                                 - b_customer(5)(2)(4)#353 - w(5)(1)(2)(4)#647
                                 - w(5)(2)(2)(4)#671  = -6
 CustomerInventoryBalance(6,2,4): - I_customer(6)(1)(4)#129
                                 + I_customer(6)(2)(4)#137
                                 - b_customer(6)(2)(4)#377 - w(6)(1)(2)(4)#695
                                 - w(6)(2)(2)(4)#719  = -14
 CustomerInventoryBalance(7,2,4): - I_customer(7)(1)(4)#153
                                 + I_customer(7)(2)(4)#161
                                 - b_customer(7)(2)(4)#401 - w(7)(1)(2)(4)#743
                                 - w(7)(2)(2)(4)#767  = -6
 CustomerInventoryBalance(8,2,4): - I_customer(8)(1)(4)#177
                                 + I_customer(8)(2)(4)#185
                                 - b_customer(8)(2)(4)#425 - w(8)(1)(2)(4)#791
                                 - w(8)(2)(2)(4)#815  = -17
 CustomerInventoryBalance(9,2,4): - I_customer(9)(1)(4)#201
                                 + I_customer(9)(2)(4)#209
                                 - b_customer(9)(2)(4)#449 - w(9)(1)(2)(4)#839
                                 - w(9)(2)(2)(4)#863  = -17
 CustomerInventoryBalance(10,2,4): - I_customer(10)(1)(4)#225
                                 + I_customer(10)(2)(4)#233
                                 - b_customer(10)(2)(4)#473
                                 - w(10)(1)(2)(4)#887 - w(10)(2)(2)(4)#911
                                  = -19
 CustomerInventoryBalance(11,2,4): - I_customer(11)(1)(4)#249
                                 + I_customer(11)(2)(4)#257
                                 - b_customer(11)(2)(4)#497
                                 - w(11)(1)(2)(4)#935 - w(11)(2)(2)(4)#959
                                  = -19
 CustomerInventoryBalance(12,2,4): - I_customer(12)(1)(4)#273
                                 + I_customer(12)(2)(4)#281
                                 - b_customer(12)(2)(4)#521
                                 - w(12)(1)(2)(4)#983 - w(12)(2)(2)(4)#1007
                                  = -15
 CustomerInventoryBalance(3,3,4): - I_customer(3)(2)(4)#65
                                 + I_customer(3)(3)(4)#73
                                 - b_customer(3)(3)(4)#313 - w(3)(1)(3)(4)#559
                                 - w(3)(2)(3)(4)#583  = -21
 CustomerInventoryBalance(4,3,4): - I_customer(4)(2)(4)#89
                                 + I_customer(4)(3)(4)#97
                                 - b_customer(4)(3)(4)#337 - w(4)(1)(3)(4)#607
                                 - w(4)(2)(3)(4)#631  = -6
 CustomerInventoryBalance(5,3,4): - I_customer(5)(2)(4)#113
                                 + I_customer(5)(3)(4)#121
                                 - b_customer(5)(3)(4)#361 - w(5)(1)(3)(4)#655
                                 - w(5)(2)(3)(4)#679  = -6
 CustomerInventoryBalance(6,3,4): - I_customer(6)(2)(4)#137
                                 + I_customer(6)(3)(4)#145
                                 - b_customer(6)(3)(4)#385 - w(6)(1)(3)(4)#703
                                 - w(6)(2)(3)(4)#727  = -14
 CustomerInventoryBalance(7,3,4): - I_customer(7)(2)(4)#161
                                 + I_customer(7)(3)(4)#169
                                 - b_customer(7)(3)(4)#409 - w(7)(1)(3)(4)#751
                                 - w(7)(2)(3)(4)#775  = -6
 CustomerInventoryBalance(8,3,4): - I_customer(8)(2)(4)#185
                                 + I_customer(8)(3)(4)#193
                                 - b_customer(8)(3)(4)#433 - w(8)(1)(3)(4)#799
                                 - w(8)(2)(3)(4)#823  = -16
 CustomerInventoryBalance(9,3,4): - I_customer(9)(2)(4)#209
                                 + I_customer(9)(3)(4)#217
                                 - b_customer(9)(3)(4)#457 - w(9)(1)(3)(4)#847
                                 - w(9)(2)(3)(4)#871  = -16
 CustomerInventoryBalance(10,3,4): - I_customer(10)(2)(4)#233
                                 + I_customer(10)(3)(4)#241
                                 - b_customer(10)(3)(4)#481
                                 - w(10)(1)(3)(4)#895 - w(10)(2)(3)(4)#919
                                  = -18
 CustomerInventoryBalance(11,3,4): - I_customer(11)(2)(4)#257
                                 + I_customer(11)(3)(4)#265
                                 - b_customer(11)(3)(4)#505
                                 - w(11)(1)(3)(4)#943 - w(11)(2)(3)(4)#967
                                  = -14
 CustomerInventoryBalance(12,3,4): - I_customer(12)(2)(4)#281
                                 + I_customer(12)(3)(4)#289
                                 - b_customer(12)(3)(4)#529
                                 - w(12)(1)(3)(4)#991 - w(12)(2)(3)(4)#1015
                                  = -12
 CustomerInventoryBalance(3,1,5): I_customer(3)(1)(5)#58
                                 - b_customer(3)(1)(5)#298 - w(3)(1)(1)(5)#544
                                 - w(3)(2)(1)(5)#568  = 20
 CustomerInventoryBalance(4,1,5): I_customer(4)(1)(5)#82
                                 - b_customer(4)(1)(5)#322 - w(4)(1)(1)(5)#592
                                 - w(4)(2)(1)(5)#616  = -1
 CustomerInventoryBalance(5,1,5): I_customer(5)(1)(5)#106
                                 - b_customer(5)(1)(5)#346 - w(5)(1)(1)(5)#640
                                 - w(5)(2)(1)(5)#664  = -1
 CustomerInventoryBalance(6,1,5): I_customer(6)(1)(5)#130
                                 - b_customer(6)(1)(5)#370 - w(6)(1)(1)(5)#688
                                 - w(6)(2)(1)(5)#712  = -1
 CustomerInventoryBalance(7,1,5): I_customer(7)(1)(5)#154
                                 - b_customer(7)(1)(5)#394 - w(7)(1)(1)(5)#736
                                 - w(7)(2)(1)(5)#760  = 6
 CustomerInventoryBalance(8,1,5): I_customer(8)(1)(5)#178
                                 - b_customer(8)(1)(5)#418 - w(8)(1)(1)(5)#784
                                 - w(8)(2)(1)(5)#808  = -3
 CustomerInventoryBalance(9,1,5): I_customer(9)(1)(5)#202
                                 - b_customer(9)(1)(5)#442 - w(9)(1)(1)(5)#832
                                 - w(9)(2)(1)(5)#856  = 13
 CustomerInventoryBalance(10,1,5): I_customer(10)(1)(5)#226
                                 - b_customer(10)(1)(5)#466
                                 - w(10)(1)(1)(5)#880 - w(10)(2)(1)(5)#904  = 3
 CustomerInventoryBalance(11,1,5): I_customer(11)(1)(5)#250
                                 - b_customer(11)(1)(5)#490
                                 - w(11)(1)(1)(5)#928 - w(11)(2)(1)(5)#952
                                  = -3
 CustomerInventoryBalance(12,1,5): I_customer(12)(1)(5)#274
                                 - b_customer(12)(1)(5)#514
                                 - w(12)(1)(1)(5)#976 - w(12)(2)(1)(5)#1000
                                  = 14
 CustomerInventoryBalance(3,2,5): - I_customer(3)(1)(5)#58
                                 + I_customer(3)(2)(5)#66
                                 - b_customer(3)(2)(5)#306 - w(3)(1)(2)(5)#552
                                 - w(3)(2)(2)(5)#576  = -22
 CustomerInventoryBalance(4,2,5): - I_customer(4)(1)(5)#82
                                 + I_customer(4)(2)(5)#90
                                 - b_customer(4)(2)(5)#330 - w(4)(1)(2)(5)#600
                                 - w(4)(2)(2)(5)#624  = -8
 CustomerInventoryBalance(5,2,5): - I_customer(5)(1)(5)#106
                                 + I_customer(5)(2)(5)#114
                                 - b_customer(5)(2)(5)#354 - w(5)(1)(2)(5)#648
                                 - w(5)(2)(2)(5)#672  = -5
 CustomerInventoryBalance(6,2,5): - I_customer(6)(1)(5)#130
                                 + I_customer(6)(2)(5)#138
                                 - b_customer(6)(2)(5)#378 - w(6)(1)(2)(5)#696
                                 - w(6)(2)(2)(5)#720  = -15
 CustomerInventoryBalance(7,2,5): - I_customer(7)(1)(5)#154
                                 + I_customer(7)(2)(5)#162
                                 - b_customer(7)(2)(5)#402 - w(7)(1)(2)(5)#744
                                 - w(7)(2)(2)(5)#768  = -6
 CustomerInventoryBalance(8,2,5): - I_customer(8)(1)(5)#178
                                 + I_customer(8)(2)(5)#186
                                 - b_customer(8)(2)(5)#426 - w(8)(1)(2)(5)#792
                                 - w(8)(2)(2)(5)#816  = -15
 CustomerInventoryBalance(9,2,5): - I_customer(9)(1)(5)#202
                                 + I_customer(9)(2)(5)#210
                                 - b_customer(9)(2)(5)#450 - w(9)(1)(2)(5)#840
                                 - w(9)(2)(2)(5)#864  = -19
 CustomerInventoryBalance(10,2,5): - I_customer(10)(1)(5)#226
                                 + I_customer(10)(2)(5)#234
                                 - b_customer(10)(2)(5)#474
                                 - w(10)(1)(2)(5)#888 - w(10)(2)(2)(5)#912
                                  = -16
 CustomerInventoryBalance(11,2,5): - I_customer(11)(1)(5)#250
                                 + I_customer(11)(2)(5)#258
                                 - b_customer(11)(2)(5)#498
                                 - w(11)(1)(2)(5)#936 - w(11)(2)(2)(5)#960
                                  = -18
 CustomerInventoryBalance(12,2,5): - I_customer(12)(1)(5)#274
                                 + I_customer(12)(2)(5)#282
                                 - b_customer(12)(2)(5)#522
                                 - w(12)(1)(2)(5)#984 - w(12)(2)(2)(5)#1008
                                  = -12
 CustomerInventoryBalance(3,3,5): - I_customer(3)(2)(5)#66
                                 + I_customer(3)(3)(5)#74
                                 - b_customer(3)(3)(5)#314 - w(3)(1)(3)(5)#560
                                 - w(3)(2)(3)(5)#584  = -17
 CustomerInventoryBalance(4,3,5): - I_customer(4)(2)(5)#90
                                 + I_customer(4)(3)(5)#98
                                 - b_customer(4)(3)(5)#338 - w(4)(1)(3)(5)#608
                                 - w(4)(2)(3)(5)#632  = -7
 CustomerInventoryBalance(5,3,5): - I_customer(5)(2)(5)#114
                                 + I_customer(5)(3)(5)#122
                                 - b_customer(5)(3)(5)#362 - w(5)(1)(3)(5)#656
                                 - w(5)(2)(3)(5)#680  = -6
 CustomerInventoryBalance(6,3,5): - I_customer(6)(2)(5)#138
                                 + I_customer(6)(3)(5)#146
                                 - b_customer(6)(3)(5)#386 - w(6)(1)(3)(5)#704
                                 - w(6)(2)(3)(5)#728  = -18
 CustomerInventoryBalance(7,3,5): - I_customer(7)(2)(5)#162
                                 + I_customer(7)(3)(5)#170
                                 - b_customer(7)(3)(5)#410 - w(7)(1)(3)(5)#752
                                 - w(7)(2)(3)(5)#776  = -7
 CustomerInventoryBalance(8,3,5): - I_customer(8)(2)(5)#186
                                 + I_customer(8)(3)(5)#194
                                 - b_customer(8)(3)(5)#434 - w(8)(1)(3)(5)#800
                                 - w(8)(2)(3)(5)#824  = -15
 CustomerInventoryBalance(9,3,5): - I_customer(9)(2)(5)#210
                                 + I_customer(9)(3)(5)#218
                                 - b_customer(9)(3)(5)#458 - w(9)(1)(3)(5)#848
                                 - w(9)(2)(3)(5)#872  = -17
 CustomerInventoryBalance(10,3,5): - I_customer(10)(2)(5)#234
                                 + I_customer(10)(3)(5)#242
                                 - b_customer(10)(3)(5)#482
                                 - w(10)(1)(3)(5)#896 - w(10)(2)(3)(5)#920
                                  = -14
 CustomerInventoryBalance(11,3,5): - I_customer(11)(2)(5)#258
                                 + I_customer(11)(3)(5)#266
                                 - b_customer(11)(3)(5)#506
                                 - w(11)(1)(3)(5)#944 - w(11)(2)(3)(5)#968
                                  = -14
 CustomerInventoryBalance(12,3,5): - I_customer(12)(2)(5)#282
                                 + I_customer(12)(3)(5)#290
                                 - b_customer(12)(3)(5)#530
                                 - w(12)(1)(3)(5)#992 - w(12)(2)(3)(5)#1016
                                  = -12
 CustomerInventoryBalance(3,1,6): I_customer(3)(1)(6)#59
                                 - b_customer(3)(1)(6)#299 - w(3)(1)(1)(6)#545
                                 - w(3)(2)(1)(6)#569  = 16
 CustomerInventoryBalance(4,1,6): I_customer(4)(1)(6)#83
                                 - b_customer(4)(1)(6)#323 - w(4)(1)(1)(6)#593
                                 - w(4)(2)(1)(6)#617  = 1
 CustomerInventoryBalance(5,1,6): I_customer(5)(1)(6)#107
                                 - b_customer(5)(1)(6)#347 - w(5)(1)(1)(6)#641
                                 - w(5)(2)(1)(6)#665  = 0
 CustomerInventoryBalance(6,1,6): I_customer(6)(1)(6)#131
                                 - b_customer(6)(1)(6)#371 - w(6)(1)(1)(6)#689
                                 - w(6)(2)(1)(6)#713  = -1
 CustomerInventoryBalance(7,1,6): I_customer(7)(1)(6)#155
                                 - b_customer(7)(1)(6)#395 - w(7)(1)(1)(6)#737
                                 - w(7)(2)(1)(6)#761  = 7
 CustomerInventoryBalance(8,1,6): I_customer(8)(1)(6)#179
                                 - b_customer(8)(1)(6)#419 - w(8)(1)(1)(6)#785
                                 - w(8)(2)(1)(6)#809  = 1
 CustomerInventoryBalance(9,1,6): I_customer(9)(1)(6)#203
                                 - b_customer(9)(1)(6)#443 - w(9)(1)(1)(6)#833
                                 - w(9)(2)(1)(6)#857  = 13
 CustomerInventoryBalance(10,1,6): I_customer(10)(1)(6)#227
                                 - b_customer(10)(1)(6)#467
                                 - w(10)(1)(1)(6)#881 - w(10)(2)(1)(6)#905  = 0
 CustomerInventoryBalance(11,1,6): I_customer(11)(1)(6)#251
                                 - b_customer(11)(1)(6)#491
                                 - w(11)(1)(1)(6)#929 - w(11)(2)(1)(6)#953  = 2
 CustomerInventoryBalance(12,1,6): I_customer(12)(1)(6)#275
                                 - b_customer(12)(1)(6)#515
                                 - w(12)(1)(1)(6)#977 - w(12)(2)(1)(6)#1001
                                  = 16
 CustomerInventoryBalance(3,2,6): - I_customer(3)(1)(6)#59
                                 + I_customer(3)(2)(6)#67
                                 - b_customer(3)(2)(6)#307 - w(3)(1)(2)(6)#553
                                 - w(3)(2)(2)(6)#577  = -19
 CustomerInventoryBalance(4,2,6): - I_customer(4)(1)(6)#83
                                 + I_customer(4)(2)(6)#91
                                 - b_customer(4)(2)(6)#331 - w(4)(1)(2)(6)#601
                                 - w(4)(2)(2)(6)#625  = -6
 CustomerInventoryBalance(5,2,6): - I_customer(5)(1)(6)#107
                                 + I_customer(5)(2)(6)#115
                                 - b_customer(5)(2)(6)#355 - w(5)(1)(2)(6)#649
                                 - w(5)(2)(2)(6)#673  = -7
 CustomerInventoryBalance(6,2,6): - I_customer(6)(1)(6)#131
                                 + I_customer(6)(2)(6)#139
                                 - b_customer(6)(2)(6)#379 - w(6)(1)(2)(6)#697
                                 - w(6)(2)(2)(6)#721  = -15
 CustomerInventoryBalance(7,2,6): - I_customer(7)(1)(6)#155
                                 + I_customer(7)(2)(6)#163
                                 - b_customer(7)(2)(6)#403 - w(7)(1)(2)(6)#745
                                 - w(7)(2)(2)(6)#769  = -6
 CustomerInventoryBalance(8,2,6): - I_customer(8)(1)(6)#179
                                 + I_customer(8)(2)(6)#187
                                 - b_customer(8)(2)(6)#427 - w(8)(1)(2)(6)#793
                                 - w(8)(2)(2)(6)#817  = -21
 CustomerInventoryBalance(9,2,6): - I_customer(9)(1)(6)#203
                                 + I_customer(9)(2)(6)#211
                                 - b_customer(9)(2)(6)#451 - w(9)(1)(2)(6)#841
                                 - w(9)(2)(2)(6)#865  = -14
 CustomerInventoryBalance(10,2,6): - I_customer(10)(1)(6)#227
                                 + I_customer(10)(2)(6)#235
                                 - b_customer(10)(2)(6)#475
                                 - w(10)(1)(2)(6)#889 - w(10)(2)(2)(6)#913
                                  = -18
 CustomerInventoryBalance(11,2,6): - I_customer(11)(1)(6)#251
                                 + I_customer(11)(2)(6)#259
                                 - b_customer(11)(2)(6)#499
                                 - w(11)(1)(2)(6)#937 - w(11)(2)(2)(6)#961
                                  = -13
 CustomerInventoryBalance(12,2,6): - I_customer(12)(1)(6)#275
                                 + I_customer(12)(2)(6)#283
                                 - b_customer(12)(2)(6)#523
                                 - w(12)(1)(2)(6)#985 - w(12)(2)(2)(6)#1009
                                  = -12
 CustomerInventoryBalance(3,3,6): - I_customer(3)(2)(6)#67
                                 + I_customer(3)(3)(6)#75
                                 - b_customer(3)(3)(6)#315 - w(3)(1)(3)(6)#561
                                 - w(3)(2)(3)(6)#585  = -18
 CustomerInventoryBalance(4,3,6): - I_customer(4)(2)(6)#91
                                 + I_customer(4)(3)(6)#99
                                 - b_customer(4)(3)(6)#339 - w(4)(1)(3)(6)#609
                                 - w(4)(2)(3)(6)#633  = -7
 CustomerInventoryBalance(5,3,6): - I_customer(5)(2)(6)#115
                                 + I_customer(5)(3)(6)#123
                                 - b_customer(5)(3)(6)#363 - w(5)(1)(3)(6)#657
                                 - w(5)(2)(3)(6)#681  = -5
 CustomerInventoryBalance(6,3,6): - I_customer(6)(2)(6)#139
                                 + I_customer(6)(3)(6)#147
                                 - b_customer(6)(3)(6)#387 - w(6)(1)(3)(6)#705
                                 - w(6)(2)(3)(6)#729  = -13
 CustomerInventoryBalance(7,3,6): - I_customer(7)(2)(6)#163
                                 + I_customer(7)(3)(6)#171
                                 - b_customer(7)(3)(6)#411 - w(7)(1)(3)(6)#753
                                 - w(7)(2)(3)(6)#777  = -7
 CustomerInventoryBalance(8,3,6): - I_customer(8)(2)(6)#187
                                 + I_customer(8)(3)(6)#195
                                 - b_customer(8)(3)(6)#435 - w(8)(1)(3)(6)#801
                                 - w(8)(2)(3)(6)#825  = -17
 CustomerInventoryBalance(9,3,6): - I_customer(9)(2)(6)#211
                                 + I_customer(9)(3)(6)#219
                                 - b_customer(9)(3)(6)#459 - w(9)(1)(3)(6)#849
                                 - w(9)(2)(3)(6)#873  = -19
 CustomerInventoryBalance(10,3,6): - I_customer(10)(2)(6)#235
                                 + I_customer(10)(3)(6)#243
                                 - b_customer(10)(3)(6)#483
                                 - w(10)(1)(3)(6)#897 - w(10)(2)(3)(6)#921
                                  = -18
 CustomerInventoryBalance(11,3,6): - I_customer(11)(2)(6)#259
                                 + I_customer(11)(3)(6)#267
                                 - b_customer(11)(3)(6)#507
                                 - w(11)(1)(3)(6)#945 - w(11)(2)(3)(6)#969
                                  = -14
 CustomerInventoryBalance(12,3,6): - I_customer(12)(2)(6)#283
                                 + I_customer(12)(3)(6)#291
                                 - b_customer(12)(3)(6)#531
                                 - w(12)(1)(3)(6)#993 - w(12)(2)(3)(6)#1017
                                  = -16
 CustomerInventoryBalance(3,1,7): I_customer(3)(1)(7)#60
                                 - b_customer(3)(1)(7)#300 - w(3)(1)(1)(7)#546
                                 - w(3)(2)(1)(7)#570  = 15
 CustomerInventoryBalance(4,1,7): I_customer(4)(1)(7)#84
                                 - b_customer(4)(1)(7)#324 - w(4)(1)(1)(7)#594
                                 - w(4)(2)(1)(7)#618  = 1
 CustomerInventoryBalance(5,1,7): I_customer(5)(1)(7)#108
                                 - b_customer(5)(1)(7)#348 - w(5)(1)(1)(7)#642
                                 - w(5)(2)(1)(7)#666  = 0
 CustomerInventoryBalance(6,1,7): I_customer(6)(1)(7)#132
                                 - b_customer(6)(1)(7)#372 - w(6)(1)(1)(7)#690
                                 - w(6)(2)(1)(7)#714  = -1
 CustomerInventoryBalance(7,1,7): I_customer(7)(1)(7)#156
                                 - b_customer(7)(1)(7)#396 - w(7)(1)(1)(7)#738
                                 - w(7)(2)(1)(7)#762  = 8
 CustomerInventoryBalance(8,1,7): I_customer(8)(1)(7)#180
                                 - b_customer(8)(1)(7)#420 - w(8)(1)(1)(7)#786
                                 - w(8)(2)(1)(7)#810  = -2
 CustomerInventoryBalance(9,1,7): I_customer(9)(1)(7)#204
                                 - b_customer(9)(1)(7)#444 - w(9)(1)(1)(7)#834
                                 - w(9)(2)(1)(7)#858  = 19
 CustomerInventoryBalance(10,1,7): I_customer(10)(1)(7)#228
                                 - b_customer(10)(1)(7)#468
                                 - w(10)(1)(1)(7)#882 - w(10)(2)(1)(7)#906  = 0
 CustomerInventoryBalance(11,1,7): I_customer(11)(1)(7)#252
                                 - b_customer(11)(1)(7)#492
                                 - w(11)(1)(1)(7)#930 - w(11)(2)(1)(7)#954
                                  = -1
 CustomerInventoryBalance(12,1,7): I_customer(12)(1)(7)#276
                                 - b_customer(12)(1)(7)#516
                                 - w(12)(1)(1)(7)#978 - w(12)(2)(1)(7)#1002
                                  = 16
 CustomerInventoryBalance(3,2,7): - I_customer(3)(1)(7)#60
                                 + I_customer(3)(2)(7)#68
                                 - b_customer(3)(2)(7)#308 - w(3)(1)(2)(7)#554
                                 - w(3)(2)(2)(7)#578  = -21
 CustomerInventoryBalance(4,2,7): - I_customer(4)(1)(7)#84
                                 + I_customer(4)(2)(7)#92
                                 - b_customer(4)(2)(7)#332 - w(4)(1)(2)(7)#602
                                 - w(4)(2)(2)(7)#626  = -8
 CustomerInventoryBalance(5,2,7): - I_customer(5)(1)(7)#108
                                 + I_customer(5)(2)(7)#116
                                 - b_customer(5)(2)(7)#356 - w(5)(1)(2)(7)#650
                                 - w(5)(2)(2)(7)#674  = -5
 CustomerInventoryBalance(6,2,7): - I_customer(6)(1)(7)#132
                                 + I_customer(6)(2)(7)#140
                                 - b_customer(6)(2)(7)#380 - w(6)(1)(2)(7)#698
                                 - w(6)(2)(2)(7)#722  = -15
 CustomerInventoryBalance(7,2,7): - I_customer(7)(1)(7)#156
                                 + I_customer(7)(2)(7)#164
                                 - b_customer(7)(2)(7)#404 - w(7)(1)(2)(7)#746
                                 - w(7)(2)(2)(7)#770  = -7
 CustomerInventoryBalance(8,2,7): - I_customer(8)(1)(7)#180
                                 + I_customer(8)(2)(7)#188
                                 - b_customer(8)(2)(7)#428 - w(8)(1)(2)(7)#794
                                 - w(8)(2)(2)(7)#818  = -20
 CustomerInventoryBalance(9,2,7): - I_customer(9)(1)(7)#204
                                 + I_customer(9)(2)(7)#212
                                 - b_customer(9)(2)(7)#452 - w(9)(1)(2)(7)#842
                                 - w(9)(2)(2)(7)#866  = -15
 CustomerInventoryBalance(10,2,7): - I_customer(10)(1)(7)#228
                                 + I_customer(10)(2)(7)#236
                                 - b_customer(10)(2)(7)#476
                                 - w(10)(1)(2)(7)#890 - w(10)(2)(2)(7)#914
                                  = -17
 CustomerInventoryBalance(11,2,7): - I_customer(11)(1)(7)#252
                                 + I_customer(11)(2)(7)#260
                                 - b_customer(11)(2)(7)#500
                                 - w(11)(1)(2)(7)#938 - w(11)(2)(2)(7)#962
                                  = -17
 CustomerInventoryBalance(12,2,7): - I_customer(12)(1)(7)#276
                                 + I_customer(12)(2)(7)#284
                                 - b_customer(12)(2)(7)#524
                                 - w(12)(1)(2)(7)#986 - w(12)(2)(2)(7)#1010
                                  = -14
 CustomerInventoryBalance(3,3,7): - I_customer(3)(2)(7)#68
                                 + I_customer(3)(3)(7)#76
                                 - b_customer(3)(3)(7)#316 - w(3)(1)(3)(7)#562
                                 - w(3)(2)(3)(7)#586  = -19
 CustomerInventoryBalance(4,3,7): - I_customer(4)(2)(7)#92
                                 + I_customer(4)(3)(7)#100
                                 - b_customer(4)(3)(7)#340 - w(4)(1)(3)(7)#610
                                 - w(4)(2)(3)(7)#634  = -7
 CustomerInventoryBalance(5,3,7): - I_customer(5)(2)(7)#116
                                 + I_customer(5)(3)(7)#124
                                 - b_customer(5)(3)(7)#364 - w(5)(1)(3)(7)#658
                                 - w(5)(2)(3)(7)#682  = -5
 CustomerInventoryBalance(6,3,7): - I_customer(6)(2)(7)#140
                                 + I_customer(6)(3)(7)#148
                                 - b_customer(6)(3)(7)#388 - w(6)(1)(3)(7)#706
                                 - w(6)(2)(3)(7)#730  = -17
 CustomerInventoryBalance(7,3,7): - I_customer(7)(2)(7)#164
                                 + I_customer(7)(3)(7)#172
                                 - b_customer(7)(3)(7)#412 - w(7)(1)(3)(7)#754
                                 - w(7)(2)(3)(7)#778  = -7
 CustomerInventoryBalance(8,3,7): - I_customer(8)(2)(7)#188
                                 + I_customer(8)(3)(7)#196
                                 - b_customer(8)(3)(7)#436 - w(8)(1)(3)(7)#802
                                 - w(8)(2)(3)(7)#826  = -16
 CustomerInventoryBalance(9,3,7): - I_customer(9)(2)(7)#212
                                 + I_customer(9)(3)(7)#220
                                 - b_customer(9)(3)(7)#460 - w(9)(1)(3)(7)#850
                                 - w(9)(2)(3)(7)#874  = -19
 CustomerInventoryBalance(10,3,7): - I_customer(10)(2)(7)#236
                                 + I_customer(10)(3)(7)#244
                                 - b_customer(10)(3)(7)#484
                                 - w(10)(1)(3)(7)#898 - w(10)(2)(3)(7)#922
                                  = -18
 CustomerInventoryBalance(11,3,7): - I_customer(11)(2)(7)#260
                                 + I_customer(11)(3)(7)#268
                                 - b_customer(11)(3)(7)#508
                                 - w(11)(1)(3)(7)#946 - w(11)(2)(3)(7)#970
                                  = -17
 CustomerInventoryBalance(12,3,7): - I_customer(12)(2)(7)#284
                                 + I_customer(12)(3)(7)#292
                                 - b_customer(12)(3)(7)#532
                                 - w(12)(1)(3)(7)#994 - w(12)(2)(3)(7)#1018
                                  = -16
 CustomerInventoryBalance(3,1,8): I_customer(3)(1)(8)#61
                                 - b_customer(3)(1)(8)#301 - w(3)(1)(1)(8)#547
                                 - w(3)(2)(1)(8)#571  = 19
 CustomerInventoryBalance(4,1,8): I_customer(4)(1)(8)#85
                                 - b_customer(4)(1)(8)#325 - w(4)(1)(1)(8)#595
                                 - w(4)(2)(1)(8)#619  = 1
 CustomerInventoryBalance(5,1,8): I_customer(5)(1)(8)#109
                                 - b_customer(5)(1)(8)#349 - w(5)(1)(1)(8)#643
                                 - w(5)(2)(1)(8)#667  = 1
 CustomerInventoryBalance(6,1,8): I_customer(6)(1)(8)#133
                                 - b_customer(6)(1)(8)#373 - w(6)(1)(1)(8)#691
                                 - w(6)(2)(1)(8)#715  = -1
 CustomerInventoryBalance(7,1,8): I_customer(7)(1)(8)#157
                                 - b_customer(7)(1)(8)#397 - w(7)(1)(1)(8)#739
                                 - w(7)(2)(1)(8)#763  = 6
 CustomerInventoryBalance(8,1,8): I_customer(8)(1)(8)#181
                                 - b_customer(8)(1)(8)#421 - w(8)(1)(1)(8)#787
                                 - w(8)(2)(1)(8)#811  = 3
 CustomerInventoryBalance(9,1,8): I_customer(9)(1)(8)#205
                                 - b_customer(9)(1)(8)#445 - w(9)(1)(1)(8)#835
                                 - w(9)(2)(1)(8)#859  = 17
 CustomerInventoryBalance(10,1,8): I_customer(10)(1)(8)#229
                                 - b_customer(10)(1)(8)#469
                                 - w(10)(1)(1)(8)#883 - w(10)(2)(1)(8)#907  = 3
 CustomerInventoryBalance(11,1,8): I_customer(11)(1)(8)#253
                                 - b_customer(11)(1)(8)#493
                                 - w(11)(1)(1)(8)#931 - w(11)(2)(1)(8)#955
                                  = -3
 CustomerInventoryBalance(12,1,8): I_customer(12)(1)(8)#277
                                 - b_customer(12)(1)(8)#517
                                 - w(12)(1)(1)(8)#979 - w(12)(2)(1)(8)#1003
                                  = 16
 CustomerInventoryBalance(3,2,8): - I_customer(3)(1)(8)#61
                                 + I_customer(3)(2)(8)#69
                                 - b_customer(3)(2)(8)#309 - w(3)(1)(2)(8)#555
                                 - w(3)(2)(2)(8)#579  = -19
 CustomerInventoryBalance(4,2,8): - I_customer(4)(1)(8)#85
                                 + I_customer(4)(2)(8)#93
                                 - b_customer(4)(2)(8)#333 - w(4)(1)(2)(8)#603
                                 - w(4)(2)(2)(8)#627  = -6
 CustomerInventoryBalance(5,2,8): - I_customer(5)(1)(8)#109
                                 + I_customer(5)(2)(8)#117
                                 - b_customer(5)(2)(8)#357 - w(5)(1)(2)(8)#651
                                 - w(5)(2)(2)(8)#675  = -5
 CustomerInventoryBalance(6,2,8): - I_customer(6)(1)(8)#133
                                 + I_customer(6)(2)(8)#141
                                 - b_customer(6)(2)(8)#381 - w(6)(1)(2)(8)#699
                                 - w(6)(2)(2)(8)#723  = -17
 CustomerInventoryBalance(7,2,8): - I_customer(7)(1)(8)#157
                                 + I_customer(7)(2)(8)#165
                                 - b_customer(7)(2)(8)#405 - w(7)(1)(2)(8)#747
                                 - w(7)(2)(2)(8)#771  = -7
 CustomerInventoryBalance(8,2,8): - I_customer(8)(1)(8)#181
                                 + I_customer(8)(2)(8)#189
                                 - b_customer(8)(2)(8)#429 - w(8)(1)(2)(8)#795
                                 - w(8)(2)(2)(8)#819  = -19
 CustomerInventoryBalance(9,2,8): - I_customer(9)(1)(8)#205
                                 + I_customer(9)(2)(8)#213
                                 - b_customer(9)(2)(8)#453 - w(9)(1)(2)(8)#843
                                 - w(9)(2)(2)(8)#867  = -15
 CustomerInventoryBalance(10,2,8): - I_customer(10)(1)(8)#229
                                 + I_customer(10)(2)(8)#237
                                 - b_customer(10)(2)(8)#477
                                 - w(10)(1)(2)(8)#891 - w(10)(2)(2)(8)#915
                                  = -13
 CustomerInventoryBalance(11,2,8): - I_customer(11)(1)(8)#253
                                 + I_customer(11)(2)(8)#261
                                 - b_customer(11)(2)(8)#501
                                 - w(11)(1)(2)(8)#939 - w(11)(2)(2)(8)#963
                                  = -14
 CustomerInventoryBalance(12,2,8): - I_customer(12)(1)(8)#277
                                 + I_customer(12)(2)(8)#285
                                 - b_customer(12)(2)(8)#525
                                 - w(12)(1)(2)(8)#987 - w(12)(2)(2)(8)#1011
                                  = -17
 CustomerInventoryBalance(3,3,8): - I_customer(3)(2)(8)#69
                                 + I_customer(3)(3)(8)#77
                                 - b_customer(3)(3)(8)#317 - w(3)(1)(3)(8)#563
                                 - w(3)(2)(3)(8)#587  = -15
 CustomerInventoryBalance(4,3,8): - I_customer(4)(2)(8)#93
                                 + I_customer(4)(3)(8)#101
                                 - b_customer(4)(3)(8)#341 - w(4)(1)(3)(8)#611
                                 - w(4)(2)(3)(8)#635  = -7
 CustomerInventoryBalance(5,3,8): - I_customer(5)(2)(8)#117
                                 + I_customer(5)(3)(8)#125
                                 - b_customer(5)(3)(8)#365 - w(5)(1)(3)(8)#659
                                 - w(5)(2)(3)(8)#683  = -7
 CustomerInventoryBalance(6,3,8): - I_customer(6)(2)(8)#141
                                 + I_customer(6)(3)(8)#149
                                 - b_customer(6)(3)(8)#389 - w(6)(1)(3)(8)#707
                                 - w(6)(2)(3)(8)#731  = -16
 CustomerInventoryBalance(7,3,8): - I_customer(7)(2)(8)#165
                                 + I_customer(7)(3)(8)#173
                                 - b_customer(7)(3)(8)#413 - w(7)(1)(3)(8)#755
                                 - w(7)(2)(3)(8)#779  = -7
 CustomerInventoryBalance(8,3,8): - I_customer(8)(2)(8)#189
                                 + I_customer(8)(3)(8)#197
                                 - b_customer(8)(3)(8)#437 - w(8)(1)(3)(8)#803
                                 - w(8)(2)(3)(8)#827  = -21
 CustomerInventoryBalance(9,3,8): - I_customer(9)(2)(8)#213
                                 + I_customer(9)(3)(8)#221
                                 - b_customer(9)(3)(8)#461 - w(9)(1)(3)(8)#851
                                 - w(9)(2)(3)(8)#875  = -18
 CustomerInventoryBalance(10,3,8): - I_customer(10)(2)(8)#237
                                 + I_customer(10)(3)(8)#245
                                 - b_customer(10)(3)(8)#485
                                 - w(10)(1)(3)(8)#899 - w(10)(2)(3)(8)#923
                                  = -17
 CustomerInventoryBalance(11,3,8): - I_customer(11)(2)(8)#261
                                 + I_customer(11)(3)(8)#269
                                 - b_customer(11)(3)(8)#509
                                 - w(11)(1)(3)(8)#947 - w(11)(2)(3)(8)#971
                                  = -16
 CustomerInventoryBalance(12,3,8): - I_customer(12)(2)(8)#285
                                 + I_customer(12)(3)(8)#293
                                 - b_customer(12)(3)(8)#533
                                 - w(12)(1)(3)(8)#995 - w(12)(2)(3)(8)#1019
                                  = -14
 CustomerInventoryCapacity(3,1,1): I_customer(3)(1)(1)#54 <= 58
 CustomerInventoryCapacity(4,1,1): I_customer(4)(1)(1)#78 <= 20
 CustomerInventoryCapacity(5,1,1): I_customer(5)(1)(1)#102 <= 19
 CustomerInventoryCapacity(6,1,1): I_customer(6)(1)(1)#126 <= 51
 CustomerInventoryCapacity(7,1,1): I_customer(7)(1)(1)#150 <= 20
 CustomerInventoryCapacity(8,1,1): I_customer(8)(1)(1)#174 <= 57
 CustomerInventoryCapacity(9,1,1): I_customer(9)(1)(1)#198 <= 45
 CustomerInventoryCapacity(10,1,1): I_customer(10)(1)(1)#222 <= 50
 CustomerInventoryCapacity(11,1,1): I_customer(11)(1)(1)#246 <= 51
 CustomerInventoryCapacity(12,1,1): I_customer(12)(1)(1)#270 <= 43
 CustomerInventoryCapacity(3,2,1): I_customer(3)(2)(1)#62 <= 59
 CustomerInventoryCapacity(4,2,1): I_customer(4)(2)(1)#86 <= 22
 CustomerInventoryCapacity(5,2,1): I_customer(5)(2)(1)#110 <= 18
 CustomerInventoryCapacity(6,2,1): I_customer(6)(2)(1)#134 <= 47
 CustomerInventoryCapacity(7,2,1): I_customer(7)(2)(1)#158 <= 20
 CustomerInventoryCapacity(8,2,1): I_customer(8)(2)(1)#182 <= 52
 CustomerInventoryCapacity(9,2,1): I_customer(9)(2)(1)#206 <= 49
 CustomerInventoryCapacity(10,2,1): I_customer(10)(2)(1)#230 <= 50
 CustomerInventoryCapacity(11,2,1): I_customer(11)(2)(1)#254 <= 51
 CustomerInventoryCapacity(12,2,1): I_customer(12)(2)(1)#278 <= 42
 CustomerInventoryCapacity(3,3,1): I_customer(3)(3)(1)#70 <= 54
 CustomerInventoryCapacity(4,3,1): I_customer(4)(3)(1)#94 <= 20
 CustomerInventoryCapacity(5,3,1): I_customer(5)(3)(1)#118 <= 18
 CustomerInventoryCapacity(6,3,1): I_customer(6)(3)(1)#142 <= 47
 CustomerInventoryCapacity(7,3,1): I_customer(7)(3)(1)#166 <= 22
 CustomerInventoryCapacity(8,3,1): I_customer(8)(3)(1)#190 <= 53
 CustomerInventoryCapacity(9,3,1): I_customer(9)(3)(1)#214 <= 47
 CustomerInventoryCapacity(10,3,1): I_customer(10)(3)(1)#238 <= 48
 CustomerInventoryCapacity(11,3,1): I_customer(11)(3)(1)#262 <= 50
 CustomerInventoryCapacity(12,3,1): I_customer(12)(3)(1)#286 <= 40
 CustomerInventoryCapacity(3,1,2): I_customer(3)(1)(2)#55 <= 54
 CustomerInventoryCapacity(4,1,2): I_customer(4)(1)(2)#79 <= 22
 CustomerInventoryCapacity(5,1,2): I_customer(5)(1)(2)#103 <= 19
 CustomerInventoryCapacity(6,1,2): I_customer(6)(1)(2)#127 <= 47
 CustomerInventoryCapacity(7,1,2): I_customer(7)(1)(2)#151 <= 22
 CustomerInventoryCapacity(8,1,2): I_customer(8)(1)(2)#175 <= 54
 CustomerInventoryCapacity(9,1,2): I_customer(9)(1)(2)#199 <= 51
 CustomerInventoryCapacity(10,1,2): I_customer(10)(1)(2)#223 <= 50
 CustomerInventoryCapacity(11,1,2): I_customer(11)(1)(2)#247 <= 45
 CustomerInventoryCapacity(12,1,2): I_customer(12)(1)(2)#271 <= 45
 CustomerInventoryCapacity(3,2,2): I_customer(3)(2)(2)#63 <= 56
 CustomerInventoryCapacity(4,2,2): I_customer(4)(2)(2)#87 <= 20
 CustomerInventoryCapacity(5,2,2): I_customer(5)(2)(2)#111 <= 17
 CustomerInventoryCapacity(6,2,2): I_customer(6)(2)(2)#135 <= 48
 CustomerInventoryCapacity(7,2,2): I_customer(7)(2)(2)#159 <= 21
 CustomerInventoryCapacity(8,2,2): I_customer(8)(2)(2)#183 <= 55
 CustomerInventoryCapacity(9,2,2): I_customer(9)(2)(2)#207 <= 50
 CustomerInventoryCapacity(10,2,2): I_customer(10)(2)(2)#231 <= 46
 CustomerInventoryCapacity(11,2,2): I_customer(11)(2)(2)#255 <= 48
 CustomerInventoryCapacity(12,2,2): I_customer(12)(2)(2)#279 <= 42
 CustomerInventoryCapacity(3,3,2): I_customer(3)(3)(2)#71 <= 60
 CustomerInventoryCapacity(4,3,2): I_customer(4)(3)(2)#95 <= 21
 CustomerInventoryCapacity(5,3,2): I_customer(5)(3)(2)#119 <= 17
 CustomerInventoryCapacity(6,3,2): I_customer(6)(3)(2)#143 <= 47
 CustomerInventoryCapacity(7,3,2): I_customer(7)(3)(2)#167 <= 22
 CustomerInventoryCapacity(8,3,2): I_customer(8)(3)(2)#191 <= 53
 CustomerInventoryCapacity(9,3,2): I_customer(9)(3)(2)#215 <= 51
 CustomerInventoryCapacity(10,3,2): I_customer(10)(3)(2)#239 <= 47
 CustomerInventoryCapacity(11,3,2): I_customer(11)(3)(2)#263 <= 46
 CustomerInventoryCapacity(12,3,2): I_customer(12)(3)(2)#287 <= 40
 CustomerInventoryCapacity(3,1,3): I_customer(3)(1)(3)#56 <= 55
 CustomerInventoryCapacity(4,1,3): I_customer(4)(1)(3)#80 <= 20
 CustomerInventoryCapacity(5,1,3): I_customer(5)(1)(3)#104 <= 18
 CustomerInventoryCapacity(6,1,3): I_customer(6)(1)(3)#128 <= 48
 CustomerInventoryCapacity(7,1,3): I_customer(7)(1)(3)#152 <= 20
 CustomerInventoryCapacity(8,1,3): I_customer(8)(1)(3)#176 <= 51
 CustomerInventoryCapacity(9,1,3): I_customer(9)(1)(3)#200 <= 50
 CustomerInventoryCapacity(10,1,3): I_customer(10)(1)(3)#224 <= 46
 CustomerInventoryCapacity(11,1,3): I_customer(11)(1)(3)#248 <= 49
 CustomerInventoryCapacity(12,1,3): I_customer(12)(1)(3)#272 <= 43
 CustomerInventoryCapacity(3,2,3): I_customer(3)(2)(3)#64 <= 56
 CustomerInventoryCapacity(4,2,3): I_customer(4)(2)(3)#88 <= 20
 CustomerInventoryCapacity(5,2,3): I_customer(5)(2)(3)#112 <= 17
 CustomerInventoryCapacity(6,2,3): I_customer(6)(2)(3)#136 <= 49
 CustomerInventoryCapacity(7,2,3): I_customer(7)(2)(3)#160 <= 21
 CustomerInventoryCapacity(8,2,3): I_customer(8)(2)(3)#184 <= 52
 CustomerInventoryCapacity(9,2,3): I_customer(9)(2)(3)#208 <= 48
 CustomerInventoryCapacity(10,2,3): I_customer(10)(2)(3)#232 <= 51
 CustomerInventoryCapacity(11,2,3): I_customer(11)(2)(3)#256 <= 46
 CustomerInventoryCapacity(12,2,3): I_customer(12)(2)(3)#280 <= 43
 CustomerInventoryCapacity(3,3,3): I_customer(3)(3)(3)#72 <= 56
 CustomerInventoryCapacity(4,3,3): I_customer(4)(3)(3)#96 <= 21
 CustomerInventoryCapacity(5,3,3): I_customer(5)(3)(3)#120 <= 19
 CustomerInventoryCapacity(6,3,3): I_customer(6)(3)(3)#144 <= 48
 CustomerInventoryCapacity(7,3,3): I_customer(7)(3)(3)#168 <= 21
 CustomerInventoryCapacity(8,3,3): I_customer(8)(3)(3)#192 <= 51
 CustomerInventoryCapacity(9,3,3): I_customer(9)(3)(3)#216 <= 45
 CustomerInventoryCapacity(10,3,3): I_customer(10)(3)(3)#240 <= 45
 CustomerInventoryCapacity(11,3,3): I_customer(11)(3)(3)#264 <= 47
 CustomerInventoryCapacity(12,3,3): I_customer(12)(3)(3)#288 <= 43
 CustomerInventoryCapacity(3,1,4): I_customer(3)(1)(4)#57 <= 55
 CustomerInventoryCapacity(4,1,4): I_customer(4)(1)(4)#81 <= 20
 CustomerInventoryCapacity(5,1,4): I_customer(5)(1)(4)#105 <= 17
 CustomerInventoryCapacity(6,1,4): I_customer(6)(1)(4)#129 <= 47
 CustomerInventoryCapacity(7,1,4): I_customer(7)(1)(4)#153 <= 20
 CustomerInventoryCapacity(8,1,4): I_customer(8)(1)(4)#177 <= 56
 CustomerInventoryCapacity(9,1,4): I_customer(9)(1)(4)#201 <= 48
 CustomerInventoryCapacity(10,1,4): I_customer(10)(1)(4)#225 <= 46
 CustomerInventoryCapacity(11,1,4): I_customer(11)(1)(4)#249 <= 49
 CustomerInventoryCapacity(12,1,4): I_customer(12)(1)(4)#273 <= 41
 CustomerInventoryCapacity(3,2,4): I_customer(3)(2)(4)#65 <= 56
 CustomerInventoryCapacity(4,2,4): I_customer(4)(2)(4)#89 <= 21
 CustomerInventoryCapacity(5,2,4): I_customer(5)(2)(4)#113 <= 18
 CustomerInventoryCapacity(6,2,4): I_customer(6)(2)(4)#137 <= 50
 CustomerInventoryCapacity(7,2,4): I_customer(7)(2)(4)#161 <= 22
 CustomerInventoryCapacity(8,2,4): I_customer(8)(2)(4)#185 <= 55
 CustomerInventoryCapacity(9,2,4): I_customer(9)(2)(4)#209 <= 47
 CustomerInventoryCapacity(10,2,4): I_customer(10)(2)(4)#233 <= 45
 CustomerInventoryCapacity(11,2,4): I_customer(11)(2)(4)#257 <= 45
 CustomerInventoryCapacity(12,2,4): I_customer(12)(2)(4)#281 <= 41
 CustomerInventoryCapacity(3,3,4): I_customer(3)(3)(4)#73 <= 55
 CustomerInventoryCapacity(4,3,4): I_customer(4)(3)(4)#97 <= 22
 CustomerInventoryCapacity(5,3,4): I_customer(5)(3)(4)#121 <= 18
 CustomerInventoryCapacity(6,3,4): I_customer(6)(3)(4)#145 <= 50
 CustomerInventoryCapacity(7,3,4): I_customer(7)(3)(4)#169 <= 22
 CustomerInventoryCapacity(8,3,4): I_customer(8)(3)(4)#193 <= 56
 CustomerInventoryCapacity(9,3,4): I_customer(9)(3)(4)#217 <= 48
 CustomerInventoryCapacity(10,3,4): I_customer(10)(3)(4)#241 <= 46
 CustomerInventoryCapacity(11,3,4): I_customer(11)(3)(4)#265 <= 50
 CustomerInventoryCapacity(12,3,4): I_customer(12)(3)(4)#289 <= 44
 CustomerInventoryCapacity(3,1,5): I_customer(3)(1)(5)#58 <= 58
 CustomerInventoryCapacity(4,1,5): I_customer(4)(1)(5)#82 <= 20
 CustomerInventoryCapacity(5,1,5): I_customer(5)(1)(5)#106 <= 17
 CustomerInventoryCapacity(6,1,5): I_customer(6)(1)(5)#130 <= 47
 CustomerInventoryCapacity(7,1,5): I_customer(7)(1)(5)#154 <= 20
 CustomerInventoryCapacity(8,1,5): I_customer(8)(1)(5)#178 <= 51
 CustomerInventoryCapacity(9,1,5): I_customer(9)(1)(5)#202 <= 45
 CustomerInventoryCapacity(10,1,5): I_customer(10)(1)(5)#226 <= 51
 CustomerInventoryCapacity(11,1,5): I_customer(11)(1)(5)#250 <= 45
 CustomerInventoryCapacity(12,1,5): I_customer(12)(1)(5)#274 <= 42
 CustomerInventoryCapacity(3,2,5): I_customer(3)(2)(5)#66 <= 54
 CustomerInventoryCapacity(4,2,5): I_customer(4)(2)(5)#90 <= 20
 CustomerInventoryCapacity(5,2,5): I_customer(5)(2)(5)#114 <= 19
 CustomerInventoryCapacity(6,2,5): I_customer(6)(2)(5)#138 <= 49
 CustomerInventoryCapacity(7,2,5): I_customer(7)(2)(5)#162 <= 22
 CustomerInventoryCapacity(8,2,5): I_customer(8)(2)(5)#186 <= 57
 CustomerInventoryCapacity(9,2,5): I_customer(9)(2)(5)#210 <= 45
 CustomerInventoryCapacity(10,2,5): I_customer(10)(2)(5)#234 <= 48
 CustomerInventoryCapacity(11,2,5): I_customer(11)(2)(5)#258 <= 46
 CustomerInventoryCapacity(12,2,5): I_customer(12)(2)(5)#282 <= 44
 CustomerInventoryCapacity(3,3,5): I_customer(3)(3)(5)#74 <= 59
 CustomerInventoryCapacity(4,3,5): I_customer(4)(3)(5)#98 <= 21
 CustomerInventoryCapacity(5,3,5): I_customer(5)(3)(5)#122 <= 18
 CustomerInventoryCapacity(6,3,5): I_customer(6)(3)(5)#146 <= 46
 CustomerInventoryCapacity(7,3,5): I_customer(7)(3)(5)#170 <= 21
 CustomerInventoryCapacity(8,3,5): I_customer(8)(3)(5)#194 <= 57
 CustomerInventoryCapacity(9,3,5): I_customer(9)(3)(5)#218 <= 47
 CustomerInventoryCapacity(10,3,5): I_customer(10)(3)(5)#242 <= 50
 CustomerInventoryCapacity(11,3,5): I_customer(11)(3)(5)#266 <= 50
 CustomerInventoryCapacity(12,3,5): I_customer(12)(3)(5)#290 <= 44
 CustomerInventoryCapacity(3,1,6): I_customer(3)(1)(6)#59 <= 54
 CustomerInventoryCapacity(4,1,6): I_customer(4)(1)(6)#83 <= 22
 CustomerInventoryCapacity(5,1,6): I_customer(5)(1)(6)#107 <= 18
 CustomerInventoryCapacity(6,1,6): I_customer(6)(1)(6)#131 <= 47
 CustomerInventoryCapacity(7,1,6): I_customer(7)(1)(6)#155 <= 21
 CustomerInventoryCapacity(8,1,6): I_customer(8)(1)(6)#179 <= 55
 CustomerInventoryCapacity(9,1,6): I_customer(9)(1)(6)#203 <= 45
 CustomerInventoryCapacity(10,1,6): I_customer(10)(1)(6)#227 <= 48
 CustomerInventoryCapacity(11,1,6): I_customer(11)(1)(6)#251 <= 50
 CustomerInventoryCapacity(12,1,6): I_customer(12)(1)(6)#275 <= 44
 CustomerInventoryCapacity(3,2,6): I_customer(3)(2)(6)#67 <= 57
 CustomerInventoryCapacity(4,2,6): I_customer(4)(2)(6)#91 <= 22
 CustomerInventoryCapacity(5,2,6): I_customer(5)(2)(6)#115 <= 17
 CustomerInventoryCapacity(6,2,6): I_customer(6)(2)(6)#139 <= 49
 CustomerInventoryCapacity(7,2,6): I_customer(7)(2)(6)#163 <= 22
 CustomerInventoryCapacity(8,2,6): I_customer(8)(2)(6)#187 <= 51
 CustomerInventoryCapacity(9,2,6): I_customer(9)(2)(6)#211 <= 50
 CustomerInventoryCapacity(10,2,6): I_customer(10)(2)(6)#235 <= 46
 CustomerInventoryCapacity(11,2,6): I_customer(11)(2)(6)#259 <= 51
 CustomerInventoryCapacity(12,2,6): I_customer(12)(2)(6)#283 <= 44
 CustomerInventoryCapacity(3,3,6): I_customer(3)(3)(6)#75 <= 58
 CustomerInventoryCapacity(4,3,6): I_customer(4)(3)(6)#99 <= 21
 CustomerInventoryCapacity(5,3,6): I_customer(5)(3)(6)#123 <= 19
 CustomerInventoryCapacity(6,3,6): I_customer(6)(3)(6)#147 <= 51
 CustomerInventoryCapacity(7,3,6): I_customer(7)(3)(6)#171 <= 21
 CustomerInventoryCapacity(8,3,6): I_customer(8)(3)(6)#195 <= 55
 CustomerInventoryCapacity(9,3,6): I_customer(9)(3)(6)#219 <= 45
 CustomerInventoryCapacity(10,3,6): I_customer(10)(3)(6)#243 <= 46
 CustomerInventoryCapacity(11,3,6): I_customer(11)(3)(6)#267 <= 50
 CustomerInventoryCapacity(12,3,6): I_customer(12)(3)(6)#291 <= 40
 CustomerInventoryCapacity(3,1,7): I_customer(3)(1)(7)#60 <= 53
 CustomerInventoryCapacity(4,1,7): I_customer(4)(1)(7)#84 <= 22
 CustomerInventoryCapacity(5,1,7): I_customer(5)(1)(7)#108 <= 18
 CustomerInventoryCapacity(6,1,7): I_customer(6)(1)(7)#132 <= 47
 CustomerInventoryCapacity(7,1,7): I_customer(7)(1)(7)#156 <= 22
 CustomerInventoryCapacity(8,1,7): I_customer(8)(1)(7)#180 <= 52
 CustomerInventoryCapacity(9,1,7): I_customer(9)(1)(7)#204 <= 51
 CustomerInventoryCapacity(10,1,7): I_customer(10)(1)(7)#228 <= 48
 CustomerInventoryCapacity(11,1,7): I_customer(11)(1)(7)#252 <= 47
 CustomerInventoryCapacity(12,1,7): I_customer(12)(1)(7)#276 <= 44
 CustomerInventoryCapacity(3,2,7): I_customer(3)(2)(7)#68 <= 55
 CustomerInventoryCapacity(4,2,7): I_customer(4)(2)(7)#92 <= 20
 CustomerInventoryCapacity(5,2,7): I_customer(5)(2)(7)#116 <= 19
 CustomerInventoryCapacity(6,2,7): I_customer(6)(2)(7)#140 <= 49
 CustomerInventoryCapacity(7,2,7): I_customer(7)(2)(7)#164 <= 21
 CustomerInventoryCapacity(8,2,7): I_customer(8)(2)(7)#188 <= 52
 CustomerInventoryCapacity(9,2,7): I_customer(9)(2)(7)#212 <= 49
 CustomerInventoryCapacity(10,2,7): I_customer(10)(2)(7)#236 <= 47
 CustomerInventoryCapacity(11,2,7): I_customer(11)(2)(7)#260 <= 47
 CustomerInventoryCapacity(12,2,7): I_customer(12)(2)(7)#284 <= 42
 CustomerInventoryCapacity(3,3,7): I_customer(3)(3)(7)#76 <= 57
 CustomerInventoryCapacity(4,3,7): I_customer(4)(3)(7)#100 <= 21
 CustomerInventoryCapacity(5,3,7): I_customer(5)(3)(7)#124 <= 19
 CustomerInventoryCapacity(6,3,7): I_customer(6)(3)(7)#148 <= 47
 CustomerInventoryCapacity(7,3,7): I_customer(7)(3)(7)#172 <= 21
 CustomerInventoryCapacity(8,3,7): I_customer(8)(3)(7)#196 <= 56
 CustomerInventoryCapacity(9,3,7): I_customer(9)(3)(7)#220 <= 45
 CustomerInventoryCapacity(10,3,7): I_customer(10)(3)(7)#244 <= 46
 CustomerInventoryCapacity(11,3,7): I_customer(11)(3)(7)#268 <= 47
 CustomerInventoryCapacity(12,3,7): I_customer(12)(3)(7)#292 <= 40
 CustomerInventoryCapacity(3,1,8): I_customer(3)(1)(8)#61 <= 57
 CustomerInventoryCapacity(4,1,8): I_customer(4)(1)(8)#85 <= 22
 CustomerInventoryCapacity(5,1,8): I_customer(5)(1)(8)#109 <= 19
 CustomerInventoryCapacity(6,1,8): I_customer(6)(1)(8)#133 <= 47
 CustomerInventoryCapacity(7,1,8): I_customer(7)(1)(8)#157 <= 20
 CustomerInventoryCapacity(8,1,8): I_customer(8)(1)(8)#181 <= 57
 CustomerInventoryCapacity(9,1,8): I_customer(9)(1)(8)#205 <= 49
 CustomerInventoryCapacity(10,1,8): I_customer(10)(1)(8)#229 <= 51
 CustomerInventoryCapacity(11,1,8): I_customer(11)(1)(8)#253 <= 45
 CustomerInventoryCapacity(12,1,8): I_customer(12)(1)(8)#277 <= 44
 CustomerInventoryCapacity(3,2,8): I_customer(3)(2)(8)#69 <= 57
 CustomerInventoryCapacity(4,2,8): I_customer(4)(2)(8)#93 <= 22
 CustomerInventoryCapacity(5,2,8): I_customer(5)(2)(8)#117 <= 19
 CustomerInventoryCapacity(6,2,8): I_customer(6)(2)(8)#141 <= 47
 CustomerInventoryCapacity(7,2,8): I_customer(7)(2)(8)#165 <= 21
 CustomerInventoryCapacity(8,2,8): I_customer(8)(2)(8)#189 <= 53
 CustomerInventoryCapacity(9,2,8): I_customer(9)(2)(8)#213 <= 49
 CustomerInventoryCapacity(10,2,8): I_customer(10)(2)(8)#237 <= 51
 CustomerInventoryCapacity(11,2,8): I_customer(11)(2)(8)#261 <= 50
 CustomerInventoryCapacity(12,2,8): I_customer(12)(2)(8)#285 <= 39
 CustomerInventoryCapacity(3,3,8): I_customer(3)(3)(8)#77 <= 61
 CustomerInventoryCapacity(4,3,8): I_customer(4)(3)(8)#101 <= 21
 CustomerInventoryCapacity(5,3,8): I_customer(5)(3)(8)#125 <= 17
 CustomerInventoryCapacity(6,3,8): I_customer(6)(3)(8)#149 <= 48
 CustomerInventoryCapacity(7,3,8): I_customer(7)(3)(8)#173 <= 21
 CustomerInventoryCapacity(8,3,8): I_customer(8)(3)(8)#197 <= 51
 CustomerInventoryCapacity(9,3,8): I_customer(9)(3)(8)#221 <= 46
 CustomerInventoryCapacity(10,3,8): I_customer(10)(3)(8)#245 <= 47
 CustomerInventoryCapacity(11,3,8): I_customer(11)(3)(8)#269 <= 48
 CustomerInventoryCapacity(12,3,8): I_customer(12)(3)(8)#293 <= 42
 WarehouseDelivery(1,1,1):       q(1)(1)(1)#534 <= 124
 WarehouseDelivery(2,1,1):       q(2)(1)(1)#537 <= 124
 WarehouseDelivery(1,1,2):       q(1)(1)(2)#535  = 0
 WarehouseDelivery(2,1,2):       q(2)(1)(2)#538  = 0
 WarehouseDelivery(1,1,3):       q(1)(1)(3)#536 <= 124
 WarehouseDelivery(2,1,3):       q(2)(1)(3)#539  = 0
 VehicleCapacityPlant(1,1):      q(1)(1)(1)#534 + q(2)(1)(1)#537 <= 570
 VehicleCapacityPlant(1,2):      q(1)(1)(2)#535 + q(2)(1)(2)#538 <= 570
 VehicleCapacityPlant(1,3):      q(1)(1)(3)#536 + q(2)(1)(3)#539 <= 570
 CustomerVisit(3,1,1,1):         w(3)(1)(1)(1)#540  = 0
 CustomerVisit(4,1,1,1):         w(4)(1)(1)(1)#588  = 0
 CustomerVisit(5,1,1,1):         w(5)(1)(1)(1)#636  = 0
 CustomerVisit(6,1,1,1):         w(6)(1)(1)(1)#684  = 0
 CustomerVisit(7,1,1,1):         w(7)(1)(1)(1)#732  = 0
 CustomerVisit(8,1,1,1):         w(8)(1)(1)(1)#780  = 0
 CustomerVisit(12,1,1,1):        w(12)(1)(1)(1)#972  = 0
 CustomerVisit(3,1,2,1):         w(3)(1)(2)(1)#548  = 0
 CustomerVisit(4,1,2,1):         w(4)(1)(2)(1)#596  = 0
 CustomerVisit(5,1,2,1):         w(5)(1)(2)(1)#644  = 0
 CustomerVisit(6,1,2,1):         w(6)(1)(2)(1)#692  = 0
 CustomerVisit(7,1,2,1):         w(7)(1)(2)(1)#740  = 0
 CustomerVisit(8,1,2,1):         w(8)(1)(2)(1)#788  = 0
 CustomerVisit(9,1,2,1):         w(9)(1)(2)(1)#836  = 0
 CustomerVisit(10,1,2,1):        w(10)(1)(2)(1)#884  = 0
 CustomerVisit(11,1,2,1):        w(11)(1)(2)(1)#932  = 0
 CustomerVisit(12,1,2,1):        w(12)(1)(2)(1)#980  = 0
 CustomerVisit(7,1,3,1):         w(7)(1)(3)(1)#748  = 0
 CustomerVisit(3,2,1,1):         w(3)(2)(1)(1)#564  = 0
 CustomerVisit(5,2,1,1):         w(5)(2)(1)(1)#660  = 0
 CustomerVisit(6,2,1,1):         w(6)(2)(1)(1)#708  = 0
 CustomerVisit(7,2,1,1):         w(7)(2)(1)(1)#756  = 0
 CustomerVisit(8,2,1,1):         w(8)(2)(1)(1)#804  = 0
 CustomerVisit(9,2,1,1):         w(9)(2)(1)(1)#852  = 0
 CustomerVisit(10,2,1,1):        w(10)(2)(1)(1)#900  = 0
 CustomerVisit(11,2,1,1):        w(11)(2)(1)(1)#948  = 0
 CustomerVisit(12,2,1,1):        w(12)(2)(1)(1)#996  = 0
 CustomerVisit(3,2,2,1):         w(3)(2)(2)(1)#572  = 0
 CustomerVisit(9,2,2,1):         w(9)(2)(2)(1)#860  = 0
 CustomerVisit(10,2,2,1):        w(10)(2)(2)(1)#908  = 0
 CustomerVisit(11,2,2,1):        w(11)(2)(2)(1)#956  = 0
 CustomerVisit(12,2,2,1):        w(12)(2)(2)(1)#1004  = 0
 CustomerVisit(3,2,3,1):         w(3)(2)(3)(1)#580  = 0
 CustomerVisit(4,2,3,1):         w(4)(2)(3)(1)#628  = 0
 CustomerVisit(5,2,3,1):         w(5)(2)(3)(1)#676  = 0
 CustomerVisit(6,2,3,1):         w(6)(2)(3)(1)#724  = 0
 CustomerVisit(7,2,3,1):         w(7)(2)(3)(1)#772  = 0
 CustomerVisit(8,2,3,1):         w(8)(2)(3)(1)#820  = 0
 CustomerVisit(9,2,3,1):         w(9)(2)(3)(1)#868  = 0
 CustomerVisit(10,2,3,1):        w(10)(2)(3)(1)#916  = 0
 CustomerVisit(11,2,3,1):        w(11)(2)(3)(1)#964  = 0
 CustomerVisit(12,2,3,1):        w(12)(2)(3)(1)#1012  = 0
 CustomerVisit(3,1,1,2):         w(3)(1)(1)(2)#541  = 0
 CustomerVisit(4,1,1,2):         w(4)(1)(1)(2)#589  = 0
 CustomerVisit(5,1,1,2):         w(5)(1)(1)(2)#637  = 0
 CustomerVisit(6,1,1,2):         w(6)(1)(1)(2)#685  = 0
 CustomerVisit(7,1,1,2):         w(7)(1)(1)(2)#733  = 0
 CustomerVisit(8,1,1,2):         w(8)(1)(1)(2)#781  = 0
 CustomerVisit(9,1,1,2):         w(9)(1)(1)(2)#829  = 0
 CustomerVisit(10,1,1,2):        w(10)(1)(1)(2)#877  = 0
 CustomerVisit(12,1,1,2):        w(12)(1)(1)(2)#973  = 0
 CustomerVisit(9,1,2,2):         w(9)(1)(2)(2)#837  = 0
 CustomerVisit(11,1,2,2):        w(11)(1)(2)(2)#933  = 0
 CustomerVisit(12,1,2,2):        w(12)(1)(2)(2)#981  = 0
 CustomerVisit(7,1,3,2):         w(7)(1)(3)(2)#749  = 0
 CustomerVisit(3,2,1,2):         w(3)(2)(1)(2)#565  = 0
 CustomerVisit(4,2,1,2):         w(4)(2)(1)(2)#613  = 0
 CustomerVisit(5,2,1,2):         w(5)(2)(1)(2)#661  = 0
 CustomerVisit(6,2,1,2):         w(6)(2)(1)(2)#709  = 0
 CustomerVisit(7,2,1,2):         w(7)(2)(1)(2)#757  = 0
 CustomerVisit(8,2,1,2):         w(8)(2)(1)(2)#805  = 0
 CustomerVisit(9,2,1,2):         w(9)(2)(1)(2)#853  = 0
 CustomerVisit(10,2,1,2):        w(10)(2)(1)(2)#901  = 0
 CustomerVisit(11,2,1,2):        w(11)(2)(1)(2)#949  = 0
 CustomerVisit(12,2,1,2):        w(12)(2)(1)(2)#997  = 0
 CustomerVisit(3,2,2,2):         w(3)(2)(2)(2)#573  = 0
 CustomerVisit(4,2,2,2):         w(4)(2)(2)(2)#621  = 0
 CustomerVisit(5,2,2,2):         w(5)(2)(2)(2)#669  = 0
 CustomerVisit(6,2,2,2):         w(6)(2)(2)(2)#717  = 0
 CustomerVisit(7,2,2,2):         w(7)(2)(2)(2)#765  = 0
 CustomerVisit(8,2,2,2):         w(8)(2)(2)(2)#813  = 0
 CustomerVisit(9,2,2,2):         w(9)(2)(2)(2)#861  = 0
 CustomerVisit(10,2,2,2):        w(10)(2)(2)(2)#909  = 0
 CustomerVisit(11,2,2,2):        w(11)(2)(2)(2)#957  = 0
 CustomerVisit(12,2,2,2):        w(12)(2)(2)(2)#1005  = 0
 CustomerVisit(3,2,3,2):         w(3)(2)(3)(2)#581  = 0
 CustomerVisit(4,2,3,2):         w(4)(2)(3)(2)#629  = 0
 CustomerVisit(5,2,3,2):         w(5)(2)(3)(2)#677  = 0
 CustomerVisit(6,2,3,2):         w(6)(2)(3)(2)#725  = 0
 CustomerVisit(7,2,3,2):         w(7)(2)(3)(2)#773  = 0
 CustomerVisit(8,2,3,2):         w(8)(2)(3)(2)#821  = 0
 CustomerVisit(9,2,3,2):         w(9)(2)(3)(2)#869  = 0
 CustomerVisit(10,2,3,2):        w(10)(2)(3)(2)#917  = 0
 CustomerVisit(11,2,3,2):        w(11)(2)(3)(2)#965  = 0
 CustomerVisit(12,2,3,2):        w(12)(2)(3)(2)#1013  = 0
 CustomerVisit(9,1,1,3):         w(9)(1)(1)(3)#830  = 0
 CustomerVisit(11,1,1,3):        w(11)(1)(1)(3)#926  = 0
 CustomerVisit(3,1,2,3):         w(3)(1)(2)(3)#550  = 0
 CustomerVisit(4,1,2,3):         w(4)(1)(2)(3)#598  = 0
 CustomerVisit(5,1,2,3):         w(5)(1)(2)(3)#646  = 0
 CustomerVisit(6,1,2,3):         w(6)(1)(2)(3)#694  = 0
 CustomerVisit(7,1,2,3):         w(7)(1)(2)(3)#742  = 0
 CustomerVisit(9,1,2,3):         w(9)(1)(2)(3)#838  = 0
 CustomerVisit(12,1,2,3):        w(12)(1)(2)(3)#982  = 0
 CustomerVisit(3,1,3,3):         w(3)(1)(3)(3)#558  = 0
 CustomerVisit(4,1,3,3):         w(4)(1)(3)(3)#606  = 0
 CustomerVisit(5,1,3,3):         w(5)(1)(3)(3)#654  = 0
 CustomerVisit(6,1,3,3):         w(6)(1)(3)(3)#702  = 0
 CustomerVisit(7,1,3,3):         w(7)(1)(3)(3)#750  = 0
 CustomerVisit(12,1,3,3):        w(12)(1)(3)(3)#990  = 0
 CustomerVisit(3,2,1,3):         w(3)(2)(1)(3)#566  = 0
 CustomerVisit(4,2,1,3):         w(4)(2)(1)(3)#614  = 0
 CustomerVisit(5,2,1,3):         w(5)(2)(1)(3)#662  = 0
 CustomerVisit(6,2,1,3):         w(6)(2)(1)(3)#710  = 0
 CustomerVisit(7,2,1,3):         w(7)(2)(1)(3)#758  = 0
 CustomerVisit(8,2,1,3):         w(8)(2)(1)(3)#806  = 0
 CustomerVisit(9,2,1,3):         w(9)(2)(1)(3)#854  = 0
 CustomerVisit(10,2,1,3):        w(10)(2)(1)(3)#902  = 0
 CustomerVisit(11,2,1,3):        w(11)(2)(1)(3)#950  = 0
 CustomerVisit(12,2,1,3):        w(12)(2)(1)(3)#998  = 0
 CustomerVisit(7,2,2,3):         w(7)(2)(2)(3)#766  = 0
 CustomerVisit(8,2,2,3):         w(8)(2)(2)(3)#814  = 0
 CustomerVisit(9,2,2,3):         w(9)(2)(2)(3)#862  = 0
 CustomerVisit(10,2,2,3):        w(10)(2)(2)(3)#910  = 0
 CustomerVisit(11,2,2,3):        w(11)(2)(2)(3)#958  = 0
 CustomerVisit(12,2,2,3):        w(12)(2)(2)(3)#1006  = 0
 CustomerVisit(3,2,3,3):         w(3)(2)(3)(3)#582  = 0
 CustomerVisit(4,2,3,3):         w(4)(2)(3)(3)#630  = 0
 CustomerVisit(5,2,3,3):         w(5)(2)(3)(3)#678  = 0
 CustomerVisit(6,2,3,3):         w(6)(2)(3)(3)#726  = 0
 CustomerVisit(7,2,3,3):         w(7)(2)(3)(3)#774  = 0
 CustomerVisit(8,2,3,3):         w(8)(2)(3)(3)#822  = 0
 CustomerVisit(9,2,3,3):         w(9)(2)(3)(3)#870  = 0
 CustomerVisit(10,2,3,3):        w(10)(2)(3)(3)#918  = 0
 CustomerVisit(11,2,3,3):        w(11)(2)(3)(3)#966  = 0
 CustomerVisit(12,2,3,3):        w(12)(2)(3)(3)#1014  = 0
 CustomerVisit(3,1,1,4):         w(3)(1)(1)(4)#543  = 0
 CustomerVisit(4,1,1,4):         w(4)(1)(1)(4)#591  = 0
 CustomerVisit(5,1,1,4):         w(5)(1)(1)(4)#639  = 0
 CustomerVisit(6,1,1,4):         w(6)(1)(1)(4)#687  = 0
 CustomerVisit(7,1,1,4):         w(7)(1)(1)(4)#735  = 0
 CustomerVisit(8,1,1,4):         w(8)(1)(1)(4)#783  = 0
 CustomerVisit(9,1,1,4):         w(9)(1)(1)(4)#831  = 0
 CustomerVisit(11,1,1,4):        w(11)(1)(1)(4)#927  = 0
 CustomerVisit(12,1,1,4):        w(12)(1)(1)(4)#975  = 0
 CustomerVisit(3,1,3,4):         w(3)(1)(3)(4)#559  = 0
 CustomerVisit(4,1,3,4):         w(4)(1)(3)(4)#607  = 0
 CustomerVisit(5,1,3,4):         w(5)(1)(3)(4)#655  = 0
 CustomerVisit(6,1,3,4):         w(6)(1)(3)(4)#703  = 0
 CustomerVisit(7,1,3,4):         w(7)(1)(3)(4)#751  = 0
 CustomerVisit(12,1,3,4):        w(12)(1)(3)(4)#991  = 0
 CustomerVisit(7,2,1,4):         w(7)(2)(1)(4)#759  = 0
 CustomerVisit(8,2,1,4):         w(8)(2)(1)(4)#807  = 0
 CustomerVisit(9,2,1,4):         w(9)(2)(1)(4)#855  = 0
 CustomerVisit(10,2,1,4):        w(10)(2)(1)(4)#903  = 0
 CustomerVisit(11,2,1,4):        w(11)(2)(1)(4)#951  = 0
 CustomerVisit(12,2,1,4):        w(12)(2)(1)(4)#999  = 0
 CustomerVisit(3,2,2,4):         w(3)(2)(2)(4)#575  = 0
 CustomerVisit(4,2,2,4):         w(4)(2)(2)(4)#623  = 0
 CustomerVisit(5,2,2,4):         w(5)(2)(2)(4)#671  = 0
 CustomerVisit(6,2,2,4):         w(6)(2)(2)(4)#719  = 0
 CustomerVisit(7,2,2,4):         w(7)(2)(2)(4)#767  = 0
 CustomerVisit(8,2,2,4):         w(8)(2)(2)(4)#815  = 0
 CustomerVisit(9,2,2,4):         w(9)(2)(2)(4)#863  = 0
 CustomerVisit(10,2,2,4):        w(10)(2)(2)(4)#911  = 0
 CustomerVisit(11,2,2,4):        w(11)(2)(2)(4)#959  = 0
 CustomerVisit(12,2,2,4):        w(12)(2)(2)(4)#1007  = 0
 CustomerVisit(3,2,3,4):         w(3)(2)(3)(4)#583  = 0
 CustomerVisit(4,2,3,4):         w(4)(2)(3)(4)#631  = 0
 CustomerVisit(5,2,3,4):         w(5)(2)(3)(4)#679  = 0
 CustomerVisit(6,2,3,4):         w(6)(2)(3)(4)#727  = 0
 CustomerVisit(7,2,3,4):         w(7)(2)(3)(4)#775  = 0
 CustomerVisit(8,2,3,4):         w(8)(2)(3)(4)#823  = 0
 CustomerVisit(9,2,3,4):         w(9)(2)(3)(4)#871  = 0
 CustomerVisit(10,2,3,4):        w(10)(2)(3)(4)#919  = 0
 CustomerVisit(11,2,3,4):        w(11)(2)(3)(4)#967  = 0
 CustomerVisit(12,2,3,4):        w(12)(2)(3)(4)#1015  = 0
 CustomerVisit(3,1,1,5):         w(3)(1)(1)(5)#544  = 0
 CustomerVisit(4,1,1,5):         w(4)(1)(1)(5)#592  = 0
 CustomerVisit(5,1,1,5):         w(5)(1)(1)(5)#640  = 0
 CustomerVisit(6,1,1,5):         w(6)(1)(1)(5)#688  = 0
 CustomerVisit(7,1,1,5):         w(7)(1)(1)(5)#736  = 0
 CustomerVisit(12,1,1,5):        w(12)(1)(1)(5)#976  = 0
 CustomerVisit(3,1,2,5):         w(3)(1)(2)(5)#552  = 0
 CustomerVisit(4,1,2,5):         w(4)(1)(2)(5)#600  = 0
 CustomerVisit(5,1,2,5):         w(5)(1)(2)(5)#648  = 0
 CustomerVisit(6,1,2,5):         w(6)(1)(2)(5)#696  = 0
 CustomerVisit(7,1,2,5):         w(7)(1)(2)(5)#744  = 0
 CustomerVisit(8,1,2,5):         w(8)(1)(2)(5)#792  = 0
 CustomerVisit(12,1,2,5):        w(12)(1)(2)(5)#984  = 0
 CustomerVisit(3,1,3,5):         w(3)(1)(3)(5)#560  = 0
 CustomerVisit(4,1,3,5):         w(4)(1)(3)(5)#608  = 0
 CustomerVisit(5,1,3,5):         w(5)(1)(3)(5)#656  = 0
 CustomerVisit(9,1,3,5):         w(9)(1)(3)(5)#848  = 0
 CustomerVisit(10,1,3,5):        w(10)(1)(3)(5)#896  = 0
 CustomerVisit(11,1,3,5):        w(11)(1)(3)(5)#944  = 0
 CustomerVisit(3,2,1,5):         w(3)(2)(1)(5)#568  = 0
 CustomerVisit(7,2,1,5):         w(7)(2)(1)(5)#760  = 0
 CustomerVisit(8,2,1,5):         w(8)(2)(1)(5)#808  = 0
 CustomerVisit(9,2,1,5):         w(9)(2)(1)(5)#856  = 0
 CustomerVisit(10,2,1,5):        w(10)(2)(1)(5)#904  = 0
 CustomerVisit(11,2,1,5):        w(11)(2)(1)(5)#952  = 0
 CustomerVisit(12,2,1,5):        w(12)(2)(1)(5)#1000  = 0
 CustomerVisit(6,2,2,5):         w(6)(2)(2)(5)#720  = 0
 CustomerVisit(7,2,2,5):         w(7)(2)(2)(5)#768  = 0
 CustomerVisit(8,2,2,5):         w(8)(2)(2)(5)#816  = 0
 CustomerVisit(9,2,2,5):         w(9)(2)(2)(5)#864  = 0
 CustomerVisit(10,2,2,5):        w(10)(2)(2)(5)#912  = 0
 CustomerVisit(11,2,2,5):        w(11)(2)(2)(5)#960  = 0
 CustomerVisit(12,2,2,5):        w(12)(2)(2)(5)#1008  = 0
 CustomerVisit(3,2,3,5):         w(3)(2)(3)(5)#584  = 0
 CustomerVisit(4,2,3,5):         w(4)(2)(3)(5)#632  = 0
 CustomerVisit(5,2,3,5):         w(5)(2)(3)(5)#680  = 0
 CustomerVisit(6,2,3,5):         w(6)(2)(3)(5)#728  = 0
 CustomerVisit(7,2,3,5):         w(7)(2)(3)(5)#776  = 0
 CustomerVisit(8,2,3,5):         w(8)(2)(3)(5)#824  = 0
 CustomerVisit(9,2,3,5):         w(9)(2)(3)(5)#872  = 0
 CustomerVisit(10,2,3,5):        w(10)(2)(3)(5)#920  = 0
 CustomerVisit(11,2,3,5):        w(11)(2)(3)(5)#968  = 0
 CustomerVisit(12,2,3,5):        w(12)(2)(3)(5)#1016  = 0
 CustomerVisit(3,1,1,6):         w(3)(1)(1)(6)#545  = 0
 CustomerVisit(4,1,1,6):         w(4)(1)(1)(6)#593  = 0
 CustomerVisit(5,1,1,6):         w(5)(1)(1)(6)#641  = 0
 CustomerVisit(6,1,1,6):         w(6)(1)(1)(6)#689  = 0
 CustomerVisit(7,1,1,6):         w(7)(1)(1)(6)#737  = 0
 CustomerVisit(8,1,1,6):         w(8)(1)(1)(6)#785  = 0
 CustomerVisit(9,1,1,6):         w(9)(1)(1)(6)#833  = 0
 CustomerVisit(10,1,1,6):        w(10)(1)(1)(6)#881  = 0
 CustomerVisit(11,1,1,6):        w(11)(1)(1)(6)#929  = 0
 CustomerVisit(12,1,1,6):        w(12)(1)(1)(6)#977  = 0
 CustomerVisit(7,1,2,6):         w(7)(1)(2)(6)#745  = 0
 CustomerVisit(12,1,2,6):        w(12)(1)(2)(6)#985  = 0
 CustomerVisit(3,2,1,6):         w(3)(2)(1)(6)#569  = 0
 CustomerVisit(4,2,1,6):         w(4)(2)(1)(6)#617  = 0
 CustomerVisit(5,2,1,6):         w(5)(2)(1)(6)#665  = 0
 CustomerVisit(6,2,1,6):         w(6)(2)(1)(6)#713  = 0
 CustomerVisit(7,2,1,6):         w(7)(2)(1)(6)#761  = 0
 CustomerVisit(8,2,1,6):         w(8)(2)(1)(6)#809  = 0
 CustomerVisit(9,2,1,6):         w(9)(2)(1)(6)#857  = 0
 CustomerVisit(10,2,1,6):        w(10)(2)(1)(6)#905  = 0
 CustomerVisit(11,2,1,6):        w(11)(2)(1)(6)#953  = 0
 CustomerVisit(12,2,1,6):        w(12)(2)(1)(6)#1001  = 0
 CustomerVisit(3,2,2,6):         w(3)(2)(2)(6)#577  = 0
 CustomerVisit(4,2,2,6):         w(4)(2)(2)(6)#625  = 0
 CustomerVisit(5,2,2,6):         w(5)(2)(2)(6)#673  = 0
 CustomerVisit(6,2,2,6):         w(6)(2)(2)(6)#721  = 0
 CustomerVisit(7,2,2,6):         w(7)(2)(2)(6)#769  = 0
 CustomerVisit(8,2,2,6):         w(8)(2)(2)(6)#817  = 0
 CustomerVisit(9,2,2,6):         w(9)(2)(2)(6)#865  = 0
 CustomerVisit(10,2,2,6):        w(10)(2)(2)(6)#913  = 0
 CustomerVisit(11,2,2,6):        w(11)(2)(2)(6)#961  = 0
 CustomerVisit(12,2,2,6):        w(12)(2)(2)(6)#1009  = 0
 CustomerVisit(3,2,3,6):         w(3)(2)(3)(6)#585  = 0
 CustomerVisit(4,2,3,6):         w(4)(2)(3)(6)#633  = 0
 CustomerVisit(5,2,3,6):         w(5)(2)(3)(6)#681  = 0
 CustomerVisit(6,2,3,6):         w(6)(2)(3)(6)#729  = 0
 CustomerVisit(7,2,3,6):         w(7)(2)(3)(6)#777  = 0
 CustomerVisit(8,2,3,6):         w(8)(2)(3)(6)#825  = 0
 CustomerVisit(9,2,3,6):         w(9)(2)(3)(6)#873  = 0
 CustomerVisit(10,2,3,6):        w(10)(2)(3)(6)#921  = 0
 CustomerVisit(11,2,3,6):        w(11)(2)(3)(6)#969  = 0
 CustomerVisit(12,2,3,6):        w(12)(2)(3)(6)#1017  = 0
 CustomerVisit(3,1,1,7):         w(3)(1)(1)(7)#546  = 0
 CustomerVisit(4,1,1,7):         w(4)(1)(1)(7)#594  = 0
 CustomerVisit(5,1,1,7):         w(5)(1)(1)(7)#642  = 0
 CustomerVisit(6,1,1,7):         w(6)(1)(1)(7)#690  = 0
 CustomerVisit(7,1,1,7):         w(7)(1)(1)(7)#738  = 0
 CustomerVisit(8,1,1,7):         w(8)(1)(1)(7)#786  = 0
 CustomerVisit(9,1,1,7):         w(9)(1)(1)(7)#834  = 0
 CustomerVisit(10,1,1,7):        w(10)(1)(1)(7)#882  = 0
 CustomerVisit(11,1,1,7):        w(11)(1)(1)(7)#930  = 0
 CustomerVisit(12,1,1,7):        w(12)(1)(1)(7)#978  = 0
 CustomerVisit(6,1,2,7):         w(6)(1)(2)(7)#698  = 0
 CustomerVisit(9,1,2,7):         w(9)(1)(2)(7)#842  = 0
 CustomerVisit(3,1,3,7):         w(3)(1)(3)(7)#562  = 0
 CustomerVisit(4,1,3,7):         w(4)(1)(3)(7)#610  = 0
 CustomerVisit(5,1,3,7):         w(5)(1)(3)(7)#658  = 0
 CustomerVisit(6,1,3,7):         w(6)(1)(3)(7)#706  = 0
 CustomerVisit(7,1,3,7):         w(7)(1)(3)(7)#754  = 0
 CustomerVisit(8,1,3,7):         w(8)(1)(3)(7)#802  = 0
 CustomerVisit(12,1,3,7):        w(12)(1)(3)(7)#994  = 0
 CustomerVisit(4,2,1,7):         w(4)(2)(1)(7)#618  = 0
 CustomerVisit(7,2,1,7):         w(7)(2)(1)(7)#762  = 0
 CustomerVisit(9,2,1,7):         w(9)(2)(1)(7)#858  = 0
 CustomerVisit(10,2,1,7):        w(10)(2)(1)(7)#906  = 0
 CustomerVisit(11,2,1,7):        w(11)(2)(1)(7)#954  = 0
 CustomerVisit(12,2,1,7):        w(12)(2)(1)(7)#1002  = 0
 CustomerVisit(3,2,2,7):         w(3)(2)(2)(7)#578  = 0
 CustomerVisit(4,2,2,7):         w(4)(2)(2)(7)#626  = 0
 CustomerVisit(5,2,2,7):         w(5)(2)(2)(7)#674  = 0
 CustomerVisit(6,2,2,7):         w(6)(2)(2)(7)#722  = 0
 CustomerVisit(7,2,2,7):         w(7)(2)(2)(7)#770  = 0
 CustomerVisit(8,2,2,7):         w(8)(2)(2)(7)#818  = 0
 CustomerVisit(9,2,2,7):         w(9)(2)(2)(7)#866  = 0
 CustomerVisit(10,2,2,7):        w(10)(2)(2)(7)#914  = 0
 CustomerVisit(11,2,2,7):        w(11)(2)(2)(7)#962  = 0
 CustomerVisit(12,2,2,7):        w(12)(2)(2)(7)#1010  = 0
 CustomerVisit(3,2,3,7):         w(3)(2)(3)(7)#586  = 0
 CustomerVisit(5,2,3,7):         w(5)(2)(3)(7)#682  = 0
 CustomerVisit(7,2,3,7):         w(7)(2)(3)(7)#778  = 0
 CustomerVisit(8,2,3,7):         w(8)(2)(3)(7)#826  = 0
 CustomerVisit(9,2,3,7):         w(9)(2)(3)(7)#874  = 0
 CustomerVisit(10,2,3,7):        w(10)(2)(3)(7)#922  = 0
 CustomerVisit(11,2,3,7):        w(11)(2)(3)(7)#970  = 0
 CustomerVisit(12,2,3,7):        w(12)(2)(3)(7)#1018  = 0
 CustomerVisit(3,1,1,8):         w(3)(1)(1)(8)#547  = 0
 CustomerVisit(4,1,1,8):         w(4)(1)(1)(8)#595  = 0
 CustomerVisit(5,1,1,8):         w(5)(1)(1)(8)#643  = 0
 CustomerVisit(6,1,1,8):         w(6)(1)(1)(8)#691  = 0
 CustomerVisit(7,1,1,8):         w(7)(1)(1)(8)#739  = 0
 CustomerVisit(8,1,1,8):         w(8)(1)(1)(8)#787  = 0
 CustomerVisit(9,1,1,8):         w(9)(1)(1)(8)#835  = 0
 CustomerVisit(12,1,1,8):        w(12)(1)(1)(8)#979  = 0
 CustomerVisit(3,1,2,8):         w(3)(1)(2)(8)#555  = 0
 CustomerVisit(4,1,2,8):         w(4)(1)(2)(8)#603  = 0
 CustomerVisit(5,1,2,8):         w(5)(1)(2)(8)#651  = 0
 CustomerVisit(6,1,2,8):         w(6)(1)(2)(8)#699  = 0
 CustomerVisit(7,1,2,8):         w(7)(1)(2)(8)#747  = 0
 CustomerVisit(8,1,2,8):         w(8)(1)(2)(8)#795  = 0
 CustomerVisit(9,1,2,8):         w(9)(1)(2)(8)#843  = 0
 CustomerVisit(10,1,2,8):        w(10)(1)(2)(8)#891  = 0
 CustomerVisit(11,1,2,8):        w(11)(1)(2)(8)#939  = 0
 CustomerVisit(12,1,2,8):        w(12)(1)(2)(8)#987  = 0
 CustomerVisit(3,2,1,8):         w(3)(2)(1)(8)#571  = 0
 CustomerVisit(4,2,1,8):         w(4)(2)(1)(8)#619  = 0
 CustomerVisit(5,2,1,8):         w(5)(2)(1)(8)#667  = 0
 CustomerVisit(6,2,1,8):         w(6)(2)(1)(8)#715  = 0
 CustomerVisit(7,2,1,8):         w(7)(2)(1)(8)#763  = 0
 CustomerVisit(8,2,1,8):         w(8)(2)(1)(8)#811  = 0
 CustomerVisit(9,2,1,8):         w(9)(2)(1)(8)#859  = 0
 CustomerVisit(10,2,1,8):        w(10)(2)(1)(8)#907  = 0
 CustomerVisit(11,2,1,8):        w(11)(2)(1)(8)#955  = 0
 CustomerVisit(12,2,1,8):        w(12)(2)(1)(8)#1003  = 0
 CustomerVisit(3,2,2,8):         w(3)(2)(2)(8)#579  = 0
 CustomerVisit(9,2,2,8):         w(9)(2)(2)(8)#867  = 0
 CustomerVisit(10,2,2,8):        w(10)(2)(2)(8)#915  = 0
 CustomerVisit(11,2,2,8):        w(11)(2)(2)(8)#963  = 0
 CustomerVisit(3,2,3,8):         w(3)(2)(3)(8)#587  = 0
 CustomerVisit(4,2,3,8):         w(4)(2)(3)(8)#635  = 0
 CustomerVisit(5,2,3,8):         w(5)(2)(3)(8)#683  = 0
 CustomerVisit(6,2,3,8):         w(6)(2)(3)(8)#731  = 0
 CustomerVisit(7,2,3,8):         w(7)(2)(3)(8)#779  = 0
 CustomerVisit(8,2,3,8):         w(8)(2)(3)(8)#827  = 0
 CustomerVisit(9,2,3,8):         w(9)(2)(3)(8)#875  = 0
 CustomerVisit(10,2,3,8):        w(10)(2)(3)(8)#923  = 0
 CustomerVisit(11,2,3,8):        w(11)(2)(3)(8)#971  = 0
 CustomerVisit(12,2,3,8):        w(12)(2)(3)(8)#1019  = 0
 VehicleCapacityWarehouse(1,1,1,1): w(3)(1)(1)(1)#540 + w(4)(1)(1)(1)#588
                                 + w(5)(1)(1)(1)#636 + w(6)(1)(1)(1)#684
                                 + w(7)(1)(1)(1)#732 + w(8)(1)(1)(1)#780
                                 + w(9)(1)(1)(1)#828 + w(10)(1)(1)(1)#876
                                 + w(11)(1)(1)(1)#924 + w(12)(1)(1)(1)#972
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,1): w(3)(2)(1)(1)#564 + w(4)(2)(1)(1)#612
                                 + w(5)(2)(1)(1)#660 + w(6)(2)(1)(1)#708
                                 + w(7)(2)(1)(1)#756 + w(8)(2)(1)(1)#804
                                 + w(9)(2)(1)(1)#852 + w(10)(2)(1)(1)#900
                                 + w(11)(2)(1)(1)#948 + w(12)(2)(1)(1)#996
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,1): w(3)(1)(2)(1)#548 + w(4)(1)(2)(1)#596
                                 + w(5)(1)(2)(1)#644 + w(6)(1)(2)(1)#692
                                 + w(7)(1)(2)(1)#740 + w(8)(1)(2)(1)#788
                                 + w(9)(1)(2)(1)#836 + w(10)(1)(2)(1)#884
                                 + w(11)(1)(2)(1)#932 + w(12)(1)(2)(1)#980
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,1): w(3)(2)(2)(1)#572 + w(4)(2)(2)(1)#620
                                 + w(5)(2)(2)(1)#668 + w(6)(2)(2)(1)#716
                                 + w(7)(2)(2)(1)#764 + w(8)(2)(2)(1)#812
                                 + w(9)(2)(2)(1)#860 + w(10)(2)(2)(1)#908
                                 + w(11)(2)(2)(1)#956 + w(12)(2)(2)(1)#1004
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,1): w(3)(1)(3)(1)#556 + w(4)(1)(3)(1)#604
                                 + w(5)(1)(3)(1)#652 + w(6)(1)(3)(1)#700
                                 + w(7)(1)(3)(1)#748 + w(8)(1)(3)(1)#796
                                 + w(9)(1)(3)(1)#844 + w(10)(1)(3)(1)#892
                                 + w(11)(1)(3)(1)#940 + w(12)(1)(3)(1)#988
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,1): w(3)(2)(3)(1)#580 + w(4)(2)(3)(1)#628
                                 + w(5)(2)(3)(1)#676 + w(6)(2)(3)(1)#724
                                 + w(7)(2)(3)(1)#772 + w(8)(2)(3)(1)#820
                                 + w(9)(2)(3)(1)#868 + w(10)(2)(3)(1)#916
                                 + w(11)(2)(3)(1)#964 + w(12)(2)(3)(1)#1012
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,2): w(3)(1)(1)(2)#541 + w(4)(1)(1)(2)#589
                                 + w(5)(1)(1)(2)#637 + w(6)(1)(1)(2)#685
                                 + w(7)(1)(1)(2)#733 + w(8)(1)(1)(2)#781
                                 + w(9)(1)(1)(2)#829 + w(10)(1)(1)(2)#877
                                 + w(11)(1)(1)(2)#925 + w(12)(1)(1)(2)#973
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,2): w(3)(2)(1)(2)#565 + w(4)(2)(1)(2)#613
                                 + w(5)(2)(1)(2)#661 + w(6)(2)(1)(2)#709
                                 + w(7)(2)(1)(2)#757 + w(8)(2)(1)(2)#805
                                 + w(9)(2)(1)(2)#853 + w(10)(2)(1)(2)#901
                                 + w(11)(2)(1)(2)#949 + w(12)(2)(1)(2)#997
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,2): w(3)(1)(2)(2)#549 + w(4)(1)(2)(2)#597
                                 + w(5)(1)(2)(2)#645 + w(6)(1)(2)(2)#693
                                 + w(7)(1)(2)(2)#741 + w(8)(1)(2)(2)#789
                                 + w(9)(1)(2)(2)#837 + w(10)(1)(2)(2)#885
                                 + w(11)(1)(2)(2)#933 + w(12)(1)(2)(2)#981
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,2): w(3)(2)(2)(2)#573 + w(4)(2)(2)(2)#621
                                 + w(5)(2)(2)(2)#669 + w(6)(2)(2)(2)#717
                                 + w(7)(2)(2)(2)#765 + w(8)(2)(2)(2)#813
                                 + w(9)(2)(2)(2)#861 + w(10)(2)(2)(2)#909
                                 + w(11)(2)(2)(2)#957 + w(12)(2)(2)(2)#1005
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,2): w(3)(1)(3)(2)#557 + w(4)(1)(3)(2)#605
                                 + w(5)(1)(3)(2)#653 + w(6)(1)(3)(2)#701
                                 + w(7)(1)(3)(2)#749 + w(8)(1)(3)(2)#797
                                 + w(9)(1)(3)(2)#845 + w(10)(1)(3)(2)#893
                                 + w(11)(1)(3)(2)#941 + w(12)(1)(3)(2)#989
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,2): w(3)(2)(3)(2)#581 + w(4)(2)(3)(2)#629
                                 + w(5)(2)(3)(2)#677 + w(6)(2)(3)(2)#725
                                 + w(7)(2)(3)(2)#773 + w(8)(2)(3)(2)#821
                                 + w(9)(2)(3)(2)#869 + w(10)(2)(3)(2)#917
                                 + w(11)(2)(3)(2)#965 + w(12)(2)(3)(2)#1013
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,3): w(3)(1)(1)(3)#542 + w(4)(1)(1)(3)#590
                                 + w(5)(1)(1)(3)#638 + w(6)(1)(1)(3)#686
                                 + w(7)(1)(1)(3)#734 + w(8)(1)(1)(3)#782
                                 + w(9)(1)(1)(3)#830 + w(10)(1)(1)(3)#878
                                 + w(11)(1)(1)(3)#926 + w(12)(1)(1)(3)#974
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,3): w(3)(2)(1)(3)#566 + w(4)(2)(1)(3)#614
                                 + w(5)(2)(1)(3)#662 + w(6)(2)(1)(3)#710
                                 + w(7)(2)(1)(3)#758 + w(8)(2)(1)(3)#806
                                 + w(9)(2)(1)(3)#854 + w(10)(2)(1)(3)#902
                                 + w(11)(2)(1)(3)#950 + w(12)(2)(1)(3)#998
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,3): w(3)(1)(2)(3)#550 + w(4)(1)(2)(3)#598
                                 + w(5)(1)(2)(3)#646 + w(6)(1)(2)(3)#694
                                 + w(7)(1)(2)(3)#742 + w(8)(1)(2)(3)#790
                                 + w(9)(1)(2)(3)#838 + w(10)(1)(2)(3)#886
                                 + w(11)(1)(2)(3)#934 + w(12)(1)(2)(3)#982
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,3): w(3)(2)(2)(3)#574 + w(4)(2)(2)(3)#622
                                 + w(5)(2)(2)(3)#670 + w(6)(2)(2)(3)#718
                                 + w(7)(2)(2)(3)#766 + w(8)(2)(2)(3)#814
                                 + w(9)(2)(2)(3)#862 + w(10)(2)(2)(3)#910
                                 + w(11)(2)(2)(3)#958 + w(12)(2)(2)(3)#1006
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,3): w(3)(1)(3)(3)#558 + w(4)(1)(3)(3)#606
                                 + w(5)(1)(3)(3)#654 + w(6)(1)(3)(3)#702
                                 + w(7)(1)(3)(3)#750 + w(8)(1)(3)(3)#798
                                 + w(9)(1)(3)(3)#846 + w(10)(1)(3)(3)#894
                                 + w(11)(1)(3)(3)#942 + w(12)(1)(3)(3)#990
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,3): w(3)(2)(3)(3)#582 + w(4)(2)(3)(3)#630
                                 + w(5)(2)(3)(3)#678 + w(6)(2)(3)(3)#726
                                 + w(7)(2)(3)(3)#774 + w(8)(2)(3)(3)#822
                                 + w(9)(2)(3)(3)#870 + w(10)(2)(3)(3)#918
                                 + w(11)(2)(3)(3)#966 + w(12)(2)(3)(3)#1014
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,4): w(3)(1)(1)(4)#543 + w(4)(1)(1)(4)#591
                                 + w(5)(1)(1)(4)#639 + w(6)(1)(1)(4)#687
                                 + w(7)(1)(1)(4)#735 + w(8)(1)(1)(4)#783
                                 + w(9)(1)(1)(4)#831 + w(10)(1)(1)(4)#879
                                 + w(11)(1)(1)(4)#927 + w(12)(1)(1)(4)#975
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,4): w(3)(2)(1)(4)#567 + w(4)(2)(1)(4)#615
                                 + w(5)(2)(1)(4)#663 + w(6)(2)(1)(4)#711
                                 + w(7)(2)(1)(4)#759 + w(8)(2)(1)(4)#807
                                 + w(9)(2)(1)(4)#855 + w(10)(2)(1)(4)#903
                                 + w(11)(2)(1)(4)#951 + w(12)(2)(1)(4)#999
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,4): w(3)(1)(2)(4)#551 + w(4)(1)(2)(4)#599
                                 + w(5)(1)(2)(4)#647 + w(6)(1)(2)(4)#695
                                 + w(7)(1)(2)(4)#743 + w(8)(1)(2)(4)#791
                                 + w(9)(1)(2)(4)#839 + w(10)(1)(2)(4)#887
                                 + w(11)(1)(2)(4)#935 + w(12)(1)(2)(4)#983
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,4): w(3)(2)(2)(4)#575 + w(4)(2)(2)(4)#623
                                 + w(5)(2)(2)(4)#671 + w(6)(2)(2)(4)#719
                                 + w(7)(2)(2)(4)#767 + w(8)(2)(2)(4)#815
                                 + w(9)(2)(2)(4)#863 + w(10)(2)(2)(4)#911
                                 + w(11)(2)(2)(4)#959 + w(12)(2)(2)(4)#1007
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,4): w(3)(1)(3)(4)#559 + w(4)(1)(3)(4)#607
                                 + w(5)(1)(3)(4)#655 + w(6)(1)(3)(4)#703
                                 + w(7)(1)(3)(4)#751 + w(8)(1)(3)(4)#799
                                 + w(9)(1)(3)(4)#847 + w(10)(1)(3)(4)#895
                                 + w(11)(1)(3)(4)#943 + w(12)(1)(3)(4)#991
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,4): w(3)(2)(3)(4)#583 + w(4)(2)(3)(4)#631
                                 + w(5)(2)(3)(4)#679 + w(6)(2)(3)(4)#727
                                 + w(7)(2)(3)(4)#775 + w(8)(2)(3)(4)#823
                                 + w(9)(2)(3)(4)#871 + w(10)(2)(3)(4)#919
                                 + w(11)(2)(3)(4)#967 + w(12)(2)(3)(4)#1015
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,5): w(3)(1)(1)(5)#544 + w(4)(1)(1)(5)#592
                                 + w(5)(1)(1)(5)#640 + w(6)(1)(1)(5)#688
                                 + w(7)(1)(1)(5)#736 + w(8)(1)(1)(5)#784
                                 + w(9)(1)(1)(5)#832 + w(10)(1)(1)(5)#880
                                 + w(11)(1)(1)(5)#928 + w(12)(1)(1)(5)#976
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,5): w(3)(2)(1)(5)#568 + w(4)(2)(1)(5)#616
                                 + w(5)(2)(1)(5)#664 + w(6)(2)(1)(5)#712
                                 + w(7)(2)(1)(5)#760 + w(8)(2)(1)(5)#808
                                 + w(9)(2)(1)(5)#856 + w(10)(2)(1)(5)#904
                                 + w(11)(2)(1)(5)#952 + w(12)(2)(1)(5)#1000
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,5): w(3)(1)(2)(5)#552 + w(4)(1)(2)(5)#600
                                 + w(5)(1)(2)(5)#648 + w(6)(1)(2)(5)#696
                                 + w(7)(1)(2)(5)#744 + w(8)(1)(2)(5)#792
                                 + w(9)(1)(2)(5)#840 + w(10)(1)(2)(5)#888
                                 + w(11)(1)(2)(5)#936 + w(12)(1)(2)(5)#984
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,5): w(3)(2)(2)(5)#576 + w(4)(2)(2)(5)#624
                                 + w(5)(2)(2)(5)#672 + w(6)(2)(2)(5)#720
                                 + w(7)(2)(2)(5)#768 + w(8)(2)(2)(5)#816
                                 + w(9)(2)(2)(5)#864 + w(10)(2)(2)(5)#912
                                 + w(11)(2)(2)(5)#960 + w(12)(2)(2)(5)#1008
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,5): w(3)(1)(3)(5)#560 + w(4)(1)(3)(5)#608
                                 + w(5)(1)(3)(5)#656 + w(6)(1)(3)(5)#704
                                 + w(7)(1)(3)(5)#752 + w(8)(1)(3)(5)#800
                                 + w(9)(1)(3)(5)#848 + w(10)(1)(3)(5)#896
                                 + w(11)(1)(3)(5)#944 + w(12)(1)(3)(5)#992
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,5): w(3)(2)(3)(5)#584 + w(4)(2)(3)(5)#632
                                 + w(5)(2)(3)(5)#680 + w(6)(2)(3)(5)#728
                                 + w(7)(2)(3)(5)#776 + w(8)(2)(3)(5)#824
                                 + w(9)(2)(3)(5)#872 + w(10)(2)(3)(5)#920
                                 + w(11)(2)(3)(5)#968 + w(12)(2)(3)(5)#1016
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,6): w(3)(1)(1)(6)#545 + w(4)(1)(1)(6)#593
                                 + w(5)(1)(1)(6)#641 + w(6)(1)(1)(6)#689
                                 + w(7)(1)(1)(6)#737 + w(8)(1)(1)(6)#785
                                 + w(9)(1)(1)(6)#833 + w(10)(1)(1)(6)#881
                                 + w(11)(1)(1)(6)#929 + w(12)(1)(1)(6)#977
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,6): w(3)(2)(1)(6)#569 + w(4)(2)(1)(6)#617
                                 + w(5)(2)(1)(6)#665 + w(6)(2)(1)(6)#713
                                 + w(7)(2)(1)(6)#761 + w(8)(2)(1)(6)#809
                                 + w(9)(2)(1)(6)#857 + w(10)(2)(1)(6)#905
                                 + w(11)(2)(1)(6)#953 + w(12)(2)(1)(6)#1001
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,6): w(3)(1)(2)(6)#553 + w(4)(1)(2)(6)#601
                                 + w(5)(1)(2)(6)#649 + w(6)(1)(2)(6)#697
                                 + w(7)(1)(2)(6)#745 + w(8)(1)(2)(6)#793
                                 + w(9)(1)(2)(6)#841 + w(10)(1)(2)(6)#889
                                 + w(11)(1)(2)(6)#937 + w(12)(1)(2)(6)#985
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,6): w(3)(2)(2)(6)#577 + w(4)(2)(2)(6)#625
                                 + w(5)(2)(2)(6)#673 + w(6)(2)(2)(6)#721
                                 + w(7)(2)(2)(6)#769 + w(8)(2)(2)(6)#817
                                 + w(9)(2)(2)(6)#865 + w(10)(2)(2)(6)#913
                                 + w(11)(2)(2)(6)#961 + w(12)(2)(2)(6)#1009
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,6): w(3)(1)(3)(6)#561 + w(4)(1)(3)(6)#609
                                 + w(5)(1)(3)(6)#657 + w(6)(1)(3)(6)#705
                                 + w(7)(1)(3)(6)#753 + w(8)(1)(3)(6)#801
                                 + w(9)(1)(3)(6)#849 + w(10)(1)(3)(6)#897
                                 + w(11)(1)(3)(6)#945 + w(12)(1)(3)(6)#993
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,6): w(3)(2)(3)(6)#585 + w(4)(2)(3)(6)#633
                                 + w(5)(2)(3)(6)#681 + w(6)(2)(3)(6)#729
                                 + w(7)(2)(3)(6)#777 + w(8)(2)(3)(6)#825
                                 + w(9)(2)(3)(6)#873 + w(10)(2)(3)(6)#921
                                 + w(11)(2)(3)(6)#969 + w(12)(2)(3)(6)#1017
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,7): w(3)(1)(1)(7)#546 + w(4)(1)(1)(7)#594
                                 + w(5)(1)(1)(7)#642 + w(6)(1)(1)(7)#690
                                 + w(7)(1)(1)(7)#738 + w(8)(1)(1)(7)#786
                                 + w(9)(1)(1)(7)#834 + w(10)(1)(1)(7)#882
                                 + w(11)(1)(1)(7)#930 + w(12)(1)(1)(7)#978
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,7): w(3)(2)(1)(7)#570 + w(4)(2)(1)(7)#618
                                 + w(5)(2)(1)(7)#666 + w(6)(2)(1)(7)#714
                                 + w(7)(2)(1)(7)#762 + w(8)(2)(1)(7)#810
                                 + w(9)(2)(1)(7)#858 + w(10)(2)(1)(7)#906
                                 + w(11)(2)(1)(7)#954 + w(12)(2)(1)(7)#1002
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,7): w(3)(1)(2)(7)#554 + w(4)(1)(2)(7)#602
                                 + w(5)(1)(2)(7)#650 + w(6)(1)(2)(7)#698
                                 + w(7)(1)(2)(7)#746 + w(8)(1)(2)(7)#794
                                 + w(9)(1)(2)(7)#842 + w(10)(1)(2)(7)#890
                                 + w(11)(1)(2)(7)#938 + w(12)(1)(2)(7)#986
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,7): w(3)(2)(2)(7)#578 + w(4)(2)(2)(7)#626
                                 + w(5)(2)(2)(7)#674 + w(6)(2)(2)(7)#722
                                 + w(7)(2)(2)(7)#770 + w(8)(2)(2)(7)#818
                                 + w(9)(2)(2)(7)#866 + w(10)(2)(2)(7)#914
                                 + w(11)(2)(2)(7)#962 + w(12)(2)(2)(7)#1010
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,7): w(3)(1)(3)(7)#562 + w(4)(1)(3)(7)#610
                                 + w(5)(1)(3)(7)#658 + w(6)(1)(3)(7)#706
                                 + w(7)(1)(3)(7)#754 + w(8)(1)(3)(7)#802
                                 + w(9)(1)(3)(7)#850 + w(10)(1)(3)(7)#898
                                 + w(11)(1)(3)(7)#946 + w(12)(1)(3)(7)#994
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,7): w(3)(2)(3)(7)#586 + w(4)(2)(3)(7)#634
                                 + w(5)(2)(3)(7)#682 + w(6)(2)(3)(7)#730
                                 + w(7)(2)(3)(7)#778 + w(8)(2)(3)(7)#826
                                 + w(9)(2)(3)(7)#874 + w(10)(2)(3)(7)#922
                                 + w(11)(2)(3)(7)#970 + w(12)(2)(3)(7)#1018
                                 <= 228
 VehicleCapacityWarehouse(1,1,1,8): w(3)(1)(1)(8)#547 + w(4)(1)(1)(8)#595
                                 + w(5)(1)(1)(8)#643 + w(6)(1)(1)(8)#691
                                 + w(7)(1)(1)(8)#739 + w(8)(1)(1)(8)#787
                                 + w(9)(1)(1)(8)#835 + w(10)(1)(1)(8)#883
                                 + w(11)(1)(1)(8)#931 + w(12)(1)(1)(8)#979
                                 <= 228
 VehicleCapacityWarehouse(2,2,1,8): w(3)(2)(1)(8)#571 + w(4)(2)(1)(8)#619
                                 + w(5)(2)(1)(8)#667 + w(6)(2)(1)(8)#715
                                 + w(7)(2)(1)(8)#763 + w(8)(2)(1)(8)#811
                                 + w(9)(2)(1)(8)#859 + w(10)(2)(1)(8)#907
                                 + w(11)(2)(1)(8)#955 + w(12)(2)(1)(8)#1003
                                 <= 228
 VehicleCapacityWarehouse(1,1,2,8): w(3)(1)(2)(8)#555 + w(4)(1)(2)(8)#603
                                 + w(5)(1)(2)(8)#651 + w(6)(1)(2)(8)#699
                                 + w(7)(1)(2)(8)#747 + w(8)(1)(2)(8)#795
                                 + w(9)(1)(2)(8)#843 + w(10)(1)(2)(8)#891
                                 + w(11)(1)(2)(8)#939 + w(12)(1)(2)(8)#987
                                 <= 228
 VehicleCapacityWarehouse(2,2,2,8): w(3)(2)(2)(8)#579 + w(4)(2)(2)(8)#627
                                 + w(5)(2)(2)(8)#675 + w(6)(2)(2)(8)#723
                                 + w(7)(2)(2)(8)#771 + w(8)(2)(2)(8)#819
                                 + w(9)(2)(2)(8)#867 + w(10)(2)(2)(8)#915
                                 + w(11)(2)(2)(8)#963 + w(12)(2)(2)(8)#1011
                                 <= 228
 VehicleCapacityWarehouse(1,1,3,8): w(3)(1)(3)(8)#563 + w(4)(1)(3)(8)#611
                                 + w(5)(1)(3)(8)#659 + w(6)(1)(3)(8)#707
                                 + w(7)(1)(3)(8)#755 + w(8)(1)(3)(8)#803
                                 + w(9)(1)(3)(8)#851 + w(10)(1)(3)(8)#899
                                 + w(11)(1)(3)(8)#947 + w(12)(1)(3)(8)#995
                                 <= 228
 VehicleCapacityWarehouse(2,2,3,8): w(3)(2)(3)(8)#587 + w(4)(2)(3)(8)#635
                                 + w(5)(2)(3)(8)#683 + w(6)(2)(3)(8)#731
                                 + w(7)(2)(3)(8)#779 + w(8)(2)(3)(8)#827
                                 + w(9)(2)(3)(8)#875 + w(10)(2)(3)(8)#923
                                 + w(11)(2)(3)(8)#971 + w(12)(2)(3)(8)#1019
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
 0 <= I_warehouse(1)(2)(1)#14 <= 124
 0 <= I_warehouse(1)(2)(2)#15 <= 124
 0 <= I_warehouse(1)(2)(3)#16 <= 124
 0 <= I_warehouse(1)(2)(4)#17 <= 124
 0 <= I_warehouse(1)(2)(5)#18 <= 124
 0 <= I_warehouse(1)(2)(6)#19 <= 124
 0 <= I_warehouse(1)(2)(7)#20 <= 124
 0 <= I_warehouse(1)(2)(8)#21 <= 124
 0 <= I_warehouse(1)(3)(1)#22 <= 124
 0 <= I_warehouse(1)(3)(2)#23 <= 124
 0 <= I_warehouse(1)(3)(3)#24 <= 124
 0 <= I_warehouse(1)(3)(4)#25 <= 124
 0 <= I_warehouse(1)(3)(5)#26 <= 124
 0 <= I_warehouse(1)(3)(6)#27 <= 124
 0 <= I_warehouse(1)(3)(7)#28 <= 124
 0 <= I_warehouse(1)(3)(8)#29 <= 124
 0 <= I_warehouse(2)(1)(1)#30 <= 124
 0 <= I_warehouse(2)(1)(2)#31 <= 124
 0 <= I_warehouse(2)(1)(3)#32 <= 124
 0 <= I_warehouse(2)(1)(4)#33 <= 124
 0 <= I_warehouse(2)(1)(5)#34 <= 124
 0 <= I_warehouse(2)(1)(6)#35 <= 124
 0 <= I_warehouse(2)(1)(7)#36 <= 124
 0 <= I_warehouse(2)(1)(8)#37 <= 124
 0 <= I_warehouse(2)(2)(1)#38 <= 124
 0 <= I_warehouse(2)(2)(2)#39 <= 124
 0 <= I_warehouse(2)(2)(3)#40 <= 124
 0 <= I_warehouse(2)(2)(4)#41 <= 124
 0 <= I_warehouse(2)(2)(5)#42 <= 124
 0 <= I_warehouse(2)(2)(6)#43 <= 124
 0 <= I_warehouse(2)(2)(7)#44 <= 124
 0 <= I_warehouse(2)(2)(8)#45 <= 124
 0 <= I_warehouse(2)(3)(1)#46 <= 124
 0 <= I_warehouse(2)(3)(2)#47 <= 124
 0 <= I_warehouse(2)(3)(3)#48 <= 124
 0 <= I_warehouse(2)(3)(4)#49 <= 124
 0 <= I_warehouse(2)(3)(5)#50 <= 124
 0 <= I_warehouse(2)(3)(6)#51 <= 124
 0 <= I_warehouse(2)(3)(7)#52 <= 124
 0 <= I_warehouse(2)(3)(8)#53 <= 124
 0 <= I_customer(3)(1)(1)#54 <= 76
 0 <= I_customer(3)(1)(2)#55 <= 76
 0 <= I_customer(3)(1)(3)#56 <= 76
 0 <= I_customer(3)(1)(4)#57 <= 76
 0 <= I_customer(3)(1)(5)#58 <= 76
 0 <= I_customer(3)(1)(6)#59 <= 76
 0 <= I_customer(3)(1)(7)#60 <= 76
 0 <= I_customer(3)(1)(8)#61 <= 76
 0 <= I_customer(3)(2)(1)#62 <= 76
 0 <= I_customer(3)(2)(2)#63 <= 76
 0 <= I_customer(3)(2)(3)#64 <= 76
 0 <= I_customer(3)(2)(4)#65 <= 76
 0 <= I_customer(3)(2)(5)#66 <= 76
 0 <= I_customer(3)(2)(6)#67 <= 76
 0 <= I_customer(3)(2)(7)#68 <= 76
 0 <= I_customer(3)(2)(8)#69 <= 76
 0 <= I_customer(3)(3)(1)#70 <= 76
 0 <= I_customer(3)(3)(2)#71 <= 76
 0 <= I_customer(3)(3)(3)#72 <= 76
 0 <= I_customer(3)(3)(4)#73 <= 76
 0 <= I_customer(3)(3)(5)#74 <= 76
 0 <= I_customer(3)(3)(6)#75 <= 76
 0 <= I_customer(3)(3)(7)#76 <= 76
 0 <= I_customer(3)(3)(8)#77 <= 76
 0 <= I_customer(4)(1)(1)#78 <= 28
 0 <= I_customer(4)(1)(2)#79 <= 28
 0 <= I_customer(4)(1)(3)#80 <= 28
 0 <= I_customer(4)(1)(4)#81 <= 28
 0 <= I_customer(4)(1)(5)#82 <= 28
 0 <= I_customer(4)(1)(6)#83 <= 28
 0 <= I_customer(4)(1)(7)#84 <= 28
 0 <= I_customer(4)(1)(8)#85 <= 28
 0 <= I_customer(4)(2)(1)#86 <= 28
 0 <= I_customer(4)(2)(2)#87 <= 28
 0 <= I_customer(4)(2)(3)#88 <= 28
 0 <= I_customer(4)(2)(4)#89 <= 28
 0 <= I_customer(4)(2)(5)#90 <= 28
 0 <= I_customer(4)(2)(6)#91 <= 28
 0 <= I_customer(4)(2)(7)#92 <= 28
 0 <= I_customer(4)(2)(8)#93 <= 28
 0 <= I_customer(4)(3)(1)#94 <= 28
 0 <= I_customer(4)(3)(2)#95 <= 28
 0 <= I_customer(4)(3)(3)#96 <= 28
 0 <= I_customer(4)(3)(4)#97 <= 28
 0 <= I_customer(4)(3)(5)#98 <= 28
 0 <= I_customer(4)(3)(6)#99 <= 28
 0 <= I_customer(4)(3)(7)#100 <= 28
 0 <= I_customer(4)(3)(8)#101 <= 28
 0 <= I_customer(5)(1)(1)#102 <= 24
 0 <= I_customer(5)(1)(2)#103 <= 24
 0 <= I_customer(5)(1)(3)#104 <= 24
 0 <= I_customer(5)(1)(4)#105 <= 24
 0 <= I_customer(5)(1)(5)#106 <= 24
 0 <= I_customer(5)(1)(6)#107 <= 24
 0 <= I_customer(5)(1)(7)#108 <= 24
 0 <= I_customer(5)(1)(8)#109 <= 24
 0 <= I_customer(5)(2)(1)#110 <= 24
 0 <= I_customer(5)(2)(2)#111 <= 24
 0 <= I_customer(5)(2)(3)#112 <= 24
 0 <= I_customer(5)(2)(4)#113 <= 24
 0 <= I_customer(5)(2)(5)#114 <= 24
 0 <= I_customer(5)(2)(6)#115 <= 24
 0 <= I_customer(5)(2)(7)#116 <= 24
 0 <= I_customer(5)(2)(8)#117 <= 24
 0 <= I_customer(5)(3)(1)#118 <= 24
 0 <= I_customer(5)(3)(2)#119 <= 24
 0 <= I_customer(5)(3)(3)#120 <= 24
 0 <= I_customer(5)(3)(4)#121 <= 24
 0 <= I_customer(5)(3)(5)#122 <= 24
 0 <= I_customer(5)(3)(6)#123 <= 24
 0 <= I_customer(5)(3)(7)#124 <= 24
 0 <= I_customer(5)(3)(8)#125 <= 24
 0 <= I_customer(6)(1)(1)#126 <= 64
 0 <= I_customer(6)(1)(2)#127 <= 64
 0 <= I_customer(6)(1)(3)#128 <= 64
 0 <= I_customer(6)(1)(4)#129 <= 64
 0 <= I_customer(6)(1)(5)#130 <= 64
 0 <= I_customer(6)(1)(6)#131 <= 64
 0 <= I_customer(6)(1)(7)#132 <= 64
 0 <= I_customer(6)(1)(8)#133 <= 64
 0 <= I_customer(6)(2)(1)#134 <= 64
 0 <= I_customer(6)(2)(2)#135 <= 64
 0 <= I_customer(6)(2)(3)#136 <= 64
 0 <= I_customer(6)(2)(4)#137 <= 64
 0 <= I_customer(6)(2)(5)#138 <= 64
 0 <= I_customer(6)(2)(6)#139 <= 64
 0 <= I_customer(6)(2)(7)#140 <= 64
 0 <= I_customer(6)(2)(8)#141 <= 64
 0 <= I_customer(6)(3)(1)#142 <= 64
 0 <= I_customer(6)(3)(2)#143 <= 64
 0 <= I_customer(6)(3)(3)#144 <= 64
 0 <= I_customer(6)(3)(4)#145 <= 64
 0 <= I_customer(6)(3)(5)#146 <= 64
 0 <= I_customer(6)(3)(6)#147 <= 64
 0 <= I_customer(6)(3)(7)#148 <= 64
 0 <= I_customer(6)(3)(8)#149 <= 64
 0 <= I_customer(7)(1)(1)#150 <= 28
 0 <= I_customer(7)(1)(2)#151 <= 28
 0 <= I_customer(7)(1)(3)#152 <= 28
 0 <= I_customer(7)(1)(4)#153 <= 28
 0 <= I_customer(7)(1)(5)#154 <= 28
 0 <= I_customer(7)(1)(6)#155 <= 28
 0 <= I_customer(7)(1)(7)#156 <= 28
 0 <= I_customer(7)(1)(8)#157 <= 28
 0 <= I_customer(7)(2)(1)#158 <= 28
 0 <= I_customer(7)(2)(2)#159 <= 28
 0 <= I_customer(7)(2)(3)#160 <= 28
 0 <= I_customer(7)(2)(4)#161 <= 28
 0 <= I_customer(7)(2)(5)#162 <= 28
 0 <= I_customer(7)(2)(6)#163 <= 28
 0 <= I_customer(7)(2)(7)#164 <= 28
 0 <= I_customer(7)(2)(8)#165 <= 28
 0 <= I_customer(7)(3)(1)#166 <= 28
 0 <= I_customer(7)(3)(2)#167 <= 28
 0 <= I_customer(7)(3)(3)#168 <= 28
 0 <= I_customer(7)(3)(4)#169 <= 28
 0 <= I_customer(7)(3)(5)#170 <= 28
 0 <= I_customer(7)(3)(6)#171 <= 28
 0 <= I_customer(7)(3)(7)#172 <= 28
 0 <= I_customer(7)(3)(8)#173 <= 28
 0 <= I_customer(8)(1)(1)#174 <= 72
 0 <= I_customer(8)(1)(2)#175 <= 72
 0 <= I_customer(8)(1)(3)#176 <= 72
 0 <= I_customer(8)(1)(4)#177 <= 72
 0 <= I_customer(8)(1)(5)#178 <= 72
 0 <= I_customer(8)(1)(6)#179 <= 72
 0 <= I_customer(8)(1)(7)#180 <= 72
 0 <= I_customer(8)(1)(8)#181 <= 72
 0 <= I_customer(8)(2)(1)#182 <= 72
 0 <= I_customer(8)(2)(2)#183 <= 72
 0 <= I_customer(8)(2)(3)#184 <= 72
 0 <= I_customer(8)(2)(4)#185 <= 72
 0 <= I_customer(8)(2)(5)#186 <= 72
 0 <= I_customer(8)(2)(6)#187 <= 72
 0 <= I_customer(8)(2)(7)#188 <= 72
 0 <= I_customer(8)(2)(8)#189 <= 72
 0 <= I_customer(8)(3)(1)#190 <= 72
 0 <= I_customer(8)(3)(2)#191 <= 72
 0 <= I_customer(8)(3)(3)#192 <= 72
 0 <= I_customer(8)(3)(4)#193 <= 72
 0 <= I_customer(8)(3)(5)#194 <= 72
 0 <= I_customer(8)(3)(6)#195 <= 72
 0 <= I_customer(8)(3)(7)#196 <= 72
 0 <= I_customer(8)(3)(8)#197 <= 72
 0 <= I_customer(9)(1)(1)#198 <= 64
 0 <= I_customer(9)(1)(2)#199 <= 64
 0 <= I_customer(9)(1)(3)#200 <= 64
 0 <= I_customer(9)(1)(4)#201 <= 64
 0 <= I_customer(9)(1)(5)#202 <= 64
 0 <= I_customer(9)(1)(6)#203 <= 64
 0 <= I_customer(9)(1)(7)#204 <= 64
 0 <= I_customer(9)(1)(8)#205 <= 64
 0 <= I_customer(9)(2)(1)#206 <= 64
 0 <= I_customer(9)(2)(2)#207 <= 64
 0 <= I_customer(9)(2)(3)#208 <= 64
 0 <= I_customer(9)(2)(4)#209 <= 64
 0 <= I_customer(9)(2)(5)#210 <= 64
 0 <= I_customer(9)(2)(6)#211 <= 64
 0 <= I_customer(9)(2)(7)#212 <= 64
 0 <= I_customer(9)(2)(8)#213 <= 64
 0 <= I_customer(9)(3)(1)#214 <= 64
 0 <= I_customer(9)(3)(2)#215 <= 64
 0 <= I_customer(9)(3)(3)#216 <= 64
 0 <= I_customer(9)(3)(4)#217 <= 64
 0 <= I_customer(9)(3)(5)#218 <= 64
 0 <= I_customer(9)(3)(6)#219 <= 64
 0 <= I_customer(9)(3)(7)#220 <= 64
 0 <= I_customer(9)(3)(8)#221 <= 64
 0 <= I_customer(10)(1)(1)#222 <= 64
 0 <= I_customer(10)(1)(2)#223 <= 64
 0 <= I_customer(10)(1)(3)#224 <= 64
 0 <= I_customer(10)(1)(4)#225 <= 64
 0 <= I_customer(10)(1)(5)#226 <= 64
 0 <= I_customer(10)(1)(6)#227 <= 64
 0 <= I_customer(10)(1)(7)#228 <= 64
 0 <= I_customer(10)(1)(8)#229 <= 64
 0 <= I_customer(10)(2)(1)#230 <= 64
 0 <= I_customer(10)(2)(2)#231 <= 64
 0 <= I_customer(10)(2)(3)#232 <= 64
 0 <= I_customer(10)(2)(4)#233 <= 64
 0 <= I_customer(10)(2)(5)#234 <= 64
 0 <= I_customer(10)(2)(6)#235 <= 64
 0 <= I_customer(10)(2)(7)#236 <= 64
 0 <= I_customer(10)(2)(8)#237 <= 64
 0 <= I_customer(10)(3)(1)#238 <= 64
 0 <= I_customer(10)(3)(2)#239 <= 64
 0 <= I_customer(10)(3)(3)#240 <= 64
 0 <= I_customer(10)(3)(4)#241 <= 64
 0 <= I_customer(10)(3)(5)#242 <= 64
 0 <= I_customer(10)(3)(6)#243 <= 64
 0 <= I_customer(10)(3)(7)#244 <= 64
 0 <= I_customer(10)(3)(8)#245 <= 64
 0 <= I_customer(11)(1)(1)#246 <= 64
 0 <= I_customer(11)(1)(2)#247 <= 64
 0 <= I_customer(11)(1)(3)#248 <= 64
 0 <= I_customer(11)(1)(4)#249 <= 64
 0 <= I_customer(11)(1)(5)#250 <= 64
 0 <= I_customer(11)(1)(6)#251 <= 64
 0 <= I_customer(11)(1)(7)#252 <= 64
 0 <= I_customer(11)(1)(8)#253 <= 64
 0 <= I_customer(11)(2)(1)#254 <= 64
 0 <= I_customer(11)(2)(2)#255 <= 64
 0 <= I_customer(11)(2)(3)#256 <= 64
 0 <= I_customer(11)(2)(4)#257 <= 64
 0 <= I_customer(11)(2)(5)#258 <= 64
 0 <= I_customer(11)(2)(6)#259 <= 64
 0 <= I_customer(11)(2)(7)#260 <= 64
 0 <= I_customer(11)(2)(8)#261 <= 64
 0 <= I_customer(11)(3)(1)#262 <= 64
 0 <= I_customer(11)(3)(2)#263 <= 64
 0 <= I_customer(11)(3)(3)#264 <= 64
 0 <= I_customer(11)(3)(4)#265 <= 64
 0 <= I_customer(11)(3)(5)#266 <= 64
 0 <= I_customer(11)(3)(6)#267 <= 64
 0 <= I_customer(11)(3)(7)#268 <= 64
 0 <= I_customer(11)(3)(8)#269 <= 64
 0 <= I_customer(12)(1)(1)#270 <= 56
 0 <= I_customer(12)(1)(2)#271 <= 56
 0 <= I_customer(12)(1)(3)#272 <= 56
 0 <= I_customer(12)(1)(4)#273 <= 56
 0 <= I_customer(12)(1)(5)#274 <= 56
 0 <= I_customer(12)(1)(6)#275 <= 56
 0 <= I_customer(12)(1)(7)#276 <= 56
 0 <= I_customer(12)(1)(8)#277 <= 56
 0 <= I_customer(12)(2)(1)#278 <= 56
 0 <= I_customer(12)(2)(2)#279 <= 56
 0 <= I_customer(12)(2)(3)#280 <= 56
 0 <= I_customer(12)(2)(4)#281 <= 56
 0 <= I_customer(12)(2)(5)#282 <= 56
 0 <= I_customer(12)(2)(6)#283 <= 56
 0 <= I_customer(12)(2)(7)#284 <= 56
 0 <= I_customer(12)(2)(8)#285 <= 56
 0 <= I_customer(12)(3)(1)#286 <= 56
 0 <= I_customer(12)(3)(2)#287 <= 56
 0 <= I_customer(12)(3)(3)#288 <= 56
 0 <= I_customer(12)(3)(4)#289 <= 56
 0 <= I_customer(12)(3)(5)#290 <= 56
 0 <= I_customer(12)(3)(6)#291 <= 56
 0 <= I_customer(12)(3)(7)#292 <= 56
 0 <= I_customer(12)(3)(8)#293 <= 56
 0 <= b_customer(3)(1)(1)#294 <= 18
 0 <= b_customer(3)(1)(2)#295 <= 22
 0 <= b_customer(3)(1)(3)#296 <= 21
 0 <= b_customer(3)(1)(4)#297 <= 21
 0 <= b_customer(3)(1)(5)#298 <= 18
 0 <= b_customer(3)(1)(6)#299 <= 22
 0 <= b_customer(3)(1)(7)#300 <= 23
 0 <= b_customer(3)(1)(8)#301 <= 19
 0 <= b_customer(3)(2)(1)#302 <= 17
 0 <= b_customer(3)(2)(2)#303 <= 20
 0 <= b_customer(3)(2)(3)#304 <= 20
 0 <= b_customer(3)(2)(4)#305 <= 20
 0 <= b_customer(3)(2)(5)#306 <= 22
 0 <= b_customer(3)(2)(6)#307 <= 19
 0 <= b_customer(3)(2)(7)#308 <= 21
 0 <= b_customer(3)(2)(8)#309 <= 19
 0 <= b_customer(3)(3)(1)#310 <= 22
 0 <= b_customer(3)(3)(2)#311 <= 16
 0 <= b_customer(3)(3)(3)#312 <= 20
 0 <= b_customer(3)(3)(4)#313 <= 21
 0 <= b_customer(3)(3)(5)#314 <= 17
 0 <= b_customer(3)(3)(6)#315 <= 18
 0 <= b_customer(3)(3)(7)#316 <= 19
 0 <= b_customer(3)(3)(8)#317 <= 15
 0 <= b_customer(4)(1)(1)#318 <= 8
 0 <= b_customer(4)(1)(2)#319 <= 6
 0 <= b_customer(4)(1)(3)#320 <= 8
 0 <= b_customer(4)(1)(4)#321 <= 8
 0 <= b_customer(4)(1)(5)#322 <= 8
 0 <= b_customer(4)(1)(6)#323 <= 6
 0 <= b_customer(4)(1)(7)#324 <= 6
 0 <= b_customer(4)(1)(8)#325 <= 6
 0 <= b_customer(4)(2)(1)#326 <= 6
 0 <= b_customer(4)(2)(2)#327 <= 8
 0 <= b_customer(4)(2)(3)#328 <= 8
 0 <= b_customer(4)(2)(4)#329 <= 7
 0 <= b_customer(4)(2)(5)#330 <= 8
 0 <= b_customer(4)(2)(6)#331 <= 6
 0 <= b_customer(4)(2)(7)#332 <= 8
 0 <= b_customer(4)(2)(8)#333 <= 6
 0 <= b_customer(4)(3)(1)#334 <= 8
 0 <= b_customer(4)(3)(2)#335 <= 7
 0 <= b_customer(4)(3)(3)#336 <= 7
 0 <= b_customer(4)(3)(4)#337 <= 6
 0 <= b_customer(4)(3)(5)#338 <= 7
 0 <= b_customer(4)(3)(6)#339 <= 7
 0 <= b_customer(4)(3)(7)#340 <= 7
 0 <= b_customer(4)(3)(8)#341 <= 7
 0 <= b_customer(5)(1)(1)#342 <= 5
 0 <= b_customer(5)(1)(2)#343 <= 5
 0 <= b_customer(5)(1)(3)#344 <= 6
 0 <= b_customer(5)(1)(4)#345 <= 7
 0 <= b_customer(5)(1)(5)#346 <= 7
 0 <= b_customer(5)(1)(6)#347 <= 6
 0 <= b_customer(5)(1)(7)#348 <= 6
 0 <= b_customer(5)(1)(8)#349 <= 5
 0 <= b_customer(5)(2)(1)#350 <= 6
 0 <= b_customer(5)(2)(2)#351 <= 7
 0 <= b_customer(5)(2)(3)#352 <= 7
 0 <= b_customer(5)(2)(4)#353 <= 6
 0 <= b_customer(5)(2)(5)#354 <= 5
 0 <= b_customer(5)(2)(6)#355 <= 7
 0 <= b_customer(5)(2)(7)#356 <= 5
 0 <= b_customer(5)(2)(8)#357 <= 5
 0 <= b_customer(5)(3)(1)#358 <= 6
 0 <= b_customer(5)(3)(2)#359 <= 7
 0 <= b_customer(5)(3)(3)#360 <= 5
 0 <= b_customer(5)(3)(4)#361 <= 6
 0 <= b_customer(5)(3)(5)#362 <= 6
 0 <= b_customer(5)(3)(6)#363 <= 5
 0 <= b_customer(5)(3)(7)#364 <= 5
 0 <= b_customer(5)(3)(8)#365 <= 7
 0 <= b_customer(6)(1)(1)#366 <= 13
 0 <= b_customer(6)(1)(2)#367 <= 17
 0 <= b_customer(6)(1)(3)#368 <= 16
 0 <= b_customer(6)(1)(4)#369 <= 17
 0 <= b_customer(6)(1)(5)#370 <= 17
 0 <= b_customer(6)(1)(6)#371 <= 17
 0 <= b_customer(6)(1)(7)#372 <= 17
 0 <= b_customer(6)(1)(8)#373 <= 17
 0 <= b_customer(6)(2)(1)#374 <= 17
 0 <= b_customer(6)(2)(2)#375 <= 16
 0 <= b_customer(6)(2)(3)#376 <= 15
 0 <= b_customer(6)(2)(4)#377 <= 14
 0 <= b_customer(6)(2)(5)#378 <= 15
 0 <= b_customer(6)(2)(6)#379 <= 15
 0 <= b_customer(6)(2)(7)#380 <= 15
 0 <= b_customer(6)(2)(8)#381 <= 17
 0 <= b_customer(6)(3)(1)#382 <= 17
 0 <= b_customer(6)(3)(2)#383 <= 17
 0 <= b_customer(6)(3)(3)#384 <= 16
 0 <= b_customer(6)(3)(4)#385 <= 14
 0 <= b_customer(6)(3)(5)#386 <= 18
 0 <= b_customer(6)(3)(6)#387 <= 13
 0 <= b_customer(6)(3)(7)#388 <= 17
 0 <= b_customer(6)(3)(8)#389 <= 16
 0 <= b_customer(7)(1)(1)#390 <= 8
 0 <= b_customer(7)(1)(2)#391 <= 6
 0 <= b_customer(7)(1)(3)#392 <= 8
 0 <= b_customer(7)(1)(4)#393 <= 8
 0 <= b_customer(7)(1)(5)#394 <= 8
 0 <= b_customer(7)(1)(6)#395 <= 7
 0 <= b_customer(7)(1)(7)#396 <= 6
 0 <= b_customer(7)(1)(8)#397 <= 8
 0 <= b_customer(7)(2)(1)#398 <= 8
 0 <= b_customer(7)(2)(2)#399 <= 7
 0 <= b_customer(7)(2)(3)#400 <= 7
 0 <= b_customer(7)(2)(4)#401 <= 6
 0 <= b_customer(7)(2)(5)#402 <= 6
 0 <= b_customer(7)(2)(6)#403 <= 6
 0 <= b_customer(7)(2)(7)#404 <= 7
 0 <= b_customer(7)(2)(8)#405 <= 7
 0 <= b_customer(7)(3)(1)#406 <= 6
 0 <= b_customer(7)(3)(2)#407 <= 6
 0 <= b_customer(7)(3)(3)#408 <= 7
 0 <= b_customer(7)(3)(4)#409 <= 6
 0 <= b_customer(7)(3)(5)#410 <= 7
 0 <= b_customer(7)(3)(6)#411 <= 7
 0 <= b_customer(7)(3)(7)#412 <= 7
 0 <= b_customer(7)(3)(8)#413 <= 7
 0 <= b_customer(8)(1)(1)#414 <= 15
 0 <= b_customer(8)(1)(2)#415 <= 18
 0 <= b_customer(8)(1)(3)#416 <= 21
 0 <= b_customer(8)(1)(4)#417 <= 16
 0 <= b_customer(8)(1)(5)#418 <= 21
 0 <= b_customer(8)(1)(6)#419 <= 17
 0 <= b_customer(8)(1)(7)#420 <= 20
 0 <= b_customer(8)(1)(8)#421 <= 15
 0 <= b_customer(8)(2)(1)#422 <= 20
 0 <= b_customer(8)(2)(2)#423 <= 17
 0 <= b_customer(8)(2)(3)#424 <= 20
 0 <= b_customer(8)(2)(4)#425 <= 17
 0 <= b_customer(8)(2)(5)#426 <= 15
 0 <= b_customer(8)(2)(6)#427 <= 21
 0 <= b_customer(8)(2)(7)#428 <= 20
 0 <= b_customer(8)(2)(8)#429 <= 19
 0 <= b_customer(8)(3)(1)#430 <= 19
 0 <= b_customer(8)(3)(2)#431 <= 19
 0 <= b_customer(8)(3)(3)#432 <= 21
 0 <= b_customer(8)(3)(4)#433 <= 16
 0 <= b_customer(8)(3)(5)#434 <= 15
 0 <= b_customer(8)(3)(6)#435 <= 17
 0 <= b_customer(8)(3)(7)#436 <= 16
 0 <= b_customer(8)(3)(8)#437 <= 21
 0 <= b_customer(9)(1)(1)#438 <= 19
 0 <= b_customer(9)(1)(2)#439 <= 13
 0 <= b_customer(9)(1)(3)#440 <= 14
 0 <= b_customer(9)(1)(4)#441 <= 16
 0 <= b_customer(9)(1)(5)#442 <= 19
 0 <= b_customer(9)(1)(6)#443 <= 19
 0 <= b_customer(9)(1)(7)#444 <= 13
 0 <= b_customer(9)(1)(8)#445 <= 15
 0 <= b_customer(9)(2)(1)#446 <= 15
 0 <= b_customer(9)(2)(2)#447 <= 14
 0 <= b_customer(9)(2)(3)#448 <= 16
 0 <= b_customer(9)(2)(4)#449 <= 17
 0 <= b_customer(9)(2)(5)#450 <= 19
 0 <= b_customer(9)(2)(6)#451 <= 14
 0 <= b_customer(9)(2)(7)#452 <= 15
 0 <= b_customer(9)(2)(8)#453 <= 15
 0 <= b_customer(9)(3)(1)#454 <= 17
 0 <= b_customer(9)(3)(2)#455 <= 13
 0 <= b_customer(9)(3)(3)#456 <= 19
 0 <= b_customer(9)(3)(4)#457 <= 16
 0 <= b_customer(9)(3)(5)#458 <= 17
 0 <= b_customer(9)(3)(6)#459 <= 19
 0 <= b_customer(9)(3)(7)#460 <= 19
 0 <= b_customer(9)(3)(8)#461 <= 18
 0 <= b_customer(10)(1)(1)#462 <= 14
 0 <= b_customer(10)(1)(2)#463 <= 14
 0 <= b_customer(10)(1)(3)#464 <= 18
 0 <= b_customer(10)(1)(4)#465 <= 18
 0 <= b_customer(10)(1)(5)#466 <= 13
 0 <= b_customer(10)(1)(6)#467 <= 16
 0 <= b_customer(10)(1)(7)#468 <= 16
 0 <= b_customer(10)(1)(8)#469 <= 13
 0 <= b_customer(10)(2)(1)#470 <= 14
 0 <= b_customer(10)(2)(2)#471 <= 18
 0 <= b_customer(10)(2)(3)#472 <= 13
 0 <= b_customer(10)(2)(4)#473 <= 19
 0 <= b_customer(10)(2)(5)#474 <= 16
 0 <= b_customer(10)(2)(6)#475 <= 18
 0 <= b_customer(10)(2)(7)#476 <= 17
 0 <= b_customer(10)(2)(8)#477 <= 13
 0 <= b_customer(10)(3)(1)#478 <= 16
 0 <= b_customer(10)(3)(2)#479 <= 17
 0 <= b_customer(10)(3)(3)#480 <= 19
 0 <= b_customer(10)(3)(4)#481 <= 18
 0 <= b_customer(10)(3)(5)#482 <= 14
 0 <= b_customer(10)(3)(6)#483 <= 18
 0 <= b_customer(10)(3)(7)#484 <= 18
 0 <= b_customer(10)(3)(8)#485 <= 17
 0 <= b_customer(11)(1)(1)#486 <= 13
 0 <= b_customer(11)(1)(2)#487 <= 19
 0 <= b_customer(11)(1)(3)#488 <= 15
 0 <= b_customer(11)(1)(4)#489 <= 15
 0 <= b_customer(11)(1)(5)#490 <= 19
 0 <= b_customer(11)(1)(6)#491 <= 14
 0 <= b_customer(11)(1)(7)#492 <= 17
 0 <= b_customer(11)(1)(8)#493 <= 19
 0 <= b_customer(11)(2)(1)#494 <= 13
 0 <= b_customer(11)(2)(2)#495 <= 16
 0 <= b_customer(11)(2)(3)#496 <= 18
 0 <= b_customer(11)(2)(4)#497 <= 19
 0 <= b_customer(11)(2)(5)#498 <= 18
 0 <= b_customer(11)(2)(6)#499 <= 13
 0 <= b_customer(11)(2)(7)#500 <= 17
 0 <= b_customer(11)(2)(8)#501 <= 14
 0 <= b_customer(11)(3)(1)#502 <= 14
 0 <= b_customer(11)(3)(2)#503 <= 18
 0 <= b_customer(11)(3)(3)#504 <= 17
 0 <= b_customer(11)(3)(4)#505 <= 14
 0 <= b_customer(11)(3)(5)#506 <= 14
 0 <= b_customer(11)(3)(6)#507 <= 14
 0 <= b_customer(11)(3)(7)#508 <= 17
 0 <= b_customer(11)(3)(8)#509 <= 16
 0 <= b_customer(12)(1)(1)#510 <= 13
 0 <= b_customer(12)(1)(2)#511 <= 11
 0 <= b_customer(12)(1)(3)#512 <= 13
 0 <= b_customer(12)(1)(4)#513 <= 15
 0 <= b_customer(12)(1)(5)#514 <= 14
 0 <= b_customer(12)(1)(6)#515 <= 12
 0 <= b_customer(12)(1)(7)#516 <= 12
 0 <= b_customer(12)(1)(8)#517 <= 12
 0 <= b_customer(12)(2)(1)#518 <= 14
 0 <= b_customer(12)(2)(2)#519 <= 14
 0 <= b_customer(12)(2)(3)#520 <= 13
 0 <= b_customer(12)(2)(4)#521 <= 15
 0 <= b_customer(12)(2)(5)#522 <= 12
 0 <= b_customer(12)(2)(6)#523 <= 12
 0 <= b_customer(12)(2)(7)#524 <= 14
 0 <= b_customer(12)(2)(8)#525 <= 17
 0 <= b_customer(12)(3)(1)#526 <= 16
 0 <= b_customer(12)(3)(2)#527 <= 16
 0 <= b_customer(12)(3)(3)#528 <= 13
 0 <= b_customer(12)(3)(4)#529 <= 12
 0 <= b_customer(12)(3)(5)#530 <= 12
 0 <= b_customer(12)(3)(6)#531 <= 16
 0 <= b_customer(12)(3)(7)#532 <= 16
 0 <= b_customer(12)(3)(8)#533 <= 14
 0 <= q(1)(1)(1)#534 <= 124
 0 <= q(1)(1)(2)#535 <= 124
 0 <= q(1)(1)(3)#536 <= 124
 0 <= q(2)(1)(1)#537 <= 124
 0 <= q(2)(1)(2)#538 <= 124
 0 <= q(2)(1)(3)#539 <= 124
 0 <= w(3)(1)(1)(1)#540 <= 18
 0 <= w(3)(1)(1)(2)#541 <= 22
 0 <= w(3)(1)(1)(3)#542 <= 21
 0 <= w(3)(1)(1)(4)#543 <= 21
 0 <= w(3)(1)(1)(5)#544 <= 18
 0 <= w(3)(1)(1)(6)#545 <= 22
 0 <= w(3)(1)(1)(7)#546 <= 23
 0 <= w(3)(1)(1)(8)#547 <= 19
 0 <= w(3)(1)(2)(1)#548 <= 17
 0 <= w(3)(1)(2)(2)#549 <= 20
 0 <= w(3)(1)(2)(3)#550 <= 20
 0 <= w(3)(1)(2)(4)#551 <= 20
 0 <= w(3)(1)(2)(5)#552 <= 22
 0 <= w(3)(1)(2)(6)#553 <= 19
 0 <= w(3)(1)(2)(7)#554 <= 21
 0 <= w(3)(1)(2)(8)#555 <= 19
 0 <= w(3)(1)(3)(1)#556 <= 22
 0 <= w(3)(1)(3)(2)#557 <= 16
 0 <= w(3)(1)(3)(3)#558 <= 20
 0 <= w(3)(1)(3)(4)#559 <= 21
 0 <= w(3)(1)(3)(5)#560 <= 17
 0 <= w(3)(1)(3)(6)#561 <= 18
 0 <= w(3)(1)(3)(7)#562 <= 19
 0 <= w(3)(1)(3)(8)#563 <= 15
 0 <= w(3)(2)(1)(1)#564 <= 18
 0 <= w(3)(2)(1)(2)#565 <= 22
 0 <= w(3)(2)(1)(3)#566 <= 21
 0 <= w(3)(2)(1)(4)#567 <= 21
 0 <= w(3)(2)(1)(5)#568 <= 18
 0 <= w(3)(2)(1)(6)#569 <= 22
 0 <= w(3)(2)(1)(7)#570 <= 23
 0 <= w(3)(2)(1)(8)#571 <= 19
 0 <= w(3)(2)(2)(1)#572 <= 17
 0 <= w(3)(2)(2)(2)#573 <= 20
 0 <= w(3)(2)(2)(3)#574 <= 20
 0 <= w(3)(2)(2)(4)#575 <= 20
 0 <= w(3)(2)(2)(5)#576 <= 22
 0 <= w(3)(2)(2)(6)#577 <= 19
 0 <= w(3)(2)(2)(7)#578 <= 21
 0 <= w(3)(2)(2)(8)#579 <= 19
 0 <= w(3)(2)(3)(1)#580 <= 22
 0 <= w(3)(2)(3)(2)#581 <= 16
 0 <= w(3)(2)(3)(3)#582 <= 20
 0 <= w(3)(2)(3)(4)#583 <= 21
 0 <= w(3)(2)(3)(5)#584 <= 17
 0 <= w(3)(2)(3)(6)#585 <= 18
 0 <= w(3)(2)(3)(7)#586 <= 19
 0 <= w(3)(2)(3)(8)#587 <= 15
 0 <= w(4)(1)(1)(1)#588 <= 8
 0 <= w(4)(1)(1)(2)#589 <= 6
 0 <= w(4)(1)(1)(3)#590 <= 8
 0 <= w(4)(1)(1)(4)#591 <= 8
 0 <= w(4)(1)(1)(5)#592 <= 8
 0 <= w(4)(1)(1)(6)#593 <= 6
 0 <= w(4)(1)(1)(7)#594 <= 6
 0 <= w(4)(1)(1)(8)#595 <= 6
 0 <= w(4)(1)(2)(1)#596 <= 6
 0 <= w(4)(1)(2)(2)#597 <= 8
 0 <= w(4)(1)(2)(3)#598 <= 8
 0 <= w(4)(1)(2)(4)#599 <= 7
 0 <= w(4)(1)(2)(5)#600 <= 8
 0 <= w(4)(1)(2)(6)#601 <= 6
 0 <= w(4)(1)(2)(7)#602 <= 8
 0 <= w(4)(1)(2)(8)#603 <= 6
 0 <= w(4)(1)(3)(1)#604 <= 8
 0 <= w(4)(1)(3)(2)#605 <= 7
 0 <= w(4)(1)(3)(3)#606 <= 7
 0 <= w(4)(1)(3)(4)#607 <= 6
 0 <= w(4)(1)(3)(5)#608 <= 7
 0 <= w(4)(1)(3)(6)#609 <= 7
 0 <= w(4)(1)(3)(7)#610 <= 7
 0 <= w(4)(1)(3)(8)#611 <= 7
 0 <= w(4)(2)(1)(1)#612 <= 8
 0 <= w(4)(2)(1)(2)#613 <= 6
 0 <= w(4)(2)(1)(3)#614 <= 8
 0 <= w(4)(2)(1)(4)#615 <= 8
 0 <= w(4)(2)(1)(5)#616 <= 8
 0 <= w(4)(2)(1)(6)#617 <= 6
 0 <= w(4)(2)(1)(7)#618 <= 6
 0 <= w(4)(2)(1)(8)#619 <= 6
 0 <= w(4)(2)(2)(1)#620 <= 6
 0 <= w(4)(2)(2)(2)#621 <= 8
 0 <= w(4)(2)(2)(3)#622 <= 8
 0 <= w(4)(2)(2)(4)#623 <= 7
 0 <= w(4)(2)(2)(5)#624 <= 8
 0 <= w(4)(2)(2)(6)#625 <= 6
 0 <= w(4)(2)(2)(7)#626 <= 8
 0 <= w(4)(2)(2)(8)#627 <= 6
 0 <= w(4)(2)(3)(1)#628 <= 8
 0 <= w(4)(2)(3)(2)#629 <= 7
 0 <= w(4)(2)(3)(3)#630 <= 7
 0 <= w(4)(2)(3)(4)#631 <= 6
 0 <= w(4)(2)(3)(5)#632 <= 7
 0 <= w(4)(2)(3)(6)#633 <= 7
 0 <= w(4)(2)(3)(7)#634 <= 7
 0 <= w(4)(2)(3)(8)#635 <= 7
 0 <= w(5)(1)(1)(1)#636 <= 5
 0 <= w(5)(1)(1)(2)#637 <= 5
 0 <= w(5)(1)(1)(3)#638 <= 6
 0 <= w(5)(1)(1)(4)#639 <= 7
 0 <= w(5)(1)(1)(5)#640 <= 7
 0 <= w(5)(1)(1)(6)#641 <= 6
 0 <= w(5)(1)(1)(7)#642 <= 6
 0 <= w(5)(1)(1)(8)#643 <= 5
 0 <= w(5)(1)(2)(1)#644 <= 6
 0 <= w(5)(1)(2)(2)#645 <= 7
 0 <= w(5)(1)(2)(3)#646 <= 7
 0 <= w(5)(1)(2)(4)#647 <= 6
 0 <= w(5)(1)(2)(5)#648 <= 5
 0 <= w(5)(1)(2)(6)#649 <= 7
 0 <= w(5)(1)(2)(7)#650 <= 5
 0 <= w(5)(1)(2)(8)#651 <= 5
 0 <= w(5)(1)(3)(1)#652 <= 6
 0 <= w(5)(1)(3)(2)#653 <= 7
 0 <= w(5)(1)(3)(3)#654 <= 5
 0 <= w(5)(1)(3)(4)#655 <= 6
 0 <= w(5)(1)(3)(5)#656 <= 6
 0 <= w(5)(1)(3)(6)#657 <= 5
 0 <= w(5)(1)(3)(7)#658 <= 5
 0 <= w(5)(1)(3)(8)#659 <= 7
 0 <= w(5)(2)(1)(1)#660 <= 5
 0 <= w(5)(2)(1)(2)#661 <= 5
 0 <= w(5)(2)(1)(3)#662 <= 6
 0 <= w(5)(2)(1)(4)#663 <= 7
 0 <= w(5)(2)(1)(5)#664 <= 7
 0 <= w(5)(2)(1)(6)#665 <= 6
 0 <= w(5)(2)(1)(7)#666 <= 6
 0 <= w(5)(2)(1)(8)#667 <= 5
 0 <= w(5)(2)(2)(1)#668 <= 6
 0 <= w(5)(2)(2)(2)#669 <= 7
 0 <= w(5)(2)(2)(3)#670 <= 7
 0 <= w(5)(2)(2)(4)#671 <= 6
 0 <= w(5)(2)(2)(5)#672 <= 5
 0 <= w(5)(2)(2)(6)#673 <= 7
 0 <= w(5)(2)(2)(7)#674 <= 5
 0 <= w(5)(2)(2)(8)#675 <= 5
 0 <= w(5)(2)(3)(1)#676 <= 6
 0 <= w(5)(2)(3)(2)#677 <= 7
 0 <= w(5)(2)(3)(3)#678 <= 5
 0 <= w(5)(2)(3)(4)#679 <= 6
 0 <= w(5)(2)(3)(5)#680 <= 6
 0 <= w(5)(2)(3)(6)#681 <= 5
 0 <= w(5)(2)(3)(7)#682 <= 5
 0 <= w(5)(2)(3)(8)#683 <= 7
 0 <= w(6)(1)(1)(1)#684 <= 13
 0 <= w(6)(1)(1)(2)#685 <= 17
 0 <= w(6)(1)(1)(3)#686 <= 16
 0 <= w(6)(1)(1)(4)#687 <= 17
 0 <= w(6)(1)(1)(5)#688 <= 17
 0 <= w(6)(1)(1)(6)#689 <= 17
 0 <= w(6)(1)(1)(7)#690 <= 17
 0 <= w(6)(1)(1)(8)#691 <= 17
 0 <= w(6)(1)(2)(1)#692 <= 17
 0 <= w(6)(1)(2)(2)#693 <= 16
 0 <= w(6)(1)(2)(3)#694 <= 15
 0 <= w(6)(1)(2)(4)#695 <= 14
 0 <= w(6)(1)(2)(5)#696 <= 15
 0 <= w(6)(1)(2)(6)#697 <= 15
 0 <= w(6)(1)(2)(7)#698 <= 15
 0 <= w(6)(1)(2)(8)#699 <= 17
 0 <= w(6)(1)(3)(1)#700 <= 17
 0 <= w(6)(1)(3)(2)#701 <= 17
 0 <= w(6)(1)(3)(3)#702 <= 16
 0 <= w(6)(1)(3)(4)#703 <= 14
 0 <= w(6)(1)(3)(5)#704 <= 18
 0 <= w(6)(1)(3)(6)#705 <= 13
 0 <= w(6)(1)(3)(7)#706 <= 17
 0 <= w(6)(1)(3)(8)#707 <= 16
 0 <= w(6)(2)(1)(1)#708 <= 13
 0 <= w(6)(2)(1)(2)#709 <= 17
 0 <= w(6)(2)(1)(3)#710 <= 16
 0 <= w(6)(2)(1)(4)#711 <= 17
 0 <= w(6)(2)(1)(5)#712 <= 17
 0 <= w(6)(2)(1)(6)#713 <= 17
 0 <= w(6)(2)(1)(7)#714 <= 17
 0 <= w(6)(2)(1)(8)#715 <= 17
 0 <= w(6)(2)(2)(1)#716 <= 17
 0 <= w(6)(2)(2)(2)#717 <= 16
 0 <= w(6)(2)(2)(3)#718 <= 15
 0 <= w(6)(2)(2)(4)#719 <= 14
 0 <= w(6)(2)(2)(5)#720 <= 15
 0 <= w(6)(2)(2)(6)#721 <= 15
 0 <= w(6)(2)(2)(7)#722 <= 15
 0 <= w(6)(2)(2)(8)#723 <= 17
 0 <= w(6)(2)(3)(1)#724 <= 17
 0 <= w(6)(2)(3)(2)#725 <= 17
 0 <= w(6)(2)(3)(3)#726 <= 16
 0 <= w(6)(2)(3)(4)#727 <= 14
 0 <= w(6)(2)(3)(5)#728 <= 18
 0 <= w(6)(2)(3)(6)#729 <= 13
 0 <= w(6)(2)(3)(7)#730 <= 17
 0 <= w(6)(2)(3)(8)#731 <= 16
 0 <= w(7)(1)(1)(1)#732 <= 8
 0 <= w(7)(1)(1)(2)#733 <= 6
 0 <= w(7)(1)(1)(3)#734 <= 8
 0 <= w(7)(1)(1)(4)#735 <= 8
 0 <= w(7)(1)(1)(5)#736 <= 8
 0 <= w(7)(1)(1)(6)#737 <= 7
 0 <= w(7)(1)(1)(7)#738 <= 6
 0 <= w(7)(1)(1)(8)#739 <= 8
 0 <= w(7)(1)(2)(1)#740 <= 8
 0 <= w(7)(1)(2)(2)#741 <= 7
 0 <= w(7)(1)(2)(3)#742 <= 7
 0 <= w(7)(1)(2)(4)#743 <= 6
 0 <= w(7)(1)(2)(5)#744 <= 6
 0 <= w(7)(1)(2)(6)#745 <= 6
 0 <= w(7)(1)(2)(7)#746 <= 7
 0 <= w(7)(1)(2)(8)#747 <= 7
 0 <= w(7)(1)(3)(1)#748 <= 6
 0 <= w(7)(1)(3)(2)#749 <= 6
 0 <= w(7)(1)(3)(3)#750 <= 7
 0 <= w(7)(1)(3)(4)#751 <= 6
 0 <= w(7)(1)(3)(5)#752 <= 7
 0 <= w(7)(1)(3)(6)#753 <= 7
 0 <= w(7)(1)(3)(7)#754 <= 7
 0 <= w(7)(1)(3)(8)#755 <= 7
 0 <= w(7)(2)(1)(1)#756 <= 8
 0 <= w(7)(2)(1)(2)#757 <= 6
 0 <= w(7)(2)(1)(3)#758 <= 8
 0 <= w(7)(2)(1)(4)#759 <= 8
 0 <= w(7)(2)(1)(5)#760 <= 8
 0 <= w(7)(2)(1)(6)#761 <= 7
 0 <= w(7)(2)(1)(7)#762 <= 6
 0 <= w(7)(2)(1)(8)#763 <= 8
 0 <= w(7)(2)(2)(1)#764 <= 8
 0 <= w(7)(2)(2)(2)#765 <= 7
 0 <= w(7)(2)(2)(3)#766 <= 7
 0 <= w(7)(2)(2)(4)#767 <= 6
 0 <= w(7)(2)(2)(5)#768 <= 6
 0 <= w(7)(2)(2)(6)#769 <= 6
 0 <= w(7)(2)(2)(7)#770 <= 7
 0 <= w(7)(2)(2)(8)#771 <= 7
 0 <= w(7)(2)(3)(1)#772 <= 6
 0 <= w(7)(2)(3)(2)#773 <= 6
 0 <= w(7)(2)(3)(3)#774 <= 7
 0 <= w(7)(2)(3)(4)#775 <= 6
 0 <= w(7)(2)(3)(5)#776 <= 7
 0 <= w(7)(2)(3)(6)#777 <= 7
 0 <= w(7)(2)(3)(7)#778 <= 7
 0 <= w(7)(2)(3)(8)#779 <= 7
 0 <= w(8)(1)(1)(1)#780 <= 15
 0 <= w(8)(1)(1)(2)#781 <= 18
 0 <= w(8)(1)(1)(3)#782 <= 21
 0 <= w(8)(1)(1)(4)#783 <= 16
 0 <= w(8)(1)(1)(5)#784 <= 21
 0 <= w(8)(1)(1)(6)#785 <= 17
 0 <= w(8)(1)(1)(7)#786 <= 20
 0 <= w(8)(1)(1)(8)#787 <= 15
 0 <= w(8)(1)(2)(1)#788 <= 20
 0 <= w(8)(1)(2)(2)#789 <= 17
 0 <= w(8)(1)(2)(3)#790 <= 20
 0 <= w(8)(1)(2)(4)#791 <= 17
 0 <= w(8)(1)(2)(5)#792 <= 15
 0 <= w(8)(1)(2)(6)#793 <= 21
 0 <= w(8)(1)(2)(7)#794 <= 20
 0 <= w(8)(1)(2)(8)#795 <= 19
 0 <= w(8)(1)(3)(1)#796 <= 19
 0 <= w(8)(1)(3)(2)#797 <= 19
 0 <= w(8)(1)(3)(3)#798 <= 21
 0 <= w(8)(1)(3)(4)#799 <= 16
 0 <= w(8)(1)(3)(5)#800 <= 15
 0 <= w(8)(1)(3)(6)#801 <= 17
 0 <= w(8)(1)(3)(7)#802 <= 16
 0 <= w(8)(1)(3)(8)#803 <= 21
 0 <= w(8)(2)(1)(1)#804 <= 15
 0 <= w(8)(2)(1)(2)#805 <= 18
 0 <= w(8)(2)(1)(3)#806 <= 21
 0 <= w(8)(2)(1)(4)#807 <= 16
 0 <= w(8)(2)(1)(5)#808 <= 21
 0 <= w(8)(2)(1)(6)#809 <= 17
 0 <= w(8)(2)(1)(7)#810 <= 20
 0 <= w(8)(2)(1)(8)#811 <= 15
 0 <= w(8)(2)(2)(1)#812 <= 20
 0 <= w(8)(2)(2)(2)#813 <= 17
 0 <= w(8)(2)(2)(3)#814 <= 20
 0 <= w(8)(2)(2)(4)#815 <= 17
 0 <= w(8)(2)(2)(5)#816 <= 15
 0 <= w(8)(2)(2)(6)#817 <= 21
 0 <= w(8)(2)(2)(7)#818 <= 20
 0 <= w(8)(2)(2)(8)#819 <= 19
 0 <= w(8)(2)(3)(1)#820 <= 19
 0 <= w(8)(2)(3)(2)#821 <= 19
 0 <= w(8)(2)(3)(3)#822 <= 21
 0 <= w(8)(2)(3)(4)#823 <= 16
 0 <= w(8)(2)(3)(5)#824 <= 15
 0 <= w(8)(2)(3)(6)#825 <= 17
 0 <= w(8)(2)(3)(7)#826 <= 16
 0 <= w(8)(2)(3)(8)#827 <= 21
 0 <= w(9)(1)(1)(1)#828 <= 19
 0 <= w(9)(1)(1)(2)#829 <= 13
 0 <= w(9)(1)(1)(3)#830 <= 14
 0 <= w(9)(1)(1)(4)#831 <= 16
 0 <= w(9)(1)(1)(5)#832 <= 19
 0 <= w(9)(1)(1)(6)#833 <= 19
 0 <= w(9)(1)(1)(7)#834 <= 13
 0 <= w(9)(1)(1)(8)#835 <= 15
 0 <= w(9)(1)(2)(1)#836 <= 15
 0 <= w(9)(1)(2)(2)#837 <= 14
 0 <= w(9)(1)(2)(3)#838 <= 16
 0 <= w(9)(1)(2)(4)#839 <= 17
 0 <= w(9)(1)(2)(5)#840 <= 19
 0 <= w(9)(1)(2)(6)#841 <= 14
 0 <= w(9)(1)(2)(7)#842 <= 15
 0 <= w(9)(1)(2)(8)#843 <= 15
 0 <= w(9)(1)(3)(1)#844 <= 17
 0 <= w(9)(1)(3)(2)#845 <= 13
 0 <= w(9)(1)(3)(3)#846 <= 19
 0 <= w(9)(1)(3)(4)#847 <= 16
 0 <= w(9)(1)(3)(5)#848 <= 17
 0 <= w(9)(1)(3)(6)#849 <= 19
 0 <= w(9)(1)(3)(7)#850 <= 19
 0 <= w(9)(1)(3)(8)#851 <= 18
 0 <= w(9)(2)(1)(1)#852 <= 19
 0 <= w(9)(2)(1)(2)#853 <= 13
 0 <= w(9)(2)(1)(3)#854 <= 14
 0 <= w(9)(2)(1)(4)#855 <= 16
 0 <= w(9)(2)(1)(5)#856 <= 19
 0 <= w(9)(2)(1)(6)#857 <= 19
 0 <= w(9)(2)(1)(7)#858 <= 13
 0 <= w(9)(2)(1)(8)#859 <= 15
 0 <= w(9)(2)(2)(1)#860 <= 15
 0 <= w(9)(2)(2)(2)#861 <= 14
 0 <= w(9)(2)(2)(3)#862 <= 16
 0 <= w(9)(2)(2)(4)#863 <= 17
 0 <= w(9)(2)(2)(5)#864 <= 19
 0 <= w(9)(2)(2)(6)#865 <= 14
 0 <= w(9)(2)(2)(7)#866 <= 15
 0 <= w(9)(2)(2)(8)#867 <= 15
 0 <= w(9)(2)(3)(1)#868 <= 17
 0 <= w(9)(2)(3)(2)#869 <= 13
 0 <= w(9)(2)(3)(3)#870 <= 19
 0 <= w(9)(2)(3)(4)#871 <= 16
 0 <= w(9)(2)(3)(5)#872 <= 17
 0 <= w(9)(2)(3)(6)#873 <= 19
 0 <= w(9)(2)(3)(7)#874 <= 19
 0 <= w(9)(2)(3)(8)#875 <= 18
 0 <= w(10)(1)(1)(1)#876 <= 14
 0 <= w(10)(1)(1)(2)#877 <= 14
 0 <= w(10)(1)(1)(3)#878 <= 18
 0 <= w(10)(1)(1)(4)#879 <= 18
 0 <= w(10)(1)(1)(5)#880 <= 13
 0 <= w(10)(1)(1)(6)#881 <= 16
 0 <= w(10)(1)(1)(7)#882 <= 16
 0 <= w(10)(1)(1)(8)#883 <= 13
 0 <= w(10)(1)(2)(1)#884 <= 14
 0 <= w(10)(1)(2)(2)#885 <= 18
 0 <= w(10)(1)(2)(3)#886 <= 13
 0 <= w(10)(1)(2)(4)#887 <= 19
 0 <= w(10)(1)(2)(5)#888 <= 16
 0 <= w(10)(1)(2)(6)#889 <= 18
 0 <= w(10)(1)(2)(7)#890 <= 17
 0 <= w(10)(1)(2)(8)#891 <= 13
 0 <= w(10)(1)(3)(1)#892 <= 16
 0 <= w(10)(1)(3)(2)#893 <= 17
 0 <= w(10)(1)(3)(3)#894 <= 19
 0 <= w(10)(1)(3)(4)#895 <= 18
 0 <= w(10)(1)(3)(5)#896 <= 14
 0 <= w(10)(1)(3)(6)#897 <= 18
 0 <= w(10)(1)(3)(7)#898 <= 18
 0 <= w(10)(1)(3)(8)#899 <= 17
 0 <= w(10)(2)(1)(1)#900 <= 14
 0 <= w(10)(2)(1)(2)#901 <= 14
 0 <= w(10)(2)(1)(3)#902 <= 18
 0 <= w(10)(2)(1)(4)#903 <= 18
 0 <= w(10)(2)(1)(5)#904 <= 13
 0 <= w(10)(2)(1)(6)#905 <= 16
 0 <= w(10)(2)(1)(7)#906 <= 16
 0 <= w(10)(2)(1)(8)#907 <= 13
 0 <= w(10)(2)(2)(1)#908 <= 14
 0 <= w(10)(2)(2)(2)#909 <= 18
 0 <= w(10)(2)(2)(3)#910 <= 13
 0 <= w(10)(2)(2)(4)#911 <= 19
 0 <= w(10)(2)(2)(5)#912 <= 16
 0 <= w(10)(2)(2)(6)#913 <= 18
 0 <= w(10)(2)(2)(7)#914 <= 17
 0 <= w(10)(2)(2)(8)#915 <= 13
 0 <= w(10)(2)(3)(1)#916 <= 16
 0 <= w(10)(2)(3)(2)#917 <= 17
 0 <= w(10)(2)(3)(3)#918 <= 19
 0 <= w(10)(2)(3)(4)#919 <= 18
 0 <= w(10)(2)(3)(5)#920 <= 14
 0 <= w(10)(2)(3)(6)#921 <= 18
 0 <= w(10)(2)(3)(7)#922 <= 18
 0 <= w(10)(2)(3)(8)#923 <= 17
 0 <= w(11)(1)(1)(1)#924 <= 13
 0 <= w(11)(1)(1)(2)#925 <= 19
 0 <= w(11)(1)(1)(3)#926 <= 15
 0 <= w(11)(1)(1)(4)#927 <= 15
 0 <= w(11)(1)(1)(5)#928 <= 19
 0 <= w(11)(1)(1)(6)#929 <= 14
 0 <= w(11)(1)(1)(7)#930 <= 17
 0 <= w(11)(1)(1)(8)#931 <= 19
 0 <= w(11)(1)(2)(1)#932 <= 13
 0 <= w(11)(1)(2)(2)#933 <= 16
 0 <= w(11)(1)(2)(3)#934 <= 18
 0 <= w(11)(1)(2)(4)#935 <= 19
 0 <= w(11)(1)(2)(5)#936 <= 18
 0 <= w(11)(1)(2)(6)#937 <= 13
 0 <= w(11)(1)(2)(7)#938 <= 17
 0 <= w(11)(1)(2)(8)#939 <= 14
 0 <= w(11)(1)(3)(1)#940 <= 14
 0 <= w(11)(1)(3)(2)#941 <= 18
 0 <= w(11)(1)(3)(3)#942 <= 17
 0 <= w(11)(1)(3)(4)#943 <= 14
 0 <= w(11)(1)(3)(5)#944 <= 14
 0 <= w(11)(1)(3)(6)#945 <= 14
 0 <= w(11)(1)(3)(7)#946 <= 17
 0 <= w(11)(1)(3)(8)#947 <= 16
 0 <= w(11)(2)(1)(1)#948 <= 13
 0 <= w(11)(2)(1)(2)#949 <= 19
 0 <= w(11)(2)(1)(3)#950 <= 15
 0 <= w(11)(2)(1)(4)#951 <= 15
 0 <= w(11)(2)(1)(5)#952 <= 19
 0 <= w(11)(2)(1)(6)#953 <= 14
 0 <= w(11)(2)(1)(7)#954 <= 17
 0 <= w(11)(2)(1)(8)#955 <= 19
 0 <= w(11)(2)(2)(1)#956 <= 13
 0 <= w(11)(2)(2)(2)#957 <= 16
 0 <= w(11)(2)(2)(3)#958 <= 18
 0 <= w(11)(2)(2)(4)#959 <= 19
 0 <= w(11)(2)(2)(5)#960 <= 18
 0 <= w(11)(2)(2)(6)#961 <= 13
 0 <= w(11)(2)(2)(7)#962 <= 17
 0 <= w(11)(2)(2)(8)#963 <= 14
 0 <= w(11)(2)(3)(1)#964 <= 14
 0 <= w(11)(2)(3)(2)#965 <= 18
 0 <= w(11)(2)(3)(3)#966 <= 17
 0 <= w(11)(2)(3)(4)#967 <= 14
 0 <= w(11)(2)(3)(5)#968 <= 14
 0 <= w(11)(2)(3)(6)#969 <= 14
 0 <= w(11)(2)(3)(7)#970 <= 17
 0 <= w(11)(2)(3)(8)#971 <= 16
 0 <= w(12)(1)(1)(1)#972 <= 13
 0 <= w(12)(1)(1)(2)#973 <= 11
 0 <= w(12)(1)(1)(3)#974 <= 13
 0 <= w(12)(1)(1)(4)#975 <= 15
 0 <= w(12)(1)(1)(5)#976 <= 14
 0 <= w(12)(1)(1)(6)#977 <= 12
 0 <= w(12)(1)(1)(7)#978 <= 12
 0 <= w(12)(1)(1)(8)#979 <= 12
 0 <= w(12)(1)(2)(1)#980 <= 14
 0 <= w(12)(1)(2)(2)#981 <= 14
 0 <= w(12)(1)(2)(3)#982 <= 13
 0 <= w(12)(1)(2)(4)#983 <= 15
 0 <= w(12)(1)(2)(5)#984 <= 12
 0 <= w(12)(1)(2)(6)#985 <= 12
 0 <= w(12)(1)(2)(7)#986 <= 14
 0 <= w(12)(1)(2)(8)#987 <= 17
 0 <= w(12)(1)(3)(1)#988 <= 16
 0 <= w(12)(1)(3)(2)#989 <= 16
 0 <= w(12)(1)(3)(3)#990 <= 13
 0 <= w(12)(1)(3)(4)#991 <= 12
 0 <= w(12)(1)(3)(5)#992 <= 12
 0 <= w(12)(1)(3)(6)#993 <= 16
 0 <= w(12)(1)(3)(7)#994 <= 16
 0 <= w(12)(1)(3)(8)#995 <= 14
 0 <= w(12)(2)(1)(1)#996 <= 13
 0 <= w(12)(2)(1)(2)#997 <= 11
 0 <= w(12)(2)(1)(3)#998 <= 13
 0 <= w(12)(2)(1)(4)#999 <= 15
 0 <= w(12)(2)(1)(5)#1000 <= 14
 0 <= w(12)(2)(1)(6)#1001 <= 12
 0 <= w(12)(2)(1)(7)#1002 <= 12
 0 <= w(12)(2)(1)(8)#1003 <= 12
 0 <= w(12)(2)(2)(1)#1004 <= 14
 0 <= w(12)(2)(2)(2)#1005 <= 14
 0 <= w(12)(2)(2)(3)#1006 <= 13
 0 <= w(12)(2)(2)(4)#1007 <= 15
 0 <= w(12)(2)(2)(5)#1008 <= 12
 0 <= w(12)(2)(2)(6)#1009 <= 12
 0 <= w(12)(2)(2)(7)#1010 <= 14
 0 <= w(12)(2)(2)(8)#1011 <= 17
 0 <= w(12)(2)(3)(1)#1012 <= 16
 0 <= w(12)(2)(3)(2)#1013 <= 16
 0 <= w(12)(2)(3)(3)#1014 <= 13
 0 <= w(12)(2)(3)(4)#1015 <= 12
 0 <= w(12)(2)(3)(5)#1016 <= 12
 0 <= w(12)(2)(3)(6)#1017 <= 16
 0 <= w(12)(2)(3)(7)#1018 <= 16
 0 <= w(12)(2)(3)(8)#1019 <= 14
End
