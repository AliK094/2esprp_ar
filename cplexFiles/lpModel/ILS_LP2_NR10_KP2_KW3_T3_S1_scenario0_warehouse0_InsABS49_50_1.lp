\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 4 I_warehouse(0)(0)#0 + 4 I_warehouse(0)(1)#1 + 4 I_warehouse(0)(2)#2
       + 7 I_customer(2)(0)#3 + 8 I_customer(3)(0)#4 + 8 I_customer(6)(0)#5
       + 8 I_customer(7)(0)#6 + 9 I_customer(8)(0)#7 + 7 I_customer(9)(0)#8
       + 7 I_customer(2)(1)#9 + 8 I_customer(3)(1)#10 + 8 I_customer(6)(1)#11
       + 8 I_customer(7)(1)#12 + 9 I_customer(8)(1)#13 + 7 I_customer(9)(1)#14
       + 7 I_customer(2)(2)#15 + 8 I_customer(3)(2)#16 + 8 I_customer(6)(2)#17
       + 8 I_customer(7)(2)#18 + 9 I_customer(8)(2)#19 + 7 I_customer(9)(2)#20
       + 1510 b_customer(2)(0)#21 + 1430 b_customer(3)(0)#22
       + 1010 b_customer(6)(0)#23 + 760 b_customer(7)(0)#24
       + 1370 b_customer(8)(0)#25 + 1370 b_customer(9)(0)#26
       + 1510 b_customer(2)(1)#27 + 1430 b_customer(3)(1)#28
       + 1010 b_customer(6)(1)#29 + 760 b_customer(7)(1)#30
       + 1370 b_customer(8)(1)#31 + 1370 b_customer(9)(1)#32
       + 1510 b_customer(2)(2)#33 + 1430 b_customer(3)(2)#34
       + 1010 b_customer(6)(2)#35 + 760 b_customer(7)(2)#36
       + 1370 b_customer(8)(2)#37 + 1370 b_customer(9)(2)#38
Subject To
 WarehouseInventoryBalance(1,1,1): I_warehouse(0)(0)#0 + w_customer(2)(0)(0)#39
                                 + w_customer(3)(0)(0)#40
                                 + w_customer(6)(0)(0)#41
                                 + w_customer(7)(0)(0)#42
                                 + w_customer(8)(0)(0)#43
                                 + w_customer(9)(0)(0)#44
                                 + w_customer(2)(0)(1)#45
                                 + w_customer(3)(0)(1)#46
                                 + w_customer(6)(0)(1)#47
                                 + w_customer(7)(0)(1)#48
                                 + w_customer(8)(0)(1)#49
                                 + w_customer(9)(0)(1)#50
                                 + w_customer(2)(0)(2)#51
                                 + w_customer(3)(0)(2)#52
                                 + w_customer(6)(0)(2)#53
                                 + w_customer(7)(0)(2)#54
                                 + w_customer(8)(0)(2)#55
                                 + w_customer(9)(0)(2)#56  = 92
 WarehouseInventoryBalance(1,2,1): - I_warehouse(0)(0)#0 + I_warehouse(0)(1)#1
                                 + w_customer(2)(1)(0)#57
                                 + w_customer(3)(1)(0)#58
                                 + w_customer(6)(1)(0)#59
                                 + w_customer(7)(1)(0)#60
                                 + w_customer(8)(1)(0)#61
                                 + w_customer(9)(1)(0)#62
                                 + w_customer(2)(1)(1)#63
                                 + w_customer(3)(1)(1)#64
                                 + w_customer(6)(1)(1)#65
                                 + w_customer(7)(1)(1)#66
                                 + w_customer(8)(1)(1)#67
                                 + w_customer(9)(1)(1)#68
                                 + w_customer(2)(1)(2)#69
                                 + w_customer(3)(1)(2)#70
                                 + w_customer(6)(1)(2)#71
                                 + w_customer(7)(1)(2)#72
                                 + w_customer(8)(1)(2)#73
                                 + w_customer(9)(1)(2)#74  = 102
 WarehouseInventoryBalance(1,3,1): - I_warehouse(0)(1)#1 + I_warehouse(0)(2)#2
                                 + w_customer(2)(2)(0)#75
                                 + w_customer(3)(2)(0)#76
                                 + w_customer(6)(2)(0)#77
                                 + w_customer(7)(2)(0)#78
                                 + w_customer(8)(2)(0)#79
                                 + w_customer(9)(2)(0)#80
                                 + w_customer(2)(2)(1)#81
                                 + w_customer(3)(2)(1)#82
                                 + w_customer(6)(2)(1)#83
                                 + w_customer(7)(2)(1)#84
                                 + w_customer(8)(2)(1)#85
                                 + w_customer(9)(2)(1)#86
                                 + w_customer(2)(2)(2)#87
                                 + w_customer(3)(2)(2)#88
                                 + w_customer(6)(2)(2)#89
                                 + w_customer(7)(2)(2)#90
                                 + w_customer(8)(2)(2)#91
                                 + w_customer(9)(2)(2)#92  = 102
 CustomerInventoryBalance(4,1,1): I_customer(2)(0)#3 - b_customer(2)(0)#21
                                 - w_customer(2)(0)(0)#39
                                 - w_customer(2)(0)(1)#45
                                 - w_customer(2)(0)(2)#51  = 2
 CustomerInventoryBalance(5,1,1): I_customer(3)(0)#4 - b_customer(3)(0)#22
                                 - w_customer(3)(0)(0)#40
                                 - w_customer(3)(0)(1)#46
                                 - w_customer(3)(0)(2)#52  = -3
 CustomerInventoryBalance(8,1,1): I_customer(6)(0)#5 - b_customer(6)(0)#23
                                 - w_customer(6)(0)(0)#41
                                 - w_customer(6)(0)(1)#47
                                 - w_customer(6)(0)(2)#53  = 29
 CustomerInventoryBalance(9,1,1): I_customer(7)(0)#6 - b_customer(7)(0)#24
                                 - w_customer(7)(0)(0)#42
                                 - w_customer(7)(0)(1)#48
                                 - w_customer(7)(0)(2)#54  = -6
 CustomerInventoryBalance(10,1,1): I_customer(8)(0)#7 - b_customer(8)(0)#25
                                 - w_customer(8)(0)(0)#43
                                 - w_customer(8)(0)(1)#49
                                 - w_customer(8)(0)(2)#55  = 32
 CustomerInventoryBalance(11,1,1): I_customer(9)(0)#8 - b_customer(9)(0)#26
                                 - w_customer(9)(0)(0)#44
                                 - w_customer(9)(0)(1)#50
                                 - w_customer(9)(0)(2)#56  = 23
 CustomerInventoryBalance(4,2,1): - I_customer(2)(0)#3 + I_customer(2)(1)#9
                                 - b_customer(2)(1)#27 - w_customer(2)(1)(0)#57
                                 - w_customer(2)(1)(1)#63
                                 - w_customer(2)(1)(2)#69  = -15
 CustomerInventoryBalance(5,2,1): - I_customer(3)(0)#4 + I_customer(3)(1)#10
                                 - b_customer(3)(1)#28 - w_customer(3)(1)(0)#58
                                 - w_customer(3)(1)(1)#64
                                 - w_customer(3)(1)(2)#70  = -7
 CustomerInventoryBalance(8,2,1): - I_customer(6)(0)#5 + I_customer(6)(1)#11
                                 - b_customer(6)(1)#29 - w_customer(6)(1)(0)#59
                                 - w_customer(6)(1)(1)#65
                                 - w_customer(6)(1)(2)#71  = -20
 CustomerInventoryBalance(9,2,1): - I_customer(7)(0)#6 + I_customer(7)(1)#12
                                 - b_customer(7)(1)#30 - w_customer(7)(1)(0)#60
                                 - w_customer(7)(1)(1)#66
                                 - w_customer(7)(1)(2)#72  = -11
 CustomerInventoryBalance(10,2,1): - I_customer(8)(0)#7 + I_customer(8)(1)#13
                                 - b_customer(8)(1)#31 - w_customer(8)(1)(0)#61
                                 - w_customer(8)(1)(1)#67
                                 - w_customer(8)(1)(2)#73  = -16
 CustomerInventoryBalance(11,2,1): - I_customer(9)(0)#8 + I_customer(9)(1)#14
                                 - b_customer(9)(1)#32 - w_customer(9)(1)(0)#62
                                 - w_customer(9)(1)(1)#68
                                 - w_customer(9)(1)(2)#74  = -21
 CustomerInventoryBalance(4,3,1): - I_customer(2)(1)#9 + I_customer(2)(2)#15
                                 - b_customer(2)(2)#33 - w_customer(2)(2)(0)#75
                                 - w_customer(2)(2)(1)#81
                                 - w_customer(2)(2)(2)#87  = -16
 CustomerInventoryBalance(5,3,1): - I_customer(3)(1)#10 + I_customer(3)(2)#16
                                 - b_customer(3)(2)#34 - w_customer(3)(2)(0)#76
                                 - w_customer(3)(2)(1)#82
                                 - w_customer(3)(2)(2)#88  = -7
 CustomerInventoryBalance(8,3,1): - I_customer(6)(1)#11 + I_customer(6)(2)#17
                                 - b_customer(6)(2)#35 - w_customer(6)(2)(0)#77
                                 - w_customer(6)(2)(1)#83
                                 - w_customer(6)(2)(2)#89  = -24
 CustomerInventoryBalance(9,3,1): - I_customer(7)(1)#12 + I_customer(7)(2)#18
                                 - b_customer(7)(2)#36 - w_customer(7)(2)(0)#78
                                 - w_customer(7)(2)(1)#84
                                 - w_customer(7)(2)(2)#90  = -13
 CustomerInventoryBalance(10,3,1): - I_customer(8)(1)#13 + I_customer(8)(2)#19
                                 - b_customer(8)(2)#37 - w_customer(8)(2)(0)#79
                                 - w_customer(8)(2)(1)#85
                                 - w_customer(8)(2)(2)#91  = -16
 CustomerInventoryBalance(11,3,1): - I_customer(9)(1)#14 + I_customer(9)(2)#20
                                 - b_customer(9)(2)#38 - w_customer(9)(2)(0)#80
                                 - w_customer(9)(2)(1)#86
                                 - w_customer(9)(2)(2)#92  = -25
 CustomerInventoryCapacity(4,1,1): I_customer(2)(0)#3 <= 32
 CustomerInventoryCapacity(5,1,1): I_customer(3)(0)#4 <= 8
 CustomerInventoryCapacity(8,1,1): I_customer(6)(0)#5 <= 106
 CustomerInventoryCapacity(9,1,1): I_customer(7)(0)#6 <= 14
 CustomerInventoryCapacity(10,1,1): I_customer(8)(0)#7 <= 99
 CustomerInventoryCapacity(11,1,1): I_customer(9)(0)#8 <= 89
 CustomerInventoryCapacity(4,2,1): I_customer(2)(1)#9 <= 30
 CustomerInventoryCapacity(5,2,1): I_customer(3)(1)#10 <= 7
 CustomerInventoryCapacity(8,2,1): I_customer(6)(1)#11 <= 112
 CustomerInventoryCapacity(9,2,1): I_customer(7)(1)#12 <= 15
 CustomerInventoryCapacity(10,2,1): I_customer(8)(1)#13 <= 98
 CustomerInventoryCapacity(11,2,1): I_customer(9)(1)#14 <= 89
 CustomerInventoryCapacity(4,3,1): I_customer(2)(2)#15 <= 29
 CustomerInventoryCapacity(5,3,1): I_customer(3)(2)#16 <= 7
 CustomerInventoryCapacity(8,3,1): I_customer(6)(2)#17 <= 108
 CustomerInventoryCapacity(9,3,1): I_customer(7)(2)#18 <= 13
 CustomerInventoryCapacity(10,3,1): I_customer(8)(2)#19 <= 98
 CustomerInventoryCapacity(11,3,1): I_customer(9)(2)#20 <= 85
 vehicleCapacityConstraint(1)1:  w_customer(2)(0)(0)#39
                                 + w_customer(3)(0)(0)#40
                                 + w_customer(6)(0)(0)#41
                                 + w_customer(7)(0)(0)#42
                                 + w_customer(8)(0)(0)#43
                                 + w_customer(9)(0)(0)#44 <= 44
 vehicleCapacityConstraint(1)2:  w_customer(2)(0)(1)#45
                                 + w_customer(3)(0)(1)#46
                                 + w_customer(6)(0)(1)#47
                                 + w_customer(7)(0)(1)#48
                                 + w_customer(8)(0)(1)#49
                                 + w_customer(9)(0)(1)#50 <= 44
 vehicleCapacityConstraint(1)3:  w_customer(2)(0)(2)#51
                                 + w_customer(3)(0)(2)#52
                                 + w_customer(6)(0)(2)#53
                                 + w_customer(7)(0)(2)#54
                                 + w_customer(8)(0)(2)#55
                                 + w_customer(9)(0)(2)#56 <= 44
 vehicleCapacityConstraint(2)1:  w_customer(2)(1)(0)#57
                                 + w_customer(3)(1)(0)#58
                                 + w_customer(6)(1)(0)#59
                                 + w_customer(7)(1)(0)#60
                                 + w_customer(8)(1)(0)#61
                                 + w_customer(9)(1)(0)#62 <= 44
 vehicleCapacityConstraint(2)2:  w_customer(2)(1)(1)#63
                                 + w_customer(3)(1)(1)#64
                                 + w_customer(6)(1)(1)#65
                                 + w_customer(7)(1)(1)#66
                                 + w_customer(8)(1)(1)#67
                                 + w_customer(9)(1)(1)#68 <= 44
 vehicleCapacityConstraint(2)3:  w_customer(2)(1)(2)#69
                                 + w_customer(3)(1)(2)#70
                                 + w_customer(6)(1)(2)#71
                                 + w_customer(7)(1)(2)#72
                                 + w_customer(8)(1)(2)#73
                                 + w_customer(9)(1)(2)#74 <= 44
 vehicleCapacityConstraint(3)1:  w_customer(2)(2)(0)#75
                                 + w_customer(3)(2)(0)#76
                                 + w_customer(6)(2)(0)#77
                                 + w_customer(7)(2)(0)#78
                                 + w_customer(8)(2)(0)#79
                                 + w_customer(9)(2)(0)#80 <= 44
 vehicleCapacityConstraint(3)2:  w_customer(2)(2)(1)#81
                                 + w_customer(3)(2)(1)#82
                                 + w_customer(6)(2)(1)#83
                                 + w_customer(7)(2)(1)#84
                                 + w_customer(8)(2)(1)#85
                                 + w_customer(9)(2)(1)#86 <= 44
 vehicleCapacityConstraint(3)3:  w_customer(2)(2)(2)#87
                                 + w_customer(3)(2)(2)#88
                                 + w_customer(6)(2)(2)#89
                                 + w_customer(7)(2)(2)#90
                                 + w_customer(8)(2)(2)#91
                                 + w_customer(9)(2)(2)#92 <= 44
 deliveryLimitConstraint(4,1,1,1): w_customer(2)(0)(0)#39 <= 0
 deliveryLimitConstraint(5,1,1,1): w_customer(3)(0)(0)#40 <= 14
 deliveryLimitConstraint(8,1,1,1): w_customer(6)(0)(0)#41 <= 0
 deliveryLimitConstraint(9,1,1,1): w_customer(7)(0)(0)#42 <= 26
 deliveryLimitConstraint(10,1,1,1): w_customer(8)(0)(0)#43 <= 0
 deliveryLimitConstraint(11,1,1,1): w_customer(9)(0)(0)#44 <= 0
 deliveryLimitConstraint(4,2,1,1): w_customer(2)(0)(1)#45 <= 0
 deliveryLimitConstraint(5,2,1,1): w_customer(3)(0)(1)#46 <= 0
 deliveryLimitConstraint(8,2,1,1): w_customer(6)(0)(1)#47 <= 44
 deliveryLimitConstraint(9,2,1,1): w_customer(7)(0)(1)#48 <= 0
 deliveryLimitConstraint(10,2,1,1): w_customer(8)(0)(1)#49 <= 0
 deliveryLimitConstraint(11,2,1,1): w_customer(9)(0)(1)#50 <= 44
 deliveryLimitConstraint(4,3,1,1): w_customer(2)(0)(2)#51 <= 0
 deliveryLimitConstraint(5,3,1,1): w_customer(3)(0)(2)#52 <= 0
 deliveryLimitConstraint(8,3,1,1): w_customer(6)(0)(2)#53 <= 0
 deliveryLimitConstraint(9,3,1,1): w_customer(7)(0)(2)#54 <= 0
 deliveryLimitConstraint(10,3,1,1): w_customer(8)(0)(2)#55 <= 0
 deliveryLimitConstraint(11,3,1,1): w_customer(9)(0)(2)#56 <= 0
 deliveryLimitConstraint(4,1,2,1): w_customer(2)(1)(0)#57 <= 31
 deliveryLimitConstraint(5,1,2,1): w_customer(3)(1)(0)#58 <= 14
 deliveryLimitConstraint(8,1,2,1): w_customer(6)(1)(0)#59 <= 0
 deliveryLimitConstraint(9,1,2,1): w_customer(7)(1)(0)#60 <= 24
 deliveryLimitConstraint(10,1,2,1): w_customer(8)(1)(0)#61 <= 0
 deliveryLimitConstraint(11,1,2,1): w_customer(9)(1)(0)#62 <= 0
 deliveryLimitConstraint(4,2,2,1): w_customer(2)(1)(1)#63 <= 0
 deliveryLimitConstraint(5,2,2,1): w_customer(3)(1)(1)#64 <= 0
 deliveryLimitConstraint(8,2,2,1): w_customer(6)(1)(1)#65 <= 0
 deliveryLimitConstraint(9,2,2,1): w_customer(7)(1)(1)#66 <= 0
 deliveryLimitConstraint(10,2,2,1): w_customer(8)(1)(1)#67 <= 0
 deliveryLimitConstraint(11,2,2,1): w_customer(9)(1)(1)#68 <= 0
 deliveryLimitConstraint(4,3,2,1): w_customer(2)(1)(2)#69 <= 0
 deliveryLimitConstraint(5,3,2,1): w_customer(3)(1)(2)#70 <= 0
 deliveryLimitConstraint(8,3,2,1): w_customer(6)(1)(2)#71 <= 0
 deliveryLimitConstraint(9,3,2,1): w_customer(7)(1)(2)#72 <= 0
 deliveryLimitConstraint(10,3,2,1): w_customer(8)(1)(2)#73 <= 0
 deliveryLimitConstraint(11,3,2,1): w_customer(9)(1)(2)#74 <= 0
 deliveryLimitConstraint(4,1,3,1): w_customer(2)(2)(0)#75 <= 16
 deliveryLimitConstraint(5,1,3,1): w_customer(3)(2)(0)#76 <= 0
 deliveryLimitConstraint(8,1,3,1): w_customer(6)(2)(0)#77 <= 0
 deliveryLimitConstraint(9,1,3,1): w_customer(7)(2)(0)#78 <= 0
 deliveryLimitConstraint(10,1,3,1): w_customer(8)(2)(0)#79 <= 0
 deliveryLimitConstraint(11,1,3,1): w_customer(9)(2)(0)#80 <= 0
 deliveryLimitConstraint(4,2,3,1): w_customer(2)(2)(1)#81 <= 0
 deliveryLimitConstraint(5,2,3,1): w_customer(3)(2)(1)#82 <= 0
 deliveryLimitConstraint(8,2,3,1): w_customer(6)(2)(1)#83 <= 0
 deliveryLimitConstraint(9,2,3,1): w_customer(7)(2)(1)#84 <= 0
 deliveryLimitConstraint(10,2,3,1): w_customer(8)(2)(1)#85 <= 0
 deliveryLimitConstraint(11,2,3,1): w_customer(9)(2)(1)#86 <= 0
 deliveryLimitConstraint(4,3,3,1): w_customer(2)(2)(2)#87 <= 0
 deliveryLimitConstraint(5,3,3,1): w_customer(3)(2)(2)#88 <= 0
 deliveryLimitConstraint(8,3,3,1): w_customer(6)(2)(2)#89 <= 0
 deliveryLimitConstraint(9,3,3,1): w_customer(7)(2)(2)#90 <= 0
 deliveryLimitConstraint(10,3,3,1): w_customer(8)(2)(2)#91 <= 0
 deliveryLimitConstraint(11,3,3,1): w_customer(9)(2)(2)#92 <= 0
Bounds
 0 <= I_warehouse(0)(0)#0 <= 136.8
 0 <= I_warehouse(0)(1)#1 <= 136.8
 0 <= I_warehouse(0)(2)#2 <= 136.8
 0 <= I_customer(2)(0)#3 <= 45
 0 <= I_customer(3)(0)#4 <= 14
 0 <= I_customer(6)(0)#5 <= 132
 0 <= I_customer(7)(0)#6 <= 26
 0 <= I_customer(8)(0)#7 <= 114
 0 <= I_customer(9)(0)#8 <= 110
 0 <= I_customer(2)(1)#9 <= 45
 0 <= I_customer(3)(1)#10 <= 14
 0 <= I_customer(6)(1)#11 <= 132
 0 <= I_customer(7)(1)#12 <= 26
 0 <= I_customer(8)(1)#13 <= 114
 0 <= I_customer(9)(1)#14 <= 110
 0 <= I_customer(2)(2)#15 <= 45
 0 <= I_customer(3)(2)#16 <= 14
 0 <= I_customer(6)(2)#17 <= 132
 0 <= I_customer(7)(2)#18 <= 26
 0 <= I_customer(8)(2)#19 <= 114
 0 <= I_customer(9)(2)#20 <= 110
 0 <= b_customer(2)(0)#21 <= 13
 0 <= b_customer(3)(0)#22 <= 6
 0 <= b_customer(6)(0)#23 <= 26
 0 <= b_customer(7)(0)#24 <= 12
 0 <= b_customer(8)(0)#25 <= 15
 0 <= b_customer(9)(0)#26 <= 21
 0 <= b_customer(2)(1)#27 <= 15
 0 <= b_customer(3)(1)#28 <= 7
 0 <= b_customer(6)(1)#29 <= 20
 0 <= b_customer(7)(1)#30 <= 11
 0 <= b_customer(8)(1)#31 <= 16
 0 <= b_customer(9)(1)#32 <= 21
 0 <= b_customer(2)(2)#33 <= 16
 0 <= b_customer(3)(2)#34 <= 7
 0 <= b_customer(6)(2)#35 <= 24
 0 <= b_customer(7)(2)#36 <= 13
 0 <= b_customer(8)(2)#37 <= 16
 0 <= b_customer(9)(2)#38 <= 25
 0 <= w_customer(2)(0)(0)#39 <= 44
 0 <= w_customer(3)(0)(0)#40 <= 14
 0 <= w_customer(6)(0)(0)#41 <= 44
 0 <= w_customer(7)(0)(0)#42 <= 26
 0 <= w_customer(8)(0)(0)#43 <= 44
 0 <= w_customer(9)(0)(0)#44 <= 44
 0 <= w_customer(2)(0)(1)#45 <= 44
 0 <= w_customer(3)(0)(1)#46 <= 14
 0 <= w_customer(6)(0)(1)#47 <= 44
 0 <= w_customer(7)(0)(1)#48 <= 26
 0 <= w_customer(8)(0)(1)#49 <= 44
 0 <= w_customer(9)(0)(1)#50 <= 44
 0 <= w_customer(2)(0)(2)#51 <= 44
 0 <= w_customer(3)(0)(2)#52 <= 14
 0 <= w_customer(6)(0)(2)#53 <= 44
 0 <= w_customer(7)(0)(2)#54 <= 26
 0 <= w_customer(8)(0)(2)#55 <= 44
 0 <= w_customer(9)(0)(2)#56 <= 44
 0 <= w_customer(2)(1)(0)#57 <= 31
 0 <= w_customer(3)(1)(0)#58 <= 14
 0 <= w_customer(6)(1)(0)#59 <= 44
 0 <= w_customer(7)(1)(0)#60 <= 24
 0 <= w_customer(8)(1)(0)#61 <= 32
 0 <= w_customer(9)(1)(0)#62 <= 44
 0 <= w_customer(2)(1)(1)#63 <= 31
 0 <= w_customer(3)(1)(1)#64 <= 14
 0 <= w_customer(6)(1)(1)#65 <= 44
 0 <= w_customer(7)(1)(1)#66 <= 24
 0 <= w_customer(8)(1)(1)#67 <= 32
 0 <= w_customer(9)(1)(1)#68 <= 44
 0 <= w_customer(2)(1)(2)#69 <= 31
 0 <= w_customer(3)(1)(2)#70 <= 14
 0 <= w_customer(6)(1)(2)#71 <= 44
 0 <= w_customer(7)(1)(2)#72 <= 24
 0 <= w_customer(8)(1)(2)#73 <= 32
 0 <= w_customer(9)(1)(2)#74 <= 44
 0 <= w_customer(2)(2)(0)#75 <= 16
 0 <= w_customer(3)(2)(0)#76 <= 7
 0 <= w_customer(6)(2)(0)#77 <= 24
 0 <= w_customer(7)(2)(0)#78 <= 13
 0 <= w_customer(8)(2)(0)#79 <= 16
 0 <= w_customer(9)(2)(0)#80 <= 25
 0 <= w_customer(2)(2)(1)#81 <= 16
 0 <= w_customer(3)(2)(1)#82 <= 7
 0 <= w_customer(6)(2)(1)#83 <= 24
 0 <= w_customer(7)(2)(1)#84 <= 13
 0 <= w_customer(8)(2)(1)#85 <= 16
 0 <= w_customer(9)(2)(1)#86 <= 25
 0 <= w_customer(2)(2)(2)#87 <= 16
 0 <= w_customer(3)(2)(2)#88 <= 7
 0 <= w_customer(6)(2)(2)#89 <= 24
 0 <= w_customer(7)(2)(2)#90 <= 13
 0 <= w_customer(8)(2)(2)#91 <= 16
 0 <= w_customer(9)(2)(2)#92 <= 25
End
