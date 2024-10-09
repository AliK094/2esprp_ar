\ENCODING=ISO-8859-1
\Problem name: IloCplex

Minimize
 obj1: 588 x(0)#0 + 784 x(1)#1 + 212 x(2)#2
Subject To
 NodeDegree(0): x(0)#0 + x(1)#1  = 2
 NodeDegree(1): x(0)#0 + x(2)#2  = 2
 NodeDegree(2): x(1)#1 + x(2)#2  = 2
Bounds
 0 <= x(0)#0 <= 1
 0 <= x(1)#1 <= 1
 0 <= x(2)#2 <= 1
Binaries
 x(0)#0  x(1)#1  x(2)#2 
End
