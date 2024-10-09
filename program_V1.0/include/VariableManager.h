#ifndef VARIABLEMANAGER_H
#define VARIABLEMANAGER_H

#include "headers.h"

class VariableManager {
public:
    VariableManager(IloEnv &env);
    IloNumVarArray create1D(int size);
    IloArray<IloNumVarArray> create2D(int rows, int cols);
    IloArray<IloArray<IloNumVarArray>> create3D(int dim1, int dim2, int dim3);
    IloArray<IloArray<IloArray<IloNumVarArray>>> create4D(int dim1, int dim2, int dim3, int dim4);
    IloArray<IloArray<IloArray<IloArray<IloNumVarArray>>>> create5D(int dim1, int dim2, int dim3, int dim4, int dim5);
    
private:
    IloEnv env;
};

#endif // VARIABLEMANAGER_H
