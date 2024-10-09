#include "VariableManager.h"

VariableManager::VariableManager(IloEnv &env): env(env) {
    // Construct: Implement the logic to read and assign parameter values
}

IloNumVarArray VariableManager::create1D(int size) {
	IloNumVarArray vars(this->env, size);
	
	return vars;
}

IloArray<IloNumVarArray> VariableManager::create2D(int rows, int cols) {
	IloArray<IloNumVarArray> vars(this->env, rows);
	for (int i = 0; i < rows; i++) {
		vars[i] = create1D(cols);
	}
	return vars;
}

IloArray<IloArray<IloNumVarArray>> VariableManager::create3D(int dim1, int dim2, int dim3) {
	IloArray<IloArray<IloNumVarArray>> vars(this->env, dim1);
	for (int i = 0; i < dim1; i++) {
		vars[i] = create2D(dim2, dim3);
	}
	return vars;
}

IloArray<IloArray<IloArray<IloNumVarArray>>> VariableManager::create4D(int dim1, int dim2, int dim3, int dim4) {
	IloArray<IloArray<IloArray<IloNumVarArray>>> vars(this->env, dim1);
	for (int i = 0; i < dim1; i++) {
		vars[i] = create3D(dim2, dim3, dim4);
	}
	return vars;
}

IloArray<IloArray<IloArray<IloArray<IloNumVarArray>>>> VariableManager::create5D(int dim1, int dim2, int dim3, int dim4, int dim5) {
	IloArray<IloArray<IloArray<IloArray<IloNumVarArray>>>> vars(this->env, dim1);
	for (int i = 0; i < dim1; i++) {
		vars[i] = create4D(dim2, dim3, dim4, dim5);
	}
	return vars;
}
