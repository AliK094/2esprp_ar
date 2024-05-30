#ifndef TSPCONCORDE_H
#define TSPCONCORDE_H

#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "CallbackManager.h"
#include "ilcplex/ilocplex.h"

class TSPConcorde {
public:
    TSPConcorde(vector<vector<double>>& transCostMat);

    bool Solve();

    const vector<double>& getXVarTSP() const {
        return x_TSP;
    }

    const  vector<int>& getRouteTSP() const {
        return routeTSP;
    }

    const  vector<int>& getRoutePositionTSP() const {
        return routeTSP_Poistion;
    }

    const vector<vector<int>>& getVisitMatrixTSP() const {
        return visitMat;
    }

    Result getResult() const {
        return result;
    }

private:
    int numNodes;
    int numEdges;
    double THRESHOLD;
    bool save_lpFile;
    bool save_mpsResultFile;
    vector<double> x_TSP;
    vector<int> routeTSP;
    vector<int> routeTSP_Position;
    vector<vector<int>> visitMat;
    IloNumVarArray x;
    Result result;

    vector<int> index_i;
    vector<int> index_j;
    vector<vector<int>> index_e;

    void initializeIndices();
    void DefineVariables (IloEnv& env, IloModel& model);
    void DefineObjectiveFunction (IloEnv& env, IloModel& model);
    void DefineConstraints (IloEnv& env, IloModel& model);
    void RetrieveSolutions (IloCplex& cplex);
    void DisplayVariables ();

};

#endif // TSPCONCORDE_H