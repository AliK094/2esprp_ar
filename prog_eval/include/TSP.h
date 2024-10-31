#ifndef TSPCONCORDE_H
#define TSPCONCORDE_H

#include "headers.h"
#include "VariableManager.h"
#include "CplexParameterManager.h"
#include "TSPCallbackManager.h"

class TSP {
public:
    explicit TSP(std::vector<std::vector<double>>& transCostMat);

    bool solve();

    const std::vector<double>& getXVarTSP() const {
        return x_TSP;
    }

    const std::vector<int>& getRouteTSP() const {
        return routeTSP;
    }

    const std::vector<int>& getRoutePositionTSP() const {
        return routeTSP_Position;
    }

    const std::vector<std::vector<int>>& getVisitMatrixTSP() const {
        return visitMat;
    }

    double getObjValue() const {
        return objValue;
    }

private:
    std::vector<std::vector<double>> transCostMat;

    string status;
    double objValue;
    double optimalityGap;
    double lowerBound;

    int numNodes;
    int numEdges;
    double THRESHOLD;
    bool save_lpFile;
    bool save_mpsResultFile;
    std::vector<double> x_TSP;
    std::vector<int> routeTSP;
    std::vector<int> routeTSP_Position;
    std::vector<std::vector<int>> visitMat;
    IloNumVarArray x;
    Result result;

    std::vector<int> index_i;
    std::vector<int> index_j;
    std::vector<std::vector<int>> index_e;

    void initializeIndices();
    void DefineVariables(IloEnv& env, IloModel& model);
    void DefineObjectiveFunction(IloEnv& env, IloModel& model);
    void DefineConstraints(IloEnv& env, IloModel& model);
    void RetrieveSolutions(IloCplex& cplex);
    void DisplayVariables();
};

#endif // TSPCONCORDE_H
