#ifndef CPLEXPARAMETERMANAGER_H
#define CPLEXPARAMETERMANAGER_H

#include "headers.h"

class CplexParameterManager {
public:
    CplexParameterManager(IloCplex& cplex);

    void setParameters(int outputLevel, double mipGapTolerance, int timeLimit, int numThreads, double availableMemory);

private:
    IloCplex& cplex;
    int outputLevel;
    double mipGapTolerance;
    int timeLimit;
    int numThreads;
    double availableMemory;
};

#endif // CPLEXPARAMETERMANAGER_H