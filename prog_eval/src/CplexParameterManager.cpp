#include "CplexParameterManager.h"

CplexParameterManager::CplexParameterManager(IloCplex& cplex): cplex(cplex) 
{}

void CplexParameterManager::setParameters(int outputLevel, double mipGapTolerance, int timeLimit, int numThreads, double availableMemory) {
    this->cplex.setParam(IloCplex::Param::MIP::Display, IloTrue);
    this->cplex.setParam(IloCplex::Param::MIP::Display, outputLevel);
    this->cplex.setParam(IloCplex::Param::TimeLimit, timeLimit);
    this->cplex.setParam(IloCplex::Param::Threads, numThreads);
    this->cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, mipGapTolerance);
    this->cplex.setParam(IloCplex::Param::MIP::Limits::TreeMemory, availableMemory);
    this->cplex.setParam(IloCplex::Param::Emphasis::MIP, 2);
    this->cplex.setParam(IloCplex::Param::Conflict::Display, 2);
    this->cplex.setParam(IloCplex::Param::Conflict::Algorithm, 6);
}
