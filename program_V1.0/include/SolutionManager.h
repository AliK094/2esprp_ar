#ifndef SOLUTIONMANAGER_H
#define SOLUTIONMANAGER_H

#include "ParameterSetting.h"

class SolutionManager
{
public:
    SolutionManager(const ParameterSetting &parameters, const string solAlg);

    void saveSolution(const Solution &solution);
    bool checkFeasibility();

private:
    ParameterSetting params;
    string Algorithm;
};

#endif // SOLUTIONMANAGER_H