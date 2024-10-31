#ifndef SOLUTIONMANAGER_H
#define SOLUTIONMANAGER_H

#include "ParameterSetting.h"

class SolutionManager
{
public:
    SolutionManager(const ParameterSetting &parameters, const string solAlg);

    void saveSolution(const SolutionFirstEchelon& solFE, const SolutionSecondEchelon &solSE);
    void saveResultSummary(const SolutionFirstEchelon &solFE, const SolutionSecondEchelon &solSE, const Result result);
    bool checkFeasibility();

private:
    ParameterSetting params;
    string Algorithm;
};

#endif // SOLUTIONMANAGER_H