#ifndef SOLUTIONMANAGER_H
#define SOLUTIONMANAGER_H

#include "ParameterSetting.h"

class SolutionManager
{
public:
    SolutionManager(const ParameterSetting &parameters);

    void saveSolution(const SolutionFirstEchelon& solFE, const SolutionSecondEchelon &solSE);
    void saveResultSummary(const SolutionFirstEchelon &solFE, const SolutionSecondEchelon &solSE, const Result result);
    bool checkFeasibility();

    void saveSolution_Deterministic(const SolutionFirstEchelon& solFE, const SolutionSecondEchelon_Deterministic &solSE, string solAlg);
    bool checkFeasibility_Deterministic(string solAlg);
    void saveResultSummary_Deterministic(const SolutionFirstEchelon &solFE, const SolutionSecondEchelon_Deterministic &solSE, const Result result, string solAlg);

private:
    ParameterSetting params;
};

#endif // SOLUTIONMANAGER_H