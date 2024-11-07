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

    void saveSolution_Deterministic(const SolutionFirstEchelon& solFE, 
                                    const SolutionSecondEchelon_Deterministic &solSE,
                                    const vector<vector<double>> &deterministicDemand,
                                    bool shortageAllowed = true,
                                    int scenarioIndex = -1);
    bool checkFeasibility_Deterministic(bool shortageAllowed = true, int scenarioIndex = -1);
    void saveResultSummary_Deterministic(const SolutionFirstEchelon &solFE, 
                                         const SolutionSecondEchelon_Deterministic &solSE, 
                                         const Result result,
                                         bool shortageAllowed = true,
                                         int scenarioIndex = -1);



private:
    ParameterSetting params;
    string Algorithm;
};

#endif // SOLUTIONMANAGER_H