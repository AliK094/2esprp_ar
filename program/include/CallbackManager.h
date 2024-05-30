#ifndef SECCALLBACK_H
#define SECCALLBACK_H

#include "ParameterSetting.h"
extern "C" {
    #include "concorde.h"
}

class SubtourEliminationCallback : public IloCplex::LazyConstraintCallbackI {
public:

    SubtourEliminationCallback(
        IloEnv& env,
        const ParameterSetting& parameters,
        const IloArray<IloArray<IloArray<IloNumVarArray>>>& x,
        const IloArray<IloArray<IloArray<IloNumVarArray>>>& z
    );

    SubtourEliminationCallback(
        IloEnv& env,
        const ParameterSetting& parameters,
        const IloArray<IloArray<IloNumVarArray>>& x,
        const IloArray<IloArray<IloNumVarArray>>& z,
        vector<SAVEDCUTS>* saved_cuts_s = nullptr
    );
    
    SubtourEliminationCallback(
        IloEnv& env,
        const ParameterSetting& parameters,
        const IloArray<IloNumVarArray>& x,
        const IloArray<IloNumVarArray>& z
    );

    SubtourEliminationCallback(
        IloEnv& env,
        const ParameterSetting& parameters,
        const IloNumVarArray& x,
        const IloNumVarArray& z = IloNumVarArray()
    );

    virtual void main() override;

    IloCplex::CallbackI* duplicateCallback() const;

private:
    const ParameterSetting& params;
    IloEnv env;

    CUTSET *myCutset;
    
    vector<SAVEDCUTS>* saved_cuts_ptr;

    int n_nodes;
    int n_edges;

    const IloArray<IloArray<IloArray<IloNumVarArray>>> x_PRPVI;
    const IloArray<IloArray<IloArray<IloNumVarArray>>> z_PRPVI;

    const IloArray<IloArray<IloNumVarArray>> x_CVRP_s;
    const IloArray<IloArray<IloNumVarArray>> z_CVRP_s;
    
    const IloArray<IloNumVarArray> x_VRP;
    const IloArray<IloNumVarArray> z_VRP;

    const IloNumVarArray x_TSP;
    const IloNumVarArray z_TSP;

    IloNumArray x_vals;
    IloNumArray z_vals;

    double separationTolerance;
    double THRESHOLD;
    double minFlow;

    double old_objval;

    bool isTSP = false;
    bool isVRP = false;
    bool isPRPVI = false;
    bool isCVRP_s = false;

    // Helper methods
    bool IsActiveNode(const IloNumArray& z_vals) const;
    double SolveSeparationProblem();
    void AddSubtourEliminationCut(const int vehicleIndex = -1, const int periodIndex = -1, const int scenarioIndex = -1);
};

#endif // SECCALLBACK_H