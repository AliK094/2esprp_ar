#ifndef StochBCCALLBACK_H
#define StochBCCALLBACK_H

#include "ParameterSetting.h"
extern "C" {
    #include "concorde.h"
}

class SEC_S2EPRP : public IloCplex::LazyConstraintCallbackI {
public:
    SEC_S2EPRP(
        IloEnv& env,
        const ParameterSetting& parameters,
        const IloArray<IloArray<IloArray<IloNumVarArray>>>& x,
        const IloArray<IloArray<IloArray<IloNumVarArray>>>& z
    );

    virtual void main() override;

    IloCplex::CallbackI* duplicateCallback() const;

private:
    const ParameterSetting& params;
    IloEnv env;

    CUTSET *myCutset;
    
    int n_nodes;
    int n_edges;

    const IloArray<IloArray<IloArray<IloNumVarArray>>> x;
    const IloArray<IloArray<IloArray<IloNumVarArray>>> z;

    IloNumArray x_vals;
    IloNumArray z_vals;

    double sepTol;
    double THRESHOLD;
    double minFlow;
    double old_objval;

    // Helper methods
    bool IsActiveNode(const IloNumArray& z_vals) const;
    double SolveSeparationProblem();
    void AddSubtourEliminationCut(const int k = -1, const int t = -1, const int s = -1);
};

#endif // StochBCCALLBACK_H