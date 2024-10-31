#ifndef TSP_SECCALLBACK_H
#define TSP_SECCALLBACK_H

#include "headers.h"
extern "C" {
    #include "concorde.h"
}

class SEC_TSP : public IloCplex::LazyConstraintCallbackI {
public:
    SEC_TSP (
        IloEnv &env,
        const IloNumVarArray& x,
        const int numNodes,
        const int numEdges,
        const std::vector<int>& index_i, 
        const std::vector<int>& index_j, 
        const std::vector<std::vector<int>>& index_e
    );

    virtual void main() override;

    IloCplex::CallbackI* duplicateCallback() const;

private:
    IloEnv env;
    int numNodes;
    int numEdges;
    std::vector<int> index_i;
    std::vector<int> index_j;
    std::vector<std::vector<int>> index_e;

    CUTSET *myCutset;
    
    std::vector<SAVEDCUTS>* saved_cuts_ptr;

    int n_nodes;
    int n_edges;

    const IloNumVarArray x;
    IloNumArray x_vals;

    double SepTol;
    double THRESHOLD;
    double minFlow;
    double old_objval;

    // Helper methods
    double solveSeparationProblem();
    void addSubtourEliminationCut();
};

#endif // SECCALLBACK_H