#include "TSPCallbackManager.h"

SEC_TSP::SEC_TSP(
    IloEnv &env,
    const IloNumVarArray& x,
    const int numNodes,
    const int numEdges,
    const std::vector<int>& index_i, 
    const std::vector<int>& index_j, 
    const std::vector<std::vector<int>>& index_e
) : env(env),
    IloCplex::LazyConstraintCallbackI(env),
    x(x),
    numNodes(numNodes),
    numEdges(numEdges),
    index_i(index_i),
    index_j(index_j),
    index_e(index_e),
    x_vals(env, numEdges),
    SepTol(1e-6),
    THRESHOLD(1e-2),
    minFlow(2.0),
    old_objval(1e12)
{
    myCutset = new CUTSET[numNodes - 1];
    for (int i = 0; i < numNodes - 1; i++) {
        myCutset[i].S = new int[numNodes];
    }
}

void SEC_TSP::main() {
    // Get the current node depth of BB tree
    IloInt depth = getCurrentNodeDepth();
    // cout << "depth: " << depth << endl;
    // Retrieve nodeid
    IloCplex::MIPCallbackI::NodeId nodeid = getNodeId();
    // Retrieve the objective value of the current node
	IloNum objval = getObjValue();
	// cout << "objval: " << objval << endl;

    double SECviol;
    if ((depth == 0 && std::abs(objval - old_objval) > THRESHOLD) || IntegerFeasibility() == CPX_INTEGER_FEASIBLE) {
        for (int edgeIndex = 0; edgeIndex < numEdges; edgeIndex++){
            x_vals[edgeIndex] = getValue(x[edgeIndex]);
        }

        SECviol = solveSeparationProblem();

        if (SECviol < SepTol){
            addSubtourEliminationCut();
        }
    }
    else {
        std::cout << "No Such SEC has been defined. " << std::endl; 
    }
}

double SEC_TSP::solveSeparationProblem() {

    int edges_list[2 * numEdges];
    double edges_cap[numEdges];

    std::vector<std::vector<double>> capacity(numNodes, std::vector<double>(numNodes));
    
    n_nodes = numNodes;
    n_edges = 0;

    for (int i = 0; i < n_nodes - 1; i++) {
        myCutset[i].dim = 0;
        for (int j = 0; j < n_nodes; j++) {
            myCutset[i].S[j] = 0;
        }
    }

    for (int i = 0; i < n_nodes; i++){
        capacity[i][i] = 0.;
        for (int j = i + 1; j < n_nodes; j++){
            capacity[i][j] = capacity[j][i] = x_vals[index_e[i][j]];
            if (capacity[i][j] > 1e-2){
                edges_list[2*n_edges] = i;
                edges_list[2*n_edges + 1] = j;
                edges_cap[n_edges++] = capacity[i][j];
            }
        }
    }

    int 	n_nodes_cut;
    double	mincut_capacity;
    int 	*cut = NULL;

    int m = 0;
    for (int node = 1; node < n_nodes; node++){
        int status = CCcut_mincut_st(n_nodes, n_edges, edges_list, edges_cap, 0, node, &mincut_capacity, &cut, &n_nodes_cut);
        if (status){		
            std::cout << "Failed to run Concorde." << std::endl;
        }
        if (mincut_capacity - 2 < SepTol){
            for (int j = 0; j < n_nodes_cut; j++) {
                myCutset[m].S[j] = cut[j];
            }
            myCutset[m].dim = n_nodes_cut;
            if (mincut_capacity < minFlow){
                minFlow = mincut_capacity;
            }
            m++;		
        }
    }

    return minFlow;
}

void SEC_TSP::addSubtourEliminationCut() {
    // The logic to add the subtour elimination cut
    for (int n = 0; n < n_nodes - 1; n++) {
        if (myCutset[n].dim > 2){
            IloExpr cut_expr(env);
            for (int i = 0; i < myCutset[n].dim - 1; i++) {
                for (int j = i + 1; j < myCutset[n].dim; j++) {
                    cut_expr += x[index_e[myCutset[n].S[i]][myCutset[n].S[j]]];
                    // std::cout << "x_" << myCutset[n].S[i] << ", " << myCutset[n].S[j] << "+ "; 
                }
            }
            cut_expr += -(myCutset[n].dim - 1);
            // std::cout << " <= " << myCutset[n].dim - 1 << std::endl;
            add(cut_expr <= 0, IloCplex::UseCutForce);
            cut_expr.end();
        }
    }
}
    
IloCplex::CallbackI* SEC_TSP::duplicateCallback() const {
    return new SEC_TSP(*this);
}