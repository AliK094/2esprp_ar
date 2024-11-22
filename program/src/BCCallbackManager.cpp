#include "BCCallbackManager.h"

SEC_S2EPRP::SEC_S2EPRP(
    IloEnv &env,
    const ParameterSetting& parameters,
    const IloArray<IloArray<IloArray<IloNumVarArray>>>& x,
    const IloArray<IloArray<IloArray<IloNumVarArray>>>& z
) : env(env),
    IloCplex::LazyConstraintCallbackI(env),
    params(parameters), 
    x(x), 
    z(z),
    x_vals(env, params.numEdges_SecondEchelon),
    z_vals(env, params.numNodes_SecondEchelon),
    sepTol(1e-6),
    THRESHOLD(1e-6),
    minFlow(2.0),
    old_objval(1e12)
{
    myCutset = new CUTSET[params.numNodes_SecondEchelon - 1];
    for (int i = 0; i < params.numNodes_SecondEchelon - 1; i++) {
        myCutset[i].S = new int[params.numNodes_SecondEchelon];
    }
}

void SEC_S2EPRP::main() {

    // myCutset = new CUTSET[params.numNodes_SecondEchelon - 1];
    // for (int i = 0; i < params.numNodes_SecondEchelon - 1; i++) {
    //     myCutset[i].S = new int[params.numNodes_SecondEchelon];
    // }

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
        for (int s = 0; s < params.numScenarios; s++){
            for (int t = 0; t < params.numPeriods; t++){
                for (int k = 0; k < params.numVehicles_SecondEchelon; k++){
                    for (int i = 0; i < params.numNodes_SecondEchelon; i++){
                        z_vals[i] = getValue(z[i][k][t][s]);
                    }
                    if (IsActiveNode(z_vals)) {
                        for (int e = 0; e < params.numEdges_SecondEchelon; e++){
                            x_vals[e] = getValue(x[e][k][t][s]);
                        }
                        SECviol = SolveSeparationProblem();

                        if (SECviol < sepTol){
                            AddSubtourEliminationCut(k, t, s);
                        }
                    }
                }
            }
        }
    }
}

bool SEC_S2EPRP::IsActiveNode(const IloNumArray& z_vals) const {
    for (int i = 0; i < params.numNodes_SecondEchelon; i++){
        if (z_vals[i] > THRESHOLD){
            return true;
        }
    }
    return false;
}

double SEC_S2EPRP::SolveSeparationProblem() 
{
    vector<int> current_TSP; 
    int edges_list[2 * params.numEdges_SecondEchelon];
    double edges_cap[params.numEdges_SecondEchelon];

    vector<vector<double>> capacity(params.numNodes_SecondEchelon, vector<double>(params.numNodes_SecondEchelon));
    for (int i = 0; i < params.numNodes_SecondEchelon; i++){
        if (z_vals[i] > THRESHOLD){
            current_TSP.push_back(i); 
        }
    }
    n_nodes = current_TSP.size();
    n_edges = 0;

    for (int i = 0; i < n_nodes - 1; i++) {
        myCutset[i].dim = 0;
        for (int j = 0; j < n_nodes; j++) {
            myCutset[i].S[j] = 0;
        }
    }

    for (int i = 0; i < n_nodes; i++){
        capacity[i][i] = 0.0;
        for (int j = i + 1; j < n_nodes; j++){
            capacity[i][j] = capacity[j][i] = x_vals[params.index_e_SecondEchelon[current_TSP[i]][current_TSP[j]]];
            if (capacity[i][j] > THRESHOLD){
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
        if (mincut_capacity - 2 < sepTol){
            for (int j = 0; j < n_nodes_cut; j++) {
                myCutset[m].S[j] = current_TSP[cut[j]];
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

void SEC_S2EPRP::AddSubtourEliminationCut(const int k, const int t, const int s) {
    for (int n = 0; n < n_nodes - 1; n++) {
        if (myCutset[n].dim > 2){
            IloExpr cut_expr(env);
            for (int i = 0; i < myCutset[n].dim - 1; i++) {
                for (int j = i + 1; j < myCutset[n].dim; j++) {
                    cut_expr += x[params.index_e_SecondEchelon[myCutset[n].S[i]][myCutset[n].S[j]]][k][t][s];
                    // std::cout << "x_" << myCutset[n].S[i] << "," << myCutset[n].S[j] << "," << k << "," << t << "," << s << " + "; 
                }
            }
            cut_expr += -(myCutset[n].dim - 1);
            // std::cout << " <= " << myCutset[n].dim - 1 << std::endl;
            add(cut_expr <= 0, IloCplex::UseCutForce);
            cut_expr.end();
        }
    }
}

IloCplex::CallbackI* SEC_S2EPRP::duplicateCallback() const {
    return new SEC_S2EPRP(*this);
}