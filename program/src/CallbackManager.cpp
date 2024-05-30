#include "CallbackManager.h"

SubtourEliminationCallback::SubtourEliminationCallback(
    IloEnv &env,
    const ParameterSetting& parameters,
    const IloArray<IloArray<IloArray<IloNumVarArray>>>& x,
    const IloArray<IloArray<IloArray<IloNumVarArray>>>& z
) : env(env),
    IloCplex::LazyConstraintCallbackI(env),
    params(parameters), 
    x_PRPVI(x), 
    z_PRPVI(z),
    x_vals(env, params.numEdges),
    z_vals(env, params.numNodes)
{
    separationTolerance = 1e-6;
    THRESHOLD = 1e-2;
    minFlow = 2.;

    myCutset = new CUTSET[params.numNodes - 1];
    for (int i = 0; i < params.numNodes - 1; i++) {
        myCutset[i].S = new int[params.numNodes];
    }

    isPRPVI = true;
}

SubtourEliminationCallback::SubtourEliminationCallback(
    IloEnv &env,
    const ParameterSetting& parameters,
    const IloArray<IloArray<IloNumVarArray>>& x,
    const IloArray<IloArray<IloNumVarArray>>& z,
    vector<SAVEDCUTS>* saved_cuts_s
) : env(env),
    IloCplex::LazyConstraintCallbackI(env),
    params(parameters), 
    x_CVRP_s(x), 
    z_CVRP_s(z),
    x_vals(env, params.numEdges),
    z_vals(env, params.numNodes),
    saved_cuts_ptr(saved_cuts_s)
{
    // Constructor initialization, if needed
    separationTolerance = 1e-6;
    THRESHOLD = 1e-2;
    minFlow = 2.;

    myCutset = new CUTSET[params.numNodes - 1];
    for (int i = 0; i < params.numNodes - 1; i++) {
        myCutset[i].S = new int[params.numNodes];
    }

    isCVRP_s = true;
}

SubtourEliminationCallback::SubtourEliminationCallback(
    IloEnv &env,
    const ParameterSetting& parameters,
    const IloArray<IloNumVarArray>& x,
    const IloArray<IloNumVarArray>& z
) : env(env),
    IloCplex::LazyConstraintCallbackI(env),
    params(parameters), 
    x_VRP(x), 
    z_VRP(z),
    x_vals(env, params.numEdges),
    z_vals(env, params.numNodes)
{
    // Constructor initialization, if needed
    separationTolerance = 1e-6;
    THRESHOLD = 1e-2;
    minFlow = 2.;

    myCutset = new CUTSET[params.numNodes - 1];
    for (int i = 0; i < params.numNodes - 1; i++) {
        myCutset[i].S = new int[params.numNodes];
    }

    isVRP = true;
}

SubtourEliminationCallback::SubtourEliminationCallback(
    IloEnv &env,
    const ParameterSetting& parameters,
    const IloNumVarArray& x,
    const IloNumVarArray& z
) : env(env),
    IloCplex::LazyConstraintCallbackI(env),
    params(parameters), 
    x_TSP(x), 
    z_TSP(z),
    x_vals(env, params.numEdges)
{
    // Constructor initialization, if needed
    separationTolerance = 1e-6;
    THRESHOLD = 1e-2;
    minFlow = 2.;

    myCutset = new CUTSET[params.numNodes - 1];
    for (int i = 0; i < params.numNodes - 1; i++) {
        myCutset[i].S = new int[params.numNodes];
    }

    isTSP = true;
}

void SubtourEliminationCallback::main() {

    // Get the current node depth of BB tree
    IloInt depth = getCurrentNodeDepth();
    // cout << "depth: " << depth << endl;
    // Retrieve nodeid
    IloCplex::MIPCallbackI::NodeId nodeid = getNodeId();
    // Retrieve the objective value of the current node
	IloNum objval = getObjValue();
	// cout << "objval: " << objval << endl;

    double SECviol;
    old_objval = 0;
    if (isTSP){
        if ((depth == 0 && std::abs(objval - old_objval) > THRESHOLD) || IntegerFeasibility() == CPX_INTEGER_FEASIBLE) {
            for (int edgeIndex = 0; edgeIndex < params.numEdges; edgeIndex++){
                x_vals[edgeIndex] = getValue(x_TSP[edgeIndex]);
            }

            SECviol = SolveSeparationProblem();

            if (SECviol < separationTolerance){
                AddSubtourEliminationCut();
            }
        }
    } else if (isVRP){
        for (int vehicleIndex = 0; vehicleIndex < params.numVehicles; vehicleIndex++){
            if ((depth == 0 && std::abs(objval - old_objval) > THRESHOLD) || IntegerFeasibility() == CPX_INTEGER_FEASIBLE) {
                for (int nodeIndex = 0; nodeIndex < params.numNodes; nodeIndex++){
                    z_vals[nodeIndex] = getValue(z_VRP[nodeIndex][vehicleIndex]);
                }
                if (IsActiveNode(z_vals)) {
                    for (int edgeIndex = 0; edgeIndex < params.numEdges; edgeIndex++){
                        x_vals[edgeIndex] = getValue(x_VRP[edgeIndex][vehicleIndex]);
                    }
                    SECviol = SolveSeparationProblem();

                    if (SECviol < separationTolerance){
                        AddSubtourEliminationCut(vehicleIndex);
                    }
                }
            }
        }
    } else if (isCVRP_s){
        for (int periodIndex = 0; periodIndex < params.numPeriods; periodIndex++){
            for (int vehicleIndex = 0; vehicleIndex < params.numVehicles; vehicleIndex++){
                if ((depth == 0 && std::abs(objval - old_objval) > THRESHOLD) || IntegerFeasibility() == CPX_INTEGER_FEASIBLE) {
                    for (int nodeIndex = 0; nodeIndex < params.numNodes; nodeIndex++){
                        z_vals[nodeIndex] = getValue(z_CVRP_s[nodeIndex][vehicleIndex][periodIndex]);
                    }
                    if (IsActiveNode(z_vals)) {
                        for (int edgeIndex = 0; edgeIndex < params.numEdges; edgeIndex++){
                            x_vals[edgeIndex] = getValue(x_CVRP_s[edgeIndex][vehicleIndex][periodIndex]);
                        }
                        SECviol = SolveSeparationProblem();

                        if (SECviol < separationTolerance){
                            AddSubtourEliminationCut(vehicleIndex, periodIndex);
                        }
                    }
                }
            }
        }
    } else if (isPRPVI){
        for (int scenarioIndex = 0; scenarioIndex < params.numScenarios; scenarioIndex++){
            for (int periodIndex = 0; periodIndex < params.numPeriods; periodIndex++){
                for (int vehicleIndex = 0; vehicleIndex < params.numVehicles; vehicleIndex++){
                    if ((depth == 0 && std::abs(objval - old_objval) > THRESHOLD) || IntegerFeasibility() == CPX_INTEGER_FEASIBLE) {
                        for (int nodeIndex = 0; nodeIndex < params.numNodes; nodeIndex++){
                            z_vals[nodeIndex] = getValue(z_PRPVI[nodeIndex][vehicleIndex][periodIndex][scenarioIndex]);
                        }
                        if (IsActiveNode(z_vals)) {
                            for (int edgeIndex = 0; edgeIndex < params.numEdges; edgeIndex++){
                                x_vals[edgeIndex] = getValue(x_PRPVI[edgeIndex][vehicleIndex][periodIndex][scenarioIndex]);
                            }
                            SECviol = SolveSeparationProblem();

                            if (SECviol < separationTolerance){
                                AddSubtourEliminationCut(vehicleIndex, periodIndex, scenarioIndex);
                            }
                        }
                    }
                }
            }
        }
    } else {
        cout << "No Such SEC has been defined. " << endl; 
    }
}

bool SubtourEliminationCallback::IsActiveNode(const IloNumArray& z_vals) const {
    for (int nodeIndex = 0; nodeIndex < params.numNodes; nodeIndex++){
        if (z_vals[nodeIndex] > THRESHOLD){
            return true;
        }
    }
    return false;
}

double SubtourEliminationCallback::SolveSeparationProblem() {

    vector<int> current_TSP;
    int edges_list[2 * params.numEdges];
    double edges_cap[params.numEdges];

    vector<vector<double>> capacity(params.numNodes, vector<double>(params.numNodes));
    if (isTSP){
        for (int nodeIndex = 0; nodeIndex < params.numNodes; nodeIndex++){
            current_TSP.push_back(nodeIndex);
        }
    } else {
        for (int nodeIndex = 0; nodeIndex < params.numNodes; nodeIndex++){
            if (z_vals[nodeIndex] > THRESHOLD){
                current_TSP.push_back(nodeIndex); 
            }
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
        capacity[i][i] = 0.;
        for (int j = i + 1; j < n_nodes; j++){
            capacity[i][j] = capacity[j][i] = x_vals[params.index_e[current_TSP[i]][current_TSP[j]]];
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
        if (mincut_capacity - 2 < separationTolerance){
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

void SubtourEliminationCallback::AddSubtourEliminationCut(const int vehicleIndex, const int periodIndex, const int scenarioIndex) {
    // The logic to add the subtour elimination cut
    if (isTSP){
        for (int n = 0; n < n_nodes - 1; n++) {
            if (myCutset[n].dim > 2){
                IloExpr cut_expr(env);
                for (int i = 0; i < myCutset[n].dim - 1; i++) {
                    for (int j = i + 1; j < myCutset[n].dim; j++) {
                        cut_expr += x_TSP[params.index_e[myCutset[n].S[i]][myCutset[n].S[j]]];
                        // std::cout << "x_" << myCutset[n].S[i] << ", " << myCutset[n].S[j] << "+ "; 
                    }
                }
                cut_expr += -(myCutset[n].dim - 1);
                // std::cout << " <= " << myCutset[n].dim - 1 << std::endl;
                add(cut_expr <= 0, IloCplex::UseCutForce);
                cut_expr.end();
            }
        }
    } else if (isVRP) {
        for (int n = 0; n < n_nodes - 1; n++) {
            if (myCutset[n].dim > 2){
                IloExpr cut_expr(env);
                for (int i = 0; i < myCutset[n].dim - 1; i++) {
                    for (int j = i + 1; j < myCutset[n].dim; j++) {
                        cut_expr += x_VRP[params.index_e[myCutset[n].S[i]][myCutset[n].S[j]]][vehicleIndex];
                        // std::cout << "x_" << myCutset[n].S[i] << "," << myCutset[n].S[j] << "," << vehicleIndex << " + "; 
                    }
                }
                cut_expr += -(myCutset[n].dim - 1);
                // std::cout << " <= " << myCutset[n].dim - 1 << std::endl;
                add(cut_expr <= 0, IloCplex::UseCutForce);
                cut_expr.end();
            }
        }
    } else if (isCVRP_s) {
        for (int n = 0; n < n_nodes - 1; n++) {
            if (myCutset[n].dim > 2){
                IloExpr cut_expr(env);
                for (int i = 0; i < myCutset[n].dim - 1; i++) {
                    for (int j = i + 1; j < myCutset[n].dim; j++) {
                        cut_expr += x_CVRP_s[params.index_e[myCutset[n].S[i]][myCutset[n].S[j]]][vehicleIndex][periodIndex];
                        // std::cout << "x_" << myCutset[n].S[i] << "," << myCutset[n].S[j] << "," << vehicleIndex << " + ";
                    }
                }
                cut_expr += -(myCutset[n].dim - 1);
                // std::cout << " <= " << myCutset[n].dim - 1 << std::endl;
                add(cut_expr <= 0, IloCplex::UseCutForce);
                cut_expr.end();

                if (saved_cuts_ptr != nullptr) {
                    SAVEDCUTS new_cut;
                    new_cut.veh_ind = vehicleIndex;
                    new_cut.per_ind = periodIndex;
                    new_cut.edge_ind = vector<int>(params.numEdges, 0);
                    for (int i = 0; i < myCutset[n].dim - 1; i++) {
                        for (int j = i + 1; j < myCutset[n].dim; j++) {
                            new_cut.edge_ind[params.index_e[myCutset[n].S[i]][myCutset[n].S[j]]] = 1;
                        }
                    }
                    new_cut.cut_rhs = myCutset[n].dim - 1;
                    saved_cuts_ptr->push_back(new_cut);
                }
            }
        }
    } else if (isPRPVI) {
        for (int n = 0; n < n_nodes - 1; n++) {
            if (myCutset[n].dim > 2){
                IloExpr cut_expr(env);
                for (int i = 0; i < myCutset[n].dim - 1; i++) {
                    for (int j = i + 1; j < myCutset[n].dim; j++) {
                        cut_expr += x_PRPVI[params.index_e[myCutset[n].S[i]][myCutset[n].S[j]]][vehicleIndex][periodIndex][scenarioIndex];
                        // std::cout << "x_" << myCutset[n].S[i] << "," << myCutset[n].S[j] << "," << vehicleIndex << "," << periodIndex << "," << scenarioIndex << " + "; 
                    }
                }
                cut_expr += -(myCutset[n].dim - 1);
                // std::cout << " <= " << myCutset[n].dim - 1 << std::endl;
                add(cut_expr <= 0, IloCplex::UseCutForce);
                cut_expr.end();
            }
        }
    }
}

IloCplex::CallbackI* SubtourEliminationCallback::duplicateCallback() const {
    return new SubtourEliminationCallback(*this);
}