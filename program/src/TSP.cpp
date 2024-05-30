#include "TSP.h"

TSP::TSP(vector<vector<double>>& transCostMat)
	: numNodes(costMatrix.size()), 
      numEdges(numNodes * (numNodes - 1) / 2), 
	  index_i(numEdges, 0),
      index_j(numEdges, 0),
      index_e(numNodes, vector<int>(numNodes, 0)),
      x_TSP(numEdges, 0.0), 
      routeTSP(1, 0), 
      routeTSP_Position(numNodes, 0), 
      visitMat(numNodes, vector<int>(numNodes, 0)), 
      THRESHOLD(1e-2), 
      save_lpFile(true), 
      save_mpsResultFile(true) 
{
	if (costMatrix.empty() || costMatrix[0].size() != numNodes) {
        throw invalid_argument("Invalid cost matrix: it should be a non-empty square matrix.");
    }

}

void TSP::initializeIndices()
{
    int e = 0;
    for (int i = 0; i < numNodes - 1; ++i)
    {
        for (int j = i + 1; j < numNodes; ++j)
        {
            index_i[e] = i;
            index_j[e] = j;
            index_e[i][j] = e;
            index_e[j][i] = e;
            e++;
        }
    }
    assert(e == numEdges);

    for (int i = 0; i < numNodes; ++i)
    {
        index_e[i][i] = NONE;
    }
}

void TSP::DefineVariables (IloEnv& env, IloModel& model) {
	// Define Decision Variables
	VariableManager varManager(env);
	x = varManager.create1D(numEdges);

	for (int edgeIndex = 0; edgeIndex < numEdges; edgeIndex++) {
		string varName = "x[" + to_string(edgeIndex) + "]";
		x[edgeIndex] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
		model.add(x[edgeIndex]);
	}
}

void TSP::DefineObjectiveFunction (IloEnv& env, IloModel& model) {
	// Define objective function
	IloExpr obj(env);
	for (int edgeIndex = 0; edgeIndex < numEdges; edgeIndex++) {
		obj += transCostMat[index_i[edgeIndex]][index_j[edgeIndex]] * x[edgeIndex];
	}
	model.add(IloMinimize(env, obj));
	obj.end();
}

void TSP::DefineConstraints (IloEnv& env, IloModel& model) {
	// Add constraints
	// Node Degree Constraints
	for (int nodeIndex = 0; nodeIndex < numNodes; nodeIndex++){
		string constraintName = "NodeDegree(" + to_string(nodeIndex) + ")";

		IloExpr expr(env);
		for (int edgeIndex = 0; edgeIndex < numEdges; edgeIndex++) {
			if (index_i[edgeIndex] == nodeIndex || index_j[edgeIndex] == nodeIndex){
				expr += x[edgeIndex];
			}
		}
		IloConstraint nodeDegreeConstraint(expr == 2);
		expr.end();

		model.add(nodeDegreeConstraint).setName(constraintName.c_str());
	}
}

bool TSP::Solve()
{
	try {
		initializeIndices();
		// Create CPLEX model

		IloEnv env;
		IloModel model(env);
		IloCplex cplex(model);

		auto startTime = std::chrono::high_resolution_clock::now();

		// Set CPLEX Parameters: (DISPLAY LEVEL(0,1,2,3,4), OPTIMALITY GAP, RUN TIME (SECS), THREADS, MEMORY (MB))
		CplexParameterManager parameterManager(cplex);
		parameterManager.setParameters(0, 1e-6, 600, 1, 3000);

		DefineVariables(env, model);
		DefineObjectiveFunction(env, model);
		DefineConstraints(env, model);

		/* Assure linear mappings between the presolved and original models */
		cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
		/* Turn on traditional search for use with control callbacks */
		cplex.setParam(IloCplex::Param::MIP::Strategy::Search, CPX_MIPSEARCH_TRADITIONAL);
		/* Let MIP callbacks work on the original model */
		cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);

		// Create an instance of the callback manager
		SubtourEliminationCallback callback(env, params, x);
    	cplex.use(&callback);

		if (save_lpFile == true){
			// Export the model to an LP file
			string lpFileName = "../cplexFiles/lpModel/TSP_N" + to_string(numNodes) + "_Ins" + instance + ".lp";
			cplex.exportModel(lpFileName.c_str());
			cout << "LP file created successfully." << endl;
		}

		// Extract model
		cplex.extract(model);
		// Solve the model
		cplex.solve();



		if (cplex.getStatus() == IloAlgorithm::Optimal){
            result.status = "Optimal";
			result.objValue = cplex.getObjValue();
			cout << "Optimal solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue << endl;
			result.optimalityGap = cplex.getMIPRelativeGap() * 100;
			result.lowerBound = cplex.getBestObjValue();

			if (save_mpsResultFile == true){
				// Write the solution to a file in MPS format
				string resultsFileName = "../cplexFiles/solVal/TSP_N" + to_string(numNodes) + "_Ins" + instance + ".mps";
				cplex.writeSolution(resultsFileName);
			}
        } else if (cplex.getStatus() == IloAlgorithm::Feasible) {
            result.status = "Incumbent";
			result.objValue = cplex.getObjValue();
			cout << "Incumbent solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue << endl;
			result.optimalityGap = cplex.getMIPRelativeGap() * 100;
			cout << "Optimality gap: " << result.optimalityGap << "%" << endl;
			result.lowerBound = cplex.getBestObjValue();
			cout << "Lower bound: " << result.lowerBound << endl;

			if (save_mpsResultFile == true){
				// Write the solution to a file in MPS format
				string resultsFileName = "../cplexFiles/solVal/TSP_N" + to_string(numNodes) + "_Ins" + instance + ".mps";
				cplex.writeSolution(resultsFileName);
			}
        } else if (cplex.getStatus() == IloAlgorithm::Infeasible) {
            result.status = "Infeasible";
			cout << "Problem is infeasible" << endl;

			IloConstraintArray conflictConstraints(env);
			cplex.getConflict(conflictConstraints);

			IloCplex::ConflictStatusArray conflictStatus(env);
			for (int i = 0; i < conflictConstraints.getSize(); i++) {
				conflictStatus.add(cplex.getConflict(conflictConstraints[i]));
			}

			for (int i = 0; i < conflictConstraints.getSize(); i++) {
				cout << "conflict: " << conflictConstraints[i] << endl;
			}

			cout << "Conflict constraints:" << endl;
			cout << conflictConstraints << endl;

			cout << "Conflict status:" << endl;
			cout << conflictStatus << endl;

			throw std::runtime_error("Solver terminated with infeasible solution.");

        } else {
            result.status = "Undefined";
			cout << "Solver terminated with status: " << result.status << endl;
        }


		auto currentTime = std::chrono::high_resolution_clock::now();
		result.CPUtime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
		cout << "Timer (seconds): " << std::fixed << std::setprecision(4) << result.CPUtime << endl;

		RetrieveSolutions(cplex);
		DisplayVariables();

		env.end();
	} catch (const IloException& e) {
        std::cerr << "Error: " << e << endl;
		return result.success = false;
	} catch (const std::runtime_error& e) {
        std::cerr << "Runtime Error: " << e.what() << endl;
        return result.success = false;
    }

	return result.success = true;
}

void TSP::RetrieveSolutions (IloCplex& cplex) {
	//Get solution values of decision variables
	for (int edgeIndex = 0; edgeIndex < numEdges; edgeIndex++) {
		x_TSP[edgeIndex] = cplex.getIntValue(x[edgeIndex]);
	}

	vector<bool> visited (numNodes, false);
	int currentNode = 0;
	int index = 1;
	while (index < numNodes){
		for (int i = 1; i < numNodes; i++){
			if (x_TSP[index_e[currentNode][i]] > THRESHOLD && visited[i] == false) {
				// cout << currentNode << endl;
				currentNode = i;
				visited[i] = true;
				routeTSP.push_back(currentNode);
				routeTSP_Poistion[currentNode] = index;
				index++;
				break;
			}
		}
	}
	currentNode = 0;
	visited[0] = true;
	routeTSP.push_back(currentNode);


	/*
	in visit_seq matrix, elements with value 1 in row i show the nodes that must be visited after i
	and elemenets with value 1 in column i show the nodes that must be visited before i
	ex. for route 0-3-2-1-0 the matrix is:
			0	1	1	1
			1	0	0	0
			1	1	0	0
			1	1	1	0
	*/

	for (int i = 1; i < numNodes; i++){
		visitMat[i][i] = 0;
	}

	for (int i = 1; i < numNodes; i++){
		visitMat[0][i] = 1;
		visitMat[i][0] = 1;
	}
	for (int i = 1; i < numNodes; i++){
		for (int j = 1; j < numNodes; j++){
			if (routeTSP_Poistion[j] > routeTSP_Poistion[i]){
				visitMat[i][j] = 1;
			}		
		}		
	}
}

void TSP::DisplayVariables() {
	bool print_route = true;
	bool print_visitMat = false;

	if (print_route == true) {
		cout << "\nroute_TSP: ";
		for (int i = 0; i < routeTSP.size(); i++) {
			cout << routeTSP[i] << " ";
		}
		cout << "\n" << endl;
	}


	if (print_visitMat == true) {
		cout << "nd: ";
		for (int i = 0; i < numNodes; i++){
			if (i < 10){
				cout << i << "  ";
			} else {
				cout << i << " ";
			}
		}
		cout << endl;

		for (int i = 0; i < numNodes; i++){
			if (i < 10){
				cout << i << ":  ";
			} else {
				cout << i << ": ";
			}

			for(int j = 0; j < numNodes; j++){
				cout << visitMat[i][j] << "  ";
			}
			cout << endl;
		}
	}
}
