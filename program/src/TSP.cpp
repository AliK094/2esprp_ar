#include "TSP.h"

TSP::TSP(std::vector<std::vector<double>>& transCostMat)
    : transCostMat(transCostMat),
	  numNodes(transCostMat.size()), 
      numEdges(numNodes * (numNodes - 1) / 2), 
      index_i(numEdges, 0),
      index_j(numEdges, 0),
      index_e(numNodes, std::vector<int>(numNodes, 0)),
      x_TSP(numEdges, 0.0), 
      routeTSP(1, 0), 
      routeTSP_Position(numNodes, 0), 
      visitMat(numNodes, std::vector<int>(numNodes, 0)), 
      THRESHOLD(1e-2), 
      save_lpFile(true), 
      save_mpsResultFile(true) 
{
    if (transCostMat.empty() || transCostMat[0].size() != numNodes) {
        throw std::invalid_argument("Invalid cost matrix: it should be a non-empty square matrix.");
    }
    initializeIndices();
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

bool TSP::solve()
{
    try {
        initializeIndices();
        
        // Create CPLEX environment and model
        IloEnv env;
        IloModel model(env);
        IloCplex cplex(model);

        auto startTime = std::chrono::high_resolution_clock::now();

        // Set CPLEX Parameters
        CplexParameterManager parameterManager(cplex);
        parameterManager.setParameters(0, 1e-6, 600, 1, 3000);

        DefineVariables(env, model);
        DefineObjectiveFunction(env, model);
        DefineConstraints(env, model);

        // Set CPLEX parameters
        cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
        cplex.setParam(IloCplex::Param::MIP::Strategy::Search, CPX_MIPSEARCH_TRADITIONAL);
        cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);

        // Create an instance of the callback manager
		// SEC_TSP LegacyCallback(env, numNodes, x);

        SEC_TSP LegacyCallback(env, x, numNodes, numEdges, index_i, index_j, index_e);
        cplex.use(&LegacyCallback);

        if (save_lpFile) {
            // Export the model to an LP file
            std::string lpFileName = "../cplexFiles/lpModel/TSP_N" + std::to_string(numNodes) + ".lp";
            cplex.exportModel(lpFileName.c_str());
            std::cout << "LP file created successfully." << std::endl;
        }

        // Extract model and solve
        cplex.extract(model);
        cplex.solve();

        if (cplex.getStatus() == IloAlgorithm::Optimal) {
            result.status = "Optimal";
            result.objValue = cplex.getObjValue();
            std::cout << "Optimal solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue << std::endl;
            result.optimalityGap = cplex.getMIPRelativeGap() * 100;
            result.lowerBound = cplex.getBestObjValue();

            if (save_mpsResultFile) {
                // Write the solution to a file in MPS format
                std::string resultsFileName = "../cplexFiles/solVal/TSP_N" + std::to_string(numNodes) + ".mps";
                cplex.writeSolution(resultsFileName.c_str());
            }
        } else if (cplex.getStatus() == IloAlgorithm::Feasible) {
            result.status = "Incumbent";
            result.objValue = cplex.getObjValue();
            std::cout << "Incumbent solution found with objective value: " << std::fixed << std::setprecision(1) << result.objValue << std::endl;
            result.optimalityGap = cplex.getMIPRelativeGap() * 100;
            std::cout << "Optimality gap: " << result.optimalityGap << "%" << std::endl;
            result.lowerBound = cplex.getBestObjValue();
            std::cout << "Lower bound: " << result.lowerBound << std::endl;

            if (save_mpsResultFile) {
                // Write the solution to a file in MPS format
                std::string resultsFileName = "../cplexFiles/solVal/TSP_N" + std::to_string(numNodes) + ".mps";
                cplex.writeSolution(resultsFileName.c_str());
            }
        } else if (cplex.getStatus() == IloAlgorithm::Infeasible) {
            result.status = "Infeasible";
            std::cout << "Problem is infeasible" << std::endl;

            IloConstraintArray conflictConstraints(env);
            cplex.getConflict(conflictConstraints);

            IloCplex::ConflictStatusArray conflictStatus(env);
            for (int i = 0; i < conflictConstraints.getSize(); i++) {
                conflictStatus.add(cplex.getConflict(conflictConstraints[i]));
            }

            std::cout << "Conflict constraints:" << std::endl;
            std::cout << conflictConstraints << std::endl;

            std::cout << "Conflict status:" << std::endl;
            std::cout << conflictStatus << std::endl;

            throw std::runtime_error("Solver terminated with infeasible solution.");
        } else {
            result.status = "Undefined";
            std::cout << "Solver terminated with status: " << result.status << std::endl;
        }

        auto currentTime = std::chrono::high_resolution_clock::now();
        result.CPUtime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
        std::cout << "Timer (seconds): " << std::fixed << std::setprecision(4) << result.CPUtime << std::endl;

        RetrieveSolutions(cplex);
        DisplayVariables();

        env.end();
    } catch (const IloException& e) {
        std::cerr << "Error: " << e << std::endl;
        return result.success = false;
    } catch (const std::runtime_error& e) {
        std::cerr << "Runtime Error: " << e.what() << std::endl;
        return result.success = false;
    }
    return result.success = true;
}

void TSP::DefineVariables(IloEnv& env, IloModel& model)
{
    // Define Decision Variables
    VariableManager varManager(env);
    
    x = varManager.create1D(numEdges);
    for (int edgeIndex = 0; edgeIndex < numEdges; ++edgeIndex) {
        std::string varName = "x[" + std::to_string(edgeIndex) + "]";
        x[edgeIndex] = IloNumVar(env, 0.0, 1.0, IloNumVar::Bool, varName.c_str());
        model.add(x[edgeIndex]);
    }
}

void TSP::DefineObjectiveFunction(IloEnv& env, IloModel& model)
{
    // Define objective function
    IloExpr obj(env);
    for (int edgeIndex = 0; edgeIndex < numEdges; ++edgeIndex) {
        obj += transCostMat[index_i[edgeIndex]][index_j[edgeIndex]] * x[edgeIndex];
    }
    model.add(IloMinimize(env, obj));
    obj.end();
}

void TSP::DefineConstraints(IloEnv& env, IloModel& model)
{
    // Add Node Degree Constraints
    for (int nodeIndex = 0; nodeIndex < numNodes; ++nodeIndex) {
        std::string constraintName = "NodeDegree(" + std::to_string(nodeIndex) + ")";

        IloExpr expr(env);
        for (int edgeIndex = 0; edgeIndex < numEdges; ++edgeIndex) {
            if (index_i[edgeIndex] == nodeIndex || index_j[edgeIndex] == nodeIndex) {
                expr += x[edgeIndex];
            }
        }
        IloConstraint nodeDegreeConstraint(expr == 2);
        expr.end();

        model.add(nodeDegreeConstraint).setName(constraintName.c_str());
    }
}

void TSP::RetrieveSolutions(IloCplex& cplex)
{
    // Get solution values of decision variables
    for (int edgeIndex = 0; edgeIndex < numEdges; ++edgeIndex) {
        x_TSP[edgeIndex] = cplex.getIntValue(x[edgeIndex]);
    }

    std::vector<bool> visited(numNodes, false);
    int currentNode = 0;
    int index = 1;
    while (index < numNodes) {
        for (int i = 1; i < numNodes; ++i) {
            if (x_TSP[index_e[currentNode][i]] > THRESHOLD && !visited[i]) {
                currentNode = i;
                visited[i] = true;
                routeTSP.push_back(currentNode);
                routeTSP_Position[currentNode] = index;
                ++index;
                break;
            }
        }
    }
    currentNode = 0;
    visited[0] = true;
    routeTSP.push_back(currentNode);

    // Initialize visit matrix
    for (int i = 1; i < numNodes; ++i) {
        visitMat[i][i] = 0;
    }

    for (int i = 1; i < numNodes; ++i) {
        visitMat[0][i] = 1;
        visitMat[i][0] = 1;
    }
    for (int i = 1; i < numNodes; ++i) {
        for (int j = 1; j < numNodes; ++j) {
            if (routeTSP_Position[j] > routeTSP_Position[i]) {
                visitMat[i][j] = 1;
            }
        }
    }
}

void TSP::DisplayVariables()
{
    bool print_route = true;
    bool print_visitMat = false;

    if (print_route) {
        std::cout << "route_TSP: ";
        for (int node : routeTSP) {
            std::cout << node << " ";
        }
        std::cout << "\n" << std::endl;
    }

    if (print_visitMat) {
        std::cout << "nd: ";
        for (int i = 0; i < numNodes; ++i) {
            std::cout << (i < 10 ? std::to_string(i) + "  " : std::to_string(i) + " ");
        }
        std::cout << std::endl;

        for (int i = 0; i < numNodes; ++i) {
            std::cout << (i < 10 ? std::to_string(i) + ":  " : std::to_string(i) + ": ");
            for (int j = 0; j < numNodes; ++j) {
                std::cout << visitMat[i][j] << "  ";
            }
            std::cout << std::endl;
        }
    }
}
