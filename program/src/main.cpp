#include "headers.h"
#include "ParameterSetting.h"
#include "algorithms.h"

void printUsage(const string &programName, const string &problemType) {
    if (problemType == "S2EPRP-AR") {
        cerr << "Usage: " << programName << " <problemType> <solutionAlgorithm> <inputFilename> "
             << "<Number of Warehouses> <Number of Customers> <Planning Horizon> <Number of Vehicles at the plant> "
             << "<Number of Vehicles Per each Warehouse> <Number of Scenarios> <Penalty Coefficient (For a Unit of Unmet demand)> "
             << "<Uncertainty Range> <ProbabilityFunction> <instanceName>" << endl;
    } else if (problemType == "EV") {
        cerr << "Usage: " << programName << " <problemType> <inputFilename> "
             << "<Number of Warehouses> <Number of Customers> <Planning Horizon> <Number of Vehicles at the plant> "
             << "<Number of Vehicles Per each Warehouse> <Number of Scenarios> <Penalty Coefficient (For a Unit of Unmet demand)> "
             << "<Uncertainty Range> <ProbabilityFunction> <instanceName>" << endl;
    } else if (problemType == "EEV" || problemType == "WS") {
        cerr << "Usage: " << programName << " <problemType> <inputFilename> "
             << "<Number of Warehouses> <Number of Customers> <Planning Horizon> <Number of Vehicles at the plant> "
             << "<Number of Vehicles Per each Warehouse> <Number of Scenarios> <Penalty Coefficient (For a Unit of Unmet demand)> "
             << "<Uncertainty Range> <ProbabilityFunction> <instanceName> <scenarioIndex>" << endl;
    } else if (problemType == "2EPRP" || problemType == "2EPRPCS") {
        cerr << "Usage: " << programName << " <problemType> <inputFilename> "
             << "<Number of Warehouses> <Number of Customers> <Planning Horizon> <Number of Vehicles at the plant> "
             << "<Number of Vehicles Per each Warehouse> <instanceName>" << endl;
    } else {
        cerr << "Invalid problem type provided." << endl;
    }
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        cerr << "Error: Problem type not specified." << endl;
        return EXIT_FAILURE;
    }

    string problemType = argv[1];
    int scenarioIndex;
    string solutionAlgorithm;

    if (problemType == "S2EPRP-AR") {
        if (argc != 14) {
            cerr << "Wrong number of arguments" << endl;
            printUsage(argv[0], problemType);
            return EXIT_FAILURE;
        }
        cout << "\nSolve The Stochastic Two-Echelon PRP with Adaptive Routing (S2EPRP-AR)." << endl;
        solutionAlgorithm = argv[2];
        cout << "Algorithm: " << solutionAlgorithm << endl;

    } else if (problemType == "EV") {
        if (argc != 13) {
            cerr << "Wrong number of arguments" << endl;
            printUsage(argv[0], problemType);
            return EXIT_FAILURE;
        }
        cout << "\nSolve The Expected Value (EV) Problem for the S2EPRP-AR." << endl;

    } else if (problemType == "EEV" || problemType == "WS") {
        if (argc != 14) {
            cerr << "Wrong number of arguments" << endl;
            printUsage(argv[0], problemType);
            return EXIT_FAILURE;
        }
        scenarioIndex = std::stoi(argv[13]); // Convert argument to integer

        if (problemType == "EEV")
            cout << "\nSolve The Expected of the Expected Value (EEV) Problem for the S2EPRP-AR For Scenario " << scenarioIndex + 1 << "." << endl;
        else
            cout << "\nSolve The Wait-and-See (WS) Problem for the S2EPRP-AR For Scenario " << scenarioIndex + 1 << "." << endl;

    } else if (problemType == "2EPRP" || problemType == "2EPRPCS") {
        if (argc != 9) {
            cerr << "Wrong number of arguments" << endl;
            printUsage(argv[0], problemType);
            return EXIT_FAILURE;
        }

        if (problemType == "2EPRP")
            cout << "\nSolve The Deterministic Two-Echelon PRP (2EPRP)." << endl;
        else
            cout << "\nSolve The Deterministic Two-Echelon PRP with Cross-Docking Satellites (2EPRPCS)." << endl;

    } else {
        cerr << "Wrong problem type" << endl;
        return EXIT_FAILURE;
    }

    ParameterSetting params(argc, argv);
    if (!params.setParameters())
    {
        cerr << "Unable to Set Parameters" << endl;
        return EXIT_FAILURE;
    }
    
    Algorithms alg(params);

	// Helper function to handle success/failure uniformly
    auto runProblem = [&](bool success) {
        if (!success) {
            std::cerr << problemType << " failed!" << std::endl;
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    };

    cout << "\n-------------------------------------------------------------------" << endl;
    
    if (problemType == "S2EPRP-AR" && solutionAlgorithm == "Hybrid-ILS") {
        return runProblem(alg.solve_S2EPRP_HILS());
    } 
    else if (problemType == "S2EPRP-AR" && solutionAlgorithm == "BC") {
        return runProblem(alg.solve_S2EPRP_BC());
    } 
    else if (problemType == "EV") {
        return runProblem(alg.solve_EV());
    } 
    else if (problemType == "EEV") {
        if (scenarioIndex < 0 || scenarioIndex >= params.numScenarios) {
            cerr << "Scenario index not valid for EEV." << endl;
            cerr << "Number of Scenarios: " << params.numScenarios << endl;
            cerr << "Scenario Index: " << scenarioIndex + 1 << endl;
            return EXIT_FAILURE;
        }
        return runProblem(alg.solve_EEV());
    } 
    else if (problemType == "WS") {
        if (scenarioIndex < 0 || scenarioIndex >= params.numScenarios) {
            cerr << "Scenario index not valid for WS." << endl;
            cerr << "Number of Scenarios: " << params.numScenarios << endl;
            cerr << "Scenario Index: " << scenarioIndex << endl;
            return EXIT_FAILURE;
        }
        return runProblem(alg.solve_WS());
    }
    else if (problemType == "2EPRP" || problemType == "2EPRPCS") {
        return runProblem(alg.solve_2EPRP());
    }
    else {
        cerr << "Invalid solution algorithm: " << solutionAlgorithm << endl;
        return EXIT_FAILURE;
    }

	return EXIT_SUCCESS;
}