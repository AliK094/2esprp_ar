#include "headers.h"
#include "ParameterSetting.h"
#include "algorithms.h"

int main(int argc, char *argv[])
{
	if (argc < 13 || argc > 14)  // 13 arguments (normal), 14 with optional scenario index
	{
		cerr << "Wrong number of arguments" << endl;
		cerr << "Usage: " << argv[0] << " <solutionAlgorithm> <inputFilename> "
			 << "<Number of Warehouses> <Number of Customers> <Planning Horizon> <Number of Vehicles at the plant> "
			 << "<Number of Vehicles Per each Warehouse> <Number of Scenarios> <Penalty Coefficient (For a Unit of Unmet demand)>"
			 << "<Uncertainty Range> <ProbabilityFunction> <instanceName> [scenarioIndex]" << endl;
		return EXIT_FAILURE;
	}
	// Check if scenario index is provided
    int scenarioIndex = (argc == 14) ? std::stoi(argv[13]) : -1;  // Default to -1 if not provided

	// Read the required parameters
    string solutionAlgorithm = argv[1];
	cout << "\n\nSolve The Stochastic Two-Echelon PRP with Adaptive Routing. " << endl;

    ParameterSetting params(argc, argv);
    if (!params.setParameters())
    {
        cerr << "Unable to Set Parameters" << endl;
        return EXIT_FAILURE;
    }

	Algorithms alg(solutionAlgorithm, params);

	// Helper function to handle success/failure uniformly
    auto runAlgorithm = [&](bool success) {
        if (!success) {
            std::cerr << solutionAlgorithm << " failed!" << std::endl;
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    };

	cout << "\n-------------------------------------------------------------------" << endl;
    
    if (solutionAlgorithm == "Hybrid-ILS") {
        return runAlgorithm(alg.solve_S2EPRP_HILS());
    } 
    else if (solutionAlgorithm == "BC") {
        return runAlgorithm(alg.solve_S2EPRP_BC());
    } 
    else if (solutionAlgorithm == "EV") {
        return runAlgorithm(alg.solve_EV());
    } 
    // else if (solutionAlgorithm == "EEV") {
    //     if (scenarioIndex < 0) {
    //         cerr << "Scenario index required for EEV." << endl;
    //         return EXIT_FAILURE;
    //     }
    //     return runAlgorithm(alg.solve_EEV(scenarioIndex));
    // } 
    // else if (solutionAlgorithm == "WS") {
    //     if (scenarioIndex < 0) {
    //         cerr << "Scenario index required for WS." << endl;
    //         return EXIT_FAILURE;
    //     }
    //     return runAlgorithm(alg.solve_WS(scenarioIndex));
    // } 
    else {
        cerr << "Invalid solution algorithm: " << solutionAlgorithm << endl;
        return EXIT_FAILURE;
    }

	return EXIT_SUCCESS;
}