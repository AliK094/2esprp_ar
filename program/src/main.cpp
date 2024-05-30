#include "headers.h"
#include "ParameterSetting.h"

int main(int argc, char *argv[])
{

	if (argc != 13)
	{
		cerr << "Wrong number of arguments" << endl;
		cerr << "Usage: " << argv[0] << "<solutionAlgorithm> <inputFilename> <NW> <NR> <T> <KP> <KW> <S> <beta> <ep> <ProbabilityFunction> <instanceName>" << endl;
		return 1;
	}

	cout << "Solve The Stochastic PRP with Service Level Constraints. " << endl;

	ParameterSetting params(argc, argv);
	if (!params.setParameters())
	{
		cerr << "Unable to Set Parameters" << endl;
		return 1;
	}

	return 0;
}
