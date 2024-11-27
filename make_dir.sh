#!/bin/bash

# Define the base directories for output, error, and results
output_base="run/out"
results_base="Results/Solutions"
summary_base="Results/Summary"

# Create directories function
create_directoriesـstochastic() {
    problemType=$1
    solutionAlgorithm=$2
    penaltyCoef=10
    for ProbabilityFunction in Uniform Normal Gamma; do
        for numScenario in 20 50 100 200 500 1000; do
            if [ "$numScenario" == "100" ]; then
                for Unc_Range in 20% 40% 60% 80%; do
                    # create output for 2EPRP-AR
                    mkdir -p "$output_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $output_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                    mkdir -p "$output_base/$problemType/SolEvaluation/EV/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $output_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                    mkdir -p "$output_base/$problemType/SolEvaluation/EEV/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $output_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                    mkdir -p "$output_base/$problemType/SolEvaluation/WS/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $output_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }

                    mkdir -p "$results_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $results_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                    mkdir -p "$results_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $results_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                    mkdir -p "$results_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $results_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                    mkdir -p "$results_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $results_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }

                    mkdir -p "$summary_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $summary_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                    mkdir -p "$summary_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $summary_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                    mkdir -p "$summary_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $summary_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                    mkdir -p "$summary_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $summary_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                done
            else
                epsilon=20%
                # create output for 2EPRP-AR
                mkdir -p "$output_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $output_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                mkdir -p "$output_base/$problemType/SolEvaluation/EV/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $output_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                mkdir -p "$output_base/$problemType/SolEvaluation/EEV/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $output_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                mkdir -p "$output_base/$problemType/SolEvaluation/WS/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $output_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }

                mkdir -p "$results_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $results_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                mkdir -p "$results_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $results_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                mkdir -p "$results_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $results_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                mkdir -p "$results_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $results_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }

                mkdir -p "$summary_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $summary_base/$problemType/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                mkdir -p "$summary_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $summary_base/$problemType/SolEvaluation/EV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                mkdir -p "$summary_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $summary_base/$problemType/SolEvaluation/EEV/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
                mkdir -p "$summary_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef" || { echo "Error creating directory: $summary_base/$problemType/SolEvaluation/WS/$solutionAlgorithm/$ProbabilityFunction/S$numScenario/UR$Unc_Range/PC$penaltyCoef"; exit 1; }
            fi
        done
    done
}

create_directoriesـdeterministic() {
    problemType=$1
    solutionAlgorithm=$2

    # create output for 2EPRP-AR
    mkdir -p "$output_base/$problemType" || { echo "Error creating directory: $output_base/$problemType/$solutionAlgorithm"; exit 1; }
    
    mkdir -p "$results_base/$problemType/$solutionAlgorithm" || { echo "Error creating directory: $results_base/$problemType/$solutionAlgorithm"; exit 1; }

    mkdir -p "$summary_base/$problemType/$solutionAlgorithm" || { echo "Error creating directory: $summary_base/$problemType/$solutionAlgorithm"; exit 1; }
}

# Create directories for IMH, BC, and FR_BC methods
create_directoriesـstochastic "S2EPRP-AR" "Hybrid-ILS"
create_directoriesـstochastic "S2EPRP-AR" "BC"

create_directoriesـdeterministic "2EPRP" "Hybrid-ILS"
create_directoriesـdeterministic "2EPRP" "BC"

create_directoriesـdeterministic "2EPRPCS" "Hybrid-ILS"
create_directoriesـdeterministic "2EPRPCS" "BC"