# Exit on the first error.
set -e

set_directories () {
    # Set Directories
    insDir="../instancesOriginal"
    progDir="../program_test"

    if [ -d "$progDir" ]; then
        cd "$progDir" || exit 1
    else
        echo "Directory not found: $progDir"
        exit 1
    fi
}

set_ins_config() {
    # solAlg="Hybrid-ILS"
    solAlg="BC"
    echo "Solution Algorithm: ${solAlg}"
    numWs=2
    numRs=10
    numPers=3
    numVPs=2
    numVWs=3
    numSs=2
    penCoef=10
    uncLev=0.2
    probFunc="Uniform"
    echo "Probability Function: ${probFunc}"

    # Instance Info
    i=49
    j=1

    inputFilename="${insDir}/ABS${i}_50_${j}.dat.txt"
    instanceName="ABS${i}_50_${j}"
}

make_prog() {
    make -f Makefile
}

make_clean () {
    echo "Cleaning up..."
    make clean
}

run_prog () {
    echo bin/./main "${solAlg}" "${inputFilename}" "${numWs}" "${numRs}" "${numPers}" "${numVPs}" "${numVWs}" "${numSs}" "${penCoef}" "${uncLev}" "${probFunc}" "${instanceName}"
    bin/./main "${solAlg}" "${inputFilename}" "${numWs}" "${numRs}" "${numPers}" "${numVPs}" "${numVWs}" "${numSs}" "${penCoef}" "${uncLev}" "${probFunc}" "${instanceName}"

    if [ $? -ne 0 ]; then
        echo "Error running program"
        exit 1
    fi
}

# Run the Program
set_directories
set_ins_config
make_clean
make_prog
run_prog
