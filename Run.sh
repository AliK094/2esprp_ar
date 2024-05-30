echo "Running Program"

i=49
j=1
solAlg="ILS"
instance="ABS${i}_50_${j}"
input="2esprp_ar/instancesOriginal/${instance}.dat.txt"
nW=3
nR=10
nP=3
nVP=2
nVW=2
nS=10
DemPenal=10
UncRange=0.6
ProbFunc="Uniform"

progDir="2esprp_ar/program"

runProg() {
    echo "Running ${solAlg} ${input} ${nW} ${nR} ${nP} ${nVP} ${nVW} ${nS} ${DemPenal} ${UncRange} ${ProbFunc} ${instance}"
    echo "python ${progDir}/main.py ${solAlg} ${input} ${nW} ${nR} ${nP} ${nVP} ${nVW} ${nS} ${DemPenal} ${UncRange} ${ProbFunc} ${instance}"
    python ${progDir}/main.py ${solAlg} ${input} ${nW} ${nR} ${nP} ${nVP} ${nVW} ${nS} ${DemPenal} ${UncRange} ${ProbFunc} ${instance}
}

# Main script
runProg