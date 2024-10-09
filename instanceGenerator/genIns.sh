#!/bin/bash

numPeriods=3
numVehicles_Plant=1
numVehicles_Warehouse=1

for insNum in 1
do
    for numWarehouses in 2 3 4 5
    do
        for numCustomers in 10 15 20 25 30
        do
            # Run the Python script with varying numCustomers
            python3 instanceGenerator.py --numWarehouses $numWarehouses --numCustomers $numCustomers --numPeriods $numPeriods --numVehicles_Plant $numVehicles_Plant --numVehicles_Warehouse $numVehicles_Warehouse --insNum $insNum
        done
    done
done