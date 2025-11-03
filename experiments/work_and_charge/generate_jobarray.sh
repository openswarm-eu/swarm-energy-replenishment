#!/bin/bash

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Template file (relative to script directory)
TEMPLATE="$SCRIPT_DIR/run_jobarray_template.sh"

# Output directory (same as script directory)
OUTPUT_DIR="$SCRIPT_DIR"

# Parameters
HOURS=2
MINUTES=00
ARRAY_START=1
ARRAY_END=1

# List of configurations (each line = one config)
CONFIG_LIST=(
    "TAU_CHARGER_CAPACITY=5 ZETA_NUM_WORKERS=5 ETA_WORK_ENERGY_RATE=1 DELTA_COMMUTE=700"
    # add more configs here
)

# Loop over configurations
for CONFIG in "${CONFIG_LIST[@]}"; do
    # Convert string into variables
    eval "$CONFIG"

    # Output script name based on parameters
    OUTPUT="$OUTPUT_DIR/run_tau${TAU_CHARGER_CAPACITY}_w${ZETA_NUM_WORKERS}_eta${ETA_WORK_ENERGY_RATE}_delta${DELTA_COMMUTE}.sh"
    
    # Copy template
    if [ ! -f "$TEMPLATE" ]; then
        echo "Error: Template file not found at $TEMPLATE"
        exit 1
    fi
    cp "$TEMPLATE" "$OUTPUT"

    JOBNAME="tau${TAU_CHARGER_CAPACITY}-w${ZETA_NUM_WORKERS}-eta${ETA_WORK_ENERGY_RATE}-delta${DELTA_COMMUTE}"

    # Replace placeholders in template
    sed -i "s/JOBNAME/$JOBNAME/g" "$OUTPUT"
    sed -i "s/HOURS/$HOURS/g" "$OUTPUT"
    sed -i "s/MINUTES/$MINUTES/g" "$OUTPUT"
    sed -i "s/ARRAY_START/$ARRAY_START/g" "$OUTPUT"
    sed -i "s/ARRAY_END/$ARRAY_END/g" "$OUTPUT"
    sed -i "s/TAU_CHARGER_CAPACITY_PLACEHOLDER/$TAU_CHARGER_CAPACITY/g" "$OUTPUT"
    sed -i "s/ZETA_NUM_WORKERS_PLACEHOLDER/$ZETA_NUM_WORKERS/g" "$OUTPUT"
    sed -i "s/ETA_WORK_ENERGY_RATE_PLACEHOLDER/$ETA_WORK_ENERGY_RATE/g" "$OUTPUT"
    sed -i "s/DELTA_COMMUTE_PLACEHOLDER/$DELTA_COMMUTE/g" "$OUTPUT"

    echo "Submitting run with TAU=$TAU_CHARGER_CAPACITY, ZETA=$ZETA_NUM_WORKERS, ETA=$ETA_WORK_ENERGY_RATE, DELTA=$DELTA_COMMUTE"
    sbatch "$OUTPUT"
done
