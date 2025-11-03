#!/bin/bash

# Template script
TEMPLATE="job_template.sh"
OUTPUT_DIR="jobs"
mkdir -p "$OUTPUT_DIR"

# Optional time parameters
HOURS=2
MINUTES=00

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
    OUTPUT="$OUTPUT_DIR/run_tau${TAU_CHARGER_CAPACITY}_w${ZETA_NUM_WORKERS}.sh"
    
    # Copy template
    if [ ! -f "$TEMPLATE" ]; then
        echo "Error: Template file not found at $TEMPLATE"
        exit 1
    fi
    cp "$TEMPLATE" "$OUTPUT"

    JOBNAME="tau${TAU_CHARGER_CAPACITY}-w${ZETA_NUM_WORKERS}"

    # Replace placeholders in template
    sed -i "s/JOBNAME/$JOBNAME/g" "$OUTPUT"
    sed -i "s/HOURS/$HOURS/g" "$OUTPUT"
    sed -i "s/MINUTES/$MINUTES/g" "$OUTPUT"
    sed -i "s/TAU_CHARGER_CAPACITY_PLACEHOLDER/$TAU_CHARGER_CAPACITY/g" "$OUTPUT"
    sed -i "s/ZETA_NUM_WORKERS_PLACEHOLDER/$ZETA_NUM_WORKERS/g" "$OUTPUT"
    sed -i "s/ETA_WORK_ENERGY_RATE_PLACEHOLDER/$ETA_WORK_ENERGY_RATE/g" "$OUTPUT"
    sed -i "s/DELTA_COMMUTE_PLACEHOLDER/$DELTA_COMMUTE/g" "$OUTPUT"

    echo "Submitting run with TAU=$TAU_CHARGER_CAPACITY, ZETA=$ZETA_NUM_WORKERS, ETA=$ETA_WORK_ENERGY_RATE, DELTA=$DELTA_COMMUTE"
    sbatch "$OUTPUT"
done
