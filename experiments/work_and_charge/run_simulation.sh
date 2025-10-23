#!/bin/bash

# Get first parameter

EXPERIMENT_NAME=work_and_charge

# Simulation configuration
STRATEGY_TYPE="mobile" # fixed or mobile
ENERGY_TYPE="supercap" #  supercap or battery
VISUALIZATION_ON="false" # true or false

if [ "$STRATEGY_TYPE" = "fixed" ]; then
    STRATEGY_NAME=fixed_charger
elif [ "$STRATEGY_TYPE" = "mobile" ]; then
    STRATEGY_NAME=mobile_charger
else
    echo "Invalid strategy type: '$1'"
    exit 1
fi

# Use current working directory as base (the directory from which the script is called)
BASE_DIR="$(pwd)"
EXPERIMENT_DIR="$BASE_DIR/experiments/$EXPERIMENT_NAME"
RESULT_DIR="$BASE_DIR/results/$EXPERIMENT_NAME"
TEMPLATE_EXPERIMENT_FILE=$EXPERIMENT_DIR/${STRATEGY_NAME}.argos

####################################################
### Parameters ###

if [ "$ENERGY_TYPE" = "supercap" ]; then
    # Supercapacitor
    CAPACITY_MAX=100
    XI_TRANSFER_LOSS=0.5 # 1 - 0.5
    NU_CHARGE=10
elif [ "$ENERGY_TYPE" = "battery" ]; then
    # Battery
    CAPACITY_MAX=2000
    XI_TRANSFER_LOSS=0.05 # 1 - 0.95
    NU_CHARGE=$(echo "scale=6; 5/9" | bc)
else
    echo "Invalid energy type: '$1'"
    exit 1
fi

# Common
NU_MIN=0.0005 # per timestep (assuming 1 timestep = 1 second)
WORK_PER_STEP=0.1

TAU_CHARGER_CAPACITY=5 # times larger than worker capacity
ZETA_NUM_WORKERS=5 # number of workers per mobile charger
ETA_WORK_ENERGY_RATE=1 # If greater than 1, it means more energy is needed to work than to move
DELTA_COMMUTE=40 # duration of one way commute in seconds

CAPACITY_CHARGER_MAX=$((TAU_CHARGER_CAPACITY * CAPACITY_MAX))

####################################################

MAX_SPEED=12.0 # cm/s
# calculate commute distance in meters
COMMUTE_DISTANCE=$(echo "scale=3; ($DELTA_COMMUTE * $MAX_SPEED) / 100" | bc -l)
# format with leading zero
COMMUTE_DISTANCE=$(printf "%.3f" "$COMMUTE_DISTANCE")
echo "Commute distance: $(printf "%.3f" $COMMUTE_DISTANCE) m"

# Arena size variables
ARENA_X=2.5
ARENA_Y=$(echo "$COMMUTE_DISTANCE + 1.6" | bc -l)
WALL_THICK=0.05
WALL_HEIGHT=0.2
REGION_X=0.3

count=0

# Output color
ORANGE='\033[38;5;208m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# Start timing
start=$(date +%s.%N)
date +%r

### RUN experiment

# Start timing
start_config=$(date +%s.%N)

seed_count=0

for k in {1..1} # Number of runs
do
    # Start timing
    start_run=$(date +%s.%N)
    
    ((count++))

    # Make trial directory
    TRIAL_DIR="mobile_tau${TAU_CHARGER_CAPACITY}_zeta${ZETA_NUM_WORKERS}_eta${ETA_WORK_ENERGY_RATE}_dcomm${DELTA_COMMUTE}_$(printf '%03d' $k)"
    TRIAL_DIR_PATH="$RESULT_DIR/$TRIAL_DIR"
    mkdir -p $TRIAL_DIR_PATH

    # copy template experiment file into TRIAL_DIR
    EXPERIMENT_FILE="$TRIAL_DIR_PATH/${STRATEGY_NAME}.argos"
    cp $TEMPLATE_EXPERIMENT_FILE $EXPERIMENT_FILE

    # Set seed
    let SEED=876*k
    sed -i "s/random_seed=['\"][0-9]*['\"]/random_seed='$SEED'/" "$EXPERIMENT_FILE"

    # Update max_speed
    sed -i "s/max_speed=\"[0-9.]*\"/max_speed=\"$MAX_SPEED\"/" "$EXPERIMENT_FILE"

    # # Set run number
    echo $TRIAL_DIR
    echo $TRIAL_DIR_PATH

    # Set trial directory path
    sed -i "s|out_path=.*|out_path='results/${EXPERIMENT_NAME}/${TRIAL_DIR}/'|" $EXPERIMENT_FILE

    # Set worker capacity
    sed -i "s/full_charge_worker=\"[0-9]*\"/full_charge_worker=\"$CAPACITY_MAX\"/" $EXPERIMENT_FILE
    sed -i "s/start_charge_worker=\"[0-9]*,[0-9]*\"/start_charge_worker=\"$CAPACITY_MAX,$CAPACITY_MAX\"/" $EXPERIMENT_FILE

    # Set idle energy
    sed -i "s/delta_time=\"[0-9.]*\"/delta_time=\"$NU_MIN\"/" "$EXPERIMENT_FILE"

    # Set work energy
    rate=$(echo "$WORK_PER_STEP*$ETA_WORK_ENERGY_RATE" | bc -l)
    rate=$(printf "%.3f" $rate)
    sed -i "s/<extra_battery_info delta_work=\"[0-9.]*\"/<extra_battery_info delta_work=\"$rate\"/" "$EXPERIMENT_FILE"

    # Set recharge per step
    rate=$(printf "%.3f" $NU_CHARGE)
    sed -i "s/delta_recharge=\"[0-9.]*\"/delta_recharge=\"$rate\"/" "$EXPERIMENT_FILE"

    # Set work performed per time step
    sed -i "s/work_per_step=\"[0-9.]*\"/work_per_step=\"$WORK_PER_STEP\"/" "$EXPERIMENT_FILE"

    if [ "$STRATEGY_TYPE" = "mobile" ]; then

        # Set mobile charger capacity
        sed -i "s/full_charge_charger=\"[0-9]*\"/full_charge_charger=\"$CAPACITY_CHARGER_MAX\"/" $EXPERIMENT_FILE
        sed -i "s/start_charge_charger=\"[0-9]*,[0-9]*\"/start_charge_charger=\"$CAPACITY_CHARGER_MAX,$CAPACITY_CHARGER_MAX\"/" $EXPERIMENT_FILE

        # Set transfer loss
        rate=$(echo "$XI_TRANSFER_LOSS" | bc -l)
        rate=$(printf "%.3f" $rate)
        sed -i "s/delta_transfer_loss=\"[0-9.]*\"/delta_transfer_loss=\"$rate\"/" "$EXPERIMENT_FILE"

        # Set charger full capacity
        sed -i "s/full_charge_charger=\"[0-9]*\"/full_charge_charger=\"$CAPACITY_CHARGER_MAX\"/" $EXPERIMENT_FILE

        # Set charger start capacity
        sed -i "s/start_charge_charger=\"[0-9]*,[0-9]*\"/start_charge_charger=\"$CAPACITY_CHARGER_MAX,$CAPACITY_CHARGER_MAX\"/" $EXPERIMENT_FILE

        # Set number of workers
        sed -i "s|<e-puck controller=\"worker_mc\" num_robots=\"[0-9]*\"/>|<e-puck controller=\"worker_mc\" num_robots=\"$ZETA_NUM_WORKERS\"/>|" "$EXPERIMENT_FILE"
    fi

    # Set commute duration />
    sed -i "s|<commute_region travel_duration=\"[0-9.]*\"|<commute_region travel_duration=\"$DELTA_COMMUTE\"|" "$EXPERIMENT_FILE"

    # Set arena size based on commute distance + 1m
    NEW_SIZE_Y=$(echo "$COMMUTE_DISTANCE + 1 + ($REGION_X * 2)" | bc -l)
    echo New arena size Y: $(printf "%.3f" $NEW_SIZE_Y) m
    sed -i "s|<arena size=\"[0-9.]\+,[0-9.]\+,[0-9.]\+\"|<arena size=\"$NEW_SIZE_Y,2,1\"|" "$EXPERIMENT_FILE"

    # wall length y = region * 2 + 1
    WALL_LENGTH_X=$(echo "($REGION_X * 2) + $COMMUTE_DISTANCE" | bc -l)

    # ---- NORTH WALL ----
    sed -i "/<box id=\"wall_north\"/s|size=\"[^\"]*\"|size=\"$WALL_LENGTH_X,$WALL_THICK,$WALL_HEIGHT\"|" "$EXPERIMENT_FILE"

    # ---- SOUTH WALL ----
    sed -i "/<box id=\"wall_south\"/s|size=\"[^\"]*\"|size=\"$WALL_LENGTH_X,$WALL_THICK,$WALL_HEIGHT\"|" "$EXPERIMENT_FILE"

    # Compute wall position
    WALL_LENGTH_Y=1.1
    WALL_WEST_X=$(echo "-$NEW_SIZE_Y/2 + 0.5" | bc -l)
    WALL_EAST_X=$(echo "$NEW_SIZE_Y/2 - 0.5" | bc -l)
    echo Wall west X: $(printf "%.3f" $WALL_WEST_X) m
    echo Wall east X: $(printf "%.3f" $WALL_EAST_X) m

    # ---- WEST WALL ----
    sed -i "/<box id=\"wall_west\"/s|size=\"[^\"]*\"|size=\"$WALL_THICK,$WALL_LENGTH_Y,$WALL_HEIGHT\"|" "$EXPERIMENT_FILE"
    sed -i "/<box id=\"wall_west\"/,/<\/box>/s|position=\"[^\"]*\"|position=\"$WALL_WEST_X,0,0\"|" "$EXPERIMENT_FILE"

    # ---- EAST WALL ----
    sed -i "/<box id=\"wall_east\"/s|size=\"[^\"]*\"|size=\"$WALL_THICK,$WALL_LENGTH_Y,$WALL_HEIGHT\"|" "$EXPERIMENT_FILE"
    sed -i "/<box id=\"wall_east\"/,/<\/box>/s|position=\"[^\"]*\"|position=\"$WALL_EAST_X,0,0\"|" "$EXPERIMENT_FILE"

    # Run experiment
    echo "Running experiment $count, t_loss = $XI_TRANSFER_LOSS, charge_rate = $NU_CHARGE, work_rate = $ETA_WORK_ENERGY_RATE, num_workers = $ZETA_NUM_WORKERS, charger_capacity = $CAPACITY_CHARGER_MAX, run = $k, (s = $SEED) ..."
    # LOG_FILE="$RESULT_DIR/log.txt"
    # LOG_ERR_FILE="$RESULT_DIR/log_err.txt"

    # Set log files to be discarded when running argos3
    LOG_FILE="/dev/null"
    LOG_ERR_FILE="/dev/null"

    # Run simulation
    if [ "$VISUALIZATION_ON" = "true" ]; then
        argos3 -c $EXPERIMENT_FILE -l $LOG_FILE -e $LOG_ERR_FILE
    else
        argos3 -c $EXPERIMENT_FILE -l $LOG_FILE -e $LOG_ERR_FILE -z
    fi

    file="$TRIAL_DIR_PATH/summary.csv"
    line1_num=10
    line2_num=11
    line3_num=12
    line4_num=13
    line1=$(sed "${line1_num}q;d" ${file})
    line2=$(sed "${line2_num}q;d" ${file})
    line3=$(sed "${line3_num}q;d" ${file})
    line4=$(sed "${line4_num}q;d" ${file})
    # Print the lines on the same line in green color
    echo -e "\033[32m${line1}, ${line2}, ${line3}, ${line4}\033[0m"

    # End timing
    end=$(date +%s.%N)

    # Calculate the elapsed time in seconds
    elapsed_seconds_run=$(echo "$end - $start_run" | bc)

    # Convert the elapsed time to hours, minutes, and seconds
    hours_run=$(printf "%02d" $(echo "$elapsed_seconds_run / 3600" | bc) 2>/dev/null)
    minutes_run=$(printf "%02d" $(echo "($elapsed_seconds_run / 60) % 60" | bc) 2>/dev/null)
    seconds_run=$(printf "%02d" $(echo "$elapsed_seconds_run % 60" | bc) 2>/dev/null)

    # Print the elapsed time in hours, minutes, and seconds in orange color
    echo -e "${YELLOW}RUN took: ${hours_run}:${minutes_run}:${seconds_run}${NC}"

done

# # End timing
# end=$(date +%s.%N)

# # Calculate the elapsed time in seconds
# elapsed_seconds_config=$(echo "$end - $start_config" | bc)
# elapsed_seconds=$(echo "$end - $start" | bc)

# # Convert the elapsed time to hours, minutes, and seconds
# hours_config=$(printf "%02d" $(echo "$elapsed_seconds_config / 3600" | bc) 2>/dev/null)
# minutes_config=$(printf "%02d" $(echo "($elapsed_seconds_config / 60) % 60" | bc) 2>/dev/null)
# seconds_config=$(printf "%02d" $(echo "$elapsed_seconds_config % 60" | bc) 2>/dev/null)
# hours=$(printf "%02d" $(echo "$elapsed_seconds / 3600" | bc) 2>/dev/null)
# minutes=$(printf "%02d" $(echo "($elapsed_seconds / 60) % 60" | bc) 2>/dev/null)
# seconds=$(printf "%02d" $(echo "$elapsed_seconds % 60" | bc) 2>/dev/null)

# echo -e "${YELLOW}CONFIG took: ${hours_config}:${minutes_config}:${seconds_config}${NC}"
# echo -e "${ORANGE}Elapsed time: ${hours}:${minutes}:${seconds}${NC}"
# date +%r


# End timing
end=$(date +%s.%N)

# Calculate the elapsed time in seconds
elapsed_seconds=$(echo "$end - $start" | bc)

# Convert the elapsed time to hours, minutes, and seconds
hours=$(printf "%02d" $(echo "$elapsed_seconds / 3600" | bc) 2>/dev/null)
minutes=$(printf "%02d" $(echo "($elapsed_seconds / 60) % 60" | bc) 2>/dev/null)
seconds=$(printf "%02d" $(echo "$elapsed_seconds % 60" | bc) 2>/dev/null)

# Print the elapsed time in hours, minutes, and seconds in orange color
ORANGE='\033[38;5;208m'
NC='\033[0m' # No Color
echo -e "${ORANGE}TOTAL elapsed time: ${hours}:${minutes}:${seconds}${NC}"

date +%r