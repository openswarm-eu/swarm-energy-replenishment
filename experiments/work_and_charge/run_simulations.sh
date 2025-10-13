#!/bin/bash

# Get first parameter

EXPERIMENT_NAME=work_and_charge
STRATEGY_TYPE="$1"

if [ "$STRATEGY_TYPE" = "fixed" ]; then
    STRATEGY_NAME=fixed_charger
    
    NUM_CHARGERS=(0)
    CHARGER_CAPACITY=(0)

    # work rate
    # DELTA_WORK=(0.1 0.25 0.5 1.0 1.5 2.0 3.0 4.0 10.0)
    DELTA_WORK=(1.0)

    # recharge rate
    # DELTA_RECHARGE=(0.1 0.25 0.5 0.75 1.0)
    DELTA_RECHARGE=(1.0)

    # transfer loss
    DELTA_TRANSFER_LOSS=(0.0)

elif [ "$STRATEGY_TYPE" = "mobile" ]; then
    STRATEGY_NAME=mobile_charger

    # number of chargers
    # NUM_CHARGERS=(1 2 4 6 8 10 12)
    NUM_CHARGERS=(1)

    # charger capacity
    # CHARGER_CAPACITY=(100 150 200 400 600)
    CHARGER_CAPACITY=(100)

    # work rate
    # DELTA_WORK=(0.1 0.25 0.5 1.0 1.5 2.0 3.0 4.0 10.0)
    DELTA_WORK=(1.0)

    # recharge rate
    # DELTA_RECHARGE=(0.1 0.25 0.5 0.75 1.0)
    DELTA_RECHARGE=(1.0)

    # transfer loss
    # DELTA_TRANSFER_LOSS=(0.0 0.1 0.2 0.3 0.4 0.5)
    DELTA_TRANSFER_LOSS=(0.0)
else
    echo "Invalid strategy type: '$1'"
    exit 1
fi

# Use current working directory as base (the directory from which the script is called)
BASE_DIR="$(pwd)"
EXPERIMENT_DIR="$BASE_DIR/experiments/$EXPERIMENT_NAME"
RESULT_DIR="$BASE_DIR/results/$EXPERIMENT_NAME"
TEMPLATE_EXPERIMENT_FILE=$EXPERIMENT_DIR/${STRATEGY_NAME}.argos

MAX_SPEED=12.0

# DEFAULT_DELTA_IDLE=0.0
DEFAULT_DELTA_IDLE=0.005
DEFAULT_DELTA_WORK=0.1
DEFAULT_DELTA_CHARGE=1.0
DEFAULT_DELTA_TRANSFER_LOSS=0.0

count=0
print_count=""

# Output color
ORANGE='\033[38;5;208m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# Start timing
start=$(date +%s.%N)
date +%r

### RUN multiple experiments

for o in ${DELTA_TRANSFER_LOSS[@]} # Transfer loss
do
    for m in ${DELTA_RECHARGE[@]} # Recharge per step
    do
        for j in ${DELTA_WORK[@]} # Worker energy
        do
            for i in ${CHARGER_CAPACITY[@]} # Charger capacity
            do
                for l in ${NUM_CHARGERS[@]} # Number of chargers per team
                do
                    # Start timing
                    start_config=$(date +%s.%N)

                    seed_count=0

                    for k in {1..50} # Number of runs
                    do
                        # Start timing
                        start_run=$(date +%s.%N)
                        
                        ((count++))

                        # Set log directory number
                        printf -v print_count "%03d" $k
                        # Make trial directory
                        TRIAL_DIR="${STRATEGY_TYPE}_${j}W_${l}C_${i}E_${m}R_${o}L_$print_count"
                        TRIAL_DIR_PATH="$RESULT_DIR/$TRIAL_DIR"
                        # echo $TRIAL_DIR_PATH
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
                        # sed -i "${LINE_RUN_NUMBER}s/.*/run_number='$k'/" $EXPERIMENT_FILE
                        echo $TRIAL_DIR
                        echo $TRIAL_DIR_PATH

                        # Set trial directory path
                        sed -i "s|out_path=.*|out_path='results/${EXPERIMENT_NAME}/${TRIAL_DIR}/'|" $EXPERIMENT_FILE

                        # Set idle energy
                        sed -i "s/delta_time=\"[0-9.]*\"/delta_time=\"$DEFAULT_DELTA_IDLE\"/" "$EXPERIMENT_FILE"

                        # Set work energy
                        rate=$(echo "$DEFAULT_DELTA_WORK*$j" | bc -l)
                        rate=$(printf "%.3f" $rate)
                        sed -i "s/<extra_battery_info delta_work=\"[0-9.]*\"/<extra_battery_info delta_work=\"$rate\"/" "$EXPERIMENT_FILE"

                        # Set recharge per step
                        rate=$(echo "$DEFAULT_DELTA_CHARGE*$m" | bc -l)
                        rate=$(printf "%.3f" $rate)
                        sed -i "s/delta_recharge=\"[0-9.]*\"/delta_recharge=\"$rate\"/" "$EXPERIMENT_FILE"

                        # Set transfer loss
                        rate=$(echo "$DEFAULT_DELTA_TRANSFER_LOSS*$o" | bc -l)
                        rate=$(printf "%.3f" $rate)
                        sed -i "s/delta_transfer_loss=\"[0-9.]*\"/delta_transfer_loss=\"$rate\"/" "$EXPERIMENT_FILE"

                        if [ "$STRATEGY_TYPE" = "mobile" ]; then
                            # # Set charger move energy
                            # sed -i "${LINE_CHARGER_MOVE_ENERGY}s|.*|delta_pos_charger='$rate'/>|" $EXPERIMENT_FILE

                            # Set charger full capacity
                            sed -i "s/full_charge_charger=\"[0-9]*\"/full_charge_charger=\"$i\"/" $EXPERIMENT_FILE

                            # Set charger start capacity
                            sed -i "s/start_charge_charger=\"[0-9]*,[0-9]*\"/start_charge_charger=\"$i,$i\"/" $EXPERIMENT_FILE

                            # Set number of chargers
                            sed -i "s|<e-puck_charger controller='charger' num_robots='[0-9]*'/>|<e-puck_charger controller='charger' num_robots='$l'/>|" "$EXPERIMENT_FILE"
                        fi

                        # Run experiment
                        echo "Running experiment $count, t_loss = $o, charge_rate = $m, work_rate = $j, num_chargers = $l, capacity = $i, run = $k, (s = $SEED) ..."
                        # LOG_FILE="$RESULT_DIR/log.txt"
                        # LOG_ERR_FILE="$RESULT_DIR/log_err.txt"

                        # Set log files to be discarded when running argos3
                        LOG_FILE="/dev/null"
                        LOG_ERR_FILE="/dev/null"
                        argos3 -c $EXPERIMENT_FILE -l $LOG_FILE -e $LOG_ERR_FILE -z
                        # argos3 -c $EXPERIMENT_FILE --no-visualization
                        # argos3 -c $EXPERIMENT_FILE

                        file="$TRIAL_DIR_PATH/summary.csv"
                        line1_num=8
                        line2_num=9
                        line3_num=10
                        line4_num=11
                        line1=$(sed "${line1_num}q;d" ${file})
                        line2=$(sed "${line2_num}q;d" ${file})
                        line3=$(sed "${line3_num}q;d" ${file})
                        line4=$(sed "${line4_num}q;d" ${file})
                        # Print the lines on the same line, with the first line in blue and the second line in green
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

                    # End timing
                    end=$(date +%s.%N)

                    # Calculate the elapsed time in seconds
                    elapsed_seconds_config=$(echo "$end - $start_config" | bc)
                    elapsed_seconds=$(echo "$end - $start" | bc)

                    # Convert the elapsed time to hours, minutes, and seconds
                    hours_config=$(printf "%02d" $(echo "$elapsed_seconds_config / 3600" | bc) 2>/dev/null)
                    minutes_config=$(printf "%02d" $(echo "($elapsed_seconds_config / 60) % 60" | bc) 2>/dev/null)
                    seconds_config=$(printf "%02d" $(echo "$elapsed_seconds_config % 60" | bc) 2>/dev/null)
                    hours=$(printf "%02d" $(echo "$elapsed_seconds / 3600" | bc) 2>/dev/null)
                    minutes=$(printf "%02d" $(echo "($elapsed_seconds / 60) % 60" | bc) 2>/dev/null)
                    seconds=$(printf "%02d" $(echo "$elapsed_seconds % 60" | bc) 2>/dev/null)

                    echo -e "${YELLOW}CONFIG took: ${hours_config}:${minutes_config}:${seconds_config}${NC}"
                    echo -e "${ORANGE}Elapsed time: ${hours}:${minutes}:${seconds}${NC}"
                    date +%r
                done
            done
        done
    done
done

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