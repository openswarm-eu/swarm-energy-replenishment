from os import listdir, environ
from os.path import isdir, join, dirname, normpath, basename

# Plotting
import numpy as np
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.patches import Rectangle
from matplotlib.font_manager import FontProperties
from matplotlib.collections import LineCollection
import seaborn as sns
import pandas as pd

# Utility
import sys
import math
import time
from collections import defaultdict
from itertools import combinations
import pprint
from copy import deepcopy

# Parse simulation log
sys.path.append(join(dirname(__file__), "..", "protos", "generated")) # Path to compiled proto files
import time_step_pb2
from load_data import SimData


# Path to simulation logs
# RESULTS_DIR = join(environ['HOME'], 'GIT/swarm-energy-replenishment_prep/results/work_and_charge_ants(main)')
# RESULTS_DIR = join(environ['HOME'], 'GIT/swarm-energy-replenishment_prep/results/work_and_charge_ants(work_rate)')
RESULTS_DIR = join(environ['HOME'], 'GIT/swarm-energy-replenishment_prep/results/work_and_charge_ants(recharge_rate)')
# RESULTS_DIR = join(environ['HOME'], 'GIT/swarm-energy-replenishment_prep/results/work_and_charge_ants(transfer_loss)')

BINARY_FILENAME = 'log_data.pb'
SUMMARY_FILENAME = 'summary.csv'


def load_log(path):

    # Load log and commands file
    log_file = join(path, BINARY_FILENAME) # path: RESULTS_DIR/scenario/BINARY_FILE
    # commands_file = join(RESULTS_DIR, scenario, COMMANDS_FILENAME) # path: RESULTS_DIR/scenario/COMMANDS_FILE
    summary_file = join(path, SUMMARY_FILENAME) # path: RESULTS_DIR/scenario/SUMMARY_FILE
    s = SimData(log_file, summary_file)
    # s = SimData(log_file) # Real robot experiments

    return s


def load_log_with_checks(path, print_result=False):
    s = load_log(path)

    # Store points scored
    trial_points = s.totalPoints

    if print_result:
        print(f'### Points scored ###')
        print(f'      scenario: {basename(normpath(path))}, seed: {s.seed}, points: {trial_points}')
        # print(f'### GLOBAL Connectivity ###')
        # print(f'      scenario: {basename(normpath(path))}, seed: {s.seed}, connectivity: {trial_connectivity}')
        # print(f'### TEAM Connectivity ###')
        # print(f'      scenario: {basename(normpath(path))}, seed: {s.seed}, connectivity: {trial_team_connectivity}')
    return s, trial_points


def plot_energy_consumption(plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Scenario':             pd.Series(dtype='str'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Connector Energy': pd.Series(dtype='float'),
        'Connector Energy / Total energy':  pd.Series(dtype='float'),
        'Energy Shared': pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Connectors Depleted': pd.Series(dtype='int'),
    })

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]:
        
        # Split variation by '_'
        var = variation.split('_')

            
        # if variation != '4T_charger(7R3CCC)':
        #     continue

        # scenario_type = var[1]
        scenario_type = variation
        scenario_labels.append(scenario_type)
        
        count = 0

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, variation)) if isdir(join(RESULTS_DIR, variation, f))]
        trial_dirs.sort()

        # loop all directories in variation
        for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

            count += 1
            # if count > 10:
            #     break
            # print(scenario)
            
            # if scenario[0] == 'X' :
            #     print('skipping', scenario)
            #     continue

            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, variation, scenario)
            print(path)

            s, _, _, _ = load_log_with_checks(path, print_result=plot)
            
            # # Counters
            # total_work = 0
            # total_energy_consumed = 0
            # total_energy_consumed_by_connectors = 0
            # total_energy_consumed_by_connectors_moving = 0

            # # Get the total amount of work performed
            # for task in s.data[s.totalTime]['log'].tasks:
            #     total_work += task.demand
            #     # print(task.name, task.demand)

            # # print('total work:', total_work)

            # # Dict to record every robot's energy consumption
            # initial_energy = 100
            # energy_idle_per_step = 0.005
            # energy_moving_per_step = 0.025
            # energy_working_per_step = 0.055
            # current_robot_energy = {}
            # energy_consumed_by_robot = {}
            # energy_consumed_by_connector = {}
            # energy_consumed_by_connector_moving = {}
            # energy_transfered = 0

            # # Init dict for every robot
            # for robot in s.data[1]['log'].robots:
            #     if robot.state is not time_step_pb2.Robot.State.LEADER:
            #         current_robot_energy[robot.name] = initial_energy
            #         energy_consumed_by_robot[robot.name] = 0
            #         energy_consumed_by_connector[robot.name] = 0
            #         energy_consumed_by_connector_moving[robot.name] = 0

            # # print(current_robot_energy)
            # # print(energy_consumed_by_robot)

            # # set of robots that ran out of energy
            # robots_out_of_energy = set()
            # connectors_out_of_energy = set()

            # # Loop every time step
            # for time in range(1, s.totalTime+1):
            # # for time in range(1, 100):

            #     # Energy shared between robots
            #     energy_transfered += s.data[time]['log'].energyShared
            #     # print('energy shared: ', s.data[time]['log'].energy_shared)

            #     # Loop robots
            #     for robot in s.data[time]['log'].robots:
            #         if robot.state is not time_step_pb2.Robot.State.LEADER:

            #             # Energy consumed
            #             if robot.energyLevel < current_robot_energy[robot.name]: 

            #                 # Robot was not charging
            #                 energy_consumed_by_robot[robot.name] += current_robot_energy[robot.name] - robot.energyLevel

            #                 if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #                     energy_consumed_by_connector[robot.name] += current_robot_energy[robot.name] - robot.energyLevel

            #                     if robot.isMoving:
            #                         energy_consumed_by_connector_moving[robot.name] += current_robot_energy[robot.name] - robot.energyLevel

            #             elif robot.isMoving: 
                            
            #                 # Robot was charing and moving
            #                 energy_consumed_by_robot[robot.name] += energy_moving_per_step

            #                 if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #                     energy_consumed_by_connector[robot.name] += energy_moving_per_step
            #                     energy_consumed_by_connector_moving[robot.name] += energy_moving_per_step
            #             else:
            #                 # Robot was charging and idle
            #                 energy_consumed_by_robot[robot.name] += energy_idle_per_step

            #                 if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #                     energy_consumed_by_connector[robot.name] += energy_idle_per_step

            #             # Update robot energy 
            #             current_robot_energy[robot.name] = robot.energyLevel

            #             if robot.energyLevel <= 0:
            #                 robots_out_of_energy.add(robot.name)

            #                 if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #                     connectors_out_of_energy.add(robot.name)

            # print(energy_consumed_by_robot)

            # Add all values in energy_consumed_by_robot
            # total_energy_consumed = sum(energy_consumed_by_robot.values())

            print('###### SCENARIO:', scenario, '######')

            # print(f'Total work performed:\t{total_work}')
            # print(f'Total energy consumed by all robots:\t{total_energy_consumed}')
            # print(f'Total energy consumed by connectors:\t{sum(energy_consumed_by_connector.values())}')
            # print(f'Total energy consumed by connectors moving:\t{sum(energy_consumed_by_connector_moving.values())}')
            # print(f'Total energy shared:\t{energy_transfered}')

            # print(f'work / total energy:', total_work / total_energy_consumed)
            # print(f'connector / total energy:', sum(energy_consumed_by_connector.values()) / total_energy_consumed)
            # print(f'connector moving / total energy:', sum(energy_consumed_by_connector_moving.values()) / total_energy_consumed)

            # print(f'Max energy consumed by a robot:\t{max(energy_consumed_by_robot.values())}')
            # print(f'Min energy consumed by a robot:\t{min(energy_consumed_by_robot.values())}')

            # df = pd.DataFrame({
            #     'Work performed':             pd.Series(dtype='int'), 
            #     'Total energy':       pd.Series(dtype='float'),
            #     'Work performed / Total energy':  pd.Series(dtype='float'),
            #     'Total energy (Connector)': pd.Series(dtype='float'),
            #     'Total energy (Connector) / Total energy':   pd.Series(dtype='float'),
            #     'Total energy (Connector Moving)': pd.Series(dtype='float'),
            # })


            # for robot, energy in current_robot_energy.items():
            #     if energy <= 0:
            #         robots_out_of_energy.add(robot)

            #         if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #             connectors_out_of_energy.add(robot)
            
            # print(f'Robots out of energy: {len(robots_out_of_energy)}')
            # print(f'Connectors out of energy: {len(connectors_out_of_energy)}')

            d = pd.DataFrame({
                'Scenario': [scenario_type],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [s.data[s.totalTime]['log'].totalEnergy],
                'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / s.data[s.totalTime]['log'].totalEnergy],
                'Connector Energy': [s.data[s.totalTime]['log'].connectorEnergy],
                'Connector Energy / Total energy':   [s.data[s.totalTime]['log'].connectorEnergy / s.data[s.totalTime]['log'].totalEnergy],
                'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Connectors Depleted': [s.data[s.totalTime]['log'].connectorsDepleted],
                # 'Total energy (Connector Moving)': [sum(energy_consumed_by_connector_moving.values())],
                # 'Total energy (Connector Moving) / Total energy':   [sum(energy_consumed_by_connector_moving.values()) / total_energy_consumed],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

            # print()

    print(df)

    ## loop scenario_labels
    for label in scenario_labels:

        print('##### RESULTS FOR SCENARIO:', label, '#####')

        # Print summary for 'travel'
        df_scenario = df[df['Scenario'] == label]

        avg = df_scenario['Work performed'].mean()
        print(f'Average Work performed: {avg}')
        avg = df_scenario['Total energy'].mean()
        print(f'Average Total energy: {avg}')
        avg = df_scenario['Work Energy'].mean()
        print(f'Average Work Energy: {avg}')
        avg = df_scenario['Work Energy / Total energy'].mean()
        print(f'Average Work Energy / Total energy: {avg}')
        avg = df_scenario['Connector Energy'].mean()
        print(f'Average Connector Energy: {avg}')
        avg = df_scenario['Connector Energy / Total energy'].mean()
        print(f'Average Connector Energy / Total energy: {avg}')
        avg = df_scenario['Energy Shared'].mean()
        print(f'Average Energy Shared: {avg}')
        avg = df_scenario['Workers depleted'].mean()
        print(f'Average Workers depleted: {avg}')
        avg = df_scenario['Connectors Depleted'].mean()
        print(f'Average Connectors Depleted: {avg}')

        # save df_scenario and the average of each column to a csv
        df_scenario.to_csv(join(RESULTS_DIR, f'energy_{label}.csv'), index=False)
        

        print()

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)



    exit(0)


    # Plot data
    fig1 = plt.figure(figsize=(5, 4))
    axes1 = fig1.gca()
    # fig2 = plt.figure(figsize=(9, 5))
    # axes2 = fig2.gca()
    # fig3 = plt.figure(figsize=(9, 5))
    # axes3 = fig3.gca()
    # fig4 = plt.figure(figsize=(9, 5))
    # axes4 = fig4.gca()
    # fig5 = plt.figure(figsize=(5, 4))
    # axes5 = fig5.gca()
    # axes = [axes1, axes5]

    print(df)

    ### LINE PLOT
    # hue_order = np.sort(df['Delay'].unique())
    # print(f'hue_order {hue_order}')
    # sns.lineplot(data=df, ax=axes1, x='Time', y='Travelers', hue='Delay', hue_order=hue_order, legend='full')
    # # sns.lineplot(data=df, ax=axes2, x='Time', y='Average Speed', hue='Delay', hue_order=hue_order)
    # # sns.lineplot(data=df, ax=axes3, x='Time', y='Minimum Speed', hue='Delay', hue_order=hue_order)
    # # sns.lineplot(data=df, ax=axes4, x='Time', y='Total Distance', hue='Delay', hue_order=hue_order)

    # sns.boxplot(data=df, ax=axes5, x='Delay', y='Robots Collided', order=hue_order, palette='Set2')
    # colors = ['.25', '.25']
    # sns.stripplot(data=df, ax=axes5, x='Delay', y='Robots Collided', order=hue_order, size=6, palette=colors, linewidth=0, dodge=True, jitter=False)

    # plot an area chart where the x axis is the time and the y axis is the number of 'Connectors', 'Workers working', 'Workers charging', 'Workers moving'
    sns.lineplot(data=df, ax=axes1, x='Time', y='Connectors', color='blue')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Workers working', color='green')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Workers charging', color='orange')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Workers moving', color='red')


    axes1.set_xlabel('Time (s)', fontproperties=font)
    axes1.set_ylabel('Number of robots', fontproperties=font)

    # legend

    # axes1.legend([str(int(label)) for label in hue_order], bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    handles, labels = axes1.get_legend_handles_labels()
    # axes1.legend(handles=handles[1:], labels=[str(int(label)) for label in hue_order], title='Delay (s)')
    # axes1.legend(handles=handles, labels=[str(int(float(label))) for label in labels], title='Delay (s)')
    # axes1.set_ylim([0,40+1])


    # max_val = df['Robots Collided'].max()
    # # axes5.set_yticks(np.arange(0, max_val+2, 2))
    # axes5.set_xticks(range(len(labels)), labels=[str(int(float(label))) for label in labels])
    # axes5.set_xlabel('Delay (s)', fontproperties=font)
    # axes5.set_ylabel('Robots Collided', fontproperties=font)
    # axes5.set_ylim([0,26+3])
    # print(labels)
    # axes5.set_xticks(range(len(labels)))

    # Define the text to be displayed above each box plot
    # text_values = []
    # for label in labels:
    #     delay_val = int(float(label)) * 10
    #     if delay_val in num_failed_runs:
    #         text_values.append(f'x{len(num_failed_runs[delay_val])}')
    #     else:
    #         text_values.append('x0')

    # print(text_values)

    # # Calculate the x-coordinate for each text based on the number of box plots (data points)
    # num_boxes = len(text_values)
    # text_positions = range(num_boxes)

    # # Add text above each box plot
    # for i, t in zip(text_positions, text_values):
    #     axes5.text(i, 26 + 2, t, ha='center', color='red', fontproperties=font2)

    # for ax in axes:
    #     for label in ax.get_xticklabels():
    #         label.set_fontproperties(font2)
    #     for label in ax.get_yticklabels():
    #         label.set_fontproperties(font2)

    #     ax.spines['top'].set_visible(False)
    #     ax.spines['right'].set_visible(False)
    #     ax.spines['bottom'].set_linewidth(1)
    #     ax.tick_params(width=1)

    fig1.tight_layout()
    # fig5.tight_layout()

    plt.show()
    # fig1.savefig(join(RESULTS_DIR, f'congestion_{var}_travellers.pdf'))
    # fig5.savefig(join(RESULTS_DIR, f'congestion_{var}_collisions.pdf'))  


def plot_energy_boxplot(chosen_num_chargers, chosen_capacity, plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Strategy':             pd.Series(dtype='str'),
        'Num Chargers':             pd.Series(dtype='int'),
        'Capacity':             pd.Series(dtype='str'),
        'Work Rate':            pd.Series(dtype='float'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Chargers depleted': pd.Series(dtype='int'),
    })

    count = 0

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(join(RESULTS_DIR, 'data')) if isdir(join(RESULTS_DIR, 'data', f))]:
        
        # Split variation by '_'
        var = variation.split('_')
        strategy = var[0]
        work_rate = var[1]
        num_chargers = var[2]
        capacity = var[3]
        recharge_rate = var[4]
        loss_rate = var[5]
        trial = var[6]

        # print(var)

        # convert work_rate to number and drop 'W'
        work_rate = float(work_rate[:-1])
        # print('work_rate', work_rate)

        # convert num_chargers to number and drop 'C'
        num_chargers = int(num_chargers[:-1])
        # print('num_chargers', num_chargers)

        # convert capacity to number and drop 'E'
        capacity = int(capacity[:-1])
        # print('capacity', capacity)

        # convert recharge_rate to number and drop 'R'
        recharge_rate = float(recharge_rate[:-1])

        # convert loss_rate to number and drop 'L'
        loss_rate = float(loss_rate[:-1])

        # convert trial to number
        trial = int(trial[1:])
        # print('trial', trial)

        # print('strategy', strategy, 'mobile', strategy == 'mobile')
        # print('num_chargers', num_chargers, chosen_num_chargers, num_chargers == chosen_num_chargers)
        # print('capacity', capacity, chosen_capacity, capacity == chosen_capacity)

        # print('\tsum', strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity)
        
        if (strategy == 'fixed') or (strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity):

            count += 1

            trial_dirs = [f for f in listdir(join(RESULTS_DIR, 'data', variation)) if isdir(join(RESULTS_DIR, 'data', variation, f))]
            trial_dirs.sort()

            # # loop all directories in variation
            # for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

                # count += 1
                # if count > 10:
                #     break
                # print(scenario)
                
            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, 'data', variation)
            print(count, path)

            s, _ = load_log_with_checks(path, print_result=plot)

            # print('###### SCENARIO:', variation, '######')

            d = pd.DataFrame({
                'Strategy': [strategy],
                'Num Chargers': [num_chargers],
                'Capacity': [f'{int(capacity/100)}x'],
                'Work Rate': [work_rate],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [s.data[s.totalTime]['log'].totalEnergy],
                'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / s.data[s.totalTime]['log'].totalEnergy],
                # 'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Chargers depleted': [s.data[s.totalTime]['log'].chargersDepleted],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

    print(df)

    fig1 = plt.figure(figsize=(6, 3.5))
    fig2 = plt.figure(figsize=(6, 3.5))
    fig3 = plt.figure(figsize=(6, 3.5))
    fig4 = plt.figure(figsize=(6, 3.5))
    fig5 = plt.figure(figsize=(6, 3.5))
    ax1 = fig1.gca()
    ax2 = fig2.gca()
    ax3 = fig3.gca()
    ax4 = fig4.gca()
    ax5 = fig5.gca()

    axes = [ax1, ax2, ax3, ax4, ax5]

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    plt.rcParams['text.usetex'] = True

    hue_order = ['fixed', 'mobile']
    legend_labels = ['Fixed', 'Mobile']

    # Sort and convert 'Work Rate' to string
    df = df.sort_values(by='Work Rate')
    df['Work Rate'] = df['Work Rate'].astype(str)

    # plot boxplot for 'Work performed'
    sns.boxplot(data=df, ax=ax1, x='Work Rate', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')
    # sns.lineplot(data=df, ax=ax1, x='Work Rate', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')

    # Calculate means by group
    means = df.groupby(['Work Rate', 'Strategy'])['Work performed'].mean().reset_index()
    # Overlay lines connecting means
    sns.lineplot(data=means, ax=ax1, x='Work Rate', y='Work performed', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)

    # Set axis labels
    ax1.set_xlabel('Energy cost of working vs moving')
    ax1.set_ylabel('Work performed')
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.0, 1, 0, 0]), ncol=1, frameon=True)    
    # ax1.set_title('Work performed')

    # lineplot

    # plot boxplot for 'Work Energy / Total energy'
    sns.boxplot(data=df, ax=ax2, x='Work Rate', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Work Rate', 'Strategy'])['Work Energy / Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax2, x='Work Rate', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Energy cost of working vs moving')
    ax2.set_ylabel('Energy efficiency')
    # ax2.legend(title='Strategy', bbox_to_anchor=([1.0, 1, 0, 0]), ncol=1, frameon=True)
    # add legend to the bottom right corner
    handles, labels = ax2.get_legend_handles_labels()
    ax2.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=(1, 0), loc='lower right', ncol=1, frameon=True)    
    # ax2.set_title('Energy efficiency')

    # plot boxplot for 'Total energy'
    sns.boxplot(data=df, ax=ax3, x='Work Rate', y='Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Work Rate', 'Strategy'])['Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax3, x='Work Rate', y='Total energy', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Energy cost of working vs moving')
    ax3.set_ylabel('Total energy')
    handles, labels = ax3.get_legend_handles_labels()
    ax3.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax3.set_title('Total energy')

    # plot boxplot for 'Workers depleted'
    sns.boxplot(data=df, ax=ax4, x='Work Rate', y='Workers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Work Rate', 'Strategy'])['Workers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax4, x='Work Rate', y='Workers depleted', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Energy cost of working vs moving')
    ax4.set_ylabel('Workers depleted')
    handles, labels = ax4.get_legend_handles_labels()
    ax4.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax4.set_title('Workers depleted')

    # plot boxplot for 'Chargers depleted'
    sns.boxplot(data=df, ax=ax5, x='Work Rate', y='Chargers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Work Rate', 'Strategy'])['Chargers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax5, x='Work Rate', y='Chargers depleted', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Energy cost of working vs moving')
    ax5.set_ylabel('Chargers depleted')
    # ax5.set_title('Chargers depleted')
    handles, labels = ax5.get_legend_handles_labels()
    ax5.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)


    for ax in axes:
        # set font size
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)
        
        # set font size of axis label
        ax.set_xlabel(ax.get_xlabel(), fontproperties=font)
        ax.set_ylabel(ax.get_ylabel(), fontproperties=font)

        # set font size of legend
        ax.get_legend().set_title(ax.get_legend().get_title().get_text(), prop=font2)
        for label in ax.get_legend().get_texts():
            label.set_fontproperties(font2)

        # set font size of title
        ax.set_title(ax.get_title(), fontproperties=font)

        # hide top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        # rename legend labels
                

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()

    fig1.savefig(join(RESULTS_DIR, f'work_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig2.savefig(join(RESULTS_DIR, f'efficiency_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig3.savefig(join(RESULTS_DIR, f'total_energy_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig4.savefig(join(RESULTS_DIR, f'depleted_workers_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'depleted_chargers_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))


def plot_recharge_rate(chosen_num_chargers, chosen_capacity, plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Strategy':             pd.Series(dtype='str'),
        'Num Chargers':             pd.Series(dtype='int'),
        'Capacity':             pd.Series(dtype='str'),
        'Work Rate':            pd.Series(dtype='float'),
        'Recharge rate':         pd.Series(dtype='float'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Chargers depleted': pd.Series(dtype='int'),
    })

    count = 0

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(join(RESULTS_DIR, 'data')) if isdir(join(RESULTS_DIR, 'data', f))]:
        
        # Split variation by '_'
        var = variation.split('_')
        strategy = var[0]
        work_rate = var[1]
        num_chargers = var[2]
        capacity = var[3]
        recharge_rate = var[4]
        loss_rate = var[5]
        trial = var[6]

        # print(var)

        # convert work_rate to number and drop 'W'
        work_rate = float(work_rate[:-1])
        # print('work_rate', work_rate)

        # convert num_chargers to number and drop 'C'
        num_chargers = int(num_chargers[:-1])
        # print('num_chargers', num_chargers)

        # convert capacity to number and drop 'E'
        capacity = int(capacity[:-1])
        # print('capacity', capacity)

        # convert recharge rate to number and drop 'R'
        recharge_rate = float(recharge_rate[:-1])

        # convert loss_rate to number and drop 'L'
        loss_rate = float(loss_rate[:-1])

        # convert trial to number
        trial = int(trial[1:])
        # print('trial', trial)

        # print('strategy', strategy, 'mobile', strategy == 'mobile')
        # print('num_chargers', num_chargers, chosen_num_chargers, num_chargers == chosen_num_chargers)
        # print('capacity', capacity, chosen_capacity, capacity == chosen_capacity)

        # print('\tsum', strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity)
        
        if (strategy == 'fixed') or (strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity):

            count += 1

            trial_dirs = [f for f in listdir(join(RESULTS_DIR, 'data', variation)) if isdir(join(RESULTS_DIR, 'data', variation, f))]
            trial_dirs.sort()

            # # loop all directories in variation
            # for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

                # count += 1
                # if count > 10:
                #     break
                # print(scenario)
                
            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, 'data', variation)
            print(count, path)

            s, _ = load_log_with_checks(path, print_result=plot)

            # print('###### SCENARIO:', variation, '######')

            d = pd.DataFrame({
                'Strategy': [strategy],
                # 'Num Chargers': [num_chargers],
                # 'Capacity': [f'{int(capacity/100)}x'],
                # 'Work Rate': [work_rate],
                'Recharge rate': [recharge_rate],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [s.data[s.totalTime]['log'].totalEnergy],
                # 'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / s.data[s.totalTime]['log'].totalEnergy],
                # 'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Chargers depleted': [s.data[s.totalTime]['log'].chargersDepleted],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

    print(df)

    fig1 = plt.figure(figsize=(6, 3.5))
    fig2 = plt.figure(figsize=(6, 3.5))
    fig3 = plt.figure(figsize=(6, 3.5))
    fig4 = plt.figure(figsize=(6, 3.5))
    fig5 = plt.figure(figsize=(6, 3.5))
    ax1 = fig1.gca()
    ax2 = fig2.gca()
    ax3 = fig3.gca()
    ax4 = fig4.gca()
    ax5 = fig5.gca()

    axes = [ax1, ax2, ax3, ax4, ax5]

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    plt.rcParams['text.usetex'] = True

    hue_order = ['fixed', 'mobile']
    legend_labels = ['Fixed', 'Mobile']

    # Sort and convert 'Recharge rate' to string
    df = df.sort_values(by='Recharge rate')
    df['Recharge rate'] = df['Recharge rate'].astype(str)

    # plot boxplot for 'Work performed'
    sns.boxplot(data=df, ax=ax1, x='Recharge rate', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')
    # sns.lineplot(data=df, ax=ax1, x='Recharge rate', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')

    # Calculate means by group
    means = df.groupby(['Recharge rate', 'Strategy'])['Work performed'].mean().reset_index()
    # Overlay lines connecting means
    sns.lineplot(data=means, ax=ax1, x='Recharge rate', y='Work performed', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)

    # Set axis labels
    ax1.set_xlabel('Charging and transfer rate')
    ax1.set_ylabel('Work performed')
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles=handles, labels=legend_labels, title='Strategy', bbox_to_anchor=(1, 0), loc='lower right', ncol=1, frameon=True)    
    # ax1.set_title('Work performed')

    # lineplot

    # plot boxplot for 'Work Energy / Total energy'
    sns.boxplot(data=df, ax=ax2, x='Recharge rate', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Recharge rate', 'Strategy'])['Work Energy / Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax2, x='Recharge rate', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Charging and transfer rate')
    ax2.set_ylabel('Energy efficiency')
    # ax2.legend(title='Strategy', bbox_to_anchor=([1.0, 1, 0, 0]), ncol=1, frameon=True)
    # add legend to the bottom right corner
    handles, labels = ax2.get_legend_handles_labels()
    ax2.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=(1, 0), loc='lower right', ncol=1, frameon=True)    
    # ax2.set_title('Energy efficiency')

    # plot boxplot for 'Total energy'
    sns.boxplot(data=df, ax=ax3, x='Recharge rate', y='Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Recharge rate', 'Strategy'])['Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax3, x='Recharge rate', y='Total energy', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Charging and transfer rate')
    ax3.set_ylabel('Total energy')
    handles, labels = ax3.get_legend_handles_labels()
    ax3.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax3.set_title('Total energy')

    # plot boxplot for 'Workers depleted'
    sns.boxplot(data=df, ax=ax4, x='Recharge rate', y='Workers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Recharge rate', 'Strategy'])['Workers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax4, x='Recharge rate', y='Workers depleted', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Charging and transfer rate')
    ax4.set_ylabel('Workers depleted')
    handles, labels = ax4.get_legend_handles_labels()
    ax4.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax4.set_title('Workers depleted')

    # plot boxplot for 'Chargers depleted'
    sns.boxplot(data=df, ax=ax5, x='Recharge rate', y='Chargers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Recharge rate', 'Strategy'])['Chargers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax5, x='Recharge rate', y='Chargers depleted', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Charging and transfer rate')
    ax5.set_ylabel('Chargers depleted')
    # ax5.set_title('Chargers depleted')
    handles, labels = ax5.get_legend_handles_labels()
    ax5.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)


    for ax in axes:
        # set font size
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)
        
        # set font size of axis label
        ax.set_xlabel(ax.get_xlabel(), fontproperties=font)
        ax.set_ylabel(ax.get_ylabel(), fontproperties=font)

        # set font size of legend
        ax.get_legend().set_title(ax.get_legend().get_title().get_text(), prop=font2)
        for label in ax.get_legend().get_texts():
            label.set_fontproperties(font2)

        # set font size of title
        ax.set_title(ax.get_title(), fontproperties=font)

        # hide top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        # rename legend labels
                

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()

    fig1.savefig(join(RESULTS_DIR, f'work_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig2.savefig(join(RESULTS_DIR, f'efficiency_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig3.savefig(join(RESULTS_DIR, f'total_energy_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig4.savefig(join(RESULTS_DIR, f'depleted_workers_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'depleted_chargers_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))


def plot_transfer_loss(chosen_num_chargers, chosen_capacity, plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Strategy':             pd.Series(dtype='str'),
        'Num Chargers':             pd.Series(dtype='int'),
        'Capacity':             pd.Series(dtype='str'),
        'Work Rate':            pd.Series(dtype='float'),
        'Recharge rate':         pd.Series(dtype='float'),
        'Transfer loss':         pd.Series(dtype='float'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Chargers depleted': pd.Series(dtype='int'),
    })

    count = 0

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(join(RESULTS_DIR, 'data')) if isdir(join(RESULTS_DIR, 'data', f))]:
        
        # Split variation by '_'
        var = variation.split('_')
        strategy = var[0]
        work_rate = var[1]
        num_chargers = var[2]
        capacity = var[3]
        recharge_rate = var[4]
        loss_rate = var[5]
        trial = var[6]

        # print(var)

        # convert work_rate to number and drop 'W'
        work_rate = float(work_rate[:-1])
        # print('work_rate', work_rate)

        # convert num_chargers to number and drop 'C'
        num_chargers = int(num_chargers[:-1])
        # print('num_chargers', num_chargers)

        # convert capacity to number and drop 'E'
        capacity = int(capacity[:-1])
        # print('capacity', capacity)

        # convert recharge rate to number and drop 'R'
        recharge_rate = float(recharge_rate[:-1])

        # convert loss_rate to number and drop 'L'
        loss_rate = float(loss_rate[:-1])

        # convert trial to number
        trial = int(trial[1:])
        # print('trial', trial)

        # print('strategy', strategy, 'mobile', strategy == 'mobile')
        # print('num_chargers', num_chargers, chosen_num_chargers, num_chargers == chosen_num_chargers)
        # print('capacity', capacity, chosen_capacity, capacity == chosen_capacity)

        # print('\tsum', strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity)
        
        if (strategy == 'fixed') or (strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity):

            count += 1

            trial_dirs = [f for f in listdir(join(RESULTS_DIR, 'data', variation)) if isdir(join(RESULTS_DIR, 'data', variation, f))]
            trial_dirs.sort()

            # # loop all directories in variation
            # for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

                # count += 1
                # if count > 10:
                #     break
                # print(scenario)
                
            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, 'data', variation)
            print(count, path)

            s, _ = load_log_with_checks(path, print_result=plot)

            total_energy = s.data[s.totalTime]['log'].totalEnergy + s.data[s.totalTime]['log'].energyLost

            # print('###### SCENARIO:', variation, '######')

            d = pd.DataFrame({
                'Strategy': [strategy],
                # 'Num Chargers': [num_chargers],
                # 'Capacity': [f'{int(capacity/100)}x'],
                # 'Work Rate': [work_rate],
                # 'Recharge rate': [recharge_rate],
                'Transfer loss': [loss_rate],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [total_energy],
                # 'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / total_energy],
                # 'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Chargers depleted': [s.data[s.totalTime]['log'].chargersDepleted],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

    print(df)

    fig1 = plt.figure(figsize=(6, 3.5))
    fig2 = plt.figure(figsize=(6, 3.5))
    fig3 = plt.figure(figsize=(6, 3.5))
    fig4 = plt.figure(figsize=(6, 3.5))
    fig5 = plt.figure(figsize=(6, 3.5))
    ax1 = fig1.gca()
    ax2 = fig2.gca()
    ax3 = fig3.gca()
    ax4 = fig4.gca()
    ax5 = fig5.gca()

    axes = [ax1, ax2, ax3, ax4, ax5]

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    plt.rcParams['text.usetex'] = True

    hue_order = ['fixed', 'mobile']
    legend_labels = ['Fixed', 'Mobile']

    # Sort and convert 'Transfer loss' to string
    df = df.sort_values(by='Transfer loss')
    df['Transfer loss'] = df['Transfer loss'].astype(str)

    # Splitting data into two parts based on 'Strategy' categories
    data_single_x = df[df['Strategy'] == 'fixed']
    data_multiple_x = df[df['Strategy'] == 'mobile']

    # Create boxplot for the category with multiple x values
    sns.boxplot(data=data_multiple_x, ax=ax1, x='Transfer loss', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')

    # Calculate and plot mean line for the category with single x value
    mean_value = data_single_x['Work performed'].mean()
    ax1.axhline(mean_value, color='#66c2a5', linestyle='-', label='fixed')

    # plot boxplot for 'Work performed'
    # sns.boxplot(data=df, ax=ax1, x='Transfer loss', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')

    # Calculate means by group
    filtered_df = df[df['Strategy'] == 'mobile']
    means = filtered_df.groupby(['Transfer loss', 'Strategy'])['Work performed'].mean().reset_index()
    # Overlay lines connecting means for data_multiple_x
    sns.lineplot(data=means, ax=ax1, x='Transfer loss', y='Work performed', markers=True, marker='D', dashes=False, color='#fc8d62', legend=False)

    # Set axis labels
    ax1.set_xlabel('Transfer loss')
    ax1.set_ylabel('Work performed')
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles=handles, labels=legend_labels, title='Strategy', bbox_to_anchor=(0, 0), loc='lower left', ncol=1, frameon=True)    
    # ax1.set_title('Work performed')

    # lineplot

    # plot boxplot for 'Work Energy / Total energy'
    sns.boxplot(data=data_multiple_x, ax=ax2, x='Transfer loss', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    mean_value = data_single_x['Work Energy / Total energy'].mean()
    ax2.axhline(mean_value, color='#66c2a5', linestyle='-', label='fixed')
    means = filtered_df.groupby(['Transfer loss', 'Strategy'])['Work Energy / Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax2, x='Transfer loss', y='Work Energy / Total energy', markers=True, marker='D', dashes=False, color='#fc8d62', legend=False)
    # sns.boxplot(data=df, ax=ax2, x='Transfer loss', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    # means = df.groupby(['Transfer loss', 'Strategy'])['Work Energy / Total energy'].mean().reset_index()
    # sns.lineplot(data=means, ax=ax2, x='Transfer loss', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, markers=True, dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Transfer loss')
    ax2.set_ylabel('Energy efficiency')
    # ax2.legend(title='Strategy', bbox_to_anchor=([1.0, 1, 0, 0]), ncol=1, frameon=True)
    # add legend to the bottom right corner
    handles, labels = ax2.get_legend_handles_labels()
    ax2.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=(0, 0), loc='lower left', ncol=1, frameon=True)    
    # ax2.set_title('Energy efficiency')

    # plot boxplot for 'Total energy'
    sns.boxplot(data=df, ax=ax3, x='Transfer loss', y='Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Transfer loss', 'Strategy'])['Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax3, x='Transfer loss', y='Total energy', hue='Strategy', hue_order=hue_order, markers=True, dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Transfer loss')
    ax3.set_ylabel('Total energy')
    handles, labels = ax3.get_legend_handles_labels()
    ax3.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax3.set_title('Total energy')

    # plot boxplot for 'Workers depleted'
    sns.boxplot(data=df, ax=ax4, x='Transfer loss', y='Workers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Transfer loss', 'Strategy'])['Workers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax4, x='Transfer loss', y='Workers depleted', hue='Strategy', hue_order=hue_order, markers=True, dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Transfer loss')
    ax4.set_ylabel('Workers depleted')
    handles, labels = ax4.get_legend_handles_labels()
    ax4.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax4.set_title('Workers depleted')

    # plot boxplot for 'Chargers depleted'
    sns.boxplot(data=df, ax=ax5, x='Transfer loss', y='Chargers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Transfer loss', 'Strategy'])['Chargers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax5, x='Transfer loss', y='Chargers depleted', hue='Strategy', hue_order=hue_order, markers=True, dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Transfer loss')
    ax5.set_ylabel('Chargers depleted')
    # ax5.set_title('Chargers depleted')
    handles, labels = ax5.get_legend_handles_labels()
    ax5.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)


    for ax in axes:
        # set font size
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)
        
        # set font size of axis label
        ax.set_xlabel(ax.get_xlabel(), fontproperties=font)
        ax.set_ylabel(ax.get_ylabel(), fontproperties=font)

        # set font size of legend
        ax.get_legend().set_title(ax.get_legend().get_title().get_text(), prop=font2)
        for label in ax.get_legend().get_texts():
            label.set_fontproperties(font2)

        # set font size of title
        ax.set_title(ax.get_title(), fontproperties=font)

        # hide top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        # rename legend labels
                

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()

    fig1.savefig(join(RESULTS_DIR, f'work_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig2.savefig(join(RESULTS_DIR, f'efficiency_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig3.savefig(join(RESULTS_DIR, f'total_energy_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig4.savefig(join(RESULTS_DIR, f'depleted_workers_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'depleted_chargers_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))


def plot_energy_heatmaps(chosen_work_rate, plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Strategy':             pd.Series(dtype='str'),
        'Num Chargers':             pd.Series(dtype='int'),
        'Capacity':             pd.Series(dtype='str'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Chargers depleted': pd.Series(dtype='int'),
    })

    count = 0

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(join(RESULTS_DIR, 'data')) if isdir(join(RESULTS_DIR, 'data', f))]:
        
        count += 1

        # Split variation by '_'
        var = variation.split('_')
        strategy = var[0]
        work_rate = var[1]
        num_chargers = var[2]
        capacity = var[3]
        # recharge_rate = var[4]
        # loss_rate = var[5]
        trial = var[4]

        # print(var)

        # if strategy == 'fixed':
        #     continue

        # convert work_rate to number and drop 'W'
        work_rate = float(work_rate[:-1])
        # print('work_rate', work_rate)

        # convert num_chargers to number and drop 'C'
        num_chargers = int(num_chargers[:-1])
        # print('num_chargers', num_chargers)

        # convert capacity to number and drop 'E'
        capacity = float(capacity[:-1])
        # print('capacity', capacity)

        # # convert recharge rate to number and drop 'R'
        # recharge_rate = float(recharge_rate[:-1])

        # # convert loss_rate to number and drop 'L'
        # loss_rate = float(loss_rate[:-1])

        # convert trial to number
        trial = int(trial[1:])
        # print('trial', trial)

        # scenario_type = var[1]
        # scenario_type = variation
        # scenario_labels.append(scenario_type)
        
        if work_rate == chosen_work_rate:

            trial_dirs = [f for f in listdir(join(RESULTS_DIR, 'data', variation)) if isdir(join(RESULTS_DIR, 'data', variation, f))]
            trial_dirs.sort()

            # # loop all directories in variation
            # for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

                # count += 1
                # if count > 10:
                #     break
                # print(scenario)
                
            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, 'data', variation)
            print(count, path)

            s, _ = load_log_with_checks(path, print_result=plot)

            # print('###### SCENARIO:', variation, '######')

            d = pd.DataFrame({
                'Strategy': [strategy],
                'Num Chargers': [num_chargers],
                'Capacity': [f'{float(capacity/100)}x'],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [s.data[s.totalTime]['log'].totalEnergy],
                'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / s.data[s.totalTime]['log'].totalEnergy],
                # 'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Chargers depleted': [s.data[s.totalTime]['log'].chargersDepleted],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

    print(df)

    fig1 = plt.figure(figsize=(6, 4))
    fig2 = plt.figure(figsize=(6, 4))
    fig3 = plt.figure(figsize=(6, 4))
    fig4 = plt.figure(figsize=(6, 4))
    fig5 = plt.figure(figsize=(6, 4))
    ax1 = fig1.gca()
    ax2 = fig2.gca()
    ax3 = fig3.gca()
    ax4 = fig4.gca()
    ax5 = fig5.gca()

    axes = [ax1, ax2, ax3, ax4, ax5]

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    # Select only the columns of interest for aggregation
    agg_work_df = df[['Num Chargers', 'Capacity', 'Work performed']]
    agg_efficiency_df = df[['Num Chargers', 'Capacity', 'Work Energy / Total energy']]
    agg_energy_df = df[['Num Chargers', 'Capacity', 'Total energy']]
    agg_depleted_workers_df = df[['Num Chargers', 'Capacity', 'Workers depleted']]
    agg_depleted_chargers_df = df[['Num Chargers', 'Capacity', 'Chargers depleted']]

    # Aggregate duplicate entries (e.g., using mean)
    grouped_work_df = agg_work_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()
    grouped_efficiency_df = agg_efficiency_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()
    grouped_energy_df = agg_energy_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()
    grouped_depleted_workers_df = agg_depleted_workers_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()
    grouped_depleted_chargers_df = agg_depleted_chargers_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()

    # plot a heatmap where the x axis is the number of chargers, the y axis is the capacity and the value is the work performed
    pivot_work_df = grouped_work_df.pivot(index="Capacity", columns="Num Chargers", values="Work performed")
    pivot_efficiency_df = grouped_efficiency_df.pivot(index="Capacity", columns="Num Chargers", values="Work Energy / Total energy")
    pivot_energy_df = grouped_energy_df.pivot(index="Capacity", columns="Num Chargers", values="Total energy")
    pivot_depleted_workers_df = grouped_depleted_workers_df.pivot(index="Capacity", columns="Num Chargers", values="Workers depleted")
    pivot_depleted_chargers_df = grouped_depleted_chargers_df.pivot(index="Capacity", columns="Num Chargers", values="Chargers depleted")

    # Reverse the order of the index
    pivot_work_df = pivot_work_df.iloc[::-1]
    pivot_efficiency_df = pivot_efficiency_df.iloc[::-1]
    pivot_energy_df = pivot_energy_df.iloc[::-1]
    pivot_depleted_workers_df = pivot_depleted_workers_df.iloc[::-1]
    pivot_depleted_chargers_df = pivot_depleted_chargers_df.iloc[::-1]

    sns.heatmap(data=pivot_work_df, ax=ax1, annot=True, cmap="YlGnBu", fmt='.0f')
    # ax1.set_title('Work performed')
    ax1.set_xlabel('Number of chargers')
    ax1.set_ylabel('Charger capacity')
    old_y_labels = [label.get_text() for label in ax1.get_yticklabels()]
    # replace 'x' with '\times'
    modified_y_labels = [string.replace('x', r'$\times$') for string in old_y_labels]
    # new_y_labels = [r'{label}' + "_new" for label in modified_y_labels]
    ax1.set_yticklabels(modified_y_labels)
    # output pivot_work_df to csv
    pivot_work_df.to_csv(join(RESULTS_DIR, f'work_heatmap.csv'))

    sns.heatmap(data=pivot_efficiency_df, ax=ax2, annot=True, cmap="YlGnBu", fmt='.3f')
    # ax2.set_title('Energy efficiency')
    ax2.set_xlabel('Number of chargers')
    ax2.set_ylabel('Charger capacity')
    ax2.set_yticklabels(modified_y_labels)
    pivot_efficiency_df.to_csv(join(RESULTS_DIR, f'efficiency_heatmap.csv'))

    sns.heatmap(data=pivot_energy_df, ax=ax3, annot=True, cmap="YlGnBu", fmt='.0f')
    # ax3.set_title('Total energy Consumed')
    ax3.set_xlabel('Number of chargers')
    ax3.set_ylabel('Charger capacity')
    ax3.set_yticklabels(modified_y_labels)
    pivot_energy_df.to_csv(join(RESULTS_DIR, f'total_energy_heatmap.csv'))

    sns.heatmap(data=pivot_depleted_workers_df, ax=ax4, annot=True, cmap="YlGnBu", fmt='.1f')
    # ax4.set_title('Number of Workers depleted')
    ax4.set_xlabel('Number of chargers')
    ax4.set_ylabel('Charger capacity')
    ax4.set_yticklabels(modified_y_labels)
    pivot_depleted_workers_df.to_csv(join(RESULTS_DIR, f'depleted_workers_heatmap.csv'))

    sns.heatmap(data=pivot_depleted_chargers_df, ax=ax5, annot=True, cmap="YlGnBu", fmt='.1f')
    # ax5.set_title('Number of chargers depleted')
    ax5.set_xlabel('Number of chargers')
    ax5.set_ylabel('Charger capacity')
    ax5.set_yticklabels(modified_y_labels)
    pivot_depleted_chargers_df.to_csv(join(RESULTS_DIR, f'depleted_chargers_heatmap.csv'))

    for ax in axes:
        # set font size
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)
        
        # set font size of axis label
        ax.set_xlabel(ax.get_xlabel(), fontproperties=font)
        ax.set_ylabel(ax.get_ylabel(), fontproperties=font)

        # set font size of title
        ax.set_title(ax.get_title(), fontproperties=font)

        # set font size of value in cell
        for text in ax.texts:
            text.set_fontsize(8)
        
        # hide top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)  
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_linewidth(1)
        ax.spines['left'].set_linewidth(1)      

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()

    fig1.savefig(join(RESULTS_DIR, f'work_heatmap.pdf'))
    fig2.savefig(join(RESULTS_DIR, f'efficiency_heatmap.pdf'))
    fig3.savefig(join(RESULTS_DIR, f'total_energy_heatmap.pdf'))
    fig4.savefig(join(RESULTS_DIR, f'depleted_workers_heatmap.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'depleted_chargers_heatmap.pdf'))


def main():

    # Load a single trial
    strategy = 'travel'
    num_run = '001'
    scenario = f'energy_sharing({strategy})_4T_10R_0C_{num_run}'
    path = join(environ['HOME'], f'GIT/swarm-energy-replenishment_prep/results/energy_sharing/', scenario)
    # print(scenario)


    # plot_energy_consumption(plot=False)
    # plot_connector_energy_over_time(plot=True)
    # path = '/home/genki/GIT/swarm-energy-replenishment_prep/results/iros3/4T_charger(5R5C)/energy_sharing(charger)_4T_5R_5C_003'
    # plot_connector_energy_over_time_single_trial(path, plot=True)

    # plot_energy_heatmaps(chosen_work_rate=1.0,
    #                      plot=False)
    # plot_energy_boxplot(chosen_num_chargers=6,
    #                     chosen_capacity=200,
    #                     plot=False)
    plot_recharge_rate(chosen_num_chargers=6,
                       chosen_capacity=200,
                       plot=False)
    # plot_transfer_loss(chosen_num_chargers=6,
    #                    chosen_capacity=200,
    #                    plot=False)

if __name__ == "__main__":
    main()
