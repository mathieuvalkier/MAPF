"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
import random
import string
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from cbs import run_CBS
from individual import run_individual
import numpy as np
import csv

#Parameters that can be changed:
simulation_time = 30
planner = 'Individual' # Choose which planner to use (currently only Independent is implemented)
high_demand  = True     # Demand situation, False => normal demand

#Visualization
plot_graph = False          #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.05   #set at 0.1 as default

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#%%Function definitions
def import_layout(nodes_file, edges_file):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges and edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates 
    """
    gates_xy = []   #lst with (x,y) positions of gates
    rwy_dep_xy = [] #lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = [] #lst with (x,y) positions of exit points of arrival runways
    
    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)
    
    #Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"],row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        #Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"],row["y_pos"]))

    #Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy, 
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}
    
    #Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"],row["to"])
        from_node =  edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos,
                           "count": 0
                           }
        edges_dict[edge_id] = edge_properties

    #Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)


    
    return nodes_dict, edges_dict, start_and_goal_locations

def create_graph(nodes_dict, edges_dict, plot_graph = True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """
    
    graph = nx.DiGraph() #create directed graph in NetworkX
    
    #Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node, 
                       node_id = nodes_dict[node]["id"],
                       xy_pos = nodes_dict[node]["xy_pos"],
                       node_type = nodes_dict[node]["type"])
        
    for edge in edges_dict.keys():

        graph.add_edge(edge[0], edge[1], 
                       edge_id = edge,
                       from_node =  edges_dict[edge]["from"],
                       to_node = edges_dict[edge]["to"],
                       weight = edges_dict[edge]["length"])
    
    #Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph

#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

# =============================================================================
# 1. While loop and visualization
# =============================================================================

def main_loop(random_id, print_path, start_time):

    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict)  # visualization properties

    #Start of while loop
    running = True
    escape_pressed = False
    block_runway = False
    time_end = simulation_time
    random.seed(random_id)
    dt = 0.1 #0.1 #should be factor of 0.5 (0.5/dt should be integer)
    t = 0
    next_ac_time = 0
    constraints = []
    node_lst = []  #list with tuples of goal and start nodes of each ac

    aircraft_lst = []   #List which can contain aircraft agents

    def ac_spawn(id,t):                                 # Make list of aircraft schedule

        # randomize aircraft inputs
        A_start_nodes = [37.0, 38.0]
        A_goal_nodes = [97.0, 34.0, 35.0, 36.0, 98.0]
        D_start_nodes = A_goal_nodes
        D_goal_nodes = [1.0, 2.0]
        a_d = random.choice(['A', 'D'])                 # Randomize arrival or departure
        if a_d == 'A':
            start_node = random.choice(A_start_nodes)   # Pick random choice between start and goal nodes
            goal_node = random.choice(A_goal_nodes)
        if a_d == 'D':
            start_node = random.choice(D_start_nodes)
            goal_node = random.choice(D_goal_nodes)

        size = random.choice([1,1,1,1,2,2,3])           # Pick random size, smaller sizes more common

        if size == 3 and a_d = 'D':                     # Large size departure need whole runway
            goal_node = 2.0

        ac = Aircraft(id, a_d, start_node, goal_node, t, nodes_dict, size)
        aircraft_lst.append(ac)


    time_difference = [0.5,1.0,1.5]         # Time separation can be either 0.5/1.0/1.5 seconds
    if high_demand:                         # In high demand shorter time separation
        time_difference = [0.5,0.5,1.0]


    for i in range(40):                 # Make schedule of 40 aircraft
        if len(aircraft_lst) == 0:
            new_id = 0                  # Count id from 0
            spawn_t = 0                 # Count spawn time from t = 0
        else:
            new_id = aircraft_lst[-1].id + 1
            spawn_t = aircraft_lst[-1].spawntime + random.choice(time_difference)

        ac_spawn(new_id,spawn_t)

    print("Simulation Started")
    while running:
        t= round(t,2)

        #Check conditions for termination
        if t >= time_end or escape_pressed:
            running = False
            pg.quit()
            print("Simulation Stopped")
            break

        #Visualization: Update map if visualization is true
        if visualization:
            current_states = {} #Collect current states of all aircraft
            for ac in aircraft_lst:
                if ac.status == "taxiing":
                    current_states[ac.id] = {"ac_id": ac.id,
                                             "xy_pos": ac.position,
                                             "heading": ac.heading,
                                             "size": ac.size}

            escape_pressed = map_running(map_properties, current_states, t)
            timer.sleep(visualization_speed)



        #Do planning
        if planner == "Independent":
            run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, print_path)
        elif planner == "Prioritized":
            run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, print_path)
        elif planner == "CBS":
            run_CBS(aircraft_lst, nodes_dict, edges_dict, heuristics, t, print_path)
        elif planner == "Individual":
            run_individual(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints, print_path, block_runway)
        #elif planner == -> you may introduce other planners here
        else:
            raise Exception("Planner:", planner, "is not defined.")

        block_runway = False

        #Move the aircraft that are taxiing
        for i, ac in enumerate(aircraft_lst):
            if ac.status == "taxiing":
                ac.move(dt, t)
            if ac.rwyblock[0] and ac.rwyblock[1]+0.5<t:
                block_runway = True

        t = t + dt

    # Save path data to data.dat file
    paths = []      # Gather path location and times of finished aircraft
    length = []     # Gather path lengths of finished aircraft

    for ac in aircraft_lst:

        path = ac.actualpath            # Only count paths of completed aircraft
        if ac.status == 'taxiing' or len(path)<1:
            continue

        loc = ['l']                     # Append data to location/time lists
        time = ['t']
        length.append(len(path))
        for entry in path:
            loc.append(entry[0])
            time.append(entry[1])

        paths.append(loc)
        paths.append(time)

    stdev = np.std(length)              # Find standard deviation of aircraft taxi lengths
    mean  = np.average(length)          # Find mean of aircraft taxi lengths
    comp_t = timer.time()-start_time    # Find computation time of simulation

    # Write data to corresponding data file

    with open('data.dat', 'a', newline='') as student_file:
        writer = csv.writer(student_file)
        for entry in paths:
            writer.writerow(entry)
    
    if planner == 'Independent':    
        with open('average_independent.dat', 'a', newline='') as student_file:
            writer = csv.writer(student_file)
            writer.writerow([mean,stdev,comp_t])
    elif planner == 'CBS':    
        with open('average_cbs.dat', 'a', newline='') as student_file:
            writer = csv.writer(student_file)
            writer.writerow([mean,stdev,comp_t])
    elif planner == 'Prioritized':
        with open('average_prioritized.dat', 'a', newline='') as student_file:
            writer = csv.writer(student_file)
            writer.writerow([mean,stdev,comp_t])
    elif planner == 'Individual':
        with open('average_individual.dat', 'a', newline='') as student_file:
            writer = csv.writer(student_file)
            writer.writerow([mean,stdev,comp_t])


i = 1
while i<100:            # Run simulation 100 times

    exclude = []        # If desired, add seeds to be excluded

    start_t = timer.time()

    if i in exclude:
        print('skipping: ', i)
    else:
        print(i)
        main_loop(random_id = i,print_path=False, start_time = start_t)

    i += 1

