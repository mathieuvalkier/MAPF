import numpy as np
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

import itertools as it
from bisect import bisect_left
from typing import List
import scipy.stats as ss
import researchpy as rp
from pylab import genfromtxt

from pandas import Categorical


def import_layout(nodes_file, edges_file):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates
    """
    gates_xy = []  # lst with (x,y) positions of gates
    rwy_dep_xy = []  # lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = []  # lst with (x,y) positions of exit points of arrival runways

    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)

    # Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"], row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties

        # Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"], row["y_pos"]))

    # Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy,
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}

    # Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"], row["to"])
        from_node = edge_id[0]
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

    # Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)

    return nodes_dict, edges_dict, start_and_goal_locations


def create_graph(nodes_dict, edges_dict, plot_graph=True):
    """
    Creates networkX graph based on nodes and edges and plots
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """

    graph = nx.DiGraph()  # create directed graph in NetworkX

    # Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node,
                       node_id=nodes_dict[node]["id"],
                       xy_pos=nodes_dict[node]["xy_pos"],
                       node_type=nodes_dict[node]["type"])

    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1],
                       edge_id=edge,
                       from_node=edges_dict[edge]["from"],
                       to_node=edges_dict[edge]["to"],
                       weight=edges_dict[edge]["length"])

    # Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)

    return graph

nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length
nodes_dict, edges_dict, _ = import_layout(nodes_file, edges_file)

ac = []
total = []

file = open('data.dat', 'r')
for line in file.readlines():
    fname = line.rstrip().split(',') #using rstrip to remove the \n
    total.append([float(i) for i in fname[1::]])

average_list = []


file = open('average_cbs.dat', 'r')
for line in file.readlines():
    fname = line.rstrip().split(',') #using rstrip to remove the \n
    average_list.append([float(i) for i in fname][0])

for i in range(0,len(total),2):
    ac.append([total[i],total[i+1]])
    
#Convert dat files to csv file
df1 = pd.read_csv('data_average.dat', sep= 's\s+', engine='python' )
df2 = pd.read_csv('average_cbs.dat', sep= 's\s+', engine='python' )
df3 = pd.read_csv('average_prioritized.dat', sep= 's\s+', engine='python' )

df_merged = pd.concat([df1,df2,df3], axis=1)
df_merged.to_csv('average.csv', header=['Independent', 'CBS', 'Prioritized'])


#Data is now available as: loc = ac[i][0], time = ac[i][1]

#Nu krijg je de xy locaties van aircraft 1
a = []
for loc in ac[0][0]:
    a.append( nodes_dict[loc]["xy_pos"] )


#graph = create_graph(nodes_dict, edges_dict)

for list_full in ac:

    list = list_full[0]

    for i in range(len(list)-1):

        if list[i]==list[i+1]:
            continue

        edges_dict[(list[i],list[i+1])]['count'] +=1

        # xcoords.append(nodes_dict[entry]['x_pos'])
        # ycoords.append(nodes_dict[entry]['y_pos'])

# for entry in edges_dict:
#     xy = edges_dict[entry]["start_end_pos"]
#
#     print(xy)
#     xc = [xy[0][0],xy[1][0]]
#     yc = [xy[0][1],xy[1][1]]
#     print(xc,yc)
#
#     plt.plot(xc, yc, 0.2, color='blue')
#
#
# for entry in edges_dict:
#     xy = edges_dict[entry]["start_end_pos"]
#
#     print(xy)
#     xc = [xy[0][0],xy[1][0]]
#     yc = [xy[0][1],xy[1][1]]
#     print(xc,yc)
#
#     plt.plot(xc, yc, linewidth=edges_dict[entry]['count']*3, color = 'red')
#
#
#
# plt.show()
 
######## -- Normal distribution Boxplot
plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True

independentdata = genfromtxt("data_average.dat")
cbsdata = genfromtxt("average_cbs.dat")
prioritizeddata = genfromtxt("average_prioritized.dat")
# plt.plot(independentdata[0:40], label="test.txt Data")
# plt.plot(cbsdata[0:40], label="test1.txt Data")

data_time = [independentdata[0:100], cbsdata[0:40], prioritizeddata[0:100]]
fig = plt.figure(figsize = (10,7))
ax = fig.add_subplot(111)
bp = ax.boxplot(data_time, showmeans=True)

ax.set_xticklabels(['Independent', 'Cbs', 'Prioritized'])
plt.title("Boxplot average taxi time")
plt.ylabel("Average taxi time [sec]")
plt.xlabel("Planning methods")
plt.legend()
plt.show()


######## -- Independent T-test: = 2 GROUPS
df_average = pd.read_csv('average.csv')
ttest = rp.ttest(group1= df_average['Independent'], group1_name= "Independent",
                 group2= df_average['Prioritized'], group2_name= "Prioritized")
                 #group3= df_average['Prioritized'], group3_name= "Prioritized")
summary, results = ttest
print(summary)
print(results)

fig2 = plt.figure(figsize= (15, 7))
ax = fig2.add_subplot(111)

normality_plot, ssi = ss.probplot(df_average['Independent'].values -\
                                 df_average['Prioritized'].values, plot= plt, rvalue=True)
ax.set_title("Probability average taxi time", fontsize= 20)
ax.set
plt.show()



######## -- A-test
def VD_A(treatment, control):
    """
    Computes Vargha and Delaney A index
    A. Vargha and H. D. Delaney.
    A critique and improvement of the CL common language
    effect size statistics of McGraw and Wong.
    Journal of Educational and Behavioral Statistics, 25(2):101-132, 2000
    The formula to compute A has been transformed to minimize accuracy errors
    See: http://mtorchiano.wordpress.com/2014/05/19/effect-size-of-r-precision/
    :param treatment: a numeric list
    :param control: another numeric list
    :returns the value estimate and the magnitude
    """
    m = len(treatment)
    n = len(control)

    print('len', m, n)

    if m != n:
        raise ValueError("Data d and f must have the same length")

    r = ss.rankdata(treatment + control)
    r1 = sum(r[0:m])

    # Compute the measure
    # A = (r1/m - (m+1)/2)/n # formula (14) in Vargha and Delaney, 2000
    A = (2 * r1 - m * (m + 1)) / (2 * n * m)  # equivalent formula to avoid accuracy errors

    levels = [0.147, 0.33, 0.474]  # effect sizes from Hess and Kromrey, 2004
    magnitude = ["negligible", "small", "medium", "large"]
    scaled_A = (A - 0.5) * 2

    magnitude = magnitude[bisect_left(levels, abs(scaled_A))]
    estimate = A

    return estimate, magnitude

print('list', average_list)

print(VD_A(average_list[0:50], average_list[50:100]))







