"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx

def calc_heuristics(graph, nodes_dict):
    """
    Calculates the exact heuristic dict (shortest distance between two nodes) to be used in A* search.
    INPUT:
        - graph = networkX graph object
        - nodes_dict = dictionary with nodes and node properties
    RETURNS:
        - heuristics = dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
    """
    
    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path == False:
                pass
            else:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics

def heuristicFinder(graph, start_node, goal_node):
    """
    Finds exact distance between start_node and goal_node using the NetworkX graph.
    INPUT:
        - graph = networkX graph object
        - start_node, goal_node [int] = node_ids of start and goal node
    RETURNS:
        - path = list with node_ids that are part of the shortest path
        - path_length = length of the shortest path
    """
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        path = False
        path_length = False
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length

def build_constraint_table(constraints, agent):

    constraint_table = []
    neg_constraints = []
    maxtimestep = 0

    for constraint in constraints:
        if agent == constraint['ac']:
            neg_constraints.append(constraint)

            #print('const timestep', constraint['timestep'])

            maxtimestep = max(constraint['timestep'],maxtimestep)

    for i in range(int(maxtimestep*2) + 1):
        #print()
        constraint_table.append([])

    for constraint in neg_constraints:
        constraint_table[int(constraint['timestep']*2)].append({'loc': constraint['loc']})


    return constraint_table, maxtimestep

def is_constrained(curr_loc, next_loc, next_time, constraint_table,agent):

    #print('agent',constraint_table)

    if len(constraint_table)<=(next_time*2+1):
        return False

    else:
        if len(constraint_table[int(next_time*2)]) == 0:
            return False
        else:
            for entries in constraint_table[int(next_time*2)]:
                coords = entries['loc']
                if len(coords) == 1:
                    if coords[0] == next_loc:
                        return True
                if len(coords) == 2:
                    if coords[0] == curr_loc and coords[1] == next_loc:
                        return True
            return False


def simple_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start, agent=0, constraints=0):
    # def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):

    print('agent', agent)
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time. 
        - Hint: do you need more inputs?
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
    """

    constraint_table, maxtimestep = build_constraint_table(constraints, agent)  # list of constraints

    # def directions(parent):
    #     for neighbor in nodes_dict[curr['loc']]["neighbors"]:
    #         if
    #         directions.append(neighbor)

    
    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start

    previous_node = 0
    
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start, 'previous': None}
    push_node(open_list, root,)
    closed_list[(root['loc'], root['timestep'])] = root

    nummer = 1

    while len(open_list) > 0:
        nummer = nummer + 1
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr)

        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                    'g_val': curr['g_val'] + 0.5,
                    'h_val': heuristics[neighbor][goal_node_id],
                    'parent': curr,
                    'timestep': curr['timestep'] + 0.5,
                    'previous': curr['loc']}

            if child['loc'] == curr['previous']:
                continue

            if is_constrained(curr['loc'], child['loc'], child['timestep']  ,
                              constraint_table, agent) == True:
                continue

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child,nummer)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child,nummer)

        #Stay at location if other options are not possible
        child = {'loc': curr['loc'],
                     'g_val': curr['g_val'] + 0.5,
                     'h_val': heuristics[curr['loc']][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + 0.5,
                     'previous': curr['previous']}
        if is_constrained(curr['loc'], child['loc'], child['timestep'],
                          constraint_table, agent) == True:
            continue
        closed_list[(child['loc'], child['timestep'])] = child
        push_node(open_list, child,nummer)


    print("No path found, "+str(len(closed_list))+" nodes visited")
    return False, [] # Failed to find solutions

def push_node(open_list, node,nummer=0):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'],nummer, node))

def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_path(goal_node):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    #print(path)
    return path