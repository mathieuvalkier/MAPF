from single_agent_planner import simple_single_agent_astar
from math import *
import time as timer
import numpy as np
import random
random.seed(1)


def push_path(ac, path, t, print_path=False):
    ac.path_to_goal = path[1:]
    if path == []:
        ac.status == 'error'
        print(path)
    if path != []:
        next_node_id = ac.path_to_goal[0][0]
        ac.from_to = [path[0][0], next_node_id]

    if print_path:
        print("Path AC", ac.id, ":", path)

    # # Check the path
    # if path[0][1] != t:
    #     raise Exception("Something is wrong with the timing of the path planning")

def wait_node(ac,print_path=True):
    path = ac.path_to_goal
    wait_node = path[0]
    new_path = []
    for node in path:
        loc, time = node[0], node[1] + 0.5
        new_path.append((loc, time))

    ac.path_to_goal = [wait_node] + new_path

    if print_path:
        path = [ac.from_to[0]] + ac.path_to_goal
        print("Path AC", ac.id, ":", path)

def NESW(edges_dict, curr, nex, nesw):
    xys = edges_dict[(curr, nex)]['start_end_pos']
    x1 = xys[0][0]
    x2 = xys[1][0]
    y1 = xys[0][1]
    y2 = xys[1][1]

    dx = x2 - x1
    dy = y2 - y1

    if dx > 0:
        value = nesw[1]
    if dx < 0:
        value = nesw[3]

    if dy > 0:
        value = nesw[0]
    if dy < 0:
        value = nesw[2]

    edges_dict[(curr, nex)]['weight'] = 0.5 + value

    return (edges_dict)


def run_individual(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints, print_path):

    #for all ac in aircraft_lst get index
    for id, ac in enumerate(aircraft_lst):


        '''
        #### Determine busyness
        # For the current a/c, count how many other a/c are in vision
        # Sort into north, east,...
        # The more busy a direction, the higher the weight for that direction becomes
        '''

        N,E,S,W = 0,0,0,0
        for ac_2 in aircraft_lst:

            #Determine if a/c is in vision ( distance<2 )
            x1, y1 = ac.position[0], ac.position[1]
            x2, y2 = ac_2.position[0], ac_2.position[1]

            dx, dy = x2 - x1, y2 - y1

            distance = sqrt( dx**2 + dy**2)

            ac_data = [ac_2.id, ac_2.heading, ac_2.from_to, ac_2.position, ac_2.size]

            if distance <= 2 and ac_data not in ac.vision and ac.id != ac_2.id:
                ac.vision.append(ac_data)
                if dx > 0:
                    E += 1
                if dx < 0:
                    W += 1

                if dy > 0:
                    N += 1
                if dy < 0:
                    S += 1
        ac.busyness = [N,E,S,W] #List holds values for a/c directions

        #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ####
        '''
        #### Maintaining separation
        # IF both a/c's are taxiing on the same edge with the same heading and their seperation distance is <= 0.5:
        # => a/c behind waits for 1 timestep
        # IF 2 a/c's are heading to the same intersection and the diagonal distance between them is < 1:  
        # => a/c with lowest size waits for 1 timestep
        #
        '''

        if ac.seperation:
            curr_node = ac.from_to[0]
            next_node = ac.from_to[1]
            constraints = []

            for n, item in enumerate(ac.vision):
                close_curr_node = item[2][0] 
                close_next_node = item[2][1] 
                x_position = item[3][0]
                y_position = item[3][1]
                sep_dx = x_position - ac.position[0]
                sep_dy = y_position - ac.position[1]

                behind = False
                diagonal = False
                
                if curr_node != 0 and next_node != 0:
                    if close_curr_node == nodes_dict[curr_node]['neighbors']:
                        sep_dx <= 0.5
                        sep_dy <= 0.5
            
                    if close_curr_node == ac.from_to[1]:    #Current a/c behind other a/c in radar
                        behind = True                       #Turn on condition for a/c's that are behind
                    
                    #Hold axial distance in x and y direction with same heading
                    if behind:
                        if item[1] == ac.heading:            #Same heading
                        
                            #Horizontal distance
                            if sep_dy == 0 and sep_dx <= abs(0.5):
                                for neighbor in nodes_dict[curr_node]['neighbors']:
                                    constraints.append(
                                        {'ac': ac.id,           #Add edge constraint for current a/c
                                          'loc': [neighbor],
                                          'timestep': ac.path_to_goal[0][1]})    #For 1 timestep 
                                    
                                success, path = simple_single_agent_astar(nodes_dict, curr_node, ac.goal, heuristics, t, ac.id, constraints, prev=ac.previous)

                                push_path(ac, path, t, print_path)  # Push path
                                
                            #Vertical distance
                            elif sep_dx == 0 and sep_dy <= abs(0.5):
                                for neighbor in nodes_dict[curr_node]['neighbors']:
                                    constraints.append(
                                        {'ac': ac.id,
                                          'loc': [neighbor],
                                          'timestep': ac.path_to_goal[0][1]})
                                    
                                success, path_n = simple_single_agent_astar(nodes_dict, curr_node, ac.goal, heuristics, t, ac.id, constraints, prev=ac.previous)

                                push_path(ac, path_n, t, print_path)  # Push path
                    
                    #Hold diagonal distance
                    else:
                        behind = False                                                          #Turn off: next condition is not valid for behind
                        sep_dxy = sqrt(sep_dx**2 + sep_dy**2)                                   #Diagonal seperation distance
                        if  (ac.heading + 90) == item[1] and close_next_node == ac.from_to[1]:
                            diagonal = True                                                     #Turn on condition for a/c's heading to same intersection
                        elif  (ac.heading - 90) == item[1] and close_next_node == ac.from_to[1]:
                            diagonal = True
                        elif ac.heading == 270 and item[1] == 0 and close_next_node == ac.from_to[1]:
                            diagonal = True
                        elif ac.heading == 0 and item[1] == 270 and close_next_node == ac.from_to[1]:
                            diagonal = True

                        if diagonal:
                            if sep_dxy <= 1 and item[4] >= ac.size:    #Current a/c lower or equal to a/c in radar
                                for entry in nodes_dict[curr_node]['neighbors']:
                                    constraints.append(
                                        {'ac': ac.id,                  #Add edge constraint to current a/c
                                          'loc': [entry],
                                          'timestep': ac.path_to_goal[0][1]}) #For 1 timestep

                                success, path2 = simple_single_agent_astar(nodes_dict, curr_node, ac.goal, heuristics, t, ac.id, constraints, prev=ac.previous)

                                push_path(ac, path2, t, print_path)  # Push path
                                    
            ac.intersectionsearch = True    #Turn on searching for intersections
            ac.seperation = False           #Turn off seperation conditions (horizontal/vertical/diagonal)

        #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ####
        '''
        ##### Spawn new a/c
        # First check if other a/c are near the gate and cannot return => replace start node
        # Then check if there are changes to edge weights (example: another a/c is waiting outside )
        # Plan path
        '''

        if ac.spawntime == t:

            for ac_g in aircraft_lst:
                A_goal_nodes = [97.0, 34.0, 35.0, 36.0, 98.0]   # Original departure gates
                if ac_g.noreturn and ac_g.goal == ac.start:     # If a/c cannot return and gate is the same
                    A_goal_nodes.remove(ac_g.goal)              # Remove gate from possibilities

                if len(A_goal_nodes)<5:                         # If there are restricted gates
                    start_node = random.choice(A_goal_nodes)    # Pick random new gate
                    ac.start = start_node

            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]

            edges = 0                   # 0 in edge is read as standard weight in planner
            if ac.edges_dict != None:   # If there are changes to the weights, take the new values,
                edges = ac.edges_dict

            success, path = simple_single_agent_astar(nodes_dict, ac.start, ac.goal, heuristics, t, ac.id, constraints, edges, prev=ac.previous)

            push_path(ac, path, t, print_path)  # Push path

        #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ####
        '''
        ##### Replan a/c w.r.t busyness
        # If an a/c's next step is an intersection, replan the path
        # New path is based on busyness (north, east, ...)
        # Each a/c in vision increases edge weight by 1
        '''

        if ac.replan:
            next_node = ac.from_to[0]
            if nodes_dict[next_node]['type'] == 'intersection':     # Only if next node is intersection
                temp_edges_dict = edges_dict                        # Make temporary edge_dict
                for entries in nodes_dict[next_node]['neighbors']:  # Change the weights of directions based on busyness
                    temp_edges_dict = NESW(temp_edges_dict, next_node, entries, ac.busyness)

                success, path = simple_single_agent_astar(nodes_dict, next_node, ac.goal, heuristics, t, ac.id,
                                                          constraints, edges = temp_edges_dict, prev=ac.previous)

                push_path(ac, path, t, print_path)  # Push path

                ac.intersectionsearch = True    # New path means new upcoming intersections
                ac.replan = False               # Turn replan off

        #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ####
        '''
        #### Check spawning a/c
        # Check if an a/c will spawn at the goal gate
        # If true: wait before gate intersection for 3 steps
        # Also: prevent new a/c from colliding with waiting a/c using weights
        '''

        if ac.check_gate:
            constraints = []
            temp_edges_dict = edges_dict  # Temporary edge_dict for new a/c
            final_loc, final_t = ac.path_to_goal[-1][0], ac.path_to_goal[-1][1]

            for id_gate, ac_gate in enumerate(aircraft_lst):

                # Find a/c with no status that spawns before current a/c arrives at the gate
                if ac_gate.status == None and ac_gate.spawntime < final_t and ac_gate.start == final_loc:

                    for i in range(3):
                        for entry in nodes_dict[ac.from_to[1]]['neighbors']:
                            constraints.append(
                                {'ac': ac.id,                               # Add constraint for current a/c
                                 'loc': [entry],
                                 'timestep': ac.path_to_goal[0][1]+0.5*i})  # For three timesteps

                            # Add weights so new a/c will avoid the waiting a/c
                            temp_edges_dict[(ac.from_to[1], entry)]['weight'] = 0.5 + 10

                    success, path_new = simple_single_agent_astar(nodes_dict, ac.from_to[0], ac.goal, heuristics, t, ac.id,
                                                              constraints, prev=ac.previous)

                    ac_gate.edges_dict = temp_edges_dict    # Store new weights for new a/c

                    push_path(ac, path_new, t, print_path)  # Push path

                    ac.intersectionsearch = True    # New path means new upcoming intersections

            ac.check_gate = False   # Turn of gate check
            ac.noreturn = True      # a/c cannot return anymore now

        #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ####
        '''
        ##### Check for crossing collisions
        # For all a/c in vision, check if right/left turn causes collision
        # Check by comparing upcoming intersections of the a/c
        '''

        for ac_v in ac.vision:
            ac_v = aircraft_lst[ac_v[0]]                                # Get a/c from vision list
            if len(ac.intersections)>2 and len(ac_v.intersections)>2:   # Only check if in the field (exclude new spawn)

                ac1_1,ac1_2, ac1_3 = ac.intersections[0],ac.intersections[1],ac.intersections[2]
                ac2_1, ac2_2, ac2_3 = ac_v.intersections[0], ac_v.intersections[1], ac_v.intersections[2]


                a = ac1_3 == ac2_2 and ac1_2 == ac2_3
                c = ac1_1 == ac2_3 and ac1_2 == ac2_2 and ac1_3 == ac2_1
                d = ac1_1 != ac2_1 and ac1_2 == ac2_2 and ac1_3 == ac2_3
                e = ac1_1 != ac2_1 and ac1_2 == ac2_3 and ac1_3 == ac2_2

                b = ac1_1 == ac2_3 and ac1_2 == ac2_2 and ac1_3 != ac2_1

                if a or c or d or e: #or (ac.intersections[2] == ac_v.intersections[0] and ac.intersections[0] == ac_v.intersections[2]):
                    ac.replannode = ac.intersections[0]             # If next intersect. is reached, replan the a/c
                    ac_v.replannode = ac_v.intersections[0]

                    # Determine priority: first size, then id number
                    if ac.size > ac_v.size or (ac.size == ac_v.size and ac.id>ac_v.id):
                        ac.intersectionpriority = [ac_v.id, True, 0]        # [id, priority, new path (empty)]
                        ac_v.intersectionpriority = [ac.id, False, 0]

                if b and not ac_v.sideside[0]:
                    ac_v.sideside = [True, ac2_2, round(t * 2) / 2+1]

        if ac.sideside[0]:

            if nodes_dict[ac.path_to_goal[0][0]]['type'] == 'intersection':
                ac.sideside = [False]
                continue

            if ac.waiting:
                continue

            if ac.intersections[0] == ac.sideside[1]:
                ac.waiting = False
                ac.sideside = [False]
                continue

            wait_node(ac, print_path)

            ac.intersectionsearch = True  # New path means new upcoming intersections
            ac.waiting = True





        #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ####
        '''
        ##### Replan crossing collision
        # Create a new path with high weight on collision path
        #  => If a/c is already standing still, take old path, let other take new path
        # If there is no new path possible, take path of other a/c, even if it has priority
        '''

        if ac.replan_inter:

            if ac.from_to[0] != ac.from_to[1]:
                temp_edges_dict = edges_dict
                temp_edges_dict[(ac.from_to[0], ac.from_to[1])]['weight'] = 0.5 + 10    # Set weight high for collision edge

                if nodes_dict[ac.path_to_goal[0][0]]['type'] == 'intersection':
                    ac.replan_inter = False
                    continue

                success, path = simple_single_agent_astar(nodes_dict, ac.from_to[0], ac.goal, heuristics, t, ac.id,
                                                          constraints, edges=temp_edges_dict, prev=ac.previous)
                
                #push_path(ac, path, t, print_path)      #Push path
                
            else:
                #If a/c already stands still, take old path => automatically take other a/c path
                path = [(ac.from_to[0],ac.path_to_goal[0][1]-0.5)] + ac.path_to_goal


            ac_v = aircraft_lst[ac.intersectionpriority[0]]

            if ac.path_to_goal == path[1:] or ac.previous == path[1][0]:    # If path is not possible
                ac.intersectionpriority[2] = 0                              # Set new path to empty

                if ac.intersectionpriority[1] == False and ac.id>ac_v.id:   # If a/c has no priority but no path
                    path = ac_v.intersectionpriority[2]                     # Take path of other a/c

                    if path != 0:
                        push_path(ac_v, path, t, print_path)  # Push path

                    ac_v.intersectionsearch = True  # New path means new upcoming intersections
            else:
                ac.intersectionpriority[2] = path   # If path is possible, store path in list

                # If ac has no priority and a path or this a/c has prio but other a/c no path
                if ac.intersectionpriority[1] == False or (ac.id>ac_v.id and ac_v.intersectionpriority[2] == 0):

                    push_path(ac, path, t, print_path)  # Push path

                    ac.intersectionsearch = True    # New path means new upcoming intersections

            ac.replan_inter = False     # Turn off replan for intersection

        #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### #### ####
