from single_agent_planner import simple_single_agent_astar
from math import *
import time as timer
import numpy as np
import random
random.seed(1)





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


def run_individual(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints):

    #separation check
    #for id, ac in enumerate(aircraft_lst):
        #print()
        #if violated, stay still, check again for new violations.

    #Check for future conflicts

    #for all ac in aircraft_lst get index (Not self.id!) and self
    for id, ac in enumerate(aircraft_lst):

        N,E,S,W = 0,0,0,0
        for ac_2 in aircraft_lst:

            x1 = ac.position[0]
            x2 = ac_2.position[0]
            y1 = ac.position[1]
            y2 = ac_2.position[1]
            
            dx = x2 - x1
            dy = y2 - y1

            distance = sqrt( dx**2 + dy**2)
            add = [ac_2.id, ac_2.heading, ac_2.from_to, ac_2.position, ac_2.size]
            if distance <= 2 and add not in ac.vision and ac.id != ac_2.id:
                ac.vision.append(add)
                print('vision', ac.id, ac.vision)
                if dx > 0:
                    E += 1
                if dx < 0:
                    W += 1

                if dy > 0:
                    N += 1
                if dy < 0:
                    S += 1
        ac.busyness = [N,E,S,W]


        
        #Maintaining separation
        if ac.seperation:
            curr_node = ac.from_to[0]
            next_node = ac.from_to[1]
            constraints = []
        
            # for i in nodes_dict[curr_node]['neighbors']:
            #     print('i', i)
            for n, item in enumerate(ac.vision):
                close_curr_node = item[2][0] 
                close_next_node = item[2][1]
                x_position = item[3][0]
                y_position = item[3][1]
                sep_dx = x_position - ac.position[0]
                sep_dy = y_position - ac.position[1]
                # print('sepx',sep_dx)
                # print('sepy',sep_dy)
                
                behind = False
                diagonal = False
                
                if curr_node != 0 and next_node != 0:
                    if close_curr_node == nodes_dict[curr_node]['neighbors']:
                        sep_dx <= abs(0.5)
                        sep_dy <= abs(0.5)
                    # if sep_dx == 0.5 and sep_dy == 0.5:
                    #     nodes_dict[curr_node]['neighbors'] = close_curr_node
                    if close_curr_node == ac.from_to[1]: # or close_next_node == ac.from_to[1] and ac.id != item[0] :
                        behind = True
                    
                    #hold distance when having the same heading
                    if behind:
                        if item[1] == ac.heading:
                            if sep_dy == 0 and sep_dx <= abs(0.5):
                                for neighbor in nodes_dict[curr_node]['neighbors']:
                                    constraints.append(
                                        {'ac': ac.id,
                                          'loc': [neighbor],
                                          'timestep': ac.path_to_goal[0][1]})
                                    
                                success, path = simple_single_agent_astar(nodes_dict, curr_node, ac.goal, heuristics, t, ac.id, constraints)
                    
                                ac.path_to_goal = path[1:]
                                next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                                ac.from_to = [path[0][0], next_node_id]
                                print("Path AC", ac.id, ":", path)
                                ac.path_total = path
                    
                                # Check the path
                                if path[0][1] != t:
                                    raise Exception("Something is wrong with the timing of the path planning")
                                    
                            elif sep_dx == 0 and sep_dy <= abs(0.5):
                                for neighbor in nodes_dict[curr_node]['neighbors']:
                                    constraints.append(
                                        {'ac': ac.id,
                                          'loc': [neighbor],
                                          'timestep': ac.path_to_goal[0][1]})
                                    
                                success, path_n = simple_single_agent_astar(nodes_dict, curr_node, ac.goal, heuristics, t, ac.id, constraints)
                    
                                ac.path_to_goal = path_n[1:]
                                next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                                ac.from_to = [path_n[0][0], next_node_id]
                                print("Path AC", ac.id, ":", path_n)
                                # ac.path_total = path
                    
                                # Check the path
                                if path_n[0][1] != t:
                                    raise Exception("Something is wrong with the timing of the path planning")
  
            # ac.seperation = False
                    
                    #hold diagonal distance
                    else:
                        behind = False
                        sep_dxy = sqrt(sep_dx**2 + sep_dy**2)
                        if  (ac.heading + 90) == item[1] and close_next_node == ac.from_to[1]:
                            diagonal = True
                        elif  (ac.heading - 90) == item[1] and close_next_node == ac.from_to[1]:
                            diagonal = True
                        elif ac.heading == 270 and item[1] == 0 and close_next_node == ac.from_to[1]:
                            diagonal = True
                        elif ac.heading == 0 and item[1] == 270 and close_next_node == ac.from_to[1]:
                            diagonal = True
                        
                        if diagonal:
                            print('sep', sep_dxy)
                            if sep_dxy <= 1 and item[4] > ac.size:    #and ac.crossingwait == True:
                                for entry in nodes_dict[curr_node]['neighbors']:
                                    constraints.append(
                                        {'ac': ac.id,
                                          'loc': [entry],
                                          'timestep': ac.path_to_goal[0][1]})
                                
                                success, path2 = simple_single_agent_astar(nodes_dict, curr_node, ac.goal, heuristics, t, ac.id, constraints)
                    
                                ac.path_to_goal = path2[1:]
                                next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                                ac.from_to = [path2[0][0], next_node_id]
                                print("Path AC", ac.id, ":", path2)
                                # ac.path_total = path
                    
                                # Check the path
                                if path2[0][1] != t:
                                    raise Exception("Something is wrong with the timing of the path planning")
                                    
                            # elif sep_dxy <= 1 and ac.crossingwait == False:
                                
                            #     for entry in nodes_dict[close_curr_node]['neighbors']:
                            #         constraints.append(
                            #             {'ac': ac_2.id,
                            #               'loc': [entry],
                            #               'timestep': ac_2.path_to_goal[0][1]})
                                
                            #     success, path2 = simple_single_agent_astar(nodes_dict, close_curr_node, ac_2.goal, heuristics, t, ac_2.id, constraints)
                    
                            #     ac_2.path_to_goal = path2[1:]
                            #     next_node_id = ac_2.path_to_goal[0][0]  # next node is first node in path_to_goal
                            #     ac_2.from_to = [path2[0][0], next_node_id]
                            #     print("Path AC", ac_2.id, ":", path2)
                            #     # ac.path_total = path
                    
                            #     # Check the path
                            #     if path2[0][1] != t:
                            #         raise Exception("Something is wrong with the timing of the path planning")
                                    
                                    
                                    
                                    
                                    
                                    
                                    
                                    
                                    
                                    
            ac.intersectionsearch = True
            ac.seperation = False

###############################################
            
        if ac.spawntime == t:

            for ac_g in aircraft_lst:
                A_goal_nodes = [97.0, 34.0, 35.0, 36.0, 98.0]
                if ac_g.noreturn and ac_g.goal == ac.start:
                    A_goal_nodes.remove(ac_g.goal)

                if len(A_goal_nodes)<5:
                    start_node = random.choice(A_goal_nodes)
                    ac.start = start_node

            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]

            #If there are changes to the weights, take the new values, otherwise use stander (0 is read as 0.5 in planner)
            edges = 0
            if ac.edges_dict != None:
                edges = ac.edges_dict
            

            success, path = simple_single_agent_astar(nodes_dict, ac.start, ac.goal, heuristics, t, ac.id, constraints, edges)

            ac.path_to_goal = path[1:]
            next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
            ac.from_to = [path[0][0], next_node_id]
            print("Path AC", ac.id, ":", path)
            ac.path_total = path

            # Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")

#################################################

        if ac.replan:
            next_node = ac.from_to[0]
            if nodes_dict[next_node]['type'] == 'intersection':
                temp_edges_dict = edges_dict
                for entries in nodes_dict[next_node]['neighbors']:
                    temp_edges_dict = NESW(temp_edges_dict, next_node, entries, ac.busyness)

                success, path = simple_single_agent_astar(nodes_dict, next_node, ac.goal, heuristics, t, ac.id,
                                                          constraints, edges = temp_edges_dict)

                ac.path_to_goal = path[1:]
                next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                ac.from_to = [path[0][0], next_node_id]
                print("Path AC", ac.id, ":", path)
                ac.path_total = path

                ac.intersectionsearch = True

                # Check the path
                if path[0][1] != t:
                    raise Exception("Something is wrong with the timing of the path planning")

                ac.replan = False

############################################3

        if ac.check_gate:
            constraints = []
            temp_edges_dict = edges_dict  # For to be spawned a/c
            final_loc, final_t = ac.path_to_goal[-1][0], ac.path_to_goal[-1][1]
            for id_gate, ac_gate in enumerate(aircraft_lst):
                if ac_gate.status == None and ac_gate.spawntime < final_t and ac_gate.start == final_loc:
                    #print('test1')
                    for i in range(3):
                        for entry in nodes_dict[ac.from_to[1]]['neighbors']:
                            constraints.append(
                                {'ac': ac.id,  # Add constraint for current ac
                                 'loc': [entry],
                                 'timestep': ac.path_to_goal[0][1]+0.5*i}) #+i

                            temp_edges_dict[(ac.from_to[1], entry)][
                                'weight'] = 0.5 + 10  # Add weights so new a/c will avoid these

                    success, path_new = simple_single_agent_astar(nodes_dict, ac.from_to[0], ac.goal, heuristics, t, ac.id,
                                                              constraints)

                    ac_gate.edges_dict = temp_edges_dict #Store new weights in a/c entry

                    ac.path_to_goal = path_new[1:]
                    next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                    ac.from_to = [path_new[0][0], next_node_id]
                    ("Path AC", ac.id, ":", path_new)

                    ac.intersectionsearch = True

                    # Check the path
                    if path_new[0][1] != t:
                        raise Exception("Something is wrong with the timing of the path planning")

            ac.check_gate = False
            ac.noreturn = True

##########################################


        def distance(loc, node, dist):
            x_n, y_n = nodes_dict[node]['x_pos'], nodes_dict[node]['y_pos']
            d = sqrt( (loc[0] - x_n)**2 + (loc[1] - y_n)**2 )
            if d<dist:
                return False
            if d>dist:
                return True

        for ac_v in ac.vision:
            ac_v = aircraft_lst[ac_v[0]]
            if len(ac.intersections)>2 and len(ac_v.intersections)>2:
                if ac.intersections[1] == ac_v.intersections[1] and ac.intersections[2] != ac_v.intersections[2] and distance(ac.position, ac.intersections[1],0.5) and distance(ac_v.position, ac_v.intersections[1],0.5):

                    # print('kruisbotsing', ac.id, ac_v.id)
                    print()
                    #Dit heb ik ff uitgecomment zodat het andere soepel loopt
                    # if ac.size > ac_v.size:
                    #     ac_v.crossingwait = True
                    # if ac.size == ac_v.size:
                    #     ac.crossingwait = True
                    # else:
                    #     ac.crossingwait = True

                    # print(ac.intersections)
                    # print(ac_v.intersections)
                    #timer.sleep(0.5)

                if ac.intersections[2] == ac_v.intersections[1] and ac.intersections[1] == ac_v.intersections[2]:
                    #print('oversteekbotsing', ac.id, ac_v.id)
                    ac.replannode = ac.intersections[1]
                    ac_v.replannode = ac_v.intersections[1]

                    if ac.size >= ac_v.size:
                        ac.intersectionpriority = [ac_v.id, True, 0]
                        ac_v.intersectionpriority = [ac.id, False, 0]

######################################

        if ac.between and ac.crossingwait and ac.waiting == False:
            # print('croswait',ac.id)
            constraints = []
            for i in range(2):
                for entry in nodes_dict[ac.from_to[0]]['neighbors']:
                    constraints.append(
                        {'ac': ac.id,  # Add constraint for current ac
                         'loc': [entry],
                         'timestep': ac.path_to_goal[0][1] + 0.5 * i})  # +i

            # print(constraints)
            success, path_new = simple_single_agent_astar(nodes_dict, ac.from_to[0], ac.goal, heuristics, t, ac.id,
                                                          constraints)

            ac.path_to_goal = path_new[1:]
            next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
            ac.from_to = [path_new[0][0], next_node_id]
            # print("crossPath AC", ac.id, ":", path_new)
            # ac.path_total = path

            ac.intersectionsearch = True

            # Check the path
            if path_new[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")
            ac.crossingwait = False
            ac.waiting = True

        constraints = []

#############################################

        if ac.replan_inter:

            print('inter', ac.id, ac.intersectionpriority[0] ,ac.path_to_goal)
            print('inter', ac.intersectionpriority)



            if ac.from_to[0] != ac.from_to[1]:
                print('inter wel path')
                temp_edges_dict = edges_dict
                temp_edges_dict[(ac.from_to[0], ac.from_to[1])]['weight'] = 0.5 + 10

                success, path = simple_single_agent_astar(nodes_dict, ac.from_to[0], ac.goal, heuristics, t, ac.id,
                                                          constraints, edges=temp_edges_dict)
            else:
                path = [(ac.from_to[0],ac.path_to_goal[0][1]-0.5)] + ac.path_to_goal

            print('inter 1', path)

            ac_v = aircraft_lst[ac.intersectionpriority[0]]

            if ac.path_to_goal == path[1:] or ac.previous == path[1][0]:
                ac.intersectionpriority[2] = 0
                print('inter 2 true')
                if ac.intersectionpriority[1] == False and ac.id>ac_v.id:
                    path = ac_v.intersectionpriority[2]
                    ac_v.path_to_goal = path[1:]
                    next_node_id = ac_v.path_to_goal[0][0]  # next node is first node in path_to_goal
                    ac_v.from_to = [path[0][0], next_node_id]
                    print("Path AC", ac_v.id, ":", path)
                    ac_v.path_total = path
                    ac_v.intersectionsearch = True
            else:
                print('inter 3 false')
                ac.intersectionpriority[2] = path
                if ac.intersectionpriority[1] == False or (ac.id>ac_v.id and ac_v.intersectionpriority[2] == 0):
                    # print(ac.id, path)
                    ac.path_to_goal = path[1:]
                    next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                    ac.from_to = [path[0][0], next_node_id]
                    print("Path AC", ac.id, ":", path)
                    ac.path_total = path
                    ac.intersectionsearch = True

            ac.replan_inter = False


            # ac.path_to_goal = path[1:]
            # next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
            # ac.from_to = [path[0][0], next_node_id]
            # print("Path AC", ac.id, ":", path)
            # ac.path_total = path
            #
            # ac.intersectionsearch = True
            #
            # # Check the path
            # if path[0][1] != t:
            #     raise Exception("Something is wrong with the timing of the path planning")





        #return




