from single_agent_planner import simple_single_agent_astar
from math import *


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
    #raise Exception("Prioritized planner not defined yet.")

    # print('edges', edges_dict[(3.0,42.0)])
    # print('edges', edges_dict[(42.0, 3.0)])

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
            add = [ac_2.id, ac_2.heading, ac_2.from_to]
            if distance <= 2 and add not in ac.vision and ac.id != ac_2.id:
                ac.vision.append(add)
                if dx > 0:
                    E += 1
                if dx < 0:
                    W += 1

                if dy > 0:
                    N += 1
                if dy < 0:
                    S += 1
        ac.busyness = [N,E,S,W]

        # print(ac.id,'busy', ac.busyness, ac.vision)

        #print(ac.id, 'vision', ac.vision)

        # print('time', ac.spawntime, t)

        if ac.spawntime == t:

            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]

            success, path = simple_single_agent_astar(nodes_dict, ac.start, ac.goal, heuristics, t, ac.id, constraints)

            ac.path_to_goal = path[1:]
            next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
            ac.from_to = [path[0][0], next_node_id]
            print("Path AC", ac.id, ":", path)
            ac.path_total = path

            # Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")


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

                # Check the path
                if path[0][1] != t:
                    raise Exception("Something is wrong with the timing of the path planning")

                ac.replan = False

        if ac.check_gate:
            constraints = []
            final_loc, final_t = ac.path_to_goal[-1][0], ac.path_to_goal[-1][1]
            for id_gate, ac_gate in enumerate(aircraft_lst):
                if ac_gate.status == None and ac_gate.spawntime < final_t and ac_gate.start == final_loc:
                    print('test1')
                    for i in range(3):
                        for entry in nodes_dict[ac.from_to[1]]['neighbors']:
                            constraints.append(
                                {'ac': ac.id,  # Add constraint for current ac
                                 'loc': [entry],
                                 'timestep': ac.path_to_goal[0][1]+0.5*i}) #+i
                    success, path_new = simple_single_agent_astar(nodes_dict, ac.from_to[0], ac.goal, heuristics, t, ac.id,
                                                              constraints)
                    print('test2')
                    ac.path_to_goal = path_new[1:]
                    next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                    ac.from_to = [path_new[0][0], next_node_id]
                    print("Path AC", ac.id, ":", path_new)
                    #ac.path_total = path

                    # Check the path
                    if path_new[0][1] != t:
                        raise Exception("Something is wrong with the timing of the path planning")

            ac.check_gate = False






        #return




