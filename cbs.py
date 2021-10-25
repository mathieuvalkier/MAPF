"""
Implement CBS here
"""

from single_agent_planner import simple_single_agent_astar
import math
import heapq
import random
random.seed(1)

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location

def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    max_len = min(len(path1),len(path2))

    for i in range(1, max_len):

        node1 = path1[i]
        node2 = path2[i]
        node1p = path1[i - 1]
        node2p = path2[i - 1]

        # vertex
        if node1 == node2:
            return [node1[0], node1[1]]

        # edge
        if node1[0] == node2p[0] and node2[0] == node1p[0]:
            return [node1p[0], node1[0], node1[1]]

    return None

def detect_collisions(paths,agent_ids):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []


    agents = len(paths)
    for i in range(agents-1):
        for j in range(i+1,agents):
            coll = detect_collision(paths[i],paths[j])
            if coll != None:
                #print('dit',{'a1': i, 'a2': j, 'loc': coll[:-1], 'timestep': coll[-1]})
                collisions.append({'a1': agent_ids[i], 'a2': agent_ids[j], 'loc': coll[:-1], 'timestep': coll[-1]})

    return collisions

def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    #print('coll', collision['loc'])

    a = {'ac': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
    b = {'ac': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep']}

    return [a,b]

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, starts, goals, heuristics, nodes_dict,time_start, agent_id, currents, constraint):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.nodes_dict = nodes_dict
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.start_times = time_start
        self.agent_ids = agent_id
        self.currents = currents
        self.constraint = constraint

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = heuristics

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        #self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}

        for i in range(self.num_of_agents):  # Find initial path for each agent

            root['constraints'] = list(self.constraint[i])
            # path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
            #               i, root['constraints'])
            success, path = simple_single_agent_astar(self.nodes_dict, self.starts[i], self.goals[i], self.heuristics,
                                             self.start_times[i], self.agent_ids[i], root['constraints'])

            if path is None:
                raise BaseException('No solutions')

            # print(self.agent_ids[i], path, self.currents[i])

            # if self.currents[i] != -1:
            #     root['paths'].append([self.currents[i]] + path)
            # else:
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'],self.agent_ids)
        self.push_node(root)

        # Task 3.1: Testing
        #print(root['collisions'])

        # Task 3.2: Testing
        #for collision in root['collisions']:
            #print(standard_splitting(collision))
            #print('root',collision)

        while len(self.open_list) > 0:


            parent = self.pop_node()

            if len(parent['collisions']) == 0:
                # print('parent', parent['paths'])
                return parent['paths']


            constraints = standard_splitting(parent['collisions'][0])
            #print('const', constraints)

            for constraint in constraints:

                #get the previous constraints and add new constraint
                constraintlist =  list(parent['constraints'])
                constraintlist.append(constraint)

                ai = self.agent_ids.index(constraint['ac'])

                child = {'cost': 0,
                        'constraints': constraintlist + list(self.constraint[ai]),
                        'paths': parent['paths'],
                        'collisions': []}



                # print('ai', ai, self.agent_ids[ai])


                # path = a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai],
                #               ai, child['constraints'])

                success, path = simple_single_agent_astar(self.nodes_dict, self.starts[ai], self.goals[ai], self.heuristics,
                                                 self.start_times[ai], self.agent_ids[ai], child['constraints'])


                if path != None:
                    # if self.currents[ai] != -1:
                    #     child['paths'][ai] = [self.currents[ai]] + path
                    # else:
                    child['paths'][ai] = path
                    child['collisions'] = detect_collisions(child['paths'],self.agent_ids)
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)

        return root['paths']


def run_CBS(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    #raise Exception("CBS not defined yet.")

    starts = []
    goals = []
    time_starts = []
    agent_ids = []
    currents = []
    constraints = []
    lock = []


    #for all ac in aircraft_lst get index (Not self.id!) and self
    for id, ac in enumerate(aircraft_lst):

        if ac.status == "taxiing":
            starts.append(ac.from_to[0])#path_to_goal[0][0])
            goals.append(ac.goal)
            time_starts.append(t) #Need to finish current movement first
            agent_ids.append(ac.id)
            currents.append((ac.from_to[0],t))
            lock.append([ac.goal, len(ac.path_to_goal)])

            backward = []
            for entry in nodes_dict[ac.from_to[0]]['neighbors']:
                if entry != ac.from_to[1]:
                    backward.append(
                        {'ac': ac.id,              #Add constraint for current ac
                         'loc': [entry],
                         'timestep': t+0.5})

            backward.append(
                {'ac': ac.id,  # Add constraint for current ac
                 'loc': [ac.previous],
                 'timestep': t + 0.5})
            print({'ac': ac.id,  # Add constraint for current ac
                 'loc': [ac.previous],
                 'timestep': t + 0.5})
            constraints.append(backward)

        #make lists: starts, goals, time_starts, agent_ids

        if ac.spawntime == t:



            starts.append(ac.start)
            goals.append(ac.goal)
            time_starts.append(t)
            agent_ids.append(ac.id)
            currents.append(-1)
            constraints.append([])

            for loc in lock:
                # print('lock', loc[0], loc[1])
                if ac.start == loc[0] and loc[1]<3:
                    starts.pop()
                    start_node = ac.start
                    while start_node == ac.start:
                        A_goal_nodes = [97.0, 34.0, 35.0, 36.0, 98.0]
                        start_node = random.choice(A_goal_nodes)
                    starts.append(start_node)
                    ac.start = start_node



            # print('start',starts)

            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]

            cbs = CBSSolver(starts, goals, heuristics, nodes_dict,time_starts, agent_ids, currents, constraints)
            paths = cbs.find_solution()

            # print('hier', paths)



            # print('path list')
            # for path in paths:
            #     print(path)

            for ids, path in enumerate(paths):
                # print('paths', path)



                aclist = aircraft_lst[agent_ids[ids]]
                if len(path) == 0:
                    aclist.status = "arrived"
                    continue
                aclist.path_to_goal = path[1:]
                next_node_id = aclist.path_to_goal[0][0]  # next node is first node in path_to_goal
                aclist.from_to = [path[0][0], next_node_id]
                print("Path AC", aclist.id, ":", path)

            # for id, ac_2 in enumerate(aircraft_lst):
            #
            #     ai = self.agent_ids.index(constraint['ac'])
            #
            #     ac_2.path_to_goal = paths[id][1:]
            #     next_node_id = ac_2.path_to_goal[0][0]  # next node is first node in path_to_goal
            #     ac_2.from_to = [paths[id][0][0], next_node_id]
            #     print("Path AC", ac_2.id, ":", paths[id])
            #     #ac_2.path_total = path

            #Update paths for all aircraft





    return

################################



