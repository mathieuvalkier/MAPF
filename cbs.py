from single_agent_planner import simple_single_agent_astar
import math
import heapq
import random
random.seed(1)
import time as timer

def get_location(path, time):   # Module to get location at certain time
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]

def detect_collision(path1, path2):         # Detect collision between two paths

    max_len = min(len(path1),len(path2))    # Only look as far as shortest path

    for i in range(1, max_len):

        node1 = path1[i]        # Locations at time t and t-1 for 2 agents
        node2 = path2[i]
        node1p = path1[i - 1]
        node2p = path2[i - 1]

        # Vertex collision
        if node1 == node2:
            return [node1[0], node1[1]]     # Return location and time of node 1

        # Edge collision
        if node1[0] == node2p[0] and node2[0] == node1p[0]:
            return [node1p[0], node1[0], node1[1]]  # Return both locations and time of collision

    return None

def detect_collisions(paths,agent_ids):     # Loop through all agent paths to find collisions

    collisions = []

    agents = len(paths)
    for i in range(agents-1):               # length -1 as list count starts at 0
        for j in range(i+1,agents):         # i+1 to skip current agent
            coll = detect_collision(paths[i],paths[j])
            if coll != None:                # Collect all collisions at location and timesteps
                collisions.append({'a1': agent_ids[i], 'a2': agent_ids[j], 'loc': coll[:-1], 'timestep': coll[-1]})

    return collisions

def standard_splitting(collision):  # Split collision into two constraints

    a = {'ac': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
    b = {'ac': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep']}

    return [a,b]

def get_sum_of_cost(paths):     # Add all lengths of all paths
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

class CBSSolver(object):

    def __init__(self, starts, goals, heuristics, nodes_dict,time_start, agent_id,
                 agent_size, constraint, previous):

        self.nodes_dict = nodes_dict
        self.starts = starts                # Start locations
        self.goals = goals                  # Goal locations
        self.previous = previous            # Previous location to prevent backtracking
        self.num_of_agents = len(goals)
        self.start_times = time_start
        self.agent_ids = agent_id           # id numbers of agents
        self.size = agent_size              # Size of agents
        self.constraint = constraint        # Constraints per agent (going backwards)

        self.num_of_generated = 0
        self.num_of_expanded = 0

        self.open_list = []
        self.heuristics = heuristics        # Lower level heuristics

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

        root = {'cost': 0,                  # Set root node
                'constraints': [],
                'paths': [],
                'collisions': []}

        for i in range(self.num_of_agents):  # Find initial path for all agents

            if len(self.constraint[i]) != 0:
                const = self.constraint[i]
                for elements in const:
                    root['constraints'].append(elements)


            success, path = simple_single_agent_astar(self.nodes_dict, self.starts[i], self.goals[i], self.heuristics,
                                             self.start_times[i], self.agent_ids[i], root['constraints']
                                                      , prev= self.previous[i], cbs = True)

            #if not success:
            #    raise BaseException('No solutions')

            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'],self.agent_ids)
        self.push_node(root)

        start_time = timer.time()


        while len(self.open_list) > 0:


            parent = self.pop_node()

            if len(parent['collisions']) == 0:
                return parent['paths']

            constraints = standard_splitting(parent['collisions'][0])

            a,b = constraints[0]['ac'],constraints[1]['ac']
            c,d= self.agent_ids.index(a),self.agent_ids.index(b)
            e,f = self.size[c], self.size[d]

            if (e<f and self.agent_ids[-1] !=d) or self.agent_ids[-1] ==c :
                constraints = constraints[::-1]

            for constraint in constraints:


                #get the previous constraints and add new constraint
                constraintlist =  list(parent['constraints'])
                constraintlist.append(constraint)

                ai = self.agent_ids.index(constraint['ac'])

                child = {'cost': 0,
                        'constraints': constraintlist + list(self.constraint[ai]),
                        'paths': parent['paths'],
                        'collisions': []}

                success, path = simple_single_agent_astar(self.nodes_dict, self.starts[ai], self.goals[ai], self.heuristics,
                                                 self.start_times[ai], self.agent_ids[ai], child['constraints']
                                                          , prev=self.previous[ai], cbs = True)

                if success:
                    child['paths'][ai] = path
                    child['collisions'] = detect_collisions(child['paths'],self.agent_ids)
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)

                if not success:
                    child['paths'][ai] = []
                    child['collisions'] = detect_collisions(child['paths'], self.agent_ids)
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)





        return root['paths']


def run_CBS(aircraft_lst, nodes_dict, edges_dict, heuristics, t, print_path):

    starts = []
    goals = []
    previous = []
    time_starts = []
    agent_ids = []
    agent_size = []
    constraints = []
    lock = []


    #for all ac in aircraft_lst get index (Not self.id!) and self
    for id, ac in enumerate(aircraft_lst):

        if ac.status == "taxiing":
            starts.append(ac.from_to[0])#path_to_goal[0][0])
            goals.append(ac.goal)
            previous.append(ac.previous)
            time_starts.append(t) #Need to finish current movement first
            agent_ids.append(ac.id)
            agent_size.append(ac.size)
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
            constraints.append(backward)

        #make lists: starts, goals, time_starts, agent_ids

        if ac.spawntime == t:

            starts.append(ac.start)
            goals.append(ac.goal)
            previous.append(ac.previous)
            time_starts.append(t)
            agent_ids.append(ac.id)
            agent_size.append(ac.size)
            constraints.append([])

            for loc in lock:
                if ac.start == loc[0] and loc[1]<3:
                    starts.pop()
                    start_node = ac.start
                    while start_node == ac.start:
                        A_goal_nodes = [97.0, 34.0, 35.0, 36.0, 98.0]
                        start_node = random.choice(A_goal_nodes)
                    starts.append(start_node)
                    ac.start = start_node

            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]


            cbs = CBSSolver(starts, goals, heuristics, nodes_dict,time_starts, agent_ids, agent_size, constraints, previous)
            paths = cbs.find_solution()

            for ids, path in enumerate(paths):

                aclist = aircraft_lst[agent_ids[ids]]
                if len(path) == 0:
                    aclist.status = "arrived"
                    continue
                aclist.path_to_goal = path[1:]
                next_node_id = aclist.path_to_goal[0][0]  # next node is first node in path_to_goal
                aclist.from_to = [path[0][0], next_node_id]

    return

################################



