from single_agent_planner import simple_single_agent_astar
import math
import heapq
import random
random.seed(1)
import time as timer

'''
#### CBS planner
# run_CBS: loop through all aircraft and set important data in lists for cbs solver
# If new aircraft can be spawned, find paths of all aircraft using cbs solver class
# Push paths to all aircraft and continue loop
'''

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

        '''
        #### Solver
        # Initial paths are found with existing constraints and start and goals
        # Find collisions and make constraints for both involved aircraft
        # For each constraint, find a solution
        # Continue with the solution with the lowest cost
        # If no collisions are left, stop the solver, return path
        '''

        root = {'cost': 0,                  # Set root node
                'constraints': [],
                'paths': [],
                'collisions': []}

        for i in range(self.num_of_agents):  # Find initial path for all agents

            if len(self.constraint[i]) != 0:    # If there are constraints for certain agents
                const = self.constraint[i]
                for elements in const:          # Append constraints to the node
                    root['constraints'].append(elements)

            # Use A* to find initial paths
            success, path = simple_single_agent_astar(self.nodes_dict, self.starts[i], self.goals[i], self.heuristics,
                                             self.start_times, self.agent_ids[i], root['constraints']
                                                      , prev= self.previous[i], cbs = True)
            # Append path to node
            root['paths'].append(path)

        # Get the total cost of all the paths and find collisions
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'],self.agent_ids)
        self.push_node(root)

        start_time = timer.time()

        while len(self.open_list) > 0:      # While there are nodes in the open_list

            parent = self.pop_node()        # Take the node with the least cost

            if len(parent['collisions']) == 0:  # If there are no collisions, stop solver
                return parent['paths']

            constraints = standard_splitting(parent['collisions'][0])   # make constraints from collisions

            for constraint in constraints:  # For both constraints find a solution


                #get the previous constraints and add new constraint
                constraintlist =  list(parent['constraints'])
                constraintlist.append(constraint)

                ai = self.agent_ids.index(constraint['ac'])     # current aircraft index indicated with 'ai'

                child = {'cost': 0,
                        'constraints': constraintlist + list(self.constraint[ai]),
                        'paths': parent['paths'],
                        'collisions': []}

                # Find a path for the child node with new constraints
                success, path = simple_single_agent_astar(self.nodes_dict, self.starts[ai], self.goals[ai], self.heuristics,
                                                 self.start_times, self.agent_ids[ai], child['constraints']
                                                          , prev=self.previous[ai], cbs = True)

                # If there is a path, push the path to the open_list and find collisions and cost
                if success:
                    child['paths'][ai] = path
                    child['collisions'] = detect_collisions(child['paths'],self.agent_ids)
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)

                # If there is no possible solution, no path is returned, aircraft is removed from simulation
                # Only used in case of deadlocks
                if not success:
                    child['paths'][ai] = []
                    child['collisions'] = detect_collisions(child['paths'], self.agent_ids)
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)

        return root['paths']


def run_CBS(aircraft_lst, nodes_dict, edges_dict, heuristics, t, print_path):
    '''
    #### run CBS
    # Data is gathered for all taxiing aircraft
    # Constraints are formed for going backwards
    # If t = spawntime of an aircraft, check if there is no deadlock
    # Solve using cbs solver
    # push paths to all taxiing aircraft
    '''

    starts = []         # Start/current node
    goals = []          # Goal nodes
    previous = []       # Previous node
    agent_ids = []      # Agent id
    agent_size = []     # Agent size
    constraints = []    # Constraints (backwards/already moving)
    lock = []           # list of goal and length of path left

    #for all ac in aircraft_lst get index (id) and self (ac)
    for id, ac in enumerate(aircraft_lst):

        if ac.status == "taxiing":
            starts.append(ac.from_to[0])
            goals.append(ac.goal)
            previous.append(ac.previous)
            agent_ids.append(ac.id)
            agent_size.append(ac.size)
            lock.append([ac.goal, len(ac.path_to_goal)])

            prohibited = []
            # Make constraints for not finishing current movement
            for entry in nodes_dict[ac.from_to[0]]['neighbors']:
                if entry != ac.from_to[1]:
                    prohibited.append(
                        {'ac': ac.id,              #Add constraint for current ac
                         'loc': [entry],
                         'timestep': t+0.5})

            # Make constraint for going backwards
            prohibited.append(
                {'ac': ac.id,  # Add constraint for current ac
                 'loc': [ac.previous],
                 'timestep': t + 0.5})

            constraints.append(prohibited)

        if ac.spawntime == t:

            starts.append(ac.start)
            goals.append(ac.goal)
            previous.append(ac.previous)
            agent_ids.append(ac.id)
            agent_size.append(ac.size)
            constraints.append([])

            # Check for previous aircraft if start gate is someone's arrival gate
            # If other aircraft is too close to gate, find new departure gate
            for loc in lock:
                if ac.start == loc[0] and loc[1]<3:
                    starts.pop()            # Remove start location from list
                    start_node = ac.start
                    while start_node == ac.start:   # Find new possible gate until different one is found
                        A_goal_nodes = [97.0, 34.0, 35.0, 36.0, 98.0]
                        start_node = random.choice(A_goal_nodes)
                    starts.append(start_node)   # Update aircraft start node
                    ac.start = start_node

            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]

            # Solve using cbs solver class
            cbs = CBSSolver(starts, goals, heuristics, nodes_dict,t, agent_ids, agent_size, constraints, previous)
            paths = cbs.find_solution()

            # for all ac in path get index of path (ids) and paths (path)
            for ids, path in enumerate(paths):

                # For all taxiing aircraft, update paths.
                aclist = aircraft_lst[agent_ids[ids]]
                if len(path) == 0:                  # First check if arrived
                    aclist.status = "arrived"
                    continue
                aclist.path_to_goal = path[1:]                  # Push path to aircraft
                next_node_id = aclist.path_to_goal[0][0]
                aclist.from_to = [path[0][0], next_node_id]

    return

################################


# a, b = constraints[0]['ac'], constraints[1]['ac']
# c, d = self.agent_ids.index(a), self.agent_ids.index(b)
# e, f = self.size[c], self.size[d]
#
# if (e < f and self.agent_ids[-1] != d) or self.agent_ids[-1] == c:
#     constraints = constraints[::-1]


