"""
Implement prioritized planner here
"""

from single_agent_planner import simple_single_agent_astar
import math

def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints):
    #raise Exception("Prioritized planner not defined yet.")

    #for all ac in aircraft_lst get index (Not self.id!) and self
    for id, ac in enumerate(aircraft_lst):

        if ac.spawntime == t:

            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]

            # vertex constraints all other preceding ac.path_to_goal
            for i in range(id):
                i_ac = aircraft_lst[i]          #obtain ac
                for step in i_ac.path_to_goal:  #loop through path to goal for preceding ac
                    constraints.append(
                        {'ac': ac.id,              #Add constraint for current ac
                         'loc': [step[0]],
                         'timestep': step[1]})


            #Loop to find non-colliding path
            colliding = True
            while colliding:

                success, path = simple_single_agent_astar(nodes_dict, ac.start, ac.goal, heuristics, t, ac.id, constraints)

                if path is None:
                    raise BaseException("No solution found for", self.id)

                if ac.id == 0:    #This is for the first ac, always priority
                    colliding = False

                # look for edge collisions, add constraint and rerun astar
                new_constraint = False
                for j in range(len(path) - 2):
                    for ids in range(id):
                        ids_ac = aircraft_lst[ids]
                        if len(ids_ac.path_to_goal)<j+2:
                            continue
                        #print('len',j,path[j+1],ids_ac.path_to_goal[j-1])
                        if path[j][0] == ids_ac.path_to_goal[j][0] and path[j+1][0] == ids_ac.path_to_goal[j-1][0]:
                            #print('ja edge')
                            constraints.append(
                                {'ac': ac.id,  # Add constraint for current ac
                                 'loc': [path[j][0],path[j+1][0]],
                                 'timestep': path[j+1][1]})
                            new_constraint = True

                if new_constraint == False:
                    colliding = False


            ac.path_to_goal = path[1:]
            next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
            ac.from_to = [path[0][0], next_node_id]
            print("Path AC", ac.id, ":", path)

            # Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")

    return constraints