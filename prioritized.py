from single_agent_planner import simple_single_agent_astar
import math


def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, print_path = False):

    '''
    #### Prioritized planner
    # For all planned aircraft, check if it is time to spawn
    # If true, gather time&locations from preceding a/c for vertex constraints
    # Plan route using A*, find edge collisions
    # If present, add edge constraint, plan route using A*
    '''

    for id, ac in enumerate(aircraft_lst):                  # Go through all planned a/c

        if ac.spawntime == t:                               # At spawn time

            constraints = []

            ac.status = "taxiing"                           # Set status and start position
            ac.position = nodes_dict[ac.start]["xy_pos"]


            for prev_ac in range(id):                       # Find vertex constraints based on preceding a/c locations
                for step in aircraft_lst[prev_ac].path_to_goal:
                    constraints.append(                     # Add constraint for current a/c
                        {'ac': ac.id,
                         'loc': [step[0]],
                         'timestep': step[1]})



            colliding = True                # Start loop to eliminate edge collisions

            while colliding:

                success, path = simple_single_agent_astar(nodes_dict, ac.start, ac.goal,
                                                          heuristics, t, ac.id, constraints, prev=ac.previous)

                if ac.id == 0:
                    colliding = False       # First a/c has always no collisions


                new_constraint = False      # Keep looking for edge collisions till no new constraints

                for j in range(len(path) - 2):      # Go through all path steps except last step,
                                                    # (length -2 as path count starts at 0)

                    for prev_ac in range(id):       # Go through all previous a/c
                        path_prev = aircraft_lst[prev_ac].path_to_goal

                        if len(path_prev)<j+2:      # If previous a/c path is already finished, no collision possible
                            continue

                        if path[j][0] == path_prev[j][0] and path[j+1][0] == path_prev[j-1][0]:

                            constraints.append(     # Add edge constraint for current a/c if edge collision
                                {'ac': ac.id,
                                 'loc': [path[j][0],path[j+1][0]],
                                 'timestep': path[j+1][1]})
                            new_constraint = True   # Path needs to be replanned again using A*

                if new_constraint == False:         # If no more ccollisions, path is finished
                    colliding = False
 
            if success is False:                    # Fail-safe if a/c cannot be planned
                print('no path found for', ac.id)
                ac.status = 'Fail'
                break

            ac.path_to_goal = path[1:]              # Push path
            next_node_id = ac.path_to_goal[0][0]    # Set next node
            ac.from_to = [path[0][0], next_node_id] # Update from-to nodes

            if print_path:
                print("Path AC", ac.id, ":", path)

            # Check the path for time inconsistency
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")
