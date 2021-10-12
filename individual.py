from single_agent_planner import simple_single_agent_astar
import math


def run_individual(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints):
    #raise Exception("Prioritized planner not defined yet.")

    #separation check
    for id, ac in enumerate(aircraft_lst):
        print()
        #if violated, stay still, check again for new violations.

    #Check for future conflicts

    #for all ac in aircraft_lst get index (Not self.id!) and self
    for id, ac in enumerate(aircraft_lst):

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

        #return