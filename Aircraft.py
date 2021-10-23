from single_agent_planner import simple_single_agent_astar
import math

#test

class Aircraft(object):
    """Aircraft class, should be used in the creation of new aircraft."""

    def __init__(self, flight_id, a_d, start_node, goal_node, spawn_time, nodes_dict, size = 1):
        """
        Initalisation of aircraft object.
        INPUT:
            - flight_id: [int] unique id for this aircraft
            - a_d: [str] "a" for arrival flight and "d" for departure flight
            - start_node: node_id of start node
            - goal_node: node_id of goal node
            - spawn_time: spawn_time of a/c 
            - nodes_dict: copy of the nodes_dict
        """
        
        #Fixed parameters
        self.speed = 1         #how much a/c moves per unit of t
        self.id = flight_id       #flight_id
        self.type = a_d           #arrival or departure (A/D)
        self.size = size
        self.spawntime = spawn_time #spawntime
        self.start = start_node   #start_node_id
        self.goal = goal_node     #goal_node_id
        self.nodes_dict = nodes_dict #keep copy of nodes dict
        
        #Route related
        self.status = None 
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [0,0]
        self.path_total = []

        #State related
        self.heading = 0
        self.position = (0,0) #xy position on map

        #Added
        self.vision = []
        self.busyness = []
        self.constraints = []
        self.replan = False
        self.check_gate = False
        self.seperation = False

        #Intersection
        self.intersections = []
        self.intersectionsearch = True
        self.replannode = 0
        self.intersectionpriority = []
        self.replan_inter = False
        self.previous = 0

        self.crossingwait = False
        self.between = False
        self.waiting = False


    def get_heading(self, xy_start, xy_next):
        """
        Determines heading of an aircraft based on a start and end xy position.
        INPUT:
            - xy_start = tuple with (x,y) position of start node
            - xy_next = typle with (x,y) position of next node
        RETURNS:
            - heading = heading of aircraft in degrees
        """

        if xy_start[0] == xy_next[0]: #moving up or down
            if xy_start[1] > xy_next[1]: #moving down
                heading = 180
            elif xy_start[1] < xy_next[1]: #moving up
                heading = 0
            else:
                heading=self.heading

        elif xy_start[1] == xy_next[1]: #moving right or left
            if xy_start[0] > xy_next[0]: #moving left
                heading = 90
            elif xy_start[0] < xy_next[0]: #moving right
                heading = 270
            else:
                heading=self.heading
        else: 
            raise Exception("Invalid movement")
    
        self.heading = heading
      
    def move(self, dt, t):   
        """
        Moves an aircraft between from_node and to_node and checks if to_node or goal is reached.
        INPUT:
            - dt = 
            - t = 
        """
        arrived = False

        #print('id', self.id)
        #print(self.path_to_goal)
        
        #Determine nodes between which the ac is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        #print('fromnode', from_node)
        xy_from = self.nodes_dict[from_node]["xy_pos"] #xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"] #xy position of to node
        distance_to_move = self.speed*dt #distance to move in this timestep
  
        #Update position with rounded values
        x = xy_to[0]-xy_from[0]
        y = xy_to[1]-xy_from[1]

        if x==0 and y==0:
            x_normalized,y_normalized = 0,0
        else:
            x_normalized = x / math.sqrt(x**2+y**2)
            y_normalized = y / math.sqrt(x**2+y**2)

        posx = round(self.position[0] + x_normalized * distance_to_move ,2) #round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move ,2) #round to prevent errors
        self.position = (posx, posy)  
        self.get_heading(xy_from, xy_to)

        #print('pathtogoal',self.path_to_goal)

        #Check if goal is reached or if to_node is reached
        if self.position == xy_to and self.path_to_goal[0][1] == t+dt: #If with this move its current to node is reached
            if self.position == self.nodes_dict[self.goal]["xy_pos"]: #if the final goal is reached
                self.status = "arrived"
                # print('arrived', self.id)
                arrived = True

            else:  #current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]
                
                new_from_id = self.from_to[1] #new from node
                new_next_id = self.path_to_goal[0][0] #new to node

                if new_from_id != self.from_to[0]:
                    self.last_node = self.from_to[0]

                self.previous = self.from_to[0]
                
                self.from_to = [new_from_id, new_next_id] #update new from and to node

                self.replan = True #Needed for weights
                
                #seperation check switch
                
                self.seperation = True

                if len(self.path_to_goal)<5 and self.type == 'A':
                    self.replan = False
                    self.check_gate = True

                if self.replannode == self.from_to[0]:
                    self.replan = False
                    self.replan_inter = True

                if self.nodes_dict[self.from_to[0]]['type'] == 'intersection':
                    self.intersectionsearch = True
                    self.waiting = False

                if self.nodes_dict[self.from_to[0]]['type'] == 'between':
                    self.between = True

                #find upcoming intersections ac will follow
        if self.intersectionsearch:
            intersectlist = []
            #if self.nodes_dict[self.from_to[0]]['type'] == 'intersection':
            intersectlist.append(self.from_to[0])
            #print(self.id, 'intersect')
            for loc in self.path_to_goal:
                if self.nodes_dict[loc[0]]['type'] == 'intersection':
                    intersectlist.append(loc[0])
            self.intersections = list(intersectlist[0:3])
            # print('int', self.id, self.intersections)
            self.intersectionsearch = False

        # print(self.id, self.intersections)

        self.vision = []
        return arrived

    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing aircraft assuming that it knows the entire layout.
        Other traffic is not taken into account.
        INPUT:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
        """
        
        if self.status == "taxiing":
            start_node = self.start #node from which planning should be done
            goal_node = self.goal #node to which planning should be done
            
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t)
            #print('path',path)
            if success:
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                print("Path AC", self.id, ":", path)
            else:
                raise Exception("No solution found for", self.id)
            
            #Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")

    


                
