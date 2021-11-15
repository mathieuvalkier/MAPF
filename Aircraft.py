from single_agent_planner import simple_single_agent_astar
import math

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
        self.speed = 1                #how much a/c moves per unit of t
        self.id = flight_id           #flight_id
        self.type = a_d               #arrival or departure (A/D)
        self.size = size
        self.spawntime = spawn_time   #spawntime
        self.start = start_node       #start_node_id
        self.goal = goal_node         #goal_node_id
        self.nodes_dict = nodes_dict  #keep copy of nodes dict
        self.edges_dict = None
        
        #Route related
        self.status = None     #aircraft status
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [0,0]   #initial current xy loc, next xy loc
        self.path_total = []   #empty list of tuples with loc and timestep

        #State related
        self.heading = 0      #initial heading 
        self.position = (0,0) #initial xy position on map

        #Added
        self.vision = []         #empty list needed for vision
        self.busyness = []       #empty list needed for NESW busyness
        self.constraints = []    #empty list needed for constratints
        self.replan = False      #replan swith
        self.check_gate = False  #check gates switch
        self.seperation = False  #maintaining separations switch

        self.rwycheck = False       #runway check
        self.rwyblock = [False, 0]  #runway block: list of True/False switch at time t

        #Intersection
        self.intersections = []         #empty list needed for intersections 
        self.intersectionsearch = True  #search for intersection nodes, switch
        self.replannode = 0             #initial location of replan node
        self.intersectionpriority = []  #empty list for priority 
        self.replan_inter = False       #switch when replan node = current (from) node
        self.previous = 0               #initial location of previous node

        self.sideside = [False]     #switch of sideside situation    

        self.crossingwait = False   #switch of crossingwait situation
        self.between = False        #switch of agents between nodes situation
        self.waiting = False        #switch of agents waiting

        self.noreturn = False       #switch of agents cannot return

        self.actualpath = []        #empty list of next (to) node xy location and next time t+dt 


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

        #Set the start location in the path list, needed for analysis
        if len(self.actualpath)<1:
            self.actualpath.append((self.from_to[0],t))

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

        #Prevents error when standing at the same location
        if x==0 and y==0:
            x_normalized,y_normalized = 0,0
        else:
            x_normalized = x / math.sqrt(x**2+y**2)
            y_normalized = y / math.sqrt(x**2+y**2)

        posx = round(self.position[0] + x_normalized * distance_to_move ,2) #round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move ,2) #round to prevent errors
        self.position = (posx, posy)  
        self.get_heading(xy_from, xy_to)

        #Check if goal is reached or if to_node is reached
        if self.position == xy_to and self.path_to_goal[0][1] == t+dt: #If with this move its current to node is reached
            if self.position == self.nodes_dict[self.goal]["xy_pos"]: #If the final goal is reached
                self.actualpath.append((self.from_to[1],t+dt))
                self.status = "arrived"

                if self.type == 'D':           #If an agent departs at runway
                    self.rwyblock = [True, t]  #runway is blocked for other agents at time t

            else:  #current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]
                self.actualpath.append((self.from_to[1],t+dt))
                
                new_from_id = self.from_to[1] #new from node
                new_next_id = self.path_to_goal[0][0] #new to node

                if new_from_id != self.from_to[0]:      #If new from node is not from_node
                    self.last_node = self.from_to[0]    #last node = from_node

                if self.from_to[0] != self.from_to[1]:  #If from_node is not equal to new next node
                    self.previous = self.from_to[0]     #previous = from_node
                
                self.from_to = [new_from_id, new_next_id] #update new from and to node

                self.replan = True #Needed for weights
                
                self.seperation = True #seperation check switch

                if len(self.path_to_goal)<5 and self.type == 'A': #If number of remaining nodes to goal smaller than 5 and goal of agents is gates
                    self.replan = False       
                    self.check_gate = True   #check gates

                if self.replannode == self.from_to[0]: #If replan node = from_node 
                    self.replan_inter = True           #replan crossing collision

                if self.nodes_dict[self.from_to[0]]['type'] == 'intersection':  #If from_node is an intersection
                    self.intersectionsearch = True   #search intersections
                    self.waiting = False             #agents not waiting

                if self.nodes_dict[self.from_to[0]]['type'] == 'between': #IF from_node is a node between intersections
                    self.between = True              #between is True

                #find upcoming intersections ac will follow
        if self.intersectionsearch:
            intersectlist = []      #open intersection list
            if self.nodes_dict[self.from_to[0]]['type'] != 'between': #If from_node is not between intersections
                intersectlist.append(self.from_to[0])                 #append from_node to intersection list
            else:
                intersectlist.append(self.intersections[0])       #Else it is an intersection
            for loc in self.path_to_goal:
                if self.nodes_dict[loc[0]]['type'] != 'between':  #If x location is not between intersections
                    intersectlist.append(loc[0])                  #append x location to intersection list
            self.intersections = list(intersectlist[0:3])         #list of 3 following intersections
            self.intersectionsearch = False

        self.vision = []
        return

    def plan_independent(self, nodes_dict, edges_dict, heuristics, t, print_path=False):
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

            if success:
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                if print_path:
                    print("Path AC", self.id, ":", path)
            else:
                raise Exception("No solution found for", self.id)
            
            #Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")