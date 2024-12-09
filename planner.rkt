#lang dssl2

let eight_principles = ["Know your rights.",
"Acknowledge your sources.",
"Protect your work.",
"Avoid suspicion.",
"Do your own work.",
"Never falsify a record or permit another person to do so.",
"Never fabricate data, citations, or experimental results.",
"Always tell the truth when discussing your work with your instructor."]

# Final project: Trip Planner

import cons
import sbox_hash
import 'project-lib/stack-queue_rkt'
import 'project-lib/graph_rkt'
import 'project-lib/dictionaries_rkt'
import 'project-lib/binheap_rkt'

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = VecKC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = VecKC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = VecKC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?


interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


class TripPlanner (TRIP_PLANNER):
    
    let lst_of_pos         # Queue of Positions
    let dict_of_poi        # Dictionary of POIs
    let map: WUGraph?      # Map Represented in WUGraph
    let pos_to_vertex      # Position Converted to Vertex
    let vertex_to_pos      # Vertex Converted to Position
    let name_to_pos        # Names Converted to Position
    
     
    def __init__(self, vect_road, vect_poi):
        
        # Creates a Dictionary of POIs
        self.dict_of_poi = HashTable(vect_road.len() * 2, make_sbox_hash())
        
        # Creates a Queue of Positions
        self.lst_of_pos = ListQueue()
        
        # Creates Bidirectional Mapping of Positions and Vertices
        self.pos_to_vertex = HashTable(vect_road.len() * 2, make_sbox_hash())
        self.vertex_to_pos = HashTable(vect_road.len() * 2, make_sbox_hash())
        self.name_to_pos = HashTable(vect_poi.len(), make_sbox_hash())
        
        # Updates Dictionary and Queue from vect_road
        let i = 0
        for v in vect_road:

            if not self.dict_of_poi.mem?([v[0],v[1]]):
                # Initializes Dict Keys
                self.dict_of_poi.put([v[0],v[1]],None) 
                # Adds Positions to List of Positions
                self.lst_of_pos.enqueue([v[0],v[1]])
                
                # Updates Bidirectional Mapping of Positions and Vertices
                self.pos_to_vertex.put([v[0],v[1]],i)
                self.vertex_to_pos.put(i,[v[0],v[1]]) 
                
                i = i + 1
                
            if not self.dict_of_poi.mem?([v[2],v[3]]):
                self.dict_of_poi.put([v[2],v[3]],None)
                self.lst_of_pos.enqueue([v[2],v[3]])
               
                self.pos_to_vertex.put([v[2],v[3]],i)
                self.vertex_to_pos.put(i,[v[2],v[3]]) 
                
                i = i + 1
        
        # Creates a Map with Number of Vertices = Number of POIs                   
        self.map = WUGraph(len(self.dict_of_poi))
        
        # Creates Road Networks Between Positions from vect_road
        for v in vect_road:
            let p1 = self.pos_to_vertex.get([v[0],v[1]])
            let p2 = self.pos_to_vertex.get([v[2],v[3]])
            let w = ((v[2] - v[0])**2 + (v[3] - v[1])**2).sqrt()
            self.map.set_edge(p1,p2,w)  
               
        # Updates Dictionary of POIs from vect_pois       
        for w in vect_poi:
            # Checks that Key is Not Empty
            if self.dict_of_poi.get([w[0],w[1]]) != None:
                # Updates List of Values in Key
                self.dict_of_poi.put([w[0],w[1]],cons([w[2],w[3]],self.dict_of_poi.get([w[0],w[1]])))
                # Updates Dict of Names to Pos
                self.name_to_pos.put(w[3],[w[0],w[1]])
            else:
                self.dict_of_poi.put([w[0],w[1]],cons([w[2],w[3]],None))
                self.name_to_pos.put(w[3],[w[0],w[1]])
        

        
    def locate_all(self,dst_cat):
        
        # List For Query Output
        let queries = None
        let copy_lst = ListQueue()
        
        # Checks All Positions
        while not self.lst_of_pos.empty?():
            # Current Position
            let pos = self.lst_of_pos.dequeue()
            # Creates Copy of Position List
            copy_lst.enqueue(pos)
            
            # Checks that POI Contains Something
            if self.dict_of_poi.get(pos) != None:
                # Goes through List of Values in Key
                let curr = self.dict_of_poi.get(pos)
                while curr != None:
                    if curr.data[0] == dst_cat:
                        queries = cons(pos,queries)
                        break
                    curr = curr.next
                    
        # Restores Position List
        self.lst_of_pos = copy_lst        
        return queries
                
        
    def plan_route(self,src_lat,src_lon,dst_name):
        
        # Checks that Name is in Dictionary
        if not self.name_to_pos.mem?(dst_name):
            return None
        
        # Desitnation Position from Name
        let target_pos = self.name_to_pos.get(dst_name) 
        # Desitnation Vertex from Position
        let target_vert = self.pos_to_vertex.get(target_pos)
        
        # Starting Position
        let pos = [src_lat,src_lon]
        # Starting Vertex
        let start_vert = self.pos_to_vertex.get(pos)
        
        # Creates Travel Paths from Start Point using Dijkstra's Algorithm
        let travel_paths = self.dijkstras_alg(self.map,start_vert)
        
        # Starting Path Point
        let path_point = travel_paths[target_vert]
        
        # Creates Plan Route
        let route = cons(self.vertex_to_pos.get(target_vert),None)

        # Loops Until Starting Position is Reached
        while path_point[1] != None:
            
            # Updates Route with Next Position
            route = cons(self.vertex_to_pos.get(path_point[1]),route)
            
            # Moves to Next Position
            path_point = travel_paths[path_point[1]]
        
            
        if route.data == pos:   
            return route         
        return None
        
        
        
    def find_nearby(self,src_lat,src_lon,dst_cat,n):
        # Starting Position
        let pos = [src_lat,src_lon]
        # Starting Vertex
        let start_vert = self.pos_to_vertex.get(pos)
        
        # Creates Travel Paths from Start Point using Dijkstra's Algorithm
        let travel_paths = self.dijkstras_alg(self.map,start_vert)
        
        # Initializes Vector for Nearby POIs     
        let nearby = None 
        
        # Creates an Empty Priority Queue to Sort POIs depending on Distance
        let sorted_POIs = BinHeap[AnyC](len(travel_paths), λ x, y: x[0][0] < y[0][0])
        
        # Inserts a Path and Index
        for i in range(len(travel_paths)):
            sorted_POIs.insert([travel_paths[i],i])  
    
        # Initializes Iterator and Current Vertex
        let i = 0
        let curr_vertex = None
        
        # Loops Until Iterator equals n
        while i < n: 
            
            # Makes Sure Priority Queue isn't Empty 
            if sorted_POIs.len() != 0:
                if sorted_POIs.find_min()[0][0] == inf:
                    sorted_POIs.remove_min()
                    i = i + 1
                else:
                    curr_vertex = sorted_POIs.find_min()[1]
                    sorted_POIs.remove_min()
            else:
                break
            
            # Get Current Position    
            let curr_pos = self.vertex_to_pos.get(curr_vertex)
            
            # Gets Current POI
            let curr = self.dict_of_poi.get(curr_pos) 
            
            # Loops Through Each POI in Position       
            while curr != None:
                if curr.data[0] == dst_cat and i < n:
                    i = i + 1
                    nearby = cons([curr_pos[0],curr_pos[1],curr.data[0],curr.data[1]],nearby)
                curr = curr.next 
                                   
        return nearby    
 
    # From Lecture 11 Pages 16 and 17
    def dijkstras_alg(self,graph,start):
        
        # Determines the Number of Vertices in Graph
        let n = graph.n_vertices()
        
        # Creates a Table for Tracking Paths
        let table = vec(n,lambda i: [inf,None])
        
        # Initializes Starting Vertex Distance to 0
        table[start][0] = 0
        
        # Creates Empty Priority Queue
        let todo = BinHeap[nat?](n, λ x, y: x < y)
        
        # Creates False Vector of Completed Vertices
        let done = vec(n,lambda i: False)
        
        # Inserts Starting Vertex to Priority Queue
        todo.insert(start)
        
        # Loops Until Priority Queue is Empty
        while todo.len() != 0:
            
            # Pulls Highest Priority Queue Vertex
            let v = todo.find_min()
            # Removes Highest Priority Queue Vertex
            todo.remove_min()
            
            # If Shortest Path D.N.E Find Shortest Path
            if not done[v]:
                
                # Complete Vertex
                done[v] = True
                
                # Gets All Adjacent Vertices to Current Vertex
                let e = graph.get_adjacent(v)
                
                # Operates Until All Vertices are Checked
                while e != None:
                    
                    # Checks if Updated Distance is Less than Current Distance  
                    if (table[v][0] + graph.get_edge(v,e.data)) < table[e.data][0]:
                        
                        # Updates Distances to Vertices
                        table[e.data][0] = table[v][0] + graph.get_edge(v,e.data)
                        # Updates Predecessors
                        table[e.data][1] = v
                        # Adds Vertices to Priority Queue
                        todo.insert(e.data)
                    e = e.next
                    
        return table
  
             
def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "Reggie's"],
                        [0,1, "food", "Dumplings"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)
        
    assert my_first_example().locate_all("bar") == \
        cons([0,0], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Dumplings") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Dumplings"], None)
 
        
               
def my_second_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0],[1,0, 1,2],[1,0, 1,3]],
    
                       [[0,0, "bar", "Tavern 1"],
                        [0,0, "food", "Little Caesar's"],
                        [0,1, "food", "Bob's Burgers"],
                        [0,1, "bar", "Reggie's"],
                        [1,0, "bar", "Tavern 2"],
                        [1,3, "food", "Food 1"],
                        [1,3, "bar", "Tavern 3"],
                        [1,2, "food", "Dumplings"]])
                        
                        
test 'My second locate_all test':
    assert my_second_example().locate_all("food") == \
        cons([1,3],cons([1,2],cons([0,1],cons([0,0], None))))
        
    assert my_second_example().locate_all("bar") == \
        cons([1,3],cons([1,0],cons([0,1],cons([0,0], None))))
        
        
def example_map():
    return TripPlanner([[0,0,1,0],[0,0,0,1],[0,1,1,1],[0,1,0,2],[1,0,1,1],[1,1,1,2],[1,2,0,2],[1,2,1,3],[1,3,-0.2,3.3]],
                       [[0,0,"food","Sandwiches"],
                        [0,1,"food","Pasta"],
                        [0,1,"clothes","Pants"],
                        [1,1,"bank","Local Credit Union"],
                        [1,3,"bar","Bar None"],
                        [1,3,"bar","H Bar"],
                        [-0.2,3.3,"food","Burritos"]])
                        
test 'example_map locate_all test':
    assert example_map().locate_all("food") == \
        cons([-0.2,3.3],cons([0,1],cons([0,0], None)))
        
    assert example_map().locate_all("bar") == \
        cons([1,3], None)
        
    assert example_map().locate_all("bank") == \
        cons([1,1], None)
        
    assert example_map().locate_all("clothes") == \
        cons([0,1], None)
        
    assert example_map().locate_all("barber") == \
        None
   
test 'example_map find_nearby test':
    assert example_map().find_nearby(1, 3, "food", 1) == \
        cons([-0.2,3.3, "food", "Burritos"], None)
        
    assert example_map().find_nearby(0, 2, "food", 1) == \
        cons([0,1, "food", "Pasta"], None)
        
    assert example_map().find_nearby(0, 2, "food", 2) == \
        cons([0,0, "food", "Sandwiches"],cons([0,1, "food", "Pasta"], None))
        
    assert example_map().find_nearby(0, 2, "food", 3) == \
        cons([-0.2,3.3,"food","Burritos"],cons([0,0, "food", "Sandwiches"],cons([0,1, "food", "Pasta"], None)))
   
    assert example_map().find_nearby(0, 2, "food", 4) == \
        cons([-0.2,3.3,"food","Burritos"],cons([0,0, "food", "Sandwiches"],cons([0,1, "food", "Pasta"], None)))    
        
    assert example_map().find_nearby(0, 2, "bar", 1) == \
        cons([1,3,"bar", "H Bar"], None)  
        
    assert example_map().find_nearby(0, 2, "bar", 2) == \
        cons([1,3,"bar", "Bar None"],cons([1,3,"bar", "H Bar"], None))
        
    assert example_map().find_nearby(0, 2, "bar", 3) == \
        cons([1,3,"bar", "Bar None"],cons([1,3,"bar", "H Bar"], None))   
         
    assert example_map().find_nearby(0, 2, "school", 5) == \
        None      
        
test 'example_map plan_route test':
    assert example_map().plan_route(0, 0, "Sandwiches") == \
       cons([0,0], None)
       
    assert example_map().plan_route(0, 1, "Sandwiches") == \
       cons([0,1],cons([0,0], None))
       
    assert example_map().plan_route(1, 1, "Sandwiches") == \
       cons([1,1],cons([0,1],cons([0,0], None))) or cons([1,1],cons([1,0],cons([0,0], None)))
       
    assert example_map().plan_route(1, 1, "Burritos") == \
       cons([1,1],cons([1,2],cons([1,3],cons([-0.2,3.3], None))))
       
    assert example_map().plan_route(1, 1, "Sushi") == \
       None
       
       
def disjoint_map():
    return TripPlanner([[0,0,1.5,0],[1.5,0,2.5,0],[2.5,0,3,0],[4,0,5,0]],
                       [[1.5,0,'bank','Union'],
                        [3,0,'barber','Tony'],
                        [5,0,'barber','Judy']])
                           
test 'plan_route test 1':
    assert disjoint_map().plan_route(0, 0, "Judy") == \
       None
       
def tp1():
    return TripPlanner([[0, 214.0, 33.5, 211.0],
       [0, 214.0, 66.5, 203.0],
       [0, 214.0, 98.0, 190.0],
       [0, 214.0, 127.0, 172.0],
       [0, 214.0, 152.5, 149.5]],
       [[0, 214.0, '"coffee"', 'Starbucks #1'],
       [33.5, 211.0, '"coffee"', 'Starbucks #2'],
       [66.5, 203.0, '"coffee"', 'Starbucks #3'],
       [98.0, 190.0, '"coffee"', 'Starbucks #4']])
       
test 'find nearest florist':
    assert tp1().find_nearby(0, 214.0, '"coffee"', 1) == cons([0, 214.0, '"coffee"', 'Starbucks #1'],None)
    
def tp2():
    return TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])

test 'already at barber':
    assert tp2().find_nearby(3, 0, 'barber', 1) == cons([3, 0, 'barber', 'Tony'],None)     
    
def tp3():
    return TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
       
test '2 relevant POIs; 1 reachable':
    assert tp3().find_nearby(0, 0, 'barber', 2) == cons([3, 0, 'barber', 'Tony'],None)
    
def tp4():
    return TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])

test "Relevant POI isn't reachable":
    assert tp4().find_nearby(0, 0, 'food', 1) == None         