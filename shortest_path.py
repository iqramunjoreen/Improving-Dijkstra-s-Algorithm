import random
import math
import re
import numpy as np
import heapq

#############################################
## CALCULATE DISTANCE BETWEEN TWO VERTICES ##
#############################################

def distance(v1,v2):
    dlat = 2 * math.pi * (v2.x - v1.x) / 360
    mlat = 2 * math.pi * (v1.x + v2.x) / 2 / 360
    dlon = 2 * math.pi * (v2.y - v1.y) / 360
    return 6371009 * (dlat ** 2 + (math.cos(mlat) * dlon) ** 2) ** 0.5

##################
## VERTEX CLASS ##
##################

class Vertex:
    def __init__ (self, key):
        self.key = key
        self.x = 0.0
        self.y = 0.0
        self.parent = Vertex
        self.distance = float('inf')
        self.heuristic = float('inf')
        self.total = float('inf')
        self.adjacencies = []
        self.visited = 0
        self.node_counter = 0
        self.landmark1 = 0.0
        self.landmark2 = 0.0
        self.landmark3 = 0.0
        self.landmark4 = 0.0

    def calc_total(self):
        self.total = self.distance + self.heuristic

##################
## QUEUE CLASS ##
##################

class PQ:
    def __init__(self):
        self.queue = []

    # for checking if the queue is empty
    def isEmpty(self):
        return len(self.queue) == 0

    # for inserting an element in the queue
    def insert(self, vertex):
        self.queue.append(vertex)

    # for popping an element based on distance from source
    def pop_dijkstra(self):
        try:
            min = 0
            for i in range(len(self.queue)):
                if self.queue[i].distance < self.queue[min].distance:
                    min = i
            item = self.queue[min]
            del self.queue[min]
            return item
        except IndexError:
            print()
            exit()

    # for popping an element based on distance from source and distance to destination
    def pop_astar(self):
        try:
            min = 0
            for i in range(len(self.queue)):
                if self.queue[i].total< self.queue[min].total:
                    min = i
            item = self.queue[min]
            del self.queue[min]
            return item
        except IndexError:
            print()
            exit()

    def exists(self, key):
        for x in range(len(self.queue)):
            if self.queue[x].key == key:
                return True
        return False

    def delete(self, key):
        for x in range(len(self.queue)):
            if self.queue[x].key == key:
                del self.queue[x]
                return True
        return False

    def print_queue(self):
        for x in range(len(self.queue)):
            print(self.queue[x].key)

################
## DIJKSTRA'S ##
################

def dijkstra(v1, v2):
    global q
    global node_counter
    if v1.distance == float('inf'):
        v1.distance = 0 #this is the source
        v1.node_counter = 0
    if v1.visited == 0:
        v1.visited = 1
    if v1.key == v2.key:
        return v2
    # find adjacencies and update
    len1 = len(v1.adjacencies) #how many adjacencies does v1 have?
    #update all distances
    for x in range(len1):
        if v1.adjacencies[x].visited == True:
            continue
        # is this vertex already in the stack?
        if q.exists(v1.adjacencies[x].key) == True:
            q.delete(v1.adjacencies[x].key)
        #need to find distance between source and adj
        if v1.adjacencies[x].distance > distance(v1, v1.adjacencies[x]) + v1.distance:
            v1.adjacencies[x].distance = distance(v1, v1.adjacencies[x]) + v1.distance
        v1.adjacencies[x].parent = v1
        q.insert(v1.adjacencies[x])
        # pop vertex with smallest distance
    v = q.pop_dijkstra()
    v.node_counter = v1.node_counter + 1
    dijkstra(v, v2)

def reset():
    for x in range(1,1001):
        vertices[x].parent = Vertex
        vertices[x].distance = float('inf')
        vertices[x].heuristic = float('inf')
        vertices[x].total = float('inf')
        vertices[x].visited = 0
        vertices[x].node_counter = 0

###################
## PRINT RESULTS ##
###################

def print_results(v1,v2,d,f):
    print(" ", v1.key,"==>",v2.key,
    "     distance: ",v2.distance,"     nodes visited by Dijkstra: ",
    d,"     nodes visited by a* search: ",v2.node_counter,
    "     nodes visited by landmark search: ",f," ")

###############
## A* SEARCH ##
###############

def a_star(v1, v2):
    global q
    global node_counter
    if v1.distance == float('inf'):
        v1.distance = 0 #this is the source
        v1.node_counter = 0
    if v1.visited == 0:
        v1.visited = 1
    if v1.key == v2.key:
        return v2
    # find adjacencies and update
    len1 = len(v1.adjacencies) #how many adjacencies does v1 have?
    #update all distances
    for x in range(len1):
        if v1.adjacencies[x].visited == True:
            continue
        # is this vertex already in the stack?
        if q.exists(v1.adjacencies[x].key) == True:
            q.delete(v1.adjacencies[x].key)
        # total = distance + heuristic
        if v1.adjacencies[x].distance > distance(v1, v1.adjacencies[x]) + v1.distance:
            v1.adjacencies[x].distance = distance(v1, v1.adjacencies[x]) + v1.distance
        v1.adjacencies[x].heuristic = distance(v1.adjacencies[x],v2)
        v1.adjacencies[x].total = v1.adjacencies[x].distance + v1.adjacencies[x].heuristic
        v1.adjacencies[x].parent = v1
        q.insert(v1.adjacencies[x])
        # pop vertex with smallest distance
    v = q.pop_astar()
    v.node_counter = v1.node_counter + 1
    a_star(v, v2)

###############
## LANDMARK ##
###############

def landmark(v1, v2):
    global q
    global node_counter
    if v1.distance == float('inf'):
        v1.distance = 0 #this is the source
        v1.node_counter = 0
    if v1.visited == 0:
        v1.visited = 1
    if v1.key == v2.key:
        return v2
    # find adjacencies and update
    len1 = len(v1.adjacencies) #how many adjacencies does v1 have?
    #update all distances
    for x in range(len1):
        if v1.adjacencies[x].visited == True:
            continue
        # is this vertex already in the stack?
        if q.exists(v1.adjacencies[x].key) == True:
            q.delete(v1.adjacencies[x].key)
        # total = distance + heuristic
        if v1.adjacencies[x].distance > distance(v1, v1.adjacencies[x]) + v1.distance:
            v1.adjacencies[x].distance = distance(v1, v1.adjacencies[x]) + v1.distance
        v1.adjacencies[x].heuristic = max(v1.landmark1-v2.landmark1,v1.landmark2-v2.landmark2,v1.landmark3-v2.landmark3,v1.landmark4-v2.landmark4)
        v1.adjacencies[x].total = v1.adjacencies[x].distance + v1.adjacencies[x].heuristic
        v1.adjacencies[x].parent = v1
        q.insert(v1.adjacencies[x])
        # pop vertex with smallest distance
    v = q.pop_astar()
    v.node_counter = v1.node_counter + 1
    landmark(v, v2)

#main

#read file
f = open("graph1000.txt","r")

#array of 1001 vertices
vertices = [Vertex(0) for x in range(1001)]
#put in the coordinates
for x in range(1,1001):
    line_str = f.readline()
    word_in_line = re.split(',| |\n',line_str)
    vertices[x].x = float(word_in_line[2])
    vertices[x].y = float(word_in_line[4])
    vertices[x].key = x
    vertices[x].landmark1 = distance(vertices[x],vertices[188])
    vertices[x].landmark2 = distance(vertices[x],vertices[464])
    vertices[x].landmark3 = distance(vertices[x],vertices[88])
    vertices[x].landmark4 = distance(vertices[x],vertices[716])

#empty line
f.readline()

# add in the adjacencies
for x in range(1,1001):
    line_str = f.readline()
    word_in_line = re.split(':|,\n',line_str)
    points_str = word_in_line[1]
    a = re.split(',|\n',points_str)
    for y in range(len(a)):
        vertices[x].adjacencies.append(vertices[int(a[y])])


#priority queue
q = PQ();

node_d = [0 for x in range(20)]
node_a = [0 for x in range(20)]
node_l = [0 for x in range(20)]
average = 0.000

#query pairs of 20 random vertices
for x in range(20):
    a = random.randint(1,1000)
    b = random.randint(1,1000)
    z = dijkstra(vertices[a],vertices[b])
    d = vertices[b].node_counter
    node_d[x] = vertices[b].node_counter
    reset()
    z = a_star(vertices[a],vertices[b])
    f = vertices[b].node_counter
    node_a[x] = vertices[b].node_counter
    reset()
    z = landmark(vertices[a],vertices[b])
    node_l[x] = vertices[b].node_counter
    print_results(vertices[a], vertices[b], d, f)
    reset()

for x in range(20):
    temp = float(node_a[x])/float(node_d[x])
    average += temp
average = average/20
average = average*100
print("")
print("On average, A* search only visited ",average,"% of the nodes visited by Dijkstra's algorithm.")

average = 0.000

for x in range(20):
    temp = float(node_l[x])/float(node_d[x])
    print(temp)
    average += temp
    print(average)
average = average/20
print(average)
average = average*10
print("")
print("On average, Landmark search only visited ",average,"% of the nodes visited by Dijkstra's algorithm.")


#calculate average
#print average
