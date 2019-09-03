#!/usr/bin/env python
import collections
import math
import matplotlib.pyplot as plt
import numpy as np
import time


show_animation = 0

class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, x):
        self.elements.append(x)
    
    def get(self):
        return self.elements.popleft()
# utility functions for dealing with square grids
def from_id_width(id, width):
	return (id % width, id // width)


# different obstacles setup
#DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,233,234,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434,463,464,493,494,523,524,553,554,583,584,613,614]]
#DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [91,92,93,94,111,112,123,124,132,131,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,233,234,235,236,237,238,243,244,253,254,273,274,283,284,303,304,313,314,315,316,333,334,335,336,343,344,373,374,375,376,403,404,433,434,620,621,622,650,651,652,680,681,682,710,711,712]]
DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [610,611,612,613,614,615,616,220,221,222,223,224,225,226,580,550,520,490,460,430,400,370,340,310,280,250,586,556,526,496,466,436,406,376,346,316,286,256]]



def plot_setup(graph, start, goal):
	obs = np.array(g.walls)
	boundary_up = np.array([[i-1, height] for i in range(g.width+2)])
	boundary_left = np.array([[-1, i-1] for i in range(g.height+2)])
	boundary_down = np.array([[i-1, -1] for i in range(g.width+2)])
	boundary_right = np.array([[width, i-1] for i in range(g.height+2)])
	plt.plot(boundary_left[:,0], boundary_left[:,1], ".k")
	plt.plot(boundary_up[:,0], boundary_up[:,1], ".k")
	plt.plot(boundary_down[:,0], boundary_down[:,1], ".k")
	plt.plot(boundary_right[:,0], boundary_right[:,1], ".k")
	plt.plot(start[0], start[1], "xr")
	plt.plot(goal[0], goal[1], "xb")
	plt.plot(obs[:,0], obs[:,1], ".k")
	plt.grid(True)
	plt.axis("equal")


#arena class
class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path

#BFS function
def breadth_first_search(graph, start, goal):
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None
    
    while not frontier.empty():
        current = frontier.get()
        
        if show_animation:
        	plt.plot(current[0], current[1], "xc")
        	plt.pause(0.001)
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current
    
    return came_from


width = 30
height = 30
start = (0, 29)
goal = (29, 0)
g = SquareGrid(width, height)
g.walls = DIAGRAM1_WALLS
if show_animation:
	plot_setup(g, start, goal)

t = time.time()
parents = breadth_first_search(g, start, goal)
optimal_path = reconstruct_path(parents, start, goal)
elapsed = time.time() - t
opt = np.array(optimal_path)
if show_animation:
	plt.plot(start[0], start[1], "xr")
	plt.plot(goal[0], goal[1], "xb")
	plt.plot(opt[:,0], opt[:,1], "-r")
	plt.pause(15)
print ("solution cost(step) is ", opt.shape[0])
print ("running time is ", elapsed)


