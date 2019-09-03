import sys
from PIL import Image
import copy
import heapq
import numpy as np
import time

class PriorityQueue(object):
    """docstring for PriorityQueue"""
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)    # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = "" # a string reference to the original import file

'''
These variables determine display coler, and can be changed by you, I guess
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)

'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []       # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}   # a dictionary of (x,y) tuples, representing nodes that have been expanded
openset = []   # a list of (x,y) tuples, representing nodes to expand to in the future


def search(map):
    """
    This function is meant to use the global variables [start, end, path, expanded, frontier] to search through the
    provided map.
    :param map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
    """
    #print map[start[0], start[1]]
    #print map[end[0], end[1]]
    #A_star practice begin
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    start_time = time.time()

    while not frontier.empty():
        current = frontier.get()

        if current == end:
            break

        for next in neighbors(map, current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(end, next)
                frontier.put(next, priority)
                came_from[next] = current

    time_diff = time.time() - start_time
    print "Time taken: " + str(time_diff)
    print "cost: " + str(new_cost)
    #A_star practice end

    

    # put your final path into this array (so visualize_search can draw it in purple)
    path.extend(reconstruct_path(came_from, end, start))
    # put your expanded nodes into this dictionary (so visualize_search can draw them in dark gray)
    expanded.update(came_from)
    # put your frontier nodes into this dictionary (so visualize_search can draw them in light gray)
    while not frontier.empty():
        to_explore = frontier.get()
        openset.extend([to_explore])


    visualize_search("out.png") # see what your search has wrought (and maybe save your results)

def reconstruct_path(came_from, end, start):
    current = end
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    return path

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    

def neighbors(map, current):
    (x, y) = current
    result = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
    result = filter(lambda id: 0 <= id[0] < width and 0 <= id[1] < height, result)
    result = filter(lambda id: map[id[0], id[1]] == 0, result)
    return result


def visualize_search(save_file="do_not_save.png"):
    """
    :param save_file: (optional) filename to save image to (no filename given means no save file)
    """
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()


    # draw expanded pixels
    for pixel in expanded.keys():
        pixel_access[pixel[0], pixel[1]] = DARK_GRAY

    # draw frontier pixels
    for pixel in openset:
        pixel_access[pixel[0], pixel[1]] = LIGHT_GRAY
    
    # draw path pixels
    for pixel in path:
       pixel_access[pixel[0], pixel[1]] = PURPLE

    # draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN

    # display and (maybe) save results
    im.show()
    if(save_file != "do_not_save.png"):
        im.save(save_file)

    im.close()


if __name__ == "__main__":
    # Throw Errors && Such
    # global difficulty, start, end
    assert sys.version_info[0] == 2                             # require python 2 (instead of python 3)
    assert len(sys.argv) == 2, "Incorrect Number of arguments"      # require difficulty input

    # Parse input arguments
    function_name = str(sys.argv[0])
    difficulty = str(sys.argv[1])
    print "running " + function_name + " with " + difficulty + " difficulty."

    # Hard code start and end positions of search for each difficulty level
    if difficulty == "trivial.gif":
        im = Image.open(difficulty)
        print im.getcolors()
        start = (8, 1)
        end = (20, 1)
    elif difficulty == "medium.gif":
        im = Image.open(difficulty)
        start = (8, 201)
        end = (110, 1)
    elif difficulty == "hard.gif":
        im = Image.open(difficulty)
        start = (10, 1)
        end = (400, 400)
    elif difficulty == "very_hard.gif":
        im = Image.open(difficulty)
        start = (1, 324)
        end = (580, 1)
    elif difficulty == "map1.gif":
        im = Image.open(difficulty)
        start = (20, 50)
        end = (600, 150)
    elif difficulty == "map2.gif":
        im = Image.open(difficulty)
        start = (20, 50)
        end = (600, 150)
    elif difficulty == "map3.gif":
        im = Image.open(difficulty)
        start = (20, 200)
        end = (530, 300)
    elif difficulty == "map4.gif":
        im = Image.open(difficulty)
        start = (40, 70)
        end = (400, 200)
    elif difficulty == "map5.gif":
        im = Image.open(difficulty)
        print im.getcolors()  
        im = im.point(lambda i: abs(i-1))    
        print im.mode  
        start = (20, 30)
        end = (360, 360)
    else:
        assert False, "Incorrect difficulty level provided"


    # Perform search on given image
    width, height = im.size
    print width, height
    print "Start: " + str(start)
    print "End: " + str(end)
    search(im.load())
