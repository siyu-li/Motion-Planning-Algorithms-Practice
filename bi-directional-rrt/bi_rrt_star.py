import math
import random
import matplotlib.pyplot as plt
# obstacle set up
OBS=[(4,4,2,2),(2,2,1,1)]


# connect range
RAN_CON = 1
# environment
WIDTH = 10
HEIGHT = 10
RADIUS = 0.5
# new node step size
EPSILON = 0.1


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def set_parent(self, parent_node):
        self.parent_node = parent_node

    def set_cost(self, cost):
        self.cost = cost

    def get_parent(self):
        return self.parent_node

    def get_cost(self):
        return self.cost


def distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

def non_obstacle(nodeA,nodeB,OBS=OBS):
    A=(nodeA.x,nodeA.y)
    B=(nodeB.x,nodeB.y)
    for o in OBS:
      obs=(o[0],o[1],o[0]+o[2],o[1]+o[3])
      C1=(obs[0],obs[1])
      D1=(obs[0],obs[3])
      C2=(obs[0],obs[1])
      D2=(obs[2],obs[1])
      C3=(obs[2],obs[3])
      D3=(obs[2],obs[1])
      C4=(obs[2],obs[3])
      D4=(obs[0],obs[3])
      inst1= ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1)
      inst2= ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2)
      inst3= ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3)
      inst4= ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4)
      if inst1==False and inst2==False and inst3==False and inst4==False:
        continue
      else:
         return False
    return True


def get_nearest_node(latest_node, tree):
    temp_cost = 9999
    nearest_node = tree[0]
    for p_node in tree:
        if distance(latest_node, p_node) < RADIUS and p_node.get_cost() < temp_cost:
            temp_cost = p_node.get_cost()
            nearest_node = p_node
    return nearest_node


def get_path(tree, node):
    path=[]
    while node != tree[0]:
        path.append(node)
        node = node.get_parent()
    path.append(tree[0])
    return reverse_path(path)
    #return path


def reverse_path(path):
    return path[::-1]




def get_new_node(rand_node, tree_node):
    theta = math.atan2(rand_node.y - tree_node.y, rand_node.x - tree_node.x)
    x = tree_node.x + EPSILON * math.cos(theta)
    y = tree_node.y + EPSILON * math.sin(theta)
    return Node(x, y)


def update_tree(tree):
    rand_node = Node(random.random() * WIDTH, random.random() * HEIGHT)
    #temp_node = copy.deepcopy(tree[0])
    temp_node = tree[0]
    for p_node in tree:
        if distance(p_node, rand_node) < distance(temp_node, rand_node):
            temp_node = p_node
    new_node = get_new_node(rand_node, temp_node)

    # find parent
    if non_obstacle(new_node, temp_node):
        temp_parent_node = temp_node
        temp_parent_cost = distance(new_node, temp_node) + temp_node.get_cost()
        for p_node in tree:
            if non_obstacle(new_node, p_node) and distance(new_node, p_node) < RADIUS and (
                    distance(new_node, p_node) + p_node.get_cost() < temp_parent_cost):
                temp_parent_node = p_node
                temp_parent_cost = distance(new_node, p_node) + p_node.get_cost()
        new_node.set_parent(temp_parent_node)
        new_node.set_cost(temp_parent_cost)
    else:
        return tree

    tree.append(new_node)

    # rewire tree
    for p_node in tree:
        #if distance(p_node, new_node) < RADIUS and non_obstacle(new_node, p_node) and p_node != new_node.get_parent() and distance(new_node, p_node) + new_node.get_cost() < p_node.get_cost():
        if distance(p_node, new_node) < RADIUS and non_obstacle(new_node, p_node):
            if p_node != new_node.get_parent() and distance(new_node, p_node) + new_node.get_cost() < p_node.get_cost():
                p_node.set_parent(new_node)
                p_node.set_cost(distance(new_node, p_node) + new_node.get_cost())

    return tree


def RRT_star(Iter, start_node, goal_node):
    # Initialize two tree
    start_tree = []
    goal_tree = []
    start_tree.append(start_node)
    goal_tree.append(goal_node)
    final_path = []

    for i in range(Iter):
        if i % 2:
            start_tree = update_tree(start_tree)
            latest_node = start_tree[-1]
            nearest_node = get_nearest_node(latest_node, goal_tree)

        else:
            goal_tree = update_tree(goal_tree)
            latest_node = goal_tree[-1]
            nearest_node = get_nearest_node(latest_node, start_tree)


        if distance(latest_node, nearest_node) < RAN_CON and non_obstacle(latest_node, nearest_node):
            ##path found, get path and return
            if i % 2:
                start_path = get_path(start_tree, latest_node)
                goal_path = get_path(goal_tree, nearest_node)
            else:
                start_path = get_path(start_tree, nearest_node)
                goal_path = get_path(goal_tree, latest_node)

            goal_path = reverse_path_parent(goal_path)
            # connect two path
            goal_path[0].set_parent(start_path[-1])
            final_path.extend(start_path)
            final_path.extend(goal_path)
            break

    return final_path, start_tree, goal_tree

def reverse_path_parent(goal_path):
    for i in range(len(goal_path)-1):
        goal_path[i].set_parent(goal_path[i+1])
    goal_path[-1].set_parent(None)
    return  reverse_path(goal_path)

def draw_obs(obstacle_list):
    for obstacle in obstacle_list:
        x = [obstacle[0], obstacle[0], obstacle[0]+obstacle[2], obstacle[0]+obstacle[2], obstacle[0]]
        y = [obstacle[1], obstacle[1]+obstacle[3], obstacle[1]+obstacle[3], obstacle[1],  obstacle[1]]
        plt.plot(x, y, color='black')



def draw_figure(final_path, start_tree, goal_tree, start_node, goal_node):
    for i in range(len(start_tree)):
        if i != 0:
            plt.plot([start_tree[i].x, start_tree[i].get_parent().x],[start_tree[i].y, start_tree[i].get_parent().y], color='blue', marker='o', markersize = 1)

    for i in range(len(goal_tree)):
        if i != 0:
            plt.plot([goal_tree[i].x, goal_tree[i].get_parent().x], [goal_tree[i].y, goal_tree[i].get_parent().y], color='orange', marker='o', markersize=1)

    for p in final_path:
        if p != final_path[0]:
            plt.plot([p.x, p.get_parent().x], [p.y, p.get_parent().y], color='red')

    plt.plot(start_node.x, start_node.y, color='green', marker='o')
    plt.plot(goal_node.x, goal_node.y, color='green', marker='o')
    plt.show()


def main():
    # update tree
    # goal,start
    start_node = Node(1, 1)
    goal_node = Node(9, 9)
    start_node.set_cost(0)
    goal_node.set_cost(0)
    #plot
    plt.figure()
    plt.axis([0,WIDTH,0,HEIGHT])
    draw_obs(OBS)
    Iter = 5000
    final_path, start_tree, goal_tree = RRT_star(Iter, start_node, goal_node)
    #plot path
    draw_figure(final_path, start_tree, goal_tree, start_node, goal_node)



if __name__ == '__main__':
    main()

