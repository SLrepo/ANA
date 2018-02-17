from __future__ import division  # need this for the right division
from PIL import Image
import heapq
import numpy as np
import sys
import time


'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)    # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = ""  # a string reference to the original import file

'''
These variables determine display color, and can be changed by you, I guess
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (100, 100, 100)
DARK_GRAY = (50, 50, 50)
RED = (255, 0, 0)
'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []       # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}   # a dictionary of (x,y) tuples, representing nodes that have been expanded
frontier = {}   # a dictionary of (x,y) tuples, representing all the nodes in the open list.
path_optimal = []


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0  # this is the cost to go for this node.
        self.pre = None
        self.h = 0
        self.e = 1
        self.cost = self.g + self.e * self.h

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.cost == other.cost

    def update(self, c):
        self.cost = c

    def setup_parent(self, pre_node):
        self.pre = pre_node


class NodeAna:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0  # this is the cost to go for this node.
        self.pre = None
        self.h = 0
        self.e = 1000000000

    def __lt__(self, other):
        return self.e > other.e

    def __cmp__(self, other):
        return self.e > other.e

    def __eq__(self, other):
        return self.e == other.e


def search(road_map, algo):
    """
    This function is meant to use the global variables [start, end, path, expanded, frontier] to search through the
    provided map.
    :param road_map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
    :param algo: The specific algorithm used.
    """

    # O is unoccupied (white); 1 is occupied (black)
    print("pixel value at start point ", road_map[start[0], start[1]])
    print("pixel value at end point ", road_map[end[0], end[1]])

    # put your final path into this array (so visualize_search can draw it in purple)
    # path.extend([(8, 2), (8, 3), (8, 4), (8, 5), (8, 6), (8, 7)])

    # put your expanded nodes into this dictionary (so visualize_search can draw them in dark gray)
    # expanded.update({(7, 2): True, (7, 3): True, (7, 4): True, (7, 5): True, (7, 6): True, (7, 7): True})

    # put your frontier nodes into this dictionary (so visualize_search can draw them in light gray)
    # frontier.update({(6, 2): True, (6, 3): True, (6, 4): True, (6, 5): True, (6, 6): True, (6, 7): True})
    if algo == "A_star":
        a_search(road_map)
    elif algo == "Dijkstras":
        d_search(road_map)
    elif algo == "ARA":
        ara_search(road_map)
    elif algo == "ANA":
        ana_search(road_map)
    else:
        assert False, "Algorithm is not supported by this program."

    visualize_search("out.gif")  # see what your search has wrought (and maybe save your results)


def h_estimator(x, y):
    h = abs(x - end[0]) + abs(y - end[1])
    if x == end[0] and y == end[1]:  # to avoid 0 h(s) for e calculation
        h = 0.01
    return h


def a_search(road_map):  # this is the A* search algorithm, calculate heuristic function using steps.
    # create an open list, make it a priority queue. put the start point into the queue.
    # im = Image.open(difficulty)
    ol = []
    heapq.heapify(ol)
    s = Node(start[0], start[1])  # start node has a g cost of 0
    s.g = 0
    s.cost = s.g + h_estimator(s.x, s.y)
    heapq.heappush(ol, s)
    frontier[(start[0], start[1])] = True
    # create a hash table to check if the node is already in the open list. put the start point into the check_exist.
    check_exist = {(s.x, s.y): s}

    steps = [[0, -1], [1, 0], [0, 1], [-1, 0]]
    current = s
    # start the loop
    while ol:  # empty list is false
        # pop the top node from priority queue
        current = heapq.heappop(ol)
        # mark the current node as the frontier.
        if current.x == end[0] and current.y == end[1]:  # check if goal has been reached.
            break
        # search on all four direction of the current node.
        for i in range(0, 4):
            new_x = current.x + steps[i][0]
            new_y = current.y + steps[i][1]
            if check_limit(im, new_x, new_y):  # check if the new pixel is in the image or not.
                # check if it is in a wall. give a great cost to it if it is in a wall.
                step_cost = get_cost(road_map, new_x, new_y)
                new_g = current.g + step_cost
                new_cost = new_g + h_estimator(new_x, new_y)
                # check if it is already in the open list heap.
                if (new_x, new_y) in check_exist:  # if in the list, update the cost if the new cost is smaller.
                    next_node = check_exist[(new_x, new_y)]
                    if new_cost < check_exist[(new_x, new_y)].cost:
                        next_node.g = new_g
                        next_node.cost = new_cost
                        next_node.pre = current
                        heapq.heapify(ol)
                else:  # if not in the list, then add the new node to the open list.
                    new_node = Node(new_x, new_y)
                    new_node.cost = new_cost
                    new_node.g = new_g
                    new_node.h = h_estimator(new_x, new_y)
                    new_node.pre = current
                    heapq.heappush(ol, new_node)
                    check_exist[(new_x, new_y)] = new_node
                    frontier[(new_x, new_y)] = True  # frontier stores every node that existed in the open list.
        # put the current node into the expanded list.
        expanded[(current.x, current.y)] = True

    # trace back to the goal to generate the path.
    cur = current.pre
    roots = []
    while cur.x != s.x or cur.y != s.y:
        roots = [[cur.x, cur.y]] + roots
        cur = cur.pre
    # assign the roots list to path.
    path.extend(roots)


def ara_search(road_map):  # this is the ARA* search algorithm, calculate heuristic function using steps.
    # create an open list, make it a priority queue. put the start point into the queue.
    # im = Image.open(difficulty)
    ol = []
    heapq.heapify(ol)
    delta_e = 1  # set initial delta e value
    e = 10  # set initial e value.
    s = Node(start[0], start[1])  # start node has a g cost of 0
    s.g = 0
    s.cost = s.g + e * h_estimator(s.x, s.y)
    heapq.heappush(ol, s)
    frontier[(start[0], start[1])] = True
    # create a hash table to check if the node is already in the open list. put the start point into the check_exist.
    check_exist = {(s.x, s.y): s}

    # set up parameters.
    steps = [[0, -1], [1, 0], [0, 1], [-1, 0]]
    big_g = np.inf  # G is the current optimal solution cost.
    # start the loop
    while ol:  # empty list is false
        # improve solution part.
        print('start to improve the next round...')
        while ol and ol[0].cost <= big_g:
            current = heapq.heappop(ol)
            if current.x == end[0] and current.y == end[1]:  # check if goal has been reached.
                big_g = current.g
                print("found a path")
                # add the found path to path.
                cur = check_exist[(end[0], end[1])].pre
                while cur.x != s.x or cur.y != s.y:
                    path.append((cur.x, cur.y))
                    cur = cur.pre

                break
            for i in range(0, 4):
                new_x = current.x + steps[i][0]
                new_y = current.y + steps[i][1]
                if check_limit(im, new_x, new_y):
                    step_cost = get_cost(road_map, new_x, new_y)
                    new_g = current.g + step_cost
                    if (new_x, new_y) in check_exist:  # if in the list, update the cost if the new cost is smaller.
                        next_node = check_exist[(new_x, new_y)]
                        if new_g < next_node.g:
                            next_node.g = new_g
                            next_node.pre = current
                            if next_node.g + next_node.h < big_g:
                                next_node.cost = new_g + e * next_node.h
                                if next_node in ol:
                                    heapq.heapify(ol)
                                else:
                                    heapq.heappush(ol, next_node)
                    else:
                        new_node = Node(new_x, new_y)
                        new_node.g = new_g
                        new_node.h = h_estimator(new_x, new_y)
                        new_node.cost = new_node.g + e * new_node.h
                        new_node.pre = current
                        if new_node.g + new_node.h < big_g:
                            heapq.heappush(ol, new_node)
                        check_exist[(new_x, new_y)] = new_node
                        frontier[(new_x, new_y)] = True
            expanded[(current.x, current.y)] = True
        print('the improvement round ended.')
        # update e value
        if e <= 0:
            break
        else:
            e = e - delta_e
        # update open list.
        temp_list = []
        for i in range(0, len(ol)):
            ol[i].cost = ol[i].g + e * ol[i].h
            if ol[i].g + ol[i].h < big_g:
                temp_list = [ol[i]] + temp_list
        heapq.heapify(temp_list)
        ol = temp_list

    print('open list is empty, nothing to explore. ')
    # trace back to the goal to generate the optimal path.
    cur = check_exist[(end[0], end[1])].pre
    while cur.x != s.x or cur.y != s.y:
        path_optimal.append((cur.x, cur.y))
        cur = cur.pre


def ana_search(road_map):  # this is the ANA* search algorithm, calculate heuristic function using steps.
    # create an open list, make it a priority queue. put the start point into the queue.
    # im = Image.open(difficulty)
    ol = []
    heapq.heapify(ol)
    big_e = np.inf  # this is the smallest e value.
    s = NodeAna(start[0], start[1])
    s.g = 0  # start node has a g cost of 0
    heapq.heappush(ol, s)
    frontier[(start[0], start[1])] = True
    # create a hash table to check if the node is seen before.
    check_exist = {(s.x, s.y): s}
    # create a hash table to check if the node is in the open list.
    steps = [[0, -1], [1, 0], [0, 1], [-1, 0]]
    big_g = np.inf  # G is the current optimal solution cost.
    # start the loop
    while ol:  # empty list is false
        # improve solution part.
        print('start to improve the next round...')
        while ol:
            current = heapq.heappop(ol)
            if current.e < big_e:
                big_e = current.e
            if current.x == end[0] and current.y == end[1]:  # check if goal has been reached.
                big_g = current.g
                print("found a path")
                # add the found path to path.
                cur = check_exist[(end[0], end[1])].pre
                while cur.x != s.x or cur.y != s.y:
                    path.append((cur.x, cur.y))
                    cur = cur.pre
                break
            for i in xrange(0, 4):
                new_x = current.x + steps[i][0]
                new_y = current.y + steps[i][1]
                if check_limit(im, new_x, new_y):
                    step_cost = get_cost(road_map, new_x, new_y)
                    new_g = current.g + step_cost
                    if (new_x, new_y) in check_exist:  # if in the list, update the cost if the new cost is smaller.
                        next_node = check_exist[(new_x, new_y)]
                        if new_g < next_node.g:
                            next_node.g = new_g
                            next_node.pre = current
                            next_node.e = (big_g - next_node.g) / next_node.h
                            if next_node.g + next_node.h < big_g:
                                if next_node in ol:
                                    heapq.heapify(ol)
                                else:
                                    heapq.heappush(ol, next_node)

                    else:
                        new_node = NodeAna(new_x, new_y)
                        new_node.g = new_g
                        new_node.h = h_estimator(new_x, new_y)
                        new_node.pre = current
                        if new_node.g + new_node.h < big_g:
                            new_node.e = (big_g - new_node.g) / new_node.h
                            heapq.heappush(ol, new_node)
                        check_exist[(new_x, new_y)] = new_node
                        frontier[(new_x, new_y)] = True
            expanded[(current.x, current.y)] = True
        print('the improvement round ended.')

        # update open list.
        temp_list = []
        for i in xrange(0, len(ol)):
            ol[i].e = (big_g - ol[i].g) / ol[i].h
            if ol[i].g + ol[i].h < big_g:
                temp_list = [ol[i]] + temp_list
        heapq.heapify(temp_list)
        ol = temp_list

    print('open list is empty, nothing to explore. ')
    # trace back to the goal to generate the optimal path.
    cur = check_exist[(end[0], end[1])].pre
    while cur.x != s.x or cur.y != s.y:
        path_optimal.append((cur.x, cur.y))
        cur = cur.pre
    # assign the roots list to path.


def get_cost(road_map, new_x, new_y):
    # calculates the step cost from (x, y) to (new_x, new_y)
    if is_wall(road_map, new_x, new_y):
        step_cost = np.inf
    else:
        step_cost = 1

    return step_cost


def d_search(road_map):  # this is the Dijkstra search algorithm
    # create an open list, make it a priority queue. put the start point into the queue.
    # im = Image.open(difficulty)
    ol = []
    heapq.heapify(ol)
    s = Node(start[0], start[1])
    s.cost = 0  # start node has a cost of 0
    heapq.heappush(ol, s)
    frontier[(start[0], start[1])] = True
    # create a hash table to check if the node is already in the open list. put the start point into the check_exist.
    check_exist = {(s.x, s.y): s}

    steps = [[0, -1], [1, 0], [0, 1], [-1, 0]]
    current = s
    # start the loop
    while ol:  # empty list is false
        # pop the top node from priority queue
        current = heapq.heappop(ol)
        # mark the current node as the frontier.
        if current.x == end[0] and current.y == end[1]:  # check if goal has been reached.
            break
        # search on all four direction of the current node.
        for i in xrange(0, 4):
            new_x = current.x + steps[i][0]
            new_y = current.y + steps[i][1]
            if check_limit(im, new_x, new_y):  # check if the new pixel is in the image or not.
                # check if it is in a wall. give a great cost to it if it is in a wall.
                step_cost = get_cost(road_map, new_x, new_y)
                new_cost = current.cost + step_cost
                # check if it is already in the open list heap.
                if (new_x, new_y) in check_exist:  # if in the list, update the cost if the new cost is smaller.
                    if new_cost < check_exist[(new_x, new_y)].cost:
                        check_exist[(new_x, new_y)].cost = new_cost
                        check_exist[(new_x, new_y)].pre = current
                        heapq.heapify(ol)
                else:  # if not in the list, then add the new node to the open list.
                    new_node = Node(new_x, new_y)
                    new_node.cost = new_cost
                    new_node.pre = current
                    heapq.heappush(ol, new_node)
                    check_exist[(new_x, new_y)] = new_node
                    frontier[(new_x, new_y)] = True  # frontier stores every node that existed in the open list.
        # put the current node into the expanded list.
        expanded[(current.x, current.y)] = True

    # trace back to the goal to generate the path.
    cur = current.pre
    roots = []
    while cur.x != s.x or cur.y != s.y:
        roots = [[cur.x, cur.y]] + roots
        cur = cur.pre
    # assign the roots list to path.
    path.extend(roots)


def is_wall(road_map, x, y):
    check_wall = road_map[x, y]
    if check_wall == 1:
        return True
    elif check_wall == (0, 0, 0, 255):
        return True
    else:
        return False


def check_limit(img, x, y):
    # check if the x, y position is in the map or not
    width, height = img.size
    if x > width - 1 or x < 0:  # x indexing from 0 - width-1
        return False
    if y > height - 1 or y < 0:
        return False
    else:
        return True


def visualize_search(save_file="do_not_save.png"):
    """
    :param save_file: (optional) filename to save image to (no filename given means no save file)
    """
    img = Image.open(difficulty).convert("RGB")
    pixel_access = img.load()

    # draw frontier pixels
    # for pixel in frontier.keys():
    #     pixel_access[pixel[0], pixel[1]] = LIGHT_GRAY

    # draw expanded pixels
    for pixel in expanded.keys():
        pixel_access[pixel[0], pixel[1]] = DARK_GRAY

    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # draw optimal path pixels
    for pixel in path_optimal:
        pixel_access[pixel[0], pixel[1]] = RED
    # draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN

    # display and (maybe) save results
    img.show()
    if save_file != "do_not_save.png":
        img.save(save_file)

    img.close()


if __name__ == "__main__":
    # Throw Errors && Such
    # global difficulty, start, end
    # assert sys.version_info[0] == 2                                 # require python 2 (instead of python 3)
    assert len(sys.argv) == 3, "Incorrect Number of arguments"      # require difficulty input

    # Parse input arguments
    function_name = str(sys.argv[1])
    difficulty = str(sys.argv[2])
    if function_name == "a":
        alg = "A_star"
    elif function_name == "d":
        alg = "Dijkstras"
    elif function_name == "ara":
        alg = "ARA"
    elif function_name == "ana":
        alg = "ANA"
    else:
        assert False, "Invalid algorithm name. "

    print("running " + alg + " with " + difficulty + " difficulty.")

    # Hard code start and end positions of search for each difficulty level
    if difficulty == "trivial.gif":
        start = (8, 1)
        end = (20, 1)
    elif difficulty == "medium.gif":
        start = (8, 201)
        end = (110, 1)
    elif difficulty == "hard.gif":
        start = (10, 1)
        end = (401, 220)
    elif difficulty == "very_hard.gif":
        start = (1, 324)
        end = (580, 1)
    elif difficulty == "test1.png" or difficulty == "test2.png":
        start = (10, 10)
        end = (79, 79)
    else:
        assert False, "Incorrect difficulty level provided"
    start_time = time.time()
    # Perform search on given image
    im = Image.open(difficulty)
    search(im.load(), alg)
    print("--- %s seconds taken by %s, with %s. " % (time.time() - start_time, alg, difficulty))
