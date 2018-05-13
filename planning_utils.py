from enum import Enum
from queue import PriorityQueue
import numpy as np
import time


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance -
                            north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance -
                            north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance -
                            east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance -
                            east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    all_actions = list(Action)
    valid_actions = []

    for new_action in all_actions:
        new_node = []
        add_action = True
        for i in range(len(current_node)):
            c = current_node[i]
            a = new_action.delta[i]
            gs = grid.shape[i] - 1
            new_c = c + a
            if new_c < 0 or new_c > gs:
                add_action = False
                #print("dont add cn:%s, i:%s, a:%s, ad:%s, c:%s, gs:%s, new_c:%s" %(
                #    current_node, i, new_action, a, c, gs, new_c))
                break
            new_node.append(new_c)

            #print("obstacle in cn:%s, nn:%s, gr:%s, a:%s, ad:%s" %(
            #     current_node, new_node, grid[new_node[0], new_node[1]],
            #     new_action, new_action.delta))
        if add_action:
            if grid[new_node[0], new_node[1]]:
                add_action = False
            else:
                valid_actions.append(new_action)
            #print("adding valid action cn:%s, nn:%s, gr:%s, a:%s, ad:%s" %(
            #     current_node, new_node, grid[new_node[0], new_node[1]],
            #     new_action, new_action.delta))

    return valid_actions


def valid_actions_check(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions


def a_star(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    # To give information about the planning process
    depth = 0
    depth_act = 0
    report_int = 2

    t0 = time.time()
    while not queue.empty():
        item = queue.get()
        current_node = item[1]

        if current_node in visited:
            continue

        visited.add(current_node)
        depth += 1

        if current_node == start:
            current_cost = 0.0
            current_action = None
            action_change_cost = 0.0
        else:
            current_cost = branch[current_node][0]
            current_action = branch[current_node][2]

        if depth % report_int == 0:
            print("#Nodes:%s, #Actions:%s, Cost:%.2f, Currenct Node:%s, Time:%.2f" %(
                depth, depth_act, current_cost, current_node, time.time() - t0))
            report_int *= 2
        if current_node == goal:
            print('Found a path.')
            found = True
            print("#Nodes:%s, #Actions:%s, Cost:%.2f, Currenct Node:%s, Time:%.2f" %(
                depth, depth_act, current_cost, current_node, time.time() - t0))
            break
        else:
            for action in valid_actions(grid, current_node):
                depth_act += 1
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0],
                             current_node[1] + da[1]
                             )
                # To prevent zigzags add a cost to changing action
                # If this cost is larger,
                # it prevents A* from planning correctly
                if current_action is not None:
                    action_change_cost = heuristic(
                        da, current_action.delta) / 100.0
                branch_cost = current_cost + action.cost + action_change_cost
                h_cost = h(next_node, goal)
                queue_cost = branch_cost + h_cost
                if next_node in branch:
                    cost_in_branch = branch[next_node][0]
                    if branch_cost < cost_in_branch:
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))
                else:
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    path = []
    path_cost = 0
    if found:

        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return path[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))
