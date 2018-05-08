from enum import Enum
from queue import PriorityQueue
import numpy as np
from scipy.spatial import Voronoi
from bresenham import bresenham


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


def create_grid25(data, safety_distance):
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
        grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = \
            int(np.rint(alt + d_alt + safety_distance))

    return grid, int(north_min), int(east_min)


def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
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
    # Initialize an empty list for Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]

        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges, int(north_min), int(east_min)


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


# Assume all actions cost the same.
class Action3D(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 0, 1)
    EAST = (0, 1, 0, 1)
    NORTH = (-1, 0, 0, 1)
    SOUTH = (1, 0, 0, 1)
    UP = (0, 0, 1, 1)
    DOWN = (0, 0, -1, 1)
    NORTH_WEST = (-1, -1, 0, np.sqrt(2))
    NORTH_EAST = (-1, 1, 0, np.sqrt(2))
    SOUTH_WEST = (1, -1, 0, np.sqrt(2))
    SOUTH_EAST = (1, 1, 0, np.sqrt(2))
    NORTH_DOWN = (-1, 0, -1, np.sqrt(2))
    NORTH_UP = (-1, 0, 1, np.sqrt(2))
    SOUTH_DOWN = (1, 0, -1, np.sqrt(2))
    SOUTH_UP = (1, 0, 1, np.sqrt(2))
    WEST_DOWN = (0, -1, -1, np.sqrt(2))
    WEST_UP = (0, -1, 1, np.sqrt(2))
    EAST_DOWN = (0, 1, -1, np.sqrt(2))
    EAST_UP = (0, 1, 1, np.sqrt(2))
    NORTH_WEST_DOWN = (-1, -1, -1, np.sqrt(3))
    NORTH_WEST_UP = (-1, -1, 1, np.sqrt(3))
    NORTH_EAST_DOWN = (-1, 1, -1, np.sqrt(3))
    NORTH_EAST_UP = (-1, 1, 1, np.sqrt(3))
    SOUTH_WEST_DOWN = (1, -1, -1, np.sqrt(3))
    SOUTH_WEST_UP = (1, -1, 1, np.sqrt(3))
    SOUTH_EAST_DOWN = (1, 1, -1, np.sqrt(3))
    SOUTH_EAST_UP = (1, 1, 1, np.sqrt(3))

    @property
    def cost(self):
        return self.value[3]

    @property
    def delta(self):
        return (self.value[0], self.value[1], self.value[2])


# Assume all actions cost the same.
class Action2D(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 0, 1)
    EAST = (0, 1, 0, 1)
    NORTH = (-1, 0, 0, 1)
    SOUTH = (1, 0, 0, 1)
    UP = (0, 0, 1, 1)
    DOWN = (0, 0, -1, 1)
    NORTH_WEST = (-1, -1, 0, np.sqrt(2))
    NORTH_EAST = (-1, 1, 0, np.sqrt(2))
    SOUTH_WEST = (1, -1, 0, np.sqrt(2))
    SOUTH_EAST = (1, 1, 0, np.sqrt(2))
    NORTH_DOWN = (-1, 0, -1, np.sqrt(2))


    @property
    def cost(self):
        return self.value[3]

    @property
    def delta(self):
        return (self.value[0], self.value[1], self.value[2])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle
    #print("A* ", x,y, grid[x-1,y])

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


def valid_actions3D(voxmap, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    all_actions = list(Action3D)
    valid_actions = []
    n, m, t = voxmap.shape[0] - 1, voxmap.shape[1] - 1, voxmap.shape[2] - 1
    x, y, z = current_node

    # check if the node is off the grid or
    # it's an obstacle

    """
    if x - 1 < 0:
        valid_actions.remove(Action3D.NORTH)
        valid_actions.remove(Action3D.NORTH_WEST)
        valid_actions.remove(Action3D.NORTH_EAST)
        valid_actions.remove(Action3D.NORTH_DOWN)
        valid_actions.remove(Action3D.NORTH_UP)
        valid_actions.remove(Action3D.NORTH_WEST_DOWN)
        valid_actions.remove(Action3D.NORTH_WEST_UP)
        valid_actions.remove(Action3D.NORTH_EAST_DOWN)
        valid_actions.remove(Action3D.NORTH_EAST_UP)
    """
    for new_action in all_actions:

        new_x = x + new_action.delta[0]
        new_y = y + new_action.delta[1]
        new_z = z + new_action.delta[2]
        #print(new_action, new_x, new_y, new_z, voxmap[new_x, new_y, new_z])
        if (new_x < 0 or new_x > n or
            new_y < 0 or new_y > m or
            new_z < 0 or new_z > t or
                voxmap[new_x, new_y, new_z]):
            pass
        else:
            valid_actions.append(new_action)
            #valid_actions.remove(new_action)
            #print("remove ", new_action)
    #print(valid_actions)

    """
    if x - 1 < 0 or voxmap[x - 1, y, z] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or voxmap[x + 1, y, z] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or voxmap[x, y - 1, z] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or voxmap[x, y + 1, z] == 1:
        valid_actions.remove(Action.EAST)
    if z - 1 < 0 or voxmap[x, y, z - 1] == 1:
        valid_actions.remove(Action.DOWN)
    if z + 1 > t or voxmap[x, y, z + 1] == 1:
        valid_actions.remove(Action.UP)
    if (x - 1 < 0 or y - 1 < 0) or voxmap[x - 1, y - 1, z] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or voxmap[x - 1, y + 1, z] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or voxmap[x + 1, y - 1, z] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or voxmap[x + 1, y + 1, z] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    """

    return valid_actions


def a_star(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    depth = 0
    depth_act = 0

    while not queue.empty():
        depth += 1

        item = queue.get()
        current_node = item[1]

        if current_node in visited:
            continue

        visited.add(current_node)

        if current_node == start:
            current_cost = 0.0
            current_action = None
            action_change_cost = 0.0
        else:
            current_cost = branch[current_node][0]
            current_action = branch[current_node][2]
        if depth % 1000 == 0:
            print(depth, depth_act, current_cost, item[0], item[1])
        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                depth_act += 1
                # get the tuple representation
                da = action.delta
                #print("A* current_node:", current_node, " da:", da)
                next_node = (current_node[0] + da[0],
                             current_node[1] + da[1]
                             )
                # if current_action is not None:
                #    action_change_cost = heuristic(da, current_action.delta)
                branch_cost = current_cost + action.cost
                h_cost = h(next_node, goal)
                queue_cost = branch_cost + h_cost + action_change_cost
                # if action_change_cost > 3:
                #     print("CC:%.4f, AC:%.4f, BC:%.4f, HC:%.4f, ACC:%.4f, QC:%.4f, CN:%s, NN:%s, CA:%s, A:%s" % (
                #         current_cost, action.cost, branch_cost,
                #         h_cost, action_change_cost, queue_cost, current_node, next_node, current_action, action))

                # print("CN: ",
                #       current_node,
                #       "; A: ",
                #       action,
                #       "; NN: ",
                #       next_node,
                #       "; BC: ",
                #       branch_cost,
                #       "; HC: ",
                #       h_cost,
                #       "; TC: ",
                #       queue_cost)
                if next_node in branch:
                    cost_in_branch = branch[next_node][0]
                    if branch_cost < cost_in_branch:
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))
                else:
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    # TODO CHECK ADDED FROM GRAPH A*
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
        # while branch[n][1] != start:
        #    path.append(branch[n][2])
        #    n = branch[n][1]
        # path.append(branch[n][2])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return path[::-1], path_cost


def a_star3D(voxmap, h, start, goal):
    print("goal type: %s, start type:%s" %(type(goal), type(start)))
    if type(goal) != type(start):
        print("type error")
        return False
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    # TODOD FOR DEBUGING
    depth = 0
    depth_act = 0

    # TODOD FOR ACTION COST, IF not used remove it
    current_action = None
    action_change_cost = 0.0


    while not queue.empty():
        depth += 1

        item = queue.get()
        current_node = item[1]

        if current_node in visited:
            continue

        visited.add(current_node)

        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]
            current_action = branch[current_node][2]
        if depth % 1000 == 0:
            hc_cost = h(current_node, goal)
            print("D, DA:", depth, depth_act, "CC:", current_cost, "HC:", hc_cost, "QC:", item[0], "CN:", item[1])
        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions3D(voxmap, current_node):
                depth_act += 1
                # get the tuple representation
                da = action.delta
                #print("A* current_node:", current_node, " da:", da)
                next_node = (current_node[0] + da[0],
                             current_node[1] + da[1],
                             current_node[2] + da[2]
                             )
                #if current_action is not None:
                #    action_change_cost = heuristic(da, current_action.delta)
                branch_cost = current_cost + action.cost
                h_cost = h(next_node, goal)
                queue_cost = branch_cost + h_cost + action_change_cost
                # if action_change_cost > 3:
                #     print("CC:%.4f, AC:%.4f, BC:%.4f, HC:%.4f, ACC:%.4f, QC:%.4f, CN:%s, NN:%s, CA:%s, A:%s" % (
                #         current_cost, action.cost, branch_cost,
                #         h_cost, action_change_cost, queue_cost, current_node, next_node, current_action, action))

                # if action_change_cost > -4:
                #     print("CC:%.4f, AC:%.4f, BC:%.4f, HC:%.4f, ACC:%.4f, QC:%.4f, CN:%s, NN:%s, S:%s, G:%s, N-G:%s, C-G:%s, CA:%s, A:%s, V:%s" % (
                #         current_cost, action.cost, branch_cost,
                #         h_cost, action_change_cost, queue_cost, current_node, next_node,
                #         start, goal, next_node == goal, current_node == goal,
                #         current_action, action, voxmap[next_node[0],next_node[1],next_node[2]]))


                # print("CN: ",
                #       current_node,
                #       "; A: ",
                #       action,
                #       "; NN: ",
                #       next_node,
                #       "; BC: ",
                #       branch_cost,
                #       "; HC: ",
                #       h_cost,
                #       "; TC: ",
                #       queue_cost)

                if next_node in branch:
                    cost_in_branch = branch[next_node][0]
                    if branch_cost < cost_in_branch:
                        branch[next_node] = (branch_cost, current_node, action)
                        queue.put((queue_cost, next_node))
                else:
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

                # TODO CHANGE FOR ZIGZAG
                """
                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
                """

    # TODO CHECK ADDED FROM GRAPH A*
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
        #while branch[n][1] != start:
        #    path.append(branch[n][2])
        #    n = branch[n][1]
        #path.append(branch[n][2])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return path[::-1], path_cost


def a_star_graph(graph, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    depth = 0
    depth_act = 0

    while not queue.empty():
        depth += 1

        item = queue.get()
        current_node = item[1]

        if current_node in visited:
            continue

        visited.add(current_node)

        if current_node == start:
            current_cost = 0.0
            current_action = None
            action_change_cost = 0.0
        else:
            current_cost = branch[current_node][0]
            current_action = branch[current_node][2]
        if depth % 1000 == 0:
            print(depth, depth_act, current_cost, item[0], item[1])
        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)

                if next_node in branch:
                    cost_in_branch = branch[next_node][0]
                    if branch_cost < cost_in_branch:
                        branch[next_node] = (new_cost, current_node)
                        queue.put((new_cost, next_node))
                else:
                    branch[next_node] = (new_cost, current_node)
                    queue.put((new_cost, next_node))


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
        # while branch[n][1] != start:
        #    path.append(branch[n][2])
        #    n = branch[n][1]
        # path.append(branch[n][2])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def heuristic3D(position, goal_position):
    #return np.linalg.norm(np.array(position) - np.array(goal_position))
    diff = np.array(position) - np.array(goal_position)
    diff_abs = np.abs(diff)
    # sorted diff
    ds = np.sort(diff_abs)
    cost = ds[0] * np.sqrt(3)
    cost += (ds[1] - ds[0]) * np.sqrt(2)
    cost += (ds[2] - ds[1])
    return cost


def heuristic2D(position, goal_position):
    #return np.linalg.norm(np.array(position) - np.array(goal_position))
    diff = np.array(position) - np.array(goal_position)
    diff_abs = np.abs(diff)
    # sorted diff
    ds = np.sort(diff_abs)
    cost = ds[0] * np.sqrt(3)
    cost += (ds[1] - ds[0]) * np.sqrt(2)
    cost += (ds[2] - ds[1])
    return cost

