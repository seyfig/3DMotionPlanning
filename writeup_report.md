## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---
TODO:STEPS FROM COMMITS
3D
VOXEL MAP, action x 24?
up do not work

TODO
# Videos
1. to the hole
2. ower the buildings
Grid Start and Goal:  (209, 911, 5) (617, 919, 5)
Vox Start and Goal:  (41, 182, 1) (123, 183, 1)
change [0.71447238 0.36429018 0.52004226]
goal:  (-122.3850218762088, 37.79512860180831, 130.09256609855177)
lon lat: (37.79248, -122.39745)  vs  (316, 445)
lla:  37.7914857 -122.3921666 0.082 ```(self._latitude, self._longitude, self._altitude)```

    feat: Implement 3D navigation motionplot3Dvox notebook

commit 2177f2879f43fc0524a8aefcd1106dab833aa7e0
Author: seyfi <sgzbyk@gmail.com>
Date:   Sun May 6 13:34:56 2018 +0300

    feat: Add voxel_size parameter to plan faster.

commit e99d5d8f8d17e079a7ed454592ef0103ebb8cfb9
Author: seyfi <sgzbyk@gmail.com>
Date:   Sun May 6 11:37:14 2018 +0300

    feat: 3D planning with voxel map

commit e884c8716d01a91e398536e09353c36389a12b73
Author: seyfi <sgzbyk@gmail.com>
Date:   Thu May 3 16:35:40 2018 +0300

    fix: Fix zig zag in A*

commit 502c8f2917bc975ae426fc9924c76311c495a8f6
Author: seyfi <sgzbyk@gmail.com>
Date:   Mon Apr 30 16:25:11 2018 +0300

    Add pruning path to motionplot notebook

commit 852616c4c0fc643e0f5c2782ba5bc2262f4b65c8
Author: seyfi <sgzbyk@gmail.com>
Date:   Sun Apr 29 22:40:57 2018 +0300

    feat: Implement A* without pruning in motionplot notebook

commit c506844fda6be60dc67d67c7736ae4479a2dabfe
Author: seyfi <sgzbyk@gmail.com>
Date:   Sun Apr 29 15:34:25 2018 +0300

    doc: Explain motion_planning vs backyard_flyer

commit 5fd186f191bd2e2638f2071487fe0b08020b8136
Author: seyfi <sgzbyk@gmail.com>
Date:   Fri Apr 20 17:01:19 2018 +0300

    init: Clone from Udacity Repo



# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`


State_callback function in the motion_planning.py includes
```
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
```


The state order
For backyard_flyer.py is:
MANUAL > ARMING > TAKEOFF > WAYPOINT > LANDING > DISARMING

For motion_planning.py is:
MANUAL > ARMING > PLANNING > TAKEOFF > WAYPOINT > LANDING > DISARMING

The motion_planning.py has a planning state and a plan_path method, where as the backyard_flyer.py has not.

The plan_path method includes the following steps, currently missing steps are marked with (T)
* Set the target_position, TARGET_ALTITUDE
* (T) read lat0, lon0 from colliders into floating point values
* (T) set home position to (lon0, lat0, 0)
* (T) retrieve current global position
* (T) convert to current local position using global_to_local()
* Read the obstacle map
* Define a grid for a particular altitude and safety margin around obstacles
* Define starting point on the grid (this is just grid center)
* (T) Convert start position to current position
* Set goal as some arbitrary position on the grid
* (T) Adapt to set goal as latitude / longitude position and convert
* Run A* to find a path from start to goal, with a heuristic cost
* Add diagonal motions with a cost of sqrt(2) to your A* implementation
* Prune path
* Convert path to waypoints
* Send the waypoints to the sim to visualize

There are three main functions to perform planning, all of them are present in the planning_utils.py.


1. heuristic  
This function provides a heuristic function of the distance between two given points.

2. create_grid  
This function converts the map to a 2D grid at the given altitude and using the specified safety distance.

3. a_star  
This function creates a path using the a_star algorithm. It takes the grid, heuristic function, start position and the goal position as inputs. Currently, it performs the search by applying all possible actions at a state. It selects the next state with the lowest cost.
  
Currently, the goal position is hardcoded as some location 10 m north and 10 m east of map center. There are no diagonal motions. Therefore, the planning results in one step north and one step east.


TODO DELETE 
These scripts contain a basic planning implementation that includes...

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm (2D Grid A* algorithm)
  
There are TODO algorithms implemented. These are
1. 2D Grid A* Algorithm
2. 3D Voxmap A* Algorithm
3. TODO Graph

TODO
This part describes the general steps. Each algorithm has a specific part below. Each part has the steps that are changing for the algorithm.
TODO

#### 1. Set your global home position
The plan_path method reads the colliders.csv file and gets the lat0 and lon0 values. Convert these values to float, and send them to the self.set_home_position function. This function sets the global home position.
  
#### 2. Set your current local position
The local position is the output of the self.global_to_local function. The inputs for the function are, self.global_position and the self.global_home.
  
#### 3. Set grid start position from local position
The start position is calculated from the local position obtained in the previous step. The local position value consists of the north value, the east value and the altitude.
  
However, these north and east values are not ready to use. It is required to subtract the offset values. Offset values are calculated in the create_grid function in the planning_utils.py file. They are the minimum north value, and the minimum east value from the data read from the colliders.csv. The create_grid function convert these offset values to integer. The values in the grid_start tuple are also converted to integer.

For 3D planning, grid_start tuple also contains the altitude. That value is calculated by summing the altitude value in the global position and the TARGET_ALTITUDE value.

#### 4. Set grid goal position from geodetic coords
There are two ways to set the grid goal. Sending a geodetic coord(lat, lon) is one way. If there is no parameter sent, then the plan_path method sets the goal randomly.

Then the method converts the goal location to local coordinates, and then the grid coordinates. If it exceeds an edge of the grid, then the value is changed back to the value of the edge in order to stay in the grid. 

For 3D planning, the randomly assigned altitude value has a maximum that is defined by the MAX_ALTITUDE.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
For 2D planning, the action object in the planning_utils.py file includes 4 additional actions that are diagonal.

In the first version of the a_star function in the planning_utils.py file, when a node can be reached with a lower cost after it was added to the branch, it is not possible to change the path for that node. In order to correct this error, the code location to add a node to the visited list is changed. A node is added to the visited list, only when it is the current node. Previously, it was added when it was the next_node. This change enabled a node to be placed in the queue more than once. It will be visited with the lowest queue cost. After it is visited, the other items in the queue, containing the same node, will be skipped without processing.

Another thing to change was adding a cost for changing action, in order to prevent too many zigzag moves. However, the cost changed the behavior of the a_star planning, therefore it isn't implemented.

#### 6. Cull waypoints
The collinearity_check was performed to determine whether a point in the path can be removed or not. If three points fit in the same line, then the second of them was removed.


### 3D Grid A* algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.


### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


