## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---
TODO:STEPS FROM COMMITS
3D
VOXEL MAP, action x 24?
up do not work

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
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

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

### Implementing Your Path Planning Algorithm

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


