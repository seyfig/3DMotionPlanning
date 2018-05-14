[//]: # (References)
[1]: ./writeup_report.md "Project Report"

# FCND - 3D Motion Planning
[Udacity Flying Car NanoDegree Term 1 Motion Planning Project](https://github.com/udacity/FCND-Motion-Planning)
![Quad Image](./misc/enroute.png)

In this project you will integrate the techniques that you have learned throughout the last several lessons to plan a path through an urban environment. Check out the [project rubric](https://review.udacity.com/#!/rubrics/1534/view) for more detail on what constitutes a passing submission.

The details of the study is given in the [Report][1].

This study implements five planners.

## 1. 3D Grid
## 2. 2D Grid
Sample run:
```
python motion_planning.py --local_goal="(290,300)"
python motion_planning.py --global_goal="(-122.400424, 37.794026))"
python motion_planning.py --grid_goal="(488,182)"
python motion_planning.py
```
First three command to specify goal, fourth one to randomly select goal.
## 3. 2D Graph
## 4. Probabilistic Roadmap
TODO TRY creating points in a smaller scale, than move to the current grid
## 5. RRT