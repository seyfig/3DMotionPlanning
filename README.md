[//]: # (References)
[1]: ./writeup_report.md "Project Report"

# FCND - 3D Motion Planning
[Udacity Flying Car NanoDegree Term 1 Motion Planning Project](https://github.com/udacity/FCND-Motion-Planning)
![Quad Image](./misc/enroute.png)

In this project you will integrate the techniques that you have learned throughout the last several lessons to plan a path through an urban environment. Check out the [project rubric](https://review.udacity.com/#!/rubrics/1534/view) for more detail on what constitutes a passing submission.

The details of the study is given in the [Report][1].

This study implements five planners.

## 1. 3D Grid  
### 1. Run with motionplot3Dvox.ipynb (View motionplot3Dvox.html)  
View motionplot3Dvox.html  
### 2. Sample video for planning to the top of a building  


[![Path Planning](http://img.youtube.com/vi/5gAs-jm4Tdw/0.jpg)](https://youtu.be/5gAs-jm4Tdw)


### 3. Sample video for planning to a hole inside of a building  


[![Path Planning](http://img.youtube.com/vi/WrGPHvKMSIs/0.jpg)](https://youtu.be/WrGPHvKMSIs)


### 4. Sample video for planning over obstacles  


[![Path Planning](http://img.youtube.com/vi/PcxGqdvs7L0)](https://youtu.be/PcxGqdvs7L0)


## 2. 2D Grid  
### 1. Run with motion_planning.ipynb (View motion_plot.html)  

### 2. Run with motion_planning.py:


```
python motion_planning.py --local_goal="(290,300)"
python motion_planning.py --global_goal="(-122.400424, 37.794026))"
python motion_planning.py --grid_goal="(488,182)"
python motion_planning.py
```


First three command to specify goal, fourth one to randomly select goal.


## 3. 2D Graph  
### 1. Run with motionplotgraph.ipynb (View motionplotgraph.html)  
## 4. Probabilistic Roadmap  
### 1. Run with probabilisticroadmap.ipynb (View probabilisticroadmap.html)  
## 5. RRT  
### 1. Run with rrt.ipynb (View rrt.html)  