## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Path Planning Project**

The goals / steps of this project are the following:

* The code compiles correctly. Code must compile without errors with cmake and make.
* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit.
* Max Acceleration and Jerk are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes.
* There is a reflection on how to generate paths.


## [Rubric](https://review.udacity.com/#!/rubrics/1020/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README  

You're reading it!


### Compilation

#### 1. The code compiles correctly. Code must compile without errors with cmake and make.

After the last push to GitHub, I compiled the code to ensure there were not any errors before submitting for review.


### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident. The car drives according to the speed limit. Max Acceleration and Jerk are not Exceeded. Car does not have collisions. The car stays in its lane, except for the time between changing lanes. The car is able to change lanes.

The car continues within its lane, driving at a max speed of approximately 48.5 mph. When there is a car in front, it slows down and attempts to lane change if possible. The max acceleration and jerk are not exceeded by smoothing the transitions between lanes. The car avoids collisions by predicting the future environment before making a lane change. The car stays in its lane until a lane change is needed. The car is able to change lanes. 


### Reflection

#### 1. There is a reflection on how to generate paths.

The way I generated the path is similar to the project walkthrough. I pushed 50 waypoints into the simulator. To protect from latency, I kept the first 10 points from the previous path's points to ensure a smooth transition (line 436). I then found what my car's s and d values would be during that 10th point into the future from the previous path (line 264). I compared this future prediction to the other vehicles' future predictions (line 300). I predicted the other vehicles based on their current velocities, respectively (line 310). I used this future position landscape of my car and the other vehicles to make a decision to switch lanes and/or change speed (line 311-391). I pushed the last 3 waypoints added to the next_waypoints from the previous path into the points used to spline the next points (line 454). This allows continuity between the curvature of the next path and the previous path. I added points 40m into the future to the model (line 464). I transformed the coordinates based on the last entered waypoint from the previous path (line 477). I splined the points and untransformed future points before adding the next waypoints into the waypoint vector (line 486-517).


---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your model likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the model might fail and how I might improve it if I were going to pursue this project further.  

A lot of my struggles with this project came with the logic and latency. My understanding of latency was, originally, not sufficient enouh to complete this project. I accredited earlier mistakes to problems with the logic in the Project Walkthrough, when they were due to not accounting for latency. Also, there were several times where I could not understand/correct the behavior of the vehicle, because I made a simple mistake in the logic that I kept overlooking. Once I understood the latency and corrected the logical errors, the project became simpler. Most of the used techniques were explained in the Project Walkthrough. 
My model would likely fail for cases where my logic has not been represented. That is one major issue when coding behavior logically; there could always be some improbable case overlooked. I could make it more robust by tuning the parameters in the model. My lane switching algorithm is conservative at the moment, but could improve with parameter tuning.