## Path Planning Project

### Lane Selector 
A cost function was developed to assist in lane selection. Four seperate components were used in calculating the cost function. 

1. A bonus for the current lane
2. Penalty for a two lane switch 
3. Penalty for a collision risk
4. Penalty if lane is blocked

Of the three possible lanes, the lane with the lowest cost is selected. Initially, the weights for each component were set at 1.0. I then tuned the weights to improve the performance of the lane selector. 

Each component of the cost function works as follows:

#### 1. Current lane
A preference bonus for the current lane.  

Final weight = 2.0

#### 2. Two Lane Switch
A penalty for attempting to switch two lanes at once. This turns out to be a really dangerous maneuver, so I set the weight high enough so that it would not trigger. However, this doesn't stop the selector from choosing a two lane switch over successive cycles. 

Final weight = 10.0

#### 3. Collision
Goes through the list of cars, and checks to see if the distance is within 30 m of the EGO car. 

`if ((car_s-30.0) < target_s && target_s < (car_s+30.0))`
then add a penalty to lane that the target car is in. 
                                                            
Final weight = 2.0

#### 4. Blocked Lane
Goes though the list of cars. If the target car is in front of the EGO car, but within 60 m, then add a penalty to the lane that the target car is in.  

`if (target_s > car_s && (target_s-car_s) < 60.0)`
add penalty to the lane that the target car is in. 
              
Final weight = 1.0

#### Lane selector reflection
The lane selector generally worked pretty well, but could be improved. I did not do any tuning of the +/-30 m distance for collision checks or the 60 m for the lane blocking check. These could be tuned, especially since there is some overlap so a single car can count in both checks and skew the results. Also, it may be helpful to use the velocity in some way to come up with a better check than just distance. 

Ultimately, the results showed that this algorithm exceeded the requirements, so this method was reasonably successful.  


### Velocity Adjustments and In-Lane Follower 
The basic idea for modifying the vehicle velocity was taken from the class walkthrough video, but the algorithm was modified to improve its performance when following behind a vehicle. 

The basic idea of the algorithm is to check within the lane by propagating the state of any car in the lane to a future point (which turns out to be 1 sec in the future). The distance, s, of the target car is checked against the prediction for the EGO car at that same time. If the EGO car final prediction is within 30 m of a target car that is in front of it, then the EGO car's reference speed is slower by a velocity increment (chosen to be 0.224 mph as not to exceed the vehicle's maximum acceleration limit). I made a small modifcation to the algorithm such that if the target car speed and the EGO car speed are within a single velocity increment, the EGO car reference speed is set to that of the target car. This has the effect of reducing the car speeding up and slowing down, and helps the EGO car maintain a more stable velocity. 

If the lane is clear, and the EGO car velocity is less than 49.5 mph, then the velocity increment (0.224 mph) is added. 
          

### Trajectory Generation
The trajectory generation algorithm I used is directly from the class walkthrough video. The basic idea is to use two points from the previous timestep (or estimate them if not yet populated) then add on additional points as needed to fill out the vector using a spline fit. The additional points to be used for the spline fit are based on the target lane, which may or may not be the same as the current lane, and are found at 30, 60, and 90 m intervals in Frenet space (and converted to XY):

`vector<double> next_mp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);`

`vector<double> next_mp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);`

`vector<double> next_mp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);`

The new points are added to the vector of XY points from the previous step, then rotated to a frame aligned with the EGO car's yaw:

`ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));`
                
`ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));`

The `ptsx` and `ptsy` vectors are used to generate the spline fit, using the `s` function. 

The target distance for the vector of points to be sent to the simulator (`next_x_vals` and `next_y_vals`) is determined by finding the Euclidian distance based on a 30 m distance in x and the result of spline fit in y. 
         
`double target_x = 30.0; // meters`

`double target_y = s(target_x);`

`double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));`
 
This target distance is used to fill out the rest of the `next_x_vals` and `next_y_vals` vectors. The resolution is found by using the target distance, vehicle velocity, and 20 ms update. 

`double N = (target_dist / (0.02*ref_vel/2.24)); // 2.24 mph to m/s`

`double x_point = x_add_on + (target_x) / N;`

`double y_point = s(x_point);`
                
These points are then rotated back to the XY frame and added to `next_x_vals` and `next_y_vals`. 

### Results 
Two trials were completed with the final algorithm. The first trial, the path planner ran for 19.38 mi before an incident occurred. The second trial went 23.86 mi before an incident occurred. This is greater than the required 4.32 mi. 

The average car speed in the second trial was greater than 45 mph, so the vehicle is doing a fairly good job of staying close to the speed limit when possible. 

Here is an image just after Trial 1, showing the 19.38 mi.
![img1](/images/img19.png "Trial 1")

Here is an image following Trial 2, showing that the car went 23.86 mi without an incident. It looks as though the incident was pretty catastrophic. 
![img2](/images/img24.png "Trial 2")


### Basic Build Instructions
To run the simulation:
1. Clone this repo.
2. Make a build directory: mkdir build && cd build
3. Compile: cmake .. && make
4. Run it: ./path_planning.
