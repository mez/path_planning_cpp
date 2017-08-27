# Path Planning
In this project, the goal is to safely navigate around a virtual highway with other 
traffic that is driving +-10 MPH of the 50 MPH speed limit. 
We are provided with the car's localization and sensor fusion data, 
there is also a sparse map list of waypoints around the highway. 
The car tries to go as close as possible to the 50 MPH speed limit, 
which means passing slower traffic when possible, 
note that other cars will try to change lanes too. 
The car should avoid hitting other cars at all cost as well as driving inside of the 
marked road lanes at all times, unless going from one lane to another. 
The car should be able to make one complete loop around the 6946m highway. 
Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
Also the car should not experience total acceleration over 10 m/s^2 and jerk 
that is greater than 50 m/s^3.

   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    

## Reflection: Model Documentation

I decided to go with a very simple heuristic for my model.
This works because the search space for the highway in the simulator 
is very constrained. Hence the idea was to make a model
that was very general and the decisions it makes could be combined
without having bad side effects. The first thing I'll discuss is the 
path generation. 

### Path Generation
The waypoints we receive from the simulator are sparse. Hence, we could not 
rely on them for a continuous smooth path. The solution was to first create anchor points 
spaced 30 meters apart and then use a library called [Spline](http://kluge.in-chemnitz.de/opensource/spline/) to fill in the points in between the anchors at the desired speed. At any points in time, there
are 50 points being fed into the simulator. Hence, if the simulator only consumed five points to move
the perfect controller, the next timestep we only generate five new points and we add that to the back of the 
paths list. This is very efficient, since we don't re-generate the unused points.

### Jerk
The model has a very simple method to control the jerk; it can only gradually speed up or slow down. 
```
if (too_close) {
  target_velocity -= .224;
} else if (target_velocity < utils::MAX_VELOCITY) {
  target_velocity += .224;
}
```

### Fusion Map
One of the first things I do when I get the sensor fusion list
is to categorize each reading into a lane bin.

```
//fusion_map[LEFT_LANE] would return the vector containing
//all readings for the LEFT_LANE
map<int, vector<utils::SensorReading>> fusion_map;
```
This allowed me to reduce unnecessary checks.

### Avoiding Collision
The model easily can avoid collision by first looking at all the 
cars in its lane `fusion_map[current_lane]` and checking
if any car will be less than 30 meter from where I'll be in the future.
We can use the car's `s` position in frenet coordinates and the car's
speed to predict if that car will be in our way in the future timestep.
If we detect a car ahead that is too close, we simple reduce
speed until we match speed.

### Lane Changing
The lane changing strategy used was pretty simple.
Once we detected a car is too close in our current lane,
we go into trying to change lanes mode. To keep things simple
we avoid trying to make multiple lane changes.
For example, Left_Lane maps only to Center_Lane.
Center_Lane maps to [Left_Lane, Right_Lane] and
Right_Lane maps to Center_Lane.

The expression we use to check if a lane change is possible
is to check if in the future timestep there is at least a 
10 meter gap to the front and back.

If we are in the Center_Lane, then it first checks the Left_Lane
if that fails we then check Right_Lane. In all cases,
if lane change fails, we simply maintain the current lane.

The search space for path generation was so small I did not need a complex cost function.