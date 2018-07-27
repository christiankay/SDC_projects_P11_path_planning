# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can find the Term3 Simulator which is required for the Path Planning Project here [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data will be provided by the simulator but there is also a sparse map list of waypoints around the highway. The car should try to operate as close as possible to the 50 MPH speed limit, which means passing slower traffic if possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost and should keep driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to drive at least 4.32 miles without incident. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### Model Documentation

The implementation was completed in the file [main.cpp](./src/main.cpp). For spline fitting a header file [spline.h](./src/spline.h) from [this page](http://kluge.in-chemnitz.de/opensource/spline/) was used. Udacity provided a number of helper functions in main.cpp. My implementation occurred in the telemetry section of the function h.onMessage(), between lines 255 and 445.

There are numerous ways to implement this project successfully. My implementation involves the use of [cubic splines](https://en.wikipedia.org/wiki/Spline_(mathematics)) to generate smooth trajectories. This is similar to an implementation provided in [Udacity's walk-through video](https://www.youtube.com/watch?v=7sI3VHFPP0w).

The implementation is divided into two categories:

1. Environmental oberserbation and decision making 
2. Generation of the vehicle's trajectory based on a target speed

```cpp      
	// Lane identifiers for other cars
	bool too_close = false;
	bool car_left = false;
	bool car_right = false;

	// Find ref_v to use, see if car is in lane
	for (int i = 0; i < sensor_fusion.size(); i++) {
		// Car is in my lane
		float d = sensor_fusion[i][6];

		// Identify the lane of the car in question
		int car_lane;
		if (d >= 0 && d < 4) {
			car_lane = 0;
		} else if (d >= 4 && d < 8) {
			car_lane = 1;
		} else if (d >= 8 && d <= 12) {
			car_lane = 2;
		} else {
			continue;
		}

		// Check width of lane, in case cars are merging into our lane
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = sqrt(vx*vx + vy*vy);
		double check_car_s = sensor_fusion[i][5];

		// If using previous points can project an s value outwards in time
		// (What position we will be in in the future)
		// check s values greater than ours and s gap
		check_car_s += ((double)prev_size*0.02*check_speed);

		int gap = 30; // m

		// Identify whether the car is ahead, to the left, or to the right
		if (car_lane == lane) {
			// Another car is ahead
			too_close |= (check_car_s > car_s) && ((check_car_s - car_s) < gap);
		} else if (car_lane - lane == 1) {
			// Another car is to the right
			car_right |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
		} else if (lane - car_lane == 1) {
			// Another car is to the left
			car_left |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
		}
	}       
  ```    
 
In lines 261 to 307 of [main.cpp](./src/main.cpp), the positions of all the other vehicles are analyzed relative to our vehicle. If the our vehicle is within 30 meters of the vehicle in front, the boolean too_close is flagged true. If vehicles are within that margin on the left or right, car_left or car_right are flagged true, respectively.

```cpp
	// Modulate the speed to avoid collisions. Change lanes if it is safe to do so (nobody to the side)
	double acc = 0.115;
	double throttle = 0.225;
	double max_speed = 49.5;
	if (too_close) {
		// A car is ahead
		// Decide to shift lanes or slow down
		if (!car_right && lane < 2) {
			// No car to the right AND there is a right lane -> shift right
			lane++;
		} else if (!car_left && lane > 0) {
			// No car to the left AND there is a left lane -> shift left
			lane--;
		} else {
			// Nowhere to shift -> slow down
			ref_vel -= throttle;
		}
	} else {
		if (lane != 1) {
			// Not in the center lane. Check if it is safe to move back
			if ((lane == 2 && !car_left) || (lane == 0 && !car_right)) {
				// Move back to the center lane
				lane = 1;
			}
		}

		if (ref_vel < max_speed) {
			// No car ahead AND we are below the speed limit -> speed limit
			ref_vel += acc;
		}
	}
```
In lines 309 to 338 of [main.cpp](./src/main.cpp), decisions are made regarding how to adjust speed and change lanes. If a car is ahead within the gap and moving slower than target speed, the lanes to the left and right are checked. If one of them is empty, the car will change lanes. Otherwise it will slow down.

The car will move back to the center lane when it's empty. This is because a car can move both left and right from the center lane, and it is more likely to get stuck going slowly if on the far left or right.

If the area in front of the car is clear, no matter the lane, the car will speed up to the target speed [max_speed].

### 2. Trajectory Generation

Lines 341 to 445 of [main.cpp](./src/main.cpp) compute the trajectory of the vehicle based on the decision made above, the vehicle's position, and historical path points. 

```cpp
	// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
	vector<double> ptsx;
	vector<double> ptsy;

	// Reference x, y, yaw states
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	// If previous size is almost empty, use the car as starting reference
	if (prev_size < 2) {
		// Use two points that make the path tangent to the car
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);
	} else {
		// Use the previous path's endpoint as starting ref
		// Redefine reference state as previous path end point

		// Last point
		ref_x = previous_path_x[prev_size-1];
		ref_y = previous_path_y[prev_size-1];

		// 2nd-to-last point
		double ref_x_prev = previous_path_x[prev_size-2];
		double ref_y_prev = previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

		// Use two points that make the path tangent to the path's previous endpoint
		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	// Using Frenet, add 30 m evenly spaced points ahead of the starting reference
	vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for (int i = 0; i < ptsx.size(); i++) {
		// Shift car reference angle to 0 degrees
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
		ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
	}
```
In lines 340 to 401 of [main.cpp](./src/main.cpp), the last two points in the already-covered terrain are computed. If the vehicle has not yet moved 60 meters, the vehicle's current position is used instead of the historical waypoints. In addition, the Frenet helper function getXY() is used to generate three points spaced evenly at 30 meters in front of the car

Because splines are the method used to generate the trajectory, a shift and rotate transform is applied.

```cpp
	// Create a spline called s
	tk::spline s;

	// Set (x,y) points to the spline
	s.set_points(ptsx, ptsy);

	// Define the actual (x,y) points we will use for the planner
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	// Start with all the previous path points from last time
	for (int i = 0; i < previous_path_x.size(); i++) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	// Compute how to break up spline points so we travel at our desired reference velocity
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
	double x_add_on = 0;

	// Fill up the rest of the path planner to always output 50 points
	for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
		double N = (target_dist/(.02*ref_vel/2.24));
		double x_point = x_add_on + (target_x) / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// Rotate back to normal after rotating it earlier
		x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}
```

In lines 403 to 445 of [main.cpp](./src/main.cpp), the computed waypoints are transformed using a spline. The spline function enables an easy computation of a smooth trajectory in 2D space while taking into account acceleration and velocity. 

50 waypoints are generated in total. Because the length of the generated trajectory is variable, after the vehicle has assumed the correct position, the rest of the waypoints are generated to keep the vehicle in the target lane. 

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.

2. There will be some latency between the simulator running and the path planner returning a path (approx. 1-3 cycles) During this delay the simulator will continue using points that it was last given, due to this its a good idea to store the last points used so to achive have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. The program would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

