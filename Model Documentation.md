# Path Planning Project
Self-Driving Car Engineer Nanodegree Program

---

## Picking a Lane and Speed

The drive function in the Car class is where we determine which lane to be in and what speed to go at. We do this by comparing the current "speed" of each lane, calculated in the laneCurrentSpeed function in the Map class. The lane speed is the speed that the closest car in front of you in that lane is driving. If there is no car close to our current position in the lane then the lane speed is just the speed limit. If it isn't safe to merge into another lane (there are cars too close to us, determined by the laneIsOpen function in the Map class) then the lane speed is zero.

We pick the lane with the highest current speed. If all the lanes are at the speed limit (meaning there are no cars close to us) then we drive in the middle lane because then we have the most options when we do get behind a car (it is no good to be stuck in the left lane because the middle lane is full while the right lane is empty).

The speed we drive at is the lane speed. Meaning if there is a car in front of us in the lane we will drive at the speed of that car. If there is no car in front we drive at the speed limit.

## Generating Trajectories

Once the lane and speed have been chosen we need to actually generate the x and y coordinates that will compose the path the car drives on. This is done in the generateTrajectory function in the Car class. In order to minimize jerk and create a smooth path we use the spline library. The spline takes an input a list of x,y points and it creates a smooth path between them.

The get the points to feed to the spline we use either the current position of the car or the position the car will be in when it finishes the path that has been previously calculated. We then generate 3 more points 30, 60, and 90 meters farther along the road from that point. These points are assuming we are just driving in the middle of the lane like normal.

Once we have these fixed guide points we use the spline to break up the path into smaller segments depending on the desired speed of the car. The faster we want to go the farther apart the points will be. Then we add those points along the spline onto the actual trajectory that the car will follow.

Using splines makes this whole process fairly easy as it takes care of smoothing the path and minimizing the jerk. The only part we have to worry about is making sure we get the speed and acceleration right.

## Potential Improvements

The "lane speed" is a pretty basic cost function. We could also consider looking at how far away the next closest car is and try to see if we'll be able to merge out of the lane before we get close to it.

I don't have any concept of a "preparing to change lanes" state like discussed in the class. The proposal in the class was that the car would slow down to try to track an opening in another lane. I don't find this behavior very intuitive, if you have to slow down to change lanes than you are likely better off just staying in the current lane. When I drive I just stay behind the car in front of me until I see an opening in the other lane, so that's what I had my self driving car do. But maybe my driving isn't optimal and there could definitely be scenarios where preparing for a lane change would be beneficial.
