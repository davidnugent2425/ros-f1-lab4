# Follow The Gap Lab

Today we will be implementing the reactive Follow The Gap autonomuos driving method. To understand how Follow The Gap works, check out this [video](https://www.youtube.com/watch?v=7VLYP-z9hTw&t=861s&ab_channel=Real-TimemLABUPenn).

## New run-simulator Script

We have updated the run-simulator script to now optionally take the name of the map you want to use. We will start by removing the old script and downloading the new script from our Crash Course repo. We will also download a new file which just contains the names of all the f110 maps, for easy access

```
rm run-simulator.sh
wget https://raw.githubusercontent.com/FT-Autonomous/Autonomous_Crash_Course/main/docker-setup/run-simulator.sh
wget https://raw.githubusercontent.com/FT-Autonomous/Autonomous_Crash_Course/main/docker-setup/maps.txt
```

We will be using this new functionality later in the lab

## File Structure Setup

We will start using our workspace from the [last tutorial](https://github.com/davidnugent2425/ros-f1-lab1) and making a new package for Follow The Gap

```
source sample_ws/devel/setup.bash
cd sample_ws/src
catkin_create_pkg follow_the_gap std_msgs rospy
```

Now let's go into the source directory of our new package and download the [driver.py](./driver.py) file. This file contains a full implementation of the Follow The Gap method. If you want to start your own from scratch, you can use the [skeleton code](https://github.com/f1tenth/f1tenth_labs/blob/master/lab4/code/src/reactive_gap_follow.py) provided by F1Tenth

```
roscd follow_the_gap/src
wget https://raw.githubusercontent.com/davidnugent2425/ros-f1-lab4/master/driver.py
chmod +x driver.py
```

## Testing the Follow The Gap Method

In another terminal, run the simulator using it's default map or pass in a map of your choice

```
source /run-simulator.sh
```

```
source /run-simulator.sh columbia
```

To view the list of the maps provided by F1Tenth

```
cat /maps.txt
```

With the simulator running, let's run our Follow The Gap driver. You should then see what steering angle we are prompting the car to move in

```
rosrun follow_the_gap driver.py
```
But the car doesn't move yet, this is because we need to press N for Navigate in the terminal running the simulator. Now the car should move. By using the green arrow button at the top of the simulator screen you can place the car wherever you want in the map

Let's take a look at the [driver.py](./driver.py) file and experiment with the hyperparameters

```
rosed follow_the_gap driver.py
```

The ```reactive_follow_gap``` class includes a number of hyperparameters you can mess around with. Change some of these values and see what effect it has on the car's movements

```python
	BUBBLE_RADIUS = 160
	PREPROCESS_CONV_SIZE = 3
	BEST_POINT_CONV_SIZE= 80
	MAX_LIDAR_DIST = 3000000
	STRAIGHTS_SPEED = 4.0
	CORNERS_SPEED = 0.5
	STRAIGHTS_STEERING_ANGLE = math.pi / 9 # 20 degrees
```
