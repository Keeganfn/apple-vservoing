## Moveit2 Apple Picking Demo
![](https://github.com/Keeganfn/apple-vservoing/blob/main/docs/github_demo_gif.gif)
This repo uses the ur5e with an attached RGBD camera in gazebo to find and pick "apples" (red spheres). It uses Moveit2 for initial apple approach and Moveit2 Servo for the final pick approach. OpenCV is used to identify "apple" locations with the RGBD stream. 

The purpose of this demo is to simulate real world approaches to robotic apple picking where an initial pick approach is chosen, and the final pick approach uses finer grain visual servoing. This helps mitigate problems that occur due to bad/noisy sensor data from an initial scan of the environment.

### Two planning stages: 
- **global_planner node:** Uses the RGBD camera feed to find the closest apple to the camera on the end effector. It then finds the desired goal pose (with noise) and uses moveit to plan/move to the pose goal. 
- **local_planner node:** Uses the RGBD camera feed to find the closest apple and sends Twist velocity commands to servo the arm to the centerpoint of the apple. Stops a predefined distance in front of the apple to simulate a pick. Uses Moveit Servo to control the arm.


### Two Demos:
- **demo1 (full):** Environment has 5 apples that need to be picked. Demo first uses the global planner for initial approach followed by the local planner for final approach. Apples should despawn after they are "picked".
- **demo2 (local):** Environment has 5 apples that need to be picked. Demo only uses visual servoing in the local planner to approach apples. This is meant to demonstrate the visual servoing controller over larger distances. Apples should despawn after they are "picked".

## Installation
This demo use modified versions of the Universal Robotics ROS2 repos to simulate the UR5 in Gazebo with Moveit. Changes had to be made to those repos to get cameras, moveit, moveit servo and gazebo working together, so they are provided in this github.

1. Clone this repo into your workspace (changes had to be made to the UR repos so do not download them seperately)
2. If needed install any missing dependencies ```sudo apt install ros-humble-ur-msgs ros-humble-gazebo-ros-packages ros-humble-controllers ros-humble-control ros-humble-moveit-servo``` (There may be other dependencies that you need to install if build fails in next step).
3. In top level directory of your workspace run: ```colcon build --symlink-install```
4. Source workspace
5. If the launch file is unable to find the pick_planning python nodes, you may need to make them executable with: ```chmod +x```

## Running the Demos
1. Open up two tabs in your terminal
2. In the first window: ```ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py``` (This starts Gazebo, Rviz, necessary spawn services and Moveit)
3. Both the Gazebo world and Rviz should open. You can subscribe to the camera feed and pointcloud from Rviz if desired.
4. In the second window: ```ros2 launch pick_planning pick_demo_launch.py demo_name:="demo1"``` Change "demo1" to "demo2" if you want to run the second demo

## Nodes
- **global_planner.py:** Uses OpenCV and the RGBD camera feed (with synced subscribers to depth and rgb) to select the apple closest to the camera. The global planner service takes in an image pair of the scene and returns a pose goal. There is a small amount of random noise added to the pose to simulate noisy sensor data so the initial approach will be slightly off.
- **move_arm.cpp:** Uses Moveit2 to plan/move the arm to pose goals and named target goals. This node is in C++ because the Moveit2 python api is not available in the humble Moveit2 binaries. Therefore the multi_apple_demo takes the result from the global_planner service and sends the pose goal to the move_to_pose service. It also has a service for moving to a named target pose. The joint_trajectory_controller must be active for the plans to execute succesfully.
- **local_planner.py:** Publishes TwistStamped velocity messages to the Moveit servo node with the RGBD camera and OpenCV. Drives the end effector to the center of the closest apple. It uses an exponential function for determining velocity scaling based on pixel distance of the apple from the center of the camera. The Local planner service returns when the camera is .05m in front of the apple. The Moveit servo node and forward_position_controller need to be activated for this node to work.
- **multi_apple_demo.py:** Controls the demo sequence for both demos by calling the services provided by the nodes above. It also calls the necessary services to change controllers (based on which planner is being used), start the servo node, and despawning picked apples. Moveit and Gazebo need to be running for this to work.

## Demo1 Flow
1. Switch controller to joint_trajectory_controller
2. Call global_planner service to get goal pose (images and selects pose in front of closest apple)
3. Call move_arm service to move to returned goal pose
4. Switch controller to forward_position_controller
5. Call Servo service to enable servoing
6. Call local_planner service to servo arm to final pick position
7. Despawn apple to simulate pick
8. Repeat until all apples are picked

## Demo2 Flow
1. Switch controller to forward_position_controller
2. Call Servo service to enable servoing
3. Call local_planner service to servo arm to final pick position
4. Despawn apple to simulate pick
5. Repeat until all apples are picked
