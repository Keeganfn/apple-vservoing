## Moveit2 Apple Picking Demo
This repo uses the ur5e with an attached RGBD camera in gazebo to find and pick "apples" (red circles). It uses Moveit2 for initial apple approach and Moveit2 Servo for the final pick approach. OpenCV is used to identify "apple" locations with the RGBD stream.

The purpose of this demo is to simulate real world approaches to robotic apple picking where an initial pick approach is chosen, and the final pick approach uses finer grain visual servoing. This helps mitigate problems that occur due to bad/noisy sensor data from an initial scan of the environment.  

### Two planning stages: 
- **global_planner node:** Uses the RGBD camera feed to find the closest apple to the camera on the end effector. It then finds the desired goal pose (with noise) and uses moveit to plan/move to the pose goal. 
- **local_planner node:** Uses the RGBD camera feed to find the closest apple and sends Twist velocity commands to servo the arm to the centerpoint of the apple. Stops a predefined distance in front of the apple to simulate a pick. Uses Moveit Servo to control the arm.


### Two Demos:
- **demo1 (full):** Environment has 5 apples that need to be picked. Demo first uses the global planner for initial approach followed by the local planner for final approach. Apples should despawn after they are "picked".
- **demo2 (local):** Environment has 5 apples that need to be picked. Demo only uses visual servoing in the local planner to approach apples. This is meant to demonstrate the visual servoing controller over larger distances. Apples should despawn after they are "picked".

## Installation
1. Clone this repo into your workspace (local changes had to be made to the UR repos so do not download them seperately)
2. If needed install any missing dependencies ```sudo apt install ros-humble-ur-msgs ros-humble-gazebo-ros-packages ros-humble-controllers ros-humble-control ros-humble-moveit-servo``` (There may be other dependencies that you need to install if build fails in next step).
3. In top level directory of your workspace run: ```colcon build --symlink-install```
4. Source workspace

## Running the Demos
1. Open up two tabs in your terminal
2. In the first window: ```ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py```
3. Both the Gazebo world and Rviz should open.
4. In the second window: ```ros2 launch pick_planning pick_demo_launch.py demo_name:="demo1"``` Change "demo1" to "demo2" if you want to run the second demo
