The meshes folder was used to experiment with loading in models to Gazebo that we created in Blender

How to run the code:
The catkin_tester directory made by James is needed to run the code

Terminal 1)
cd catkin_tester
source devel/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch

Terminal 2)
cd catkin_ws
source devel/setup.bash
cd src
rosrun turt_control botControl

Terminal 3)
cd catkin_ws
source devel/setup.bash
rosrun rviz rviz
add -> by topic -> /map