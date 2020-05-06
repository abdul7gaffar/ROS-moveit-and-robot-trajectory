code devloped by abdul gaffar abdul khadeer, QMUL
to run the package:

0) download and install the following packages:
-git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel
-git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel

1) unzip the ar_week8_test.zip folder in the src floder of your catkin worksapce

2)build the catkin workspace

3) run the following commands on differnt terminals-
--roslaunch panda_moveit_config demo.launch
--rosrun ar_week8_test square_size_generator.py
----rosrun ar_week8_test move_panda_square.py
rosrun rqt_plot rqt_plot

