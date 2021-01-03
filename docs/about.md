# ROS2 Line Follower
**This is a ROS2 package** used to control a differential drive robot in order to follow a Robotrace course, by using a camera and a P controller. The rules of the course and the methods to run in the course can be found [here](http://www.ntf.or.jp/archives/directory/en/game/taikai/24-rule-rt.pdf).

This package is licensed under the [MIT License](/LICENSE) by Gabriel Nascarella Hishida do Nascimento.

The package also gives the user tools to simulate the `follower_node` on a [modified version](/NOTICE) of the **Turtlebot 3 Waffle Pi**, made by **ROBOTIS CO., LTD**, licensed under the Apache License 2.0.

In order to run the simulations, the user may find installation and execution insctructions in the `README.md` file. The user can call the ROS2 services `/start_follower` and `/stop_follower` to operate the robot. 

If the user wants to run the node on a different robot, it is going to be necessary to edit some (if not all) of the user-defined parameters in `follower/follower_node.py`. Check the parameters guide at `parameters.md`.

Beware! There are some known issues. Check out the `warnings.md` file.