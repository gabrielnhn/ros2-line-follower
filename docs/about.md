# ROS2 Line Follower
## About:

**This is a ROS2 package** that offers a node ([follower_node](/follower/follower_node.py)) used to have a differential drive robot  run a Robotrace course, by using a camera and an implemented P controller. The rules of the course and the methods to run in it can be found at [ntf.or.jp](http://www.ntf.or.jp/archives/directory/en/game/taikai/24-rule-rt.pdf).

The package also gives the user tools to simulate the `follower_node` in a [Gazebo world](/worlds/) using a [Robotrace track model](/models/track2) and a [modified version](/models/custom_turtlebot/) of the **Turtlebot 3 Waffle Pi**, copyrights **ROBOTIS CO., LTD**, licensed under the [Apache License 2.0](/models/custom_turtlebot/LICENSE).

## License:
This package is licensed under the [MIT License](/LICENSE) by Gabriel Nascarella Hishida do Nascimento.

## Using the package:
In order to run the simulations, the user may find installation and execution insctructions in the [README](/README.md) file. The user can call the ROS2 services `/start_follower` and `/stop_follower` to operate the robot. 

If the user wants to run the node on a different robot, it is going to be necessary to edit some (if not all) of the user-defined parameters in `follower/follower_node.py`. Check the parameters guide at [parameters](/docs/parameters.md).

Beware! There are some known issues. Check out the [warnings](/docs/warnings.md) file.
