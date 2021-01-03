* The track used in the simulations (`models/track2`) may not be in real life proportions.

* Calibration of the parameters by the user is quite important in order to make the node work. 

* If the user interrupts the execution of the node **without** calling the `/stop_follower` service, if the robot has no built-in feature of checking whether velocity command messages have been sent recently, the robot will keep moving according to its last Twist message. **If you find a way to solve this problem, feel free to make a PR or contact me by [e-mail](mailto:gabrielnhn@ufpr.br).**  
