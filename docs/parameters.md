## About the parameters:
* MIN_AREA

Is the minimum size for a contour to be considered the smallest element of the course: a "track mark".
This parameter should be small enough that every track mark is recognized, but large enough so dust and noises won't be considered anything.

* MIN_AREA_TRACK

Is the minimum size for a contour to be considered the line to be followed, a part of the track.
This parameter should be small enough that the track is recognized, but large enough so marks won't be tagged as part of the track, and when turning, the line is missed in order to turn faster.

* LINEAR_SPEED = 0.2

Is the robot's base speed when following the line.
This parameter should be calibrated along with `KP`. It shall be as large as possible, if still the robot does not miss the line completely.

* KP 

Is the proportional constant to be applied on the angular speed when turning. It is the constant value of the P controller.
This parameter should be calibrated along with `LINEAR_SPEED`. It shall be large enough to make the robot turn in the direction of the line, but small enough not to make it shimmy its way in the course.

* LOSS_FACTOR

If the line is completely lost, the error value shall be compensated by multiplying itself by `LOSS_FACTOR`.
To calibrate this parameter, one should examine what happens if/when the robot misses the line.

* TIMER_PERIOD

The node will analyze the images every `TIMER_PERIOD` seconds.

* FINALIZATION_PERIOD

When about to end the track (the robot indetifies the right mark(stop sign)), the robot will move for ~`FINALIZATION_PERIOD` seconds before stopping completely.

* MAX_ERROR

Is the maximum error value for which the robot is still in a straight line. It is used to check whether a right mark, a.k.a. the Stop sign, should be considered.
To calibrate this parameter, the user should analyze the error values when the robot is in a straight line.

* lower_bgr_values and upper_bgr_values

Are the BGR colour values that specify the range of colours in which the line can be.

The user should also update the function `crop_size()`. The cropped image should be big enough so the marks can be identified, but small enough so the robot can only see one part of the course(one line) at a time.

The user can also rename the topics and services used by their robot in `main()` function.