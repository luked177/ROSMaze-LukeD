My Robot starts off by scanning the front lasers between 300 and 360 degrees. This is then added to an array, any NaNs removed and the mean calculated, to give the average distance from the front to the wall.

If the wall is closer then 0.75, the robot will turn rather then advancing forward. If the distance is further then 0.75, the robot will advance forward.

The robot checks all possible degrees of the scanner and splits them into left and right arrays, these are then also NaNs removed and the mean calculated, to decide which way the robot should move. If there is a wall closer on the left side, the robot will move right.

This robot loops forever, as unfortuantely he does not detect any coloured squares.

I understood the openCV aspect, however I could not get the robot to move based on the colour.

To run this script, launch gazebo, make the script executable and run the script through the visual studio terminal in the catkin src script folder.
