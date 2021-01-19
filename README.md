# warmup_project

## Behavoir Programming: Driving in a Square

#### Problem Description
This behavoir is self explanatory, as I am tasked to program the turtlebot so that it *Drives in a Square*. This means that it has to drive straight for a set amount of distance and be able to turn a perfect 90 degrees 4 times in a row, so that the robot arrives back to where it started.

#### Initial Approach
For this implementation, I had initially attempted to use the *odometer* to calculate how long the turtlebot had traveled and had planned to monitor this value in order to determine when the robot should stop moving forward and turn. After struggling to properly set up the subscriber/publisher relationship and running into errors with formatting the data appropriately for the odometer publisher, I decided to instead use time and *cmd_vel* to control my bots movement.

#### Current Approach
By using *cmd_vel*, I was able to control the linear and angular velocity of the turtlebot. In order to properly use this topic, I had to first initialize *cmd_vel* and set up a publisher that I can reference within my class. Since we want to travel set consistent distances when driving in a square, I made the distance into referencable global constant. A subscriber to the *odometer* is also initialized and is just use to output and monitor the current position of the robot. From there, it was as simple as telling the robot to drive straight and turn four times until it had completed a square and returned to its intiial spot.

I broke down a lot of functionality into helper functions to help organize and abstract the work of this project. Look below for detailed explanations of each function.

##### Helper Function: manageMovement
Simple function that is attatched to *odometer* subscriber that continously prints the current position and sleeps


##### Helper Function: clearVel
This function ensures that all linear and angular values within the vel_msg are zero'd out by manually setting their values.

##### Helper Function: straight
This function sets the turtlebot's linear velocity of x to a speed of .25. With this set speed, we can record how much time has passed since the robot started moving in order to calculate how far it has traveled, by multiplying the amount of time that has passed by the current velocity. Once this value has reached the desired set distance, the velocity is set to zero and the function ends.

##### Helper Function: turn
This function sets the turtlebot's angular velocity of z to a speed of .3 radians. With this set speed, we can record how much time has passed since the robot started turning in order to calculate its current angle. In order to ensure it has turned 90 degees, I set its desired value to PI/2, as that is the value of 90 degrees in radians. Similar to the approach of the straight function, I utilized this set speed and the amount of time that had passed in order to determine when the robot had turned adequately enough and to set the velocity to zero.

##### Helper Function: run
This function handles looping between driving straight and turning, calling the other helper functions to do so. It also calls sleep to help prevent the turtlebot from jerking and misaligning itself when switching from different movements.

#### Challenges
One of the major challenges I faced was getting the robot to be properly aligned. If I used too high of a velocity, the time calculations used to measure the distance/degrees would be inaccurate and would tell the robot to stop too early. Another issue I faced was that the robot would randomly jerk or move before doing its next action, misaligning it and runining its path. My current solutions to these issues were to use a lower velocity value and to make the program sleep in between transitions in hopes to mitigate these issues. For future implementations, a solution to this issue would be to use the *odometer*, as the inacuracy of these time calculations would not be present and because random jerks and misalignment can be rectified through the exact position/orientation achievable with this approach.

#### Recording
See **drivingSquareVideo.mp4** for recording of simulation