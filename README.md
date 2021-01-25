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

## Behavoir Programming: Wall Follower

#### Problem Description
At a high level, this behavoir wants the turtlebot to recognize a wall and drive along side this obstacle. This means that it has to drive straight for an  unset amount of distance until it finds a wall and then must be able to constantly turn and adjust, so that the robot never collides with the wall while tracing it.

#### Initial Approach
For this implementation, I had initially attempted to use the *scan* and *cmd_vel* topic to calculate how far way from the wall the turtlebot was, and have it move forward until it was within desired range of it. Once this happened, a flag would be set that would indicate that the robot had found a wall and may now start adjusting its angular velocity to "follow" the wall. This was a decent implementation but the algorithm that was used to adjust the angular velocity wasn't satisfactory and led to over-adjusting and veering either of course or into the wall. 

#### Current Approach
Unlike my initial approach, I abandoned the idea of using a "wall flag" and just used the topics of *cmd_vel* and *scan* to constantly move and adjust the turtlebot. In order to do this, I had my *scan* subscriber determine the robot's current distance from the wall by focusing on the wall's position relative to the robot's "left". While the robot moved forward, this value would be used to constantly adjust the robot's angular velocity, resulting in it moving in a clockwise motion. Just utilizing the difference between the current and desired distance from the wall was inadequate for adjusting the robot's angle, so I integrated the most recent difference value and a summation of previous differences, modeling the equation after the PID control function.

I broke down a lot of functionality into helper functions to help organize and abstract the work of this project. Look below for detailed explanations of each function.

##### Helper Function: calc_control
This function determines the angular velocity for the turtlebot. It takes in the difference between the current and desired location of the robot and references the global values of prevAdjust and sumAdjust, where the most recent previous difference value and summation of all previous difference are stored. With these values, and the constants of kp, dt, ti, and td (determined from trial and error and intuition) an equation is constructed that is mostly influenced by the current offset and is slightly adjusted by the difference of the current and previous differnce and the sum of all differences. This equation is modeled after examples of PID controllers, with constants that for allow for the robot to turn without over adjusting and going haywire or under adjusting and getting stuck in the wall

##### Helper Function: scan_callback
This function gathers the data from the *scan* subscriber, providing the necessary data to the *calc_control()* helper. With this value, it uses the *cmd_vel* publisher to change the turtlebot's linear velocity to a constant 0.3 and it's angular velocity to whatever value determined from *calc_control()*

##### Helper Function: run
This function calls sleep to help prevent the turtlebot from overadjusting

#### Challenges
One of the major challenges I faced was getting the turtlebot to properly adjust itself, as I felt that the equation being used was inadequate for a satisfactory or perfect solution. With its current setup, the robot would follow the wall from farther away than desired and slowly get closer as the behavoir ran. This "ramp up" time suggests that the robot would struggle to follow a more complicated wall. The two issues that lead to this challenge is the difficulty of properly interpreting the *scan* data and the values for the constants within *calc_control()*, as more appropriate values would prevent the robot from overturning and subsequently overadjusting where it continously turns left and right.

#### Future Work
If I had more time, I would investigate the data from *scan* more to properly interpret it and find a more adequate approach for determing the robot's angle and distance from the wall. In order to do this, I should have the robot drive in a square and see how the *scan* data changes in critical positions like corners, where the biggest adjustment in angle is needed. I would also love to implement functionality that determined if the robot was stuck and have it free itself, as this issue can always be a problem and is a plausible real world issue.

#### Takeaways
* How to utilize and partition information from the *scan* topic. It is necessary not only knowing how to seperate the scan messages into left, right, front, etc segments, but it is even more important to know what those values mean and translate to the robot's environment. I can see this information being critical for dynamic movement and awareness that will be necessary for future projects and complicated tasks in dynamic environments
*  Virtual simulations are surprisingly inconsistent, and it is important to ensure that the testing environment is refreshed and that ROS is properly functioning. This can be easily remedied by checking on the status of vital background roscore tasks within the terminal, ensuring no errors arise. This sanity check helps ensure that changes that are being made are properly reflected in the environment and that since there is no "perfect percision" in these simulations, being able to dynamicaly adjust the robot is vital.

#### Recording
See **wallFollower.mp4** for recording of simulation

## Behavoir Programming: Person Follower

#### Problem Description
At a high level, this behavoir wants the turtlebot to recognize a person (closest object) and drive to this object, stopping at a desired distance while facing directly towards it. This means that the robot has to sit at rest if no object is within range and later determine where the object is in relation to itself so that it can turn and approach it.

#### Initial Approach
For this implementation, my initial approach is my current approach. See below for information.

#### Current Approach
This implementation utilizes the *cmd_vel* and *scan* topic. It is mainly dependent on conditionals based on data from the *scan* subscriber. After partitioning this data in directions, it determines if any direction has detected and object within the scanner's range. If not, nothing happens. If there is, the robot determines the direction of the object and moves towards it, constantly checking and adjusting its angle if it becomes misaligned. Once it reaches its desired distance from the object the robot stops. 

I broke down a lot of functionality into helper functions to help organize and abstract the work of this project. Look below for detailed explanations of each function.

##### Helper Function: scan_callback
This function gathers the data from the *scan* subscriber. With these values, it uses conditionals to determine if the scanner has picked up any nearby objects (The value of the scanner defaults to 10 if there is nothing in range). If there is such and object in range, it determines whether or not it is more efficient to turn left or right by seeing if the object was detected in the robots front-left sensor. This prevents extranious movement from the robot and having it do a small turn instead of an unnecessary full rotation. (To prevent this causing an endless feedback loop of turning left and right, this conditional sets a flag that can only be changed once the robot is properly oriented) Once the difference between the left scanner and the right scanner is within a range of 0-0.2 (while also not being their default values of 10), the robot is adequately oriented and starts approaching the object, stopping to readjust if the robot becomes misaligned.

##### Helper Function: run
This function calls sleep to help prevent the turtlebot from overadjusting

#### Challenges
The challenges I faced while programming this behavoir can be perfectly encapsulated by the conditionals I have set up to handle the edge cases and issues I came across. Feedback loops became prevelant when I tried to make the turtlebot's turning more efficent, as I was setting the value of the directionMod to equal 1 at the beggining of *scan_callback()* which would cause the robot to constantly turn left and right. This was remedied by removing this initialization and treating the variable like a flag that can only be modified after the robot was oriented. Basic trial and error helped me determine other issues my implementation would come across and were easy to solve.

#### Future Work
To improve the turtlebot's behavoir, I would have the robot have its velocity be dynamically calculated based on its distance from the object and how much it needs to adjust its angle. Another improvement would be to use the data from *scan* to calculate an exact angle, is this will have a multitude of uses for future behavoirs and lead to more exact orientation of the robot.

#### Takeaways
* This behavoir provided a deeper understanding to how *scan* works and its data. If I had done this behvoir before the wallFollower, I feel like it would've gone a lot smoother. It will allow me to use *scan* to properly understand the robot's surroundings.
* A lot of possible edge cases and issues were exposed to me. Being aware of these issues will help in future behavoirs as my implementations can be conscious of these problems and lead to quicker debugging and bug prevention.

#### Recording
See **personFollower.mp4** for recording of simulation