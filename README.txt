				*****************************
				** Description of the exam **
				*****************************

MOTOR (34:1 Metal Gearmotor 25Dx67L mm HP 12V with 48 CPR Encoder): 

Since the in the datasheet the specified Stall Torque varied for two differet voltages(12V and 6V), the considered for this practice was 12V, thus the Stall Torque chose was 120 oz·in which convert to N·m is 0.847 N·m and this is the maxTorque put in each RotationalMotor.
The next attribute to modify was the maxVelocity, in this the formula proportionated was useful:

			      MaxVel = (RPM·2·PI)/(60)

which in this case the datasheet indicates that the RPM is equal to 290, therefore the MaxVel = 30.36.

POSITION SENSOR: 

For the position sensor I based on the formula provided:

			   resolution = 2·PI/(countersPerRevolution)

Therefore, the resolution put in PositionSensor attribute was 0.003848, the noise part was left in 0.

DISTANCE SENSOR (VCNL4040):

The distance sensors contain a shape of a cube of dimension 0.01 x 0.01 x 0.01. In the lookupTable attribute basing on the datasheet, the sensor to reproduce is a proximity sensor that operates with a range from 0mm to 200mm, therefore in the lookupTable the the values put were: x = 0, y = 0, z = 0. The datasheet also states that the resolution of the sensor is of 16-bit, for instance using the formula given in class:

				resolution = 2^(bit) - 1
Giving as result resolution = 65535 which will be put together with the maximum distance that the sensor can read (0.2m). 

				x = 0    y = 0     z = 0
				x = 0.2  y = 65535 z = 0


This lookupTable for both distace sensor.In this way sensing from 0 (the start of the sensor) to the end of it.

Also, a function was created, this in order to "convert" the bits to centimeters, and so the functions returns a value in bits, this with the intention of justa giving the parameter, this was possible thanks to a three rule. This also was helpful for the point where is requiered to stop and move the robot at 17 cm. So that the three rule is like this:

			bitsToCentimeters = (desiredCentimeters · 65535 bits)/(20 cm)

In this case for 17cm the result was 55704.75 bits, however the functions thanks to the rule of three can compute any bit at any centimeter. 

GUN DISTANCE SENSOR:
The gun distance sensor is a box shape with dimensions 0.005m x 0.003m x 0.015m which is placed before the detector distance sensor and is green.
According to the requested instructions it is a 10 bits distance sensor, with a min range of 10cm and max range of 2m. Therefore in order to compute the values that are to be placed in webots was done the next, given the next equation:

				resolution = 2^(bit) - 1
The resolution can be computed and given that is the bits are 10, the result for the resolution is: 
				resolution = 2^(10) - 1
				resolution = 1023
Therefore the lookUpTable is:

				x = 0    y = 0     z = 0
				x = 0.1  y = 0     z = 0
				x = 2    y = 1023  z = 0

which has the minimum range of 10cm and the maximum range of 2m.
It is worth to say that the formula:

			resolution = maxRange - minRange/ (minStep) 

was not used since the minStep was not provided.

DETECTOR DISTANCE SENSOR:
The detector distance sensor is a box shape with dimensions 0.005 x 0.003 x 0.02 which is placed at the top of the post and is of color red. 
According to the requested instructions it is a 8 bits distance sensor, with a minRange of 5cm and a maxRange of 40cm. Therefore, in order to compute the values that are to be placed in webot was done the next, given the equation: 

				resolution = 2^(bit) - 1
which is: 
				resolution = 2^(8) - 1
				resolution = 255

Therefore the lookUpTable is:

				x = 0     y = 0     z = 0
				x = 0.05  y = 0     z = 0
				x = 0.4   y = 255   z = 0

which has the minimum range of 5cm and the maximum range of 40cm. 
It is worth to say that the formula:

			resolution = maxRange - minRange/ (minStep) 

was not used since the minStep was not provided.



JUSTIFICATION: 

The car now turns on in the same position, I've fixed the constant move with a counter for the left and right distance sensor, I was having troubles with the movement to the right and the left, however I've found the perfect combination so that it moves to the right and left correctly.
Sometimes the robot crashes with some obstacles since the sensors are not able to detect them and the robot may get there just a moment and there it goes out there, maybe adjusting the DISTANCE_OBSTACLE Macros to a higher value would correct this and therefore the robot will be able to rotate even before the 17cm (19cm were the best).  


INSTRUCTIONS: 

The robot starts 1 meter apart from the 0,0,0 point respect to the floor, then dodge one obstacle, once the robot has dodge that obstacle start to approximate to the "Enemy", and as soon the robot detects it the gun turns to the position of the enemy and the message 'THATHATHA' is displayed, the first position for the enemy is 40 cm, the second position is 30cm and the third is 20cm.
In order to proof the different 'levels' of distance of the enemies it is necessary to restart the world it hasn't been restarted and move a little bit the violet object. 
However, it has been seen that at the first when it initializes and detects the first enemy (which was the last mentioned) if the object (violet enemy) is moved the gun will move to the position that the object (enemy) is as soon as the other sensor, the one that constantly turns, detects the enemy.

NOTE: 

Might be the case that the at the beginning a WARNING appears, this is due to a triangle shape that I am using to cover the robot, however if this shape is quit the WARNING should disappear without affecting the mobility of the robot.

I added some irrelevant mass (0.1) to each wheel and roller that is why at the beginning the bunch of WARNINGS, but it should work correctly.

Also, at the moment of opening some WARNINGS with respect to the mass and density appear, since I've added some mass (0.1) to each roller of each of three wheels, and that's why those messages appear at the beginning.  

SINCE THE MOTORS FOR THE 'POST' (gun and detector) WERE NOT SPECIFIED I USED THE SAME SPECIFICATIONS OF THE WHEELS (MOTOR (34:1 Metal Gearmotor 25Dx67L mm HP 12V with 48 CPR Encoder) and also the encoder that comes with the motor
 

