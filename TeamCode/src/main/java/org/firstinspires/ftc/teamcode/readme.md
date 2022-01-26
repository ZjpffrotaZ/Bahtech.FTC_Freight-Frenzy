# Programs #
This is a list of our three main programs, their usage, methods, and innovations

## MMMovement ##
### My Mechanum: Movement ###
It contains all the methods used to move the robot during the TeleOp and autonomous period.
The methods are:

**Methods used to define the hardware's configuration or some abstract actions**

- **defHardware:** It receives the hardwareMap to map the robot's components in our program and in
the robot configuration.
``` java
public void defHardware (HardwareMap hardwareMap)
```
- **initIMU:** It receives hardwareMap and creates the IMU configuration.
``` java
public void initIMU (HardwareMap hardwareMap)
```
- **resetEncoder:** It is self-explanatory
``` java
public void resetEncoder()
```

**Methods used to move the robot**

- **turn:** It receives the motors' force, the direction (left or right), the angle and the
 [smoother](#robot-mapping) value.
``` java
public void turn(double force, boolean isRight, double targetAngle, final double smoother)
```
- **move:** It receives the gamepad controls to move the robot using the IMU
``` java
public void move(double leftY, double leftX, double rightX, boolean slower, boolean faster)
```
- **intakeForce:** It receives the target force to the intake
``` java
public void intakeForce(double force)
```
- **shoot:** It receives the gamepad key used to shot a ring into the High Goal
``` java
public void shoot(boolean trigger)
```
- **powerShot:** It receives the gamepad key used to shot a ring into the Power Shot
``` java
public void powerShot(boolean trigger)
```
- **claw:** It receives the gamepad keys used to move the claw
``` java
public void claw(boolean halfDown, boolean totalUp, boolean totalDown, boolean open, boolean close)
```

## MMCore ##
### My Mechanum: Core ###
The main TeleOp program contains a linear OpMode and the gamepad controls that we will use.

First, it will define the Hardware and initialize the IMU, after this, it will start a loop that will
 use the methods created in MMMovement and the gamepad to move the robot

This class has no methods

## MATflowCore ##
### My Autonomous: Tensor Flow Core ###

The main autonomous program contains a linear OpMode and some methods used to drive the robot.
 The methods are: 

- **goWhite:** it moves the robot forward until it detects a white tape on the floor
``` java
private void goWhite()
```
- **goZoneA:** it contains the instruction to move the robot when there are no rings in front of it 
``` java
private void goZoneA()
```
- **goZoneB:** it contains the instruction to move the robot when there are one rings in front of it
``` java
private void goZoneB()
```
- **goZoneC:** it contains the instruction to move the robot when there are four rings in front of it
``` java
private void goZoneC()
```
- **initVuforia:** Initialize the vuforia and the webcam
``` java
private void initVuforia()
```
- **initTfod:** Initialize the TensorFlow, library that detect the objects using vuforia 
``` java
private void initTfod()
```
### computer-vision ###
Through the Tensor Flow recognitions we can recognize either if there is or there isn't any ring at
the starter stack. Then, we can move to the target zone according to the game rules.
- 0 Rings detected = Zone A
- 1 Ring detected  = Zone B
- 4 Rings detected = Zone C

This is the program's part that is responsible to recognize the number of rings detected by our
robot and show it in the Driver station's screen  

``` java
if (opModeIsActive()) {

    while (opModeIsActive()) {
        sleep(1500);
        recognitions = tfod.getRecognitions();
        tfod.shutdown();

        if (recognitions.size() == 0) {
            telemetry.addData("Target Zone", "A");
        } else {

            for (Recognition recognition_item : recognitions) {
                recognition = recognition_item;
            }

            if (recognition.getLabel().equals("Single")) {
                telemetry.addData("Target Zone", "B");
            } else if (recognition.getLabel().equals("Quad")) {
                telemetry.addData("Target Zone", "C");
            }

        }
        telemetry.update();
    }
}
```

## Smoother ##

Different than everything else, this is a part of the program that appears in many movement methods,
 it is responsible to make our robot slow down after a movement, making it more precise.
 it is extremely useful and can be implemented in two ways:
- **Simpler:**
To explain the way it works, I will use the example present in our *turn* method.

The smoother value is with the subtraction between the target angle of our turn with the current
robot's angle. The result will be divided with the Smoother constant that, in most cases, will be
defined inside the method.

``` java
(angle - currentAngle) / smoother
```

With the smoother value on, we can use it with the maximum desired force, let's pretend that it's
0.5, we can use an if-else to define the motor force. If the desired force is smaller, we use it,
if isn't, we use the smoother calculation.

``` java 
SmootherCalc = (angle - currentAngle) / smoother; 

if (SmootherCalc > desiredForce) {
    movementMotor.setPower(desiredForce)
} else {
    movementMotor.setPower(SmootherCalc)
}
``` 

- **Harder:**
This way is only used in the TeleOp movement method and is harder to implement, but I will try to
simplify it in here

To implement this, we will need to have:
1.  The current motor force
1.  The last updated motor force
1.  The smoother constant

With this, we will compare if the absolute value of the difference between the current and last
force is bigger than the smoother constant

``` java
Math.abs( lastForce - currentForce ) > smoother
``` 

if it isn't we can directly define the force to the motors, but if isn't, we will need to change the
value with some logic

if the last force was bigger than the current force, we will use it less the constant smoother as
the motor force, else, we will use the last force plus the smoother as the force

``` java
if (Math.abs(lastForce - currentForce) > smoother) {
    if (lastForce > currentForce) {
        movementMotor.setPower(lastForce - smoother)
    } else {
        movementMotor.setPower(lastForce + smoother)
    }
} else {
    movementMotor.setPower(currentForce)
}
``` 

## PID ##
PID is an abbreviation of proportional-integral-derivative and it is used to prevent errors that
can happen during the movement. We use it with the angular sensors integrated in the IMU. The PID's
components are:

- **Proportional:** Its responsibility to create an variable that get the difference between the
current angle and target angle. This value is multiplied by a constant kP and summed with the
target movement force
``` java
final double kP = 1;

while(pid.isActive){
    error = angle - currentAngle;
    p = error * kp;
}
```
- **Integral:** Its responsibility to get all the error values and sum. It is especially useful
when our target is different than zero (the proportional will not be able to solve this alone) or if the
force that the proportional gives is not enough. The integral value is multiplied by kI
``` java
final double kP = 1;
final double kI = 0.4;
             
while(pid.isActive){
    error = angle - currentAngle;
    p = error * kp;
    i += error * kI;    
}
```
- **Derivative:** Its responsibility to avoid the integral to get to an enormous value, this can
happen when the error is equals to zero. To calculate this we will need to save the last error and
subtract it with the current error, the result will be multiplied by kD
``` java
final double kP = 1;
final double kI = 0.4;
final double kD = 1.5;
double lastError;

while (pid.isActive){
    error = angle - currentAngle;
    p = error * kp;
    i += error * kI;
    d += (lastError - error) * kd;
}
```

We get the three values that we got before and sum them to get an fully functional PID. This logic
is in all over our code, especially in the autonomous period. You can see it working in the movePID
, movePIDSide and the goToWhite methods.
 
# Robot mapping #
Robot Mapping help us with cable management, it is used to avoid unnecessary changes in the cables
 and the complications that come with it.

We had put some colorful tags in our robot's cables to help us identify it more easily.
## HUB 1 / Blue tag ##
### Motors ###
| Port | Name          | Color  |
|------|---------------|--------|
|  0   | arm_motor     | Yellow |
|  1   | intake_motor  | Blue   |
|  2   | shooter_motor | White  |

### I2C 0 ###
| Port | Name          |
|------|---------------|
|  0   | imu           |
|  1   | sensor_color  | 

## HUB 2 / Pink tag ##
### Motors ###
| Port | Name          | Color   |
|------|---------------|---------|
|  0   | FL            | Yellow  |
|  1   | FR            | Blue    |
|  2   | BL            | White   |
|  3   | BR            | Green   |

### Servos ###
| Port | Name          |
|------|---------------|
|  0   | claw_servo    |
|  1   | trig_servo    |

# Gamepad #
We configured only one gamepad, we had some brainstorms and decided that only one pilot would be
better than two.

the gamepad mapping is:

| Button | Function          |
|--------|-------------------|
| X      | Open the claw     |
| Y      | Lift the arm      |
| A      | Lift down the arm |
| B      | Close the claw    |
| RT     | Speed up the robot                   |
| RB     | Shoot the rings (Power Shot)         |
| Left trigger   | Shoot the rings (High Goal)  |
| DPad up        | Start/stop the intake        |
| DPad down      | Lift down the arm halfway    |
| Right joystick | Control the robot's turns    |
| Left joystick  | Control the robot's movement |
