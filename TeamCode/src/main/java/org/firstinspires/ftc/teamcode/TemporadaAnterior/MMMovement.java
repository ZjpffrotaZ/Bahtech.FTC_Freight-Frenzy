package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import static java.lang.Thread.sleep;
@Disabled
public class MMMovement {

    // SENSORS
    TouchSensor touchSensor;

    // Abstract values
    private final double[] force = new double[4];
    private final double[] lastForce = new double[4];
    private int up = 0;
    private int down = -1000;

    // values used in the movement
    private double internal = 1;
    private double external = 1;

    // MOVEMENT MOTORS
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;

    // SECONDARY MOTORS
    private DcMotor shooterMotor;
    private DcMotor intakeMotor;
    private DcMotor armMotor;

    // SERVOS
    private Servo shooterServo;
    private Servo clawServo;

    // IMU variables
    private BNO055IMU imu;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    /*METHODS USED AT THE START*/
    // Define the robot's hardware
    public void defHardware(HardwareMap hardwareMap) {

        // Motors used in the movement
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        // Motors used to perform actions
        armMotor = hardwareMap.dcMotor.get("arm_motor");
        shooterMotor = hardwareMap.dcMotor.get("shooter_motor");
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");

        // Servos used to perform actions
        shooterServo = hardwareMap.servo.get("trig_servo");
        clawServo = hardwareMap.servo.get("claw_servo");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);

        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
    }

    // Init the IMU variables
    public void initIMU (HardwareMap hardwareMap){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    /*METHODS USED IN THE TELE-OPERATED PERIOD*/
    // Move the robot with an absolute front
    public void moveByArena(double leftY, double leftX, double rightX, boolean slower, boolean faster) {

        // Create a variable using the IMU
        double angle = getCurrentDegree();

        // Will get the IMU value and change the "internal" and "external" variables
        if (0 < angle && angle <= 90) {
            internal = -(angle / 45 - 1);
            external = 1;
        }
        else if (90   < angle && angle <= 180) {
            internal = -1;
            external = -((angle - 90) / 45 - 1);
        }
        else if (-90  < angle && angle <= 0) {
            internal = 1;
            external = angle / 45 + 1;
        }
        else if (-180 < angle && angle <= 90) {
            internal = (angle + 90) / 45 + 1;
            external = -1;
        }

        moveByRobot(leftY, leftX, rightX, slower, faster);
    }

    // Move the robot using it's front
    public void moveByRobot(double leftY, double leftX, double rightX, boolean slower, boolean faster) {

        // Defines the smoother for the movement
        final double smoother = 0.1;

        // Add to the vector the forces of the gamepad and the angle variables
        force[0] = leftY * internal + leftX * external + rightX;
        force[1] = leftY * external - leftX * internal + rightX;
        force[2] = leftY * external - leftX * internal - rightX;
        force[3] = leftY * internal + leftX * external - rightX;

        // See if the difference of the last force and the current one is bigger than the
        // smoother value, if it is, it will change gradually to not damage the motors
        for (int s = 0; s <= 3; s++) {
            if (Math.abs(lastForce[s] - force[s]) > smoother) {
                if      (lastForce[s] > force[s]) force[s] = lastForce[s] - smoother;
                else                              force[s] = lastForce[s] + smoother;
            }
        }

        // Save the used force in variables to get the difference
        lastForce[0] = force[0];
        lastForce[1] = force[1];
        lastForce[2] = force[2];
        lastForce[3] = force[3];

        // If the right bumper is pressed, the robot will move slower
        if (slower) {
            FR.setPower(force[0] / 4);
            BR.setPower(force[1] / 4);
            FL.setPower(force[2] / 4);
            BL.setPower(force[3] / 4);
        }
        // If the right bumper is pressed, the robot will move faster
        else if (faster) {
            FR.setPower(force[0]);
            BR.setPower(force[1]);
            FL.setPower(force[2]);
            BL.setPower(force[3]);
        }
        // If neither bumper is pressed, the robot will move half the speed
        else {
            FR.setPower(force[0] / 2);
            BR.setPower(force[1] / 2);
            FL.setPower(force[2] / 2);
            BL.setPower(force[3] / 2);
        }
    }

    // Define the intake force
    public void intakeForce(double force) { intakeMotor.setPower(force); }

    // Define the shooter force
    public void shotForce(boolean stronger, boolean weaker) {

        if (stronger) {
            shooterMotor.setPower(0.725);
        } else if (weaker){
            shooterMotor.setPower(0.637);
        }
    }

    // Activate or deactivate the intake
    public void controlIntake(boolean button) throws InterruptedException {

        if (button && getIntakeForce() != 0){
            intakeMotor.setPower(0);
            sleep(250);
        }
        else if (button && getIntakeForce() != 0.8){
            intakeMotor.setPower(0.8);
            sleep(250);
        }
    }

    // Shoot the rings to the high goals
    public void shoot(boolean rt, boolean lt) {

        if ((rt || lt) /*&& armMotor.getCurrentPosition() < down + 100*/) {
            shooterServo.setPosition(0.8);
        } else {
            shooterServo.setPosition(0  );
        }
    }

    // Move the robot's claw
    public void claw(boolean totalUp, boolean totalDown, boolean open, boolean close) {

        final double force = 0.7;

        if (open) {
            clawServo.setPosition(1);
        } else if (close) {
            clawServo.setPosition(0);
        }

        if (totalUp){
            armMotor.setTargetPosition(up);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(-force);
        }
        else if (totalDown) {
            armMotor.setTargetPosition(down);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(force);
        }
        if (!armMotor.isBusy()){
            armMotor.setPower(0);
        }
    }
    public void calibrateArm(){

        final int difference = -900;

        while (!touchSensor.isPressed()) {
            armMotor.setPower(0.3);
        }

        armMotor.setPower(0);
        up = armMotor.getCurrentPosition();
        down = up + difference;
    }

    /*Getters and setters*/
    public double getFlForce(){ return FL.getPower(); }
    public double getFrForce(){ return FR.getPower(); }
    public double getBlForce(){ return BL.getPower(); }
    public double getBrForce(){ return BR.getPower(); }
    public double getIntakeForce(){ return intakeMotor.getPower(); }
    private double getArmEncoder() { return armMotor.getCurrentPosition(); }
    private double getCurrentDegree() {return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;}
}