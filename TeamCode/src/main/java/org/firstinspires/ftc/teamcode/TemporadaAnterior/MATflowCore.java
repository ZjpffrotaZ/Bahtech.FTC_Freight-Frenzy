package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Disabled
@Autonomous(name ="MATflowCore", group = "official", preselectTeleOp = "MMCore")
public class MATflowCore extends LinearOpMode {

    private final double[] force = new double[4];
    private final double[] lastForce = new double[4];

    // Hardware variables
    ColorSensor colorSensor;
    TouchSensor touchSensor;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    BNO055IMU imu;
    final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // Secondary motors
    DcMotor armMotor;
    DcMotor intakeMotor;
    DcMotor shooterMotor;

    // Servos
    Servo clawServo;
    Servo trigServo;

    // Encoder constants
    final int TICKS_REV = 1120;
    final double GEAR_REDUCTION = 1.25;
    final double WHEEL_CIRCUMFERENCE_CM = Math.PI*10.16;

    // Arm variables
//    int up = 0;
//    int down = -1000;

    // Defines the constant multipliers to the PID
    double kp = 1;
    double ki = 0.4;
    double kd = 1.45;
    final double k = 35;
    final long updateRate = 75L;
    double angle = 0;

    //Detection variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    Recognition recognition;
    ElapsedTime runtime = new ElapsedTime();

    // Vuforia key
    public static final String VUFORIA_KEY =
            "Ad0n+XX/////AAABmQ/41s5hKkoQl9XvVGzFatosnvXWi3lcaK406bSM6BCiUoPYCNo83nOVmmi0PcL1v6+gDcdnZddtNmRaYSKGqMhsoHzczhJbbzJa6vsQPZ6Bzzs/9icQySfGBy4wUXNFBysun4H4G2qQEeaWP/PwNu7FkREo28S5DXYbk5G1SToOk/6MiOG4v8zuy1WvA2Mrg2gbJMlj181zI7Wj+CYJXDUpE2ugflgq9iDDzBCJGb0r1/sEwkqpBq/u3G8F2iPK5kRxLSsz2yB8AjpiLZlQJoZ9NpIwkEyuS9i6AVc9lnPTMst+gortmeJ+ThBBhscsJ55tdDf8yLn11cQgJSyrWBr9+9HIyvRc3ixR/og6y/Mc";

    @Override
    public void runOpMode() {

        List<Recognition> recognitions;

        // Find motors in the hub
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        // Wobble grabber
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        // Servos
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        trigServo = hardwareMap.get(Servo.class, "trig_servo");

        // Sensors
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

        // Left motor's reverse direction
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        // Set motors zero power behavior
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize IMU, Vuforia and TensorFlow. Calibrate the arm motor
        initIMU();
//        calibrateArm();
        initVuforia();
        initTfod();

//        // Lift the arm and close the claw
//        moveArmAuto(false);
        sleep(1500);
        moveClawAuto(false);
//        sleep(3000);
//        moveArmAuto(true);

        // Show a message to press play

        while (!gamepad1.back){
            if (gamepad1.dpad_up){
                kp +=0.01;
                sleep(100);
            }

            if (gamepad1.dpad_down){
                kp -=0.01;
                sleep(100);
            }

            if (gamepad1.dpad_right){
                ki +=0.01;
                sleep(100);
            }

            if (gamepad1.dpad_left){
                ki -=0.01;
                sleep(100);
            }

            if (gamepad1.right_stick_button){
                kd +=0.01;
                sleep(100);
            }

            if (gamepad1.left_stick_button){
                kd -=0.01;
                sleep(100);
            }
        }

        telemetry.addData(">", "Press Play to start!");
        telemetry.update();
        waitForStart();

        // Activate TensorFlow
        if (tfod != null) {
            tfod.activate();
        }

        runtime.reset();

        // Start the programs to the determinate zone
        while (opModeIsActive()) {
            sleep(1500);

            // Get the size of the initial ring stack
            recognitions = tfod.getRecognitions();

            if (tfod != null) {
                tfod.shutdown();
            }

            if (recognitions.size() == 0) {
                telemetry.addData("TFLOW", "No object detected");
                telemetry.addData("Target zone", "A");
                telemetry.update();
                goZoneA();
                break;
            } else {

                for (Recognition recognition_item : recognitions) {
                    recognition = recognition_item;
                }

                if (recognition.getLabel().equals("Single")) {
                    telemetry.addData("Target zone", "B");
                    telemetry.update();
                    goZoneB();
                    break;
                } else if (recognition.getLabel().equals("Quad")) {
                    telemetry.addData("Target zone", "C");
                    telemetry.update();
                    goZoneC();
                    break;
                }
            }
            telemetry.update();
        }
    }

    // Program used to reset the movement encoders
    private void resetEncoder() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move the robot to the white tape
    private void goWhite(boolean isForward) {
        double speed = 0.8;
        int whiteTape = 500; //Alpha
        int redTape = 110;   //Based on RGB
        double p;
        double currentAngle;
        double error;

        while (colorSensor.alpha() < whiteTape && colorSensor.red() < redTape) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            error = angle - currentAngle;

            p = (error * kp) / k;

            if(isForward) {
                FL.setPower(speed-p);
                FR.setPower(speed+p);
                BL.setPower(speed-p);
                BR.setPower(speed+p);
            } else {
                FL.setPower(-speed-p);
                FR.setPower(-speed+p);
                BL.setPower(-speed-p);
                BR.setPower(-speed+p);
            }
        }
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

//    private void goBlue() {
//        double speed = 0.7;
//        int blueTape;
//        int redTape;
//
//        double p;
//        double currentAngle;
//        double error;
//
//        while (colorSensor.alpha() < blueTape && colorSensor.red() < redTape) {
//            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//
//            error = angle - currentAngle;
//
//            p = (error * kp) / k;
//
//            FL.setPower(speed-p);
//            FR.setPower(speed+p);
//            BL.setPower(speed-p);
//            BR.setPower(speed+p);
//        }
//
//        FL.setPower(0);
//        BL.setPower(0);
//        FR.setPower(0);
//        BR.setPower(0);
//    }

    // Program run when there's no rings in front of the robot
    private void goZoneA() {

        //Delivery 1st wobble goal
        goWhite(true);
//        moveArmAuto(false);
        sleep(500);
        moveClawAuto(true);
        sleep(500);
//        moveArmAuto(true);
        sleep(500);
        moveClawAuto(false);

        // Align to the power shoot position
        shooterMotor.setPower(0.725);
        movePID(-35, 0.8);
        movePIDSide(95, 0.7, true);
        shootPowerShoots();

        if (runtime.seconds() <= 20){
            movePID(-150, 0.7);
            moveClawAuto(true);
//            moveArmAuto(false);
            sleep(1000);
            moveClawAuto(false);
            sleep(500);
//            moveArmAuto(true);
            goWhite(true);
            movePIDSide(95, 0.7, false);
//            moveArmAuto(false);
            sleep(500);
            moveClawAuto(false);
        } else

            // Land over the launch line
            goWhite(true);
    }

    // Program run when there's two rings in front of the robot
    private void goZoneB() {

        // Align to the starter stack by strafing right
        movePIDSide(45, 0.8, true);
        shooterMotor.setPower(0.73);
        movePID(60, 1);

        // Shoot in the high goal and lift the arm
        sleep(250);
        shootRing(1);
//        moveArmAuto(true);

        // Turn on the intake and take the ring
        intakeMotor.setPower(1);
        movePID(65, 1);

        // Align to the power shoot
        shooterMotor.setPower(0.62);
        intakeMotor.setPower(0);
        movePIDSide(55, 0.8, true);

        // Shoot power  shots
        shootPowerShoots();
//        moveArmAuto(true);
        turn(0.75, false, 13);

        // Deliver the first Wobble Goal
        goWhite(true);
        movePIDSide(20,0.8, false);
        movePID(45,1);
//        moveArmAuto(false);
        sleep(500);
        moveClawAuto(true);
        sleep(500);
//        moveArmAuto(true);

        // Return and park in the white tape
        moveWithoutFinalCorrectionPID(-40, 1);
    }

    // Program run when there's four rings in front of the robot
    private void goZoneC() {

        // Align to the starter stack by strafing right
        movePIDSide(45, 0.8, true);
        shooterMotor.setPower(0.73);
        movePID(60, 1);

        // Shoot in the high goal and lift the arm
        sleep(250);
        shootRing(3);

        shooterMotor.setPower(0.62);
        movePID(20, 0.6);
        sleep(500);
        shootRing(1);

        // Turn on the intake and take the ring
        intakeMotor.setPower(1);
        movePID(45, 1);


        // Align to the power shoot
        intakeMotor.setPower(0);
        shooterMotor.setPower(0.62);
        movePIDSide(55, 0.8, true);

        // Shoot power  shots

        shootPowerShoots();
//        moveArmAuto(true);
        turn(0.75, false, 13);

        // Deliver the first Wobble Goal
        goWhite(true);
        movePIDSide(20,0.8, false);
        movePID(45,1);
//        moveArmAuto(false);
        sleep(500);
        moveClawAuto(true);
        sleep(500);
//        moveArmAuto(true);

        // Return and park in the white tape
        moveWithoutFinalCorrectionPID(-40, 1);
    }

    /*MOVEMENT METHODS*/
    // Move the robot using PID
    public void moveWithoutFinalCorrectionPID(double distance, double speed){
        double currentAngle = 0;

        // define the PD variables
        double p;
        double i = 0;
        double d;
        double pid;
        double error;
        double lastError = 0;

        // Convert the encoder values to centimeters
        double rotation = (distance / WHEEL_CIRCUMFERENCE_CM) / GEAR_REDUCTION;
        int targetEncoder = (int)(rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FL.setTargetPosition(targetEncoder);
        FR.setTargetPosition(targetEncoder);
        BR.setTargetPosition(targetEncoder);
        BL.setTargetPosition(targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(speed);
        FR.setPower(speed);
        BL.setPower(speed);
        BR.setPower(speed);

        while ( FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() ) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // the PID in action
            error = angle - currentAngle;

            p = error * kp;
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            FL.setPower(speed - pid);
            FR.setPower(speed + pid);
            BL.setPower(speed - pid);
            BR.setPower(speed + pid);

            sleep(updateRate);
            lastError = error;

            telemetry.update();
        }

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void movePID(double distance, double speed){
        double currentAngle = 0;
        final double smoother = 0.2;

        // define the PD variables
        double p;
        double i = 0;
        double d;
        double pid;
        double error;
        double lastError = 0;

        // Convert the encoder values to centimeters
        double rotation = (distance / WHEEL_CIRCUMFERENCE_CM) / GEAR_REDUCTION;
        int targetEncoder = (int)(rotation * TICKS_REV);

        resetEncoder();

        //This gets the position and makes the robot ready to move
        FL.setTargetPosition(targetEncoder);
        FR.setTargetPosition(targetEncoder);
        BR.setTargetPosition(targetEncoder);
        BL.setTargetPosition(targetEncoder);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(0.01);
        FR.setPower(0.01);
        BL.setPower(0.01);
        BR.setPower(0.01);

        while ( FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() ) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // the PID in action
            error = angle - currentAngle;
            p = error * kp;
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            force[0] =   speed - pid;
            force[1] = - speed + pid;
            force[2] = - speed - pid;
            force[3] =   speed + pid;

            // The smoother
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

            FL.setPower( force[0] );
            FR.setPower( force[1] );
            BL.setPower( force[2] );
            BR.setPower( force[3] );

            sleep(updateRate);
            lastError = error;

            telemetry.addData("proportional", p);
            telemetry.addData("integral", i);
            telemetry.addData("derivative", d);
            telemetry.addData("pid", pid);
            telemetry.update();
        }

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(angle-currentAngle)>0.7) && distance < 0){
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            error = angle - currentAngle;
            p = error * kp;
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            FL.setPower( - pid);
            FR.setPower( + pid);
            BL.setPower( - pid);
            BR.setPower( + pid);

            sleep(updateRate);

            lastError = error;
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }


    // Move the robot sideways using PID
    public void movePIDSide(double distance, double speed, boolean isRight){
        double currentAngle;
        final double smoother = 0.2;
        final double threshold = 1;

        // define the PID variables
        double p;
        double i = 0;
        double d;
        double pid;
        double error;
        double lastError = 0;

        // Convert the encoder values to centimeters
        double rotation = (distance / WHEEL_CIRCUMFERENCE_CM) / GEAR_REDUCTION;
        int targetEncoder = (int)(rotation * TICKS_REV);

        resetEncoder();

        // This gets the position and makes the robot ready to move
        if (!isRight) speed *=- 1;


        FL.setPower(0.01);
        FR.setPower(0.01);
        BL.setPower(0.01);
        BR.setPower(0.01);


        while ( Math.abs(FL.getCurrentPosition()) < targetEncoder || Math.abs(FR.getCurrentPosition()) < targetEncoder ||
                Math.abs(BL.getCurrentPosition()) < targetEncoder || Math.abs(BR.getCurrentPosition()) < targetEncoder) {

            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            error = angle - currentAngle;
            p = error * kp;
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            force[0] =   speed - pid;
            force[1] = - speed + pid;
            force[2] = - speed - pid;
            force[3] =   speed + pid;

            // The smoother
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

            FL.setPower( force[0] );
            FR.setPower( force[1] );
            BL.setPower( force[2] );
            BR.setPower( force[3] );

            telemetry.addData("Moving right", isRight);
            telemetry.update();

            sleep(updateRate);

            lastError = error;
        }

        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (Math.abs(angle-currentAngle)>threshold){
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            error = angle - currentAngle;
            p = error * kp;
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            FL.setPower(- pid);
            FR.setPower(+ pid);
            BL.setPower(- pid);
            BR.setPower(+ pid);

            telemetry.addData("Angle correction", error);
            telemetry.update();

            sleep(updateRate);

            lastError = error;
        }

        // Reset the smoother
        lastForce[0] = 0;
        lastForce[1] = 0;
        lastForce[2] = 0;
        lastForce[3] = 0;
        force[0] = 0;
        force[1] = 0;
        force[2] = 0;
        force[3] = 0;

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Stop the motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        telemetry.clear();
        telemetry.update();

    }

    // Program used to precisely turn the robot
    public void turn(double pidK, boolean isRight, double targetAngle) {

        final double threshold = 0.25;

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angle;

        double p;
        double i = 0;
        double d;
        double pid;
        double error;
        double lastError = 0;


        if (isRight) {

            angle = -targetAngle + currentAngle;
            if (angle < -180) angle += 360;
            if (angle - currentAngle > 180) currentAngle += 360;

            while (Math.abs(angle-currentAngle) > threshold) {

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (angle - currentAngle > 180) currentAngle += 360;

                error = angle - currentAngle;
                p = error * kp;
                i += error * ki;
                d = (error - lastError) * kd;
                pid = (p+i+d) / -k;

                FL.setPower( + pid * pidK);
                FR.setPower( - pid * pidK);
                BL.setPower( + pid * pidK);
                BR.setPower( - pid * pidK);

                sleep(updateRate);

                lastError=error;
            }

        } else {

            angle = targetAngle + currentAngle;
            if (angle > 180) angle -= 360;
            if (currentAngle - angle > 180) currentAngle -= 360;

            while (Math.abs(angle-currentAngle) > threshold) {

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (currentAngle - angle > 180) currentAngle -= 360;

                error = angle - currentAngle;
                p = error * kp;
                i += error * ki;
                d = (error - lastError) * kd;
                pid = -(p+i+d) / k;

                FL.setPower(+ pid * pidK);
                FR.setPower(- pid * pidK);
                BL.setPower(+ pid * pidK);
                BR.setPower(- pid * pidK);

                sleep(updateRate);

                lastError=error;
            }
        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    // Move the robot's arm
//    public void moveArmAuto(boolean isUp ) {
//
//        final double force = 0.7;
//
//        if (isUp) {
//
//            armMotor.setTargetPosition(up);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armMotor.setPower(-force);
//        } else {
//            armMotor.setTargetPosition(down);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armMotor.setPower(force);
//        }
//
//        if (!armMotor.isBusy()) {
//            armMotor.setPower(0);
//        }
//    }

    // Open or close the arm's claw
    private void shootPowerShoots() {

//        moveArmAuto(false);
        sleep(500);

        for (int n = 1; n <= 2; n++) {
            trigServo.setPosition(0.8);
            sleep(750);
            trigServo.setPosition(0);
            sleep(750);
            turn(0.75, true, 4);
        }
        turn(0.75, true, 5);
        trigServo.setPosition(0.8);
        sleep(750);
        trigServo.setPosition(0);
        sleep(750);
        shooterMotor.setPower(0);
    }

    // Move the robot's claw
    public void moveClawAuto(boolean isOpen) {
        if(isOpen) {
            clawServo.setPosition(0);
        }
        if(!isOpen) {
            clawServo.setPosition(1);
        }
    }

    // Shoot rings
    private void shootRing(int quantity) {

//        moveArmAuto(false);
        sleep(500);

        for (int n = 1; n <= quantity; n++) {
            trigServo.setPosition(0.8);
            sleep(1500);
            trigServo.setPosition(0);
            sleep(500);
        }

        shooterMotor.setPower(0);
//        moveArmAuto(true);
    }

    /* INITIALIZER METHODS*/
    // Initialize the Vuforia Library
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // Initialize the IMU
    public void initIMU (){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    // Initialize the Tensor Flow Object Detection
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    // Calibrate the arm
//    public void calibrateArm(){
//
//        final int difference = -1100;
//
//        while (!touchSensor.isPressed()) {
//            armMotor.setPower(0.3);
//        }
//
//        armMotor.setPower(0);
//        up = armMotor.getCurrentPosition();
//        down = up + difference;
//    }
}