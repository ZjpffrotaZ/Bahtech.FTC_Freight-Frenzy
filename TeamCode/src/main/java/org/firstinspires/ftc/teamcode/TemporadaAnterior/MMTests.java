package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name = "Tests", group = "BahTech")
public class MMTests extends LinearOpMode {

    @Override
    public void runOpMode() {

        // MOTORS
        DcMotor FR = hardwareMap.dcMotor.get("FL");
        DcMotor FL = hardwareMap.dcMotor.get("FR");
        DcMotor BR = hardwareMap.dcMotor.get("BL");
        DcMotor BL = hardwareMap.dcMotor.get("BR");
        DcMotor shooterMotor = hardwareMap.dcMotor.get("shooter_motor");
        DcMotor intake = hardwareMap.dcMotor.get("intake_motor");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm_motor");

        // SERVOS
        Servo shooterServo = hardwareMap.servo.get("trig_servo");
        Servo clawServo = hardwareMap.servo.get("claw_servo");

        // SENSORS
        ColorRangeSensor colorSensor = hardwareMap.get(ColorRangeSensor.class, "sensor_color");
        TouchSensor      touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
        BNO055IMU        imu         = hardwareMap.get(BNO055IMU.class, "imu");
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // Configure the motors
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the IMU
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        // Test the Movement motors
        try {
            String Errors = "Movement OK";

            // Move the robot forward for one second
            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);

            sleep(1000);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            // Show the encoder data on screen
            telemetry.addData("Encoder FL", FL.getCurrentPosition());
            telemetry.addData("Encoder FR", FR.getCurrentPosition());
            telemetry.addData("Encoder BL", BL.getCurrentPosition());
            telemetry.addData("Encoder BR", BR.getCurrentPosition());
            telemetry.update();

            // Identify if there's an unplugged encoder
            if (FL.getCurrentPosition() == 0 || FR.getCurrentPosition() == 0 || BL.getCurrentPosition() == 0 || BR.getCurrentPosition() == 0) {
                Errors = "UNPLUGGED ENCODER: ";
            }

            // Identify the unplugged encoder
            if (FL.getCurrentPosition() == 0) {
                Errors += "FL, ";
            }
            if (FR.getCurrentPosition() == 0) {
                Errors += "FR, ";
            }
            if (BL.getCurrentPosition() == 0) {
                Errors += "BL, ";
            }
            if (BR.getCurrentPosition() == 0) {
                Errors += "BR.";
            }

            throw new Exception(Errors);

        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(3000);
        }

        // Test the arm motor
        try {
            telemetry.addData("TESTING MOTOR", " ARM");
            telemetry.update();

            // Move the arm up
            while (!touchSensor.isPressed()) {
                armMotor.setPower(0.3);
            }
            armMotor.setPower(0);
            sleep(500);

            // Move the arm down
            armMotor.setPower(-0.1);
            sleep(2000);
            armMotor.setPower(0);

            // Test if the encoder registered any value
            if (armMotor.getCurrentPosition() == 0)
                throw new Exception("UNPLUGGED ARM ENCODER");

            telemetry.addData("ARM MOTOR", " OK");
            telemetry.update();
            sleep(3000);

        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(3000);
        }

        // Test the intake motor
        try {
            telemetry.addData("TESTING MOTOR", "INTAKE");
            telemetry.update();

            // Turn on and off the intake
            sleep(5000);
            intake.setPower(1);
            sleep(1000);
            intake.setPower(0);

        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        // Test the shooter motor
        try {
            telemetry.addData("TESTING MOTOR", "SHOOTER");
            telemetry.update();

            // Turn on and off the shooter
            shooterMotor.setPower(1);
            sleep(2000);
            shooterMotor.setPower(0);

        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        // Test the shooter servo
        try {
            telemetry.addData("TESTING SERVO", "SHOOTER");
            telemetry.update();

            // Open and closes the shooter servo
            sleep(2000);
            shooterServo.setPosition(1);
            sleep(2000);
            shooterServo.setPosition(0);

        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        // Test the claw servo
        try {
            telemetry.addData("TESTING SERVO", "CLAW");
            telemetry.update();

            // Open and closes the claw servo
            sleep(2000);
            clawServo.setPosition(1);
            sleep(1500);
            clawServo.setPosition(0);

        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            telemetry.update();
            sleep(5000);
        }

        telemetry.addData("Press play to", " test the sensors");
        telemetry.addData("SENSORS", " COLOR, DISTANCE, IMU and TOUCH"); telemetry.update();
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            // Test the Light, Distance and IMU sensors
            try {
                telemetry.addData("DISTANCE    ", colorSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("ALPHA       ", colorSensor.alpha());
                telemetry.addData("IMU ANGLE   ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("TOUCH SENSOR", touchSensor.getValue());
                telemetry.update();

            } catch (Exception e) {
                telemetry.addData("ERROR", e);
                telemetry.update();
                sleep(5000);
            }
        }
    }
}

