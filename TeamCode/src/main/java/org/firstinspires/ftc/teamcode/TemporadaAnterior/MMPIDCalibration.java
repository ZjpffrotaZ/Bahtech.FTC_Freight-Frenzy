package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

import static java.lang.Thread.sleep;
@Disabled
@TeleOp(name =  "Calibrate PID", group = "BahTech")
public class MMPIDCalibration extends LinearOpMode {

    final int TICKS_REV = 1120;
    final double GEAR_REDUCTION = 1.25;
    final double WHEEL_CIRCUMFERENCE_CM = Math.PI*10.16;

    private final double[] force = new double[4];
    private final double[] lastForce = new double[4];

    double kp = 1;
    double ki = 1;
    double kd = 1;
    final double k = 35;
    final long updateRate = 100L;
    double angle = 0;

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    BNO055IMU imu;

    final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override
    public void runOpMode() {

        Locale.setDefault(Locale.US);
        // Find motors in the hub
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        // Left motor's reverse direction
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        initIMU();

        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("KP", String.format(Locale.getDefault(), "%.2f", kp));
            telemetry.addData("KI", String.format(Locale.getDefault(), "%.2f",  ki));
            telemetry.addData("KD", String.format(Locale.getDefault(), "%.2f", kd));
            telemetry.addData("K", k);
            telemetry.update();

            try {

                if (gamepad1.right_bumper) {
                    turn(true);
                }

                if (gamepad1.left_bumper) {
                    turn(false);
                }

                if (gamepad1.b) {
                    movePIDSide(100, 1, true);
                }

                if (gamepad1.x) {
                    movePIDSide(100, 1, false);
                }

                if (gamepad1.y) {
                    movePID(100, 1);
                }

                if (gamepad1.a) {
                    movePID(-100, 1);
                }

                if (gamepad1.dpad_up){
                    kp +=0.02;
                    sleep(100);
                }

                if (gamepad1.dpad_down){
                    kp -=0.02;
                    sleep(100);
                }

                if (gamepad1.dpad_right){
                    ki +=0.02;
                    sleep(100);
                }

                if (gamepad1.dpad_left){
                    ki -=0.02;
                    sleep(100);
                }

                if (gamepad1.right_stick_button){
                    kd +=0.02;
                    sleep(100);
                }

                if (gamepad1.left_stick_button){
                    kd -=0.02;
                    sleep(100);
                }

            } catch (InterruptedException e) {
                telemetry.addData("ERROR", e.getMessage());
            } catch (TurnException e){
                telemetry.addData("ERROR", e.getMessage());
                telemetry.update();
                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
                sleep(1000);
            }
        }
    }


    private void turn(boolean isRight) throws InterruptedException {

        final double threshold = 0.5;

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angle;

        double p;
        double i = 0;
        double d;
        double pid;
        double error;
        double lastError = 0;


        if (isRight) {

            angle = -(double) 90 + currentAngle;
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

                FL.setPower( + pid );
                FR.setPower( - pid );
                BL.setPower( + pid );
                BR.setPower( - pid );

                if (gamepad1.back){
                    throw new TurnException("EMERGENCY STOP");
                }

                sleep(updateRate);
                lastError=error;
            }

        } else {

            angle = (double) 90 + currentAngle;
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

                FL.setPower(+ pid );
                FR.setPower(- pid );
                BL.setPower(+ pid );
                BR.setPower(- pid );

                if (gamepad1.back){
                    throw new TurnException("EMERGENCY STOP");
                }

                sleep(updateRate);
                lastError=error;
            }
        }

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

            if (gamepad1.back){
                throw new TurnException("EMERGENCY STOP");
            }

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

        while (Math.abs(angle-currentAngle)>0.7){
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

            if (gamepad1.back){
                throw new TurnException("EMERGENCY STOP");
            }

            lastError = error;
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

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

            if (gamepad1.back){
                throw new TurnException("EMERGENCY STOP");
            }

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

            if (gamepad1.back){
                throw new TurnException("EMERGENCY STOP");
            }

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

    private void initIMU(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
}
