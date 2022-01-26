package org.firstinspires.ftc.teamcode;

//Import the general libraries to use in the program

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "My Mechanum: Core", group = "BahTech")
public class MMCore extends LinearOpMode {

    // Map the hardware to autonomous and TeleOp
    final MMMovement move = new MMMovement();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        // Create the Hardware Map and start IMU
        move.defHardware(hardwareMap);
        move.initIMU(hardwareMap);
        move.intakeForce(0);
//        move.calibrateArm();

        // Run once you press PLAY
        waitForStart();

        // Run repeatedly after you press play
        while(opModeIsActive()){

            // Create variables for the control
            double leftX = -gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y;
            double rightX = -gamepad1.right_stick_x;

//            boolean buttonY = gamepad1.y;
//            boolean buttonX = gamepad1.x;
//            boolean buttonA = gamepad1.a;
//            boolean buttonB = gamepad1.b;
            boolean upButton = gamepad1.dpad_up;

            double lt = gamepad1.left_trigger;
            double rt = gamepad1.right_trigger;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            boolean rStickButton = gamepad1.right_stick_button;
            boolean lStickButton = gamepad1.left_stick_button;

            // MMMovement actions
            move.shoot(rt >= 0.3, lt >= 0.3);
            move.controlIntake(upButton);
            move.shotForce(lStickButton, rStickButton);
            move.moveByRobot(leftY, leftX, rightX, lb, rb);
//            move.claw(buttonY, buttonA, buttonB, buttonX);

            // Add info in the Driver Station screen
            telemetry.addData("Front Left Motor",  String.format("%.2f", move.getFlForce()));
            telemetry.addData("Front Right Motor", String.format("%.2f", move.getFrForce()));
            telemetry.addData("Back Left Motor",   String.format("%.2f", move.getBlForce()));
            telemetry.addData("Back Right Motor",  String.format("%.2f", move.getBrForce()));
            telemetry.update();
        }
    }
}