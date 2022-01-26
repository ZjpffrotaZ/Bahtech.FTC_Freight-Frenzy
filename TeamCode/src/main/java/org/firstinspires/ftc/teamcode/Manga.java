package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name ="manga", group = "pessego")
public class Manga extends LinearOpMode {
    DcMotor motor;
    TouchSensor tq;

    double banana = 0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "teste");
        tq = hardwareMap.get(TouchSensor.class, "touch");

    waitForStart();
        while (opModeIsActive()){
           if (tq.isPressed()){
               if (banana < 0.9){
                   sleep(100);
                  banana = banana + 0.01;
               }

               motor.setPower(banana);
               telemetry.addData("touchSensor:",true);
               telemetry.update();
           }else{
               motor.setPower(banana = 0);
               telemetry.addData("touchSensor:", false);
               telemetry.update();
           }
        }}}

