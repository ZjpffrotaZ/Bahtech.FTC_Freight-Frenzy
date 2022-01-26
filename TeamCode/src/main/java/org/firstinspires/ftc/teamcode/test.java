package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="test", group="Pushbot")
public class test extends LinearOpMode {
        DcMotor motor;
        TouchSensor ts;

        double valor;

        @Override
        public void runOpMode() {
                motor = hardwareMap.get(DcMotor.class, "motor");
                ts = hardwareMap.get(TouchSensor.class, "touch");

                waitForStart();
                while (opModeIsActive()){
                        if (ts.isPressed()){
                                if (valor < 0.9){
                                        sleep(100);
                                        valor = valor + 0.01;
                                }

                                motor.setPower(valor);
                                telemetry.addLine("touchSensor: true");
                                telemetry.update();
                        }else{
                                motor.setPower(valor = 0);
                                telemetry.addLine("touchSensor: false");
                                telemetry.update();
                        }
                }}}




