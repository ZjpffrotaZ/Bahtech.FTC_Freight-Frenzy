package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Pegasus02", group ="Concept")
public class pegasus02 extends LinearOpMode {

    // Declaração dos componentes físicos

    DcMotor arm;
    DcMotor take;
    DcMotor fl;
    DcMotor fr;

    ServoImplEx deposit;
    ServoImplEx duck;

    BNO055IMU imu;

    /* Declaração das variaveis que vão receber os valores do joystick do gamepad, X e Y,
       configurando os parâmetros do imu .
       Estamos usando o IMU para termos ideia de como vamos usar os seus valores para
       realizar o algoritimo PID, que será usado no período autônomo.*/

    double setmode = 1;
    double setmode2 = 1;
    double x, y, orientation, right, left, speed,up, down;
    final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override
    public void runOpMode() {

        //Comandos para o robô reconhecer o hardware.

        imu  = hardwareMap.get(BNO055IMU.class, "imu");

        duck = hardwareMap.get(ServoImplEx.class, "duck");

        arm  = hardwareMap.get(DcMotor.class, "arm");
        take = hardwareMap.get(DcMotor.class, "take");
        fl   = hardwareMap.get(DcMotor.class, "fl");
        fr   = hardwareMap.get(DcMotor.class, "fr");

        //Aqui estamos definindo os parâmetros que nos interessa.

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        /* Vou deixar esses comandos comentados, caso o pessoal da mecânica
           queira mudar a frente do robô ;).
        *
           FL.setDirection(DcMotor.Direction.REVERSE);
           FR.setDirection(DcMotor.Direction.REVERSE);
        *
            Logo abaixo temos o comando para iniciar os parâmetros do imu.
        */

        imu.initialize(parameters);

        waitForStart();
                                            //Aqui estamos atribuindo os valores do joystick para as
        while (opModeIsActive()) {          //variaveis "X" e "Y".
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;

            if (gamepad1.left_stick_button == true) {
                fl.setPower(0.8);
                fr.setPower(-0.8);
            }

            right = x + y;      //Aqui estou declarando que o motor da direita vai ser
            left = x - y;        // o resultado do x+y e o da esquerda x-y.

            fr.setPower(right);  //Aqui estou colocando o resultado da
            fl.setPower(left);      //soma e da subtração nos motores.

            // ACIONA INTAKE APERTANDO A. //
            if (gamepad1.a) {               //Aqui estamos definindo a configuração do intake, onde
                setmode++;                    //se o botão for precionado uma vez ele liga e se pressionado
                if (setmode % 2 == 0){          //denovo o sistema é desligado.
                    while (speed <= 1.0) {
                        sleep(30);        //Estamos usando esse "while" e essa seuquência de
                        speed += 0.01;               //cáuculos para reduzir o torque do motor quando ele
                        take.setPower(speed);}         //acionado e assim reduzindo o desgaste.
                }else{ speed=0;}
            }

            // ACIONA SERVO DO PATO
            if(gamepad1.y){                 //Quando precionado o sistema liga e se manteém,
                setmode2++;                  // quando precionado novamente desliga.
                if (setmode2 % 2 == 0){
                    while (true) {
                        duck.setPosition(1.0);}
                }else{ duck.setPosition(0);}
            }

            // ACIONA O MOTOR DO BRAÇO
            if(gamepad1.dpad_up){          //Se precionarmos o dpad para cima o braço levanta,
                arm.setPower(0.7);          // e se precionarmos para baixo, ele abaixa.
            }
            if(gamepad1.dpad_down){
                arm.setPower(-0.7);
            }

            //ACIONA O DEPOSITO
            if (gamepad1.dpad_left){
                deposit.setPosition(-0.6);
            }
            if (gamepad1.dpad_right){
                deposit.setPosition(0.6);
            }

            Orientation orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            telemetry.addData("X angle:", orientation.firstAngle);
            telemetry.addData("Y angle:", orientation.secondAngle);
            telemetry.addData("Z angle:", orientation.thirdAngle);
            telemetry.addData("Motor FR:", right);
            telemetry.addData("Motor FL:", left);
            telemetry.addData("Intake: ", take);
            telemetry.update();
        }
    }
}