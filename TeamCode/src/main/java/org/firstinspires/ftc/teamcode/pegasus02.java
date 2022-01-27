package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="Pegasus02", group ="Concept")
public class pegasus02 extends LinearOpMode {

    // Declaração dos componentes físicos

    DcMotor arm;
    DcMotor take;
    DcMotor fl;
    DcMotor fr;

    Servo deposit;
    Servo duck;


    /* Declaração das variaveis que vão receber os valores do joystick do gamepad, X e Y.*/

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double position = (MAX_POS - MIN_POS) / 2;

    double setmode = 1;
    double setmode2 = 1;
    double x, y, right, left, speed;

    @Override
    public void runOpMode() {

        //Comandos para o robô reconhecer o hardware.

        deposit =hardwareMap.get(Servo.class, "deposit");
        duck = hardwareMap.get(Servo.class, "duck");

        arm  = hardwareMap.get(DcMotor.class, "arm");
        take = hardwareMap.get(DcMotor.class, "take");
        fl   = hardwareMap.get(DcMotor.class, "fl");
        fr   = hardwareMap.get(DcMotor.class, "fr");

        /* Vou deixar esses comandos comentados, caso o pessoal da mecânica
           queira mudar a frente do robô ;).
        *
           FL.setDirection(DcMotor.Direction.REVERSE);
           FR.setDirection(DcMotor.Direction.REVERSE);
        */

        waitForStart();
                                            //Aqui estamos atribuindo os valores do joystick para as
        while (opModeIsActive()) {          //variaveis "X" e "Y".
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;

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
                        duck.setPosition(1.0);
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
                sleep(30);
                deposit.setPosition(0);
            }
            if (gamepad1.dpad_right){
                deposit.setPosition(0.6);
                sleep(30);
                deposit.setPosition(0);
            }

            telemetry.addData("Motor FR:", right);
            telemetry.addData("Motor FL:", left);
            telemetry.addData("Intake: ", take);
            telemetry.update();
        }
    }
}