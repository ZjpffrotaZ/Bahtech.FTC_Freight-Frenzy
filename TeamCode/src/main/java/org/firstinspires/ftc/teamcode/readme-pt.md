# Programas #
Esta é uma lista de nossos três programas principais, seu uso, métodos e inovações

## MMMovement ##
### My Mechanum: Movement ###
Contém todos os métodos utilizados para movimentar o robô durante o período tele operado e autônomo.
Os métodos são:

**Métodos usados para definir a configuração do hardware ou algumas ações abstratas**

- **defHardware** Recebe o HardwareMap para mapear os componentes do robô em nosso programa e na
 configure robot.
``` java
public void defHardware (HardwareMap hardwareMap)
```
- **initIMU:** Recebe hardwareMap e cria a configuração IMU.
``` java
public void initIMU (HardwareMap hardwareMap)
```
- **resetEncoder:** Reseta o encoder dos motores.
``` java
public void resetEncoder()
```

**Métodos usados para mover o robô**

- **turn:** recebe a força dos motores, a direção (esquerda ou direita), o ângulo e a contante
 [smoother](#mapeamento-do-robô).
``` java
public void turn(double force, boolean isRight, double targetAngle, final double smoother)
```
- **move:** recebe os controles do gamepad para mover o robô usando o IMU
``` java
public void move(double leftY, double leftX, double rightX, boolean slower, boolean faster)
```
- **intakeForce:** Recebe a força objetivo para ativar o intake
``` java
public void intakeForce(double force)
```
- **shoot:** Recebe o botão do gamepad usado para atirar um anel para o gol alto
``` java
public void shoot(boolean trigger)
```
- **powerShot:** Recebe o botão do gamepad usado para disparar um anel no Power Shot
``` java
public void powerShot(boolean trigger)
```
- **claw:** Recebe as teclas do gamepad usadas para mover a garra
``` java
public void claw(boolean halfDown, boolean totalUp, boolean totalDown, boolean open, boolean close)
```

## MMCore ##
### My Mechanum: Core ###
O programa TeleOp principal, contém um modo de operação linear e os controles do gamepad que usaremos.

Primeiro ele irá definir o Hardware e inicializar o IMU, depois disso irá iniciar um loop que irá
 usar os métodos criados no MMMovement e no gamepad para mover o robô

Essa classe não possui métodos

## MATflowCore ##
### My Autonomous: Tensor Flow Core ###

O principal programa autônomo, contém um modo de operação linear e alguns métodos usados para conduzir o robô.
 Os métodos são:

- **goWhite:** move o robô para frente até detectar uma fita branca no chão
``` java
private void goWhite()
```
- **goZoneA:** contém as instruções para mover o robô quando não há anéis na frente dele 
``` java
private void goZoneA()
```
- **goZoneB:** contém as instruções para mover o robô quando há um anél na frente dele
``` java
private void goZoneB()
```
- **goZoneC:** contém as instruções para mover o robô quando há quatro anéis na frente dele
``` java
private void goZoneC()
```
- **initVuforia:** Inicializa o vuforia e a webcam
``` java
private void initVuforia()
```
- **initTfod:** Inicializa o TensorFlow, biblioteca que detecta objetos usando o vuforia 
``` java
private void initTfod()
```

### computer-vision ###
Por meio dos reconhecimentos do Tensor Flow, podemos reconhecer se há ou não algum anel na
pilha inicial. Então, podemos nos mover para a zona alvo de acordo com as regras do jogo.
- 0 anéis detectados = Zona A
- 1 anel detectado   = Zona B
- 4 anéis detectados = Zona C

Esta é a parte do programa que é responsável por reconhecer a quantidade de aneis detectados pelo
nosso robô e mostrá-lo na tela do Driver station. 

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

Diferente de tudo, esta é uma parte do programa que aparece em muitos métodos de movimento,
 é responsável por fazer nosso robô desacelerar após um movimento, tornando-o mais preciso.
 ele é extremamente útil e pode ser implementado de duas maneiras:
- **Simples:**
Para explicar como funciona, usarei o exemplo presente em nosso método * turn *.

O valor do * smoother * é dado com a subtração entre o ângulo alvo de nosso giro com o atual
ângulo do robô. O resultado será dividido com a constante Smoother que, na maioria dos casos, será
definido dentro do método.

``` java
(anguloObjetivo - anguloAtual) / smoother
```

O valor * smoother * pode ser usado com a força máxima desejada, vamos imaginar que é
0,5. Podemos usar um if-else para definir a força do motor. Se a força desejada for menor, nós a usamos,
se não for, usamos o cálculo do * smoother *.

``` java 
smootherCalc = (anguloObjetivo - anguloAtual) / smoother; 

if (SmootherCalc > forcaDesejada) {
    movementMotor.setPower(forcaDesejada)
} else {
    movementMotor.setPower(SmootherCalc)
}
``` 

- **Complexo:**
Esta forma só é usada no método de movimento no tele operado e é mais difícil de implementar, mas vou tentar
simplificar aqui

Para implementar isso, precisaremos ter:
1. A força do motor atual
1. A última força do motor
1. A constante * smoother *

Com isso, vamos comparar se o valor absoluto da diferença entre a atual e última força do motor
é maior do que a constante * smoother *

``` java
Math.abs( ultimaForca - forcaAtual ) > smoother
``` 

se não for, podemos definir diretamente a força para os motores, mas se for, teremos que mudar o
valor usando lógica

se a última força foi maior do que a força atual, vamos definir a força do motor como ela menos
a constante * smooother *, caso contrário, usaremos a última força mais a constante

``` java
if (Math.abs( ultimaForca - forcaAtual ) > smoother) {
    if (ultimaForca > forcaAtual) {
        movementMotor.setPower(ultimaForca - smoother)
    } else {
        movementMotor.setPower(ultimaForca + smoother)
    }
} else {
    movementMotor.setPower(forcaAtual)
}
``` 

## PID ##
PID é uma abreviação para proporcional-integral-derivada e é usado para previnir erros que podem
acontecer durante o movimento. Para o PID nós usamos o valor angular do IMU. Os componentes do PID
são:

- **Proporcional:** É responsável por criar uma variável que pega a diferença entre o ângulo
 atual e o objetivo. Esse valoe é multiplicado por uma contante kP e somado com o
força de movimento desejada
``` java
final double kP = 1;

while(pid.isActive){
    erro = angulo - anguloAtual;
    p = erro * kp;
}
```
- **Integral:** Sua responsabilidade é obter todos os valores e somar dos erros. Ele é
especialmente útil quando nosso alvo é diferente de zero (a proporcional não será capaz de resolver
isso sozinha) ou se a força que o proporcional gera não é suficiente. O valor integral é
multiplicado por kI
``` java
final double kP = 1;
final double kI = 0.4;
             
while(pid.isActive){
    erro = angulo - anguloAtual;
    p = erro * kp;
    i += erro * kI;    
}
```
- **Derivada:** É responsabilidade dela evitar que a integral chegue a um valor enorme, isso pode
acontecer quando o erro é igual a zero. Para calcular isso, precisaremos salvar o último erro e
subtrair com o erro atual, o resultado será multiplicado por kD
``` java
final double kP = 1;
final double kI = 0.4;
final double kD = 1.5;
double ultimoErro;

while (pid.isActive){
    erro = angulo - anguloAtual;
    p = erro * kp;
    i += erro * kI;    
    d += (ultimoErro - erro) * kd;
}
```

Pegamos os três valores que obtivemos antes e os somamos para obter um PID totalmente funcional.
Essa lógica está em todo o nosso código, especialmente no período autônomo. Você pode vê-lo
funcionando nos métodos movePID, MovePIDSide e goToWhite.
 
# Mapeamento do robô #
Mapear o robô nos ajuda com o gerenciamento de cabos, elaé usada para evitar mudanlas desnecessárias
 de cabos e as complicações que isso causa.

Nós colocamos algumas fitas coloridas nos cabos do nosso robô para nos ajudar a identificá-los.
## HUB 1 / Etiqueta Azul ##
### Motores ###
| Porta | Nome          | Cor     |
|-------|---------------|---------|
|  0    | arm_motor     | Amarelo |
|  1    | intake_motor  | Azul    |
|  2    | shooter_motor | Branco  |

### I2C 0 ###
| Porta | Nome          |
|-------|---------------|
|  0    | imu           |
|  1    | sensor_color  | 

## HUB 2 / Etiqueta rosa ##
### Motors ###
| Porta | Nome          | Cor     |
|------|----------------|---------|
|  0   | FL             | Amarelo |
|  1   | FR             | Azul    |
|  2   | BL             | Branco  |
|  3   | BR             | Verde   |

### Servos ###
| Porta | Nome          |
|-------|---------------|
|  0    | claw_servo    |
|  1    | trig_servo    |

# Gamepad #
Configuramos apenas um gamepad, tivemos alguns brainstorms e decidimos que apenas um piloto seria
melhor do que dois.

o mapeamento do gamepad é:

| Botão  | Função          |
|--------|-----------------|
| X      | Abre a garra    |
| Y      | Levanta o braço |
| A      | Abaixa o braço  |
| B      | Fecha a garra   |
| RB     | Aumenta a velocidade do robô            |
| RT     | Atira um anel (Power Shot)              |
| RB     | Atira um anel (Gol alto)                |
| Dpad para cima    | Inicia/para o intake         |
| DPad para baixo   | Abaixa a garra pela metade   |
| Joystick direito  | Controla o giro do robô      |
| Joystick esquerdo | Controla o movimento do robô |
