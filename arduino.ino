#include <Stepper.h>
#include <IRremote.h>
#include <EEPROM.h> // Inclui a biblioteca EEPROM

// definir pinos do arduino
#define PinSDriver 12 // Pino "S" do Adaptador do Driver
#define PinDDriver 13 // Pino "D" do Adaptador do Driver

#define PinIRRemote 2 // Pino de sinal do Modulo KY-022
#define saltosDeVelocidade 30 //Define qual será o salto toda vez que precionar o botão "+" ou "-"

//Motor Stepper
const int passosPorRevolucao = 360;
Stepper motor(passosPorRevolucao, PinDDriver, PinSDriver);

//Velocidades do Motor
const int velocidadeMinima = 18;
const int velocidadeMaxima = 700;
int velocidadeAtual = 0;

IRrecv receptorIR(PinIRRemote); // Pino conectado ao receptor IR
decode_results resultadosIR;

void setup() {
  motor.setSpeed(velocidadeAtual);
  receptorIR.enableIRIn(); // Inicializa o receptor IR
  Serial.begin(19200);

  // Carrega a velocidade salva na EEPROM
  velocidadeAtual = EEPROM.read(0); // Lê o valor da posição 0 da EEPROM

  // Se o valor lido for inválido, define a velocidade máxima como padrão
  if (velocidadeAtual < velocidadeMinima || velocidadeAtual > velocidadeMaxima) {
    velocidadeAtual = velocidadeMaxima;
  }

  motor.setSpeed(velocidadeAtual);
}

void loop() {
  Serial.print("Velocidade atual do Motor: ");
  Serial.println(velocidadeAtual);
  
  if (receptorIR.decode(&resultadosIR)) {
    // Verifica se um código do controle remoto foi recebido

    if (resultadosIR.value == 16754775) {
      // Código correspondente a "VOL+"
      velocidadeAtual += saltosDeVelocidade;
    }
    else if (resultadosIR.value == 16769055) {
      // Código correspondente a "VOL-"
      velocidadeAtual -= saltosDeVelocidade;
    }

    // Limita a velocidade dentro do intervalo definido
    if (velocidadeAtual < velocidadeMinima) {
      velocidadeAtual = velocidadeMinima;
    } else if (velocidadeAtual > velocidadeMaxima) {
      velocidadeAtual = velocidadeMaxima;
    }

    motor.setSpeed(velocidadeAtual); // Atualiza a velocidade do motor
    receptorIR.resume(); // Continua aguardando sinais do controle remoto
  }

  motor.step(passosPorRevolucao);

  // Salva a velocidade atual na EEPROM
  EEPROM.write(0, velocidadeAtual); // Escreve o valor da velocidade atual na posição 0 da EEPROM
}

