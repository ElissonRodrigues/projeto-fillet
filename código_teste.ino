#include <Stepper.h>
#include <IRremote.h>
#include <EEPROM.h> // Inclui a biblioteca EEPROM

// Definir pinos do Arduino
#define PinSDriver 12 // Pino "S" do Adaptador do Driver
#define PinDDriver 13 // Pino "D" do Adaptador do Driver

#define PinIRRemote 2 // Pino de sinal do Módulo KY-022
const int PinControlMosfet = 6; // Pino de controle do módulo MOSFET

#define saltosDeVelocidade 30 // Define qual será o salto toda vez que pressionar o botão "+" ou "-"

// Motor Stepper
const int passosPorRevolucao = 360;
Stepper motor(passosPorRevolucao, PinDDriver, PinSDriver);

// Velocidades do Motor
const int velocidadeMinima = 18;
const int velocidadeMaxima = 700;
int velocidadeAtual = 0;

IRrecv receptorIR(PinIRRemote); // Pino conectado ao receptor IR
decode_results resultadosIR;

// Controle de Temperatura
const int pinSensorTemp = A0; // Pino do sensor de temperatura
const float setpointTemp = 25.0; // Temperatura desejada
const float kp = 0.09;
const float ki = 1;
const float kd = 8;
const int period = 100; // Frequência de atualização do controle em milissegundos

float error_previo = 0;
float integral = 0;

void setup() {
  motor.setSpeed(velocidadeAtual);
  receptorIR.enableIRIn(); // Inicializa o receptor IR
  Serial.begin(19200);

  pinMode(PinControlMosfet, OUTPUT); // Define o pino de controle do módulo MOSFET como saída

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

  // Verifica se um código do controle remoto foi recebido
  if (receptorIR.decode(&resultadosIR)) {

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

  // Controle de Temperatura
  if (millis() % period == 0) {
    float temperatura = lerTemperatura();
    float erro = setpointTemp - temperatura;

    float p = kp * erro;
    integral += ki * erro * period / 1000.0;
    float d = kd * (erro - error_previo) / (period / 1000.0);

    float controle = p + integral + d;

    // Ativa o módulo MOSFET com base no controle de temperatura
    if (controle > 0) {
      digitalWrite(PinControlMosfet, HIGH);
    } else {
      digitalWrite(PinControlMosfet, LOW);
    }

    error_previo = erro;
  }
}

float lerTemperatura() {
  int valorLeitura = analogRead(pinSensorTemp);
  float tensao = valorLeitura * (5.0 / 1023.0);
  float temperatura = (tensao - 0.5) * 100.0;

  return temperatura;
}
