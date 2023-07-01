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
float setpointTemp = 25.0; // Temperatura desejada
const float kp = 0.09;
const float ki = 1;
const float kd = 8;
const int period = 100; // Frequência de atualização do controle em milissegundos

float error_previo = 0;
float integral = 0;

// Definir os códigos do controle remoto para os botões "Prev" e "Next"
const unsigned long codigoBotaoPrev = 0x12345678; // Substitua pelo código real do controle para o botão "Prev"
const unsigned long codigoBotaoNext = 0x87654321; // Substitua pelo código real do controle para o botão "Next"

void setup() {
  motor.setSpeed(velocidadeAtual);
  receptorIR.enableIRIn(); // Inicializa o receptor IR
  Serial.begin(9600);

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

  Serial.print("Botão Pressionado: ");
  Serial.println(resultadosIR.value);

  // Verifica se um código do controle remoto foi recebido
  if (receptorIR.decode(&resultadosIR)) {
    // Verifica se o código recebido corresponde ao botão "Prev"
    if (resultadosIR.value == codigoBotaoPrev) {
      // Reduzir a temperatura desejada
      setpointTemp--;
    }

    // HOTEND (EXTRUSORA)
    // Verifica se o código recebido corresponde ao botão "Next"
    else if (resultadosIR.value == codigoBotaoNext) {
      // Aumentar a temperatura desejada
      setpointTemp++;
    }
    // Atualizar a velocidade do motor conforme os botões "Prev" e "Next"
    if (resultadosIR.value == 16754775) {
      // Código correspondente a "VOL+"
      velocidadeAtual += saltosDeVelocidade;
    }

    //MOTOR DE PASSSO (STEPPER NEMA17)
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

    receptorIR.resume();
  }

  // Exibir a temperatura desejada no monitor serial
  Serial.print("Temperatura desejada: ");
  Serial.println(setpointTemp);

  motor.setSpeed(velocidadeAtual); // Atualiza a velocidade do motor

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
