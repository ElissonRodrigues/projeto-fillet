#include <Stepper.h>

// Definir pinos do Arduino
#define PinSDriver 12 // Pino "S" do Adaptador do Driver
#define PinDDriver 13 // Pino "D" do Adaptador do Driver

// Motor Stepper
const int passosPorRevolucao = 360;
Stepper motor(passosPorRevolucao, PinDDriver, PinSDriver);

// Velocidades do Motor
const int velocidadeMinima = 18;
const int velocidadeMaxima = 700;
int velocidadeAtual = velocidadeMinima;

void setup() {
  motor.setSpeed(velocidadeAtual);
  Serial.begin(9600);
  
  velocidadeAtual = 700;
  motor.setSpeed(velocidadeAtual);
}

void loop() {
  motor.step(passosPorRevolucao);
}
