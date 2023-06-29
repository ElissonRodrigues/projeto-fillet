/// Inclua a biblioteca Stepper
#include <Stepper.h>

// Velocidade Minima 10
// Velocidade Máxima 700

// Define o número de passos por revolução do motor
const int passosPorRevolucao = 360;

// Cria um objeto Stepper
// Parâmetros: número de passos por revolução, pino de controle 1, pino de controle 2
Stepper motor(passosPorRevolucao, 13, 12);

// Define a velocidade inicial do motor (em RPM)
const int velocidadeInicial = 700;

void setup() {
  // Define a velocidade do motor (em RPM)
  motor.setSpeed(velocidadeInicial);
}

void loop() {
  // Faz o motor girar uma volta no sentido horário
  motor.step(passosPorRevolucao);
}
