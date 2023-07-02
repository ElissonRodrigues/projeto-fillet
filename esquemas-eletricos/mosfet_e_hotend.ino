/*

CONFIGURAR CONEXÕES DO HOTEND E TERMISTOR NTC 100K B3950.

TERMISTOR NTC 100K B3950

1. Conecte um terminal do termistor a um pino analógico do Arduino (A0).

2. Conecte o outro terminal do termistor a um resistor de 10k Ohms.

3. Conecte a outra extremidade do resistor ao GND (terra) do Arduino.

4 Conecte o pino analógico do Arduino ao ponto de junção entre o termistor e o resistor.

#MÓDULO MOSFET

1. Conecte a alimentação da hotend ao módulo Mosfet MF-1000. Conecte o fio vermelho (+) da hotend ao terminal positivo (VIN) do módulo Mosfet e o fio preto (-) ao terminal negativo (GND) do módulo Mosfet.

2.  Conecte o fio de controle do módulo Mosfet ao Arduino. Escolha um pino digital disponível no Arduino, por exemplo, o pino 9. Conecte um fio do pino 9 do Arduino ao terminal com a estrela (*) do módulo Mosfet.

3. Conecte o terminal restante do módulo Mosfet (sem a estrela) ao GND do Arduino.

No código do Arduino, você precisa usar a biblioteca "PID" para controlar a temperatura da hotend. Certifique-se de ter essa biblioteca instalada no seu Arduino IDE.
*/

// Defina os pinos
#define MOSFET_PIN 9
#define TEMPERATURE_PIN A0  // Pino analógico para leitura da temperatura

// Constantes para o termistor
const int THERMISTOR_NOMINAL = 100000;  // Valor nominal do termistor em ohms
const int TEMPERATURE_NOMINAL = 25;     // Temperatura nominal do termistor em graus Celsius
const int B_COEFFICIENT = 3950;         // Coeficiente B do termistor

void setup() {
  // Inicialize os pinos
  pinMode(MOSFET_PIN, OUTPUT);
  analogReference(DEFAULT);  // Use a referência de tensão padrão do Arduino
  Serial.begin(9600);        // Inicialize a comunicação serial
}

void loop() {
  // Defina a temperatura desejada (em graus Celsius)
  double temperaturaDesejada = 200.0;

  // Leia a temperatura atual da hotend
  double temperaturaAtual = lerTemperaturaHotend();

  // Compare a temperatura atual com a temperatura desejada
  // e ligue/desligue o Mosfet com base nessa comparação
  if (temperaturaAtual < temperaturaDesejada) {
    ligarHotend();
  } else {
    desligarHotend();
  }

  delay(1000);  // Aguarde um segundo antes de verificar novamente
}

// Função para ler a temperatura da hotend
double lerTemperaturaHotend() {
  int valorADC = analogRead(TEMPERATURE_PIN);  // Ler valor analógico

  // Calcular resistência do termistor
  double tensao = valorADC * (5.0 / 1023.0);
  double resistencia = (5.0 * 10000.0) / tensao - 10000.0;

  // Calcular temperatura usando a fórmula do termistor
  double steinhart;
  steinhart = resistencia / THERMISTOR_NOMINAL;         // (R/R0)
  steinhart = log(steinhart);                          // ln(R/R0)
  steinhart /= B_COEFFICIENT;                           // 1/B * ln(R/R0)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);    // + (1/To)
  steinhart = 1.0 / steinhart;                          // Inverter
  steinhart -= 273.15;                                  // Converter para Celsius

  return steinhart;
}

// Função para ligar a hotend
void ligarHotend() {
  digitalWrite(MOSFET_PIN, HIGH);
}

// Função para desligar a hotend
void desligarHotend() {
  digitalWrite(MOSFET_PIN, LOW);
}
