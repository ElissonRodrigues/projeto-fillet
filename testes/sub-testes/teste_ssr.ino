#include <PID_v1.h>

// Definir o pino do termistor
#define TERMISTOR_PIN A2

// Definir os parâmetros do termistor
#define R0 100000 // resistência nominal do termistor em ohms
#define B 3950 // constante B do termistor em K
#define T0 298.15 // temperatura de referência em K (25°C)

// Definir a resistência do divisor de tensão em ohms
#define R_DIV 10000

// Definir o pino do SSR
#define SSR_PIN 3

// Definir a temperatura alvo em °C
#define TARGET_TEMP 200

// Definir os parâmetros do controlador PID
#define KP 2
#define KI 5
#define KD 1

double input, output, setpoint;
PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT);

void setup() {
  // Inicializar a comunicação serial com 9600 baud
  Serial.begin(9600);
  
  // Configurar o pino do SSR como saída
  pinMode(SSR_PIN, OUTPUT);
  
  // Inicializar o controlador PID
  setpoint = TARGET_TEMP;
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // Ler o valor analógico do termistor
  int adc = analogRead(TERMISTOR_PIN);
  
  // Calcular a tensão no termistor em volts
  float v = adc * (5.0 / 1023.0);
  
  // Calcular a resistência do termistor em ohms
  float r = R_DIV * (5.0 / v - 1.0);
  
  // Calcular a temperatura do termistor em K usando a equação de Steinhart-Hart
  float t = 1.0 / (1.0 / T0 + log(r / R0) / B);
  
  // Converter a temperatura para °C
  input = t - 273.15;
  
  // Executar o controlador PID
  myPID.Compute();
  
  // Controlar a hotend usando o SSR e o valor de saída do controlador PID
    // Controlar a hotend usando o SSR e o valor de saída do controlador PID
  analogWrite(SSR_PIN, output);
  
  // Printar a temperatura em °C no monitor Serial com uma casa decimal
  Serial.print("Temperatura: ");
  Serial.print(input, 1);
  Serial.println(" C");
  
  // Aguardar um intervalo de tempo
  delay(500);
}
