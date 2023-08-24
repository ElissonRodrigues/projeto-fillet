#include <PID_v1.h>

#define TERMISTOR_PIN A2
#define R0 100000
#define B 3950
#define T0 298.15
#define R_DIV 10000
#define N_SAMPLES 10

#define SSR_PIN 3 // Pino do Arduino conectado ao SSR (pino de controle)

const float TEMPERATURE_REFERENCE = 200.0; // Temperatura de referência em graus Celsius
double kp = 2.0;  // Ganho proporcional do PID
double ki = 5.0;  // Ganho integral do PID
double kd = 1.0;  // Ganho derivativo do PID

double temperature = 0.0;
double output = 0.0;

PID myPID(&temperature, &output, &TEMPERATURE_REFERENCE, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);
  
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW); // Certifica-se de que o SSR está desligado no início
  
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // Leitura do termistor
  int adc = analogRead(TERMISTOR_PIN);
  float v = adc * (5.0 / 1023.0);
  float r = R_DIV * (5.0 / v - 1.0);
  float t = 1.0 / (1.0 / T0 + log(r / R0) / B);
  float c = t - 273.15;
  static float c_avg = c;
  c_avg = (c_avg * (N_SAMPLES - 1) + c) / N_SAMPLES;

  // Controle do aquecedor usando o PID
  temperature = c_avg; // Usamos a temperatura média suavizada como entrada para o PID
  myPID.Compute();
  digitalWrite(SSR_PIN, output > 0 ? HIGH : LOW); // Aciona o SSR baseado na saída do PID

  // Exibir informações no Monitor Serial
  Serial.print("Temperatura: ");
  Serial.print(c_avg, 1);
  Serial.print(" C - Saída PID: ");
  Serial.print(output);
  Serial.print(" - Aquecedor: ");
  Serial.println(output > 0 ? "Ligado" : "Desligado");

  delay(1000);
}
