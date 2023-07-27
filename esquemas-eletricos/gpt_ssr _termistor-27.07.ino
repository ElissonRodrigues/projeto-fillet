#include <PID_v1.h>

// Definir o pino do termistor
#define TERMISTOR_PIN A2

// Definir os parâmetros do termistor
#define R0 100000 // resistência nominal do termistor em ohms
#define B 3950 // constante B do termistor em K
#define T0 298.15 // temperatura de referência em K (25°C)

// Definir a resistência do divisor de tensão em ohms
#define R_DIV 10000

// Definir o número de amostras para a média móvel
#define N_SAMPLES 10

// Define as variáveis para o sensor de temperatura e o elemento de aquecimento
#define TEMP_SENSOR_PIN A0
#define HEATER_PIN 3

// Define os parâmetros do controlador PID
#define KP 2
#define KI 5
#define KD 1

// Define a temperatura desejada
#define SETPOINT 50

double input, output, setpoint;
// Cria uma instância do controlador PID
PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT);

void setup() {
  // Inicializa o sensor de temperatura e o elemento de aquecimento
  pinMode(TEMP_SENSOR_PIN, INPUT);
  pinMode(HEATER_PIN, OUTPUT);

  // Define a temperatura desejada
  setpoint = SETPOINT;

  // Ativa o controlador PID
  myPID.SetMode(AUTOMATIC);
  Serial.begin(9600);
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
  float c = t - 273.15;

  // Aplicar uma média móvel para suavizar a leitura
  static float c_avg = c; // valor médio inicializado com a primeira leitura
  c_avg = (c_avg * (N_SAMPLES - 1) + c) / N_SAMPLES; // atualizar o valor médio com a nova leitura

  // Lê a temperatura atual do sensor
  input = c_avg;

  Serial.print("Temperatura: ");
  Serial.print(input);
  Serial.println(" ºC");
  Serial.print("PID: ");
  Serial.println(output);

  // Executa o cálculo do controlador PID
  myPID.Compute();

  // Ajusta a potência do elemento de aquecimento de acordo com o resultado do cálculo do PID
  analogWrite(HEATER_PIN, output);
}
