// Definir o pino do termistor
#define TERMISTOR_PIN A2
#define rele 5

// Definir os parâmetros do termistor
#define R0 100000 // resistência nominal do termistor em ohms
#define B 3950 // constante B do termistor em K
#define T0 298.15 // temperatura de referência em K (25°C)

// Definir a resistência do divisor de tensão em ohms
#define R_DIV 10000

// Definir o número de amostras para a média móvel
#define N_SAMPLES 10

void setup() {
  pinMode(rele, OUTPUT);
  //digitalWrite(rele, HIGH);
  // Inicializar a comunicação serial com 9600 baud
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
  
  // Printar a temperatura em °C no monitor Serial com uma casa decimal
  Serial.print("Temperatura: ");
  Serial.print(c_avg, 1);
  Serial.println(" C");
  
  // Aguardar um intervalo de tempo
  delay(500);

  if(c_avg <= 70)
  {
    digitalWrite(rele, LOW);
  } else {
    digitalWrite(rele, HIGH);
  }
}
