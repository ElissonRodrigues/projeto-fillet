#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Stepper.h>
#include <PID_v1.h>

//pino
#define CLK 2
#define DT 5
#define SW 4

#define passosPorRevolucao 240

bool mudandoVEL = true;
bool mudandoTEMP = false;

Encoder r(CLK, DT);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Stepper motor(passosPorRevolucao, 12, 13);

long posicao_atual_M = 0;
long posicao_antiga_M = 0;
int velocidade = 700;
int intervalo_M = 1000;
long posicao_anterior_M = 0;
unsigned long tempo_anterior_M = 0;

long posicao_atual_E = 0;
long posicao_antiga_E = 0;
int temperatura = 0;
int temp_atual = 0;
int temp_atual_old = 0;

int intervalo_E = 1000;
long posicao_anterior_E = 0;
unsigned long tempo_anterior_E = 0;

byte graus[8] = {
  B00111,
  B00101,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

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
#define HEATER_PIN 7

// Define os parâmetros do controlador PID
#define KP 5
#define KI 3
#define KD 2


double input, output, setpoint;
// Cria uma instância do controlador PID
PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT);

void setup() {
  motor.setSpeed(velocidade);
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, graus);
  LCDVel();
  pinMode(SW, INPUT_PULLUP);
  
  // Initialize the sensor of temperature and the heating element
  pinMode(TERMISTOR_PIN, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
  
  // Activate the PID controller
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  int estadoDoBotao = digitalRead(SW);
  setpoint = 80;

  if (estadoDoBotao != 1) {
    // Set the desired temperature
    
    mudandoVEL = !mudandoVEL;
    mudandoTEMP = !mudandoTEMP;
    if (mudandoVEL) {
      LCDVel();

    } else {
      LCDTemp();
      
    }
    delay(100);
  }

  if (mudandoVEL) {
    posicao_atual_M = r.read();
    if (posicao_atual_M != posicao_antiga_M) {
      velocidade = alterandoParametro(posicao_atual_M, posicao_antiga_M, velocidade, intervalo_M, 16, 1100);
      LCDVel(); // Atualiza o display LCD somente quando a posição do encoder muda
    }
  } else {
    posicao_atual_E = r.read();
    if (posicao_atual_E != posicao_antiga_E) {
      temperatura = alterandoParametro(posicao_atual_E, posicao_antiga_E, temperatura, intervalo_E, 5, 300);
      LCDTemp(); // Atualiza o display LCD somente quando a posição do encoder muda
    }
  }
  motor.setSpeed(velocidade);
  motor.step(passosPorRevolucao);

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

  myPID.Compute();
  // Lê a temperatura atual do sensor
  temp_atual = c_avg;
  
  input = temp_atual;

  if(temp_atual != temp_atual_old){
    Serial.print("Temperatura: ");
    Serial.print(temp_atual);
    //Serial.print("TEmperatura desejada: ");
    //Serial.print(setpoint);
    Serial.println(" ºC");
    Serial.print("PID: ");
    Serial.println(output);
    temp_atual_old = temp_atual;
  }
  
  analogWrite(HEATER_PIN, output);
 
}

void LCDVel() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">VEL: ");
  lcd.print(velocidade);
  lcd.setCursor(0, 1);
  lcd.print(" TMP: ");
  lcd.print(temp_atual);
  lcd.print("->");
  lcd.print(temperatura);
  lcd.write(byte(0));
  lcd.print("C");
}

void LCDTemp() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" VEL: ");
  lcd.print(velocidade);
  lcd.setCursor(0, 1);
  lcd.print(">TMP: ");
  lcd.print(temp_atual);
  lcd.print("->");
  lcd.print(temperatura);
  lcd.write(byte(0));
  lcd.print("C");
}

int alterandoParametro(long posicao_atual, long& posicao_anterior, int parametro, int& intervalo, int passo, int limite) {
  if (posicao_atual != posicao_anterior) {
    parametro = constrain(parametro - (posicao_atual > posicao_anterior ? passo : -passo), 0, limite);
    intervalo = 2000 / parametro;
    posicao_anterior = posicao_atual;
  }
  return parametro;
}
