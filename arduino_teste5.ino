#include <Stepper.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Definir pinos do Arduino
#define PinSDriver 12
#define PinDDriver 13

// Encoder - velocidade do motor 
const int encoderCLKPin = 2;
const int encoderDTPin = 3;
volatile long position = 18;
volatile int direction = 0;

const int PinControlMosfet = 6;

// Motor Stepper
const int passosPorRevolucao = 360;
Stepper motor(passosPorRevolucao, PinDDriver, PinSDriver);

// Velocidades do Motor
const int velocidadeMinima = 18;
const int velocidadeMaxima = 700;
int velocidadeAtual = 0;

// Controle de Temperatura
const int pinSensorTemp = A0;
float setpointTemp = 25.0;
const float kp = 0.09;
const float ki = 1;
const float kd = 8;
const int period = 100;

float error_previo = 0;
float integral = 0;

const unsigned long codigoBotaoPrev = 0x12345678;
const unsigned long codigoBotaoNext = 0x87654321;

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Endereço I2C do LCD: 0x27
bool lcdNeedsUpdate = true;

// Variáveis do agendamento de tarefas
unsigned long previousMotorTime = 0;
const unsigned long motorInterval = 100;  // Intervalo de atualização do motor (em milissegundos)

unsigned long previousTempTime = 0;
const unsigned long tempInterval = 1000;  // Intervalo de leitura e controle de temperatura (em milissegundos)

void setup() { 
  velocidadeAtual = 700;
  motor.setSpeed(velocidadeAtual);
  Serial.begin(9600);

  pinMode(PinControlMosfet, OUTPUT);
  
  pinMode(encoderCLKPin,INPUT);
  pinMode(encoderDTPin, INPUT);
  digitalWrite(encoderCLKPin, HIGH);
  digitalWrite(encoderDTPin, HIGH);

  
  attachInterrupt(digitalPinToInterrupt(encoderCLKPin), ChangeEncoder, CHANGE);
  
  if (velocidadeAtual < velocidadeMinima || velocidadeAtual > velocidadeMaxima) {
    velocidadeAtual = velocidadeMaxima;
  }

  motor.setSpeed(velocidadeAtual);

  Wire.begin(); // Inicializa a comunicação I2C
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
}


void loop() {
     
  //velocidadeAtual = position;
  Serial.print(velocidadeAtual);
  
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" Direction: ");
  Serial.println(direction);
  
  // Executar tarefa do motor
  unsigned long currentMillis = millis();
  if (currentMillis - previousMotorTime >= motorInterval) {
    previousMotorTime = currentMillis;
    
    motor.setSpeed(velocidadeAtual);
    motor.step(passosPorRevolucao);
  }

  // Executar tarefa de controle de temperatura
  if (currentMillis - previousTempTime >= tempInterval) {
    previousTempTime = currentMillis;

    float temperatura = lerTemperatura();
    float erro = setpointTemp - temperatura;

    float p = kp * erro;
    integral += ki * erro * tempInterval / 1000.0;
    float d = kd * (erro - error_previo) / (tempInterval / 1000.0);

    float controle = p + integral + d;

    if (controle > 0) {
      digitalWrite(PinControlMosfet, HIGH);
    } else {
      digitalWrite(PinControlMosfet, LOW);
    }

    error_previo = erro;
  }

  // Atualizar o LCD
    if(lcdNeedsUpdate){
      lcd.clear();
  
      lcd.print("Temp: ");
      lcd.print(lerTemperatura());
      lcd.print(" C");
      lcd.print(" / ");
      lcd.print(setpointTemp);
      lcd.print(" C");
      
      lcd.setCursor(0, 1);
      lcd.print("Vel. Motor: ");
      lcd.print(velocidadeAtual);
      lcdNeedsUpdate = false;
    }
}

float lerTemperatura() {
  int valorLeitura = analogRead(pinSensorTemp);
  float tensao = valorLeitura * (5.0 / 1023.0);
  float temperatura = (tensao - 0.5) * 100.0;

  return temperatura;
}

void ChangeEncoder() {
  if (digitalRead(encoderCLKPin) == digitalRead(encoderDTPin)) {
    if(position > 18){
      position--;
    }
    direction = -1;
  } else {
    if(position < 700){
      position++;
    }
    direction = 1;
  }
}
