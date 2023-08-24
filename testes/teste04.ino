#include <Stepper.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Definir pinos do Arduino
#define PinSDriver 12
#define PinDDriver 13

// Potenciometro 
#define potData A1 // Pin DT
#define potClock 4 // Pin CLK
int valPot; // Velocidade do potenciometro

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
  motor.setSpeed(velocidadeAtual);
  Serial.begin(9600);

  pinMode(PinControlMosfet, OUTPUT);

  velocidadeAtual = EEPROM.read(0);
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
  valPot = digitalRead(potClock);
  

  valPot = map(valPot,0,1023,0,200);

  Serial.println(valPot);
  
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

    /*Serial.print("Temperatura desejada: ");
    Serial.println(setpointTemp);*/

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
