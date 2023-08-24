#include <Stepper.h>
#include <IRremote.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Definir pinos do Arduino
#define PinSDriver 52
#define PinDDriver 53

#define PinIRRemote 51
const int PinControlMosfet = 6;

#define saltosDeVelocidade 30

// Motor Stepper
const int passosPorRevolucao = 360;
Stepper motor(passosPorRevolucao, PinDDriver, PinSDriver);

// Velocidades do Motor
const int velocidadeMinima = 18;
const int velocidadeMaxima = 700;
int velocidadeAtual = 0;

IRrecv receptorIR(PinIRRemote);
decode_results resultadosIR;

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

// Variáveis do agendamento de tarefas
unsigned long previousMotorTime = 0;
const unsigned long motorInterval = 100;  // Intervalo de atualização do motor (em milissegundos)

unsigned long previousTempTime = 0;
const unsigned long tempInterval = 1000;  // Intervalo de leitura e controle de temperatura (em milissegundos)

void setup() {
  motor.setSpeed(velocidadeAtual);
  receptorIR.enableIRIn();
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

    Serial.print("Temperatura desejada: ");
    Serial.println(setpointTemp);

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

    // Atualizar o LCD
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
  }

  // Executar tarefa do receptor IR
  if (receptorIR.decode(&resultadosIR)) {
    lcd.clear();

    if (resultadosIR.value == codigoBotaoPrev) {
      setpointTemp--;
    }
    else if (resultadosIR.value == codigoBotaoNext) {
      setpointTemp++;
    }

    lcd.setCursor(0, 0);
    lcd.print("Motor ");

    if (resultadosIR.value == 16754775) {
      velocidadeAtual += saltosDeVelocidade;
      lcd.print("(+): ");
    }
    else if (resultadosIR.value == 16769055) {
      velocidadeAtual -= saltosDeVelocidade;
      lcd.print("(-): ");
    }

    if (velocidadeAtual < velocidadeMinima) {
      velocidadeAtual = velocidadeMinima;
    } else if (velocidadeAtual > velocidadeMaxima) {
      velocidadeAtual = velocidadeMaxima;
    }

    receptorIR.resume();
    lcd.setCursor(0, 1);
    lcd.print(velocidadeAtual);
    delay(500);
  }
}

float lerTemperatura() {
  int valorLeitura = analogRead(pinSensorTemp);
  float tensao = valorLeitura * (5.0 / 1023.0);
  float temperatura = (tensao - 0.5) * 100.0;

  return temperatura;
}
