#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Stepper.h>

#define CLK 2
#define DT 5
#define SW 4

#define passosPorRevolucao 240

bool mudandoVEL = true;
bool mudandoTEMP = false;

Encoder r(CLK,DT);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Stepper motor(passosPorRevolucao, 12, 13);

long posicao_atual_M = 0;
long posicao_antiga_M = 0;
int velocidade = 700;
int intervalo_M = 1000 / velocidade;
long posicao_anterior_M = 0;
unsigned long tempo_anterior_M = 0;

long posicao_atual_E = 0;
long posicao_antiga_E = 0;
int temperatura = 1;
int temp_atual = 0;
int intervalo_E = 1000 / temperatura;
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

void setup() {
  motor.setSpeed(velocidade);
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, graus);
  LCDVel();
  pinMode(SW, INPUT_PULLUP);
}

void loop() {
  int estadoDoBotao = digitalRead(SW);
  
  if (estadoDoBotao != 1) {
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
      velocidade = alterandoParametro(posicao_atual_M, posicao_antiga_M, velocidade, intervalo_M, 10, 1100); 
      LCDVel(); // Atualiza o display LCD somente quando a posição do encoder muda
    }
  } else {
    posicao_atual_E = r.read();
    if (posicao_atual_E != posicao_antiga_E) {
      temperatura = alterandoParametro(posicao_atual_E, posicao_antiga_E, temperatura, intervalo_E, 10, 300);
      LCDTemp(); // Atualiza o display LCD somente quando a posição do encoder muda
    }
  }
  motor.setSpeed(velocidade);
  motor.step(passosPorRevolucao);
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
    parametro = constrain(parametro + (posicao_atual > posicao_anterior ? passo : -passo), 0, limite);
    intervalo = 2000 / parametro;
    posicao_anterior = posicao_atual;
  }
  return parametro;
}
