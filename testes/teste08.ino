
// Bibliotecas
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Stepper.h>

// Pinos do adaptador HeSal
#define S 13
#define D 12
#define E 8

// Pinos do encoder rotativo KY-040
#define CLK 2
#define DT 5
#define SW 4

// Mudança entre a VEL e o TEMP
bool mudandoVEL = true;
bool mudandoTEMP = false;

#define passosPorRevolucao 240

int posicao_antiga = 0;

// Objeto do encoder rotativo
Encoder r(CLK, DT);

// Objeto do visor LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Função do motor
Stepper motor(passosPorRevolucao, D, S);

// Variáveis globais
int velocidade = 700; // Velocidade inicial do motor em passos por segundo
int intervalo = 800 / velocidade; // Intervalo entre os passos em milissegundos
unsigned long tempo_anterior = 0; // Tempo anterior para controlar o intervalo dos passos
bool habilitado = true; // Estado de habilitação do motor
long posicao_anterior = 0; // Posição anterior do encoder rotativo

// Definição do caractere personalizado para o símbolo de graus
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
  int velocidade_inicial = 400;
  motor.setSpeed(400);

  Serial.begin(9600);
  // Inicializa o visor LCD I2C
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, graus);
  LCDVel();

  // Inicializa o botão do encoder rotativo com uma função de callback para detectar os cliques
  pinMode(SW, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(SW), Clique, CHANGE);
}
// Função setup
void loop() {

  // Verificar se o botão foi precionado
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


  // Lê a posição atual do encoder rotativo e verifica se houve alguma mudança
  long posicao_atual = r.read();

  // Atualiza o visor LCD com a velocidade atual do motor a cada meio segundo
  static unsigned long tempo_lcd = millis();
  if (millis() - tempo_lcd > 1000 && posicao_atual != posicao_antiga) {
    if (mudandoVEL) {
      LCDVel();
    } else {
      LCDTemp();
    }
    tempo_lcd = millis();
    posicao_antiga = posicao_atual;
  }

  // Gira o motor de acordo com a velocidade e o intervalo definidos
  if (habilitado && millis() - tempo_anterior > intervalo) {
    motor.setSpeed(velocidade);
    tempo_anterior = millis();
  }

  if (posicao_atual != posicao_anterior) {
    // Se a posição atual for maior que a anterior, significa que o encoder rotativo girou no sentido horário
    if (posicao_atual > posicao_anterior) {
      velocidade += 10; // Aumenta a velocidade do motor em 10 passos por segundo
      if (velocidade > 2000) velocidade = 2000; // Limita a velocidade máxima em 1000 passos por segundo
      intervalo = 2000 / velocidade; // Atualiza o intervalo entre os passos
    }
    // Se a posição atual for menor que a anterior, significa que o encoder rotativo girou no sentido anti-horário
    else if (posicao_atual < posicao_anterior) {
      velocidade -= 10; // Diminui a velocidade do motor em 10 passos por segundo
      if (velocidade < 0) velocidade = 0; // Limita a velocidade mínima em 0 passos por segundo
      intervalo = 2000 / velocidade; // Atualiza o intervalo entre os passos
    }
    posicao_anterior = posicao_atual; // Atualiza a posição anterior do encoder rotativo
    Serial.print("Posição: ");
    Serial.println(posicao_anterior);
  }
  motor.step(passosPorRevolucao);
}

void LCDVel() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">  VEL: ");
  lcd.print(velocidade);
  lcd.setCursor(0, 1);
  lcd.print("  TEMP: ");
  lcd.print("232");
  lcd.write(byte(0));
  lcd.print("C");
}

void LCDTemp(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   VEL: ");
  lcd.print(velocidade);
  lcd.setCursor(0, 1);
  lcd.print("> TEMP: ");
  lcd.print("232");
  lcd.write(byte(0));
  lcd.print("C");
}
