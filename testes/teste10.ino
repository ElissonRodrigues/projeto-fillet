
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

#define passosPorRevolucao 240

// Mudança entre a VEL e o TEMP
bool mudandoVEL = true;
bool mudandoTEMP = false;

// Objeto do encoder rotativo
Encoder r(CLK, DT);

// Objeto do visor LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Função do motor
Stepper motor(passosPorRevolucao, D, S);

// Variáveis globais do MOTOR
long posicao_atual_M = 0; //Posição do Encoder (MOTOR)
int posicao_antiga_M = 0;
int velocidade = 700; // Velocidade inicial do motor em passos por segundo
int intervalo_M = 800 / velocidade; // intervalo_M entre os passos em milissegundos
long posicao_anterior_M = 0; // Posição anterior do encoder rotativo
unsigned long tempo_anterior_M = 0; // Tempo anterior para controlar o intervalo_M dos passos

// Variáveis globais da EXTRUSORA
long posicao_atual_E = 0;
int posicao_antiga_E = 0;
int temperatura = 1; // Temperatura desejada
int temp_atual = 0;
int intervalo_E = 800 / temperatura; // intervalo_E entre os passos em milissegundos
long posicao_anterior_E = 0; // Posição anterior do encoder rotativo
unsigned long tempo_anterior_E = 0; // Tempo anterior para controlar o intervalo_M dos passos

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
  if (mudandoVEL) {
    posicao_atual_M = r.read();
    velocidade = alterandoVel();
  } else {
    posicao_atual_E = r.read();
    temperatura = alterandoTemp();
  }



  static unsigned long tempo_lcd = millis();
  if (millis() - tempo_lcd > 1000 && (posicao_atual_M != posicao_antiga_M || posicao_atual_E != posicao_antiga_E)) {
    if (mudandoVEL) {
      LCDVel();
      posicao_antiga_M = posicao_atual_M;
    } else {
      LCDTemp();
      posicao_antiga_E = posicao_atual_E;
    }
    tempo_lcd = millis();
  }

  // Gira o motor de acordo com a velocidade e o intervalo definidos
  if (mudandoVEL) {
    if (millis() - tempo_anterior_M > intervalo_M) {
      motor.setSpeed(velocidade);
      tempo_anterior_M = millis();
    } else {
      if (mudandoVEL) {
        if (millis() - tempo_anterior_M > intervalo_M) {
          motor.setSpeed(velocidade);
          tempo_anterior_M = millis();
        }
      } else {
        if (millis() - tempo_anterior_E > intervalo_E) {
          Serial.println("temperatura");
          tempo_anterior_E = millis();
        }
      }
    }
  }

if (mudandoVEL) {
  alterandoVel();
} else {
  alterandoTemp();
}

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

int alterandoVel() {
  if (posicao_atual_M != posicao_anterior_M) {
    // Se a posição atual for maior que a anterior, significa que o encoder rotativo girou no sentido horário
    if (posicao_atual_M > posicao_anterior_M) {
      velocidade += 10; // Aumenta a velocidade do motor em 10 passos por segundo
      if (velocidade > 2000) velocidade = 2000; // Limita a velocidade máxima em 1000 passos por segundo
      intervalo_M = 2000 / velocidade; // Atualiza o intervalo_M entre os passos
    }
    // Se a posição atual for menor que a anterior, significa que o encoder rotativo girou no sentido anti-horário
    else if (posicao_atual_M < posicao_anterior_M) {
      velocidade -= 10; // Diminui a velocidade do motor em 10 passos por segundo
      if (velocidade < 0) velocidade = 0; // Limita a velocidade mínima em 0 passos por segundo
      intervalo_M = 2000 / velocidade; // Atualiza o intervalo_M entre os passos
    }
    posicao_anterior_M = posicao_atual_M; // Atualiza a posição anterior do encoder rotativo
  }
  return velocidade;
}

int alterandoTemp() {
  if (posicao_atual_E != posicao_anterior_E) {
    if (posicao_atual_E > posicao_anterior_E) {
      temperatura += 10;
      if (temperatura > 2000) temperatura = 2000;
      intervalo_E = 2000 / temperatura;
    }
    else if (posicao_atual_E < posicao_anterior_E) {
      temperatura -= 10;
      if (temperatura < 0) temperatura = 0;
      intervalo_E = 2000 / temperatura;
    }
    posicao_anterior_E = posicao_atual_E;
  }
  return temperatura;  // Retorna o valor atualizado de temperatura
}
