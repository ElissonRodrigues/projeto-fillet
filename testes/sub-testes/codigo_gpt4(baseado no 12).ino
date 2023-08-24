#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Stepper.h>

// Constantes para os pinos do Arduino
const int PINO_CLK = 2;
const int PINO_DT = 5;
const int PINO_SW = 4;

// Constantes para os parâmetros do motor
const int PASSOS_POR_REVOLUCAO = 240;
const int PASSO_VEL = 10;
const int LIMITE_VEL = 2000;
const int PASSO_TEMP = 10;
const int LIMITE_TEMP = 2000;

// Variáveis globais para armazenar os valores dos parâmetros
int velocidade = 700;
int temperatura = 1;
int temp_atual = 0;

// Variáveis globais para armazenar o estado e o sentido do encoder
bool mudandoVEL = true;
bool sentido_horario = true;

// Objetos para controlar o LCD, o encoder e o motor
LiquidCrystal_I2C lcd(0x27, 16, 2);
Encoder r(PINO_DT, PINO_CLK);
Stepper motor(PASSOS_POR_REVOLUCAO, 12, 13);

// Caractere personalizado para representar o símbolo de graus no LCD
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
  pinMode(PINO_SW, INPUT_PULLUP);
}

void loop() {
  lerBotao();
  
  if (mudandoVEL) {
    lerEncoder();
    atualizarVelocidade();
    LCDVel(); 
  } else {
    lerEncoder();
    atualizarTemperatura();
    LCDTemp(); 
  }
  
  girarMotor();
}

void lerBotao() {
  // Lê o estado do botão do encoder
  int estadoDoBotao = digitalRead(PINO_SW);
  
  // Se o botão for pressionado, alterna entre os modos de mudança de velocidade e temperatura
  if (estadoDoBotao != 1) {
    mudandoVEL = !mudandoVEL;
    delay(100);
  }
}

void lerEncoder() {
  // Lê a posição atual do encoder
  long posicao_atual = r.read();
  
  // Se a posição atual for diferente de zero, significa que o encoder foi girado
  if (posicao_atual != 0) {
    // Se a posição atual for positiva, significa que o encoder foi girado no sentido horário
    if (posicao_atual > 0) {
      sentido_horario = true;
    }
    // Se a posição atual for negativa, significa que o encoder foi girado no sentido anti-horário
    else {
      sentido_horario = false;
    }
    // Reseta a posição do encoder para zero
    r.write(0);
  }
}

void atualizarVelocidade() {
  // Se o encoder foi girado no sentido horário, aumenta a velocidade em um passo
  if (sentido_horario) {
    velocidade += PASSO_VEL;
    // Se a velocidade ultrapassar o limite, mantém o limite como valor máximo
    if (velocidade > LIMITE_VEL) velocidade = LIMITE_VEL;
  }
  // Se o encoder foi girado no sentido anti-horário, diminui a velocidade em um passo
  else {
    velocidade -= PASSO_VEL;
    // Se a velocidade ficar negativa, mantém zero como valor mínimo
    if (velocidade < 0) velocidade = 0;
  }
}

void atualizarTemperatura() {
  // Se o encoder foi girado no sentido horário, aumenta a temperatura em um passo
  if (sentido_horario) {
    temperatura += PASSO_TEMP;
    // Se a temperatura ultrapassar o limite, mantém o limite como valor máximo
    if (temperatura > LIMITE_TEMP) temperatura = LIMITE_TEMP;
  }
  // Se o encoder foi girado no sentido anti-horário, diminui a temperatura em um passo
  else {
    temperatura -= PASSO_TEMP;
    // Se a temperatura ficar negativa, mantém zero como valor mínimo
    if (temperatura < 0) temperatura = 0;
  }
}

void LCDVel() {
  // Limpa o LCD e posiciona o cursor no início da primeira linha
  lcd.clear();
  lcd.setCursor(0, 0);
  
  // Mostra o símbolo ">" para indicar que o modo de mudança de velocidade está ativo
  lcd.print(">");
  
  // Mostra o texto "VEL: " e o valor da velocidade
  lcd.print("VEL: ");
  lcd.print(velocidade);
  
  // Posiciona o cursor no início da segunda linha
  lcd.setCursor(0, 1);
  
  // Mostra o texto " TMP: " e os valores da temperatura atual e desejada
  lcd.print(" TMP: ");
  lcd.print(temp_atual);
  lcd.print("->");
  lcd.print(temperatura);
  
  // Mostra o caractere personalizado de graus e a letra "C"
  lcd.write(byte(0));
  lcd.print("C");
}

void LCDTemp() {
  // Limpa o LCD e posiciona o cursor no início da primeira linha
  lcd.clear();
  lcd.setCursor(0, 0);
  
  // Mostra o texto " VEL: " e o valor da velocidade
  lcd.print(" VEL: ");
  lcd.print(velocidade);
  
  // Posiciona o cursor no início da segunda linha
  lcd.setCursor(0, 1);
  
  // Mostra o símbolo ">" para indicar que o modo de mudança de temperatura está ativo
  lcd.print(">");
  
  // Mostra o texto "TMP: " e os valores da temperatura atual e desejada
  lcd.print("TMP: ");
  lcd.print(temp_atual);
  lcd.print("->");
  lcd.print(temperatura);
  
  // Mostra o caractere personalizado de graus e a letra "C"
  lcd.write(byte(0));
  lcd.print("C");
}

void girarMotor() {
  // Define a velocidade do motor de acordo com o valor da variável velocidade
  motor.setSpeed(velocidade);
  
  // Faz o motor girar uma revolução por ciclo
  motor.step(PASSOS_POR_REVOLUCAO);
}



