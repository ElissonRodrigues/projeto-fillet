/* Pins used for control signals */
#define ENABLE 8
#define DIRECTION 9
#define STEP 10

#define FORWARD HIGH
#define REVERSE LOW

/* Change this values to alter the clock speed */
#define SPEED 1

void setup() 
{
  pinMode(ENABLE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(STEP, OUTPUT);

  /* Pull the enable pin low to enable the driver */
  digitalWrite(ENABLE, LOW);
}


void loop() 
{
  /* The the rotational direction to the forward direction */
  digitalWrite(DIRECTION, REVERSE);

  /* Keep stepping the motor in an infinite loop */
  while(1)
  {
    digitalWrite(STEP, HIGH);                 
    digitalWrite(STEP, LOW);    
    delay(SPEED);            
  }
}
