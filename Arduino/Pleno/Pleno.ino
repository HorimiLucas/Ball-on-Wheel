#include <Wire.h>
#include <avr/pgmspace.h>
#define RPWM 6
#define LPWM 5
#define REN 9
#define LEN 8
#include <math.h>
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

unsigned long tempo;
long ePos = 0;
long unsigned int tmp = 0;
int ciclos = 0;
int pwr;
int dir = 0;
float dist_atual = 0;
int dist_inicial = 0;
int vetor[3][1000];
float pos = 0;
float pos_1 = 0;
float ang = 0;
float cont = 0;
float ang_1 = 0;
const PROGMEM float conv_ang = (2 * M_PI) / 800;
const PROGMEM float conv_pos = (M_PI * 0.024) / 1600; 
const PROGMEM float conv_sai = 255 / 24;
float target = 0;
float target_1 = 0;
float e = 0;
const PROGMEM float K[4] = { 129.71  ,   -0.81852   ,    18.865 ,     -1.2716};//trocar
const PROGMEM float L1[4][4] = { 
  {   1.0096       ,     0   ,  0.020064 , -1.8666E-05},
  {  0.00096257      ,      1  , 6.4173E-06 ,     0.01992},
{   0.95951   ,         0 ,       1.0096 ,  -0.0018671},
  {    0.096283       ,     0  , 0.00096257 ,     0.99206},
};
const PROGMEM float B[4] = {  0.0002684,    0.0011438,     0.026847 ,     0.11423}; //trocar
const PROGMEM float L[4][2] = { //trocar
    {  -1.5159  ,   -0.25556},
  { -0.0015304   ,   -1.9162},
{     47.921    ,   5.0114},
  {  4.7707   ,    1.8207},
};
float Xv[4] = {0, 0, 0, 0};
float Xv_1[4] = {0, 0, 0, 0};
float uo = 0;
float uo_1 = 0;
float l = 0.150;


union u_tag1 {  
  byte b[4];  
  long LePos;  
} u1;


void setup() {
  tmp = 0;
  ciclos = 0;
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(LEN, OUTPUT);
  pinMode(REN, OUTPUT);

  Wire.setClock(400000);
  Wire.begin();        
  Serial.begin(115200); 


  while (! Serial) {
    ;
  }

  if (!lox.begin()) {
    while (1);
    }
  
  dist_inicial = lox.readRange();
  float ang = 0;
  lox.startRangeContinuous();


  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 1249;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

}



ISR(TIMER1_COMPA_vect) {
  tmp++;

  tempo = millis();



  cont = 500;
  int dez_sec = 4883;


 Serial.print(dist_inicial);
 Serial.print(" , ");
  Serial.print(dist_atual);
  Serial.print(" , ");
  Serial.print(pos);
   Serial.print(" , ");
  Serial.print(ang);
   Serial.print(" , ");
  Serial.print(uo);
   Serial.print(" , ");
   Serial.print(Xv[0]);
   Serial.print(" , ");
    Serial.print(Xv[1]);
    Serial.print(" , ");
      Serial.println(tmp*20);



  if (ciclos <= 1000 && tmp % 10 == 0) {
    vetor[0][ciclos] = ciclos;
    vetor[1][ciclos] = u1.LePos;
    ciclos++;
  }


  pos = ((dist_inicial - (dist_atual)) * (0.001) / (l));
  ang = (u1.LePos * conv_ang);

  Xv[0] = L1[0][0]*Xv_1[0] + L[0][1]*Xv_1[1] + L[0][2]*Xv_1[2]+ L[0][3]*Xv_1[3] + B[0]*uo_1 + L[0][0] * pos_1 + L[0][1] * ang_1;
  Xv[1] = L1[1][0]*Xv_1[0] + L[1][1]*Xv_1[1] + L[1][2]*Xv_1[2]+ L[1][3]*Xv_1[3] + B[1]*uo_1 + L[1][0] * pos_1 + L[1][1] * ang_1;
  Xv[2] = L1[2][0]*Xv_1[0] + L[2][1]*Xv_1[1] + L[2][2]*Xv_1[2]+ L[2][3]*Xv_1[3] + B[2]*uo_1 + L[2][0] * pos_1 + L[2][1] * ang_1;
  Xv[3] = L1[3][0]*Xv_1[0] + L[3][1]*Xv_1[1] + L[3][2]*Xv_1[2]+ L[3][3]*Xv_1[3] + B[3]*uo_1 + L[3][0] * pos_1 + L[3][1] * ang_1;

  uo = - (Xv[0]*K[0] + Xv[1]*K[1] + Xv[2]*K[2] + Xv[3]*K[3]);

  e = uo;

  pos_1 = pos;
  ang_1 = ang;
  Xv_1[0] = Xv[0];
  Xv_1[1] = Xv[1];
  Xv_1[2] = Xv[2];
  Xv_1[3] = Xv[3];
  uo_1 = uo;



  pwr = abs(e * conv_sai);
  if ( pwr > 255 ) {
    pwr = 255;
  }
  if ( pwr < 0 ) {
    pwr = 0;
  }
  if (e < 0) {
    dir = -1;
  }
  if (e > 0) {
    dir = 1;
  }


  setMotor(dir, pwr, RPWM, LPWM, REN, LEN);

}

void setMotor(int dir, int pwmVal, int rpwm, int lpwm, int ren, int len) {

if (pos < -0.015 || pos > 0.015) {
  if (dir == 1) {
    digitalWrite(ren, HIGH);
    analogWrite(rpwm, pwmVal);

    digitalWrite(len, HIGH);
    analogWrite(lpwm, 0);
  }
  else if (dir == -1) {
    digitalWrite(ren, HIGH);
    analogWrite(rpwm, 0);
    digitalWrite(len, HIGH);
    analogWrite(lpwm, pwmVal);
  }
  else {
    analogWrite(rpwm, 0);
    digitalWrite(ren, LOW);
    analogWrite(lpwm, 0);
    digitalWrite(len, LOW);
  }
}
 




}
void loop()

{
  if (lox.isRangeComplete()) {
    dist_atual = lox.readRange();
  }
  Wire.requestFrom(8, 4);   
  for (int i = 0; i < 4; i++) 
  {
    u1.b[i] = Wire.read();
  }

  if (target != target_1) {
//    Serial.println(target);
  }
  target_1 = target;


}
