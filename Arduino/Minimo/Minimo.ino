#include <Wire.h>
#include <VL53L0X.h>
#include <avr/pgmspace.h>
#define RPWM 6
#define LPWM 5
#define REN 9
#define LEN 8
#include <math.h>
VL53L0X sensor;

long ePos = 0;
int cont=0;
int tmp=0;
int ciclos=0;
int pwr;
int dir=0;
int bolinha=0;
int vetor[3][1000];
float pos=0;
float pos_1=0;
float ang=0;
float ang_1=0;
float vel=0;
float vel_ang=0;
const PROGMEM float conv_ang=(2*M_PI)/800;
const PROGMEM float conv_pos=(M_PI*0.024)/1600;
const PROGMEM float conv_sai=255/24;
float target= 0;
float target_1= 0;
float e=0;
const PROGMEM float K[4]={      103.32       ,    -1   ,    19.192  ,    -1.6355}; //trocar
const PROGMEM float Aa[2][2]={//trocar
  {     0.93663  , -0.0001888},
  {   1.017E-05  ,    0.93672},
};
const PROGMEM float Bp[2]={   0.0025479 ,    0.010007};//trocar
const PROGMEM float Ke[2][2]={//trocar
   {         30.972  ,  0.0056654},
   {       0.0027043    ,   30.559},
};
const PROGMEM float Ab[2][2]={//trocar
  {   -30.912  , -0.0056654},
  {     0.012471   ,   -30.559},
};
float Xv[2]={0,0};
float Xv_1[2]={0,0};
float uo=0;
float uo_1=0;
float l=0.249;


union u_tag1 {  
   byte b[4];  
   long LePos; 
} u1;


void setup() {
   tmp=0;
  ciclos=0;
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(LEN,OUTPUT);
  pinMode(REN,OUTPUT);
  
  Wire.setClock(400000);
  Wire.begin();         
  Serial.begin(115200); 

  sensor.setMeasurementTimingBudget(200000);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

 
  TIMSK2 = (TIMSK2 & B11111110) | 0x01;
  TCCR2B = (TCCR2B & B11111000) | 0x04;

  Serial.println("Angu;Posica");
}



ISR(TIMER2_OVF_vect){
  
  tmp++;
  
  cont=500;
  int dez_sec=4883;
  if(tmp<cont){target=0;}
  if(tmp>cont&&tmp<dez_sec){target=-0.05;}
  if(tmp>dez_sec){target=0.05;}

    Serial.print(bolinha);
    Serial.print(",");
     Serial.print(pos);
       Serial.print(",");
        Serial.println(ang);


  if(ciclos<=1000 && tmp%10==0){
    vetor[0][ciclos]=ciclos;
    vetor[1][ciclos]=u1.LePos;
    ciclos++;
    }
  
pos = (((235-bolinha)-10)*(0.001))/(l); 
ang=u1.LePos*conv_ang;



Xv[0]=Aa[0][0]*Xv_1[0]+Aa[0][1]*Xv_1[1]+ Bp[0]*uo_1 + Ke[0][0]*pos + Ke[0][1]*ang + Ab[0][0]*pos_1 + Ab[0][1]*ang_1;
Xv[1]=Aa[1][0]*Xv_1[0]+Aa[1][1]*Xv_1[1]+ Bp[1]*uo_1 + Ke[1][0]*pos + Ke[1][1]*ang + Ab[1][0]*pos_1 + Ab[1][1]*ang_1;

  if( Xv[0] > 255 ){Xv[0] = 255;}
  if( Xv[0] < -255 ){Xv[0] = -255;} 

  if( Xv[1] > 255 ){Xv[1] = 255;}
  if( Xv[1] < -255 ){Xv[1] = -255;} 


 uo = target - (pos*K[0]+ang*K[1]+Xv[0]*K[2]+Xv[1]*K[3]);

  if( uo > 255 ){uo = 255;}
  if( uo < -255 ){uo = -255;} 
  
 e=uo;
 
  pos_1=pos;
  ang_1=ang;
  Xv_1[0]=Xv[0];
  Xv_1[1]=Xv[1];
  uo_1=uo;



  pwr = abs(e*conv_sai); 
  if( pwr > 255 ){pwr = 255;}
  if( pwr < 0 ){pwr = 0;} 
  if(e<0){dir = -1;} 
  if(e>0){dir = 1;}  


setMotor(dir,pwr,RPWM,LPWM,REN,LEN);

}

void setMotor(int dir, int pwmVal, int rpwm, int lpwm, int ren, int len){

  if(dir == 1){
    digitalWrite(ren,HIGH);
    analogWrite(rpwm,pwmVal);
    digitalWrite(len,HIGH);
    analogWrite(lpwm,0);
    
  }
  else if(dir == -1){
    digitalWrite(ren,HIGH);
    analogWrite(rpwm,0);
    digitalWrite(len,HIGH);
    analogWrite(lpwm,pwmVal);
  }
  else{
    analogWrite(rpwm,0);
    digitalWrite(ren,LOW);
    analogWrite(lpwm,0);
    digitalWrite(len,LOW);
  }  


}

void loop()
{
    Wire.requestFrom(8, 4);    
  
  for (int i=0; i<4; i++) 
  {
      u1.b[i] = Wire.read();  
  }

 bolinha = sensor.readRangeSingleMillimeters()-30;
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT");}

  
  if(target!=target_1){Serial.println(target);}
  target_1=target;


}
