#include <TimerOne.h>
#define timer1_sample 10000    // thoi gian lay mau us(100ms)
#define CW 4
#define CCW 5
/*===========Truyen thong==================*/
bool stringComplete = false;
float st1;
String inputString = "", String1 = "";

/*===============var_speed==============*/
volatile unsigned long pulse = 0;
volatile float act_speed = 0, des_Speed = 0, P = 0, I = 0 , D = 0, PID = 0, pre_err = 0;
//volatile float Kp =0.03, Kd = 0, Ki = 0.001, PWM = 0, err=0;//Kp =0.025, Kd = 0.003, Ki = 0
volatile float Kp = 0.045, Kd = 0.001, Ki = 0.001, PWM = 0, err = 0;
unsigned long set_timer = 0, set_timer2 = 0;
/*===============var_position==============*/
int pos=0, set_pos=0,loi=0;
/*=============================*/
void encoder()
{
  pulse++;
}
/*=============================*/
void Control_PID()
{
  err = des_Speed - act_speed;
  P = Kp * err;
  I += (Ki * err) * 10 / 1000;
  if (I > 50)
  {
    I = 50;
  }
  if (I < -50)
  {
    I = -50;
  }
  D = Kd * (err - pre_err) * 50;
  PID = P + I + D;
  PWM = PWM + PID;
  if (PWM > 255)
  {
    PWM = 255;
  }
  if (PWM < 0)
  {
    PWM = 0;
  }
  pre_err = err;
  analogWrite(6, PWM);
}
/*=============================*/
void Control_speed()
{
  //  act_speed = (pulse / (334*25)); //vong/25ms
  //  act_speed=act_speed*1000;//vong/s
  //  act_speed=act_speed*60;
  act_speed = (pulse * 2400 / 334); //vong/ph
  pulse = 0;
  Control_PID();

}
/*================CODE POSITION=================*/
/*=============================*/
void quay(int nangluong)
{
  if(nangluong>0)
  {
    analogWrite(CCW,0);
    analogWrite(CW,nangluong);
  }
  if(nangluong<0)
  {
    analogWrite(CW,0);
    analogWrite(CCW,-nangluong);
  }
}
/*=============================*/
int pid( int loi , int kp, int ki, int kd)
{
  int dloi;
  static int loitr=0,iloi=0;
  int temp;
  
  dloi=loi-loitr;
  iloi+=loi;
  if(iloi>=100)
  {
    iloi = 100;
  }
  if(iloi<=-100)
  {
    iloi =- 100;
  }
  loitr=loi;
  temp = kp*loi +ki*iloi + kd*dloi;
  if(temp>=255)
  {
    temp = 255;
  }
  if(temp<=-255)
  {
    temp =- 255;
  }
  return temp;
}
/*==========================*/
void setup() {
  
  pinMode(2, INPUT_PULLUP);
  pinMode(CW, OUTPUT);//4
  pinMode(CCW, OUTPUT);//5
  pinMode(6, OUTPUT);
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  attachInterrupt(0, encoder, RISING);
  Timer1.initialize(timer1_sample);
  Timer1.attachInterrupt(Control_speed);
  set_timer = millis();//tao thoi gian truyen len may tinh
  set_timer2 = micros();
  Serial.begin(9600);
}
void convert()
{
  if (stringComplete)
  {
    if (inputString.charAt(0) == '@')
    {
      if (inputString.charAt(1) == '1') // Mode speed khung truyen @1#
      {
        k = 1;
      }
      if (inputString.charAt(1) == '2') // Mode position  @2#
      {
        k = 2;
      }
      if (inputString.charAt(2) == 'S')//SPEED @MS123#
      {
        st1 = inputString.indexOf('#');
        String1 = inputString;
        String1.remove(st1);
        String1.remove(0, 3);
        des_Speed = String1.toFloat();
        Serial.println(des_Speed);
        String1 = "";
        inputString = "";
        stringComplete = false;
      }
      if (inputString.charAt(2) == 'P')//POSITION @MP123#
      {
        st1 = inputString.indexOf('#');
        String1 = inputString;
        String1.remove(st1);
        String1.remove(0, 3);
        Set_pos = String1.toFloat();
        Serial.println(des_Speed);
        String1 = "";
        inputString = "";
        stringComplete = false;
      }
    }
  }
}

/*=============================*/
void loop() {
  convert();
  if (k == 1)
  {
    if (millis() - set_timer > 1000)
    {
      Serial.println(act_speed);
      set_timer = millis();
    }
  }
  if (k == 2)
  {
    Serial.println(pos);
    loi = set_pos - pos;
    quay(pid(loi, 3, 0, 1));
  }
}
/*=============================*/
void serialEvent()
{
  while (Serial.available())
  {
    char inchar = (char)Serial.read();
    inputString += inchar;
    if(inputString.charAt(0)!='@')
    {
      inputString="";
    }
    if (inchar == '#')
    {
      stringComplete = true;
    }
  }
}
