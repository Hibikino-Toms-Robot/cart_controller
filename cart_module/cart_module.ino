#include "PinChangeInterrupt.h"
#include <MsTimer2.h>

//Motor_Pin
#define motorDirection 9
#define runbrake 10
#define motorEnabled 11


#define pwm 5
#define encoder 1
//don't use
#define AlarmReset 7
#define INT 8

//Bumper_PIN
#define Forward_Bumper_PIN 2
#define Behind_Bumper_PIN 3

//Ultrasonic_Sensors_Pin 
#define TRIG 12          //Trig(トリガー端子)：超音波の出力用の信号を送信
#define ECHO 13          //Echo(エコー端子)：超音波の入力信号を受信
#define threshold 15.0   //2台間の距離の閾値(cm)

//motor_patam
int ENC_COUNTS = 450 ;            //エンコーダー一回転での回転数
float WHEEL_DIAMETER = 0.05;      //車輪半径[m]
float x = 0.0 ;                   //スタート位置を基準とした移動距離[m]
float tmp_dist = 0.0 ;
volatile unsigned long pulse = 0; //パルス立ち上がり数
int PWM = 30;                     //PWM(モータへの)
int direction = 0;                //進行方向


//Bumper_Sensors_param
volatile unsigned long lastInterruptTime = 0;
const unsigned long debounceDelay = 1000; // デバウンス処理の待機時間

//Ultrasonic_Sensors_param
float speed_of_sound = 331.5 + 0.6 * 25; // 25℃の気温の想定
const int k = 0.8;
float Dist = 0;


//receive_data_param
uint8_t count = 0;
char data[32];
bool readflag = false ;

//mode_param
int status = 0; //今の状態を示すフラグ

//read_data_param
float target_distanse;
float distance_cm;

void setup() {
  //motor_setup
  pinMode(motorEnabled, OUTPUT);     // H:START L:STOP
  pinMode(runbrake, OUTPUT);         // H:RUN L:BREAK(Instant stop)
  pinMode(motorDirection, OUTPUT);   // H:CW(Right) L:CCW(Left)
  pinMode(INT, OUTPUT);              // Don't use. Always LOW.
  pinMode(AlarmReset, OUTPUT);       // Don't use. Always LOW.
  pinMode(pwm, OUTPUT);              // This pin is Analog pin. Output 0～5V.
  digitalWrite(motorEnabled, HIGH);  //Keep the motor stopped at first.
  digitalWrite(runbrake, HIGH);
  digitalWrite(INT, HIGH);           //don't use    
  digitalWrite(AlarmReset, HIGH);    //don't use

  // Ultrasonic_Sensors_setup［HC-SR04］
  pinMode(ECHO, INPUT);            // Trig(トリガー端子)：超音波の出力用の信号を送信
  pinMode(TRIG, OUTPUT);           // Echo(エコー端子)：超音波の入力信号を受信
 
  //Bumper_setup
  pinMode(Forward_Bumper_PIN, INPUT_PULLUP);
  pinMode(Behind_Bumper_PIN, INPUT_PULLUP);

  //interrupt_processing_setup
  attachPinChangeInterrupt(digitalPinToPCINT(encoder), Pulse_Counter, FALLING);
  attachPinChangeInterrupt(digitalPinToPCINT(Forward_Bumper_PIN), Forward_Bumper, FALLING);
  attachPinChangeInterrupt(digitalPinToPCINT(Behind_Bumper_PIN), Behind_Bumper, FALLING);
  MsTimer2::set(500, ultrasonic_sensor); //Timer割り込み
  MsTimer2::start();
  Serial.begin(115200);
}

/*Bumper_Sensor-----------------------------------------------------
チャタリング防止
ボタンの反応が一時的なものでなければ障害物にぶつかったと判定する
*/
void Forward_Bumper() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastInterruptTime >= debounceDelay) {
    lastInterruptTime = currentMillis;
    status = 1;
  }
}
void Behind_Bumper() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastInterruptTime >= debounceDelay) {
    lastInterruptTime = currentMillis;
    status = 2;
  }
}

/*----------------------------------------------------------------*/

/*
Ultrasonic_Sensor(超音波センサ)--------------------------------------------------------------------
*/
void ultrasonic_sensor(){
  digitalWrite(TRIG, LOW);  //両方のピンを初期化
  delayMicroseconds(1); 
  digitalWrite(TRIG, HIGH); //超音波パルスを発射
  delayMicroseconds(10);    //0us続ける(最低でも10usだそう)
  digitalWrite(TRIG, LOW);  //発射停止
  float duration = pulseIn(ECHO, HIGH); //duration : echoPinがHIGHになっている時間   //pulseIn():パルス（電流の流れた間隔)
  //［物体との距離］＝［速度(音速)］✕［時間］÷2  [cm]
  duration = duration / 2; //出てきた時間は往復の距離なので実際の距離は半分
  float currentDist = duration * speed_of_sound * 100 / 1000000 ; //m/s ⇨　cm/μs　に変更(durationがμsででるので単位を合わせてる)
  Dist = k*Dist + (1-k)*(float)currentDist; //RCフィルタ
  if(Dist > 400.0){
    //4m以上は測定できないので400に指定
    Dist = 400.0;
  }
  if(Dist < threshold){
    status = 3;
  }
}
/*----------------------------------------------------------------------------*/


/*
Functions for motors------------------------------------------------------------
*/
void Pulse_Counter(void){
  pulse++;
}

void Calculate_distance(void){
  float distance = direction * (pulse * 2.0 * 3.14159 * (WHEEL_DIAMETER / 2.0)) / ENC_COUNTS;
  x += distance; // 新しい自己位置を計算する
  tmp_dist += distance*1000; //[m]⇨[mm]
  pulse = 0 ;
}

void Forward_motor(int vel){
  digitalWrite(motorEnabled,LOW);
  digitalWrite(runbrake,LOW); 
  digitalWrite(motorDirection,HIGH);   
  analogWrite(pwm, abs(PWM));
}

void Reverse_motor(int vel){
  digitalWrite(motorEnabled,LOW); 
  digitalWrite(runbrake,LOW);  
  digitalWrite(motorDirection,LOW);     
  analogWrite(pwm, abs(PWM));
}

void Stop_motor(void){
  digitalWrite(motorEnabled,HIGH);   
  digitalWrite(runbrake,HIGH);   
  analogWrite(pwm, 0);
}

void Control_Motor(void){ 
  tmp_dist = 0.0 ;
  if (direction>0) {
    while(target_distanse > abs(tmp_dist)){
      Forward_motor(PWM);
      Calculate_distance();
      if(status == 1 || status == 2 || status == 3){
          break;
        }
    }
  }else{
    while(target_distanse > abs(tmp_dist)){
      Reverse_motor(PWM);
      Calculate_distance();
      if(status == 1 || status == 2 || status == 3){
          break;
        }
    }
  }
  Stop_motor();
  Calculate_distance();
}
/*-----------------------------------------------------------*/

float Conversion(int num1, int num2, int num3, int num4){
  // 目標位置計算  
  // 百の位
  int Num1 = num1 - '0';      
  if (Num1 > 0 && Num1 < 9){
    Num1 = Num1*100;           
  } else Num1 = 0;
  // 十の位
  int Num2 = num2 - '0';
  if (Num2 > 0 && Num2 < 10){
    Num2 = Num2*10;            
  } else Num2 = 0;
  // 一の位
  int Num3 = num3 - '0';
  if (Num3 > 0 && Num3 < 10){
    Num3 = Num3*1;            
  } else Num3 = 0;
  // 10^-1の位
  int Num4 = num4 - '0';
  if (Num4 > 0 && Num4 < 10){
    Num4 = Num4*1;            
  } else Num4 = 0;
  float Num = Num1 + Num2 + Num3 + Num4*0.1; 
  return Num;
}


void loop() {
  if(Serial.available()){
    char receivedChar = Serial.read();
    if (receivedChar == 'F' || receivedChar == 'R') {
      if (receivedChar=='F'){
        direction = 1 ;
      }else if(receivedChar=='R') {
        direction = -1 ;        
      }
      readflag = true ;
    }
    if (readflag){
      data[count] = receivedChar;
      if(data[count] == ','){
        target_distanse = Conversion(data[1], data[2], data[3], data[4]);
        if(data[0] == 'F'){
          direction = -1;
        }else{
          direction = 1;
        }
        Control_Motor();
        String dataToSend = "M" + String(status) + "D" + String(x) ;
        Serial.println(dataToSend);
        status = 0;
        count = 0;
        readflag = false ;
      }else{
        count ++;
      }
    }
  }
}
