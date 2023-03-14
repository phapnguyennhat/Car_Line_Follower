#include <PID_v1.h>   //khai báo thư viện PID

// Khai báo chân cảm biến hồng ngoại
int sensor1 = A0;      // Bên trái
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;      // Bên phải
 
// Đặt giá trị ban đầu là 0
int sensor[4] = {0, 0, 0, 0};

// Khai báo chân L298N              
int IN1 = 3;  //DC trái: 3,4,5
int IN2 = 4;
int ENA = 5;
int ENB = 6;  //DC phải: 6,7,8
int IN3 = 7;
int IN4 = 8;
                         
//Thiết lập tốc độ nền
double base_speed = 185; // 195-200-205-210-212.5-200-205.0-200-195
double motor_speed = base_speed ;

//Thiết lập tốc độ rẽ, lùi
int banh_chinh = 170;//-135-100-110.0-90-100
int banh_phu = 100; //Đảo ngược-75-95-100.0-75-80-100
int toc_do_lui = 135;

//Thiết lập hệ số PID
double Kp = 0.06;
double Ki = 0;
double Kd = 0.0009;

//Khai báo các biến nhớ cần dùng
int memory = 0;
int count = 0;

//Khai bao de dung millis
unsigned long time_count_1;
unsigned long time_now_1 = 0;

//Khai báo PID
  double max_PID_value = 255 - motor_speed;
  double error = 0, PID_value = 0, Setpoint = 2500;
  PID myPID(&error, &PID_value, &Setpoint, Kp, Ki, Kd, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
                                                                //P_ON_E (Proportional on Error) is the default behavior
  
void setup()
{
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Serial.begin(9600);      
  delay(500);
  Serial.println("RUN");
  delay(1000); 
  
  myPID.SetOutputLimits(-max_PID_value, max_PID_value); // gia tri output nam trong khoang (-55, 55)
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(5); //40-30-20-10-12-12.5-13.5-12.5-14-13-12.5-14-16-13.5-12-13
}

void loop()
{
  read_sensor();
  // Serial.print(PID_value);
  Serial.println(error);

  // if ((error >=2 )&&(error <= 3)) memory = error; //tạo memory
  
  if (error == 15) 
  {
    do 
    {                            
      DiLui();
      time_count_1 = millis ();
      if (time_count_1 - time_now_1 > 20) {
      read_sensor();
      } // Di lui trong hay ngoai ngoac ?
    } 
    while (error == 15);
  }
  
  // if (error == 20) {
  //     if (count == 0) count =1;
  //     else count = 0;
  // }
  else if (error == 10) {
      // count=0;
      // delay(1);
      // read_sensor();
      // if (error == 20) break;
    do {
      RePhai();
      read_sensor();
    }
    while (error =! 2500); // or 2.5p;..;;;;;;.
  }
  else if (error == - 5) {

    // count=0;
    do {
    ReTrai();
    read_sensor();
    }
    while (error != 2500);
    } 
  else 
  {
    if (error >= 2000 and error <= 3000) base_speed =195;
    else base_speed = 175; 
    myPID.Compute();   
    motor_control();               
}
}


void read_sensor()
{
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
 // Đã sửa theo setpoint là 2.5, cần sửa chữa quẹo trái, quẹo phải, nữa là ổn
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1))
  error=5000;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1))
  error=4000;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0))
  error=3000;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0))
  error=2500;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0))
  error=2000;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0))
  error=1000;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0))
  error=0;
  //Cần sửa ở dưới
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1))// Giam toc
  error = 20;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Rẽ Phải
  error = 10;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)) // Rẽ Trái
  error = -5;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) // Out line
  error = 15;
  else {}
    // error = memory;    
}

void motor_control()
{ 
  int left_motor_speed = motor_speed  + PID_value;  //Nếu chạy ngược thì đổi + thành - 
  int right_motor_speed = motor_speed - PID_value;  

  // Giới hạn giá trị xuất xung từ 0 - 255
  left_motor_speed = constrain(left_motor_speed, 0, 255);   
  right_motor_speed = constrain(right_motor_speed, 0, 255); 

  analogWrite(ENA, left_motor_speed);  
  analogWrite(ENB, right_motor_speed);
  
  DiThang();
}

void DiThang()
{
  /*Quy ước: tiến là nhỏ HIGH - lớn LOW */
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void DiLui()
{
  /*Quy ước: lùi là nhỏ LOW - lớn HIGH */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, toc_do_lui);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, toc_do_lui);
}

void RePhai() {
  /*Banh phải nhanh hơn bánh trái */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 0); // stop
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, banh_chinh);
}
void ReTrai() {
  /*Banh trái nhanh hơn bánh phải */
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, banh_chinh);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, 0);
}

