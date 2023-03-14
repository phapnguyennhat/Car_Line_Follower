/*By Mai Huy - Training LT 6 - DEE YOUTH UNION */

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
                         
//Thiết lập tốc độ nền    toi uu thoi gian trong khoang 200
double base_speed = 200; // 195-200-205-210-212.5-200-205.0-200-195
double motor_speed = base_speed ;

//Thiết lập tốc độ rẽ, lùi    khuc nay neu ch dc thi se GIAM TOC DO va TANG DO LECH 2 BANH
int banh_chinh = 140;//-135-100-110.0-90-100
int banh_phu = 50; //Đảo ngược-75-95-100.0-75-80-100
int toc_do_lui = 110;
// co nen tang toc do luu de toi uu thoi gian do line sau khi lech??

//Thiết lập hệ số PID
// pid_value=Kp*error+Kd*error'(t)
// voi base_speed=200 thi pid_value thuoc [-55,55]
// PID_value=55=Kp*2.5+Kd*2.5
//kp+pd=22 hoac nho hon
double Kp = 12;//4-5-6-7.5-8.5-13-14-15-16-16.5-17-17.5-18.0-15-16
double Ki = 0;//2-0.25-0.15-0.02-0.05-0.25-0.1-0.08-0.06-0.040000-0.02-0.04-0
double Kd = 10;//-1.7-1.5-1.7-1.6-1.4-0.2-1.9-10-8.5-8-8-7.5-6.500-8-7-8.0-1.5-2.5-3-1.9-7.5-13
//Kp lon thi cua gắt xe bị lạng k ổn định
//kd tăng xe ổn định 
//Khai báo các biến nhớ cần dùng
int memory = 0;

//Khai báo PID
  double max_PID_value = 255 - motor_speed;
  double error = 0, PID_value = 0, Setpoint = 0;
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
  myPID.SetSampleTime(13); //40-30-20-10-12-12.5-13.5-12.5-14-13-12.5-14-16-13.5-12-13
  //nếu đi qua line chắn mà đọc nhầm thành rẽ thì sẽ TĂNG SAMPLE TIME hoặc TĂNG TỐC ĐỌ
}

void loop()
{
  read_sensor();
  Serial.println(error);
  if ((error >= -1)&&(error <= 1)) memory = error; //tạo memory
  
  if (error == 31)      //lech line
  {
    do 
    {                            
      DiLui();
      delay(20);  // nên học cách thay delay() bằng millis()
      read_sensor();
      //vua di lui vua read_sensor 
    } 
    while (error == 31);
  }
  
  else if (error == -30)                 // Rẽ Trái 90*    
  {
    do                             // Quay sang trái cho tới khi phát hiện ngay giữa line - error == 0
    {
      ReTrai();
      read_sensor();
    }
    while (error != 0);
  }
  
  else if (error == 30)          // Rẽ Phải 90* 
  {    
    do                           // Quay sang phải cho tới khi phát hiện ngay giữa line
    {   
      RePhai();
      read_sensor();
    }
    while (error != 0);        
  } 
      
  else 
  {
    myPID.Compute();    // Sau khi loại bỏ hết các error đặc biệt mới bỏ vào bộ tính toán PID
    motor_control();
    Serial.println(PID_value);    //xem gia tri pid_value                
  }
}


void read_sensor()
{
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
 //thang -2.5 0  2.5
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1))
  error=2.5;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1))
  error=1.5;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0))
  error=0.5;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0))
  error=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0))
  error=-0.5;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0))
  error=-1;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0))
  error=-2.5;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Rẽ Phải
  error = 30;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)) // Rẽ Trái
  error = -30;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) // Out line
  error = 31;
  else 
    error = memory;    //Memory
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
  analogWrite(ENA, banh_phu);
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
  analogWrite(ENB, banh_phu);
}

