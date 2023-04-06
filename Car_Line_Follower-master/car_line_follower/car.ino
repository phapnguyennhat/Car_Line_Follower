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
double base_speed = 190; // 195-200-205-210-212.5-200-205.0-200-195
double motor_speed = base_speed;

int left_motor_speed;
int right_motor_speed;
//Thiết lập tốc độ rẽ, lùi
int banh_chinh = 120;//-135-100-110.0-90-100
int banh_phu = 100; //Đảo ngược-75-95-100.0-75-80-100
int toc_do_lui = 110;
//Thiết lập hệ số PID
double Kp =17;    //7
double Ki = 0;
double Kd =15;

//Khai báo các biến nhớ cần dùng
int memory = 0;
int count = 0;
int slow = 0;
int background = 1;
int line = 0;
//Khai bao de dung millis
unsigned long time_count_1;
unsigned long time_now_1 = 0;

//Khai báo PID
  //double max_PID_value = 255 - motor_speed;
  double max_PID_value = 100;
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
  myPID.SetSampleTime(5); //40-30-20-10-12-12.5-13.5-12.5-14-13-12.5-14-16-13.5-12-13
}

void loop()
{

  read_sensor();
 // Serial.println(error);
  Serial.println("vong loop");
  Serial.println(motor_speed);
  if ((error >=-1) && (error <= 1)) memory = error; //tạo memory
  if (error == 31) 
  {
    do 
    {                            
      DiLui();
      time_count_1 = millis ();
      if (time_count_1 - time_now_1 > 20) {
      time_now_1 = millis(); 
      read_sensor();
      }
    }  
    while (error == 31);
  }
  else if(error==30||error==-30){
    error=memory;
  }
else if(error==0.01){
  myPID.Compute();
  motor_control();
  delay(200);
  motor_speed=130;
  
  do{
  read_sensor();
  Serial.println("vong while line chan");
  if(error==31){
    do{
      DiLui();
      delay(20);
      read_sensor();
      Serial.println("vong while di lui");
    }
    while(error==31);
  }
  else if(error==30){
    do{
      ReTrai();
      read_sensor();
      Serial.println("vong while re trai");
    }
    while(error!=0);
    motor_speed = base_speed;
    
    break;
  }
  else if(error==-30){
    do{
      RePhai();
      read_sensor();      
      Serial.println("vong while re phai");
    }
    while(error!=0);
    motor_speed=base_speed;
   // Kp = 7;
    break;
  }
  else{
    myPID.Compute();
    motor_control();
  }
    Serial.println(motor_speed);
  }
  while(1);
}
  else
  {
    motor_speed=base_speed;
    myPID.Compute();    // Sau khi loại bỏ hết các error đặc biệt mới bỏ vào bộ tính toán PID
    motor_control();
    // Serial.println(PID_value);    //xem gia tri pid_value                
  }
    // DiThang();
    // analogWrite(ENA,220);//0.1-0.15
    // analogWrite(ENB,220);
}


void read_sensor()
{
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
 // Đã sửa theo setpoint là 2.5, cần sửa chữa quẹo trái, quẹo phải, nữa là ổn
  if((sensor[0]==background)&&(sensor[1]==background)&&(sensor[2]==background)&&(sensor[3]==line))
  error=6;
  else if((sensor[1]==background)&&(sensor[1]==background)&&(sensor[2]==line)&&(sensor[3]==line))
  error=3.5;
  else if((sensor[0]==background)&&(sensor[1]==background)&&(sensor[2]==line)&&(sensor[3]==background))
  error=1;
  else if((sensor[0]==background)&&(sensor[1]==line)&&(sensor[2]==line)&&(sensor[3]==background))
  error=0;
  else if((sensor[0]==background)&&(sensor[1]==line)&&(sensor[2]==background)&&(sensor[3]==background))
  error=1;
  else if((sensor[0]==line)&&(sensor[1]==line)&&(sensor[2]==background)&&(sensor[3]==background))
  error=-3.5;
  else if((sensor[0]==line)&&(sensor[1]==background)&&(sensor[2]==background)&&(sensor[3]==background))
  error=-6;
  //Cần sửa ở dưới
  else if ((sensor[0] == line) && (sensor[1] == line) && (sensor[2] == line) && (sensor[3] == line))// Giam toc
  error = 0.01;
  else if ((sensor[0] == background) && (sensor[1] == line) && (sensor[2] == line) && (sensor[3] ==line)) // Rẽ Phải
  error = 30;
  else if ((sensor[0] == line) && (sensor[1] == line) && (sensor[2] == line) && (sensor[3] == background)) // Rẽ Trái
  error = -30;
  else if ((sensor[0] == background) && (sensor[1] == background) && (sensor[2] == background) && (sensor[3] == background)) // Out line
  error = 31;
  else {
     error = memory;
  }      
}

void motor_control()
{ 
  left_motor_speed = motor_speed  + PID_value; // Khai bao bien toan cuc
  right_motor_speed = motor_speed - PID_value - 0.09*(motor_speed + PID_value);  // Khai bao bien toan cuc

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

void ReTrai() {
  /*Banh phải nhanh hơn bánh trái */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, banh_phu); // stop
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, banh_chinh);
}
void RePhai() {
  /*Banh trái nhanh hơn bánh phải */
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, banh_chinh);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, banh_phu);
}

