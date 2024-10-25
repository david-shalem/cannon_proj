#include<Servo.h> 
#include<math.h> 

Servo ver_serv;
Servo hor_serv;

// arduino pins
#define HOR_SERVO_PIN 2
#define VERR_SERVO_PIN 3
#define LASER_PIN 6
#define SOLENOID_PIN 7
#define SWITCH_PIN 13
#define BUTTON_PIN 12
#define JOYSTICK_BUTTON_PIN 10
#define JSK_PIN_X A0
#define JSK_PIN_Y A1

#define DC_1_1 9
#define DC_1_2 8
#define DC_2_1 4
#define DC_2_2 5

#define BASE_ANGLE_HOR 90 
#define BASE_ANGLE_VER 90

#define ACCAPTED_ANGLES_DEVIATION 3 // the biggest deviation between the angles of the servos and the targets that is accaptable too shoot  

// joystick info 
#define NONE 0
#define UP 2
#define DOWN 1
#define RIGHT 2
#define LEFT 1

// joystick threshhold 
#define LEFT_THRESHOLD  400
#define RIGHT_THRESHOLD 800
#define UP_THRESHOLD    400
#define DOWN_THRESHOLD  800

struct complexNumber {
  unsigned int x;
  unsigned int y;
};

typedef struct complexNumber complexN;

complexN extractAngles(String angles);
complexN getJoystickInfo();
complexN reciveAngles();
bool checkCurrAngles();
void shoot();

int hor_angle = BASE_ANGLE_HOR;
int ver_angle = BASE_ANGLE_VER;
bool targetFlag = false; // if there is a target in the cannon sight 

///function returns the diraction of the joystick 
complexN getJoystickInfo()
{
  complexN res = {0,0};

  // get analog info from the joystick 
  int x = analogRead(JSK_PIN_X);
  int y = analogRead(JSK_PIN_Y);

  // convert analog info to diractions
  if (x > RIGHT_THRESHOLD)
    res.x = RIGHT;
  else if (x < LEFT_THRESHOLD)
    res.x = LEFT;

  if (y < UP_THRESHOLD)
    res.y = UP;
  else if (y > DOWN_THRESHOLD)
    res.y = DOWN;

  return res;
}

void shoot()
{
  digitalWrite(SOLENOID_PIN, HIGH);
  delay(500);
  digitalWrite(SOLENOID_PIN, LOW);
  delay(500);
}

//function get the info from the rp and returns the angles to the target
complexN reciveAngles()
{
  String res = "";

  // read the rp info from the serial 
  for(int i = Serial.available(); i > 0; i--) {
     res += Serial.read();
  }
  
  // if there is no target in sight
  if(res == "") {
    targetFlag = false;
    return {hor_angle, ver_angle}; // return the current angles 
  }

  targetFlag = true;
  return extractAngles(res); 
}

// function converts the data (as string) to angles (two int numbers)
complexN extractAngles(String angles)
{
  complexN anglesN = {0,0};

  // get the strings of the angles (for example '34' and '170')
  String first = angles.substring(0,angles.indexOf(','));
  String second = angles.substring(angles.indexOf(',') + 1);

  // convert the strings to int 
  anglesN.x = (int)(first.toFloat());
  anglesN.y = (int)(second.toFloat());
  return anglesN;
}

// function checks if the current angles of the servos are close enough to the wanted angles (the angles of the target)  
bool checkCurrAngles()
{
  int hor_serv_angle = hor_serv.read();
  int ver_serv_angle = ver_serv.read();

  return (abs(hor_serv_angle - hor_angle) < ACCAPTED_ANGLES_DEVIATION && 
                abs(ver_serv_angle - ver_angle) < ACCAPTED_ANGLES_DEVIATION);  
}

void setup() {
  hor_serv.attach(HOR_SERVO_PIN);
  ver_serv.attach(VERR_SERVO_PIN);
  
  // defining the pins mode
  pinMode(LASER_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(JOYSTICK_BUTTON_PIN, INPUT);

  pinMode(DC_1_1, OUTPUT);
  pinMode(DC_1_2, OUTPUT);
  pinMode(DC_2_1, OUTPUT);
  pinMode(DC_2_2, OUTPUT);

  digitalWrite(DC_1_2, HIGH);
  digitalWrite(DC_1_1, LOW);

  digitalWrite(DC_2_1, LOW);
  digitalWrite(DC_2_2, HIGH);
  
  Serial.begin(9600); 
  digitalWrite(LASER_PIN, HIGH); // turn on laser 
}


void loop() {
  if(digitalRead(SWITCH_PIN) == HIGH) { // if cannon is on automatic mode 
    Serial.println("auto");
    if(targetFlag && checkCurrAngles()) // the angle of the servo and the destanation are close enough and there is a target in sight 
      shoot();
    
    // update the destanation angles
    complexN angles = reciveAngles();
    hor_angle = angles.x;
    ver_angle = angles.y;
  
    hor_serv.write(hor_angle); // TODO: check if the write function stops the servo
    ver_serv.write(ver_angle);
  }
  else // if the cannon is on hand-controll mode
  {
    if(digitalRead(JOYSTICK_BUTTON_PIN) == LOW){ // if shooting button is pressed 
      shoot();
    }
    complexN joystick_res = getJoystickInfo(); // read the joystick diractions
    
    // move the cannon according to the joystick diractions
    String log = "";
    switch(joystick_res.x) {
      case LEFT:
        hor_serv.write(hor_serv.read() - 3);
        log += "left ";
        break;
      case RIGHT:
        hor_serv.write(hor_serv.read() + 3);
        log += "right ";
        break;
      case NONE:
        break;
    }

    switch(joystick_res.y) {
      case UP:
        hor_serv.write(hor_serv.read() + 3);
        log += "up ";
        break;
      case DOWN:
        hor_serv.write(hor_serv.read() - 3);
        log += "down ";
        break;
      case NONE:
        break;
    }
    Serial.println(log);
  }
  
  delay(100);

}
