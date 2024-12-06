#include <Servo.h>
#include <Gizmo.h>
#include <map>

struct Button {
  bool pressed;
  bool wasPressed;
  unsigned long lastDebounceTime = 0;
}; 
unsigned long debounceDelay = 50;

std::map<int, Button> buttonMap;

const int MOTOR_CLOCKWISE = 180, MOTOR_COUNTERCLOCKWISE = 0, MOTOR_STATIONARY = 90;

Servo driveLeft, driveRight;
Servo rakeMotor;

int collisionSwitchR = GIZMO_GPIO_8, collisionSwitchL = GIZMO_GPIO_2;
int rakePot = GIZMO_ADC_1;

// The Gizmo provides access to the data that is held by the field
// management system and the gizmo system processor.
Gizmo gizmo;

/*
  Drive left, right = Joystick L, R
  Rake up, Rake down = RB, RT
  HiP up, HiP down = LB, LT
  Manny Safety Bar = Btn-A
  MMM floor = Btn-B

*/

/* Logging standards
  ctx = function name, affected assembly, or general context
  name = value name
  val = actual value

  Serial.println("ctx_name:val");
*/

// Initialize the hardware and configure libraries for use.
void setup() {
  // Initialize the connection to the system processor.
  gizmo.begin();

  // Configure the builtin LED so we can tell the program is running.
  pinMode(LED_BUILTIN, OUTPUT);

  // Configure Pins
  pinMode(GIZMO_MOTOR_1, OUTPUT);
  pinMode(GIZMO_MOTOR_3, OUTPUT);
  pinMode(GIZMO_MOTOR_2, OUTPUT);

  pinMode(collisionSwitchR, INPUT);
  pinMode(collisionSwitchL, INPUT);
  pinMode(rakePot, INPUT);

  buttonInit();
  
  driveLeft.attach(GIZMO_MOTOR_4);
  driveRight.attach(GIZMO_MOTOR_1);
  rakeMotor.attach(GIZMO_MOTOR_2);
}

// Runs in a loop to perform tasks that are required to run the robot.
void loop() {
  // Toggle the built-in LED each time through the loop so we can see
  // that the program really is running.
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // Refreshes the information about axis and button state.
  gizmo.refresh();

  axisDriveLogic();
  // padDriveLogic();

  if (buttonJustPressed(GIZMO_BUTTON_LSHOULDER)) {
    setRakePosition(45);
  }

  else if (buttonJustPressed(GIZMO_BUTTON_LT)) {
    setRakePosition(90);
  }

  // if (buttonJustPressed(GIZMO_BUTTON_START)) {
  //   autonomousTask();
  // }

  Serial.println("left:" + String(digitalRead(collisionSwitchL)));
  Serial.println("right:" + String(digitalRead(collisionSwitchR)));

  rakeLogic();

  buttonUpdate();

  // Limit tick speed to something less crazy. Just makes the math easier
  delay(20);
}

void axisDriveLogic() {
  // Fetch the speed of each axis, then convert this to the range
  // expected by the motors.
  float targetL, targetR;
  const float SPEED_MULTIPLIER = 0.5;
  float axisL = gizmo.getAxis(GIZMO_AXIS_LY), axisR = gizmo.getAxis(GIZMO_AXIS_RY);

  targetL = map(axisL, 0, 255, 180, 0);
  targetR = map(axisR, 0, 255, 0, 180);

  driveLeft.write(targetL);
  driveRight.write(targetR);
}

void padDriveLogic() {
  // Read the D-pad values
  int dpadX = gizmo.getAxis(GIZMO_AXIS_DX);  // Normalize to -128 to 128
  int dpadY = gizmo.getAxis(GIZMO_AXIS_DY);  // Normalize to -128 to 128
  
  // Determine motor speeds based on D-pad input
  int leftMotorSpeed = 0;
  int rightMotorSpeed = constrain(dpadY - dpadX, -128, 128) + 127;

  // Set motor speeds
  driveLeft.write(leftMotorSpeed);
  driveRight.write(rightMotorSpeed);  
}

void servoToggle(Servo &servo, int min, int max) {
  int target = servo.read() == max ? min : max;

  servo.write(target);
}

void rakeLogic() {
  const int MAX = 90, MIN = 0;

  // RSHOuLDER and RT
  if (gizmo.getButton(GIZMO_BUTTON_RSHOULDER)) {
    rakeMotor.write(MOTOR_COUNTERCLOCKWISE);
  }

  else if (gizmo.getButton(GIZMO_BUTTON_RT)) {
    rakeMotor.write(MOTOR_CLOCKWISE);
  }
  else {
    rakeMotor.write(MOTOR_STATIONARY);
  }
}

void autonomousTask() {
  // pause for dramatic effect 💅
  delay(1000);

  // starting rake position
  setRakePosition(55);

  bool switchL = false, switchR = false;
  while(!switchL || !switchR) {
    if (!switchL) {
      driveLeft.write(MOTOR_COUNTERCLOCKWISE);
    }

    if (!switchR) {
      driveRight.write(MOTOR_CLOCKWISE);
    }

    // allows for task cancelling
    if (gizmo.getButton(GIZMO_BUTTON_BACK)) {
      return;
    }

    switchL = digitalRead(collisionSwitchL) == HIGH;
    switchR = digitalRead(collisionSwitchR) == HIGH;
  }
  robotStop();

  // lower rake all the way
  setRakePosition(120);

  const float BACKUP_SPEED = 0.25;
  robotBackward(BACKUP_SPEED);

  // time for a backup at top speed
  const float DRIVE_TIME_SECONDS = .9;
  // time for a backup at set speed
  const float DRIVE_TIME_ADJUSTED = DRIVE_TIME_SECONDS * (1/BACKUP_SPEED);

  delay(DRIVE_TIME_ADJUSTED * 1000);

  robotStop();

  // reset the rake
  setRakePosition(0);
}

void setRakePosition(int posDegrees) {
  int rakePos = getRakePosition();

  while (rakePos != posDegrees) {
    if (posDegrees > rakePos) {
      rakeMotor.write(MOTOR_CLOCKWISE);
    }

    else if (posDegrees < rakePos) {
      rakeMotor.write(MOTOR_COUNTERCLOCKWISE);
    }

    rakePos = getRakePosition();
  }
}

int getRakePosition() {
  const int POT_MAX = 695, POT_MIN = 250;
  return map(analogRead(rakePot), POT_MIN, POT_MAX, 0, 92);
}

void robotForward(float speed) {
  Serial.println("robot_y_status:1");
  driveLeft.write(MOTOR_COUNTERCLOCKWISE * speed);
  driveRight.write(MOTOR_CLOCKWISE * speed);
}

void robotBackward(float speed) {
  Serial.println("robot_y_status:-1");
  driveLeft.write(MOTOR_CLOCKWISE * speed);
  driveRight.write(MOTOR_COUNTERCLOCKWISE * speed);
}

void robotClockwise(float speed) {
  Serial.println("robot_x_status:1");
  driveLeft.write(MOTOR_COUNTERCLOCKWISE * speed);
  driveRight.write(MOTOR_COUNTERCLOCKWISE * speed);
}

void robotCounterclockwise(float speed) {
  Serial.println("robot_x_status:-1");
  driveLeft.write(MOTOR_CLOCKWISE * speed);
  driveRight.write(MOTOR_CLOCKWISE * speed);
}

void robotStop() {
  Serial.println("robot_status:0");
  driveLeft.write(MOTOR_STATIONARY);
  driveRight.write(MOTOR_STATIONARY);
}

void buttonInit() {
  int buttons[12] = {
    GIZMO_BUTTON_X,          
    GIZMO_BUTTON_A,          
    GIZMO_BUTTON_B,          
    GIZMO_BUTTON_Y,          
    GIZMO_BUTTON_LSHOULDER,  
    GIZMO_BUTTON_RSHOULDER,  
    GIZMO_BUTTON_LT,         
    GIZMO_BUTTON_RT,         
    GIZMO_BUTTON_BACK,       
    GIZMO_BUTTON_START,      
    GIZMO_BUTTON_LEFTSTICK,  
    GIZMO_BUTTON_RIGHTSTICK 
  };

  for (int i : buttons) {
    buttonMap[i] = {false, false};
  }
}

// button debounce logic
void buttonUpdate() {
  // update was pressed and is pressed
  // update last time button pressed
  for (auto& [index, button] : buttonMap) {
    button.wasPressed = button.pressed;
    button.pressed = gizmo.getButton(index);
    // Serial.println("buttons_btn" + String(index) + "_pressed:" + String(button.pressed));
    // Serial.println("buttons_btn" + String(index) + "_wasPressed:" + String(button.wasPressed));
  }
}

bool buttonJustPressed(int index) {
  // saves a little bit of time, but mostly this was useful for debugging
  if (gizmo.getButton(index) == 0) return false;

  Button button = buttonMap[index];

  if (button.wasPressed == button.pressed) return false;

  return button.pressed;
}
