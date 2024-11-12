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
Servo hipMotor, rakeMotor;

Servo safetyBarServo;
Servo mmmFloorLeft, mmmFloorRight;

bool isAutonomous = false;

int rakePot = GIZMO_ADC_1;
int collisionSwitch = GIZMO_GPIO_1;


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

// Initialize the hardware and configure libraries for use.
void setup() {
  // Initialize the connection to the system processor.
  gizmo.begin();

  // Configure the builtin LED so we can tell the program is running.
  pinMode(LED_BUILTIN, OUTPUT);

  // Configure Pins
  pinMode(GIZMO_MOTOR_1, OUTPUT);
  pinMode(GIZMO_MOTOR_2, OUTPUT);
  pinMode(GIZMO_MOTOR_3, OUTPUT);
  pinMode(GIZMO_MOTOR_4, OUTPUT);

  pinMode(GIZMO_SERVO_1, OUTPUT);
  pinMode(GIZMO_SERVO_2, OUTPUT);
  pinMode(GIZMO_SERVO_3, OUTPUT);

  pinMode(rakePot, INPUT);
  pinMode(collisionSwitch, INPUT);

  buttonInit();
  
  driveLeft.attach(GIZMO_MOTOR_1);
  driveRight.attach(GIZMO_MOTOR_2);

  hipMotor.attach(GIZMO_MOTOR_3);
  rakeMotor.attach(GIZMO_MOTOR_4);

  safetyBarServo.attach(GIZMO_SERVO_1);
  safetyBarServo.write(0);

  mmmFloorLeft.attach(GIZMO_SERVO_2);
  mmmFloorLeft.write(0);
  mmmFloorRight.attach(GIZMO_SERVO_3);
  mmmFloorRight.write(0);
}

// Runs in a loop to perform tasks that are required to run the robot.
void loop() {
  // Toggle the built-in LED each time through the loop so we can see
  // that the program really is running.
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // Refreshes the information about axis and button state.
  gizmo.refresh();

  if (buttonJustPressed(GIZMO_BUTTON_START)) {
    isAutonomous = true;
  }

  if (buttonJustPressed(GIZMO_BUTTON_BACK)) {
    isAutonomous = false;
  }

  if (isAutonomous) {
    autonomousTask();
    return;
  }

  driveLogic();
  
  // this is right next to the drive logic in order to override the drive commands sent if you are pressing the joysticks.
  if (gizmo.getAxis(GIZMO_AXIS_DY) == 254) {
    robotBackward(1);
  }

  Serial.println("Rake Pot:" + String(analogRead(rakePot)));
  Serial.println("Collision Switch:" + String(digitalRead(collisionSwitch)));

  // Floor logic
  if (buttonJustPressed(GIZMO_BUTTON_B)) {
    servoToggle(mmmFloorLeft, 0, 180);
    servoToggle(mmmFloorRight, 180, 0);
  }

  if (gizmo.getButton(GIZMO_BUTTON_LSHOULDER)) {
    // Full speed couterclockwwise
    hipMotor.write(MOTOR_COUNTERCLOCKWISE);
  } else if (gizmo.getButton(GIZMO_BUTTON_LT)) {
    // Full speed clockwise
    hipMotor.write(MOTOR_CLOCKWISE);
  } else {
    // No speed
    hipMotor.write(MOTOR_STATIONARY);
  }

  if (gizmo.getButton(GIZMO_BUTTON_RSHOULDER)) {
    // Full speed counterclockwise
    rakeMotor.write(MOTOR_COUNTERCLOCKWISE);
   } else if (gizmo.getButton(GIZMO_BUTTON_RT)) {
    // Full speed clockwise
    rakeMotor.write(MOTOR_CLOCKWISE);
  } else {
    // No speed
    rakeMotor.write(MOTOR_STATIONARY);
  }

  buttonUpdate();

  // Limit tick speed to something less crazy. Just makes the math easier
  delay(20);
}

void driveLogic() {
  // Fetch the speed of each axis, then convert this to the range
  // expected by the motors.
  int targetL, targetR;
  targetL = map(gizmo.getAxis(GIZMO_AXIS_LY), 0, 255, 0, 180);
  targetR = map(gizmo.getAxis(GIZMO_AXIS_RY), 0, 255, 0, 180);

  // Write the target speeds to the motor controllers.
  driveLeft.write(targetL);
  driveRight.write(targetR);
}

void servoToggle(Servo &servo, int min, int max) {
  int target = servo.read() == max ? min : max;

  servo.write(target);
}

void autonomousTask() {
  // pause for dramatic effect 💅
  delay(1000);

  // Raise rake to preliminary angle
  moveMotor(45, rakeMotor, rakePot);

  // accounting for duration of motion
  delay(1000);

  // drive until wall
  while(digitalRead(collisionSwitch) != HIGH) {
    robotForward(1);

    // allows for task cancelling
    if (gizmo.getButton(GIZMO_BUTTON_BACK)) {
      isAutonomous = false;
      return;
    }
  }

  // lower rake all the way
  moveMotor(90, rakeMotor, rakePot);

  const float BACKUP_SPEED = 0.25;
  robotBackward(BACKUP_SPEED);

  // time for a backup at top speed
  const float DRIVE_TIME_SECONDS = .9;
  // time for a backup at set speed
  const float DRIVE_TIME_ADJUSTED = DRIVE_TIME_SECONDS * (1/BACKUP_SPEED);

  delay(DRIVE_TIME_ADJUSTED * 1000);

  robotStop();

  moveMotor(0, rakeMotor, rakePot);

  isAutonomous = false;
}

void robotForward(float speed) {
  driveLeft.write(MOTOR_CLOCKWISE * speed);
  driveRight.write(MOTOR_COUNTERCLOCKWISE * speed);
}

void robotBackward(float speed) {
  driveLeft.write(MOTOR_CLOCKWISE * speed);
  driveRight.write(MOTOR_CLOCKWISE * speed);
}

void robotStop() {
  driveLeft.write(MOTOR_STATIONARY);
  driveRight.write(MOTOR_STATIONARY);
}

void moveMotor(int posDegrees, Servo &motor, int pot) {
  // Convert the potentiometer position to degrees
  int position = map(analogRead(pot), 0, 1024, 0, 180);

  while (position != posDegrees) {
    if (position < posDegrees) {
      motor.write(MOTOR_CLOCKWISE);
    } else {
      motor.write(MOTOR_COUNTERCLOCKWISE);
    }
  }

  motor.write(MOTOR_STATIONARY);
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
  }
}

bool buttonJustPressed(int index) {
  // saves a little bit of time, but mostly this was useful for debugging
  if (gizmo.getButton(index) == 0) return false;

  Button button = buttonMap[index];

  if (button.wasPressed == button.pressed) return false;
  if (button.pressed == true) return true;

  return false;
}
