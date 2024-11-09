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

const bool DEBUG_MODE = true;

Servo driveLeft, driveRight;
Servo hipMotor, rakeMotor;

Servo safetyBarServo;
Servo mmmFloorLeft, mmmFloorRight;

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

  drive();

  if (buttonJustPressed(GIZMO_BUTTON_A))
    servoToggle(safetyBarServo);

  if (buttonJustPressed(GIZMO_BUTTON_B)) {
    servoToggle(mmmFloorLeft);
    servoToggle(mmmFloorRight, 0, 180);
  }

  if (gizmo.getButton(GIZMO_BUTTON_LSHOULDER)) {
    hipMotor.write(0);
  } else if (gizmo.getButton(GIZMO_BUTTON_LT)) {
    hipMotor.write(180);
  } else {
    hipMotor.write(90);
  }

  if (gizmo.getButton(GIZMO_BUTTON_RSHOULDER)) {
    rakeMotor.write(0);
   } else if (gizmo.getButton(GIZMO_BUTTON_RT)) {
    rakeMotor.write(180);
  } else {
    rakeMotor.write(90);
  }

  buttonUpdate();

  delay(20);
}

void drive() {
  // Fetch the speed of each axis, then convert this to the range
  // expected by the motors.
  int targetL, targetR;
  targetL = map(gizmo.getAxis(GIZMO_AXIS_LY), 0, 255, 0, 180);
  targetR = map(gizmo.getAxis(GIZMO_AXIS_RY), 0, 255, 0, 180);

  // Write the target speeds to the motor controllers.
  driveLeft.write(targetL);
  driveRight.write(targetR);
}

void servoToggle(Servo servo, int min, int max) {
  int target = servo.read() == max ? min : max;
  Serial.println(servo.read());
  Serial.println(target);
  servo.write(target);
}

void servoToggle(Servo servo) {
  servoToggle(servo, 0, 180);
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
  if (gizmo.getButton(index) == 0) return false;

  Button button = buttonMap[index];

  if (button.wasPressed == button.pressed) return false;
  if (button.pressed == true) return true;

  return false;
}
