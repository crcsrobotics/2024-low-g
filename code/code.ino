#include <Servo.h>
#include <Gizmo.h>
#include <map>

struct Button {
  bool pressed;
  bool wasPressed;
  }; 

std::map<int, Button> buttonMap;

const int MOTOR_CLOCKWISE = 180, MOTOR_COUNTERCLOCKWISE = 0, MOTOR_STATIONARY = 90;

Servo driveLeft, driveRight;
Servo rakeMotor;

int collisionSwitchR = GIZMO_GPIO_8, collisionSwitchL = GIZMO_GPIO_2;
int rakePot = GIZMO_ADC_1;

Gizmo gizmo;

bool isAutonomous = false;

/*
  Drive left, right = Joystick L, R
  Rake up, Rake down = RB, RT
  Rake to 45 degrees = LB
  Rake to 90 = LT
  Start autonomous = Start
  Exit autonomous (only works before hitting the wall) = Back
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
  pinMode(GIZMO_MOTOR_4, OUTPUT);

  pinMode(collisionSwitchR, INPUT);
  pinMode(collisionSwitchL, INPUT);
  pinMode(rakePot, INPUT);

  buttonInit();
  
  driveLeft.attach(GIZMO_MOTOR_3);
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

  if (buttonJustPressed(GIZMO_BUTTON_START)) {
    autonomousTask();
  }

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
  int dpadX = gizmo.getAxis(GIZMO_AXIS_DX);
  int dpadY = gizmo.getAxis(GIZMO_AXIS_DY);
  
  float targetL = map(dpadY, 0, 255, 180, 0), targetR = map(dpadY, 0, 255, 0, 180);

  // Set motor speeds
  driveLeft.write(targetL);
  driveRight.write(targetR);  
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
  /*
    Stage 1 = Setup
    Stage 2 = Drive until wall
    Stage 3 = Back up until rake in correct pos
    Stage 4 = Drop Rake
    Stage 5 = Back up until line
    Stage 6 = Reset
  */

  isAutonomous = true;

  // pause for dramatic effect 💅
  delay(1000);

// STAGE 1
  const int START_POS_DEGREES = 65;
  // the move command is more accurate going up rather than down. this was easier than making it accurate lol
  setRakePosition(START_POS_DEGREES - 5);
  setRakePosition(START_POS_DEGREES);

// STAGE 2
  const float STAGE_2_SPEED = 0.5;
  bool switchL = false, switchR = false;
  while(!switchR) {
    robotForward(STAGE_2_SPEED);   

    gizmo.refresh();
    // allows for task cancelling
    if (gizmo.getButton(GIZMO_BUTTON_BACK)) {
      return;
    }

    switchL = digitalRead(collisionSwitchL) == HIGH;
    switchR = digitalRead(collisionSwitchR) == HIGH;
  }
  robotStop();

// STAGE 3
  const float STAGE_3_SPEED = 0.25;
  robotBackward(STAGE_3_SPEED);
  const float STAGE_3_BACKUP_S = 1.1;
  delay(STAGE_3_BACKUP_S * 1000);
  robotStop();

// STAGE 4
  rakeMotor.write(MOTOR_CLOCKWISE);
  const int STAGE_4_BACKUP_S = 1;
  delay(STAGE_4_BACKUP_S * 1000);
  rakeMotor.write(MOTOR_STATIONARY);

// STAGE 5
  const float BACKUP_SPEED = 0.5;
  robotBackward(BACKUP_SPEED);

  // // time for a backup at top speed
  const float DRIVE_TIME_SECONDS = 2;
  // // time for a backup at set speed
  const float DRIVE_TIME_ADJUSTED = DRIVE_TIME_SECONDS * (1/BACKUP_SPEED);

  delay(DRIVE_TIME_ADJUSTED * 1000);

  robotStop();

// STAGE 6
  const int END_POS = 20;
  setRakePosition(END_POS);
  isAutonomous = false;
  return;
}

// sets the rake to a specific position in degrees
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

    // this is autonomous task code only - we had this problem with the rake getting pinned and the spool spinning infinitely in a giant loop...
    if (isAutonomous && (digitalRead(collisionSwitchR) == HIGH || digitalRead(collisionSwitchL) == HIGH)) {
      rakeMotor.write(MOTOR_STATIONARY);
      return;
    }
  }
  rakeMotor.write(MOTOR_STATIONARY);
}

// returns the rake's position in degrees
int getRakePosition() {
  // you have to change these values if you change environments or if humidity has changed a lot.
  const int POT_MAX = 722, POT_MIN = 254;

  // I added these dirty hacky offsets. The values should be right, but for some reason they weren't, and it made sense to me to put it on a scale of 0-92 + 5... no clue... 
  return map(analogRead(rakePot), POT_MIN, POT_MAX, 0, 92) + 5;
}

/*
  The motors (for some reason) operate using 90 as stationary, with 180 rotating one direction and 0 rotating the other at full speed. 
  These functions allow you to use a value between 0 and 1 for speed instead. (because that makes more sense)
  Basically the idea is that if we want to travel at full speed straight forward we take 180 on one wheel, and 0 on the other.
  Initially in development we (I) tried to add in variable speed control by just multiplying the top speed by the percent speed. 180 * 0.25. 
  Unsure how this ever made sense to me as that literally equals 45 which drives the motor in the opposite direction at half speed. Double failure. 
  After some actual critical thinking I realized a solution. Say we take a variable called "offset". Let offset equal 90 * our percent speed.
  Now if we want to move clockwise at quarter speed, we take 90 and add our offset to it.

    offset = 22.5
    motor.write(90 + offset) 
    
  this writes 112.5 to the motor, which is quarter-speed

  there's probably a better way to do this with map functions, but I wrote that code less than 72 hours before competition day lol, 
*/

void robotForward(float speed) {
  float offset = MOTOR_STATIONARY * speed;
  driveLeft.write(MOTOR_STATIONARY + offset);
  driveRight.write(MOTOR_STATIONARY - offset);
}

void robotBackward(float speed) {
  float offset = MOTOR_STATIONARY * speed;
  driveLeft.write(MOTOR_STATIONARY - offset);
  driveRight.write(MOTOR_STATIONARY + offset);
}

void robotClockwise(float speed) {
  float offset = MOTOR_STATIONARY * speed;
  driveLeft.write(MOTOR_STATIONARY + offset);
  driveRight.write(MOTOR_STATIONARY + offset);
}

void robotCounterclockwise(float speed) {
  float offset = MOTOR_STATIONARY * speed;
  driveLeft.write(MOTOR_STATIONARY - offset);
  driveRight.write(MOTOR_STATIONARY - offset);
}

void robotStop() {
  driveLeft.write(MOTOR_STATIONARY);
  driveRight.write(MOTOR_STATIONARY);
}

/*
  So this is called button debounce

  I have no idea what debounce is sposeda mean, but basically we're just turning the raw button is pressed data that we get from Gizmo, and turning that into button pressed checks

  There are several much better written articles/papers (ik because i read them to learn how to code this lol)

  basically we just keep track of whether or not each button is pressed this tick of the loop, and if it was pressed last tick.
  if the button was pressed last tick and it isn't anymore, then the button was just let go. if it wasn't pressed, but now it is, then it was just clicked down. 
  i opted for the function to only return true when the button was just pressed down as this seemed more standard.
*/

// creates the list of buttons that we use and sets them up
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


// updates each buttons value
void buttonUpdate() {
  // update was pressed and is pressed
  // update last time button pressed
  for (auto& [index, button] : buttonMap) {
    button.wasPressed = button.pressed;
    button.pressed = gizmo.getButton(index);
  }
}

// actual method for checking if a button was just pressed
bool buttonJustPressed(int index) {
  // saves a little bit of time, but mostly this was useful for debugging
  if (gizmo.getButton(index) == 0) return false;

  Button button = buttonMap[index];

  // only return true if the button wasn't pressed last frame, and is pressed now
  return !button.wasPressed && button.pressed;
}
