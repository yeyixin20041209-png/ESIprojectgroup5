// ============================================================
// Assistive Grasping Glove
// Core Control Code
// ============================================================
//
// Sensors:
// A0  -> Flex Sensor 1
// A1  -> Flex Sensor 2
// A2  -> FSR
//
// Buttons:
// D3  -> RELEASE
// D4  -> RESET / HOME
// D5  -> SET FLEX THRESHOLD
//
// DRV8825:
// D8  -> STEP
// D9  -> DIR
// D10 -> ENABLE
//
// Flex sensor behaviour:
// More bending -> HIGHER ADC value
//
// Main control sequence:
//
// IDLE
//   -> BOTH Flex sensors detect bending
// CLOSING
//   -> FSR detects object contact
// HOLD
//   -> RELEASE button
// OPENING
//   -> stepCount reaches 0
// IDLE
//
// RESET/HOME alone controls HOMING with four separate presses:
// enter stopped -> DIR HIGH -> DIR LOW -> confirm zero and exit.
// SET and RELEASE do not control movement during HOMING.
// ============================================================


#include <stdio.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

// Sensors
const int FLEX1_PIN = A0;
const int FLEX2_PIN = A1;
const int FSR_PIN   = A2;

// Buttons
const int RELEASE_PIN = 3;
const int RESET_PIN   = 4;
const int SET_PIN     = 5;

// DRV8825
const int STEP_PIN = 8;
const int DIR_PIN  = 9;
const int EN_PIN   = 10;


// ============================================================
// SENSOR THRESHOLDS
// ============================================================

// Default Flex activation thresholds.
// These can be updated using the SET button.
// 200 is a grasp activation value, not the maximum bend reading.
// Observed full bending can reach 300 or higher.
int flex1Threshold = 200;
int flex2Threshold = 200;

// Straight Flex is approximately 40.
// <= 60 is provisionally considered sufficiently straight.
const int STRAIGHT_THRESHOLD = 60;

// FSR contact threshold.
// This value should be adjusted after physical testing.
const int FSR_THRESHOLD = 500;


// ============================================================
// MOTOR SETTINGS
// ============================================================

// Time between STEP pulses.
// Smaller value -> faster motor movement.
const unsigned long STEP_INTERVAL = 20000;  // microseconds

unsigned long lastStepTime = 0;

// Records how many steps are used during closing.
// Normal release reverses the same number of steps.
long stepCount = 0;

// Total steps in BOTH directions during one manual HOMING session.
long homeStepCount = 0;

// Reaching the limit stops movement but does not confirm zero.
const long MAX_HOME_STEPS = 2000;

// Manual adjustment runs slower than normal grasp/release.
const unsigned long HOME_STEP_INTERVAL = 40000;  // microseconds
const unsigned long HOME_BUTTON_DEBOUNCE_MS = 30;
unsigned long lastHomeStepTime = 0;
int homeStage = 0;  // 0 = entered/stopped, 1 = forward, 2 = reverse
int homeDirection = 0;  // 0 = STOP, 1 = DIR HIGH, -1 = DIR LOW

// Only enabled after manual zero confirmation; normal startup is unchanged.
bool waitForStraightAfterHoming = false;


// ============================================================
// STATE MACHINE
// ============================================================
//
// IDLE:
// Wait for grasp intention.
//
// CLOSING:
// Motor pulls the tendon and closes the fingers.
//
// HOLD:
// FSR has detected contact. Motor stops.
//
// OPENING:
// Motor reverses the recorded closing steps.
//
// HOMING:
// RESET cycles through stopped -> forward -> reverse -> confirm zero.
// Each stage requires a separate debounced press and release.
//
enum State
{
  IDLE,
  CLOSING,
  HOLD,
  OPENING,
  HOMING
};

State state = IDLE;


// ============================================================
// BUTTON EDGE DETECTION
// ============================================================
//
// Buttons use INPUT_PULLUP:
//
// Not pressed -> HIGH
// Pressed     -> LOW
//
// The previous button state is stored so that one physical
// press is treated as one command instead of being repeatedly
// triggered while the button is held.
//
bool lastReleaseButton = HIGH;
bool lastResetButton   = HIGH;
bool lastSetButton     = HIGH;

// Debounce RESET so one physical press cannot enter and confirm HOMING.
bool stableResetButton = HIGH;
unsigned long resetButtonChangeTime = 0;


// ============================================================
// SERIAL STATUS TABLE
// Open Serial Monitor at 115200 baud. Columns use fixed widths.
// THR1 / THR2 are the current calibrated Flex thresholds.
// FSR_T is the contact threshold; STR_T is the straight threshold.
// STEPCOUNT is the recorded closing position (decreases on release).
// HOME_DIR is STOP, FWD (DIR HIGH), or REV (DIR LOW).
// Repeat the header every 20 samples. Queue each sample every
// 100 ms and send it in chunks without waiting for buffer space.
// ============================================================

const unsigned long SERIAL_BAUD = 115200;
const unsigned long SERIAL_INTERVAL_MS = 100;
unsigned long lastSerialTime = 0;

const char *serialStateName()
{
  switch (state)
  {
    case IDLE:    return "IDLE";
    case CLOSING: return "CLOSING";
    case HOLD:    return "HOLD";
    case OPENING: return "OPENING";
    case HOMING:  return "HOMING";
    default:      return "UNKNOWN";
  }
}

void printSensorReadings(int flex1, int flex2, int fsr)
{
  static char output[192];
  static size_t outputLength = 0;
  static size_t outputOffset = 0;
  static unsigned int rowsSinceHeader = 0;

  if (!Serial)
  {
    outputLength = 0;
    outputOffset = 0;
    rowsSinceHeader = 0;
    return;
  }

  unsigned long now = millis();
  if (outputOffset == outputLength &&
      now - lastSerialTime >= SERIAL_INTERVAL_MS)
  {
    lastSerialTime = now;
    const char *header = rowsSinceHeader == 0
      ? "STATE   | FLEX1 | THR1 | FLEX2 | THR2 |  FSR | FSR_T | STR_T |   STEPCOUNT | HOME_DIR\r\n"
      : "";
    int length = snprintf(output, sizeof(output),
                          "%s%-7s | %5d | %4d | %5d | %4d | %4d | %5d | %5d | %11ld | %-8s\r\n",
                          header, serialStateName(), flex1, flex1Threshold,
                          flex2, flex2Threshold, fsr, FSR_THRESHOLD,
                          STRAIGHT_THRESHOLD, stepCount,
                          homeDirection > 0 ? "FWD" : (homeDirection < 0 ? "REV" : "STOP"));

    if (length <= 0 || length >= (int)sizeof(output))
    {
      return;
    }
    outputLength = (size_t)length;
    outputOffset = 0;
    rowsSinceHeader = (rowsSinceHeader + 1) % 20;
  }

  // A table row can exceed the Arduino transmit buffer size.
  // Drain the pending text over successive loop iterations.
  int available = Serial.availableForWrite();
  if (available > 0 && outputOffset < outputLength)
  {
    size_t count = outputLength - outputOffset;
    if (count > (size_t)available)
    {
      count = (size_t)available;
    }
    outputOffset += Serial.write((const uint8_t *)output + outputOffset, count);
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  // ----------------------------------------------------------
  // Configure buttons
  // ----------------------------------------------------------

  pinMode(RELEASE_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(SET_PIN, INPUT_PULLUP);


  // ----------------------------------------------------------
  // Configure DRV8825 control outputs
  // ----------------------------------------------------------

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);


  // ----------------------------------------------------------
  // Initial motor state
  // ----------------------------------------------------------

  // STEP starts LOW.
  digitalWrite(STEP_PIN, LOW);

  // Direction:
  // HIGH = CLOSE
  // LOW  = OPEN
  digitalWrite(DIR_PIN, LOW);

  // DRV8825 ENABLE is active LOW:
  // LOW  = enabled
  // HIGH = disabled
  //
  // Start with the motor disabled.
  digitalWrite(EN_PIN, HIGH);

  Serial.begin(SERIAL_BAUD);
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // READ SENSORS
  // ==========================================================

  int flex1 = analogRead(FLEX1_PIN);
  int flex2 = analogRead(FLEX2_PIN);
  int fsr   = analogRead(FSR_PIN);


  // ==========================================================
  // READ BUTTONS
  // ==========================================================

  bool releaseButton = digitalRead(RELEASE_PIN);
  bool resetButton   = digitalRead(RESET_PIN);
  bool setButton     = digitalRead(SET_PIN);

  bool resetPressed = false;
  bool homeExitedThisLoop = false;
  unsigned long buttonTime = millis();
  if (resetButton != lastResetButton)
  {
    resetButtonChangeTime = buttonTime;
  }
  if (buttonTime - resetButtonChangeTime >= HOME_BUTTON_DEBOUNCE_MS &&
      resetButton != stableResetButton)
  {
    stableResetButton = resetButton;
    resetPressed = (stableResetButton == LOW);
  }


  // ==========================================================
  // BUTTON: RESET / HOME
  //
  // Logic:
  // 1. First press enters HOMING with the motor stopped.
  // 2. Second press starts continuous movement with DIR HIGH.
  // 3. Third press stops pulses, then starts movement with DIR LOW.
  // 4. Fourth press stops, confirms zero, clears the counters,
  //    and returns to IDLE. SET / RELEASE are ignored in HOMING.
  // 5. Require straight fingers before the next automatic grasp.
  // ==========================================================

  if (resetPressed)
  {
    if (state != HOMING)
    {
      state = HOMING;
      homeStepCount = 0;
      homeDirection = 0;
      homeStage = 0;
      lastHomeStepTime = micros();
      digitalWrite(EN_PIN, HIGH);
    }
    else if (homeStage < 2)
    {
      // Stage is independent of homeDirection, which also becomes
      // STOP at the travel limit. Never restart the travel counter.
      digitalWrite(EN_PIN, HIGH);
      homeDirection = 0;
      homeStage++;
      lastHomeStepTime = micros();
    }
    else
    {
      digitalWrite(EN_PIN, HIGH);
      homeDirection = 0;
      stepCount = 0;
      homeStepCount = 0;
      homeStage = 0;
      waitForStraightAfterHoming = true;
      homeExitedThisLoop = true;
      state = IDLE;
    }
  }


  // ==========================================================
  // BUTTON: SET FLEX THRESHOLD
  //
  // Logic:
  // 1. Only works while the system is IDLE.
  // 2. User bends fingers to the desired activation position.
  // 3. User presses the SET button.
  // 4. Current Flex readings become the new thresholds.
  //
  // After calibration:
  // BOTH Flex readings >= their thresholds -> grasp intention detected.
  // ==========================================================

  if (setButton == LOW &&
      lastSetButton == HIGH &&
      state == IDLE && !homeExitedThisLoop)
  {
    flex1Threshold = flex1;
    flex2Threshold = flex2;
  }


  // ==========================================================
  // BUTTON: RELEASE
  //
  // Logic:
  // 1. Works while the glove is CLOSING or HOLDING.
  // 2. Change state to OPENING.
  // 3. Enable the motor.
  // 4. Set direction to OPEN.
  // 5. OPENING state reverses the recorded closing steps.
  // ==========================================================

  if (releaseButton == LOW &&
      lastReleaseButton == HIGH)
  {
    if (state == CLOSING || state == HOLD)
    {
      state = OPENING;

      // Enable motor.
      digitalWrite(EN_PIN, LOW);

      // Set direction to OPEN.
      digitalWrite(DIR_PIN, LOW);
    }
  }


  // ==========================================================
  // SAVE BUTTON STATES
  //
  // Used for edge detection.
  // A command is triggered only when the button changes
  // from HIGH -> LOW.
  // ==========================================================

  lastReleaseButton = releaseButton;
  lastResetButton   = resetButton;
  lastSetButton     = setButton;


  // ==========================================================
  // STATE: IDLE
  //
  // Logic:
  // 1. Motor is disabled.
  // 2. Continuously monitor both Flex sensors.
  // 3. Flex ADC increases when the finger bends.
  // 4. If BOTH Flex sensors reach or exceed their calibrated
  //    thresholds, grasp intention is detected.
  // 5. Reset stepCount to zero.
  // 6. Enable the motor.
  // 7. Set direction to CLOSE.
  // 8. Change state to CLOSING.
  //
  // Transition:
  //
  // IDLE -> CLOSING
  // Condition: Flex1 AND Flex2 detect bending.
  // ==========================================================

  if (state == IDLE)
  {
    // Motor remains disabled while waiting.
    digitalWrite(EN_PIN, HIGH);


    // Check whether each finger is bent.
    bool flex1Bent = flex1 >= flex1Threshold;
    bool flex2Bent = flex2 >= flex2Threshold;


    // After manual zero confirmation, first require straight fingers
    // and released buttons. A later bend can start another grasp.
    if (waitForStraightAfterHoming)
    {
      if (flex1 <= STRAIGHT_THRESHOLD && flex2 <= STRAIGHT_THRESHOLD &&
          resetButton == HIGH && stableResetButton == HIGH &&
          releaseButton == HIGH && setButton == HIGH)
      {
        waitForStraightAfterHoming = false;
      }
    }

    // Both Flex sensors must indicate grasp intention.
    else if (flex1Bent && flex2Bent)
    {
      // Start closing.
      state = CLOSING;


      // Start recording motor movement from zero.
      stepCount = 0;


      // Enable motor driver.
      digitalWrite(EN_PIN, LOW);


      // Set motor direction to CLOSE.
      digitalWrite(DIR_PIN, HIGH);
    }
  }


  // ==========================================================
  // STATE: CLOSING
  //
  // Logic:
  // 1. Motor moves in the CLOSE direction.
  // 2. STEP pulses pull the tendon and close the fingers.
  // 3. Each STEP pulse increases stepCount by 1.
  // 4. Continuously monitor the FSR.
  // 5. If the FSR reaches the contact threshold,
  //    an object is considered detected.
  // 6. Stop the motor.
  // 7. Change state to HOLD.
  //
  // Transitions:
  //
  // CLOSING -> HOLD
  // Condition: FSR detects object contact.
  //
  // CLOSING -> OPENING
  // Condition: RELEASE button is pressed.
  // ==========================================================

  else if (state == CLOSING)
  {
    // --------------------------------------------------------
    // Check object contact first
    // --------------------------------------------------------

    if (fsr >= FSR_THRESHOLD)
    {
      // Object/contact detected.
      state = HOLD;


      // Stop motor to prevent further tightening.
      digitalWrite(EN_PIN, HIGH);
    }


    // --------------------------------------------------------
    // No contact -> continue closing
    // --------------------------------------------------------

    else
    {
      // Generate the next STEP only after STEP_INTERVAL.
      if (micros() - lastStepTime >= STEP_INTERVAL)
      {
        lastStepTime = micros();


        // Move one motor step.
        makeStep();


        // Record the movement.
        stepCount++;
      }
    }
  }


  // ==========================================================
  // STATE: HOLD
  //
  // Logic:
  // 1. FSR has detected contact with an object.
  // 2. The motor remains disabled.
  // 3. This prevents additional tendon tightening.
  // 4. The system waits for a user command.
  //
  // Transitions:
  //
  // HOLD -> OPENING
  // Condition: RELEASE button is pressed.
  //
  // HOLD -> HOMING
  // Condition: RESET/HOME button is pressed.
  // ==========================================================

  else if (state == HOLD)
  {
    // Keep motor disabled.
    digitalWrite(EN_PIN, HIGH);
  }


  // ==========================================================
  // STATE: OPENING
  //
  // Logic:
  // 1. Motor direction has already been set to OPEN.
  // 2. stepCount contains the number of steps previously
  //    used during CLOSING.
  // 3. Generate reverse STEP pulses.
  // 4. Decrease stepCount after every reverse step.
  // 5. When stepCount reaches zero, the motor has reversed
  //    the same number of steps used during closing.
  // 6. Disable the motor.
  // 7. Return to IDLE.
  //
  // Transition:
  //
  // OPENING -> IDLE
  // Condition: stepCount reaches 0.
  // ==========================================================

  else if (state == OPENING)
  {
    // There are still recorded closing steps to reverse.
    if (stepCount > 0)
    {
      if (micros() - lastStepTime >= STEP_INTERVAL)
      {
        lastStepTime = micros();


        // Move one step in OPEN direction.
        makeStep();


        // One recorded closing step has now been reversed.
        stepCount--;
      }
    }


    // --------------------------------------------------------
    // All recorded closing steps have been reversed
    // --------------------------------------------------------

    else
    {
      // Stop motor.
      digitalWrite(EN_PIN, HIGH);


      // Return to waiting state.
      state = IDLE;
    }
  }


  // ==========================================================
  // STATE: HOMING
  //
  // Logic:
  // 1. Enter stopped, then use separate RESET presses to advance.
  // 2. Stage 1 continuously moves DIR HIGH; stage 2 moves DIR LOW.
  // 3. Releasing RESET does not stop movement. SET / RELEASE are ignored.
  // 4. Flex readings remain visible but do not finish HOMING.
  // 5. Both directions add to homeStepCount; reversing cannot
  //    reset or reduce the total travel limit.
  // 6. At MAX_HOME_STEPS, remain stopped in HOMING. Only the
  //    fourth RESET press confirms the new zero and exits.
  // ==========================================================

  else if (state == HOMING)
  {
    if (homeStage == 0 || homeStepCount >= MAX_HOME_STEPS)
    {
      digitalWrite(EN_PIN, HIGH);
      homeDirection = 0;
      lastHomeStepTime = micros();
    }
    else
    {
      int requestedDirection = (homeStage == 1) ? 1 : -1;
      if (requestedDirection != homeDirection)
      {
        // Set direction with the driver disabled, then wait a full
        // homing interval before the first pulse in this direction.
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(DIR_PIN, requestedDirection > 0 ? HIGH : LOW);
        homeDirection = requestedDirection;
        lastHomeStepTime = micros();
        digitalWrite(EN_PIN, LOW);
      }
      else if (micros() - lastHomeStepTime >= HOME_STEP_INTERVAL)
      {
        lastHomeStepTime = micros();
        makeStep();
        homeStepCount++;
        if (homeStepCount >= MAX_HOME_STEPS)
        {
          digitalWrite(EN_PIN, HIGH);
          homeDirection = 0;
        }
      }
    }
  }

  // Report state and thresholds after handling this loop's commands.
  printSensorReadings(flex1, flex2, fsr);
}


// ============================================================
// GENERATE ONE DRV8825 STEP PULSE
//
// Logic:
// DRV8825 moves the stepper motor when a pulse is applied
// to its STEP input.
//
// STEP HIGH
//    ↓
// Wait 10 microseconds
//    ↓
// STEP LOW
//
// One call to makeStep() represents one commanded motor step.
// ============================================================

void makeStep()
{
  digitalWrite(STEP_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(STEP_PIN, LOW);
}
