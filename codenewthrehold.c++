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
// More bending -> LOWER ADC value
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
// RESET/HOME can enter HOMING to recover the reference position.
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
// Bent readings are around 115; <= 125 allows some fluctuation.
int flex1Threshold = 125;
int flex2Threshold = 125;

// Straight Flex is approximately 160.
// >= 155 allows small fluctuations around the measured straight value.
const int STRAIGHT_THRESHOLD = 155;

// Keep calibrated activation thresholds below the straight threshold.
const int FLEX_CALIBRATION_GAP = 20;
const unsigned long FLEX_STABLE_MS = 100;

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

// Separate counter used during HOMING.
long homeStepCount = 0;

// Prevents unlimited reverse movement during HOMING.
const long MAX_HOME_STEPS = 2000;


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
// Motor opens until both Flex sensors indicate straight fingers.
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

// Require straight fingers before each new grasp, including at startup.
bool graspArmed = false;
bool homeFailed = false;
bool straightTiming = false;
bool bendTiming = false;
unsigned long straightSince = 0;
unsigned long bendSince = 0;

// Confirm a condition only after it remains true continuously for 100 ms.
bool flexConditionStable(bool condition, bool &timing,
                         unsigned long &since)
{
  if (!condition)
  {
    timing = false;
    return false;
  }

  unsigned long now = millis();
  if (!timing)
  {
    timing = true;
    since = now;
  }
  return now - since >= FLEX_STABLE_MS;
}


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

// RESET must first be released stably, including after power-up.
const unsigned long RESET_DEBOUNCE_MS = 30;
bool resetReady = false;
unsigned long resetLastChange = 0;

bool resetHomePressed(bool resetButton)
{
  unsigned long now = millis();
  if (resetButton != lastResetButton)
  {
    lastResetButton = resetButton;
    resetLastChange = now;
  }

  if (now - resetLastChange < RESET_DEBOUNCE_MS)
  {
    return false;
  }

  if (resetButton == HIGH)
  {
    resetReady = true;
  }
  else if (resetReady)
  {
    resetReady = false;
    return true;
  }
  return false;
}


// ============================================================
// SERIAL STATUS TABLE
// Open Serial Monitor at 115200 baud. Columns use fixed widths.
// THR1 / THR2 are the current calibrated Flex thresholds.
// FSR_T is the contact threshold; STR_T is the straight threshold.
// STEPCOUNT is the recorded closing position (decreases on release).
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
      ? "STATE   | FLEX1 | THR1 | FLEX2 | THR2 |  FSR | FSR_T | STR_T |   STEPCOUNT\r\n"
      : "";
    int length = snprintf(output, sizeof(output),
                          "%s%-7s | %5d | %4d | %5d | %4d | %4d | %5d | %5d | %11ld\r\n",
                          header, serialStateName(), flex1, flex1Threshold,
                          flex2, flex2Threshold, fsr, FSR_THRESHOLD,
                          STRAIGHT_THRESHOLD, stepCount);

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

  // Sample the actual startup level; an initial LOW is not a press.
  lastResetButton = digitalRead(RESET_PIN);
  resetLastChange = millis();
  resetReady = false;


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


  // ==========================================================
  // BUTTON: RESET / HOME
  //
  // Logic:
  // 1. RESET/HOME is used to recover the reference position.
  // 2. Require a stable release, then a stable press (30 ms each).
  //    Only that press enters HOMING; startup LOW is ignored.
  // 3. Ignore the normal recorded closing step count.
  // 4. Enable the motor.
  // 5. Set motor direction to OPEN.
  // 6. HOMING state will determine when the fingers are straight.
  // ==========================================================

  if (resetHomePressed(resetButton))
  {
    state = HOMING;

    // Start a new homing operation.
    homeStepCount = 0;
    graspArmed = false;
    straightTiming = false;
    bendTiming = false;

    // Enable DRV8825.
    digitalWrite(EN_PIN, LOW);

    // Set direction to OPEN.
    digitalWrite(DIR_PIN, LOW);
  }


  // ==========================================================
  // BUTTON: SET FLEX THRESHOLD
  //
  // Logic:
  // 1. Only works while the system is IDLE.
  // 2. User bends fingers to the desired activation position.
  // 3. User presses the SET button.
  // 4. Save both readings only if each is at least 20 below 155.
  // 5. Require straight fingers again before allowing a grasp.
  //
  // After calibration:
  // BOTH Flex readings <= their thresholds -> grasp intention detected.
  // ==========================================================

  if (setButton == LOW &&
      lastSetButton == HIGH &&
      state == IDLE)
  {
    if (flex1 <= STRAIGHT_THRESHOLD - FLEX_CALIBRATION_GAP &&
        flex2 <= STRAIGHT_THRESHOLD - FLEX_CALIBRATION_GAP)
    {
      flex1Threshold = flex1;
      flex2Threshold = flex2;
    }
    graspArmed = false;
    straightTiming = false;
    bendTiming = false;
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
      graspArmed = false;
      straightTiming = false;
      bendTiming = false;

      // Enable motor.
      digitalWrite(EN_PIN, LOW);

      // Set direction to OPEN.
      digitalWrite(DIR_PIN, LOW);
    }
  }


  // ==========================================================
  // SAVE BUTTON STATES
  //
  // Used for RELEASE and SET edge detection.
  // RESET is tracked separately by resetHomePressed().
  // A command is triggered only when the button changes
  // from HIGH -> LOW.
  // ==========================================================

  lastReleaseButton = releaseButton;
  lastSetButton     = setButton;


  // ==========================================================
  // STATE: IDLE
  //
  // Logic:
  // 1. Motor is disabled.
  // 2. Continuously monitor both Flex sensors.
  // 3. First require BOTH Flex sensors >= 155 for 100 ms.
  // 4. Flex ADC decreases when the finger bends. If BOTH sensors
  //    stay <= their calibrated thresholds for 100 ms, start grasping.
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

    // A failed HOME must be recovered by a successful RESET/HOME.
    if (homeFailed)
    {
      printSensorReadings(flex1, flex2, fsr);
      return;
    }

    if (!graspArmed)
    {
      bool bothStraight = flex1 >= STRAIGHT_THRESHOLD &&
                          flex2 >= STRAIGHT_THRESHOLD;
      if (flexConditionStable(bothStraight, straightTiming, straightSince))
      {
        graspArmed = true;
        straightTiming = false;
      }
    }

    // Check whether each finger is bent.
    bool flex1Bent = flex1 <= flex1Threshold;
    bool flex2Bent = flex2 <= flex2Threshold;


    // Both Flex sensors must indicate grasp intention.
    if (flexConditionStable(graspArmed && flex1Bent && flex2Bent,
                            bendTiming, bendSince))
    {
      // Start closing.
      state = CLOSING;
      graspArmed = false;
      straightTiming = false;
      bendTiming = false;


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
      graspArmed = false;
      straightTiming = false;
      bendTiming = false;
    }
  }


  // ==========================================================
  // STATE: HOMING
  //
  // Logic:
  // 1. HOMING is a recovery/reference-position operation.
  // 2. It does NOT rely on the normal stepCount.
  // 3. Motor moves in the OPEN direction.
  // 4. Continuously monitor both Flex sensors.
  // 5. A Flex value >= STRAIGHT_THRESHOLD means that
  //    finger is considered sufficiently straight.
  // 6. BOTH Flex sensors must indicate straight fingers for 100 ms.
  // 7. When home is reached, reset all position counters.
  // 8. Return to IDLE.
  //
  // Safety:
  // If MAX_HOME_STEPS is reached before both fingers become
  // straight, stop the motor to prevent unlimited reverse
  // movement.
  //
  // Transitions:
  //
  // HOMING -> IDLE
  // Condition 1: Both Flex sensors are straight.
  //
  // HOMING -> IDLE
  // Condition 2: MAX_HOME_STEPS is reached; block grasping until
  // a subsequent RESET/HOME successfully recovers the reference.
  // ==========================================================

  else if (state == HOMING)
  {
    // Determine whether each finger is sufficiently straight.
    bool flex1IsStraight = flex1 >= STRAIGHT_THRESHOLD;
    bool flex2IsStraight = flex2 >= STRAIGHT_THRESHOLD;
    bool bothStraight = flex1IsStraight && flex2IsStraight;
    bool homeReached = flexConditionStable(bothStraight, straightTiming,
                                           straightSince);


    // --------------------------------------------------------
    // HOME POSITION REACHED
    // --------------------------------------------------------

    if (homeReached)
    {
      // Stop motor.
      digitalWrite(EN_PIN, HIGH);


      // Reset position information.
      stepCount = 0;
      homeStepCount = 0;
      homeFailed = false;
      graspArmed = true;
      straightTiming = false;
      bendTiming = false;


      // Return to normal waiting state.
      state = IDLE;
    }


    // --------------------------------------------------------
    // HOME NOT REACHED
    // --------------------------------------------------------

    else if (bothStraight)
    {
      // Pause movement while confirming straight fingers for 100 ms.
      digitalWrite(EN_PIN, HIGH);
    }
    else
    {
      // ------------------------------------------------------
      // Safety check
      // ------------------------------------------------------

      if (homeStepCount >= MAX_HOME_STEPS)
      {
        // Maximum reverse movement reached.
        // Stop the motor to prevent continuous movement.
        digitalWrite(EN_PIN, HIGH);


        // Exit HOMING.
        state = IDLE;
        homeFailed = true;
        graspArmed = false;
        straightTiming = false;
        bendTiming = false;


        printSensorReadings(flex1, flex2, fsr);
        return;
      }


      // ------------------------------------------------------
      // Continue moving in OPEN direction
      // ------------------------------------------------------

      digitalWrite(EN_PIN, LOW);

      if (micros() - lastStepTime >= STEP_INTERVAL)
      {
        lastStepTime = micros();


        // Generate one reverse motor step.
        makeStep();


        // Record homing movement for the safety limit.
        homeStepCount++;
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
