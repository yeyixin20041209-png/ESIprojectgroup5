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
// D4 toggles manual HOMING. D3 HIGH holds left rotation;
// D5 HIGH holds right rotation. Both LOW or both HIGH stop.
// D4 exits, stops the motor, and sets a new zero.
// ============================================================


#include <Arduino.h>
#include <stdio.h>

void makeStep();

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
const int DEFAULT_FLEX_THRESHOLD = 125;
int flex1Threshold = DEFAULT_FLEX_THRESHOLD;
int flex2Threshold = DEFAULT_FLEX_THRESHOLD;

// Straight Flex is approximately 160.
// >= 155 allows small fluctuations around the measured straight value.
const int STRAIGHT_THRESHOLD = 155;

// Keep calibrated activation thresholds below the straight threshold. set 时候用 防止误触， 防止set 阈值太高 
const int FLEX_CALIBRATION_GAP = 20;

//至少100 ms

const unsigned long FLEX_STABLE_MS = 100;

// FSR contact threshold.
// This value should be adjusted after physical testing.
const int FSR_THRESHOLD = 500;


// ============================================================
// MOTOR SETTINGS
// ============================================================

// Time between STEP pulses.
// Smaller value -> faster motor movement.不能太快 会丢步
const unsigned long STEP_INTERVAL = 20000;  // microseconds

// 不能为负数  可以存储较大的整数 初始化为零 记录电机上一次执行步进操作的时间，用来控制电机转速。 

unsigned long lastStepTime = 0;

// Records how many steps are used during closing.
// Normal release reverses the same number of steps.
long stepCount = 0;

// Manual homing directions. Swap these values if the physical motor
// turns the opposite way with the installed wiring.
// HOMING_RIGHT_DIR：定义一个方向对应的电平。  HOMING_LEFT_DIR：定义相反方向对应的电平。  homingDirection：保存当前选择的回零方向。
const int HOMING_RIGHT_DIR = HIGH;
const int HOMING_LEFT_DIR  = LOW;
int homingDirection = HOMING_RIGHT_DIR;

enum HomingMotion
{
  HOMING_STOPPED,
  HOMING_LEFT,
  HOMING_RIGHT
};

HomingMotion homingMotion = HOMING_STOPPED;


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
// Manual movement follows active-HIGH levels, with no toggle/latch.
// D3 HIGH alone -> left; D5 HIGH alone -> right; equal levels -> stop.
// D4 is the only command that exits HOMING and sets the new zero.
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
bool straightTiming = false;
bool bendTiming = false;
unsigned long straightSince = 0;
unsigned long bendSince = 0;

// Confirm a condition only after it remains true continuously for 100 ms.
// 当手指伸直时，flexConditionStable()函数会被调用，传入参数为true。
// 函数会检查这个条件是否已经持续了至少100毫秒，如果是，则返回true，表示手指已经稳定地伸直了。否则，返回false，表示手指还没有稳定地伸直。
bool flexConditionStable(bool condition, bool &timing,
                         unsigned long &since)
{
  if (!condition)
  {
    timing = false;
    return false;
  }
// 如果条件为真，函数会检查是否已经开始计时。如果还没有开始计时，则将timing设置为true，并记录当前时间为since。
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
// 记录上一次按钮状态，初始为HIGH，表示未按下。
bool lastReleaseButton = HIGH;
bool lastResetButton = HIGH;
bool lastSetButton = HIGH;

// RESET must first be released stably, including after power-up.
// 同时防止长按、按键抖动以及开机时误触发。 按键状态需要稳定 30 ms 
const unsigned long RESET_DEBOUNCE_MS = 30;
bool resetReady = false;
unsigned long resetLastChange = 0;

// ============================================================
// BUTTON: RESET / HOME 
bool resetHomePressed(bool resetButton)
{
  unsigned long now = millis(); //millis() 表示 Arduino 从开始运行到现在经过了多少毫秒。

  // 检查当前按钮状态是否与上一次状态不同，如果不同，则更新lastResetButton和resetLastChange。
  //如果只写：
//if (resetButton == LOW)
//{
    //state = HOMING;
//}
//那么只要按钮一直按着，loop() 就会不断执行 state = HOMING。
//这可能导致每次循环都重新启动回零操作。
//所以我们需要检测的是：按钮从 HIGH 变为 LOW 的有效按下事件，而不是按钮一直处于 LOW。//


  if (resetButton != lastResetButton)
  {
    lastResetButton = resetButton;
    resetLastChange = now;
  }

  // 如果距离上一次状态变化的时间小于防抖时间，则返回false，表示按钮状态不稳定。
  if (now - resetLastChange < RESET_DEBOUNCE_MS)
  {
    return false;
  }

  // 如果按钮被按下，则设置resetReady为true。
  if (resetButton == HIGH)
  {
    resetReady = true;
  }

  // 如果按钮被释放，并且之前已经按下过，则返回true，表示RESET/HOME按钮被按下并释放。
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

  // Read the current button states. These are active LOW.
  bool releaseButton = digitalRead(RELEASE_PIN);
  bool resetButton = digitalRead(RESET_PIN);
  bool setButton = digitalRead(SET_PIN);


  // ==========================================================
  // BUTTON: RESET / HOME
  //
  // A debounced D4 press toggles manual HOMING from any state.
  // Exit stops the motor, sets the current position as zero, and
  // restores the default Flex thresholds. Require straight fingers
  // again before normal grasping can start.
  // ==========================================================

  if (resetHomePressed(resetButton))
  {

    digitalWrite(EN_PIN, HIGH);//禁用 DRV8825 电机驱动器 //需要调整
    digitalWrite(STEP_PIN, LOW);//将 STEP 信号拉低
    graspArmed = false;//暂时不允许启动新抓握
    straightTiming = false;//清除伸直稳定计时状态
    bendTiming = false;//清除弯曲稳定计时状态
    lastStepTime = micros();//更新上一次步进时间
    homingMotion = HOMING_STOPPED;// 将回零运动状态设置为停止

    if (state == HOMING)//如果当前正在回零，再按一次 RESET，就退出 HOMING。
    {
      state = IDLE;
      stepCount = 0;
      flex1Threshold = DEFAULT_FLEX_THRESHOLD;
      flex2Threshold = DEFAULT_FLEX_THRESHOLD;

      // Consume the current button levels so leaving HOMING cannot
      // also calibrate thresholds or trigger a normal release. 这是为了防止按钮误触发。
      //例如，退出 HOMING 时，SET 按钮正好处于 LOW（按下）。如果 lastSetButton 仍然是 HIGH，后面的代码可能误认为 SET 刚被按下，进而触发校准。
      //提前保存当前按钮状态，就能避免这种情况。
      lastReleaseButton = releaseButton;
      lastSetButton = setButton;
      printSensorReadings(flex1, flex2, fsr);
      return;
    }
    //如果不是 HOMING 状态，按下 RESET 就进入 HOMING。 设置初始回零方向为 HOMING_RIGHT_DIR。  将方向信号发送给 DRV8825。
    state = HOMING;
    homingDirection = HOMING_RIGHT_DIR;
    digitalWrite(DIR_PIN, homingDirection);
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
  // 只有在 IDLE 状态下，SET 按钮才有效。  当手指弯曲到期望位置时，按下 SET 按钮，保存当前的 Flex 传感器读数作为新的阈值。
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
    //校准 Flex 阈值后，禁止手套立即自动抓握，必须先让手指重新伸直，才能进行下一次抓握。
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
    //确认手指已经伸直
    if (!graspArmed)
    {

      //这个函数的作用是：检查 bothStraight 是否连续保持 true 至少 100 ms。
      // bothStraight：当前是否伸直。
      // straightTiming：是否已经开始计算连续伸直的时间。
      // straightSince：从什么时候开始伸直。
      // 如果连续满足 100 ms，函数就返回 true。
      bool bothStraight = flex1 >= STRAIGHT_THRESHOLD &&
                          flex2 >= STRAIGHT_THRESHOLD;
      if (flexConditionStable(bothStraight, straightTiming, straightSince))
      {
        //graspArmed = true 表示系统已经准备好接收抓握动作。
        //straightTiming = false 则是清除这次伸直检测的计时状态。
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

      //graspArmed = false：本次抓握许可已经使用，下次抓握必须重新确认伸直。
      //straightTiming = false：清除伸直计时状态。
      //bendTiming = false：清除弯曲计时状态。

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
  // Follow the input levels on every loop, with no toggle/latch:
  // D3 HIGH and D5 LOW -> left; D5 HIGH and D3 LOW -> right.
  // Both LOW (released) or both HIGH (conflicting commands) -> stop.
  // These active-HIGH commands require LOW on release in the wiring.
  // Stop immediately on release; no button debounce delays stopping.
  // Flex readings do not end manual homing. Only D4 exits.
  // Normal closing stepCount is not used for manual movement.
  // ==========================================================

  else if (state == HOMING)
  {
    HomingMotion previousMotion = homingMotion;

    if (releaseButton == HIGH && setButton == LOW)
    {
      homingMotion = HOMING_LEFT;
    }
    else if (setButton == HIGH && releaseButton == LOW)
    {
      homingMotion = HOMING_RIGHT;
    }
    else
    {
      homingMotion = HOMING_STOPPED;
    }

    if (homingMotion != previousMotion)
    {
      // Stop before starting or reversing. Wait a full step interval
      // before the next pulse so the new direction can settle.
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(STEP_PIN, LOW);
      lastStepTime = micros();
      if (homingMotion != HOMING_STOPPED)
      {
        homingDirection = homingMotion == HOMING_LEFT
          ? HOMING_LEFT_DIR : HOMING_RIGHT_DIR;
        digitalWrite(DIR_PIN, homingDirection);
      }
    }

    if (homingMotion == HOMING_STOPPED)
    {
      digitalWrite(EN_PIN, HIGH);
      lastStepTime = micros();
    }
    else
    {
      digitalWrite(EN_PIN, LOW);
      if (micros() - lastStepTime >= STEP_INTERVAL)
      {
        lastStepTime = micros();
        makeStep();
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
