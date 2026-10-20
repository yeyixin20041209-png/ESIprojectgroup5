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
// In IDLE, press D5 once to enter calibration; release it, position
// fingers, then press D5 again within 15 seconds to save thresholds.
// D3 cancels calibration. Serial STATE shows CALIBRATING while pending.
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

// 校准阈值上限为 155 - 20 = 135，与伸直阈值留出间隔，避免轻微波动触发抓握。
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
// CALIBRATING:
// First SET enters; release and press SET again to save. No grasping.
// RELEASE cancels; RESET enters HOMING; timeout keeps old thresholds.
//
// CLOSING:
// Motor pulls the tendon and closes the fingers.
//
// HOLD:
// FSR has detected contact. STEP stops; driver stays enabled for torque.
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
  CALIBRATING,
  CLOSING,
  HOLD,
  OPENING,
  HOMING
};

State state = IDLE;

// 从第一次 SET 按下开始计时；15 秒内未成功保存就退出，保留原阈值。
const unsigned long CALIBRATION_TIMEOUT_MS = 15000;
unsigned long calibrationStarted = 0;

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

const unsigned long SET_DEBOUNCE_MS = 30;
bool setReady = false;
unsigned long setLastChange = 0;

// One event per debounced press, with a stable release between presses.
// Call every loop, even outside IDLE, so held buttons cannot queue a SET.
// HIGH 为松开、LOW 为按下；电平稳定 30 ms 后才处理。
// setReady 在松开后置 true，按下时消耗一次，因此长按不会同时完成“进入”和“保存”。
bool setThresholdPressed(bool setButton)
{
  unsigned long now = millis();
  if (setButton != lastSetButton)
  {
    lastSetButton = setButton;
    setLastChange = now;
  }
  if (now - setLastChange < SET_DEBOUNCE_MS)
  {
    return false;
  }
  if (setButton == HIGH)
  {
    setReady = true;
  }
  else if (setReady)
  {
    setReady = false;
    return true;
  }
  return false;
}

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

  // 按钮稳定松开（HIGH）后，允许识别下一次按下。
  if (resetButton == HIGH)
  {
    resetReady = true;
  }

  // 按钮稳定按下（LOW）且已准备好时，只返回一次 true；长按不会重复触发。
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
    case CALIBRATING: return "CALIBRATING";
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
      ? "STATE       | FLEX1 | THR1 | FLEX2 | THR2 |  FSR | FSR_T | STR_T |   STEPCOUNT\r\n"
      : "";
    int length = snprintf(output, sizeof(output),
                          "%s%-11s | %5d | %4d | %5d | %4d | %4d | %5d | %5d | %11ld\r\n",
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
  bool setPressed = setThresholdPressed(setButton);


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

      // Require a fresh SET release/press after leaving HOMING.
      // Consume RELEASE so exiting cannot also trigger normal opening.
      lastReleaseButton = releaseButton;
      setReady = false;
      setLastChange = millis();
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
  // 1. First debounced SET press in IDLE enters CALIBRATING.
  // 2. Release SET and bend fingers to the desired activation position.
  // 3. A second debounced press saves both readings if each <= 135.
  //    Invalid readings leave calibration pending; release and retry.
  // 4. RELEASE or a 15-second timeout cancels without saving.
  // 5. No automatic grasping during calibration. Require straight
  //    fingers again after saving/cancelling. Never block on a button.
  //
  // After calibration:
  // BOTH Flex readings <= their thresholds -> grasp intention detected.
  // ==========================================================
  // 操作：IDLE 按一次 D5 → 松开并摆好弯曲姿势 → 再按 D5 保存。
  // D3 取消；D4 进入手动回零。校准过程中 loop 继续运行，不阻塞按键和串口。
  // 先判断超时：即使本轮同时检测到 SET，超时后也不会保存或重新进入校准。
  if (state == CALIBRATING &&
      millis() - calibrationStarted >= CALIBRATION_TIMEOUT_MS)
  {
    state = IDLE;
  }
  else if (setPressed && state == IDLE)
  {
    // 第一次只进入校准，不修改阈值；校准时暂停自动抓握。
    state = CALIBRATING;
    calibrationStarted = millis();
    graspArmed = false;
    straightTiming = false;
    bendTiming = false;
  }
  else if (setPressed && state == CALIBRATING && releaseButton == HIGH)
  {
    // 第二次按下才取本轮的两个 ADC 读数（不是多次采样平均值）。
    // D3 必须处于松开状态，避免同时按下取消键时仍保存阈值。
    // 两个读数都必须 <= 135；任意一个不合格都保留旧阈值，继续等待重试。
    if (flex1 <= STRAIGHT_THRESHOLD - FLEX_CALIBRATION_GAP &&
        flex2 <= STRAIGHT_THRESHOLD - FLEX_CALIBRATION_GAP)
    {
      // 两个传感器各自保存阈值；仅保存在 RAM，断电重启恢复默认值。
      flex1Threshold = flex1;
      flex2Threshold = flex2;
      state = IDLE;
    }
    // 清除抓握许可和稳定计时；返回 IDLE 后须先双指伸直稳定 100 ms，再弯曲触发。
    // 无效读数重试不会重置 calibrationStarted，15 秒仍从首次进入校准计算。
    graspArmed = false;
    straightTiming = false;
    bendTiming = false;
  }


  // ==========================================================
  // BUTTON: RELEASE
  //
  // Logic:
  // During CALIBRATING, cancel and retain the previous thresholds.
  // 1. Works while the glove is CLOSING or HOLDING.
  // 2. Change state to OPENING.
  // 3. Enable the motor.
  // 4. Set direction to OPEN.
  // 5. OPENING state reverses the recorded closing steps.
  // ==========================================================

  if (releaseButton == LOW &&
      lastReleaseButton == HIGH)
  {
    if (state == CALIBRATING)
    {
      // 取消只退出状态，不改动 flex1Threshold / flex2Threshold。
      state = IDLE;
    }
    else if (state == CLOSING || state == HOLD)
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
  // Used for RELEASE edge detection.
  // RESET and SET are tracked by their debounced press helpers.
  // A command is triggered only when the button changes
  // from HIGH -> LOW.
  // ==========================================================

  lastReleaseButton = releaseButton;


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
  // STATE: CALIBRATING
  // ==========================================================

  else if (state == CALIBRATING)
  {
    // Wait without stepping or running the automatic grasp detector.
    // 禁用驱动并保持 STEP 为 LOW，用户调整手指姿势时不会启动自动抓握。
    digitalWrite(EN_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
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


      // 停转不停机：停止 STEP，保持驱动使能以提供保持扭矩。
      digitalWrite(STEP_PIN, LOW);
      digitalWrite(EN_PIN, LOW);
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
  // 2. The driver remains enabled to provide holding torque.
  // 3. No STEP pulses are sent, preventing commanded tightening.
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
    // Hold position with coil current; do not generate STEP pulses.
    digitalWrite(EN_PIN, LOW);
    digitalWrite(STEP_PIN, LOW);
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
