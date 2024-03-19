#include <Arduino.h>
#include <U8g2lib.h>
#include <sstream>
#include <bitset>
#include <cmath>
#include <string>
#include <vector>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

// Constants
const uint32_t interval = 100; // Display update interval

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

struct
{
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
  uint8_t RX_Message[8] = {0};

  int knob3Rotation = 0;
} sysState;
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;

void setRow(uint8_t x)
{
  // Convert to 3 bit binary
  std::bitset<3> row(x);
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, row[0]);
  digitalWrite(RA1_PIN, row[1]);
  digitalWrite(RA2_PIN, row[2]);
  digitalWrite(REN_PIN, HIGH);
}

std::bitset<4> readCols()
{
  std::bitset<4> res;
  res[0] = digitalRead(C0_PIN);
  res[1] = digitalRead(C1_PIN);
  res[2] = digitalRead(C2_PIN);
  res[3] = digitalRead(C3_PIN);
  return res;
}
uint32_t calcStepSize(float n)
{
  return static_cast<uint32_t>(std::pow(2.0, n / 12) * std::pow(2, 32) * 440 / 22000);
}

const uint32_t stepSizes[12] = {
    calcStepSize(-9),
    calcStepSize(-8),
    calcStepSize(-7),
    calcStepSize(-6),
    calcStepSize(-5),
    calcStepSize(-4),
    calcStepSize(-3),
    calcStepSize(-2),
    calcStepSize(-1),
    calcStepSize(0),
    calcStepSize(1),
    calcStepSize(2)};

volatile uint32_t currentStepSize;
volatile uint32_t currentKnob3Rotation;

void sampleISR()

{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - currentKnob3Rotation);
  analogWrite(OUTR_PIN, Vout + 128);
}
void CAN_RX_ISR(void)
{
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_Task(void *pvParameters)
{
  uint8_t msgOut[8];
  while (1)
  {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
}

void CAN_TX_ISR(void)
{
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}
class Knob
{
private:
  uint32_t upperLimit;
  uint32_t lowerLimit;
  uint32_t knobIndex;
  int currentValue = 0;
  std::bitset<2> prevB3A3 = 0b00;
  std::bitset<2> B3A3;
  uint32_t row;
  std::bitset<4> cols;
  bool increment = false;

public:
  Knob(uint32_t _knobIndex, int _upperLimit = INT_MAX, int _lowerLimit = INT_MIN)
      : knobIndex(_knobIndex), upperLimit(_upperLimit), lowerLimit(_lowerLimit)
  {
  }

  void setUpperLimit(int newUpperLimit)
  {
    upperLimit = newUpperLimit;
  }

  void setLowerLimit(int newLowerLimit)
  {
    lowerLimit = newLowerLimit;
  }

  int getCurrentValue()
  {
    return currentValue;
  }

  void updateCurrentValue()
  {
    if (knobIndex == 3 || knobIndex == 2)
      row = 3;
    else
      row = 4;
    setRow(row);
    delayMicroseconds(3);
    cols = readCols();

    if (knobIndex == 3 || knobIndex == 1)
      B3A3 = cols[1] << 1 | cols[0];
    else
      B3A3 = cols[3] << 1 | cols[2];
    if (prevB3A3 == 0b00 && B3A3 == 0b01 || prevB3A3 == 0b11 && B3A3 == 0b10)
    {
      currentValue++;
      increment = true;
    }
    else if (prevB3A3 == 0b01 && B3A3 == 0b00 || prevB3A3 == 0b10 && B3A3 == 0b11)
    {
      currentValue--;
      increment = false;
    }
    else if (prevB3A3 == 0b11 && B3A3 == 0b00)
    {
      currentValue += increment ? 1 : -1;
    }
    if (currentValue > upperLimit)
      currentValue = upperLimit;
    else if (currentValue < lowerLimit)
      currentValue = lowerLimit;
    prevB3A3 = B3A3;
  }
};
void decodeTask(void *pvParameters)
{
  // TX_Message[0] = 'R';
  //     TX_Message[1] = 0;
  //     TX_Message[2] = 0;
  uint8_t localMessage[8] = {0};
  while (1)
  {
    xQueueReceive(msgInQ, localMessage, portMAX_DELAY);
    if (localMessage[0] == 'P')
    {

      __atomic_store_n(&currentStepSize, stepSizes[localMessage[2]] << (localMessage[1] - 4), __ATOMIC_RELAXED);
    }

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy(sysState.RX_Message, localMessage, sizeof(localMessage));
    // __atomic_store_n(&currentStepSize, stepSizes[localMessage[2]] << (localMessage[1] - 4), __ATOMIC_RELAXED);
    xSemaphoreGive(sysState.mutex);
  }
}
void scanKeysTask(void *pvParameters)
{
  uint8_t TX_Message[8] = {0};
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  std::bitset<2> prevB3A3 = 0b00;
  std::bitset<2> B3A3;
  Knob knob3(3, 8, 0);
  bool increment = false;
  std::vector<int> indexes;

  std::bitset<4> cols;
  uint32_t localCurrentStepSize;
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    indexes = {};
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    for (int row = 0; row <= 3; row++)
    {

      setRow(row);
      delayMicroseconds(3);
      cols = readCols();
      sysState.inputs[row * 4] = cols[0];
      sysState.inputs[row * 4 + 1] = cols[1];
      sysState.inputs[row * 4 + 2] = cols[2];
      sysState.inputs[row * 4 + 3] = cols[3];
      if (row == 3)
      {
        knob3.updateCurrentValue();
        sysState.knob3Rotation = knob3.getCurrentValue();
        __atomic_store_n(&currentKnob3Rotation, sysState.knob3Rotation, __ATOMIC_RELAXED);
      }
    }

    for (int i = 0; i <= 11; i++)
    {
      if (sysState.inputs[i] == 0)
      {
        indexes.push_back(i);
      };
    }
    xSemaphoreGive(sysState.mutex);
    if (indexes.size() == 0)
    {
      if (sysState.RX_Message[0] != 'P')
      {
        localCurrentStepSize = 0;
        __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
      }
      TX_Message[0] = 'R';
      TX_Message[1] = 0;
      TX_Message[2] = 0;
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }
    else
    {
      localCurrentStepSize = stepSizes[indexes[0]];
      TX_Message[0] = 'P';
      TX_Message[1] = 4;
      TX_Message[2] = indexes[0];
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    }
  }
}

void displayUpdateTask(void *pvParameters)
{
  std::vector<String> Keys = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  String Pressed;
  uint32_t ID;

  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    Pressed = "";
    int idx = -1;
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    for (int i = 0; i <= 11; i++)
    {
      if (sysState.inputs[i] == 0)
        Pressed += (" " + (Keys[i]));
    }
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if (Pressed.length() == 0)
      u8g2.drawStr(2, 10, String("No Keys pressed!").c_str());
    else
      u8g2.drawStr(2, 10, String(Pressed + " Pressed !").c_str());
    u8g2.setCursor(2, 20);
    u8g2.print(sysState.knob3Rotation);

    u8g2.setCursor(66, 30);
    u8g2.print((char)sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    u8g2.print(sysState.RX_Message[2]);

    xSemaphoreGive(sysState.mutex);

    u8g2.sendBuffer();
    // Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}
void setup()
{
  // put your setup code here, to run once:
  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  msgOutQ = xQueueCreate(36, 8);
  msgInQ = xQueueCreate(36, 8);
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);
  // CAN BUS setup CAN_inti = false if u wan to communicate with others
  CAN_Init(false);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  setCANFilter(0x123, 0x7ff);
  CAN_Start();
  // Interrupt timer setup
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  // Threading setup
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,
      "scankeys",
      64,
      NULL,
      3,
      &scanKeysHandle);
  xTaskCreate(
      displayUpdateTask,
      "display",
      256,
      NULL,
      2,
      &scanKeysHandle);
  xTaskCreate(
      decodeTask,
      "decode",
      256,
      NULL,
      1,
      &scanKeysHandle);
    xTaskCreate(
      CAN_TX_Task,
      "Tx Queue",
      256,
      NULL,
      4,
      &scanKeysHandle);
  sysState.mutex = xSemaphoreCreateMutex();
  // Start the threading
  vTaskStartScheduler();
}

// dimension of the screen are 128x32

void loop()
{
}