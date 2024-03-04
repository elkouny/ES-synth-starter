#include <Arduino.h>
#include <U8g2lib.h>
#include <sstream>
#include <bitset>
#include <cmath>
#include <string>
#include <vector>
#include <STM32FreeRTOS.h>

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
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

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

std::vector<String> Keys = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

volatile uint32_t currentStepSize;

void sampleISR()

{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}

struct
{
  std::bitset<32> inputs;
} sysState;

void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 50 /portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    std::bitset<4> cols;
    uint32_t localCurrentStepSize;

    for (int i = 0; i <= 2; i++)
    {

      setRow(i);
      delayMicroseconds(3);
      cols = readCols();
      sysState.inputs[i * 4] = cols[0];
      sysState.inputs[i * 4 + 1] = cols[1];
      sysState.inputs[i * 4 + 2] = cols[2];
      sysState.inputs[i * 4 + 3] = cols[3];
    }
    
    int idx = -1;
    for (int i = 0; i <= 11; i++)
    {
      if (sysState.inputs[i] == 0)
      {
        idx = i;
      };
    }
    if (idx == -1)
      localCurrentStepSize = 0;
    else
      localCurrentStepSize = stepSizes[idx];
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
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
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,
    "scankeys",
    64,
    NULL,
    1,
    &scanKeysHandle
  );
  vTaskStartScheduler();

}

// dimension of the screen are 128x32

void loop()
{
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  while (millis() < next)
    ; // Wait for next interval

  next += interval;
  

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  u8g2.drawStr(2, 20, String("Piano!").c_str());
  u8g2.sendBuffer();
  // Toggle LED
  digitalToggle(LED_BUILTIN);
}