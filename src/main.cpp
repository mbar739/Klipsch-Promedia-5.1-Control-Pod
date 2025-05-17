/**
 * @file main.cpp
 * @brief Volume control system for a multi-channel audio amplifier (Klipsch ProMedia 5.1)
 *
 * This program implements a volume control system for a multi-channel audio amplifier
 * using a PT2258 electronic volume controller. It provides comprehensive audio control
 * features for a 5.1 channel system with display feedback.
 *
 * Features:
 * - Main volume control (0 to 79 dB attenuation)
 * - Individual channel offset adjustments (±15 dB)
 * - Digital display feedback via TM1637
 * - Power on/off control with soft start
 * - Mute functionality
 * - Auto-timeout for offset adjustments
 * - Headphone output support
 * 
 * Hardware Components:
 * - ESP32 microcontroller
 * - TM1637 4-digit 7-segment display
 * - Rotary encoder with push button
 * - 3 control switches (Power, Mute, Offset)
 * Klipsch Components:
 * - PT2258 electronic volume controller (I²C)
 * - Main amplifier and headphone amplifier control
 * 
 * Pin Configurations:
 * - ENCODER_A: Pin 27 (Rotary encoder A, active LOW)
 * - ENCODER_B: Pin 26 (Rotary encoder B, active LOW)
 * - TM_CLK: Pin 14 (TM1637 clock)
 * - TM_DIO: Pin 12 (TM1637 data)
 * - SDA_PIN: Pin 21 (I²C SDA for PT2258)
 * - SCL_PIN: Pin 22 (I²C SCL for PT2258)
 * - SW_PWR: Pin 0 (Power switch, active LOW)
 * - SW_OFFS: Pin 25 (Offset mode switch, active LOW)
 * - SW_MUTE: Pin 2 (Mute switch, active LOW)
 * - AMP_EN: Pin 33 (Amplifier enable, active LOW)
 * - HP_EN: Pin 32 (Headphone enable, active LOW)
 * 
 * Control Ranges:
 * - Main Volume: 0 (max) to 79 (min) dB attenuation
 * - Channel Offsets: -15 to +15 dB
 * 
 * Display Modes:
 * - Main Volume: Shows volume level (00-79)
 * - Rear Offset: Shows 'r' followed by offset value
 * - Center Offset: Shows 'c' followed by offset value
 * - Subwoofer Offset: Shows 'S' followed by offset value
 * - Mute: Shows '----'
 * - Power Off: Display blank
 * 
 * Operation Notes:
 * - System auto-returns to main volume mode after 4 seconds of inactivity
 * - Mute state is cleared on power off
 * - Display brightness set to medium level (5/16)
 * - Rotary encoder uses interrupt-driven detection
 * - Buttons implement 200ms debounce protection
 * 
 * Dependencies:
 * -PT2258 (marclura/PT2258-Arduino-Library) https://github.com/marclura/PT2258-Arduino-Library.git
 * -TM1637Display (avishorp/TM1637) https://github.com/avishorp/TM1637.git
 * 
 * References:
 * https://thompdale.com/bash_amplifier/5-1/5-1_bash_amp.htm
 * http://www.daacwave.com/2021/09/how-to-make-51-remote-kit-at-home-with.html
 * 
 * 
 * @note The PT2258 address is fixed at 0x88
 * @note Channel mapping: 1=FL, 2=FR, 3=RL, 4=RR, 5=C, 6=SUB
 * @note All switches use internal pull-up resistors
 * 
 * @author Original code and documentation
 * @date Last updated: [Current Date]
 */

#include <Wire.h>
#include <PT2258.h>
#include <TM1637Display.h>

// --- Pin Definitions ---
#define ENCODER_A 27 // Rotary encoder A pin (active LOW)
#define ENCODER_B 26 // Rotary encoder B pin (active LOW)
#define TM_CLK 14    // TM1637 clock pin
#define TM_DIO 12    // TM1637 data pin
#define SDA_PIN 21   // I²C SDA pin for PT2258
#define SCL_PIN 22   // I²C SCL pin for PT2258
#define SW_PWR 0     // Power-state switch (active low)
#define SW_OFFS 25   // Offset-mode switch (active low)
#define SW_MUTE 2    // Mute toggle switch (active LOW)
#define AMP_EN 33    // Main amplifier enable (active LOW)
#define HP_EN 32     // Headphone amplifier enable (active LOW)
#define PT2258_ADDR 0x88

// --- Global State ---
TM1637Display display(TM_CLK, TM_DIO);
PT2258 amp(PT2258_ADDR);

// UI/channel states
enum Mode
{
  MODE_MAIN,
  MODE_REAR,
  MODE_CENTER,
  MODE_SUB
};
Mode currentMode = MODE_MAIN;
unsigned long lastInputTime = 0;
const unsigned long OFFSET_TIMEOUT = 4000; // milliseconds

// Volume & offset values
#define OFFSET_MAX 15 // As per factory box
#define OFFSET_MIN -15
uint8_t mainVolume = 30; // 0=max, 79=min
int8_t rearOffset = 0;   // -15..+15
int8_t centerOffset = 0; // -15..+15
int8_t subOffset = 0;    // -15..+15

bool ampPower = false;
bool isMuted = false;

// Rotary encoder state
volatile int8_t encoderDelta = 0;
int lastEncA = HIGH, lastEncB = HIGH;

// Debounce for buttons
unsigned long lastButtonPress[3] = {0, 0, 0};
const unsigned long debounceDelay = 200;

// Display characters
const byte SEG_d = {SEG_B | SEG_C | SEG_D | SEG_E | SEG_G};
const byte SEG_o = {SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F}; // O
const byte SEG_s = {SEG_A | SEG_C | SEG_D | SEG_F | SEG_G};         // S
const byte SEG_r = {SEG_A | SEG_B | SEG_C | SEG_D | SEG_E};         // r
const byte SEG_c = {SEG_A | SEG_D | SEG_E | SEG_F};                 // c
const byte SEG_n = {SEG_C | SEG_E | SEG_G};                         // n
const byte SEG_e = {SEG_A | SEG_D | SEG_E | SEG_F | SEG_G};
const u_int8_t SEG_mute[] = {SEG_G, SEG_G, SEG_G, SEG_G}; // Custom: show '----'
const byte SEG_Negative = {SEG_G};                        //- For negative numbers
// --- Function Prototypes ---

// Function prototypes
void updateAllChannels();
void updateDisplay();
void handleButtons();
void handleEncoder();
void displayOff();
void displayOn();
void initDisplay();

// -- Helper: Clamp and map offsets --
uint8_t applyOffset(int8_t base, int8_t offset)
{
  int v = base + offset;
  if (v < 0)
    v = 0;
  if (v > 79)
    v = 79;
  return (uint8_t)v;
}

// --- Rotary Encoder Interrupt ---
void IRAM_ATTR encoderISR()
{
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  if (a != lastEncA)
  {
    if (b != a)
      encoderDelta++;
    else
      encoderDelta--;
    lastEncA = a;
  }
}

// --- Hardware Init ---
void setup()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);

  // Init display
  initDisplay();

  // PT2258 init
  amp.begin();
  amp.mute(false);

  // Rotary encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  // Buttons
  pinMode(SW_PWR, INPUT_PULLUP);
  pinMode(SW_OFFS, INPUT_PULLUP);
  pinMode(SW_MUTE, INPUT_PULLUP);

  // Outputs
  pinMode(AMP_EN, OUTPUT);
  pinMode(HP_EN, OUTPUT);
  digitalWrite(AMP_EN, LOW); // Amp off (active LOW)
  digitalWrite(HP_EN, LOW);
  pinMode(TM_CLK, OUTPUT);
  pinMode(TM_DIO, OUTPUT);

  updateAllChannels();
  updateDisplay();
}

// --- Main Loop ---
void loop()
{
  handleButtons();
  handleEncoder();

  // Timeout for offset mode
  if (currentMode != MODE_MAIN && (millis() - lastInputTime > OFFSET_TIMEOUT))
  {
    currentMode = MODE_MAIN;
    updateDisplay();
  }
}

// --- Button Handling ---
void handleButtons()
{
  unsigned long now = millis();

  // Power button
  if (digitalRead(SW_PWR) == LOW && now - lastButtonPress[0] > debounceDelay)
  {
    lastButtonPress[0] = now;
    ampPower = !ampPower;
    digitalWrite(AMP_EN, ampPower ? HIGH : LOW); // Active LOW
    if (!ampPower)
      isMuted = false; // Reset mute on power-off
    updateDisplay();
  }
  // Offset select button
  if (digitalRead(SW_OFFS) == LOW && now - lastButtonPress[1] > debounceDelay)
  {
    lastButtonPress[1] = now;
    switch (currentMode)
    {
    case MODE_MAIN:
      currentMode = MODE_REAR;
      break;
    case MODE_REAR:
      currentMode = MODE_CENTER;
      break;
    case MODE_CENTER:
      currentMode = MODE_SUB;
      break;
    case MODE_SUB:
      currentMode = MODE_MAIN;
      break;
    }
    lastInputTime = now;
    updateDisplay();
  }
  // Mute button
  if (digitalRead(SW_MUTE) == LOW && now - lastButtonPress[2] > debounceDelay)
  {
    lastButtonPress[2] = now;
    if (ampPower)
    {
      isMuted = !isMuted;
      amp.mute(isMuted);
      updateDisplay();
    }
  }
}

// --- Rotary Encoder for Volume/Offsets ---
void handleEncoder()
{
  int delta;
  noInterrupts();
  delta = encoderDelta;
  encoderDelta = 0;
  interrupts();

  if (delta != 0 && ampPower)
  {
    lastInputTime = millis();
    switch (currentMode)
    {
    case MODE_MAIN:
      mainVolume = constrain(mainVolume + delta, 0, 79);
      break;
    case MODE_REAR:
      rearOffset = constrain(rearOffset + delta, OFFSET_MIN, OFFSET_MAX);
      break;
    case MODE_CENTER:
      centerOffset = constrain(centerOffset + delta, OFFSET_MIN, OFFSET_MAX);
      break;
    case MODE_SUB:
      subOffset = constrain(subOffset + delta, OFFSET_MIN, OFFSET_MAX);
      break;
    }
    updateAllChannels();
    updateDisplay();
  }
}

// --- PT2258: Set all channels appropriately ---
void updateAllChannels()
{
  // 1=Front Left, 2=Front Right, 5=Center, 6=Sub, 3=Rear Left, 4=Rear Right
  amp.volume(1, mainVolume);                            // Front L
  amp.volume(2, mainVolume);                            // Front R
  amp.volume(3, applyOffset(mainVolume, rearOffset));   // Rear L
  amp.volume(4, applyOffset(mainVolume, rearOffset));   // Rear R
  amp.volume(5, applyOffset(mainVolume, centerOffset)); // Center
  amp.volume(6, applyOffset(mainVolume, subOffset));    // Sub
}

void displayOff()
{
  // Serial.println("displayOff()");
  display.clear();
  display.setBrightness(0x00, false); // Set brightness to 0
  return;
}

void displayOn()
{
  // Serial.println("displayOn()");
  display.clear();
  display.setBrightness(0x05, true); // Set brightness to 5
  return;
}

void initDisplay()
{
  // Serial.println("initDisplay()");
  display.setBrightness(0x05, true); // Set brightness to 5
  display.setSegments(&SEG_d, 1, 0); // Custom: show 'd'
  display.setSegments(&SEG_o, 1, 1); // Custom: show 'o'
  display.setSegments(&SEG_n, 1, 2); // Custom: show 'n'
  display.setSegments(&SEG_e, 1, 3); // Custom: show 'e'
  delay(1000);
  display.clear();
  display.showNumberDec(0, true, 4, 0); // Show "00"
  return;
}

// --- Display Logic: Show relevant info ---
void updateDisplay()
{
  display.clear();
  if (!ampPower)
  {
    displayOff();
    return;
  }
  else if (ampPower)
  {
    displayOn();
    if (isMuted)
    {
      display.setSegments(SEG_mute, 4, 0); // Custom: show '----'
      return;
    }
    switch (currentMode)
    {
    case MODE_MAIN:
      display.showNumberDec(mainVolume, false, 2, 2);
      display.showNumberDec(0, false, 2, 0); // blank left
      break;
    case MODE_REAR:
      // 'r' left, offset value right
      display.setSegments(&SEG_r, 1, 0); // Custom: show 'r'
      if (rearOffset < 0)
      {
        display.setSegments(&SEG_Negative, 1, 1);
      } //- For negative numbers
      else
      {
        display.showNumberDec(0, 0, false, 1);
      }
      display.showNumberDec(rearOffset, true, 2, 2);
      break;
    case MODE_CENTER:
      // 'c' left, offset value right
      display.setSegments(&SEG_c, 1, 0); // Custom: show 'c'
      if (centerOffset < 0)
      {
        display.setSegments(&SEG_Negative, 1, 1);
      }
      else
      {
        display.showNumberDec(0, 0, false, 1);
      }
      //- For negative numbers
      display.showNumberDec(centerOffset, true, 2, 2);
      break;
    case MODE_SUB:
      // 'S' left, offset value right
      display.setSegments(&SEG_s, 1, 0); // Custom: show 'S'
      if (subOffset < 0)
      {
        display.setSegments(&SEG_Negative, 1, 1); //- For negative numbers
      }
      else
      {
        display.showNumberDec(0, 0, false, 1);
      }
      display.showNumberDec(subOffset, true, 2, 2);
      break;
    default:
      break;
    }
    return;
  }
}
