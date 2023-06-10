/*
 * Main code for GART6520's bot for VRC2023
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

/******************************************************************
 * Pins config for the library :
 * - On the motorshield of VIA Makerbot BANHMI, there is a 6-pin
 *   header designed for connecting PS2 remote controller.
 * Header pins and corresponding GPIO pins:
 *   MOSI | MISO | GND | 3.3V | CS | CLK
 *    12     13    GND   3.3V   15   14
 ******************************************************************/

#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SLK

/******************************************************************
 * Select mode for PS2 controller:
 *   - pressures = Read analog value from buttons
 *   - rumble    = Turn on / off rumble mode
 ******************************************************************/
#define pressures false
#define rumble false

PS2X ps2x; // Create ps2x instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Create pwm instance

void connect_ps2()
{
  // Connect until it's connected, not just dying in the middle of the competition
  while (1) {
    Serial.print("Connecting to PS2 controller: ");

    int error = -1;
    for (int i = 0; i < 10; i++) // Trying to connect to PS2 controller. Max 10 times
    {
      delay(1000); // Wait 1 sec
      // Config pins and modes: GamePad(clock, command, attention, data, Pressures?, Rumble?)
      error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
      if (error == 0) break; // Break if connected, to save time
      Serial.print(".");
    }

    switch (error) // Check the error code
    {
    case 0:
      Serial.println("Connected to PS2 controller");
      break;
    case 1:
      Serial.println("Error: Cannot find controller. Check the connection");
      break;
    case 2:
      Serial.println("Error: Cannot send command");
      break;
    case 3:
      Serial.println("Error: Cannot enter pressures mode");
      break;
    }

    if (error == 0) break;
  }
}

void setup()
{
  Serial.begin(115200);

  // Connect to PS2 controller
  connect_ps2();

  // Init motor controller
  pwm.begin(); // Initialize PCA9685 
  pwm.setOscillatorFrequency(27000000); // Set frequency for PCA9685
  pwm.setPWMFreq(50); // PWM frequency. Should be 50-60 Hz for controlling both DC motor and servo
  Wire.setClock(400000); // Set to max i2c frequency @ 400000
}

void loop()
{
  // Read the gamepad state
  ps2x.read_gamepad(false, false);

  Serial.print("Stick Values:");
  Serial.print(ps2x.Analog(PSS_LY)); // Y axis of the left joystick
  Serial.print(",");
  Serial.println(ps2x.Analog(PSS_RY), DEC); // Y axis of the right joystick

  // Default coefficient for driving
  int co = 16;
  if (ps2x.Button(PSB_R2)) {
    // Boost mode
    co = 32;
  }

  // Left wheel control
  if (ps2x.Analog(PSS_LY) < 127) {
    // Forward

    // Motor
    pwm.setPWM(8, 0, co * (128 - ps2x.Analog(PSS_LY)));
    pwm.setPWM(9, 0, 0);

    // Support servo
    //if (ps2x.Button(PSB_L1)) pwm.setPWM(7, 0, 0);
  } else if (ps2x.Analog(PSS_LY) > 128) {
    // Backward

    // Motor
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, co * (ps2x.Analog(PSS_LY) - 128));

    // Support servo
    //if (ps2x.Button(PSB_L1)) pwm.setPWM(7, 0, 180);
  } else {
    // Stop

    // Motor
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, 0);

    // Support servo
    //pwm.setPWM(7, 0, 0);
  }

  // Right wheel control
  if (ps2x.Analog(PSS_RY) < 127) {
    // Forward

    // Motor
    pwm.setPWM(14, 0, 0);
    pwm.setPWM(15, 0, co * (128 - ps2x.Analog(PSS_RY)));

    // Support servo
    //if (ps2x.Button(PSB_L1)) pwm.setPWM(2, 0, 180);
  } else if (ps2x.Analog(PSS_RY) > 128) {
    // Backward

    // Motor
    pwm.setPWM(14, 0, co * (ps2x.Analog(PSS_RY) - 128));
    pwm.setPWM(15, 0, 0);

    // Support servo
    //if (ps2x.Button(PSB_L1)) pwm.setPWM(2, 0, 0);
  } else {
    // Stop

    // Motor
    pwm.setPWM(14, 0, 0);
    pwm.setPWM(15, 0, 0);

    // Support servo
    //pwm.setPWM(2, 0, 0);
  }

  if (ps2x.Button(PSB_L1)) {
    if (ps2x.Button(PSB_L2)) {
      pwm.setPWM(10, 0, 4095);
      pwm.setPWM(11, 0, 0);
    } else {
      pwm.setPWM(10, 0, 0);
      pwm.setPWM(11, 0, 4095);
    }
  } else {
    pwm.setPWM(10, 0, 0);
    pwm.setPWM(11, 0, 0);
  }

  if (ps2x.Button(PSB_R1)) {
    if (ps2x.Button(PSB_L2)) {
      pwm.setPWM(12, 0, 4095);
      pwm.setPWM(13, 0, 0);
    } else {
      pwm.setPWM(12, 0, 0);
      pwm.setPWM(13, 0, 4095);
    }
  } else {
    pwm.setPWM(12, 0, 0);
    pwm.setPWM(13, 0, 0);
  }

  // Delay or fucking things will happend with esp32
  // If it still crashes consider feeding or disabling watchdog
  delay(50);
}
