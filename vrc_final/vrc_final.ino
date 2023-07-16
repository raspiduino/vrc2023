/*
 * Main code for GART6520's bot for VRC2023
 */

// Enable / disable debug log
#define DEBUG

// For controlling PCA9685
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// For using PS2 controller
#include <PS2X_lib.h>

#define dbg Serial

// PWM mapping
const int left_1 = 8;
const int left_2 = 9;
const int right_1 = 14;
const int right_2 = 15;
const int fintake_1 = 10;
const int fintake_2 = 11;
const int bintake_1 = 12;
const int bintake_2 = 13;
const int catapult = 7;

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

void setup() {
#ifdef DEBUG
  // Initialize serial
  Serial.begin(115200);
#endif

  // Connect to PS2 
  Serial.print("Ket noi voi tay cam PS2:");

  int error = -1;
  for (int i = 0; i < 10; i++) // thử kết nối với tay cầm ps2 trong 10 lần
  {
    delay(1000); // đợi 1 giây
    // cài đặt chân và các chế độ: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
    if (error == 0) break;
  }

  switch (error) // kiểm tra lỗi nếu sau 10 lần không kết nối được
  {
  case 0:
    Serial.println(" Ket noi tay cam PS2 thanh cong");
    break;
  case 1:
    Serial.println(" LOI: Khong tim thay tay cam, hay kiem tra day ket noi vơi tay cam ");
    break;
  case 2:
    Serial.println(" LOI: khong gui duoc lenh");
    break;
  case 3:
    Serial.println(" LOI: Khong vao duoc Pressures mode ");
    break;
  }

  // Init motor controller
  pwm.begin(); // Initialize PCA9685 
  pwm.setOscillatorFrequency(27000000); // Set frequency for PCA9685
  pwm.setPWMFreq(50); // PWM frequency. Should be 50-60 Hz for controlling both DC motor and servo
  Wire.setClock(400000); // Set to max i2c frequency @ 400000
}

// Control mode
int bco = 19;
bool pov_control = false;   // POV control mode or joystick control mode
bool toggle_intake = false; // Enable both intake or not

void loop() {
  // Read the gamepad state
  ps2x.read_gamepad(false, false);

#ifdef DEBUG
  dbg.print("Stick Values:");
  dbg.print(ps2x.Analog(PSS_LY)); // Y axis of the left joystick
  dbg.print(",");
  dbg.println(ps2x.Analog(PSS_RY), DEC); // Y axis of the right joystick
#endif

  // Coefficient for driving
  if (ps2x.NewButtonState(PSB_R2)) {
    if (ps2x.Button(PSB_R2)) {
      // Boost mode
      bco = 29;
    } else {
      bco = 19;
    }
  }

  // Joystick drive mode
  if (!pov_control) {
    // Normal tank drive mode
    // Left wheel control
    if (ps2x.Analog(PSS_LY) <= 127) {
      // Forward
      pwm.setPWM(left_1, 0, bco * (128 - ps2x.Analog(PSS_LY)));
      pwm.setPWM(left_2, 0, 0);
    } else {
      // Backward
      pwm.setPWM(left_1, 0, 0);
      pwm.setPWM(left_2, 0, bco * (ps2x.Analog(PSS_LY) - 128));
    }

    // Right wheel control
    if (ps2x.Analog(PSS_RY) <= 127) {
      // Forward
      pwm.setPWM(right_1, 0, 0);
      pwm.setPWM(right_2, 0, bco * (128 - ps2x.Analog(PSS_RY)));
    } else {
      // Backward
      pwm.setPWM(right_1, 0, bco * (ps2x.Analog(PSS_RY) - 128));
      pwm.setPWM(right_2, 0, 0);
    }
  }

  // Intake 1
  if (ps2x.Button(PSB_L1)) {
    if (ps2x.Button(PSB_L2)) {
      pwm.setPWM(bintake_1, 0, 4095);
      pwm.setPWM(bintake_2, 0, 0);
    } else {
      pwm.setPWM(bintake_1, 0, 0);
      pwm.setPWM(bintake_2, 0, 4095);
    }
  } else {
    if (!toggle_intake) {                                                                                           
      pwm.setPWM(bintake_1, 0, 0);
      pwm.setPWM(bintake_2, 0, 0);
    }
  }

  // Intake 2
  if (ps2x.Button(PSB_R1)) {
    if (ps2x.Button(PSB_GREEN)) {
      pwm.setPWM(fintake_1, 0, 4095);
      pwm.setPWM(fintake_2, 0, 0);
    } else {
      pwm.setPWM(fintake_1, 0, 0);
      pwm.setPWM(fintake_2, 0, 4095);
    }
  } else {
    if (!toggle_intake) {
      pwm.setPWM(fintake_1, 0, 0);
      pwm.setPWM(fintake_2, 0, 0);
    }
  }

  // Catapult servo
  if (!ps2x.Button(PSB_BLUE)) {
    // Turn it off
    pwm.setPWM(catapult, 0, 0);
  } else {
    // Turn it on
    if (ps2x.Button(PSB_L2) || ps2x.Button(PSB_GREEN)) {
      pwm.setPWM(catapult, 0, 400);
    } else {
      pwm.setPWM(catapult, 0, 150);
    }
  }

  // POV up
  if (ps2x.NewButtonState(PSB_PAD_UP)) {
    dbg.println("New PSB_PAD_UP state");
    if (!ps2x.Button(PSB_PAD_UP)) {
      // Stop bot
      dbg.println("Stop bot 0");
      pwm.setPWM(right_1, 0, 0);
      pwm.setPWM(right_2, 0, 0);
      pwm.setPWM(right_1, 0, 0);
      pwm.setPWM(right_2, 0, 0);
      pov_control = false;
    } else {
      // Go forward
      pov_control = true;
      dbg.println("Go forward");
      pwm.setPWM(left_1, 0, 128 * bco);
      pwm.setPWM(left_2, 0, 0);
      pwm.setPWM(right_1, 0, 0);
      pwm.setPWM(right_2, 0, 128 * bco);
    }
  }
  
  // POV down
  if (ps2x.NewButtonState(PSB_PAD_DOWN)) {
    dbg.println("New PSB_PAD_DOWN state");
    if (!ps2x.Button(PSB_PAD_DOWN)) {
      // Stop bot
      dbg.println("Stop bot 1");
      pwm.setPWM(right_1, 0, 0);
      pwm.setPWM(right_2, 0, 0);
      pwm.setPWM(right_1, 0, 0);
      pwm.setPWM(right_2, 0, 0);
      pov_control = false;
    } else {
      // Go backward
      pov_control = true;
      dbg.println("Go backward");
      pwm.setPWM(left_1, 0, 0);
      pwm.setPWM(left_2, 0, 128 * bco);
      pwm.setPWM(right_1, 0, 128 * bco);
      pwm.setPWM(right_2, 0, 0); 
    }
  }

  // Toggle both intake to input
  if (ps2x.NewButtonState(PSB_RED)) {
    if (ps2x.Button(PSB_RED)) {
      toggle_intake = !toggle_intake;
      if (toggle_intake) {
        pwm.setPWM(bintake_1, 0, 0);
        pwm.setPWM(bintake_2, 0, 4095);
        pwm.setPWM(fintake_1, 0, 0);
        pwm.setPWM(fintake_2, 0, 4095);
      } else {
        pwm.setPWM(bintake_1, 0, 0);
        pwm.setPWM(bintake_2, 0, 0);
        pwm.setPWM(fintake_1, 0, 0);
        pwm.setPWM(fintake_2, 0, 0);
      }
    }
  }

  // Delay or fucking things will happend with esp32
  // If it still crashes consider feeding or disabling watchdog
  delay(10);
}
