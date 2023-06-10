/*
 * Main code for GART6520's bot for VRC2023
 */

// For web interface
#include <DNSServer.h>
#include <ESPUI.h>

// For controlling PCA9685
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// For using PS2 controller
#include <PS2X_lib.h>

// For storing mapping
#include <Preferences.h>

// For HTTPS client
#include <HTTPClient.h>
#include <HTTPUpdate.h>

// Root CA certificate for github.com
const char* rootCACertificate = "-----BEGIN CERTIFICATE-----\nMIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\nMQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\nd3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\nQTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\nMRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\nb20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\nCSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\nnh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\nT19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\ngdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\nBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\nTLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\nDQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\nhMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\nPnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\nYSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\nCAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n-----END CERTIFICATE-----\n";

// Create pref instance
Preferences pref;

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

// Servo PWM pin mapping
#define Servo_1 2
#define Servo_2 3
#define Servo_3 4
#define Servo_4 5
#define Servo_5 6
#define Servo_6 7

// Initialize network
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
DNSServer dnsServer;
#include <WiFi.h>

const char *c_ssid = "VIETQUOCLAW";
const char *c_password = "123456ok";

const char *ssid = "Let us cook";
const char *password = "memaybeotiful";

const char *hostname = "mmb.io";

bool bot_wifi_mode = false;

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

void setPWM(int chan1, int chan2, bool state, uint16_t val)
{
  Serial.println(val);
  if (state)
  {
    pwm.setPWM(chan1, 0, val);
    pwm.setPWM(chan2, 0, 0);
    //    pwm.setPWM(chan1,  val,0 );
    //    pwm.setPWM(chan2, 0, 4096 );
  }
  else
  {
    pwm.setPWM(chan2, 0, val);
    pwm.setPWM(chan1, 0, 0);
    //pwm.setPWM(chan2,  val,0 );
    //    pwm.setPWM(chan1, 0, 4096 );
  }
}

void DC_slider(Control *sender, int type)
{
  Serial.print("Slider: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");

  int16_t val;
  if (sender->value)
    val = (sender->value).toInt() * 4095 / 100;
  else
    val = 0;

  bool dir = 0x8000 & val;
  val = abs(val);

  if (val < 250) {
    val = 0;
    dir = 0;
  }

  switch(sender->id){
  case 2:
  {
    setPWM(8, 9, dir ,  val);
    //setPWM(14, 15, dir,  val);
    break;
  }
  case 5:
  {
    setPWM(10, 11,dir, val);
    break;
  }
  case 8 :
  {
       setPWM(12, 13, dir,  val);
    break;
  }
  case 11:
  {
    setPWM(14, 15, dir,  val);
    break;
  }
  }
  Serial.println(val);

}

void Servo_slider(Control *sender, int type)
{
  Serial.print("Slider: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");

  uint16_t val;
  if (sender->value)
    val = (sender->value).toInt();
  else
    val = 0;
  Serial.println(val);
  switch(sender->id){
  case 15:
  {
        pwm.setPWM(Servo_1, 0, val);
    break;
  }
  case 18:
  {
            pwm.setPWM(Servo_2, 0, val);
    break;
  }
  case 21 :
  {
        pwm.setPWM(Servo_3, 0, val);
    break;
  }
  case 24:
  {
        pwm.setPWM(Servo_4, 0, val);
    break;
  }
  case 27:
  {
        pwm.setPWM(Servo_5, 0, val);
    break;
  }
    case 30:
  {
        pwm.setPWM(Servo_6, 0, val);
    break;
  }
  }
}

// Not sure if WiFiClientSecure checks the validity date of the certificate. 
// Setting clock just to be sure...
void setClock() {
  configTime(0, 0, "pool.ntp.org");

  Serial.print(F("Waiting for NTP time sync: "));
  time_t nowSecs = time(nullptr);
  while (nowSecs < 8 * 3600 * 2) {
    delay(500);
    Serial.print(F("."));
    yield();
    nowSecs = time(nullptr);
  }

  Serial.println();
  struct tm timeinfo;
  gmtime_r(&nowSecs, &timeinfo);
  Serial.print(F("Current time: "));
  Serial.print(asctime(&timeinfo));
}

void update_fw()
{
  Serial.println("\nUpdating from Github...");
  WiFiClientSecure client;
  client.setCACert(rootCACertificate);

  // Reading data over SSL may be slow, use an adequate timeout
  client.setTimeout(12000 / 1000); // timeout argument is defined in seconds for setTimeout

  // The line below is optional. It can be used to blink the LED on the board during flashing
  // The LED will be on during download of one buffer of data from the network. The LED will
  // be off during writing that buffer to flash
  // On a good connection the LED should flash regularly. On a bad connection the LED will be
  // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
  // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
  httpUpdate.setLedPin(13, HIGH);

  /*t_httpUpdate_return ret = httpUpdate.update(client, "https://server/file.bin", "", [](HTTPClient *client) {
    client->setAuthorization("test", "password");
  });*/

  // Or:
  t_httpUpdate_return ret = httpUpdate.update(client, "raw.githubusercontent.com", 443, "/raspiduino/esp32-ota-update/main/vrc_mixed.ino.bin");

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }

  Serial.println("Done updating from Github!");
}

void init_wifi()
{
  // Set esp32 hostname
  WiFi.setHostname(hostname);

  uint8_t timeout = 10;

  // Try to connect to existing network
  if (bot_wifi_mode) {
    // If selected to connect to network
    WiFi.begin(c_ssid, c_password);
    Serial.print("\n\nTry to connect to existing network");

    // Wait for connection, 5s timeout
    do
    {
      delay(500);
      Serial.print(".");
      timeout--;
    } while (timeout && WiFi.status() != WL_CONNECTED);

    // If connected -> get out of here
    if (WiFi.status() == WL_CONNECTED) {
      // Update firmware from Github
      update_fw();
      return;
    }
  }
  
  // Create hotspot
  {
    Serial.print("\n\nCreating hotspot");

    WiFi.begin(ssid, password);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(ssid);

    timeout = 5;

    do
    {
      delay(500);
      Serial.print(".");
      timeout--;
    } while (timeout);
  }

  dnsServer.start(DNS_PORT, "*", apIP);

  Serial.println("\n\nWiFi parameters:");
  Serial.print("Mode: ");
  Serial.println(WiFi.getMode() == WIFI_AP ? "Station" : "Client");
  Serial.print("IP address: ");
  Serial.println(WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP());

  ESPUI.begin("VRC2023");
}

// pref variables
int left_1, left_2, right_1, right_2, fintake_1, fintake_2, bintake_1, bintake_2;
int catapult;

void pref_set_default(const char* key, int val) {
  if (pref.getInt(key, -1) == -1) {
    pref.putInt(key, val);
  }
}

void init_pref()
{
  // Initialize pref storage
  pref.begin("vrc2023", false);

  // Check and set default value for config if it does not exist
  pref_set_default("left_1", 8);
  pref_set_default("left_2", 9);
  pref_set_default("right_1", 14);
  pref_set_default("right_2", 15);
  pref_set_default("fintake_1", 10);
  pref_set_default("fintake_2", 11);
  pref_set_default("bintake_1", 12);
  pref_set_default("bintake_2", 13);
  pref_set_default("catapult", 2);

  // Read values
  left_1 = pref.getInt("left_1", 8);
  left_2 = pref.getInt("left_2", 9);
  right_1 = pref.getInt("right_1", 14);
  right_2 = pref.getInt("right_2", 15);
  fintake_1 = pref.getInt("fintake_1", 10);
  fintake_2 = pref.getInt("fintake_2", 11);
  bintake_1 = pref.getInt("bintake_1", 12);
  bintake_2 = pref.getInt("bintake_2", 13);
  catapult = pref.getInt("catapult", 2);
}

void setup(void)
{
  // Just to make sure it still works
  pinMode(13, OUTPUT);
  
  // Set mode for ESPUI
  ESPUI.setVerbosity(Verbosity::VerboseJSON);
  
  // Initialize serial
  Serial.begin(115200);

  // Connect to PS2 controller
  connect_ps2();
  digitalWrite(13, 1);

  // Init motor controller
  pwm.begin(); // Initialize PCA9685 
  pwm.setOscillatorFrequency(27000000); // Set frequency for PCA9685
  pwm.setPWMFreq(50); // PWM frequency. Should be 50-60 Hz for controlling both DC motor and servo
  Wire.setClock(400000); // Set to max i2c frequency @ 400000
  //Wire.setClock(70000);

  // Mapping init
  init_pref();

  // Setup pins for hardware enable/disable of WIFI (incase PS2 controller does not work)
  pinMode(25, INPUT_PULLUP);
  pinMode(32, INPUT_PULLUP);

  // ESPUI init
  uint16_t tab1 = ESPUI.addControl(ControlType::Tab, "", "DC Motor");
//  ESPUI.addControl(ControlType::Switcher, "Switch two", "", ControlColor::None, tab1, &otherSwitchExample1);
  uint16_t  slider_1 = ESPUI.addControl(ControlType::Slider, "DC Motor 1", "0", ControlColor::Turquoise, tab1, &DC_slider);
  ESPUI.addControl(Min, "", "0", None, slider_1);
  ESPUI.addControl(Max, "", "100", None, slider_1);
  
  uint16_t slider_2 = ESPUI.addControl(ControlType::Slider, "DC motor 2", "0", ControlColor::Emerald, tab1, &DC_slider);
  ESPUI.addControl(Min, "", "-100", None, slider_2);
  ESPUI.addControl(Max, "", "100", None, slider_2);
  
  uint16_t slider_3 = ESPUI.addControl(ControlType::Slider, "DC motor 3", "0", ControlColor::Peterriver, tab1, &DC_slider);
  ESPUI.addControl(Min, "", "-100", None, slider_3);
  ESPUI.addControl(Max, "", "100", None, slider_3);
  
  uint16_t slider_4 = ESPUI.addControl(ControlType::Slider, "DC motor 4", "0", ControlColor::Wetasphalt, tab1, &DC_slider);
  ESPUI.addControl(Min, "", "0", None, slider_4);
  ESPUI.addControl(Max, "", "100", None, slider_4);

  uint16_t tab2 = ESPUI.addControl(ControlType::Tab, "", "Servos");
  
  uint16_t  slider_5 = ESPUI.addControl(ControlType::Slider, "Servo 1", "0", ControlColor::Turquoise, tab2, &Servo_slider);
   ESPUI.addControl(Min, "", "70", None, slider_5);
  ESPUI.addControl(Max, "", "600", None, slider_5);
    
  uint16_t  slider_6 = ESPUI.addControl(ControlType::Slider, "Servo 2", "0", ControlColor::Carrot, tab2, &Servo_slider);
   ESPUI.addControl(Min, "", "70", None, slider_6);
  ESPUI.addControl(Max, "", "600", None, slider_6);  
  
  uint16_t  slider_7 = ESPUI.addControl(ControlType::Slider, "Servo 3", "0", ControlColor::Alizarin, tab2, &Servo_slider);
   ESPUI.addControl(Min, "", "70", None, slider_7);
  ESPUI.addControl(Max, "", "600", None, slider_7);

    
  uint16_t  slider_8 = ESPUI.addControl(ControlType::Slider, "Servo 4", "0", ControlColor::Peterriver, tab2, &Servo_slider);
   ESPUI.addControl(Min, "", "70", None, slider_8);
  ESPUI.addControl(Max, "", "600", None, slider_8);
    
  uint16_t  slider_9 = ESPUI.addControl(ControlType::Slider, "Servo 5", "0", ControlColor::Wetasphalt, tab2, &Servo_slider);
   ESPUI.addControl(Min, "", "70", None, slider_9);
  ESPUI.addControl(Max, "", "600", None, slider_9);  
  uint16_t  slider_10 = ESPUI.addControl(ControlType::Slider, "Servo 6", "0", ControlColor::Emerald, tab2, &Servo_slider);
   ESPUI.addControl(Min, "", "70", None, slider_10);
  ESPUI.addControl(Max, "", "600", None, slider_10);
  
  /*
     .begin loads and serves all files from PROGMEM directly.
     If you want to serve the files from SPIFFS use ESPUI.beginSPIFFS
     (.prepareFileSystem has to be run in an empty sketch before)
  */

  // Enable this option if you want sliders to be continuous (update during move) and not discrete (update on stop)
  ESPUI.sliderContinuous = true;

  /*
     Optionally you can use HTTP BasicAuth. Keep in mind that this is NOT a
     SECURE way of limiting access.
     Anyone who is able to sniff traffic will be able to intercept your password
     since it is transmitted in cleartext. Just add a string as username and
     password, for example begin("ESPUI Control", "username", "password")
  */
//  motorSlider a(18, 19, "acnv");
  //ESPUI.begin("VRC2023", "mmb", "memaybeotiful");
}

bool ps2_control = true;

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
    co = 30;
  }

  // Left wheel control
  if (ps2x.Analog(PSS_LY) < 127) {
    // Forward

    // Motor
    pwm.setPWM(left_1, 0, co * (128 - ps2x.Analog(PSS_LY)));
    pwm.setPWM(left_2, 0, 0);

    // Support servo
    //if (ps2x.Button(PSB_L1)) pwm.setPWM(7, 0, 0);
  } else if (ps2x.Analog(PSS_LY) > 128) {
    // Backward

    // Motor
    pwm.setPWM(left_1, 0, 0);
    pwm.setPWM(left_2, 0, co * (ps2x.Analog(PSS_LY) - 128));

    // Support servo
    //if (ps2x.Button(PSB_L1)) pwm.setPWM(7, 0, 180);
  } else {
    // Stop

    // Motor
    if (ps2_control) {
      pwm.setPWM(left_1, 0, 0);
      pwm.setPWM(left_2, 0, 0);
    }

    // Support servo
    //pwm.setPWM(7, 0, 0);
  }

  // Right wheel control
  if (ps2x.Analog(PSS_RY) < 127) {
    // Forward

    // Motor
    pwm.setPWM(right_1, 0, 0);
    pwm.setPWM(right_2, 0, co * (128 - ps2x.Analog(PSS_RY)));

    // Support servo
    //if (ps2x.Button(PSB_L1)) pwm.setPWM(2, 0, 180);
  } else if (ps2x.Analog(PSS_RY) > 128) {
    // Backward

    // Motor
    pwm.setPWM(right_1, 0, co * (ps2x.Analog(PSS_RY) - 128));
    pwm.setPWM(right_2, 0, 0);

    // Support servo
    //if (ps2x.Button(PSB_L1)) pwm.setPWM(2, 0, 0);
  } else {
    // Stop

    // Motor
    if (ps2_control) {
      pwm.setPWM(right_1, 0, 0);
      pwm.setPWM(right_2, 0, 0);
    }

    // Support servo
    //pwm.setPWM(2, 0, 0);
  }

  if (ps2x.Button(PSB_L1)) {
    if (ps2x.Button(PSB_L2)) {
      pwm.setPWM(fintake_1, 0, 4095);
      pwm.setPWM(fintake_2, 0, 0);
    } else {
      pwm.setPWM(fintake_1, 0, 0);
      pwm.setPWM(fintake_2, 0, 4095);
    }
  } else {
    if (ps2_control) {
      pwm.setPWM(fintake_1, 0, 0);
      pwm.setPWM(fintake_2, 0, 0);
    }
  }

  if (ps2x.Button(PSB_R1)) {
    if (ps2x.Button(PSB_L2)) {
      pwm.setPWM(bintake_1, 0, 4095);
      pwm.setPWM(bintake_2, 0, 0);
    } else {
      pwm.setPWM(bintake_1, 0, 0);
      pwm.setPWM(bintake_2, 0, 4095);
    }
  } else {
    if (ps2_control) {
      pwm.setPWM(bintake_1, 0, 0);
      pwm.setPWM(bintake_2, 0, 0);
    }
  }

  // Catapult servo
  if (ps2x.NewButtonState(PSB_GREEN)) {
    if (!ps2x.Button(PSB_GREEN)) {
      // Turn it off
      if (ps2_control) pwm.setPWM(catapult, 0, 0);
    } else {
      // Turn it on
      if (ps2x.Button(PSB_L2)) {
        pwm.setPWM(catapult, 0, 400);
      } else {
        pwm.setPWM(catapult, 0, 150);
      }
    }
  }

  // Check for WIFI turn on / off request from GPIO and PS2
  if (digitalRead(25) == 0 || ps2x.ButtonPressed(PSB_START)) {
    // Turn on WIFI
    Serial.println("Turn on WIFI");
    init_wifi();
    bot_wifi_mode = !bot_wifi_mode; // Switch mode
    ps2_control = false;
  }

  if (digitalRead(32) == 0 || ps2x.ButtonPressed(PSB_SELECT)) {
    // Turn off WIFI
    Serial.print("PSB_SELECT_STATE: ");
    Serial.println(ps2x.ButtonPressed(PSB_SELECT));
    Serial.println("Turn off WIFI");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    ps2_control = true;
  }

  // Process DNS request
  dnsServer.processNextRequest();

  // Delay or fucking things will happend with esp32
  // If it still crashes consider feeding or disabling watchdog
  delay(50);
}
