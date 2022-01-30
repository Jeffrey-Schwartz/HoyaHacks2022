#include <ArduinoBLE.h>        // Bluetooth
#include <Arduino_LSM6DS3.h>   // IMU: Accelerometer & Gyroscope

const int out1     =   1; // Digital (output) pin used (D1)
const int out2     =   2; // Digital (output) pin used (D2)
const int out3     =   3; // Digital (output) pin used (D3)
const int out4     =   4; // Digital (output) pin used (D4)

long lastTimer1;      // Last time (in ms) light toggle-1
long lastTimer2;      // Last time (in ms) light toggle-2
long lastTimer3;      // Last time (in ms) light toggle-3
long lastTimer4;      // Last time (in ms) light toggle-4

bool enableTimer1;    // Turn on(true)/off(false) timer-1
bool enableTimer2;    // Turn on(true)/off(false) timer-2
bool enableTimer3;    // Turn on(true)/off(false) timer-3
bool enableTimer4;    // Turn on(true)/off(false) timer-4

short perMinute1;     // Blink frequency (number of blink cycles per minute)
float timerDelay1;    // Time between blink cycles (30000/perMinute); milliseconds

short perMinute2;     // Blink frequency (number of blink cycles per minute)
float timerDelay2;    // Time between blink cycles (30000/perMinute); milliseconds

short perMinute3;     // Blink frequency (number of blink cycles per minute)
float timerDelay3;    // Time between blink cycles (30000/perMinute); milliseconds

short perMinute4;     // Blink frequency (number of blink cycles per minute)
float timerDelay4;    // Time between blink cycles (30000/perMinute); milliseconds

int ledState1;        // On(LOW)/Off(HIGH) state of indicator LED
int ledState2;        // On(LOW)/Off(HIGH) state of indicator LED
int ledState3;        // On(LOW)/Off(HIGH) state of indicator LED
int ledState4;        // On(LOW)/Off(HIGH) state of indicator LED

bool SOS;             // Flag for SOS blink pattern

BLEService bikeLightService("X2AFB");
BLEBoolCharacteristic enableTimer1Char("0001", BLERead | BLEWrite | BLENotify);
BLEShortCharacteristic perMinute1Char("0002", BLERead | BLEWrite | BLENotify);
BLEBoolCharacteristic OnOffState1Char("0003", BLERead | BLEWrite | BLENotify);
BLEBoolCharacteristic enableTimer2Char("0004", BLERead | BLEWrite | BLENotify);
BLEShortCharacteristic perMinute2Char("0005", BLERead | BLEWrite | BLENotify);
BLEBoolCharacteristic OnOffState2Char("0006", BLERead | BLEWrite | BLENotify);
BLEBoolCharacteristic enableTimer3Char("0007", BLERead | BLEWrite | BLENotify);
BLEShortCharacteristic perMinute3Char("0008", BLERead | BLEWrite | BLENotify);
BLEBoolCharacteristic OnOffState3Char("0009", BLERead | BLEWrite | BLENotify);
BLEBoolCharacteristic enableTimer4Char("0010", BLERead | BLEWrite | BLENotify);
BLEShortCharacteristic perMinute4Char("0011", BLERead | BLEWrite | BLENotify);
BLEBoolCharacteristic OnOffState4Char("0012", BLERead | BLEWrite | BLENotify);
BLEFloatCharacteristic TemperatureChar("2A1C", BLERead | BLEWrite | BLENotify);
BLEBoolCharacteristic SOSChar("0101", BLERead | BLEWrite | BLENotify);

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);
  pinMode(out4, OUTPUT);

  Serial.println(LED_BUILTIN);

  // Initialize Serial connection
  Serial.begin(9600);
  //while (!Serial);

  // Initialize IMU
  if (!IMU.begin())
  {
    while (1);
  }

  // Initialize BLE
  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    //while (1);
  }
  else
  {
    /* Set a local name for the BLE device
       This name will appear in advertising packets
       and can be used by remote devices to identify this BLE device
       The name can be changed but may be truncated based on space left in advertisement packet
    */
    BLE.setLocalName("Bike Light");
    BLE.setAdvertisedService(bikeLightService);           // add the service UUID
    bikeLightService.addCharacteristic(enableTimer1Char); // add the BLE characteristics
    bikeLightService.addCharacteristic(perMinute1Char);
    bikeLightService.addCharacteristic(OnOffState1Char);
    bikeLightService.addCharacteristic(enableTimer2Char);
    bikeLightService.addCharacteristic(perMinute2Char);
    bikeLightService.addCharacteristic(OnOffState2Char);
    bikeLightService.addCharacteristic(enableTimer3Char);
    bikeLightService.addCharacteristic(perMinute3Char);
    bikeLightService.addCharacteristic(OnOffState3Char);    
    bikeLightService.addCharacteristic(enableTimer4Char);
    bikeLightService.addCharacteristic(perMinute4Char);
    bikeLightService.addCharacteristic(OnOffState4Char);
    bikeLightService.addCharacteristic(TemperatureChar);
    bikeLightService.addCharacteristic(SOSChar);
    BLE.addService(bikeLightService);                     // Add the BLE service
    reset(); // Set global variables and write to BLE characteristics
    enableTimer1Char.setEventHandler(BLEWritten, toggleBlinking1);
    perMinute1Char.setEventHandler(BLEWritten, updateBlinkRate1);
    OnOffState1Char.setEventHandler(BLEWritten, OnOff1);
    enableTimer2Char.setEventHandler(BLEWritten, toggleBlinking2);
    perMinute2Char.setEventHandler(BLEWritten, updateBlinkRate2);
    OnOffState2Char.setEventHandler(BLEWritten, OnOff2);
    enableTimer3Char.setEventHandler(BLEWritten, toggleBlinking3);
    perMinute3Char.setEventHandler(BLEWritten, updateBlinkRate3);
    OnOffState3Char.setEventHandler(BLEWritten, OnOff3);
    enableTimer4Char.setEventHandler(BLEWritten, toggleBlinking4);
    perMinute4Char.setEventHandler(BLEWritten, updateBlinkRate4);
    OnOffState4Char.setEventHandler(BLEWritten, OnOff4);
    SOSChar.setEventHandler(BLEWritten, setSOS);
    /* Start advertising BLE.  It will start continuously transmitting BLE
       advertising packets and will be visible to remote BLE central devices
       until it receives a new connection */
    // start advertising
    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");
  }
}

void reset()
{
  // Initialize global variables
  enableTimer1 = true;
  perMinute1 = 360;                 // (6 Hz, default)
  timerDelay1 = 30000.0/perMinute1; // period in milliseconds
  enableTimer2 = true;
  perMinute2 = 360;                 // (6 Hz, default)
  timerDelay2 = 30000.0/perMinute2; // period in milliseconds
  enableTimer3 = true;
  perMinute3 = 360;                 // (6 Hz, default)
  timerDelay3 = 30000.0/perMinute3; // period in milliseconds
  enableTimer4 = true;
  perMinute4 = 360;                 // (6 Hz, default)
  timerDelay4 = 30000.0/perMinute4; // period in milliseconds
  ledState1 = HIGH;
  ledState2 = HIGH;
  ledState3 = HIGH;
  ledState4 = HIGH;
  lastTimer1 = 0;
  lastTimer2 = 0;
  lastTimer3 = 0;
  lastTimer4 = 0;
  SOS = false;

  // Set initial value for this characteristic
  enableTimer1Char.writeValue(enableTimer1);
  perMinute1Char.writeValue(perMinute1);
  OnOffState1Char.writeValue(true);
  enableTimer2Char.writeValue(enableTimer2);
  perMinute2Char.writeValue(perMinute2);
  OnOffState2Char.writeValue(true);
  enableTimer3Char.writeValue(enableTimer3);
  perMinute3Char.writeValue(perMinute3);
  OnOffState3Char.writeValue(true);
  enableTimer4Char.writeValue(enableTimer4);
  perMinute4Char.writeValue(perMinute4);
  OnOffState4Char.writeValue(true);
  TemperatureChar.writeValue(-999.9);
  SOSChar.writeValue(false);
}

void loop()
{
  BLEDevice central = BLE.central();

  // Take new measurements and report every {delayTime/ms} while the central is connected:
  //while (central.connected())
  if (! SOS)
  {
    bool didToggle = false;
    long currentTime = millis();
    if (enableTimer1)
    {
      if ( (currentTime - lastTimer1) >= timerDelay1)
      {
        if (ledState1 == LOW)
        {
          ledState1 = HIGH;
          Serial.println("High1");
        }
        else
        {
          ledState1 = LOW;
          Serial.println("Low1");
        }
        digitalWrite(out1, ledState1);
        lastTimer1 = currentTime;
        didToggle = true;
      }
    }
    if (enableTimer2)
    {
      if ( (currentTime - lastTimer2) >= timerDelay2)
      {
        if (ledState2 == LOW)
        {
          ledState2 = HIGH;
          Serial.println("High2");
        }
        else
        {
          ledState2 = LOW;
          Serial.println("Low2");
        }
        digitalWrite(out2, ledState2);
        lastTimer2 = currentTime;
        didToggle = true;
      }
    }
    if (enableTimer3)
    {
      if ( (currentTime - lastTimer3) >= timerDelay3)
      {
        if (ledState3 == LOW)
        {
          ledState3 = HIGH;
          Serial.println("High3");
        }
        else
        {
          ledState3 = LOW;
          Serial.println("Low3");
        }
        digitalWrite(out3, ledState3);
        lastTimer3 = currentTime;
        didToggle = true;
      }
    }
    if (enableTimer4)
    {
      if ( (currentTime - lastTimer4) >= timerDelay4)
      {
        if (ledState4 == LOW)
        {
          ledState4 = HIGH;
          Serial.println("High4");
        }
        else
        {
          ledState4 = LOW;
          Serial.println("Low4");
        }
        digitalWrite(out4, ledState4);
        lastTimer4 = currentTime;
        didToggle = true;
      }
    }
    if (didToggle)
    {
      getTemperature(currentTime);
    }
  }
  else
  {
    doSOS();
  }
}

void getTemperature(long currentTime)
{
  // Read temperature data
  float temperature = 0.0;
  if (IMU.temperatureAvailable())
  {
    IMU.readTemperature(temperature);
  }
  TemperatureChar.writeValue(temperature);

  if (Serial)
  {
    String writeStr = String(temperature,3) + "\t";
    // Blink status
    writeStr += String(enableTimer1) + "\t" + String(enableTimer2) + "\t";
    // Blink rate
    writeStr += String(perMinute1) + "\t" + String(perMinute2) + "\t";
    // LED states
    writeStr += String(ledState1) + "\t" + String(ledState2) + "\t";
    // SOS
    writeStr += String(SOS)+ "\t";
  
    writeStr = String(currentTime,6) + "\t" + writeStr;
    Serial.println(writeStr);
  }
}

void toggleBlinking1(BLEDevice central, BLECharacteristic characteristic)
{
  enableTimer1 = enableTimer1Char.value();
  digitalWrite(out1, HIGH); // If not blinking, default to constant on
}

void toggleBlinking2(BLEDevice central, BLECharacteristic characteristic)
{
  enableTimer2 = enableTimer2Char.value();
  digitalWrite(out2, HIGH); // If not blinking, default to constant on
}

void toggleBlinking3(BLEDevice central, BLECharacteristic characteristic)
{
  enableTimer3 = enableTimer3Char.value();
  digitalWrite(out3, HIGH); // If not blinking, default to constant on
}

void toggleBlinking4(BLEDevice central, BLECharacteristic characteristic)
{
  enableTimer4 = enableTimer4Char.value();
  digitalWrite(out4, HIGH); // If not blinking, default to constant on
}
void updateBlinkRate1(BLEDevice central, BLECharacteristic characteristic)
{
  short rate1 = perMinute1Char.value();
  if (rate1 > 0)
  {
    perMinute1 = rate1;
    timerDelay1 = 30000.0/perMinute1; // period in milliseconds
    enableTimer1 = true; // whenever updating timer value, enable timmer (turn on)
    Serial.println(String(rate1) + "\t" + String(timerDelay1));
  }
}

void updateBlinkRate2(BLEDevice central, BLECharacteristic characteristic)
{
  int rate2 = perMinute2Char.value();
  if (rate2 > 0)
  {
    perMinute2 = rate2;
    timerDelay2 = 30000.0/perMinute2; // period in milliseconds
    enableTimer2 = true; // whenever updating timer value, enable timmer (turn on)
    Serial.println(String(rate2) + "\t" + String(timerDelay2));
  }
}

void updateBlinkRate3(BLEDevice central, BLECharacteristic characteristic)
{
  int rate3 = perMinute3Char.value();
  if (rate3 > 0)
  {
    perMinute3 = rate3;
    timerDelay3 = 30000.0/perMinute3; // period in milliseconds
    enableTimer3 = true; // whenever updating timer value, enable timmer (turn on)
    Serial.println(String(rate3) + "\t" + String(timerDelay3));
  }
}

void updateBlinkRate4(BLEDevice central, BLECharacteristic characteristic)
{
  int rate4 = perMinute4Char.value();
  if (rate4 > 0)
  {
    perMinute4 = rate4;
    timerDelay4 = 30000.0/perMinute4; // period in milliseconds
    enableTimer4 = true; // whenever updating timer value, enable timmer (turn on)
    Serial.println(String(rate4) + "\t" + String(timerDelay4));
  }
}

void OnOff1(BLEDevice central, BLECharacteristic characteristic)
{
  enableTimer1 = false; // whenever turn off setting is actuated, stop timmer (either on or off)
  bool state = OnOffState1Char.value();
  SOS = false;
  if (state)
  {
    ledState1 = LOW;
  }
  else
  {
    ledState1 = HIGH;
  }
  digitalWrite(out1, ledState1);
}

void OnOff2(BLEDevice central, BLECharacteristic characteristic)
{
  bool state = OnOffState2Char.value();
  enableTimer2 = false; // whenever turn off setting is actuated, stop timmer (either on or off)
  SOS = false;
  if (state)
  {
    ledState2 = LOW;
  }
  else
  {
    ledState2 = HIGH;
  }
  digitalWrite(out2, ledState2);
}

void OnOff3(BLEDevice central, BLECharacteristic characteristic)
{
  enableTimer3 = false; // whenever turn off setting is actuated, stop timmer (either on or off)
  bool state = OnOffState3Char.value();
  SOS = false;
  if (state)
  {
    ledState3 = LOW;
  }
  else
  {
    ledState3 = HIGH;
  }
  digitalWrite(out3, ledState3);
}

void OnOff4(BLEDevice central, BLECharacteristic characteristic)
{
  enableTimer4 = false; // whenever turn off setting is actuated, stop timmer (either on or off)
  bool state = OnOffState4Char.value();
  SOS = false;
  if (state)
  {
    ledState4 = LOW;
  }
  else
  {
    ledState4 = HIGH;
  }
  digitalWrite(out4, ledState4);
}

void setSOS(BLEDevice central, BLECharacteristic characteristic)
{
  bool changed = !(SOS & SOSChar.value());
  if (changed)
  {
    SOS = SOSChar.value();
    if (SOS)
    {
      doSOS();
    }
    else
    {
      reset();
    }
  }
}

void doSOS()
{
  // First blank out lights for 2 sec
  digitalWrite(out1, HIGH);
  digitalWrite(out2, HIGH);
  digitalWrite(out3, HIGH);
  digitalWrite(out4, HIGH);
  delay(1800);

  // ... (S)
  dot(3);
  delay(200);

  // --- (O)
  dash(3);
  delay(200);

  // ... (S)
  dot(3);
}
void dot(int count)
{
  for (int i = 0; i < count; ++i)
  {
    digitalWrite(out1, LOW);
    digitalWrite(out2, LOW);
    digitalWrite(out3, LOW);
    digitalWrite(out4, LOW);
    delay(100);
    digitalWrite(out1, HIGH);
    digitalWrite(out2, HIGH);
    digitalWrite(out3, HIGH);
    digitalWrite(out4, HIGH);
    delay(100);
    Serial.print(".");
  }
  Serial.println("");
}

void dash(int count)
{
  for (int i = 0; i < count; ++i)
  {
    digitalWrite(out1, LOW);
    digitalWrite(out2, LOW);
    digitalWrite(out3, LOW);
    digitalWrite(out4, LOW);
    delay(400);
    digitalWrite(out1, HIGH);
    digitalWrite(out2, HIGH);
    digitalWrite(out3, HIGH);
    digitalWrite(out4, HIGH);
    delay(400);
    Serial.print("-");
  }
  Serial.println("");
}
