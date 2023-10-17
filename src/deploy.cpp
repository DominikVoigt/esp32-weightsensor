/*
   Code skeleton for weight measurements acquired from:
      https://github.com/olkal/HX711_ADC/blob/master/examples/Read_1x_load_cell_interrupt_driven/Read_1x_load_cell_interrupt_driven.ino
   WiFi and MQTT code skeleton acquired from:
      https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/

*/

#include <HX711_ADC.h>
#include <EEPROM.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <string>
#include <WiFi.h>
#include <PubSubClient.h>

const int HX711_dout = 4; // mcu > HX711 dout pin, must be external interrupt capable!
const int HX711_sck = 16; // mcu > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
float calibrationValue = 743.72;
const int calVal_eepromAdress = 0;
bool useEEPROM = false;

unsigned long t = 0;
const int serialPrintInterval = 100;
volatile boolean newDataReady;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
bool enableBootDisplay = false;

// Replace the next variables with your SSID/Password combination
const char *ssid = "Cocktail_Mixer";
const char *password = "process_hubby";
//const char *ssid = "FRITZ!Box 7530 BS";
//const char *password = "09324416513504437202";

IPAddress server(192, 168, 0, 118);
const int port = 1900;
const std::string sensorID = "1111";

// Weight displayed on the display
int displayedWeight = 0;
// Weight Measured in the previous iteration
int lastWeight = 0;
bool firstMeasurement = true;

WiFiClient espClient;
PubSubClient client(espClient);

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client"))
    {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// interrupt routine:
void dataReadyISR()
{
  if (LoadCell.update())
  {
    newDataReady = 1;
  }
}

void setupLoadcell()
{
  if (useEEPROM)
  {
    EEPROM.begin(512);
    EEPROM.get(calVal_eepromAdress, calibrationValue);
  }
  Serial.print("Using Calibration value:");
  Serial.println(calibrationValue);

  LoadCell.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check wiring to HX711 and pin designations");
    while (true)
      ;
  }
  else
  {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Startup is complete");
  }
  attachInterrupt(digitalPinToInterrupt(HX711_dout), dataReadyISR, FALLING);
}

void setupDisplay()
{
  u8g2.begin();
}

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *message, unsigned int length)
{
  // For now we do not care about any topics, so we do not implement it
}

void setupMQTT()
{
  setup_wifi();
  client.setServer(server, 1900);
  client.setCallback(callback);
}

void displayBoot()
{
  u8g2.setCursor(20, 45);
  u8g2.setFont(u8g2_font_ncenB24_tr);
  u8g2.print("TUM");
  u8g2.sendBuffer();
  sleep(1);
}

void setup()
{
  Serial.begin(9600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  setupLoadcell();
  setupDisplay();
  if (enableBootDisplay)
  {
    displayBoot();
  }
  setupMQTT();
}

void displaySensorIDBottom()
{
  u8g2.setCursor(50, 60);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.print("Sensor ID: ");
  u8g2.print(sensorID.c_str());
}

void displayWeight()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.setCursor(0, 15);
  u8g2.print("Current Weight:");
  u8g2.setCursor(0, 40);
  u8g2.print(displayedWeight);
  u8g2.print(" grams");
  displaySensorIDBottom();
  u8g2.sendBuffer();
}

int gearState = 0;
void displayMeasuring()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.setCursor(0, 15);
  u8g2.print("Measuring");
  u8g2.setCursor(0, 40);
  u8g2.print("Weight  ");

  switch (gearState)
  {
  case 0:
    u8g2.print("/");
    break;
  case 1:
    u8g2.print("-");
    break;
  case 2:
    u8g2.print("\\");
    break;
  }
  gearState = ++gearState % 3;
  displaySensorIDBottom();
  u8g2.sendBuffer();
}

void publishWeightToMQTT()
{
  std::string topic = "cocktail/weight/sensor_";
  topic = topic + sensorID;
  client.publish(topic.c_str(), reinterpret_cast<uint8_t *>(&displayedWeight), sizeof(float));
}

void handleNewWeightData()
{
  newDataReady = 0;
  // Cutoff milligrams as the weighing setup is not that precise
  int newWeight = static_cast<int>(LoadCell.getData());
  // The delta in weight is not large, do not update the display (could be due to noise)
  bool smallDelta = abs(newWeight - displayedWeight) <= 2;
  // The delta in weight is pretty large and thus the weight measurement has not yet stabilized, do not update the display
  bool largeDelta = abs(newWeight - lastWeight) >= 3;
  if (!firstMeasurement)
  {
    if (largeDelta)
    {
      displayMeasuring();
    }
    if (smallDelta || largeDelta)
    {
      lastWeight = newWeight;
      return;
    }
  }
  else
  {
    firstMeasurement = false;
    lastWeight = newWeight;
  }
  if (newWeight <= 2)
  {
    // for very small values or negative values just display 0 as the precision of the scale is not that great
    newWeight = 0;
  }
  displayedWeight = newWeight;
  publishWeightToMQTT();
  displayWeight();
  Serial.print("Load_cell output val: ");
  Serial.print("New weight:");
  Serial.println(newWeight);
  Serial.println(displayedWeight);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // get smoothed value from the dataset:
  if (newDataReady)
  {
    if (millis() > t + serialPrintInterval)
    {
      handleNewWeightData();
      t = millis();
    }
  }

  if (Serial.available() > 0)
  {
    char inByte = Serial.read();
    if (inByte == 't')
      LoadCell.tareNoDelay();
  }

  if (LoadCell.getTareStatus())
  {
    Serial.println("Tare complete");
  }
}