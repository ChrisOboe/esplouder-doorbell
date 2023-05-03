#include "ArduinoHA.h"
#include "SPIFFS.h"
#include "esp_system.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <AudioFileSourceSPIFFS.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>
#include <SD.h>
#include <TelnetStream.h>
#include <WiFi.h>
#include <Wire.h>
#include <tas5805m.hpp>

tas5805m Tas5805m(&Wire);

AudioOutputI2S *out;
AudioFileSourceSPIFFS *file;
AudioGeneratorWAV *wav;

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HABinarySensor sensor("doorbell");
HAButton button("ring");
HANumber loudness("loudness");

bool buttonPressed = false;
int last19 = 0;
int last23 = 0;

void audioLoop() {
  if (wav->isRunning()) {
    if (!wav->loop()) {
      wav->stop();
    }
  }
}

void wifiLoop() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  WiFi.scanNetworks();
  WiFi.begin("gob", "pfefferpapageiknotenhaken");

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    TelnetStream.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
};

void otaInit() {

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        TelnetStream.println("Start updating " + type);
      })
      .onEnd([]() { TelnetStream.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        TelnetStream.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        TelnetStream.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          TelnetStream.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          TelnetStream.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          TelnetStream.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          TelnetStream.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          TelnetStream.println("End Failed");
      });

  ArduinoOTA.begin();
}

void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("doorbell-beta");
  WiFi.disconnect();
  delay(100);
  wifiLoop();
}

void audioPlay() {
  if (!file->isOpen()) {
    file->open("/bell.wav");
  }
  wav->begin(file, out);
}

void onButtonCommand(HAButton *sender) { audioPlay(); }
void onLoudnessChange(HANumeric number, HANumber *sender) {
  out->SetGain(float(number.toInt8()) / 100.0);
};

void buttonInit() {
  pinMode(19, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
}
void buttonLoop() {
  int stateA = digitalRead(19);
  int stateB = digitalRead(23);

  if (stateA != last19) {
    TelnetStream.println("19 toggled");
  }

  if (stateB != last23) {
    TelnetStream.println("23 toggled");
  }

  last19 = stateA;
  last23 = stateB;

  bool lastButtonState = buttonPressed;
  if (stateA == 0 && stateB == 1) {
    buttonPressed = true;
    sensor.setState(true);
  } else {
    buttonPressed = false;
    sensor.setState(false);
  }

  if (lastButtonState == false && buttonPressed == true) {
    TelnetStream.println("Button pressed!");
    // audioPlay();
  }
};

void mqttInit() {
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  device.setUniqueId(baseMac, sizeof(baseMac));
  device.setManufacturer("ChrisOboe");
  device.setName("Doorbell-Beta");
  device.setSoftwareVersion("0.0.1");
  device.setModel("ESP Louder");
  device.enableSharedAvailability();
  device.enableLastWill();

  sensor.setCurrentState(false);
  sensor.setName("Door bell");

  button.setIcon("mdi:siren");
  button.setName("Ring Bell");
  button.onCommand(onButtonCommand);

  loudness.setIcon("mdi:volume-medium");
  loudness.setName("Volume");

  loudness.setMin(0);
  loudness.setMax(100);
  loudness.setCurrentState(70);
  loudness.setUnitOfMeasurement("%");
  loudness.setStep(10);
  loudness.setMode(HANumber::Mode::ModeSlider);
  loudness.setOptimistic(true);
  loudness.onCommand(onLoudnessChange);

  mqtt.begin("chump.gob.zone");
};

void audioInit() {
  Tas5805m.init();
  out = new AudioOutputI2S();
  TelnetStream.printf("Setting I2S pins: clk = %d, ws = %d, data = %d\n",
                      PIN_I2S_SCK, PIN_I2S_FS, PIN_I2S_SD);
  if (!out->SetPinout(PIN_I2S_SCK, PIN_I2S_FS, PIN_I2S_SD)) {
    TelnetStream.println("Failed to set pinout");
  }

  Tas5805m.begin();
  wav = new AudioGeneratorWAV();
  file = new AudioFileSourceSPIFFS("/bell.wav");
  out->SetGain(0.7);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Joho");
  wifiInit();
  Serial.println("Wifi Done!");
  TelnetStream.begin();

  TelnetStream.println(F("Starting up...\n"));

  if (!SPIFFS.begin()) {
    TelnetStream.println(F("An Error has occurred while mounting SPIFFS"));
    while (true) {
    };
  } else
    TelnetStream.println(F("SPIFFS mounted"));

  // Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // audioInit();
  buttonInit();

  mqttInit();
  // otaInit();
}

void loop() {
  wifiLoop();
  mqtt.loop();
  // audioLoop();
  buttonLoop();
  // ArduinoOTA.handle();
}
