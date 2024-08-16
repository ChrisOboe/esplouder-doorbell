#include "ArduinoHA.h"
#include "SPIFFS.h"
#include "esp_system.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <AudioFileSourceSPIFFS.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>
#include <RemoteDebug.h>
#include <SD.h>
#include <WiFi.h>
#include <Wire.h>
#include <tas5805m.hpp>

RemoteDebug Debug;

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
      Debug.println("Stopping audio");
      wav->stop();
      file->close();
    }
  }
}

void wifiLoop() {
  Debug.handle();
  debugHandle();

  if (WiFi.status() == WL_CONNECTED) {
    return;
  } else {
    Debug.printf("WiFi.status: %i", WiFi.status());
    WiFi.reconnect();
    WiFi.waitForConnectResult();
  }
};

void otaInit() {

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
          type = "sketch";
        } else { // U_SPIFFS
          type = "filesystem";
        }
        SPIFFS.end();
        Debug.println("Start updating");
      })
      .onEnd([]() { Debug.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Debug.println("Updating");
      })
      .onError([](ota_error_t error) {
        Debug.println("OTA Error");
        if (error == OTA_AUTH_ERROR)
          Debug.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Debug.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Debug.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Debug.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Debug.println("End Failed");
      });

  ArduinoOTA.begin();
}

void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("doorbell");
  WiFi.scanNetworks();
  WiFi.begin("gob", "pfefferpapageiknotenhaken");
  WiFi.waitForConnectResult();
  Debug.begin("doorbell");
}

void audioPlay() {
  if (!file->isOpen()) {
    file->open("/bell.wav");
  }
  Debug.println("Starting audio");
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
    Debug.println("19 toggled");
  }

  if (stateB != last23) {
    Debug.println("23 toggled");
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
    audioPlay();
  }
};

void mqttInit() {
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  device.setUniqueId(baseMac, sizeof(baseMac));
  device.setManufacturer("ChrisOboe");
  device.setName("Doorbell");
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
  if (!out->SetPinout(PIN_I2S_SCK, PIN_I2S_FS, PIN_I2S_SD)) {
    Debug.println("Failed to set pinout");
  }

  Tas5805m.begin();
  wav = new AudioGeneratorWAV();
  file = new AudioFileSourceSPIFFS("/bell.wav");
  out->SetGain(0.7);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  wifiInit();
  Debug.println("Starting up...");

  if (!SPIFFS.begin()) {
    Debug.println("An Error has occurred while mounting SPIFFS");
  }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  audioInit();
  buttonInit();

  mqttInit();
  otaInit();
}

void loop() {
  wifiLoop();
  mqtt.loop();
  audioLoop();
  buttonLoop();
  ArduinoOTA.handle();
}
