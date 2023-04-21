#include "ArduinoHA.h"
#include "SPIFFS.h"
#include "esp_system.h"
#include <Arduino.h>
#include <AudioFileSourceSPIFFS.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>
#include <SD.h>
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

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  };
};

void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(esp32 - doorbell);
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

void buttonInit() { pinMode(2, INPUT_PULLUP); }
void buttonLoop() {
  int state = digitalRead(2);
  bool lastButtonState = buttonPressed;
  if (state == 0) {
    buttonPressed = true;
    sensor.setState(true);
  } else {
    buttonPressed = false;
    sensor.setState(false);
  }

  if (lastButtonState == false && buttonPressed == true) {
    Serial.println("Button pressed!");
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

  sensor.setCurrentState(false);
  sensor.setName("Door bell");

  button.setIcon("mdi:siren");
  button.setName("Ring Bell");
  button.onCommand(onButtonCommand);

  loudness.setIcon("mdi:volume-medium");
  loudness.setName("Volume");

  loudness.setMin(0);
  loudness.setMax(100);
  loudness.setCurrentState(100);
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
  Serial.printf("Setting I2S pins: clk = %d, ws = %d, data = %d\n", PIN_I2S_SCK,
                PIN_I2S_FS, PIN_I2S_SD);
  if (!out->SetPinout(PIN_I2S_SCK, PIN_I2S_FS, PIN_I2S_SD)) {
    Serial.println("Failed to set pinout");
  }

  Tas5805m.begin();
  wav = new AudioGeneratorWAV();
  file = new AudioFileSourceSPIFFS("/bell.wav");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("Starting up...\n"));

  if (!SPIFFS.begin()) {
    Serial.println(F("An Error has occurred while mounting SPIFFS"));
    while (true) {
    };
  } else
    Serial.println(F("SPIFFS mounted"));

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  audioInit();
  buttonInit();
  wifiInit();
  mqttInit();
}

void loop() {
  wifiLoop();
  mqtt.loop();
  audioLoop();
  buttonLoop();
}
