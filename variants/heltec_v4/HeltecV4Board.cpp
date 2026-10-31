#include "HeltecV4Board.h"

#include <math.h>

void HeltecV4Board::begin() {
    ESP32Board::begin();


    pinMode(PIN_ADC_CTRL, OUTPUT);
    digitalWrite(PIN_ADC_CTRL, LOW); // Initially inactive

    loRaFEMControl.init();

    periph_power.begin();
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_DEEPSLEEP) {
      long wakeup_source = esp_sleep_get_ext1_wakeup_status();
      if (wakeup_source & (1 << P_LORA_DIO_1)) {  // received a LoRa packet (while in deep sleep)
        startup_reason = BD_STARTUP_RX_PACKET;
    }

      rtc_gpio_hold_dis((gpio_num_t)P_LORA_NSS);
      rtc_gpio_deinit((gpio_num_t)P_LORA_DIO_1);
    }
  }

  void HeltecV4Board::onBeforeTransmit(void) {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED on
    loRaFEMControl.setTxModeEnable();
  }

  void HeltecV4Board::onAfterTransmit(void) {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED off
    loRaFEMControl.setRxModeEnable();
  }

  void HeltecV4Board::powerOff() {
    // Turn off PA
    digitalWrite(P_LORA_PA_POWER, LOW);
    rtc_gpio_hold_en((gpio_num_t)P_LORA_PA_POWER);

    ESP32Board::powerOff();
  }

  uint16_t HeltecV4Board::getBattMilliVolts()  {
    analogReadResolution(10);
    digitalWrite(PIN_ADC_CTRL, HIGH);
    delay(10);
    uint32_t raw = 0;
    for (int i = 0; i < 8; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / 8;

    digitalWrite(PIN_ADC_CTRL, LOW);

    return (adc_mult * (3.3 / 1024.0) * raw) * 1000;
  }

  const char* HeltecV4Board::getManufacturerName() const {
#ifdef HELTEC_LORA_V4_TFT
    return loRaFEMControl.getFEMType() == KCT8103L_PA ? "Heltec V4.3 TFT" : "Heltec V4 TFT";
#else
    return loRaFEMControl.getFEMType() == KCT8103L_PA ? "Heltec V4.3 OLED" : "Heltec V4 OLED";
#endif
  }

  bool HeltecV4Board::setLoRaFemLnaEnabled(bool enable) {
    if (!loRaFEMControl.isLnaCanControl()) {
      return false;
    }

    loRaFEMControl.setLNAEnable(enable);
    loRaFEMControl.setRxModeEnable();
    return true;
  }

  bool HeltecV4Board::isLoRaFemLnaEnabled() const {
    return loRaFEMControl.isLNAEnabled();
  }

void HeltecV4Board::attachDynamicPrefs(KeyValueStore* prefs) {
  _prefs = prefs;

  char adc_mult_str[16] = { 0 };
  if (_prefs->getByKey("adc_mult", adc_mult_str, sizeof(adc_mult_str) - 1)) {
      char* end = nullptr;
      float multiplier = strtof(adc_mult_str, &end);

      if (end != adc_mult_str && *end == '\0' &&
          isfinite(multiplier) && multiplier >= 0.0f && multiplier <= 10.0f) {
          setAdcMultiplier(multiplier);
      }
  }

  char radio_fem_rxgain[8] = { 0 };
  _prefs->getByKey("fem_rxgain", radio_fem_rxgain, 7);  // get initial values

  setLoRaFemLnaEnabled(strcmp(radio_fem_rxgain, "1") == 0);
}

bool HeltecV4Board::handleCommand(const char* command, uint32_t sender_timestamp, char* reply) {
  if (strcmp(command, "get adc.multiplier") == 0) {
    sprintf(reply, "> %.4f", getAdcMultiplier());
    return true;
  }

  if (memcmp(command, "set adc.multiplier ", 19) == 0) {
    const char* value_str = command + 19;
    char* end = nullptr;
    float multiplier = strtof(value_str, &end);
    if (end == value_str || *end != '\0' ||
        !isfinite(multiplier) || multiplier < 0.0f || multiplier > 10.0f) {
      strcpy(reply, "Error: multiplier must be 0-10");
      return true;
    }
    if (_prefs == nullptr) {
      strcpy(reply, "Error: preferences unavailable");
      return true;
    }
    float previous = getAdcMultiplier();
    if (!setAdcMultiplier(multiplier)) {
      strcpy(reply, "Error: unsupported");
      return true;
    }
    char saved_value[16];
    snprintf(saved_value, sizeof(saved_value), "%.4f", multiplier);
    if (!_prefs->setByKey("adc_mult", saved_value)) {
      setAdcMultiplier(previous);
      strcpy(reply, "Error: failed to save multiplier");
      return true;
    }
    if (multiplier == 0.0f) {
      strcpy(reply, "OK - using default board multiplier");
    } else {
      sprintf(reply, "OK - multiplier set to %.4f", multiplier);
    }
    return true;
  }

  if (strcmp(command, "get pwrmgt.bootreason") == 0) {
    const char* reason_str = "unknown";
    switch (esp_reset_reason()) {
      case ESP_RST_UNKNOWN:
        reason_str = "unknown";
        break;
      case ESP_RST_POWERON:
        reason_str = "power-on";
        break;
      case ESP_RST_EXT:
        reason_str = "external reset";
        break;
      case ESP_RST_SW:
        reason_str = "software reset";
        break;
      case ESP_RST_PANIC:
        reason_str = "panic";
        break;
      case ESP_RST_INT_WDT:
        reason_str = "interrupt watchdog";
        break;
      case ESP_RST_TASK_WDT:
        reason_str = "task watchdog";
        break;
      case ESP_RST_WDT:
        reason_str = "watchdog";
        break;
      case ESP_RST_DEEPSLEEP:
        reason_str = "deep sleep wakeup";
        break;
      case ESP_RST_BROWNOUT:
        reason_str = "brownout";
        break;
      case ESP_RST_SDIO:
        reason_str = "SDIO reset";
        break;
      default:
        reason_str = "other";
        break;
    }
    sprintf(reply, "> %s", reason_str);
    return true;
  }

  if (strcmp(command, "get radio.fem.rxgain") == 0) {
    if (!loRaFEMControl.isLnaCanControl()) {
      strcpy(reply, "Error: unsupported");
    } else {
      sprintf(reply, "> %s", isLoRaFemLnaEnabled() ? "on" : "off");
    }
    return true;
  }
  if (memcmp(command, "set radio.fem.rxgain ", 21) == 0) {
    if (!loRaFEMControl.isLnaCanControl()) {
      strcpy(reply, "Error: unsupported");
    } else if (memcmp(&command[21], "on", 2) == 0) {
      if (setLoRaFemLnaEnabled(true)) {
        _prefs->setByKey("fem_rxgain", "1");
        strcpy(reply, "OK - LoRa FEM RX gain on");
      } else {
        strcpy(reply, "Error: failed to apply LoRa FEM RX gain");
      }
    } else if (memcmp(&command[21], "off", 3) == 0) {
      if (setLoRaFemLnaEnabled(false)) {
        _prefs->setByKey("fem_rxgain", "0");
        strcpy(reply, "OK - LoRa FEM RX gain off");
      } else {
        strcpy(reply, "Error: failed to apply LoRa FEM RX gain");
      }
    } else {
      strcpy(reply, "Error: state must be on or off");
    }
    return true;
  }

  return false; // not handled
}
