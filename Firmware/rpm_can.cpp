#include <Arduino.h>
#include "driver/twai.h"
#include "driver/pcnt.h"
#include "esp_err.h"

#define CAN_TX_GPIO GPIO_NUM_21
#define CAN_RX_GPIO GPIO_NUM_20

#define HALL_SENSOR GPIO_NUM_3 //Digital output of hall sensor

#define MOTOR_NUM_POLE_PAIRS 3.0 //6 poles

hw_timer_t *timer = NULL;
volatile bool readyToCount = false;

int16_t rpm = 0;

// Timer ISR
void IRAM_ATTR onTimer() {
  readyToCount = true;
}

void readRPM() {
  if (readyToCount) {
    int16_t erpm = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &erpm);
    
    Serial.print("Pulses in last second: ");
    Serial.println(erpm);

    erpm *= 60;
    rpm = (int16_t)(erpm / MOTOR_NUM_POLE_PAIRS);
    
    // Reset counter
    pcnt_counter_clear(PCNT_UNIT_0);
    
    readyToCount = false;
  }
  delay(10);
}

void sendCANMessage(uint32_t id, uint8_t *data, uint8_t length)
{
  twai_message_t message;
  message.extd = 1;         // Standard vs extended format
  message.rtr = 0;          // Data vs RTR frame
  message.ss = 0;           // Whether the message is single shot (i.e., does not repeat on error)
  message.self = 0;         // Whether the message is a self reception request (loopback)
  message.dlc_non_comp = 0; // DLC is less than 8
  message.identifier = id;
  message.data_length_code = length;

  memcpy(message.data, data, length);

  esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(1000));
  if (err == ESP_OK)
  {
    Serial.println("Message queued for transmission");
  }
  else
  {
    Serial.print("Failed to queue message for transmission: ");
    Serial.println(esp_err_to_name(err));
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing PCNT for ERPM calculation");
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = HALL_SENSOR,       // Pulse input pin
    .ctrl_gpio_num = PCNT_PIN_NOT_USED, // Not using control pin
    .lctrl_mode = PCNT_MODE_KEEP,       // Not using control pin
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,         // Count on rising edge
    .neg_mode = PCNT_COUNT_INC,         // Count on falling edge
    .counter_h_lim = 50000,
    .counter_l_lim = 0,
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0,
  };
  ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

  Serial.println("Initializing 1 second timer for PCNT");

  //Every second, clear the pcnt counter value and start over
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true); //1 second, periodic
  timerAlarmEnable(timer);

  Serial.println("Initializing TWAI...");

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  esp_err_t res = twai_driver_install(&g_config, &t_config, &f_config);
  if (res == ESP_OK)
  {
    Serial.println("TWAI driver installed successfully");
  }
  else
  {
    Serial.print("Failed to install TWAI driver. Error code: ");
    Serial.println(res);
    return;
  }

  // Start TWAI driver
  res = twai_start();
  if (res == ESP_OK)
  {
    Serial.println("TWAI driver started successfully");
  }
  else
  {
    Serial.print("Failed to start TWAI driver. Error code: ");
    Serial.println(res);
    return;
  }

  // Double-check driver status
  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK)
  {
    Serial.print("TWAI State: ");
    Serial.println(status.state);
    if (status.state != TWAI_STATE_RUNNING)
    {
      Serial.println("WARNING: TWAI driver is not in running state!");
    }
  }

  Serial.println("Setup completed");
  Serial.flush();
}

void loop()
{
  delay(100);
  Serial.print("Sending packet... ");
  uint8_t val = (uint8_t)rpm;

  // Double-check driver status
  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK)
  {
    Serial.print("TWAI State: ");
    Serial.println(status.state);
    if (status.state != TWAI_STATE_RUNNING)
    {
      Serial.println("WARNING: TWAI driver is not in running state!");
    }
    if (status.state == TWAI_STATE_BUS_OFF)
    {
      Serial.println("Bus off state detected, attempting recovery...");
      twai_stop();
      delay(100);
      twai_initiate_recovery();
      delay(100);
      esp_err_t res = twai_start();
      if (res == ESP_OK)
      {
        Serial.println("Recovery successful");
      }
      else
      {
        Serial.print("Recovery failed: ");
        Serial.println(esp_err_to_name(res));
      }
    }
    Serial.println("\n");
  }

  readRPM();
  sendCANMessage(0x0012, &val, sizeof(val));
}