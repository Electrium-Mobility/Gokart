#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <driver/twai.h>
#include "esp_err.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define THUMB_LEFT_GPIO 2
#define THUMB_RIGHT_GPIO 3

#define SHIFT_UP_GPIO 8
#define SHIFT_DOWN_GPIO 4

#define TX_GPIO GPIO_NUM_21
#define RX_GPIO GPIO_NUM_20

#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int speed = 25;
int pageIndex = 1;
int gear = 1;

bool thumbLeftPressed = false;
bool thumbRightPressed = false;

bool shiftUpPressed = false;
bool shiftDownPressed = false;

void displayToScreen(int index)
{
    //Display numerical value
    display.setCursor(42, 20);
    display.setTextColor(WHITE);
    display.setTextSize(3);
    display.printf("%d", speed);
  
    //Display unit
    display.setCursor(80, 20);
    display.setTextSize(1);
    display.println("km");
    display.setCursor(80, 30);
    display.println("h");

    display.setCursor(10, 20);
    display.setTextSize(2);
    display.println(gear);

  if (index == 0)
  {
    display.fillTriangle(54, 50, 49, 55, 54, 60, WHITE);
    display.drawCircle(64, 55, 5, WHITE);
    display.drawTriangle(74, 50, 79, 55, 74, 60, WHITE);
  }
  if (index == 1)
  {
    display.drawTriangle(54, 50, 49, 55, 54, 60, WHITE);
    display.fillCircle(64, 55, 5, WHITE);
    display.drawTriangle(74, 50, 79, 55, 74, 60, WHITE);
  }
  if (index == 2)
  {
    display.drawTriangle(54, 50, 49, 55, 54, 60, WHITE);
    display.drawCircle(64, 55, 5, WHITE);
    display.fillTriangle(74, 50, 79, 55, 74, 60, WHITE);
  }
}

void receiveCANMessage()
{
  twai_message_t message;
  esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(1000));
  if (err == ESP_OK) {
      Serial.println("Message received");
      // process message...
  } else {
      Serial.print("Failed to receive message: ");
      Serial.println(esp_err_to_name(err));
      return;
  }

  // Process received message
  if (message.extd)
  {
    printf("Message is in Extended Format\n");
  }
  else
  {
    printf("Message is in Standard Format\n");
  }
  printf("ID is %d\n", message.identifier);
  if (!(message.rtr))
  {
    for (int i = 0; i < message.data_length_code; i++)
    {
      printf("Data byte %d = %d\n", i, message.data[i]);
      speed = message.data[i];
    }
  }
}

void setup()
{
  Serial.begin(9600);
  delay(1000);

  //Initialize CAN
    Serial.println("Initializing TWAI...");

    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO, RX_GPIO, TWAI_MODE_NORMAL);
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

  // Initialize GPIO
  pinMode(THUMB_LEFT_GPIO, INPUT);
  pinMode(THUMB_RIGHT_GPIO, INPUT);
  pinMode(SHIFT_UP_GPIO, INPUT);
  pinMode(SHIFT_DOWN_GPIO, INPUT);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  display.clearDisplay();
  display.setCursor(0, 0);

  Serial.println("Setup completed");
  Serial.flush();

}

void loop()
{
  // Poll every 50ms
  delay(50);

  receiveCANMessage();

  // Check state of pushbuttons
  thumbLeftPressed = (digitalRead(THUMB_LEFT_GPIO) == HIGH);
  thumbRightPressed = (digitalRead(THUMB_RIGHT_GPIO) == HIGH);
  shiftUpPressed = (digitalRead(SHIFT_UP_GPIO) == HIGH);
  shiftDownPressed = (digitalRead(SHIFT_DOWN_GPIO) == HIGH);

  if (thumbLeftPressed && pageIndex > 0)
  {
    --pageIndex;
  }
  if (thumbRightPressed && pageIndex < 2)
  {
    ++pageIndex;
  }
  if (shiftUpPressed && gear < 5) {
    ++gear;
  }
  if (shiftDownPressed && gear > 0) {
    --gear;
  }

  display.clearDisplay();

  displayToScreen(pageIndex);

  display.display();
}