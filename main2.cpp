#include <DFRobot_AXP313A.h>

#include <valadis_tsir-project-6_inferencing.h>
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/edgeimpulse/fomo.h>
#include "DFRobot_AXP313A.h"
#include "DFRobot_AHT20.h"
#include "Adafruit_SHT31.h"
#include <Wire.h>
#include <APDS9930.h>
#include <SPI.h>
#include <LoRa.h>
using eloq::camera;
using eloq::ei::fomo;
float temperature=0;
float humidity=0;
int battery=10;
int id=1;
// Constants
#define PROX_INT_HIGH   800 // Proximity level for interrupt
#define PROX_INT_LOW    20  // No far interrupt
APDS9930 apds = APDS9930();
uint16_t proximity_data = 0;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
#define ss                 18
#define rst                38
#define dio0                9
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     45
#define SIOD_GPIO_NUM     1
#define SIOC_GPIO_NUM     2
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       46
#define Y7_GPIO_NUM       8
#define Y6_GPIO_NUM       7
#define Y5_GPIO_NUM       4
#define Y4_GPIO_NUM       41
#define Y3_GPIO_NUM       40
#define Y2_GPIO_NUM       39
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     42
#define PCLK_GPIO_NUM     5

DFRobot_AXP313A axp;

// Time constants
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  100        /* Time ESP32 will go to sleep (in seconds) */
#define RUN_TIME 10000  /* Time to run the loop (in milliseconds) */

RTC_DATA_ATTR int bootCount = 0;
void read_sht30();
void lora_sent_packet(char *message);
void init_lora();
void init_proximity_sensor();


void init_proximity_sensor()
{

 // Initialize APDS-9930 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9930 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9930 init!"));
  }
  
  // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    Serial.println(F("Something went wrong trying to set PGAIN"));
  }
  
  // Set proximity interrupt thresholds
  if ( !apds.setProximityIntLowThreshold(PROX_INT_LOW) ) {
    Serial.println(F("Error writing low threshold"));
  }
  if ( !apds.setProximityIntHighThreshold(PROX_INT_HIGH) ) {
    Serial.println(F("Error writing high threshold"));
  }
  
  // Start running the APDS-9930 proximity sensor (interrupts)
  if ( apds.enableProximitySensor(true) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
  if ( !apds.clearProximityInt() ) 
  {
     Serial.println("Error clearing interrupt");
  }

}
void init_lora()
{
  LoRa.setPins(ss, rst, dio0);
  LoRa.begin(866E6);
  LoRa.setTxPower(14,0);  

}
void lora_sent_packet(char *message)
{
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
  LoRa.sleep();
  axp.disablePower();
  while(1)
  {
    Serial.println(message);
    delay(500);
  }
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_3, 0);
  esp_deep_sleep_start();
}

void read_sht30()
{
 
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  temperature=t;
  humidity=h;
  Serial.print("Temp *C = "); Serial.print(t); Serial.print("\t\t");
  Serial.print("Hum. % = "); Serial.println(h); 
  delay(500);
  
}

void setup()
{
  delay(3000); 
  Serial.begin(115200);  

  Serial.println("__EDGE IMPULSE FOMO (NO-PSRAM)__");
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) 
  {
    case ESP_SLEEP_WAKEUP_EXT0:    
      sht31.begin(0x45);
      read_sht30();
      init_lora();
      init_proximity_sensor();
      setup_image_recognition();
      
    break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        
    init_proximity_sensor();
    sht31.begin(0x45);
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    Serial.print("Temp *C = "); Serial.print(t); Serial.print("\t\t");
    Serial.print("Hum. % = "); Serial.println(h);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_3, 0);
    apds.clearProximityInt();
    esp_deep_sleep_start();
    
    break;
  }

}
void setup_image_recognition()
{
   Serial.println("__EDGE IMPULSE FOMO (NO-PSRAM)__");
  // Initialize power management and camera
  while(axp.begin() != 0){
    Serial.println("init error");
    delay(1000);
  }
  axp.enableCameraPower(axp.eOV2640);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  // Camera initialization
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // Initial sensor adjustments
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // Flip it back
    s->set_brightness(s, 1); // Increase brightness slightly
    s->set_saturation(s, -2); // Lower saturation
  }
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QQVGA);
  }

  Serial.println("Camera OK");
  Serial.println("Put object in front of camera");
}

bool detect=false;

void loop() {
  unsigned long startTime = millis();  // Record the start time
  unsigned long currentTime;
  while (true) {
    currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;

    // Check if 100 seconds have passed
    if (elapsedTime >= RUN_TIME) {
      break;  // Exit the loop if 100 seconds have passed
    }
    Serial.println(elapsedTime);
    Serial.println("Capturing image...");

    // Capture picture
    if (!camera.capture().isOk()) {
      Serial.println(camera.exception.toString());
      return;
    }

    // Run FOMO model
    Serial.println("Running FOMO model...");
    if (!fomo.run().isOk()) {
      Serial.println(fomo.exception.toString());
      return;
    }

    // How many objects were found?
    Serial.printf("Found3 %d object(s) in %dms", 
      fomo.count(),
      fomo.benchmark.millis()
    );

    // If no object is detected, continue
    if (!fomo.foundAnyObject())
    {
      continue;
    }
    else
    {
      char buffer[10];     
      sprintf(buffer, "%.2f,%.2f,%d,%d,", temperature,humidity,battery,id);
      lora_sent_packet(buffer);
    }
    // If you expect to find a single object, use fomo.first
    Serial.printf(
      "Found1 %s at (x = %d, y = %d) (size %d x %d). Proba is %.2f \r\n",
      fomo.first.label,
      fomo.first.x,
      fomo.first.y,
      fomo.first.width,
      fomo.first.height,
      fomo.first.proba
    );

    // If you expect to find many objects, use fomo.forEach
    if (fomo.count() > 1) {
      fomo.forEach([](int i, bbox_t bbox) {
        Serial.printf(
          "#%d) Found2 %s at (x = %d, y = %d) (size %d x %d). Proba is %.2f \r\n",
          i + 1,
          bbox.label,
          bbox.x,
          bbox.y,
          bbox.width,
          bbox.height,
          bbox.proba
        );
      });
    }
  }

  // After the loop completes, go to deep sleep
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_3, 0);       
  esp_deep_sleep_start();
  Serial.flush();
  // This line will never be executed

}
