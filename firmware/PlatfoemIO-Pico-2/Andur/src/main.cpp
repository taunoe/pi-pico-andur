/*
 * Tauno Erik
 * Hardware:
 *  -Pico 2
 *  -TOF400C/VL53L1X
 * Started: 29.09.2025
 * Edited:  14.12.2025
 * 
 * Links:
 * - https://arduino-pico.readthedocs.io/en/latest/platformio.html
 * - https://randomnerdtutorials.com/raspberry-pi-pico-i2c-scanner-arduino/
 * - https://github.com/pololu/vl53l1x-arduino/
 * - https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library
 * - https://github.com/adafruit/Adafruit_VL53L0X/tree/master
 * 
 * I2C0: SDA GPIOs -> GPIO0, GPIO4, GPIO8, GPIO12, GPIO16, GPIL20
 * I2C0: SCL GPIOs -> GPIO1, GPIO5, GPIO9, GPIO13, GPIO17, GPIO21
 * 
 * I2C1: SDA GPIOs -> GPIO2, GPIO6, GPIO10, GPIO14, GPIO18, GPIO26
 * I2C1: SCL GPIOs -> GPIO3, GPIO7, GPIO11, GPIO15, GPIO19, GPIO27
 * 
 * TODO: Mode Short and Long
 * 
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
/* Sensor */
//#include "SparkFun_VL53L1X.h"  // Uuemale sensorile
#include "Adafruit_VL53L0X.h"  // https://github.com/adafruit/Adafruit_VL53L0X/tree/master
/* Ekraan */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
/* Rotary Encoder */
// https://www.upesy.com/blogs/tutorials/rotary-encoder-raspberry-pi-pico-with-arduino-code#
#include "hardware/pio.h"
#include "quadrature.pio.h"  // https://github.com/jamon/pi-pico-pio-quadrature-encoder/tree/main



/********************************
  Pins
*********************************/
// SDA - GPIO4
// SCL - GPIO5
#define SENSOR_GPIO1_PIN    6  // Used by the sensor to indicate that data is ready
#define SENSOR_XSHUT_PIN    7  // By default it's pulled high. When the pin is pulled low, the sensor goes into shutdown mode.

#define STATUS_GREEN_LED    3
#define STATUS_RED_LED      2
//#define SELECTED_GREEN_LED 16  // Selected distance
//#define SELECTED_RED_LED   17  // Selected distance

#define MIN_RE_CLK_PIN     16
#define MIN_RE_DT_PIN      17
#define MIN_RE_SW_PIN      18
#define MAX_RE_CLK_PIN     19
#define MAX_RE_DT_PIN      20
#define MAX_RE_SW_PIN      21

#define SIGNAL_OUT_1_PIN   22
#define SIGNAL_OUT_2_PIN   15


/********************************
  OLED Screen
*********************************/
#define SCREEN_WIDTH      128  // OLED display width, in pixels
#define SCREEN_HEIGHT      32  // OLED display height, in pixels
#define OLED_RESET         -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS   0x3C  // < See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


/********************************
  Laser sensor
  can handle about 30mm to 1200mm of range distance
*********************************/
#define DISTANCE_HISTORY_SIZE 3
int distance_history[DISTANCE_HISTORY_SIZE] = {0};
uint8_t dis_history_i = 0;

Adafruit_VL53L0X vl53lxx_sensor = Adafruit_VL53L0X();


/********************************
  Global Variables
*********************************/
long avg_distance = 0;
uint distance = 0;
uint selected_value_min = 10;
uint selected_value_max = 100;


#define ENCODER_MIN_VALUE         0
#define ENCODER_MAX_VALUE      4000
#define ENCODER_SMALLEST_RANGE    5

/********************************
  Rotary Encoder MAX
*********************************/
int32_t current_encoder_value_max = selected_value_max;
uint32_t last_pio_count_max = 0;
PIO re_pio_max = pio0;
uint re_offset_max;
uint re_sm_max;


/********************************
  Rotary Encoder MIN
*********************************/
int32_t current_encoder_value_min = selected_value_min;
uint32_t last_pio_count_min = 0;
PIO re_pio_min = pio0;
uint re_offset_min;
uint re_sm_min;


/********************************
  Functions
*********************************/
void i2c_scanner();
void init_LED_pins();
void init_display();
void init_vl53lxx_sensor();


/********************************
  SETUP
*********************************/
void setup() {
  Serial.begin(115200); // 921600
  // Do not change! Serial pordi lugemisega tekivad probleemid!!!
  //Wire.setSDA(2); // default 4
  //Wire.setSCL(3); // default 5
  Wire.begin();
  //Wire.setClock(400000); // use 400 kHz I2C

  delay(3000);
  i2c_scanner();

  init_LED_pins();
  init_display();
  init_vl53lxx_sensor();

  // Rotary encoder 1 MAX
  re_offset_max = pio_add_program(re_pio_max, &quadratureA_program);
  re_sm_max = pio_claim_unused_sm(re_pio_max, true);
  quadratureA_program_init(re_pio_max, re_sm_max, re_offset_max, MAX_RE_CLK_PIN, MAX_RE_DT_PIN);

  // Rotary encoder 2 MIN
  re_offset_min = pio_add_program(re_pio_min, &quadratureA_program);
  re_sm_min = pio_claim_unused_sm(re_pio_min, true);
  quadratureA_program_init(re_pio_min, re_sm_min, re_offset_min, MIN_RE_CLK_PIN, MIN_RE_DT_PIN);
}

/********************************
  Core 0 loop
*********************************/
void loop() {
  // Time
  long start_time = millis();
  // Sensor
  VL53L0X_RangingMeasurementData_t measure;

  vl53lxx_sensor.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  Serial.print("measure.RangeStatus ");
  Serial.print(measure.RangeStatus);
  Serial.print(" D=");
 

  if (measure.RangeStatus != 4) {
    distance = measure.RangeMilliMeter;
    Serial.println(distance);

    distance_history[dis_history_i] = distance;

    if (++dis_history_i == DISTANCE_HISTORY_SIZE) {
      dis_history_i = 0;
    }

    for (int x = 0; x < DISTANCE_HISTORY_SIZE; x++) {
      avg_distance += distance_history[x];
    }

    avg_distance /= DISTANCE_HISTORY_SIZE;
    //Serial.print("\tKeskmine: ");
    //Serial.print(avg_distance);

     //////
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10,0);
  display.print(distance);//avg_distance
  display.println(F(" mm"));

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10,20);
  display.print(F("min:"));
  display.print(selected_value_min);

  //display.setTextSize(1);
  //display.setTextColor(SSD1306_WHITE);
  display.setCursor(50,20);
  display.print(F("  max:"));
  display.print(selected_value_max);

  display.display();
  ///

  }

  delay(100);

  long end_time = millis();

  

 

  /* Rotary Encoders MAX */
  pio_sm_exec_wait_blocking(re_pio_max, re_sm_max, pio_encode_in(pio_x, 32));
  uint32_t new_pio_count_max = pio_sm_get_blocking(re_pio_max, re_sm_max);

  // Calculate the change (delta) since the last reading.
  // Casting to int32_t correctly handles the wrap-around when the
  // PIO counter goes from 0 to 4294967295 and vice-versa.
  int32_t delta_max = (int32_t)new_pio_count_max - (int32_t)last_pio_count_max;
  last_pio_count_max = new_pio_count_max;

  current_encoder_value_max += delta_max;

  // Clamp the value to stay within your defined min/max range
  if (current_encoder_value_max < ENCODER_MIN_VALUE) {
    current_encoder_value_max = ENCODER_MIN_VALUE;
  } else if (current_encoder_value_max > ENCODER_MAX_VALUE) {
    current_encoder_value_max = ENCODER_MAX_VALUE;
  }
    selected_value_max = current_encoder_value_max;

  if (selected_value_max <= selected_value_min + ENCODER_SMALLEST_RANGE)
  {
    selected_value_max = selected_value_min + ENCODER_SMALLEST_RANGE;
  }

    printf("MAX Value: %ld\n", current_encoder_value_max);


  /* MIN */
  pio_sm_exec_wait_blocking(re_pio_min, re_sm_min, pio_encode_in(pio_x, 32));
  uint32_t new_pio_count_min = pio_sm_get_blocking(re_pio_min, re_sm_min);

  // Calculate the change (delta) since the last reading.
  // Casting to int32_t correctly handles the wrap-around when the
  // PIO counter goes from 0 to 4294967295 and vice-versa.
  int32_t delta_min = (int32_t)new_pio_count_min - (int32_t)last_pio_count_min;
  last_pio_count_min = new_pio_count_min;

  current_encoder_value_min += delta_min;

  // Clamp the value to stay within your defined min/max range
  if (current_encoder_value_min < ENCODER_MIN_VALUE) {
    current_encoder_value_min = ENCODER_MIN_VALUE;
  } else if (current_encoder_value_min > ENCODER_MAX_VALUE) {
    current_encoder_value_min = ENCODER_MAX_VALUE;
  }

  selected_value_min = current_encoder_value_min;

  if (selected_value_min >= selected_value_max - ENCODER_SMALLEST_RANGE)
  {
    selected_value_min = selected_value_max - ENCODER_SMALLEST_RANGE;
  }
  
  printf("MIN Value: %ld\n", current_encoder_value_min);
}


/********************************
  Core 1 loop
*********************************/
void loop1() {
  if (distance < selected_value_min || distance > selected_value_max) {
    // Red LED ON
    digitalWrite(STATUS_GREEN_LED, HIGH);
    digitalWrite(STATUS_RED_LED, LOW);
  }
  else {
    // Green LED ON
    digitalWrite(STATUS_GREEN_LED, LOW);
    digitalWrite(STATUS_RED_LED, HIGH);
  }

}


/********************************
 * Scanns connected I2C devices
 * and prints addresses
 ********************************/
void i2c_scanner() {
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning I2C ...");

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    //delay(1);
    error = Wire.endTransmission();
    //delay(1);
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);
}

/********************************
*********************************/
void init_LED_pins() {
  pinMode(STATUS_GREEN_LED, OUTPUT);
  pinMode(STATUS_RED_LED, OUTPUT);

  digitalWrite(STATUS_RED_LED, LOW);
  digitalWrite(STATUS_GREEN_LED, LOW);
}

/********************************
*********************************/
void init_display() {
   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
   // SSD1306_EXTERNALVCC
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1) {
      Serial.println("Err 1");
      delay(500);
    }
  }
  display.display();
  delay(10);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10,0);
  display.print(avg_distance);
  display.println(F(" mm"));

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10,20);
  display.print(F("min:"));
  display.print(selected_value_min);
  display.print(F("  max:"));
  display.print(selected_value_max);
  display.display();
}

/********************************
 * Init TOF sensor
 * VL53Lxx-v2
 ********************************/
void init_vl53lxx_sensor() {
  Serial.println("Init VL53L0X sensor");

  if (!vl53lxx_sensor.begin()) {
    Serial.println(F("VL53L0X sensor failed!"));

    while (1) {
      Serial.println("Err 2");
      delay(500);
    }
  }
}

